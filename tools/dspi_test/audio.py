"""
audio.py — host-side audio measurement engine for the DSPi loopback rig.

Plays a signal out the DSPi USB audio OUTPUT (host -> DSPi USB input), lets the
DSP process it, and captures DSPi's output slot 0 via DSPi's own USB audio INPUT
(the DSPI_LOOPBACK capture interface), then extracts level / noise / THD and the
filter frequency response.

The capture is integrated into DSPi itself (the DSPI_LOOPBACK firmware build):
output slot 0 is tapped after all DSP and streamed back on a USB capture
endpoint, so no external recorder or S/PDIF wiring is involved. DSPi appears as
one USB device exposing both an OUTPUT (playback) and an INPUT (capture); macOS
presents these as two Core Audio entries that share the name "DSPi".

The chain is single-clock: DSPi is the master, host playback is rate-slaved to
DSPi via the USB Audio feedback endpoint, and the capture is slaved to DSPi's
output rate. So the capture is a fixed-latency, near bit-exact copy of slot 0;
the latency is recovered by cross-correlation, with no resampling. Because the
whole path is digital, measurement SNR is ~24-bit (~140 dB), so a single sweep
gives a very clean transfer function.

This module is the only place that touches PortAudio. It is import-safe even
when sounddevice/numpy are missing: the optional deps are probed at call time and
raise AudioUnavailable so the loopback tests can SKIP cleanly.

Optional deps:  pip install sounddevice numpy scipy   (macOS also: brew install portaudio)

CLI (bring-up / diagnosis):
    python3 -m tools.dspi_test.audio --list      # enumerate host audio devices
    python3 -m tools.dspi_test.audio --probe     # raw loopback tone + level/THD/noise
"""

from __future__ import annotations

import atexit
import os
import pathlib
import pickle
import select
import subprocess
import sys
import time

try:
    import numpy as np
except ImportError:  # pragma: no cover
    np = None

try:
    import sounddevice as sd
except Exception:  # ImportError, or OSError when the PortAudio lib is absent
    sd = None

try:
    from scipy.signal import correlate as _sp_correlate
except Exception:  # scipy optional; fall back to an FFT cross-correlation
    _sp_correlate = None


# Default host-device name substrings (case-insensitive). The DSPI_LOOPBACK
# build exposes capture on the SAME USB device as playback, so both default to
# "DSPi"; direction filtering (output vs input channels) disambiguates the two
# Core Audio entries. Override via the functions' arguments if your OS names
# them differently. (USBRX_IN_NAME kept as a back-compat alias.)
DSPI_OUT_NAME = "DSPi"
DSPI_IN_NAME = "DSPi"
USBRX_IN_NAME = DSPI_IN_NAME

DEFAULT_FS = 48000
PAD_S = 0.10          # leading/trailing silence around an excitation
TAIL_S = 0.40         # extra record time after playback ends

# Stream blocksize. 1536 frames keeps the client inside coreaudiod's IO cycle
# budget, silencing the chronic overloads that precondition the zero-stuffing
# runaway (test_failures_diag.md 8c). Safe as the default ONLY because stream
# IO runs in a disposable worker process: PortAudio's stop path can deadlock
# (rare), and a wedged worker is killed and respawned instead of hanging the
# suite. DSPI_AUDIO_BLOCKSIZE=0 restores the PortAudio-chosen size.
BLOCKSIZE = int(os.environ.get("DSPI_AUDIO_BLOCKSIZE", "1536") or 0)

# Diagnostic tap: play_record() records its most recent run here so the test
# layer can dump raw waveforms (see test_failures_diag.md section 8). Fields:
# exc, cap, fs, attempts (per-attempt host xrun counts), final_bad (True when
# every retry glitched and the returned capture is known-corrupt).
LAST_RUN = None


class AudioUnavailable(Exception):
    """sounddevice/numpy or a required loopback audio device is not present."""


def _require():
    missing = []
    if np is None:
        missing.append("numpy")
    if sd is None:
        missing.append("sounddevice (and PortAudio: brew install portaudio)")
    if missing:
        raise AudioUnavailable(
            "audio loopback needs " + " + ".join(missing)
            + "  (pip install sounddevice numpy scipy)")


# ---------------------------------------------------------------------------
# Device discovery
# ---------------------------------------------------------------------------

def list_devices() -> str:
    """Human-readable enumeration of host audio devices (for --list / errors)."""
    _require()
    lines = []
    for i, d in enumerate(sd.query_devices()):
        io = f"in={d['max_input_channels']} out={d['max_output_channels']}"
        lines.append(f"  [{i:2d}] {d['name']}  ({io}, {int(d['default_samplerate'])} Hz)")
    return "\n".join(lines)


def find_devices(out_name: str = DSPI_OUT_NAME, in_name: str = DSPI_IN_NAME):
    """Locate the DSPi output and the DSPi capture input by name substring.

    Returns (out_index, in_index, info_dict). Raises AudioUnavailable (with the
    available device list) if either is missing, so callers can SKIP.
    """
    _require()
    devs = sd.query_devices()

    def match(name, want_out):
        key = "max_output_channels" if want_out else "max_input_channels"
        return [i for i, d in enumerate(devs)
                if name.lower() in d["name"].lower() and d[key] > 0]

    out_hits = match(out_name, True)
    in_hits = match(in_name, False)
    if not out_hits or not in_hits:
        avail_out = [d["name"] for d in devs if d["max_output_channels"] > 0]
        avail_in = [d["name"] for d in devs if d["max_input_channels"] > 0]
        what = []
        if not out_hits:
            what.append(f"no output matching '{out_name}' (have: {avail_out})")
        if not in_hits:
            what.append(f"no input matching '{in_name}' (have: {avail_in})")
        raise AudioUnavailable("; ".join(what))

    out_i, in_i = out_hits[0], in_hits[0]
    return out_i, in_i, {"out": devs[out_i], "in": devs[in_i]}


# ---------------------------------------------------------------------------
# Signal generation
# ---------------------------------------------------------------------------

def _fade(x, fs, ms=5.0):
    n = int(fs * ms / 1000.0)
    if n > 0 and 2 * n < len(x):
        w = np.hanning(2 * n)
        x = x.copy()
        x[:n] *= w[:n]
        x[-n:] *= w[n:]
    return x


def make_sweep(fs, dur_s=1.0, f1=20.0, f2=None, amp=0.4):
    """Exponential (log) sine sweep f1 -> f2, faded at the edges."""
    _require()
    f2 = f2 if f2 is not None else fs * 0.45
    n = int(dur_s * fs)
    t = np.arange(n) / fs
    L = dur_s / np.log(f2 / f1)
    phase = 2.0 * np.pi * f1 * L * (np.exp(t / L) - 1.0)
    return _fade(amp * np.sin(phase).astype(np.float32), fs)


def make_tone(fs, freq=1000.0, dur_s=0.5, amp=0.4):
    _require()
    n = int(dur_s * fs)
    t = np.arange(n) / fs
    return _fade(amp * np.sin(2.0 * np.pi * freq * t).astype(np.float32), fs)


# ---------------------------------------------------------------------------
# Play + record (two independent streams; single clock domain)
# ---------------------------------------------------------------------------

def _play_record_streams(exc, fs, out_dev, in_dev,
                         in_channels, out_channels, tail_s, max_retries,
                         blocksize):
    """The actual PortAudio stream IO. WORKER PROCESS ONLY: PortAudio's
    CoreAudio stop path can deadlock, so the parent must never open streams
    (see play_record and test_failures_diag.md 8c)."""
    _require()
    n = exc.shape[0]

    # Retry on an xrun: opening/closing two cross-device streams repeatedly can
    # make CoreAudio drop buffers (input overflow / output underflow), which
    # corrupts a capture. `latency="high"` makes that rare; a retry catches the
    # stragglers so a long test run stays reliable.
    last = np.zeros((0, in_channels), dtype=np.float32)
    attempts_log = []
    for _attempt in range(max_retries + 1):
        pos = {"i": 0}
        rec_frames = []
        # in_underflow is the flag PortAudio raises when it delivers SILENCE in
        # place of real input ("input data is all silence (zeros) because no
        # real data is available") — exactly the zero-stuffing signature under
        # investigation, and previously unmonitored (test_failures_diag.md).
        counts = {"in_overflow": 0, "in_underflow": 0,
                  "out_underflow": 0, "out_overflow": 0}

        def _out_cb(outdata, frames, time_info, status):  # noqa: ARG001
            if status.output_underflow:
                counts["out_underflow"] += 1
            if status.output_overflow:
                counts["out_overflow"] += 1
            i0 = pos["i"]
            chunk = exc[i0:i0 + frames]
            m = chunk.shape[0]
            if m:
                outdata[:m] = chunk
            if m < frames:
                outdata[m:] = 0.0       # feed silence once the excitation is done
            pos["i"] = i0 + frames

        def _in_cb(indata, frames, time_info, status):  # noqa: ARG001
            if status.input_overflow:
                counts["in_overflow"] += 1
            if status.input_underflow:
                counts["in_underflow"] += 1
            rec_frames.append(indata.copy())

        instream = sd.InputStream(samplerate=fs, device=in_dev, channels=in_channels,
                                  dtype="float32", latency="high",
                                  blocksize=blocksize, callback=_in_cb)
        outstream = sd.OutputStream(samplerate=fs, device=out_dev, channels=out_channels,
                                    dtype="float32", latency="high",
                                    blocksize=blocksize, callback=_out_cb)
        instream.start()                # capture first so we never miss the onset
        outstream.start()
        sd.sleep(int((n / fs + tail_s) * 1000.0))
        outstream.stop(); instream.stop()
        outstream.close(); instream.close()

        attempts_log.append(dict(counts))
        got = np.concatenate(rec_frames, axis=0) if rec_frames else None
        if got is not None:
            last = got
        if got is not None and not (counts["in_overflow"] or counts["out_underflow"]):
            break
        sd.sleep(120)                   # settle, then retry
    final = attempts_log[-1]
    final_bad = bool(final["in_overflow"] or final["out_underflow"]) or last.shape[0] == 0
    blocks = [f.shape[0] for f in rec_frames]
    return {"cap": last, "attempts": attempts_log, "final_bad": final_bad,
            "in_blocks": blocks[:8] + (["..."] if len(blocks) > 8 else []),
            "in_block_max": max(blocks) if blocks else 0}


# ---------------------------------------------------------------------------
# Capture worker process
# ---------------------------------------------------------------------------
# All stream IO runs in a disposable subprocess (python3 -m ...audio --worker)
# speaking length-prefixed pickles over stdin/stdout. A wedged worker (the
# rare PortAudio stop deadlock) is killed and respawned instead of hanging
# the suite; the interrupted capture is retried on the fresh worker.

_REPO_ROOT = str(pathlib.Path(__file__).resolve().parents[2])
_WORKER = None          # subprocess.Popen, or None
_WORKER_TIMEOUT_SLACK_S = 15.0


class _WorkerWedged(Exception):
    """The worker did not answer in time (presumed native deadlock)."""


def _pipe_send(proc, obj):
    data = pickle.dumps(obj, protocol=pickle.HIGHEST_PROTOCOL)
    proc.stdin.write(len(data).to_bytes(4, "little") + data)
    proc.stdin.flush()


def _pipe_recv(proc, timeout_s):
    """Receive one message, enforcing the deadline across partial reads."""
    deadline = time.monotonic() + timeout_s
    buf = b""
    need = 4
    body = None
    while True:
        remain = deadline - time.monotonic()
        if remain <= 0:
            raise _WorkerWedged(f"no reply within {timeout_s:.0f}s")
        r, _w, _x = select.select([proc.stdout], [], [], min(remain, 1.0))
        if not r:
            if proc.poll() is not None:
                raise _WorkerWedged(f"worker exited (rc={proc.returncode})")
            continue
        chunk = proc.stdout.read1(65536)
        if not chunk:
            raise _WorkerWedged("worker closed its pipe")
        buf += chunk
        while True:
            if body is None and len(buf) >= 4:
                need = int.from_bytes(buf[:4], "little")
                buf = buf[4:]
                body = b""
            if body is not None:
                take = min(need - len(body), len(buf))
                body += buf[:take]
                buf = buf[take:]
                if len(body) == need:
                    return pickle.loads(body)
            if body is None or len(body) < need:
                break


def _kill_worker():
    global _WORKER
    if _WORKER is not None:
        _WORKER.kill()
        try:
            _WORKER.wait(timeout=5)
        except Exception:  # noqa: BLE001 — unkillable child; abandon it
            pass
        _WORKER = None


def _ensure_worker():
    global _WORKER
    if _WORKER is not None and _WORKER.poll() is None:
        return _WORKER
    _WORKER = subprocess.Popen(
        [sys.executable, "-m", "tools.dspi_test.audio", "--worker"],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=None,
        cwd=_REPO_ROOT)
    atexit.register(_kill_worker)
    return _WORKER


def worker_ping(timeout_s=20.0):
    """Round-trip a no-op through the worker (selftest / bring-up)."""
    proc = _ensure_worker()
    _pipe_send(proc, {"op": "ping"})
    return _pipe_recv(proc, timeout_s) == ("ok", "pong")


def _worker_loop():
    """Child main: serve requests until stdin closes. stdout is the protocol
    channel, so nothing in the child may print to it."""
    inp = sys.stdin.buffer
    out = sys.stdout.buffer

    def read_exact(n):
        b = b""
        while len(b) < n:
            chunk = inp.read(n - len(b))
            if not chunk:
                return None
            b += chunk
        return b

    while True:
        hdr = read_exact(4)
        if hdr is None:
            return 0
        body = read_exact(int.from_bytes(hdr, "little"))
        if body is None:
            return 0
        req = pickle.loads(body)
        try:
            if req.get("op") == "ping":
                resp = ("ok", "pong")
            else:
                resp = ("ok", _play_record_streams(**req["args"]))
        except Exception as e:  # noqa: BLE001 — reported to the parent
            resp = ("err", f"{type(e).__name__}: {e}")
        data = pickle.dumps(resp, protocol=pickle.HIGHEST_PROTOCOL)
        out.write(len(data).to_bytes(4, "little") + data)
        out.flush()


def play_record(excitation, fs, out_dev, in_dev,
                in_channels=2, out_channels=2, tail_s=TAIL_S, max_retries=3):
    """Play `excitation` on out_dev while recording in_channels from in_dev.

    `excitation` is mono [N] (duplicated across out_channels) or [N, out_channels].
    Returns the captured float32 array, shape [M, in_channels].

    Uses two explicit callback streams (a recording InputStream + a feeding
    OutputStream) started together, NOT sd.play()+InputStream (which does not
    sync cleanly) and NOT a combined cross-device duplex stream (CoreAudio will
    not open one reliably across two devices). The streams free-run on the shared
    DSPi clock; the caller recovers the fixed latency by cross-correlation.

    The IO itself runs in the capture worker process; a wedged worker is
    killed, respawned, and the capture retried once before giving up.
    """
    _require()
    global LAST_RUN
    exc = np.asarray(excitation, dtype=np.float32)
    if exc.ndim == 1:
        exc = np.column_stack([exc] * out_channels)
    n = exc.shape[0]
    per_attempt_s = n / fs + tail_s + 0.5
    timeout_s = per_attempt_s * (max_retries + 1) + _WORKER_TIMEOUT_SLACK_S
    args = {"exc": exc, "fs": fs, "out_dev": out_dev, "in_dev": in_dev,
            "in_channels": in_channels, "out_channels": out_channels,
            "tail_s": tail_s, "max_retries": max_retries,
            "blocksize": BLOCKSIZE}

    for _spawn_try in range(2):
        proc = _ensure_worker()
        try:
            _pipe_send(proc, {"op": "capture", "args": args})
            status, res = _pipe_recv(proc, timeout_s)
        except (_WorkerWedged, BrokenPipeError, OSError) as e:
            print(f"    ! capture worker wedged ({e}); respawning", flush=True)
            _kill_worker()
            continue
        if status == "err":
            raise AudioUnavailable(f"capture worker: {res}")
        LAST_RUN = {"exc": exc, "fs": fs, **res}
        return res["cap"]
    raise AudioUnavailable(
        "capture worker wedged twice in a row (PortAudio stop deadlock?); "
        "see tools/dspi_test/test_failures_diag.md 8c")


# ---------------------------------------------------------------------------
# Alignment + analysis
# ---------------------------------------------------------------------------

def _xcorr_lag(y, ref):
    """Integer lag (samples) of `ref` within the longer `y`, by cross-correlation,
    plus a normalized peak strength in [0,1] to detect 'no signal'."""
    y = np.asarray(y, np.float64)
    ref = np.asarray(ref, np.float64)
    if _sp_correlate is not None:
        corr = _sp_correlate(y, ref, mode="valid", method="fft")
    else:
        n = len(y) - len(ref) + 1
        corr = np.array([np.dot(y[i:i + len(ref)], ref) for i in range(max(n, 0))])
    if len(corr) == 0:
        return 0, 0.0
    lag = int(np.argmax(np.abs(corr)))
    peak = float(np.abs(corr[lag]))
    # Normalize against ||ref|| * ||y-window|| for a coherence-like strength.
    seg = y[lag:lag + len(ref)]
    denom = np.linalg.norm(ref) * (np.linalg.norm(seg) + 1e-20)
    strength = peak / (denom + 1e-20)
    return lag, strength


def align(captured_ch, reference):
    """Return (aligned_segment, strength). Caller checks `strength` (a weak peak
    means no signal / wrong routing / clock fault)."""
    lag, strength = _xcorr_lag(captured_ch, reference)
    seg = np.asarray(captured_ch, np.float32)[lag:lag + len(reference)]
    if len(seg) < len(reference):  # pad if the tail was clipped
        seg = np.concatenate([seg, np.zeros(len(reference) - len(seg), np.float32)])
    return seg, strength


def dbfs(x):
    """RMS level of x in dBFS (full-scale sine = ~ -3 dBFS RMS)."""
    r = float(np.sqrt(np.mean(np.square(np.asarray(x, np.float64))))) if len(x) else 0.0
    return 20.0 * np.log10(r + 1e-20)


def measure_transfer(out_dev, in_dev, in_channel, fs, freqs,
                     dur_s=1.0, f1=20.0, f2=None, amp=0.4):
    """Play a log sweep, capture `in_channel`, return (mag_db, phase_deg, strength)
    sampled at `freqs`. H = FFT(captured)/FFT(reference) over the aligned window;
    valid because DSPi applies the filter to exactly the played samples.
    """
    _require()
    f2 = f2 if f2 is not None else fs * 0.45
    sweep = make_sweep(fs, dur_s, f1, f2, amp)
    pad = np.zeros(int(PAD_S * fs), np.float32)
    exc = np.concatenate([pad, sweep, pad])

    cap = play_record(exc, fs, out_dev, in_dev)
    if cap.shape[0] == 0:
        raise AudioUnavailable("no audio captured (DSPi capture delivered no frames)")
    y = cap[:, in_channel] if cap.ndim > 1 else cap
    seg, strength = align(y, sweep)

    X = np.fft.rfft(sweep)
    Y = np.fft.rfft(seg)
    fbins = np.fft.rfftfreq(len(sweep), 1.0 / fs)
    H = Y / (X + 1e-12)
    mag = 20.0 * np.log10(np.abs(H) + 1e-20)
    phase = np.unwrap(np.angle(H))

    freqs = np.asarray(freqs, np.float64)
    mag_at = np.interp(freqs, fbins, mag)
    phase_at = np.degrees(np.interp(freqs, fbins, phase))
    return mag_at, phase_at, strength


def measure_complex_2ch(out_dev, in_dev, fs, freqs, dur_s=1.0, f1=20.0, f2=None, amp=0.4):
    """Play one sweep, capture both input channels, and return (H0, H1, strength)
    as complex transfer functions at `freqs`. Both channels are aligned with a
    SHARED lag (recovered from L+R, which is broadband), so their RELATIVE phase
    is valid for summing — e.g. the Linkwitz-Riley LP+HP complementary-sum test.
    A common path delay / polarity is shared by both and cancels in |H0 + H1|.
    """
    _require()
    f2 = f2 if f2 is not None else fs * 0.45
    sweep = make_sweep(fs, dur_s, f1, f2, amp)
    pad = np.zeros(int(PAD_S * fs), np.float32)
    cap = play_record(np.concatenate([pad, sweep, pad]), fs, out_dev, in_dev)
    if cap.shape[0] == 0:
        raise AudioUnavailable("no audio captured (DSPi capture delivered no frames)")
    lag, strength = _xcorr_lag(cap[:, 0].astype(np.float64) + cap[:, 1].astype(np.float64), sweep)
    X = np.fft.rfft(sweep)
    fbins = np.fft.rfftfreq(len(sweep), 1.0 / fs)
    freqs = np.asarray(freqs, np.float64)

    def _h_at(ch):
        seg = cap[lag:lag + len(sweep), ch]
        if len(seg) < len(sweep):
            seg = np.concatenate([seg, np.zeros(len(sweep) - len(seg), np.float32)])
        H = np.fft.rfft(seg) / (X + 1e-12)
        return np.interp(freqs, fbins, H.real) + 1j * np.interp(freqs, fbins, H.imag)

    return _h_at(0), _h_at(1), strength


def measure_interchannel_lag(out_dev, in_dev, fs, dur_s=0.3, amp=0.4):
    """Play a sweep, capture both channels in one go, and return (lag, strength)
    where lag = samples that channel 0 is delayed relative to channel 1 (by
    cross-correlation). Measuring both legs in ONE capture cancels per-capture
    stream-start jitter, so this reliably reads a per-output delay difference.
    """
    _require()
    sweep = make_sweep(fs, dur_s, 30.0, fs * 0.45, amp)
    pad = np.zeros(int(PAD_S * fs), np.float32)
    cap = play_record(np.concatenate([pad, sweep, pad]), fs, out_dev, in_dev)
    if cap.shape[0] == 0:
        raise AudioUnavailable("no audio captured (DSPi capture delivered no frames)")
    a = cap[:, 0].astype(np.float64)
    b = cap[:, 1].astype(np.float64)
    if _sp_correlate is not None:
        corr = _sp_correlate(a, b, mode="full", method="fft")
    else:  # slow fallback
        corr = np.correlate(a, b, mode="full")
    lags = np.arange(-(len(b) - 1), len(a))
    k = int(np.argmax(np.abs(corr)))
    strength = float(np.abs(corr[k]) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-20))
    return int(lags[k]), strength


def measure_tone(out_dev, in_dev, in_channel, fs, freq=1000.0, dur_s=0.5, amp=0.4):
    """Play a sine, capture it, return (level_dbfs, thd_pct, strength)."""
    _require()
    tone = make_tone(fs, freq, dur_s, amp)
    pad = np.zeros(int(PAD_S * fs), np.float32)
    cap = play_record(np.concatenate([pad, tone, pad]), fs, out_dev, in_dev)
    if cap.shape[0] == 0:
        return -200.0, 100.0, 0.0
    y = cap[:, in_channel] if cap.ndim > 1 else cap
    seg, strength = align(y, tone)
    # Trim fades for a clean steady-state THD window.
    g = int(0.01 * fs)
    core = seg[g:len(seg) - g] if len(seg) > 2 * g else seg
    level = dbfs(core)
    thd = _thd_pct(core, fs, freq)
    return level, thd, strength


def measure_tone_2ch(out_dev, in_dev, fs, freq=1000.0, dur_s=0.8, amp=0.4, left_only=False):
    """Play a tone (on the LEFT output only if left_only, else both) and capture
    both input channels. Returns (level0_dbfs, level1_dbfs, thd0_pct) measured on
    the STEADY TAIL of the aligned tone (so a settling effect like the leveller
    is read at steady state, and a delayed bleed like crossfeed is still in-window).
    """
    _require()
    tone = make_tone(fs, freq, dur_s, amp)
    pad = np.zeros(int(PAD_S * fs), np.float32)
    sig = np.concatenate([pad, tone, pad])
    exc = np.column_stack([sig, np.zeros_like(sig) if left_only else sig])
    cap = play_record(exc, fs, out_dev, in_dev)
    if cap.shape[0] == 0:
        return -200.0, -200.0, 100.0
    lag, _ = _xcorr_lag(cap[:, 0], tone)          # ch0 always carries the (left) tone
    lo, hi = int(0.30 * len(tone)), int(0.95 * len(tone))   # steady tail of the tone

    def seg(ch):
        s = cap[lag:lag + len(tone), ch]
        if len(s) < len(tone):
            s = np.concatenate([s, np.zeros(len(tone) - len(s), np.float32)])
        return s[lo:hi]

    s0, s1 = seg(0), seg(1)
    return dbfs(s0), dbfs(s1), _thd_pct(s0, fs, freq)


def measure_tone_bins(out_dev, in_dev, in_channel, fs, freq, probes, dur_s=0.8, amp=0.4):
    """Play a sine at `freq`, capture it, and return {probe_hz: level_dbfs} for
    each probe frequency, read from a Hann-windowed FFT of the steady tail (so
    a divider or envelope-driven stage is measured after it has locked).  The
    scale is calibrated so a full-scale sine reads 0 dBFS at its own bin."""
    _require()
    tone = make_tone(fs, freq, dur_s, amp)
    pad = np.zeros(int(PAD_S * fs), np.float32)
    cap = play_record(np.concatenate([pad, tone, pad]), fs, out_dev, in_dev)
    if cap.shape[0] == 0:
        return {p: -200.0 for p in probes}
    y = cap[:, in_channel] if cap.ndim > 1 else cap
    seg, _strength = align(y, tone)
    lo, hi = int(0.30 * len(seg)), int(0.95 * len(seg))
    core = seg[lo:hi]
    win = np.hanning(len(core))
    X = np.abs(np.fft.rfft(core * win)) * (2.0 / np.sum(win))
    df = fs / len(core)
    out = {}
    for p in probes:
        k = int(round(p / df))
        a, b = max(k - 2, 0), min(k + 3, len(X))
        peak = float(np.max(X[a:b])) if b > a else 0.0
        out[p] = 20.0 * np.log10(max(peak, 1e-12))
    return out


def _thd_pct(x, fs, f0, n_harm=6):
    win = np.hanning(len(x))
    X = np.abs(np.fft.rfft(x * win))
    fbins = np.fft.rfftfreq(len(x), 1.0 / fs)

    def bin_energy(f):
        if f >= fs / 2:
            return 0.0
        k = int(round(f / (fs / len(x))))
        lo, hi = max(k - 2, 0), min(k + 3, len(X))
        return float(np.sum(X[lo:hi] ** 2))

    fund = bin_energy(f0)
    harm = sum(bin_energy(f0 * h) for h in range(2, n_harm + 1))
    if fund <= 0:
        return 100.0
    return 100.0 * np.sqrt(harm / fund)


def measure_noise(out_dev, in_dev, in_channel, fs, dur_s=0.4):
    """Play silence, return the captured noise floor in dBFS."""
    _require()
    cap = play_record(np.zeros(int(dur_s * fs), np.float32), fs, out_dev, in_dev)
    if cap.shape[0] == 0:
        return -200.0
    y = cap[:, in_channel] if cap.ndim > 1 else cap
    return dbfs(y)


def capture_zero_gaps(cap, exc=None, dead=1e-7, live_rms=1e-3, min_run=4, guard=64):
    """Count hard zero gaps inside otherwise-live audio on channel 0.

    This is the coreaudiod input-engine zero-stuffing signature (see
    tools/dspi_test/test_failures_diag.md): runs of exact zeros dropped into
    the middle of healthy signal. A gap only counts when the `guard` samples
    on BOTH sides are live (RMS > live_rms), so filter-attenuated regions,
    silence tests, mutes, and capture edges never trigger it. With `exc`,
    the scan is bounded to the excitation's aligned span: the capture tail
    legitimately alternates silence and stale ring bursts while the loopback
    servo re-primes, and must not read as gaps.
    """
    if np is None or cap is None or len(cap) == 0:
        return 0
    sig = np.asarray(cap[:, 0] if cap.ndim > 1 else cap, np.float64)
    if exc is not None:
        ref = np.asarray(exc[:, 0] if exc.ndim > 1 else exc, np.float64)
        nz = np.flatnonzero(np.abs(ref) > 0)
        if len(nz) == 0:
            return 0                     # silence excitation: nothing to bound
        ref = ref[nz[0]:nz[-1] + 1]
        if len(sig) <= len(ref):
            return 0
        lag, strength = _xcorr_lag(sig, ref)
        if strength < 0.1:
            return 0                     # no recognizable signal to scan inside
        sig = sig[lag:lag + len(ref)]
    y = np.abs(sig)
    is_dead = y < dead
    d = np.diff(is_dead.astype(np.int8))
    starts = np.flatnonzero(d == 1) + 1
    ends = np.flatnonzero(d == -1) + 1
    if is_dead[0]:
        starts = np.r_[0, starts]
    if is_dead[-1]:
        ends = np.r_[ends, len(is_dead)]
    gaps = 0
    for s, e in zip(starts, ends):
        if e - s < min_run or s < guard or e + guard > len(y):
            continue
        pre = np.sqrt(np.mean(y[s - guard:s] ** 2))
        post = np.sqrt(np.mean(y[e:e + guard] ** 2))
        if pre > live_rms and post > live_rms:
            gaps += 1
    return gaps


def bit_exact_residual(out_dev, in_dev, in_channel, fs, dur_s=0.4, amp=0.4):
    """Flat-path fidelity: play a sweep, align the capture, fit the best scalar
    gain, and return (residual_dbfs, scale) where residual is the RMS of
    (captured - scale*reference) relative to full scale. For a clean unity
    digital path, residual sits near the 24-bit floor and scale ~ 1.0.
    """
    _require()
    sweep = make_sweep(fs, dur_s, 30.0, fs * 0.45, amp)
    pad = np.zeros(int(PAD_S * fs), np.float32)
    cap = play_record(np.concatenate([pad, sweep, pad]), fs, out_dev, in_dev)
    if cap.shape[0] == 0:
        return 0.0, 0.0
    y = cap[:, in_channel] if cap.ndim > 1 else cap
    seg, _ = align(y, sweep)
    ref = sweep.astype(np.float64)
    seg = seg.astype(np.float64)
    scale = float(np.dot(ref, seg) / (np.dot(ref, ref) + 1e-20))
    residual = seg - scale * ref
    return dbfs(residual), scale


# ---------------------------------------------------------------------------
# CLI (bring-up / diagnosis)
# ---------------------------------------------------------------------------

def _main(argv=None):
    import argparse
    # Worker mode first: stdout is the pickle protocol channel, so nothing
    # else (argparse help, device listings) may touch it.
    if (argv or sys.argv[1:]) == ["--worker"]:
        return _worker_loop()
    ap = argparse.ArgumentParser(prog="dspi_test.audio",
                                 description="DSPi loopback audio bring-up tool")
    ap.add_argument("--list", action="store_true", help="enumerate host audio devices")
    ap.add_argument("--probe", action="store_true",
                    help="play a 1 kHz tone out DSPi, read it back from DSPi's capture input")
    ap.add_argument("--out-name", default=DSPI_OUT_NAME)
    ap.add_argument("--in-name", default=DSPI_IN_NAME)
    ap.add_argument("--fs", type=int, default=DEFAULT_FS)
    ap.add_argument("--channel", type=int, default=0, help="slot-0 capture channel (0=L,1=R)")
    args = ap.parse_args(argv)

    try:
        _require()
    except AudioUnavailable as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 2

    if args.list or not args.probe:
        print("Host audio devices:")
        print(list_devices())
        if not args.probe:
            return 0

    try:
        out_i, in_i, info = find_devices(args.out_name, args.in_name)
    except AudioUnavailable as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 2
    print(f"\nDSPi output : [{out_i}] {info['out']['name']}")
    print(f"DSPi capture: [{in_i}] {info['in']['name']}")
    print(f"Sample rate : {args.fs} Hz   capture channel: {args.channel}\n")

    print("NOTE: this plays a tone to DSPi but does not configure DSPi routing —")
    print("      route USB -> output slot 0 first, or use the test suite.\n")

    level, thd, strength = measure_tone(out_i, in_i, args.channel, args.fs, 1000.0)
    noise = measure_noise(out_i, in_i, args.channel, args.fs)
    print(f"1 kHz tone : level {level:7.2f} dBFS   THD {thd:6.3f}%   "
          f"corr {strength:4.2f}")
    print(f"noise floor: {noise:7.2f} dBFS")
    if strength < 0.2:
        print("\n  ⚠ weak correlation — no signal reaching the DSPi capture. Check "
              "DSPi input source (USB), output slot 0 enable/routing, and that the "
              "tone is routed to output slot 0.")
    return 0


if __name__ == "__main__":
    sys.exit(_main())
