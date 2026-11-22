"""
Upmixer loopback bench (v2 phase 6, RP2350 DSPI_LOOPBACK build, 48 kHz). Each scenario routes the
generated channels it checks onto the tapped output pair; expected levels are upmixer_spec.md's equations.
"""

import struct
import threading
import time

from ..framework import test, Skip
from .. import audio, coreaudio
from .audio_loopback import _get_rig, _capture, _slot_indices, _upmix_route

try:
    import numpy as np
except ImportError:                       # pragma: no cover
    np = None

A, B, C, SA, SB, BA, BB = range(7)
C_PASSIVE, C_ADAPTIVE, C_OFF = 0, 1, 2
S_OFF, S_PASSIVE, S_ADAPTIVE = 0, 1, 2
AMP = 0.25
LVL = 20.0 * np.log10(AMP) if np is not None else 0.0    # dBFS of one input tone
TOL = 0.3                                                # dB, deterministic paths
SILENT = -40.0                                           # dB below the input tone

_PARAMS = {"enable": "UPMIX_ENABLE", "center": "UPMIX_CENTER_MODE", "surround": "UPMIX_SURROUND_MODE",
           "strength": "UPMIX_STRENGTH", "width": "UPMIX_CENTER_WIDTH", "threshold": "UPMIX_THRESHOLD",
           "presence": "UPMIX_PRESENCE", "sur_delay": "UPMIX_SUR_DELAY", "decorr": "UPMIX_DECORR",
           "src_a": "UPMIX_SOURCE_A", "src_b": "UPMIX_SOURCE_B", "counts": "UPMIX_ACTIVE_COUNTS",
           "back": "UPMIX_BACK_OFFSET"}


def _match_hal_rates(fs):
    """Both DSPi HAL devices at fs: the rig checks only the firmware's rate, so a capture device
    left at another rate after a format switch would make CoreAudio resample one side."""
    for d, _n, _i, _o in coreaudio.find_devices(audio.DSPI_OUT_NAME):
        if coreaudio.get_rate(d) != fs:
            coreaudio.set_rate(d, fs)


class _Bench:
    """Snapshot both profiles on entry and put them back on exit, whatever happens."""

    def __init__(self, dev, profile):
        if profile.platform_id != 1:
            raise Skip("upmixer is RP2350-only")
        if np is None:
            raise Skip("numpy not installed")
        self.dev = dev
        _match_hal_rates(48000)
        self.rig = _get_rig(dev, profile)
        if self.rig["fs"] != 48000:
            raise Skip("bench runs at 48 kHz")
        self.out_l, self.out_r, _cl, _cr = _slot_indices(profile, self.rig["slot"])

    def __enter__(self):
        d = self.dev
        self.saved = {p: (bytes(d.pget("FX.UPMIX_CONFIG", addr=p)), bytes(d.pget("FX.UPMIX_CONFIG_EXT", addr=p)))
                      for p in (0, 1)}
        for p in (0, 1):
            self.set(p, enable=0)
            self.clear(p)
        return self

    def __exit__(self, *exc):
        for p in (0, 1):
            cfg, ext = self.saved[p]
            self.dev.pset("FX.UPMIX_CONFIG_EXT", ext, addr=p)
            self.dev.pset("FX.UPMIX_CONFIG", cfg, addr=p)
        self.dev.wait_ready()
        return False

    def set(self, p=0, **kw):
        for k, v in kw.items():
            self.dev.pset(f"FX.{_PARAMS[k]}", struct.pack("<f", float(v)), addr=p)
        self.dev.wait_ready()

    def clear(self, p=0):
        for g in range(7):
            _upmix_route(self.dev, g, 0, 0, 0, p)

    def route(self, g, outs, trim_db=0.0, invert=False, p=0):
        mask = sum(1 << o for o in outs)
        _upmix_route(self.dev, g, mask, int(round(trim_db * 2)), 1 if invert else 0, p)
        self.dev.wait_ready()

    def status(self):
        return bytes(self.dev.pget("FX.UPMIX_STATUS", length=32))

    def play(self, chans, out_channels=2):
        fs = self.rig["fs"]
        self.dur = len(chans[0]) / fs
        pad = np.zeros(int(audio.PAD_S * fs), np.float32)
        exc = np.column_stack([np.concatenate([pad, c, pad]) for c in chans]).astype(np.float32)
        return _capture(self.dev, lambda: audio.play_record(exc, fs, self.rig["out"], self.rig["in"],
                                                            out_channels=out_channels))


def _tone(f, amp=AMP, dur=1.5, fs=48000):
    t = np.arange(int(dur * fs)) / fs
    x = amp * np.sin(2 * np.pi * f * t)
    n = int(0.005 * fs)
    x[:n] *= np.linspace(0, 1, n)
    x[-n:] *= np.linspace(1, 0, n)
    return x.astype(np.float32)


def _silence(dur=1.5, fs=48000):
    return np.zeros(int(dur * fs), np.float32)


def _steady(cap, fs=48000, lo=0.35, hi=0.85):
    """The settled middle of the excitation, located from the capture's own energy."""
    e = np.sum(np.asarray(cap, np.float64) ** 2, axis=1)
    k = int(0.01 * fs)
    env = np.convolve(e, np.ones(k) / k, "same")
    idx = np.flatnonzero(env > env.max() * 1e-3)
    if len(idx) < fs // 10:
        raise AssertionError("no excitation found in the capture")
    a, b = idx[0], idx[-1]
    return cap[a + int(lo * (b - a)):a + int(hi * (b - a))]


def _fit_db(x, probes, fs=48000):
    """Least-squares amplitude of each known tone, in dBFS: exact for whole or fractional cycles,
    unlike an FFT bin, which scallops by up to 1.4 dB with a Hann window."""
    t = np.arange(len(x)) / fs
    cols = [np.ones_like(t)]
    for f in probes:
        cols += [np.cos(2 * np.pi * f * t), np.sin(2 * np.pi * f * t)]
    coef, *_ = np.linalg.lstsq(np.column_stack(cols), np.asarray(x, np.float64), rcond=None)
    return {f: _db(float(np.hypot(coef[1 + 2 * i], coef[2 + 2 * i]))) for i, f in enumerate(probes)}


def _bins(u, cap, probes, fs=48000):
    """Tone levels per capture channel over a fixed window inside the excitation, so a correctly
    silent output still reads (the capture starts with the stream, latency is well under the pad)."""
    a, b = int((audio.PAD_S + 0.4 * u.dur) * fs), int((audio.PAD_S + 0.8 * u.dur) * fs)
    return [_fit_db(cap[a:b, ch], probes) for ch in (0, 1)]


def _lag(cap, max_ms=40.0):
    """Samples capture channel 0 lags channel 1, and the peak normalized correlation."""
    a, b = cap[:, 0].astype(np.float64), cap[:, 1].astype(np.float64)
    n = 1 << (2 * len(a)).bit_length()
    c = np.fft.irfft(np.fft.rfft(a, n) * np.conj(np.fft.rfft(b, n)), n)
    w = int(max_ms * 48)
    corr, lags = np.concatenate([c[-w:], c[:w + 1]]), np.arange(-w, w + 1)
    k = int(np.argmax(np.abs(corr)))
    return int(lags[k]), float(np.abs(corr[k]) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-20))


def _db(x):
    return 20.0 * np.log10(max(x, 1e-12))


def _check(chk, got, want, what, tol=TOL):
    chk.approx(got, want, tol, f"{what}: {got:.2f} dBFS (want {want:.2f})")


# ---------------------------------------------------------------------------

@test("audio", mutating=True)
def upmix_bench_passive_levels(dev, profile, chk):
    """Passive engines: every generated channel's gain for two uncorrelated tones matches the equations."""
    with _Bench(dev, profile) as u:
        u.set(center=C_PASSIVE, surround=S_PASSIVE, strength=100, width=25, presence=0, decorr=90)
        u.set(enable=1)
        fl, fr = 1000.0, 1500.0
        stim = [_tone(fl), _tone(fr)]
        rem, k = 0.375, 0.7071
        # A' = L - rem (L + R), B' = R - rem (L + R); C = k (L + R); SA = k (A' - B') = k (L - R).
        cases = [((A, B), {0: {fl: 1 - rem, fr: rem}, 1: {fr: 1 - rem, fl: rem}}),
                 ((C, SA), {0: {fl: k, fr: k}, 1: {fl: k, fr: k}}),
                 ((SB, BA), {0: {fl: k, fr: k}, 1: {fl: k, fr: k}})]
        names = "A' B' C SA SB BA BB".split()
        for (gl, gr), want in cases:
            u.clear()
            u.route(gl, [u.out_l])
            u.route(gr, [u.out_r])
            got = _bins(u, u.play(stim), [fl, fr])
            for ch, g in ((0, gl), (1, gr)):
                for f, gain in want[ch].items():
                    _check(chk, got[ch][f], LVL + _db(gain), f"{names[g]} {f:g} Hz")
        chk.note("passive: A'/B' 0.625 and 0.375, C and all four ambience channels 0.7071, within 0.3 dB")


@test("audio", mutating=True)
def upmix_bench_adaptive_behaviour(dev, profile, chk):
    """Adaptive engines: centred content goes to C, panned and antiphase content stays out of it, ambience reaches the sides."""
    with _Bench(dev, profile) as u:
        u.set(center=C_ADAPTIVE, surround=S_ADAPTIVE, strength=100, width=25, threshold=30, presence=0)
        u.set(enable=1)
        f = 1000.0
        u.route(C, [u.out_l])
        u.route(A, [u.out_r])
        got = _bins(u, u.play([_tone(f), _tone(f)]), [f])
        _check(chk, got[0][f], LVL + 3.01, "mono: C", 0.6)
        _check(chk, got[1][f], LVL - 12.04, "mono: A'", 0.6)
        got = _bins(u, u.play([_tone(f), _silence()]), [f])
        chk.ok(got[0][f] < LVL + SILENT, f"left only: C stays out ({got[0][f]:.1f} dBFS)")
        _check(chk, got[1][f], LVL, "left only: A' is L", 0.5)
        got = _bins(u, u.play([_tone(f), -_tone(f)]), [f])
        chk.ok(got[0][f] < LVL + SILENT, f"antiphase: C stays out ({got[0][f]:.1f} dBFS)")
        _check(chk, got[1][f], LVL, "antiphase: A' is L", 0.5)
        # Uncorrelated ambience: C near silent, both sides carry it.
        rng = np.random.default_rng(3)
        nl = (0.1 * rng.standard_normal(72000)).astype(np.float32)
        nr = (0.1 * rng.standard_normal(72000)).astype(np.float32)
        u.clear()
        u.route(C, [u.out_l])
        u.route(SA, [u.out_r])
        core = _steady(u.play([nl, nr]))
        c_db, sa_db = _db(np.std(core[:, 0])), _db(np.std(core[:, 1]))
        in_db = _db(0.1)
        chk.ok(c_db < in_db - 20, f"ambience: C {c_db - in_db:+.1f} dB re input (below -20)")
        chk.ok(sa_db > in_db - 16, f"ambience: SA {sa_db - in_db:+.1f} dB re input (above -16)")
        u.clear()
        u.route(SB, [u.out_l])
        u.route(BA, [u.out_r])
        core = _steady(u.play([nl, nr]))
        sb_db, ba_db = _db(np.std(core[:, 0])), _db(np.std(core[:, 1]))
        chk.ok(sb_db > in_db - 16 and ba_db > in_db - 16,
               f"ambience: SB {sb_db - in_db:+.1f} dB, BA {ba_db - in_db:+.1f} dB re input")
        chk.note(f"adaptive ambience: C {c_db - in_db:+.1f}, SA {sa_db - in_db:+.1f}, SB {sb_db - in_db:+.1f}, "
                 f"BA {ba_db - in_db:+.1f} dB re input")


@test("audio", mutating=True)
def upmix_bench_timing(dev, profile, chk):
    """Zero latency on A', B' and C against the matrix; the sides land Haas plus 10 ms later, the backs back_offset after them."""
    with _Bench(dev, profile) as u:
        fs = 48000
        sweep = audio.make_sweep(fs, 0.6, 30.0, fs * 0.45, 0.4)
        zero = np.zeros_like(sweep)
        u.set(center=C_OFF, surround=S_PASSIVE, sur_delay=12, back=2)
        u.set(enable=1)
        u.route(A, [u.out_l])                       # out_r keeps the plain matrix row
        lag, r = _lag(u.play([sweep, sweep]))
        chk.eq(lag, 0, f"A' against the plain matrix output: lag {lag} samples (r {r:.3f})")
        u.clear()
        u.set(center=C_PASSIVE)
        u.route(C, [u.out_l])
        u.route(A, [u.out_r])
        lag, r = _lag(u.play([sweep, 0.5 * sweep]))
        chk.eq(lag, 0, f"C against A': lag {lag} samples (r {r:.3f})")
        u.clear()
        u.set(center=C_OFF)
        for dly in (12.0, 5.0):
            u.set(sur_delay=dly)
            u.route(SA, [u.out_l])
            u.route(A, [u.out_r])
            lag, r = _lag(u.play([sweep, zero]))
            want = int(round(dly * 48)) + 480       # Haas, then the 10 ms allpass main tap
            chk.approx(lag, want, 2, f"SA after A' at {dly:g} ms delay: {lag} samples (want {want})")
        u.set(sur_delay=12)
        for side, back, name in ((SA, BA, "A"), (SB, BB, "B")):
            u.clear()
            u.route(back, [u.out_l])
            u.route(side, [u.out_r])
            lag, r = _lag(u.play([sweep, zero]))
            chk.approx(lag, 97, 2, f"back after side, {name} side: {lag} samples (want 97, 2.02 ms)")
        chk.note("timing: A', C and the matrix at 0 samples; SA 1056 / 720 samples; backs 97 samples after their sides")


def _hp_peak_db(u, cap, cut=5000.0, fs=48000):
    """Peak of each capture channel above `cut` Hz inside the excitation: a gain step in a sine is
    broadband, a ramp is not.  A fixed window keeps out the tone's fades and the capture tail,
    where the loopback re-primes; filtering the whole capture keeps filter edges out too."""
    from scipy.signal import butter, sosfilt
    sos = butter(8, cut, "highpass", fs=fs, output="sos")
    a, b = int((audio.PAD_S + 0.3) * fs), int((audio.PAD_S + u.dur - 0.3) * fs)
    return [_db(float(np.max(np.abs(sosfilt(sos, cap[:, ch].astype(np.float64))[a:b])))) for ch in (0, 1)]


def _ideal_fade_db(stim, cut=5000.0, fs=48000):
    """HF peak of an ideal 10 ms linear fade (in and out) of the stimulus tones: a ramp's corners
    splatter too, so this is the floor a correct fade path is held to."""
    from scipy.signal import butter, sosfilt
    sos = butter(8, cut, "highpass", fs=fs, output="sos")
    x = np.asarray(stim[0], np.float64) + np.asarray(stim[1], np.float64)
    g = np.ones(len(x))
    m = len(x) // 2
    g[m - 480:m] = np.linspace(1.0, 0.0, 480, endpoint=False)
    g[m:m + 480] = np.linspace(0.0, 1.0, 480, endpoint=False)
    a, b = int(0.3 * fs), len(x) - int(0.3 * fs)
    return _db(float(np.max(np.abs(sosfilt(sos, x * g)[a:b]))))


def _with_events(u, stim, events, t0=1.4, step=1.0):
    """Play stim while `events` run one per second from a helper thread."""
    def run():
        time.sleep(t0)
        for ev in events:
            ev()
            time.sleep(step)
    th = threading.Thread(target=run, daemon=True)
    th.start()
    try:
        return u.play(stim)
    finally:
        th.join()


@test("audio", mutating=True)
def upmix_bench_click_free(dev, profile, chk):
    """Every fade path (enable, routes, trims, polarity, profile change, source edit, back pair start) stays click-free."""
    with _Bench(dev, profile) as u:
        dur = 5.0
        stim = [_tone(997.0, dur=dur), _tone(1499.0, dur=dur)]
        u.set(center=C_PASSIVE, surround=S_PASSIVE, width=25, back=2)
        u.set(1, center=C_PASSIVE, surround=S_PASSIVE, counts=1)
        d = u.dev
        x = lambda g, outs, trim=0.0, inv=False, p=0: (lambda: u.route(g, outs, trim, inv, p))
        sets = lambda p=0, **kw: (lambda: u.set(p, **kw))

        def base():
            u.clear(0)
            u.clear(1)
            u.set(enable=1)
            u.set(1, enable=0)
            u.route(A, [u.out_l])

        quiet = None
        scenarios = [
            ("none", lambda: None, []),
            ("enable", lambda: u.route(C, [u.out_r]), [sets(enable=0), sets(enable=1), sets(enable=0)]),
            ("route", lambda: None, [x(C, [u.out_r]), x(C, []), x(C, [u.out_r])]),
            ("trim", lambda: u.route(C, [u.out_r]), [x(C, [u.out_r], -6.0), x(C, [u.out_r], 6.0),
                                                     x(C, [u.out_r], 0.0)]),
            ("invert", lambda: u.route(C, [u.out_r]), [x(C, [u.out_r], 0.0, True), x(C, [u.out_r]),
                                                       x(C, [u.out_r], 0.0, True)]),
            ("profile", lambda: (u.route(C, [u.out_r]), u.route(A, [u.out_l], p=1), u.route(B, [u.out_r], p=1),
                                 u.set(1, enable=1)),
             [sets(enable=0), sets(enable=1), sets(enable=0)]),
            ("source", lambda: u.route(C, [u.out_r]), [sets(src_b=0), sets(src_b=1), sets(src_b=0)]),
            ("back pair", lambda: None, [x(BA, [u.out_r]), x(BA, []), x(BA, [u.out_r])]),
            ("centre mode", lambda: u.route(C, [u.out_r]), [sets(center=C_ADAPTIVE), sets(center=C_OFF),
                                                            sets(center=C_PASSIVE)]),
        ]
        ideal = _ideal_fade_db(stim)
        worst = {}
        for name, prep, events in scenarios:
            base()
            prep()
            u.dev.wait_ready()
            time.sleep(0.3)
            pk = _hp_peak_db(u, _with_events(u, stim, events))
            worst[name] = max(pk)
            if name == "none":
                quiet = max(pk)
                continue
            chk.ok(max(pk) < max(quiet + 12.0, ideal + 3.0),
                   f"{name}: peak above 5 kHz {max(pk):.1f} dBFS (quiet run {quiet:.1f}, "
                   f"ideal 10 ms fade {ideal:.1f})")
        # Positive control: an instant matrix crosspoint toggle must register as a click.
        base()
        from ..helpers import write_route
        ctrl = _hp_peak_db(u, _with_events(u, stim, [lambda: write_route(d, 1, u.out_r, 0),
                                                  lambda: write_route(d, 1, u.out_r, 1, 0.0)]))
        chk.ok(max(ctrl) > quiet + 20.0, f"control: a hard matrix toggle reads {max(ctrl):.1f} dBFS (the metric bites)")
        write_route(d, 1, u.out_r, 1, 0.0)
        chk.ok(max(worst.values()) < max(ctrl) - 40.0,
               f"every fade path at least 40 dB under the hard toggle ({max(worst.values()):.1f} against {max(ctrl):.1f})")
        chk.note("click peaks above 5 kHz: " + ", ".join(f"{k} {v:.1f}" for k, v in worst.items())
                 + f", ideal 10 ms fade {ideal:.1f}, hard toggle {max(ctrl):.1f} dBFS")


@test("audio", mutating=True)
def upmix_bench_use_cases(dev, profile, chk):
    """Vocal reduction, dialogue lift and centre fill behave as the templates describe."""
    with _Bench(dev, profile) as u:
        f = 1000.0
        mono, left = [_tone(f), _tone(f)], [_tone(f), _silence()]
        u.set(center=C_ADAPTIVE, surround=S_OFF, strength=100, width=0, threshold=30)
        u.set(enable=1)
        u.route(A, [u.out_l])
        u.route(B, [u.out_r])
        got = _bins(u, u.play(mono), [f])
        depth = LVL - max(got[0][f], got[1][f])
        chk.ok(depth > 30, f"vocal reduction: centred tone down {depth:.1f} dB (at least 30)")
        got = _bins(u, u.play(left), [f])
        _check(chk, got[0][f], LVL, "vocal reduction: a side-panned tone passes", 0.5)

        u.set(width=100)
        u.route(C, [u.out_l, u.out_r], -6.0)
        got = _bins(u, u.play(mono), [f])
        want = LVL + _db(1 + 0.7071 * 2 * 10 ** (-6 / 20))
        lift = got[0][f] - LVL
        _check(chk, got[0][f], want, "dialogue lift: centred tone lifted", 0.6)
        got = _bins(u, u.play(left), [f])
        _check(chk, got[0][f], LVL, "dialogue lift: a side-panned tone unchanged", 0.5)

        u.clear()
        u.route(C, [u.out_r])                       # out_l unclaimed: the plain matrix left
        got = _bins(u, u.play(mono), [f])
        _check(chk, got[0][f], LVL, "centre fill: the left output keeps its matrix feed", 0.2)
        _check(chk, got[1][f], LVL + 3.01, "centre fill: the fill output carries C", 0.6)
        chk.note(f"use cases: vocal reduction {depth:.1f} dB, dialogue lift {lift:+.2f} dB, "
                 f"centre fill C {got[1][f] - LVL:+.2f} dB")


def _playback_dev_id():
    hits = [d for d, _n, _i, has_out in coreaudio.find_devices(audio.DSPI_OUT_NAME) if has_out]
    return hits[0] if hits else None


@test("audio", mutating=True)
def upmix_bench_profiles_multichannel(dev, profile, chk):
    """Two profiles: a 6-channel stream runs profile 1 on inputs 5/6, a stereo stream profile 0, never both."""
    with _Bench(dev, profile) as u:
        dev_id = _playback_dev_id()
        if dev_id is None or not any(c == 6 for c, _b, _r in coreaudio.output_formats(dev_id)):
            raise Skip("no 6-channel output format on the DSPi playback device")
        u.set(1, center=C_PASSIVE, surround=S_PASSIVE, width=25, src_a=4, src_b=5, counts=0x04)
        u.route(A, [u.out_l], p=1)
        u.route(B, [u.out_r], p=1)
        u.set(1, enable=1)
        u.set(0, center=C_PASSIVE, surround=S_OFF, counts=0x01)
        u.route(C, [u.out_l], p=0)
        u.set(0, enable=1)
        f0, f1, f4, f5 = 500.0, 700.0, 1000.0, 1300.0
        try:
            if not coreaudio.set_output_channels(dev_id, 6, 48000):
                raise Skip("the HAL would not select the 6-channel format")
            audio._kill_worker()                 # the worker's PortAudio must see the new format
            time.sleep(1.0)
            stim = [_tone(f0), _tone(f1), _silence(), _silence(), _tone(f4), _tone(f5)]
            got = _bins(u, u.play(stim, out_channels=6), [f0, f1, f4, f5])
            st = u.status()
            chk.eq(st[16], 1, "6 channels: profile 1 runs")
            _check(chk, got[0][f0], LVL, "6 ch: input 1 keeps its matrix row on the claimed left output")
            _check(chk, got[0][f4], LVL + _db(0.625), "6 ch: A' from input 5")
            _check(chk, got[0][f5], LVL + _db(0.375), "6 ch: centre removal of input 6 in A'")
            _check(chk, got[1][f5], LVL + _db(0.625), "6 ch: B' from input 6")
            chk.ok(got[0][f1] < LVL + SILENT, f"6 ch: profile 0's centre stays off ({got[0][f1]:.1f} dBFS)")
            chk.note(f"6 ch: A' {got[0][f4] - LVL:+.2f} dB, B' {got[1][f5] - LVL:+.2f} dB, "
                     f"input 1 kept {got[0][f0] - LVL:+.2f} dB")
        finally:
            coreaudio.set_output_channels(dev_id, 2, 48000)
            _match_hal_rates(48000)
            audio._kill_worker()
            time.sleep(1.0)
        got = _bins(u, u.play([_tone(f0), _tone(f1)]), [f0, f1])
        st = u.status()
        chk.eq(st[16], 0, "stereo: profile 0 runs")
        _check(chk, got[0][f0], LVL - 3.01, "stereo: C carries L on the claimed left output")
        _check(chk, got[0][f1], LVL - 3.01, "stereo: C carries R on the claimed left output")
        _check(chk, got[1][f1], LVL, "stereo: the right output keeps its matrix row")
