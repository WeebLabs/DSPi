"""
Spatializer loopback bench (DSPI_LOOPBACK build, 48 kHz, output pair 0). Expected responses are
stereo_spatializer_spec.md's equations, evaluated from the same coefficient formulas the firmware uses.
"""

import math
import struct
import threading
import time

from ..framework import test, Skip
from .. import audio
from ..helpers import levels, pipeline_stats, write_route
from .audio_loopback import _get_rig, _capture, _slot_indices
from .audio_upmix import _match_hal_rates, _tone, _db, _hp_peak_db, _ideal_fade_db, _lag

try:
    import numpy as np
except ImportError:                       # pragma: no cover
    np = None

WIDTH, XTC = 0, 1
FS = 48000
_PARAMS = {"enable": ("SPATIAL_ENABLE", "B"), "mode": ("SPATIAL_MODE", "B"), "pairs": ("SPATIAL_PAIRS", "B"),
           "width": ("SPATIAL_WIDTH", "f"), "bass": ("SPATIAL_BASS_WIDTH", "f"), "shelf": ("SPATIAL_SHELF", "f"),
           "span": ("SPATIAL_SPAN", "f"), "atten": ("SPATIAL_ATTEN", "f"), "mid": ("SPATIAL_MID", "f"),
           "trim": ("SPATIAL_TRIM", "f")}
DEFAULTS = dict(enable=0, mode=WIDTH, pairs=1, width=125.0, bass=150.0, shelf=700.0, span=10.0, atten=3.0,
                mid=100.0, trim=0.0)


class _Bench:
    """Snapshot SPATIAL_CONFIG on entry, start from the defaults with the stage off, restore on exit."""

    def __init__(self, dev, profile):
        if np is None:
            raise Skip("numpy not installed")
        try:
            import scipy.signal  # noqa: F401  (the click bench's high-pass)
        except ImportError:
            raise Skip("scipy not installed")
        self.dev, self.profile = dev, profile
        _match_hal_rates(FS)
        self.rig = _get_rig(dev, profile)
        if self.rig["fs"] != FS:
            raise Skip("bench runs at 48 kHz")
        if self.rig["slot"] != 0:
            raise Skip("the loopback tap is not on output pair 0")
        self.out_l, self.out_r, _cl, _cr = _slot_indices(profile, 0)

    def __enter__(self):
        self.saved = bytes(self.dev.pget("FX.SPATIAL_CONFIG"))
        self.set(**DEFAULTS)
        return self

    def __exit__(self, *exc):
        self.dev.pset("FX.SPATIAL_CONFIG", self.saved)
        self.dev.wait_ready()
        return False

    def set(self, **kw):
        for k, v in kw.items():
            name, fmt = _PARAMS[k]
            self.dev.pset(f"FX.{name}", struct.pack("<" + fmt, int(v) if fmt == "B" else float(v)))
        self.dev.wait_ready()

    def play(self, chans):
        self.dur = len(chans[0]) / FS
        pad = np.zeros(int(audio.PAD_S * FS), np.float32)
        exc = np.column_stack([np.concatenate([pad, c, pad]) for c in chans]).astype(np.float32)
        self.exc = exc
        return _capture(self.dev, lambda: audio.play_record(exc, FS, self.rig["out"], self.rig["in"]))

    def play_events(self, chans, events, t0=2.0, step=1.5):
        """Play while `events` run t0, t0 + step, ... seconds after each capture attempt starts,
        so a retried capture carries them too."""
        self.dur = len(chans[0]) / FS
        pad = np.zeros(int(audio.PAD_S * FS), np.float32)
        exc = np.column_stack([np.concatenate([pad, c, pad]) for c in chans]).astype(np.float32)
        self.exc = exc

        def attempt():
            def run():
                time.sleep(t0)
                for ev in events:
                    ev()
                    time.sleep(step)
            th = threading.Thread(target=run, daemon=True)
            th.start()
            try:
                return audio.play_record(exc, FS, self.rig["out"], self.rig["in"])
            finally:
                th.join()
        return _capture(self.dev, attempt)

    def window(self, cap, lo=0.4, hi=0.85):
        a, b = int((audio.PAD_S + lo * self.dur) * FS), int((audio.PAD_S + hi * self.dur) * FS)
        return cap[a:b]


# --- Model (spec sections 3 and 4, same formulas as spatial_compute_coefficients) -------------

def _one_pole(a, z1):
    return a / (1.0 - (1.0 - a) * z1)


def _width_h(f, width, bass, shelf):
    """Side gain of width mode: 1 + (w - 1) + (wb - w) LP(f)."""
    w, wb = width / 100.0, bass / 100.0
    aw = 1.0 - math.exp(-2.0 * math.pi * shelf / FS)
    z1 = np.exp(-2j * np.pi * np.asarray(f, float) / FS)
    return w + (wb - w) * _one_pole(aw, z1)


def _xtc_loop(f, span, atten):
    """g H(f) of the crosstalk loop: g k, the fractional tap, then LP5 and (1 - LP250)."""
    x = math.exp(-2.0 * math.pi * 5000.0 / FS)
    a5, ah = 1.0 - x, 1.0 - math.exp(-2.0 * math.pi * 250.0 / FS)
    th = math.radians(span)
    d = 0.0875 / 343.0 * (th + math.sin(th)) * FS - x / (1.0 - x)
    d = min(max(d, 1.0), 30.0)
    grid = 20.0 * (0.45 * FS / 20.0) ** (np.arange(128) / 127.0)
    zg = np.exp(-2j * np.pi * grid / FS)
    k = 0.999 / np.max(np.abs(_one_pole(a5, zg) * (1.0 - _one_pole(ah, zg))))
    g = 10.0 ** (-atten / 20.0) * k
    z1 = np.exp(-2j * np.pi * np.asarray(f, float) / FS)
    di, fr = int(d), d - int(d)
    tap = (1.0 - fr) * z1 ** di + fr * z1 ** (di + 1)
    return g * tap * _one_pole(a5, z1) * (1.0 - _one_pole(ah, z1))


def _fit_c(x, probes):
    """Complex amplitude of each known tone by least squares (cos + j sin convention)."""
    t = np.arange(len(x)) / FS
    cols = [np.ones_like(t)]
    for f in probes:
        cols += [np.cos(2 * np.pi * f * t), np.sin(2 * np.pi * f * t)]
    coef, *_ = np.linalg.lstsq(np.column_stack(cols), np.asarray(x, np.float64), rcond=None)
    return np.array([coef[1 + 2 * i] - 1j * coef[2 + 2 * i] for i in range(len(probes))])


def _multitone(probes, amp, dur=1.5, phase_seed=1):
    rng = np.random.default_rng(phase_seed)
    t = np.arange(int(dur * FS)) / FS
    x = sum(amp * np.sin(2 * np.pi * f * t + rng.uniform(0, 2 * np.pi)) for f in probes)
    n = int(0.005 * FS)
    x[:n] *= np.linspace(0, 1, n)
    x[-n:] *= np.linspace(1, 0, n)
    return x.astype(np.float32)


def _gains_db(u, cap, stim_ch, probes):
    """Per capture channel, each tone's level relative to the same tone in the stimulus channel."""
    ref = np.abs(_fit_c(u.window(u.exc[:, stim_ch]), probes))
    out = []
    for ch in (0, 1):
        got = np.abs(_fit_c(u.window(cap[:, ch]), probes))
        out.append(20.0 * np.log10(np.maximum(got, 1e-12) / ref))
    return out


def _report(chk, what, probes, got, want, tol):
    err = np.abs(np.asarray(got) - np.asarray(want))
    worst = int(np.argmax(err))
    chk.ok(err[worst] < tol, f"{what}: worst {probes[worst]:g} Hz {got[worst]:+.2f} dB "
                             f"(model {want[worst]:+.2f}, tol {tol})")
    _WORST.append((float(err[worst]), f"{what} at {probes[worst]:g} Hz"))
    return float(err[worst])


_WORST = []


# ---------------------------------------------------------------------------

WPROBES = [80.0, 250.0, 700.0, 2000.0, 7000.0]
XPROBES = [150.0, 400.0, 900.0, 1700.0, 3100.0, 5300.0, 9100.0, 14000.0]


@test("audio", mutating=True)
def spatial_bench_width_response(dev, profile, chk):
    """Width mode: centred content passes unchanged; panned and antiphase tones follow the shelf model."""
    with _Bench(dev, profile) as u:
        _WORST.clear()
        tone = _multitone(WPROBES, 0.05)
        worst = 0.0
        for width, bass, shelf in ((125.0, 150.0, 700.0), (200.0, 60.0, 300.0), (0.0, 0.0, 700.0)):
            u.set(width=width, bass=bass, shelf=shelf, enable=1)
            hw = _width_h(WPROBES, width, bass, shelf)
            got = _gains_db(u, u.play([tone, tone]), 0, WPROBES)
            for ch in (0, 1):
                worst = max(worst, _report(chk, f"w {width:g}/{bass:g} centred out {ch}", WPROBES, got[ch],
                                           np.zeros(len(WPROBES)), 0.05))
            got = _gains_db(u, u.play([tone, -tone]), 0, WPROBES)
            if width == 0.0 and bass == 0.0:
                chk.ok(float(np.max(got[0])) < -100.0,
                       f"w 0/0 antiphase: the side is removed ({float(np.max(got[0])):.1f} dB at worst)")
                continue                    # the left-only check below would compare two nulls
            worst = max(worst, _report(chk, f"w {width:g}/{bass:g} antiphase", WPROBES, got[0],
                                       20 * np.log10(np.maximum(np.abs(hw), 1e-12)), 0.1))
            got = _gains_db(u, u.play([tone, np.zeros_like(tone)]), 0, WPROBES)
            worst = max(worst, _report(chk, f"w {width:g}/{bass:g} left only, left out", WPROBES, got[0],
                                       20 * np.log10(np.abs(1 + hw) / 2), 0.1))
            worst = max(worst, _report(chk, f"w {width:g}/{bass:g} left only, right out", WPROBES, got[1],
                                       20 * np.log10(np.maximum(np.abs(1 - hw), 1e-9) / 2), 0.2))
        chk.note(f"width: centred exact, panned and antiphase within {worst:.3f} dB of the model "
                 f"(125/150 @700, 200/60 @300, 0/0); worst case {max(_WORST)[1]}")


@test("audio", mutating=True)
def spatial_bench_xtc_response(dev, profile, chk):
    """Crosstalk cancellation: side and mid responses match 1/(1 - gH) and 1/(1 + gm H) at three spans."""
    with _Bench(dev, profile) as u:
        tone = _multitone(XPROBES, 0.02)
        u.set(mode=XTC, enable=1)
        worst, peak = 0.0, -99.0
        for span, atten, mid in ((10.0, 3.0, 100.0), (30.0, 2.5, 50.0), (5.0, 8.0, 0.0)):
            u.set(span=span, atten=atten, mid=mid)
            gh = _xtc_loop(XPROBES, span, atten)
            side = 20 * np.log10(np.abs(1.0 / (1.0 - gh)))
            midr = 20 * np.log10(np.abs(1.0 / (1.0 + (mid / 100.0) * gh)))
            got = _gains_db(u, u.play([tone, -tone]), 0, XPROBES)
            worst = max(worst, _report(chk, f"side, span {span:g} atten {atten:g}", XPROBES, got[0], side, 0.1))
            worst = max(worst, _report(chk, f"side right, span {span:g}", XPROBES, got[1], side, 0.1))
            peak = max(peak, float(np.max(got[0])))
            got = _gains_db(u, u.play([tone, tone]), 0, XPROBES)
            worst = max(worst, _report(chk, f"mid {mid:g} %, span {span:g}", XPROBES, got[0], midr, 0.1))
        chk.note(f"xtc: side and mid within {worst:.3f} dB of the model at 10/30/5 degrees, "
                 f"side peak +{peak:.1f} dB")


def _polarity(u, x):
    """Sign of the loopback path with the stage off (the rig inverts on some builds)."""
    u.set(enable=0)
    cap = u.play([x, x]).astype(np.float64)
    lag, _r = audio._xcorr_lag(cap[:, 0], x)
    seg = cap[lag:lag + len(x), 0]
    return 1.0 if float(np.dot(seg, x)) >= 0 else -1.0


@test("audio", mutating=True)
def spatial_bench_xtc_virtual_listener(dev, profile, chk):
    """Broadband noise through XTC, then through the acoustic crosstalk the canceller assumes, returns the input."""
    with _Bench(dev, profile) as u:
        rng = np.random.default_rng(7)
        n = int(1.5 * FS)
        nl = (0.04 * rng.standard_normal(n)).astype(np.float32)
        nr = (0.04 * rng.standard_normal(n)).astype(np.float32)
        sign = _polarity(u, nl)
        u.set(mode=XTC, span=15.0, atten=3.0, mid=100.0, enable=1)
        cap = sign * u.play([nl, nr]).astype(np.float64)
        # Acoustic model: each ear hears its speaker plus g H of the other one, as an FIR from the
        # loop response, long enough for the 250 Hz pole to die out.
        nf = 1 << 15
        f = np.fft.rfftfreq(nf, 1.0 / FS)
        hgh = np.fft.irfft(_xtc_loop(f, 15.0, 3.0), nf)[:8192]
        ear_l = cap[:, 0] + np.convolve(cap[:, 1], hgh)[:len(cap)]
        ear_r = cap[:, 1] + np.convolve(cap[:, 0], hgh)[:len(cap)]
        res = []
        for ear, ref in ((ear_l, nl), (ear_r, nr)):
            seg, strength = audio.align(ear, ref)
            mid = slice(int(0.2 * FS), int(1.3 * FS))
            e = seg[mid].astype(np.float64) - ref[mid].astype(np.float64)
            res.append(_db(float(np.std(e)) / float(np.std(ref[mid]))))
            chk.ok(strength > 0.99, f"ear signal aligns with its input (r {strength:.5f})")
        chk.ok(max(res) < -50.0, f"virtual listener: residual {max(res):.1f} dB re input (below -50)")
        chk.note(f"virtual listener at 15 degrees: residual L {res[0]:.1f} dB, R {res[1]:.1f} dB re input "
                 f"(loopback polarity {sign:+.0f})")


@test("audio", mutating=True)
def spatial_bench_zero_latency(dev, profile, chk):
    """The processed side path lands on the same sample as the untouched mid path, in width and in XTC."""
    with _Bench(dev, profile) as u:
        rng = np.random.default_rng(11)
        n = int(1.2 * FS)
        a = (0.05 * rng.standard_normal(n)).astype(np.float32)     # mid content
        b = (0.05 * rng.standard_normal(n)).astype(np.float32)     # side content
        notes = []
        for name, kw in (("width", dict(mode=WIDTH, width=200, bass=200, enable=1)),
                         ("xtc", dict(mode=XTC, span=20, atten=3, mid=0, enable=1))):
            u.set(**kw)
            cap = u.play([a + b, a - b]).astype(np.float64)
            m_lag, m_r = audio._xcorr_lag(0.5 * (cap[:, 0] + cap[:, 1]), a)
            s_lag, s_r = audio._xcorr_lag(0.5 * (cap[:, 0] - cap[:, 1]), b)
            chk.eq(s_lag, m_lag, f"{name}: side lag {s_lag} = mid lag {m_lag} (r {s_r:.3f} / {m_r:.3f})")
            chk.ok(m_r > 0.999, f"{name}: mid passes untouched (r {m_r:.5f})")
            notes.append(f"{name} side-mid lag {s_lag - m_lag}")
        chk.note("zero latency: " + ", ".join(notes))


@test("audio", mutating=True)
def spatial_bench_click_free(dev, profile, chk):
    """Live changes under signal (enable, mode, mask, span, mid, atten, width, corner, trim) stay click-free."""
    with _Bench(dev, profile) as u:
        dur = 8.0
        stim = [_tone(997.0, dur=dur), _tone(1499.0, dur=dur)]
        s = lambda **kw: (lambda: u.set(**kw))
        width = dict(mode=WIDTH, enable=1)
        xtc = dict(mode=XTC, enable=1)
        scenarios = [
            ("none", dict(enable=0), []),
            ("enable width", dict(mode=WIDTH), [s(enable=1), s(enable=0), s(enable=1)]),
            ("enable xtc", dict(mode=XTC), [s(enable=1), s(enable=0), s(enable=1)]),
            ("mode", width, [s(mode=XTC), s(mode=WIDTH), s(mode=XTC)]),
            ("mask", xtc, [s(pairs=0), s(pairs=1), s(pairs=0)]),
            ("span", xtc, [s(span=30), s(span=5), s(span=20)]),
            ("mid", xtc, [s(mid=0), s(mid=100), s(mid=40)]),
            ("atten", xtc, [s(atten=2.5), s(atten=12), s(atten=4)]),
            ("width", width, [s(width=150), s(width=100), s(width=175)]),
            ("corner", width, [s(shelf=1500), s(shelf=100), s(shelf=700)]),
            ("trim", width, [s(trim=-3), s(trim=0), s(trim=-1.5)]),
        ]
        ideal = _ideal_fade_db(stim)
        worst, quiet = {}, None
        # Two runs per scenario, keeping the lower peak: a real click repeats, a one-off transport
        # glitch does not.  Both are reported.
        runs = {}
        for name, prep, events in scenarios:
            pks = []
            for _rep in range(2):
                u.set(**DEFAULTS)
                u.set(**prep)
                time.sleep(0.3)
                pks.append(max(_hp_peak_db(u, u.play_events(stim, events))))
            runs[name] = pks
            worst[name] = min(pks)
            if name == "none":
                quiet = min(pks)
                continue
            chk.ok(worst[name] < max(quiet + 12.0, ideal + 3.0),
                   f"{name}: peak above 5 kHz {worst[name]:.1f} dBFS, runs {pks[0]:.1f} / {pks[1]:.1f} "
                   f"(quiet run {quiet:.1f}, ideal 10 ms fade {ideal:.1f})")
        u.set(**DEFAULTS)
        d = u.dev
        ctrl = _hp_peak_db(u, u.play_events(stim, [lambda: write_route(d, 1, u.out_r, 0),
                                                  lambda: write_route(d, 1, u.out_r, 1, 0.0)]))
        write_route(d, 1, u.out_r, 1, 0.0)
        chk.ok(max(ctrl) > quiet + 20.0, f"control: a hard matrix toggle reads {max(ctrl):.1f} dBFS (the metric bites)")
        chk.note("click peaks above 5 kHz (lower of two runs / higher): "
                 + ", ".join(f"{k} {v:.1f}/{max(runs[k]):.1f}" for k, v in worst.items())
                 + f", ideal 10 ms fade {ideal:.1f}, hard toggle {max(ctrl):.1f} dBFS")


@test("audio", mutating=True)
def spatial_bench_cpu(dev, profile, chk):
    """CPU per mode under signal; every pair in crosstalk cancellation runs without ring underruns."""
    with _Bench(dev, profile) as u:
        all_pairs = (1 << profile.num_spdif) - 1
        stim = [_tone(997.0, dur=3.0), _tone(1499.0, dur=3.0)]
        rows = {}
        for name, kw in (("off", dict(enable=0)), ("width", dict(mode=WIDTH, enable=1)),
                         ("xtc", dict(mode=XTC, enable=1)), ("xtc all pairs", dict(mode=XTC, pairs=all_pairs, enable=1))):
            u.set(**kw)
            samples = []

            def poll():
                time.sleep(0.8)
                for _ in range(12):
                    lv = levels(u.dev, profile.num_channels)
                    samples.append((lv["cpu0_load"], lv["cpu1_load"]))
                    time.sleep(0.1)
            before = pipeline_stats(u.dev)["ring_underruns"]
            th = threading.Thread(target=poll, daemon=True)
            th.start()
            u.play(stim)
            th.join()
            under = pipeline_stats(u.dev)["ring_underruns"] - before
            c0 = sum(a for a, _b in samples) / max(len(samples), 1)
            c1 = sum(b for _a, b in samples) / max(len(samples), 1)
            rows[name] = (c0, c1, under)
            chk.eq(under, 0, f"{name}: no ring underruns")
        base = rows["off"]
        chk.ok(rows["xtc all pairs"][0] < 95 and rows["xtc all pairs"][1] < 95, "xtc on every pair: both cores under 95 %")
        chk.note("cpu0/cpu1 %: " + ", ".join(f"{k} {v[0]:.1f}/{v[1]:.1f}" for k, v in rows.items())
                 + f" (xtc adds {rows['xtc'][0] - base[0]:+.1f}/{rows['xtc'][1] - base[1]:+.1f} on one pair)")
