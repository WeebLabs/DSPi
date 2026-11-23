"""selftest.py: no-hardware checks for the audio-loopback harness.

Covers the parts of tests/audio_loopback.py whose correctness does not depend on
a device being attached, against mocks that model the firmware's actual
semantics:

  * Deferred-operation barriers.  A SET arms a pending flag and NO-OPS when the
    request equals the APPLIED state; the main loop applies it later
    (vendor/vc_page_output.c output_type_set, main.c process_type_switches).
    Test 3 deliberately reproduces the pre-barrier bug, so a regression that
    reintroduces fire-and-assume sequencing shows up here rather than as an
    intermittent hardware failure.
  * Matrix isolation.  _route_only() must clear EVERY input row, since
    multichannel USB feeds up to row 7.
  * Per-channel band ceilings, which are probed rather than taken from the
    profile's channel-0 value.

Then the paged transport, paged_sys, the page 0 guard and the native helper layer run against
FakePagedFirmware. A section whose optional module (numpy, sounddevice) is missing reports SKIP.

    python3 -m tools.dspi_test.selftest      # exit 0 = all good
"""
import json
import pathlib
import re
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tools.dspi_test.device import PagedStall, Stall, Timeout
from tools.dspi_test.helpers import BAND_FMT, ROUTE_FMT
from tools.dspi_test.tests import audio_loopback as L


class MockDev:
    """Models the real deferral: SET arms a pending flag and no-ops when the
    request equals the APPLIED array; the main loop applies it `apply_after_s`
    later.  `stall_n` makes the first N GETs raise, mimicking the control-IRQ
    blackout."""

    def __init__(self, applied=0, apply_after_s=0.3, stall_n=0):
        self.applied = applied          # output_types[slot]
        self.pending = None
        self.pending_at = None
        self.apply_after_s = apply_after_s
        self.stall_n = stall_n
        self.sets = 0
        self.gets = 0

    def _tick(self):
        if self.pending is not None and time.monotonic() >= self.pending_at:
            self.applied = self.pending
            self.pending = None

    def pget_u8(self, cmd, addr=0):
        self._tick()
        assert cmd == "OUTPUT.TYPE", cmd
        self.gets += 1
        if self.stall_n > 0:
            self.stall_n -= 1
            raise Stall(0x10, "GET", -9, "control-IRQ blackout")
        return self.applied

    def pset_u8(self, cmd, val, addr=0):
        self._tick()
        assert cmd == "OUTPUT.TYPE", cmd
        self.sets += 1
        if val == self.applied:      # <-- the stale-comparison no-op
            return 1
        self.pending = val
        self.pending_at = time.monotonic() + self.apply_after_s
        return 1


def old_set_output_type(dev, slot, otype):
    """Fire and assume, with no barrier: the sequence the barrier exists to prevent."""
    return dev.pset_u8("OUTPUT.TYPE", otype, slot)


fails = []
skips = []
oks = [0]


def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if cond:
        oks[0] += 1
    else:
        fails.append(msg)


def skip(msg):
    """A section that needs an optional module this host lacks: reported, not failed."""
    print("  SKIP " + msg)
    skips.append(msg)


# Make the settle cheap so the harness test runs fast.
L.PIPELINE_SETTLE_S = 0.01

print("1. barrier: single switch actually lands")
d = MockDev(applied=L.TYPE_SPDIF)
r = L._set_output_type(d, 0, L.TYPE_I2S)
check(r is True, "returns True")
check(d.applied == L.TYPE_I2S, f"device applied I2S (got {d.applied})")

print("2. barrier: SPDIF -> I2S -> SPDIF round trip ends on SPDIF")
d = MockDev(applied=L.TYPE_SPDIF)
L._set_output_type(d, 0, L.TYPE_I2S)
L._set_output_type(d, 0, L.TYPE_SPDIF)
check(d.applied == L.TYPE_SPDIF, f"ended on S/PDIF (got {d.applied})")

print("3. control: the OLD raw sequence loses the restore (bug reproduced)")
d = MockDev(applied=L.TYPE_SPDIF)
old_set_output_type(d, 0, L.TYPE_I2S)
old_set_output_type(d, 0, L.TYPE_SPDIF)   # no-ops against stale applied
time.sleep(0.4)
d._tick()
check(d.applied == L.TYPE_I2S,
      f"old path is stuck in I2S (got {d.applied}) -- confirms the mock models the bug")

print("4. barrier: unchanged request is a genuine no-op (no SET, no settle)")
d = MockDev(applied=L.TYPE_SPDIF)
t0 = time.monotonic()
r = L._set_output_type(d, 0, L.TYPE_SPDIF)
check(r is True and d.sets == 0, f"no SET issued (sets={d.sets})")
check((time.monotonic() - t0) < L.PIPELINE_SETTLE_S, "no settle paid")

print("5. barrier: never-applied switch reports failure, does not hang")
d = MockDev(applied=L.TYPE_SPDIF, apply_after_s=999)
L.BARRIER_TIMEOUT_S = 0.4
t0 = time.monotonic()
r = L._set_output_type(d, 0, L.TYPE_I2S)
el = time.monotonic() - t0
check(r is False, "returns False")
check(0.3 < el < 1.5, f"bounded by the timeout ({el:.2f}s)")

print("6. barrier: tolerates a control-IRQ blackout on the ENTRY read")
d = MockDev(applied=L.TYPE_SPDIF, apply_after_s=0.2, stall_n=3)
L.BARRIER_TIMEOUT_S = 4.0
try:
    r = L._set_output_type(d, 0, L.TYPE_I2S)
except (Stall, Timeout) as e:
    r = None
    check(False, f"stall escaped the helper: {e}")
check(r is True and d.applied == L.TYPE_I2S, "switch still confirmed through stalls")

print("6b. barrier: entry read that NEVER answers reports failure, no exception")
d = MockDev(applied=L.TYPE_SPDIF, stall_n=10**6)
L.BARRIER_TIMEOUT_S = 0.3
try:
    r = L._set_output_type(d, 0, L.TYPE_I2S)
except (Stall, Timeout) as e:
    r = None
    check(False, f"stall escaped the helper: {e}")
check(r is False, f"returns False (got {r!r})")
check(d.sets == 0, f"no SET issued against an unknown state (sets={d.sets})")

print("7. _wait_applied returns last value on timeout, None if all polls raised")
class AllStall:
    def get(self):
        raise Timeout("nope")
L.BARRIER_TIMEOUT_S = 0.2
v = L._wait_applied(AllStall().get, 1, timeout_s=0.2)
check(v is None, f"None when nothing was ever read (got {v!r})")


# --------------------------------------------------------------------------
# Phase 2: deterministic baseline
# --------------------------------------------------------------------------

class MatrixDev:
    """Records matrix crosspoint writes and answers per-channel band probes."""

    def __init__(self, n_in, n_out, band_counts=None):
        self.n_in, self.n_out = n_in, n_out
        self.xp = {}                       # (inp, out) -> enabled
        self.band_counts = band_counts or {}

    def pset(self, cmd, data=b"", addr=0):
        assert cmd == "ROUTE.MATRIX_ROUTE", cmd
        enabled, _phase, _resv, _gain = struct.unpack(ROUTE_FMT, data)
        self.xp[(addr >> 8, addr & 0xFF)] = bool(enabled)
        return len(data)

    def pget(self, cmd, addr=0, length=None):
        assert cmd == "FILTER.BAND", cmd
        ch, band = addr >> 8, addr & 0xFF
        if band >= self.band_counts.get(ch, 10):
            raise PagedStall(0x0610, "GET", addr, 32, "band above ceiling", 0x13)
        return bytes(16)


class Profile:
    def __init__(self, n_in, n_out):
        self.num_input_channels = n_in
        self.num_output_channels = n_out


print("8. _route_only clears EVERY input row, not just the stereo pair")
d = MatrixDev(n_in=8, n_out=11)
L._route_only(d, Profile(8, 11), 0, 1)
on = {k for k, v in d.xp.items() if v}
check(on == {(0, 0), (1, 1)}, f"exactly the target pair is enabled (got {sorted(on)})")
covered = {(i, o) for i in range(8) for o in range(11)}
check(set(d.xp) == covered,
      f"all {len(covered)} crosspoints written (got {len(d.xp)})")
upper_rows = [k for k in d.xp if k[0] >= 2]
check(len(upper_rows) == 6 * 11 and not any(d.xp[k] for k in upper_rows),
      "multichannel rows 2..7 explicitly disabled")

print("9. _route_only on RP2040 geometry (2 inputs)")
d = MatrixDev(n_in=2, n_out=7)
L._route_only(d, Profile(2, 7), 2, 3)
on = {k for k, v in d.xp.items() if v}
check(on == {(0, 2), (1, 3)}, f"target pair only (got {sorted(on)})")

print("10. _band_ceiling probes per channel and caches")
L._BAND_CEILING.clear()
d = MatrixDev(n_in=8, n_out=11, band_counts={8: 10, 9: 6})
check(L._band_ceiling(d, 8) == 10, "channel 8 ceiling 10")
check(L._band_ceiling(d, 9) == 6, "channel 9 ceiling 6 (differs from channel 8)")
d.band_counts[9] = 99                      # would change the answer if re-probed
check(L._band_ceiling(d, 9) == 6, "second call served from cache")
L._BAND_CEILING.clear()

print("11. baseline comparison: float tolerance, int equality, None skipped")
rows = [("a", 0.0, 0.0), ("b", 0, 0), ("c", 0.0, None), ("d", 0.0, 0.5), ("e", 0, 1)]
bad = []
for label, want, got in rows:
    if got is None:
        continue
    ok = abs(got - want) < 1e-3 if isinstance(want, float) else got == want
    if not ok:
        bad.append(label)
check(bad == ["d", "e"], f"only real mismatches reported (got {bad})")


# --------------------------------------------------------------------------
# Phase 3: rate parameterisation
# --------------------------------------------------------------------------

print("12. rate tags: primary is unsuffixed, others are identifier-safe")
check(L._rate_tag(L.RATES[0]) == "", "first rate has no suffix")
check(L._rate_tag(44100) == "_44k1", f"44.1k tag (got {L._rate_tag(44100)!r})")
for fs in (44100, 96000, 32000):
    tag = L._rate_tag(fs)
    check(("x" + tag).isidentifier(), f"{fs} tag {tag!r} is identifier-safe")

print("13. every test body is registered at every rate")
from tools.dspi_test.framework import REGISTRY
# The bench modules (audio_upmix, audio_spatial) run at 48 kHz only and are not replayed.
BENCH = {tc.name for tc in REGISTRY if tc.group == "audio" and tc.fn.__module__ != L.__name__}
audio_all = [tc.name for tc in REGISTRY if tc.group == "audio" and tc.name not in BENCH]
base = [n for n in audio_all if not n.endswith("_44k1") and n != "rate_switch_round_trip"]
var = [n for n in audio_all if n.endswith("_44k1")]
check(len(var) == len(base),
      f"full matrix replayed at 44.1k ({len(var)} variants for {len(base)} tests)")
missing = [n for n in base if n + "_44k1" not in audio_all]
check(not missing, f"no test lacks a 44.1k variant (missing: {missing[:3]})")

print("14. rate variants are contiguous and last, so only ONE rate switch")
audio_names = [tc.name for tc in REGISTRY if tc.group == "audio"]
idx = [i for i, n in enumerate(audio_names) if n.endswith("_44k1")]
check(idx == list(range(min(idx), max(idx) + 1)),
      "44.1k variants form one contiguous block")
check(max(idx) < len(audio_names) - 1 and audio_names[-1] == "rate_switch_round_trip",
      f"round-trip test registered last (last is {audio_names[-1]!r})")

print("15. _rate_variant restores _ACTIVE_FS even when the body raises")
seen = {}


def _body(dev, profile, chk):
    seen["fs"] = L._ACTIVE_FS
    raise RuntimeError("boom")


L._ACTIVE_FS = None
v = L._rate_variant(_body, 44100)
try:
    v(None, None, None)
except RuntimeError:
    pass
check(seen["fs"] == 44100, f"body saw the variant rate (got {seen['fs']})")
check(L._ACTIVE_FS is None, f"_ACTIVE_FS restored after the raise (got {L._ACTIVE_FS})")
# _rate_variant registers as a side effect; drop the throwaway, asserting what
# we remove so a future reorder cannot silently delete a real test.
assert REGISTRY[-1].name == "_body_44k1", REGISTRY[-1].name
REGISTRY.pop()

print("16. _get_rig honours _ACTIVE_FS and keeps per-rate caches apart")
L._RIG.clear(); L._RIG_FAIL.clear()
L._RIG[48000] = {"fs": 48000, "slot": 0, "out": 0, "in": 1}
L._RIG[44100] = {"fs": 44100, "slot": 0, "out": 0, "in": 1}
L._DEVS = (0, 1, {"out": {"name": "m"}, "in": {"name": "m"}})


class RateDev:
    def __init__(self, rate):
        self.rate = rate

    def pget(self, cmd, addr=0, length=None):
        assert cmd == "SYS.SYSTEM_STATUS", cmd
        return struct.pack("<IIHhBBH", 0, self.rate, 0, 0, 0, 0, 0)


if L.np is None:
    skip("_get_rig needs numpy")
else:
    L._ACTIVE_FS = 44100
    check(L._get_rig(RateDev(44100), None)["fs"] == 44100, "picks the 44.1k rig")
    L._ACTIVE_FS = None
    check(L._get_rig(RateDev(48000), None)["fs"] == 48000, "falls back to the primary rig")
    check(L._get_rig(RateDev(44100), None, 44100)["fs"] == 44100, "explicit fs wins")
L._RIG.clear(); L._RIG_FAIL.clear()

print("17. a cached per-rate failure does not poison the other rate")
L._RIG_FAIL[44100] = "nope"
L._RIG[48000] = {"fs": 48000, "slot": 0, "out": 0, "in": 1}
try:
    L._get_rig(RateDev(48000), None, 44100)
    check(False, "should have raised Skip for the failed rate")
except L.Skip as e:
    check(str(e) == "nope", "failed rate skips with its cached reason")
if L.np is None:
    skip("_get_rig needs numpy")
else:
    check(L._get_rig(RateDev(48000), None, 48000)["fs"] == 48000,
          "the healthy rate still works")
L._RIG.clear(); L._RIG_FAIL.clear(); L._DEVS = None; L._ACTIVE_FS = None

print("18. --audio-rates registration modes (end to end, via --list)")
import subprocess

ROOT = Path(__file__).resolve().parents[2]


def _list_audio(*extra):
    out = subprocess.run(
        [sys.executable, "-m", "tools.dspi_test.run", "--list", "--group", "audio", *extra],
        cwd=ROOT, capture_output=True, text=True, check=True).stdout
    names = [ln.strip().split()[0] for ln in out.splitlines()
             if ln.startswith("  ") and ln.strip() and not ln.startswith("  [")]
    return [n for n in names if n not in BENCH]


base = _list_audio()
both = _list_audio("--audio-rates", "48000,44100")
alt = _list_audio("--audio-rates", "44100")

n_primary = len(base) - sum(n.endswith("_44k1") for n in base) - 1   # minus round-trip
check(sum(n.endswith("_44k1") for n in base) == n_primary,
      f"default: FULL matrix at 44.1k too ({sum(n.endswith('_44k1') for n in base)} "
      f"variants for {n_primary} tests)")
check(len(both) == 2 * n_primary + 1,
      f"override lists the FULL matrix twice plus the round trip "
      f"({len(both)} vs {2 * n_primary + 1})")
check(len(alt) == n_primary + 1, f"single-rate override runs one full matrix ({len(alt)})")
check(not any(n.endswith("_44k1") for n in alt),
      "44.1k-as-primary names are unsuffixed")
for label, names in (("default", base), ("override", both)):
    idx = [i for i, n in enumerate(names) if n.endswith("_44k1")]
    check(idx == list(range(min(idx), max(idx) + 1)),
          f"{label}: 44.1k variants contiguous, so exactly one rate change")
    check(names[-1] == "rate_switch_round_trip",
          f"{label}: round trip last, leaving the device on the primary rate")


# --------------------------------------------------------------------------
# Phase 4: capture-glitch counters
# --------------------------------------------------------------------------

class GlitchDev:
    """Serves the two counters; `bump` adds to them on each measurement."""

    def __init__(self, bump=(0, 0), available=True, bump_times=99):
        self.c = [0, 0]
        self.bump, self.available, self.bump_times = bump, available, bump_times
        self.calls = 0

    def pget(self, cmd, addr=0, length=None):
        assert cmd == "DIAG.LOOPBACK_STATS", cmd
        if not self.available:
            raise PagedStall(0xF020, "GET", addr, 32, "release build: no loopback stats", 0x10)
        return struct.pack("<II", *self.c)

    def measure(self):
        self.calls += 1
        if self.calls <= self.bump_times:
            self.c[0] += self.bump[0]
            self.c[1] += self.bump[1]
        return "result"


print("19. clean capture runs once, no retry")
d = GlitchDev(bump=(0, 0))
check(L._capture(d, d.measure) == "result", "returns the measurement")
check(d.calls == 1, f"no needless retry (calls={d.calls})")

print("20. a transient glitch is retried and the clean result returned")
d = GlitchDev(bump=(3, 1), bump_times=1)      # only the first attempt glitches
check(L._capture(d, d.measure) == "result", "returns the retried result")
check(d.calls == 2, f"retried once (calls={d.calls})")

print("21. a persistent glitch Skips instead of asserting on a dead capture")
d = GlitchDev(bump=(5, 2))
try:
    L._capture(d, d.measure)
    check(False, "should have raised Skip")
except L.Skip as e:
    check("underrun" in str(e) and "dropped" in str(e), f"Skip names the counters: {e}")
check(d.calls == 3, f"bounded retries (calls={d.calls})")

print("22. release build (counters absent) degrades to a single plain call")
d = GlitchDev(available=False)
check(L._glitches(d) is None, "counters unavailable reads as None")
check(L._capture(d, d.measure) == "result" and d.calls == 1,
      "runs once and is treated as clean")

print("23. every capture in the module goes through _capture")
src = pathlib.Path("tools/dspi_test/tests/audio_loopback.py").read_text()
bare = []
for m in re.finditer(r"audio\.(measure_\w+|bit_exact_residual)\(", src):
    ls = src.rfind("\n", 0, m.start()) + 1
    if "lambda" not in src[ls:m.start()]:
        bare.append(src[ls:m.end()].strip()[:60])
check(not bare, f"no unwrapped capture calls (found {len(bare)}: {bare[:3]})")

print("24. LT band write is one FILTER.BAND record with Qp in qp_x512")


class EqDev:
    """FILTER.BAND as the firmware stores it: the GET reads back the last applied record."""

    def __init__(self):
        self.payload = self.addr = None

    def pset(self, cmd, data=b"", addr=0):
        assert cmd == "FILTER.BAND", cmd
        self.payload, self.addr = bytes(data), addr
        return len(data)

    def pget(self, cmd, addr=0, length=None):
        assert cmd == "FILTER.BAND" and addr == self.addr, (cmd, addr)
        return self.payload


d = EqDev()
L._set_lt_band(d, 8, 0, 80.0, 0.707, 50.0, 0.707)
check(len(d.payload) == 16, f"16-byte record (got {len(d.payload)})")
ftype, byp, qp, f0, q0, fp = struct.unpack(BAND_FMT, d.payload)
check(ftype == 11, f"FILTER_LINKWITZ_TRANSFORM (got {ftype})")
check(d.addr == (8 << 8) | 0, f"address (channel << 8) | band (got 0x{d.addr:04X})")
check(abs(f0 - 80.0) < 1e-3 and abs(q0 - 0.707) < 1e-3 and abs(fp - 50.0) < 1e-3,
      "f0/Q0/fp carried in the freq/Q/gain fields")
check(qp == round(0.707 * 512), f"Qp = round(Qp*512) = {qp}")

print("25. LT reference matches the documented DC boost (f0/fp)^2")
try:
    from tools.filter_tester.user_linkwitz import _lt_biquad
    import numpy as _np
except ImportError as e:
    _lt_biquad = None
    skip(f"LT reference needs numpy ({e})")
for f0, fp, fs in (((80.0, 50.0, 48000.0), (80.0, 50.0, 44100.0)) if _lt_biquad else ()):
    b, a = _lt_biquad(f0, 0.707, fp, 0.707, fs)
    z = _np.exp(-1j * 2 * _np.pi * 5.0 / fs)          # near DC
    H = (b[0] + b[1] * z + b[2] * z * z) / (a[0] + a[1] * z + a[2] * z * z)
    want = 20 * _np.log10((f0 / fp) ** 2)
    got = 20 * _np.log10(abs(H))
    check(abs(got - want) < 0.5,
          f"{fs:.0f} Hz: DC boost {got:.2f} dB ~ (f0/fp)^2 = {want:.2f} dB")

print("26. LT corners stay inside the firmware's 0.15*Fs clamp at both rates")
for fs in (48000.0, 44100.0):
    check(max(80.0, 50.0) < 0.15 * fs,
          f"{fs:.0f} Hz: f0/fp below the {0.15 * fs:.0f} Hz clamp")

print("27. Phase 5 tests registered, and the replay block still comes last")
audio_names = [tc.name for tc in REGISTRY if tc.group == "audio"]
for n in ("peq_linkwitz_transform", "xo_all_band_slots", "xo_two_band_cascade",
          "loudness_output_mask", "upmix_centre_off_passthrough", "psybass_harmonics",
          "subharm_octave", "subharm_top_band", "subharm_solo", "subharm_ceiling",
          "subharm_selectivity", "subharm_link_pair", "subharm_meter"):
    check(n in audio_names, f"{n} registered")
check(audio_names[-1] == "rate_switch_round_trip",
      f"round trip still last (got {audio_names[-1]!r})")
check(audio_names.index("psybass_harmonics") < audio_names.index("rate_switch_round_trip"),
      "Phase 5 tests precede the per-rate replay, so --audio-rates replays them")

print("28. crossover band slots under test span the firmware's whole range")
check(L.XO_BAND == 20 and L.XO_BANDS == 4, "XOVER_BAND_BASE=20, MAX_XOVER_BANDS=4")


# --------------------------------------------------------------------------
# CoreAudio HAL rate control
# --------------------------------------------------------------------------

print("29. coreaudio helper imports and degrades cleanly")
from tools.dspi_test import coreaudio as ca

check(isinstance(ca.available(), bool), "available() answers without raising")
check(ca._fourcc("nsrt") == 0x6E737274, "FourCC packing matches the HAL constants")
check(ca.kAudioDevicePropertyNominalSampleRate == ca._fourcc("nsrt"),
      "nominal-sample-rate selector")
check(ca.kAudioHardwarePropertyDevices == ca._fourcc("dev#"), "device-list selector")
if not ca.available():
    check(ca.find_devices("anything") == [], "off macOS: find_devices returns empty")
    check(ca.set_rate_by_name("anything", 48000) is False,
          "off macOS: set_rate_by_name reports failure rather than raising")
else:
    check(ca.find_devices("\x00no such device\x00") == [],
          "unknown name matches nothing")
    check(ca.set_rate_by_name("\x00no such device\x00", 48000) is False,
          "unknown name reports failure rather than raising")

print("30. _set_rate uses the HAL, not a stream open")
import inspect
src = inspect.getsource(L._set_rate)
check("coreaudio.set_rate_by_name" in src, "_set_rate calls the HAL setter")
check("measure_tone" not in src,
      "_set_rate no longer relies on opening a stream (CoreAudio would resample)")

print("31. zero-stuffing detector (capture_zero_gaps)")
from tools.dspi_test import audio as A

if A.np is None:
    skip("zero-stuffing detector needs numpy")
else:
    np = A.np
    fs = 48000
    tone = 0.4 * np.sin(2 * np.pi * 1000.0 * np.arange(fs) / fs).astype(np.float32)
    exc = np.concatenate([np.zeros(4800, np.float32), tone, np.zeros(4800, np.float32)])
    lead = np.zeros(6000, np.float32)

    def cap_of(x):
        return np.column_stack([x, x])

    clean = np.concatenate([lead, exc, np.zeros(8000, np.float32)])
    check(A.capture_zero_gaps(cap_of(clean), cap_of(exc)) == 0,
          "clean capture: no gaps")

    stuffed = clean.copy()
    for s in range(6000 + 4800 + 2000, 6000 + 4800 + 40000, 512):
        stuffed[s:s + 40] = 0.0          # the host engine's mid-signal zero bursts
    check(A.capture_zero_gaps(cap_of(stuffed), cap_of(exc)) > 10,
          "zero-stuffed capture: gaps detected")

    check(A.capture_zero_gaps(cap_of(np.zeros(20000, np.float32)),
                              cap_of(np.zeros(19200, np.float32))) == 0,
          "silence test (noise floor measurement): no gaps")

    muted = np.concatenate([lead, np.zeros_like(exc), np.zeros(8000, np.float32)])
    check(A.capture_zero_gaps(cap_of(muted), cap_of(exc)) == 0,
          "muted capture (no signal present): no gaps")

    # Post-excitation servo re-prime chatter must not read as gaps: real
    # bursts alternate with silence in the TAIL, outside the aligned span.
    tail = clean.copy()
    for s in range(6000 + len(exc) + 500, len(tail) - 600, 1024):
        tail[s:s + 400] = 0.11
    check(A.capture_zero_gaps(cap_of(tail), cap_of(exc)) == 0,
          "tail re-prime bursts outside the excitation span: no gaps")

print("32. capture worker isolation")
src_audio = pathlib.Path("tools/dspi_test/audio.py").read_text()
streams_in = [ln for ln in src_audio.splitlines()
              if "sd.InputStream(" in ln or "sd.OutputStream(" in ln]
check(len(streams_in) == 2, f"streams opened in exactly one place ({len(streams_in)} sites)")
body = src_audio[src_audio.index("def _play_record_streams"):
                 src_audio.index("def _worker_loop")]
check("sd.InputStream(" in body and "sd.OutputStream(" in body,
      "both stream opens live inside _play_record_streams (worker-only code)")
if A.np is not None and A.sd is not None:
    check(A.worker_ping(), "worker subprocess round-trips a ping")
    A._kill_worker()
    check(A.worker_ping(), "worker respawns after a kill")
    A._kill_worker()
else:
    skip("worker ping needs sounddevice and numpy")

print("33. subharm audio tests restore state, and never leak monitor solo")
src_loop = pathlib.Path("tools/dspi_test/tests/audio_loopback.py").read_text()
check('"FX.SUBHARM_SOLO", 0' in inspect.getsource(L._subharm_restore),
      "_subharm_restore forces solo off rather than restoring it")
for name in ("subharm_top_band", "subharm_solo", "subharm_ceiling",
             "subharm_selectivity", "subharm_link_pair", "subharm_meter"):
    body = inspect.getsource(getattr(L, name))
    check("finally:" in body and "_subharm_restore(dev, saved)" in body,
          f"{name} restores every subharm field in a finally")
solo_writers = [n for n in ("subharm_top_band", "subharm_solo", "subharm_ceiling",
                            "subharm_selectivity", "subharm_link_pair", "subharm_meter")
                if '"FX.SUBHARM_SOLO", 1' in inspect.getsource(getattr(L, n))]
for name in solo_writers:
    body = inspect.getsource(getattr(L, name))
    tail = body[body.rindex("finally:"):]
    check('"FX.SUBHARM_SOLO", 0' in tail, f"{name} turns solo off in its finally")
check(solo_writers == ["subharm_solo"],
      f"only subharm_solo enables monitor solo (got {solo_writers})")

# --- Paged transport, paged_sys, the page 0 guard and the native helpers on a fake device ---------

import usb.core as _usbcore
from tools.dspi_test import device as D

_VC = D.VC_CODES
_NONE = None


class _Cmd:
    def __init__(self, cid, name, glen=None, srange=None, addrs=_NONE, get=None, set_=None):
        self.cid, self.name, self.glen, self.srange = cid, name, glen, srange
        self.addrs, self.get, self.set = addrs, get, set_


class FakePagedFirmware:
    """A pyusb-shaped device running the paged dispatcher rules of the phase 1 contract over a
    representative command set. Page 0 always STALLs, as after the flip."""

    def __init__(self, paged=True, legacy=False):
        self.paged = paged
        self.legacy = legacy        # page 0 answers like 1.1.6 (0xA5 fill)
        self.page0_log = []
        self.cmds = {}
        self.store = {}
        self.actions = []
        self.page0_hits = 0
        self.paged_hits = 0
        self.last = (0, 0, 0, 0)
        self.corrupt_last = False
        self.cs_busy = False
        self.clips = 0x05
        self._build()

    # -- table ---------------------------------------------------------------
    def _id(self, name):
        return D.resolve_command(name).id

    def add(self, name, glen=None, srange=None, addrs=_NONE, get=None, set_=None):
        cid = self._id(name)
        self.cmds[cid] = _Cmd(cid, name, glen, srange, addrs, get, set_)
        return cid

    def stored(self, name, addr=0, n=None):
        c = self.cmds[self._id(name)]
        return self.store.get((c.cid, addr), bytes(n if n is not None else (c.glen or 0)))

    def put(self, name, data, addr=0):
        self.store[(self._id(name), addr)] = bytes(data)

    def _rw(self, name, n, addrs=_NONE, check=None):
        def s(c, a, d):
            if check:
                r = check(a, d)
                if r:
                    return r
            self.store[(c.cid, a)] = bytes(d)
            return 0
        return self.add(name, n, (n, n), addrs, None, s)

    def _act(self, name, addrs=_NONE, check=None):
        def s(c, a, d):
            if check:
                r = check(a, d)
                if r:
                    return r
            self.actions.append((name, a))
            return 0
        return self.add(name, None, (0, 0), addrs, None, s)

    def _ro(self, name, value, addrs=_NONE):
        return self.add(name, len(value), None, addrs, get=lambda c, a: value)

    def _build(self):
        for n in ("SYS.PROTOCOL_INFO", "SYS.PAGE_LIST", "SYS.PAGE_COMMANDS", "SYS.COMMAND_INFO", "SYS.LAST_RESULT"):
            self.add(n, get=getattr(self, "_" + n.split(".")[1].lower()))
        self.cmds[self._id("SYS.PAGE_LIST")].glen = "var"
        self.cmds[self._id("SYS.PROTOCOL_INFO")].glen = 16
        self.cmds[self._id("SYS.PAGE_COMMANDS")].glen = 64
        self.cmds[self._id("SYS.PAGE_COMMANDS")].addrs = "PAGE"
        self.cmds[self._id("SYS.COMMAND_INFO")].glen = 8
        self.cmds[self._id("SYS.COMMAND_INFO")].addrs = "PAGE_CMD"
        self.cmds[self._id("SYS.LAST_RESULT")].glen = 8
        self._ro("SYS.PLATFORM", bytes([1, 1, 0x16, 9, 1, 6, 0]))
        self._ro("SYS.SERIAL", b"0123456789ABCDEF")
        self._ro("SYS.BUILD_INFO", b"v1.1.7-fake".ljust(48, b"\0") + b"2026-11-16".ljust(16, b"\0"))
        self._ro("SYS.CORE1_MODE", b"\x02")
        self._ro("SYS.SYSTEM_STATUS", struct.pack("<IIHhBBH", 307200000, 48000, 1100, -512, 37, 12, 0))
        self._ro("SYS.USB_AUDIO_STATUS", struct.pack("<IIIBBH", 1000, 3, 1250, 1, 1, 0))
        self._ro("SYS.PDM_STATS", struct.pack("<IIII", 11, 12, 13, 14))
        self._ro("SYS.SPDIF_TX_STATS", struct.pack("<IIIIII", 21, 22, 23, 24, 25, 26))
        self._ro("SYS.PIPELINE_STATS", struct.pack("<IIIIII", 31, 32, 33, 34, 35, 0))
        self._rw("SYS.STATS_RESET", 1, check=lambda a, d: _VC["VC_ERR_VALUE"] if d[0] & 0xFE else 0)
        self.cmds[self._id("SYS.STATS_RESET")].glen = None   # SET only
        self.buffer_stats_seq = 0

        def buffer_stats(c, a):
            # The firmware bumps the sequence (bytes 2-3) on every read.
            self.buffer_stats_seq = (self.buffer_stats_seq + 1) & 0xFFFF
            return bytes([4, 0]) + struct.pack("<H", self.buffer_stats_seq) + bytes(40)
        self.add("SYS.BUFFER_STATS", 44, None, _NONE, get=buffer_stats)

        def device_name(c, a, d):
            self.store[(c.cid, a)] = (bytes(d[:31]) if d[0] else b"").ljust(32, b"\0")
            return 0
        self.add("SYS.DEVICE_NAME", 32, (1, 32), _NONE, None, device_name)
        self.add("STORE.ALL_PARAMS", 6284, (1, 6284), _NONE, None,
                 lambda c, a, d: self.store.__setitem__((c.cid, a), bytes(d)) or 0)
        self._act("STORE.PRESET_SAVE", range(10))
        self._ro("STORE.PRESET_ACTIVE", b"\x02")
        self._ro("STORE.DIRECTORY", bytes([0x05, 0, 1, 3, 2, 1, 0]))
        self._rw("STORE.STARTUP", 2)
        self.put("STORE.STARTUP", b"\x01\x03")
        self._act("STORE.OUTPUT_CONFIG_SAVE")
        self._ro("INPUT.STATUS", struct.pack("<BBHIi", 1, 2, 0, 47999123, -250))
        self._rw("INPUT.SPDIF_RX_PIN", 1, range(4))

        # The input block commands are unaddressed; values outside their range are VC_ERR_VALUE.
        self._rw("INPUT.SPDIF_RX_ENABLE", 1, check=lambda a, d: _VC["VC_ERR_VALUE"] if d[0] > 1 else 0)
        self._rw("INPUT.SPDIF_RX_COUNT", 1, check=lambda a, d: 0 if 1 <= d[0] <= 4 else _VC["VC_ERR_VALUE"])
        self._rw("INPUT.I2S_RX_ENABLE", 1, check=lambda a, d: _VC["VC_ERR_VALUE"] if d[0] > 1 else 0)
        self.put("INPUT.SPDIF_RX_ENABLE", b"\x01")
        self.put("INPUT.SPDIF_RX_COUNT", b"\x02")
        self._ro("INPUT.SPDIF_RX_CONFIG", bytes([4, 1, 2, 5, 20, 21, 22]))
        self._rw("INPUT.I2S_RX_RATE", 4)
        self.put("INPUT.I2S_RX_RATE", struct.pack("<I", 96000))

        def out_pin(a, d):
            if d[0] in (23, 24, 25):
                return _VC["VC_ERR_PIN_INVALID"]
            if any(self.stored("OUTPUT.PIN", o, 1)[0] == d[0] for o in range(5) if o != a):
                return _VC["VC_ERR_PIN_IN_USE"]
            return 0
        self._rw("OUTPUT.PIN", 1, range(5), out_pin)
        for o in range(5):
            self.put("OUTPUT.PIN", bytes([6 + o]), o)
        self._rw("ROUTE.OUTPUT_GAIN", 4, range(9))
        self._rw("ROUTE.MATRIX_ROUTE", 8, "IN_OUT")
        self._rw("ROUTE.PREAMP", 4, range(8))

        def band(a, d):
            return _VC["VC_ERR_ADDRESS"] if (a & 0xFF) not in list(range(10)) + [20, 21, 22, 23] else 0
        self._rw("FILTER.BAND", 16, "CH_BAND", band)
        self.cmds[self._id("FILTER.BAND")].get = (
            lambda c, a: None if band(a, b"") else self.store.get((c.cid, a), bytes(16)))
        for n in ("ENABLE", "THRESHOLD", "RELEASE", "LINK_GROUP"):
            self._rw("DYN.LIMITER_" + n, 4, "OUTPUT_ALL")
        self._ro("DYN.LIMITER_STATUS", bytes([0, 32, 16, 9]))
        self._rw("FX.TUBE_DRIVE", 4)
        self._ro("MEAS.LEVELS", b"".join(struct.pack("<H", 100 * (ch + 1)) for ch in range(17))
                 + bytes([41, 7]) + struct.pack("<I", 0x05) + b"\x02")

        def clip_get(c, a):
            return struct.pack("<I", self.clips)
        self.add("MEAS.CLIP_FLAGS", 4, None, _NONE, clip_get)

        def clip_clear(c, a, d):
            self.clips = 0
            return 0
        self.add("MEAS.CLIP_CLEAR", None, (0, 0), _NONE, None, clip_clear)
        for n in ("RTA_START", "RTA_STOP", "RTA_RESET"):
            self._act("MEAS." + n)
        self._act("CS.SAVE", check=lambda a, d: _VC["VC_ERR_BUSY"] if self.cs_busy else 0)
        self._ro("CS.CAPS", bytes(range(52)))
        self._ro("CS.NOUN_DESC", bytes(12), range(81))
        self._ro("CS.DISPLAY_CAPS", bytes([16, 9, 0, 0]))
        self._rw("CS.DISPLAY_CONFIG", 12)
        self.put("CS.DISPLAY_CONFIG", bytes(range(1, 13)))
        self._rw("CLOCK.MCK_MULTIPLIER", 1, _NONE, lambda a, d: _VC["VC_ERR_VALUE"] if d[0] > 1 else 0)

    # -- SYS discovery handlers -------------------------------------------------
    def _pages(self):
        return sorted({cid >> 8 for cid in self.cmds})

    def _protocol_info(self, c, a):
        return b"DSPi" + struct.pack("<BBBBHHI", 1, 0, len(self._pages()), 0, 64, 256, 0)

    def _page_list(self, c, a):
        return b"".join(bytes([p, 1]) for p in self._pages())

    def _page_commands(self, c, a):
        if a not in self._pages():
            return None
        g, s = bytearray(32), bytearray(32)
        for cid, cmd in self.cmds.items():
            if cid >> 8 == a:
                n = cid & 0xFF
                if cmd.get or cmd.glen:
                    g[n // 8] |= 1 << (n % 8)
                if cmd.set:
                    s[n // 8] |= 1 << (n % 8)
        return bytes(g + s)

    def _command_info(self, c, a):
        cmd = self.cmds.get(a)
        if cmd is None:
            return None
        has_get = bool(cmd.get or cmd.glen)
        gl = 0xFFFF if cmd.glen == "var" else (cmd.glen or 0) if has_get else 0
        smin, smax = cmd.srange or (0, 0)
        kind = self.KINDS.index(self._kind(cmd))
        return struct.pack("<HHBBBB", 0, gl, smin, smax, kind, (1 if has_get else 0) | (2 if cmd.set else 0))

    def _last_result(self, c, a):
        cid, d, res, wv = self.last
        if self.corrupt_last:
            cid ^= 0x0101
        return struct.pack("<HBBHH", cid, d, res, wv, 0)

    KINDS = ["NONE", "INDEX", "PAGE", "PAGE_CMD", "IN_OUT", "CH_BAND", "OUTPUT_ALL"]

    @staticmethod
    def _kind(c):
        return "NONE" if c.addrs is _NONE else c.addrs if isinstance(c.addrs, str) else "INDEX"

    def registry(self):
        """dspi_commands.json with its SYS page replaced by this fake's, so names still resolve."""
        cmds = []
        for cid, c in sorted(self.cmds.items()):
            if cid >> 8 != 1:
                continue
            has_get = bool(c.get or c.glen)
            kind = self._kind(c)
            cmds.append({"cmd": cid & 0xFF, "name": c.name.split(".")[1], "id": cid, "get": has_get,
                         "set": bool(c.set), "addr": kind, "get_len": c.glen if has_get else None,
                         "set_min": (c.srange or (0, 0))[0], "set_max": (c.srange or (0, 0))[1],
                         "flags": [], "arg": "0", "build": None, "doc": ""})
        real = json.loads((ROOT / D.REGISTRY_REL).read_text())
        sys_pg = {"page": 1, "mnemonic": "SYS", "revision": 1, "doc": "", "commands": cmds, "reserved": []}
        return {"protocol": {"major": 1, "minor": 0},
                "pages": [sys_pg] + [p for p in real["pages"] if p["page"] != 1]}

    # -- pyusb surface ----------------------------------------------------------
    def _addr_ok(self, cmd, a, is_set):
        k = cmd.addrs
        if k is _NONE:
            return a == 0
        if isinstance(k, range):
            return a in k
        if k == "OUTPUT_ALL":
            return a < 9 or (is_set and a == 0xFF)
        return True

    def ctrl_transfer(self, bm, req, wvalue, windex, data_or_len, timeout=None):
        page = windex >> 8
        is_get = bool(bm & 0x80)
        if page == 0:
            self.page0_hits += 1
            self.page0_log.append(("IN" if is_get else "OUT", req, wvalue))
            if self.legacy:
                return b"\xa5" * data_or_len if is_get else len(data_or_len)
            if is_get and req == 0x01 and windex == 7:
                return (b"\x0a\x00\x00\x00" + b"WINUSB\0\0" + bytes(64))[:data_or_len]
            self.last = (req, 1 if is_get else 0, _VC["VC_ERR_UNKNOWN"], wvalue)
            raise _usbcore.USBError("Pipe error", errno=32)
        self.paged_hits += 1
        if not self.paged:
            raise _usbcore.USBError("Pipe error", errno=32)
        cid = (page << 8) | req
        cmd = self.cmds.get(cid)
        res, reply = 0, None
        if cmd is None:
            res = _VC["VC_ERR_UNKNOWN"]
        elif is_get and not (cmd.get or cmd.glen):
            res = _VC["VC_ERR_DIRECTION"]
        elif not is_get and not cmd.set:
            res = _VC["VC_ERR_DIRECTION"]
        elif not is_get and not (cmd.srange[0] <= len(data_or_len) <= cmd.srange[1]):
            res = _VC["VC_ERR_LENGTH"]
        elif not self._addr_ok(cmd, wvalue, not is_get):
            res = _VC["VC_ERR_ADDRESS"]
        elif is_get:
            reply = cmd.get(cmd, wvalue) if cmd.get else self.store.get((cid, wvalue), bytes(cmd.glen))
            if reply is None:
                res = _VC["VC_ERR_ADDRESS"]
        else:
            res = cmd.set(cmd, wvalue, bytes(data_or_len))
        if cid != D.SYS_LAST_RESULT:
            self.last = (cid, 1 if is_get else 0, res, wvalue)
        if res:
            raise _usbcore.USBError("Pipe error", errno=32)
        return bytes(reply)[:data_or_len] if is_get else len(data_or_len)


def _paged_dev(**kw):
    fw = FakePagedFirmware(**kw)
    return fw, D.DspiDevice(usb_dev=fw, registry=fw.registry())


print("P1. paged transport: addressing, LAST_RESULT, PROTOCOL_INFO and the old-firmware guard")
fw, dv = _paged_dev()
check(D.paged_windex(0x06) == 0x0602, "page rides in the wIndex high byte, interface 2 low")
pi = dv.protocol_info()
check(pi is not None and pi["magic"] == "DSPi" and pi["max_set"] == 64, f"protocol_info parsed: {pi}")
check(dv.pget("SYS.PLATFORM")[:2] == b"\x01\x01", "pget by name uses the registry length")
check(dv.pget(0x0120, length=4) == bytes([1, 1, 0x16, 9]), "pget by numeric ID, truncated")
try:
    dv.pset("SYS.PLATFORM")
    check(False, "SET to a GET-only command must STALL")
except D.PagedStall as s:
    check(s.result_name == "VC_ERR_DIRECTION" and isinstance(s, Stall), f"typed stall carries the code ({s})")
fw.corrupt_last = True
try:
    dv.pget("ROUTE.OUTPUT_GAIN", addr=50)
except D.PagedStall as s:
    check(s.result is None, "a LAST_RESULT for another request is not trusted")
fw.corrupt_last = False
lr = dv.last_result()
check(lr is not None and lr.cmd_id == D.resolve_command("ROUTE.OUTPUT_GAIN").id and lr.wvalue == 50,
      f"last_result decodes ID, direction and wValue ({lr})")
old_fw, old_dv = _paged_dev(paged=False)
check(old_dv.protocol_info() is None, "protocol_info is None where PROTOCOL_INFO STALLs")
for what, fn in (("GET", lambda: old_dv.pget("ROUTE.OUTPUT_GAIN")),
                 ("SET", lambda: old_dv.pset("ROUTE.OUTPUT_GAIN", bytes(4)))):
    try:
        fn()
        check(False, f"a paged {what} must be refused on firmware without the dispatcher")
    except D.ProtocolError as e:
        check(old_fw.paged_hits == 1 and "1.1.7" in str(e),
              f"{what}: only the probe reached the old firmware, and the error names 1.1.7 ({old_fw.paged_hits})")
fw2 = FakePagedFirmware()
D.DspiDevice(usb_dev=fw2)
check(fw2.paged_hits == 0 and fw2.page0_hits == 0, "opening a device sends nothing until the first request")
dv2 = D.DspiDevice(usb_dev=fw2, registry=fw2.registry())
check(dv2.is_alive() and dv2.wait_ready(timeout_s=0.5) and fw2.page0_hits == 0,
      "is_alive / wait_ready answer over SYS.PLATFORM, nothing on page 0")
check(not D.DspiDevice(usb_dev=FakePagedFirmware(paged=False)).is_alive(), "is_alive is False without the dispatcher")

print("P2. paged_sys against the fake")
fw, dv = _paged_dev()
from tools.dspi_test.framework import Check, Skip as _Skip
from tools.dspi_test.tests import paged_sys as PS
for name in [n for n in dir(PS) if callable(getattr(PS, n)) and getattr(getattr(PS, n), "__module__", "")
             == PS.__name__ and not n.startswith("_")]:
    fn = getattr(PS, name)
    if fn.__code__.co_argcount != 3:
        continue
    c = Check()
    try:
        fn(dv, None, c)
        check(not c.failures, f"paged_sys.{name} passes on the fake ({c.failures[:2]})")
    except _Skip as s:
        check(False, f"paged_sys.{name} skipped on the fake: {s}")
fw2, dv2 = _paged_dev(paged=False)
c = Check()
try:
    PS.protocol_info_fields(dv2, None, c)
    check(False, "paged_sys must SKIP on firmware without the dispatcher")
except _Skip:
    check(True, "paged_sys SKIPs on firmware without the dispatcher")

print("P3. the page 0 walk guard")
from tools.dspi_test.tests import paged_page0 as PG
v117 = bytes([1, 1, 0x17, 9, 1, 7, 1])
for label, kw, plat, hits in (("1.1.6 firmware", dict(paged=False, legacy=True), None, 0),
                              ("paged build that still answers page 0", dict(legacy=True), v117, 1),
                              ("paged firmware reporting 1.1.6", {}, None, 0)):
    fw = FakePagedFirmware(**kw)
    if plat:
        fw._ro("SYS.PLATFORM", plat)
    dv = D.DspiDevice(usb_dev=fw, registry=fw.registry())
    try:
        PG.page0_walk_all_stall(dv, None, Check())
        check(False, f"page 0 walk must refuse {label}")
    except _Skip:
        check(fw.page0_hits == hits, f"page 0 walk refuses {label} ({fw.page0_hits} page 0 requests sent)")
fw = FakePagedFirmware()
fw._ro("SYS.PLATFORM", v117)
dv = D.DspiDevice(usb_dev=fw, registry=fw.registry())
for fn in (PG.ms_os20_descriptor_served, PG.page0_walk_all_stall):
    c = Check()
    fn(dv, None, c)
    check(not c.failures, f"paged_page0.{fn.__name__} passes on the flipped fake ({c.failures[:2]})")
check(fw.page0_hits == len(PG._plan()) + 2, f"walk sent every planned request ({fw.page0_hits})")
check(len(PG._plan()) == 2304 and all(r[2] != 7 for r in PG._plan()), "2,304 requests, MS OS 2.0 not among them")

print("P4. native helper layer: helpers, telemetry, lifecycle and actions on the paged fake")
from types import SimpleNamespace as _NS
from tools.dspi_test import actions as ACT, helpers as H, lifecycle as LC, telemetry as TM


def _busy_first(fw, name, n):
    """The next n SETs of `name` answer VC_ERR_BUSY, then the original handler runs."""
    c = fw.cmds[fw._id(name)]
    orig, left = c.set, [n]

    def s(cmd, a, d):
        if left[0]:
            left[0] -= 1
            return _VC["VC_ERR_BUSY"]
        return orig(cmd, a, d)
    c.set = s
    return left


fw, dv = _paged_dev()
c = Check()
H.float_roundtrip(dv, c, "ROUTE.OUTPUT_GAIN", -6.5, addr=3)
H.float_clamp(dv, c, "ROUTE.OUTPUT_GAIN", 2.0, 2.0, addr=3)
check(not c.failures and fw.stored("ROUTE.OUTPUT_GAIN", 3) == struct.pack("<f", 2.0),
      f"native float_roundtrip / float_clamp by PAGE.NAME and addr ({c.failures[:1]})")
check(H.try_set(dv, "ROUTE.OUTPUT_GAIN", struct.pack("<f", 1.0), 40) == "VC_ERR_ADDRESS"
      and H.try_set(dv, "ROUTE.OUTPUT_GAIN", struct.pack("<f", 1.0), 1) == "VC_OK", "try_set names the result")
c = Check()
H.expect_stall(c, lambda: dv.pset("ROUTE.OUTPUT_GAIN", bytes(4), 40), "VC_ERR_ADDRESS", "bad output")
H.expect_result(c, lambda: dv.pset("ROUTE.OUTPUT_GAIN", bytes(4), 1), "VC_OK", "good output")
H.expect_result(c, lambda: dv.pset("ROUTE.OUTPUT_GAIN", bytes(3), 1), "VC_ERR_LENGTH", "short payload")
check(not c.failures, f"expect_stall / expect_result accept the right outcomes ({c.failures[:1]})")
c = Check()
H.expect_stall(c, lambda: dv.pset("ROUTE.OUTPUT_GAIN", bytes(4), 1), "VC_ERR_ADDRESS", "accepted")
H.expect_result(c, lambda: dv.pset("ROUTE.OUTPUT_GAIN", bytes(4), 40), "VC_OK", "refused")
H.expect_stall(c, lambda: dv.pset("ROUTE.OUTPUT_GAIN", bytes(4), 40), "VC_ERR_VALUE", "wrong code")
check(len(c.failures) == 3, f"and fail on the wrong ones ({len(c.failures)} of 3)")
c = Check()
H.wrong_length_refused(dv, c, "ROUTE.OUTPUT_GAIN", addr=2)
check(not c.failures, f"wrong_length_refused: 3- and 5-byte SETs are VC_ERR_LENGTH ({c.failures[:1]})")

c = Check()
H.nan_rejected(dv, c, "ROUTE.OUTPUT_GAIN", addr=5, label="fake gain")
check(c.failures, "nan_rejected flags a command that stores NaN")
fw._rw("ROUTE.PREAMP", 4, range(8), lambda a, d: _VC["VC_ERR_VALUE"] if struct.unpack("<f", d)[0] != struct.unpack("<f", d)[0] else 0)
c = Check()
H.nan_rejected(dv, c, "ROUTE.PREAMP", addr=1, label="preamp")
check(not c.failures, f"nan_rejected passes a VC_ERR_VALUE refusal ({c.failures[:1]})")
fw.add("ROUTE.USER_MUTE", 1, (1, 1), _NONE, None,
       lambda cm, a, d: fw.store.__setitem__((cm.cid, a), bytes([1 if d[0] else 0])) or 0)
c = Check()
H.bool_roundtrip(dv, c, "ROUTE.USER_MUTE", label="mute")
check(not c.failures, f"native bool_roundtrip ({c.failures[:1]})")

left = _busy_first(fw, "FILTER.BAND", 2)
sent = H.write_band(dv, 8, 3, 11, 40.0, 0.6, 30.0, qp_x512=700, wait=True)
check(left[0] == 0 and H.read_band(dv, 8, 3) == H.Band(11, 0, 700, 40.0, 0.6000000238418579, 30.0)
      and H.read_band(dv, 8, 3).pack() == sent, "write_band retries VC_ERR_BUSY and reads back as a Band")
_busy_first(fw, "FILTER.BAND", 10**6)
try:
    H.set_retry_busy(dv, "FILTER.BAND", sent, H.band_addr(8, 3), retries=3, delay_s=0)
    check(False, "set_retry_busy must give up")
except AssertionError as e:
    check("VC_ERR_BUSY" in str(e), "set_retry_busy gives up after its retries")
rb = H.band_readback(H.pack_band(34, 80.0, 0.707, 0.0, bypass=0xFF, qp_x512=5), 21)
check(struct.unpack(H.BAND_FMT, rb)[:3] == (34, 0, 0), "band_readback: bypass 0xFF reads 0, a crossover band reads qp 0")
H.write_route(dv, 1, 2, 1, -4.0, phase_invert=1)
check(H.read_route(dv, 1, 2) == H.Route(1, 1, -4.0) and fw.stored("ROUTE.MATRIX_ROUTE", 0x0102)[2:4] == b"\0\0",
      "write_route / read_route address (input << 8) | output, reserved zero")
check(H.wait_for(lambda: 3, lambda v: v > 2, timeout_s=0) == 3 and H.wait_for(lambda: 1, 2, timeout_s=0) == 1,
      "wait_for returns the last value, matched or not")
fw, dv = _paged_dev()
cfg = H.spdif_rx_config(dv)
check((cfg["limit"], cfg["enabled"], cfg["count"], cfg["pins"]) == (4, 1, 2, [5, 20, 21, 22]),
      "spdif_rx_config decodes the 7-byte SPDIF_RX_CONFIG")
want_blocks = dict(spdif=0, count=3, i2s=1, adat=0)
H.restore_input_blocks(dv, want_blocks)
check(H.input_blocks(dv) == want_blocks, "restore_input_blocks then input_blocks round-trips (no ADAT built)")
blob = bytearray(6284)
blob[H.INPUT_CONFIG_OFF + 11], blob[H.INPUT_CONFIG_OFF + 14] = 3, 0b110 + 1
check(H.blob_input_blocks(bytes(blob)) == dict(spdif=0, count=3, i2s=1, adat=1)
      and H.blob_input_blocks(bytes(6284)) is None,
      "blob_input_blocks decodes spdif_rx_count and input_enable_p1; 0 is absent")
check(H.spdif_source(0) == 1 and [H.spdif_source(i) for i in (1, 2, 3)] == [4, 5, 6],
      "spdif_source maps S/PDIF inputs 1 to 4 onto sources 1, 4, 5, 6")

_fmt = {"u8": "B", "u16": "H", "i16": "h", "u32": "I", "i32": "i"}
_lmap = json.loads((ROOT / "Documentation" / "protocol" / "legacy_map.json").read_text())
_fields = _lmap["opcodes"]["GET_STATUS"]["in"]["fields"]
bad = [sel for sel, (src, typ, off) in _fields.items()
       if not any(o == off and f == _fmt[typ] for _n, o, f in TM.STRUCTS.get(src, (0, ()))[1])]
check(not bad, f"every legacy_map.json GET_STATUS selector is a telemetry.STRUCTS field ({bad})")
fw, dv = _paged_dev()
st = TM.system_status(dv)
check((st["sample_rate_hz"], st["temperature_cdeg"], st["cpu1_load"]) == (48000, -512, 12) and TM.sample_rate(dv) == 48000,
      "system_status decodes SYS.SYSTEM_STATUS")
lv = TM.levels(dv, 17)
check(lv["peaks"][:3] == [100, 200, 300] and (lv["cpu0_load"], lv["cpu1_load"], lv["clip_flags"],
      lv["active_input_channels"]) == (41, 7, 5, 2), "levels decodes MEAS.LEVELS")

fw, dv = _paged_dev()
for name, n in (("STORE.OUTPUT_CONFIG_MODE", 1), ("ROUTE.MASTER_VOLUME_MODE", 1), ("ROUTE.MASTER_VOLUME", 4),
                ("ROUTE.USER_VOLUME", 4), ("ROUTE.USER_MUTE", 1), ("INPUT.SOURCE", 1), ("CLOCK.MCK_ENABLE", 1),
                ("CLOCK.MCK_PIN", 1), ("INPUT.LG_SOUND_SYNC_ENABLE", 1), ("OUTPUT.DAC_MUTE_CONFIG", 16)):
    fw._rw(name, n)
fw._ro("ROUTE.MASTER_VOLUME_SAVED", struct.pack("<f", -20.0))
fw._rw("CLOCK.I2S_BCK_PIN", 1, range(2))
fw._rw("OUTPUT.TYPE", 1, range(4))
fw._rw("ROUTE.CHANNEL_NAME", 32, range(17))
fw.put("ROUTE.USER_VOLUME", struct.pack("<f", -12.0))
fw.put("INPUT.SPDIF_RX_PIN", b"\x05", 0)
fw.put("OUTPUT.TYPE", b"\x01", 2)
fw.put("ROUTE.CHANNEL_NAME", b"Sub".ljust(32, b"\0"), 16)
fw.put("STORE.ALL_PARAMS", bytes(range(256)) * 24 + bytes(140))
prof = _NS(bulk_payload_len=6284, num_pin_outputs=5, num_spdif=4, num_channels=17)
snap = LC.capture(dv, prof)
check(len(snap.bulk) == 6284 and snap.bulk[:4] == bytes([0, 1, 2, 3]), "capture reads STORE.ALL_PARAMS")
check((snap.directory, snap.startup, snap.active_slot, snap.occupied_mask)
      == (bytes([5, 0, 1, 3, 2, 1, 0]), bytes([1, 3, 2]), 2, 5), "capture: directory, 3-byte startup, active slot")
check((snap.user_vol, snap.saved_mv, snap.spdif_rx_pin, snap.output_pins, snap.output_types, snap.sample_rate)
      == (-12.0, -20.0, 5, [6, 7, 8, 9, 10], [0, 0, 1, 0], 48000), "capture: scalars, pins, types, sample rate")
check(snap.channel_names[16][:3] == b"Sub" and len(snap.channel_names) == 17, "capture: channel names")
fw.put("STORE.ALL_PARAMS", bytes(6284))
left = _busy_first(fw, "STORE.ALL_PARAMS", 2)
LC.restore_live(dv, snap, settle_s=0)
check(left[0] == 0 and LC.diff_bulk(dv, snap, prof) == [], "restore_live retries VC_ERR_BUSY; diff_bulk is empty after")
fw.put("STORE.ALL_PARAMS", bytes(6284))
check(len(LC.diff_bulk(dv, snap, prof)) == sum(1 for x in snap.bulk if x), "diff_bulk lists every changed byte")
c = fw.cmds[fw._id("STORE.ALL_PARAMS")]
c.set = lambda cm, a, d: _VC["VC_ERR_VALUE"]
try:
    LC.restore_live(dv, snap, settle_s=0)
    check(False, "a refused bulk restore must raise")
except D.PagedStall as s:
    check(s.result_name == "VC_ERR_VALUE", "restore_live raises a refused bulk SET")

fw, dv = _paged_dev()
prof = _NS(num_input_channels=8, num_output_channels=9, num_channels=17, band_ceiling=10)
eq_act = dict(ACT.live_param_actions(dv, prof, __import__("random").Random(5)))["eq"]
_busy_first(fw, "FILTER.BAND", 1)
eq_act()
check(not [k for k in fw.store if k[0] == fw._id("FILTER.BAND")], "eq action drops a VC_ERR_BUSY band write")
eq_act()
check([k for k in fw.store if k[0] == fw._id("FILTER.BAND")], "and stores the next one")
fw.cmds[fw._id("FILTER.BAND")].set = lambda cm, a, d: _VC["VC_ERR_VALUE"]
try:
    eq_act()
    check(False, "eq action must raise a refusal other than VC_ERR_BUSY")
except D.PagedStall:
    check(True, "eq action raises a refusal other than VC_ERR_BUSY")

print("34. envelope alignment finds a burst at a frequency the reference lacks")
if A.np is None:
    skip("envelope alignment needs numpy")
else:
    _np = A.np
    fs = 48000
    ref = A.make_tone(fs, 60.0, 0.5, 0.4)
    # What subharm solo returns: only the f/2 sub, which the 60 Hz reference
    # does not correlate with at all.
    sub = A.make_tone(fs, 30.0, 0.5, 0.2)
    lead = 7000
    cap = _np.concatenate([_np.zeros(lead, _np.float32), sub,
                           _np.zeros(4000, _np.float32)])
    lag, strength = A._envelope_lag(cap, ref, fs)
    check(abs(lag - lead) < 0.02 * fs, f"envelope lag {lag} ~ {lead} (within 20 ms)")
    check(strength > 0.5, f"envelope alignment reports a strong lock ({strength:.2f})")
    wave_lag, wave_strength = A._xcorr_lag(cap, ref)
    check(wave_strength < strength,
          f"waveform correlation is the weaker lock ({wave_strength:.2f} at lag {wave_lag})")

print()
print(f"OK: {oks[0]}  SKIP: {len(skips)}  FAILURES: {len(fails)}")
for m in skips:
    print("  skipped:", m)
for f in fails:
    print("  -", f)
sys.exit(1 if fails else 0)
