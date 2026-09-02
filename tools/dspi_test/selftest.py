"""selftest.py: no-hardware checks for the audio-loopback harness.

Covers the parts of tests/audio_loopback.py whose correctness does not depend on
a device being attached, against mocks that model the firmware's actual
semantics:

  * Deferred-operation barriers.  A SET arms a pending flag and NO-OPS when the
    request equals the APPLIED state; the main loop applies it later
    (vendor_commands.c REQ_SET_OUTPUT_TYPE, main.c process_type_switches).
    Test 3 deliberately reproduces the pre-barrier bug, so a regression that
    reintroduces fire-and-assume sequencing shows up here rather than as an
    intermittent hardware failure.
  * Matrix isolation.  _route_only() must clear EVERY input row, since the
    upmixer feeds rows 2..4 and multichannel USB feeds up to row 7.
  * Per-channel band ceilings, which are probed rather than taken from the
    profile's channel-0 value.

    python3 -m tools.dspi_test.selftest      # exit 0 = all good
"""
import pathlib
import re
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tools.dspi_test.device import OP, Stall, Timeout
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

    def get_u8(self, opcode, wvalue=0):
        self._tick()
        if opcode == OP.GET_OUTPUT_TYPE:
            self.gets += 1
            if self.stall_n > 0:
                self.stall_n -= 1
                raise Stall(opcode, "IN", -9, "control-IRQ blackout")
            return self.applied
        if opcode == OP.SET_OUTPUT_TYPE:
            self.sets += 1
            new_type = (wvalue >> 8) & 0xFF
            if new_type == self.applied:      # <-- the stale-comparison no-op
                return 0
            self.pending = new_type
            self.pending_at = time.monotonic() + self.apply_after_s
            return 0
        raise AssertionError(f"unexpected opcode {opcode:#x}")


def old_set_output_type(dev, slot, otype):
    """What the code did before Phase 1: fire and assume."""
    return dev.get_u8(OP.SET_OUTPUT_TYPE, wvalue=(otype << 8) | slot)


fails = []


def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        fails.append(msg)


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

    def set(self, opcode, payload=b"", wvalue=0, windex=0):
        assert opcode == OP.SET_MATRIX_ROUTE
        inp, out, enabled, _phase, _gain = struct.unpack("<BBBBf", payload)
        self.xp[(inp, out)] = bool(enabled)
        return 0

    def get(self, opcode, length, wvalue=0):
        assert opcode == OP.GET_EQ_PARAM
        ch, band = (wvalue >> 8) & 0xFF, (wvalue >> 3) & 0x1F
        if band >= self.band_counts.get(ch, 10):
            raise Stall(opcode, "IN", -9, "band above ceiling")
        return b"\0\0\0\0"


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
upmix_rows = [k for k in d.xp if k[0] in (2, 3, 4)]
check(len(upmix_rows) == 3 * 11 and not any(d.xp[k] for k in upmix_rows),
      "upmixer rows 2..4 explicitly disabled")

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
audio_all = [tc.name for tc in REGISTRY if tc.group == "audio"]
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

    def get_u32(self, opcode, wvalue=0):
        assert wvalue == L.STATUS_SAMPLE_RATE
        return self.rate


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
    return names


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

    def get_u32(self, opcode, wvalue=0):
        if not self.available:
            raise Stall(opcode, "IN", -9, "release build: index unsupported")
        return self.c[0 if wvalue == L.STATUS_LB_OVERFLOW else 1]

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

print("24. LT SET packet is 18 bytes with the Qp sidecar the firmware reads")


class EqDev:
    def __init__(self):
        self.payload = None

    def set(self, opcode, payload=b"", wvalue=0, windex=0):
        assert opcode == OP.SET_EQ_PARAM
        self.payload = payload
        return len(payload)


d = EqDev()
L._set_lt_band(d, 8, 0, 80.0, 0.707, 50.0, 0.707)
check(len(d.payload) == 18, f"18-byte packet (got {len(d.payload)})")
ch, band, ftype, byp, f0, q0, fp = struct.unpack("<BBBBfff", d.payload[:16])
check(ftype == 11, f"FILTER_LINKWITZ_TRANSFORM (got {ftype})")
check((ch, band) == (8, 0), f"channel/band ({ch},{band})")
check(abs(f0 - 80.0) < 1e-3 and abs(q0 - 0.707) < 1e-3 and abs(fp - 50.0) < 1e-3,
      "f0/Q0/fp carried in the freq/Q/gain fields")
# Firmware reads vendor_rx_buf[16] | (vendor_rx_buf[17] << 8) as Qp * 512.
qp = struct.unpack("<H", d.payload[16:18])[0]
check(qp == round(0.707 * 512), f"Qp sidecar = round(Qp*512) = {qp}")

print("25. LT reference matches the documented DC boost (f0/fp)^2")
from tools.filter_tester.user_linkwitz import _lt_biquad
import numpy as _np
for f0, fp, fs in ((80.0, 50.0, 48000.0), (80.0, 50.0, 44100.0)):
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
          "subharm_octave"):
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
    print("  (numpy missing: skipped)")
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
    print("  (sounddevice/numpy missing: worker ping skipped)")

print()
print("FAILURES:", len(fails))
for f in fails:
    print("  -", f)
sys.exit(1 if fails else 0)
