"""
Control Surfaces noun dispatch, driven through a macro on the live preview (plan section 12, Phase 3A).
Each case fires steps from an empty macro, checks the effect through the normal GETs and restores the
target. Every value is harmless: cuts only, effects left as found, bypassed or flat EQ bands.
"""

import struct
import time

from ..device import PagedStall, Stall
from ..framework import Skip, test
from .paged_cs import AUX_TYPES, PENDING, _limits, _paged, _settle, _snapshot, _status

# CsNoun numbers (control_surfaces.h).
MASTER_VOLUME, USER_MUTE, PRESET, CLIP, LEVELLER_SPEED = 1, 2, 6, 8, 14
PREAMP, OUTPUT_GAIN, OUTPUT_MUTE = 16, 17, 18
FILTER_FREQ, FILTER_GAIN, FILTER_Q, FILTER_TYPE, FILTER_BYPASS = 20, 21, 22, 23, 24
UPMIX_CENTER_MODE, UPMIX_WIDTH, PSYBASS_CUTOFF = 36, 39, 42
PRESET_RELOAD, LOUDNESS_SPL = 48, 49
SUBHARM_LOW, SUBHARM_BOOST, SUBHARM_TOP, SUBHARM_SELECT = 58, 60, 61, 62
TUBE_DRIVE, LIMITER_THRESHOLD, LIMITER_RELEASE = 71, 75, 76

INC, DEC, SET, TRIGGER = 2, 3, 5, 7
FLAG_GROUP = 0x20
TARGET_DSP_CH = 3
PEAKING, LOWSHELF, LT = 1, 2, 11
FLAT_WHEN_ZERO_GAIN = (PEAKING, LOWSHELF, 3, 9, 10)
BAND_FMT = "<BBHfff"
QP = 777                      # marker Qp: survives only if every RMW carries the stored value


def _q8(x):
    return int(round(x * 256))


def _f32(b):
    return struct.unpack("<f", b)[0]


def _step(noun, action, value=0, target=0, index=0, flags=0, step=0):
    return struct.pack("<BBBBBBhhH", noun, action, flags, target, index, 0, value, step, 0)


def _poll(read, done, timeout_s=2.0):
    """Last read once done(value) holds or the timeout passes; rides out a reset's busy window."""
    deadline = time.monotonic() + timeout_s
    v = None
    while True:
        try:
            v = read()
        except (Stall, OSError):
            time.sleep(0.05)
            if time.monotonic() > deadline:
                raise
            continue
        if done(v) or time.monotonic() > deadline:
            return v
        time.sleep(0.02)


class _Ctx:
    def __init__(self, dev, chk, profile, lim, mac, grp):
        self.dev, self.chk, self.profile, self.lim = dev, chk, profile, lim
        self.mac, self.grp = mac, grp
        self.seen = []

    def put(self, name, payload, addr, tag):
        """Preview SET that must apply cleanly; CS.STATUS reports the outcome."""
        self.dev.pset(name, payload, addr=addr)
        st = _settle(self.dev)
        if st is None or (st["last"], st["slot"]) != (0, tag):
            raise AssertionError(f"{name} @0x{addr:04X} not applied: {st and (hex(st['last']), hex(st['slot']))}")

    def run(self, steps, timeout_s=5.0):
        for k, s in enumerate(steps):
            self.put("CS.MACRO_STEP", s, (k << 8) | self.mac, 0x60 | self.mac)
        hdr = struct.pack("<32sBBBB", b"NounDispatch", len(steps), 0, 0, 0)
        self.put("CS.MACRO_HEADER", hdr, self.mac, 0x60 | self.mac)
        self.dev.pset("CS.MACRO_FIRE", addr=self.mac)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                if self.dev.pget("CS.EXT_STATUS")[3] == 0xFF:
                    return
            except (Stall, OSError):
                self.dev.wait_ready()
            time.sleep(0.02)
        raise AssertionError(f"macro {self.mac} still running after {timeout_s} s")

    def record(self, label, value):
        self.seen.append((label, value))


def _value_case(c, label, noun, cmd, want, value, *, action=SET, addr=0, target=0, step=0,
                kind="f32", tol=0.01, untouched=()):
    """Fire one step, compare the readback of `cmd` with `want`, restore its original bytes."""
    dev, chk = c.dev, c.chk
    orig = dev.pget(cmd, addr=addr)
    others = [(n, a, dev.pget(n, addr=a)) for n, a in untouched]
    dec = _f32 if kind == "f32" else (lambda b: b[0])
    close = (lambda v: abs(v - want) <= tol) if kind == "f32" else (lambda v: v == want)
    try:
        c.run([_step(noun, action, value, target, step=step)])
        got = _poll(lambda: dec(dev.pget(cmd, addr=addr)), close)
        if kind == "f32":
            chk.approx(got, want, tol, label)
        else:
            chk.eq(got, want, label)
        c.record(label, round(got, 4) if kind == "f32" else got)
        for n, a, b in others:
            chk.eq(dev.pget(n, addr=a), b, f"{label}: {n} untouched")
    finally:
        dev.pset(cmd, orig, addr=addr)
        chk.eq(dev.pget(cmd, addr=addr), orig, f"{label}: restored")


def _below(v, lo):
    """One whole dB under v, or None when that would leave the noun's range."""
    w = round(v) - 1
    return w if w >= lo else None


def _cut(c, label, v, lo):
    """_below, noting the skip so a missing case is visible in the report."""
    w = _below(v, lo)
    if w is None:
        c.chk.note(f"{label} at {v}: no cut inside the range, skipped")
    return w


def _other(v, lo):
    """A different whole value inside the range, for parameters of an effect that is off."""
    return _below(v, lo) if _below(v, lo) is not None else round(v) + 1


# ---- Cases ----

def _case_volume_and_mute(c):
    w = _cut(c, "MASTER_VOLUME", _f32(c.dev.pget("ROUTE.MASTER_VOLUME")), -127)
    if w is not None:
        _value_case(c, "MASTER_VOLUME SET (f32, no address)", MASTER_VOLUME, "ROUTE.MASTER_VOLUME", w, _q8(w))
    if c.dev.pget("ROUTE.USER_MUTE")[0] == 0:
        _value_case(c, "USER_MUTE SET 1 (u8 bool)", USER_MUTE, "ROUTE.USER_MUTE", 1, 1, kind="u8")
    else:
        c.chk.note("user mute already on: USER_MUTE skipped (a SET 0 would unmute)")
    ref = _f32(c.dev.pget("DYN.LOUDNESS_REF"))
    if ref - 1 >= 40:
        _value_case(c, "LOUDNESS_SPL DEC 1 dB (relative f32)", LOUDNESS_SPL, "DYN.LOUDNESS_REF", ref - 1, 0,
                    action=DEC, step=256, tol=0.5)
    else:
        _value_case(c, "LOUDNESS_SPL INC 1 dB (relative f32)", LOUDNESS_SPL, "DYN.LOUDNESS_REF", ref + 1, 0,
                    action=INC, step=256, tol=0.5)
    spd = c.dev.pget("DYN.LEVELLER_SPEED")[0]
    _value_case(c, "LEVELLER_SPEED SET (u8 enum)", LEVELLER_SPEED, "DYN.LEVELLER_SPEED", (spd + 1) % 3,
                (spd + 1) % 3, kind="u8")


def _case_per_channel(c):
    w = _cut(c, "OUTPUT_GAIN out 0", _f32(c.dev.pget("ROUTE.OUTPUT_GAIN", addr=0)), -60)
    if w is not None:
        _value_case(c, "OUTPUT_GAIN SET out 0 (f32, output address)", OUTPUT_GAIN, "ROUTE.OUTPUT_GAIN", w,
                    _q8(w), addr=0, target=0)
    outs = [o for o in range(c.profile.num_output_channels) if c.dev.pget("ROUTE.OUTPUT_MUTE", addr=o)[0] == 0]
    if not outs:
        c.chk.note("every output is muted: OUTPUT_MUTE skipped (a SET 0 would unmute)")
    else:
        o = outs[-1]
        _value_case(c, f"OUTPUT_MUTE SET 1 out {o} (u8, output address)", OUTPUT_MUTE, "ROUTE.OUTPUT_MUTE", 1, 1,
                    addr=o, target=o, kind="u8")
    w = _cut(c, "PREAMP in 1", _f32(c.dev.pget("ROUTE.PREAMP", addr=1)), -24)
    if w is not None:
        _value_case(c, "PREAMP SET in 1 (f32, input address)", PREAMP, "ROUTE.PREAMP", w, _q8(w), addr=1, target=1)


def _case_limiter(c):
    outs = [o for o in range(c.profile.num_output_channels)
            if _f32(c.dev.pget("DYN.LIMITER_LINK_GROUP", addr=o)) == 0]
    if not outs:
        c.chk.note("every output is in a limiter link group: limiter cases skipped")
        return
    o = outs[-1]
    w = _cut(c, f"LIMITER_THRESHOLD out {o}", _f32(c.dev.pget("DYN.LIMITER_THRESHOLD", addr=o)), -30)
    if w is not None:
        _value_case(c, f"LIMITER_THRESHOLD SET out {o}", LIMITER_THRESHOLD, "DYN.LIMITER_THRESHOLD", w, _q8(w),
                    addr=o, target=o, untouched=[("DYN.LIMITER_RELEASE", o), ("DYN.LIMITER_ENABLE", o)])
    r = _f32(c.dev.pget("DYN.LIMITER_RELEASE", addr=o))
    w = 200 if abs(r - 200) > 5 else 300
    _value_case(c, f"LIMITER_RELEASE SET out {o} (log ms)", LIMITER_RELEASE, "DYN.LIMITER_RELEASE", w, w,
                addr=o, target=o, tol=0.5, untouched=[("DYN.LIMITER_THRESHOLD", o)])


def _case_effects(c):
    dev = c.dev
    w = _cut(c, "TUBE_DRIVE", _f32(dev.pget("FX.TUBE_DRIVE")), -30)
    if w is not None:
        _value_case(c, "TUBE_DRIVE SET (former indexed tube command)", TUBE_DRIVE, "FX.TUBE_DRIVE", w, _q8(w),
                    untouched=[("FX.TUBE_MIX", 0), ("FX.TUBE_TYPE", 0), ("FX.TUBE_ENABLE", 0)])
    if dev.pget("FX.SUBHARM_ENABLE")[0]:
        c.chk.note("subharmonic synth is on: subharm cases skipped")
    else:
        w = _other(_f32(dev.pget("FX.SUBHARM_LOW")), -30)
        _value_case(c, "SUBHARM_LOW SET", SUBHARM_LOW, "FX.SUBHARM_LOW", w, _q8(w),
                    untouched=[("FX.SUBHARM_HIGH", 0)])
        w = _other(_f32(dev.pget("FX.SUBHARM_TOP")), -30)
        _value_case(c, "SUBHARM_TOP SET (its command follows BOOST's)", SUBHARM_TOP, "FX.SUBHARM_TOP", w,
                    _q8(w), untouched=[("FX.SUBHARM_BOOST", 0)])
        b = _f32(dev.pget("FX.SUBHARM_BOOST"))
        w = 2.0 if abs(b - 2.0) > 0.1 else 1.0
        _value_case(c, "SUBHARM_BOOST SET (synth off)", SUBHARM_BOOST, "FX.SUBHARM_BOOST", w, _q8(w),
                    untouched=[("FX.SUBHARM_TOP", 0)])
        sel = dev.pget("FX.SUBHARM_SELECT")[0]
        _value_case(c, "SUBHARM_SELECT SET (u8 enum)", SUBHARM_SELECT, "FX.SUBHARM_SELECT", (sel + 1) % 3,
                    (sel + 1) % 3, kind="u8")
    if dev.pget("FX.PSYBASS_ENABLE")[0]:
        c.chk.note("psychoacoustic bass is on: psybass case skipped")
    else:
        cut = _f32(dev.pget("FX.PSYBASS_CUTOFF"))
        w = 100 if abs(cut - 100) > 2 else 150
        _value_case(c, "PSYBASS_CUTOFF SET (Hz)", PSYBASS_CUTOFF, "FX.PSYBASS_CUTOFF", w, w,
                    untouched=[("FX.PSYBASS_HARMONICS", 0)])
    if 0x94 not in dev.paged_commands().get(0x08, (set(), set()))[0]:
        c.chk.note("no upmixer commands on this platform: upmix cases skipped")
    elif _f32(dev.pget("FX.UPMIX_ENABLE")) != 0:
        c.chk.note("upmixer is on: upmix cases skipped")
    else:
        wd = _f32(dev.pget("FX.UPMIX_CENTER_WIDTH"))
        w = 50 if abs(wd - 50) > 1 else 60
        _value_case(c, "UPMIX_WIDTH SET (former indexed upmix command)", UPMIX_WIDTH, "FX.UPMIX_CENTER_WIDTH", w,
                    _q8(w), untouched=[("FX.UPMIX_STRENGTH", 0)])
        m = _f32(dev.pget("FX.UPMIX_CENTER_MODE"))
        w = 0 if m != 0 else 1
        _value_case(c, "UPMIX_CENTER_MODE SET (enum as f32)", UPMIX_CENTER_MODE, "FX.UPMIX_CENTER_MODE", w, w)


def _band_set(dev, ch, band, payload, retries=50):
    for _ in range(retries):
        try:
            return dev.pset("FILTER.BAND", payload, addr=(ch << 8) | band)
        except PagedStall as s:
            if s.result_name != "VC_ERR_BUSY":
                raise
            time.sleep(0.01)
    raise AssertionError("FILTER.BAND stayed VC_ERR_BUSY")


def _band(dev, ch, band):
    return dev.pget("FILTER.BAND", addr=(ch << 8) | band)


def _band_put(c, ch, band, fields):
    payload = struct.pack(BAND_FMT, *fields)
    _band_set(c.dev, ch, band, payload)
    got = _poll(lambda: _band(c.dev, ch, band), lambda b: b == payload)
    if got != payload:
        raise AssertionError(f"band {ch}/{band} setup did not apply")


def _flat(rec):
    t, byp, _qp, _f, _q, g = struct.unpack(BAND_FMT, rec)
    return byp == 1 or t == 0 or (t in FLAT_WHEN_ZERO_GAIN and g == 0.0)


def _pick_bands(c):
    """Two output channels sharing a PEQ band index whose current setting is inaudible."""
    n_in = c.profile.num_input_channels
    chs = (n_in, n_in + 1)
    for band in range(19, -1, -1):
        try:
            recs = [_band(c.dev, ch, band) for ch in chs]
        except PagedStall:
            continue
        if all(_flat(r) for r in recs):
            return chs, band, recs
    return None


def _band_case(c, label, steps, ch, band, check, settle_s=0.0):
    c.run(steps)
    if settle_s:
        time.sleep(settle_s)
    got = _poll(lambda: struct.unpack(BAND_FMT, _band(c.dev, ch, band)), check)
    c.chk.ok(check(got), f"{label}: band {ch}/{band} = {got}")
    c.record(label, tuple(round(x, 3) if isinstance(x, float) else x for x in got))


def _case_eq(c):
    picked = _pick_bands(c)
    if picked is None:
        c.chk.note("no shared PEQ band is flat or bypassed on outputs 0 and 1: EQ cases skipped")
        return
    (a, b), band, orig = picked
    near = lambda x, y, r=0.005: abs(x - y) <= abs(y) * r   # noqa: E731
    try:
        _band_put(c, a, band, (PEAKING, 1, QP, 1000.0, 0.707, 0.0))
        _band_case(c, "FILTER_TYPE SET lowshelf (bypassed band)", [_step(FILTER_TYPE, SET, LOWSHELF, a, band)],
                   a, band, lambda v: v[0] == LOWSHELF and v[1:3] == (1, QP) and v[3] == 1000.0 and v[5] == 0.0)

        _band_put(c, a, band, (LT, 1, QP, 1000.0, 0.707, 50.0))
        _band_case(c, "FILTER_FREQ SET 2000 Hz (bypassed LT band, Qp kept)", [_step(FILTER_FREQ, SET, 2000, a, band)],
                   a, band, lambda v: v[:3] == (LT, 1, QP) and near(v[3], 2000.0) and v[5] == 50.0)
        before = struct.unpack(BAND_FMT, _band(c.dev, a, band))
        _band_case(c, "FILTER_GAIN SET on an LT band is a no-op", [_step(FILTER_GAIN, SET, _q8(-1), a, band)],
                   a, band, lambda v: v == before, settle_s=0.2)
        _band_case(c, "FILTER_Q SET 1.0 (bypassed LT band, Qp kept)", [_step(FILTER_Q, SET, 256, a, band)],
                   a, band, lambda v: v[:3] == (LT, 1, QP) and abs(v[4] - 1.0) < 1e-4)

        _band_put(c, a, band, (PEAKING, 0, QP, 1000.0, 0.707, 0.0))
        _band_case(c, "FILTER_BYPASS SET 1 (flat band)", [_step(FILTER_BYPASS, SET, 1, a, band)],
                   a, band, lambda v: v == (PEAKING, 1, QP, 1000.0, v[4], 0.0) and abs(v[4] - 0.707) < 1e-6)

        # Two members in one pump: the second always meets the first's pending apply (BUSY, retried).
        flat = (PEAKING, 1, QP, 1000.0, 0.707, 0.0)
        _band_put(c, a, band, flat)
        _band_put(c, b, band, flat)
        grp_rec = struct.pack("<BBBBI32s", TARGET_DSP_CH, 0, 0, 0, (1 << a) | (1 << b), b"NounGroup")
        c.put("CS.GROUP", grp_rec, c.grp, 0x40 | c.grp)
        c.run([_step(FILTER_FREQ, SET, 3000, c.grp, band, flags=FLAG_GROUP)])
        for ch in (a, b):
            v = _poll(lambda: struct.unpack(BAND_FMT, _band(c.dev, ch, band)), lambda v: near(v[3], 3000.0))
            c.chk.ok(near(v[3], 3000.0) and v[:3] == (PEAKING, 1, QP),
                     f"group FILTER_FREQ SET 3000 Hz reaches channel {ch} through the BUSY retry: {v}")
            c.record(f"group FILTER_FREQ ch {ch}", round(v[3], 2))
    finally:
        for ch, rec in ((a, orig[0]), (b, orig[1])):
            _band_set(c.dev, ch, band, rec)
            got = _poll(lambda: _band(c.dev, ch, band), lambda x: x == rec)
            c.chk.eq(got, rec, f"band {ch}/{band} restored")


def _case_clip(c):
    flags = struct.unpack("<I", c.dev.pget("MEAS.CLIP_FLAGS"))[0]
    c.run([_step(CLIP, TRIGGER)])
    after = struct.unpack("<I", c.dev.pget("MEAS.CLIP_FLAGS"))[0]
    c.chk.eq(after, 0, f"CLIP TRIGGER clears the latch (was 0x{flags:X})")
    if flags == 0:
        c.chk.note("clip latch was already clear, so CLIP shows no visible change")


def _case_preset(c, label, noun, action):
    """Reload the active slot from flash. A changed live marker proves the load ran; any unsaved
    live edit the reload discarded is put back from a bulk snapshot."""
    dev, chk = c.dev, c.chk
    active = dev.pget("STORE.PRESET_ACTIVE")[0]
    bulk0 = dev.pget("STORE.ALL_PARAMS")
    mv0 = dev.pget("ROUTE.MASTER_VOLUME")
    g = _f32(dev.pget("ROUTE.OUTPUT_GAIN", addr=0))
    if g - 0.37 < -60:
        chk.note(f"{label}: output 0 gain too low for a marker, skipped")
        return
    marker = struct.pack("<f", g - 0.37)
    dev.pset("ROUTE.OUTPUT_GAIN", marker, addr=0)
    try:
        c.run([_step(noun, action, active if noun == PRESET else 0)], timeout_s=10.0)
        dev.wait_ready()
        got = _poll(lambda: dev.pget("ROUTE.OUTPUT_GAIN", addr=0), lambda x: x != marker, timeout_s=5.0)
        chk.ne(got, marker, f"{label}: the live marker was replaced by slot {active}")
        chk.eq(dev.pget("STORE.PRESET_ACTIVE")[0], active, f"{label}: active slot unchanged")
        c.record(label, "reloaded" if got != marker else "not reloaded")
    finally:
        dev.wait_ready()
        if dev.pget("STORE.ALL_PARAMS") != bulk0:
            chk.note(f"{label}: live state differed from slot {active}; restored from the bulk snapshot")
            dev.pset("STORE.ALL_PARAMS", bulk0)
            time.sleep(0.4)
            dev.wait_ready()
        if dev.pget("ROUTE.MASTER_VOLUME") != mv0:
            dev.pset("ROUTE.MASTER_VOLUME", mv0)
        chk.eq(dev.pget("STORE.ALL_PARAMS"), bulk0, f"{label}: live parameters equal the pre-case snapshot")


@test("paged_cs_nouns", mutating=True, flash=1)
def pcsn_macro_noun_dispatch(dev, profile, chk):
    """Every noun dispatch shape fired from a macro changes its target as a host SET would; all restored, CS saved unchanged."""
    _paged(dev)
    lim = _limits(dev)
    st0 = _status(dev)
    if st0["dirty"]:
        raise Skip("Control Surfaces has an unsaved preview")
    if st0["last"] == PENDING:
        raise Skip("a deferred CS request is still pending")
    if dev.pget("CS.EXT_STATUS")[3] != 0xFF:
        raise Skip("a macro is running")
    snap = _snapshot(dev, lim)
    if any(b[0] in AUX_TYPES for b in snap["binding"]):
        raise Skip("an aux binding exists: SAVE would fold its live value")
    empty_mac = [m for m, r in enumerate(snap["macro"]) if r == bytes(132)]
    empty_grp = [g for g, r in enumerate(snap["group"]) if r == bytes(40)]
    if not empty_mac or not empty_grp:
        raise Skip("no empty macro or group record")
    c = _Ctx(dev, chk, profile, lim, empty_mac[-1], empty_grp[-1])

    try:
        for case in (_case_volume_and_mute, _case_per_channel, _case_limiter, _case_effects, _case_eq, _case_clip):
            case(c)
        _case_preset(c, "PRESET_RELOAD TRIGGER", PRESET_RELOAD, TRIGGER)
        _case_preset(c, "PRESET SET active slot", PRESET, SET)
    finally:
        mac, grp = c.mac, c.grp
        c.put("CS.MACRO_HEADER", snap["macro"][mac][:36], mac, 0x60 | mac)
        for k in range(lim["steps"]):
            stp = snap["macro"][mac][36 + 12 * k:48 + 12 * k]
            if dev.pget("CS.MACRO_STEP", addr=(k << 8) | mac) != stp:
                c.put("CS.MACRO_STEP", stp, (k << 8) | mac, 0x60 | mac)
        if dev.pget("CS.GROUP", addr=grp) != snap["group"][grp]:
            c.put("CS.GROUP", snap["group"][grp], grp, 0x40 | grp)
        for label, v in c.seen:
            chk.note(f"observed {label}: {v}")

    after = _snapshot(dev, lim)
    diff = [k for k in snap if snap[k] != after[k]]
    if diff:
        chk.ok(False, f"records differ from the snapshot after restore ({diff}); CS.SAVE NOT sent, preview left dirty")
        return
    chk.ok(True, "every CS record equals the pre-test snapshot")
    if _status(dev)["dirty"]:
        chk.eq(dev.pset("CS.SAVE"), 0, "CS.SAVE accepted")
        time.sleep(0.3)
        dev.wait_ready()
        st = _settle(dev, timeout_s=5.0)
        chk.ok(st is not None and (st["last"], st["slot"], st["dirty"]) == (0, 0xFF, 0),
               f"SAVE completes clean: {st and (hex(st['last']), hex(st['slot']), st['dirty'])}")
    chk.eq(_snapshot(dev, lim), snap, "records unchanged by the SAVE")
