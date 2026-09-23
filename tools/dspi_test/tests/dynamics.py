"""
Loudness / crossfeed / matrix-mixer group.

Loudness   0x58-0x5D
Crossfeed  0x5E-0x67
Matrix     0x70/0x71
Subharm    0x10-0x1F, 0x2C-0x2F, 0xA9-0xAE
Tube       0x3E-0x3F
Limiter    0x81 (OUT sets, IN gets)
"""

import struct

from ..device import OP, Stall
from ..framework import test
from ..helpers import float_roundtrip, float_clamp, bool_roundtrip


# --- Loudness ---------------------------------------------------------------

@test("dynamics", mutating=True)
def loudness_enable_bool(dev, profile, chk):
    """0x58/0x59 loudness enable != 0 coercion."""
    bool_roundtrip(dev, chk, OP.SET_LOUDNESS, OP.GET_LOUDNESS, label="loudness enable")


@test("dynamics", mutating=True)
def loudness_ref_clamp(dev, profile, chk):
    """0x5A/0x5B reference SPL clamps to [40,100]."""
    float_roundtrip(dev, chk, OP.SET_LOUDNESS_REF, OP.GET_LOUDNESS_REF, 60.0, label="ref 60")
    float_roundtrip(dev, chk, OP.SET_LOUDNESS_REF, OP.GET_LOUDNESS_REF, 40.0, label="ref 40")
    float_roundtrip(dev, chk, OP.SET_LOUDNESS_REF, OP.GET_LOUDNESS_REF, 100.0, label="ref 100")
    float_clamp(dev, chk, OP.SET_LOUDNESS_REF, OP.GET_LOUDNESS_REF, 0.0, 40.0, label="ref low clamp")
    float_clamp(dev, chk, OP.SET_LOUDNESS_REF, OP.GET_LOUDNESS_REF, 1000.0, 100.0, label="ref high clamp")


@test("dynamics", mutating=True)
def loudness_intensity_clamp(dev, profile, chk):
    """0x5C/0x5D intensity clamps to [0,200]."""
    float_roundtrip(dev, chk, OP.SET_LOUDNESS_INTENSITY, OP.GET_LOUDNESS_INTENSITY, 50.0, label="intensity 50")
    float_clamp(dev, chk, OP.SET_LOUDNESS_INTENSITY, OP.GET_LOUDNESS_INTENSITY, -10.0, 0.0, label="intensity low")
    float_clamp(dev, chk, OP.SET_LOUDNESS_INTENSITY, OP.GET_LOUDNESS_INTENSITY, 999.0, 200.0, label="intensity high")


# --- Crossfeed --------------------------------------------------------------

@test("dynamics", mutating=True)
def crossfeed_enable_itd_bools(dev, profile, chk):
    """0x5E/0x5F enable and 0x66/0x67 ITD use != 0 coercion."""
    bool_roundtrip(dev, chk, OP.SET_CROSSFEED, OP.GET_CROSSFEED, label="crossfeed enable")
    bool_roundtrip(dev, chk, OP.SET_CROSSFEED_ITD, OP.GET_CROSSFEED_ITD, label="crossfeed ITD")


@test("dynamics", mutating=True)
def crossfeed_preset_enum(dev, profile, chk):
    """0x60/0x61 preset 0-3 round-trips; >3 silently dropped (NOT clamped)."""
    for p in (0, 1, 2, 3):
        dev.set_u8(OP.SET_CROSSFEED_PRESET, p)
        chk.eq(dev.get_u8(OP.GET_CROSSFEED_PRESET), p, f"preset {p} roundtrip")
    dev.set_u8(OP.SET_CROSSFEED_PRESET, 1)       # known good
    dev.set_u8(OP.SET_CROSSFEED_PRESET, 4)       # invalid
    chk.eq(dev.get_u8(OP.GET_CROSSFEED_PRESET), 1, "preset 4 dropped (stays 1, not clamped to 3)")
    dev.set_u8(OP.SET_CROSSFEED_PRESET, 255)
    chk.eq(dev.get_u8(OP.GET_CROSSFEED_PRESET), 1, "preset 255 dropped (stays 1)")


@test("dynamics", mutating=True)
def crossfeed_freq_clamp(dev, profile, chk):
    """0x62/0x63 custom freq clamps to [500,2000] (value stored regardless of active preset)."""
    float_roundtrip(dev, chk, OP.SET_CROSSFEED_FREQ, OP.GET_CROSSFEED_FREQ, 1000.0, label="freq 1000")
    float_roundtrip(dev, chk, OP.SET_CROSSFEED_FREQ, OP.GET_CROSSFEED_FREQ, 500.0, label="freq 500")
    float_roundtrip(dev, chk, OP.SET_CROSSFEED_FREQ, OP.GET_CROSSFEED_FREQ, 2000.0, label="freq 2000")
    float_clamp(dev, chk, OP.SET_CROSSFEED_FREQ, OP.GET_CROSSFEED_FREQ, 100.0, 500.0, label="freq low clamp")
    float_clamp(dev, chk, OP.SET_CROSSFEED_FREQ, OP.GET_CROSSFEED_FREQ, 5000.0, 2000.0, label="freq high clamp")


@test("dynamics", mutating=True)
def crossfeed_feed_clamp(dev, profile, chk):
    """0x64/0x65 custom feed clamps to [0,15]."""
    float_roundtrip(dev, chk, OP.SET_CROSSFEED_FEED, OP.GET_CROSSFEED_FEED, 6.0, label="feed 6")
    float_clamp(dev, chk, OP.SET_CROSSFEED_FEED, OP.GET_CROSSFEED_FEED, -5.0, 0.0, label="feed low clamp")
    float_clamp(dev, chk, OP.SET_CROSSFEED_FEED, OP.GET_CROSSFEED_FEED, 30.0, 15.0, label="feed high clamp")


# --- Matrix mixer -----------------------------------------------------------

def _route_packet(inp, out, enabled, phase_invert, gain_db):
    return struct.pack("<BBBBf", inp, out, enabled, phase_invert, gain_db)


def _get_route(dev, inp, out):
    raw = dev.get(OP.GET_MATRIX_ROUTE, 8, wvalue=(inp << 8) | out)
    i, o, en, ph, gain = struct.unpack("<BBBBf", raw)
    return i, o, en, ph, gain


@test("dynamics", mutating=True)
def matrix_crosspoint_roundtrip(dev, profile, chk):
    """0x70/0x71: a crosspoint round-trips enabled/phase/gain; gain not clamped."""
    dev.set(OP.SET_MATRIX_ROUTE, _route_packet(0, 2, 1, 0, -3.0))
    i, o, en, ph, gain = _get_route(dev, 0, 2)
    chk.eq(i, 0, "route input echo")
    chk.eq(o, 2, "route output echo")
    chk.eq(en, 1, "route enabled")
    chk.eq(ph, 0, "route phase")
    chk.approx(gain, -3.0, 1e-3, "route gain")
    # Phase invert + extreme (unclamped) gain on the last valid output (platform-relative:
    # 4 on RP2040, 8 on RP2350 — output index must be < num_output_channels).
    out = profile.num_output_channels - 1
    dev.set(OP.SET_MATRIX_ROUTE, _route_packet(1, out, 1, 1, 30.0))
    _, _, en2, ph2, gain2 = _get_route(dev, 1, out)
    chk.eq(ph2, 1, "phase invert set")
    chk.approx(gain2, 30.0, 1e-3, "gain +30 stored verbatim (no clamp)")


@test("dynamics", mutating=True)
def matrix_set_drop_get_stall_asymmetry(dev, profile, chk):
    """SET to a bad index is a silent no-op; GET of a bad index STALLs (documented asymmetry)."""
    ni, no = profile.num_input_channels, profile.num_output_channels
    # Baseline a valid crosspoint.
    dev.set(OP.SET_MATRIX_ROUTE, _route_packet(0, 0, 1, 0, -1.0))
    _, _, _, _, before = _get_route(dev, 0, 0)
    chk.no_stall(lambda: dev.set(OP.SET_MATRIX_ROUTE, _route_packet(ni, 0, 1, 0, 9.0)),
                 "SET bad input no STALL")
    chk.no_stall(lambda: dev.set(OP.SET_MATRIX_ROUTE, _route_packet(0, no, 1, 0, 9.0)),
                 "SET bad output no STALL")
    _, _, _, _, after = _get_route(dev, 0, 0)
    chk.approx(after, before, 1e-3, "valid crosspoint unchanged by bad SETs")
    # GET out of range STALLs.
    chk.stalls(lambda: dev.get(OP.GET_MATRIX_ROUTE, 8, wvalue=(ni << 8) | 0), "GET bad input STALL")
    chk.stalls(lambda: dev.get(OP.GET_MATRIX_ROUTE, 8, wvalue=(0 << 8) | no), "GET bad output STALL")


@test("dynamics", mutating=True)
def matrix_full_sweep(dev, profile, chk):
    """Every input x output crosspoint round-trips (offset/index math)."""
    ni, no = profile.num_input_channels, profile.num_output_channels
    for inp in range(ni):
        for out in range(no):
            gain = -2.0 * inp - 0.25 * out
            en = (inp + out) % 2
            dev.set(OP.SET_MATRIX_ROUTE, _route_packet(inp, out, en, 0, gain))
    fails = 0
    for inp in range(ni):
        for out in range(no):
            gain = -2.0 * inp - 0.25 * out
            en = (inp + out) % 2
            i, o, ge, gp, gg = _get_route(dev, inp, out)
            if not (i == inp and o == out and ge == en and abs(gg - gain) < 1e-3):
                fails += 1
                if fails <= 4:
                    chk.ok(False, f"crosspoint [{inp}][{out}] mismatch: en={ge} gain={gg:.3f} (want en={en} gain={gain:.3f})")
    chk.eq(fails, 0, f"all {ni*no} crosspoints round-trip")


# --- Subharmonic synthesizer ------------------------------------------------

@test("dynamics", mutating=True)
def subharm_enable_bool(dev, profile, chk):
    """0x10/0x11 subharm enable != 0 coercion."""
    bool_roundtrip(dev, chk, OP.SET_SUBHARM, OP.GET_SUBHARM, label="subharm enable")


@test("dynamics", mutating=True)
def subharm_level_clamps(dev, profile, chk):
    """0x12-0x17 band levels clamp to [-30,+12], boost to [0,+6]."""
    for name, s, g in (("low", OP.SET_SUBHARM_LOW, OP.GET_SUBHARM_LOW),
                       ("high", OP.SET_SUBHARM_HIGH, OP.GET_SUBHARM_HIGH)):
        float_roundtrip(dev, chk, s, g, -6.0, label=f"{name} -6")
        float_clamp(dev, chk, s, g, -99.0, -30.0, label=f"{name} low clamp")
        float_clamp(dev, chk, s, g, 40.0, 12.0, label=f"{name} high clamp")
    float_roundtrip(dev, chk, OP.SET_SUBHARM_BOOST, OP.GET_SUBHARM_BOOST, 3.0, label="boost 3")
    float_clamp(dev, chk, OP.SET_SUBHARM_BOOST, OP.GET_SUBHARM_BOOST, -5.0, 0.0, label="boost low clamp")
    float_clamp(dev, chk, OP.SET_SUBHARM_BOOST, OP.GET_SUBHARM_BOOST, 20.0, 6.0, label="boost high clamp")


@test("dynamics", mutating=True)
def subharm_mask_roundtrip(dev, profile, chk):
    """0x18/0x19 output mask round-trips as raw uint16."""
    for m in (0x0001, 0x0005, 0xFFFF):
        dev.set(OP.SET_SUBHARM_MASK, struct.pack("<H", m))
        chk.eq(dev.get_u16(OP.GET_SUBHARM_MASK), m, f"mask 0x{m:04X}")


@test("dynamics", mutating=True)
def subharm_headroom(dev, profile, chk):
    """0x1A headroom is 0 dB while disabled and grows with band level and boost."""
    dev.set_u8(OP.SET_SUBHARM, 0)
    chk.eq(dev.get_f32(OP.GET_SUBHARM_HEADROOM), 0.0, "disabled reads 0 dB")
    dev.set_f32(OP.SET_SUBHARM_LOW, 0.0)
    dev.set_f32(OP.SET_SUBHARM_HIGH, -30.0)
    dev.set_f32(OP.SET_SUBHARM_BOOST, 0.0)
    dev.set_u8(OP.SET_SUBHARM, 1)
    one_band = dev.get_f32(OP.GET_SUBHARM_HEADROOM)
    # One band at 0 dB: the divided sub is ~0.85 of the band on top of the
    # direct tone, so the bound sits between 5 and 7 dB.
    chk.ok(5.0 < one_band < 7.0, f"one band at 0 dB needs {one_band:.2f} dB")
    dev.set_f32(OP.SET_SUBHARM_HIGH, 0.0)
    two_band = dev.get_f32(OP.GET_SUBHARM_HEADROOM)
    chk.ok(two_band > one_band, f"second band raises it ({one_band:.2f} -> {two_band:.2f} dB)")
    dev.set_f32(OP.SET_SUBHARM_BOOST, 6.0)
    boosted = dev.get_f32(OP.GET_SUBHARM_HEADROOM)
    chk.ok(boosted >= two_band + 3.0, f"+6 dB boost raises it ({two_band:.2f} -> {boosted:.2f} dB)")
    dev.set_u8(OP.SET_SUBHARM, 0)


@test("dynamics", mutating=True)
def subharm_top_clamp(dev, profile, chk):
    """0x1B/0x1C third band level clamps to [-30,+12]."""
    prev = dev.get_f32(OP.GET_SUBHARM_TOP)
    float_roundtrip(dev, chk, OP.SET_SUBHARM_TOP, OP.GET_SUBHARM_TOP, -12.0, label="top -12")
    float_clamp(dev, chk, OP.SET_SUBHARM_TOP, OP.GET_SUBHARM_TOP, -99.0, -30.0, label="top low clamp")
    float_clamp(dev, chk, OP.SET_SUBHARM_TOP, OP.GET_SUBHARM_TOP, 40.0, 12.0, label="top high clamp")
    dev.set_f32(OP.SET_SUBHARM_TOP, prev)


@test("dynamics", mutating=True)
def subharm_select_clamps(dev, profile, chk):
    """0x1D/0x1E selectivity mode round-trips 0-2; above 2 clamps (not dropped)."""
    prev = dev.get_u8(OP.GET_SUBHARM_SELECT)
    for m in (0, 1, 2):
        dev.set_u8(OP.SET_SUBHARM_SELECT, m)
        chk.eq(dev.get_u8(OP.GET_SUBHARM_SELECT), m, f"mode {m} roundtrip")
    dev.set_u8(OP.SET_SUBHARM_SELECT, 7)
    chk.eq(dev.get_u8(OP.GET_SUBHARM_SELECT), 2, "mode 7 clamps to 2")
    dev.set_u8(OP.SET_SUBHARM_SELECT, prev)


@test("dynamics", mutating=True)
def subharm_select_depth_hold_clamps(dev, profile, chk):
    """0xA9-0xAC selectivity depth clamps to [0,100] %, hold to [50,400] ms."""
    prev_depth = dev.get_f32(OP.GET_SUBHARM_DEPTH)
    prev_hold = dev.get_f32(OP.GET_SUBHARM_HOLD)
    float_roundtrip(dev, chk, OP.SET_SUBHARM_DEPTH, OP.GET_SUBHARM_DEPTH, 50.0, label="depth 50")
    float_clamp(dev, chk, OP.SET_SUBHARM_DEPTH, OP.GET_SUBHARM_DEPTH, -10.0, 0.0, label="depth low clamp")
    float_clamp(dev, chk, OP.SET_SUBHARM_DEPTH, OP.GET_SUBHARM_DEPTH, 999.0, 100.0, label="depth high clamp")
    float_roundtrip(dev, chk, OP.SET_SUBHARM_HOLD, OP.GET_SUBHARM_HOLD, 200.0, label="hold 200")
    float_clamp(dev, chk, OP.SET_SUBHARM_HOLD, OP.GET_SUBHARM_HOLD, 1.0, 50.0, label="hold low clamp")
    float_clamp(dev, chk, OP.SET_SUBHARM_HOLD, OP.GET_SUBHARM_HOLD, 5000.0, 400.0, label="hold high clamp")
    dev.set_f32(OP.SET_SUBHARM_DEPTH, prev_depth)
    dev.set_f32(OP.SET_SUBHARM_HOLD, prev_hold)


@test("dynamics", mutating=True)
def subharm_ceiling_clamp(dev, profile, chk):
    """0xAD/0xAE sub ceiling clamps to [-40,0] dBFS (0 = off)."""
    prev = dev.get_f32(OP.GET_SUBHARM_CEILING)
    float_roundtrip(dev, chk, OP.SET_SUBHARM_CEILING, OP.GET_SUBHARM_CEILING, -6.0, label="ceiling -6")
    float_clamp(dev, chk, OP.SET_SUBHARM_CEILING, OP.GET_SUBHARM_CEILING, -99.0, -40.0, label="ceiling low clamp")
    float_clamp(dev, chk, OP.SET_SUBHARM_CEILING, OP.GET_SUBHARM_CEILING, 12.0, 0.0, label="ceiling high clamp")
    dev.set_f32(OP.SET_SUBHARM_CEILING, prev)


@test("dynamics", mutating=True)
def subharm_link_solo_bools(dev, profile, chk):
    """0x2E/0x2F link and 0x2C/0x2D solo use != 0 coercion."""
    prev_link = dev.get_u8(OP.GET_SUBHARM_LINK)
    bool_roundtrip(dev, chk, OP.SET_SUBHARM_LINK, OP.GET_SUBHARM_LINK, label="subharm link")
    bool_roundtrip(dev, chk, OP.SET_SUBHARM_SOLO, OP.GET_SUBHARM_SOLO, label="subharm solo")
    dev.set_u8(OP.SET_SUBHARM_LINK, prev_link)
    # Solo is monitor-only and must never be left on for the next test.
    dev.set_u8(OP.SET_SUBHARM_SOLO, 0)
    chk.eq(dev.get_u8(OP.GET_SUBHARM_SOLO), 0, "solo restored to off")


@test("dynamics")
def subharm_meter_length(dev, profile, chk):
    """0x1F returns one uint16 sub peak per output channel."""
    n = profile.num_output_channels
    data = dev.get(OP.GET_SUBHARM_METER, 2 * n)
    chk.eq(len(data), 2 * n, f"{n} outputs -> {2 * n} bytes")
    peaks = struct.unpack(f"<{n}H", data)
    chk.ok(all(p <= 32767 for p in peaks), "every peak inside the 0..32767 status scale")


@test("dynamics", mutating=True)
def subharm_bulk_roundtrip(dev, profile, chk):
    """The V30 subharm wire section carries the new fields through GET/SET_ALL_PARAMS."""
    before = dev.get_ready(OP.GET_ALL_PARAMS, profile.bulk_payload_len)
    saved = {op: dev.get_f32(op) for op in (OP.GET_SUBHARM_TOP, OP.GET_SUBHARM_DEPTH,
                                            OP.GET_SUBHARM_HOLD, OP.GET_SUBHARM_CEILING)}
    saved_mode = dev.get_u8(OP.GET_SUBHARM_SELECT)
    saved_link = dev.get_u8(OP.GET_SUBHARM_LINK)
    dev.set_f32(OP.SET_SUBHARM_TOP, -9.0)
    dev.set_f32(OP.SET_SUBHARM_DEPTH, 25.0)
    dev.set_f32(OP.SET_SUBHARM_HOLD, 250.0)
    dev.set_f32(OP.SET_SUBHARM_CEILING, -12.0)
    dev.set_u8(OP.SET_SUBHARM_SELECT, 1)
    dev.set_u8(OP.SET_SUBHARM_LINK, 0)
    dev.set_u8(OP.SET_SUBHARM_SOLO, 1)
    blob = dev.get_ready(OP.GET_ALL_PARAMS, profile.bulk_payload_len)
    # Clear the live values, then prove the blob restores every persisted one.
    dev.set_f32(OP.SET_SUBHARM_TOP, -30.0)
    dev.set_f32(OP.SET_SUBHARM_DEPTH, 100.0)
    dev.set_f32(OP.SET_SUBHARM_HOLD, 50.0)
    dev.set_f32(OP.SET_SUBHARM_CEILING, 0.0)
    dev.set_u8(OP.SET_SUBHARM_SELECT, 0)
    dev.set_u8(OP.SET_SUBHARM_LINK, 1)
    dev.set(OP.SET_ALL_PARAMS, blob)
    dev.wait_ready()
    chk.approx(dev.get_f32(OP.GET_SUBHARM_TOP), -9.0, 1e-3, "top restored")
    chk.approx(dev.get_f32(OP.GET_SUBHARM_DEPTH), 25.0, 1e-3, "depth restored")
    chk.approx(dev.get_f32(OP.GET_SUBHARM_HOLD), 250.0, 1e-3, "hold restored")
    chk.approx(dev.get_f32(OP.GET_SUBHARM_CEILING), -12.0, 1e-3, "ceiling restored")
    chk.eq(dev.get_u8(OP.GET_SUBHARM_SELECT), 1, "select mode restored")
    chk.eq(dev.get_u8(OP.GET_SUBHARM_LINK), 0, "link restored")
    # Solo is deliberately off the wire, so a bulk apply must leave it alone.
    chk.eq(dev.get_u8(OP.GET_SUBHARM_SOLO), 1, "solo untouched by bulk apply")
    dev.set_u8(OP.SET_SUBHARM_SOLO, 0)
    dev.set(OP.SET_ALL_PARAMS, before)
    dev.wait_ready()
    for op, val in saved.items():
        chk.approx(dev.get_f32(op), val, 1e-3, f"pre-test 0x{op:02X} value restored")
    chk.eq(dev.get_u8(OP.GET_SUBHARM_SELECT), saved_mode, "pre-test select mode restored")
    chk.eq(dev.get_u8(OP.GET_SUBHARM_LINK), saved_link, "pre-test link restored")


# --- Tube preamp ------------------------------------------------------------

# Parameter indices travel in wValue on both 0x3E and 0x3F; they mirror
# TUBE_PARAM_* in firmware/DSPi/tube.h and are the wire/flash field order.
T_ENABLED, T_MASK, T_TYPE, T_DRIVE = 0, 1, 2, 3
T_BIAS, T_ASYM, T_HARDNESS, T_SAG = 4, 5, 6, 7
T_RECTIFIER, T_XFMR_EN, T_XDAMP, T_XRES = 8, 9, 10, 11
T_MIX, T_TRIM = 12, 13
T_NUM_PARAMS = 14


def _tube_set(dev, index, value):
    return dev.set_f32(OP.SET_TUBE_PARAM, value, wvalue=index)


def _tube_get(dev, index):
    return dev.get_f32(OP.GET_TUBE_PARAM, wvalue=index)


@test("dynamics", mutating=True)
def tube_param_roundtrip(dev, profile, chk):
    """0x3E/0x3F carry the parameter index in wValue: drive round-trips."""
    prev = _tube_get(dev, T_DRIVE)
    float_roundtrip(dev, chk, OP.SET_TUBE_PARAM, OP.GET_TUBE_PARAM, 12.0,
                    wvalue=T_DRIVE, label="drive 12 dB")
    _tube_set(dev, T_DRIVE, prev)


@test("dynamics", mutating=True)
def tube_param_clamps(dev, profile, chk):
    """0x3E clamps every parameter: drive, mix, damping factor and resonance."""
    prev_drive = _tube_get(dev, T_DRIVE)
    prev_mix = _tube_get(dev, T_MIX)
    prev_damp = _tube_get(dev, T_XDAMP)
    prev_res = _tube_get(dev, T_XRES)
    float_clamp(dev, chk, OP.SET_TUBE_PARAM, OP.GET_TUBE_PARAM, 99.0, 24.0,
                wvalue=T_DRIVE, label="drive high clamp")
    float_clamp(dev, chk, OP.SET_TUBE_PARAM, OP.GET_TUBE_PARAM, -20.0, -6.0,
                wvalue=T_DRIVE, label="drive low clamp")
    float_clamp(dev, chk, OP.SET_TUBE_PARAM, OP.GET_TUBE_PARAM, 150.0, 100.0,
                wvalue=T_MIX, label="mix high clamp")
    float_clamp(dev, chk, OP.SET_TUBE_PARAM, OP.GET_TUBE_PARAM, 0.5, 1.0,
                wvalue=T_XDAMP, label="damping low clamp")
    float_clamp(dev, chk, OP.SET_TUBE_PARAM, OP.GET_TUBE_PARAM, 99.0, 20.0,
                wvalue=T_XDAMP, label="damping high clamp")
    float_clamp(dev, chk, OP.SET_TUBE_PARAM, OP.GET_TUBE_PARAM, 10.0, 30.0,
                wvalue=T_XRES, label="resonance low clamp")
    float_clamp(dev, chk, OP.SET_TUBE_PARAM, OP.GET_TUBE_PARAM, 999.0, 150.0,
                wvalue=T_XRES, label="resonance high clamp")
    _tube_set(dev, T_DRIVE, prev_drive)
    _tube_set(dev, T_MIX, prev_mix)
    _tube_set(dev, T_XDAMP, prev_damp)
    _tube_set(dev, T_XRES, prev_res)


# Character fields must go back before the type: restoring a non-custom type
# last re-runs its row lookup, so the two can never be left disagreeing.
_T_CHARACTER = (T_BIAS, T_ASYM, T_HARDNESS, T_SAG)


def _tube_save_voicing(dev):
    return [(idx, _tube_get(dev, idx)) for idx in _T_CHARACTER + (T_TYPE,)]


def _tube_restore_voicing(dev, saved):
    for idx, val in saved:
        _tube_set(dev, idx, val)


@test("dynamics", mutating=True)
def tube_enable_bool(dev, profile, chk):
    """Index 0 is a bool on a float wire: it reads back as exactly 1.0 / 0.0."""
    prev = _tube_get(dev, T_ENABLED)
    _tube_set(dev, T_ENABLED, 1.0)
    chk.approx(_tube_get(dev, T_ENABLED), 1.0, 1e-6, "enabled set 1")
    _tube_set(dev, T_ENABLED, 0.0)
    chk.approx(_tube_get(dev, T_ENABLED), 0.0, 1e-6, "enabled set 0")
    _tube_set(dev, T_ENABLED, prev)


@test("dynamics", mutating=True)
def tube_mask_roundtrip(dev, profile, chk):
    """Index 1 is a uint16 mask carried as a float and read back unrounded."""
    prev = _tube_get(dev, T_MASK)
    _tube_set(dev, T_MASK, 3.0)
    chk.approx(_tube_get(dev, T_MASK), 3.0, 1e-6, "mask 0x0003")
    _tube_set(dev, T_MASK, 65535.0)
    chk.approx(_tube_get(dev, T_MASK), 65535.0, 1e-6, "mask 0xFFFF")
    _tube_set(dev, T_MASK, prev)


@test("dynamics", mutating=True)
def tube_type_loads_row(dev, profile, chk):
    """Index 2 copies a character row in; editing a row field flips it to Custom."""
    saved = _tube_save_voicing(dev)
    _tube_set(dev, T_TYPE, 16.0)   # 300B / 2A3
    chk.approx(_tube_get(dev, T_BIAS), 12.0, 1e-3, "300B bias")
    chk.approx(_tube_get(dev, T_ASYM), 6.0, 1e-3, "300B asymmetry")
    chk.approx(_tube_get(dev, T_HARDNESS), 10.0, 1e-3, "300B hardness")
    chk.approx(_tube_get(dev, T_SAG), 12.0, 1e-3, "300B sag")
    # A character edit that lands on a different value must reset the picker.
    _tube_set(dev, T_BIAS, 12.5)
    chk.approx(_tube_get(dev, T_TYPE), 0.0, 1e-6, "bias edit flips type to Custom")
    _tube_restore_voicing(dev, saved)


@test("dynamics", mutating=True)
def tube_same_value_edit_keeps_type(dev, profile, chk):
    """A character SET landing on the stored value must leave the type picker alone."""
    saved = _tube_save_voicing(dev)
    _tube_set(dev, T_TYPE, 16.0)   # 300B / 2A3: bias 12, asym 6, hardness 10, sag 12
    _tube_set(dev, T_BIAS, 12.0)   # identical to what the row just stored
    chk.approx(_tube_get(dev, T_TYPE), 16.0, 1e-6, "same-value bias SET keeps type 16")
    _tube_set(dev, T_BIAS, 13.0)
    chk.approx(_tube_get(dev, T_TYPE), 0.0, 1e-6, "changed bias SET flips type to Custom")
    _tube_restore_voicing(dev, saved)


@test("dynamics")
def tube_bad_index_stalls(dev, profile, chk):
    """A GET past the last parameter index STALLs (never returns stale bytes)."""
    chk.stalls(lambda: dev.get(OP.GET_TUBE_PARAM, 4, wvalue=T_NUM_PARAMS),
               f"index {T_NUM_PARAMS} STALL")


@test("dynamics", mutating=True)
def tube_bad_set_is_silent_noop(dev, profile, chk):
    """SET at an index past the last one, or with a short payload, ACKs and changes nothing."""
    prev = _tube_get(dev, T_DRIVE)
    _tube_set(dev, T_DRIVE, 9.0)
    chk.no_stall(lambda: _tube_set(dev, T_NUM_PARAMS, 21.0), "bad SET index no STALL")
    chk.approx(_tube_get(dev, T_DRIVE), 9.0, 1e-3, "drive unchanged by bad-index SET")
    # Under four bytes the dispatcher never reaches tube_set_param at all.
    chk.no_stall(lambda: dev.set(OP.SET_TUBE_PARAM, b"\x00\x00", wvalue=T_DRIVE),
                 "short tube payload no STALL")
    chk.approx(_tube_get(dev, T_DRIVE), 9.0, 1e-3, "drive unchanged by short SET")
    _tube_set(dev, T_DRIVE, prev)


@test("dynamics", mutating=True)
def tube_bulk_roundtrip(dev, profile, chk):
    """The V31 tube wire section carries the module through GET/SET_ALL_PARAMS."""
    before = dev.get_ready(OP.GET_ALL_PARAMS, profile.bulk_payload_len)
    probes = ((T_DRIVE, 15.0, 0.0), (T_MIX, 40.0, 100.0), (T_TRIM, -6.0, 0.0),
              (T_XRES, 120.0, 30.0))
    saved = {idx: _tube_get(dev, idx) for idx, _, _ in probes}
    voicing = _tube_save_voicing(dev)
    for idx, want, _ in probes:
        _tube_set(dev, idx, want)
    # Park on a Custom voicing no row can produce (EL34 leaves bias at 0), so an
    # apply that re-derived the character fields from tube_type would lose it.
    _tube_set(dev, T_TYPE, 12.0)   # EL34
    _tube_set(dev, T_BIAS, 40.0)   # drops the picker to 0 Custom
    chk.approx(_tube_get(dev, T_TYPE), 0.0, 1e-6, "bias edit armed Custom before the GET")
    blob = dev.get_ready(OP.GET_ALL_PARAMS, profile.bulk_payload_len)
    # Clear the live values, then prove the blob restores every one of them.
    for idx, _, clear in probes:
        _tube_set(dev, idx, clear)
    _tube_set(dev, T_TYPE, 16.0)   # 300B row overwrites bias with 12
    dev.set(OP.SET_ALL_PARAMS, blob)
    dev.wait_ready()
    for idx, want, _ in probes:
        chk.approx(_tube_get(dev, idx), want, 1e-3, f"index {idx} restored")
    chk.approx(_tube_get(dev, T_BIAS), 40.0, 1e-3, "custom bias survives the apply")
    chk.approx(_tube_get(dev, T_TYPE), 0.0, 1e-6, "type 0 Custom survives the apply")
    dev.set(OP.SET_ALL_PARAMS, before)
    dev.wait_ready()
    for idx, val in saved.items():
        chk.approx(_tube_get(dev, idx), val, 1e-3, f"pre-test index {idx} restored")
    for idx, val in voicing:
        chk.approx(_tube_get(dev, idx), val, 1e-3, f"pre-test voicing index {idx} restored")


# --- Output limiter ----------------------------------------------------------

# One opcode for both directions: wValue = (output << 8) | index.  Indices
# mirror LIMITER_PARAM_* / LIMITER_GET_* in firmware/DSPi/limiter.h.
L_ENABLED, L_THRESH, L_RELEASE, L_GROUP = 0, 1, 2, 3
L_NUM_PARAMS = 4
L_METER, L_STATUS = 0x80, 0x81
L_ALL = 0xFF


def _lim_set(dev, out, index, value):
    return dev.set_f32(OP.LIMITER, value, wvalue=(out << 8) | index)


def _lim_get(dev, out, index):
    return dev.get_f32(OP.LIMITER, wvalue=(out << 8) | index)


def _lim_status(dev):
    engaged, lookahead, block, n_out = dev.get(OP.LIMITER, 4, wvalue=L_STATUS)
    return engaged, lookahead, block, n_out


def _lim_save(dev, profile):
    return [[_lim_get(dev, k, i) for i in range(L_NUM_PARAMS)]
            for k in range(profile.num_output_channels)]


def _lim_restore(dev, saved):
    for k, vals in enumerate(saved):
        for i, v in enumerate(vals):
            _lim_set(dev, k, i, v)


@test("dynamics")
def limiter_status_block(dev, profile, chk):
    """GET index 0x81 reports the fixed 32-sample lookahead, 16-sample block and output count."""
    _, lookahead, block, n_out = _lim_status(dev)
    chk.eq(lookahead, 32, "lookahead samples")
    chk.eq(block, 16, "block samples")
    chk.eq(n_out, profile.num_output_channels, "num_outputs")


@test("dynamics", mutating=True)
def limiter_param_roundtrip(dev, profile, chk):
    """Each parameter round-trips per output, independently of its neighbours."""
    saved = _lim_save(dev, profile)
    _lim_set(dev, 1, L_THRESH, -7.5)
    _lim_set(dev, 1, L_RELEASE, 250.0)
    _lim_set(dev, 1, L_GROUP, 2.0)
    _lim_set(dev, 0, L_THRESH, -3.0)
    chk.approx(_lim_get(dev, 1, L_THRESH), -7.5, 1e-4, "out 1 threshold")
    chk.approx(_lim_get(dev, 1, L_RELEASE), 250.0, 1e-3, "out 1 release")
    chk.approx(_lim_get(dev, 1, L_GROUP), 2.0, 1e-6, "out 1 link group")
    chk.approx(_lim_get(dev, 0, L_THRESH), -3.0, 1e-4, "out 0 threshold untouched by out 1")
    _lim_restore(dev, saved)


@test("dynamics", mutating=True)
def limiter_param_clamps(dev, profile, chk):
    """Threshold -30..0 dB, release 10..1000 ms, link group 0..4 (rounded)."""
    saved = _lim_save(dev, profile)
    for val, want, what in ((5.0, 0.0, "threshold hi"), (-99.0, -30.0, "threshold lo")):
        _lim_set(dev, 0, L_THRESH, val)
        chk.approx(_lim_get(dev, 0, L_THRESH), want, 1e-4, what)
    for val, want, what in ((1.0, 10.0, "release lo"), (5000.0, 1000.0, "release hi")):
        _lim_set(dev, 0, L_RELEASE, val)
        chk.approx(_lim_get(dev, 0, L_RELEASE), want, 1e-3, what)
    for val, want, what in ((9.0, 4.0, "group hi"), (2.4, 2.0, "group rounds"), (-3.0, 0.0, "group lo")):
        _lim_set(dev, 0, L_GROUP, val)
        chk.approx(_lim_get(dev, 0, L_GROUP), want, 1e-6, what)
    _lim_restore(dev, saved)


@test("dynamics", mutating=True)
def limiter_all_outputs_set(dev, profile, chk):
    """Output byte 0xFF on a SET writes that parameter on every output."""
    saved = _lim_save(dev, profile)
    _lim_set(dev, L_ALL, L_THRESH, -4.0)
    for k in range(profile.num_output_channels):
        chk.approx(_lim_get(dev, k, L_THRESH), -4.0, 1e-4, f"out {k} threshold")
    _lim_restore(dev, saved)


@test("dynamics")
def limiter_bad_get_stalls(dev, profile, chk):
    """A GET for a missing output, a missing index or output 0xFF STALLs."""
    n = profile.num_output_channels
    chk.stalls(lambda: dev.get(OP.LIMITER, 4, wvalue=(n << 8) | L_THRESH), "output past the end")
    chk.stalls(lambda: dev.get(OP.LIMITER, 4, wvalue=L_NUM_PARAMS), "index past the end")
    chk.stalls(lambda: dev.get(OP.LIMITER, 4, wvalue=(L_ALL << 8) | L_THRESH), "all-outputs GET")


@test("dynamics", mutating=True)
def limiter_bad_set_is_silent_noop(dev, profile, chk):
    """SET with a bad output, a bad index or a short payload ACKs and changes nothing."""
    saved = _lim_save(dev, profile)
    _lim_set(dev, 0, L_THRESH, -9.0)
    n = profile.num_output_channels
    chk.no_stall(lambda: _lim_set(dev, n, L_THRESH, -2.0), "bad output no STALL")
    chk.no_stall(lambda: _lim_set(dev, 0, L_NUM_PARAMS, -2.0), "bad index no STALL")
    chk.no_stall(lambda: dev.set(OP.LIMITER, b"\x00\x00", wvalue=L_THRESH), "short payload no STALL")
    chk.approx(_lim_get(dev, 0, L_THRESH), -9.0, 1e-4, "threshold unchanged")
    _lim_restore(dev, saved)


@test("dynamics")
def limiter_meter_shape(dev, profile, chk):
    """GET index 0x80 returns one uint16 per output."""
    n = profile.num_output_channels
    raw = dev.get(OP.LIMITER, 2 * n, wvalue=L_METER)
    chk.eq(len(raw), 2 * n, "meter length")


@test("dynamics", mutating=True)
def limiter_engage_follows_enable(dev, profile, chk):
    """Enabling any output engages the lookahead delay; disabling the last releases it."""
    import time
    saved = _lim_save(dev, profile)
    _lim_set(dev, L_ALL, L_ENABLED, 0.0)

    def wait_engaged(want):
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if _lim_status(dev)[0] == want:
                return True
            time.sleep(0.02)
        return False

    chk.ok(wait_engaged(0), "released with every output off")
    _lim_set(dev, 0, L_ENABLED, 1.0)
    chk.ok(wait_engaged(1), "engaged after enabling output 0")
    _lim_set(dev, 0, L_ENABLED, 0.0)
    chk.ok(wait_engaged(0), "released after disabling output 0")
    _lim_restore(dev, saved)
    if any(vals[L_ENABLED] for vals in saved):
        chk.ok(wait_engaged(1), "pre-test engaged state restored")


@test("dynamics", mutating=True, flash=2)
def limiter_bulk_roundtrip(dev, profile, chk):
    """Bulk SET applies the V32 limiter section in with-preset mode and ignores it in independent mode."""
    orig_mode = dev.get_u8(OP.GET_OUTPUT_CONFIG_MODE)
    before = dev.get_ready(OP.GET_ALL_PARAMS, profile.bulk_payload_len)
    saved = _lim_save(dev, profile)
    last = profile.num_output_channels - 1
    marked = ((L_THRESH, -12.5), (L_RELEASE, 333.0), (L_GROUP, 3.0), (L_ENABLED, 1.0))
    cleared = ((L_THRESH, -1.0), (L_RELEASE, 100.0), (L_GROUP, 0.0), (L_ENABLED, 0.0))
    try:
        for idx, v in marked:
            _lim_set(dev, last, idx, v)
        blob = dev.get_ready(OP.GET_ALL_PARAMS, profile.bulk_payload_len)

        dev.set_u8(OP.SET_OUTPUT_CONFIG_MODE, 1)
        dev.wait_ready()
        for idx, v in cleared:
            _lim_set(dev, last, idx, v)
        dev.set(OP.SET_ALL_PARAMS, blob)
        dev.wait_ready()
        for idx, v in marked:
            chk.approx(_lim_get(dev, last, idx), v, 1e-3, f"with-preset: index {idx} restored")

        dev.set_u8(OP.SET_OUTPUT_CONFIG_MODE, 0)
        dev.wait_ready()
        for idx, v in cleared:
            _lim_set(dev, last, idx, v)
        dev.set(OP.SET_ALL_PARAMS, blob)
        dev.wait_ready()
        for idx, v in cleared:
            chk.approx(_lim_get(dev, last, idx), v, 1e-3, f"independent: index {idx} untouched")
    finally:
        dev.set_u8(OP.SET_OUTPUT_CONFIG_MODE, orig_mode)
        dev.wait_ready()
        dev.set(OP.SET_ALL_PARAMS, before)
        dev.wait_ready()
        _lim_restore(dev, saved)   # bulk SET skips the limiter in independent mode
    for k, vals in enumerate(saved):
        for i, v in enumerate(vals):
            chk.approx(_lim_get(dev, k, i), v, 1e-3, f"pre-test out {k} index {i} restored")
