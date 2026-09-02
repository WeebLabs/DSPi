"""
Loudness / crossfeed / matrix-mixer group.

Loudness   0x58-0x5D
Crossfeed  0x5E-0x67
Matrix     0x70/0x71
Subharm    0x10-0x1A
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
    """0x12-0x17 band levels clamp to [-30,+6], boost to [0,+6]."""
    for name, s, g in (("low", OP.SET_SUBHARM_LOW, OP.GET_SUBHARM_LOW),
                       ("high", OP.SET_SUBHARM_HIGH, OP.GET_SUBHARM_HIGH)):
        float_roundtrip(dev, chk, s, g, -6.0, label=f"{name} -6")
        float_clamp(dev, chk, s, g, -99.0, -30.0, label=f"{name} low clamp")
        float_clamp(dev, chk, s, g, 40.0, 6.0, label=f"{name} high clamp")
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
