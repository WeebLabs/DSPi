"""
EQ / preamp / bypass / delay / legacy gain & mute group.

EQ band          0x42/0x43
Legacy preamp    0x44/0x45
Master EQ bypass 0x46/0x47
Per-channel delay 0x48/0x49
Legacy gain      0x54/0x55
Legacy mute      0x56/0x57
Per-channel preamp 0xD0/0xD1
Per-band bypass  0xD8/0xD9
"""

import struct

from ..device import OP, Stall
from ..framework import test
from ..helpers import bool_roundtrip, nan_rejected

# FilterType enum.
FLAT, PEAKING, LOWSHELF, HIGHSHELF, LOWPASS, HIGHPASS, NOTCH, ALLPASS = range(8)
# EQ GET param selectors.
P_TYPE, P_FREQ, P_Q, P_GAIN, P_BYPASS = 0, 1, 2, 3, 4


def _eq_packet(ch, band, ftype, freq, Q, gain, bypass=0):
    return struct.pack("<BBBBfff", ch, band, ftype, bypass, freq, Q, gain)


def _set_band(dev, ch, band, ftype, freq, Q, gain, bypass=0):
    dev.set(OP.SET_EQ_PARAM, _eq_packet(ch, band, ftype, freq, Q, gain, bypass))


def _get(dev, ch, band, param):
    wv = (ch << 8) | (band << 4) | param
    if param in (P_FREQ, P_Q, P_GAIN):
        return dev.get_f32(OP.GET_EQ_PARAM, wvalue=wv)
    return dev.get_u32(OP.GET_EQ_PARAM, wvalue=wv)


@test("eq", mutating=True)
def eq_band_roundtrip(dev, profile, chk):
    """0x42/0x43: a full band round-trips type/freq/Q/gain/bypass on representative channels."""
    for ch in (0, 2, profile.num_channels - 1):
        _set_band(dev, ch, 0, PEAKING, 1000.0, 1.0, 6.0, bypass=0)
        chk.eq(_get(dev, ch, 0, P_TYPE), PEAKING, f"ch{ch} type")
        chk.approx(_get(dev, ch, 0, P_FREQ), 1000.0, 1e-2, f"ch{ch} freq")
        chk.approx(_get(dev, ch, 0, P_Q), 1.0, 1e-3, f"ch{ch} Q")
        chk.approx(_get(dev, ch, 0, P_GAIN), 6.0, 1e-3, f"ch{ch} gain")
        chk.eq(_get(dev, ch, 0, P_BYPASS), 0, f"ch{ch} bypass")


@test("eq", mutating=True)
def eq_all_filter_types(dev, profile, chk):
    """All filter types 0-7 set/read on (ch0,band0); unknown type 8 stored verbatim, no STALL."""
    for t in range(8):
        _set_band(dev, 0, 0, t, 500.0, 0.707, 3.0)
        chk.eq(_get(dev, 0, 0, P_TYPE), t, f"filter type {t}")
    _set_band(dev, 0, 0, 8, 500.0, 0.707, 3.0)
    chk.eq(_get(dev, 0, 0, P_TYPE), 8, "unknown type 8 stored verbatim")


@test("eq", mutating=True)
def eq_freq_q_stored_raw_quirk(dev, profile, chk):
    """Quirk: freq/Q clamp only the running copy; the recipe (and GET) keep RAW values."""
    # Non-flat band so coefficients are actually computed (clamp path runs internally).
    _set_band(dev, 0, 1, PEAKING, 1.0, 50.0, 6.0)   # freq below 10, Q above 20
    chk.approx(_get(dev, 0, 1, P_FREQ), 1.0, 1e-3, "freq stored RAW (not clamped to 10)")
    chk.approx(_get(dev, 0, 1, P_Q), 50.0, 1e-3, "Q stored RAW (not clamped to 20)")
    _set_band(dev, 0, 1, PEAKING, 1e6, 0.001, 6.0)
    chk.approx(_get(dev, 0, 1, P_FREQ), 1e6, 1.0, "freq stored RAW (not clamped to 0.45Fs)")
    chk.approx(_get(dev, 0, 1, P_Q), 0.001, 1e-4, "Q stored RAW (not clamped to 0.1)")


@test("eq")
def eq_invalid_index_stalls(dev, profile, chk):
    """GET_EQ_PARAM STALLs on channel >= count and band >= ceiling."""
    chk.stalls(lambda: _get(dev, profile.num_channels, 0, P_TYPE), "ch==count STALL")
    chk.stalls(lambda: _get(dev, 0, profile.band_ceiling, P_TYPE), "band==ceiling STALL")
    # Last valid coordinates must NOT stall.
    chk.no_stall(lambda: _get(dev, profile.num_channels - 1, 0, P_TYPE), "last channel ok")
    chk.no_stall(lambda: _get(dev, 0, profile.band_ceiling - 1, P_TYPE), "last band ok")


@test("eq", mutating=True)
def eq_bad_set_is_silent_noop(dev, profile, chk):
    """SET to an invalid channel/band ACKs (no STALL) and changes nothing."""
    # Establish a known value first.
    _set_band(dev, 0, 0, PEAKING, 800.0, 1.0, 2.0)
    before = _get(dev, 0, 0, P_FREQ)
    chk.no_stall(lambda: dev.set(OP.SET_EQ_PARAM, _eq_packet(profile.num_channels, 0, PEAKING, 1234.0, 1.0, 2.0)),
                 "bad-channel SET no STALL")
    chk.no_stall(lambda: dev.set(OP.SET_EQ_PARAM, _eq_packet(0, profile.band_ceiling, PEAKING, 1234.0, 1.0, 2.0)),
                 "bad-band SET no STALL")
    chk.approx(_get(dev, 0, 0, P_FREQ), before, 1e-3, "valid band unchanged by bad SETs")
    # Short payload also a silent no-op.
    chk.no_stall(lambda: dev.set(OP.SET_EQ_PARAM, b"\x00\x00"), "short EQ payload no STALL")
    chk.approx(_get(dev, 0, 0, P_FREQ), before, 1e-3, "valid band unchanged by short SET")


@test("eq", mutating=True)
def band_bypass_strict_one(dev, profile, chk):
    """0xD8 per-band bypass: strict ==1 (0xFF/2 -> active) and preserves freq/Q/gain/type."""
    _set_band(dev, 3, 2, PEAKING, 500.0, 2.0, 4.0, bypass=0)
    dev.set_u8(OP.SET_BAND_BYPASS, 1, wvalue=(3 << 8) | 2)
    chk.eq(dev.get_u8(OP.GET_BAND_BYPASS, wvalue=(3 << 8) | 2), 1, "bypass set 1")
    chk.eq(_get(dev, 3, 2, P_BYPASS), 1, "EQ param bypass reads 1")
    # Strict ==1: 0xFF and 2 mean ACTIVE (0), unlike the != 0 commands.
    dev.set_u8(OP.SET_BAND_BYPASS, 0xFF, wvalue=(3 << 8) | 2)
    chk.eq(dev.get_u8(OP.GET_BAND_BYPASS, wvalue=(3 << 8) | 2), 0, "0xFF -> active (strict ==1)")
    dev.set_u8(OP.SET_BAND_BYPASS, 2, wvalue=(3 << 8) | 2)
    chk.eq(dev.get_u8(OP.GET_BAND_BYPASS, wvalue=(3 << 8) | 2), 0, "2 -> active (strict ==1)")
    # Params preserved across bypass toggling.
    chk.approx(_get(dev, 3, 2, P_FREQ), 500.0, 1e-2, "freq preserved")
    chk.approx(_get(dev, 3, 2, P_Q), 2.0, 1e-3, "Q preserved")
    chk.approx(_get(dev, 3, 2, P_GAIN), 4.0, 1e-3, "gain preserved")
    chk.eq(_get(dev, 3, 2, P_TYPE), PEAKING, "type preserved")


@test("eq")
def band_bypass_invalid_index_stalls(dev, profile, chk):
    """0xD9 GET STALLs on bad channel/band; 0xD8 SET to bad index is a silent no-op."""
    chk.stalls(lambda: dev.get_u8(OP.GET_BAND_BYPASS, wvalue=(profile.num_channels << 8)), "bad ch STALL")
    chk.stalls(lambda: dev.get_u8(OP.GET_BAND_BYPASS, wvalue=profile.band_ceiling), "bad band STALL")
    chk.no_stall(lambda: dev.set_u8(OP.SET_BAND_BYPASS, 1, wvalue=(profile.num_channels << 8)),
                 "bad-index SET no STALL")


@test("eq", mutating=True)
def master_eq_bypass_bool(dev, profile, chk):
    """0x46/0x47 master EQ bypass uses != 0 (0xFF -> 1)."""
    bool_roundtrip(dev, chk, OP.SET_BYPASS, OP.GET_BYPASS, label="master EQ bypass")


@test("eq", mutating=True)
def legacy_preamp(dev, profile, chk):
    """0x44 sets all input channels; 0x45 returns channel 0; NaN rejected."""
    dev.set_f32(OP.SET_PREAMP, 6.0)
    chk.approx(dev.get_f32(OP.GET_PREAMP), 6.0, 1e-3, "legacy preamp GET (ch0)")
    chk.approx(dev.get_f32(OP.GET_PREAMP_CH, wvalue=0), 6.0, 1e-3, "ch0 == 6")
    chk.approx(dev.get_f32(OP.GET_PREAMP_CH, wvalue=1), 6.0, 1e-3, "ch1 == 6 (both set)")
    # Extreme finite values accepted (no clamp).
    dev.set_f32(OP.SET_PREAMP, 48.0)
    chk.approx(dev.get_f32(OP.GET_PREAMP), 48.0, 1e-3, "no clamp on +48")
    # NaN rejected.
    nan_rejected(dev, chk, OP.SET_PREAMP, OP.GET_PREAMP, label="legacy preamp")


@test("eq", mutating=True)
def per_channel_preamp(dev, profile, chk):
    """0xD0/0xD1 per-channel preamp: independent channels, NaN rejected, ch>=2 GET STALLs."""
    dev.set_f32(OP.SET_PREAMP_CH, 3.0, wvalue=0)
    dev.set_f32(OP.SET_PREAMP_CH, -3.0, wvalue=1)
    chk.approx(dev.get_f32(OP.GET_PREAMP_CH, wvalue=0), 3.0, 1e-3, "ch0 independent")
    chk.approx(dev.get_f32(OP.GET_PREAMP_CH, wvalue=1), -3.0, 1e-3, "ch1 independent")
    nan_rejected(dev, chk, OP.SET_PREAMP_CH, OP.GET_PREAMP_CH, wvalue=0, label="preamp ch0")
    chk.stalls(lambda: dev.get_f32(OP.GET_PREAMP_CH, wvalue=profile.num_input_channels),
               "GET preamp ch==NUM_INPUT STALL")
    chk.no_stall(lambda: dev.set_f32(OP.SET_PREAMP_CH, 1.0, wvalue=profile.num_input_channels),
                 "SET preamp bad ch no STALL")


@test("eq", mutating=True)
def per_channel_delay(dev, profile, chk):
    """0x48/0x49 delay: round-trip, negative clamps to 0, large value stored raw, ch>=count STALL."""
    dev.set_f32(OP.SET_DELAY, 10.0, wvalue=2)
    chk.approx(dev.get_f32(OP.GET_DELAY, wvalue=2), 10.0, 1e-3, "delay roundtrip")
    dev.set_f32(OP.SET_DELAY, -5.0, wvalue=2)
    chk.approx(dev.get_f32(OP.GET_DELAY, wvalue=2), 0.0, 1e-3, "negative clamps to 0")
    dev.set_f32(OP.SET_DELAY, 1000.0, wvalue=2)
    chk.approx(dev.get_f32(OP.GET_DELAY, wvalue=2), 1000.0, 1e-2, "large value stored raw")
    chk.stalls(lambda: dev.get_f32(OP.GET_DELAY, wvalue=profile.num_channels), "ch==count STALL")
    chk.no_stall(lambda: dev.set_f32(OP.SET_DELAY, 1.0, wvalue=profile.num_channels), "bad-ch SET no STALL")


@test("eq", mutating=True)
def legacy_channel_gain(dev, profile, chk):
    """0x54/0x55 legacy 3-channel gain: round-trip ch0-2 (finite only); ch3 GET STALLs."""
    for ch in range(3):
        dev.set_f32(OP.SET_CHANNEL_GAIN, -3.0 - ch, wvalue=ch)
        chk.approx(dev.get_f32(OP.GET_CHANNEL_GAIN, wvalue=ch), -3.0 - ch, 1e-3, f"gain ch{ch}")
    chk.stalls(lambda: dev.get_f32(OP.GET_CHANNEL_GAIN, wvalue=3), "gain ch3 STALL")
    chk.no_stall(lambda: dev.set_f32(OP.SET_CHANNEL_GAIN, 0.0, wvalue=3), "gain bad-ch SET no STALL")


@test("eq", mutating=True)
def legacy_channel_mute(dev, profile, chk):
    """0x56/0x57 legacy 3-channel mute: != 0 coercion; ch3 GET STALLs."""
    for ch in range(3):
        bool_roundtrip(dev, chk, OP.SET_CHANNEL_MUTE, OP.GET_CHANNEL_MUTE, wvalue=ch, label=f"mute ch{ch}")
    chk.stalls(lambda: dev.get_u8(OP.GET_CHANNEL_MUTE, wvalue=3), "mute ch3 STALL")
