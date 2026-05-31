"""
Diagnostics & DSP-worker group.

Buffer stats     0xB0/0xB1
USB error stats  0xB2/0xB3   (TinyUSB stub — always all-zero)
Volume leveller  0xB4-0xBF
"""

import struct

from ..device import OP, Stall
from ..framework import test
from ..helpers import float_roundtrip, float_clamp, bool_roundtrip, nan_rejected


# --- Buffer statistics ------------------------------------------------------

@test("diagnostics")
def buffer_stats_shape(dev, profile, chk):
    """0xB0 returns a 44-byte packet; num_spdif matches platform; all % in [0,100]."""
    data = dev.get(OP.GET_BUFFER_STATS, 44)
    chk.eq(len(data), 44, "buffer stats length")
    num_spdif = data[0]
    chk.eq(num_spdif, profile.num_spdif, "num_spdif header")
    # SpdifBufferStats[4] start at offset 4, 8 bytes each.
    for i in range(4):
        base = 4 + i * 8
        free, prep, playing, fill, mn, mx = data[base:base + 6]
        chk.in_range(free, 0, 16, f"spdif[{i}] consumer_free")
        chk.in_range(prep, 0, 16, f"spdif[{i}] consumer_prepared")
        chk.member(playing, (0, 1), f"spdif[{i}] consumer_playing")
        chk.in_range(fill, 0, 100, f"spdif[{i}] fill_pct")
        chk.in_range(mn, 0, 100, f"spdif[{i}] min_fill_pct")
        chk.in_range(mx, 0, 100, f"spdif[{i}] max_fill_pct")
    # PDM stats at offset 36.
    for j, name in enumerate(["dma_fill", "dma_min", "dma_max", "ring_fill", "ring_min", "ring_max"]):
        chk.in_range(data[36 + j], 0, 100, f"pdm {name}")


@test("diagnostics")
def buffer_stats_sequence_monotonic(dev, profile, chk):
    """The sequence is a per-read monotonic counter: advances, never jumps backward (mod 2^16).

    Firmware bumps it once per GET (`buffer_stats_sequence++`).  We don't assert
    an exact +1 on every consecutive pair because the host USB stack can coalesce
    or re-issue a control read under load/re-enumeration; the real invariant is a
    small forward step each time and overall advance.
    """
    seq = [struct.unpack_from("<H", dev.get(OP.GET_BUFFER_STATS, 44), 2)[0] for _ in range(8)]
    for a, b in zip(seq, seq[1:]):
        delta = (b - a) & 0xFFFF
        chk.ok(0 <= delta <= 3, f"sequence step {a}->{b} = {delta} (expected small forward step)")
    total = (seq[-1] - seq[0]) & 0xFFFF
    chk.in_range(total, 1, 24, "sequence advanced over 8 reads (monotonic counter alive)")


@test("diagnostics", mutating=True)
def buffer_stats_reset(dev, profile, chk):
    """0xB1 acks 0x01 and reseeds watermarks to a coherent state.

    If the pipeline is processing a source (e.g. SPDIF input is active) the
    watermarks immediately re-fill, so we assert coherence — each in [0,100] and
    min<=max, or the untouched reset sentinel (min=100,max=0) — rather than a
    silent-idle 100/0 (which only holds when nothing is flowing).
    """
    ack = dev.get(OP.RESET_BUFFER_STATS, 1, wvalue=1)
    chk.eq(ack[0], 0x01, "reset ack")
    data = dev.get(OP.GET_BUFFER_STATS, 44)
    for i in range(profile.num_spdif):
        base = 4 + i * 8
        mn, mx = data[base + 4], data[base + 5]
        chk.in_range(mn, 0, 100, f"spdif[{i}] min_fill in range")
        chk.in_range(mx, 0, 100, f"spdif[{i}] max_fill in range")
        coherent = (mn <= mx) or (mn == 100 and mx == 0)
        chk.ok(coherent, f"spdif[{i}] watermarks coherent (min={mn} max={mx})")


# --- USB error stats (TinyUSB stub) -----------------------------------------

@test("diagnostics")
def usb_error_stats_zero_stub(dev, profile, chk):
    """0xB2 returns 24 bytes, all six u32 counters zero (TinyUSB stub); nonzero = regression."""
    data = dev.get(OP.GET_USB_ERROR_STATS, 24)
    chk.eq(len(data), 24, "usb error stats length")
    fields = struct.unpack("<6I", data)
    names = ["total", "crc", "bitstuff", "rx_overflow", "rx_timeout", "data_seq"]
    for n, v in zip(names, fields):
        chk.eq(v, 0, f"usb_error.{n} (stub must be 0)")


@test("diagnostics", mutating=True)
def usb_error_stats_reset(dev, profile, chk):
    """0xB3 acks 0x01 and counters stay zero."""
    chk.eq(dev.get(OP.RESET_USB_ERROR_STATS, 1)[0], 0x01, "reset ack")
    fields = struct.unpack("<6I", dev.get(OP.GET_USB_ERROR_STATS, 24))
    chk.ok(all(v == 0 for v in fields), "all counters zero after reset")


# --- Volume leveller --------------------------------------------------------

@test("diagnostics", mutating=True)
def leveller_enable_lookahead_bools(dev, profile, chk):
    """0xB4/0xBC enable & lookahead booleans round-trip and coerce nonzero->1."""
    bool_roundtrip(dev, chk, OP.SET_LEVELLER_ENABLE, OP.GET_LEVELLER_ENABLE, label="leveller enable")
    bool_roundtrip(dev, chk, OP.SET_LEVELLER_LOOKAHEAD, OP.GET_LEVELLER_LOOKAHEAD, label="leveller lookahead")


@test("diagnostics", mutating=True)
def leveller_amount_clamp(dev, profile, chk):
    """0xB6 amount round-trips and clamps to [0,100]."""
    float_roundtrip(dev, chk, OP.SET_LEVELLER_AMOUNT, OP.GET_LEVELLER_AMOUNT, 50.0, label="amount 50")
    float_roundtrip(dev, chk, OP.SET_LEVELLER_AMOUNT, OP.GET_LEVELLER_AMOUNT, 0.0, label="amount 0")
    float_roundtrip(dev, chk, OP.SET_LEVELLER_AMOUNT, OP.GET_LEVELLER_AMOUNT, 100.0, label="amount 100")
    float_clamp(dev, chk, OP.SET_LEVELLER_AMOUNT, OP.GET_LEVELLER_AMOUNT, -10.0, 0.0, label="amount low clamp")
    float_clamp(dev, chk, OP.SET_LEVELLER_AMOUNT, OP.GET_LEVELLER_AMOUNT, 150.0, 100.0, label="amount high clamp")


@test("diagnostics", mutating=True)
def leveller_speed_rejects_out_of_range(dev, profile, chk):
    """0xB8 speed enum 0-2 round-trips; >=3 is REJECTED (value unchanged), not clamped."""
    for v in (0, 1, 2):
        dev.set_u8(OP.SET_LEVELLER_SPEED, v)
        chk.eq(dev.get_u8(OP.GET_LEVELLER_SPEED), v, f"speed {v} roundtrip")
    dev.set_u8(OP.SET_LEVELLER_SPEED, 1)  # known good
    dev.set_u8(OP.SET_LEVELLER_SPEED, 3)  # invalid
    chk.eq(dev.get_u8(OP.GET_LEVELLER_SPEED), 1, "speed 3 rejected (stays 1, NOT clamped to 2)")
    dev.set_u8(OP.SET_LEVELLER_SPEED, 255)
    chk.eq(dev.get_u8(OP.GET_LEVELLER_SPEED), 1, "speed 255 rejected (stays 1)")


@test("diagnostics", mutating=True)
def leveller_max_gain_clamp(dev, profile, chk):
    """0xBA max_gain round-trips and clamps to [0,35] (NOT 24)."""
    float_roundtrip(dev, chk, OP.SET_LEVELLER_MAX_GAIN, OP.GET_LEVELLER_MAX_GAIN, 15.0, label="max_gain 15")
    float_roundtrip(dev, chk, OP.SET_LEVELLER_MAX_GAIN, OP.GET_LEVELLER_MAX_GAIN, 35.0, label="max_gain 35")
    float_clamp(dev, chk, OP.SET_LEVELLER_MAX_GAIN, OP.GET_LEVELLER_MAX_GAIN, -5.0, 0.0, label="max_gain low clamp")
    float_clamp(dev, chk, OP.SET_LEVELLER_MAX_GAIN, OP.GET_LEVELLER_MAX_GAIN, 50.0, 35.0, label="max_gain ceiling 35")


@test("diagnostics", mutating=True)
def leveller_gate_clamp(dev, profile, chk):
    """0xBE gate round-trips and clamps to [-96,0]."""
    float_roundtrip(dev, chk, OP.SET_LEVELLER_GATE, OP.GET_LEVELLER_GATE, -96.0, label="gate -96")
    float_roundtrip(dev, chk, OP.SET_LEVELLER_GATE, OP.GET_LEVELLER_GATE, -50.0, label="gate -50")
    float_roundtrip(dev, chk, OP.SET_LEVELLER_GATE, OP.GET_LEVELLER_GATE, 0.0, label="gate 0")
    float_clamp(dev, chk, OP.SET_LEVELLER_GATE, OP.GET_LEVELLER_GATE, -120.0, -96.0, label="gate low clamp")
    float_clamp(dev, chk, OP.SET_LEVELLER_GATE, OP.GET_LEVELLER_GATE, 10.0, 0.0, label="gate high clamp")


@test("diagnostics", mutating=True)
def leveller_independence(dev, profile, chk):
    """Changing one leveller param leaves the other five untouched (no struct aliasing)."""
    # Set a known baseline.
    dev.set_u8(OP.SET_LEVELLER_ENABLE, 1)
    dev.set_f32(OP.SET_LEVELLER_AMOUNT, 40.0)
    dev.set_u8(OP.SET_LEVELLER_SPEED, 2)
    dev.set_f32(OP.SET_LEVELLER_MAX_GAIN, 20.0)
    dev.set_u8(OP.SET_LEVELLER_LOOKAHEAD, 0)
    dev.set_f32(OP.SET_LEVELLER_GATE, -60.0)
    # Mutate only amount; everything else must hold.
    dev.set_f32(OP.SET_LEVELLER_AMOUNT, 77.0)
    chk.eq(dev.get_u8(OP.GET_LEVELLER_ENABLE), 1, "enable unchanged")
    chk.eq(dev.get_u8(OP.GET_LEVELLER_SPEED), 2, "speed unchanged")
    chk.approx(dev.get_f32(OP.GET_LEVELLER_MAX_GAIN), 20.0, 1e-3, "max_gain unchanged")
    chk.eq(dev.get_u8(OP.GET_LEVELLER_LOOKAHEAD), 0, "lookahead unchanged")
    chk.approx(dev.get_f32(OP.GET_LEVELLER_GATE), -60.0, 1e-3, "gate unchanged")
    chk.approx(dev.get_f32(OP.GET_LEVELLER_AMOUNT), 77.0, 1e-3, "amount changed")
