"""
helpers.py — reusable SET/GET assertion patterns shared across test modules.

Live DSP params mutated here are all captured in the pre-suite bulk snapshot and
restored at the end, so individual tests need not restore unless they touch flash
or hardware resources (pins/types/input-source), which restore inline.
"""

import struct

from .device import OP


def float_roundtrip(dev, chk, set_op, get_op, value, *, tol=1e-3, wvalue=0, label=""):
    """SET a float, GET it back, assert equality within tol."""
    dev.set_f32(set_op, value, wvalue=wvalue)
    got = dev.get_f32(get_op, wvalue=wvalue)
    chk.approx(got, value, tol, label or f"roundtrip {value}")
    return got


def float_clamp(dev, chk, set_op, get_op, value, expected, *, tol=1e-3, wvalue=0, label=""):
    """SET an out-of-range float, assert it clamps to `expected`."""
    dev.set_f32(set_op, value, wvalue=wvalue)
    got = dev.get_f32(get_op, wvalue=wvalue)
    chk.approx(got, expected, tol, label or f"clamp {value}->{expected}")
    return got


def bool_roundtrip(dev, chk, set_op, get_op, *, wvalue=0, label="", truthy_byte=0xFF):
    """SET 1/0/truthy, assert GET normalizes to 1/0."""
    dev.set_u8(set_op, 1, wvalue=wvalue)
    chk.eq(dev.get_u8(get_op, wvalue=wvalue), 1, f"{label} set 1")
    dev.set_u8(set_op, 0, wvalue=wvalue)
    chk.eq(dev.get_u8(get_op, wvalue=wvalue), 0, f"{label} set 0")
    # Any nonzero coerces to 1 for the !=0 commands.
    dev.set_u8(set_op, truthy_byte, wvalue=wvalue)
    chk.eq(dev.get_u8(get_op, wvalue=wvalue), 1, f"{label} set 0x{truthy_byte:02X}->1")


def nan_rejected(dev, chk, set_op, get_op, *, wvalue=0, label=""):
    """SET a known value, then SET NaN, assert value unchanged (NaN rejected)."""
    dev.set_f32(set_op, -7.0, wvalue=wvalue)
    before = dev.get_f32(get_op, wvalue=wvalue)
    dev.set(set_op, struct.pack("<I", 0x7FC00000), wvalue=wvalue)  # quiet NaN
    after = dev.get_f32(get_op, wvalue=wvalue)
    chk.approx(after, before, 1e-3, f"{label} NaN rejected (unchanged)")


def short_payload_noop(dev, chk, set_op, get_op, *, four_byte=True, wvalue=0, label="", decode="f32"):
    """A too-short SET payload must ACK (no STALL) and leave the value unchanged."""
    if decode == "f32":
        before = dev.get_f32(get_op, wvalue=wvalue)
    else:
        before = dev.get_u8(get_op, wvalue=wvalue)
    # 1 byte for a 4-byte command, or 0 bytes is handled separately.
    chk.no_stall(lambda: dev.set(set_op, b"\x01", wvalue=wvalue), f"{label} short payload no STALL")
    after = dev.get_f32(get_op, wvalue=wvalue) if decode == "f32" else dev.get_u8(get_op, wvalue=wvalue)
    if decode == "f32":
        chk.approx(after, before, 1e-3, f"{label} short payload no-op")
    else:
        chk.eq(after, before, f"{label} short payload no-op")
