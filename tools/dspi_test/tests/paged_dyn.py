"""
Native paged tests for the DYN page (plan section 5.7): limiter, leveller, loudness.

Old opcodes are read with raw_get/raw_set so the A/B comparison never goes through the
translation layer. The limiter keeps its float32 payloads for every parameter.
"""

import struct

from ..device import OP
from ..framework import Skip, test
from .paged_sys import _expect_stall, _paged

DYN = 0x07
NAN = struct.pack("<I", 0x7FC00000)
LIM = ["LIMITER_ENABLE", "LIMITER_THRESHOLD", "LIMITER_RELEASE", "LIMITER_LINK_GROUP"]
ALL = 0xFF


def _page(dev):
    _paged(dev)
    if DYN not in dev.paged_commands():
        raise Skip("DYN page not implemented on this firmware")


def _f(v):
    return struct.pack("<f", v)


def _f32(b):
    return struct.unpack("<f", b)[0]


def _lim_save(dev, profile):
    return [[dev.pget(f"DYN.{n}", k) for n in LIM] for k in range(profile.num_output_channels)]


def _lim_restore(dev, saved):
    # Unlink first: a SET on a linked output also writes its group members.
    dev.pset("DYN.LIMITER_LINK_GROUP", _f(0.0), ALL)
    for k, vals in enumerate(saved):
        for i in range(3):
            dev.pset(f"DYN.{LIM[i]}", vals[i], k)
    for k, vals in enumerate(saved):
        dev.pset("DYN.LIMITER_LINK_GROUP", vals[3], k)


# ---- Limiter ----

@test("paged_dyn")
def limiter_get_parity(dev, profile, chk):
    """Each former LIMITER selector reaches its new command with identical bytes, on every output."""
    _page(dev)
    n = profile.num_output_channels
    for k in range(n):
        for i, name in enumerate(LIM):
            chk.eq(dev.pget(f"DYN.{name}", k), dev.raw_get(OP.LIMITER, 4, (k << 8) | i), f"{name} output {k}")
    chk.eq(dev.pget("DYN.LIMITER_STATUS"), dev.raw_get(OP.LIMITER, 4, 0x81), "LIMITER_STATUS")
    meters = dev.pget("DYN.LIMITER_METERS", length=64)
    chk.eq(len(meters), 2 * n, "LIMITER_METERS is one u16 per output")
    chk.eq(len(dev.raw_get(OP.LIMITER, 2 * n, 0x80)), len(meters), "legacy meter length")
    status = dev.pget("DYN.LIMITER_STATUS")
    chk.eq(list(status[1:]), [32, 16, n], "lookahead, block, output count")


@test("paged_dyn", mutating=True)
def limiter_set_roundtrip(dev, profile, chk):
    """Limiter SETs round-trip through both GETs, clamp as before, 0xFF writes every output."""
    _page(dev)
    n = profile.num_output_channels
    saved = _lim_save(dev, profile)
    try:
        dev.pset("DYN.LIMITER_LINK_GROUP", _f(0.0), ALL)
        cases = [("LIMITER_THRESHOLD", 1, -7.5, -7.5), ("LIMITER_THRESHOLD", 1, 5.0, 0.0),
                 ("LIMITER_THRESHOLD", 1, -99.0, -30.0), ("LIMITER_RELEASE", 1, 250.0, 250.0),
                 ("LIMITER_RELEASE", 1, 1.0, 10.0), ("LIMITER_LINK_GROUP", 1, 2.4, 2.0),
                 ("LIMITER_LINK_GROUP", 1, 9.0, 4.0), ("LIMITER_ENABLE", 0, 3.0, 1.0),
                 ("LIMITER_ENABLE", 0, 0.0, 0.0)]
        for name, k, v, want in cases:
            dev.pset(f"DYN.{name}", _f(v), k)
            chk.eq(_f32(dev.pget(f"DYN.{name}", k)), want, f"{name} {v} on output {k}")
            idx = LIM.index(name)
            chk.eq(_f32(dev.raw_get(OP.LIMITER, 4, (k << 8) | idx)), want, f"{name} {v} legacy readback")
        dev.pset("DYN.LIMITER_LINK_GROUP", _f(0.0), 1)
        dev.pset("DYN.LIMITER_THRESHOLD", _f(-4.0), ALL)
        for k in range(n):
            chk.eq(_f32(dev.pget("DYN.LIMITER_THRESHOLD", k)), -4.0, f"0xFF reached output {k}")
        dev.raw_set(OP.LIMITER, _f(-11.0), (0 << 8) | 1)
        chk.eq(_f32(dev.pget("DYN.LIMITER_THRESHOLD", 0)), -11.0, "legacy write, paged read")
        _expect_stall(chk, lambda: dev.pset("DYN.LIMITER_THRESHOLD", NAN, 0), "VC_ERR_VALUE", "NaN")
        chk.eq(_f32(dev.pget("DYN.LIMITER_THRESHOLD", 0)), -11.0, "NaN left the value alone")
    finally:
        _lim_restore(dev, saved)


@test("paged_dyn")
def limiter_errors(dev, profile, chk):
    """0xFF on a GET, an output past the end, a short payload and a SET to a read-only command are refused."""
    _page(dev)
    n = profile.num_output_channels
    for name in LIM:
        _expect_stall(chk, lambda: dev.pget(f"DYN.{name}", ALL), "VC_ERR_ADDRESS", f"{name} GET 0xFF")
        _expect_stall(chk, lambda: dev.pget(f"DYN.{name}", n), "VC_ERR_ADDRESS", f"{name} GET past the end")
        _expect_stall(chk, lambda: dev.pset(f"DYN.{name}", _f(0.0), n), "VC_ERR_ADDRESS", f"{name} SET past the end")
        _expect_stall(chk, lambda: dev.pset(f"DYN.{name}", b"\x00\x00", 0), "VC_ERR_LENGTH", f"{name} 2-byte SET")
    for name in ("LIMITER_METERS", "LIMITER_STATUS"):
        _expect_stall(chk, lambda: dev.pset(f"DYN.{name}", bytes(4)), "VC_ERR_DIRECTION", f"SET {name}")
        _expect_stall(chk, lambda: dev.pget(f"DYN.{name}", 1, 64), "VC_ERR_ADDRESS", f"{name} at wValue 1")


# ---- Leveller and loudness ----

# (name, legacy GET, legacy SET, size, [(sent, expected)])
SCALARS = [
    ("LEVELLER_ENABLE", "GET_LEVELLER_ENABLE", "SET_LEVELLER_ENABLE", 1,
     [(b"\x01", b"\x01"), (b"\xFF", b"\x01"), (b"\x00", b"\x00")]),
    ("LEVELLER_AMOUNT", "GET_LEVELLER_AMOUNT", "SET_LEVELLER_AMOUNT", 4,
     [(_f(40.0), _f(40.0)), (_f(150.0), _f(100.0)), (_f(-5.0), _f(0.0))]),
    ("LEVELLER_SPEED", "GET_LEVELLER_SPEED", "SET_LEVELLER_SPEED", 1,
     [(b"\x02", b"\x02"), (b"\x00", b"\x00")]),
    ("LEVELLER_MAX_GAIN", "GET_LEVELLER_MAX_GAIN", "SET_LEVELLER_MAX_GAIN", 4,
     [(_f(12.0), _f(12.0)), (_f(99.0), _f(35.0)), (_f(-1.0), _f(0.0))]),
    ("LEVELLER_LOOKAHEAD", "GET_LEVELLER_LOOKAHEAD", "SET_LEVELLER_LOOKAHEAD", 1,
     [(b"\x01", b"\x01"), (b"\x09", b"\x01"), (b"\x00", b"\x00")]),
    ("LEVELLER_GATE", "GET_LEVELLER_GATE", "SET_LEVELLER_GATE", 4,
     [(_f(-50.0), _f(-50.0)), (_f(-200.0), _f(-96.0)), (_f(3.0), _f(0.0))]),
    ("LEVELLER_MASKS", "GET_LEVELLER_MASKS", "SET_LEVELLER_MASKS", 2,
     [(b"\x03\x0C", b"\x03\x0C"), (b"\xFF\x00", b"\xFF\x00")]),
    ("LOUDNESS_ENABLE", "GET_LOUDNESS", "SET_LOUDNESS", 1,
     [(b"\x01", b"\x01"), (b"\x80", b"\x01"), (b"\x00", b"\x00")]),
    ("LOUDNESS_REF", "GET_LOUDNESS_REF", "SET_LOUDNESS_REF", 4,
     [(_f(75.0), _f(75.0)), (_f(10.0), _f(40.0)), (_f(150.0), _f(100.0))]),
    ("LOUDNESS_INTENSITY", "GET_LOUDNESS_INTENSITY", "SET_LOUDNESS_INTENSITY", 4,
     [(_f(50.0), _f(50.0)), (_f(-1.0), _f(0.0)), (_f(500.0), _f(200.0))]),
    ("LOUDNESS_MASK", "GET_LOUDNESS_MASK", "SET_LOUDNESS_MASK", 2,
     [(b"\x34\x01", b"\x34\x01"), (b"\x00\x00", b"\x00\x00")]),
]


@test("paged_dyn")
def scalar_get_parity(dev, profile, chk):
    """Every leveller and loudness GET equals its legacy opcode; wValue other than 0 is VC_ERR_ADDRESS."""
    _page(dev)
    for name, get_op, _, size, _ in SCALARS:
        chk.eq(dev.pget(f"DYN.{name}"), dev.raw_get(getattr(OP, get_op), size), name)
        _expect_stall(chk, lambda: dev.pget(f"DYN.{name}", 1), "VC_ERR_ADDRESS", f"{name} at wValue 1")


@test("paged_dyn", mutating=True)
def scalar_set_roundtrip(dev, profile, chk):
    """Leveller and loudness SETs clamp as before and read back through both GETs; legacy writes read back too."""
    _page(dev)
    saved = {name: dev.pget(f"DYN.{name}") for name, *_ in SCALARS}
    try:
        for name, get_op, set_op, size, cases in SCALARS:
            for sent, want in cases:
                dev.pset(f"DYN.{name}", sent)
                chk.eq(dev.pget(f"DYN.{name}"), want, f"{name} {sent.hex()}")
                chk.eq(dev.raw_get(getattr(OP, get_op), size), want, f"{name} {sent.hex()} legacy readback")
            dev.raw_set(getattr(OP, set_op), cases[0][0])
            chk.eq(dev.pget(f"DYN.{name}"), cases[0][1], f"{name} legacy write, paged read")
            _expect_stall(chk, lambda: dev.pset(f"DYN.{name}", bytes(size + 1)), "VC_ERR_LENGTH",
                          f"{name} {size + 1}-byte SET")
        dev.pset("DYN.LEVELLER_SPEED", b"\x01")
        _expect_stall(chk, lambda: dev.pset("DYN.LEVELLER_SPEED", b"\x03"), "VC_ERR_VALUE", "speed 3")
        chk.eq(dev.pget("DYN.LEVELLER_SPEED"), b"\x01", "bad speed left the value alone")
    finally:
        for name, raw in saved.items():
            dev.pset(f"DYN.{name}", raw)
