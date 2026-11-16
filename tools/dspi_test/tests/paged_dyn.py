"""
Native paged tests for the DYN page (plan section 5.7): limiter, leveller, loudness.
The limiter keeps its float32 payloads for every parameter.
"""

import math
import struct

from ..framework import Skip, test
from .paged_route import finite_rule, nonfinite_refused
from .paged_sys import _expect_stall, _paged

DYN = 0x07
NAN = struct.pack("<I", 0x7FC00000)
INF = struct.pack("<f", math.inf)
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

LIM_RANGES = {"LIMITER_ENABLE": (0.0, 1.0), "LIMITER_THRESHOLD": (-30.0, 0.0),
              "LIMITER_RELEASE": (10.0, 1000.0), "LIMITER_LINK_GROUP": (0.0, 4.0)}


@test("paged_dyn")
def limiter_gets_all(dev, profile, chk):
    """Every limiter parameter is a finite float32 inside its clamp range on every output; status and meters fit."""
    _page(dev)
    n = profile.num_output_channels
    for k in range(n):
        for name in LIM:
            r = dev.pget(f"DYN.{name}", k)
            lo, hi = LIM_RANGES[name]
            chk.ok(len(r) == 4 and math.isfinite(_f32(r)) and lo <= _f32(r) <= hi, f"{name} output {k}: {r.hex()}")
        chk.member(_f32(dev.pget("DYN.LIMITER_ENABLE", k)), (0.0, 1.0), f"enable output {k} is 0 or 1")
        chk.eq(_f32(dev.pget("DYN.LIMITER_LINK_GROUP", k)) % 1.0, 0.0, f"link group output {k} is whole")
    meters = dev.pget("DYN.LIMITER_METERS", length=64)
    chk.eq(len(meters), 2 * n, "LIMITER_METERS is one u16 per output")
    status = dev.pget("DYN.LIMITER_STATUS")
    chk.eq(len(status), 4, "LIMITER_STATUS length")
    chk.member(status[0], (0, 1), "engaged flag")
    chk.eq(list(status[1:]), [32, 16, n], "lookahead, block, output count")


@test("paged_dyn", mutating=True)
def limiter_set_roundtrip(dev, profile, chk):
    """Limiter SETs round-trip, clamp to their ranges, 0xFF writes every output, NaN is refused."""
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
        dev.pset("DYN.LIMITER_LINK_GROUP", _f(0.0), 1)
        dev.pset("DYN.LIMITER_THRESHOLD", _f(-4.0), ALL)
        for k in range(n):
            chk.eq(_f32(dev.pget("DYN.LIMITER_THRESHOLD", k)), -4.0, f"0xFF reached output {k}")
        dev.pset("DYN.LIMITER_THRESHOLD", _f(-11.0), 0)
        chk.eq(_f32(dev.pget("DYN.LIMITER_THRESHOLD", 0)), -11.0, "single-output write after 0xFF")
        chk.eq(_f32(dev.pget("DYN.LIMITER_THRESHOLD", n - 1)), -4.0, "other outputs keep the 0xFF value")
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

# (name, size, (min, max) of a read for floats or allowed values for ints, [(sent, expected)])
RANGES = {
    "LEVELLER_ENABLE": (0, 1), "LEVELLER_AMOUNT": (0.0, 100.0), "LEVELLER_SPEED": (0, 1, 2),
    "LEVELLER_MAX_GAIN": (0.0, 35.0), "LEVELLER_LOOKAHEAD": (0, 1), "LEVELLER_GATE": (-96.0, 0.0),
    "LEVELLER_MASKS": None, "LOUDNESS_ENABLE": (0, 1), "LOUDNESS_REF": (40.0, 100.0),
    "LOUDNESS_INTENSITY": (0.0, 200.0), "LOUDNESS_MASK": None,
}
SCALARS = [
    ("LEVELLER_ENABLE", 1, RANGES["LEVELLER_ENABLE"],
     [(b"\x01", b"\x01"), (b"\xFF", b"\x01"), (b"\x00", b"\x00")]),
    ("LEVELLER_AMOUNT", 4, RANGES["LEVELLER_AMOUNT"],
     [(_f(40.0), _f(40.0)), (_f(150.0), _f(100.0)), (_f(-5.0), _f(0.0))]),
    ("LEVELLER_SPEED", 1, RANGES["LEVELLER_SPEED"],
     [(b"\x02", b"\x02"), (b"\x00", b"\x00")]),
    ("LEVELLER_MAX_GAIN", 4, RANGES["LEVELLER_MAX_GAIN"],
     [(_f(12.0), _f(12.0)), (_f(99.0), _f(35.0)), (_f(-1.0), _f(0.0))]),
    ("LEVELLER_LOOKAHEAD", 1, RANGES["LEVELLER_LOOKAHEAD"],
     [(b"\x01", b"\x01"), (b"\x09", b"\x01"), (b"\x00", b"\x00")]),
    ("LEVELLER_GATE", 4, RANGES["LEVELLER_GATE"],
     [(_f(-50.0), _f(-50.0)), (_f(-200.0), _f(-96.0)), (_f(3.0), _f(0.0))]),
    ("LEVELLER_MASKS", 2, RANGES["LEVELLER_MASKS"],
     [(b"\x03\x0C", b"\x03\x0C"), (b"\xFF\x00", b"\xFF\x00")]),
    ("LOUDNESS_ENABLE", 1, RANGES["LOUDNESS_ENABLE"],
     [(b"\x01", b"\x01"), (b"\x80", b"\x01"), (b"\x00", b"\x00")]),
    ("LOUDNESS_REF", 4, RANGES["LOUDNESS_REF"],
     [(_f(75.0), _f(75.0)), (_f(10.0), _f(40.0)), (_f(150.0), _f(100.0))]),
    ("LOUDNESS_INTENSITY", 4, RANGES["LOUDNESS_INTENSITY"],
     [(_f(50.0), _f(50.0)), (_f(-1.0), _f(0.0)), (_f(500.0), _f(200.0))]),
    ("LOUDNESS_MASK", 2, RANGES["LOUDNESS_MASK"],
     [(b"\x34\x01", b"\x34\x01"), (b"\x00\x00", b"\x00\x00")]),
]


@test("paged_dyn")
def scalar_gets(dev, profile, chk):
    """Every leveller and loudness GET has its size and a value inside its clamp range; wValue 1 is VC_ERR_ADDRESS."""
    _page(dev)
    for name, size, rng, _ in SCALARS:
        r = dev.pget(f"DYN.{name}")
        if not chk.eq(len(r), size, f"{name} length"):
            continue
        if size == 1:
            chk.member(r[0], rng, name)
        elif size == 4:
            chk.ok(math.isfinite(_f32(r)) and rng[0] <= _f32(r) <= rng[1], f"{name} in {rng}: {_f32(r)}")
        _expect_stall(chk, lambda: dev.pget(f"DYN.{name}", 1), "VC_ERR_ADDRESS", f"{name} at wValue 1")


@test("paged_dyn", mutating=True)
def scalar_set_roundtrip(dev, profile, chk):
    """Leveller and loudness SETs clamp and read back; a GET reply is a valid SET payload; bad sizes are refused."""
    _page(dev)
    saved = {name: dev.pget(f"DYN.{name}") for name, *_ in SCALARS}
    try:
        for name, size, _rng, cases in SCALARS:
            for sent, want in cases:
                dev.pset(f"DYN.{name}", sent)
                chk.eq(dev.pget(f"DYN.{name}"), want, f"{name} {sent.hex()}")
            dev.pset(f"DYN.{name}", cases[0][0])
            dev.pset(f"DYN.{name}", dev.pget(f"DYN.{name}"))
            chk.eq(dev.pget(f"DYN.{name}"), cases[0][1], f"{name} GET reply written back unchanged")
            _expect_stall(chk, lambda: dev.pset(f"DYN.{name}", bytes(size + 1)), "VC_ERR_LENGTH",
                          f"{name} {size + 1}-byte SET")
        dev.pset("DYN.LEVELLER_SPEED", b"\x01")
        _expect_stall(chk, lambda: dev.pset("DYN.LEVELLER_SPEED", b"\x03"), "VC_ERR_VALUE", "speed 3")
        chk.eq(dev.pget("DYN.LEVELLER_SPEED"), b"\x01", "bad speed left the value alone")
    finally:
        for name, raw in saved.items():
            dev.pset(f"DYN.{name}", raw)


@test("paged_dyn", mutating=True)
def nonfinite_floats_refused(dev, profile, chk):
    """Leveller and loudness floats refuse NaN and infinity; every limiter parameter refuses NaN."""
    _page(dev)
    if not finite_rule(dev, profile, chk):
        return
    k = profile.num_output_channels - 1
    for name in LIM:
        nonfinite_refused(chk, lambda: dev.pget(f"DYN.{name}", k), lambda pl: dev.pset(f"DYN.{name}", pl, k),
                          [(NAN, "NaN")], f"{name} output {k}")
    for name in ("LEVELLER_AMOUNT", "LEVELLER_MAX_GAIN", "LEVELLER_GATE", "LOUDNESS_REF", "LOUDNESS_INTENSITY"):
        nonfinite_refused(chk, lambda: dev.pget(f"DYN.{name}"), lambda pl: dev.pset(f"DYN.{name}", pl),
                          [(NAN, "NaN"), (INF, "infinity")], name)
