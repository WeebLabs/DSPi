"""
Native paged tests for the FX page (plan section 5.8): crossfeed, subharmonic synth, psychoacoustic
bass, tube and the RP2350 upmixer. Every effect is switched off, with its output mask cleared, while
its enable bit is exercised, and every test puts back what it changed. The upmixer enable is never toggled.
"""

import math
import struct

from ..device import OP, Stall
from ..framework import Skip, test
from .paged_sys import _bits, _expect_stall, _paged, _registry

FX = 0x08


def _u8(v):
    return bytes([v & 0xFF])


def _u16(v):
    return struct.pack("<H", v & 0xFFFF)


def _f32(v):
    return struct.pack("<f", v)


ENC = {"bool": _u8, "u8": _u8, "u16": _u16, "f32": _f32}

# name, old GET, old SET, kind, (value a, value b), [(written, read back)]
SCALARS = [
    ("CROSSFEED_ENABLE", OP.GET_CROSSFEED, OP.SET_CROSSFEED, "bool", (1, 0), [(7, 1)]),
    ("CROSSFEED_PRESET", OP.GET_CROSSFEED_PRESET, OP.SET_CROSSFEED_PRESET, "u8", (2, 3), []),
    ("CROSSFEED_FREQ", OP.GET_CROSSFEED_FREQ, OP.SET_CROSSFEED_FREQ, "f32", (900.0, 1250.0),
     [(100.0, 500.0), (5000.0, 2000.0)]),
    ("CROSSFEED_FEED", OP.GET_CROSSFEED_FEED, OP.SET_CROSSFEED_FEED, "f32", (4.5, 9.0),
     [(-3.0, 0.0), (40.0, 15.0)]),
    ("CROSSFEED_ITD", OP.GET_CROSSFEED_ITD, OP.SET_CROSSFEED_ITD, "bool", (1, 0), [(0x80, 1)]),
    ("CROSSFEED_OUTPUTS", OP.GET_CROSSFEED_OUTPUTS, OP.SET_CROSSFEED_OUTPUTS, "u8", (1, 2), []),
    ("SUBHARM_ENABLE", OP.GET_SUBHARM, OP.SET_SUBHARM, "bool", (1, 0), [(5, 1)]),
    ("SUBHARM_LOW", OP.GET_SUBHARM_LOW, OP.SET_SUBHARM_LOW, "f32", (-12.0, -6.5),
     [(-99.0, -30.0), (99.0, 12.0)]),
    ("SUBHARM_HIGH", OP.GET_SUBHARM_HIGH, OP.SET_SUBHARM_HIGH, "f32", (-10.0, 3.0), [(50.0, 12.0)]),
    ("SUBHARM_TOP", OP.GET_SUBHARM_TOP, OP.SET_SUBHARM_TOP, "f32", (-20.0, -1.5), [(-31.0, -30.0)]),
    ("SUBHARM_BOOST", OP.GET_SUBHARM_BOOST, OP.SET_SUBHARM_BOOST, "f32", (2.0, 4.5),
     [(-1.0, 0.0), (7.0, 6.0)]),
    ("SUBHARM_MASK", OP.GET_SUBHARM_MASK, OP.SET_SUBHARM_MASK, "u16", (0x0005, 0x0102), []),
    ("SUBHARM_SELECT", OP.GET_SUBHARM_SELECT, OP.SET_SUBHARM_SELECT, "u8", (1, 2), [(9, 2)]),
    ("SUBHARM_DEPTH", OP.GET_SUBHARM_DEPTH, OP.SET_SUBHARM_DEPTH, "f32", (25.0, 60.0),
     [(-5.0, 0.0), (150.0, 100.0)]),
    ("SUBHARM_HOLD", OP.GET_SUBHARM_HOLD, OP.SET_SUBHARM_HOLD, "f32", (120.0, 300.0),
     [(10.0, 50.0), (900.0, 400.0)]),
    ("SUBHARM_CEILING", OP.GET_SUBHARM_CEILING, OP.SET_SUBHARM_CEILING, "f32", (-12.0, -3.0),
     [(-80.0, -40.0), (6.0, 0.0)]),
    ("SUBHARM_LINK", OP.GET_SUBHARM_LINK, OP.SET_SUBHARM_LINK, "bool", (1, 0), [(2, 1)]),
    ("SUBHARM_SOLO", OP.GET_SUBHARM_SOLO, OP.SET_SUBHARM_SOLO, "bool", (1, 0), [(3, 1)]),
    ("PSYBASS_ENABLE", OP.GET_PSYBASS, OP.SET_PSYBASS, "bool", (1, 0), [(9, 1)]),
    ("PSYBASS_CUTOFF", OP.GET_PSYBASS_CUTOFF, OP.SET_PSYBASS_CUTOFF, "f32", (80.0, 150.0),
     [(5.0, 30.0), (900.0, 300.0)]),
    ("PSYBASS_HARMONICS", OP.GET_PSYBASS_HARMONICS, OP.SET_PSYBASS_HARMONICS, "f32", (-6.0, 3.0),
     [(-50.0, -24.0), (20.0, 12.0)]),
    ("PSYBASS_DRIVE", OP.GET_PSYBASS_DRIVE, OP.SET_PSYBASS_DRIVE, "f32", (6.0, 12.0), [(30.0, 18.0)]),
    ("PSYBASS_CHARACTER", OP.GET_PSYBASS_CHARACTER, OP.SET_PSYBASS_CHARACTER, "f32", (30.0, 70.0),
     [(-1.0, 0.0), (101.0, 100.0)]),
    ("PSYBASS_ORIGINAL", OP.GET_PSYBASS_ORIGINAL, OP.SET_PSYBASS_ORIGINAL, "f32", (-12.0, -3.0),
     [(-70.0, -60.0), (1.0, 0.0)]),
    ("PSYBASS_MASK", OP.GET_PSYBASS_MASK, OP.SET_PSYBASS_MASK, "u16", (0x0003, 0x0040), []),
]
SCALAR = {s[0]: s for s in SCALARS}

# TUBE_PARAM_* order: name, (value a, value b), [(written, read back)]
TUBE = [
    ("TUBE_ENABLE", (1.0, 0.0), [(-3.0, 1.0)]),
    ("TUBE_OUTPUT_MASK", (5.0, 0x30 * 1.0), [(70000.0, 65535.0)]),
    ("TUBE_TYPE", (3.0, 0.0), [(99.0, 16.0)]),
    ("TUBE_DRIVE", (6.0, -4.0), [(-99.0, -30.0), (99.0, 24.0)]),
    ("TUBE_BIAS", (20.0, -35.0), [(500.0, 100.0)]),
    ("TUBE_ASYM", (3.0, -2.0), [(20.0, 12.0)]),
    ("TUBE_HARDNESS", (40.0, 75.0), [(-5.0, 0.0)]),
    ("TUBE_SAG", (10.0, 55.0), [(200.0, 100.0)]),
    ("TUBE_RECTIFIER", (1.0, 2.0), [(9.0, 3.0)]),
    ("TUBE_XFMR_ENABLE", (1.0, 0.0), [(0.5, 1.0)]),
    ("TUBE_XFMR_DAMPING", (4.0, 12.0), [(0.0, 1.0), (40.0, 20.0)]),
    ("TUBE_XFMR_RES", (60.0, 110.0), [(10.0, 30.0), (400.0, 150.0)]),
    ("TUBE_MIX", (50.0, 80.0), [(120.0, 100.0)]),
    ("TUBE_TRIM", (-3.0, 2.5), [(-20.0, -12.0)]),
]
# Restore order: row fields before the type, so a stock type rewrites them to its own row.
TUBE_RESTORE = ["TUBE_OUTPUT_MASK", "TUBE_DRIVE", "TUBE_BIAS", "TUBE_ASYM", "TUBE_HARDNESS",
                "TUBE_SAG", "TUBE_RECTIFIER", "TUBE_XFMR_ENABLE", "TUBE_XFMR_DAMPING",
                "TUBE_XFMR_RES", "TUBE_MIX", "TUBE_TRIM", "TUBE_TYPE", "TUBE_ENABLE"]

# UPMIX_PARAM_* order: name, test value (enable is never written with a new value).
UPMIX = [
    ("UPMIX_ENABLE", None), ("UPMIX_CENTER_MODE", None), ("UPMIX_SURROUND_MODE", None),
    ("UPMIX_STRENGTH", 42.0), ("UPMIX_CENTER_WIDTH", 33.0), ("UPMIX_THRESHOLD", 21.0),
    ("UPMIX_ATTACK", 15.0), ("UPMIX_RELEASE", 250.0), ("UPMIX_DET_HPF", 140.0),
    ("UPMIX_SUR_DELAY", 7.5), ("UPMIX_SUR_HPF", 160.0), ("UPMIX_SUR_LPF", 9000.0),
    ("UPMIX_DECORR", 35.0), ("UPMIX_PRESENCE", 1.5),
]

# family: (enable, mask, zero mask payload)
GUARDS = {
    "crossfeed": ("CROSSFEED_ENABLE", "CROSSFEED_OUTPUTS", _u8(0)),
    "subharm": ("SUBHARM_ENABLE", "SUBHARM_MASK", _u16(0)),
    "psybass": ("PSYBASS_ENABLE", "PSYBASS_MASK", _u16(0)),
    "tube": ("TUBE_ENABLE", "TUBE_OUTPUT_MASK", _f32(0.0)),
}


def _fx(dev):
    _paged(dev)
    if FX not in dev.paged_commands():
        raise Skip("FX page not implemented on this firmware")


def _name(n):
    return f"FX.{n}"


def _get(dev, n, length=None):
    return dev.pget(_name(n), length=length)


def _set(dev, n, data):
    return dev.pset(_name(n), data)


def _rp2350(profile):
    return profile.platform_id == 1


def _save(dev, names):
    return {n: _get(dev, n) for n in names}


def _restore(dev, saved, order):
    for n in order:
        if n in saved:
            try:
                _set(dev, n, saved[n])
            except Stall:
                pass


def _parked(n, saved):
    """The value a command returns to between steps: guarded enables off, guarded masks clear."""
    for en, m, z in GUARDS.values():
        if n == en:
            return _f32(0.0) if en == "TUBE_ENABLE" else _u8(0)
        if n == m:
            return z
    return saved[n]


def _guard_all(dev):
    """Switch every effect off and clear its mask. Returns what to restore."""
    names = [n for e, m, _ in GUARDS.values() for n in (e, m)] + ["SUBHARM_SOLO", "SUBHARM_LINK"]
    saved = _save(dev, names)
    for en, _m, _z in GUARDS.values():
        _set(dev, en, _f32(0.0) if en == "TUBE_ENABLE" else _u8(0))
    for _e, m, z in GUARDS.values():
        _set(dev, m, z)
    return saved


def _unguard(dev, saved):
    masks = [m for _e, m, _z in GUARDS.values()]
    enables = [e for e, _m, _z in GUARDS.values()]
    _restore(dev, saved, ["SUBHARM_SOLO", "SUBHARM_LINK"] + masks + enables)


def page_matches_registry(dev, chk, page, rp2350):
    """PAGE_COMMANDS and COMMAND_INFO of `page` match the registry; build-gated rows exist only on RP2350."""
    pg = next((p for p in _registry(dev)["pages"] if p["page"] == page), None)
    if pg is None:
        raise Skip(f"registry has no page 0x{page:02X}")
    bm = dev.pget("SYS.PAGE_COMMANDS", addr=page, length=64)
    got_get, got_set = _bits(bm[:32]), _bits(bm[32:])
    for c in pg["commands"]:
        name = f"{pg['mnemonic']}.{c['name']}"
        present = not c.get("build") or rp2350
        chk.eq(c["cmd"] in got_get, bool(c["get"]) and present, f"GET bit of {name}")
        chk.eq(c["cmd"] in got_set, bool(c["set"]) and present, f"SET bit of {name}")
        if not present:
            continue
        r = dev.pget("SYS.COMMAND_INFO", addr=(page << 8) | c["cmd"], length=8)
        _flags, get_len, smin, smax, _ak, has = struct.unpack("<HHBBBB", r)
        want = 0xFFFF if c["get_len"] == "var" else (c["get_len"] or 0)
        chk.eq((get_len, smin, smax, has & 3), (want, c["set_min"], c["set_max"],
               (1 if c["get"] else 0) | (2 if c["set"] else 0)), f"{name} COMMAND_INFO")
    chk.eq(sorted((got_get | got_set) - {c["cmd"] for c in pg["commands"]}), [], "unlisted bits")


@test("paged_fx")
def fx_page_matches_registry(dev, profile, chk):
    """FX.PAGE_COMMANDS and COMMAND_INFO match the registry; the upmixer block exists only on RP2350."""
    _fx(dev)
    page_matches_registry(dev, chk, FX, _rp2350(profile))


@test("paged_fx")
def fx_reads_match_legacy(dev, profile, chk):
    """Every FX GET is byte-identical to its 1.1.6 opcode (meters compared by shape)."""
    _fx(dev)
    for n, og, _os, kind, _v, _c in SCALARS:
        chk.eq(_get(dev, n), dev.raw_get(og, len(ENC[kind](0))), f"FX.{n} == old 0x{og:02X}")
    chk.eq(_get(dev, "SUBHARM_HEADROOM"), dev.raw_get(OP.GET_SUBHARM_HEADROOM, 4), "FX.SUBHARM_HEADROOM")
    new, old = _get(dev, "SUBHARM_METER", 64), dev.raw_get(OP.GET_SUBHARM_METER, 64)
    chk.eq(len(new), 2 * profile.num_output_channels, "SUBHARM_METER: one u16 per output")
    chk.eq(len(new), len(old), "SUBHARM_METER length matches old 0x1F")
    for i, (n, _v, _c) in enumerate(TUBE):
        chk.eq(_get(dev, n), dev.raw_get(OP.GET_TUBE_PARAM, 4, wvalue=i), f"FX.{n} == old 0x3F index {i}")
    if _rp2350(profile):
        chk.eq(_get(dev, "UPMIX_CONFIG"), dev.raw_get(OP.UPMIX_GET_CONFIG, 44), "FX.UPMIX_CONFIG")
        for i, (n, _v) in enumerate(UPMIX):
            chk.eq(_get(dev, n), dev.raw_get(OP.UPMIX_GET_PARAM, 4, wvalue=i), f"FX.{n} == old 0x4D index {i}")
        new, old = _get(dev, "UPMIX_STATUS"), dev.raw_get(OP.UPMIX_GET_STATUS, 16)
        chk.eq(new[:2], old[:2], "FX.UPMIX_STATUS active and parked reason")
        chk.eq(len(new), 16, "UPMIX_STATUS length")


@test("paged_fx", mutating=True)
def fx_scalars_roundtrip_both_ways(dev, profile, chk):
    """Each scalar: new SET reads back through new and old GET, old SET reads back through new GET, clamps hold."""
    _fx(dev)
    saved = _save(dev, SCALAR)
    guard = _guard_all(dev)
    try:
        enables = [e for e, _m, _z in GUARDS.values() if e in SCALAR]
        masks = [m for _e, m, _z in GUARDS.values() if m in SCALAR]
        order = masks + [n for n in SCALAR if n not in enables + masks] + enables
        for n in order:
            _n, og, os_, kind, (a, b), clamps = SCALAR[n]
            enc = ENC[kind]
            chk.no_stall(lambda: _set(dev, n, enc(a)), f"FX.{n} SET")
            chk.eq(_get(dev, n), enc(a), f"FX.{n} new GET after new SET")
            chk.eq(dev.raw_get(og, len(enc(a))), enc(a), f"old 0x{og:02X} after new SET of FX.{n}")
            dev.raw_set(os_, enc(b))
            chk.eq(_get(dev, n), enc(b), f"FX.{n} new GET after old SET 0x{os_:02X}")
            for w, r in clamps:
                chk.no_stall(lambda: _set(dev, n, enc(w)), f"FX.{n} SET {w}")
                chk.eq(_get(dev, n), enc(r), f"FX.{n} {w} reads back {r}")
            _set(dev, n, _parked(n, saved))
    finally:
        _restore(dev, saved, [n for n in SCALAR if n not in guard])
        _unguard(dev, guard)
    chk.eq(_save(dev, SCALAR), saved, "every FX scalar restored")


@test("paged_fx", mutating=True)
def fx_tube_split_roundtrip(dev, profile, chk):
    """Each old 0x3E/0x3F index reaches its own FX.TUBE_* command, both directions, with the same clamps."""
    _fx(dev)
    names = [n for n, _v, _c in TUBE]
    saved = _save(dev, names)
    guard = _guard_all(dev)
    try:
        for i, (n, (a, b), clamps) in enumerate(TUBE):
            chk.no_stall(lambda: _set(dev, n, _f32(a)), f"FX.{n} SET")
            chk.eq(_get(dev, n), _f32(a), f"FX.{n} after new SET")
            chk.eq(dev.raw_get(OP.GET_TUBE_PARAM, 4, wvalue=i), _f32(a), f"old 0x3F[{i}] after new SET of FX.{n}")
            dev.raw_set(OP.SET_TUBE_PARAM, _f32(b), wvalue=i)
            chk.eq(_get(dev, n), _f32(b), f"FX.{n} after old 0x3E[{i}]")
            for w, r in clamps:
                _set(dev, n, _f32(w))
                chk.eq(_get(dev, n), _f32(r), f"FX.{n} {w} reads back {r}")
            if n == "TUBE_TYPE":
                _restore(dev, saved, TUBE_RESTORE[1:-1])
            else:
                _set(dev, n, _parked(n, saved))
    finally:
        _restore(dev, saved, TUBE_RESTORE[:-1])
        _unguard(dev, guard)
    chk.eq(_save(dev, names), saved, "every tube parameter restored")


@test("paged_fx", mutating=True)
def fx_tube_nan_rejected(dev, profile, chk):
    """FX.TUBE_DRIVE refuses NaN with VC_ERR_VALUE and keeps its value (1.1.6 ignored it silently)."""
    _fx(dev)
    before = _get(dev, "TUBE_DRIVE")
    _expect_stall(chk, lambda: _set(dev, "TUBE_DRIVE", _f32(math.nan)), "VC_ERR_VALUE", "NaN drive")
    chk.eq(_get(dev, "TUBE_DRIVE"), before, "drive unchanged")


@test("paged_fx", mutating=True)
def fx_value_and_length_errors(dev, profile, chk):
    """Bad values, lengths, addresses and directions STALL with the right VC_* code and change nothing."""
    _fx(dev)
    preset = _get(dev, "CROSSFEED_PRESET")
    for bad in (4, 0xFF):
        _expect_stall(chk, lambda: _set(dev, "CROSSFEED_PRESET", _u8(bad)), "VC_ERR_VALUE", f"preset {bad}")
    chk.eq(_get(dev, "CROSSFEED_PRESET"), preset, "preset unchanged by rejected values")
    for n, payload in (("SUBHARM_LOW", b"\x00\x00\x00"), ("SUBHARM_LOW", bytes(5)),
                       ("SUBHARM_ENABLE", b""), ("SUBHARM_ENABLE", b"\x00\x00"),
                       ("PSYBASS_MASK", b"\x00"), ("TUBE_DRIVE", b"\x00\x00"),
                       ("CROSSFEED_FREQ", bytes(8))):
        _expect_stall(chk, lambda: _set(dev, n, payload), "VC_ERR_LENGTH", f"FX.{n} {len(payload)} bytes")
    for n in ("SUBHARM_HEADROOM", "SUBHARM_METER"):
        _expect_stall(chk, lambda: _set(dev, n, bytes(4)), "VC_ERR_DIRECTION", f"SET to FX.{n}")
    _expect_stall(chk, lambda: dev.pget("FX.SUBHARM_LOW", addr=1), "VC_ERR_ADDRESS", "GET at wValue 1")
    _expect_stall(chk, lambda: dev.pset("FX.SUBHARM_LOW", _f32(-30.0), addr=1), "VC_ERR_ADDRESS",
                  "SET at wValue 1")
    if _rp2350(profile):
        _expect_stall(chk, lambda: _set(dev, "UPMIX_STATUS", bytes(16)), "VC_ERR_DIRECTION", "SET UPMIX_STATUS")
        _expect_stall(chk, lambda: _set(dev, "UPMIX_CONFIG", bytes(43)), "VC_ERR_LENGTH", "43-byte UPMIX_CONFIG")


@test("paged_fx", mutating=True)
def fx_upmix_roundtrip(dev, profile, chk):
    """UPMIX_CONFIG and each old 0x4C/0x4D index round-trip through the new commands; enable is never changed."""
    _fx(dev)
    if not _rp2350(profile):
        _expect_stall(chk, lambda: _get(dev, "UPMIX_CONFIG", 44), "VC_ERR_UNKNOWN", "UPMIX_CONFIG on RP2040")
        return
    names = [n for n, _v in UPMIX]
    cfg = _get(dev, "UPMIX_CONFIG")
    saved = _save(dev, names)
    try:
        chk.no_stall(lambda: _set(dev, "UPMIX_CONFIG", cfg), "write back the current config")
        chk.eq(_get(dev, "UPMIX_CONFIG"), cfg, "UPMIX_CONFIG GET is a valid SET payload")
        mod = bytearray(cfg)
        struct.pack_into("<f", mod, 4, 37.0)            # strength_pct
        _set(dev, "UPMIX_CONFIG", bytes(mod))
        chk.eq(dev.raw_get(OP.UPMIX_GET_CONFIG, 44), bytes(mod), "old 0x4B after new UPMIX_CONFIG")
        chk.eq(_get(dev, "UPMIX_STRENGTH"), _f32(37.0), "UPMIX_STRENGTH follows the config")
        dev.raw_set(OP.UPMIX_SET_CONFIG, cfg)
        chk.eq(_get(dev, "UPMIX_CONFIG"), cfg, "new GET after old 0x4A")
        for i, (n, v) in enumerate(UPMIX):
            if v is None:
                chk.no_stall(lambda: _set(dev, n, saved[n]), f"FX.{n} rewrite of its own value")
                chk.eq(_get(dev, n), saved[n], f"FX.{n} unchanged")
                continue
            _set(dev, n, _f32(v))
            chk.eq(dev.raw_get(OP.UPMIX_GET_PARAM, 4, wvalue=i), _f32(v), f"old 0x4D[{i}] after FX.{n}")
            dev.raw_set(OP.UPMIX_SET_PARAM, _f32(v + 1.0), wvalue=i)
            chk.eq(_get(dev, n), _f32(v + 1.0), f"FX.{n} after old 0x4C[{i}]")
            _set(dev, n, saved[n])
    finally:
        try:
            _set(dev, "UPMIX_CONFIG", cfg)
            _restore(dev, saved, names[3:])
        except Stall:
            pass
    chk.eq(_get(dev, "UPMIX_CONFIG"), cfg, "upmix config restored")
    chk.eq(_save(dev, names), saved, "every upmix parameter restored")
