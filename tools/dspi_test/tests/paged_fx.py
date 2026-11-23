"""
Native paged tests for the FX page (plan section 5.8): crossfeed, subharmonic synth, psychoacoustic
bass, tube, the spatializer and the RP2350 upmixer. Every effect is switched off, with its output mask cleared, while
its enable bit is exercised, and every test puts back what it changed. The upmixer enable is never toggled.
"""

import math
import struct

from ..device import Stall
from ..framework import test
from ..helpers import expect_stall, finite_rule, nonfinite_refused, page_matches_registry, require_page

FX = 0x08


def _u8(v):
    return bytes([v & 0xFF])


def _u16(v):
    return struct.pack("<H", v & 0xFFFF)


def _f32(v):
    return struct.pack("<f", v)


ENC = {"bool": _u8, "u8": _u8, "u16": _u16, "f32": _f32}

# name, kind, (value a, value b), [(written, read back)]
SCALARS = [
    ("CROSSFEED_ENABLE", "bool", (1, 0), [(7, 1)]),
    ("CROSSFEED_PRESET", "u8", (2, 3), []),
    ("CROSSFEED_FREQ", "f32", (900.0, 1250.0),
     [(100.0, 500.0), (5000.0, 2000.0)]),
    ("CROSSFEED_FEED", "f32", (4.5, 9.0),
     [(-3.0, 0.0), (40.0, 15.0)]),
    ("CROSSFEED_ITD", "bool", (1, 0), [(0x80, 1)]),
    ("CROSSFEED_OUTPUTS", "u8", (1, 2), []),
    ("SUBHARM_ENABLE", "bool", (1, 0), [(5, 1)]),
    ("SUBHARM_LOW", "f32", (-12.0, -6.5),
     [(-99.0, -30.0), (99.0, 12.0)]),
    ("SUBHARM_HIGH", "f32", (-10.0, 3.0), [(50.0, 12.0)]),
    ("SUBHARM_TOP", "f32", (-20.0, -1.5), [(-31.0, -30.0)]),
    ("SUBHARM_BOOST", "f32", (2.0, 4.5),
     [(-1.0, 0.0), (7.0, 6.0)]),
    ("SUBHARM_MASK", "u16", (0x0005, 0x0102), []),
    ("SUBHARM_SELECT", "u8", (1, 2), [(9, 2)]),
    ("SUBHARM_DEPTH", "f32", (25.0, 60.0),
     [(-5.0, 0.0), (150.0, 100.0)]),
    ("SUBHARM_HOLD", "f32", (120.0, 300.0),
     [(10.0, 50.0), (900.0, 400.0)]),
    ("SUBHARM_CEILING", "f32", (-12.0, -3.0),
     [(-80.0, -40.0), (6.0, 0.0)]),
    ("SUBHARM_LINK", "bool", (1, 0), [(2, 1)]),
    ("SUBHARM_SOLO", "bool", (1, 0), [(3, 1)]),
    ("PSYBASS_ENABLE", "bool", (1, 0), [(9, 1)]),
    ("PSYBASS_CUTOFF", "f32", (80.0, 150.0),
     [(5.0, 30.0), (900.0, 300.0)]),
    ("PSYBASS_HARMONICS", "f32", (-6.0, 3.0),
     [(-50.0, -24.0), (20.0, 12.0)]),
    ("PSYBASS_DRIVE", "f32", (6.0, 12.0), [(30.0, 18.0)]),
    ("PSYBASS_CHARACTER", "f32", (30.0, 70.0),
     [(-1.0, 0.0), (101.0, 100.0)]),
    ("PSYBASS_ORIGINAL", "f32", (-12.0, -3.0),
     [(-70.0, -60.0), (1.0, 0.0)]),
    ("PSYBASS_MASK", "u16", (0x0003, 0x0040), []),
    ("SPATIAL_ENABLE", "bool", (1, 0), [(4, 1)]),
    ("SPATIAL_MODE", "u8", (1, 0), []),
    ("SPATIAL_PAIRS", "u8", (1, 2), []),
    ("SPATIAL_WIDTH", "f32", (150.0, 80.0), [(-5.0, 0.0), (250.0, 200.0)]),
    ("SPATIAL_BASS_WIDTH", "f32", (175.0, 40.0), [(-1.0, 0.0), (300.0, 200.0)]),
    ("SPATIAL_SHELF", "f32", (300.0, 1200.0), [(20.0, 100.0), (5000.0, 1500.0)]),
    ("SPATIAL_SPAN", "f32", (15.0, 25.0), [(1.0, 5.0), (90.0, 30.0)]),
    ("SPATIAL_ATTEN", "f32", (6.0, 12.0), [(0.0, 2.5), (40.0, 20.0)]),
    ("SPATIAL_MID", "f32", (50.0, 0.0), [(-10.0, 0.0), (150.0, 100.0)]),
    ("SPATIAL_TRIM", "f32", (-3.0, -6.5), [(5.0, 0.0), (-30.0, -12.0)]),
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
# Params 3-12 sit as float32 at UPMIX_CONFIG offset 4 + 4 * (index - 3); presence is byte 3 in 0.5 dB.
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
    "spatial": ("SPATIAL_ENABLE", "SPATIAL_PAIRS", _u8(0)),
}


def _name(n):
    return f"FX.{n}"


def _get(dev, n, length=None):
    return dev.pget(_name(n), length=length)


def _set(dev, n, data):
    return dev.pset(_name(n), data)


def _upmix_cfg_disagreements(cfg, params):
    """Fields where UPMIX_CONFIG and the per-parameter UPMIX_* reads describe different values."""
    vals = [struct.unpack("<f", params[n])[0] for n, _v in UPMIX]
    bad = [UPMIX[i][0] for i in range(3) if cfg[i] != int(vals[i])]
    bad += [UPMIX[i][0] for i in range(3, 13) if struct.unpack_from("<f", cfg, 4 + 4 * (i - 3))[0] != vals[i]]
    if struct.unpack_from("<b", cfg, 3)[0] != round(vals[13] * 2):
        bad.append("UPMIX_PRESENCE")
    return bad


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


@test("paged_fx")
def fx_page_matches_registry(dev, profile, chk):
    """FX.PAGE_COMMANDS and COMMAND_INFO match the registry; the upmixer block exists only on RP2350."""
    require_page(dev, FX)
    page_matches_registry(dev, chk, FX, _rp2350(profile))


@test("paged_fx")
def fx_reads_shape(dev, profile, chk):
    """Every FX GET has its table size and a well-formed value; UPMIX_CONFIG agrees with the UPMIX_* parameters."""
    require_page(dev, FX)
    for n, kind, _v, _c in SCALARS:
        r = _get(dev, n)
        if not chk.eq(len(r), len(ENC[kind](0)), f"FX.{n} length"):
            continue
        if kind == "bool":
            chk.member(r[0], (0, 1), f"FX.{n}")
        elif kind == "f32":
            chk.ok(math.isfinite(struct.unpack("<f", r)[0]), f"FX.{n} finite: {r.hex()}")
    head = _get(dev, "SUBHARM_HEADROOM")
    chk.ok(len(head) == 4 and math.isfinite(struct.unpack("<f", head)[0]), f"FX.SUBHARM_HEADROOM: {head.hex()}")
    chk.eq(len(_get(dev, "SUBHARM_METER", 64)), 2 * profile.num_output_channels, "SUBHARM_METER: one u16 per output")
    for n, _v, _c in TUBE:
        r = _get(dev, n)
        chk.ok(len(r) == 4 and math.isfinite(struct.unpack("<f", r)[0]), f"FX.{n}: {r.hex()}")
    if _rp2350(profile):
        cfg = _get(dev, "UPMIX_CONFIG")
        chk.eq(len(cfg), 44, "UPMIX_CONFIG length")
        chk.eq(len(_get(dev, "UPMIX_CONFIG_EXT")), 36, "UPMIX_CONFIG_EXT length")
        params = _save(dev, [n for n, _v in UPMIX])
        chk.eq(_upmix_cfg_disagreements(cfg, params), [], "UPMIX_CONFIG fields == UPMIX_* parameters")
        st = _get(dev, "UPMIX_STATUS")
        chk.eq(len(st), 32, "UPMIX_STATUS length")
        chk.member(st[0], (0, 1), "UPMIX_STATUS active")
        chk.in_range(st[1], 0, 6, "UPMIX_STATUS parked reason")
        chk.eq(st[0] == 1, st[1] == 0, f"active exactly when not parked: {st[:2].hex()}")
        chk.eq(st[12:16], bytes(4), "UPMIX_STATUS reserved")
        chk.member(st[16], (0, 1, 0xFF), "UPMIX_STATUS active_profile")
        chk.eq(st[16] == 0xFF, st[1] in (1, 2, 3, 4, 6), f"a profile is named unless parked: {st[:2].hex()} {st[16]:#x}")
        chk.eq(st[17] & 0x80, 0, "UPMIX_STATUS clip bits within the seven generated channels")


@test("paged_fx", mutating=True)
def fx_scalars_roundtrip(dev, profile, chk):
    """Each scalar: two SETs read back exactly, clamps hold, and everything is restored."""
    require_page(dev, FX)
    saved = _save(dev, SCALAR)
    guard = _guard_all(dev)
    try:
        enables = [e for e, _m, _z in GUARDS.values() if e in SCALAR]
        masks = [m for _e, m, _z in GUARDS.values() if m in SCALAR]
        order = masks + [n for n in SCALAR if n not in enables + masks] + enables
        for n in order:
            _n, kind, (a, b), clamps = SCALAR[n]
            enc = ENC[kind]
            chk.no_stall(lambda: _set(dev, n, enc(a)), f"FX.{n} SET")
            chk.eq(_get(dev, n), enc(a), f"FX.{n} GET after SET {a}")
            _set(dev, n, enc(b))
            chk.eq(_get(dev, n), enc(b), f"FX.{n} GET after SET {b}")
            for w, r in clamps:
                chk.no_stall(lambda: _set(dev, n, enc(w)), f"FX.{n} SET {w}")
                chk.eq(_get(dev, n), enc(r), f"FX.{n} {w} reads back {r}")
            _set(dev, n, _parked(n, saved))
    finally:
        _restore(dev, saved, [n for n in SCALAR if n not in guard])
        _unguard(dev, guard)
    chk.eq(_save(dev, SCALAR), saved, "every FX scalar restored")


@test("paged_fx", mutating=True)
def fx_tube_roundtrip(dev, profile, chk):
    """Each FX.TUBE_* command holds its own value through two SETs, clamps, and is restored."""
    require_page(dev, FX)
    names = [n for n, _v, _c in TUBE]
    saved = _save(dev, names)
    guard = _guard_all(dev)
    try:
        for n, (a, b), clamps in TUBE:
            chk.no_stall(lambda: _set(dev, n, _f32(a)), f"FX.{n} SET")
            chk.eq(_get(dev, n), _f32(a), f"FX.{n} after SET {a}")
            _set(dev, n, _f32(b))
            chk.eq(_get(dev, n), _f32(b), f"FX.{n} after SET {b}")
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
    require_page(dev, FX)
    before = _get(dev, "TUBE_DRIVE")
    expect_stall(chk, lambda: _set(dev, "TUBE_DRIVE", _f32(math.nan)), "VC_ERR_VALUE", "NaN drive")
    chk.eq(_get(dev, "TUBE_DRIVE"), before, "drive unchanged")


@test("paged_fx", mutating=True)
def fx_nonfinite_refused(dev, profile, chk):
    """Every tube parameter refuses NaN. Crossfeed, subharm, psybass and upmix floats refuse NaN and infinity."""
    require_page(dev, FX)
    if not finite_rule(dev, profile, chk):
        return
    nan, inf = _f32(math.nan), _f32(math.inf)
    for n, _v, _c in TUBE:
        nonfinite_refused(chk, lambda: _get(dev, n), lambda pl: _set(dev, n, pl), [(nan, "NaN")], f"FX.{n}")
    both = [(nan, "NaN"), (inf, "infinity")]
    for n in [s[0] for s in SCALARS if s[1] == "f32"]:
        nonfinite_refused(chk, lambda: _get(dev, n), lambda pl: _set(dev, n, pl), both, f"FX.{n}")
    if not _rp2350(profile):
        return
    for n, v in UPMIX:
        if v is not None:
            nonfinite_refused(chk, lambda: _get(dev, n), lambda pl: _set(dev, n, pl), both, f"FX.{n}")
    nonfinite_refused(chk, lambda: _get(dev, "UPMIX_BACK_OFFSET"), lambda pl: _set(dev, "UPMIX_BACK_OFFSET", pl),
                      both, "FX.UPMIX_BACK_OFFSET")
    cfg = _get(dev, "UPMIX_CONFIG")
    cases = []
    for off, field in ((4, "strength"), (40, "decorr")):
        for raw, label in both:
            cases.append((cfg[:off] + raw + cfg[off + 4:], f"{label} {field}"))
    nonfinite_refused(chk, lambda: _get(dev, "UPMIX_CONFIG"), lambda pl: _set(dev, "UPMIX_CONFIG", pl),
                      cases, "FX.UPMIX_CONFIG")


@test("paged_fx", mutating=True)
def fx_value_and_length_errors(dev, profile, chk):
    """Bad values, lengths, addresses and directions STALL with the right VC_* code and change nothing."""
    require_page(dev, FX)
    preset = _get(dev, "CROSSFEED_PRESET")
    for bad in (4, 0xFF):
        expect_stall(chk, lambda: _set(dev, "CROSSFEED_PRESET", _u8(bad)), "VC_ERR_VALUE", f"preset {bad}")
    chk.eq(_get(dev, "CROSSFEED_PRESET"), preset, "preset unchanged by rejected values")
    for n, payload in (("SUBHARM_LOW", b"\x00\x00\x00"), ("SUBHARM_LOW", bytes(5)),
                       ("SUBHARM_ENABLE", b""), ("SUBHARM_ENABLE", b"\x00\x00"),
                       ("PSYBASS_MASK", b"\x00"), ("TUBE_DRIVE", b"\x00\x00"),
                       ("CROSSFEED_FREQ", bytes(8))):
        expect_stall(chk, lambda: _set(dev, n, payload), "VC_ERR_LENGTH", f"FX.{n} {len(payload)} bytes")
    for n in ("SUBHARM_HEADROOM", "SUBHARM_METER"):
        expect_stall(chk, lambda: _set(dev, n, bytes(4)), "VC_ERR_DIRECTION", f"SET to FX.{n}")
    expect_stall(chk, lambda: dev.pget("FX.SUBHARM_LOW", addr=1), "VC_ERR_ADDRESS", "GET at wValue 1")
    expect_stall(chk, lambda: dev.pset("FX.SUBHARM_LOW", _f32(-30.0), addr=1), "VC_ERR_ADDRESS",
                 "SET at wValue 1")
    if _rp2350(profile):
        expect_stall(chk, lambda: _set(dev, "UPMIX_STATUS", bytes(16)), "VC_ERR_DIRECTION", "SET UPMIX_STATUS")
        expect_stall(chk, lambda: _set(dev, "UPMIX_CONFIG", bytes(43)), "VC_ERR_LENGTH", "43-byte UPMIX_CONFIG")
        expect_stall(chk, lambda: _set(dev, "UPMIX_CONFIG", bytes(45)), "VC_ERR_LENGTH", "45-byte UPMIX_CONFIG")


@test("paged_fx", mutating=True)
def fx_upmix_route_roundtrip(dev, profile, chk):
    """UPMIX_ROUTE per (profile, channel): exact readback, trim clamp, and reserved-flag, address and mask refusals."""
    require_page(dev, FX)
    if not _rp2350(profile):
        expect_stall(chk, lambda: _get(dev, "UPMIX_ROUTE", 4), "VC_ERR_UNKNOWN", "UPMIX_ROUTE on RP2040")
        return
    addrs = [(p << 8) | g for p in range(2) for g in range(7)]
    saved = {a: dev.pget("FX.UPMIX_ROUTE", addr=a, length=4) for a in addrs}
    nout = profile.num_output_channels
    pk = lambda m, t, f: struct.pack("<HbB", m, t, f)
    try:
        for a in addrs:
            v = pk((a * 37) & ((1 << nout) - 1), -((a & 0x3F) % 80), a & 1)
            dev.pset("FX.UPMIX_ROUTE", v, addr=a)
            chk.eq(dev.pget("FX.UPMIX_ROUTE", addr=a, length=4), v, f"UPMIX_ROUTE {a:#06x} readback")
        a = 0x0102
        dev.pset("FX.UPMIX_ROUTE", pk(1, -100, 1), addr=a)
        chk.eq(dev.pget("FX.UPMIX_ROUTE", addr=a, length=4), pk(1, -80, 1), "trim clamps to -40 dB")
        expect_stall(chk, lambda: dev.pset("FX.UPMIX_ROUTE", pk(1, 0, 0x02), addr=a), "VC_ERR_VALUE",
                     "reserved flag bit")
        dev.pset("FX.UPMIX_ROUTE", pk(1, 100, 0), addr=a)
        chk.eq(dev.pget("FX.UPMIX_ROUTE", addr=a, length=4), pk(1, 12, 0), "trim clamps to +6 dB")
        expect_stall(chk, lambda: dev.pset("FX.UPMIX_ROUTE", pk(1 << nout, 0, 0), addr=0), "VC_ERR_VALUE",
                     "mask bit above the last output")
        for bad in (0x0007, 0x0200, 0xFF00):
            expect_stall(chk, lambda b=bad: dev.pget("FX.UPMIX_ROUTE", addr=b, length=4), "VC_ERR_ADDRESS",
                         f"GET UPMIX_ROUTE at {bad:#06x}")
            expect_stall(chk, lambda b=bad: dev.pset("FX.UPMIX_ROUTE", pk(0, 0, 0), addr=b), "VC_ERR_ADDRESS",
                         f"SET UPMIX_ROUTE at {bad:#06x}")
    finally:
        for a, v in saved.items():
            dev.pset("FX.UPMIX_ROUTE", v, addr=a)
    chk.eq({a: dev.pget("FX.UPMIX_ROUTE", addr=a, length=4) for a in addrs}, saved, "routes restored")


# Profile-addressed v2 parameters: name, byte offset in UPMIX_CONFIG_EXT, test value.
UPMIX_V2 = [("UPMIX_SOURCE_A", 0, 4.0), ("UPMIX_SOURCE_B", 1, 5.0),
            ("UPMIX_ACTIVE_COUNTS", 2, 12.0), ("UPMIX_BACK_OFFSET", 4, 6.5)]


@test("paged_fx", mutating=True)
def fx_upmix_profiles(dev, profile, chk):
    """Profile addressing: profile 1 is independent of 0, wValue 2 is refused, the v2 fields round-trip and refuse bad values."""
    require_page(dev, FX)
    if not _rp2350(profile):
        return
    cfgs = {p: dev.pget("FX.UPMIX_CONFIG", addr=p) + dev.pget("FX.UPMIX_CONFIG_EXT", addr=p) for p in (0, 1)}
    names = [n for n, _o, _v in UPMIX_V2] + ["UPMIX_STRENGTH", "UPMIX_PRESENCE"]
    saved = {(n, p): dev.pget(f"FX.{n}", addr=p) for n in names for p in (0, 1)}
    try:
        for n, off, v in UPMIX_V2:
            dev.pset(f"FX.{n}", _f32(v), addr=1)
            chk.eq(dev.pget(f"FX.{n}", addr=1), _f32(v), f"profile 1 FX.{n} after SET {v}")
            got = dev.pget("FX.UPMIX_CONFIG_EXT", addr=1)
            raw = struct.unpack_from("<f", got, off)[0] if off == 4 else got[off]
            chk.eq(raw, v if off == 4 else int(v), f"UPMIX_CONFIG_EXT byte {off} follows FX.{n}")
            chk.eq(dev.pget(f"FX.{n}", addr=0), saved[(n, 0)], f"profile 0 FX.{n} untouched")
        dev.pset("FX.UPMIX_SOURCE_A", _f32(3.4), addr=1)
        chk.eq(dev.pget("FX.UPMIX_SOURCE_A", addr=1), _f32(3.0), "a source is rounded")
        for n, bad in (("UPMIX_SOURCE_A", 8.0), ("UPMIX_SOURCE_B", -1.0), ("UPMIX_ACTIVE_COUNTS", 16.0)):
            expect_stall(chk, lambda n=n, bad=bad: dev.pset(f"FX.{n}", _f32(bad), addr=1), "VC_ERR_VALUE",
                         f"FX.{n} {bad}")
        dev.pset("FX.UPMIX_STRENGTH", _f32(55.0), addr=1)
        chk.eq(dev.pget("FX.UPMIX_STRENGTH", addr=0), saved[("UPMIX_STRENGTH", 0)], "profile 0 strength untouched")
        for n in ("UPMIX_CONFIG", "UPMIX_STRENGTH", "UPMIX_SOURCE_A"):
            expect_stall(chk, lambda n=n: dev.pget(f"FX.{n}", addr=2), "VC_ERR_ADDRESS", f"GET FX.{n} at profile 2")
        expect_stall(chk, lambda: dev.pset("FX.UPMIX_STRENGTH", _f32(1.0), addr=2), "VC_ERR_ADDRESS",
                     "SET FX.UPMIX_STRENGTH at profile 2")
        expect_stall(chk, lambda: dev.pset("FX.UPMIX_CONFIG", cfgs[1][:44], addr=2), "VC_ERR_ADDRESS",
                     "SET FX.UPMIX_CONFIG at profile 2")
        # UPMIX_CONFIG_EXT: a GET is a valid SET; bad fields are refused whole.
        ext = bytearray(dev.pget("FX.UPMIX_CONFIG_EXT", addr=1))
        struct.pack_into("<HbB", ext, 8 + 4 * 6, 0x80, -6, 1)          # BB to output 7, -3 dB, inverted
        dev.pset("FX.UPMIX_CONFIG_EXT", bytes(ext), addr=1)
        chk.eq(dev.pget("FX.UPMIX_CONFIG_EXT", addr=1), bytes(ext), "UPMIX_CONFIG_EXT round-trips")
        chk.eq(dev.pget("FX.UPMIX_ROUTE", addr=0x106, length=4), struct.pack("<HbB", 0x80, -6, 1),
               "UPMIX_ROUTE sees the routes written through UPMIX_CONFIG_EXT")
        for off, raw, what in ((0, b"\x08", "source 8"), (2, b"\x10", "count bit 4"), (11, b"\x02", "route flag bit 1")):
            bad = bytes(ext[:off]) + raw + bytes(ext[off + 1:])
            expect_stall(chk, lambda b=bad: dev.pset("FX.UPMIX_CONFIG_EXT", b, addr=1), "VC_ERR_VALUE",
                         f"UPMIX_CONFIG_EXT with {what}")
        chk.eq(dev.pget("FX.UPMIX_CONFIG_EXT", addr=1), bytes(ext), "refused SETs changed nothing")
        dev.pset("FX.UPMIX_PRESENCE", _f32(3.3), addr=1)
        dev.pset("FX.UPMIX_CONFIG_EXT", bytes(ext), addr=1)
        chk.eq(dev.pget("FX.UPMIX_PRESENCE", addr=1), _f32(3.3), "an EXT SET leaves presence unquantized")
    finally:
        for p in (0, 1):
            dev.pset("FX.UPMIX_CONFIG_EXT", cfgs[p][44:], addr=p)
        for (n, p), v in saved.items():
            dev.pset(f"FX.{n}", v, addr=p)
    chk.eq({p: dev.pget("FX.UPMIX_CONFIG", addr=p) + dev.pget("FX.UPMIX_CONFIG_EXT", addr=p) for p in (0, 1)},
           cfgs, "both profiles restored")


@test("paged_fx", mutating=True)
def fx_upmix_roundtrip(dev, profile, chk):
    """UPMIX_CONFIG and each UPMIX_* parameter round-trip and stay in step with each other; enable is never changed."""
    require_page(dev, FX)
    if not _rp2350(profile):
        expect_stall(chk, lambda: _get(dev, "UPMIX_CONFIG", 44), "VC_ERR_UNKNOWN", "UPMIX_CONFIG on RP2040")
        return
    names = [n for n, _v in UPMIX]
    cfg = _get(dev, "UPMIX_CONFIG")
    ext = _get(dev, "UPMIX_CONFIG_EXT")
    saved = _save(dev, names)
    try:
        chk.no_stall(lambda: _set(dev, "UPMIX_CONFIG", cfg), "write back the current config")
        chk.eq(_get(dev, "UPMIX_CONFIG_EXT"), ext, "UPMIX_CONFIG SET keeps UPMIX_CONFIG_EXT")
        chk.eq(_get(dev, "UPMIX_CONFIG"), cfg, "UPMIX_CONFIG GET is a valid SET payload")
        mod = bytearray(cfg)
        struct.pack_into("<f", mod, 4, 37.0)            # strength_pct
        _set(dev, "UPMIX_CONFIG", bytes(mod))
        chk.eq(_get(dev, "UPMIX_CONFIG"), bytes(mod), "UPMIX_CONFIG after SET")
        chk.eq(_get(dev, "UPMIX_STRENGTH"), _f32(37.0), "UPMIX_STRENGTH follows the config")
        _set(dev, "UPMIX_CONFIG", cfg)
        chk.eq(_get(dev, "UPMIX_CONFIG"), cfg, "original config written back")
        for n, v in UPMIX:
            if v is None:
                chk.no_stall(lambda: _set(dev, n, saved[n]), f"FX.{n} rewrite of its own value")
                chk.eq(_get(dev, n), saved[n], f"FX.{n} unchanged")
                continue
            _set(dev, n, _f32(v))
            chk.eq(_get(dev, n), _f32(v), f"FX.{n} after SET {v}")
            chk.eq(_upmix_cfg_disagreements(_get(dev, "UPMIX_CONFIG"), _save(dev, names)), [],
                   f"UPMIX_CONFIG follows FX.{n}")
            _set(dev, n, _f32(v + 1.0))
            chk.eq(_get(dev, n), _f32(v + 1.0), f"FX.{n} after SET {v + 1.0}")
            _set(dev, n, saved[n])
    finally:
        try:
            _set(dev, "UPMIX_CONFIG", cfg)
            _restore(dev, saved, names[3:])
        except Stall:
            pass
    chk.eq(_get(dev, "UPMIX_CONFIG"), cfg, "upmix config restored")
    chk.eq(_save(dev, names), saved, "every upmix parameter restored")


SPATIAL_FIELDS = "<BBBBfffffff"     # SpatialConfigPacket, spec 8.2


@test("paged_fx", mutating=True)
def fx_spatial_config(dev, profile, chk):
    """SPATIAL_CONFIG agrees with the SPATIAL_* parameters, round-trips, masks the pair mask and refuses bad modes and NaN."""
    require_page(dev, FX)
    names = [s[0] for s in SCALARS if s[0].startswith("SPATIAL_")]
    cfg = _get(dev, "SPATIAL_CONFIG")
    saved = _save(dev, names)
    if not chk.eq(len(cfg), 32, "SPATIAL_CONFIG length"):
        return
    f = struct.unpack(SPATIAL_FIELDS, cfg)
    chk.eq(f[3], 0, "SPATIAL_CONFIG reserved byte reads 0")
    want = [saved[n][0] for n in names[:3]] + [struct.unpack("<f", saved[n])[0] for n in names[3:]]
    chk.eq(list(f[:3]) + list(f[4:]), want, "SPATIAL_CONFIG fields == SPATIAL_* parameters")
    pmask = (1 << profile.num_spdif) - 1
    try:
        mod = struct.pack(SPATIAL_FIELDS, 0, 1, 0xFF, 0x5A, 110.0, 90.0, 400.0, 22.0, 5.0, 60.0, -2.0)
        chk.no_stall(lambda: _set(dev, "SPATIAL_CONFIG", mod), "SPATIAL_CONFIG SET")
        back = _get(dev, "SPATIAL_CONFIG")
        chk.eq(back, struct.pack(SPATIAL_FIELDS, 0, 1, pmask, 0, 110.0, 90.0, 400.0, 22.0, 5.0, 60.0, -2.0),
               f"config reads back with the mask cut to {pmask:#x} and reserved zero")
        chk.eq(_get(dev, "SPATIAL_SPAN"), _f32(22.0), "SPATIAL_SPAN follows the config")
        clamp = struct.pack(SPATIAL_FIELDS, 0, 0, 1, 0, 999.0, -9.0, 9.0, 99.0, 1.0, 500.0, 6.0)
        _set(dev, "SPATIAL_CONFIG", clamp)
        chk.eq(_get(dev, "SPATIAL_CONFIG"),
               struct.pack(SPATIAL_FIELDS, 0, 0, 1, 0, 200.0, 0.0, 100.0, 30.0, 2.5, 100.0, 0.0),
               "out-of-range config fields clamp")
        _set(dev, "SPATIAL_PAIRS", _u8(0xFF))
        chk.eq(_get(dev, "SPATIAL_PAIRS"), _u8(pmask), f"SPATIAL_PAIRS 0xFF reads back {pmask:#x}")
        before = _get(dev, "SPATIAL_CONFIG")
        expect_stall(chk, lambda: _set(dev, "SPATIAL_MODE", _u8(2)), "VC_ERR_VALUE", "SPATIAL_MODE 2")
        bad = bytearray(before)
        bad[1] = 2
        expect_stall(chk, lambda: _set(dev, "SPATIAL_CONFIG", bytes(bad)), "VC_ERR_VALUE", "SPATIAL_CONFIG mode 2")
        for off, field in ((4, "width"), (16, "span"), (28, "trim")):
            for raw, label in ((_f32(math.nan), "NaN"), (_f32(math.inf), "infinity")):
                bad = before[:off] + raw + before[off + 4:]
                expect_stall(chk, lambda: _set(dev, "SPATIAL_CONFIG", bad), "VC_ERR_VALUE", f"{label} {field}")
        chk.eq(_get(dev, "SPATIAL_CONFIG"), before, "refused SETs changed nothing")
        for n in (31, 33):
            expect_stall(chk, lambda: _set(dev, "SPATIAL_CONFIG", bytes(n)), "VC_ERR_LENGTH", f"{n}-byte SPATIAL_CONFIG")
    finally:
        _set(dev, "SPATIAL_CONFIG", cfg)
    chk.eq(_get(dev, "SPATIAL_CONFIG"), cfg, "spatial config restored")
    chk.eq(_save(dev, names), saved, "every spatial parameter restored")
