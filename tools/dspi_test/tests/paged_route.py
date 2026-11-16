"""
Native paged tests for the ROUTE page (plan section 5.5): matrix, per-output controls, channel delay
and name, preamp, master and user volume. Flash-writing tests need --allow-flash.
"""

import math
import struct
import time

from ..device import PagedStall
from ..framework import Skip, test
from .paged_sys import _expect_stall, _paged

ROUTE = 0x05
NAN = struct.pack("<I", 0x7FC00000)
INF = struct.pack("<f", math.inf)


def _page(dev):
    _paged(dev)
    if ROUTE not in dev.paged_commands():
        raise Skip("ROUTE page not implemented on this firmware")


def _f32(b):
    return struct.unpack("<f", b)[0]


def _finite(b):
    return math.isfinite(_f32(b))


# id(dev) -> whether this run saw the device refuse a NaN ROUTE.OUTPUT_GAIN.
_FINITE_RULE = {}


def finite_rule(dev, profile, chk):
    """Prove once per run that NaN floats are refused before any test sends more of them.
    Firmware without the finite-value rule stores NaN here, so no further non-finite value is sent to it."""
    key = id(dev)
    if key not in _FINITE_RULE:
        o = profile.num_output_channels - 1
        before = dev.pget("ROUTE.OUTPUT_GAIN", o)
        code = "accepted"
        for _ in range(3):
            try:
                dev.pset("ROUTE.OUTPUT_GAIN", NAN, o)
                break
            except PagedStall as s:
                code = s.result_name or "STALL, LAST_RESULT unreadable"
                if s.result is not None:
                    break
        after = dev.pget("ROUTE.OUTPUT_GAIN", o)
        if after != before:
            dev.pset("ROUTE.OUTPUT_GAIN", before, o)
        _FINITE_RULE[key] = (code == "VC_ERR_VALUE" and after == before, code, after.hex())
    proven, code, after = _FINITE_RULE[key]
    chk.ok(proven, f"NaN to ROUTE.OUTPUT_GAIN was not refused ({code}, read back {after}): this firmware lacks "
                   "the finite-value rule, so no further non-finite values are sent in this run")
    return proven


def nonfinite_refused(chk, get, put, cases, what):
    """Each (payload, label) in `cases` must STALL with VC_ERR_VALUE and leave get() unchanged."""
    before = get()
    try:
        for payload, label in cases:
            _expect_stall(chk, lambda: put(payload), "VC_ERR_VALUE", f"{what} {label}")
            chk.eq(get(), before, f"{what} {label} left the value alone")
    finally:
        if get() != before:
            put(before)


def _poll(fn, want, timeout=1.0):
    deadline = time.monotonic() + timeout
    got = fn()
    while got != want and time.monotonic() < deadline:
        time.sleep(0.05)
        got = fn()
    return got


# ---- Matrix ----

@test("paged_route")
def matrix_get_all(dev, profile, chk):
    """Every MATRIX_ROUTE crosspoint is 8 bytes: flags 0 or 1, reserved zero, finite gain."""
    _page(dev)
    bad = 0
    for i in range(profile.num_input_channels):
        for o in range(profile.num_output_channels):
            r = dev.pget("ROUTE.MATRIX_ROUTE", (i << 8) | o)
            ok = len(r) == 8 and r[0] in (0, 1) and r[1] in (0, 1) and r[2:4] == b"\0\0" and _finite(r[4:8])
            if not ok:
                bad += 1
                if bad <= 4:
                    chk.ok(False, f"[{i}][{o}] malformed crosspoint {r.hex()}")
    chk.eq(bad, 0, "malformed crosspoints")


@test("paged_route", mutating=True)
def matrix_set_roundtrip(dev, profile, chk):
    """MATRIX_ROUTE SET reads back exactly; reserved bytes are ignored; a later write replaces the whole point."""
    _page(dev)
    ni, no = profile.num_input_channels, profile.num_output_channels
    points = [(0, 1), (ni - 1, no - 1)]
    saved = {p: dev.pget("ROUTE.MATRIX_ROUTE", (p[0] << 8) | p[1]) for p in points}
    nbrs = {(i, (o + 1) % no) for i, o in points} - set(points)
    nbr_saved = {p: dev.pget("ROUTE.MATRIX_ROUTE", (p[0] << 8) | p[1]) for p in nbrs}
    try:
        for k, (i, o) in enumerate(points):
            a = (i << 8) | o
            pl = struct.pack("<BBHf", 1, 1, 0, -6.5 - k)
            dev.pset("ROUTE.MATRIX_ROUTE", pl, a)
            chk.eq(dev.pget("ROUTE.MATRIX_ROUTE", a), pl, f"[{i}][{o}] readback")
            dev.pset("ROUTE.MATRIX_ROUTE", struct.pack("<BBHf", 0, 0, 0xFFFF, 1.0), a)
            chk.eq(dev.pget("ROUTE.MATRIX_ROUTE", a), struct.pack("<BBHf", 0, 0, 0, 1.0), "reserved ignored, read 0")
            dev.pset("ROUTE.MATRIX_ROUTE", struct.pack("<BBHf", 1, 0, 0, 2.5), a)
            chk.eq(dev.pget("ROUTE.MATRIX_ROUTE", a), struct.pack("<BBHf", 1, 0, 0, 2.5), "second write")
        for (i, o), raw in nbr_saved.items():
            chk.eq(dev.pget("ROUTE.MATRIX_ROUTE", (i << 8) | o), raw, f"neighbouring crosspoint [{i}][{o}] untouched")
    finally:
        for (i, o), raw in saved.items():
            dev.pset("ROUTE.MATRIX_ROUTE", raw, (i << 8) | o)


@test("paged_route")
def matrix_errors(dev, profile, chk):
    """Bad input or output is VC_ERR_ADDRESS both ways; a payload other than 8 bytes is VC_ERR_LENGTH."""
    _page(dev)
    ni, no = profile.num_input_channels, profile.num_output_channels
    pl = struct.pack("<BBHf", 1, 0, 0, 0.0)
    for a, what in (((ni << 8), "input past the end"), (no, "output past the end")):
        _expect_stall(chk, lambda: dev.pget("ROUTE.MATRIX_ROUTE", a), "VC_ERR_ADDRESS", f"GET {what}")
        _expect_stall(chk, lambda: dev.pset("ROUTE.MATRIX_ROUTE", pl, a), "VC_ERR_ADDRESS", f"SET {what}")
    for n in (0, 7, 9):
        _expect_stall(chk, lambda: dev.pset("ROUTE.MATRIX_ROUTE", bytes(n), 0), "VC_ERR_LENGTH", f"{n}-byte SET")


# ---- Per-output controls ----

OUT_CMDS = [("OUTPUT_ENABLE", 1), ("OUTPUT_GAIN", 4), ("OUTPUT_MUTE", 1), ("OUTPUT_DELAY", 4),
            ("OUTPUT_ENABLE_CONFLICT", 1)]


@test("paged_route")
def output_gets_all(dev, profile, chk):
    """Per-output GETs have their sizes and ranges on every output, OUTPUT_DELAY equals the channel's delay."""
    _page(dev)
    no, n_in = profile.num_output_channels, profile.num_input_channels
    for name, n in OUT_CMDS:
        for o in range(no):
            r = dev.pget(f"ROUTE.{name}", o)
            if not chk.eq(len(r), n, f"{name} output {o} length"):
                continue
            if n == 1:
                chk.member(r[0], (0, 1), f"{name} output {o}")
            else:
                chk.ok(_finite(r), f"{name} output {o} finite: {r.hex()}")
        _expect_stall(chk, lambda: dev.pget(f"ROUTE.{name}", no), "VC_ERR_ADDRESS", f"{name} output {no}")
    for o in range(no):
        chk.eq(dev.pget("ROUTE.OUTPUT_DELAY", o), dev.pget("ROUTE.CHANNEL_DELAY", n_in + o),
               f"OUTPUT_DELAY {o} == CHANNEL_DELAY {n_in + o}")
        chk.ok(_f32(dev.pget("ROUTE.OUTPUT_DELAY", o)) >= 0.0, f"output {o} delay not negative")


@test("paged_route", mutating=True)
def output_gain_mute_delay_roundtrip(dev, profile, chk):
    """OUTPUT_GAIN, MUTE and DELAY round-trip; negative delay clamps to 0; an output delay also sets its channel's."""
    _page(dev)
    o = profile.num_output_channels - 1
    n_in = profile.num_input_channels
    saved = {n: dev.pget(f"ROUTE.{n}", o) for n in ("OUTPUT_GAIN", "OUTPUT_MUTE", "OUTPUT_DELAY")}
    saved_ch = dev.pget("ROUTE.CHANNEL_DELAY", n_in + o)
    try:
        cases = [("OUTPUT_GAIN", struct.pack("<f", -4.25), None),
                 ("OUTPUT_GAIN", struct.pack("<f", 20.0), None),
                 ("OUTPUT_MUTE", b"\x01", None),
                 ("OUTPUT_MUTE", b"\x05", None),
                 ("OUTPUT_MUTE", b"\x00", None),
                 ("OUTPUT_DELAY", struct.pack("<f", 3.5), None),
                 ("OUTPUT_DELAY", struct.pack("<f", -2.0), struct.pack("<f", 0.0))]
        for name, pl, want in cases:
            want = want or pl
            dev.pset(f"ROUTE.{name}", pl, o)
            chk.eq(dev.pget(f"ROUTE.{name}", o), want, f"{name} {pl.hex()}")
        dev.pset("ROUTE.OUTPUT_DELAY", struct.pack("<f", 1.25), o)
        chk.eq(dev.pget("ROUTE.CHANNEL_DELAY", n_in + o), struct.pack("<f", 1.25), "output delay mirrors channel delay")
    finally:
        for n, raw in saved.items():
            dev.pset(f"ROUTE.{n}", raw, o)
        dev.pset("ROUTE.CHANNEL_DELAY", saved_ch, n_in + o)


@test("paged_route", mutating=True)
def output_enable_roundtrip_and_conflict(dev, profile, chk):
    """OUTPUT_ENABLE round-trips; enabling PDM while an EQ-worker output runs is VC_ERR_STATE and changes nothing."""
    _page(dev)
    no = profile.num_output_channels
    pdm, eq_out = no - 1, 2
    saved = [dev.pget("ROUTE.OUTPUT_ENABLE", o) for o in range(no)]
    try:
        for v, want in ((b"\x00", b"\x00"), (b"\x07", b"\x01")):
            dev.pset("ROUTE.OUTPUT_ENABLE", v, 0)
            chk.eq(dev.pget("ROUTE.OUTPUT_ENABLE", 0), want, f"enable {v.hex()}")
        dev.pset("ROUTE.OUTPUT_ENABLE", b"\x00", 0)
        chk.eq(dev.pget("ROUTE.OUTPUT_ENABLE", 0), b"\x00", "disable again")

        dev.pset("ROUTE.OUTPUT_ENABLE", b"\x00", pdm)
        dev.pset("ROUTE.OUTPUT_ENABLE", b"\x01", eq_out)
        chk.eq(dev.pget("ROUTE.OUTPUT_ENABLE_CONFLICT", pdm), b"\x01", "PDM predicted to conflict")
        _expect_stall(chk, lambda: dev.pset("ROUTE.OUTPUT_ENABLE", b"\x01", pdm), "VC_ERR_STATE", "enable PDM")
        chk.eq(dev.pget("ROUTE.OUTPUT_ENABLE", pdm), b"\x00", "PDM still disabled")
        dev.pset("ROUTE.OUTPUT_ENABLE", b"\x00", pdm)   # disabling is always allowed
        chk.eq(dev.pget("ROUTE.OUTPUT_ENABLE", pdm), b"\x00", "disable PDM accepted")
    finally:
        # Disable first so no restore step collides with the core 1 interlock.
        for o, raw in enumerate(saved):
            if raw == b"\x00":
                dev.pset("ROUTE.OUTPUT_ENABLE", raw, o)
        for o, raw in enumerate(saved):
            if raw != b"\x00":
                dev.pset("ROUTE.OUTPUT_ENABLE", raw, o)


# ---- Channel delay and name ----

@test("paged_route", mutating=True)
def channel_delay(dev, profile, chk):
    """CHANNEL_DELAY is a finite non-negative float on every channel, round-trips, clamps negatives, refuses past the end."""
    _page(dev)
    n = profile.num_channels
    for ch in range(n):
        r = dev.pget("ROUTE.CHANNEL_DELAY", ch)
        chk.ok(len(r) == 4 and _finite(r) and _f32(r) >= 0.0, f"ch{ch} delay {r.hex()}")
    _expect_stall(chk, lambda: dev.pget("ROUTE.CHANNEL_DELAY", n), "VC_ERR_ADDRESS", "GET past the end")
    _expect_stall(chk, lambda: dev.pset("ROUTE.CHANNEL_DELAY", bytes(4), n), "VC_ERR_ADDRESS", "SET past the end")
    _expect_stall(chk, lambda: dev.pset("ROUTE.CHANNEL_DELAY", bytes(3), 0), "VC_ERR_LENGTH", "3-byte SET")
    ch = 1
    orig = dev.pget("ROUTE.CHANNEL_DELAY", ch)
    try:
        for v, want in ((7.5, 7.5), (-1.0, 0.0), (1000.0, 1000.0)):
            dev.pset("ROUTE.CHANNEL_DELAY", struct.pack("<f", v), ch)
            chk.eq(_f32(dev.pget("ROUTE.CHANNEL_DELAY", ch)), want, f"{v} ms")
    finally:
        dev.pset("ROUTE.CHANNEL_DELAY", orig, ch)


@test("paged_route", mutating=True)
def channel_name(dev, profile, chk):
    """CHANNEL_NAME is 32 NUL-terminated bytes per channel, takes 1-32 bytes and keeps at most 31."""
    _page(dev)
    n = profile.num_channels
    for ch in range(n):
        r = dev.pget("ROUTE.CHANNEL_NAME", ch)
        chk.ok(len(r) == 32 and b"\0" in r, f"ch{ch} name {r!r}")
    _expect_stall(chk, lambda: dev.pget("ROUTE.CHANNEL_NAME", n), "VC_ERR_ADDRESS", "GET past the end")
    _expect_stall(chk, lambda: dev.pset("ROUTE.CHANNEL_NAME", b"x", n), "VC_ERR_ADDRESS", "SET past the end")
    _expect_stall(chk, lambda: dev.pset("ROUTE.CHANNEL_NAME", b"", 0), "VC_ERR_LENGTH", "empty SET")
    _expect_stall(chk, lambda: dev.pset("ROUTE.CHANNEL_NAME", bytes(33), 0), "VC_ERR_LENGTH", "33-byte SET")
    ch = n - 1
    orig = dev.pget("ROUTE.CHANNEL_NAME", ch)
    try:
        dev.pset("ROUTE.CHANNEL_NAME", b"Woofer", ch)
        want = b"Woofer".ljust(32, b"\x00")
        chk.eq(dev.pget("ROUTE.CHANNEL_NAME", ch), want, "short name")
        long = bytes(range(0x41, 0x41 + 32))
        dev.pset("ROUTE.CHANNEL_NAME", long, ch)
        chk.eq(dev.pget("ROUTE.CHANNEL_NAME", ch), long[:31] + b"\x00", "32 bytes keeps 31")
        dev.pset("ROUTE.CHANNEL_NAME", b"Tw", ch)
        chk.eq(dev.pget("ROUTE.CHANNEL_NAME", ch), b"Tw".ljust(32, b"\x00"), "a shorter name clears the old tail")
    finally:
        dev.pset("ROUTE.CHANNEL_NAME", orig.rstrip(b"\x00") or b"\x00", ch)


# ---- Preamp ----

@test("paged_route", mutating=True)
def preamp(dev, profile, chk):
    """PREAMP is a finite float per input channel, round-trips independently, refuses non-finite and outputs."""
    _page(dev)
    ni = profile.num_input_channels
    saved = [dev.pget("ROUTE.PREAMP", ch) for ch in range(ni)]
    for ch in range(ni):
        chk.ok(len(saved[ch]) == 4 and _finite(saved[ch]), f"input {ch} preamp {saved[ch].hex()}")
    _expect_stall(chk, lambda: dev.pget("ROUTE.PREAMP", ni), "VC_ERR_ADDRESS", "GET past the inputs")
    _expect_stall(chk, lambda: dev.pset("ROUTE.PREAMP", bytes(4), ni), "VC_ERR_ADDRESS", "SET past the inputs")
    try:
        for ch in range(ni):
            dev.pset("ROUTE.PREAMP", struct.pack("<f", ch - 3.0), ch)
        for ch in range(ni):
            chk.eq(_f32(dev.pget("ROUTE.PREAMP", ch)), ch - 3.0, f"input {ch} independent")
        dev.pset("ROUTE.PREAMP", struct.pack("<f", 48.0), 0)
        chk.eq(_f32(dev.pget("ROUTE.PREAMP", 0)), 48.0, "no clamp")
        for bad, what in ((NAN, "NaN"), (INF, "infinity")):
            _expect_stall(chk, lambda: dev.pset("ROUTE.PREAMP", bad, 0), "VC_ERR_VALUE", what)
            chk.eq(_f32(dev.pget("ROUTE.PREAMP", 0)), 48.0, f"{what} left the value alone")
    finally:
        for ch, raw in enumerate(saved):
            dev.pset("ROUTE.PREAMP", raw, ch)


# ---- Master volume ----

@test("paged_route", mutating=True)
def master_volume(dev, profile, chk):
    """MASTER_VOLUME reads inside -128..0, clamps writes to that range, refuses non-finite."""
    _page(dev)
    orig = dev.pget("ROUTE.MASTER_VOLUME")
    chk.in_range(_f32(orig), -128.0, 0.0, "current master volume")
    _expect_stall(chk, lambda: dev.pget("ROUTE.MASTER_VOLUME", 1), "VC_ERR_ADDRESS", "wValue 1")
    try:
        for v, want in ((-12.5, -12.5), (-128.0, -128.0), (10.0, 0.0), (-200.0, -128.0)):
            dev.pset("ROUTE.MASTER_VOLUME", struct.pack("<f", v))
            chk.eq(_f32(dev.pget("ROUTE.MASTER_VOLUME")), want, f"{v} dB")
        dev.pset("ROUTE.MASTER_VOLUME", struct.pack("<f", -30.0))
        _expect_stall(chk, lambda: dev.pset("ROUTE.MASTER_VOLUME", NAN), "VC_ERR_VALUE", "NaN")
        chk.eq(_f32(dev.pget("ROUTE.MASTER_VOLUME")), -30.0, "NaN left the value alone")
        _expect_stall(chk, lambda: dev.pset("ROUTE.MASTER_VOLUME", bytes(4), 1), "VC_ERR_ADDRESS", "SET wValue 1")
    finally:
        dev.pset("ROUTE.MASTER_VOLUME", orig)


@test("paged_route")
def master_volume_reads(dev, profile, chk):
    """MASTER_VOLUME_MODE equals STORE.DIRECTORY byte 6, MASTER_VOLUME_SAVED is in range; SAVE has no GET."""
    _page(dev)
    mode = dev.pget("ROUTE.MASTER_VOLUME_MODE")
    chk.member(mode, (b"\x00", b"\x01"), "mode")
    chk.eq(mode, dev.pget("STORE.DIRECTORY", length=7)[6:7], "mode == STORE.DIRECTORY byte 6")
    saved = dev.pget("ROUTE.MASTER_VOLUME_SAVED")
    chk.ok(len(saved) == 4 and _finite(saved) and -128.0 <= _f32(saved) <= 0.0, f"saved volume {saved.hex()}")
    _expect_stall(chk, lambda: dev.pget("ROUTE.MASTER_VOLUME_SAVE", length=1), "VC_ERR_DIRECTION", "GET of SAVE")
    _expect_stall(chk, lambda: dev.pset("ROUTE.MASTER_VOLUME_SAVED", bytes(4)), "VC_ERR_DIRECTION", "SET of SAVED")
    _expect_stall(chk, lambda: dev.pset("ROUTE.MASTER_VOLUME_SAVE", b"\x00"), "VC_ERR_LENGTH", "SAVE with payload")
    _expect_stall(chk, lambda: dev.pset("ROUTE.MASTER_VOLUME_MODE", b""), "VC_ERR_LENGTH", "empty MODE SET")


@test("paged_route", mutating=True, flash=3)
def master_volume_mode_roundtrip(dev, profile, chk):
    """MASTER_VOLUME_MODE round-trips 0 and 1, stores 0 for anything else (deferred directory write)."""
    _page(dev)
    orig = dev.pget("ROUTE.MASTER_VOLUME_MODE")
    try:
        for v, want in ((1, 1), (5, 0), (1, 1)):
            dev.pset("ROUTE.MASTER_VOLUME_MODE", bytes([v]))
            got = _poll(lambda: dev.pget("ROUTE.MASTER_VOLUME_MODE"), bytes([want]))
            chk.eq(got, bytes([want]), f"mode {v}")
            chk.eq(dev.pget("STORE.DIRECTORY", length=7)[6], want, f"mode {v} in STORE.DIRECTORY")
    finally:
        dev.pset("ROUTE.MASTER_VOLUME_MODE", orig)
        _poll(lambda: dev.pget("ROUTE.MASTER_VOLUME_MODE"), orig)


@test("paged_route", mutating=True, flash=2)
def master_volume_save(dev, profile, chk):
    """MASTER_VOLUME_SAVE stores the live master volume; MASTER_VOLUME_SAVED reads it back."""
    _page(dev)
    orig_saved = dev.pget("ROUTE.MASTER_VOLUME_SAVED")
    orig_live = dev.pget("ROUTE.MASTER_VOLUME")
    try:
        dev.pset("ROUTE.MASTER_VOLUME", struct.pack("<f", -18.0))
        dev.pset("ROUTE.MASTER_VOLUME_SAVE")
        want = struct.pack("<f", -18.0)
        chk.eq(_poll(lambda: dev.pget("ROUTE.MASTER_VOLUME_SAVED"), want), want, "saved after SAVE")
        chk.eq(dev.pget("ROUTE.MASTER_VOLUME"), want, "SAVE leaves the live volume alone")
    finally:
        dev.pset("ROUTE.MASTER_VOLUME", orig_saved)
        dev.pset("ROUTE.MASTER_VOLUME_SAVE")
        _poll(lambda: dev.pget("ROUTE.MASTER_VOLUME_SAVED"), orig_saved)
        dev.pset("ROUTE.MASTER_VOLUME", orig_live)


# ---- User volume and mute ----

@test("paged_route", mutating=True)
def user_volume_and_mute(dev, profile, chk):
    """USER_VOLUME clamps to -60..0 in 1/256 dB and refuses non-finite; USER_MUTE is non-zero mutes."""
    _page(dev)
    lg = dev.pget("INPUT.LG_SOUND_SYNC_ENABLE", length=1)
    vol, mute = dev.pget("ROUTE.USER_VOLUME"), dev.pget("ROUTE.USER_MUTE")
    chk.ok(len(vol) == 4 and _finite(vol) and -60.0 <= _f32(vol) <= 0.0, f"current user volume {vol.hex()}")
    chk.member(mute, (b"\x00", b"\x01"), "current user mute")
    try:
        dev.pset("INPUT.LG_SOUND_SYNC_ENABLE", b"\x00")
        for v, want in ((-10.0, -10.0), (6.0, 0.0), (-100.0, -60.0), (-1.3, round(-1.3 * 256) / 256)):
            dev.pset("ROUTE.USER_VOLUME", struct.pack("<f", v))
            chk.approx(_f32(dev.pget("ROUTE.USER_VOLUME")), want, 1e-6, f"{v} dB")
        _expect_stall(chk, lambda: dev.pset("ROUTE.USER_VOLUME", NAN), "VC_ERR_VALUE", "NaN")
        for v, want in ((1, 1), (0, 0), (0xFF, 1)):
            dev.pset("ROUTE.USER_MUTE", bytes([v]))
            chk.eq(dev.pget("ROUTE.USER_MUTE"), bytes([want]), f"mute 0x{v:02X}")
    finally:
        dev.pset("ROUTE.USER_VOLUME", vol)
        dev.pset("ROUTE.USER_MUTE", mute)
        dev.pset("INPUT.LG_SOUND_SYNC_ENABLE", lg)


@test("paged_route", mutating=True)
def nonfinite_floats_refused(dev, profile, chk):
    """NaN and infinity are VC_ERR_VALUE on matrix gain, output gain and delay, channel delay; nothing changes."""
    _page(dev)
    if not finite_rule(dev, profile, chk):
        return
    n_in, no = profile.num_input_channels, profile.num_output_channels
    both = [(NAN, "NaN"), (INF, "infinity")]
    a = (n_in - 1) << 8 | (no - 1)
    nonfinite_refused(chk, lambda: dev.pget("ROUTE.MATRIX_ROUTE", a),
                      lambda pl: dev.pset("ROUTE.MATRIX_ROUTE", pl, a),
                      [(struct.pack("<BBH", 1, 0, 0) + v, k) for v, k in both], "MATRIX_ROUTE gain")
    for name, addr in (("OUTPUT_GAIN", no - 1), ("OUTPUT_DELAY", no - 1),
                       ("CHANNEL_DELAY", 0), ("CHANNEL_DELAY", n_in + no - 1)):
        nonfinite_refused(chk, lambda: dev.pget(f"ROUTE.{name}", addr),
                          lambda pl: dev.pset(f"ROUTE.{name}", pl, addr), both, f"{name} {addr}")


@test("paged_route")
def wrong_direction(dev, profile, chk):
    """A SET to OUTPUT_ENABLE_CONFLICT is VC_ERR_DIRECTION."""
    _page(dev)
    _expect_stall(chk, lambda: dev.pset("ROUTE.OUTPUT_ENABLE_CONFLICT", b"\x01", 0), "VC_ERR_DIRECTION",
                  "SET to a GET-only command")
