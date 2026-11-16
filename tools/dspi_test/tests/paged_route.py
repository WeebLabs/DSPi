"""
Native paged tests for the ROUTE page (plan section 5.5): matrix, per-output controls, channel delay
and name, preamp, master and user volume. Old opcodes are read with raw_get/raw_set so the A/B
comparison never goes through the translation layer. Flash-writing tests need --allow-flash.
"""

import math
import struct
import time

from ..device import OP
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


def _poll(fn, want, timeout=1.0):
    deadline = time.monotonic() + timeout
    got = fn()
    while got != want and time.monotonic() < deadline:
        time.sleep(0.05)
        got = fn()
    return got


# ---- Matrix ----

@test("paged_route")
def matrix_get_parity_all(dev, profile, chk):
    """MATRIX_ROUTE equals GET_MATRIX_ROUTE on every crosspoint, minus the echoed address."""
    _page(dev)
    bad = 0
    for i in range(profile.num_input_channels):
        for o in range(profile.num_output_channels):
            new = dev.pget("ROUTE.MATRIX_ROUTE", (i << 8) | o)
            old = dev.raw_get(OP.GET_MATRIX_ROUTE, 8, (i << 8) | o)
            if new != old[2:4] + b"\x00\x00" + old[4:8] or old[:2] != bytes([i, o]):
                bad += 1
                if bad <= 4:
                    chk.ok(False, f"[{i}][{o}] paged {new.hex()} legacy {old.hex()}")
    chk.eq(bad, 0, "crosspoint mismatches")


@test("paged_route", mutating=True)
def matrix_set_roundtrip(dev, profile, chk):
    """MATRIX_ROUTE SET reads back through both GETs; a legacy packet reads back through the paged GET."""
    _page(dev)
    ni, no = profile.num_input_channels, profile.num_output_channels
    points = [(0, 1), (ni - 1, no - 1)]
    saved = {p: dev.pget("ROUTE.MATRIX_ROUTE", (p[0] << 8) | p[1]) for p in points}
    try:
        for k, (i, o) in enumerate(points):
            a = (i << 8) | o
            pl = struct.pack("<BBHf", 1, 1, 0, -6.5 - k)
            dev.pset("ROUTE.MATRIX_ROUTE", pl, a)
            chk.eq(dev.pget("ROUTE.MATRIX_ROUTE", a), pl, f"[{i}][{o}] paged readback")
            chk.eq(dev.raw_get(OP.GET_MATRIX_ROUTE, 8, a), bytes([i, o]) + pl[:2] + pl[4:], "legacy readback")
            dev.pset("ROUTE.MATRIX_ROUTE", struct.pack("<BBHf", 0, 0, 0xFFFF, 1.0), a)
            chk.eq(dev.pget("ROUTE.MATRIX_ROUTE", a), struct.pack("<BBHf", 0, 0, 0, 1.0), "reserved ignored, read 0")
            dev.raw_set(OP.SET_MATRIX_ROUTE, struct.pack("<BBBBf", i, o, 1, 0, 2.5))
            chk.eq(dev.pget("ROUTE.MATRIX_ROUTE", a), struct.pack("<BBHf", 1, 0, 0, 2.5), "legacy write, paged read")
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

OUT_TWINS = [("OUTPUT_ENABLE", "GET_OUTPUT_ENABLE", 1), ("OUTPUT_GAIN", "GET_OUTPUT_GAIN", 4),
             ("OUTPUT_MUTE", "GET_OUTPUT_MUTE", 1), ("OUTPUT_DELAY", "GET_OUTPUT_DELAY", 4),
             ("OUTPUT_ENABLE_CONFLICT", "GET_CORE1_CONFLICT", 1)]


@test("paged_route")
def output_get_parity_all(dev, profile, chk):
    """Each per-output GET equals its legacy opcode on every output; past the end is VC_ERR_ADDRESS."""
    _page(dev)
    no = profile.num_output_channels
    for new, old, n in OUT_TWINS:
        for o in range(no):
            chk.eq(dev.pget(f"ROUTE.{new}", o), dev.raw_get(getattr(OP, old), n, o), f"{new} output {o}")
        _expect_stall(chk, lambda: dev.pget(f"ROUTE.{new}", no), "VC_ERR_ADDRESS", f"{new} output {no}")


@test("paged_route", mutating=True)
def output_gain_mute_delay_roundtrip(dev, profile, chk):
    """OUTPUT_GAIN, MUTE and DELAY round-trip through both GETs; negative delay clamps to 0."""
    _page(dev)
    o = profile.num_output_channels - 1
    n_in = profile.num_input_channels
    saved = {n: dev.pget(f"ROUTE.{n}", o) for n in ("OUTPUT_GAIN", "OUTPUT_MUTE", "OUTPUT_DELAY")}
    saved_ch = dev.pget("ROUTE.CHANNEL_DELAY", n_in + o)
    try:
        cases = [("OUTPUT_GAIN", "GET_OUTPUT_GAIN", struct.pack("<f", -4.25), None),
                 ("OUTPUT_GAIN", "GET_OUTPUT_GAIN", struct.pack("<f", 20.0), None),
                 ("OUTPUT_MUTE", "GET_OUTPUT_MUTE", b"\x01", None),
                 ("OUTPUT_MUTE", "GET_OUTPUT_MUTE", b"\x05", None),
                 ("OUTPUT_DELAY", "GET_OUTPUT_DELAY", struct.pack("<f", 3.5), None),
                 ("OUTPUT_DELAY", "GET_OUTPUT_DELAY", struct.pack("<f", -2.0), struct.pack("<f", 0.0))]
        for new, old, pl, want in cases:
            want = want or pl
            dev.pset(f"ROUTE.{new}", pl, o)
            chk.eq(dev.pget(f"ROUTE.{new}", o), want, f"{new} {pl.hex()} paged")
            chk.eq(dev.raw_get(getattr(OP, old), len(want), o), want, f"{new} {pl.hex()} legacy")
        dev.pset("ROUTE.OUTPUT_DELAY", struct.pack("<f", 1.25), o)
        chk.eq(dev.pget("ROUTE.CHANNEL_DELAY", n_in + o), struct.pack("<f", 1.25), "output delay mirrors channel delay")
        dev.raw_set(OP.SET_OUTPUT_GAIN, struct.pack("<f", -1.5), o)
        chk.eq(dev.pget("ROUTE.OUTPUT_GAIN", o), struct.pack("<f", -1.5), "legacy gain write, paged read")
        dev.raw_set(OP.SET_OUTPUT_MUTE, b"\x00", o)
        chk.eq(dev.pget("ROUTE.OUTPUT_MUTE", o), b"\x00", "legacy mute write, paged read")
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
            chk.eq(dev.raw_get(OP.GET_OUTPUT_ENABLE, 1, 0), want, f"legacy readback {v.hex()}")
        dev.raw_set(OP.SET_OUTPUT_ENABLE, b"\x00", 0)
        chk.eq(dev.pget("ROUTE.OUTPUT_ENABLE", 0), b"\x00", "legacy write, paged read")

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
    """CHANNEL_DELAY matches GET_DELAY on every channel, round-trips, clamps negatives, refuses past the end."""
    _page(dev)
    n = profile.num_channels
    for ch in range(n):
        chk.eq(dev.pget("ROUTE.CHANNEL_DELAY", ch), dev.raw_get(OP.GET_DELAY, 4, ch), f"parity ch{ch}")
    _expect_stall(chk, lambda: dev.pget("ROUTE.CHANNEL_DELAY", n), "VC_ERR_ADDRESS", "GET past the end")
    _expect_stall(chk, lambda: dev.pset("ROUTE.CHANNEL_DELAY", bytes(4), n), "VC_ERR_ADDRESS", "SET past the end")
    _expect_stall(chk, lambda: dev.pset("ROUTE.CHANNEL_DELAY", bytes(3), 0), "VC_ERR_LENGTH", "3-byte SET")
    ch = 1
    orig = dev.pget("ROUTE.CHANNEL_DELAY", ch)
    try:
        for v, want in ((7.5, 7.5), (-1.0, 0.0), (1000.0, 1000.0)):
            dev.pset("ROUTE.CHANNEL_DELAY", struct.pack("<f", v), ch)
            chk.eq(_f32(dev.pget("ROUTE.CHANNEL_DELAY", ch)), want, f"{v} ms")
            chk.eq(_f32(dev.raw_get(OP.GET_DELAY, 4, ch)), want, f"{v} ms legacy readback")
        dev.raw_set(OP.SET_DELAY, struct.pack("<f", 2.0), ch)
        chk.eq(_f32(dev.pget("ROUTE.CHANNEL_DELAY", ch)), 2.0, "legacy write, paged read")
    finally:
        dev.pset("ROUTE.CHANNEL_DELAY", orig, ch)


@test("paged_route", mutating=True)
def channel_name(dev, profile, chk):
    """CHANNEL_NAME matches GET_CHANNEL_NAME, takes 1-32 bytes and keeps at most 31."""
    _page(dev)
    n = profile.num_channels
    for ch in range(n):
        chk.eq(dev.pget("ROUTE.CHANNEL_NAME", ch), dev.raw_get(OP.GET_CHANNEL_NAME, 32, ch), f"parity ch{ch}")
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
        chk.eq(dev.raw_get(OP.GET_CHANNEL_NAME, 32, ch), want, "short name legacy readback")
        long = bytes(range(0x41, 0x41 + 32))
        dev.pset("ROUTE.CHANNEL_NAME", long, ch)
        chk.eq(dev.pget("ROUTE.CHANNEL_NAME", ch), long[:31] + b"\x00", "32 bytes keeps 31")
        dev.raw_set(OP.SET_CHANNEL_NAME, b"Legacy", ch)
        chk.eq(dev.pget("ROUTE.CHANNEL_NAME", ch), b"Legacy".ljust(32, b"\x00"), "legacy write, paged read")
    finally:
        dev.pset("ROUTE.CHANNEL_NAME", orig.rstrip(b"\x00") or b"\x00", ch)


# ---- Preamp ----

@test("paged_route", mutating=True)
def preamp(dev, profile, chk):
    """PREAMP matches GET_PREAMP_CH per input channel, round-trips, refuses non-finite and outputs."""
    _page(dev)
    ni = profile.num_input_channels
    saved = [dev.pget("ROUTE.PREAMP", ch) for ch in range(ni)]
    for ch in range(ni):
        chk.eq(saved[ch], dev.raw_get(OP.GET_PREAMP_CH, 4, ch), f"parity input {ch}")
    _expect_stall(chk, lambda: dev.pget("ROUTE.PREAMP", ni), "VC_ERR_ADDRESS", "GET past the inputs")
    _expect_stall(chk, lambda: dev.pset("ROUTE.PREAMP", bytes(4), ni), "VC_ERR_ADDRESS", "SET past the inputs")
    try:
        for ch in range(ni):
            dev.pset("ROUTE.PREAMP", struct.pack("<f", ch - 3.0), ch)
        for ch in range(ni):
            chk.eq(_f32(dev.pget("ROUTE.PREAMP", ch)), ch - 3.0, f"input {ch} independent")
            chk.eq(_f32(dev.raw_get(OP.GET_PREAMP_CH, 4, ch)), ch - 3.0, f"input {ch} legacy readback")
        dev.pset("ROUTE.PREAMP", struct.pack("<f", 48.0), 0)
        chk.eq(_f32(dev.pget("ROUTE.PREAMP", 0)), 48.0, "no clamp")
        for bad, what in ((NAN, "NaN"), (INF, "infinity")):
            _expect_stall(chk, lambda: dev.pset("ROUTE.PREAMP", bad, 0), "VC_ERR_VALUE", what)
            chk.eq(_f32(dev.pget("ROUTE.PREAMP", 0)), 48.0, f"{what} left the value alone")
        dev.raw_set(OP.SET_PREAMP_CH, struct.pack("<f", -2.0), 0)
        chk.eq(_f32(dev.pget("ROUTE.PREAMP", 0)), -2.0, "legacy write, paged read")
    finally:
        for ch, raw in enumerate(saved):
            dev.pset("ROUTE.PREAMP", raw, ch)


# ---- Master volume ----

@test("paged_route", mutating=True)
def master_volume(dev, profile, chk):
    """MASTER_VOLUME matches GET_MASTER_VOLUME, clamps to -128..0, refuses non-finite."""
    _page(dev)
    orig = dev.pget("ROUTE.MASTER_VOLUME")
    chk.eq(orig, dev.raw_get(OP.GET_MASTER_VOLUME, 4), "parity")
    _expect_stall(chk, lambda: dev.pget("ROUTE.MASTER_VOLUME", 1), "VC_ERR_ADDRESS", "wValue 1")
    try:
        for v, want in ((-12.5, -12.5), (-128.0, -128.0), (10.0, 0.0), (-200.0, -128.0)):
            dev.pset("ROUTE.MASTER_VOLUME", struct.pack("<f", v))
            chk.eq(_f32(dev.pget("ROUTE.MASTER_VOLUME")), want, f"{v} dB")
            chk.eq(_f32(dev.raw_get(OP.GET_MASTER_VOLUME, 4)), want, f"{v} dB legacy readback")
        dev.pset("ROUTE.MASTER_VOLUME", struct.pack("<f", -30.0))
        _expect_stall(chk, lambda: dev.pset("ROUTE.MASTER_VOLUME", NAN), "VC_ERR_VALUE", "NaN")
        chk.eq(_f32(dev.pget("ROUTE.MASTER_VOLUME")), -30.0, "NaN left the value alone")
        dev.raw_set(OP.SET_MASTER_VOLUME, struct.pack("<f", -20.0))
        chk.eq(_f32(dev.pget("ROUTE.MASTER_VOLUME")), -20.0, "legacy write, paged read")
    finally:
        dev.pset("ROUTE.MASTER_VOLUME", orig)


@test("paged_route")
def master_volume_reads_parity(dev, profile, chk):
    """MASTER_VOLUME_MODE and MASTER_VOLUME_SAVED match their legacy opcodes; SAVE has no GET."""
    _page(dev)
    chk.eq(dev.pget("ROUTE.MASTER_VOLUME_MODE"), dev.raw_get(OP.GET_MASTER_VOLUME_MODE, 1), "mode")
    chk.eq(dev.pget("ROUTE.MASTER_VOLUME_SAVED"), dev.raw_get(OP.GET_SAVED_MASTER_VOLUME, 4), "saved")
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
            chk.eq(dev.raw_get(OP.GET_MASTER_VOLUME_MODE, 1), bytes([want]), f"mode {v} legacy readback")
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
        chk.eq(dev.raw_get(OP.GET_SAVED_MASTER_VOLUME, 4), want, "legacy readback")
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
    lg = dev.raw_get(OP.GET_LG_SOUND_SYNC_ENABLE, 1)
    vol, mute = dev.pget("ROUTE.USER_VOLUME"), dev.pget("ROUTE.USER_MUTE")
    chk.eq(vol, dev.raw_get(OP.GET_USER_VOLUME, 4), "volume parity")
    chk.eq(mute, dev.raw_get(OP.GET_USER_MUTE, 1), "mute parity")
    try:
        dev.raw_set(OP.SET_LG_SOUND_SYNC_ENABLE, b"\x00")
        for v, want in ((-10.0, -10.0), (6.0, 0.0), (-100.0, -60.0), (-1.3, round(-1.3 * 256) / 256)):
            dev.pset("ROUTE.USER_VOLUME", struct.pack("<f", v))
            chk.approx(_f32(dev.pget("ROUTE.USER_VOLUME")), want, 1e-6, f"{v} dB")
            chk.eq(dev.raw_get(OP.GET_USER_VOLUME, 4), dev.pget("ROUTE.USER_VOLUME"), f"{v} dB legacy readback")
        _expect_stall(chk, lambda: dev.pset("ROUTE.USER_VOLUME", NAN), "VC_ERR_VALUE", "NaN")
        for v, want in ((1, 1), (0, 0), (0xFF, 1)):
            dev.pset("ROUTE.USER_MUTE", bytes([v]))
            chk.eq(dev.pget("ROUTE.USER_MUTE"), bytes([want]), f"mute 0x{v:02X}")
            chk.eq(dev.raw_get(OP.GET_USER_MUTE, 1), bytes([want]), f"mute 0x{v:02X} legacy readback")
        dev.raw_set(OP.SET_USER_MUTE, b"\x00")
        chk.eq(dev.pget("ROUTE.USER_MUTE"), b"\x00", "legacy mute write, paged read")
    finally:
        dev.pset("ROUTE.USER_VOLUME", vol)
        dev.pset("ROUTE.USER_MUTE", mute)
        dev.raw_set(OP.SET_LG_SOUND_SYNC_ENABLE, lg)


@test("paged_route")
def wrong_direction(dev, profile, chk):
    """A SET to OUTPUT_ENABLE_CONFLICT is VC_ERR_DIRECTION."""
    _page(dev)
    _expect_stall(chk, lambda: dev.pset("ROUTE.OUTPUT_ENABLE_CONFLICT", b"\x01", 0), "VC_ERR_DIRECTION",
                  "SET to a GET-only command")
