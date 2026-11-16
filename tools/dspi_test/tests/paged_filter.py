"""
Native paged tests for the FILTER page (plan section 5.6): BAND, BAND_BYPASS, EQ_BYPASS.

Old opcodes are read with raw_get/raw_set so the A/B comparison never goes through the
translation layer. Band writes are applied by the main loop, so readback polls briefly.
"""

import struct
import time

from ..device import OP, PagedStall, Stall
from ..framework import Skip, test
from .paged_sys import _expect_stall, _paged

FILTER = 0x06
XOVER_BASE, XOVER_COUNT = 20, 4
LT = 11                       # FILTER_LINKWITZ_TRANSFORM
PEAKING = 1
LR4_HP = 35
BAND_FMT = "<BBHfff"          # type, bypass, qp_x512, freq, Q, gain


def _page(dev):
    _paged(dev)
    if FILTER not in dev.paged_commands():
        raise Skip("FILTER page not implemented on this firmware")


def _peq_count(profile):
    return profile.band_ceiling


def _valid(profile, ch, band):
    if band < _peq_count(profile):
        return True
    return XOVER_BASE <= band < XOVER_BASE + XOVER_COUNT and ch >= profile.num_input_channels


def _addr(ch, band):
    return (ch << 8) | band


def _old_band(dev, ch, band):
    """The 16-byte BAND layout assembled from the six 1.1.6 GET_EQ_PARAM scalars."""
    p = [dev.raw_get(OP.GET_EQ_PARAM, 4, (ch << 8) | (band << 3) | i) for i in range(6)]
    t, byp, qp = (struct.unpack("<I", p[i])[0] for i in (0, 4, 5))
    return struct.pack("<BBH", t & 0xFF, byp & 0xFF, qp & 0xFFFF) + p[1] + p[2] + p[3]


def _band_set(dev, ch, band, payload, retries=20):
    """pset BAND, retrying VC_ERR_BUSY while an earlier write is still being applied."""
    for _ in range(retries):
        try:
            return dev.pset("FILTER.BAND", payload, _addr(ch, band))
        except PagedStall as s:
            if s.result_name != "VC_ERR_BUSY":
                raise
            time.sleep(0.01)
    raise AssertionError("FILTER.BAND stayed VC_ERR_BUSY")


def _wait_band(dev, ch, band, want, timeout=0.5):
    """Poll until the deferred apply lands; return the last read."""
    deadline = time.monotonic() + timeout
    got = dev.pget("FILTER.BAND", _addr(ch, band))
    while got != want and time.monotonic() < deadline:
        time.sleep(0.01)
        got = dev.pget("FILTER.BAND", _addr(ch, band))
    return got


def _sample_bands(profile):
    n_in, n_ch, peq = profile.num_input_channels, profile.num_channels, _peq_count(profile)
    out = [(0, 0), (n_in - 1, peq - 1), (n_in, 0), (n_ch - 1, peq - 1)]
    out += [(n_in, XOVER_BASE), (n_ch - 1, XOVER_BASE + XOVER_COUNT - 1)]
    return out


@test("paged_filter")
def band_get_parity_all(dev, profile, chk):
    """BAND equals the six GET_EQ_PARAM scalars on every channel and band 0-31; refused ones are VC_ERR_ADDRESS."""
    _page(dev)
    bad = 0
    for ch in range(profile.num_channels):
        for band in range(32):
            if _valid(profile, ch, band):
                new = dev.pget("FILTER.BAND", _addr(ch, band))
                old = _old_band(dev, ch, band)
                if new != old:
                    bad += 1
                    if bad <= 4:
                        chk.ok(False, f"ch{ch} band{band}: paged {new.hex()} != legacy {old.hex()}")
            else:
                try:
                    dev.raw_get(OP.GET_EQ_PARAM, 4, (ch << 8) | (band << 3))
                    chk.ok(False, f"legacy GET_EQ_PARAM ch{ch} band{band} answered; expected STALL")
                except Stall:
                    pass
                if band in (profile.band_ceiling, XOVER_BASE - 1, XOVER_BASE, XOVER_BASE + XOVER_COUNT):
                    _expect_stall(chk, lambda: dev.pget("FILTER.BAND", _addr(ch, band)), "VC_ERR_ADDRESS",
                                  f"BAND ch{ch} band{band}")
    chk.eq(bad, 0, "BAND mismatches against GET_EQ_PARAM")


@test("paged_filter", mutating=True)
def band_set_roundtrip(dev, profile, chk):
    """BAND SET reads back through BAND and GET_EQ_PARAM; a legacy 18-byte write reads back through BAND."""
    _page(dev)
    saved = {cb: dev.pget("FILTER.BAND", _addr(*cb)) for cb in _sample_bands(profile)}
    try:
        for i, (ch, band) in enumerate(saved):
            peq = band < XOVER_BASE
            qp = 700 + i if peq else 0
            if peq:
                want = struct.pack(BAND_FMT, LT, 0, qp, 40.0 + i, 0.6 + i / 10, 30.0 + i)
            else:
                want = struct.pack(BAND_FMT, LR4_HP, 0, 0, 20.0 + i, 0.707, 0.0)
            _band_set(dev, ch, band, want)
            chk.eq(_wait_band(dev, ch, band, want), want, f"paged readback ch{ch} band{band}")
            chk.eq(_old_band(dev, ch, band), want, f"legacy readback ch{ch} band{band}")

            t = PEAKING if peq else LR4_HP
            pkt = struct.pack("<BBBBfff", ch, band, t, 1, 1000.0 + i, 1.5, -3.0 - i)
            dev.raw_set(OP.SET_EQ_PARAM, pkt + struct.pack("<H", 123))
            want = struct.pack(BAND_FMT, t, 1, 123 if peq else 0, 1000.0 + i, 1.5, -3.0 - i)
            chk.eq(_wait_band(dev, ch, band, want), want, f"legacy 18-byte write, paged read ch{ch} band{band}")
            # A 16-byte legacy write keeps the stored qp.
            dev.raw_set(OP.SET_EQ_PARAM, pkt)
            chk.eq(_wait_band(dev, ch, band, want), want, f"legacy 16-byte write keeps qp ch{ch} band{band}")
    finally:
        for (ch, band), raw in saved.items():
            _band_set(dev, ch, band, raw)
            _wait_band(dev, ch, band, raw)


@test("paged_filter", mutating=True)
def band_bypass_normalised(dev, profile, chk):
    """BAND bypass byte: exactly 1 bypasses, 0xFF and 2 store 0; GET is a valid SET payload."""
    _page(dev)
    ch, band = 0, 1
    saved = dev.pget("FILTER.BAND", _addr(ch, band))
    try:
        for byte, want in ((1, 1), (0xFF, 0), (2, 0), (0, 0)):
            pl = struct.pack(BAND_FMT, PEAKING, byte, 0, 500.0, 2.0, 4.0)
            _band_set(dev, ch, band, pl)
            exp = struct.pack(BAND_FMT, PEAKING, want, 0, 500.0, 2.0, 4.0)
            chk.eq(_wait_band(dev, ch, band, exp), exp, f"bypass byte 0x{byte:02X}")
        got = dev.pget("FILTER.BAND", _addr(ch, band))
        _band_set(dev, ch, band, got)
        chk.eq(_wait_band(dev, ch, band, got), got, "GET reply written back unchanged")
    finally:
        _band_set(dev, ch, band, saved)
        _wait_band(dev, ch, band, saved)


@test("paged_filter")
def band_errors(dev, profile, chk):
    """Bad channel, reserved gap, crossover on an input channel, band past 23, bad length, all refused."""
    _page(dev)
    n_ch, n_in = profile.num_channels, profile.num_input_channels
    pl = struct.pack(BAND_FMT, PEAKING, 0, 0, 1000.0, 1.0, 0.0)
    for ch, band, what in ((n_ch, 0, "channel past the end"), (0, profile.band_ceiling, "reserved gap"),
                           (0, XOVER_BASE, "crossover on input channel"), (n_in, XOVER_BASE + XOVER_COUNT, "band 24"),
                           (n_in, 0xFF, "band 255")):
        _expect_stall(chk, lambda: dev.pget("FILTER.BAND", _addr(ch, band)), "VC_ERR_ADDRESS", f"GET {what}")
        _expect_stall(chk, lambda: dev.pset("FILTER.BAND", pl, _addr(ch, band)), "VC_ERR_ADDRESS", f"SET {what}")
        _expect_stall(chk, lambda: dev.pget("FILTER.BAND_BYPASS", _addr(ch, band)), "VC_ERR_ADDRESS",
                      f"BAND_BYPASS GET {what}")
        _expect_stall(chk, lambda: dev.pset("FILTER.BAND_BYPASS", b"\x01", _addr(ch, band)), "VC_ERR_ADDRESS",
                      f"BAND_BYPASS SET {what}")
    for n in (0, 15, 17, 18):
        _expect_stall(chk, lambda: dev.pset("FILTER.BAND", bytes(n), 0), "VC_ERR_LENGTH", f"{n}-byte BAND SET")
    _expect_stall(chk, lambda: dev.pset("FILTER.BAND_BYPASS", b"", 0), "VC_ERR_LENGTH", "empty BAND_BYPASS SET")
    _expect_stall(chk, lambda: dev.pset("FILTER.EQ_BYPASS", b"\x00\x00"), "VC_ERR_LENGTH", "2-byte EQ_BYPASS SET")
    _expect_stall(chk, lambda: dev.pget("FILTER.EQ_BYPASS", 1), "VC_ERR_ADDRESS", "EQ_BYPASS at wValue 1")


@test("paged_filter")
def band_bypass_get_parity_all(dev, profile, chk):
    """BAND_BYPASS equals GET_BAND_BYPASS on every valid channel and band."""
    _page(dev)
    bad = 0
    for ch in range(profile.num_channels):
        for band in range(XOVER_BASE + XOVER_COUNT):
            if not _valid(profile, ch, band):
                continue
            new = dev.pget("FILTER.BAND_BYPASS", _addr(ch, band))
            old = dev.raw_get(OP.GET_BAND_BYPASS, 1, _addr(ch, band))
            bad += new != old
    chk.eq(bad, 0, "BAND_BYPASS mismatches against GET_BAND_BYPASS")


@test("paged_filter", mutating=True)
def band_bypass_roundtrip(dev, profile, chk):
    """BAND_BYPASS SET keeps the rest of the band; legacy SET_BAND_BYPASS reads back through the paged GET."""
    _page(dev)
    n_in = profile.num_input_channels
    for ch, band in ((2 % profile.num_channels, 2), (n_in, XOVER_BASE + 1)):
        saved = dev.pget("FILTER.BAND", _addr(ch, band))
        try:
            base = struct.pack(BAND_FMT, PEAKING if band < XOVER_BASE else LR4_HP, 0,
                               0, 500.0, 2.0, 4.0 if band < XOVER_BASE else 0.0)
            _band_set(dev, ch, band, base)
            _wait_band(dev, ch, band, base)
            for byte, want in ((1, 1), (0xFF, 0), (1, 1), (2, 0)):
                for _ in range(20):
                    try:
                        dev.pset("FILTER.BAND_BYPASS", bytes([byte]), _addr(ch, band))
                        break
                    except PagedStall as s:
                        if s.result_name != "VC_ERR_BUSY":
                            raise
                        time.sleep(0.01)
                exp = base[:1] + bytes([want]) + base[2:]
                chk.eq(_wait_band(dev, ch, band, exp), exp, f"ch{ch} band{band} bypass 0x{byte:02X}, rest kept")
                chk.eq(dev.raw_get(OP.GET_BAND_BYPASS, 1, _addr(ch, band)), bytes([want]), "legacy readback")
            dev.raw_set(OP.SET_BAND_BYPASS, b"\x01", _addr(ch, band))
            exp = base[:1] + b"\x01" + base[2:]
            chk.eq(_wait_band(dev, ch, band, exp), exp, f"ch{ch} band{band} legacy write, paged read")
            chk.eq(dev.pget("FILTER.BAND_BYPASS", _addr(ch, band)), b"\x01", "BAND_BYPASS after legacy write")
        finally:
            _band_set(dev, ch, band, saved)
            _wait_band(dev, ch, band, saved)


@test("paged_filter", mutating=True)
def eq_bypass_roundtrip(dev, profile, chk):
    """EQ_BYPASS matches GET_BYPASS, non-zero bypasses, and the legacy SET reads back through it."""
    _page(dev)
    orig = dev.pget("FILTER.EQ_BYPASS")
    chk.eq(orig, dev.raw_get(OP.GET_BYPASS, 1), "parity")
    try:
        for byte, want in ((1, 1), (0, 0), (0xFF, 1), (0, 0)):
            dev.pset("FILTER.EQ_BYPASS", bytes([byte]))
            chk.eq(dev.pget("FILTER.EQ_BYPASS"), bytes([want]), f"paged 0x{byte:02X}")
            chk.eq(dev.raw_get(OP.GET_BYPASS, 1), bytes([want]), f"legacy readback 0x{byte:02X}")
        dev.raw_set(OP.SET_BYPASS, b"\x01")
        chk.eq(dev.pget("FILTER.EQ_BYPASS"), b"\x01", "legacy write, paged read")
    finally:
        dev.pset("FILTER.EQ_BYPASS", orig)
