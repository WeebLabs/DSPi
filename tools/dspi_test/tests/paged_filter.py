"""
Native paged tests for the FILTER page (plan section 5.6): BAND, BAND_BYPASS, EQ_BYPASS.
Band writes are applied by the main loop, so readback polls briefly.
"""

import math
import struct
import time

from ..device import PagedStall
from ..framework import test
from ..helpers import (BAND_FMT, XOVER_BASE, band_addr, expect_stall, finite_rule, nonfinite_refused, require_page,
                       set_retry_busy, wait_band)

FILTER = 0x06
XOVER_COUNT = 4
LT = 11                       # FILTER_LINKWITZ_TRANSFORM
PEAKING = 1
LR4_HP = 35


def _peq_count(profile):
    return profile.band_ceiling


def _valid(profile, ch, band):
    if band < _peq_count(profile):
        return True
    return XOVER_BASE <= band < XOVER_BASE + XOVER_COUNT and ch >= profile.num_input_channels


def _sample_bands(profile):
    n_in, n_ch, peq = profile.num_input_channels, profile.num_channels, _peq_count(profile)
    out = [(0, 0), (n_in - 1, peq - 1), (n_in, 0), (n_ch - 1, peq - 1)]
    out += [(n_in, XOVER_BASE), (n_ch - 1, XOVER_BASE + XOVER_COUNT - 1)]
    return out


@test("paged_filter")
def band_get_all(dev, profile, chk):
    """Every valid band reads 16 bytes of a well-formed band whose bypass byte equals BAND_BYPASS; the rest are VC_ERR_ADDRESS."""
    require_page(dev, FILTER)
    bad = 0
    for ch in range(profile.num_channels):
        for band in range(32):
            if _valid(profile, ch, band):
                r = dev.pget("FILTER.BAND", band_addr(ch, band))
                byp = dev.pget("FILTER.BAND_BYPASS", band_addr(ch, band))
                t, b, _qp, f, q, g = struct.unpack(BAND_FMT, r) if len(r) == 16 else (None,) * 6
                types_ok = t is not None and (t < 32 if band < XOVER_BASE else (t == 0 or 32 <= t < 64))
                ok = (types_ok and b in (0, 1) and byp == bytes([b])
                      and all(math.isfinite(v) for v in (f, q, g)))
                if not ok:
                    bad += 1
                    if bad <= 4:
                        chk.ok(False, f"ch{ch} band{band}: BAND {r.hex()} BAND_BYPASS {byp.hex()}")
            elif band in (profile.band_ceiling, XOVER_BASE - 1, XOVER_BASE, XOVER_BASE + XOVER_COUNT):
                expect_stall(chk, lambda: dev.pget("FILTER.BAND", band_addr(ch, band)), "VC_ERR_ADDRESS",
                             f"BAND ch{ch} band{band}")
    chk.eq(bad, 0, "malformed bands or BAND/BAND_BYPASS disagreements")


@test("paged_filter", mutating=True)
def band_set_roundtrip(dev, profile, chk):
    """BAND SET reads back exactly (crossover bands read qp 0) and BAND_BYPASS follows the bypass byte."""
    require_page(dev, FILTER)
    saved = {cb: dev.pget("FILTER.BAND", band_addr(*cb)) for cb in _sample_bands(profile)}
    try:
        for i, (ch, band) in enumerate(saved):
            peq = band < XOVER_BASE
            qp = 700 + i if peq else 0
            if peq:
                want = struct.pack(BAND_FMT, LT, 0, qp, 40.0 + i, 0.6 + i / 10, 30.0 + i)
            else:
                want = struct.pack(BAND_FMT, LR4_HP, 0, 0, 20.0 + i, 0.707, 0.0)
            set_retry_busy(dev, "FILTER.BAND", want, band_addr(ch, band), retries=20)
            chk.eq(wait_band(dev, ch, band, want, tolerate=()), want, f"readback ch{ch} band{band}")
            chk.eq(dev.pget("FILTER.BAND_BYPASS", band_addr(ch, band)), b"\x00", f"BAND_BYPASS ch{ch} band{band} clear")

            t = PEAKING if peq else LR4_HP
            set_retry_busy(dev, "FILTER.BAND", struct.pack(BAND_FMT, t, 1, 123, 1000.0 + i, 1.5, -3.0 - i),
                           band_addr(ch, band), retries=20)
            want = struct.pack(BAND_FMT, t, 1, 123 if peq else 0, 1000.0 + i, 1.5, -3.0 - i)
            chk.eq(wait_band(dev, ch, band, want, tolerate=()), want, f"second write ch{ch} band{band}")
            chk.eq(dev.pget("FILTER.BAND_BYPASS", band_addr(ch, band)), b"\x01", f"BAND_BYPASS ch{ch} band{band} set")
    finally:
        for (ch, band), raw in saved.items():
            set_retry_busy(dev, "FILTER.BAND", raw, band_addr(ch, band), retries=20)
            wait_band(dev, ch, band, raw, tolerate=())


@test("paged_filter", mutating=True)
def band_bypass_normalised(dev, profile, chk):
    """BAND bypass byte: exactly 1 bypasses, 0xFF and 2 store 0; GET is a valid SET payload."""
    require_page(dev, FILTER)
    ch, band = 0, 1
    saved = dev.pget("FILTER.BAND", band_addr(ch, band))
    try:
        for byte, want in ((1, 1), (0xFF, 0), (2, 0), (0, 0)):
            pl = struct.pack(BAND_FMT, PEAKING, byte, 0, 500.0, 2.0, 4.0)
            set_retry_busy(dev, "FILTER.BAND", pl, band_addr(ch, band), retries=20)
            exp = struct.pack(BAND_FMT, PEAKING, want, 0, 500.0, 2.0, 4.0)
            chk.eq(wait_band(dev, ch, band, exp, tolerate=()), exp, f"bypass byte 0x{byte:02X}")
        got = dev.pget("FILTER.BAND", band_addr(ch, band))
        set_retry_busy(dev, "FILTER.BAND", got, band_addr(ch, band), retries=20)
        chk.eq(wait_band(dev, ch, band, got, tolerate=()), got, "GET reply written back unchanged")
    finally:
        set_retry_busy(dev, "FILTER.BAND", saved, band_addr(ch, band), retries=20)
        wait_band(dev, ch, band, saved, tolerate=())


@test("paged_filter")
def band_errors(dev, profile, chk):
    """Bad channel, reserved gap, crossover on an input channel, band past 23, bad length, all refused."""
    require_page(dev, FILTER)
    n_ch, n_in = profile.num_channels, profile.num_input_channels
    pl = struct.pack(BAND_FMT, PEAKING, 0, 0, 1000.0, 1.0, 0.0)
    for ch, band, what in ((n_ch, 0, "channel past the end"), (0, profile.band_ceiling, "reserved gap"),
                           (0, XOVER_BASE, "crossover on input channel"), (n_in, XOVER_BASE + XOVER_COUNT, "band 24"),
                           (n_in, 0xFF, "band 255")):
        expect_stall(chk, lambda: dev.pget("FILTER.BAND", band_addr(ch, band)), "VC_ERR_ADDRESS", f"GET {what}")
        expect_stall(chk, lambda: dev.pset("FILTER.BAND", pl, band_addr(ch, band)), "VC_ERR_ADDRESS", f"SET {what}")
        expect_stall(chk, lambda: dev.pget("FILTER.BAND_BYPASS", band_addr(ch, band)), "VC_ERR_ADDRESS",
                     f"BAND_BYPASS GET {what}")
        expect_stall(chk, lambda: dev.pset("FILTER.BAND_BYPASS", b"\x01", band_addr(ch, band)), "VC_ERR_ADDRESS",
                     f"BAND_BYPASS SET {what}")
    for n in (0, 15, 17, 18):
        expect_stall(chk, lambda: dev.pset("FILTER.BAND", bytes(n), 0), "VC_ERR_LENGTH", f"{n}-byte BAND SET")
    expect_stall(chk, lambda: dev.pset("FILTER.BAND_BYPASS", b"", 0), "VC_ERR_LENGTH", "empty BAND_BYPASS SET")
    expect_stall(chk, lambda: dev.pset("FILTER.EQ_BYPASS", b"\x00\x00"), "VC_ERR_LENGTH", "2-byte EQ_BYPASS SET")
    expect_stall(chk, lambda: dev.pget("FILTER.EQ_BYPASS", 1), "VC_ERR_ADDRESS", "EQ_BYPASS at wValue 1")


@test("paged_filter", mutating=True)
def band_bypass_roundtrip(dev, profile, chk):
    """BAND_BYPASS SET normalises to 0 or 1, keeps the rest of the band and reads back through both GETs."""
    require_page(dev, FILTER)
    n_in = profile.num_input_channels
    for ch, band in ((2 % profile.num_channels, 2), (n_in, XOVER_BASE + 1)):
        saved = dev.pget("FILTER.BAND", band_addr(ch, band))
        try:
            base = struct.pack(BAND_FMT, PEAKING if band < XOVER_BASE else LR4_HP, 0,
                               0, 500.0, 2.0, 4.0 if band < XOVER_BASE else 0.0)
            set_retry_busy(dev, "FILTER.BAND", base, band_addr(ch, band), retries=20)
            wait_band(dev, ch, band, base, tolerate=())
            for byte, want in ((1, 1), (0xFF, 0), (1, 1), (2, 0)):
                for _ in range(20):
                    try:
                        dev.pset("FILTER.BAND_BYPASS", bytes([byte]), band_addr(ch, band))
                        break
                    except PagedStall as s:
                        if s.result_name != "VC_ERR_BUSY":
                            raise
                        time.sleep(0.01)
                exp = base[:1] + bytes([want]) + base[2:]
                chk.eq(wait_band(dev, ch, band, exp, tolerate=()), exp,
                       f"ch{ch} band{band} bypass 0x{byte:02X}, rest kept")
                chk.eq(dev.pget("FILTER.BAND_BYPASS", band_addr(ch, band)), bytes([want]), "BAND_BYPASS readback")
        finally:
            set_retry_busy(dev, "FILTER.BAND", saved, band_addr(ch, band), retries=20)
            wait_band(dev, ch, band, saved, tolerate=())


@test("paged_filter", mutating=True)
def eq_bypass_roundtrip(dev, profile, chk):
    """EQ_BYPASS reads 0 or 1, non-zero bypasses, and the GET reply is a valid SET payload."""
    require_page(dev, FILTER)
    orig = dev.pget("FILTER.EQ_BYPASS")
    chk.member(orig, (b"\x00", b"\x01"), "current value")
    try:
        for byte, want in ((1, 1), (0, 0), (0xFF, 1), (0, 0)):
            dev.pset("FILTER.EQ_BYPASS", bytes([byte]))
            chk.eq(dev.pget("FILTER.EQ_BYPASS"), bytes([want]), f"0x{byte:02X}")
        dev.pset("FILTER.EQ_BYPASS", b"\x01")
        dev.pset("FILTER.EQ_BYPASS", dev.pget("FILTER.EQ_BYPASS"))
        chk.eq(dev.pget("FILTER.EQ_BYPASS"), b"\x01", "GET reply written back unchanged")
    finally:
        dev.pset("FILTER.EQ_BYPASS", orig)


@test("paged_filter", mutating=True)
def band_nonfinite_refused(dev, profile, chk):
    """A NaN or infinite freq, Q or gain in a BAND SET is VC_ERR_VALUE and the band is unchanged."""
    require_page(dev, FILTER)
    if not finite_rule(dev, profile, chk):
        return

    def get(a):
        time.sleep(0.05)   # a wrongly accepted write would land on the deferred apply
        return dev.pget("FILTER.BAND", a)

    for ch, band in ((0, 0), (profile.num_input_channels, XOVER_BASE)):
        a = band_addr(ch, band)
        cur = dev.pget("FILTER.BAND", a)
        cases = []
        for off, field in ((4, "freq"), (8, "Q"), (12, "gain")):
            for v, label in ((math.nan, "NaN"), (math.inf, "infinity")):
                cases.append((cur[:off] + struct.pack("<f", v) + cur[off + 4:], f"{label} {field}"))
        nonfinite_refused(chk, lambda: get(a), lambda pl: set_retry_busy(dev, "FILTER.BAND", pl, a, retries=20), cases,
                          f"ch{ch} band{band}")
