"""
Native paged tests for the DIAG page (plan section 5.13). Each command exists only in the bench
build that enables it; on a standard build every one answers VC_ERR_UNKNOWN.
"""

import struct

from ..device import OP, ProtocolError, Stall
from ..framework import Skip, test
from .paged_sys import _expect_stall

DIAG = 0xF0
CLOCK_DIAG, LOOPBACK_STATS = 0x10, 0x20
LOOP_PROF = (0x30, 0x31, 0x32, 0x33)


def _diag_cmds(dev):
    try:
        info = dev.protocol_info()
    except ProtocolError as e:
        raise AssertionError(str(e)) from None
    if info is None:
        raise Skip("no paged dispatcher: SYS.PROTOCOL_INFO STALLs (firmware 1.1.6 or older)")
    gets, _sets = dev.paged_commands().get(DIAG, (frozenset(), frozenset()))
    return gets


@test("paged_diag")
def pdiag_absent_commands_unknown(dev, profile, chk):
    """Every DIAG command the build leaves out, and the reserved loop-profiler numbers, STALL with VC_ERR_UNKNOWN."""
    gets = _diag_cmds(dev)
    for cmd in (CLOCK_DIAG, LOOPBACK_STATS) + LOOP_PROF:
        if cmd in gets:
            chk.note(f"DIAG 0x{cmd:02X} is built in")
            continue
        _expect_stall(chk, lambda: dev.pget((DIAG << 8) | cmd, length=8), "VC_ERR_UNKNOWN", f"GET DIAG 0x{cmd:02X}")
    if not gets:
        page_list = dev.pget("SYS.PAGE_LIST", length=256)
        chk.ok(DIAG not in page_list[0::2], "DIAG page not listed when it has no command")


@test("paged_diag")
def pdiag_clock_diag_matches_legacy(dev, profile, chk):
    """DIAG.CLOCK_DIAG returns the same ClockDiagPacket as 0x82; absent, 0x82 is zero-length."""
    gets = _diag_cmds(dev)
    try:
        old = dev.raw_get(OP.GET_CLOCK_DIAG, 256)
    except Stall:
        raise Skip("page 0 no longer answers GET_CLOCK_DIAG")
    if CLOCK_DIAG not in gets:
        chk.eq(old, b"", "0x82 is zero-length without DSPI_CLOCK_DIAG")
        return
    new = dev.pget("DIAG.CLOCK_DIAG", length=256)
    chk.eq(len(new), len(old), "same packet length")
    chk.eq(new[:4], old[:4], "version, platform, slot count, PLL lock")
    _expect_stall(chk, lambda: dev.pget("DIAG.CLOCK_DIAG", addr=1, length=256), "VC_ERR_ADDRESS", "wValue 1")


@test("paged_diag")
def pdiag_loopback_stats_matches_legacy(dev, profile, chk):
    """DIAG.LOOPBACK_STATS carries GET_STATUS 24 and 25; absent, those selectors read zero."""
    gets = _diag_cmds(dev)
    try:
        o24 = struct.unpack("<I", dev.raw_get(OP.GET_STATUS, 4, 24))[0]
        o25 = struct.unpack("<I", dev.raw_get(OP.GET_STATUS, 4, 25))[0]
    except Stall:
        raise Skip("page 0 no longer answers GET_STATUS")
    if LOOPBACK_STATS not in gets:
        chk.eq((o24, o25), (0, 0), "GET_STATUS 24/25 read zero without DSPI_LOOPBACK")
        return
    over, under = struct.unpack("<II", dev.pget("DIAG.LOOPBACK_STATS"))
    chk.ok(over >= o24 and under >= o25, f"counters at or past the legacy reads ({over},{under}) vs ({o24},{o25})")
