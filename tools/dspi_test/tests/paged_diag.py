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
LOOP_PROF = (0x30, 0x31, 0x32, 0x33, 0x34)
LP_SECTION, LP_JOB, LP_COMMAND, LP_BG_PACE, LP_RESET = (0xF000 | c for c in LOOP_PROF)
SLOW_GET = ("STORE.ALL_PARAMS", 6136)   # a handler that always takes measurable time


def _diag_cmds(dev):
    try:
        info = dev.protocol_info()
    except ProtocolError as e:
        raise AssertionError(str(e)) from None
    if info is None:
        raise Skip("no paged dispatcher: SYS.PROTOCOL_INFO STALLs (firmware 1.1.6 or older)")
    gets, sets = dev.paged_commands().get(DIAG, (frozenset(), frozenset()))
    return gets | sets


@test("paged_diag")
def pdiag_absent_commands_unknown(dev, profile, chk):
    """Every DIAG command the build leaves out, and the reserved loop-profiler numbers, STALL with VC_ERR_UNKNOWN."""
    gets = _diag_cmds(dev)
    for cmd in (CLOCK_DIAG, LOOPBACK_STATS) + LOOP_PROF:
        if cmd in gets:
            chk.note(f"DIAG 0x{cmd:02X} is built in")
            continue
        _expect_stall(chk, lambda: dev.pget((DIAG << 8) | cmd, length=8), "VC_ERR_UNKNOWN", f"GET DIAG 0x{cmd:02X}")
        _expect_stall(chk, lambda: dev.pset((DIAG << 8) | cmd), "VC_ERR_UNKNOWN", f"SET DIAG 0x{cmd:02X}")
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


def _loop_prof(dev):
    if LOOP_PROF[0] not in _diag_cmds(dev):
        raise Skip("not a DSPI_LOOP_PROF build: DIAG.LOOP_PROF_* absent")
    from tools.latency_bench.loop_prof import JOBS, SECTIONS
    return SECTIONS, JOBS


def _lp_u32(dev, cmd, addr):
    return struct.unpack("<I", dev.pget(cmd, addr=addr, length=4))[0]


@test("paged_diag")
def pdiag_loop_prof_sections_and_jobs(dev, profile, chk):
    """Every section and job in loop_prof.h reads 4 bytes, one past either list is VC_ERR_ADDRESS, and reads never clear."""
    sections, jobs = _loop_prof(dev)
    first = [_lp_u32(dev, LP_SECTION, i) for i in range(len(sections))]
    second = [_lp_u32(dev, LP_SECTION, i) for i in range(len(sections))]
    chk.ok(max(first) > 0, f"some section has a nonzero worst time ({max(first)} us)")
    chk.ok(all(b >= a for a, b in zip(first, second)), "a second read is never below the first (no clear-on-read)")
    for i in range(len(jobs)):
        chk.eq(len(dev.pget(LP_JOB, addr=i, length=4)), 4, f"job {jobs[i]} reads 4 bytes")
    _expect_stall(chk, lambda: dev.pget(LP_SECTION, addr=len(sections), length=4), "VC_ERR_ADDRESS",
                  "section one past LOOP_PROF_SECTION_LIST")
    _expect_stall(chk, lambda: dev.pget(LP_JOB, addr=len(jobs), length=4), "VC_ERR_ADDRESS",
                  "job one past BG_JOB_LIST")


@test("paged_diag")
def pdiag_loop_prof_command_times(dev, profile, chk):
    """Paged handlers are timed by command ID; RESET zeroes them and an unbuilt ID is VC_ERR_ADDRESS."""
    _loop_prof(dev)
    name, length = SLOW_GET
    slow = dev.resolve(name).id
    dev.pget(name, length=length)
    before = _lp_u32(dev, LP_COMMAND, slow)
    chk.ok(before > 0, f"{name} GET was timed ({before} us)")
    chk.eq(_lp_u32(dev, LP_COMMAND, slow), before, "reading it again does not clear it")
    dev.pset(LP_RESET)
    chk.eq(_lp_u32(dev, LP_COMMAND, slow), 0, "LOOP_PROF_RESET zeroes it")
    chk.eq(len(dev.pget(LP_COMMAND, addr=LP_RESET, length=4)), 4, "a SET-only command has a slot too")
    for bad in (0x0050, 0xDF10, (DIAG << 8) | 0xFF):
        _expect_stall(chk, lambda b=bad: dev.pget(LP_COMMAND, addr=b, length=4), "VC_ERR_ADDRESS",
                      f"command 0x{bad:04X} is not built")
    _expect_stall(chk, lambda: dev.pget(LP_RESET, length=4), "VC_ERR_DIRECTION", "RESET has no GET")
    _expect_stall(chk, lambda: dev.pset(LP_RESET, b"\x00"), "VC_ERR_LENGTH", "RESET takes no payload")


@test("paged_diag")
def pdiag_loop_prof_bg_pace(dev, profile, chk):
    """BG_PACE reads back what was set, 0 sets 1, the payload is exactly 2 bytes; restored afterwards."""
    _loop_prof(dev)
    pace = lambda: struct.unpack("<H", dev.pget(LP_BG_PACE, length=2))[0]  # noqa: E731
    orig = pace()
    try:
        dev.pset(LP_BG_PACE, struct.pack("<H", 7))
        chk.eq(pace(), 7, "SET 7 reads back 7")
        dev.pset(LP_BG_PACE, struct.pack("<H", 0))
        chk.eq(pace(), 1, "SET 0 reads back 1")
        _expect_stall(chk, lambda: dev.pset(LP_BG_PACE, b"\x05"), "VC_ERR_LENGTH", "1-byte payload")
        _expect_stall(chk, lambda: dev.pget(LP_BG_PACE, addr=1, length=2), "VC_ERR_ADDRESS", "wValue 1")
    finally:
        dev.pset(LP_BG_PACE, struct.pack("<H", orig or 1))
    chk.eq(pace(), orig or 1, "pacing restored")
