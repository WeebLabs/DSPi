"""
Native paged tests for the DIAG page (plan section 5.13). Each command exists only in the bench
build that enables it; on a standard build every one answers VC_ERR_UNKNOWN.
"""

import struct

from ..framework import Skip, test
from ..helpers import expect_stall, require_paged

DIAG = 0xF0
CLOCK_DIAG, LOOPBACK_STATS = 0x10, 0x20
LOOP_PROF = (0x30, 0x31, 0x32, 0x33, 0x34)
LP_SECTION, LP_JOB, LP_COMMAND, LP_BG_PACE, LP_RESET = (0xF000 | c for c in LOOP_PROF)
SLOW_GET = ("STORE.ALL_PARAMS", 6284)   # a handler that always takes measurable time


def _diag_cmds(dev):
    require_paged(dev)
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
        expect_stall(chk, lambda: dev.pget((DIAG << 8) | cmd, length=8), "VC_ERR_UNKNOWN", f"GET DIAG 0x{cmd:02X}")
        expect_stall(chk, lambda: dev.pset((DIAG << 8) | cmd), "VC_ERR_UNKNOWN", f"SET DIAG 0x{cmd:02X}")
    if not gets:
        page_list = dev.pget("SYS.PAGE_LIST", length=256)
        chk.ok(DIAG not in page_list[0::2], "DIAG page not listed when it has no command")


@test("paged_diag")
def pdiag_clock_diag_shape(dev, profile, chk):
    """DIAG.CLOCK_DIAG carries a ClockDiagPacket for this platform and refuses a non-zero wValue."""
    if CLOCK_DIAG not in _diag_cmds(dev):
        raise Skip("not a DSPI_CLOCK_DIAG build: DIAG.CLOCK_DIAG absent")
    pkt = dev.pget("DIAG.CLOCK_DIAG", length=256)
    # Version and sizes follow ClockDiagPacket in firmware/DSPi/clock_diag.h.
    chk.eq(len(pkt), {0: 226, 1: 248}.get(profile.platform_id), "packet length")
    chk.eq(pkt[0], 2, "format version")
    chk.eq((pkt[1], pkt[2]), (profile.platform_id, profile.num_spdif), "platform and slot count match the profile")
    chk.member(pkt[3], (0, 1), "PLL lock flag")
    expect_stall(chk, lambda: dev.pget("DIAG.CLOCK_DIAG", addr=1, length=256), "VC_ERR_ADDRESS", "wValue 1")


@test("paged_diag")
def pdiag_loopback_stats_shape(dev, profile, chk):
    """DIAG.LOOPBACK_STATS is two u32 counters that never run backwards; a non-zero wValue is refused."""
    if LOOPBACK_STATS not in _diag_cmds(dev):
        raise Skip("not a DSPI_LOOPBACK build: DIAG.LOOPBACK_STATS absent")
    a = dev.pget("DIAG.LOOPBACK_STATS", length=8)
    b = dev.pget("DIAG.LOOPBACK_STATS", length=8)
    chk.eq(len(a), 8, "reply length")
    (o1, u1), (o2, u2) = struct.unpack("<II", a), struct.unpack("<II", b)
    chk.ok(o2 >= o1 and u2 >= u1, f"counters never run backwards ({o1},{u1}) then ({o2},{u2})")
    expect_stall(chk, lambda: dev.pget("DIAG.LOOPBACK_STATS", addr=1, length=8), "VC_ERR_ADDRESS", "wValue 1")


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
    expect_stall(chk, lambda: dev.pget(LP_SECTION, addr=len(sections), length=4), "VC_ERR_ADDRESS",
                 "section one past LOOP_PROF_SECTION_LIST")
    expect_stall(chk, lambda: dev.pget(LP_JOB, addr=len(jobs), length=4), "VC_ERR_ADDRESS",
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
        expect_stall(chk, lambda b=bad: dev.pget(LP_COMMAND, addr=b, length=4), "VC_ERR_ADDRESS",
                     f"command 0x{bad:04X} is not built")
    expect_stall(chk, lambda: dev.pget(LP_RESET, length=4), "VC_ERR_DIRECTION", "RESET has no GET")
    expect_stall(chk, lambda: dev.pset(LP_RESET, b"\x00"), "VC_ERR_LENGTH", "RESET takes no payload")


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
        expect_stall(chk, lambda: dev.pset(LP_BG_PACE, b"\x05"), "VC_ERR_LENGTH", "1-byte payload")
        expect_stall(chk, lambda: dev.pget(LP_BG_PACE, addr=1, length=2), "VC_ERR_ADDRESS", "wValue 1")
    finally:
        dev.pset(LP_BG_PACE, struct.pack("<H", orig or 1))
    chk.eq(pace(), orig or 1, "pacing restored")
