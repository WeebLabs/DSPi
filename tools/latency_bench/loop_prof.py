#!/usr/bin/env python3
"""Main-loop profiler readers (firmware/DSPi/loop_prof.h) over DIAG.LOOP_PROF_*, DSPI_LOOP_PROF builds only.
Values are worst us since boot or reset() and reads never clear; other builds read {} and writes do nothing.
Usage: python3 tools/latency_bench/loop_prof.py [--seconds 5] [--top 15] [--pace N]
"""

from __future__ import annotations

import argparse
import re
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tools.dspi_test.device import DspiDevice, ProtocolError, Stall, Timeout, resolve_command  # noqa: E402

HEADER = Path(__file__).resolve().parents[2] / "firmware" / "DSPi" / "loop_prof.h"
DIAG = 0xF0
SECTION, JOB, COMMAND, BG_PACE, RESET = (f"DIAG.LOOP_PROF_{n}" for n in
                                         ("SECTION", "JOB", "COMMAND", "BG_PACE", "RESET"))


def _parse():
    text = HEADER.read_text()

    def x_list(macro):
        body = re.search(rf"#define {macro}\(X\)(.*?)\n\n", text, re.S).group(1)
        return re.findall(r'X\(\s*\w+\s*,\s*"([^"]+)"\s*\)', body)

    return x_list("LOOP_PROF_SECTION_LIST"), x_list("BG_JOB_LIST")


SECTIONS, JOBS = _parse()


def available(dev) -> bool:
    """True when the device was built with DSPI_LOOP_PROF. Cached on the handle."""
    if getattr(dev, "_loop_prof", None) is None:
        try:
            ok = dev.protocol_info() is not None
            gets = dev.paged_commands().get(DIAG, (frozenset(), frozenset()))[0] if ok else ()
        except (ProtocolError, Stall, Timeout):
            gets = ()
        dev._loop_prof = (resolve_command(SECTION).cmd in gets)
    return dev._loop_prof


def _u32(dev, cmd, addr):
    return struct.unpack("<I", dev.pget(cmd, addr=addr, length=4))[0]


def reset(dev):
    if available(dev):
        dev.pset(RESET)


def sections(dev):
    if not available(dev):
        return {}
    return {name: _u32(dev, SECTION, i) for i, name in enumerate(SECTIONS)}


def jobs(dev):
    if not available(dev):
        return {}
    return {name: _u32(dev, JOB, i) for i, name in enumerate(JOBS)}


def commands(dev):
    """Worst handler time per paged command built in, GET and SET together, nonzero only."""
    if not available(dev):
        return {}
    out = {}
    for page, (gets, sets) in sorted(dev.paged_commands().items()):
        for cmd in sorted(gets | sets):
            cid = (page << 8) | cmd
            us = _u32(dev, COMMAND, cid)
            if us:
                out[resolve_command(cid).name] = us
    return out


def get_pace(dev):
    return struct.unpack("<H", dev.pget(BG_PACE, length=2))[0] if available(dev) else None


def set_pace(dev, n):
    """Each background step does one unit, on one pass in n (1 = normal). True when applied."""
    if not available(dev):
        return False
    dev.pset(BG_PACE, struct.pack("<H", n))
    return True


def merge_worst(acc, new):
    for k, v in new.items():
        acc[k] = max(acc.get(k, 0), v)
    return acc


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=5.0, help="measurement window after the reset")
    ap.add_argument("--top", type=int, default=15, help="commands to list, costliest first")
    ap.add_argument("--pace", type=int, default=None,
                    help="set background pacing for the window, then restore 1")
    a = ap.parse_args()
    dev = DspiDevice()
    if not available(dev):
        raise SystemExit("device has no DIAG.LOOP_PROF_* commands (not a DSPI_LOOP_PROF build)")
    try:
        if a.pace is not None:
            set_pace(dev, a.pace)
            print(f"pace set to {a.pace}, reads back {get_pace(dev)}")
        reset(dev)
        time.sleep(a.seconds)
        sect, job, cmds = sections(dev), jobs(dev), commands(dev)
    finally:
        if a.pace is not None:
            set_pace(dev, 1)
            print(f"pace restored, reads back {get_pace(dev)}")
    print(f"worst times over {a.seconds:g} s, us")
    print("sections:")
    for k, v in sect.items():
        print(f"  {k:20s} {v:7d}")
    print("background jobs:")
    for k, v in job.items():
        print(f"  {k:20s} {v:7d}")
    print(f"commands ({len(cmds)} nonzero, top {a.top}):")
    for k, v in sorted(cmds.items(), key=lambda kv: -kv[1])[:a.top]:
        print(f"  {k:28s} {v:7d}")


if __name__ == "__main__":
    main()
