"""Readers for the firmware's main-loop profiler (firmware/DSPi/loop_prof.h).

Section and job names, and the REQ_GET_STATUS bases, are parsed from the header
so the firmware and this tool cannot drift. Every read returns the worst time in
us since the previous read of that index, and clears it; release builds read 0.
"""

from __future__ import annotations

import re
from pathlib import Path

from tools.dspi_test.device import OPCODE_NAME
from tools.latency_bench.latency_bench import status_u32

HEADER = Path(__file__).resolve().parents[2] / "firmware" / "DSPi" / "loop_prof.h"


def _parse():
    text = HEADER.read_text()

    def x_list(macro):
        body = re.search(rf"#define {macro}\(X\)(.*?)\n\n", text, re.S).group(1)
        return re.findall(r'X\(\s*\w+\s*,\s*"([^"]+)"\s*\)', body)

    def base(name):
        return int(re.search(rf"#define {name}\s+(\d+)", text).group(1))

    return (x_list("LOOP_PROF_SECTION_LIST"), x_list("BG_JOB_LIST"),
            base("LOOP_PROF_STATUS_BASE"), base("LOOP_PROF_JOB_BASE"), base("LOOP_PROF_OPCODE_BASE"))


SECTIONS, JOBS, SECTION_BASE, JOB_BASE, OPCODE_BASE = _parse()


def take_sections(dev):
    return {name: status_u32(dev, SECTION_BASE + i) for i, name in enumerate(SECTIONS)}


def take_jobs(dev):
    return {name: status_u32(dev, JOB_BASE + i) for i, name in enumerate(JOBS)}


def take_opcodes(dev):
    """Worst handler time per vendor opcode, nonzero entries only."""
    out = {}
    for op in range(256):
        us = status_u32(dev, OPCODE_BASE + op)
        if us:
            out[OPCODE_NAME.get(op, f"0x{op:02X}")] = us
    return out


def merge_worst(acc, new):
    for k, v in new.items():
        acc[k] = max(acc.get(k, 0), v)
    return acc
