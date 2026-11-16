"""Latency-bench helper with no harness equivalent; the telemetry struct readers live in
tools/dspi_test/telemetry.py.
"""

from __future__ import annotations

STATS_RESET_WATERMARKS = 0x01     # SYS.STATS_RESET bit 0: watermarks and worst-gap fields


def stats_reset(dev, mask=STATS_RESET_WATERMARKS):
    dev.pset("SYS.STATS_RESET", bytes([mask]))
