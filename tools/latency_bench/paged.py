"""Paged-protocol helpers shared by the latency bench scripts: one read per telemetry struct,
fields named as in firmware/DSPi/vendor/vc_page_sys.c and vc_page_input.c.
"""

from __future__ import annotations

import struct
import time

from tools.dspi_test.device import Stall, Timeout, resolve_command

STATS_RESET_WATERMARKS = 0x01     # SYS.STATS_RESET bit 0: watermarks and worst-gap fields


def has_command(dev, name, direction="GET") -> bool:
    """True when this build implements the command (DIAG commands exist only in bench builds)."""
    info = resolve_command(name)
    gets, sets = dev.paged_commands().get(info.page, (frozenset(), frozenset()))
    return info.cmd in (gets if direction == "GET" else sets)


def system_status(dev):
    clk, fs, mv, cdeg, c0, c1 = struct.unpack_from("<IIHhBB", dev.pget("SYS.SYSTEM_STATUS", length=16))
    return dict(clk_sys_hz=clk, sample_rate_hz=fs, core_mv=mv, temperature_cdeg=cdeg,
                cpu0_load=c0, cpu1_load=c1)


def usb_audio_status(dev):
    pk, ov, gap, alt, mounted = struct.unpack_from("<IIIBB", dev.pget("SYS.USB_AUDIO_STATUS", length=16))
    return dict(usb_audio_packets=pk, ring_overruns=ov, worst_packet_gap_us=gap,
                usb_audio_alt_set=alt, usb_audio_mounted=mounted)


def pipeline_stats(dev):
    """min_prewrite reads 0xFFFFFFFF until a write lands after a STATS_RESET."""
    ur, drops, minpre, gap, bg = struct.unpack_from("<IIIII", dev.pget("SYS.PIPELINE_STATS", length=24))
    return dict(ring_underruns=ur, ring_drops=drops, min_prewrite=minpre,
                input_delivery_worst_gap_us=gap, bg_over_budget_count=bg)


def pdm_stats(dev):
    v = struct.unpack_from("<IIII", dev.pget("SYS.PDM_STATS", length=16))
    return dict(zip(("ring_overruns", "ring_underruns", "dma_overruns", "dma_underruns"), v))


def spdif_tx_stats(dev):
    """spdif_underruns counts USB packet gaps over 2 ms (capped at 50 ms), not S/PDIF line events."""
    v = struct.unpack_from("<IIIIII", dev.pget("SYS.SPDIF_TX_STATS", length=24))
    return dict(spdif_underruns=v[0], dma_starvations=v[1], dma_starvations_instance=list(v[2:]))


def input_status(dev):
    src, ch, rate, ppm = struct.unpack_from("<BBxxIi", dev.pget("INPUT.STATUS", length=12))
    return dict(active_input_source=src, active_input_channels=ch, measured_rate_mhz=rate,
                vcxo_ppm_x100=ppm)


_SLOT = ("consumer_free", "consumer_prepared", "consumer_playing", "consumer_fill_pct",
         "consumer_min_fill_pct", "consumer_max_fill_pct", "fill_centi_pct")
_PDM = ("dma_fill_pct", "dma_min_fill_pct", "dma_max_fill_pct",
        "ring_fill_pct", "ring_min_fill_pct", "ring_max_fill_pct")


def buffer_stats(dev):
    """BufferStatsPacket (config.h). Each read advances `sequence`."""
    b = dev.pget("SYS.BUFFER_STATS", length=44)
    n, flags, seq = struct.unpack_from("<BBH", b)
    slots = [dict(zip(_SLOT, struct.unpack_from("<BBBBBBH", b, 4 + 8 * i))) for i in range(n)]
    pdm = dict(zip(_PDM, struct.unpack_from("<6B", b, 36)))
    return dict(num_spdif=n, flags=flags, sequence=seq, spdif=slots, pdm=pdm)


def stats_reset(dev, mask=STATS_RESET_WATERMARKS):
    dev.pset("SYS.STATS_RESET", bytes([mask]))


def wait_ready(dev, timeout_s=4.0, poll_s=0.1) -> bool:
    """Poll SYS.PLATFORM until it answers: a deferred restart briefly blocks control transfers."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            if len(dev.pget("SYS.PLATFORM", length=7)) == 7:
                return True
        except (Stall, Timeout):
            pass
        time.sleep(poll_s)
    return False


def pget_ready(dev, cmd, length=None, addr=0, retries=3):
    """A read with no refusal path of its own, retried across a transient busy window."""
    for attempt in range(retries):
        try:
            return dev.pget(cmd, addr=addr, length=length)
        except (Stall, Timeout):
            if attempt == retries - 1:
                raise
            wait_ready(dev)
            time.sleep(0.05)
