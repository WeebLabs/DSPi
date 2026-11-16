"""Readers for the paged telemetry structs (layouts in Documentation/commands.md sections 6.5, 8.1,
14.1 and 18), shared by the QA suite and tools/latency_bench.
"""

from __future__ import annotations

import struct

# name -> (reply length, ((field, offset, struct format char), ...))
STRUCTS = {
    "SYS.SYSTEM_STATUS": (16, (
        ("clk_sys_hz", 0, "I"), ("sample_rate_hz", 4, "I"), ("core_mv", 8, "H"),
        ("temperature_cdeg", 10, "h"), ("cpu0_load", 12, "B"), ("cpu1_load", 13, "B"))),
    "SYS.USB_AUDIO_STATUS": (16, (
        ("usb_audio_packets", 0, "I"), ("ring_overruns", 4, "I"), ("worst_packet_gap_us", 8, "I"),
        ("usb_audio_alt_set", 12, "B"), ("usb_audio_mounted", 13, "B"))),
    "SYS.PIPELINE_STATS": (24, (
        ("ring_underruns", 0, "I"), ("ring_drops", 4, "I"), ("min_prewrite", 8, "I"),
        ("input_delivery_worst_gap_us", 12, "I"), ("bg_over_budget_count", 16, "I"))),
    "SYS.PDM_STATS": (16, (
        ("ring_overruns", 0, "I"), ("ring_underruns", 4, "I"), ("dma_overruns", 8, "I"),
        ("dma_underruns", 12, "I"))),
    "SYS.SPDIF_TX_STATS": (24, (
        ("spdif_underruns", 0, "I"), ("dma_starvations", 4, "I"), ("dma_starvations_0", 8, "I"),
        ("dma_starvations_1", 12, "I"), ("dma_starvations_2", 16, "I"), ("dma_starvations_3", 20, "I"))),
    "INPUT.STATUS": (12, (
        ("active_input_source", 0, "B"), ("active_input_channels", 1, "B"),
        ("measured_rate_mhz", 4, "I"), ("vcxo_ppm_x100", 8, "i"))),
    # Exists only in DSPI_LOOPBACK builds; elsewhere the GET is VC_ERR_UNKNOWN.
    "DIAG.LOOPBACK_STATS": (8, (("overflow_frames", 0, "I"), ("underrun_frames", 4, "I"))),
}


def _decode(name: str, raw: bytes) -> dict:
    return {f: struct.unpack_from("<" + fmt, raw, off)[0] for f, off, fmt in STRUCTS[name][1]}


def read_struct(dev, name: str) -> dict:
    """Every field of one telemetry struct from a single GET."""
    return _decode(name, dev.pget(name, length=STRUCTS[name][0]))


def read_field(dev, name: str, field: str):
    """One field of a telemetry struct, e.g. read_field(dev, "SYS.SYSTEM_STATUS", "sample_rate_hz")."""
    for f, off, fmt in STRUCTS[name][1]:
        if f == field:
            return struct.unpack_from("<" + fmt, dev.pget(name, length=STRUCTS[name][0]), off)[0]
    raise KeyError(f"{name} has no field {field!r}")


def system_status(dev) -> dict:
    return read_struct(dev, "SYS.SYSTEM_STATUS")


def usb_audio_status(dev) -> dict:
    return read_struct(dev, "SYS.USB_AUDIO_STATUS")


def pipeline_stats(dev) -> dict:
    """min_prewrite reads 0xFFFFFFFF until a write lands after a SYS.STATS_RESET."""
    return read_struct(dev, "SYS.PIPELINE_STATS")


def pdm_stats(dev) -> dict:
    return read_struct(dev, "SYS.PDM_STATS")


def spdif_tx_stats(dev) -> dict:
    """spdif_underruns counts USB packet gaps of 2 to 50 ms, not S/PDIF line events."""
    return read_struct(dev, "SYS.SPDIF_TX_STATS")


def input_status(dev) -> dict:
    return read_struct(dev, "INPUT.STATUS")


def sample_rate(dev) -> int:
    """The device's operating rate in Hz (host driven, so not in the bulk blob)."""
    return read_field(dev, "SYS.SYSTEM_STATUS", "sample_rate_hz")


def levels(dev, num_channels: int) -> dict:
    """MEAS.LEVELS: per-channel peaks (u16, 32767 = full scale), CPU loads, clip flags, input count."""
    n = num_channels
    raw = dev.pget("MEAS.LEVELS", length=2 * n + 7)
    cpu0, cpu1, clips, n_in = struct.unpack_from("<BBIB", raw, 2 * n)
    return dict(peaks=list(struct.unpack_from(f"<{n}H", raw)), cpu0_load=cpu0, cpu1_load=cpu1,
                clip_flags=clips, active_input_channels=n_in, raw=raw)


_SLOT = ("consumer_free", "consumer_prepared", "consumer_playing", "consumer_fill_pct",
         "consumer_min_fill_pct", "consumer_max_fill_pct", "fill_centi_pct")
_PDM = ("dma_fill_pct", "dma_min_fill_pct", "dma_max_fill_pct",
        "ring_fill_pct", "ring_min_fill_pct", "ring_max_fill_pct")


def buffer_stats(dev) -> dict:
    """SYS.BUFFER_STATS. Each read advances `sequence`, which restarts near 0 after a reboot."""
    b = dev.pget("SYS.BUFFER_STATS", length=44)
    n, flags, seq = struct.unpack_from("<BBH", b)
    slots = [dict(zip(_SLOT, struct.unpack_from("<BBBBBBH", b, 4 + 8 * i))) for i in range(n)]
    pdm = dict(zip(_PDM, struct.unpack_from("<6B", b, 36)))
    return dict(num_spdif=n, flags=flags, sequence=seq, spdif=slots, pdm=pdm)
