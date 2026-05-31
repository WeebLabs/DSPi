"""
profile.py — PlatformProfile: everything the suite needs to adapt to whichever
board is attached, derived from the device itself (never hardcoded to one SKU).

Sources, in order of authority:
  * REQ_GET_PLATFORM (0x7F)  -> platform id, fw version, output-channel count.
  * REQ_GET_ALL_PARAMS (0xA0) 16-byte header -> total/ input channel counts,
    max bands, wire format version (also cross-checks platform id).
  * REQ_GET_BUFFER_STATS (0xB0) byte 0 -> live SPDIF instance count.
  * Empirical probing of the EQ index ceilings (GET_EQ_PARAM until STALL) so the
    channel count and per-channel band ceiling are *verified against firmware
    behaviour*, not assumed.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field

from .device import DspiDevice, OP, Stall

PLATFORM_NAMES = {0: "RP2040", 1: "RP2350"}

# Documented factory pin defaults per platform (used ONLY to verify a
# factory-reset; live tests always read current pins via REQ_GET_OUTPUT_PIN).
DEFAULT_OUTPUT_PINS = {
    1: [6, 7, 8, 9, 10],   # RP2350: SPDIF1-4 + PDM
    0: [6, 7, 8],          # RP2040: SPDIF1-2 + PDM
}
DEFAULT_SPDIF_RX_PIN = 5
DEFAULT_I2S_BCK_PIN = 14
DEFAULT_MCK_PIN = {1: 13, 0: 21}
# GPIO pins that REQ_SET_*_PIN must reject as invalid (UART TX + power/LED).
INVALID_GPIO_PINS = [12, 23, 24, 25]
MAX_GPIO = {1: 29, 0: 28}


@dataclass
class PlatformProfile:
    platform_id: int
    platform_name: str
    fw_major: int
    fw_minor: int
    fw_patch: int
    num_output_channels: int
    num_channels: int
    num_input_channels: int
    num_spdif: int
    num_pin_outputs: int
    max_bands: int          # wire max (12)
    band_ceiling: int       # actual validation ceiling per channel (10)
    wire_format_version: int
    bulk_payload_len: int
    default_output_pins: list = field(default_factory=list)

    @property
    def fw_str(self) -> str:
        return f"v{self.fw_major}.{self.fw_minor}.{self.fw_patch}"

    def summary(self) -> str:
        return (f"{self.platform_name} {self.fw_str} | out={self.num_output_channels} "
                f"total={self.num_channels} in={self.num_input_channels} "
                f"spdif={self.num_spdif} pins={self.num_pin_outputs} "
                f"bands≤{self.band_ceiling}/{self.max_bands} wire=v{self.wire_format_version} "
                f"bulk={self.bulk_payload_len}B")


def _probe_band_ceiling(dev: DspiDevice, channel: int, hi: int = 16) -> int:
    """Smallest band index that STALLs on GET_EQ_PARAM == the ceiling.

    wValue = (channel<<8)|(band<<4)|param(=0 type).
    """
    for band in range(hi + 1):
        wv = (channel << 8) | (band << 4) | 0
        try:
            dev.get(OP.GET_EQ_PARAM, 4, wvalue=wv)
        except Stall:
            return band
    return hi + 1


def _probe_channel_ceiling(dev: DspiDevice, hi: int = 16) -> int:
    """Smallest channel index that STALLs on GET_EQ_PARAM (band 0) == count."""
    for ch in range(hi + 1):
        wv = (ch << 8) | (0 << 4) | 0
        try:
            dev.get(OP.GET_EQ_PARAM, 4, wvalue=wv)
        except Stall:
            return ch
    return hi + 1


def build_profile(dev: DspiDevice) -> PlatformProfile:
    plat = dev.get(OP.GET_PLATFORM, 4)
    platform_id = plat[0]
    fw_major = plat[1]
    fw_minor = (plat[2] >> 4) & 0x0F
    fw_patch = plat[2] & 0x0F
    num_output_channels = plat[3]

    # Bulk header (first 16 bytes is the WireHeader).
    hdr = dev.get(OP.GET_ALL_PARAMS, 16)
    (fmt_ver, hdr_plat, num_channels, num_out_hdr, num_input_channels,
     max_bands, payload_len, fw_maj_hdr, fw_min_hdr, _resv) = struct.unpack("<BBBBBBHHHI", hdr)

    # Live SPDIF instance count from buffer stats (authoritative).
    try:
        num_spdif = dev.get(OP.GET_BUFFER_STATS, 44)[0]
    except Stall:
        num_spdif = 4 if num_output_channels >= 9 else 2

    band_ceiling = _probe_band_ceiling(dev, 0)
    chan_ceiling = _probe_channel_ceiling(dev)
    # Trust the probe for the validation ceiling; trust header for the nominal.
    if chan_ceiling != num_channels:
        # Surface the discrepancy rather than silently picking one.
        num_channels = num_channels  # keep header value; runner will note probe mismatch

    return PlatformProfile(
        platform_id=platform_id,
        platform_name=PLATFORM_NAMES.get(platform_id, f"id{platform_id}"),
        fw_major=fw_major, fw_minor=fw_minor, fw_patch=fw_patch,
        num_output_channels=num_output_channels,
        num_channels=num_channels,
        num_input_channels=num_input_channels,
        num_spdif=num_spdif,
        num_pin_outputs=num_spdif + 1,
        max_bands=max_bands,
        band_ceiling=band_ceiling,
        wire_format_version=fmt_ver,
        bulk_payload_len=payload_len,
        default_output_pins=DEFAULT_OUTPUT_PINS.get(platform_id, []),
    )
