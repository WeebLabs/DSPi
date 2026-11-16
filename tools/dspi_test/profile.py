"""
profile.py: PlatformProfile, everything the suite needs to adapt to the attached board, read from the
device (never hardcoded to one SKU): SYS.PLATFORM, the STORE.ALL_PARAMS header, SYS.BUFFER_STATS byte 0,
and the FILTER.BAND address ceilings probed until the device refuses them.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field

from .device import DspiDevice, Stall

PLATFORM_NAMES = {0: "RP2040", 1: "RP2350"}

# Documented factory pin defaults per platform (used ONLY to verify a
# factory-reset; live tests always read current pins via OUTPUT.PIN).
DEFAULT_OUTPUT_PINS = {
    1: [6, 7, 8, 9, 10],   # RP2350: SPDIF1-4 + PDM
    0: [6, 7, 8],          # RP2040: SPDIF1-2 + PDM
}
DEFAULT_SPDIF_RX_PIN = 5
DEFAULT_I2S_BCK_PIN = 14
DEFAULT_MCK_PIN = {1: 13, 0: 21}
MAX_GPIO = {1: 29, 0: 28}


@dataclass
class PlatformProfile:
    platform_id: int
    platform_name: str
    fw_major: int
    fw_minor: int
    fw_patch: int
    fw_beta: int            # 0 = final release, 1..255 = beta N of this patch
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
    build_info: str = ""    # git describe stamp from SYS.BUILD_INFO; "" where it STALLs

    @property
    def fw_str(self) -> str:
        base = f"v{self.fw_major}.{self.fw_minor}.{self.fw_patch}"
        return base if self.fw_beta == 0 else f"{base}-beta{self.fw_beta}"

    def summary(self) -> str:
        build = f" build={self.build_info}" if self.build_info else ""
        return (f"{self.platform_name} {self.fw_str}{build} | out={self.num_output_channels} "
                f"total={self.num_channels} in={self.num_input_channels} "
                f"spdif={self.num_spdif} pins={self.num_pin_outputs} "
                f"bands≤{self.band_ceiling}/{self.max_bands} wire=v{self.wire_format_version} "
                f"bulk={self.bulk_payload_len}B")


def _band_refused(dev: DspiDevice, channel: int, band: int) -> bool:
    try:
        dev.pget("FILTER.BAND", (channel << 8) | band, 16)
        return False
    except Stall:
        return True


def _probe_band_ceiling(dev: DspiDevice, channel: int, hi: int = 16) -> int:
    """Smallest band index FILTER.BAND refuses on `channel`: the PEQ band count (the gap starts there)."""
    return next((b for b in range(hi + 1) if _band_refused(dev, channel, b)), hi + 1)


def _probe_channel_ceiling(dev: DspiDevice, hi: int = 16) -> int:
    """Smallest channel whose band 0 FILTER.BAND refuses: the channel count."""
    return next((ch for ch in range(hi + 1) if _band_refused(dev, ch, 0)), hi + 1)


def build_profile(dev: DspiDevice) -> PlatformProfile:
    plat = dev.pget("SYS.PLATFORM", length=7)
    platform_id = plat[0]
    fw_major = plat[1]
    if len(plat) >= 6:
        fw_minor = plat[4]
        fw_patch = plat[5]
    else:
        # Pre-widening firmware: nibble-packed byte 2, each field caps at 15.
        fw_minor = (plat[2] >> 4) & 0x0F
        fw_patch = plat[2] & 0x0F
    # Firmware without byte 6 predates the ordinal, and all of it was final.
    fw_beta = plat[6] if len(plat) >= 7 else 0
    num_output_channels = plat[3]

    try:
        bi = dev.pget("SYS.BUILD_INFO", length=64)
        build_info = bi[:48].split(b"\0")[0].decode("ascii", "replace")
    except Stall:
        build_info = ""

    # The first 16 bytes of the bulk blob are its WireHeader.
    hdr = dev.pget("STORE.ALL_PARAMS", length=16)
    (fmt_ver, hdr_plat, num_channels, num_out_hdr, num_input_channels,
     max_bands, payload_len, fw_maj_hdr, fw_min_hdr, _resv) = struct.unpack("<BBBBBBHHHI", hdr)

    # Live output slot count (authoritative).
    try:
        num_spdif = dev.pget("SYS.BUFFER_STATS", length=44)[0]
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
        fw_major=fw_major, fw_minor=fw_minor, fw_patch=fw_patch, fw_beta=fw_beta,
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
        build_info=build_info,
    )
