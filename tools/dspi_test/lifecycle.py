"""
lifecycle.py: capture the device state before the suite and restore it after. STORE.ALL_PARAMS
round-trips all live state, live IO included, with no flash write. Directory and preset metadata are
outside it: they are captured to report drift, and the flash-gated tests restore what they change.
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass

from .device import DspiDevice, PagedStall
from .telemetry import sample_rate

BULK_BUSY_RETRIES = 10


@dataclass
class Snapshot:
    bulk: bytes                       # STORE.ALL_PARAMS
    directory: bytes                  # STORE.DIRECTORY (7 bytes)
    startup: bytes                    # STORE.STARTUP {mode, slot} + DIRECTORY byte 4 (last active)
    output_config_mode: int           # STORE.OUTPUT_CONFIG_MODE
    mv_mode: int                      # ROUTE.MASTER_VOLUME_MODE
    saved_mv: float                   # ROUTE.MASTER_VOLUME_SAVED
    master_vol: float                 # ROUTE.MASTER_VOLUME (live)
    user_vol: float
    user_mute: int
    input_source: int
    spdif_rx_pin: int                 # S/PDIF input 0
    i2s_bck_pin: int                  # role 0
    mck_enable: int
    mck_pin: int
    mck_mult: int
    lg_enable: int
    dac_mute_cfg: bytes               # 16 bytes
    output_pins: list                 # OUTPUT.PIN per pin output
    output_types: list                # OUTPUT.TYPE per slot
    channel_names: list               # ROUTE.CHANNEL_NAME per channel (raw 32 B)
    active_slot: int
    occupied_mask: int
    sample_rate: int                  # SYS.SYSTEM_STATUS sample rate


def _bulk_len(profile) -> int:
    return max(profile.bulk_payload_len, 2960)


def capture(dev: DspiDevice, profile) -> Snapshot:
    bulk = read_bulk(dev, profile)
    directory = dev.pget("STORE.DIRECTORY", length=7)
    u8, f32 = dev.pget_u8, dev.pget_f32
    return Snapshot(
        bulk=bulk,
        directory=directory,
        # Kept in the 1.1.6 three-byte shape so pickled snapshots compare equal.
        startup=dev.pget("STORE.STARTUP", length=2)[:2] + directory[4:5],
        output_config_mode=u8("STORE.OUTPUT_CONFIG_MODE"),
        mv_mode=u8("ROUTE.MASTER_VOLUME_MODE"),
        saved_mv=f32("ROUTE.MASTER_VOLUME_SAVED"),
        master_vol=f32("ROUTE.MASTER_VOLUME"),
        user_vol=f32("ROUTE.USER_VOLUME"),
        user_mute=u8("ROUTE.USER_MUTE"),
        input_source=u8("INPUT.SOURCE"),
        spdif_rx_pin=u8("INPUT.SPDIF_RX_PIN"),
        i2s_bck_pin=u8("CLOCK.I2S_BCK_PIN"),
        mck_enable=u8("CLOCK.MCK_ENABLE"),
        mck_pin=u8("CLOCK.MCK_PIN"),
        mck_mult=u8("CLOCK.MCK_MULTIPLIER"),
        lg_enable=u8("INPUT.LG_SOUND_SYNC_ENABLE"),
        dac_mute_cfg=dev.pget("OUTPUT.DAC_MUTE_CONFIG", length=16),
        output_pins=[u8("OUTPUT.PIN", i) for i in range(profile.num_pin_outputs)],
        output_types=[u8("OUTPUT.TYPE", s) for s in range(profile.num_spdif)],
        channel_names=[dev.pget("ROUTE.CHANNEL_NAME", ch, 32) for ch in range(profile.num_channels)],
        active_slot=u8("STORE.PRESET_ACTIVE"),
        occupied_mask=struct.unpack_from("<H", directory)[0],
        # Host driven, so not in the bulk blob and not restorable from it; captured to report drift.
        sample_rate=sample_rate(dev),
    )


def read_bulk(dev: DspiDevice, profile) -> bytes:
    """The whole STORE.ALL_PARAMS blob, retried across a busy window."""
    return dev.pget_ready("STORE.ALL_PARAMS", length=_bulk_len(profile))


def push_bulk(dev: DspiDevice, blob: bytes, settle_s: float = 0.4):
    """SET STORE.ALL_PARAMS (live state, no flash), then wait out its deferred restart.
    A refusal raises PagedStall; only VC_ERR_BUSY (bulk buffer held elsewhere) is retried."""
    for attempt in range(BULK_BUSY_RETRIES):
        try:
            dev.pset("STORE.ALL_PARAMS", blob)
            break
        except PagedStall as s:
            if s.result_name != "VC_ERR_BUSY" or attempt == BULK_BUSY_RETRIES - 1:
                raise
            time.sleep(0.1)
    time.sleep(settle_s)
    dev.wait_ready()


def restore_live(dev: DspiDevice, snap: Snapshot, settle_s: float = 0.4):
    """Push the captured bulk blob back."""
    push_bulk(dev, snap.bulk, settle_s)


def diff_bulk(dev: DspiDevice, snap: Snapshot, profile) -> list:
    """Re-read the bulk blob and return [(offset, before, after), ...] differences.
    The blob is deterministic config, so a clean restore matches byte for byte."""
    after = read_bulk(dev, profile)
    n = min(len(snap.bulk), len(after))
    return [(i, snap.bulk[i], after[i]) for i in range(n) if snap.bulk[i] != after[i]]
