"""
lifecycle.py — capture the full device state before the suite and restore it
after, so a run leaves the DSPi exactly as it was found.

Primary restore primitive is the bulk parameter blob (REQ_GET_ALL_PARAMS /
REQ_SET_ALL_PARAMS), which round-trips all *live* DSP state with ZERO flash
writes.  Directory/preset metadata (startup mode, output-config mode, master-
volume mode, saved master volume, slot names/occupancy, active slot) is NOT in
the bulk blob — those are captured separately and only ever rewritten by the
flash-gated tests, which restore what they touch.  This module also captures them
so a restore helper is available and so the final report can flag any drift.

Note: bulk GET/SET round-trips live IO (pins/types/MCK/RX pin) regardless of the
output-config mode — that mode only governs preset save/load and boot, not the
live snapshot.  Tests that move pins/types restore them inline; this is the
backstop for everything else.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

from .device import DspiDevice, OP, Stall


@dataclass
class Snapshot:
    bulk: bytes                       # full REQ_GET_ALL_PARAMS blob
    directory: bytes                  # REQ_PRESET_GET_DIR (7 bytes)
    startup: bytes                    # REQ_PRESET_GET_STARTUP (3 bytes)
    output_config_mode: int           # REQ_GET_OUTPUT_CONFIG_MODE
    mv_mode: int                      # REQ_GET_MASTER_VOLUME_MODE
    saved_mv: float                   # REQ_GET_SAVED_MASTER_VOLUME
    master_vol: float                 # REQ_GET_MASTER_VOLUME (live)
    user_vol: float
    user_mute: int
    input_source: int
    spdif_rx_pin: int
    i2s_bck_pin: int
    mck_enable: int
    mck_pin: int
    mck_mult: int
    lg_enable: int
    dac_mute_cfg: bytes               # 16 bytes
    output_pins: list                 # current pins
    output_types: list                # current per-slot types
    channel_names: list               # current per-channel names (raw 32B)
    active_slot: int
    occupied_mask: int


def capture(dev: DspiDevice, profile) -> Snapshot:
    bulk = dev.get_ready(OP.GET_ALL_PARAMS, max(profile.bulk_payload_len, 2960))
    directory = dev.get(OP.PRESET_GET_DIR, 7)
    occupied_mask = directory[0] | (directory[1] << 8)
    output_pins = [dev.get_u8(OP.GET_OUTPUT_PIN, wvalue=i) for i in range(profile.num_pin_outputs)]
    output_types = [dev.get_u8(OP.GET_OUTPUT_TYPE, wvalue=i) for i in range(profile.num_spdif)]
    channel_names = [dev.get(OP.GET_CHANNEL_NAME, 32, wvalue=ch) for ch in range(profile.num_channels)]
    return Snapshot(
        bulk=bulk,
        directory=directory,
        startup=dev.get(OP.PRESET_GET_STARTUP, 3),
        output_config_mode=dev.get_u8(OP.GET_OUTPUT_CONFIG_MODE),
        mv_mode=dev.get_u8(OP.GET_MASTER_VOLUME_MODE),
        saved_mv=dev.get_f32(OP.GET_SAVED_MASTER_VOLUME),
        master_vol=dev.get_f32(OP.GET_MASTER_VOLUME),
        user_vol=dev.get_f32(OP.GET_USER_VOLUME),
        user_mute=dev.get_u8(OP.GET_USER_MUTE),
        input_source=dev.get_u8(OP.GET_INPUT_SOURCE),
        spdif_rx_pin=dev.get_u8(OP.GET_SPDIF_RX_PIN),
        i2s_bck_pin=dev.get_u8(OP.GET_I2S_BCK_PIN),
        mck_enable=dev.get_u8(OP.GET_MCK_ENABLE),
        mck_pin=dev.get_u8(OP.GET_MCK_PIN),
        mck_mult=dev.get_u8(OP.GET_MCK_MULTIPLIER),
        lg_enable=dev.get_u8(OP.GET_LG_SOUND_SYNC_ENABLE),
        dac_mute_cfg=dev.get(OP.GET_DAC_HW_MUTE_CONFIG, 16),
        output_pins=output_pins,
        output_types=output_types,
        channel_names=channel_names,
        active_slot=dev.get_u8(OP.PRESET_GET_ACTIVE),
        occupied_mask=occupied_mask,
    )


def restore_live(dev: DspiDevice, snap: Snapshot, settle_s: float = 0.4):
    """Push the captured bulk blob back (live DSP state, 0 flash).

    The apply runs a deferred, mute-bracketed pipeline reset; allow it to settle
    before the caller re-reads (the transport also retries transient timeouts).
    """
    dev.set(OP.SET_ALL_PARAMS, snap.bulk, wvalue=0)
    time.sleep(settle_s)
    dev.wait_ready()


def diff_bulk(dev: DspiDevice, snap: Snapshot, profile) -> list:
    """Re-read bulk params and return [(offset, before, after), ...] differences.

    Skips the header's volatile-but-harmless bytes (none currently) — the blob is
    deterministic config, so a clean restore should match byte-for-byte.
    """
    after = dev.get_ready(OP.GET_ALL_PARAMS, max(profile.bulk_payload_len, 2960))
    n = min(len(snap.bulk), len(after))
    diffs = []
    for i in range(n):
        if snap.bulk[i] != after[i]:
            diffs.append((i, snap.bulk[i], after[i]))
    return diffs
