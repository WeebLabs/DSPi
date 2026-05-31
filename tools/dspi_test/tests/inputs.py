"""
Inputs / SPDIF-RX / LG Sound Sync / DAC hardware mute group.

Input source        0xE0/0xE1
SPDIF RX status     0xE2 (16B) / channel status 0xE3 (24B)
SPDIF RX pin        0xE4/0xE5
LG Sound Sync       0xE6/0xE7 / status 0xE8 (16B)
DAC hardware mute   0xEA/0xEB (config 16B) / 0xEC test pulse
"""

import struct
import time

from ..device import OP, Stall
from ..framework import test
from ..helpers import bool_roundtrip

PIN_SUCCESS, PIN_INVALID_PIN, PIN_IN_USE, PIN_INVALID_OUTPUT, PIN_OUTPUT_ACTIVE = range(5)
INPUT_USB, INPUT_SPDIF = 0, 1


@test("inputs")
def input_source_get(dev, profile, chk):
    """0xE1 returns a valid InputSource enum value."""
    chk.member(dev.get_u8(OP.GET_INPUT_SOURCE), (INPUT_USB, INPUT_SPDIF), "input source")


@test("inputs", mutating=True)
def input_source_switch_roundtrip(dev, profile, chk):
    """0xE0/0xE1: switch to the other source and back (deferred, mute+reset); invalid dropped."""
    orig = dev.get_u8(OP.GET_INPUT_SOURCE)
    other = INPUT_USB if orig == INPUT_SPDIF else INPUT_SPDIF
    try:
        dev.set_u8(OP.SET_INPUT_SOURCE, other)
        dev.wait_ready()
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline and dev.get_u8(OP.GET_INPUT_SOURCE) != other:
            time.sleep(0.03)
        chk.eq(dev.get_u8(OP.GET_INPUT_SOURCE), other, f"switched to {other}")
        # Invalid source silently dropped (no STALL, no change).
        chk.no_stall(lambda: dev.set_u8(OP.SET_INPUT_SOURCE, 2), "invalid source no STALL")
        time.sleep(0.05)
        chk.eq(dev.get_u8(OP.GET_INPUT_SOURCE), other, "invalid source ignored")
    finally:
        dev.set_u8(OP.SET_INPUT_SOURCE, orig)
        dev.wait_ready()
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline and dev.get_u8(OP.GET_INPUT_SOURCE) != orig:
            time.sleep(0.03)
        chk.eq(dev.get_u8(OP.GET_INPUT_SOURCE), orig, "input source restored")


@test("inputs")
def spdif_rx_status_plausible(dev, profile, chk):
    """0xE2 returns a 16-byte status with in-range fields (idle/no-cable tolerated)."""
    data = dev.get(OP.GET_SPDIF_RX_STATUS, 16)
    chk.eq(len(data), 16, "RX status length")
    state, in_src, lock_cnt, loss_cnt = data[0], data[1], data[2], data[3]
    sample_rate = struct.unpack_from("<I", data, 4)[0]
    fifo = struct.unpack_from("<H", data, 12)[0]
    chk.in_range(state, 0, 3, "RX state enum")
    chk.member(in_src, (0, 1), "RX input_source")
    chk.in_range(fifo, 0, 100, "RX fifo fill %")
    chk.member(sample_rate, (0, 44100, 48000, 88200, 96000), "RX sample rate")
    chk.note(f"RX state={state} src={in_src} rate={sample_rate} fifo={fifo}% locks={lock_cnt} losses={loss_cnt}")


@test("inputs")
def spdif_rx_channel_status_shape(dev, profile, chk):
    """0xE3 returns exactly 24 bytes (IEC 60958 channel status; all-zero when inactive)."""
    data = dev.get(OP.GET_SPDIF_RX_CH_STATUS, 24)
    chk.eq(len(data), 24, "RX channel status length")


@test("inputs", mutating=True)
def spdif_rx_pin_validation(dev, profile, chk):
    """0xE4/0xE5: validation status codes (no state change); move tested only on USB input."""
    orig = dev.get_u8(OP.GET_SPDIF_RX_PIN)
    # Invalid pin -> INVALID_PIN (no move).
    chk.eq(dev.get_u8(OP.SET_SPDIF_RX_PIN, wvalue=12), PIN_INVALID_PIN, "pin 12 -> INVALID_PIN")
    # Same pin -> SUCCESS no-op.
    chk.eq(dev.get_u8(OP.SET_SPDIF_RX_PIN, wvalue=orig), PIN_SUCCESS, "same pin -> SUCCESS")
    # A pin used by an output -> PIN_IN_USE.
    out_pin = dev.get_u8(OP.GET_OUTPUT_PIN, wvalue=0)
    chk.eq(dev.get_u8(OP.SET_SPDIF_RX_PIN, wvalue=out_pin), PIN_IN_USE, f"pin {out_pin} -> PIN_IN_USE")
    chk.eq(dev.get_u8(OP.GET_SPDIF_RX_PIN), orig, "pin unchanged by rejected sets")
    # Actual move only when on USB input (avoids a live RX hot-swap blackout).
    if dev.get_u8(OP.GET_INPUT_SOURCE) != INPUT_USB:
        chk.note("on SPDIF input — skipping live RX pin move to avoid hot-swap")
        return
    from .outputs import _free_pin
    free = _free_pin(dev, profile, prefer=(16, 17, 18, 19, 20))
    if free is None:
        chk.note("no free pin for RX move")
        return
    try:
        chk.eq(dev.get_u8(OP.SET_SPDIF_RX_PIN, wvalue=free), PIN_SUCCESS, f"RX move -> {free}")
        chk.eq(dev.get_u8(OP.GET_SPDIF_RX_PIN), free, "RX pin reflects move")
    finally:
        dev.get_u8(OP.SET_SPDIF_RX_PIN, wvalue=orig)
        chk.eq(dev.get_u8(OP.GET_SPDIF_RX_PIN), orig, "RX pin restored")


@test("inputs", mutating=True)
def lg_sound_sync_enable_and_status(dev, profile, chk):
    """0xE6/0xE7 enable round-trips; 0xE8 status 16B with volume 0xFF sentinel when never decoded."""
    orig = dev.get_u8(OP.GET_LG_SOUND_SYNC_ENABLE)
    try:
        bool_roundtrip(dev, chk, OP.SET_LG_SOUND_SYNC_ENABLE, OP.GET_LG_SOUND_SYNC_ENABLE, label="LG enable")
        st = dev.get(OP.GET_LG_SOUND_SYNC_STATUS, 16)
        chk.eq(len(st), 16, "LG status length")
        enabled, present, volume, muted = st[0], st[1], st[2], st[3]
        chk.eq(enabled, dev.get_u8(OP.GET_LG_SOUND_SYNC_ENABLE), "LG status.enabled matches 0xE7")
        chk.member(present, (0, 1), "LG present")
        chk.member(muted, (0, 1), "LG muted")
        chk.ok(volume == 0xFF or volume <= 100, f"LG volume {volume} (0xFF sentinel or 0..100)")
        chk.ok(all(b == 0 for b in st[4:16]), "LG status reserved zero")
    finally:
        dev.set_u8(OP.SET_LG_SOUND_SYNC_ENABLE, orig)


@test("inputs")
def dac_hw_mute_config_get(dev, profile, chk):
    """0xEB returns a 16-byte config with in-range fields."""
    cfg = dev.get(OP.GET_DAC_HW_MUTE_CONFIG, 16)
    chk.eq(len(cfg), 16, "DAC mute config length")
    enabled, active_low, pin = cfg[0], cfg[1], cfg[2]
    hold, release = struct.unpack_from("<H", cfg, 4)[0], struct.unpack_from("<H", cfg, 6)[0]
    chk.member(enabled, (0, 1), "DAC mute enabled")
    chk.member(active_low, (0, 1), "DAC mute active_low")
    chk.ok(pin == 0xFF or pin <= 29, f"DAC mute pin {pin}")
    if enabled:
        chk.in_range(hold, 1, 500, "DAC mute hold_ms")
    chk.in_range(release, 0, 500, "DAC mute release_ms")
    chk.note(f"DAC mute enabled={enabled} active_low={active_low} pin={pin} hold={hold} release={release}")


@test("inputs", mutating=True)
def dac_hw_mute_silent_reject(dev, profile, chk):
    """0xEA rejects bad fields silently (config unchanged, no STALL) — no flash on reject."""
    before = dev.get(OP.GET_DAC_HW_MUTE_CONFIG, 16)
    # Bad active_low (2), and below-min hold_ms with enabled — both must be ignored.
    bad = bytearray(before)
    bad[0] = 1            # enabled
    bad[1] = 2            # invalid active_low
    cfg = bytes(bad)
    chk.no_stall(lambda: dev.set(OP.SET_DAC_HW_MUTE_CONFIG, cfg), "bad active_low no STALL")
    chk.eq(bytes(dev.get_ready(OP.GET_DAC_HW_MUTE_CONFIG, 16)), bytes(before), "config unchanged (bad active_low)")
    bad2 = bytearray(before)
    bad2[0] = 1
    bad2[1] = 1
    struct.pack_into("<H", bad2, 4, 0)   # hold_ms = 0 (below min)
    chk.no_stall(lambda: dev.set(OP.SET_DAC_HW_MUTE_CONFIG, bytes(bad2)), "hold=0 no STALL")
    chk.eq(bytes(dev.get_ready(OP.GET_DAC_HW_MUTE_CONFIG, 16)), bytes(before), "config unchanged (hold=0)")


@test("inputs", mutating=True)
def dac_hw_mute_test_pulse(dev, profile, chk):
    """0xEC returns promptly (0x00 if enabled & pinned, else 0x03) and device stays responsive."""
    t0 = time.monotonic()
    st = dev.get(OP.TEST_DAC_HW_MUTE, 1)
    dt = time.monotonic() - t0
    chk.member(st[0], (PIN_SUCCESS, PIN_INVALID_OUTPUT), "test status 0x00 or 0x03")
    chk.ok(dt < 0.5, f"test command returned promptly ({dt*1000:.0f} ms, not blocking for the pulse)")
    # If it triggered a pulse, the device may briefly mute; confirm it recovers.
    chk.ok(dev.wait_ready(), "device responsive after test pulse")


@test("inputs", mutating=True, flash=2)
def dac_hw_mute_config_roundtrip(dev, profile, chk):
    """0xEA/0xEB: a valid config round-trips through the directory and restores (flash)."""
    before = dev.get(OP.GET_DAC_HW_MUTE_CONFIG, 16)
    free = None
    from .outputs import _busy_pins
    busy = _busy_pins(dev, profile)
    for p in (11, 16, 17, 18, 19, 20):
        if p not in busy and p <= 29:
            free = p
            break
    if free is None:
        chk.note("no free pin for DAC mute config test")
        return
    try:
        cfg = bytearray(16)
        cfg[0] = 1          # enabled
        cfg[1] = 1          # active_low
        cfg[2] = free       # pin
        struct.pack_into("<H", cfg, 4, 7)    # hold_ms
        struct.pack_into("<H", cfg, 6, 3)    # release_ms
        dev.set(OP.SET_DAC_HW_MUTE_CONFIG, bytes(cfg))
        got = dev.get_ready(OP.GET_DAC_HW_MUTE_CONFIG, 16)
        chk.eq(got[0], 1, "enabled echoed")
        chk.eq(got[2], free, "pin echoed")
        chk.eq(struct.unpack_from("<H", got, 4)[0], 7, "hold_ms echoed")
        chk.eq(struct.unpack_from("<H", got, 6)[0], 3, "release_ms echoed")
    finally:
        dev.set(OP.SET_DAC_HW_MUTE_CONFIG, bytes(before))
        dev.wait_ready()
