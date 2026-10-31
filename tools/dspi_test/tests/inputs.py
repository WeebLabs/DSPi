"""
Inputs / SPDIF-RX / I2S-RX / LG Sound Sync / DAC hardware mute group.

Input source        0xE0/0xE1
SPDIF RX status     0xE2 (16B) / channel status 0xE3 (24B)
SPDIF RX pin        0xE4/0xE5
I2S input rate      0xED/0xEE (uint32 Hz set / {current,selected} 8B get)
I2S RX data pin     0xF1/0xF2
LG Sound Sync       0xE6/0xE7 / status 0xE8 (16B)
DAC hardware mute   0xEA/0xEB (config 16B) / 0xEC test pulse
Source names        0x54/0x55 (32B, device-global)
"""

import struct
import time

from ..device import OP, Stall
from ..framework import test
from ..helpers import bool_roundtrip

PIN_SUCCESS, PIN_INVALID_PIN, PIN_IN_USE, PIN_INVALID_OUTPUT, PIN_OUTPUT_ACTIVE = range(5)
INPUT_USB, INPUT_SPDIF, INPUT_I2S, INPUT_ADAT = 0, 1, 2, 3
# Optional SPDIF inputs 2..4 (contiguous from 4); selectable only once enabled.
INPUT_SPDIF2, INPUT_SPDIF3, INPUT_SPDIF4 = 4, 5, 6
INPUT_SOURCE_MAX = INPUT_SPDIF4
# Every structurally valid source, whether or not it is currently selectable.
INPUT_ALL = (INPUT_USB, INPUT_SPDIF, INPUT_I2S, INPUT_ADAT,
             INPUT_SPDIF2, INPUT_SPDIF3, INPUT_SPDIF4)
# First value past INPUT_SOURCE_MAX; must be rejected/ignored.
INPUT_INVALID = INPUT_SOURCE_MAX + 1
I2S_RATES = (44100, 48000, 96000)


def _switch_source(dev, target, timeout_s=1.5):
    """Request a deferred input-source switch and wait for it to apply."""
    dev.set_u8(OP.SET_INPUT_SOURCE, target)
    dev.wait_ready()
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and dev.get_u8(OP.GET_INPUT_SOURCE) != target:
        time.sleep(0.03)


@test("inputs")
def input_source_get(dev, profile, chk):
    """0xE1 returns a valid InputSource enum value."""
    chk.member(dev.get_u8(OP.GET_INPUT_SOURCE), INPUT_ALL, "input source")


@test("inputs", mutating=True)
def input_source_switch_roundtrip(dev, profile, chk):
    """0xE0/0xE1: switch USB<->SPDIF and back (deferred, mute+reset); out-of-range dropped."""
    orig = dev.get_u8(OP.GET_INPUT_SOURCE)
    other = INPUT_USB if orig == INPUT_SPDIF else INPUT_SPDIF
    try:
        _switch_source(dev, other)
        chk.eq(dev.get_u8(OP.GET_INPUT_SOURCE), other, f"switched to {other}")
        # Out-of-range source (past INPUT_SOURCE_MAX) silently dropped (no STALL, no change).
        chk.no_stall(lambda: dev.set_u8(OP.SET_INPUT_SOURCE, INPUT_INVALID), "invalid source no STALL")
        time.sleep(0.05)
        chk.eq(dev.get_u8(OP.GET_INPUT_SOURCE), other, "invalid source ignored")
    finally:
        _switch_source(dev, orig)
        chk.eq(dev.get_u8(OP.GET_INPUT_SOURCE), orig, "input source restored")


@test("inputs", mutating=True)
def input_source_i2s_switch(dev, profile, chk):
    """0xE0/0xE1: switch to I2S input and back.

    The device is the I2S clock master, so no external source need be connected;
    the input SM simply drives BCK/LRCLK and reads the (idle) data pin. The
    liveness sentinel confirms the deferred mute+reset switch did not wedge.
    """
    orig = dev.get_u8(OP.GET_INPUT_SOURCE)
    if orig == INPUT_I2S:
        chk.note("already on I2S input")
        return
    try:
        _switch_source(dev, INPUT_I2S)
        chk.eq(dev.get_u8(OP.GET_INPUT_SOURCE), INPUT_I2S, "switched to I2S")
    finally:
        _switch_source(dev, orig)
        chk.eq(dev.get_u8(OP.GET_INPUT_SOURCE), orig, "input source restored")


@test("inputs", mutating=True)
def i2s_input_output_type_switch_stress(dev, profile, chk):
    """Regression: output-type switches and the USB return while on I2S input must not hang.

    Reproduces the DMA-ring teardown race that intermittently watchdog-reset the
    device. Every output-type switch (and the switch back to USB) tears down the
    I2S input's self-retriggering chained capture DMA via i2s_input_stop(); doing
    that naively could leave a channel re-arming the other, hanging the abort.
    Each switch also flips the input clock-master/slave role. The runner's
    liveness sentinel after this mutating test catches a hang/crash.
    """
    from .outputs import _poll_type, TYPE_SPDIF, TYPE_I2S  # noqa: F401

    orig_src = dev.get_u8(OP.GET_INPUT_SOURCE)
    orig_t0 = dev.get_u8(OP.GET_OUTPUT_TYPE, wvalue=0)
    try:
        _switch_source(dev, INPUT_I2S)
        if dev.get_u8(OP.GET_INPUT_SOURCE) != INPUT_I2S:
            chk.note("could not switch to I2S input; skipping")
            return
        other = TYPE_I2S if orig_t0 == TYPE_SPDIF else TYPE_SPDIF
        # Toggle slot 0's output type repeatedly while on I2S input. Each
        # switch tears down and rebuilds the capture ring and re-elects the
        # input clock role (master <-> slave); the exact crash trigger.
        for i in range(4):
            t = other if (i % 2 == 0) else orig_t0
            st = dev.get_u8(OP.SET_OUTPUT_TYPE, wvalue=(t << 8) | 0)
            chk.eq(st, PIN_SUCCESS, f"type switch #{i} -> {t} accepted")
            chk.ok(_poll_type(dev, 0, t), f"slot0 type became {t}")
            chk.ok(dev.wait_ready(), f"device responsive after type switch #{i}")
        # Restore slot 0 type, confirm live, THEN return to USB (the second
        # reported crash path: a change on I2S followed by the USB switch).
        dev.get_u8(OP.SET_OUTPUT_TYPE, wvalue=(orig_t0 << 8) | 0)
        chk.ok(_poll_type(dev, 0, orig_t0), "slot0 type restored on I2S")
        _switch_source(dev, INPUT_USB)
        chk.eq(dev.get_u8(OP.GET_INPUT_SOURCE), INPUT_USB, "returned to USB after change")
    finally:
        dev.get_u8(OP.SET_OUTPUT_TYPE, wvalue=(orig_t0 << 8) | 0)
        _poll_type(dev, 0, orig_t0)
        _switch_source(dev, orig_src)
        chk.eq(dev.get_u8(OP.GET_INPUT_SOURCE), orig_src, "input source restored")
        chk.eq(dev.get_u8(OP.GET_OUTPUT_TYPE, wvalue=0), orig_t0, "slot0 type restored")


@test("inputs")
def spdif_rx_status_plausible(dev, profile, chk):
    """0xE2 returns a 16-byte status with in-range fields (idle/no-cable tolerated)."""
    data = dev.get(OP.GET_SPDIF_RX_STATUS, 16)
    chk.eq(len(data), 16, "RX status length")
    state, in_src, lock_cnt, loss_cnt = data[0], data[1], data[2], data[3]
    sample_rate = struct.unpack_from("<I", data, 4)[0]
    fifo = struct.unpack_from("<H", data, 12)[0]
    chk.in_range(state, 0, 3, "RX state enum")
    chk.member(in_src, INPUT_ALL, "RX input_source")
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


def _selected_i2s_rate(dev):
    """Stored I2S input rate (bytes 4..7 of 0xEE); valid on any active source."""
    return struct.unpack("<II", dev.get(OP.GET_INPUT_RATE, 8))[1]


@test("inputs")
def i2s_input_rate_get(dev, profile, chk):
    """0xEE returns 8 bytes: {current pipeline Hz, selected I2S Hz}, both supported rates."""
    data = dev.get(OP.GET_INPUT_RATE, 8)
    chk.eq(len(data), 8, "input rate length")
    current, selected = struct.unpack("<II", data)
    chk.member(current, I2S_RATES, "current pipeline rate")
    chk.member(selected, I2S_RATES, "selected I2S rate")
    chk.note(f"input rate current={current} selected={selected}")


@test("inputs", mutating=True)
def i2s_input_rate_roundtrip(dev, profile, chk):
    """0xED/0xEE: each supported rate stores into the selected field; bad rate ignored.

    Verified via the selected field (bytes 4..7) so the check holds on any active
    source; a live pipeline rate change only happens while I2S is the active input
    (the liveness sentinel then confirms the deferred reset did not wedge).
    """
    orig = _selected_i2s_rate(dev)
    try:
        for hz in I2S_RATES:
            dev.set(OP.SET_INPUT_RATE, struct.pack("<I", hz))
            dev.wait_ready()
            chk.eq(_selected_i2s_rate(dev), hz, f"selected rate -> {hz}")
        # Unsupported rate silently ignored (OUT command has no error channel).
        keep = _selected_i2s_rate(dev)
        chk.no_stall(lambda: dev.set(OP.SET_INPUT_RATE, struct.pack("<I", 88200)),
                     "unsupported rate no STALL")
        dev.wait_ready()
        chk.eq(_selected_i2s_rate(dev), keep, "unsupported rate ignored")
        # Too-short payload ignored, no STALL.
        chk.no_stall(lambda: dev.set(OP.SET_INPUT_RATE, b"\x80\xbb"), "short payload no STALL")
        dev.wait_ready()
        chk.eq(_selected_i2s_rate(dev), keep, "short payload ignored")
    finally:
        dev.set(OP.SET_INPUT_RATE, struct.pack("<I", orig))
        dev.wait_ready()
        chk.eq(_selected_i2s_rate(dev), orig, "selected rate restored")


@test("inputs", mutating=True)
def i2s_rx_pin_validation(dev, profile, chk):
    """0xF1/0xF2: validation status codes (no state change); move tested only off I2S input."""
    orig = dev.get_u8(OP.GET_I2S_RX_PIN)
    # Invalid pin -> INVALID_PIN (no move).
    chk.eq(dev.get_u8(OP.SET_I2S_RX_PIN, wvalue=12), PIN_INVALID_PIN, "pin 12 -> INVALID_PIN")
    # Same pin -> SUCCESS no-op.
    chk.eq(dev.get_u8(OP.SET_I2S_RX_PIN, wvalue=orig), PIN_SUCCESS, "same pin -> SUCCESS")
    # A pin used by an output -> PIN_IN_USE.
    out_pin = dev.get_u8(OP.GET_OUTPUT_PIN, wvalue=0)
    chk.eq(dev.get_u8(OP.SET_I2S_RX_PIN, wvalue=out_pin), PIN_IN_USE, f"pin {out_pin} -> PIN_IN_USE")
    # The SPDIF RX pin is also reserved against the I2S RX pin.
    spdif_pin = dev.get_u8(OP.GET_SPDIF_RX_PIN)
    if spdif_pin != orig:
        chk.eq(dev.get_u8(OP.SET_I2S_RX_PIN, wvalue=spdif_pin), PIN_IN_USE,
               f"SPDIF RX pin {spdif_pin} -> PIN_IN_USE")
    chk.eq(dev.get_u8(OP.GET_I2S_RX_PIN), orig, "pin unchanged by rejected sets")
    # Actual move only when NOT on I2S input (avoids a live hot-swap blackout).
    if dev.get_u8(OP.GET_INPUT_SOURCE) == INPUT_I2S:
        chk.note("on I2S input — skipping live RX pin move to avoid hot-swap")
        return
    from .outputs import _free_pin
    free = _free_pin(dev, profile, prefer=(16, 17, 18, 19, 20))
    if free is None:
        chk.note("no free pin for I2S RX move")
        return
    try:
        chk.eq(dev.get_u8(OP.SET_I2S_RX_PIN, wvalue=free), PIN_SUCCESS, f"I2S RX move -> {free}")
        chk.eq(dev.get_u8(OP.GET_I2S_RX_PIN), free, "I2S RX pin reflects move")
    finally:
        dev.get_u8(OP.SET_I2S_RX_PIN, wvalue=orig)
        chk.eq(dev.get_u8(OP.GET_I2S_RX_PIN), orig, "I2S RX pin restored")


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


def _source_name(dev, src):
    return dev.get_ready(OP.GET_INPUT_SOURCE_NAME, 32, wvalue=src)


@test("inputs", mutating=True, flash=8)
def input_source_name_roundtrip(dev, profile, chk):
    """0x54/0x55 source names: round-trip, 31-char truncation, NUL clears, bad ID STALLs, survive preset load."""
    touched = (INPUT_USB, INPUT_SPDIF, INPUT_SOURCE_MAX)
    orig = {src: _source_name(dev, src) for src in touched}
    for src, raw in orig.items():
        chk.eq(len(raw), 32, f"source {src} GET is 32 bytes")
    try:
        for src in touched:
            dev.set(OP.SET_INPUT_SOURCE_NAME, b"QA Src %d" % src, wvalue=src)
        for src in touched:
            chk.eq(_source_name(dev, src).split(b"\x00", 1)[0], b"QA Src %d" % src,
                   f"source {src} name round-trip")

        dev.set(OP.SET_INPUT_SOURCE_NAME, b"A" * 32, wvalue=INPUT_USB)
        chk.eq(_source_name(dev, INPUT_USB), b"A" * 31 + b"\x00", "32-byte payload keeps 31 + NUL")
        dev.set(OP.SET_INPUT_SOURCE_NAME, b"\x00junk", wvalue=INPUT_USB)
        chk.eq(_source_name(dev, INPUT_USB), bytes(32), "leading NUL clears to all zeros")

        chk.stalls(lambda: dev.get(OP.GET_INPUT_SOURCE_NAME, 32, wvalue=INPUT_INVALID),
                   "GET past INPUT_SOURCE_MAX STALL")
        chk.stalls(lambda: dev.get(OP.GET_INPUT_SOURCE_NAME, 32, wvalue=0x100),
                   "GET with a high wValue byte STALL")
        chk.no_stall(lambda: dev.set(OP.SET_INPUT_SOURCE_NAME, b"x", wvalue=INPUT_INVALID),
                     "SET bad ID no STALL")
        # The high byte must not alias source 0 (USB, cleared just above).
        dev.set(OP.SET_INPUT_SOURCE_NAME, b"alias", wvalue=0x100)
        chk.eq(_source_name(dev, INPUT_USB), bytes(32), "SET with a high wValue byte is ignored")

        directory = dev.get(OP.PRESET_GET_DIR, 7)
        occupied = directory[0] | (directory[1] << 8)
        active = dev.get_u8(OP.PRESET_GET_ACTIVE)
        if occupied & (1 << active):
            dev.get(OP.PRESET_LOAD, 1, wvalue=active)
            dev.wait_ready()
            chk.eq(_source_name(dev, INPUT_SPDIF).split(b"\x00", 1)[0], b"QA Src %d" % INPUT_SPDIF,
                   "name survives a preset load")
        else:
            chk.note("active preset slot empty; skipping the preset-load check")
    finally:
        for src, raw in orig.items():
            dev.set(OP.SET_INPUT_SOURCE_NAME, raw.split(b"\x00", 1)[0] or b"\x00", wvalue=src)
        dev.wait_ready()
