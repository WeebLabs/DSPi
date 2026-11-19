"""
Input source switching, S/PDIF and I2S receiver pins, LG Sound Sync, the DAC hardware mute and source names
(INPUT.*, OUTPUT.DAC_MUTE_*). Source switches restore the original source in a finally block, and pin moves
run only on a source that does not use the pin and only when the original pin can be put back. Input block
switches and the S/PDIF count are restored after the source, since a block cannot drop a live source.
"""

import struct
import time

from ..device import Timeout
from ..framework import Skip, test
from ..helpers import (INPUT_CONFIG_OFF, TYPE_I2S, TYPE_SPDIF, bool_roundtrip, expect_result, expect_stall,
                       free_gpio, gpio_owners, has_command, held_by_others, input_blocks, invalid_gpios, poll_type,
                       push_bulk, read_bulk, restore_input_blocks, set_type, settle, spdif_rx_config, spdif_source,
                       try_set, wait_for)
from ..profile import MAX_GPIO

INPUT_USB, INPUT_SPDIF, INPUT_I2S, INPUT_ADAT = 0, 1, 2, 3
INPUT_SPDIF2, INPUT_SPDIF3, INPUT_SPDIF4 = 4, 5, 6
INPUT_SOURCE_MAX = INPUT_SPDIF4
INPUT_ALL = (INPUT_USB, INPUT_SPDIF, INPUT_I2S, INPUT_ADAT, INPUT_SPDIF2, INPUT_SPDIF3, INPUT_SPDIF4)
INPUT_INVALID = INPUT_SOURCE_MAX + 1


def _source(dev):
    return dev.pget_u8("INPUT.SOURCE")


def _switch_source(dev, target, timeout_s=2.0):
    """Queue a source switch (deferred, muted pipeline reset) and wait for it; returns the source in force.
    A timeout in the restart window is retried; a refusal raises."""
    for _ in range(3):
        try:
            dev.pset_u8("INPUT.SOURCE", target)
        except Timeout:
            dev.wait_ready()
            continue
        if wait_for(lambda: _source(dev), target, timeout_s) == target:
            break
    return wait_for(lambda: _source(dev), target, 0.5)


def _i2s_block_on(dev, chk):
    """Switch the I2S block on so I2S can be selected; True when on. A pin refusal is noted, not failed."""
    r = try_set(dev, "INPUT.I2S_RX_ENABLE", b"\x01")
    if r != "VC_OK":
        chk.member(r, ("VC_ERR_PIN_IN_USE", "VC_ERR_PIN_INVALID"), "I2S block switch-on refusal")
        chk.note(f"I2S block could not switch on ({r}); I2S cases skipped")
    return r == "VC_OK"


@test("inputs", mutating=True)
def input_source_switch_roundtrip(dev, profile, chk):
    """INPUT.SOURCE: switch USB <-> S/PDIF and back (deferred, mute and reset); a source past 6 is VC_ERR_VALUE."""
    orig = _source(dev)
    other = INPUT_USB if orig == INPUT_SPDIF else INPUT_SPDIF
    if other == INPUT_SPDIF and not spdif_rx_config(dev)["enabled"]:
        raise Skip("the S/PDIF input block is off")
    try:
        chk.eq(_switch_source(dev, other), other, f"switched to {other}")
        expect_stall(chk, lambda: dev.pset_u8("INPUT.SOURCE", INPUT_INVALID), "VC_ERR_VALUE",
                     f"source {INPUT_INVALID}")
        time.sleep(0.05)
        chk.eq(_source(dev), other, "refused source left the switch alone")
    finally:
        chk.eq(_switch_source(dev, orig), orig, "input source restored")


@test("inputs", mutating=True)
def input_source_i2s_switch(dev, profile, chk):
    """INPUT.SOURCE: switch to I2S input and back.
    The device is the clock master, so nothing need be connected; the liveness sentinel catches a wedge."""
    orig = _source(dev)
    if orig == INPUT_I2S:
        chk.note("already on I2S input")
        return
    saved = input_blocks(dev)
    try:
        if _i2s_block_on(dev, chk):
            chk.eq(_switch_source(dev, INPUT_I2S), INPUT_I2S, "switched to I2S")
    finally:
        chk.eq(_switch_source(dev, orig), orig, "input source restored")
        restore_input_blocks(dev, saved)


@test("inputs", mutating=True)
def i2s_input_output_type_switch_stress(dev, profile, chk):
    """Output type switches, and the return to USB, while on I2S input must not hang.
    Each tears down the chained capture DMA and re-elects the input clock role; the sentinel catches a hang."""
    orig_src = _source(dev)
    orig_t0 = dev.pget_u8("OUTPUT.TYPE", 0)
    saved = input_blocks(dev)
    try:
        if not _i2s_block_on(dev, chk) or _switch_source(dev, INPUT_I2S) != INPUT_I2S:
            chk.note("could not switch to I2S input; skipping")
            return
        other = TYPE_I2S if orig_t0 == TYPE_SPDIF else TYPE_SPDIF
        for i in range(4):
            t = other if (i % 2 == 0) else orig_t0
            expect_result(chk, lambda: dev.pset_u8("OUTPUT.TYPE", t, 0), "VC_OK", f"type switch #{i} -> {t}")
            chk.ok(poll_type(dev, 0, t), f"slot0 type became {t}")
            chk.ok(dev.wait_ready(), f"device responsive after type switch #{i}")
        # A change on I2S followed by the switch to USB was the second crash path.
        chk.ok(set_type(dev, 0, orig_t0), "slot0 type restored on I2S")
        chk.eq(_switch_source(dev, INPUT_USB), INPUT_USB, "returned to USB after change")
    finally:
        try:
            chk.ok(set_type(dev, 0, orig_t0), "slot0 type restored")
        finally:
            chk.eq(_switch_source(dev, orig_src), orig_src, "input source restored")
            restore_input_blocks(dev, saved)


@test("inputs", mutating=True)
def source_refused_outside_block(dev, profile, chk):
    """INPUT.SOURCE answers VC_ERR_STATE for an S/PDIF input above the count, S/PDIF with its block off, I2S
    with its block off and a disabled ADAT input, and nothing switches. USB source only; restores the blocks."""
    if _source(dev) != INPUT_USB:
        raise Skip("input source is not USB")
    saved = input_blocks(dev)
    E = expect_stall
    try:
        if saved["count"] == 4:
            dev.pset_u8("INPUT.SPDIF_RX_COUNT", 3)
        n = dev.pget_u8("INPUT.SPDIF_RX_COUNT")
        E(chk, lambda: dev.pset_u8("INPUT.SOURCE", spdif_source(n)), "VC_ERR_STATE",
          f"S/PDIF input {n + 1} above count {n}")
        dev.pset_u8("INPUT.SPDIF_RX_ENABLE", 0)
        E(chk, lambda: dev.pset_u8("INPUT.SOURCE", INPUT_SPDIF), "VC_ERR_STATE", "S/PDIF input 1, block off")
        dev.pset_u8("INPUT.I2S_RX_ENABLE", 0)
        E(chk, lambda: dev.pset_u8("INPUT.SOURCE", INPUT_I2S), "VC_ERR_STATE", "I2S, block off")
        if has_command(dev, "INPUT.ADAT_RX_ENABLE") and not saved["adat"]:
            E(chk, lambda: dev.pset_u8("INPUT.SOURCE", INPUT_ADAT), "VC_ERR_STATE", "ADAT, disabled")
        time.sleep(0.05)
        chk.eq(_source(dev), INPUT_USB, "no refused switch went through")
    finally:
        restore_input_blocks(dev, saved)
        chk.eq(input_blocks(dev), saved, "input blocks restored")


@test("inputs", mutating=True)
def block_cut_refused_while_live(dev, profile, chk):
    """With S/PDIF input 2, then I2S, as the source (first right after the SET, likely still pending, then once
    applied), switching its block off, or an S/PDIF count below it, is VC_ERR_STATE and changes nothing."""
    orig = _source(dev)
    saved = input_blocks(dev)

    def refused(cmd, val, what):
        before = input_blocks(dev)
        expect_stall(chk, lambda: dev.pset_u8(cmd, val), "VC_ERR_STATE", what)
        chk.eq(input_blocks(dev), before, f"{what}: blocks unchanged")

    def cuts(src, when):
        if src == INPUT_SPDIF2:
            refused("INPUT.SPDIF_RX_COUNT", 1, f"S/PDIF count 1 {when}")
            refused("INPUT.SPDIF_RX_ENABLE", 0, f"S/PDIF block off {when}")
        else:
            refused("INPUT.I2S_RX_ENABLE", 0, f"I2S block off {when}")
    try:
        if saved["spdif"]:
            spdif_ok = saved["count"] >= 2 or try_set(dev, "INPUT.SPDIF_RX_COUNT", b"\x02") == "VC_OK"
        else:
            dev.pset_u8("INPUT.SPDIF_RX_COUNT", max(2, saved["count"]))
            spdif_ok = try_set(dev, "INPUT.SPDIF_RX_ENABLE", b"\x01") == "VC_OK"
        if not spdif_ok:
            chk.note("S/PDIF block with input 2 could not switch on; S/PDIF cases skipped")
        for src, ready in ((INPUT_SPDIF2, spdif_ok), (INPUT_I2S, None)):
            if ready is None:
                ready = _i2s_block_on(dev, chk)
            if not ready:
                continue
            dev.pset_u8("INPUT.SOURCE", src)
            cuts(src, f"right after selecting source {src}")
            if chk.eq(_switch_source(dev, src), src, f"switched to source {src}"):
                cuts(src, f"with source {src} active")
    finally:
        chk.eq(_switch_source(dev, orig), orig, "input source restored")
        restore_input_blocks(dev, saved)
        chk.eq(input_blocks(dev), saved, "input blocks restored")


@test("inputs", mutating=True)
def bulk_push_keeps_live_spdif_block(dev, profile, chk):
    """On S/PDIF input 1, a bulk SET that clears the S/PDIF switch keeps the block on and the source
    (output-config mode WITH_PRESET only). Restores the blob and the source."""
    if dev.pget_u8("STORE.OUTPUT_CONFIG_MODE") != 1:
        raise Skip("output-config mode is not WITH_PRESET: a bulk SET leaves the input fields live")
    saved = input_blocks(dev)
    if not saved["spdif"]:
        raise Skip("the S/PDIF input block is off")
    orig = _source(dev)
    blob0 = read_bulk(dev, profile)
    try:
        if not chk.eq(_switch_source(dev, INPUT_SPDIF), INPUT_SPDIF, "switched to S/PDIF input 1"):
            return
        b = bytearray(read_bulk(dev, profile))
        b[INPUT_CONFIG_OFF + 14] = ((b[INPUT_CONFIG_OFF + 14] - 1) & 6) + 1     # input_enable_p1, S/PDIF bit off
        push_bulk(dev, bytes(b))
        chk.eq(dev.pget_u8("INPUT.SPDIF_RX_ENABLE"), 1, "S/PDIF block kept on")
        chk.eq(_source(dev), INPUT_SPDIF, "source kept")
    finally:
        push_bulk(dev, blob0)
        chk.eq(_switch_source(dev, orig), orig, "input source restored")
        chk.eq(input_blocks(dev), saved, "input blocks restored")


@test("inputs")
def spdif_rx_status_plausible(dev, profile, chk):
    """INPUT.SPDIF_RX_STATUS has in-range fields and a known sample rate (idle with no cable is fine)."""
    data = dev.pget("INPUT.SPDIF_RX_STATUS", length=16)
    chk.eq(len(data), 16, "RX status length")
    state, in_src, lock_cnt, loss_cnt = data[0], data[1], data[2], data[3]
    sample_rate = struct.unpack_from("<I", data, 4)[0]
    fifo = struct.unpack_from("<H", data, 12)[0]
    chk.in_range(state, 0, 3, "RX state enum")
    chk.member(in_src, INPUT_ALL, "RX input_source")
    chk.in_range(fifo, 0, 100, "RX fifo fill %")
    chk.member(sample_rate, (0, 44100, 48000, 88200, 96000), "RX sample rate")
    chk.note(f"RX state={state} src={in_src} rate={sample_rate} fifo={fifo}% locks={lock_cnt} losses={loss_cnt}")


def _movable(chk, dev, profile, orig, me, what):
    """A free GPIO to move `me` to, or None (with a note) when there is none or `orig` could not be restored."""
    owners = gpio_owners(dev, profile)
    others = held_by_others(owners, (orig,), me)
    if others:
        chk.note(f"{what} GPIO {orig} is also held by {sorted(others)}; not moving what cannot go back")
        return None
    free = free_gpio(dev, profile, prefer=[p for p in (16, 17, 18, 19, 20) if p not in owners])
    if free is None:
        chk.note(f"no free GPIO for the {what} move")
    return free


@test("inputs", mutating=True)
def spdif_rx_pin_validation(dev, profile, chk):
    """INPUT.SPDIF_RX_PIN input 0 refuses invalid and held GPIOs and moves to a free one and back (USB input).
    Invalid GPIOs are VC_ERR_PIN_INVALID, an output's GPIO VC_ERR_PIN_IN_USE (block on), the current GPIO VC_OK."""
    pin = lambda: dev.pget_u8("INPUT.SPDIF_RX_PIN", 0)  # noqa: E731
    orig = pin()
    for g in invalid_gpios(profile):
        expect_stall(chk, lambda: dev.pset_u8("INPUT.SPDIF_RX_PIN", g, 0), "VC_ERR_PIN_INVALID", f"GPIO {g}")
    expect_result(chk, lambda: dev.pset_u8("INPUT.SPDIF_RX_PIN", orig, 0), "VC_OK", "same pin")
    out_pin = dev.pget_u8("OUTPUT.PIN", 0)
    if spdif_rx_config(dev)["enabled"]:
        expect_stall(chk, lambda: dev.pset_u8("INPUT.SPDIF_RX_PIN", out_pin, 0), "VC_ERR_PIN_IN_USE",
                     f"GPIO {out_pin} held by output 0")
    else:
        chk.note("S/PDIF block off: input 0 only stores its pin, held-GPIO refusal not probed")
    chk.eq(pin(), orig, "pin unchanged by refused SETs")
    if _source(dev) != INPUT_USB:
        chk.note("not on USB input: live S/PDIF RX pin move skipped")
        return
    free = _movable(chk, dev, profile, orig, "spdif_rx0", "S/PDIF RX")
    if free is None:
        return
    try:
        expect_result(chk, lambda: dev.pset_u8("INPUT.SPDIF_RX_PIN", free, 0), "VC_OK", f"RX move -> {free}")
        chk.eq(pin(), free, "RX pin reflects move")
    finally:
        dev.pset_u8("INPUT.SPDIF_RX_PIN", orig, 0)
        settle(dev)
        chk.eq(pin(), orig, "RX pin restored")


@test("inputs", mutating=True)
def i2s_rx_pin_validation(dev, profile, chk):
    """INPUT.I2S_RX_PIN pair 0 refuses invalid and held GPIOs and moves to a free one and back (not on I2S).
    Invalid GPIOs are VC_ERR_PIN_INVALID; with the I2S block on an output's or S/PDIF input 0's GPIO is
    VC_ERR_PIN_IN_USE."""
    pin = lambda: dev.pget_u8("INPUT.I2S_RX_PIN", 0)  # noqa: E731
    orig = pin()
    for g in invalid_gpios(profile):
        expect_stall(chk, lambda: dev.pset_u8("INPUT.I2S_RX_PIN", g, 0), "VC_ERR_PIN_INVALID", f"GPIO {g}")
    expect_result(chk, lambda: dev.pset_u8("INPUT.I2S_RX_PIN", orig, 0), "VC_OK", "same pin")
    # Pair 0 claims its pin only while the I2S block is on; otherwise it stores any valid GPIO.
    if dev.pget_u8("INPUT.I2S_RX_ENABLE"):
        out_pin = dev.pget_u8("OUTPUT.PIN", 0)
        expect_stall(chk, lambda: dev.pset_u8("INPUT.I2S_RX_PIN", out_pin, 0), "VC_ERR_PIN_IN_USE",
                     f"GPIO {out_pin} held by output 0")
        spdif_pin = dev.pget_u8("INPUT.SPDIF_RX_PIN", 0)
        if spdif_pin != orig and spdif_rx_config(dev)["enabled"]:
            expect_stall(chk, lambda: dev.pset_u8("INPUT.I2S_RX_PIN", spdif_pin, 0), "VC_ERR_PIN_IN_USE",
                         f"GPIO {spdif_pin} held by S/PDIF input 0")
    chk.eq(pin(), orig, "pin unchanged by refused SETs")
    if _source(dev) == INPUT_I2S:
        chk.note("on I2S input: live I2S RX pin move skipped")
        return
    free = _movable(chk, dev, profile, orig, "i2s_rx0", "I2S RX")
    if free is None:
        return
    try:
        expect_result(chk, lambda: dev.pset_u8("INPUT.I2S_RX_PIN", free, 0), "VC_OK", f"I2S RX move -> {free}")
        chk.eq(pin(), free, "I2S RX pin reflects move")
    finally:
        dev.pset_u8("INPUT.I2S_RX_PIN", orig, 0)
        settle(dev)
        chk.eq(pin(), orig, "I2S RX pin restored")


@test("inputs", mutating=True)
def lg_sound_sync_enable_and_status(dev, profile, chk):
    """INPUT.LG_SOUND_SYNC_ENABLE normalises to 0/1; LG_SOUND_SYNC_STATUS echoes it, volume 0xFF until decoded."""
    orig = dev.pget_u8("INPUT.LG_SOUND_SYNC_ENABLE")
    try:
        bool_roundtrip(dev, chk, "INPUT.LG_SOUND_SYNC_ENABLE", label="LG enable")
        st = dev.pget("INPUT.LG_SOUND_SYNC_STATUS", length=16)
        chk.eq(len(st), 16, "LG status length")
        enabled, present, volume, muted = st[0], st[1], st[2], st[3]
        chk.eq(enabled, dev.pget_u8("INPUT.LG_SOUND_SYNC_ENABLE"), "LG status enabled matches the enable")
        chk.member(present, (0, 1), "LG present")
        chk.member(muted, (0, 1), "LG muted")
        chk.ok(volume == 0xFF or volume <= 100, f"LG volume {volume} (0xFF sentinel or 0..100)")
        chk.ok(all(b == 0 for b in st[4:16]), "LG status reserved zero")
    finally:
        dev.pset_u8("INPUT.LG_SOUND_SYNC_ENABLE", orig)


@test("inputs")
def dac_hw_mute_config_get(dev, profile, chk):
    """OUTPUT.DAC_MUTE_CONFIG is 16 bytes: pin a GPIO or 0xFF, hold 1..500 ms when enabled, release 0..500 ms."""
    cfg = dev.pget("OUTPUT.DAC_MUTE_CONFIG", length=16)
    chk.eq(len(cfg), 16, "DAC mute config length")
    enabled, active_low, pin = cfg[0], cfg[1], cfg[2]
    hold, release = struct.unpack_from("<HH", cfg, 4)
    chk.member(enabled, (0, 1), "DAC mute enabled")
    chk.member(active_low, (0, 1), "DAC mute active_low")
    chk.ok(pin == 0xFF or pin <= MAX_GPIO[profile.platform_id], f"DAC mute pin {pin}")
    if enabled:
        chk.in_range(hold, 1, 500, "DAC mute hold_ms")
    chk.in_range(release, 0, 500, "DAC mute release_ms")
    chk.note(f"DAC mute enabled={enabled} active_low={active_low} pin={pin} hold={hold} release={release}")


@test("inputs", mutating=True)
def dac_hw_mute_test_pulse(dev, profile, chk):
    """OUTPUT.DAC_MUTE_TEST returns promptly: VC_OK when the feature is enabled, else VC_ERR_STATE."""
    want = "VC_OK" if dev.pget("OUTPUT.DAC_MUTE_CONFIG", length=16)[0] else "VC_ERR_STATE"
    t0 = time.monotonic()
    r = try_set(dev, "OUTPUT.DAC_MUTE_TEST")
    dt = time.monotonic() - t0
    chk.eq(r, want, "test pulse result")
    chk.ok(dt < 0.5, f"test command returned promptly ({dt*1000:.0f} ms, not blocking for the pulse)")
    chk.ok(dev.wait_ready(), "device responsive after test pulse")


@test("inputs", mutating=True, flash=2)
def dac_hw_mute_config_roundtrip(dev, profile, chk):
    """OUTPUT.DAC_MUTE_CONFIG: a valid config on a free GPIO reads back once applied, then restores (flash)."""
    cfg_now = lambda: dev.pget("OUTPUT.DAC_MUTE_CONFIG", length=16)  # noqa: E731
    before = cfg_now()
    owners = gpio_owners(dev, profile)
    free = free_gpio(dev, profile, prefer=[p for p in (11, 16, 17, 18, 19, 20) if p not in owners])
    if free is None:
        chk.note("no free pin for DAC mute config test")
        return
    try:
        cfg = bytearray(16)
        cfg[0], cfg[1], cfg[2] = 1, 1, free     # enabled, active low, pin
        struct.pack_into("<HH", cfg, 4, 7, 3)    # hold, release ms
        dev.pset("OUTPUT.DAC_MUTE_CONFIG", bytes(cfg))
        got = wait_for(cfg_now, bytes(cfg), timeout_s=3.0)
        chk.eq(got, bytes(cfg), "config applied and reads back")
    finally:
        dev.pset("OUTPUT.DAC_MUTE_CONFIG", before)
        settle(dev)
        chk.eq(wait_for(cfg_now, before, timeout_s=3.0), before, "config restored")


def _source_name(dev, src):
    return dev.pget_ready("INPUT.SOURCE_NAME", src, 32)


@test("inputs", mutating=True, flash=8)
def input_source_name_roundtrip(dev, profile, chk):
    """INPUT.SOURCE_NAME round-trips, keeps 31 bytes, clears on a leading NUL and survives a preset load (flash).
    A source past 6, or a high address byte, is VC_ERR_ADDRESS both ways."""
    touched = (INPUT_USB, INPUT_SPDIF, INPUT_SOURCE_MAX)
    orig = {src: _source_name(dev, src) for src in touched}
    for src, raw in orig.items():
        chk.eq(len(raw), 32, f"source {src} GET is 32 bytes")
    try:
        for src in touched:
            dev.pset("INPUT.SOURCE_NAME", b"QA Src %d" % src, src)
        for src in touched:
            chk.eq(_source_name(dev, src).split(b"\x00", 1)[0], b"QA Src %d" % src,
                   f"source {src} name round-trip")

        dev.pset("INPUT.SOURCE_NAME", b"A" * 32, INPUT_USB)
        chk.eq(_source_name(dev, INPUT_USB), b"A" * 31 + b"\x00", "32-byte payload keeps 31 + NUL")
        dev.pset("INPUT.SOURCE_NAME", b"\x00junk", INPUT_USB)
        chk.eq(_source_name(dev, INPUT_USB), bytes(32), "leading NUL clears to all zeros")

        for addr in (INPUT_INVALID, 0x100):
            expect_stall(chk, lambda: dev.pget("INPUT.SOURCE_NAME", addr, 32), "VC_ERR_ADDRESS",
                         f"GET source {addr:#x}")
            expect_stall(chk, lambda: dev.pset("INPUT.SOURCE_NAME", b"alias", addr), "VC_ERR_ADDRESS",
                         f"SET source {addr:#x}")
        chk.eq(_source_name(dev, INPUT_USB), bytes(32), "a high address byte does not alias source 0")

        occupied = struct.unpack_from("<H", dev.pget("STORE.DIRECTORY", length=7))[0]
        active = dev.pget_u8("STORE.PRESET_ACTIVE")
        if occupied & (1 << active):
            dev.pset("STORE.PRESET_LOAD", b"", active)
            settle(dev)
            chk.eq(_source_name(dev, INPUT_SPDIF).split(b"\x00", 1)[0], b"QA Src %d" % INPUT_SPDIF,
                   "name survives a preset load")
        else:
            chk.note("active preset slot empty; skipping the preset-load check")
    finally:
        for src, raw in orig.items():
            dev.pset("INPUT.SOURCE_NAME", raw.split(b"\x00", 1)[0] or b"\x00", src)
        settle(dev)
