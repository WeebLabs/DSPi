"""
Channel names, the STORE.ALL_PARAMS header and live apply, and how a preset load treats live IO and the limiter
under each STORE.OUTPUT_CONFIG_MODE. Flash tests (--allow-flash) use one scratch slot, delete it even when a
step fails, and push the pre-test live state back.
"""

import struct

from ..framework import test
from ..helpers import expect_stall, push_bulk, read_bulk, settle, wait_for

PRESET_SLOTS = 10


def _occupied(dev):
    return struct.unpack_from("<H", dev.pget_ready("STORE.DIRECTORY", length=7))[0]


def _scratch_slot(dev):
    occ = _occupied(dev)
    return next((s for s in range(PRESET_SLOTS - 1, -1, -1) if not occ & (1 << s)), None)


def _slot_used(dev, slot):
    return (_occupied(dev) >> slot) & 1


def _drop_scratch(dev, slot):
    """Delete the scratch preset if a save reached flash; raise naming the slot if it will not go."""
    dev.wait_ready()
    wait_for(lambda: _slot_used(dev, slot), 1, timeout_s=1.0)   # a queued save may still be landing
    if not _slot_used(dev, slot):
        return
    dev.pset("STORE.PRESET_DELETE", b"", slot)
    if wait_for(lambda: _slot_used(dev, slot), 0) != 0:
        raise AssertionError(f"scratch preset slot {slot} is still occupied after cleanup: delete it by hand")


def _run_all(*steps):
    """Run every cleanup step even when an earlier one raises, then re-raise the first failure."""
    first = None
    for step in steps:
        try:
            step()
        except Exception as e:  # noqa: BLE001
            first = first or e
    if first is not None:
        raise first


def _mode(dev):
    return dev.pget_ready("STORE.OUTPUT_CONFIG_MODE", length=1)[0]


def _set_mode(dev, mode):
    dev.pset_u8("STORE.OUTPUT_CONFIG_MODE", mode)
    settle(dev)
    return wait_for(lambda: _mode(dev), mode)


def _load(dev, slot):
    dev.pset("STORE.PRESET_LOAD", b"", slot)
    settle(dev, 0.3)


# --- Non-flash --------------------------------------------------------------

@test("presets", mutating=True)
def channel_name_roundtrip(dev, profile, chk):
    """ROUTE.CHANNEL_NAME round-trips in RAM and keeps 31 of 32 bytes; longer names change nothing.
    33 or more bytes is VC_ERR_LENGTH; a channel past the end is VC_ERR_ADDRESS."""
    ch = 2
    name = lambda: dev.pget("ROUTE.CHANNEL_NAME", ch, 32)  # noqa: E731
    orig = name()
    try:
        dev.pset("ROUTE.CHANNEL_NAME", b"QA Test Name", ch)
        chk.eq(name().split(b"\x00", 1)[0], b"QA Test Name", "channel name round-trip")
        dev.pset("ROUTE.CHANNEL_NAME", b"A" * 32, ch)
        chk.eq(name(), b"A" * 31 + b"\x00", "32-byte name keeps 31 + NUL")
        before = name()
        for n in (33, 40):
            expect_stall(chk, lambda: dev.pset("ROUTE.CHANNEL_NAME", b"B" * n, ch), "VC_ERR_LENGTH",
                         f"{n}-byte name")
            chk.eq(name(), before, f"{n}-byte name left the stored name alone")
        n_ch = profile.num_channels
        expect_stall(chk, lambda: dev.pget("ROUTE.CHANNEL_NAME", n_ch, 32), "VC_ERR_ADDRESS", "GET ch==count")
        expect_stall(chk, lambda: dev.pset("ROUTE.CHANNEL_NAME", b"x", n_ch), "VC_ERR_ADDRESS", "SET ch==count")
    finally:
        dev.pset("ROUTE.CHANNEL_NAME", orig.split(b"\x00", 1)[0] or b"\x00", ch)


@test("presets")
def bulk_get_header(dev, profile, chk):
    """STORE.ALL_PARAMS header reports the wire format, platform, channel geometry and payload length."""
    hdr = dev.pget("STORE.ALL_PARAMS", length=16)
    (fmt, plat, nch, nout, nin, mbands, plen) = struct.unpack_from("<BBBBBBH", hdr, 0)
    chk.eq(fmt, profile.wire_format_version, "format version")
    chk.eq(plat, profile.platform_id, "platform id")
    chk.eq(nch, profile.num_channels, "num_channels")
    chk.eq(nout, profile.num_output_channels, "num_output_channels (SYS.PLATFORM)")
    chk.eq(nin, profile.num_input_channels, "num_input_channels")
    chk.eq(plen, profile.bulk_payload_len, "payload length")


@test("presets", mutating=True)
def bulk_roundtrip_apply(dev, profile, chk):
    """STORE.ALL_PARAMS apply reaches a fixpoint, and a live change shows in the next snapshot.
    The first apply may clamp fields. A re-enumeration mid-apply reloads the preset, so it is not compared."""
    d = None
    for attempt in range(3):
        a = read_bulk(dev, profile)
        push_bulk(dev, a, settle_s=0.1)
        c = read_bulk(dev, profile)
        reenum0 = dev.reenumerations
        push_bulk(dev, c, settle_s=0.1)
        d = read_bulk(dev, profile)
        if dev.reenumerations != reenum0:
            chk.note(f"re-enumeration during bulk apply (attempt {attempt}); device reloaded preset, retrying")
            continue
        chk.eq(c, d, "bulk apply reaches a stable fixpoint (idempotent round-trip)")
        break
    else:
        chk.note("could not complete a re-enum-free bulk fixpoint in 3 tries")
        chk.ok(dev.is_alive(), "device still responsive after bulk-apply re-enum churn")
    byp = dev.pget_u8("FILTER.EQ_BYPASS")
    try:
        dev.pset_u8("FILTER.EQ_BYPASS", 0 if byp else 1)
        chk.ne(read_bulk(dev, profile), d, "bulk snapshot reflects a live param change")
    finally:
        dev.pset_u8("FILTER.EQ_BYPASS", byp)


# --- Flash-gated ------------------------------------------------------------

def _mode_isolation(dev, profile, chk, what, get, put, slot_val, live_val):
    """Save `slot_val` into a scratch preset, then load it in mode 0 (live `live_val` must survive) and in
    mode 1 (the slot's value must apply). Cleans up the scratch slot, the mode and the live state."""
    scratch = _scratch_slot(dev)
    if scratch is None:
        chk.note(f"all preset slots occupied, skipping {what} isolation test")
        return
    live = read_bulk(dev, profile)
    orig_mode = _mode(dev)
    orig_active = dev.pget_u8("STORE.PRESET_ACTIVE")
    saved = False
    try:
        put(slot_val)
        saved = True
        dev.pset("STORE.PRESET_SAVE", b"", scratch)
        chk.eq(wait_for(lambda: _slot_used(dev, scratch), 1), 1, "scratch preset saved")

        chk.eq(_set_mode(dev, 0), 0, "mode 0 (independent)")
        put(live_val)
        _load(dev, scratch)
        chk.eq(get(), live_val, f"independent: preset load left the live {what} unchanged")

        chk.eq(_set_mode(dev, 1), 1, "mode 1 (with preset)")
        put(live_val)
        _load(dev, scratch)
        chk.eq(wait_for(get, slot_val), slot_val, f"with-preset: preset load applied the slot's {what}")
    finally:
        # Reload the original slot so the scratch is no longer active, then drop it and push the live state.
        _run_all(lambda: _set_mode(dev, orig_mode),
                 lambda: _load(dev, orig_active),
                 lambda: _drop_scratch(dev, scratch) if saved else None,
                 lambda: push_bulk(dev, live))


@test("presets", mutating=True, flash=10)
def output_config_independent_isolation(dev, profile, chk):
    """A preset load leaves live IO alone in INDEPENDENT mode and applies the slot's IO in WITH_PRESET mode.
    The marker is CLOCK.MCK_MULTIPLIER: part of the IO config but inert while MCK is off."""
    mult = dev.pget_u8("CLOCK.MCK_MULTIPLIER")
    _mode_isolation(dev, profile, chk, "MCK multiplier",
                    lambda: dev.pget_u8("CLOCK.MCK_MULTIPLIER"),
                    lambda v: dev.pset_u8("CLOCK.MCK_MULTIPLIER", v), 1 - mult, mult)


@test("presets", mutating=True, flash=10)
def limiter_follows_output_config_mode(dev, profile, chk):
    """The output limiter follows STORE.OUTPUT_CONFIG_MODE across a preset load, like the IO config.
    The marker is the last output's DYN.LIMITER_THRESHOLD with its limiter off; OUTPUT_CONFIG_SAVE is never sent."""
    out = profile.num_output_channels - 1
    _mode_isolation(dev, profile, chk, "limiter threshold",
                    lambda: round(dev.pget_f32("DYN.LIMITER_THRESHOLD", out), 3),
                    lambda v: dev.pset_f32("DYN.LIMITER_THRESHOLD", v, out), -17.0, -5.0)
