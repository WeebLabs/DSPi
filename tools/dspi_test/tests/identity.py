"""
Identity, meters, clip flags and the device name (SYS.BUILD_INFO, MEAS.LEVELS, INPUT.STATUS,
MEAS.CLIP_FLAGS / CLIP_CLEAR, SYS.DEVICE_NAME). Platform, serial, sizes and the name's length
and address rules are in paged_sys; these add what those do not assert.
"""

import re

from ..framework import test
from ..helpers import input_status, levels, settle, wait_for


@test("identity")
def build_info(dev, profile, chk):
    """SYS.BUILD_INFO: a git describe string, a YYYY-MM-DD build date and four reserved zero bytes."""
    data = dev.pget("SYS.BUILD_INFO", length=64)
    chk.eq(len(data), 64, "build info length")
    if len(data) != 64:
        return
    describe = data[:48].split(b"\0")[0]
    date = data[48:60].split(b"\0")[0]
    chk.ok(describe != b"", "describe string empty")
    chk.ok(all(0x20 <= b < 0x7F for b in describe), f"describe not printable: {describe!r}")
    chk.ok(re.fullmatch(rb"\d{4}-\d{2}-\d{2}", date) is not None,
           f"build date not YYYY-MM-DD: {date!r}")
    chk.eq(data[60:64], b"\0\0\0\0", "reserved tail zero")
    # A published release must be built from a clean tagged checkout; flag a
    # dirty stamp loudly but do not fail (bench builds are expected dirty).
    if b"-dirty" in describe:
        chk.note("DIRTY BUILD: binary was built from uncommitted changes")
    chk.note(f"build {describe.decode('ascii', 'replace')} "
             f"({date.decode('ascii', 'replace')})")


@test("identity")
def status_combined(dev, profile, chk):
    """MEAS.LEVELS carries plausible CPU loads and a live input count of 2, 4, 6 or 8 that INPUT.STATUS agrees with."""
    lv = levels(dev, profile.num_channels)
    chk.eq(len(lv["raw"]), profile.num_channels * 2 + 7, "MEAS.LEVELS length")
    chk.in_range(lv["cpu0_load"], 0, 100, "cpu0 load")
    chk.in_range(lv["cpu1_load"], 0, 100, "cpu1 load")
    chk.member(lv["active_input_channels"], (2, 4, 6, 8), "active input channel count")
    st = input_status(dev)
    chk.eq(st["active_input_channels"], lv["active_input_channels"], "INPUT.STATUS agrees with MEAS.LEVELS")
    chk.note(f"idle peaks max={max(lv['peaks'])} cpu0={lv['cpu0_load']} cpu1={lv['cpu1_load']} "
             f"clip=0x{lv['clip_flags']:08X} active_in={lv['active_input_channels']}")


@test("identity", mutating=True)
def clear_clips_read_then_clear(dev, profile, chk):
    """MEAS.CLIP_FLAGS then MEAS.CLIP_CLEAR: idle, the flags and the MEAS.LEVELS clip field then read 0."""
    first = dev.pget("MEAS.CLIP_FLAGS", length=4)
    chk.eq(len(first), 4, "clip flags length")
    chk.no_stall(lambda: dev.pset("MEAS.CLIP_CLEAR"), "MEAS.CLIP_CLEAR")
    chk.eq(dev.pget_u32("MEAS.CLIP_FLAGS"), 0, "clip flags must be 0 after a clear when idle")
    chk.eq(levels(dev, profile.num_channels)["clip_flags"], 0, "MEAS.LEVELS clip flags 0 after clear")


def _device_name(dev):
    return dev.pget_ready("SYS.DEVICE_NAME", length=32)


@test("identity", mutating=True, flash=5)
def device_name_roundtrip(dev, profile, chk):
    """SYS.DEVICE_NAME is device-wide: it survives a preset load, and a leading NUL clears it to all zeros."""
    orig = _device_name(dev)
    chk.eq(len(orig), 32, "GET is 32 bytes")
    try:
        dev.pset("SYS.DEVICE_NAME", b"QA Board")
        chk.eq(_device_name(dev).split(b"\x00", 1)[0], b"QA Board", "name round-trip")

        directory = dev.pget("STORE.DIRECTORY", length=7)
        occupied = directory[0] | (directory[1] << 8)
        active = dev.pget_u8("STORE.PRESET_ACTIVE")
        if occupied & (1 << active):
            settle(dev)          # the name's own flash write must finish before the load
            dev.pset("STORE.PRESET_LOAD", addr=active)
            settle(dev)
            got = wait_for(lambda: _device_name(dev).split(b"\x00", 1)[0], b"QA Board")
            chk.eq(got, b"QA Board", "name survives a preset load")
        else:
            chk.note("active preset slot empty; skipping the preset-load check")

        dev.pset("SYS.DEVICE_NAME", b"\x00junk")
        chk.eq(_device_name(dev), bytes(32), "leading NUL clears to all zeros")
    finally:
        dev.pset("SYS.DEVICE_NAME", orig.split(b"\x00", 1)[0] or b"\x00")
        dev.wait_ready()
