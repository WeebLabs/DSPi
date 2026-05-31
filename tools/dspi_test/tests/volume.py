"""
Volume system group.

Master volume        0xD2/0xD3   (live; clamp [-127,0]; -128 mute sentinel)
Master volume mode   0xD4/0xD5   (flash: directory write)
Save master volume   0xD6/0xD7   (flash: directory write)
User volume          0xDA/0xDB   (clamp [-60,0]; 1/256-dB quantization)
User mute            0xDC/0xDD
"""

import struct

from ..device import OP, Stall
from ..framework import test
from ..helpers import bool_roundtrip

CENTER_VOLUME_INDEX = 60  # user-volume lower bound is -CENTER_VOLUME_INDEX dB


@test("volume", mutating=True)
def master_volume_clamp_and_sentinel(dev, profile, chk):
    """0xD2/0xD3: round-trip, clamp to [-127,0], -128 mute sentinel, NaN rejected, oversized STALL."""
    orig = dev.get_f32(OP.GET_MASTER_VOLUME)
    try:
        for v in (-12.5, 0.0, -127.0):
            dev.set_f32(OP.SET_MASTER_VOLUME, v)
            chk.approx(dev.get_f32(OP.GET_MASTER_VOLUME), v, 1e-3, f"master vol {v}")
        # -128 is the mute sentinel (stored verbatim).
        dev.set_f32(OP.SET_MASTER_VOLUME, -128.0)
        chk.approx(dev.get_f32(OP.GET_MASTER_VOLUME), -128.0, 1e-3, "mute sentinel -128")
        # Out-of-range clamps.
        dev.set_f32(OP.SET_MASTER_VOLUME, 10.0)
        chk.approx(dev.get_f32(OP.GET_MASTER_VOLUME), 0.0, 1e-3, "+10 clamps to 0")
        dev.set_f32(OP.SET_MASTER_VOLUME, -200.0)
        chk.approx(dev.get_f32(OP.GET_MASTER_VOLUME), -128.0, 1e-3, "-200 clamps to -128")
        # NaN rejected (value unchanged).
        dev.set_f32(OP.SET_MASTER_VOLUME, -30.0)
        dev.set(OP.SET_MASTER_VOLUME, struct.pack("<I", 0x7FC00000))
        chk.approx(dev.get_f32(OP.GET_MASTER_VOLUME), -30.0, 1e-3, "NaN rejected")
        # Oversized payload STALLs (contract).
        chk.stalls(lambda: dev.set(OP.SET_MASTER_VOLUME, b"\x00" * 65), "oversized SET STALL")
    finally:
        dev.set_f32(OP.SET_MASTER_VOLUME, orig)


@test("volume", mutating=True)
def user_volume_clamp_quant(dev, profile, chk):
    """0xDA/0xDB: clamp [-60,0], 1/256-dB quantization, NaN rejected (LG temporarily disabled)."""
    # LG Sound Sync can overwrite user volume on locked SPDIF; disable it for the
    # duration so the round-trip is deterministic, then restore.
    lg = dev.get_u8(OP.GET_LG_SOUND_SYNC_ENABLE)
    orig = dev.get_f32(OP.GET_USER_VOLUME)
    try:
        dev.set_u8(OP.SET_LG_SOUND_SYNC_ENABLE, 0)
        for v in (-10.0, 0.0, -60.0):
            dev.set_f32(OP.SET_USER_VOLUME, v)
            chk.approx(dev.get_f32(OP.GET_USER_VOLUME), v, 1e-3, f"user vol {v}")
        dev.set_f32(OP.SET_USER_VOLUME, 6.0)
        chk.approx(dev.get_f32(OP.GET_USER_VOLUME), 0.0, 1e-3, "+6 clamps to 0")
        dev.set_f32(OP.SET_USER_VOLUME, -100.0)
        chk.approx(dev.get_f32(OP.GET_USER_VOLUME), -CENTER_VOLUME_INDEX, 1e-3, "-100 clamps to -60")
        # Quantization to 1/256 dB.
        dev.set_f32(OP.SET_USER_VOLUME, -10.003)
        got = dev.get_f32(OP.GET_USER_VOLUME)
        chk.approx(got, round(-10.003 * 256) / 256, 1.0 / 256 + 1e-4, "1/256 dB quantization")
        # NaN rejected.
        dev.set_f32(OP.SET_USER_VOLUME, -20.0)
        dev.set(OP.SET_USER_VOLUME, struct.pack("<I", 0x7FC00000))
        chk.approx(dev.get_f32(OP.GET_USER_VOLUME), -20.0, 1e-3, "NaN rejected")
    finally:
        dev.set_f32(OP.SET_USER_VOLUME, orig)
        dev.set_u8(OP.SET_LG_SOUND_SYNC_ENABLE, lg)


@test("volume", mutating=True)
def user_mute_bool(dev, profile, chk):
    """0xDC/0xDD user mute uses != 0 coercion."""
    bool_roundtrip(dev, chk, OP.SET_USER_MUTE, OP.GET_USER_MUTE, label="user mute")


@test("volume", mutating=True)
def master_user_volume_independence(dev, profile, chk):
    """Master and user volume are independent stages; setting one never moves the other."""
    lg = dev.get_u8(OP.GET_LG_SOUND_SYNC_ENABLE)
    om, ou = dev.get_f32(OP.GET_MASTER_VOLUME), dev.get_f32(OP.GET_USER_VOLUME)
    try:
        dev.set_u8(OP.SET_LG_SOUND_SYNC_ENABLE, 0)
        dev.set_f32(OP.SET_MASTER_VOLUME, -30.0)
        dev.set_f32(OP.SET_USER_VOLUME, -10.0)
        chk.approx(dev.get_f32(OP.GET_MASTER_VOLUME), -30.0, 1e-3, "master holds -30")
        chk.approx(dev.get_f32(OP.GET_USER_VOLUME), -10.0, 1e-3, "user holds -10")
        dev.set_f32(OP.SET_MASTER_VOLUME, -6.0)
        chk.approx(dev.get_f32(OP.GET_USER_VOLUME), -10.0, 1e-3, "user unchanged by master")
        dev.set_f32(OP.SET_USER_VOLUME, -20.0)
        chk.approx(dev.get_f32(OP.GET_MASTER_VOLUME), -6.0, 1e-3, "master unchanged by user")
    finally:
        dev.set_f32(OP.SET_MASTER_VOLUME, om)
        dev.set_f32(OP.SET_USER_VOLUME, ou)
        dev.set_u8(OP.SET_LG_SOUND_SYNC_ENABLE, lg)


@test("volume")
def master_volume_mode_get(dev, profile, chk):
    """0xD5 master-volume mode reads {0,1}; 0xD7 saved master volume is a finite dB."""
    chk.member(dev.get_u8(OP.GET_MASTER_VOLUME_MODE), (0, 1), "mv mode")
    sv = dev.get_f32(OP.GET_SAVED_MASTER_VOLUME)
    chk.in_range(sv, -128.0, 0.0, "saved master volume range")


@test("volume", mutating=True, flash=3)
def master_volume_mode_roundtrip(dev, profile, chk):
    """0xD4/0xD5 mode round-trips {0,1}; value > 1 clamps to 0 (writes directory; flash)."""
    orig = dev.get_u8(OP.GET_MASTER_VOLUME_MODE)
    try:
        dev.set_u8(OP.SET_MASTER_VOLUME_MODE, 1)
        chk.eq(dev.get_u8(OP.GET_MASTER_VOLUME_MODE), 1, "mode 1")
        dev.set_u8(OP.SET_MASTER_VOLUME_MODE, 0)
        chk.eq(dev.get_u8(OP.GET_MASTER_VOLUME_MODE), 0, "mode 0")
        dev.set_u8(OP.SET_MASTER_VOLUME_MODE, 5)
        chk.eq(dev.get_u8(OP.GET_MASTER_VOLUME_MODE), 0, "mode 5 clamps to 0")
    finally:
        dev.set_u8(OP.SET_MASTER_VOLUME_MODE, orig)


@test("volume", mutating=True, flash=2)
def master_volume_save_roundtrip(dev, profile, chk):
    """0xD6 persists live master volume to the directory; 0xD7 reads it back (flash)."""
    import time
    orig_saved = dev.get_f32(OP.GET_SAVED_MASTER_VOLUME)
    orig_live = dev.get_f32(OP.GET_MASTER_VOLUME)
    try:
        dev.set_f32(OP.SET_MASTER_VOLUME, -18.0)
        ack = dev.get(OP.SAVE_MASTER_VOLUME, 1)
        chk.eq(ack[0], 0x00, "save-master-volume accepted")
        # Deferred flash write; poll until reflected.
        deadline = time.monotonic() + 1.0
        ok = False
        while time.monotonic() < deadline:
            if abs(dev.get_f32(OP.GET_SAVED_MASTER_VOLUME) - (-18.0)) < 1e-3:
                ok = True
                break
            time.sleep(0.05)
        chk.ok(ok, "saved master volume became -18.0 after 0xD6")
    finally:
        # Restore live + saved to originals.
        dev.set_f32(OP.SET_MASTER_VOLUME, orig_saved)
        dev.get(OP.SAVE_MASTER_VOLUME, 1)
        import time as _t
        _t.sleep(0.15)
        dev.set_f32(OP.SET_MASTER_VOLUME, orig_live)
