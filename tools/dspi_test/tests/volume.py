"""
Volume group: ROUTE.MASTER_VOLUME and ROUTE.USER_VOLUME range ends, refusals and independence.
Mute, the master volume mode and save, and the main clamp cases are covered by paged_route.
"""

from ..framework import test
from ..helpers import NAN, expect_stall, wrong_length_refused

CENTER_VOLUME_INDEX = 60  # user-volume lower bound is -CENTER_VOLUME_INDEX dB


@test("volume", mutating=True)
def master_volume_clamp_and_sentinel(dev, profile, chk):
    """ROUTE.MASTER_VOLUME: range ends and the -128 mute sentinel round-trip; a wrong or oversized payload is VC_ERR_LENGTH."""
    orig = dev.pget("ROUTE.MASTER_VOLUME")
    try:
        for v in (0.0, -127.0, -128.0):
            dev.pset_f32("ROUTE.MASTER_VOLUME", v)
            chk.approx(dev.pget_f32("ROUTE.MASTER_VOLUME"), v, 1e-3, f"master vol {v}")
        dev.pset_f32("ROUTE.MASTER_VOLUME", -30.0)
        wrong_length_refused(dev, chk, "ROUTE.MASTER_VOLUME", lengths=(3, 5, 65), label="master volume")
    finally:
        dev.pset("ROUTE.MASTER_VOLUME", orig)


@test("volume", mutating=True)
def user_volume_clamp_quant(dev, profile, chk):
    """ROUTE.USER_VOLUME: range ends round-trip, 1/256 dB steps, NaN is VC_ERR_VALUE and changes nothing (LG Sound Sync off)."""
    # LG Sound Sync can overwrite user volume on locked S/PDIF, so it is off for the duration.
    lg = dev.pget("INPUT.LG_SOUND_SYNC_ENABLE")
    orig = dev.pget("ROUTE.USER_VOLUME")
    try:
        dev.pset_u8("INPUT.LG_SOUND_SYNC_ENABLE", 0)
        for v in (0.0, -float(CENTER_VOLUME_INDEX)):
            dev.pset_f32("ROUTE.USER_VOLUME", v)
            chk.approx(dev.pget_f32("ROUTE.USER_VOLUME"), v, 1e-3, f"user vol {v}")
        dev.pset_f32("ROUTE.USER_VOLUME", -10.003)
        chk.approx(dev.pget_f32("ROUTE.USER_VOLUME"), round(-10.003 * 256) / 256, 1.0 / 256 + 1e-4,
                   "1/256 dB quantization")
        dev.pset_f32("ROUTE.USER_VOLUME", -20.0)
        expect_stall(chk, lambda: dev.pset("ROUTE.USER_VOLUME", NAN), "VC_ERR_VALUE", "user volume NaN")
        chk.approx(dev.pget_f32("ROUTE.USER_VOLUME"), -20.0, 1e-3, "NaN left the value alone")
    finally:
        dev.pset("ROUTE.USER_VOLUME", orig)
        dev.pset("INPUT.LG_SOUND_SYNC_ENABLE", lg)


@test("volume", mutating=True)
def master_user_volume_independence(dev, profile, chk):
    """Master and user volume are independent stages; setting one never moves the other."""
    lg = dev.pget("INPUT.LG_SOUND_SYNC_ENABLE")
    om, ou = dev.pget("ROUTE.MASTER_VOLUME"), dev.pget("ROUTE.USER_VOLUME")
    try:
        dev.pset_u8("INPUT.LG_SOUND_SYNC_ENABLE", 0)
        dev.pset_f32("ROUTE.MASTER_VOLUME", -30.0)
        dev.pset_f32("ROUTE.USER_VOLUME", -10.0)
        chk.approx(dev.pget_f32("ROUTE.MASTER_VOLUME"), -30.0, 1e-3, "master holds -30")
        chk.approx(dev.pget_f32("ROUTE.USER_VOLUME"), -10.0, 1e-3, "user holds -10")
        dev.pset_f32("ROUTE.MASTER_VOLUME", -6.0)
        chk.approx(dev.pget_f32("ROUTE.USER_VOLUME"), -10.0, 1e-3, "user unchanged by master")
        dev.pset_f32("ROUTE.USER_VOLUME", -20.0)
        chk.approx(dev.pget_f32("ROUTE.MASTER_VOLUME"), -6.0, 1e-3, "master unchanged by user")
    finally:
        dev.pset("ROUTE.MASTER_VOLUME", om)
        dev.pset("ROUTE.USER_VOLUME", ou)
        dev.pset("INPUT.LG_SOUND_SYNC_ENABLE", lg)
