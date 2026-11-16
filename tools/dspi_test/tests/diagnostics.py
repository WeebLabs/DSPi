"""
Buffer statistics (SYS.BUFFER_STATS) and the volume leveller (DYN.LEVELLER_*): the field ranges and
cross-parameter behaviour that paged_sys and paged_dyn do not already assert.
"""

from ..framework import test
from ..helpers import buffer_stats, expect_stall


# --- Buffer statistics ------------------------------------------------------

@test("diagnostics")
def buffer_stats_shape(dev, profile, chk):
    """SYS.BUFFER_STATS: num_spdif matches the platform; every slot and PDM field is inside its range."""
    st = buffer_stats(dev)
    chk.eq(st["num_spdif"], profile.num_spdif, "num_spdif header")
    for i, s in enumerate(st["spdif"]):
        chk.in_range(s["consumer_free"], 0, 16, f"spdif[{i}] consumer_free")
        chk.in_range(s["consumer_prepared"], 0, 16, f"spdif[{i}] consumer_prepared")
        chk.member(s["consumer_playing"], (0, 1), f"spdif[{i}] consumer_playing")
        chk.in_range(s["consumer_fill_pct"], 0, 100, f"spdif[{i}] fill_pct")
        chk.in_range(s["consumer_min_fill_pct"], 0, 100, f"spdif[{i}] min_fill_pct")
        chk.in_range(s["consumer_max_fill_pct"], 0, 100, f"spdif[{i}] max_fill_pct")
    for name, v in st["pdm"].items():
        chk.in_range(v, 0, 100, f"pdm {name}")


# --- Volume leveller --------------------------------------------------------

@test("diagnostics", mutating=True)
def leveller_speed_rejects_out_of_range(dev, profile, chk):
    """DYN.LEVELLER_SPEED 0-2 round-trips; 3 and 255 are VC_ERR_VALUE and leave the value alone (not clamped)."""
    for v in (0, 1, 2):
        dev.pset_u8("DYN.LEVELLER_SPEED", v)
        chk.eq(dev.pget_u8("DYN.LEVELLER_SPEED"), v, f"speed {v} roundtrip")
    dev.pset_u8("DYN.LEVELLER_SPEED", 1)  # known good
    for bad in (3, 255):
        expect_stall(chk, lambda: dev.pset_u8("DYN.LEVELLER_SPEED", bad), "VC_ERR_VALUE", f"speed {bad}")
        chk.eq(dev.pget_u8("DYN.LEVELLER_SPEED"), 1, f"speed {bad} refused (stays 1, NOT clamped to 2)")


@test("diagnostics", mutating=True)
def leveller_independence(dev, profile, chk):
    """Changing one leveller param leaves the other five untouched (no struct aliasing)."""
    dev.pset_u8("DYN.LEVELLER_ENABLE", 1)
    dev.pset_f32("DYN.LEVELLER_AMOUNT", 40.0)
    dev.pset_u8("DYN.LEVELLER_SPEED", 2)
    dev.pset_f32("DYN.LEVELLER_MAX_GAIN", 20.0)
    dev.pset_u8("DYN.LEVELLER_LOOKAHEAD", 0)
    dev.pset_f32("DYN.LEVELLER_GATE", -60.0)
    # Mutate only amount; everything else must hold.
    dev.pset_f32("DYN.LEVELLER_AMOUNT", 77.0)
    chk.eq(dev.pget_u8("DYN.LEVELLER_ENABLE"), 1, "enable unchanged")
    chk.eq(dev.pget_u8("DYN.LEVELLER_SPEED"), 2, "speed unchanged")
    chk.approx(dev.pget_f32("DYN.LEVELLER_MAX_GAIN"), 20.0, 1e-3, "max_gain unchanged")
    chk.eq(dev.pget_u8("DYN.LEVELLER_LOOKAHEAD"), 0, "lookahead unchanged")
    chk.approx(dev.pget_f32("DYN.LEVELLER_GATE"), -60.0, 1e-3, "gate unchanged")
    chk.approx(dev.pget_f32("DYN.LEVELLER_AMOUNT"), 77.0, 1e-3, "amount changed")
