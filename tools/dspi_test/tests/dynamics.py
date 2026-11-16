"""
Dynamics group: matrix sweeps, subharmonic synth headroom and bulk carriage, tube voicing and clamps,
limiter clamps, link groups, engagement and bulk carriage. Plain round trips and refusals of these
commands are covered by paged_route, paged_dyn and paged_fx.
"""

import struct

from ..framework import test
from ..helpers import push_bulk, read_bulk, read_route, route_addr, set_and_wait, settle, wait_for, write_route


def _f(v):
    return struct.pack("<f", v)


def _save(dev, names, addr=0):
    return {n: dev.pget(n, addr) for n in names}


def _restore(dev, saved, addr=0):
    for n, raw in saved.items():
        dev.pset(n, raw, addr)


# --- Matrix mixer -----------------------------------------------------------

@test("dynamics", mutating=True)
def matrix_crosspoint_roundtrip(dev, profile, chk):
    """ROUTE.MATRIX_ROUTE: a crosspoint round-trips enabled, phase and gain; gain is not clamped."""
    out = profile.num_output_channels - 1
    points = ((0, 2), (1, out))
    saved = {p: dev.pget("ROUTE.MATRIX_ROUTE", route_addr(*p)) for p in points}
    try:
        write_route(dev, 0, 2, 1, -3.0, 0)
        r = read_route(dev, 0, 2)
        chk.eq(r.enabled, 1, "route enabled")
        chk.eq(r.phase_invert, 0, "route phase")
        chk.approx(r.gain_db, -3.0, 1e-3, "route gain")
        write_route(dev, 1, out, 1, 30.0, 1)
        r = read_route(dev, 1, out)
        chk.eq(r.phase_invert, 1, "phase invert set")
        chk.approx(r.gain_db, 30.0, 1e-3, "gain +30 stored verbatim (no clamp)")
    finally:
        for p, raw in saved.items():
            dev.pset("ROUTE.MATRIX_ROUTE", raw, route_addr(*p))


@test("dynamics", mutating=True)
def matrix_full_sweep(dev, profile, chk):
    """Every input x output crosspoint round-trips (offset/index math)."""
    ni, no = profile.num_input_channels, profile.num_output_channels
    points = [(i, o) for i in range(ni) for o in range(no)]
    saved = {p: dev.pget("ROUTE.MATRIX_ROUTE", route_addr(*p)) for p in points}
    try:
        for i, o in points:
            write_route(dev, i, o, (i + o) % 2, -2.0 * i - 0.25 * o)
        fails = 0
        for i, o in points:
            gain, en = -2.0 * i - 0.25 * o, (i + o) % 2
            r = read_route(dev, i, o)
            if not (r.enabled == en and abs(r.gain_db - gain) < 1e-3):
                fails += 1
                if fails <= 4:
                    chk.ok(False, f"crosspoint [{i}][{o}] mismatch: en={r.enabled} gain={r.gain_db:.3f} "
                                  f"(want en={en} gain={gain:.3f})")
        chk.eq(fails, 0, f"all {ni * no} crosspoints round-trip")
    finally:
        for p, raw in saved.items():
            dev.pset("ROUTE.MATRIX_ROUTE", raw, route_addr(*p))


# --- Subharmonic synthesizer ------------------------------------------------

@test("dynamics", mutating=True)
def subharm_level_clamps(dev, profile, chk):
    """FX.SUBHARM_LOW and FX.SUBHARM_HIGH clamp to [-30, +12] dB, FX.SUBHARM_BOOST to [0, +6] dB."""
    saved = _save(dev, ("FX.SUBHARM_LOW", "FX.SUBHARM_HIGH", "FX.SUBHARM_BOOST"))
    try:
        for name in ("FX.SUBHARM_LOW", "FX.SUBHARM_HIGH"):
            for v, want, what in ((-6.0, -6.0, "-6"), (-99.0, -30.0, "low clamp"), (40.0, 12.0, "high clamp")):
                dev.pset_f32(name, v)
                chk.approx(dev.pget_f32(name), want, 1e-3, f"{name} {what}")
        for v, want, what in ((3.0, 3.0, "3"), (-5.0, 0.0, "low clamp"), (20.0, 6.0, "high clamp")):
            dev.pset_f32("FX.SUBHARM_BOOST", v)
            chk.approx(dev.pget_f32("FX.SUBHARM_BOOST"), want, 1e-3, f"boost {what}")
    finally:
        _restore(dev, saved)


# Figures from subharmonic_synth_spec.md section 2.6, third band and ceiling off.
HEADROOM_ONE_BAND_DB = 4.2
HEADROOM_TWO_BANDS_DB = 6.3
HEADROOM_TOL_DB = 0.3


@test("dynamics", mutating=True)
def subharm_headroom(dev, profile, chk):
    """FX.SUBHARM_HEADROOM is 0 dB while disabled and matches the documented bound as bands and boost rise."""
    names = ("FX.SUBHARM_LOW", "FX.SUBHARM_HIGH", "FX.SUBHARM_TOP", "FX.SUBHARM_BOOST", "FX.SUBHARM_CEILING")
    saved = _save(dev, names)
    saved_en, saved_mask = dev.pget("FX.SUBHARM_ENABLE"), dev.pget("FX.SUBHARM_MASK")
    try:
        # The reading ignores the mask, so the synth is enabled with no output to reach.
        dev.pset_u16("FX.SUBHARM_MASK", 0)
        dev.pset_u8("FX.SUBHARM_ENABLE", 0)
        chk.eq(dev.pget_f32("FX.SUBHARM_HEADROOM"), 0.0, "disabled reads 0 dB")
        for name, v in (("FX.SUBHARM_LOW", 0.0), ("FX.SUBHARM_HIGH", -30.0), ("FX.SUBHARM_TOP", -30.0),
                        ("FX.SUBHARM_BOOST", 0.0), ("FX.SUBHARM_CEILING", 0.0)):
            dev.pset_f32(name, v)
        dev.pset_u8("FX.SUBHARM_ENABLE", 1)
        one_band = dev.pget_f32("FX.SUBHARM_HEADROOM")
        chk.approx(one_band, HEADROOM_ONE_BAND_DB, HEADROOM_TOL_DB, f"one band at 0 dB needs {one_band:.2f} dB")
        dev.pset_f32("FX.SUBHARM_HIGH", 0.0)
        two_band = dev.pget_f32("FX.SUBHARM_HEADROOM")
        chk.approx(two_band, HEADROOM_TWO_BANDS_DB, HEADROOM_TOL_DB, f"two bands at 0 dB need {two_band:.2f} dB")
        dev.pset_f32("FX.SUBHARM_BOOST", 6.0)
        boosted = dev.pget_f32("FX.SUBHARM_HEADROOM")
        chk.ok(boosted >= two_band + 3.0, f"+6 dB boost raises it ({two_band:.2f} -> {boosted:.2f} dB)")
    finally:
        dev.pset("FX.SUBHARM_ENABLE", saved_en)
        _restore(dev, saved)
        dev.pset("FX.SUBHARM_MASK", saved_mask)


@test("dynamics", mutating=True)
def subharm_top_clamp(dev, profile, chk):
    """FX.SUBHARM_TOP (third band level) clamps to [-30, +12] dB."""
    saved = _save(dev, ("FX.SUBHARM_TOP",))
    try:
        for v, want, what in ((-12.0, -12.0, "-12"), (-99.0, -30.0, "low clamp"), (40.0, 12.0, "high clamp")):
            dev.pset_f32("FX.SUBHARM_TOP", v)
            chk.approx(dev.pget_f32("FX.SUBHARM_TOP"), want, 1e-3, f"top {what}")
    finally:
        _restore(dev, saved)


@test("dynamics")
def subharm_meter_length(dev, profile, chk):
    """FX.SUBHARM_METER returns one u16 sub peak per output channel, each inside the 0..32767 scale."""
    n = profile.num_output_channels
    data = dev.pget("FX.SUBHARM_METER", length=64)
    if chk.eq(len(data), 2 * n, f"{n} outputs -> {2 * n} bytes"):
        peaks = struct.unpack(f"<{n}H", data)
        chk.ok(all(p <= 32767 for p in peaks), f"every peak inside the 0..32767 status scale: {peaks}")


SUBHARM_BULK = (("FX.SUBHARM_TOP", -9.0, -30.0), ("FX.SUBHARM_DEPTH", 25.0, 100.0),
                ("FX.SUBHARM_HOLD", 250.0, 50.0), ("FX.SUBHARM_CEILING", -12.0, 0.0))


@test("dynamics", mutating=True)
def subharm_bulk_roundtrip(dev, profile, chk):
    """STORE.ALL_PARAMS carries the subharm top, depth, hold, ceiling, select and link; solo stays off the wire."""
    before = read_bulk(dev, profile)
    saved = _save(dev, [n for n, _, _ in SUBHARM_BULK])
    saved_mode, saved_link = dev.pget_u8("FX.SUBHARM_SELECT"), dev.pget_u8("FX.SUBHARM_LINK")
    saved_solo = dev.pget("FX.SUBHARM_SOLO")
    try:
        for n, mark, _ in SUBHARM_BULK:
            dev.pset_f32(n, mark)
        dev.pset_u8("FX.SUBHARM_SELECT", 1)
        dev.pset_u8("FX.SUBHARM_LINK", 0)
        dev.pset_u8("FX.SUBHARM_SOLO", 1)
        blob = read_bulk(dev, profile)
        # Clear the live values, then prove the blob restores every persisted one.
        for n, _, clear in SUBHARM_BULK:
            dev.pset_f32(n, clear)
        dev.pset_u8("FX.SUBHARM_SELECT", 0)
        dev.pset_u8("FX.SUBHARM_LINK", 1)
        push_bulk(dev, blob)
        for n, mark, _ in SUBHARM_BULK:
            chk.approx(dev.pget_f32(n), mark, 1e-3, f"{n} restored")
        chk.eq(dev.pget_u8("FX.SUBHARM_SELECT"), 1, "select mode restored")
        chk.eq(dev.pget_u8("FX.SUBHARM_LINK"), 0, "link restored")
        chk.eq(dev.pget_u8("FX.SUBHARM_SOLO"), 1, "solo untouched by bulk apply")
    finally:
        # Solo is monitor-only and not in the bulk blob, so it is put back by hand.
        dev.pset("FX.SUBHARM_SOLO", saved_solo)
        push_bulk(dev, before)
    for n, raw in saved.items():
        chk.eq(dev.pget(n), raw, f"pre-test {n} restored")
    chk.eq(dev.pget_u8("FX.SUBHARM_SELECT"), saved_mode, "pre-test select mode restored")
    chk.eq(dev.pget_u8("FX.SUBHARM_LINK"), saved_link, "pre-test link restored")


# --- Tube preamp ------------------------------------------------------------

# Character fields go back before the type: restoring a stock type last re-runs its row lookup,
# so the two can never be left disagreeing.
T_CHARACTER = ("FX.TUBE_BIAS", "FX.TUBE_ASYM", "FX.TUBE_HARDNESS", "FX.TUBE_SAG")

# Documented clamp ranges of the continuous tube parameters (tube_preamp_spec.md section 2, tube.h).
TUBE_RANGES = (("FX.TUBE_DRIVE", -30.0, 24.0), ("FX.TUBE_BIAS", -100.0, 100.0), ("FX.TUBE_ASYM", -12.0, 12.0),
               ("FX.TUBE_HARDNESS", 0.0, 100.0), ("FX.TUBE_SAG", 0.0, 100.0),
               ("FX.TUBE_XFMR_DAMPING", 1.0, 20.0), ("FX.TUBE_XFMR_RES", 30.0, 150.0),
               ("FX.TUBE_MIX", 0.0, 100.0), ("FX.TUBE_TRIM", -12.0, 12.0))


def _tube_save(dev, names=()):
    keep = [n for n in names if n not in T_CHARACTER + ("FX.TUBE_TYPE",)]
    return [(n, dev.pget(n)) for n in keep + list(T_CHARACTER) + ["FX.TUBE_TYPE"]]


def _tube_restore(dev, saved):
    for n, raw in saved:
        dev.pset(n, raw)


@test("dynamics", mutating=True)
def tube_param_clamps(dev, profile, chk):
    """Every continuous FX.TUBE_* parameter clamps to its documented range at both ends."""
    saved = _tube_save(dev, [n for n, _, _ in TUBE_RANGES])
    try:
        dev.pset_f32("FX.TUBE_DRIVE", -20.0)
        chk.approx(dev.pget_f32("FX.TUBE_DRIVE"), -20.0, 1e-3, "drive -20 dB is inside the -30 dB floor")
        for name, lo, hi in TUBE_RANGES:
            dev.pset_f32(name, lo - 500.0)
            chk.approx(dev.pget_f32(name), lo, 1e-3, f"{name} low clamp")
            dev.pset_f32(name, hi + 500.0)
            chk.approx(dev.pget_f32(name), hi, 1e-3, f"{name} high clamp")
    finally:
        _tube_restore(dev, saved)


@test("dynamics", mutating=True)
def tube_type_loads_row(dev, profile, chk):
    """FX.TUBE_TYPE copies a character row in; editing a row field flips it to Custom."""
    saved = _tube_save(dev)
    try:
        dev.pset_f32("FX.TUBE_TYPE", 16.0)   # 300B / 2A3
        for name, want in zip(T_CHARACTER, (12.0, 6.0, 10.0, 12.0)):
            chk.approx(dev.pget_f32(name), want, 1e-3, f"300B {name}")
        # A character edit that lands on a different value must reset the picker.
        dev.pset_f32("FX.TUBE_BIAS", 12.5)
        chk.approx(dev.pget_f32("FX.TUBE_TYPE"), 0.0, 1e-6, "bias edit flips type to Custom")
    finally:
        _tube_restore(dev, saved)


@test("dynamics", mutating=True)
def tube_same_value_edit_keeps_type(dev, profile, chk):
    """A character SET landing on the stored value must leave the type picker alone."""
    saved = _tube_save(dev)
    try:
        dev.pset_f32("FX.TUBE_TYPE", 16.0)   # 300B / 2A3: bias 12, asym 6, hardness 10, sag 12
        dev.pset_f32("FX.TUBE_BIAS", 12.0)   # identical to what the row just stored
        chk.approx(dev.pget_f32("FX.TUBE_TYPE"), 16.0, 1e-6, "same-value bias SET keeps type 16")
        dev.pset_f32("FX.TUBE_BIAS", 13.0)
        chk.approx(dev.pget_f32("FX.TUBE_TYPE"), 0.0, 1e-6, "changed bias SET flips type to Custom")
    finally:
        _tube_restore(dev, saved)


TUBE_BULK = (("FX.TUBE_DRIVE", 15.0, 0.0), ("FX.TUBE_MIX", 40.0, 100.0), ("FX.TUBE_TRIM", -6.0, 0.0),
             ("FX.TUBE_XFMR_RES", 120.0, 30.0))


@test("dynamics", mutating=True)
def tube_bulk_roundtrip(dev, profile, chk):
    """STORE.ALL_PARAMS carries the tube module, including a Custom voicing no stock row produces."""
    before = read_bulk(dev, profile)
    saved = _tube_save(dev, [n for n, _, _ in TUBE_BULK])
    try:
        for n, want, _ in TUBE_BULK:
            dev.pset_f32(n, want)
        # EL34 leaves bias at 0, so bias 40 is a Custom voicing an apply re-derived from the type would lose.
        dev.pset_f32("FX.TUBE_TYPE", 12.0)
        dev.pset_f32("FX.TUBE_BIAS", 40.0)
        chk.approx(dev.pget_f32("FX.TUBE_TYPE"), 0.0, 1e-6, "bias edit armed Custom before the GET")
        blob = read_bulk(dev, profile)
        for n, _, clear in TUBE_BULK:
            dev.pset_f32(n, clear)
        dev.pset_f32("FX.TUBE_TYPE", 16.0)   # 300B row overwrites bias with 12
        push_bulk(dev, blob)
        for n, want, _ in TUBE_BULK:
            chk.approx(dev.pget_f32(n), want, 1e-3, f"{n} restored")
        chk.approx(dev.pget_f32("FX.TUBE_BIAS"), 40.0, 1e-3, "custom bias survives the apply")
        chk.approx(dev.pget_f32("FX.TUBE_TYPE"), 0.0, 1e-6, "type 0 Custom survives the apply")
    finally:
        push_bulk(dev, before)
    for n, raw in saved:
        chk.eq(dev.pget(n), raw, f"pre-test {n} restored")


# --- Output limiter ----------------------------------------------------------

LIM = ("DYN.LIMITER_ENABLE", "DYN.LIMITER_THRESHOLD", "DYN.LIMITER_RELEASE", "DYN.LIMITER_LINK_GROUP")
L_ENABLE, L_THRESH, L_RELEASE, L_GROUP = LIM
ALL_OUTPUTS = 0xFF


def _lim_save(dev, profile):
    return [[dev.pget(n, k) for n in LIM] for k in range(profile.num_output_channels)]


def _lim_restore(dev, saved):
    # Unlink first: a SET on a linked output also writes its group members.
    dev.pset_f32(L_GROUP, 0.0, ALL_OUTPUTS)
    for k, vals in enumerate(saved):
        for n, raw in zip(LIM[:3], vals[:3]):
            dev.pset(n, raw, k)
    for k, vals in enumerate(saved):
        dev.pset(L_GROUP, vals[3], k)


@test("dynamics", mutating=True)
def limiter_param_clamps(dev, profile, chk):
    """Threshold -30..0 dB, release 10..1000 ms, link group 0..4 (rounded)."""
    saved = _lim_save(dev, profile)
    try:
        for name, cases in ((L_THRESH, ((5.0, 0.0, "threshold hi"), (-99.0, -30.0, "threshold lo"))),
                            (L_RELEASE, ((1.0, 10.0, "release lo"), (5000.0, 1000.0, "release hi"))),
                            (L_GROUP, ((9.0, 4.0, "group hi"), (2.4, 2.0, "group rounds"), (-3.0, 0.0, "group lo")))):
            for val, want, what in cases:
                dev.pset_f32(name, val, 0)
                chk.approx(dev.pget_f32(name, 0), want, 1e-4, what)
    finally:
        _lim_restore(dev, saved)


@test("dynamics", mutating=True)
def limiter_link_group_gangs(dev, profile, chk):
    """Linked outputs share enable, threshold and release: joining adopts the group, edits reach every member."""
    saved = _lim_save(dev, profile)
    try:
        dev.pset_f32(L_GROUP, 0.0, ALL_OUTPUTS)
        dev.pset_f32(L_THRESH, -6.0, 0)
        dev.pset_f32(L_RELEASE, 300.0, 0)
        dev.pset_f32(L_THRESH, -2.0, 1)
        dev.pset_f32(L_RELEASE, 50.0, 1)
        dev.pset_f32(L_GROUP, 3.0, 0)
        chk.approx(dev.pget_f32(L_THRESH, 0), -6.0, 1e-4, "first member keeps its threshold")
        dev.pset_f32(L_GROUP, 3.0, 1)
        chk.approx(dev.pget_f32(L_THRESH, 1), -6.0, 1e-4, "joiner adopts group threshold")
        chk.approx(dev.pget_f32(L_RELEASE, 1), 300.0, 1e-3, "joiner adopts group release")
        dev.pset_f32(L_THRESH, -9.0, 1)
        dev.pset_f32(L_ENABLE, 1.0, 1)
        chk.approx(dev.pget_f32(L_THRESH, 0), -9.0, 1e-4, "edit on out 1 reaches out 0")
        chk.approx(dev.pget_f32(L_ENABLE, 0), 1.0, 1e-6, "enable on out 1 reaches out 0")
        dev.pset_f32(L_GROUP, 0.0, 1)
        dev.pset_f32(L_THRESH, -12.0, 1)
        chk.approx(dev.pget_f32(L_THRESH, 0), -9.0, 1e-4, "unlinked edit leaves out 0 alone")
    finally:
        _lim_restore(dev, saved)


@test("dynamics", mutating=True)
def limiter_engage_follows_enable(dev, profile, chk):
    """Enabling any output engages the lookahead delay; disabling the last releases it (DYN.LIMITER_STATUS byte 0)."""
    saved = _lim_save(dev, profile)

    def engaged(want):
        return wait_for(lambda: dev.pget("DYN.LIMITER_STATUS")[0], want, timeout_s=2.0, poll_s=0.02) == want

    try:
        dev.pset_f32(L_ENABLE, 0.0, ALL_OUTPUTS)
        chk.ok(engaged(0), "released with every output off")
        dev.pset_f32(L_ENABLE, 1.0, 0)
        chk.ok(engaged(1), "engaged after enabling output 0")
        dev.pset_f32(L_ENABLE, 0.0, 0)
        chk.ok(engaged(0), "released after disabling output 0")
    finally:
        _lim_restore(dev, saved)
    if any(struct.unpack("<f", vals[0])[0] for vals in saved):
        chk.ok(engaged(1), "pre-test engaged state restored")


def _config_mode(dev, mode):
    """STORE.OUTPUT_CONFIG_MODE is a deferred directory write followed by a restart."""
    set_and_wait(dev, "STORE.OUTPUT_CONFIG_MODE", bytes([mode]))
    settle(dev)


@test("dynamics", mutating=True, flash=2)
def limiter_bulk_roundtrip(dev, profile, chk):
    """STORE.ALL_PARAMS applies the limiter section in with-preset mode and ignores it in independent mode."""
    orig_mode = dev.pget_u8("STORE.OUTPUT_CONFIG_MODE")
    before = read_bulk(dev, profile)
    saved = _lim_save(dev, profile)
    last = profile.num_output_channels - 1
    marked = ((L_THRESH, -12.5), (L_RELEASE, 333.0), (L_GROUP, 3.0), (L_ENABLE, 1.0))
    cleared = ((L_THRESH, -1.0), (L_RELEASE, 100.0), (L_GROUP, 0.0), (L_ENABLE, 0.0))
    try:
        for n, v in marked:
            dev.pset_f32(n, v, last)
        blob = read_bulk(dev, profile)

        _config_mode(dev, 1)
        for n, v in cleared:
            dev.pset_f32(n, v, last)
        push_bulk(dev, blob)
        for n, v in marked:
            chk.approx(dev.pget_f32(n, last), v, 1e-3, f"with-preset: {n} restored")

        _config_mode(dev, 0)
        for n, v in cleared:
            dev.pset_f32(n, v, last)
        push_bulk(dev, blob)
        for n, v in cleared:
            chk.approx(dev.pget_f32(n, last), v, 1e-3, f"independent: {n} untouched")
    finally:
        _config_mode(dev, orig_mode)
        push_bulk(dev, before)
        _lim_restore(dev, saved)   # bulk SET skips the limiter in independent mode
    for k, vals in enumerate(saved):
        for n, raw in zip(LIM, vals):
            chk.eq(dev.pget(n, k), raw, f"pre-test out {k} {n} restored")
