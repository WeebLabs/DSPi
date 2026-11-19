"""
Pin availability query tests (Documentation/Features/pin_availability_query_spec.md): reply shapes,
the claim list, and the consistency rule that every mask bit matches what the real SET does.
Store-mode SETs touch functions that are not in use, so they move no hardware; an in-use function is
only probed with pins it must refuse.
"""

import struct
import time

from ..framework import Skip, test
from ..helpers import (INPUT_CONFIG_OFF, PIN_CLAIM_ACTIVE, expect_stall, has_command, input_blocks, pin_claims,
                       push_bulk, reached, read_bulk, require_paged, restore_input_blocks, settle, spdif_rx_config,
                       try_set)

F_IN_USE, F_PAIR_PLUS1, F_UNASSIGNED, F_START_OK = 0x01, 0x02, 0x04, 0x08
FN_SPDIF_RX, FN_I2S_RX = 7, 8
OUTPUT_CONFIG_WITH_PRESET = 1


def _require(dev):
    require_paged(dev)
    if not has_command(dev, "SYS.PIN_STATE", "GET"):
        raise Skip("firmware has no pin availability queries")


def _pin_state(dev):
    raw = dev.pget("SYS.PIN_STATE", length=24)
    gen, count, n_claims, subst = struct.unpack_from("<IBBH", raw, 0)
    mask = int.from_bytes(raw[8:16], "little")
    return dict(gen=gen, gpio_count=count, claims=n_claims, subst=subst, mask=mask, reserved=raw[16:24])


def _avail(dev, cmd, addr=0, cs=False):
    """Status, flags, groups, roles, generation and the (valid, free) mask pairs in group-major order."""
    raw = dev.pget(cmd, addr=addr)
    status, flags, groups, roles = raw[:4]
    gen = int.from_bytes(raw[4:8], "little")
    n = 4 if cs else groups * roles
    pairs = [(int.from_bytes(raw[8 + 16 * i: 16 + 16 * i], "little"),
              int.from_bytes(raw[16 + 16 * i: 24 + 16 * i], "little")) for i in range(n)]
    share = int.from_bytes(raw[72:80], "little") if cs else 0
    return dict(raw=raw, status=status, flags=flags, groups=groups, roles=roles, gen=gen,
                pairs=pairs, share=share)


def _single_functions(dev, profile):
    """(label, AVAIL command, SET command, addr) for every one-pin function this build has."""
    out = []
    for i in range(profile.num_pin_outputs):
        out.append((f"output pin {i}", "OUTPUT.PIN_AVAIL", "OUTPUT.PIN", i))
    for i in range(spdif_rx_config(dev)["limit"]):
        out.append((f"S/PDIF input {i}", "INPUT.SPDIF_RX_PIN_AVAIL", "INPUT.SPDIF_RX_PIN", i))
    for p in range(4 if profile.platform_id == 1 else 1):
        out.append((f"I2S RX pair {p}", "INPUT.I2S_RX_PIN_AVAIL", "INPUT.I2S_RX_PIN", p))
    for r in (0, 1):
        out.append((f"BCK role {r}", "CLOCK.I2S_BCK_PIN_AVAIL", "CLOCK.I2S_BCK_PIN", r))
    out.append(("MCK", "CLOCK.MCK_PIN_AVAIL", "CLOCK.MCK_PIN", 0))
    if has_command(dev, "OUTPUT.ADAT_TX_PIN_AVAIL"):
        out.append(("ADAT TX", "OUTPUT.ADAT_TX_PIN_AVAIL", "OUTPUT.ADAT_TX_PIN", 0))
    if has_command(dev, "INPUT.ADAT_RX_PIN_AVAIL"):
        out.append(("ADAT RX", "INPUT.ADAT_RX_PIN_AVAIL", "INPUT.ADAT_RX_PIN", 0))
    return out


@test("paged_pins")
def pin_state_and_claims_shapes(dev, profile, chk):
    """SYS.PIN_STATE and SYS.PIN_CLAIMS agree, entries are sorted and well formed, reserved bytes are zero."""
    _require(dev)
    st = _pin_state(dev)
    claims = pin_claims(dev)
    chk.ok(st["gpio_count"] >= 30, f"GPIO count {st['gpio_count']}")
    chk.eq(st["mask"] >> st["gpio_count"], 0, "no assignable GPIO past the GPIO count")
    chk.eq(st["reserved"], bytes(8), "PIN_STATE reserved")
    chk.eq(st["claims"], len(claims), "claim count matches PIN_CLAIMS")
    chk.eq([c[0] for c in claims], sorted(c[0] for c in claims), "entries sorted by GPIO")
    for g, fn, idx, state, role, cs_type in claims:
        chk.in_range(fn, 1, 12, f"GPIO {g} function ID")
        chk.member(state, (0, 1), f"GPIO {g} claim state")
        chk.ok(cs_type == 0 or fn == 12, f"GPIO {g}: CS type only on CS claims")
        chk.ok(role <= 1, f"GPIO {g} role")


@test("paged_pins")
def availability_reply_shapes(dev, profile, chk):
    """Every AVAIL reply: lengths, groups and roles, free inside valid, valid inside the GPIO mask."""
    _require(dev)
    gmask = _pin_state(dev)["mask"]
    for label, avail_cmd, _set_cmd, addr in _single_functions(dev, profile):
        a = _avail(dev, avail_cmd, addr)
        chk.eq(len(a["raw"]), 24, f"{label}: reply length")
        chk.eq((a["groups"], a["roles"]), (1, 1), f"{label}: one group, one role")
        valid, free = a["pairs"][0]
        chk.eq(free & ~valid, 0, f"{label}: free inside valid")
        chk.eq(valid & ~gmask, 0, f"{label}: valid inside the GPIO mask")
        chk.member(a["status"], (0x00, 0x17, 0x18), f"{label}: status")
        chk.eq(bool(a["flags"] & F_PAIR_PLUS1), avail_cmd == "CLOCK.I2S_BCK_PIN_AVAIL", f"{label}: pair flag")
    for cmd in ("XPORT.UART_PIN_AVAIL", "XPORT.I2C_PIN_AVAIL"):
        a = _avail(dev, cmd)
        chk.eq(len(a["raw"]), 72, f"{cmd}: reply length")
        chk.eq(a["roles"], 2, f"{cmd}: two roles")
        for valid, free in a["pairs"]:
            chk.eq(free & ~valid, 0, f"{cmd}: free inside valid")
    if has_command(dev, "CS.BINDING_PIN_AVAIL"):
        a = _avail(dev, "CS.BINDING_PIN_AVAIL", addr=(1 << 8) | 0, cs=True)    # a button in slot 0
        chk.eq(len(a["raw"]), 80, "CS button: reply length")
        valid, free = a["pairs"][0]
        chk.eq(free & a["share"], 0, "CS button: a pin is never both free and shareable")
        chk.eq((free | a["share"]) & ~valid, 0, "CS button: free and shareable inside valid")
        expect_stall(chk, lambda: dev.pget("CS.BINDING_PIN_AVAIL", addr=0, length=80), "VC_ERR_ADDRESS",
                     "CS type NONE")


@test("paged_pins")
def free_masks_avoid_active_claims(dev, profile, chk):
    """A free bit never names a GPIO another function holds (the ADAT loopback share excepted)."""
    _require(dev)
    claims = [c for c in pin_claims(dev) if c[3] == PIN_CLAIM_ACTIVE]
    fn_of = {"OUTPUT.PIN_AVAIL": 1, "INPUT.SPDIF_RX_PIN_AVAIL": 7, "INPUT.I2S_RX_PIN_AVAIL": 8,
             "CLOCK.I2S_BCK_PIN_AVAIL": 3, "CLOCK.MCK_PIN_AVAIL": 4, "OUTPUT.ADAT_TX_PIN_AVAIL": 5,
             "INPUT.ADAT_RX_PIN_AVAIL": 6}
    share = {5: {6}, 6: {5}}
    for label, cmd, _s, addr in _single_functions(dev, profile):
        fn = fn_of[cmd]
        if cmd == "OUTPUT.PIN_AVAIL" and addr == profile.num_pin_outputs - 1:
            fn, idx = 2, 0
        else:
            idx = addr
        free = _avail(dev, cmd, addr)["pairs"][0][1]
        for g, cfn, cidx, _st, _r, _t in claims:
            if cfn == fn and (cidx == idx or fn == 3):
                continue
            if cfn in share.get(fn, ()):
                continue
            pair = (g, g - 1) if fn == 3 else (g,)
            for p in pair:
                if p >= 0 and (free >> p) & 1:
                    chk.ok(False, f"{label}: GPIO {p} free but held by function {cfn} index {cidx}")


@test("paged_pins", mutating=True)
def masks_match_real_sets(dev, profile, chk):
    """The rule the query exists for: a function not in use accepts exactly its valid pins; one in use
    refuses every valid pin outside free with VC_ERR_PIN_IN_USE and every invalid one with PIN_INVALID."""
    _require(dev)
    n = _pin_state(dev)["gpio_count"]
    for label, avail_cmd, set_cmd, addr in _single_functions(dev, profile):
        a = _avail(dev, avail_cmd, addr)
        if a["status"] != 0:
            chk.note(f"{label}: status 0x{a['status']:02X}, refusals by state not probed")
            continue
        valid, free = a["pairs"][0]
        orig = dev.pget_u8(set_cmd, addr)
        restore = bytes([0xFF]) if (set_cmd == "INPUT.ADAT_RX_PIN" and orig == 0) else bytes([orig])
        in_use = bool(a["flags"] & F_IN_USE)
        try:
            for g in range(n):
                want_ok = (valid >> g) & 1 if not in_use else (g == orig)
                if in_use and (free >> g) & 1 and g != orig:
                    continue                      # a real move of a live function: not probed here
                r = try_set(dev, set_cmd, bytes([g]), addr)
                if want_ok:
                    chk.eq(r, "VC_OK", f"{label}: GPIO {g} (valid)")
                elif in_use and (valid >> g) & 1:
                    chk.eq(r, "VC_ERR_PIN_IN_USE", f"{label}: GPIO {g} (valid, not free)")
                else:
                    chk.eq(r, "VC_ERR_PIN_INVALID", f"{label}: GPIO {g} (invalid)")
        finally:
            if dev.pget_u8(set_cmd, addr) != orig:
                dev.pset(set_cmd, restore, addr=addr)
            chk.eq(dev.pget_u8(set_cmd, addr), orig, f"{label}: restored")


@test("paged_pins", mutating=True)
def start_refused_on_held_stored_pin(dev, profile, chk):
    """A disabled MCK stores a held GPIO; enabling it then answers VC_ERR_PIN_IN_USE, leaves it off, and
    START_OK was clear. Restores the pin."""
    _require(dev)
    if dev.pget_u8("CLOCK.MCK_ENABLE"):
        raise Skip("MCK is enabled")
    a = _avail(dev, "CLOCK.MCK_PIN_AVAIL")
    valid, free = a["pairs"][0]
    held = [g for g in range(64) if (valid >> g) & 1 and not (free >> g) & 1]
    if not held:
        raise Skip("no clk_gpout GPIO is held by another function")
    orig = dev.pget_u8("CLOCK.MCK_PIN")
    try:
        dev.pset("CLOCK.MCK_PIN", bytes([held[0]]))
        chk.eq(dev.pget_u8("CLOCK.MCK_PIN"), held[0], "held GPIO stored while disabled")
        chk.eq(_avail(dev, "CLOCK.MCK_PIN_AVAIL")["flags"] & F_START_OK, 0, "START_OK clear")
        expect_stall(chk, lambda: dev.pset("CLOCK.MCK_ENABLE", b"\x01"), "VC_ERR_PIN_IN_USE", "enable on a held pin")
        chk.eq(dev.pget_u8("CLOCK.MCK_ENABLE"), 0, "MCK still disabled")
    finally:
        dev.pset("CLOCK.MCK_PIN", bytes([orig]))
        chk.eq(dev.pget_u8("CLOCK.MCK_PIN"), orig, "MCK pin restored")


@test("paged_pins", mutating=True)
def generation_tracks_pin_changes(dev, profile, chk):
    """Moving a stored pin advances the generation; the change shows within 50 ms."""
    _require(dev)
    if dev.pget_u8("CLOCK.MCK_ENABLE"):
        raise Skip("MCK is enabled")
    a = _avail(dev, "CLOCK.MCK_PIN_AVAIL")
    orig = dev.pget_u8("CLOCK.MCK_PIN")
    other = next((g for g in range(64) if (a["pairs"][0][0] >> g) & 1 and g != orig), None)
    if other is None:
        raise Skip("MCK has one valid pin")
    g0 = _pin_state(dev)["gen"]
    try:
        dev.pset("CLOCK.MCK_PIN", bytes([other]))
        chk.ok(_pin_state(dev)["gen"] != g0, "generation changed after a pin move (a GET recomputes it)")
    finally:
        dev.pset("CLOCK.MCK_PIN", bytes([orig]))


def _refused_start(dev, chk, label, store_cmd, store_addr, held_pin, start, restore_value=None,
                   code="VC_ERR_PIN_IN_USE", unchanged=None):
    """Store `held_pin` on a function not in use, then `start()` must answer `code`, and `unchanged()`, when
    given, must read the same before and after. The stored pin is restored whatever happens."""
    orig = dev.pget_u8(store_cmd, store_addr)
    try:
        r = try_set(dev, store_cmd, bytes([held_pin]), store_addr)
        chk.eq(r, "VC_OK", f"{label}: a held GPIO is stored while not in use")
        if r != "VC_OK":
            return
        before = unchanged() if unchanged else None
        expect_stall(chk, start, code, f"{label}: start on a held pin")
        if unchanged:
            chk.eq(unchanged(), before, f"{label}: nothing changed")
    finally:
        dev.pset(store_cmd, bytes([orig if restore_value is None else restore_value]), addr=store_addr)
        chk.eq(dev.pget_u8(store_cmd, store_addr), orig, f"{label}: stored pin restored")


@test("paged_pins", mutating=True)
def spdif_count_raise_refused_on_held_pin(dev, profile, chk):
    """With S/PDIF input 4 stored on output 0's GPIO, a count raise from 1 to 4 is refused as one unit: the
    count and every claim are unchanged, so inputs 2 and 3 did not claim either. Restores pin and count."""
    _require(dev)
    saved = input_blocks(dev)
    if not saved["spdif"]:
        raise Skip("the S/PDIF input block is off")
    if dev.pget_u8("INPUT.SOURCE") in (4, 5, 6):
        raise Skip("an S/PDIF input above 1 is the source")
    try:
        dev.pset_u8("INPUT.SPDIF_RX_COUNT", 1)
        _refused_start(dev, chk, "S/PDIF count 1 to 4", "INPUT.SPDIF_RX_PIN", 3, dev.pget_u8("OUTPUT.PIN", 0),
                       lambda: dev.pset("INPUT.SPDIF_RX_COUNT", b"\x04"),
                       unchanged=lambda: (dev.pget_u8("INPUT.SPDIF_RX_COUNT"), pin_claims(dev)))
    finally:
        restore_input_blocks(dev, saved)
        chk.eq(input_blocks(dev), saved, "S/PDIF count restored")


@test("paged_pins", mutating=True)
def source_switch_never_pin_error(dev, profile, chk):
    """A source switch runs no pin check. A source outside its block, its stored pin on output 0's GPIO, is
    refused with VC_ERR_STATE and never a pin error: I2S with the block off, S/PDIF input 4 above the count."""
    _require(dev)
    src = dev.pget_u8("INPUT.SOURCE")
    held = dev.pget_u8("OUTPUT.PIN", 0)
    ran = 0
    if not dev.pget_u8("INPUT.I2S_RX_ENABLE"):
        _refused_start(dev, chk, "I2S source, block off", "INPUT.I2S_RX_PIN", 0, held,
                       lambda: dev.pset("INPUT.SOURCE", b"\x02"), code="VC_ERR_STATE")
        ran += 1
    cfg = spdif_rx_config(dev)
    if not cfg["enabled"] or cfg["count"] < 4:
        _refused_start(dev, chk, "S/PDIF input 4 source, above the count", "INPUT.SPDIF_RX_PIN", 3, held,
                       lambda: dev.pset("INPUT.SOURCE", b"\x06"), code="VC_ERR_STATE")
        ran += 1
    if not ran:
        raise Skip("the I2S block is on and the S/PDIF block holds all four inputs")
    time.sleep(0.05)
    chk.eq(dev.pget_u8("INPUT.SOURCE"), src, "source unchanged")


def _i2s_rx_idle(dev, profile):
    """Skip unless the I2S block is off and no output slot is I2S, so no I2S clock pair is in use."""
    if dev.pget_u8("INPUT.I2S_RX_ENABLE"):
        raise Skip("the I2S input block is on")
    if any(dev.pget_u8("OUTPUT.TYPE", i) for i in range(profile.num_spdif)):
        raise Skip("an output slot is I2S: the clocks are running")


def _clock_modes(dev):
    return dev.pget_u8("CLOCK.I2S_CLOCK_PIN_MODE"), dev.pget_u8("CLOCK.I2S_CLOCK_MODE")


def _set_clock_modes(dev, pin_mode, mode):
    """Set the clock-pin mode, then the clock mode, which the main loop applies; True once both read back."""
    dev.pset_u8("CLOCK.I2S_CLOCK_PIN_MODE", pin_mode)
    dev.pset_u8("CLOCK.I2S_CLOCK_MODE", mode)
    ok = reached(lambda: _clock_modes(dev), (pin_mode, mode))
    settle(dev)
    return ok


def _in_clock_layouts(dev, chk, body):
    """body(role, label) in UNIFIED clock-pin mode (pair 0) and in SPLIT slave mode (pair 1), the two pairs
    the I2S input can claim, then both modes restored. Callers hold the clocks stopped first."""
    orig = _clock_modes(dev)
    try:
        for pin_mode, mode, role, label in ((0, orig[1], 0, "unified"), (1, 1, 1, "split slave")):
            if chk.ok(_set_clock_modes(dev, pin_mode, mode), f"{label}: clock modes applied"):
                body(role, label)
    finally:
        _set_clock_modes(dev, *orig)
        chk.eq(_clock_modes(dev), orig, "clock-pin mode and clock mode restored")


def _i2s_off_unchanged(dev):
    return dev.pget_u8("INPUT.I2S_RX_ENABLE"), pin_claims(dev)


@test("paged_pins", mutating=True)
def i2s_enable_refused_on_held_data_pin(dev, profile, chk):
    """With I2S RX pair 0 stored on output 0's GPIO, switching the I2S block on is refused with
    VC_ERR_PIN_IN_USE and nothing claims, in unified and in split slave mode. Restores pin and modes."""
    _require(dev)
    _i2s_rx_idle(dev, profile)
    held = dev.pget_u8("OUTPUT.PIN", 0)

    def body(role, label):
        if not _avail(dev, "CLOCK.I2S_BCK_PIN_AVAIL", role)["flags"] & F_START_OK:
            chk.note(f"{label}: clock pair {role} could not claim either; the data pin case is ambiguous")
            return
        _refused_start(dev, chk, f"{label} I2S data pin", "INPUT.I2S_RX_PIN", 0, held,
                       lambda: dev.pset("INPUT.I2S_RX_ENABLE", b"\x01"), unchanged=lambda: _i2s_off_unchanged(dev))
    _in_clock_layouts(dev, chk, body)


@test("paged_pins", mutating=True)
def i2s_enable_refused_on_held_clock_pin(dev, profile, chk):
    """With the clock pair the I2S input would use stored on a held GPIO, switching the I2S block on is refused
    with VC_ERR_PIN_IN_USE and nothing claims, in unified and in split slave mode. Restores pins and modes."""
    _require(dev)
    _i2s_rx_idle(dev, profile)
    pairs = dev.pget_u8("INPUT.I2S_RX_CHANNELS") // 2

    def body(role, label):
        if not all(_avail(dev, "INPUT.I2S_RX_PIN_AVAIL", p)["flags"] & F_START_OK for p in range(pairs)):
            chk.note(f"{label}: a counted data pin could not claim either; the clock case is ambiguous")
            return
        valid, free = _avail(dev, "CLOCK.I2S_BCK_PIN_AVAIL", role)["pairs"][0]
        held = next((g for g in range(64) if (valid >> g) & 1 and not (free >> g) & 1), None)
        if held is None:
            chk.note(f"{label}: no valid clock pair overlaps a held GPIO")
            return
        _refused_start(dev, chk, f"{label} I2S clock pair {role}", "CLOCK.I2S_BCK_PIN", role, held,
                       lambda: dev.pset("INPUT.I2S_RX_ENABLE", b"\x01"), unchanged=lambda: _i2s_off_unchanged(dev))
    _in_clock_layouts(dev, chk, body)


@test("paged_pins", mutating=True)
def output_type_refused_on_held_clock_pair(dev, profile, chk):
    """With the idle clock pair stored on a held GPIO and no clocks running, OUTPUT.TYPE to I2S is
    refused and the type is unchanged."""
    _require(dev)
    types = [dev.pget_u8("OUTPUT.TYPE", i) for i in range(profile.num_spdif)]
    if any(types) or dev.pget_u8("INPUT.I2S_RX_ENABLE"):
        raise Skip("the I2S clocks are running or the I2S input block holds a pair")
    split = dev.pget_u8("CLOCK.I2S_CLOCK_PIN_MODE") == 1
    role = 1 if (split and dev.pget_u8("CLOCK.I2S_CLOCK_MODE") == 1) else 0
    slot = profile.num_spdif - 1
    valid, free = _avail(dev, "CLOCK.I2S_BCK_PIN_AVAIL", role)["pairs"][0]
    held = next((g for g in range(64) if (valid >> g) & 1 and not (free >> g) & 1), None)
    if held is None:
        raise Skip("no valid clock pair overlaps a held GPIO")
    _refused_start(dev, chk, "I2S type", "CLOCK.I2S_BCK_PIN", role, held,
                   lambda: dev.pset("OUTPUT.TYPE", b"\x01", addr=slot))
    time.sleep(0.05)
    chk.eq(dev.pget_u8("OUTPUT.TYPE", slot), 0, "slot type unchanged")


@test("paged_pins", mutating=True)
def pdm_enable_refused_on_held_pin(dev, profile, chk):
    """A stopped PDM stores output 0's GPIO, and enabling the PDM output is refused with it left off."""
    _require(dev)
    out = profile.num_output_channels - 1
    if dev.pget_u8("ROUTE.OUTPUT_ENABLE", out):
        raise Skip("the PDM output is enabled")
    if dev.pget_u8("ROUTE.OUTPUT_ENABLE_CONFLICT", out):
        raise Skip("core 1 runs the EQ worker: the enable would be refused for that first")
    pdm = profile.num_pin_outputs - 1
    if _avail(dev, "OUTPUT.PIN_AVAIL", pdm)["status"] != 0:
        raise Skip("PDM is still running or fading out")
    _refused_start(dev, chk, "PDM enable", "OUTPUT.PIN", pdm, dev.pget_u8("OUTPUT.PIN", 0),
                   lambda: dev.pset("ROUTE.OUTPUT_ENABLE", b"\x01", addr=out))
    chk.eq(dev.pget_u8("ROUTE.OUTPUT_ENABLE", out), 0, "PDM output still disabled")


@test("paged_pins", mutating=True)
def availability_follows_input_blocks(dev, profile, chk):
    """S/PDIF inputs above the count and pairs of an off I2S block report no IN_USE; inputs within the count, and
    I2S pairs and a clock pair of an enabled block with I2S not selected, do. The generation moves on each
    switch and count change that changes a claim. USB source only; restores."""
    _require(dev)
    if dev.pget_u8("INPUT.SOURCE") != 0:
        raise Skip("input source is not USB")
    saved = input_blocks(dev)
    pairs = 4 if profile.platform_id == 1 else 1
    gen = [_pin_state(dev)["gen"]]

    def in_use(cmd, n):
        return [bool(_avail(dev, cmd, i)["flags"] & F_IN_USE) for i in range(n)]

    def moved(what):
        g = _pin_state(dev)["gen"]
        chk.ok(g != gen[0], f"generation changed after {what}")
        gen[0] = g
    try:
        dev.pset_u8("INPUT.SPDIF_RX_ENABLE", 0)
        dev.pset_u8("INPUT.SPDIF_RX_COUNT", 1)
        gen[0] = _pin_state(dev)["gen"]
        chk.eq(in_use("INPUT.SPDIF_RX_PIN_AVAIL", 4), [False] * 4, "no S/PDIF input in use with the block off")
        if try_set(dev, "INPUT.SPDIF_RX_ENABLE", b"\x01") == "VC_OK":
            moved("S/PDIF block on")
            for n in (1, 2, 3, 4):
                if n > 1:
                    if try_set(dev, "INPUT.SPDIF_RX_COUNT", bytes([n])) != "VC_OK":
                        chk.note(f"S/PDIF count {n} refused; walk stopped")
                        break
                    moved(f"S/PDIF count {n}")
                chk.eq(in_use("INPUT.SPDIF_RX_PIN_AVAIL", 4), [i < n for i in range(4)], f"S/PDIF in use at count {n}")
            dev.pset_u8("INPUT.SPDIF_RX_ENABLE", 0)
            moved("S/PDIF block off")
        else:
            chk.note("S/PDIF block refused to switch on; its part skipped")

        dev.pset_u8("INPUT.I2S_RX_ENABLE", 0)
        gen[0] = _pin_state(dev)["gen"]
        nch = dev.pget_u8("INPUT.I2S_RX_CHANNELS")
        chk.eq(in_use("INPUT.I2S_RX_PIN_AVAIL", pairs), [False] * pairs, "no I2S pair in use with the block off")
        if try_set(dev, "INPUT.I2S_RX_ENABLE", b"\x01") == "VC_OK":
            moved("I2S block on")
            chk.eq(in_use("INPUT.I2S_RX_PIN_AVAIL", pairs), [p < nch // 2 for p in range(pairs)],
                   "counted I2S pairs in use with I2S not selected")
            chk.ok(any(in_use("CLOCK.I2S_BCK_PIN_AVAIL", 2)), "a clock pair in use with I2S not selected")
            dev.pset_u8("INPUT.I2S_RX_ENABLE", 0)
            moved("I2S block off")
        else:
            chk.note("I2S block refused to switch on; its part skipped")
    finally:
        restore_input_blocks(dev, saved)
        chk.eq(input_blocks(dev), saved, "input blocks restored")


def _require_bulk_pins(dev):
    """Skip unless a bulk SET applies the pin and input fields (output-config mode WITH_PRESET) on USB."""
    if dev.pget_u8("STORE.OUTPUT_CONFIG_MODE") != OUTPUT_CONFIG_WITH_PRESET:
        raise Skip("output-config mode is not WITH_PRESET: a bulk SET leaves the input fields live")
    if dev.pget_u8("INPUT.SOURCE") != 0:
        raise Skip("input source is not USB")


def _pushed(dev, profile, chk, edit, check):
    """Push the live bulk blob with `edit(bytearray)` applied, run check(), then push the original back."""
    blob0 = read_bulk(dev, profile)
    saved = input_blocks(dev)
    b = bytearray(blob0)
    edit(b)
    try:
        push_bulk(dev, bytes(b))
        check()
    finally:
        push_bulk(dev, blob0)
        chk.eq(input_blocks(dev), saved, "input blocks restored by the original blob")
        chk.eq(read_bulk(dev, profile)[INPUT_CONFIG_OFF:INPUT_CONFIG_OFF + 16],
               blob0[INPUT_CONFIG_OFF:INPUT_CONFIG_OFF + 16], "input config section restored")


@test("paged_pins", mutating=True)
def bulk_push_cuts_spdif_count_on_clash(dev, profile, chk):
    """A bulk SET of S/PDIF count 4 with input 4 on output 0's GPIO cuts the count below 4, keeps the block
    on and the stored pin, and sets the S/PDIF bit in PIN_STATE's substituted mask. Restores the blob."""
    _require(dev)
    _require_bulk_pins(dev)
    held = dev.pget_u8("OUTPUT.PIN", 0)

    def edit(b):
        b[INPUT_CONFIG_OFF + 10] = held                                  # spdif_rx_pin_ext[2], input 4
        b[INPUT_CONFIG_OFF + 11] = 4                                     # spdif_rx_count
        b[INPUT_CONFIG_OFF + 14] = ((b[INPUT_CONFIG_OFF + 14] - 1) & 7 | 1) + 1   # S/PDIF bit on

    def check():
        cfg = spdif_rx_config(dev)
        chk.eq(cfg["enabled"], 1, "S/PDIF block still on")
        chk.ok(cfg["count"] <= 3, f"count cut below input 4 (got {cfg['count']})")
        chk.eq(cfg["pins"][3], held, "input 4 keeps its stored pin")
        chk.ok(_pin_state(dev)["subst"] >> FN_SPDIF_RX & 1, "S/PDIF bit set in the substituted mask")
        chk.eq(_avail(dev, "INPUT.SPDIF_RX_PIN_AVAIL", 3)["flags"] & F_IN_USE, 0, "input 4 not in use")
    _pushed(dev, profile, chk, edit, check)


@test("paged_pins", mutating=True)
def bulk_push_switches_i2s_off_on_clash(dev, profile, chk):
    """A bulk SET of the I2S block on with pair 0 on output 0's GPIO leaves the block off, I2S unselected, and
    sets the I2S RX bit in PIN_STATE's substituted mask. Restores the blob."""
    _require(dev)
    _require_bulk_pins(dev)
    held = dev.pget_u8("OUTPUT.PIN", 0)

    def edit(b):
        b[INPUT_CONFIG_OFF + 2] = held                                   # i2s_rx_pin, pair 0
        b[INPUT_CONFIG_OFF + 14] = ((b[INPUT_CONFIG_OFF + 14] - 1) & 7 | 2) + 1   # I2S bit on

    def check():
        chk.eq(dev.pget_u8("INPUT.I2S_RX_ENABLE"), 0, "I2S block switched off")
        chk.eq(dev.pget_u8("INPUT.SOURCE"), 0, "source still USB")
        chk.ok(_pin_state(dev)["subst"] >> FN_I2S_RX & 1, "I2S RX bit set in the substituted mask")
    _pushed(dev, profile, chk, edit, check)
