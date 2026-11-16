"""
Native paged tests for the CLOCK page (plan section 5.12): GET shapes, round trips and refusals.
Never changed: the I2S clock mode, the clock-pin mode, the role 0 BCK pair and MCK enable. Moved and
restored: the dormant slave BCK pair (UNIFIED mode only), the MCK pin and multiplier (MCK disabled only).
"""

from ..framework import Skip, test
from ..helpers import BAD_GPIO, HIGHEST_GPIO, expect_stall, read_len, require_page, try_set

CLOCK = 0x0C


@test("paged_clock")
def clock_shapes(dev, profile, chk):
    """Every CLOCK GET has its table length and a documented value; I2S_SLAVE_STATUS echoes the clock mode."""
    require_page(dev, CLOCK)
    for role in (0, 1):
        chk.in_range(read_len(chk, dev, "CLOCK.I2S_BCK_PIN", 1, addr=role, label=f"[{role}]")[0], 0, HIGHEST_GPIO - 1,
                     f"BCK pin role {role} (LRCLK is the next GPIO)")
    chk.member(read_len(chk, dev, "CLOCK.I2S_CLOCK_PIN_MODE", 1)[0], (0, 1), "clock-pin mode")
    mode = read_len(chk, dev, "CLOCK.I2S_CLOCK_MODE", 1)[0]
    chk.member(mode, (0, 1), "I2S clock mode")
    chk.member(read_len(chk, dev, "CLOCK.MCK_ENABLE", 1)[0], (0, 1), "MCK enable")
    chk.in_range(read_len(chk, dev, "CLOCK.MCK_PIN", 1)[0], 0, HIGHEST_GPIO, "MCK pin")
    chk.member(read_len(chk, dev, "CLOCK.MCK_MULTIPLIER", 1)[0], (0, 1), "MCK multiplier code")
    st = read_len(chk, dev, "CLOCK.I2S_SLAVE_STATUS", 16)
    chk.in_range(st[0], 0, 3, "slave state")
    chk.eq(st[1], mode, "I2S_SLAVE_STATUS clock mode == I2S_CLOCK_MODE")
    chk.eq(st[13:16], bytes(3), "I2S_SLAVE_STATUS reserved")
    if st[0] != 3:
        chk.eq(st[4:8], bytes(4), "detected rate is 0 unless LOCKED")


@test("paged_clock", mutating=True)
def noop_sets_of_current_values(dev, profile, chk):
    """SET of the current value is accepted, except MCK_PIN while MCK runs (VC_ERR_STATE, as 1.1.6)."""
    require_page(dev, CLOCK)
    before = {n: dev.pget_u8(f"CLOCK.{n}") for n in
              ("I2S_CLOCK_PIN_MODE", "I2S_CLOCK_MODE", "MCK_ENABLE", "MCK_PIN", "MCK_MULTIPLIER")}
    for role in (0, 1):
        v = dev.pget_u8("CLOCK.I2S_BCK_PIN", role)
        chk.eq(try_set(dev, "CLOCK.I2S_BCK_PIN", bytes([v]), role), "VC_OK", f"BCK[{role}] = current")
    chk.eq(try_set(dev, "CLOCK.I2S_CLOCK_PIN_MODE", bytes([before["I2S_CLOCK_PIN_MODE"]])), "VC_OK",
           "clock-pin mode = current")
    chk.eq(try_set(dev, "CLOCK.I2S_CLOCK_MODE", bytes([before["I2S_CLOCK_MODE"]])), "VC_OK",
           "clock mode = current")
    chk.eq(try_set(dev, "CLOCK.MCK_ENABLE", bytes([before["MCK_ENABLE"]])), "VC_OK", "MCK enable = current")
    chk.eq(try_set(dev, "CLOCK.MCK_MULTIPLIER", bytes([before["MCK_MULTIPLIER"]])), "VC_OK",
           "MCK multiplier = current")
    want = "VC_ERR_STATE" if before["MCK_ENABLE"] else "VC_OK"
    chk.eq(try_set(dev, "CLOCK.MCK_PIN", bytes([before["MCK_PIN"]])), want, "MCK pin = current")
    dev.wait_ready()
    chk.eq({n: dev.pget_u8(f"CLOCK.{n}") for n in before}, before, "CLOCK state unchanged")


@test("paged_clock", mutating=True)
def refused_writes(dev, profile, chk):
    """Bad role, value, pin and length STALL with the matching code and change nothing."""
    require_page(dev, CLOCK)
    E = expect_stall
    snap = lambda: ([dev.pget_u8("CLOCK.I2S_BCK_PIN", r) for r in (0, 1)],
                    [dev.pget_u8(f"CLOCK.{n}") for n in ("I2S_CLOCK_PIN_MODE", "I2S_CLOCK_MODE", "MCK_ENABLE",
                                                         "MCK_PIN", "MCK_MULTIPLIER")])
    before = snap()
    E(chk, lambda: dev.pget("CLOCK.I2S_BCK_PIN", addr=2, length=1), "VC_ERR_ADDRESS", "GET BCK role 2")
    E(chk, lambda: dev.pset("CLOCK.I2S_BCK_PIN", b"\x10", addr=2), "VC_ERR_ADDRESS", "SET BCK role 2")
    E(chk, lambda: dev.pset("CLOCK.I2S_BCK_PIN", bytes([BAD_GPIO]), addr=1), "VC_ERR_PIN_INVALID", "BCK 0xFE")
    E(chk, lambda: dev.pset("CLOCK.I2S_CLOCK_PIN_MODE", b"\x02"), "VC_ERR_VALUE", "clock-pin mode 2")
    E(chk, lambda: dev.pset("CLOCK.I2S_CLOCK_MODE", b"\x02"), "VC_ERR_VALUE", "clock mode 2")
    E(chk, lambda: dev.pset("CLOCK.I2S_CLOCK_MODE", b"\x00\x00"), "VC_ERR_LENGTH", "2-byte clock mode")
    E(chk, lambda: dev.pset("CLOCK.MCK_MULTIPLIER", b"\x02"), "VC_ERR_VALUE", "MCK multiplier 2")
    E(chk, lambda: dev.pset("CLOCK.MCK_PIN", bytes([BAD_GPIO])), "VC_ERR_PIN_INVALID", "MCK pin 0xFE")
    E(chk, lambda: dev.pset("CLOCK.MCK_PIN", b"\x10"), "VC_ERR_PIN_INVALID", "MCK pin 16 (no GPOUT)")
    E(chk, lambda: dev.pset("CLOCK.I2S_SLAVE_STATUS", b"\x00"), "VC_ERR_DIRECTION", "SET to I2S_SLAVE_STATUS")
    E(chk, lambda: dev.pget("CLOCK.MCK_ENABLE", addr=1, length=1), "VC_ERR_ADDRESS", "MCK_ENABLE at wValue 1")
    dev.wait_ready()
    chk.eq(snap(), before, "CLOCK state unchanged")


@test("paged_clock", mutating=True)
def slave_bck_pair_roundtrip(dev, profile, chk):
    """The dormant slave BCK pair round-trips through two moves and restores (UNIFIED mode only)."""
    require_page(dev, CLOCK)
    if dev.pget_u8("CLOCK.I2S_CLOCK_PIN_MODE") != 0:
        raise Skip("SPLIT clock-pin mode: the slave pair may be live")
    orig = dev.pget_u8("CLOCK.I2S_BCK_PIN", 1)
    moved = []
    try:
        for cand in (16, 18, 20, 26, 22):
            if cand in (orig, orig + 1) or cand + 1 == orig:
                continue
            r = try_set(dev, "CLOCK.I2S_BCK_PIN", bytes([cand]), 1)
            if r == "VC_OK":
                moved.append(cand)
                if len(moved) == 2:
                    break
            else:
                chk.member(r, ("VC_ERR_PIN_IN_USE", "VC_ERR_PIN_INVALID"), f"pair at {cand} refusal code")
        if not moved:
            chk.note("no free pin pair accepted for the slave BCK")
            return
        chk.eq(dev.pget_u8("CLOCK.I2S_BCK_PIN", 1), moved[-1], "GET after SET")
        if len(moved) == 2:
            dev.pset("CLOCK.I2S_BCK_PIN", bytes([moved[0]]), addr=1)
            chk.eq(dev.pget_u8("CLOCK.I2S_BCK_PIN", 1), moved[0], "GET after moving back to the first pair")
    finally:
        dev.pset("CLOCK.I2S_BCK_PIN", bytes([orig]), addr=1)
        chk.eq(dev.pget_u8("CLOCK.I2S_BCK_PIN", 1), orig, "slave BCK restored")


@test("paged_clock", mutating=True)
def mck_storage_roundtrip(dev, profile, chk):
    """MCK multiplier and pin round-trip while MCK is disabled, then restore."""
    require_page(dev, CLOCK)
    if dev.pget_u8("CLOCK.MCK_ENABLE"):
        raise Skip("MCK is running: multiplier and pin left alone")
    mult, pin = dev.pget_u8("CLOCK.MCK_MULTIPLIER"), dev.pget_u8("CLOCK.MCK_PIN")
    try:
        for v in (1 - mult, mult):
            dev.pset("CLOCK.MCK_MULTIPLIER", bytes([v]))
            chk.eq(dev.pget_u8("CLOCK.MCK_MULTIPLIER"), v, f"multiplier -> {v}")
        gpout = (13, 15, 21) if profile.platform_id == 1 else (21,)
        for cand in (p for p in gpout if p != pin):
            r = try_set(dev, "CLOCK.MCK_PIN", bytes([cand]))
            if r != "VC_OK":
                chk.eq(r, "VC_ERR_PIN_IN_USE", f"MCK pin {cand} refusal code")
                continue
            chk.eq(dev.pget_u8("CLOCK.MCK_PIN"), cand, f"MCK pin -> {cand}")
            break
        else:
            chk.note("no other GPOUT pin free for MCK")
    finally:
        dev.pset("CLOCK.MCK_MULTIPLIER", bytes([mult]))
        dev.pset("CLOCK.MCK_PIN", bytes([pin]))
        chk.eq((dev.pget_u8("CLOCK.MCK_MULTIPLIER"), dev.pget_u8("CLOCK.MCK_PIN")), (mult, pin), "MCK restored")
