"""
Native paged tests for the CLOCK page (plan section 5.12), A/B against the 1.1.6 opcodes on page 0.
Never changed: the I2S clock mode, the clock-pin mode, the role 0 BCK pair and MCK enable. Moved and
restored: the dormant slave BCK pair (UNIFIED mode only), the MCK pin and multiplier (MCK disabled only).
"""

from ..framework import Skip, test
from .paged_input import BAD_GPIO, legacy, legacy_act, need_page, parity, try_set, u8
from .paged_sys import _expect_stall

CLOCK = 0x0C
PIN_SUCCESS = 0x00


@test("paged_clock")
def clock_parity(dev, profile, chk):
    """Every CLOCK GET matches its 1.1.6 twin."""
    need_page(dev, CLOCK)
    for role in (0, 1):
        parity(chk, dev, "CLOCK.I2S_BCK_PIN", "GET_I2S_BCK_PIN", 1, addr=role, label=f"[{role}]")
    parity(chk, dev, "CLOCK.I2S_CLOCK_PIN_MODE", "GET_I2S_CLOCK_PIN_MODE", 1)
    parity(chk, dev, "CLOCK.I2S_CLOCK_MODE", "GET_I2S_CLOCK_MODE", 1)
    parity(chk, dev, "CLOCK.MCK_ENABLE", "GET_MCK_ENABLE", 1)
    parity(chk, dev, "CLOCK.MCK_PIN", "GET_MCK_PIN", 1)
    parity(chk, dev, "CLOCK.MCK_MULTIPLIER", "GET_MCK_MULTIPLIER", 1)
    for _ in range(3):
        new = dev.pget("CLOCK.I2S_SLAVE_STATUS", length=16)
        old = legacy(dev, "GET_I2S_SLAVE_STATUS", 16)
        if old is None or new == old:
            break
    if old is not None:
        chk.eq(new, old, "I2S_SLAVE_STATUS == GET_I2S_SLAVE_STATUS")


@test("paged_clock", mutating=True)
def noop_sets_of_current_values(dev, profile, chk):
    """SET of the current value is accepted, except MCK_PIN while MCK runs (VC_ERR_STATE, as 1.1.6)."""
    need_page(dev, CLOCK)
    before = {n: u8(dev, f"CLOCK.{n}") for n in
              ("I2S_CLOCK_PIN_MODE", "I2S_CLOCK_MODE", "MCK_ENABLE", "MCK_PIN", "MCK_MULTIPLIER")}
    for role in (0, 1):
        v = u8(dev, "CLOCK.I2S_BCK_PIN", role)
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
    chk.eq({n: u8(dev, f"CLOCK.{n}") for n in before}, before, "CLOCK state unchanged")


@test("paged_clock", mutating=True)
def refused_writes(dev, profile, chk):
    """Bad role, value, pin and length STALL with the matching code and change nothing."""
    need_page(dev, CLOCK)
    E = _expect_stall
    snap = lambda: ([u8(dev, "CLOCK.I2S_BCK_PIN", r) for r in (0, 1)],
                    [u8(dev, f"CLOCK.{n}") for n in ("I2S_CLOCK_PIN_MODE", "I2S_CLOCK_MODE", "MCK_ENABLE",
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
    """The dormant slave BCK pair round-trips through both opcodes and restores (UNIFIED mode only)."""
    need_page(dev, CLOCK)
    if u8(dev, "CLOCK.I2S_CLOCK_PIN_MODE") != 0:
        raise Skip("SPLIT clock-pin mode: the slave pair may be live")
    orig = u8(dev, "CLOCK.I2S_BCK_PIN", 1)
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
        chk.eq(u8(dev, "CLOCK.I2S_BCK_PIN", 1), moved[-1], "new GET after SET")
        old = legacy(dev, "GET_I2S_BCK_PIN", 1, 1)
        if old is not None:
            chk.eq(old[0], moved[-1], "GET_I2S_BCK_PIN after new SET")
            if len(moved) == 2:
                chk.eq(legacy_act(dev, "SET_I2S_BCK_PIN", (1 << 8) | moved[0]), PIN_SUCCESS, "old SET")
                chk.eq(u8(dev, "CLOCK.I2S_BCK_PIN", 1), moved[0], "new GET after old SET")
    finally:
        dev.pset("CLOCK.I2S_BCK_PIN", bytes([orig]), addr=1)
        chk.eq(u8(dev, "CLOCK.I2S_BCK_PIN", 1), orig, "slave BCK restored")


@test("paged_clock", mutating=True)
def mck_storage_roundtrip(dev, profile, chk):
    """MCK multiplier and pin round-trip through both opcodes while MCK is disabled, then restore."""
    need_page(dev, CLOCK)
    if u8(dev, "CLOCK.MCK_ENABLE"):
        raise Skip("MCK is running: multiplier and pin left alone")
    mult, pin = u8(dev, "CLOCK.MCK_MULTIPLIER"), u8(dev, "CLOCK.MCK_PIN")
    try:
        for v in (1 - mult, mult):
            dev.pset("CLOCK.MCK_MULTIPLIER", bytes([v]))
            chk.eq(u8(dev, "CLOCK.MCK_MULTIPLIER"), v, f"multiplier -> {v}")
            old = legacy(dev, "GET_MCK_MULTIPLIER", 1)
            if old is not None:
                chk.eq(old[0], v, f"GET_MCK_MULTIPLIER after SET {v}")
        if legacy(dev, "GET_MCK_MULTIPLIER", 1) is not None:
            chk.eq(legacy_act(dev, "SET_MCK_MULTIPLIER", 1 - mult), PIN_SUCCESS, "old SET multiplier")
            chk.eq(u8(dev, "CLOCK.MCK_MULTIPLIER"), 1 - mult, "new GET after old SET")
        gpout = (13, 15, 21) if profile.platform_id == 1 else (21,)
        for cand in (p for p in gpout if p != pin):
            r = try_set(dev, "CLOCK.MCK_PIN", bytes([cand]))
            if r != "VC_OK":
                chk.eq(r, "VC_ERR_PIN_IN_USE", f"MCK pin {cand} refusal code")
                continue
            chk.eq(u8(dev, "CLOCK.MCK_PIN"), cand, f"MCK pin -> {cand}")
            old = legacy(dev, "GET_MCK_PIN", 1)
            if old is not None:
                chk.eq(old[0], cand, "GET_MCK_PIN after new SET")
            break
        else:
            chk.note("no other GPOUT pin free for MCK")
    finally:
        dev.pset("CLOCK.MCK_MULTIPLIER", bytes([mult]))
        dev.pset("CLOCK.MCK_PIN", bytes([pin]))
        chk.eq((u8(dev, "CLOCK.MCK_MULTIPLIER"), u8(dev, "CLOCK.MCK_PIN")), (mult, pin), "MCK restored")
