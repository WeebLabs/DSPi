"""
Native paged tests for the INPUT page (plan section 5.3), A/B against the 1.1.6 opcodes on page 0.
Never switches the input source, clock modes or anything live, and never moves I2S RX pair 1. Writes touch
only settings that are storage-only while USB is the source (disabled S/PDIF inputs, inactive I2S pairs, the
I2S rate, the disabled ADAT input pin) and restore them. Helpers here serve paged_output/clock/xport too.
"""

import struct

from ..device import OP, PagedStall, Stall
from ..framework import Skip, test
from .paged_sys import _expect_stall, _paged

INPUT = 0x03
INPUT_USB, INPUT_I2S, INPUT_ADAT = 0, 2, 3
I2S_RATES = (44100, 48000, 96000)
BAD_GPIO = 0xFE           # never a valid GPIO; 0xFF is the reset sentinel
PIN_SUCCESS = 0x00


def need_page(dev, page):
    _paged(dev)
    if page not in dev.paged_commands():
        raise Skip(f"page 0x{page:02X} not implemented on this firmware")


def has_cmd(dev, name) -> bool:
    info = dev.resolve(name)
    g, s = dev.paged_commands().get(info.page, (frozenset(), frozenset()))
    return info.cmd in g or info.cmd in s


def legacy(dev, opname, n, wvalue=0):
    """Untranslated page-0 GET, or None once page 0 no longer answers it."""
    try:
        return dev.raw_get(getattr(OP, opname), n, wvalue)
    except Stall:
        return None


def legacy_act(dev, opname, wvalue):
    """1.1.6 write-as-read: the status byte, or None once page 0 no longer answers it."""
    r = legacy(dev, opname, 1, wvalue)
    return r[0] if r else None


def parity(chk, dev, name, opname, n, addr=0, wvalue=None, label=""):
    new = dev.pget(name, addr=addr, length=n)
    old = legacy(dev, opname, n, addr if wvalue is None else wvalue)
    if old is None:
        chk.note(f"page 0 no longer answers {opname}: parity skipped")
        return new
    chk.eq(new, old, f"{name}{label} == {opname}")
    return new


def try_set(dev, name, data, addr=0):
    """pset that returns the VC result name ('VC_OK' on success)."""
    try:
        dev.pset(name, data, addr=addr)
        return "VC_OK"
    except PagedStall as s:
        return s.result_name or "STALL"


def u8(dev, name, addr=0):
    return dev.pget(name, addr=addr, length=1)[0]


def _source(dev):
    return u8(dev, "INPUT.SOURCE")


@test("paged_input")
def scalar_parity(dev, profile, chk):
    """Every INPUT GET that keeps a 1.1.6 twin returns the same bytes."""
    need_page(dev, INPUT)
    parity(chk, dev, "INPUT.SOURCE", "GET_INPUT_SOURCE", 1)
    parity(chk, dev, "INPUT.SPDIF_RX_CONFIG", "GET_SPDIF_INPUT_CONFIG", 6)
    parity(chk, dev, "INPUT.SPDIF_RX_CH_STATUS", "GET_SPDIF_RX_CH_STATUS", 24)
    parity(chk, dev, "INPUT.I2S_RX_CHANNELS", "GET_I2S_INPUT_CHANNELS", 1)
    parity(chk, dev, "INPUT.LG_SOUND_SYNC_ENABLE", "GET_LG_SOUND_SYNC_ENABLE", 1)
    parity(chk, dev, "INPUT.LG_SOUND_SYNC_STATUS", "GET_LG_SOUND_SYNC_STATUS", 16)
    for i in range(4):
        parity(chk, dev, "INPUT.SPDIF_RX_PIN", "GET_SPDIF_RX_PIN", 1, addr=i, label=f"[{i}]")
    pairs = 4 if profile.platform_id == 1 else 1
    for p in range(pairs):
        parity(chk, dev, "INPUT.I2S_RX_PIN", "GET_I2S_RX_PIN", 1, addr=p, label=f"[{p}]")
    for s in range(7):
        parity(chk, dev, "INPUT.SOURCE_NAME", "GET_INPUT_SOURCE_NAME", 32, addr=s, label=f"[{s}]")
    rate = dev.pget("INPUT.I2S_RX_RATE", length=4)
    old = legacy(dev, "GET_INPUT_RATE", 8)
    if old is not None:
        chk.eq(rate, old[4:8], "I2S_RX_RATE == GET_INPUT_RATE selected half")
    if has_cmd(dev, "INPUT.ADAT_RX_ENABLE"):
        parity(chk, dev, "INPUT.ADAT_RX_ENABLE", "GET_ADAT_INPUT_ENABLE", 1)
        parity(chk, dev, "INPUT.ADAT_RX_PIN", "GET_ADAT_INPUT_PIN", 1)
        parity(chk, dev, "INPUT.ADAT_RX_CLOCK_MODE", "GET_ADAT_INPUT_CLOCK_MODE", 1)
    else:
        chk.note("no ADAT RX commands on this build")


@test("paged_input")
def status_packets_parity(dev, profile, chk):
    """S/PDIF RX and ADAT RX status match their 1.1.6 twins outside the moving fields."""
    need_page(dev, INPUT)
    for _ in range(3):
        new = dev.pget("INPUT.SPDIF_RX_STATUS", length=16)
        old = legacy(dev, "GET_SPDIF_RX_STATUS", 16)
        if old is None or new[:12] == old[:12]:
            break
    if old is not None:
        chk.eq(len(new), 16, "SPDIF_RX_STATUS length")
        chk.eq(new[:12], old[:12], "SPDIF_RX_STATUS stable fields == GET_SPDIF_RX_STATUS")
    if has_cmd(dev, "INPUT.ADAT_RX_STATUS"):
        new = dev.pget("INPUT.ADAT_RX_STATUS", length=20)
        old = legacy(dev, "GET_ADAT_INPUT_STATUS", 20)
        chk.eq(len(new), 20, "ADAT_RX_STATUS length")
        if old is not None:
            chk.eq(new[:8], old[:8], "ADAT_RX_STATUS config and counters == GET_ADAT_INPUT_STATUS")


@test("paged_input")
def status_struct(dev, profile, chk):
    """INPUT.STATUS carries the source and GET_STATUS selectors 23, 32 and 33."""
    need_page(dev, INPUT)
    st = dev.pget("INPUT.STATUS", length=12)
    chk.eq(len(st), 12, "STATUS length")
    src, nch, rsv, rate_mhz, trim = struct.unpack("<BBHIi", st)
    chk.eq(src, _source(dev), "source == INPUT.SOURCE")
    chk.eq(rsv, 0, "reserved")
    old = legacy(dev, "GET_STATUS", 4, 23)
    if old is None:
        chk.note("page 0 no longer answers GET_STATUS: selector parity skipped")
        return
    chk.eq(nch, struct.unpack("<I", old)[0], "active channels == GET_STATUS 23")
    old_rate = struct.unpack("<I", legacy(dev, "GET_STATUS", 4, 32))[0]
    chk.approx(rate_mhz, old_rate, max(50, old_rate // 10000), "measured rate == GET_STATUS 32")
    old_trim = struct.unpack("<i", legacy(dev, "GET_STATUS", 4, 33))[0]
    chk.approx(trim, old_trim, 200, "VCXO trim == GET_STATUS 33")


@test("paged_input")
def spdif_rx_enable_get(dev, profile, chk):
    """SPDIF_RX_ENABLE GET is the enable bit of inputs 1-3 in SPDIF_RX_CONFIG; input 0 is VC_ERR_ADDRESS."""
    need_page(dev, INPUT)
    mask = dev.pget("INPUT.SPDIF_RX_CONFIG", length=6)[1]
    for i in (1, 2, 3):
        chk.eq(u8(dev, "INPUT.SPDIF_RX_ENABLE", i), (mask >> i) & 1, f"input {i} enable bit")
    _expect_stall(chk, lambda: dev.pget("INPUT.SPDIF_RX_ENABLE", addr=0, length=1), "VC_ERR_ADDRESS",
                  "GET SPDIF_RX_ENABLE input 0")
    _expect_stall(chk, lambda: dev.pset("INPUT.SPDIF_RX_ENABLE", b"\x01", addr=0), "VC_ERR_ADDRESS",
                  "SET SPDIF_RX_ENABLE input 0")


@test("paged_input")
def address_length_direction_errors(dev, profile, chk):
    """Bad addresses, lengths and directions STALL with the matching VC code."""
    need_page(dev, INPUT)
    pairs = 4 if profile.platform_id == 1 else 1
    E = _expect_stall
    E(chk, lambda: dev.pget("INPUT.SOURCE_NAME", addr=7, length=32), "VC_ERR_ADDRESS", "SOURCE_NAME source 7")
    E(chk, lambda: dev.pset("INPUT.SOURCE_NAME", b"x", addr=7), "VC_ERR_ADDRESS", "SET SOURCE_NAME source 7")
    E(chk, lambda: dev.pget("INPUT.SPDIF_RX_PIN", addr=4, length=1), "VC_ERR_ADDRESS", "SPDIF_RX_PIN input 4")
    E(chk, lambda: dev.pset("INPUT.SPDIF_RX_PIN", b"\x05", addr=4), "VC_ERR_ADDRESS", "SET SPDIF_RX_PIN input 4")
    E(chk, lambda: dev.pget("INPUT.I2S_RX_PIN", addr=pairs, length=1), "VC_ERR_ADDRESS", f"I2S_RX_PIN pair {pairs}")
    E(chk, lambda: dev.pget("INPUT.SOURCE", addr=1, length=1), "VC_ERR_ADDRESS", "SOURCE at wValue 1")
    E(chk, lambda: dev.pget("INPUT.STATUS", addr=1, length=12), "VC_ERR_ADDRESS", "STATUS at wValue 1")
    E(chk, lambda: dev.pset("INPUT.SOURCE", b"\x00\x00"), "VC_ERR_LENGTH", "2-byte SOURCE")
    E(chk, lambda: dev.pset("INPUT.SOURCE"), "VC_ERR_LENGTH", "empty SOURCE")
    E(chk, lambda: dev.pset("INPUT.I2S_RX_RATE", b"\x80\xbb"), "VC_ERR_LENGTH", "2-byte I2S_RX_RATE")
    E(chk, lambda: dev.pset("INPUT.SOURCE_NAME", bytes(33), addr=0), "VC_ERR_LENGTH", "33-byte SOURCE_NAME")
    E(chk, lambda: dev.pset("INPUT.SOURCE_NAME", b"", addr=0), "VC_ERR_LENGTH", "empty SOURCE_NAME")
    for ro in ("STATUS", "SPDIF_RX_STATUS", "SPDIF_RX_CH_STATUS", "SPDIF_RX_CONFIG", "LG_SOUND_SYNC_STATUS"):
        E(chk, lambda: dev.pset(f"INPUT.{ro}", b"\x00"), "VC_ERR_DIRECTION", f"SET to {ro}")


@test("paged_input", mutating=True)
def value_errors_leave_state(dev, profile, chk):
    """Refused values STALL with VC_ERR_VALUE / PIN_* / STATE and change nothing."""
    need_page(dev, INPUT)
    E = _expect_stall
    src = _source(dev)
    rate = dev.pget("INPUT.I2S_RX_RATE", length=4)
    nch = u8(dev, "INPUT.I2S_RX_CHANNELS")
    E(chk, lambda: dev.pset("INPUT.SOURCE", b"\x07"), "VC_ERR_VALUE", "source 7")
    mask = dev.pget("INPUT.SPDIF_RX_CONFIG", length=6)[1]
    off = next((i for i in (1, 2, 3) if not (mask >> i) & 1), None)
    if off is not None and src != 3 + off:
        E(chk, lambda: dev.pset("INPUT.SOURCE", bytes([3 + off])), "VC_ERR_STATE",
          f"disabled S/PDIF input {off} as source")
    E(chk, lambda: dev.pset("INPUT.I2S_RX_RATE", struct.pack("<I", 88200)), "VC_ERR_VALUE", "rate 88200")
    E(chk, lambda: dev.pset("INPUT.I2S_RX_CHANNELS", b"\x03"), "VC_ERR_VALUE", "3 channels")
    if profile.platform_id != 1:
        E(chk, lambda: dev.pset("INPUT.I2S_RX_CHANNELS", b"\x04"), "VC_ERR_UNSUPPORTED", "4 channels on RP2040")
    E(chk, lambda: dev.pset("INPUT.SPDIF_RX_PIN", bytes([BAD_GPIO]), addr=0), "VC_ERR_PIN_INVALID", "S/PDIF pin 0xFE")
    out0 = u8(dev, "OUTPUT.PIN", 0) if has_cmd(dev, "OUTPUT.PIN") else dev.get_u8(OP.GET_OUTPUT_PIN, 0)
    if u8(dev, "INPUT.SPDIF_RX_PIN", 0) != out0:
        E(chk, lambda: dev.pset("INPUT.SPDIF_RX_PIN", bytes([out0]), addr=0), "VC_ERR_PIN_IN_USE",
          f"S/PDIF input 0 onto output pin {out0}")
    E(chk, lambda: dev.pset("INPUT.I2S_RX_PIN", bytes([BAD_GPIO]), addr=0), "VC_ERR_PIN_INVALID", "I2S pin 0xFE")
    if u8(dev, "INPUT.I2S_RX_PIN", 0) != out0:
        E(chk, lambda: dev.pset("INPUT.I2S_RX_PIN", bytes([out0]), addr=0), "VC_ERR_PIN_IN_USE",
          f"I2S pair 0 onto output pin {out0}")
    if has_cmd(dev, "INPUT.ADAT_RX_ENABLE"):
        E(chk, lambda: dev.pset("INPUT.ADAT_RX_ENABLE", b"\x02"), "VC_ERR_VALUE", "ADAT enable 2")
        E(chk, lambda: dev.pset("INPUT.ADAT_RX_CLOCK_MODE", b"\x02"), "VC_ERR_VALUE", "ADAT clock mode 2")
        E(chk, lambda: dev.pset("INPUT.ADAT_RX_PIN", bytes([BAD_GPIO])), "VC_ERR_PIN_INVALID", "ADAT pin 0xFE")
    chk.eq(_source(dev), src, "source unchanged")
    chk.eq(dev.pget("INPUT.I2S_RX_RATE", length=4), rate, "I2S rate unchanged")
    chk.eq(u8(dev, "INPUT.I2S_RX_CHANNELS"), nch, "I2S channels unchanged")


@test("paged_input", mutating=True)
def noop_sets_of_current_values(dev, profile, chk):
    """SET of the current value is accepted and changes nothing."""
    need_page(dev, INPUT)
    src = _source(dev)
    before = {n: dev.pget(f"INPUT.{n}", length=6) for n in ("SPDIF_RX_CONFIG",)}
    chk.eq(try_set(dev, "INPUT.SOURCE", bytes([src])), "VC_OK", "SOURCE = current")
    for i in range(4):
        p = u8(dev, "INPUT.SPDIF_RX_PIN", i)
        chk.eq(try_set(dev, "INPUT.SPDIF_RX_PIN", bytes([p]), i), "VC_OK", f"SPDIF_RX_PIN[{i}] = current")
    for i in (1, 2, 3):
        e = u8(dev, "INPUT.SPDIF_RX_ENABLE", i)
        chk.eq(try_set(dev, "INPUT.SPDIF_RX_ENABLE", bytes([e]), i), "VC_OK", f"SPDIF_RX_ENABLE[{i}] = current")
    pairs = 4 if profile.platform_id == 1 else 1
    for p in range(pairs):
        v = u8(dev, "INPUT.I2S_RX_PIN", p)
        chk.eq(try_set(dev, "INPUT.I2S_RX_PIN", bytes([v]), p), "VC_OK", f"I2S_RX_PIN[{p}] = current")
    nch = u8(dev, "INPUT.I2S_RX_CHANNELS")
    chk.eq(try_set(dev, "INPUT.I2S_RX_CHANNELS", bytes([nch])), "VC_OK", "I2S_RX_CHANNELS = current")
    lg = u8(dev, "INPUT.LG_SOUND_SYNC_ENABLE")
    chk.eq(try_set(dev, "INPUT.LG_SOUND_SYNC_ENABLE", bytes([lg])), "VC_OK", "LG enable = current")
    if has_cmd(dev, "INPUT.ADAT_RX_ENABLE"):
        for n in ("ADAT_RX_ENABLE", "ADAT_RX_PIN", "ADAT_RX_CLOCK_MODE"):
            v = u8(dev, f"INPUT.{n}")
            if n == "ADAT_RX_PIN" and v == 0xFF:
                continue   # 0xFF as a payload means "clear", not "this pin"
            chk.eq(try_set(dev, f"INPUT.{n}", bytes([v])), "VC_OK", f"{n} = current")
    chk.eq(_source(dev), src, "source unchanged")
    chk.eq(dev.pget("INPUT.SPDIF_RX_CONFIG", length=6), before["SPDIF_RX_CONFIG"], "S/PDIF config unchanged")
    chk.eq(u8(dev, "INPUT.LG_SOUND_SYNC_ENABLE"), lg, "LG enable unchanged")


def _require_usb_source(dev):
    if _source(dev) != INPUT_USB:
        raise Skip("input source is not USB: storage-only writes skipped")


@test("paged_input", mutating=True)
def i2s_rate_roundtrip(dev, profile, chk):
    """I2S_RX_RATE round-trips through the new and old commands (USB source only) and restores."""
    need_page(dev, INPUT)
    _require_usb_source(dev)
    orig = dev.pget("INPUT.I2S_RX_RATE", length=4)
    try:
        for hz in I2S_RATES:
            dev.pset("INPUT.I2S_RX_RATE", struct.pack("<I", hz))
            chk.eq(struct.unpack("<I", dev.pget("INPUT.I2S_RX_RATE", length=4))[0], hz, f"new GET after SET {hz}")
            old = legacy(dev, "GET_INPUT_RATE", 8)
            if old is not None:
                chk.eq(struct.unpack_from("<I", old, 4)[0], hz, f"GET_INPUT_RATE after SET {hz}")
        if legacy(dev, "GET_INPUT_RATE", 8) is not None:
            dev.raw_set(OP.SET_INPUT_RATE, struct.pack("<I", 44100))
            chk.eq(struct.unpack("<I", dev.pget("INPUT.I2S_RX_RATE", length=4))[0], 44100, "new GET after old SET")
    finally:
        dev.pset("INPUT.I2S_RX_RATE", orig)
        chk.eq(dev.pget("INPUT.I2S_RX_RATE", length=4), orig, "rate restored")


def _disabled_spdif_input(dev):
    mask = dev.pget("INPUT.SPDIF_RX_CONFIG", length=6)[1]
    return next((i for i in (3, 2, 1) if not (mask >> i) & 1), None)


@test("paged_input", mutating=True)
def spdif_rx_pin_disabled_input_roundtrip(dev, profile, chk):
    """A disabled S/PDIF input's stored pin round-trips through both opcodes, 0xFF resets, and restores."""
    need_page(dev, INPUT)
    _require_usb_source(dev)
    idx = _disabled_spdif_input(dev)
    if idx is None:
        raise Skip("every optional S/PDIF input is enabled")
    orig = u8(dev, "INPUT.SPDIF_RX_PIN", idx)
    a, b = [p for p in (19, 18, 17, 16) if p != orig][:2]
    try:
        dev.pset("INPUT.SPDIF_RX_PIN", bytes([a]), addr=idx)
        chk.eq(u8(dev, "INPUT.SPDIF_RX_PIN", idx), a, f"new GET after SET {a}")
        old = legacy(dev, "GET_SPDIF_RX_PIN", 1, idx)
        if old is not None:
            chk.eq(old[0], a, "GET_SPDIF_RX_PIN after new SET")
            chk.eq(legacy_act(dev, "SET_SPDIF_RX_PIN", (idx << 8) | b), PIN_SUCCESS, f"old SET {b}")
            chk.eq(u8(dev, "INPUT.SPDIF_RX_PIN", idx), b, "new GET after old SET")
        dev.pset("INPUT.SPDIF_RX_PIN", b"\xff", addr=idx)
        chk.eq(u8(dev, "INPUT.SPDIF_RX_PIN", idx), (20, 21, 22)[idx - 1], "0xFF restores the default")
    finally:
        dev.pset("INPUT.SPDIF_RX_PIN", bytes([orig]), addr=idx)
        chk.eq(u8(dev, "INPUT.SPDIF_RX_PIN", idx), orig, "pin restored")


@test("paged_input", mutating=True)
def spdif_rx_enable_roundtrip(dev, profile, chk):
    """Enabling a disabled optional S/PDIF input and disabling it again (USB source only)."""
    need_page(dev, INPUT)
    _require_usb_source(dev)
    idx = _disabled_spdif_input(dev)
    if idx is None:
        raise Skip("every optional S/PDIF input is enabled")
    r = try_set(dev, "INPUT.SPDIF_RX_ENABLE", b"\x01", idx)
    try:
        if r != "VC_OK":
            chk.member(r, ("VC_ERR_PIN_IN_USE", "VC_ERR_PIN_INVALID"), f"enable input {idx} refused")
            chk.eq(u8(dev, "INPUT.SPDIF_RX_ENABLE", idx), 0, "refused enable left it disabled")
            return
        chk.eq(u8(dev, "INPUT.SPDIF_RX_ENABLE", idx), 1, "new GET after enable")
        chk.eq((dev.pget("INPUT.SPDIF_RX_CONFIG", length=6)[1] >> idx) & 1, 1, "SPDIF_RX_CONFIG mask bit")
        dev.pset("INPUT.SPDIF_RX_ENABLE", b"\x00", addr=idx)
        chk.eq(u8(dev, "INPUT.SPDIF_RX_ENABLE", idx), 0, "new GET after disable")
        if legacy(dev, "GET_SPDIF_INPUT_CONFIG", 6) is not None:
            chk.eq(legacy_act(dev, "SET_SPDIF_INPUT_ENABLE", (idx << 8) | 1), PIN_SUCCESS, "old enable")
            chk.eq(u8(dev, "INPUT.SPDIF_RX_ENABLE", idx), 1, "new GET after old enable")
    finally:
        dev.pset("INPUT.SPDIF_RX_ENABLE", b"\x00", addr=idx)
        chk.eq(u8(dev, "INPUT.SPDIF_RX_ENABLE", idx), 0, "input disabled again")


@test("paged_input", mutating=True)
def i2s_rx_pin_inactive_pair_roundtrip(dev, profile, chk):
    """An inactive I2S RX pair's parked pin round-trips through both opcodes and restores (RP2350)."""
    need_page(dev, INPUT)
    if profile.platform_id != 1:
        raise Skip("RP2040 has one I2S RX pair")
    if _source(dev) == INPUT_I2S:
        raise Skip("I2S is the active input")
    active = u8(dev, "INPUT.I2S_RX_CHANNELS") // 2
    pair = 3
    if pair < active:
        raise Skip("pair 3 is active")
    orig = u8(dev, "INPUT.I2S_RX_PIN", pair)
    moved = []
    try:
        for cand in (19, 18, 17, 16, 20, 22):
            if cand == orig:
                continue
            r = try_set(dev, "INPUT.I2S_RX_PIN", bytes([cand]), pair)
            if r == "VC_OK":
                moved.append(cand)
                if len(moved) == 2:
                    break
            else:
                chk.member(r, ("VC_ERR_PIN_IN_USE", "VC_ERR_PIN_INVALID"), f"pin {cand} refusal code")
        if not moved:
            chk.note("no candidate pin accepted for pair 3")
            return
        chk.eq(u8(dev, "INPUT.I2S_RX_PIN", pair), moved[-1], "new GET after SET")
        old = legacy(dev, "GET_I2S_RX_PIN", 1, pair)
        if old is not None:
            chk.eq(old[0], moved[-1], "GET_I2S_RX_PIN after new SET")
            chk.eq(legacy_act(dev, "SET_I2S_RX_PIN", (pair << 8) | moved[0]), PIN_SUCCESS, "old SET")
            chk.eq(u8(dev, "INPUT.I2S_RX_PIN", pair), moved[0], "new GET after old SET")
    finally:
        dev.pset("INPUT.I2S_RX_PIN", bytes([orig]), addr=pair)
        chk.eq(u8(dev, "INPUT.I2S_RX_PIN", pair), orig, "pair 3 pin restored")


@test("paged_input", mutating=True)
def adat_rx_pin_disabled_roundtrip(dev, profile, chk):
    """The disabled ADAT input's pin round-trips, 0xFF clears it, and it restores (RP2350, USB source)."""
    need_page(dev, INPUT)
    if not has_cmd(dev, "INPUT.ADAT_RX_PIN"):
        raise Skip("no ADAT RX on this build")
    _require_usb_source(dev)
    if u8(dev, "INPUT.ADAT_RX_ENABLE"):
        chk.eq(try_set(dev, "INPUT.ADAT_RX_PIN", b"\xff"), "VC_ERR_STATE", "clear while enabled")
        raise Skip("ADAT input is enabled: pin left alone")
    orig = u8(dev, "INPUT.ADAT_RX_PIN")
    try:
        cand = next((p for p in (11, 16, 17, 18, 19) if p != orig and
                     try_set(dev, "INPUT.ADAT_RX_PIN", bytes([p])) == "VC_OK"), None)
        if cand is None:
            chk.note("no free pin accepted for the ADAT input")
            return
        chk.eq(u8(dev, "INPUT.ADAT_RX_PIN"), cand, "new GET after SET")
        old = legacy(dev, "GET_ADAT_INPUT_PIN", 1)
        if old is not None:
            chk.eq(old[0], cand, "GET_ADAT_INPUT_PIN after new SET")
        dev.pset("INPUT.ADAT_RX_PIN", b"\xff")
        chk.eq(u8(dev, "INPUT.ADAT_RX_PIN"), 0xFF, "0xFF clears the pin")
    finally:
        dev.pset("INPUT.ADAT_RX_PIN", bytes([orig]))
        chk.eq(u8(dev, "INPUT.ADAT_RX_PIN"), orig, "ADAT pin restored")


@test("paged_input", mutating=True, flash=3)
def source_name_roundtrip(dev, profile, chk):
    """SOURCE_NAME round-trips through the new and old commands and restores (flash)."""
    need_page(dev, INPUT)
    src = 6
    orig = dev.pget("INPUT.SOURCE_NAME", addr=src, length=32)

    def name():
        dev.wait_ready()
        return dev.pget("INPUT.SOURCE_NAME", addr=src, length=32).split(b"\x00", 1)[0]
    try:
        dev.pset("INPUT.SOURCE_NAME", b"PagedName", addr=src)
        chk.eq(name(), b"PagedName", "new GET after new SET")
        old = legacy(dev, "GET_INPUT_SOURCE_NAME", 32, src)
        if old is not None:
            chk.eq(old.split(b"\x00", 1)[0], b"PagedName", "old GET after new SET")
            dev.raw_set(OP.SET_INPUT_SOURCE_NAME, b"OldName", src)
            chk.eq(name(), b"OldName", "new GET after old SET")
    finally:
        dev.pset("INPUT.SOURCE_NAME", orig.split(b"\x00", 1)[0] or b"\x00", addr=src)
        chk.eq(name(), orig.split(b"\x00", 1)[0], "name restored")
