"""
Shared paged-protocol helpers for the test modules: assertions, refusals, waits, bands and crosspoints.
Commands are named PAGE.NAME with the address in `addr`. Live DSP params changed here are covered by
the pre-suite bulk snapshot (lifecycle.py); flash and hardware resources are restored by each test.
"""

import json
import math
import struct
import time
from collections import namedtuple

from .device import REGISTRY_REL, PagedStall, ProtocolError, Stall, Timeout, _find_repo_file
from .framework import Skip
from .profile import MAX_GPIO
# Telemetry struct readers, re-exported so a test imports one module.
from .telemetry import (buffer_stats, input_status, levels, pdm_stats, pipeline_stats,  # noqa: F401
                        read_field, read_struct, sample_rate, spdif_tx_stats, system_status,
                        usb_audio_status)
from .lifecycle import push_bulk, read_bulk  # noqa: F401

NAN = struct.pack("<I", 0x7FC00000)     # quiet NaN as a float32 payload
INF = struct.pack("<f", math.inf)
NEG_INF = struct.pack("<f", -math.inf)

BAND_FMT = "<BBHfff"                    # FILTER.BAND: type, bypass, qp_x512, freq, Q, gain dB
ROUTE_FMT = "<BBHf"                     # ROUTE.MATRIX_ROUTE: enabled, phase invert, reserved, gain dB
XOVER_BASE = 20                         # crossover bands are 20..23, output channels only


# --- Gates ------------------------------------------------------------------------------------

def require_paged(dev) -> dict:
    """SYS.PROTOCOL_INFO fields; Skip on firmware without the paged dispatcher."""
    try:
        info = dev.protocol_info()
    except ProtocolError as e:
        raise AssertionError(str(e)) from None
    if info is None:
        raise Skip("no paged dispatcher: SYS.PROTOCOL_INFO STALLs (firmware 1.1.6 or older)")
    return info


def require_page(dev, page: int):
    require_paged(dev)
    if page not in dev.paged_commands():
        raise Skip(f"page 0x{page:02X} not implemented on this firmware")


def has_command(dev, cmd, direction: str | None = None) -> bool:
    """True when this build implements `cmd` (direction "GET", "SET" or None for either)."""
    info = dev.resolve(cmd)
    gets, sets = dev.paged_commands().get(info.page, (frozenset(), frozenset()))
    if direction is None:
        return info.cmd in gets or info.cmd in sets
    return info.cmd in (gets if direction == "GET" else sets)


def registry(dev) -> dict:
    """The generated command registry (Documentation/protocol/dspi_commands.json); Skip until it exists."""
    if isinstance(getattr(dev, "registry", None), dict):
        return dev.registry
    p = _find_repo_file(REGISTRY_REL)
    if p is None:
        raise Skip(f"{REGISTRY_REL} not generated yet")
    return json.loads(p.read_text())


def bits(bitmap: bytes) -> set:
    """Indices of the set bits in a SYS.PAGE_COMMANDS bitmap, bit 0 of byte 0 first."""
    return {i for i in range(len(bitmap) * 8) if bitmap[i // 8] >> (i % 8) & 1}


def page_matches_registry(dev, chk, page, rp2350):
    """PAGE_COMMANDS and COMMAND_INFO of `page` match the registry; build-gated rows exist only on RP2350."""
    pg = next((p for p in registry(dev)["pages"] if p["page"] == page), None)
    if pg is None:
        raise Skip(f"registry has no page 0x{page:02X}")
    bm = dev.pget("SYS.PAGE_COMMANDS", addr=page, length=64)
    got_get, got_set = bits(bm[:32]), bits(bm[32:])
    for c in pg["commands"]:
        name = f"{pg['mnemonic']}.{c['name']}"
        present = not c.get("build") or rp2350
        chk.eq(c["cmd"] in got_get, bool(c["get"]) and present, f"GET bit of {name}")
        chk.eq(c["cmd"] in got_set, bool(c["set"]) and present, f"SET bit of {name}")
        if not present:
            continue
        r = dev.pget("SYS.COMMAND_INFO", addr=(page << 8) | c["cmd"], length=8)
        _flags, get_len, smin, smax, _ak, has = struct.unpack("<HHBBBB", r)
        want = 0xFFFF if c["get_len"] == "var" else (c["get_len"] or 0)
        chk.eq((get_len, smin, smax, has & 3), (want, c["set_min"], c["set_max"],
               (1 if c["get"] else 0) | (2 if c["set"] else 0)), f"{name} COMMAND_INFO")
    chk.eq(sorted((got_get | got_set) - {c["cmd"] for c in pg["commands"]}), [], "unlisted bits")


def read_len(chk, dev, cmd, n, addr=0, label=""):
    """GET `cmd` and check the reply has the table length `n`."""
    r = dev.pget(cmd, addr=addr, length=n)
    chk.eq(len(r), n, f"{cmd}{label} length")
    return r


# --- Refusals ---------------------------------------------------------------------------------

def try_set(dev, cmd, data=b"", addr=0) -> str:
    """pset that returns the VC_* result name: "VC_OK", the refusal, or "STALL" if LAST_RESULT was lost."""
    try:
        dev.pset(cmd, data, addr=addr)
        return "VC_OK"
    except PagedStall as s:
        return s.result_name or "STALL"


def expect_stall(chk, fn, code, msg, retries=3):
    """fn() must STALL with VC code `code`. Retried because LAST_RESULT is shared by all USB hosts."""
    seen = None
    for _ in range(retries):
        try:
            r = fn()
        except PagedStall as s:
            if s.result is not None:
                return chk.eq(s.result_name, code, f"{msg}: LAST_RESULT")
            seen = s
            continue
        return chk.ok(False, f"{msg}: expected STALL with {code}, got {r!r}")
    return chk.ok(False, f"{msg}: STALLed but LAST_RESULT unreadable or overwritten ({seen})")


def expect_result(chk, fn, code, msg, retries=3):
    """As expect_stall, but code "VC_OK" means fn() must succeed. For tables mixing both outcomes."""
    if code == "VC_OK":
        try:
            fn()
            return True
        except PagedStall as s:
            return chk.ok(False, f"{msg}: expected VC_OK, got {s.result_name or 'STALL'}")
    return expect_stall(chk, fn, code, msg, retries)


def nonfinite_refused(chk, get, put, cases, what):
    """Each (payload, label) in `cases` must STALL with VC_ERR_VALUE and leave get() unchanged."""
    before = get()
    try:
        for payload, label in cases:
            expect_stall(chk, lambda: put(payload), "VC_ERR_VALUE", f"{what} {label}")
            chk.eq(get(), before, f"{what} {label} left the value alone")
    finally:
        if get() != before:
            put(before)


_FINITE_RULE = {}   # id(dev) -> (proven, code, readback hex)


def finite_rule(dev, profile, chk):
    """Prove once per run that NaN floats are refused before any test sends more of them.
    Firmware without the finite-value rule stores NaN, so no further non-finite value is sent to it."""
    key = id(dev)
    if key not in _FINITE_RULE:
        o = profile.num_output_channels - 1
        before = dev.pget("ROUTE.OUTPUT_GAIN", o)
        code = "accepted"
        for _ in range(3):
            try:
                dev.pset("ROUTE.OUTPUT_GAIN", NAN, o)
                break
            except PagedStall as s:
                code = s.result_name or "STALL, LAST_RESULT unreadable"
                if s.result is not None:
                    break
        after = dev.pget("ROUTE.OUTPUT_GAIN", o)
        if after != before:
            dev.pset("ROUTE.OUTPUT_GAIN", before, o)
        _FINITE_RULE[key] = (code == "VC_ERR_VALUE" and after == before, code, after.hex())
    proven, code, after = _FINITE_RULE[key]
    chk.ok(proven, f"NaN to ROUTE.OUTPUT_GAIN was not refused ({code}, read back {after}): this firmware lacks "
                   "the finite-value rule, so no further non-finite values are sent in this run")
    return proven


# --- Waits and retries ------------------------------------------------------------------------

def wait_for(fn, want, timeout_s=2.0, poll_s=0.04, tolerate=(Stall, Timeout)):
    """Poll fn() until it equals `want` (or want(value) is true when callable); return the last value.
    Reads raising `tolerate` (a deferred apply's busy window) keep polling; None if none ever answered."""
    ok = want if callable(want) else (lambda v: v == want)
    deadline = time.monotonic() + timeout_s
    last = None
    while True:
        try:
            last = fn()
            if ok(last):
                return last
        except tolerate:
            pass
        if time.monotonic() >= deadline:
            return last
        time.sleep(poll_s)


def reached(fn, want, timeout_s=2.0, poll_s=0.04, tolerate=(Stall, Timeout)) -> bool:
    """wait_for as a yes or no: True once fn() reads `want` within the timeout."""
    return wait_for(fn, want, timeout_s, poll_s, tolerate) == want


def settle(dev, extra_s=0.1):
    """Wait out a deferred restart, then a short margin for the main loop to finish."""
    dev.wait_ready()
    time.sleep(extra_s)


def set_retry_busy(dev, cmd, data=b"", addr=0, retries=50, delay_s=0.01):
    """pset, resent while the device answers VC_ERR_BUSY (one pending slot, e.g. FILTER.BAND)."""
    for _ in range(retries):
        try:
            return dev.pset(cmd, data, addr=addr)
        except PagedStall as s:
            if s.result_name != "VC_ERR_BUSY":
                raise
            time.sleep(delay_s)
    raise AssertionError(f"{cmd} stayed VC_ERR_BUSY after {retries} tries")


def set_and_wait(dev, cmd, data, addr=0, *, want=None, timeout_s=2.0):
    """pset a deferred command, then poll its GET until it reads `want` (default: data). Returns the last read."""
    want = bytes(data if want is None else want)
    dev.pset(cmd, data, addr=addr)
    return wait_for(lambda: dev.pget(cmd, addr=addr, length=len(want)), want, timeout_s)


# --- FILTER.BAND ------------------------------------------------------------------------------

class Band(namedtuple("Band", "type bypass qp_x512 freq q gain_db")):
    __slots__ = ()

    def pack(self) -> bytes:
        return struct.pack(BAND_FMT, *self)


def band_addr(ch: int, band: int) -> int:
    return (ch << 8) | band


def pack_band(ftype, freq, q, gain_db, *, bypass=0, qp_x512=0) -> bytes:
    return struct.pack(BAND_FMT, ftype, bypass, qp_x512, freq, q, gain_db)


def band_readback(payload: bytes, band: int) -> bytes:
    """What a FILTER.BAND GET returns after `payload` applied: bypass is 0 or 1, a crossover band reads qp 0."""
    t, byp, qp, f, q, g = struct.unpack(BAND_FMT, payload)
    return struct.pack(BAND_FMT, t, 1 if byp == 1 else 0, 0 if band >= XOVER_BASE else qp, f, q, g)


def read_band(dev, ch, band) -> Band:
    return Band(*struct.unpack(BAND_FMT, dev.pget("FILTER.BAND", band_addr(ch, band), 16)))


def wait_band(dev, ch, band, want: bytes, timeout_s=0.5, tolerate=(Stall, Timeout)) -> bytes:
    """Poll FILTER.BAND until it reads `want` (the GET shows a write only once applied); last read."""
    return wait_for(lambda: dev.pget("FILTER.BAND", band_addr(ch, band), 16), bytes(want), timeout_s, 0.01,
                    tolerate)


def write_band(dev, ch, band, ftype, freq, q, gain_db, *, bypass=0, qp_x512=0, wait=False) -> bytes:
    """Write one band (VC_ERR_BUSY retried). wait=True also blocks until it reads back. Returns the payload."""
    payload = pack_band(ftype, freq, q, gain_db, bypass=bypass, qp_x512=qp_x512)
    set_retry_busy(dev, "FILTER.BAND", payload, band_addr(ch, band))
    if wait:
        wait_band(dev, ch, band, band_readback(payload, band))
    return payload


# --- ROUTE.MATRIX_ROUTE -----------------------------------------------------------------------

Route = namedtuple("Route", "enabled phase_invert gain_db")


def route_addr(inp: int, out: int) -> int:
    return (inp << 8) | out


def pack_route(enabled, gain_db=0.0, phase_invert=0) -> bytes:
    return struct.pack(ROUTE_FMT, enabled, phase_invert, 0, gain_db)


def read_route(dev, inp, out) -> Route:
    en, ph, _resv, g = struct.unpack(ROUTE_FMT, dev.pget("ROUTE.MATRIX_ROUTE", route_addr(inp, out), 8))
    return Route(en, ph, g)


def write_route(dev, inp, out, enabled, gain_db=0.0, phase_invert=0):
    return dev.pset("ROUTE.MATRIX_ROUTE", pack_route(enabled, gain_db, phase_invert), route_addr(inp, out))


# --- GPIOs ------------------------------------------------------------------------------------

BAD_GPIO = 0xFE                         # never a valid GPIO; 0xFF is the reset sentinel
HIGHEST_GPIO = max(MAX_GPIO.values())   # highest GPIO on any platform


def invalid_gpios(profile):
    """GPIOs every pin SET must refuse without a platform pin map: power and LED, and one past the last GPIO.
    Mirrors is_valid_gpio_pin() in firmware/DSPi/vendor/vc_pins.c."""
    return (23, 24, 25, MAX_GPIO[profile.platform_id] + 1)


def busy_gpios(dev, profile) -> set:
    """GPIOs owned by an output, input or clock function right now, plus those no pin SET accepts."""
    pins = {dev.pget_u8("OUTPUT.PIN", i) for i in range(profile.num_pin_outputs)}
    pins.add(dev.pget_u8("INPUT.I2S_RX_PIN"))
    pins.add(dev.pget_u8("INPUT.SPDIF_RX_PIN"))
    bck = dev.pget_u8("CLOCK.I2S_BCK_PIN")
    pins.update((bck, bck + 1))
    if dev.pget_u8("CLOCK.MCK_ENABLE"):
        pins.add(dev.pget_u8("CLOCK.MCK_PIN"))
    pins.update(invalid_gpios(profile))
    return pins


def free_gpio(dev, profile, prefer=(16, 17, 18, 19, 20, 21, 22, 26, 27)):
    """First GPIO in `prefer` no function owns, or None."""
    busy = busy_gpios(dev, profile)
    return next((p for p in prefer if p not in busy and p <= 29), None)


def gpio_owners(dev, profile) -> dict:
    """GPIO -> functions holding it now, from every readable config. Control Surfaces bindings are not
    readable here, so the firmware's own claim check stays the last word."""
    u8 = dev.pget_u8
    own = {}

    def add(pin, who):
        if pin != 0xFF:
            own.setdefault(pin, set()).add(who)
    for i in range(profile.num_pin_outputs):
        add(u8("OUTPUT.PIN", i), f"output{i}")
    cfg = dev.pget("INPUT.SPDIF_RX_CONFIG", length=6)
    for i in range(cfg[0]):
        if (cfg[1] >> i) & 1:
            add(cfg[2 + i], f"spdif_rx{i}")
    pairs = 4 if profile.platform_id == 1 else 1
    for p in range(min(pairs, u8("INPUT.I2S_RX_CHANNELS") // 2)):
        add(u8("INPUT.I2S_RX_PIN", p), f"i2s_rx{p}")
    roles = (0, 1) if u8("CLOCK.I2S_CLOCK_PIN_MODE") else (0,)
    for r in roles:
        bck = u8("CLOCK.I2S_BCK_PIN", r)
        add(bck, f"i2s_bck{r}")
        add(bck + 1, f"i2s_bck{r}")
    if u8("CLOCK.MCK_ENABLE"):
        add(u8("CLOCK.MCK_PIN"), "mck")
    dac = dev.pget("OUTPUT.DAC_MUTE_CONFIG", length=16)
    if dac[0]:
        add(dac[2], "dac_mute")
    xst = dev.pget("XPORT.STATUS", length=8)
    if xst[1]:
        uart = dev.pget("XPORT.UART_CONFIG", length=8)
        add(uart[1], "uart")
        add(uart[2], "uart")
    if xst[3]:
        i2c = dev.pget("XPORT.I2C_CONFIG", length=8)
        add(i2c[1], "i2c")
        add(i2c[2], "i2c")
    if has_command(dev, "OUTPUT.ADAT_TX_ENABLE") and u8("OUTPUT.ADAT_TX_ENABLE"):
        add(u8("OUTPUT.ADAT_TX_PIN"), "adat_tx")
    if has_command(dev, "INPUT.ADAT_RX_ENABLE") and u8("INPUT.ADAT_RX_ENABLE"):
        add(u8("INPUT.ADAT_RX_PIN"), "adat_rx")
    return own


def held_by_others(owners, pins, me) -> set:
    """Functions other than `me` holding any of `pins`: a non-empty set means a move could not be undone."""
    return set().union(*(owners.get(p, set()) for p in pins)) - {me}


# --- OUTPUT.TYPE ------------------------------------------------------------------------------

TYPE_SPDIF, TYPE_I2S = 0, 1


def poll_type(dev, slot, want, timeout_s=2.0) -> bool:
    return reached(lambda: dev.pget_u8("OUTPUT.TYPE", slot), want, timeout_s)


def set_type(dev, slot, t, timeout_s=2.0) -> bool:
    """Queue a slot type switch and wait for it; a timeout in the restart window is retried. True once applied."""
    for _ in range(3):
        try:
            dev.pset_u8("OUTPUT.TYPE", t, slot)
        except Timeout:
            dev.wait_ready()
            continue
        if poll_type(dev, slot, t, timeout_s):
            return True
    return poll_type(dev, slot, t, timeout_s)


# --- Scalar assertions ------------------------------------------------------------------------

def float_roundtrip(dev, chk, cmd, value, *, addr=0, tol=1e-3, label=""):
    """SET a float, GET it back, assert equality within tol."""
    dev.pset_f32(cmd, value, addr)
    got = dev.pget_f32(cmd, addr)
    chk.approx(got, value, tol, label or f"roundtrip {value}")
    return got


def float_clamp(dev, chk, cmd, value, expected, *, addr=0, tol=1e-3, label=""):
    """SET an out-of-range finite float, assert it reads back clamped to `expected`."""
    dev.pset_f32(cmd, value, addr)
    got = dev.pget_f32(cmd, addr)
    chk.approx(got, expected, tol, label or f"clamp {value}->{expected}")
    return got


def bool_roundtrip(dev, chk, cmd, *, addr=0, label="", truthy_byte=0xFF):
    """SET 1, 0, then `truthy_byte`; assert the GET normalises to 1, 0, 1."""
    dev.pset_u8(cmd, 1, addr)
    chk.eq(dev.pget_u8(cmd, addr), 1, f"{label} set 1")
    dev.pset_u8(cmd, 0, addr)
    chk.eq(dev.pget_u8(cmd, addr), 0, f"{label} set 0")
    dev.pset_u8(cmd, truthy_byte, addr)
    chk.eq(dev.pget_u8(cmd, addr), 1, f"{label} set 0x{truthy_byte:02X}->1")


def nan_rejected(dev, chk, cmd, *, addr=0, label="", code="VC_ERR_VALUE"):
    """After a known value, a NaN SET must STALL with `code` and change nothing.
    code=None also accepts an ACK that ignores the NaN."""
    dev.pset_f32(cmd, -7.0, addr)
    before = dev.pget_f32(cmd, addr)
    if code is None:
        try_set(dev, cmd, NAN, addr)
    else:
        expect_stall(chk, lambda: dev.pset(cmd, NAN, addr), code, f"{label} NaN")
    chk.approx(dev.pget_f32(cmd, addr), before, 1e-3, f"{label} NaN rejected (unchanged)")


def wrong_length_refused(dev, chk, cmd, *, addr=0, lengths=None, label=""):
    """SETs of each length in `lengths` (default: one short and one long of the GET length) must STALL
    with VC_ERR_LENGTH and leave the value unchanged."""
    before = dev.pget(cmd, addr)
    if lengths is None:
        lengths = (len(before) - 1, len(before) + 1)
    for n in lengths:
        expect_stall(chk, lambda: dev.pset(cmd, bytes(n), addr), "VC_ERR_LENGTH", f"{label or cmd} {n}-byte SET")
    chk.eq(dev.pget(cmd, addr), before, f"{label or cmd} unchanged after wrong-length SETs")
