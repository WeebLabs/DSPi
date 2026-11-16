"""
Native paged tests for the OUTPUT page (plan section 5.4). Output types, output pins, the ADAT output and
the DAC mute config are never changed: only GET shapes and cross-checks, SETs of the current value and
refused writes. DAC_MUTE_TEST is never sent zero-length.
"""

import struct

from ..framework import test
from ..helpers import BAD_GPIO, HIGHEST_GPIO, expect_stall, has_command, read_len, require_page, try_set

OUTPUT = 0x04


@test("paged_output")
def type_pin_shapes(dev, profile, chk):
    """TYPE is S/PDIF or I2S for every slot and PIN is a distinct GPIO for every pin output."""
    require_page(dev, OUTPUT)
    for s in range(profile.num_spdif):
        chk.member(read_len(chk, dev, "OUTPUT.TYPE", 1, addr=s, label=f"[{s}]")[0], (0, 1), f"slot {s} type")
    pins = [read_len(chk, dev, "OUTPUT.PIN", 1, addr=p, label=f"[{p}]")[0] for p in range(profile.num_pin_outputs)]
    chk.ok(all(p <= HIGHEST_GPIO for p in pins), f"output pins are GPIOs: {pins}")
    chk.eq(len(set(pins)), len(pins), f"output pins are distinct: {pins}")


@test("paged_output")
def latency_status_and_options(dev, profile, chk):
    """LATENCY_STATUS is 12 bytes with valid IDs; LATENCY_OPTIONS lists 1-7 once each; LATENCY is the request."""
    require_page(dev, OUTPUT)
    st = read_len(chk, dev, "OUTPUT.LATENCY_STATUS", 12)
    req, eff, _eff_us, _target, _batch, pending, count, rsv = struct.unpack("<BBHHHBBH", st)
    chk.in_range(req, 1, 7, "requested option ID")
    chk.in_range(eff, 1, 7, "effective option ID")
    chk.member(pending, (0, 1), "pending flag")
    chk.eq(rsv, 0, "LATENCY_STATUS reserved")
    opts = dev.pget("OUTPUT.LATENCY_OPTIONS", length=256)
    chk.eq(len(opts), 8 * count, "LATENCY_OPTIONS is 8 bytes per option")
    ids = [opts[i] for i in range(0, len(opts), 8)]
    chk.eq(sorted(ids), list(range(1, count + 1)), "option IDs are 1..option_count once each")
    chk.ok(req in ids and eff in ids, f"requested {req} and effective {eff} are listed options")
    for i in range(0, len(opts), 8):
        oid, _flags, lat_us, _batch_us, _eff_us = struct.unpack_from("<BBHHH", opts, i)
        chk.ok(lat_us > 0, f"option {oid} latency_us is set")
    if not pending:
        chk.eq(dev.pget_u8("OUTPUT.LATENCY"), req, "LATENCY == requested_id with nothing pending")


@test("paged_output")
def adat_dac_shapes(dev, profile, chk):
    """DAC mute config and the ADAT TX status have their sizes, zero reserved bytes and echo their config."""
    require_page(dev, OUTPUT)
    dac = read_len(chk, dev, "OUTPUT.DAC_MUTE_CONFIG", 16)
    chk.member(dac[0], (0, 1), "DAC mute enabled")
    chk.member(dac[1], (0, 1), "DAC mute active_low")
    chk.eq(dac[3], 0, "DAC mute reserved0")
    chk.eq(dac[8:], bytes(8), "DAC mute reserved")
    if not has_command(dev, "OUTPUT.ADAT_TX_ENABLE"):
        chk.note("no ADAT TX commands on this build")
        return
    en = read_len(chk, dev, "OUTPUT.ADAT_TX_ENABLE", 1)[0]
    pin = read_len(chk, dev, "OUTPUT.ADAT_TX_PIN", 1)[0]
    st = read_len(chk, dev, "OUTPUT.ADAT_TX_STATUS", 8)
    chk.eq((st[0], st[2]), (en, pin), "ADAT_TX_STATUS enabled and pin == ADAT_TX_ENABLE and ADAT_TX_PIN")
    chk.member(st[1], (0, 1), "ADAT TX active flag")
    chk.member(st[3], (0, 1), "ADAT TX rate_ok flag")
    if not en:
        chk.eq(st[1], 0, "ADAT TX not running while disabled")


@test("paged_output", mutating=True)
def noop_sets_of_current_values(dev, profile, chk):
    """SET of the current type, pin, latency and ADAT config is accepted and changes nothing."""
    require_page(dev, OUTPUT)
    types = [dev.pget_u8("OUTPUT.TYPE", s) for s in range(profile.num_spdif)]
    pins = [dev.pget_u8("OUTPUT.PIN", p) for p in range(profile.num_pin_outputs)]
    for s, t in enumerate(types):
        chk.eq(try_set(dev, "OUTPUT.TYPE", bytes([t]), s), "VC_OK", f"TYPE[{s}] = current")
    for p, v in enumerate(pins):
        chk.eq(try_set(dev, "OUTPUT.PIN", bytes([v]), p), "VC_OK", f"PIN[{p}] = current")
    st = dev.pget("OUTPUT.LATENCY_STATUS", length=12)
    if not st[8]:
        chk.eq(try_set(dev, "OUTPUT.LATENCY", bytes([st[0]])), "VC_OK", "LATENCY = current")
        chk.eq(dev.pget("OUTPUT.LATENCY_STATUS", length=12)[8], 0, "no latency change queued")
    if has_command(dev, "OUTPUT.ADAT_TX_ENABLE"):
        for n in ("ADAT_TX_ENABLE", "ADAT_TX_PIN"):
            v = dev.pget_u8(f"OUTPUT.{n}")
            chk.eq(try_set(dev, f"OUTPUT.{n}", bytes([v])), "VC_OK", f"{n} = current")
    chk.eq([dev.pget_u8("OUTPUT.TYPE", s) for s in range(profile.num_spdif)], types, "types unchanged")
    chk.eq([dev.pget_u8("OUTPUT.PIN", p) for p in range(profile.num_pin_outputs)], pins, "pins unchanged")


@test("paged_output", mutating=True)
def refused_writes(dev, profile, chk):
    """Bad address, length, value and direction STALL with the matching code and change nothing."""
    require_page(dev, OUTPUT)
    E = expect_stall
    types = [dev.pget_u8("OUTPUT.TYPE", s) for s in range(profile.num_spdif)]
    pins = [dev.pget_u8("OUTPUT.PIN", p) for p in range(profile.num_pin_outputs)]
    dac = dev.pget("OUTPUT.DAC_MUTE_CONFIG", length=16)
    lat = dev.pget("OUTPUT.LATENCY_STATUS", length=12)
    ns, npin = profile.num_spdif, profile.num_pin_outputs
    E(chk, lambda: dev.pget("OUTPUT.TYPE", addr=ns, length=1), "VC_ERR_ADDRESS", f"TYPE slot {ns}")
    E(chk, lambda: dev.pset("OUTPUT.TYPE", bytes([types[0]]), addr=ns), "VC_ERR_ADDRESS", f"SET TYPE slot {ns}")
    E(chk, lambda: dev.pset("OUTPUT.TYPE", b"\x02", addr=0), "VC_ERR_VALUE", "type 2")
    E(chk, lambda: dev.pset("OUTPUT.TYPE", b"\x00\x00", addr=0), "VC_ERR_LENGTH", "2-byte TYPE")
    E(chk, lambda: dev.pget("OUTPUT.PIN", addr=npin, length=1), "VC_ERR_ADDRESS", f"PIN output {npin}")
    E(chk, lambda: dev.pset("OUTPUT.PIN", bytes([BAD_GPIO]), addr=0), "VC_ERR_PIN_INVALID", "pin 0xFE")
    if pins[0] != pins[1]:
        E(chk, lambda: dev.pset("OUTPUT.PIN", bytes([pins[1]]), addr=0), "VC_ERR_PIN_IN_USE",
          f"output 0 onto output 1's pin {pins[1]}")
    E(chk, lambda: dev.pset("OUTPUT.LATENCY", b"\x00"), "VC_ERR_VALUE", "latency option 0")
    E(chk, lambda: dev.pset("OUTPUT.LATENCY", b"\x63"), "VC_ERR_VALUE", "latency option 99")
    E(chk, lambda: dev.pset("OUTPUT.LATENCY", b"\x05\x00"), "VC_ERR_LENGTH", "2-byte LATENCY")
    for ro in ("LATENCY_STATUS", "LATENCY_OPTIONS"):
        E(chk, lambda: dev.pset(f"OUTPUT.{ro}", b"\x00"), "VC_ERR_DIRECTION", f"SET to {ro}")
        E(chk, lambda: dev.pget(f"OUTPUT.{ro}", addr=1, length=12), "VC_ERR_ADDRESS", f"{ro} at wValue 1")
    bad = bytearray(dac)
    bad[0], bad[1] = 1, 2
    E(chk, lambda: dev.pset("OUTPUT.DAC_MUTE_CONFIG", bytes(bad)), "VC_ERR_VALUE", "DAC mute active_low 2")
    bad = bytearray(dac)
    bad[0], bad[1] = 1, 1
    struct.pack_into("<H", bad, 4, 0)
    E(chk, lambda: dev.pset("OUTPUT.DAC_MUTE_CONFIG", bytes(bad)), "VC_ERR_VALUE", "DAC mute hold 0 ms")
    bad = bytearray(dac)
    bad[0], bad[1] = 1, 1
    struct.pack_into("<HH", bad, 4, 10, 501)
    E(chk, lambda: dev.pset("OUTPUT.DAC_MUTE_CONFIG", bytes(bad)), "VC_ERR_VALUE", "DAC mute release 501 ms")
    E(chk, lambda: dev.pset("OUTPUT.DAC_MUTE_CONFIG", dac[:15]), "VC_ERR_LENGTH", "15-byte DAC mute config")
    E(chk, lambda: dev.pget("OUTPUT.DAC_MUTE_TEST", length=1), "VC_ERR_DIRECTION", "GET DAC_MUTE_TEST")
    E(chk, lambda: dev.pset("OUTPUT.DAC_MUTE_TEST", b"\x00"), "VC_ERR_LENGTH", "DAC_MUTE_TEST with a payload")
    if has_command(dev, "OUTPUT.ADAT_TX_ENABLE"):
        E(chk, lambda: dev.pset("OUTPUT.ADAT_TX_ENABLE", b"\x02"), "VC_ERR_VALUE", "ADAT TX enable 2")
        E(chk, lambda: dev.pset("OUTPUT.ADAT_TX_PIN", bytes([BAD_GPIO])), "VC_ERR_PIN_INVALID", "ADAT TX pin 0xFE")
        E(chk, lambda: dev.pset("OUTPUT.ADAT_TX_STATUS", b"\x00"), "VC_ERR_DIRECTION", "SET to ADAT_TX_STATUS")
    dev.wait_ready()
    chk.eq([dev.pget_u8("OUTPUT.TYPE", s) for s in range(profile.num_spdif)], types, "types unchanged")
    chk.eq([dev.pget_u8("OUTPUT.PIN", p) for p in range(profile.num_pin_outputs)], pins, "pins unchanged")
    chk.eq(dev.pget("OUTPUT.DAC_MUTE_CONFIG", length=16), dac, "DAC mute config unchanged")
    chk.eq(dev.pget("OUTPUT.LATENCY_STATUS", length=12)[:2], lat[:2], "latency request unchanged")

