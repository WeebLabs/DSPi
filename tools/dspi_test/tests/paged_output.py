"""
Native paged tests for the OUTPUT page (plan section 5.4), A/B against the 1.1.6 opcodes on page 0.

Output types, output pins, the ADAT output and the DAC mute config are never changed: only GET
parity, SETs of the current value and refused writes. DAC_MUTE_TEST is never sent zero-length.
"""

import struct

from ..framework import test
from .paged_input import BAD_GPIO, has_cmd, legacy, need_page, parity, try_set, u8
from .paged_sys import _expect_stall

OUTPUT = 0x04


@test("paged_output")
def type_pin_parity(dev, profile, chk):
    """TYPE and PIN match GET_OUTPUT_TYPE / GET_OUTPUT_PIN for every slot and pin output."""
    need_page(dev, OUTPUT)
    for s in range(profile.num_spdif):
        parity(chk, dev, "OUTPUT.TYPE", "GET_OUTPUT_TYPE", 1, addr=s, label=f"[{s}]")
    for p in range(profile.num_pin_outputs):
        parity(chk, dev, "OUTPUT.PIN", "GET_OUTPUT_PIN", 1, addr=p, label=f"[{p}]")


@test("paged_output")
def latency_split_parity(dev, profile, chk):
    """OUTPUT_LATENCY wValue 0 and 1 reach LATENCY_STATUS and LATENCY_OPTIONS; LATENCY is the request."""
    need_page(dev, OUTPUT)
    st = parity(chk, dev, "OUTPUT.LATENCY_STATUS", "OUTPUT_LATENCY", 12, wvalue=0)
    opts = dev.pget("OUTPUT.LATENCY_OPTIONS", length=256)
    chk.eq(len(opts), 8 * st[9], "LATENCY_OPTIONS is 8 bytes per option")
    old = legacy(dev, "OUTPUT_LATENCY", 256, 1)
    if old is not None:
        chk.eq(opts, old, "LATENCY_OPTIONS == OUTPUT_LATENCY wValue 1")
    if not st[8]:
        chk.eq(u8(dev, "OUTPUT.LATENCY"), st[0], "LATENCY == requested_id with nothing pending")


@test("paged_output")
def adat_dac_parity(dev, profile, chk):
    """ADAT TX and DAC mute GETs match their 1.1.6 twins."""
    need_page(dev, OUTPUT)
    parity(chk, dev, "OUTPUT.DAC_MUTE_CONFIG", "GET_DAC_HW_MUTE_CONFIG", 16)
    if not has_cmd(dev, "OUTPUT.ADAT_TX_ENABLE"):
        chk.note("no ADAT TX commands on this build")
        return
    parity(chk, dev, "OUTPUT.ADAT_TX_ENABLE", "GET_ADAT_ENABLE", 1)
    parity(chk, dev, "OUTPUT.ADAT_TX_PIN", "GET_ADAT_PIN", 1)
    parity(chk, dev, "OUTPUT.ADAT_TX_STATUS", "GET_ADAT_STATUS", 8)


@test("paged_output", mutating=True)
def noop_sets_of_current_values(dev, profile, chk):
    """SET of the current type, pin, latency and ADAT config is accepted and changes nothing."""
    need_page(dev, OUTPUT)
    types = [u8(dev, "OUTPUT.TYPE", s) for s in range(profile.num_spdif)]
    pins = [u8(dev, "OUTPUT.PIN", p) for p in range(profile.num_pin_outputs)]
    for s, t in enumerate(types):
        chk.eq(try_set(dev, "OUTPUT.TYPE", bytes([t]), s), "VC_OK", f"TYPE[{s}] = current")
    for p, v in enumerate(pins):
        chk.eq(try_set(dev, "OUTPUT.PIN", bytes([v]), p), "VC_OK", f"PIN[{p}] = current")
    st = dev.pget("OUTPUT.LATENCY_STATUS", length=12)
    if not st[8]:
        chk.eq(try_set(dev, "OUTPUT.LATENCY", bytes([st[0]])), "VC_OK", "LATENCY = current")
        chk.eq(dev.pget("OUTPUT.LATENCY_STATUS", length=12)[8], 0, "no latency change queued")
    if has_cmd(dev, "OUTPUT.ADAT_TX_ENABLE"):
        for n in ("ADAT_TX_ENABLE", "ADAT_TX_PIN"):
            v = u8(dev, f"OUTPUT.{n}")
            chk.eq(try_set(dev, f"OUTPUT.{n}", bytes([v])), "VC_OK", f"{n} = current")
    chk.eq([u8(dev, "OUTPUT.TYPE", s) for s in range(profile.num_spdif)], types, "types unchanged")
    chk.eq([u8(dev, "OUTPUT.PIN", p) for p in range(profile.num_pin_outputs)], pins, "pins unchanged")


@test("paged_output", mutating=True)
def refused_writes(dev, profile, chk):
    """Bad address, length, value and direction STALL with the matching code and change nothing."""
    need_page(dev, OUTPUT)
    E = _expect_stall
    types = [u8(dev, "OUTPUT.TYPE", s) for s in range(profile.num_spdif)]
    pins = [u8(dev, "OUTPUT.PIN", p) for p in range(profile.num_pin_outputs)]
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
    if has_cmd(dev, "OUTPUT.ADAT_TX_ENABLE"):
        E(chk, lambda: dev.pset("OUTPUT.ADAT_TX_ENABLE", b"\x02"), "VC_ERR_VALUE", "ADAT TX enable 2")
        E(chk, lambda: dev.pset("OUTPUT.ADAT_TX_PIN", bytes([BAD_GPIO])), "VC_ERR_PIN_INVALID", "ADAT TX pin 0xFE")
        E(chk, lambda: dev.pset("OUTPUT.ADAT_TX_STATUS", b"\x00"), "VC_ERR_DIRECTION", "SET to ADAT_TX_STATUS")
    dev.wait_ready()
    chk.eq([u8(dev, "OUTPUT.TYPE", s) for s in range(profile.num_spdif)], types, "types unchanged")
    chk.eq([u8(dev, "OUTPUT.PIN", p) for p in range(profile.num_pin_outputs)], pins, "pins unchanged")
    chk.eq(dev.pget("OUTPUT.DAC_MUTE_CONFIG", length=16), dac, "DAC mute config unchanged")
    chk.eq(dev.pget("OUTPUT.LATENCY_STATUS", length=12)[:2], lat[:2], "latency request unchanged")

