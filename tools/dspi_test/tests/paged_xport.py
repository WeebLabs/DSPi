"""
Native paged tests for the XPORT page (plan section 5.11): GET shapes and refusals.
The UART and I2C control configs are never written with a valid value: a SET re-applies the live
interface and writes flash even when unchanged. Only refusals caught before queueing are exercised.
"""

import struct

from ..framework import test
from .paged_input import BAD_GPIO, MAX_GPIO, need_page, read_len
from .paged_sys import _expect_stall

XPORT = 0x0B


@test("paged_xport")
def xport_shapes(dev, profile, chk):
    """UART_CONFIG, I2C_CONFIG and STATUS have their 8-byte layouts with in-range fields and zero reserved bytes."""
    need_page(dev, XPORT)
    en, tx, rx, ntf, baud = struct.unpack("<BBBBI", read_len(chk, dev, "XPORT.UART_CONFIG", 8))
    chk.member(en, (0, 1), "UART enabled")
    chk.member(ntf, (0, 1), "UART notify")
    chk.in_range(baud, 9600, 1_000_000, "UART baud")
    chk.ok(tx <= MAX_GPIO and rx <= MAX_GPIO, f"UART pins are GPIOs: {tx}/{rx}")
    i2c = read_len(chk, dev, "XPORT.I2C_CONFIG", 8)
    chk.member(i2c[0], (0, 1), "I2C enabled")
    chk.ok(i2c[1] <= MAX_GPIO and i2c[2] <= MAX_GPIO, f"I2C pins are GPIOs: {i2c[1]}/{i2c[2]}")
    chk.in_range(i2c[3], 0x08, 0x77, "I2C target address")
    chk.eq(i2c[4:], bytes(4), "I2C config reserved")
    st = read_len(chk, dev, "XPORT.STATUS", 8)
    chk.eq(st[4], 1, "external protocol version")
    chk.member(st[1], (0, 1), "UART live flag")
    chk.member(st[3], (0, 1), "I2C live flag")
    chk.ok(st[1] <= en and st[3] <= i2c[0], f"a disabled interface is never live: {st.hex()}")
    chk.eq(st[5:], bytes(3), "STATUS reserved")


@test("paged_xport", mutating=True)
def refused_config_writes(dev, profile, chk):
    """Bad scalar fields, bad GPIOs and bad lengths STALL and leave config and STATUS untouched."""
    need_page(dev, XPORT)
    E = _expect_stall
    uart = dev.pget("XPORT.UART_CONFIG", length=8)
    i2c = dev.pget("XPORT.I2C_CONFIG", length=8)
    st = dev.pget("XPORT.STATUS", length=8)

    def uart_with(**kw):
        en, tx, rx, ntf, baud = struct.unpack("<BBBBI", uart)
        f = dict(en=en, tx=tx, rx=rx, ntf=ntf, baud=baud)
        f.update(kw)
        return struct.pack("<BBBBI", f["en"], f["tx"], f["rx"], f["ntf"], f["baud"])

    def i2c_with(**kw):
        f = dict(zip(("en", "sda", "scl", "addr"), i2c[:4]))
        f.update(kw)
        return bytes([f["en"], f["sda"], f["scl"], f["addr"]]) + i2c[4:]

    E(chk, lambda: dev.pset("XPORT.UART_CONFIG", uart_with(en=2)), "VC_ERR_VALUE", "UART enabled 2")
    E(chk, lambda: dev.pset("XPORT.UART_CONFIG", uart_with(ntf=2)), "VC_ERR_VALUE", "UART notify 2")
    E(chk, lambda: dev.pset("XPORT.UART_CONFIG", uart_with(baud=1200)), "VC_ERR_VALUE", "UART baud 1200")
    E(chk, lambda: dev.pset("XPORT.UART_CONFIG", uart_with(baud=2_000_000)), "VC_ERR_VALUE", "UART baud 2M")
    E(chk, lambda: dev.pset("XPORT.UART_CONFIG", uart_with(tx=BAD_GPIO)), "VC_ERR_PIN_INVALID", "UART TX 0xFE")
    E(chk, lambda: dev.pset("XPORT.UART_CONFIG", uart[:7]), "VC_ERR_LENGTH", "7-byte UART config")
    E(chk, lambda: dev.pset("XPORT.UART_CONFIG", uart, addr=1), "VC_ERR_ADDRESS", "UART config at wValue 1")
    E(chk, lambda: dev.pset("XPORT.I2C_CONFIG", i2c_with(en=2)), "VC_ERR_VALUE", "I2C enabled 2")
    E(chk, lambda: dev.pset("XPORT.I2C_CONFIG", i2c_with(addr=0x00)), "VC_ERR_VALUE", "I2C address 0x00")
    E(chk, lambda: dev.pset("XPORT.I2C_CONFIG", i2c_with(addr=0x78)), "VC_ERR_VALUE", "I2C address 0x78")
    E(chk, lambda: dev.pset("XPORT.I2C_CONFIG", i2c_with(scl=BAD_GPIO)), "VC_ERR_PIN_INVALID", "I2C SCL 0xFE")
    E(chk, lambda: dev.pset("XPORT.I2C_CONFIG", i2c + b"\x00"), "VC_ERR_LENGTH", "9-byte I2C config")
    E(chk, lambda: dev.pset("XPORT.STATUS", b"\x00"), "VC_ERR_DIRECTION", "SET to STATUS")
    dev.wait_ready()
    chk.eq(dev.pget("XPORT.UART_CONFIG", length=8), uart, "UART config unchanged")
    chk.eq(dev.pget("XPORT.I2C_CONFIG", length=8), i2c, "I2C config unchanged")
    chk.eq(dev.pget("XPORT.STATUS", length=8), st, "STATUS unchanged")
