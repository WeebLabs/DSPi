#!/usr/bin/env python3
"""Prove that this host delivers the wIndex high byte (the page) of a vendor request to DSPi.

Needs DSPi firmware 1.1.7 beta 1 or later and pyusb with a libusb backend. Procedure and pass
criteria: Documentation/Features/windex_page_test.md.
"""

import platform
import struct
import sys

try:
    import usb.backend.libusb1
    import usb.core
except ImportError:
    sys.exit("pyusb is missing. Run: pip install pyusb libusb-package")

VID, PID = 0x2E8B, 0xFEAA
IFACE = 2
SYS_PAGE = 0x01
PROTOCOL_INFO = 0x10
PLATFORM_CMD = 0x20
LAST_RESULT = 0x14
OLD_GET_PLATFORM = 0x7F
# Defined on no page in any build (Documentation/protocol/dspi_commands.json), so the device refuses
# it with VC_ERR_UNKNOWN and records it. Pages 0x55 and up are not allocated at all.
PROBE_CMD = 0xEE
PAGES = (0x02, 0x55, 0x7F, 0xAA, 0xF1, 0xFF)
VC_ERR_UNKNOWN = 0x10
TIMEOUT_MS = 1000


def find_device():
    backend = None
    try:
        # Windows has no system libusb; libusb-package bundles the DLL.
        import libusb_package
        backend = usb.backend.libusb1.get_backend(find_library=libusb_package.find_library)
    except ImportError:
        pass
    dev = usb.core.find(idVendor=VID, idProduct=PID, backend=backend)
    if dev is None:
        sys.exit("No DSPi found (VID 2E8B, PID FEAA).")
    return dev


def windex(page):
    return (page << 8) | IFACE


def is_stall(e):
    return getattr(e, "errno", None) == 32 or "pipe" in str(e).lower()


def read_last_result(dev, bm_in):
    """(page, command, direction, result, wValue) of the last request the device saw on USB."""
    r = bytes(dev.ctrl_transfer(bm_in, LAST_RESULT, 0, windex(SYS_PAGE), 8, TIMEOUT_MS))
    cmd_id, is_get, result, wvalue = struct.unpack_from("<HBBH", r)
    return cmd_id >> 8, cmd_id & 0xFF, "GET" if is_get else "SET", result, wvalue


def refuse_unless_paged(dev):
    """Exit unless the device answers SYS.PROTOCOL_INFO. On 1.1.6 the page byte is ignored and the
    test requests would run as old opcodes, so nothing else is sent until this passes."""
    for bm in (0xC1, 0xC0):
        try:
            r = bytes(dev.ctrl_transfer(bm, PROTOCOL_INFO, 0, windex(SYS_PAGE), 16, TIMEOUT_MS))
        except usb.core.USBError:
            continue
        if r[:4] == b"DSPi":
            return
        sys.exit(f"Unexpected reply to the protocol probe ({r.hex()}). Stopping without testing.")
    try:
        old = bytes(dev.ctrl_transfer(0xC1, OLD_GET_PLATFORM, 0, IFACE, 7, TIMEOUT_MS))
    except usb.core.USBError:
        sys.exit("FAIL: the DSPi did not answer the protocol probe on either recipient.\n"
                 "If it runs firmware 1.1.7 beta 1 or later, this host is changing the page byte. "
                 "Send this output back. Otherwise flash 1.1.7 beta 1 and run the test again.")
    ver = f"{old[1]}.{old[4]}.{old[5]}" if len(old) >= 6 else "1.1.x"
    sys.exit(f"This DSPi runs firmware {ver}, which has no pages. Flash 1.1.7 beta 1 or later and "
             "run the test again. Nothing else was sent.")


def check(dev, recipient, name, wvalue_base):
    """Run every check for one recipient. Returns (page_ok, exact_ok)."""
    bm_in, bm_out = 0xC0 | recipient, 0x40 | recipient
    page_ok = exact_ok = True

    try:
        info = bytes(dev.ctrl_transfer(bm_in, PROTOCOL_INFO, 0, windex(SYS_PAGE), 16, TIMEOUT_MS))
        ok = info[:4] == b"DSPi"
        print(f"  {name:9} GET       page 0x01 PROTOCOL_INFO    reply starts {info[:4]!r:9}  "
              f"{'PASS' if ok else 'FAIL'}")
    except usb.core.USBError as e:
        ok = False
        print(f"  {name:9} GET       page 0x01 PROTOCOL_INFO    ERROR  {e}")
    page_ok &= ok
    try:
        dev.ctrl_transfer(bm_in, PROTOCOL_INFO, 0, windex(0), 16, TIMEOUT_MS)
        ok = False
        print(f"  {name:9} GET       page 0x00 PROTOCOL_INFO    answered, but page 0 must refuse it  FAIL")
    except usb.core.USBError:
        ok = True
        print(f"  {name:9} GET       page 0x00 PROTOCOL_INFO    refused, so the page decides  PASS")
    page_ok &= ok

    n = 0
    for page in PAGES:
        for label, data in (("GET", 4), ("SET len 0", b""), ("SET len 4", b"\x01\x02\x03\x04")):
            n += 1
            wvalue = wvalue_base + n
            bm = bm_in if label == "GET" else bm_out
            sent = f"sent page 0x{page:02X} cmd 0x{PROBE_CMD:02X} wValue 0x{wvalue:04X}"
            note = ""
            try:
                dev.ctrl_transfer(bm, PROBE_CMD, wvalue, windex(page), data, TIMEOUT_MS)
                note = "  (device accepted it, expected a STALL)"
            except usb.core.USBError as e:
                if not is_stall(e):
                    note = f"  (failed without a STALL: {e})"
            try:
                p, c, d, res, wv = read_last_result(dev, bm_in)
            except usb.core.USBError as e:
                print(f"  {name:9} {label:9} {sent}  LAST_RESULT unreadable ({e})  FAIL")
                page_ok = exact_ok = False
                continue
            hi = p == page and wv == wvalue
            ex = hi and c == PROBE_CMD and d == label[:3] and res == VC_ERR_UNKNOWN and not note
            page_ok &= hi
            exact_ok &= ex
            if hi and not ex and not note:
                note = "  (command, direction or result differs)"
            print(f"  {name:9} {label:9} {sent}  device saw page 0x{p:02X} cmd 0x{c:02X} "
                  f"wValue 0x{wv:04X} {d} result 0x{res:02X}  {'PASS' if hi else 'FAIL'}{note}")
    return page_ok, exact_ok


def main():
    dev = find_device()
    print(f"Host: {platform.system()} {platform.release()}  Python {platform.python_version()}")
    print(f"Backend: {type(dev.backend).__module__}")
    refuse_unless_paged(dev)
    try:
        plat = bytes(dev.ctrl_transfer(0xC1, PLATFORM_CMD, 0, windex(SYS_PAGE), 7, TIMEOUT_MS))
        beta = f"-beta{plat[6]}" if len(plat) >= 7 and plat[6] else ""
        print(f"Firmware: {plat[1]}.{plat[4]}.{plat[5]}{beta}  platform {'RP2350' if plat[0] else 'RP2040'}")
    except usb.core.USBError as e:
        print(f"Firmware: SYS.PLATFORM unreadable ({e})")

    print("\nInterface recipient (0xC1 / 0x41), the planned addressing:")
    iface_page, iface_exact = check(dev, 0x01, "interface", 0xA500)
    print("\nDevice recipient (0xC0 / 0x40), the fallback:")
    dev_page, dev_exact = check(dev, 0x00, "device", 0xB500)

    print("\nRESULT")
    print(f"  interface recipient, page byte intact : {'PASS' if iface_page else 'FAIL'}")
    print(f"  interface recipient, all fields exact : {'yes' if iface_exact else 'no'}")
    print(f"  device recipient, page byte intact    : {'PASS' if dev_page else 'FAIL'}")
    print(f"  device recipient, all fields exact    : {'yes' if dev_exact else 'no'}")
    return 0 if iface_page else 1


if __name__ == "__main__":
    sys.exit(main())
