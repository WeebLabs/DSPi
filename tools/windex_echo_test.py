#!/usr/bin/env python3
"""Prove that this host delivers the wIndex high byte of a vendor request.

Needs a DSPi running a -DDSPI_WINDEX_ECHO=ON bench build and pyusb with a
libusb backend. Procedure and pass criteria: Documentation/Features/windex_echo_test.md.
"""

import platform
import sys

try:
    import usb.backend.libusb1
    import usb.core
except ImportError:
    sys.exit("pyusb is missing. Run: pip install pyusb libusb-package")

VID, PID = 0x2E8B, 0xFEAA
ECHO_REQ = 0x4F
IFACE = 2
PAGES = (0x01, 0x7F, 0xF0, 0xFF)
WVALUE = 0xA55A


def find_device():
    backend = None
    try:
        # Windows has no system libusb; libusb-package bundles the DLL.
        import libusb_package
        backend = usb.backend.libusb1.get_backend(
            find_library=libusb_package.find_library)
    except ImportError:
        pass
    dev = usb.core.find(idVendor=VID, idProduct=PID, backend=backend)
    if dev is None:
        sys.exit("No DSPi found (VID 2E8B, PID FEAA).")
    return dev


def fields(b):
    return {
        "bm": b[0], "req": b[1],
        "wValue": b[2] | (b[3] << 8),
        "wIndex": b[4] | (b[5] << 8),
        "wLength": b[6] | (b[7] << 8),
    }


def read_echo(dev, bm_in, windex):
    return bytes(dev.ctrl_transfer(bm_in, ECHO_REQ, WVALUE, windex, 16, 1000))


def check(dev, recipient, name):
    """Run the IN and OUT checks for one recipient. Returns (high_ok, exact_ok)."""
    bm_in, bm_out = 0xC0 | recipient, 0x40 | recipient
    high_ok = exact_ok = True
    for page in PAGES:
        windex = (page << 8) | IFACE
        rows = []
        try:
            rows.append(("IN", fields(read_echo(dev, bm_in, windex)[:8])))
            for label, payload in (("OUT len 0", b""), ("OUT len 4", b"\x01\x02\x03\x04")):
                dev.ctrl_transfer(bm_out, ECHO_REQ, WVALUE, windex, payload, 1000)
                # Read back through a different page so a stale echo cannot pass.
                rows.append((label, fields(read_echo(dev, bm_in, IFACE)[8:])))
        except usb.core.USBError as e:
            print(f"  {name:9} wIndex 0x{windex:04X}  ERROR  {e}")
            high_ok = exact_ok = False
            continue
        for label, got in rows:
            hi = (got["wIndex"] >> 8) == page
            ex = got["wIndex"] == windex and got["wValue"] == WVALUE
            high_ok &= hi
            exact_ok &= ex
            verdict = "PASS" if hi else "FAIL"
            note = "" if ex else "  (low byte or wValue altered)"
            print(f"  {name:9} {label:9} sent wIndex 0x{windex:04X}  "
                  f"device saw 0x{got['wIndex']:04X}  wValue 0x{got['wValue']:04X}  "
                  f"{verdict}{note}")
    return high_ok, exact_ok


def main():
    dev = find_device()
    print(f"Host: {platform.system()} {platform.release()}  Python {platform.python_version()}")
    print(f"Backend: {type(dev.backend).__module__}")
    try:
        probe = read_echo(dev, 0xC1, IFACE)
    except usb.core.USBError as e:
        sys.exit(f"Echo request failed ({e}). Is the DSPI_WINDEX_ECHO bench build flashed, "
                 "and is interface 2 bound to WinUSB?")
    if len(probe) != 16 or probe[1] != ECHO_REQ:
        sys.exit(f"Unexpected echo reply {probe.hex()}. Wrong firmware?")

    print("\nInterface recipient (0xC1 / 0x41), the planned addressing:")
    iface_high, iface_exact = check(dev, 0x01, "interface")
    print("\nDevice recipient (0xC0 / 0x40), the fallback:")
    dev_high, dev_exact = check(dev, 0x00, "device")

    print("\nRESULT")
    print(f"  interface recipient, high byte intact : {'PASS' if iface_high else 'FAIL'}")
    print(f"  interface recipient, all fields exact : {'yes' if iface_exact else 'no'}")
    print(f"  device recipient, high byte intact    : {'PASS' if dev_high else 'FAIL'}")
    print(f"  device recipient, all fields exact    : {'yes' if dev_exact else 'no'}")
    return 0 if iface_high else 1


if __name__ == "__main__":
    sys.exit(main())
