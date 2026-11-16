"""
Page 0 is empty from firmware 1.1.7 (plan 7.3, 8.11): every vendor request with a zero wIndex high byte
STALLs and is recorded as VC_ERR_UNKNOWN, while the Microsoft OS 2.0 descriptor request is still served.
The walk sends raw requests with no retries, so a STALL costs one SETUP and the whole walk is quick.
"""

import usb.core

from ..device import ProtocolError, Stall
from ..framework import Skip, test

BM_IN_IFACE, BM_OUT_IFACE, BM_IN_DEV, BM_OUT_DEV = 0xC1, 0x41, 0xC0, 0x40
MS_OS_20 = (BM_IN_DEV, 0x01, 0x0007)       # bmRequestType, bRequest, wIndex
MS_OS_20_LEN = 0xB2
OLD_GET_PLATFORM = 0x7F
WALK_TIMEOUT_MS = 500


def _guard(dev):
    """Raise Skip unless the firmware provably has no page 0.
    On 1.1.6 a page 0 walk runs every old opcode, FACTORY_RESET and ENTER_BOOTLOADER included,
    so all three checks must pass before the first walk request is sent."""
    try:
        info = dev.protocol_info()
    except ProtocolError as e:
        raise Skip(f"refusing the page 0 walk: {e}") from None
    if info is None:
        raise Skip("refusing the page 0 walk: SYS.PROTOCOL_INFO STALLs, so this is 1.1.6 or older")
    plat = dev.pget("SYS.PLATFORM", length=7)
    ver = (plat[1], plat[4], plat[5]) if len(plat) >= 6 else (0, 0, 0)
    if ver < (1, 1, 7):
        raise Skip(f"refusing the page 0 walk: SYS.PLATFORM reports {ver[0]}.{ver[1]}.{ver[2]}, "
                   f"page 0 is only empty from 1.1.7")
    try:
        dev.raw_get(OLD_GET_PLATFORM, 7)
    except Stall:
        return
    raise Skip("refusing the page 0 walk: page 0 still answers the old GET_PLATFORM")


def _plan():
    """(bmRequestType, bRequest, wIndex, wLength or payload); the last entry is a device-recipient SET."""
    out = []
    for windex in (0x0000, 0x0002):
        for req in range(256):
            out += [(BM_IN_IFACE, req, windex, 4), (BM_OUT_IFACE, req, windex, b""),
                    (BM_OUT_IFACE, req, windex, b"\x00\x00\x00\x00")]
    for req in range(256):
        out += [(BM_IN_DEV, req, 0x0000, 4), (BM_OUT_DEV, req, 0x0000, b""),
                (BM_OUT_DEV, req, 0x0000, b"\x00\x00\x00\x00")]
    return [r for r in out if (r[0], r[1], r[2]) != MS_OS_20]


def _state(dev):
    return dev.pget("STORE.ALL_PARAMS", length=6136), dev.pget("STORE.DIRECTORY", length=7)


def _label(bm, req, windex, dl):
    n = dl if isinstance(dl, int) else len(dl)
    return f"0x{bm:02X}/0x{req:02X} wIndex 0x{windex:04X} len {n}"


@test("paged_page0")
def ms_os20_descriptor_served(dev, profile, chk):
    """The Microsoft OS 2.0 request (0xC0, 0x01, wIndex 7) still returns the WINUSB descriptor set."""
    bm, req, windex = MS_OS_20
    dev.gets += 1
    try:
        ms = bytes(dev.dev.ctrl_transfer(bm, req, 0, windex, MS_OS_20_LEN, timeout=1500))
    except usb.core.USBError as e:
        chk.ok(False, f"MS OS 2.0 descriptor request failed: {e}")
        return
    chk.ok(len(ms) >= 10, f"descriptor set length {len(ms)}")
    chk.ok(b"WINUSB" in ms, "descriptor set names WINUSB")


@test("paged_page0")
def page0_walk_all_stall(dev, profile, chk):
    """Every bRequest on page 0 (wIndex 0 and 2, GET and both SET shapes, both recipients) STALLs, state unchanged."""
    _guard(dev)
    plan = _plan()
    before = _state(dev)
    answered, errors = [], []
    for bm, req, windex, dl in plan:
        if bm & 0x80:
            dev.gets += 1
        else:
            dev.sets += 1
        try:
            r = dev.dev.ctrl_transfer(bm, req, 0, windex, dl, timeout=WALK_TIMEOUT_MS)
            answered.append(f"{_label(bm, req, windex, dl)} -> {bytes(r).hex() if bm & 0x80 else r}")
        except usb.core.USBError as e:
            if getattr(e, "errno", None) != 32 and "pipe" not in str(e).lower():
                errors.append(f"{_label(bm, req, windex, dl)}: {e}")
    lr = dev.last_result()
    chk.eq(len(answered), 0, f"page 0 requests answered (first: {answered[:5]})")
    chk.eq(len(errors), 0, f"page 0 requests failed other than by STALL (first: {errors[:5]})")
    bm, req, windex, dl = plan[-1]
    want = ((windex & 0xFF00) | req, "GET" if bm & 0x80 else "SET", "VC_ERR_UNKNOWN", 0)
    got = (lr.cmd_id, lr.direction, lr.result_name, lr.wvalue) if lr else None
    chk.eq(got, want, f"SYS.LAST_RESULT after the last request ({_label(bm, req, windex, dl)})")
    after = _state(dev)
    chk.eq(len(after[0]), len(before[0]), "STORE.ALL_PARAMS length")
    diff = [i for i, (a, b) in enumerate(zip(before[0], after[0])) if a != b]
    chk.eq(diff[:8], [], "STORE.ALL_PARAMS bytes changed by the walk (first offsets)")
    chk.eq(after[1], before[1], "STORE.DIRECTORY")
    if not answered and not errors:
        chk.note(f"{len(plan)} page 0 requests, all STALLed")
