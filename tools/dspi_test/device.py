"""
device.py — DspiDevice: thin, test-oriented wrapper around USB vendor control
transfers to a connected DSPi.

This is the single transport layer for the QA suite.  It mirrors the conventions
in tools/dspi_poke/dspi_poke.py (same VID/PID, same bmRequestType bytes) but adds:

  * Typed get_*/set_* helpers (u8/u16/u32/i32/f32) with little-endian packing.
  * STALL detection: a control transfer that the firmware refuses (handler
    `return false`, oversized payload, or unknown opcode) shows up as a libusb
    pipe error.  We translate that into a `Stall` exception so tests can assert
    "this should STALL" / "this must NOT STALL".
  * Disconnect detection: a vanished device raises `Disconnected` (distinct from
    a STALL) so the runner can abort loudly instead of mislabelling a crash as a
    failed assertion.
  * `OP`: the canonical opcode map, parsed from firmware/DSPi/config.h so the
    suite never drifts from the firmware's `#define REQ_* 0xNN` table.

Control-plane only: no audio streaming, no isochronous endpoints.
"""

from __future__ import annotations

import re
import struct
import sys
import time
from pathlib import Path
from types import SimpleNamespace

try:
    import usb.core
    import usb.util
except ImportError:  # pragma: no cover
    sys.exit("pyusb not installed.  Run: pip install pyusb  (macOS also needs `brew install libusb`)")


# --- USB constants (match dspi_poke.py) -------------------------------------

VID, PID = 0x2E8B, 0xFEAA
VENDOR_IFACE = 2
BM_VENDOR_IN = 0xC1   # device -> host (vendor | interface)
BM_VENDOR_OUT = 0x41  # host   -> device


# --- Exceptions -------------------------------------------------------------

class Stall(Exception):
    """The firmware refused/stalled the control transfer (libusb pipe error)."""

    def __init__(self, opcode, direction, errno, message):
        self.opcode = opcode
        self.direction = direction
        self.errno = errno
        super().__init__(f"STALL on {direction} 0x{opcode:02X}: {message}")


class Disconnected(Exception):
    """The device vanished from the bus (crash / re-enumeration / unplug)."""


class Timeout(Exception):
    """A transfer kept timing out even after retries (device wedged, not a stall)."""


# --- Opcode table (parsed from config.h, single source of truth) ------------

def _find_config_h() -> Path | None:
    rel = Path("firmware") / "DSPi" / "config.h"
    for start in (Path(__file__).resolve().parent, Path.cwd().resolve()):
        for parent in (start, *start.parents):
            cand = parent / rel
            if cand.is_file():
                return cand
    return None


def load_opcodes(config_h: Path | None = None) -> dict[str, int]:
    """Parse `#define REQ_FOO 0xNN` lines into {SHORTNAME: opcode}.

    Keys are stored WITHOUT the REQ_ prefix (e.g. "GET_PLATFORM") so tests can
    write `OP.GET_PLATFORM`.  Returns {} on failure (callers should treat an
    empty map as a fatal setup error).
    """
    path = config_h or _find_config_h()
    if path is None or not path.is_file():
        return {}
    pat = re.compile(r"^\s*#define\s+REQ_(\w+)\s+(0[xX][0-9A-Fa-f]+|\d+)")
    out: dict[str, int] = {}
    for line in path.read_text().splitlines():
        m = pat.match(line)
        if m:
            out[m.group(1).upper()] = int(m.group(2), 0)
    return out


_OPCODES = load_opcodes()
# Attribute access: OP.GET_PLATFORM == 0x7F
OP = SimpleNamespace(**_OPCODES)
# Reverse map for labelling in reports/errors.
OPCODE_NAME = {v: f"REQ_{k}" for k, v in _OPCODES.items()}


def opcode_label(opcode: int) -> str:
    return OPCODE_NAME.get(opcode, f"0x{opcode:02X}")


# --- The device wrapper -----------------------------------------------------

class DspiDevice:
    def __init__(self, timeout_ms: int = 1500):
        dev = usb.core.find(idVendor=VID, idProduct=PID)
        if dev is None:
            raise Disconnected(
                f"no DSPi at {VID:04X}:{PID:04X} — is it plugged in and not held by another app?")
        self.dev = dev
        self.timeout = timeout_ms
        # Count every transfer for the report.
        self.gets = 0
        self.sets = 0
        # A device that drops off and comes back (re-enumeration) staled the
        # handle; we re-acquire and continue.  Counted because frequent
        # re-enumerations on benign commands are themselves a finding.
        self.reenumerations = 0

    def _reacquire(self) -> bool:
        try:
            usb.util.dispose_resources(self.dev)
        except Exception:  # noqa: BLE001
            pass
        d = usb.core.find(idVendor=VID, idProduct=PID)
        if d is not None:
            self.dev = d
            self.reenumerations += 1
            return True
        return False

    # -- raw transfers -------------------------------------------------------

    # Transient timeouts (errno 60 ETIMEDOUT / 110) happen when the device is
    # briefly busy with a deferred pipeline reset (type switch, pin move, bulk
    # apply) during which it disables the USB control IRQ.  These are NOT stalls
    # and NOT crashes — retry a few times before giving up.
    _TIMEOUT_RETRIES = 4
    _TIMEOUT_BACKOFF_S = 0.15
    _REACQUIRE_ATTEMPTS = 6
    _REACQUIRE_BACKOFF_S = 0.4

    def _xfer(self, direction, opcode, wvalue, windex, data_or_len):
        bm = BM_VENDOR_IN if direction == "GET" else BM_VENDOR_OUT
        timeouts = 0
        reacquires = 0
        while True:
            try:
                return self.dev.ctrl_transfer(bm, opcode, wvalue, windex, data_or_len, timeout=self.timeout)
            except usb.core.USBError as e:
                msg = str(e)
                errno = getattr(e, "errno", None)
                # ENODEV: a re-enumeration staled the handle — re-acquire & retry.
                if errno == 19 or "No such device" in msg or "no device" in msg.lower():
                    if reacquires < self._REACQUIRE_ATTEMPTS:
                        reacquires += 1
                        time.sleep(self._REACQUIRE_BACKOFF_S)
                        self._reacquire()
                        continue
                    raise Disconnected(f"device gone during {direction} 0x{opcode:02X} "
                                       f"after {reacquires} re-acquire attempts: {msg}") from e
                # Transient busy window during a deferred reset.
                if errno in (60, 110) or "timed out" in msg.lower() or "timeout" in msg.lower():
                    if timeouts < self._TIMEOUT_RETRIES:
                        timeouts += 1
                        time.sleep(self._TIMEOUT_BACKOFF_S)
                        continue
                    raise Timeout(f"{direction} 0x{opcode:02X} timed out after "
                                  f"{timeouts} retries: {msg}") from e
                # EPIPE / everything else on a healthy session => protocol stall.
                raise Stall(opcode, direction, errno, msg) from e

    def get(self, opcode: int, length: int, wvalue: int = 0, windex: int = VENDOR_IFACE) -> bytes:
        """Device->host vendor request.  Raises Stall if refused, Timeout if wedged."""
        self.gets += 1
        return bytes(self._xfer("GET", opcode, wvalue, windex, length))

    def set(self, opcode: int, payload: bytes = b"", wvalue: int = 0, windex: int = VENDOR_IFACE) -> int:
        """Host->device vendor request.  Returns bytes accepted.  Raises Stall if refused."""
        self.sets += 1
        return self._xfer("SET", opcode, wvalue, windex, payload)

    # -- typed GET helpers ---------------------------------------------------

    def get_u8(self, opcode, wvalue=0) -> int:
        return self.get(opcode, 1, wvalue)[0]

    def get_u16(self, opcode, wvalue=0) -> int:
        return struct.unpack("<H", self.get(opcode, 2, wvalue))[0]

    def get_u32(self, opcode, wvalue=0) -> int:
        return struct.unpack("<I", self.get(opcode, 4, wvalue))[0]

    def get_i32(self, opcode, wvalue=0) -> int:
        return struct.unpack("<i", self.get(opcode, 4, wvalue))[0]

    def get_f32(self, opcode, wvalue=0) -> float:
        return struct.unpack("<f", self.get(opcode, 4, wvalue))[0]

    def get_str(self, opcode, length=32, wvalue=0) -> bytes:
        """Raw bytes up to the first NUL (caller decodes)."""
        return self.get(opcode, length, wvalue).split(b"\x00", 1)[0]

    def get_ready(self, opcode, length, wvalue=0, retries=3):
        """GET that must always succeed; retry through wait_ready() on a transient
        stall/timeout (the busy window after a deferred reset or re-enumeration).

        Use ONLY for reads that have no invalid-param path of their own — never
        for STALL assertions, where a transient retry would mask the result.
        """
        for attempt in range(retries):
            try:
                return self.get(opcode, length, wvalue)
            except (Stall, Timeout):
                if attempt == retries - 1:
                    raise
                self.wait_ready()
                time.sleep(0.05)

    # -- typed SET helpers ---------------------------------------------------

    def set_u8(self, opcode, val, wvalue=0) -> int:
        return self.set(opcode, struct.pack("<B", val & 0xFF), wvalue)

    def set_u16(self, opcode, val, wvalue=0) -> int:
        return self.set(opcode, struct.pack("<H", val & 0xFFFF), wvalue)

    def set_f32(self, opcode, val, wvalue=0) -> int:
        return self.set(opcode, struct.pack("<f", val), wvalue)

    def set_bytes(self, opcode, payload, wvalue=0) -> int:
        return self.set(opcode, payload, wvalue)

    # -- liveness ------------------------------------------------------------

    def is_alive(self) -> bool:
        """A GET_PLATFORM that returns the expected shape => device responsive.

        Re-finds the handle first so a re-enumeration (crash recovery) is seen
        as a fresh, distinct device rather than a silent success on a stale one.
        """
        try:
            data = self.get(OP.GET_PLATFORM, 4)
            return len(data) == 4
        except (Stall, Disconnected, Timeout):
            return False
        except usb.core.USBError:
            return False

    def wait_ready(self, timeout_s: float = 4.0, poll_s: float = 0.1) -> bool:
        """Poll GET_PLATFORM until it answers cleanly.

        Operations that defer a mute-bracketed pipeline reset (output-type switch,
        pin move, bulk apply, input switch, preset load, factory reset) briefly
        disable the USB control IRQ; a transfer in that window stalls or times out.
        That is expected and transient — tolerate it here and only report failure
        if the device never recovers (a real crash/hang).
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                if len(self.get(OP.GET_PLATFORM, 4)) == 4:
                    return True
            except Disconnected:
                return False
            except (Stall, Timeout):
                pass
            time.sleep(poll_s)
        return False
