"""
DspiDevice: the QA suite's single USB vendor-control transport (VID/PID as in tools/dspi_poke). It speaks
only the paged protocol (Documentation/commands.md): pget/pset name a command as PAGE.NAME or its ID.
Refusals raise PagedStall carrying the VC_* reason, a vanished device raises Disconnected.
Control-plane only: no audio streaming, no isochronous endpoints.
"""

from __future__ import annotations

import json
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path

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

SYS_PAGE = 0x01
SYS_PROTOCOL_INFO = 0x0110
SYS_LAST_RESULT = 0x0114
PROTOCOL_MAGIC = b"DSPi"

VC_RESULTS = {
    0x00: "VC_OK", 0x10: "VC_ERR_UNKNOWN", 0x11: "VC_ERR_DIRECTION", 0x12: "VC_ERR_LENGTH",
    0x13: "VC_ERR_ADDRESS", 0x14: "VC_ERR_VALUE", 0x15: "VC_ERR_UNSUPPORTED", 0x16: "VC_ERR_BLOCKED",
    0x17: "VC_ERR_BUSY", 0x18: "VC_ERR_STATE", 0x19: "VC_ERR_PIN_INVALID", 0x1A: "VC_ERR_PIN_IN_USE",
}
VC_CODES = {v: k for k, v in VC_RESULTS.items()}


def paged_windex(page: int) -> int:
    """The one place the page carrier is defined (plan 11.1 may move it off wIndex)."""
    return ((page & 0xFF) << 8) | VENDOR_IFACE


# --- Exceptions -------------------------------------------------------------

class Stall(Exception):
    """The firmware refused/stalled the control transfer (libusb pipe error)."""

    def __init__(self, request, direction, errno, message):
        self.request = request   # bRequest, or the command ID for a PagedStall
        self.direction = direction
        self.errno = errno
        super().__init__(f"STALL on {direction} 0x{request:02X}: {message}")


class PagedStall(Stall):
    """A paged request STALLed. `result` is the VC_* code from SYS.LAST_RESULT, None if unreadable."""

    def __init__(self, cmd_id, direction, addr, errno, message, result=None):
        self.cmd_id = cmd_id
        self.addr = addr
        self.result = result
        self.result_name = VC_RESULTS.get(result, f"0x{result:02X}") if result is not None else None
        why = f" [{self.result_name}]" if self.result_name else ""
        super().__init__(cmd_id, direction, errno, f"addr 0x{addr:04X}{why}: {message}")


class Disconnected(Exception):
    """The device vanished from the bus (crash / re-enumeration / unplug)."""


class Timeout(Exception):
    """A transfer kept timing out even after retries (device wedged, not a stall)."""


class ProtocolError(Exception):
    """The device does not speak the paged protocol (SYS.PROTOCOL_INFO STALLed or lacked "DSPi")."""


# --- Repo files -------------------------------------------------------------

def _find_repo_file(rel: Path) -> Path | None:
    for start in (Path(__file__).resolve().parent, Path.cwd().resolve()):
        for parent in (start, *start.parents):
            cand = parent / rel
            if cand.is_file():
                return cand
    return None


def _find_config_h() -> Path | None:
    return _find_repo_file(Path("firmware") / "DSPi" / "config.h")


REGISTRY_REL = Path("Documentation") / "protocol" / "dspi_commands.json"


# --- Paged command metadata -------------------------------------------------

@dataclass(frozen=True)
class CommandInfo:
    page: int
    cmd: int
    name: str            # PAGE.NAME
    has_get: bool
    has_set: bool
    addr: str            # contract ADDR_KIND token, "?" when unknown
    get_len: object      # int, "var" or None
    source: str          # "registry" or "numeric"

    @property
    def id(self) -> int:
        return (self.page << 8) | self.cmd


_REGISTRY_CACHE: dict = {}


def load_registry(path=None) -> dict[str, CommandInfo]:
    """{PAGE.NAME: CommandInfo} from dspi_commands.json; {} when the file is absent or unreadable."""
    if isinstance(path, dict):
        return _registry_from_json(path)
    key = str(path) if path else "default"
    if key not in _REGISTRY_CACHE:
        p = Path(path) if path else _find_repo_file(REGISTRY_REL)
        try:
            _REGISTRY_CACHE[key] = _registry_from_json(json.loads(p.read_text())) if p else {}
        except (OSError, ValueError, KeyError, TypeError):
            _REGISTRY_CACHE[key] = {}
    return _REGISTRY_CACHE[key]


def _registry_from_json(doc: dict) -> dict[str, CommandInfo]:
    out = {}
    for pg in doc.get("pages", []):
        for c in pg.get("commands", []):
            name = f"{pg['mnemonic']}.{c['name']}"
            out[name] = CommandInfo(pg["page"], c["cmd"], name, bool(c.get("get")), bool(c.get("set")),
                                    c.get("addr", "?"), c.get("get_len"), "registry")
    return out


def resolve_command(cmd, registry=None) -> CommandInfo:
    """Numeric IDs always resolve, names only through the registry (dspi_commands.json)."""
    if isinstance(cmd, CommandInfo):
        return cmd
    if isinstance(cmd, int):
        if not 0 <= cmd <= 0xFFFF:
            raise ValueError(f"command ID out of range: {cmd:#x}")
        for info in load_registry(registry).values():
            if info.id == cmd:
                return info
        return CommandInfo(cmd >> 8, cmd & 0xFF, f"0x{cmd:04X}", True, True, "?", None, "numeric")
    name = str(cmd).upper()
    table = load_registry(registry)
    if name in table:
        return table[name]
    raise KeyError(f"unknown paged command {cmd!r} (not in dspi_commands.json)")


@dataclass(frozen=True)
class LastResult:
    cmd_id: int
    direction: str       # "SET" or "GET"
    result: int
    wvalue: int

    @property
    def result_name(self) -> str:
        return VC_RESULTS.get(self.result, f"0x{self.result:02X}")


# --- The device wrapper -----------------------------------------------------

class DspiDevice:
    def __init__(self, timeout_ms: int = 1500, usb_dev=None, registry=None):
        dev = usb_dev if usb_dev is not None else usb.core.find(idVendor=VID, idProduct=PID)
        if dev is None:
            raise Disconnected(
                f"no DSPi at {VID:04X}:{PID:04X}. Is it plugged in and not held by another app?")
        self.dev = dev
        self.timeout = timeout_ms
        self.registry = registry
        # Count every transfer for the report.
        self.gets = 0
        self.sets = 0
        # Re-enumerations on benign commands are themselves a finding, so they are counted.
        self.reenumerations = 0
        self._paged_ok = None   # None until SYS.PROTOCOL_INFO has been probed
        self._paged_cmds = None  # {page: (GET cmds, SET cmds)} read from the device once

    def _reacquire(self) -> bool:
        try:
            usb.util.dispose_resources(self.dev)
        except Exception:  # noqa: BLE001
            pass
        d = usb.core.find(idVendor=VID, idProduct=PID)
        if d is not None:
            self.dev = d
            self.reenumerations += 1
            self._paged_ok = None   # may be different firmware now
            self._paged_cmds = None
            return True
        return False

    # -- transfers -----------------------------------------------------------

    # Transient timeouts (errno 60 ETIMEDOUT / 110) happen when the device is
    # briefly busy with a deferred pipeline reset (type switch, pin move, bulk
    # apply) during which it disables the USB control IRQ.  These are NOT stalls
    # and NOT crashes, so retry a few times before giving up.
    _TIMEOUT_RETRIES = 4
    _TIMEOUT_BACKOFF_S = 0.15
    _REACQUIRE_ATTEMPTS = 6
    _REACQUIRE_BACKOFF_S = 0.4

    def _xfer(self, direction, request, wvalue, windex, data_or_len):
        bm = BM_VENDOR_IN if direction == "GET" else BM_VENDOR_OUT
        timeouts = 0
        reacquires = 0
        while True:
            try:
                return self.dev.ctrl_transfer(bm, request, wvalue, windex, data_or_len, timeout=self.timeout)
            except usb.core.USBError as e:
                msg = str(e)
                errno = getattr(e, "errno", None)
                # ENODEV: a re-enumeration staled the handle. Re-acquire and retry.
                if errno == 19 or "No such device" in msg or "no device" in msg.lower():
                    if reacquires < self._REACQUIRE_ATTEMPTS:
                        reacquires += 1
                        time.sleep(self._REACQUIRE_BACKOFF_S)
                        self._reacquire()
                        continue
                    raise Disconnected(f"device gone during {direction} 0x{request:02X} "
                                       f"after {reacquires} re-acquire attempts: {msg}") from e
                # Transient busy window during a deferred reset.
                if errno in (60, 110) or "timed out" in msg.lower() or "timeout" in msg.lower():
                    if timeouts < self._TIMEOUT_RETRIES:
                        timeouts += 1
                        time.sleep(self._TIMEOUT_BACKOFF_S)
                        continue
                    raise Timeout(f"{direction} 0x{request:02X} timed out after "
                                  f"{timeouts} retries: {msg}") from e
                # EPIPE / everything else on a healthy session => protocol stall.
                raise Stall(request, direction, errno, msg) from e

    def raw_request(self, bm_request_type: int, request: int, wvalue: int, windex: int, data_or_len,
                    timeout_ms: int | None = None):
        """One control request sent exactly as given: no page carrier, probe, retry or re-acquire, and a
        USBError propagates. Only tests/paged_page0.py uses it, to prove requests outside the paged scheme STALL."""
        if bm_request_type & 0x80:
            self.gets += 1
        else:
            self.sets += 1
        r = self.dev.ctrl_transfer(bm_request_type, request, wvalue, windex, data_or_len,
                                   timeout=self.timeout if timeout_ms is None else timeout_ms)
        return bytes(r) if bm_request_type & 0x80 else r

    def paged_commands(self) -> dict:
        """{page: (GET cmd set, SET cmd set)} from SYS.PAGE_LIST / PAGE_COMMANDS, read once per handle.
        Empty on firmware without the dispatcher. A failed read is not cached, so it is retried."""
        if self._paged_cmds is not None:
            return self._paged_cmds
        if self._paged_ok is None:
            self.protocol_info()
        if not self._paged_ok:
            self._paged_cmds = {}
            return self._paged_cmds
        try:
            pl = self.pget(0x0111, length=256)
            out = {}
            for page in pl[0::2]:
                bm = self.pget(0x0112, page, 64)
                bits = [{i for i in range(256) if bm[half + i // 8] >> (i % 8) & 1} for half in (0, 32)]
                out[page] = (frozenset(bits[0]), frozenset(bits[1]))
        except (Stall, Timeout):
            return {}
        self._paged_cmds = out
        return out

    # -- paged commands ------------------------------------------------------

    def resolve(self, cmd) -> CommandInfo:
        return resolve_command(cmd, self.registry)

    def _require_paged(self):
        # 1.1.6 ignores wIndex and would run a paged command as the old opcode
        # with the same number (0x53 is FACTORY_RESET), so probe first (plan 5.14).
        if self._paged_ok is None:
            self.protocol_info()
        if not self._paged_ok:
            raise ProtocolError("device does not answer SYS.PROTOCOL_INFO, so it predates the paged protocol: "
                                "this harness needs DSPi firmware 1.1.7 or later and sends it nothing else")

    def pget(self, cmd, addr: int = 0, length: int | None = None) -> bytes:
        """GET a paged command by PAGE.NAME or ID. Length defaults to the table's fixed reply size."""
        info = self.resolve(cmd)
        self._require_paged()
        if length is None:
            length = info.get_len if isinstance(info.get_len, int) and info.get_len > 0 else 256
        self.gets += 1
        try:
            return bytes(self._xfer("GET", info.cmd, addr, paged_windex(info.page), length))
        except Stall as e:
            raise self._paged_stall(info, "GET", addr, e) from e

    def pset(self, cmd, data: bytes = b"", addr: int = 0) -> int:
        """SET a paged command by PAGE.NAME or ID. Raises PagedStall carrying the VC_* reason."""
        info = self.resolve(cmd)
        self._require_paged()
        self.sets += 1
        try:
            return self._xfer("SET", info.cmd, addr, paged_windex(info.page), bytes(data))
        except Stall as e:
            raise self._paged_stall(info, "SET", addr, e) from e

    def _paged_stall(self, info: CommandInfo, direction: str, addr: int, e: Stall) -> PagedStall:
        try:
            lr = self.last_result()
        except Timeout:
            lr = None
        # LAST_RESULT is per transport, so another USB host can overwrite it.
        ok = lr is not None and lr.cmd_id == info.id and lr.direction == direction and lr.wvalue == addr
        return PagedStall(info.id, direction, addr, e.errno, f"{info.name} refused",
                          lr.result if ok else None)

    def last_result(self) -> LastResult | None:
        """SYS.LAST_RESULT, or None where it STALLs (firmware without the dispatcher)."""
        if self._paged_ok is None:
            self.protocol_info()
        if not self._paged_ok:
            return None
        try:
            self.gets += 1
            r = bytes(self._xfer("GET", SYS_LAST_RESULT & 0xFF, 0, paged_windex(SYS_PAGE), 8))
        except Stall:
            return None
        if len(r) < 6:
            return None
        cmd_id, dirb, res, wv = struct.unpack_from("<HBBH", r)
        return LastResult(cmd_id, "GET" if dirb == 1 else "SET", res, wv)

    def protocol_info(self) -> dict | None:
        """Probe SYS.PROTOCOL_INFO: None on a STALL (1.1.6 or older), parsed fields otherwise."""
        try:
            self.gets += 1
            r = bytes(self._xfer("GET", SYS_PROTOCOL_INFO & 0xFF, 0, paged_windex(SYS_PAGE), 16))
        except Stall:
            self._paged_ok = False
            return None
        if len(r) < 16 or r[:4] != PROTOCOL_MAGIC:
            self._paged_ok = False
            raise ProtocolError(f"SYS.PROTOCOL_INFO without the DSPi magic: {r.hex()}")
        major, minor, pages, flags, max_set, max_get, reserved = struct.unpack_from("<BBBBHHI", r, 4)
        self._paged_ok = True
        return {"magic": r[:4].decode(), "major": major, "minor": minor, "page_count": pages,
                "flags": flags, "max_set": max_set, "max_get": max_get, "reserved": reserved}

    # -- typed paged helpers -------------------------------------------------

    def pget_u8(self, cmd, addr: int = 0) -> int:
        return self.pget(cmd, addr, 1)[0]

    def pget_u16(self, cmd, addr: int = 0) -> int:
        return struct.unpack("<H", self.pget(cmd, addr, 2))[0]

    def pget_u32(self, cmd, addr: int = 0) -> int:
        return struct.unpack("<I", self.pget(cmd, addr, 4))[0]

    def pget_i32(self, cmd, addr: int = 0) -> int:
        return struct.unpack("<i", self.pget(cmd, addr, 4))[0]

    def pget_f32(self, cmd, addr: int = 0) -> float:
        return struct.unpack("<f", self.pget(cmd, addr, 4))[0]

    def pget_str(self, cmd, addr: int = 0, length: int = 32) -> bytes:
        """Raw bytes up to the first NUL (caller decodes)."""
        return self.pget(cmd, addr, length).split(b"\x00", 1)[0]

    def pset_u8(self, cmd, val: int, addr: int = 0) -> int:
        return self.pset(cmd, struct.pack("<B", val & 0xFF), addr)

    def pset_u16(self, cmd, val: int, addr: int = 0) -> int:
        return self.pset(cmd, struct.pack("<H", val & 0xFFFF), addr)

    def pset_u32(self, cmd, val: int, addr: int = 0) -> int:
        return self.pset(cmd, struct.pack("<I", val & 0xFFFFFFFF), addr)

    def pset_i32(self, cmd, val: int, addr: int = 0) -> int:
        return self.pset(cmd, struct.pack("<i", val), addr)

    def pset_f32(self, cmd, val: float, addr: int = 0) -> int:
        return self.pset(cmd, struct.pack("<f", val), addr)

    def pget_ready(self, cmd, addr: int = 0, length: int | None = None, retries: int = 3) -> bytes:
        """pget that must succeed, retried through wait_ready() across a deferred-reset busy window.
        Never use it where a refusal is the expected answer: the retry would hide it."""
        for attempt in range(retries):
            try:
                return self.pget(cmd, addr, length)
            except (Stall, Timeout, ProtocolError):
                if attempt == retries - 1:
                    raise
                self.wait_ready()
                time.sleep(0.05)

    def is_alive(self) -> bool:
        """One SYS.PLATFORM read answered cleanly."""
        try:
            return len(self.pget("SYS.PLATFORM", length=7)) == 7
        except (Stall, Disconnected, Timeout, ProtocolError, usb.core.USBError):
            return False

    def wait_ready(self, timeout_s: float = 4.0, poll_s: float = 0.1) -> bool:
        """Poll SYS.PLATFORM until it answers. A deferred restart (type switch, pin move, bulk apply, preset
        load) briefly blocks control transfers, so only a device that never recovers reports False."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                if len(self.pget("SYS.PLATFORM", length=7)) == 7:
                    return True
            except Disconnected:
                return False
            except ProtocolError:
                # A probe that STALLed inside the busy window cached "not paged"; probe again.
                self._paged_ok = None
            except (Stall, Timeout):
                pass
            time.sleep(poll_s)
        return False
