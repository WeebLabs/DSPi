"""
DspiDevice: the QA suite's single USB vendor-control transport (VID/PID and bmRequestType as in
tools/dspi_poke). Refusals raise Stall, a vanished device raises Disconnected, OP is the frozen 1.1.6
opcode table, and pget/pset plus the protocol modes are described in README.md.
Control-plane only: no audio streaming, no isochronous endpoints.
"""

from __future__ import annotations

import atexit
import json
import os
import re
import struct
import sys
import time
from dataclasses import dataclass
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

    def __init__(self, opcode, direction, errno, message):
        self.opcode = opcode
        self.direction = direction
        self.errno = errno
        super().__init__(f"STALL on {direction} 0x{opcode:02X}: {message}")


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


class UnmappedRequest(Exception):
    """Hybrid mode on firmware without page 0 got an old opcode legacy_map.json does not know.
    Deliberately not a Stall, so a test cannot mistake it for a device refusal."""


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
LEGACY_MAP_REL = Path("Documentation") / "protocol" / "legacy_map.json"


# --- Opcode table (parsed from config.h, single source of truth) ------------

def load_opcodes(config_h: Path | None = None) -> dict[str, int]:
    """Parse `#define REQ_FOO 0xNN` lines into {SHORTNAME: opcode}, keys WITHOUT the REQ_ prefix
    (so tests write `OP.GET_PLATFORM`). Returns {} where config.h has none, as from 1.1.7 on.
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


def load_legacy_opcodes() -> dict[str, int]:
    """The frozen 1.1.6 table, used once the flip removes REQ_* from config.h."""
    path = Path(__file__).resolve().parent / "golden" / "legacy_opcodes.json"
    try:
        return {k.upper(): int(v) for k, v in json.loads(path.read_text())["opcodes"].items()}
    except (OSError, ValueError, KeyError):
        return {}


_OPCODES = load_opcodes() or load_legacy_opcodes()
# Attribute access: OP.GET_PLATFORM == 0x7F
OP = SimpleNamespace(**_OPCODES)
# Reverse map for labelling in reports/errors.
OPCODE_NAME = {v: f"REQ_{k}" for k, v in _OPCODES.items()}


def opcode_label(opcode: int) -> str:
    return OPCODE_NAME.get(opcode, f"0x{opcode:02X}")


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
    source: str          # "registry", "legacy_map" or "numeric"

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


_CATALOG_CACHE: dict = {}


def load_catalog() -> dict[str, CommandInfo]:
    """The plan section 5 numbering kept in legacy_map.json, the fallback for names."""
    if "c" not in _CATALOG_CACHE:
        p = _find_repo_file(LEGACY_MAP_REL)
        out = {}
        try:
            doc = json.loads(p.read_text()) if p else {}
            pages = doc.get("pages", {})
            for name, c in doc.get("commands", {}).items():
                out[name] = CommandInfo(pages[c["page"]], c["cmd"], name, "G" in c["dir"], "S" in c["dir"],
                                        c.get("addr", "?"), c.get("get_len"), "legacy_map")
        except (OSError, ValueError, KeyError, TypeError):
            out = {}
        _CATALOG_CACHE["c"] = out
    return _CATALOG_CACHE["c"]


def resolve_command(cmd, registry=None) -> CommandInfo:
    """Numeric IDs always resolve. Names try the registry first, then the legacy_map catalog."""
    if isinstance(cmd, CommandInfo):
        return cmd
    if isinstance(cmd, int):
        if not 0 <= cmd <= 0xFFFF:
            raise ValueError(f"command ID out of range: {cmd:#x}")
        for table in (load_registry(registry), load_catalog()):
            for info in table.values():
                if info.id == cmd:
                    return info
        return CommandInfo(cmd >> 8, cmd & 0xFF, f"0x{cmd:04X}", True, True, "?", None, "numeric")
    name = str(cmd).upper()
    for table in (load_registry(registry), load_catalog()):
        if name in table:
            return table[name]
    raise KeyError(f"unknown paged command {cmd!r} (not in dspi_commands.json or legacy_map.json)")


@dataclass(frozen=True)
class LastResult:
    cmd_id: int
    direction: str       # "SET" or "GET"
    result: int
    wvalue: int

    @property
    def result_name(self) -> str:
        return VC_RESULTS.get(self.result, f"0x{self.result:02X}")


def _normalise_protocol(p: str | None) -> str:
    p = (p or os.environ.get("DSPI_TEST_PROTOCOL") or "auto").strip().lower()
    if p in ("old", "send-old", "send_old", "legacy"):
        return "old"
    if p in ("translate", "paged", "hybrid", "auto"):
        return "translate" if p == "paged" else p
    raise ValueError(f"unknown DSPI_TEST_PROTOCOL {p!r} (use auto, old, translate or hybrid)")


_LEGACY_NAMES: dict = {}


def _legacy_name(code: int) -> str | None:
    """1.1.6 name of an old opcode that legacy_map.json translates, None for any other number."""
    if not _LEGACY_NAMES:
        from . import legacy_map
        _LEGACY_NAMES.update({r["code"]: n for n, r in legacy_map.load_map()["opcodes"].items()})
    return _LEGACY_NAMES.get(code)


# --- The device wrapper -----------------------------------------------------

class DspiDevice:
    def __init__(self, timeout_ms: int = 1500, protocol: str | None = None, usb_dev=None, registry=None):
        dev = usb_dev if usb_dev is not None else usb.core.find(idVendor=VID, idProduct=PID)
        if dev is None:
            raise Disconnected(
                f"no DSPi at {VID:04X}:{PID:04X}. Is it plugged in and not held by another app?")
        self.dev = dev
        self.timeout = timeout_ms
        self.protocol = _normalise_protocol(protocol)
        self.protocol_auto = self.protocol == "auto"
        self.registry = registry
        # Count every transfer for the report.
        self.gets = 0
        self.sets = 0
        # A device that drops off and comes back (re-enumeration) staled the
        # handle; we re-acquire and continue.  Counted because frequent
        # re-enumerations on benign commands are themselves a finding.
        self.reenumerations = 0
        # Translated OUT writes the firmware refused. 1.1.6 ACKed these silently,
        # so legacy_map swallows the STALL and records it here instead.
        self.swallowed: list[dict] = []
        self._paged_ok = None   # None until SYS.PROTOCOL_INFO has been probed
        self._paged_cmds = None  # {page: (GET cmds, SET cmds)} read from the device once
        self._page0 = None      # None until page0_alive() has probed
        # Hybrid routing per old opcode name: calls sent paged vs left on page 0.
        self.hybrid_counts = {"translated": {}, "passed": {}}
        self._hybrid_atexit = False
        if self.protocol_auto:
            # The probe STALLs harmlessly on 1.1.6 (plan 5.14), which still needs the old opcodes.
            self.protocol = "translate" if self.protocol_info() is not None else "old"
        if self.protocol == "hybrid":
            self._register_hybrid_summary()

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
            self._page0 = None
            return True
        return False

    # -- raw transfers -------------------------------------------------------

    # Transient timeouts (errno 60 ETIMEDOUT / 110) happen when the device is
    # briefly busy with a deferred pipeline reset (type switch, pin move, bulk
    # apply) during which it disables the USB control IRQ.  These are NOT stalls
    # and NOT crashes, so retry a few times before giving up.
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
                # ENODEV: a re-enumeration staled the handle. Re-acquire and retry.
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

    def raw_get(self, opcode: int, length: int, wvalue: int = 0, windex: int = VENDOR_IFACE) -> bytes:
        """Untranslated legacy IN request, whatever the protocol mode."""
        self.gets += 1
        return bytes(self._xfer("GET", opcode, wvalue, windex, length))

    def raw_set(self, opcode: int, payload: bytes = b"", wvalue: int = 0, windex: int = VENDOR_IFACE) -> int:
        """Untranslated legacy OUT request, whatever the protocol mode."""
        self.sets += 1
        return self._xfer("SET", opcode, wvalue, windex, payload)

    def _translating(self, windex: int) -> bool:
        return self.protocol in ("translate", "hybrid") and (windex >> 8) == 0

    def get(self, opcode: int, length: int, wvalue: int = 0, windex: int = VENDOR_IFACE) -> bytes:
        """Device->host vendor request.  Raises Stall if refused, Timeout if wedged."""
        if self._translating(windex):
            from . import legacy_map
            if self.protocol == "hybrid" and not self._hybrid_paged(opcode, "in", wvalue):
                return self.raw_get(opcode, length, wvalue)
            return legacy_map.translate_get(self, opcode, wvalue, length)
        return self.raw_get(opcode, length, wvalue, windex)

    def set(self, opcode: int, payload: bytes = b"", wvalue: int = 0, windex: int = VENDOR_IFACE) -> int:
        """Host->device vendor request.  Returns bytes accepted.  Raises Stall if refused."""
        if self._translating(windex):
            from . import legacy_map
            if self.protocol == "hybrid" and not self._hybrid_paged(opcode, "out", wvalue):
                return self.raw_set(opcode, bytes(payload), wvalue)
            return legacy_map.translate_set(self, opcode, wvalue, payload)
        return self.raw_set(opcode, payload, wvalue, windex)

    def _hybrid_paged(self, opcode: int, direction: str, wvalue: int) -> bool:
        """Hybrid routing: True translates, False sends the old request to page 0 unchanged."""
        from . import legacy_map
        name = _legacy_name(opcode) or f"0x{opcode:02X}"
        paged = legacy_map.routes_paged(self, opcode, direction, wvalue) or not self.page0_alive()
        if paged and _legacy_name(opcode) is None:
            raise UnmappedRequest(f"hybrid: old opcode 0x{opcode:02X} has no translation and this "
                                  f"firmware has no page 0 to send it to")
        self.note_hybrid(name, paged)
        return paged

    def page0_alive(self) -> bool:
        """True while page 0 still answers 1.1.6 opcodes (1.1.6 itself, and builds before the flip).
        The probe is the old GET_PLATFORM, a read on every firmware that has it."""
        if self._page0 is None:
            if self._paged_ok is None:
                self.protocol_info()
            if not self._paged_ok:
                self._page0 = True
            else:
                try:
                    self.raw_get(0x7F, 7)
                    self._page0 = True
                except Stall:
                    self._page0 = False
        return self._page0

    # -- hybrid bookkeeping --------------------------------------------------

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

    def note_hybrid(self, name: str, translated: bool):
        bucket = self.hybrid_counts["translated" if translated else "passed"]
        bucket[name] = bucket.get(name, 0) + 1
        self._register_hybrid_summary()

    def hybrid_summary(self) -> str:
        t, p = self.hybrid_counts["translated"], self.hybrid_counts["passed"]
        return (f"hybrid: {len(t)} opcodes translated ({sum(t.values())} calls), {len(p)} passed through "
                f"({sum(p.values())} calls); translated: {', '.join(sorted(t)) or 'none'}")

    def _register_hybrid_summary(self):
        if not self._hybrid_atexit:
            self._hybrid_atexit = True
            atexit.register(lambda: (self.hybrid_counts["translated"] or self.hybrid_counts["passed"])
                            and print(self.hybrid_summary()))

    # -- paged commands ------------------------------------------------------

    def resolve(self, cmd) -> CommandInfo:
        return resolve_command(cmd, self.registry)

    def _require_paged(self):
        # 1.1.6 ignores wIndex and would run a paged command as the old opcode
        # with the same number (0x53 is FACTORY_RESET), so probe first (plan 5.14).
        if self._paged_ok is None:
            self.protocol_info()
        if not self._paged_ok:
            raise ProtocolError("device does not answer SYS.PROTOCOL_INFO; refusing paged requests")

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
        Use ONLY for reads with no invalid-param path of their own. Never use it
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
        """Poll GET_PLATFORM until it answers cleanly. A deferred, mute-bracketed pipeline
        reset (type switch, pin move, bulk apply, input switch, preset load, factory reset)
        briefly disables the USB control IRQ, so a transfer then stalls or times out. Only a
        device that never recovers (a real crash or hang) reports failure.
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
