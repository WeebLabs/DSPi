#!/usr/bin/env python3
"""
dspi_poke.py: send one paged vendor request to a connected DSPi and show the reply.
Run `dspi_poke.py help` for the usage guide. Needs pyusb (and libusb on macOS).
"""

import argparse
import json
import struct
import sys
from pathlib import Path

try:
    import usb.core
except ImportError:
    sys.exit("pyusb not installed.  Run: pip install pyusb  (macOS also needs `brew install libusb`)")


VID, PID = 0x2E8B, 0xFEAA
VENDOR_IFACE = 2
BM_VENDOR_IN = 0xC1    # device -> host, vendor, interface
BM_VENDOR_OUT = 0x41   # host -> device, vendor, interface

SYS_PAGE = 0x01
CMD_PROTOCOL_INFO, CMD_PAGE_LIST, CMD_PAGE_COMMANDS = 0x10, 0x11, 0x12
CMD_COMMAND_INFO, CMD_LAST_RESULT = 0x13, 0x14
CMD_PLATFORM, CMD_BUILD_INFO = 0x20, 0x22
OLD_GET_PLATFORM = 0x7F   # 1.1.6 and older only; never send it to paged firmware
MAX_BULK_GET = 8192       # wLength for a variable bulk reply; the device sends what it has

VC_RESULTS = {
    0x00: ("VC_OK", "done, or accepted for deferred work"),
    0x10: ("VC_ERR_UNKNOWN", "no such page or command on this build"),
    0x11: ("VC_ERR_DIRECTION", "command has no handler for this direction"),
    0x12: ("VC_ERR_LENGTH", "SET payload length outside the allowed range"),
    0x13: ("VC_ERR_ADDRESS", "wValue out of range"),
    0x14: ("VC_ERR_VALUE", "a payload field is out of range"),
    0x15: ("VC_ERR_UNSUPPORTED", "not available on this platform"),
    0x16: ("VC_ERR_BLOCKED", "not allowed on this transport"),
    0x17: ("VC_ERR_BUSY", "try again shortly"),
    0x18: ("VC_ERR_STATE", "not allowed in the current state"),
    0x19: ("VC_ERR_PIN_INVALID", "GPIO not usable for this function"),
    0x1A: ("VC_ERR_PIN_IN_USE", "GPIO already claimed"),
}

# SYS.COMMAND_INFO encodings. Order and bits must match VcAddrKind and VC_F_* in vc_core.h.
ADDR_KINDS = (
    "NONE", "CHANNEL", "INPUT", "OUTPUT", "OUTPUT_ALL", "SLOT", "PIN_OUTPUT",
    "PRESET", "CH_BAND", "IN_OUT", "SOURCE", "SPDIF_RX", "I2S_PAIR", "ROLE",
    "CS_SLOT", "CS_SUB", "CS_GROUP", "CS_MACRO", "CS_MACRO_STEP", "CS_PAGE",
    "CS_NOUN", "PAGE", "PAGE_CMD", "OFFSET", "INDEX",
)
FLAG_BITS = ((0x0001, "USB_ONLY"), (0x0002, "SET_USB_ONLY"), (0x0004, "BULK"),
             (0x0008, "DEFERRED"), (0x0010, "FLASH"), (0x0020, "RESTART"), (0x0040, "KEEPALIVE"))

REGISTRY_REL = Path("Documentation") / "protocol" / "dspi_commands.json"


# --- Registry ---------------------------------------------------------------

class Registry:
    """Pages and commands from dspi_commands.json; empty when the file is missing."""

    def __init__(self, path=None):
        self.path = Path(path) if path else self._find()
        self.pages = {}      # page number -> page dict
        self.by_name = {}    # "PAGE.NAME" -> (page dict, command dict)
        self.by_id = {}      # 16-bit ID -> (page dict, command dict)
        if self.path is None:
            return
        try:
            doc = json.loads(self.path.read_text())
        except (OSError, ValueError) as e:
            sys.exit(f"cannot read the command registry {self.path}: {e}")
        for pg in doc.get("pages", []):
            self.pages[pg["page"]] = pg
            for c in pg["commands"]:
                self.by_name[f"{pg['mnemonic']}.{c['name']}"] = (pg, c)
                self.by_id[(pg["page"] << 8) | c["cmd"]] = (pg, c)

    @staticmethod
    def _find():
        for start in (Path(__file__).resolve().parent, Path.cwd().resolve()):
            for parent in (start, *start.parents):
                if (parent / REGISTRY_REL).is_file():
                    return parent / REGISTRY_REL
        return None

    def need(self, what):
        if self.path is None:
            sys.exit(f"{what} needs {REGISTRY_REL}, which was not found next to this script or the "
                     f"working directory.  Pass --registry PATH, or address commands numerically "
                     f"(page.cmd such as 0x05.0x21, or a 16-bit ID such as 0x0521).")

    def page_by_token(self, tok):
        """A page mnemonic or number, or None when the token is neither."""
        for num, pg in self.pages.items():
            if pg["mnemonic"] == tok.upper():
                return num
        try:
            n = int(tok, 0)
        except ValueError:
            return None
        return n if 0 <= n <= 0xFF else None

    def mnemonic(self, page):
        pg = self.pages.get(page)
        return pg["mnemonic"] if pg else f"0x{page:02X}"


class Command:
    def __init__(self, page, cmd, reg):
        self.page, self.cmd = page, cmd
        self.id = (page << 8) | cmd
        found = reg.by_id.get(self.id)
        self.entry = found[1] if found else None
        self.name = f"{found[0]['mnemonic']}.{found[1]['name']}" if found else \
            f"{reg.mnemonic(page)}.0x{cmd:02X}"

    def label(self):
        return f"{self.name} (0x{self.id:04X})"


def parse_num(tok, what, hi):
    try:
        n = int(tok, 0)
    except ValueError:
        raise SystemExit(f"bad {what} '{tok}'")
    if not 0 <= n <= hi:
        raise SystemExit(f"{what} out of range: {tok} (expect 0..0x{hi:X})")
    return n


def resolve(tok, reg, page_opt=None):
    """PAGE.NAME, page.cmd (numbers or mnemonic), a bare NAME, or a number (ID, or cmd with --page)."""
    t = tok.strip().upper().replace("-", "_")
    if "." in t:
        left, right = t.split(".", 1)
        page = reg.page_by_token(left)
        if page is None:
            reg.need(f"page '{left}'")
            raise SystemExit(f"unknown page '{left}'.  Run `dspi_poke.py list` for the pages.")
        try:
            return Command(page, parse_num(right, "command number", 0xFF), reg)
        except SystemExit:
            pass
        reg.need(f"command name '{tok}'")
        if t in reg.by_name:
            pg, c = reg.by_name[t]
            return Command(pg["page"], c["cmd"], reg)
        raise SystemExit(f"unknown command '{tok}'{suggest(reg, right)}")
    try:
        n = int(t, 0)
    except ValueError:
        n = None
    if n is not None:
        if page_opt is not None:
            return Command(page_opt, parse_num(t, "command number", 0xFF), reg)
        n = parse_num(t, "command ID", 0xFFFF)
        if n >> 8 == 0:
            raise SystemExit(f"'{tok}' is on page 0, which has no commands.  Give page.cmd "
                             f"(0x05.0x21), a 16-bit ID (0x0521), or --page with a command byte.")
        return Command(n >> 8, n & 0xFF, reg)
    reg.need(f"command name '{tok}'")
    hits = [k for k in reg.by_name if k.split(".", 1)[1] == t]
    if len(hits) == 1:
        pg, c = reg.by_name[hits[0]]
        return Command(pg["page"], c["cmd"], reg)
    if hits:
        raise SystemExit(f"'{tok}' is ambiguous: {', '.join(hits)}")
    raise SystemExit(f"unknown command '{tok}'{suggest(reg, t)}")


def suggest(reg, stem):
    cands = [k for k in reg.by_name if stem in k]
    if not cands and len(stem) >= 4:
        cands = [k for k in reg.by_name if stem[:4] in k]
    return ("\n  did you mean: " + ", ".join(cands[:8])) if cands else ""


def fmt_get_len(c):
    if not c["get"]:
        return "-"
    return "var" if c["get_len"] == "var" else str(c["get_len"])


def fmt_set_len(c):
    if not c["set"]:
        return "-"
    if "BULK" in c["flags"]:
        return "bulk"
    lo, hi = c["set_min"], c["set_max"]
    return str(lo) if lo == hi else f"{lo}-{hi}"


# --- Payload builders / response decoders -----------------------------------

PACK = {
    "u8": ("<B", 1), "u16": ("<H", 2), "u32": ("<I", 4),
    "i8": ("<b", 1), "i16": ("<h", 2), "i32": ("<i", 4),
    "float": ("<f", 4),
}


def build_payload(args):
    sources = []
    for name, (fmt, _size) in PACK.items():
        vals = getattr(args, name, None)
        if vals is not None:
            try:
                sources.append(("--" + name, b"".join(struct.pack(fmt, v) for v in vals)))
            except struct.error as e:
                raise SystemExit(f"--{name}: {e}")
    if args.hex is not None:
        sources.append(("--hex", parse_hex_tokens(args.hex.split())))
    if args.string is not None:
        sources.append(("--string", args.string.encode("utf-8")))
    if args.bytes:
        sources.append(("positional bytes", parse_hex_tokens(args.bytes)))
    if len(sources) > 1:
        raise SystemExit(f"choose only one payload source (got: {', '.join(s[0] for s in sources)})")
    return sources[0][1] if sources else b""


def parse_hex_tokens(tokens):
    joined = "".join(tokens).replace("0x", "").replace("0X", "").replace(" ", "")
    if len(joined) % 2:
        raise SystemExit(f"hex payload must have an even nibble count: '{' '.join(tokens)}'")
    try:
        return bytes.fromhex(joined)
    except ValueError as e:
        raise SystemExit(f"bad hex in payload: {e}")


def decode_response(buf, kind):
    if kind in (None, "hex"):
        return hexdump(buf)
    if kind == "string":
        trimmed = buf.split(b"\x00", 1)[0]
        try:
            return "  " + repr(trimmed.decode("utf-8"))
        except UnicodeDecodeError:
            return "  " + repr(trimmed)
    fmt, size = PACK[kind]
    if len(buf) < size:
        raise SystemExit(f"--as {kind} needs {size} bytes, got {len(buf)}")
    vals = [struct.unpack_from(fmt, buf, o)[0] for o in range(0, len(buf) - size + 1, size)]
    out = [f"{v:.6g}" if kind == "float" else str(v) for v in vals]
    tail = f"  (+{len(buf) % size} trailing byte(s))" if len(buf) % size else ""
    return "  " + (out[0] if len(out) == 1 else "[" + ", ".join(out) + "]") + tail


def hexdump(data):
    if not data:
        return "  (empty)"
    lines = []
    for row in range(0, len(data), 16):
        chunk = data[row:row + 16]
        hex_ = " ".join(f"{b:02X}" for b in chunk)
        ascii_ = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        lines.append(f"  [{row:04X}]  {hex_:<47}  {ascii_}")
    return "\n".join(lines)


# --- Device I/O -------------------------------------------------------------

def is_stall(e):
    return getattr(e, "errno", None) == 32 or "pipe" in str(e).lower()


class Device:
    def __init__(self, timeout_ms):
        self.dev = usb.core.find(idVendor=VID, idProduct=PID)
        if self.dev is None:
            sys.exit(f"no DSPi found at VID:PID {VID:04X}:{PID:04X}.  Is it plugged in?")
        self.timeout = timeout_ms
        self.paged = False

    def get(self, page, cmd, addr, length, windex=None):
        wi = ((page << 8) | VENDOR_IFACE) if windex is None else windex
        return bytes(self.dev.ctrl_transfer(BM_VENDOR_IN, cmd, addr, wi, length, timeout=self.timeout))

    def set(self, page, cmd, addr, payload, windex=None):
        wi = ((page << 8) | VENDOR_IFACE) if windex is None else windex
        return self.dev.ctrl_transfer(BM_VENDOR_OUT, cmd, addr, wi, payload, timeout=self.timeout)

    def probe(self, allow_old):
        """SYS.PROTOCOL_INFO first: older firmware ignores the page and would run whatever
        1.1.6 opcode shares the command number (plan 5.14). Returns the parsed fields or None."""
        try:
            r = self.get(SYS_PAGE, CMD_PROTOCOL_INFO, 0, 16)
        except usb.core.USBError as e:
            if not is_stall(e):
                sys.exit(f"USB error on the SYS.PROTOCOL_INFO probe: {e}")
            return self._old_firmware(allow_old, "STALLed the SYS.PROTOCOL_INFO probe")
        if len(r) < 16 or r[:4] != b"DSPi":
            return self._old_firmware(allow_old, f"answered SYS.PROTOCOL_INFO without the DSPi "
                                                 f"magic ({r.hex(' ')})")
        major, minor, pages, flags, max_set, max_get = struct.unpack_from("<BBBBHH", r, 4)
        self.paged = True
        return {"major": major, "minor": minor, "pages": pages, "flags": flags,
                "max_set": max_set, "max_get": max_get}

    def _old_firmware(self, allow_old, why):
        ver = ""
        try:
            p = self.get(0, OLD_GET_PLATFORM, 0, 7, windex=VENDOR_IFACE)
            if len(p) >= 6:
                ver = f" Its GET_PLATFORM reports v{p[1]}.{p[4]}.{p[5]} ({p.hex(' ')})."
        except usb.core.USBError:
            pass
        msg = f"the device {why}, so it does not speak the paged protocol (firmware 1.1.6 or older).{ver}"
        if not allow_old:
            sys.exit(msg + "\nRefusing to send: old firmware ignores the page and could run an "
                           "unrelated opcode, factory reset included.  Pass --allow-old-firmware "
                           "only if you know what that opcode does there.")
        print(f"warning: {msg} Sending anyway (--allow-old-firmware).", file=sys.stderr)
        return None

    def explain_stall(self, direction, cmd_id, addr):
        """The VC_* reason from SYS.LAST_RESULT, when it describes this request."""
        if not self.paged:
            return "no SYS.LAST_RESULT on this firmware"
        try:
            r = self.get(SYS_PAGE, CMD_LAST_RESULT, 0, 8)
        except usb.core.USBError as e:
            return f"SYS.LAST_RESULT could not be read: {e}"
        rid, is_get, res, raddr = struct.unpack_from("<HBBH", r)
        rdir = "GET" if is_get else "SET"
        if (rid, rdir, raddr) != (cmd_id, direction, addr):
            return (f"SYS.LAST_RESULT describes another request ({rdir} 0x{rid:04X} wValue "
                    f"0x{raddr:04X}), perhaps from another host, so the reason is unknown")
        name, meaning = VC_RESULTS.get(res, (f"0x{res:02X}", "unknown result code"))
        return f"{name} (0x{res:02X}): {meaning}"


def open_device(args):
    dev = Device(args.timeout)
    info = dev.probe(args.allow_old_firmware)
    return dev, info


def target_of(args, c):
    """(page, wIndex) actually sent: the command's page, unless --windex overrides it."""
    if args.windex is None:
        return c.page, (c.page << 8) | VENDOR_IFACE
    return args.windex >> 8, args.windex


def addr_note(c):
    return f" ({c.entry['addr']})" if c.entry and c.entry["addr"] != "NONE" else ""


# --- Subcommands ------------------------------------------------------------

def do_get(args):
    reg = Registry(args.registry)
    c = resolve(args.request, reg, args.page)
    e = c.entry
    if e and not e["get"] and not args.no_check:
        sys.exit(f"{c.name} has no GET handler (it is SET only).  Use `set`, or --no-check to "
                 f"send the GET anyway.")
    dev, info = open_device(args)
    length = args.length
    if length is None:
        if e and isinstance(e.get("get_len"), int) and e["get_len"] > 0:
            length = e["get_len"]
        elif e and "BULK" in e["flags"]:
            length = MAX_BULK_GET
        else:
            length = info["max_get"] if info else 256
    page, windex = target_of(args, c)
    cmd_id = (page << 8) | c.cmd
    print(f"GET {c.label()}  wValue=0x{args.addr:04X}{addr_note(c)}  wIndex=0x{windex:04X}  "
          f"length={length}")
    try:
        buf = dev.get(page, c.cmd, args.addr, length, windex)
    except usb.core.USBError as ex:
        if is_stall(ex):
            sys.exit(f"  STALL: {dev.explain_stall('GET', cmd_id, args.addr)}")
        sys.exit(f"  USB error: {ex}")
    print(f"  received {len(buf)} byte(s):")
    print(decode_response(buf, args.decode))


def do_set(args):
    reg = Registry(args.registry)
    c = resolve(args.request, reg, args.page)
    e = c.entry
    payload = build_payload(args)
    if e and not args.no_check:
        if not e["set"]:
            sys.exit(f"{c.name} has no SET handler (it is GET only).  Use `get`, or --no-check to "
                     f"send the SET anyway.")
        if "BULK" not in e["flags"] and not e["set_min"] <= len(payload) <= e["set_max"]:
            sys.exit(f"{c.name} takes a {fmt_set_len(e)} byte SET payload, got {len(payload)}.  "
                     f"Pass --no-check to send it anyway.")
    dev, _info = open_device(args)
    page, windex = target_of(args, c)
    cmd_id = (page << 8) | c.cmd
    print(f"SET {c.label()}  wValue=0x{args.addr:04X}{addr_note(c)}  wIndex=0x{windex:04X}  "
          f"payload={len(payload)} byte(s)")
    if payload:
        print(hexdump(payload))
    try:
        sent = dev.set(page, c.cmd, args.addr, payload, windex)
    except usb.core.USBError as ex:
        if is_stall(ex):
            sys.exit(f"  STALL: {dev.explain_stall('SET', cmd_id, args.addr)}")
        sys.exit(f"  USB error: {ex}")
    note = ""
    if e and "DEFERRED" in e["flags"]:
        note = "  Deferred: the main loop finishes the work, so read it back to confirm."
    print(f"  device accepted {sent} byte(s).{note}")


def do_info(args):
    reg = Registry(args.registry)
    if args.target is None:
        return info_device(args, reg)
    tok = args.target.strip()
    page = None if "." in tok else reg.page_by_token(tok)
    if page is not None:
        return info_page(args, reg, page)
    return info_command(args, reg, resolve(args.target, reg))


def info_device(args, reg):
    dev, info = open_device(args)
    if info is None:
        return
    print(f"protocol {info['major']}.{info['minor']}, {info['pages']} page(s), flags 0x{info['flags']:02X}, "
          f"largest SET {info['max_set']} B, largest GET {info['max_get']} B")
    try:
        p = dev.get(SYS_PAGE, CMD_PLATFORM, 0, 7)
        b = dev.get(SYS_PAGE, CMD_BUILD_INFO, 0, 64)
        beta = f"-beta{p[6]}" if p[6] else ""
        plat = {0: "RP2040", 1: "RP2350"}.get(p[0], f"platform {p[0]}")
        desc, date = (x.split(b"\0", 1)[0].decode(errors="replace") for x in (b[:48], b[48:]))
        print(f"firmware v{p[1]}.{p[4]}.{p[5]}{beta} on {plat}, {p[3]} outputs, build {desc} {date}")
        pl = dev.get(SYS_PAGE, CMD_PAGE_LIST, 0, 256)
    except usb.core.USBError as e:
        sys.exit(f"USB error reading identity: {e}")
    seen = set()
    print("pages on the device:")
    for page, rev in zip(pl[0::2], pl[1::2]):
        seen.add(page)
        pg = reg.pages.get(page)
        if pg is None:
            note = "not in the registry"
        elif pg["revision"] != rev:
            note = f"registry has revision {pg['revision']}"
        else:
            note = pg["doc"]
        print(f"  0x{page:02X}  {reg.mnemonic(page):<7} rev {rev:<3} {note}")
    for page in sorted(set(reg.pages) - seen):
        print(f"  0x{page:02X}  {reg.mnemonic(page):<7} absent on this build (registry rev "
              f"{reg.pages[page]['revision']})")


def info_page(args, reg, page):
    dev, info = open_device(args)
    if info is None:
        return
    try:
        bm = dev.get(SYS_PAGE, CMD_PAGE_COMMANDS, page, 64)
    except usb.core.USBError as e:
        if is_stall(e):
            sys.exit(f"SYS.PAGE_COMMANDS for page 0x{page:02X} STALLed: "
                     f"{dev.explain_stall('GET', (SYS_PAGE << 8) | CMD_PAGE_COMMANDS, page)}"
                     f" (the page is not on this build)")
        sys.exit(f"USB error: {e}")
    has = lambda half, n: bool(bm[half + n // 8] >> (n % 8) & 1)
    print(f"page 0x{page:02X} {reg.mnemonic(page)} on the device:")
    pg = reg.pages.get(page, {"commands": []})
    known = {c["cmd"]: c for c in pg["commands"]}
    for n in range(256):
        g, s = has(0, n), has(32, n)
        c = known.get(n)
        if not (g or s or c):
            continue
        dirs = ("G" if g else "-") + ("S" if s else "-")
        if c is None:
            note = "not in the registry"
        elif not (g or s):
            note = f"absent on this build (build: {c['build']})" if c["build"] else "absent on this build"
        elif (g, s) != (c["get"], c["set"]):
            note = "directions differ from the registry"
        else:
            note = ""
        name = c["name"] if c else f"0x{n:02X}"
        print(f"  0x{n:02X}  {dirs}  {name:<26} {note}".rstrip())


def info_command(args, reg, c):
    dev, info = open_device(args)
    if info is None:
        return
    ci = (SYS_PAGE << 8) | CMD_COMMAND_INFO
    try:
        r = dev.get(SYS_PAGE, CMD_COMMAND_INFO, c.id, 8)
    except usb.core.USBError as e:
        if is_stall(e):
            sys.exit(f"SYS.COMMAND_INFO for {c.label()} STALLed: {dev.explain_stall('GET', ci, c.id)}"
                     f" (not on this build)")
        sys.exit(f"USB error: {e}")
    flags, get_len, set_min, set_max, kind, dirs = struct.unpack_from("<HHBBBB", r)
    dev_e = {
        "get": bool(dirs & 1), "set": bool(dirs & 2),
        "addr": ADDR_KINDS[kind] if kind < len(ADDR_KINDS) else f"kind {kind}",
        "get_len": "var" if get_len == 0xFFFF else (get_len if dirs & 1 else None),
        "set_min": set_min, "set_max": set_max,
        "flags": [n for bit, n in FLAG_BITS if flags & bit] + (
            [f"0x{flags & ~0x7F:04X}"] if flags & ~0x7F else []),
    }
    print(f"{c.label()} on the device:")
    print(f"  directions  {' '.join(d for d, on in (('GET', dev_e['get']), ('SET', dev_e['set'])) if on)}")
    print(f"  address     {dev_e['addr']}")
    print(f"  GET length  {fmt_get_len(dev_e)}")
    print(f"  SET length  {fmt_set_len(dev_e)}")
    print(f"  flags       {', '.join(dev_e['flags']) or 'none'}")
    e = c.entry
    if e is None:
        print("  (not in the registry)")
        return
    if e.get("doc"):
        print(f"  doc         {e['doc']}")
    diffs = [k for k in ("get", "set", "addr", "get_len", "flags")
             if e[k] != dev_e[k]] + [k for k in ("set_min", "set_max")
                                     if e["set"] and e[k] != dev_e[k]]
    if diffs:
        print(f"  differs from the registry in: {', '.join(diffs)}")


def do_list(args):
    reg = Registry(args.registry)
    reg.need("list")
    needle = (args.filter or "").upper().replace("-", "_")
    total = 0
    for num in sorted(reg.pages):
        pg = reg.pages[num]
        rows = [c for c in pg["commands"] if needle in f"{pg['mnemonic']}.{c['name']}"]
        if not rows:
            continue
        print(f"0x{num:02X} {pg['mnemonic']}  rev {pg['revision']}  {pg['doc']}")
        width = max(len(c["name"]) for c in rows) + len(pg["mnemonic"]) + 1
        for c in rows:
            dirs = ("G" if c["get"] else "-") + ("S" if c["set"] else "-")
            extra = ", ".join(c["flags"] + ([f"build {c['build']}"] if c["build"] else []))
            extra = f"  [{extra}]" if extra else ""
            name = f"{pg['mnemonic']}.{c['name']}"
            print(f"  0x{(num << 8) | c['cmd']:04X}  {name:<{width}}  {dirs}  {c['addr']:<13} "
                  f"get {fmt_get_len(c):<4} set {fmt_set_len(c):<5} {c['doc']}{extra}")
            total += 1
        print()
    if not total:
        sys.exit(f"no commands match '{args.filter}'.")
    print(f"{total} command(s) shown, from {reg.path}")


HELP_TEXT = """\
dspi_poke.py: send one vendor request to a DSPi
================================================

WHAT IT IS
  A thin wrapper around one USB control transfer.  It lets you GET or SET any
  command of the paged vendor protocol (firmware 1.1.7 and later) without
  writing Console UI for it.

THE REQUEST
  Every command is PAGE.NAME, for example ROUTE.OUTPUT_GAIN.  Its 16-bit ID is
  (page << 8) | cmd, here 0x0521.  On the wire:
      bmRequestType  0xC1 for GET, 0x41 for SET
      bRequest       the command number inside the page (0x10 to 0xFF)
      wIndex         (page << 8) | 2.  Set from the page; --windex overrides it
      wValue         the address: which output, channel, band or slot (--addr)
      payload        the value, little-endian
  A refused request STALLs.  The tool then reads SYS.LAST_RESULT and prints the
  reason, for example VC_ERR_ADDRESS.

NAMING A COMMAND
      ROUTE.OUTPUT_GAIN      page and name (case-insensitive)
      output_gain            a name alone, when only one page has it
      route.0x21  5.33       page and command number, by mnemonic or number
      0x0521                 the 16-bit ID
      0x21 --page 5          a command byte with --page
  Names come from Documentation/protocol/dspi_commands.json, found by walking
  up from this script or the working directory.  --registry PATH points
  elsewhere.  Numeric forms work without the file.

SUBCOMMANDS
  list [filter]          every page and command in the registry, with
                         direction (G, S), address kind and sizes
  info                   protocol version, firmware identity and the pages on
                         the device, checked against the registry
  info PAGE              the commands the device implements on one page
  info CMD               SYS.COMMAND_INFO for one command, checked against the
                         registry
  get CMD [N]            read N bytes (default: the registry length)
  set CMD [payload]      write a payload (zero bytes for an action)

PAYLOAD (for set; pick one source)
      set CMD 01 02 03 04          positional hex bytes
      set CMD --hex "01 02 03 04"  quoted hex string
      set CMD --u8 5               little-endian integers: --u8 --u16 --u32
                                   --i8 --i16 --i32, one or more values
      set CMD --float -12.5 0.5    little-endian float32, one or more values
      set CMD --string "Desk"      UTF-8, no NUL terminator
      set CMD                      empty payload, for an action
  The payload length is checked against the registry.  --no-check sends it
  anyway, and also lets you send a direction the command does not have.

REPLY (for get)
      get CMD                      hex dump (default)
      get CMD --as float           every whole float32 in the reply
      get CMD --as u8 / u16 / u32 / i8 / i16 / i32
      get CMD --as string          bytes up to the first NUL, as UTF-8

EXAMPLES
      dspi_poke.py list route
      dspi_poke.py info
      dspi_poke.py get SYS.PLATFORM
      dspi_poke.py get ROUTE.OUTPUT_GAIN --addr 0 --as float
      dspi_poke.py set ROUTE.OUTPUT_GAIN --addr 0 --float -3.0
      dspi_poke.py set MEAS.CLIP_CLEAR
      dspi_poke.py info ROUTE.OUTPUT_GAIN

OLD FIRMWARE
  Before anything else the tool sends GET SYS.PROTOCOL_INFO.  Firmware 1.1.6
  and older STALL it, and the tool stops there.  Such firmware ignores the
  page, so a paged command would run whatever old opcode shares its number
  (0x53 was factory reset).  --allow-old-firmware sends the request anyway.

TROUBLESHOOTING
  "no DSPi found"
      Unplug and replug.  Linux may need a udev rule or sudo.
  "STALL: VC_ERR_..."
      The firmware refused the request and says why.  The handlers live in
      firmware/DSPi/vendor/vc_page_<page>.c and the reference is
      Documentation/commands.md.
  "STALL: SYS.LAST_RESULT describes another request"
      Another host (the Console, say) sent a request in between.  Retry.

SAFETY
  These transfers hit live state.  STORE.FACTORY_RESET really resets, and
  SYS.ENTER_BOOTLOADER really reboots into BOOTSEL.  Commands flagged FLASH
  write flash.  A wrong value at audio rate may click, but does no damage.
"""


def do_help(_args):
    print(HELP_TEXT, end="")


# --- Arg parsing ------------------------------------------------------------

def int_auto(s):
    return int(s, 0)


def u16_auto(s):
    n = int(s, 0)
    if not 0 <= n <= 0xFFFF:
        raise argparse.ArgumentTypeError(f"{s} does not fit 16 bits")
    return n


def make_parser():
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--registry", type=Path, default=None,
                        help="path to dspi_commands.json (found automatically inside the repo)")
    dev = argparse.ArgumentParser(add_help=False)
    dev.add_argument("--allow-old-firmware", action="store_true",
                     help="send even when SYS.PROTOCOL_INFO fails (old firmware ignores the page)")
    dev.add_argument("--timeout", type=int, default=1000, help="transfer timeout, ms (default 1000)")
    req = argparse.ArgumentParser(add_help=False)
    req.add_argument("request", help="PAGE.NAME, NAME, page.cmd, a 16-bit ID, or a cmd byte with --page")
    req.add_argument("--addr", "--wvalue", dest="addr", type=u16_auto, default=0,
                     help="wValue: the address (output, channel, (ch << 8) | band ...), default 0")
    req.add_argument("--page", type=int_auto, default=None,
                     help="page for a bare command byte")
    req.add_argument("--windex", type=u16_auto, default=None,
                     help="raw wIndex override for experiments (default (page << 8) | 2)")
    req.add_argument("--no-check", action="store_true",
                     help="skip the registry direction and SET length checks")

    p = argparse.ArgumentParser(
        prog="dspi_poke.py",
        description="Send one paged vendor request to a DSPi.  `dspi_poke.py help` has the guide.")
    sub = p.add_subparsers(dest="cmd", required=True, metavar="COMMAND")

    sub.add_parser("help", help="long-form usage guide with examples").set_defaults(func=do_help)

    pl = sub.add_parser("list", parents=[common], help="list pages and commands from the registry")
    pl.add_argument("filter", nargs="?", help="substring of PAGE.NAME (case-insensitive)")
    pl.set_defaults(func=do_list)

    pi = sub.add_parser("info", parents=[common, dev],
                        help="protocol and pages, one page's commands, or one command's info")
    pi.add_argument("target", nargs="?", help="page (mnemonic or number) or command")
    pi.set_defaults(func=do_info)

    pg = sub.add_parser("get", parents=[common, dev, req], help="device to host request")
    pg.add_argument("length", type=int_auto, nargs="?", default=None,
                    help="bytes to read (default: the registry length)")
    pg.add_argument("--as", dest="decode",
                    choices=["hex", "u8", "u16", "u32", "i8", "i16", "i32", "float", "string"],
                    help="decode the reply (default: hex dump)")
    pg.set_defaults(func=do_get)

    ps = sub.add_parser("set", parents=[common, dev, req], help="host to device request")
    ps.add_argument("bytes", nargs="*", help="payload as raw hex bytes (01 02 0xff DE AD)")
    ps.add_argument("--hex", help="payload as one quoted hex string, \"01 80 ff 00\"")
    ps.add_argument("--string", help="payload as a UTF-8 string (no NUL terminator)")
    for name in PACK:
        ps.add_argument(f"--{name}", nargs="+", type=(float if name == "float" else int_auto),
                        help=f"payload as one or more little-endian {name} values")
    ps.set_defaults(func=do_set)
    return p


def main():
    args = make_parser().parse_args()
    if getattr(args, "page", None) is not None and not 1 <= args.page <= 0xFF:
        sys.exit("--page must be 1..0xFF (page 0 has no commands)")
    args.func(args)


if __name__ == "__main__":
    main()
