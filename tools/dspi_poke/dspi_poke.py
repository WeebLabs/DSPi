#!/usr/bin/env python3
"""
dspi_poke.py — Generic DSPi vendor-command poker.

Send arbitrary USB vendor control transfers to a connected DSPi.  Useful when
firmware exposes a new parameter that doesn't have a Console-app widget yet —
type the request opcode + payload here and iterate without writing UI.

Quick examples:

    # Long-form usage guide (anatomy of a request, payload encodings, etc.)
    dspi_poke.py help

    # See every REQ_ opcode the firmware exposes (parsed from config.h)
    dspi_poke.py list                       # full list with inline comments
    dspi_poke.py list preamp                # substring filter

    # GET — read N bytes back, optional --wvalue, optional decoding
    dspi_poke.py get REQ_GET_INPUT_SOURCE   1
    dspi_poke.py get 0xE1                   1   --as u8
    dspi_poke.py get REQ_GET_MASTER_VOLUME  4   --as float
    dspi_poke.py get REQ_GET_PREAMP_CH      4   --wvalue 0  --as float

    # SET — payload as raw hex bytes OR as a typed value
    dspi_poke.py set 0x42  01 02 03 04                      # raw hex bytes
    dspi_poke.py set REQ_SET_MASTER_VOLUME --float -12.5
    dspi_poke.py set REQ_SET_INPUT_SOURCE  --u8 1
    dspi_poke.py set REQ_SET_PREAMP_CH     --wvalue 0  --float -3.0
    dspi_poke.py set 0xAB  --hex "01 80 ff 00"              # quoted hex string
    dspi_poke.py set REQ_FACTORY_RESET                      # zero-length OUT

The opcode argument accepts:
  - hex literal: 0xAB, 0xab
  - decimal:     171
  - symbol:      REQ_SET_PREAMP, or just "set_preamp" (case-insensitive,
                 REQ_ prefix optional, underscores can be hyphens)

Install:
    pip install pyusb
    # macOS also needs:  brew install libusb
"""

import argparse
import os
import re
import struct
import sys
from pathlib import Path

try:
    import usb.core, usb.util
except ImportError:
    sys.exit("pyusb not installed.  Run: pip install pyusb  (macOS also needs `brew install libusb`)")


# --- USB constants ----------------------------------------------------------

VID, PID = 0x2E8B, 0xFEAA
VENDOR_IFACE = 2

# bmRequestType: vendor type | interface recipient
BM_VENDOR_IN  = 0xC1   # device -> host
BM_VENDOR_OUT = 0x41   # host   -> device


# --- Opcode table loaded from config.h --------------------------------------

# Map of canonical name (e.g. "REQ_SET_PREAMP") -> (opcode int, trailing comment or "")
OPCODES = {}

def _find_config_h():
    """Walk up from this script and from cwd looking for firmware/DSPi/config.h.

    Lets the tool live anywhere under the repo (tools/dspi_poke/, scripts/, …)
    and still find the header without a hardcoded relative depth.
    """
    candidates = []
    rel = Path("firmware") / "DSPi" / "config.h"
    for start in (Path(__file__).resolve().parent, Path.cwd().resolve()):
        for parent in (start, *start.parents):
            candidates.append(parent / rel)
    return candidates


CONFIG_H_CANDIDATES = _find_config_h()


def load_opcodes():
    """Best-effort parse of `#define REQ_FOO 0xNN  // comment` lines.

    Failure is silent — symbolic names just won't resolve, raw hex still works.
    """
    pattern = re.compile(
        r"""^\s*\#define\s+
            (REQ_\w+)\s+
            (0[xX][0-9A-Fa-f]+|\d+)
            \s*(?://\s*(.+?))?\s*$""",
        re.VERBOSE,
    )
    for path in CONFIG_H_CANDIDATES:
        if not path.is_file():
            continue
        try:
            for line in path.read_text().splitlines():
                m = pattern.match(line)
                if not m:
                    continue
                name, value, comment = m.group(1), m.group(2), (m.group(3) or "").strip()
                OPCODES[name.upper()] = (int(value, 0), comment)
            return path
        except OSError:
            continue
    return None


def resolve_opcode(token):
    """Accept hex / decimal / symbolic name → return int opcode in [0,255]."""
    # Try numeric first (covers 0xAB, 0xab, 0XaB, 171, 0b…, 0o…).
    try:
        n = int(token, 0)
        if 0 <= n <= 0xFF:
            return n
        raise SystemExit(f"opcode out of range: {token} (expect 0..0xFF)")
    except ValueError:
        pass

    # Symbolic.  Allow "set_preamp", "REQ-SET-PREAMP", etc.
    norm = token.strip().upper().replace("-", "_")
    if not norm.startswith("REQ_"):
        norm = "REQ_" + norm
    if norm in OPCODES:
        return OPCODES[norm][0]

    # Helpful suggestion if user mistyped.  Match on the post-REQ_ stem,
    # then fall back to any substring (handles double-letter typos like
    # "LEVELER" vs the correct "LEVELLER").
    stem = norm[4:] if norm.startswith("REQ_") else norm
    candidates = [n for n in OPCODES if stem in n]
    if not candidates and len(stem) >= 4:
        prefix = stem[:4]
        candidates = [n for n in OPCODES if prefix in n]
    hint = ("\n  did you mean: " + ", ".join(candidates[:6])) if candidates else ""
    raise SystemExit(f"unknown opcode '{token}' (not numeric, not in config.h){hint}")


def opcode_label(opcode):
    """Reverse lookup: 0xE1 -> 'REQ_GET_INPUT_SOURCE' or '' if unknown."""
    for name, (val, _comment) in OPCODES.items():
        if val == opcode:
            return name
    return ""


# --- Payload builders / response decoders -----------------------------------

PACK = {
    "u8":    ("<B", 1),
    "u16":   ("<H", 2),
    "u32":   ("<I", 4),
    "i8":    ("<b", 1),
    "i16":   ("<h", 2),
    "i32":   ("<i", 4),
    "float": ("<f", 4),
}


def build_payload(args):
    """Assemble payload bytes from typed-value flags and/or positional hex bytes.

    Exactly one source is allowed.  Returns a bytes object (possibly empty).
    """
    sources = []

    # Typed scalars
    for name, (fmt, _size) in PACK.items():
        val = getattr(args, name, None)
        if val is not None:
            sources.append(("--" + name, struct.pack(fmt, val)))

    # --hex "01 80 ff"
    if args.hex is not None:
        sources.append(("--hex", parse_hex_string(args.hex)))

    # --string "hello"
    if args.string is not None:
        sources.append(("--string", args.string.encode("utf-8")))

    # Positional hex bytes (set 0x42 01 02 03 04)
    if args.bytes:
        sources.append(("positional bytes", parse_hex_tokens(args.bytes)))

    if len(sources) > 1:
        names = ", ".join(s[0] for s in sources)
        raise SystemExit(f"choose only one payload source (got: {names})")
    return sources[0][1] if sources else b""


def parse_hex_tokens(tokens):
    """['01','02','de','ad'] or ['01020dead'] -> bytes.  Mixed forms tolerated."""
    joined = "".join(tokens).replace("0x", "").replace("0X", "").replace(" ", "")
    if len(joined) % 2:
        raise SystemExit(f"hex payload must have an even nibble count: '{' '.join(tokens)}'")
    try:
        return bytes.fromhex(joined)
    except ValueError as e:
        raise SystemExit(f"bad hex in payload: {e}")


def parse_hex_string(s):
    return parse_hex_tokens(s.split())


def decode_response(buf, kind):
    if kind == "hex" or kind is None:
        return hexdump(buf)
    if kind == "string":
        # Strip trailing NULs and undecodable tails for friendliness.
        trimmed = buf.split(b"\x00", 1)[0]
        try:
            return repr(trimmed.decode("utf-8"))
        except UnicodeDecodeError:
            return repr(trimmed)
    fmt, size = PACK[kind]
    if len(buf) < size:
        raise SystemExit(f"--as {kind} needs {size} bytes, got {len(buf)}")
    value = struct.unpack(fmt, buf[:size])[0]
    if kind == "float":
        return f"{value:.6g}"
    return str(value)


def hexdump(data):
    if not data:
        return "(empty)"
    lines = []
    for row in range(0, len(data), 16):
        chunk = data[row:row + 16]
        hex_  = " ".join(f"{b:02X}" for b in chunk)
        ascii_ = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        lines.append(f"  [{row:04X}]  {hex_:<47}  {ascii_}")
    return "\n".join(lines)


# --- Device I/O -------------------------------------------------------------

def find_device():
    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        sys.exit(f"no DSPi found at VID:PID {VID:04X}:{PID:04X} — is it plugged in?")
    return dev


def do_get(args):
    dev = find_device()
    opcode = resolve_opcode(args.request)
    label  = opcode_label(opcode)
    label_s = f" ({label})" if label else ""
    try:
        buf = bytes(dev.ctrl_transfer(
            BM_VENDOR_IN, opcode, args.wvalue, args.windex, args.length, timeout=1000))
    except usb.core.USBError as e:
        sys.exit(f"USB error on GET 0x{opcode:02X}{label_s}: {e}")

    print(f"GET 0x{opcode:02X}{label_s}  wValue=0x{args.wvalue:04X}  "
          f"wIndex=0x{args.windex:04X}  length={args.length}")
    print(f"  received {len(buf)} byte(s):")
    print(decode_response(buf, args.decode))


def do_set(args):
    dev = find_device()
    opcode = resolve_opcode(args.request)
    label  = opcode_label(opcode)
    label_s = f" ({label})" if label else ""
    payload = build_payload(args)
    try:
        sent = dev.ctrl_transfer(
            BM_VENDOR_OUT, opcode, args.wvalue, args.windex, payload, timeout=1000)
    except usb.core.USBError as e:
        sys.exit(f"USB error on SET 0x{opcode:02X}{label_s}: {e}")

    print(f"SET 0x{opcode:02X}{label_s}  wValue=0x{args.wvalue:04X}  "
          f"wIndex=0x{args.windex:04X}  payload={len(payload)} byte(s)")
    if payload:
        print(hexdump(payload))
    print(f"  device acked {sent} byte(s).")


HELP_TEXT = """\
dspi_poke.py — DSPi vendor-command poker
=========================================

WHAT IT IS
  A thin host-side wrapper around USB control transfers to a connected DSPi.
  Lets you exercise any vendor request (GET or SET) by opcode + payload
  without writing Console-app UI for it.  Useful while developing new
  firmware parameters: opcode goes in config.h, command becomes pokeable.

REQUIREMENTS
  pip install pyusb       # plus `brew install libusb` on macOS
  A DSPi plugged in (VID 0x2E8B, PID 0xFEAA).

THE THREE SUBCOMMANDS
  list [filter]   Show every REQ_* opcode parsed from firmware/DSPi/config.h.
                  Optional substring filter (case-insensitive).
  get  <op> [N]   Read up to N bytes back from the device (default 64).
  set  <op> ...   Write a payload to the device.

ANATOMY OF A REQUEST
  Every vendor request carries four pieces of data:
      bRequest   - the opcode byte (e.g. 0xD2 = REQ_SET_MASTER_VOLUME)
      wValue     - 16-bit data field, often used as a channel/index selector
      wIndex     - 16-bit USB routing field; defaults to 2 (the vendor
                   interface number).  Almost no firmware command uses this
                   for data — leave it at the default unless docs say so.
      payload    - 0..N bytes of body, format determined by the firmware

NAMING THE OPCODE
  The <op> argument accepts any of:
      0xD2                  hex literal
      210                   decimal
      REQ_SET_MASTER_VOLUME canonical symbol from config.h
      set_master_volume     short form (case-insensitive, REQ_ optional)

ENCODING THE PAYLOAD (for `set`)
  Pick one source.  Mixing is rejected.
      set 0x42 01 02 03 04           positional hex bytes
      set 0x42 --hex "01 02 03 04"   quoted hex string
      set 0x42 --u8 5                little-endian uint8
      set 0x42 --u16 1024            little-endian uint16
      set 0x42 --u32 100000          little-endian uint32
      set 0x42 --i8 / --i16 / --i32  signed variants
      set 0x42 --float -12.5         little-endian IEEE-754 float32
      set 0x42 --string "hello"      UTF-8, no NUL terminator
      set 0x42                       empty payload (e.g. REQ_FACTORY_RESET)

DECODING THE RESPONSE (for `get`)
      get 0xE1 1                     raw hex dump (default)
      get 0xE1 1 --as u8             one byte as an unsigned int
      get 0xD3 4 --as float          four bytes as float32
      get 0x93 32 --as string        bytes up to first NUL, decoded UTF-8

COMMON WORKFLOWS
  Find an opcode you half-remember:
      dspi_poke.py list leveller

  Read a typed value:
      dspi_poke.py get REQ_GET_MASTER_VOLUME 4 --as float

  Set a per-channel value (wValue selects the channel):
      dspi_poke.py set REQ_SET_PREAMP_CH --wvalue 0 --float -3.0
      dspi_poke.py set REQ_SET_PREAMP_CH --wvalue 1 --float -3.0

  Fire a side-effect command with no payload:
      dspi_poke.py set REQ_CLEAR_CLIPS
      dspi_poke.py set REQ_FACTORY_RESET

  Use a raw opcode that doesn't have a symbol yet (e.g. WIP firmware):
      dspi_poke.py set 0xEE --u8 2
      dspi_poke.py get 0xEF 1 --as u8

PER-SUBCOMMAND DETAILS
  Each subcommand has its own --help:
      dspi_poke.py get --help
      dspi_poke.py set --help

TROUBLESHOOTING
  "no DSPi found"
      Unplug/replug.  On Linux you may need a udev rule or sudo.  On macOS,
      ensure another app (Console GUI, audio driver) isn't holding the
      device exclusively.
  "USB error ... Pipe error" / "STALL"
      The firmware refused the request.  Common causes: opcode doesn't
      exist, wrong direction (sent GET to a SET opcode or vice versa),
      wrong payload length, wValue out of range.  Check the corresponding
      case in firmware/DSPi/vendor_commands.c.
  "unknown opcode 'FOO'"
      Symbol isn't in config.h.  Either pass the raw hex, or run
      `dspi_poke.py list <substring>` to find the right name.  Hint
      candidates print automatically for near-matches.
  "no opcodes loaded — config.h not found"
      Running outside the repo.  Use `--config-h /path/to/config.h` (placed
      *before* the subcommand: `dspi_poke.py --config-h <path> list`).

SAFETY NOTES
  These transfers hit live state.  Setting REQ_FACTORY_RESET really
  factory-resets.  REQ_ENTER_BOOTLOADER (0xF0) really enters BOOTSEL.
  Wrong payloads at audio rate may cause a click but not damage.
"""


def do_help(args):
    print(HELP_TEXT, end="")


def do_list(args):
    if not OPCODES:
        sys.exit("no opcodes loaded — config.h not found.  Pass --config-h <path> "
                 "or run from the DSPi repo root.")
    needle = (args.filter or "").upper()
    rows = sorted(OPCODES.items(), key=lambda kv: kv[1][0])
    matched = [(n, v, c) for n, (v, c) in rows if needle in n]
    if not matched:
        sys.exit(f"no opcodes match '{args.filter}'.")
    width = max(len(n) for n, _, _ in matched)
    for name, val, comment in matched:
        c = f"  // {comment}" if comment else ""
        print(f"  0x{val:02X}   {name:<{width}}{c}")
    print(f"\n{len(matched)} opcode(s) shown.")


# --- Arg parsing ------------------------------------------------------------

def int_auto(s):
    """argparse type that accepts 0xAB / 171 / 0b… etc."""
    return int(s, 0)


def make_parser():
    p = argparse.ArgumentParser(
        prog="dspi_poke.py",
        description="Generic DSPi vendor-command poker (GET/SET arbitrary requests).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("Install:", 1)[0],   # keep examples in --help
    )
    p.add_argument("--config-h", type=Path, default=None,
                   help="path to firmware/DSPi/config.h for symbolic opcode names "
                        "(auto-detected if running inside the repo)")

    sub = p.add_subparsers(dest="cmd", required=True, metavar="COMMAND")

    # --- help
    ph = sub.add_parser("help", help="long-form usage guide with examples")
    ph.set_defaults(func=do_help)

    # --- list
    pl = sub.add_parser("list", help="list known REQ_ opcodes from config.h")
    pl.add_argument("filter", nargs="?", help="substring to filter by (case-insensitive)")
    pl.set_defaults(func=do_list)

    # --- get
    pg = sub.add_parser("get", help="device->host vendor request")
    pg.add_argument("request", help="opcode: 0xAB / 171 / REQ_GET_X / get_x")
    pg.add_argument("length", type=int_auto, nargs="?", default=64,
                    help="bytes to read (default 64; firmware decides what it actually returns)")
    pg.add_argument("--wvalue", type=int_auto, default=0, help="USB wValue (default 0)")
    pg.add_argument("--windex", type=int_auto, default=VENDOR_IFACE,
                    help=f"USB wIndex (default {VENDOR_IFACE} = vendor interface; "
                         "override only if firmware uses the high byte for data)")
    pg.add_argument("--as", dest="decode",
                    choices=["hex", "u8", "u16", "u32", "i8", "i16", "i32", "float", "string"],
                    help="decode response (default: hex dump)")
    pg.set_defaults(func=do_get)

    # --- set
    ps = sub.add_parser("set", help="host->device vendor request")
    ps.add_argument("request", help="opcode: 0xAB / 171 / REQ_SET_X / set_x")
    ps.add_argument("bytes", nargs="*",
                    help="payload as raw hex bytes (e.g. 01 02 0xff DE AD)")
    ps.add_argument("--wvalue", type=int_auto, default=0, help="USB wValue (default 0)")
    ps.add_argument("--windex", type=int_auto, default=VENDOR_IFACE,
                    help=f"USB wIndex (default {VENDOR_IFACE} = vendor interface)")
    ps.add_argument("--hex", help="payload as a single quoted hex string, e.g. \"01 80 ff 00\"")
    ps.add_argument("--string", help="payload as a UTF-8 string (no NUL terminator)")
    for name in PACK:
        ps.add_argument(f"--{name}", type=(float if name == "float" else int_auto),
                        help=f"payload as a single little-endian {name}")
    ps.set_defaults(func=do_set)

    return p


def main():
    parser = make_parser()
    args = parser.parse_args()

    if args.config_h:
        CONFIG_H_CANDIDATES.insert(0, Path(args.config_h))
    load_opcodes()

    args.func(args)


if __name__ == "__main__":
    main()
