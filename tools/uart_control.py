#!/usr/bin/env python3
"""Small DSPi UART control helper.

The firmware protocol is ASCII:
  PING
  G <request> [wValue] [wLength]
  S <request> [wValue] <hex-payload>
  BGET
  BSET <hex-WireBulkParams>

Without --port the tool prints the encoded command, which is useful for tests
and for copying into another transport.  With --port it sends the command and
prints one response line.
"""

from __future__ import annotations

import argparse
import pathlib
import sys


def number(text: str) -> int:
    return int(text, 0)


def txrx(port: str | None, baud: int, command: str) -> str:
    line = command.rstrip("\r\n") + "\r\n"
    if port is None:
        return line.rstrip("\r\n")

    try:
        import serial  # type: ignore[import-not-found]
    except ImportError as exc:
        raise SystemExit("pyserial is required when --port is used") from exc

    with serial.Serial(port, baudrate=baud, timeout=1.0) as ser:
        ser.write(line.encode("ascii"))
        response = ser.readline().decode("ascii", errors="replace").rstrip("\r\n")
    return response


def cmd_ping(args: argparse.Namespace) -> int:
    print(txrx(args.port, args.baud, "PING"))
    return 0


def cmd_get(args: argparse.Namespace) -> int:
    command = f"G 0x{args.request:02X} 0x{args.wvalue:04X} {args.length}"
    print(txrx(args.port, args.baud, command))
    return 0


def cmd_set(args: argparse.Namespace) -> int:
    payload = args.hex_payload.upper()
    bytes.fromhex(payload)
    command = f"S 0x{args.request:02X} 0x{args.wvalue:04X} {payload}"
    print(txrx(args.port, args.baud, command))
    return 0


def cmd_bget(args: argparse.Namespace) -> int:
    print(txrx(args.port, args.baud, "BGET"))
    return 0


def cmd_bset(args: argparse.Namespace) -> int:
    data = pathlib.Path(args.file).read_bytes()
    command = f"BSET {data.hex().upper()}"
    print(txrx(args.port, args.baud, command))
    return 0


def cmd_self_test(_: argparse.Namespace) -> int:
    cases = {
        "ping": "PING",
        "get-platform": "G 0x7F 0x0000 4",
        "set-master-volume-minus-20db": "S 0xD2 0x0000 0000A0C1",
    }
    for name, expected in cases.items():
        actual = txrx(None, 115200, expected)
        if actual != expected:
            print(f"{name}: expected {expected!r}, got {actual!r}", file=sys.stderr)
            return 1
        print(f"{name}: {actual}")
    return 0


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", help="Serial device path. Omit to print the encoded command.")
    p.add_argument("--baud", type=int, default=115200)
    sub = p.add_subparsers(required=True)

    ping = sub.add_parser("ping")
    ping.set_defaults(func=cmd_ping)

    get = sub.add_parser("get")
    get.add_argument("request", type=number)
    get.add_argument("--wvalue", type=number, default=0)
    get.add_argument("--length", type=int, default=64)
    get.set_defaults(func=cmd_get)

    set_cmd = sub.add_parser("set")
    set_cmd.add_argument("request", type=number)
    set_cmd.add_argument("hex_payload")
    set_cmd.add_argument("--wvalue", type=number, default=0)
    set_cmd.set_defaults(func=cmd_set)

    bget = sub.add_parser("bget")
    bget.set_defaults(func=cmd_bget)

    bset = sub.add_parser("bset")
    bset.add_argument("file")
    bset.set_defaults(func=cmd_bset)

    self_test = sub.add_parser("self-test")
    self_test.set_defaults(func=cmd_self_test)
    return p


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
