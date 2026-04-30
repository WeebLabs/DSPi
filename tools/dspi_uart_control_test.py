#!/usr/bin/env python3
"""Raspberry Pi UART smoke test and example for DSPi control frames."""

from __future__ import annotations

import argparse
import os
import select
import sys
import termios
import time
import tty

from dspi_control_common import (
    CONTROL_DIR_IN,
    CONTROL_DIR_OUT,
    CONTROL_RESPONSE_HEADER_LEN,
    CONTROL_STATUS_BAD_FRAME,
    CONTROL_STATUS_BUSY,
    ControlFrameError,
    ControlResponse,
    REQ_GET_ALL_PARAMS,
    REQ_GET_PLATFORM,
    REQ_GET_MASTER_VOLUME,
    REQ_SET_MASTER_VOLUME,
    TestReporter,
    WIRE_BULK_PARAMS_SIZE,
    build_request,
    pack_float32,
    parse_bulk_header,
    parse_platform,
    parse_response_header,
    platform_summary,
    print_setup_notes,
    unpack_float32,
)


class DspiUart:
    def __init__(self, device: str, baud: int, *, timeout_s: float) -> None:
        self.device = device
        self.baud = baud
        self.timeout_s = timeout_s
        self.fd: int | None = None
        self.old_attrs: list | None = None

    def __enter__(self) -> "DspiUart":
        self.fd = os.open(self.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.old_attrs = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        attrs = termios.tcgetattr(self.fd)
        speed = baud_constant(self.baud)
        attrs[4] = speed
        attrs[5] = speed
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] |= termios.CLOCAL | termios.CREAD | termios.CS8
        attrs[2] &= ~termios.PARENB
        attrs[2] &= ~termios.CSTOPB
        attrs[2] &= ~termios.CSIZE
        attrs[2] |= termios.CS8
        if hasattr(termios, "CRTSCTS"):
            attrs[2] &= ~termios.CRTSCTS
        attrs[3] = 0
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self.fd is not None:
            if self.old_attrs is not None:
                termios.tcsetattr(self.fd, termios.TCSANOW, self.old_attrs)
            os.close(self.fd)
            self.fd = None
            self.old_attrs = None

    def _fd(self) -> int:
        if self.fd is None:
            raise RuntimeError("UART device is not open")
        return self.fd

    def transact(self, frame: bytes, context: str) -> ControlResponse:
        self._write_all(frame, context)
        return self.read_response(context)

    def transact_raw(self, frame: bytes, context: str) -> ControlResponse:
        self._write_all(frame, context)
        return self.read_response(context)

    def read_response(self, context: str) -> ControlResponse:
        deadline = time.monotonic() + self.timeout_s
        while True:
            header = self._read_exact(CONTROL_RESPONSE_HEADER_LEN, deadline, context)
            try:
                status, payload_len = parse_response_header(header)
            except ControlFrameError as exc:
                raise ControlFrameError(
                    f"{context}: {exc}; raw response header {header.hex(' ')}"
                ) from exc
            if status != CONTROL_STATUS_BUSY:
                break
        payload = self._read_exact(payload_len, deadline, context) if payload_len else b""
        return ControlResponse(status, payload)

    def _write_all(self, data: bytes, context: str) -> None:
        fd = self._fd()
        view = memoryview(data)
        while view:
            _, writable, _ = select.select([], [fd], [], self.timeout_s)
            if not writable:
                raise TimeoutError(f"{context}: timed out writing UART")
            written = os.write(fd, view)
            view = view[written:]

    def _read_exact(self, size: int, deadline: float, context: str) -> bytes:
        fd = self._fd()
        chunks: list[bytes] = []
        remaining = size
        while remaining:
            timeout = max(0.0, deadline - time.monotonic())
            if timeout == 0.0:
                raise TimeoutError(f"{context}: timed out reading UART")
            readable, _, _ = select.select([fd], [], [], timeout)
            if not readable:
                raise TimeoutError(f"{context}: timed out reading UART")
            chunk = os.read(fd, remaining)
            if not chunk:
                continue
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)


def baud_constant(baud: int) -> int:
    name = f"B{baud}"
    if not hasattr(termios, name):
        raise ValueError(f"termios does not expose {name} on this platform")
    return getattr(termios, name)


def choose_master_volume_target(original_db: float, requested_db: float) -> float:
    if abs(original_db - requested_db) > 0.001:
        return requested_db
    return requested_db + 1.0 if requested_db <= -1.0 else requested_db - 1.0


def require_master_volume(actual_db: float, expected_db: float, context: str) -> None:
    if abs(actual_db - expected_db) > 0.001:
        raise ControlFrameError(
            f"{context} read {actual_db:.4f} dB, expected {expected_db:.4f} dB"
        )


def require_master_volume_changed(actual_db: float, original_db: float) -> None:
    if abs(actual_db - original_db) <= 0.001:
        raise ControlFrameError(
            f"master volume did not change from original {original_db:.4f} dB"
        )


HANDLED_EXCEPTIONS = (ControlFrameError, OSError, TimeoutError, ValueError)


def run_step(reporter: TestReporter, name: str, func) -> None:
    reporter.testing(name)
    try:
        func()
    except HANDLED_EXCEPTIONS as exc:
        reporter.fail_result(str(exc))
    except Exception as exc:
        reporter.fail_result(f"unexpected {type(exc).__name__}: {exc}")


def run_checks(args: argparse.Namespace) -> bool:
    reporter = TestReporter()
    print_setup_notes("uart")
    reporter.line(f"UART device: {args.device}, baud: {args.baud}")

    uart = DspiUart(args.device, args.baud, timeout_s=args.timeout)
    state: dict[str, float] = {}

    reporter.testing("UART device open")
    reporter.executing(f"open {args.device} and configure {args.baud} baud 8N1")
    try:
        uart.__enter__()
        reporter.pass_result("UART device opened")
    except HANDLED_EXCEPTIONS as exc:
        reporter.fail_result(str(exc))
        try:
            uart.__exit__(None, None, None)
        except HANDLED_EXCEPTIONS:
            pass
        for name in [
            "REQ_GET_PLATFORM",
            "REQ_GET_MASTER_VOLUME initial read",
            "REQ_SET_MASTER_VOLUME / REQ_GET_MASTER_VOLUME round trip",
            "REQ_GET_ALL_PARAMS bulk read",
            "malformed-frame BAD_FRAME response",
        ]:
            reporter.testing(name)
            reporter.skip_result("UART device did not open")
        reporter.summary("UART checks")
        return False

    try:
        def platform_check() -> None:
            reporter.executing("send REQ_GET_PLATFORM IN wLength=4")
            platform_resp = uart.transact(
                build_request(CONTROL_DIR_IN, REQ_GET_PLATFORM, length=4),
                "REQ_GET_PLATFORM",
            ).require_ok("REQ_GET_PLATFORM")
            reporter.executing(
                f"parse REQ_GET_PLATFORM payload {platform_resp.payload.hex(' ')}"
            )
            platform = parse_platform(platform_resp.payload)
            reporter.pass_result(platform_summary(platform))

        run_step(reporter, "REQ_GET_PLATFORM", platform_check)

        def master_volume_read() -> None:
            reporter.executing("send REQ_GET_MASTER_VOLUME IN wLength=4")
            current_volume = uart.transact(
                build_request(CONTROL_DIR_IN, REQ_GET_MASTER_VOLUME, length=4),
                "REQ_GET_MASTER_VOLUME before",
            ).require_ok("REQ_GET_MASTER_VOLUME before")
            reporter.executing(
                f"decode little-endian float payload {current_volume.payload.hex(' ')}"
            )
            original = unpack_float32(
                current_volume.payload,
                "REQ_GET_MASTER_VOLUME before",
            )
            state["original_master_volume_db"] = original
            reporter.pass_result(f"master volume is {original:.4f} dB")

        run_step(reporter, "REQ_GET_MASTER_VOLUME initial read", master_volume_read)

        def master_volume_roundtrip() -> None:
            if "original_master_volume_db" not in state:
                reporter.skip_result("initial master-volume read did not complete")
                return
            original = state["original_master_volume_db"]
            target = choose_master_volume_target(original, args.master_volume_target)
            state["target_master_volume_db"] = target
            payload = pack_float32(target)
            reporter.executing(
                "send REQ_SET_MASTER_VOLUME OUT "
                f"wLength=4 payload={payload.hex(' ')} ({target:.4f} dB)"
            )
            uart.transact(
                build_request(
                    CONTROL_DIR_OUT,
                    REQ_SET_MASTER_VOLUME,
                    payload=payload,
                ),
                "REQ_SET_MASTER_VOLUME",
            ).require_ok("REQ_SET_MASTER_VOLUME")
            reporter.executing("send REQ_GET_MASTER_VOLUME IN wLength=4")
            after_set = uart.transact(
                build_request(CONTROL_DIR_IN, REQ_GET_MASTER_VOLUME, length=4),
                "REQ_GET_MASTER_VOLUME after",
            ).require_ok("REQ_GET_MASTER_VOLUME after")
            reporter.executing(
                f"decode little-endian float payload {after_set.payload.hex(' ')}"
            )
            actual = unpack_float32(after_set.payload, "REQ_GET_MASTER_VOLUME after")
            require_master_volume(actual, target, "master volume round trip")
            require_master_volume_changed(actual, original)
            reporter.pass_result(
                f"master volume changed {original:.4f} dB -> {actual:.4f} dB"
            )

        run_step(
            reporter,
            "REQ_SET_MASTER_VOLUME / REQ_GET_MASTER_VOLUME round trip",
            master_volume_roundtrip,
        )

        def restore_master_volume() -> None:
            if "original_master_volume_db" not in state:
                reporter.skip_result("initial master-volume read did not complete")
                return
            original = state["original_master_volume_db"]
            payload = pack_float32(original)
            reporter.executing(
                "send REQ_SET_MASTER_VOLUME OUT restore "
                f"payload={payload.hex(' ')} ({original:.4f} dB)"
            )
            uart.transact(
                build_request(
                    CONTROL_DIR_OUT,
                    REQ_SET_MASTER_VOLUME,
                    payload=payload,
                ),
                "restore REQ_SET_MASTER_VOLUME",
            ).require_ok("restore REQ_SET_MASTER_VOLUME")
            reporter.executing("send REQ_GET_MASTER_VOLUME IN wLength=4")
            restored = uart.transact(
                build_request(CONTROL_DIR_IN, REQ_GET_MASTER_VOLUME, length=4),
                "REQ_GET_MASTER_VOLUME restored",
            ).require_ok("REQ_GET_MASTER_VOLUME restored")
            restored_db = unpack_float32(
                restored.payload,
                "REQ_GET_MASTER_VOLUME restored",
            )
            require_master_volume(restored_db, original, "master volume restore")
            reporter.pass_result(f"master volume restored to {restored_db:.4f} dB")

        run_step(
            reporter,
            "restore REQ_SET_MASTER_VOLUME after changed-value round trip",
            restore_master_volume,
        )

        def bulk_read() -> None:
            reporter.executing(
                f"send REQ_GET_ALL_PARAMS IN wLength={WIRE_BULK_PARAMS_SIZE}"
            )
            bulk_resp = uart.transact(
                build_request(
                    CONTROL_DIR_IN,
                    REQ_GET_ALL_PARAMS,
                    length=WIRE_BULK_PARAMS_SIZE,
                ),
                "REQ_GET_ALL_PARAMS",
            ).require_ok("REQ_GET_ALL_PARAMS")
            reporter.executing(
                f"parse WireBulkParams payload length {len(bulk_resp.payload)}"
            )
            bulk = parse_bulk_header(bulk_resp.payload)
            reporter.pass_result(
                f"{bulk['payload_length']} bytes, format v{bulk['format_version']}, "
                f"channels={bulk['num_channels']}, "
                f"outputs={bulk['num_output_channels']}"
            )

        run_step(reporter, "REQ_GET_ALL_PARAMS bulk read", bulk_read)

        def bad_frame() -> None:
            if args.skip_error_test:
                reporter.skip_result("--skip-error-test was supplied")
                return
            reporter.executing("send malformed frame bytes: 44 43 02")
            bad = uart.transact_raw(b"DC\x02", "bad-version frame")
            reporter.executing(
                f"inspect transport status {bad.status} ({bad.status_name})"
            )
            if bad.status != CONTROL_STATUS_BAD_FRAME:
                raise ControlFrameError(
                    f"bad-version frame returned status {bad.status_name}"
                )
            reporter.pass_result("transport returned BAD_FRAME")

        run_step(reporter, "malformed-frame BAD_FRAME response", bad_frame)

    finally:
        reporter.testing("UART device close")
        reporter.executing(f"close {args.device}")
        try:
            uart.__exit__(None, None, None)
            reporter.pass_result("UART device closed")
        except HANDLED_EXCEPTIONS as exc:
            reporter.fail_result(str(exc))

    reporter.summary("UART checks")
    return reporter.failures == 0


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument(
        "--master-volume-target",
        type=float,
        default=-37.0,
        help="dB value to set and read back before restoring the original value",
    )
    parser.add_argument("--mutating-roundtrip", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument(
        "--skip-error-test",
        action="store_true",
        help="skip the deterministic malformed-frame BAD_FRAME check",
    )
    args = parser.parse_args(argv)
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if not -127.0 <= args.master_volume_target <= 0.0:
        parser.error("--master-volume-target must be in the -127..0 dB range")
    return args


def main(argv: list[str]) -> int:
    return 0 if run_checks(parse_args(argv)) else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
