#!/usr/bin/env python3
"""Raspberry Pi I2C smoke test and example for DSPi control frames."""

from __future__ import annotations

import argparse
import errno
import fcntl
import os
import sys
import time

from dspi_control_common import (
    CONTROL_DIR_IN,
    CONTROL_DIR_OUT,
    CONTROL_REQUEST_HEADER_LEN,
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


I2C_SLAVE = 0x0703


class DspiI2C:
    def __init__(
        self,
        device: str,
        address: int,
        *,
        timeout_s: float,
        poll_interval_s: float,
        turnaround_delay_s: float,
    ) -> None:
        self.device = device
        self.address = address
        self.timeout_s = timeout_s
        self.poll_interval_s = poll_interval_s
        self.turnaround_delay_s = turnaround_delay_s
        self.fd: int | None = None

    def __enter__(self) -> "DspiI2C":
        self.fd = os.open(self.device, os.O_RDWR)
        fcntl.ioctl(self.fd, I2C_SLAVE, self.address)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def _fd(self) -> int:
        if self.fd is None:
            raise RuntimeError("I2C device is not open")
        return self.fd

    def transact(self, frame: bytes, context: str) -> ControlResponse:
        written = os.write(self._fd(), frame)
        if written != len(frame):
            raise OSError(f"{context}: short I2C write {written}/{len(frame)}")
        if self.turnaround_delay_s > 0:
            time.sleep(self.turnaround_delay_s)
        return self.read_response(context, self._read_len_for_frame(frame))

    def transact_raw(self, frame: bytes, context: str) -> ControlResponse:
        written = os.write(self._fd(), frame)
        if written != len(frame):
            raise OSError(f"{context}: short I2C write {written}/{len(frame)}")
        if self.turnaround_delay_s > 0:
            time.sleep(self.turnaround_delay_s)
        return self.read_response(context, CONTROL_RESPONSE_HEADER_LEN)

    def _read_len_for_frame(self, frame: bytes) -> int:
        if len(frame) >= CONTROL_REQUEST_HEADER_LEN and frame[3] == CONTROL_DIR_IN:
            payload_len = int.from_bytes(frame[9:11], "little")
            return CONTROL_RESPONSE_HEADER_LEN + payload_len
        return CONTROL_RESPONSE_HEADER_LEN

    def read_response(self, context: str, read_len: int) -> ControlResponse:
        # DSPi ends an I2C response on STOP, so header and payload must be read
        # in one transaction.  Extra bytes after the declared payload are ignored.
        read_len = max(read_len, CONTROL_RESPONSE_HEADER_LEN)
        deadline = time.monotonic() + self.timeout_s
        while True:
            raw = self._read_once(read_len, deadline, context)
            if len(raw) < CONTROL_RESPONSE_HEADER_LEN:
                raise TimeoutError(
                    f"{context}: short I2C response header "
                    f"{len(raw)}/{CONTROL_RESPONSE_HEADER_LEN}"
                )
            header = raw[:CONTROL_RESPONSE_HEADER_LEN]
            try:
                status, payload_len = parse_response_header(header)
            except ControlFrameError as exc:
                raise ControlFrameError(
                    f"{context}: {exc}; raw response "
                    f"{raw.hex(' ')}"
                ) from exc
            if status != CONTROL_STATUS_BUSY:
                break
            if time.monotonic() >= deadline:
                raise TimeoutError(f"{context}: timed out waiting for response")
            time.sleep(self.poll_interval_s)

        payload_start = CONTROL_RESPONSE_HEADER_LEN
        payload_end = payload_start + payload_len
        if payload_end > len(raw):
            raise ControlFrameError(
                f"{context}: I2C read returned {len(raw) - payload_start} "
                f"payload bytes, response header declared {payload_len}"
            )
        payload = raw[payload_start:payload_end]
        return ControlResponse(status, payload)

    def _read_once(self, size: int, deadline: float, context: str) -> bytes:
        while True:
            try:
                data = os.read(self._fd(), size)
            except OSError as exc:
                if exc.errno in (errno.EAGAIN, errno.EREMOTEIO, errno.ENXIO):
                    if time.monotonic() >= deadline:
                        raise TimeoutError(f"{context}: timed out reading I2C") from exc
                    time.sleep(self.poll_interval_s)
                    continue
                raise

            if data:
                return data
            if time.monotonic() >= deadline:
                raise TimeoutError(f"{context}: timed out reading I2C")
            time.sleep(self.poll_interval_s)


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
    print_setup_notes("i2c")
    reporter.line(f"I2C device: {args.device}, address: 0x{args.address:02x}")
    reporter.line(f"I2C turnaround delay: {args.turnaround_delay:.6f} s")

    bus = DspiI2C(
        args.device,
        args.address,
        timeout_s=args.timeout,
        poll_interval_s=args.poll_interval,
        turnaround_delay_s=args.turnaround_delay,
    )
    state: dict[str, float] = {}

    reporter.testing("I2C device open")
    reporter.executing(f"open {args.device} and select slave address 0x{args.address:02x}")
    try:
        bus.__enter__()
        reporter.pass_result("I2C device opened")
    except HANDLED_EXCEPTIONS as exc:
        reporter.fail_result(str(exc))
        try:
            bus.__exit__(None, None, None)
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
            reporter.skip_result("I2C device did not open")
        reporter.summary("I2C checks")
        return False

    try:
        def platform_check() -> None:
            reporter.executing("send REQ_GET_PLATFORM IN wLength=4")
            platform_resp = bus.transact(
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
            current_volume = bus.transact(
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
            bus.transact(
                build_request(
                    CONTROL_DIR_OUT,
                    REQ_SET_MASTER_VOLUME,
                    payload=payload,
                ),
                "REQ_SET_MASTER_VOLUME",
            ).require_ok("REQ_SET_MASTER_VOLUME")
            reporter.executing("send REQ_GET_MASTER_VOLUME IN wLength=4")
            after_set = bus.transact(
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
            bus.transact(
                build_request(
                    CONTROL_DIR_OUT,
                    REQ_SET_MASTER_VOLUME,
                    payload=payload,
                ),
                "restore REQ_SET_MASTER_VOLUME",
            ).require_ok("restore REQ_SET_MASTER_VOLUME")
            reporter.executing("send REQ_GET_MASTER_VOLUME IN wLength=4")
            restored = bus.transact(
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
                "send REQ_GET_ALL_PARAMS IN "
                f"wLength={WIRE_BULK_PARAMS_SIZE}; read header and payload together"
            )
            bulk_resp = bus.transact(
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
            bad = bus.transact_raw(b"DC\x02", "bad-version frame")
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
        reporter.testing("I2C device close")
        reporter.executing(f"close {args.device}")
        try:
            bus.__exit__(None, None, None)
            reporter.pass_result("I2C device closed")
        except HANDLED_EXCEPTIONS as exc:
            reporter.fail_result(str(exc))

    reporter.summary("I2C checks")
    return reporter.failures == 0


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--device", default="/dev/i2c-1")
    parser.add_argument("--address", type=lambda s: int(s, 0), default=0x2A)
    parser.add_argument("--chunk-size", type=int, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--poll-interval", type=float, default=0.01)
    parser.add_argument(
        "--turnaround-delay",
        type=float,
        default=0.001,
        help="seconds to wait between writing a request and reading its response",
    )
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
    if args.turnaround_delay < 0:
        parser.error("--turnaround-delay must be non-negative")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.poll_interval <= 0:
        parser.error("--poll-interval must be positive")
    if not -127.0 <= args.master_volume_target <= 0.0:
        parser.error("--master-volume-target must be in the -127..0 dB range")
    return args


def main(argv: list[str]) -> int:
    return 0 if run_checks(parse_args(argv)) else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
