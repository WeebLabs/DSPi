#!/usr/bin/env python3
"""Shared DSPi Option A control-frame helpers for Raspberry Pi scripts."""

from __future__ import annotations

from dataclasses import dataclass
import struct


CONTROL_MAGIC = b"DC"
CONTROL_REQUEST_VERSION = 0x01
CONTROL_RESPONSE_VERSION = 0x81
CONTROL_REQUEST_HEADER_LEN = 11
CONTROL_RESPONSE_HEADER_LEN = 6

CONTROL_DIR_OUT = 0
CONTROL_DIR_IN = 1

CONTROL_STATUS_OK = 0
CONTROL_STATUS_BUSY = 1
CONTROL_STATUS_STALL = 2
CONTROL_STATUS_BAD_FRAME = 3
CONTROL_STATUS_OVERFLOW = 4

STATUS_NAMES = {
    CONTROL_STATUS_OK: "OK",
    CONTROL_STATUS_BUSY: "BUSY",
    CONTROL_STATUS_STALL: "STALL",
    CONTROL_STATUS_BAD_FRAME: "BAD_FRAME",
    CONTROL_STATUS_OVERFLOW: "OVERFLOW",
}

REQ_GET_PLATFORM = 0x7F
REQ_GET_ALL_PARAMS = 0xA0
REQ_SET_MASTER_VOLUME = 0xD2
REQ_GET_MASTER_VOLUME = 0xD3

WIRE_FORMAT_VERSION = 7
WIRE_BULK_PARAMS_SIZE = 2912
WIRE_BULK_BUF_SIZE = 4096
WIRE_PLATFORM_RP2040 = 0
WIRE_PLATFORM_RP2350 = 1


class ControlFrameError(RuntimeError):
    """Raised when a transport frame is malformed or reports an error status."""


class TestReporter:
    def __init__(self) -> None:
        self.failures = 0
        self.skips = 0

    def line(self, message: str = "") -> None:
        print(message, flush=True)

    def testing(self, message: str) -> None:
        self.line()
        self.line(f"Testing {message}")

    def executing(self, message: str) -> None:
        self.line(f"Executing {message}")

    def pass_result(self, message: str) -> None:
        self.line(f"Result: PASS - {message}")

    def fail_result(self, message: str) -> None:
        self.failures += 1
        self.line(f"Result: FAIL - {message}")

    def skip_result(self, message: str) -> None:
        self.skips += 1
        self.line(f"Result: SKIP - {message}")

    def summary(self, label: str) -> None:
        if self.failures:
            self.line(
                f"{label} summary: FAIL ({self.failures} failed, "
                f"{self.skips} skipped)"
            )
        else:
            self.line(f"{label} summary: PASS ({self.skips} skipped)")


@dataclass(frozen=True)
class ControlResponse:
    status: int
    payload: bytes

    @property
    def status_name(self) -> str:
        return STATUS_NAMES.get(self.status, f"UNKNOWN_{self.status}")

    def require_ok(self, context: str) -> "ControlResponse":
        if self.status != CONTROL_STATUS_OK:
            raise ControlFrameError(f"{context}: transport status {self.status_name}")
        return self


def build_request(
    direction: int,
    request: int,
    *,
    value: int = 0,
    index: int = 0,
    length: int | None = None,
    payload: bytes = b"",
) -> bytes:
    if direction not in (CONTROL_DIR_OUT, CONTROL_DIR_IN):
        raise ValueError(f"invalid direction {direction}")
    if not 0 <= request <= 0xFF:
        raise ValueError(f"invalid request {request}")
    if not 0 <= value <= 0xFFFF:
        raise ValueError(f"invalid wValue {value}")
    if not 0 <= index <= 0xFFFF:
        raise ValueError(f"invalid wIndex {index}")

    payload = bytes(payload)
    if direction == CONTROL_DIR_OUT:
        if length is None:
            length = len(payload)
        if length != len(payload):
            raise ValueError("OUT wLength must match payload length")
    elif payload:
        raise ValueError("IN requests do not carry a payload")
    elif length is None:
        length = 0

    if not 0 <= length <= WIRE_BULK_BUF_SIZE:
        raise ValueError(f"invalid wLength {length}")

    return struct.pack(
        "<2sBBBHHH",
        CONTROL_MAGIC,
        CONTROL_REQUEST_VERSION,
        direction,
        request,
        value,
        index,
        length,
    ) + payload


def parse_response_header(header: bytes) -> tuple[int, int]:
    if len(header) != CONTROL_RESPONSE_HEADER_LEN:
        raise ControlFrameError(f"short response header: {len(header)} bytes")
    magic, version, status, length = struct.unpack("<2sBBH", header)
    if magic != CONTROL_MAGIC:
        raise ControlFrameError(f"bad response magic: {magic!r}")
    if version != CONTROL_RESPONSE_VERSION:
        raise ControlFrameError(f"bad response version: 0x{version:02x}")
    if length > WIRE_BULK_BUF_SIZE:
        raise ControlFrameError(f"impossible response length: {length}")
    return status, length


def pack_float32(value: float) -> bytes:
    return struct.pack("<f", value)


def unpack_float32(payload: bytes, context: str) -> float:
    if len(payload) != 4:
        raise ControlFrameError(f"{context} returned {len(payload)} bytes")
    return struct.unpack("<f", payload)[0]


def parse_platform(payload: bytes) -> dict[str, int | str]:
    if len(payload) != 4:
        raise ControlFrameError(f"REQ_GET_PLATFORM returned {len(payload)} bytes")
    platform, fw_major, fw_minor_bcd, output_count = payload
    if platform == WIRE_PLATFORM_RP2040:
        platform_name = "RP2040"
    elif platform == WIRE_PLATFORM_RP2350:
        platform_name = "RP2350"
    else:
        raise ControlFrameError(f"unexpected platform id {platform}")
    return {
        "platform": platform,
        "platform_name": platform_name,
        "fw_major": fw_major,
        "fw_minor_bcd": fw_minor_bcd,
        "output_count": output_count,
    }


def parse_bulk_header(payload: bytes) -> dict[str, int]:
    if len(payload) != WIRE_BULK_PARAMS_SIZE:
        raise ControlFrameError(
            f"REQ_GET_ALL_PARAMS returned {len(payload)} bytes, expected "
            f"{WIRE_BULK_PARAMS_SIZE}"
        )
    header = struct.unpack_from("<BBBBBBHHHI", payload, 0)
    (
        format_version,
        platform_id,
        num_channels,
        num_output_channels,
        num_input_channels,
        max_bands,
        payload_length,
        fw_version_major,
        fw_version_minor,
        reserved,
    ) = header
    if format_version != WIRE_FORMAT_VERSION:
        raise ControlFrameError(
            f"unexpected wire format version {format_version}, expected "
            f"{WIRE_FORMAT_VERSION}"
        )
    if platform_id not in (WIRE_PLATFORM_RP2040, WIRE_PLATFORM_RP2350):
        raise ControlFrameError(f"unexpected platform id {platform_id}")
    if payload_length != WIRE_BULK_PARAMS_SIZE:
        raise ControlFrameError(
            f"bulk payload_length {payload_length}, expected {WIRE_BULK_PARAMS_SIZE}"
        )
    if num_input_channels != 2:
        raise ControlFrameError(f"unexpected input channel count {num_input_channels}")
    if reserved != 0:
        raise ControlFrameError(f"bulk header reserved field is nonzero: {reserved}")

    return {
        "format_version": format_version,
        "platform_id": platform_id,
        "num_channels": num_channels,
        "num_output_channels": num_output_channels,
        "num_input_channels": num_input_channels,
        "max_bands": max_bands,
        "payload_length": payload_length,
        "fw_version_major": fw_version_major,
        "fw_version_minor": fw_version_minor,
    }


def platform_summary(info: dict[str, int | str]) -> str:
    return (
        f"{info['platform_name']} fw={info['fw_major']}."
        f"0x{info['fw_minor_bcd']:02x} outputs={info['output_count']}"
    )


def print_setup_notes(transport: str) -> None:
    def say(message: str) -> None:
        print(message, flush=True)

    say("DSPi transport-enabled firmware build example:")
    say(
        "  cmake -S firmware -B firmware/build-rp2350-control -DPICO_BOARD=pico2 "
        "-DDSPI_CONTROL_I2C_PRIORITY=20 -DDSPI_CONTROL_UART_PRIORITY=10"
    )
    say("  cmake --build firmware/build-rp2350-control --config Release")
    if transport == "i2c":
        say("Default I2C wiring:")
        say("  Raspberry Pi GPIO2/SDA1 pin 3  -> DSPi GPIO4/SDA")
        say("  Raspberry Pi GPIO3/SCL1 pin 5  -> DSPi GPIO5/SCL")
        say("  Raspberry Pi GND               -> DSPi GND")
        say("  Enable Raspberry Pi I2C with raspi-config or dtparam=i2c_arm=on")
    elif transport == "uart":
        say("Default UART wiring:")
        say("  Raspberry Pi GPIO14/TXD pin 8  -> DSPi GPIO13/RX")
        say("  Raspberry Pi GPIO15/RXD pin 10 -> DSPi GPIO12/TX")
        say("  Raspberry Pi GND               -> DSPi GND")
        say("  Disable the Raspberry Pi serial console on this port before testing")
        say("  Choose alternate DSPi UART pins if GPIO13 is needed for I2S MCK")
    say("Use 3.3 V signalling only. Do not connect 5 V to Pico GPIO.")
