#!/usr/bin/env python3
"""Structural parity checks for the shared control executor.

These tests intentionally avoid hardware.  They verify that USB and non-USB
transport adapters route through one semantic executor and that the command
surface still has a handler in that executor.
"""

from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FW = ROOT / "firmware" / "DSPi"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def extract_function(source: str, name: str) -> str:
    match = re.search(rf"\b{name}\s*\([^)]*\)\s*\{{", source)
    if not match:
        raise AssertionError(f"missing function {name}")
    start = match.start()
    brace_start = source.index("{", match.end() - 1)
    depth = 0
    for idx in range(brace_start, len(source)):
        if source[idx] == "{":
            depth += 1
        elif source[idx] == "}":
            depth -= 1
            if depth == 0:
                return source[start : idx + 1]
    raise AssertionError(f"unterminated function {name}")


def test_usb_uses_shared_executor() -> None:
    vendor = read(FW / "vendor_commands.c")
    callback = extract_function(vendor, "tud_vendor_control_xfer_cb")
    required = [
        "control_executor_execute_in",
        "control_executor_prepare_out",
        "control_executor_execute_out",
        "control_executor_commit_out",
    ]
    for symbol in required:
        assert symbol in callback, f"USB callback does not call {symbol}"
    assert "vendor_handle_get(" not in callback
    assert "vendor_handle_set_data(" not in callback


def test_non_usb_uses_shared_executor() -> None:
    transport = read(FW / "control_transport.c")
    for symbol in [
        "control_executor_execute_in",
        "control_executor_prepare_out",
        "control_executor_execute_out",
        "control_executor_commit_out",
    ]:
        assert symbol in transport, f"non-USB transport does not call {symbol}"


def test_option_a_frame_shape_is_documented_in_code() -> None:
    transport = read(FW / "control_transport.c")
    for token in [
        '"D" "C" 0x01 dir bRequest wValueLE wIndexLE wLengthLE payload',
        "CONTROL_REQUEST_HEADER_LEN 11",
        "CONTROL_RESPONSE_HEADER_LEN 6",
        "CONTROL_RESPONSE_VERSION 0x81",
    ]:
        assert token in transport, f"missing frame-shape token: {token}"


def test_request_ids_are_covered_by_executor_switches() -> None:
    config = read(FW / "config.h")
    vendor = read(FW / "vendor_commands.c")
    request_ids = set(re.findall(r"#define\s+(REQ_[A-Z0-9_]+)\s+0x[0-9A-Fa-f]+", config))
    handled = set(re.findall(r"case\s+(REQ_[A-Z0-9_]+)\s*:", vendor))
    special = {"REQ_SET_ALL_PARAMS"}
    missing = sorted(request_ids - handled - special)
    assert not missing, "request IDs missing executor cases: " + ", ".join(missing)


def test_required_plan_docs_exist_with_expected_headings() -> None:
    expected = {
        "PLAN.md": [
            "Roadmap alignment",
            "Objective",
            "Assumptions",
            "Scenario mapping",
            "Exit criteria",
            "Promotion rule",
        ],
        "DESIGN.md": ["Control interface architecture"],
        "tests/TESTS.md": ["Control parity tests"],
        "tests/SCENARIOS.md": ["Shared control executor parity"],
    }
    for rel, headings in expected.items():
        text = read(ROOT / rel)
        for heading in headings:
            assert f"## {heading}" in text or f"# {heading}" in text, (
                f"{rel} missing heading {heading}"
            )


def main() -> None:
    tests = [
        test_usb_uses_shared_executor,
        test_non_usb_uses_shared_executor,
        test_option_a_frame_shape_is_documented_in_code,
        test_request_ids_are_covered_by_executor_switches,
        test_required_plan_docs_exist_with_expected_headings,
    ]
    for test in tests:
        test()
        print(f"PASS {test.__name__}")


if __name__ == "__main__":
    main()
