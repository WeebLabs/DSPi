#!/usr/bin/env python3
"""Structural parity checks for the shared control executor.

These tests intentionally avoid hardware.  They verify that USB and non-USB
transport adapters route through one semantic executor and that the command
surface still has a handler in that executor.
"""

from __future__ import annotations

import re
import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FW = ROOT / "firmware" / "DSPi"
TOOLS = ROOT / "tools"


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
    docs = {
        "DESIGN.md": ["# Design"],
        "TESTING.md": ["# Testing Strategy"],
        "ROADMAP.md": ["# Roadmap", "## Now", "## Later", "## Completed"],
        "PLAN.md": [
            "# Plan",
            "## Roadmap alignment",
            "## Objective",
            "## Assumptions",
            "## Scenario mapping",
            "## Exit criteria",
            "## Promotion rule",
        ],
        "tests/TESTS.md": ["# Tests"],
        "tests/SCENARIOS.md": ["# Scenarios"],
    }
    for relative, headings in docs.items():
        content = read(ROOT / relative)
        for heading in headings:
            assert heading in content, f"{relative} missing heading {heading}"


def load_common_module():
    path = TOOLS / "dspi_control_common.py"
    spec = importlib.util.spec_from_file_location("dspi_control_common", path)
    if spec is None or spec.loader is None:
        raise AssertionError(f"could not load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_raspberry_pi_scripts_exist() -> None:
    for relative in [
        "dspi_control_common.py",
        "dspi_i2c_control_test.py",
        "dspi_uart_control_test.py",
        "dspi_control_examples.md",
    ]:
        assert (TOOLS / relative).exists(), f"missing Raspberry Pi tool {relative}"


def test_raspberry_pi_frame_helper_matches_firmware_contract() -> None:
    common = load_common_module()
    frame = common.build_request(common.CONTROL_DIR_IN, common.REQ_GET_PLATFORM, length=4)
    assert frame == b"DC\x01\x01\x7f\x00\x00\x00\x00\x04\x00"
    assert common.parse_response_header(b"DC\x81\x00\x04\x00") == (common.CONTROL_STATUS_OK, 4)
    assert common.REQ_SET_MASTER_VOLUME == 0xD2
    assert common.REQ_GET_MASTER_VOLUME == 0xD3


def test_raspberry_pi_docs_reference_scripts_and_checks() -> None:
    examples = read(TOOLS / "dspi_control_examples.md")
    tests_doc = read(ROOT / "tests" / "TESTS.md")
    for token in [
        "tools/dspi_i2c_control_test.py",
        "tools/dspi_uart_control_test.py",
        "REQ_GET_PLATFORM",
        "REQ_SET_MASTER_VOLUME",
        "REQ_GET_MASTER_VOLUME",
        "REQ_GET_ALL_PARAMS",
    ]:
        assert token in examples, f"example docs missing {token}"
    for token in [
        "dspi_i2c_control_test.py",
        "dspi_uart_control_test.py",
        "REQ_SET_MASTER_VOLUME",
        "REQ_GET_MASTER_VOLUME",
        "REQ_GET_ALL_PARAMS",
    ]:
        assert token in tests_doc, f"tests docs missing {token}"


def main() -> None:
    tests = [
        test_usb_uses_shared_executor,
        test_non_usb_uses_shared_executor,
        test_option_a_frame_shape_is_documented_in_code,
        test_request_ids_are_covered_by_executor_switches,
        test_required_plan_docs_exist_with_expected_headings,
        test_raspberry_pi_scripts_exist,
        test_raspberry_pi_frame_helper_matches_firmware_contract,
        test_raspberry_pi_docs_reference_scripts_and_checks,
    ]
    for test in tests:
        test()
        print(f"PASS {test.__name__}")


if __name__ == "__main__":
    main()
