# Tests

## Current Coverage

- `tools/uart_control.py self-test` verifies host-side command encoding without requiring hardware.
- Firmware build verification should compile both RP2040 and RP2350 targets with UART disabled and with UART enabled using board-specific pins.
- Hardware-in-the-loop testing should run UART `PING`, representative `G` and `S` commands, malformed input recovery, and `BGET`/`BSET` on each target platform.

## Gaps

- There is not yet an off-target C unit test harness for the firmware UART parser.
- CI build coverage depends on the local Pico SDK/CMake toolchain being available.
- USB/UART concurrent last-writer-wins needs hardware or simulator support because it exercises real shared firmware state.
