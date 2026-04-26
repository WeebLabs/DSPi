# Plan

## Roadmap alignment
- Implement the `Now` roadmap item by designing and delivering feature-equivalent UART control as a second transport over the same DSPi control semantics currently exposed through USB vendor requests.

## Objective
- Refactor control handling so USB and UART commands share one transport-neutral command core, then add a robust build-time configurable ASCII UART protocol that can read, write, persist, and query the same settings as USB without duplicating state or command rules.

## Assumptions
- The UART peer is a trusted local control MCU, so the first implementation does not need authentication, encryption, rate-limited security policy, or user-accessible hardening beyond parser robustness.
- The UART protocol should be human-readable ASCII because the same command vocabulary should be reusable later over network or Bluetooth control links.
- UART instance, TX/RX pins, baud rate, line endings, RX/TX buffer sizes, and hardware flow-control policy are compile-time configuration options.
- USB and UART may both be active; whichever transport successfully applies a command last owns the resulting value.
- UART responses should be structured and brief, with success, error code, request correlation where practical, and compact payload formatting.
- Flow control is determined at build time; the runtime parser still needs bounded queues and nonblocking behavior when the peer sends too much data.
- Firmware update entry over UART is not part of the first implementation unless the developer explicitly promotes it from the `Later` review item.

## Scenario mapping
- Given the control MCU sends a valid ASCII `GET` command for any setting currently readable over USB, when the line is received over the configured UART, then firmware returns the same effective value as the equivalent USB request.
- Given the control MCU sends a valid ASCII `SET` command for any setting currently writable over USB, when the command is accepted, then live DSP state changes exactly as the equivalent USB command would and a brief structured success response is returned.
- Given USB and UART both set the same parameter, when the second command completes successfully, then subsequent reads over both transports report the second value.
- Given UART saves, loads, deletes, names, or selects presets, when the operation completes or is accepted for deferred processing, then flash persistence and pipeline reset behavior matches the equivalent USB command.
- Given UART requests bulk state read or write, when the transfer completes, then the resulting state matches USB bulk parameter semantics and rejects malformed or mismatched payloads without partial application.
- Given UART receives malformed, unknown, too-long, partial, or timed-out input, when the parser handles it, then it returns a concise error where possible, resets to a known parser state, and leaves live DSP state unchanged.
- Given UART control is enabled in RP2040 and RP2350 builds, when automated host tooling runs command-equivalence and malformed-input tests, then both platforms build and the documented hardware-in-the-loop scenarios pass.

## Exit criteria
- Control-command behavior is factored so USB and UART route through shared typed command handlers for all applicable settings, queries, actions, deferred flash operations, and bulk parameter operations.
- UART protocol grammar, parser state machine, response format, error codes, timeout behavior, buffer limits, and build-time configuration options are documented.
- UART implementation is nonblocking in the main loop or IRQ context, uses bounded buffers, handles parser reset explicitly, and cannot perform flash erases/programming directly from interrupt context.
- USB behavior remains backward-compatible with the existing vendor request IDs and wire payloads.
- UART read/write coverage is feature-equivalent to the existing USB control table, with any deliberate exclusions documented and approved.
- Persistence semantics are shared between transports, including preset directory updates, preset slot operations, independent master-volume save, pin inclusion, and factory reset behavior.
- Tests or tooling cover parser behavior, command equivalence, malformed input, oversized input, timeout recovery, concurrent USB/UART last-writer-wins behavior, and representative hardware-in-the-loop UART sessions.
- `DESIGN.md` or the current architecture documentation, `TESTING.md`, `tests/SCENARIOS.md`, and protocol documentation are updated to match the implemented design before promotion.

## Promotion rule
- Promote this roadmap item to `Completed` only after both RP2040 and RP2350 firmware builds pass, UART control has passed the agreed automated and hardware-in-the-loop scenarios, USB regression coverage still passes, and the developer signs off that the UART ASCII protocol is suitable for the connected control processor.
