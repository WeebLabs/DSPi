# Roadmap

## Now
- Add feature-equivalent UART control alongside the existing USB vendor control path, using a build-time configurable ASCII protocol for a trusted local control processor.
  - Scope: every setting, query, action, telemetry packet, bulk-state operation, preset operation, pin configuration command, and persistence path exposed over USB must be reachable over UART unless a documented hardware limitation prevents it.
  - Architecture: extract the current USB request handling into a transport-neutral control-command layer that owns parsing typed command intents, validating indices/ranges, applying live DSP state, scheduling deferred flash/pipeline work, and formatting typed results.
  - UART transport: add a line-oriented ASCII protocol with explicit parser states, bounded buffers, command timeouts, structured terse responses, and build-time options for UART instance, pins, baud rate, line ending, RX/TX buffering, and hardware flow-control policy.
  - Concurrency: allow USB and UART control at the same time with last-writer-wins semantics, no duplicated parameter state, and shared deferred work queues for flash and pipeline reset operations.
  - Robustness: malformed, oversized, partial, unknown, or out-of-range UART input must not corrupt live state, block audio processing, wedge the parser, overflow buffers, or starve watchdog/main-loop work.
  - Persistence: UART-initiated setting changes must use the same live-state and flash persistence behavior as the equivalent USB commands, including presets and independent master-volume storage.
  - Testing: add host-side protocol tooling, parser/unit coverage where the code can run off-target, command-equivalence checks against the USB command map, malformed-input/fuzz cases, and documented hardware-in-the-loop UART scenarios for RP2040 and RP2350.
  - Documentation: update the architecture, testing, and scenario documentation as implementation lands so the USB control protocol, UART protocol, shared command state machine, and promotion evidence stay synchronized.

## Later
- Add an optional higher-level control bridge profile for future network and Bluetooth transports that reuses the UART ASCII command vocabulary without binding firmware behavior to any one external link.
- Add versioned protocol discovery and capability reporting so host tools can negotiate supported control commands, platform limits, and build-time UART configuration at runtime.
- Add richer integration tooling for automated bench runs, including replayable command scripts, golden response captures, and latency/error summaries for long-running serial control tests.
- Review whether non-USB firmware update entry should be exposed over UART, and define any additional safety gate needed before allowing a trusted local MCU to request UF2 bootloader entry.

## Completed
- Clarified UART control requirements: feature parity with USB control, ASCII command shape, build-time UART configuration, last-writer-wins multi-transport behavior, read/write support, shared persistence, trusted local-MCU threat model, robust structured errors, explicit parser state, and relevant test tooling.
