# Testing Strategy

## Current roadmap item
- Add UART control as a second transport for the existing DSPi control surface.
- USB regression, UART parser robustness, and command-equivalence testing are the promotion-critical areas.

## Testing priorities
- Build RP2040 and RP2350 firmware with UART disabled to preserve existing release behavior.
- Build RP2040 and RP2350 firmware with UART enabled and explicit board UART pins.
- Exercise representative shared control requests from both USB and UART: platform query, master volume, per-channel preamp, matrix route, output enable, preset actions, and bulk state transfer.
- Stress malformed UART input: unknown commands, invalid hex, bad lengths, oversized lines, partial lines, and timeout recovery.
- Verify deferred operations still run from the main loop rather than UART or USB interrupt context.

## Research validation
- Confirm selected UART pins do not conflict with S/PDIF/I2S/PDM/MCK pins for each board build.
- Confirm line-buffer sizing is sufficient for full `WireBulkParams` hex transfer when `BSET` is enabled.

## Future application testing
- Add a bridge test that reuses the ASCII UART vocabulary over a simulated network or Bluetooth link.
- Add replayable golden command sessions for bench validation.

## Promotion expectations
- Do not promote the UART-control roadmap item until both target platforms build, USB control behavior is unchanged, UART hardware scenarios pass, and malformed input recovery is demonstrated.
