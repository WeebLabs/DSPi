# Scenarios

## UART Reads USB-Equivalent Values

GIVEN UART control is enabled for the board build
WHEN the control MCU sends `G 0x7F 0 4`
THEN the firmware returns `OK` with the same 4-byte platform packet exposed by `REQ_GET_PLATFORM`.

## UART Writes USB-Equivalent Settings

GIVEN UART control is enabled and the device is running
WHEN the control MCU sends `S 0xD2 0 0000A0C1`
THEN the master volume changes to -20 dB and subsequent USB or UART reads return that value.

## Last Writer Wins

GIVEN USB and UART are both connected
WHEN USB writes a setting and UART then writes the same setting
THEN subsequent reads over both transports return the UART value.

## Deferred Persistence

GIVEN UART control is enabled
WHEN the control MCU sends a preset save, preset load, preset delete, factory reset, or master-volume save command
THEN the UART response acknowledges acceptance and the existing main-loop flash and pipeline reset path performs the operation.

## Malformed UART Input

GIVEN UART control is enabled
WHEN the control MCU sends an unknown command, invalid hex, an oversized line, or a partial line that times out
THEN firmware returns a brief `ERR` response where possible, resets the parser to `Idle`, and leaves live DSP state unchanged.

## Bulk State Transfer

GIVEN UART control is enabled
WHEN the control MCU sends `BGET`
THEN firmware returns the complete `WireBulkParams` payload as hex.

GIVEN UART control is enabled
WHEN the control MCU sends `BSET` with a complete valid `WireBulkParams` hex payload
THEN firmware accepts the transfer and defers live-state application to the main loop.
