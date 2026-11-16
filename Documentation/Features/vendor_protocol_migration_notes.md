# Vendor Protocol Migration Notes (working file)

*Last updated: 2026-11-16 (page 0, finite-value rule, `RTA_BINS`, ROUTE delay note)*

Raw material for the host migration guide (`Documentation/vendor_protocol_migration.md`, plan Phase 7). Each entry is a place where the paged protocol in 1.1.7 behaves differently from the 1.1.6 opcode, beyond the renumbering. They were recorded by the agents who ported each page and checked against the device. This file is folded into the guide and then deleted.

## Rules that apply everywhere

- A rejected SET is an error (STALL on USB). 1.1.6 ACKed most rejected OUT writes and changed nothing.
- SET payloads take the exact documented size. 1.1.6 accepted any payload at least as long as it needed and ignored the rest, and silently dropped short ones. The new result is `VC_ERR_LENGTH`.
- `wValue` is checked as a whole 16-bit address. 1.1.6 often used only the low byte (0x0100 addressed index 0). A set high byte is now `VC_ERR_ADDRESS`.
- A command that takes no address requires `wValue` 0. 1.1.6 ignored `wValue` there.
- A GET for a bad address is `VC_ERR_ADDRESS`. 1.1.6 often answered zeros.
- A NaN or infinite float in a SET is `VC_ERR_VALUE` and stores nothing. This holds in ROUTE (matrix gain, output gain, output and channel delay, preamp, master and user volume), DYN (leveller amount, max gain and gate, loudness reference and intensity), FX (crossfeed cutoff and feed, every subharm and psybass float, every upmix parameter and `UPMIX_CONFIG`) and `FILTER.BAND` (freq, Q, gain). 1.1.6 ignored some of these and stored others.
- The limiter (`DYN.LIMITER_*`) and tube (`FX.TUBE_*`) setters refuse NaN with `VC_ERR_VALUE` but clamp infinity to the end of the range. `MEAS.SIGGEN_CONFIG` sanitises its floats. The bulk SET (`STORE.ALL_PARAMS`) does not validate floats and stores them as sent.
- Out-of-range finite values that 1.1.6 clamped are still clamped.
- Former write-as-read commands (IN transfers that changed state and returned a status byte) are SETs. The status byte is gone. The outcome is ACK or STALL, and the reason is in `SYS.LAST_RESULT`.
- Old `PIN_CONFIG_*` bytes map to results as follows. 0x01 is `VC_ERR_PIN_INVALID`, 0x02 is `VC_ERR_PIN_IN_USE`, 0x03 is `VC_ERR_ADDRESS`, 0x04 is `VC_ERR_STATE`, 0x05 is `VC_ERR_VALUE`. Per-command exceptions are listed below.
- Commands for hardware a platform lacks do not exist there and answer `VC_ERR_UNKNOWN`. 1.1.6 on RP2040 answered zeros or 0x03 for ADAT and upmixer opcodes.

## Page 0

Every request with a zero `wIndex` high byte is refused. USB STALLs at SETUP, UART and I2C return status 0x10, and `SYS.LAST_RESULT` records `VC_ERR_UNKNOWN`. Page 0 has no `GET_PLATFORM` and no `ENTER_BOOTLOADER`, so a 1.1.6 host can neither read the version of a 1.1.7 device nor reboot it into the bootloader. The Microsoft OS 2.0 descriptor request (IN, `bRequest` 0x01, `wIndex` 7) is not a DSPi command and is served as before.

## SYS

| Old | New | Difference |
|---|---|---|
| 0x50 selectors 11, 12 | `USB_AUDIO_STATUS` | Alt setting and mounted are u8 fields (were u32 replies) |
| 0x50 selector 14 | `SYSTEM_STATUS` | Core voltage is u16 |
| 0x50 selector 16 | `SYSTEM_STATUS` | Temperature is i16 centi-degrees |
| 0x50 unknown selector | none | 1.1.6 answered four zero bytes. There is no selector any more |
| 0xB0 | `BUFFER_STATS` | Identical bytes. Each read still advances the packet's sequence counter |
| 0xB1 | `STATS_RESET` | Mask is a u8 payload (was `wValue`). Any bit other than bit 0 is `VC_ERR_VALUE` (was ignored). No reply byte |
| 0x56 SET | `DEVICE_NAME` | Empty payload is `VC_ERR_LENGTH`. More than 32 bytes is `VC_ERR_LENGTH` |
| 0xF0 | `ENTER_BOOTLOADER` | Zero-length SET. Over USB the status stage completes before the reboot |

## STORE

| Old | New | Difference |
|---|---|---|
| 0x90, 0x91, 0x92 | `PRESET_SAVE`, `PRESET_LOAD`, `PRESET_DELETE` | Zero-length SET addressed by slot. A bad slot is `VC_ERR_ADDRESS` (was in-band 0x01) |
| 0x94 | `PRESET_NAME` SET | Bad slot is `VC_ERR_ADDRESS` (was dropped later, silently). 0 or more than 32 bytes is `VC_ERR_LENGTH`. Still stores 31 bytes plus NUL |
| 0x96 | `STARTUP` SET | Exactly 2 bytes. Mode above 1 or slot above 9 is `VC_ERR_VALUE` (was refused later, silently) |
| 0x97 | `STARTUP` GET | 2 bytes `{mode, slot}`. The old third byte, last active slot, is `DIRECTORY` byte 4 |
| 0x98 | `OUTPUT_CONFIG_MODE` SET | Exactly 1 byte |
| 0x52, 0x53 | `OUTPUT_CONFIG_SAVE`, `FACTORY_RESET` | Zero-length SETs, no status byte |
| 0x51 | none | Retired. Read `PRESET_ACTIVE`, then `PRESET_SAVE` to that slot |
| 0xA1 | `ALL_PARAMS` SET | Any length other than 6136 STALLs at SETUP with `VC_ERR_LENGTH` (1.1.6 ACKed and ignored 64 bytes or fewer). A header the apply would refuse STALLs the status stage with `VC_ERR_VALUE` (1.1.6 ACKed and dropped it) |
| 0xA2, 0xA3 | `ALL_PARAMS_CHUNK` | Same semantics, USB only, chunk size 1 to the remainder of the struct (hosts on WinUSB use 4096 or less). Refusals now carry a reason: offset past the end is `VC_ERR_ADDRESS`, a SET running past the end is `VC_ERR_LENGTH`, no session or out of order is `VC_ERR_STATE`, lock held elsewhere is `VC_ERR_BUSY`. A final chunk with a bad header fails with `VC_ERR_VALUE` and applies nothing |

## INPUT

| Old | New | Difference |
|---|---|---|
| 0xE0 | `SOURCE` SET | Value above 6 is `VC_ERR_VALUE`. A disabled optional S/PDIF or ADAT input is `VC_ERR_STATE`. ADAT on RP2040 is `VC_ERR_UNSUPPORTED`. All three were ignored silently |
| 0xE1 plus 0x50 selectors 23, 32, 33 | `STATUS` | One 12-byte struct |
| 0x54 | `SOURCE_NAME` SET | Bad source is `VC_ERR_ADDRESS` (was a no-op) |
| 0xE9 | `SPDIF_RX_ENABLE` | Addresses inputs 1 to 3 only. Input 0 is `VC_ERR_ADDRESS` (1.1.6 answered success for enabling it). Invalid pin and pin in use are separate results (both were 0x02). Disable while active is `VC_ERR_STATE`. The GET is new |
| 0xE4 | `SPDIF_RX_PIN` SET | Index in `wValue`, GPIO in a 1-byte payload |
| 0xE5, 0xF2 with a bad index | `SPDIF_RX_PIN`, `I2S_RX_PIN` GET | `VC_ERR_ADDRESS` (was a zero byte) |
| 0xED | `I2S_RX_RATE` SET | A rate other than 44100, 48000 or 96000 is `VC_ERR_VALUE` (was ignored) |
| 0xEE | `I2S_RX_RATE` GET | 4 bytes, the selected I2S rate. The live rate half is `SYS.SYSTEM_STATUS` |
| 0xF3 | `I2S_RX_CHANNELS` SET | Bad count is `VC_ERR_VALUE` (old 0x01). More pairs than the part has is `VC_ERR_UNSUPPORTED` (old 0x03) |
| 0x68 | `ADAT_RX_ENABLE` SET | Disable while live or pending is `VC_ERR_STATE` (old 0x02). Enable with no pin is `VC_ERR_PIN_INVALID` (old 0x01) |
| 0x6A | `ADAT_RX_PIN` SET | 0xFF clears the pin and is only accepted while disabled, otherwise `VC_ERR_STATE` (old 0x02) |
| 0x68 to 0x6E on RP2040 | none | Do not exist. 1.1.6 on RP2040 stored and returned the clock mode |

## OUTPUT

| Old | New | Difference |
|---|---|---|
| 0xC0 | `TYPE` SET | Slot in `wValue`, type in a 1-byte payload. Bad type is `VC_ERR_VALUE` (old 0x01) |
| 0x7C | `PIN` SET | Pin output in `wValue`, GPIO in a 1-byte payload |
| 0x03 OUT | `LATENCY` SET | Unknown option ID is `VC_ERR_VALUE` (was ignored on USB) |
| 0x03 IN | `LATENCY` GET (new, 1 byte), `LATENCY_STATUS`, `LATENCY_OPTIONS` | The new 1-byte GET returns the queued ID while a change is pending, otherwise the stored one |
| 0xEA | `DAC_MUTE_CONFIG` SET | With enable set, a bad active-low flag, hold time outside 1 to 500 ms or release time above 500 ms is `VC_ERR_VALUE` at once. Pin range and pin conflicts are still checked later, so read back to confirm |
| 0xEC | `DAC_MUTE_TEST` | Zero-length SET. While the feature is disabled it is `VC_ERR_STATE` (old 0x03) |

## CLOCK

| Old | New | Difference |
|---|---|---|
| 0xC2 | `I2S_BCK_PIN` SET | Role in `wValue`, GPIO in a 1-byte payload |
| 0xC3 with a bad role | `I2S_BCK_PIN` GET | `VC_ERR_ADDRESS` (was a zero byte) |
| 0x88 | `I2S_CLOCK_MODE` SET | Value above 1 is `VC_ERR_VALUE` (was ignored) |
| 0xC4 | `MCK_ENABLE` SET | 1-byte payload |
| 0xC6 | `MCK_PIN` SET | 1-byte payload. Still refused while MCK runs, even for the current pin (`VC_ERR_STATE`, old 0x04) |
| 0xC8 | `MCK_MULTIPLIER` SET | 1-byte payload. Value above 1 is `VC_ERR_VALUE` (old 0x01) |

## XPORT

| Old | New | Difference |
|---|---|---|
| 0xF5, 0xF7 | `UART_CONFIG`, `I2C_CONFIG` SET | Scalar fields and GPIO range are checked before anything is queued. A bad enable, notify flag, baud rate (outside 9600 to 1000000) or I2C address (outside 0x08 to 0x77) is `VC_ERR_VALUE`, and a GPIO that is not usable is `VC_ERR_PIN_INVALID`. The live interface is left untouched and `XPORT.STATUS` is not updated. 1.1.6 queued everything and reported later. Pin mux and ownership checks are still deferred and still report through `XPORT.STATUS` |

## ROUTE

| Old | New | Difference |
|---|---|---|
| 0x70, 0x71 | `MATRIX_ROUTE` | Address `(input << 8) \| output` in `wValue` for both directions. Payload is u8 enabled, u8 phase invert, u16 reserved, f32 gain dB. The GET no longer echoes input and output |
| 0x72 | `OUTPUT_ENABLE` SET | An enable that conflicts with the Core 1 mode is `VC_ERR_STATE` (was ignored silently) |
| 0x7B | `OUTPUT_ENABLE_CONFLICT` | An output past the end is `VC_ERR_ADDRESS` (was a zero byte) |
| 0x9B | `CHANNEL_NAME` SET | 1 to 32 bytes. 1.1.6 accepted up to 64 and kept 31. Both store at most 31 |
| 0x44, 0x45 | none | Retired. Use `PREAMP` per input channel |
| 0xD6 | `MASTER_VOLUME_SAVE` | Zero-length SET, no status byte |

A `CHANNEL_DELAY` write to an output channel does not change what `OUTPUT_DELAY` reads back for that output, while an `OUTPUT_DELAY` write updates both. This is unchanged from 1.1.6. Hosts should set output delays through `OUTPUT_DELAY`.

## FILTER

| Old | New | Difference |
|---|---|---|
| 0x42 | `BAND` SET | Address `(channel << 8) \| band` in `wValue`. Payload is u8 type, u8 bypass, u16 qp_x512, f32 freq, f32 Q, f32 gain dB (16 bytes). There is no 18-byte form and no "preserve the stored qp" case, so always send qp. A crossover band ignores qp |
| 0x43 | `BAND` GET | Returns the whole band in the SET layout. The 3-bit parameter selector is gone. Bypass reads as exactly 0 or 1 |
| 0x42, 0xD8 | `BAND`, `BAND_BYPASS` SET | While an earlier band write is still waiting for the main loop, a new one is `VC_ERR_BUSY`. 1.1.6 overwrote the single pending slot and lost the earlier write. Retry after a short delay |
| 0x42, 0x43, 0xD8, 0xD9 | same | A band in the reserved gap, or a crossover band on an input channel, is `VC_ERR_ADDRESS` |

## DYN

| Old | New | Difference |
|---|---|---|
| 0x81 index 0 to 3 | `LIMITER_ENABLE`, `LIMITER_THRESHOLD`, `LIMITER_RELEASE`, `LIMITER_LINK_GROUP` | One command per parameter, output in `wValue`, all four still float32. Output 0xFF means all outputs, on SET only. Bad output is `VC_ERR_ADDRESS` (was ignored) |
| 0x81 index 0x80, 0x81 | `LIMITER_METERS`, `LIMITER_STATUS` | `wValue` must be 0 (the output byte was ignored) |
| 0xB8 | `LEVELLER_SPEED` SET | Value of 3 or more is `VC_ERR_VALUE` (was ignored) |

## FX

| Old | New | Difference |
|---|---|---|
| 0x60 | `CROSSFEED_PRESET` SET | Value above 3 is `VC_ERR_VALUE` (was ignored) |
| 0x3E, 0x3F | `TUBE_*` | One command per parameter, in `TUBE_PARAM_*` order, all float32. There is no index, so no bad index |
| 0x4C, 0x4D | `UPMIX_*` | One command per parameter, in `UPMIX_PARAM_*` order, all float32 |
| 0x4A to 0x4E on RP2040 | none | Do not exist. 1.1.6 answered the GETs with zeros and STALLed the SETs |

## MEAS

| Old | New | Difference |
|---|---|---|
| 0x50 selector 9 | `LEVELS` | Identical bytes |
| 0x83 | `CLIP_FLAGS`, `CLIP_CLEAR` | Read and clear are two requests. A clip that lands between them is lost |
| 0x0A | `RTA_CAPS`, `RTA_BAND_CENTRES` | Old `wValue` n (1 and up) is chunk n-1. A chunk past the table is `VC_ERR_ADDRESS` |
| 0x0B | `RTA_BANDS` | Bad channel is `VC_ERR_ADDRESS`. The read still counts as a keepalive first, as before |
| 0x0C | `RTA_BINS` | At most 256 bytes per request (1.1.6 sent up to 529 in one). Read in chunks by offset. Before the first transform the device returns a frame with a zero header and a 0xFF sequence tail, so a host must check the tail. The handler's `VC_ERR_STATE` for a missing frame is not reachable in practice. An offset at or past the frame length is `VC_ERR_ADDRESS` |
| 0x0E | `RTA_START`, `RTA_STOP`, `RTA_RESET` | Zero-length SETs (old actions 1, 0, 2) |
| 0x0F | `RTA_BANDS_ALL` | Off USB it is `VC_ERR_BLOCKED`. A held bulk lock is `VC_ERR_BUSY` |
| 0xA6 | `SIGGEN_START`, `SIGGEN_STOP`, `SIGGEN_STOP_NOW` | Zero-length SETs (old actions 1, 0, 2). Start with no valid config is `VC_ERR_STATE` |
| 0xA8 | `SIGGEN_CAPS`, `SIGGEN_TYPE_DESC` | Old `wValue` 0xFFFF is `SIGGEN_CAPS`. A bad type index is `VC_ERR_ADDRESS` |
| 0x08, 0xA4 | `RTA_CONFIG`, `SIGGEN_CONFIG` SET | A rejected config STALLed in 1.1.6 and still does, now with `VC_ERR_VALUE`. `SIGGEN_CONFIG` takes exactly 36 bytes |
| 0xB2, 0xB3 | none | Retired. The counters were always zero |

## CS

The preview, Save and Revert model is unchanged. Preview SETs still only stage a record, and the result still arrives in the status packet.

| Old | New | Difference |
|---|---|---|
| 0x84, 0x8B, 0x8D, 0x20, 0x22, 0x24, 0x27, 0x29 | `BINDING`, `SLOT_NAME`, `IR_CMD`, `GROUP`, `MACRO_HEADER`, `MACRO_STEP`, `DISPLAY_CONFIG`, `DISPLAY_PAGE` SET | A bad index is `VC_ERR_ADDRESS` and a wrong length is `VC_ERR_LENGTH`. Neither touches the status packet any more (1.1.6 ACKed and posted INVALID_SLOT or INVALID_VALUE). A SET while one of the same kind is still pending is `VC_ERR_BUSY` and still posts BUSY |
| 0x04, 0x06 | `AUX_STATE`, `AUX_LEVEL` SET | A slot that is not an aux output is `VC_ERR_STATE` and posts INVALID_AUX. A bad slot is `VC_ERR_ADDRESS`. During a revert it is `VC_ERR_BUSY` (was ignored) |
| 0x05, 0x07 | `AUX_STATE`, `AUX_LEVEL` GET | A slot that is not an active aux is `VC_ERR_STATE` (was a STALL) |
| 0x05 `wValue` 0xFFFF | `AUX_ALL` | Own command |
| 0x9D, 0x9E | `SAVE`, `REVERT` | Zero-length SETs (were IN transfers returning 0x01). Busy is `VC_ERR_BUSY` with the same BUSY status |
| 0x8F | `IR_LEARN_ARM`, `IR_LEARN_CANCEL`, `IR_LEARN_STATUS` | Old `wValue` 1, 0, 2. Arm with no receiver is `VC_ERR_STATE` and posts NO_IR. During a revert it is `VC_ERR_BUSY` |
| 0x25 | `MACRO_FIRE`, `MACRO_CANCEL` | Old `wValue` 0xFFFF is `MACRO_CANCEL`. A bad macro is `VC_ERR_ADDRESS`. During a revert it is `VC_ERR_BUSY` |
| 0x22, 0x24 | `MACRO_HEADER`, `MACRO_STEP` GET | New. Each returns exactly what its SET accepts |
| 0x28 | `DISPLAY_CONFIG` GET, `DISPLAY_CAPS` | The old 16-byte reply is split: 4 limit bytes in `DISPLAY_CAPS`, 12 config bytes in `DISPLAY_CONFIG` |
| 0x86 | `CAPS`, `NOUN_DESC` | Old `wValue` 0xFFFF is `CAPS`. A noun number is `NOUN_DESC` |

## DIAG

| Old | New | Difference |
|---|---|---|
| 0x82 | `CLOCK_DIAG` | Exists only in `DSPI_CLOCK_DIAG` builds. Elsewhere it is `VC_ERR_UNKNOWN` (was a zero-length reply). USB only, so UART and I2C get `VC_ERR_BLOCKED` |
| 0x50 selectors 24, 25 | `LOOPBACK_STATS` | Exists only in `DSPI_LOOPBACK` builds (the selectors read zero elsewhere) |

## Shared-state hazard for USB hosts

`SYS.LAST_RESULT` is kept per transport, not per host. Two USB hosts talking to one device (for example DSPi Console and a script) overwrite each other's last result. A host should read it immediately after the STALL it wants to explain, and check that the command ID, direction and `wValue` in the reply match its own request.
