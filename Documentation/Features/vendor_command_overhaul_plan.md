# Vendor Command Overhaul Plan

*Last updated: 2026-11-17 (status line, 8.13 RTA test, 8.14 short guide, decisions on `CLOCK_DIAG`, `VC_F_RESTART` and the RTA test moved from 14.1 to 14). Previously 2026-11-16 (status line, 8.14 As built (Phase 7), Phase 7 as done); 2026-11-16 (status line, 8.13 As built (Phase 6), 7.5 and Phase 6 as done, items awaiting a decision in 14.1); 2026-11-16 (8.12 As built (Phase 5), 5.1 and 6.1 `SPDIF_TX_STATS` wording, Phase 5 text); 2026-11-16 (8.11, Phase 4)*

Status: approved by the user on 2026-11-15 with the decisions in section 14. Phases 0 to 7 are done, and sections 8.8 to 8.14 record what was built. The host migration guide is complete, with four host appendices (8.14), and a short client update guide sits beside it. The Windows run of the `wIndex` test is still owed by the user (11.1). What remains is release readiness (Phase 8). The open questions were decided on 2026-11-17 (14). Branch `vendor-command-overhaul`, cut from `main` at fb9a254 (firmware 1.1.6).

## 1. TL;DR

DSPi has used 246 of its 256 vendor command numbers, and about four are really free. This plan gives the protocol a second dimension so that it cannot run out again, and it tidies the rules at the same time.

Every command gets a two-part address. The page says which subsystem (filters, inputs, presets and so on). The command number says which thing inside that subsystem. The page travels in the high byte of `wIndex`, which no host uses today. That gives 256 pages of 240 usable commands each.

Three rules keep the new space clean.

1. One number per thing. Reading and writing the same setting use the same number, and the transfer direction says which.
2. `wValue` only says which one (channel, band, slot). It never picks an operation, and it never carries a value. Values go in the payload.
3. Reads never change anything. Anything that changes the device is a write.

The firmware side becomes a table. Each row names a command, its read and write handlers, its payload sizes and its restrictions. One dispatcher checks length, direction, address and transport before any handler runs. The same table generates the command reference and a machine-readable registry for the host apps, and the device can report it at runtime so hosts detect features without comparing version numbers.

This is a clean break. The new scheme ships in firmware v1.1.7-beta1, the release contributors were told would overhaul the opcodes, and the old numbers stop working in that release. Old hosts send a zero in the `wIndex` high byte, so they land on page 0, which answers nothing.

What it means in practice: every host must be ported for that release. This work delivers the firmware, the test harness and a comprehensive migration guide. It does not change any host app. The refactor is proven without shipping any compatibility code. A byte-for-byte capture of 1.1.6 replies was the reference, and a translation layer inside the test harness let the existing 173 tests run against the new handlers until Phase 6 ported them to native paged calls and deleted the layer.

## 2. Facts verified for this plan

Each item was checked against the tree at fb9a254 or the named repo.

| Fact | Evidence |
|---|---|
| 246 `REQ_*` opcodes are defined, and `MS_VENDOR_CODE` is 0x01 | `firmware/DSPi/config.h` |
| No handler reads `wIndex`. The only `wIndex` test is the MS OS 2.0 intercept (`wIndex == 7`) | `vendor_commands.c:4574`, `:4781` |
| Neither the recipient bits nor `wIndex` filter dispatch today | `vendor_commands.c` entry point |
| `tud_vendor_control_xfer_cb` runs in main-loop context (`tud_task` at `main.c:2181`). Only the EP0 data copy runs in the USB ISR | `main.c`, `tusb_config.h:22` |
| A zero-length OUT is ACKed at SETUP and never reaches a handler. This is why 34 actions are issued as reads today | `vendor_commands.c` SETUP path |
| TinyUSB STALLs the status stage when the DATA-stage callback returns false. The current code always returns true | `usbd_control.c:193-202` |
| UART and I2C frames already carry a 16-bit `wIndex` and an explicit SET or GET type | `uart_control.c:267`, `i2c_control.c:290` |
| The I2C ISR calls `vendor_is_bulk_set()` | `i2c_control.c:115` |
| Control Surfaces bindings store nouns, not opcodes. The noun to opcode map is a switch with 51 opcodes | `control_surfaces_nouns.c:552-797` |
| Notifications carry bulk offsets, never opcodes | `notify.c`, Console `SnapshotCache.swift:64-70` |
| `WireBulkParams` is V32, 6136 bytes | `bulk_params.h:478`, Console `Constants.swift:725` |
| UAC1 requests are class-type and never reach the vendor callback | `vendor_commands.c` comment at the MS intercept, `usb_audio.c` |
| The Console sends `wIndex` 0 on about 126 call sites and 2 on about 153. No site sends anything else | Console `Commands.swift`, `StatsView.swift` |
| DSPi Link frames carry a 16-bit `wIndex` untouched, but its policy table and routing key on `bRequest` alone | Console `Link/LinkFrames.swift:46-76`, `Link/policy/commands.json` |
| IN request with `bRequest` 0x10 STALLs on firmware 1.1.6 (0x10 exists only in the SET switch) | `vendor_commands.c:993` |

Corrections to the brief:

- The Console's StatsView mostly sends `wIndex` 0, not 2. Both values have a zero high byte, so every 1.1.6 host lands on page 0.
- A Windows Console exists (`WeebLabs/DSPi-Console-Windows`, C#, WinUSB). It is not on this machine and was not inspected. iOS and Linux Consoles and DSPi Terminal also exist locally.
- A separate `DSPi-Link` repo holds a TypeScript client, a web app and an ESP32 bridge. They share the `bRequest`-keyed policy file.
- `bench/input-batch` has no unique commits. The bench command lives in `stash@{1}` and the untracked `batch_bench.h`.
- The WebUSB spec (design only, not implemented) reserves `bRequest` 0x02 as `WEBUSB_VENDOR_CODE`. The AC-3 branch claims 0x02 too.

## 3. Addressing model

### 3.1 The request

| Field | Meaning |
|---|---|
| `bmRequestType` | Vendor type, interface recipient. `0xC1` is GET and `0x41` is SET. Unchanged |
| `bRequest` | Command number within the page, `0x10`-`0xFF` |
| `wIndex` high byte | Page, `0x01`-`0xFF`. Page `0x00` is never used and every request to it is refused |
| `wIndex` low byte | Interface number, always 2 on USB. UART, I2C and Link send it and the firmware ignores it |
| `wValue` | Address only. Zero when the command addresses nothing |
| Payload | Values, little-endian |

In source and docs a command is written `PAGE.NAME`, for example `FILTER.BAND`. Its 16-bit ID is `(page << 8) | cmd`.

### 3.2 Conventions for contributors

These are the rules a new command must follow. The generator in section 8 enforces the mechanical ones.

1. **One number per thing.** A setting that can be read and written is one command with a GET and a SET handler. Never define a pair.
2. **GET does not change state.** No GET changes configuration, runtime state or flash. Two narrow exceptions exist: a read may refresh a keepalive timer (the RTA auto-off), and a chunked read may take the bulk lock. Clear-on-read is not allowed.
3. **Anything that changes state is a SET.** This includes actions with no value, which are zero-length SETs. There is no write-as-read in the new pages.
4. **`wValue` is an address.** It names which instance: a channel, an output, a slot, a band, a byte offset. It never selects an operation, a parameter or a response shape. Two-level addresses pack as `(major << 8) | minor`.
5. **Values go in the payload.** A GPIO number, a mode and an enable flag are values.
6. **A new operation gets a new number.** Do not add a selector to an existing command. Numbers are plentiful.
7. **One command, one shape.** A command has exactly one GET reply layout and one SET payload layout.
8. **GET and SET are symmetric.** For a read-write command, the GET reply is a valid SET payload. Read-only status and capability data are separate GET-only commands.
9. **Shapes only grow at the end.** A reply or payload may gain trailing fields in a later page revision. Hosts ask for the length they know, and the device accepts the shorter SET lengths it has ever documented. Any other change of shape needs a new number.
10. **Numbers are never reused.** A retired command stays in the table as a tombstone.
11. **`0x00`-`0x0F` are reserved in every page.** 0x01 is the Microsoft OS 2.0 vendor code and 0x02 is set aside for WebUSB. The rest are held for protocol-level use.
12. **Features own blocks of 16.** Within a page, each feature starts on a `0x_0` boundary and grows upward. Leave the gap.
13. **A command lives with the state it touches.** The Presets page holds commands about the preset store itself. A feature's own persistence switch stays with the feature.
14. **Failure is explicit.** A rejected SET STALLs on USB and returns a result code on UART and I2C. The reason is readable from `SYS.LAST_RESULT`. Nothing is silently dropped in the new pages.
15. **`0xFF` in an index position means "all"** only on a SET, and only on commands flagged for it.

### 3.3 Result codes

One enum covers every command. It replaces the separate `PIN_CONFIG_*`, `PRESET_*` and `FLASH_*` status bytes in the new pages.

| Code | Name | Meaning |
|---|---|---|
| 0x00 | `VC_OK` | Done, or accepted for deferred work |
| 0x10 | `VC_ERR_UNKNOWN` | No such page or command on this build |
| 0x11 | `VC_ERR_DIRECTION` | Command has no handler for this direction |
| 0x12 | `VC_ERR_LENGTH` | SET payload length outside the allowed range |
| 0x13 | `VC_ERR_ADDRESS` | `wValue` out of range |
| 0x14 | `VC_ERR_VALUE` | A payload field is out of range |
| 0x15 | `VC_ERR_UNSUPPORTED` | Not available on this platform |
| 0x16 | `VC_ERR_BLOCKED` | Not allowed on this transport |
| 0x17 | `VC_ERR_BUSY` | Try again shortly |
| 0x18 | `VC_ERR_STATE` | Not allowed in the current state (output active, slot empty, feature disabled) |
| 0x19 | `VC_ERR_PIN_INVALID` | GPIO not usable for this function |
| 0x1A | `VC_ERR_PIN_IN_USE` | GPIO already claimed |

Codes `0x01`-`0x07` remain the transport-level `CTRL_STATUS_*` values on UART and I2C. For paged commands those transports return the `VC_*` code in the same status byte, so an external controller gets the reason in the response frame. USB hosts see a STALL and may read `SYS.LAST_RESULT`, which is kept per transport.

A deferred command (preset load, flash writes, pin moves) returns `VC_OK` when it passes validation and is queued. Completion keeps its existing channels: notifications, the Control Surfaces status packet and readback. The table marks these commands so the docs and the capability data say so.

## 4. Page map

| Page | Mnemonic | Contents |
|---|---|---|
| 0x00 | (none) | Empty. Every request is refused, apart from the Microsoft OS 2.0 descriptor request |
| 0x01 | `SYS` | Discovery, identity, device name, last result, bootloader, system and stream telemetry |
| 0x02 | `STORE` | Preset slots, directory, startup, IO config persistence, factory reset, bulk parameters |
| 0x03 | `INPUT` | Source select, S/PDIF RX, I2S RX, ADAT RX, LG Sound Sync, source names |
| 0x04 | `OUTPUT` | Output type and pins, output latency, ADAT TX, DAC hardware mute |
| 0x05 | `ROUTE` | Matrix, per-output controls, delays, channel names, preamp, master and user volume |
| 0x06 | `FILTER` | EQ and crossover bands, bypasses |
| 0x07 | `DYN` | Limiter, leveller, loudness |
| 0x08 | `FX` | Crossfeed, subharmonic synth, psychoacoustic bass, tube, upmixer |
| 0x09 | `MEAS` | Level meters, clip flags, spectrum analyser, signal generator |
| 0x0A | `CS` | Control Surfaces |
| 0x0B | `XPORT` | UART and I2C control transports |
| 0x0C | `CLOCK` | I2S clock pins and modes, MCK |
| 0x0D-0x7F | | Unallocated, for future subsystems |
| 0x80-0xDF | | Reserved |
| 0xE0-0xEF | | Private use for forks and ports. Never allocated upstream |
| 0xF0 | `DIAG` | Bench and diagnostic commands. No stability promise. Present only in bench builds |
| 0xF1-0xFF | | Reserved |

Changes from the starting map, with reasons:

- **`CLOCK` (0x0C) is new.** BCK pins, clock-pin mode, I2S clock mode and MCK are shared between I2S input and output, so they fit neither page.
- **`DIAG` (0xF0) is new.** Clock diagnostics, the loop profiler and the loopback counters are unstable bench tools. Keeping them out of the product pages means the stable pages never carry a command that exists only in some builds.
- **Telemetry counters live in `SYS`, signal measurement in `MEAS`.** Buffer and pipeline statistics describe device health. Meters, the analyser and the generator describe the audio.
- **There is no `NAME` page.** Every name lives with the thing it names. The device name is in `SYS`, input source names in `INPUT`, channel names in `ROUTE`, preset names in `STORE` and slot names in `CS`. A three-command page would have been the only one organized by data type and not by subsystem.

### 4.1 Experimental branches

`overclock`, `ac3-dts-decode` and the batch bench are experiments for possible future features. They are not merged and they get no reserved numbers. If one is taken up later, it is ported then and takes numbers from the free space in the page it belongs to, which every page has.

`bRequest` 0x02 stays reserved in every page for the WebUSB vendor code, so the WebUSB spec stands as written.

## 5. New command set

Sizes are bytes. "G" is GET, "S" is SET, "S0" is a zero-length action. "Addr" is the `wValue` meaning. Payloads marked "unchanged" keep the 1.1.6 layout from `commands.md`.

### 5.1 `SYS` (0x01)

| Cmd | Name | Dir | Addr | Size | Notes |
|---|---|---|---|---|---|
| 0x10 | `PROTOCOL_INFO` | G | none | 16 | See 5.14 |
| 0x11 | `PAGE_LIST` | G | none | 2 per page | `{page, revision}` for each implemented page |
| 0x12 | `PAGE_COMMANDS` | G | page | 64 | 256-bit GET bitmap then 256-bit SET bitmap |
| 0x13 | `COMMAND_INFO` | G | `(page << 8) \| cmd` | 8 | Flags, sizes and address kind from the table |
| 0x14 | `LAST_RESULT` | G | none | 8 | Last request on this transport: ID, direction, result |
| 0x20 | `PLATFORM` | G | none | 7 | Unchanged layout |
| 0x21 | `SERIAL` | G | none | 16 | Unchanged |
| 0x22 | `BUILD_INFO` | G | none | 64 | Unchanged |
| 0x23 | `CORE1_MODE` | G | none | 1 | Unchanged |
| 0x24 | `DEVICE_NAME` | G, S | none | 32, 1-32 | Deferred, flash |
| 0x30 | `ENTER_BOOTLOADER` | S0 | none | 0 | |
| 0x40 | `SYSTEM_STATUS` | G | none | 16 | u32 clk_sys Hz, u32 sample rate Hz, u16 core mV, i16 temperature cdeg, u8 cpu0, u8 cpu1, u16 reserved |
| 0x41 | `USB_AUDIO_STATUS` | G | none | 16 | u32 packets, u32 ring overruns, u32 worst packet gap us, u8 alt, u8 mounted, u16 reserved |
| 0x42 | `BUFFER_STATS` | G | none | 44 | Unchanged `BufferStatsPacket` |
| 0x43 | `PIPELINE_STATS` | G | none | 24 | u32 ring underruns, ring drops, min prewrite, input gap us, bg over budget, reserved |
| 0x44 | `PDM_STATS` | G | none | 16 | u32 ring overruns, ring underruns, DMA overruns, DMA underruns |
| 0x45 | `SPDIF_TX_STATS` | G | none | 24 | u32 USB audio packet gaps of 2 to 50 ms (`spdif_underruns`), total starvations, starvations for instances 0-3 |
| 0x4F | `STATS_RESET` | S | none | 1 | Bitmask. Bit 0 resets watermarks and worst-gap fields |

### 5.2 `STORE` (0x02)

| Cmd | Name | Dir | Addr | Size | Notes |
|---|---|---|---|---|---|
| 0x10 | `PRESET_SAVE` | S0 | slot | 0 | Deferred, flash |
| 0x11 | `PRESET_LOAD` | S0 | slot | 0 | Deferred, restart |
| 0x12 | `PRESET_DELETE` | S0 | slot | 0 | Deferred, flash |
| 0x13 | `PRESET_NAME` | G, S | slot | 32, 1-32 | Deferred, flash |
| 0x14 | `PRESET_ACTIVE` | G | none | 1 | |
| 0x15 | `DIRECTORY` | G | none | 7 | Unchanged |
| 0x16 | `STARTUP` | G, S | none | 2 | `{mode, slot}`. Last-active slot is in `DIRECTORY` |
| 0x20 | `OUTPUT_CONFIG_MODE` | G, S | none | 1 | Deferred, flash |
| 0x21 | `OUTPUT_CONFIG_SAVE` | S0 | none | 0 | Deferred, flash |
| 0x30 | `FACTORY_RESET` | S0 | none | 0 | Deferred, restart |
| 0x40 | `ALL_PARAMS` | G, S | none | 6252 | `WireBulkParams` V35. The overhaul left it untouched at V32, 6136 bytes. Bulk buffer |
| 0x41 | `ALL_PARAMS_CHUNK` | G, S | byte offset | 1 to the rest of the struct | USB only, bulk buffer. WinUSB hosts use 4096 or less per chunk |

### 5.3 `INPUT` (0x03)

| Cmd | Name | Dir | Addr | Size | Notes |
|---|---|---|---|---|---|
| 0x10 | `SOURCE` | G, S | none | 1 | Deferred, restart |
| 0x11 | `STATUS` | G | none | 12 | u8 source, u8 active channels, u16 reserved, u32 measured rate mHz, i32 VCXO trim ppm x100 |
| 0x12 | `SOURCE_NAME` | G, S | source ID | 32, 1-32 | Deferred, flash |
| 0x20 | `SPDIF_RX_STATUS` | G | none | 16 | Unchanged |
| 0x21 | `SPDIF_RX_CH_STATUS` | G | none | 24 | Unchanged |
| 0x22 | `SPDIF_RX_PIN` | G, S | input 0-3 | 1 | GPIO, 0xFF resets to default |
| 0x23 | `SPDIF_RX_ENABLE` | G, S | input 1-3 | 1 | |
| 0x24 | `SPDIF_RX_CONFIG` | G | none | 6 | Unchanged |
| 0x30 | `I2S_RX_PIN` | G, S | pair 0-3 | 1 | |
| 0x31 | `I2S_RX_CHANNELS` | G, S | none | 1 | |
| 0x32 | `I2S_RX_RATE` | G, S | none | 4 | Selected I2S rate. Live rate is in `SYS.SYSTEM_STATUS` |
| 0x40 | `ADAT_RX_ENABLE` | G, S | none | 1 | RP2350 |
| 0x41 | `ADAT_RX_PIN` | G, S | none | 1 | RP2350 |
| 0x42 | `ADAT_RX_CLOCK_MODE` | G, S | none | 1 | RP2350 |
| 0x43 | `ADAT_RX_STATUS` | G | none | 20 | RP2350 |
| 0x50 | `LG_SOUND_SYNC_ENABLE` | G, S | none | 1 | |
| 0x51 | `LG_SOUND_SYNC_STATUS` | G | none | 16 | |

### 5.4 `OUTPUT` (0x04)

| Cmd | Name | Dir | Addr | Size | Notes |
|---|---|---|---|---|---|
| 0x10 | `TYPE` | G, S | slot | 1 | Deferred, restart |
| 0x11 | `PIN` | G, S | pin output | 1 | Deferred, restart |
| 0x20 | `LATENCY` | G, S | none | 1 | Requested option ID. Deferred, flash, restart |
| 0x21 | `LATENCY_STATUS` | G | none | 12 | Unchanged `OutputLatencyStatus` |
| 0x22 | `LATENCY_OPTIONS` | G | none | 8 per option | Unchanged entries |
| 0x30 | `ADAT_TX_ENABLE` | G, S | none | 1 | RP2350 |
| 0x31 | `ADAT_TX_PIN` | G, S | none | 1 | RP2350 |
| 0x32 | `ADAT_TX_STATUS` | G | none | 8 | RP2350 |
| 0x40 | `DAC_MUTE_CONFIG` | G, S | none | 16 | Deferred, flash |
| 0x41 | `DAC_MUTE_TEST` | S0 | none | 0 | |

### 5.5 `ROUTE` (0x05)

| Cmd | Name | Dir | Addr | Size | Notes |
|---|---|---|---|---|---|
| 0x10 | `MATRIX_ROUTE` | G, S | `(input << 8) \| output` | 8 | u8 enabled, u8 phase invert, u16 reserved, f32 gain dB |
| 0x20 | `OUTPUT_ENABLE` | G, S | output | 1 | |
| 0x21 | `OUTPUT_GAIN` | G, S | output | 4 | |
| 0x22 | `OUTPUT_MUTE` | G, S | output | 1 | |
| 0x23 | `OUTPUT_DELAY` | G, S | output | 4 | |
| 0x24 | `OUTPUT_ENABLE_CONFLICT` | G | output | 1 | Was `GET_CORE1_CONFLICT` |
| 0x30 | `CHANNEL_DELAY` | G, S | channel | 4 | |
| 0x31 | `CHANNEL_NAME` | G, S | channel | 32, 1-32 | |
| 0x40 | `PREAMP` | G, S | input channel | 4 | |
| 0x50 | `MASTER_VOLUME` | G, S | none | 4 | |
| 0x51 | `MASTER_VOLUME_MODE` | G, S | none | 1 | Deferred, flash |
| 0x52 | `MASTER_VOLUME_SAVE` | S0 | none | 0 | Deferred, flash |
| 0x53 | `MASTER_VOLUME_SAVED` | G | none | 4 | |
| 0x60 | `USER_VOLUME` | G, S | none | 4 | |
| 0x61 | `USER_MUTE` | G, S | none | 1 | |

### 5.6 `FILTER` (0x06)

| Cmd | Name | Dir | Addr | Size | Notes |
|---|---|---|---|---|---|
| 0x10 | `BAND` | G, S | `(channel << 8) \| band` | 16 | u8 type, u8 bypass, u16 qp_x512, f32 freq, f32 Q, f32 gain dB |
| 0x11 | `BAND_BYPASS` | G, S | `(channel << 8) \| band` | 1 | |
| 0x12 | `EQ_BYPASS` | G, S | none | 1 | Master EQ bypass |

`BAND` replaces the write-only 16 or 18 byte `EqParamPacket` and the scalar-at-a-time read. The address moves out of the payload, the Linkwitz `qp` fits inside 16 bytes, and a whole band reads back in one transfer.

### 5.7 `DYN` (0x07)

| Cmd | Name | Dir | Addr | Size |
|---|---|---|---|---|
| 0x10 | `LIMITER_ENABLE` | G, S | output (0xFF = all on SET) | 4 (f32, as today) |
| 0x11 | `LIMITER_THRESHOLD` | G, S | output (0xFF = all on SET) | 4 |
| 0x12 | `LIMITER_RELEASE` | G, S | output (0xFF = all on SET) | 4 |
| 0x13 | `LIMITER_LINK_GROUP` | G, S | output (0xFF = all on SET) | 4 (f32, as today) |
| 0x18 | `LIMITER_METERS` | G | none | 2 per output |
| 0x19 | `LIMITER_STATUS` | G | none | 4 |
| 0x20 | `LEVELLER_ENABLE` | G, S | none | unchanged |
| 0x21 | `LEVELLER_AMOUNT` | G, S | none | unchanged |
| 0x22 | `LEVELLER_SPEED` | G, S | none | unchanged |
| 0x23 | `LEVELLER_MAX_GAIN` | G, S | none | unchanged |
| 0x24 | `LEVELLER_LOOKAHEAD` | G, S | none | unchanged |
| 0x25 | `LEVELLER_GATE` | G, S | none | unchanged |
| 0x26 | `LEVELLER_MASKS` | G, S | none | 2 |
| 0x30 | `LOUDNESS_ENABLE` | G, S | none | unchanged |
| 0x31 | `LOUDNESS_REF` | G, S | none | unchanged |
| 0x32 | `LOUDNESS_INTENSITY` | G, S | none | unchanged |
| 0x33 | `LOUDNESS_MASK` | G, S | none | 2 |

### 5.8 `FX` (0x08)

| Cmd | Name | Dir | Size |
|---|---|---|---|
| 0x10-0x15 | `CROSSFEED_ENABLE`, `_PRESET`, `_FREQ`, `_FEED`, `_ITD`, `_OUTPUTS` | G, S | unchanged |
| 0x20-0x2B | `SUBHARM_ENABLE`, `_LOW`, `_HIGH`, `_TOP`, `_BOOST`, `_MASK`, `_SELECT`, `_DEPTH`, `_HOLD`, `_CEILING`, `_LINK`, `_SOLO` | G, S | unchanged |
| 0x2E | `SUBHARM_HEADROOM` | G | 4 |
| 0x2F | `SUBHARM_METER` | G | 2 per output |
| 0x40-0x46 | `PSYBASS_ENABLE`, `_CUTOFF`, `_HARMONICS`, `_DRIVE`, `_CHARACTER`, `_ORIGINAL`, `_MASK` | G, S | unchanged |
| 0x60-0x6D | `TUBE_ENABLE`, `_OUTPUT_MASK`, `_TYPE`, `_DRIVE`, `_BIAS`, `_ASYM`, `_HARDNESS`, `_SAG`, `_RECTIFIER`, `_XFMR_ENABLE`, `_XFMR_DAMPING`, `_XFMR_RES`, `_MIX`, `_TRIM` | G, S | 4 (f32, as today) |
| 0x80 | `UPMIX_CONFIG` | G, S | 44 |
| 0x81 | `UPMIX_STATUS` | G | 16 |
| 0x90-0x9D | `UPMIX_ENABLE`, `_CENTER_MODE`, `_SURROUND_MODE`, `_STRENGTH`, `_CENTER_WIDTH`, `_THRESHOLD`, `_ATTACK`, `_RELEASE`, `_DET_HPF`, `_SUR_DELAY`, `_SUR_HPF`, `_SUR_LPF`, `_DECORR`, `_PRESENCE` | G, S | 4 (f32, as today) |

No command in this page takes an address. Tube and upmixer commands are in `TUBE_PARAM_*` and `UPMIX_PARAM_*` order, and the upmixer page block is RP2350 only.

### 5.9 `MEAS` (0x09)

| Cmd | Name | Dir | Addr | Size | Notes |
|---|---|---|---|---|---|
| 0x10 | `LEVELS` | G | none | 2N+7 | The combined packet from `GET_STATUS` 9 |
| 0x11 | `CLIP_FLAGS` | G | none | 4 | Does not clear |
| 0x12 | `CLIP_CLEAR` | S0 | none | 0 | |
| 0x20 | `RTA_CONFIG` | G, S | none | 12 | |
| 0x21 | `RTA_CAPS` | G | none | 16 | |
| 0x22 | `RTA_BAND_CENTRES` | G | chunk from 0 | up to 64 | |
| 0x23 | `RTA_BANDS` | G | channel at the configured tap | 82 | Keepalive |
| 0x24 | `RTA_BINS` | G | byte offset | variable | Keepalive |
| 0x25 | `RTA_STATUS` | G | none | 24 | |
| 0x26 | `RTA_BANDS_ALL` | G | none | 82 per live channel | USB only, bulk buffer |
| 0x28 | `RTA_START` | S0 | none | 0 | |
| 0x29 | `RTA_STOP` | S0 | none | 0 | |
| 0x2A | `RTA_RESET` | S0 | none | 0 | Clears averages and peaks |
| 0x40 | `SIGGEN_CONFIG` | G, S | none | 36 | |
| 0x41 | `SIGGEN_STATUS` | G | none | 16 | |
| 0x42 | `SIGGEN_CAPS` | G | none | 8 | |
| 0x43 | `SIGGEN_TYPE_DESC` | G | type index | 62 | |
| 0x48 | `SIGGEN_START` | S0 | none | 0 | |
| 0x49 | `SIGGEN_STOP` | S0 | none | 0 | |
| 0x4A | `SIGGEN_STOP_NOW` | S0 | none | 0 | |

### 5.10 `CS` (0x0A)

| Cmd | Name | Dir | Addr | Size |
|---|---|---|---|---|
| 0x10 | `CAPS` | G | none | 52 |
| 0x11 | `NOUN_DESC` | G | noun | 12 |
| 0x12 | `STATUS` | G | none | 41 |
| 0x13 | `EXT_STATUS` | G | none | 24 |
| 0x18 | `SAVE` | S0 | none | 0 |
| 0x19 | `REVERT` | S0 | none | 0 |
| 0x20 | `BINDING` | G, S | slot | 24 |
| 0x21 | `SLOT_NAME` | G, S | slot | 32, 1-32 |
| 0x30 | `IR_CMD` | G, S | sub-slot | 16 |
| 0x31 | `IR_LEARN_STATUS` | G | none | unchanged |
| 0x32 | `IR_LEARN_ARM` | S0 | none | 0 |
| 0x33 | `IR_LEARN_CANCEL` | S0 | none | 0 |
| 0x40 | `GROUP` | G, S | group | 40 |
| 0x50 | `MACRO` | G | macro | 132 |
| 0x51 | `MACRO_HEADER` | G, S | macro | 36 |
| 0x52 | `MACRO_STEP` | G, S | `(step << 8) \| macro` | 12 |
| 0x58 | `MACRO_FIRE` | S0 | macro | 0 |
| 0x59 | `MACRO_CANCEL` | S0 | none | 0 |
| 0x60 | `AUX_STATE` | G, S | slot | 1 |
| 0x61 | `AUX_LEVEL` | G, S | slot | 2 |
| 0x62 | `AUX_ALL` | G | none | 48 |
| 0x70 | `DISPLAY_CONFIG` | G, S | none | 12 |
| 0x71 | `DISPLAY_CAPS` | G | none | 4 |
| 0x72 | `DISPLAY_PAGE` | G, S | display page | 4 |
| 0x73 | `DISPLAY_STATUS` | G | none | 8 |

### 5.11 `XPORT` (0x0B)

| Cmd | Name | Dir | Size | Notes |
|---|---|---|---|---|
| 0x10 | `UART_CONFIG` | G, S | 8 | SET is USB only |
| 0x11 | `I2C_CONFIG` | G, S | 8 | SET is USB only |
| 0x12 | `STATUS` | G | 8 | |

### 5.12 `CLOCK` (0x0C)

| Cmd | Name | Dir | Addr | Size | Notes |
|---|---|---|---|---|---|
| 0x10 | `I2S_BCK_PIN` | G, S | role 0-1 | 1 | |
| 0x11 | `I2S_CLOCK_PIN_MODE` | G, S | none | 1 | |
| 0x12 | `I2S_CLOCK_MODE` | G, S | none | 1 | Deferred, restart |
| 0x13 | `I2S_SLAVE_STATUS` | G | none | 16 | |
| 0x20 | `MCK_ENABLE` | G, S | none | 1 | |
| 0x21 | `MCK_PIN` | G, S | none | 1 | |
| 0x22 | `MCK_MULTIPLIER` | G, S | none | 1 | |

### 5.13 `DIAG` (0xF0)

| Cmd | Name | Dir | Build flag |
|---|---|---|---|
| 0x10 | `CLOCK_DIAG` | G | `DSPI_CLOCK_DIAG` |
| 0x20 | `LOOPBACK_STATS` | G (8 B) | `DSPI_LOOPBACK` |
| 0x30-0x34 | `LOOP_PROF_SECTION`, `_JOB`, `_COMMAND`, `_BG_PACE`, `_RESET` | G, G, G, G and S, S0 | `DSPI_LOOP_PROF` |

A command whose build flag is off is absent from the table. It returns `VC_ERR_UNKNOWN` and does not appear in `PAGE_COMMANDS`. This replaces the "zero-length reply in release builds" behaviour of 0x82.

### 5.14 Discovery

`SYS.PROTOCOL_INFO` reply (16 bytes):

| Offset | Type | Field |
|---|---|---|
| 0 | u8[4] | Magic `"DSPi"` |
| 4 | u8 | Protocol major. 1 for this design. Bumps only on an incompatible change |
| 5 | u8 | Protocol minor |
| 6 | u8 | Number of implemented pages |
| 7 | u8 | Flags. Reserved, 0 |
| 8 | u16 | Largest non-bulk SET payload (64) |
| 10 | u16 | Largest non-bulk GET reply |
| 12 | u32 | Reserved, 0 |

How a host starts a session:

1. Send GET `SYS.PROTOCOL_INFO` (page 0x01, command 0x10).
2. A reply that starts with the magic means the paged protocol is available. Read `PAGE_LIST` and gate features on page revisions or on `PAGE_COMMANDS` bits.
3. A STALL means firmware 1.1.6 or older. Such firmware still answers its own `GET_PLATFORM` (`bRequest` 0x7F, `wIndex` 0 or 2), so a host can read the version and offer the update. Never send any other paged command to such a device.

Step 3 matters because old firmware ignores `wIndex` and would run a paged command as whatever legacy opcode shares its number. The probe itself is safe: an IN request with `bRequest` 0x10 has no handler on old firmware and STALLs.

A page revision increases whenever a command is added to the page or a shape grows. Hosts compare revisions or test bits. They never compare firmware versions to decide whether a feature exists.

## 6. Old to new mapping

Kinds used below:

- **Fold.** A SET and GET pair becomes one command. Address and payload unchanged unless noted.
- **Act.** A write-as-read becomes a SET. The value moves from `wValue` to the payload, or the SET is zero-length.
- **Split.** A multiplexed command becomes several commands.
- **Same.** Only the number changes.
- **Retire.** No new command. Gone in 1.1.7-beta1.

| Old | Old name | Kind | New |
|---|---|---|---|
| 0x03 | `OUTPUT_LATENCY` | Split | SET and new 1-byte GET: `OUTPUT.LATENCY`. GET `wValue` 0: `OUTPUT.LATENCY_STATUS`. GET `wValue` 1: `OUTPUT.LATENCY_OPTIONS` |
| 0x04, 0x05 | `SET/GET_CS_AUX_STATE` | Fold, Split | `CS.AUX_STATE`. GET `wValue` 0xFFFF: `CS.AUX_ALL` |
| 0x06, 0x07 | `SET/GET_CS_AUX_LEVEL` | Fold | `CS.AUX_LEVEL` |
| 0x08, 0x09 | `RTA_SET/GET_CONFIG` | Fold | `MEAS.RTA_CONFIG` |
| 0x0A | `RTA_GET_CAPS` | Split | `wValue` 0: `MEAS.RTA_CAPS`. `wValue` n: `MEAS.RTA_BAND_CENTRES` chunk n-1 |
| 0x0B | `RTA_GET_BANDS` | Same | `MEAS.RTA_BANDS` |
| 0x0C | `RTA_GET_BINS` | Same | `MEAS.RTA_BINS` |
| 0x0D | `RTA_GET_STATUS` | Same | `MEAS.RTA_STATUS` |
| 0x0E | `RTA_CONTROL` | Act, Split | 0: `MEAS.RTA_STOP`. 1: `MEAS.RTA_START`. 2: `MEAS.RTA_RESET` |
| 0x0F | `RTA_GET_BANDS_ALL` | Same | `MEAS.RTA_BANDS_ALL` |
| 0x10, 0x11 | `SET/GET_SUBHARM` | Fold | `FX.SUBHARM_ENABLE` |
| 0x12, 0x13 | `SET/GET_SUBHARM_LOW` | Fold | `FX.SUBHARM_LOW` |
| 0x14, 0x15 | `SET/GET_SUBHARM_HIGH` | Fold | `FX.SUBHARM_HIGH` |
| 0x16, 0x17 | `SET/GET_SUBHARM_BOOST` | Fold | `FX.SUBHARM_BOOST` |
| 0x18, 0x19 | `SET/GET_SUBHARM_MASK` | Fold | `FX.SUBHARM_MASK` |
| 0x1A | `GET_SUBHARM_HEADROOM` | Same | `FX.SUBHARM_HEADROOM` |
| 0x1B, 0x1C | `SET/GET_SUBHARM_TOP` | Fold | `FX.SUBHARM_TOP` |
| 0x1D, 0x1E | `SET/GET_SUBHARM_SELECT` | Fold | `FX.SUBHARM_SELECT` |
| 0x1F | `GET_SUBHARM_METER` | Same | `FX.SUBHARM_METER` |
| 0x20, 0x21 | `SET/GET_CS_GROUP` | Fold | `CS.GROUP` |
| 0x22 | `SET_CS_MACRO` | Same | `CS.MACRO_HEADER` SET (GET is new) |
| 0x23 | `GET_CS_MACRO` | Same | `CS.MACRO` |
| 0x24 | `SET_CS_MACRO_STEP` | Same | `CS.MACRO_STEP` SET (GET is new) |
| 0x25 | `CS_MACRO_FIRE` | Act, Split | `wValue` = macro: `CS.MACRO_FIRE`. 0xFFFF: `CS.MACRO_CANCEL` |
| 0x26 | `GET_CS_EXT_STATUS` | Same | `CS.EXT_STATUS` |
| 0x27, 0x28 | `SET/GET_CS_DISPLAY_CFG` | Fold, Split | `CS.DISPLAY_CONFIG` (12 B both ways). The 4 limit bytes of the old GET: `CS.DISPLAY_CAPS` |
| 0x29, 0x2A | `SET/GET_CS_DISPLAY_PAGE` | Fold | `CS.DISPLAY_PAGE` |
| 0x2B | `GET_CS_DISPLAY_STATUS` | Same | `CS.DISPLAY_STATUS` |
| 0x2C, 0x2D | `SET/GET_SUBHARM_SOLO` | Fold | `FX.SUBHARM_SOLO` |
| 0x2E, 0x2F | `SET/GET_SUBHARM_LINK` | Fold | `FX.SUBHARM_LINK` |
| 0x30, 0x31 | `SET/GET_PSYBASS` | Fold | `FX.PSYBASS_ENABLE` |
| 0x32, 0x33 | `SET/GET_PSYBASS_CUTOFF` | Fold | `FX.PSYBASS_CUTOFF` |
| 0x34, 0x35 | `SET/GET_PSYBASS_HARMONICS` | Fold | `FX.PSYBASS_HARMONICS` |
| 0x36, 0x37 | `SET/GET_PSYBASS_DRIVE` | Fold | `FX.PSYBASS_DRIVE` |
| 0x38, 0x39 | `SET/GET_PSYBASS_CHARACTER` | Fold | `FX.PSYBASS_CHARACTER` |
| 0x3A, 0x3B | `SET/GET_PSYBASS_ORIGINAL` | Fold | `FX.PSYBASS_ORIGINAL` |
| 0x3C, 0x3D | `SET/GET_PSYBASS_MASK` | Fold | `FX.PSYBASS_MASK` |
| 0x3E, 0x3F | `SET/GET_TUBE_PARAM` | Fold, Split | `wValue` index 0-13: `FX.TUBE_*` 0x60-0x6D |
| 0x42 | `SET_EQ_PARAM` | Same | `FILTER.BAND` SET. Channel and band move to `wValue`, `qp` moves inside the 16 bytes |
| 0x43 | `GET_EQ_PARAM` | Split | `FILTER.BAND` GET returns the whole band. The 3-bit param selector goes away |
| 0x44, 0x45 | `SET/GET_PREAMP` | Retire | Use `ROUTE.PREAMP` per channel |
| 0x46, 0x47 | `SET/GET_BYPASS` | Fold | `FILTER.EQ_BYPASS` |
| 0x48, 0x49 | `SET/GET_DELAY` | Fold | `ROUTE.CHANNEL_DELAY` |
| 0x4A, 0x4B | `UPMIX_SET/GET_CONFIG` | Fold | `FX.UPMIX_CONFIG` |
| 0x4C, 0x4D | `UPMIX_SET/GET_PARAM` | Fold, Split | `wValue` index 0-13: `FX.UPMIX_*` 0x90-0x9D |
| 0x4E | `UPMIX_GET_STATUS` | Same | `FX.UPMIX_STATUS` |
| 0x50 | `GET_STATUS` | Split | See 6.1 |
| 0x51 | `SAVE_PARAMS` | Retire | Use `STORE.PRESET_ACTIVE` then `STORE.PRESET_SAVE` |
| 0x52 | `SAVE_OUTPUT_CONFIG` | Act | `STORE.OUTPUT_CONFIG_SAVE` |
| 0x53 | `FACTORY_RESET` | Act | `STORE.FACTORY_RESET` |
| 0x54, 0x55 | `SET/GET_INPUT_SOURCE_NAME` | Fold | `INPUT.SOURCE_NAME` |
| 0x56, 0x57 | `SET/GET_DEVICE_NAME` | Fold | `SYS.DEVICE_NAME` |
| 0x58, 0x59 | `SET/GET_LOUDNESS` | Fold | `DYN.LOUDNESS_ENABLE` |
| 0x5A, 0x5B | `SET/GET_LOUDNESS_REF` | Fold | `DYN.LOUDNESS_REF` |
| 0x5C, 0x5D | `SET/GET_LOUDNESS_INTENSITY` | Fold | `DYN.LOUDNESS_INTENSITY` |
| 0x5E, 0x5F | `SET/GET_CROSSFEED` | Fold | `FX.CROSSFEED_ENABLE` |
| 0x60, 0x61 | `SET/GET_CROSSFEED_PRESET` | Fold | `FX.CROSSFEED_PRESET` |
| 0x62, 0x63 | `SET/GET_CROSSFEED_FREQ` | Fold | `FX.CROSSFEED_FREQ` |
| 0x64, 0x65 | `SET/GET_CROSSFEED_FEED` | Fold | `FX.CROSSFEED_FEED` |
| 0x66, 0x67 | `SET/GET_CROSSFEED_ITD` | Fold | `FX.CROSSFEED_ITD` |
| 0x68, 0x69 | `SET/GET_ADAT_INPUT_ENABLE` | Act, Fold | `INPUT.ADAT_RX_ENABLE` |
| 0x6A, 0x6B | `SET/GET_ADAT_INPUT_PIN` | Act, Fold | `INPUT.ADAT_RX_PIN` |
| 0x6C, 0x6D | `SET/GET_ADAT_INPUT_CLOCK_MODE` | Act, Fold | `INPUT.ADAT_RX_CLOCK_MODE` |
| 0x6E | `GET_ADAT_INPUT_STATUS` | Same | `INPUT.ADAT_RX_STATUS` |
| 0x70, 0x71 | `SET/GET_MATRIX_ROUTE` | Fold | `ROUTE.MATRIX_ROUTE`. Input and output move to `wValue` on SET |
| 0x72, 0x73 | `SET/GET_OUTPUT_ENABLE` | Fold | `ROUTE.OUTPUT_ENABLE` |
| 0x74, 0x75 | `SET/GET_OUTPUT_GAIN` | Fold | `ROUTE.OUTPUT_GAIN` |
| 0x76, 0x77 | `SET/GET_OUTPUT_MUTE` | Fold | `ROUTE.OUTPUT_MUTE` |
| 0x78, 0x79 | `SET/GET_OUTPUT_DELAY` | Fold | `ROUTE.OUTPUT_DELAY` |
| 0x7A | `GET_CORE1_MODE` | Same | `SYS.CORE1_MODE` |
| 0x7B | `GET_CORE1_CONFLICT` | Same | `ROUTE.OUTPUT_ENABLE_CONFLICT` |
| 0x7C, 0x7D | `SET/GET_OUTPUT_PIN` | Act, Fold | `OUTPUT.PIN`. GPIO moves to the payload |
| 0x7E | `GET_SERIAL` | Same | `SYS.SERIAL` |
| 0x7F | `GET_PLATFORM` | Same | `SYS.PLATFORM` |
| 0x80 | `GET_BUILD_INFO` | Same | `SYS.BUILD_INFO` |
| 0x81 | `LIMITER` | Split | Index 0-3: `DYN.LIMITER_ENABLE`, `_THRESHOLD`, `_RELEASE`, `_LINK_GROUP`. 0x80: `DYN.LIMITER_METERS`. 0x81: `DYN.LIMITER_STATUS` |
| 0x82 | `GET_CLOCK_DIAG` | Same | `DIAG.CLOCK_DIAG` |
| 0x83 | `CLEAR_CLIPS` | Split | Read: `MEAS.CLIP_FLAGS`. Clear: `MEAS.CLIP_CLEAR` |
| 0x84, 0x85 | `SET/GET_CS_BINDING` | Fold | `CS.BINDING` |
| 0x86 | `GET_CS_CAPS` | Split | 0xFFFF: `CS.CAPS`. Noun: `CS.NOUN_DESC` |
| 0x87 | `GET_CS_STATUS` | Same | `CS.STATUS` |
| 0x88, 0x89 | `SET/GET_I2S_CLOCK_MODE` | Fold | `CLOCK.I2S_CLOCK_MODE` |
| 0x8A | `GET_I2S_SLAVE_STATUS` | Same | `CLOCK.I2S_SLAVE_STATUS` |
| 0x8B, 0x8C | `SET/GET_CS_NAME` | Fold | `CS.SLOT_NAME` |
| 0x8D, 0x8E | `SET/GET_CS_IR_CMD` | Fold | `CS.IR_CMD` |
| 0x8F | `CS_IR_LEARN` | Act, Split | 1: `CS.IR_LEARN_ARM`. 0: `CS.IR_LEARN_CANCEL`. 2: `CS.IR_LEARN_STATUS` |
| 0x90 | `PRESET_SAVE` | Act | `STORE.PRESET_SAVE` |
| 0x91 | `PRESET_LOAD` | Act | `STORE.PRESET_LOAD` |
| 0x92 | `PRESET_DELETE` | Act | `STORE.PRESET_DELETE` |
| 0x93, 0x94 | `PRESET_GET/SET_NAME` | Fold | `STORE.PRESET_NAME` |
| 0x95 | `PRESET_GET_DIR` | Same | `STORE.DIRECTORY` |
| 0x96, 0x97 | `PRESET_SET/GET_STARTUP` | Fold | `STORE.STARTUP`. GET drops the last-active byte, which `DIRECTORY` already has |
| 0x98, 0x99 | `SET/GET_OUTPUT_CONFIG_MODE` | Fold | `STORE.OUTPUT_CONFIG_MODE` |
| 0x9A | `PRESET_GET_ACTIVE` | Same | `STORE.PRESET_ACTIVE` |
| 0x9B, 0x9C | `SET/GET_CHANNEL_NAME` | Fold | `ROUTE.CHANNEL_NAME` |
| 0x9D | `CS_SAVE` | Act | `CS.SAVE` |
| 0x9E | `CS_REVERT` | Act | `CS.REVERT` |
| 0xA0, 0xA1 | `GET/SET_ALL_PARAMS` | Fold | `STORE.ALL_PARAMS` |
| 0xA2, 0xA3 | `GET/SET_ALL_PARAMS_CHUNK` | Fold | `STORE.ALL_PARAMS_CHUNK` |
| 0xA4, 0xA5 | `SIGGEN_SET/GET_CONFIG` | Fold | `MEAS.SIGGEN_CONFIG` |
| 0xA6 | `SIGGEN_CONTROL` | Act, Split | 0: `MEAS.SIGGEN_STOP`. 1: `MEAS.SIGGEN_START`. 2: `MEAS.SIGGEN_STOP_NOW` |
| 0xA7 | `SIGGEN_GET_STATUS` | Same | `MEAS.SIGGEN_STATUS` |
| 0xA8 | `SIGGEN_GET_CAPS` | Split | 0xFFFF: `MEAS.SIGGEN_CAPS`. Type index: `MEAS.SIGGEN_TYPE_DESC` |
| 0xA9, 0xAA | `SET/GET_SUBHARM_DEPTH` | Fold | `FX.SUBHARM_DEPTH` |
| 0xAB, 0xAC | `SET/GET_SUBHARM_HOLD` | Fold | `FX.SUBHARM_HOLD` |
| 0xAD, 0xAE | `SET/GET_SUBHARM_CEILING` | Fold | `FX.SUBHARM_CEILING` |
| 0xB0 | `GET_BUFFER_STATS` | Same | `SYS.BUFFER_STATS` |
| 0xB1 | `RESET_BUFFER_STATS` | Act | `SYS.STATS_RESET`. The mask moves to the payload |
| 0xB2 | `GET_USB_ERROR_STATS` | Retire | Always zero under TinyUSB |
| 0xB3 | `RESET_USB_ERROR_STATS` | Retire | No-op |
| 0xB4, 0xB5 | `SET/GET_LEVELLER_ENABLE` | Fold | `DYN.LEVELLER_ENABLE` |
| 0xB6, 0xB7 | `SET/GET_LEVELLER_AMOUNT` | Fold | `DYN.LEVELLER_AMOUNT` |
| 0xB8, 0xB9 | `SET/GET_LEVELLER_SPEED` | Fold | `DYN.LEVELLER_SPEED` |
| 0xBA, 0xBB | `SET/GET_LEVELLER_MAX_GAIN` | Fold | `DYN.LEVELLER_MAX_GAIN` |
| 0xBC, 0xBD | `SET/GET_LEVELLER_LOOKAHEAD` | Fold | `DYN.LEVELLER_LOOKAHEAD` |
| 0xBE, 0xBF | `SET/GET_LEVELLER_GATE` | Fold | `DYN.LEVELLER_GATE` |
| 0xC0, 0xC1 | `SET/GET_OUTPUT_TYPE` | Act, Fold | `OUTPUT.TYPE`. Type moves to the payload |
| 0xC2, 0xC3 | `SET/GET_I2S_BCK_PIN` | Act, Fold | `CLOCK.I2S_BCK_PIN`. Role stays in `wValue`, GPIO moves to the payload |
| 0xC4, 0xC5 | `SET/GET_MCK_ENABLE` | Act, Fold | `CLOCK.MCK_ENABLE` |
| 0xC6, 0xC7 | `SET/GET_MCK_PIN` | Act, Fold | `CLOCK.MCK_PIN` |
| 0xC8, 0xC9 | `SET/GET_MCK_MULTIPLIER` | Act, Fold | `CLOCK.MCK_MULTIPLIER` |
| 0xCA, 0xCB | `SET/GET_ADAT_ENABLE` | Act, Fold | `OUTPUT.ADAT_TX_ENABLE` |
| 0xCC, 0xCD | `SET/GET_ADAT_PIN` | Act, Fold | `OUTPUT.ADAT_TX_PIN` |
| 0xCE | `GET_ADAT_STATUS` | Same | `OUTPUT.ADAT_TX_STATUS` |
| 0xD0, 0xD1 | `SET/GET_PREAMP_CH` | Fold | `ROUTE.PREAMP` |
| 0xD2, 0xD3 | `SET/GET_MASTER_VOLUME` | Fold | `ROUTE.MASTER_VOLUME` |
| 0xD4, 0xD5 | `SET/GET_MASTER_VOLUME_MODE` | Fold | `ROUTE.MASTER_VOLUME_MODE` |
| 0xD6 | `SAVE_MASTER_VOLUME` | Act | `ROUTE.MASTER_VOLUME_SAVE` |
| 0xD7 | `GET_SAVED_MASTER_VOLUME` | Same | `ROUTE.MASTER_VOLUME_SAVED` |
| 0xD8, 0xD9 | `SET/GET_BAND_BYPASS` | Fold | `FILTER.BAND_BYPASS` |
| 0xDA, 0xDB | `SET/GET_USER_VOLUME` | Fold | `ROUTE.USER_VOLUME` |
| 0xDC, 0xDD | `SET/GET_USER_MUTE` | Fold | `ROUTE.USER_MUTE` |
| 0xDE, 0xDF | `SET/GET_LEVELLER_MASKS` | Fold | `DYN.LEVELLER_MASKS` |
| 0xE0, 0xE1 | `SET/GET_INPUT_SOURCE` | Fold | `INPUT.SOURCE` |
| 0xE2 | `GET_SPDIF_RX_STATUS` | Same | `INPUT.SPDIF_RX_STATUS` |
| 0xE3 | `GET_SPDIF_RX_CH_STATUS` | Same | `INPUT.SPDIF_RX_CH_STATUS` |
| 0xE4, 0xE5 | `SET/GET_SPDIF_RX_PIN` | Act, Fold | `INPUT.SPDIF_RX_PIN`. Index stays in `wValue`, GPIO moves to the payload |
| 0xE6, 0xE7 | `SET/GET_LG_SOUND_SYNC_ENABLE` | Fold | `INPUT.LG_SOUND_SYNC_ENABLE` |
| 0xE8 | `GET_LG_SOUND_SYNC_STATUS` | Same | `INPUT.LG_SOUND_SYNC_STATUS` |
| 0xE9 | `SET_SPDIF_INPUT_ENABLE` | Act | `INPUT.SPDIF_RX_ENABLE` (GET is new) |
| 0xEA, 0xEB | `SET/GET_DAC_HW_MUTE_CONFIG` | Fold | `OUTPUT.DAC_MUTE_CONFIG` |
| 0xEC | `TEST_DAC_HW_MUTE` | Act | `OUTPUT.DAC_MUTE_TEST` |
| 0xED, 0xEE | `SET/GET_INPUT_RATE` | Fold, Split | `INPUT.I2S_RX_RATE` (selected rate, 4 B both ways). The live rate half of the old GET: `SYS.SYSTEM_STATUS` |
| 0xEF | `GET_SPDIF_INPUT_CONFIG` | Same | `INPUT.SPDIF_RX_CONFIG` |
| 0xF0 | `ENTER_BOOTLOADER` | Act | `SYS.ENTER_BOOTLOADER` |
| 0xF1, 0xF2 | `SET/GET_I2S_RX_PIN` | Act, Fold | `INPUT.I2S_RX_PIN` |
| 0xF3, 0xF4 | `SET/GET_I2S_INPUT_CHANNELS` | Act, Fold | `INPUT.I2S_RX_CHANNELS` |
| 0xF5, 0xF6 | `SET/GET_UART_CONFIG` | Fold | `XPORT.UART_CONFIG` |
| 0xF7, 0xF8 | `SET/GET_I2C_CONFIG` | Fold | `XPORT.I2C_CONFIG` |
| 0xF9 | `GET_CTRL_IFACE_STATUS` | Same | `XPORT.STATUS` |
| 0xFA, 0xFB | `SET/GET_LOUDNESS_MASK` | Fold | `DYN.LOUDNESS_MASK` |
| 0xFC, 0xFD | `SET/GET_CROSSFEED_OUTPUTS` | Fold | `FX.CROSSFEED_OUTPUTS` |
| 0xFE, 0xFF | `SET/GET_I2S_CLOCK_PIN_MODE` | Act, Fold | `CLOCK.I2S_CLOCK_PIN_MODE` |

The 246 opcodes become about 215 commands in 13 pages, even after every multiplexed command is split. The largest page uses about 60 of its 240 numbers.

### 6.1 `GET_STATUS` (0x50) split

| `wValue` | Field | New home |
|---|---|---|
| 0, 1, 2 | Packed peak pairs and CPU | Retire. `MEAS.LEVELS` and `SYS.SYSTEM_STATUS` |
| 3-6 | PDM ring and DMA counters | `SYS.PDM_STATS` |
| 7 | Retired | Retire |
| 8 | USB audio packet gaps of 2 to 50 ms (`spdif_underruns`) | `SYS.SPDIF_TX_STATS` |
| 9 | Combined meter packet | `MEAS.LEVELS` |
| 10, 11, 12 | USB packets, alt, mounted | `SYS.USB_AUDIO_STATUS` |
| 13, 14, 15, 16 | clk_sys, core voltage, sample rate, temperature | `SYS.SYSTEM_STATUS` |
| 17-21 | S/PDIF DMA starvations | `SYS.SPDIF_TX_STATS` |
| 22 | USB ring overruns | `SYS.USB_AUDIO_STATUS` |
| 23 | Active input channels | `INPUT.STATUS` |
| 24, 25 | Loopback counters | `DIAG.LOOPBACK_STATS` |
| 26, 27, 28 | Output ring underruns, drops, min prewrite | `SYS.PIPELINE_STATS` |
| 29 | Worst input delivery gap | `SYS.PIPELINE_STATS` |
| 30 | Worst USB packet gap | `SYS.USB_AUDIO_STATUS` |
| 31 | Background over budget | `SYS.PIPELINE_STATS` |
| 32, 33 | S/PDIF measured rate, VCXO trim | `INPUT.STATUS` |
| 64 and up | Loop profiler (bench builds) | `DIAG.LOOP_PROF_*` |

A stats screen that makes 25 scalar reads today makes 6 struct reads afterwards.

## 7. Compatibility and migration

### 7.1 Decision: clean break in v1.1.7-beta1

The paged protocol ships in firmware v1.1.7-beta1 (`FW_VERSION_PATCH` 7, `FW_VERSION_BETA` 1). The old opcodes stop working in that release. There is no legacy page, no translation shim in the firmware and no later removal step. Contributors have already been told that 1.1.7 Beta 1 is the release that overhauls the opcodes.

### 7.2 Scope of this work

This work delivers three things:

1. The firmware.
2. The `tools/dspi_test` harness and the other tools inside this repo, which would otherwise stop working.
3. A comprehensive migration guide for host maintainers (section 10).

It does not change any host app. DSPi Console (macOS, iOS, Linux, Windows), DSPi Link, DSPi Terminal and the ESP32 front panel are ported by their maintainers from the migration guide. Their repos are read, never edited, so the guide can be concrete about each one.

### 7.3 Page 0 is empty

Page 0 answers nothing. A GET or a SET with a zero `wIndex` high byte STALLs on USB, and UART and I2C return `VC_ERR_UNKNOWN`. Nothing on page 0 is silently ACKed any more. The Microsoft OS 2.0 descriptor request is not a DSPi command and is still served, because Windows needs it to bind WinUSB.

That matters for safety. A 1.1.6 host sends `wIndex` 0 or 2, so all of its requests land on page 0. It cannot change anything on a 1.1.7 device by accident.

The user decided against keeping `GET_PLATFORM` and `ENTER_BOOTLOADER` alive on page 0. The consequences are accepted:

- A 1.1.6 Console that meets 1.1.7 firmware cannot read its version. It reports a communication fault, and the user updates the app by hand.
- A 1.1.6 Console cannot put 1.1.7 firmware into the bootloader. Going back to 1.1.6 uses the BOOTSEL button.

### 7.4 Hosts meeting the wrong firmware

| Host | Firmware | What happens |
|---|---|---|
| 1.1.6 host, any transport | 1.1.7-beta1 | Every request is refused. No state changes |
| 1.1.7 host | 1.1.6 or older | The `PROTOCOL_INFO` probe STALLs. The host may read the old firmware's own `GET_PLATFORM` (0x7F) and offer the update, and may send its own `ENTER_BOOTLOADER` (0xF0) to start it. It sends no paged command |

The second row is the dangerous direction. Old firmware ignores `wIndex`, so an unprobed paged command would run as whatever old opcode shares its number. Section 5.14 makes the probe mandatory for that reason, and the migration guide repeats it for every transport.

### 7.5 Proving the refactor without a compatibility path

A clean break leaves no old path in the shipped firmware to compare against. Three things provided the regression proof, and none of them ships.

1. **A golden capture of 1.1.6.** Before the code changed, every GET reply of firmware 1.1.6 was recorded twice, before and after a scripted sequence that writes a non-default value to every settable command it can safely touch. A mask list excludes fields that legitimately vary (meters, counters, temperature, serial, build info, version).
2. **A temporary translation layer in the test harness.** `tools/dspi_test/legacy_map.py` turned each old call (`OP.X`, direction, `wValue`, payload) into the new paged request, using a data file that is the machine-readable form of section 6. It needed only a few kinds of adapter. They were direct, value from `wValue`, packed address and value, and a small custom function for each split command. The existing 173 tests then ran against the new handlers without being rewritten. Replaying the golden script through this layer and comparing bytes was the regression proof. On 1.1.7 beta 1 it matched the 1.1.6 capture with 0 unexplained differences.
3. **Old and new side by side during development.** Until the flip phase, page 0 on the working branch still reached the old switches. Every migrated command was compared with its original on the same device, in the same state. The flip deleted the old code before anything was released.

The translation layer was scaffolding. Phase 6 ported the tests to native paged calls and deleted it (8.13). The native golden reference was captured in the same session as the matching translated replay, which ties it to 1.1.6. The mapping data file stays, as `Documentation/protocol/legacy_map.json`, because it is also the machine-readable migration table for host maintainers. `scripts/gen_vendor_protocol.py --check` keeps it consistent with the registry.

Tests that asserted an old quirk were the exception to "unchanged". A rejected write is now an error, not a silent drop, and an unknown status selector no longer returns zeros. Those tests were updated to expect the new behaviour and listed in the phase reports.

### 7.6 Release gate for v1.1.7-beta1

The firmware is ready to release when all of these hold:

1. The Phase 0 `wIndex` test has passed on Windows.
2. The golden replay matches, with every difference explained.
3. The native paged harness passes on hardware, including the loopback alignment tests.
4. `FW_VERSION_MAJOR/MINOR/PATCH/BETA` read 1, 1, 7, 1.
5. The migration guide is complete and checked against each host repo.

Cutting the release itself stays the user's call, through the normal release process.

## 8. Dispatcher and table design

### 8.1 File layout

```
firmware/DSPi/vendor/
  vc_core.h / vc_core.c       table types, lookup, validation, dispatch, results
  vc_usb.c                    tud_vendor_control_xfer_cb, EP0 stages, MS OS 2.0 intercept
  vc_bulk.c                   bulk buffer lock, ALL_PARAMS, chunk sessions
  vc_pins.c / vc_pins.h       pin validation helpers (moved from vendor_commands.c)
  pages/sys.def      + vc_page_sys.c
  pages/store.def    + vc_page_store.c
  ... one .def and one .c per page
```

`vendor_commands.h` stays as the public header, so `uart_control.c`, `i2c_control.c` and the Control Surfaces code keep their include. `vendor_commands.c` keeps the old switches on the working branch until the flip phase and is then deleted, along with every `REQ_*` define.

### 8.2 The table entry

```c
typedef struct {
    CtrlSource     src;       // transport of this request
    uint8_t        arg;       // per-entry constant from the table
    uint16_t       addr;      // wValue, already range-checked
    const uint8_t *in;        // SET payload
    uint16_t       in_len;
    uint8_t       *out;       // GET reply buffer
    uint16_t       out_len;   // handler sets for variable-length replies
} VcCtx;

typedef VcResult (*VcHandler)(VcCtx *c);

typedef struct {
    uint8_t   cmd;            // command number within the page
    uint8_t   addr_kind;      // VC_A_NONE, VC_A_OUTPUT, VC_A_CH_BAND, ...
    uint8_t   set_min;        // SET payload bounds (non-bulk, <= 64)
    uint8_t   set_max;
    uint16_t  get_len;        // fixed reply length, or VC_LEN_VAR
    uint16_t  flags;          // VC_F_*
    uint8_t   arg;            // lets one handler serve a family
    VcHandler get;            // NULL = no GET
    VcHandler set;            // NULL = no SET
} VcCmd;                      // 20 bytes, const, in flash

typedef struct {
    uint8_t      page;
    uint8_t      revision;
    uint8_t      count;
    const VcCmd *cmds;
} VcPage;
```

Flags the dispatcher enforces:

| Flag | Meaning |
|---|---|
| `VC_F_USB_ONLY` | Both directions refused on UART, I2C and internal dispatch |
| `VC_F_SET_USB_ONLY` | SET refused off USB (transport self-configuration) |
| `VC_F_BULK` | Payload or reply lives in `bulk_param_buf` under the bulk lock |

Flags that only inform the docs and `COMMAND_INFO`:

| Flag | Meaning |
|---|---|
| `VC_F_DEFERRED` | Applied by the main loop after the reply |
| `VC_F_FLASH` | Writes flash, so a short busy window follows |
| `VC_F_RESTART` | Restarts or mutes the outputs |
| `VC_F_KEEPALIVE` | The GET refreshes a timer |

There is no write-as-read flag, and no write-as-read command is left in the firmware.

### 8.3 Registration

Each page is an X-macro file. It is the single source for the C table, the command constants, the handler prototypes, the docs and the host registry.

```c
// pages/filter.def
VC_PAGE(FILTER, 0x06, 1 /* revision */, "EQ and crossover bands")

VC_CMD(0x10, BAND,        filter_band_get,   filter_band_set,
       ADDR(CH_BAND), GET(16), SET(16, 16), VC_F_DEFERRED, ARG(0),
       "One EQ or crossover band")
VC_CMD(0x11, BAND_BYPASS, filter_bypass_get, filter_bypass_set,
       ADDR(CH_BAND), GET(1),  SET(1, 1),   VC_F_DEFERRED, ARG(0),
       "Bypass one band")
VC_CMD(0x12, EQ_BYPASS,   eq_bypass_get,     eq_bypass_set,
       ADDR(NONE),    GET(1),  SET(1, 1),   0,             ARG(0),
       "Master EQ bypass")
// VC_RESERVED(0x20, "future: per-channel EQ copy")
```

`vc_core.c` includes every `.def` three times with different macro definitions. The first pass emits `enum { VC_FILTER_BAND = 0x0610, ... }`. The second emits handler prototypes. The third emits the `VcCmd` arrays and the page directory. A fourth pass in a dummy `switch` turns every command number into a `case` label, so a duplicate number fails the build.

This avoids linker-section registration, which is fragile with the custom XIP linker scripts. It also needs no code generation step in the firmware build.

### 8.4 Validation and dispatch

One function does every check, for every transport, before a handler runs.

```c
VcResult vc_dispatch(CtrlSource src, bool is_get, uint8_t page, uint8_t cmd,
                     uint16_t addr, const uint8_t *in, uint16_t in_len,
                     const uint8_t **out, uint16_t *out_len);
```

Order of checks:

1. Find the page, then the command. Otherwise `VC_ERR_UNKNOWN`.
2. Direction has a handler. Otherwise `VC_ERR_DIRECTION`.
3. (Platform and build conditions are resolved at compile time. A command that is not built is simply unknown.)
4. Transport flags. Otherwise `VC_ERR_BLOCKED`.
5. For SET, `set_min <= in_len <= set_max`. Otherwise `VC_ERR_LENGTH`.
6. Address in range for `addr_kind`, using the platform's channel, output, slot and band counts. Otherwise `VC_ERR_ADDRESS`.
7. Set the notify source tag, call the handler, clear the tag.
8. For a fixed-length GET, assert the handler filled `get_len` bytes.
9. Record `{id, direction, result}` in the per-transport `LAST_RESULT` slot.

Handlers therefore contain only value checks and the actual work. The repeated bounds checks and length checks in today's 246 cases go away.

Lookup is a linear scan of one page, at most about 60 entries, on the control path. No hot code is involved.

Replies go into one static 256-byte buffer owned by `vc_core.c`. It replaces three existing buffers (`resp_buf`, `rta_resp_buf`, `_ext_resp_copy`, 278 bytes together), so BSS does not grow. Anything larger is `VC_F_BULK`.

### 8.5 USB front end

`vc_usb.c` keeps the stage logic and loses every opcode comparison.

- **SETUP.** The MS OS 2.0 intercept runs first, unchanged. Then `page = wIndex >> 8`. Page 0 is refused. Otherwise the entry is looked up and checks 1-6 run at once, so a bad length or address STALLs before any data stage.
- **GET.** The handler runs and the reply is sent, clamped to `wLength`.
- **Zero-length SET.** The handler runs at SETUP. This is what makes real actions possible.
- **SET with data.** The payload is received into the 64-byte buffer, or into `bulk_param_buf` for `VC_F_BULK`. The handler runs at the DATA stage and a failure returns false, which STALLs the status stage.
- **Bulk and chunk sessions.** The existing lock and session logic moves to `vc_bulk.c` and keys on the `VC_F_BULK` flag and the two `STORE` IDs.

### 8.6 External transports and internal callers

`vendor_dispatch_get()` and `vendor_dispatch_set()` keep their signatures. They already receive `wIndex`, so they take the page from its high byte and call `vc_dispatch()`. Page 0 is refused with `VC_ERR_UNKNOWN`.

`vendor_is_bulk_set()` gains the page. It stays a small standalone function with the two IDs written in, because the I2C ISR calls it and it must not walk the table.

Internal callers get a thin typed wrapper:

```c
VcResult vc_set(CtrlSource src, uint16_t id, uint16_t addr,
                const void *payload, uint16_t len);
```

### 8.7 Invariants the refactor must hold

- **Handlers are moved, not re-sequenced.** Every deferred operation keeps its pending flag and its main-loop path. No handler may apply a change that today goes through `prepare_pipeline_reset()` and `complete_pipeline_reset()` by any shorter route. Output slot alignment depends on it.
- **The notify source bracket is preserved.** `notify_set_source()` wraps every handler call and is cleared afterwards.
- **`usb_device.c` in pico-extras is not touched.**
- **No new notification events.** If one is ever added, it needs both the push and the `notify_peek_next_for` case.
- **The USB-in-flight guard and the bulk lock keep their semantics.**

### 8.8 As built (Phase 1)

The grammar below is what the `.def` files actually use. It replaces the wrapped `ADDR(...)`, `GET(...)`, `SET(...)` form sketched in 8.3.

```
VC_PAGE(MNEM, 0xPP, REV, "one line description")
VC_CMD(0xCC, NAME, get_fn, set_fn, ADDR_KIND, GET_LEN, SET_MIN, SET_MAX, FLAGS, ARG, "one line doc")
VC_RESERVED(0xCC, "note")
```

- `get_fn` or `set_fn` is `VC_NONE` when the command has no handler for that direction. `GET_LEN` is a byte count, `VC_LEN_VAR`, or 0.
- Address kinds are bare tokens such as `NONE`, `OUTPUT`, `OUTPUT_ALL`, `CH_BAND`, `IN_OUT`, `CS_SLOT`, `OFFSET`. `OUTPUT_ALL` accepts 0xFF on a SET, which replaces the `VC_F_ADDR_ALL` flag.
- Platform and build limits are whole-line `#if PICO_RP2350` or `#ifdef DSPI_X` blocks around entries, with no `#else` and no nesting. A command outside its build does not exist and answers `VC_ERR_UNKNOWN`. This replaces the `VC_F_RP2350` flag.
- Each page's table lives in its own `vc_page_<mnem>.c`, which includes `vc_page_table.inc` last. Implementing a page means editing that file and its `.def` and nothing else.
- `vc_ids.h` holds the `VC_<MNEM>_<NAME>` constants and can be included anywhere.
- `vc_page_of(bmRequestType, wIndex)` is the only place the page is derived from a request. The harness has the matching single helper, `paged_windex(page)`.
- A duplicate number, a number below 0x10, a duplicate name and inconsistent lengths all fail the build.
- `SYS.ENTER_BOOTLOADER` reboots after the reply has gone out: after the status stage on USB, and after the response frame on UART and I2C.

### 8.9 As built (Phase 2)

All 13 pages are implemented: 206 commands on RP2350 release builds, with the old switches still serving page 0 beside them.

- `vc_core.h` carries the helpers more than one page needs: `vc_param_src()`, `vc_from_pin_status()` and `vc_clampf()`.
- `STORE.ALL_PARAMS` and `STORE.ALL_PARAMS_CHUNK` run in the bulk buffer. The chunk-session state moved to `vc_bulk.c` and is shared with the old 0xA2 and 0xA3 paths, so a session opened on one path is closed by a request on the other.
- A bulk SET checks its header before it queues the apply, so a snapshot the main loop would refuse now fails at the status stage.
- `FILTER.BAND` and `FILTER.BAND_BYPASS` return `VC_ERR_BUSY` while an earlier band write is still waiting for the main loop. The old opcodes overwrote the single pending slot.
- `MEAS.RTA_BINS` returns at most 256 bytes per request.
- `DIAG.CLOCK_DIAG` is guarded by `#if DSPI_CLOCK_DIAG`, because `config.h` defines the symbol as 0 when the option is off.
- Every place the new behaviour differs from 1.1.6 was recorded in a working notes file. Phase 7 absorbed it into the migration guide (`vendor_protocol_migration_guide.md`) and removed it.
- The harness had a third mode, hybrid, which sent an old call through the paged commands only when the device had every command it needed. It was used to run the existing tests and the golden replay through each page as it landed. Phase 6 removed it (8.13).

### 8.10 As built (Phase 3)

- **Transport API.** `vendor/vc_ctrl.h` holds what UART, I2C and internal callers need: `CtrlSource`, `VcResult`, the `CTRL_STATUS_*` codes and the entry points. The transports call `ctrl_dispatch_get()` and `ctrl_dispatch_set()`, which return the wire status byte. The bulk lock and `vendor_is_bulk_set()` live in `vc_bulk.c`.
- **Status byte on UART and I2C.** 0x00 is OK. 0x01 to 0x07 are transport conditions where the command did not run (busy, bulk lock held, CRC, oversize, bad frame). 0x10 and up is the `VcResult` itself. Any non-zero value is a failure. 0x01, 0x04 and 0x17 are worth a retry.
- **Reply size.** Both transports carry replies up to the 256-byte reply buffer.
- **Bulk.** `STORE.ALL_PARAMS` works over UART and I2C with the same zero-copy streaming and lock rules the old opcodes had. `ALL_PARAMS_CHUNK`, `RTA_BANDS_ALL` and the `XPORT` config SETs stay USB only. `CLOCK_DIAG` was USB only too until 2026-11-17 (14).
- **Control Surfaces.** `cs_noun_dispatch()` sends every noun through `vc_set()` with `CTRL_SOURCE_GPIO`. It returns false only on `VC_ERR_BUSY`, which the engine retries on the next tick.
- **Loop profiler.** `DIAG` 0x30 to 0x34 replace the `GET_STATUS` selectors. Reads no longer clear, and `LOOP_PROF_RESET` does. Per-command times are kept by table index and read by command ID.
- **Page 0 on the external transports** still reaches the old code through one marked call-through in `vendor_commands.c`, which the flip deletes.
- **Not exercised on hardware.** No UART or I2C host is available on the bench, so the wire framing, the I2C interrupt path and the deferred reboot on those transports were checked by build, by reading, and by a temporary on-device harness that drove the dispatch entry points as UART and I2C sources.

### 8.11 As built (Phase 4)

- **Version.** `config.h` reads 1.1.7 beta 1 (`FW_VERSION_PATCH` 7, `FW_VERSION_BETA` 1). `SYS.PLATFORM` returns the platform byte, then 1, 0x17, the output count, 1, 7, 1.
- **Deleted.** `vendor_commands.c` and `vendor_commands.h` (both switches, `vendor_send_response`, the scalar shim, the page 0 call-through), every `REQ_*` define in `config.h`, `MatrixRoutePacket`, the unused `VendorCmdPacket` and `VendorRespPacket`, `OUTPUT_LATENCY_GET_*`, `LIMITER_GET_METER` / `LIMITER_GET_STATUS`, `WIRE_BULK_PARAMS_MIN_SIZE`, `CtrlDispatchResult` with `vendor_dispatch_get/set` and its two converters, `CTRL_STATUS_ERROR` (0x02) and `CTRL_STATUS_BLOCKED` (0x03), `vc_bulk_legacy_handoff()`, and the `DSPI_WINDEX_ECHO` option with its request 0x4F. Codes 0x02 and 0x03 stay reserved on UART and I2C and are never reused.
- **USB entry.** `tud_vendor_control_xfer_cb` lives in `vendor/vc_usb.c`. At SETUP it closes any chunk session the request does not continue, drops a USB-held bulk lock, clears the in-flight guard, serves the MS OS 2.0 descriptor set (IN, `bRequest` 0x01, `wIndex` 7, any recipient, exactly as 1.1.6 did), and sends everything else to the paged front end. The EP0 receive buffer is `vc_usb_rx` in the same file.
- **Page 0 is refused, not special-cased.** There is no page 0 table, so `vc_check()` returns `VC_ERR_UNKNOWN` and the request is recorded in `SYS.LAST_RESULT` for its transport. USB STALLs at SETUP, so no data stage is ever accepted. UART and I2C return status 0x10, and `vc_set()` / `vc_get()` return the code. Other uses of `bRequest` 0x01 also fall through and are refused the same way, since no page has a command below 0x10.
- **Moved.** `vreg_voltage_to_mv()` and `read_temperature_cdeg()` are static in `vc_page_sys.c`, `mck_encode()` / `mck_decode()` static in `vc_page_clock.c`, the `usb_audio_ring_overrun_count()` prototype is in `usb_audio.h`, and the `RTA_BANDS_ALL` fits-the-bulk-buffer assert is in `vc_page_meas.c`. `vc_core.h` includes `config.h` directly.
- **Chunk sessions** are static to `vc_bulk.c` and serve `STORE.ALL_PARAMS_CHUNK` only. `vc_bulk_chunk_continues(id, is_get)` keeps a session open across SETUPs for the next request in the same direction.
- **Size.** Flash text falls by 19.8 KB (RP2040) and 19.6 KB (RP2350). BSS falls by 216 B and 220 B. `scripts/ram_baseline.json` holds the new figures.
- **Dead save path removed.** `save_params_pending` lost its last setter at the flip. The flag, its main-loop branch and `flash_save_params()` are deleted.
- **Finite-value checks.** A NaN or infinite float SET is refused with `VC_ERR_VALUE` and stores nothing in ROUTE (matrix gain, output gain, output delay, channel delay), DYN (leveller amount, max gain and gate, loudness reference and intensity), FX (crossfeed cutoff and feed, every subharm and psybass float, every upmix parameter and `UPMIX_CONFIG`) and `FILTER.BAND` (freq, Q, gain). The limiter and tube setters refuse NaN and clamp infinity. The siggen config sanitises its floats. The bulk SET stores floats raw. The handlers as copied from 1.1.6 stored a NaN output gain or channel delay.
- **Harness auto mode.** `device.py` probed `SYS.PROTOCOL_INFO` and selected `translate` on paged firmware and `old` on 1.1.6. `DSPI_TEST_PROTOCOL` forced a mode. Phase 6 removed the modes (8.13).
- **Page 0 test.** `tests/paged_page0.py` walks every `bRequest` on page 0 (both recipients, GET and both SET shapes) and checks that each one STALLs and leaves the parameters and directory unchanged, and that the MS OS 2.0 request is still served. It refuses to run unless the firmware provably has no page 0.
- **Paged tests stand alone.** The 14 `tests/paged_*.py` modules no longer send old opcodes. Their A/B halves are replaced by assertions against the spec, and new tests cover the finite-value rule. Each of those first proves the rule on one `ROUTE.OUTPUT_GAIN` write and sends no further non-finite value if it fails.
- **Windows test.** The echo build is gone. `tools/windex_page_test.py` runs against the normal firmware (11.1).

### 8.12 As built (Phase 5)

- **`commands.md` is generated per page.** Each page section holds its command table between `<!-- VC:BEGIN <MNEM> -->` and `<!-- VC:END <MNEM> -->`, the page map sits in a `PAGES` block, and the name-sorted command index at the end is an `INDEX` block. The index links each command to its page section when the `## N. MNEM (0xPP)` heading exists. `PAGES` and `INDEX` are reserved mnemonics. `scripts/gen_vendor_protocol.py --check` fails when any block, or the JSON registry, differs from the `.def` files. The prose around the blocks stays hand-written.
- **Old numbers** appear only in section 25 of `commands.md`, which points to the migration guide (`vendor_protocol_migration_guide.md`) and `legacy_map.json`. `current_architecture.md`, the feature specs and `README.md` name commands as `PAGE.NAME`.
- **Tools.** `tools/dspi_poke/` reads names, directions, address kinds and sizes from `dspi_commands.json`, found by walking up from the script or the working directory, and also accepts numeric forms without it. `tools/latency_bench/` resolves names through `tools.dspi_test.device` (`resolve_command`, `pget`), with the shared struct readers in `tools/latency_bench/paged.py` (moved to `tools/dspi_test/telemetry.py` in Phase 6). `tools/clkdiag_poll.py` and `tools/ring_health_poll.py` stay standalone pyusb scripts with page and command numbers as named constants. `dspi_poke`, `clkdiag_poll.py` and `ring_health_poll.py` probe `SYS.PROTOCOL_INFO` first and stop on older firmware. The latency bench relies on the harness device's own protocol probe.
- **Flag audit.** `RESTART` now marks every command that can interrupt audio. Every flash-writing command carries it, because each ends in `complete_flash_write_operation_full()`. So do the input pin and clock commands that restart a live input under mute (`INPUT.SPDIF_RX_PIN`, `INPUT.I2S_RX_PIN`, `INPUT.I2S_RX_CHANNELS`, `INPUT.ADAT_RX_PIN`, `CLOCK.I2S_BCK_PIN`, `CLOCK.I2S_CLOCK_PIN_MODE`), `OUTPUT.DAC_MUTE_TEST` and `SYS.ENTER_BOOTLOADER`. `STORE.FACTORY_RESET` lost `FLASH`: it resets the live state, writes no flash and keeps every stored preset, as 1.1.6 did. The protocol is unreleased, so no page revision changed.
- **Doc strings.** `SYS.SPDIF_TX_STATS` describes its first field as the USB packet-gap count it is, `SYS.PLATFORM` lists its bytes, and `STORE.ALL_PARAMS_CHUNK` says that only SET chunks must arrive in order.

### 8.13 As built (Phase 6)

- **Deleted.** `tools/dspi_test/legacy_map.py`, the `old`, `translate`, `hybrid` and `auto` protocol modes with `DSPI_TEST_PROTOCOL`, the old-style `dev.get` / `dev.set` / `OP` API, and `raw_get` / `raw_set`. From `golden/`, `script.py`, `rp2350_1.1.6.json`, `intended.json` and `legacy_opcodes.json`. All of them are in commit edf11eb.
- **Device API.** `pget` / `pset` and their typed forms, `pget_ready`, `last_result()`, `protocol_info()`, `paged_commands()`, `resolve()`, `wait_ready()` / `is_alive()` over `SYS.PLATFORM`, and one raw primitive, `raw_request()`, used only by the page 0 test. A paged request to firmware that does not answer `SYS.PROTOCOL_INFO` raises `ProtocolError`, which says the harness needs firmware 1.1.7 or later.
- **Shared support.** `helpers.py` holds the native helpers. They cover float and bool round trips, expected result codes, non-finite refusal, waits, busy retry, `FILTER.BAND` and `ROUTE.MATRIX_ROUTE` packing, and GPIO choice from the firmware's valid-pin rule. The new `telemetry.py` reads the telemetry structs for the harness and for `tools/latency_bench`, whose `paged.py` keeps only `stats_reset`. The new `tests/_cs.py` holds the Control Surfaces support.
- **Two test families, not merged.** The plan said to merge the Phase 2 paged tests into the older modules. They were kept apart, because the two do different jobs. The `paged_*` modules check per-page protocol conformance (shapes, round trips, refusals, discovery). The other 13 modules, all ported to native calls, check feature behaviour (clamps and ranges, cross-feature interactions, bulk round trips, stress and fuzz, the spectrum analyser, preset isolation, pin moves, audio loopback). Tests in the older modules that only duplicated `paged_*` coverage were removed.
- **Stale tests fixed.** These were test errors, not firmware errors. Pin tests assumed GPIO 12 was invalid, but it is valid, and one test moved a live output pin by accident. The tube drive clamp test expected -6 dB, where the documented range is -30 to 24 dB. A subharm headroom expectation now follows the spec's 4.2 dB for one band. The RTA bin frame reader used 64-byte reads, slower than a frame.
- **Native golden regression.** `capture.py` has the `capture` and `compare` modes and `--selfcheck`, with the `--allow-flash` and `--allow-reconfig` tiers. `script_rp2350.json` and `script_rp2040.json` hold native requests (format 2). The RP2040 script is derived and has no reference. `reference_rp2350.json` was captured from 1.1.7 beta 1 in the same session in which the translated replay matched the 1.1.6 capture with 0 unexplained differences. Its metadata records how to recover the 1.1.6 capture, with `git show edf11eb:tools/dspi_test/golden/rp2350_1.1.6.json`.
- **Migration data checked.** `scripts/gen_vendor_protocol.py --check` also verifies `Documentation/protocol/legacy_map.json` against the registry, so the old-to-new table cannot drift from the firmware.
- **Opt-in CS revert tests.** `tests/control_surfaces.py` (three `CS.REVERT` tests) SKIPped unless `DSPI_TEST_CS_REVERT=1` was set, because `CS.REVERT` took stored non-button bindings down. The bug was fixed on 2026-11-17 and the gate removed (14).
- **Selftest.** A section that needs a missing optional module (`numpy`, `sounddevice`) reports SKIP and the run exits 0. The result is OK 170, SKIP 1, FAILURES 0.
- **Hardware result.** The whole suite with `--allow-flash` on the bench RP2350 gave 180 of 191 passing, 1 failing and 10 SKIPs. The skips are the bootloader policy (1), no `sounddevice` (1), the CS revert opt-in (3) and the bench-build `DIAG` commands (5). The audio loopback group was not part of this run.
- **The one failure** was `rta.rta_busy_within_budget`. The analyser's worst frame time measures 1.04 to 1.08 ms at FFT order 10 on RP2350, where `spectrum_analyser_spec.md` section 3.1 says "a fraction of a millisecond". That sentence describes the capture gap, while the test measured the summed processing time of the transform's slices. On 2026-11-17 the user chose to change the test (14).
- **Not ported.** `tools/batch_bench/` drives an experimental firmware command that this firmware does not have. It still imports `OP` and fails at import, and its README says so.

### 8.14 As built (Phase 7)

- **The guide.** `Documentation/Features/vendor_protocol_migration_guide.md` is the one document a host maintainer needs. After a TL;DR it has 13 sections. They are who reads what (1), what changed (2), the request field by field with byte-level examples (3), the probe and discovery (4), errors and result codes (5), the old-to-new map for all 246 opcodes with a recipe for every split, reshaped, retired and former write-as-read opcode (6), every GET reply (7), per-page notes (8), bulk parameters (9), notifications (10), per-transport notes (11), the porting checklist and tests (12) and the host appendices (13). The map, the `GET_STATUS` table, the write-as-read and retired lists and the reply lengths are generated blocks, which `scripts/gen_vendor_protocol.py --check` keeps in step with the registry and `legacy_map.json`.
- **The short guide.** `Documentation/Features/vendor_protocol_client_update.md` (added 2026-11-17) keeps only what a client must change. The generator fills its map, `GET_STATUS` and retired tables, and their links fall back to the full guide's sections.
- **Four host appendices** in section 13, each written from a read-only audit of the host repository and naming the branch and commit it audited.
    - 13.1 ESP32 front panel (UART). Self-contained for a maintainer who reads nothing else. It has a row and an old and new frame for each of the panel's 69 opcodes, the probe with a protocol state, the new status codes, the RP2040 ADAT gate and the 1.1.6 number collisions of the panel's own requests.
    - 13.2 DSPi Console for macOS. The 16-bit transport, the probe with a legacy state that keeps the firmware update working for 1.1.6 devices, a SET path that reports refusals, a band writer for `VC_ERR_BUSY`, the statistics struct reads and the rekeying of the embedded Link hub.
    - 13.3 DSPi Link. The policy table keyed on the 16-bit ID with a rule that derives each class, the 15 SETs that a table carried over by number would open to `control` sessions, the bridge's probe, bulk path and analyser arbitration, and the Link spec text to rewrite.
    - 13.4 DSPi Terminal. The transport widened to a command ID type, constants generated from the registry, the probe in `probe()` and `dspi raw`, every parameter row that is more than a renumber, and older packing and direction bugs that the port fixes.
- **Generic hazards in the core.** Audit findings that apply to any host went into the core sections, not into one appendix. Section 4 covers the 1.1.6 number collisions of GETs and SETs, with the unclamped `SET_PREAMP` write as the worst SET. Section 5 says to retry only busy results, which codes mean busy on each transport, and how to trust `SYS.LAST_RESULT`. Section 6.3 covers the short-read trap of the struct reads. Section 9 covers bulk handlers keyed on the old opcode. Section 11.5 covers status codes from `0x10` up, 256-byte reply buffers and the firmware's own 50 ms retry. Section 12 covers the 8-bit narrowing trap and tables keyed on `bRequest` alone.
- **Notes file absorbed.** The working notes file that recorded each behaviour difference since Phase 2 is folded into the guide and deleted. `commands.md` section 25, this plan and the feature specs that cited it point to the guide.
- **Appendices only for the four audited hosts.** Section 10.3 lists them. Any other host follows the core sections, and DSPi Console on Windows in particular follows the WinUSB notes (guide 11.2) and the porting checklist (guide 12).
- **Several writers, not one.** The plan named a single writer. Each appendix had its own writer working from its audit, and an integration pass spliced them in and made them agree with each other and with the core. Where an audit disagreed with the registry or the firmware, the registry and the firmware won.

Verification: by script, with no device attached. `scripts/gen_vendor_protocol.py --check` and its 31 tests pass. All 246 opcodes of the 1.1.6 `config.h` are in the guide's map and in `legacy_map.json`. Every `PAGE.NAME` in the guide (211 distinct) is in the registry, and the 529 IDs written beside them match. The 238 reply lengths a script could extract match the registry or the variable-length rule. All 244 UART frames have a correct CRC and field layout, and the 10 I2C frames are well formed. The 33 USB SETUP lines match their labels. The 73 request rows, 146 frames and 23 collision rows of the ESP32 appendix match the registry, the 1.1.6 numbers and `legacy_map.json`. Every map link resolves, no added line has a dash, and every item of the deleted notes file is in the guide.

Not verified on hardware. The reply bytes in the guide's USB examples and the variable reply lengths were derived from the firmware and the registry, not read from a device. No UART or I2C host exists on the bench, so every UART and I2C frame is computed only. The Windows run of the page byte is still owed (11.1). No host has been ported yet, so no appendix has been tried on a real port. With a device attached, run `python3 -m tools.dspi_test.run --group crosscut`, send each USB example of guide sections 3, 4 and 6.6 on an RP2350 and an RP2040 and compare the bytes, and run `tools/windex_page_test.py` on Windows.

## 9. Generated docs and capability data

`scripts/gen_vendor_protocol.py` (standard library only) parses the `.def` files. They are regular enough to read with a small tokenizer, with no C preprocessor needed.

| Output | Use |
|---|---|
| Command tables in `Documentation/commands.md` | Written between `<!-- VC:BEGIN page -->` and `<!-- VC:END -->` markers. Prose around them stays hand-written |
| `Documentation/protocol/dspi_commands.json` | Machine-readable registry: pages, revisions, commands, sizes, address kinds, flags, doc strings |
| `--check` mode | Fails if the committed docs or JSON differ from the `.def` files, or if a number is duplicated, out of block order, or below 0x10 |

The JSON is what hosts consume. `tools/dspi_test` and `dspi_poke` load it in place of the `config.h` regex. Host maintainers can generate their constants and policy tables from it, which would remove the hand-maintained copies of the opcode list that exist today.

At runtime the device reports the same data through `SYS.PAGE_LIST`, `SYS.PAGE_COMMANDS` and `SYS.COMMAND_INFO`. The bitmaps come straight from the table, so they reflect the platform and the build flags of the firmware that is actually running. No names are stored on the device.

## 10. Impact

### 10.1 Firmware transports

| Transport | Change |
|---|---|
| USB | New front end in `vc_usb.c`. Recipient and interface byte unchanged. Page 0 is refused. Bad SETs STALL |
| UART | No framing change. Page is the `wIndex` high byte, and a client that leaves it at 0 gets errors. Bulk special cases use the new `STORE` IDs. The status byte carries `VC_*` codes. Spec updated, including two existing drifts (bulk size, blocked list) |
| I2C | Same as UART. `vendor_is_bulk_set()` in the ISR gains a page argument |
| Control Surfaces | `cs_noun_dispatch()` switches from `REQ_*` to `VC_*` IDs through `vc_set()`. Stored bindings hold nouns, so no flash format change and no `SLOT_DATA_VERSION` bump |
| Loop profiler | The 256-entry array keyed by `bRequest` becomes keyed by a dense table index. Bench builds only |

### 10.2 Tools in this repo

- **`tools/dspi_test`.** `device.py` is the single choke point. It gains a page argument and loads the JSON registry in place of the `config.h` regex, since the `REQ_*` defines are deleted. The existing tests ran through the translation layer until Phase 6 ported them (8.13). Three spots depended on opcode values and needed care. They were `crosscut.py:35` (hard-coded free list), the `OPCODE_NAME` map keyed by integer, and the `selftest.py` mocks.
- **`dspi_poke`.** Accept `PAGE.NAME` and a `--page` option. Its own `config.h` parser is replaced by the registry.
- **Pollers and benches** (`clkdiag_poll.py`, `ring_health_poll.py`, `latency_bench`, `batch_bench`). Move literals to names from the registry. `loop_prof.py` assumes an 8-bit opcode key and follows the profiler change. `batch_bench` talks to an experimental command that is not ported, so it is left alone.

### 10.3 The migration guide

`Documentation/Features/vendor_protocol_migration_guide.md` is the deliverable for host maintainers. It must be complete enough to port a host without reading firmware source. Its contents:

1. **What changed and why**, in one page.
2. **The request format.** Every field of the control transfer, with worked byte-level examples for a GET, a SET with payload and a zero-length action.
3. **Discovery and probing.** The mandatory `PROTOCOL_INFO` probe, what a 1.1.6 device answers, page revisions, `PAGE_COMMANDS` bits, and how to replace version checks and probe-by-STALL with them.
4. **Errors.** STALL semantics on USB, `LAST_RESULT`, the `VC_*` codes, the status byte on UART and I2C, and how each old `PIN_CONFIG_*`, `PRESET_*` and `FLASH_*` byte maps to a `VC_*` code.
5. **The full old-to-new table** for all 246 opcodes, with a recipe per row: where the address goes, where the value goes, and what the reply is.
6. **Every payload**, old layout beside new layout, for every command whose bytes changed. Commands whose bytes are unchanged are listed as such.
7. **Split commands in detail**: `GET_STATUS`, `LIMITER`, `GET_EQ_PARAM`, the caps selectors, the control actions.
8. **Retired commands** and the replacement for each.
9. **Former write-as-reads**: the list of 34, with before and after for each.
10. **Per-transport notes.** USB on macOS (IOKit), USB on Windows (WinUSB, including the `wIndex` low byte rule and the 4 KB control limit that the chunked bulk command exists for), USB on Linux (usbfs interface claim), WebUSB, UART framing and status byte, I2C framing and status byte.
11. **What did not change**: bulk parameters, notifications, UAC1, descriptors.
12. **Per-host appendices**, written from a read-only audit of each repo. Each lists the files and call sites that touch the protocol and the specific changes that host needs.

| Host | Repo |
|---|---|
| DSPi ESP32 Front Panel | `/Users/weeblabs/DSPi-ESP32-Front-Panel` |
| DSPi Console for macOS | `/Users/weeblabs/DSPi Console` |
| DSPi Link | `/Users/weeblabs/DSPi-Link` |
| DSPi Terminal | `/Users/weeblabs/DSPi Terminal` |
| DSPi Console for Windows | Not on this machine. Covered by the WinUSB notes (guide 11.2) and the generic recipe (guide 12) |

The machine-readable companions are `Documentation/protocol/dspi_commands.json` (the new command set) and `Documentation/protocol/legacy_map.json` (old to new).

### 10.4 What each kind of host has to do

This is the summary. The guide carries the detail.

- **Every host.** Add the probe. Put the page in the `wIndex` high byte. Renumber. Turn the 34 write-as-reads into SETs. Replace split commands with their new commands. Handle STALL on a rejected SET. Stop using the retired commands.
- **DSPi Console (macOS).** All requests pass through three functions in `USBDevice.swift` and one `Constants.swift` table, with about 286 call sites. It sends `wIndex` 0 on some calls and 2 on others, which the page byte replaces. It uses three retired opcodes (0x44, 0x45, 0x51).
- **DSPi Link.** Frames already carry `wIndex`, so the wire format is unchanged. Its policy table, command router keys, poll subscriptions and bulk opcode lists key on `bRequest` and must key on `(page, command)`.
- **ESP32 front panel.** UART, scalar commands, strict readback. Reply bytes of commands marked Fold or Same are unchanged, so its checks keep their constants.

### 10.5 Experimental branches

`overclock` and `ac3-dts-decode` are not merged, not rebased and not given numbers by this work (section 4.1). Their old opcode numbers no longer exist after the flip. The untracked `batch_bench.h` is left alone.

## 11. Risks and early validation

| Risk | Why it matters | Mitigation |
|---|---|---|
| WinUSB alters the `wIndex` high byte on interface-recipient requests | The whole page scheme rides on that byte | Test first, before anything is built on it (11.1). WinUSB is documented to overwrite only the low byte, but that must be seen on hardware |
| No fallback path in the field | With a clean break, a wrong handler is a broken feature for every user of the beta | Golden replay and on-device A/B before the flip (7.5). Release gate (7.6) |
| A host is ported from the guide and the guide is wrong or thin | Maintainers cannot be expected to read firmware source | The guide is generated from the same tables as the firmware where possible, and each host appendix is checked against that host's code |
| A new host sends a paged command to old firmware | Old firmware ignores `wIndex` and would run an unrelated opcode | Mandatory `PROTOCOL_INFO` probe (5.14). The probe number was chosen because it STALLs on 1.1.6 |
| The harness translation layer hid a firmware bug, or had one of its own | It sat between the tests and the device | It was data-driven from the same mapping file the docs use. Native paged tests per page cross-checked it. The on-device A/B in Phase 2 did not go through it. It was deleted in Phase 6 |
| Slot alignment regression | Hard project constraint | Invariant 8.7. Handlers move verbatim. Review gate on any diff that touches a pending flag or a reset call. Loopback alignment tests in the harness |
| Flash growth during development | Old switches and new handlers coexist until the flip | Expected and temporary. The size baseline in `scripts/ram_baseline.json` is updated once, after the flip |
| RAM placement | Firmware is XIP with hot code pinned | The dispatcher and table are cold and stay in flash. Run `check_ram_placement.py` every phase. Nothing new is called from an ISR except `vendor_is_bulk_set()`, which keeps its current shape |
| RP2040 XIP linker lists | A moved file can drop out of a pull list | Phase 1 checks both linker scripts for references to moved symbols |
| A bench build reports 1.1.6 while speaking the new protocol | A build that lies about itself confuses every host | The flip phase sets the version to 1.1.7 beta 1 in the same change that removes the old opcodes |
| Scope creep into payload redesign | Each reshaped payload adds work in every host | Only two payloads are reshaped (`FILTER.BAND`, `ROUTE.MATRIX_ROUTE`). Everything else keeps its bytes |

### 11.1 The `wIndex` test (run by the user on Windows)

`tools/windex_page_test.py` runs against the normal 1.1.7 firmware. No bench build is needed. It sends requests to command 0xEE, which no page defines, so the device refuses each one and changes nothing. After each request it reads `SYS.LAST_RESULT`, which records the page, command, `wValue`, direction and result of the last request on that transport, and checks them against what was sent. Every request carries its own `wValue`, so a stale record cannot pass. The script refuses to run on 1.1.6, which ignores the page byte. The step-by-step procedure is in `Documentation/Features/windex_page_test.md`.

| Check | Send | Pass condition |
|---|---|---|
| Discovery | `SYS.PROTOCOL_INFO` on page 0x01, then the same read on page 0x00 | The first answers `DSPi`, the second is refused |
| Interface recipient, IN and OUT | A GET, a zero-length SET and a 4-byte SET (`0xC1` and `0x41`) to command 0xEE on pages 0x02, 0x55, 0x7F, 0xAA, 0xF1 and 0xFF | `LAST_RESULT` shows the page byte intact |
| Device recipient, IN and OUT | `0xC0` and `0x40`, the same requests | Recorded as the fallback result |

The script runs on macOS and Linux too, which checks the firmware and the script before the Windows run. On macOS against 1.1.7 beta 1 both recipients pass with every field exact.

If WinUSB alters the high byte, the fallback is device-recipient requests (`0xC0` and `0x40`), where `wIndex` is entirely ours. The firmware does not check the recipient today, and the MS OS 2.0 request (`wIndex` 7) stays unambiguous because it lands on page 0. Only if both fail does the page need a different carrier, and the plan returns to the user before going further.

## 12. Phased implementation

Each phase is run by Opus subagents under one orchestrator. The orchestrator assigns files, and no two agents hold the same file in the same phase. A device is attached, so agents run the harness on hardware. Every firmware phase ends with the same gate:

- `cmake --build build-rp2040 --clean-first` and `cmake --build build-rp2350 --clean-first`.
- `python3 scripts/check_ram_placement.py` on both ELFs.
- `arm-none-eabi-size` on both ELFs, recorded.
- `python3 scripts/gen_vendor_protocol.py --check`.
- `python3 -m tools.dspi_test.selftest` (no hardware).
- The hardware harness on the attached device.
- `Documentation/current_architecture.md` updated by one agent at a time.

Each phase is committed once, on `vendor-command-overhaul`, after its gate passes. Only that phase's files are staged. Nothing is pushed, tagged or merged.

Phases 0 to 6 are done. Their text below is the plan as it was carried out, kept as a record, and sections 8.8 to 8.13 say what was actually built.

### Phase 0. Validation spikes and golden capture

| | |
|---|---|
| Work | The echo command and its test script and procedure. The golden capture of 1.1.6: a write script that sets every settable command to a non-default value, a capture of every GET reply before and after it, and the mask list of volatile fields |
| Files | A bench-only `#ifdef` in `vendor_commands.c` and a CMake option. `tools/windex_echo_test.py`, `Documentation/Features/windex_echo_test.md`. New `tools/dspi_test/golden/` |
| Output | The echo UF2s for both platforms. The golden files for the attached platform |
| Gate | The user runs the Windows test and reports the result. Nothing is built on paged addressing until then |

### Phase 1. Core scaffolding and harness

Three agents in parallel.

| Agent | Owns | Work |
|---|---|---|
| 1A firmware core | `vendor/vc_core.*`, `vendor/vc_usb.c`, `vendor/vc_bulk.c`, `vendor/vc_pins.*`, `vendor/pages/sys.def`, `vendor/vc_page_sys.c`, empty `.def` and `.c` stubs for every other page, `CMakeLists.txt`, the entry points in `vendor_commands.c` and `.h` | Table types, lookup, validation, results, reply buffer. USB front end that sends other pages to the table. On this branch only, page 0 still reaches the old switches. `SYS` discovery and identity commands. Pin helpers moved out |
| 1B generator | `scripts/gen_vendor_protocol.py`, `Documentation/protocol/dspi_commands.json` | Generator with `--check` |
| 1C harness | `tools/dspi_test/device.py`, `tools/dspi_test/legacy_map.py`, `Documentation/protocol/legacy_map.json`, `tools/dspi_test/tests/paged_sys.py` | Paged transport. The translation layer and its mapping data for all 246 opcodes, switchable between "send old" and "translate to paged". Tests for the `SYS` discovery commands |

Output: a development firmware where page 0 behaves exactly as 1.1.6 (same code) and page 1 answers discovery, plus a harness that can drive either.

Verification beyond the gate: the harness in "send old" mode passes on hardware with no changes, and the golden capture replays clean against this build.

### Phase 2. Page migration

Up to five agents in parallel. Each owns one or more pages: the `.def`, the `.c`, and a `tools/dspi_test/tests/paged_<page>.py`. No agent edits `vendor_commands.c`, `CMakeLists.txt`, `vc_core.*` or the harness core. Handler logic is copied out of the old switches, which stay in place on the branch. Hardware runs are serialized by the orchestrator, since there is one device.

| Agent | Pages | Commands (approx.) |
|---|---|---|
| 2A | `SYS` telemetry block and device name, `STORE` | 24 |
| 2B | `INPUT`, `OUTPUT`, `CLOCK`, `XPORT` | 37 |
| 2C | `ROUTE`, `FILTER`, `DYN` | 36 |
| 2D | `FX`, `MEAS` | 75 |
| 2E | `CS`, `DIAG` | 30 |

Inputs for each agent: sections 3, 5, 6 and 8 of this plan, the old case bodies for its opcodes, and the 1A core.

Outputs: filled pages, paged tests, and a list of any helper each agent needed that lives in another agent's file. The orchestrator resolves those between rounds.

Verification beyond the gate:

- **On-device A/B.** For every Fold or Same command, a test reads the old opcode on page 0 and the new command and compares bytes. Both implementations are live in this phase.
- **Translated harness.** The existing tests for the agent's features pass in "translate to paged" mode.
- **Golden replay.** The agent's slice of the golden script replays through the translation layer and matches.

### Phase 3. Internal callers and transports

Three agents in parallel. This must finish before the flip, because these files still name `REQ_*` opcodes.

| Agent | Owns | Work |
|---|---|---|
| 3A | `control_surfaces_nouns.c` | Noun dispatch through `vc_set()` with `VC_*` IDs. Macro and group paths re-tested |
| 3B | `uart_control.c`, `i2c_control.c`, `Documentation/Features/control_interfaces_spec.md` | Page-aware bulk handling, `VC_*` status codes, spec refresh including the two drifts |
| 3C | `loop_prof.c`, `loop_prof.h`, `tools/latency_bench/loop_prof.py` | Profiler keyed by table index, moved to `DIAG` |

### Phase 4. The flip

One agent. It owns `vendor_commands.c` (deleted), `vendor_commands.h`, `config.h`, `vendor/vc_usb.c` and `CMakeLists.txt`.

Work:

- Delete both switches, every `REQ_*` define and the echo command.
- Refuse every page 0 request.
- Set `FW_VERSION_PATCH` to 7 and `FW_VERSION_BETA` to 1.
- Switch the harness default to "translate to paged". As built, the harness probed the firmware and picked the mode itself (8.11), until Phase 6 removed the modes.
- Make the native paged tests independent of page 0. Their old-opcode A/B halves become spec assertions.

Verification beyond the gate:

- No `REQ_` symbol remains in `firmware/DSPi/`.
- The full harness passes through the translation layer, with the quirk tests updated and listed.
- The full golden script replays and matches.
- A test walks all 256 `bRequest` values on page 0 in both directions and every one is refused.
- Flash size is recorded and `scripts/ram_baseline.json` is updated once, deliberately.

### Phase 5. Reference documentation and repo tools

Five agents in parallel, one per row below, then one closing pass. Done. Section 8.12 records the result.

| Owned files | Work |
|---|---|
| `Documentation/commands.md` | Rewritten by page around generated tables. Old numbers appear only in the pointer to the migration guide |
| `Documentation/current_architecture.md` | Vendor Command Reference and Vendor Protocol sections rewritten, old opcode references replaced, timestamps |
| `Documentation/Features/*.md`, `README.md` | Opcode references replaced by `PAGE.NAME` |
| `tools/dspi_poke/`, `tools/clkdiag_poll.py`, `tools/ring_health_poll.py` | Ported to paged calls. `dspi_poke` reads the registry |
| `tools/latency_bench/` | Ported to paged calls through the harness, with `paged.py` |

The closing pass corrected the page flags and doc strings, fixed stale firmware comments, added the generated `INDEX` block with its tests, and ran the gate on both platforms and on the device.

### Phase 6. Port the tests and delete the translation layer

Done. Section 8.13 records the result.

Agents split the 13 older test modules in `tools/dspi_test/tests/` (the ones that still called old opcode names through the translation layer) between them, one owner per file. Each test was rewritten to call paged commands by name, and tests that only duplicated `paged_*` coverage were removed. The plan was to merge the Phase 2 paged tests into the matching modules. That was not done. The `paged_*` modules check per-page protocol conformance and the older modules check feature behaviour, so they stay two families.

When every module was ported, `legacy_map.py` and the protocol modes were deleted from the harness, and `crosscut.py`, `stress.py` and `selftest.py` were fixed. `Documentation/protocol/legacy_map.json` stays as migration data, checked by the generator.

Verification: the full native harness ran on hardware with 180 of 191 passing. The one failure was a test that measured the wrong quantity, since changed (14). The golden regression was re-expressed in native requests, and the native reference was captured in the same session as a matching translated replay.

### Phase 7. Migration guide

Done. Section 8.14 records the result.

The core of `Documentation/Features/vendor_protocol_migration_guide.md` was written against the final 1.1.7 beta 1 firmware, and the working notes file was absorbed into it and deleted. One read-only audit agent per host repository in section 10.3 then reported every place that host touches the protocol. One writer per host turned each audit into an appendix, and an integration pass spliced the four appendices into section 13 and made them agree with the core. Hazards common to every host were folded into the core sections rather than repeated in each appendix.

Verification: every row of the old-to-new table, every reply length and every UART and I2C frame was checked by script against the registry, `legacy_map.json` and the 1.1.6 sources. No device was attached during this phase, so the byte-level examples have not yet been run against hardware. Section 8.14 lists what remains and the commands to run.

### Phase 8. Release readiness

Run the section 7.6 gate and report. Cutting the release waits for the user's explicit instruction.

## 13. Departures from the agreed direction

| Departure | Reason |
|---|---|
| No write-as-read anywhere. Actions are zero-length SETs, and failure is a STALL plus `LAST_RESULT` | A read that changes state is the same kind of hidden behaviour the overhaul is removing. It also makes every GET safe to probe or dump. The enabling fix is small: run zero-length SETs at SETUP |
| Page 0 is empty, not a legacy table | The user chose a clean break with no old commands kept |
| A compatibility layer existed only in the test harness, and only until the tests were ported (deleted in Phase 6) | It let the existing tests prove the new handlers, and it shipped nothing |
| Commands start at 0x10 in every page, not just 0x01 reserved | Also covers WebUSB's 0x02 and leaves room for protocol-level use at no real cost |
| `CLOCK` and `DIAG` pages added, `NAME` page dropped. Private range 0xE0-0xEF added | Section 4 |
| Parameter-indexed commands (tube, upmixer, limiter) are split per parameter | A parameter index in `wValue` is an operation selector by another name, and it hides the commands from the table and the capability data |
| Two payloads reshaped (`FILTER.BAND`, `ROUTE.MATRIX_ROUTE`) | Needed for the address-in-`wValue` and GET-equals-SET rules. Limited to two on purpose |
| The table source is X-macro `.def` files, with JSON generated from them | C consumes them with no build-time generator, and they are trivially machine-readable |

## 14. Decisions

The user answered the open questions on 2026-11-15. Three were left to the designer.

| Question | Decision |
|---|---|
| Keep `GET_PLATFORM` and `ENTER_BOOTLOADER` on page 0 | No. Page 0 is empty (7.3) |
| ESP32 front panel | The maintainer ports from the migration guide. No other hand-off |
| Windows `wIndex` test | The user runs it (11.1) |
| Hardware | A device is attached for the whole job. Agents run the harness on it |
| Reshape `FILTER.BAND` and `ROUTE.MATRIX_ROUTE` | Yes (designer's call). Every host is rewriting these call sites in this release anyway. The new layouts put the address in `wValue`, make the read return exactly what the write accepts, and end the 16-or-18-byte EQ payload. Deferring it would mean a second break later |
| Limiter value types | Keep float32 for all four limiter parameters (designer's call). The wire values stay exactly as they are in 1.1.6, which matches how tube and upmixer parameters are carried and cannot lose any behaviour. Only the numbering changes |
| Retirements | Approved. Legacy preamp (0x44, 0x45), `SAVE_PARAMS` (0x51), the USB error stats pair (0xB2, 0xB3) and `GET_STATUS` selectors 0-2 have no new command |
| `NAME` page | Dropped (designer's call). Each name lives with the thing it names (section 4) |
| `overclock`, `ac3-dts-decode` | Not merged, not rebased, no reserved numbers |
| Which hosts block the release | Not applicable. This work delivers firmware, harness and migration guide. Host apps are ported by their maintainers |
| DSPi Link | The repo is at `/Users/weeblabs/DSPi-Link`, read-only for this work. Its port is covered by a guide appendix |
| Harness translation layer | Temporary. The tests were ported to native paged calls and the layer was deleted in Phase 6 (8.13) |
| Commits | One commit per finished and verified phase, on `vendor-command-overhaul` only. No push, tag or merge |
| `DIAG.CLOCK_DIAG` transports | Served on USB, UART and I2C (2026-11-17, designer's call at the user's request). The USB-only flag existed because the packet outgrew the old 132-byte UART and I2C reply. Replies are now 256 bytes everywhere, and a static assert keeps the packet inside that |
| Meaning of `VC_F_RESTART` | Kept wide (2026-11-17, designer's call at the user's request). It marks every command that can interrupt audio, including `OUTPUT.DAC_MUTE_TEST` and `SYS.ENTER_BOOTLOADER`, because a host uses it to warn before a write that drops audio, whatever the cause |
| `CS.REVERT` took stored bindings down | Fixed 2026-11-17 at the user's request. During a revert, `control_surfaces_owns_pin()` reserved every stored pin, including the pin the slot being reloaded validates, so non-button bindings came back down with a pin-in-use status. A reload unit now validates against live bindings only. `cs_revert_reloads_stored_pot` covers it and the opt-in gate is gone |
| RTA frame-time test | The test changed, not the spec or firmware (2026-11-17, the user's decision). `last_frame_us` sums slice run time, so `rta.rta_busy_within_budget` now asserts that no background step overruns and that the per-frame cost stays under 15% (RP2350) or 30% (RP2040) of the frame period |

### 14.1 Still open

None blocks the remaining phases. These inputs and decisions are outstanding, recorded here so they are not lost.

| Item | Status |
|---|---|
| Phase 0 Windows `wIndex` result | The user runs `tools/windex_page_test.py` on Windows (11.1). It passes on macOS |
