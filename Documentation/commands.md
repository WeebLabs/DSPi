# DSPi Vendor Command Reference

*Last updated: 2026-11-16*

This is the complete reference for the vendor control protocol of DSPi firmware 1.1.7. It is written for anyone who drives the device from a host program, a network bridge or a microcontroller, such as DSPi Console and its ports, test scripts, an ESP32 front panel or a TCP gateway. It describes every request byte for byte, so a client can be written without reading the firmware source. The command tables are generated from the firmware's own command table, and the prose around them explains what the tables cannot.

## TL;DR

A host controls DSPi with USB vendor control transfers on interface 2. Every command has a two-part address. The **page** names a subsystem, such as filters, presets or inputs, and travels in the high byte of `wIndex`. The **command number** names one thing inside that page and travels in `bRequest`. In this document a command is written `PAGE.NAME`, for example `FILTER.BAND`, and its 16-bit ID is `(page << 8) | command`.

Three rules make the protocol predictable. Reading and writing the same setting use the same command, and the transfer direction says which one you mean. `wValue` only says *which* instance you mean (a channel, an output, a preset slot) and never carries a value. Reads never change anything, so every action is a write, even one with no payload.

When the device refuses a request it STALLs it. The reason is then readable from `SYS.LAST_RESULT`. Writes that touch flash or restart the audio outputs are accepted at once and finished a moment later by the firmware's main loop, so a host confirms them by reading the setting back.

A host must start every session with one read of `SYS.PROTOCOL_INFO` (page `0x01`, command `0x10`) and check that the reply starts with `DSPi`. Older firmware ignores the page and would run an unrelated command with the same number, so nothing else may be sent until that probe succeeds. After the probe the device can list its own pages and commands, so a host detects features from the device and not from version numbers.

The same commands work over the UART and I2C control transports, which carry the same fields in a small frame (section 22).

---

## 1. Transport conventions

### 1.1 The request

| Field | Value |
|---|---|
| USB VID:PID | `0x2E8B : 0xFEAA` |
| Interface | 2 (the vendor interface) |
| `bmRequestType` | `0xC1` for a GET (device to host). `0x41` for a SET (host to device). Vendor type, interface recipient |
| `bRequest` | Command number within the page, `0x10` to `0xFF` |
| `wIndex` | `(page << 8) \| 2`. The high byte is the page. The low byte is the interface number |
| `wValue` | The address: which channel, output, slot or band. 0 when the command addresses nothing |
| `wLength` | GET: the number of bytes the host accepts. SET: the payload length |
| Payload | The values. Little-endian integers, IEEE 754 single-precision floats (4 bytes, little-endian), packed structures with no padding |

The firmware reads the page from the `wIndex` high byte and ignores the low byte and the recipient bits. A host should still send interface 2 with an interface recipient, because host USB stacks (WinUSB in particular) route interface requests by that byte.

Strings (names) are fixed 32-byte fields, NUL padded, and are not guaranteed to be valid UTF-8. A name SET takes 1 to 32 bytes and the device keeps at most the first 31, so a stored name always ends in NUL.

The reference host implementation is `tools/dspi_test/device.py`. Its `pget(cmd, addr, length)` and `pset(cmd, data, addr)` take a `PAGE.NAME` or a numeric ID, look the command up in the generated registry `Documentation/protocol/dspi_commands.json`, and turn a STALL into an exception that carries the reason from `SYS.LAST_RESULT`.

### 1.2 Worked examples

The SETUP packets below are the 8 bytes on the wire. They hold `bmRequestType`, `bRequest`, then `wValue`, `wIndex` and `wLength` as 2 little-endian bytes each. Every reply shown was read from an RP2350 running firmware 1.1.7 beta 1.

**A GET.** Read `SYS.PROTOCOL_INFO` (page `0x01`, command `0x10`, 16-byte reply).

```
SETUP  C1 10 00 00 02 01 10 00
IN     44 53 50 69 01 00 0C 00 40 00 00 01 00 00 00 00
```

The reply decodes as the magic `DSPi`, protocol 1.0, 12 implemented pages, no flags, a 64-byte SET limit (`0x0040`) and a 256-byte reply limit (`0x0100`). Section 6.1 has the layout.

**A SET with a payload.** Set `ROUTE.OUTPUT_GAIN` (page `0x05`, command `0x21`) of output 0 to -3.0 dB. The output number is the address in `wValue`. The gain is the payload, a float32 (`-3.0` is `0xC0400000`).

```
SETUP  41 21 00 00 02 05 04 00
OUT    00 00 40 C0
```

Reading it back with `C1 21 00 00 02 05 04 00` returns `00 00 40 C0`.

**A zero-length action.** Clear the latched clip flags with `MEAS.CLIP_CLEAR` (page `0x09`, command `0x12`). An action is a SET with `wLength` 0, so there is no data stage.

```
SETUP  41 12 00 00 02 09 00 00
```

**A refused request and its reason.** Read `ROUTE.OUTPUT_GAIN` of output 9. No platform has an output 9 (RP2350 has outputs 0 to 8), so the device STALLs. The next request reads `SYS.LAST_RESULT` (page `0x01`, command `0x14`).

```
SETUP  C1 21 09 00 02 05 04 00     -> STALL
SETUP  C1 14 00 00 02 01 08 00
IN     21 05 01 13 09 00 00 00
```

The reply says command `0x0521` (`ROUTE.OUTPUT_GAIN`), direction 1 (GET), result `0x13` (`VC_ERR_ADDRESS`), `wValue` `0x0009`. Section 1.5 lists the codes.

### 1.3 Control stages and when a request STALLs

The device checks a request in a fixed order and stops at the first failure.

1. The page and command exist on this build. Otherwise `VC_ERR_UNKNOWN`.
2. The command has a handler for this direction. Otherwise `VC_ERR_DIRECTION`.
3. The command is allowed on this transport. Otherwise `VC_ERR_BLOCKED`.
4. For a SET, the payload length is within the command's bounds. Otherwise `VC_ERR_LENGTH`.
5. `wValue` is in range for the command's address kind (section 4.5). Otherwise `VC_ERR_ADDRESS`.
6. The command's handler runs and may refuse a value (`VC_ERR_VALUE`), a state (`VC_ERR_STATE`), a pin and so on.

On USB the stage where a refusal shows up depends on the kind of request.

| Request | Checks 1 to 5 | Handler runs | A refusal |
|---|---|---|---|
| GET | At SETUP | At SETUP | STALLs the data stage. No reply bytes are sent |
| Zero-length SET | At SETUP | At SETUP | STALLs the status stage |
| SET with payload | At SETUP | After the data stage arrives | A check failure STALLs the data stage. A handler failure STALLs the status stage |

Every outcome, success or failure, is recorded in `SYS.LAST_RESULT` for the transport it came from. The one exception is a GET of `SYS.LAST_RESULT` itself (section 1.5). A host library sees each STALL as a pipe error (`EPIPE` in libusb) on that transfer.

A STALL is an answer, not a fault. Retrying the same request gives the same answer, except for `VC_ERR_BUSY`.

### 1.4 Sizes, truncation and long replies

- **SET payloads** are at most 64 bytes (`SYS.PROTOCOL_INFO` bytes 8 and 9). A SET must use a length inside the command's bounds, shown as "SET bytes" in the tables. Most commands take exactly one length. The bulk commands `STORE.ALL_PARAMS` and `STORE.ALL_PARAMS_CHUNK` run in a separate buffer and have their own rules (section 19).
- **GET replies** are at most 256 bytes (`SYS.PROTOCOL_INFO` bytes 10 and 11), except for the bulk commands. A fixed-length reply always has the length in the "GET bytes" column. A `var` reply has the length the command describes.
- **Truncation.** The device sends at most `wLength` bytes. A shorter `wLength` gets a prefix of the same reply (for example 6 bytes of `SYS.PROTOCOL_INFO` return `44 53 50 69 01 00`). A longer `wLength` gets the whole reply and a short packet, which is how a host learns the length of a `var` reply. Ask for the length you know.
- **Long data** is read in pieces addressed by a byte offset in `wValue`. `MEAS.RTA_BINS` returns at most 256 bytes per request, and `STORE.ALL_PARAMS_CHUNK` moves the bulk structure in pieces.
- **Bulk commands** (`BULK` flag) use the 6136-byte bulk buffer instead of the 64-byte receive buffer and the 256-byte reply buffer. They are `STORE.ALL_PARAMS`, `STORE.ALL_PARAMS_CHUNK` and `MEAS.RTA_BANDS_ALL`. Only one transport can hold the bulk buffer at a time. A request that finds it held answers `VC_ERR_BUSY`.

### 1.5 Result codes and `SYS.LAST_RESULT`

One set of result codes covers every command.

| Code | Name | Meaning |
|---|---|---|
| `0x00` | `VC_OK` | Done, or accepted and queued for deferred work |
| `0x10` | `VC_ERR_UNKNOWN` | No such page or command on this build (every page 0 request, every command this platform or build lacks) |
| `0x11` | `VC_ERR_DIRECTION` | The command has no handler for this direction |
| `0x12` | `VC_ERR_LENGTH` | SET payload length outside the allowed range |
| `0x13` | `VC_ERR_ADDRESS` | `wValue` out of range, or names something that does not exist |
| `0x14` | `VC_ERR_VALUE` | A payload field is out of range, or a float is NaN or infinite |
| `0x15` | `VC_ERR_UNSUPPORTED` | The value is valid but this platform cannot do it |
| `0x16` | `VC_ERR_BLOCKED` | Not allowed on this transport |
| `0x17` | `VC_ERR_BUSY` | Try again shortly. An earlier request of the same kind is still queued, or the bulk buffer is held |
| `0x18` | `VC_ERR_STATE` | Not allowed in the current state (output active, feature disabled, input in use) |
| `0x19` | `VC_ERR_PIN_INVALID` | The GPIO cannot be used for this function |
| `0x1A` | `VC_ERR_PIN_IN_USE` | The GPIO is already claimed by another function |

`SYS.LAST_RESULT` returns 8 bytes describing the last request on the transport it is read from.

| Offset | Type | Field |
|---|---|---|
| 0 | u16 | Command ID, `(page << 8) \| cmd` |
| 2 | u8 | Direction: 1 GET, 0 SET |
| 3 | u8 | Result code |
| 4 | u16 | `wValue` of that request |
| 6 | u16 | Reserved, 0 |

A GET of `SYS.LAST_RESULT` is never recorded, whether it succeeds or is refused (for example with a non-zero `wValue`), so reading it never overwrites it. Every other request is recorded, a SET of `SYS.LAST_RESULT` and the discovery commands included. A UART or I2C frame that fails at the transport (status `0x01` to `0x07`) never reaches a command and is not recorded. The record is kept per transport (USB, UART, I2C), not per host. Two USB hosts on one device, such as DSPi Console and a script, overwrite each other's result. A host should read it straight after the STALL it wants to explain and check that the ID, direction and `wValue` match its own request.

### 1.6 Deferred operations and the busy window

Some writes are not finished when the request completes. The handler validates the request, queues it, and returns `VC_OK`. The firmware's main loop then does the work. These commands carry the `DEFERRED` flag. They include every flash write, input source switches, output type and pin changes, preset loads, bulk applies, filter band changes and Control Surfaces edits.

The work that follows can make the device briefly unresponsive.

- A **flash write** masks interrupts for about 45 ms per 4 KB sector, so USB control transfers are not serviced. Commands with the `FLASH` flag do this.
- An **output restart** mutes every output, waits for the DAC hardware mute hold if one is configured, and restarts all output slots together so they stay sample-aligned. Every flash write ends with one, because the blackout stops the output clocks mid-frame. A pin or clock change for an input restarts the outputs only while that input is the active source.
- Commands with the `RESTART` flag can interrupt audio. Most do it with an output restart. The input pin and clock commands mute the outputs while a live input restarts, `OUTPUT.DAC_MUTE_TEST` pulses the DAC hardware mute, and `SYS.ENTER_BOOTLOADER` reboots. A command without the flag never restarts or mutes the outputs. `STORE.FACTORY_RESET` restarts the outputs but writes no flash.

A host should expect a timeout or a STALL right after such a command. On a timeout, wait about 150 ms and retry, then confirm the device answers a cheap read such as `SYS.PLATFORM` before continuing. The reference `device.py` retries a timeout four times with a 150 ms back-off and re-opens the device if it re-enumerates.

Two deferred requests of the same kind do not queue behind each other. Where a command keeps a single pending slot, a second request before the first is applied answers `VC_ERR_BUSY` (`FILTER.BAND`, `FILTER.BAND_BYPASS` and the Control Surfaces preview SETs). Elsewhere a later request replaces the queued value.

### 1.7 Confirming a deferred write

`VC_OK` on a deferred command means the request passed every check the handler can make and is queued. A few checks can only run in the main loop, for example some pin conflicts, so confirmation means reading the setting back. Section 23 maps every write to its readback. Some commands also have their own completion signal.

| Command | Completion signal |
|---|---|
| `STORE.PRESET_LOAD` | `STORE.PRESET_ACTIVE` changes, and notifications `PRESET_LOADED` then `BULK_INVALIDATED` arrive |
| `OUTPUT.LATENCY` | `OUTPUT.LATENCY_STATUS` byte 8 (`pending`) drops to 0 |
| `CS.*` preview SETs, `CS.SAVE`, `CS.REVERT` | `CS.STATUS` byte 0 leaves `0x16` (PENDING) |
| `XPORT.UART_CONFIG`, `XPORT.I2C_CONFIG` | `XPORT.STATUS` carries the apply result |
| `OUTPUT.DAC_MUTE_CONFIG` | `OUTPUT.DAC_MUTE_CONFIG` reads back what was sent only if the apply accepted it |
| Names (`SYS.DEVICE_NAME`, `INPUT.SOURCE_NAME`) | The name reads back at once. Notification `DEVICE_NAME` or `INPUT_SOURCE_NAME` arrives when it is stored |

---

## 2. Discovery and compatibility

### 2.1 The mandatory probe

Before any other request, a host sends one GET of `SYS.PROTOCOL_INFO` (page `0x01`, command `0x10`, 16 bytes).

- A reply that starts with `DSPi` means the paged protocol is available. The protocol major version (byte 4) is 1. It changes only on an incompatible change, so a host should refuse a major it does not know.
- A STALL means firmware 1.1.6 or older. Send no other paged request to it. Such firmware ignores `wIndex`, so a paged command would run as whatever old command shares its number (for example, number `0x53` was a factory reset). The probe itself is safe because no old firmware has a GET handler for `bRequest` `0x10`. Old firmware still answers its own version read (`bRequest` `0x7F`, `wIndex` 2, 7-byte reply, same layout as `SYS.PLATFORM`), so a host can show the version and offer an update.

### 2.2 Pages and revisions

`SYS.PAGE_LIST` returns two bytes per implemented page, `{page, revision}`, in ascending page order. A page that a build compiles out entirely is absent, which is how release builds hide `DIAG`. A page's revision increases whenever a command is added to it or a reply or payload grows at the end.

### 2.3 Command bitmaps

`SYS.PAGE_COMMANDS` with `wValue` = page number returns 64 bytes. Bytes 0 to 31 are the GET bitmap and bytes 32 to 63 are the SET bitmap. Command `n` has a GET when bit `n & 7` of byte `n >> 3` is set, and a SET when the same bit of byte `32 + (n >> 3)` is set. A page this build does not implement (page 0, or `DIAG` in a release build) STALLs with `VC_ERR_ADDRESS`. It does not return an empty bitmap.

The bitmaps come from the table of the running firmware, so they reflect its platform and its build options.

### 2.4 Command info

`SYS.COMMAND_INFO` with `wValue` = command ID returns 8 bytes about one command. An ID the build does not have answers `VC_ERR_ADDRESS`.

| Offset | Type | Field |
|---|---|---|
| 0 | u16 | Flags (below) |
| 2 | u16 | GET reply length. 0 when there is no GET, `0xFFFF` when the length varies |
| 4 | u8 | Smallest SET payload |
| 5 | u8 | Largest SET payload. Both are 0 for a bulk command and for a zero-length action |
| 6 | u8 | Address kind (section 4.5) |
| 7 | u8 | Directions: bit 0 GET, bit 1 SET |

| Bit | Value | Flag | Meaning |
|---|---|---|---|
| 0 | `0x0001` | `USB_ONLY` | Both directions refused on UART and I2C (`VC_ERR_BLOCKED`) |
| 1 | `0x0002` | `SET_USB_ONLY` | The SET is refused on UART and I2C. The GET works everywhere |
| 2 | `0x0004` | `BULK` | The payload or reply uses the bulk buffer |
| 3 | `0x0008` | `DEFERRED` | The main loop finishes the work after the reply (section 1.6) |
| 4 | `0x0010` | `FLASH` | Writes flash, so a busy window follows |
| 5 | `0x0020` | `RESTART` | Can restart or mute the outputs (section 1.6) |
| 6 | `0x0040` | `KEEPALIVE` | The GET refreshes the spectrum analyser's idle timer |

Bits 0 to 2 are enforced by the device. Bits 3 to 6 describe what the command does, for documentation and for hosts that want to show a busy indicator. Bit values never change.

For example, `SYS.COMMAND_INFO` of `0x0610` (`FILTER.BAND`) returns `08 00 10 00 10 10 08 03`. That is flag `DEFERRED`, a 16-byte GET, a 16-byte SET, address kind 8 (`CH_BAND`) and both directions.

### 2.5 Feature detection

A host decides whether a feature exists by asking the device, never by comparing firmware versions.

- To test one command, look for its bit in `SYS.PAGE_COMMANDS`, or read `SYS.COMMAND_INFO`.
- To test a whole subsystem, look for its page in `SYS.PAGE_LIST` and compare the page revision.
- To size a request, use the lengths from `SYS.COMMAND_INFO` or from the registry.

Some commands exist only on one platform or in one kind of build. A command that is not built does not exist: it answers `VC_ERR_UNKNOWN` and has no bit in `SYS.PAGE_COMMANDS`. The generated tables mark such commands with "(build: ...)".

| Condition | Commands |
|---|---|
| RP2350 only (`PICO_RP2350`) | `INPUT.ADAT_RX_*`, `OUTPUT.ADAT_TX_*`, `FX.UPMIX_*` |
| Bench builds only | Every command of the `DIAG` page, each behind its own build option |

The registry `Documentation/protocol/dspi_commands.json` is generated from the same table as the firmware. It holds every page, revision, command number, name, direction, size, address kind, flag, build condition and doc string, and hosts can generate their constants from it.

### 2.6 Page 0 and the Microsoft OS 2.0 request

Page 0 has no commands. Every request with a zero `wIndex` high byte is refused with `VC_ERR_UNKNOWN` at SETUP, so nothing on the device changes. A host built for firmware 1.1.6 sends `wIndex` 0 or 2, so all its requests land on page 0 and are refused.

The one exception is the Microsoft OS 2.0 descriptor request (IN, `bRequest` `0x01`, `wIndex` 7), which Windows sends to bind WinUSB. It is not a DSPi command and is answered as before. Command numbers `0x00` to `0x0F` are reserved in every page for requests of this kind, so no DSPi command will ever collide with it.

---

## 3. Rules every command follows

These rules hold for every command in every page. A host can rely on them.

1. **One number per setting.** A setting that can be read and written is one command with a GET and a SET. There are no separate read and write commands for the same thing.
2. **A GET changes nothing.** No read changes configuration, runtime state or flash. Two narrow exceptions exist. A spectrum analyser read refreshes the analyser's idle timer and can start it (`KEEPALIVE`). A chunked bulk read takes the bulk buffer for the length of the session.
3. **Anything that changes the device is a SET.** That includes actions with no value, which are zero-length SETs.
4. **`wValue` is an address.** It names which instance, such as a channel, an output, a slot, a band or a byte offset. It never selects an operation or a parameter. A two-level address packs as `(major << 8) | minor`. A command with no address requires `wValue` 0.
5. **Values go in the payload.** A GPIO number, a mode and an enable flag are all values.
6. **One command, one shape.** Each command has exactly one GET reply layout and one SET payload layout.
7. **GET and SET are symmetric.** For a read-write command, the GET reply is a valid SET payload with the same meaning. Status and capability data are separate read-only commands.
8. **Shapes only grow at the end.** A reply or payload may gain trailing fields in a later page revision. A host that asks for the length it knows keeps working, and the device keeps accepting every shorter SET length it has ever documented.
9. **Numbers are never reused.** A retired command stays reserved.
10. **Command numbers `0x00` to `0x0F` are reserved** in every page for protocol-level use.
11. **Failure is explicit.** A refused request STALLs on USB and returns a result code on UART and I2C. Nothing is silently dropped. A SET with the wrong length, an out-of-range address or (where the command says so) a NaN or infinite float is refused and changes nothing.
12. **`0xFF` in an address means "all"** only on a SET, and only for commands whose address kind says so (`OUTPUT_ALL`).
13. **A GPIO value of `0xFF` means "reset to default".** Every pin SET treats `0xFF` as the function's default pin (or, where the function has no default, as "unset"). GPIO 0 means GPIO 0.
14. **Out-of-range finite values are clamped where the command says "clamped".** Otherwise the command refuses them with `VC_ERR_VALUE`. A read after a clamped write returns the clamped value.

---

## 4. Index and addressing reference

Several index spaces exist. Mixing them up is the most common integration bug.

### 4.1 Channel index

`NUM_CHANNELS` is 7 on RP2040 and 17 on RP2350. Inputs and outputs share one index space, inputs first. Inputs are channels in their own right, with their own EQ and meters. Channel indices address EQ bands, channel delays, channel names, meters and the analyser's band data.

| Index | RP2040 (2 in + 5 out) | RP2350 (8 in + 9 out) |
|---|---|---|
| 0 | Input 1 (L) | Input 1 (L) |
| 1 | Input 2 (R) | Input 2 (R) |
| 2 | Out 1: S/PDIF 1 L | Input 3 |
| 3 | Out 2: S/PDIF 1 R | Input 4 |
| 4 | Out 3: S/PDIF 2 L | Input 5 |
| 5 | Out 4: S/PDIF 2 R | Input 6 |
| 6 | Out 5: PDM sub | Input 7 |
| 7 | - | Input 8 |
| 8 | - | Out 1: S/PDIF 1 L |
| 9 | - | Out 2: S/PDIF 1 R |
| 10 | - | Out 3: S/PDIF 2 L |
| 11 | - | Out 4: S/PDIF 2 R |
| 12 | - | Out 5: S/PDIF 3 L |
| 13 | - | Out 6: S/PDIF 3 R |
| 14 | - | Out 7: S/PDIF 4 L |
| 15 | - | Out 8: S/PDIF 4 R |
| 16 | - | Out 9: PDM sub |

`NUM_INPUT_CHANNELS` (2 or 8) is the platform maximum. How many inputs carry audio depends on the active source. USB follows its alt setting (2, 4, 6 or 8 channels on RP2350), I2S follows `INPUT.I2S_RX_CHANNELS`, ADAT is always 8 and S/PDIF is always 2. `INPUT.STATUS` and `MEAS.LEVELS` report the live count. Input channels have EQ and meters but no crossover. Output channels have EQ, crossover, gain, delay and mute.

### 4.2 Output index

`NUM_OUTPUT_CHANNELS` is 5 on RP2040 and 9 on RP2350. Output indices address the matrix columns, the per-output controls in `ROUTE`, the limiter, and every per-output bit mask (bit k is output k). Output `k` is channel `k + NUM_INPUT_CHANNELS`, so add 2 on RP2040 and 8 on RP2350. The last output is always the PDM sub.

| Output | RP2040 channel | RP2350 channel | Meaning |
|---|---|---|---|
| 0 | 2 | 8 | S/PDIF 1 L |
| 1 | 3 | 9 | S/PDIF 1 R |
| ... | ... | ... | ... |
| `NUM_OUTPUT_CHANNELS - 1` | 6 | 16 | PDM sub |

### 4.3 Output slots and pin outputs

An **output slot** is one physical stereo output (`NUM_SPDIF_INSTANCES` is 2 on RP2040 and 4 on RP2350). Slot `s` carries outputs `2s` and `2s + 1`. Any slot can be S/PDIF or I2S (`OUTPUT.TYPE`), and the output index does not change when a slot changes type. Bit `s` of `FX.CROSSFEED_OUTPUTS` is slot `s`.

A **pin output** is one output data pin (`NUM_PIN_OUTPUTS` is 3 on RP2040 and 5 on RP2350). There is one per slot, then the PDM sub.

| Pin output | Output | Default GPIO |
|---|---|---|
| 0 | Slot 0 (outputs 0, 1) | 6 |
| 1 | Slot 1 (outputs 2, 3) | 7 |
| 2 (RP2350) | Slot 2 (outputs 4, 5) | 8 |
| 3 (RP2350) | Slot 3 (outputs 6, 7) | 9 |
| last (2 on RP2040, 4 on RP2350) | PDM sub | 10 |

### 4.4 Band index

`FILTER.BAND` and `FILTER.BAND_BYPASS` address one band as `(channel << 8) | band`.

| Band | Meaning |
|---|---|
| 0 to 9 | Parametric EQ bands (10 per channel today) |
| 10 to 19 | Reserved gap. Refused with `VC_ERR_ADDRESS` |
| 20 to 23 | Crossover bands 0 to 3. Output channels only. On an input channel they are refused with `VC_ERR_ADDRESS` |

The bulk structure stores 12 EQ bands per channel (`max_bands` in its header), so the active count may grow without a format change.

### 4.5 Address kinds

Every command has one address kind. It fixes what `wValue` means and which values pass check 5 of section 1.3. `SYS.COMMAND_INFO` byte 6 carries the number.

| # | Kind | `wValue` | Valid range RP2040 | Valid range RP2350 |
|---|---|---|---|---|
| 0 | `NONE` | Nothing | 0 only | 0 only |
| 1 | `CHANNEL` | Channel (4.1) | 0-6 | 0-16 |
| 2 | `INPUT` | Input channel | 0-1 | 0-7 |
| 3 | `OUTPUT` | Output (4.2) | 0-4 | 0-8 |
| 4 | `OUTPUT_ALL` | Output, or `0xFF` for all outputs on a SET | 0-4, `0xFF` | 0-8, `0xFF` |
| 5 | `SLOT` | Output slot (4.3) | 0-1 | 0-3 |
| 6 | `PIN_OUTPUT` | Pin output (4.3) | 0-2 | 0-4 |
| 7 | `PRESET` | Preset slot | 0-9 | 0-9 |
| 8 | `CH_BAND` | `(channel << 8) \| band` (4.4) | channel 0-6 | channel 0-16 |
| 9 | `IN_OUT` | `(input << 8) \| output` | input 0-1, output 0-4 | input 0-7, output 0-8 |
| 10 | `SOURCE` | Input source ID (4.6) | 0-6 | 0-6 |
| 11 | `SPDIF_RX` | S/PDIF input index | 0-3 | 0-3 |
| 12 | `I2S_PAIR` | I2S input stereo pair | 0 | 0-3 |
| 13 | `ROLE` | I2S clock pair role | 0-1 | 0-1 |
| 14 | `CS_SLOT` | Control Surfaces binding slot | 0-15 | 0-15 |
| 15 | `CS_SUB` | IR command sub-slot | 0-15 | 0-15 |
| 16 | `CS_GROUP` | Target group | 0-7 | 0-7 |
| 17 | `CS_MACRO` | Macro | 0-7 | 0-7 |
| 18 | `CS_MACRO_STEP` | `(step << 8) \| macro` | step 0-7, macro 0-7 | step 0-7, macro 0-7 |
| 19 | `CS_PAGE` | Display page | 0-15 | 0-15 |
| 20 | `CS_NOUN` | Control Surfaces noun | 0 to `noun_count - 1` | 0 to `noun_count - 1` |
| 21 | `PAGE` | Protocol page number | 0-255 | 0-255 |
| 22 | `PAGE_CMD` | Command ID `(page << 8) \| cmd` | any, checked by the command | any, checked by the command |
| 23 | `OFFSET` | Byte offset | any, checked by the command | any, checked by the command |
| 24 | `INDEX` | Table or channel index | any, checked by the command | any, checked by the command |

A whole 16-bit `wValue` is checked. A stray high byte (for example `0x0100` for output 0) is `VC_ERR_ADDRESS`. For `CH_BAND`, `PAGE`, `PAGE_CMD`, `OFFSET` and `INDEX` the dispatcher checks what it can, and the command checks the rest and also answers `VC_ERR_ADDRESS`.

### 4.6 Input sources

| ID | Source | RP2040 | RP2350 |
|---|---|---|---|
| 0 | USB | yes | yes |
| 1 | S/PDIF input 1 | yes, always enabled | yes, always enabled |
| 2 | I2S | yes | yes |
| 3 | ADAT | no | only while `INPUT.ADAT_RX_ENABLE` is 1 and a pin is set |
| 4 | S/PDIF input 2 | while enabled with `INPUT.SPDIF_RX_ENABLE` | same |
| 5 | S/PDIF input 3 | same | same |
| 6 | S/PDIF input 4 | same | same |

S/PDIF input index `i` (`SPDIF_RX` address kind) is source 1 for `i` = 0 and source `i + 3` otherwise.

### 4.7 Filter types

`FILTER.BAND` byte 0 and the bulk band records use these values.

| Value | Type | Parameters used |
|---|---|---|
| 0 | Flat (off) | none |
| 1 | Peaking | freq, Q, gain |
| 2 | Low shelf | freq, Q, gain |
| 3 | High shelf | freq, Q, gain |
| 4 | Low pass (2nd order) | freq, Q |
| 5 | High pass (2nd order) | freq, Q |
| 6 | Notch | freq, Q |
| 7 | All pass (2nd order) | freq, Q |
| 8 | All pass (1st order) | freq |
| 9 | Low shelf (1st order) | freq, gain |
| 10 | High shelf (1st order) | freq, gain |
| 11 | Linkwitz Transform | freq = f0, Q = Q0, gain field = fp in Hz, `qp_x512` = Qp x 512 (0 selects 0.707) |
| 12 | Low pass (1st order) | freq |
| 13 | High pass (1st order) | freq |
| 14 to 31 | Reserved for future EQ types | |
| 32 to 63 | Crossover types | freq |

Crossover encoding, used in crossover bands 20 to 23.

| Family | Low pass / high pass values |
|---|---|
| Linkwitz-Riley LR2, LR4, LR6, LR8 | 32/33, 34/35, 36/37, 38/39 |
| Butterworth BW1 to BW8 | 40/41 (BW1), 42/43 (BW2), and so on to 54/55 (BW8) |
| Bessel BES2, BES4, BES6, BES8 | 56/57, 58/59, 60/61, 62/63 |

The device stores the type byte as sent and does not check it against the band kind. A host must send an EQ type to an EQ band and a crossover type to a crossover band. Unused parameters should still be sent, so the band reads back as written. See `Features/peq_filters.md` and `Features/crossover_filters_spec.md`.

### 4.8 GPIO pins

Without a board-specific pin map, a GPIO is usable when it is not 23, 24 or 25 (power and LED) and is at most 28 on RP2040 or 29 on RP2350. A build with a platform pin map (`DSPI_PLATFORM_PINS`, for ports to other boards) applies that map's rule per function instead. Some functions add their own rule. MCK needs a clock-output pin (GPIO 21 on RP2040, GPIO 13, 15 or 21 on RP2350). An I2S clock base pin needs the next GPIO free as well, because LRCLK is BCK + 1.

Pin SETs refuse a GPIO that another function already claims with `VC_ERR_PIN_IN_USE`. The claim check covers these owners.

- Every output data pin (S/PDIF or I2S slots, PDM).
- S/PDIF input 1 always, and S/PDIF inputs 2 to 4 only while enabled.
- The ADAT output and ADAT input pins (RP2350) while each is enabled. The ADAT input may share the ADAT output's pin for a loopback test.
- The I2S BCK and LRCLK pair (both pairs in split clock-pin mode) while any slot is I2S or I2S is the active input.
- The I2S input data pin of every active stereo pair.
- MCK while enabled.
- The DAC hardware mute pin.
- The UART and I2C control pins while that interface is live.
- GPIOs owned by live Control Surfaces bindings.

Pin moves are RAM only. They persist when they are saved with the IO configuration (section 20.8).

---

## 5. Page map

The page map is generated from the firmware's page table. Pages `0x0D` to `0x7F` are free for future subsystems, `0x80` to `0xDF` and `0xF1` to `0xFF` are reserved, and `0xE0` to `0xEF` are kept for private use by forks and ports and are never allocated upstream.

<!-- VC:BEGIN PAGES -->

| Page | Mnemonic | Revision | Commands | Description |
|---|---|---|---|---|
| 0x01 | `SYS` | 1 | 20 | Discovery, identity, device name, last result, bootloader, system and stream telemetry |
| 0x02 | `STORE` | 1 | 12 | Preset slots, directory, startup, IO config persistence, factory reset, bulk parameters |
| 0x03 | `INPUT` | 1 | 22 | Source select, S/PDIF RX, I2S RX, ADAT RX, LG Sound Sync, source names |
| 0x04 | `OUTPUT` | 1 | 13 | Output type and pins, output latency, ADAT TX, DAC hardware mute |
| 0x05 | `ROUTE` | 1 | 15 | Matrix, per-output controls, delays, channel names, preamp, master and user volume |
| 0x06 | `FILTER` | 1 | 3 | EQ and crossover bands, bypasses |
| 0x07 | `DYN` | 1 | 17 | Limiter, leveller, loudness |
| 0x08 | `FX` | 1 | 57 | Crossfeed, subharmonic synth, psychoacoustic bass, tube, upmixer |
| 0x09 | `MEAS` | 1 | 20 | Level meters, clip flags, spectrum analyser, signal generator |
| 0x0A | `CS` | 1 | 26 | Control Surfaces |
| 0x0B | `XPORT` | 1 | 5 | UART and I2C control transports |
| 0x0C | `CLOCK` | 1 | 9 | I2S clock pins and modes, MCK |
| 0xF0 | `DIAG` | 1 | 15 | Bench and diagnostic commands with no stability promise |

<!-- VC:END PAGES -->

Each page section below starts with the generated table. Its columns are as follows. "Dir" is G for a GET, S for a SET with a payload and S0 for a zero-length SET. "wValue" is the address kind. "GET bytes" and "SET bytes" are the reply length and the accepted payload lengths, where `var` is a variable length and `bulk` uses the bulk buffer. "Flags" are the `SYS.COMMAND_INFO` flags of section 2.4.

---

## 6. SYS (0x01)

Discovery, identity, the device name, the last result, the bootloader jump, and device-health telemetry. Everything here works on both platforms and every transport.

<!-- VC:BEGIN SYS -->

| Cmd | Name | Dir | wValue | GET bytes | SET bytes | Flags | Notes |
|---|---|---|---|---|---|---|---|
| 0x10 | `PROTOCOL_INFO` | G | none | 16 | - |  | Protocol magic, version, page count and transfer limits |
| 0x11 | `PAGE_LIST` | G | none | var | - |  | Page number and revision of every implemented page |
| 0x12 | `PAGE_COMMANDS` | G | page | 64 | - |  | GET then SET bitmaps of the commands a page implements |
| 0x13 | `COMMAND_INFO` | G | `(page << 8) \| cmd` | 8 | - |  | Flags, sizes and address kind of one command |
| 0x14 | `LAST_RESULT` | G | none | 8 | - |  | Result of the last request on this transport |
| 0x20 | `PLATFORM` | G | none | 7 | - |  | Platform ID, packed version, output count, then minor, patch and beta numbers |
| 0x21 | `SERIAL` | G | none | 16 | - |  | USB serial number string |
| 0x22 | `BUILD_INFO` | G | none | 64 | - |  | Git describe and build date |
| 0x23 | `CORE1_MODE` | G | none | 1 | - |  | Current core 1 mode |
| 0x24 | `DEVICE_NAME` | G, S | none | 32 | 1-32 | DEFERRED, FLASH, RESTART | Board name, NUL padded; a SET keeps at most 31 bytes |
| 0x30 | `ENTER_BOOTLOADER` | S0 | none | - | 0 | RESTART | Reboot into the USB bootloader after the reply has gone out |
| 0x40 | `SYSTEM_STATUS` | G | none | 16 | - |  | clk_sys Hz, sample rate Hz, core mV, temperature cdeg, CPU loads |
| 0x41 | `USB_AUDIO_STATUS` | G | none | 16 | - |  | USB audio packets, ring overruns, worst packet gap us, alt, mounted |
| 0x42 | `BUFFER_STATS` | G | none | 44 | - |  | BufferStatsPacket; each read advances its sequence counter |
| 0x43 | `PIPELINE_STATS` | G | none | 24 | - |  | Output ring underruns, drops, min prewrite, input gap us, background over budget |
| 0x44 | `PDM_STATS` | G | none | 16 | - |  | PDM ring overruns, ring underruns, DMA overruns, DMA underruns |
| 0x45 | `SPDIF_TX_STATS` | G | none | 24 | - |  | USB audio packet gaps of 2 to 50 ms, total output DMA starvations, starvations of instances 0-3 |
| 0x4F | `STATS_RESET` | S | none | - | 1 |  | Bitmask; bit 0 resets buffer watermarks and worst-gap fields |
| 0x50 | `PIN_STATE` | G | none | 24 | - |  | Pin map generation, GPIO count, claim count, last restore substitutions, GPIO mask |
| 0x51 | `PIN_CLAIMS` | G | none | var | - |  | Generation, entry count, then one 4-byte claim per pin use, sorted by GPIO |

<!-- VC:END SYS -->

### 6.1 Discovery commands

`SYS.PROTOCOL_INFO` (16 bytes) is the probe of section 2.1.

| Offset | Type | Field |
|---|---|---|
| 0 | u8[4] | Magic `"DSPi"` |
| 4 | u8 | Protocol major (1) |
| 5 | u8 | Protocol minor (0) |
| 6 | u8 | Number of implemented pages |
| 7 | u8 | Flags, reserved, 0 |
| 8 | u16 | Largest non-bulk SET payload (64) |
| 10 | u16 | Largest non-bulk GET reply (256) |
| 12 | u32 | Reserved, 0 |

`SYS.PAGE_LIST`, `SYS.PAGE_COMMANDS` and `SYS.COMMAND_INFO` are described in sections 2.2 to 2.4, and `SYS.LAST_RESULT` in section 1.5. The three discovery commands are recorded in `SYS.LAST_RESULT` like any other request, success included. A GET of `SYS.LAST_RESULT` is the only request never recorded, even when it is refused.

### 6.2 Identity

`SYS.PLATFORM` (7 bytes).

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Platform: 0 RP2040, 1 RP2350 |
| 1 | u8 | Firmware major |
| 2 | u8 | `(minor << 4) \| patch`, a legacy packed form |
| 3 | u8 | `NUM_OUTPUT_CHANNELS` (5 or 9) |
| 4 | u8 | Firmware minor |
| 5 | u8 | Firmware patch |
| 6 | u8 | Beta number, 0 for a final release |

Firmware 1.1.7 beta 1 on RP2350 answers `01 01 17 09 01 07 01`. The version is for display and for update offers. Feature decisions use section 2.5.

`SYS.SERIAL` is the 16-character USB serial number string, ASCII, with no terminator. `SYS.BUILD_INFO` is 64 bytes of ASCII. Bytes 0 to 47 hold the `git describe` string, bytes 48 to 59 the build date `YYYY-MM-DD`, and the rest is zero. It is provenance for people and no host logic should parse it.

`SYS.CORE1_MODE` is one byte, 0 idle, 1 PDM output or 2 EQ worker. Core 1 runs either the PDM sub output or the EQ of outputs 2 to 7 (RP2350) or 2 to 3 (RP2040), never both. `ROUTE.OUTPUT_ENABLE_CONFLICT` tells whether enabling an output would collide with the current mode.

### 6.3 Device name

`SYS.DEVICE_NAME` is the board's user name, 32 bytes NUL padded, all zeros when unnamed. A SET takes 1 to 32 bytes, keeps at most 31, and clears the name when the first byte is NUL. The new name reads back at once. The main loop writes it to the preset directory in flash and then sends notification `DEVICE_NAME` (`0x10`). The name is device-wide and does not travel with presets. Spec: `Features/device_name_spec.md`.

### 6.4 Bootloader

`SYS.ENTER_BOOTLOADER` reboots into the RP2 USB bootloader. Over USB the reboot happens after the status stage, so the host sees a successful transfer. Over UART and I2C it happens after the response frame has gone out. The device then disappears and re-enumerates as the `RPI-RP2` mass-storage drive, and answers no vendor request until new firmware is flashed and it boots back into the application. Gate it behind an explicit user confirmation.

### 6.5 Telemetry

`SYS.SYSTEM_STATUS` (16 bytes).

| Offset | Type | Field |
|---|---|---|
| 0 | u32 | `clk_sys` frequency, Hz |
| 4 | u32 | Device sample rate, Hz |
| 8 | u16 | Core voltage, mV |
| 10 | i16 | Chip temperature, hundredths of a degree C |
| 12 | u8 | Core 0 load, percent |
| 13 | u8 | Core 1 load, percent |
| 14 | u16 | Reserved, 0 |

`SYS.USB_AUDIO_STATUS` (16 bytes).

| Offset | Type | Field |
|---|---|---|
| 0 | u32 | USB audio packets received |
| 4 | u32 | USB audio ring overruns |
| 8 | u32 | Worst gap between USB audio packets, us (reset by `SYS.STATS_RESET`) |
| 12 | u8 | Current USB audio alt setting |
| 13 | u8 | 1 while the USB audio interface is mounted |
| 14 | u16 | Reserved, 0 |

`SYS.PIPELINE_STATS` (24 bytes) describes the output rings that feed every output slot.

| Offset | Type | Field |
|---|---|---|
| 0 | u32 | Output ring underrun episodes |
| 4 | u32 | Blocks refused by ring admission (drops) |
| 8 | u32 | Lowest ring fill seen before a write, frames. `0xFFFFFFFF` until the first sample. Reset by `SYS.STATS_RESET` |
| 12 | u32 | Worst gap between input deliveries, us. Reset by `SYS.STATS_RESET` |
| 16 | u32 | Background steps that ran over their time budget |
| 20 | u32 | Reserved, 0 |

`SYS.PDM_STATS` (16 bytes) is four u32 counters. They count PDM ring overruns, PDM ring underruns, PDM DMA overruns and PDM DMA underruns.

`SYS.SPDIF_TX_STATS` (24 bytes) is six u32 counters. The first, named `spdif_underruns` in the firmware, counts USB audio packet gaps longer than 2 ms and shorter than 50 ms (not counted during a preset load), the gaps that would starve the outputs. It does not measure the S/PDIF transmitters. Then come total output DMA starvations and the starvations of slots 0, 1, 2 and 3. Slots 2 and 3 read 0 on RP2040.

`SYS.BUFFER_STATS` (44 bytes) is the per-slot ring fill picture. Each read advances its sequence counter, so it is the one read with a visible counter of its own.

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | `num_spdif`: output slots (2 or 4) |
| 1 | u8 | Flags: bit 0 PDM enabled, bit 1 USB stream started |
| 2 | u16 | Sequence, increments on every read |
| 4 | 4 x 8 bytes | One record per output slot (S/PDIF or I2S). Records past `num_spdif` are zero |
| 36 | 8 bytes | PDM record. Zero while PDM is disabled |

Per-slot record (8 bytes).

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | 16 minus the ring fill in 48-frame buckets |
| 1 | u8 | Ring fill in 48-frame buckets, 0 to 16 (768 frames is full scale) |
| 2 | u8 | 1 while the slot's ring DMA runs |
| 3 | u8 | Fill percent, bucket resolution |
| 4 | u8 | Lowest fill percent since the last reset |
| 5 | u8 | Highest fill percent since the last reset |
| 6 | u16 | Fill in hundredths of a percent of 768 frames, sample resolution. May exceed 10000. Saturates at 65535. 0 when there is no valid reading |

During an output type switch a slot reads 16, 0, 0. The PDM record is six u8 fields followed by 2 reserved bytes. The fields are DMA fill percent, DMA minimum, DMA maximum, ring fill percent, ring minimum and ring maximum.

`SYS.STATS_RESET` takes a 1-byte mask. Bit 0 resets the watermarks. They are the per-slot and PDM minimum and maximum fill, the lowest pre-write fill, the worst input gap and the worst USB packet gap. Counters are never reset. Spec: `Features/buffer_statistics_spec.md`. Any other bit is `VC_ERR_VALUE`.

---

## 7. STORE (0x02)

The preset store, the startup policy, IO configuration persistence, factory reset, and the whole-state bulk transfer. Every write on this page is deferred to the main loop.

<!-- VC:BEGIN STORE -->

| Cmd | Name | Dir | wValue | GET bytes | SET bytes | Flags | Notes |
|---|---|---|---|---|---|---|---|
| 0x10 | `PRESET_SAVE` | S0 | preset slot | - | 0 | DEFERRED, FLASH, RESTART | Save the live state to a slot and make it active |
| 0x11 | `PRESET_LOAD` | S0 | preset slot | - | 0 | DEFERRED, FLASH, RESTART | Load a slot; an empty slot loads factory defaults |
| 0x12 | `PRESET_DELETE` | S0 | preset slot | - | 0 | DEFERRED, FLASH, RESTART | Erase a slot |
| 0x13 | `PRESET_NAME` | G, S | preset slot | 32 | 1-32 | DEFERRED, FLASH, RESTART | Slot name, NUL padded; a SET keeps at most 31 bytes |
| 0x14 | `PRESET_ACTIVE` | G | none | 1 | - |  | Active slot |
| 0x15 | `DIRECTORY` | G | none | 7 | - |  | Occupied mask u16, startup mode, default slot, last active, IO config mode, master volume mode |
| 0x16 | `STARTUP` | G, S | none | 2 | 2 | DEFERRED, FLASH, RESTART | Startup mode and default slot |
| 0x20 | `OUTPUT_CONFIG_MODE` | G, S | none | 1 | 1 | DEFERRED, FLASH, RESTART | IO config persistence: 0 independent, 1 with preset; other values store 0 |
| 0x21 | `OUTPUT_CONFIG_SAVE` | S0 | none | - | 0 | DEFERRED, FLASH, RESTART | Persist the live IO config to the device-wide block |
| 0x30 | `FACTORY_RESET` | S0 | none | - | 0 | DEFERRED, RESTART | Reset the live state to factory defaults; stored presets are kept and no flash is written |
| 0x40 | `ALL_PARAMS` | G, S | none | 6136 | bulk | BULK, DEFERRED, RESTART | WireBulkParams V32; a SET must be the whole struct |
| 0x41 | `ALL_PARAMS_CHUNK` | G, S | byte offset | var | bulk | USB_ONLY, BULK, DEFERRED, RESTART | WireBulkParams in pieces at a byte offset; offset 0 opens the session and SET chunks must follow in order |

<!-- VC:END STORE -->

### 7.1 Presets

There are 10 preset slots, 0 to 9. A preset is always active, and there is no "no preset" state. Loading a slot that has never been saved applies factory defaults.

- `STORE.PRESET_SAVE` saves the live state into the slot and makes it the active slot. It writes the slot body and the directory.
- `STORE.PRESET_LOAD` loads the slot into the live state under a muted, synchronized restart of every output, zeroes the delay lines, records the slot as last active, and sends notifications `PRESET_LOADED` (`0x04`) and `BULK_INVALIDATED` (`0x03`). Confirm with `STORE.PRESET_ACTIVE`.
- `STORE.PRESET_DELETE` erases the slot and clears its name. If the deleted slot is the active one, it stays selected and loads factory defaults the next time it is loaded.
- `STORE.PRESET_NAME` reads or sets a slot name (32 bytes, a SET keeps at most 31). Names live in the directory, so naming an empty slot is allowed.

Which parts of the state travel with a preset is set by two modes. `STORE.OUTPUT_CONFIG_MODE` covers the IO configuration (output types and pins, input pins, I2S and ADAT settings, the limiter). `ROUTE.MASTER_VOLUME_MODE` covers the master volume. In mode 0 (independent) the value is device-wide and stored in the directory, and preset loads leave it alone. In mode 1 (with preset) it is saved and loaded with each preset. A SET of either mode with a value above 1 stores 0.

`STORE.OUTPUT_CONFIG_SAVE` stores the live IO configuration in the device-wide directory block. It matters in mode 0. See `Features/user_presets_spec.md` and `Features/output_config_independent_load.md`.

### 7.2 Directory and startup

`STORE.DIRECTORY` (7 bytes).

| Offset | Type | Field |
|---|---|---|
| 0 | u16 | Occupied slots, bit n = slot n |
| 2 | u8 | Startup mode: 0 load the default slot, 1 load the last active slot |
| 3 | u8 | Default slot |
| 4 | u8 | Last active slot |
| 5 | u8 | `OUTPUT_CONFIG_MODE`: 0 independent, 1 with preset |
| 6 | u8 | `MASTER_VOLUME_MODE`: 0 independent, 1 with preset |

`STORE.STARTUP` reads and writes bytes 2 and 3 as `{mode, slot}`. A mode above 1 or a slot above 9 is `VC_ERR_VALUE`.

### 7.3 Factory reset

`STORE.FACTORY_RESET` resets the live state to firmware defaults under a muted, synchronized restart, and switches the input source to USB. It does not erase stored presets, slot names, the startup policy or any device-wide setting, and it keeps the active slot selected. In IO config mode 1 the IO configuration returns to firmware defaults. In mode 0 the stored device-wide IO configuration is applied again. The host receives one `BULK_INVALIDATED` notification with source 4 (factory). To make the reset state permanent, save it to a preset afterwards. To remove stored data, delete the slots.

### 7.4 Bulk parameters

`STORE.ALL_PARAMS` reads or replaces the whole live state as one `WireBulkParams` structure, and `STORE.ALL_PARAMS_CHUNK` does the same in pieces for hosts limited to 4 KB control transfers. Section 19 describes both.

---

## 8. INPUT (0x03)

Input source selection, the S/PDIF, I2S and ADAT receivers, LG Sound Sync and the user names of the sources. The I2S clock pins and modes, which input and output share, are on the `CLOCK` page.

<!-- VC:BEGIN INPUT -->

| Cmd | Name | Dir | wValue | GET bytes | SET bytes | Flags | Notes |
|---|---|---|---|---|---|---|---|
| 0x10 | `SOURCE` | G, S | none | 1 | 1 | DEFERRED, RESTART | Active input source; a SET switches source on the main loop |
| 0x11 | `STATUS` | G | none | 12 | - |  | Active source, active channels, measured S/PDIF rate in mHz, VCXO trim in ppm x100 |
| 0x12 | `SOURCE_NAME` | G, S | source ID | 32 | 1-32 | DEFERRED, FLASH, RESTART | User name of one input source; a leading NUL clears it |
| 0x20 | `SPDIF_RX_STATUS` | G | none | 16 | - |  | S/PDIF receiver status packet |
| 0x21 | `SPDIF_RX_CH_STATUS` | G | none | 24 | - |  | S/PDIF channel status bytes |
| 0x22 | `SPDIF_RX_PIN` | G, S | S/PDIF input | 1 | 1 | DEFERRED, RESTART | GPIO of one S/PDIF input; 0xFF restores its default |
| 0x24 | `SPDIF_RX_CONFIG` | G | none | 7 | - |  | Input count limit, block switch, block count, then one GPIO per input |
| 0x25 | `SPDIF_RX_PIN_AVAIL` | G | S/PDIF input | 24 | - |  | Pins one S/PDIF input could take: status, flags, generation, valid and free masks |
| 0x26 | `SPDIF_RX_COUNT` | G, S | none | 1 | 1 |  | S/PDIF inputs in the block, 1 to 4; inputs 1 to N hold their pins while the block is on |
| 0x27 | `SPDIF_RX_ENABLE` | G, S | none | 1 | 1 |  | S/PDIF input block switch, 0 or 1 |
| 0x30 | `I2S_RX_PIN` | G, S | I2S pair | 1 | 1 | DEFERRED, RESTART | Data GPIO of one I2S RX stereo pair; 0xFF restores its default |
| 0x31 | `I2S_RX_CHANNELS` | G, S | none | 1 | 1 | DEFERRED, RESTART | I2S input channel count, 2, 4, 6 or 8 |
| 0x32 | `I2S_RX_RATE` | G, S | none | 4 | 4 | DEFERRED, RESTART | Selected I2S and ADAT master rate in Hz: 44100, 48000 or 96000 |
| 0x33 | `I2S_RX_PIN_AVAIL` | G | I2S pair | 24 | - |  | Pins one I2S RX pair could take: status, flags, generation, valid and free masks |
| 0x34 | `I2S_RX_ENABLE` | G, S | none | 1 | 1 |  | I2S input block switch, 0 or 1; on holds the clock pair and counted data pins |
| 0x40 | `ADAT_RX_ENABLE` | G, S | none | 1 | 1 |  | ADAT input enable (build: `PICO_RP2350` only) |
| 0x41 | `ADAT_RX_PIN` | G, S | none | 1 | 1 | DEFERRED, RESTART | ADAT input GPIO; 0xFF clears it while the input is disabled (build: `PICO_RP2350` only) |
| 0x42 | `ADAT_RX_CLOCK_MODE` | G, S | none | 1 | 1 | DEFERRED, RESTART | ADAT input clock mode, 0 master or 1 slave (build: `PICO_RP2350` only) |
| 0x43 | `ADAT_RX_STATUS` | G | none | 20 | - |  | ADAT input status packet (build: `PICO_RP2350` only) |
| 0x44 | `ADAT_RX_PIN_AVAIL` | G | none | 24 | - |  | Pins the ADAT input could take: status, flags, generation, valid and free masks (build: `PICO_RP2350` only) |
| 0x50 | `LG_SOUND_SYNC_ENABLE` | G, S | none | 1 | 1 |  | LG Sound Sync decoding enable |
| 0x51 | `LG_SOUND_SYNC_STATUS` | G | none | 16 | - |  | LG Sound Sync status packet |

<!-- VC:END INPUT -->

### 8.1 Source selection and status

`INPUT.SOURCE` takes a source ID from section 4.6. The switch runs on the main loop under a full pipeline reset, and the GET reports the active source, not a pending one. A SET of the active source is a successful no-op.

| Refusal | Result |
|---|---|
| Value above 6 | `VC_ERR_VALUE` |
| ADAT (3) on RP2040 | `VC_ERR_UNSUPPORTED` |
| An optional S/PDIF input that is disabled, or ADAT while it is disabled or has no pin | `VC_ERR_STATE` |

`INPUT.STATUS` (12 bytes).

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Active source ID |
| 1 | u8 | Active input channel count |
| 2 | u16 | Reserved, 0 |
| 4 | u32 | Measured S/PDIF input rate, millihertz. 0 when nothing is measured |
| 8 | i32 | Clock trim the device applies to follow the input, ppm x 100 |

`INPUT.SOURCE_NAME` is the user name of one source (address = source ID 0 to 6), with the same rules as `SYS.DEVICE_NAME`. It is stored in the directory and announced by notification `INPUT_SOURCE_NAME` (`0x0F`). Spec: `Features/input_source_names_spec.md`.

### 8.2 S/PDIF receiver

There are four S/PDIF inputs on both platforms. Input 0 (source 1) is always enabled. Inputs 1 to 3 (sources 4 to 6) are optional, disabled by default, and hold no GPIO while disabled. Default pins are GPIO 5, 20, 21 and 22. Only the selected input claims its pin in hardware.

`INPUT.SPDIF_RX_STATUS` (16 bytes).

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | State: 0 inactive, 1 acquiring, 2 locked, 3 relocking |
| 1 | u8 | Active source ID |
| 2 | u8 | Locks since activation |
| 3 | u8 | Lock losses since activation |
| 4 | u32 | Detected sample rate, Hz. 0 unless locked |
| 8 | u32 | Parity errors, cumulative |
| 12 | u16 | Receive FIFO fill, percent |
| 14 | u16 | Reserved |

`INPUT.SPDIF_RX_CH_STATUS` returns the 24 raw IEC 60958 channel status bytes of the locked stream.

`INPUT.SPDIF_RX_CONFIG` (6 bytes) is the input count (4), an enable mask over all inputs (bit i is input i, bit 0 always set), then the GPIO of inputs 0 to 3.

`INPUT.SPDIF_RX_PIN` sets the GPIO of one input (address 0 to 3). `0xFF` restores that input's default. The GPIO must be valid for S/PDIF input (`VC_ERR_PIN_INVALID`). An enabled input also checks for conflicts (`VC_ERR_PIN_IN_USE`). A disabled input stores the pin as a preference, and the enable checks it. If the input is the active source, the receiver moves to the new pin on the main loop with a short silence.

`INPUT.SPDIF_RX_ENABLE` addresses inputs 1 to 3 only. Address 0 is `VC_ERR_ADDRESS` on both directions, because input 0 is always on. Any non-zero payload enables.

| Refusal | Result |
|---|---|
| Enable with an invalid stored pin | `VC_ERR_PIN_INVALID` |
| Enable with a stored pin another function holds | `VC_ERR_PIN_IN_USE` |
| Disable while the input is the active source, or the target of a pending switch | `VC_ERR_STATE` |

Spec: `Features/SPDIF_input_spec.md`.

### 8.3 I2S receiver

The device is the clock master for I2S input unless `CLOCK.I2S_CLOCK_MODE` selects slave mode. RP2350 takes up to four stereo pairs (8 channels). RP2040 takes one pair.

`INPUT.I2S_RX_PIN` sets the data GPIO of one stereo pair (address 0 to 3 on RP2350, 0 on RP2040). `0xFF` restores the pair's default, which is GPIO 1 + pair. Pin results are `VC_ERR_PIN_INVALID` and `VC_ERR_PIN_IN_USE`. The BCK and LRCLK pair counts as claimed here even while no I2S is running. While I2S is the active input, a pin change restarts the input on the main loop.

`INPUT.I2S_RX_CHANNELS` is the channel count, which is 2, 4, 6 or 8. Another value is `VC_ERR_VALUE`, and more pairs than the platform has is `VC_ERR_UNSUPPORTED` (RP2040 accepts only 2). Raising the count re-checks the data pin of every newly active pair and refuses with that pin's result. A change while I2S is the active input restarts it and sends notification `INPUT_FORMAT` (`0x05`).

`INPUT.I2S_RX_RATE` is the rate in Hz the device uses when it is the clock master for I2S or ADAT input. It is 44100, 48000 or 96000. Another value is `VC_ERR_VALUE`. The value is always stored. The device rate changes only while that rate authority is active (I2S input in master mode, or ADAT input in master mode). The live device rate is in `SYS.SYSTEM_STATUS`.

See `Features/i2s_multi_input.md` and `Features/i2s_slave_input_spec.md`.

### 8.4 ADAT receiver (RP2350)

The ADAT input is an 8-channel lightpipe receiver. These commands exist only on RP2350.

- Set the pin first with `INPUT.ADAT_RX_PIN`, then enable with `INPUT.ADAT_RX_ENABLE`, then select source 3 with `INPUT.SOURCE`. A disabled input holds no pin. Enabling claims the pin but starts no hardware until the source is selected.
- `INPUT.ADAT_RX_PIN` has no default pin, so `0xFF` clears the pin to "unset", which is the boot state. Clearing is allowed only while the input is disabled (`VC_ERR_STATE` otherwise). The pin may be the ADAT output's pin, for a loopback test. While ADAT is the active source, a pin change restarts the receiver under mute.
- `INPUT.ADAT_RX_ENABLE` takes 0 or 1 (`VC_ERR_VALUE` otherwise). Enable with no valid pin is `VC_ERR_PIN_INVALID`, with a pin in use `VC_ERR_PIN_IN_USE`. Disable while ADAT is the active source or the target of a pending switch is `VC_ERR_STATE`.
- `INPUT.ADAT_RX_CLOCK_MODE` is 0 master (the device clock runs from `INPUT.I2S_RX_RATE`) or 1 slave (the device follows the stream). Another value is `VC_ERR_VALUE`. The change is applied by the main loop, under a muted receiver restart when ADAT is live. The GET reports the live mode, so a pending change shows only once applied.

`INPUT.ADAT_RX_STATUS` (20 bytes).

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | State: 0 inactive, 1 acquiring, 2 syncing, 3 locked, 4 relocking |
| 1 | u8 | Live clock mode: 0 master, 1 slave |
| 2 | u8 | Configured enable |
| 3 | u8 | Configured pin, `0xFF` unset |
| 4 | u8 | `rate_ok`: 0 while master mode is parked because the device rate is above 48 kHz. Always 1 in slave mode |
| 5 | u8 | Locks (saturates at 255, zeroed when the receiver starts) |
| 6 | u8 | Lock losses (saturates at 255, zeroed when the receiver starts) |
| 7 | u8 | Losses caused by header verification failure |
| 8 | u16 | Header mismatches, cumulative, wraps |
| 10 | u16 | Reserved |
| 12 | u32 | Detected rate while locked, Hz (master mode: the device rate), else 0 |
| 16 | u32 | Slave mode: raw measured wire rate, Hz. 0 in master mode |

Lock-state changes send notification `ADAT_INPUT_STATE` (`0x0B`). Spec: `Features/adat_input_spec.md`.

### 8.5 LG Sound Sync

`INPUT.LG_SOUND_SYNC_ENABLE` turns LG Sound Sync decoding on or off (non-zero enables). When it is on and an LG TV sends its volume and mute, the device follows them. It is stored per preset. Spec: `Features/lg_sound_sync_spec.md`.

`INPUT.LG_SOUND_SYNC_STATUS` (16 bytes).

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Enable |
| 1 | u8 | 1 while the LG signature is detected |
| 2 | u8 | Last decoded volume, 0 to 100. `0xFF` means never decoded since boot |
| 3 | u8 | Last decoded mute, 0 or 1 |
| 4 | u8[12] | Reserved |

---

## 9. OUTPUT (0x04)

The physical output bus. This page holds slot types and pins, output latency, the ADAT transmitter and the DAC hardware mute line.

<!-- VC:BEGIN OUTPUT -->

| Cmd | Name | Dir | wValue | GET bytes | SET bytes | Flags | Notes |
|---|---|---|---|---|---|---|---|
| 0x10 | `TYPE` | G, S | output slot | 1 | 1 | DEFERRED, RESTART | Type of one output slot, 0 S/PDIF or 1 I2S |
| 0x11 | `PIN` | G, S | pin output | 1 | 1 | DEFERRED, RESTART | GPIO of one pin output; 0xFF restores its default |
| 0x12 | `PIN_AVAIL` | G | pin output | 24 | - |  | Pins one pin output could take: status, flags, generation, valid and free masks |
| 0x20 | `LATENCY` | G, S | none | 1 | 1 | DEFERRED, FLASH, RESTART | Requested output latency option ID |
| 0x21 | `LATENCY_STATUS` | G | none | 12 | - |  | Requested and effective latency, ring target, batch, pending flag, option count |
| 0x22 | `LATENCY_OPTIONS` | G | none | var | - |  | Every latency option, 8 bytes each, with its effective latency now |
| 0x30 | `ADAT_TX_ENABLE` | G, S | none | 1 | 1 | DEFERRED, RESTART | ADAT output enable (build: `PICO_RP2350` only) |
| 0x31 | `ADAT_TX_PIN` | G, S | none | 1 | 1 | DEFERRED, RESTART | ADAT output GPIO; 0xFF restores the default (build: `PICO_RP2350` only) |
| 0x32 | `ADAT_TX_STATUS` | G | none | 8 | - |  | ADAT output status packet (build: `PICO_RP2350` only) |
| 0x33 | `ADAT_TX_PIN_AVAIL` | G | none | 24 | - |  | Pins the ADAT output could take: status, flags, generation, valid and free masks (build: `PICO_RP2350` only) |
| 0x40 | `DAC_MUTE_CONFIG` | G, S | none | 16 | 16 | DEFERRED, FLASH, RESTART | DAC hardware mute pin, polarity and timing |
| 0x41 | `DAC_MUTE_TEST` | S0 | none | - | 0 | DEFERRED, RESTART | Pulse the DAC hardware mute pin for about one second |
| 0x42 | `DAC_MUTE_PIN_AVAIL` | G | none | 24 | - |  | Pins the DAC hardware mute could take: status, flags, generation, valid and free masks |

<!-- VC:END OUTPUT -->

### 9.1 Slot type and pins

`OUTPUT.TYPE` sets one slot (address 0 to 1 on RP2040, 0 to 3 on RP2350) to 0 S/PDIF or 1 I2S. Another value is `VC_ERR_VALUE`. The main loop tears the slot's output down and builds the new one inside a synchronized restart of every slot, so all outputs stay sample-aligned. The GET reports the live type, which changes once the switch has run. I2S slots share one BCK and LRCLK pair (`CLOCK` page).

`OUTPUT.PIN` moves the data pin of one pin output (section 4.3). `0xFF` restores the default. A slot pin change is applied by the main loop, which mutes and restarts every slot together. The PDM sub pin moves at once, and only while the PDM output is not running (`VC_ERR_STATE` otherwise). Disable the PDM output first.

| Refusal | Result |
|---|---|
| GPIO not valid for the output function | `VC_ERR_PIN_INVALID` |
| GPIO held by another function | `VC_ERR_PIN_IN_USE` |
| PDM pin while PDM runs | `VC_ERR_STATE` |

### 9.2 Output latency

The output latency sets how far ahead of the output the rings are filled. It is device-wide and stored in the directory, not per preset.

| ID | Latency | Note |
|---|---|---|
| 1 | 0.67 ms | Stream inputs only |
| 2 | 1 ms | Stream inputs only |
| 3 | 2 ms | |
| 4 | 3 ms | |
| 5 | 4 ms | Default |
| 6 | 6 ms | |
| 7 | 8 ms | |

A stream-only option requested while the source is USB runs as 2 ms. `OUTPUT.LATENCY` takes an option ID (`VC_ERR_VALUE` for an unknown one). The main loop writes it to flash and restarts every output together. The GET returns the queued ID while a change is pending and the stored ID otherwise, so a SET reads back at once. Re-sending the stored ID with nothing pending is a no-op. No notification is sent.

`OUTPUT.LATENCY_STATUS` (12 bytes).

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Requested option ID (stored) |
| 1 | u8 | Effective option ID (in force) |
| 2 | u16 | Effective latency, us (ring fill target at the current rate) |
| 4 | u16 | Live ring fill target, frames |
| 6 | u16 | Stream input batch at the current rate, frames |
| 8 | u8 | 1 until a SET has fully applied |
| 9 | u8 | Number of options in `OUTPUT.LATENCY_OPTIONS` |
| 10 | u16 | Reserved |

`OUTPUT.LATENCY_OPTIONS` returns 8 bytes per option, `option_count` entries (56 bytes today).

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Option ID |
| 1 | u8 | Flags: bit 0 stream inputs only, bit 7 (`0x80`) runs as itself on the active source now |
| 2 | u16 | Nominal latency, us |
| 4 | u16 | Nominal stream input batch, us |
| 6 | u16 | What selecting it gives now, us, with the source and rate applied |

### 9.3 ADAT transmitter (RP2350)

The ADAT output mirrors outputs 1 to 8 (post-gain) as one 8-channel lightpipe stream. It runs at 44.1 and 48 kHz and suspends itself above 48 kHz. These commands exist only on RP2350.

- `OUTPUT.ADAT_TX_ENABLE` takes 0 or 1 (`VC_ERR_VALUE` otherwise). Enable checks the configured pin (`VC_ERR_PIN_INVALID`, `VC_ERR_PIN_IN_USE`). Disable always succeeds.
- `OUTPUT.ADAT_TX_PIN` takes a GPIO, `0xFF` restores the default (GPIO 12). A move is allowed while enabled.
- Both apply on the main loop inside a muted, synchronized output restart. Both persist with the IO configuration.

`OUTPUT.ADAT_TX_STATUS` (8 bytes).

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Configured enable |
| 1 | u8 | 1 while the stream runs |
| 2 | u8 | Configured pin |
| 3 | u8 | 1 when the device rate is 44.1 or 48 kHz |
| 4 | u16 | Stream restarts since boot |
| 6 | u16 | Emergency local resyncs since boot (should stay 0) |

Enabled, not active and `rate_ok` 0 is the suspended state above 48 kHz. Stream starts and stops send notification `ADAT_STATE` (`0x08`). Spec: `Features/adat_output_spec.md`.

### 9.4 DAC hardware mute

A GPIO that mutes an external DAC around every output clock stop, such as a flash write or an output restart. `OUTPUT.DAC_MUTE_CONFIG` (16 bytes).

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Enable, 0 or 1 |
| 1 | u8 | 1 asserts the pin low to mute, 0 high |
| 2 | u8 | GPIO, `0xFF` none |
| 3 | u8 | Reserved |
| 4 | u16 | Hold before the clocks stop, ms, 1 to 500 |
| 6 | u16 | Hold after the clocks restart, ms, 0 to 500 |
| 8 | u8[8] | Reserved |

With enable set, a polarity above 1 or a hold time out of range is refused at once with `VC_ERR_VALUE`. The pin's validity and conflicts are checked by the main loop, which also claims the pin and writes the configuration to the directory. A configuration the main loop refuses is dropped, so read it back to confirm. The setting is device-wide.

`OUTPUT.DAC_MUTE_TEST` pulses the mute pin for about one second so an installer can confirm the wiring by ear. While the feature is disabled it is `VC_ERR_STATE`. Spec: `Features/dac_hardware_mute_spec.md`.

---

## 10. ROUTE (0x05)

The matrix mixer, the per-output controls, delays, channel names, input preamps and the two volume stages. Every SET here applies at once in the handler, except the two flash commands.

<!-- VC:BEGIN ROUTE -->

| Cmd | Name | Dir | wValue | GET bytes | SET bytes | Flags | Notes |
|---|---|---|---|---|---|---|---|
| 0x10 | `MATRIX_ROUTE` | G, S | `(input << 8) \| output` | 8 | 8 |  | One crosspoint: u8 enabled, u8 phase invert, u16 reserved, f32 gain dB |
| 0x20 | `OUTPUT_ENABLE` | G, S | output | 1 | 1 |  | Output enable; an enable that collides with the core 1 mode is VC_ERR_STATE |
| 0x21 | `OUTPUT_GAIN` | G, S | output | 4 | 4 |  | Output gain, f32 dB |
| 0x22 | `OUTPUT_MUTE` | G, S | output | 1 | 1 |  | Output mute, byte stored as sent |
| 0x23 | `OUTPUT_DELAY` | G, S | output | 4 | 4 |  | Output delay, f32 ms, negative clamps to 0 |
| 0x24 | `OUTPUT_ENABLE_CONFLICT` | G | output | 1 | - |  | 1 when enabling this output would collide with the core 1 mode |
| 0x30 | `CHANNEL_DELAY` | G, S | channel | 4 | 4 |  | Channel delay, f32 ms, negative clamps to 0 |
| 0x31 | `CHANNEL_NAME` | G, S | channel | 32 | 1-32 |  | Channel name, 32 bytes NUL padded; SET stores at most 31 |
| 0x40 | `PREAMP` | G, S | input channel | 4 | 4 |  | Input channel preamp, f32 dB, non-finite is VC_ERR_VALUE |
| 0x50 | `MASTER_VOLUME` | G, S | none | 4 | 4 |  | Master volume ceiling, f32 dB, clamped to -128..0 (-128 mutes) |
| 0x51 | `MASTER_VOLUME_MODE` | G, S | none | 1 | 1 | DEFERRED, FLASH, RESTART | 0 independent, 1 per preset; other values store 0 |
| 0x52 | `MASTER_VOLUME_SAVE` | S0 | none | - | 0 | DEFERRED, FLASH, RESTART | Save the live master volume as the independent value |
| 0x53 | `MASTER_VOLUME_SAVED` | G | none | 4 | - |  | Saved independent master volume, f32 dB |
| 0x60 | `USER_VOLUME` | G, S | none | 4 | 4 |  | User volume, f32 dB, clamped to -60..0, 1/256 dB steps |
| 0x61 | `USER_MUTE` | G, S | none | 1 | 1 |  | User mute, non-zero mutes |

<!-- VC:END ROUTE -->

### 10.1 Matrix

`ROUTE.MATRIX_ROUTE` is one crosspoint, addressed `(input << 8) | output`.

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Enabled, non-zero routes |
| 1 | u8 | Phase invert, non-zero inverts |
| 2 | u16 | Reserved, 0 |
| 4 | f32 | Gain, dB. A non-finite value is `VC_ERR_VALUE` |

On RP2350, while the stereo upmixer runs, matrix rows 2, 3 and 4 carry its derived centre, left surround and right surround signals instead of inputs 3 to 5 (section 13.5). Spec: `Features/matrixmixer_spec.md`.

### 10.2 Per-output controls

Addressed by output index.

- `ROUTE.OUTPUT_ENABLE`: non-zero enables. Core 1 runs either the PDM output or the EQ of outputs 2 to 7 (RP2350) or 2 to 3 (RP2040). Enabling an output that would need both is `VC_ERR_STATE`. `ROUTE.OUTPUT_ENABLE_CONFLICT` reads 1 when that would happen. A change of Core 1 mode takes effect at once.
- `ROUTE.OUTPUT_GAIN` is in dB and takes any finite value.
- `ROUTE.OUTPUT_MUTE`: non-zero mutes. The byte is stored as sent.
- `ROUTE.OUTPUT_DELAY` is in ms. Negative values clamp to 0.

### 10.3 Delays

There is one delay per channel. `ROUTE.CHANNEL_DELAY` addresses it by channel index and `ROUTE.OUTPUT_DELAY` by output index. Only output channels are delayed. A value stored on an input channel reads back but has no effect. The delay line holds 1023 samples on RP2040 and 2047 on RP2350, about 21 ms and 42 ms at 48 kHz and half that at 96 kHz. Longer values are stored but capped when applied. The PDM sub carries a fixed alignment offset, so it reaches the cap slightly earlier.

`ROUTE.OUTPUT_DELAY` writes both the output's record and its channel delay. `ROUTE.CHANNEL_DELAY` on an output channel writes the channel delay only, so `ROUTE.OUTPUT_DELAY` then still reads the old value. Set output delays through `ROUTE.OUTPUT_DELAY`.

### 10.4 Names

`ROUTE.CHANNEL_NAME` is a 32-byte name per channel, stored with the preset. A SET of 1 to 32 bytes keeps at most 31.

### 10.5 Preamp and volume

`ROUTE.PREAMP` is the gain of one input channel in dB, applied before EQ. It is not clamped. A non-finite value is `VC_ERR_VALUE`.

`ROUTE.MASTER_VOLUME` is a device-side ceiling on the final output gain, -128 to 0 dB, clamped. -128 is a mute sentinel (true silence). The fresh-device default is -20 dB. It does not affect loudness compensation, the leveller, EQ or crossfeed. Whether it travels with presets is set by `ROUTE.MASTER_VOLUME_MODE` (section 7.1). `ROUTE.MASTER_VOLUME_SAVE` stores the live value as the device-wide value, and `ROUTE.MASTER_VOLUME_SAVED` reads that stored value, which differs from the live one until saved. A master volume change also sends the legacy 8-byte notification `MASTER_VOLUME` (`0x01`) in addition to `PARAM_CHANGED`.

`ROUTE.USER_VOLUME` is the same volume the host's UAC1 volume control drives, so it round-trips with the operating system's slider. It is clamped to -60 to 0 dB and stored in 1/256 dB steps. Loudness compensation follows it. `ROUTE.USER_MUTE` is a vendor mute (non-zero mutes), separate from the host's UAC1 mute control.

See `Features/master_volume_spec.md` and `Features/per_channel_preamp_spec.md`.

---

## 11. FILTER (0x06)

Parametric EQ and crossover bands for every channel, and the master EQ bypass.

<!-- VC:BEGIN FILTER -->

| Cmd | Name | Dir | wValue | GET bytes | SET bytes | Flags | Notes |
|---|---|---|---|---|---|---|---|
| 0x10 | `BAND` | G, S | `(channel << 8) \| band` | 16 | 16 | DEFERRED | One PEQ or crossover band: u8 type, u8 bypass, u16 qp_x512, f32 freq, f32 Q, f32 gain dB |
| 0x11 | `BAND_BYPASS` | G, S | `(channel << 8) \| band` | 1 | 1 | DEFERRED | Bypass one band, exactly 1 bypasses |
| 0x12 | `EQ_BYPASS` | G, S | none | 1 | 1 |  | Master EQ bypass, non-zero bypasses |

<!-- VC:END FILTER -->

### 11.1 Bands

`FILTER.BAND` is one band, addressed `(channel << 8) | band` (section 4.4).

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Filter type (section 4.7) |
| 1 | u8 | Bypass: exactly 1 bypasses. Reads back as 0 or 1 |
| 2 | u16 | Linkwitz Transform Qp x 512 (0 selects 0.707). Ignored and read as 0 for any band other than an EQ band |
| 4 | f32 | Frequency, Hz |
| 8 | f32 | Q |
| 12 | f32 | Gain, dB (for the Linkwitz Transform, fp in Hz) |

A band in the reserved gap, or a crossover band on an input channel, is `VC_ERR_ADDRESS` in both directions. A non-finite frequency, Q or gain is `VC_ERR_VALUE`. Always send `qp_x512`. It is stored for every EQ band, and a value of 0 selects the default.

The main loop recomputes the coefficients and sends `PARAM_CHANGED` at the band's bulk offset. There is one pending slot. A second `FILTER.BAND` or `FILTER.BAND_BYPASS` before the main loop has taken the first is `VC_ERR_BUSY`. Nothing is lost. Wait a millisecond or two and resend. The GET returns the stored band, which includes a queued write only once it is applied.

`FILTER.BAND_BYPASS` reads or sets byte 1 of a band without sending the rest, with the same addressing and the same pending slot.

`FILTER.EQ_BYPASS` bypasses every EQ band of every channel at once (non-zero bypasses). It applies at once. See `Features/peq_filters.md` and `Features/band_bypass_spec.md`.

---

## 12. DYN (0x07)

Dynamics. This page holds the per-output brickwall limiter, the volume leveller and loudness compensation. All SETs apply at once in the handler. Where coefficients must be rebuilt, the main loop does it a few milliseconds later and switches to them without a glitch.

<!-- VC:BEGIN DYN -->

| Cmd | Name | Dir | wValue | GET bytes | SET bytes | Flags | Notes |
|---|---|---|---|---|---|---|---|
| 0x10 | `LIMITER_ENABLE` | G, S | output, 0xFF = all (SET) | 4 | 4 |  | Limiter enable as f32 (non-zero enables); linked outputs follow |
| 0x11 | `LIMITER_THRESHOLD` | G, S | output, 0xFF = all (SET) | 4 | 4 |  | Limiter threshold, f32 dB, clamped to -30..0 |
| 0x12 | `LIMITER_RELEASE` | G, S | output, 0xFF = all (SET) | 4 | 4 |  | Limiter release, f32 ms, clamped to 10..1000 |
| 0x13 | `LIMITER_LINK_GROUP` | G, S | output, 0xFF = all (SET) | 4 | 4 |  | Limiter link group as f32, rounded and clamped to 0..4 (0 unlinked) |
| 0x18 | `LIMITER_METERS` | G | none | var | - |  | Gain reduction per output, u16 in 0.01 dB |
| 0x19 | `LIMITER_STATUS` | G | none | 4 | - |  | u8 engaged, u8 lookahead samples, u8 block samples, u8 output count |
| 0x20 | `LEVELLER_ENABLE` | G, S | none | 1 | 1 |  | Leveller enable, non-zero enables |
| 0x21 | `LEVELLER_AMOUNT` | G, S | none | 4 | 4 |  | Leveller amount, f32 percent, clamped |
| 0x22 | `LEVELLER_SPEED` | G, S | none | 1 | 1 |  | Leveller speed index; out of range is VC_ERR_VALUE |
| 0x23 | `LEVELLER_MAX_GAIN` | G, S | none | 4 | 4 |  | Leveller maximum gain, f32 dB, clamped |
| 0x24 | `LEVELLER_LOOKAHEAD` | G, S | none | 1 | 1 |  | Leveller lookahead, non-zero enables |
| 0x25 | `LEVELLER_GATE` | G, S | none | 4 | 4 |  | Leveller gate threshold, f32 dB, clamped |
| 0x26 | `LEVELLER_MASKS` | G, S | none | 2 | 2 |  | Leveller u8 detector mask, u8 apply mask |
| 0x30 | `LOUDNESS_ENABLE` | G, S | none | 1 | 1 |  | Loudness compensation enable, non-zero enables |
| 0x31 | `LOUDNESS_REF` | G, S | none | 4 | 4 |  | Loudness reference SPL, f32, clamped |
| 0x32 | `LOUDNESS_INTENSITY` | G, S | none | 4 | 4 |  | Loudness intensity, f32 percent, clamped |
| 0x33 | `LOUDNESS_MASK` | G, S | none | 2 | 2 |  | Loudness output mask, u16 |

<!-- VC:END DYN -->

### 12.1 Limiter

Each output has its own lookahead limiter. The four parameters are float32 values, addressed by output, and `0xFF` on a SET writes every output.

| Command | Range | Default | Notes |
|---|---|---|---|
| `DYN.LIMITER_ENABLE` | non-zero enables | 0 | Reads back 0.0 or 1.0 |
| `DYN.LIMITER_THRESHOLD` | -30 to 0 dB, clamped | -1 | |
| `DYN.LIMITER_RELEASE` | 10 to 1000 ms, clamped | 100 | |
| `DYN.LIMITER_LINK_GROUP` | 0 to 4, rounded and clamped | 0 | 0 is unlinked |

A NaN is `VC_ERR_VALUE`. An infinity clamps to the end of the range. A SET on a linked output writes every member of its group. Joining a group adopts the settings of the group's lowest-numbered member. Each SET sends `PARAM_CHANGED` for every output it wrote. The lookahead delay engages or releases only during silence.

`DYN.LIMITER_METERS` returns one u16 per output (10 bytes on RP2040, 18 on RP2350). Each is the gain reduction in 0.01 dB, 0 for none and 12000 at the floor.

`DYN.LIMITER_STATUS` (4 bytes) is `engaged` (1 while the lookahead delay is in the signal path), the lookahead in samples (32), the block size in samples (16) and the output count.

The limiter settings persist with the IO configuration (section 7.1). Spec: `Features/output_limiter_spec.md`.

### 12.2 Leveller

The leveller is a slow multichannel compressor that evens out loudness between programmes.

| Command | Payload | Range | Notes |
|---|---|---|---|
| `DYN.LEVELLER_ENABLE` | u8 | non-zero enables | Resets the leveller's state |
| `DYN.LEVELLER_AMOUNT` | f32 | 0 to 100 percent, clamped | |
| `DYN.LEVELLER_SPEED` | u8 | 0 slow, 1 medium, 2 fast | 3 or more is `VC_ERR_VALUE` |
| `DYN.LEVELLER_MAX_GAIN` | f32 | 0 to 35 dB, clamped | |
| `DYN.LEVELLER_LOOKAHEAD` | u8 | non-zero enables | Clears the delay buffer |
| `DYN.LEVELLER_GATE` | f32 | -96 to 0 dBFS, clamped | Silence gate, keeps the leveller from pumping the noise floor |
| `DYN.LEVELLER_MASKS` | u8, u8 | detector mask, apply mask | Bit k is input channel k. Stored as sent. Default `0xFF`, `0xFF` |

Both masks are combined with the set of live inputs, so bits for inputs that carry no audio do nothing. A mask change is glitch-free. Spec: `Features/volume_leveller_spec.md`.

### 12.3 Loudness compensation

Loudness compensation boosts low and high frequencies as the user volume falls, so the balance sounds the same at low levels. It runs per output channel, after the output gain.

| Command | Payload | Range |
|---|---|---|
| `DYN.LOUDNESS_ENABLE` | u8 | non-zero enables |
| `DYN.LOUDNESS_REF` | f32 | Reference level, 40 to 100 dB SPL, clamped |
| `DYN.LOUDNESS_INTENSITY` | f32 | 0 to 200 percent, clamped |
| `DYN.LOUDNESS_MASK` | u16 | Bit k is output k, PDM included. Default `0xFFFF`. Bits past the output count are stored and ignored |

The compensation follows `ROUTE.USER_VOLUME` (or LG Sound Sync, when it drives the volume). Spec: `Features/loudness_compensation_spec.md`.

---

## 13. FX (0x08)

Effects. This page holds headphone crossfeed, the subharmonic synthesizer, psychoacoustic bass, the tube preamp and, on RP2350, the stereo upmixer. No command on this page takes an address, so `wValue` is always 0.

These rules apply to the whole page.

- Every SET updates the live configuration at once. A command flagged `DEFERRED` also asks the main loop to rebuild that effect's coefficients, which it publishes glitch-free within a few milliseconds. Masks, the subharmonic pair link and solo are read live by the audio path and need no rebuild.
- Floats out of range are clamped, and the GET returns the clamped value. A NaN or infinite float is `VC_ERR_VALUE`, except on the tube commands (NaN is `VC_ERR_VALUE`, infinity clamps).
- Every SET sends `PARAM_CHANGED` at the field's bulk offset, except `FX.SUBHARM_SOLO`.
- Nothing here writes flash. The settings are part of the preset.

Output masks are u16, bit k = output k (5 outputs on RP2040, 9 on RP2350).

<!-- VC:BEGIN FX -->

| Cmd | Name | Dir | wValue | GET bytes | SET bytes | Flags | Notes |
|---|---|---|---|---|---|---|---|
| 0x10 | `CROSSFEED_ENABLE` | G, S | none | 1 | 1 | DEFERRED | Crossfeed on or off (u8 bool) |
| 0x11 | `CROSSFEED_PRESET` | G, S | none | 1 | 1 | DEFERRED | Crossfeed preset (u8, 0 to CROSSFEED_PRESET_CUSTOM) |
| 0x12 | `CROSSFEED_FREQ` | G, S | none | 4 | 4 | DEFERRED | Custom preset cutoff in Hz (f32, clamped) |
| 0x13 | `CROSSFEED_FEED` | G, S | none | 4 | 4 | DEFERRED | Custom preset feed level in dB (f32, clamped) |
| 0x14 | `CROSSFEED_ITD` | G, S | none | 1 | 1 | DEFERRED | Interaural time delay on or off (u8 bool) |
| 0x15 | `CROSSFEED_OUTPUTS` | G, S | none | 1 | 1 |  | Output pairs crossfeed applies to (u8 mask, masked to the slot count) |
| 0x20 | `SUBHARM_ENABLE` | G, S | none | 1 | 1 | DEFERRED | Subharmonic synth on or off (u8 bool) |
| 0x21 | `SUBHARM_LOW` | G, S | none | 4 | 4 | DEFERRED | 24-36 Hz band level in dB (f32, clamped) |
| 0x22 | `SUBHARM_HIGH` | G, S | none | 4 | 4 | DEFERRED | 36-56 Hz band level in dB (f32, clamped) |
| 0x23 | `SUBHARM_TOP` | G, S | none | 4 | 4 | DEFERRED | 56-80 Hz band level in dB (f32, clamped) |
| 0x24 | `SUBHARM_BOOST` | G, S | none | 4 | 4 | DEFERRED | LF boost in dB (f32, clamped) |
| 0x25 | `SUBHARM_MASK` | G, S | none | 2 | 2 |  | Outputs the synth feeds (u16 mask) |
| 0x26 | `SUBHARM_SELECT` | G, S | none | 1 | 1 | DEFERRED | Selectivity mode (u8, clamped to SUBHARM_SELECT_MODE_MAX) |
| 0x27 | `SUBHARM_DEPTH` | G, S | none | 4 | 4 | DEFERRED | Selectivity depth in percent (f32, clamped) |
| 0x28 | `SUBHARM_HOLD` | G, S | none | 4 | 4 | DEFERRED | Selectivity hold in ms (f32, clamped) |
| 0x29 | `SUBHARM_CEILING` | G, S | none | 4 | 4 | DEFERRED | Output ceiling in dBFS (f32, clamped) |
| 0x2A | `SUBHARM_LINK` | G, S | none | 1 | 1 |  | Link output pairs (u8 bool) |
| 0x2B | `SUBHARM_SOLO` | G, S | none | 1 | 1 |  | Monitor the synth alone (u8 bool, never persisted) |
| 0x2E | `SUBHARM_HEADROOM` | G | none | 4 | - |  | Worst-case gain of the current settings in dB (f32) |
| 0x2F | `SUBHARM_METER` | G | none | var | - |  | Synth output level per output (u16 each, 0 to 32767) |
| 0x40 | `PSYBASS_ENABLE` | G, S | none | 1 | 1 | DEFERRED | Psychoacoustic bass on or off (u8 bool) |
| 0x41 | `PSYBASS_CUTOFF` | G, S | none | 4 | 4 | DEFERRED | Cutoff in Hz (f32, clamped) |
| 0x42 | `PSYBASS_HARMONICS` | G, S | none | 4 | 4 | DEFERRED | Harmonic mix level in dB (f32, clamped) |
| 0x43 | `PSYBASS_DRIVE` | G, S | none | 4 | 4 | DEFERRED | Odd-path drive in dB (f32, clamped) |
| 0x44 | `PSYBASS_CHARACTER` | G, S | none | 4 | 4 | DEFERRED | Even to odd character in percent (f32, clamped) |
| 0x45 | `PSYBASS_ORIGINAL` | G, S | none | 4 | 4 | DEFERRED | Original low-band level in dB (f32, clamped) |
| 0x46 | `PSYBASS_MASK` | G, S | none | 2 | 2 |  | Outputs psychoacoustic bass applies to (u16 mask) |
| 0x60 | `TUBE_ENABLE` | G, S | none | 4 | 4 | DEFERRED | Tube preamp on or off (f32) |
| 0x61 | `TUBE_OUTPUT_MASK` | G, S | none | 4 | 4 |  | Outputs the tube applies to (f32, rounded to u16) |
| 0x62 | `TUBE_TYPE` | G, S | none | 4 | 4 | DEFERRED | Tube type (f32, rounded and clamped); a stock type also rewrites its row |
| 0x63 | `TUBE_DRIVE` | G, S | none | 4 | 4 | DEFERRED | Drive in dB (f32, clamped) |
| 0x64 | `TUBE_BIAS` | G, S | none | 4 | 4 | DEFERRED | Bias in percent (f32, clamped) |
| 0x65 | `TUBE_ASYM` | G, S | none | 4 | 4 | DEFERRED | Asymmetry in dB (f32, clamped) |
| 0x66 | `TUBE_HARDNESS` | G, S | none | 4 | 4 | DEFERRED | Hardness in percent (f32, clamped) |
| 0x67 | `TUBE_SAG` | G, S | none | 4 | 4 | DEFERRED | Sag in percent (f32, clamped) |
| 0x68 | `TUBE_RECTIFIER` | G, S | none | 4 | 4 | DEFERRED | Rectifier type (f32, rounded and clamped) |
| 0x69 | `TUBE_XFMR_ENABLE` | G, S | none | 4 | 4 | DEFERRED | Output transformer on or off (f32) |
| 0x6A | `TUBE_XFMR_DAMPING` | G, S | none | 4 | 4 | DEFERRED | Transformer damping (f32, clamped) |
| 0x6B | `TUBE_XFMR_RES` | G, S | none | 4 | 4 | DEFERRED | Transformer resonance in Hz (f32, clamped) |
| 0x6C | `TUBE_MIX` | G, S | none | 4 | 4 | DEFERRED | Wet mix in percent (f32, clamped) |
| 0x6D | `TUBE_TRIM` | G, S | none | 4 | 4 | DEFERRED | Output trim in dB (f32, clamped) |
| 0x80 | `UPMIX_CONFIG` | G, S | none | 44 | 44 | DEFERRED | Whole upmixer config (UpmixConfigPacket) (build: `PICO_RP2350` only) |
| 0x81 | `UPMIX_STATUS` | G | none | 16 | - |  | Live upmixer telemetry (UpmixStatus) (build: `PICO_RP2350` only) |
| 0x90 | `UPMIX_ENABLE` | G, S | none | 4 | 4 | DEFERRED | Upmixer on or off (f32) (build: `PICO_RP2350` only) |
| 0x91 | `UPMIX_CENTER_MODE` | G, S | none | 4 | 4 | DEFERRED | Centre mode (f32, rounded and clamped) (build: `PICO_RP2350` only) |
| 0x92 | `UPMIX_SURROUND_MODE` | G, S | none | 4 | 4 | DEFERRED | Surround mode (f32, rounded and clamped to 0-2) (build: `PICO_RP2350` only) |
| 0x93 | `UPMIX_STRENGTH` | G, S | none | 4 | 4 | DEFERRED | Strength in percent (f32) (build: `PICO_RP2350` only) |
| 0x94 | `UPMIX_CENTER_WIDTH` | G, S | none | 4 | 4 | DEFERRED | Centre width in percent (f32) (build: `PICO_RP2350` only) |
| 0x95 | `UPMIX_THRESHOLD` | G, S | none | 4 | 4 | DEFERRED | Correlation threshold in percent (f32) (build: `PICO_RP2350` only) |
| 0x96 | `UPMIX_ATTACK` | G, S | none | 4 | 4 | DEFERRED | Steering attack in ms (f32) (build: `PICO_RP2350` only) |
| 0x97 | `UPMIX_RELEASE` | G, S | none | 4 | 4 | DEFERRED | Steering release in ms (f32) (build: `PICO_RP2350` only) |
| 0x98 | `UPMIX_DET_HPF` | G, S | none | 4 | 4 | DEFERRED | Detector high-pass in Hz (f32) (build: `PICO_RP2350` only) |
| 0x99 | `UPMIX_SUR_DELAY` | G, S | none | 4 | 4 | DEFERRED | Surround delay in ms (f32) (build: `PICO_RP2350` only) |
| 0x9A | `UPMIX_SUR_HPF` | G, S | none | 4 | 4 | DEFERRED | Surround high-pass in Hz (f32) (build: `PICO_RP2350` only) |
| 0x9B | `UPMIX_SUR_LPF` | G, S | none | 4 | 4 | DEFERRED | Surround low-pass in Hz (f32) (build: `PICO_RP2350` only) |
| 0x9C | `UPMIX_DECORR` | G, S | none | 4 | 4 | DEFERRED | Surround decorrelation in percent (f32) (build: `PICO_RP2350` only) |
| 0x9D | `UPMIX_PRESENCE` | G, S | none | 4 | 4 | DEFERRED | Centre presence in dB (f32) (build: `PICO_RP2350` only) |

<!-- VC:END FX -->

### 13.1 Crossfeed

Headphone crossfeed mixes a filtered, delayed part of each channel into the other.

| Command | Payload | Range |
|---|---|---|
| `FX.CROSSFEED_ENABLE` | u8 | non-zero enables |
| `FX.CROSSFEED_PRESET` | u8 | 0 default (700 Hz, 4.5 dB), 1 Chu Moy (700 Hz, 6.0 dB), 2 Meier (650 Hz, 9.5 dB), 3 custom. Above 3 is `VC_ERR_VALUE` |
| `FX.CROSSFEED_FREQ` | f32 | Custom cutoff, 500 to 2000 Hz, clamped. Used only by the custom preset |
| `FX.CROSSFEED_FEED` | f32 | Custom feed level, 0 to 15 dB, clamped. Used only by the custom preset |
| `FX.CROSSFEED_ITD` | u8 | Interaural time delay, non-zero enables |
| `FX.CROSSFEED_OUTPUTS` | u8 | Output slots crossfeed applies to, bit s = slot s (outputs 2s and 2s+1). Masked to the slot count. Default `0x01`. The PDM sub is not a slot |

### 13.2 Subharmonic synthesizer

A dbx-style octave divider that synthesizes a sub-octave in three bands and mixes it into the selected outputs. It ships disabled.

| Command | Payload | Range, clamped | Default |
|---|---|---|---|
| `FX.SUBHARM_ENABLE` | u8 | non-zero enables | off |
| `FX.SUBHARM_LOW` | f32 dB | 24 to 36 Hz band level, -30 to +12. -30 turns the band off | 0 |
| `FX.SUBHARM_HIGH` | f32 dB | 36 to 56 Hz band level, -30 to +12 | 0 |
| `FX.SUBHARM_TOP` | f32 dB | 56 to 80 Hz band level, -30 to +12 | -30 (off) |
| `FX.SUBHARM_BOOST` | f32 dB | LF boost bell at 70 Hz, 0 to +6. 0 is off | 0 |
| `FX.SUBHARM_MASK` | u16 | Outputs fed, stored as sent | `0xFFFF` |
| `FX.SUBHARM_SELECT` | u8 | Selectivity: 0 all, 1 percussive, 2 sustained. Above 2 clamps to 2 | 0 |
| `FX.SUBHARM_DEPTH` | f32 % | Selectivity depth, 0 to 100 | 100 |
| `FX.SUBHARM_HOLD` | f32 ms | Selectivity hold, 50 to 400 | 150 |
| `FX.SUBHARM_CEILING` | f32 dBFS | Output ceiling, -40 to 0. 0 is off | 0 |
| `FX.SUBHARM_LINK` | u8 | Non-zero synthesizes each output pair from its mono sum | on |
| `FX.SUBHARM_SOLO` | u8 | Non-zero leaves only the synthesized sub on the masked outputs | off |

`FX.SUBHARM_SOLO` is for monitoring. It is never saved, it is not in the bulk structure and it sends no notification, so poll it.

`FX.SUBHARM_HEADROOM` (f32 dB) is the preamp headroom the host must free so the effect cannot clip, computed from the live settings on each read. It reads 0 while the synth is off.

`FX.SUBHARM_METER` returns one u16 per output. Each is a decaying peak of the synthesized sub mixed into that output, 0 to 32767 (the same scale as `MEAS.LEVELS`).

Spec: `Features/subharmonic_synth_spec.md`.

### 13.3 Psychoacoustic bass

Adds harmonics of the bass the speakers cannot reproduce, so the ear hears the missing fundamental. It runs before the crossover. It ships disabled.

| Command | Payload | Range, clamped | Default |
|---|---|---|---|
| `FX.PSYBASS_ENABLE` | u8 | non-zero enables | off |
| `FX.PSYBASS_CUTOFF` | f32 Hz | 30 to 300 | 80 |
| `FX.PSYBASS_HARMONICS` | f32 dB | Harmonic mix level, -24 to +12 | 0 |
| `FX.PSYBASS_DRIVE` | f32 dB | Odd-path drive, 0 to 18 | 6 |
| `FX.PSYBASS_CHARACTER` | f32 % | 0 even harmonics only, 100 odd only | 50 |
| `FX.PSYBASS_ORIGINAL` | f32 dB | Original low-band level, -60 to 0 | 0 |
| `FX.PSYBASS_MASK` | u16 | Outputs it applies to, stored as sent | `0xFFFF` |

Spec: `Features/psychoacoustic_bass_spec.md`.

### 13.4 Tube preamp

A biased, asymmetric waveshaper with power-supply sag and an optional output transformer stage. Every parameter travels as a float32, including the flags, the mask and the enumerations. Flags are non-zero true. The mask and enumerations are clamped and rounded to the nearest integer.

| Command | Range | Default |
|---|---|---|
| `FX.TUBE_ENABLE` | 0 or 1 | 0 |
| `FX.TUBE_OUTPUT_MASK` | 0 to 65535, bit k = output k | 65535 |
| `FX.TUBE_TYPE` | 0 to 16, 0 custom | 1 (12AX7) |
| `FX.TUBE_DRIVE` | -30 to +24 dB | -12 |
| `FX.TUBE_BIAS` | -100 to +100 % | 10 |
| `FX.TUBE_ASYM` | -12 to +12 dB | 3 |
| `FX.TUBE_HARDNESS` | 0 to 100 % | 40 |
| `FX.TUBE_SAG` | 0 to 100 % | 15 |
| `FX.TUBE_RECTIFIER` | 0 to 3, 0 solid state | 1 (GZ34) |
| `FX.TUBE_XFMR_ENABLE` | 0 or 1 | 1 |
| `FX.TUBE_XFMR_DAMPING` | 1 to 20 | 2 |
| `FX.TUBE_XFMR_RES` | 30 to 150 Hz | 95 |
| `FX.TUBE_MIX` | 0 to 100 % | 100 |
| `FX.TUBE_TRIM` | -12 to +12 dB | 0 |

Setting `FX.TUBE_TYPE` to a stock tube (1 to 16) also rewrites bias, asymmetry, hardness and sag from that tube's row, and each changed field sends its own notification. Type 0 leaves them alone. Writing bias, asymmetry, hardness or sag to a value different from the stored one sets the type to 0 (custom). Re-read those five after either kind of write. Spec: `Features/tube_preamp_spec.md`.

### 13.5 Stereo upmixer (RP2350)

Derives centre, left surround and right surround signals from a stereo input. While it runs, they replace inputs 3, 4 and 5 as matrix rows 2, 3 and 4, and the matrix routes them like any other input. It runs only for a stereo input at up to 48 kHz. These commands exist only on RP2350.

`FX.UPMIX_CONFIG` (44 bytes) carries the whole configuration. `FX.UPMIX_ENABLE` to `FX.UPMIX_PRESENCE` carry one field each as a float32.

| Offset | Type | Field | Single command | Range applied at compute time |
|---|---|---|---|---|
| 0 | u8 | Enable, non-zero on | `FX.UPMIX_ENABLE` | |
| 1 | u8 | Centre mode: 0 passive, 1 adaptive, 2 off | `FX.UPMIX_CENTER_MODE` | Outside 0 to 2 becomes 1 |
| 2 | u8 | Surround mode: 0 off, 1 passive, 2 adaptive | `FX.UPMIX_SURROUND_MODE` | Above 2 becomes 2 |
| 3 | i8 | Presence, dB x 2 | `FX.UPMIX_PRESENCE` (dB as f32) | -12 to +12 dB |
| 4 | f32 | Strength, percent | `FX.UPMIX_STRENGTH` | 0 to 100 |
| 8 | f32 | Centre width, percent | `FX.UPMIX_CENTER_WIDTH` | 0 to 100 |
| 12 | f32 | Correlation threshold, percent | `FX.UPMIX_THRESHOLD` | 0 to 95 |
| 16 | f32 | Steering attack, ms | `FX.UPMIX_ATTACK` | 1 to 500 |
| 20 | f32 | Steering release, ms | `FX.UPMIX_RELEASE` | 5 to 2000 |
| 24 | f32 | Detector high-pass, Hz | `FX.UPMIX_DET_HPF` | 20 to 1000 |
| 28 | f32 | Surround delay, ms | `FX.UPMIX_SUR_DELAY` | 0 to 20 |
| 32 | f32 | Surround high-pass, Hz | `FX.UPMIX_SUR_HPF` | 20 to 2000 |
| 36 | f32 | Surround low-pass, Hz | `FX.UPMIX_SUR_LPF` | 1000 to 20000 |
| 40 | f32 | Surround decorrelation, percent | `FX.UPMIX_DECORR` | 0 to 100 |

The mode fields are sanitized when written. The float fields are stored as sent and clamped only when the coefficients are computed, so they read back unclamped. Presence is the exception in `FX.UPMIX_CONFIG`, which carries it in 0.5 dB steps. A non-finite float in either form is `VC_ERR_VALUE`.

`FX.UPMIX_STATUS` (16 bytes).

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | 1 while processing |
| 1 | u8 | Why it is parked: 0 active, 1 disabled, 2 input not stereo, 3 rate above 48 kHz |
| 2 | i16 | L/R correlation, value / 16384 (-1 to +1) |
| 4 | i16 | Balance, 0 centred, 16384 fully one-sided |
| 6 | u16 | Centre extraction gain, value / 32767 |
| 8 | u16 | Left surround steering gain, value / 32767 |
| 10 | u16 | Right surround steering gain, value / 32767 |
| 12 | u8[4] | Reserved |

Spec: `Features/upmixer_spec.md`.

---

## 14. MEAS (0x09)

Measurement. This page holds the level meters, the clip flags, the spectrum analyser (RTA) and the test signal generator. The analyser and the generator are transient. They are idle at boot and never saved, and a preset load or factory reset stops the generator.

<!-- VC:BEGIN MEAS -->

| Cmd | Name | Dir | wValue | GET bytes | SET bytes | Flags | Notes |
|---|---|---|---|---|---|---|---|
| 0x10 | `LEVELS` | G | none | var | - |  | Peaks per channel (u16 each), cpu0, cpu1, u32 clip flags, active input count |
| 0x11 | `CLIP_FLAGS` | G | none | 4 | - |  | Latched clip flags (u32); reading does not clear them |
| 0x12 | `CLIP_CLEAR` | S0 | none | - | 0 |  | Clear the latched clip flags |
| 0x20 | `RTA_CONFIG` | G, S | none | 12 | 12 | DEFERRED | Analyser config (RtaConfig); applied by the main loop |
| 0x21 | `RTA_CAPS` | G | none | 16 | - |  | Analyser capabilities (RtaCaps) |
| 0x22 | `RTA_BAND_CENTRES` | G | index | var | - |  | Band centre table chunk n: up to 32 u16 Hz values |
| 0x23 | `RTA_BANDS` | G | index | 82 | - | KEEPALIVE | Latest band frame for one channel at the tap (RtaBandFrame) |
| 0x24 | `RTA_BINS` | G | byte offset | var | - | KEEPALIVE | Published bin frame from a byte offset, read in chunks |
| 0x25 | `RTA_STATUS` | G | none | 24 | - |  | Analyser status (RtaStatus); not a keepalive |
| 0x26 | `RTA_BANDS_ALL` | G | none | var | - | USB_ONLY, BULK, KEEPALIVE | Band frames of every live channel, back to back |
| 0x28 | `RTA_START` | S0 | none | - | 0 |  | Start the analyser |
| 0x29 | `RTA_STOP` | S0 | none | - | 0 |  | Stop the analyser |
| 0x2A | `RTA_RESET` | S0 | none | - | 0 |  | Clear averages and peaks |
| 0x40 | `SIGGEN_CONFIG` | G, S | none | 36 | 36 |  | Signal generator config (SiggenConfig); restarts a running generator |
| 0x41 | `SIGGEN_STATUS` | G | none | 16 | - |  | Signal generator status (SiggenStatus) |
| 0x42 | `SIGGEN_CAPS` | G | none | 8 | - |  | Signal generator capabilities header (SiggenCapsHeader) |
| 0x43 | `SIGGEN_TYPE_DESC` | G | index | 62 | - |  | Description of one signal type (SiggenTypeDesc) |
| 0x48 | `SIGGEN_START` | S0 | none | - | 0 |  | Start, or restart, with the applied config |
| 0x49 | `SIGGEN_STOP` | S0 | none | - | 0 |  | Fade out, then stop |
| 0x4A | `SIGGEN_STOP_NOW` | S0 | none | - | 0 |  | Stop at once, no fade |

<!-- VC:END MEAS -->

### 14.1 Meters and clip flags

`MEAS.LEVELS` returns `2 x NUM_CHANNELS + 7` bytes (21 on RP2040, 41 on RP2350). N is `NUM_CHANNELS`.

| Offset | Type | Field |
|---|---|---|
| 0 | u16[N] | Peak level per channel, in channel order (inputs, then outputs). Linear, 0 to 32767 is 0 to full scale |
| 2N | u8 | Core 0 load, percent |
| 2N + 1 | u8 | Core 1 load, percent |
| 2N + 2 | u32 | Latched clip flags, bit k = channel k |
| 2N + 6 | u8 | Active input channel count |

`MEAS.CLIP_FLAGS` reads the latched clip flags without clearing them. `MEAS.CLIP_CLEAR` clears them. A clip that lands between the two requests is cleared without being seen. Spec: `Features/peak_clip_metering_spec.md`.

### 14.2 Spectrum analyser

The analyser measures up to every channel at one tap point, either the inputs after their EQ or the outputs after gain and delay. It combines an FFT above 200 Hz with a continuous filter bank for 10 to 200 Hz. It stops itself 5000 ms after the last band or bin read, unless its `MANUAL` flag is set. The reads marked `KEEPALIVE` (`MEAS.RTA_BANDS`, `MEAS.RTA_BINS`, `MEAS.RTA_BANDS_ALL`) refresh that timer and start the analyser if it is idle and not manual. `MEAS.RTA_BANDS` and `MEAS.RTA_BINS` count as a keepalive even when they refuse the address. `MEAS.RTA_STATUS` does not count.

`MEAS.RTA_START`, `MEAS.RTA_STOP` and `MEAS.RTA_RESET` act at once. Start does nothing if the analyser already runs. Reset clears the averages and peaks.

`MEAS.RTA_CONFIG` (12 bytes). The SET is staged and applied by the main loop, which then sends notification `RTA_CONFIG` (`0x0D`), even when nothing changed. The GET returns the applied configuration.

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Version, must be 3 |
| 1 | u8 | Tap: 0 inputs (after EQ), 1 outputs (after gain and delay) |
| 2 | u16 | Channel mask, bit i = channel i at that tap. At least one bit must fall inside the tap's width |
| 4 | u8 | FFT order, 8 to 10 (default 10 on RP2350, 9 on RP2040) |
| 5 | u8 | Reserved, forced to 0 |
| 6 | u16 | Extra averaging, ms, clamped to 0 to 10000 |
| 8 | u8 | Peak decay, dB/s, clamped to 0 to 100. 0 holds peaks |
| 9 | u8 | Flags: bit 0 `MANUAL` (no automatic start, no idle stop) |
| 10 | u16 | Reserved, forced to 0 |

A wrong version, a tap other than 0 or 1, an FFT order outside 8 to 10 or a mask with no bit in range is `VC_ERR_VALUE`.

`MEAS.RTA_CAPS` (16 bytes).

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Version (3) |
| 1 | u8 | Input channels |
| 2 | u8 | Output channels |
| 3 | u8 | Smallest FFT order (8) |
| 4 | u8 | Largest FFT order (10) |
| 5 | u8 | Default FFT order |
| 6 | u8 | Bass bands (14), the filter-bank bands starting at band 0 |
| 7 | u8 | Bands per frame (37) |
| 8 | u8 | Level byte that means 0 dBFS (243) |
| 9 | u8 | Dynamic range, dB (120 RP2350, 78 RP2040) |
| 10 | u16 | Idle timeout, ms (5000) |
| 12 | u16 | Largest bin frame, bytes (529) |
| 14 | u16 | Bass bank dynamic range, dB (70) |

`MEAS.RTA_BAND_CENTRES` with `wValue` = chunk n returns up to 32 u16 band centre frequencies in Hz, starting at band `32 x n`. With 37 bands, chunk 0 is 64 bytes and chunk 1 is 10 bytes. A chunk past the table is `VC_ERR_ADDRESS`.

Level bytes in band and bin frames are 0.5 dB steps. 243 is 0 dBFS, 255 is +6 dBFS and 0 is the floor.

`MEAS.RTA_BANDS` with `wValue` = channel at the configured tap returns one 82-byte band frame. A channel outside the tap's width is `VC_ERR_ADDRESS`.

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Version (3) |
| 1 | u8 | Channel |
| 2 | u8 | Frame counter for this channel |
| 3 | u8 | Bands valid at this rate. 0 when the rate has no band table |
| 4 | u16 | Age of the frame, ms. `0xFFFF` means never |
| 6 | u16 | Reserved |
| 8 | u8[37] | Averaged levels |
| 45 | u8[37] | Peak levels |

`MEAS.RTA_BANDS_ALL` returns the band frames of every live channel back to back, 82 bytes each, in channel order. It is USB only and uses the bulk buffer, so a held buffer is `VC_ERR_BUSY`. No live channel returns zero bytes, which is not an error.

`MEAS.RTA_BINS` returns the raw FFT bins of the most recent frame, from the byte offset in `wValue`, at most 256 bytes per request. The frame is a 16-byte header, `n_bins` level bytes, and one final byte that repeats the header's counter.

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Version (3) |
| 1 | u8 | Channel |
| 2 | u8 | Frame counter, repeated as the frame's last byte |
| 3 | u8 | FFT order |
| 4 | u32 | Sample rate, Hz |
| 8 | u16 | `n_bins` (N/2) |
| 10 | u16[3] | Reserved |
| 16 | u8[n_bins] | Bin levels |
| 16 + n_bins | u8 | Frame counter again |

The frame is 145, 273 or 529 bytes for FFT order 8, 9 or 10. Read it in pieces at offsets 0, 256 and 512, then compare byte 2 with the last byte. If they differ, the frame changed between pieces, so read it again. Before the first transform the frame has a zero header and a `0xFF` final byte, so check the counter before trusting the data. An offset at or past the frame length is `VC_ERR_ADDRESS`.

`MEAS.RTA_STATUS` (24 bytes).

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Version (3) |
| 1 | u8 | State: 0 idle, 1 capturing, 2 transforming |
| 2 | u8 | Tap |
| 3 | u8 | Channel being worked on, `0xFF` when idle |
| 4 | u8 | Reserved |
| 5 | u8 | Live channel count |
| 6 | u16 | Live mask (channels selected and live at the tap) |
| 8 | u16 | Frames per second |
| 10 | u16 | Transform time per second, us (averaged) |
| 12 | u16 | Time of the last frame, us |
| 14 | u16 | Time since the last keepalive read, ms. `0xFFFF` means never |
| 16 | u32 | Sample rate, Hz |
| 20 | u8 | First band: 0 with a band table, `0xFF` for an unsupported rate |
| 21 | u8 | Reserved |
| 22 | u16 | Bass bank time per second, us, saturates at 65535 |

Spec: `Features/spectrum_analyser_spec.md`.

### 14.3 Signal generator

The generator synthesizes measurement signals straight into the output path, with no host audio needed. It replaces the routed audio on the selected outputs between the matrix and the per-output processing, so slot alignment is untouched. An output plays the signal only if it is in the generator's channel mask and enabled in the matrix.

`MEAS.SIGGEN_CONFIG` (36 bytes) never starts the generator. A SET while it runs fades out, applies the new configuration and fades back in.

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Version, must be 1 |
| 1 | u8 | Signal type, 0 to 14 (below) |
| 2 | u16 | Output mask, bit i = output i. Bits past the output count are dropped |
| 4 | u16 | Polarity-inverted subset of the mask |
| 6 | u8 | Flags: bit 0 RAW (bypass crossover and EQ on the generator's outputs), bit 1 DECORR (independent noise per output), bit 2 WALK (step through the outputs) |
| 7 | u8 | Reserved |
| 8 | f32 | Peak level, dBFS, -120 to 0, clamped (NaN becomes -120) |
| 12 | u32 | Duration, ms (meaning depends on the timing model) |
| 16 | u16 | Repeat count (meaning depends on the timing model) |
| 18 | u16 | Gap between cycles, ms |
| 20 | f32 x 4 | Parameters p1 to p4 (per type). Unused ones become 0. NaN becomes 0, which selects the default. Others clamp to the range in the type descriptor |

A wrong version, an unknown type, a mask with no valid output, or a sweep with duration 0 is `VC_ERR_VALUE`.

| Type | Name | Timing | p1 | p2 | p3 | p4 |
|---|---|---|---|---|---|---|
| 0 | Sine | continuous | freq Hz | | | |
| 1 | Square | continuous | freq Hz | | | |
| 2 | White noise | continuous | | | | |
| 3 | Pink noise | continuous | | | | |
| 4 | Log sweep | sweep | f1 Hz | f2 Hz | | |
| 5 | Linear sweep | sweep | f1 Hz | f2 Hz | | |
| 6 | Stepped sweep | sweep | f1 Hz | f2 Hz | steps per octave | dwell ms |
| 7 | Impulse | pattern | period ms | | | |
| 8 | Alternating clicks | pattern | period ms | | | |
| 9 | Polarity pulse | pattern | pulse ms | period ms | | |
| 10 | Tone burst | pattern | freq Hz | on cycles | off cycles | edge cycles |
| 11 | Tone pair | continuous | f1 Hz | f2 Hz | amplitude ratio | |
| 12 | Multitone | continuous | tone count | f low Hz | f high Hz | |
| 13 | Inter-sample peak | continuous | pattern 0 or 1 | | | |
| 14 | Channel ID | pattern | blip ms | | | |

The timing model gives duration, repeat and gap their meaning.

- **Continuous.** Duration is the total time, 0 until stopped. With WALK it is the time per output, and repeat is the number of passes over the mask.
- **Sweep.** Duration is one sweep and must be above 0. Repeat is the number of sweeps, 0 for endless. Gap separates sweeps.
- **Pattern.** Repeat is the number of periods, 0 for endless. Gap adds silence per period.

`MEAS.SIGGEN_START` starts with the applied configuration, or restarts a running generator. Before any valid configuration it is `VC_ERR_STATE`. `MEAS.SIGGEN_STOP` fades out over 5 ms. `MEAS.SIGGEN_STOP_NOW` stops at once.

`MEAS.SIGGEN_STATUS` (16 bytes).

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Version (1) |
| 1 | u8 | State: 0 idle, 1 fade in, 2 run, 3 gap, 4 fade out |
| 2 | u8 | Signal type |
| 3 | u8 | Current output in WALK mode, else `0xFF` |
| 4 | u32 | Elapsed, ms |
| 8 | u16 | Completed cycles, saturates at `0xFFFF` |
| 10 | u8 | Stop reason: 0 none, 1 host, 2 completed, 3 preset load, 4 reconfigured |
| 11 | u8 | Reserved |
| 12 | f32 | Current sweep frequency, Hz, else 0 |

`MEAS.SIGGEN_CAPS` (8 bytes) is the version, the type count, the output count, the largest multitone count (8 on RP2040, 16 on RP2350), a u16 mask of valid outputs and 2 reserved bytes.

`MEAS.SIGGEN_TYPE_DESC` with `wValue` = type returns 62 bytes. They are u8 type, an 8-byte name, u8 timing model (0 continuous, 1 sweep, 2 pattern), then four 13-byte parameter descriptors. Each descriptor is u8 meaning (0 unused, 1 frequency Hz, 2 ms, 3 cycles, 4 count, 5 ratio, 6 pattern) and f32 minimum, maximum and default. These are the authoritative ranges for the platform. A type past the table is `VC_ERR_ADDRESS`.

Starts, stops and completions send notification `SIGGEN_STATE` (`0x07`).

Example: a 1 kHz sine at -20 dBFS on outputs 0 and 1. Send `MEAS.SIGGEN_CONFIG` with this payload, then `MEAS.SIGGEN_START`.

```
01 00 03 00 00 00 00 00  00 00 A0 C1 00 00 00 00
00 00 00 00 00 00 7A 44  00 00 00 00 00 00 00 00
00 00 00 00
```

Spec: `Features/test_signals_spec.md`.

---

## 15. CS (0x0A)

Control Surfaces bind physical controls and indicators on spare GPIOs (buttons, switches, pots, encoders, LEDs, an IR receiver, an I2C display, aux outputs) to device parameters. Every command works on both platforms and every transport.

<!-- VC:BEGIN CS -->

| Cmd | Name | Dir | wValue | GET bytes | SET bytes | Flags | Notes |
|---|---|---|---|---|---|---|---|
| 0x10 | `CAPS` | G | none | 52 | - |  | Capability header and component type table |
| 0x11 | `NOUN_DESC` | G | CS noun | 12 | - |  | Descriptor of one noun |
| 0x12 | `STATUS` | G | none | 41 | - |  | Last deferred result, dirty flag, per-slot and IR status |
| 0x13 | `EXT_STATUS` | G | none | 24 | - |  | Group and macro limits, validity and the running macro |
| 0x18 | `SAVE` | S0 | none | - | 0 | DEFERRED, FLASH, RESTART | Persist the whole live configuration |
| 0x19 | `REVERT` | S0 | none | - | 0 | DEFERRED | Discard the live preview and re-apply the stored configuration |
| 0x20 | `BINDING` | G, S | CS slot | 24 | 24 | DEFERRED | One binding, live preview |
| 0x21 | `SLOT_NAME` | G, S | CS slot | 32 | 1-32 | DEFERRED | Slot name, NUL padded; a single NUL clears it |
| 0x22 | `BINDING_PIN_AVAIL` | G | `(CS type << 8) \| slot` | 80 | - |  | Pins a binding of one type in one slot could take: status, flags, generation, masks, shareable |
| 0x30 | `IR_CMD` | G, S | CS sub-slot | 16 | 16 | DEFERRED | One IR command, live preview |
| 0x31 | `IR_LEARN_STATUS` | G | none | 8 | - |  | Learn state, protocol and the last learned code |
| 0x32 | `IR_LEARN_ARM` | S0 | none | - | 0 |  | Arm IR learn on the live receiver |
| 0x33 | `IR_LEARN_CANCEL` | S0 | none | - | 0 |  | Cancel IR learn |
| 0x40 | `GROUP` | G, S | CS group | 40 | 40 | DEFERRED | One target group, live preview |
| 0x50 | `MACRO` | G | CS macro | 132 | - |  | Whole macro: header and every step |
| 0x51 | `MACRO_HEADER` | G, S | CS macro | 36 | 36 | DEFERRED | Macro name and step count |
| 0x52 | `MACRO_STEP` | G, S | CS macro step | 12 | 12 | DEFERRED | One macro step, address (step << 8) \| macro |
| 0x58 | `MACRO_FIRE` | S0 | CS macro | - | 0 | DEFERRED | Run a macro; an empty macro is a no-op |
| 0x59 | `MACRO_CANCEL` | S0 | none | - | 0 |  | Stop the running macro |
| 0x60 | `AUX_STATE` | G, S | CS slot | 1 | 1 |  | Aux output on or off |
| 0x61 | `AUX_LEVEL` | G, S | CS slot | 2 | 2 |  | Aux PWM level, 8.8 percent, clamped to 100 |
| 0x62 | `AUX_ALL` | G | none | 48 | - |  | 16 state bytes then 16 levels; non-aux slots read zero |
| 0x70 | `DISPLAY_CONFIG` | G, S | none | 12 | 12 | DEFERRED | Display mode, timing, brightness and flags |
| 0x71 | `DISPLAY_CAPS` | G | none | 4 | - |  | Display page count and model count |
| 0x72 | `DISPLAY_PAGE` | G, S | CS page | 4 | 4 | DEFERRED | One display page; all zero is empty |
| 0x73 | `DISPLAY_STATUS` | G | none | 8 | - |  | Link state, shown page, live model and I2C NAK count |

<!-- VC:END CS -->

### 15.1 Preview, status, Save and Revert

The edit SETs (`CS.BINDING`, `CS.SLOT_NAME`, `CS.IR_CMD`, `CS.GROUP`, `CS.MACRO_HEADER`, `CS.MACRO_STEP`, `CS.DISPLAY_CONFIG`, `CS.DISPLAY_PAGE`) are live previews.

1. The dispatcher checks the address and the length. A bad one is refused (`VC_ERR_ADDRESS`, `VC_ERR_LENGTH`) and leaves the status packet alone.
2. The handler copies the record into that kind's single pending slot and sets `CS.STATUS` byte 0 to `0x16` (PENDING). If the slot is still occupied by an earlier SET of the same kind, the request is `VC_ERR_BUSY` and the status becomes `0x1B` (BUSY). The earlier one is kept.
3. The main loop applies the record. It claims pins and takes effect, but writes nothing to flash. It then sets `CS.STATUS` byte 0 to the result and, on success, sets the dirty flag.

So `VC_OK` on an edit SET means queued. Poll `CS.STATUS` until byte 0 is no longer `0x16`, and check byte 1 to see which record the result belongs to. All kinds share one result pair, so let each SET resolve before the next one of any kind.

| `last_slot` (byte 1) | Source |
|---|---|
| `0x00` to `0x0F` | `CS.BINDING` or `CS.SLOT_NAME` of that slot, or a refused aux SET |
| `0x40` to `0x47` | `CS.GROUP`, `0x40 \| group` |
| `0x50` | `CS.DISPLAY_CONFIG` (and `CS.DISPLAY_PAGE` of page 0) |
| `0x50` to `0x5F` | `CS.DISPLAY_PAGE`, `0x50 \| page` |
| `0x60` to `0x67` | `CS.MACRO_HEADER`, `CS.MACRO_STEP`, `0x60 \| macro` |
| `0x80` to `0x8F` | `CS.IR_CMD`, `0x80 \| sub-slot` |
| `0xFF` | `CS.SAVE`, `CS.REVERT`, or `CS.IR_LEARN_ARM` refused for want of an IR receiver |

`CS.SAVE` stores the whole live configuration (16 bindings, 16 IR commands, 16 slot names, 8 groups, 8 macros, the display configuration and pages) in one directory flash write. The result is `0x00` or `0x1C` (flash error), and the dirty flag clears on success. `CS.REVERT` discards the preview and re-applies the stored configuration as a background job, releasing and reclaiming pins, PWM slices, the IR receiver and the display as at boot. Both report PENDING, then the result. A Save or Revert while one is already queued is `VC_ERR_BUSY` (status BUSY). A reboot discards an unsaved preview.

While a revert runs, queued edit SETs wait. `CS.MACRO_FIRE`, `CS.IR_LEARN_ARM`, `CS.IR_LEARN_CANCEL`, `CS.AUX_STATE` and `CS.AUX_LEVEL` SETs are `VC_ERR_BUSY` and leave the status alone. `CS.BINDING` and `CS.IR_CMD` GETs return the stored records, and `CS.STATUS` returns the per-slot fields as they were when the revert began.

Status codes. `0x00` to `0x02` are shared with the pin results.

| Code | Name | Code | Name |
|---|---|---|---|
| `0x00` | SUCCESS | `0x1A` | EVENT_IN_USE |
| `0x01` | INVALID_PIN | `0x1B` | BUSY |
| `0x02` | PIN_IN_USE | `0x1C` | FLASH_ERROR |
| `0x10` | INVALID_SLOT | `0x1D` | IR_IN_USE |
| `0x11` | INVALID_TYPE | `0x1E` | NO_IR |
| `0x12` | INVALID_NOUN | `0x1F` | INVALID_GROUP |
| `0x13` | INVALID_ACTION | `0x20` | INVALID_MACRO |
| `0x14` | INVALID_VALUE | `0x21` | INVALID_STEP |
| `0x15` | PIN_NOT_ADC | `0x22` | DISPLAY_IN_USE |
| `0x16` | PENDING | `0x23` | PIN_NOT_I2C |
| `0x17` | INVALID_TARGET | `0x24` | I2C_IN_USE |
| `0x18` | INVALID_EVENT | `0x25` | INVALID_PAGE |
| `0x19` | PWM_CONFLICT | `0x26` | INVALID_AUX |

Spec: `Features/control_surfaces_spec.md`.

### 15.2 Capabilities and status

`CS.CAPS` (52 bytes at capability version 22).

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Capability version (22) |
| 1 | u8 | Binding slots (16) |
| 2 | u8 | Component type count (11) |
| 3 | u8 | Noun count (81) |
| 4 | 4 bytes per type | Per type: u16 allowed actions mask, u8 pin count, u8 pin class (0 any, 1 ADC) |
| 4 + 4 x types | u8 | IR command slots (16) |
| +1 | u8 | Groups (8) |
| +2 | u8 | Macros (8) |
| +3 | u8 | Steps per macro (8) |

The type table grows when types are added, so find the last four bytes at `4 + 4 x type_count`.

`CS.NOUN_DESC` with `wValue` = noun returns 12 bytes. Enumerate nouns 0 to `noun_count - 1` at connect instead of hard-coding the list.

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Kind: 0 continuous, 1 bool, 2 enum |
| 1 | u8 | Enum count |
| 2 | u16 | Allowed actions mask. 0 means the noun is not available on this platform |
| 4 | i16 | Minimum, unit-encoded |
| 6 | i16 | Maximum, unit-encoded |
| 8 | u8 | Unit |
| 9 | u8 | Target kind |
| 10 | u8 | Target count |
| 11 | u8 | Flags: bit 0 deferred apply |

`CS.STATUS` (41 bytes).

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Result of the most recent deferred request (15.1) |
| 1 | u8 | Which record it belongs to (15.1) |
| 2 | u8 | Binding slots (16) |
| 3 | u8 | 1 while the live configuration has unsaved changes |
| 4 | u16 | Live bindings, bit n = slot n |
| 6 | u8[16] | Apply status per binding slot |
| 22 | u16 | Live IR commands, bit n = sub-slot n |
| 24 | u8 | IR learn state: 0 idle, 1 armed, 2 done, 3 timeout |
| 25 | u8[16] | Apply status per IR sub-slot |

`CS.EXT_STATUS` (24 bytes) is the group count (8), macro count (8), steps per macro (8), the running macro (`0xFF` idle), its current step (0 when idle), 3 reserved bytes, then 8 group status bytes and 8 macro status bytes (the worst step validity of each macro).

### 15.3 Bindings and slot names

`CS.BINDING` (24 bytes, address = slot 0 to 15). The same layout is stored in flash. Type 0 clears the slot.

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Type: 0 none, 1 button, 2 switch, 3 pot, 4 encoder, 5 LED, 6 PWM LED, 7 IR receiver, 8 display, 9 aux output, 10 aux PWM output |
| 1 | u8 | Noun |
| 2 | u8 | Action |
| 3 | u8 | Flags: `0x01` invert, `0x02` reverse, `0x04` wrap, `0x08` accelerate, `0x10` repeat, `0x20` group target, `0x40` absolute link, `0x80` whole group |
| 4 | u8 | GPIO 0 |
| 5 | u8 | GPIO 1, `0xFF` unless the type uses two pins (encoder, display) |
| 6 | u8 | Event, buttons only: 0 press, 1 long press, 2 double press |
| 7 | u8 | Target: a channel, or a group with the group flag |
| 8 | u8 | Index: a filter band for band nouns, else 0 |
| 9 | u8 | PWM brightness ceiling, 1 to 100 percent, 0 full |
| 10 | i16 | Value |
| 12 | i16 | Step, 0 for the unit default |
| 14 | i16 | Range minimum |
| 16 | i16 | Range maximum. Both 0 selects the noun's full range |
| 18 | u16 | On delay, 0.1 s |
| 20 | u16 | Off delay, 0.1 s |
| 22 | u8 | Extras (aux slots), else 0 |
| 23 | u8 | Reserved, write 0 |

Value, step and range are encoded in the noun's unit. IR, display and aux slots are containers with their own rules. See `Features/control_surfaces_spec.md`, `Features/control_surfaces_display_spec.md` and `Features/control_surfaces_aux_spec.md`.

`CS.SLOT_NAME` is a 32-byte name per slot. A SET of 1 to 32 bytes keeps at most 31, and a single NUL clears it. Names survive binding changes and can be set before a binding exists. Slot names send no notification.

### 15.4 IR commands and learning

The IR receiver is a binding of type 7. Its commands live in 16 sub-slots. `CS.IR_CMD` (16 bytes, address = sub-slot 0 to 15). An all-zero record with protocol 0 clears the sub-slot. A command may be set before the receiver exists and goes live when it comes up.

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Noun |
| 1 | u8 | Action, the button subset |
| 2 | u8 | Flags: wrap, repeat, group |
| 3 | u8 | Target |
| 4 | u8 | Index |
| 5 | u8 | Protocol: 0 none, 1 NEC, 2 RC5, 3 RC6, 4 hash |
| 6 | i16 | Value |
| 8 | i16 | Step |
| 10 | u8[2] | Reserved, write 0 |
| 12 | u32 | Code |

To learn a code, send `CS.IR_LEARN_ARM`. The receiver listens for 10 s and captures the first decoded press. Completion or timeout sends notification `CS_IR_LEARN` (`0x0A`), and removing the receiver while armed counts as a timeout. A host without notifications polls `CS.IR_LEARN_STATUS` (8 bytes: state as in `CS.STATUS` byte 24, protocol, 2 zero bytes, u32 code, valid only in state 2). Arm with no live receiver is `VC_ERR_STATE` and sets the status to NO_IR. `CS.IR_LEARN_CANCEL` returns to idle and sends no event.

### 15.5 Groups and macros

A group names a set of channels that one control drives together. `CS.GROUP` is 40 bytes, address = group 0 to 7. It holds u8 target kind (0 empty, 1 input channels, 2 output channels, 3 channels by channel index), 3 reserved bytes, u32 member mask (bit n = channel n, non-zero, within the platform's channel count), 32-byte name. An empty group must be all zero. Applying a group re-validates every binding and IR command that uses it.

A macro is up to 8 steps that run in order, each after its own delay. `CS.MACRO_STEP` (12 bytes, address = `(step << 8) | macro`).

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Noun (not the macro noun itself) |
| 1 | u8 | Action: set, toggle, increment, decrement or trigger |
| 2 | u8 | Flags: wrap, group |
| 3 | u8 | Target |
| 4 | u8 | Index |
| 5 | u8 | Reserved |
| 6 | i16 | Value |
| 8 | i16 | Step |
| 10 | u16 | Delay before the step, 10 ms units |

`CS.MACRO_HEADER` (36 bytes) is a 32-byte name, u8 step count (0 to 8) and 3 reserved bytes that must be 0. `CS.MACRO` (132 bytes) is the header followed by the 8 steps, read in one transfer. Write the steps first and the header, with the final step count, last. The header and a step have separate pending slots, so both can be queued at once.

`CS.MACRO_FIRE` starts a macro at once. A macro already running is cancelled at its next step boundary. An empty macro is a successful no-op. `CS.MACRO_CANCEL` stops the running macro and always succeeds, even during a revert. Spec: `Features/control_surfaces_groups_macros_spec.md`.

### 15.6 Aux outputs

An aux output is a binding of type 9 (on and off) or 10 (PWM level). These commands address the binding slot and apply at once, with no flash write, no dirty flag and no preview.

- `CS.AUX_STATE` is 1 byte, non-zero on.
- `CS.AUX_LEVEL` is a u16 in 8.8 fixed-point percent (25600 is 100 percent), clamped to 100 percent. Only an aux PWM slot takes a level. An on-and-off slot reads 0.
- `CS.AUX_ALL` (48 bytes) is 16 state bytes, then 16 u16 levels. Slots that are not a live aux output read 0.

A GET or SET of a slot that is not a live aux output is `VC_ERR_STATE`, and a refused SET sets the status to INVALID_AUX. A SET that changes a value sends notification `CS_AUX` (`0x0C`). Spec: `Features/control_surfaces_aux_spec.md`.

### 15.7 Display

The display is a binding of type 8. Its GPIO 0 is SDA, GPIO 1 is SCL and the index is the display model, 1 to 8. The configuration is device-wide.

`CS.DISPLAY_CONFIG` (12 bytes).

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Mode: 0 fixed, 1 cycle the selected pages, 2 cycle all pages |
| 1 | u8 | Home page, 0 to 15 |
| 2 | u16 | Dwell per page, 0.1 s. At least 10 in the cycle modes |
| 4 | u16 | Change overlay hold, 0.1 s. 0 turns the overlay off |
| 6 | u8 | Brightness |
| 7 | u8 | Flags: bit 0 overlay for any change, bit 1 edits need arming, bits 3:2 label alignment, bits 5:4 value alignment (0 left, 1 centre, 2 right) |
| 8 | u16 | Edit timeout, 0.1 s |
| 10 | u8[2] | Reserved, write 0 |

`CS.DISPLAY_CAPS` (4 bytes) is the page count (16), the model count (9) and 2 reserved bytes. `CS.DISPLAY_PAGE` (4 bytes, address = page 0 to 15) is noun, target, index and flags (`0x01` active, `0x02` group, `0x04` large, `0x08` bar). All zero is an empty page.

`CS.DISPLAY_STATUS` (8 bytes).

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | State: 0 down, 1 starting, 2 live, 3 error |
| 1 | u8 | Page shown, `0xFF` for none or the overlay |
| 2 | u8 | Flags: bit 0 overlay showing, bit 1 edit armed |
| 3 | u8 | Live model |
| 4 | u16 | I2C NAK count |
| 6 | u8[2] | Reserved |

Spec: `Features/control_surfaces_display_spec.md`.

---

## 16. XPORT (0x0B)

The UART and I2C control transports. Both ship disabled. Their configuration is device-wide, persists across reboots and survives a factory reset. The SETs are USB only, so an external controller can never reconfigure or lock itself out of its own link (`VC_ERR_BLOCKED` on UART and I2C).

<!-- VC:BEGIN XPORT -->

| Cmd | Name | Dir | wValue | GET bytes | SET bytes | Flags | Notes |
|---|---|---|---|---|---|---|---|
| 0x10 | `UART_CONFIG` | G, S | none | 8 | 8 | SET_USB_ONLY, DEFERRED, FLASH, RESTART | Stored UART control config; the apply result is in STATUS |
| 0x11 | `I2C_CONFIG` | G, S | none | 8 | 8 | SET_USB_ONLY, DEFERRED, FLASH, RESTART | Stored I2C target control config; the apply result is in STATUS |
| 0x12 | `STATUS` | G | none | 8 | - |  | Last apply result and live state of each control transport |
| 0x13 | `UART_PIN_AVAIL` | G | none | 72 | - |  | UART TX and RX pins by instance: status, flags, generation, valid and free masks |
| 0x14 | `I2C_PIN_AVAIL` | G | none | 72 | - |  | I2C SDA and SCL pins by instance: status, flags, generation, valid and free masks |

<!-- VC:END XPORT -->

`XPORT.UART_CONFIG` (8 bytes).

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Enable, 0 or 1 |
| 1 | u8 | TX GPIO (a UART TX pin, GPIO % 4 == 0) |
| 2 | u8 | RX GPIO (the RX pin of the same UART, GPIO % 4 == 1) |
| 3 | u8 | 1 to push notifications over the link as type `0x40` frames |
| 4 | u32 | Baud rate, 9600 to 1000000 |

The defaults are disabled, TX GPIO 16, RX GPIO 17, notifications off and 115200 baud.

`XPORT.I2C_CONFIG` (8 bytes).

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Enable, 0 or 1 |
| 1 | u8 | SDA GPIO (an I2C SDA pin, even) |
| 2 | u8 | SCL GPIO (the SCL pin of the same I2C block, odd) |
| 3 | u8 | 7-bit target address, `0x08` to `0x77` |
| 4 | u8[4] | Reserved |

The defaults are disabled, SDA GPIO 18, SCL GPIO 19 and address `0x42`.

A SET checks the scalar fields and the GPIO numbers at once. A bad enable or notify flag, a baud rate or address out of range is `VC_ERR_VALUE`, and a GPIO that is never usable is `VC_ERR_PIN_INVALID`. The main loop then tears the interface down, checks the pin pairing and ownership, brings it up, and stores the configuration only if that worked. The outcome is in `XPORT.STATUS`. The GETs return the stored configuration.

`XPORT.STATUS` (8 bytes) is the UART's last apply result, 1 while the UART is live, the I2C's last apply result, 1 while the I2C is live, the transport framing version (1) and 3 reserved bytes. The apply results are pin codes: 0 success, 1 invalid pin, 2 pin in use, 5 invalid parameter. A stored configuration whose pins collide with the wiring at boot stays down, which `live` shows.

Spec: `Features/control_interfaces_spec.md`.

---

## 17. CLOCK (0x0C)

The I2S clock pins and clock mode, which I2S input and I2S output share, and the optional MCK output.

<!-- VC:BEGIN CLOCK -->

| Cmd | Name | Dir | wValue | GET bytes | SET bytes | Flags | Notes |
|---|---|---|---|---|---|---|---|
| 0x10 | `I2S_BCK_PIN` | G, S | role 0-1 | 1 | 1 | DEFERRED, RESTART | BCK GPIO of role 0 master or unified, 1 slave (LRCLK is BCK + 1); 0xFF restores the default |
| 0x11 | `I2S_CLOCK_PIN_MODE` | G, S | none | 1 | 1 | DEFERRED, RESTART | 0 one clock pair for both modes, 1 a separate slave pair |
| 0x12 | `I2S_CLOCK_MODE` | G, S | none | 1 | 1 | DEFERRED, RESTART | I2S clock mode, 0 master or 1 slave |
| 0x13 | `I2S_SLAVE_STATUS` | G | none | 16 | - |  | I2S slave clock lock state and measured rate |
| 0x14 | `I2S_BCK_PIN_AVAIL` | G | role 0-1 | 24 | - |  | BCK pins one clock pair could take (LRCLK is BCK + 1): status, flags, generation, valid and free masks |
| 0x20 | `MCK_ENABLE` | G, S | none | 1 | 1 |  | MCK output enable |
| 0x21 | `MCK_PIN` | G, S | none | 1 | 1 |  | MCK GPIO, a clk_gpout pin; 0xFF restores the default |
| 0x22 | `MCK_MULTIPLIER` | G, S | none | 1 | 1 |  | MCK multiplier, 0 for 128x or 1 for 256x |
| 0x23 | `MCK_PIN_AVAIL` | G | none | 24 | - |  | Pins MCK could take: status, flags, generation, valid and free masks |

<!-- VC:END CLOCK -->

### 17.1 Clock pins and clock-pin mode

I2S uses one BCK pin and LRCLK on the next GPIO (BCK + 1). There are two clock pairs. Role 0 is the master pair, and role 1 is a separate slave pair used only in split clock-pin mode. The role 0 default is GPIO 14 (LRCLK 15). The role 1 default is GPIO 26 (27) on RP2350 and GPIO 12 (13) on RP2040.

`CLOCK.I2S_CLOCK_PIN_MODE` is 0 unified (one pair serves both master and slave clocking) or 1 split (slave clocking uses the role 1 pair). Another value is `VC_ERR_VALUE`. Entering split checks that the slave pair is distinct from the master pair and free (`VC_ERR_PIN_IN_USE`). A change that would move the live clock pair while any output slot is I2S is `VC_ERR_STATE`.

`CLOCK.I2S_BCK_PIN` sets the base pin of one role. `0xFF` restores the default. Both pins of the pair must be valid (`VC_ERR_PIN_INVALID`) and free (`VC_ERR_PIN_IN_USE`). Moving the pair that the I2S output slots use while any slot is I2S is `VC_ERR_STATE`. When I2S is the active input and the change moves its live pair, the input restarts. Spec: `Features/clock_pins_spec.md`.

### 17.2 Clock mode and slave status

`CLOCK.I2S_CLOCK_MODE` is 0 master (the device drives BCK and LRCLK) or 1 slave (an external master drives them and the device follows its rate). Another value is `VC_ERR_VALUE`. The main loop applies it. If I2S is not the active input it only records the mode. If it is, the device mutes, stops the input, rebuilds the I2S output clocking, sets MCK (off in slave mode, back on at the selected rate in master mode) and restarts the input. The GET reports the live mode, so a pending change shows only once applied.

`CLOCK.I2S_SLAVE_STATUS` (16 bytes).

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | State: 0 inactive, 1 acquiring, 2 relocking, 3 locked |
| 1 | u8 | Live clock mode, 0 or 1 |
| 2 | u8 | Locks since boot, saturates at 255 |
| 3 | u8 | Losses since boot, saturates at 255 |
| 4 | u32 | Detected rate, Hz (44100, 48000 or 96000), 0 unless locked |
| 8 | u32 | Raw measured external rate, Hz, 0 with no clocks |
| 12 | u8 | Framing slips since boot, saturates at 255 |
| 13 | u8[3] | Reserved |

Lock-state changes send notification `I2S_SLAVE_STATE` (`0x09`). Spec: `Features/i2s_slave_input_spec.md`.

### 17.3 MCK

MCK comes from a hardware clock output, so its pin must be a clock-output GPIO (GPIO 21 on RP2040, 13, 15 or 21 on RP2350). The default is GPIO 21 on RP2040 and 13 on RP2350.

- `CLOCK.MCK_ENABLE` enables MCK when non-zero. Enabling checks that the pin is free (`VC_ERR_PIN_IN_USE`). It applies at once.
- `CLOCK.MCK_PIN` takes a GPIO, and `0xFF` restores the default. A pin that is not a clock output is `VC_ERR_PIN_INVALID`. Changing the pin while MCK is enabled is `VC_ERR_STATE`, even to the same pin, so disable MCK first.
- `CLOCK.MCK_MULTIPLIER` is 0 for 128 x fs or 1 for 256 x fs. Another value is `VC_ERR_VALUE`. It applies at once.

Spec: `Features/master_clock_spec.md`.

---

## 18. DIAG (0xF0)

Bench and diagnostic commands with no stability promise. Release firmware builds none of them, so the whole page is absent from `SYS.PAGE_LIST`. Each command exists only in the bench build whose option enables it.

<!-- VC:BEGIN DIAG -->

| Cmd | Name | Dir | wValue | GET bytes | SET bytes | Flags | Notes |
|---|---|---|---|---|---|---|---|
| 0x10 | `CLOCK_DIAG` | G | none | var | - |  | Latest 1 Hz ClockDiagPacket snapshot (build: `DSPI_CLOCK_DIAG` only) |
| 0x20 | `LOOPBACK_STATS` | G | none | 8 | - |  | Loopback capture overflow and underrun frame counts (build: `DSPI_LOOPBACK` only) |
| 0x30 | `LOOP_PROF_SECTION` | G | index | 4 | - |  | Worst time of main-loop section wValue (LOOP_PROF_SECTION_LIST) (build: `DSPI_LOOP_PROF` only) |
| 0x31 | `LOOP_PROF_JOB` | G | index | 4 | - |  | Worst step of background job wValue (BG_JOB_LIST) (build: `DSPI_LOOP_PROF` only) |
| 0x32 | `LOOP_PROF_COMMAND` | G | `(page << 8) \| cmd` | 4 | - |  | Worst handler time of paged command wValue, GET and SET together (build: `DSPI_LOOP_PROF` only) |
| 0x33 | `LOOP_PROF_BG_PACE` | G, S | none | 2 | 2 |  | Background pacing n: above 1, one unit per step on one pass in n; 0 sets 1 (build: `DSPI_LOOP_PROF` only) |
| 0x34 | `LOOP_PROF_RESET` | S0 | none | - | 0 |  | Zero every section, job and command worst time (build: `DSPI_LOOP_PROF` only) |
| 0x40 | `EQ_PROF` | G | none | 48 | - |  | Per core: u64 cycles, u32 calls, u32 samples, u32 min call cycles, u32 its samples (build: `DSPI_EQ_PROF` only) |
| 0x41 | `EQ_PROF_RESET_MIN` | S0 | none | - | 0 |  | Restart the per-call minimum on both cores (build: `DSPI_EQ_PROF` only) |
| 0x50 | `WIRECAP_ARM` | S | none | - | 10 |  | Start a one-shot pin capture: u8 base, u8 count, u16 div int, u8 div frac, u8 0, u32 bytes (0 aborts) (build: `DSPI_WIRECAP` only) |
| 0x51 | `WIRECAP_STATUS` | G | none | 32 | - |  | Capture state, RXSTALL flag, pins, clk_sys Hz, divider, PIO, bytes, buffer size, sequence (build: `DSPI_WIRECAP` only) |
| 0x52 | `WIRECAP_READ` | G | index | var | - |  | Captured bytes, 256-byte chunk wValue (build: `DSPI_WIRECAP` only) |
| 0x53 | `WIRECAP_CLKGEN` | G, S | none | 24 | 5 |  | I2S clock generator on GPIO 24/25: SET u8 enable, u32 rate; GET status (build: `DSPI_WIRECAP` only) |
| 0x54 | `WIRECAP_CLKGEN_GLITCH` | S | none | - | 2 |  | Inject one clock glitch: u8 kind (1 runt, 2 missing BCK, 3 LRCLK early, 4 late), u8 cells (build: `DSPI_WIRECAP` only) |
| 0x55 | `WIRECAP_RATE` | S | none | - | 4 |  | Request an output rate change through the host path, u32 Hz (USB input only) (build: `DSPI_WIRECAP` only) |

<!-- VC:END DIAG -->

- `DIAG.CLOCK_DIAG` returns the latest once-per-second `ClockDiagPacket` (sysclk, clock trim, servo, per-slot ring, PDM, input and USB feedback state). The authoritative layout is the packed struct in `firmware/DSPi/clock_diag.h`, and `tools/clkdiag_poll.py` decodes it.
- `DIAG.LOOPBACK_STATS` returns two u32 counts, loopback capture overflow frames then underrun frames.
- `DIAG.LOOP_PROF_SECTION`, `DIAG.LOOP_PROF_JOB` and `DIAG.LOOP_PROF_COMMAND` return a u32 worst time in us, since boot or the last `DIAG.LOOP_PROF_RESET`, for one main-loop section, one background job or one paged command (GET and SET together). Section and job numbers follow the lists in `firmware/DSPi/loop_prof.h`. Reads never clear them. `DIAG.LOOP_PROF_BG_PACE` is a u16 pacing divider for background jobs, where 0 stores 1.

---

## 19. Bulk parameter transfer

`STORE.ALL_PARAMS` moves the whole live DSP and IO state in one `WireBulkParams` structure. It is 6136 bytes at wire format version 32, the same size on both platforms. A host reads it at connect to build its picture of the device, and writes it to restore a saved state in one step. `STORE.ALL_PARAMS_CHUNK` moves the same structure in pieces.

### 19.1 `STORE.ALL_PARAMS`

**GET.** `wValue` 0. The device collects the live state when the request arrives and returns `min(wLength, 6136)` bytes. Over USB the reply goes out in 64-byte packets from the bulk buffer, so the host's USB layer must accept a multi-packet control read. A host that already has the header can use `payload_length` (offset 6) as the length.

**SET.** `wValue` 0, exactly 6136 bytes. The device checks the request in two steps.

| When | Check | Result |
|---|---|---|
| SETUP | Length is not 6136 | `VC_ERR_LENGTH`. On UART and I2C a length the transport cannot buffer is status `0x06` |
| SETUP | Another transport holds the bulk buffer, or an earlier bulk apply is still pending | `VC_ERR_BUSY` (UART and I2C: status `0x04`) |
| After the data | `format_version` is not 32, `payload_length` is not 6136, `platform_id` or any channel count differs from the device | `VC_ERR_VALUE`, nothing applied |

Then the main loop applies the structure under a muted, synchronized restart of every output, and sends one `BULK_INVALIDATED` notification with source 2. The bulk SET does not check float values and stores them as sent.

- **It writes no flash.** To keep a bulk-applied state across a power cycle, save it to a preset afterwards.
- **The IO configuration follows the mode.** The physical IO configuration (pins, input pin and enable settings, ADAT) and the limiter are applied only when `STORE.OUTPUT_CONFIG_MODE` is 1. In mode 0 a bulk SET leaves the device-wide wiring alone.
- **There is no version tolerance.** A host must send exactly the version the device reports in the header of its own GET.
- Confirm the apply by reading `STORE.ALL_PARAMS` again after the busy window.

Over UART and I2C, `STORE.ALL_PARAMS` streams through the same bulk buffer under the same lock (`Features/control_interfaces_spec.md`).

### 19.2 `STORE.ALL_PARAMS_CHUNK`

WinUSB limits a control transfer to 4096 bytes, so USB hosts on Windows move the structure in pieces. `wValue` is the byte offset and `wLength` the chunk size, 1 to the rest of the structure. It is USB only.

**Reading in chunks.** A GET at offset 0 takes the bulk buffer and snapshots the whole state. Later GETs read that snapshot at any offset. The session ends when a chunk that reaches the end of the structure completes, or when any other USB vendor request arrives.

**Writing in chunks.** Chunks must arrive in order. Offset 0 opens the session and takes the bulk buffer. Each later chunk must start where the previous one ended. When the received count reaches 6136, the header is checked as for `STORE.ALL_PARAMS` and the apply is queued. Any other USB vendor request closes the session and discards what was received.

| Refusal | Result |
|---|---|
| Offset at or past 6136 | `VC_ERR_ADDRESS` |
| A SET chunk that runs past the end, or a zero-length chunk | `VC_ERR_LENGTH` |
| A chunk with no open session, or a SET chunk at the wrong offset | `VC_ERR_STATE`. Restart at offset 0 (a SET refused for its offset keeps the session, so the right offset can be resent) |
| The bulk buffer is held elsewhere when offset 0 arrives | `VC_ERR_BUSY` |
| The final SET chunk completes a structure with a bad header | `VC_ERR_VALUE`, nothing applied |

Spec: `Features/bulk_params_chunking.md`.

### 19.3 `WireBulkParams` layout

Offsets are bytes from the start of the structure. Arrays are sized for RP2350 on both platforms. Rows sized `[17]` (delays, EQ, channel names, crossovers) are indexed by channel (section 4.1). On RP2040 rows 0 and 1 are inputs, rows 2 to 6 are outputs and rows 7 to 16 are zero. Crosspoints always use the `[8][9]` stride.

| Offset | Size | Section | Contents |
|---:|---:|---|---|
| 0 | 16 | `header` | `WireHeader` (19.4) |
| 16 | 16 | `global` | f32 preamp_gain_db (legacy baseline, `preamp` overrides it); u8 bypass; u8 loudness_enabled; u16 loudness_output_mask; f32 loudness_ref_spl; f32 loudness_intensity_pct |
| 32 | 16 | `crossfeed` | u8 enabled; u8 preset; u8 itd_enabled; u8 output_pair_mask; f32 custom_fc; f32 custom_feed_db; u32 reserved |
| 48 | 16 | `legacy` | Retired. Zero on GET, ignored on SET |
| 64 | 68 | `delays` | f32 delay_ms[17], by channel |
| 132 | 576 | `crosspoints` | `[8][9]`, input-major, 8 bytes each: u8 enabled; u8 phase_invert; u8[2] reserved; f32 gain_db |
| 708 | 108 | `outputs` | `[9]`, 12 bytes each: u8 enabled; u8 mute; u8[2] reserved; f32 gain_db; f32 delay_ms |
| 816 | 8 | `pins` | u8 num_pin_outputs (3 or 5); u8 pins[5]; u8[2] reserved |
| 824 | 3264 | `eq` | `WireBandParams[17][12]`, channel-major (19.5) |
| 4088 | 544 | `channel_names` | char[17][32], NUL padded |
| 4632 | 16 | `i2s_config` | u8 output_types[4] (0 S/PDIF, 1 I2S); u8 bck_pin; u8 mck_pin; u8 mck_enabled; u8 mck_multiplier (0 128x, 1 256x); u8 clock_pin_mode_p1 (0 absent, 1 unified, 2 split); u8 bck_pin_slave (0 absent); u8[6] reserved |
| 4648 | 20 | `leveller` | u8 enabled; u8 speed; u8 lookahead; u8 reserved; f32 amount; f32 max_gain_db; f32 gate_threshold_db; u8 detector_mask; u8 apply_mask; u8[2] reserved |
| 4668 | 32 | `preamp` | f32 preamp_db[8], by input channel |
| 4700 | 16 | `master_volume` | f32 master_volume_db (-128 is mute); u8[12] reserved |
| 4716 | 16 | `input_config` | u8 input_source; u8 spdif_rx_pin; u8 i2s_rx_pin; u8 i2s_input_rate (0 44.1k, 1 48k, 2 96k); u8 i2s_input_channels (0 absent); u8 i2s_rx_pin_ext[3]; u8 spdif_rx_pin_ext[3]; u8 spdif_rx_count (1..4, 0 absent); u8 i2s_clock_mode; u8 adat_input_pin (0 absent); u8 input_enable_p1 (bit 0 S/PDIF, bit 1 I2S, bit 2 ADAT, plus one, 0 absent); u8 adat_clock_mode_p1 |
| 4732 | 16 | `lg_sound_sync` | u8 enabled; u8 present; u8 volume; u8 muted; u8[12] reserved. A SET honours only `enabled` |
| 4748 | 16 | `user_volume` | f32 user_volume_db; u8 user_mute; u8[11] reserved |
| 4764 | 16 | `dac_hw_mute` | u8 enabled; u8 active_low; u8 pin; u8 reserved; u16 hold_ms; u16 release_ms; u8[8] reserved |
| 4780 | 1088 | `crossovers` | `WireBandParams[17][4]`, channel-major. Columns are bands 20 to 23. Input rows are zero on GET and skipped on SET |
| 5868 | 8 | `adat_config` | u8 enabled; u8 pin (0 selects the default on SET); u8[6] reserved. Zero on RP2040 |
| 5876 | 24 | `psybass` | u8 enabled; u8 reserved; u16 output_mask; f32 cutoff_hz; f32 harmonics_db; f32 drive_db; f32 character_pct; f32 original_db |
| 5900 | 44 | `upmix` | The `FX.UPMIX_CONFIG` layout. Zero on RP2040 |
| 5944 | 36 | `subharm` | u8 enabled; u8 reserved; u16 output_mask; f32 low_db; f32 high_db; f32 boost_db; f32 top_db; f32 select_depth; f32 select_hold_ms; f32 ceiling_db; u8 select_mode; u8 link_pairs; u8[2] reserved |
| 5980 | 48 | `tube` | u8 enabled; u8 tube_type; u8 rectifier; u8 xfmr_enabled; u16 output_mask; u8[2] reserved; f32 drive_db; f32 bias_pct; f32 asym_db; f32 hardness_pct; f32 sag_pct; f32 xfmr_damping; f32 xfmr_res_hz; f32 mix_pct; f32 trim_db; f32 reserved |
| 6028 | 108 | `limiter` | `[9]`, 12 bytes each: u8 enabled; u8 link_group (0 unlinked, 1 to 4); u8[2] reserved; f32 threshold_db; f32 release_ms |

Fields named `*_p1` hold the value plus one, so 0 means "absent, keep the live value" on a SET.

### 19.4 `WireHeader` (16 bytes)

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | `format_version` (32). A SET must match |
| 1 | u8 | `platform_id`: 0 RP2040, 1 RP2350. A SET must match the device |
| 2 | u8 | `num_channels` (7 or 17). A SET must match |
| 3 | u8 | `num_output_channels` (5 or 9). A SET must match |
| 4 | u8 | `num_input_channels` (2 or 8). A SET must match |
| 5 | u8 | `max_bands` (12) |
| 6 | u16 | `payload_length` (6136). A SET must match |
| 8 | u16 | Firmware major (GET only) |
| 10 | u16 | Firmware minor (GET only) |
| 12 | u32 | Reserved, 0 |

### 19.5 `WireBandParams` (16 bytes)

The record used by `eq` and `crossovers`. The band is implied by its position in the array.

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Filter type (section 4.7) |
| 1 | u8 | Bypass, 1 bypasses |
| 2 | u16 | Linkwitz Transform Qp x 512 for an EQ band of type 11, else 0 |
| 4 | f32 | Frequency, Hz |
| 8 | f32 | Q |
| 12 | f32 | Gain, dB |

The bulk bytes are authoritative. A SET always overwrites the stored Qp, which is 0 (the 0.707 default) for every type other than 11. Notifications for one band carry this same 16-byte record at the band's offset.

---

## 20. Stateful and flash operations in detail

The commands below write flash, rewrite much of the live state, reconfigure physical I/O or leave the application. They are the ones a host is most likely to get wrong.

### 20.1 The three shared mechanisms

**The deferred pattern.** The handler validates the request, records it in a pending flag or a pending slot, and returns `VC_OK`. The main loop does the work. Operations that restart the outputs also wait until any DAC hardware mute hold has elapsed. Between "accepted" and "applied" there is a busy window (section 1.6).

**The flash bracket.** Flash is erased and programmed one 4 KB sector at a time. Each sector stops core 1 and masks interrupts for the erase and program, about 45 ms, so USB control transfers are not serviced. The stored region at the top of flash is one directory sector followed by 10 preset slots. A slot spans two sectors on RP2350 and one on RP2040. The directory holds the slot occupancy, the slot names, the startup policy, the last active slot, the device and source names, the Control Surfaces configuration and every device-wide setting (the independent master volume and IO configuration, the DAC mute configuration, the transport configuration, the output latency, the modes). The slots hold the per-preset DSP state.

**The muted, synchronized restart.** Any change that could disturb output timing is bracketed by a soft mute of about 5 ms and ends by restarting every output slot together. This is how the device keeps all S/PDIF instances, the I2S slots and PDM sample-aligned across the change. A host must never try to reconfigure a subset of slots by some other route to avoid the mute.

### 20.2 Bulk apply

`STORE.ALL_PARAMS` SET drains the USB audio ring, mutes, suspends the S/PDIF and I2S receivers, applies the structure, recomputes every filter and delay, re-applies MCK and the core 1 mode, switches any slot whose output type changed or restarts all slots, and restarts the receivers. It writes no flash (section 19.1).

### 20.3 Device-wide saves and modes

`ROUTE.MASTER_VOLUME_SAVE` and `STORE.OUTPUT_CONFIG_SAVE` copy the live master volume or the live IO configuration into the directory. `ROUTE.MASTER_VOLUME_MODE` and `STORE.OUTPUT_CONFIG_MODE` set whether presets carry those values. Each is a one-sector directory write. `ROUTE.MASTER_VOLUME_SAVED` reads the stored master volume, which differs from the live `ROUTE.MASTER_VOLUME` until it is saved.

### 20.4 Preset operations

- `STORE.PRESET_SAVE` collects the live state, writes the slot body, then updates the directory (occupancy and last active). That is the slot's sectors plus the directory.
- `STORE.PRESET_LOAD` is a read and a full muted restart, not a slot write. It drains, mutes, suspends the receivers, reads the slot (or applies factory defaults for an empty slot), recomputes filters and delays, zeroes every delay line so audio from the previous preset cannot bleed through, re-derives the core 1 mode, switches output types or restarts all slots, restarts the receivers, and finally writes the directory to record the last active slot. It then sends `PRESET_LOADED` and `BULK_INVALIDATED`.
- `STORE.PRESET_DELETE` erases the slot and clears its occupancy bit and name in the directory.
- `STORE.PRESET_NAME` and `STORE.STARTUP` are one-sector directory writes.

### 20.5 Pin moves

`OUTPUT.PIN`, `INPUT.SPDIF_RX_PIN`, `INPUT.I2S_RX_PIN`, `INPUT.ADAT_RX_PIN`, `OUTPUT.ADAT_TX_PIN`, `CLOCK.I2S_BCK_PIN` and `CLOCK.MCK_PIN` write no flash. A moved pin persists when it is saved with the IO configuration (a preset save in IO config mode 1, or `STORE.OUTPUT_CONFIG_SAVE` in mode 0). An output slot pin moves inside the synchronized restart of every slot. An input pin moves by restarting that receiver, with a brief silence.

### 20.6 Output configuration

`OUTPUT.TYPE` tears down and rebuilds the slot's S/PDIF or I2S output inside its own synchronized restart. `CLOCK.MCK_ENABLE`, `CLOCK.MCK_PIN` and `CLOCK.MCK_MULTIPLIER` act at once with no restart. `OUTPUT.DAC_MUTE_CONFIG` is validated, applied and written to the directory by the main loop. It is the one output setting that persists on its own. `OUTPUT.DAC_MUTE_TEST` pulses the pin for about a second and changes nothing.

### 20.7 Factory reset

`STORE.FACTORY_RESET` is a live-state reset under a full muted restart and writes no flash (section 7.3).

### 20.8 The persistence model

| Kind | Commands |
|---|---|
| Live only, lost at reboot unless saved | Every ordinary SET (EQ, routing, gains, delays, dynamics, effects), every pin, type and MCK change, `STORE.ALL_PARAMS`, `STORE.FACTORY_RESET`, the Control Surfaces preview SETs and aux states |
| Writes flash itself | `STORE.PRESET_SAVE`, `STORE.PRESET_DELETE`, `STORE.PRESET_NAME`, `STORE.STARTUP`, `STORE.OUTPUT_CONFIG_MODE`, `STORE.OUTPUT_CONFIG_SAVE`, `ROUTE.MASTER_VOLUME_MODE`, `ROUTE.MASTER_VOLUME_SAVE`, `SYS.DEVICE_NAME`, `INPUT.SOURCE_NAME`, `OUTPUT.LATENCY`, `OUTPUT.DAC_MUTE_CONFIG`, `XPORT.UART_CONFIG`, `XPORT.I2C_CONFIG`, `CS.SAVE` |
| Reads flash and records the last active slot | `STORE.PRESET_LOAD` |

Live state reaches flash only through a preset save, or through the device-wide saves for the settings that are independent of presets. A typical "commit the current tuning" flow is to push the changes, confirm them by readback, then `STORE.PRESET_SAVE` to the active slot.

### 20.9 Bootloader

`SYS.ENTER_BOOTLOADER` jumps to the ROM bootloader after the reply (section 6.4). It does not write flash. After it, the host must release the device handle and send nothing more.

---

## 21. Notifications

The device pushes change and event notifications on bulk IN endpoint `0x83` of interface 2, 64-byte packets. A host that wants live updates reads this endpoint. A host that does not can poll the readback commands instead. With nothing queued, a 1-byte idle packet `00` goes out only after 100 ms of quiet.

Every packet except the idle and the legacy master volume packet starts `[0x02, event, flags, seq]`. All multi-byte fields are little-endian.

| Event | Name | Length | Layout |
|---|---|---:|---|
| `0x00` | `IDLE` | 1 | `[0x00]`, discard |
| `0x01` | `MASTER_VOLUME` (legacy, USB only) | 8 | `[0x01, 0, 0, 0, f32 dB]` |
| `0x02` | `PARAM_CHANGED` | 12 + size | `[2, 0x02, 0, seq, offset u16, size u16, source, 0, 0, 0, value]`, size at most 52 |
| `0x03` | `BULK_INVALIDATED` | 8 | `[2, 0x03, 0, seq, source, 0, 0, 0]` |
| `0x04` | `PRESET_LOADED` | 8 | `[2, 0x04, 0, seq, slot, 0, 0, 0]`, followed by `0x03` |
| `0x05` | `INPUT_FORMAT` | 8 | `[2, 0x05, 0, seq, channels, 0, 0, 0]`, the active input channel count changed |
| `0x06` | reserved | | |
| `0x07` | `SIGGEN_STATE` | 8 | `[2, 0x07, 0, seq, state, reason, signal_type, channel]`, channel is the WALK output or `0xFF` |
| `0x08` | `ADAT_STATE` (RP2350) | 8 | `[2, 0x08, 0, seq, enabled, active, pin, 0]` |
| `0x09` | `I2S_SLAVE_STATE` | 9 | `[2, 0x09, 0, seq, state, rate u32]`, rate 0 until locked |
| `0x0A` | `CS_IR_LEARN` | 12 | `[2, 0x0A, 0, seq, state, protocol, 0, 0, code u32]`, state 2 done or 3 timeout |
| `0x0B` | `ADAT_INPUT_STATE` (RP2350) | 10 | `[2, 0x0B, 0, seq, state, rate u32, clock_mode]`, rate 0 unless locked |
| `0x0C` | `CS_AUX` | 9 | `[2, 0x0C, 0, seq, slot, state, level u16 (8.8 percent), source]` |
| `0x0D` | `RTA_CONFIG` | 17 | `[2, 0x0D, 0, seq, RtaConfig (12 bytes, as applied), source]` |
| `0x0E` | reserved | | |
| `0x0F` | `INPUT_SOURCE_NAME` | 40 | `[2, 0x0F, 0, seq, source_id, source, 0, 0, name (32)]`, all zero when cleared |
| `0x10` | `DEVICE_NAME` | 40 | `[2, 0x10, 0, seq, 0, source, 0, 0, name (32)]` |

`PARAM_CHANGED` names the changed field by its byte offset in `WireBulkParams` (section 19.3) and carries its new bytes, so a host can keep a mirror of the bulk structure and dispatch on the offset with no per-command switch. Notifications never name a command.

The source byte says where a change came from.

| Source | Origin |
|---|---|
| 0 | Unknown |
| 1 | A USB vendor SET |
| 2 | `STORE.ALL_PARAMS` SET |
| 3 | A preset load |
| 4 | A factory reset |
| 5 | Control Surfaces (a physical control) |
| 6 | The firmware itself (a clamp, a recalculation) |
| 7 | The host's UAC1 volume or mute control |
| 8 | The UART transport |
| 9 | The I2C transport |

On `BULK_INVALIDATED` or `PRESET_LOADED`, re-read `STORE.ALL_PARAMS`. A gap in `seq` means events were dropped for this consumer, and the same re-read recovers. The same v2 packets can be delivered over UART as type `0x40` frames when `XPORT.UART_CONFIG` enables it. The legacy master volume packet is USB only. The I2C target cannot start a transfer, so it has no notification channel. Spec: `Features/notification_protocol_v2_spec.md`.

---

## 22. UART and I2C

The UART and the I2C target carry exactly the same commands as USB. A client builds the same `bmRequestType`, `bRequest`, `wValue`, `wIndex` and payload it would send over USB and wraps them in a frame. The page is in the high byte of `wIndex` as on USB, and the low byte is ignored. The framing, CRC, timing and electrical details are in `Features/control_interfaces_spec.md`.

- **Status byte.** Every response starts with one status byte. `0x00` is success. `0x01` to `0x07` are transport conditions, and the command did not run. They are `0x01` busy, `0x04` bulk buffer held, `0x05` CRC error, `0x06` payload too large and `0x07` bad frame. `0x10` and up is the command's own result code (section 1.5), the same one a USB host reads from `SYS.LAST_RESULT`. Any non-zero status is a failure. Only `0x01`, `0x04` and `0x17` are worth retrying unchanged. Codes `0x02` and `0x03` are never sent by firmware 1.1.7, and old firmware answers the `SYS.PROTOCOL_INFO` probe with `0x02`.
- **Sizes.** SET payloads are at most 64 bytes and GET replies at most 256 bytes, delivered whole, except `STORE.ALL_PARAMS`, which streams through the bulk buffer. A GET's `wLength` caps the reply as on USB, but `wLength` 0 means "the whole reply".
- **USB only.** A few requests answer `VC_ERR_BLOCKED` (`0x16`) on UART and I2C. They are the `XPORT.UART_CONFIG` and `XPORT.I2C_CONFIG` SETs, both directions of `STORE.ALL_PARAMS_CHUNK` and `MEAS.RTA_BANDS_ALL`. `SYS.COMMAND_INFO` flag bits 0 and 1 mark them.
- **Bootloader.** `SYS.ENTER_BOOTLOADER` reboots once the response frame has gone out.
- **`SYS.LAST_RESULT`** is kept per transport, so a UART client reads its own.

---

## 23. Write to readback map

Every setting that can be written can be read back with the same command, by rule 7 of section 3. This table lists the cases where the readback is somewhere else, or where it needs care.

| Write | Readback |
|---|---|
| `STORE.PRESET_SAVE`, `STORE.PRESET_DELETE` | `STORE.DIRECTORY` (occupancy, last active) |
| `STORE.PRESET_LOAD` | `STORE.PRESET_ACTIVE`, then `STORE.ALL_PARAMS` on `BULK_INVALIDATED` |
| `STORE.OUTPUT_CONFIG_SAVE` | Nothing direct. The stored IO configuration applies at boot and on factory reset |
| `STORE.FACTORY_RESET` | `STORE.ALL_PARAMS` on `BULK_INVALIDATED` |
| `STORE.ALL_PARAMS` SET | `STORE.ALL_PARAMS` GET after the busy window |
| `ROUTE.MASTER_VOLUME_SAVE` | `ROUTE.MASTER_VOLUME_SAVED` |
| `ROUTE.CHANNEL_DELAY` on an output channel | `ROUTE.CHANNEL_DELAY` (`ROUTE.OUTPUT_DELAY` is not updated) |
| `INPUT.SOURCE` | `INPUT.SOURCE` after the switch, or `INPUT.STATUS` |
| `INPUT.SPDIF_RX_ENABLE` | `INPUT.SPDIF_RX_ENABLE` or `INPUT.SPDIF_RX_CONFIG` |
| `INPUT.ADAT_RX_CLOCK_MODE`, `CLOCK.I2S_CLOCK_MODE` | The same command, once the main loop has applied it |
| `OUTPUT.TYPE` | `OUTPUT.TYPE`, once the switch has run |
| `OUTPUT.LATENCY` | `OUTPUT.LATENCY` at once, `OUTPUT.LATENCY_STATUS` byte 8 clears when applied |
| `OUTPUT.DAC_MUTE_CONFIG` | `OUTPUT.DAC_MUTE_CONFIG`. Compare: a refused config is dropped |
| `XPORT.UART_CONFIG`, `XPORT.I2C_CONFIG` | `XPORT.STATUS`, then the config GET |
| `FILTER.BAND`, `FILTER.BAND_BYPASS` | The same command, once applied (a millisecond or two) |
| `FX.TUBE_TYPE`, `FX.TUBE_BIAS`, `FX.TUBE_ASYM`, `FX.TUBE_HARDNESS`, `FX.TUBE_SAG` | All five, since each can change the others |
| `FX.UPMIX_*` float fields | Read back unclamped (section 13.5) |
| `MEAS.RTA_CONFIG` | `MEAS.RTA_CONFIG` once applied, or notification `RTA_CONFIG` |
| `MEAS.SIGGEN_START`, `MEAS.SIGGEN_STOP` | `MEAS.SIGGEN_STATUS` |
| `CS.*` preview SETs, `CS.SAVE`, `CS.REVERT` | `CS.STATUS` byte 0 and 1, then the matching GET |
| `CS.IR_LEARN_ARM` | `CS.IR_LEARN_STATUS`, or notification `CS_IR_LEARN` |
| `CS.MACRO_FIRE` | `CS.EXT_STATUS` (running macro and step) |
| `SYS.STATS_RESET` | The watermark fields of `SYS.BUFFER_STATS`, `SYS.PIPELINE_STATS`, `SYS.USB_AUDIO_STATUS` |

---

## 24. Commands to treat carefully

These commands change physical I/O, claim GPIOs, write flash, restart the audio pipeline or leave the application. A host should gate them behind explicit user intent, rate-limit them, and read back to confirm.

| Command | Why |
|---|---|
| `SYS.ENTER_BOOTLOADER` | Leaves the application. The device disappears until it is reflashed |
| `STORE.FACTORY_RESET` | Replaces the whole live state and switches the input to USB |
| `STORE.ALL_PARAMS` SET | Replaces the whole live state under a restart. Large transfer |
| `STORE.PRESET_SAVE`, `STORE.PRESET_DELETE` | Flash writes of several sectors |
| `STORE.PRESET_LOAD` | Replaces the live state under a restart |
| Every other `FLASH` command | A flash blackout and an output restart |
| `OUTPUT.TYPE` | Rebuilds an output slot, restarts every slot |
| `OUTPUT.PIN` | Moves an output data pin, restarts every slot. The PDM pin needs the PDM output disabled |
| `OUTPUT.LATENCY` | Flash write and a restart of every output |
| `OUTPUT.ADAT_TX_ENABLE`, `OUTPUT.ADAT_TX_PIN` | Claims a GPIO, restarts every output |
| `OUTPUT.DAC_MUTE_CONFIG` | Claims a GPIO. A bad config is dropped later, so it must be read back |
| `OUTPUT.DAC_MUTE_TEST` | Mutes the DAC for about a second |
| `INPUT.SOURCE` | Full pipeline reset |
| `INPUT.SPDIF_RX_PIN`, `INPUT.SPDIF_RX_ENABLE`, `INPUT.I2S_RX_PIN`, `INPUT.I2S_RX_CHANNELS` | Claim or release GPIOs, restart a live receiver |
| `INPUT.I2S_RX_RATE` | Changes the device rate while I2S or ADAT is the master |
| `INPUT.ADAT_RX_ENABLE`, `INPUT.ADAT_RX_PIN`, `INPUT.ADAT_RX_CLOCK_MODE` | Claim a GPIO, restart a live receiver |
| `CLOCK.I2S_BCK_PIN`, `CLOCK.I2S_CLOCK_PIN_MODE`, `CLOCK.I2S_CLOCK_MODE` | Move or re-clock the shared I2S clock pair |
| `CLOCK.MCK_ENABLE`, `CLOCK.MCK_PIN`, `CLOCK.MCK_MULTIPLIER` | Drive a clock onto a GPIO and into an external DAC |
| `XPORT.UART_CONFIG`, `XPORT.I2C_CONFIG` | Claim GPIOs, can take down an external controller's link |
| `CS.BINDING`, `CS.IR_CMD` | Claim GPIOs and PWM slices (a preview until `CS.SAVE`) |
| `CS.SAVE` | Flash write |

Inter-slot sample alignment is a hard product invariant. The firmware keeps every output slot aligned across all of these operations by routing them through the muted, synchronized restart of section 20.1. A host must use these commands as defined and must not try to reconfigure part of the output bus by another route.

---

## 25. Coming from firmware 1.1.6

Firmware 1.1.7 replaced the flat opcode space of 1.1.6 with the paged protocol in this document. The old opcode numbers are gone. A request with a zero `wIndex` high byte reaches page 0 and is refused. Many commands also changed shape, because values moved out of `wValue` into the payload and write-as-read commands became SETs. A host written for 1.1.6 should follow the host migration guide, `Documentation/Features/vendor_protocol_migration_guide.md`. It covers the probe a host must send first, the old-to-new map for every opcode, every reply and refusal that changed, per-transport notes and a porting checklist. The machine-readable map from every old opcode to its new command is `Documentation/protocol/legacy_map.json`.

---

## 26. Command index

Every command, sorted by name and then by page, with its 16-bit ID. Each command links to its page's section. Commands marked with a build condition exist only in some builds (section 2.5).

<!-- VC:BEGIN INDEX -->

| Name | Command | ID | Availability |
|---|---|---|---|
| ADAT_RX_CLOCK_MODE | [`INPUT.ADAT_RX_CLOCK_MODE`](#8-input-0x03) | `0x0342` | RP2350 only |
| ADAT_RX_ENABLE | [`INPUT.ADAT_RX_ENABLE`](#8-input-0x03) | `0x0340` | RP2350 only |
| ADAT_RX_PIN | [`INPUT.ADAT_RX_PIN`](#8-input-0x03) | `0x0341` | RP2350 only |
| ADAT_RX_PIN_AVAIL | [`INPUT.ADAT_RX_PIN_AVAIL`](#8-input-0x03) | `0x0344` | RP2350 only |
| ADAT_RX_STATUS | [`INPUT.ADAT_RX_STATUS`](#8-input-0x03) | `0x0343` | RP2350 only |
| ADAT_TX_ENABLE | [`OUTPUT.ADAT_TX_ENABLE`](#9-output-0x04) | `0x0430` | RP2350 only |
| ADAT_TX_PIN | [`OUTPUT.ADAT_TX_PIN`](#9-output-0x04) | `0x0431` | RP2350 only |
| ADAT_TX_PIN_AVAIL | [`OUTPUT.ADAT_TX_PIN_AVAIL`](#9-output-0x04) | `0x0433` | RP2350 only |
| ADAT_TX_STATUS | [`OUTPUT.ADAT_TX_STATUS`](#9-output-0x04) | `0x0432` | RP2350 only |
| ALL_PARAMS | [`STORE.ALL_PARAMS`](#7-store-0x02) | `0x0240` |  |
| ALL_PARAMS_CHUNK | [`STORE.ALL_PARAMS_CHUNK`](#7-store-0x02) | `0x0241` |  |
| AUX_ALL | [`CS.AUX_ALL`](#15-cs-0x0a) | `0x0A62` |  |
| AUX_LEVEL | [`CS.AUX_LEVEL`](#15-cs-0x0a) | `0x0A61` |  |
| AUX_STATE | [`CS.AUX_STATE`](#15-cs-0x0a) | `0x0A60` |  |
| BAND | [`FILTER.BAND`](#11-filter-0x06) | `0x0610` |  |
| BAND_BYPASS | [`FILTER.BAND_BYPASS`](#11-filter-0x06) | `0x0611` |  |
| BINDING | [`CS.BINDING`](#15-cs-0x0a) | `0x0A20` |  |
| BINDING_PIN_AVAIL | [`CS.BINDING_PIN_AVAIL`](#15-cs-0x0a) | `0x0A22` |  |
| BUFFER_STATS | [`SYS.BUFFER_STATS`](#6-sys-0x01) | `0x0142` |  |
| BUILD_INFO | [`SYS.BUILD_INFO`](#6-sys-0x01) | `0x0122` |  |
| CAPS | [`CS.CAPS`](#15-cs-0x0a) | `0x0A10` |  |
| CHANNEL_DELAY | [`ROUTE.CHANNEL_DELAY`](#10-route-0x05) | `0x0530` |  |
| CHANNEL_NAME | [`ROUTE.CHANNEL_NAME`](#10-route-0x05) | `0x0531` |  |
| CLIP_CLEAR | [`MEAS.CLIP_CLEAR`](#14-meas-0x09) | `0x0912` |  |
| CLIP_FLAGS | [`MEAS.CLIP_FLAGS`](#14-meas-0x09) | `0x0911` |  |
| CLOCK_DIAG | [`DIAG.CLOCK_DIAG`](#18-diag-0xf0) | `0xF010` | `DSPI_CLOCK_DIAG` |
| COMMAND_INFO | [`SYS.COMMAND_INFO`](#6-sys-0x01) | `0x0113` |  |
| CORE1_MODE | [`SYS.CORE1_MODE`](#6-sys-0x01) | `0x0123` |  |
| CROSSFEED_ENABLE | [`FX.CROSSFEED_ENABLE`](#13-fx-0x08) | `0x0810` |  |
| CROSSFEED_FEED | [`FX.CROSSFEED_FEED`](#13-fx-0x08) | `0x0813` |  |
| CROSSFEED_FREQ | [`FX.CROSSFEED_FREQ`](#13-fx-0x08) | `0x0812` |  |
| CROSSFEED_ITD | [`FX.CROSSFEED_ITD`](#13-fx-0x08) | `0x0814` |  |
| CROSSFEED_OUTPUTS | [`FX.CROSSFEED_OUTPUTS`](#13-fx-0x08) | `0x0815` |  |
| CROSSFEED_PRESET | [`FX.CROSSFEED_PRESET`](#13-fx-0x08) | `0x0811` |  |
| DAC_MUTE_CONFIG | [`OUTPUT.DAC_MUTE_CONFIG`](#9-output-0x04) | `0x0440` |  |
| DAC_MUTE_PIN_AVAIL | [`OUTPUT.DAC_MUTE_PIN_AVAIL`](#9-output-0x04) | `0x0442` |  |
| DAC_MUTE_TEST | [`OUTPUT.DAC_MUTE_TEST`](#9-output-0x04) | `0x0441` |  |
| DEVICE_NAME | [`SYS.DEVICE_NAME`](#6-sys-0x01) | `0x0124` |  |
| DIRECTORY | [`STORE.DIRECTORY`](#7-store-0x02) | `0x0215` |  |
| DISPLAY_CAPS | [`CS.DISPLAY_CAPS`](#15-cs-0x0a) | `0x0A71` |  |
| DISPLAY_CONFIG | [`CS.DISPLAY_CONFIG`](#15-cs-0x0a) | `0x0A70` |  |
| DISPLAY_PAGE | [`CS.DISPLAY_PAGE`](#15-cs-0x0a) | `0x0A72` |  |
| DISPLAY_STATUS | [`CS.DISPLAY_STATUS`](#15-cs-0x0a) | `0x0A73` |  |
| ENTER_BOOTLOADER | [`SYS.ENTER_BOOTLOADER`](#6-sys-0x01) | `0x0130` |  |
| EQ_BYPASS | [`FILTER.EQ_BYPASS`](#11-filter-0x06) | `0x0612` |  |
| EQ_PROF | [`DIAG.EQ_PROF`](#18-diag-0xf0) | `0xF040` | `DSPI_EQ_PROF` |
| EQ_PROF_RESET_MIN | [`DIAG.EQ_PROF_RESET_MIN`](#18-diag-0xf0) | `0xF041` | `DSPI_EQ_PROF` |
| EXT_STATUS | [`CS.EXT_STATUS`](#15-cs-0x0a) | `0x0A13` |  |
| FACTORY_RESET | [`STORE.FACTORY_RESET`](#7-store-0x02) | `0x0230` |  |
| GROUP | [`CS.GROUP`](#15-cs-0x0a) | `0x0A40` |  |
| I2C_CONFIG | [`XPORT.I2C_CONFIG`](#16-xport-0x0b) | `0x0B11` |  |
| I2C_PIN_AVAIL | [`XPORT.I2C_PIN_AVAIL`](#16-xport-0x0b) | `0x0B14` |  |
| I2S_BCK_PIN | [`CLOCK.I2S_BCK_PIN`](#17-clock-0x0c) | `0x0C10` |  |
| I2S_BCK_PIN_AVAIL | [`CLOCK.I2S_BCK_PIN_AVAIL`](#17-clock-0x0c) | `0x0C14` |  |
| I2S_CLOCK_MODE | [`CLOCK.I2S_CLOCK_MODE`](#17-clock-0x0c) | `0x0C12` |  |
| I2S_CLOCK_PIN_MODE | [`CLOCK.I2S_CLOCK_PIN_MODE`](#17-clock-0x0c) | `0x0C11` |  |
| I2S_RX_CHANNELS | [`INPUT.I2S_RX_CHANNELS`](#8-input-0x03) | `0x0331` |  |
| I2S_RX_ENABLE | [`INPUT.I2S_RX_ENABLE`](#8-input-0x03) | `0x0334` |  |
| I2S_RX_PIN | [`INPUT.I2S_RX_PIN`](#8-input-0x03) | `0x0330` |  |
| I2S_RX_PIN_AVAIL | [`INPUT.I2S_RX_PIN_AVAIL`](#8-input-0x03) | `0x0333` |  |
| I2S_RX_RATE | [`INPUT.I2S_RX_RATE`](#8-input-0x03) | `0x0332` |  |
| I2S_SLAVE_STATUS | [`CLOCK.I2S_SLAVE_STATUS`](#17-clock-0x0c) | `0x0C13` |  |
| IR_CMD | [`CS.IR_CMD`](#15-cs-0x0a) | `0x0A30` |  |
| IR_LEARN_ARM | [`CS.IR_LEARN_ARM`](#15-cs-0x0a) | `0x0A32` |  |
| IR_LEARN_CANCEL | [`CS.IR_LEARN_CANCEL`](#15-cs-0x0a) | `0x0A33` |  |
| IR_LEARN_STATUS | [`CS.IR_LEARN_STATUS`](#15-cs-0x0a) | `0x0A31` |  |
| LAST_RESULT | [`SYS.LAST_RESULT`](#6-sys-0x01) | `0x0114` |  |
| LATENCY | [`OUTPUT.LATENCY`](#9-output-0x04) | `0x0420` |  |
| LATENCY_OPTIONS | [`OUTPUT.LATENCY_OPTIONS`](#9-output-0x04) | `0x0422` |  |
| LATENCY_STATUS | [`OUTPUT.LATENCY_STATUS`](#9-output-0x04) | `0x0421` |  |
| LEVELLER_AMOUNT | [`DYN.LEVELLER_AMOUNT`](#12-dyn-0x07) | `0x0721` |  |
| LEVELLER_ENABLE | [`DYN.LEVELLER_ENABLE`](#12-dyn-0x07) | `0x0720` |  |
| LEVELLER_GATE | [`DYN.LEVELLER_GATE`](#12-dyn-0x07) | `0x0725` |  |
| LEVELLER_LOOKAHEAD | [`DYN.LEVELLER_LOOKAHEAD`](#12-dyn-0x07) | `0x0724` |  |
| LEVELLER_MASKS | [`DYN.LEVELLER_MASKS`](#12-dyn-0x07) | `0x0726` |  |
| LEVELLER_MAX_GAIN | [`DYN.LEVELLER_MAX_GAIN`](#12-dyn-0x07) | `0x0723` |  |
| LEVELLER_SPEED | [`DYN.LEVELLER_SPEED`](#12-dyn-0x07) | `0x0722` |  |
| LEVELS | [`MEAS.LEVELS`](#14-meas-0x09) | `0x0910` |  |
| LG_SOUND_SYNC_ENABLE | [`INPUT.LG_SOUND_SYNC_ENABLE`](#8-input-0x03) | `0x0350` |  |
| LG_SOUND_SYNC_STATUS | [`INPUT.LG_SOUND_SYNC_STATUS`](#8-input-0x03) | `0x0351` |  |
| LIMITER_ENABLE | [`DYN.LIMITER_ENABLE`](#12-dyn-0x07) | `0x0710` |  |
| LIMITER_LINK_GROUP | [`DYN.LIMITER_LINK_GROUP`](#12-dyn-0x07) | `0x0713` |  |
| LIMITER_METERS | [`DYN.LIMITER_METERS`](#12-dyn-0x07) | `0x0718` |  |
| LIMITER_RELEASE | [`DYN.LIMITER_RELEASE`](#12-dyn-0x07) | `0x0712` |  |
| LIMITER_STATUS | [`DYN.LIMITER_STATUS`](#12-dyn-0x07) | `0x0719` |  |
| LIMITER_THRESHOLD | [`DYN.LIMITER_THRESHOLD`](#12-dyn-0x07) | `0x0711` |  |
| LOOPBACK_STATS | [`DIAG.LOOPBACK_STATS`](#18-diag-0xf0) | `0xF020` | `DSPI_LOOPBACK` |
| LOOP_PROF_BG_PACE | [`DIAG.LOOP_PROF_BG_PACE`](#18-diag-0xf0) | `0xF033` | `DSPI_LOOP_PROF` |
| LOOP_PROF_COMMAND | [`DIAG.LOOP_PROF_COMMAND`](#18-diag-0xf0) | `0xF032` | `DSPI_LOOP_PROF` |
| LOOP_PROF_JOB | [`DIAG.LOOP_PROF_JOB`](#18-diag-0xf0) | `0xF031` | `DSPI_LOOP_PROF` |
| LOOP_PROF_RESET | [`DIAG.LOOP_PROF_RESET`](#18-diag-0xf0) | `0xF034` | `DSPI_LOOP_PROF` |
| LOOP_PROF_SECTION | [`DIAG.LOOP_PROF_SECTION`](#18-diag-0xf0) | `0xF030` | `DSPI_LOOP_PROF` |
| LOUDNESS_ENABLE | [`DYN.LOUDNESS_ENABLE`](#12-dyn-0x07) | `0x0730` |  |
| LOUDNESS_INTENSITY | [`DYN.LOUDNESS_INTENSITY`](#12-dyn-0x07) | `0x0732` |  |
| LOUDNESS_MASK | [`DYN.LOUDNESS_MASK`](#12-dyn-0x07) | `0x0733` |  |
| LOUDNESS_REF | [`DYN.LOUDNESS_REF`](#12-dyn-0x07) | `0x0731` |  |
| MACRO | [`CS.MACRO`](#15-cs-0x0a) | `0x0A50` |  |
| MACRO_CANCEL | [`CS.MACRO_CANCEL`](#15-cs-0x0a) | `0x0A59` |  |
| MACRO_FIRE | [`CS.MACRO_FIRE`](#15-cs-0x0a) | `0x0A58` |  |
| MACRO_HEADER | [`CS.MACRO_HEADER`](#15-cs-0x0a) | `0x0A51` |  |
| MACRO_STEP | [`CS.MACRO_STEP`](#15-cs-0x0a) | `0x0A52` |  |
| MASTER_VOLUME | [`ROUTE.MASTER_VOLUME`](#10-route-0x05) | `0x0550` |  |
| MASTER_VOLUME_MODE | [`ROUTE.MASTER_VOLUME_MODE`](#10-route-0x05) | `0x0551` |  |
| MASTER_VOLUME_SAVE | [`ROUTE.MASTER_VOLUME_SAVE`](#10-route-0x05) | `0x0552` |  |
| MASTER_VOLUME_SAVED | [`ROUTE.MASTER_VOLUME_SAVED`](#10-route-0x05) | `0x0553` |  |
| MATRIX_ROUTE | [`ROUTE.MATRIX_ROUTE`](#10-route-0x05) | `0x0510` |  |
| MCK_ENABLE | [`CLOCK.MCK_ENABLE`](#17-clock-0x0c) | `0x0C20` |  |
| MCK_MULTIPLIER | [`CLOCK.MCK_MULTIPLIER`](#17-clock-0x0c) | `0x0C22` |  |
| MCK_PIN | [`CLOCK.MCK_PIN`](#17-clock-0x0c) | `0x0C21` |  |
| MCK_PIN_AVAIL | [`CLOCK.MCK_PIN_AVAIL`](#17-clock-0x0c) | `0x0C23` |  |
| NOUN_DESC | [`CS.NOUN_DESC`](#15-cs-0x0a) | `0x0A11` |  |
| OUTPUT_CONFIG_MODE | [`STORE.OUTPUT_CONFIG_MODE`](#7-store-0x02) | `0x0220` |  |
| OUTPUT_CONFIG_SAVE | [`STORE.OUTPUT_CONFIG_SAVE`](#7-store-0x02) | `0x0221` |  |
| OUTPUT_DELAY | [`ROUTE.OUTPUT_DELAY`](#10-route-0x05) | `0x0523` |  |
| OUTPUT_ENABLE | [`ROUTE.OUTPUT_ENABLE`](#10-route-0x05) | `0x0520` |  |
| OUTPUT_ENABLE_CONFLICT | [`ROUTE.OUTPUT_ENABLE_CONFLICT`](#10-route-0x05) | `0x0524` |  |
| OUTPUT_GAIN | [`ROUTE.OUTPUT_GAIN`](#10-route-0x05) | `0x0521` |  |
| OUTPUT_MUTE | [`ROUTE.OUTPUT_MUTE`](#10-route-0x05) | `0x0522` |  |
| PAGE_COMMANDS | [`SYS.PAGE_COMMANDS`](#6-sys-0x01) | `0x0112` |  |
| PAGE_LIST | [`SYS.PAGE_LIST`](#6-sys-0x01) | `0x0111` |  |
| PDM_STATS | [`SYS.PDM_STATS`](#6-sys-0x01) | `0x0144` |  |
| PIN | [`OUTPUT.PIN`](#9-output-0x04) | `0x0411` |  |
| PIN_AVAIL | [`OUTPUT.PIN_AVAIL`](#9-output-0x04) | `0x0412` |  |
| PIN_CLAIMS | [`SYS.PIN_CLAIMS`](#6-sys-0x01) | `0x0151` |  |
| PIN_STATE | [`SYS.PIN_STATE`](#6-sys-0x01) | `0x0150` |  |
| PIPELINE_STATS | [`SYS.PIPELINE_STATS`](#6-sys-0x01) | `0x0143` |  |
| PLATFORM | [`SYS.PLATFORM`](#6-sys-0x01) | `0x0120` |  |
| PREAMP | [`ROUTE.PREAMP`](#10-route-0x05) | `0x0540` |  |
| PRESET_ACTIVE | [`STORE.PRESET_ACTIVE`](#7-store-0x02) | `0x0214` |  |
| PRESET_DELETE | [`STORE.PRESET_DELETE`](#7-store-0x02) | `0x0212` |  |
| PRESET_LOAD | [`STORE.PRESET_LOAD`](#7-store-0x02) | `0x0211` |  |
| PRESET_NAME | [`STORE.PRESET_NAME`](#7-store-0x02) | `0x0213` |  |
| PRESET_SAVE | [`STORE.PRESET_SAVE`](#7-store-0x02) | `0x0210` |  |
| PROTOCOL_INFO | [`SYS.PROTOCOL_INFO`](#6-sys-0x01) | `0x0110` |  |
| PSYBASS_CHARACTER | [`FX.PSYBASS_CHARACTER`](#13-fx-0x08) | `0x0844` |  |
| PSYBASS_CUTOFF | [`FX.PSYBASS_CUTOFF`](#13-fx-0x08) | `0x0841` |  |
| PSYBASS_DRIVE | [`FX.PSYBASS_DRIVE`](#13-fx-0x08) | `0x0843` |  |
| PSYBASS_ENABLE | [`FX.PSYBASS_ENABLE`](#13-fx-0x08) | `0x0840` |  |
| PSYBASS_HARMONICS | [`FX.PSYBASS_HARMONICS`](#13-fx-0x08) | `0x0842` |  |
| PSYBASS_MASK | [`FX.PSYBASS_MASK`](#13-fx-0x08) | `0x0846` |  |
| PSYBASS_ORIGINAL | [`FX.PSYBASS_ORIGINAL`](#13-fx-0x08) | `0x0845` |  |
| REVERT | [`CS.REVERT`](#15-cs-0x0a) | `0x0A19` |  |
| RTA_BANDS | [`MEAS.RTA_BANDS`](#14-meas-0x09) | `0x0923` |  |
| RTA_BANDS_ALL | [`MEAS.RTA_BANDS_ALL`](#14-meas-0x09) | `0x0926` |  |
| RTA_BAND_CENTRES | [`MEAS.RTA_BAND_CENTRES`](#14-meas-0x09) | `0x0922` |  |
| RTA_BINS | [`MEAS.RTA_BINS`](#14-meas-0x09) | `0x0924` |  |
| RTA_CAPS | [`MEAS.RTA_CAPS`](#14-meas-0x09) | `0x0921` |  |
| RTA_CONFIG | [`MEAS.RTA_CONFIG`](#14-meas-0x09) | `0x0920` |  |
| RTA_RESET | [`MEAS.RTA_RESET`](#14-meas-0x09) | `0x092A` |  |
| RTA_START | [`MEAS.RTA_START`](#14-meas-0x09) | `0x0928` |  |
| RTA_STATUS | [`MEAS.RTA_STATUS`](#14-meas-0x09) | `0x0925` |  |
| RTA_STOP | [`MEAS.RTA_STOP`](#14-meas-0x09) | `0x0929` |  |
| SAVE | [`CS.SAVE`](#15-cs-0x0a) | `0x0A18` |  |
| SERIAL | [`SYS.SERIAL`](#6-sys-0x01) | `0x0121` |  |
| SIGGEN_CAPS | [`MEAS.SIGGEN_CAPS`](#14-meas-0x09) | `0x0942` |  |
| SIGGEN_CONFIG | [`MEAS.SIGGEN_CONFIG`](#14-meas-0x09) | `0x0940` |  |
| SIGGEN_START | [`MEAS.SIGGEN_START`](#14-meas-0x09) | `0x0948` |  |
| SIGGEN_STATUS | [`MEAS.SIGGEN_STATUS`](#14-meas-0x09) | `0x0941` |  |
| SIGGEN_STOP | [`MEAS.SIGGEN_STOP`](#14-meas-0x09) | `0x0949` |  |
| SIGGEN_STOP_NOW | [`MEAS.SIGGEN_STOP_NOW`](#14-meas-0x09) | `0x094A` |  |
| SIGGEN_TYPE_DESC | [`MEAS.SIGGEN_TYPE_DESC`](#14-meas-0x09) | `0x0943` |  |
| SLOT_NAME | [`CS.SLOT_NAME`](#15-cs-0x0a) | `0x0A21` |  |
| SOURCE | [`INPUT.SOURCE`](#8-input-0x03) | `0x0310` |  |
| SOURCE_NAME | [`INPUT.SOURCE_NAME`](#8-input-0x03) | `0x0312` |  |
| SPDIF_RX_CH_STATUS | [`INPUT.SPDIF_RX_CH_STATUS`](#8-input-0x03) | `0x0321` |  |
| SPDIF_RX_CONFIG | [`INPUT.SPDIF_RX_CONFIG`](#8-input-0x03) | `0x0324` |  |
| SPDIF_RX_COUNT | [`INPUT.SPDIF_RX_COUNT`](#8-input-0x03) | `0x0326` |  |
| SPDIF_RX_ENABLE | [`INPUT.SPDIF_RX_ENABLE`](#8-input-0x03) | `0x0327` |  |
| SPDIF_RX_PIN | [`INPUT.SPDIF_RX_PIN`](#8-input-0x03) | `0x0322` |  |
| SPDIF_RX_PIN_AVAIL | [`INPUT.SPDIF_RX_PIN_AVAIL`](#8-input-0x03) | `0x0325` |  |
| SPDIF_RX_STATUS | [`INPUT.SPDIF_RX_STATUS`](#8-input-0x03) | `0x0320` |  |
| SPDIF_TX_STATS | [`SYS.SPDIF_TX_STATS`](#6-sys-0x01) | `0x0145` |  |
| STARTUP | [`STORE.STARTUP`](#7-store-0x02) | `0x0216` |  |
| STATS_RESET | [`SYS.STATS_RESET`](#6-sys-0x01) | `0x014F` |  |
| STATUS | [`INPUT.STATUS`](#8-input-0x03) | `0x0311` |  |
| STATUS | [`CS.STATUS`](#15-cs-0x0a) | `0x0A12` |  |
| STATUS | [`XPORT.STATUS`](#16-xport-0x0b) | `0x0B12` |  |
| SUBHARM_BOOST | [`FX.SUBHARM_BOOST`](#13-fx-0x08) | `0x0824` |  |
| SUBHARM_CEILING | [`FX.SUBHARM_CEILING`](#13-fx-0x08) | `0x0829` |  |
| SUBHARM_DEPTH | [`FX.SUBHARM_DEPTH`](#13-fx-0x08) | `0x0827` |  |
| SUBHARM_ENABLE | [`FX.SUBHARM_ENABLE`](#13-fx-0x08) | `0x0820` |  |
| SUBHARM_HEADROOM | [`FX.SUBHARM_HEADROOM`](#13-fx-0x08) | `0x082E` |  |
| SUBHARM_HIGH | [`FX.SUBHARM_HIGH`](#13-fx-0x08) | `0x0822` |  |
| SUBHARM_HOLD | [`FX.SUBHARM_HOLD`](#13-fx-0x08) | `0x0828` |  |
| SUBHARM_LINK | [`FX.SUBHARM_LINK`](#13-fx-0x08) | `0x082A` |  |
| SUBHARM_LOW | [`FX.SUBHARM_LOW`](#13-fx-0x08) | `0x0821` |  |
| SUBHARM_MASK | [`FX.SUBHARM_MASK`](#13-fx-0x08) | `0x0825` |  |
| SUBHARM_METER | [`FX.SUBHARM_METER`](#13-fx-0x08) | `0x082F` |  |
| SUBHARM_SELECT | [`FX.SUBHARM_SELECT`](#13-fx-0x08) | `0x0826` |  |
| SUBHARM_SOLO | [`FX.SUBHARM_SOLO`](#13-fx-0x08) | `0x082B` |  |
| SUBHARM_TOP | [`FX.SUBHARM_TOP`](#13-fx-0x08) | `0x0823` |  |
| SYSTEM_STATUS | [`SYS.SYSTEM_STATUS`](#6-sys-0x01) | `0x0140` |  |
| TUBE_ASYM | [`FX.TUBE_ASYM`](#13-fx-0x08) | `0x0865` |  |
| TUBE_BIAS | [`FX.TUBE_BIAS`](#13-fx-0x08) | `0x0864` |  |
| TUBE_DRIVE | [`FX.TUBE_DRIVE`](#13-fx-0x08) | `0x0863` |  |
| TUBE_ENABLE | [`FX.TUBE_ENABLE`](#13-fx-0x08) | `0x0860` |  |
| TUBE_HARDNESS | [`FX.TUBE_HARDNESS`](#13-fx-0x08) | `0x0866` |  |
| TUBE_MIX | [`FX.TUBE_MIX`](#13-fx-0x08) | `0x086C` |  |
| TUBE_OUTPUT_MASK | [`FX.TUBE_OUTPUT_MASK`](#13-fx-0x08) | `0x0861` |  |
| TUBE_RECTIFIER | [`FX.TUBE_RECTIFIER`](#13-fx-0x08) | `0x0868` |  |
| TUBE_SAG | [`FX.TUBE_SAG`](#13-fx-0x08) | `0x0867` |  |
| TUBE_TRIM | [`FX.TUBE_TRIM`](#13-fx-0x08) | `0x086D` |  |
| TUBE_TYPE | [`FX.TUBE_TYPE`](#13-fx-0x08) | `0x0862` |  |
| TUBE_XFMR_DAMPING | [`FX.TUBE_XFMR_DAMPING`](#13-fx-0x08) | `0x086A` |  |
| TUBE_XFMR_ENABLE | [`FX.TUBE_XFMR_ENABLE`](#13-fx-0x08) | `0x0869` |  |
| TUBE_XFMR_RES | [`FX.TUBE_XFMR_RES`](#13-fx-0x08) | `0x086B` |  |
| TYPE | [`OUTPUT.TYPE`](#9-output-0x04) | `0x0410` |  |
| UART_CONFIG | [`XPORT.UART_CONFIG`](#16-xport-0x0b) | `0x0B10` |  |
| UART_PIN_AVAIL | [`XPORT.UART_PIN_AVAIL`](#16-xport-0x0b) | `0x0B13` |  |
| UPMIX_ATTACK | [`FX.UPMIX_ATTACK`](#13-fx-0x08) | `0x0896` | RP2350 only |
| UPMIX_CENTER_MODE | [`FX.UPMIX_CENTER_MODE`](#13-fx-0x08) | `0x0891` | RP2350 only |
| UPMIX_CENTER_WIDTH | [`FX.UPMIX_CENTER_WIDTH`](#13-fx-0x08) | `0x0894` | RP2350 only |
| UPMIX_CONFIG | [`FX.UPMIX_CONFIG`](#13-fx-0x08) | `0x0880` | RP2350 only |
| UPMIX_DECORR | [`FX.UPMIX_DECORR`](#13-fx-0x08) | `0x089C` | RP2350 only |
| UPMIX_DET_HPF | [`FX.UPMIX_DET_HPF`](#13-fx-0x08) | `0x0898` | RP2350 only |
| UPMIX_ENABLE | [`FX.UPMIX_ENABLE`](#13-fx-0x08) | `0x0890` | RP2350 only |
| UPMIX_PRESENCE | [`FX.UPMIX_PRESENCE`](#13-fx-0x08) | `0x089D` | RP2350 only |
| UPMIX_RELEASE | [`FX.UPMIX_RELEASE`](#13-fx-0x08) | `0x0897` | RP2350 only |
| UPMIX_STATUS | [`FX.UPMIX_STATUS`](#13-fx-0x08) | `0x0881` | RP2350 only |
| UPMIX_STRENGTH | [`FX.UPMIX_STRENGTH`](#13-fx-0x08) | `0x0893` | RP2350 only |
| UPMIX_SURROUND_MODE | [`FX.UPMIX_SURROUND_MODE`](#13-fx-0x08) | `0x0892` | RP2350 only |
| UPMIX_SUR_DELAY | [`FX.UPMIX_SUR_DELAY`](#13-fx-0x08) | `0x0899` | RP2350 only |
| UPMIX_SUR_HPF | [`FX.UPMIX_SUR_HPF`](#13-fx-0x08) | `0x089A` | RP2350 only |
| UPMIX_SUR_LPF | [`FX.UPMIX_SUR_LPF`](#13-fx-0x08) | `0x089B` | RP2350 only |
| UPMIX_THRESHOLD | [`FX.UPMIX_THRESHOLD`](#13-fx-0x08) | `0x0895` | RP2350 only |
| USB_AUDIO_STATUS | [`SYS.USB_AUDIO_STATUS`](#6-sys-0x01) | `0x0141` |  |
| USER_MUTE | [`ROUTE.USER_MUTE`](#10-route-0x05) | `0x0561` |  |
| USER_VOLUME | [`ROUTE.USER_VOLUME`](#10-route-0x05) | `0x0560` |  |
| WIRECAP_ARM | [`DIAG.WIRECAP_ARM`](#18-diag-0xf0) | `0xF050` | `DSPI_WIRECAP` |
| WIRECAP_CLKGEN | [`DIAG.WIRECAP_CLKGEN`](#18-diag-0xf0) | `0xF053` | `DSPI_WIRECAP` |
| WIRECAP_CLKGEN_GLITCH | [`DIAG.WIRECAP_CLKGEN_GLITCH`](#18-diag-0xf0) | `0xF054` | `DSPI_WIRECAP` |
| WIRECAP_RATE | [`DIAG.WIRECAP_RATE`](#18-diag-0xf0) | `0xF055` | `DSPI_WIRECAP` |
| WIRECAP_READ | [`DIAG.WIRECAP_READ`](#18-diag-0xf0) | `0xF052` | `DSPI_WIRECAP` |
| WIRECAP_STATUS | [`DIAG.WIRECAP_STATUS`](#18-diag-0xf0) | `0xF051` | `DSPI_WIRECAP` |

<!-- VC:END INDEX -->
