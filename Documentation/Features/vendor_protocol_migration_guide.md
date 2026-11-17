# DSPi Host Migration Guide: Firmware 1.1.6 to 1.1.7

*Last updated: 2026-11-16*

This guide is for the maintainers of every program that talks to a DSPi over its control protocol. That includes DSPi Console on every platform, DSPi Link and its bridges, DSPi Terminal, test scripts and third-party controllers such as an ESP32 front panel on UART. It tells you, for every 1.1.6 opcode, what to send to firmware 1.1.7 instead and exactly what comes back. You can port a host from this guide alone, without reading firmware source.

The full reference for the new protocol is `Documentation/commands.md`. This guide repeats what a porter needs from it, so you only open that file for detail you choose to look up.

A short version with only the changes a client must make is [`vendor_protocol_client_update.md`](vendor_protocol_client_update.md).

---

## TL;DR

Firmware 1.1.7 beta 1 replaced the control protocol. In 1.1.6 every command was one 8-bit number in `bRequest`. In 1.1.7 every command has a two-part address. The **page** names a subsystem, such as filters, presets or inputs, and travels in the high byte of `wIndex`. The **command number** names one thing inside that page and travels in `bRequest`. Nothing else about the USB request changes. It is still a vendor control transfer to interface 2.

The old numbers are gone. Every 1.1.6 request has a zero in the `wIndex` high byte, so it lands on page 0, which has no commands. The device refuses it and changes nothing. A 1.1.6 host therefore fails safely on 1.1.7, but it fails on every request, so every host must be ported.

Porting a host comes down to six changes.

1. **Probe first.** Before anything else, read `SYS.PROTOCOL_INFO` and check that the reply starts with `DSPi`. Old firmware ignores the page byte and would run an unrelated old command with the same number. Some of those old commands reset the device or erase a preset.
2. **Renumber.** Look each old opcode up in section 6 and send the new page and command number. Put the page in the high byte of `wIndex` and keep the interface number, 2, in the low byte.
3. **Move values out of `wValue`.** In 1.1.7 `wValue` only says which channel, output or slot you mean. Values always travel in the payload.
4. **Turn write-as-reads into writes.** 34 old commands changed state through an IN transfer and answered a status byte. They are now ordinary SETs, and an action with no value is a SET with no payload.
5. **Treat a STALL as a refusal.** A rejected request now fails explicitly. On USB it STALLs, and `SYS.LAST_RESULT` says why. On UART and I2C the status byte carries the reason.
6. **Re-check every reply length.** Most replies kept their 1.1.6 bytes exactly. A few changed or split. Section 7 lists every reply length and says which ones changed.

What did not change is just as important. The bulk parameter blob, the notification packets and endpoint, the USB audio class, the descriptors, the VID and PID, the interface numbers and the UART and I2C frame formats are all the same as in 1.1.6.

---

## 1. Who must read what

Read the TL;DR and section 2 whatever you maintain. Then follow your row.

| You maintain | Read | Skim |
|---|---|---|
| A USB application (DSPi Console on any platform, DSPi Terminal, a script) | 3, 4, 5, 6, 7, 11.1 to 11.4 for your platform, 12 | 8, 9, 10 |
| A UART or I2C controller (ESP32 front panel, microcontroller client) | 3.1, 4, 5, 6, 7 (all of it, if you check reply lengths or values), 11.5 or 11.6, 12 | 8 |
| A bridge or hub that forwards requests (DSPi Link, a TCP gateway) | 3, 4, 5.1 to 5.3, 6.1, 11 for each transport you carry, 12 | 6.2, 7 |
| A tool author generating constants or policy tables | 4.7, 6.1, 7.1, 12.1 | everything else as needed |

A bridge that passes `wIndex` through untouched keeps working at the wire level. What breaks is any table, policy, router, cache, bulk handler or analyser arbitration keyed on `bRequest` alone. Key them on the pair `(page, command)`, or on the 16-bit command ID `(page << 8) | command`. A 1.1.6 policy table indexed by the new command byte gives wrong classes, not just missing ones. For example `SYS.ENTER_BOOTLOADER` (`0x0130`) and `STORE.FACTORY_RESET` (`0x0230`) both have command `0x30`, which in 1.1.6 was `SET_PSYBASS`, an ordinary DSP write.

The host appendices in section 13 cover the ESP32 front panel, DSPi Console for macOS, DSPi Link and DSPi Terminal. Other hosts, for example DSPi Console on Windows, follow sections 11 and 12.

---

## 2. What changed, in one page

### 2.1 The new model

- **Pages.** Pages are numbered `0x01` to `0xFF`, each with room for 240 commands. Page 0 is never used. Firmware 1.1.7 implements 12 pages in a release build, plus a diagnostics page in bench builds. In this guide a command is written `PAGE.NAME`, for example `FILTER.BAND`, and its 16-bit ID is `(page << 8) | command`, for example `0x0610`.
- **One command per thing.** A setting you can read and write is one command. A GET reads it and a SET writes it. In 1.1.6 most settings had a SET opcode and a GET opcode with neighbouring numbers. They are folded into one command.
- **The address goes in `wValue`.** `wValue` names which instance you mean, such as a channel, an output, a preset slot or a filter band. A two-level address packs as `(major << 8) | minor`. A command that addresses nothing requires `wValue` 0. `wValue` never selects an operation and never carries a value.
- **Values go in the payload.** A GPIO number, a mode, an enable flag and a gain all travel in the payload of a SET.
- **Actions are zero-length SETs.** A request that does something, such as loading a preset or clearing clip flags, is a SET with `wLength` 0. Reads never change anything.
- **Failure is explicit.** A refused request STALLs on USB and returns a result code on UART and I2C. 1.1.6 accepted most bad writes silently and changed nothing. 1.1.7 tells you.
- **Discovery.** The device reports its pages, page revisions and the commands each page implements. A host decides whether a feature exists by asking the device, not by comparing firmware versions.

### 2.2 What did not change

| Item | Status in 1.1.7 |
|---|---|
| USB VID and PID | `0x2E8B : 0xFEAA`, unchanged |
| Interface numbers | Audio control 0, audio streaming 1, vendor interface 2 (class `0xFF`), unchanged |
| `bmRequestType` | `0xC1` for a GET, `0x41` for a SET, unchanged |
| USB audio class (UAC1), its volume and mute controls, descriptors | Unchanged |
| Microsoft OS 2.0 descriptor request (`bRequest` `0x01`, `wIndex` 7) | Unchanged. Windows still binds WinUSB to interface 2 with no driver install |
| Bulk parameter blob `WireBulkParams` | Unchanged. Format version 32, 6136 bytes, same layout on both platforms (section 9) |
| Notifications | Unchanged. Bulk IN endpoint `0x83`, same packets, same events, same offsets (section 10) |
| UART and I2C frame formats, CRC and timing | Unchanged (sections 11.5 and 11.6). Only the meaning of the status byte grew |
| Reply bytes of most GETs | Unchanged. Section 7.2 lists the exceptions |
| Payload bytes of most SETs | Unchanged. Only `FILTER.BAND` and `ROUTE.MATRIX_ROUTE` were reshaped (section 6.5) |

---

## 3. The request, field by field

### 3.1 Fields

| Field | 1.1.6 | 1.1.7 |
|---|---|---|
| `bmRequestType` | `0xC1` read, `0x41` write | Same. `0xC1` GET, `0x41` SET. Vendor type, interface recipient |
| `bRequest` | The opcode, `0x03` to `0xFF` | The command number within the page, `0x10` to `0xFF`. `0x00` to `0x0F` are reserved in every page |
| `wIndex` high byte | Ignored. Hosts sent 0 | **The page**, `0x01` to `0xFF` |
| `wIndex` low byte | Hosts sent 0 or 2 | The interface number, 2. The firmware ignores it, but host USB stacks route interface requests by it |
| `wValue` | Address, value, selector or packed fields, depending on the opcode | The address only. 0 when the command addresses nothing. Checked as a whole 16-bit value |
| `wLength` | GET: bytes wanted. SET: payload length | Same. A SET must use a length the command accepts |
| Payload | Little-endian integers and floats | Same encoding. Values that used to ride in `wValue` now go here |

So `wIndex` is always `(page << 8) | 2` on USB. For page `0x05` it is `0x0502`, which goes on the wire as the two bytes `02 05`.

The firmware reads the page from the `wIndex` high byte and ignores the low byte and the recipient bits. UART and I2C clients send the same `wIndex`, and `0x00` in the low byte is fine there.

### 3.2 Before and after, byte by byte

Each example shows the 8-byte SETUP packet as it goes on the wire. The bytes are `bmRequestType`, `bRequest`, then `wValue`, `wIndex` and `wLength` as two little-endian bytes each. `OUT` is the data the host sends and `IN` is the data the device returns. The 1.1.7 replies are those of an RP2350 running firmware 1.1.7 beta 1. The 1.1.6 lines show what a 1.1.6 host sent.

**A SET with an address.** Set the gain of output 1 to -6.0 dB. The output is the address in `wValue`. The gain is a float32 payload (`-6.0` is `0xC0C00000`). The payload did not change. Only `bRequest` and `wIndex` did.

```
1.1.6  SET_OUTPUT_GAIN 0x74
SETUP  41 74 01 00 02 00 04 00
OUT    00 00 C0 C0

1.1.7  ROUTE.OUTPUT_GAIN (page 0x05, command 0x21)
SETUP  41 21 01 00 02 05 04 00
OUT    00 00 C0 C0
```

**A GET.** Read the same gain back. The reply is the same 4 bytes as in 1.1.6.

```
1.1.6  GET_OUTPUT_GAIN 0x75
SETUP  C1 75 01 00 02 00 04 00
IN     00 00 C0 C0

1.1.7  ROUTE.OUTPUT_GAIN (page 0x05, command 0x21)
SETUP  C1 21 01 00 02 05 04 00
IN     00 00 C0 C0
```

**A former write-as-read.** Reset the buffer watermarks. In 1.1.6 this was an IN transfer that carried the mask in `wValue` and answered one status byte. In 1.1.7 it is a SET with the mask as a 1-byte payload and no reply data.

```
1.1.6  RESET_BUFFER_STATS 0xB1 (write-as-read)
SETUP  C1 B1 01 00 02 00 01 00
IN     01

1.1.7  SYS.STATS_RESET (page 0x01, command 0x4F)
SETUP  41 4F 00 00 02 01 01 00
OUT    01
```

**A zero-length action.** Clear the spectrum analyser's averages and peaks. In 1.1.6 the action was a selector in `wValue` of `RTA_CONTROL`. In 1.1.7 each action is its own command, sent as a SET with `wLength` 0, so there is no data stage at all.

```
1.1.6  RTA_CONTROL 0x0E, action 2 (write-as-read)
SETUP  C1 0E 02 00 02 00 01 00
IN     01

1.1.7  MEAS.RTA_RESET (page 0x09, command 0x2A)
SETUP  41 2A 00 00 02 09 00 00
```

**A struct read that replaces several `GET_STATUS` reads.** In 1.1.6 a stats screen read `GET_STATUS` once per value, with the value's selector in `wValue`. In 1.1.7 one read returns a whole group of related values. Here `SYS.SYSTEM_STATUS` replaces selectors 13, 14, 15 and 16. Section 6.3 maps every selector.

```
1.1.6  GET_STATUS 0x50, four requests, 4 bytes each
SETUP  C1 50 0D 00 02 00 04 00     clk_sys
SETUP  C1 50 0E 00 02 00 04 00     core voltage
SETUP  C1 50 0F 00 02 00 04 00     sample rate
SETUP  C1 50 10 00 02 00 04 00     temperature

1.1.7  SYS.SYSTEM_STATUS (page 0x01, command 0x40), one request
SETUP  C1 40 00 00 02 01 10 00
IN     16 bytes: u32 clk_sys Hz, u32 sample rate Hz, u16 core mV, i16 temperature x 100, u8 core 0 load, u8 core 1 load, 00 00
```

The values are live readings, so they differ on every device and every read. Section 6.3 has the layout.

**A refused request and its reason.** Read the gain of output 9. No platform has an output 9, so the device STALLs. The host then reads `SYS.LAST_RESULT` (page `0x01`, command `0x14`, 8 bytes).

```
1.1.7  ROUTE.OUTPUT_GAIN, output 9
SETUP  C1 21 09 00 02 05 04 00     -> STALL
SETUP  C1 14 00 00 02 01 08 00
IN     21 05 01 13 09 00 00 00
```

The reply says command `0x0521` (`ROUTE.OUTPUT_GAIN`), direction 1 (GET), result `0x13` (`VC_ERR_ADDRESS`) and `wValue` `0x0009`. In 1.1.6 the same read also STALLed, but with no way to learn why. Section 5 covers errors.

---

## 4. Probe first: discovery and compatibility

### 4.1 Why the probe is mandatory

Firmware 1.1.6 ignores `wIndex`. If a 1.1.7 host sends a paged request to it, the old firmware runs whatever 1.1.6 opcode has the same number as the command. For reads this is dangerous, because 1.1.6 ran 34 state-changing commands on the IN path. These ordinary-looking 1.1.7 reads each trigger a 1.1.6 write-as-read on old firmware.

| 1.1.7 GET | On 1.1.6 it runs | Effect on a 1.1.6 device |
|---|---|---|
| `ROUTE.MASTER_VOLUME_SAVED` (`0x0553`) | `0x53` `FACTORY_RESET` | Resets the live state to factory defaults |
| `FX.UPMIX_SURROUND_MODE` (`0x0892`) | `0x92` `PRESET_DELETE`, slot 0 | Erases preset 0 |
| `FX.UPMIX_CENTER_MODE` (`0x0891`) | `0x91` `PRESET_LOAD`, slot 0 | Replaces the live state with preset 0 |
| `FX.UPMIX_ENABLE` (`0x0890`) | `0x90` `PRESET_SAVE`, slot 0 | Overwrites preset 0 |
| `ROUTE.MASTER_VOLUME_MODE` (`0x0551`), `INPUT.LG_SOUND_SYNC_STATUS` (`0x0351`), `CS.MACRO_HEADER` (`0x0A51`) | `0x51` `SAVE_PARAMS` | Saves the live state over the active preset (flash write) |
| `CS.MACRO_STEP` (`0x0A52`) | `0x52` `SAVE_OUTPUT_CONFIG` | Saves the live output and pin configuration (flash write) |
| `FX.UPMIX_PRESENCE` (`0x089D`) | `0x9D` `CS_SAVE` | Saves the Control Surfaces configuration (flash write) |
| `FX.SUBHARM_MASK` (`0x0825`), `DYN.LEVELLER_GATE` (`0x0725`), `MEAS.RTA_STATUS` (`0x0925`) | `0x25` `CS_MACRO_FIRE`, macro 0 | Runs Control Surfaces macro 0 |
| `FX.TUBE_RECTIFIER` (`0x0868`) | `0x68` `SET_ADAT_INPUT_ENABLE` | Disables the ADAT input (RP2350) |
| `FX.TUBE_XFMR_DAMPING` (`0x086A`) | `0x6A` `SET_ADAT_INPUT_PIN` | Moves the ADAT input to GPIO 0 (RP2350) |
| `FX.TUBE_MIX` (`0x086C`) | `0x6C` `SET_ADAT_INPUT_CLOCK_MODE` | Sets the ADAT clock mode to master |

1.1.6 sent an IN transfer only to its read handlers and an OUT transfer only to its write handlers, so these 15 reads are the complete list. A 1.1.7 GET never reaches an old OUT-only write.

SETs collide too. Every 1.1.7 SET sent to 1.1.6 firmware runs the 1.1.6 OUT handler with the same number, with a payload meant for something else, and 1.1.6 usually acknowledges it as a success. For example a `ROUTE.MATRIX_ROUTE` SET (`0x0510`) is an old `SET_SUBHARM` (`0x10`) and switches the subharmonic synth on or off. The worst case is `0x44`, the old `SET_PREAMP`, which sets the preamp of every input channel to the payload in dB with no clamp. A `FX.PSYBASS_CHARACTER` SET (`0x0844`) of 50 percent therefore puts +50 dB of gain on every input of a 1.1.6 device. Only a zero-length action changes no setting on 1.1.6, because the old firmware ignores an empty write.

The probe itself is safe on both generations. 1.1.6 has no IN handler for `bRequest` `0x10`, so it STALLs and changes nothing.

### 4.2 The probe

Send one GET of `SYS.PROTOCOL_INFO` (page `0x01`, command `0x10`, 16 bytes) before any other request.

```
SETUP  C1 10 00 00 02 01 10 00
IN     44 53 50 69 01 00 0C 00 40 00 00 01 00 00 00 00
```

| Offset | Type | Field | 1.1.7 beta 1 |
|---|---|---|---|
| 0 | u8[4] | Magic `"DSPi"` | `44 53 50 69` |
| 4 | u8 | Protocol major | 1 |
| 5 | u8 | Protocol minor | 0 |
| 6 | u8 | Number of implemented pages | 12 in a release build |
| 7 | u8 | Flags, reserved | 0 |
| 8 | u16 | Largest non-bulk SET payload | 64 |
| 10 | u16 | Largest non-bulk GET reply | 256 |
| 12 | u32 | Reserved | 0 |

The protocol major changes only on an incompatible change. Refuse a major you do not know and tell the user to update the host.

### 4.3 What each firmware answers

| Request | Firmware 1.1.6 or older | Firmware 1.1.7 |
|---|---|---|
| Probe `C1 10 00 00 02 01 10 00` | STALL (UART and I2C: status `0x02`) | 16 bytes starting `DSPi` |
| Old `GET_PLATFORM` `C1 7F 00 00 02 00 07 00` | 7 bytes, the same layout as `SYS.PLATFORM` (4.6). `wIndex` 0 works too | STALL. Page 0 has no commands (`VC_ERR_UNKNOWN`) |
| Old `ENTER_BOOTLOADER` `C1 F0 00 00 02 00 01 00` | Answers `01`, then reboots into the USB bootloader about 100 ms later | STALL, nothing happens |
| `SYS.PLATFORM` `C1 20 00 00 02 01 07 00` | Do not send. On 1.1.6 it is opcode `0x20` | 7 bytes (4.6) |
| `SYS.ENTER_BOOTLOADER` `41 30 00 00 02 01 00 00` | Do not send. 1.1.6 takes it as an empty write to opcode `0x30` (`SET_PSYBASS`), acknowledges it and ignores it, so the device never reboots. Use the old `ENTER_BOOTLOADER` above | Reboots into the USB bootloader after the status stage |
| Any 1.1.6 request (`wIndex` high byte 0) | Works as in 1.1.6 | STALL at SETUP, `VC_ERR_UNKNOWN`, nothing changes |

On 1.1.7, a request with a zero page is refused before any handler runs, so a 1.1.6 host can neither read the version of a 1.1.7 device nor send it to the bootloader. A user with an old host and new firmware must update the host, or hold BOOTSEL while plugging the board in to reflash it.

### 4.4 A host that must support both generations

A host with a firmware update flow has to recognise old firmware well enough to offer the update. Use this procedure at connect.

1. Send the probe. On a timeout, wait about 150 ms and retry a few times. The device may be busy with a flash write.
2. **The reply starts with `DSPi`.** The device speaks the paged protocol. Check the protocol major (byte 4). If it is not 1, tell the user to update the host and send nothing else. Otherwise continue with section 4.5.
3. **The probe STALLs.** The device runs 1.1.6 or older. From now on send it only two requests.
    - Old `GET_PLATFORM`, `C1 7F 00 00 02 00 07 00`, to show the version. A host that asks for fewer bytes gets a prefix.
    - Old `ENTER_BOOTLOADER`, `C1 F0 00 00 02 00 01 00`, once the user confirms the update. It answers `01` and the device reappears as the `RPI-RP2` drive.
4. Never send any other request to old firmware, and never send a paged request to it.

On UART and I2C the same procedure applies. Old firmware answers the probe with status `0x02` instead of a STALL, and its `GET_PLATFORM` is a GET frame with `bRequest` `0x7F`, `wIndex` `0x0000` and `wLength` 7 (section 11.5).

### 4.5 A host that supports only 1.1.7

1. Send the probe. A STALL means old firmware. Tell the user to update the firmware, then stop. Do not send anything else.
2. Read `SYS.PLATFORM` for the version and platform you show the user.
3. Read `SYS.PAGE_LIST`, and `SYS.PAGE_COMMANDS` for each page you use, and keep the result for the session (4.7).
4. Read `STORE.ALL_PARAMS` to build your picture of the device state, as in 1.1.6 (section 9).
5. Open the notification endpoint if you use it (section 10).

### 4.6 Identity and version: `SYS.PLATFORM`

`SYS.PLATFORM` (page `0x01`, command `0x20`, 7 bytes) has exactly the layout of the 1.1.6 `GET_PLATFORM` reply.

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Platform: 0 RP2040, 1 RP2350 |
| 1 | u8 | Firmware major |
| 2 | u8 | `(minor << 4) \| patch`, a legacy packed form |
| 3 | u8 | Number of outputs: 5 on RP2040, 9 on RP2350 |
| 4 | u8 | Firmware minor |
| 5 | u8 | Firmware patch |
| 6 | u8 | Beta number, 0 for a final release |

```
SETUP  C1 20 00 00 02 01 07 00
IN     01 01 17 09 01 07 01
```

That is RP2350, firmware 1.1.7 beta 1. An RP2040 on the same firmware answers `00 01 17 05 01 07 01`. Use bytes 1, 4, 5 and 6 for the version. Byte 2 cannot hold a minor or patch above 15. Show the version to the user and use it for update offers. Do not use it to decide which features exist (4.7).

### 4.7 Pages, commands and feature detection

Three discovery commands describe the running firmware. All three are on page `0x01` and take no payload.

**`SYS.PAGE_LIST`** (command `0x11`) returns two bytes per implemented page, `{page, revision}`, in ascending page order. Its length is twice the page count in `SYS.PROTOCOL_INFO` byte 6. A release build of 1.1.7 beta 1 answers 24 bytes, every page at revision 1.

```
SETUP  C1 11 00 00 02 01 00 01
IN     01 01 02 01 03 01 04 01 05 01 06 01 07 01 08 01 09 01 0A 01 0B 01 0C 01
```

A page's revision increases when a command is added to it or a reply or payload grows at the end. A page a build leaves out, such as the diagnostics page `0xF0` in a release build, is absent from the list.

**`SYS.PAGE_COMMANDS`** (command `0x12`, `wValue` = page) returns 64 bytes. Bytes 0 to 31 are the GET bitmap and bytes 32 to 63 the SET bitmap. Command `n` has a GET when bit `n & 7` of byte `n >> 3` is set, and a SET when the same bit of byte `32 + (n >> 3)` is set. The `FILTER` page has commands `0x10`, `0x11` and `0x12`, each with a GET and a SET.

```
SETUP  C1 12 06 00 02 01 40 00
IN     00 00 07 00 00 00 00 00 00 00 00 00 00 00 00 00
       00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
       00 00 07 00 00 00 00 00 00 00 00 00 00 00 00 00
       00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
```

A page this build does not implement (page 0, or `0xF0` in a release build) is refused with `VC_ERR_ADDRESS`. It never returns an empty bitmap, so treat that refusal as "page absent".

**`SYS.COMMAND_INFO`** (command `0x13`, `wValue` = command ID) returns 8 bytes about one command. An ID the build does not have is refused with `VC_ERR_ADDRESS`.

| Offset | Type | Field |
|---|---|---|
| 0 | u16 | Flags (below) |
| 2 | u16 | GET reply length. 0 when there is no GET, `0xFFFF` when it varies |
| 4 | u8 | Smallest SET payload |
| 5 | u8 | Largest SET payload. Both are 0 for a zero-length action and for a bulk command |
| 6 | u8 | Address kind (`Documentation/commands.md` section 4.5) |
| 7 | u8 | Directions: bit 0 GET, bit 1 SET |

| Bit | Value | Flag | Meaning |
|---|---|---|---|
| 0 | `0x0001` | `USB_ONLY` | Refused on UART and I2C in both directions (`VC_ERR_BLOCKED`) |
| 1 | `0x0002` | `SET_USB_ONLY` | The SET is refused on UART and I2C. The GET works everywhere |
| 2 | `0x0004` | `BULK` | Uses the bulk buffer |
| 3 | `0x0008` | `DEFERRED` | The main loop finishes the work after the reply (5.8) |
| 4 | `0x0010` | `FLASH` | Writes flash, so a busy window follows |
| 5 | `0x0020` | `RESTART` | Can interrupt audio. A command without it never restarts or mutes the outputs |
| 6 | `0x0040` | `KEEPALIVE` | The GET refreshes the spectrum analyser's idle timer |

Bits 0 to 2 are enforced by the device. Bits 3 to 6 describe what the command does, so a host can show a busy indicator or warn before a write that interrupts audio. `RESTART` marks the flash writers (each ends in a synchronized output restart), the input pin and clock commands (only while that input is live), `OUTPUT.DAC_MUTE_TEST`, `SYS.ENTER_BOOTLOADER` and every command that restarts the outputs. For example, `STORE.PRESET_LOAD` (`0x0211`) is a deferred, flash-writing, restarting zero-length SET addressed by preset slot.

```
SETUP  C1 13 11 02 02 01 08 00
IN     38 00 00 00 00 00 07 02
```

**Replacing version checks and probe-by-STALL.** 1.1.6 hosts decided whether a feature existed by comparing the firmware version, or by sending a command and treating a STALL as "not supported". Replace both.

- At connect, read `SYS.PAGE_LIST` and the `SYS.PAGE_COMMANDS` bitmap of every page you use, once. Keep them for the session.
- To test one command, test its bit. To test a subsystem, look for its page and compare the page revision with the one you were written for.
- To size a request, use `SYS.COMMAND_INFO` or the registry `Documentation/protocol/dspi_commands.json`, which is generated from the same table as the firmware.
- Do not probe by STALL. A STALL in 1.1.7 can mean many things (section 5), and some STALLs come from commands that changed state.

### 4.8 Platform and build conditions

Some commands exist only on one platform or in one kind of build. A command that is not built does not exist. It has no bit in `SYS.PAGE_COMMANDS` and any request to it is refused with `VC_ERR_UNKNOWN`. In 1.1.6 most of these answered zeros on RP2040.

| Condition | Commands |
|---|---|
| RP2350 only | `INPUT.ADAT_RX_ENABLE`, `INPUT.ADAT_RX_PIN`, `INPUT.ADAT_RX_CLOCK_MODE`, `INPUT.ADAT_RX_STATUS`, `OUTPUT.ADAT_TX_ENABLE`, `OUTPUT.ADAT_TX_PIN`, `OUTPUT.ADAT_TX_STATUS`, `FX.UPMIX_CONFIG`, `FX.UPMIX_STATUS` and the 14 `FX.UPMIX_*` parameter commands |
| Bench builds only | Every command of the `DIAG` page (`0xF0`), each behind its own build option. Release builds have no `DIAG` page |

So an RP2040 has no ADAT input, ADAT output or upmixer commands at all, although the `INPUT`, `OUTPUT` and `FX` pages exist there. A host that reads them unconditionally, for example while filling a settings screen or a state mirror, is refused on every RP2040. Read them only when their bit is set in `SYS.PAGE_COMMANDS` (4.7).

A value can also be platform-dependent without the command being absent. Selecting input source 3 (ADAT) with `INPUT.SOURCE` on RP2040 is refused with `VC_ERR_UNSUPPORTED`, and `INPUT.I2S_RX_CHANNELS` of 4, 6 or 8 on RP2040 is refused the same way.

---

## 5. Errors

### 5.1 How a refusal shows on USB

The device checks every request in a fixed order and stops at the first failure.

1. The page and command exist on this build. Otherwise `VC_ERR_UNKNOWN`.
2. The command has a handler for this direction. Otherwise `VC_ERR_DIRECTION`.
3. The command is allowed on this transport. Otherwise `VC_ERR_BLOCKED`.
4. For a SET, the payload length is one the command accepts. Otherwise `VC_ERR_LENGTH`.
5. `wValue` is in range for the command, checked as a whole 16-bit value. Otherwise `VC_ERR_ADDRESS`.
6. The handler runs and may refuse a value, a state or a pin.

On USB, where the STALL lands depends on the kind of request.

| Request | Checks 1 to 5 | Handler | What a refusal looks like |
|---|---|---|---|
| GET | At SETUP | At SETUP | The data stage STALLs. No reply bytes arrive |
| Zero-length SET | At SETUP | At SETUP | The status stage STALLs |
| SET with payload | At SETUP | After the data stage | A failed check STALLs the data stage. A handler refusal STALLs the status stage |

Your USB library reports each case as a failed transfer with its stall error, for example `LIBUSB_ERROR_PIPE` in libusb. Treat it as "refused", not as a fault. Retrying the same request gives the same answer, except for `VC_ERR_BUSY` (5.8). A refused request changes nothing.

In 1.1.6 a rejected OUT write was ACKed and ignored. In 1.1.7 the same write STALLs. Any host code that assumed every SET succeeds must now handle failure.

### 5.2 `SYS.LAST_RESULT`

`SYS.LAST_RESULT` (page `0x01`, command `0x14`, 8 bytes) tells a USB host why its last request failed.

| Offset | Type | Field |
|---|---|---|
| 0 | u16 | Command ID of the recorded request, `(page << 8) \| cmd` |
| 2 | u8 | Direction: 1 GET, 0 SET |
| 3 | u8 | Result code (5.3) |
| 4 | u16 | `wValue` of the recorded request |
| 6 | u16 | Reserved, 0 |

What it records:

- Every request is recorded, success or failure, including SETs of `SYS.LAST_RESULT` and the discovery commands.
- A GET of `SYS.LAST_RESULT` itself is never recorded, even when it is refused. Reading it never overwrites it.
- A UART or I2C frame that fails at the transport (status `0x01` to `0x07`) never reaches a command and is not recorded.
- The record is kept per transport (USB, UART, I2C), not per host. Two USB programs on one device, such as DSPi Console and a script, overwrite each other's result.

So read it straight after the STALL you want to explain, and check that the command ID, direction and `wValue` match your own request before trusting the code. On UART and I2C you do not need it, because the status byte of the response already carries the same code.

### 5.3 Result codes

| Code | Name | Meaning | Typical cause |
|---|---|---|---|
| `0x00` | `VC_OK` | Done, or accepted and queued | |
| `0x10` | `VC_ERR_UNKNOWN` | No such page or command on this build | Page 0 (a 1.1.6 request), a typo in the page or number, an RP2350-only command on RP2040, a `DIAG` command on a release build |
| `0x11` | `VC_ERR_DIRECTION` | The command has no handler for this direction | A GET of an action, a SET of a status read |
| `0x12` | `VC_ERR_LENGTH` | SET payload length outside the allowed range | The 1.1.6 habit of sending a longer or shorter buffer, an empty name |
| `0x13` | `VC_ERR_ADDRESS` | `wValue` out of range, or names something that does not exist | A channel, output, slot or band past the end, a stray high byte, a non-zero `wValue` on a command that addresses nothing |
| `0x14` | `VC_ERR_VALUE` | A payload field is out of range, or a float is NaN or infinite | An unknown enum value, a bad mode, a NaN gain |
| `0x15` | `VC_ERR_UNSUPPORTED` | The value is valid but this platform cannot do it | ADAT source or 4 I2S pairs on RP2040 |
| `0x16` | `VC_ERR_BLOCKED` | Not allowed on this transport | A USB-only command sent over UART or I2C |
| `0x17` | `VC_ERR_BUSY` | Try again shortly | A previous band write or Control Surfaces edit still queued, the bulk buffer held (USB only, UART and I2C answer status `0x04`) |
| `0x18` | `VC_ERR_STATE` | Not allowed in the current state | Disabling the active input, a PDM pin move while PDM runs, an output enable that collides with the core 1 mode |
| `0x19` | `VC_ERR_PIN_INVALID` | The GPIO cannot be used for this function | GPIO 23 to 25, a non-clock pin for MCK |
| `0x1A` | `VC_ERR_PIN_IN_USE` | The GPIO is already claimed | Another output, input, interface or binding holds it |

### 5.4 Old status bytes and the result codes that replace them

The 1.1.6 write-as-reads answered a status byte from one of four families. In 1.1.7 there is no status byte. Success is an ACK and failure is a STALL with a result code. A host that switched on the old byte must now switch on the result code. The per-command details are in the write-as-read table (section 6.6).

`PIN_CONFIG_*` (pin, output, clock, ADAT, I2S and S/PDIF input commands).

| Old byte | Old name | New result |
|---|---|---|
| `0x00` | `PIN_CONFIG_SUCCESS` | `VC_OK` |
| `0x01` | `PIN_CONFIG_INVALID_PIN` | `VC_ERR_PIN_INVALID` |
| `0x02` | `PIN_CONFIG_PIN_IN_USE` | `VC_ERR_PIN_IN_USE` |
| `0x03` | `PIN_CONFIG_INVALID_OUTPUT` | `VC_ERR_ADDRESS` (index out of range), `VC_ERR_UNKNOWN` (command absent on this platform) or `VC_ERR_UNSUPPORTED` (value this platform cannot do) |
| `0x04` | `PIN_CONFIG_OUTPUT_ACTIVE` | `VC_ERR_STATE` |
| `0x05` | `PIN_CONFIG_INVALID_PARAM` | `VC_ERR_VALUE` |

Some commands used the old bytes differently, so their old byte maps to a different code.

| Old command | Old byte | New result |
|---|---|---|
| `0x68` `SET_ADAT_INPUT_ENABLE`, `0x6A` `SET_ADAT_INPUT_PIN` | `0x02` | `VC_ERR_STATE` (disable or clear while live) as well as `VC_ERR_PIN_IN_USE` |
| `0xC0` `SET_OUTPUT_TYPE`, `0xC8` `SET_MCK_MULTIPLIER` | `0x01` | `VC_ERR_VALUE` (bad type or multiplier) as well as `VC_ERR_PIN_INVALID` |
| `0xE9` `SET_SPDIF_INPUT_ENABLE` | `0x02` | `VC_ERR_PIN_INVALID`, `VC_ERR_PIN_IN_USE` or `VC_ERR_STATE`. 1.1.6 used `0x02` for all three |
| `0xEC` `TEST_DAC_HW_MUTE` | `0x03` | `VC_ERR_STATE` (feature disabled) |
| `0xF3` `SET_I2S_INPUT_CHANNELS` | `0x01`, `0x03` | `VC_ERR_VALUE` (count not 2, 4, 6 or 8) and `VC_ERR_UNSUPPORTED` (more pairs than the platform has) |

`PRESET_*` (preset save, load and delete, and the device-wide saves `0x52` and `0xD6`).

| Old byte | Old name | New result |
|---|---|---|
| `0x00` | `PRESET_OK` | `VC_OK` |
| `0x01` | `PRESET_ERR_INVALID_SLOT` | `VC_ERR_ADDRESS` |
| `0x02` | `PRESET_ERR_SLOT_EMPTY` | `VC_ERR_STATE`. Loading an empty slot is not an error. It loads factory defaults, as in 1.1.6 |
| `0x03`, `0x04` | `PRESET_ERR_CRC`, `PRESET_ERR_FLASH_WRITE` | None. 1.1.6 never returned them from the command, because the work happens later in the main loop |

`FLASH_OK` (`0x51` `SAVE_PARAMS`, `0x53` `FACTORY_RESET`). Always `0x00` in 1.1.6. `STORE.FACTORY_RESET` succeeds with `VC_OK`. `SAVE_PARAMS` is retired (6.7).

The acknowledge byte `0x01` (`RTA_CONTROL`, `CS_MACRO_FIRE`, `CS_IR_LEARN`, `CS_SAVE`, `CS_REVERT`, `SIGGEN_CONTROL`, `RESET_BUFFER_STATS`, `ENTER_BOOTLOADER`). 1.1.6 answered `0x01` on success and STALLed every refusal. 1.1.7 ACKs success and STALLs a refusal, now with a result code.

### 5.5 Silent before, explicit now

These rules apply on every page.

- **A rejected SET is an error.** It STALLs on USB and returns a result code on UART and I2C. 1.1.6 ACKed most rejected OUT writes and changed nothing.
- **Payload lengths are exact.** A SET must use a length the command accepts, usually exactly one length (shown in section 6.2 and in `Documentation/commands.md`). 1.1.6 accepted any payload at least as long as it needed and ignored the rest, and silently dropped short ones. A wrong length is now `VC_ERR_LENGTH`.
- **`wValue` is checked as a whole 16-bit address.** 1.1.6 often used only the low byte, so `0x0100` addressed index 0. A stray high byte is now `VC_ERR_ADDRESS`.
- **A command that addresses nothing requires `wValue` 0.** 1.1.6 ignored `wValue` there. Commands such as `CS.STATUS`, `CS.EXT_STATUS`, `CS.DISPLAY_CONFIG`, `CS.DISPLAY_STATUS`, `DYN.LIMITER_METERS`, `DYN.LIMITER_STATUS` and `STORE.ALL_PARAMS` refuse any other value with `VC_ERR_ADDRESS`.
- **A GET for a bad address is refused.** 1.1.6 often answered zero bytes of value for it (`GET_SPDIF_RX_PIN`, `GET_I2S_RX_PIN` and `GET_I2S_BCK_PIN` with a bad index, `GET_CORE1_CONFLICT` past the last output). These now STALL with `VC_ERR_ADDRESS`.
- **Commands for missing hardware do not exist.** On RP2040 the ADAT and upmixer commands answer `VC_ERR_UNKNOWN`. 1.1.6 answered zeros or `0x03`.
- **Out-of-range finite values that 1.1.6 clamped are still clamped.** Each command says whether it clamps or refuses (`Documentation/commands.md`). A read after a clamped write returns the clamped value.

### 5.6 Floats: NaN and infinity

- A NaN or infinite float in a SET is `VC_ERR_VALUE` and stores nothing. This holds for every float in `ROUTE` (matrix gain, output gain, output and channel delay, preamp, master and user volume), `DYN` (leveller amount, maximum gain and gate, loudness reference and intensity), `FX` (crossfeed cutoff and feed, every subharmonic and psychoacoustic bass float, every upmixer parameter and `FX.UPMIX_CONFIG`) and `FILTER.BAND` (frequency, Q and gain). 1.1.6 ignored some of these and stored others.
- The limiter (`DYN.LIMITER_*`) and tube (`FX.TUBE_*`) setters refuse NaN with `VC_ERR_VALUE` but clamp an infinity to the end of the range.
- `MEAS.SIGGEN_CONFIG` sanitizes its floats. A NaN level becomes -120 dBFS and a NaN parameter becomes 0, which selects the default.
- The bulk SET (`STORE.ALL_PARAMS`) does not validate floats and stores them as sent.

### 5.7 Lengths and addresses at a glance

Every SET payload length and every GET reply length is listed per command in the generated tables. Section 6.2 gives them for each old opcode. Section 7.1 gives every GET reply. The registry `Documentation/protocol/dspi_commands.json` holds both in machine-readable form.

### 5.8 Busy, deferred work and retries

**Deferred commands.** Writes that touch flash, restart the outputs, move pins, switch the input, load presets, apply the bulk blob, change filter bands or edit Control Surfaces are accepted at once and finished a moment later by the main loop. `SYS.COMMAND_INFO` marks them `DEFERRED`. `VC_OK` means the request passed every check the handler can make and is queued. Confirm the result by reading the setting back. `Documentation/commands.md` section 23 maps every write to its readback.

**The busy window.** A command flagged `FLASH` masks interrupts for about 45 ms per 4 KB flash sector, so USB control transfers are not serviced and UART bytes can be lost. A command flagged `RESTART` mutes and restarts the outputs. Right after such a command, expect a timeout. On a timeout, wait about 150 ms and retry, up to about four times, then confirm the device answers a cheap read such as `SYS.PLATFORM`. Do not send the next request immediately behind a flash-writing command. This is the same behaviour as 1.1.6.

**`VC_ERR_BUSY`.** This is the one refusal worth retrying unchanged. It appears in these cases.

| Command | Cause | What to do |
|---|---|---|
| `FILTER.BAND`, `FILTER.BAND_BYPASS` | An earlier band write has not been applied yet. There is one pending slot for both commands | Wait 1 to 2 ms and resend. Nothing was lost. 1.1.6 overwrote the pending write silently |
| `CS.BINDING`, `CS.SLOT_NAME`, `CS.IR_CMD`, `CS.GROUP`, `CS.MACRO_HEADER`, `CS.MACRO_STEP`, `CS.DISPLAY_CONFIG`, `CS.DISPLAY_PAGE`, `CS.SAVE`, `CS.REVERT` | A request of the same kind is still queued | Poll `CS.STATUS` until byte 0 leaves `0x16` (pending), then resend |
| `CS.MACRO_FIRE`, `CS.IR_LEARN_ARM`, `CS.IR_LEARN_CANCEL`, `CS.AUX_STATE` and `CS.AUX_LEVEL` SETs | A Control Surfaces revert is running | Wait for the revert to finish (`CS.STATUS`) |
| `STORE.ALL_PARAMS`, `STORE.ALL_PARAMS_CHUNK`, `MEAS.RTA_BANDS_ALL` | Another transport holds the bulk buffer, or a bulk apply is pending. This is `VC_ERR_BUSY` on USB only. Over UART and I2C the same condition is transport status `0x04` (11.5), and the other two commands are USB only | Retry after a short wait |

**Do not retry every refusal.** The flash busy window shows up as a timeout, not as a STALL. A client that treats every STALL, or every non-zero UART or I2C status, as "busy, try again" is slow and still never learns what went wrong. Read the reason first, from `SYS.LAST_RESULT` on USB or the status byte on UART and I2C. Retry unchanged only on `VC_ERR_BUSY`, and on UART and I2C also on status `0x01` and `0x04` (11.5). Over UART and I2C the device retries these three conditions itself for 50 ms before it answers with them, except a bulk SET refused at its header. A UART or I2C client that still gets one is facing a longer hold, such as a flash write, and gains nothing by resending at once.

**Timeouts.** Use a per-request timeout of about 1.5 s on USB and a few hundred milliseconds per UART or I2C request. The reference implementation `tools/dspi_test/device.py` retries a USB timeout four times with a 150 ms back-off and re-opens the device if it re-enumerates.

---

## 6. The complete old-to-new map

### 6.1 How to read the tables

The tables in this section are generated from `Documentation/protocol/legacy_map.json` and the command registry, so they cannot drift from the firmware. Columns:

| Column | Meaning |
|---|---|
| Old | The 1.1.6 opcode, the `bRequest` a 1.1.6 host sent |
| Old name | Its `REQ_` name without the prefix. A link leads to the subsection that explains it |
| Old dir | How a 1.1.6 host issued it. `W` an OUT transfer (`0x41`), `R` an IN transfer (`0xC1`), `W-as-R` an IN transfer that changed state. `0x03` and `0x81` had both directions and have one row for each |
| Old wValue | Which old requests the row covers. `any` covers every `wValue`. A split opcode has one row per selector value, where `old & 0xFF = 2` means "old requests whose `wValue` low byte was 2", and `any other` covers the remaining values |
| Change | `same`: only the number changed. `folded`: a SET and GET pair became one command. `action`: a write-as-read became a SET. `split`: one opcode became several commands, chosen by the old selector. `custom`: the reply or payload was reshaped or combined, and the linked recipe explains it. `retired`: no new command (6.7) |
| New command | What to send now, with its availability when it is platform- or build-dependent |
| ID | The 16-bit ID `(page << 8) \| command`. Send the high byte as the `wIndex` high byte and the low byte as `bRequest` |
| New dir | `G` a GET (`0xC1`), `S` a SET with payload (`0x41`), `S0` a SET with no payload |
| New wValue | The new `wValue` as an expression of the old one, called `old`. `old` means unchanged, `old & 0xFF` its low byte, `old >> 8` its high byte, `0` the constant 0 |
| Payload | For a SET, what to send. `unchanged` means the 1.1.6 payload bytes. For a former write-as-read, the 1-byte payload built from the old `wValue`, for example `u8 old >> 8` |
| Bytes | The exact GET reply length or SET payload length. `var` replies are explained in section 7.1. `chunk` is any length from 1 to the rest of the bulk blob |

A row whose New command is `none` and whose Payload says "1.1.6 STALLed it" or "1.1.6 ignored it" covers selector values that 1.1.6 refused too. They have no equivalent.

### 6.2 Every old opcode

<!-- VC:BEGIN LEGACY_MAP -->

| Old | Old name | Old dir | Old wValue | Change | New command | ID | New dir | New wValue | Payload | Bytes |
|---|---|---|---|---|---|---|---|---|---|---|
| 0x03 | [`OUTPUT_LATENCY`](#output_latency-0x03) | W | any | same | `OUTPUT.LATENCY` | `0x0420` | S | `0` | unchanged | 1 |
| 0x03 | [`OUTPUT_LATENCY`](#output_latency-0x03) | R | 0 | split | `OUTPUT.LATENCY_STATUS` | `0x0421` | G | `0` | - | 12 |
| 0x03 | [`OUTPUT_LATENCY`](#output_latency-0x03) | R | 1 | split | `OUTPUT.LATENCY_OPTIONS` | `0x0422` | G | `0` | - | var |
| 0x03 | [`OUTPUT_LATENCY`](#output_latency-0x03) | R | any other | split | none | - | - | - | 1.1.6 STALLed it | - |
| 0x04 | `SET_CS_AUX_STATE` | W | any | folded | `CS.AUX_STATE` | `0x0A60` | S | `old & 0xFF` | unchanged | 1 |
| 0x05 | [`GET_CS_AUX_STATE`](#get_cs_aux_state-0x05) | R | 0xFFFF | split | `CS.AUX_ALL` | `0x0A62` | G | `0` | - | 48 |
| 0x05 | [`GET_CS_AUX_STATE`](#get_cs_aux_state-0x05) | R | any other | split | `CS.AUX_STATE` | `0x0A60` | G | `old` | - | 1 |
| 0x06 | `SET_CS_AUX_LEVEL` | W | any | folded | `CS.AUX_LEVEL` | `0x0A61` | S | `old & 0xFF` | unchanged | 2 |
| 0x07 | `GET_CS_AUX_LEVEL` | R | any | folded | `CS.AUX_LEVEL` | `0x0A61` | G | `old` | - | 2 |
| 0x08 | `RTA_SET_CONFIG` | W | any | folded | `MEAS.RTA_CONFIG` | `0x0920` | S | `0` | unchanged | 12 |
| 0x09 | `RTA_GET_CONFIG` | R | any | folded | `MEAS.RTA_CONFIG` | `0x0920` | G | `0` | - | 12 |
| 0x0A | [`RTA_GET_CAPS`](#rta_get_caps-0x0a) | R | 0 | split | `MEAS.RTA_CAPS` | `0x0921` | G | `0` | - | 16 |
| 0x0A | [`RTA_GET_CAPS`](#rta_get_caps-0x0a) | R | any other | split | `MEAS.RTA_BAND_CENTRES` | `0x0922` | G | `old - 1` | - | var |
| 0x0B | `RTA_GET_BANDS` | R | any | same | `MEAS.RTA_BANDS` | `0x0923` | G | `old` | - | 82 |
| 0x0C | `RTA_GET_BINS` | R | any | same | `MEAS.RTA_BINS` | `0x0924` | G | `old` | - | var |
| 0x0D | `RTA_GET_STATUS` | R | any | same | `MEAS.RTA_STATUS` | `0x0925` | G | `0` | - | 24 |
| 0x0E | [`RTA_CONTROL`](#rta_control-0x0e-and-siggen_control-0xa6) | W-as-R | old & 0xFF = 0 | split | `MEAS.RTA_STOP` | `0x0929` | S0 | `0` | none | 0 |
| 0x0E | [`RTA_CONTROL`](#rta_control-0x0e-and-siggen_control-0xa6) | W-as-R | old & 0xFF = 1 | split | `MEAS.RTA_START` | `0x0928` | S0 | `0` | none | 0 |
| 0x0E | [`RTA_CONTROL`](#rta_control-0x0e-and-siggen_control-0xa6) | W-as-R | old & 0xFF = 2 | split | `MEAS.RTA_RESET` | `0x092A` | S0 | `0` | none | 0 |
| 0x0E | [`RTA_CONTROL`](#rta_control-0x0e-and-siggen_control-0xa6) | W-as-R | any other | split | none | - | - | - | 1.1.6 STALLed it | - |
| 0x0F | `RTA_GET_BANDS_ALL` | R | any | same | `MEAS.RTA_BANDS_ALL` | `0x0926` | G | `0` | - | var |
| 0x10 | `SET_SUBHARM` | W | any | folded | `FX.SUBHARM_ENABLE` | `0x0820` | S | `0` | unchanged | 1 |
| 0x11 | `GET_SUBHARM` | R | any | folded | `FX.SUBHARM_ENABLE` | `0x0820` | G | `0` | - | 1 |
| 0x12 | `SET_SUBHARM_LOW` | W | any | folded | `FX.SUBHARM_LOW` | `0x0821` | S | `0` | unchanged | 4 |
| 0x13 | `GET_SUBHARM_LOW` | R | any | folded | `FX.SUBHARM_LOW` | `0x0821` | G | `0` | - | 4 |
| 0x14 | `SET_SUBHARM_HIGH` | W | any | folded | `FX.SUBHARM_HIGH` | `0x0822` | S | `0` | unchanged | 4 |
| 0x15 | `GET_SUBHARM_HIGH` | R | any | folded | `FX.SUBHARM_HIGH` | `0x0822` | G | `0` | - | 4 |
| 0x16 | `SET_SUBHARM_BOOST` | W | any | folded | `FX.SUBHARM_BOOST` | `0x0824` | S | `0` | unchanged | 4 |
| 0x17 | `GET_SUBHARM_BOOST` | R | any | folded | `FX.SUBHARM_BOOST` | `0x0824` | G | `0` | - | 4 |
| 0x18 | `SET_SUBHARM_MASK` | W | any | folded | `FX.SUBHARM_MASK` | `0x0825` | S | `0` | unchanged | 2 |
| 0x19 | `GET_SUBHARM_MASK` | R | any | folded | `FX.SUBHARM_MASK` | `0x0825` | G | `0` | - | 2 |
| 0x1A | `GET_SUBHARM_HEADROOM` | R | any | same | `FX.SUBHARM_HEADROOM` | `0x082E` | G | `0` | - | 4 |
| 0x1B | `SET_SUBHARM_TOP` | W | any | folded | `FX.SUBHARM_TOP` | `0x0823` | S | `0` | unchanged | 4 |
| 0x1C | `GET_SUBHARM_TOP` | R | any | folded | `FX.SUBHARM_TOP` | `0x0823` | G | `0` | - | 4 |
| 0x1D | `SET_SUBHARM_SELECT` | W | any | folded | `FX.SUBHARM_SELECT` | `0x0826` | S | `0` | unchanged | 1 |
| 0x1E | `GET_SUBHARM_SELECT` | R | any | folded | `FX.SUBHARM_SELECT` | `0x0826` | G | `0` | - | 1 |
| 0x1F | `GET_SUBHARM_METER` | R | any | same | `FX.SUBHARM_METER` | `0x082F` | G | `0` | - | var |
| 0x20 | `SET_CS_GROUP` | W | any | folded | `CS.GROUP` | `0x0A40` | S | `old` | unchanged | 40 |
| 0x21 | `GET_CS_GROUP` | R | any | folded | `CS.GROUP` | `0x0A40` | G | `old` | - | 40 |
| 0x22 | `SET_CS_MACRO` | W | any | same | `CS.MACRO_HEADER` | `0x0A51` | S | `old` | unchanged | 36 |
| 0x23 | `GET_CS_MACRO` | R | any | same | `CS.MACRO` | `0x0A50` | G | `old` | - | 132 |
| 0x24 | `SET_CS_MACRO_STEP` | W | any | same | `CS.MACRO_STEP` | `0x0A52` | S | `old` | unchanged | 12 |
| 0x25 | [`CS_MACRO_FIRE`](#cs_macro_fire-0x25) | W-as-R | 0xFFFF | split | `CS.MACRO_CANCEL` | `0x0A59` | S0 | `0` | none | 0 |
| 0x25 | [`CS_MACRO_FIRE`](#cs_macro_fire-0x25) | W-as-R | any other | split | `CS.MACRO_FIRE` | `0x0A58` | S0 | `old` | none | 0 |
| 0x26 | `GET_CS_EXT_STATUS` | R | any | same | `CS.EXT_STATUS` | `0x0A13` | G | `0` | - | 24 |
| 0x27 | `SET_CS_DISPLAY_CFG` | W | any | same | `CS.DISPLAY_CONFIG` | `0x0A70` | S | `0` | unchanged | 12 |
| 0x28 | [`GET_CS_DISPLAY_CFG`](#get_cs_display_cfg-0x28) | R | any | custom | `CS.DISPLAY_CAPS` G, `CS.DISPLAY_CONFIG` G | `0x0A71`, `0x0A70` | - | see recipe | [see recipe](#get_cs_display_cfg-0x28) | see recipe |
| 0x29 | `SET_CS_DISPLAY_PAGE` | W | any | folded | `CS.DISPLAY_PAGE` | `0x0A72` | S | `old` | unchanged | 4 |
| 0x2A | `GET_CS_DISPLAY_PAGE` | R | any | folded | `CS.DISPLAY_PAGE` | `0x0A72` | G | `old` | - | 4 |
| 0x2B | `GET_CS_DISPLAY_STATUS` | R | any | same | `CS.DISPLAY_STATUS` | `0x0A73` | G | `0` | - | 8 |
| 0x2C | `SET_SUBHARM_SOLO` | W | any | folded | `FX.SUBHARM_SOLO` | `0x082B` | S | `0` | unchanged | 1 |
| 0x2D | `GET_SUBHARM_SOLO` | R | any | folded | `FX.SUBHARM_SOLO` | `0x082B` | G | `0` | - | 1 |
| 0x2E | `SET_SUBHARM_LINK` | W | any | folded | `FX.SUBHARM_LINK` | `0x082A` | S | `0` | unchanged | 1 |
| 0x2F | `GET_SUBHARM_LINK` | R | any | folded | `FX.SUBHARM_LINK` | `0x082A` | G | `0` | - | 1 |
| 0x30 | `SET_PSYBASS` | W | any | folded | `FX.PSYBASS_ENABLE` | `0x0840` | S | `0` | unchanged | 1 |
| 0x31 | `GET_PSYBASS` | R | any | folded | `FX.PSYBASS_ENABLE` | `0x0840` | G | `0` | - | 1 |
| 0x32 | `SET_PSYBASS_CUTOFF` | W | any | folded | `FX.PSYBASS_CUTOFF` | `0x0841` | S | `0` | unchanged | 4 |
| 0x33 | `GET_PSYBASS_CUTOFF` | R | any | folded | `FX.PSYBASS_CUTOFF` | `0x0841` | G | `0` | - | 4 |
| 0x34 | `SET_PSYBASS_HARMONICS` | W | any | folded | `FX.PSYBASS_HARMONICS` | `0x0842` | S | `0` | unchanged | 4 |
| 0x35 | `GET_PSYBASS_HARMONICS` | R | any | folded | `FX.PSYBASS_HARMONICS` | `0x0842` | G | `0` | - | 4 |
| 0x36 | `SET_PSYBASS_DRIVE` | W | any | folded | `FX.PSYBASS_DRIVE` | `0x0843` | S | `0` | unchanged | 4 |
| 0x37 | `GET_PSYBASS_DRIVE` | R | any | folded | `FX.PSYBASS_DRIVE` | `0x0843` | G | `0` | - | 4 |
| 0x38 | `SET_PSYBASS_CHARACTER` | W | any | folded | `FX.PSYBASS_CHARACTER` | `0x0844` | S | `0` | unchanged | 4 |
| 0x39 | `GET_PSYBASS_CHARACTER` | R | any | folded | `FX.PSYBASS_CHARACTER` | `0x0844` | G | `0` | - | 4 |
| 0x3A | `SET_PSYBASS_ORIGINAL` | W | any | folded | `FX.PSYBASS_ORIGINAL` | `0x0845` | S | `0` | unchanged | 4 |
| 0x3B | `GET_PSYBASS_ORIGINAL` | R | any | folded | `FX.PSYBASS_ORIGINAL` | `0x0845` | G | `0` | - | 4 |
| 0x3C | `SET_PSYBASS_MASK` | W | any | folded | `FX.PSYBASS_MASK` | `0x0846` | S | `0` | unchanged | 2 |
| 0x3D | `GET_PSYBASS_MASK` | R | any | folded | `FX.PSYBASS_MASK` | `0x0846` | G | `0` | - | 2 |
| 0x3E | [`SET_TUBE_PARAM`](#set_tube_param-and-get_tube_param-0x3e-0x3f) | W | old & 0xFF = 0 | split | `FX.TUBE_ENABLE` | `0x0860` | S | `0` | unchanged | 4 |
| 0x3E | [`SET_TUBE_PARAM`](#set_tube_param-and-get_tube_param-0x3e-0x3f) | W | old & 0xFF = 1 | split | `FX.TUBE_OUTPUT_MASK` | `0x0861` | S | `0` | unchanged | 4 |
| 0x3E | [`SET_TUBE_PARAM`](#set_tube_param-and-get_tube_param-0x3e-0x3f) | W | old & 0xFF = 2 | split | `FX.TUBE_TYPE` | `0x0862` | S | `0` | unchanged | 4 |
| 0x3E | [`SET_TUBE_PARAM`](#set_tube_param-and-get_tube_param-0x3e-0x3f) | W | old & 0xFF = 3 | split | `FX.TUBE_DRIVE` | `0x0863` | S | `0` | unchanged | 4 |
| 0x3E | [`SET_TUBE_PARAM`](#set_tube_param-and-get_tube_param-0x3e-0x3f) | W | old & 0xFF = 4 | split | `FX.TUBE_BIAS` | `0x0864` | S | `0` | unchanged | 4 |
| 0x3E | [`SET_TUBE_PARAM`](#set_tube_param-and-get_tube_param-0x3e-0x3f) | W | old & 0xFF = 5 | split | `FX.TUBE_ASYM` | `0x0865` | S | `0` | unchanged | 4 |
| 0x3E | [`SET_TUBE_PARAM`](#set_tube_param-and-get_tube_param-0x3e-0x3f) | W | old & 0xFF = 6 | split | `FX.TUBE_HARDNESS` | `0x0866` | S | `0` | unchanged | 4 |
| 0x3E | [`SET_TUBE_PARAM`](#set_tube_param-and-get_tube_param-0x3e-0x3f) | W | old & 0xFF = 7 | split | `FX.TUBE_SAG` | `0x0867` | S | `0` | unchanged | 4 |
| 0x3E | [`SET_TUBE_PARAM`](#set_tube_param-and-get_tube_param-0x3e-0x3f) | W | old & 0xFF = 8 | split | `FX.TUBE_RECTIFIER` | `0x0868` | S | `0` | unchanged | 4 |
| 0x3E | [`SET_TUBE_PARAM`](#set_tube_param-and-get_tube_param-0x3e-0x3f) | W | old & 0xFF = 9 | split | `FX.TUBE_XFMR_ENABLE` | `0x0869` | S | `0` | unchanged | 4 |
| 0x3E | [`SET_TUBE_PARAM`](#set_tube_param-and-get_tube_param-0x3e-0x3f) | W | old & 0xFF = 10 | split | `FX.TUBE_XFMR_DAMPING` | `0x086A` | S | `0` | unchanged | 4 |
| 0x3E | [`SET_TUBE_PARAM`](#set_tube_param-and-get_tube_param-0x3e-0x3f) | W | old & 0xFF = 11 | split | `FX.TUBE_XFMR_RES` | `0x086B` | S | `0` | unchanged | 4 |
| 0x3E | [`SET_TUBE_PARAM`](#set_tube_param-and-get_tube_param-0x3e-0x3f) | W | old & 0xFF = 12 | split | `FX.TUBE_MIX` | `0x086C` | S | `0` | unchanged | 4 |
| 0x3E | [`SET_TUBE_PARAM`](#set_tube_param-and-get_tube_param-0x3e-0x3f) | W | old & 0xFF = 13 | split | `FX.TUBE_TRIM` | `0x086D` | S | `0` | unchanged | 4 |
| 0x3E | [`SET_TUBE_PARAM`](#set_tube_param-and-get_tube_param-0x3e-0x3f) | W | any other | split | none | - | - | - | 1.1.6 ignored it | - |
| 0x3F | [`GET_TUBE_PARAM`](#set_tube_param-and-get_tube_param-0x3e-0x3f) | R | old & 0xFF = 0 | split | `FX.TUBE_ENABLE` | `0x0860` | G | `0` | - | 4 |
| 0x3F | [`GET_TUBE_PARAM`](#set_tube_param-and-get_tube_param-0x3e-0x3f) | R | old & 0xFF = 1 | split | `FX.TUBE_OUTPUT_MASK` | `0x0861` | G | `0` | - | 4 |
| 0x3F | [`GET_TUBE_PARAM`](#set_tube_param-and-get_tube_param-0x3e-0x3f) | R | old & 0xFF = 2 | split | `FX.TUBE_TYPE` | `0x0862` | G | `0` | - | 4 |
| 0x3F | [`GET_TUBE_PARAM`](#set_tube_param-and-get_tube_param-0x3e-0x3f) | R | old & 0xFF = 3 | split | `FX.TUBE_DRIVE` | `0x0863` | G | `0` | - | 4 |
| 0x3F | [`GET_TUBE_PARAM`](#set_tube_param-and-get_tube_param-0x3e-0x3f) | R | old & 0xFF = 4 | split | `FX.TUBE_BIAS` | `0x0864` | G | `0` | - | 4 |
| 0x3F | [`GET_TUBE_PARAM`](#set_tube_param-and-get_tube_param-0x3e-0x3f) | R | old & 0xFF = 5 | split | `FX.TUBE_ASYM` | `0x0865` | G | `0` | - | 4 |
| 0x3F | [`GET_TUBE_PARAM`](#set_tube_param-and-get_tube_param-0x3e-0x3f) | R | old & 0xFF = 6 | split | `FX.TUBE_HARDNESS` | `0x0866` | G | `0` | - | 4 |
| 0x3F | [`GET_TUBE_PARAM`](#set_tube_param-and-get_tube_param-0x3e-0x3f) | R | old & 0xFF = 7 | split | `FX.TUBE_SAG` | `0x0867` | G | `0` | - | 4 |
| 0x3F | [`GET_TUBE_PARAM`](#set_tube_param-and-get_tube_param-0x3e-0x3f) | R | old & 0xFF = 8 | split | `FX.TUBE_RECTIFIER` | `0x0868` | G | `0` | - | 4 |
| 0x3F | [`GET_TUBE_PARAM`](#set_tube_param-and-get_tube_param-0x3e-0x3f) | R | old & 0xFF = 9 | split | `FX.TUBE_XFMR_ENABLE` | `0x0869` | G | `0` | - | 4 |
| 0x3F | [`GET_TUBE_PARAM`](#set_tube_param-and-get_tube_param-0x3e-0x3f) | R | old & 0xFF = 10 | split | `FX.TUBE_XFMR_DAMPING` | `0x086A` | G | `0` | - | 4 |
| 0x3F | [`GET_TUBE_PARAM`](#set_tube_param-and-get_tube_param-0x3e-0x3f) | R | old & 0xFF = 11 | split | `FX.TUBE_XFMR_RES` | `0x086B` | G | `0` | - | 4 |
| 0x3F | [`GET_TUBE_PARAM`](#set_tube_param-and-get_tube_param-0x3e-0x3f) | R | old & 0xFF = 12 | split | `FX.TUBE_MIX` | `0x086C` | G | `0` | - | 4 |
| 0x3F | [`GET_TUBE_PARAM`](#set_tube_param-and-get_tube_param-0x3e-0x3f) | R | old & 0xFF = 13 | split | `FX.TUBE_TRIM` | `0x086D` | G | `0` | - | 4 |
| 0x3F | [`GET_TUBE_PARAM`](#set_tube_param-and-get_tube_param-0x3e-0x3f) | R | any other | split | none | - | - | - | 1.1.6 STALLed it | - |
| 0x42 | [`SET_EQ_PARAM`](#set_eq_param-and-get_eq_param-0x42-0x43) | W | any | custom | `FILTER.BAND` G, `FILTER.BAND` S | `0x0610`, `0x0610` | - | see recipe | [see recipe](#set_eq_param-and-get_eq_param-0x42-0x43) | see recipe |
| 0x43 | [`GET_EQ_PARAM`](#set_eq_param-and-get_eq_param-0x42-0x43) | R | any | custom | `FILTER.BAND` G | `0x0610` | - | see recipe | [see recipe](#set_eq_param-and-get_eq_param-0x42-0x43) | see recipe |
| 0x44 | `SET_PREAMP` | W | any | retired | none | - | - | - | [replacement](#67-retired-opcodes) | - |
| 0x45 | `GET_PREAMP` | R | any | retired | none | - | - | - | [replacement](#67-retired-opcodes) | - |
| 0x46 | `SET_BYPASS` | W | any | folded | `FILTER.EQ_BYPASS` | `0x0612` | S | `0` | unchanged | 1 |
| 0x47 | `GET_BYPASS` | R | any | folded | `FILTER.EQ_BYPASS` | `0x0612` | G | `0` | - | 1 |
| 0x48 | `SET_DELAY` | W | any | folded | `ROUTE.CHANNEL_DELAY` | `0x0530` | S | `old` | unchanged | 4 |
| 0x49 | `GET_DELAY` | R | any | folded | `ROUTE.CHANNEL_DELAY` | `0x0530` | G | `old` | - | 4 |
| 0x4A | `UPMIX_SET_CONFIG` | W | any | folded | `FX.UPMIX_CONFIG` (RP2350 only) | `0x0880` | S | `0` | unchanged | 44 |
| 0x4B | `UPMIX_GET_CONFIG` | R | any | folded | `FX.UPMIX_CONFIG` (RP2350 only) | `0x0880` | G | `0` | - | 44 |
| 0x4C | [`UPMIX_SET_PARAM`](#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | W | 0 | split | `FX.UPMIX_ENABLE` (RP2350 only) | `0x0890` | S | `0` | unchanged | 4 |
| 0x4C | [`UPMIX_SET_PARAM`](#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | W | 1 | split | `FX.UPMIX_CENTER_MODE` (RP2350 only) | `0x0891` | S | `0` | unchanged | 4 |
| 0x4C | [`UPMIX_SET_PARAM`](#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | W | 2 | split | `FX.UPMIX_SURROUND_MODE` (RP2350 only) | `0x0892` | S | `0` | unchanged | 4 |
| 0x4C | [`UPMIX_SET_PARAM`](#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | W | 3 | split | `FX.UPMIX_STRENGTH` (RP2350 only) | `0x0893` | S | `0` | unchanged | 4 |
| 0x4C | [`UPMIX_SET_PARAM`](#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | W | 4 | split | `FX.UPMIX_CENTER_WIDTH` (RP2350 only) | `0x0894` | S | `0` | unchanged | 4 |
| 0x4C | [`UPMIX_SET_PARAM`](#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | W | 5 | split | `FX.UPMIX_THRESHOLD` (RP2350 only) | `0x0895` | S | `0` | unchanged | 4 |
| 0x4C | [`UPMIX_SET_PARAM`](#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | W | 6 | split | `FX.UPMIX_ATTACK` (RP2350 only) | `0x0896` | S | `0` | unchanged | 4 |
| 0x4C | [`UPMIX_SET_PARAM`](#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | W | 7 | split | `FX.UPMIX_RELEASE` (RP2350 only) | `0x0897` | S | `0` | unchanged | 4 |
| 0x4C | [`UPMIX_SET_PARAM`](#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | W | 8 | split | `FX.UPMIX_DET_HPF` (RP2350 only) | `0x0898` | S | `0` | unchanged | 4 |
| 0x4C | [`UPMIX_SET_PARAM`](#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | W | 9 | split | `FX.UPMIX_SUR_DELAY` (RP2350 only) | `0x0899` | S | `0` | unchanged | 4 |
| 0x4C | [`UPMIX_SET_PARAM`](#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | W | 10 | split | `FX.UPMIX_SUR_HPF` (RP2350 only) | `0x089A` | S | `0` | unchanged | 4 |
| 0x4C | [`UPMIX_SET_PARAM`](#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | W | 11 | split | `FX.UPMIX_SUR_LPF` (RP2350 only) | `0x089B` | S | `0` | unchanged | 4 |
| 0x4C | [`UPMIX_SET_PARAM`](#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | W | 12 | split | `FX.UPMIX_DECORR` (RP2350 only) | `0x089C` | S | `0` | unchanged | 4 |
| 0x4C | [`UPMIX_SET_PARAM`](#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | W | 13 | split | `FX.UPMIX_PRESENCE` (RP2350 only) | `0x089D` | S | `0` | unchanged | 4 |
| 0x4C | [`UPMIX_SET_PARAM`](#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | W | any other | split | none | - | - | - | 1.1.6 ignored it | - |
| 0x4D | [`UPMIX_GET_PARAM`](#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | R | 0 | split | `FX.UPMIX_ENABLE` (RP2350 only) | `0x0890` | G | `0` | - | 4 |
| 0x4D | [`UPMIX_GET_PARAM`](#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | R | 1 | split | `FX.UPMIX_CENTER_MODE` (RP2350 only) | `0x0891` | G | `0` | - | 4 |
| 0x4D | [`UPMIX_GET_PARAM`](#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | R | 2 | split | `FX.UPMIX_SURROUND_MODE` (RP2350 only) | `0x0892` | G | `0` | - | 4 |
| 0x4D | [`UPMIX_GET_PARAM`](#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | R | 3 | split | `FX.UPMIX_STRENGTH` (RP2350 only) | `0x0893` | G | `0` | - | 4 |
| 0x4D | [`UPMIX_GET_PARAM`](#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | R | 4 | split | `FX.UPMIX_CENTER_WIDTH` (RP2350 only) | `0x0894` | G | `0` | - | 4 |
| 0x4D | [`UPMIX_GET_PARAM`](#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | R | 5 | split | `FX.UPMIX_THRESHOLD` (RP2350 only) | `0x0895` | G | `0` | - | 4 |
| 0x4D | [`UPMIX_GET_PARAM`](#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | R | 6 | split | `FX.UPMIX_ATTACK` (RP2350 only) | `0x0896` | G | `0` | - | 4 |
| 0x4D | [`UPMIX_GET_PARAM`](#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | R | 7 | split | `FX.UPMIX_RELEASE` (RP2350 only) | `0x0897` | G | `0` | - | 4 |
| 0x4D | [`UPMIX_GET_PARAM`](#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | R | 8 | split | `FX.UPMIX_DET_HPF` (RP2350 only) | `0x0898` | G | `0` | - | 4 |
| 0x4D | [`UPMIX_GET_PARAM`](#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | R | 9 | split | `FX.UPMIX_SUR_DELAY` (RP2350 only) | `0x0899` | G | `0` | - | 4 |
| 0x4D | [`UPMIX_GET_PARAM`](#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | R | 10 | split | `FX.UPMIX_SUR_HPF` (RP2350 only) | `0x089A` | G | `0` | - | 4 |
| 0x4D | [`UPMIX_GET_PARAM`](#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | R | 11 | split | `FX.UPMIX_SUR_LPF` (RP2350 only) | `0x089B` | G | `0` | - | 4 |
| 0x4D | [`UPMIX_GET_PARAM`](#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | R | 12 | split | `FX.UPMIX_DECORR` (RP2350 only) | `0x089C` | G | `0` | - | 4 |
| 0x4D | [`UPMIX_GET_PARAM`](#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | R | 13 | split | `FX.UPMIX_PRESENCE` (RP2350 only) | `0x089D` | G | `0` | - | 4 |
| 0x4D | [`UPMIX_GET_PARAM`](#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | R | any other | split | none | - | - | - | 1.1.6 STALLed it | - |
| 0x4E | `UPMIX_GET_STATUS` | R | any | same | `FX.UPMIX_STATUS` (RP2350 only) | `0x0881` | G | `0` | - | 16 |
| 0x50 | [`GET_STATUS`](#63-get_status-0x50) | R | any | custom | `MEAS.LEVELS` G, `SYS.SYSTEM_STATUS` G, `SYS.PDM_STATS` G, `SYS.SPDIF_TX_STATS` G, `SYS.USB_AUDIO_STATUS` G, `SYS.PIPELINE_STATS` G, `INPUT.STATUS` G, `DIAG.LOOPBACK_STATS` (DSPI_LOOPBACK) G | `0x0910`, `0x0140`, `0x0144`, `0x0145`, `0x0141`, `0x0143`, `0x0311`, `0xF020` | - | see recipe | [see recipe](#63-get_status-0x50) | see recipe |
| 0x51 | `SAVE_PARAMS` | W-as-R | any | retired | none | - | - | - | [replacement](#67-retired-opcodes) | - |
| 0x52 | `SAVE_OUTPUT_CONFIG` | W-as-R | any | action | `STORE.OUTPUT_CONFIG_SAVE` | `0x0221` | S0 | `0` | none | 0 |
| 0x53 | `FACTORY_RESET` | W-as-R | any | action | `STORE.FACTORY_RESET` | `0x0230` | S0 | `0` | none | 0 |
| 0x54 | `SET_INPUT_SOURCE_NAME` | W | any | folded | `INPUT.SOURCE_NAME` | `0x0312` | S | `old` | unchanged | 1-32 |
| 0x55 | `GET_INPUT_SOURCE_NAME` | R | any | folded | `INPUT.SOURCE_NAME` | `0x0312` | G | `old` | - | 32 |
| 0x56 | `SET_DEVICE_NAME` | W | any | folded | `SYS.DEVICE_NAME` | `0x0124` | S | `0` | unchanged | 1-32 |
| 0x57 | `GET_DEVICE_NAME` | R | any | folded | `SYS.DEVICE_NAME` | `0x0124` | G | `0` | - | 32 |
| 0x58 | `SET_LOUDNESS` | W | any | folded | `DYN.LOUDNESS_ENABLE` | `0x0730` | S | `0` | unchanged | 1 |
| 0x59 | `GET_LOUDNESS` | R | any | folded | `DYN.LOUDNESS_ENABLE` | `0x0730` | G | `0` | - | 1 |
| 0x5A | `SET_LOUDNESS_REF` | W | any | folded | `DYN.LOUDNESS_REF` | `0x0731` | S | `0` | unchanged | 4 |
| 0x5B | `GET_LOUDNESS_REF` | R | any | folded | `DYN.LOUDNESS_REF` | `0x0731` | G | `0` | - | 4 |
| 0x5C | `SET_LOUDNESS_INTENSITY` | W | any | folded | `DYN.LOUDNESS_INTENSITY` | `0x0732` | S | `0` | unchanged | 4 |
| 0x5D | `GET_LOUDNESS_INTENSITY` | R | any | folded | `DYN.LOUDNESS_INTENSITY` | `0x0732` | G | `0` | - | 4 |
| 0x5E | `SET_CROSSFEED` | W | any | folded | `FX.CROSSFEED_ENABLE` | `0x0810` | S | `0` | unchanged | 1 |
| 0x5F | `GET_CROSSFEED` | R | any | folded | `FX.CROSSFEED_ENABLE` | `0x0810` | G | `0` | - | 1 |
| 0x60 | `SET_CROSSFEED_PRESET` | W | any | folded | `FX.CROSSFEED_PRESET` | `0x0811` | S | `0` | unchanged | 1 |
| 0x61 | `GET_CROSSFEED_PRESET` | R | any | folded | `FX.CROSSFEED_PRESET` | `0x0811` | G | `0` | - | 1 |
| 0x62 | `SET_CROSSFEED_FREQ` | W | any | folded | `FX.CROSSFEED_FREQ` | `0x0812` | S | `0` | unchanged | 4 |
| 0x63 | `GET_CROSSFEED_FREQ` | R | any | folded | `FX.CROSSFEED_FREQ` | `0x0812` | G | `0` | - | 4 |
| 0x64 | `SET_CROSSFEED_FEED` | W | any | folded | `FX.CROSSFEED_FEED` | `0x0813` | S | `0` | unchanged | 4 |
| 0x65 | `GET_CROSSFEED_FEED` | R | any | folded | `FX.CROSSFEED_FEED` | `0x0813` | G | `0` | - | 4 |
| 0x66 | `SET_CROSSFEED_ITD` | W | any | folded | `FX.CROSSFEED_ITD` | `0x0814` | S | `0` | unchanged | 1 |
| 0x67 | `GET_CROSSFEED_ITD` | R | any | folded | `FX.CROSSFEED_ITD` | `0x0814` | G | `0` | - | 1 |
| 0x68 | `SET_ADAT_INPUT_ENABLE` | W-as-R | any | action | `INPUT.ADAT_RX_ENABLE` (RP2350 only) | `0x0340` | S | `0` | u8 `old & 0xFF` | 1 |
| 0x69 | `GET_ADAT_INPUT_ENABLE` | R | any | same | `INPUT.ADAT_RX_ENABLE` (RP2350 only) | `0x0340` | G | `0` | - | 1 |
| 0x6A | `SET_ADAT_INPUT_PIN` | W-as-R | any | action | `INPUT.ADAT_RX_PIN` (RP2350 only) | `0x0341` | S | `0` | u8 `old & 0xFF` | 1 |
| 0x6B | `GET_ADAT_INPUT_PIN` | R | any | same | `INPUT.ADAT_RX_PIN` (RP2350 only) | `0x0341` | G | `0` | - | 1 |
| 0x6C | `SET_ADAT_INPUT_CLOCK_MODE` | W-as-R | any | action | `INPUT.ADAT_RX_CLOCK_MODE` (RP2350 only) | `0x0342` | S | `0` | u8 `old & 0xFF` | 1 |
| 0x6D | `GET_ADAT_INPUT_CLOCK_MODE` | R | any | same | `INPUT.ADAT_RX_CLOCK_MODE` (RP2350 only) | `0x0342` | G | `0` | - | 1 |
| 0x6E | `GET_ADAT_INPUT_STATUS` | R | any | same | `INPUT.ADAT_RX_STATUS` (RP2350 only) | `0x0343` | G | `0` | - | 20 |
| 0x70 | [`SET_MATRIX_ROUTE`](#set_matrix_route-and-get_matrix_route-0x70-0x71) | W | any | custom | `ROUTE.MATRIX_ROUTE` S | `0x0510` | - | see recipe | [see recipe](#set_matrix_route-and-get_matrix_route-0x70-0x71) | see recipe |
| 0x71 | [`GET_MATRIX_ROUTE`](#set_matrix_route-and-get_matrix_route-0x70-0x71) | R | any | custom | `ROUTE.MATRIX_ROUTE` G | `0x0510` | - | see recipe | [see recipe](#set_matrix_route-and-get_matrix_route-0x70-0x71) | see recipe |
| 0x72 | `SET_OUTPUT_ENABLE` | W | any | folded | `ROUTE.OUTPUT_ENABLE` | `0x0520` | S | `old` | unchanged | 1 |
| 0x73 | `GET_OUTPUT_ENABLE` | R | any | folded | `ROUTE.OUTPUT_ENABLE` | `0x0520` | G | `old` | - | 1 |
| 0x74 | `SET_OUTPUT_GAIN` | W | any | folded | `ROUTE.OUTPUT_GAIN` | `0x0521` | S | `old` | unchanged | 4 |
| 0x75 | `GET_OUTPUT_GAIN` | R | any | folded | `ROUTE.OUTPUT_GAIN` | `0x0521` | G | `old` | - | 4 |
| 0x76 | `SET_OUTPUT_MUTE` | W | any | folded | `ROUTE.OUTPUT_MUTE` | `0x0522` | S | `old` | unchanged | 1 |
| 0x77 | `GET_OUTPUT_MUTE` | R | any | folded | `ROUTE.OUTPUT_MUTE` | `0x0522` | G | `old` | - | 1 |
| 0x78 | `SET_OUTPUT_DELAY` | W | any | folded | `ROUTE.OUTPUT_DELAY` | `0x0523` | S | `old` | unchanged | 4 |
| 0x79 | `GET_OUTPUT_DELAY` | R | any | folded | `ROUTE.OUTPUT_DELAY` | `0x0523` | G | `old` | - | 4 |
| 0x7A | `GET_CORE1_MODE` | R | any | same | `SYS.CORE1_MODE` | `0x0123` | G | `0` | - | 1 |
| 0x7B | `GET_CORE1_CONFLICT` | R | any | same | `ROUTE.OUTPUT_ENABLE_CONFLICT` | `0x0524` | G | `old` | - | 1 |
| 0x7C | `SET_OUTPUT_PIN` | W-as-R | any | action | `OUTPUT.PIN` | `0x0411` | S | `old & 0xFF` | u8 `old >> 8` | 1 |
| 0x7D | `GET_OUTPUT_PIN` | R | any | same | `OUTPUT.PIN` | `0x0411` | G | `old` | - | 1 |
| 0x7E | `GET_SERIAL` | R | any | same | `SYS.SERIAL` | `0x0121` | G | `0` | - | 16 |
| 0x7F | `GET_PLATFORM` | R | any | same | `SYS.PLATFORM` | `0x0120` | G | `0` | - | 7 |
| 0x80 | `GET_BUILD_INFO` | R | any | same | `SYS.BUILD_INFO` | `0x0122` | G | `0` | - | 64 |
| 0x81 | [`LIMITER`](#limiter-0x81) | W | old & 0xFF = 0 | split | `DYN.LIMITER_ENABLE` | `0x0710` | S | `old >> 8` | unchanged | 4 |
| 0x81 | [`LIMITER`](#limiter-0x81) | W | old & 0xFF = 1 | split | `DYN.LIMITER_THRESHOLD` | `0x0711` | S | `old >> 8` | unchanged | 4 |
| 0x81 | [`LIMITER`](#limiter-0x81) | W | old & 0xFF = 2 | split | `DYN.LIMITER_RELEASE` | `0x0712` | S | `old >> 8` | unchanged | 4 |
| 0x81 | [`LIMITER`](#limiter-0x81) | W | old & 0xFF = 3 | split | `DYN.LIMITER_LINK_GROUP` | `0x0713` | S | `old >> 8` | unchanged | 4 |
| 0x81 | [`LIMITER`](#limiter-0x81) | W | any other | split | none | - | - | - | 1.1.6 ignored it | - |
| 0x81 | [`LIMITER`](#limiter-0x81) | R | old & 0xFF = 0 | split | `DYN.LIMITER_ENABLE` | `0x0710` | G | `old >> 8` | - | 4 |
| 0x81 | [`LIMITER`](#limiter-0x81) | R | old & 0xFF = 1 | split | `DYN.LIMITER_THRESHOLD` | `0x0711` | G | `old >> 8` | - | 4 |
| 0x81 | [`LIMITER`](#limiter-0x81) | R | old & 0xFF = 2 | split | `DYN.LIMITER_RELEASE` | `0x0712` | G | `old >> 8` | - | 4 |
| 0x81 | [`LIMITER`](#limiter-0x81) | R | old & 0xFF = 3 | split | `DYN.LIMITER_LINK_GROUP` | `0x0713` | G | `old >> 8` | - | 4 |
| 0x81 | [`LIMITER`](#limiter-0x81) | R | old & 0xFF = 128 | split | `DYN.LIMITER_METERS` | `0x0718` | G | `0` | - | var |
| 0x81 | [`LIMITER`](#limiter-0x81) | R | old & 0xFF = 129 | split | `DYN.LIMITER_STATUS` | `0x0719` | G | `0` | - | 4 |
| 0x81 | [`LIMITER`](#limiter-0x81) | R | any other | split | none | - | - | - | 1.1.6 STALLed it | - |
| 0x82 | `GET_CLOCK_DIAG` | R | any | same | `DIAG.CLOCK_DIAG` (DSPI_CLOCK_DIAG) | `0xF010` | G | `0` | - | var |
| 0x83 | [`CLEAR_CLIPS`](#clear_clips-0x83) | W-as-R | any | custom | `MEAS.CLIP_FLAGS` G, `MEAS.CLIP_CLEAR` S | `0x0911`, `0x0912` | - | see recipe | [see recipe](#clear_clips-0x83) | see recipe |
| 0x84 | `SET_CS_BINDING` | W | any | folded | `CS.BINDING` | `0x0A20` | S | `old` | unchanged | 24 |
| 0x85 | `GET_CS_BINDING` | R | any | folded | `CS.BINDING` | `0x0A20` | G | `old` | - | 24 |
| 0x86 | [`GET_CS_CAPS`](#get_cs_caps-0x86) | R | 0xFFFF | split | `CS.CAPS` | `0x0A10` | G | `0` | - | 52 |
| 0x86 | [`GET_CS_CAPS`](#get_cs_caps-0x86) | R | any other | split | `CS.NOUN_DESC` | `0x0A11` | G | `old` | - | 12 |
| 0x87 | `GET_CS_STATUS` | R | any | same | `CS.STATUS` | `0x0A12` | G | `0` | - | 41 |
| 0x88 | `SET_I2S_CLOCK_MODE` | W | any | folded | `CLOCK.I2S_CLOCK_MODE` | `0x0C12` | S | `0` | unchanged | 1 |
| 0x89 | `GET_I2S_CLOCK_MODE` | R | any | folded | `CLOCK.I2S_CLOCK_MODE` | `0x0C12` | G | `0` | - | 1 |
| 0x8A | `GET_I2S_SLAVE_STATUS` | R | any | same | `CLOCK.I2S_SLAVE_STATUS` | `0x0C13` | G | `0` | - | 16 |
| 0x8B | `SET_CS_NAME` | W | any | folded | `CS.SLOT_NAME` | `0x0A21` | S | `old` | first 32 bytes | 1-32 |
| 0x8C | `GET_CS_NAME` | R | any | folded | `CS.SLOT_NAME` | `0x0A21` | G | `old & 0xFF` | - | 32 |
| 0x8D | `SET_CS_IR_CMD` | W | any | folded | `CS.IR_CMD` | `0x0A30` | S | `old` | unchanged | 16 |
| 0x8E | `GET_CS_IR_CMD` | R | any | folded | `CS.IR_CMD` | `0x0A30` | G | `old` | - | 16 |
| 0x8F | [`CS_IR_LEARN`](#cs_ir_learn-0x8f) | W-as-R | 0 | split | `CS.IR_LEARN_CANCEL` | `0x0A33` | S0 | `0` | none | 0 |
| 0x8F | [`CS_IR_LEARN`](#cs_ir_learn-0x8f) | W-as-R | 1 | split | `CS.IR_LEARN_ARM` | `0x0A32` | S0 | `0` | none | 0 |
| 0x8F | [`CS_IR_LEARN`](#cs_ir_learn-0x8f) | W-as-R | 2 | split | `CS.IR_LEARN_STATUS` | `0x0A31` | G | `0` | - | 8 |
| 0x8F | [`CS_IR_LEARN`](#cs_ir_learn-0x8f) | W-as-R | any other | split | none | - | - | - | 1.1.6 STALLed it | - |
| 0x90 | `PRESET_SAVE` | W-as-R | any | action | `STORE.PRESET_SAVE` | `0x0210` | S0 | `old & 0xFF` | none | 0 |
| 0x91 | `PRESET_LOAD` | W-as-R | any | action | `STORE.PRESET_LOAD` | `0x0211` | S0 | `old & 0xFF` | none | 0 |
| 0x92 | `PRESET_DELETE` | W-as-R | any | action | `STORE.PRESET_DELETE` | `0x0212` | S0 | `old & 0xFF` | none | 0 |
| 0x93 | `PRESET_GET_NAME` | R | any | folded | `STORE.PRESET_NAME` | `0x0213` | G | `old & 0xFF` | - | 32 |
| 0x94 | `PRESET_SET_NAME` | W | any | folded | `STORE.PRESET_NAME` | `0x0213` | S | `old` | unchanged | 1-32 |
| 0x95 | `PRESET_GET_DIR` | R | any | same | `STORE.DIRECTORY` | `0x0215` | G | `0` | - | 7 |
| 0x96 | `PRESET_SET_STARTUP` | W | any | same | `STORE.STARTUP` | `0x0216` | S | `0` | unchanged | 2 |
| 0x97 | [`PRESET_GET_STARTUP`](#preset_get_startup-0x97) | R | any | custom | `STORE.STARTUP` G, `STORE.DIRECTORY` G | `0x0216`, `0x0215` | - | see recipe | [see recipe](#preset_get_startup-0x97) | see recipe |
| 0x98 | `SET_OUTPUT_CONFIG_MODE` | W | any | folded | `STORE.OUTPUT_CONFIG_MODE` | `0x0220` | S | `0` | unchanged | 1 |
| 0x99 | `GET_OUTPUT_CONFIG_MODE` | R | any | folded | `STORE.OUTPUT_CONFIG_MODE` | `0x0220` | G | `0` | - | 1 |
| 0x9A | `PRESET_GET_ACTIVE` | R | any | same | `STORE.PRESET_ACTIVE` | `0x0214` | G | `0` | - | 1 |
| 0x9B | `SET_CHANNEL_NAME` | W | any | folded | `ROUTE.CHANNEL_NAME` | `0x0531` | S | `old` | unchanged | 1-32 |
| 0x9C | `GET_CHANNEL_NAME` | R | any | folded | `ROUTE.CHANNEL_NAME` | `0x0531` | G | `old` | - | 32 |
| 0x9D | `CS_SAVE` | W-as-R | any | action | `CS.SAVE` | `0x0A18` | S0 | `0` | none | 0 |
| 0x9E | `CS_REVERT` | W-as-R | any | action | `CS.REVERT` | `0x0A19` | S0 | `0` | none | 0 |
| 0xA0 | `GET_ALL_PARAMS` | R | any | same | `STORE.ALL_PARAMS` | `0x0240` | G | `0` | - | 6136 |
| 0xA1 | [`SET_ALL_PARAMS`](#set_all_params-0xa1) | W | any | custom | `STORE.ALL_PARAMS` S | `0x0240` | - | see recipe | [see recipe](#set_all_params-0xa1) | see recipe |
| 0xA2 | `GET_ALL_PARAMS_CHUNK` | R | any | folded | `STORE.ALL_PARAMS_CHUNK` | `0x0241` | G | `old` | - | var |
| 0xA3 | `SET_ALL_PARAMS_CHUNK` | W | any | folded | `STORE.ALL_PARAMS_CHUNK` | `0x0241` | S | `old` | unchanged | chunk |
| 0xA4 | `SIGGEN_SET_CONFIG` | W | any | folded | `MEAS.SIGGEN_CONFIG` | `0x0940` | S | `0` | unchanged | 36 |
| 0xA5 | `SIGGEN_GET_CONFIG` | R | any | folded | `MEAS.SIGGEN_CONFIG` | `0x0940` | G | `0` | - | 36 |
| 0xA6 | [`SIGGEN_CONTROL`](#rta_control-0x0e-and-siggen_control-0xa6) | W-as-R | old & 0xFF = 0 | split | `MEAS.SIGGEN_STOP` | `0x0949` | S0 | `0` | none | 0 |
| 0xA6 | [`SIGGEN_CONTROL`](#rta_control-0x0e-and-siggen_control-0xa6) | W-as-R | old & 0xFF = 1 | split | `MEAS.SIGGEN_START` | `0x0948` | S0 | `0` | none | 0 |
| 0xA6 | [`SIGGEN_CONTROL`](#rta_control-0x0e-and-siggen_control-0xa6) | W-as-R | old & 0xFF = 2 | split | `MEAS.SIGGEN_STOP_NOW` | `0x094A` | S0 | `0` | none | 0 |
| 0xA6 | [`SIGGEN_CONTROL`](#rta_control-0x0e-and-siggen_control-0xa6) | W-as-R | any other | split | none | - | - | - | 1.1.6 STALLed it | - |
| 0xA7 | `SIGGEN_GET_STATUS` | R | any | same | `MEAS.SIGGEN_STATUS` | `0x0941` | G | `0` | - | 16 |
| 0xA8 | [`SIGGEN_GET_CAPS`](#siggen_get_caps-0xa8) | R | 0xFFFF | split | `MEAS.SIGGEN_CAPS` | `0x0942` | G | `0` | - | 8 |
| 0xA8 | [`SIGGEN_GET_CAPS`](#siggen_get_caps-0xa8) | R | any other | split | `MEAS.SIGGEN_TYPE_DESC` | `0x0943` | G | `old` | - | 62 |
| 0xA9 | `SET_SUBHARM_DEPTH` | W | any | folded | `FX.SUBHARM_DEPTH` | `0x0827` | S | `0` | unchanged | 4 |
| 0xAA | `GET_SUBHARM_DEPTH` | R | any | folded | `FX.SUBHARM_DEPTH` | `0x0827` | G | `0` | - | 4 |
| 0xAB | `SET_SUBHARM_HOLD` | W | any | folded | `FX.SUBHARM_HOLD` | `0x0828` | S | `0` | unchanged | 4 |
| 0xAC | `GET_SUBHARM_HOLD` | R | any | folded | `FX.SUBHARM_HOLD` | `0x0828` | G | `0` | - | 4 |
| 0xAD | `SET_SUBHARM_CEILING` | W | any | folded | `FX.SUBHARM_CEILING` | `0x0829` | S | `0` | unchanged | 4 |
| 0xAE | `GET_SUBHARM_CEILING` | R | any | folded | `FX.SUBHARM_CEILING` | `0x0829` | G | `0` | - | 4 |
| 0xB0 | `GET_BUFFER_STATS` | R | any | same | `SYS.BUFFER_STATS` | `0x0142` | G | `0` | - | 44 |
| 0xB1 | `RESET_BUFFER_STATS` | W-as-R | any | action | `SYS.STATS_RESET` | `0x014F` | S | `0` | u8 `old & 0xFF` | 1 |
| 0xB2 | `GET_USB_ERROR_STATS` | R | any | retired | none | - | - | - | [replacement](#67-retired-opcodes) | - |
| 0xB3 | `RESET_USB_ERROR_STATS` | W-as-R | any | retired | none | - | - | - | [replacement](#67-retired-opcodes) | - |
| 0xB4 | `SET_LEVELLER_ENABLE` | W | any | folded | `DYN.LEVELLER_ENABLE` | `0x0720` | S | `0` | unchanged | 1 |
| 0xB5 | `GET_LEVELLER_ENABLE` | R | any | folded | `DYN.LEVELLER_ENABLE` | `0x0720` | G | `0` | - | 1 |
| 0xB6 | `SET_LEVELLER_AMOUNT` | W | any | folded | `DYN.LEVELLER_AMOUNT` | `0x0721` | S | `0` | unchanged | 4 |
| 0xB7 | `GET_LEVELLER_AMOUNT` | R | any | folded | `DYN.LEVELLER_AMOUNT` | `0x0721` | G | `0` | - | 4 |
| 0xB8 | `SET_LEVELLER_SPEED` | W | any | folded | `DYN.LEVELLER_SPEED` | `0x0722` | S | `0` | unchanged | 1 |
| 0xB9 | `GET_LEVELLER_SPEED` | R | any | folded | `DYN.LEVELLER_SPEED` | `0x0722` | G | `0` | - | 1 |
| 0xBA | `SET_LEVELLER_MAX_GAIN` | W | any | folded | `DYN.LEVELLER_MAX_GAIN` | `0x0723` | S | `0` | unchanged | 4 |
| 0xBB | `GET_LEVELLER_MAX_GAIN` | R | any | folded | `DYN.LEVELLER_MAX_GAIN` | `0x0723` | G | `0` | - | 4 |
| 0xBC | `SET_LEVELLER_LOOKAHEAD` | W | any | folded | `DYN.LEVELLER_LOOKAHEAD` | `0x0724` | S | `0` | unchanged | 1 |
| 0xBD | `GET_LEVELLER_LOOKAHEAD` | R | any | folded | `DYN.LEVELLER_LOOKAHEAD` | `0x0724` | G | `0` | - | 1 |
| 0xBE | `SET_LEVELLER_GATE` | W | any | folded | `DYN.LEVELLER_GATE` | `0x0725` | S | `0` | unchanged | 4 |
| 0xBF | `GET_LEVELLER_GATE` | R | any | folded | `DYN.LEVELLER_GATE` | `0x0725` | G | `0` | - | 4 |
| 0xC0 | `SET_OUTPUT_TYPE` | W-as-R | any | action | `OUTPUT.TYPE` | `0x0410` | S | `old & 0xFF` | u8 `old >> 8` | 1 |
| 0xC1 | `GET_OUTPUT_TYPE` | R | any | same | `OUTPUT.TYPE` | `0x0410` | G | `old` | - | 1 |
| 0xC2 | `SET_I2S_BCK_PIN` | W-as-R | any | action | `CLOCK.I2S_BCK_PIN` | `0x0C10` | S | `old >> 8` | u8 `old & 0xFF` | 1 |
| 0xC3 | `GET_I2S_BCK_PIN` | R | any | same | `CLOCK.I2S_BCK_PIN` | `0x0C10` | G | `old` | - | 1 |
| 0xC4 | `SET_MCK_ENABLE` | W-as-R | any | action | `CLOCK.MCK_ENABLE` | `0x0C20` | S | `0` | u8 `old != 0` (1 or 0) | 1 |
| 0xC5 | `GET_MCK_ENABLE` | R | any | same | `CLOCK.MCK_ENABLE` | `0x0C20` | G | `0` | - | 1 |
| 0xC6 | `SET_MCK_PIN` | W-as-R | any | action | `CLOCK.MCK_PIN` | `0x0C21` | S | `0` | u8 `old & 0xFF` | 1 |
| 0xC7 | `GET_MCK_PIN` | R | any | same | `CLOCK.MCK_PIN` | `0x0C21` | G | `0` | - | 1 |
| 0xC8 | `SET_MCK_MULTIPLIER` | W-as-R | any | action | `CLOCK.MCK_MULTIPLIER` | `0x0C22` | S | `0` | u8 `min(old, 255)` | 1 |
| 0xC9 | `GET_MCK_MULTIPLIER` | R | any | same | `CLOCK.MCK_MULTIPLIER` | `0x0C22` | G | `0` | - | 1 |
| 0xCA | `SET_ADAT_ENABLE` | W-as-R | any | action | `OUTPUT.ADAT_TX_ENABLE` (RP2350 only) | `0x0430` | S | `0` | u8 `old & 0xFF` | 1 |
| 0xCB | `GET_ADAT_ENABLE` | R | any | same | `OUTPUT.ADAT_TX_ENABLE` (RP2350 only) | `0x0430` | G | `0` | - | 1 |
| 0xCC | `SET_ADAT_PIN` | W-as-R | any | action | `OUTPUT.ADAT_TX_PIN` (RP2350 only) | `0x0431` | S | `0` | u8 `old & 0xFF` | 1 |
| 0xCD | `GET_ADAT_PIN` | R | any | same | `OUTPUT.ADAT_TX_PIN` (RP2350 only) | `0x0431` | G | `0` | - | 1 |
| 0xCE | `GET_ADAT_STATUS` | R | any | same | `OUTPUT.ADAT_TX_STATUS` (RP2350 only) | `0x0432` | G | `0` | - | 8 |
| 0xD0 | `SET_PREAMP_CH` | W | any | folded | `ROUTE.PREAMP` | `0x0540` | S | `old` | unchanged | 4 |
| 0xD1 | `GET_PREAMP_CH` | R | any | folded | `ROUTE.PREAMP` | `0x0540` | G | `old` | - | 4 |
| 0xD2 | `SET_MASTER_VOLUME` | W | any | folded | `ROUTE.MASTER_VOLUME` | `0x0550` | S | `0` | unchanged | 4 |
| 0xD3 | `GET_MASTER_VOLUME` | R | any | folded | `ROUTE.MASTER_VOLUME` | `0x0550` | G | `0` | - | 4 |
| 0xD4 | `SET_MASTER_VOLUME_MODE` | W | any | folded | `ROUTE.MASTER_VOLUME_MODE` | `0x0551` | S | `0` | unchanged | 1 |
| 0xD5 | `GET_MASTER_VOLUME_MODE` | R | any | folded | `ROUTE.MASTER_VOLUME_MODE` | `0x0551` | G | `0` | - | 1 |
| 0xD6 | `SAVE_MASTER_VOLUME` | W-as-R | any | action | `ROUTE.MASTER_VOLUME_SAVE` | `0x0552` | S0 | `0` | none | 0 |
| 0xD7 | `GET_SAVED_MASTER_VOLUME` | R | any | same | `ROUTE.MASTER_VOLUME_SAVED` | `0x0553` | G | `0` | - | 4 |
| 0xD8 | `SET_BAND_BYPASS` | W | any | folded | `FILTER.BAND_BYPASS` | `0x0611` | S | `old` | unchanged | 1 |
| 0xD9 | `GET_BAND_BYPASS` | R | any | folded | `FILTER.BAND_BYPASS` | `0x0611` | G | `old` | - | 1 |
| 0xDA | `SET_USER_VOLUME` | W | any | folded | `ROUTE.USER_VOLUME` | `0x0560` | S | `0` | unchanged | 4 |
| 0xDB | `GET_USER_VOLUME` | R | any | folded | `ROUTE.USER_VOLUME` | `0x0560` | G | `0` | - | 4 |
| 0xDC | `SET_USER_MUTE` | W | any | folded | `ROUTE.USER_MUTE` | `0x0561` | S | `0` | unchanged | 1 |
| 0xDD | `GET_USER_MUTE` | R | any | folded | `ROUTE.USER_MUTE` | `0x0561` | G | `0` | - | 1 |
| 0xDE | `SET_LEVELLER_MASKS` | W | any | folded | `DYN.LEVELLER_MASKS` | `0x0726` | S | `0` | unchanged | 2 |
| 0xDF | `GET_LEVELLER_MASKS` | R | any | folded | `DYN.LEVELLER_MASKS` | `0x0726` | G | `0` | - | 2 |
| 0xE0 | `SET_INPUT_SOURCE` | W | any | folded | `INPUT.SOURCE` | `0x0310` | S | `0` | unchanged | 1 |
| 0xE1 | `GET_INPUT_SOURCE` | R | any | folded | `INPUT.SOURCE` | `0x0310` | G | `0` | - | 1 |
| 0xE2 | `GET_SPDIF_RX_STATUS` | R | any | same | `INPUT.SPDIF_RX_STATUS` | `0x0320` | G | `0` | - | 16 |
| 0xE3 | `GET_SPDIF_RX_CH_STATUS` | R | any | same | `INPUT.SPDIF_RX_CH_STATUS` | `0x0321` | G | `0` | - | 24 |
| 0xE4 | `SET_SPDIF_RX_PIN` | W-as-R | any | action | `INPUT.SPDIF_RX_PIN` | `0x0322` | S | `old >> 8` | u8 `old & 0xFF` | 1 |
| 0xE5 | `GET_SPDIF_RX_PIN` | R | any | same | `INPUT.SPDIF_RX_PIN` | `0x0322` | G | `old` | - | 1 |
| 0xE6 | `SET_LG_SOUND_SYNC_ENABLE` | W | any | folded | `INPUT.LG_SOUND_SYNC_ENABLE` | `0x0350` | S | `0` | unchanged | 1 |
| 0xE7 | `GET_LG_SOUND_SYNC_ENABLE` | R | any | folded | `INPUT.LG_SOUND_SYNC_ENABLE` | `0x0350` | G | `0` | - | 1 |
| 0xE8 | `GET_LG_SOUND_SYNC_STATUS` | R | any | same | `INPUT.LG_SOUND_SYNC_STATUS` | `0x0351` | G | `0` | - | 16 |
| 0xE9 | `SET_SPDIF_INPUT_ENABLE` | W-as-R | any | action | `INPUT.SPDIF_RX_ENABLE` | `0x0323` | S | `old >> 8` | u8 `old & 0xFF != 0` (1 or 0) | 1 |
| 0xEA | `SET_DAC_HW_MUTE_CONFIG` | W | any | folded | `OUTPUT.DAC_MUTE_CONFIG` | `0x0440` | S | `0` | unchanged | 16 |
| 0xEB | `GET_DAC_HW_MUTE_CONFIG` | R | any | folded | `OUTPUT.DAC_MUTE_CONFIG` | `0x0440` | G | `0` | - | 16 |
| 0xEC | `TEST_DAC_HW_MUTE` | W-as-R | any | action | `OUTPUT.DAC_MUTE_TEST` | `0x0441` | S0 | `0` | none | 0 |
| 0xED | `SET_INPUT_RATE` | W | any | same | `INPUT.I2S_RX_RATE` | `0x0332` | S | `0` | unchanged | 4 |
| 0xEE | [`GET_INPUT_RATE`](#get_input_rate-0xee) | R | any | custom | `SYS.SYSTEM_STATUS` G, `INPUT.I2S_RX_RATE` G | `0x0140`, `0x0332` | - | see recipe | [see recipe](#get_input_rate-0xee) | see recipe |
| 0xEF | `GET_SPDIF_INPUT_CONFIG` | R | any | same | `INPUT.SPDIF_RX_CONFIG` | `0x0324` | G | `0` | - | 6 |
| 0xF0 | `ENTER_BOOTLOADER` | W-as-R | any | action | `SYS.ENTER_BOOTLOADER` | `0x0130` | S0 | `0` | none | 0 |
| 0xF1 | `SET_I2S_RX_PIN` | W-as-R | any | action | `INPUT.I2S_RX_PIN` | `0x0330` | S | `old >> 8` | u8 `old & 0xFF` | 1 |
| 0xF2 | `GET_I2S_RX_PIN` | R | any | same | `INPUT.I2S_RX_PIN` | `0x0330` | G | `old` | - | 1 |
| 0xF3 | `SET_I2S_INPUT_CHANNELS` | W-as-R | any | action | `INPUT.I2S_RX_CHANNELS` | `0x0331` | S | `0` | u8 `old & 0xFF` | 1 |
| 0xF4 | `GET_I2S_INPUT_CHANNELS` | R | any | same | `INPUT.I2S_RX_CHANNELS` | `0x0331` | G | `0` | - | 1 |
| 0xF5 | `SET_UART_CONFIG` | W | any | folded | `XPORT.UART_CONFIG` | `0x0B10` | S | `0` | unchanged | 8 |
| 0xF6 | `GET_UART_CONFIG` | R | any | folded | `XPORT.UART_CONFIG` | `0x0B10` | G | `0` | - | 8 |
| 0xF7 | `SET_I2C_CONFIG` | W | any | folded | `XPORT.I2C_CONFIG` | `0x0B11` | S | `0` | unchanged | 8 |
| 0xF8 | `GET_I2C_CONFIG` | R | any | folded | `XPORT.I2C_CONFIG` | `0x0B11` | G | `0` | - | 8 |
| 0xF9 | `GET_CTRL_IFACE_STATUS` | R | any | same | `XPORT.STATUS` | `0x0B12` | G | `0` | - | 8 |
| 0xFA | `SET_LOUDNESS_MASK` | W | any | folded | `DYN.LOUDNESS_MASK` | `0x0733` | S | `0` | unchanged | 2 |
| 0xFB | `GET_LOUDNESS_MASK` | R | any | folded | `DYN.LOUDNESS_MASK` | `0x0733` | G | `0` | - | 2 |
| 0xFC | `SET_CROSSFEED_OUTPUTS` | W | any | folded | `FX.CROSSFEED_OUTPUTS` | `0x0815` | S | `0` | unchanged | 1 |
| 0xFD | `GET_CROSSFEED_OUTPUTS` | R | any | folded | `FX.CROSSFEED_OUTPUTS` | `0x0815` | G | `0` | - | 1 |
| 0xFE | `SET_I2S_CLOCK_PIN_MODE` | W-as-R | any | action | `CLOCK.I2S_CLOCK_PIN_MODE` | `0x0C11` | S | `0` | u8 `old & 0xFF` | 1 |
| 0xFF | `GET_I2S_CLOCK_PIN_MODE` | R | any | same | `CLOCK.I2S_CLOCK_PIN_MODE` | `0x0C11` | G | `0` | - | 1 |

<!-- VC:END LEGACY_MAP -->

Ten numbers have no rows because 1.1.6 defined no opcode there. They are `0x00` to `0x02` (`0x01` is the Microsoft OS 2.0 request, unchanged), `0x40`, `0x41`, `0x4F`, `0x6F`, `0x9F`, `0xAF` and `0xCF`.

### 6.3 `GET_STATUS` (0x50)

In 1.1.6, `GET_STATUS` returned one 4-byte value chosen by `wValue`, except selector 9, which returned the combined meter packet. In 1.1.7 the values live in a few structures, each read in one request. Read the structure, then take the field at the offset shown. The last column says how to rebuild the old 4-byte value if your code still expects it. Selectors 0, 1, 2 and 7 have no single new home and are listed as retired in 6.7.

<!-- VC:BEGIN LEGACY_STATUS -->

| Old wValue | 1.1.6 meaning | Read now | Offset | Type | Old 4-byte value |
|---|---|---|---|---|---|
| 0 | Peaks of channels 0 and 1 | `MEAS.LEVELS` | 0, 2 | u16, u16 | bits 0-15 = u16 at 0, bits 16-31 = u16 at 2 |
| 1 | Peaks of channels 2 and 3 | `MEAS.LEVELS` | 4, 6 | u16, u16 | bits 0-15 = u16 at 4, bits 16-31 = u16 at 6 |
| 2 | Peak of channel 4, CPU loads | `MEAS.LEVELS`, `SYS.SYSTEM_STATUS` | 8, then 12 and 13 | u16, then u8, u8 | bits 0-15 = peak, 16-23 = byte 12, 24-31 = byte 13 |
| 3 | PDM ring overruns | `SYS.PDM_STATS` | 0 | u32 | as is |
| 4 | PDM ring underruns | `SYS.PDM_STATS` | 4 | u32 | as is |
| 5 | PDM DMA overruns | `SYS.PDM_STATS` | 8 | u32 | as is |
| 6 | PDM DMA underruns | `SYS.PDM_STATS` | 12 | u32 | as is |
| 7 | Retired, always 0 | none | - | - | always 0 |
| 8 | USB audio packet gaps of 2 to 50 ms (`spdif_underruns`) | `SYS.SPDIF_TX_STATS` | 0 | u32 | as is |
| 9 | Combined meter packet | `MEAS.LEVELS` | whole reply | var | same bytes as the old reply |
| 10 | USB audio packets | `SYS.USB_AUDIO_STATUS` | 0 | u32 | as is |
| 11 | USB audio alt setting | `SYS.USB_AUDIO_STATUS` | 12 | u8 | zero-extend |
| 12 | USB audio mounted | `SYS.USB_AUDIO_STATUS` | 13 | u8 | zero-extend |
| 13 | clk_sys, Hz | `SYS.SYSTEM_STATUS` | 0 | u32 | as is |
| 14 | Core voltage, mV | `SYS.SYSTEM_STATUS` | 8 | u16 | zero-extend |
| 15 | Device sample rate, Hz | `SYS.SYSTEM_STATUS` | 4 | u32 | as is |
| 16 | Temperature, centi-degrees C | `SYS.SYSTEM_STATUS` | 10 | i16 | sign-extend |
| 17 | Output DMA starvations, total | `SYS.SPDIF_TX_STATS` | 4 | u32 | as is |
| 18 | Output DMA starvations, slot 0 | `SYS.SPDIF_TX_STATS` | 8 | u32 | as is |
| 19 | Output DMA starvations, slot 1 | `SYS.SPDIF_TX_STATS` | 12 | u32 | as is |
| 20 | Output DMA starvations, slot 2 | `SYS.SPDIF_TX_STATS` | 16 | u32 | as is |
| 21 | Output DMA starvations, slot 3 | `SYS.SPDIF_TX_STATS` | 20 | u32 | as is |
| 22 | USB audio ring overruns | `SYS.USB_AUDIO_STATUS` | 4 | u32 | as is |
| 23 | Active input channel count | `INPUT.STATUS` | 1 | u8 | zero-extend |
| 24 | Loopback capture overflows | `DIAG.LOOPBACK_STATS` (DSPI_LOOPBACK), read 0 if `VC_ERR_UNKNOWN` | 0 | u32 | as is |
| 25 | Loopback capture underruns | `DIAG.LOOPBACK_STATS` (DSPI_LOOPBACK), read 0 if `VC_ERR_UNKNOWN` | 4 | u32 | as is |
| 26 | Output ring underrun episodes | `SYS.PIPELINE_STATS` | 0 | u32 | as is |
| 27 | Output ring drops | `SYS.PIPELINE_STATS` | 4 | u32 | as is |
| 28 | Lowest ring fill before a write, frames | `SYS.PIPELINE_STATS` | 8 | u32 | as is |
| 29 | Worst input delivery gap, us | `SYS.PIPELINE_STATS` | 12 | u32 | as is |
| 30 | Worst USB packet gap, us | `SYS.USB_AUDIO_STATUS` | 8 | u32 | as is |
| 31 | Background steps over budget | `SYS.PIPELINE_STATS` | 16 | u32 | as is |
| 32 | S/PDIF input measured rate, mHz | `INPUT.STATUS` | 4 | u32 | as is |
| 33 | Clock trim, ppm x 100 | `INPUT.STATUS` | 8 | i32 | as is |
| any other | Unused (4 zero bytes) | none | - | - | no equivalent |

<!-- VC:END LEGACY_STATUS -->

Read each structure whole, or at least up to the end of the field you want. A port that keeps the old 4-byte `wLength` gets a prefix, which is the field at offset 0, not the one it wanted. For example selector 15 was the sample rate, but a 4-byte read of `SYS.SYSTEM_STATUS` returns `clk_sys`. The sample rate is at offset 4.

The structures, in full.

`SYS.SYSTEM_STATUS` (`0x0140`, 16 bytes).

| Offset | Type | Field |
|---|---|---|
| 0 | u32 | `clk_sys` frequency, Hz |
| 4 | u32 | Device sample rate, Hz |
| 8 | u16 | Core voltage, mV |
| 10 | i16 | Chip temperature, hundredths of a degree C |
| 12 | u8 | Core 0 load, percent |
| 13 | u8 | Core 1 load, percent |
| 14 | u16 | Reserved, 0 |

`SYS.USB_AUDIO_STATUS` (`0x0141`, 16 bytes).

| Offset | Type | Field |
|---|---|---|
| 0 | u32 | USB audio packets received |
| 4 | u32 | USB audio ring overruns |
| 8 | u32 | Worst gap between USB audio packets, us. Reset by `SYS.STATS_RESET` |
| 12 | u8 | Current USB audio alt setting |
| 13 | u8 | 1 while the USB audio interface is mounted |
| 14 | u16 | Reserved, 0 |

`SYS.PIPELINE_STATS` (`0x0143`, 24 bytes).

| Offset | Type | Field |
|---|---|---|
| 0 | u32 | Output ring underrun episodes |
| 4 | u32 | Blocks refused by ring admission (drops) |
| 8 | u32 | Lowest ring fill seen before a write, frames. `0xFFFFFFFF` until the first sample. Reset by `SYS.STATS_RESET` |
| 12 | u32 | Worst gap between input deliveries, us. Reset by `SYS.STATS_RESET` |
| 16 | u32 | Background steps that ran over their time budget |
| 20 | u32 | Reserved, 0 |

`SYS.PDM_STATS` (`0x0144`, 16 bytes) is four u32 counters. They are PDM ring overruns at offset 0, PDM ring underruns at 4, PDM DMA overruns at 8 and PDM DMA underruns at 12.

`SYS.SPDIF_TX_STATS` (`0x0145`, 24 bytes) is six u32 counters. Offset 0 is `spdif_underruns`. Despite its name it counts USB audio packet gaps longer than 2 ms and shorter than 50 ms (none are counted during a preset load), the gaps that would starve the outputs. It does not measure the S/PDIF transmitters, although 1.1.6 described it as S/PDIF underruns. Offset 4 is the total of output DMA starvations, and offsets 8, 12, 16 and 20 are the starvations of output slots 0, 1, 2 and 3. Slots 2 and 3 read 0 on RP2040.

`INPUT.STATUS` (`0x0311`, 12 bytes).

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Active input source ID (0 to 6) |
| 1 | u8 | Active input channel count |
| 2 | u16 | Reserved, 0 |
| 4 | u32 | Measured S/PDIF input rate, millihertz. 0 when nothing is measured |
| 8 | i32 | Clock trim the device applies to follow the input, ppm x 100 |

`MEAS.LEVELS` (`0x0910`) is byte for byte the 1.1.6 combined packet of selector 9. Its length is `2 x NUM_CHANNELS + 7`, which is 21 bytes on RP2040 and 41 on RP2350. N is `NUM_CHANNELS` (7 or 17).

| Offset | Type | Field |
|---|---|---|
| 0 | u16[N] | Peak level per channel, inputs then outputs. Linear, 0 to 32767 is 0 to full scale |
| 2N | u8 | Core 0 load, percent |
| 2N + 1 | u8 | Core 1 load, percent |
| 2N + 2 | u32 | Latched clip flags, bit k = channel k |
| 2N + 6 | u8 | Active input channel count |

`DIAG.LOOPBACK_STATS` (`0xF020`, 8 bytes, `DSPI_LOOPBACK` builds only) is two u32 counts, loopback capture overflow frames then underrun frames. A release build answers `VC_ERR_UNKNOWN`. 1.1.6 answered selectors 24 and 25 with 0 there.

A stats screen that made about 25 scalar reads in 1.1.6 now makes six struct reads. `SYS.STATS_RESET` with bit 0 set resets the watermark fields of `SYS.BUFFER_STATS`, `SYS.PIPELINE_STATS` (offsets 8 and 12) and `SYS.USB_AUDIO_STATUS` (offset 8), the same fields 1.1.6 reset.

### 6.4 Other split commands

Each of these 1.1.6 opcodes picked an operation or a parameter with `wValue`. In 1.1.7 each operation is its own command.

#### `LIMITER` (0x81)

1.1.6 used one opcode for every limiter request, with `wValue = (output << 8) | index`. Indexes 0 to 3 were the four parameters, both directions. Index `0x80` read the meters and `0x81` the status. In 1.1.7 each parameter is its own command, addressed by output alone. The values are still float32, exactly as in 1.1.6.

| Old index (`wValue` low byte) | New command | ID | New `wValue` | Bytes |
|---|---|---|---|---|
| 0 | `DYN.LIMITER_ENABLE` | `0x0710` | Output 0 to 4 (RP2040) or 0 to 8 (RP2350). `0xFF` writes every output, on a SET only | 4, f32, non-zero enables |
| 1 | `DYN.LIMITER_THRESHOLD` | `0x0711` | Same | 4, f32 dB, clamped to -30..0 |
| 2 | `DYN.LIMITER_RELEASE` | `0x0712` | Same | 4, f32 ms, clamped to 10..1000 |
| 3 | `DYN.LIMITER_LINK_GROUP` | `0x0713` | Same | 4, f32, rounded and clamped to 0..4 |
| `0x80` (GET) | `DYN.LIMITER_METERS` | `0x0718` | 0 | 2 x outputs, one u16 gain reduction per output in 0.01 dB |
| `0x81` (GET) | `DYN.LIMITER_STATUS` | `0x0719` | 0 | 4. u8 engaged, u8 lookahead samples (32), u8 block samples (16), u8 output count |

A bad output is now `VC_ERR_ADDRESS` (1.1.6 ignored it on a SET). `0xFF` on a GET is `VC_ERR_ADDRESS`, as it STALLed in 1.1.6. The meters and status commands require `wValue` 0, where 1.1.6 ignored the output byte.

#### `SET_TUBE_PARAM` and `GET_TUBE_PARAM` (0x3E, 0x3F)

`wValue` was the parameter index in `TUBE_PARAM_*` order. Each index is now a command with `wValue` 0 and the same 4-byte float32 payload or reply.

| Old index | New command | ID |
|---|---|---|
| 0 | `FX.TUBE_ENABLE` | `0x0860` |
| 1 | `FX.TUBE_OUTPUT_MASK` | `0x0861` |
| 2 | `FX.TUBE_TYPE` | `0x0862` |
| 3 | `FX.TUBE_DRIVE` | `0x0863` |
| 4 | `FX.TUBE_BIAS` | `0x0864` |
| 5 | `FX.TUBE_ASYM` | `0x0865` |
| 6 | `FX.TUBE_HARDNESS` | `0x0866` |
| 7 | `FX.TUBE_SAG` | `0x0867` |
| 8 | `FX.TUBE_RECTIFIER` | `0x0868` |
| 9 | `FX.TUBE_XFMR_ENABLE` | `0x0869` |
| 10 | `FX.TUBE_XFMR_DAMPING` | `0x086A` |
| 11 | `FX.TUBE_XFMR_RES` | `0x086B` |
| 12 | `FX.TUBE_MIX` | `0x086C` |
| 13 | `FX.TUBE_TRIM` | `0x086D` |

There is no index any more, so there is no bad index. Setting `FX.TUBE_TYPE` to a stock tube still rewrites bias, asymmetry, hardness and sag, and writing one of those four still sets the type to 0 (custom). Re-read all five after either kind of write.

#### `UPMIX_SET_PARAM` and `UPMIX_GET_PARAM` (0x4C, 0x4D)

`wValue` was the parameter index in `UPMIX_PARAM_*` order. Each index is now a command with `wValue` 0 and the same 4-byte float32 payload or reply. These commands exist on RP2350 only. On RP2040 they answer `VC_ERR_UNKNOWN`, where 1.1.6 answered the GETs with zeros and STALLed the SETs.

| Old index | New command | ID |
|---|---|---|
| 0 | `FX.UPMIX_ENABLE` | `0x0890` |
| 1 | `FX.UPMIX_CENTER_MODE` | `0x0891` |
| 2 | `FX.UPMIX_SURROUND_MODE` | `0x0892` |
| 3 | `FX.UPMIX_STRENGTH` | `0x0893` |
| 4 | `FX.UPMIX_CENTER_WIDTH` | `0x0894` |
| 5 | `FX.UPMIX_THRESHOLD` | `0x0895` |
| 6 | `FX.UPMIX_ATTACK` | `0x0896` |
| 7 | `FX.UPMIX_RELEASE` | `0x0897` |
| 8 | `FX.UPMIX_DET_HPF` | `0x0898` |
| 9 | `FX.UPMIX_SUR_DELAY` | `0x0899` |
| 10 | `FX.UPMIX_SUR_HPF` | `0x089A` |
| 11 | `FX.UPMIX_SUR_LPF` | `0x089B` |
| 12 | `FX.UPMIX_DECORR` | `0x089C` |
| 13 | `FX.UPMIX_PRESENCE` | `0x089D` |

`UPMIX_SET_CONFIG` and `UPMIX_GET_CONFIG` (`0x4A`, `0x4B`) fold into `FX.UPMIX_CONFIG` (`0x0880`, 44 bytes, unchanged layout), and `UPMIX_GET_STATUS` (`0x4E`) is `FX.UPMIX_STATUS` (`0x0881`, 16 bytes).

#### `RTA_GET_CAPS` (0x0A)

| Old `wValue` | New command | New `wValue` | Reply |
|---|---|---|---|
| 0 | `MEAS.RTA_CAPS` (`0x0921`) | 0 | 16 bytes, unchanged |
| n, from 1 | `MEAS.RTA_BAND_CENTRES` (`0x0922`) | n - 1 | Up to 32 u16 band centres in Hz, starting at band `32 x (n - 1)`. With 37 bands, chunk 0 is 64 bytes and chunk 1 is 10 bytes |

A chunk past the table is `VC_ERR_ADDRESS`.

#### `GET_CS_CAPS` (0x86)

| Old `wValue` | New command | New `wValue` | Reply |
|---|---|---|---|
| `0xFFFF` | `CS.CAPS` (`0x0A10`) | 0 | 52 bytes at capability version 22, unchanged |
| A noun number | `CS.NOUN_DESC` (`0x0A11`) | The noun | 12 bytes, unchanged |

#### `SIGGEN_GET_CAPS` (0xA8)

| Old `wValue` | New command | New `wValue` | Reply |
|---|---|---|---|
| `0xFFFF` | `MEAS.SIGGEN_CAPS` (`0x0942`) | 0 | 8 bytes, unchanged |
| A signal type | `MEAS.SIGGEN_TYPE_DESC` (`0x0943`) | The type | 62 bytes, unchanged |

A type past the table is `VC_ERR_ADDRESS`.

#### `GET_CS_AUX_STATE` (0x05)

| Old `wValue` | New command | New `wValue` | Reply |
|---|---|---|---|
| `0xFFFF` | `CS.AUX_ALL` (`0x0A62`) | 0 | 48 bytes, unchanged. 16 state bytes, then 16 u16 levels |
| A slot | `CS.AUX_STATE` (`0x0A60`) | The slot | 1 byte, unchanged |

A slot that is not a live aux output is `VC_ERR_STATE`. 1.1.6 STALLed it too, with no reason.

#### `OUTPUT_LATENCY` (0x03)

| 1.1.6 request | New command | Bytes |
|---|---|---|
| OUT, `wValue` 0, 1-byte option ID | `OUTPUT.LATENCY` SET (`0x0420`) | 1 |
| IN, `wValue` 0 | `OUTPUT.LATENCY_STATUS` (`0x0421`) | 12, unchanged |
| IN, `wValue` 1 | `OUTPUT.LATENCY_OPTIONS` (`0x0422`) | 8 per option, 56 today, unchanged |
| (none) | `OUTPUT.LATENCY` GET (`0x0420`) | 1, new. The queued option ID while a change is pending, otherwise the stored one |

An unknown option ID is now `VC_ERR_VALUE`. 1.1.6 ignored it on USB.

#### `RTA_CONTROL` (0x0E) and `SIGGEN_CONTROL` (0xA6)

Both were write-as-reads with the action in the `wValue` low byte and an answer of `01`. Each action is now a zero-length SET with `wValue` 0.

| Old action | `RTA_CONTROL` becomes | `SIGGEN_CONTROL` becomes |
|---|---|---|
| 0 | `MEAS.RTA_STOP` (`0x0929`) | `MEAS.SIGGEN_STOP` (`0x0949`), fades out over 5 ms |
| 1 | `MEAS.RTA_START` (`0x0928`) | `MEAS.SIGGEN_START` (`0x0948`) |
| 2 | `MEAS.RTA_RESET` (`0x092A`) | `MEAS.SIGGEN_STOP_NOW` (`0x094A`) |

`MEAS.SIGGEN_START` before any valid configuration is `VC_ERR_STATE`.

#### `CS_IR_LEARN` (0x8F)

| Old `wValue` | New command | Kind |
|---|---|---|
| 0 | `CS.IR_LEARN_CANCEL` (`0x0A33`) | Zero-length SET |
| 1 | `CS.IR_LEARN_ARM` (`0x0A32`) | Zero-length SET |
| 2 | `CS.IR_LEARN_STATUS` (`0x0A31`) | GET, 8 bytes, unchanged. u8 state, u8 protocol, 2 zero bytes, u32 code |

Arm with no live IR receiver is `VC_ERR_STATE` and still posts NO_IR (`0x1E`) in `CS.STATUS`. Arm or cancel during a revert is `VC_ERR_BUSY`.

#### `CS_MACRO_FIRE` (0x25)

| Old `wValue` | New command | New `wValue` |
|---|---|---|
| A macro, 0 to 7 | `CS.MACRO_FIRE` (`0x0A58`), zero-length SET | The macro |
| `0xFFFF` | `CS.MACRO_CANCEL` (`0x0A59`), zero-length SET | 0 |

A bad macro is `VC_ERR_ADDRESS`. Fire during a revert is `VC_ERR_BUSY`. Cancel always succeeds.

### 6.5 Reshaped and combined commands

Only two payloads changed shape on purpose, the EQ band and the matrix crosspoint. The other entries here are 1.1.6 replies that combined several values, now read from more than one command.

#### `SET_EQ_PARAM` and `GET_EQ_PARAM` (0x42, 0x43)

In 1.1.6 a band was written with a 16- or 18-byte packet that carried the channel and band inside it, and read one parameter at a time with a 3-bit selector. In 1.1.7 `FILTER.BAND` (`0x0610`) reads and writes the whole band, addressed by `wValue = (channel << 8) | band`, with one 16-byte layout in both directions.

Old SET payload (`EqParamPacket`, 16 bytes, or 18 with the Linkwitz Qp).

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Channel |
| 1 | u8 | Band |
| 2 | u8 | Filter type |
| 3 | u8 | Bypass, exactly 1 bypasses |
| 4 | f32 | Frequency, Hz |
| 8 | f32 | Q |
| 12 | f32 | Gain, dB |
| 16 | u16 | Optional. Linkwitz Transform Qp x 512. A 16-byte write kept the stored Qp |

New `FILTER.BAND` payload and reply (16 bytes, `wValue = (channel << 8) | band`).

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Filter type (`Documentation/commands.md` section 4.7) |
| 1 | u8 | Bypass: exactly 1 bypasses. Reads back as 0 or 1 |
| 2 | u16 | Linkwitz Transform Qp x 512, 0 selects 0.707. Ignored and read as 0 for a crossover band |
| 4 | f32 | Frequency, Hz |
| 8 | f32 | Q |
| 12 | f32 | Gain, dB (for the Linkwitz Transform, fp in Hz) |

To convert an old packet `p` into a new request, send `wValue = (p[0] << 8) | p[1]` and the payload `p[2], p[3], qp low, qp high, p[4..15]`. Bands 0 to 9 are EQ bands and 20 to 23 are crossover bands. A band in the gap 10 to 19, or a crossover band on an input channel, is `VC_ERR_ADDRESS` in both directions (1.1.6 ignored the write). A non-finite frequency, Q or gain is `VC_ERR_VALUE`.

The old 4-byte reads map onto the new reply like this. Old `wValue` was `(channel << 8) | (band << 3) | param`.

| Old param | Old 4-byte reply | Where it is in the `FILTER.BAND` reply |
|---|---|---|
| 0 | Type, as u32 | Byte 0, zero-extended |
| 1 | Frequency, f32 | Bytes 4 to 7 |
| 2 | Q, f32 | Bytes 8 to 11 |
| 3 | Gain, f32 | Bytes 12 to 15 |
| 4 | Bypass (1 or 0), as u32 | Byte 1, zero-extended |
| 5 | Qp x 512, as u32 | Bytes 2 and 3 (u16), zero-extended |

**Writing one parameter.** There is no single-parameter write. To change one field, read the band, change the field and write all 16 bytes back. Three rules keep this safe.

1. **Always send the Qp.** 1.1.6 kept the stored Qp when a host sent 16 bytes. 1.1.7 always stores what you send, so copy bytes 2 and 3 from your read.
2. **Mind the busy rule.** There is one pending band write for the whole device. A second `FILTER.BAND` or `FILTER.BAND_BYPASS` before the main loop has taken the first is `VC_ERR_BUSY`. Wait 1 to 2 ms and resend.
3. **Read after the write is applied.** A GET returns the stored band, and a queued write appears only once applied, a millisecond or two later. If you read, modify and write two fields in quick succession, the second read can still show the old band and undo the first change. Keep a host-side copy of every band (from `STORE.ALL_PARAMS` and `PARAM_CHANGED` notifications) and write from that copy, or read back until the band matches what you wrote.

`FILTER.BAND_BYPASS` (`0x0611`, 1 byte) still reads or sets the bypass byte alone with the same addressing and the same pending slot.

#### `SET_MATRIX_ROUTE` and `GET_MATRIX_ROUTE` (0x70, 0x71)

The crosspoint address moved from the payload into `wValue`, and the GET no longer echoes it. `ROUTE.MATRIX_ROUTE` (`0x0510`) uses `wValue = (input << 8) | output` in both directions.

| Offset | Old `MatrixRoutePacket` (SET payload and GET reply, 8 bytes) | New `ROUTE.MATRIX_ROUTE` (SET payload and GET reply, 8 bytes) |
|---|---|---|
| 0 | u8 input | u8 enabled, non-zero routes |
| 1 | u8 output | u8 phase invert, non-zero inverts |
| 2 | u8 enabled | u16 reserved, send 0 |
| 3 | u8 phase invert | (second byte of the reserved u16) |
| 4 | f32 gain, dB | f32 gain, dB. A non-finite value is `VC_ERR_VALUE` |

To convert an old SET packet `p` (sent with `wValue` 0), send `wValue = (p[0] << 8) | p[1]` and the payload `p[2], p[3], 00, 00, p[4..7]`. To rebuild the old GET reply from a new reply `r` read at `(input << 8) | output`, use `input, output, r[0], r[1], r[4..7]`. Input ranges over 0 to 1 (RP2040) or 0 to 7 (RP2350) and output over 0 to 4 or 0 to 8. Anything else is `VC_ERR_ADDRESS`.

#### `CLEAR_CLIPS` (0x83)

1.1.6 read the latched clip flags and cleared them in one write-as-read, answering the 4-byte flags. 1.1.7 splits that into a read and an action.

1. `MEAS.CLIP_FLAGS` (`0x0911`), GET, `wValue` 0, 4 bytes, u32 with bit k = channel k. Reading does not clear.
2. `MEAS.CLIP_CLEAR` (`0x0912`), zero-length SET, `wValue` 0.

A clip that lands between the two requests is cleared without being seen. The flags are also bytes `2N + 2` to `2N + 5` of `MEAS.LEVELS`, so a meter loop that already reads `MEAS.LEVELS` needs only the clear.

#### `GET_INPUT_RATE` (0xEE)

1.1.6 answered 8 bytes. In 1.1.7 each half has its own home.

| Old bytes | Old field | New source |
|---|---|---|
| 0 to 3 | u32 current device rate, Hz | `SYS.SYSTEM_STATUS` (`0x0140`) bytes 4 to 7 |
| 4 to 7 | u32 selected I2S rate, Hz | `INPUT.I2S_RX_RATE` (`0x0332`) GET, 4 bytes |

`SET_INPUT_RATE` (`0xED`) folds into the SET of `INPUT.I2S_RX_RATE` with the same 4-byte payload. A rate other than 44100, 48000 or 96000 is now `VC_ERR_VALUE` (1.1.6 ignored it).

#### `PRESET_GET_STARTUP` (0x97)

1.1.6 answered 3 bytes, `{mode, default slot, last active slot}`. `STORE.STARTUP` (`0x0216`) answers 2 bytes, `{mode, default slot}`, which is also exactly what its SET takes. The last active slot is byte 4 of `STORE.DIRECTORY` (`0x0215`, 7 bytes, unchanged).

#### `GET_CS_DISPLAY_CFG` (0x28)

1.1.6 answered 16 bytes, 4 limit bytes followed by the 12-byte display configuration. In 1.1.7 the two parts are separate commands.

| Old bytes | New source |
|---|---|
| 0 to 3 | `CS.DISPLAY_CAPS` (`0x0A71`, 4 bytes). It holds the page count (16), the model count (9) and 2 reserved bytes |
| 4 to 15 | `CS.DISPLAY_CONFIG` (`0x0A70`, 12 bytes), the same bytes the SET takes (7.3) |

#### `SET_ALL_PARAMS` (0xA1)

The payload is unchanged, but the length rule is now strict. Send exactly 6136 bytes to `STORE.ALL_PARAMS` (`0x0240`) with `wValue` 0. 1.1.6 ACKed and ignored a payload of 64 bytes or fewer and STALLed other wrong lengths at SETUP. 1.1.7 STALLs every length other than 6136 at SETUP with `VC_ERR_LENGTH`, and a header the apply would refuse STALLs the status stage with `VC_ERR_VALUE` instead of being dropped after the ACK. Section 9 has the details.

### 6.6 Former write-as-reads

1.1.6 had 34 opcodes that changed state through an IN transfer. They carried their parameters in `wValue` and answered a status byte. In 1.1.7 each is a SET. The address stays in `wValue`, the value moves into a 1-byte payload, and an action with no value is a zero-length SET. The status byte is gone. Success is an ACK and a refusal is a STALL whose reason is in `SYS.LAST_RESULT`, or the status byte on UART and I2C.

In the last column, `0x01 for VALUE or PIN_INVALID` means "a host that used to receive `0x01` now receives `VC_ERR_VALUE` or `VC_ERR_PIN_INVALID`". Result names are shown without their `VC_ERR_` prefix.

<!-- VC:BEGIN LEGACY_WASR -->

| Old | Old name | Old wValue | Old reply | New command | New wValue | SET payload | Old status byte: new result |
|---|---|---|---|---|---|---|---|
| 0x0E | [`RTA_CONTROL`](#rta_control-0x0e-and-siggen_control-0xa6) | old & 0xFF = 0 | 1 byte `0x01` | `MEAS.RTA_STOP` `0x0929` | `0` | none (zero-length) | 0x01 was success. Every refusal STALLed then and STALLs now |
| 0x0E | [`RTA_CONTROL`](#rta_control-0x0e-and-siggen_control-0xa6) | old & 0xFF = 1 | 1 byte `0x01` | `MEAS.RTA_START` `0x0928` | `0` | none (zero-length) | 0x01 was success. Every refusal STALLed then and STALLs now |
| 0x0E | [`RTA_CONTROL`](#rta_control-0x0e-and-siggen_control-0xa6) | old & 0xFF = 2 | 1 byte `0x01` | `MEAS.RTA_RESET` `0x092A` | `0` | none (zero-length) | 0x01 was success. Every refusal STALLed then and STALLs now |
| 0x0E | [`RTA_CONTROL`](#rta_control-0x0e-and-siggen_control-0xa6) | any other | - | none | - | - | 1.1.6 STALLed it |
| 0x25 | [`CS_MACRO_FIRE`](#cs_macro_fire-0x25) | 0xFFFF | 1 byte `0x01` | `CS.MACRO_CANCEL` `0x0A59` | `0` | none (zero-length) | 0x01 was success. Every refusal STALLed then and STALLs now |
| 0x25 | [`CS_MACRO_FIRE`](#cs_macro_fire-0x25) | any other | 1 byte `0x01` | `CS.MACRO_FIRE` `0x0A58` | `old` | none (zero-length) | 0x01 was success. Every refusal STALLed then and STALLs now |
| 0x51 | `SAVE_PARAMS` | any | see recipe | `STORE.PRESET_ACTIVE`, `STORE.PRESET_SAVE` | see recipe | [see recipe](#67-retired-opcodes) | see recipe |
| 0x52 | `SAVE_OUTPUT_CONFIG` | 0 | 1 byte `PRESET_*` | `STORE.OUTPUT_CONFIG_SAVE` `0x0221` | `0` | none (zero-length) | 0x00 for OK. 0x01 for `ADDRESS`. 0x02 for `STATE` |
| 0x53 | `FACTORY_RESET` | 0 | 1 byte `FLASH_OK` | `STORE.FACTORY_RESET` `0x0230` | `0` | none (zero-length) | 0x00 was success. 1.1.6 never refused it |
| 0x68 | `SET_ADAT_INPUT_ENABLE` | value in `old & 0xFF` | 1 byte `PIN_CONFIG_*` | `INPUT.ADAT_RX_ENABLE` (RP2350 only) `0x0340` | `0` | 1 byte: u8 `old & 0xFF` | 0x00 for OK. 0x01 for `PIN_INVALID`. 0x02 for `STATE` or `PIN_IN_USE`. 0x03 for `UNKNOWN`, `ADDRESS` or `UNSUPPORTED`. 0x05 for `VALUE` |
| 0x6A | `SET_ADAT_INPUT_PIN` | value in `old & 0xFF` | 1 byte `PIN_CONFIG_*` | `INPUT.ADAT_RX_PIN` (RP2350 only) `0x0341` | `0` | 1 byte: u8 `old & 0xFF` | 0x00 for OK. 0x01 for `PIN_INVALID`. 0x02 for `STATE` or `PIN_IN_USE`. 0x03 for `UNKNOWN`, `ADDRESS` or `UNSUPPORTED`. 0x05 for `VALUE` |
| 0x6C | `SET_ADAT_INPUT_CLOCK_MODE` | value in `old & 0xFF` | 1 byte `PIN_CONFIG_*` | `INPUT.ADAT_RX_CLOCK_MODE` (RP2350 only) `0x0342` | `0` | 1 byte: u8 `old & 0xFF` | 0x00 for OK. 0x01 for `PIN_INVALID`. 0x02 for `PIN_IN_USE`. 0x03 for `UNKNOWN`, `ADDRESS` or `UNSUPPORTED`. 0x04 for `STATE`. 0x05 for `VALUE` |
| 0x7C | `SET_OUTPUT_PIN` | pin output in `old & 0xFF`, value in `old >> 8` | 1 byte `PIN_CONFIG_*` | `OUTPUT.PIN` `0x0411` | `old & 0xFF` | 1 byte: u8 `old >> 8` | 0x00 for OK. 0x01 for `PIN_INVALID`. 0x02 for `PIN_IN_USE`. 0x03 for `UNKNOWN`, `ADDRESS` or `UNSUPPORTED`. 0x04 for `STATE`. 0x05 for `VALUE` |
| 0x83 | [`CLEAR_CLIPS`](#clear_clips-0x83) | any | see recipe | `MEAS.CLIP_FLAGS`, `MEAS.CLIP_CLEAR` | see recipe | [see recipe](#clear_clips-0x83) | see recipe |
| 0x8F | [`CS_IR_LEARN`](#cs_ir_learn-0x8f) | 0 | 1 byte `0x01` | `CS.IR_LEARN_CANCEL` `0x0A33` | `0` | none (zero-length) | 0x01 was success. Every refusal STALLed then and STALLs now |
| 0x8F | [`CS_IR_LEARN`](#cs_ir_learn-0x8f) | 1 | 1 byte `0x01` | `CS.IR_LEARN_ARM` `0x0A32` | `0` | none (zero-length) | 0x01 was success. Every refusal STALLed then and STALLs now |
| 0x8F | [`CS_IR_LEARN`](#cs_ir_learn-0x8f) | 2 | 8 bytes | `CS.IR_LEARN_STATUS` `0x0A31` | `0` | none, it is a GET | reply unchanged |
| 0x8F | [`CS_IR_LEARN`](#cs_ir_learn-0x8f) | any other | - | none | - | - | 1.1.6 STALLed it |
| 0x90 | `PRESET_SAVE` | preset slot in `old & 0xFF` | 1 byte `PRESET_*` | `STORE.PRESET_SAVE` `0x0210` | `old & 0xFF` | none (zero-length) | 0x00 for OK. 0x01 for `ADDRESS`. 0x02 for `STATE` |
| 0x91 | `PRESET_LOAD` | preset slot in `old & 0xFF` | 1 byte `PRESET_*` | `STORE.PRESET_LOAD` `0x0211` | `old & 0xFF` | none (zero-length) | 0x00 for OK. 0x01 for `ADDRESS`. 0x02 for `STATE` |
| 0x92 | `PRESET_DELETE` | preset slot in `old & 0xFF` | 1 byte `PRESET_*` | `STORE.PRESET_DELETE` `0x0212` | `old & 0xFF` | none (zero-length) | 0x00 for OK. 0x01 for `ADDRESS`. 0x02 for `STATE` |
| 0x9D | `CS_SAVE` | 0 | 1 byte `0x01` | `CS.SAVE` `0x0A18` | `0` | none (zero-length) | 0x01 was success. Every refusal STALLed then and STALLs now |
| 0x9E | `CS_REVERT` | 0 | 1 byte `0x01` | `CS.REVERT` `0x0A19` | `0` | none (zero-length) | 0x01 was success. Every refusal STALLed then and STALLs now |
| 0xA6 | [`SIGGEN_CONTROL`](#rta_control-0x0e-and-siggen_control-0xa6) | old & 0xFF = 0 | 1 byte `0x01` | `MEAS.SIGGEN_STOP` `0x0949` | `0` | none (zero-length) | 0x01 was success. Every refusal STALLed then and STALLs now |
| 0xA6 | [`SIGGEN_CONTROL`](#rta_control-0x0e-and-siggen_control-0xa6) | old & 0xFF = 1 | 1 byte `0x01` | `MEAS.SIGGEN_START` `0x0948` | `0` | none (zero-length) | 0x01 was success. Every refusal STALLed then and STALLs now |
| 0xA6 | [`SIGGEN_CONTROL`](#rta_control-0x0e-and-siggen_control-0xa6) | old & 0xFF = 2 | 1 byte `0x01` | `MEAS.SIGGEN_STOP_NOW` `0x094A` | `0` | none (zero-length) | 0x01 was success. Every refusal STALLed then and STALLs now |
| 0xA6 | [`SIGGEN_CONTROL`](#rta_control-0x0e-and-siggen_control-0xa6) | any other | - | none | - | - | 1.1.6 STALLed it |
| 0xB1 | `RESET_BUFFER_STATS` | value in `old & 0xFF` | 1 byte `0x01` | `SYS.STATS_RESET` `0x014F` | `0` | 1 byte: u8 `old & 0xFF` | 0x01 was success. Every refusal STALLed then and STALLs now |
| 0xB3 | `RESET_USB_ERROR_STATS` | any | see recipe | none | see recipe | [see recipe](#67-retired-opcodes) | see recipe |
| 0xC0 | `SET_OUTPUT_TYPE` | output slot in `old & 0xFF`, value in `old >> 8` | 1 byte `PIN_CONFIG_*` | `OUTPUT.TYPE` `0x0410` | `old & 0xFF` | 1 byte: u8 `old >> 8` | 0x00 for OK. 0x01 for `VALUE` or `PIN_INVALID`. 0x02 for `PIN_IN_USE`. 0x03 for `UNKNOWN`, `ADDRESS` or `UNSUPPORTED`. 0x04 for `STATE` |
| 0xC2 | `SET_I2S_BCK_PIN` | role 0-1 in `old >> 8`, value in `old & 0xFF` | 1 byte `PIN_CONFIG_*` | `CLOCK.I2S_BCK_PIN` `0x0C10` | `old >> 8` | 1 byte: u8 `old & 0xFF` | 0x00 for OK. 0x01 for `PIN_INVALID`. 0x02 for `PIN_IN_USE`. 0x03 for `UNKNOWN`, `ADDRESS` or `UNSUPPORTED`. 0x04 for `STATE`. 0x05 for `VALUE` |
| 0xC4 | `SET_MCK_ENABLE` | value in `old` | 1 byte `PIN_CONFIG_*` | `CLOCK.MCK_ENABLE` `0x0C20` | `0` | 1 byte: u8 `old != 0` (1 or 0) | 0x00 for OK. 0x01 for `PIN_INVALID`. 0x02 for `PIN_IN_USE`. 0x03 for `UNKNOWN`, `ADDRESS` or `UNSUPPORTED`. 0x04 for `STATE`. 0x05 for `VALUE` |
| 0xC6 | `SET_MCK_PIN` | value in `old & 0xFF` | 1 byte `PIN_CONFIG_*` | `CLOCK.MCK_PIN` `0x0C21` | `0` | 1 byte: u8 `old & 0xFF` | 0x00 for OK. 0x01 for `PIN_INVALID`. 0x02 for `PIN_IN_USE`. 0x03 for `UNKNOWN`, `ADDRESS` or `UNSUPPORTED`. 0x04 for `STATE`. 0x05 for `VALUE` |
| 0xC8 | `SET_MCK_MULTIPLIER` | value in `old` | 1 byte `PIN_CONFIG_*` | `CLOCK.MCK_MULTIPLIER` `0x0C22` | `0` | 1 byte: u8 `min(old, 255)` | 0x00 for OK. 0x01 for `VALUE` or `PIN_INVALID`. 0x02 for `PIN_IN_USE`. 0x03 for `UNKNOWN`, `ADDRESS` or `UNSUPPORTED`. 0x04 for `STATE` |
| 0xCA | `SET_ADAT_ENABLE` | value in `old & 0xFF` | 1 byte `PIN_CONFIG_*` | `OUTPUT.ADAT_TX_ENABLE` (RP2350 only) `0x0430` | `0` | 1 byte: u8 `old & 0xFF` | 0x00 for OK. 0x01 for `PIN_INVALID`. 0x02 for `PIN_IN_USE`. 0x03 for `UNKNOWN`, `ADDRESS` or `UNSUPPORTED`. 0x04 for `STATE`. 0x05 for `VALUE` |
| 0xCC | `SET_ADAT_PIN` | value in `old & 0xFF` | 1 byte `PIN_CONFIG_*` | `OUTPUT.ADAT_TX_PIN` (RP2350 only) `0x0431` | `0` | 1 byte: u8 `old & 0xFF` | 0x00 for OK. 0x01 for `PIN_INVALID`. 0x02 for `PIN_IN_USE`. 0x03 for `UNKNOWN`, `ADDRESS` or `UNSUPPORTED`. 0x04 for `STATE`. 0x05 for `VALUE` |
| 0xD6 | `SAVE_MASTER_VOLUME` | 0 | 1 byte `PRESET_*` | `ROUTE.MASTER_VOLUME_SAVE` `0x0552` | `0` | none (zero-length) | 0x00 for OK. 0x01 for `ADDRESS`. 0x02 for `STATE` |
| 0xE4 | `SET_SPDIF_RX_PIN` | S/PDIF input in `old >> 8`, value in `old & 0xFF` | 1 byte `PIN_CONFIG_*` | `INPUT.SPDIF_RX_PIN` `0x0322` | `old >> 8` | 1 byte: u8 `old & 0xFF` | 0x00 for OK. 0x01 for `PIN_INVALID`. 0x02 for `PIN_IN_USE`. 0x03 for `UNKNOWN`, `ADDRESS` or `UNSUPPORTED`. 0x04 for `STATE`. 0x05 for `VALUE` |
| 0xE9 | `SET_SPDIF_INPUT_ENABLE` | S/PDIF input in `old >> 8`, value in `old & 0xFF` | 1 byte `PIN_CONFIG_*` | `INPUT.SPDIF_RX_ENABLE` `0x0323` | `old >> 8` | 1 byte: u8 `old & 0xFF != 0` (1 or 0) | 0x00 for OK. 0x02 for `STATE`, `PIN_INVALID` or `PIN_IN_USE`. 0x03 for `UNKNOWN`, `ADDRESS` or `UNSUPPORTED`. 0x05 for `VALUE` |
| 0xEC | `TEST_DAC_HW_MUTE` | 0 | 1 byte `PIN_CONFIG_*` | `OUTPUT.DAC_MUTE_TEST` `0x0441` | `0` | none (zero-length) | 0x00 for OK. 0x01 for `PIN_INVALID`. 0x02 for `PIN_IN_USE`. 0x03 for `UNKNOWN`, `ADDRESS`, `UNSUPPORTED` or `STATE`. 0x05 for `VALUE` |
| 0xF0 | `ENTER_BOOTLOADER` | 0 | 1 byte `0x01` | `SYS.ENTER_BOOTLOADER` `0x0130` | `0` | none (zero-length) | 0x01 was success. Every refusal STALLed then and STALLs now |
| 0xF1 | `SET_I2S_RX_PIN` | I2S pair in `old >> 8`, value in `old & 0xFF` | 1 byte `PIN_CONFIG_*` | `INPUT.I2S_RX_PIN` `0x0330` | `old >> 8` | 1 byte: u8 `old & 0xFF` | 0x00 for OK. 0x01 for `PIN_INVALID`. 0x02 for `PIN_IN_USE`. 0x03 for `UNKNOWN`, `ADDRESS` or `UNSUPPORTED`. 0x04 for `STATE`. 0x05 for `VALUE` |
| 0xF3 | `SET_I2S_INPUT_CHANNELS` | value in `old & 0xFF` | 1 byte `PIN_CONFIG_*` | `INPUT.I2S_RX_CHANNELS` `0x0331` | `0` | 1 byte: u8 `old & 0xFF` | 0x00 for OK. 0x01 for `VALUE` or `PIN_INVALID`. 0x02 for `PIN_IN_USE`. 0x03 for `UNKNOWN`, `ADDRESS` or `UNSUPPORTED`. 0x04 for `STATE` |
| 0xFE | `SET_I2S_CLOCK_PIN_MODE` | value in `old & 0xFF` | 1 byte `PIN_CONFIG_*` | `CLOCK.I2S_CLOCK_PIN_MODE` `0x0C11` | `0` | 1 byte: u8 `old & 0xFF` | 0x00 for OK. 0x01 for `PIN_INVALID`. 0x02 for `PIN_IN_USE`. 0x03 for `UNKNOWN`, `ADDRESS` or `UNSUPPORTED`. 0x04 for `STATE`. 0x05 for `VALUE` |

<!-- VC:END LEGACY_WASR -->

**Worked example 1: a refused output type change.** 1.1.6 packed the slot and the new type into `wValue` as `(type << 8) | slot`. Asking slot 0 for type 5, which does not exist, answered `01` (`PIN_CONFIG_INVALID_PIN`). In 1.1.7 the slot is the address and the type is the payload. The handler refuses the value after the data stage, so the status stage STALLs, and `SYS.LAST_RESULT` reads `VC_ERR_VALUE`. Nothing changes on the device. A valid type (0 S/PDIF or 1 I2S) would be ACKed and applied by the main loop under a synchronized restart of every output.

```
1.1.6  SET_OUTPUT_TYPE 0xC0, slot 0, type 5 (write-as-read)
SETUP  C1 C0 00 05 02 00 01 00
IN     01

1.1.7  OUTPUT.TYPE (page 0x04, command 0x10), slot 0
SETUP  41 10 00 00 02 04 01 00
OUT    05                              -> status stage STALL
SETUP  C1 14 00 00 02 01 08 00
IN     10 04 00 14 00 00 00 00         OUTPUT.TYPE, SET, VC_ERR_VALUE, wValue 0
```

**Worked example 2: cancelling a macro.** 1.1.6 used `CS_MACRO_FIRE` with `wValue` `0xFFFF` to cancel the running macro, and answered `01`. In 1.1.7 cancel is its own zero-length SET, and it always succeeds.

```
1.1.6  CS_MACRO_FIRE 0x25, wValue 0xFFFF (write-as-read)
SETUP  C1 25 FF FF 02 00 01 00
IN     01

1.1.7  CS.MACRO_CANCEL (page 0x0A, command 0x59)
SETUP  41 59 00 00 02 0A 00 00
```

The same pattern covers the presets. `PRESET_LOAD` of slot 3 was the IN request `C1 91 03 00 02 00 01 00`, answering a `PRESET_*` byte. In 1.1.7 it is `STORE.PRESET_LOAD` (page `0x02`, command `0x11`), a zero-length SET with the slot in `wValue`, `41 11 03 00 02 02 00 00`. Confirm the load with `STORE.PRESET_ACTIVE`, or wait for the `PRESET_LOADED` and `BULK_INVALIDATED` notifications. A slot that does not exist STALLs at SETUP.

```
1.1.7  STORE.PRESET_LOAD, slot 10
SETUP  41 11 0A 00 02 02 00 00     -> STALL
SETUP  C1 14 00 00 02 01 08 00
IN     11 02 00 13 0A 00 00 00         STORE.PRESET_LOAD, SET, VC_ERR_ADDRESS, wValue 10
```

### 6.7 Retired opcodes

These 1.1.6 requests have no command in 1.1.7. Each row gives the replacement.

<!-- VC:BEGIN RETIRED -->

| Old | Old name | Old dir | Replacement |
|---|---|---|---|
| 0x44 | `SET_PREAMP` | W | Write `ROUTE.PREAMP` (`0x0540`) once per input channel, `wValue` = input channel (0 to 1 on RP2040, 0 to 7 on RP2350), payload f32 dB |
| 0x45 | `GET_PREAMP` | R | Read `ROUTE.PREAMP` (`0x0540`) with `wValue` 0. 1.1.6 returned input channel 0 |
| 0x50 | `GET_STATUS` `wValue` 0 | R | `MEAS.LEVELS`, u16 at offsets 0 and 2 |
| 0x50 | `GET_STATUS` `wValue` 1 | R | `MEAS.LEVELS`, u16 at offsets 4 and 6 |
| 0x50 | `GET_STATUS` `wValue` 2 | R | `MEAS.LEVELS`, u16 at offset 8, then `SYS.SYSTEM_STATUS` bytes 12 and 13 |
| 0x50 | `GET_STATUS` `wValue` 7 | R | None. It always read 0 |
| 0x51 | `SAVE_PARAMS` | W-as-R | Read `STORE.PRESET_ACTIVE` (`0x0214`), then send `STORE.PRESET_SAVE` (`0x0210`) with `wValue` = that slot |
| 0xB2 | `GET_USB_ERROR_STATS` | R | None. Its six counters always read 0. Live USB health is in `SYS.USB_AUDIO_STATUS` and `SYS.PIPELINE_STATS` |
| 0xB3 | `RESET_USB_ERROR_STATS` | W-as-R | None. It did nothing. `SYS.STATS_RESET` resets the watermarks |

<!-- VC:END RETIRED -->

`SET_PREAMP` (`0x44`) wrote the same gain to every input channel. Do the same with one `ROUTE.PREAMP` SET per input channel. The input count is 2 on RP2040 and 8 on RP2350 (`SYS.PLATFORM` byte 0 tells you which). `GET_PREAMP` (`0x45`) returned input channel 0, which is `ROUTE.PREAMP` with `wValue` 0.

`SAVE_PARAMS` (`0x51`) saved the live state into the active preset. Read `STORE.PRESET_ACTIVE` (1 byte), then send `STORE.PRESET_SAVE` with that slot in `wValue`. It is a flash write followed by an output restart.

`GET_USB_ERROR_STATS` and `RESET_USB_ERROR_STATS` (`0xB2`, `0xB3`) always read zero and did nothing. Drop them.

`GET_STATUS` selectors 0, 1 and 2 packed peaks and CPU loads into 4 bytes. Read `MEAS.LEVELS` and `SYS.SYSTEM_STATUS` instead (6.3). Selector 7 always read 0.

---

## 7. Every GET reply, exactly

### 7.1 Reply lengths

Every command that has a GET, by page and command number, with its exact reply length. A host that asks for the length in this table gets exactly that many bytes. Asking for fewer gets a prefix of the same reply, and asking for more gets the whole reply and a short packet. A client that checks reply lengths strictly should re-derive every constant from this table (or from the registry) rather than from 1.1.6 code.

<!-- VC:BEGIN GET_LENGTHS -->

| Command | Page | Cmd | wValue | Reply bytes | Availability | Notes |
|---|---|---|---|---|---|---|
| `SYS.PROTOCOL_INFO` | 0x01 | 0x10 | none | 16 |  |  |
| `SYS.PAGE_LIST` | 0x01 | 0x11 | none | var |  | 2 x pages (`SYS.PROTOCOL_INFO` byte 6). 24 on a release build |
| `SYS.PAGE_COMMANDS` | 0x01 | 0x12 | page | 64 |  |  |
| `SYS.COMMAND_INFO` | 0x01 | 0x13 | `(page << 8) \| cmd` | 8 |  |  |
| `SYS.LAST_RESULT` | 0x01 | 0x14 | none | 8 |  |  |
| `SYS.PLATFORM` | 0x01 | 0x20 | none | 7 |  |  |
| `SYS.SERIAL` | 0x01 | 0x21 | none | 16 |  |  |
| `SYS.BUILD_INFO` | 0x01 | 0x22 | none | 64 |  |  |
| `SYS.CORE1_MODE` | 0x01 | 0x23 | none | 1 |  |  |
| `SYS.DEVICE_NAME` | 0x01 | 0x24 | none | 32 |  |  |
| `SYS.SYSTEM_STATUS` | 0x01 | 0x40 | none | 16 |  |  |
| `SYS.USB_AUDIO_STATUS` | 0x01 | 0x41 | none | 16 |  |  |
| `SYS.BUFFER_STATS` | 0x01 | 0x42 | none | 44 |  |  |
| `SYS.PIPELINE_STATS` | 0x01 | 0x43 | none | 24 |  |  |
| `SYS.PDM_STATS` | 0x01 | 0x44 | none | 16 |  |  |
| `SYS.SPDIF_TX_STATS` | 0x01 | 0x45 | none | 24 |  |  |
| `STORE.PRESET_NAME` | 0x02 | 0x13 | preset slot | 32 |  |  |
| `STORE.PRESET_ACTIVE` | 0x02 | 0x14 | none | 1 |  |  |
| `STORE.DIRECTORY` | 0x02 | 0x15 | none | 7 |  |  |
| `STORE.STARTUP` | 0x02 | 0x16 | none | 2 |  |  |
| `STORE.OUTPUT_CONFIG_MODE` | 0x02 | 0x20 | none | 1 |  |  |
| `STORE.ALL_PARAMS` | 0x02 | 0x40 | none | 6136 |  | Bulk buffer, `min(wLength, 6136)` |
| `STORE.ALL_PARAMS_CHUNK` | 0x02 | 0x41 | byte offset | var | USB only | min(wLength, 6136 - offset). wLength 0 is `VC_ERR_LENGTH` |
| `INPUT.SOURCE` | 0x03 | 0x10 | none | 1 |  |  |
| `INPUT.STATUS` | 0x03 | 0x11 | none | 12 |  |  |
| `INPUT.SOURCE_NAME` | 0x03 | 0x12 | source ID | 32 |  |  |
| `INPUT.SPDIF_RX_STATUS` | 0x03 | 0x20 | none | 16 |  |  |
| `INPUT.SPDIF_RX_CH_STATUS` | 0x03 | 0x21 | none | 24 |  |  |
| `INPUT.SPDIF_RX_PIN` | 0x03 | 0x22 | S/PDIF input | 1 |  |  |
| `INPUT.SPDIF_RX_ENABLE` | 0x03 | 0x23 | S/PDIF input | 1 |  |  |
| `INPUT.SPDIF_RX_CONFIG` | 0x03 | 0x24 | none | 6 |  |  |
| `INPUT.I2S_RX_PIN` | 0x03 | 0x30 | I2S pair | 1 |  |  |
| `INPUT.I2S_RX_CHANNELS` | 0x03 | 0x31 | none | 1 |  |  |
| `INPUT.I2S_RX_RATE` | 0x03 | 0x32 | none | 4 |  |  |
| `INPUT.ADAT_RX_ENABLE` | 0x03 | 0x40 | none | 1 | RP2350 only |  |
| `INPUT.ADAT_RX_PIN` | 0x03 | 0x41 | none | 1 | RP2350 only |  |
| `INPUT.ADAT_RX_CLOCK_MODE` | 0x03 | 0x42 | none | 1 | RP2350 only |  |
| `INPUT.ADAT_RX_STATUS` | 0x03 | 0x43 | none | 20 | RP2350 only |  |
| `INPUT.LG_SOUND_SYNC_ENABLE` | 0x03 | 0x50 | none | 1 |  |  |
| `INPUT.LG_SOUND_SYNC_STATUS` | 0x03 | 0x51 | none | 16 |  |  |
| `OUTPUT.TYPE` | 0x04 | 0x10 | output slot | 1 |  |  |
| `OUTPUT.PIN` | 0x04 | 0x11 | pin output | 1 |  |  |
| `OUTPUT.LATENCY` | 0x04 | 0x20 | none | 1 |  |  |
| `OUTPUT.LATENCY_STATUS` | 0x04 | 0x21 | none | 12 |  |  |
| `OUTPUT.LATENCY_OPTIONS` | 0x04 | 0x22 | none | var |  | 8 x options (`OUTPUT.LATENCY_STATUS` byte 9). 56 today |
| `OUTPUT.ADAT_TX_ENABLE` | 0x04 | 0x30 | none | 1 | RP2350 only |  |
| `OUTPUT.ADAT_TX_PIN` | 0x04 | 0x31 | none | 1 | RP2350 only |  |
| `OUTPUT.ADAT_TX_STATUS` | 0x04 | 0x32 | none | 8 | RP2350 only |  |
| `OUTPUT.DAC_MUTE_CONFIG` | 0x04 | 0x40 | none | 16 |  |  |
| `ROUTE.MATRIX_ROUTE` | 0x05 | 0x10 | `(input << 8) \| output` | 8 |  |  |
| `ROUTE.OUTPUT_ENABLE` | 0x05 | 0x20 | output | 1 |  |  |
| `ROUTE.OUTPUT_GAIN` | 0x05 | 0x21 | output | 4 |  |  |
| `ROUTE.OUTPUT_MUTE` | 0x05 | 0x22 | output | 1 |  |  |
| `ROUTE.OUTPUT_DELAY` | 0x05 | 0x23 | output | 4 |  |  |
| `ROUTE.OUTPUT_ENABLE_CONFLICT` | 0x05 | 0x24 | output | 1 |  |  |
| `ROUTE.CHANNEL_DELAY` | 0x05 | 0x30 | channel | 4 |  |  |
| `ROUTE.CHANNEL_NAME` | 0x05 | 0x31 | channel | 32 |  |  |
| `ROUTE.PREAMP` | 0x05 | 0x40 | input channel | 4 |  |  |
| `ROUTE.MASTER_VOLUME` | 0x05 | 0x50 | none | 4 |  |  |
| `ROUTE.MASTER_VOLUME_MODE` | 0x05 | 0x51 | none | 1 |  |  |
| `ROUTE.MASTER_VOLUME_SAVED` | 0x05 | 0x53 | none | 4 |  |  |
| `ROUTE.USER_VOLUME` | 0x05 | 0x60 | none | 4 |  |  |
| `ROUTE.USER_MUTE` | 0x05 | 0x61 | none | 1 |  |  |
| `FILTER.BAND` | 0x06 | 0x10 | `(channel << 8) \| band` | 16 |  |  |
| `FILTER.BAND_BYPASS` | 0x06 | 0x11 | `(channel << 8) \| band` | 1 |  |  |
| `FILTER.EQ_BYPASS` | 0x06 | 0x12 | none | 1 |  |  |
| `DYN.LIMITER_ENABLE` | 0x07 | 0x10 | output, 0xFF = all (SET) | 4 |  |  |
| `DYN.LIMITER_THRESHOLD` | 0x07 | 0x11 | output, 0xFF = all (SET) | 4 |  |  |
| `DYN.LIMITER_RELEASE` | 0x07 | 0x12 | output, 0xFF = all (SET) | 4 |  |  |
| `DYN.LIMITER_LINK_GROUP` | 0x07 | 0x13 | output, 0xFF = all (SET) | 4 |  |  |
| `DYN.LIMITER_METERS` | 0x07 | 0x18 | none | var |  | 2 x outputs. 10 on RP2040, 18 on RP2350 |
| `DYN.LIMITER_STATUS` | 0x07 | 0x19 | none | 4 |  |  |
| `DYN.LEVELLER_ENABLE` | 0x07 | 0x20 | none | 1 |  |  |
| `DYN.LEVELLER_AMOUNT` | 0x07 | 0x21 | none | 4 |  |  |
| `DYN.LEVELLER_SPEED` | 0x07 | 0x22 | none | 1 |  |  |
| `DYN.LEVELLER_MAX_GAIN` | 0x07 | 0x23 | none | 4 |  |  |
| `DYN.LEVELLER_LOOKAHEAD` | 0x07 | 0x24 | none | 1 |  |  |
| `DYN.LEVELLER_GATE` | 0x07 | 0x25 | none | 4 |  |  |
| `DYN.LEVELLER_MASKS` | 0x07 | 0x26 | none | 2 |  |  |
| `DYN.LOUDNESS_ENABLE` | 0x07 | 0x30 | none | 1 |  |  |
| `DYN.LOUDNESS_REF` | 0x07 | 0x31 | none | 4 |  |  |
| `DYN.LOUDNESS_INTENSITY` | 0x07 | 0x32 | none | 4 |  |  |
| `DYN.LOUDNESS_MASK` | 0x07 | 0x33 | none | 2 |  |  |
| `FX.CROSSFEED_ENABLE` | 0x08 | 0x10 | none | 1 |  |  |
| `FX.CROSSFEED_PRESET` | 0x08 | 0x11 | none | 1 |  |  |
| `FX.CROSSFEED_FREQ` | 0x08 | 0x12 | none | 4 |  |  |
| `FX.CROSSFEED_FEED` | 0x08 | 0x13 | none | 4 |  |  |
| `FX.CROSSFEED_ITD` | 0x08 | 0x14 | none | 1 |  |  |
| `FX.CROSSFEED_OUTPUTS` | 0x08 | 0x15 | none | 1 |  |  |
| `FX.SUBHARM_ENABLE` | 0x08 | 0x20 | none | 1 |  |  |
| `FX.SUBHARM_LOW` | 0x08 | 0x21 | none | 4 |  |  |
| `FX.SUBHARM_HIGH` | 0x08 | 0x22 | none | 4 |  |  |
| `FX.SUBHARM_TOP` | 0x08 | 0x23 | none | 4 |  |  |
| `FX.SUBHARM_BOOST` | 0x08 | 0x24 | none | 4 |  |  |
| `FX.SUBHARM_MASK` | 0x08 | 0x25 | none | 2 |  |  |
| `FX.SUBHARM_SELECT` | 0x08 | 0x26 | none | 1 |  |  |
| `FX.SUBHARM_DEPTH` | 0x08 | 0x27 | none | 4 |  |  |
| `FX.SUBHARM_HOLD` | 0x08 | 0x28 | none | 4 |  |  |
| `FX.SUBHARM_CEILING` | 0x08 | 0x29 | none | 4 |  |  |
| `FX.SUBHARM_LINK` | 0x08 | 0x2A | none | 1 |  |  |
| `FX.SUBHARM_SOLO` | 0x08 | 0x2B | none | 1 |  |  |
| `FX.SUBHARM_HEADROOM` | 0x08 | 0x2E | none | 4 |  |  |
| `FX.SUBHARM_METER` | 0x08 | 0x2F | none | var |  | 2 x outputs. 10 on RP2040, 18 on RP2350 |
| `FX.PSYBASS_ENABLE` | 0x08 | 0x40 | none | 1 |  |  |
| `FX.PSYBASS_CUTOFF` | 0x08 | 0x41 | none | 4 |  |  |
| `FX.PSYBASS_HARMONICS` | 0x08 | 0x42 | none | 4 |  |  |
| `FX.PSYBASS_DRIVE` | 0x08 | 0x43 | none | 4 |  |  |
| `FX.PSYBASS_CHARACTER` | 0x08 | 0x44 | none | 4 |  |  |
| `FX.PSYBASS_ORIGINAL` | 0x08 | 0x45 | none | 4 |  |  |
| `FX.PSYBASS_MASK` | 0x08 | 0x46 | none | 2 |  |  |
| `FX.TUBE_ENABLE` | 0x08 | 0x60 | none | 4 |  |  |
| `FX.TUBE_OUTPUT_MASK` | 0x08 | 0x61 | none | 4 |  |  |
| `FX.TUBE_TYPE` | 0x08 | 0x62 | none | 4 |  |  |
| `FX.TUBE_DRIVE` | 0x08 | 0x63 | none | 4 |  |  |
| `FX.TUBE_BIAS` | 0x08 | 0x64 | none | 4 |  |  |
| `FX.TUBE_ASYM` | 0x08 | 0x65 | none | 4 |  |  |
| `FX.TUBE_HARDNESS` | 0x08 | 0x66 | none | 4 |  |  |
| `FX.TUBE_SAG` | 0x08 | 0x67 | none | 4 |  |  |
| `FX.TUBE_RECTIFIER` | 0x08 | 0x68 | none | 4 |  |  |
| `FX.TUBE_XFMR_ENABLE` | 0x08 | 0x69 | none | 4 |  |  |
| `FX.TUBE_XFMR_DAMPING` | 0x08 | 0x6A | none | 4 |  |  |
| `FX.TUBE_XFMR_RES` | 0x08 | 0x6B | none | 4 |  |  |
| `FX.TUBE_MIX` | 0x08 | 0x6C | none | 4 |  |  |
| `FX.TUBE_TRIM` | 0x08 | 0x6D | none | 4 |  |  |
| `FX.UPMIX_CONFIG` | 0x08 | 0x80 | none | 44 | RP2350 only |  |
| `FX.UPMIX_STATUS` | 0x08 | 0x81 | none | 16 | RP2350 only |  |
| `FX.UPMIX_ENABLE` | 0x08 | 0x90 | none | 4 | RP2350 only |  |
| `FX.UPMIX_CENTER_MODE` | 0x08 | 0x91 | none | 4 | RP2350 only |  |
| `FX.UPMIX_SURROUND_MODE` | 0x08 | 0x92 | none | 4 | RP2350 only |  |
| `FX.UPMIX_STRENGTH` | 0x08 | 0x93 | none | 4 | RP2350 only |  |
| `FX.UPMIX_CENTER_WIDTH` | 0x08 | 0x94 | none | 4 | RP2350 only |  |
| `FX.UPMIX_THRESHOLD` | 0x08 | 0x95 | none | 4 | RP2350 only |  |
| `FX.UPMIX_ATTACK` | 0x08 | 0x96 | none | 4 | RP2350 only |  |
| `FX.UPMIX_RELEASE` | 0x08 | 0x97 | none | 4 | RP2350 only |  |
| `FX.UPMIX_DET_HPF` | 0x08 | 0x98 | none | 4 | RP2350 only |  |
| `FX.UPMIX_SUR_DELAY` | 0x08 | 0x99 | none | 4 | RP2350 only |  |
| `FX.UPMIX_SUR_HPF` | 0x08 | 0x9A | none | 4 | RP2350 only |  |
| `FX.UPMIX_SUR_LPF` | 0x08 | 0x9B | none | 4 | RP2350 only |  |
| `FX.UPMIX_DECORR` | 0x08 | 0x9C | none | 4 | RP2350 only |  |
| `FX.UPMIX_PRESENCE` | 0x08 | 0x9D | none | 4 | RP2350 only |  |
| `MEAS.LEVELS` | 0x09 | 0x10 | none | var |  | 2 x channels + 7. 21 on RP2040, 41 on RP2350 |
| `MEAS.CLIP_FLAGS` | 0x09 | 0x11 | none | 4 |  |  |
| `MEAS.RTA_CONFIG` | 0x09 | 0x20 | none | 12 |  |  |
| `MEAS.RTA_CAPS` | 0x09 | 0x21 | none | 16 |  |  |
| `MEAS.RTA_BAND_CENTRES` | 0x09 | 0x22 | index | var |  | 2 x min(32, bands - 32n) for chunk n. 64, then 10 with 37 bands |
| `MEAS.RTA_BANDS` | 0x09 | 0x23 | index | 82 |  |  |
| `MEAS.RTA_BINS` | 0x09 | 0x24 | byte offset | var |  | min(256, frame - offset). The frame is 145, 273 or 529 bytes for FFT order 8, 9 or 10 |
| `MEAS.RTA_STATUS` | 0x09 | 0x25 | none | 24 |  |  |
| `MEAS.RTA_BANDS_ALL` | 0x09 | 0x26 | none | var | USB only | 82 x live channels at the tap, 0 when none |
| `MEAS.SIGGEN_CONFIG` | 0x09 | 0x40 | none | 36 |  |  |
| `MEAS.SIGGEN_STATUS` | 0x09 | 0x41 | none | 16 |  |  |
| `MEAS.SIGGEN_CAPS` | 0x09 | 0x42 | none | 8 |  |  |
| `MEAS.SIGGEN_TYPE_DESC` | 0x09 | 0x43 | index | 62 |  |  |
| `CS.CAPS` | 0x0A | 0x10 | none | 52 |  |  |
| `CS.NOUN_DESC` | 0x0A | 0x11 | CS noun | 12 |  |  |
| `CS.STATUS` | 0x0A | 0x12 | none | 41 |  |  |
| `CS.EXT_STATUS` | 0x0A | 0x13 | none | 24 |  |  |
| `CS.BINDING` | 0x0A | 0x20 | CS slot | 24 |  |  |
| `CS.SLOT_NAME` | 0x0A | 0x21 | CS slot | 32 |  |  |
| `CS.IR_CMD` | 0x0A | 0x30 | CS sub-slot | 16 |  |  |
| `CS.IR_LEARN_STATUS` | 0x0A | 0x31 | none | 8 |  |  |
| `CS.GROUP` | 0x0A | 0x40 | CS group | 40 |  |  |
| `CS.MACRO` | 0x0A | 0x50 | CS macro | 132 |  |  |
| `CS.MACRO_HEADER` | 0x0A | 0x51 | CS macro | 36 |  |  |
| `CS.MACRO_STEP` | 0x0A | 0x52 | CS macro step | 12 |  |  |
| `CS.AUX_STATE` | 0x0A | 0x60 | CS slot | 1 |  |  |
| `CS.AUX_LEVEL` | 0x0A | 0x61 | CS slot | 2 |  |  |
| `CS.AUX_ALL` | 0x0A | 0x62 | none | 48 |  |  |
| `CS.DISPLAY_CONFIG` | 0x0A | 0x70 | none | 12 |  |  |
| `CS.DISPLAY_CAPS` | 0x0A | 0x71 | none | 4 |  |  |
| `CS.DISPLAY_PAGE` | 0x0A | 0x72 | CS page | 4 |  |  |
| `CS.DISPLAY_STATUS` | 0x0A | 0x73 | none | 8 |  |  |
| `XPORT.UART_CONFIG` | 0x0B | 0x10 | none | 8 |  |  |
| `XPORT.I2C_CONFIG` | 0x0B | 0x11 | none | 8 |  |  |
| `XPORT.STATUS` | 0x0B | 0x12 | none | 8 |  |  |
| `CLOCK.I2S_BCK_PIN` | 0x0C | 0x10 | role 0-1 | 1 |  |  |
| `CLOCK.I2S_CLOCK_PIN_MODE` | 0x0C | 0x11 | none | 1 |  |  |
| `CLOCK.I2S_CLOCK_MODE` | 0x0C | 0x12 | none | 1 |  |  |
| `CLOCK.I2S_SLAVE_STATUS` | 0x0C | 0x13 | none | 16 |  |  |
| `CLOCK.MCK_ENABLE` | 0x0C | 0x20 | none | 1 |  |  |
| `CLOCK.MCK_PIN` | 0x0C | 0x21 | none | 1 |  |  |
| `CLOCK.MCK_MULTIPLIER` | 0x0C | 0x22 | none | 1 |  |  |
| `DIAG.CLOCK_DIAG` | 0xF0 | 0x10 | none | var | `DSPI_CLOCK_DIAG`, USB only | `ClockDiagPacket`. 226 on RP2040, 248 on RP2350 |
| `DIAG.LOOPBACK_STATS` | 0xF0 | 0x20 | none | 8 | `DSPI_LOOPBACK` |  |
| `DIAG.LOOP_PROF_SECTION` | 0xF0 | 0x30 | index | 4 | `DSPI_LOOP_PROF` |  |
| `DIAG.LOOP_PROF_JOB` | 0xF0 | 0x31 | index | 4 | `DSPI_LOOP_PROF` |  |
| `DIAG.LOOP_PROF_COMMAND` | 0xF0 | 0x32 | `(page << 8) \| cmd` | 4 | `DSPI_LOOP_PROF` |  |
| `DIAG.LOOP_PROF_BG_PACE` | 0xF0 | 0x33 | none | 2 | `DSPI_LOOP_PROF` |  |

<!-- VC:END GET_LENGTHS -->

On UART and I2C a GET with `wLength` 0 returns the whole reply. On USB `wLength` 0 returns nothing.

### 7.2 Which replies kept their 1.1.6 bytes

For each page, "identical" means the 1.1.7 reply has the same length, layout and values as the 1.1.6 reply of the old opcode named in section 6.2, for the same request. A strict client can keep its parser and its constants for those and change only the request. "Changed" and "new" replies are written out in full in section 7.3 or in the section named.

| Page | Identical to 1.1.6 | Changed | New |
|---|---|---|---|
| `SYS` | `PLATFORM` (7), `SERIAL` (16), `BUILD_INFO` (64), `CORE1_MODE` (1), `DEVICE_NAME` (32), `BUFFER_STATS` (44) | None | `PROTOCOL_INFO` (4.2), `PAGE_LIST`, `PAGE_COMMANDS`, `COMMAND_INFO` (4.7), `LAST_RESULT` (5.2), `SYSTEM_STATUS`, `USB_AUDIO_STATUS`, `PIPELINE_STATS`, `PDM_STATS`, `SPDIF_TX_STATS` (6.3, replacing `GET_STATUS`) |
| `STORE` | `PRESET_NAME` (32), `PRESET_ACTIVE` (1), `DIRECTORY` (7), `OUTPUT_CONFIG_MODE` (1), `ALL_PARAMS` (6136), `ALL_PARAMS_CHUNK` | `STARTUP`, 2 bytes where `PRESET_GET_STARTUP` was 3 (6.5) | None |
| `INPUT` | `SOURCE` (1), `SOURCE_NAME` (32), `SPDIF_RX_STATUS` (16), `SPDIF_RX_CH_STATUS` (24), `SPDIF_RX_PIN` (1), `SPDIF_RX_CONFIG` (6), `I2S_RX_PIN` (1), `I2S_RX_CHANNELS` (1), `ADAT_RX_ENABLE` (1), `ADAT_RX_PIN` (1), `ADAT_RX_CLOCK_MODE` (1), `ADAT_RX_STATUS` (20), `LG_SOUND_SYNC_ENABLE` (1), `LG_SOUND_SYNC_STATUS` (16) | `I2S_RX_RATE`, 4 bytes where `GET_INPUT_RATE` was 8 (6.5) | `STATUS` (6.3), `SPDIF_RX_ENABLE` GET (7.3) |
| `OUTPUT` | `TYPE` (1), `PIN` (1), `LATENCY_STATUS` (12), `LATENCY_OPTIONS` (8 per option, 56 today), `ADAT_TX_ENABLE` (1), `ADAT_TX_PIN` (1), `ADAT_TX_STATUS` (8), `DAC_MUTE_CONFIG` (16) | None | `LATENCY` GET (7.3) |
| `ROUTE` | `OUTPUT_ENABLE` (1), `OUTPUT_GAIN` (4), `OUTPUT_MUTE` (1), `OUTPUT_DELAY` (4), `OUTPUT_ENABLE_CONFLICT` (1), `CHANNEL_DELAY` (4), `CHANNEL_NAME` (32), `PREAMP` (4), `MASTER_VOLUME` (4), `MASTER_VOLUME_MODE` (1), `MASTER_VOLUME_SAVED` (4), `USER_VOLUME` (4), `USER_MUTE` (1) | `MATRIX_ROUTE`, same length, new layout (6.5) | None |
| `FILTER` | `BAND_BYPASS` (1), `EQ_BYPASS` (1) | `BAND`, 16 bytes replacing the 4-byte `GET_EQ_PARAM` reads (6.5) | None |
| `DYN` | Every reply. The four limiter parameters (4 each) are the 1.1.6 `LIMITER` index 0 to 3 replies, `LIMITER_METERS` and `LIMITER_STATUS` the index `0x80` and `0x81` replies | None | None |
| `FX` | Every reply, including the tube and upmixer parameters (4 each, float32 as before) | None | None |
| `MEAS` | `LEVELS` (the old `GET_STATUS` selector 9 packet), `RTA_CONFIG` (12), `RTA_CAPS` (16), `RTA_BAND_CENTRES`, `RTA_BANDS` (82), `RTA_STATUS` (24), `RTA_BANDS_ALL`, `SIGGEN_CONFIG` (36), `SIGGEN_STATUS` (16), `SIGGEN_CAPS` (8), `SIGGEN_TYPE_DESC` (62) | `RTA_BINS`, same frame, at most 256 bytes per request (7.3) | `CLIP_FLAGS` (6.5, the bytes `CLEAR_CLIPS` returned, without the clear) |
| `CS` | `CAPS` (52), `NOUN_DESC` (12), `STATUS` (41), `EXT_STATUS` (24), `BINDING` (24), `SLOT_NAME` (32), `IR_CMD` (16), `IR_LEARN_STATUS` (8), `GROUP` (40), `MACRO` (132), `AUX_STATE` (1), `AUX_LEVEL` (2), `AUX_ALL` (48), `DISPLAY_PAGE` (4), `DISPLAY_STATUS` (8) | `DISPLAY_CONFIG`, 12 bytes where `GET_CS_DISPLAY_CFG` was 16 (6.5) | `MACRO_HEADER` GET, `MACRO_STEP` GET, `DISPLAY_CAPS` (7.3) |
| `XPORT` | `UART_CONFIG` (8), `I2C_CONFIG` (8), `STATUS` (8) | None | None |
| `CLOCK` | `I2S_BCK_PIN` (1), `I2S_CLOCK_PIN_MODE` (1), `I2S_CLOCK_MODE` (1), `I2S_SLAVE_STATUS` (16), `MCK_ENABLE` (1), `MCK_PIN` (1), `MCK_MULTIPLIER` (1) | None | None |
| `DIAG` | `CLOCK_DIAG` in a `DSPI_CLOCK_DIAG` build (226 or 248) | None | `LOOPBACK_STATS` (6.3), `LOOP_PROF_SECTION`, `LOOP_PROF_JOB`, `LOOP_PROF_COMMAND`, `LOOP_PROF_BG_PACE` (7.3) |

Three things can still differ for an identical reply, because they are refusals rather than replies.

- A request that 1.1.6 answered with zeros now STALLs (5.5). That covers bad indexes on the pin reads, `ROUTE.OUTPUT_ENABLE_CONFLICT` past the last output, RP2350-only commands on RP2040 and `DIAG.CLOCK_DIAG` on a release build (1.1.6 answered zero bytes).
- A non-zero `wValue` on a command that addresses nothing now STALLs.
- A request with the wrong page or command number gets nothing at all. On UART and I2C the frame comes back with a non-zero status and a length of 0.

### 7.3 Layouts of new and changed replies

The new and changed replies not already written out above.

`STORE.STARTUP` (`0x0216`, 2 bytes, GET and SET).

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Startup mode: 0 load the default slot, 1 load the last active slot |
| 1 | u8 | Default slot, 0 to 9 |

`INPUT.SPDIF_RX_ENABLE` (`0x0323`, 1 byte, GET and SET, `wValue` = S/PDIF input 1 to 3). 1 while the optional input is enabled, else 0. Input 0 is always enabled, so `wValue` 0 is `VC_ERR_ADDRESS` in both directions.

`INPUT.I2S_RX_RATE` (`0x0332`, 4 bytes, GET and SET). u32, the rate in Hz the device uses when it is the clock master for I2S or ADAT input, 44100, 48000 or 96000.

`OUTPUT.LATENCY` (`0x0420`, 1 byte, GET and SET). The option ID, 1 to 7. The GET returns the queued ID while a change is pending and the stored ID otherwise, so a SET reads back at once.

`MEAS.CLIP_FLAGS` (`0x0911`, 4 bytes). u32, latched clip flags, bit k = channel k. Reading does not clear them.

`MEAS.RTA_BINS` (`0x0924`, `wValue` = byte offset). The frame is unchanged. It is a 16-byte header, `n_bins` level bytes and a final byte that repeats the header's counter. It is 145, 273 or 529 bytes for FFT order 8, 9 or 10. 1.1.6 sent up to the whole frame in one read. 1.1.7 sends at most 256 bytes per read, so read at offsets 0, 256 and 512 and compare byte 2 with the last byte. If they differ, the frame changed between reads, so read it again. Before the first transform the device returns a frame with a zero header and a final byte of `0xFF`, so check the counter before trusting the data. An offset at or past the frame length is `VC_ERR_ADDRESS`.

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Version (3) |
| 1 | u8 | Channel |
| 2 | u8 | Frame counter, repeated as the frame's last byte |
| 3 | u8 | FFT order |
| 4 | u32 | Sample rate, Hz |
| 8 | u16 | `n_bins` |
| 10 | u16[3] | Reserved |
| 16 | u8[n_bins] | Bin levels, 0.5 dB steps, 243 is 0 dBFS |
| 16 + n_bins | u8 | Frame counter again |

`CS.MACRO_HEADER` (`0x0A51`, 36 bytes, GET and SET, `wValue` = macro 0 to 7). The GET is new and returns exactly what the SET accepts.

| Offset | Type | Field |
|---|---|---|
| 0 | char[32] | Name, NUL padded |
| 32 | u8 | Step count, 0 to 8 |
| 33 | u8[3] | Reserved, 0 |

`CS.MACRO_STEP` (`0x0A52`, 12 bytes, GET and SET, `wValue` = `(step << 8) | macro`). The GET is new.

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Noun |
| 1 | u8 | Action |
| 2 | u8 | Flags |
| 3 | u8 | Target |
| 4 | u8 | Index |
| 5 | u8 | Reserved |
| 6 | i16 | Value |
| 8 | i16 | Step |
| 10 | u16 | Delay before the step, 10 ms units |

`CS.DISPLAY_CONFIG` (`0x0A70`, 12 bytes, GET and SET, `wValue` 0). These are bytes 4 to 15 of the old `GET_CS_DISPLAY_CFG` reply and exactly the old SET payload.

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Mode: 0 fixed, 1 cycle the selected pages, 2 cycle all pages |
| 1 | u8 | Home page, 0 to 15 |
| 2 | u16 | Dwell per page, 0.1 s. At least 10 in the cycle modes |
| 4 | u16 | Change overlay hold, 0.1 s. 0 turns the overlay off |
| 6 | u8 | Brightness |
| 7 | u8 | Flags: bit 0 overlay for any change, bit 1 edits need arming, bits 3:2 label alignment, bits 5:4 value alignment |
| 8 | u16 | Edit timeout, 0.1 s |
| 10 | u8[2] | Reserved, 0 |

`CS.DISPLAY_CAPS` (`0x0A71`, 4 bytes). u8 page count (16), u8 model count (9), 2 reserved bytes. These are bytes 0 to 3 of the old `GET_CS_DISPLAY_CFG` reply.

`DIAG.LOOP_PROF_SECTION`, `DIAG.LOOP_PROF_JOB` and `DIAG.LOOP_PROF_COMMAND` (`0xF030` to `0xF032`, 4 bytes each, `DSPI_LOOP_PROF` builds only) return a u32 worst time in us. `DIAG.LOOP_PROF_BG_PACE` (`0xF033`, 2 bytes) is a u16 pacing divider. In 1.1.6 these were `GET_STATUS` selectors 64 and up in the same bench builds.

### 7.4 Constants a strict client checks

These values have not changed since 1.1.6. They are what a client that checks ranges and counts is likely to assert.

| Quantity | RP2040 | RP2350 |
|---|---|---|
| Channels (`NUM_CHANNELS`) | 7 | 17 |
| Input channels | 2 | 8 |
| Outputs (`SYS.PLATFORM` byte 3) | 5 | 9 |
| Output slots | 2 | 4 |
| Pin outputs | 3 | 5 |
| EQ bands per channel in use | 10 (bands 0 to 9) | 10 |
| Crossover bands | 20 to 23, output channels only | same |
| Input source IDs (`INPUT.SOURCE`) | 0 to 6, with 3 (ADAT) refused | 0 to 6 |
| S/PDIF inputs (`INPUT.SPDIF_RX_CONFIG` byte 0) | 4 | 4 |
| I2S input pairs | 1 | 4 |
| Preset slots | 10 (0 to 9) | 10 |
| Output latency options | 7 (IDs 1 to 7, default 5) | 7 |
| Control Surfaces binding slots, IR sub-slots | 16, 16 | 16, 16 |
| Groups, macros, steps per macro, display pages, display models | 8, 8, 8, 16, 9 | same |
| `CS.CAPS` capability version, type count, noun count | 22, 11, 81 | same |
| Spectrum analyser version, bands, bass bands | 3, 37, 14 | same |
| Signal generator version, signal types | 1, 15 | same |
| `WireBulkParams` format version, length | 32, 6136 | same |
| `XPORT.STATUS` byte 4, transport framing version | 1 | 1 |

New constants. The protocol major is 1 and the minor 0. A release build implements 12 pages, every page at revision 1. The largest SET payload is 64 bytes and the largest non-bulk reply 256 bytes.

---

## 8. Per-page change notes

Every place where a 1.1.7 command behaves differently from its 1.1.6 opcode beyond the renumbering. The rules of section 5.5 apply on top.

### 8.1 Page 0

Every request with a zero `wIndex` high byte is refused. USB STALLs at SETUP, UART and I2C return status `0x10`, and `SYS.LAST_RESULT` records `VC_ERR_UNKNOWN`. Page 0 has no `GET_PLATFORM` and no `ENTER_BOOTLOADER`, so a 1.1.6 host can neither read the version of a 1.1.7 device nor reboot it into the bootloader. The Microsoft OS 2.0 descriptor request (IN, `bRequest` `0x01`, `wIndex` 7) is not a DSPi command and is served as before. Command numbers `0x00` to `0x0F` are reserved in every page so that no DSPi command can collide with requests of that kind.

### 8.2 Discovery

- `SYS.PAGE_COMMANDS` for a page the build does not implement (page 0, or `DIAG` in a release build) is `VC_ERR_ADDRESS`. It never returns an empty bitmap, so treat that result as "page absent".
- The `SYS.COMMAND_INFO` flags from bit 3 up are informational (4.7). `RESTART` marks every command that can interrupt audio, including the flash writers, the input pin and clock commands (only while that input is live), `OUTPUT.DAC_MUTE_TEST` and `SYS.ENTER_BOOTLOADER`. A command without the flag never restarts or mutes the outputs.
- The discovery commands are recorded in `SYS.LAST_RESULT` like any other request.

### 8.3 SYS

| Old | New | Difference |
|---|---|---|
| `0x50` selector 8 | `SYS.SPDIF_TX_STATS` offset 0 | Same counter, `spdif_underruns`. It counts USB audio packet gaps longer than 2 ms and shorter than 50 ms (none during a preset load), not S/PDIF transmitter events. 1.1.6 described it as S/PDIF underruns |
| `0x50` selectors 11, 12 | `SYS.USB_AUDIO_STATUS` | Alt setting and mounted are u8 fields. They were u32 replies |
| `0x50` selector 14 | `SYS.SYSTEM_STATUS` | Core voltage is a u16 |
| `0x50` selector 16 | `SYS.SYSTEM_STATUS` | Temperature is an i16 in hundredths of a degree. 1.1.6 sent it sign-extended to 32 bits |
| `0x50` unknown selector | None | 1.1.6 answered four zero bytes. There are no selectors any more |
| `0xB0` | `SYS.BUFFER_STATS` | Identical bytes. Each read still advances the packet's sequence counter |
| `0xB1` | `SYS.STATS_RESET` | The mask is a 1-byte payload (it was `wValue`). Any bit other than bit 0 is `VC_ERR_VALUE` (it was ignored). No reply byte |
| `0x56` | `SYS.DEVICE_NAME` SET | An empty payload is `VC_ERR_LENGTH`, and so is one longer than 32 bytes. The device still keeps at most 31 bytes plus a NUL |
| `0xF0` | `SYS.ENTER_BOOTLOADER` | Zero-length SET. Over USB the status stage completes before the reboot, so the host sees a successful transfer. Over UART and I2C the reboot follows the response frame |

### 8.4 STORE

| Old | New | Difference |
|---|---|---|
| `0x90`, `0x91`, `0x92` | `STORE.PRESET_SAVE`, `STORE.PRESET_LOAD`, `STORE.PRESET_DELETE` | Zero-length SETs addressed by slot. A bad slot is `VC_ERR_ADDRESS` (it was the in-band byte `0x01`) |
| `0x94` | `STORE.PRESET_NAME` SET | A bad slot is `VC_ERR_ADDRESS` (it was dropped later, silently). 0 or more than 32 bytes is `VC_ERR_LENGTH`. The device still stores 31 bytes plus a NUL. Naming an empty slot is allowed |
| `0x96` | `STORE.STARTUP` SET | Exactly 2 bytes. A mode above 1 or a slot above 9 is `VC_ERR_VALUE` (it was refused later, silently) |
| `0x97` | `STORE.STARTUP` GET | 2 bytes, `{mode, slot}`. The old third byte, the last active slot, is `STORE.DIRECTORY` byte 4 |
| `0x98` | `STORE.OUTPUT_CONFIG_MODE` SET | Exactly 1 byte. A value above 1 stores 0 |
| `0x52`, `0x53` | `STORE.OUTPUT_CONFIG_SAVE`, `STORE.FACTORY_RESET` | Zero-length SETs, no status byte |
| `0x51` | None | Retired. Read `STORE.PRESET_ACTIVE`, then `STORE.PRESET_SAVE` to that slot |
| `0xA1` | `STORE.ALL_PARAMS` SET | Any length other than 6136 STALLs at SETUP with `VC_ERR_LENGTH` (1.1.6 ACKed and ignored 64 bytes or fewer). A header the apply would refuse STALLs the status stage with `VC_ERR_VALUE` (1.1.6 ACKed and dropped it) |
| `0xA2`, `0xA3` | `STORE.ALL_PARAMS_CHUNK` | Same semantics, USB only, chunk size 1 to the rest of the blob. Refusals now carry a reason (section 9.3) |

`STORE.FACTORY_RESET` behaves exactly as `FACTORY_RESET` did, and its flags now state it (`DEFERRED`, `RESTART`, no `FLASH`). It resets the live state to factory defaults under a muted, synchronized output restart and switches the input to USB. It writes no flash. It keeps every stored preset, slot name and device-wide setting, keeps the active slot selected and keeps the master volume. In IO config mode 1 the IO configuration returns to firmware defaults, and in mode 0 the stored device-wide IO configuration is applied again. The host receives one `BULK_INVALIDATED` notification with source 4. To make the reset state permanent, save it to a preset afterwards.

### 8.5 INPUT

| Old | New | Difference |
|---|---|---|
| `0xE0` | `INPUT.SOURCE` SET | A value above 6 is `VC_ERR_VALUE`. A disabled optional S/PDIF input, or ADAT while it is disabled or has no pin, is `VC_ERR_STATE`. ADAT on RP2040 is `VC_ERR_UNSUPPORTED`. 1.1.6 ignored all three silently. A SET of the active source is still a successful no-op |
| `0xE1` and `0x50` selectors 23, 32, 33 | `INPUT.SOURCE` GET and `INPUT.STATUS` | `INPUT.STATUS` is one 12-byte struct with the source, the channel count, the measured S/PDIF rate and the clock trim |
| `0x54` | `INPUT.SOURCE_NAME` SET | A bad source is `VC_ERR_ADDRESS` (it was a no-op) |
| `0xE9` | `INPUT.SPDIF_RX_ENABLE` | Addresses inputs 1 to 3 only. Input 0 is `VC_ERR_ADDRESS` (1.1.6 answered success for enabling it). Invalid pin and pin in use are separate results (both were `0x02`). Disable while active is `VC_ERR_STATE`. The GET is new |
| `0xE4` | `INPUT.SPDIF_RX_PIN` SET | Index in `wValue`, GPIO in a 1-byte payload. `0xFF` restores the input's default pin |
| `0xE5`, `0xF2` with a bad index | `INPUT.SPDIF_RX_PIN`, `INPUT.I2S_RX_PIN` GET | `VC_ERR_ADDRESS` (it was a zero byte) |
| `0xF1` | `INPUT.I2S_RX_PIN` SET | Pair in `wValue`, GPIO in a 1-byte payload |
| `0xED` | `INPUT.I2S_RX_RATE` SET | A rate other than 44100, 48000 or 96000 is `VC_ERR_VALUE` (it was ignored) |
| `0xEE` | `INPUT.I2S_RX_RATE` GET | 4 bytes, the selected I2S rate. The live rate half is `SYS.SYSTEM_STATUS` bytes 4 to 7 |
| `0xF3` | `INPUT.I2S_RX_CHANNELS` SET | A bad count is `VC_ERR_VALUE` (old `0x01`). More pairs than the platform has is `VC_ERR_UNSUPPORTED` (old `0x03`) |
| `0x68` | `INPUT.ADAT_RX_ENABLE` SET | Disable while live or pending is `VC_ERR_STATE` (old `0x02`). Enable with no pin is `VC_ERR_PIN_INVALID` (old `0x01`) |
| `0x6A` | `INPUT.ADAT_RX_PIN` SET | `0xFF` clears the pin and is accepted only while the input is disabled, otherwise `VC_ERR_STATE` (old `0x02`) |
| `0x68` to `0x6E` on RP2040 | None | The commands do not exist on RP2040. 1.1.6 on RP2040 stored and returned the clock mode, and answered the other reads with zeros |

### 8.6 OUTPUT

| Old | New | Difference |
|---|---|---|
| `0xC0` | `OUTPUT.TYPE` SET | Slot in `wValue`, type in a 1-byte payload. A bad type is `VC_ERR_VALUE` (old `0x01`) |
| `0x7C` | `OUTPUT.PIN` SET | Pin output in `wValue`, GPIO in a 1-byte payload |
| `0x03` OUT | `OUTPUT.LATENCY` SET | An unknown option ID is `VC_ERR_VALUE` (it was ignored on USB) |
| `0x03` IN | `OUTPUT.LATENCY_STATUS`, `OUTPUT.LATENCY_OPTIONS`, and a new 1-byte `OUTPUT.LATENCY` GET | The new GET returns the queued ID while a change is pending, otherwise the stored one |
| `0xEA` | `OUTPUT.DAC_MUTE_CONFIG` SET | With enable set, a bad active-low flag, a hold time outside 1 to 500 ms or a release time above 500 ms is `VC_ERR_VALUE` at once. The pin's validity and conflicts are still checked later by the main loop, and a configuration it refuses is still dropped, so read it back to confirm |
| `0xEC` | `OUTPUT.DAC_MUTE_TEST` | Zero-length SET. While the feature is disabled it is `VC_ERR_STATE` (old `0x03`) |
| `0xCA`, `0xCC` | `OUTPUT.ADAT_TX_ENABLE`, `OUTPUT.ADAT_TX_PIN` SET | Value in a 1-byte payload. RP2350 only. On RP2040 the commands do not exist (1.1.6 answered `0x03`) |

### 8.7 CLOCK

| Old | New | Difference |
|---|---|---|
| `0xC2` | `CLOCK.I2S_BCK_PIN` SET | Role in `wValue`, GPIO in a 1-byte payload |
| `0xC3` with a bad role | `CLOCK.I2S_BCK_PIN` GET | `VC_ERR_ADDRESS` (it was a zero byte) |
| `0x88` | `CLOCK.I2S_CLOCK_MODE` SET | A value above 1 is `VC_ERR_VALUE` (it was ignored) |
| `0xC4` | `CLOCK.MCK_ENABLE` SET | 1-byte payload, non-zero enables |
| `0xC6` | `CLOCK.MCK_PIN` SET | 1-byte payload. Still refused while MCK runs, even for the current pin (`VC_ERR_STATE`, old `0x04`) |
| `0xC8` | `CLOCK.MCK_MULTIPLIER` SET | 1-byte payload. A value above 1 is `VC_ERR_VALUE` (old `0x01`) |
| `0xFE` | `CLOCK.I2S_CLOCK_PIN_MODE` SET | 1-byte payload. A value above 1 is `VC_ERR_VALUE` (old `0x05`) |

### 8.8 XPORT

| Old | New | Difference |
|---|---|---|
| `0xF5`, `0xF7` | `XPORT.UART_CONFIG`, `XPORT.I2C_CONFIG` SET | The scalar fields and the GPIO range are checked before anything is queued. A bad enable or notify flag, a baud rate outside 9600 to 1000000 or an I2C address outside `0x08` to `0x77` is `VC_ERR_VALUE`, and a GPIO that is never usable is `VC_ERR_PIN_INVALID`. The live interface is then left untouched and `XPORT.STATUS` is not updated. 1.1.6 queued everything and reported later. Pin pairing and ownership checks are still deferred and still report through `XPORT.STATUS` |

Both SETs are USB only, as in 1.1.6. Over UART or I2C they answer `VC_ERR_BLOCKED` (`0x16`), where 1.1.6 answered status `0x03`.

### 8.9 ROUTE

| Old | New | Difference |
|---|---|---|
| `0x70`, `0x71` | `ROUTE.MATRIX_ROUTE` | Address `(input << 8) \| output` in `wValue` for both directions. Payload is u8 enabled, u8 phase invert, u16 reserved, f32 gain dB. The GET no longer echoes input and output (6.5) |
| `0x72` | `ROUTE.OUTPUT_ENABLE` SET | An enable that collides with the core 1 mode is `VC_ERR_STATE` (it was ignored silently) |
| `0x7B` | `ROUTE.OUTPUT_ENABLE_CONFLICT` | An output past the end is `VC_ERR_ADDRESS` (it was a zero byte) |
| `0x9B` | `ROUTE.CHANNEL_NAME` SET | 1 to 32 bytes. 1.1.6 accepted up to 64. Both keep at most 31 |
| `0x44`, `0x45` | None | Retired. Use `ROUTE.PREAMP` per input channel (6.7) |
| `0xD6` | `ROUTE.MASTER_VOLUME_SAVE` | Zero-length SET, no status byte |

A `ROUTE.CHANNEL_DELAY` write to an output channel does not change what `ROUTE.OUTPUT_DELAY` reads back for that output, while a `ROUTE.OUTPUT_DELAY` write updates both. This is unchanged from 1.1.6 (`SET_DELAY` and `SET_OUTPUT_DELAY`). Set output delays through `ROUTE.OUTPUT_DELAY`.

### 8.10 FILTER

| Old | New | Difference |
|---|---|---|
| `0x42` | `FILTER.BAND` SET | Address `(channel << 8) \| band` in `wValue`. Payload u8 type, u8 bypass, u16 qp_x512, f32 freq, f32 Q, f32 gain dB, 16 bytes. There is no 18-byte form and no "keep the stored Qp" case, so always send the Qp. A crossover band ignores it |
| `0x43` | `FILTER.BAND` GET | Returns the whole band in the SET layout. The 3-bit parameter selector is gone. Bypass reads as exactly 0 or 1 |
| `0x42`, `0xD8` | `FILTER.BAND`, `FILTER.BAND_BYPASS` SET | While an earlier band write still waits for the main loop, a new one is `VC_ERR_BUSY`. 1.1.6 overwrote the single pending slot and lost the earlier write. Retry after a short delay |
| `0x42`, `0x43`, `0xD8`, `0xD9` | Same | A band in the reserved gap, or a crossover band on an input channel, is `VC_ERR_ADDRESS` |

### 8.11 DYN

| Old | New | Difference |
|---|---|---|
| `0x81` index 0 to 3 | `DYN.LIMITER_ENABLE`, `DYN.LIMITER_THRESHOLD`, `DYN.LIMITER_RELEASE`, `DYN.LIMITER_LINK_GROUP` | One command per parameter, output in `wValue`, all four still float32. Output `0xFF` means all outputs, on a SET only. A bad output is `VC_ERR_ADDRESS` (it was ignored) |
| `0x81` index `0x80`, `0x81` | `DYN.LIMITER_METERS`, `DYN.LIMITER_STATUS` | `wValue` must be 0 (the output byte was ignored) |
| `0xB8` | `DYN.LEVELLER_SPEED` SET | A value of 3 or more is `VC_ERR_VALUE` (it was ignored) |

### 8.12 FX

| Old | New | Difference |
|---|---|---|
| `0x60` | `FX.CROSSFEED_PRESET` SET | A value above 3 is `VC_ERR_VALUE` (it was ignored) |
| `0x3E`, `0x3F` | `FX.TUBE_*` | One command per parameter, in `TUBE_PARAM_*` order, all float32 (6.4). There is no index, so no bad index |
| `0x4C`, `0x4D` | `FX.UPMIX_*` | One command per parameter, in `UPMIX_PARAM_*` order, all float32 (6.4) |
| `0x4A` to `0x4E` on RP2040 | None | The commands do not exist on RP2040. 1.1.6 answered the GETs with zeros and STALLed the SETs |

### 8.13 MEAS

| Old | New | Difference |
|---|---|---|
| `0x50` selector 9 | `MEAS.LEVELS` | Identical bytes |
| `0x83` | `MEAS.CLIP_FLAGS`, `MEAS.CLIP_CLEAR` | Read and clear are two requests. A clip that lands between them is lost |
| `0x0A` | `MEAS.RTA_CAPS`, `MEAS.RTA_BAND_CENTRES` | Old `wValue` n (from 1) is chunk n - 1. A chunk past the table is `VC_ERR_ADDRESS` |
| `0x0B` | `MEAS.RTA_BANDS` | A bad channel is `VC_ERR_ADDRESS`. The read still counts as a keepalive first, as before |
| `0x0C` | `MEAS.RTA_BINS` | At most 256 bytes per request (1.1.6 sent up to 529 in one). Read in pieces by offset. Before the first transform the device returns a frame with a zero header and a `0xFF` final byte, so check the final byte. An offset at or past the frame length is `VC_ERR_ADDRESS` |
| `0x0E` | `MEAS.RTA_START`, `MEAS.RTA_STOP`, `MEAS.RTA_RESET` | Zero-length SETs (old actions 1, 0 and 2) |
| `0x0F` | `MEAS.RTA_BANDS_ALL` | Over UART and I2C it is `VC_ERR_BLOCKED`. A held bulk buffer is `VC_ERR_BUSY`. No live channel answers zero bytes, which is not an error |
| `0xA6` | `MEAS.SIGGEN_START`, `MEAS.SIGGEN_STOP`, `MEAS.SIGGEN_STOP_NOW` | Zero-length SETs (old actions 1, 0 and 2). Start with no valid configuration is `VC_ERR_STATE` |
| `0xA8` | `MEAS.SIGGEN_CAPS`, `MEAS.SIGGEN_TYPE_DESC` | Old `wValue` `0xFFFF` is `MEAS.SIGGEN_CAPS`. A bad type index is `VC_ERR_ADDRESS` |
| `0x08`, `0xA4` | `MEAS.RTA_CONFIG`, `MEAS.SIGGEN_CONFIG` SET | A rejected configuration STALLed in 1.1.6 and still does, now with `VC_ERR_VALUE`. `MEAS.SIGGEN_CONFIG` takes exactly 36 bytes and `MEAS.RTA_CONFIG` exactly 12 |
| `0xB2`, `0xB3` | None | Retired. The counters were always zero |

### 8.14 CS

The preview, Save and Revert model of Control Surfaces is unchanged. Preview SETs still only stage a record, and the apply result still arrives in `CS.STATUS`.

| Old | New | Difference |
|---|---|---|
| `0x84`, `0x8B`, `0x8D`, `0x20`, `0x22`, `0x24`, `0x27`, `0x29` | `CS.BINDING`, `CS.SLOT_NAME`, `CS.IR_CMD`, `CS.GROUP`, `CS.MACRO_HEADER`, `CS.MACRO_STEP`, `CS.DISPLAY_CONFIG`, `CS.DISPLAY_PAGE` SET | A bad index is `VC_ERR_ADDRESS` and a wrong length is `VC_ERR_LENGTH`. Neither touches the status packet any more. 1.1.6 ACKed them and posted INVALID_SLOT or INVALID_VALUE. A SET while one of the same kind is still pending is `VC_ERR_BUSY` and still posts BUSY (`0x1B`) |
| `0x8B` | `CS.SLOT_NAME` SET | 1 to 32 bytes. 1.1.6 accepted up to 64. A single NUL still clears the name |
| `0x04`, `0x06` | `CS.AUX_STATE`, `CS.AUX_LEVEL` SET | A slot that is not an aux output is `VC_ERR_STATE` and posts INVALID_AUX. A bad slot is `VC_ERR_ADDRESS`. During a revert it is `VC_ERR_BUSY` (it was ignored) |
| `0x05`, `0x07` | `CS.AUX_STATE`, `CS.AUX_LEVEL` GET | A slot that is not an active aux output is `VC_ERR_STATE` (it was a STALL) |
| `0x05` `wValue` `0xFFFF` | `CS.AUX_ALL` | Its own command |
| `0x9D`, `0x9E` | `CS.SAVE`, `CS.REVERT` | Zero-length SETs. They were IN transfers answering `0x01`. Busy is `VC_ERR_BUSY`, with the same BUSY status in `CS.STATUS` |
| `0x8F` | `CS.IR_LEARN_ARM`, `CS.IR_LEARN_CANCEL`, `CS.IR_LEARN_STATUS` | Old `wValue` 1, 0 and 2. Arm with no receiver is `VC_ERR_STATE` and posts NO_IR. During a revert it is `VC_ERR_BUSY` |
| `0x25` | `CS.MACRO_FIRE`, `CS.MACRO_CANCEL` | Old `wValue` `0xFFFF` is `CS.MACRO_CANCEL`. A bad macro is `VC_ERR_ADDRESS`. During a revert fire is `VC_ERR_BUSY` |
| `0x22`, `0x24` | `CS.MACRO_HEADER`, `CS.MACRO_STEP` GET | New. Each returns exactly what its SET accepts |
| `0x28` | `CS.DISPLAY_CONFIG` GET, `CS.DISPLAY_CAPS` | The old 16-byte reply is split into 4 limit bytes in `CS.DISPLAY_CAPS` and 12 configuration bytes in `CS.DISPLAY_CONFIG` |
| `0x86` | `CS.CAPS`, `CS.NOUN_DESC` | Old `wValue` `0xFFFF` is `CS.CAPS`. A noun number is `CS.NOUN_DESC` |
| `0x87`, `0x26`, `0x2B` | `CS.STATUS`, `CS.EXT_STATUS`, `CS.DISPLAY_STATUS` | `wValue` must be 0. 1.1.6 ignored it |

### 8.15 DIAG

| Old | New | Difference |
|---|---|---|
| `0x82` | `DIAG.CLOCK_DIAG` | Exists only in `DSPI_CLOCK_DIAG` builds. Elsewhere it is `VC_ERR_UNKNOWN` (1.1.6 answered zero bytes). USB only, so UART and I2C get `VC_ERR_BLOCKED` |
| `0x50` selectors 24, 25 | `DIAG.LOOPBACK_STATS` | Exists only in `DSPI_LOOPBACK` builds. 1.1.6 answered the selectors with 0 elsewhere |
| `0x50` selectors 64 and up | `DIAG.LOOP_PROF_*` | Exist only in `DSPI_LOOP_PROF` builds, as before |

---

## 9. Bulk parameters

### 9.1 The blob is unchanged

`WireBulkParams` is format version 32 and 6136 bytes on both platforms, exactly as in 1.1.6. Every offset, every section and the 16-byte header are the same, and so are the `PARAM_CHANGED` offsets that refer to it. The layout is in `Documentation/commands.md` sections 19.3 to 19.5. A host's bulk parser, its mirror of the device state and its notification dispatch need no change.

### 9.2 `STORE.ALL_PARAMS`

`STORE.ALL_PARAMS` (page `0x02`, command `0x40`, ID `0x0240`) replaces both `GET_ALL_PARAMS` (`0xA0`) and `SET_ALL_PARAMS` (`0xA1`).

- **GET.** `wValue` 0. The device collects the live state when the request arrives and returns `min(wLength, 6136)` bytes. Over USB the reply goes out in 64-byte packets, so the host USB layer must accept a multi-packet control read, as before.
- **SET.** `wValue` 0 and exactly 6136 bytes. The device checks the request in two steps.

| When | Check | Result |
|---|---|---|
| SETUP | Length is not 6136 | `VC_ERR_LENGTH`. On UART and I2C a payload the transport cannot buffer is status `0x06` |
| SETUP | Another transport holds the bulk buffer, or an earlier bulk apply is pending | `VC_ERR_BUSY`. UART and I2C answer status `0x04` |
| After the data | `format_version` is not 32, `payload_length` is not 6136, or `platform_id` or a channel count differs from the device | `VC_ERR_VALUE`, nothing applied |

The main loop then applies the blob under a muted, synchronized restart of every output and sends one `BULK_INVALIDATED` notification with source 2. As in 1.1.6, the bulk SET writes no flash, does not validate floats, and applies the physical IO configuration and the limiter only when `STORE.OUTPUT_CONFIG_MODE` is 1. There is no version tolerance. Send the version the device reports in the header of its own GET. Confirm the apply by reading `STORE.ALL_PARAMS` again after the busy window.

A bridge or transport that recognised the bulk SET by `bRequest` `0xA1` must now recognise page `0x02`, command `0x40` instead, and its buffer must still hold all 6136 bytes. Keyed on `0xA1`, it never recognises the 1.1.7 bulk SET and handles a 6136-byte payload as an ordinary request.

What is new is the strictness. 1.1.6 ACKed and ignored a SET of 64 bytes or fewer, and ACKed a blob with a bad header and then dropped it silently. 1.1.7 refuses both and says why. A non-zero `wValue` is `VC_ERR_ADDRESS`.

### 9.3 `STORE.ALL_PARAMS_CHUNK`

WinUSB limits a control transfer to 4096 bytes, so USB hosts on Windows move the blob in pieces with `STORE.ALL_PARAMS_CHUNK` (page `0x02`, command `0x41`, ID `0x0241`). It replaces `GET_ALL_PARAMS_CHUNK` and `SET_ALL_PARAMS_CHUNK` (`0xA2`, `0xA3`) with the same semantics. `wValue` is the byte offset and `wLength` the chunk size, from 1 to the rest of the blob. It is USB only.

- **Reading.** A GET at offset 0 takes the bulk buffer and snapshots the whole state. Later GETs read that snapshot at any offset. A GET with `wLength` 0 is `VC_ERR_LENGTH`. The session ends when a chunk that reaches the end of the blob completes, or when any other USB vendor request arrives.
- **Writing.** Chunks must arrive in order. Offset 0 opens the session and takes the bulk buffer. Each later chunk must start where the previous one ended. When 6136 bytes have arrived, the header is checked as for `STORE.ALL_PARAMS` and the apply is queued. Any other USB vendor request closes the session and discards what was received.

| Refusal | Result |
|---|---|
| Offset at or past 6136 | `VC_ERR_ADDRESS` |
| A SET chunk that runs past the end, or a zero-length chunk | `VC_ERR_LENGTH` |
| A chunk with no open session, or a SET chunk at the wrong offset | `VC_ERR_STATE`. Restart at offset 0. A SET refused for its offset keeps the session, so the right offset can be resent |
| The bulk buffer is held elsewhere when offset 0 arrives | `VC_ERR_BUSY` |
| The final SET chunk completes a blob with a bad header | `VC_ERR_VALUE`, nothing applied |

In 1.1.6 these failures were STALLs with no reason. Now each carries a result code.

### 9.4 UART and I2C

Over UART and I2C, `STORE.ALL_PARAMS` streams through the same bulk buffer under the same lock, in one GET response or one SET request, exactly as in 1.1.6. `STORE.ALL_PARAMS_CHUNK` is refused there with `VC_ERR_BLOCKED`. Over I2C a 6136-byte reply is read out in pieces, and a parked reply that is not read for 1 s is dropped. See `Documentation/Features/control_interfaces_spec.md` sections 3.7 and 4.7 for wire times.

---

## 10. Notifications

Notifications did not change.

- They arrive on bulk IN endpoint `0x83` of interface 2, in packets of up to 64 bytes. With nothing queued, a 1-byte idle packet `00` goes out after 100 ms of quiet.
- Every packet except the idle packet and the legacy master volume packet starts `[0x02, event, flags, seq]`. The events, their numbers and their layouts are the same as in 1.1.6 (`Documentation/commands.md` section 21).
- `PARAM_CHANGED` names the changed field by its byte offset in `WireBulkParams`, which is unchanged (9.1). Notifications never name a command, so a host that dispatches on the offset needs no change. A SET through a 1.1.7 command produces the same notifications as the 1.1.6 opcode it replaces.
- The source byte still says where a change came from. It is 1 for a USB vendor SET, 2 for a bulk SET, 3 for a preset load, 4 for a factory reset, 5 for Control Surfaces, 6 for the firmware itself, 7 for the host's UAC1 volume or mute, 8 for the UART transport and 9 for the I2C transport.
- On `BULK_INVALIDATED` or `PRESET_LOADED`, or on a gap in `seq`, re-read `STORE.ALL_PARAMS`, as before.
- The same v2 packets are delivered over UART as type `0x40` frames when `XPORT.UART_CONFIG` enables them (11.5). The legacy master volume packet is USB only, and I2C has no notification channel.

---

## 11. Per-transport notes

### 11.1 USB on macOS (IOKit)

Send vendor requests on the default control pipe exactly as before, through the device interface or the interface object. Only `bRequest` and `wIndex` change. The page is simply the `wIndex` high byte, so `wIndex = (page << 8) | 2`.

The 1.1.6 Console sent `wIndex` 0 on some calls and 2 on others. Both had a zero high byte, so both land on page 0 on 1.1.7. Both forms become `(page << 8) | 2`. Keep the low byte at 2 everywhere so the request names the vendor interface.

The `wIndex` page check (`tools/windex_page_test.py`) passes on macOS with firmware 1.1.7 beta 1, for both interface-recipient and device-recipient requests, with every field exact.

### 11.2 USB on Windows (WinUSB)

- **Binding.** The Microsoft OS 2.0 descriptor still binds WinUSB to interface 2 with no driver install. Its request (IN, `bRequest` `0x01`, `wIndex` 7) is not part of the protocol and is answered as before.
- **The page byte.** For an interface-recipient request, WinUSB overwrites the **low** byte of `wIndex` with the interface number of the handle you send on, and leaves the high byte alone. The page rides in the high byte, so it should reach the device unchanged. Send `wIndex = (page << 8) | 2` on the interface 2 handle.
- **Transfer size.** WinUSB limits a control transfer to 4096 bytes. Use `STORE.ALL_PARAMS_CHUNK` (9.3) for the 6136-byte blob, with chunks of at most 4096 bytes. Every other reply is at most 256 bytes, except `MEAS.RTA_BANDS_ALL` at 82 bytes per live channel, which stays well under 4096.
- **Confirming the page byte on your setup.** The repository does not yet record a Windows run of the page check. `tools/windex_page_test.py` sends harmless refused requests to several pages and reads `SYS.LAST_RESULT` after each one to confirm the device saw the page byte intact, for interface and device recipients. It needs Python and a DSPi on 1.1.7. Run it once on your Windows setup before relying on the page byte, following `Documentation/Features/windex_page_test.md`. If it fails for interface recipients and passes for device recipients, send device-recipient requests (`0xC0` and `0x40`), which the firmware also accepts.

### 11.3 USB on Linux (libusb, usbfs)

- Claim interface 2 with `libusb_claim_interface` before sending requests. For an interface-recipient request, usbfs uses the `wIndex` **low** byte to find the interface and requires that the calling process has claimed it. A low byte of 0 names the audio control interface, which the kernel's audio driver owns, so the request fails. Always send a low byte of 2.
- usbfs uses only the low byte for that check, so the page in the high byte passes through unchanged.
- A STALL shows up as `LIBUSB_ERROR_PIPE`. A timeout during the busy window shows up as `LIBUSB_ERROR_TIMEOUT` (5.8).

### 11.4 WebUSB

Claim interface 2 (vendor class `0xFF`) and send requests with recipient `"interface"`.

```
await device.claimInterface(2);
// GET SYS.PROTOCOL_INFO
const r = await device.controlTransferIn(
  { requestType: "vendor", recipient: "interface", request: 0x10, value: 0, index: (0x01 << 8) | 2 }, 16);
// SET ROUTE.OUTPUT_GAIN, output 1, -6.0 dB
await device.controlTransferOut(
  { requestType: "vendor", recipient: "interface", request: 0x21, value: 1, index: (0x05 << 8) | 2 },
  new Float32Array([-6.0]));
```

The browser checks the low 8 bits of `index` against the claimed interfaces, so the low byte must be 2. A refusal comes back as a transfer result with status `"stall"`. Command numbers `0x00` to `0x0F` are reserved in every page. `0x01` is the Microsoft OS 2.0 vendor code and `0x02` is set aside for a future WebUSB landing-page descriptor, so neither will ever be a DSPi command. Notifications are bulk transfers on endpoint 3 (`transferIn(3, 64)`).

### 11.5 UART

The UART frame format, CRC, baud rates and timing are unchanged. Every frame starts with the sync byte `0xA5` and a type byte. All multi-byte fields are little-endian.

```
SET request    A5 01 bReq wValL wValH wIdxL wIdxH wLenL wLenH payload[wLen] crcL crcH
GET request    A5 02 bReq wValL wValH wIdxL wIdxH wLenL wLenH               crcL crcH
SET response   A5 81 status 00 00                                            crcL crcH
GET response   A5 82 status lenL lenH payload[len]                           crcL crcH
Notification   A5 40 00 lenL lenH packet[len]                                crcL crcH
```

The CRC is CRC16-CCITT-FALSE. The polynomial is `0x1021`, the initial value `0xFFFF`, with no reflection and no final XOR. It covers every byte after the sync byte, from the type byte to the last payload byte, and goes on the wire low byte first. The CRC of the ASCII string `123456789` is `0x29B1`. A GET response carries a payload only when the status is `0x00`. Otherwise its length field is 0.

What changes is `wIdxH`, which is now the page, and the meaning of the status byte.

| Status | Meaning | Retry? |
|---|---|---|
| `0x00` | Success. A GET response carries `len` payload bytes | |
| `0x01` | Busy. A USB transfer held the shared buffers. The command did not run | Yes |
| `0x02`, `0x03` | Never sent by 1.1.7. `0x02` in answer to the probe means firmware 1.1.6 or older | No |
| `0x04` | Bulk buffer held by another transport or a pending bulk apply | Yes |
| `0x05` | CRC error. The frame was not dispatched | Resend the frame |
| `0x06` | Payload larger than the transport can hold (64 bytes, or the bulk buffer for `STORE.ALL_PARAMS`) | No |
| `0x07` | Bad frame (I2C only) | No |
| `0x10` and up | The command's own result code (5.3), the same code a USB host reads from `SYS.LAST_RESULT` | Only `0x17` |

So any non-zero status is a failure, and only `0x01`, `0x04` and `0x17` are worth retrying unchanged. A client that decodes the status byte as a closed set of `0x00` to `0x07` must accept `0x10` and up as well, or it reports every refusal as a corrupt or unknown frame. In 1.1.6 a handler failure came back as `0x02` and a USB-only command as `0x03`. In 1.1.7 those cases come back as the specific result code, and a USB-only command is `0x16` (`VC_ERR_BLOCKED`). The device itself retries `0x01`, `0x04` and `0x17` for 50 ms before it answers with them. The exception is a `STORE.ALL_PARAMS` SET that finds the bulk buffer held, which is answered `0x04` as soon as its frame ends.

**Frames, byte by byte.** Each was computed with the CRC above, and its layout follows the firmware's frame parser. They use `wIndex` low byte `0x00`.

The probe, as the first frame after the link comes up. A 1.1.7 RP2350 release build answers with the 16-byte `SYS.PROTOCOL_INFO`.

```
Host -> device   A5 02 10 00 00 00 01 10 00 A0 BD
                 GET, bRequest 0x10, wValue 0, wIndex 0x0100 (page 0x01), wLength 16, CRC 0xBDA0
Device -> host   A5 82 00 10 00 44 53 50 69 01 00 0C 00 40 00 00 01 00 00 00 00 C5 E2
                 GET response, status 0x00, len 16, "DSPi" 1.0, 12 pages, 64, 256, CRC 0xE2C5
Old firmware     A5 82 02 00 00 F0 DA
                 status 0x02: firmware 1.1.6 or older. Send it nothing paged
```

A SET with a payload. Set the gain of output 1 to -6.0 dB with `ROUTE.OUTPUT_GAIN` (page `0x05`, command `0x21`).

```
Host -> device   A5 01 21 01 00 00 05 04 00 00 00 C0 C0 E4 2A
                 SET, bRequest 0x21, wValue 1, wIndex 0x0500, wLength 4, payload -6.0f, CRC 0x2AE4
Device -> host   A5 81 00 00 00 4C 2F
                 SET response, status 0x00
```

A GET. Read it back.

```
Host -> device   A5 02 21 01 00 00 05 04 00 9B 0A
Device -> host   A5 82 00 04 00 00 00 C0 C0 A0 8A
                 status 0x00, len 4, -6.0f
```

A zero-length action. `MEAS.RTA_RESET` (page `0x09`, command `0x2A`).

```
Host -> device   A5 01 2A 00 00 00 09 00 00 C4 75
Device -> host   A5 81 00 00 00 4C 2F
```

A refused request. Read the gain of output 9, which does not exist.

```
Host -> device   A5 02 21 09 00 00 05 04 00 D9 07
Device -> host   A5 82 13 00 00 A3 AE
                 status 0x13 VC_ERR_ADDRESS, len 0
```

A 1.1.6-era client that leaves the `wIndex` high byte at 0 gets status `0x10` for everything. For the probe with page 0 the answer is `A5 82 10 00 00 F3 F7`. A USB-only SET such as `XPORT.UART_CONFIG` sent over the UART is answered `A5 81 16 00 00 8F DE`.

**Sizes.** SET payloads are at most 64 bytes. GET replies are at most 256 bytes and arrive whole, except `STORE.ALL_PARAMS`, which streams 6136 bytes. 1.1.6 limited UART and I2C replies to 132 bytes, so a receive buffer sized for 132 must grow to 256. The largest fixed-length reply is still `CS.MACRO` at 132 bytes. `MEAS.RTA_BINS` returns up to 256 bytes per read. `wLength` caps the reply as on USB, and `wLength` 0 means "the whole reply".

**USB-only commands** answer `VC_ERR_BLOCKED` (`0x16`) over UART. They are the `XPORT.UART_CONFIG` and `XPORT.I2C_CONFIG` SETs, both directions of `STORE.ALL_PARAMS_CHUNK`, `MEAS.RTA_BANDS_ALL` and `DIAG.CLOCK_DIAG`. `SYS.COMMAND_INFO` flag bits 0 and 1 mark them.

**Start-up order for a UART client.**

1. Bring the link up and drain any stray bytes.
2. Send the probe and wait a few hundred milliseconds for the response. Retry on a timeout. A device in a flash write can lose bytes.
3. On status `0x00` with `DSPi`, continue. On status `0x02` the firmware is 1.1.6 or older. Show "update the DSPi firmware" and send nothing else, except its own `GET_PLATFORM` if you want the version (the GET frame `A5 02 7F 00 00 00 00 07 00 87 29`). On any other answer, treat the link as faulty and retry later.
4. Read `SYS.PLATFORM` and, if you assert counts, the discovery commands (4.7).
5. Start normal polling.

**Notifications** keep their frame. As an illustration, a `PARAM_CHANGED` for the gain of output 1 (bulk offset 724, 4 bytes, value -6.0, source 8 for the UART, sequence 5) arrives as `A5 40 00 10 00 02 02 00 05 D4 02 04 00 08 00 00 00 00 00 C0 C0 75 65`. Ignore a stray `0x40` frame if you did not enable notifications.

`SYS.ENTER_BOOTLOADER` over UART answers `A5 81 00 00 00 4C 2F` and the device reboots once the last byte has left. The full transport specification is `Documentation/Features/control_interfaces_spec.md`.

### 11.6 I2C

The I2C framing is unchanged. A request is one I2C write of an 8-byte header and the SET payload, if any. The header is the UART frame body from the type byte to `wLenH`, with no sync byte and no CRC.

```
Request (write)   [type][bReq][wValL][wValH][wIdxL][wIdxH][wLenL][wLenH][payload...]
                  type 0x01 SET, 0x02 GET
Response (read)   [status][lenL][lenH][payload...] then 0xFF padding
```

The status byte has exactly the UART meanings above. A read that arrives before the device has dispatched the request returns the BUSY frame `01 00 00`, so read again a moment later. A parked response survives across reads until it has been read completely, and a new request write discards any unread response.

The same requests as on UART.

```
GET SYS.PROTOCOL_INFO
  write  02 10 00 00 00 01 10 00
  read   00 10 00 44 53 50 69 01 00 0C 00 40 00 00 01 00 00 00 00 FF FF ...

SET ROUTE.OUTPUT_GAIN, output 1, -6.0 dB
  write  01 21 01 00 00 05 04 00 00 00 C0 C0
  read   00 00 00 FF FF ...

GET ROUTE.OUTPUT_GAIN, output 1
  write  02 21 01 00 00 05 04 00
  read   00 04 00 00 00 C0 C0 FF FF ...

SET MEAS.RTA_RESET (zero-length)
  write  01 2A 00 00 00 09 00 00
  read   00 00 00 FF FF ...

GET ROUTE.OUTPUT_GAIN, output 9 (refused)
  write  02 21 09 00 00 05 04 00
  read   13 00 00 FF FF ...
```

On old firmware the probe reads back status `0x02`. I2C has no notification channel, so an I2C client polls. `SYS.ENTER_BOOTLOADER` parks its OK response and reboots once the controller has read it, or after 1 s anyway.

---

## 12. Porting checklist

Work through these in order. Each item names the section that explains it.

1. [ ] Add the probe at connect, before any other request, with the old-firmware branch you need (4.4 or 4.5).
2. [ ] Build every request from a page and a command number. Put the page in the `wIndex` high byte and 2 in the low byte (3.1). On UART and I2C, page in `wIdxH`.
3. [ ] Replace every opcode constant with its new command, using section 6.2. Prefer generating the constants from `Documentation/protocol/dspi_commands.json`. If you widen the constants to 16-bit IDs, widen every helper parameter, struct field and table column that carries them too. One left at 8 bits silently drops the page, so the request lands on page 0 and is refused. Even the probe then STALLs on 1.1.7 and looks like old firmware. On 1.1.6 firmware the request runs the old opcode with that number (4.1).
4. [ ] For every command that addresses nothing, send `wValue` 0. For every addressed command, send the whole 16-bit address with no stray high byte (5.5).
5. [ ] Convert the 34 write-as-reads into SETs with the payloads from section 6.6, and drop the code that parsed their status bytes.
6. [ ] Replace the split commands of 6.3 and 6.4. They are `GET_STATUS`, `LIMITER`, the tube and upmixer parameters, `RTA_GET_CAPS`, `GET_CS_CAPS`, `SIGGEN_GET_CAPS`, `GET_CS_AUX_STATE` with `0xFFFF`, `OUTPUT_LATENCY`, `RTA_CONTROL`, `SIGGEN_CONTROL`, `CS_IR_LEARN` and `CS_MACRO_FIRE`.
7. [ ] Port the two reshaped payloads of 6.5, `FILTER.BAND` with its read-modify-write rules and `ROUTE.MATRIX_ROUTE`. Port the combined replies `CLEAR_CLIPS`, `GET_INPUT_RATE`, `PRESET_GET_STARTUP` and `GET_CS_DISPLAY_CFG`.
8. [ ] Remove the retired requests (6.7) and put in their replacements.
9. [ ] Send exact SET payload lengths (6.2). Stop sending padded or oversized buffers.
10. [ ] Handle a STALL on every request as a refusal, read `SYS.LAST_RESULT` where you need the reason, and show or log it (5.1, 5.2). On UART and I2C, handle every status code, including `0x10` and up (11.5).
11. [ ] Retry `VC_ERR_BUSY`, and on UART and I2C status `0x01` and `0x04`, after a short wait. Retry nothing else unchanged. Keep the timeout and back-off around flash-writing commands (5.8).
12. [ ] Replace firmware-version feature gates and probe-by-STALL with `SYS.PAGE_LIST`, `SYS.PAGE_COMMANDS` and `SYS.COMMAND_INFO` (4.7).
13. [ ] Re-derive every reply length and every range or count you assert from section 7 (7.1, 7.4). Re-check the changed replies (7.2, 7.3). Size reply buffers for 256 bytes, and gate RP2350-only commands on discovery (4.8).
14. [ ] Re-key any table, policy, router, cache, log, bulk detection or analyser arbitration keyed on `bRequest` alone onto `(page, command)` or the 16-bit ID (1, 9.2).
15. [ ] On Windows, move the bulk blob with `STORE.ALL_PARAMS_CHUNK` in chunks of at most 4096 bytes, and run the page check once (11.2).
16. [ ] Leave the bulk blob parser, the notification handler and the UART and I2C framing as they are (9, 10, 11.5, 11.6).

### 12.1 Test your port

Test against a real device running firmware 1.1.7, on both platforms if your host supports both.

- **Probe both generations.** Confirm your host recognises 1.1.7, and that on a 1.1.6 device it sends nothing but the probe, the old `GET_PLATFORM` and, on request, the old `ENTER_BOOTLOADER`.
- **Read every GET you use** and compare its length with section 7.1. A strict-length client should run its own length checks against the device once, with every constant taken from the `GET_LENGTHS` table or the registry rather than from 1.1.6 code.
- **Write and read back** every setting you write, and check the value. For deferred commands, read back after the busy window.
- **Provoke the refusals** you handle. Try an address past the end (`VC_ERR_ADDRESS`), a wrong payload length (`VC_ERR_LENGTH`), a bad enum value (`VC_ERR_VALUE`) and, on UART or I2C, a USB-only command (`VC_ERR_BLOCKED`). Confirm your host reports each one and the device state does not change.
- **Exercise the busy paths.** Write two EQ bands back to back and confirm a `VC_ERR_BUSY` is retried. Run a preset save and confirm the following requests survive the busy window.
- **Watch notifications** during your writes and confirm the offsets you dispatch on still arrive.

Machine-readable inputs for generating code and checks:

- `Documentation/protocol/dspi_commands.json` is the registry. It holds every page, revision, command number, name, direction, reply length, SET payload bounds, address kind, flag, build condition and description. It is generated from the firmware's own command table.
- `Documentation/protocol/legacy_map.json` is the old-to-new map behind section 6. For every 1.1.6 opcode and direction it gives the new command, how the address and value are derived from the old `wValue`, the status family and its per-command overrides. Its `_schema` key explains every field. A bridge can generate its policy and routing tables from it.

Reference implementations in the firmware repository:

- `tools/dspi_test/device.py` sends paged requests by name or ID (`pget`, `pset`), probes before anything else, turns a STALL into an exception carrying the reason from `SYS.LAST_RESULT`, and handles the busy window.
- `tools/dspi_poke` is a small command-line tool that sends any command to a device. Use it to try a request by hand before you write the code for it.

---

## 13. Host appendices

Each appendix applies this guide to one host repository. It names the code to change, maps the requests that host sends, lists the checks that break and gives an order to port and test in. Read the one for the host you maintain.

- **13.1 ESP32 front panel (UART).** For the maintainer of the third-party ESP32 front panel. It is self-contained, so a maintainer who reads nothing else in this guide can port the panel from it. Every frame it shows is computed byte for byte.
- **13.2 DSPi Console for macOS.** For the Console maintainer. It covers the USB transport, the update path for devices still on 1.1.6 and the DSPi Link hub built into the Console.
- **13.3 DSPi Link.** For the maintainers of the Link spec, its TypeScript library and web app, and the ESP32 bridge. Console hubs follow the same spec and policy table, so read it together with 13.2.
- **13.4 DSPi Terminal.** For the maintainer of the Rust command-line tool and terminal UI.

DSPi Console on Windows has no appendix. It follows the WinUSB notes (11.2) and the porting checklist (12). Each appendix names the branch and commit it was written against. File paths, function names and line numbers in a host repository drift as that repository moves on, so search for the function when a reference no longer fits.

### 13.1 ESP32 front panel (UART)

**TL;DR.** The ESP32 front panel is a third-party controller that drives DSPi over the DSPi UART control link. Firmware 1.1.7 gave every command a two-part address. The page travels in the high byte of `wIndex` and the command number in `bRequest`. The panel still sends a page of 0 on every request, so firmware 1.1.7 refuses its very first request with status `0x10` and the panel shows "Communication fault" forever. The device itself is never changed. The port is small and local. One function builds every frame, about 60 of the 69 opcodes only need a new number, and a handful of requests need real care. Those are four status reads that became structure reads, the split input-rate read and four former write-as-reads that became zero-length SETs. Before any of that, the panel must send one probe frame, because firmware 1.1.6 would run unrelated old commands for some of the new numbers, a factory reset among them.

This appendix is self-contained. Everything a maintainer needs to port the panel is here, with every frame computed byte for byte. The rest of the guide gives background, for example section 5.8 on busy handling and section 11.5 on the UART transport in general.

#### 13.1.1 At a glance

| Item | Value |
|---|---|
| Host | DSPi ESP32 Front Panel, an Arduino C++ sketch for the Waveshare ESP32-S3-LCD-2. It has a rotary encoder, BLE remote learning, menus, level meters, and an SD card media player that feeds the DSPi I2S input |
| Repository | `github.com/CrawlingKingSn8ke/DSPi-ESP32-Front-Panel` (upstream, third party). The DSPi project's fork is `github.com/WeebLabs/DSPi-ESP32-Front-Panel` |
| Audited | Branch `spdif4-compat`, commit `de6573c`, panel version 1.1.2 |
| Protocol code | All of it is in `firmware/DSPi_ESP32_Front_Panel_v1_1_2/DSPi_ESP32_Front_Panel_v1_1_2.ino` (called the sketch below). No other file talks to DSPi |
| Transport | UART only, 115200 baud 8N1, ESP32 GPIO16 (RX) and GPIO17 (TX) |
| Requests | 69 opcodes. `GET_STATUS` is used with 4 selectors and `GET_INPUT_RATE` splits in two, so 13.1.6 has 73 rows |
| Not used | The bulk parameter blob, Control Surfaces, EQ, matrix, limiter, tube, upmixer, pin commands and the bootloader. The panel never enables notifications but handles them if the user did |

**What happens today.** An unported panel meets firmware 1.1.7 like this.

1. `setup()` opens the UART and sends nothing. The first `loop()` pass finds `dspi.connected` false and calls `syncCurrentState(true)`, and repeats that every 1000 ms.
2. The first request of every sync is `GET_PLATFORM`, `A5 02 7F 00 00 02 00 04 00 BC 91`. Its `wIndex` is `0x0002`, so the page (the high byte) is 0. Page 0 has no commands, and firmware 1.1.7 answers `A5 82 10 00 00 F3 F7`, which is status `0x10` (`VC_ERR_UNKNOWN`) with no payload.
3. `dspiResultForStatus` maps `0x10` to `DSPI_TXN_UNEXPECTED`, which is not retried. The serial log shows `DSPi request failed: req=0x7F reason=unexpected response`, and the sync stops at its first step.
4. `dspi.connected` never becomes true. The panel shows "Communication fault", "DSPi FW unavailable", "UART offline" and "NO DSPi", and every control that needs the DSPi is disabled.
5. The one path not gated on `dspi.connected` is System > Volume Limit. Its page 0 SETs are refused too, each with a response like `A5 81 10 00 00 2F 6C`. So the unported panel is harmless but completely dead.

**Size of the job.** About 60 opcodes are a mechanical renumber with unchanged payloads and replies. The rest is a short list of local changes.

- Add the probe and a protocol state that gates every other request (13.1.5).
- Decode the new status codes (13.1.9).
- Skip the two ADAT reads on RP2040 (13.1.10).
- Re-key the psychoacoustic bass capability probe (13.1.7).
- Replace `GET_STATUS` selectors 9, 10, 11 and 15 and the 8-byte `GET_INPUT_RATE` read (13.1.7).
- Turn the four write-as-reads into zero-length SETs (13.1.7).
- Read the 7-byte platform reply for the version display (13.1.5).

#### 13.1.2 Before you start

**Port from `spdif4-compat`, not from `main`.** Upstream `main` (`8938926`) and `review/v1.1.2` (`d8e8a64`) still carry two checks that firmware 1.1.5 already broke. In `syncCurrentState` they read `GET_SPDIF_INPUT_CONFIG` with an exact length of 5 and require byte 0 to be 3, and they reject input source 6. Firmware 1.1.5, 1.1.6 and 1.1.7 all report 4 S/PDIF inputs and sources 0 to 6. A port of `main` would therefore still show "Communication fault" after every change in this appendix. Branch `spdif4-compat` is one commit (`de6573c`) ahead of `main`, and that commit changes only the sketch. If you work from an upstream tree that lacks it, apply that commit first.

**What stays exactly as it is.**

- The wiring. ESP32 GPIO16 receives from DSPi GPIO16 (TX), and ESP32 GPIO17 sends to DSPi GPIO17 (RX), at 3.3 V logic.
- The link settings, 115200 baud 8N1.
- The frame format, the sync byte, the type bytes, the CRC and the one-request-in-flight rule (13.1.4). `dspiReadFrame` and `dspiCrc16Step` need no change.
- Notification frames (type `0x40`) and `acceptDspiNotification`. The packet layout is unchanged.
- The payload bytes of every SET, and the reply bytes of every GET except the ones 13.1.7 lists.
- The DSPi side. The UART control interface is enabled and pinned over USB, from DSPi Console, and the panel never configures it. The flash layout of the stored configuration did not change between 1.1.6 and 1.1.7, so a device updated from 1.1.6 keeps its UART settings.

No generated files and no tools are needed. If you prefer to generate the command constants, every command is listed in `Documentation/protocol/dspi_commands.json` in the DSPi repository.

#### 13.1.3 Transport and choke points

**TL;DR.** Every request leaves the panel through one function, so the page can be added in one place. The trap is the parameter type. The opcode travels as `uint8_t` through fourteen helpers, and one helper left at `uint8_t` silently drops the page.

`dspiTransactionResult` sends every request through `dspiSendRequest`, waits for the response, maps the status and retries. Only two wrappers call it, and they are the only places that choose `wIndex`. `dspiGetResult` and `dspiSet` both pass the constant 2. Everything else (`dspiGet`, `dspiGetByte`, `dspiGetFloat`, `dspiSetByte`, `dspiSetFloat`, `getExact`, `getExactByte`, `getExactFloat`) sits on those two.

**What must change there.**

1. Carry the page with the command. A command is `(page, cmd)`, or as one 16-bit ID, `(page << 8) | cmd`.
2. In `dspiSendRequest`, put `cmd` in the `bRequest` byte and the page in the high byte of `wIndex` (`wIdxH`). Send `0x00` in the low byte (`wIdxL`). Firmware 1.1.7 ignores the low byte, so keeping today's `0x02` also works, but every 1.1.7 frame in this appendix uses `0x00` and the CRC differs.
3. Drop the `index` argument from the two wrappers. The page comes from the command.
4. Print the 16-bit ID and the numeric status in every log line. Today the logs print `req=0x%02X`, and after the port the bare command number is ambiguous. For example `0x10` is `SYS.PROTOCOL_INFO`, `INPUT.SOURCE`, `FX.CROSSFEED_ENABLE`, `STORE.PRESET_SAVE` and `MEAS.LEVELS`.

A safe shape is a small struct, because C++ never converts a struct to `uint8_t` by accident.

```cpp
struct DspiCmd { uint8_t page; uint8_t cmd; };
static constexpr DspiCmd CMD_ROUTE_USER_VOLUME = {0x05, 0x60};

// in dspiSendRequest(bool isGet, DspiCmd c, uint16_t value, ...)
header[1] = c.cmd;      // bRequest
header[4] = 0x00;       // wIdxL, ignored by 1.1.7
header[5] = c.page;     // wIdxH, the page
```

**The 8-bit narrowing trap.** If you keep integers instead, and widen the constants to `uint16_t` IDs, every one of these signatures must be widened in the same change. Each takes the opcode as `uint8_t` today.

- `void dspiSendRequest(bool isGet, uint8_t request, uint16_t value, uint16_t index, const uint8_t *payload, uint16_t length)`
- `DspiTxnResult dspiTransactionResult(bool isGet, uint8_t request, uint16_t value, uint16_t index, ...)`, both the prototype near the protocol constants and the definition
- `DspiTxnResult dspiGetResult(uint8_t request, uint16_t value, uint16_t length, ...)`, both the prototype and the definition
- `bool dspiGet(uint8_t request, ...)` and `bool dspiSet(uint8_t request, ...)`
- `bool dspiGetByte(uint8_t request, uint8_t &value, uint16_t wValue)`, `bool dspiGetFloat(uint8_t request, float &value)`, `bool dspiSetByte(uint8_t request, uint8_t value)` and `bool dspiSetFloat(uint8_t request, float value)`
- `bool getExact(uint8_t request, ...)`, `bool getExactByte(uint8_t request, ...)` and `bool getExactFloat(uint8_t request, ...)`
- `bool setPsybassFloatVerified(uint8_t setRequest, uint8_t getRequest, ...)`, which takes two opcodes. In 1.1.7 the SET and the GET are one command, so it can take one
- `void dspiRecoverAfterTimeout(uint8_t request)`, which only logs
- The 69 constants themselves, `static const uint8_t REQ_* = 0x..;`

Miss one, and C++ silently keeps the low byte of the ID. The page is lost, the request lands on page 0, and the device answers status `0x10` to it. Arduino ESP32 builds do not warn about this narrowing by default.

**Receive buffer.** `UartFrame.payload` is 64 bytes. `dspiReadFrame` consumes a longer frame for its CRC and then rejects it as `DSPI_FRAME_OVERSIZE`, which `dspiTransactionResult` turns into a non-retried `DSPI_TXN_LOCAL_FRAME`. Firmware 1.1.7 GET replies can be up to 256 bytes, where 1.1.6 limited UART replies to 132 bytes. The device never sends more than the request's `wLength`, and `wLength` 0 means "the whole reply". The largest reply this panel asks for after the port is 41 bytes (`MEAS.LEVELS` on RP2350, asked with `wLength` 48), or 64 bytes if you adopt `SYS.PAGE_COMMANDS` (13.1.10). So the 64-byte buffer is enough, provided every GET sends a non-zero `wLength` of at most 64. The largest notification packet is also 64 bytes.

**Status handling at the choke point.** `dspiResultForStatus` and `dspiResultCanRetry` decide what every status means. Both need the changes in 13.1.9, and the link-loss counter in `dspiTransactionResult` should stop counting refusals from a live device.

#### 13.1.4 Frame format

The UART frame is the same as in 1.1.6. Only the meaning of `wIdxH` and of the status byte changed. Every multi-byte field is little-endian, low byte first.

```
SET request    A5 01 bReq wValL wValH wIdxL wIdxH wLenL wLenH payload[wLen] crcL crcH
GET request    A5 02 bReq wValL wValH wIdxL wIdxH wLenL wLenH               crcL crcH
SET response   A5 81 status 00 00                                            crcL crcH
GET response   A5 82 status lenL lenH payload[len]                           crcL crcH
Notification   A5 40 00 lenL lenH packet[len]                                crcL crcH
```

| Field | Meaning |
|---|---|
| `A5` | Sync byte. It starts every frame and is not covered by the CRC |
| Type | `0x01` SET request, `0x02` GET request, `0x81` SET response, `0x82` GET response, `0x40` notification |
| `bReq` | The command number within the page, `0x10` to `0xFF` |
| `wValL wValH` | The address, for example a preset slot. 0 for a command that addresses nothing |
| `wIdxL` | Ignored by 1.1.7. Send `0x00` |
| `wIdxH` | **The page**, `0x01` to `0xFF` |
| `wLenL wLenH` | SET: the payload length, at most 64. GET: the largest reply wanted. The device sends `min(reply, wLength)` bytes, and `wLength` 0 means the whole reply |
| Payload | SET only. A GET request never carries a payload |
| `status` | `0x00` success, `0x01` to `0x07` transport conditions, `0x10` and up the command's result (13.1.9) |
| `lenL lenH` | Number of payload bytes in a GET response. It is 0 whenever the status is not `0x00`, and always 0 in a SET response |
| `crcL crcH` | CRC16-CCITT-FALSE. The polynomial is `0x1021` and the initial value `0xFFFF`, with no reflection and no final XOR. It covers every byte after the sync byte, from the type byte to the last payload byte, and goes on the wire low byte first. The CRC of ASCII `123456789` is `0x29B1` |

The firmware parses requests this way. It hunts for `A5`, accepts only type `0x01` or `0x02`, reads the 7 header bytes, reads a payload only for a SET, and checks the CRC (low byte first). A bad CRC is answered with status `0x05`. A SET payload above 64 bytes is consumed and answered with status `0x06`. A frame that stalls for more than 100 ms between bytes is dropped without a response. The device answers one request at a time, so send the next request only after the response to the previous one.

One annotated example. Set the user volume to -20.0 dB with `ROUTE.USER_VOLUME` (page `0x05`, command `0x60`).

```
A5 01 60 00 00 00 05 04 00 00 00 A0 C1 36 C2

A5            sync
01            type: SET request
60            bReq: command 0x60
00 00         wValue 0 (the command addresses nothing)
00 05         wIndex 0x0500: wIdxL 0x00, wIdxH 0x05 = page ROUTE
04 00         wLength 4
00 00 A0 C1   payload, float32 -20.0, little-endian
36 C2         CRC16 0xC236 over 01 through C1, low byte first

Response:  A5 81 00 00 00 4C 2F      SET response, status 0x00, length 0
```

#### 13.1.5 Probe and identification

**TL;DR.** Before any other request, send one fixed 11-byte frame and check that the answer starts with `DSPi`. Firmware 1.1.6 ignores the page byte and would run whatever old command shares the new number (13.1.10 lists them for this panel). The probe itself is harmless on 1.1.6, which refuses it with status `0x02`.

**Where it goes.** Today `syncCurrentState` is the whole handshake, and the version read inside it is the first request. Add a protocol state. Its four values are unknown, paged, legacy and incompatible.

- Run the probe in the `!dspi.connected` branch of `loop()`, before `syncCurrentState(true)`, whenever the state is unknown. Run the sync in the same pass once the probe succeeds.
- Gate every request except the probe on the state inside `dspiTransactionResult`. Then no path can bypass it, including the Volume Limit apply, which today runs without `dspi.connected`.
- Reset the state to unknown wherever `dspiTransactionResult` sets `dspi.connected = false` (four failed transactions in a row), where `dspiRecoverAfterTimeout` resets the UART, and after any timeout. The DSPi can be reflashed from 1.1.7 back to 1.1.6, or the other way, while the ESP32 stays powered, and a reflash always produces a timeout. `dspi.connected` itself cannot serve as the "probed" flag, because any single successful frame sets it.

**The probe on firmware 1.1.7.** A GET of `SYS.PROTOCOL_INFO` (page `0x01`, command `0x10`) with `wLength` 16.

```
1.1.7  SYS.PROTOCOL_INFO (page 0x01, command 0x10)
write  A5 02 10 00 00 00 01 10 00 A0 BD
read   A5 82 00 10 00 44 53 50 69 01 00 0C 00 40 00 00 01 00 00 00 00 C5 E2
```

Layout A, `SYS.PROTOCOL_INFO` (16 bytes).

| Offset | Type | Field | 1.1.7 beta 1 | What to assert |
|---|---|---|---|---|
| 0 | u8[4] | Magic `"DSPi"` | `44 53 50 69` | Exactly |
| 4 | u8 | Protocol major | 1 | Exactly 1. Any other value means "incompatible". Show "update the panel" and send nothing else |
| 5 | u8 | Protocol minor | 0 | Nothing. A later minor only adds |
| 6 | u8 | Number of implemented pages | 12 in a release build | At least 1. Bench builds have more pages |
| 7 | u8 | Flags, reserved | 0 | Nothing |
| 8 | u16 | Largest SET payload | 64 | At least 4, the largest payload this panel sends |
| 10 | u16 | Largest GET reply | 256 | At least 64, the largest `wLength` this panel sends |
| 12 | u32 | Reserved | 0 | Nothing |

Also assert the frame itself. The type is `0x82`, the status `0x00` and the length exactly 16.

**The probe on firmware 1.1.6 or older.** Old firmware has no GET handler for `bRequest` `0x10`, so it changes nothing and answers status `0x02` with no payload.

```
1.1.6  probe answered by old firmware
write  A5 02 10 00 00 00 01 10 00 A0 BD
read   A5 82 02 00 00 F0 DA
```

On status `0x02`, set the state to legacy and choose one of two behaviours.

- **Stop.** Show "DSPi firmware 1.1.6, update required" and send nothing paged. The old version read still works on old firmware if you want to show the exact version. It is the frame below, the old `GET_PLATFORM` with `wLength` 7, and its reply has the layout B bytes of the old firmware.
- **Fall back.** Keep today's 1.1.6 code (the `spdif4-compat` request set, unchanged) as a second protocol table and use it while the state is legacy. Never mix the two tables on one device.

```
1.1.6  GET_PLATFORM 0x7F (on 1.1.6 firmware, RP2350)
write  A5 02 7F 00 00 02 00 07 00 EF C4
read   A5 82 00 07 00 01 01 16 09 01 06 00 42 16
```

Any other answer (a timeout, a CRC error, status `0x01`) means the link is not ready. Keep sending the probe, not the sync, on the existing 1000 ms retry.

**Identity: `SYS.PLATFORM` replaces `GET_PLATFORM`.** `SYS.PLATFORM` (page `0x01`, command `0x20`) has exactly the layout of the 1.1.6 `GET_PLATFORM` reply. The sync reads it with `wLength` 4 today, which still returns the same 4-byte prefix. Ask for 7 instead, so the version display can use the full version.

```
1.1.7  SYS.PLATFORM (page 0x01, command 0x20), RP2350, firmware 1.1.7 beta 1
write  A5 02 20 00 00 00 01 07 00 C9 7E
read   A5 82 00 07 00 01 01 17 09 01 07 01 03 9F
```

Layout B, `SYS.PLATFORM` (7 bytes).

| Offset | Type | Field | RP2350 1.1.7 beta 1 | RP2040 1.1.7 beta 1 |
|---|---|---|---|---|
| 0 | u8 | Platform: 0 RP2040, 1 RP2350 | `01` | `00` |
| 1 | u8 | Firmware major | `01` | `01` |
| 2 | u8 | `(minor << 4) \| patch`, legacy packed form | `17` | `17` |
| 3 | u8 | Number of outputs: 5 on RP2040, 9 on RP2350 | `09` | `05` |
| 4 | u8 | Firmware minor | `01` | `01` |
| 5 | u8 | Firmware patch | `07` | `07` |
| 6 | u8 | Beta number, 0 for a final release | `01` | `01` |

The RP2040 reply frame is `A5 82 00 07 00 00 01 17 05 01 07 01 50 68`. Show the version from bytes 1, 4 and 5, and append "beta n" when byte 6 is n above 0. Remove the check in the status screen that labels firmware `0x16` as a beta. It labels 1.1.6 final as a beta and would not label any 1.1.7 beta.

#### 13.1.6 Requests to port

**TL;DR.** Every request the panel sends is in the three tables below, one row per request. For most rows only `bRequest` and `wIndex` change. Rows marked `changed`, `split` or `W-as-R` also change the reply or the frame type, and 13.1.7 says exactly how.

How to read the tables.

- **Old** columns are the frame the panel sends today. Every old frame has `wIndex` `0x0002`. Old names are the 1.1.6 firmware names. The sketch spells four of them differently. Its `REQ_SET_LOUDNESS_REFERENCE` and `REQ_GET_LOUDNESS_REFERENCE` are `SET_LOUDNESS_REF` (0x5A) and `GET_LOUDNESS_REF` (0x5B). Its `REQ_SET_CROSSFEED_FREQUENCY` and `REQ_GET_CROSSFEED_FREQUENCY` are `SET_CROSSFEED_FREQ` (0x62) and `GET_CROSSFEED_FREQ` (0x63).
- **New** columns are the 1.1.7 frame. `wIndex` is `Page << 8`, so `wIdxH` is the page and `wIdxL` is 0.
- **Kind.** `same` means only the numbers change. `changed` means the reply layout or length changed. `split` means one old read became two requests. `W-as-R` marks a former write-as-read, a GET frame that changed state and is now a zero-length SET. `RP2350` marks a command that does not exist on RP2040, where it is refused with status `0x10`.
- **Reply bytes** is the exact payload length of a successful response. For a SET it is 0, because a SET response never carries a payload.
- **wValue.** Every command here addresses nothing except `STORE.PRESET_NAME`, `STORE.PRESET_SAVE` and `STORE.PRESET_LOAD` (preset slot 0 to 9) and `INPUT.I2S_RX_PIN` (I2S pair, 0 here). The four `GET_STATUS` selectors must become `wValue` 0. A non-zero `wValue` on a command that addresses nothing is refused with status `0x13`.
- No request of this panel is retired. Every one has a 1.1.7 equivalent.

Reads (GET frames today and in 1.1.7).

| Old request | Old no. | Old type | Old wValue | Old wLength | Kind | New command | Page | Cmd | New type | New wValue | SET payload | New wLength | Reply bytes | Reply |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `GET_PLATFORM` | 0x7F | GET | 0 | 4 | same | `SYS.PLATFORM` | 0x01 | 0x20 | GET | 0 | none | 7 | 7 | Bytes 0 to 3 identical to 1.1.6, bytes 4 to 6 new (layout B) |
| `GET_SPDIF_INPUT_CONFIG` | 0xEF | GET | 0 | 6 | same | `INPUT.SPDIF_RX_CONFIG` | 0x03 | 0x24 | GET | 0 | none | 6 | 6 | Identical to 1.1.6 (layout F) |
| `GET_ADAT_INPUT_ENABLE` | 0x69 | GET | 0 | 1 | RP2350 | `INPUT.ADAT_RX_ENABLE` | 0x03 | 0x40 | GET | 0 | none | 1 | 1 | u8 0 or 1, identical to 1.1.6. RP2040 refuses with 0x10 |
| `GET_ADAT_INPUT_PIN` | 0x6B | GET | 0 | 1 | RP2350 | `INPUT.ADAT_RX_PIN` | 0x03 | 0x41 | GET | 0 | none | 1 | 1 | u8 GPIO or 0xFF, identical to 1.1.6. RP2040 refuses with 0x10 |
| `GET_INPUT_SOURCE` | 0xE1 | GET | 0 | 1 | same | `INPUT.SOURCE` | 0x03 | 0x10 | GET | 0 | none | 1 | 1 | u8 source 0 to 6, identical to 1.1.6 |
| `GET_USER_VOLUME` | 0xDB | GET | 0 | 4 | same | `ROUTE.USER_VOLUME` | 0x05 | 0x60 | GET | 0 | none | 4 | 4 | f32 dB, identical to 1.1.6 |
| `GET_MASTER_VOLUME` | 0xD3 | GET | 0 | 4 | same | `ROUTE.MASTER_VOLUME` | 0x05 | 0x50 | GET | 0 | none | 4 | 4 | f32 dB, identical to 1.1.6 |
| `GET_USER_MUTE` | 0xDD | GET | 0 | 1 | same | `ROUTE.USER_MUTE` | 0x05 | 0x61 | GET | 0 | none | 1 | 1 | u8 0 or 1, identical to 1.1.6 |
| `PRESET_GET_ACTIVE` | 0x9A | GET | 0 | 1 | same | `STORE.PRESET_ACTIVE` | 0x02 | 0x14 | GET | 0 | none | 1 | 1 | u8 slot 0 to 9, identical to 1.1.6 |
| `GET_LOUDNESS` | 0x59 | GET | 0 | 1 | same | `DYN.LOUDNESS_ENABLE` | 0x07 | 0x30 | GET | 0 | none | 1 | 1 | u8 0 or 1, identical to 1.1.6 |
| `GET_LOUDNESS_REF` | 0x5B | GET | 0 | 4 | same | `DYN.LOUDNESS_REF` | 0x07 | 0x31 | GET | 0 | none | 4 | 4 | f32 dB SPL, identical to 1.1.6 |
| `GET_LOUDNESS_INTENSITY` | 0x5D | GET | 0 | 4 | same | `DYN.LOUDNESS_INTENSITY` | 0x07 | 0x32 | GET | 0 | none | 4 | 4 | f32 percent, identical to 1.1.6 |
| `GET_CROSSFEED` | 0x5F | GET | 0 | 1 | same | `FX.CROSSFEED_ENABLE` | 0x08 | 0x10 | GET | 0 | none | 1 | 1 | u8 0 or 1, identical to 1.1.6 |
| `GET_CROSSFEED_PRESET` | 0x61 | GET | 0 | 1 | same | `FX.CROSSFEED_PRESET` | 0x08 | 0x11 | GET | 0 | none | 1 | 1 | u8 0 to 3, identical to 1.1.6 |
| `GET_CROSSFEED_FREQ` | 0x63 | GET | 0 | 4 | same | `FX.CROSSFEED_FREQ` | 0x08 | 0x12 | GET | 0 | none | 4 | 4 | f32 Hz, identical to 1.1.6 |
| `GET_CROSSFEED_FEED` | 0x65 | GET | 0 | 4 | same | `FX.CROSSFEED_FEED` | 0x08 | 0x13 | GET | 0 | none | 4 | 4 | f32 dB, identical to 1.1.6 |
| `GET_LEVELLER_ENABLE` | 0xB5 | GET | 0 | 1 | same | `DYN.LEVELLER_ENABLE` | 0x07 | 0x20 | GET | 0 | none | 1 | 1 | u8 0 or 1, identical to 1.1.6 |
| `GET_LEVELLER_AMOUNT` | 0xB7 | GET | 0 | 4 | same | `DYN.LEVELLER_AMOUNT` | 0x07 | 0x21 | GET | 0 | none | 4 | 4 | f32 percent, identical to 1.1.6 |
| `GET_LEVELLER_SPEED` | 0xB9 | GET | 0 | 1 | same | `DYN.LEVELLER_SPEED` | 0x07 | 0x22 | GET | 0 | none | 1 | 1 | u8 0 to 2, identical to 1.1.6 |
| `GET_LEVELLER_MAX_GAIN` | 0xBB | GET | 0 | 4 | same | `DYN.LEVELLER_MAX_GAIN` | 0x07 | 0x23 | GET | 0 | none | 4 | 4 | f32 dB, identical to 1.1.6 |
| `GET_LEVELLER_GATE` | 0xBF | GET | 0 | 4 | same | `DYN.LEVELLER_GATE` | 0x07 | 0x25 | GET | 0 | none | 4 | 4 | f32 dBFS, identical to 1.1.6 |
| `GET_LEVELLER_LOOKAHEAD` | 0xBD | GET | 0 | 1 | same | `DYN.LEVELLER_LOOKAHEAD` | 0x07 | 0x24 | GET | 0 | none | 1 | 1 | u8 0 or 1, identical to 1.1.6 |
| `GET_PSYBASS` | 0x31 | GET | 0 | 1 | same | `FX.PSYBASS_ENABLE` | 0x08 | 0x40 | GET | 0 | none | 1 | 1 | u8 0 or 1, identical to 1.1.6 |
| `GET_PSYBASS_CUTOFF` | 0x33 | GET | 0 | 4 | same | `FX.PSYBASS_CUTOFF` | 0x08 | 0x41 | GET | 0 | none | 4 | 4 | f32 Hz, identical to 1.1.6 |
| `GET_PSYBASS_HARMONICS` | 0x35 | GET | 0 | 4 | same | `FX.PSYBASS_HARMONICS` | 0x08 | 0x42 | GET | 0 | none | 4 | 4 | f32 dB, identical to 1.1.6 |
| `GET_PSYBASS_DRIVE` | 0x37 | GET | 0 | 4 | same | `FX.PSYBASS_DRIVE` | 0x08 | 0x43 | GET | 0 | none | 4 | 4 | f32 dB, identical to 1.1.6 |
| `GET_PSYBASS_CHARACTER` | 0x39 | GET | 0 | 4 | same | `FX.PSYBASS_CHARACTER` | 0x08 | 0x44 | GET | 0 | none | 4 | 4 | f32 percent, identical to 1.1.6 |
| `GET_PSYBASS_ORIGINAL` | 0x3B | GET | 0 | 4 | same | `FX.PSYBASS_ORIGINAL` | 0x08 | 0x45 | GET | 0 | none | 4 | 4 | f32 dB, identical to 1.1.6 |
| `PRESET_GET_DIR` | 0x95 | GET | 0 | 7 | same | `STORE.DIRECTORY` | 0x02 | 0x15 | GET | 0 | none | 7 | 7 | Identical to 1.1.6 (layout F) |
| `PRESET_GET_NAME` | 0x93 | GET | slot 0-9 | 32 | same | `STORE.PRESET_NAME` | 0x02 | 0x13 | GET | slot 0-9 | none | 32 | 32 | 32 bytes NUL padded, identical to 1.1.6 |
| `GET_STATUS` sel 15 | 0x50 | GET | 15 | 4 | changed | `SYS.SYSTEM_STATUS` | 0x01 | 0x40 | GET | 0 | none | 16 | 16 | Changed. Sample rate is u32 at offset 4 (layout C) |
| `GET_SPDIF_RX_STATUS` | 0xE2 | GET | 0 | 16 | same | `INPUT.SPDIF_RX_STATUS` | 0x03 | 0x20 | GET | 0 | none | 16 | 16 | Identical to 1.1.6 (layout F) |
| `GET_SPDIF_RX_CH_STATUS` | 0xE3 | GET | 0 | 24 | same | `INPUT.SPDIF_RX_CH_STATUS` | 0x03 | 0x21 | GET | 0 | none | 24 | 24 | 24 raw channel status bytes, identical to 1.1.6 |
| `GET_ADAT_INPUT_STATUS` | 0x6E | GET | 0 | 20 | RP2350 | `INPUT.ADAT_RX_STATUS` | 0x03 | 0x43 | GET | 0 | none | 20 | 20 | Identical to 1.1.6 (layout F). RP2040 refuses with 0x10 |
| `GET_STATUS` sel 9 | 0x50 | GET | 9 | 48 | changed | `MEAS.LEVELS` | 0x09 | 0x10 | GET | 0 | none | 48 | 21 or 41 | Identical to 1.1.6 selector 9 (layout E) |
| `GET_STATUS` sel 10 | 0x50 | GET | 10 | 4 | changed | `SYS.USB_AUDIO_STATUS` | 0x01 | 0x41 | GET | 0 | none | 16 | 16 | Changed. Packet count is u32 at offset 0 (layout D) |
| `GET_STATUS` sel 11 | 0x50 | GET | 11 | 4 | changed | `SYS.USB_AUDIO_STATUS` | 0x01 | 0x41 | GET | 0 | none | 16 | 16 | Changed. Alt setting is u8 at offset 12 (layout D) |
| `GET_SAVED_MASTER_VOLUME` | 0xD7 | GET | 0 | 4 | same | `ROUTE.MASTER_VOLUME_SAVED` | 0x05 | 0x53 | GET | 0 | none | 4 | 4 | f32 dB, identical to 1.1.6 |
| `GET_MASTER_VOLUME_MODE` | 0xD5 | GET | 0 | 1 | same | `ROUTE.MASTER_VOLUME_MODE` | 0x05 | 0x51 | GET | 0 | none | 1 | 1 | u8 0 or 1, identical to 1.1.6 |
| `GET_I2S_CLOCK_MODE` | 0x89 | GET | 0 | 1 | same | `CLOCK.I2S_CLOCK_MODE` | 0x0C | 0x12 | GET | 0 | none | 1 | 1 | u8 0 or 1, identical to 1.1.6 |
| `GET_INPUT_RATE` (current rate) | 0xEE | GET | 0 | 8 | split | `SYS.SYSTEM_STATUS` | 0x01 | 0x40 | GET | 0 | none | 16 | 16 | Split. Old bytes 0 to 3 (current rate) are u32 at offset 4 (layout C) |
| `GET_INPUT_RATE` (selected rate) | 0xEE | GET | 0 | 8 | split | `INPUT.I2S_RX_RATE` | 0x03 | 0x32 | GET | 0 | none | 4 | 4 | Split. Old bytes 4 to 7 (selected rate) are this u32 Hz |
| `GET_I2S_RX_PIN` | 0xF2 | GET | pair 0 | 1 | same | `INPUT.I2S_RX_PIN` | 0x03 | 0x30 | GET | pair 0 | none | 1 | 1 | u8 GPIO, identical to 1.1.6 |

Former write-as-reads (GET frames today, zero-length SET frames in 1.1.7).

| Old request | Old no. | Old type | Old wValue | Old wLength | Kind | New command | Page | Cmd | New type | New wValue | SET payload | New wLength | Reply bytes | Reply |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `CLEAR_CLIPS` | 0x83 | GET | 0 | 4 | W-as-R | `MEAS.CLIP_CLEAR` | 0x09 | 0x12 | SET | 0 | 0, none | 0 | 0 | SET response, length 0. The 1.1.6 4-byte flags reply is gone |
| `PRESET_SAVE` | 0x90 | GET | slot 0-9 | 1 | W-as-R | `STORE.PRESET_SAVE` | 0x02 | 0x10 | SET | slot 0-9 | 0, none | 0 | 0 | SET response, length 0. The 1.1.6 status byte is gone |
| `PRESET_LOAD` | 0x91 | GET | slot 0-9 | 1 | W-as-R | `STORE.PRESET_LOAD` | 0x02 | 0x11 | SET | slot 0-9 | 0, none | 0 | 0 | SET response, length 0. The 1.1.6 status byte is gone |
| `SAVE_MASTER_VOLUME` | 0xD6 | GET | 0 | 1 | W-as-R | `ROUTE.MASTER_VOLUME_SAVE` | 0x05 | 0x52 | SET | 0 | 0, none | 0 | 0 | SET response, length 0. The 1.1.6 status byte is gone |

Writes (SET frames today and in 1.1.7). Every payload is byte for byte the 1.1.6 payload, sent with `wValue` 0.

| Old request | Old no. | Old type | Old wValue | Old wLength | Kind | New command | Page | Cmd | New type | New wValue | SET payload | New wLength | Reply bytes | Reply |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `SET_USER_VOLUME` | 0xDA | SET | 0 | 4 | same | `ROUTE.USER_VOLUME` | 0x05 | 0x60 | SET | 0 | 4, f32 dB | 4 | 0 | SET response, length 0 |
| `SET_MASTER_VOLUME` | 0xD2 | SET | 0 | 4 | same | `ROUTE.MASTER_VOLUME` | 0x05 | 0x50 | SET | 0 | 4, f32 dB | 4 | 0 | SET response, length 0 |
| `SET_MASTER_VOLUME_MODE` | 0xD4 | SET | 0 | 1 | same | `ROUTE.MASTER_VOLUME_MODE` | 0x05 | 0x51 | SET | 0 | 1, u8 mode | 1 | 0 | SET response, length 0 |
| `SET_USER_MUTE` | 0xDC | SET | 0 | 1 | same | `ROUTE.USER_MUTE` | 0x05 | 0x61 | SET | 0 | 1, u8 0 or 1 | 1 | 0 | SET response, length 0 |
| `SET_INPUT_SOURCE` | 0xE0 | SET | 0 | 1 | same | `INPUT.SOURCE` | 0x03 | 0x10 | SET | 0 | 1, u8 source | 1 | 0 | SET response, length 0 |
| `SET_INPUT_RATE` | 0xED | SET | 0 | 4 | same | `INPUT.I2S_RX_RATE` | 0x03 | 0x32 | SET | 0 | 4, u32 Hz | 4 | 0 | SET response, length 0 |
| `SET_I2S_CLOCK_MODE` | 0x88 | SET | 0 | 1 | same | `CLOCK.I2S_CLOCK_MODE` | 0x0C | 0x12 | SET | 0 | 1, u8 mode | 1 | 0 | SET response, length 0 |
| `SET_LOUDNESS` | 0x58 | SET | 0 | 1 | same | `DYN.LOUDNESS_ENABLE` | 0x07 | 0x30 | SET | 0 | 1, u8 0 or 1 | 1 | 0 | SET response, length 0 |
| `SET_LOUDNESS_REF` | 0x5A | SET | 0 | 4 | same | `DYN.LOUDNESS_REF` | 0x07 | 0x31 | SET | 0 | 4, f32 dB SPL | 4 | 0 | SET response, length 0 |
| `SET_LOUDNESS_INTENSITY` | 0x5C | SET | 0 | 4 | same | `DYN.LOUDNESS_INTENSITY` | 0x07 | 0x32 | SET | 0 | 4, f32 percent | 4 | 0 | SET response, length 0 |
| `SET_CROSSFEED` | 0x5E | SET | 0 | 1 | same | `FX.CROSSFEED_ENABLE` | 0x08 | 0x10 | SET | 0 | 1, u8 0 or 1 | 1 | 0 | SET response, length 0 |
| `SET_CROSSFEED_PRESET` | 0x60 | SET | 0 | 1 | same | `FX.CROSSFEED_PRESET` | 0x08 | 0x11 | SET | 0 | 1, u8 0 to 3 | 1 | 0 | SET response, length 0 |
| `SET_CROSSFEED_FREQ` | 0x62 | SET | 0 | 4 | same | `FX.CROSSFEED_FREQ` | 0x08 | 0x12 | SET | 0 | 4, f32 Hz | 4 | 0 | SET response, length 0 |
| `SET_CROSSFEED_FEED` | 0x64 | SET | 0 | 4 | same | `FX.CROSSFEED_FEED` | 0x08 | 0x13 | SET | 0 | 4, f32 dB | 4 | 0 | SET response, length 0 |
| `SET_LEVELLER_ENABLE` | 0xB4 | SET | 0 | 1 | same | `DYN.LEVELLER_ENABLE` | 0x07 | 0x20 | SET | 0 | 1, u8 0 or 1 | 1 | 0 | SET response, length 0 |
| `SET_LEVELLER_AMOUNT` | 0xB6 | SET | 0 | 4 | same | `DYN.LEVELLER_AMOUNT` | 0x07 | 0x21 | SET | 0 | 4, f32 percent | 4 | 0 | SET response, length 0 |
| `SET_LEVELLER_SPEED` | 0xB8 | SET | 0 | 1 | same | `DYN.LEVELLER_SPEED` | 0x07 | 0x22 | SET | 0 | 1, u8 0 to 2 | 1 | 0 | SET response, length 0 |
| `SET_LEVELLER_MAX_GAIN` | 0xBA | SET | 0 | 4 | same | `DYN.LEVELLER_MAX_GAIN` | 0x07 | 0x23 | SET | 0 | 4, f32 dB | 4 | 0 | SET response, length 0 |
| `SET_LEVELLER_GATE` | 0xBE | SET | 0 | 4 | same | `DYN.LEVELLER_GATE` | 0x07 | 0x25 | SET | 0 | 4, f32 dBFS | 4 | 0 | SET response, length 0 |
| `SET_LEVELLER_LOOKAHEAD` | 0xBC | SET | 0 | 1 | same | `DYN.LEVELLER_LOOKAHEAD` | 0x07 | 0x24 | SET | 0 | 1, u8 0 or 1 | 1 | 0 | SET response, length 0 |
| `SET_PSYBASS` | 0x30 | SET | 0 | 1 | same | `FX.PSYBASS_ENABLE` | 0x08 | 0x40 | SET | 0 | 1, u8 0 or 1 | 1 | 0 | SET response, length 0 |
| `SET_PSYBASS_CUTOFF` | 0x32 | SET | 0 | 4 | same | `FX.PSYBASS_CUTOFF` | 0x08 | 0x41 | SET | 0 | 4, f32 Hz | 4 | 0 | SET response, length 0 |
| `SET_PSYBASS_HARMONICS` | 0x34 | SET | 0 | 4 | same | `FX.PSYBASS_HARMONICS` | 0x08 | 0x42 | SET | 0 | 4, f32 dB | 4 | 0 | SET response, length 0 |
| `SET_PSYBASS_DRIVE` | 0x36 | SET | 0 | 4 | same | `FX.PSYBASS_DRIVE` | 0x08 | 0x43 | SET | 0 | 4, f32 dB | 4 | 0 | SET response, length 0 |
| `SET_PSYBASS_CHARACTER` | 0x38 | SET | 0 | 4 | same | `FX.PSYBASS_CHARACTER` | 0x08 | 0x44 | SET | 0 | 4, f32 percent | 4 | 0 | SET response, length 0 |
| `SET_PSYBASS_ORIGINAL` | 0x3A | SET | 0 | 4 | same | `FX.PSYBASS_ORIGINAL` | 0x08 | 0x45 | SET | 0 | 4, f32 dB | 4 | 0 | SET response, length 0 |

**Every frame, old and new.** These are the exact bytes, so you can unit-test the new frame builder without a device. Rows with a slot use slot 0. For another slot, put it in `wValL` and recompute the CRC. SET rows use example payloads. They are -20.0 dB for both volumes, mode 0, mute 1, source 2 (I2S), 48000 Hz, clock mode 0, enables 1, loudness reference 83.0 and intensity 100.0, crossfeed preset 0, 700.0 Hz and 4.5 dB, leveller amount 50.0, speed 1, maximum gain 12.0 and gate -60.0, psychoacoustic bass cutoff 80.0, harmonics 0.0, drive 6.0, character 50.0 and original 0.0.

| Request | 1.1.6 frame (sent today) | 1.1.7 frame |
|---|---|---|
| `GET_PLATFORM` | `A5 02 7F 00 00 02 00 04 00 BC 91` | `A5 02 20 00 00 00 01 07 00 C9 7E` |
| `GET_SPDIF_INPUT_CONFIG` | `A5 02 EF 00 00 02 00 06 00 5C 6B` | `A5 02 24 00 00 00 03 06 00 5E E2` |
| `GET_ADAT_INPUT_ENABLE` | `A5 02 69 00 00 02 00 01 00 17 F8` | `A5 02 40 00 00 00 03 01 00 15 09` |
| `GET_ADAT_INPUT_PIN` | `A5 02 6B 00 00 02 00 01 00 F4 98` | `A5 02 41 00 00 00 03 01 00 74 B1` |
| `GET_INPUT_SOURCE` | `A5 02 E1 00 00 02 00 01 00 43 C0` | `A5 02 10 00 00 00 03 01 00 82 E3` |
| `GET_USER_VOLUME` | `A5 02 DB 00 00 02 00 04 00 75 95` | `A5 02 60 00 00 00 05 04 00 B6 2A` |
| `GET_MASTER_VOLUME` | `A5 02 D3 00 00 02 00 04 00 D8 06` | `A5 02 50 00 00 00 05 04 00 3B 73` |
| `GET_USER_MUTE` | `A5 02 DD 00 00 02 00 01 00 A5 CB` | `A5 02 61 00 00 00 05 01 00 22 6D` |
| `PRESET_GET_ACTIVE` | `A5 02 9A 00 00 02 00 01 00 0D 0F` | `A5 02 14 00 00 00 02 01 00 74 15` |
| `GET_LOUDNESS` | `A5 02 59 00 00 02 00 01 00 9A A1` | `A5 02 30 00 00 00 07 01 00 B4 51` |
| `GET_LOUDNESS_REF` | `A5 02 5B 00 00 02 00 04 00 8C 3E` | `A5 02 31 00 00 00 07 04 00 20 16` |
| `GET_LOUDNESS_INTENSITY` | `A5 02 5D 00 00 02 00 04 00 A9 9F` | `A5 02 32 00 00 00 07 04 00 A2 CE` |
| `GET_CROSSFEED` | `A5 02 5F 00 00 02 00 01 00 BF 00` | `A5 02 10 00 00 00 08 01 00 73 13` |
| `GET_CROSSFEED_PRESET` | `A5 02 61 00 00 02 00 01 00 BA 6B` | `A5 02 11 00 00 00 08 01 00 12 AB` |
| `GET_CROSSFEED_FREQ` | `A5 02 63 00 00 02 00 04 00 AC F4` | `A5 02 12 00 00 00 08 04 00 65 8C` |
| `GET_CROSSFEED_FEED` | `A5 02 65 00 00 02 00 04 00 89 55` | `A5 02 13 00 00 00 08 04 00 04 34` |
| `GET_LEVELLER_ENABLE` | `A5 02 B5 00 00 02 00 01 00 12 EB` | `A5 02 20 00 00 00 07 01 00 CF 66` |
| `GET_LEVELLER_AMOUNT` | `A5 02 B7 00 00 02 00 04 00 04 74` | `A5 02 21 00 00 00 07 04 00 5B 21` |
| `GET_LEVELLER_SPEED` | `A5 02 B9 00 00 02 00 01 00 79 B9` | `A5 02 22 00 00 00 07 01 00 2C 06` |
| `GET_LEVELLER_MAX_GAIN` | `A5 02 BB 00 00 02 00 04 00 6F 26` | `A5 02 23 00 00 00 07 04 00 B8 41` |
| `GET_LEVELLER_GATE` | `A5 02 BF 00 00 02 00 04 00 A9 E7` | `A5 02 25 00 00 00 07 04 00 9D E0` |
| `GET_LEVELLER_LOOKAHEAD` | `A5 02 BD 00 00 02 00 01 00 BF 78` | `A5 02 24 00 00 00 07 01 00 09 A7` |
| `GET_PSYBASS` | `A5 02 31 00 00 02 00 01 00 2D 81` | `A5 02 40 00 00 00 08 01 00 E4 F9` |
| `GET_PSYBASS_CUTOFF` | `A5 02 33 00 00 02 00 04 00 3B 1E` | `A5 02 41 00 00 00 08 04 00 70 BE` |
| `GET_PSYBASS_HARMONICS` | `A5 02 35 00 00 02 00 04 00 1E BF` | `A5 02 42 00 00 00 08 04 00 F2 66` |
| `GET_PSYBASS_DRIVE` | `A5 02 37 00 00 02 00 04 00 FD DF` | `A5 02 43 00 00 00 08 04 00 93 DE` |
| `GET_PSYBASS_CHARACTER` | `A5 02 39 00 00 02 00 04 00 75 ED` | `A5 02 44 00 00 00 08 04 00 D7 C7` |
| `GET_PSYBASS_ORIGINAL` | `A5 02 3B 00 00 02 00 04 00 96 8D` | `A5 02 45 00 00 00 08 04 00 B6 7F` |
| `PRESET_GET_DIR` | `A5 02 95 00 00 02 00 07 00 42 2F` | `A5 02 15 00 00 00 02 07 00 B3 07` |
| `PRESET_GET_NAME` | `A5 02 93 00 00 02 00 20 00 16 11` | `A5 02 13 00 00 00 02 20 00 E7 39` |
| `GET_STATUS` sel 15 | `A5 02 50 0F 00 02 00 04 00 A0 B0` | `A5 02 40 00 00 00 01 10 00 37 57` |
| `GET_SPDIF_RX_STATUS` | `A5 02 E2 00 00 02 00 10 00 83 28` | `A5 02 20 00 00 00 03 10 00 4D 8A` |
| `GET_SPDIF_RX_CH_STATUS` | `A5 02 E3 00 00 02 00 18 00 4B 19` | `A5 02 21 00 00 00 03 18 00 85 BB` |
| `GET_ADAT_INPUT_STATUS` | `A5 02 6E 00 00 02 00 14 00 D5 1D` | `A5 02 43 00 00 00 03 14 00 11 2D` |
| `GET_STATUS` sel 9 | `A5 02 50 09 00 02 00 30 00 10 F4` | `A5 02 10 00 00 00 09 30 00 E7 12` |
| `GET_STATUS` sel 10 | `A5 02 50 0A 00 02 00 04 00 A1 F3` | `A5 02 41 00 00 00 01 10 00 56 EF` |
| `GET_STATUS` sel 11 | `A5 02 50 0B 00 02 00 04 00 01 B6` | `A5 02 41 00 00 00 01 10 00 56 EF` |
| `GET_SAVED_MASTER_VOLUME` | `A5 02 D7 00 00 02 00 04 00 1E C7` | `A5 02 53 00 00 00 05 04 00 B9 AB` |
| `GET_MASTER_VOLUME_MODE` | `A5 02 D5 00 00 02 00 01 00 08 58` | `A5 02 51 00 00 00 05 01 00 AF 34` |
| `GET_I2S_CLOCK_MODE` | `A5 02 89 00 00 02 00 01 00 F4 E0` | `A5 02 12 00 00 00 0C 01 00 50 AF` |
| `GET_INPUT_RATE` (current rate) | `A5 02 EE 00 00 02 00 08 00 32 F0` | `A5 02 40 00 00 00 01 10 00 37 57` |
| `GET_INPUT_RATE` (selected rate) | `A5 02 EE 00 00 02 00 08 00 32 F0` | `A5 02 32 00 00 00 03 04 00 62 12` |
| `GET_I2S_RX_PIN` | `A5 02 F2 00 00 02 00 01 00 BA 2F` | `A5 02 30 00 00 00 03 01 00 74 8D` |
| `CLEAR_CLIPS` | `A5 02 83 00 00 02 00 04 00 4F EC` | `A5 01 12 00 00 00 09 00 00 E4 BF` |
| `PRESET_SAVE` | `A5 02 90 00 00 02 00 01 00 43 FC` | `A5 01 10 00 00 00 02 00 00 F6 2F` |
| `PRESET_LOAD` | `A5 02 91 00 00 02 00 01 00 22 44` | `A5 01 11 00 00 00 02 00 00 97 97` |
| `SAVE_MASTER_VOLUME` | `A5 02 D6 00 00 02 00 01 00 8A 80` | `A5 01 52 00 00 00 05 00 00 69 17` |
| `SET_USER_VOLUME` | `A5 01 DA 00 00 02 00 04 00 00 00 A0 C1 E7 40` | `A5 01 60 00 00 00 05 04 00 00 00 A0 C1 36 C2` |
| `SET_MASTER_VOLUME` | `A5 01 D2 00 00 02 00 04 00 00 00 A0 C1 69 E2` | `A5 01 50 00 00 00 05 04 00 00 00 A0 C1 71 3D` |
| `SET_MASTER_VOLUME_MODE` | `A5 01 D4 00 00 02 00 01 00 00 6A B9` | `A5 01 51 00 00 00 05 01 00 00 93 F4` |
| `SET_USER_MUTE` | `A5 01 DC 00 00 02 00 01 00 01 91 B7` | `A5 01 61 00 00 00 05 01 00 01 6E A2` |
| `SET_INPUT_SOURCE` | `A5 01 E0 00 00 02 00 01 00 02 99 D0` | `A5 01 10 00 00 00 03 01 00 02 4B 42` |
| `SET_INPUT_RATE` | `A5 01 ED 00 00 02 00 04 00 80 BB 00 00 38 91` | `A5 01 32 00 00 00 03 04 00 80 BB 00 00 6D D5` |
| `SET_I2S_CLOCK_MODE` | `A5 01 88 00 00 02 00 01 00 00 B9 63` | `A5 01 12 00 00 00 0C 01 00 00 41 39` |
| `SET_LOUDNESS` | `A5 01 58 00 00 02 00 01 00 01 7D 45` | `A5 01 30 00 00 00 07 01 00 01 B1 C3` |
| `SET_LOUDNESS_REF` | `A5 01 5A 00 00 02 00 04 00 00 00 A6 42 00 C2` | `A5 01 31 00 00 00 07 04 00 00 00 A6 42 39 60` |
| `SET_LOUDNESS_INTENSITY` | `A5 01 5C 00 00 02 00 04 00 00 00 C8 42 D1 1B` | `A5 01 32 00 00 00 07 04 00 00 00 C8 42 E6 30` |
| `SET_CROSSFEED` | `A5 01 5E 00 00 02 00 01 00 01 B6 C5` | `A5 01 10 00 00 00 08 01 00 01 37 6C` |
| `SET_CROSSFEED_PRESET` | `A5 01 60 00 00 02 00 01 00 00 5A 0D` | `A5 01 11 00 00 00 08 01 00 00 C5 3B` |
| `SET_CROSSFEED_FREQ` | `A5 01 62 00 00 02 00 04 00 00 00 2F 44 0F 5E` | `A5 01 12 00 00 00 08 04 00 00 00 2F 44 96 F9` |
| `SET_CROSSFEED_FEED` | `A5 01 64 00 00 02 00 04 00 00 00 90 40 4C E1` | `A5 01 13 00 00 00 08 04 00 00 00 90 40 68 6F` |
| `SET_LEVELLER_ENABLE` | `A5 01 B4 00 00 02 00 01 00 01 F3 24` | `A5 01 20 00 00 00 07 01 00 01 05 FE` |
| `SET_LEVELLER_AMOUNT` | `A5 01 B6 00 00 02 00 04 00 00 00 48 42 71 65` | `A5 01 21 00 00 00 07 04 00 00 00 48 42 B9 06` |
| `SET_LEVELLER_SPEED` | `A5 01 B8 00 00 02 00 01 00 01 44 35` | `A5 01 22 00 00 00 07 01 00 01 A3 71` |
| `SET_LEVELLER_MAX_GAIN` | `A5 01 BA 00 00 02 00 04 00 00 00 40 41 72 2F` | `A5 01 23 00 00 00 07 04 00 00 00 40 41 C0 1F` |
| `SET_LEVELLER_GATE` | `A5 01 BE 00 00 02 00 04 00 00 00 70 C2 4B DA` | `A5 01 25 00 00 00 07 04 00 00 00 70 C2 4A 4A` |
| `SET_LEVELLER_LOOKAHEAD` | `A5 01 BC 00 00 02 00 01 00 01 29 3A` | `A5 01 24 00 00 00 07 01 00 01 68 F1` |
| `SET_PSYBASS` | `A5 01 30 00 00 02 00 01 00 01 1F D6` | `A5 01 40 00 00 00 08 01 00 01 53 A7` |
| `SET_PSYBASS_CUTOFF` | `A5 01 32 00 00 02 00 04 00 00 00 A0 42 87 24` | `A5 01 41 00 00 00 08 04 00 00 00 A0 42 E4 FB` |
| `SET_PSYBASS_HARMONICS` | `A5 01 34 00 00 02 00 04 00 00 00 00 00 8B A0` | `A5 01 42 00 00 00 08 04 00 00 00 00 00 E6 F6` |
| `SET_PSYBASS_DRIVE` | `A5 01 36 00 00 02 00 04 00 00 00 C0 40 A8 5E` | `A5 01 43 00 00 00 08 04 00 00 00 C0 40 3F 70` |
| `SET_PSYBASS_CHARACTER` | `A5 01 38 00 00 02 00 04 00 00 00 48 42 A1 BF` | `A5 01 44 00 00 00 08 04 00 00 00 48 42 F1 EB` |
| `SET_PSYBASS_ORIGINAL` | `A5 01 3A 00 00 02 00 04 00 00 00 00 00 F1 F3` | `A5 01 45 00 00 00 08 04 00 00 00 00 00 5B DF` |

#### 13.1.7 Replies and checks that change

**TL;DR.** Almost every reply is byte for byte what 1.1.6 sent, so the panel's strict checks keep working. Five reads are the exception. Three of them are old `GET_STATUS` selectors that became fields inside 16-byte structures, so a 4-byte read now returns the wrong field. The fourth is the 8-byte input-rate read, which is now two reads. The fifth is the meter read, whose bytes are unchanged but whose `wValue` must become 0. The four write-as-reads lose their reply byte.

**The 4-byte trap.** The device cuts a reply to `wLength`. A strict client that keeps asking for 4 bytes of a 16-byte structure gets the first 4 bytes, which is the first field, not the one it wanted. Old selector 15 was the sample rate. `SYS.SYSTEM_STATUS` puts `clk_sys` at offset 0 and the sample rate at offset 4. A port that asks `SYS.SYSTEM_STATUS` for 4 bytes passes the length check and then shows 307200000 Hz as the sample rate (13.1.8 shows the frames). Always ask these structures for 16 bytes, require exactly 16, and read the field at its offset.

Layout C, `SYS.SYSTEM_STATUS` (page `0x01`, command `0x40`, 16 bytes). It replaces selector 15 and the first half of `GET_INPUT_RATE`.

| Offset | Type | Field | Replaces |
|---|---|---|---|
| 0 | u32 | `clk_sys` frequency, Hz | (selector 13, not used by the panel) |
| 4 | u32 | Device sample rate, Hz | Selector 15, and bytes 0 to 3 of `GET_INPUT_RATE` |
| 8 | u16 | Core voltage, mV | |
| 10 | i16 | Chip temperature, hundredths of a degree C | |
| 12 | u8 | Core 0 load, percent | |
| 13 | u8 | Core 1 load, percent | |
| 14 | u16 | Reserved, 0 | |

Layout D, `SYS.USB_AUDIO_STATUS` (page `0x01`, command `0x41`, 16 bytes). One read replaces selectors 10 and 11.

| Offset | Type | Field | Replaces |
|---|---|---|---|
| 0 | u32 | USB audio packets received | Selector 10, the same value |
| 4 | u32 | USB audio ring overruns | |
| 8 | u32 | Worst gap between USB audio packets, us | |
| 12 | u8 | Current USB audio alt setting. 0 means the host closed the stream | Selector 11, which was a u32. Now a single byte |
| 13 | u8 | 1 while the USB audio interface is mounted | |
| 14 | u16 | Reserved, 0 | |

Layout E, `MEAS.LEVELS` (page `0x09`, command `0x10`). It is byte for byte the 1.1.6 selector 9 packet. Its length is `2 x N + 7`, where N is the channel count, so it is 21 bytes on RP2040 (N = 7) and 41 bytes on RP2350 (N = 17).

| Offset | Type | Field |
|---|---|---|
| 0 | u16[N] | Peak level per channel, inputs then outputs. 0 to 32767 is 0 to full scale |
| 2N | u8 | Core 0 load, percent |
| 2N + 1 | u8 | Core 1 load, percent |
| 2N + 2 | u32 | Latched clip flags, bit k = channel k |
| 2N + 6 | u8 | Active input channel count |

`pollMeters` asks for 48 bytes and picks the layout from the length. Its existing heuristic reads 41 bytes as 17 channels and 21 bytes as 7 channels, both with a 4-byte clip mask, so it keeps working once `wValue` is 0. If you tighten it, accept exactly 21 or 41.

Layout F. These replies are identical to 1.1.6. They are written out because the panel checks individual bytes in them.

| Command | Offset | Type | Field | 1.1.7 value |
|---|---|---|---|---|
| `INPUT.SPDIF_RX_CONFIG` (6 bytes) | 0 | u8 | Number of S/PDIF inputs | 4 on both platforms |
| | 1 | u8 | Enable mask over all inputs, bit i = input i | Bit 0 always set, at most `0x0F` |
| | 2 to 5 | u8[4] | Stored GPIO of inputs 0 to 3 | 0 to 29 on RP2040, 0 to 47 on RP2350. A disabled optional input may read `0xFF`, which the panel already allows |
| `STORE.DIRECTORY` (7 bytes) | 0 | u16 | Occupied slots, bit n = slot n | At most `0x03FF` |
| | 2 | u8 | Startup mode | 0 or 1 |
| | 3 | u8 | Default slot | 0 to 9 |
| | 4 | u8 | Last active slot | 0 to 9 |
| | 5 | u8 | IO config mode | 0 or 1 |
| | 6 | u8 | Master volume mode | 0 or 1 |
| `INPUT.SPDIF_RX_STATUS` (16 bytes) | 0 | u8 | State: 0 inactive, 1 acquiring, 2 locked, 3 relocking | 0 to 3 |
| | 1 | u8 | Active source ID | The source the panel read |
| | 2 | u8 | Locks since activation | |
| | 3 | u8 | Lock losses since activation | |
| | 4 | u32 | Detected sample rate, Hz. 0 unless locked | |
| | 8 | u32 | Parity errors | |
| | 12 | u16 | Receive FIFO fill, percent | |
| | 14 | u16 | Reserved | |
| `INPUT.ADAT_RX_STATUS` (20 bytes, RP2350 only) | 0 | u8 | State: 0 inactive, 1 acquiring, 2 syncing, 3 locked, 4 relocking | 0 to 4 |
| | 1 | u8 | Live clock mode, 0 master, 1 slave | 0 or 1 |
| | 2 | u8 | Configured enable | Equals `INPUT.ADAT_RX_ENABLE` |
| | 3 | u8 | Configured pin, `0xFF` unset | Equals `INPUT.ADAT_RX_PIN` |
| | 4 | u8 | Rate OK | 0 or 1 |
| | 5 to 11 | | Locks, losses, header counters, reserved | |
| | 12 | u32 | Detected rate while locked, Hz | |
| | 16 | u32 | Slave mode measured wire rate, Hz | |

**Every strict check, before and after.** Function names are those of the sketch.

| Where | Request after the port | Check today | Check after the port |
|---|---|---|---|
| `syncCurrentState`, step 1 | `SYS.PLATFORM`, `wLength` 7 | Length exactly 4. Byte 0 at most 1, byte 3 not 0 | Length exactly 7. Byte 0 at most 1. Byte 3 is 5 or 9. Version from bytes 1, 4, 5 and 6 |
| `syncCurrentState`, S/PDIF inventory | `INPUT.SPDIF_RX_CONFIG`, `wLength` 6 | Byte 0 from 1 to 8, length at least 2 plus the inputs used, mask and pin checks | Unchanged. 1.1.7 answers exactly 6 bytes with byte 0 = 4 |
| `syncCurrentState`, ADAT | `INPUT.ADAT_RX_ENABLE`, `INPUT.ADAT_RX_PIN`, `wLength` 1 | Length 1, enable at most 1, pin sane | Unchanged on RP2350. On RP2040 do not send them (13.1.10). Use enable 0 and pin `0xFF`, what an RP2040 on 1.1.6 normally answered |
| `syncCurrentState` and `setInputSource` | `INPUT.SOURCE` | Length 1, at most 6 (`SRC_MAX`), equal after a SET | Unchanged |
| `syncCurrentState` and `loadPreset` | `ROUTE.USER_VOLUME` | Length 4 and finite. After a preset load, -60.01 to 0.01 | Unchanged. The device clamps to -60 to 0 |
| `syncCurrentState` and `setMasterVolumeVerified` | `ROUTE.MASTER_VOLUME` | Length 4 and finite. Readback within 0.26 dB | Unchanged. The device clamps to -128 to 0 |
| `syncCurrentState` and `setUserMuteVerified` | `ROUTE.USER_MUTE` | At most 1, equal after a SET | Unchanged |
| `syncCurrentState` and `loadPreset` | `STORE.PRESET_ACTIVE` | Below 10, equal to the slot after a load | Unchanged |
| `syncCurrentState` | `DYN.LOUDNESS_ENABLE`, `FX.CROSSFEED_ENABLE`, `DYN.LEVELLER_ENABLE`, `DYN.LEVELLER_LOOKAHEAD` | Length 1, at most 1 | Unchanged |
| `syncCurrentState` | `FX.CROSSFEED_PRESET`, `DYN.LEVELLER_SPEED` | At most 3, at most 2 | Unchanged |
| `syncCurrentState` | The loudness, crossfeed and leveller floats | Length 4 and finite | Unchanged |
| `syncPsybassCandidate`, capability probe | `FX.PSYBASS_ENABLE` | Status `0x02` means "not supported" and the sync continues. Otherwise length 1, at most 1 | 1.1.7 never sends `0x02`. Key "not supported" on status `0x10`, or on the `SYS.PAGE_COMMANDS` bit of page `0x08` command `0x40`. The command exists on both platforms in 1.1.7, so the probe succeeds today |
| `syncPsybassCandidate` and the psybass setters | `FX.PSYBASS_CUTOFF` to `FX.PSYBASS_ORIGINAL` | Length 4, ranges 30 to 300, -24 to 12, 0 to 18, 0 to 100 and -60 to 0 | Unchanged. The device clamps to the same ranges |
| `syncPresetDirectoryCandidate` | `STORE.DIRECTORY`, `wLength` 7 | Length 7, occupancy at most `0x03FF`, byte 6 at most 1 | Unchanged |
| `syncPresetNameCandidate` | `STORE.PRESET_NAME`, `wValue` slot | Length 32 | Unchanged |
| `syncCurrentState`, sample rate | `SYS.SYSTEM_STATUS`, `wValue` 0, `wLength` 16 | `GET_STATUS` selector 15, length 4, u32 at offset 0 | Length exactly 16, rate is the u32 at **offset 4** |
| `syncCurrentState`, S/PDIF source | `INPUT.SPDIF_RX_STATUS`, `wLength` 16 | Length 16, byte 0 at most 3, byte 1 equal to the source, rate at offset 4 | Unchanged |
| `syncCurrentState`, locked S/PDIF | `INPUT.SPDIF_RX_CH_STATUS`, `wLength` 24 | Length 24, byte 0 bit 1 is "non-audio" | Unchanged |
| `syncCurrentState`, ADAT source | `INPUT.ADAT_RX_STATUS`, `wLength` 20 | Length 20 and the byte checks of layout F | Unchanged. RP2350 only, and source 3 never appears on RP2040 |
| `pollMeters` | `MEAS.LEVELS`, `wValue` 0, `wLength` 48 | Length picks the layout, clip mask must fit | Unchanged. Exact lengths are 21 and 41 |
| `pollMeters` and `enterVisualizer`, clip clear | `MEAS.CLIP_CLEAR`, zero-length SET | Reply ignored | SET response, status `0x00`, length 0. Send it with `dspiSet` and no payload. The flags are already in `MEAS.LEVELS` |
| `readStatusU32ForMeter` and `pollUsbMeterStreamActive` | `SYS.USB_AUDIO_STATUS`, `wValue` 0, `wLength` 16, once per poll | Selectors 10 and 11, each length 4 | Length exactly 16. Packets are the u32 at offset 0. The alt setting is the **u8 at offset 12** |
| `ensureIndependentMasterVolumeModeVerified` | `ROUTE.MASTER_VOLUME_MODE` | Mode equal to 0 within 6 polls | Unchanged |
| `persistIndependentMasterVolumeVerified` | `ROUTE.MASTER_VOLUME_SAVE`, zero-length SET | Length 1 and byte 0 equal to 0 | Status `0x00`, length 0 (`dspiSet` returns true). Remove the byte test |
| `persistIndependentMasterVolumeVerified` | `ROUTE.MASTER_VOLUME_SAVED` | Within 0.26 dB of the live value within 6 polls | Unchanged |
| `saveCurrentPreset` | `STORE.PRESET_SAVE`, zero-length SET, `wValue` slot | Length 1 and byte 0 equal to 0 | Status `0x00`, length 0. Keep the completion check in `servicePresetSaveCompletion` |
| `loadPreset` | `STORE.PRESET_LOAD`, zero-length SET, `wValue` slot | Accepted when length 1 and byte 0 equal to 0, otherwise resolve by polling the active slot | Accepted when the status is `0x00`. A status of `0x10` or above is a definite refusal and nothing was loaded. Only a timeout is ambiguous and needs the polling. Log the status code |
| `restoreDspiMediaRoute`, media route, `loadPreset` | `CLOCK.I2S_CLOCK_MODE` | Length 1, equal to 0 or to the captured mode | Unchanged. The GET reports the live mode, so poll after a SET as today |
| `getDspiInputRates` | `SYS.SYSTEM_STATUS` (16) and `INPUT.I2S_RX_RATE` (4) | One read, length 8, current rate at 0, selected rate at 4 | Two reads. Current rate is the u32 at offset 4 of `SYS.SYSTEM_STATUS`, length exactly 16. Selected rate is the whole 4-byte `INPUT.I2S_RX_RATE` reply, length exactly 4, value 44100, 48000 or 96000. The seven callers keep their signature |
| `activateDspiMediaRoute`, `prepareDspiMediaRoute`, `printMediaPlaybackStatus` | `INPUT.I2S_RX_PIN`, `wValue` 0 | Length 1, equal to GPIO 1 | Unchanged |
| Every SET | Any SET | Status `0x00` and length 0 | Unchanged. Refusals now arrive as status codes (13.1.9) |

Do not map `GET_INPUT_RATE` to `INPUT.I2S_RX_RATE` alone. Its reply is only the selected rate, and reading it as the current rate breaks the media route's "rate has switched" test.

#### 13.1.8 Worked frames

Each pair shows the frame the panel sends today and what 1.1.6 answers, then the 1.1.7 replacement and what 1.1.7 answers. Values in replies are examples. Live readings such as the sample rate or packet count differ on every device.

**A scalar GET.** Read the user volume (-20.0 dB here). Only the number and page change, and the reply bytes are the same.

```
1.1.6  GET_USER_VOLUME 0xDB
write  A5 02 DB 00 00 02 00 04 00 75 95
read   A5 82 00 04 00 00 00 A0 C1 AB 91

1.1.7  ROUTE.USER_VOLUME (page 0x05, command 0x60)
write  A5 02 60 00 00 00 05 04 00 B6 2A
read   A5 82 00 04 00 00 00 A0 C1 AB 91
```

**A SET with a payload.** Set the user volume to -20.0 dB. The payload is unchanged.

```
1.1.6  SET_USER_VOLUME 0xDA
write  A5 01 DA 00 00 02 00 04 00 00 00 A0 C1 E7 40
read   A5 81 00 00 00 4C 2F

1.1.7  ROUTE.USER_VOLUME (page 0x05, command 0x60)
write  A5 01 60 00 00 00 05 04 00 00 00 A0 C1 36 C2
read   A5 81 00 00 00 4C 2F
```

A refused SET returns the reason in the status byte. A NaN volume (`00 00 C0 7F`) is refused with `VC_ERR_VALUE`.

```
1.1.7  ROUTE.USER_VOLUME (page 0x05, command 0x60), NaN payload
write  A5 01 60 00 00 00 05 04 00 00 00 C0 7F 09 8F
read   A5 81 14 00 00 EF B0
```

**Former write-as-read: clear the clip flags.** 1.1.6 answered the flags it cleared (here bit 8). 1.1.7 clears with a zero-length SET and answers nothing. A mechanical port that keeps the GET frame is refused with `VC_ERR_DIRECTION` (`0x11`).

```
1.1.6  CLEAR_CLIPS 0x83 (write-as-read)
write  A5 02 83 00 00 02 00 04 00 4F EC
read   A5 82 00 04 00 00 01 00 00 88 72

1.1.7  MEAS.CLIP_CLEAR (page 0x09, command 0x12)
write  A5 01 12 00 00 00 09 00 00 E4 BF
read   A5 81 00 00 00 4C 2F

1.1.7  MEAS.CLIP_CLEAR sent as a GET (wrong)
write  A5 02 12 00 00 00 09 04 00 55 BB
read   A5 82 11 00 00 C3 C0
```

**Former write-as-read: save a preset (slot 3).** The slot stays in `wValue`. The status byte `00` becomes status `0x00` of a SET response.

```
1.1.6  PRESET_SAVE 0x90 (write-as-read), slot 3
write  A5 02 90 03 00 02 00 01 00 A3 32
read   A5 82 00 01 00 00 6F 40

1.1.7  STORE.PRESET_SAVE (page 0x02, command 0x10), slot 3
write  A5 01 10 03 00 00 02 00 00 16 E1
read   A5 81 00 00 00 4C 2F
```

**Former write-as-read: load a preset (slot 3).** A slot that does not exist is refused with `VC_ERR_ADDRESS` (`0x13`), and nothing is loaded.

```
1.1.6  PRESET_LOAD 0x91 (write-as-read), slot 3
write  A5 02 91 03 00 02 00 01 00 C2 8A
read   A5 82 00 01 00 00 6F 40

1.1.7  STORE.PRESET_LOAD (page 0x02, command 0x11), slot 3
write  A5 01 11 03 00 00 02 00 00 77 59
read   A5 81 00 00 00 4C 2F

1.1.7  STORE.PRESET_LOAD (page 0x02, command 0x11), slot 10
write  A5 01 11 0A 00 00 02 00 00 95 11
read   A5 81 13 00 00 7F 35
```

**Former write-as-read: save the master volume.**

```
1.1.6  SAVE_MASTER_VOLUME 0xD6 (write-as-read)
write  A5 02 D6 00 00 02 00 01 00 8A 80
read   A5 82 00 01 00 00 6F 40

1.1.7  ROUTE.MASTER_VOLUME_SAVE (page 0x05, command 0x52)
write  A5 01 52 00 00 00 05 00 00 69 17
read   A5 81 00 00 00 4C 2F
```

**Struct read: the sample rate.** Selector 15 answered the rate alone. `SYS.SYSTEM_STATUS` answers 16 bytes, here `clk_sys` 307200000 Hz, rate 48000 Hz (`80 BB 00 00` at offset 4), 1100 mV, 31.50 degrees C and loads of 23 and 0 percent.

```
1.1.6  GET_STATUS 0x50, selector 15
write  A5 02 50 0F 00 02 00 04 00 A0 B0
read   A5 82 00 04 00 80 BB 00 00 8E 96

1.1.7  SYS.SYSTEM_STATUS (page 0x01, command 0x40)
write  A5 02 40 00 00 00 01 10 00 37 57
read   A5 82 00 10 00 00 80 4F 12 80 BB 00 00 4C 04 4E 0C 17 00 00 00 E3 76
```

The two ways to get it wrong. With `wLength` 4 the reply passes a length-4 check but holds `clk_sys`. With the old selector left in `wValue` the request is refused with `VC_ERR_ADDRESS`.

```
1.1.7  SYS.SYSTEM_STATUS with wLength 4 (wrong field)
write  A5 02 40 00 00 00 01 04 00 80 98
read   A5 82 00 04 00 00 80 4F 12 63 51

1.1.7  SYS.SYSTEM_STATUS with wValue 15 (refused)
write  A5 02 40 0F 00 00 01 10 00 34 92
read   A5 82 13 00 00 A3 AE
```

**Struct read: the USB stream detector.** Two 4-byte reads become one 16-byte read. Here 123456 packets, no overruns, a worst gap of 1100 us, alt setting 1 at offset 12 and mounted at offset 13.

```
1.1.6  GET_STATUS 0x50, selector 10
write  A5 02 50 0A 00 02 00 04 00 A1 F3
read   A5 82 00 04 00 40 E2 01 00 44 D6

1.1.6  GET_STATUS 0x50, selector 11
write  A5 02 50 0B 00 02 00 04 00 01 B6
read   A5 82 00 04 00 01 00 00 00 0C 33

1.1.7  SYS.USB_AUDIO_STATUS (page 0x01, command 0x41)
write  A5 02 41 00 00 00 01 10 00 56 EF
read   A5 82 00 10 00 40 E2 01 00 00 00 00 00 4C 04 00 00 01 01 00 00 60 3B
```

**Struct read: the meters.** The reply bytes are identical. Here an RP2040 (21 bytes) with peaks on channels 0 and 1, core 0 load 23 percent, no clips and 2 active input channels.

```
1.1.6  GET_STATUS 0x50, selector 9
write  A5 02 50 09 00 02 00 30 00 10 F4
read   A5 82 00 15 00 B0 04 9C 04 00 00 00 00 00 00 00 00 00 00 17 00 00 00 00 00 02 C0 E5

1.1.7  MEAS.LEVELS (page 0x09, command 0x10)
write  A5 02 10 00 00 00 09 30 00 E7 12
read   A5 82 00 15 00 B0 04 9C 04 00 00 00 00 00 00 00 00 00 00 17 00 00 00 00 00 02 C0 E5
```

**The split input-rate read.** Here the device runs at 48000 Hz and the selected I2S rate is 44100 Hz. The current rate now comes from `SYS.SYSTEM_STATUS` offset 4, the 16-byte read shown above.

```
1.1.6  GET_INPUT_RATE 0xEE
write  A5 02 EE 00 00 02 00 08 00 32 F0
read   A5 82 00 08 00 80 BB 00 00 44 AC 00 00 B3 4A

1.1.7  INPUT.I2S_RX_RATE (page 0x03, command 0x32), the selected rate
write  A5 02 32 00 00 00 03 04 00 62 12
read   A5 82 00 04 00 44 AC 00 00 28 29

1.1.7  SYS.SYSTEM_STATUS (page 0x01, command 0x40), the current rate at offset 4
write  A5 02 40 00 00 00 01 10 00 37 57
read   A5 82 00 10 00 00 80 4F 12 80 BB 00 00 4C 04 4E 0C 17 00 00 00 E3 76
```

**An RP2350-only command on RP2040.** 1.1.6 answered 0 here. 1.1.7 has no such command on RP2040.

```
1.1.7  INPUT.ADAT_RX_ENABLE (page 0x03, command 0x40), on RP2040
write  A5 02 40 00 00 00 03 01 00 15 09
read   A5 82 10 00 00 F3 F7
```

#### 13.1.9 Errors, status and busy handling

**TL;DR.** A refusal is now an answer from a working device, with the reason in the status byte. Today the panel only knows status `0x00` to `0x07` and files everything else as "unexpected response". Teach it the `0x10` and up codes, retry only the three "try again" codes, and stop counting refusals as a lost link.

**The rule.** Status `0x00` is success. `0x01` to `0x07` are transport conditions, and the command did not run. `0x10` and up is the command's own result. Any non-zero status is a failure. Only `0x01`, `0x04` and `0x17` are worth retrying unchanged.

| Status | Name | 1.1.6 meaning | 1.1.7 meaning | Panel action |
|---|---|---|---|---|
| `0x00` | OK | Success | Success. For a deferred command, accepted and queued | Continue |
| `0x01` | BUSY | USB transfer in progress | Same | Retry after a short wait |
| `0x02` | ERROR | Any handler failure | Never sent. Only old firmware sends it, in answer to the probe | Treat as "legacy firmware" during the probe. Anywhere else, a fault |
| `0x03` | BLOCKED | USB-only command | Never sent (`0x16` replaces it) | Fault |
| `0x04` | BULK_LOCKED | Bulk buffer held | Same | Retry after a short wait |
| `0x05` | CRC_ERROR | Frame failed its CRC | Same. The frame was not dispatched | Resend, as today |
| `0x06` | OVERSIZE | Payload too large | Same | Fault. Fix the request |
| `0x07` | FRAME_ERROR | Bad frame | I2C only | Fault |
| `0x10` | `VC_ERR_UNKNOWN` | | No such page or command on this build | "Absent". Expected for ADAT on RP2040. Anywhere else it means a wrong page or number |
| `0x11` | `VC_ERR_DIRECTION` | | No handler for this direction | Bug. For example, a former write-as-read still sent as a GET |
| `0x12` | `VC_ERR_LENGTH` | | SET payload length not accepted | Bug. Send the exact length |
| `0x13` | `VC_ERR_ADDRESS` | | `wValue` out of range, or non-zero on a command that addresses nothing | Bug, or a preset slot above 9 |
| `0x14` | `VC_ERR_VALUE` | | Payload out of range, or a NaN or infinite float | Refusal. Show it. 1.1.6 accepted and ignored these silently |
| `0x15` | `VC_ERR_UNSUPPORTED` | | Valid, but not on this platform | Refusal. ADAT source 3 on RP2040 |
| `0x16` | `VC_ERR_BLOCKED` | | Not allowed on UART | Refusal. No command of this panel is affected |
| `0x17` | `VC_ERR_BUSY` | | The command asks for a retry | Retry after a short wait |
| `0x18` | `VC_ERR_STATE` | | Not allowed in the current state | Refusal. Selecting a disabled optional S/PDIF input or a disabled ADAT input |
| `0x19` | `VC_ERR_PIN_INVALID` | | GPIO not usable | Not reachable, the panel sets no pins |
| `0x1A` | `VC_ERR_PIN_IN_USE` | | GPIO already claimed | Not reachable, the panel sets no pins |

**What to change in the sketch.**

- `dspiResultForStatus` maps a closed set, `0x00` to `0x07`, and turns everything else into `DSPI_TXN_UNEXPECTED`. Add a result such as `DSPI_TXN_REFUSED` for every status from `0x10` up, keep the code itself, and print it in the log. Keep `0x10` distinguishable, because the psychoacoustic bass probe and any RP2040 fallback need "absent".
- `dspiResultCanRetry` retries a timeout, a local CRC error, `0x01`, `0x04` and `0x05`. Add `0x17`. The device already retries `0x01`, `0x04` and `0x17` internally for 50 ms before it answers, so wait 20 to 50 ms before the panel's own retry (today `DSPI_RETRY_DELAY_MS` is 12) and keep the attempt count small.
- `dspiTransactionResult` counts every failure toward "DSPi link lost" (4 in a row). A refusal (`0x10` and up) proves the device is alive. Do not count it, or four refused user actions in a row flip the UI to "Communication fault".
- Treat a refusal as a refusal. Treating every non-zero status as "flash busy" and retrying blindly is slow, and it hides the cause. A `0x13` or `0x14` gives the same answer on every retry.

**Busy windows.** Commands that write flash (`STORE.PRESET_SAVE`, `STORE.PRESET_LOAD`, `ROUTE.MASTER_VOLUME_SAVE` and a `ROUTE.MASTER_VOLUME_MODE` SET) answer at once and do the work a moment later. The flash write then masks interrupts for about 45 ms per 4 KB sector, and UART bytes sent during it can be lost. The panel already waits after these (`PRESET_SAVE_REFRESH_MS` is 350 ms, and the mode and save readbacks start after 110 ms), and that stays right. A timeout right after one of them is expected. The existing recovery quarantine in `dspiRecoverAfterTimeout` (60 to 90 ms) and the 160 ms response timeout can stay.

**`SYS.LAST_RESULT`.** A USB host reads the reason for a refusal from `SYS.LAST_RESULT` (page `0x01`, command `0x14`, 8 bytes, `A5 02 14 00 00 00 01 08 00 BC F6`). On UART the status byte already carries the same code, so the panel never needs it. It can still help when debugging from a serial log. Its bytes are the command ID (u16), the direction (1 GET, 0 SET), the result code, the request's `wValue` (u16) and two zero bytes.

#### 13.1.10 Host-specific hazards

**RP2040 has no ADAT commands.** The `INPUT.ADAT_RX_*` commands (and the upmixer commands, which this panel does not use) exist only on RP2350. On RP2040 they are refused with `0x10`, where 1.1.6 answered 0 and `0xFF`. The sync reads `INPUT.ADAT_RX_ENABLE` and `INPUT.ADAT_RX_PIN` unconditionally, so a mechanical port fails every sync on every RP2040 and shows a permanent "Communication fault". The affected requests are these three.

- `GET_ADAT_INPUT_ENABLE`, now `INPUT.ADAT_RX_ENABLE`, read on every sync.
- `GET_ADAT_INPUT_PIN`, now `INPUT.ADAT_RX_PIN`, read on every sync.
- `GET_ADAT_INPUT_STATUS`, now `INPUT.ADAT_RX_STATUS`, read only while the source is ADAT, which `inputSourceAvailable` already never offers on RP2040.

Gate them on the platform byte the sync reads first (`SYS.PLATFORM` byte 0 is 1 on RP2350). To ask the device instead, read `SYS.PAGE_COMMANDS` for page 3 once after the probe, `A5 02 12 03 00 00 01 40 00 1C 1D`. The 64-byte reply holds the GET bitmap in bytes 0 to 31, and command `n` has a GET when bit `n & 7` of byte `n >> 3` is set. Byte 8 covers commands `0x40` to `0x47`. On RP2350 its low four bits are set (`0x0F`, the four ADAT commands), and on RP2040 the byte is `0x00`.

**The 8-bit narrowing trap.** Any helper that still takes the opcode as `uint8_t` drops the page, so its requests land on page 0 and are refused with `0x10` (13.1.3 lists every signature).

**Number collisions on 1.1.6.** Firmware 1.1.6 ignores `wIndex` and dispatches on `bRequest` alone. A GET frame reaches its old IN handlers, which include the write-as-reads, and a SET frame reaches its old OUT handlers. So if the ported panel ever skips the probe and talks to a 1.1.6 device, each request below runs the old opcode with the same number. The table covers every request of this panel whose number has an old handler in that direction. All other requests, the probe included, are refused by 1.1.6 with status `0x02` and change nothing. The SET rows also answer status `0x00`, so the panel cannot even tell that something went wrong.

| Frame and number | This panel's 1.1.7 requests with that number | 1.1.6 opcode it runs | Effect on a 1.1.6 device |
|---|---|---|---|
| GET 0x25 | `DYN.LEVELLER_GATE` | 0x25 `CS_MACRO_FIRE` | **Dangerous.** Fires Control Surfaces macro 0, whatever the user programmed it to do. The sync sends this read every 1.2 s once it gets past its first request |
| GET 0x51 | `ROUTE.MASTER_VOLUME_MODE` | 0x51 `SAVE_PARAMS` | **Dangerous.** Saves the live state over the active preset (flash write and output restart). The volume limit apply polls this read up to 6 times |
| GET 0x53 | `ROUTE.MASTER_VOLUME_SAVED` | 0x53 `FACTORY_RESET` | **Dangerous.** Factory reset of the live state. The volume limit apply polls this read up to 6 times |
| SET 0x10 | `FX.CROSSFEED_ENABLE`, `INPUT.SOURCE`, `STORE.PRESET_SAVE` | 0x10 `SET_SUBHARM` | **Dangerous.** A 1-byte payload turns the subharmonic synth on (non-zero) or off. The zero-length `STORE.PRESET_SAVE` is ignored but answered 0x00, so the panel reports a save that never happened |
| SET 0x12 | `CLOCK.I2S_CLOCK_MODE`, `FX.CROSSFEED_FREQ`, `MEAS.CLIP_CLEAR` | 0x12 `SET_SUBHARM_LOW` | **Dangerous.** The 4-byte `FX.CROSSFEED_FREQ` sets the subharmonic 24 to 36 Hz band level. The 1-byte and zero-length requests are ignored but answered 0x00 |
| SET 0x30 | `DYN.LOUDNESS_ENABLE` | 0x30 `SET_PSYBASS` | **Dangerous.** Turns psychoacoustic bass on or off |
| SET 0x32 | `DYN.LOUDNESS_INTENSITY`, `INPUT.I2S_RX_RATE` | 0x32 `SET_PSYBASS_CUTOFF` | **Dangerous.** Sets the psychoacoustic bass cutoff from the payload read as a float, clamped to 30 to 300 Hz. A sample rate in Hz reads as a tiny float and lands on 30 Hz |
| SET 0x44 | `FX.PSYBASS_CHARACTER` | 0x44 `SET_PREAMP` | **Dangerous.** Sets the preamp of every input channel to the payload in dB, unclamped. The default character of 50 percent becomes +50 dB of gain |
| SET 0x60 | `ROUTE.USER_VOLUME` | 0x60 `SET_CROSSFEED_PRESET` | **Dangerous.** Sets the crossfeed preset from the first payload byte (the low byte of the float, 0x00 for -20.0 dB) whenever that byte is 3 or less |
| GET 0x11 | `FX.CROSSFEED_PRESET` | 0x11 `GET_SUBHARM` | Read only. Answers unrelated data, which the panel may accept |
| GET 0x13 | `FX.CROSSFEED_FEED`, `STORE.PRESET_NAME` | 0x13 `GET_SUBHARM_LOW` | Read only. Answers unrelated data, which the panel may accept |
| GET 0x15 | `STORE.DIRECTORY` | 0x15 `GET_SUBHARM_HIGH` | Read only. Answers unrelated data, which the panel may accept |
| GET 0x21 | `DYN.LEVELLER_AMOUNT`, `INPUT.SPDIF_RX_CH_STATUS` | 0x21 `GET_CS_GROUP` | Read only. Answers unrelated data, which the panel may accept |
| GET 0x23 | `DYN.LEVELLER_MAX_GAIN` | 0x23 `GET_CS_MACRO` | Read only. Answers unrelated data, which the panel may accept |
| GET 0x31 | `DYN.LOUDNESS_REF` | 0x31 `GET_PSYBASS` | Read only. Answers unrelated data, which the panel may accept |
| GET 0x43 | `FX.PSYBASS_DRIVE`, `INPUT.ADAT_RX_STATUS` | 0x43 `GET_EQ_PARAM` | Read only. Answers unrelated data, which the panel may accept |
| GET 0x45 | `FX.PSYBASS_ORIGINAL` | 0x45 `GET_PREAMP` | Read only. Answers unrelated data, which the panel may accept |
| GET 0x50 | `ROUTE.MASTER_VOLUME` | 0x50 `GET_STATUS` | Read only. Answers unrelated data, which the panel may accept |
| GET 0x61 | `ROUTE.USER_MUTE` | 0x61 `GET_CROSSFEED_PRESET` | Read only. Answers unrelated data, which the panel may accept |
| SET 0x20 | `DYN.LEVELLER_ENABLE` | 0x20 `SET_CS_GROUP` | Payload too short. Nothing is stored, the Control Surfaces status records an invalid value, and the answer is 0x00 |
| SET 0x22 | `DYN.LEVELLER_SPEED` | 0x22 `SET_CS_MACRO` | Payload too short. Nothing is stored, the Control Surfaces status records an invalid value, and the answer is 0x00 |
| SET 0x24 | `DYN.LEVELLER_LOOKAHEAD` | 0x24 `SET_CS_MACRO_STEP` | Payload too short. Nothing is stored, the Control Surfaces status records an invalid value, and the answer is 0x00 |
| SET 0x42 | `FX.PSYBASS_HARMONICS` | 0x42 `SET_EQ_PARAM` | Payload shorter than the 16-byte EQ packet. Nothing changes, and the answer is 0x00 |

With today's sync order, an unprobed sync against 1.1.6 stops at its first request, because 1.1.6 refuses `SYS.PLATFORM` (GET `0x20`) with `0x02`. The leveller gate read is never reached. The Volume Limit apply also stops at its first step, because 1.1.6 refuses the `ROUTE.USER_MUTE` SET (`0x61`), so its preset save and factory reset reads are never reached either. That protection is an accident of ordering. It disappears as soon as the sync order changes, and user actions bypass it entirely. In the seconds after a DSPi is reflashed to 1.1.6, `dspi.connected` is still true, so changing the psychoacoustic bass character would set +50 dB of preamp gain on every input. This is why the probe is mandatory and why the protocol state must return to unknown after every timeout.

**Timeout retries of non-idempotent requests.** `DSPI_TRANSACTION_ATTEMPTS` is 2, so `dspiTransactionResult` resends a request after a timeout. A SET response carries no request ID, so a lost response to `STORE.PRESET_SAVE`, `STORE.PRESET_LOAD`, `ROUTE.MASTER_VOLUME_SAVE` or a `ROUTE.MASTER_VOLUME_MODE` SET leads to a second flash write and a second output restart. The same happened in 1.1.6 with the GET-frame versions, but 1.1.7 SETs no longer return a status byte that could tell the two apart. Do not auto-retry these four on a timeout. Confirm them by their readback instead, as `loadPreset`, `servicePresetSaveCompletion` and the Volume Limit code already do (`STORE.PRESET_ACTIVE`, `STORE.DIRECTORY`, `ROUTE.MASTER_VOLUME_SAVED`, `ROUTE.MASTER_VOLUME_MODE`). A resend after status `0x05` (CRC error) is safe, because that frame never ran.

#### 13.1.11 Porting order and tests

**Suggested order.**

1. Start from `spdif4-compat`, or apply `de6573c` to your tree (13.1.2).
2. Change the transport. Introduce the command type, put the page in `wIdxH`, and widen or replace every helper signature at once (13.1.3). Unit-test the frame builder against the 1.1.7 column of the frames table in 13.1.6 before touching any device.
3. Decode the new status codes, add `0x17` to the retry set, and stop counting refusals as link loss (13.1.9).
4. Add the probe, the protocol state, the gate in `dspiTransactionResult` and the state resets (13.1.5). Decide what the panel does on 1.1.6.
5. Renumber the `same` rows of 13.1.6.
6. Port the non-mechanical rows. They are `SYS.PLATFORM` with 7 bytes and the version display, the RP2040 ADAT gate, the psychoacoustic bass probe on `0x10`, `SYS.SYSTEM_STATUS` for the sample rate, one `SYS.USB_AUDIO_STATUS` read for the stream detector, `wValue` 0 for `MEAS.LEVELS`, the two-read `getDspiInputRates`, and the four zero-length SETs (13.1.7).
7. Stop auto-retrying the four flash-writing SETs on a timeout (13.1.10).
8. Fix the log lines to print the 16-bit ID and the status.

**Tests against a 1.1.7 device.** Run them on an RP2350 and on an RP2040.

- [ ] The probe frame matches 13.1.5 byte for byte, and the reply passes the layout A checks.
- [ ] A full sync passes and the panel shows the right platform and "DSPi FW 1.1.7 beta 1". On RP2040, a serial log or a logic analyser shows no request to page `0x03` commands `0x40`, `0x41` or `0x43`.
- [ ] The status screen shows the real sample rate (for example 48000), not `clk_sys`.
- [ ] USB playback on and off flips the meter stream detector. Closing the stream (alt setting 0) clears the meters at once.
- [ ] Meters move, and an output clip lights and then clears. `MEAS.LEVELS` answers 21 bytes on RP2040 and 41 on RP2350.
- [ ] Every setter writes and reads back. That covers volume, mute, master volume, source, loudness, crossfeed, leveller and psychoacoustic bass.
- [ ] Preset load and save work, including the guarded load sequence and the save completion refresh.
- [ ] System > Volume Limit applies, saves and survives a power cycle of the DSPi.
- [ ] The SD media route switches DSPi to I2S at the file's rate and restores the previous source, rate and clock mode afterwards.
- [ ] Provoke refusals and confirm the log shows the code and the link stays up. `STORE.PRESET_LOAD` of slot 10 answers `0x13`. `INPUT.SOURCE` with value 7 answers `0x14`. Selecting a disabled optional S/PDIF input answers `0x18`.
- [ ] Power-cycle or reflash the DSPi while the panel runs. The panel re-probes before it sends anything else.
- [ ] With UART notifications enabled in DSPi Console, a preset load from the Console still triggers the panel's refresh.

**Tests against a 1.1.6 device.**

- [ ] The probe is answered `A5 82 02 00 00 F0 DA`. The panel then sends nothing paged, only the old `GET_PLATFORM` if it shows the version, or only its 1.1.6 request set if it keeps the fallback.
- [ ] Nothing on the device changes. Check in DSPi Console that the presets, the subharmonic synth, psychoacoustic bass, the crossfeed preset and the preamps are as before.

### 13.2 DSPi Console for macOS

#### 13.2.1 At a glance

**TL;DR.** DSPi Console is the native macOS control app, and it is the largest of the four ports. Every request it sends today has a zero page, so a 1.1.6-era Console finds a 1.1.7 device on USB, cannot read its version and shows it as not connected. Most of the port is renumbering behind one widened transport call. The real work is in six places. They are the probe with a legacy mode for old firmware, a SET path that reports refusals, a band writer that survives `VC_ERR_BUSY`, struct reads in the statistics window, a few reshaped payloads, and re-keying the embedded DSPi Link hub on (page, command).

- **Repository.** `/Users/weeblabs/DSPi Console`, audited on branch `release/v1.1.6` at commit `5ec256c`. App sources live in `DSPi Console/`, tests in `DSPi ConsoleTests/`. The app declares `MARKETING_VERSION = 1.1.6` and bundles the 1.1.6 UF2 pair.
- **Transport.** USB vendor control transfers through IOKit (`IOUSBDeviceInterface500::DeviceRequest`) on the device handle. An embedded DSPi Link hub (SwiftNIO, WebSocket) carries the same requests for remote clients, and the local UI also goes through it. Notifications arrive on bulk endpoint `0x83` and do not change.
- **Size.** 286 transfer call sites send 175 live opcodes. 130 of them only need a renumber from section 6.2. 45 need per-site work (13.2.5). 47 more opcodes are sent only from functions nothing calls, and they can be deleted instead of ported.

#### 13.2.2 Before you start

- Port from `release/v1.1.6`. The branches `room-correction`, `room_correction_poles`, `room_correction_scheduled_fit` and `zero2w` add protocol reads of their own and need the same port when they are rebased. The `overclock` branch sends 0x40 and 0x41 for the system clock, which have no command in 1.1.7 beta 1, so it cannot be ported yet.
- Vendor a copy of `Documentation/protocol/dspi_commands.json` from the firmware tag you pair with, and generate the command constants from it (13.2.5). Never hand-copy numbers again.
- Coordinate the Link changes (13.2.8) with the DSPi Link appendix (13.3). The Console embeds the same `commands.json` policy table and frame format as the other hubs and clients.
- These stay exactly as they are. The bulk blob parser (`fetchAllParams`, `WireBulkParams` version 32, 6136 bytes), the hub's `SnapshotCache`, the notification path (`USBNotificationReader`, `applyNotifiedParamChange`, `InterruptMonitor`), `FirmwareVersion.fromPlatformReply` (`SYS.PLATFORM` has the same 7 bytes), CoreAudio streaming, and the Link frame wire format, which already carries a 16-bit `wIndex` (`Link/LinkFrames.swift`).
- The "USB Protocol" section of the Console's own `CLAUDE.md` still describes 1.1.6 opcodes. Rewrite it from this guide once the port lands.

#### 13.2.3 Transport and choke points

Every vendor request passes through one of these places.

| Choke point | File | What it does today |
|---|---|---|
| The `DeviceTransport` protocol with `sendControlRequest(request: UInt8, ...)`, `getControlResult(request: UInt8, ...)` and the `getControlRequest` extension, which returns `nil` on any failure | `DSPi Console/Link/DeviceTransport.swift` | The API every call site uses. Implemented by `USBDevice`, `HubTransport`, `NetworkTransport`, `CompositeTransport` and the test fakes |
| `USBDevice.sendControlRequest` / `getControlResult` / `sendControlResult` | `DSPi Console/USBDevice.swift` | The only code that touches IOKit. `bmRequestType` is fixed inside (`0x41` SET, `0xC1` GET). `sendControlRequest` runs async and throws the `IOReturn` away. `sendControlResult` returns it and is used only by the hub |
| `HubTransport` | `DSPi Console/Link/HubTransport.swift` | Wraps each call in a `LinkCmdRequest` for the `LinkHub` router. Production wiring (`DSPi_ConsoleApp.swift`) sends every local request this way. Its SET completion looks only for `.locked` |
| `USBHubDevice.execute` and `readInfo` | `DSPi Console/Link/USBHubDevice.swift` | Executes tunnelled requests with `request.bRequest` and `request.wIndex`. `readInfo` is the first code to talk to a new device |
| Hub-originated frames | `DSPi Console/Link/LinkHub.swift` (`writeBoardName`, `warmSnapshot`) | Build `LinkCmdRequest`s for the device name and the bulk read directly, with `wIndex: 2` |
| Poll subscriptions | `DSPi Console/Link/PollScheduler.swift` | `PollSpec` carries `req: UInt8`, `wValue`, `wIndex` and `wLength` from the subscriber and replays them |

**The wIndex mix.** Call sites pass `wIndex` themselves, and there is no rule behind the two values. Newer features use 2 and older ones 0, and `GET_STATUS` uses both (selectors 17 to 22 with 2, the rest with 0).

| File | `index: 0` | `index: 2` |
|---|---|---|
| `DSPi Console/Commands.swift` | 112 | 125 |
| `DSPi Console/StatsView.swift` | 14 | 14 |
| `DSPi Console/SpectrumAnalyser.swift` | 0 | 10 |
| `DSPi Console/FirmwareUpdateView.swift` | 0 | 1 |
| `DSPi Console/Link/USBHubDevice.swift` | 0 | 3 |
| `DSPi Console/Link/LinkHub.swift` (frames, not transport calls) | 0 | 2 |

Both values put zero in the high byte, so on 1.1.7 every one lands on page 0 (11.1). Do not fix the sites one by one. Change `request: UInt8` in `DeviceTransport` and every implementation to the 16-bit command ID `(page << 8) | command` (or a small `(page, cmd)` struct), let `USBDevice` build `wIndex = (id & 0xFF00) | 2` and `bRequest = id & 0xFF`, and delete the `index:` argument from all 286 sites. The compiler then finds every site that still passes an old constant.

**The parameter-width trap.** Any helper that keeps an 8-bit opcode parameter silently drops the page, because the page has nowhere to go. Widen each of these with the transport, or the build stays green while the requests go to page 0.

- `sendFloatParamToDevice(_ request: UInt8, _ value: Float, wValue: UInt16 = 0, wIndex: UInt16 = 0)` in `Commands.swift`, which the drag handlers of `SubharmonicSynthView`, `PsychoacousticBassView`, `VolumeLevellerView`, `LoudnessView` and `CrossfeedView` call with old constants. Its `wIndex: 0` default is page 0 in its own right, so remove the parameter.
- `sendLimiterParam(output: UInt8, _ index: UInt8, _ value: Float)`, `sendTubeParam(_ index: UInt16, ...)` and `sendUpmixParam(_ paramId: UInt16, ...)`. The index becomes a command (6.4), so take a generated command constant, not an index.
- `LinkCmdRequest.bRequest: UInt8`, `PollSpec.req: UInt8` and `CommandRouter.WriterEntry.code: UInt8`. These still travel next to a 16-bit `wIndex`, so the wire works, but every lookup keyed on them is wrong (13.2.8).

**STALL handling at the choke point.** Today a STALL becomes `LinkStatus.error` on a GET, vanishes on a plain USB SET, and is dropped by `HubTransport` on a hub SET. Give `DeviceTransport` one SET that returns the outcome, and make every write go through it. In `USBDevice`, read `SYS.LAST_RESULT` (8 bytes) on the same `serialQueue` block straight after a STALL, so no other Console request can land in between. Trust it only when its command ID, direction and `wValue` match the request, because other USB programs on the same device share the record (5.2). Then return the result code to the caller. `LinkStatus` cannot carry it yet (13.2.8). Also replace the plain `DeviceRequest` with `DeviceRequestTO` and a completion timeout of about 1.5 s (5.8), since the IOKit default is not set anywhere in the code.

#### 13.2.4 Probe and identification

**Where it goes.** The first requests a new device sees are the hub's. `LinkHub.usbDidConnect` calls `USBHubDevice.readInfo`, retrying with `identityRetryDelays` (0 to 1 s back-off, six tries), and `readInfo` sends `GET_SERIAL` before `GET_PLATFORM`. Put the probe first in `readInfo`, and store its result in `HubDeviceInfo` as a protocol state (paged or legacy). Then gate everything else on that state. That covers the hub's `warmSnapshot`, router attach, `DSPViewModel.fetchAll(afterConnect:)` with `waitForDeviceReady` and the 60 ms meter timer, `StatsViewModel.fetchDeviceInfo` and its timers, and the spectrum analyser.

The gate matters for safety, not just tidiness. Two GETs a ported Console sends routinely are write-as-reads on 1.1.6 (4.1). `INPUT.LG_SOUND_SYNC_STATUS` (`0x0351`, read every 2 s by the statistics window) runs `SAVE_PARAMS`, a flash write, and `MEAS.RTA_STATUS` (`0x0925`, read by the analyser) fires Control Surfaces macro 0.

**The request and the reply.** Treat only a STALL (`LinkStatus.error`) as old firmware. On a timeout keep using the existing retry delays.

```
1.1.7  SYS.PROTOCOL_INFO (page 0x01, command 0x10)
SETUP  C1 10 00 00 02 01 10 00
IN     44 53 50 69 01 00 0C 00 40 00 00 01 00 00 00 00
```

Accept the device only when the reply starts `44 53 50 69` and byte 4 is 1. With any other major, publish the device as needing a newer Console and send nothing else. On success read `SYS.PLATFORM` (`C1 20 00 00 02 01 07 00`, 7 bytes) into the unchanged `FirmwareVersion.fromPlatformReply`, then `SYS.SERIAL` (16) and `SYS.DEVICE_NAME` (32), then `SYS.PAGE_LIST` and `SYS.PAGE_COMMANDS` (64) for each page the Console uses (4.7).

**On 1.1.6 firmware.** A STALL means 1.1.6 or older. Give the device a legacy state that allows exactly two requests, both with `wIndex` 2 as the Console sends them today (1.1.6 ignores `wIndex`, so 0 would also work).

```
1.1.6  GET_PLATFORM 0x7F (legacy state only)
SETUP  C1 7F 00 00 02 00 07 00
IN     7 bytes, the SYS.PLATFORM layout (a shorter reply on older firmware)

1.1.6  ENTER_BOOTLOADER 0xF0 (legacy state only, write-as-read)
SETUP  C1 F0 00 00 02 00 01 00
IN     01, then the board reboots into the RPI-RP2 drive
```

The update flow needs four changes to use this state.

1. Publish the legacy device's version so `firmwareMatch` yields `.deviceOlder`. Today it returns `nil` unless `isDeviceConnected`, which a legacy device must never set, because that would start `fetchAll` and the pollers. The banner then reads "This device runs firmware 1.1.6; DSPi Console expects 1.1.7 beta 1."
2. Let `FirmwareUpdateView.start()` and `enterBootloaderOnly()` act on a legacy device. Both are guarded by `vm.isDeviceConnected` today.
3. Make `enterBootloaderForInstall` send `SYS.ENTER_BOOTLOADER` (`41 30 00 00 02 01 00 00`) to a paged device. Send the old IN request above to a legacy one straight through `USBDevice`, because no router is attached to carry it. The device lock it takes is already a no-op in that case.
4. Keep `ViewModelFirmwareVerifier` as it is. After the flash the board comes back paged, the probe passes and `SYS.PLATFORM` supplies the version it waits for.

A remote hub that meets a legacy device has to publish the same state to its clients. That is a Link protocol question (13.3).

**What a 1.1.6-era Console shows on 1.1.7 firmware.** This comes from the code, not from a run. `readInfo` gets a STALL for `GET_SERIAL` (it falls back to the IOKit serial) and for `GET_PLATFORM`. The hub retries six times over about 2.5 s and gives up. The hub never attaches a router, so `isDeviceConnected` stays false.

- The device name appears in the picker with a red status dot whose tooltip reads "Not connected. Right-click the device name to retry." Retrying repeats the same failure. The device controls stay dimmed, and no firmware version or mismatch banner appears.
- The Firmware Update window shows the "Connect a board" card ("No device is connected. Hold the BOOTSEL button while plugging a board in, and it will appear here.").
- "Update Firmware" arms the installer but sends no bootloader request, because the button is guarded by `isDeviceConnected`. A request at page 0 would be refused anyway (4.3).
- If the user holds BOOTSEL, the old Console writes its bundled 1.1.6 image. That is a downgrade, and verification passes against 1.1.6.

So the user must update the Console first, or hold BOOTSEL and accept the downgrade. The in-app updater (`AppUpdateChecker`) does not depend on the device, so it can still offer the paired Console release. Say this in the Console's release notes.

#### 13.2.5 Requests to port

**Generate the constants.** Today `DSPi Console/Constants.swift` holds 238 hand-written `let REQ_NAME: UInt8 = 0xNN` lines. Four of them are spelled differently from the firmware (`REQ_SET_LEVELLER`, `REQ_GET_LEVELLER`, `REQ_SET_LEVELLER_MAXGAIN`, `REQ_GET_LEVELLER_MAXGAIN`). Replace them with a Swift file generated from the vendored registry by a small script in `Tools/`. Emit one constant per command with its 16-bit ID, GET length (or variable), SET payload bounds, flags and build condition, named after `PAGE.NAME`. Add a unit test that re-runs the generator and fails on any difference, so a hand edit or a stale copy cannot pass. Refresh the vendored registry in the same release step that downloads the UF2 pair, from the same firmware tag. The selector constants that only existed to pick an operation (`OUTPUT_LATENCY_GET_*`, `LIMITER_PARAM_*`, `TUBE_PARAM_*`, `UPMIX_PARAM_*`, `CS_CAPS_ALL`, `SIGGEN_CAPS_HEADER`, `CS_IR_LEARN_*`, `SIGGEN_CTL_*`, `RTA_CTL_*`) go away with the splits.

**Mechanical renumbers.** All of these are "renumber and set the page, `wValue` and payload unchanged" (section 6.2), unless 13.2.6 or 13.2.7 says otherwise.

| Page | Live opcodes | Mechanical | Per-site work |
|---|---|---|---|
| `SYS` | 8 | 6 | 2 |
| `STORE` | 12 | 7 | 5 |
| `INPUT` | 22 | 15 | 7 |
| `OUTPUT` | 11 | 5 | 6 |
| `ROUTE` | 15 | 13 | 2 |
| `FILTER` | 3 | 2 | 1 |
| `DYN` | 12 | 11 | 1 |
| `FX` | 32 | 30 | 2 |
| `MEAS` | 14 | 9 | 5 |
| `CS` | 26 | 19 | 7 |
| `XPORT` | 5 | 5 | 0 |
| `CLOCK` | 13 | 8 | 5 |
| Several pages (`GET_STATUS`, `GET_INPUT_RATE`) | 2 | 0 | 2 |
| **Total** | **175** | **130** | **45** |

**Per-site work, exhaustively.** The functions are in `Commands.swift` unless another file is named.

| Old opcode | Console function | New command(s) | What changes |
|---|---|---|---|
| `SET_EQ_PARAM` 0x42 | `setFilter`, `sendFilterToDevice`, `eqParamPacket`, `setCrossoverBand` | `FILTER.BAND` | Reshaped payload and the busy rule (13.2.6, 13.2.7) |
| `SET_MATRIX_ROUTE` 0x70 | `setMatrixRoute` | `ROUTE.MATRIX_ROUTE` | Reshaped payload (13.2.6) |
| `GET_INPUT_RATE` 0xEE | `fetchInputRate` | `INPUT.I2S_RX_RATE` | 4-byte reply (13.2.6) |
| `GET_CS_DISPLAY_CFG` 0x28 | `fetchCsDisplayCfg` | `CS.DISPLAY_CAPS`, `CS.DISPLAY_CONFIG` | Two reads (13.2.6) |
| `GET_STATUS` 0x50 | `fetchStatus`, `fetchSampleRate`, `StatsView.swift` | `MEAS.LEVELS` and five `SYS` structs | 13.2.6 |
| `OUTPUT_LATENCY` 0x03 IN | `fetchOutputLatency`, `setOutputLatency` (status poll), `StatsView.fetchRingHealth` | `OUTPUT.LATENCY_STATUS` (`wValue` 0 was status), `OUTPUT.LATENCY_OPTIONS` (`wValue` 1 was options) | The selector becomes the command, `wValue` 0. The OUT direction is a plain renumber to `OUTPUT.LATENCY` |
| `LIMITER` 0x81 | `sendLimiterParam`, `pollLimiter` | `DYN.LIMITER_ENABLE`, `_THRESHOLD`, `_RELEASE`, `_LINK_GROUP`, `DYN.LIMITER_METERS` | `wValue` keeps only the output, `0xFF` on SETs only. Meters at `wValue` 0 (6.4) |
| `SET_TUBE_PARAM` 0x3E | `sendTubeParam` (14 setters) | `FX.TUBE_ENABLE` to `FX.TUBE_TRIM` | One command per index, `wValue` 0 (6.4) |
| `UPMIX_SET_PARAM` 0x4C | `sendUpmixParam` (15 setters) | `FX.UPMIX_ENABLE` to `FX.UPMIX_PRESENCE` | As tube. RP2350 only |
| `RTA_GET_CAPS` 0x0A | `SpectrumAnalyser.fetchCaps` | `MEAS.RTA_CAPS`, `MEAS.RTA_BAND_CENTRES` | Centre chunk `n` (1 to 15) becomes `wValue = n - 1` |
| `RTA_CONTROL` 0x0E | `SpectrumAnalyser.release` (STOP) | `MEAS.RTA_STOP` | Zero-length SET |
| `GET_CS_CAPS` 0x86 | `fetchControlSurfaces` | `CS.CAPS` (`0xFFFF` header), `CS.NOUN_DESC` | Header at `wValue` 0 |
| `SIGGEN_GET_CAPS` 0xA8 | `fetchSiggen` | `MEAS.SIGGEN_CAPS` (`0xFFFF` header), `MEAS.SIGGEN_TYPE_DESC` | Header at `wValue` 0 |
| `GET_CS_AUX_STATE` 0x05 | `fetchCsAuxValues` | `CS.AUX_ALL` | `0xFFFF` becomes `wValue` 0 |
| `SIGGEN_CONTROL` 0xA6 | `siggenControl` | `MEAS.SIGGEN_STOP`, `MEAS.SIGGEN_START`, `MEAS.SIGGEN_STOP_NOW` | Action 0, 1, 2 becomes the command. ACK replaces `d.first == 1` |
| `CS_MACRO_FIRE` 0x25 | `csMacroFire` | `CS.MACRO_FIRE`, `CS.MACRO_CANCEL` (`0xFFFF`) | Zero-length SETs, no status byte |
| `CS_IR_LEARN` 0x8F | `csIrLearnArm`, `csIrLearnCancel`, `csIrLearnRead` | `CS.IR_LEARN_ARM`, `CS.IR_LEARN_CANCEL`, `CS.IR_LEARN_STATUS` | Arm and cancel are zero-length SETs. Read is an 8-byte GET at `wValue` 0 |
| `CLEAR_CLIPS` 0x83 | `clearClips` | `MEAS.CLIP_CLEAR` | Zero-length SET. The flags already arrive in `MEAS.LEVELS` |
| Pin family, 17 write-as-reads | `setOutputPin`, `setOutputSlotType`, `setI2SBckPin`, `setI2SClockPinMode`, `setMckEnable`, `setMckPin`, `setMckMultiplier`, `setAdatEnable`, `setAdatPin`, `setAdatInputEnable`, `setAdatInputPin`, `setAdatInputClockMode`, `setSpdifRxPin`, `setSpdifInputEnable`, `setI2SRxPin`, `setI2SInputChannels`, `testDacHwMute` | Section 6.6 | Address stays in `wValue`, value moves to a 1-byte payload (`testDacHwMute` is zero-length). Status byte becomes ACK or STALL (13.2.7) |
| Preset and flash family | `savePresetUnlocked`, `loadPreset`, `deletePreset`, `saveOutputConfigResult`, `saveMasterVolume`, `factoryReset` | `STORE.PRESET_SAVE`, `STORE.PRESET_LOAD`, `STORE.PRESET_DELETE`, `STORE.OUTPUT_CONFIG_SAVE`, `ROUTE.MASTER_VOLUME_SAVE`, `STORE.FACTORY_RESET` | Zero-length SETs, slot in `wValue` for the preset three (13.2.7) |
| Acknowledge family | `csSave`, `csRevert`, `StatsView.resetBufferWatermarks`, `enterBootloaderForInstall`, `FirmwareUpdateView.enterBootloaderOnly` | `CS.SAVE`, `CS.REVERT`, `SYS.STATS_RESET`, `SYS.ENTER_BOOTLOADER` | Zero-length SETs, except `SYS.STATS_RESET`, which takes the mask `01` as a 1-byte payload. Bootloader entry keeps the legacy branch (13.2.4) |

**Delete instead of porting.** A static search of the app target finds 49 functions in `Commands.swift` and `resetAveraging` in `SpectrumAnalyser.swift` that nothing calls. They are `sendPreampToDevice`, `setPreamp`, `fetchPreamp`, `fetchPreampChannel`, `fetchFilter`, `fetchBandBypass`, `fetchDelay`, `fetchBypass`, `fetchMasterVolume`, `fetchMasterVolumeMode`, `fetchSavedMasterVolume`, `fetchLoudness`, `fetchLoudnessRef`, `fetchLoudnessIntensity`, `fetchLoudnessMask`, the six `fetchCrossfeed*` readers, the seven `fetchPsybass*` readers, eleven `fetchSubharm*` readers (all but solo, headroom and meter), `fetchTubeParam`, `fetchLimiterParam`, `fetchMatrixRoute`, `fetchOutputGainDB`, `fetchOutputMute`, `fetchOutputDelay`, `checkCore1Conflict`, `fetchPresetStartup`, `fetchOutputConfigMode` and `saveParams`. Only `saveParams` has callers, in `DSPi ConsoleTests/LinkLockUsageTests.swift`, so move that test to `STORE.PRESET_SAVE`. Deleting them retires `SET_PREAMP` and `SAVE_PARAMS` (6.7) without a replacement. The 16 constants that are defined but never sent go with the generated file.

#### 13.2.6 Replies and checks that change

**Reshaped payloads.** The Console's 1.1.6 lengths are refused with `VC_ERR_LENGTH`.

- `FILTER.BAND` takes `wValue = (channel << 8) | band` and exactly 16 bytes, `u8 type, u8 bypass, u16 qp_x512, f32 freq, f32 Q, f32 gain`. `eqParamPacket` sends channel and band in the payload and 18 bytes for a Linkwitz Transform. `setCrossoverBand` builds its own 16-byte packet in the old order. Build both from one encoder and always send the Qp (0 for crossover bands). Bands 20 to 23 stay the crossover bands.

```
1.1.6  SET_EQ_PARAM 0x42, channel 2, band 3, as the Console sends it
SETUP  41 42 00 00 00 00 10 00
OUT    02 03 type bypass freq(f32) Q(f32) gain(f32)

1.1.7  FILTER.BAND (page 0x06, command 0x10), channel 2, band 3
SETUP  41 10 03 02 02 06 10 00
OUT    type bypass qp(u16) freq(f32) Q(f32) gain(f32)
```

- `ROUTE.MATRIX_ROUTE` takes `wValue = (input << 8) | output` and exactly 8 bytes, `u8 enabled, u8 invert, u16 0, f32 gain`. `setMatrixRoute` builds `Data(count: 9)` with input and output in bytes 0 and 1.
- `STORE.PRESET_NAME` and `ROUTE.CHANNEL_NAME` take 1 to 32 bytes. `setPresetName` sends `name.prefix(31).utf8` plus a NUL, and `setChannelName` grows its 32-byte buffer, so a name of 31 non-ASCII characters exceeds 32 bytes. Truncate on UTF-8 byte count without splitting a character.

**Reply checks that fail or misread.**

| Function | Check today | New reply | Fix |
|---|---|---|---|
| `fetchInputRate` | `data.count >= 8`, u32 at offset 4 | `INPUT.I2S_RX_RATE`, 4 bytes, selected rate at offset 0 | Check 4, read offset 0. The live rate is `SYS.SYSTEM_STATUS` offset 4, which the Console already polls |
| `fetchCsDisplayCfg` | `CS_DISPLAY_CFG_GET_LEN` (16), limits at bytes 0 and 1, config at offset 4 | `CS.DISPLAY_CAPS`, 4 bytes (page count, model count), and `CS.DISPLAY_CONFIG`, 12 bytes | Two reads. `CsDisplayCfg.fromData` at offset 0 of the second |
| `StatsView.readStats`, selector 14 | `load(as: UInt32.self)`, no length check | `SYS.SYSTEM_STATUS` offset 8 is a u16 | Reading 4 bytes there would put the temperature in the high half |
| `StatsView.readStats`, selector 16 | `load(as: Int32.self)` | `SYS.SYSTEM_STATUS` offset 10 is an i16 | Sign-extend 2 bytes |
| `fetchPresetStartup` (dead) | `count >= 3`, last active slot at byte 2 | `STORE.STARTUP`, 2 bytes | Delete. The last active slot is byte 4 of `STORE.DIRECTORY` (7 bytes) |
| `SpectrumAnalyser.fetchCaps` centre loop | stops on a short chunk or a STALL | `MEAS.RTA_BAND_CENTRES`, 0-based index | A chunk past the table is `VC_ERR_ADDRESS`, which the loop already treats as the end |

These checks still hold and need no change. They are the exact 32 of `SYS.DEVICE_NAME` (`USBHubDevice.readInfo`, `fetchDeviceName`) and `INPUT.SOURCE_NAME`, `STORE.ALL_PARAMS` at `count >= 6136` with byte 0 equal to 32, `MEAS.LEVELS` at `2N + 7` (21 on RP2040, 41 on RP2350), byte 0 equal to 3 in the analyser's config, status and bin frames, `CS.STATUS` at 41, `CS.CAPS` at 52 (the Console asks for 64 and parses by type count), `SYS.BUFFER_STATS` at 44, `OUTPUT.LATENCY_STATUS` at 12 and `INPUT.SPDIF_RX_CONFIG` at 6. `MEAS.RTA_BINS` answers at most 256 bytes per request, which `readBinFrame` already handles by reading on from the returned offset.

**Status polling moves to the struct commands.** The view model reads selector 9 every 60 ms while connected, and selector 15 every 16th tick (about 1 s). Once the System Statistics window has been opened, `StatsViewModel` keeps polling for the rest of the session, with the rows below every 2 s and `SYS.BUFFER_STATS` every 60 ms.

| Selector | Console field and caller | Read now | Offset |
|---|---|---|---|
| 9 | meters, `fetchStatus`, 60 ms | `MEAS.LEVELS` | whole reply, same bytes |
| 15 | `sampleRateHz`, `fetchSampleRate`, about 1 s, and `StatsView` | `SYS.SYSTEM_STATUS` | 4 (u32) |
| 13 | `systemClockHz` | `SYS.SYSTEM_STATUS` | 0 (u32) |
| 14 | `coreVoltageMillivolts` | `SYS.SYSTEM_STATUS` | 8 (u16) |
| 16 | `systemTempCentiC` | `SYS.SYSTEM_STATUS` | 10 (i16) |
| 3, 4, 5, 6 | PDM ring and DMA counters | `SYS.PDM_STATS` | 0, 4, 8, 12 |
| 7 | `spdifOverruns` | Retired, always 0 | Drop the row |
| 8 | `spdifUnderruns` (USB packet gaps of 2 to 50 ms) | `SYS.SPDIF_TX_STATS` | 0 |
| 17 | starvation total, `fetchStarvationStats` | `SYS.SPDIF_TX_STATS` | 4 |
| 18 to 21 | per-slot starvations, read only when the total is non-zero | `SYS.SPDIF_TX_STATS` | 8, 12, 16, 20 |
| 22 | `usbRingOverruns` | `SYS.USB_AUDIO_STATUS` | 4 |
| 26, 27, 28, 29, 31 | ring health, `fetchRingHealth` | `SYS.PIPELINE_STATS` | 0, 4, 8, 12, 16 (`0xFFFFFFFF` at 8 still means no sample) |
| 30 | worst USB packet gap | `SYS.USB_AUDIO_STATUS` | 8 |

The 2 s block shrinks from about 15 GETs to five struct reads (`SYS.SYSTEM_STATUS` 16 bytes, `SYS.PDM_STATS` 16, `SYS.SPDIF_TX_STATS` 24, `SYS.USB_AUDIO_STATUS` 16, `SYS.PIPELINE_STATS` 24). Check each length before decoding. Today none of the `readStats` reads checks one, so an empty reply would trap.

#### 13.2.7 Errors, status and busy handling

**Write-as-read status bytes.** 36 call sites send a state change as an IN transfer and read a status byte. With the SET path of 13.2.3 they become SETs that ACK or STALL with a result code.

- The 17 pin-family setters return a `PIN_CONFIG_*` byte that about 60 references in `DSPi_ConsoleApp.swift` (Settings) and 8 in `PresetDocumentTransfer.swift` turn into messages. The least disruptive port keeps their `UInt8` return and maps the result code back through `status_families.pin` in `legacy_map.json`, including its per-command overrides (5.4, 6.6). Move the UI to result codes later. `assignOutputPin` retries after disabling PDM when the status is `PIN_CONFIG_OUTPUT_ACTIVE` (0x04), which is now `VC_ERR_STATE`.
- `savePresetUnlocked`, `loadPreset` and `deletePreset` compare with `PRESET_OK`. An ACK is success, `VC_ERR_ADDRESS` replaces 0x01 and `VC_ERR_STATE` replaces 0x02. Loading an empty slot is not an error. `saveOutputConfigResult`, `saveMasterVolume` and `factoryReset` treat byte 0 as success, and an ACK replaces it.
- `siggenControl` checks `d.first == 1`. `csMacroFire` returns the byte as a CS status, so return `VC_ERR_ADDRESS` (bad macro) and `VC_ERR_BUSY` (revert running) instead. `csIrLearnArm` treats any reply as armed. Now `VC_ERR_STATE` means no receiver and `VC_ERR_BUSY` means a revert is running, which both read as `nil` today.

**Writes that were silent and now STALL.** The Console publishes SET values optimistically, so log or surface these refusals once the SET path returns them (5.5). They are an output enable that collides with the core 1 mode (`VC_ERR_STATE`), `INPUT.SOURCE` to a disabled input, any NaN or infinite float, a crossfeed preset above 3, a leveller speed of 3 or more, and a `STORE.STARTUP` mode or slot out of range.

**XPORT read-back.** `setUartCtrlConfig` and `setI2cCtrlConfig` send the config, sleep 250 ms and report the last-status byte of `XPORT.STATUS`. 1.1.7 refuses a bad scalar or pin range at once with `VC_ERR_VALUE` or `VC_ERR_PIN_INVALID` and leaves `XPORT.STATUS` alone, so the Console would report the previous apply's outcome. Check the SET's own result first, and read `XPORT.STATUS` only after an ACK.

**The FILTER.BAND busy rule.** This is the one behavioural change that a straight renumber turns into wrong device state.

- *The rule.* `FILTER.BAND` and `FILTER.BAND_BYPASS` share one pending slot for the whole device. The handler stores the band and returns `VC_OK`, and the firmware's main loop takes it, recomputes the coefficients and frees the slot on its next pass, normally within a millisecond or two. Longer main-loop work, such as a flash write or an output restart, holds the slot longer. A band write that arrives while the slot is full is refused with `VC_ERR_BUSY` and changes nothing. 1.1.6 instead overwrote the pending write, so the last write always won (`Documentation/commands.md` section 11.1, and 5.8 and 6.5 here).
- *Why the Console hits it.* An ACK means "queued", not "applied", and the Console sends band writes back to back, fire and forget. The graph editor (`PeqGraphEditor.sendToDevice`) already coalesces per band and sends at most 30 times a second. Each tick, though, it sends every dirty band in one burst, and `sendGraphBandsToDevice` adds a second write per band for a linked input pair. The band list sends one `setFilter` per edit, also mirrored. Clear All (`clearPEQBands`, `clearChannelPEQ`) sends 10, `clearCrossoverBands` 4, `syncInputPair` 10, `pasteChannelParams` up to 20, and AutoEQ and preset documents a full channel. On 1.1.7 every write after the first in a burst can be refused, and nothing retries it. The visible failure is a linked pair whose second channel never gets the final value of a drag, so the device differs from the UI until the next bulk read.
- *Options.* Retrying alone (wait 1 to 2 ms, resend) is correct but replays every stale intermediate value when the main loop is slow. Coalescing alone (keep only the latest value per band and send it when the previous write has finished) cuts traffic, but the host cannot see when the slot frees, because an ACK only means queued. So the next band can still be refused.
- *Recommendation.* Use both, in one `BandWriter` owned by the view model. Keep a map from (channel, band) to the latest 16-byte record, plus a bypass-only flag when the change was a bypass toggle. Drain it on one serial queue through the outcome-returning SET. On `VC_ERR_BUSY`, wait 1 to 2 ms (5.8) and resend the newest value for that band. Give up after about 50 ms, the window the firmware's own UART and I2C retry uses, then log and leave the band dirty for the next drain. Route `setFilter`, `sendFilterToDevice`, `setCrossoverBand`, `setBandBypass` and `setCrossoverBandBypass` through it. Remote Link sessions share the same slot, so the retry also covers a remote write landing between two of yours.

**Other busy cases.** The Control Surfaces preview SETs are already serialized by `pollCsDeferred`, which waits for `CS.STATUS` byte 0 to leave `0x16`. `STORE.ALL_PARAMS` and `MEAS.RTA_BANDS_ALL` answer `VC_ERR_BUSY` while another transport holds the bulk buffer, and `fetchAllParamsRetrying` already retries the former. Keep the existing retries around flash writes (5.8).

#### 13.2.8 Host-specific hazards

**The embedded Link hub keys on the 8-bit opcode.** Paged commands reuse the numbers `0x10` to `0xFF` on every page, so any table keyed on `bRequest` alone maps unrelated commands onto one entry. Key every one of these on the 16-bit ID, or on (page, command) with the direction where it matters.

| File | Keyed on the opcode today | Key it on |
|---|---|---|
| `Link/policy/commands.json`, `Link/LinkPolicy.swift` | 246 rows by `code` with a class per direction, `byCode: [UInt8: ...]`. An unlisted code is `config` (admin only) | (page, command, direction). Generate the rows from the registry plus a class column |
| `Link/CommandRouter.swift` | The policy check on `request.bRequest`. `bulkOpcodes` 0xA0 to 0xA3 for the 5 s timeout. `parameterKey`, which special-cases `SET_EQ_PARAM` payload bytes | The ID. Take bulk from the registry `BULK` flag. `FILTER.BAND` has its address in `wValue`, so key writes on (ID, `wValue`) and drop the special case |
| `Link/CommandRouter.swift` | RTA arbitration by `rtaWatchCodes` (0x0B, 0x0C, 0x0F) and by `RTA_CONTROL` with `wValue` START or STOP | `MEAS.RTA_BANDS`, `MEAS.RTA_BINS`, `MEAS.RTA_BANDS_ALL` and the `MEAS.RTA_START` and `MEAS.RTA_STOP` IDs. A swallowed STOP must answer a zero-length ACK, not the 1-byte `01` it answers today |
| `Link/PollScheduler.swift` | `PollSpec.req: UInt8` for classification and `sessionsSubscribed(toAnyOf:)` | The ID |
| `Link/LinkClient.swift`, `Link/NetworkTransport.swift` | The client's `bulkOpcodes`. The `REQ_GET_ALL_PARAMS` snapshot shortcut | `STORE.ALL_PARAMS` and the `BULK` flag |
| `Link/LinkSessionHandler.swift`, `Link/LinkMessages.swift` | `policy.denied` sent as `[bRequest, dir]` pairs in `hello` | Pairs carrying the ID |
| `Link/DeviceTransport.swift`, `Link/LinkFrames.swift` | `LinkStatus` knows 0x00 to 0x07 and 0x80 to 0x87. `LinkFrames.decode` throws `unknownStatus` for anything else | Add the result codes `0x10` to `0x1A` so a refusal reason, `VC_ERR_BUSY` above all, can cross the hub |

A policy table carried over by number gives wrong classes, and some of them grant too much. `SYS.ENTER_BOOTLOADER` (`0x0130`) would take the row of code 0x30, `SET_PSYBASS`, class `control`, so a control-role remote client could reboot the device. `ROUTE.MASTER_VOLUME_SAVED` (`0x0553`), a harmless read, would take the row of 0x53, `FACTORY_RESET`, class `config`, and be denied to viewers. `SYS.SYSTEM_STATUS` (`0x0140`) has no row at 0x40, so it would fall to `config` and the statistics would be admin only. Agree the new table format with the DSPi Link spec (13.3) before changing it, because other hubs and clients embed the same file.

**The bundled web app.** `DSPi Console/Web/assets/index-Dbz_9Gsv.js` is a built bundle that the hub serves. It hard-codes `GET_STATUS` 80 with selector 9, `SET_MASTER_VOLUME` 210, `GET_MASTER_VOLUME` 211 and a default `wIndex` of 2, and subscribes polls as `{req: 80, val: 9, idx: 2}`. Its source is not in this repository. Rebuild it from its source against the new IDs (`MEAS.LEVELS`, `ROUTE.MASTER_VOLUME`) before shipping a ported hub.

**ADAT pin 0.** `setAdatPin` sends `wValue` 0 to mean "platform default" and mirrors 0 as `ADAT_PIN_DEFAULT`. In 1.1.7 a pin SET treats `0xFF` as the default and 0 as GPIO 0 (`Documentation/commands.md` section 3, rule 13), so a 0 from the UI would move the ADAT output to GPIO 0. Send `0xFF` for the default. The bulk blob is unchanged. It still reports the live ADAT output pin, and a 0 written there still selects the default, so the blob and notification decoders stay as they are.

**Feature gates.** The Console decides features three ways. They are release-version gates (`firmwareSupportsNotch`, `firmwareSupportsAllPass`, `firmwareSupportsBandBypass` at 1.1.4 or later), bulk wire-format gates, and probe-by-STALL flags (`inputSourceSupported`, `multiSpdifSupported`, `i2sInputSupported`, `controlSurfacesSupported`, `siggenSupported`, `adatSupported`, `outputLatencySupported`, `dacHwMuteSupported`, `lgSoundSyncSupported` and others). On a paged device, replace the STALL probes with bits from `SYS.PAGE_COMMANDS`, read once at connect (4.7). A STALL in 1.1.7 can mean many things. Every version gate is true on any paged firmware.

#### 13.2.9 Porting order and tests

1. Vendor the registry, generate the constants and add the regeneration test. Delete the dead functions of 13.2.5.
2. Widen `DeviceTransport` and every implementation and fake to the 16-bit ID, build `wIndex` in `USBDevice`, and fix the helpers of 13.2.3.
3. Add the probe, the protocol state in `HubDeviceInfo` and the legacy mode with its update flow (13.2.4).
4. Add the outcome-returning SET and the `SYS.LAST_RESULT` read (13.2.3).
5. Renumber the 130 mechanical opcodes.
6. Port the write-as-reads, splits and reshaped payloads (13.2.5, 13.2.6), the `BandWriter` (13.2.7) and the statistics struct reads.
7. Re-key the Link hub, regenerate `commands.json` and rebuild the web app, together with the DSPi Link port (13.3).

**The test that parses config.h.** `testEveryFirmwareRequestCodeIsInTheTable` in `DSPi ConsoleTests/LinkPolicyTests.swift` reads `/Users/weeblabs/DSPi/firmware/DSPi/config.h`, matches `#define REQ_*` and asserts that more than 200 exist and every code is in `commands.json`. The 1.1.7 `config.h` defines none, so the test fails rather than skipping. Replace it with a test that reads the vendored `dspi_commands.json` and asserts that every (page, command, direction) has a policy row and that every generated constant the app uses exists. That also removes the hard-coded path into another checkout. Its sibling `testEveryAppRequestConstantIsInTheTable` parses `Constants.swift` for `let REQ_*: UInt8` and must move to the generated file as well.

**Other tests to update.** The live-device wire tests drive `USBDevice` with old constants and `wIndex` 0 or 2 (`SubharmWireTests` alone makes 39 transfers, then `RtaWireTests`, `SiggenHardwareDiagTests`, `I2sClockPinModeWireTests`, `PsybassWireTests` and about a dozen more). They skip without hardware, so they pass silently until someone runs them. The Link tests use raw opcodes (`LinkHubCoreTests`, `LinkHubIntegrationTests`, `LinkPolicyTests`, `LinkFrameTests`, `LinkRtaArbitrationTests`, `LinkPollSchedulerTests`, `LinkClientTests`). So do the fakes (`LinkFakeDevice`, `FakeDevice`, `FakePollDevice`, `RtaFakeTransport`), and `LinkFixtureGeneratorTests` hard-codes `GET_BUILD_INFO` 0x80 (now `SYS.BUILD_INFO`). `FirmwarePlatformReplyTests` stays valid.

**On hardware**, on top of section 12.1, check these Console-specific cases.

- On a 1.1.6 device the Console sends only the probe, 0x7F and, after the user confirms, 0xF0. It shows the mismatch banner and completes an update to 1.1.7.
- Drag a band on a linked input pair, multi-select drag, Clear All and paste, then read `FILTER.BAND` back for both channels and compare with the UI.
- The statistics window shows the core voltage (1100 mV at the default setting) and a plausible temperature, not values built from neighbouring fields.
- A rejected pin assignment shows the same message as in 1.1.6.

**Paired release.** Firmware and Console ship as a matched pair with the same version number (the firmware repository's `CLAUDE.md`, release process steps 2 and 8). The Console bundles the matching UF2s, `FirmwareVersion.expected` follows `MARKETING_VERSION`, and `firmwareMatch` is `.match` only on an exact version, so a 1.1.7 beta 1 Console offers an update or a downgrade to any other build. Keep the version for that offer only. Gate features on discovery (`SYS.PAGE_LIST`, `SYS.PAGE_COMMANDS`) so that a feature appears exactly when the connected firmware has its command, whatever the version number says.

### 13.3 DSPi Link

#### 13.3.1 At a glance

**TL;DR.** DSPi Link shares DSPi devices over a WebSocket. A hub owns the physical link to the device and tunnels each vendor request for its clients, byte for byte. The tunnel frames already carry a 16-bit `wIndex`, so the page byte travels through untouched. What breaks is everything a hub decides by `bRequest` alone. That covers its authorization table, its bulk path, its analyser arbitration and its own identification reads, which go out on page 0. Today the ESP32 bridge simply never attaches a 1.1.7 device, so nothing bad happens. The danger is a half-finished port. A bridge that attaches 1.1.7 but still classes commands by `bRequest` lets a `control` session send the device to its bootloader or reset it to factory defaults.

- **Repository.** `/Users/weeblabs/DSPi-Link`, branch `main`, audited at `86f10e3` (2026-11-02). The published bridge firmware is tag `bridge-v0.1.0` (`60550e3`). Both behave the same against 1.1.7.
- **Parts.** The Link spec and the bridge spec (`spec/`), the shared authorization table `policy/commands.json` (246 rows, one per 1.1.6 opcode), byte-exact fixtures (`fixtures/`), the TypeScript library `js/packages/dspi-link` and web app `js/apps/web`, and the ESP32 bridge firmware `bridge/esp32` (C on ESP-IDF). The bridge reaches one DSPi over the UART control transport (11.5). Console hubs reach the device over USB and are implemented in the DSPi Console repository (13.2), but the rules they follow are written here.
- **Against 1.1.7 today.** The bridge's baud probe is the old 0xF9 `GET_CTRL_IFACE_STATUS` on page 0. Firmware 1.1.7 answers it with status `0x10`, and any answer counts as "a DSPi is listening". The serial read that follows gets `0x10` too, so the identity read fails. The bridge logs "identity could not be read" once and probes again every 2 s for ever. Every client command is answered `0x80 NO_DEVICE`, and the web settings page tells the user to check the wiring. Nothing on the DSPi changes, because every request it receives is a refused read.
- **Size of the job.** The repository sends 10 old opcodes. The bridge sends 8 of them from 8 call sites, and the web app and two bench scripts send the rest, which is mechanical work. The policy table, the rules hard-coded on five pass-through opcodes and about twenty spec statements need design work, including a Link spec version decision.

#### 13.3.2 Before you start

Settle the Link-level design first. These changes are not additive under the spec's own compatibility rules (spec section 10), and the DSPi Console branch `feature/dspi-link` embeds the same policy file as `DSPi Console/Link/policy/commands.json` with `LinkPolicy.swift`, so both repositories move together.

| Decision | Why it is needed | Recommendation |
|---|---|---|
| Policy key | Up to 13 commands share one command number (`0x10` is used on 13 pages), so a `bRequest` cannot name a command | Key every row by the 16-bit ID `(page << 8) \| command`, with the `PAGE.NAME` as its name |
| `policy.denied` element | `[bRequest, dir]` pairs cannot name a paged command | Send `[id, dir]` in a new field and leave `denied` empty, so old clients enable everything and the hub still enforces |
| Status range | Spec rule 5 reserves only `0x00` to `0x07` for the firmware, but 1.1.7 command results are `0x10` to `0x1A` | Reserve `0x08` to `0x7F` for firmware codes, forwarded verbatim |
| Device protocol | Old firmware ignores the page byte, so a client must know which generation a handle speaks | Add a device-object field filled from `SYS.PROTOCOL_INFO` bytes 4 and 5, and let clients send paged requests only when it is present |
| Write-as-read rules | 1.1.7 has none (6.6) | Delete them. "Every GET is a read" becomes a rule |

Inputs for the regenerated policy are `Documentation/protocol/dspi_commands.json` (the registry) and `Documentation/protocol/legacy_map.json` from the firmware repository at 1.1.7 beta 1. Vendor both into the Link repository, as the fixtures are, so CI can check the generated table against them.

These stay exactly as they are. The binary frame header and every frame layout, including the unused reserved byte at CMD offset 7. The JSON control plane apart from the fields above. The UART framing, CRC, resync, quarantine and timeout code in `bridge/esp32/components/dspi_frame` and `dspi_uart/dspi_link.c`. Notification relay and origin attribution, because v2 packets did not change (section 10). The bulk header parsing in `hub_device.c` and the TypeScript bulk decoders, because the blob is still format 32 and 6136 bytes (9.1). The platform decoders, because `SYS.PLATFORM` kept the 1.1.6 layout (4.6).

#### 13.3.3 Transport and choke points

Two Link carriers hold `wIndex`, and both already give it 16 bits, so a page fits without a format change. The other frames carry no address.

| Carrier | Fields (spec section) | 1.1.7 content |
|---|---|---|
| CMD request, type `0x01` | offset 6 `bRequest` (u8), offset 8 `wValue` (u16), offset 10 `wIndex` (u16), offset 12 `wLength` (u16) (8.2) | `bRequest` = command, `wIndex` = `(page << 8) \| 2`, `wValue` = address only |
| `poll.subscribe` entries | `req`, `val`, `idx`, `len` (7.6) | `req` = command, `idx` = `(page << 8) \| 2`. The bridge already accepts `idx` up to `0xFFFF` |
| CMD response, POLL, NOTIFY | No address at all (8.2, 8.3, 8.4) | Unchanged |

Clients send the low byte 2 because a Console hub forwards `wIndex` to USB, where the low byte must name the vendor interface (11.1 to 11.3). The bridge may send its own requests with low byte 0, which is fine on UART.

These are the places every request passes, and what each needs.

- **TypeScript library.** `LinkClient.command` in `js/packages/dspi-link/src/client.ts` encodes the full 16-bit `wIndex` already. Its wrappers `get` and `set` default `wIndex` to 2, which is page 0, so a caller that forgets the page is refused rather than misrouted. Replace the `bRequest` and `wIndex` parameters with one command ID and derive `bRequest = id & 0xFF` and `wIndex = (id & 0xFF00) | 2` inside, with no default.
- **Web app.** `APP_WINDEX = 2` in `js/apps/web/src/device/volume.ts` is passed on every request from `volume.ts` and `page.ts`. It becomes a per-command `wIndex`. That is `0x0502` for the volume, `0x0202` for the bulk read and `0x0902` for the meters.
- **Bridge, client traffic.** `hub_commands_handle_frame` in `hub/hub_commands.c` decodes the frame, asks `link_policy_class(command.request, command.is_get)`, runs `admission_status` and `enqueue_command`, and copies `index` into the UART request untouched. Only the policy lookup and the hard-coded rules of 13.3.5 change here.
- **Bridge, its own requests.** `dspi_link.c` (the probe `IFACE_STATUS_REQUEST`, `SERIAL_REQUEST` and `read_get`) and `hub_device.c` (`submit_name_write`, `submit_status_read`, `start_snapshot_read`) build `dspi_request_t` with designated initialisers that never set `.index`. Each needs `.index = page << 8`, and `read_get` needs a command ID instead of a `uint8_t` code.
- **Bridge, polls.** `parse_poll` and `read_requests` in `hub/hub_polls.c` class a poll by `req` alone. Pass the page from `idx >> 8`. The poll key `(req, val, idx, len)` already keeps identical commands on different pages apart.
- **Bridge, status.** `link_status_for` returns the device's status byte unchanged. That is right for 1.1.7, which carries the command's own result code in that byte (11.5).

#### 13.3.4 Probe and identification

The bridge looks for its device in `look_for_device()` (`dspi_uart/dspi_uart.c`). It finds the baud rate with `dspi_link_find_baud`, then reads the identity with `dspi_link_read_identity`. Make the baud probe itself the `SYS.PROTOCOL_INFO` GET. It is harmless on both generations, any well-formed answer still proves the baud rate, and the same answer tells the bridge which firmware it found. Then read the identity with the paged commands below. Every frame was computed with the CRC of 11.5.

| Step | Command | UART request | Reply |
|---|---|---|---|
| Probe and baud | `SYS.PROTOCOL_INFO` (`0x0110`) | `A5 02 10 00 00 00 01 10 00 A0 BD` | 16 bytes starting `44 53 50 69`, major 1 at byte 4. Bytes 8 to 11 give the 64-byte SET and 256-byte GET limits |
| Serial | `SYS.SERIAL` (`0x0121`) | `A5 02 21 00 00 00 01 10 00 4C 5C` | 16 bytes, unchanged |
| Platform | `SYS.PLATFORM` (`0x0120`) | `A5 02 20 00 00 00 01 07 00 C9 7E` | 7 bytes, unchanged layout. Beta 1 on RP2350 is `01 01 17 09 01 07 01` |
| Interface status | `XPORT.STATUS` (`0x0B12`) | `A5 02 12 00 00 00 0B 08 00 58 90` | 8 bytes, unchanged layout |
| UART config | `XPORT.UART_CONFIG` GET (`0x0B10`) | `A5 02 10 00 00 00 0B 08 00 BB F0` | 8 bytes, unchanged layout. The GET works on UART, only the SET is USB only |
| Board name | `SYS.DEVICE_NAME` GET (`0x0124`) | `A5 02 24 00 00 00 01 20 00 7E 20` | 32 bytes on every 1.1.7 build |

**On firmware 1.1.6 or older** the probe answers status `0x02` with no payload (`A5 82 02 00 00 F0 DA`). Use the 1.1.7-only flow (4.5). Do not attach the device, send it nothing paged, and keep probing every 2 s, since the probe is safe. The bridge has no firmware install path (spec 9.2), so the fix is a USB update with DSPi Console. Say so on the setup and settings pages and in the uart status, instead of "check the three wires". To show the old version, the only other request allowed is the old `GET_PLATFORM` frame `A5 02 7F 00 00 00 00 07 00 87 29`. Users who must stay on 1.1.6 keep `bridge-v0.1.0`. Supporting both generations in one bridge would need two policy tables and two request sets, which the bridge has no reason to carry.

**Any other answer** to the probe means the device is not ready or the link is faulty. Status `0x01`, for example, comes back while a USB transfer holds the shared buffers. Treat it as not found yet and retry at the next probe.

**Heartbeat and swaps.** `dspi_link_heartbeat` reads the serial after 1 s of quiet. Keep it on `SYS.SERIAL`. If a 1.1.6 board is swapped in, that read lands on its old opcode 0x21 `GET_CS_GROUP`, which is a read, so the serial mismatch reports the device lost and the next probe classifies the new board. A device lost after three failures is already found again from the probe, never resumed. Keep it that way.

A Console hub runs the same probe in its own attach path (13.2). The spec must also require every hub to report the device's protocol in the device object (13.3.2), because a paged client tunnelling to a 1.1.6 device runs unrelated old opcodes, some of which reset the device or overwrite a preset (4.1).

#### 13.3.5 Requests to port

**Mechanical renumbering.** These are a new command ID, `wIndex` and `wValue` 0 with the same bytes. Take each from section 6.2.

| Area | Old opcodes | New commands |
|---|---|---|
| Bridge identity and heartbeat (`dspi_link.c`) | 0xF9 `GET_CTRL_IFACE_STATUS`, 0x7E `GET_SERIAL`, 0x7F `GET_PLATFORM`, 0xF6 `GET_UART_CONFIG`, 0x57 `GET_DEVICE_NAME` | `XPORT.STATUS`, `SYS.SERIAL`, `SYS.PLATFORM`, `XPORT.UART_CONFIG` GET, `SYS.DEVICE_NAME` GET |
| Bridge rename and adoption (`hub_device.c`) | 0x56 `SET_DEVICE_NAME` | `SYS.DEVICE_NAME` SET, 1 to 32 bytes. The 32-byte zero-padded payload is still valid |
| Bridge snapshot (`hub_device.c`), web bulk read (`page.ts`), `link_conformance.mjs` | 0xA0 `GET_ALL_PARAMS` | `STORE.ALL_PARAMS` GET, 6136 bytes |
| Web volume (`volume.ts`), `link_smoke.mjs` | 0xD2 `SET_MASTER_VOLUME`, 0xD3 `GET_MASTER_VOLUME` | `ROUTE.MASTER_VOLUME`, 4 bytes both ways, one command number for both directions |
| Analyser reads the bridge recognises (`hub_commands.c`) | 0x0B `RTA_GET_BANDS`, 0x0C `RTA_GET_BINS`, 0x0F `RTA_GET_BANDS_ALL` | `MEAS.RTA_BANDS`, `MEAS.RTA_BINS`, `MEAS.RTA_BANDS_ALL` (USB only) |

**Non-mechanical items.** This list is exhaustive for the repository.

1. **Meter frame.** 0x50 `GET_STATUS` with `wValue` 9 becomes `MEAS.LEVELS` (`0x0910`) with `wValue` **0**. A non-zero `wValue` on a command that addresses nothing is `VC_ERR_ADDRESS` (5.5). The reply is the same `2N + 7` bytes. Change the bridge's `submit_status_read` (`STATUS_COMBINED_FRAME`), the web poll in `page.ts` (`STATUS_WVALUE_COMBINED`), `link_smoke.mjs` and `link_conformance.mjs`. A poll becomes `{"req": 16, "val": 0, "idx": 2306, "len": 41}`.
2. **Analyser control.** 0x0E `RTA_CONTROL` was a GET with the action in `wValue`. It is now three zero-length SETs, `MEAS.RTA_STOP` (`0x0929`), `MEAS.RTA_START` (`0x0928`) and `MEAS.RTA_RESET` (`0x092A`), with `wValue` 0 (6.4). `rta_stop_swallowed` and `is_rta_read` in `hub_commands.c` must key on these IDs and on the `KEEPALIVE` reads, `MEAS.RTA_BANDS`, `MEAS.RTA_BINS` and `MEAS.RTA_BANDS_ALL`. The swallowed STOP must be answered `OK` with **no payload**, because a SET response carries none. Today `answer_swallowed_stop` and `release_deferred_acks` send the byte `01`.
3. **Bulk SET.** 0xA1 `SET_ALL_PARAMS` becomes the `STORE.ALL_PARAMS` SET (`0x0240`), exactly 6136 bytes, else `VC_ERR_LENGTH` (9.2). `admission_status` and `enqueue_command` exempt 0xA1 from the 64-byte cap and route it to `bulk_set_buffer` (8192 bytes, so the blob fits). Key both on ID `0x0240` and change them together (13.3.8). Do not key them on the registry `BULK` flag. That would also admit `STORE.ALL_PARAMS_CHUNK` SETs of up to 8192 bytes, which the device refuses on UART with `0x16` only after the bridge has spent about 80 ms of the link sending them.
4. **Write tracking and retries.** `enqueue_command` sets `writes = !is_get || class_ != LINK_CLASS_READ` and `may_retry = is_get && class_ == LINK_CLASS_READ`. With no write-as-reads left these become `writes = !is_get` and `may_retry = is_get`. Under the old table, 111 paged GETs class as non-read, so they would claim other sessions' notification origins, mark the snapshot stale and lose their retry.
5. **The policy table.** All 246 rows of `policy/commands.json` are keyed by 1.1.6 opcode. Regenerate it with the rule below, rewrite `bridge/esp32/tools/gen_policy.py` to emit it, and change `link_policy_class` to take the page.
6. **The enabling command** the user runs in Console (spec 9.2, bridge spec 5) is now the `XPORT.UART_CONFIG` SET. It is still refused on UART, now with `0x16 VC_ERR_BLOCKED`.

**Deriving each command's class.** Build the table from the registry, which lists every command and direction, and from `legacy_map.json`, which says which old opcodes each command replaces.

1. **Every GET is `read`.** In 1.1.7 a GET never changes anything (`Documentation/commands.md` section 3, rule 2). The only side effect is that a `KEEPALIVE` read starts the analyser, which band reads already did in 1.1.6. This covers all 188 GETs, including the 13 that no old opcode reaches (`SYS.PROTOCOL_INFO`, `SYS.PAGE_LIST`, `SYS.PAGE_COMMANDS`, `SYS.COMMAND_INFO`, `SYS.LAST_RESULT`, `INPUT.SPDIF_RX_ENABLE`, `OUTPUT.LATENCY`, `CS.MACRO_HEADER`, `CS.MACRO_STEP` and the bench-only `DIAG.LOOP_PROF_*` reads).
2. **A SET takes the class of the old opcode that reaches it.** Walk every adapter of every opcode in `legacy_map.json`. `direct` targets in an `out` adapter, `act_value`, `act_addr` and `act_packed` targets, and `custom` uses tagged `:S` all name a SET. Take the old opcode's class from the current `policy/commands.json` for the direction the old host used. Of the 151 SETs, 148 resolve this way with one class.
3. **Flags check the result, they do not decide it.** `FLASH` and `RESTART` do not imply `config`. `STORE.PRESET_SAVE`, `STORE.PRESET_LOAD`, `STORE.PRESET_DELETE`, `STORE.PRESET_NAME`, `SYS.DEVICE_NAME`, `INPUT.SOURCE`, `INPUT.SOURCE_NAME` and the `STORE.ALL_PARAMS` SET are `control` by the existing design (spec 6.3), and several `config` SETs carry no flag. `USB_ONLY` and `SET_USB_ONLY` commands never reach a device through a bridge, but keep their class for Console hubs. `BULK` decides buffer routing, not class.

Three SETs cannot be derived and need a human decision.

| SET | Why | Suggested class |
|---|---|---|
| `STORE.PRESET_SAVE` (`0x0210`) | Reached by 0x90 `PRESET_SAVE` (`control`) and by the emulation of the retired 0x51 `SAVE_PARAMS` (`config`) | `control`, the direct replacement |
| `DIAG.LOOP_PROF_BG_PACE` (`0xF033`) | No old opcode. Bench builds only | `config` |
| `DIAG.LOOP_PROF_RESET` (`0xF034`) | No old opcode. Bench builds only | `config` |

**Spec text to rewrite.** Each of these statements is false for 1.1.7.

| Where | Statement | 1.1.7 |
|---|---|---|
| Link spec 2, 7.4 | Device identified by `REQ_GET_SERIAL` (0x7E), named by 0x57, platform from 0x7F | `SYS.SERIAL`, `SYS.DEVICE_NAME`, `SYS.PLATFORM`, same bytes |
| Link spec 6.3 | "The hub classifies each `bRequest`", `policy.denied` as `[bRequest, dir]` pairs | Classify by command ID (13.3.2) |
| Link spec 6.3, 8.2 | Write-as-read commands "must list them explicitly" and "are sent with dir = 1" | None exist. Every GET is a read |
| Link spec 6.3 | Per-direction classes for 0x81 `LIMITER` and 0x03 `OUTPUT_LATENCY` | Split into one command per parameter or view (6.4). GET is read, the SET has one class |
| Bridge spec 7.6, Link spec 7.4 | Older firmware "STALLs it or answers one byte" for the name | Every 1.1.7 build answers 32 bytes. The old-firmware branch serves only 1.1.6, which a paged hub no longer attaches |
| Link spec 7.5, 11.3 | `status_b64` is the "`REQ_GET_STATUS` wValue=9 response" | The `MEAS.LEVELS` reply, same bytes |
| Link spec 7.6, 11.3 | Poll examples `req 80, val 9, idx 2` and `req 11, val 3, idx 2, len 82` | `req 16, val 0, idx 2306` and `req 35, val 3, idx 2306, len 82` |
| Link spec 7.7 | Lock flows named by `REQ_PRESET_SAVE`, `REQ_SAVE_OUTPUT_CONFIG`, `REQ_SAVE_PARAMS`, `REQ_ENTER_BOOTLOADER` | 0x51 `SAVE_PARAMS` is retired (6.7). Rename the others |
| Link spec 7.8, 8.2 | Install "enters the bootloader (0xF0)", which "is `config` class" | `SYS.ENTER_BOOTLOADER`, a zero-length SET (`0x0130`) |
| Link spec 7.15, bridge spec 8.3 | Watching means reads of 0x0B, 0x0C, 0x0F, and STOP is answered "`01`" | The `KEEPALIVE` reads and three SETs. The answer has no payload |
| Link spec 8.1, rule 5 | Codes `0x00` to `0x07` are the firmware's, and "USB STALL maps here" (`0x02`) | 1.1.7 never sends `0x02` or `0x03`. Its results are `0x10` to `0x1A` |
| Link spec 8.2 | `wIndex` "Pass 2 for application commands" | `(page << 8) \| 2` |
| Link spec 8.2, 9.1.1 | Bulk is 0xA0 and 0xA1, chunked is 0xA2 and 0xA3 in 2048-byte pieces | `STORE.ALL_PARAMS` and `STORE.ALL_PARAMS_CHUNK` (9.2, 9.3) |
| Link spec 9.2 | Bridge clients receive "`0x02 ERROR` (not `BLOCKED`)" for bands of all channels, and replies are capped at 132 bytes | `MEAS.RTA_BANDS_ALL` answers `0x16`. Replies are up to 256 bytes, delivered whole |
| Link spec 9.3, rule 7 | `policy/commands.json` holds "Every `bRequest`", and an unlisted one is `config` | Every command ID. An unlisted ID stays `config` |
| Link spec rule 6 | Clients gate features on `REQ_GET_PLATFORM` and `REQ_GET_CAPS` | Gate on `SYS.PAGE_LIST`, `SYS.PAGE_COMMANDS` and `SYS.COMMAND_INFO` (4.7) |
| Link spec 11.1, 11.2 | Worked bytes with 0xD2 and 0x7F, `wIndex` 2 | `01 00 07 00 00 00 50 00 00 00 02 05 04 00 00 00 A0 C1` and `01 00 08 00 01 01 20 00 00 00 02 01 07 00` |
| Bridge spec 7.3, 7.5, 8.4 | Retry and write rules exclude write-as-reads. The baud probe is 0xF9 | Every GET may be retried. The probe is `SYS.PROTOCOL_INFO` |

#### 13.3.6 Replies and checks that change

| Check | Where | 1.1.6 assumption | 1.1.7 |
|---|---|---|---|
| Probe reply | New, in `dspi_link_read_identity` or the baud probe | None | 16 bytes, `44 53 50 69`, byte 4 = 1 |
| Serial | `dspi_link_read_identity` | More than 0 bytes, not empty | `SYS.SERIAL`, 16 bytes. Check unchanged |
| Platform | `dspi_link_read_identity`, `hub_device.c` decode | At least 4 bytes (`PLATFORM_MIN_BYTES`) | `SYS.PLATFORM` is always 7 bytes. Byte 6 is the beta number, 1 on 1.1.7 beta 1 |
| Interface status | `dspi_link_read_identity` | At least 2 bytes, byte 1 non-zero | `XPORT.STATUS`, 8 bytes, same layout |
| UART config | `dspi_link_read_identity` | Exactly 8 bytes | `XPORT.UART_CONFIG`, 8 bytes, same layout |
| Board name | `dspi_link_read_identity` | Exactly 32 bytes, else old firmware | `SYS.DEVICE_NAME`, 32 bytes on every build |
| Meter frame | `hub.h` snapshot buffer, `decodeStatusFrame` | `2N + 7`, at most 132 | `MEAS.LEVELS`, 21 bytes on RP2040, 41 on RP2350 |
| Bulk blob | snapshot, web `isSupportedBulkVersion` | 6136 bytes, format 30 to 32 | Unchanged, format 32 |
| Volume read | `readDeviceVolume` in `volume.ts` | At least 4 bytes | `ROUTE.MASTER_VOLUME`, 4 bytes |
| Largest non-bulk reply | `DSPI_UART_MAX_GET_REPLY_BYTES` 132, event pool small buffers | 132 bytes | 256 bytes. `MEAS.RTA_BINS` returns up to 256 per request (7.3) |
| Swallowed analyser STOP | `answer_swallowed_stop`, `release_deferred_acks` | `OK` + `01` | `OK`, length 0 |

The bridge's shape rule in `matches()` (a GET answer may not exceed `wLength`) still holds. A GET with `wLength` 0 on UART now means "the whole reply" (11.5), and the bridge already treats 0 as "no limit" there.

#### 13.3.7 Errors, status and busy handling

**Decode the new codes.** `LinkStatus` in `js/packages/dspi-link/src/frames.ts` stops at `0x07` and `statusName` prints anything else in hex. Add `0x10` to `0x1A` with the `VC_ERR_*` names (5.3). The bridge's `DSPI_STATUS_*` constants in `dspi_frame.h` need the same. `0x02` and `0x03` stay defined for 1.1.6 but never come from 1.1.7. Over a bridge a USB-only command is now `0x16`, not `0x03` or `0x02`.

**Show the right sentence.** `refusal()` in `volume.ts` gives specific words only for `LOCKED`, `DENIED`, `BUSY` and `TIMEOUT`. The rename result in `hub_device.c` answers `busy` only for `BUSY`, `BULK_LOCKED` and no answer, and `internal` for everything else. Add `0x17 VC_ERR_BUSY` to the busy cases in both. Other codes may keep the generic message, but log the code name.

**Console hubs must forward the reason.** Spec 9.1 and 9.1.1 tell a USB hub to map a STALL to `0x02 ERROR`. That throws away the reason, and a client then cannot tell `VC_ERR_LENGTH` (permanent) from `VC_ERR_BUSY` (retry). The spec should say instead that a USB hub reads `SYS.LAST_RESULT` straight after the STALL, checks that the command ID, direction and `wValue` match its own request, and forwards the result byte as the status. The record is per transport (5.2), so another USB program on the same computer can overwrite it. When the record does not match, forward `0x02`, which 1.1.7 never sends and can therefore mean "refused, reason unknown". This gives clients the same code from a USB hub as from a bridge.

**Retry only what can succeed.** A client that treats every non-OK status, or every STALL, as a flash busy window will retry permanent refusals until it gives up. That makes it slow, and it never learns the reason. Retry unchanged only on `0x01`, `0x04`, `0x17` and the Link-level `0x84 RATE_LIMITED`. On `0x82 TIMEOUT`, retry a read after about 150 ms, and confirm a write by reading it back, since it may have run (5.8). Treat every other code as final and show it. The bridge itself is already right. `worth_retrying` in `dspi_link.c` resends only on `0x05` and `0x07`, or a read with no answer, and passes busy codes on because the firmware has already retried for 50 ms.

**Bulk busy has more than one code on UART.** A `STORE.ALL_PARAMS` request that finds the bulk buffer held by another transport, or a bulk apply still pending, is refused by the UART transport itself as `0x04`, before the command runs. A USB host reads `VC_ERR_BUSY` for the same condition (5.8). The firmware retries a held-buffer GET for 50 ms before it answers, but it answers a bulk SET with `0x04` as soon as the frame ends (11.5), so the bridge's pass-through leaves all the waiting for that SET to the client. Treat `0x01`, `0x04` and `0x17` alike as "retry shortly".

#### 13.3.8 Host-specific hazards

**Carrying the old table over by number escalates privileges.** If the 8-bit table is kept and indexed with the new command byte, each paged command inherits the class of whichever 1.1.6 opcode had its number. For 15 SETs that class is `control` where `config` is due, so any `control` session on the hub can run them. A Link client writes raw frames, so the session does not need an updated client to do it. The full list follows. It was computed over all 339 paged (command, direction) pairs against the 1.1.6 `config.h` (`fb9a254`).

| SET | ID | Old opcode with that number (class `control`) | Due class, from the opcode it replaces |
|---|---|---|---|
| `SYS.ENTER_BOOTLOADER` | `0x0130` | 0x30 `SET_PSYBASS` | `config` (0xF0 `ENTER_BOOTLOADER`) |
| `STORE.FACTORY_RESET` | `0x0230` | 0x30 `SET_PSYBASS` | `config` (0x53 `FACTORY_RESET`) |
| `STORE.STARTUP` | `0x0216` | 0x16 `SET_SUBHARM_BOOST` | `config` (0x96 `PRESET_SET_STARTUP`) |
| `INPUT.I2S_RX_PIN` | `0x0330` | 0x30 `SET_PSYBASS` | `config` (0xF1 `SET_I2S_RX_PIN`) |
| `INPUT.I2S_RX_RATE` | `0x0332` | 0x32 `SET_PSYBASS_CUTOFF` | `config` (0xED `SET_INPUT_RATE`) |
| `INPUT.ADAT_RX_CLOCK_MODE` | `0x0342` | 0x42 `SET_EQ_PARAM` | `config` (0x6C `SET_ADAT_INPUT_CLOCK_MODE`) |
| `OUTPUT.TYPE` | `0x0410` | 0x10 `SET_SUBHARM` | `config` (0xC0 `SET_OUTPUT_TYPE`) |
| `OUTPUT.ADAT_TX_ENABLE` | `0x0430` | 0x30 `SET_PSYBASS` | `config` (0xCA `SET_ADAT_ENABLE`) |
| `CS.SAVE` | `0x0A18` | 0x18 `SET_SUBHARM_MASK` | `config` (0x9D `CS_SAVE`) |
| `CS.IR_CMD` | `0x0A30` | 0x30 `SET_PSYBASS` | `config` (0x8D `SET_CS_IR_CMD`) |
| `CS.IR_LEARN_ARM` | `0x0A32` | 0x32 `SET_PSYBASS_CUTOFF` | `config` (0x8F `CS_IR_LEARN`) |
| `CS.DISPLAY_CONFIG` | `0x0A70` | 0x70 `SET_MATRIX_ROUTE` | `config` (0x27 `SET_CS_DISPLAY_CFG`) |
| `XPORT.UART_CONFIG` | `0x0B10` | 0x10 `SET_SUBHARM` | `config` (0xF5 `SET_UART_CONFIG`). Refused on UART anyway |
| `CLOCK.I2S_BCK_PIN` | `0x0C10` | 0x10 `SET_SUBHARM` | `config` (0xC2 `SET_I2S_BCK_PIN`) |
| `CLOCK.I2S_CLOCK_MODE` | `0x0C12` | 0x12 `SET_SUBHARM_LOW` | `config` (0x88 `SET_I2S_CLOCK_MODE`) |

The same carry-over is wrong in the other direction for 187 pairs, and those break the hub instead. 105 GETs land on a SET-only or absent row and become `config`, 6 GETs land on a write-as-read row and become `control`, and 76 SETs become `config` where `control` is due. Link's own traffic is hit directly. Viewers and control users are denied `SYS.PROTOCOL_INFO`, `SYS.PLATFORM`, the `SYS.DEVICE_NAME` GET, the `STORE.ALL_PARAMS` GET and `MEAS.RTA_BINS`. `MEAS.LEVELS` cannot be polled by anyone. Control users are denied the `ROUTE.MASTER_VOLUME` SET, the `SYS.DEVICE_NAME` SET and `MEAS.RTA_START`, `MEAS.RTA_STOP` and `MEAS.RTA_CONFIG`. Only 134 pairs come out right by coincidence.

**Port in the right order.** Today the escalation is masked only because the bridge never attaches a 1.1.7 device. Rekey the policy before you readdress the identity reads, never after.

**The stored denial list now names other commands.** The fixture `fixtures/messages/auth_ok.json` denies `[[240,1],[83,1]]`, meaning the 1.1.6 bootloader and factory reset. Read as paged pairs, `[83,1]` greys out reads of `ROUTE.MASTER_VOLUME_SAVED` and `[240,1]` names no command. In 1.1.7 the two commands it meant are both `[0x30, SET]`, the same pair as `INPUT.I2S_RX_PIN`, `OUTPUT.ADAT_TX_ENABLE`, `ROUTE.CHANNEL_DELAY`, `DYN.LOUDNESS_ENABLE` and `CS.IR_CMD`.

**A full ID table does not fit the ESP32.** A table indexed by every 16-bit ID is 128 KB. Emit a sorted array of `{id, SET class, GET class}` (211 rows) and search it, with a missing ID answering `config`.

**The bulk cap and the bulk buffer must move together.** `hub_pending_t.payload` in `hub/include/hub.h` is 64 bytes. If `admission_status` lets a 6136-byte SET past the 64-byte cap but `enqueue_command` does not route it to `bulk_set_buffer`, the `memcpy` overflows the pending entry.

**A paged request must never reach a 1.1.6 device.** A hub that keeps a 1.1.6 device attached, as a Console hub does to offer a firmware update, must refuse every CMD whose `wIndex` high byte is not 0 for that handle. Otherwise a paged read such as `ROUTE.MASTER_VOLUME_SAVED` runs 0x53 `FACTORY_RESET` there (4.1).

**`wLength` 0 means different things by hub kind.** On UART it returns the whole reply. On USB a control read with `wLength` 0 has no data stage, so it returns nothing. The spec should require clients to send the length they want. The bridge's update gate (`HUB_UPDATE_MAX_GET_BYTES`, 2048) checks `length > 2048` only, so a client GET of `STORE.ALL_PARAMS` with `wLength` 0 slips past it during a bridge update. Gate on `length == 0` as well. Note also that `parse_poll` refuses `len` 0.

**Replies above 132 bytes can turn into timeouts.** The uart-to-hub event pool has 32 buffers of 132 bytes and 2 of 8192 (`HUB_EVENT_*` in `bridge_config.h`). A 133 to 256-byte `MEAS.RTA_BINS` chunk takes a large buffer. While a bulk read holds one and a second chunk holds the other, a further answer "arrives as a timeout". Raise the small buffers to 256 bytes, the limit `SYS.PROTOCOL_INFO` reports (about 4 KB more RAM), and rename the constant, whose comment still cites the 1.1.6 firmware.

**The analyser rule goes dead silently.** No paged command is numbered below `0x10`, so the 0x0B to 0x0F checks in `hub_commands.c` never match on 1.1.7. A `MEAS.RTA_STOP` from one session then stops the analyser for every session, which breaks spec 7.15 without any error.

#### 13.3.9 Porting order and tests

1. Decide the spec changes of 13.3.2 and bump the Link spec version. Agree them with the DSPi Console `feature/dspi-link` branch.
2. Vendor the registry and legacy map. Rewrite `policy/commands.json` keyed by command ID with the derivation of 13.3.5, settle the three undecidable SETs, and rewrite `gen_policy.py` and `link_policy_class`. Check the 15 escalation rows by hand, starting with `SYS.ENTER_BOOTLOADER` and `STORE.FACTORY_RESET`.
3. Change the bridge semantics in `hub_commands.c` and `hub_polls.c`. That covers the page in the policy lookup, the bulk path on `0x0240`, the analyser rule with a zero-length ack, `writes` and `may_retry`, and the update gate.
4. Only then readdress the bridge's own requests, add the probe and the 1.1.6 refusal (13.3.4), and raise the reply buffers.
5. Port the TypeScript library (`get`/`set` by command ID, status names, the new `policy` field) and the web app (volume, bulk read, meter poll, busy message).
6. Rewrite the spec and bridge spec text of 13.3.5 and add a change-history entry.
7. Regenerate fixtures and tests last.

Tests to change.

- `bridge/esp32/test/host/fake_dspi.c` keys its canned answers by `bRequest` only, records no `wIndex`, and answers unknown commands with `0x02`. Record `wIndex`, key answers by command ID, answer page 0 with `0x10`, and add a 1.1.6 mode that answers the probe with `0x02`.
- `test_hub.c` builds every CMD in `send_cmd` with `wIndex` 2. Give it a page. Its 0x0E, 0x91 and 0x0B/0x0C literals test write-as-read and analyser rules that no longer exist. Rewrite those around `MEAS.RTA_START`, `MEAS.RTA_STOP` and `STORE.PRESET_LOAD` SETs. Keep the 65-byte SET to `OVERSIZE` case and the bulk SET case, the latter on `0x0240`.
- `test_dspi_frame.c` checks the 1.1.6 worked frame with CRC `CF 6D`. Use the 1.1.7 example `A5 01 50 00 00 00 05 04 00 00 00 A0 C1 71 3D` from `Documentation/Features/control_interfaces_spec.md`.
- `test_link_policy.c` and `js/packages/dspi-link/test/policy.test.ts` assert codes 0 to 255, spec version `"1.1"` and the 0x81 row. Assert unique IDs instead, and add one test per escalation row that it is `config`.
- Fixtures `cmd_request_set_master_volume.bin`, `cmd_request_get_platform.bin`, `poll_subscribe.json`, `auth_ok.json` and the `spec_version` in `frames.json` take the bytes of 13.3.5. `bulk_v32.bin`, `status_wvalue9.bin` and `platform.bin` stay valid as payloads.
- On hardware, check that the bridge attaches a 1.1.7 RP2040 and RP2350, that it refuses a 1.1.6 board with the update message, and that a `control` session gets `0x81 DENIED` for `SYS.ENTER_BOOTLOADER` and `STORE.FACTORY_RESET`. Check too that two sessions share the analyser and that a 6136-byte `STORE.ALL_PARAMS` SET round-trips through the bridge.

### 13.4 DSPi Terminal

#### 13.4.1 At a glance

**TL;DR.** DSPi Terminal is a Rust command-line tool (`dspi`) and full-screen terminal UI that controls a DSPi over USB. Every request it sends passes through one small transport interface, and every command number it knows is generated at build time from the firmware's C headers. Both assume an 8-bit command number and a fixed `wIndex` of 2. Against 1.1.7 every request therefore lands on page 0 and is refused, and the tool cannot even connect. The port widens the transport to carry the page, points the generator at the command registry, adds the probe, and then works through the Terminal's own parameter table. The design helps. Once the generated constants change type, the compiler names every call site that still sends an old number.

| Item | Fact |
|---|---|
| Repository | `/Users/weeblabs/DSPi Terminal`, audited on `main` at `8389b20` (2026-09-30). The only other branch, `console-parity`, is fully merged |
| Firmware it was written for | Headers vendored from DSPi `release/v1.1.6` at `557bce7`, firmware 1.1.6 beta 4, wire format 32 (`crates/dspi-proto/firmware/PROVENANCE.toml`). It was never re-vendored for 1.1.6 final |
| Transport | USB only, through the `nusb` 0.2 crate. It claims interface 2 for vendor control transfers and reads notifications from bulk IN `0x83`. No UART, I2C or network path |
| What happens on 1.1.7 today | `probe()` sends old 0x7F `GET_PLATFORM` with `wIndex` 2 as its first request. It lands on page 0 and STALLs with `VC_ERR_UNKNOWN`. `with_busy_retry` takes the STALL for a flash blackout and retries it four more times 150 ms apart, then the session fails with "the device refused the request (stalled): opcode 0x7F". The device changes nothing, but the user is not told that the firmware is newer. `dspi raw` sends without identifying the device, and every raw request STALLs |
| Size of the job | 181 parameter rows (116 OUT, 32 IN, 33 write-as-read), about 110 of them a pure renumber. About 70 direct transfer calls in a dozen files. 1286 tests, most of them scripting a mock transport by 8-bit opcode |

#### 13.4.2 Before you start

- Work in a branch of `main`. Nothing else in the repository carries protocol code.
- Copy `Documentation/protocol/dspi_commands.json` from the DSPi repository, at the 1.1.7 commit you target, into `crates/dspi-proto/firmware/`. Add it to the `files` list in `PROVENANCE.toml`, update the branch, commit and dates there, and refresh `SHA256SUMS`. Rewrite `docs/firmware-bump.md` to match, because its procedure is built on `#define REQ_*` lines.
- Re-vendor the 13 headers at the same commit. All 13 still exist. `config.h` no longer has any `#define REQ_` line, and `limiter.h` no longer defines `LIMITER_GET_METER` or `LIMITER_GET_STATUS`. Everything else the scanner lifts (`WIRE_*`, `CS_*`, `TUBE_PARAM_*`, `NOTIFY_*`, `VENDOR_EP_*`, `FW_VERSION_*`) is still there.
- Add `serde_json` to `[build-dependencies]` of `crates/dspi-proto`. It is already a workspace dependency. The current note in that `Cargo.toml`, saying the build script is dependency free, has to go.
- Keep two devices to hand, one on 1.1.6 and one on 1.1.7, and both platforms if you can.

These stay exactly as they are. The bulk blob codec (`crates/dspi-proto/src/wire.rs`, version 32, 6136 bytes, generated section offsets), the notification parser (`crates/dspi-session/src/notify.rs`), the USB identity constants, `bmRequestType`, the 1024-byte chunking of the blob, the `.dspipreset` and filter-file formats, and the command grammar. Users keep typing `vol.user` and `eq.freq`. The undo journal, the write gate and the dry-run log are keyed by parameter path, so they survive unchanged.

#### 13.4.3 Transport and choke points

Every production request goes through the `Transport` trait in `crates/dspi-transport/src/lib.rs`. Its only real implementation is `UsbTransport` in `crates/dspi-transport/src/usb.rs`, which puts the `opcode` argument in `bRequest` and the constant `VENDOR_INTERFACE` (2) in `wIndex`. The trait carries no page today.

```rust
// crates/dspi-transport/src/lib.rs, today
fn control_in(&mut self, opcode: u8, value: u16, len: u16) -> Result<Vec<u8>>;
fn control_in_upto(&mut self, opcode: u8, value: u16, max_len: u16) -> Result<Vec<u8>>;
fn control_out(&mut self, opcode: u8, value: u16, data: &[u8]) -> Result<()>;
```

Widen it to a 16-bit command ID. A newtype is better than a bare `u16`, because it cannot be mixed up with `wValue` and an `as u8` cast on it does not compile. Define it in `dspi-proto`, which `dspi-transport` already depends on.

```rust
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct CmdId(pub u16); // (page << 8) | command

fn control_in(&mut self, id: CmdId, value: u16, len: u16) -> Result<Vec<u8>>;
fn control_in_upto(&mut self, id: CmdId, value: u16, max_len: u16) -> Result<Vec<u8>>;
fn control_out(&mut self, id: CmdId, value: u16, data: &[u8]) -> Result<()>;
```

In `UsbTransport` the split happens in exactly one place. Set `request: id.0 as u8` and `index: (id.0 & 0xFF00) | VENDOR_INTERFACE` in the `ControlIn` and `ControlOut` structs, and `debug_assert!` that the page is not 0. This is the only place an `as u8` belongs.

**The narrowing trap.** Any parameter, field or key left as `u8` silently keeps only the command byte. The page is lost, the request goes out on page 0, and 1.1.7 refuses every request that passes through it. A mock keyed the same way would still answer, so the tests would pass. These places carry the opcode as `u8` today, and every one must take the full ID.

| Where | What is `u8` today |
|---|---|
| `crates/dspi-transport/src/lib.rs` | The three trait methods, `TransportError::{Stalled, TimedOut, ShortRead}.opcode`, and `with_busy_retry(.., opcode: u8)` |
| `crates/dspi-transport/src/usb.rs` | `map_transfer_error(e, opcode: u8)` |
| `crates/dspi-transport/src/mock.rs` | `MockTransport.replies: HashMap<u8, Reply>`, `reply`, `data`, `window`, `resolve`, and `Exchange.opcode`, which tests filter on |
| `crates/dspi-proto/build.rs` | The emitter types every `REQ_*` constant as `u8`, and `emit_opcode_table` casts `*v as u8` into `ALL_OPCODES` |
| `crates/dspi-proto/src/registry.rs` | `ParamDesc.set` and `.get` (`Option<u8>`), `EXCLUDED: &[(u8, &str)]`, `referenced_opcodes() -> Vec<u8>` |
| `crates/dspi-session/src/probe.rs` | The feature probe table `(&str, u8, u16, u16)` and its `"0x{opcode:02X} answered"` evidence strings, which `dspi dump --json` prints |
| `crates/dspi-session/src/surfaces.rs` | `Surfaces::send(request: u8, ..)` |
| `crates/dspi-tui/src/actions.rs`, `crates/dspi-tui/src/settings/mod.rs` | The `packet(.., opcode: u8, ..)` helper and the two `get(session, opcode: u8, len)` closures |
| `crates/dspi/src/main.rs` | `raw_in`, `raw_out` and `raw_war` parse a `u32` and send `opcode as u8` |
| Test fakes | `BandLimited` in `crates/dspi-session/src/probe.rs` and `Fake` in `crates/dspi-session/src/undo.rs` implement the trait with `opcode: u8` |

**STALL handling at the choke point.** `map_transfer_error` turns `TransferError::Stall` into `Stalled { opcode }`. Keep that mapping, carry the `CmdId` instead, and add the reason. Once the probe has confirmed 1.1.7, have `UsbTransport` read `SYS.LAST_RESULT` (8 bytes) straight after any STALL and attach the result code to the error when the echoed ID, direction and `wValue` match (13.4.7). Gate that read on a flag set by a successful probe, so the transport never sends a paged request to a device that has not answered `SYS.PROTOCOL_INFO`. A timeout (`TransferError::Cancelled`) stays `TimedOut` and stays retryable.

#### 13.4.4 Probe and identification

Every entry point except `dspi raw` identifies the device through one function, `probe()` in `crates/dspi-session/src/probe.rs`. The CLI reaches it from `connect()` and `cmd_dump` in `crates/dspi/src/main.rs`, `dspi doctor` from `crates/dspi/src/doctor.rs`, and the TUI from `open_device` in `crates/dspi-tui/src/live.rs`. That function also serves the TUI's device switch and its `reconnect` verb. So the probe goes in one place, as the first request of `probe()`, before the 0x7F `GET_PLATFORM` read that is first today.

```
SETUP  C1 10 00 00 02 01 10 00
IN     44 53 50 69 01 00 0C 00 40 00 00 01 00 00 00 00
```

1. Send `SYS.PROTOCOL_INFO` (page `0x01`, command `0x10`, 16 bytes). Retry a timeout up to four times 150 ms apart, but never retry a STALL. Today the first read is wrapped in `with_busy_retry`, which would retry the STALL that identifies old firmware.
2. **The reply starts with `DSPi` and byte 4 is 1.** Continue with `SYS.PLATFORM` (7 bytes, the layout `PlatformInfo::decode` already parses), then `SYS.SERIAL` (16), then `SYS.PAGE_LIST` and one `SYS.PAGE_COMMANDS` (64) per page, kept in `Capabilities`. Then read the bulk blob with `STORE.ALL_PARAMS_CHUNK`, the Control Surfaces and signal generator caps, `SYS.BUILD_INFO` (64) and `STORE.PRESET_ACTIVE` (1), as `probe()` does now.
3. **Any other major.** Refuse with an `Incompatible` error that says to update DSPi Terminal, and send nothing else.
4. **The probe STALLs.** The device runs 1.1.6 or older. Send old 0x7F `GET_PLATFORM` with `wIndex` 2 once, only to name the version, then return `Incompatible` with an update-the-firmware message. The existing `refusal()` wording can carry it. Give the transport a dedicated method for that one page-0 read, so no `CmdId` ever has to encode page 0.

The Terminal has no firmware update flow, so it needs nothing else from old firmware. If you want `dspi` to help a 1.1.6 user reach the bootloader, old 0xF0 `ENTER_BOOTLOADER` is the only other request section 4.4 allows. `dspi doctor` should report the outcome of step 4 as a named finding, not as a transport error.

`dspi raw` calls `open()` and sends at once, with no identification. Give it the same probe and refuse old firmware unless the user passes an explicit override, as `tools/dspi_poke` does with `--allow-old-firmware` (13.4.8).

#### 13.4.5 Requests to port

Look up every old opcode in section 6.2. The Terminal's table of parameters, `REGISTRY` in `crates/dspi-proto/src/registry.rs`, has one hand-written row per parameter with 14 fields. They are `path`, `label`, `plain`, `group`, `level`, `kind`, `target`, `set`, `get`, `dir`, `wvalue`, `hazard`, `persists` and `requires`. Five of them become wrong.

- `set` and `get` hold 8-bit opcodes. They become `Option<CmdId>`. Keep two fields rather than one, because several rows read and write different commands. `cs.macro` writes `CS.MACRO_HEADER` and reads `CS.MACRO`, and `cs.macro.step` writes `CS.MACRO_STEP` and reads either `CS.MACRO` or `CS.MACRO_STEP`.
- `dir` loses `Dir::WriteAsRead`. Every one of the 33 rows that use it becomes a SET.
- `wvalue` loses `EqScalar`, `ValueSlot`, `ValueIndex`, `ValueOnly`, `RoleValue`, `OutputIndex` and every `Fixed(..)` selector. `wValue` is now only the row's `target`, packed by the command's address kind. `CH_BAND`, `IN_OUT` and `CS_MACRO_STEP` are the only two-index kinds.
- `requires` names probe-by-STALL features (`Requires::Feature`) and `Requires::Rp2350`. Both become bitmap lookups.
- `hazard` is hand-labelled. It can be derived from the registry's `DEFERRED`, `FLASH` and `RESTART` flags.

The rows cannot be regenerated, because they carry the Terminal's own labels and grammar. What you regenerate is the table they point at. In `crates/dspi-proto/build.rs`, read `dspi_commands.json` and emit `pub const <PAGE>_<NAME>: CmdId` for every command plus a `COMMANDS` table with each command's name, ID, directions, address kind, `get_len` (a number or `var`), `set_min`, `set_max`, flags and build condition. Drop `REQ_` from `DEFINE_GROUPS` and replace `emit_opcode_table`. Then add a test per row that checks two things. The SET payload width from `ParamDesc::repr` must lie within `set_min..set_max`, and the GET length the row asks for must equal `get_len`. Rewrite `crates/dspi-proto/tests/coverage.rs` over `COMMANDS`. Every command whose build condition is none or `PICO_RP2350` must be referenced by a row or listed in `EXCLUDED` with a reason, and the `DIAG` page is excluded.

The port per area follows. Section 6.2 has the numbers.

| Area | Terminal code | What the port does |
|---|---|---|
| Identity, stats | `crates/dspi-session/src/probe.rs`, `crates/dspi-tui/src/actions.rs` (Stats panel), `crates/dspi-tui/src/settings/mod.rs` `read_sample_rate` | Probe as in 13.4.4. Replace the `GET_STATUS` word reads with four struct reads (13.4.6) |
| Presets, bulk | `crates/dspi-tui/src/live.rs` `load_preset`, `crates/dspi-tui/src/settings/mod.rs` `SettingsData::read`, `crates/dspi-session/src/probe.rs` `read_bulk_from` | Actions become zero-length SETs. Startup reply shrinks. The blob read is a renumber to `STORE.ALL_PARAMS_CHUNK` |
| Inputs, outputs, clocks | Registry rows `in.*`, `out.*`, `i2s.*`, `adat.*`, `crates/dspi-tui/src/settings/inputs.rs`, `crates/dspi-tui/src/settings/outputs.rs`, `crates/dspi-session/src/preset_file.rs` | Write-as-read pin and clock setters become SETs with the value in the payload |
| Routing, volume | Rows `mix`, `ch.*`, `out.*`, `pre`, `vol.*` | Renumber, except the matrix payload (6.5) |
| Filters | `crates/dspi-session/src/write.rs` `read_band`, `write_band`, `write_eq_field`, `decode_read`, `crates/dspi-session/src/probe.rs` `probe_band_count`, `crates/dspi-proto/src/value.rs` `EqParamPacket` | Rewrite for `FILTER.BAND` (6.5) |
| Dynamics, effects | Rows `lev.*`, `loud.*`, `limit.*`, `cf.*`, `sub.*`, `bass.*`, `tube.*`, `up.*`, `crates/dspi-session/src/runtime.rs` | Renumber, plus the limiter, tube and upmixer splits (6.4) |
| Analyser, signal generator | `crates/dspi-session/src/rta.rs`, `crates/dspi-tui/src/screens/signals.rs`, rows `rta.*`, `sig.*` | Renumber, plus the caps and control splits (6.4) |
| Control Surfaces | `crates/dspi-session/src/surfaces.rs`, rows `cs.*` | Renumber, plus the caps, aux, IR learn, macro fire and display splits, and STALL handling in `Surfaces::send` (13.4.7) |
| Transports | `crates/dspi-session/src/pins.rs`, `crates/dspi-tui/src/settings/mod.rs`, rows `dev.uart`, `dev.i2c`, `dev.ctrl.status` | Renumber to `XPORT.UART_CONFIG`, `XPORT.I2C_CONFIG`, `XPORT.STATUS` |

These are the non-mechanical items, exhaustively.

- **Write-as-reads that carry a value** become SETs with a 1-byte payload and the address in `wValue`. They are `out.pin` (`OUTPUT.PIN`, pin index in `wValue`), `out.type` (`OUTPUT.TYPE`, slot), `in.spdif.pin` (`INPUT.SPDIF_RX_PIN`, input), `in.spdif.enable` (`INPUT.SPDIF_RX_ENABLE`, input 1 to 3, and its GET moves to the same command), `in.i2s.pin` (`INPUT.I2S_RX_PIN`, pair), `i2s.bck` and `i2s.bck.slave` (`CLOCK.I2S_BCK_PIN`, role 0 or 1). The rest take `wValue` 0. They are `in.i2s.channels` (`INPUT.I2S_RX_CHANNELS`), `in.adat.pin` (`INPUT.ADAT_RX_PIN`), `i2s.clockpins` (`CLOCK.I2S_CLOCK_PIN_MODE`), `i2s.mck` (`CLOCK.MCK_ENABLE`), `i2s.mck.pin` (`CLOCK.MCK_PIN`), `i2s.mck.mult` (`CLOCK.MCK_MULTIPLIER`), `adat.enable` (`OUTPUT.ADAT_TX_ENABLE`), `adat.pin` (`OUTPUT.ADAT_TX_PIN`) and `diag.buffers.reset` (`SYS.STATS_RESET`, mask 1 as the payload).
- **Write-as-reads with no value** become zero-length SETs. They are `vol.master.save` (`ROUTE.MASTER_VOLUME_SAVE`), `dev.dacmute.test` (`OUTPUT.DAC_MUTE_TEST`), `dev.reset` (`STORE.FACTORY_RESET`), `dev.bootloader` (`SYS.ENTER_BOOTLOADER`), `dev.save.io` (`STORE.OUTPUT_CONFIG_SAVE`), `preset.save`, `preset.load` and `preset.delete` (`STORE.PRESET_SAVE`, `STORE.PRESET_LOAD`, `STORE.PRESET_DELETE`, slot in `wValue`), `cs.save` (`CS.SAVE`), `cs.revert` (`CS.REVERT`), `cs.macro.fire` (`CS.MACRO_FIRE`, with `0xFFFF` becoming `CS.MACRO_CANCEL` at `wValue` 0) and `meters.clear` (`MEAS.CLIP_CLEAR`).
- **Choice rows that pick an action.** `sig.control` becomes `MEAS.SIGGEN_STOP`, `MEAS.SIGGEN_START` or `MEAS.SIGGEN_STOP_NOW`. `rta.control` becomes `MEAS.RTA_STOP`, `MEAS.RTA_START` or `MEAS.RTA_RESET`. `cs.ir.learn` becomes `CS.IR_LEARN_ARM` or `CS.IR_LEARN_CANCEL`, and its read is `CS.IR_LEARN_STATUS`. Each is one row per action or a row that maps its choice to a command, since a single `set` field no longer covers it.
- **Selector rows.** The four `limit.*` parameters become `DYN.LIMITER_ENABLE`, `DYN.LIMITER_THRESHOLD`, `DYN.LIMITER_RELEASE` and `DYN.LIMITER_LINK_GROUP`, with `wValue` = output (`0xFF` on a SET only). `limit.meter` and `limit.status` become `DYN.LIMITER_METERS` and `DYN.LIMITER_STATUS` with `wValue` 0. The 14 `tube.*` rows become `FX.TUBE_ENABLE` to `FX.TUBE_TRIM` and the 14 `up.*` rows `FX.UPMIX_ENABLE` to `FX.UPMIX_PRESENCE`, all with `wValue` 0. `cs.caps`, `cs.aux.all`, `sig.caps` and `meters` lose their `Fixed` selectors and become `CS.CAPS`, `CS.AUX_ALL`, `MEAS.SIGGEN_CAPS` and `MEAS.LEVELS` with `wValue` 0. The noun and signal-type reads in `probe_cs_caps` and `probe_siggen_caps` move to `CS.NOUN_DESC` and `MEAS.SIGGEN_TYPE_DESC`.
- **`ParamDesc::repr`** forces float payloads by matching on the tube, upmixer and limiter opcodes, an 8.8 payload on the aux level and one byte on the crossfeed outputs. Re-key it on the new IDs, or derive the width from `set_min` and `get_len`.
- **EQ and matrix.** `FILTER.BAND` and `ROUTE.MATRIX_ROUTE` are rewritten as in 6.5. `probe_band_count` binary-searches bands on channel 0 by STALL. Gap bands still STALL with `VC_ERR_ADDRESS`, so it ports to `FILTER.BAND` at `wValue` `band`, but it should accept only that reason.
- **Feature probes.** `probe_features` sends 20 GETs and records a STALL as "absent". Replace it with bitmap tests from `SYS.PAGE_COMMANDS` and keep the names that `Requires::Feature` uses.

| Feature name | 1.1.7 command whose GET bit to test |
|---|---|
| `input_source`, `spdif_multi_input`, `i2s_input_channels`, `adat_input`, `lg_sound_sync` | `INPUT.SOURCE`, `INPUT.SPDIF_RX_CONFIG`, `INPUT.I2S_RX_CHANNELS`, `INPUT.ADAT_RX_ENABLE`, `INPUT.LG_SOUND_SYNC_ENABLE` |
| `adat_output`, `dac_hardware_mute` | `OUTPUT.ADAT_TX_ENABLE`, `OUTPUT.DAC_MUTE_CONFIG` |
| `i2s_slave_clock` | `CLOCK.I2S_CLOCK_MODE` |
| `loudness_output_mask`, `leveller_masks`, `output_limiter` | `DYN.LOUDNESS_MASK`, `DYN.LEVELLER_MASKS`, `DYN.LIMITER_STATUS` |
| `crossfeed_output_mask`, `psychoacoustic_bass`, `upmixer`, `subharmonic_synth`, `tube_preamp` | `FX.CROSSFEED_OUTPUTS`, `FX.PSYBASS_ENABLE`, `FX.UPMIX_STATUS`, `FX.SUBHARM_ENABLE`, `FX.TUBE_ENABLE` |
| `test_signals`, `spectrum_analyser` | `MEAS.SIGGEN_STATUS`, `MEAS.RTA_CAPS` |
| `uart_control`, `i2c_control` | `XPORT.UART_CONFIG`, `XPORT.I2C_CONFIG` |

`Requires::Rp2350` on the `up.*` rows becomes the same kind of test, since the upmixer and ADAT commands are absent on RP2040 (4.8).

- **Retired rows to delete.** `pre.all` (0x44/0x45 `SET/GET_PREAMP`, use the per-input `pre` row on `ROUTE.PREAMP`), `dev.save` (0x51 `SAVE_PARAMS`, read `STORE.PRESET_ACTIVE` then send `STORE.PRESET_SAVE`), `diag.usb` and `diag.usb.reset` (0xB2/0xB3), `legacy.gain` and `legacy.mute` (0x54 to 0x57, see 13.4.8), and the Stats field read from `GET_STATUS` selector 7.

#### 13.4.6 Replies and checks that change

`UsbTransport::control_in` returns `ShortRead` whenever a reply is shorter than `wLength`, and the readout table `shape()` in `crates/dspi-session/src/readout.rs` asks for `Exact(n)` on most status and packet rows. These checks change.

| Host check | Today | 1.1.7 |
|---|---|---|
| `PresetStartup::SIZE` and `decode` (`crates/dspi-proto/src/packets.rs`), readout `Exact(3)` for `preset.startup`, `SettingsData::read` | 3 bytes `{mode, slot, last active}` | `STORE.STARTUP` answers 2 bytes `{mode, slot}`. The last active slot is byte 4 of `STORE.DIRECTORY` (7 bytes) |
| `CsDisplayCfgReply::SIZE` (`crates/dspi-proto/src/packets.rs`), `probe_cs_caps`, `crates/dspi-session/src/surfaces.rs`, readout `Exact(16)` for `cs.display` | 16 bytes | `CS.DISPLAY_CAPS` answers 4 and `CS.DISPLAY_CONFIG` answers 12. A 16-byte read of either is a `ShortRead` |
| `read_band` (`crates/dspi-session/src/write.rs`), `decode_read` for `EqScalar` | Five or six 4-byte reads per band | One 16-byte `FILTER.BAND` read `{type, bypass, u16 qp_x512, f32 freq, f32 Q, f32 gain}` |
| Crosspoint readback in `Session::read`, `MatrixRoutePacket` | 8 bytes, enabled at byte 2 and invert at byte 3 | 8 bytes, enabled at byte 0, invert at byte 1, bytes 2 and 3 reserved |
| `status_word` in `crates/dspi-tui/src/actions.rs`, `read_sample_rate` in `crates/dspi-tui/src/settings/mod.rs` | One 4-byte u32 per `GET_STATUS` selector | `SYS.SYSTEM_STATUS` (16), `SYS.PDM_STATS` (16), `SYS.SPDIF_TX_STATS` (24), `SYS.USB_AUDIO_STATUS` (16) at the offsets in 6.3. Core voltage is a u16 at offset 8 and temperature an i16 at offset 10, so sign-extend it instead of casting a u32 `as i32` |
| `in.rate` readback | First u32 of the 8-byte reply, the current rate | `INPUT.I2S_RX_RATE` answers 4 bytes, the selected rate |
| `in.spdif.enable` readback | 1 byte of the 6-byte S/PDIF config, which is the input count | `INPUT.SPDIF_RX_ENABLE` answers 1 byte for the addressed input |
| `cs.macro.step` readback, readout `Exact(132)` at `wValue` = macro | Whole macro | Unchanged on `CS.MACRO` (132). If you move it to `CS.MACRO_STEP`, ask for 12 bytes at `(step << 8) \| macro` and change `read_wvalue_for` to match |
| `RtaCaps::centre_chunks` (`crates/dspi-proto/src/packets.rs`), `RtaEngine::connect` (`crates/dspi-session/src/rta.rs`) | Chunks at `wValue` 1, 2, ... | `MEAS.RTA_BAND_CENTRES` at `wValue` n - 1. With 37 bands the replies are 64 and 10 bytes |
| Bin frame loop in `crates/dspi-session/src/rta.rs` | Up to the whole frame per read | `MEAS.RTA_BINS` returns at most 256 bytes per read. The loop already advances by what arrives |
| Every write-as-read (`Session::write`, `load_preset`, `crates/dspi-session/src/surfaces.rs`, `raw_war`) | A 1-byte status reply | No reply. Success is the ACK |

These stay strict and correct. `SYS.PLATFORM` is 7 bytes in the same layout, and `MEAS.LEVELS` is `2 x channels + 7` bytes (41 on RP2350, 21 on RP2040) with identical bytes. `CS.STATUS` is 41, `CS.CAPS` 52, `CS.AUX_ALL` 48, `SYS.SERIAL` 16, `SYS.BUILD_INFO` 64 and `SYS.BUFFER_STATS` 44. `FX.SUBHARM_METER` and `DYN.LIMITER_METERS` are one u16 per output (18 or 10 bytes). The blob checks in `read_bulk_from` and `crates/dspi-proto/src/wire.rs` (byte 0 equal to 32, 6136 bytes) pass unchanged.

#### 13.4.7 Errors, status and busy handling

**STALL means refused.** Today a STALL is `TransportError::Stalled { opcode }`, whose text names an 8-bit opcode and no reason. Carry the `CmdId` and the result code from `SYS.LAST_RESULT` instead, and print `PAGE.NAME` and the `VC_ERR_*` name. Read `SYS.LAST_RESULT` (page `0x01`, command `0x14`, 8 bytes) straight after the STALL. It returns `{u16 command ID, u8 direction (1 GET, 0 SET), u8 result, u16 wValue, u16 reserved}`. The record is kept per transport, not per program. Another program that reaches the device over USB can overwrite it, and so can any request the Terminal itself sends in between. Trust the code only when the ID, direction and `wValue` all match the request that stalled. Otherwise report the STALL without a reason.

**Retry only busy.** `with_busy_retry` retries `Stalled` and `TimedOut` four more times, 150 ms apart, on the theory that a STALL may be a flash blackout. In 1.1.7 a STALL is final except `VC_ERR_BUSY` (`0x17`), and the busy window shows as a timeout (5.8). Retry timeouts as now. Retry a STALL only when its matched result is `VC_ERR_BUSY`. Otherwise every refused GET costs about 600 ms before it fails.

**Status bytes become result codes.** Code that read the write-as-read byte from `Session::last_write_status()` must switch to the result code. That covers `attempt()` and the PDM pin retry in `crates/dspi-session/src/preset_file.rs`, the pin reset loop in `crates/dspi-tui/src/settings/outputs.rs`, `crates/dspi-tui/src/settings/inputs.rs`, the generic pin and clock write in `crates/dspi-tui/src/settings/mod.rs`, and `load_preset` in `crates/dspi-tui/src/live.rs`, which today requires a status byte of 0. `describe_pin_status` explains `PIN_CONFIG_*` codes. Feed it through `status_families.pin.from_vc` in `Documentation/protocol/legacy_map.json`, which maps `VC_ERR_PIN_INVALID` to 1, `VC_ERR_PIN_IN_USE` to 2, `VC_ERR_STATE` to 4 and `VC_ERR_VALUE` to 5, or rewrite the explanations against the `VC_*` codes (5.4). Many refusals that used to end as `Outcome::Rejected` after a readback now arrive as an error before any readback, so give `WriteError` a refusal variant with the result code. Decide whether `dspi` exits 3 (rejected) or 4 (transport) for it, since today a STALL exits 4.

**Control Surfaces.** `Surfaces::send` writes, then polls `CS.STATUS` and re-sends once on the status packet's `BUSY` (`0x1B`). In 1.1.7 a busy edit STALLs with `VC_ERR_BUSY` and still posts `0x1B`, so `send` gets an error before it polls. A bad slot or length STALLs without touching the status packet, so the poll would only time out. Check the STALL first. On `VC_ERR_BUSY` run `wait_idle` and re-send once, as now, and report any other code at once. `aux_outcome` needs the same error path, because an aux SET on a non-aux slot STALLs with `VC_ERR_STATE` (and still posts `INVALID_AUX`). The constants in `crates/dspi-session/src/surfaces.rs` `mod status` are Control Surfaces status codes, not `VC_*` codes. There `0x16` means pending and `0x1B` busy, while in the `VC_*` space `0x16` is `VC_ERR_BLOCKED` and busy is `0x17`, so keep the two apart.

**EQ busy.** A second `FILTER.BAND` or `FILTER.BAND_BYPASS` before the first is applied STALLs with `VC_ERR_BUSY`. The TUI sends band edits up to 30 times a second while a key is held, and `write_band` reads back at once. Retry the write after 1 to 2 ms, and confirm from the host copy or by reading until the band matches (6.5, rule 3).

#### 13.4.8 Host-specific hazards

- **Narrowing.** A single `as u8` left on the request path sends page 0 (13.4.3). Add a test that drives `UsbTransport`'s field mapping, or a mock keyed on the full ID, so that a dropped page fails a test instead of passing it.
- **`dspi raw` and old firmware.** `raw_in`, `raw_out` and `raw_war` send any number with no probe. Give `raw` a page. Accept `PAGE.NAME`, a bare name when unambiguous, `page.cmd` or a 16-bit ID, resolved against the generated `COMMANDS`, and refuse page 0 and any bare 8-bit number without a page. Without that check, `dspi raw 0x0610 16` today truncates to `bRequest` `0x10` on page 0. Probe first and refuse old firmware unless the user passes an override, since on 1.1.6 a paged number runs an unrelated old command (4.1). Take default GET lengths from `get_len`, check SET lengths against `set_min` and `set_max` unless the user opts out, and drop `raw war`. Update the usage text in `crates/dspi/src/main.rs`, which shows `dspi raw out 0x42 0 08 03 01 00` and `dspi raw war 0xC2 0x010E`, and the `dspi raw 0x87 41` example in `README.md`.
- **Readers that swallow errors.** `status_word`, `packet` in `crates/dspi-tui/src/actions.rs` and the `get` closures in `crates/dspi-tui/src/settings/mod.rs` turn every error into "absent". A wrong page or length on a half-ported build shows as an empty Stats panel or a missing settings section, not as a failure. Check each one against a 1.1.7 device rather than trusting a quiet screen.
- **Pre-existing packing bug.** `in.spdif.pin`, `in.spdif.enable` and `in.i2s.pin` use `WValue::ValueIndex`, which `wvalue_for` packs as `(value << 8) | index`. 1.1.6 decoded all three as `(index << 8) | value`, so `dspi in.spdif.pin 0 2` sent `0x0200`, which 1.1.6 read as input 2, GPIO 0. Only `out.pin` really is `(GPIO << 8) | index`. Porting these rows from section 6.2 puts the index in `wValue` and the value in the payload, which fixes them. The `wvalue_for` tests in `crates/dspi-session/src/write.rs` that pin the old packing must be rewritten.
- **Pre-existing direction bug.** `in.adat.enable` and `in.adat.clock` are declared OUT with a 1-byte payload, but 1.1.6 handled those opcodes only as write-as-reads, so the OUT was refused. `INPUT.ADAT_RX_ENABLE` and `INPUT.ADAT_RX_CLOCK_MODE` are SETs of exactly that shape, so these two rows only need renumbering. Both are RP2350 only, so `.dspipreset` import must skip `in.adat.clock` on RP2040 instead of relying on 1.1.6 accepting it.
- **Pre-existing readback bugs.** The `in.spdif.enable` readback reads the input count, which is always non-zero, so a refused enable was never caught. The `in.rate` readback compares the current rate with the selected one. Both are fixed by the 1.1.7 commands in 13.4.6.
- **Reassigned legacy rows.** `legacy.gain` and `legacy.mute` send 0x54 to 0x57, which 1.1.6 final reused for input source names and the device name, so on a 1.1.6 final device `dspi legacy.mute 0 on` renames the device. They have no 1.1.7 command and are not in `legacy_map.json`. Delete them.
- **RP2040 features.** 1.1.6 on RP2040 answered zeros for the ADAT output and upmixer probes, so the Terminal shows both as present there. With bitmaps they are correctly absent. Expect the RP2040 UI to lose those sections.
- **Control Surfaces caps.** Re-vendoring `control_surfaces.h` moves the caps version from 20 to 22. The test `the_noun_and_type_tables_match_the_vendored_header` will fail until `CS_NOUNS` and `CS_TYPES` in `crates/dspi-proto/src/packets.rs` are extended.

#### 13.4.9 Porting order and tests

1. Vendor `dspi_commands.json` and the headers, and generate `CmdId` constants and `COMMANDS` (13.4.2, 13.4.5).
2. Widen the trait, `UsbTransport`, `TransportError`, `with_busy_retry` and `MockTransport` to `CmdId`, with `SYS.LAST_RESULT` read after a STALL. Fix every compile error the new constant type raises.
3. Put the probe at the top of `probe()` and give `dspi raw` the same guard.
4. Renumber the bulk chunk read, so the TUI connects and draws from the blob.
5. Renumber the roughly 110 plain rows.
6. Convert the write-as-reads and their status consumers.
7. Port the splits, then `FILTER.BAND` and the matrix, then `Surfaces::send`, then the Stats structs.
8. Replace the feature probes with bitmaps, delete the retired rows, and update the help text, `README.md`, `docs/firmware-notes.md` and `docs/firmware-bump.md`.

For tests, re-key the mock on `CmdId` and make it answer `SYS.LAST_RESULT` after a scripted STALL. More than 500 `REQ_*` references sit in test modules, heaviest in `crates/dspi-session/src/preset_file.rs`, `crates/dspi-tui/src/live.rs`, `crates/dspi-session/src/surfaces.rs` and `crates/dspi-session/src/rta.rs`. Rewrite the tests that pin 1.1.6 behaviour, such as the short `GET_PLATFORM` replies, the limiter probe's `wValue` `0x0081`, the CS status and busy sequences, the EQ scalar reads, the crosspoint bytes and the write-as-read status codes. Add a mock that STALLs `SYS.PROTOCOL_INFO` and check that `probe()` then sends exactly one old `GET_PLATFORM` and stops. On hardware, follow section 12.1 on both generations. Hold an EQ key in the TUI to exercise `VC_ERR_BUSY`, run `dspi doctor` against a 1.1.6 device, and send `dspi raw` with a bare 8-bit number to check that it is refused.
