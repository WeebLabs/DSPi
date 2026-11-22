# DSPi Client Update Guide: Firmware 1.1.7

*Last updated: 2026-11-22*

This is the short version of the host migration guide. It covers only what a client must change to work with firmware 1.1.7. The full guide, [`vendor_protocol_migration_guide.md`](vendor_protocol_migration_guide.md), adds worked examples, every reply layout, transport detail and appendices for the ESP32 front panel, DSPi Console for macOS, DSPi Link and DSPi Terminal. Links in the tables below lead to it when this file has no matching section.

---

## 1. What changed

In 1.1.6 every command was one 8-bit number in `bRequest`. In 1.1.7 every command has a two-part address. The **page** names a subsystem and travels in the high byte of `wIndex`. The **command number** names one thing inside that page and travels in `bRequest`. A command is written `PAGE.NAME`, and its 16-bit ID is `(page << 8) | command`.

| Field | 1.1.7 |
|---|---|
| `bmRequestType` | `0xC1` for a GET, `0x41` for a SET, as before |
| `bRequest` | The command number, `0x10` to `0xFF` |
| `wIndex` | `(page << 8) \| 2` on USB. The low byte is still the vendor interface, 2. On UART and I2C the low byte may be 0 |
| `wValue` | The address only (channel, output, slot, band), or 0 |
| `wLength` and payload | As before. Values that used to ride in `wValue` now go in the payload |

Set the gain of output 1 to -6.0 dB, then read it back.

```
1.1.6  SET_OUTPUT_GAIN 0x74          SETUP 41 74 01 00 02 00 04 00   OUT 00 00 C0 C0
1.1.7  ROUTE.OUTPUT_GAIN 0x0521      SETUP 41 21 01 00 02 05 04 00   OUT 00 00 C0 C0
1.1.7  read it back                  SETUP C1 21 01 00 02 05 04 00   IN  00 00 C0 C0
```

Every 1.1.6 request has page 0, and page 0 has no commands. A 1.1.6 client therefore fails on every request against 1.1.7, but it changes nothing.

These are unchanged: the VID and PID, the interface numbers, the USB audio class, the Microsoft OS 2.0 descriptor, the notification endpoint and packets, and the UART and I2C frame format and CRC. Most GET replies are also byte-for-byte the same. Section 7 lists the ones that changed.

The bulk parameter blob grew. `WireBulkParams` was version 32 and 6136 bytes in 1.1.6. In 1.1.7 it is version 35 and 6252 bytes on both platforms, with a 116-byte upmixer section appended at offset 6136. The earlier sections keep their offsets. A bulk SET must match the device's version and length exactly, so regenerate your struct.

---

## 2. Probe before anything else

Send a GET of `SYS.PROTOCOL_INFO` (page `0x01`, command `0x10`, 16 bytes) as the first request after connecting.

```
SETUP  C1 10 00 00 02 01 10 00
IN     44 53 50 69 01 00 0C 00 40 00 00 01 00 00 00 00
```

- **The reply starts with `DSPi`.** The device speaks the paged protocol. Byte 4 is the protocol major, which is 1. If it is anything else, tell the user to update the client and send nothing more.
- **The probe STALLs, or UART and I2C answer status `0x02`.** The device runs 1.1.6 or older. Send it nothing else except, if you need them, the old `GET_PLATFORM` (`C1 7F 00 00 02 00 07 00`) to show its version and the old `ENTER_BOOTLOADER` (`C1 F0 00 00 02 00 01 00`) to start a firmware update.

The probe is mandatory because old firmware ignores the page byte. It would run whatever 1.1.6 opcode has the same number as your command. Some of those numbers belong to factory reset, preset save, preset load and preset delete, so an unprobed paged read can wipe a 1.1.6 device. The probe itself is safe on both generations.

**Version.** `SYS.PLATFORM` (`0x0120`, 7 bytes) has exactly the layout of the old `GET_PLATFORM` reply. Bytes 1, 4, 5 and 6 are major, minor, patch and beta.

**Feature detection.** Do not gate features on the version number, and do not probe by STALL. `SYS.PAGE_COMMANDS` (`0x0112`, `wValue` = page, 64 bytes) is a bitmap of the commands a page implements. Command `n` has a GET when bit `n & 7` of byte `n >> 3` is set, and a SET when the same bit of byte `32 + (n >> 3)` is set. Read it once per page at connect. The ADAT and upmixer commands do not exist on RP2040, and the `DIAG` page does not exist in release builds. Requests to them are refused with `VC_ERR_UNKNOWN`.

---

## 3. Rules for every request

1. **`wValue` is an address or 0.** A command that addresses nothing requires `wValue` 0. An addressed command checks all 16 bits, so a stray high byte is refused. `wValue` never carries a value or picks an operation.
2. **Values go in the payload.** A pin number, mode, enable flag or gain that 1.1.6 packed into `wValue` is now the SET payload.
3. **Send the exact payload length.** 1.1.6 accepted longer buffers and ignored the rest. 1.1.7 refuses any length the command does not accept.
4. **Write-as-reads are now SETs.** 1.1.6 had 34 commands that changed state through an IN transfer and answered a status byte. In 1.1.7 each one is a SET. An action with no value, such as loading a preset, is a SET with `wLength` 0. Reads never change anything.
5. **Widen opcode variables to 16 bits.** A helper, struct field or table column that keeps an 8-bit opcode drops the page, so the request lands on page 0 and is refused. Even the probe then STALLs and looks like old firmware.
6. **Re-key tables on the 16-bit ID.** Any policy, cache, router or log keyed on `bRequest` alone must use `(page, command)`. Different pages reuse the same command numbers.

---

## 4. Errors and retries

**A refusal is explicit.** On USB a refused request STALLs. On UART and I2C the status byte of the response carries the reason. A refused request changes nothing. 1.1.6 accepted most bad writes silently, so code that assumed every SET succeeds must now handle failure.

**Why it was refused.** On USB, read `SYS.LAST_RESULT` (`0x0114`, 8 bytes) straight after the STALL. Check that its command ID, direction and `wValue` match your request before you trust the code, because the record is shared by every USB program on the device.

| Offset | Type | Field |
|---|---|---|
| 0 | u16 | Command ID of the recorded request |
| 2 | u8 | Direction: 1 GET, 0 SET |
| 3 | u8 | Result code |
| 4 | u16 | `wValue` of the recorded request |
| 6 | u16 | Reserved, 0 |

| Code | Name | Meaning |
|---|---|---|
| `0x00` | `VC_OK` | Done, or accepted and queued |
| `0x10` | `VC_ERR_UNKNOWN` | No such page or command on this build |
| `0x11` | `VC_ERR_DIRECTION` | The command has no GET, or no SET |
| `0x12` | `VC_ERR_LENGTH` | Wrong SET payload length |
| `0x13` | `VC_ERR_ADDRESS` | `wValue` out of range |
| `0x14` | `VC_ERR_VALUE` | A payload value out of range, or a NaN or infinite float |
| `0x15` | `VC_ERR_UNSUPPORTED` | A valid value this platform cannot do |
| `0x16` | `VC_ERR_BLOCKED` | Not allowed on this transport (USB-only command over UART or I2C) |
| `0x17` | `VC_ERR_BUSY` | Try again shortly |
| `0x18` | `VC_ERR_STATE` | Not allowed in the current state |
| `0x19` | `VC_ERR_PIN_INVALID` | The GPIO cannot be used for this function |
| `0x1A` | `VC_ERR_PIN_IN_USE` | The GPIO is already claimed |

**Retry only what is retryable.** Retry unchanged only on `VC_ERR_BUSY`, and on UART and I2C also on status `0x01` and `0x04`. Two quick `FILTER.BAND` writes are the common case: the second is refused as busy until the first is applied, about 1 to 2 ms later.

**The flash busy window.** A write that saves to flash or restarts the outputs (presets, saves, pin and output type changes, factory reset) can make the device miss requests for tens of milliseconds. It shows up as a timeout, not a STALL. On a timeout, wait about 150 ms and retry, up to about four times. This is the same as 1.1.6.

**Deferred writes.** Many writes are accepted at once and applied by the main loop a moment later. `VC_OK` means the write is queued. Confirm the result by reading the setting back.

**UART and I2C.** The frame format and CRC are unchanged, and `wIdxH` carries the page. The status byte now uses `0x00` for success, `0x01` and `0x04` for busy, `0x05` to `0x07` for frame errors, and the result codes above from `0x10` up. A client that treats the status as a closed set of `0x00` to `0x07` must accept the new codes. Size the receive buffer for 256 bytes, up from 132.

---

## 5. Old-to-new map

Each row covers one 1.1.6 opcode and direction, or one selector value of an opcode that was split. Columns:

- **Old dir.** `W` was an OUT transfer, `R` an IN transfer, and `W-as-R` an IN transfer that changed state.
- **Old wValue.** Which old requests the row covers. `old & 0xFF = 2` means "old requests whose `wValue` low byte was 2".
- **Change.** `same` and `folded` mean only the number changed. `action` means a write-as-read became a SET. `split` means one opcode became several commands. `custom` means the payload or reply changed shape, and section 6 explains it. `retired` means there is no new command.
- **ID.** The 16-bit ID. Send the high byte as the `wIndex` high byte and the low byte as `bRequest`.
- **New dir.** `G` a GET, `S` a SET with payload, `S0` a SET with no payload.
- **New wValue.** An expression of the old `wValue`, called `old`. `0` means the constant 0.
- **Payload.** For a SET, `unchanged` means the 1.1.6 payload bytes. For a former write-as-read, it is the 1-byte payload built from the old `wValue`.
- **Bytes.** The exact GET reply length or SET payload length. `var` replies are explained in the full guide.

<!-- VC:BEGIN LEGACY_MAP -->

| Old | Old name | Old dir | Old wValue | Change | New command | ID | New dir | New wValue | Payload | Bytes |
|---|---|---|---|---|---|---|---|---|---|---|
| 0x03 | [`OUTPUT_LATENCY`](vendor_protocol_migration_guide.md#output_latency-0x03) | W | any | same | `OUTPUT.LATENCY` | `0x0420` | S | `0` | unchanged | 1 |
| 0x03 | [`OUTPUT_LATENCY`](vendor_protocol_migration_guide.md#output_latency-0x03) | R | 0 | split | `OUTPUT.LATENCY_STATUS` | `0x0421` | G | `0` | - | 12 |
| 0x03 | [`OUTPUT_LATENCY`](vendor_protocol_migration_guide.md#output_latency-0x03) | R | 1 | split | `OUTPUT.LATENCY_OPTIONS` | `0x0422` | G | `0` | - | var |
| 0x03 | [`OUTPUT_LATENCY`](vendor_protocol_migration_guide.md#output_latency-0x03) | R | any other | split | none | - | - | - | 1.1.6 STALLed it | - |
| 0x04 | `SET_CS_AUX_STATE` | W | any | folded | `CS.AUX_STATE` | `0x0A60` | S | `old & 0xFF` | unchanged | 1 |
| 0x05 | [`GET_CS_AUX_STATE`](vendor_protocol_migration_guide.md#get_cs_aux_state-0x05) | R | 0xFFFF | split | `CS.AUX_ALL` | `0x0A62` | G | `0` | - | 48 |
| 0x05 | [`GET_CS_AUX_STATE`](vendor_protocol_migration_guide.md#get_cs_aux_state-0x05) | R | any other | split | `CS.AUX_STATE` | `0x0A60` | G | `old` | - | 1 |
| 0x06 | `SET_CS_AUX_LEVEL` | W | any | folded | `CS.AUX_LEVEL` | `0x0A61` | S | `old & 0xFF` | unchanged | 2 |
| 0x07 | `GET_CS_AUX_LEVEL` | R | any | folded | `CS.AUX_LEVEL` | `0x0A61` | G | `old` | - | 2 |
| 0x08 | `RTA_SET_CONFIG` | W | any | folded | `MEAS.RTA_CONFIG` | `0x0920` | S | `0` | unchanged | 12 |
| 0x09 | `RTA_GET_CONFIG` | R | any | folded | `MEAS.RTA_CONFIG` | `0x0920` | G | `0` | - | 12 |
| 0x0A | [`RTA_GET_CAPS`](vendor_protocol_migration_guide.md#rta_get_caps-0x0a) | R | 0 | split | `MEAS.RTA_CAPS` | `0x0921` | G | `0` | - | 16 |
| 0x0A | [`RTA_GET_CAPS`](vendor_protocol_migration_guide.md#rta_get_caps-0x0a) | R | any other | split | `MEAS.RTA_BAND_CENTRES` | `0x0922` | G | `old - 1` | - | var |
| 0x0B | `RTA_GET_BANDS` | R | any | same | `MEAS.RTA_BANDS` | `0x0923` | G | `old` | - | 82 |
| 0x0C | `RTA_GET_BINS` | R | any | same | `MEAS.RTA_BINS` | `0x0924` | G | `old` | - | var |
| 0x0D | `RTA_GET_STATUS` | R | any | same | `MEAS.RTA_STATUS` | `0x0925` | G | `0` | - | 24 |
| 0x0E | [`RTA_CONTROL`](vendor_protocol_migration_guide.md#rta_control-0x0e-and-siggen_control-0xa6) | W-as-R | old & 0xFF = 0 | split | `MEAS.RTA_STOP` | `0x0929` | S0 | `0` | none | 0 |
| 0x0E | [`RTA_CONTROL`](vendor_protocol_migration_guide.md#rta_control-0x0e-and-siggen_control-0xa6) | W-as-R | old & 0xFF = 1 | split | `MEAS.RTA_START` | `0x0928` | S0 | `0` | none | 0 |
| 0x0E | [`RTA_CONTROL`](vendor_protocol_migration_guide.md#rta_control-0x0e-and-siggen_control-0xa6) | W-as-R | old & 0xFF = 2 | split | `MEAS.RTA_RESET` | `0x092A` | S0 | `0` | none | 0 |
| 0x0E | [`RTA_CONTROL`](vendor_protocol_migration_guide.md#rta_control-0x0e-and-siggen_control-0xa6) | W-as-R | any other | split | none | - | - | - | 1.1.6 STALLed it | - |
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
| 0x25 | [`CS_MACRO_FIRE`](vendor_protocol_migration_guide.md#cs_macro_fire-0x25) | W-as-R | 0xFFFF | split | `CS.MACRO_CANCEL` | `0x0A59` | S0 | `0` | none | 0 |
| 0x25 | [`CS_MACRO_FIRE`](vendor_protocol_migration_guide.md#cs_macro_fire-0x25) | W-as-R | any other | split | `CS.MACRO_FIRE` | `0x0A58` | S0 | `old` | none | 0 |
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
| 0x3E | [`SET_TUBE_PARAM`](vendor_protocol_migration_guide.md#set_tube_param-and-get_tube_param-0x3e-0x3f) | W | old & 0xFF = 0 | split | `FX.TUBE_ENABLE` | `0x0860` | S | `0` | unchanged | 4 |
| 0x3E | [`SET_TUBE_PARAM`](vendor_protocol_migration_guide.md#set_tube_param-and-get_tube_param-0x3e-0x3f) | W | old & 0xFF = 1 | split | `FX.TUBE_OUTPUT_MASK` | `0x0861` | S | `0` | unchanged | 4 |
| 0x3E | [`SET_TUBE_PARAM`](vendor_protocol_migration_guide.md#set_tube_param-and-get_tube_param-0x3e-0x3f) | W | old & 0xFF = 2 | split | `FX.TUBE_TYPE` | `0x0862` | S | `0` | unchanged | 4 |
| 0x3E | [`SET_TUBE_PARAM`](vendor_protocol_migration_guide.md#set_tube_param-and-get_tube_param-0x3e-0x3f) | W | old & 0xFF = 3 | split | `FX.TUBE_DRIVE` | `0x0863` | S | `0` | unchanged | 4 |
| 0x3E | [`SET_TUBE_PARAM`](vendor_protocol_migration_guide.md#set_tube_param-and-get_tube_param-0x3e-0x3f) | W | old & 0xFF = 4 | split | `FX.TUBE_BIAS` | `0x0864` | S | `0` | unchanged | 4 |
| 0x3E | [`SET_TUBE_PARAM`](vendor_protocol_migration_guide.md#set_tube_param-and-get_tube_param-0x3e-0x3f) | W | old & 0xFF = 5 | split | `FX.TUBE_ASYM` | `0x0865` | S | `0` | unchanged | 4 |
| 0x3E | [`SET_TUBE_PARAM`](vendor_protocol_migration_guide.md#set_tube_param-and-get_tube_param-0x3e-0x3f) | W | old & 0xFF = 6 | split | `FX.TUBE_HARDNESS` | `0x0866` | S | `0` | unchanged | 4 |
| 0x3E | [`SET_TUBE_PARAM`](vendor_protocol_migration_guide.md#set_tube_param-and-get_tube_param-0x3e-0x3f) | W | old & 0xFF = 7 | split | `FX.TUBE_SAG` | `0x0867` | S | `0` | unchanged | 4 |
| 0x3E | [`SET_TUBE_PARAM`](vendor_protocol_migration_guide.md#set_tube_param-and-get_tube_param-0x3e-0x3f) | W | old & 0xFF = 8 | split | `FX.TUBE_RECTIFIER` | `0x0868` | S | `0` | unchanged | 4 |
| 0x3E | [`SET_TUBE_PARAM`](vendor_protocol_migration_guide.md#set_tube_param-and-get_tube_param-0x3e-0x3f) | W | old & 0xFF = 9 | split | `FX.TUBE_XFMR_ENABLE` | `0x0869` | S | `0` | unchanged | 4 |
| 0x3E | [`SET_TUBE_PARAM`](vendor_protocol_migration_guide.md#set_tube_param-and-get_tube_param-0x3e-0x3f) | W | old & 0xFF = 10 | split | `FX.TUBE_XFMR_DAMPING` | `0x086A` | S | `0` | unchanged | 4 |
| 0x3E | [`SET_TUBE_PARAM`](vendor_protocol_migration_guide.md#set_tube_param-and-get_tube_param-0x3e-0x3f) | W | old & 0xFF = 11 | split | `FX.TUBE_XFMR_RES` | `0x086B` | S | `0` | unchanged | 4 |
| 0x3E | [`SET_TUBE_PARAM`](vendor_protocol_migration_guide.md#set_tube_param-and-get_tube_param-0x3e-0x3f) | W | old & 0xFF = 12 | split | `FX.TUBE_MIX` | `0x086C` | S | `0` | unchanged | 4 |
| 0x3E | [`SET_TUBE_PARAM`](vendor_protocol_migration_guide.md#set_tube_param-and-get_tube_param-0x3e-0x3f) | W | old & 0xFF = 13 | split | `FX.TUBE_TRIM` | `0x086D` | S | `0` | unchanged | 4 |
| 0x3E | [`SET_TUBE_PARAM`](vendor_protocol_migration_guide.md#set_tube_param-and-get_tube_param-0x3e-0x3f) | W | any other | split | none | - | - | - | 1.1.6 ignored it | - |
| 0x3F | [`GET_TUBE_PARAM`](vendor_protocol_migration_guide.md#set_tube_param-and-get_tube_param-0x3e-0x3f) | R | old & 0xFF = 0 | split | `FX.TUBE_ENABLE` | `0x0860` | G | `0` | - | 4 |
| 0x3F | [`GET_TUBE_PARAM`](vendor_protocol_migration_guide.md#set_tube_param-and-get_tube_param-0x3e-0x3f) | R | old & 0xFF = 1 | split | `FX.TUBE_OUTPUT_MASK` | `0x0861` | G | `0` | - | 4 |
| 0x3F | [`GET_TUBE_PARAM`](vendor_protocol_migration_guide.md#set_tube_param-and-get_tube_param-0x3e-0x3f) | R | old & 0xFF = 2 | split | `FX.TUBE_TYPE` | `0x0862` | G | `0` | - | 4 |
| 0x3F | [`GET_TUBE_PARAM`](vendor_protocol_migration_guide.md#set_tube_param-and-get_tube_param-0x3e-0x3f) | R | old & 0xFF = 3 | split | `FX.TUBE_DRIVE` | `0x0863` | G | `0` | - | 4 |
| 0x3F | [`GET_TUBE_PARAM`](vendor_protocol_migration_guide.md#set_tube_param-and-get_tube_param-0x3e-0x3f) | R | old & 0xFF = 4 | split | `FX.TUBE_BIAS` | `0x0864` | G | `0` | - | 4 |
| 0x3F | [`GET_TUBE_PARAM`](vendor_protocol_migration_guide.md#set_tube_param-and-get_tube_param-0x3e-0x3f) | R | old & 0xFF = 5 | split | `FX.TUBE_ASYM` | `0x0865` | G | `0` | - | 4 |
| 0x3F | [`GET_TUBE_PARAM`](vendor_protocol_migration_guide.md#set_tube_param-and-get_tube_param-0x3e-0x3f) | R | old & 0xFF = 6 | split | `FX.TUBE_HARDNESS` | `0x0866` | G | `0` | - | 4 |
| 0x3F | [`GET_TUBE_PARAM`](vendor_protocol_migration_guide.md#set_tube_param-and-get_tube_param-0x3e-0x3f) | R | old & 0xFF = 7 | split | `FX.TUBE_SAG` | `0x0867` | G | `0` | - | 4 |
| 0x3F | [`GET_TUBE_PARAM`](vendor_protocol_migration_guide.md#set_tube_param-and-get_tube_param-0x3e-0x3f) | R | old & 0xFF = 8 | split | `FX.TUBE_RECTIFIER` | `0x0868` | G | `0` | - | 4 |
| 0x3F | [`GET_TUBE_PARAM`](vendor_protocol_migration_guide.md#set_tube_param-and-get_tube_param-0x3e-0x3f) | R | old & 0xFF = 9 | split | `FX.TUBE_XFMR_ENABLE` | `0x0869` | G | `0` | - | 4 |
| 0x3F | [`GET_TUBE_PARAM`](vendor_protocol_migration_guide.md#set_tube_param-and-get_tube_param-0x3e-0x3f) | R | old & 0xFF = 10 | split | `FX.TUBE_XFMR_DAMPING` | `0x086A` | G | `0` | - | 4 |
| 0x3F | [`GET_TUBE_PARAM`](vendor_protocol_migration_guide.md#set_tube_param-and-get_tube_param-0x3e-0x3f) | R | old & 0xFF = 11 | split | `FX.TUBE_XFMR_RES` | `0x086B` | G | `0` | - | 4 |
| 0x3F | [`GET_TUBE_PARAM`](vendor_protocol_migration_guide.md#set_tube_param-and-get_tube_param-0x3e-0x3f) | R | old & 0xFF = 12 | split | `FX.TUBE_MIX` | `0x086C` | G | `0` | - | 4 |
| 0x3F | [`GET_TUBE_PARAM`](vendor_protocol_migration_guide.md#set_tube_param-and-get_tube_param-0x3e-0x3f) | R | old & 0xFF = 13 | split | `FX.TUBE_TRIM` | `0x086D` | G | `0` | - | 4 |
| 0x3F | [`GET_TUBE_PARAM`](vendor_protocol_migration_guide.md#set_tube_param-and-get_tube_param-0x3e-0x3f) | R | any other | split | none | - | - | - | 1.1.6 STALLed it | - |
| 0x42 | [`SET_EQ_PARAM`](#set_eq_param-and-get_eq_param-0x42-0x43) | W | any | custom | `FILTER.BAND` G, `FILTER.BAND` S | `0x0610`, `0x0610` | - | see recipe | [see recipe](#set_eq_param-and-get_eq_param-0x42-0x43) | see recipe |
| 0x43 | [`GET_EQ_PARAM`](#set_eq_param-and-get_eq_param-0x42-0x43) | R | any | custom | `FILTER.BAND` G | `0x0610` | - | see recipe | [see recipe](#set_eq_param-and-get_eq_param-0x42-0x43) | see recipe |
| 0x44 | `SET_PREAMP` | W | any | retired | none | - | - | - | [replacement](#retired-opcodes) | - |
| 0x45 | `GET_PREAMP` | R | any | retired | none | - | - | - | [replacement](#retired-opcodes) | - |
| 0x46 | `SET_BYPASS` | W | any | folded | `FILTER.EQ_BYPASS` | `0x0612` | S | `0` | unchanged | 1 |
| 0x47 | `GET_BYPASS` | R | any | folded | `FILTER.EQ_BYPASS` | `0x0612` | G | `0` | - | 1 |
| 0x48 | `SET_DELAY` | W | any | folded | `ROUTE.CHANNEL_DELAY` | `0x0530` | S | `old` | unchanged | 4 |
| 0x49 | `GET_DELAY` | R | any | folded | `ROUTE.CHANNEL_DELAY` | `0x0530` | G | `old` | - | 4 |
| 0x4A | `UPMIX_SET_CONFIG` | W | any | folded | `FX.UPMIX_CONFIG` (RP2350 only) | `0x0880` | S | `0` | unchanged | 44 |
| 0x4B | `UPMIX_GET_CONFIG` | R | any | folded | `FX.UPMIX_CONFIG` (RP2350 only) | `0x0880` | G | `0` | - | 44 |
| 0x4C | [`UPMIX_SET_PARAM`](vendor_protocol_migration_guide.md#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | W | 0 | split | `FX.UPMIX_ENABLE` (RP2350 only) | `0x0890` | S | `0` | unchanged | 4 |
| 0x4C | [`UPMIX_SET_PARAM`](vendor_protocol_migration_guide.md#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | W | 1 | split | `FX.UPMIX_CENTER_MODE` (RP2350 only) | `0x0891` | S | `0` | unchanged | 4 |
| 0x4C | [`UPMIX_SET_PARAM`](vendor_protocol_migration_guide.md#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | W | 2 | split | `FX.UPMIX_SURROUND_MODE` (RP2350 only) | `0x0892` | S | `0` | unchanged | 4 |
| 0x4C | [`UPMIX_SET_PARAM`](vendor_protocol_migration_guide.md#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | W | 3 | split | `FX.UPMIX_STRENGTH` (RP2350 only) | `0x0893` | S | `0` | unchanged | 4 |
| 0x4C | [`UPMIX_SET_PARAM`](vendor_protocol_migration_guide.md#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | W | 4 | split | `FX.UPMIX_CENTER_WIDTH` (RP2350 only) | `0x0894` | S | `0` | unchanged | 4 |
| 0x4C | [`UPMIX_SET_PARAM`](vendor_protocol_migration_guide.md#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | W | 5 | split | `FX.UPMIX_THRESHOLD` (RP2350 only) | `0x0895` | S | `0` | unchanged | 4 |
| 0x4C | [`UPMIX_SET_PARAM`](vendor_protocol_migration_guide.md#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | W | 6 | split | `FX.UPMIX_ATTACK` (RP2350 only) | `0x0896` | S | `0` | unchanged | 4 |
| 0x4C | [`UPMIX_SET_PARAM`](vendor_protocol_migration_guide.md#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | W | 7 | split | `FX.UPMIX_RELEASE` (RP2350 only) | `0x0897` | S | `0` | unchanged | 4 |
| 0x4C | [`UPMIX_SET_PARAM`](vendor_protocol_migration_guide.md#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | W | 8 | split | `FX.UPMIX_DET_HPF` (RP2350 only) | `0x0898` | S | `0` | unchanged | 4 |
| 0x4C | [`UPMIX_SET_PARAM`](vendor_protocol_migration_guide.md#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | W | 9 | split | `FX.UPMIX_SUR_DELAY` (RP2350 only) | `0x0899` | S | `0` | unchanged | 4 |
| 0x4C | [`UPMIX_SET_PARAM`](vendor_protocol_migration_guide.md#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | W | 10 | split | `FX.UPMIX_SUR_HPF` (RP2350 only) | `0x089A` | S | `0` | unchanged | 4 |
| 0x4C | [`UPMIX_SET_PARAM`](vendor_protocol_migration_guide.md#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | W | 11 | split | `FX.UPMIX_SUR_LPF` (RP2350 only) | `0x089B` | S | `0` | unchanged | 4 |
| 0x4C | [`UPMIX_SET_PARAM`](vendor_protocol_migration_guide.md#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | W | 12 | split | `FX.UPMIX_DECORR` (RP2350 only) | `0x089C` | S | `0` | unchanged | 4 |
| 0x4C | [`UPMIX_SET_PARAM`](vendor_protocol_migration_guide.md#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | W | 13 | split | `FX.UPMIX_PRESENCE` (RP2350 only) | `0x089D` | S | `0` | unchanged | 4 |
| 0x4C | [`UPMIX_SET_PARAM`](vendor_protocol_migration_guide.md#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | W | any other | split | none | - | - | - | 1.1.6 ignored it | - |
| 0x4D | [`UPMIX_GET_PARAM`](vendor_protocol_migration_guide.md#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | R | 0 | split | `FX.UPMIX_ENABLE` (RP2350 only) | `0x0890` | G | `0` | - | 4 |
| 0x4D | [`UPMIX_GET_PARAM`](vendor_protocol_migration_guide.md#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | R | 1 | split | `FX.UPMIX_CENTER_MODE` (RP2350 only) | `0x0891` | G | `0` | - | 4 |
| 0x4D | [`UPMIX_GET_PARAM`](vendor_protocol_migration_guide.md#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | R | 2 | split | `FX.UPMIX_SURROUND_MODE` (RP2350 only) | `0x0892` | G | `0` | - | 4 |
| 0x4D | [`UPMIX_GET_PARAM`](vendor_protocol_migration_guide.md#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | R | 3 | split | `FX.UPMIX_STRENGTH` (RP2350 only) | `0x0893` | G | `0` | - | 4 |
| 0x4D | [`UPMIX_GET_PARAM`](vendor_protocol_migration_guide.md#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | R | 4 | split | `FX.UPMIX_CENTER_WIDTH` (RP2350 only) | `0x0894` | G | `0` | - | 4 |
| 0x4D | [`UPMIX_GET_PARAM`](vendor_protocol_migration_guide.md#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | R | 5 | split | `FX.UPMIX_THRESHOLD` (RP2350 only) | `0x0895` | G | `0` | - | 4 |
| 0x4D | [`UPMIX_GET_PARAM`](vendor_protocol_migration_guide.md#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | R | 6 | split | `FX.UPMIX_ATTACK` (RP2350 only) | `0x0896` | G | `0` | - | 4 |
| 0x4D | [`UPMIX_GET_PARAM`](vendor_protocol_migration_guide.md#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | R | 7 | split | `FX.UPMIX_RELEASE` (RP2350 only) | `0x0897` | G | `0` | - | 4 |
| 0x4D | [`UPMIX_GET_PARAM`](vendor_protocol_migration_guide.md#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | R | 8 | split | `FX.UPMIX_DET_HPF` (RP2350 only) | `0x0898` | G | `0` | - | 4 |
| 0x4D | [`UPMIX_GET_PARAM`](vendor_protocol_migration_guide.md#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | R | 9 | split | `FX.UPMIX_SUR_DELAY` (RP2350 only) | `0x0899` | G | `0` | - | 4 |
| 0x4D | [`UPMIX_GET_PARAM`](vendor_protocol_migration_guide.md#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | R | 10 | split | `FX.UPMIX_SUR_HPF` (RP2350 only) | `0x089A` | G | `0` | - | 4 |
| 0x4D | [`UPMIX_GET_PARAM`](vendor_protocol_migration_guide.md#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | R | 11 | split | `FX.UPMIX_SUR_LPF` (RP2350 only) | `0x089B` | G | `0` | - | 4 |
| 0x4D | [`UPMIX_GET_PARAM`](vendor_protocol_migration_guide.md#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | R | 12 | split | `FX.UPMIX_DECORR` (RP2350 only) | `0x089C` | G | `0` | - | 4 |
| 0x4D | [`UPMIX_GET_PARAM`](vendor_protocol_migration_guide.md#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | R | 13 | split | `FX.UPMIX_PRESENCE` (RP2350 only) | `0x089D` | G | `0` | - | 4 |
| 0x4D | [`UPMIX_GET_PARAM`](vendor_protocol_migration_guide.md#upmix_set_param-and-upmix_get_param-0x4c-0x4d) | R | any other | split | none | - | - | - | 1.1.6 STALLed it | - |
| 0x4E | `UPMIX_GET_STATUS` | R | any | same | `FX.UPMIX_STATUS` (RP2350 only) | `0x0881` | G | `0` | - | 32 |
| 0x50 | [`GET_STATUS`](#get_status-0x50) | R | any | custom | `MEAS.LEVELS` G, `SYS.SYSTEM_STATUS` G, `SYS.PDM_STATS` G, `SYS.SPDIF_TX_STATS` G, `SYS.USB_AUDIO_STATUS` G, `SYS.PIPELINE_STATS` G, `INPUT.STATUS` G, `DIAG.LOOPBACK_STATS` (DSPI_LOOPBACK) G | `0x0910`, `0x0140`, `0x0144`, `0x0145`, `0x0141`, `0x0143`, `0x0311`, `0xF020` | - | see recipe | [see recipe](#get_status-0x50) | see recipe |
| 0x51 | `SAVE_PARAMS` | W-as-R | any | retired | none | - | - | - | [replacement](#retired-opcodes) | - |
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
| 0x81 | [`LIMITER`](vendor_protocol_migration_guide.md#limiter-0x81) | W | old & 0xFF = 0 | split | `DYN.LIMITER_ENABLE` | `0x0710` | S | `old >> 8` | unchanged | 4 |
| 0x81 | [`LIMITER`](vendor_protocol_migration_guide.md#limiter-0x81) | W | old & 0xFF = 1 | split | `DYN.LIMITER_THRESHOLD` | `0x0711` | S | `old >> 8` | unchanged | 4 |
| 0x81 | [`LIMITER`](vendor_protocol_migration_guide.md#limiter-0x81) | W | old & 0xFF = 2 | split | `DYN.LIMITER_RELEASE` | `0x0712` | S | `old >> 8` | unchanged | 4 |
| 0x81 | [`LIMITER`](vendor_protocol_migration_guide.md#limiter-0x81) | W | old & 0xFF = 3 | split | `DYN.LIMITER_LINK_GROUP` | `0x0713` | S | `old >> 8` | unchanged | 4 |
| 0x81 | [`LIMITER`](vendor_protocol_migration_guide.md#limiter-0x81) | W | any other | split | none | - | - | - | 1.1.6 ignored it | - |
| 0x81 | [`LIMITER`](vendor_protocol_migration_guide.md#limiter-0x81) | R | old & 0xFF = 0 | split | `DYN.LIMITER_ENABLE` | `0x0710` | G | `old >> 8` | - | 4 |
| 0x81 | [`LIMITER`](vendor_protocol_migration_guide.md#limiter-0x81) | R | old & 0xFF = 1 | split | `DYN.LIMITER_THRESHOLD` | `0x0711` | G | `old >> 8` | - | 4 |
| 0x81 | [`LIMITER`](vendor_protocol_migration_guide.md#limiter-0x81) | R | old & 0xFF = 2 | split | `DYN.LIMITER_RELEASE` | `0x0712` | G | `old >> 8` | - | 4 |
| 0x81 | [`LIMITER`](vendor_protocol_migration_guide.md#limiter-0x81) | R | old & 0xFF = 3 | split | `DYN.LIMITER_LINK_GROUP` | `0x0713` | G | `old >> 8` | - | 4 |
| 0x81 | [`LIMITER`](vendor_protocol_migration_guide.md#limiter-0x81) | R | old & 0xFF = 128 | split | `DYN.LIMITER_METERS` | `0x0718` | G | `0` | - | var |
| 0x81 | [`LIMITER`](vendor_protocol_migration_guide.md#limiter-0x81) | R | old & 0xFF = 129 | split | `DYN.LIMITER_STATUS` | `0x0719` | G | `0` | - | 4 |
| 0x81 | [`LIMITER`](vendor_protocol_migration_guide.md#limiter-0x81) | R | any other | split | none | - | - | - | 1.1.6 STALLed it | - |
| 0x82 | `GET_CLOCK_DIAG` | R | any | same | `DIAG.CLOCK_DIAG` (DSPI_CLOCK_DIAG) | `0xF010` | G | `0` | - | var |
| 0x83 | [`CLEAR_CLIPS`](#clear_clips-0x83) | W-as-R | any | custom | `MEAS.CLIP_FLAGS` G, `MEAS.CLIP_CLEAR` S | `0x0911`, `0x0912` | - | see recipe | [see recipe](#clear_clips-0x83) | see recipe |
| 0x84 | `SET_CS_BINDING` | W | any | folded | `CS.BINDING` | `0x0A20` | S | `old` | unchanged | 24 |
| 0x85 | `GET_CS_BINDING` | R | any | folded | `CS.BINDING` | `0x0A20` | G | `old` | - | 24 |
| 0x86 | [`GET_CS_CAPS`](vendor_protocol_migration_guide.md#get_cs_caps-0x86) | R | 0xFFFF | split | `CS.CAPS` | `0x0A10` | G | `0` | - | 52 |
| 0x86 | [`GET_CS_CAPS`](vendor_protocol_migration_guide.md#get_cs_caps-0x86) | R | any other | split | `CS.NOUN_DESC` | `0x0A11` | G | `old` | - | 12 |
| 0x87 | `GET_CS_STATUS` | R | any | same | `CS.STATUS` | `0x0A12` | G | `0` | - | 41 |
| 0x88 | `SET_I2S_CLOCK_MODE` | W | any | folded | `CLOCK.I2S_CLOCK_MODE` | `0x0C12` | S | `0` | unchanged | 1 |
| 0x89 | `GET_I2S_CLOCK_MODE` | R | any | folded | `CLOCK.I2S_CLOCK_MODE` | `0x0C12` | G | `0` | - | 1 |
| 0x8A | `GET_I2S_SLAVE_STATUS` | R | any | same | `CLOCK.I2S_SLAVE_STATUS` | `0x0C13` | G | `0` | - | 16 |
| 0x8B | `SET_CS_NAME` | W | any | folded | `CS.SLOT_NAME` | `0x0A21` | S | `old` | first 32 bytes | 1-32 |
| 0x8C | `GET_CS_NAME` | R | any | folded | `CS.SLOT_NAME` | `0x0A21` | G | `old & 0xFF` | - | 32 |
| 0x8D | `SET_CS_IR_CMD` | W | any | folded | `CS.IR_CMD` | `0x0A30` | S | `old` | unchanged | 16 |
| 0x8E | `GET_CS_IR_CMD` | R | any | folded | `CS.IR_CMD` | `0x0A30` | G | `old` | - | 16 |
| 0x8F | [`CS_IR_LEARN`](vendor_protocol_migration_guide.md#cs_ir_learn-0x8f) | W-as-R | 0 | split | `CS.IR_LEARN_CANCEL` | `0x0A33` | S0 | `0` | none | 0 |
| 0x8F | [`CS_IR_LEARN`](vendor_protocol_migration_guide.md#cs_ir_learn-0x8f) | W-as-R | 1 | split | `CS.IR_LEARN_ARM` | `0x0A32` | S0 | `0` | none | 0 |
| 0x8F | [`CS_IR_LEARN`](vendor_protocol_migration_guide.md#cs_ir_learn-0x8f) | W-as-R | 2 | split | `CS.IR_LEARN_STATUS` | `0x0A31` | G | `0` | - | 8 |
| 0x8F | [`CS_IR_LEARN`](vendor_protocol_migration_guide.md#cs_ir_learn-0x8f) | W-as-R | any other | split | none | - | - | - | 1.1.6 STALLed it | - |
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
| 0xA0 | `GET_ALL_PARAMS` | R | any | same | `STORE.ALL_PARAMS` | `0x0240` | G | `0` | - | 6252 |
| 0xA1 | [`SET_ALL_PARAMS`](#set_all_params-0xa1) | W | any | custom | `STORE.ALL_PARAMS` S | `0x0240` | - | see recipe | [see recipe](#set_all_params-0xa1) | see recipe |
| 0xA2 | `GET_ALL_PARAMS_CHUNK` | R | any | folded | `STORE.ALL_PARAMS_CHUNK` | `0x0241` | G | `old` | - | var |
| 0xA3 | `SET_ALL_PARAMS_CHUNK` | W | any | folded | `STORE.ALL_PARAMS_CHUNK` | `0x0241` | S | `old` | unchanged | chunk |
| 0xA4 | `SIGGEN_SET_CONFIG` | W | any | folded | `MEAS.SIGGEN_CONFIG` | `0x0940` | S | `0` | unchanged | 36 |
| 0xA5 | `SIGGEN_GET_CONFIG` | R | any | folded | `MEAS.SIGGEN_CONFIG` | `0x0940` | G | `0` | - | 36 |
| 0xA6 | [`SIGGEN_CONTROL`](vendor_protocol_migration_guide.md#rta_control-0x0e-and-siggen_control-0xa6) | W-as-R | old & 0xFF = 0 | split | `MEAS.SIGGEN_STOP` | `0x0949` | S0 | `0` | none | 0 |
| 0xA6 | [`SIGGEN_CONTROL`](vendor_protocol_migration_guide.md#rta_control-0x0e-and-siggen_control-0xa6) | W-as-R | old & 0xFF = 1 | split | `MEAS.SIGGEN_START` | `0x0948` | S0 | `0` | none | 0 |
| 0xA6 | [`SIGGEN_CONTROL`](vendor_protocol_migration_guide.md#rta_control-0x0e-and-siggen_control-0xa6) | W-as-R | old & 0xFF = 2 | split | `MEAS.SIGGEN_STOP_NOW` | `0x094A` | S0 | `0` | none | 0 |
| 0xA6 | [`SIGGEN_CONTROL`](vendor_protocol_migration_guide.md#rta_control-0x0e-and-siggen_control-0xa6) | W-as-R | any other | split | none | - | - | - | 1.1.6 STALLed it | - |
| 0xA7 | `SIGGEN_GET_STATUS` | R | any | same | `MEAS.SIGGEN_STATUS` | `0x0941` | G | `0` | - | 16 |
| 0xA8 | [`SIGGEN_GET_CAPS`](vendor_protocol_migration_guide.md#siggen_get_caps-0xa8) | R | 0xFFFF | split | `MEAS.SIGGEN_CAPS` | `0x0942` | G | `0` | - | 8 |
| 0xA8 | [`SIGGEN_GET_CAPS`](vendor_protocol_migration_guide.md#siggen_get_caps-0xa8) | R | any other | split | `MEAS.SIGGEN_TYPE_DESC` | `0x0943` | G | `old` | - | 62 |
| 0xA9 | `SET_SUBHARM_DEPTH` | W | any | folded | `FX.SUBHARM_DEPTH` | `0x0827` | S | `0` | unchanged | 4 |
| 0xAA | `GET_SUBHARM_DEPTH` | R | any | folded | `FX.SUBHARM_DEPTH` | `0x0827` | G | `0` | - | 4 |
| 0xAB | `SET_SUBHARM_HOLD` | W | any | folded | `FX.SUBHARM_HOLD` | `0x0828` | S | `0` | unchanged | 4 |
| 0xAC | `GET_SUBHARM_HOLD` | R | any | folded | `FX.SUBHARM_HOLD` | `0x0828` | G | `0` | - | 4 |
| 0xAD | `SET_SUBHARM_CEILING` | W | any | folded | `FX.SUBHARM_CEILING` | `0x0829` | S | `0` | unchanged | 4 |
| 0xAE | `GET_SUBHARM_CEILING` | R | any | folded | `FX.SUBHARM_CEILING` | `0x0829` | G | `0` | - | 4 |
| 0xB0 | `GET_BUFFER_STATS` | R | any | same | `SYS.BUFFER_STATS` | `0x0142` | G | `0` | - | 44 |
| 0xB1 | `RESET_BUFFER_STATS` | W-as-R | any | action | `SYS.STATS_RESET` | `0x014F` | S | `0` | u8 `old & 0xFF` | 1 |
| 0xB2 | `GET_USB_ERROR_STATS` | R | any | retired | none | - | - | - | [replacement](#retired-opcodes) | - |
| 0xB3 | `RESET_USB_ERROR_STATS` | W-as-R | any | retired | none | - | - | - | [replacement](#retired-opcodes) | - |
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
| 0xE9 | `SET_SPDIF_INPUT_ENABLE` | W-as-R | any | retired | none | - | - | - | [replacement](#retired-opcodes) | - |
| 0xEA | `SET_DAC_HW_MUTE_CONFIG` | W | any | folded | `OUTPUT.DAC_MUTE_CONFIG` | `0x0440` | S | `0` | unchanged | 16 |
| 0xEB | `GET_DAC_HW_MUTE_CONFIG` | R | any | folded | `OUTPUT.DAC_MUTE_CONFIG` | `0x0440` | G | `0` | - | 16 |
| 0xEC | `TEST_DAC_HW_MUTE` | W-as-R | any | action | `OUTPUT.DAC_MUTE_TEST` | `0x0441` | S0 | `0` | none | 0 |
| 0xED | `SET_INPUT_RATE` | W | any | same | `INPUT.I2S_RX_RATE` | `0x0332` | S | `0` | unchanged | 4 |
| 0xEE | [`GET_INPUT_RATE`](#get_input_rate-0xee) | R | any | custom | `SYS.SYSTEM_STATUS` G, `INPUT.I2S_RX_RATE` G | `0x0140`, `0x0332` | - | see recipe | [see recipe](#get_input_rate-0xee) | see recipe |
| 0xEF | `GET_SPDIF_INPUT_CONFIG` | R | any | same | `INPUT.SPDIF_RX_CONFIG` | `0x0324` | G | `0` | - | 7 |
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

---

## 6. Commands that need more than a renumber

### `GET_STATUS` (0x50)

1.1.6 read one 4-byte value per request with a selector in `wValue`. 1.1.7 groups related values into a few struct reads, so one read replaces several old requests. Each row says where the old value now lives.

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

### `SET_EQ_PARAM` and `GET_EQ_PARAM` (0x42, 0x43)

`FILTER.BAND` (`0x0610`) reads and writes a whole band with one 16-byte layout in both directions. The band is addressed by `wValue = (channel << 8) | band`. Bands 0 to 9 are EQ bands and 20 to 23 are crossover bands, which exist on output channels only.

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Filter type |
| 1 | u8 | Bypass: exactly 1 bypasses |
| 2 | u16 | Linkwitz Transform Qp x 512, 0 selects 0.707 |
| 4 | f32 | Frequency, Hz |
| 8 | f32 | Q |
| 12 | f32 | Gain, dB |

To convert an old 16- or 18-byte packet `p`, send `wValue = (p[0] << 8) | p[1]` and the payload `p[2], p[3], qp low, qp high, p[4..15]`. The old one-parameter reads are fields of the new reply: type at byte 0, frequency at 4, Q at 8, gain at 12, bypass at 1 and Qp at 2.

There is no single-parameter write, so a change to one field is a read, modify and write of all 16 bytes. Always send the Qp, because 1.1.7 stores what you send. Write from a host-side copy of every band (kept from `STORE.ALL_PARAMS` and notifications) rather than from a fresh read, because a write is applied a millisecond or two later and a quick second read can still show the old band. `FILTER.BAND_BYPASS` (`0x0611`, 1 byte) still sets the bypass alone.

### `SET_MATRIX_ROUTE` and `GET_MATRIX_ROUTE` (0x70, 0x71)

`ROUTE.MATRIX_ROUTE` (`0x0510`) moves the crosspoint address into `wValue = (input << 8) | output`, and the reply no longer echoes it. The payload and reply are 8 bytes.

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Enabled, non-zero routes |
| 1 | u8 | Phase invert, non-zero inverts |
| 2 | u16 | Reserved, send 0 |
| 4 | f32 | Gain, dB |

To convert an old SET packet `p`, send `wValue = (p[0] << 8) | p[1]` and the payload `p[2], p[3], 00, 00, p[4..7]`.

### `CLEAR_CLIPS` (0x83)

The old read-and-clear is now a read and an action. Read `MEAS.CLIP_FLAGS` (`0x0911`, 4 bytes, bit k = channel k), then send `MEAS.CLIP_CLEAR` (`0x0912`) as a zero-length SET. The flags are also in `MEAS.LEVELS`, so a meter loop that reads it needs only the clear.

### `GET_INPUT_RATE` (0xEE)

The old 8-byte reply is split. The device rate is bytes 4 to 7 of `SYS.SYSTEM_STATUS` (`0x0140`). The selected I2S rate is `INPUT.I2S_RX_RATE` (`0x0332`, 4 bytes), which is also the SET that replaces `SET_INPUT_RATE`.

### `PRESET_GET_STARTUP` (0x97)

`STORE.STARTUP` (`0x0216`) answers 2 bytes, `{mode, default slot}`, which is also what its SET takes. The last active slot is byte 4 of `STORE.DIRECTORY` (`0x0215`).

### `GET_CS_DISPLAY_CFG` (0x28)

The old 16-byte reply is split. Bytes 0 to 3 are `CS.DISPLAY_CAPS` (`0x0A71`, 4 bytes). Bytes 4 to 15 are `CS.DISPLAY_CONFIG` (`0x0A70`, 12 bytes), which is also exactly what its SET takes.

### `SET_ALL_PARAMS` (0xA1)

The blob grew to version 35, 6252 bytes (section 1). Send exactly 6252 bytes to `STORE.ALL_PARAMS` (`0x0240`) with `wValue` 0. Any other length, or a header with any other version, is refused. On Windows, move the blob with `STORE.ALL_PARAMS_CHUNK` (`0x0241`, `wValue` = byte offset) in pieces of at most 4096 bytes.

### Retired opcodes

These 1.1.6 requests have no command in 1.1.7.

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
| 0xE9 | `SET_SPDIF_INPUT_ENABLE` | W-as-R | None per input. Set `INPUT.SPDIF_RX_COUNT` (`0x0326`) to cover the input and `INPUT.SPDIF_RX_ENABLE` (`0x0327`) to 1 |

<!-- VC:END RETIRED -->

---

## 7. Replies that changed

Every other GET reply has the same length, layout and values as in 1.1.6, so its parser can stay as it is.

- `FILTER.BAND` is 16 bytes and replaces the 4-byte `GET_EQ_PARAM` reads (section 6).
- `ROUTE.MATRIX_ROUTE` is the same length with a new layout (section 6).
- `STORE.STARTUP` is 2 bytes where `PRESET_GET_STARTUP` was 3.
- `INPUT.I2S_RX_RATE` is 4 bytes where `GET_INPUT_RATE` was 8.
- `CS.DISPLAY_CONFIG` is 12 bytes where `GET_CS_DISPLAY_CFG` was 16.
- `GET_STATUS` is replaced by the struct reads in section 6.
- `MEAS.RTA_BINS` returns the same frame, but at most 256 bytes per request. Read at offsets 0, 256 and 512, and compare byte 2 with the frame's last byte to check the frame did not change between reads.

Refusals also differ. A request that 1.1.6 answered with zeros, such as a bad index on a pin read or an ADAT command on RP2040, now STALLs. A non-zero `wValue` on a command that addresses nothing now STALLs.

---

## 8. Checklist

1. [ ] Probe with `SYS.PROTOCOL_INFO` before any other request, and handle old firmware (section 2).
2. [ ] Build every request from a page and a command number, with the page in the `wIndex` high byte (section 1).
3. [ ] Replace every opcode with its new command from section 5. Widen every opcode variable to 16 bits.
4. [ ] Send `wValue` 0 to every command that addresses nothing, and move values from `wValue` into the payload.
5. [ ] Turn the write-as-reads into SETs and drop the code that parsed their status bytes.
6. [ ] Port the reshaped and split commands of section 6, and remove the retired ones.
7. [ ] Send exact payload lengths.
8. [ ] Treat a STALL, or a non-zero UART or I2C status, as a refusal. Retry only the busy codes.
9. [ ] Replace version-based feature gates with `SYS.PAGE_COMMANDS`.
10. [ ] Re-check every reply length you assert against section 5, and the changed replies in section 7.
11. [ ] Test against a real 1.1.7 device on each platform you support, and confirm a 1.1.6 device gets nothing but the probe.

The command registry `Documentation/protocol/dspi_commands.json` and the map `Documentation/protocol/legacy_map.json` hold all of this in machine-readable form, for clients that generate their constants.
