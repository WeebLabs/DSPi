# External Control Interfaces (UART and I2C Target)

*Last updated: 2026-11-16 (paged vendor protocol, firmware 1.1.7)*

*Transport framing version: 1. Command protocol: paged, major version 1.*

This document specifies how to drive a DSPi from an external microcontroller over UART or over the I2C target (slave) interface. It is written for a firmware integrator building an ESP32, STM32, Arduino or SBC controller. No DSPi source is needed to implement a client.

## TL;DR

A UART or I2C controller sends exactly the same commands a USB host sends. Each request names a command with two numbers. The page says which subsystem (filters, presets, inputs and so on) and travels in the high byte of `wIndex`. The command number says which thing inside that page and travels in `bRequest`. The UART and I2C frames carry these fields unchanged, so a client builds the same request it would send over USB and wraps it in a frame.

Every response starts with one status byte. Zero means success. A value from `0x01` to `0x07` is a transport problem, such as a corrupt frame, an oversized payload or a busy device, and the command did not run. A value of `0x10` or above is the command's own answer, for example "no such command" or "value out of range". Treat any non-zero status as failure, and retry only `0x01`, `0x04` and `0x17`.

Before sending anything else to a device whose firmware version is unknown, send one `SYS.PROTOCOL_INFO` read and check that the reply starts with `DSPi`. Firmware 1.1.6 and older ignores `wIndex` and would run an unrelated old command with the same number, so this probe is mandatory.

Firmware 1.1.7 answers nothing on page 0. A client that leaves the `wIndex` high byte at zero gets status `0x10` for every request, and nothing on the device changes.

---

## 1. Overview

### 1.1 One command surface, three transports

DSPi exposes a single vendor command surface on three transports.

- **USB.** EP0 vendor control transfers.
- **UART.** An asynchronous serial link at 3.3 V logic.
- **I2C target.** The device is an I2C slave and the external controller is the bus master.

Each transport turns its wire frame into the same request fields and hands them to one dispatcher. The dispatcher checks the page, the command, the direction, the payload length and the address before any handler runs, so every transport gets the same answer for the same request.

| Field | Meaning |
|---|---|
| Type | SET (host to device) or GET (device to host) |
| `bRequest` | Command number within the page, `0x10` to `0xFF` |
| `wIndex` high byte | Page, `0x01` to `0xFF` |
| `wIndex` low byte | Ignored on UART and I2C. Send `0x00` (USB hosts send the interface number, 2) |
| `wValue` | Address. It says which channel, output, slot, band or byte offset. Zero when the command addresses nothing |
| `wLength` | SET: payload length. GET: largest reply wanted, 0 for the whole reply |
| Payload | Values, little-endian |

In this document a command is written `PAGE.NAME`, for example `ROUTE.MASTER_VOLUME`, which is page `0x05`, command `0x50`. The full command set with every payload layout is in `Documentation/commands.md` and, machine-readable, in `Documentation/protocol/dspi_commands.json`.

Three protocol rules matter on every transport.

1. A setting that can be read and written is one command. A GET reads it and a SET writes it.
2. A GET never changes anything. Actions with no value, such as clearing clip flags, are zero-length SETs.
3. A rejected request fails explicitly with a result code. Nothing is silently ignored.

### 1.2 Discovery and old firmware

`SYS.PROTOCOL_INFO` is page `0x01`, command `0x10`. Its 16-byte reply is the same on every transport.

| Offset | Type | Field |
|---|---|---|
| 0 | u8[4] | Magic `"DSPi"` |
| 4 | u8 | Protocol major (1) |
| 5 | u8 | Protocol minor |
| 6 | u8 | Number of implemented pages |
| 7 | u8 | Flags, reserved, 0 |
| 8 | u16 | Largest non-bulk SET payload (64) |
| 10 | u16 | Largest non-bulk GET reply (256) |
| 12 | u32 | Reserved, 0 |

A client starts every session the same way.

1. Send a GET for `SYS.PROTOCOL_INFO`.
2. If the status is `0x00` and the reply starts with `DSPi`, the paged protocol is available. Use `SYS.PAGE_LIST`, `SYS.PAGE_COMMANDS` and `SYS.COMMAND_INFO` to find out which commands this build has.
3. Anything else means firmware 1.1.6 or older. On UART and I2C that firmware answers the probe with status `0x02`. Send no other paged command to it. It still answers its own `GET_PLATFORM` (`bRequest` `0x7F`, `wIndex` `0x0000`, 7-byte reply) if the client wants to read the version and offer an update.

The probe itself is safe on old firmware because it has no GET handler for `bRequest` `0x10`.

**A 1.1.6-era client on 1.1.7 firmware.** Such a client sends `wIndex` `0x0000`, so every request lands on page 0. Firmware 1.1.7 refuses all of them with status `0x10` and changes nothing.

### 1.3 Commands refused on UART and I2C

A few commands are reachable over USB only. Over UART or I2C they answer `VC_ERR_BLOCKED` (`0x16`) and never run.

| Command | Page, cmd | Refused direction | Why |
|---|---|---|---|
| `XPORT.UART_CONFIG` | `0x0B`, `0x10` | SET | An external controller must not reconfigure or disable its own link. The GET works |
| `XPORT.I2C_CONFIG` | `0x0B`, `0x11` | SET | Same. The GET works |
| `STORE.ALL_PARAMS_CHUNK` | `0x02`, `0x41` | GET and SET | Exists only for the 4 KB WinUSB control limit. Use `STORE.ALL_PARAMS` |
| `MEAS.RTA_BANDS_ALL` | `0x09`, `0x26` | GET | Read `MEAS.RTA_BANDS` per channel instead |
| `DIAG.CLOCK_DIAG` | `0xF0`, `0x10` | GET | Bench builds only |

The device itself is the authority. `SYS.COMMAND_INFO` reports a command's flags in bytes 0 and 1, and bit 0 (`0x0001`, USB only) or bit 1 (`0x0002`, SET USB only) marks a command this table would list.

### 1.4 Ship state

Both interfaces ship **disabled**. A fresh device, or one after a factory reset, has no UART or I2C control activity and holds none of their pins. A one-time USB setup enables and pins each interface, and the choice persists across reboots and factory resets.

---

## 2. Configuration over USB

| Command | Page, cmd | Direction | Payload or reply |
|---|---|---|---|
| `XPORT.UART_CONFIG` | `0x0B`, `0x10` | GET, SET (SET over USB only) | 8-byte `UartCtrlConfig` |
| `XPORT.I2C_CONFIG` | `0x0B`, `0x11` | GET, SET (SET over USB only) | 8-byte `I2cCtrlConfig` |
| `XPORT.STATUS` | `0x0B`, `0x12` | GET | 8-byte `CtrlIfaceStatus` |

### 2.1 Structures

All fields little-endian, packed, no padding.

```c
// UART control interface configuration (8 bytes). Framing is fixed 8N1.
typedef struct __attribute__((packed)) {
    uint8_t  enabled;       // 0 = off, 1 = on
    uint8_t  tx_pin;        // GPIO with a UARTx TX mux; pin % 4 == 0
    uint8_t  rx_pin;        // GPIO with a UARTx RX mux; pin % 4 == 1, same instance
    uint8_t  notify_enable; // 0 = off (default), 1 = push type 0x40 notification frames (3.6)
    uint32_t baud;          // 9600 .. 1000000
} UartCtrlConfig;

// I2C target control interface configuration (8 bytes).
typedef struct __attribute__((packed)) {
    uint8_t  enabled;   // 0 = off, 1 = on
    uint8_t  sda_pin;   // GPIO with an I2Cx SDA mux; even
    uint8_t  scl_pin;   // GPIO with an I2Cx SCL mux; odd, same instance
    uint8_t  address;   // 7-bit target address, 0x08 .. 0x77
    uint8_t  reserved[4];
} I2cCtrlConfig;

// XPORT.STATUS reply (8 bytes).
typedef struct __attribute__((packed)) {
    uint8_t uart_last_status; // PIN_CONFIG_* from the last UART config apply
    uint8_t uart_live;        // 1 if the UART is up and listening
    uint8_t i2c_last_status;  // PIN_CONFIG_* from the last I2C config apply
    uint8_t i2c_live;         // 1 if the I2C target is up and listening
    uint8_t proto_version;    // transport framing version, 1
    uint8_t reserved[3];      // 0
} CtrlIfaceStatus;
```

The GPIO mux constrains both interfaces. A UART TX pin must be a GPIO whose function-2 mux is that UART instance's TX (`pin % 4 == 0`), and its RX pin must be the same instance's RX (`pin % 4 == 1`). An I2C SDA pin must be even and its SCL pin the next odd GPIO on the same instance. Board builds with their own pin map apply their own rule.

### 2.2 Defaults

These apply on a fresh flash and whenever a stored config fails a bounds check on load.

| Field | UART default | I2C default |
|---|---|---|
| enabled | 0 (off) | 0 (off) |
| TX or SDA pin | GPIO 16 | GPIO 18 |
| RX or SCL pin | GPIO 17 | GPIO 19 |
| baud | 115200 | n/a |
| address | n/a | 0x42 |

### 2.3 Apply, persist and result

A config SET is checked at once. A field out of range (baud, address, an `enabled` or `notify_enable` value above 1) fails with `VC_ERR_VALUE`, and a GPIO number the board does not have fails with `VC_ERR_PIN_INVALID`. Over USB a failed SET STALLs and the code is readable from `SYS.LAST_RESULT`.

A SET that passes is queued, and the main loop applies it later, because it touches GPIO and IRQ state and writes flash. The main loop works in this order.

1. **Live apply.** It tears the current interface down first, so its own pins do not look taken. Then it validates the new config against every other pin user and, if valid and enabled, brings the interface up. On failure the previous config is restored where possible.
2. **Persist on success only.** The config is written to flash only when the apply succeeded. A rejected config never replaces a good stored one.

The outcome lands in `XPORT.STATUS` as a `PIN_CONFIG_*` code.

| Code | Name | Meaning |
|---|---|---|
| `0x00` | `PIN_CONFIG_SUCCESS` | Applied and persisted |
| `0x01` | `PIN_CONFIG_INVALID_PIN` | A pin lacks the required mux function |
| `0x02` | `PIN_CONFIG_PIN_IN_USE` | A pin is claimed by an output, an input, MCK, DAC mute, the display or the other control interface |
| `0x05` | `PIN_CONFIG_INVALID_PARAM` | A non-pin field is out of range |

The flash write causes the usual interrupt blackout (see 5.2). Interface config is device-level. It is not part of any preset and not part of `STORE.ALL_PARAMS`.

### 2.4 Changes reported to the USB host

When a command arriving over UART or I2C changes a parameter, the device sends its normal change notification to the USB host, tagged with the transport.

| Source tag | Value |
|---|---|
| `PARAM_SRC_UART` | 8 |
| `PARAM_SRC_I2C` | 9 |

### 2.5 Boot behavior

At boot the firmware reads the stored configs and brings each enabled interface up. This happens after the outputs, inputs, MCK and DAC mute have claimed their pins. A stored config whose pins now collide stays **down**. Its stored `enabled` is still 1, but `XPORT.STATUS` shows `*_live == 0` and the last status `PIN_CONFIG_PIN_IN_USE`. Compare the two to detect it.

---

## 3. UART Transport

### 3.1 Electrical and link parameters

- **Logic level.** 3.3 V. Level-shift a 5 V host.
- **Framing.** Fixed 8N1. The frame CRC provides integrity, so there is no parity option.
- **Baud.** 9600 to 1000000, default 115200.
- **Direction.** Full duplex, request and response. With notifications enabled the device also sends its own frames between responses (3.6).

### 3.2 Frame formats

Every frame starts with the sync byte `0xA5` and a type byte. Multi-byte fields are little-endian.

```
Request SET   (host -> device):
  A5 01 bReq wValL wValH wIdxL wIdxH wLenL wLenH  payload[wLen]  crcL crcH

Request GET   (host -> device):
  A5 02 bReq wValL wValH wIdxL wIdxH wLenL wLenH                 crcL crcH
      (no payload; wLen caps the reply, 0 = whole reply)

Response SET  (device -> host):
  A5 81 status 00 00                                              crcL crcH

Response GET  (device -> host):
  A5 82 status lenL lenH payload[len]                             crcL crcH
      (len is 0 and there is no payload unless status is 0x00)

Notification  (device -> host):
  A5 40 00 lenL lenH packet[len]                                  crcL crcH
```

| Type | Meaning |
|---|---|
| `0x01` | SET request |
| `0x02` | GET request |
| `0x40` | Notification (opt-in, see 3.6) |
| `0x81` | SET response |
| `0x82` | GET response |

`wIdxH` is the page and `wIdxL` is ignored. Bytes that arrive while the parser is not synced, and frames with an unknown type byte, are dropped. The parser resynchronizes on the next `0xA5`.

### 3.3 CRC16

- **Algorithm.** CRC16-CCITT-FALSE. Polynomial `0x1021`, initial value `0xFFFF`, no reflection, no final XOR.
- **Coverage.** Every byte after the sync byte, from the type byte to the last payload byte. The sync byte is not included.
- **On the wire.** Low byte first.

Check value. The CRC of the ASCII string `123456789` is `0x29B1`.

### 3.4 Timing and the one-request rule

- **One request in flight.** Send a request, wait for its response, then send the next. The device does not queue pipelined requests.
- **Dispatch runs from the main loop.** The UART interrupt only moves bytes into a ring. Response latency is about one main-loop pass, longer during a flash write.
- **Partial frames.** A frame that stalls for more than 100 ms between bytes is dropped without a response.
- **Retries.** When the dispatcher reports a transient condition (`0x01`, `0x04` or `0x17`), the device retries internally for 50 ms before it answers with that status. A client should still be ready to retry.
- **Timeouts.** Use a per-request timeout of a few hundred milliseconds and retry on timeout. Bytes sent during a flash write can be lost (5.2).

### 3.5 Bootloader entry

`SYS.ENTER_BOOTLOADER` (page `0x01`, command `0x30`) is a zero-length SET. The device sends its OK response, waits until the last byte has left the UART, and then reboots into the USB bootloader. The UART link is gone after that.

### 3.6 Asynchronous notifications

The device can push change notifications to the UART controller in type `0x40` frames. They mirror the USB notification endpoint, so a UART client can follow parameter changes, preset loads and input format changes without polling.

**Enabling.** Set `notify_enable` in `UartCtrlConfig` to 1. Like all interface configuration this is done over USB only.

**Frame.** `A5 40 00 lenL lenH packet[len] crcL crcH`. Byte 2 is a fixed `0x00` in the status position. `packet` is the v2 notification packet byte for byte, as described in `Documentation/Features/notification_protocol_v2_spec.md`. The v1 master-volume packet is never sent over UART.

**Idle priority.** A notification goes out only between frames while nothing else is being sent, parsed or dispatched. It never splits or delays a response. A client that sends requests back to back can starve notifications indefinitely.

**Recovery.** Every v2 packet carries an 8-bit sequence number for this consumer. A jump of more than 1 (allowing for the wrap at 256) means events were dropped. Recover by reading the whole state with a `STORE.ALL_PARAMS` GET (3.7).

A client that leaves notifications off never sees a `0x40` frame. It should still ignore a stray one rather than treat it as an error.

### 3.7 Bulk transfer

`STORE.ALL_PARAMS` (page `0x02`, command `0x40`) moves the whole parameter set as one `WireBulkParams` blob. On firmware 1.1.7 the blob is **6136 bytes** (wire format V32). It travels in one GET response or one SET request.

- **GET.** Send `wValue` 0 and `wLength` 0 for the whole blob, or a smaller `wLength` for a prefix. The reply streams straight out of the device's bulk buffer.
- **SET.** The payload must be the whole blob, header included. Any shorter length fails with `VC_ERR_LENGTH`. A payload longer than the bulk buffer is discarded unread and answered with `CTRL_STATUS_OVERSIZE`. A header that does not match this device (platform, channel counts, wire format version, payload length) fails with `VC_ERR_VALUE`. Status `0x00` means the blob passed these checks and the main loop will apply it. Watch notifications or read the state back to see it applied.
- **Shared buffer.** One bulk buffer serves USB, UART and I2C. While another transport holds it, or while a previous bulk SET is still being applied, a bulk request answers `CTRL_STATUS_BULK_LOCKED` (`0x04`). Retry shortly. The device frees the buffer from an external owner that makes no progress for 500 ms.

Budget the wire time. At 8N1 a byte costs about 10 bit times.

| Baud | Approximate time for one 6136-byte bulk frame |
|---|---|
| 9600 | 6.4 s |
| 115200 | 0.53 s |
| 460800 | 0.13 s |
| 1000000 | 0.06 s |

For interactive control use the per-parameter commands. Use bulk for a full snapshot or restore, at a high baud rate to keep the buffer hold short.

---

## 4. I2C Transport

### 4.1 Electrical and bus parameters

- **Role.** Target only. The DSPi never starts a bus transfer, so there is no notification channel on I2C. An I2C controller polls the GET commands it cares about.
- **Addressing.** 7-bit, `0x08` to `0x77`, default `0x42`.
- **Speed.** Up to 400 kHz (Fast-mode).
- **Pull-ups.** The device enables the weak internal pull-ups as a safety net. Fit proper external pull-ups (typically 2.2k to 4.7k to 3.3 V).
- **Logic level.** 3.3 V.

### 4.2 Frame formats

A request is one I2C write transaction. It carries an 8-byte header and then the SET payload, if there is one.

```
Request (I2C write):
  [type][bReq][wValL][wValH][wIdxL][wIdxH][wLenL][wLenH][payload...]
    type 0x01 = SET  (wLen payload bytes follow the header)
    type 0x02 = GET  (no payload; wLen caps the reply, 0 = whole reply)
```

The response is read back with one or more I2C read transactions.

```
Response (I2C read):
  [status][lenL][lenH][payload...]   then 0xFF padding past the end
    len is 0 and there is no payload unless status is 0x00 on a GET
```

There is no sync byte and no CRC. The header matches the UART frame body from the type byte to `wLenH`, so one request builder serves both transports. `wIdxH` is the page and `wIdxL` is ignored.

**Reserved request types.** Only `0x01` and `0x02` are defined. Types `0x03` and up are reserved for future read-channel selectors, for example a notification drain. Such a write would select what later reads return, and a normal request would reset the selection to its own response. Current firmware answers any other type with `CTRL_STATUS_FRAME_ERROR`. `XPORT.STATUS` byte 4 (`proto_version`) will rise above 1 when a new type exists.

### 4.3 The BUSY frame and repeated-START

The main loop dispatches requests, so a read that arrives before dispatch finds no response yet. It returns a BUSY frame and then padding.

```
  [0x01, 0x00, 0x00]   (status CTRL_STATUS_BUSY, len 0)
```

Read again a moment later. With repeated-START (write, then read in one combined transaction) the first read usually returns BUSY. Without repeated-START, write, wait briefly, then poll the read until the status is not `0x01`.

### 4.4 Chunked, resumable reads

A parked response survives across read transactions until it has been read completely. A controller with a small buffer can read it in pieces, and each read continues where the last stopped. Past the end the device returns `0xFF`.

A new request write discards any unread response. Finish reading before sending the next request.

### 4.5 Clock stretching

The device stretches SCL only for interrupt latency at FIFO boundaries. It never stretches while waiting for the main loop. The BUSY frame covers that case.

### 4.6 Bootloader entry

`SYS.ENTER_BOOTLOADER` (page `0x01`, command `0x30`, zero-length SET) parks its OK response and reboots into the USB bootloader once the controller has read that response. If the controller never reads it, the device reboots after 1 s anyway.

### 4.7 Bulk transfer

`STORE.ALL_PARAMS` works as in 3.7. A GET reply of 6136 bytes is read out in chunks per 4.4. A SET is one long write transaction of the 8-byte header plus 6136 payload bytes. At 400 kHz the blob takes about 140 ms of bus time plus transaction overhead. The same shared buffer and `BULK_LOCKED` rule apply. A parked bulk reply that is not read for 1 s is dropped and the buffer freed. Send the GET again.

---

## 5. Shared Behavior

### 5.1 The status byte

Byte 0 of every UART and I2C response is the status. It holds one of two kinds of code.

**Transport codes, `0x01` to `0x07`.** The transport could not run the command. These are never recorded in `SYS.LAST_RESULT`.

| Code | Name | Meaning and client action |
|---|---|---|
| `0x00` | `CTRL_STATUS_OK` | Success. On a GET, `len` payload bytes follow |
| `0x01` | `CTRL_STATUS_BUSY` | A USB transfer was holding the shared buffers, or (I2C) the request has not been dispatched yet. Retry |
| `0x04` | `CTRL_STATUS_BULK_LOCKED` | The bulk buffer is held by another transport or by a pending bulk apply. Retry |
| `0x05` | `CTRL_STATUS_CRC_ERROR` | UART only. The frame failed its CRC and was not dispatched. Resend |
| `0x06` | `CTRL_STATUS_OVERSIZE` | The payload is larger than the transport can hold: 64 bytes, or the bulk buffer for `STORE.ALL_PARAMS` |
| `0x07` | `CTRL_STATUS_FRAME_ERROR` | I2C only. The write was truncated, too long, or had an unknown type |

Codes `0x02` and `0x03` belonged to the 1.1.6 protocol. Firmware 1.1.7 never sends them on a paged request, so a client that receives `0x02` in answer to its `SYS.PROTOCOL_INFO` probe is talking to old firmware.

**Command results, `0x10` and up.** The request reached the dispatcher, and this is its answer. It is the same code a USB host reads from `SYS.LAST_RESULT` after a STALL.

| Code | Name | Meaning |
|---|---|---|
| `0x10` | `VC_ERR_UNKNOWN` | No such page or command on this build (including every page 0 request) |
| `0x11` | `VC_ERR_DIRECTION` | The command has no handler for this direction |
| `0x12` | `VC_ERR_LENGTH` | SET payload length outside the allowed range |
| `0x13` | `VC_ERR_ADDRESS` | `wValue` out of range |
| `0x14` | `VC_ERR_VALUE` | A payload field is out of range |
| `0x15` | `VC_ERR_UNSUPPORTED` | Not available on this platform |
| `0x16` | `VC_ERR_BLOCKED` | Not allowed on this transport (1.3) |
| `0x17` | `VC_ERR_BUSY` | The command asks for a retry, for example while an earlier write still waits for the main loop |
| `0x18` | `VC_ERR_STATE` | Not allowed in the current state |
| `0x19` | `VC_ERR_PIN_INVALID` | GPIO not usable for this function |
| `0x1A` | `VC_ERR_PIN_IN_USE` | GPIO already claimed |

**One rule.** Transport conditions use the `0x0x` codes. Command results use the `0x1x` codes. Any non-zero status is a failure, and only `0x01`, `0x04` and `0x17` are worth retrying unchanged. `0x01` and `0x17` both mean "try again". `0x01` says the request never reached the command, and `0x17` says the command itself declined for now. `0x16` and the retired `0x03` both mean "USB only", and paged commands only ever use `0x16`.

**What `0x00` means on a SET.** The command accepted the request. For most commands the change has taken effect. Commands that write flash, restart the audio pipeline or move pins are deferred. For them `0x00` means the request passed validation and is queued for the main loop. Their completion shows up in notifications and readback, and `SYS.COMMAND_INFO` marks them with flag bit `0x0008`.

**`SYS.LAST_RESULT`.** Page `0x01`, command `0x14`, 8 bytes. It is kept separately for USB, UART and I2C, so each transport reads its own last result. The bytes are the command ID (u16, `(page << 8) | cmd`), the direction (1 for GET), the result code and the `wValue` of that request (u16), then two zero bytes. Reading it does not overwrite it. On UART and I2C the status byte already carries the same code.

### 5.2 Reply and payload size limits

| Limit | Value | Notes |
|---|---|---|
| Non-bulk SET payload | 64 bytes | `SYS.PROTOCOL_INFO` bytes 8 and 9 |
| Non-bulk GET reply | 256 bytes | `SYS.PROTOCOL_INFO` bytes 10 and 11. Delivered whole on every transport |
| Bulk (`STORE.ALL_PARAMS`) | 6136 bytes in 1.1.7 | Read the size from the blob header, not from this table |

A GET's `wLength` caps the reply exactly as on USB. A reply longer than `wLength` is cut to `wLength`, and `wLength` 0 means the whole reply. Commands with longer data, such as the spectrum analyser bins, take a byte offset in `wValue` and are read in pieces.

### 5.3 Flash blackout

Any command that writes flash (preset save, load and delete, device-level saves, factory reset, an interface config apply, a persisting bulk apply) causes an interrupt blackout of about 45 ms.

- **UART.** Bytes that arrive during the blackout can be lost.
- **I2C.** A transaction in progress stalls until it ends.

Use timeouts of a few hundred milliseconds and retry. Do not send a request immediately behind a flash-writing command.

### 5.4 Pin collisions

- A **disabled** interface holds no pins.
- All validation happens when a config is applied, and at boot.
- A **live** interface reserves its pins. Any other pin command that targets one of them fails with `VC_ERR_PIN_IN_USE` (`PIN_CONFIG_PIN_IN_USE` in the status structures).
- A stored config whose pins collide at boot leaves that interface down (2.5).
- An I2C target interface and the Control Surfaces display (an I2C master) must use different hardware instances.

Reconfiguring an interface tears it down before validating the new pins, so moving it from one valid pin pair to another never collides with itself.

### 5.5 Factory reset

Interface configs are device-level and survive a factory reset. To return an interface to its disabled default, write a disabled config over USB.

---

## 6. Worked Examples

All examples send `wIndex` low byte `0x00`. The CRCs were computed with the algorithm in 3.3.

### 6.1 UART: the discovery probe (GET)

GET `SYS.PROTOCOL_INFO` is page `0x01`, command `0x10`, with `wValue` 0 and `wLength` 16.

```
Host -> device:
  A5 02 10 00 00 00 01 10 00 A0 BD

  A5          sync
  02          type: GET
  10          bRequest 0x10
  00 00       wValue 0
  00 01       wIndex 0x0100: page 0x01
  10 00       wLength 16
  A0 BD       CRC16, low byte first

Device -> host (RP2350 release build, 12 pages):
  A5 82 00 10 00 44 53 50 69 01 00 0C 00 40 00 00 01 00 00 00 00 C5 E2

  A5 82       sync, type: GET response
  00          status 0x00
  10 00       len 16
  44 53 50 69 "DSPi"
  01 00       protocol 1.0
  0C 00       12 pages, flags 0
  40 00       largest SET payload 64
  00 01       largest GET reply 256
  00 00 00 00 reserved
  C5 E2       CRC16
```

### 6.2 UART: set the master volume (SET with payload)

SET `ROUTE.MASTER_VOLUME` is page `0x05`, command `0x50`, with `wValue` 0 and a little-endian f32 payload. The target, -20.0 dB, is `00 00 A0 C1`.

```
Host -> device:
  A5 01 50 00 00 00 05 04 00 00 00 A0 C1 71 3D

  A5          sync
  01          type: SET
  50          bRequest 0x50
  00 00       wValue 0
  00 05       wIndex 0x0500: page 0x05
  04 00       wLength 4
  00 00 A0 C1 payload, -20.0f
  71 3D       CRC16

Device -> host:
  A5 81 00 00 00 4C 2F        status 0x00, len 0, CRC16
```

Reading it back is a GET of the same command, `A5 02 50 00 00 00 05 04 00 3B 73`. The device answers `A5 82 00 04 00 00 00 A0 C1 AB 91`.

### 6.3 UART: clear the clip flags (zero-length SET)

SET `MEAS.CLIP_CLEAR` is page `0x09`, command `0x12`, with no payload.

```
Host -> device:
  A5 01 12 00 00 00 09 00 00 E4 BF

  A5          sync
  01          type: SET
  12          bRequest 0x12
  00 00       wValue 0
  00 09       wIndex 0x0900: page 0x09
  00 00       wLength 0
  E4 BF       CRC16

Device -> host:
  A5 81 00 00 00 4C 2F        status 0x00
```

### 6.4 UART: two refusals

A GET that leaves the page at 0, as a 1.1.6-era client would, is refused.

```
Host -> device:   A5 02 10 00 00 00 00 10 00 90 8A
Device -> host:   A5 82 10 00 00 F3 F7          (status 0x10 VC_ERR_UNKNOWN)
```

A SET of `XPORT.UART_CONFIG` sent over the UART itself is refused with `A5 81 16 00 00 8F DE` (status `0x16` `VC_ERR_BLOCKED`).

### 6.5 I2C: the same three requests

The header is the UART frame body without the sync byte and the CRC.

```
GET SYS.PROTOCOL_INFO
  write: 02 10 00 00 00 01 10 00
  read:  00 10 00 44 53 50 69 01 00 0C 00 40 00 00 01 00 00 00 00  FF FF ...
         (status 0x00, len 16, then the 16-byte reply from 6.1, then padding)

SET ROUTE.MASTER_VOLUME to -20.0 dB
  write: 01 50 00 00 00 05 04 00 00 00 A0 C1
  read:  00 00 00  FF FF ...                  (status 0x00, len 0)

SET MEAS.CLIP_CLEAR (zero-length)
  write: 01 12 00 00 00 09 00 00
  read:  00 00 00  FF FF ...

A read that lands before dispatch returns 01 00 00 (BUSY). Read again.
```

---

## 7. Related Documentation

- `Documentation/commands.md` and `Documentation/protocol/dspi_commands.json`: every command, its payload and its flags.
- `Documentation/Features/vendor_command_overhaul_plan.md`: the paged protocol design, sections 3 and 5.14.
- `Documentation/Features/notification_protocol_v2_spec.md`: the notification packet carried in UART `0x40` frames.
- `Documentation/current_architecture.md`, section "External Control Interfaces (UART / I2C Target)": the firmware side.
- `firmware/DSPi/vendor/vc_ctrl.h`: the transport entry points and the status codes.
