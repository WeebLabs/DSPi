# DSPi Vendor Command Reference (Control Protocol)

This document describes the complete USB vendor control protocol exposed by the
DSPi firmware. It is written for a developer building a **remote control bridge**
(e.g. a TCP/WiFi gateway, an ESP32 host, or a mobile app) that needs to drive the
device without reading the firmware source.

Everything here is derived from `firmware/DSPi/config.h`,
`firmware/DSPi/vendor_commands.c`, `firmware/DSPi/bulk_params.h`, and the
supporting subsystem headers. The reference host implementation lives in
`tools/dspi_test/device.py` and `tools/dspi_poke/dspi_poke.py`.

> Writing style note: this doc avoids em-dashes per project convention.

---

## 1. Transport conventions

All control happens over **USB EP0 control transfers** to the vendor interface.
A TCP/WiFi bridge must translate each command into a control transfer with the
exact `bmRequestType`, `bRequest`, `wValue`, `wIndex`, and payload shown below,
then relay the response bytes back.

| Field | Value |
|-------|-------|
| USB VID:PID | `0x2E8B : 0xFEAA` |
| Vendor interface number | `2` |
| `bmRequestType` for reads (Device to Host) | `0xC1` (IN \| Vendor \| Interface) |
| `bmRequestType` for writes (Host to Device) | `0x41` (OUT \| Vendor \| Interface) |
| `bRequest` | the command ID (hex) from the tables below |
| `wIndex` | `2` (the vendor interface number) for every application command |
| `wValue` | command-specific (channel, slot, packed fields, or 0) |
| Byte order | **little-endian** for all multi-byte integers |
| Floats | **IEEE 754 single-precision** (4 bytes, little-endian) |
| Max single-transfer payload | 64 bytes (larger transfers use EP0 chunking; see Bulk) |

`wIndex` is **never** used to carry application parameters; it is always the
interface number `2`. (The only reserved use of `wIndex` is the Microsoft OS 2.0
descriptor handshake on `bRequest=0x01`, which application code must not touch.)

### 1.1 Read vs. write direction (important gotcha)

The firmware splits handlers into a **GET path** (`bmRequestType=0xC1`, IN) and a
**SET path** (`bmRequestType=0x41`, OUT). Most `REQ_SET_*` opcodes are true OUT
transfers that carry their payload in the data stage.

**However, a group of write/action commands are dispatched on the IN (read) path.**
They carry all their parameters packed into `wValue` and return a 1-byte (or
2-byte) status. You must issue these as a **GET / IN transfer (`0xC1`)** even
though they mutate device state. They are flagged **"write-as-read"** throughout
this doc. Treating them as OUT transfers will STALL or silently fail.

Write-as-read commands: `0x51`, `0x52`, `0x53`, `0x7C`, `0x83`, `0x90`, `0x91`,
`0x92`, `0xB1`, `0xB3`, `0xC0`, `0xC2`, `0xC4`, `0xC6`, `0xC8`, `0xD6`, `0xE4`,
`0xEC`, `0xF0`, `0xF1`.

### 1.2 Control-stage flow

* **Reads (`0xC1`)**: SETUP then IN data stage of `wLength` bytes, then OUT status
  (ZLP). Request the exact length shown in the "Response" column.
* **Writes (`0x41`)**: SETUP then OUT data stage carrying the payload, then IN
  status (ZLP). A zero-length payload write is ACKed with no data stage.
* **STALL** means the firmware refused the request (unknown opcode, bad index,
  oversized payload, or a handler that returned failure). A bridge should surface
  this as an error rather than retrying blindly.

### 1.3 Deferred operations and the "busy window"

Flash writes and pipeline-affecting changes (preset load/save/delete, factory
reset, output-type switch, pin move, input-source switch, bulk apply) are
**deferred to the main loop**. The vendor handler returns immediately with an
"accepted" status; the actual work runs later and briefly disables the USB
control IRQ. During that window (tens of ms, up to ~45 ms per flash sector) a
control transfer can **time out or STALL transiently**.

A bridge must tolerate this: on timeout/STALL right after a deferred command,
back off ~150 ms and retry, and poll a cheap read like `REQ_GET_PLATFORM`
(`0x7F`) to confirm the device is responsive again before continuing. The
reference `device.py` retries timeouts 4 times with 150 ms backoff and re-acquires
on re-enumeration.

### 1.4 Confirming a deferred write actually applied

Because deferred writes return "accepted" before they run, the return status is
**not** proof the value took effect (validation may still reject it later, e.g.
DAC mute pin conflicts). For commands that matter, follow up with the matching
**readback** command (Section 11) and compare against what you sent.

---

## 2. Index and addressing reference

Several commands address "channels", "outputs", "pin outputs", or "bands". These
are **different index spaces**. Mixing them up is the most common integration bug.

### 2.1 Channel index (`NUM_CHANNELS` = 7 on RP2040, 11 on RP2350)

Used by EQ, per-channel delay, channel names, combined status peaks.

| Index | RP2040 | RP2350 |
|-------|--------|--------|
| 0 | Master L | Master L |
| 1 | Master R | Master R |
| 2 | S/PDIF 1 L | S/PDIF 1 L |
| 3 | S/PDIF 1 R | S/PDIF 1 R |
| 4 | S/PDIF 2 L | S/PDIF 2 L |
| 5 | S/PDIF 2 R | S/PDIF 2 R |
| 6 | PDM sub | S/PDIF 3 L |
| 7 | - | S/PDIF 3 R |
| 8 | - | S/PDIF 4 L |
| 9 | - | S/PDIF 4 R |
| 10 | - | PDM sub |

### 2.2 Output index (`NUM_OUTPUT_CHANNELS` = 5 on RP2040, 9 on RP2350)

Used by the matrix mixer and per-output gain/mute/enable/delay commands. Output
index `out` maps to channel index `out + 2` (it excludes the two master channels).

| Output index | Maps to channel | Meaning |
|--------------|-----------------|---------|
| 0 | 2 | S/PDIF 1 L |
| 1 | 3 | S/PDIF 1 R |
| ... | ... | ... |
| `NUM_OUTPUT_CHANNELS-1` | last channel | PDM sub |

### 2.3 Pin-output index (`NUM_PIN_OUTPUTS` = 3 on RP2040, 5 on RP2350)

Used by `REQ_SET/GET_OUTPUT_PIN`. This is **per physical output instance**, not
per channel. Each S/PDIF/I2S slot carries 2 channels.

| Pin-output index | Output instance | Default GPIO |
|------------------|-----------------|--------------|
| 0 | S/PDIF/I2S slot 0 | 6 |
| 1 | S/PDIF/I2S slot 1 | 7 |
| 2 (RP2350) | S/PDIF/I2S slot 2 | 8 |
| 3 (RP2350) | S/PDIF/I2S slot 3 | 9 |
| last (2 on RP2040, 4 on RP2350) | PDM sub | 10 |

Slot indices `0 .. NUM_SPDIF_INSTANCES-1` are also the **slot index** used by the
output-type and I2S commands (`0xC0`/`0xC1`).

### 2.4 EQ band index

| Band index | Meaning |
|------------|---------|
| 0 .. 9 | Parametric EQ bands (10 active bands per channel; `channel_band_counts[ch]`) |
| 10 .. 19 | Reserved gap (rejected) |
| 20 .. 23 | Crossover filter bands (output channels only, `channel >= 2`) |

Crossover bands are valid only on **output** channels (index >= 2); they are
rejected on the master channels.

### 2.5 Status codes

Pin/output/config commands return one of (`config.h`):

| Code | Name | Meaning |
|------|------|---------|
| 0x00 | `PIN_CONFIG_SUCCESS` | Applied (or accepted for deferred work) |
| 0x01 | `PIN_CONFIG_INVALID_PIN` | GPIO not allowed, or bad enum value |
| 0x02 | `PIN_CONFIG_PIN_IN_USE` | GPIO already claimed by another function |
| 0x03 | `PIN_CONFIG_INVALID_OUTPUT` | Output/slot index out of range, or feature off |
| 0x04 | `PIN_CONFIG_OUTPUT_ACTIVE` | Must disable the output first |

Preset commands return (`config.h`):

| Code | Name |
|------|------|
| 0x00 | `PRESET_OK` |
| 0x01 | `PRESET_ERR_INVALID_SLOT` |
| 0x02 | `PRESET_ERR_SLOT_EMPTY` |
| 0x03 | `PRESET_ERR_CRC` |
| 0x04 | `PRESET_ERR_FLASH_WRITE` |

`REQ_SAVE_PARAMS` / `REQ_FACTORY_RESET` return `FLASH_OK` = `0x00`.

### 2.6 Valid GPIO pins

`is_valid_gpio_pin()` rejects: GPIO 12 (UART TX), GPIO 23-25 (power/LED). Upper
bound is 28 on RP2040, 29 on RP2350. Any pin command also rejects a GPIO already
in use by an output, the I2S BCK/LRCLK pair, MCK (if enabled), the S/PDIF RX pin,
the I2S RX pin, or a DAC hardware-mute pin.

---

## 3. Identity and platform

| ID | Name | Dir | wValue | Payload / Response | Notes |
|----|------|-----|--------|--------------------|-------|
| 0x7E | `GET_SERIAL` | R | 0 | resp **16 bytes** | Device serial string (raw bytes, NUL-padded) |
| 0x7F | `GET_PLATFORM` | R | 0 | resp **6 bytes** | `[0]`=platform (0=RP2040, 1=RP2350), `[1]`=fw major, `[2]`=legacy (minor<<4)\|patch nibbles, `[3]`=`NUM_OUTPUT_CHANNELS`, `[4]`=fw minor, `[5]`=fw patch |
| 0x80 | `GET_BUILD_INFO` | R | 0 | resp **64 bytes** | `[0..47]`=`git describe --always --dirty` string, `[48..59]`=build date `YYYY-MM-DD`, `[60..63]`=reserved 0. ASCII, NUL-padded. Provenance for humans only; no software may compare it |

`GET_PLATFORM` is the canonical liveness/probe read; use it to detect the platform
(and therefore channel/output counts) before issuing anything else.

---

## 4. EQ and filters

### 4.1 Set one EQ band (`REQ_SET_EQ_PARAM`, 0x42, write)

`bmRequestType=0x41`. `wValue`/`wIndex` carry no band info; the full descriptor
is in the **16-byte payload** (`EqParamPacket`):

| Offset | Type | Field |
|--------|------|-------|
| 0 | u8 | `channel` (0 .. NUM_CHANNELS-1) |
| 1 | u8 | `band` (0..9 PEQ, or 20..23 crossover) |
| 2 | u8 | `type` (FilterType enum, see 4.3) |
| 3 | u8 | `bypass` (exactly 1 = bypassed; any other value = active) |
| 4 | f32 | `freq` (Hz) |
| 8 | f32 | `Q` |
| 12 | f32 | `gain_db` |

The write is ignored if `channel`/`band` is out of range or a crossover band is
addressed on a master channel.

**Optional 18-byte payload (Linkwitz Transform target Q).** For the Linkwitz
Transform type (`11`, see 4.3) the band needs a fourth parameter, `Qp`, that does
not fit the 16-byte packet. Send an **18-byte** payload: the 16-byte `EqParamPacket`
followed by a `uint16` LE `qp` at offsets 16-17, encoded as `Q*512` (`0` = the
0.707 default). A plain **16-byte** payload is still accepted and **preserves the
stored `qp`** for that band (so non-LT writes never need the extra bytes). For a
Linkwitz Transform band, `freq` = `f0`, `Q` = `Q0`, and `gain_db` carries `fp` in
Hz (the `gain_db` field is otherwise unused for this type).

### 4.2 Read one EQ scalar (`REQ_GET_EQ_PARAM`, 0x43, read)

`bmRequestType=0xC1`. There is **no full-packet read**; you read one scalar at a
time. Build `wValue` as:

```
wValue = (channel << 8) | (band << 3) | param
```

* `channel`: bits [15:8]
* `band`: bits [7:3] (5 bits, 0..31, so crossover bands 20..23 are addressable)
* `param`: bits [2:0]: `0`=type, `1`=freq, `2`=Q, `3`=gain_db, `4`=bypass,
  `5`=Linkwitz-Transform `qp_x512`

Response is always **4 bytes**. For `param=0` and `param=4` the value is a small
integer in the low byte (remaining bytes zero); for `1/2/3` it is an f32. For
`param=5` it is the stored `qp_x512` (`Q*512`) as a `u32` (`0` = the 0.707 default).

To read an entire band, issue 5 reads, or read everything at once via
`REQ_GET_ALL_PARAMS` (Section 13).

### 4.3 FilterType enum

| Value | Type |
|-------|------|
| 0 | Flat (off) |
| 1 | Peaking |
| 2 | Low shelf |
| 3 | High shelf |
| 4 | Low pass |
| 5 | High pass |
| 6 | Notch |
| 7 | All pass (2nd-order RBJ) |
| 8 | First-order all pass (`FILTER_ALLPASS1`) |
| 9 | First-order low shelf (`FILTER_LOWSHELF1`) |
| 10 | First-order high shelf (`FILTER_HIGHSHELF1`) |
| 11 | Linkwitz Transform (`FILTER_LINKWITZ_TRANSFORM`): `freq`/`Q`/`gain_db` map to `f0`/`Q0`/`fp` (Hz); `Qp` is the sidecar (see 4.1 / param 5) |
| 12..31 | Reserved (future PEQ types) |
| 32..63 | Crossover types (LR/Butterworth/Bessel, see below) |

Crossover encoding (used in crossover bands 20..23):

| Family | LP / HP values |
|--------|----------------|
| Linkwitz-Riley LR2/LR4/LR6/LR8 | 32/33, 34/35, 36/37, 38/39 |
| Butterworth BW1..BW8 | 40/41 ... 54/55 |
| Bessel BES2/BES4/BES6/BES8 | 56/57, 58/59, 60/61, 62/63 |

For crossover filter types, `Q` and `gain_db` are ignored by the design code (only
`type` and `freq` matter), but should still be sent for wire parity.

### 4.4 Per-band bypass

| ID | Name | Dir | wValue | Payload / Response |
|----|------|-----|--------|--------------------|
| 0xD8 | `SET_BAND_BYPASS` | W (0x41) | `(channel << 8) \| band` | payload **1 byte**: 1=bypass, else active |
| 0xD9 | `GET_BAND_BYPASS` | R (0xC1) | `(channel << 8) \| band` | resp **1 byte** (0/1) |

Accepts both PEQ (0..9) and crossover (20..23) band indices.

---

## 5. Gain, preamp, volume, mute, bypass

### 5.1 Input preamp

| ID | Name | Dir | wValue | Payload / Response | Readback |
|----|------|-----|--------|--------------------|----------|
| 0x44 | `SET_PREAMP` (legacy, all input ch) | W | 0 | payload f32 dB | 0x45 |
| 0x45 | `GET_PREAMP` (legacy, returns ch 0) | R | 0 | resp f32 dB | - |
| 0xD0 | `SET_PREAMP_CH` | W | input ch (0=L,1=R) | payload f32 dB | 0xD1 |
| 0xD1 | `GET_PREAMP_CH` | R | input ch | resp f32 dB | - |

`NUM_INPUT_CHANNELS` = 2.

### 5.2 Master volume (device-side output ceiling)

| ID | Name | Dir | wValue | Payload / Response |
|----|------|-----|--------|--------------------|
| 0xD2 | `SET_MASTER_VOLUME` | W | 0 | payload f32 dB |
| 0xD3 | `GET_MASTER_VOLUME` | R | 0 | resp f32 dB |

Range: `-127.0 .. 0.0` dB; **`-128.0` is the mute sentinel** (true -inf). Default
on a fresh device is `-20.0` dB. Master volume scales final output gain only; it
does **not** affect loudness, leveller, EQ, or crossfeed.

### 5.3 User volume / mute (mirrors the OS volume slider)

| ID | Name | Dir | wValue | Payload / Response |
|----|------|-----|--------|--------------------|
| 0xDA | `SET_USER_VOLUME` | W | 0 | payload f32 dB (clamped to `[-60, 0]`) |
| 0xDB | `GET_USER_VOLUME` | R | 0 | resp f32 dB |
| 0xDC | `SET_USER_MUTE` | W | 0 | payload **1 byte** (0/1) |
| 0xDD | `GET_USER_MUTE` | R | 0 | resp **1 byte** (0/1) |

User volume writes the same field the UAC1 host slider drives (so it round-trips
with the OS volume control) and is what loudness compensation keys off. The clamp
floor is `-CENTER_VOLUME_INDEX` = `-60` dB. `user_mute` is a separate, always-honored
vendor mute (distinct from the UAC1 mute key).

### 5.4 Master EQ bypass

| ID | Name | Dir | wValue | Payload / Response |
|----|------|-----|--------|--------------------|
| 0x46 | `SET_BYPASS` | W | 0 | payload **1 byte** (0/1) |
| 0x47 | `GET_BYPASS` | R | 0 | resp **1 byte** (0/1) |

### 5.5 Legacy per-channel gain/mute (master path, channels 0..2)

| ID | Name | Dir | wValue | Payload / Response |
|----|------|-----|--------|--------------------|
| 0x54 | `SET_CHANNEL_GAIN` | W | ch (0..2) | payload f32 dB |
| 0x55 | `GET_CHANNEL_GAIN` | R | ch (0..2) | resp f32 dB |
| 0x56 | `SET_CHANNEL_MUTE` | W | ch (0..2) | payload **1 byte** (0/1) |
| 0x57 | `GET_CHANNEL_MUTE` | R | ch (0..2) | resp **1 byte** (0/1) |

These are the legacy 3-channel gain/mute controls. For per-output control prefer
the matrix mixer commands (Section 8).

---

## 6. Delays

| ID | Name | Dir | wValue | Payload / Response |
|----|------|-----|--------|--------------------|
| 0x48 | `SET_DELAY` | W | channel index | payload f32 ms (negatives clamped to 0) |
| 0x49 | `GET_DELAY` | R | channel index | resp f32 ms |

`wValue` is a **full channel index** (0 .. NUM_CHANNELS-1). Max delay is 21 ms on
RP2040, 42 ms on RP2350. Per-output delay (Section 8) writes the same underlying
per-channel delay for `channel = output + 2`.

---

## 7. Loudness, crossfeed, leveller

### 7.1 Loudness compensation

| ID | Name | Dir | wValue | Payload / Response | Range |
|----|------|-----|--------|--------------------|-------|
| 0x58 | `SET_LOUDNESS` | W | 0 | 1 byte (0/1) | - |
| 0x59 | `GET_LOUDNESS` | R | 0 | 1 byte | - |
| 0x5A | `SET_LOUDNESS_REF` | W | 0 | f32 SPL | clamped 40..100 |
| 0x5B | `GET_LOUDNESS_REF` | R | 0 | f32 | - |
| 0x5C | `SET_LOUDNESS_INTENSITY` | W | 0 | f32 % | clamped 0..200 |
| 0x5D | `GET_LOUDNESS_INTENSITY` | R | 0 | f32 | - |
| 0xFA | `SET_LOUDNESS_MASK` | W | 0 | u16 LE output mask | bit k = output k |
| 0xFB | `GET_LOUDNESS_MASK` | R | 0 | u16 LE | - |

Loudness runs **per output channel** (post output-gain, pre delay), gated by the
output mask (bit k = output channel k, PDM included; bits above the platform's
output count are stored but ignored; default `0xFFFF` = all outputs). Mask writes
apply from the next packet with no recompute or reset. The mask exists from wire
format V19 / slot V26; on older firmware 0xFA/0xFB stall. Full behavioral spec:
`Features/loudness_compensation_spec.md`.

### 7.2 Crossfeed (headphone)

| ID | Name | Dir | wValue | Payload / Response | Range |
|----|------|-----|--------|--------------------|-------|
| 0x5E | `SET_CROSSFEED` | W | 0 | 1 byte (0/1) | - |
| 0x5F | `GET_CROSSFEED` | R | 0 | 1 byte | - |
| 0x60 | `SET_CROSSFEED_PRESET` | W | 0 | 1 byte | 0..3 |
| 0x61 | `GET_CROSSFEED_PRESET` | R | 0 | 1 byte | - |
| 0x62 | `SET_CROSSFEED_FREQ` | W | 0 | f32 Hz | 500..2000 |
| 0x63 | `GET_CROSSFEED_FREQ` | R | 0 | f32 | - |
| 0x64 | `SET_CROSSFEED_FEED` | W | 0 | f32 dB | 0..15 |
| 0x65 | `GET_CROSSFEED_FEED` | R | 0 | f32 | - |
| 0x66 | `SET_CROSSFEED_ITD` | W | 0 | 1 byte (0/1) | - |
| 0x67 | `GET_CROSSFEED_ITD` | R | 0 | 1 byte | - |

Preset values: 0=Default (700 Hz / 4.5 dB), 1=Chu Moy (700/6.0), 2=Meier
(650/9.5), 3=Custom (uses the freq/feed fields).

### 7.3 Volume leveller

| ID | Name | Dir | wValue | Payload / Response | Range |
|----|------|-----|--------|--------------------|-------|
| 0xB4 | `SET_LEVELLER_ENABLE` | W | 0 | 1 byte (0/1) | - |
| 0xB5 | `GET_LEVELLER_ENABLE` | R | 0 | 1 byte | - |
| 0xB6 | `SET_LEVELLER_AMOUNT` | W | 0 | f32 % | 0..100 |
| 0xB7 | `GET_LEVELLER_AMOUNT` | R | 0 | f32 | - |
| 0xB8 | `SET_LEVELLER_SPEED` | W | 0 | 1 byte | 0=Slow,1=Med,2=Fast |
| 0xB9 | `GET_LEVELLER_SPEED` | R | 0 | 1 byte | - |
| 0xBA | `SET_LEVELLER_MAX_GAIN` | W | 0 | f32 dB | 0..35 |
| 0xBB | `GET_LEVELLER_MAX_GAIN` | R | 0 | f32 | - |
| 0xBC | `SET_LEVELLER_LOOKAHEAD` | W | 0 | 1 byte (0/1) | - |
| 0xBD | `GET_LEVELLER_LOOKAHEAD` | R | 0 | 1 byte | - |
| 0xBE | `SET_LEVELLER_GATE` | W | 0 | f32 dBFS | -96..0 |
| 0xBF | `GET_LEVELLER_GATE` | R | 0 | f32 | - |

---

## 8. Matrix mixer and output channels

### 8.1 Crosspoint routing

| ID | Name | Dir | wValue | Payload / Response |
|----|------|-----|--------|--------------------|
| 0x70 | `SET_MATRIX_ROUTE` | W (0x41) | 0 | **8-byte** `MatrixRoutePacket` |
| 0x71 | `GET_MATRIX_ROUTE` | R (0xC1) | `(input << 8) \| output` | resp **8 bytes** (`MatrixRoutePacket`) |

`MatrixRoutePacket` (8 bytes):

| Offset | Type | Field |
|--------|------|-------|
| 0 | u8 | `input` (0..1, USB L/R) |
| 1 | u8 | `output` (0 .. NUM_OUTPUT_CHANNELS-1) |
| 2 | u8 | `enabled` (0/1) |
| 3 | u8 | `phase_invert` (0/1) |
| 4 | f32 | `gain_db` |

### 8.2 Per-output controls

`wValue` = output index (0 .. NUM_OUTPUT_CHANNELS-1).

| ID | Name | Dir | Payload / Response |
|----|------|-----|--------------------|
| 0x72 | `SET_OUTPUT_ENABLE` | W | 1 byte (0/1) |
| 0x73 | `GET_OUTPUT_ENABLE` | R | 1 byte |
| 0x74 | `SET_OUTPUT_GAIN` | W | f32 dB |
| 0x75 | `GET_OUTPUT_GAIN` | R | f32 |
| 0x76 | `SET_OUTPUT_MUTE` | W | 1 byte (0/1) |
| 0x77 | `GET_OUTPUT_MUTE` | R | 1 byte |
| 0x78 | `SET_OUTPUT_DELAY` | W | f32 ms |
| 0x79 | `GET_OUTPUT_DELAY` | R | f32 |

`SET_OUTPUT_ENABLE` enforces a Core 1 interlock: PDM and the Core-1 EQ-worker
outputs are mutually exclusive. Enabling a conflicting output is silently ignored.
Use `GET_CORE1_CONFLICT` (Section 9) to test before enabling.

---

## 9. Core 1 mode query

| ID | Name | Dir | wValue | Response |
|----|------|-----|--------|----------|
| 0x7A | `GET_CORE1_MODE` | R | 0 | 1 byte: 0=Idle, 1=PDM, 2=EQ worker |
| 0x7B | `GET_CORE1_CONFLICT` | R | proposed output index | 1 byte: 1 = enabling would conflict, 0 = OK |

---

## 10. Input source, S/PDIF RX, I2S RX

| ID | Name | Dir | wValue | Payload / Response | Notes |
|----|------|-----|--------|--------------------|-------|
| 0xE0 | `SET_INPUT_SOURCE` | W (0x41) | 0 | 1 byte: 0=USB, 1=SPDIF, 2=I2S | Deferred (pipeline reset) |
| 0xE1 | `GET_INPUT_SOURCE` | R | 0 | 1 byte (active source) | Reports active, not pending |
| 0xE2 | `GET_SPDIF_RX_STATUS` | R | 0 | **16 bytes** `SpdifRxStatusPacket` | See 10.1 |
| 0xE3 | `GET_SPDIF_RX_CH_STATUS` | R | 0 | **24 bytes** IEC 60958 channel status | Raw channel-status bytes |
| 0xE4 | `SET_SPDIF_RX_PIN` | **write-as-read (0xC1)** | new GPIO | resp **1 byte** status | Hot-swaps if SPDIF active; persist via preset save |
| 0xE5 | `GET_SPDIF_RX_PIN` | R | 0 | 1 byte | - |
| 0xED | `SET_INPUT_RATE` | W (0x41) | 0 | **u32** Hz (44100/48000/96000) | I2S-input rate authority |
| 0xEE | `GET_INPUT_RATE` | R | 0 | **8 bytes**: `u32 current_Hz`, `u32 selected_I2S_Hz` | - |
| 0xF1 | `SET_I2S_RX_PIN` | **write-as-read (0xC1)** | new GPIO | resp **1 byte** status | Mirrors 0xE4 |
| 0xF2 | `GET_I2S_RX_PIN` | R | 0 | 1 byte | - |

### 10.1 `SpdifRxStatusPacket` (16 bytes, read-only meter)

| Offset | Type | Field |
|--------|------|-------|
| 0 | u8 | `state` (0=Inactive, 1=Acquiring, 2=Locked, 3=Relocking) |
| 1 | u8 | `input_source` (active InputSource) |
| 2 | u8 | `lock_count` |
| 3 | u8 | `loss_count` |
| 4 | u32 | `sample_rate` Hz (0 if not locked) |
| 8 | u32 | `parity_errors` |
| 12 | u16 | `fifo_fill_pct` (0-100) |
| 14 | u16 | reserved |

When I2S is the active input the device is the **rate authority**: the rate is
selected with `SET_INPUT_RATE` (not detected). `i2s_input_rate` accepts only
44100 / 48000 / 96000.

---

## 11. LG Sound Sync

| ID | Name | Dir | wValue | Payload / Response |
|----|------|-----|--------|--------------------|
| 0xE6 | `SET_LG_SOUND_SYNC_ENABLE` | W (0x41) | 0 | 1 byte (0/1) |
| 0xE7 | `GET_LG_SOUND_SYNC_ENABLE` | R | 0 | 1 byte |
| 0xE8 | `GET_LG_SOUND_SYNC_STATUS` | R | 0 | **16 bytes** `LgSoundSyncStatus` |

`LgSoundSyncStatus` (16 bytes):

| Offset | Type | Field |
|--------|------|-------|
| 0 | u8 | `enabled` (user gate, persists on preset save) |
| 1 | u8 | `present` (read-only detection state) |
| 2 | u8 | `volume` (0..100, 0xFF = never decoded) |
| 3 | u8 | `muted` (0/1) |
| 4 | u8[12] | reserved |

The enable flag is per-preset; it persists only when you `REQ_PRESET_SAVE`.

---

## 12. Output type, I2S, and MCK configuration

These configure the **physical output bus**. Output-type, BCK, and MCK setters are
**write-as-read** (issue as `0xC1`); they return a 1-byte `PIN_CONFIG_*` status.

| ID | Name | Dir | wValue | Response | Notes |
|----|------|-----|--------|----------|-------|
| 0xC0 | `SET_OUTPUT_TYPE` | write-as-read | `(new_type << 8) \| slot` | 1 byte status | type 0=S/PDIF, 1=I2S; deferred switch |
| 0xC1 | `GET_OUTPUT_TYPE` | R | slot index | 1 byte (0/1) | - |
| 0xC2 | `SET_I2S_BCK_PIN` | write-as-read | new GPIO | 1 byte status | LRCLK = BCK+1; rejected while any slot is I2S |
| 0xC3 | `GET_I2S_BCK_PIN` | R | 0 | 1 byte | - |
| 0xC4 | `SET_MCK_ENABLE` | write-as-read | 0/1 | 1 byte status | - |
| 0xC5 | `GET_MCK_ENABLE` | R | 0 | 1 byte | - |
| 0xC6 | `SET_MCK_PIN` | write-as-read | new GPIO | 1 byte status | Must be a CLK_GPOUTn-capable pin |
| 0xC7 | `GET_MCK_PIN` | R | 0 | 1 byte | - |
| 0xC8 | `SET_MCK_MULTIPLIER` | write-as-read | 0=128x, 1=256x | 1 byte status | - |
| 0xC9 | `GET_MCK_MULTIPLIER` | R | 0 | 1 byte (0=128x, 1=256x) | - |

`slot` is `0 .. NUM_SPDIF_INSTANCES-1` (2 on RP2040, 4 on RP2350). BCK/LRCLK are
shared across all I2S slots. MCK is driven by hardware CLK_GPOUTn so its pin must
map to clk_gpout0..3 (GPIO 21 on RP2040; GPIO 13/15/21 on RP2350).

---

## 13. Bulk parameter transfer (read/write all state in one shot)

### 13.1 `REQ_GET_ALL_PARAMS` (0xA0, read)

`bmRequestType=0xC1`, `wValue=0`. Request `wLength` up to `sizeof(WireBulkParams)`
= **3664 bytes** (V14). The firmware fills the current live DSP state and returns
`min(wLength, 3664)` bytes via EP0 chunking (handle the 64-byte chunking and the
trailing ZLP in your USB layer). Read `header.payload_length` to learn the actual
length.

### 13.2 `REQ_SET_ALL_PARAMS` (0xA1, write)

`bmRequestType=0x41`, `wValue=0`. Send the full `WireBulkParams` (or a truncated
prefix down to the V2 size; older/shorter payloads are accepted by range gate, and
the firmware honors only the sections present per `header.format_version` and
`payload_length`). The apply is **deferred** to the main loop and brackets a muted,
synchronized pipeline reset; expect a busy window afterward. The host should
re-read `REQ_GET_ALL_PARAMS` afterward to confirm.

### 13.3 `WireBulkParams` layout (V14, 3664 bytes, little-endian)

> The byte layout has been stable since V11; V12-V14 only added `FilterType` enum
> values and version-gated field interpretations, so the size stays 3664 and the
> firmware still accepts any `format_version` from 2 up to the current 14. V14 added
> the first-order shelf types (`FILTER_LOWSHELF1`=9, `FILTER_HIGHSHELF1`=10), which
> ride in the existing `WireBandParams.type` byte.

Header first; all offsets are byte offsets from the start of the packet. Array
sections are sized at the **RP2350 maximum** and zero-padded on RP2040 (use the
header counts to know how many entries are valid).

| Offset | Size | Section | Contents |
|-------:|-----:|---------|----------|
| 0 | 16 | `header` | see 13.4 |
| 16 | 16 | `global` | f32 preamp_gain_db; u8 bypass; u8 loudness_enabled; u16 loudness_output_mask (V19+; reserved before V19); f32 loudness_ref_spl; f32 loudness_intensity_pct |
| 32 | 16 | `crossfeed` | u8 enabled; u8 preset; u8 itd_enabled; u8 rsv; f32 custom_fc; f32 custom_feed_db; u32 rsv |
| 48 | 16 | `legacy` | f32 gain_db[3]; u8 mute[3]; u8 rsv |
| 64 | 44 | `delays` | f32 delay_ms[11] (per channel, ms) |
| 108 | 144 | `crosspoints` | `WireCrosspoint[2][9]` (input-major); each 8 bytes: u8 enabled; u8 phase_invert; u8[2] rsv; f32 gain_db |
| 252 | 108 | `outputs` | `WireOutputChannel[9]`; each 12 bytes: u8 enabled; u8 mute; u8[2] rsv; f32 gain_db; f32 delay_ms |
| 360 | 8 | `pins` | u8 num_pin_outputs; u8 pins[5]; u8[2] rsv |
| 368 | 2112 | `eq` | `WireBandParams[11][12]` (channel-major); each 16 bytes (see 13.5) |
| 2480 | 352 | `channel_names` | `char[11][32]` (NUL-padded names) |
| 2832 | 16 | `i2s_config` | u8 output_types[4]; u8 bck_pin; u8 mck_pin; u8 mck_enabled; u8 mck_multiplier (128/256); u8[8] rsv |
| 2848 | 16 | `leveller` | u8 enabled; u8 speed; u8 lookahead; u8 rsv; f32 amount; f32 max_gain_db; f32 gate_threshold_db |
| 2864 | 16 | `preamp` | f32 preamp_db[2] (per input ch); u8[8] rsv |
| 2880 | 16 | `master_volume` | f32 master_volume_db (-128 = mute); u8[12] rsv |
| 2896 | 16 | `input_config` | u8 input_source; u8 spdif_rx_pin; u8 i2s_rx_pin; u8 i2s_input_rate (0=44.1k,1=48k,2=96k); u8[12] rsv |
| 2912 | 16 | `lg_sound_sync` | u8 enabled; u8 present(ro); u8 volume(ro); u8 muted(ro); u8[12] rsv |
| 2928 | 16 | `user_volume` | f32 user_volume_db; u8 user_mute; u8[11] rsv |
| 2944 | 16 | `dac_hw_mute` | u8 enabled; u8 active_low; u8 pin; u8 rsv; u16 hold_ms; u16 release_ms; u8[8] rsv |
| 2960 | 704 | `crossovers` | `WireBandParams[11][4]` (channel-major); crossover bands, columns map to vendor band 20..23 |

On bulk SET, only `enabled` is honored in the `lg_sound_sync` section; `present`/
`volume`/`muted` are runtime-only and ignored.

### 13.4 `WireHeader` (16 bytes)

| Offset | Type | Field |
|--------|------|-------|
| 0 | u8 | `format_version` (currently 14) |
| 1 | u8 | `platform_id` (0=RP2040, 1=RP2350) |
| 2 | u8 | `num_channels` (7 or 11) |
| 3 | u8 | `num_output_channels` (5 or 9) |
| 4 | u8 | `num_input_channels` (2) |
| 5 | u8 | `max_bands` (12) |
| 6 | u16 | `payload_length` (actual packet size) |
| 8 | u16 | `fw_version_major` |
| 10 | u16 | `fw_version_minor` |
| 12 | u32 | reserved |

### 13.5 `WireBandParams` (16 bytes, used by `eq` and `crossovers`)

| Offset | Type | Field |
|--------|------|-------|
| 0 | u8 | `type` (FilterType enum) |
| 1 | u8 | `bypass` (1 = bypassed, else active) |
| 2 | u8[2] | reserved; carries the Linkwitz-Transform `qp` (u16 LE, `Q*512`) for LT bands (`type` = 11), else must be 0 (wire format V22+) |
| 4 | f32 | `freq` (Hz) |
| 8 | f32 | `q` |
| 12 | f32 | `gain_db` |

The band index is implicit in array position (row = channel, column = band).

> **Wire format version.** The `reserved[2]` reuse for the Linkwitz-Transform
> `qp` bumped `WIRE_FORMAT_VERSION` to **22** (byte layout and struct size are
> unchanged). Unlike the single-band command, the bulk bytes are authoritative:
> apply always overwrites the stored `qp` (for an LT band `0` selects the 0.707
> default; for any other type it is forced to `0`). For an LT band,
> `freq`/`q`/`gain_db` hold `f0`/`Q0`/`fp` (Hz).

---

## 14. Meter / status polling

These are the commands a remote UI polls for live metering. All are reads.

### 14.1 `REQ_GET_STATUS` (0x50, read): `wValue` selects the sub-query

Most sub-queries return **4 bytes** (a `u32`/packed value). `wValue=9` is special
and returns a combined packet.

| wValue | Returns (4 bytes unless noted) |
|-------:|--------------------------------|
| 0 | peaks[0] (low u16) \| peaks[1] (high u16) = Master L / R peak |
| 1 | peaks[2] \| peaks[3] = S/PDIF 1 L / R peak |
| 2 | peaks[4] (low u16) \| cpu0_load (byte 2) \| cpu1_load (byte 3) |
| 3 | `pdm_ring_overruns` |
| 4 | `pdm_ring_underruns` |
| 5 | `pdm_dma_overruns` |
| 6 | `pdm_dma_underruns` |
| 7 | `spdif_overruns` |
| 8 | `spdif_underruns` |
| 9 | **Combined packet** (see below) |
| 10 | `usb_audio_packets` |
| 11 | `usb_audio_alt_set` |
| 12 | `usb_audio_mounted` |
| 13 | system clock `clk_sys` Hz |
| 14 | core voltage (mV) |
| 15 | sample rate (Hz) |
| 16 | temperature (centi-degrees C; signed int16 in low 2 bytes) |
| 17 | total S/PDIF DMA starvations |
| 18 | S/PDIF instance 0 DMA starvations |
| 19 | S/PDIF instance 1 DMA starvations |
| 20 | S/PDIF instance 2 DMA starvations |
| 21 | S/PDIF instance 3 DMA starvations |
| 22 | USB audio ring overruns |

**Combined packet (`wValue=9`)** length = `NUM_CHANNELS*2 + 4` bytes (18 on
RP2040, 26 on RP2350):

```
[ peaks[0..NUM_CHANNELS-1] as u16 LE ] [ cpu0_load u8 ] [ cpu1_load u8 ] [ clip_flags u16 LE ]
```

`peaks[]` are per-channel peak meters; `clip_flags` is a sticky per-channel clip
bitmask. CPU loads are 0..100 (percent).

### 14.2 Clip flags

| ID | Name | Dir | wValue | Response |
|----|------|-----|--------|----------|
| 0x83 | `CLEAR_CLIPS` | **write-as-read (0xC1)** | 0 | **2 bytes** = clip_flags that were set (then cleared) |

Read-then-clear: returns the latched clip bitmask and resets it in one transfer.

### 14.3 Buffer statistics

| ID | Name | Dir | wValue | Response |
|----|------|-----|--------|----------|
| 0xB0 | `GET_BUFFER_STATS` | R | 0 | **44 bytes** `BufferStatsPacket` |
| 0xB1 | `RESET_BUFFER_STATS` | **write-as-read (0xC1)** | bit0=1 to reset watermarks | 1 byte (1) |
| 0xB2 | `GET_USB_ERROR_STATS` | R | 0 | **24 bytes** (all zero under TinyUSB) |
| 0xB3 | `RESET_USB_ERROR_STATS` | **write-as-read (0xC1)** | 0 | 1 byte (no-op) |

`BufferStatsPacket` (44 bytes):

| Offset | Type | Field |
|--------|------|-------|
| 0 | u8 | `num_spdif` (2 or 4) |
| 1 | u8 | `flags` (bit0=PDM active, bit1=audio streaming) |
| 2 | u16 | `sequence` (monotonic, wraps) |
| 4 | 8×4 | `spdif[4]` (`SpdifBufferStats`, 8 bytes each) |
| 36 | 8 | `pdm` (`PdmBufferStats`, 8 bytes) |

`SpdifBufferStats` (8 bytes): u8 consumer_free; u8 consumer_prepared; u8
consumer_playing; u8 consumer_fill_pct; u8 consumer_min_fill_pct; u8
consumer_max_fill_pct; u8[2] pad. `PdmBufferStats` (8 bytes): u8 dma_fill_pct; u8
dma_min; u8 dma_max; u8 ring_fill_pct; u8 ring_min; u8 ring_max; u8[2] pad.
Instances beyond `num_spdif` are zeroed.

`GET_USB_ERROR_STATS` returns a 24-byte struct of six u32 counters
(total, crc, bitstuff, rx_overflow, rx_timeout, data_seq); they are all zero on
current firmware (TinyUSB does not expose them).

---

## 15. Presets and persistence

### 15.1 Preset slot operations (`PRESET_SLOTS` = 10, slots 0..9)

| ID | Name | Dir | wValue | Payload / Response | Notes |
|----|------|-----|--------|--------------------|-------|
| 0x90 | `PRESET_SAVE` | write-as-read | slot | 1 byte `PRESET_*` | Deferred flash write |
| 0x91 | `PRESET_LOAD` | write-as-read | slot | 1 byte `PRESET_*` | Deferred; poll `GET_ACTIVE` to confirm |
| 0x92 | `PRESET_DELETE` | write-as-read | slot | 1 byte `PRESET_*` | Deferred |
| 0x93 | `PRESET_GET_NAME` | R | slot | **32 bytes** (NUL-padded) | - |
| 0x94 | `PRESET_SET_NAME` | W (0x41) | slot | payload 1..32 bytes | Deferred flash write |
| 0x95 | `PRESET_GET_DIR` | R | 0 | **7 bytes** (see 15.2) | - |
| 0x96 | `PRESET_SET_STARTUP` | W (0x41) | 0 | payload **2 bytes**: `mode`, `slot` | mode 0=specified, 1=last active |
| 0x97 | `PRESET_GET_STARTUP` | R | 0 | **3 bytes**: mode, default_slot, last_active | - |
| 0x9A | `PRESET_GET_ACTIVE` | R | 0 | 1 byte (active slot 0..9) | - |
| 0x9B | `SET_CHANNEL_NAME` | W (0x41) | channel index | payload 1..32 bytes | - |
| 0x9C | `GET_CHANNEL_NAME` | R | channel index | **32 bytes** | - |

A preset is always active (0..9); there is no "no preset" state. Loading an
unconfigured slot applies factory defaults.

### 15.2 Directory summary (`PRESET_GET_DIR`, 7 bytes)

| Offset | Type | Field |
|--------|------|-------|
| 0 | u16 | `slot_occupied` bitmask (LE) |
| 2 | u8 | `startup_mode` (0=specified, 1=last active) |
| 3 | u8 | `default_slot` |
| 4 | u8 | `last_active_slot` |
| 5 | u8 | `output_config_mode` (0=independent, 1=with-preset) |
| 6 | u8 | `master_volume_mode` (0=independent, 1=per-preset) |

### 15.3 Persistence-mode and device-global save commands

| ID | Name | Dir | wValue | Payload / Response | Notes |
|----|------|-----|--------|--------------------|-------|
| 0x98 | `SET_OUTPUT_CONFIG_MODE` | W (0x41) | 0 | 1 byte (0/1) | 0=IO config device-global, 1=travels with preset |
| 0x99 | `GET_OUTPUT_CONFIG_MODE` | R | 0 | 1 byte | - |
| 0x52 | `SAVE_OUTPUT_CONFIG` | write-as-read | 0 | 1 byte `PRESET_OK` | Persist live IO config to device-global block (independent mode) |
| 0xD4 | `SET_MASTER_VOLUME_MODE` | W (0x41) | 0 | 1 byte (0/1) | 0=independent, 1=per-preset |
| 0xD5 | `GET_MASTER_VOLUME_MODE` | R | 0 | 1 byte | - |
| 0xD6 | `SAVE_MASTER_VOLUME` | write-as-read | 0 | 1 byte `PRESET_OK` | Persist live master volume to device-global field |
| 0xD7 | `GET_SAVED_MASTER_VOLUME` | R | 0 | f32 dB | Reads the device-global stored value |
| 0x51 | `SAVE_PARAMS` (legacy) | write-as-read | 0 | 1 byte `FLASH_OK` | Deferred; legacy "save current state" |
| 0x53 | `FACTORY_RESET` | write-as-read | 0 | 1 byte `FLASH_OK` | Deferred; resets live state, keeps active slot |

"Independent" (device-global) settings do not travel with presets: master volume
mode 0 and output-config mode 0 store their values in the directory sector and
apply at boot and on factory reset, untouched by preset load.

---

## 16. DAC hardware mute (board-level, handle with care)

| ID | Name | Dir | wValue | Payload / Response | Notes |
|----|------|-----|--------|--------------------|-------|
| 0xEA | `SET_DAC_HW_MUTE_CONFIG` | W (0x41) | 0 | **16-byte** `DacHwMuteConfig` | Deferred; validation/pin-claim/flash all run later |
| 0xEB | `GET_DAC_HW_MUTE_CONFIG` | R | 0 | **16 bytes** `DacHwMuteConfig` | - |
| 0xEC | `TEST_DAC_HW_MUTE` | write-as-read | 0 | 1 byte status | Pulses the mute pin ~1 s so an installer can confirm by ear |

`DacHwMuteConfig` (16 bytes):

| Offset | Type | Field |
|--------|------|-------|
| 0 | u8 | `enabled` (0/1) |
| 1 | u8 | `active_low` (1 = assert LOW to mute) |
| 2 | u8 | `pin` (GPIO; 0xFF = none) |
| 3 | u8 | reserved0 |
| 4 | u16 | `hold_ms` (1..500 when enabled) |
| 6 | u16 | `release_ms` (0..500) |
| 8 | u8[8] | reserved |

The SET return status is "accepted" only; **validation happens in the deferred
handler and a bad config is silently dropped**. Always read back with `0xEB` and
compare to confirm it applied.

---

## 17. System

| ID | Name | Dir | wValue | Response | Notes |
|----|------|-----|--------|----------|-------|
| 0xF0 | `ENTER_BOOTLOADER` | write-as-read (0xC1) | 0 | 1 byte (1) then **device reboots to USB bootloader** | Disconnect/re-enumerate as RPI-RP2 |

After the 1-byte ACK the firmware waits ~100 ms and resets into the UF2
bootloader. The bridge will lose the device until new firmware is flashed and it
re-enumerates as the DSPi again. Gate this behind explicit user confirmation.

### 17.1 External control interfaces (UART / I2C target)

The firmware can also be driven over an external UART or I2C target interface at
full parity with USB. These five commands configure and inspect those transports.
The two SET commands (`0xF5`, `0xF7`) are **USB only**: when issued over the UART
or I2C transport itself they are refused with a `BLOCKED` status, so an external
controller can never reconfigure or lock itself out of its own link. The full
transport wire protocol is in `Documentation/Features/control_interfaces_spec.md`.

| ID | Name | Dir | wValue | Payload / Response | Notes |
|----|------|-----|--------|--------------------|-------|
| 0xF5 | `SET_UART_CONFIG` | W (0x41) | 0 | payload 8-byte `UartCtrlConfig` | **USB only.** Validates + applies deferred; persists on success; returns `PIN_CONFIG_*` (0=ok, 1=bad pin, 2=pin in use, 5=bad baud) |
| 0xF6 | `GET_UART_CONFIG` | R | 0 | resp 8 bytes | Live `UartCtrlConfig` |
| 0xF7 | `SET_I2C_CONFIG` | W (0x41) | 0 | payload 8-byte `I2cCtrlConfig` | **USB only.** As 0xF5; status 5 = bad I2C address (valid 0x08..0x77) |
| 0xF8 | `GET_I2C_CONFIG` | R | 0 | resp 8 bytes | Live `I2cCtrlConfig` |
| 0xF9 | `GET_CTRL_IFACE_STATUS` | R | 0 | resp 8 bytes | `CtrlIfaceStatus`: `[uart_last_status, uart_live, i2c_last_status, i2c_live, proto_version(=1), 0,0,0]` |

`UartCtrlConfig` (8 bytes, LE): `[enabled(0/1), tx_pin, rx_pin, notify_enable(0/1), baud(u32)]`.
Defaults: disabled, TX GPIO 16, RX GPIO 17, notifications off, baud 115200 (range 9600..1000000).
`notify_enable` claims the byte that was formerly reserved: set it to 1 to have the
device push device-initiated notification frames (type `0x40`, carrying the verbatim
v2 notify packet) over the UART link; 0 (the default) leaves the link poll-only. All
prior stored/wired configs carry 0 there, so the default is off with no wire or
directory-version change. Like all interface config it is USB-only to set. See
section 18 and `Documentation/Features/control_interfaces_spec.md` (3.6).

`I2cCtrlConfig` (8 bytes, LE): `[enabled(0/1), sda_pin, scl_pin, address, reserved[4]]`.
Defaults: disabled, SDA GPIO 18, SCL GPIO 19, address 0x42 (range 0x08..0x77).

Both interfaces ship disabled; configuration is device-level, persists across
reboots, and survives a factory reset. `uart_live` / `i2c_live` in the status
readback reflect whether the interface actually came up: a stored config whose
pins collide with the current output wiring at boot stays down even though its
`enabled` byte is 1.

---

## 18. Asynchronous notifications (optional)

The device pushes change/event notifications on a **bulk IN endpoint `0x83`** on
the vendor interface (interface 2), 64-byte max packet. A USB host that wants live
push updates reads this endpoint; a bridge can either poll the readback commands
instead, or forward these packets. Packets are self-describing by their first
bytes:

| First bytes | Event | Layout |
|-------------|-------|--------|
| `0x01,0,0,0,...` | v1 master volume (legacy) | `[0x01, 0,0,0, f32 db_LE]` (8 bytes) |
| `0x02, 0x02, ...` | PARAM_CHANGED | `[ver=2, 0x02, flags, seq, off_LE(2), size_LE(2), src, 0,0,0, value...]` |
| `0x02, 0x03, ...` | BULK_INVALIDATED (re-read all) | `[ver=2, 0x03, flags, seq, src, 0,0,0]` |
| `0x02, 0x04, ...` | PRESET_LOADED | `[ver=2, 0x04, flags, seq, slot, 0,0,0]` |

For `PARAM_CHANGED`, `off` is the **byte offset into `WireBulkParams`** of the
changed field and `size` is its length; `value` is the new little-endian bytes.
This lets a host dispatch on offset rather than a per-command switch. `src` is the
origin (1=host SET, 2=bulk SET, 3=preset, 4=factory, 5=GPIO, 6=internal, 7=UAC1).
On `BULK_INVALIDATED` or `PRESET_LOADED`, re-read `REQ_GET_ALL_PARAMS`.

The same v2 packets can also be delivered over the **UART control transport** as
type-`0x40` frames when `UartCtrlConfig.notify_enable` is set (see section 17 and
`control_interfaces_spec.md` 3.6); the v1 legacy master-volume packet is USB-only.
A gap in the packet `seq` byte means events were dropped for that consumer; recover
with `REQ_GET_ALL_PARAMS`. The I2C target is poll-only (it cannot initiate a
transfer, so it has no notification channel).

---

## 19. Write -> readback map (for state confirmation)

Every settable parameter has a matching read. Use this to verify a write landed
(especially after the deferred busy window).

| Write | Readback |
|-------|----------|
| 0x42 SET_EQ_PARAM | 0x43 GET_EQ_PARAM (per scalar) or 0xA0 bulk |
| 0xD8 SET_BAND_BYPASS | 0xD9 GET_BAND_BYPASS |
| 0x44 / 0xD0 SET_PREAMP(_CH) | 0x45 / 0xD1 GET_PREAMP(_CH) |
| 0xD2 SET_MASTER_VOLUME | 0xD3 GET_MASTER_VOLUME |
| 0xDA SET_USER_VOLUME | 0xDB GET_USER_VOLUME |
| 0xDC SET_USER_MUTE | 0xDD GET_USER_MUTE |
| 0x46 SET_BYPASS | 0x47 GET_BYPASS |
| 0x54 / 0x56 SET_CHANNEL_GAIN/MUTE | 0x55 / 0x57 GET_CHANNEL_GAIN/MUTE |
| 0x48 SET_DELAY | 0x49 GET_DELAY |
| 0x58/0x5A/0x5C loudness | 0x59/0x5B/0x5D |
| 0xFA SET_LOUDNESS_MASK | 0xFB GET_LOUDNESS_MASK |
| 0x5E..0x66 crossfeed | 0x5F..0x67 |
| 0xB4..0xBE leveller | 0xB5..0xBF |
| 0x70 SET_MATRIX_ROUTE | 0x71 GET_MATRIX_ROUTE |
| 0x72/0x74/0x76/0x78 per-output | 0x73/0x75/0x77/0x79 |
| 0x7C SET_OUTPUT_PIN | 0x7D GET_OUTPUT_PIN |
| 0xC0 SET_OUTPUT_TYPE | 0xC1 GET_OUTPUT_TYPE |
| 0xC2/0xC4/0xC6/0xC8 I2S/MCK | 0xC3/0xC5/0xC7/0xC9 |
| 0xE0 SET_INPUT_SOURCE | 0xE1 GET_INPUT_SOURCE |
| 0xE4 SET_SPDIF_RX_PIN | 0xE5 GET_SPDIF_RX_PIN |
| 0xF1 SET_I2S_RX_PIN | 0xF2 GET_I2S_RX_PIN |
| 0xED SET_INPUT_RATE | 0xEE GET_INPUT_RATE |
| 0xE6 SET_LG_SOUND_SYNC_ENABLE | 0xE7 GET_LG_SOUND_SYNC_ENABLE |
| 0xEA SET_DAC_HW_MUTE_CONFIG | 0xEB GET_DAC_HW_MUTE_CONFIG |
| 0xF5 SET_UART_CONFIG (USB only) | 0xF6 GET_UART_CONFIG |
| 0xF7 SET_I2C_CONFIG (USB only) | 0xF8 GET_I2C_CONFIG |
| 0x9B SET_CHANNEL_NAME | 0x9C GET_CHANNEL_NAME |
| 0x94 PRESET_SET_NAME | 0x93 PRESET_GET_NAME |
| 0x96 PRESET_SET_STARTUP | 0x97 PRESET_GET_STARTUP |
| 0x98 SET_OUTPUT_CONFIG_MODE | 0x99 GET_OUTPUT_CONFIG_MODE |
| 0xD4 SET_MASTER_VOLUME_MODE | 0xD5 GET_MASTER_VOLUME_MODE |
| 0xD6 SAVE_MASTER_VOLUME | 0xD7 GET_SAVED_MASTER_VOLUME |
| 0x91 PRESET_LOAD | 0x9A PRESET_GET_ACTIVE |
| 0xA1 SET_ALL_PARAMS | 0xA0 GET_ALL_PARAMS |

---

## 20. Hardware / pin commands to treat carefully

These commands change physical I/O, claim GPIOs, write flash, or restart the
audio pipeline. A bridge should rate-limit them, gate them behind explicit user
intent, and **always read back to confirm**. They can return a `PIN_CONFIG_*`
rejection or trigger a busy window / re-enumeration.

| ID | Command | Why careful |
|----|---------|-------------|
| 0x7C | `SET_OUTPUT_PIN` | Moves an output GPIO; deferred, muted pipeline reset; PDM pin requires output disabled first |
| 0xC0 | `SET_OUTPUT_TYPE` | S/PDIF<->I2S switch; heap alloc + pipeline reset; busy window |
| 0xC2 | `SET_I2S_BCK_PIN` | Moves BCK/LRCLK pair; rejected while any slot is I2S |
| 0xC4/0xC6/0xC8 | MCK enable/pin/multiplier | MCK pin must be CLK_GPOUTn-capable; wrong pin returns INVALID_PIN |
| 0xE4 | `SET_SPDIF_RX_PIN` | Hot-swaps RX input; stops/restarts RX library; brief silence |
| 0xF1 | `SET_I2S_RX_PIN` | Same hot-swap behavior as 0xE4 |
| 0xED | `SET_INPUT_RATE` | Triggers a rate change / pipeline reset when I2S is active input |
| 0xE0 | `SET_INPUT_SOURCE` | Full pipeline reset; busy window; report active state via 0xE1 |
| 0xEA | `SET_DAC_HW_MUTE_CONFIG` | Claims a GPIO; silent validation failure; **must read back 0xEB** |
| 0xEC | `TEST_DAC_HW_MUTE` | Audibly mutes for ~1 s |
| 0x90/0x91/0x92 | preset save/load/delete | ~45 ms flash blackout per sector; deferred |
| 0x51/0x52/0x53/0xD6 | save/factory-reset/persist | Flash write; deferred busy window |
| 0xA1 | `SET_ALL_PARAMS` | Rewrites all state; muted pipeline reset; large transfer |
| 0xF0 | `ENTER_BOOTLOADER` | **Reboots out of the application**; device disappears |

> Inter-slot phase alignment is a hard product invariant. The firmware preserves
> sample-level alignment across all of these operations by routing pin/type/source
> changes through a synchronized, muted pipeline reset. A bridge must not attempt
> to "optimize" by issuing partial or out-of-band reconfiguration; always use these
> opcodes as defined.

**Section 22 walks through the exact end-to-end workings of the most consequential
of these operations** (bulk apply, the device-global saves, preset writes, pin
writes, output-bus reconfiguration, factory reset, the persistence model, and the
bootloader jump). Read it before implementing any of them in a bridge.

---

## 21. Complete opcode list and internal handler map

Every `REQ_*` vendor command (121 total) with the internal handler it maps to. The
**Internal mapping** column names the firmware function(s) and/or globals each command
touches. Many writes are *deferred*: the handler only sets a `*_pending` flag (or a
change bitmask) and `main.c`'s loop performs the real operation later, sometimes after
the pipeline-reset settle window (see section 1.3). Such rows read
`<pending_flag> -> <main.c operation>`. The deferred flags are summarized in section 21.1.

`Dir`: `W` = OUT/write (has a data stage); `R` = IN/read; `W-as-R` = an action or write
with no OUT data, issued as an IN transfer (returns a status/echo byte).

| Hex | Name | Dir | Internal mapping |
|-----|------|-----|------------------|
| 0x42 | SET_EQ_PARAM | W | copy payload to `pending_packet`, validate ch/band; `eq_update_pending` -> write `filter_recipes`/`xover_recipes` + `dsp_compute_coefficients()` / `xover_design_filter()` |
| 0x43 | GET_EQ_PARAM | R | read `filter_recipes[ch][band]` / `xover_recipes[ch][band-20]`; wValue packs ch/band/param(0..4); returns 4-byte scalar |
| 0x44 | SET_PREAMP | W | legacy: `update_preamp(ch, db)` for all input channels (dB->linear->Q28) |
| 0x45 | GET_PREAMP | R | returns `global_preamp_db[0]` |
| 0x46 | SET_BYPASS | W | writes `bypass_master_eq` |
| 0x47 | GET_BYPASS | R | returns `bypass_master_eq` |
| 0x48 | SET_DELAY | W | writes `channel_delays_ms[ch]`; immediate `dsp_update_delay_samples()` |
| 0x49 | GET_DELAY | R | returns `channel_delays_ms[ch]` |
| 0x50 | GET_STATUS | R | wValue selects scalar/combined meters: `global_status` peaks/CPU/clip, PDM/SPDIF counters, `clock_get_hz`, vreg, `audio_state.freq`, temperature, DMA/ring stats |
| 0x51 | SAVE_PARAMS | W-as-R | legacy: `save_params_pending` -> `flash_save_params()` (flash bracket; not gated) |
| 0x52 | SAVE_OUTPUT_CONFIG | W-as-R | `flash_save_output_config_pending` -> `preset_save_output_config()` (flash; not gated) |
| 0x53 | FACTORY_RESET | W-as-R | `factory_reset_pending` -> `flash_factory_reset()` + `dsp_recalculate_all_filters()` + delay/Core1/type resets (gated) |
| 0x54 | SET_CHANNEL_GAIN | W | legacy (ch<3): writes `channel_gain_db/_mul/_linear[ch]` |
| 0x55 | GET_CHANNEL_GAIN | R | returns `channel_gain_db[ch]` |
| 0x56 | SET_CHANNEL_MUTE | W | writes `channel_mute[ch]` (ch<3) |
| 0x57 | GET_CHANNEL_MUTE | R | returns `channel_mute[ch]` |
| 0x58 | SET_LOUDNESS | W | writes `loudness_enabled`; reselects `current_loudness_coeffs` immediately |
| 0x59 | GET_LOUDNESS | R | returns `loudness_enabled` |
| 0x5A | SET_LOUDNESS_REF | W | clamps `loudness_ref_spl`; `loudness_recompute_pending` -> `loudness_recompute_table()` |
| 0x5B | GET_LOUDNESS_REF | R | returns `loudness_ref_spl` |
| 0x5C | SET_LOUDNESS_INTENSITY | W | clamps `loudness_intensity_pct`; `loudness_recompute_pending` -> `loudness_recompute_table()` |
| 0x5D | GET_LOUDNESS_INTENSITY | R | returns `loudness_intensity_pct` |
| 0x5E | SET_CROSSFEED | W | writes `crossfeed_config.enabled`; `crossfeed_update_pending` -> `crossfeed_compute_coefficients()` |
| 0x5F | GET_CROSSFEED | R | returns `crossfeed_config.enabled` |
| 0x60 | SET_CROSSFEED_PRESET | W | writes `crossfeed_config.preset`; `crossfeed_update_pending` -> compute |
| 0x61 | GET_CROSSFEED_PRESET | R | returns `crossfeed_config.preset` |
| 0x62 | SET_CROSSFEED_FREQ | W | writes `crossfeed_config.custom_fc`; if preset==CUSTOM `crossfeed_update_pending` -> compute |
| 0x63 | GET_CROSSFEED_FREQ | R | returns `crossfeed_config.custom_fc` |
| 0x64 | SET_CROSSFEED_FEED | W | writes `crossfeed_config.custom_feed_db`; if preset==CUSTOM `crossfeed_update_pending` -> compute |
| 0x65 | GET_CROSSFEED_FEED | R | returns `crossfeed_config.custom_feed_db` |
| 0x66 | SET_CROSSFEED_ITD | W | writes `crossfeed_config.itd_enabled`; `crossfeed_update_pending` -> compute |
| 0x67 | GET_CROSSFEED_ITD | R | returns `crossfeed_config.itd_enabled` |
| 0x70 | SET_MATRIX_ROUTE | W | writes `matrix_mixer.crosspoints[in][out]` (enabled/phase/gain) immediately |
| 0x71 | GET_MATRIX_ROUTE | R | wValue=in<<8\|out; returns `MatrixRoutePacket` from crosspoints |
| 0x72 | SET_OUTPUT_ENABLE | W | PDM/EQ-worker interlock; writes `matrix_mixer.outputs[out].enabled`; `derive_core1_mode()`, `pdm_set_enabled()`, `__sev()` immediately |
| 0x73 | GET_OUTPUT_ENABLE | R | returns `matrix_mixer.outputs[out].enabled` |
| 0x74 | SET_OUTPUT_GAIN | W | writes `matrix_mixer.outputs[out].gain_db/gain_linear` immediately |
| 0x75 | GET_OUTPUT_GAIN | R | returns `matrix_mixer.outputs[out].gain_db` |
| 0x76 | SET_OUTPUT_MUTE | W | writes `matrix_mixer.outputs[out].mute` immediately |
| 0x77 | GET_OUTPUT_MUTE | R | returns `matrix_mixer.outputs[out].mute` |
| 0x78 | SET_OUTPUT_DELAY | W | writes `outputs[out].delay_ms` + `channel_delays_ms[CH_OUT_1+out]`; `dsp_update_delay_samples()` |
| 0x79 | GET_OUTPUT_DELAY | R | returns `matrix_mixer.outputs[out].delay_ms` |
| 0x7A | GET_CORE1_MODE | R | returns `core1_mode` (Core1Mode enum) |
| 0x7B | GET_CORE1_CONFLICT | R | returns 1 if enabling wValue output would violate PDM-vs-EQ-worker exclusion |
| 0x7C | SET_OUTPUT_PIN | W-as-R | SPDIF/I2S: writes `output_pins[idx]` + `output_pin_change_mask` -> `process_pin_changes()` (gated); PDM: immediate `pdm_change_pin()` |
| 0x7D | GET_OUTPUT_PIN | R | returns `output_pins[idx]` |
| 0x7E | GET_SERIAL | R | returns 16 bytes of the serial string |
| 0x7F | GET_PLATFORM | R | returns {platform id, FW major, legacy minor.patch nibbles, NUM_OUTPUT_CHANNELS, FW minor, FW patch} |
| 0x80 | GET_BUILD_INFO | R | returns 64-byte build stamp: git describe [0..47] + build date [48..59]; informational only |
| 0x83 | CLEAR_CLIPS | W-as-R | read-and-clear: returns `global_status.clip_flags`, then zeroes it |
| 0x90 | PRESET_SAVE | W-as-R | `pending_preset_save_slot` + `preset_save_pending` -> `preset_save(slot)` (flash; not gated) |
| 0x91 | PRESET_LOAD | W-as-R | `pending_preset_load_slot` + `preset_load_pending` -> `preset_load(slot)` + pipeline reset / type switches (gated) |
| 0x92 | PRESET_DELETE | W-as-R | `preset_delete_mask` -> `preset_delete(slot)` per bit (flash; not gated) |
| 0x93 | PRESET_GET_NAME | R | `preset_get_name(slot)` -> 32-byte name |
| 0x94 | PRESET_SET_NAME | W | `flash_set_name_buf/_slot` + `flash_set_name_pending` -> `preset_set_name()` |
| 0x95 | PRESET_GET_DIR | R | `preset_get_directory()` -> 7 bytes (occupied, startup, default, last_active, oc_mode, mv_mode) |
| 0x96 | PRESET_SET_STARTUP | W | `flash_set_startup_mode/_slot` + `flash_set_startup_pending` -> `preset_set_startup()` |
| 0x97 | PRESET_GET_STARTUP | R | `preset_get_directory()` -> 3 bytes (startup_mode, default, last_active) |
| 0x98 | SET_OUTPUT_CONFIG_MODE | W | `flash_set_output_config_mode_val` + pending -> `preset_set_output_config_mode()` |
| 0x99 | GET_OUTPUT_CONFIG_MODE | R | `preset_get_directory()` -> oc_mode byte |
| 0x9A | PRESET_GET_ACTIVE | R | returns `preset_get_active()` |
| 0x9B | SET_CHANNEL_NAME | W | wValue=ch; copies payload to `channel_names[ch]` (RAM; persisted on preset save) |
| 0x9C | GET_CHANNEL_NAME | R | returns `channel_names[ch]` (32 bytes) |
| 0xA0 | GET_ALL_PARAMS | R | `bulk_params_collect()` -> chunked `WireBulkParams` |
| 0xA1 | SET_ALL_PARAMS | W | payload -> `bulk_param_buf` (chunked); `bulk_params_pending` -> `bulk_params_apply()` + `dsp_recalculate_all_filters()` + reset (gated) |
| 0xA2 | GET_ALL_PARAMS_CHUNK | R | wValue = byte offset; offset 0 snapshots under the bulk lock; USB-only (WinUSB 4 KB cap; see `Features/bulk_params_chunking.md`) |
| 0xA3 | SET_ALL_PARAMS_CHUNK | W | wValue = byte offset, sequential from 0 into `bulk_param_buf`; final byte sets `bulk_params_pending`; USB-only |
| 0xA4 | SIGGEN_SET_CONFIG | W | payload = `SiggenConfig` (36 B) -> `siggen_stage_config()` (validate + stage; restarts if running, never auto-starts); STALL on reject. See section 23 |
| 0xA5 | SIGGEN_GET_CONFIG | R | returns applied `SiggenConfig` (36 B) |
| 0xA6 | SIGGEN_CONTROL | W-as-R | wValue = `SIGGEN_CTL_*` (0 stop, 1 start, 2 stop-now) -> `siggen_control()`; returns 1-byte ack; STALL on reject |
| 0xA7 | SIGGEN_GET_STATUS | R | returns `SiggenStatus` (16 B) |
| 0xA8 | SIGGEN_GET_CAPS | R | wValue = 0xFFFF -> `SiggenCapsHeader` (8 B); wValue = type index -> `SiggenTypeDesc` (62 B) |
| 0xB0 | GET_BUFFER_STATS | R | `BufferStatsPacket` from consumer stats / watermarks / PDM fill |
| 0xB1 | RESET_BUFFER_STATS | W-as-R | `reset_buffer_watermarks()` if wValue&1 |
| 0xB2 | GET_USB_ERROR_STATS | R | returns zeroed `UsbErrorStatsPacket` (TinyUSB exposes no counters) |
| 0xB3 | RESET_USB_ERROR_STATS | W-as-R | no-op |
| 0xB4 | SET_LEVELLER_ENABLE | W | writes `leveller_config.enabled`; `leveller_update_pending` + `leveller_reset_pending` -> `leveller_compute_coefficients()` + `leveller_reset_state()` |
| 0xB5 | GET_LEVELLER_ENABLE | R | returns `leveller_config.enabled` |
| 0xB6 | SET_LEVELLER_AMOUNT | W | writes `leveller_config.amount`; `leveller_update_pending` -> compute |
| 0xB7 | GET_LEVELLER_AMOUNT | R | returns `leveller_config.amount` |
| 0xB8 | SET_LEVELLER_SPEED | W | writes `leveller_config.speed`; `leveller_update_pending` -> compute |
| 0xB9 | GET_LEVELLER_SPEED | R | returns `leveller_config.speed` |
| 0xBA | SET_LEVELLER_MAX_GAIN | W | writes `leveller_config.max_gain_db`; `leveller_update_pending` -> compute |
| 0xBB | GET_LEVELLER_MAX_GAIN | R | returns `leveller_config.max_gain_db` |
| 0xBC | SET_LEVELLER_LOOKAHEAD | W | writes `leveller_config.lookahead`; `leveller_update_pending` + `leveller_reset_pending` -> compute + state reset |
| 0xBD | GET_LEVELLER_LOOKAHEAD | R | returns `leveller_config.lookahead` |
| 0xBE | SET_LEVELLER_GATE | W | writes `leveller_config.gate_threshold_db`; `leveller_update_pending` -> compute |
| 0xBF | GET_LEVELLER_GATE | R | returns `leveller_config.gate_threshold_db` |
| 0xC0 | SET_OUTPUT_TYPE | W-as-R | `pending_output_types[slot]` + `output_type_change_mask` -> `process_type_switches()` (SPDIF<->I2S, gated) |
| 0xC1 | GET_OUTPUT_TYPE | R | returns `output_types[slot]` |
| 0xC2 | SET_I2S_BCK_PIN | W-as-R | writes `i2s_bck_pin`; if I2S input active `i2s_input_restart_pending` -> I2S restart |
| 0xC3 | GET_I2S_BCK_PIN | R | returns `i2s_bck_pin` |
| 0xC4 | SET_MCK_ENABLE | W-as-R | immediate `audio_i2s_mck_update_frequency()` + `audio_i2s_mck_set_enabled()`; writes `i2s_mck_enabled` |
| 0xC5 | GET_MCK_ENABLE | R | returns `i2s_mck_enabled` |
| 0xC6 | SET_MCK_PIN | W-as-R | immediate `audio_i2s_mck_change_pin()`; writes `i2s_mck_pin` |
| 0xC7 | GET_MCK_PIN | R | returns `i2s_mck_pin` |
| 0xC8 | SET_MCK_MULTIPLIER | W-as-R | writes `i2s_mck_multiplier`; if enabled `audio_i2s_mck_update_frequency()` |
| 0xC9 | GET_MCK_MULTIPLIER | R | returns `mck_encode(i2s_mck_multiplier)` (0/1) |
| 0xD0 | SET_PREAMP_CH | W | wValue=input ch; `update_preamp(ch, db)` immediately |
| 0xD1 | GET_PREAMP_CH | R | returns `global_preamp_db[ch]` |
| 0xD2 | SET_MASTER_VOLUME | W | `update_master_volume(db)` immediately (clamp/mute sentinel; sets `vol_mul_master`) |
| 0xD3 | GET_MASTER_VOLUME | R | returns `master_volume_db` |
| 0xD4 | SET_MASTER_VOLUME_MODE | W | `flash_set_master_volume_mode_val` + pending -> `preset_set_master_volume_mode()` |
| 0xD5 | GET_MASTER_VOLUME_MODE | R | `preset_get_directory()` -> mv_mode byte |
| 0xD6 | SAVE_MASTER_VOLUME | W-as-R | `flash_save_master_volume_pending` -> `preset_save_master_volume()` |
| 0xD7 | GET_SAVED_MASTER_VOLUME | R | returns `preset_get_saved_master_volume()` (directory field) |
| 0xD8 | SET_BAND_BYPASS | W | load recipe, set bypass -> `pending_packet` + `eq_update_pending` -> recipe write + coeff recompute |
| 0xD9 | GET_BAND_BYPASS | R | returns recipe `.bypass` byte (0/1) |
| 0xDA | SET_USER_VOLUME | W | `update_user_volume(db)` immediately (writes `audio_state.volume`, drives `vol_mul` + loudness) |
| 0xDB | GET_USER_VOLUME | R | returns `audio_state.volume / 256` (dB) |
| 0xDC | SET_USER_MUTE | W | writes `user_mute` (OR'd into pipeline) immediately |
| 0xDD | GET_USER_MUTE | R | returns `user_mute` |
| 0xE0 | SET_INPUT_SOURCE | W | `pending_input_source` + `input_source_change_pending` -> stop old / start new source + rate/MCK (gated) |
| 0xE1 | GET_INPUT_SOURCE | R | returns `active_input_source` |
| 0xE2 | GET_SPDIF_RX_STATUS | R | `spdif_input_get_status()` -> 16-byte `SpdifRxStatusPacket` |
| 0xE3 | GET_SPDIF_RX_CH_STATUS | R | `spdif_input_get_channel_status()` -> 24-byte IEC 60958 channel status |
| 0xE4 | SET_SPDIF_RX_PIN | W-as-R | writes `spdif_rx_pin`; if SPDIF active `spdif_rx_pin_change_pending` -> stop/restart RX on new pin (not gated) |
| 0xE5 | GET_SPDIF_RX_PIN | R | returns `spdif_rx_pin` |
| 0xE6 | SET_LG_SOUND_SYNC_ENABLE | W | `lg_sound_sync_set_enabled(bool)` immediately |
| 0xE7 | GET_LG_SOUND_SYNC_ENABLE | R | returns `lg_sound_sync_get_enabled()` |
| 0xE8 | GET_LG_SOUND_SYNC_STATUS | R | `lg_sound_sync_get_status()` -> 16-byte `LgSoundSyncStatus` |
| 0xEA | SET_DAC_HW_MUTE_CONFIG | W | `flash_set_dac_hw_mute_val` + `flash_set_dac_hw_mute_pending` -> `dac_hw_mute_set_config()` |
| 0xEB | GET_DAC_HW_MUTE_CONFIG | R | `dac_hw_mute_get_config()` -> 16-byte `DacHwMuteConfig` |
| 0xEC | TEST_DAC_HW_MUTE | W-as-R | if enabled: `dac_hw_mute_test_pending` -> `dac_hw_mute_test_start()` (~1s async pulse) |
| 0xED | SET_INPUT_RATE | W | validate -> `i2s_input_rate`; if I2S active and differs: `pending_rate` + `rate_change_pending` -> `perform_rate_change()` (gated) |
| 0xEE | GET_INPUT_RATE | R | returns 2x u32 {`audio_state.freq`, `i2s_input_rate`} |
| 0xF0 | ENTER_BOOTLOADER | W-as-R | returns 1, then `reset_usb_boot(0,0)` (never returns) |
| 0xF1 | SET_I2S_RX_PIN | W-as-R | writes `i2s_rx_pin`; if I2S active `i2s_rx_pin_change_pending` -> I2S stop/start on new pin (gated) |
| 0xF2 | GET_I2S_RX_PIN | R | returns `i2s_rx_pin` |
| 0xF5 | SET_UART_CONFIG | W | **USB only** (BLOCKED over UART/I2C); `ctrl_set_uart_pending` -> main loop `uart_ctrl_apply()`, persist directory on success; returns `PIN_CONFIG_*` |
| 0xF6 | GET_UART_CONFIG | R | returns persisted 8-byte `UartCtrlConfig` |
| 0xF7 | SET_I2C_CONFIG | W | **USB only** (BLOCKED over UART/I2C); `ctrl_set_i2c_pending` -> main loop `i2c_ctrl_apply()`, persist directory on success; returns `PIN_CONFIG_*` |
| 0xF8 | GET_I2C_CONFIG | R | returns persisted 8-byte `I2cCtrlConfig` |
| 0xF9 | GET_CTRL_IFACE_STATUS | R | returns 8-byte `CtrlIfaceStatus` (uart/i2c last_status + live flags, proto_version=1) |
| 0xFA | SET_LOUDNESS_MASK | W | writes `loudness_output_mask` (u16 LE, bit k = output k); no recompute; applies next packet |
| 0xFB | GET_LOUDNESS_MASK | R | returns `loudness_output_mask` (u16 LE) |

> The Microsoft OS 2.0 vendor code (`0x01`) is intercepted earlier in
> `tud_vendor_control_xfer_cb` for descriptor delivery and is not an application opcode.

### 21.1 Deferred pending-flag legend

Each flag below is set by a handler and consumed in `main.c`'s main loop. "Gated" means
the block runs only when `pipeline_reset_ready()` is true (the non-blocking DAC
hardware-mute settle hold has elapsed); "not gated" runs as soon as the loop reaches it.

| Pending flag | main.c operation | Gated |
|---|---|---|
| `eq_update_pending` | write `filter_recipes`/`xover_recipes` from `pending_packet` (Core 1 fence if EQ-worker output), then `dsp_compute_coefficients()` + `channel_bypassed` recompute (PEQ) or `xover_design_filter()` + `xover_update_channel_bypass()` (crossover) | no |
| `loudness_recompute_pending` | `loudness_recompute_table()` then re-key `current_loudness_coeffs` (also raised by rate change / factory reset) | no |
| `crossfeed_update_pending` | `crossfeed_apply_config()` (recompute into the inactive coeff buffer, publish `current_crossfeed_coeffs`; NULL = disabled) | no |
| `leveller_update_pending` (+ `leveller_reset_pending`) | `leveller_compute_coefficients()` (+ `leveller_reset_state()`); set `leveller_bypassed` | no |
| `rate_change_pending` (+ `pending_rate`) | drain ring, `perform_rate_change(rate)` (PIO dividers, coeffs, delays, feedback) | yes |
| `stream_restart_resync_pending` | drain/flush ring, `prepare_pipeline_reset()` + `complete_pipeline_reset()` to resync all output slots after USB alt 0->N | yes |
| `preset_load_pending` (+ `pending_preset_load_slot`) | drain, prepare reset, suspend RX, `preset_load(slot)`, MCK apply, type switches or `complete_pipeline_reset()`, restart RX | yes |
| `save_params_pending` | legacy `flash_save_params()` (flash bracket) | no |
| `preset_save_pending` (+ `pending_preset_save_slot`) | `preset_save(slot)` (flash bracket) | no |
| `preset_delete_mask` (bitmask) | `preset_delete(slot)` per bit; reset types if active slot deleted | no |
| `factory_reset_pending` | drain, prepare reset, suspend RX, `flash_factory_reset()`, `dsp_recalculate_all_filters()`, zero delay lines, Core 1 mode, type switches, restart RX | yes |
| `flash_set_name_pending` | `preset_set_name()` (light flash bracket) | no |
| `flash_set_startup_pending` | `preset_set_startup()` (light flash bracket) | no |
| `flash_set_output_config_mode_pending` | `preset_set_output_config_mode()` (light flash bracket) | no |
| `flash_save_output_config_pending` | `preset_save_output_config()` (light flash bracket) | no |
| `flash_set_master_volume_mode_pending` | `preset_set_master_volume_mode()` (light flash bracket) | no |
| `flash_save_master_volume_pending` | `preset_save_master_volume()` (light flash bracket) | no |
| `flash_set_dac_hw_mute_pending` | `dac_hw_mute_set_config()` (validate, pin claim, flash, notify) | no |
| `dac_hw_mute_test_pending` | `dac_hw_mute_test_start()` (async ~1s mute pulse, released by `dac_hw_mute_tick()`) | no |
| `output_type_change_mask` (bitmask, + `pending_output_types[]`) | `process_type_switches()` (SPDIF<->I2S teardown/setup; runs its own pipeline reset) | yes |
| `output_pin_change_mask` (bitmask) | `process_pin_changes()` (mute, repin, restart all slots in sync) | yes |
| `bulk_params_pending` | drain, prepare reset, suspend RX, `bulk_params_apply()`, `dsp_recalculate_all_filters()`, MCK apply, Core 1 mode, type switches, restart RX | yes |
| `input_source_change_pending` (+ `pending_input_source`) | drain, prepare reset, stop old source, regen channel names, set `active_input_source`, start new source | yes |
| `spdif_rx_pin_change_pending` | if SPDIF active: `spdif_input_stop()`, prepare reset, `spdif_input_start()` on new pin | no |
| `i2s_rx_pin_change_pending` / `i2s_input_restart_pending` | if I2S active: prepare reset, `i2s_input_stop()` + `i2s_input_start()` | yes |

"W-as-R" = write/action dispatched on the IN (read, `0xC1`) path; see Section 1.1.

---

## 22. Detailed workings: stateful, flash, and reconfiguration operations

The commands below either **write flash**, **rewrite a large amount of live
state**, **reconfigure physical I/O**, or **leave the application entirely**.
They are the ones a bridge is most likely to get wrong. Everything you need to
drive them correctly is collected here in one place. Read Section 22.1 first: the
three shared mechanisms it describes underlie every operation that follows.

### 22.1 The three shared mechanisms

**(a) The deferred pending-flag pattern.** Most of these handlers do almost
nothing at request time: they validate the shape of the request, set a `volatile`
`*_pending` flag (or a change bitmask), and return an "accepted" status
immediately. The real work runs later in `main.c`'s main loop. Consequences:

* The status byte you get back means "**request queued and shaped correctly**", not
  "applied". Later validation can still reject the value silently (DAC mute pin
  conflicts, GPIO already in use, bad output slot). Always follow a write that
  matters with its readback (Section 19) and compare.
* Between "accepted" and "applied" there is a **busy window**. If the deferred work
  writes flash or resets the pipeline it briefly disables the USB control IRQ, so a
  control transfer issued during that window can time out or STALL. Back off ~150 ms
  and retry, and poll `GET_PLATFORM` (0x7F) to confirm the device is responsive
  again (Section 1.3).
* Flags marked **"gated"** additionally wait for `pipeline_reset_ready()`: if a DAC
  hardware-mute settle hold is armed, the operation does not begin until that
  non-blocking hold has elapsed. "Not gated" flags run as soon as the loop reaches
  them.

**(b) The flash write bracket and blackout.** A flash write erases and programs one
**4 KB sector** at a time. Each sector write halts Core 1
(`multicore_lockout_start_blocking`) and masks all interrupts
(`save_and_disable_interrupts`) for the erase plus program, so the USB control
endpoint is dead for the duration. Budget **up to ~45 ms of blackout per sector**.
Commands that touch two sectors (a preset slot plus the directory) cost roughly
double. The persistent region is 12 sectors (48 KB) at the top of flash:

| Sector | Contents |
|--------|----------|
| 0 | **Directory** (`DSP2`): 10-bit slot-occupancy bitmask, 10 x 32-byte slot names, startup mode/slot, last-active slot, and the **device-global** fields (independent master volume, independent IO config, DAC hardware-mute config, mode flags) |
| 1 .. 10 | **Preset slots 0 .. 9** (`DSP3`): one full DSP-state body per slot |
| 11 | **Legacy** (`DSP1`): original single-sector format, auto-migrated to slot 0 on first boot |

So slot **names, startup policy, occupancy, and every device-global setting live in
sector 0**; only the per-slot DSP bodies live in sectors 1..10. A rename or a
startup change is a light one-sector (directory) write; a preset save is a
two-sector write (slot body plus directory update).

**(c) The muted, synchronized pipeline reset (the slot-alignment guarantee).**
Any operation that could disturb output timing is bracketed by
`prepare_pipeline_reset(PRESET_MUTE_SAMPLES)` (soft-mute the output and fence Core 1;
`PRESET_MUTE_SAMPLES` = 256, about 5 ms at 48 kHz) and, at the end,
`complete_pipeline_reset()` (or `process_type_switches()` / `process_pin_changes()`),
which restarts **every** output slot in lockstep. This is how the firmware keeps all
four S/PDIF instances, the I2S slots, and PDM sample-aligned across the change. Inter-
slot phase alignment is an inviolable product invariant; a bridge must never try to
reconfigure a subset of slots out of band to avoid the mute.

### 22.2 `0xA1` SET_ALL_PARAMS (bulk apply of all live state)

* **Transport.** OUT write (`0x41`), `wValue=0`. Payload is a `WireBulkParams`
  packet (or a valid shorter prefix). The firmware accepts `wLength` in
  `[WIRE_BULK_PARAMS_MIN_SIZE, sizeof(WireBulkParams)]` (the V2 size up to the current
  3664-byte V14) and receives it into `bulk_param_buf` via EP0 64-byte chunking. It is
  **version tolerant**: it honors only the sections present per `header.format_version`
  and `header.payload_length`, so an older host's shorter packet still applies cleanly.
* **Handler (immediate).** The full transfer lands in `bulk_param_buf`; on the control
  **status (ACK) stage** the handler sets `bulk_params_pending = true`. No state changes
  in IRQ context.
* **Deferred work (gated).** In the main loop: snapshot current `output_types`; drain the
  USB ring; `prepare_pipeline_reset()` (mute + Core 1 fence); suspend S/PDIF and I2S RX
  if active; `bulk_params_apply()` (pin/IO config is applied **only** in with-preset
  output-config mode, so a bulk push does not stomp device-global wiring in independent
  mode); `dsp_recalculate_all_filters()` + delays; re-sync MCK; re-derive Core 1 mode;
  then either `process_type_switches()` for any slot whose output type changed, or
  `complete_pipeline_reset()` to re-align all slots; finally restart RX.
* **Flash footprint.** **None.** `SET_ALL_PARAMS` rewrites **live RAM state only**; it does
  **not** persist anything. To make a bulk-applied state survive a power cycle you must
  separately save it (a preset save, or the device-global saves in 22.3). This is the most
  common misconception about this command.
* **Slot alignment.** Preserved via the muted synchronized reset (22.1c).
* **Confirm.** Re-read with `GET_ALL_PARAMS` (`0xA0`) after the busy window; compare
  `header.payload_length` and the fields you set.
* **Cautions.** Largest single write in the protocol; always brackets a mute of at least
  ~5 ms; can flip the physical output bus (S/PDIF <-> I2S) if the payload changes output
  types.

### 22.3 Device-global saves: `0xD6` SAVE_MASTER_VOLUME (and siblings)

These persist a **device-global** value into the **directory sector** (sector 0). Device-
global settings do **not** travel with presets; they are read at boot and on factory
reset and are untouched by preset load.

| ID | Command | Persists | Deferred flag | Gated |
|----|---------|----------|---------------|-------|
| 0xD6 | `SAVE_MASTER_VOLUME` | live master volume -> directory device-global field | `flash_save_master_volume_pending` | no |
| 0x52 | `SAVE_OUTPUT_CONFIG` | live physical IO config -> directory device-global block | `flash_save_output_config_pending` | no |
| 0x51 | `SAVE_PARAMS` (legacy) | legacy "save current state" | `save_params_pending` | no |

* **Transport.** All three are **write-as-read** (issue as `0xC1`, `wValue=0`); each returns a
  1-byte status (`PRESET_OK` / `FLASH_OK`) meaning "accepted".
* **Handler / deferred.** The handler only sets the pending flag; the main loop performs the
  directory-sector write (one sector, so one ~45 ms blackout; **not** gated, so it runs as
  soon as the loop reaches it).
* **Confirm.** For `0xD6`, read back `GET_SAVED_MASTER_VOLUME` (`0xD7`), which returns the
  **stored** device-global value (distinct from `GET_MASTER_VOLUME` `0xD3`, which returns the
  **live** ceiling set by `0xD2`).
* **Note on modes.** Whether a preset load overwrites the live master volume / IO config is
  governed by `master_volume_mode` (`0xD4/0xD5`) and `output_config_mode` (`0x98/0x99`). In
  independent mode (0) the device-global copy saved here is authoritative; in per-preset mode
  (1) the value travels inside each preset instead.

### 22.4 Preset save / load / delete / rename / startup writes

All slot bodies live in sectors 1..10; occupancy, names, startup policy, and last-active
live in the directory (sector 0). Every one of these is a flash write.

* **`0x90` PRESET_SAVE** (write-as-read, `wValue`=slot). Deferred (`preset_save_pending`,
  **not** gated). `preset_save()` collects live DSP state, engages the audio mute, writes the
  **slot sector**, then updates the **directory** (sets the occupied bit and last-active).
  That is a **two-sector write**; budget ~2x the single-sector blackout. Returns `PRESET_*`.
* **`0x91` PRESET_LOAD** (write-as-read, `wValue`=slot). Deferred (`preset_load_pending`,
  **gated**). This is a **read plus a full muted pipeline reset**, not a slot-body write: it
  drains, mutes, suspends RX, reads the slot (or applies factory defaults if the slot is
  unconfigured), applies to live state, recalculates filters/delays, **zeroes all delay lines**
  (so stale audio from the previous preset cannot bleed through), re-derives Core 1 mode,
  performs any needed output-type switches or a `complete_pipeline_reset()`, restarts RX, and
  finally does one **light directory write** to record last-active. It pushes a `PRESET_LOADED`
  then a `BULK_INVALIDATED` notification (0x83). **Confirm with `PRESET_GET_ACTIVE` (`0x9A`)**,
  and re-read `GET_ALL_PARAMS` on `BULK_INVALIDATED`.
* **`0x92` PRESET_DELETE** (write-as-read, `wValue`=slot; the loop consumes a `preset_delete_mask`
  bit, **not** gated). Erases the **slot sector** and updates the **directory** (clears the
  occupied bit and the name). Two-sector cost. The active slot **stays selected** even if you
  delete it; it simply becomes unconfigured and will load factory defaults on the next load.
* **`0x94` PRESET_SET_NAME** (OUT write, `wValue`=slot, 1..32-byte payload). Deferred
  (`flash_set_name_pending`, **not** gated). Names live **in the directory**, so this is a light
  **one-sector (directory) write**. Read back with `PRESET_GET_NAME` (`0x93`).
* **`0x96` PRESET_SET_STARTUP** (OUT write, 2-byte payload `mode`,`slot`; mode 0=specified,
  1=last-active). Deferred (`flash_set_startup_pending`, **not** gated). Light **directory**
  write. Read back with `PRESET_GET_STARTUP` (`0x97`).

Related light directory writes handled the same way: `0x98` SET_OUTPUT_CONFIG_MODE and
`0xD4` SET_MASTER_VOLUME_MODE (each flips a mode flag stored in the directory).

### 22.5 Hardware pin writes

Move a GPIO to a new physical pin. All are **write-as-read** (`0xC1`), take the new GPIO in
`wValue`, and return a 1-byte `PIN_CONFIG_*` status. **None of them write flash**: a moved pin
persists only when you subsequently save it (a preset save in with-preset mode, or
`SAVE_OUTPUT_CONFIG` in independent mode). GPIO validity and in-use rules are in Section 2.6;
a rejected pin returns `INVALID_PIN`/`PIN_IN_USE`/`OUTPUT_ACTIVE` and changes nothing.

| ID | Command | Deferred work | Gated | Notes |
|----|---------|---------------|-------|-------|
| 0x7C | `SET_OUTPUT_PIN` | S/PDIF or I2S slot: `output_pin_change_mask` -> `process_pin_changes()` (mute, repin, restart **all** slots in sync). PDM: **immediate** `pdm_change_pin()` | yes (S/PDIF/I2S path) | PDM pin move requires that output be **disabled first** (else `OUTPUT_ACTIVE`) |
| 0xE4 | `SET_SPDIF_RX_PIN` | if S/PDIF is the active input: `spdif_rx_pin_change_pending` -> `spdif_input_stop()`/`start()` on the new pin | no | brief input silence during the hot-swap |
| 0xF1 | `SET_I2S_RX_PIN` | if I2S is the active input: `i2s_rx_pin_change_pending` -> I2S stop/start | yes | mirrors 0xE4 |
| 0xC2 | `SET_I2S_BCK_PIN` | moves the BCK/LRCLK pair (LRCLK = BCK+1); if I2S input active, `i2s_input_restart_pending` -> restart | (input restart) | **rejected while any output slot is I2S**; BCK/LRCLK are shared by all I2S slots |

Because the S/PDIF/I2S output-pin path routes through `process_pin_changes()`, all output slots
are muted and restarted together: **slot alignment is preserved**. Read back the matching GET
(0x7D / 0xE5 / 0xF2 / 0xC3) to confirm.

### 22.6 I2S / S/PDIF / DAC output configuration writes

These reconfigure the physical output bus or the board-level DAC mute.

* **`0xC0` SET_OUTPUT_TYPE** (write-as-read, `wValue = (new_type << 8) | slot`; type 0=S/PDIF,
  1=I2S). Deferred (`output_type_change_mask` + `pending_output_types[]`, **gated**).
  `process_type_switches()` tears down and re-sets the S/PDIF or I2S library for the slot
  (heap alloc/free) and runs its **own synchronized pipeline reset**, so all slots stay
  aligned. Expect a busy window. Read back with `GET_OUTPUT_TYPE` (`0xC1`). Not persisted until
  saved (22.5 note).
* **`0xC4/0xC6/0xC8` MCK enable / pin / multiplier** (write-as-read). Mostly **immediate**
  library calls (`audio_i2s_mck_*`); no pipeline reset. The MCK pin must be a CLK_GPOUTn-capable
  GPIO (GPIO 21 on RP2040; 13/15/21 on RP2350) or the write returns `INVALID_PIN`. Read back
  0xC5 / 0xC7 / 0xC9.
* **`0xEA` SET_DAC_HW_MUTE_CONFIG** (OUT write, 16-byte `DacHwMuteConfig`). Deferred
  (`flash_set_dac_hw_mute_pending`, **not** gated). The main loop validates the config, claims
  the mute GPIO, and **persists it to the directory sector** (it writes the in-RAM `dir_cache`
  and marks the directory dirty for a deferred flush). **Validation happens in the deferred
  handler and a bad config is silently dropped**, so you must **always read back with `0xEB`**
  and compare. This is the one output-config write that does persist to flash on its own.
* **`0xEC` TEST_DAC_HW_MUTE** (write-as-read). Fires a ~1-second async mute pulse on the
  configured pin so an installer can confirm it by ear; no flash, no lasting state change.

### 22.7 `0x53` FACTORY_RESET

* **Transport.** Write-as-read (`0xC1`, `wValue=0`); returns `FLASH_OK` (accepted).
* **Deferred work (gated).** `factory_reset_pending` -> drain, `prepare_pipeline_reset()`,
  suspend RX, `flash_factory_reset()`, `dsp_recalculate_all_filters()`, zero the delay lines,
  re-derive Core 1 mode, perform any output-type switches (if the live config had I2S outputs),
  restart RX.
* **Flash footprint.** **None.** Despite the name, `flash_factory_reset()` only resets **live
  DSP state** to firmware defaults (via `apply_factory_defaults()`) and re-applies the
  device-global IO config; it does **not** erase any stored preset slot or the directory. So a
  factory reset does **not** wipe your saved presets, names, startup policy, or device-global
  settings; it resets the currently-running state and **keeps the active slot selected**. Use
  `PRESET_DELETE` (22.4) to actually remove stored slot data.
* **Slot alignment.** Preserved via the muted synchronized reset (22.1c).

### 22.8 The persistence model (bulk / live vs. flash)

Because so many commands return "accepted" without touching flash, it is worth stating plainly
what does and does not survive a power cycle:

* **Live-only (lost on reboot unless separately saved):** every `SET_*` scalar
  (EQ, delays, gains, loudness, crossfeed, leveller), all pin/output-type/MCK writes, and
  crucially **`SET_ALL_PARAMS` (`0xA1`)**. These change RAM only.
* **Writes flash directly:** `PRESET_SAVE` (`0x90`), `PRESET_DELETE` (`0x92`),
  `PRESET_SET_NAME` (`0x94`), `PRESET_SET_STARTUP` (`0x96`), the mode flags
  (`0x98`/`0xD4`), the device-global saves `SAVE_PARAMS`/`SAVE_OUTPUT_CONFIG`/`SAVE_MASTER_VOLUME`
  (`0x51`/`0x52`/`0xD6`), and `SET_DAC_HW_MUTE_CONFIG` (`0xEA`).
* **Neither (state/action only):** `FACTORY_RESET` (`0x53`, resets live state, no flash write),
  `PRESET_LOAD` (`0x91`, reads a slot plus a light directory last-active write), and
  `TEST_DAC_HW_MUTE` (`0xEC`).

A typical "commit the current tuning" flow is therefore: push live changes (individual `SET_*`
or one `SET_ALL_PARAMS`), confirm via readback, **then** `PRESET_SAVE` to a slot to persist.

### 22.9 `0xF0` ENTER_BOOTLOADER (firmware update)

* **Transport.** Write-as-read (`0xC1`, `wValue=0`). Returns a 1-byte `1` (ACK), then after
  ~100 ms the firmware calls `reset_usb_boot(0,0)` and never returns.
* **Effect.** The device drops off USB and re-enumerates as the RP2 **UF2 mass-storage
  bootloader** (`RPI-RP2`). It is no longer the DSPi and answers no vendor commands until a UF2
  is flashed and it reboots back into the application. This command does not itself write flash;
  it jumps to the ROM bootloader.
* **Bridge handling.** Gate strictly behind explicit user confirmation; there is no undo from the
  host side. Expect the device handle to disappear immediately after the ACK; the host must
  release it and wait for the mass-storage device (to drive the update) and, later, for the DSPi
  to re-enumerate. Do not issue any further vendor command after the ACK.

---

## 23. Test signal generator (siggen)

The onboard **test signal generator** synthesizes measurement and diagnostic
signals (sines, sweeps, noise, tone bursts, IMD pairs, inter-sample-peak patterns,
channel-ID melodies) directly into the output pipeline, with no host audio stream
required. Full reference: `Documentation/Features/test_signals_spec.md`.

State is **transient**: off at boot, never persisted, and stopped by preset load
and factory reset. Injection replaces routed audio on masked output channels
between the matrix mix and per-output processing, so inter-slot sample alignment is
preserved.

### 23.1 Commands

| ID | Name | Dir | wValue | Payload / Response |
|----|------|-----|--------|--------------------|
| 0xA4 | `SIGGEN_SET_CONFIG` | write (`0x41`) | 0 | Payload: `SiggenConfig` (36 B). Validates and stages; STALL on reject |
| 0xA5 | `SIGGEN_GET_CONFIG` | read (`0xC1`) | 0 | Response: `SiggenConfig` (36 B) |
| 0xA6 | `SIGGEN_CONTROL` | write-as-read (`0xC1`) | `SIGGEN_CTL_*` | Response: 1-byte ack (1); STALL on reject |
| 0xA7 | `SIGGEN_GET_STATUS` | read (`0xC1`) | 0 | Response: `SiggenStatus` (16 B) |
| 0xA8 | `SIGGEN_GET_CAPS` | read (`0xC1`) | 0xFFFF or type index | Response: `SiggenCapsHeader` (8 B) or `SiggenTypeDesc` (62 B) |

`0xA6` is **write-as-read**: issue it as an IN transfer even though it mutates
state. `SIGGEN_CTL_*` values: `0` = STOP (5 ms fade out), `1` = START (or restart
if running), `2` = STOP_NOW (hard stop). `SET_CONFIG` never auto-starts; a `SET`
while running restarts the generator (fade-out, apply, fade-in). Opcodes
`0xA9..0xAF` are reserved. All five commands are also reachable over the UART and
I2C target transports.

### 23.2 `SiggenConfig` (36 bytes, little-endian)

| Offset | Type | Field | Meaning |
|-------:|------|-------|---------|
| 0 | u8 | `version` | `SIGGEN_CFG_VERSION` = 1 (mismatch rejected) |
| 1 | u8 | `signal_type` | `SiggenType` 0..14 (see 23.5) |
| 2 | u16 | `channel_mask` | output-channel select, bit i = output i (clamped to valid outputs) |
| 4 | u16 | `invert_mask` | polarity-inverted subset of `channel_mask` |
| 6 | u8 | `flags` | bit0 RAW (bypass crossover+PEQ), bit1 DECORR (noise: per-channel), bit2 WALK |
| 7 | u8 | `reserved0` | 0 |
| 8 | f32 | `level_db` | peak dBFS, -120..0 (clamped) |
| 12 | u32 | `duration_ms` | timing-model dependent (23.4) |
| 16 | u16 | `repeat` | timing-model dependent (23.4) |
| 18 | u16 | `gap_ms` | inter-cycle silence |
| 20 | f32 | `p1` | per-type parameter |
| 24 | f32 | `p2` | per-type parameter |
| 28 | f32 | `p3` | per-type parameter |
| 32 | f32 | `p4` | per-type parameter |

A channel emits only if it is **both** in `channel_mask` **and** enabled in the
matrix mixer (the enabled-intersection rule). RAW mode bypasses only crossover +
PEQ on generator channels; output trim, master volume, mute, and delay still apply.

### 23.3 `SiggenStatus` (16 bytes, `0xA7` response)

| Offset | Type | Field |
|-------:|------|-------|
| 0 | u8 | `version` (1) |
| 1 | u8 | `state` (0 IDLE, 1 FADE_IN, 2 RUN, 3 GAP, 4 FADE_OUT) |
| 2 | u8 | `signal_type` |
| 3 | u8 | `active_channel` (walk channel, or 0xFF) |
| 4 | u32 | `elapsed_ms` |
| 8 | u16 | `cycles_done` (saturates 0xFFFF) |
| 10 | u8 | `stop_reason` (0 NONE, 1 HOST, 2 COMPLETED, 3 PRESET, 4 RECONFIG) |
| 11 | u8 | `reserved0` |
| 12 | f32 | `current_freq` (sweep Hz, else 0) |

### 23.4 `SiggenCapsHeader` (8 B) and `SiggenTypeDesc` (62 B) (`0xA8`)

`wValue = 0xFFFF` returns the header: `u8 version; u8 type_count; u8
output_channels; u8 multitone_max (8 RP2040 / 16 RP2350); u16 valid_channel_mask;
u16 reserved0`.

`wValue = 0..type_count-1` returns a type descriptor: `u8 id; char name[8]; u8
timing_model (0 continuous / 1 sweep / 2 pattern); SiggenParamDesc p[4]`, where each
`SiggenParamDesc` is `u8 semantic; f32 min; f32 max; f32 def` (13 B). Semantic
codes: 0 unused, 1 FREQ_HZ, 2 MS, 3 CYCLES, 4 COUNT, 5 RATIO, 6 PATTERN. The caps
are the authoritative per-platform parameter ranges; prefer them over hard-coded
tables.

### 23.5 Signal types and timing

Timing model reinterprets `duration_ms` / `repeat` / `gap_ms`: **continuous**
(`duration_ms` = total, 0 = until stopped; with WALK it is the per-channel dwell
and `repeat` = passes over the mask); **sweep** (`duration_ms` = one sweep, must be
> 0; `repeat` = sweep count, 0 = infinite; `gap_ms` between sweeps); **pattern**
(`repeat` = periods, 0 = infinite; `gap_ms` extra silence per period).

| ID | Type | Timing | p1 | p2 | p3 | p4 |
|----|------|--------|----|----|----|----|
| 0 | SINE | continuous | freq Hz | - | - | - |
| 1 | SQUARE | continuous | freq Hz | - | - | - |
| 2 | WHITE | continuous | - | - | - | - |
| 3 | PINK | continuous | - | - | - | - |
| 4 | SWEEP_LOG | sweep | f1 Hz | f2 Hz | - | - |
| 5 | SWEEP_LIN | sweep | f1 Hz | f2 Hz | - | - |
| 6 | SWEEP_STEP | sweep | f1 Hz | f2 Hz | steps/oct | dwell ms |
| 7 | IMPULSE | pattern | period ms | - | - | - |
| 8 | CLICKS_ALT | pattern | period ms | - | - | - |
| 9 | POLARITY | pattern | pulse ms | period ms | - | - |
| 10 | TONE_BURST | pattern | freq Hz | on cyc | off cyc | edge cyc |
| 11 | TONE_PAIR | continuous | f1 Hz | f2 Hz | amp ratio A1/A2 | - |
| 12 | MULTITONE | continuous | tone count | f_lo Hz | f_hi Hz | - |
| 13 | ISP | continuous | pattern 0/1 | - | - | - |
| 14 | CHANNEL_ID | pattern | blip ms | - | - | - |

### 23.6 State notifications

Start / stop / completion / reconfigure are pushed as `NOTIFY_EVT_SIGGEN_STATE`
(0x07) on the notification endpoint (see section 18):
`[ver=2, 0x07, flags=0, seq, state, reason, signal_type, channel]` (8 bytes).

### 23.7 Example: 1 kHz sine at -20 dBFS on channels 0 and 1

`SET_CONFIG` (0xA4, OUT) 36-byte payload, then `CONTROL` START (0xA6, `wValue=1`):

```
01 00 03 00 00 00 00 00  00 00 A0 C1 00 00 00 00
00 00 00 00 00 00 7A 44  00 00 00 00 00 00 00 00
00 00 00 00
```

(`type=SINE mask=0x0003 level=-20.0 duration=0 p1=1000.0`.) See
`test_signals_spec.md` section 13 for out-of-phase, log-sweep, and channel-ID walk
examples.
