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

Write-as-read commands (34, `vendor_commands.c` GET path, updated 2026-11-15):
`0x0E`, `0x25`, `0x51`, `0x52`, `0x53`, `0x68`, `0x6A`, `0x6C`, `0x7C`, `0x83`,
`0x8F` (`wValue` 0 or 1), `0x90`, `0x91`, `0x92`, `0x9D`, `0x9E`, `0xA6`, `0xB1`,
`0xB3`, `0xC0`, `0xC2`, `0xC4`, `0xC6`, `0xC8`, `0xCA`, `0xCC`, `0xD6`, `0xE4`,
`0xE9`, `0xEC`, `0xF0`, `0xF1`, `0xF3`, `0xFE`.

Over UART and I2C these must be sent as GET-type frames; a SET-type frame returns
`CTRL_STATUS_ERROR`.

Two opcodes are handled on **both** paths with the same `bRequest`: an OUT
transfer sets and an IN transfer reads. They are `0x03` `OUTPUT_LATENCY`
(Section 12.3) and `0x81` `LIMITER` (Section 8.3).

A few reads have side effects. `0x0B`, `0x0C` and `0x0F` count as spectrum
analyser reads, so they keep it running and can auto-start it (Section 24).
`0xA2` at offset 0 takes the bulk lock and snapshots the state.

### 1.2 Control-stage flow

* **Reads (`0xC1`)**: SETUP then IN data stage of `wLength` bytes, then OUT status
  (ZLP). Request the exact length shown in the "Response" column.
* **Writes (`0x41`)**: SETUP then OUT data stage carrying the payload, then IN
  status (ZLP). A zero-length payload write is ACKed with no data stage.
* **STALL** means the firmware refused the request. On the IN path that covers
  an unknown opcode, a bad index, or a handler that returned failure. On the OUT
  path only an oversized payload STALLs, at the SETUP stage. A true OUT write
  (`0x41`) that the handler rejects (bad index, short payload, out-of-range
  value, feature absent on this platform) is still ACKed and silently changes
  nothing (`vendor_commands.c`, `CONTROL_STAGE_DATA`). Confirm important writes
  with the matching read (Section 19). A bridge should surface a STALL as an
  error rather than retrying blindly.
* Over the UART and I2C control transports a write whose handler reports failure
  returns `CTRL_STATUS_ERROR` (`Features/control_interfaces_spec.md`). Many
  handlers ignore a short payload without reporting failure, so an `OK` status is
  still not proof the value applied.

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
**readback** command (Section 19) and compare against what you sent.

---

## 2. Index and addressing reference

Several commands address "channels", "outputs", "pin outputs", or "bands". These
are **different index spaces**. Mixing them up is the most common integration bug.

### 2.1 Channel index (`NUM_CHANNELS` = 7 on RP2040, 17 on RP2350)

*Last updated: 2026-11-15*

Used by EQ, per-channel delay, channel names, and the combined peak meter.
Inputs and outputs share one index space. Inputs come first, then outputs.
There is no "master" channel; inputs are channels in their own right with their
own PEQ and meters (`config.h`, "Unified channel model").

| Index | RP2040 (2 in + 5 out) | RP2350 (8 in + 9 out) |
|-------|--------|--------|
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

`NUM_INPUT_CHANNELS` (2 or 8) is the platform maximum. On RP2350 the number of
inputs that carry audio depends on the active source (`active_input_channel_count()`).
USB follows its alt setting (2, 4, 6 or 8 channels), I2S follows
`SET_I2S_INPUT_CHANNELS` (0xF3), ADAT is always 8, and S/PDIF is always 2. Input channels
have PEQ and metering but no crossover. Output channels have PEQ, crossover,
gain, delay and mute.

### 2.2 Output index (`NUM_OUTPUT_CHANNELS` = 5 on RP2040, 9 on RP2350)

*Last updated: 2026-11-15*

Used by the matrix mixer and the per-output gain/mute/enable/delay commands, and
by the per-output masks (loudness, limiter, leveller and similar). Output index
`out` maps to channel index `out + NUM_INPUT_CHANNELS` (`CH_OUT_1 + out`): add 2
on RP2040 and 8 on RP2350.

| Output index | RP2040 channel | RP2350 channel | Meaning |
|--------------|----------------|----------------|---------|
| 0 | 2 | 8 | S/PDIF 1 L |
| 1 | 3 | 9 | S/PDIF 1 R |
| ... | ... | ... | ... |
| `NUM_OUTPUT_CHANNELS-1` | 6 | 16 | PDM sub |

Each S/PDIF slot carries two outputs, and any slot can be switched to I2S
(Section 12). The output index does not change when a slot changes type.

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
| 20 .. 23 | Crossover filter bands (output channels only, `channel >= NUM_INPUT_CHANNELS`) |

Crossover bands are valid only on **output** channels (index >=
`NUM_INPUT_CHANNELS`, so 2 on RP2040 and 8 on RP2350). A crossover write to an
input channel is silently ignored.

### 2.5 Status codes

Pin/output/config commands return one of (`config.h`):

| Code | Name | Meaning |
|------|------|---------|
| 0x00 | `PIN_CONFIG_SUCCESS` | Applied (or accepted for deferred work) |
| 0x01 | `PIN_CONFIG_INVALID_PIN` | GPIO not allowed, or bad enum value |
| 0x02 | `PIN_CONFIG_PIN_IN_USE` | GPIO already claimed by another function |
| 0x03 | `PIN_CONFIG_INVALID_OUTPUT` | Output/slot index out of range, or feature off |
| 0x04 | `PIN_CONFIG_OUTPUT_ACTIVE` | Must disable the output first |
| 0x05 | `PIN_CONFIG_INVALID_PARAM` | A non-pin field is out of range (enable value, mode, baud rate, I2C address) |

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

*Last updated: 2026-11-15*

`is_valid_gpio_pin()` rejects GPIO 23-25 (power/LED) and anything above 28 on
RP2040 or 29 on RP2350. GPIO 12, 16 and 17 are ordinary pins. A build with
`DSPI_PLATFORM_PINS` defined (a non-RP2040/RP2350 port) applies that platform's
own per-function rule instead.

In every pin SET, GPIO `0xFF` (`PIN_RESET_TO_DEFAULT`) restores that function's
default pin. GPIO `0` means GPIO 0.

Pin commands also reject a GPIO already claimed by another function
(`PIN_CONFIG_PIN_IN_USE`). The claim check covers:

* every output data pin (S/PDIF or I2S slots, PDM);
* S/PDIF RX input 1 always, and S/PDIF inputs 2-4 only while enabled;
* the ADAT output and ADAT input pins (RP2350) while enabled;
* the I2S BCK/LRCLK pair (both pairs in split clock-pin mode) while any slot is
  I2S or I2S is the active input;
* the I2S RX data pin of every active input pair;
* MCK while enabled;
* the DAC hardware-mute pin;
* the UART and I2C control pins while that interface is live;
* GPIOs owned by live control-surface bindings.

---

## 3. Identity and platform

| ID | Name | Dir | wValue | Payload / Response | Notes |
|----|------|-----|--------|--------------------|-------|
| 0x7E | `GET_SERIAL` | R | 0 | resp **16 bytes** | Device serial string (raw bytes, NUL-padded) |
| 0x7F | `GET_PLATFORM` | R | 0 | resp **7 bytes** | `[0]`=platform (0=RP2040, 1=RP2350), `[1]`=fw major, `[2]`=legacy (minor<<4)\|patch nibbles, `[3]`=`NUM_OUTPUT_CHANNELS`, `[4]`=fw minor, `[5]`=fw patch, `[6]`=`FW_VERSION_BETA` (0 = final release). Clamped to `wLength`, so 4- and 6-byte readers still work |
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
addressed on an input channel.

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
| 0xD0 | `SET_PREAMP_CH` | W | input ch (0 .. NUM_INPUT_CHANNELS-1) | payload f32 dB | 0xD1 |
| 0xD1 | `GET_PREAMP_CH` | R | input ch | resp f32 dB | - |

`NUM_INPUT_CHANNELS` = 2 on RP2040 and 8 on RP2350. The legacy `0x44` writes
the same value to every input channel.

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

### 5.5 Input source and device names

| ID | Name | Dir | wValue | Payload / Response |
|----|------|-----|--------|--------------------|
| 0x54 | `SET_INPUT_SOURCE_NAME` | W | source ID (0..6) | payload 1-32 bytes; a leading NUL clears; bytes past 31 dropped |
| 0x55 | `GET_INPUT_SOURCE_NAME` | R | source ID (0..6) | resp **32 bytes**, NUL-terminated; all zeros = unnamed |
| 0x56 | `SET_DEVICE_NAME` | W | 0 | payload 1-32 bytes; a leading NUL clears; bytes past 31 dropped |
| 0x57 | `GET_DEVICE_NAME` | R | 0 | resp **32 bytes**, NUL-terminated; all zeros = unnamed |

Names are device-global and persist in the preset directory. Until 2026-10-31
0x54-0x57 carried a legacy 3-channel gain/mute that no audio path read;
firmware from before then answers 0x55 with 4 bytes and 0x57 with 1 byte.
Probe 0x57 on its own: firmware from 2026-10-31 has source names but STALLs
it. Each stored name is announced by notify event 0x0F (source) or 0x10
(board). See `Features/input_source_names_spec.md` and
`Features/device_name_spec.md`.

---

## 6. Delays

| ID | Name | Dir | wValue | Payload / Response |
|----|------|-----|--------|--------------------|
| 0x48 | `SET_DELAY` | W | channel index | payload f32 ms (negatives clamped to 0) |
| 0x49 | `GET_DELAY` | R | channel index | resp f32 ms |

`wValue` is a **full channel index** (0 .. NUM_CHANNELS-1). Only output channels
are delayed: a value stored on an input channel is accepted and read back but has
no audible effect. The delay line holds `MAX_DELAY_SAMPLES - 1` samples (1023 on
RP2040, 2047 on RP2350), which is about 21 ms and 42 ms at 48 kHz and half that
at 96 kHz. Longer values are stored but capped when applied. The PDM sub output
also carries a fixed alignment offset, so it reaches the cap slightly early.
Per-output delay (Section 8) writes the same underlying per-channel delay for
`channel = output + NUM_INPUT_CHANNELS`.

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

#### Crossfeed output pairs

*Last updated: 2026-11-15*

| ID | Name | Dir | wValue | Payload / Response | Notes |
|----|------|-----|--------|--------------------|-------|
| 0xFC | `SET_CROSSFEED_OUTPUTS` | W (0x41) | 0 | 1 byte pair mask | Bit p = output pair p (outputs 2p, 2p+1). Masked to 4 bits on RP2350, 2 bits on RP2040. Immediate |
| 0xFD | `GET_CROSSFEED_OUTPUTS` | R (0xC1) | 0 | 1 byte pair mask | |

Default mask is `0x01` (pair 0). The PDM sub is not a pair. The pipeline reads the mask each packet,
so no recompute is queued. The SET sends `PARAM_CHANGED` for `crossfeed.output_pair_mask`. Stored per
preset. Crossfeed itself is still switched on and off by `0x5E` `SET_CROSSFEED`.

There is no separate spec; see `crossfeed.h`.

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

### 7.4 Volume leveller channel masks

*Last updated: 2026-11-15*

Both platforms. Bit k = input channel k. Both masks are ANDed with the active input set at run time, so bits for inactive inputs do nothing.

| ID | Name | Dir | wValue | Payload / Response | Notes |
|----|------|-----|--------|--------------------|-------|
| 0xDE | `SET_LEVELLER_MASKS` | W (0x41) | 0 | 2 bytes: `[detector_mask, apply_mask]` | Stored as sent. Default 0xFF / 0xFF. Recompute without state reset, so a mask change is glitch-free |
| 0xDF | `GET_LEVELLER_MASKS` | R (0xC1) | 0 | 2 bytes: `[detector_mask, apply_mask]` | - |

Spec: `Features/volume_leveller_spec.md` (section 2.7).

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
| 0 | u8 | `input` (0 .. NUM_INPUT_CHANNELS-1) |
| 1 | u8 | `output` (0 .. NUM_OUTPUT_CHANNELS-1) |
| 2 | u8 | `enabled` (0/1) |
| 3 | u8 | `phase_invert` (0/1) |
| 4 | f32 | `gain_db` |

On RP2350, while the stereo upmixer is running (stereo input, rate up to 48 kHz),
matrix input rows 2, 3 and 4 carry its derived C, Ls and Rs signals instead of
inputs 3-5 (Section 25.5). Their crosspoints route those signals like any other row.

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

### 8.3 Output limiter (`0x81`)

*Last updated: 2026-11-15*

One opcode for both directions. `wValue = (output << 8) | index`. Values are float32 LE.

| ID | Name | Dir | wValue | Payload / Response | Notes |
|----|------|-----|--------|--------------------|-------|
| 0x81 | `LIMITER` | W (0x41) | `(output << 8) \| index` | 4 B float32 | index 0-3. `output = 0xFF` sets every output. Bad output/index is a no-op; NaN is ignored; payload < 4 B is ignored |
| 0x81 | `LIMITER` | R (0xC1) | `(output << 8) \| index`, index 0-3 | 4 B float32 | STALLs on bad output (including 0xFF) or index |
| 0x81 | `LIMITER` | R (0xC1) | low byte `0x80` | `NUM_OUTPUT_CHANNELS` x u16 LE (18 B RP2350, 10 B RP2040) | Gain reduction per output in 0.01 dB (0 = none, 12000 = floor). Output byte ignored |
| 0x81 | `LIMITER` | R (0xC1) | low byte `0x81` | 4 B (below) | Status. Output byte ignored. Use it to feature-detect the limiter |

| Index | Parameter | Range (clamped) | Default |
|-------|-----------|-----------------|---------|
| 0 | `enabled` | any non-zero = on | 0 |
| 1 | `threshold_db` | -30.0 .. 0.0 | -1.0 |
| 2 | `release_ms` | 10 .. 1000 | 100 |
| 3 | `link_group` | 0 .. 4, rounded (0 = unlinked) | 0 |

A SET on a linked output writes every member of its group. Joining a group adopts the settings of
the group's lowest-numbered existing member. Each SET emits `PARAM_CHANGED` for every output it
wrote. The SET is immediate in the handler; `limiter_update_pending` makes the main loop recompute
coefficients, and the lookahead delay engages or releases only in silence.

Status block (`index = 0x81`):

| Offset | Type | Field |
|--------|------|-------|
| 0 | u8 | `engaged` (1 = lookahead delay in the signal path) |
| 1 | u8 | lookahead samples (32) |
| 2 | u8 | block samples (16) |
| 3 | u8 | `NUM_OUTPUT_CHANNELS` (9 RP2350, 5 RP2040) |

Spec: `Features/output_limiter_spec.md` sections 3-5.

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
| 0xE0 | `SET_INPUT_SOURCE` | W (0x41) | 0 | 1 byte `InputSource` (table below) | Deferred (pipeline reset). A source that is not selectable, or equals the active one, is ACKed and ignored |
| 0xE1 | `GET_INPUT_SOURCE` | R | 0 | 1 byte (active source) | Reports active, not pending |
| 0xE2 | `GET_SPDIF_RX_STATUS` | R | 0 | **16 bytes** `SpdifRxStatusPacket` | See 10.1 |
| 0xE3 | `GET_SPDIF_RX_CH_STATUS` | R | 0 | **24 bytes** IEC 60958 channel status | Raw channel-status bytes |
| 0xE4 | `SET_SPDIF_RX_PIN` | **write-as-read (0xC1)** | `(index << 8) \| GPIO`, index 0-3 = S/PDIF input 1-4 | resp **1 byte** status | `0x03` if index >= 4. A disabled optional input stores the pin unchecked; it is validated at enable. Hot-swaps if that input is the active source; persist via preset save |
| 0xE5 | `GET_SPDIF_RX_PIN` | R | index (0-3) | 1 byte | Index >= 4 returns 0, not a STALL |
| 0xED | `SET_INPUT_RATE` | W (0x41) | 0 | **u32** Hz (44100/48000/96000) | I2S-input rate authority |
| 0xEE | `GET_INPUT_RATE` | R | 0 | **8 bytes**: `u32 current_Hz`, `u32 selected_I2S_Hz` | - |
| 0xF1 | `SET_I2S_RX_PIN` | **write-as-read (0xC1)** | `(pair << 8) \| GPIO`, pair 0-3 (RP2350) or 0 (RP2040) | resp **1 byte** status | `0x03` for a pair the part lacks. Mirrors 0xE4 |
| 0xF2 | `GET_I2S_RX_PIN` | R | pair | 1 byte | Bad pair returns 0 |

`InputSource` values for `0xE0` / `0xE1`:

| Value | Source | RP2040 | RP2350 |
|-------|--------|--------|--------|
| 0 | USB | yes | yes |
| 1 | S/PDIF input 1 | yes (always enabled) | yes (always enabled) |
| 2 | I2S | yes | yes |
| 3 | ADAT | never | only when ADAT input is enabled (0x68) and has a pin (0x6A) |
| 4 | S/PDIF input 2 | when enabled with 0xE9 | when enabled with 0xE9 |
| 5 | S/PDIF input 3 | when enabled with 0xE9 | when enabled with 0xE9 |
| 6 | S/PDIF input 4 | when enabled with 0xE9 | when enabled with 0xE9 |

The default pins for S/PDIF inputs 1-4 are GPIO 5, 20, 21 and 22.

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

### 10.2 Optional S/PDIF inputs 2-4

*Last updated: 2026-11-15*

There are 4 S/PDIF inputs on both platforms. Input 1 (index 0) is always enabled. Inputs 2-4
(index 1-3) are optional and disabled by default.

| ID | Name | Dir | wValue | Payload / Response | Notes |
|----|------|-----|--------|--------------------|-------|
| 0xE9 | `SET_SPDIF_INPUT_ENABLE` | write-as-read (0xC1) | `(index << 8) \| enable` | 1 byte `PIN_CONFIG_*` | Any non-zero low byte enables. RAM only; persisted on `PRESET_SAVE` |
| 0xEF | `GET_SPDIF_INPUT_CONFIG` | R (0xC1) | 0 | 6 bytes (below) | A shorter wLength gets a prefix of the same layout |

`0xE9` status: `0x00` success (also for no change, and enable of index 0), `0x02` PIN_IN_USE (enable:
the input's GPIO is invalid or taken; disable: the input is the active source or the target of a
pending source switch), `0x03` INVALID_OUTPUT (index >= 4, or disable of index 0). A change sends
`PARAM_CHANGED` for `input_config.spdif_rx_enabled_ext_p1` (mask + 1).

`0xEF` response (6 bytes):

| Offset | Type | Field |
|--------|------|-------|
| 0 | u8 | input count (4) |
| 1 | u8 | enable mask, bit i = input i+1 (bit 0 always set) |
| 2 | u8[4] | GPIO of inputs 1..4 |

Spec: `Features/SPDIF_input_spec.md` sections 4.7-4.8.

### 10.3 I2S input channel count

*Last updated: 2026-11-15*

| ID | Name | Dir | wValue | Payload / Response | Notes |
|----|------|-----|--------|--------------------|-------|
| 0xF3 | `SET_I2S_INPUT_CHANNELS` | write-as-read (0xC1) | low byte: 2, 4, 6 or 8 | 1 byte `PIN_CONFIG_*` | RP2040 accepts only 2 |
| 0xF4 | `GET_I2S_INPUT_CHANNELS` | R (0xC1) | 0 | 1 byte active count | |

`0xF3` status: `0x00` success (also for no change), `0x01` INVALID_PIN (count not 2/4/6/8),
`0x03` INVALID_OUTPUT (more pairs than the part has: RP2350 4, RP2040 1), or the first failing
`PIN_CONFIG_*` from re-validating each newly activated pair's data pin (`0x01` invalid GPIO, `0x02`
in use). A change while I2S is the active source restarts the input in the main loop and pushes
`NOTIFY_EVT_INPUT_FORMAT` (0x05). Every change sends `PARAM_CHANGED` for
`input_config.i2s_input_channels`.

Spec: `Features/i2s_multi_input.md` section 7.1.

### 10.4 I2S clock mode (master / slave)

*Last updated: 2026-11-15*

Available on both platforms. Only acts while the input source is I2S.

| ID | Name | Dir | wValue | Payload / Response | Notes |
|----|------|-----|--------|--------------------|-------|
| 0x88 | `SET_I2S_CLOCK_MODE` | W (0x41) | 0 | 1 byte: 0 = master, 1 = slave | Values > 1 ignored. Deferred. Same mode with nothing pending is a no-op |
| 0x89 | `GET_I2S_CLOCK_MODE` | R (0xC1) | 0 | 1 byte live mode | A pending SET is not reflected until applied |
| 0x8A | `GET_I2S_SLAVE_STATUS` | R (0xC1) | 0 | `I2sSlaveStatusPacket` (16 B) | |

The main loop applies the SET. If the source is not I2S it only records the mode. If it is I2S it
waits for the pipeline reset window, then mutes, stops the I2S input, rebuilds I2S output clocking,
sets the MCK policy (MCK off in slave mode; back to the selected rate in master mode), and restarts
the input. `PARAM_CHANGED` for `input_config.i2s_clock_mode` is sent at apply time, not at SET time.
Lock-state changes in slave mode push `NOTIFY_EVT_I2S_SLAVE_STATE` (0x09).

`I2sSlaveStatusPacket` (16 bytes):

| Offset | Type | Field |
|--------|------|-------|
| 0 | u8 | `state`: 0 inactive, 1 acquiring, 2 relocking, 3 locked |
| 1 | u8 | `clock_mode` (live, 0/1) |
| 2 | u8 | `lock_count` since boot (saturates at 255) |
| 3 | u8 | `loss_count` since boot (saturates at 255) |
| 4 | u32 | `detected_rate` Hz (44100/48000/96000; 0 unless locked) |
| 8 | u32 | `measured_hz` raw external rate (0 when no clocks) |
| 12 | u8 | `slip_count` framing slips since boot (saturates at 255) |
| 13 | u8[3] | reserved (0) |

Spec: `Features/i2s_slave_input_spec.md`.

### 10.5 ADAT input (`0x68`-`0x6E`, RP2350)

*Last updated: 2026-11-15*

Every ADAT input setter is **write-as-read**. It carries its argument in `wValue`, runs on
the IN path and answers with one `PIN_CONFIG_*` status byte. 0x6F is reserved.

| ID | Name | Dir | wValue | Payload / Response | Notes |
|----|------|-----|--------|--------------------|-------|
| 0x68 | `SET_ADAT_INPUT_ENABLE` | write-as-read (0xC1) | low byte: 0 = disable, 1 = enable | resp **1 byte** status | Immediate RAM flag. Enable needs a valid, set pin (see below). Disable is refused with `0x02` while ADAT is the active source or the target of a pending source switch. RP2040: always `0x03` |
| 0x69 | `GET_ADAT_INPUT_ENABLE` | R (0xC1) | 0 | resp **1 byte** (0/1) | Configured enable. Answers on both platforms (0 on RP2040) |
| 0x6A | `SET_ADAT_INPUT_PIN` | write-as-read (0xC1) | low byte: GPIO, or `0xFF` to clear | resp **1 byte** status | `0xFF` clears the pin to "unset". Clear is allowed only while disabled (else `0x02`). May share the ADAT output's GPIO (loopback). If ADAT is the live source, a pin change arms a muted receiver restart. RP2040: always `0x03` |
| 0x6B | `GET_ADAT_INPUT_PIN` | R (0xC1) | 0 | resp **1 byte** | GPIO, `0xFF` = unset (the boot default) |
| 0x6C | `SET_ADAT_INPUT_CLOCK_MODE` | write-as-read (0xC1) | low byte: 0 = master, 1 = slave | resp **1 byte** status | **Deferred.** Accepted on both platforms so presets round-trip. `0x05` if value > 1 |
| 0x6D | `GET_ADAT_INPUT_CLOCK_MODE` | R (0xC1) | 0 | resp **1 byte** (0/1) | Live mode. A pending 0x6C is not reflected until the main loop applies it |
| 0x6E | `GET_ADAT_INPUT_STATUS` | R (0xC1) | 0 | resp **20 bytes** `AdatInputStatusPacket` | RP2040: 20 zero bytes |

Status codes returned by 0x68 / 0x6A / 0x6C:

| Code | Name | When |
|------|------|------|
| 0x00 | `PIN_CONFIG_SUCCESS` | Applied, accepted for deferred work, or a no-op (value already set) |
| 0x01 | `PIN_CONFIG_INVALID_PIN` | 0x6A: GPIO fails `is_valid_gpio_pin()`. 0x68 enable: stored pin is unset (`0xFF`) or invalid |
| 0x02 | `PIN_CONFIG_PIN_IN_USE` | GPIO owned by another function (the ADAT output's pin is exempt). 0x68 disable while ADAT is live or pending. 0x6A clear while enabled |
| 0x03 | `PIN_CONFIG_INVALID_OUTPUT` | 0x68 / 0x6A on RP2040 |
| 0x05 | `PIN_CONFIG_INVALID_PARAM` | 0x68 or 0x6C value > 1 |

Behavior:

* Platform: the receiver exists on RP2350 only. On RP2040, 0x68 and 0x6A return `0x03`
  and 0x6E returns zeros. 0x69, 0x6B, 0x6C and 0x6D still store and return the config.
* Order: set the pin (0x6A) before enabling (0x68). A disabled input reserves nothing.
  An enabled input's GPIO counts as owned by every other pin command.
* Source selection is `SET_INPUT_SOURCE` (0xE0) with value 3. It is silently ignored unless
  the input is enabled and has a pin.
* Enable (0x68) only flips a RAM flag. No hardware starts until source 3 is selected.
* Pin change (0x6A) is a RAM write. When ADAT is the active source it sets
  `adat_input_restart_pending`. The main loop then fades, stops and restarts the receiver.
* Clock mode (0x6C) sets `pending_adat_clock_mode` and `adat_clock_mode_change_pending`.
  When ADAT is not the active source the main loop adopts the mode at once. When it is
  active, the main loop waits for `pipeline_reset_ready()`, then restarts the receiver under
  mute in the new mode. Returning to master also restores the stored device rate.
* Notifications: a successful 0x68 emits `PARAM_CHANGED` for
  `input_config.adat_input_enabled_p1` (offset 4730, value = enable + 1). A successful 0x6A
  emits `PARAM_CHANGED` for `input_config.adat_input_pin` (offset 4729, 0 = unset). The clock
  mode emits `PARAM_CHANGED` for `input_config.adat_clock_mode_p1` (offset 4731, value =
  mode + 1) when the main loop applies it, not at SET time. Lock-state changes emit
  `NOTIFY_EVT_ADAT_INPUT_STATE` (0x0B).
* Persistence: the three fields live in the preset slot and in the directory IO block, like
  the other physical IO config.
* Spec: `Features/adat_input_spec.md`.

`AdatInputStatusPacket` (20 bytes, packed, little-endian):

| Offset | Type | Field |
|--------|------|-------|
| 0 | u8 | `state`: 0 = Inactive, 1 = Acquiring, 2 = Syncing, 3 = Locked, 4 = Relocking |
| 1 | u8 | `clock_mode` (live): 0 = master, 1 = slave |
| 2 | u8 | `enabled` (configured) |
| 3 | u8 | `pin` (configured RX GPIO, `0xFF` = unset) |
| 4 | u8 | `rate_ok`: 0 = master mode parked because the device rate is above 48 kHz; always 1 in slave mode |
| 5 | u8 | `lock_count` (saturates at 255; zeroed each time the receiver starts) |
| 6 | u8 | `loss_count` (saturates at 255; zeroed each time the receiver starts) |
| 7 | u8 | `slip_count`: losses caused by header verification failure |
| 8 | u16 | `header_err`: cumulative header mismatches (wraps) |
| 10 | u16 | reserved (0) |
| 12 | u32 | `detected_rate` Hz while locked (master mode: device rate), else 0 |
| 16 | u32 | `measured_hz`: slave mode raw measured wire rate; 0 in master mode |

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
| 0xC2 | `SET_I2S_BCK_PIN` | write-as-read | `(role << 8) \| GPIO`, role 0 = master/unified pair, 1 = slave pair | 1 byte status | LRCLK = BCK+1; rejected while any slot is I2S. Legacy hosts send role 0 |
| 0xC3 | `GET_I2S_BCK_PIN` | R | role (0/1) | 1 byte | Bad role returns 0 |
| 0xC4 | `SET_MCK_ENABLE` | write-as-read | 0/1 | 1 byte status | - |
| 0xC5 | `GET_MCK_ENABLE` | R | 0 | 1 byte | - |
| 0xC6 | `SET_MCK_PIN` | write-as-read | new GPIO | 1 byte status | Must be a CLK_GPOUTn-capable pin |
| 0xC7 | `GET_MCK_PIN` | R | 0 | 1 byte | - |
| 0xC8 | `SET_MCK_MULTIPLIER` | write-as-read | 0=128x, 1=256x | 1 byte status | - |
| 0xC9 | `GET_MCK_MULTIPLIER` | R | 0 | 1 byte (0=128x, 1=256x) | - |

`slot` is `0 .. NUM_SPDIF_INSTANCES-1` (2 on RP2040, 4 on RP2350). BCK/LRCLK are
shared across all I2S slots. MCK is driven by hardware CLK_GPOUTn so its pin must
map to clk_gpout0..3 (GPIO 21 on RP2040; GPIO 13/15/21 on RP2350).

### 12.1 I2S clock-pin mode (unified / split)

*Last updated: 2026-11-15*

| ID | Name | Dir | wValue | Payload / Response | Notes |
|----|------|-----|--------|--------------------|-------|
| 0xFE | `SET_I2S_CLOCK_PIN_MODE` | write-as-read (0xC1) | low byte: 0 = unified, 1 = split | 1 byte `PIN_CONFIG_*` | Applied synchronously |
| 0xFF | `GET_I2S_CLOCK_PIN_MODE` | R (0xC1) | 0 | 1 byte live mode (0/1) | |

`0xFE` status: `0x00` success (also for no change), `0x02` PIN_IN_USE (entering split while the slave
BCK/LRCLK pair overlaps the master pair or either slave GPIO is in use), `0x04` OUTPUT_ACTIVE (the
clock mode is, or is pending, slave and any output slot is I2S), `0x05` INVALID_PARAM (mode > 1).
When the change moves the live pair (slave clocking) and I2S is the active source, the input
restarts in the main loop. Success with a change sends `PARAM_CHANGED` for
`i2s_config.clock_pin_mode_p1`. The slave BCK pin itself is set with `0xC2` role 1.

Spec: `Features/clock_pins_spec.md`.

### 12.2 ADAT output (`0xCA`-`0xCE`, RP2350)

*Last updated: 2026-11-15*

The ADAT output mirrors output channels 1-8 (post-gain) as one lightpipe stream. It runs at
44.1 / 48 kHz only and auto-suspends above 48 kHz.

| ID | Name | Dir | wValue | Payload / Response | Notes |
|----|------|-----|--------|--------------------|-------|
| 0xCA | `SET_ADAT_ENABLE` | write-as-read (0xC1) | low byte: 0 = disable, 1 = enable | resp **1 byte** status | Same value = no-op success. Enable validates the configured pin. Disable always succeeds. **Deferred** hardware apply. RP2040: always `0x03` |
| 0xCB | `GET_ADAT_ENABLE` | R (0xC1) | 0 | resp **1 byte** (0/1) | Configured enable (the intent, not the running state). RP2040: 0 |
| 0xCC | `SET_ADAT_PIN` | write-as-read (0xC1) | low byte: GPIO; `0xFF` = restore default (GPIO 12) | resp **1 byte** status | 0 means GPIO 0. Allowed while enabled; the stream moves under a muted restart. **Deferred** hardware apply. RP2040: always `0x03` |
| 0xCD | `GET_ADAT_PIN` | R (0xC1) | 0 | resp **1 byte** | Configured GPIO. RP2040: 0 |
| 0xCE | `GET_ADAT_STATUS` | R (0xC1) | 0 | resp **8 bytes** `AdatStatus` | RP2040: 8 zero bytes |

Status codes returned by 0xCA / 0xCC:

| Code | Name | When |
|------|------|------|
| 0x00 | `PIN_CONFIG_SUCCESS` | Recorded (hardware apply follows), or a no-op |
| 0x01 | `PIN_CONFIG_INVALID_PIN` | GPIO fails `is_valid_gpio_pin()` (0xCC), or the configured pin is invalid (0xCA enable) |
| 0x02 | `PIN_CONFIG_PIN_IN_USE` | GPIO owned by another function |
| 0x03 | `PIN_CONFIG_INVALID_OUTPUT` | Any SET on RP2040 |
| 0x05 | `PIN_CONFIG_INVALID_PARAM` | 0xCA value > 1 |

Behavior:

* Platform: RP2350 only. RP2040 SETs return `0x03`; GETs return zeros.
* Deferred: the handler records the config with `adat_output_set_config()`, which sets
  `adat_output_config_dirty`. The main loop applies it with `prepare_pipeline_reset()` +
  `complete_pipeline_reset()` once `pipeline_reset_ready()` is true. Expect the brief global
  mute-and-resync of any output reconfiguration. The status byte is answered at once.
* While enabled, the ADAT GPIO is owned. Other pin commands reject it with `0x02`.
  The ADAT input may share it (loopback).
* Notifications: a successful SET emits `PARAM_CHANGED` for `adat_config.enabled`
  (offset 5868) or `adat_config.pin` (offset 5869). Stream start/stop, including rate
  suspend/resume, emits `NOTIFY_EVT_ADAT_STATE` (0x08).
* Persistence: follows `output_config_mode`. Save the preset (0x90) in WITH_PRESET mode, or
  send `SAVE_OUTPUT_CONFIG` (0x52) in INDEPENDENT mode.
* Spec: `Features/adat_output_spec.md`.

`AdatStatus` (8 bytes, packed, little-endian):

| Offset | Type | Field |
|--------|------|-------|
| 0 | u8 | `enabled` (configured intent) |
| 1 | u8 | `active` (stream running) |
| 2 | u8 | `pin` (configured data GPIO) |
| 3 | u8 | `rate_ok` (device rate is 44.1 or 48 kHz) |
| 4 | u16 | `resync_count` (stream restarts since boot) |
| 6 | u16 | `slip_count` (emergency local resyncs since boot; should stay 0) |

`enabled=1, active=0, rate_ok=0` is the auto-suspended state above 48 kHz.

### 12.3 Output latency (`0x03`)

*Last updated: 2026-11-15*

One opcode for both directions. The transfer direction selects SET or GET.

| ID | Name | Dir | wValue | Payload / Response | Notes |
|----|------|-----|--------|--------------------|-------|
| 0x03 | `OUTPUT_LATENCY` | W (0x41) | 0 | 1 byte option ID | Deferred. Unknown ID is ignored on USB, ERROR on UART/I2C. Re-sending the stored ID with nothing pending is a no-op |
| 0x03 | `OUTPUT_LATENCY` | R (0xC1) | 0 | `OutputLatencyStatus` (12 B) | Status |
| 0x03 | `OUTPUT_LATENCY` | R (0xC1) | 1 | `option_count` x `OutputLatencyOptionWire` (8 B each, 56 B today) | Option table. Any other wValue STALLs |

Option IDs: 1 = 0.67 ms, 2 = 1 ms (both stream-input only, flag `0x01`), 3 = 2 ms, 4 = 3 ms,
5 = 4 ms (default), 6 = 6 ms, 7 = 8 ms. A stream-only option requested while the source is USB
runs as 2 ms (`effective_id` differs from `requested_id`).

The SET is applied by the main loop: a directory flash write (device-global, not per preset)
followed by a full synchronized output restart (one short mute). `pending` reads 1 until that
completes. No notification is sent.

`OutputLatencyStatus` (12 bytes):

| Offset | Type | Field |
|--------|------|-------|
| 0 | u8 | `requested_id` (stored option) |
| 1 | u8 | `effective_id` (option in force) |
| 2 | u16 | `effective_us` (ring fill target at the current rate, ceiling applied) |
| 4 | u16 | `target_frames` (live slot ring fill target) |
| 6 | u16 | `batch_frames` (stream-input batch at the current rate) |
| 8 | u8 | `pending` (1 until a SET has fully applied) |
| 9 | u8 | `option_count` (entries in the wValue 1 reply) |
| 10 | u16 | reserved (0) |

`OutputLatencyOptionWire` (8 bytes per entry):

| Offset | Type | Field |
|--------|------|-------|
| 0 | u8 | `id` |
| 1 | u8 | `flags`: bit 0 = stream-input only, bit 7 (`0x80`) = runs as itself on the active source now |
| 2 | u16 | `latency_us` (nominal fill target) |
| 4 | u16 | `batch_us` (nominal stream-input batch) |
| 6 | u16 | `effective_us` (what selecting it gives now, source and rate applied) |

Spec: `current_architecture.md` section "Output Latency" (there is no `Features/` spec).

---

## 13. Bulk parameter transfer (read/write all state in one shot)

### 13.1 `REQ_GET_ALL_PARAMS` (0xA0, read)

*Last updated: 2026-11-15*

`bmRequestType=0xC1`, `wValue=0`. Request `wLength` up to `sizeof(WireBulkParams)`
= **6136 bytes** (wire format V32, the same size on both platforms). The firmware
collects the live DSP state at request time and returns `min(wLength, 6136)`
bytes via EP0 chunking (handle the 64-byte chunking and the trailing ZLP in your
USB layer).

### 13.2 `REQ_SET_ALL_PARAMS` (0xA1, write)

*Last updated: 2026-11-15*

`bmRequestType=0x41`, `wValue=0`. Send the full `WireBulkParams`, exactly 6136
bytes. The apply is **deferred** to the main loop and brackets a muted,
synchronized pipeline reset; expect a busy window afterward. Before applying, the
main loop checks every one of these and **silently drops** the payload (no status,
no notification) if any fails:

* `format_version` is exactly 32 and `payload_length` is exactly 6136;
* `platform_id` matches the device;
* `num_channels`, `num_input_channels` and `num_output_channels` match the device.

There is no backward compatibility. It was broken on purpose at V16 (unified
channel model), and every later version appended sections or claimed reserved
bytes, so a host must match the version exactly. A `0xA1` of 64 bytes or fewer is
ACKed and ignored; 65 to 6135 bytes STALLs at SETUP. Re-read `REQ_GET_ALL_PARAMS`
afterward to confirm the apply.

The `pins` and `limiter` sections are applied only when `output_config_mode` is
1 (with-preset). In independent mode they are ignored on SET.

**Chunked transfer (`0xA2` / `0xA3`).** WinUSB caps a control transfer at 4 KB, so
USB hosts can move the struct in pieces. `wValue` is the byte offset and `wLength`
the chunk size. `0xA2` at offset 0 takes the bulk lock and snapshots the whole
struct; later chunks read that snapshot. `0xA3` chunks must arrive in order from
offset 0, and the apply is armed when the received byte count reaches 6136. Any
other vendor request tears an open chunk session down. UART and I2C refuse both
(`CTRL_DISPATCH_BLOCKED`). See `Features/bulk_params_chunking.md`.

### 13.3 `WireBulkParams` layout (V32, 6136 bytes, little-endian, packed)

*Last updated: 2026-11-15 (offsets printed by compiling `bulk_params.h` on the host; they match the firmware's `_Static_assert` checks)*

All offsets are byte offsets from the start of the packet. Arrays are sized at the
**RP2350 maximum** on both platforms. Rows sized `[17]` (delays, EQ, channel names,
crossovers) are indexed by the firmware channel index (Section 2.1): on RP2040
rows 0-1 are inputs, rows 2-6 are outputs and rows 7-16 are zero. Crosspoints
always use the fixed `[8][9]` stride.

| Offset | Size | Section | Contents |
|-------:|-----:|---------|----------|
|-------:|-----:|---------|----------|
| 0 | 16 | `header` | `WireHeader`, see 13.4 |
| 16 | 16 | `global` | f32 preamp_gain_db (legacy baseline for all inputs; `preamp` overrides); u8 bypass; u8 loudness_enabled; u16 loudness_output_mask; f32 loudness_ref_spl; f32 loudness_intensity_pct |
| 32 | 16 | `crossfeed` | u8 enabled; u8 preset; u8 itd_enabled; u8 output_pair_mask; f32 custom_fc; f32 custom_feed_db; u32 reserved |
| 48 | 16 | `legacy` | Retired. f32 gain_db[3]; u8 mute[3]; u8 rsv. Always zero on GET, ignored on SET |
| 64 | 68 | `delays` | f32 delay_ms[17], per firmware channel index |
| 132 | 576 | `crosspoints` | `WireCrosspoint[8][9]` (input-major); 8 B each: u8 enabled; u8 phase_invert; u8[2] rsv; f32 gain_db |
| 708 | 108 | `outputs` | `WireOutputChannel[9]`; 12 B each: u8 enabled; u8 mute; u8[2] rsv; f32 gain_db; f32 delay_ms |
| 816 | 8 | `pins` | u8 num_pin_outputs (3 or 5); u8 pins[5]; u8[2] rsv |
| 824 | 3264 | `eq` | `WireBandParams[17][12]` (channel-major), see 13.5 |
| 4088 | 544 | `channel_names` | char[17][32], NUL-padded |
| 4632 | 16 | `i2s_config` | u8 output_types[4] (0 = S/PDIF, 1 = I2S); u8 bck_pin; u8 mck_pin; u8 mck_enabled; u8 mck_multiplier (0 = 128x, 1 = 256x); u8 clock_pin_mode_p1 (0 = absent, 1 = unified, 2 = split); u8 bck_pin_slave (0 = absent); u8[6] rsv |
| 4648 | 20 | `leveller` | u8 enabled; u8 speed; u8 lookahead; u8 rsv; f32 amount; f32 max_gain_db; f32 gate_threshold_db; u8 detector_mask; u8 apply_mask; u8[2] rsv |
| 4668 | 32 | `preamp` | f32 preamp_db[8], per input channel |
| 4700 | 16 | `master_volume` | f32 master_volume_db (-128 = mute); u8[12] rsv |
| 4716 | 16 | `input_config` | u8 input_source; u8 spdif_rx_pin; u8 i2s_rx_pin; u8 i2s_input_rate (0 = 44.1k, 1 = 48k, 2 = 96k); u8 i2s_input_channels (0 = absent); u8 i2s_rx_pin_ext[3]; u8 spdif_rx_pin_ext[3]; u8 spdif_rx_enabled_ext_p1; u8 i2s_clock_mode; u8 adat_input_pin (0 = absent); u8 adat_input_enabled_p1; u8 adat_clock_mode_p1. No reserved bytes left |
| 4732 | 16 | `lg_sound_sync` | u8 enabled; u8 present (ro); u8 volume (ro); u8 muted (ro); u8[12] rsv |
| 4748 | 16 | `user_volume` | f32 user_volume_db; u8 user_mute; u8[11] rsv |
| 4764 | 16 | `dac_hw_mute` | u8 enabled; u8 active_low; u8 pin; u8 rsv; u16 hold_ms; u16 release_ms; u8[8] rsv |
| 4780 | 1088 | `crossovers` | `WireBandParams[17][4]` (channel-major); columns map to vendor bands 20..23; input rows zero on GET and skipped on SET |
| 5868 | 8 | `adat_config` | u8 enabled; u8 pin (0 = default on SET); u8[6] rsv. Zero on RP2040 |
| 5876 | 24 | `psybass` | u8 enabled; u8 rsv; u16 output_mask; f32 cutoff_hz; f32 harmonics_db; f32 drive_db; f32 character_pct; f32 original_db |
| 5900 | 44 | `upmix` | u8 enabled; u8 center_mode; u8 surround_mode; i8 presence_q1 (dB x 2); f32 x 10 (strength, center width, correlation threshold, attack, release, detector HPF, surround delay, surround HPF, surround LPF, decorrelation). Zero on RP2040 |
| 5944 | 36 | `subharm` | u8 enabled; u8 rsv; u16 output_mask; f32 low_db; f32 high_db; f32 boost_db; f32 top_db; f32 select_depth; f32 select_hold_ms; f32 ceiling_db; u8 select_mode; u8 link_pairs; u8[2] rsv |
| 5980 | 48 | `tube` | u8 enabled; u8 tube_type; u8 rectifier; u8 xfmr_enabled; u16 output_mask; u8[2] rsv; f32 drive_db; f32 bias_pct; f32 asym_db; f32 hardness_pct; f32 sag_pct; f32 xfmr_damping; f32 xfmr_res_hz; f32 mix_pct; f32 trim_db; f32 rsv |
| 6028 | 108 | `limiter` | `WireLimiterOutput[9]`; 12 B each: u8 enabled; u8 link_group (0 = unlinked, 1..4); u8[2] rsv; f32 threshold_db; f32 release_ms |

Fields named `*_p1` hold the value plus one, so 0 means "absent, keep the live
value". On bulk SET, only `enabled` is honored in the `lg_sound_sync` section;
`present`/`volume`/`muted` are runtime-only and ignored.

### 13.4 `WireHeader` (16 bytes)

| Offset | Type | Field |
|--------|------|-------|
| 0 | u8 | `format_version` (currently 32; SET must match exactly) |
| 1 | u8 | `platform_id` (0 = RP2040, 1 = RP2350; SET must match the device) |
| 2 | u8 | `num_channels` (7 on RP2040, 17 on RP2350; SET must match) |
| 3 | u8 | `num_output_channels` (5 on RP2040, 9 on RP2350; SET must match) |
| 4 | u8 | `num_input_channels` (2 on RP2040, 8 on RP2350; SET must match) |
| 5 | u8 | `max_bands` (12) |
| 6 | u16 | `payload_length` (6136; SET must match) |
| 8 | u16 | `fw_version_major` (GET only) |
| 10 | u16 | `fw_version_minor` (GET only; the minor number, not minor.patch) |
| 12 | u32 | reserved (0) |

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
> `qp` arrived in `WIRE_FORMAT_VERSION` 22 (byte layout and struct size were
> unchanged). The current version is 32. Unlike the single-band command, the bulk bytes are authoritative:
> apply always overwrites the stored `qp` (for an LT band `0` selects the 0.707
> default; for any other type it is forced to `0`). For an LT band,
> `freq`/`q`/`gain_db` hold `f0`/`Q0`/`fp` (Hz).

---

## 14. Meter / status polling

These are the commands a remote UI polls for live metering. All are reads.

### 14.1 `REQ_GET_STATUS` (0x50, read): `wValue` selects the sub-query

*Last updated: 2026-11-15*

Every sub-query except `wValue=9` returns exactly **4 bytes** (a `u32` LE). The
transfer is clamped to `wLength`. An unsupported `wValue` in a release build
returns 4 zero bytes; it does not STALL.

| wValue | Response (4 bytes, u32 LE) |
|-------:|----------------------------|
|-------:|----------------------------|
| 0 | `peaks[0]` (low u16) \| `peaks[1]` (high u16): channels 0 and 1 (inputs 0/1 on both platforms) |
| 1 | `peaks[2]` \| `peaks[3]`: channels 2 and 3 (RP2040: outputs 0/1; RP2350: inputs 2/3) |
| 2 | `peaks[4]` (low u16) \| cpu0_load (byte 2) \| cpu1_load (byte 3) |
| 3 | `pdm_ring_overruns` |
| 4 | `pdm_ring_underruns` |
| 5 | `pdm_dma_overruns` |
| 6 | `pdm_dma_underruns` |
| 7 | Retired, always 0 |
| 8 | `spdif_underruns` |
| 9 | Combined packet, variable length (below) |
| 10 | `usb_audio_packets` |
| 11 | `usb_audio_alt_set` |
| 12 | `usb_audio_mounted` |
| 13 | `clk_sys` frequency, Hz |
| 14 | Core voltage, mV |
| 15 | Device sample rate, Hz |
| 16 | Temperature, centi-degrees C, as a sign-extended int32 |
| 17 | Total S/PDIF DMA starvations |
| 18 | S/PDIF instance 0 DMA starvations |
| 19 | S/PDIF instance 1 DMA starvations |
| 20 | S/PDIF instance 2 DMA starvations |
| 21 | S/PDIF instance 3 DMA starvations |
| 22 | USB audio ring overruns |
| 23 | Live active input channel count (source-aware: USB / I2S / S/PDIF / ADAT) |
| 24 | Loopback capture overflow count (`DSPI_LOOPBACK` builds only; 0 otherwise) |
| 25 | Loopback capture underrun count (`DSPI_LOOPBACK` builds only; 0 otherwise) |
| 26 | Output ring underrun episodes (`overruns`) |
| 27 | `output_ring_drops`: blocks refused by ring admission |
| 28 | `output_ring_min_prewrite`: lowest pre-write ring fill, frames (`0xFFFFFFFF` until first sample; reset by 0xB1 bit 0) |
| 29 | `input_delivery_worst_gap_us`: worst input delivery gap, us (reset by 0xB1 bit 0) |
| 30 | `usb_audio_worst_packet_gap_us`: worst USB packet gap, us (reset by 0xB1 bit 0) |
| 31 | `bg_over_budget_count`: background steps over budget |
| 32 | S/PDIF input measured rate, mHz (0 = none) |
| 33 | Soft VCXO trim, ppm x 100, signed int32 |

Peaks are linear, 0..32767 = 0..full scale. CPU loads are percent.
`DSPI_LOOP_PROF` debug builds route other `wValue`s to profiling counters; release
builds do not.

**Combined packet (`wValue=9`)** length = `NUM_CHANNELS*2 + 7` bytes (**21 on
RP2040**, **41 on RP2350**). Request at least that many bytes; a shorter
`wLength` truncates it.

| Offset | Type | Field |
|--------|------|-------|
| 0 | u16[NUM_CHANNELS] LE | `peaks[ch]` in channel-index order (inputs, then outputs) |
| 2N | u8 | `cpu0_load` (%) |
| 2N+1 | u8 | `cpu1_load` (%) |
| 2N+2 | u32 LE | `clip_flags`: sticky, bit k = channel k |
| 2N+6 | u8 | live active input channel count (same as `wValue = 23`) |

(N = `NUM_CHANNELS`.) `clip_flags` is a sticky per-channel clip bitmask.

### 14.2 Clip flags

| ID | Name | Dir | wValue | Response |
|----|------|-----|--------|----------|
| 0x83 | `CLEAR_CLIPS` | **write-as-read (0xC1)** | 0 | **4 bytes** = `u32` clip_flags that were set (then cleared); bit k = channel k |

Read-then-clear: returns the latched clip bitmask and resets it in one transfer.

### 14.3 Buffer statistics

| ID | Name | Dir | wValue | Response |
|----|------|-----|--------|----------|
| 0xB0 | `GET_BUFFER_STATS` | R | 0 | **44 bytes** `BufferStatsPacket` |
| 0xB1 | `RESET_BUFFER_STATS` | **write-as-read (0xC1)** | bit0=1 to reset watermarks | 1 byte (1) |
| 0xB2 | `GET_USB_ERROR_STATS` | R | 0 | **24 bytes** (all zero under TinyUSB) |
| 0xB3 | `RESET_USB_ERROR_STATS` | **write-as-read (0xC1)** | 0 | 1 byte (no-op) |

`BufferStatsPacket` (44 bytes). Outputs run from per-slot DMA rings, so the
per-slot fields describe ring fill (updated 2026-11-15).

| Offset | Type | Field |
|--------|------|-------|
| 0 | u8 | `num_spdif` (2 on RP2040, 4 on RP2350) |
| 1 | u8 | `flags`: bit 0 = PDM enabled, bit 1 = `sync_started` (USB stream started) |
| 2 | u16 | `sequence` (increments per read, wraps) |
| 4 | 8 x 4 | `spdif[4]`: one `SpdifBufferStats` per output slot (S/PDIF or I2S); slots beyond `num_spdif` are zero |
| 36 | 8 | `pdm` (`PdmBufferStats`); zero while PDM is disabled |

`SpdifBufferStats` (8 bytes), now describing each slot's output ring:

| Offset | Type | Field |
|--------|------|-------|
| 0 | u8 | `consumer_free`: 16 minus the fill in 48-frame buckets |
| 1 | u8 | `consumer_prepared`: ring fill in 48-frame buckets, 0..16 (768 frames full scale) |
| 2 | u8 | `consumer_playing`: 1 if the slot's ring DMA is enabled |
| 3 | u8 | `consumer_fill_pct`: (16 - free) x 100 / 16 |
| 4 | u8 | `consumer_min_fill_pct`: lowest fill % since the last 0xB1 reset |
| 5 | u8 | `consumer_max_fill_pct`: highest fill % since the last 0xB1 reset |
| 6 | u16 | `fill_centi_pct`: sample-granular fill in 0.01 % of 768 frames (5000 = 50.00 %); may exceed 10000; saturates at 65535; 0 when there is no valid reading. Was `pad[2]` |

During an output type switch a slot reads free = 16, prepared = 0, playing = 0.

`PdmBufferStats` is unchanged: u8 dma_fill_pct; u8 dma_min; u8 dma_max; u8 ring_fill_pct;
u8 ring_min; u8 ring_max; u8[2] pad.

`0xB1` `RESET_BUFFER_STATS` with `wValue` bit 0 set also resets `GET_STATUS`
sub-queries 28, 29 and 30, not only the fill watermarks.

`GET_USB_ERROR_STATS` returns a 24-byte struct of six u32 counters
(total, crc, bitstuff, rx_overflow, rx_timeout, data_seq); they are all zero on
current firmware (TinyUSB does not expose them).

### 14.4 Clock diagnostics (bench builds)

*Last updated: 2026-11-15*

| ID | Name | Dir | wValue | Payload / Response | Notes |
|----|------|-----|--------|--------------------|-------|
| 0x82 | `GET_CLOCK_DIAG` | R (0xC1) | 0 | `ClockDiagPacket`: 248 B on RP2350, 226 B on RP2040 (`version` = 2) | Only with `-DDSPI_CLOCK_DIAG=ON`. Release builds answer zero bytes (not a STALL). UART/I2C always get zero bytes because the packet exceeds their 132 B response buffer |

The packet is a once-per-second snapshot of sysclk, soft-VCXO, servo, per-slot ring, PDM, input and
USB feedback state. It is a bench instrument with no stability promise. The authoritative layout is
the packed struct in `firmware/DSPi/clock_diag.h`; `tools/clkdiag_poll.py` decodes it. No spec in
`Features/`.

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

*Last updated: 2026-11-15*

The device pushes change/event notifications on a **bulk IN endpoint `0x83`** on
the vendor interface (interface 2), 64-byte max packet. A USB host that wants live
push updates reads this endpoint; a bridge can either poll the readback commands
instead, or forward these packets. Every v2 packet starts `[0x02, event, flags,
seq]`. With nothing queued, a 1-byte idle packet `{0x00}` goes out only after
100 ms of quiet.

| Event | Name | Length | Layout |
|-------|------|-------:|--------|
| 0x00 | `IDLE` | 1 | `[0x00]` keep-alive; discard |
| 0x01 | `MASTER_VOLUME` (v1 legacy, USB only) | 8 | `[0x01, 0, 0, 0, f32 db]` |
| 0x02 | `PARAM_CHANGED` | 12 + size | `[2, 0x02, 0, seq, off u16, size u16, src, 0, 0, 0, value[size]]`; size <= 52 |
| 0x03 | `BULK_INVALIDATED` | 8 | `[2, 0x03, 0, seq, src, 0, 0, 0]` |
| 0x04 | `PRESET_LOADED` | 8 | `[2, 0x04, 0, seq, slot, 0, 0, 0]`; followed by 0x03 |
| 0x05 | `INPUT_FORMAT` | 8 | `[2, 0x05, 0, seq, channels, 0, 0, 0]`; active input channel count changed |
| 0x06 | reserved | - | batched PARAM_CHANGED, not implemented |
| 0x07 | `SIGGEN_STATE` | 8 | `[2, 0x07, 0, seq, state, reason, signal_type, channel]`; channel = walk channel or `0xFF` |
| 0x08 | `ADAT_STATE` (RP2350) | 8 | `[2, 0x08, 0, seq, enabled, active, pin, 0]` |
| 0x09 | `I2S_SLAVE_STATE` | 9 | `[2, 0x09, 0, seq, state, rate u32]`; state 0 = inactive, 1 = acquiring, 2 = relocking, 3 = locked; rate 0 until locked |
| 0x0A | `CS_IR_LEARN` | 12 | `[2, 0x0A, 0, seq, state, protocol, 0, 0, code u32]`; state 2 = done, 3 = timeout; protocol and code valid only for done |
| 0x0B | `ADAT_INPUT_STATE` (RP2350) | 10 | `[2, 0x0B, 0, seq, state, rate u32, clock_mode]`; state per `AdatInputStatusPacket`; rate 0 unless locked |
| 0x0C | `CS_AUX` | 9 | `[2, 0x0C, 0, seq, slot, state, level_q8 u16, src]`; level is 8.8 percent |
| 0x0D | `RTA_CONFIG` | 17 | `[2, 0x0D, 0, seq, RtaConfig (12 bytes), src]` |
| 0x0E | not emitted | - | held for the AC-3/DTS branch; not in this tree |
| 0x0F | `INPUT_SOURCE_NAME` | 40 | `[2, 0x0F, 0, seq, source_id, src, 0, 0, name (32)]`; name all zero when cleared |
| 0x10 | `DEVICE_NAME` | 40 | `[2, 0x10, 0, seq, 0, src, 0, 0, name (32)]` |

All multi-byte fields are little-endian. For `PARAM_CHANGED`, `off` is the **byte
offset into `WireBulkParams`** of the changed field and `size` is its length;
`value` is the new little-endian bytes. This lets a host dispatch on offset rather
than a per-command switch. `src` is the origin: 0=unknown, 1=host SET, 2=bulk SET,
3=preset, 4=factory, 5=GPIO / Control Surfaces, 6=internal, 7=UAC1, 8=UART,
9=I2C. On `BULK_INVALIDATED` or `PRESET_LOADED`, re-read `REQ_GET_ALL_PARAMS`.

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
| 0x54 SET_INPUT_SOURCE_NAME | 0x55 GET_INPUT_SOURCE_NAME (staged name reads back at once) |
| 0x56 SET_DEVICE_NAME | 0x57 GET_DEVICE_NAME (staged name reads back at once) |
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
| 0x03 OUTPUT_LATENCY (OUT) | 0x03 OUTPUT_LATENCY (IN, wValue 0; `pending` clears when applied) |
| 0x04 / 0x06 CS aux state / level | 0x05 / 0x07 |
| 0x08 RTA_SET_CONFIG | 0x09 RTA_GET_CONFIG |
| 0x10..0x1D, 0x2C, 0x2E, 0xA9/0xAB/0xAD subharm | the odd opcode one above each (0x11..0x1E, 0x2D, 0x2F, 0xAA/0xAC/0xAE) |
| 0x30..0x3C psybass | 0x31..0x3D |
| 0x3E SET_TUBE_PARAM | 0x3F GET_TUBE_PARAM (same index) |
| 0x4A / 0x4C upmix | 0x4B / 0x4D |
| 0x68 / 0x6A / 0x6C ADAT input | 0x69 / 0x6B / 0x6D |
| 0x81 LIMITER (OUT) | 0x81 LIMITER (IN, same wValue) |
| 0x88 SET_I2S_CLOCK_MODE | 0x89 GET_I2S_CLOCK_MODE (after apply) |
| 0xCA / 0xCC ADAT output | 0xCB / 0xCD |
| 0xDE SET_LEVELLER_MASKS | 0xDF GET_LEVELLER_MASKS |
| 0xE9 SET_SPDIF_INPUT_ENABLE | 0xEF GET_SPDIF_INPUT_CONFIG |
| 0xF3 SET_I2S_INPUT_CHANNELS | 0xF4 GET_I2S_INPUT_CHANNELS |
| 0xFC SET_CROSSFEED_OUTPUTS | 0xFD GET_CROSSFEED_OUTPUTS |
| 0xFE SET_I2S_CLOCK_PIN_MODE | 0xFF GET_I2S_CLOCK_PIN_MODE |
| Control Surfaces SETs (0x20, 0x22, 0x24, 0x27, 0x29, 0x84, 0x8B, 0x8D, 0x9D, 0x9E) | 0x87 GET_CS_STATUS (`last_status`), then the matching GET |

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
| 0x03 | `OUTPUT_LATENCY` (OUT) | Directory flash write plus a synchronized restart of every output |
| 0x68/0x6A/0x6C | ADAT input enable/pin/clock mode | Claims a GPIO; a live source restarts the receiver |
| 0xCA/0xCC | ADAT output enable/pin | Claims a GPIO; muted pipeline reset |
| 0x88 | `SET_I2S_CLOCK_MODE` | Rebuilds I2S clocking and restarts the I2S input when I2S is live |
| 0xE9 | `SET_SPDIF_INPUT_ENABLE` | Claims or releases an S/PDIF input GPIO |
| 0xF3 | `SET_I2S_INPUT_CHANNELS` | Claims I2S RX pins; restarts the I2S input when live |
| 0xFE | `SET_I2S_CLOCK_PIN_MODE` | Moves the live I2S clock pair in slave mode |
| 0x84/0x8D | CS binding / IR command | Claims GPIOs and PWM slices (preview until `0x9D` saves) |
| 0x9D | `CS_SAVE` | Flash write |

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

Every `REQ_*` vendor command (246 total, 2026-11-15) with the internal handler it maps to. The
**Internal mapping** column names the firmware function(s) and/or globals each command
touches. Many writes are *deferred*: the handler only sets a `*_pending` flag (or a
change bitmask) and `main.c`'s loop performs the real operation later, sometimes after
the pipeline-reset settle window (see section 1.3). Such rows read
`<pending_flag> -> <main.c operation>`. The deferred flags are summarized in section 21.1.

`Dir`: `W` = OUT/write (has a data stage); `R` = IN/read; `W-as-R` = an action or write
with no OUT data, issued as an IN transfer (returns a status/echo byte).

| Hex | Name | Dir | Internal mapping |
|-----|------|-----|------------------|
| 0x03 | OUTPUT_LATENCY | W+R | OUT: `output_latency_find()` validates; `flash_set_output_latency_id` + `flash_set_output_latency_pending` -> `preset_set_output_latency()` (directory flash) + `output_latency_request()` + `output_latency_apply()` + `dsp_update_delay_samples()` + `complete_flash_write_operation_full()` (synchronized output restart). IN wValue 0: `OutputLatencyStatus` from `output_latency_requested/effective()`, `output_ring_target_frames`, `input_batch_frames`; wValue 1: `output_latency_options[]` via `output_latency_resolve()` / `output_latency_target_frames()` |
| 0x04 | SET_CS_AUX_STATE | W | `control_surfaces_set_aux_state(slot)`; immediate, RAM only; `notify_push_cs_aux()` on change; rejects set `cs_last_status` |
| 0x05 | GET_CS_AUX_STATE | R | `control_surfaces_aux_up()` / `control_surfaces_aux_state()`; wValue 0xFFFF adds `control_surfaces_aux_level()` for all 16 slots (48 B) |
| 0x06 | SET_CS_AUX_LEVEL | W | `control_surfaces_set_aux_level(slot)` clamps to `cs_noun_table[CS_NOUN_AUX_LEVEL].max_q` (25600); immediate; `notify_push_cs_aux()` on change |
| 0x07 | GET_CS_AUX_LEVEL | R | `control_surfaces_aux_level(slot)` |
| 0x08 | RTA_SET_CONFIG | W | `rta_set_config()` validates + stages `rta.cfg_next`; `rta.cfg_pending` -> `rta_service()` -> `apply_config()` (clamps, restarts frame on tap/mask/order change, `notify_push_rta_config()`) |
| 0x09 | RTA_GET_CONFIG | R | `rta_get_config()` (applied `rta.cfg`) |
| 0x0A | RTA_GET_CAPS | R | wValue 0: `rta_get_caps()`; 1..: `rta_get_caps_centres()` |
| 0x0B | RTA_GET_BANDS | R | `rta_note_read()` (keepalive/auto-start) + `rta_get_band_frame(channel)` |
| 0x0C | RTA_GET_BINS | R | `rta_note_read()` + `rta_bin_frame()`; wValue = byte offset, length clamped to wLength |
| 0x0D | RTA_GET_STATUS | R | `rta_get_status()`; not a keepalive |
| 0x0E | RTA_CONTROL | W-as-R | `rta_control(wValue & 0xFF)`: 0 `stop_engine()`, 1 `arm()` if idle, 2 clear averages/peaks; returns 1; STALL on unknown action |
| 0x0F | RTA_GET_BANDS_ALL | R | USB only; bulk lock via `vendor_is_bulk_get()`; `rta_live_mask()` + `rta_get_band_frame()` per live channel into `bulk_param_buf` |
| 0x10 | SET_SUBHARM | W | writes `subharm_config.enabled`; `subharm_update_pending` -> `subharm_apply_config()` (publish `current_subharm_coeffs`; NULL = disabled) |
| 0x11 | GET_SUBHARM | R | returns `subharm_config.enabled` |
| 0x12 | SET_SUBHARM_LOW | W | clamp, write `subharm_config.low_db`; `subharm_update_pending` -> `subharm_apply_config()` |
| 0x13 | GET_SUBHARM_LOW | R | returns `subharm_config.low_db` |
| 0x14 | SET_SUBHARM_HIGH | W | clamp, write `subharm_config.high_db`; `subharm_update_pending` -> `subharm_apply_config()` |
| 0x15 | GET_SUBHARM_HIGH | R | returns `subharm_config.high_db` |
| 0x16 | SET_SUBHARM_BOOST | W | clamp, write `subharm_config.boost_db`; `subharm_update_pending` -> `subharm_apply_config()` |
| 0x17 | GET_SUBHARM_BOOST | R | returns `subharm_config.boost_db` |
| 0x18 | SET_SUBHARM_MASK | W | writes `subharm_config.output_mask`; immediate (read live per packet, no pending flag) |
| 0x19 | GET_SUBHARM_MASK | R | returns `subharm_config.output_mask` |
| 0x1A | GET_SUBHARM_HEADROOM | R | `subharm_headroom_db()` on a snapshot of `subharm_config` |
| 0x1B | SET_SUBHARM_TOP | W | clamp, write `subharm_config.top_db`; `subharm_update_pending` -> `subharm_apply_config()` |
| 0x1C | GET_SUBHARM_TOP | R | returns `subharm_config.top_db` |
| 0x1D | SET_SUBHARM_SELECT | W | clamp, write `subharm_config.select_mode`; `subharm_update_pending` -> `subharm_apply_config()` |
| 0x1E | GET_SUBHARM_SELECT | R | returns `subharm_config.select_mode` |
| 0x1F | GET_SUBHARM_METER | R | `subharm_meter_u16(k)` for each output (from `subharm_output_state[k].meter`) |
| 0x20 | SET_CS_GROUP | W | copy to `cs_set_group_val`/`cs_set_group_slot`, `cs_last_status`=PENDING; `cs_set_group_pending` -> `control_surfaces_apply_group()` (revalidates dependent bindings/IR commands), set dirty on success |
| 0x21 | GET_CS_GROUP | R | `control_surfaces_get_group(idx)` -> live `s_groups.groups[idx]` (40 B) |
| 0x22 | SET_CS_MACRO | W | copy to `cs_set_macro_hdr_val`/`_slot`; `cs_set_macro_hdr_pending` -> `control_surfaces_apply_macro_header()`, set dirty on success |
| 0x23 | GET_CS_MACRO | R | `control_surfaces_get_macro(idx)` -> live `s_macros.macros[idx]` (132 B) |
| 0x24 | SET_CS_MACRO_STEP | W | copy to `cs_set_macro_step_val`/`_slot`/`_idx`; `cs_set_macro_step_pending` -> `control_surfaces_apply_macro_step()`, set dirty on success |
| 0x25 | CS_MACRO_FIRE | W-as-R | synchronous `control_surfaces_macro_fire(idx)` or `control_surfaces_macro_cancel()` (sequencer `s_macro_run`); runs on the CS tick |
| 0x26 | GET_CS_EXT_STATUS | R | `control_surfaces_get_ext_status()` (limits, `s_group_status`, `s_macro_status`, `s_macro_run`/`s_macro_step`) |
| 0x27 | SET_CS_DISPLAY_CFG | W | copy to `cs_set_disp_cfg_val`; `cs_set_disp_cfg_pending` -> `control_surfaces_apply_display_cfg()` (`s_disp.cfg`, repaint), set dirty on success |
| 0x28 | GET_CS_DISPLAY_CFG | R | `{CS_MAX_DISPLAY_PAGES, CS_DISP_MODEL_COUNT, 0, 0}` + `control_surfaces_display_flash()->cfg` |
| 0x29 | SET_CS_DISPLAY_PAGE | W | copy to `cs_set_disp_page_val`/`_slot`; `cs_set_disp_page_pending` -> `control_surfaces_apply_display_page()`, set dirty on success |
| 0x2A | GET_CS_DISPLAY_PAGE | R | `control_surfaces_get_display_page(idx)` -> live `s_disp.pages[idx]` |
| 0x2B | GET_CS_DISPLAY_STATUS | R | `control_surfaces_get_display_status()` (`s_state`, current page, overlay/edit, `s_model`, `s_nak`) |
| 0x2C | SET_SUBHARM_SOLO | W | writes `subharm_config.solo`; immediate (read live via `subharm_flags_snapshot()`); no notify, never persisted |
| 0x2D | GET_SUBHARM_SOLO | R | returns `subharm_config.solo` |
| 0x2E | SET_SUBHARM_LINK | W | writes `subharm_config.link_pairs`; immediate (read live via `subharm_flags_snapshot()`) |
| 0x2F | GET_SUBHARM_LINK | R | returns `subharm_config.link_pairs` |
| 0x30 | SET_PSYBASS | W | writes `psybass_config.enabled`; `psybass_update_pending` -> `psybass_apply_config()` (publish `current_psybass_coeffs`; NULL = disabled) |
| 0x31 | GET_PSYBASS | R | returns `psybass_config.enabled` |
| 0x32 | SET_PSYBASS_CUTOFF | W | clamp, write `psybass_config.cutoff_hz`; `psybass_update_pending` -> `psybass_apply_config()` |
| 0x33 | GET_PSYBASS_CUTOFF | R | returns `psybass_config.cutoff_hz` |
| 0x34 | SET_PSYBASS_HARMONICS | W | clamp, write `psybass_config.harmonics_db`; `psybass_update_pending` -> `psybass_apply_config()` |
| 0x35 | GET_PSYBASS_HARMONICS | R | returns `psybass_config.harmonics_db` |
| 0x36 | SET_PSYBASS_DRIVE | W | clamp, write `psybass_config.drive_db`; `psybass_update_pending` -> `psybass_apply_config()` |
| 0x37 | GET_PSYBASS_DRIVE | R | returns `psybass_config.drive_db` |
| 0x38 | SET_PSYBASS_CHARACTER | W | clamp, write `psybass_config.character_pct`; `psybass_update_pending` -> `psybass_apply_config()` |
| 0x39 | GET_PSYBASS_CHARACTER | R | returns `psybass_config.character_pct` |
| 0x3A | SET_PSYBASS_ORIGINAL | W | clamp, write `psybass_config.original_db`; `psybass_update_pending` -> `psybass_apply_config()` |
| 0x3B | GET_PSYBASS_ORIGINAL | R | returns `psybass_config.original_db` |
| 0x3C | SET_PSYBASS_MASK | W | writes `psybass_config.output_mask`; immediate (read live per packet) |
| 0x3D | GET_PSYBASS_MASK | R | returns `psybass_config.output_mask` |
| 0x3E | SET_TUBE_PARAM | W | `tube_set_param(wValue & 0xFF, f32)` (clamps, notifies, loads tube rows into `tube_config`); `tube_update_pending` -> `tube_apply_config()` (not raised for index 1, the mask) |
| 0x3F | GET_TUBE_PARAM | R | `tube_get_param(wValue & 0xFF)` from `tube_config`; STALL on bad index |
| 0x42 | SET_EQ_PARAM | W | copy payload to `pending_packet`, validate ch/band; `eq_update_pending` -> write `filter_recipes`/`xover_recipes` + `dsp_compute_coefficients()` / `xover_design_filter()` |
| 0x43 | GET_EQ_PARAM | R | read `filter_recipes[ch][band]` / `xover_recipes[ch][band-20]`; wValue packs ch/band/param(0..4); returns 4-byte scalar |
| 0x44 | SET_PREAMP | W | legacy: `update_preamp(ch, db)` for all input channels (dB->linear->Q28) |
| 0x45 | GET_PREAMP | R | returns `global_preamp_db[0]` |
| 0x46 | SET_BYPASS | W | writes `bypass_master_eq` |
| 0x47 | GET_BYPASS | R | returns `bypass_master_eq` |
| 0x48 | SET_DELAY | W | writes `channel_delays_ms[ch]`; immediate `dsp_update_delay_samples()` |
| 0x49 | GET_DELAY | R | returns `channel_delays_ms[ch]` |
| 0x4A | UPMIX_SET_CONFIG | W | RP2350: decode `UpmixConfigPacket` into `upmix_config`, `upmix_notify_config()`; `upmix_update_pending` -> `upmix_apply_config()` (publish `current_upmix_coeffs`). RP2040: refused |
| 0x4B | UPMIX_GET_CONFIG | R | RP2350: `upmix_config_to_packet()`. RP2040: 44 zero bytes |
| 0x4C | UPMIX_SET_PARAM | W | RP2350: write one `upmix_config` field by id, `upmix_notify_config()`; `upmix_update_pending` -> `upmix_apply_config()`. RP2040: refused |
| 0x4D | UPMIX_GET_PARAM | R | RP2350: one `upmix_config` field by id; STALL on bad id. RP2040: 0.0 |
| 0x4E | UPMIX_GET_STATUS | R | RP2350: `upmix_get_status()` (live engine state). RP2040: 16 zero bytes |
| 0x50 | GET_STATUS | R | wValue selects scalar/combined meters: `global_status` peaks/CPU/clip, PDM/SPDIF counters, `clock_get_hz`, vreg, `audio_state.freq`, temperature, DMA/ring stats |
| 0x51 | SAVE_PARAMS | W-as-R | legacy: `save_params_pending` -> `flash_save_params()` (flash bracket; not gated) |
| 0x52 | SAVE_OUTPUT_CONFIG | W-as-R | `flash_save_output_config_pending` -> `preset_save_output_config()` (flash; not gated) |
| 0x53 | FACTORY_RESET | W-as-R | `factory_reset_pending` -> `flash_factory_reset()` + `dsp_recalculate_all_filters()` + delay/Core1/type resets (gated) |
| 0x54 | SET_INPUT_SOURCE_NAME | W | `input_source_name_stage()` -> `input_source_names_apply_pending()` -> `preset_set_input_source_names()` (flash bracket; one write for every staged source) |
| 0x55 | GET_INPUT_SOURCE_NAME | R | returns the staged name if pending, else `dir_cache.input_source_names[src]` |
| 0x56 | SET_DEVICE_NAME | W | `device_name_stage()` -> the same `input_source_names_apply_pending()` pass -> `preset_set_input_source_names(..., device_name)` (one write with any staged source names) |
| 0x57 | GET_DEVICE_NAME | R | returns the staged name if pending, else `dir_cache.device_name` |
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
| 0x68 | SET_ADAT_INPUT_ENABLE | W-as-R | RP2350: validates `adat_input_pin`, writes `adat_input_enabled` (immediate; gates `input_source_selectable()` and pin ownership); disable refused while ADAT active/pending; RP2040 returns `0x03` |
| 0x69 | GET_ADAT_INPUT_ENABLE | R | returns `adat_input_enabled` |
| 0x6A | SET_ADAT_INPUT_PIN | W-as-R | RP2350: writes `adat_input_pin` (`0xFF` clears, only while disabled); if ADAT is the active source `adat_input_restart_pending` -> `adat_input_stop()` / `adat_input_start()` under mute (not gated); RP2040 returns `0x03` |
| 0x6B | GET_ADAT_INPUT_PIN | R | returns `adat_input_pin` (`0xFF` = unset) |
| 0x6C | SET_ADAT_INPUT_CLOCK_MODE | W-as-R | `pending_adat_clock_mode` + `adat_clock_mode_change_pending` -> dormant: adopt mode at once; ADAT active: receiver restart in the new mode + rate restore (gated) |
| 0x6D | GET_ADAT_INPUT_CLOCK_MODE | R | returns live `adat_clock_mode` |
| 0x6E | GET_ADAT_INPUT_STATUS | R | `adat_input_get_status()` -> 20-byte `AdatInputStatusPacket` (zeros on RP2040) |
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
| 0x7F | GET_PLATFORM | R | returns 7 bytes {platform id, FW major, legacy minor.patch nibbles, NUM_OUTPUT_CHANNELS, FW minor, FW patch, `FW_VERSION_BETA`} |
| 0x80 | GET_BUILD_INFO | R | returns 64-byte build stamp: git describe [0..47] + build date [48..59]; informational only |
| 0x81 | LIMITER | W+R | OUT: `limiter_set_param(output, index, float)` (clamp, group ganging, `notify_param_write`), `limiter_update_pending` -> `limiter_apply_config()` + `limiter_engage_service()`. IN: index 0-3 `limiter_get_param()`; 0x80 `limiter_meter_centidb()` per output; 0x81 `limiter_is_engaged()`, `LIMITER_DELAY`, `LIMITER_BLOCK`, `NUM_OUTPUT_CHANNELS` |
| 0x82 | GET_CLOCK_DIAG | R | `DSPI_CLOCK_DIAG` builds: `clock_diag_get_snapshot()` (1 Hz latched `ClockDiagPacket`); otherwise zero-length reply |
| 0x83 | CLEAR_CLIPS | W-as-R | read-and-clear: returns the 4-byte `global_status.clip_flags`, then zeroes it |
| 0x84 | SET_CS_BINDING | W | copy to `cs_set_binding_val`/`_slot`; `cs_set_binding_pending` -> `control_surfaces_apply_binding()` (release old pins, validate, claim, seed runtime), set dirty on success |
| 0x85 | GET_CS_BINDING | R | `control_surfaces_get_binding(slot)` -> live `s_cfg.bindings[slot]` (stored copy via `preset_stored_cs_config()` during a revert) |
| 0x86 | GET_CS_CAPS | R | `control_surfaces_caps_header()` (`s_caps`, 52 B) or `control_surfaces_noun_desc(noun)` (`cs_noun_table[]`, 12 B) |
| 0x87 | GET_CS_STATUS | R | `control_surfaces_get_status()` (`cs_last_status`/`cs_last_slot`, `s_dirty`, active masks, per-slot status; `s_rv_status` snapshot during a revert) |
| 0x88 | SET_I2S_CLOCK_MODE | W | `pending_i2s_clock_mode` + `i2s_clock_mode_change_pending` -> main loop: dormant store if source is not I2S, else (gated on `pipeline_reset_ready()`) `prepare_pipeline_reset()` + `i2s_input_stop()` + `rebuild_i2s_output_clocking()` + MCK policy + `i2s_input_start()`; `notify_param_write` at apply |
| 0x89 | GET_I2S_CLOCK_MODE | R | returns live `i2s_clock_mode` |
| 0x8A | GET_I2S_SLAVE_STATUS | R | `i2s_slave_get_status()` -> `I2sSlaveStatusPacket` |
| 0x8B | SET_CS_NAME | W | copy to `cs_set_name_val`/`_slot`; `cs_set_name_pending` -> `control_surfaces_apply_name()` (`s_names[slot]`), set dirty on success |
| 0x8C | GET_CS_NAME | R | `control_surfaces_get_name(slot)` -> live `s_names[slot]` (32 B) |
| 0x8D | SET_CS_IR_CMD | W | copy to `cs_set_ir_cmd_val`/`_slot`; `cs_set_ir_cmd_pending` -> `control_surfaces_apply_ir_cmd()`, set dirty on success |
| 0x8E | GET_CS_IR_CMD | R | `control_surfaces_get_ir_cmd(sub)` -> live `s_ir.cmds[sub]` (stored copy during a revert) |
| 0x8F | CS_IR_LEARN | W-as-R | wValue 0/1: synchronous `control_surfaces_ir_learn_control()` -> `cs_ir_learn_arm()`/`cs_ir_learn_cancel()`; wValue 2: `control_surfaces_get_ir_learn()`; completion -> `notify_push_cs_ir_learn()` (event 0x0A) from the CS tick |
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
| 0x9D | CS_SAVE | W-as-R | `cs_save_pending` -> `control_surfaces_aux_prepare_save()` + `preset_set_cs_all(config, ir, names, groups, macros, display)` (flash bracket), clear dirty on success; not gated |
| 0x9E | CS_REVERT | W-as-R | `cs_revert_pending` -> `control_surfaces_revert_begin()` -> background job `cs_revert_step()` (bg_job `BG_JOB_CS_REVERT`); clears dirty and posts SUCCESS on completion; not gated |
| 0xA0 | GET_ALL_PARAMS | R | `bulk_params_collect()` -> chunked `WireBulkParams` |
| 0xA1 | SET_ALL_PARAMS | W | exactly 6136 bytes -> `bulk_param_buf` (EP0 chunked); `bulk_params_pending` -> version/platform/count checks, then `bulk_params_apply()` + `dsp_recalculate_all_filters()` + reset (gated); a failed check drops it silently |
| 0xA2 | GET_ALL_PARAMS_CHUNK | R | wValue = byte offset; offset 0 snapshots under the bulk lock; USB-only (WinUSB 4 KB cap; see `Features/bulk_params_chunking.md`) |
| 0xA3 | SET_ALL_PARAMS_CHUNK | W | wValue = byte offset, sequential from 0 into `bulk_param_buf`; final byte sets `bulk_params_pending`; USB-only |
| 0xA4 | SIGGEN_SET_CONFIG | W | payload = `SiggenConfig` (36 B) -> `siggen_stage_config()` (validate + stage; restarts if running, never auto-starts); STALL on reject. See section 23 |
| 0xA5 | SIGGEN_GET_CONFIG | R | returns applied `SiggenConfig` (36 B) |
| 0xA6 | SIGGEN_CONTROL | W-as-R | wValue = `SIGGEN_CTL_*` (0 stop, 1 start, 2 stop-now) -> `siggen_control()`; returns 1-byte ack; STALL on reject |
| 0xA7 | SIGGEN_GET_STATUS | R | returns `SiggenStatus` (16 B) |
| 0xA8 | SIGGEN_GET_CAPS | R | wValue = 0xFFFF -> `SiggenCapsHeader` (8 B); wValue = type index -> `SiggenTypeDesc` (62 B) |
| 0xA9 | SET_SUBHARM_DEPTH | W | clamp 0..100, write `subharm_config.select_depth`; `subharm_update_pending` -> `subharm_apply_config()` |
| 0xAA | GET_SUBHARM_DEPTH | R | returns `subharm_config.select_depth` |
| 0xAB | SET_SUBHARM_HOLD | W | clamp 50..400, write `subharm_config.select_hold_ms`; `subharm_update_pending` -> `subharm_apply_config()` |
| 0xAC | GET_SUBHARM_HOLD | R | returns `subharm_config.select_hold_ms` |
| 0xAD | SET_SUBHARM_CEILING | W | clamp -40..0, write `subharm_config.ceiling_db`; `subharm_update_pending` -> `subharm_apply_config()` |
| 0xAE | GET_SUBHARM_CEILING | R | returns `subharm_config.ceiling_db` |
| 0xB0 | GET_BUFFER_STATS | R | `BufferStatsPacket` from per-slot output ring fill / watermarks / PDM fill |
| 0xB1 | RESET_BUFFER_STATS | W-as-R | `reset_buffer_watermarks()` if wValue&1 (also resets GET_STATUS 28-30) |
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
| 0xC2 | SET_I2S_BCK_PIN | W-as-R | wValue = (role << 8) \| GPIO; writes `i2s_bck_pin` (role 0) or `i2s_bck_pin_slave` (role 1); if I2S input active `i2s_input_restart_pending` -> I2S restart |
| 0xC3 | GET_I2S_BCK_PIN | R | wValue = role; returns `i2s_bck_pin` or `i2s_bck_pin_slave` (bad role: 0) |
| 0xC4 | SET_MCK_ENABLE | W-as-R | immediate `audio_i2s_mck_update_frequency()` + `audio_i2s_mck_set_enabled()`; writes `i2s_mck_enabled` |
| 0xC5 | GET_MCK_ENABLE | R | returns `i2s_mck_enabled` |
| 0xC6 | SET_MCK_PIN | W-as-R | immediate `audio_i2s_mck_change_pin()`; writes `i2s_mck_pin` |
| 0xC7 | GET_MCK_PIN | R | returns `i2s_mck_pin` |
| 0xC8 | SET_MCK_MULTIPLIER | W-as-R | writes `i2s_mck_multiplier`; if enabled `audio_i2s_mck_update_frequency()` |
| 0xC9 | GET_MCK_MULTIPLIER | R | returns `mck_encode(i2s_mck_multiplier)` (0/1) |
| 0xCA | SET_ADAT_ENABLE | W-as-R | RP2350: `adat_output_set_config(enable, pin)` sets `adat_output_config_dirty` -> `prepare_pipeline_reset()` + `complete_pipeline_reset()` (gated); RP2040 returns `0x03` |
| 0xCB | GET_ADAT_ENABLE | R | returns `adat_output_config_enabled()` (0 on RP2040) |
| 0xCC | SET_ADAT_PIN | W-as-R | RP2350: `adat_output_set_config(enabled, pin)` (`0xFF` -> `PICO_ADAT_PIN`) sets `adat_output_config_dirty` -> muted pipeline reset (gated); RP2040 returns `0x03` |
| 0xCD | GET_ADAT_PIN | R | returns `adat_output_pin()` (0 on RP2040) |
| 0xCE | GET_ADAT_STATUS | R | `adat_output_get_status()` -> 8-byte `AdatStatus` (zeros on RP2040) |
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
| 0xDE | SET_LEVELLER_MASKS | W | writes `leveller_config.detector_mask` / `apply_mask`; `leveller_update_pending` -> `leveller_compute_coefficients()` (no `leveller_reset_pending`) |
| 0xDF | GET_LEVELLER_MASKS | R | returns `leveller_config.detector_mask`, `apply_mask` |
| 0xE0 | SET_INPUT_SOURCE | W | `input_source_selectable()` check; `pending_input_source` + `input_source_change_pending` -> stop old / start new source + rate/MCK (gated) |
| 0xE1 | GET_INPUT_SOURCE | R | returns `active_input_source` |
| 0xE2 | GET_SPDIF_RX_STATUS | R | `spdif_input_get_status()` -> 16-byte `SpdifRxStatusPacket` |
| 0xE3 | GET_SPDIF_RX_CH_STATUS | R | `spdif_input_get_channel_status()` -> 24-byte IEC 60958 channel status |
| 0xE4 | SET_SPDIF_RX_PIN | W-as-R | wValue = (index << 8) \| GPIO; writes `spdif_rx_pin` (index 0) or `spdif_rx_pin_ext[index-1]`; if that input is active `spdif_rx_pin_change_pending` -> stop/restart RX on new pin (not gated) |
| 0xE5 | GET_SPDIF_RX_PIN | R | wValue = index; returns `spdif_rx_pin_for_index(index)` (index >= 4: 0) |
| 0xE6 | SET_LG_SOUND_SYNC_ENABLE | W | `lg_sound_sync_set_enabled(bool)` immediately |
| 0xE7 | GET_LG_SOUND_SYNC_ENABLE | R | returns `lg_sound_sync_get_enabled()` |
| 0xE8 | GET_LG_SOUND_SYNC_STATUS | R | `lg_sound_sync_get_status()` -> 16-byte `LgSoundSyncStatus` |
| 0xE9 | SET_SPDIF_INPUT_ENABLE | W-as-R | `spdif_input_enable_acceptable()` / active-or-pending source check; writes `spdif_rx_enabled_ext` (RAM; persisted on preset save); `notify_param_write` on change; returns `PIN_CONFIG_*` |
| 0xEA | SET_DAC_HW_MUTE_CONFIG | W | `flash_set_dac_hw_mute_val` + `flash_set_dac_hw_mute_pending` -> `dac_hw_mute_set_config()` |
| 0xEB | GET_DAC_HW_MUTE_CONFIG | R | `dac_hw_mute_get_config()` -> 16-byte `DacHwMuteConfig` |
| 0xEC | TEST_DAC_HW_MUTE | W-as-R | if enabled: `dac_hw_mute_test_pending` -> `dac_hw_mute_test_start()` (~1s async pulse) |
| 0xED | SET_INPUT_RATE | W | validate -> `i2s_input_rate`; if I2S active and differs: `pending_rate` + `rate_change_pending` -> `perform_rate_change()` (gated) |
| 0xEE | GET_INPUT_RATE | R | returns 2x u32 {`audio_state.freq`, `i2s_input_rate`} |
| 0xEF | GET_SPDIF_INPUT_CONFIG | R | `SPDIF_RX_NUM_INPUTS`, `(spdif_rx_enabled_ext << 1) \| 1`, `spdif_rx_pin_for_index(0..3)` |
| 0xF0 | ENTER_BOOTLOADER | W-as-R | returns 1, then `reset_usb_boot(0,0)` (never returns) |
| 0xF1 | SET_I2S_RX_PIN | W-as-R | wValue = (pair << 8) \| GPIO; `check_i2s_rx_pin()`; writes `i2s_rx_pin[pair]`; if I2S active `i2s_rx_pin_change_pending` -> I2S stop/start on new pin (gated) |
| 0xF2 | GET_I2S_RX_PIN | R | wValue = pair; returns `i2s_rx_pin[pair]` (bad pair: 0) |
| 0xF3 | SET_I2S_INPUT_CHANNELS | W-as-R | `check_i2s_rx_pin()` for each newly active pair; writes `i2s_input_channels`; if I2S active `i2s_input_restart_pending` -> main-loop I2S input restart + `notify_push_input_format()`; `notify_param_write`; returns `PIN_CONFIG_*` |
| 0xF4 | GET_I2S_INPUT_CHANNELS | R | returns `i2s_input_channels` |
| 0xF5 | SET_UART_CONFIG | W | **USB only** (BLOCKED over UART/I2C); `ctrl_set_uart_pending` -> main loop `uart_ctrl_apply()`, persist directory on success; returns `PIN_CONFIG_*` |
| 0xF6 | GET_UART_CONFIG | R | returns persisted 8-byte `UartCtrlConfig` |
| 0xF7 | SET_I2C_CONFIG | W | **USB only** (BLOCKED over UART/I2C); `ctrl_set_i2c_pending` -> main loop `i2c_ctrl_apply()`, persist directory on success; returns `PIN_CONFIG_*` |
| 0xF8 | GET_I2C_CONFIG | R | returns persisted 8-byte `I2cCtrlConfig` |
| 0xF9 | GET_CTRL_IFACE_STATUS | R | returns 8-byte `CtrlIfaceStatus` (uart/i2c last_status + live flags, proto_version=1) |
| 0xFA | SET_LOUDNESS_MASK | W | writes `loudness_output_mask` (u16 LE, bit k = output k); no recompute; applies next packet |
| 0xFB | GET_LOUDNESS_MASK | R | returns `loudness_output_mask` (u16 LE) |
| 0xFC | SET_CROSSFEED_OUTPUTS | W | writes `crossfeed_config.output_pair_mask` (masked to `NUM_SPDIF_INSTANCES` bits); read live by the pipeline, no pending flag; `notify_param_write` |
| 0xFD | GET_CROSSFEED_OUTPUTS | R | returns `crossfeed_config.output_pair_mask` |
| 0xFE | SET_I2S_CLOCK_PIN_MODE | W-as-R | validates against `output_types[]`, effective clock mode, slave-pair overlap and `is_pin_in_use()`; writes `i2s_clock_pin_mode` synchronously; if the live pair moves with I2S active, `i2s_input_restart_pending` -> main-loop input restart; `notify_param_write`; returns `PIN_CONFIG_*` |
| 0xFF | GET_I2S_CLOCK_PIN_MODE | R | returns `i2s_clock_pin_mode` |

> The Microsoft OS 2.0 vendor code (`0x01`) is intercepted earlier in
> `tud_vendor_control_xfer_cb` for descriptor delivery and is not an application opcode.

### 21.1 Deferred pending-flag legend

Each flag below is set by a handler and consumed in `main.c`'s main loop. "Gated" means
the block runs only when `pipeline_reset_ready()` is true (the non-blocking DAC
hardware-mute settle hold has elapsed); "not gated" runs as soon as the loop reaches it.

| Pending flag | main.c operation | Gated |
|---|---|---|
| `eq_update_pending` | write `filter_recipes`/`xover_recipes` from `pending_packet` (Core 1 fence if EQ-worker output), then `dsp_compute_coefficients()` + `channel_bypassed` recompute (PEQ) or `xover_design_filter()` + `xover_update_output_bypass()` on the output index (crossover) | no |
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
| `psybass_update_pending` | `psybass_apply_config()` (recompute into the inactive coeff buffer, publish `current_psybass_coeffs`; NULL = disabled) | no |
| `subharm_update_pending` | `subharm_apply_config()` (same publish model, `current_subharm_coeffs`) | no |
| `tube_update_pending` | `tube_apply_config()` (same publish model, `current_tube_coeffs`) | no |
| `upmix_update_pending` | RP2350 only: `upmix_apply_config()` (same publish model, `current_upmix_coeffs`) | no |
| `cs_set_binding_pending` | `control_surfaces_apply_binding()`; result to `cs_last_status`; dirty on success | no (held during revert) |
| `cs_set_name_pending` | `control_surfaces_apply_name()` | no (held during revert) |
| `cs_set_ir_cmd_pending` | `control_surfaces_apply_ir_cmd()` | no (held during revert) |
| `cs_set_group_pending` | `control_surfaces_apply_group()` | no (held during revert) |
| `cs_set_macro_hdr_pending` / `cs_set_macro_step_pending` | `control_surfaces_apply_macro_header()` / `_step()` | no (held during revert) |
| `cs_set_disp_cfg_pending` / `cs_set_disp_page_pending` | `control_surfaces_apply_display_cfg()` / `_page()` | no (held during revert) |
| `cs_save_pending` | `control_surfaces_aux_prepare_save()`, `preset_set_cs_all()` inside the flash bracket | no |
| `cs_revert_pending` | `control_surfaces_revert_begin()` (background job) | no |
| `flash_set_output_latency_pending` (+ `flash_set_output_latency_id`) | `preset_set_output_latency()` (directory flash), `output_latency_request()` + `output_latency_apply()`, `dsp_update_delay_samples()`, then `complete_flash_write_operation_full()` (synchronized restart of every output); stays set until done so `0x03` reports `pending` | no |
| `limiter_update_pending` | `limiter_apply_config()`. Separately, `limiter_engage_service()` runs every pass and engages or releases the lookahead only in silence | no |
| `rta.cfg_pending` (`rta.cfg_next`) | `rta_service()` -> `apply_config()` (clamp, restart the frame on tap/mask/order change, `notify_push_rta_config()`) | no |
| `i2s_clock_mode_change_pending` (+ `pending_i2s_clock_mode`) | source not I2S: store the mode. I2S active: `prepare_pipeline_reset()`, `i2s_input_stop()`, `rebuild_i2s_output_clocking()`, MCK policy, `i2s_input_start()` | yes (I2S active only) |
| `adat_clock_mode_change_pending` (+ `pending_adat_clock_mode`) | source not ADAT: adopt the mode. ADAT active: receiver restart under mute in the new mode, rate restore on return to master | yes (ADAT active only) |
| `adat_input_restart_pending` | if ADAT is the active source: fade, `adat_input_stop()`, `adat_input_start()` on the new pin | no |
| `adat_output_config_dirty` | `prepare_pipeline_reset()` + ADAT output reconfigure + `complete_pipeline_reset()` | yes |

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

* **Transport.** OUT write (`0x41`), `wValue=0`. Payload is a full `WireBulkParams`
  packet. `WIRE_BULK_PARAMS_MIN_SIZE` equals `sizeof(WireBulkParams)` (6136 bytes at
  V32), so `wLength` must be exactly 6136; the firmware receives it into
  `bulk_param_buf` via EP0 64-byte chunking. It is **not version tolerant**: the
  deferred apply drops the packet silently unless `format_version` is 32,
  `payload_length` is 6136, and the platform and channel counts match the device
  (Section 13.2). Updated 2026-11-15.
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
`0xA9`-`0xAE` belong to the subharmonic synthesizer (Section 25.2) and `0xAF`
is unused. All five commands are also reachable over the UART and
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

---

## 24. Spectrum analyser (RTA)

*Last updated: 2026-11-15*

Available on both platforms. The analyser is transient: idle at boot, never persisted, and it stops
itself 5000 ms after the last band or bin read unless `RTA_FLAG_MANUAL` is set. `0x0B`, `0x0C` and
`0x0F` count as reads: they refresh that timer and auto-start the engine unless MANUAL is set.
`0x0D` does not.

| ID | Name | Dir | wValue | Payload / Response | Notes |
|----|------|-----|--------|--------------------|-------|
| 0x08 | `RTA_SET_CONFIG` | W (0x41) | 0 | `RtaConfig`, exactly 12 B | Staged, applied by the main loop. Rejected on wrong length, `version != 3`, tap not 0/1, `fft_order` outside 8..10, or no mask bit inside the tap's width |
| 0x09 | `RTA_GET_CONFIG` | R (0xC1) | 0 | `RtaConfig` (12 B) | Applied config, not the staged one |
| 0x0A | `RTA_GET_CAPS` | R (0xC1) | 0, or chunk 1..n | 0: `RtaCaps` (16 B). n: up to 32 u16 LE band-centre Hz, starting at band `(n-1)*32` | 37 bands today: chunk 1 = 64 B, chunk 2 = 10 B. A chunk past the table or wValue > 0xFF STALLs |
| 0x0B | `RTA_GET_BANDS` | R (0xC1) | channel | `RtaBandFrame` (82 B) | Channel at the applied tap. STALLs if channel >= tap width or wValue > 0xFF |
| 0x0C | `RTA_GET_BINS` | R (0xC1) | byte offset | `min(frame_len - offset, wLength)` bytes of the bin frame | STALLs if offset >= frame length. Frame = 16 B header + `n_bins` + 1 seq byte |
| 0x0D | `RTA_GET_STATUS` | R (0xC1) | 0 | `RtaStatus` (24 B) | Not a read for auto-off |
| 0x0E | `RTA_CONTROL` | write-as-read (0xC1) | action | 1 byte, always `1` | 0 = stop, 1 = start (if idle), 2 = reset averages and peaks. Other actions STALL. Immediate |
| 0x0F | `RTA_GET_BANDS_ALL` | R (0xC1) | 0 | 82 B `RtaBandFrame` per live channel, back to back, clamped to wLength | USB only (ERROR on UART/I2C). Served from the bulk buffer under the bulk lock; STALLs if another transport holds it. No live channel = zero-length data stage, not a STALL |

Frame length for `0x0C` is 145, 273 or 529 bytes for `fft_order` 8, 9 or 10. Read the header's `seq`
(offset 2) and the frame's final byte; if they differ the frame turned over between chunks, so
re-read. Over UART/I2C a chunk is capped at 132 bytes by the transport. A successful `0x08` apply
always pushes `NOTIFY_EVT_RTA_CONFIG` (0x0D), even if nothing changed.

Level bytes (bands and bins): 0.5 dB steps, 243 = 0 dBFS, 255 = +6 dBFS, 0 = floor.

`RtaConfig` (12 bytes):

| Offset | Type | Field |
|--------|------|-------|
| 0 | u8 | `version` (must be 3) |
| 1 | u8 | `tap`: 0 = input (after per-input PEQ), 1 = output (after gain and delay) |
| 2 | u16 | `channel_mask` (bit i = channel i at that tap; masked to the tap's width on apply) |
| 4 | u8 | `fft_order` (8..10; default 10 on RP2350, 9 on RP2040) |
| 5 | u8 | reserved (forced 0) |
| 6 | u16 | `avg_ms` (clamped to 0..10000; 0 = no extra averaging) |
| 8 | u8 | `peak_decay_db_s` (clamped to 0..100; 0 = peak hold off) |
| 9 | u8 | `flags`: bit 0 `RTA_FLAG_MANUAL` (no auto-start, no auto-off) |
| 10 | u16 | reserved (forced 0) |

`RtaCaps` (16 bytes):

| Offset | Type | Field |
|--------|------|-------|
| 0 | u8 | `version` (3) |
| 1 | u8 | `input_channels` (8 RP2350, 2 RP2040) |
| 2 | u8 | `output_channels` (9 RP2350, 5 RP2040) |
| 3 | u8 | `fft_order_min` (8) |
| 4 | u8 | `fft_order_max` (10) |
| 5 | u8 | `fft_order_default` (10 RP2350, 9 RP2040) |
| 6 | u8 | `bass_bands` (14; continuous bands starting at index 0) |
| 7 | u8 | `max_bands` (37) |
| 8 | u8 | `level_zero` (243) |
| 9 | u8 | `dynamic_range_db` (120 RP2350, 78 RP2040) |
| 10 | u16 | `idle_timeout_ms` (5000) |
| 12 | u16 | `max_bin_frame` (529) |
| 14 | u16 | `bass_dynamic_range_db` (70) |

`RtaBandFrame` (82 bytes):

| Offset | Type | Field |
|--------|------|-------|
| 0 | u8 | `version` (3) |
| 1 | u8 | `channel` |
| 2 | u8 | `seq` (per-channel frame counter) |
| 3 | u8 | `n_bands` (bands valid at this rate; 0 if the rate has no band table) |
| 4 | u16 | `age_ms` since the last frame (0xFFFF = never) |
| 6 | u16 | reserved |
| 8 | u8[37] | `avg` levels |
| 45 | u8[37] | `peak` levels |

Bin frame header `RtaBinFrameHeader` (first 16 bytes of the `0x0C` frame):

| Offset | Type | Field |
|--------|------|-------|
| 0 | u8 | `version` (3) |
| 1 | u8 | `channel` |
| 2 | u8 | `seq` (repeated as the frame's final byte) |
| 3 | u8 | `fft_order` |
| 4 | u32 | `sample_rate_hz` |
| 8 | u16 | `n_bins` (N/2) |
| 10 | u16[3] | reserved |
| 16 | u8[n_bins] | bin levels |
| 16 + n_bins | u8 | `seq` again |

`RtaStatus` (24 bytes):

| Offset | Type | Field |
|--------|------|-------|
| 0 | u8 | `version` (3) |
| 1 | u8 | `state`: 0 idle, 1 capturing, 2 transforming |
| 2 | u8 | `tap` |
| 3 | u8 | `channel` being worked on (0xFF when idle) |
| 4 | u8 | reserved |
| 5 | u8 | `live_count` |
| 6 | u16 | `live_mask` (channels selected and live at the tap) |
| 8 | u16 | `frames_per_s` |
| 10 | u16 | `busy_us_per_s` (main-loop transform time, EMA) |
| 12 | u16 | `last_frame_us` |
| 14 | u16 | `idle_ms` since the last data read (0xFFFF = never) |
| 16 | u32 | `sample_rate_hz` |
| 20 | u8 | `first_band` (0 with a band table, 0xFF for an unsupported rate) |
| 21 | u8 | reserved |
| 22 | u16 | `bass_busy_us_per_s` (saturates at 65535) |

Spec: `Features/spectrum_analyser_spec.md` section 5.

---

## 25. Bass enhancement, tube preamp, and stereo upmixer

*Last updated: 2026-11-15*

### 25.1 Common behaviour

These rules apply to every opcode below unless a row says otherwise.

* Every write is a true OUT transfer (`0x41`) with the payload in the data stage. No opcode in this group is write-as-read.
* A write whose payload is shorter than the size shown is ignored. Nothing changes. On USB the transfer is still ACKed. On UART/I2C it reports OK. The upmixer writes are the exception (see 0x4A/0x4C).
* Floats are clamped silently to the range shown. A read after a write returns the clamped value. Exceptions are noted (tube NaN, upmixer floats).
* Writes update the live config at once. Parameters marked "recompute" raise a pending flag. The main loop then rebuilds the coefficient set and publishes it glitch-free, typically within a few ms. Masks, the subharm pair link and subharm solo are read live by the audio path, so no recompute is needed.
* Nothing here writes flash. Persist with a preset save (`0x90`) or `SAVE_PARAMS` (`0x51`).
* Each write emits a `PARAM_CHANGED` notification at the field's `WireBulkParams` offset, except `SET_SUBHARM_SOLO` (0x2C), which emits none.
* `wIndex` is ignored by these handlers. `wValue` is ignored unless the table says it carries an index.

### 25.2 Subharmonic synthesizer

Both platforms. Output masks are `uint16` with bit k = output channel k (9 outputs on RP2350, 5 on RP2040). The effect ships disabled.

| ID | Name | Dir | wValue | Payload / Response | Notes |
|----|------|-----|--------|--------------------|-------|
| 0x10 | `SET_SUBHARM` | W (0x41) | 0 | 1 byte, nonzero = on | Recompute. Default off |
| 0x11 | `GET_SUBHARM` | R (0xC1) | 0 | 1 byte (0/1) | - |
| 0x12 | `SET_SUBHARM_LOW` | W (0x41) | 0 | f32 dB | 24-36 Hz band level. Clamps -30..+12. -30 = band off. Default 0. Recompute |
| 0x13 | `GET_SUBHARM_LOW` | R (0xC1) | 0 | f32 dB | - |
| 0x14 | `SET_SUBHARM_HIGH` | W (0x41) | 0 | f32 dB | 36-56 Hz band level. Clamps -30..+12. Default 0. Recompute |
| 0x15 | `GET_SUBHARM_HIGH` | R (0xC1) | 0 | f32 dB | - |
| 0x16 | `SET_SUBHARM_BOOST` | W (0x41) | 0 | f32 dB | LF boost bell (70 Hz). Clamps 0..+6. 0 = off. Recompute |
| 0x17 | `GET_SUBHARM_BOOST` | R (0xC1) | 0 | f32 dB | - |
| 0x18 | `SET_SUBHARM_MASK` | W (0x41) | 0 | 2 bytes, u16 LE output mask | Stored as sent, no validation. Default 0xFFFF. Read live |
| 0x19 | `GET_SUBHARM_MASK` | R (0xC1) | 0 | 2 bytes, u16 LE | - |
| 0x1A | `GET_SUBHARM_HEADROOM` | R (0xC1) | 0 | f32 dB | Read-only. Preamp headroom the host must free so the effect cannot clip. Computed from the live config on each read, so a read straight after a write is valid. 0.0 while disabled |
| 0x1B | `SET_SUBHARM_TOP` | W (0x41) | 0 | f32 dB | 56-80 Hz band level. Clamps -30..+12. Default -30 (off). Recompute |
| 0x1C | `GET_SUBHARM_TOP` | R (0xC1) | 0 | f32 dB | - |
| 0x1D | `SET_SUBHARM_SELECT` | W (0x41) | 0 | 1 byte mode | 0 = all, 1 = percussive, 2 = sustained. Values above 2 clamp to 2. Recompute |
| 0x1E | `GET_SUBHARM_SELECT` | R (0xC1) | 0 | 1 byte | - |
| 0x1F | `GET_SUBHARM_METER` | R (0xC1) | 0 | `NUM_OUTPUT_CHANNELS` x u16 LE (18 B RP2350, 10 B RP2040) | Read-only. Decaying peak of the synthesized sub mixed into each output, 0..32767 (same scale as status peaks) |
| 0x2C | `SET_SUBHARM_SOLO` | W (0x41) | 0 | 1 byte, nonzero = on | Monitoring only: masked outputs carry just the sub. Never persisted, not in bulk params, no notification. Read live |
| 0x2D | `GET_SUBHARM_SOLO` | R (0xC1) | 0 | 1 byte (0/1) | Poll this; there is no change event |
| 0x2E | `SET_SUBHARM_LINK` | W (0x41) | 0 | 1 byte, nonzero = on | Synthesize each output pair from its mono sum. Default on. Read live |
| 0x2F | `GET_SUBHARM_LINK` | R (0xC1) | 0 | 1 byte (0/1) | - |
| 0xA9 | `SET_SUBHARM_DEPTH` | W (0x41) | 0 | f32 % | Selectivity depth. Clamps 0..100. Default 100. Recompute |
| 0xAA | `GET_SUBHARM_DEPTH` | R (0xC1) | 0 | f32 % | - |
| 0xAB | `SET_SUBHARM_HOLD` | W (0x41) | 0 | f32 ms | Selectivity hold. Clamps 50..400. Default 150. Recompute |
| 0xAC | `GET_SUBHARM_HOLD` | R (0xC1) | 0 | f32 ms | - |
| 0xAD | `SET_SUBHARM_CEILING` | W (0x41) | 0 | f32 dBFS | Sub ceiling. Clamps -40..0. 0 = off. Recompute |
| 0xAE | `GET_SUBHARM_CEILING` | R (0xC1) | 0 | f32 dBFS | - |

`GET_SUBHARM_METER` response layout (entry k = output channel k, in output-mask order):

| Offset | Type | Field |
|--------|------|-------|
| 0 | u16 | sub peak, output 0 |
| 2 | u16 | sub peak, output 1 |
| 2k | u16 | sub peak, output k (k < `NUM_OUTPUT_CHANNELS`) |

Spec: `Features/subharmonic_synth_spec.md`.

### 25.3 Psychoacoustic bass

Both platforms. Output mask as for subharm. The effect ships disabled.

| ID | Name | Dir | wValue | Payload / Response | Notes |
|----|------|-----|--------|--------------------|-------|
| 0x30 | `SET_PSYBASS` | W (0x41) | 0 | 1 byte, nonzero = on | Recompute. Default off |
| 0x31 | `GET_PSYBASS` | R (0xC1) | 0 | 1 byte (0/1) | - |
| 0x32 | `SET_PSYBASS_CUTOFF` | W (0x41) | 0 | f32 Hz | Clamps 30..300. Default 80. Recompute |
| 0x33 | `GET_PSYBASS_CUTOFF` | R (0xC1) | 0 | f32 Hz | - |
| 0x34 | `SET_PSYBASS_HARMONICS` | W (0x41) | 0 | f32 dB | Harmonic mix level. Clamps -24..+12. Default 0. Recompute |
| 0x35 | `GET_PSYBASS_HARMONICS` | R (0xC1) | 0 | f32 dB | - |
| 0x36 | `SET_PSYBASS_DRIVE` | W (0x41) | 0 | f32 dB | Odd-path clipper drive. Clamps 0..18. Default 6. Recompute |
| 0x37 | `GET_PSYBASS_DRIVE` | R (0xC1) | 0 | f32 dB | - |
| 0x38 | `SET_PSYBASS_CHARACTER` | W (0x41) | 0 | f32 % | 0 = even only, 100 = odd only. Clamps 0..100. Default 50. Recompute |
| 0x39 | `GET_PSYBASS_CHARACTER` | R (0xC1) | 0 | f32 % | - |
| 0x3A | `SET_PSYBASS_ORIGINAL` | W (0x41) | 0 | f32 dB | Original low-band level. Clamps -60..0. Default 0. Recompute |
| 0x3B | `GET_PSYBASS_ORIGINAL` | R (0xC1) | 0 | f32 dB | - |
| 0x3C | `SET_PSYBASS_MASK` | W (0x41) | 0 | 2 bytes, u16 LE output mask | Stored as sent. Default 0xFFFF. Read live |
| 0x3D | `GET_PSYBASS_MASK` | R (0xC1) | 0 | 2 bytes, u16 LE | - |

Spec: `Features/psychoacoustic_bass_spec.md`.

### 25.4 Tube preamp

Both platforms. One indexed pair covers every parameter. The index is the **low byte** of `wValue` on both directions. The high byte is ignored. Every value travels as an f32 LE, including booleans, the mask and the enums.

| ID | Name | Dir | wValue | Payload / Response | Notes |
|----|------|-----|--------|--------------------|-------|
| 0x3E | `SET_TUBE_PARAM` | W (0x41) | param index (0..13) | 4 bytes, f32 LE | Index above 13 or payload under 4 bytes: ignored, transfer still ACKed. NaN: ignored |
| 0x3F | `GET_TUBE_PARAM` | R (0xC1) | param index (0..13) | 4 bytes, f32 LE | Index above 13 STALLs, so index 0 works as a feature probe |

Parameter indices. Booleans are nonzero = true. The mask and enums are clamped, then rounded to the nearest integer. Floats are clamped to the range.

| Index | Field | Range | Default | Recompute |
|-------|-------|-------|---------|-----------|
| 0 | enabled | 0 / 1 | 0 | yes |
| 1 | output_mask | 0..65535 (bit k = output k) | 65535 | no (read live) |
| 2 | tube_type | 0..16 (0 = custom) | 1 (12AX7) | yes |
| 3 | drive_db | -30..+24 dB | -12 | yes |
| 4 | bias_pct | -100..+100 % | 10 | yes |
| 5 | asym_db | -12..+12 dB | 3 | yes |
| 6 | hardness_pct | 0..100 % | 40 | yes |
| 7 | sag_pct | 0..100 % | 15 | yes |
| 8 | rectifier | 0..3 (0 = solid state) | 1 (GZ34) | yes |
| 9 | xfmr_enabled | 0 / 1 | 1 | yes |
| 10 | xfmr_damping | 1..20 | 2 | yes |
| 11 | xfmr_res_hz | 30..150 Hz | 95 | yes |
| 12 | mix_pct | 0..100 % | 100 | yes |
| 13 | trim_db | -12..+12 dB | 0 | yes |

Side effects. Writing `tube_type` 1..16 also overwrites indices 4..7 from that tube's row, and each changed field emits its own notification. Writing 0 leaves 4..7 alone. Writing any of 4..7 to a value different from the stored one resets `tube_type` to 0. Re-read 2 and 4..7 after either kind of write.

Spec: `Features/tube_preamp_spec.md` (tube rows in section 2.3).

### 25.5 Stereo upmixer

**RP2350 only.** On RP2040 both writes are refused (USB ACKs and ignores them; UART/I2C report ERROR) and all three reads succeed with all-zero payloads. Identify the platform with `GET_PLATFORM` (0x7F) first.

| ID | Name | Dir | wValue | Payload / Response | Notes |
|----|------|-----|--------|--------------------|-------|
| 0x4A | `UPMIX_SET_CONFIG` | W (0x41) | 0 | `UpmixConfigPacket`, exactly 44 B | Any other length is refused (USB: ACKed, nothing applied; UART/I2C: ERROR). Recompute |
| 0x4B | `UPMIX_GET_CONFIG` | R (0xC1) | 0 | `UpmixConfigPacket`, 44 B | 44 zero bytes on RP2040 |
| 0x4C | `UPMIX_SET_PARAM` | W (0x41) | param id (0..13), full 16 bits | 4 bytes, f32 LE | Id 14 or above, or payload under 4 bytes, is refused like 0x4A. Recompute |
| 0x4D | `UPMIX_GET_PARAM` | R (0xC1) | param id (0..13), full 16 bits | 4 bytes, f32 LE | Id 14 or above STALLs on RP2350. RP2040 returns 0.0 for any id |
| 0x4E | `UPMIX_GET_STATUS` | R (0xC1) | 0 | `UpmixStatus`, 16 B | Live telemetry. 16 zero bytes on RP2040 |

Range handling. Mode fields are sanitized when written. `center_mode` outside 0..2 becomes 1 (ADAPTIVE), not the top value. `surround_mode` above 2 becomes 2. Float fields are stored **as sent** and clamped only when coefficients are computed, so reads return the unclamped value. Presence is the special case: `GET_PARAM` id 13 returns the stored float, while `GET_CONFIG` returns it clamped to +/-12 dB in 0.5 dB steps.

`UpmixConfigPacket` (44 bytes, packed, little-endian):

| Offset | Type | Field |
|--------|------|-------|
| 0 | u8 | enabled (nonzero = on; read back as 0/1) |
| 1 | u8 | center_mode: 0 = PASSIVE, 1 = ADAPTIVE, 2 = OFF |
| 2 | u8 | surround_mode: 0 = OFF, 1 = PASSIVE, 2 = ADAPTIVE |
| 3 | i8 | presence_q1: presence in 0.5 dB steps (dB x 2, -24..+24) |
| 4 | f32 | strength_pct (0..100) |
| 8 | f32 | center_width_pct (0..100) |
| 12 | f32 | corr_threshold_pct (0..95) |
| 16 | f32 | attack_ms (1..500) |
| 20 | f32 | release_ms (5..2000) |
| 24 | f32 | detector_hpf_hz (20..1000) |
| 28 | f32 | surround_delay_ms (0..20) |
| 32 | f32 | surround_hpf_hz (20..2000) |
| 36 | f32 | surround_lpf_hz (1000..20000) |
| 40 | f32 | decorr_pct (0..100) |

Ranges in brackets are the clamps applied at coefficient compute time.

`UPMIX_SET_PARAM` / `UPMIX_GET_PARAM` ids (all values f32):

| Id | Field | Write handling |
|----|-------|----------------|
| 0 | enabled | nonzero = on |
| 1 | center_mode | rounded; outside 0..2 becomes 1 |
| 2 | surround_mode | rounded; clamped to 0..2 |
| 3 | strength_pct | stored as sent |
| 4 | center_width_pct | stored as sent |
| 5 | corr_threshold_pct | stored as sent |
| 6 | attack_ms | stored as sent |
| 7 | release_ms | stored as sent |
| 8 | detector_hpf_hz | stored as sent |
| 9 | surround_delay_ms | stored as sent |
| 10 | surround_hpf_hz | stored as sent |
| 11 | surround_lpf_hz | stored as sent |
| 12 | decorr_pct | stored as sent |
| 13 | presence_db (-12..+12 at compute) | stored as sent |

`UpmixStatus` (16 bytes, packed, little-endian):

| Offset | Type | Field |
|--------|------|-------|
| 0 | u8 | active: 1 = processing now |
| 1 | u8 | parked_reason: 0 = active, 1 = disabled, 2 = input not stereo, 3 = rate above 48 kHz |
| 2 | i16 | corr_q14: L/R correlation, value/16384 in -1..+1 |
| 4 | i16 | balance_q14: 0 = centred, 16384 = fully one-sided |
| 6 | u16 | center_gain_q15: centre extraction gain, value/32767 |
| 8 | u16 | ls_gain_q15: Ls steering gain, Q15 |
| 10 | u16 | rs_gain_q15: Rs steering gain, Q15 |
| 12 | u8[4] | reserved (0) |

Spec: `Features/upmixer_spec.md` (section 6).

---

## 26. Control Surfaces

*Last updated: 2026-11-15*

Control Surfaces bind physical controls and indicators on spare GPIOs to firmware parameters.
All commands below work on RP2040 and RP2350. None are refused on the UART or I2C control transports.
The auxiliary output runtime commands 0x04-0x07 are in 26.7.

Current capability format: `caps_version` = 22 (`control_surfaces.c`).

### 26.1 Shared model: preview, status channel, Save and Revert

Every Control Surfaces SET is a deferred, single-deep handoff:

1. The handler checks the index and payload length, copies the record into a pending buffer, and sets `last_status` = `0x16` (PENDING).
2. The main loop applies it later. The apply is a live preview only. It claims pins and takes effect, but writes nothing to flash.
3. The main loop overwrites `last_status` with the result. On success it sets the `dirty` flag.

A SET never returns its result directly. Poll `GET_CS_STATUS` (0x87) until `last_status` is no longer `0x16`, and check `last_slot` to see which record it refers to.

Rules that hold for every SET in this section:

* Over USB a SET is always ACKed, even when refused. Over UART and I2C it returns status `0x00` (OK) even when refused. Only `GET_CS_STATUS` shows the outcome.
* A refusal at the handler (bad index, short payload, previous SET of the same kind still queued) is written to `last_status` at once. It never becomes PENDING.
* A SET of a kind that is still queued records `0x1B` (BUSY) and is dropped. The queued one is kept.
* Only the low byte of `wValue` is used for the index (except `SET_CS_MACRO_STEP`, which uses both bytes).
* A payload longer than the record is accepted. Extra bytes are ignored. A payload over 64 bytes STALLs at SETUP on USB.
* All result kinds share one `last_status` / `last_slot` pair. Let each SET resolve before you send the next one of any kind, or the later result overwrites the earlier one.
* A rejected apply leaves the previous record intact and does not touch `dirty`.

`last_slot` tags:

| `last_slot` | Source |
|---|---|
| `0x00`-`0x0F` | binding slot (0x84) or slot name (0x8B) |
| `0x40`-`0x47` | group SET (0x20), as `0x40 \| group` |
| `0x50` | display config SET (0x27) |
| `0x50`-`0x5F` | display page SET (0x29), as `0x50 \| page` (page 0 also reads `0x50`) |
| `0x60`-`0x67` | macro header (0x22), macro step (0x24), or failed macro fire (0x25), as `0x60 \| macro` |
| `0x80`-`0x8F` | IR command SET (0x8D), as `0x80 \| sub-slot` |
| `0xFF` | Save (0x9D), Revert (0x9E), or IR learn arm refused with NO_IR (0x8F) |

Save (`CS_SAVE`, 0x9D) persists the whole live configuration in one preset-directory flash write. That covers the 16 bindings, 16 IR commands, 16 slot names, 8 groups, 8 macros, and the display config and pages. It uses the same flash path as other metadata writes (`prepare_flash_write_operation` / `complete_flash_write_operation_full`). The result is `0x00` or `0x1C` (FLASH_ERROR). `dirty` clears on success.

Revert (`CS_REVERT`, 0x9E) discards the preview. It re-applies the stored configuration from the directory cache as a background job, with no flash write. Pins, PWM slices, the IR receiver, and the display are released and reclaimed as at boot. When the job finishes, `dirty` clears and `last_status` becomes `0x00`, unless a newer request took over the status channel during the revert (`control_surfaces.c`).

While a revert runs:

* Queued SETs wait. The main loop does not consume them until the job ends.
* `CS_MACRO_FIRE` (fire only) and `CS_IR_LEARN` (arm and cancel) STALL. `last_status` is left unchanged.
* `GET_CS_STATUS` returns the per-slot fields as they were when the revert started, with live `last_status`, `last_slot`, and `dirty`.
* `GET_CS_BINDING` and `GET_CS_IR_CMD` return the stored records.

A reboot discards an unsaved preview.

Status codes (`control_surfaces.h`). `0x00`-`0x02` come from the shared `PIN_CONFIG_*` set:

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

Every Control Surfaces structure is packed and little-endian.

Spec: `Features/control_surfaces_spec.md` sections 3.2-3.5.

### 26.2 Bindings, capabilities, status (0x84-0x87)

| ID | Name | Dir | wValue | Payload / Response | Notes |
|---|---|---|---|---|---|
| 0x84 | `SET_CS_BINDING` | W (0x41) | slot 0-15 | 24-byte `CsBinding` | Deferred preview. Type `0` (NONE) clears the slot. A payload under 24 bytes records `0x14`. Slot 16 or higher records `0x10`. |
| 0x85 | `GET_CS_BINDING` | R (0xC1) | slot 0-15 | 24-byte `CsBinding` | Live record. Returns the stored record during a revert. STALL if `wValue` is 16 or higher. |
| 0x86 | `GET_CS_CAPS` | R (0xC1) | `0xFFFF`, or noun 0 to `noun_count`-1 | `0xFFFF`: 52-byte `CsCapsHeader`. Noun: 12-byte `CsNounDesc` | STALL if the noun is out of range. `noun_count` is 81 (nouns 0-80) at caps v22. |
| 0x87 | `GET_CS_STATUS` | R (0xC1) | ignored | 41-byte `CsStatusPacket` | The result channel for every deferred CS SET, Save, and Revert. |

`CsBinding` (24 bytes, `control_surfaces.h`). The wire and flash formats are identical.

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | `type`: `CsType` 0-10 (0 NONE, 1 BUTTON, 2 SWITCH, 3 POT, 4 ENCODER, 5 LED, 6 LED_PWM, 7 IR, 8 DISPLAY, 9 AUX_OUT, 10 AUX_PWM) |
| 1 | u8 | `noun`: `CsNoun` |
| 2 | u8 | `action`: `CsAction` 0-11 |
| 3 | u8 | `flags`: `CS_FLAG_*` (0x01 INVERT, 0x02 REVERSE, 0x04 WRAP, 0x08 ACCEL, 0x10 REPEAT, 0x20 GROUP, 0x40 LINK_ABS, 0x80 GROUP_ALL) |
| 4 | u8 | `gpio[0]` |
| 5 | u8 | `gpio[1]`: `0xFF` unless the type uses two pins (encoder, display) |
| 6 | u8 | `event`: buttons only (0 press, 1 long, 2 double), else 0 |
| 7 | u8 | `target`: channel index, or group index when `CS_FLAG_GROUP` is set |
| 8 | u8 | `index`: filter band for band-addressed nouns, else 0 |
| 9 | u8 | `base_bright`: PWM brightness ceiling 1-100 %, 0 = full |
| 10 | i16 | `value` |
| 12 | i16 | `step`: 0 = unit default |
| 14 | i16 | `range_min` |
| 16 | i16 | `range_max`: both 0 = the noun's full range |
| 18 | u16 | `on_delay`: 0.1 s units |
| 20 | u16 | `off_delay`: 0.1 s units |
| 22 | u8 | `extras`: `CS_AUX_X_*` on aux slots, else 0 |
| 23 | u8 | `reserved2`: write 0 |

The encoding of value, step, and range follows the noun's unit. IR, display, and aux slots are containers with their own rules. See `control_surfaces_spec.md` sections 2.1-2.2, `control_surfaces_display_spec.md` section 2.1, and `control_surfaces_aux_spec.md`.

`CsCapsHeader` (52 bytes at caps v22, `control_surfaces.h`):

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | `caps_version` (22) |
| 1 | u8 | `max_bindings` (16) |
| 2 | u8 | `type_count` (11) |
| 3 | u8 | `noun_count` (81) |
| 4 | `CsTypeDesc[type_count]` | per type, 4 bytes each: u16 `actions` mask, u8 `pin_count`, u8 `pin_class` (0 any, 1 ADC) |
| 4+4*type_count | u8 | `max_ir_commands` (16) |
| +1 | u8 | `max_groups` (8) |
| +2 | u8 | `max_macros` (8) |
| +3 | u8 | `max_macro_steps` (8) |

Hosts must locate the trailing four bytes at `4 + 4*type_count`. The type table grows when types are added.

`CsNounDesc` (12 bytes, `control_surfaces.h`):

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | `kind`: 0 continuous, 1 bool, 2 enum |
| 1 | u8 | `enum_count` |
| 2 | u16 | `actions`: `CS_ACT_BIT` mask. 0 = noun unavailable on this platform |
| 4 | i16 | `min_q`: unit-encoded |
| 6 | i16 | `max_q` |
| 8 | u8 | `unit`: `CS_UNIT_*` 0-6 |
| 9 | u8 | `target_kind`: `CS_TARGET_*` 0-5 |
| 10 | u8 | `target_count` |
| 11 | u8 | `dflags`: bit 0 = deferred apply |

The noun list is not reproduced here. Enumerate it at connect with `GET_CS_CAPS` for nouns 0 to `noun_count`-1. See `control_surfaces_spec.md` sections 4.3 and 5.

`CsStatusPacket` (41 bytes, `control_surfaces.h`):

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | `last_status`: result of the most recent deferred CS request |
| 1 | u8 | `last_slot`: see the tag table above |
| 2 | u8 | `max_bindings` (16) |
| 3 | u8 | `dirty`: 1 = unsaved live preview |
| 4 | u16 | `active_mask`: bit N = binding N live |
| 6 | u8[16] | `slot_status`: per-slot apply status |
| 22 | u16 | `ir_active_mask`: bit N = IR command N live |
| 24 | u8 | `ir_learn_state`: 0 idle, 1 armed, 2 done, 3 timeout |
| 25 | u8[16] | `ir_cmd_status`: per-sub-slot apply status |

### 26.3 Slot names (0x8B-0x8C)

| ID | Name | Dir | wValue | Payload / Response | Notes |
|---|---|---|---|---|---|
| 0x8B | `SET_CS_NAME` | W (0x41) | slot 0-15 | 1-64 bytes of name | Deferred preview. The first 31 bytes are kept and the name is cut at the first NUL. A single NUL byte clears it. An empty payload over UART or I2C records `0x14`. No notify. |
| 0x8C | `GET_CS_NAME` | R (0xC1) | slot 0-15 | 32 bytes, NUL-terminated | Live name (the unsaved preview while `dirty` is set). STALL if the low byte is 16 or higher. The high byte is not checked. |

Names are slot metadata. They survive binding changes and may be set before a binding exists.

### 26.4 IR commands and learn (0x8D-0x8F)

| ID | Name | Dir | wValue | Payload / Response | Notes |
|---|---|---|---|---|---|
| 0x8D | `SET_CS_IR_CMD` | W (0x41) | sub-slot 0-15 | 16-byte `IrCommand` | Deferred preview. `protocol` 0 with an all-zero record clears the sub-slot. A command may be set before the IR receiver binding exists. It goes live when the receiver comes up. |
| 0x8E | `GET_CS_IR_CMD` | R (0xC1) | sub-slot 0-15 | 16-byte `IrCommand` | Live record. Returns the stored record during a revert. STALL if `wValue` is 16 or higher. |
| 0x8F | `CS_IR_LEARN` | write-as-read (0xC1) for 0 and 1. R for 2 | 1 = arm, 0 = cancel, 2 = read result | 0 and 1: 1 byte, always `0x01`. 2: 8 bytes `{state, protocol, 0, 0, code u32}` | Arm and cancel run synchronously. Arm with no live IR binding STALLs and records `0x1E` with `last_slot` `0xFF`. Arm or cancel during a revert STALLs. Any other `wValue` STALLs. |

`IrCommand` (16 bytes, `control_surfaces.h`):

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | `noun` |
| 1 | u8 | `action`: button subset |
| 2 | u8 | `flags`: WRAP, REPEAT, GROUP |
| 3 | u8 | `target` |
| 4 | u8 | `index` |
| 5 | u8 | `protocol`: 0 none, 1 NEC, 2 RC5, 3 RC6, 4 hash |
| 6 | i16 | `value` |
| 8 | i16 | `step` |
| 10 | u8[2] | `reserved`: write 0 |
| 12 | u32 | `code`: the learned code |

Learn flow:

1. Arm with `wValue` 1. The receiver listens for 10 s (`control_surfaces_ir.c`).
2. The device captures the first decoded press, or times out after 10 s.
3. Completion pushes notify event `0x0A` (`NOTIFY_EVT_CS_IR_LEARN`). The packet is 12 bytes: `[ver=2, 0x0A, 0, seq, state, protocol, 0, 0, code u32 LE]` (`notify.c`). State 2 = done and state 3 = timeout. Removing the IR binding while armed also posts a timeout event.
4. A host without the notify endpoint polls `wValue` 2 or `ir_learn_state` in 0x87. In the 8-byte result, `protocol` and `code` are zero unless the state is 2.

Cancel (`wValue` 0) returns to idle and posts no event.

Spec: `control_surfaces_spec.md` sections 2.7 and 3.6.

### 26.5 Save and Revert (0x9D-0x9E)

| ID | Name | Dir | wValue | Payload / Response | Notes |
|---|---|---|---|---|---|
| 0x9D | `CS_SAVE` | write-as-read (0xC1) | ignored | 1 byte, always `0x01` | Sets `cs_save_pending` and records PENDING with `last_slot` `0xFF`. STALLs and records `0x1B` if a Save or Revert is already queued. Final result via 0x87: `0x00` or `0x1C`. |
| 0x9E | `CS_REVERT` | write-as-read (0xC1) | ignored | 1 byte, always `0x01` | Sets `cs_revert_pending`, with the same PENDING and BUSY rules as Save. The main loop starts the background revert job. Success is posted when the job completes. |

### 26.6 Target groups and macros (0x20-0x26, caps v9)

| ID | Name | Dir | wValue | Payload / Response | Notes |
|---|---|---|---|---|---|
| 0x20 | `SET_CS_GROUP` | W (0x41) | group 0-7 | 40-byte `CsGroup` | Deferred preview. An all-zero record clears the group. The apply re-validates every binding and IR command that references the group. Group 8 or higher records `0x1F`. |
| 0x21 | `GET_CS_GROUP` | R (0xC1) | group 0-7 | 40-byte `CsGroup` | Live record. STALL if `wValue` is 8 or higher. |
| 0x22 | `SET_CS_MACRO` | W (0x41) | macro 0-7 | 36-byte `CsMacroHeaderWire` | Deferred preview of the name and `step_count` only. `step_count` over 8 records `0x20`. Nonzero reserved bytes record `0x14`. |
| 0x23 | `GET_CS_MACRO` | R (0xC1) | macro 0-7 | 132-byte `CsMacro` | Live record. Larger than 64 bytes but within the 132-byte UART and I2C GET limit. STALL if `wValue` is 8 or higher. |
| 0x24 | `SET_CS_MACRO_STEP` | W (0x41) | `(step << 8) \| macro`, step 0-7, macro 0-7 | 12-byte `CsMacroStep` | Deferred preview. An all-zero record clears the step. A bad macro records `0x20`. A bad step records `0x21`. Its pending flag is separate from 0x22, so a header and a step may be queued together. |
| 0x25 | `CS_MACRO_FIRE` | write-as-read (0xC1) | macro 0-7, or `0xFFFF` = cancel | 1 byte, always `0x01` | Runs synchronously. Firing cancels any running macro at its step boundary. An empty macro is a successful no-op. Cancel always succeeds, even during a revert. Fire during a revert STALLs. Any other `wValue` STALLs. |
| 0x26 | `GET_CS_EXT_STATUS` | R (0xC1) | ignored | 24-byte `CsExtStatusPacket` | Table limits, per-group and per-macro validity, and the running macro. |

For macros, write the steps first and the header (with the final `step_count`) last.

`CsGroup` (40 bytes):

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | `target_kind`: 1 input ch, 2 output ch, 3 DSP ch. 0 = empty, and the record must then be all-zero |
| 1 | u8[3] | `reserved`: write 0 |
| 4 | u32 | `member_mask`: bit N = channel N. Must be nonzero and within the platform's channel count |
| 8 | char[32] | `name` |

`CsMacroStep` (12 bytes):

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | `noun`: not `CS_NOUN_MACRO` |
| 1 | u8 | `action`: SET, TOGGLE, INC, DEC, or TRIGGER |
| 2 | u8 | `flags`: WRAP, GROUP |
| 3 | u8 | `target` |
| 4 | u8 | `index` |
| 5 | u8 | `reserved` |
| 6 | i16 | `value` |
| 8 | i16 | `step` |
| 10 | u16 | `pre_delay`: 10 ms units |

`CsMacroHeaderWire` (36 bytes) is `{char name[32]; u8 step_count; u8 reserved[3]}`.

`CsMacro` (132 bytes) is the same 36-byte header followed by `CsMacroStep steps[8]` at offset 36.

`CsExtStatusPacket` (24 bytes):

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | `max_groups` (8) |
| 1 | u8 | `max_macros` (8) |
| 2 | u8 | `max_macro_steps` (8) |
| 3 | u8 | `macro_running`: `0xFF` = idle |
| 4 | u8 | `macro_step`: 0 when idle |
| 5 | u8[3] | `reserved` |
| 8 | u8[8] | `group_status` |
| 16 | u8[8] | `macro_status`: worst step validity |

Spec: `Features/control_surfaces_groups_macros_spec.md` sections 2-3.

### 26.7 Auxiliary outputs (0x04-0x07)

`wValue` is the binding slot (0-15) holding a `CS_TYPE_AUX_OUT` (9) or `CS_TYPE_AUX_PWM` (10)
component. Writes apply immediately in the handler: no flash write, no dirty flag, no deferral.
Levels are 8.8 fixed-point percent, little-endian (25600 = 100 %).

| ID | Name | Dir | wValue | Payload / Response | Notes |
|----|------|-----|--------|--------------------|-------|
| 0x04 | `SET_CS_AUX_STATE` | W (0x41) | slot (low byte used) | 1 byte, non-zero = on | Rejected unless the slot is an aux output that is up |
| 0x05 | `GET_CS_AUX_STATE` | R (0xC1) | slot, or `0xFFFF` | slot: 1 byte state. `0xFFFF`: 48 bytes (below) | Single-slot read STALLs unless the slot is an aux output that is up |
| 0x06 | `SET_CS_AUX_LEVEL` | W (0x41) | slot (low byte used) | 2 bytes u16 LE, 8.8 percent | Clamped to 25600. Rejected unless the slot is an up `AUX_PWM` |
| 0x07 | `GET_CS_AUX_LEVEL` | R (0xC1) | slot | 2 bytes u16 LE, 8.8 percent | Reads 0 on an `AUX_OUT` slot. STALLs unless the slot is an aux output that is up |

A rejected SET records `cs_last_status` / `cs_last_slot` (read with `GET_CS_STATUS` 0x87):
`CS_STATUS_INVALID_AUX` (0x26) for slot >= 16, a non-aux slot, a down slot, or a level on
`AUX_OUT`; `CS_STATUS_INVALID_VALUE` (0x14) for a short payload. While a Control Surfaces revert is
running, a SET is refused without touching `cs_last_status`, and GETs report the value the slot will
come back with. A SET that changes a value pushes `NOTIFY_EVT_CS_AUX` (0x0C). An unchanged write is
silent.

`0x05` with `wValue = 0xFFFF` (48 bytes):

| Offset | Type | Field |
|--------|------|-------|
| 0 | u8[16] | `state[slot]` (0 for slots that are not an up aux output) |
| 16 | u16[16] LE | `level_q8[slot]` (0 for non-aux slots and `AUX_OUT` slots) |

Spec: `Features/control_surfaces_aux_spec.md` section 3.

### 26.8 I2C display (0x27-0x2B, caps v10)

| ID | Name | Dir | wValue | Payload / Response | Notes |
|---|---|---|---|---|---|
| 0x27 | `SET_CS_DISPLAY_CFG` | W (0x41) | ignored | 12-byte `CsDisplayCfg` | Deferred preview. Device-global. Status tag `0x50`. |
| 0x28 | `GET_CS_DISPLAY_CFG` | R (0xC1) | ignored | 16 bytes: `{max_pages (16), model_count (9), 0, 0}` + live `CsDisplayCfg` | |
| 0x29 | `SET_CS_DISPLAY_PAGE` | W (0x41) | page 0-15 | 4-byte `CsDisplayPage` | Deferred preview. An all-zero record clears the page. Page 16 or higher records `0x25`. |
| 0x2A | `GET_CS_DISPLAY_PAGE` | R (0xC1) | page 0-15 | 4-byte `CsDisplayPage` | Live record, all-zero when empty. STALL if `wValue` is 16 or higher. |
| 0x2B | `GET_CS_DISPLAY_STATUS` | R (0xC1) | ignored | 8-byte `CsDisplayStatus` | Runtime state. |

The display itself is a `CS_TYPE_DISPLAY` (8) binding set with 0x84. Its `gpio[0]` is SDA, `gpio[1]` is SCL, and `index` is the model 1-8.

`CsDisplayCfg` (12 bytes):

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | `mode`: 0 fixed, 1 cycle selected, 2 cycle all |
| 1 | u8 | `home_page`: 0-15 |
| 2 | u16 | `dwell`: 0.1 s units. Must be 10 or more in the cycle modes |
| 4 | u16 | `overlay_hold`: 0.1 s units. 0 = overlay off |
| 6 | u8 | `brightness` |
| 7 | u8 | `flags`: bit 0 overlay-any, bit 1 edit-gated, bits 3:2 label align, bits 5:4 value align (0 left, 1 centre, 2 right) |
| 8 | u16 | `edit_timeout`: 0.1 s units |
| 10 | u8[2] | `reserved`: write 0 |

`CsDisplayPage` (4 bytes) is `{noun, target, index, flags}`. The flags are 0x01 ACTIVE, 0x02 GROUP, 0x04 LARGE, and 0x08 BAR.

`CsDisplayStatus` (8 bytes):

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | `init_state`: 0 down, 1 init, 2 live, 3 error |
| 1 | u8 | `current_page`: `0xFF` = none or overlay |
| 2 | u8 | `flags`: bit 0 overlay showing, bit 1 edit armed |
| 3 | u8 | `model` |
| 4 | u16 | `nak_count` |
| 6 | u8[2] | `reserved` |

Spec: `Features/control_surfaces_display_spec.md` sections 2.2-2.4.

