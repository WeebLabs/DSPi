# DSPi Multichannel I2S Input - App Developer Guide

*Last updated: 2026-11-16 (host interface on the paged protocol of firmware 1.1.7)*

This document is the single, self-contained reference an app developer needs to
add support for DSPi's I2S input channel modes (2 / 4 / 6 / 8 channels). It
covers the working principles, every relevant vendor command, the unified
channel model, validation rules, persistence, host notifications, and concrete
workflow patterns. No prior knowledge of the firmware internals is assumed.

If you only read one section, read **"Quick start"** then **"App workflow
patterns"**.

---

## 1. What this feature is

DSPi can take its audio from one of three input sources: USB (UAC1), S/PDIF, or
**I2S**. When the source is I2S, the device samples an external I2S transmitter
(typically an ADC) on a shared bit-clock / word-clock pair plus one serial-data
line per stereo pair. This guide is about the **multichannel** I2S modes: 2, 4,
6, or 8 input channels, captured as 1 to 4 stereo pairs that are guaranteed to be
sample-aligned with each other.

Key facts up front:

- **Platform:** multichannel (4/6/8) is **RP2350 only**. RP2040 supports I2S
  input but is **stereo-only** (2 channels / 1 pair).
- **Channel counts:** `2`, `4`, `6`, `8` (i.e. 1 to 4 stereo pairs).
- **Format:** 24-bit audio in 32-bit I2S frames, 44.1 / 48 / 96 kHz; the device
  is the clock master (or follows the I2S output clock if an output slot is I2S;
  see "Clock roles").
- **Pins:** one shared BCK (bit clock) + LRCLK (word clock) pair, plus one
  independently-assignable serial-data GPIO per stereo pair.
- **Unified channel model:** each active input channel (0..7) is a first-class
  DSP channel with its own parametric EQ, preamp, metering, and name; all inputs
  feed an 8 x N routing matrix to the outputs.

---

## 2. Quick start

The minimal sequence to get 8-channel I2S input running (RP2350):

```
1. Set channel count:        INPUT.I2S_RX_CHANNELS SET  payload = 8
2. Assign the 3 extra pins:  INPUT.I2S_RX_PIN SET  wValue = 1, payload = gpio1
                             INPUT.I2S_RX_PIN SET  wValue = 2, payload = gpio2
                             INPUT.I2S_RX_PIN SET  wValue = 3, payload = gpio3
   (pair 0's data pin and the shared BCK pin already have valid defaults)
3. Switch the source to I2S: INPUT.SOURCE SET  payload = 2  (INPUT_SOURCE_I2S)
4. (optional) Select rate:   INPUT.I2S_RX_RATE SET  payload = 48000 (u32)
5. Route inputs to outputs:  ROUTE.MATRIX_ROUTE SET  (per crosspoint)
```

Any step can be refused. A refused SET STALLs, and the reason is readable from
`SYS.LAST_RESULT` (see "Validation & errors"). After step 3 the device emits a `NOTIFY_EVT_INPUT_FORMAT` push so your
UI can relayout to the active channel count.

Recommended order is **count first, then pins** (the default extra pins are
distinct and free, so raising the count never fails on a stale pin), but both
orders work.

---

## 3. Working principles

### 3.1 Clock + data topology

I2S input uses one **shared clock pair** for every stereo pair:

- **BCK** (bit clock) on `i2s_bck_pin`.
- **LRCLK** (word/frame clock) on `i2s_bck_pin + 1` (always BCK+1; not separately
  assignable).
- One **serial-data** line per stereo pair, on `i2s_rx_pin[0..3]`, each
  independently assignable.

```
            +-------------------- BCK  (i2s_bck_pin)
            |  +----------------- LRCLK (i2s_bck_pin + 1)
   external |  |   data0 ------>  pair 0 -> input channels 0,1
   ADC(s)   |  |   data1 ------>  pair 1 -> input channels 2,3
            |  |   data2 ------>  pair 2 -> input channels 4,5
            |  |   data3 ------>  pair 3 -> input channels 6,7
```

All four data lines are clocked by the one BCK/LRCLK pair, so the external
transmitter(s) must share that clock.

### 3.2 Clock roles (who drives BCK/LRCLK)

- **Master role (default):** no output slot is configured as I2S, so the device
  generates BCK/LRCLK and the external ADC slaves to them. This is the normal
  case for an ADC front-end.
- **Slave role:** at least one output slot is I2S, so the I2S *output* clock
  master drives BCK/LRCLK and the input samples against those same pads.

You do not select the role; the firmware derives it from the output
configuration. It matters only in that BCK/LRCLK are shared between I2S input
and I2S output.

### 3.3 Sample alignment guarantee

When you select 4/6/8 channels, the pairs are started on a single
synchronized cycle and advance in lockstep on the shared clock, so the
2/4/6/8 channels are **sample-aligned** (no inter-channel skew). This is a hard
guarantee of the firmware; you can rely on phase coherence across all input
channels for multichannel imaging, sub integration, and measurement.

### 3.4 Unified channel model

DSPi treats input channels as first-class DSP channels:

- Channel indices `0..NUM_INPUT_CHANNELS-1` are **inputs** (0..7 on RP2350,
  0..1 on RP2040).
- Channel indices `CH_OUT_1..NUM_CHANNELS-1` are **outputs**
  (`CH_OUT_1 == NUM_INPUT_CHANNELS`).
- Each input channel has its own **parametric EQ** (up to `MAX_BANDS` = 12
  bands), **preamp gain**, **peak meter**, and **name**.
- All active inputs feed an **N x M matrix mixer** (8 x 9 on RP2350) whose
  crosspoints route any input to any output with gain + polarity.

So "4-channel I2S input" means input channels 0..3 are live, each independently
processable, then mixed to the outputs through the matrix.

### 3.5 What is bypassed in multichannel mode

Some DSP blocks are inherently stereo and are **bypassed when more than 2 input
channels are active** (whether the multichannel source is I2S or USB):

- **Loudness compensation**
- **Volume leveller**
- **Crossfeed**

In multichannel mode each input gets only its own per-channel PEQ + preamp, then
the matrix. Per-output PEQ, crossover, gain, and delay still apply normally on
the output side. If your app exposes loudness/leveller/crossfeed, grey them out
(or mark them inactive) whenever the active input count is greater than 2.

---

## 4. Platform support

| Capability | RP2040 | RP2350 |
|---|---|---|
| I2S input | Yes (stereo) | Yes |
| I2S input channels | 2 only | 2 / 4 / 6 / 8 |
| Max stereo pairs (`I2S_RX_MAX_PAIRS`) | 1 | 4 |
| Input channels (`NUM_INPUT_CHANNELS`) | 2 | 8 |
| Output channels (`NUM_OUTPUT_CHANNELS`) | 5 (4 S/PDIF + PDM) | 9 (8 S/PDIF + PDM) |

On RP2040, `INPUT.I2S_RX_CHANNELS` accepts only `2`; `4/6/8` are rejected with
`VC_ERR_UNSUPPORTED`. `INPUT.I2S_RX_PIN` accepts only pair `0` (other pairs are
`VC_ERR_ADDRESS`). Write your app to read the supported
maximum from the device rather than hard-coding 8 (see "Capability detection").

---

## 5. Hardware / wiring

- Wire the external transmitter's BCK to `i2s_bck_pin` and its LRCLK to
  `i2s_bck_pin + 1`.
- Wire each ADC's serial-data output to the GPIO you assign for that pair
  (`i2s_rx_pin[0..3]`).
- Standard I2S framing: 24-bit data, MSB first, 1-bit delay, left channel when
  LRCLK is low (matches the device's I2S output framing).
- The data pins are inputs only; the device never drives them.

**Defaults (firmware power-on / factory):**

| Item | Default GPIO |
|---|---|
| BCK (`i2s_bck_pin`) | 14 (LRCLK = 15) |
| Pair 0 data (`i2s_rx_pin[0]`) | 1 |
| Pair 1 data (`i2s_rx_pin[1]`) | 2 |
| Pair 2 data (`i2s_rx_pin[2]`) | 3 |
| Pair 3 data (`i2s_rx_pin[3]`) | 4 |
| Channel count | 2 |
| Rate | 48000 |

The four data-pin defaults (GPIO 1/2/3/4) are a contiguous block chosen to be
free of every default peripheral assignment, so enabling 4/6/8-channel input out
of the box does not self-collide. They are still placeholders: multichannel input
requires wiring one ADC data line per pair, so always let the user assign the
data pins to match their board, then save a preset.

---

## 6. Transport: how to send a command

All commands are **USB control transfers on the vendor interface**, using the
paged protocol of firmware 1.1.7 (full reference in `../commands.md`):

- `bmRequestType`: vendor type (bits 6:5 = `10`). Direction bit 7 = `1` (IN) for
  GET, `0` (OUT) for SET. The same command number serves both directions.
- `bRequest`: the command number within its page (e.g. `0x31` for `INPUT.I2S_RX_CHANNELS`).
- `wIndex`: `(page << 8) | interface`. The page is the high byte (`0x03` for
  `INPUT`); the low byte is the vendor interface number, 2.
- `wValue`: the address only (a stereo pair, a channel, a crosspoint), else 0.
- Data stage: the payload (SET) or the response (GET). Values always travel in
  the payload.

In `pyusb` terms:

```python
def windex(page): return (page << 8) | 2
# SET (host -> device), OUT
dev.ctrl_transfer(0x41, cmd, wValue, windex(page), payload_bytes)
# GET (device -> host), IN, request `length` bytes
resp = dev.ctrl_transfer(0xC1, cmd, wValue, windex(page), length)
```

`0x41` = OUT|vendor|interface, `0xC1` = IN|vendor|interface. The device does not
filter on recipient. A refused SET raises a pipe error (STALL). Read
`SYS.LAST_RESULT` (page `0x01`, command `0x14`) for the `VC_*` reason.

---

## 7. Vendor command reference (I2S-input relevant)

A SET has no status byte. It is ACKed, or it STALLs with a `VC_*` result code
(see "Validation & errors").

| Command | Page | Cmd | Dir | Address (`wValue`) | Size |
|---|---|---|---|---|---|
| `INPUT.I2S_RX_CHANNELS` | 0x03 | 0x31 | GET, SET | none (0) | GET 1, SET 1 |
| `INPUT.I2S_RX_PIN` | 0x03 | 0x30 | GET, SET | stereo pair 0-3 (0 on RP2040) | GET 1, SET 1 |
| `CLOCK.I2S_BCK_PIN` | 0x0C | 0x10 | GET, SET | role 0-1 | GET 1, SET 1 |
| `INPUT.SOURCE` | 0x03 | 0x10 | GET, SET | none (0) | GET 1, SET 1 |
| `INPUT.I2S_RX_RATE` | 0x03 | 0x32 | GET, SET | none (0) | GET 4, SET 4 |
| `MEAS.LEVELS` | 0x09 | 0x10 | GET | none (0) | GET var |
| `INPUT.STATUS` | 0x03 | 0x11 | GET | none (0) | GET 12 |
| `ROUTE.PREAMP` | 0x05 | 0x40 | GET, SET | input channel 0-7 | GET 4, SET 4 |
| `FILTER.BAND` | 0x06 | 0x10 | GET, SET | `(channel << 8) \| band` | GET 16, SET 16 |
| `ROUTE.MATRIX_ROUTE` | 0x05 | 0x10 | GET, SET | `(input << 8) \| output` | GET 8, SET 8 |
| `ROUTE.CHANNEL_NAME` | 0x05 | 0x31 | GET, SET | channel | GET 32, SET 1-32 |
| `STORE.ALL_PARAMS` | 0x02 | 0x40 | GET, SET | none (0) | GET 6136, SET 6136 |

### 7.1 Channel count

`INPUT.I2S_RX_CHANNELS`: SET payload is the channel count `2/4/6/8` (1 byte). The
GET returns the configured count (1 byte).

Setting the count (de)allocates the stereo pairs and, if I2S is the active
source, restarts the input so all pairs re-sync. A count *increase* validates
each newly-activated pair's data pin first (valid GPIO, no peripheral / clock /
cross-pair conflict) and is **rejected** if any clashes; fix the offending pin
then retry. A count other than 2/4/6/8 is `VC_ERR_VALUE`. RP2040 accepts only
`2`. A larger valid count there is `VC_ERR_UNSUPPORTED`.

### 7.2 Per-pair data pins

`INPUT.I2S_RX_PIN`: `wValue` is the stereo-pair index 0..3. The SET payload is
the GPIO number (1 byte), and 0xFF restores that pair's default. The GET returns
that pair's GPIO. A pair past the platform maximum is `VC_ERR_ADDRESS` in either
direction.

The firmware keeps all four data pins mutually distinct and clear of the clocks:
a SET is rejected if the GPIO is invalid, is the BCK or LRCLK pin, is already
used by another peripheral (an output, MCK, the S/PDIF RX pin, or the DAC-mute
pin), or is already another I2S pair's data pin. Pair 0 alone (stereo) hot-swaps
its pin; any higher pair, or a multichannel config, restarts the input so every
pair re-syncs.

### 7.3 BCK (clock) pin

`CLOCK.I2S_BCK_PIN`: `wValue` is the clock-pair role (0 = master/unified pair,
the one described here, or 1 = the separate slave pair of `clock_pins_spec.md`).
The SET payload is the BCK GPIO, and the GET returns it.

LRCLK is always BCK+1, so both BCK and BCK+1 must be valid GPIOs. The SET is
rejected while an I2S *output* is active (`VC_ERR_STATE`) and on a
peripheral conflict (`VC_ERR_PIN_IN_USE`). Changing BCK affects both I2S input
and I2S output (shared clock).

### 7.4 Input source

`INPUT.SOURCE`: SET payload is 1 byte `InputSource`. The GET returns the active one.

`InputSource`: `0` = USB, `1` = S/PDIF, `2` = I2S. Switching to I2S brings up the
input on the configured pins/count/rate and runs a glitch-free prefill handshake
before unmuting.

### 7.5 Input rate

`INPUT.I2S_RX_RATE`: `uint32` Hz (44100 / 48000 / 96000) both ways. The GET
returns the selected I2S rate. The current pipeline rate is the sample-rate
field of `SYS.SYSTEM_STATUS` (u32 at offset 4). Any other rate is `VC_ERR_VALUE`.

In I2S mode the device is the rate authority; set the rate to match your ADC.
The change brackets the input cleanly (mute, divider update, re-sync).

### 7.6 Status + metering

- `MEAS.LEVELS`: combined meter packet (see below).
- `INPUT.STATUS`: byte 1 is the live active input channel count (`uint8`).

The **combined packet** (`MEAS.LEVELS`) is the one to poll for meters + the live
input count in a single round-trip. Layout:

```
offset 0 .. NUM_CHANNELS*2-1 : peaks[NUM_CHANNELS], uint16 LE each
                               (inputs are indices 0..NUM_INPUT_CHANNELS-1)
+0  : cpu0_load   (uint8)
+1  : cpu1_load   (uint8)
+2..+5 : clip_flags (uint32 LE)
+6  : active_input_channels (uint8)   <-- 2/4/6/8, source-aware
```

Total length: RP2350 = 17*2 + 7 = 41 bytes; RP2040 = 7*2 + 7 = 21 bytes.

`active_input_channels` (the `+6` byte, and byte 1 of `INPUT.STATUS`) is
**source-aware**: it reports the I2S count when I2S is the active source, the USB
alt count for USB, and 2 for S/PDIF. Use it to drive your channel-strip layout;
do not assume it equals the I2S count unless I2S is the active source.

Input peak meters are `peaks[0 .. active_input_channels-1]`.

### 7.7 Per-input preamp

`ROUTE.PREAMP`: `wValue` is the input channel index (0..7); payload and reply
are a `float` dB.

Per-input-channel preamp gain, applied at capture before EQ/matrix. Use this to
trim individual ADC inputs. (The 1.1.6 all-channel preamp, opcodes 0x44/0x45, is
retired. Set each channel through `ROUTE.PREAMP`.)

### 7.8 Per-input parametric EQ

`FILTER.BAND`: `wValue = (channel << 8) | band`. SET and GET both carry the
16-byte band record below. The GET returns the whole band.

```c
// wValue = (channel << 8) | band; channel 0..7 = inputs (unified model)
typedef struct __attribute__((packed)) {
    uint8_t  type;     // FilterType (see below)
    uint8_t  bypass;   // exactly 1 = bypassed; anything else = active (reads back 0/1)
    uint16_t qp_x512;  // Linkwitz transform Qp x 512; send it for every band
    float    freq;     // Hz
    float    Q;
    float    gain_db;
} FilterBand;          // 16 bytes
```

`FilterType`: `0` FLAT, `1` PEAKING, `2` LOWSHELF, `3` HIGHSHELF, `4` LOWPASS,
`5` HIGHPASS, `6` NOTCH, `7` ALLPASS, `8` ALLPASS1 (first-order), `9` LOWSHELF1,
`10` HIGHSHELF1. (Crossover band types are output-channel-only; see the crossover
spec.) PEQ on input channels works exactly as on any channel; just put the
input index 0..7 in the `wValue` high byte.

### 7.9 Matrix routing (inputs -> outputs)

`ROUTE.MATRIX_ROUTE`: `wValue = (input << 8) | output`, with input
0..NUM_INPUT_CHANNELS-1 (0..7 on RP2350) and output 0..NUM_OUTPUT_CHANNELS-1
(0..8 on RP2350). SET and GET carry the same 8-byte record:

```c
typedef struct __attribute__((packed)) {
    uint8_t  enabled;      // 0 or 1
    uint8_t  phase_invert; // 0 or 1
    uint16_t reserved;     // 0
    float    gain_db;      // up to +12 dB
} MatrixRoute;             // 8 bytes
```

The matrix is how multichannel input reaches the outputs. With 8 inputs you have
an 8 x 9 grid of crosspoints; enable the ones you need (e.g. input 0 -> S/PDIF 1
L, input 2 -> S/PDIF 2 L, etc.). Output indices: `0..7` are the 4 S/PDIF stereo
slots (L,R interleaved), `8` is PDM (RP2350).

### 7.10 Channel names

`ROUTE.CHANNEL_NAME`: `wValue` is the channel index. The SET takes a 1..32 byte
UTF-8 name (at most 31 bytes are kept). The GET returns 32 bytes.

Default input names are source-aware. I2S input uses numbered stereo pairs in the
same `<prefix> <pair> <L/R>` form as the outputs: `"I2S 1 L"`, `"I2S 1 R"`,
`"I2S 2 L"`, `"I2S 2 R"` ... `"I2S 4 R"`. (For reference, USB input uses discrete
per-channel numbers `"USB 1"`..`"USB 8"`, and S/PDIF uses `"SPDIF L"`/`"SPDIF R"`.)
They switch automatically when the
source changes (only names you have not customized). Custom names persist with
the preset.

### 7.11 Bulk parameter transfer (snapshot / restore)

`STORE.ALL_PARAMS` reads (GET) or writes (SET) the entire DSP state in one
control transfer.

The bulk `WireInputConfig` section carries the I2S input config:
`i2s_rx_pin` (pair 0), `i2s_input_rate`, `i2s_input_channels`, and
`i2s_rx_pin_ext[3]` (pairs 1..3). Per-field `0 = absent / keep the live value`.
On restore, the I2S pin set + BCK are validated as a unit and an inconsistent
config is rejected rather than applied (the live config is retained), so a
round-tripped or hand-built bulk blob can never bring two state machines up on
one GPIO or on a clock pin. Prefer the dedicated commands above for interactive
edits and bulk only for full snapshot/restore.

---

## 8. Validation & errors

A SET is ACKed when applied. A refused SET STALLs, and `SYS.LAST_RESULT` gives
the reason:

| Code | Name | Meaning |
|---|---|---|
| `0x00` | `VC_OK` | applied |
| `0x19` | `VC_ERR_PIN_INVALID` | not a usable GPIO |
| `0x14` | `VC_ERR_VALUE` | channel count other than 2/4/6/8 |
| `0x1A` | `VC_ERR_PIN_IN_USE` | conflicts with another peripheral / clock / I2S pair |
| `0x13` | `VC_ERR_ADDRESS` | no such pair (or BCK role) |
| `0x15` | `VC_ERR_UNSUPPORTED` | count exceeds the platform max |
| `0x18` | `VC_ERR_STATE` | BCK change rejected while an I2S output is active |

Pin rules the firmware enforces (so your app does not have to, but should mirror
for good UX):

- A GPIO is invalid if it is reserved (UART/power/LED) or out of range for the
  platform.
- An I2S data pin may not equal the BCK or LRCLK pin (checked even while I2S is
  inactive, because they coexist once the input runs).
- The four data pins must be mutually distinct.
- A data pin may not equal an output pin, MCK, the S/PDIF RX pin, or the DAC-mute
  pin that is currently in use.
- Raising the channel count validates each newly-activated pair's pin and rejects
  the increase if any clashes.

Recommended UX: when a SET is refused, surface the reason and keep your UI
in sync by issuing the matching GET (the device did not change state on a
rejected SET).

---

## 9. Persistence

The I2S input config participates in DSPi's normal persistence:

- **Per preset (flash):** channel count, all four data pins, BCK pin, rate, and
  every per-input PEQ/preamp/name/matrix crosspoint are captured when you save a
  preset and restored on load. Saving a preset is how a user makes a wired
  multichannel setup permanent.
- **Device-global (flash directory):** in INDEPENDENT output-config mode the
  physical IO (pins, BCK, count, rate) is stored once and applied at boot.
- **Bulk (`WireInputConfig`):** for full host-driven snapshot/restore.

You do not need to manage versions; just use `STORE.PRESET_SAVE` / `STORE.PRESET_LOAD` and the
bulk commands. Restores are validated (see 7.11) so a stale or cross-device
config degrades gracefully (the bad fields are ignored) rather than producing a
broken or clashing setup.

---

## 10. Host notifications (push)

DSPi pushes asynchronous events so your UI can stay live without polling. The
ones relevant here:

| Event | ID | When it fires |
|---|---|---|
| `NOTIFY_EVT_INPUT_FORMAT` | `0x05` | active input channel count changes |
| `NOTIFY_EVT_PARAM_CHANGED` | `0x02` | any single parameter changed (offset + bytes) |

`NOTIFY_EVT_INPUT_FORMAT` fires whenever the **active** count changes: a USB alt
change, an **input-source switch** (e.g. into I2S), and a **live I2S channel-count
change**. Its payload carries the new count. Treat it as "relayout your channel
strips now"; re-read the count (or take it from the event) and rebuild the view.

`NOTIFY_EVT_PARAM_CHANGED` carries a wire offset + value for the field that
changed (e.g. an I2S pin, the count, a crosspoint). Use it to keep multiple
clients or views consistent. The dedicated SETs above also emit these so a
second app sees your edits.

If your app does not subscribe to notifications, polling `MEAS.LEVELS`
for meters already includes the live count, so you will pick up
changes on the next poll.

---

## 11. App workflow patterns

### 11.1 Capability detection (do this first)

Do not hard-code 8 channels. Probe what the device supports rather than assuming:

```
# Attempt the count you want; the result tells you if the part supports it.
result = SET(INPUT.I2S_RX_CHANNELS, 8)       # None on ACK, else the VC_* code
if result == VC_ERR_UNSUPPORTED:
    # stereo-only part (RP2040): cap the UI at 2 channels / 1 pair
    max_channels = 2
elif result is None:
    max_channels = 8
```

`VC_ERR_UNSUPPORTED` from a count SET means "this part cannot do that many
pairs"; treat it as the capability signal. (You can also read the platform from
byte 0 of `SYS.PLATFORM` if you prefer not to issue a probing SET.)

### 11.2 Enable a multichannel I2S setup

```
# 1. Pick the channel count
result = SET(INPUT.I2S_RX_CHANNELS, count)      # 2/4/6/8
assert result is None                           # ACK

# 2. Assign data pins for the active pairs (pair 0 + (count/2 - 1) extras)
for pair in range(count // 2):
    result = SET(INPUT.I2S_RX_PIN, gpio_for[pair], wValue=pair)
    if result is not None: show_error(pair, result); abort()

# 3. (optional) BCK pin + rate
SET(CLOCK.I2S_BCK_PIN, bck_gpio, wValue=0)      # only if changing from default
SET(INPUT.I2S_RX_RATE, u32(48000))

# 4. Make I2S the active source
SET(INPUT.SOURCE, 2)                            # INPUT_SOURCE_I2S
# -> NOTIFY_EVT_INPUT_FORMAT arrives; relayout to `count` strips

# 5. Route inputs to outputs
for (inp, outp, gain) in routing:
    SET(ROUTE.MATRIX_ROUTE, route(enabled=1, gain_db=gain), wValue=(inp << 8) | outp)

# 6. Persist
SET(STORE.PRESET_SAVE, wValue=slot)             # zero-length SET
```

### 11.3 Relayout on the fly

```
on NOTIFY_EVT_INPUT_FORMAT(count):
    rebuild_channel_strips(count)          # show inputs 0..count-1
    grey_out_stereo_only_fx = (count > 2)  # loudness/leveller/crossfeed
# (or poll MEAS.LEVELS and read the +6 byte)
```

### 11.4 Per-input processing

```
# Trim input 2 by -3 dB, add a 1 kHz bell on input 2 band 0:
SET(ROUTE.PREAMP, f32(-3.0), wValue=2)
SET(FILTER.BAND, band(type=PEAKING, freq=1000, Q=1.0, gain_db=+4), wValue=(2 << 8) | 0)
# Name it:
SET(ROUTE.CHANNEL_NAME, b"Sub L", wValue=2)
```

### 11.5 Meter the inputs

```
loop:
    packet = GET(MEAS.LEVELS)
    count  = packet[NUM_CHANNELS*2 + 6]
    for ch in range(count):
        peak = u16le(packet, ch*2)
        draw_meter(ch, peak)
```

### 11.6 Changing count live

You can change the count while I2S is the active source; the device restarts the
input transparently and re-syncs. Just SET `INPUT.I2S_RX_CHANNELS` to the new count and
relayout on the resulting `NOTIFY_EVT_INPUT_FORMAT`. A *raise* whose new pins
clash is rejected (fix the pins first); a *lower* always succeeds.

---

## 12. Gotchas and edge cases

- **RP2040 is stereo-only.** Cap your UI; `4/6/8` and pairs `1..3` are rejected.
- **Extra data pins need real wiring.** The GPIO 1/2/3/4 defaults are
  placeholders (a collision-free block, but almost never where your ADC is
  wired); assign them to your board before enabling 4/6/8, then save a preset.
- **Order tip.** Set the count first, then the pins. The defaults are distinct
  and free, so a raise never trips on a stale pin; if you set pins first while
  the count is still 2, the extra pairs are inactive and not yet validated
  against peripherals until you raise the count.
- **Stereo-only FX bypass.** Loudness, leveller, and crossfeed are inactive
  whenever the active input count is greater than 2 (by design); reflect that in
  the UI.
- **Source-aware count.** `active_input_channels` follows the *active* source.
  After switching away from I2S it reports the new source's count, not the I2S
  count you configured.
- **BCK is shared with I2S output.** Changing BCK is rejected while an I2S output
  is active; the data pins must always avoid BCK and LRCLK.
- **Alignment is guaranteed.** You do not need to (and cannot) phase-align the
  channels yourself; the firmware starts all pairs in sync.
- **A rejected SET does not change state.** Re-issue the matching GET to resync
  your UI after a STALL.

---

## 13. Quick reference

### Commands

| Command | Page | Cmd | Dir | Address (`wValue`) | Size | Data |
|---|---|---|---|---|---|---|
| `INPUT.I2S_RX_CHANNELS` | 0x03 | 0x31 | GET, SET | none (0) | GET 1, SET 1 | count 2/4/6/8 |
| `INPUT.I2S_RX_PIN` | 0x03 | 0x30 | GET, SET | pair | GET 1, SET 1 | gpio (0xFF = default) |
| `CLOCK.I2S_BCK_PIN` | 0x0C | 0x10 | GET, SET | role (0) | GET 1, SET 1 | gpio |
| `INPUT.SOURCE` | 0x03 | 0x10 | GET, SET | none (0) | GET 1, SET 1 | 2 = I2S |
| `INPUT.I2S_RX_RATE` | 0x03 | 0x32 | GET, SET | none (0) | GET 4, SET 4 | u32 Hz (selected) |
| `SYS.SYSTEM_STATUS` | 0x01 | 0x40 | GET | none (0) | GET 16 | live rate u32 at offset 4 |
| `MEAS.LEVELS` | 0x09 | 0x10 | GET | none (0) | GET var | meter packet |
| `INPUT.STATUS` | 0x03 | 0x11 | GET | none (0) | GET 12 | active input count at byte 1 |
| `ROUTE.PREAMP` | 0x05 | 0x40 | GET, SET | input channel | GET 4, SET 4 | float dB |
| `FILTER.BAND` | 0x06 | 0x10 | GET, SET | `(ch << 8) \| band` | GET 16, SET 16 | band record |
| `ROUTE.MATRIX_ROUTE` | 0x05 | 0x10 | GET, SET | `(in << 8) \| out` | GET 8, SET 8 | crosspoint |
| `ROUTE.CHANNEL_NAME` | 0x05 | 0x31 | GET, SET | channel | GET 32, SET 1-32 | name |
| `STORE.ALL_PARAMS` | 0x02 | 0x40 | GET, SET | none (0) | GET 6136, SET 6136 | bulk blob |

### Enums / constants

```
InputSource:   USB=0  SPDIF=1  I2S=2
Channel count: 2 / 4 / 6 / 8   (1..4 stereo pairs)
Pairs:         pair p -> input channels 2p, 2p+1
VC result:     OK=0x00  ADDRESS=0x13  VALUE=0x14  UNSUPPORTED=0x15  STATE=0x18
               PIN_INVALID=0x19  PIN_IN_USE=0x1A
NOTIFY_EVT:    PARAM_CHANGED=0x02  INPUT_FORMAT=0x05
RP2350:        NUM_INPUT_CHANNELS=8  NUM_OUTPUT_CHANNELS=9  pairs<=4
RP2040:        NUM_INPUT_CHANNELS=2  NUM_OUTPUT_CHANNELS=5  pairs=1
```

---

*See also: `current_architecture.md` (I2S Input section) for firmware internals,
and the per-channel preamp, crossover, interrupt-interface, and master-volume
specs for the shared subsystems referenced here.*
