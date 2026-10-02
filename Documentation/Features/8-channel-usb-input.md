# Multichannel USB Input — App Developer Integration Guide

*Last updated: 2026-09-17*

> **Updated for the unified channel model (firmware wire V16 / slot V21).** Three things
> changed from the original 8-channel-only design and supersede older framing anywhere in this
> guide:
> 1. **Inputs are 2 / 4 / 6 / 8** (RP2350), selected by the host audio format (USB alt). Not just
>    2-or-8. All multichannel formats are 48 kHz; 4ch supports 16/24-bit, 6ch/8ch 16-bit.
> 2. **There is no "master" channel.** Every *input* channel is now a first-class channel with
>    its own 10-band PEQ and peak/clip metering (no crossover). Output channels keep PEQ +
>    crossover + gain/delay/mute. Channel indices are **inputs first (0..7), then outputs
>    (8..16)** on RP2350.
> 3. **The active input count is live** (2/4/6/8) — read it from the status packet (Section 7.7)
>    and lay out your mixer/sidebar to match; a push notification fires the instant it changes.
>
> Compatibility with pre-V16/V21 firmware is intentionally broken (no migration). Detect the
> versions (Section 4) before using any of this.

This is the **practical, start-to-finish guide** for adding multichannel USB input support to a
host application (desktop or web) that controls a DSPi device. It assumes you already talk to
the device over USB vendor control transfers; if not, read "USB Transport Basics" first.

If you want the firmware-internals / on-the-wire rationale, see the companion
`usb_8ch_input_spec.md`. This document is the app side.

---

## Table of Contents

1. [The 60-second mental model](#1-the-60-second-mental-model)
2. [The two halves: streaming vs configuration](#2-the-two-halves-streaming-vs-configuration)
3. [USB transport basics](#3-usb-transport-basics)
4. [Capability detection — does this device do 8 channels?](#4-capability-detection)
5. [Channel maps you must memorize](#5-channel-maps)
6. [Signal flow in 8-channel mode](#6-signal-flow-in-8-channel-mode)
7. [The commands you need](#7-the-commands-you-need)
8. [Reading & writing the full state in one shot (bulk transfer)](#8-bulk-transfer)
9. [The V16 wire format byte map](#9-the-v16-wire-format-byte-map)
10. [Step-by-step: add 8-channel support to your app](#10-step-by-step)
11. [Worked routing recipes](#11-worked-routing-recipes)
12. [Persistence (presets)](#12-persistence-presets)
13. [Backward / forward compatibility rules](#13-compatibility-rules)
14. [Gotchas & FAQ](#14-gotchas--faq)
15. [Quick reference tables](#15-quick-reference-tables)

---

## 1. The 60-second mental model

A DSPi on **RP2350** can receive **2, 4, 6, or 8 channels** of audio from the host (the host
picks the format). Each active **input** channel has its own **10-band PEQ + peak/clip meter**.
The inputs then go through a **matrix mixer** (8 inputs × 9 outputs), and each of the 9 outputs
(4 S/PDIF stereo pairs + 1 PDM sub) has its own **EQ, crossover, gain, delay, mute**. So the
device is an N-in / 8-out (plus sub) DSP engine: think active multi-way crossovers, multichannel
room correction, or surround processing.

- **Multichannel (>2) is RP2350-only.** RP2040 has only 2 S/PDIF pairs (4 output channels) and
  stays stereo. Your app must detect this (Section 4) and only show multichannel UI on RP2350.
- **Multichannel input is fixed at 48 kHz.** 4ch supports 16-bit (alt 3) and packed
  24-bit (alt 6); 6ch/8ch remain 16-bit. Stereo input keeps its 16/24-bit ×
  44.1/48/96 kHz options. On ALSA, open 4ch/24-bit as `S24_3LE`, 4 channels,
  48000 Hz. Alt 6 needs at most 49 × 4 × 3 = 588 bytes per packet and uses
  the existing 788-byte endpoint/ring allocation. No 6ch/24-bit or 8ch/24-bit
  format is advertised.
- **The active input count is 2/4/6/8 and is live.** It follows whichever audio format the host
  selected. Read it from the status packet (Section 7.7) and a push event signals changes — lay
  out exactly that many input strips.
- Inputs have **PEQ + metering but no crossover**; outputs have PEQ + crossover + gain/delay/mute.
- In multichannel mode (>2 active inputs) the device **bypasses** the stereo-only effects
  (loudness, volume leveller, crossfeed). Per-input EQ and per-output EQ/crossover/gain/delay
  still apply. Don't present loudness/leveller/crossfeed as affecting a multichannel stream.

---

## 2. The two halves: streaming vs configuration

This trips up everyone. There are **two independent things**, owned by two different systems:

| Concern | Who controls it | How |
|---|---|---|
| **Streaming** — actually sending N channels of PCM to the device | The OS audio stack / your audio-playback code | The device exposes multichannel **formats** (2 / 4 / 6 / 8 ch @ 48 kHz, plus the stereo 16/24-bit formats), each a USB Audio "alt setting". You select one the way you select any output format: in OS sound settings, or by opening the device in that format via WASAPI (Windows) / CoreAudio (macOS). |
| **Configuration** — routing, EQ, gain, delay, presets | Your control app | USB **vendor control transfers** (this guide). |

**You do NOT switch the device's channel count with a vendor command.** There is no
"set channel count" command. The device *advertises* all the formats (on RP2350); the active
input count follows whichever format the OS/audio-engine opened. Your control app **reads** the
live count from the status packet (Section 7.7) and configures the matrix/DSP, which is live
whenever a matching stream is playing.

Practical consequence: you can configure the matrix at any time, even while the device is
playing stereo — your settings for inputs ≥2 simply do nothing until a multichannel stream is
playing. Configure freely; lay out your input strips from the **live active count**, and don't
otherwise gate config on "is it currently streaming".

---

## 3. USB transport basics

All control happens on the **vendor interface (interface number 2)**, class 0xFF, bound to
WinUSB on Windows (via MS OS 2.0 descriptors — no Zadig needed) and accessible via libusb on
macOS/Linux.

- **Vendor ID / Product ID:** `0x2E8B` / `0xFEAA`.
- **Find the device (Windows):** enumerate by the published DeviceInterfaceGUID
  `{9D9B8609-E6D1-4FF0-92AF-403119CB7692}` (SetupDiGetClassDevs → WinUsb_Initialize). On
  macOS/Linux use libusb by VID/PID and claim interface 2.
- **Every control transfer:**
  - `bmRequestType` = `0x40` for SET (host→device, OUT) or `0xC0` for GET (device→host, IN).
    (Bit layout: direction | vendor(0x20) | device-recipient.)
  - `bRequest` = the command code (Section 7).
  - `wValue` = command-specific (often a channel/output index, or an encoded field).
  - `wIndex` = `2` (the vendor interface number).
  - `wLength` / data = the payload.
- **All multi-byte values are little-endian.** All `float` are IEEE-754 single precision.
- **dB convention:** the device converts dB → linear as `linear = 10^(dB/20)`.

> **Do not modify the audio interfaces (0/1) or the device's `usb_device.c`-equivalent.** You
> only talk to interface 2. The audio streaming format is negotiated by the OS audio driver.

---

## 4. Capability detection

Before showing any 8-channel UI, confirm the connected device supports it. The authoritative
signal is the **bulk-params header** (Section 8): read the full state once with
`REQ_GET_ALL_PARAMS` (0xA0) and inspect the 16-byte header.

```
struct WireHeader {            // first 16 bytes of the GET_ALL_PARAMS response
  u8  format_version;          // 16 == this firmware (unified channel model)
  u8  platform_id;             // 0 = RP2040, 1 = RP2350
  u8  num_channels;            // 7 (RP2040) or 17 (RP2350)
  u8  num_output_channels;     // 5 (RP2040) or 9 (RP2350)
  u8  num_input_channels;      // device MAX inputs: 2 (RP2040) or 8 (RP2350)  <-- THE FLAG
  u8  max_bands;               // 12
  u16 payload_length;          // total bytes in this packet (5864 for V16)
  u16 fw_version_major;
  u16 fw_version_minor;
  u32 reserved;
};
```

`num_input_channels` is the device's **maximum** input count (8 on RP2350). The **live active**
count (2/4/6/8, which follows the host audio format) is a separate value — read it from the
status packet (Section 7.7), not from this header.

**Decision rule:**

```
supports_multichannel = (header.platform_id == 1)         // RP2350
                  && (header.format_version == 16)         // unified channel model
                  && (header.num_input_channels == 8);     // device supports 8 inputs
```

Because compatibility is intentionally broken at V16, treat `format_version != 16` as
"incompatible firmware" (don't try to parse it). Pair this with the slot version (V21) if you
read presets.

If false, show your existing stereo UI (2 matrix inputs). If true, show 8 matrix inputs and the
8-channel features below.

You can also call `REQ_GET_PLATFORM` (0x7F) for a platform string, but the header fields above
are the canonical machine-readable capability check.

---

## 5. Channel maps

There are **three** index spaces. Keep them straight.

### USB input channels (what the host sends), 7.1 order

| USB ch | 7.1 role | Matrix input index |
|---|---|---|
| 0 | Front Left | 0 |
| 1 | Front Right | 1 |
| 2 | Front Center | 2 |
| 3 | LFE | 3 |
| 4 | Back Left | 4 |
| 5 | Back Right | 5 |
| 6 | Side Left | 6 |
| 7 | Side Right | 7 |

(Channels 0/1 are the same "USB L/R" bus used by stereo and by S/PDIF/I2S input sources. Each
input — 0..7 — is its own EQ/metering channel; see EQ indices below.)

### Matrix output channels (0-8) → physical outputs

The **matrix output index** is 0-8 (used by routing/gain/delay/mute commands). The **EQ/channel
index** for an output is `output index + CH_OUT_1`, where `CH_OUT_1 = NUM_INPUT_CHANNELS` = **8 on
RP2350** (2 on RP2040). So on RP2350 outputs occupy channel indices **8..16**.

| Output index | Physical | EQ/channel index (RP2350) |
|---|---|---|
| 0 | S/PDIF 1 L | 8 (`CH_OUT_1`) |
| 1 | S/PDIF 1 R | 9 |
| 2 | S/PDIF 2 L | 10 |
| 3 | S/PDIF 2 R | 11 |
| 4 | S/PDIF 3 L | 12 |
| 5 | S/PDIF 3 R | 13 |
| 6 | S/PDIF 4 L | 14 |
| 7 | S/PDIF 4 R | 15 |
| 8 | PDM sub | 16 (`CH_OUT_9_PDM`) |

### EQ / channel indices (RP2350: 0-16)

Channels are **inputs first, then outputs**:

- `0..7` = the **per-input EQ + metering** channels (input 0..7). Inputs have PEQ + peak/clip
  meters but **no crossover**. Only the first *N* (= active input count) carry audio; EQ/meters
  for inputs ≥ N are inert.
- `8..16` = the **per-output EQ + crossover** channels for outputs `0..8`. So
  **per-output EQ channel = output index + 8** (= `output + CH_OUT_1`).

On RP2040 (2 inputs + 5 outputs) the same rule gives inputs `0..1` and outputs `2..6`
(`CH_OUT_1 = 2`). Always compute output EQ channel as `output + CH_OUT_1`; never hardcode `+2`
or `+8` — read `num_input_channels` from the bulk header and use it as `CH_OUT_1`.

---

## 6. Signal flow in multichannel mode

```
USB ch0..N-1 ─► per-input preamp (0xD0) ─► per-input EQ + meter ─► ┌────────────────────┐
                                            (0x42, ch 0..N-1;      │  MATRIX MIXER 8×9   │
                                             peaks/clip per input) │  crosspoint[in][out]│
                                                                   │  gain + phase       │
                                                                   └─────────┬──────────┘
                                                        │ (per output)
                                       ┌────────────────┼───────────────┐
                                       ▼                ▼               ▼
                                  Output EQ (0x42)  Output gain    Output delay
                                  + crossover       (0x74)         (0x78)
                                       │                │               │
                                       └────────────────┴───────────────┘
                                                        ▼
                                              Mute (0x76) + master volume
                                                        ▼
                                          S/PDIF 1-4 (8 ch) + PDM sub
```

What **still applies** in multichannel mode: per-input preamp, **per-input EQ + metering** (each
active input, channels `0..N-1`), the matrix, and all per-output processing (EQ + crossover +
gain/delay/mute) and master volume. What is **bypassed** (works only in stereo mode): loudness,
volume leveller, and crossfeed — they are inherently stereo. (In stereo mode, input EQ on
channels 0/1 is exactly what older firmware called "master EQ".)

---

## 7. The commands you need

All on interface 2 (`wIndex = 2`). `dir` is the `bmRequestType`: `OUT`=0x40, `IN`=0xC0.

### Capability & whole-state
| Code | Name | dir | wValue | Data | Notes |
|---|---|---|---|---|---|
| 0xA0 | `REQ_GET_ALL_PARAMS` | IN | 0 | 5864 B `WireBulkParams` (V16) | Read everything (Section 8) |
| 0xA1 | `REQ_SET_ALL_PARAMS` | OUT | 0 | 5864 B `WireBulkParams` (V16) | Write everything (Section 8) |
| 0x7F | `REQ_GET_PLATFORM` | IN | 0 | platform info | Optional; header is canonical |

### Matrix routing (inputs 0-7 on RP2350)
| Code | Name | dir | wValue | Data | Notes |
|---|---|---|---|---|---|
| 0x70 | `REQ_SET_MATRIX_ROUTE` | OUT | 0 | 9 B `MatrixRoutePacket` | One crosspoint |
| 0x71 | `REQ_GET_MATRIX_ROUTE` | IN | `(input<<8)\|output` | 9 B `MatrixRoutePacket` | One crosspoint |

`MatrixRoutePacket` (9 bytes): `u8 input` (0-7), `u8 output` (0-8), `u8 enabled`,
`u8 phase_invert`, `float gain_db` (-60..+12). **The only change for 8 channels: `input` may be
0-7** (was 0-1). Out-of-range indices are silently ignored.

### Per-input preamp (channels 0-7 on RP2350)
| Code | Name | dir | wValue | Data | Notes |
|---|---|---|---|---|---|
| 0xD0 | `REQ_SET_PREAMP_CH` | OUT | channel (0-7) | `float` dB | Per-input gain |
| 0xD1 | `REQ_GET_PREAMP_CH` | IN | channel (0-7) | `float` dB | |

### Per-output controls (output 0-8)
| Code | Name | dir | wValue | Data |
|---|---|---|---|---|
| 0x72 / 0x73 | SET/GET `OUTPUT_ENABLE` | OUT/IN | output | `u8` 0/1 |
| 0x74 / 0x75 | SET/GET `OUTPUT_GAIN` | OUT/IN | output | `float` dB (-60..+12) |
| 0x76 / 0x77 | SET/GET `OUTPUT_MUTE` | OUT/IN | output | `u8` 0/1 |
| 0x78 / 0x79 | SET/GET `OUTPUT_DELAY` | OUT/IN | output | `float` ms (0..170 RP2350) |

### Per-input AND per-output EQ (same command; channel = the EQ index)
| Code | Name | dir | wValue | Data |
|---|---|---|---|---|
| 0x42 | `REQ_SET_EQ_PARAM` | OUT | 0 | 16 B `EqParamPacket` |
| 0x43 | `REQ_GET_EQ_PARAM` | IN | `(channel<<8)\|(band<<4)\|param` | 4 B |

`EqParamPacket` (16 bytes): `u8 channel`, `u8 band` (0-9), `u8 type`, `u8 reserved=0`,
`float freq`, `float Q`, `float gain_db`. The **channel** field is the unified channel index:

- **Input EQ:** `channel` = input index `0..N-1` (the active inputs). Same command, no new
  opcode — inputs are just channels now. (Crossover commands are rejected for input channels.)
- **Output EQ:** `channel` = `output + CH_OUT_1` (= `output + 8` on RP2350, `+2` on RP2040).

### 7.7 Live active-input count + per-input metering (status)
| Code | Name | dir | wValue | Data |
|---|---|---|---|---|
| 0x30 | `REQ_GET_STATUS` | IN | 9 | combined status packet (see below) |
| 0x30 | `REQ_GET_STATUS` | IN | 23 | `u32` active input channel count (2/4/6/8) |
| 0x31 | `REQ_CLEAR_CLIPS` | IN | 0 | `u32` cleared clip flags |

The **combined status packet** (`wValue=9`) is what you poll for meters; it now also carries the
active input count:
- `peaks[NUM_CHANNELS]` — `u16` each (channel-indexed; inputs `0..N-1` then outputs). Inactive
  inputs read 0.
- `u8 cpu0_load`, `u8 cpu1_load`
- `u32 clip_flags` — **4 bytes** (one bit per channel; bit `ch` set = that channel clipped).
- `u8 active_input_channels` — the live count (2/4/6/8). **Use this to lay out your mixer/
  sidebar** (show exactly this many input strips).

**Push notification.** When the host switches the audio format (alt), the device sends a
`NOTIFY_EVT_INPUT_FORMAT` (0x05) event on the notification endpoint:
`[ver=2, evt=0x05, flags=0, seq, channels, 0,0,0]`. Re-read the active count and relayout
immediately rather than waiting for the next status poll.

### Persistence
| Code | Name | dir | wValue | Data |
|---|---|---|---|---|
| 0x90 | `REQ_PRESET_SAVE` | IN | slot (0-9) | result byte |
| 0x91 | `REQ_PRESET_LOAD` | IN | slot (0-9) | result byte |

> There are many more commands (loudness, crossfeed, leveller, input source, I2S, etc.). They
> are unchanged by this feature and are documented in their own specs. For multichannel work you
> need the tables above.

---

## 8. Bulk transfer

For anything beyond a single tweak, use the **bulk parameter transfer** rather than dozens of
small commands. It moves the entire DSP state in one control transfer.

- **GET** (`0xA0`, dir IN): the device returns a `WireBulkParams` (**5864 bytes for V16** —
  always trust `header.payload_length`). Read this on connect, parse the header, then parse the
  sections you care about.
- **SET** (`0xA1`, dir OUT): you send a full `WireBulkParams`. V16 is **all-or-nothing**: the
  firmware accepts the payload only when it matches exactly, so you MUST send a complete V16
  packet:
  - `header.format_version = 16`
  - `header.payload_length = 5864` (== `sizeof(WireBulkParams)` for V16)
  - matching `num_channels` / `num_input_channels` / `num_output_channels` for the platform
  - send all 5864 bytes
  - There is no partial/legacy acceptance anymore (the old per-version section gating is gone).
    A mismatched version, length, or channel count is rejected outright. Always GET first, mutate,
    then SET the whole buffer back.

Implementation notes:
- Allocate an **8192-byte** buffer for the transfer (the firmware's buffer is 8192; the V16
  payload is 5864). EP0 chunks the transfer in 64-byte packets automatically.
- The round-trip pattern: GET → mutate fields in your local copy → SET the whole thing. This is
  far more robust than issuing individual commands for a large config.
- After a SET, the device recalculates filters/delays; allow a few ms before reading back.

---

## 9. The V16 wire format byte map

`WireBulkParams` is a **packed** struct (no padding). All offsets are absolute byte offsets from
the start of the packet. V16 is a **direct, flat** layout (the old V15 `input_ext` tail is gone):
matrix and preamp now hold **all 8 inputs inline**, and the per-channel arrays are sized for the
full 17-channel model. Total = **5864 bytes** (RP2350). The offsets below are for RP2350
(`WIRE_MAX_CHANNELS=17`, `WIRE_MAX_INPUT_CHANNELS=8`); they are identical on RP2040 because the
wire format is platform-independent (channels ≥ the platform's count are zero-padded).

| Offset | Size | Section | Contents |
|---|---|---|---|
| 0 | 16 | `header` | version/platform/counts/length (Section 4) |
| 16 | 16 | `global` | preamp(legacy ch0), master EQ-bypass flag, loudness |
| 32 | 16 | `crossfeed` | crossfeed params |
| 48 | 16 | `legacy` | legacy 3-ch gain/mute |
| 64 | 68 | `delays` | `float delay_ms[17]` |
| **132** | **576** | **`crosspoints[8][9]`** | **matrix all 8 inputs** (row-major) |
| 708 | 108 | `outputs[9]` | per-output enable/mute/gain/delay |
| 816 | 8 | `pins` | output GPIO pins |
| 824 | 3264 | `eq[17][12]` | EQ bands, row-major by **channel** (inputs 0-7 then outputs 8-16) |
| 4088 | 544 | `channel_names[17][32]` | input + output names |
| 4632 | 16 | `i2s_config` | |
| 4648 | 16 | `leveller` | |
| **4664** | **32** | **`preamp`** | **`float preamp_db[8]`** (all inputs) |
| 4696 | 16 | `master_volume` | |
| 4712 | 16 | `input_config` | input source, pins, rate |
| 4728 | 16 | `lg_sound_sync` | |
| 4744 | 16 | `user_volume` | |
| 4760 | 16 | `dac_hw_mute` | |
| 4776 | 1088 | `crossovers[17][4]` | per-output crossover bands (input rows unused) |

### Element layouts

```
WireCrosspoint   (8 bytes):  u8 enabled; u8 phase_invert; u8 reserved[2]; float gain_db;
WireOutputChannel(12 bytes): u8 enabled; u8 mute; u8 reserved[2]; float gain_db; float delay_ms;
WireBandParams   (16 bytes): u8 type; u8 bypass; u8 reserved[2]; float freq; float q; float gain_db;
```

### Reading a crosspoint (input, output) from a V16 buffer

```c
// Returns byte offset of the WireCrosspoint for (input 0-7, output 0-8).
// crosspoints[8][9], row-major, directly inline — no special case for inputs >= 2.
size_t crosspoint_offset(int input, int output) {
    return 132 + (input * 9 + output) * 8;
}
```

### Reading a per-input preamp (channel) from a V16 buffer

```c
size_t preamp_offset(int channel) {            // channel 0-7
    return 4664 + channel * 4;                 // preamp.preamp_db[8], direct
}
```

(These exact offsets are also what the device reports in its push notifications, so live-update
parsing and bulk parsing use the same math. The `eq` channel index is the unified index: inputs
`0..7`, outputs `output + 8`.)

---

## 10. Step-by-step

### A. On connect
1. `REQ_GET_ALL_PARAMS` (0xA0) → parse `WireHeader`.
2. Compute `supports_8ch_input` (Section 4). Cache it.
3. Parse `outputs[9]`, `eq[][]`, `crosspoints[2][9]`, and (if 8ch) `input_ext` into your model.

### B. Render the matrix UI
- If `supports_8ch_input`, draw an **8-row × 9-column** grid (inputs 0-7 × outputs 0-8). Label
  rows with the 7.1 names (Section 5). Otherwise draw the 2×9 grid.
- Each cell = a crosspoint: enabled toggle, gain (dB), phase invert.

### C. Apply a change
- **Single tweak:** `REQ_SET_MATRIX_ROUTE` (0x70) with a `MatrixRoutePacket` (input 0-7).
- **Bulk apply:** mutate your local `WireBulkParams`, set `format_version=16`,
  `payload_length=5864`, `REQ_SET_ALL_PARAMS` (0xA1) with all 5864 bytes.

### D. Offer a "1:1" default for newcomers
The device's factory routing is **stereo only** (FL→S/PDIF1 L, FR→S/PDIF1 R). It does **not**
auto-populate 8-channel routing — so out of the box, an 8-channel stream only produces sound on
S/PDIF 1. Give users a one-click "1:1 / direct" button that routes input *i* → output *i* for
i=0..7. (See recipe 11.1.)

### E. Per-output DSP
For each of outputs 0-7 (the 8 speaker channels), expose EQ (channels 2-9), gain (0x74),
delay (0x78), mute (0x76). Output 8 is the PDM sub (EQ channel 10).

### F. Save
`REQ_PRESET_SAVE` (0x90) with the target slot in `wValue`. The 8-channel routing + preamp are
persisted (the device stores them in its flash preset, V20).

---

## 11. Worked routing recipes

Each recipe = a set of crosspoints + output enables. Send via `REQ_SET_MATRIX_ROUTE` per cell,
or build them into a bulk SET.

### 11.1 Direct 1:1 (8-in → 8-out)
```
for i in 0..7:
    SET_MATRIX_ROUTE { input=i, output=i, enabled=1, phase_invert=0, gain_db=0 }
    SET_OUTPUT_ENABLE(output=i, 1)
SET_OUTPUT_ENABLE(output=8, 0)   // PDM off unless you want a sub
```
FL→S/PDIF1L, FR→S/PDIF1R, FC→S/PDIF2L, LFE→S/PDIF2R, BL→S/PDIF3L, BR→S/PDIF3R, SL→S/PDIF4L,
SR→S/PDIF4R.

### 11.2 7.1 → 4 stereo S/PDIF pairs, LFE to the PDM sub
```
FL(0)→Out0, FR(1)→Out1            // pair 1 = fronts
FC(2)→Out2, FC(2)→Out3           // pair 2 = center duplicated to both, -3 dB each
BL(4)→Out4, BR(5)→Out5           // pair 3 = surrounds back
SL(6)→Out6, SR(7)→Out7           // pair 4 = surrounds side
LFE(3)→Out8 (0 dB)               // sub on PDM
Enable outputs 0-8; LPF EQ on EQ-channel 10 for the sub
```

### 11.3 Active 2-way stereo from an 8-channel host (bi-amp using extra channels)
```
FL(0)→Out0 (tweeter L, HPF on EQ-ch 2)
FR(1)→Out1 (tweeter R, HPF on EQ-ch 3)
FC(2)→Out2 (woofer L, LPF on EQ-ch 4)   // host sends woofer feed on ch 2/3
LFE(3)→Out3 (woofer R, LPF on EQ-ch 5)
```

### 11.4 Downmix 8→2 (sum a multichannel source to one stereo pair)
```
FL(0)→Out0, FC(2)→Out0(-3dB), BL(4)→Out0(-3dB), SL(6)→Out0(-3dB)
FR(1)→Out1, FC(2)→Out1(-3dB), BR(5)→Out1(-3dB), SR(7)→Out1(-3dB)
```
(Multiple inputs to one output sum; trim gains to avoid clipping.)

---

## 12. Persistence (presets)

- The device has 10 preset slots (0-9). `REQ_PRESET_SAVE`(0x90, wValue=slot) /
  `REQ_PRESET_LOAD`(0x91, wValue=slot) capture the **full** DSP state, including the matrix,
  per-input preamp, and **per-input EQ** (stored in flash slot format **V21**; the slot spans 2
  flash sectors on RP2350).
- **No migration.** A slot saved by pre-V21 firmware fails validation and loads **factory
  defaults** (clean, never garbage). Likewise, upgrading to V21 firmware abandons old presets
  (the flash region moved); the device boots to factory defaults.
- Saving on RP2350 then loading on an RP2040 is not a concern (different hardware, separate
  flash); cross-device transfer is via your app + the bulk format.

---

## 13. Compatibility rules

Compatibility is **intentionally broken at V16/V21** — there is no forward/back-compat path.

- **Detect, don't assume.** Read the header (Section 4). Treat anything other than
  `format_version == 16` as incompatible firmware and do not parse its bulk payload. On RP2040,
  `num_input_channels == 2` (stereo-only) — hide multichannel UI.
- **Bulk SET is all-or-nothing.** Send a complete V16 packet (`format_version=16`,
  `payload_length=5864`, matching channel counts). A mismatch is rejected outright; there is no
  partial/legacy section gating anymore.
- **Never hardcode channel counts/offsets.** Size your matrix from `num_input_channels`, and
  compute the output EQ channel as `output + CH_OUT_1` (`CH_OUT_1 = num_input_channels`). The
  live active input count (2/4/6/8) is separate — read it from the status packet (Section 7.7).

---

## 14. Gotchas & FAQ

- **"I configured routes but only hear 2 channels."** The host is streaming stereo, not a
  multichannel format. Configuration ≠ streaming (Section 2). Select the device's 4/6/8-channel
  48 kHz format in the OS sound settings or open it from your audio engine.
- **"My multichannel format is greyed out / sample rate won't change."** Multichannel input is
  **48 kHz only**. The device rejects 44.1/96 kHz while a multichannel format is selected.
- **"How many input strips should I show?"** Exactly the **live active input count** (2/4/6/8)
  from the status packet (Section 7.7). It changes when the host switches format; a
  `NOTIFY_EVT_INPUT_FORMAT` push fires so you can relayout immediately.
- **"Loudness / crossfeed / leveller has no effect on my multichannel stream."** Correct — those
  are stereo-only and bypassed when >2 inputs are active. **Per-input EQ and per-output
  EQ/crossover still work.**
- **"Out of the box a multichannel stream is mostly silent."** Factory routing is stereo. Apply a
  1:1 default (recipe 11.1) — consider a one-click button.
- **"Which input is which?"** USB channel order is 7.1 (FL FR FC LFE BL BR SL SR) =
  matrix inputs / EQ channels 0-7 (Section 5).
- **"Can I EQ and meter the inputs?"** Yes — that's new. Each active input is EQ channel `0..N-1`
  (`REQ_SET_EQ_PARAM`), and the status packet reports its peak/clip (`peaks[0..N-1]`,
  `clip_flags`). Inputs ≥ active count are inert.
- **"Do I name the inputs?"** Inputs 0..7 now have channel-name storage like outputs (defaults
  "USB L/R", "USB 3".."USB 8"); set them with the channel-name commands or label in your UI.
- **Buffer size:** allocate 8192 bytes for bulk transfers even though V16 is 5864 — matches the
  device buffer and leaves room for growth.
- **macOS/Windows host channel mapping** can reorder channels per the OS's speaker layout. Test
  the actual mapping and expose a per-input label/trim so users can correct it.

---

## 15. Quick reference tables

### Commands for multichannel work
| Code | Name | dir | wValue | Data |
|---|---|---|---|---|
| 0xA0 | GET_ALL_PARAMS | IN | 0 | 5864 B WireBulkParams (V16) |
| 0xA1 | SET_ALL_PARAMS | OUT | 0 | 5864 B WireBulkParams (V16) |
| 0x30 | GET_STATUS | IN | 9 / 23 | combined status / active input count |
| 0x31 | CLEAR_CLIPS | IN | 0 | u32 cleared clip flags |
| 0x70 | SET_MATRIX_ROUTE | OUT | 0 | 9 B (input 0-7) |
| 0x71 | GET_MATRIX_ROUTE | IN | (in<<8)\|out | 9 B |
| 0xD0 | SET_PREAMP_CH | OUT | ch 0-7 | float dB |
| 0xD1 | GET_PREAMP_CH | IN | ch 0-7 | float dB |
| 0x72/73 | SET/GET_OUTPUT_ENABLE | OUT/IN | out 0-8 | u8 |
| 0x74/75 | SET/GET_OUTPUT_GAIN | OUT/IN | out 0-8 | float dB |
| 0x76/77 | SET/GET_OUTPUT_MUTE | OUT/IN | out 0-8 | u8 |
| 0x78/79 | SET/GET_OUTPUT_DELAY | OUT/IN | out 0-8 | float ms |
| 0x42 | SET_EQ_PARAM | OUT | 0 | 16 B (ch = input 0-7 or output+8) |
| 0x43 | GET_EQ_PARAM | IN | encoded | 4 B |
| 0x90/91 | PRESET_SAVE/LOAD | IN | slot 0-9 | result byte |
| 0x7F | GET_PLATFORM | IN | 0 | platform info |

### Data types
| Type | Size | Range |
|---|---|---|
| input index | u8 | 0-7 (RP2350) / 0-1 (RP2040) |
| output index | u8 | 0-8 (0-7 S/PDIF, 8 PDM) |
| EQ / channel index | u8 | 0-16 (RP2350): inputs 0-7, outputs `output + 8` |
| EQ band | u8 | 0-9 |
| gain (dB) | float | -60.0 .. +12.0 (EQ band gain -24..+24) |
| delay (ms) | float | 0 .. 170 (RP2350) |
| crosspoint | 8 B | enabled, phase_invert, reserved[2], gain_db |
| clip_flags | u32 | one bit per channel |

### Key constants
| Name | Value |
|---|---|
| Vendor interface | 2 |
| VID / PID | 0x2E8B / 0xFEAA |
| `bcdDevice` | 0x0204 |
| WinUSB GUID | {9D9B8609-E6D1-4FF0-92AF-403119CB7692} |
| `WIRE_FORMAT_VERSION` | 16 |
| `SLOT_DATA_VERSION` | 21 |
| `WireBulkParams` size (V16) | 5864 bytes |
| Bulk transfer buffer | 8192 bytes |
| USB input alts (RP2350) | 1=2ch/16, 2=2ch/24, 3=4ch/16, 4=6ch/16, 5=8ch/16, 6=4ch/24 |
| 8-channel format | 8 ch / 48 kHz / 16-bit (USB AS alt 5) |
| Matrix (RP2350) | 8 inputs × 9 outputs |

---

*Companion docs:* `usb_8ch_input_spec.md` (firmware/architecture spec), `matrixmixer_spec.md`
(full matrix command reference), `per_channel_preamp_spec.md`, `current_architecture.md`.
