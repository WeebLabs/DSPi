# DSPi Peak, Clip & CPU Metering - Developer Reference

This document provides a complete specification for implementing real-time level meters, clip (OVER) indicators, and CPU load displays in control software for DSPi. All communication occurs via USB vendor control transfers on interface 2.

---

## Overview

DSPi provides three categories of real-time metering data, all updated once per audio block (~1-4 ms depending on sample rate and block size):

- **Peak Levels** — Per-channel instantaneous block peak, `uint16_t` in Q15 format (0-32767 maps to 0.0-1.0 full scale). No decay or hold — the host application is responsible for ballistics.
- **Clip Flags** — Per-channel sticky OVER indicators. A `uint16_t` bitmask where each bit latches when the corresponding channel's block peak exceeds the clip threshold. Bits accumulate until explicitly cleared by the host.
- **CPU Load** — Per-core and combined system load as `uint8_t` percentages (0-100). Budget-based measurement where 100% means processing consumes the full audio time budget (underrun imminent).

All metering data is read via `REQ_GET_STATUS` (0x50). Clip flags are cleared via `REQ_CLEAR_CLIPS` (0x83).

---

## Channel Layout

Peak and clip metering covers all input and output channels. The channel count and index mapping are platform-dependent:

### RP2350 (11 Channels)

| Index | Constant | Description | Peak Source |
|-------|----------|-------------|-------------|
| 0 | `CH_MASTER_LEFT` | Input left (post-preamp, post-master EQ) | Core 0 |
| 1 | `CH_MASTER_RIGHT` | Input right (post-preamp, post-master EQ) | Core 0 |
| 2 | `CH_OUT_1` | S/PDIF 1 left | Core 0 |
| 3 | `CH_OUT_2` | S/PDIF 1 right | Core 0 |
| 4 | `CH_OUT_3` | S/PDIF 2 left | Core 1 (EQ worker) or Core 0 (single-core) |
| 5 | `CH_OUT_4` | S/PDIF 2 right | Core 1 (EQ worker) or Core 0 (single-core) |
| 6 | `CH_OUT_5` | S/PDIF 3 left | Core 1 (EQ worker) or Core 0 (single-core) |
| 7 | `CH_OUT_6` | S/PDIF 3 right | Core 1 (EQ worker) or Core 0 (single-core) |
| 8 | `CH_OUT_7` | S/PDIF 4 left | Core 1 (EQ worker) or Core 0 (single-core) |
| 9 | `CH_OUT_8` | S/PDIF 4 right | Core 1 (EQ worker) or Core 0 (single-core) |
| 10 | `CH_OUT_9_PDM` | PDM subwoofer | Core 0 (single-core or PDM mode) |

### RP2040 (7 Channels)

| Index | Constant | Description | Peak Source |
|-------|----------|-------------|-------------|
| 0 | `CH_MASTER_LEFT` | Input left | Core 0 |
| 1 | `CH_MASTER_RIGHT` | Input right | Core 0 |
| 2 | `CH_OUT_1` | S/PDIF 1 left | Core 0 |
| 3 | `CH_OUT_2` | S/PDIF 1 right | Core 0 |
| 4 | `CH_OUT_3` | S/PDIF 2 left | Core 1 (EQ worker) or Core 0 (single-core) |
| 5 | `CH_OUT_4` | S/PDIF 2 right | Core 1 (EQ worker) or Core 0 (single-core) |
| 6 | `CH_OUT_5_PDM` | PDM subwoofer | Core 0 (single-core or PDM mode) |

### Notes

- **Disabled outputs** report peak 0 and never set clip flags.
- **PDM in EQ_WORKER mode** is mutually exclusive — the PDM channel reports peak 0 when Core 1 is running the EQ worker.
- The channel index used in the peak array matches the EQ channel index used in `REQ_SET_EQ_PARAM` and the bit position in `clip_flags`.

---

## Vendor Commands

All commands use the standard DSPi vendor control transfer format:

- **bmRequestType:** `0xC1` (Device→Host GET) or `0x41` (Host→Device SET)
- **wIndex:** `2` (vendor interface number)
- **Timeout:** Recommended 1000 ms

---

### REQ_GET_STATUS (0x50) — Read Metering Data

**Direction:** Device → Host (GET)

The response format depends on `wValue`. For metering, use **wValue=9** — it returns all data in a single transfer.

---

#### wValue=9: Combined Status (Recommended)

Returns all peaks, CPU loads, and clip flags in one response.

| Field | Value |
|-------|-------|
| bmRequestType | 0xC1 (IN) |
| bRequest | 0x50 |
| wValue | 9 |
| wIndex | 2 |
| wLength | `NUM_CHANNELS * 2 + 4` (base) or `NUM_CHANNELS * 2 + 5` (extended) |

**Response size:** The firmware checks `wLength` and adapts the response:
- Base (`wLength = NUM_CHANNELS * 2 + 4`): RP2350: 26 bytes, RP2040: 18 bytes
- Extended (`wLength >= NUM_CHANNELS * 2 + 5`): RP2350: 27 bytes, RP2040: 19 bytes (includes `cpu_system_load`)

**Backward compatibility:** The firmware only includes `cpu_system_load` when the host requests enough bytes. Old apps requesting `NUM_CHANNELS * 2 + 4` bytes receive exactly that — no trailing byte, no size mismatch. New apps request `NUM_CHANNELS * 2 + 5` to get the extra byte.

**Response layout (RP2350, 27 bytes):**

| Offset | Size | Type | Field | Description |
|--------|------|------|-------|-------------|
| 0 | 2 | uint16_t LE | `peaks[0]` | Input left peak |
| 2 | 2 | uint16_t LE | `peaks[1]` | Input right peak |
| 4 | 2 | uint16_t LE | `peaks[2]` | S/PDIF 1 L peak |
| 6 | 2 | uint16_t LE | `peaks[3]` | S/PDIF 1 R peak |
| 8 | 2 | uint16_t LE | `peaks[4]` | S/PDIF 2 L peak |
| 10 | 2 | uint16_t LE | `peaks[5]` | S/PDIF 2 R peak |
| 12 | 2 | uint16_t LE | `peaks[6]` | S/PDIF 3 L peak |
| 14 | 2 | uint16_t LE | `peaks[7]` | S/PDIF 3 R peak |
| 16 | 2 | uint16_t LE | `peaks[8]` | S/PDIF 4 L peak |
| 18 | 2 | uint16_t LE | `peaks[9]` | S/PDIF 4 R peak |
| 20 | 2 | uint16_t LE | `peaks[10]` | PDM sub peak |
| 22 | 1 | uint8_t | `cpu0_load` | Core 0 CPU load (0-100%) |
| 23 | 1 | uint8_t | `cpu1_load` | Core 1 CPU load (0-100%) |
| 24 | 2 | uint16_t LE | `clip_flags` | Per-channel clip bitmask |
| 26 | 1 | uint8_t | `cpu_system_load` | max(cpu0, cpu1) (0-100%) |

**Response layout (RP2040, 19 bytes):**

| Offset | Size | Type | Field | Description |
|--------|------|------|-------|-------------|
| 0 | 2 | uint16_t LE | `peaks[0]` | Input left peak |
| 2 | 2 | uint16_t LE | `peaks[1]` | Input right peak |
| 4 | 2 | uint16_t LE | `peaks[2]` | S/PDIF 1 L peak |
| 6 | 2 | uint16_t LE | `peaks[3]` | S/PDIF 1 R peak |
| 8 | 2 | uint16_t LE | `peaks[4]` | S/PDIF 2 L peak |
| 10 | 2 | uint16_t LE | `peaks[5]` | S/PDIF 2 R peak |
| 12 | 2 | uint16_t LE | `peaks[6]` | PDM sub peak |
| 14 | 1 | uint8_t | `cpu0_load` | Core 0 CPU load (0-100%) |
| 15 | 1 | uint8_t | `cpu1_load` | Core 1 CPU load (0-100%) |
| 16 | 2 | uint16_t LE | `clip_flags` | Per-channel clip bitmask |
| 18 | 1 | uint8_t | `cpu_system_load` | max(cpu0, cpu1) (0-100%) |

**Generic parsing formula:**

```
peaks[i]        = bytes[i*2] | (bytes[i*2+1] << 8)    for i in 0..NUM_CHANNELS-1
cpu0_load       = bytes[NUM_CHANNELS * 2]
cpu1_load       = bytes[NUM_CHANNELS * 2 + 1]
clip_flags      = bytes[NUM_CHANNELS * 2 + 2] | (bytes[NUM_CHANNELS * 2 + 3] << 8)
cpu_system_load = bytes[NUM_CHANNELS * 2 + 4]   // only present if response length == NUM_CHANNELS*2+5
```

---

#### wValue=0..2: Legacy 4-Byte Status (Individual Queries)

These return a `uint32_t` (4 bytes, little-endian) via a compact control transfer. Useful for lightweight polling of specific channel pairs.

| wValue | Bytes 0-1 | Bytes 2-3 |
|--------|-----------|-----------|
| 0 | `peaks[0]` (Input L) | `peaks[1]` (Input R) |
| 1 | `peaks[2]` (Out 1 L) | `peaks[3]` (Out 1 R) |
| 2 | `peaks[4]` (Out 2 L) | byte 2: `cpu0_load`, byte 3: `cpu1_load` |

---

#### wValue=3..8: Debug Counters

These return a `uint32_t` counter value:

| wValue | Counter | Description |
|--------|---------|-------------|
| 3 | `pdm_ring_overruns` | Core 0 couldn't push PDM sample (ring full) |
| 4 | `pdm_ring_underruns` | Core 1 needed PDM sample but ring empty |
| 5 | `pdm_dma_overruns` | Core 1 PDM write caught up to DMA read |
| 6 | `pdm_dma_underruns` | Core 1 PDM write fell behind DMA read |
| 7 | `spdif_overruns` | USB callback couldn't get S/PDIF buffer (pool full) |
| 8 | `spdif_underruns` | USB packet gap > 2 ms (consumer likely starved) |

---

#### wValue=10..16: System Diagnostics

| wValue | Returns | Description |
|--------|---------|-------------|
| 10 | `uint32_t` | USB audio packet count |
| 11 | `uint32_t` | Last USB alt setting selected |
| 12 | `uint32_t` | USB audio mounted state |
| 13 | `uint32_t` | System clock frequency (Hz) |
| 14 | `uint32_t` | Core voltage (mV) |
| 15 | `uint32_t` | Current sample rate (Hz) |
| 16 | `uint32_t` | Die temperature (centi-degrees C) |

---

### REQ_CLEAR_CLIPS (0x83) — Clear Clip Flags

**Direction:** Device → Host (GET — returns data then clears)

| Field | Value |
|-------|-------|
| bmRequestType | 0xC1 (IN) |
| bRequest | 0x83 |
| wValue | 0 (unused) |
| wIndex | 2 |
| wLength | 2 |

**Response (2 bytes):**

| Offset | Size | Type | Field | Description |
|--------|------|------|-------|-------------|
| 0 | 2 | uint16_t LE | `clip_flags` | Clip flags that were set before clearing |

**Behavior:** Atomic read-then-clear. Returns the current `clip_flags` value, then resets it to 0. This gives the host an acknowledgment of which channels had clipped since the last clear.

---

## Peak Level Format

### Data Representation

- **Type:** `uint16_t`
- **Format:** Q15 unsigned (15 fractional bits)
- **Range:** 0 to 32767
- **0** = silence (no signal or disabled output)
- **32767** = 0 dBFS (full scale)

### Measurement Point

Peaks are measured **post-DSP, post-gain, post-delay** — at the final output stage, just before S/PDIF encoding or PDM modulation. Input peaks (channels 0-1) are measured post-preamp, post-master EQ, post-crossfeed — after all master-stage processing.

### Conversion to dBFS

```c
float linear = (float)peak_u16 / 32767.0f;
float dbfs   = 20.0f * log10f(linear);
// peak_u16=32767 → 0.0 dBFS
// peak_u16=16384 → -6.0 dBFS
// peak_u16=1     → -90.3 dBFS
// peak_u16=0     → -inf (silence)
```

### Ballistics

The firmware reports **raw instantaneous block peaks** with no decay, hold, or smoothing. Each reading represents the absolute maximum sample magnitude in the most recent audio block (up to 192 samples, ~1-4 ms depending on sample rate).

The host application must implement its own meter ballistics. Recommended approaches:

**VU-style decay (simple):**
```c
// Call once per poll interval
if (new_peak > display_peak)
    display_peak = new_peak;                    // instant attack
else
    display_peak *= 0.95f;                       // ~300ms fall time at 50Hz poll
```

**IEC 60268-10 Type I (PPM, broadcast standard):**
```
Integration time:  5 ms (attack)
Return time:       1.7 s (20 dB)
Overshoot:         ≤ 0.5 dB
```

**Peak hold:**
```c
if (new_peak > hold_peak) {
    hold_peak = new_peak;
    hold_timer = HOLD_TIME_MS;
}
if (hold_timer > 0)
    hold_timer -= poll_interval_ms;
else
    hold_peak *= 0.99f;  // slow release after hold expires
```

### Internal Precision

| Platform | Internal Format | Peak Conversion |
|----------|----------------|-----------------|
| RP2350 | IEEE 754 float (0.0-1.0 range) | `(uint16_t)(fminf(1.0f, peak) * 32767.0f)` |
| RP2040 | Q28 fixed-point | `(uint16_t)(peak >> 13)` |

Both produce the same Q15 `uint16_t` output. Host code does not need to differentiate.

---

## Clip Detection (OVER Indicator)

### Threshold

Clip detection uses a threshold slightly above unity to avoid false positives from floating-point precision noise when 0 dBFS signals pass through biquad filters:

| Platform | Threshold | Equivalent |
|----------|-----------|------------|
| RP2350 | `1.001f` | +0.009 dB above unity |
| RP2040 | `(1 << 28) + 268` | ~+0.001 dB above unity in Q28 |

A signal at exactly 0 dBFS that passes through EQ with all-flat settings will **not** trigger a false clip. Only signals that genuinely exceed unity after processing are flagged.

### Bitmask Layout

`clip_flags` is a `uint16_t` where bit *N* corresponds to channel index *N*:

```
Bit  0: CH_MASTER_LEFT    (Input L)
Bit  1: CH_MASTER_RIGHT   (Input R)
Bit  2: CH_OUT_1           (S/PDIF 1 L)
Bit  3: CH_OUT_2           (S/PDIF 1 R)
Bit  4: CH_OUT_3           (S/PDIF 2 L)
Bit  5: CH_OUT_4           (S/PDIF 2 R)
Bit  6: CH_OUT_5 / PDM     (RP2350: S/PDIF 3 L, RP2040: PDM Sub)
Bit  7: CH_OUT_6           (RP2350: S/PDIF 3 R)
Bit  8: CH_OUT_7           (RP2350: S/PDIF 4 L)
Bit  9: CH_OUT_8           (RP2350: S/PDIF 4 R)
Bit 10: CH_OUT_9_PDM       (RP2350: PDM Sub)
Bits 11-15: unused (always 0)
```

**Valid bits mask:**
- RP2350: `0x07FF` (bits 0-10)
- RP2040: `0x007F` (bits 0-6)

### Sticky Behavior

Clip flags follow the **IEC 60268-18 sticky OVER indicator** pattern:

1. A bit is **set** when any sample in a processing block exceeds the clip threshold on that channel.
2. Bits **accumulate** — once set, a bit stays set across subsequent blocks regardless of whether clipping continues.
3. The firmware **never autonomously clears** clip flags.
4. The host must explicitly call `REQ_CLEAR_CLIPS` (0x83) to reset the bitmask.

This design ensures that brief clipping events (even a single sample in a single block) are reliably detected by the host, regardless of the polling interval.

### Recommended Clear Strategy

```c
// Option 1: Clear-on-read — always clear after reading, display for minimum hold time
uint16_t flags = clear_clips(handle);
for (int ch = 0; ch < num_channels; ch++) {
    if (flags & (1 << ch)) {
        clip_hold_timer[ch] = 2000;  // Show OVER for 2 seconds
    }
}

// Option 2: Manual clear — user presses a button to acknowledge
// Only call REQ_CLEAR_CLIPS when the user clicks the clip indicator
```

---

## CPU Load Metering

### Budget-Based Formula

CPU load is measured as `work / budget`, where:

- **work** = wall-clock microseconds spent in DSP processing for a single audio block
- **budget** = `sample_count * 1000000 / sample_freq` — the deterministic audio time available per block

At 48 kHz with a 48-sample block, the budget is 1000 us (1 ms). If DSP processing takes 500 us, the instantaneous load is 50%.

**100% means the processing time equals the audio budget** — the next block cannot start before the current one finishes. Underruns begin at this point.

### EMA Smoothing

Raw instantaneous load values are smoothed with a first-order IIR (exponential moving average):

```
load_q8 = load_q8 - (load_q8 >> 3) + (inst_q8 >> 3)
```

- **Coefficient:** alpha = 1/8 = 0.125
- **Time constant:** ~8 audio blocks (~8 ms at 48 kHz with 48-sample blocks)
- **Internal format:** Q8 fixed-point (0-25600 maps to 0-100%)
- **Output format:** `uint8_t` (0-100), with rounding: `(load_q8 + 128) >> 8`

### Per-Core Metrics

**`cpu0_load` — Core 0:**
- Measures the entire `process_audio_packet` function wall time
- **Excludes** `__wfe()` synchronization wait for Core 1. When Core 1 EQ worker is active, Core 0 may idle waiting for Core 1 to finish — this wait time is measured and subtracted so that enabling Core 1 does not inflate Core 0's reported load.
- Covers: input decoding, preamp, loudness, master EQ, crossfeed, matrix mixing, Core 0 output EQ/gain/delay, S/PDIF pair 1 conversion, PDM push, peak metering

**`cpu1_load` — Core 1:**
- **EQ worker mode:** Measures the EQ/gain/delay/SPDIF conversion block for Core 1's assigned outputs. Same budget formula as Core 0.
- **PDM mode:** Accumulates active microseconds over 48-sample windows (one audio sample at 48 kHz = ~20.8 us per sigma-delta modulation cycle), normalized to a 1 ms budget.
- **Idle mode:** Reports 0.

**`cpu_system_load` — Combined:**
- `max(cpu0_load, cpu1_load)`, computed after each Core 0 metering update.
- Single number to watch for overall headroom. When this approaches 100%, the system is near its processing limit.

### Interpreting CPU Load

| `cpu_system_load` | Meaning |
|-------------------|---------|
| 0-50% | Comfortable headroom |
| 50-75% | Moderate load, consider disabling unused outputs |
| 75-90% | High load, may see occasional underruns at higher sample rates |
| 90-100% | Critical — underruns imminent or occurring |
| 100% | Saturated — processing cannot complete within the audio budget |

### Core 1 Mode Context

The meaning of `cpu1_load` depends on which mode Core 1 is running. Query `REQ_GET_CORE1_MODE` (0x7A) to determine the current mode:

| Mode | Value | `cpu1_load` Meaning |
|------|-------|---------------------|
| IDLE | 0 | Always 0 |
| PDM | 1 | PDM sigma-delta modulation load |
| EQ_WORKER | 2 | Output EQ + gain + delay + S/PDIF conversion load |

PDM and EQ_WORKER are mutually exclusive — enabling any output beyond pair 1 activates EQ_WORKER mode, which disables PDM. The PDM subwoofer channel is available only when all higher outputs are disabled.

---

## Application Patterns

### Polling Combined Status (Recommended)

```c
#include <libusb-1.0/libusb.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#define REQ_GET_STATUS   0x50
#define REQ_CLEAR_CLIPS  0x83
#define VENDOR_INTF      2

// Platform-adaptive channel count — detect via REQ_GET_PLATFORM (0x7F)
// or simply request the maximum and use the actual transfer length.
#define MAX_CHANNELS     11
#define MAX_STATUS_SIZE  (MAX_CHANNELS * 2 + 5)  // 27 bytes

typedef struct {
    uint16_t peaks[MAX_CHANNELS];
    uint8_t  cpu0_load;
    uint8_t  cpu1_load;
    uint8_t  cpu_system_load;
    uint16_t clip_flags;
    int      num_channels;  // Actual channel count (derived from transfer length)
} MeterStatus;

int read_meter_status(libusb_device_handle *handle, MeterStatus *out) {
    uint8_t buf[MAX_STATUS_SIZE];
    int ret = libusb_control_transfer(handle,
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
        REQ_GET_STATUS,
        9,              // wValue = 9 (combined status)
        VENDOR_INTF,    // wIndex
        buf,
        MAX_STATUS_SIZE,
        1000);

    if (ret < 0) return ret;

    // Response is NUM_CHANNELS*2 + 4 bytes (legacy) or +5 bytes (with cpu_system_load).
    // Derive channel count from response length, accounting for both formats.
    bool has_system_load = false;
    int num_channels;
    if ((ret - 5) > 0 && ((ret - 5) % 2) == 0) {
        num_channels = (ret - 5) / 2;
        has_system_load = true;
    } else {
        num_channels = (ret - 4) / 2;
    }
    if (num_channels < 1 || num_channels > MAX_CHANNELS) return -1;

    out->num_channels = num_channels;
    for (int i = 0; i < num_channels; i++) {
        out->peaks[i] = buf[i*2] | (buf[i*2 + 1] << 8);
    }
    int offset = num_channels * 2;
    out->cpu0_load  = buf[offset];
    out->cpu1_load  = buf[offset + 1];
    out->clip_flags = buf[offset + 2] | (buf[offset + 3] << 8);

    // cpu_system_load is appended after clip_flags (backward-compatible extension)
    if (has_system_load)
        out->cpu_system_load = buf[offset + 4];
    else
        out->cpu_system_load = (out->cpu0_load > out->cpu1_load)
                             ? out->cpu0_load : out->cpu1_load;

    return 0;
}
```

### Display Loop with Ballistics

```c
// Meter display state (persistent across polls)
float display_peak[MAX_CHANNELS] = {0};
float hold_peak[MAX_CHANNELS] = {0};
int   hold_timer[MAX_CHANNELS] = {0};
int   clip_hold[MAX_CHANNELS] = {0};

#define POLL_INTERVAL_MS  20   // 50 Hz refresh — good balance of responsiveness and USB traffic
#define HOLD_TIME_MS      2000
#define DECAY_COEFF       0.92f  // Per-poll decay (~300ms fall time at 50Hz)
#define HOLD_DECAY_COEFF  0.995f

void update_meters(libusb_device_handle *handle) {
    MeterStatus status;
    if (read_meter_status(handle, &status) != 0) return;

    for (int ch = 0; ch < status.num_channels; ch++) {
        float raw = (float)status.peaks[ch] / 32767.0f;

        // Bar meter: instant attack, exponential decay
        if (raw > display_peak[ch])
            display_peak[ch] = raw;
        else
            display_peak[ch] *= DECAY_COEFF;

        // Peak hold: instant capture, timed hold, slow release
        if (raw > hold_peak[ch]) {
            hold_peak[ch] = raw;
            hold_timer[ch] = HOLD_TIME_MS;
        }
        if (hold_timer[ch] > 0)
            hold_timer[ch] -= POLL_INTERVAL_MS;
        else
            hold_peak[ch] *= HOLD_DECAY_COEFF;

        // Clip indicator: latch from device, hold for visibility
        if (status.clip_flags & (1 << ch))
            clip_hold[ch] = HOLD_TIME_MS;
        if (clip_hold[ch] > 0)
            clip_hold[ch] -= POLL_INTERVAL_MS;
    }

    // CPU load — no ballistics needed (firmware EMA handles smoothing)
    printf("CPU: Core0=%d%% Core1=%d%% System=%d%%\n",
           status.cpu0_load, status.cpu1_load, status.cpu_system_load);
}
```

### Clearing Clips on User Action

```c
uint16_t clear_clips(libusb_device_handle *handle) {
    uint8_t buf[2] = {0};
    int ret = libusb_control_transfer(handle,
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
        REQ_CLEAR_CLIPS,
        0, VENDOR_INTF,
        buf, 2,
        1000);
    if (ret == 2) {
        return buf[0] | (buf[1] << 8);  // Returns flags that were set before clearing
    }
    return 0;
}

// Example: user clicks the clip indicator
void on_clip_indicator_clicked(libusb_device_handle *handle) {
    uint16_t cleared = clear_clips(handle);
    // Reset all clip hold timers
    memset(clip_hold, 0, sizeof(clip_hold));
    printf("Cleared clips on channels: 0x%04X\n", cleared);
}
```

### Detecting Platform and Channel Count

```c
#define REQ_GET_PLATFORM 0x7F

int get_num_channels(libusb_device_handle *handle) {
    uint8_t platform;
    int ret = libusb_control_transfer(handle,
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
        REQ_GET_PLATFORM,
        0, VENDOR_INTF,
        &platform, 1,
        1000);
    if (ret == 1) {
        return (platform == 1) ? 11 : 7;  // 1=RP2350, 0=RP2040
    }
    return -1;
}
```

Alternatively, parse the response length from `REQ_GET_STATUS` wValue=9 as shown in the `read_meter_status()` example — no separate platform query needed.

### Reading Debug Counters

```c
uint32_t read_status_counter(libusb_device_handle *handle, uint16_t wValue) {
    uint32_t val = 0;
    int ret = libusb_control_transfer(handle,
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
        REQ_GET_STATUS,
        wValue, VENDOR_INTF,
        (uint8_t *)&val, 4,
        1000);
    return (ret == 4) ? val : 0;
}

// Example: read all buffer health counters
void print_buffer_health(libusb_device_handle *handle) {
    printf("SPDIF overruns:      %u\n", read_status_counter(handle, 7));
    printf("SPDIF underruns:     %u\n", read_status_counter(handle, 8));
    printf("PDM ring overruns:   %u\n", read_status_counter(handle, 3));
    printf("PDM ring underruns:  %u\n", read_status_counter(handle, 4));
    printf("PDM DMA overruns:    %u\n", read_status_counter(handle, 5));
    printf("PDM DMA underruns:   %u\n", read_status_counter(handle, 6));
    printf("Sample rate:         %u Hz\n", read_status_counter(handle, 15));
    printf("Temperature:         %.1f C\n", read_status_counter(handle, 16) / 100.0f);
}
```

---

## Polling Interval Recommendations

| Use Case | Interval | Notes |
|----------|----------|-------|
| Meter display (bar graph) | 20-50 ms | 20-50 Hz visual refresh. Slower than 50 ms causes visually sluggish meters. |
| Clip detection only | 100-500 ms | Sticky flags ensure nothing is missed even at slow poll rates. |
| CPU load monitoring | 200-500 ms | EMA smoothing in firmware means faster polling adds no useful information. |
| Background health check | 1000-5000 ms | For logging or diagnostics, not real-time display. |
| Inactive/minimized UI | Pause polling | No metering traffic when the user isn't looking. |

**USB bandwidth:** Each wValue=9 poll is a single USB control transfer (8-byte setup + 19-27 byte data phase). At 50 Hz, this is ~1.75 KB/s — negligible compared to audio isochronous traffic.

---

## Interaction with Other Features

### Volume and Mute

Peak levels reflect the signal **after** master volume and per-output gain/mute are applied. A muted output reports peak 0. Reducing the master volume reduces all output peaks proportionally. Input peaks (channels 0-1) are measured after the preamp but before the master volume.

### EQ and Crossfeed

Input peaks are measured after master EQ and crossfeed processing. Output peaks are measured after per-output EQ. Boosting EQ bands can cause peaks to exceed unity, triggering clip flags even when the input signal is below 0 dBFS.

### Matrix Mixer

Peaks are per-output, after matrix summing. If both L and R inputs are routed to the same output at 0 dB, the output peak can be up to +6 dB higher than either input peak (correlated signals). The clip threshold accounts for this — signals at or below 0 dBFS on each input will not clip on outputs with default 0 dB routing.

### Sample Rate Changes

CPU load values are automatically scaled to the new sample rate's budget. The same DSP workload will report a higher percentage at 96 kHz than at 48 kHz (the budget halves). Peak format and clip thresholds are sample-rate independent.

### S/PDIF Input Mode (RP2350 Only)

Metering works identically regardless of whether the input source is USB or S/PDIF. The same `REQ_GET_STATUS` command returns the same format. During an input source switch (via `REQ_SET_AUDIO_SOURCE`), metering briefly reflects the mute period (peaks drop to 0, then resume with the new source).

### Dual-Core Processing

When Core 1 is running as the EQ worker, peaks for its assigned outputs are computed by Core 1 and written to `global_status` before the Core 1 → Core 0 handshake completes. The `__dmb()` memory barrier and `work_done` flag guarantee that the host always reads a consistent snapshot — no torn reads or stale data across cores.

---

## Platform Differences

| Feature | RP2040 | RP2350 |
|---------|--------|--------|
| Channel count | 7 | 11 |
| wValue=9 response size | 18 or 19 bytes | 26 or 27 bytes |
| Valid `clip_flags` bits | 0x007F (bits 0-6) | 0x07FF (bits 0-10) |
| Internal peak format | Q28 → Q15 via `>> 13` | float → Q15 via `* 32767` |
| Clip threshold | `(1<<28) + 268` (~+0.001 dB) | `1.001f` (~+0.009 dB) |
| Core 1 EQ outputs | 2-3 (1 S/PDIF pair) | 2-7 (3 S/PDIF pairs) |
| CPU load formula | Budget-based, same on both | Budget-based, same on both |

Both platforms expose identical vendor command interfaces and response formats (differing only in size). Control software that parses the response length dynamically (as shown in the examples) works on both platforms without modification.

---

## Backward Compatibility

- The `cpu_system_load` byte was added in firmware v1.1.2, **appended after `clip_flags`** to preserve the existing byte layout. The firmware checks `wLength` and only includes it when the host requests `NUM_CHANNELS * 2 + 5` bytes. To detect firmware support:
  - Request `NUM_CHANNELS * 2 + 5` bytes — if the firmware supports `cpu_system_load`, the last byte contains it
  - Request `NUM_CHANNELS * 2 + 4` bytes — the firmware omits the extra byte, returning exactly the base layout
  - Older firmware (pre-v1.1.2) returns `NUM_CHANNELS * 2 + 4` bytes regardless of `wLength`; compute system load client-side as `max(cpu0, cpu1)`
- The first `NUM_CHANNELS * 2 + 4` bytes of the response are **layout-identical** across all firmware versions. Existing apps that request this size are unaffected by the new firmware.
- `REQ_CLEAR_CLIPS` (0x83) was added in firmware v1.1.1. Older firmware will STALL. Handle gracefully.
- The legacy wValue=0..2 responses are unchanged across all firmware versions.

---

## Diagnostic Checklist

| Symptom | Likely Cause | Check |
|---------|-------------|-------|
| All peaks stuck at 0 | No audio playing, or all outputs disabled | Verify output enables (`REQ_GET_OUTPUT_ENABLE`), confirm audio stream is active |
| Peaks present but clip never triggers | Signal below threshold | Clip only triggers above +0.009 dB. A 0 dBFS signal through flat EQ will not clip. |
| Clip triggers on output but not input | EQ boost causing output to exceed unity | Check per-output EQ settings, add negative preamp to compensate |
| `cpu0_load` jumps when Core 1 outputs enabled | (Should not happen with v1.1.2+) | Verify firmware version. Older firmware had a bug where Core 0 wait time inflated its load. |
| `cpu_system_load` near 100% | DSP pipeline near capacity | Disable unused outputs, reduce EQ bands, or lower sample rate |
| `cpu1_load` is 0 but outputs 2+ are enabled | Core 1 is not in EQ_WORKER mode | Check `REQ_GET_CORE1_MODE`. If IDLE, outputs 2+ are processed single-core on Core 0. |
| Meters update slowly | Poll interval too long | Reduce polling interval to 20-50 ms for responsive metering |
| Peaks don't reach 32767 at full scale | Normal — Q15 clips at 32767 | 32767 represents 1.0 (0 dBFS). Signals above unity are clamped. |
| Clip flags set but peaks show < 32767 | Peak is clamped, clip threshold is above the peak range | The clip threshold (~1.001) is above the peak display range (1.0 → 32767). A brief clip can set the flag even if the clamped peak reads 32767. |

---

## Request Code Summary

| Code | Command | Direction | wValue | Data Size | Description |
|------|---------|-----------|--------|-----------|-------------|
| 0x50 | `REQ_GET_STATUS` | IN | 9 | 18-19 or 26-27 bytes | All peaks + CPU load + clip flags [+ system load] |
| 0x50 | `REQ_GET_STATUS` | IN | 0 | 4 bytes | Input L/R peaks |
| 0x50 | `REQ_GET_STATUS` | IN | 1 | 4 bytes | Output 1 L/R peaks |
| 0x50 | `REQ_GET_STATUS` | IN | 2 | 4 bytes | Output 2 L peak + CPU loads |
| 0x50 | `REQ_GET_STATUS` | IN | 3-8 | 4 bytes | Debug counters |
| 0x50 | `REQ_GET_STATUS` | IN | 10-16 | 4 bytes | System diagnostics |
| 0x83 | `REQ_CLEAR_CLIPS` | IN | 0 | 2 bytes | Read-then-clear clip flags |
