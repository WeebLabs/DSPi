# DSPi Peak, Clip & CPU Metering - Developer Reference

*Last updated: 2026-11-16 (vendor commands, reply layout and channel layout for the paged protocol of firmware 1.1.7)*

This document provides a complete specification for implementing real-time level meters, clip (OVER) indicators, and CPU load displays in control software for DSPi. All communication occurs via USB vendor control transfers on interface 2.

---

## Overview

DSPi provides three categories of real-time metering data, all updated once per audio block (~1-4 ms depending on sample rate and block size):

- **Peak Levels** — Per-channel instantaneous block peak, `uint16_t` in Q15 format (0-32767 maps to 0.0-1.0 full scale). No decay or hold — the host application is responsible for ballistics.
- **Clip Flags:** Per-channel sticky OVER indicators. A `uint32_t` bitmask where each bit latches when the corresponding channel's block peak exceeds the clip threshold. Bits accumulate until explicitly cleared by the host.
- **CPU Load** — Per-core and combined system load as `uint8_t` percentages (0-100). Budget-based measurement where 100% means processing consumes the full audio time budget (underrun imminent).

All metering data is read with one `MEAS.LEVELS` request. Clip flags are cleared with `MEAS.CLIP_CLEAR`. Both are paged commands (`../commands.md`).

---

## Channel Layout

Peak and clip metering covers all input and output channels. The channel count and index mapping are platform-dependent:

### RP2350 (17 Channels)

Inputs are first-class channels (unified channel model). Indices 0-7 are the inputs. The active input count is 2, 4, 6 or 8, and an inactive input reports peak 0.

| Index | Constant | Description | Peak Source |
|-------|----------|-------------|-------------|
| 0-7 | (inputs) | Input channels 1-8. 0 and 1 are the stereo pair | Core 0 |
| 8 | `CH_OUT_1` | S/PDIF 1 left | Core 0 |
| 9 | `CH_OUT_2` | S/PDIF 1 right | Core 0 |
| 10 | `CH_OUT_3` | S/PDIF 2 left | Core 1 (EQ worker) or Core 0 (single-core) |
| 11 | `CH_OUT_4` | S/PDIF 2 right | Core 1 (EQ worker) or Core 0 (single-core) |
| 12 | `CH_OUT_5` | S/PDIF 3 left | Core 1 (EQ worker) or Core 0 (single-core) |
| 13 | `CH_OUT_6` | S/PDIF 3 right | Core 1 (EQ worker) or Core 0 (single-core) |
| 14 | `CH_OUT_7` | S/PDIF 4 left | Core 1 (EQ worker) or Core 0 (single-core) |
| 15 | `CH_OUT_8` | S/PDIF 4 right | Core 1 (EQ worker) or Core 0 (single-core) |
| 16 | `CH_OUT_9_PDM` | PDM subwoofer | Core 0 (single-core or PDM mode) |

### RP2040 (7 Channels)

| Index | Constant | Description | Peak Source |
|-------|----------|-------------|-------------|
| 0 | (input) | Input left | Core 0 |
| 1 | (input) | Input right | Core 0 |
| 2 | `CH_OUT_1` | S/PDIF 1 left | Core 0 |
| 3 | `CH_OUT_2` | S/PDIF 1 right | Core 0 |
| 4 | `CH_OUT_3` | S/PDIF 2 left | Core 1 (EQ worker) or Core 0 (single-core) |
| 5 | `CH_OUT_4` | S/PDIF 2 right | Core 1 (EQ worker) or Core 0 (single-core) |
| 6 | `CH_OUT_5_PDM` | PDM subwoofer | Core 0 (single-core or PDM mode) |

### Notes

- **Disabled outputs** report peak 0 and never set clip flags.
- **PDM in EQ_WORKER mode** is mutually exclusive — the PDM channel reports peak 0 when Core 1 is running the EQ worker.
- The channel index used in the peak array matches the EQ channel index in the `FILTER.BAND` address and the bit position in `clip_flags`.

---

## Vendor Commands

All three are paged commands. Page 0x09 (`MEAS`) travels in the `wIndex` high byte and the interface number 2 in the low byte, so `wIndex` is `0x0902`. A refused request STALLs, and `SYS.LAST_RESULT` gives the reason (`../commands.md`).

| Command | Page | Cmd | Dir | Address (`wValue`) | Size | Notes |
|---|---|---|---|---|---|---|
| `MEAS.LEVELS` | 0x09 | 0x10 | GET | none (0) | GET var | `2 * NUM_CHANNELS + 7` bytes: 41 on RP2350, 21 on RP2040 |
| `MEAS.CLIP_FLAGS` | 0x09 | 0x11 | GET | none (0) | GET 4 | u32 `clip_flags`. Reading does not clear |
| `MEAS.CLIP_CLEAR` | 0x09 | 0x12 | SET (zero-length) | none (0) | SET 0 | Clears every clip flag |

- **Timeout:** Recommended 1000 ms

---

### `MEAS.LEVELS`: Read Metering Data

Returns all peaks, CPU loads, clip flags and the active input count in one response. These are the same bytes as the 1.1.6 `GET_STATUS` selector 9.

| Field | Value |
|-------|-------|
| bmRequestType | 0xC1 (IN) |
| bRequest | 0x10 |
| wValue | 0 |
| wIndex | 0x0902 |
| wLength | `NUM_CHANNELS * 2 + 7` (RP2350: 41, RP2040: 21) |

**Response layout (RP2350, 41 bytes):**

| Offset | Size | Type | Field | Description |
|--------|------|------|-------|-------------|
| 0 | 34 | uint16_t LE [17] | `peaks[0..16]` | Peaks in channel-index order (table above) |
| 34 | 1 | uint8_t | `cpu0_load` | Core 0 CPU load (0-100%) |
| 35 | 1 | uint8_t | `cpu1_load` | Core 1 CPU load (0-100%) |
| 36 | 4 | uint32_t LE | `clip_flags` | Per-channel clip bitmask |
| 40 | 1 | uint8_t | `active_inputs` | Live active input channel count |

**Response layout (RP2040, 21 bytes):**

| Offset | Size | Type | Field | Description |
|--------|------|------|-------|-------------|
| 0 | 14 | uint16_t LE [7] | `peaks[0..6]` | Peaks in channel-index order (table above) |
| 14 | 1 | uint8_t | `cpu0_load` | Core 0 CPU load (0-100%) |
| 15 | 1 | uint8_t | `cpu1_load` | Core 1 CPU load (0-100%) |
| 16 | 4 | uint32_t LE | `clip_flags` | Per-channel clip bitmask |
| 20 | 1 | uint8_t | `active_inputs` | Live active input channel count |

**Generic parsing formula:**

```
peaks[i]        = bytes[i*2] | (bytes[i*2+1] << 8)    for i in 0..NUM_CHANNELS-1
cpu0_load       = bytes[NUM_CHANNELS * 2]
cpu1_load       = bytes[NUM_CHANNELS * 2 + 1]
clip_flags      = u32 LE at bytes[NUM_CHANNELS * 2 + 2]
active_inputs   = bytes[NUM_CHANNELS * 2 + 6]
```

---

### Retired: 1.1.6 `GET_STATUS` selectors

The 1.1.6 scalar selectors are gone. Their data now comes from struct reads.

| 1.1.6 selector | Now |
|---|---|
| 0, 1, 2 (packed peak pairs and CPU) | Retired. Use `MEAS.LEVELS` |
| 3-6 (PDM ring and DMA counters) | `SYS.PDM_STATS` offsets 0, 4, 8, 12 |
| 7 | Retired, always read 0 |
| 8 (`spdif_underruns`) | `SYS.SPDIF_TX_STATS` offset 0 |
| 10 (USB audio packets) | `SYS.USB_AUDIO_STATUS` offset 0 (u32) |
| 11, 12 (alt setting, mounted) | `SYS.USB_AUDIO_STATUS` offsets 12 and 13 (u8 each) |
| 13 (clk_sys Hz) | `SYS.SYSTEM_STATUS` offset 0 (u32) |
| 14 (core voltage mV) | `SYS.SYSTEM_STATUS` offset 8 (u16) |
| 15 (sample rate Hz) | `SYS.SYSTEM_STATUS` offset 4 (u32) |
| 16 (temperature, centi-degrees C) | `SYS.SYSTEM_STATUS` offset 10 (i16) |

`SYS.SYSTEM_STATUS` (page 0x01, command 0x40, 16 bytes) also carries `cpu0_load` and `cpu1_load` at offsets 12 and 13. `SYS.PDM_STATS` is page 0x01, command 0x44, 16 bytes. `SYS.USB_AUDIO_STATUS` is page 0x01, command 0x41, 16 bytes.

---

### `MEAS.CLIP_FLAGS` and `MEAS.CLIP_CLEAR`: Read and Clear Clip Flags

`MEAS.CLIP_FLAGS` returns the current 4-byte `clip_flags` and leaves them set. `MEAS.CLIP_CLEAR` is a zero-length SET (`bmRequestType` 0x41, `bRequest` 0x12, `wIndex` 0x0902, `wLength` 0) that resets them to 0. It has no reply data.

**Behavior:** Read and clear are two requests. 1.1.6 opcode 0x83 did both atomically. A clip that lands between the read and the clear is lost, so a host that needs every event should read `clip_flags` from `MEAS.LEVELS` on every poll and clear only on user action.

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

`clip_flags` is a `uint32_t` where bit *N* corresponds to channel index *N* in the channel layout above. On RP2350 bits 0-7 are the inputs, bits 8-15 the S/PDIF outputs and bit 16 the PDM sub. On RP2040 bits 0-1 are the inputs, bits 2-5 the S/PDIF outputs and bit 6 the PDM sub. Higher bits are always 0.

**Valid bits mask:**
- RP2350: `0x0001FFFF` (bits 0-16)
- RP2040: `0x0000007F` (bits 0-6)

### Sticky Behavior

Clip flags follow the **IEC 60268-18 sticky OVER indicator** pattern:

1. A bit is **set** when any sample in a processing block exceeds the clip threshold on that channel.
2. Bits **accumulate** — once set, a bit stays set across subsequent blocks regardless of whether clipping continues.
3. The firmware **never autonomously clears** clip flags.
4. The host must explicitly send `MEAS.CLIP_CLEAR` to reset the bitmask.

This design ensures that brief clipping events (even a single sample in a single block) are reliably detected by the host, regardless of the polling interval.

### Recommended Clear Strategy

```c
// Option 1: Clear after each read, display for minimum hold time.
// A clip between the read and the clear is lost (two requests).
uint32_t flags = read_and_clear_clips(handle);
for (int ch = 0; ch < num_channels; ch++) {
    if (flags & (1 << ch)) {
        clip_hold_timer[ch] = 2000;  // Show OVER for 2 seconds
    }
}

// Option 2: Manual clear — user presses a button to acknowledge
// Only send MEAS.CLIP_CLEAR when the user clicks the clip indicator
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

**System load (computed by the host):**
- `max(cpu0_load, cpu1_load)`. The firmware does not report a combined byte, so the host computes it from the two per-core values.
- Single number to watch for overall headroom. When this approaches 100%, the system is near its processing limit.

### Interpreting CPU Load

| System load | Meaning |
|-------------|---------|
| 0-50% | Comfortable headroom |
| 50-75% | Moderate load, consider disabling unused outputs |
| 75-90% | High load, may see occasional underruns at higher sample rates |
| 90-100% | Critical — underruns imminent or occurring |
| 100% | Saturated — processing cannot complete within the audio budget |

### Core 1 Mode Context

The meaning of `cpu1_load` depends on which mode Core 1 is running. Query `SYS.CORE1_MODE` (page 0x01, command 0x23, 1 byte) to determine the current mode.

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

// Paged protocol: bRequest = command, wIndex = (page << 8) | interface.
#define PAGE_MEAS        0x09
#define MEAS_LEVELS      0x10
#define MEAS_CLIP_FLAGS  0x11
#define MEAS_CLIP_CLEAR  0x12
#define VENDOR_INTF      2
#define WINDEX(page)     (((page) << 8) | VENDOR_INTF)

// Request the largest reply and derive the channel count from the length.
#define MAX_CHANNELS     17
#define MAX_STATUS_SIZE  (MAX_CHANNELS * 2 + 7)  // 41 bytes

typedef struct {
    uint16_t peaks[MAX_CHANNELS];
    uint8_t  cpu0_load;
    uint8_t  cpu1_load;
    uint8_t  cpu_system_load;   // computed here: max(cpu0, cpu1)
    uint32_t clip_flags;
    uint8_t  active_inputs;
    int      num_channels;      // derived from the transfer length
} MeterStatus;

int read_meter_status(libusb_device_handle *handle, MeterStatus *out) {
    uint8_t buf[MAX_STATUS_SIZE];
    int ret = libusb_control_transfer(handle,
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
        MEAS_LEVELS,
        0,                      // wValue: no address
        WINDEX(PAGE_MEAS),      // wIndex 0x0902
        buf,
        MAX_STATUS_SIZE,
        1000);

    if (ret < 0) return ret;

    // Response is NUM_CHANNELS*2 + 7 bytes.
    if (ret < 9 || ((ret - 7) % 2) != 0) return -1;
    int num_channels = (ret - 7) / 2;
    if (num_channels > MAX_CHANNELS) return -1;

    out->num_channels = num_channels;
    for (int i = 0; i < num_channels; i++) {
        out->peaks[i] = buf[i*2] | (buf[i*2 + 1] << 8);
    }
    int offset = num_channels * 2;
    out->cpu0_load  = buf[offset];
    out->cpu1_load  = buf[offset + 1];
    out->clip_flags = (uint32_t)buf[offset + 2] | ((uint32_t)buf[offset + 3] << 8) |
                      ((uint32_t)buf[offset + 4] << 16) | ((uint32_t)buf[offset + 5] << 24);
    out->active_inputs = buf[offset + 6];
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
// Zero-length SET. A refused request STALLs (ret < 0).
int clear_clips(libusb_device_handle *handle) {
    return libusb_control_transfer(handle,
        LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
        MEAS_CLIP_CLEAR,
        0, WINDEX(PAGE_MEAS),
        NULL, 0,
        1000);
}

// Two requests: a clip that lands between them is lost.
uint32_t read_and_clear_clips(libusb_device_handle *handle) {
    uint8_t buf[4] = {0};
    int ret = libusb_control_transfer(handle,
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
        MEAS_CLIP_FLAGS,
        0, WINDEX(PAGE_MEAS),
        buf, 4,
        1000);
    clear_clips(handle);
    if (ret == 4)
        return (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) |
               ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
    return 0;
}

// Example: user clicks the clip indicator
void on_clip_indicator_clicked(libusb_device_handle *handle) {
    uint32_t cleared = read_and_clear_clips(handle);
    // Reset all clip hold timers
    memset(clip_hold, 0, sizeof(clip_hold));
    printf("Cleared clips on channels: 0x%08X\n", cleared);
}
```

### Detecting Platform and Channel Count

```c
#define PAGE_SYS      0x01
#define SYS_PLATFORM  0x20

int get_num_channels(libusb_device_handle *handle) {
    uint8_t platform[7];
    int ret = libusb_control_transfer(handle,
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
        SYS_PLATFORM,
        0, WINDEX(PAGE_SYS),
        platform, sizeof(platform),
        1000);
    if (ret >= 1) {
        return (platform[0] == 1) ? 17 : 7;  // 1=RP2350, 0=RP2040
    }
    return -1;
}
```

Alternatively, parse the response length of `MEAS.LEVELS` as shown in the `read_meter_status()` example. No separate platform query is needed.

### Reading Debug Counters

```c
static uint32_t u32_at(const uint8_t *b, int off) {
    return (uint32_t)b[off] | ((uint32_t)b[off + 1] << 8) |
           ((uint32_t)b[off + 2] << 16) | ((uint32_t)b[off + 3] << 24);
}

static int sys_get(libusb_device_handle *handle, uint8_t cmd, uint8_t *buf, int len) {
    return libusb_control_transfer(handle,
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
        cmd, 0, WINDEX(PAGE_SYS), buf, len, 1000);
}

// Example: read all buffer health counters (three struct reads)
void print_buffer_health(libusb_device_handle *handle) {
    uint8_t tx[24], pdm[16], sys[16];
    if (sys_get(handle, 0x45, tx, 24) != 24) return;    // SYS.SPDIF_TX_STATS
    if (sys_get(handle, 0x44, pdm, 16) != 16) return;   // SYS.PDM_STATS
    if (sys_get(handle, 0x40, sys, 16) != 16) return;   // SYS.SYSTEM_STATUS
    printf("USB packet gaps:     %u\n", u32_at(tx, 0));   // gaps of 2 to 50 ms
    printf("PDM ring overruns:   %u\n", u32_at(pdm, 0));
    printf("PDM ring underruns:  %u\n", u32_at(pdm, 4));
    printf("PDM DMA overruns:    %u\n", u32_at(pdm, 8));
    printf("PDM DMA underruns:   %u\n", u32_at(pdm, 12));
    printf("Sample rate:         %u Hz\n", u32_at(sys, 4));
    printf("Temperature:         %.1f C\n", (int16_t)(sys[10] | (sys[11] << 8)) / 100.0f);
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

**USB bandwidth:** Each `MEAS.LEVELS` poll is a single USB control transfer (8-byte setup + 21-41 byte data phase). At 50 Hz, this is ~1.75 KB/s. That is negligible compared to audio isochronous traffic.

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

Metering works identically regardless of whether the input source is USB or S/PDIF. The same `MEAS.LEVELS` command returns the same format. During an input source switch (via `INPUT.SOURCE`), metering briefly reflects the mute period (peaks drop to 0, then resume with the new source).

### Dual-Core Processing

When Core 1 is running as the EQ worker, peaks for its assigned outputs are computed by Core 1 and written to `global_status` before the Core 1 → Core 0 handshake completes. The `__dmb()` memory barrier and `work_done` flag guarantee that the host always reads a consistent snapshot — no torn reads or stale data across cores.

---

## Platform Differences

| Feature | RP2040 | RP2350 |
|---------|--------|--------|
| Channel count | 7 | 17 |
| `MEAS.LEVELS` response size | 21 bytes | 41 bytes |
| Valid `clip_flags` bits | 0x0000007F (bits 0-6) | 0x0001FFFF (bits 0-16) |
| Internal peak format | Q28 → Q15 via `>> 13` | float → Q15 via `* 32767` |
| Clip threshold | `(1<<28) + 268` (~+0.001 dB) | `1.001f` (~+0.009 dB) |
| Core 1 EQ outputs | 2-3 (1 S/PDIF pair) | 2-7 (3 S/PDIF pairs) |
| CPU load formula | Budget-based, same on both | Budget-based, same on both |

Both platforms expose identical vendor command interfaces and response formats (differing only in size). Control software that parses the response length dynamically (as shown in the examples) works on both platforms without modification.

---

## Backward Compatibility

- Every paged firmware (1.1.7 and later) has `MEAS.LEVELS`, `MEAS.CLIP_FLAGS` and `MEAS.CLIP_CLEAR`. Firmware 1.1.6 and older does not answer the paged protocol (`vendor_command_overhaul_plan.md` section 5.14).
- `MEAS.LEVELS` returns the same bytes as the 1.1.6 `GET_STATUS` selector 9, so a parser for that packet works unchanged.
- There is no `cpu_system_load` byte. Compute `max(cpu0_load, cpu1_load)` on the host.
- A future page revision may append fields to `MEAS.LEVELS`. Hosts should parse the fields they know and ignore the rest.

---

## Diagnostic Checklist

| Symptom | Likely Cause | Check |
|---------|-------------|-------|
| All peaks stuck at 0 | No audio playing, or all outputs disabled | Verify output enables (`ROUTE.OUTPUT_ENABLE`), confirm audio stream is active |
| Peaks present but clip never triggers | Signal below threshold | Clip only triggers above +0.009 dB. A 0 dBFS signal through flat EQ will not clip. |
| Clip triggers on output but not input | EQ boost causing output to exceed unity | Check per-output EQ settings, add negative preamp to compensate |
| `cpu0_load` jumps when Core 1 outputs enabled | (Should not happen with v1.1.2+) | Verify firmware version. Older firmware had a bug where Core 0 wait time inflated its load. |
| System load (max of the two cores) near 100% | DSP pipeline near capacity | Disable unused outputs, reduce EQ bands, or lower sample rate |
| `cpu1_load` is 0 but outputs 2+ are enabled | Core 1 is not in EQ_WORKER mode | Check `SYS.CORE1_MODE`. If IDLE, outputs 2+ are processed single-core on Core 0. |
| Meters update slowly | Poll interval too long | Reduce polling interval to 20-50 ms for responsive metering |
| Peaks don't reach 32767 at full scale | Normal — Q15 clips at 32767 | 32767 represents 1.0 (0 dBFS). Signals above unity are clamped. |
| Clip flags set but peaks show < 32767 | Peak is clamped, clip threshold is above the peak range | The clip threshold (~1.001) is above the peak display range (1.0 → 32767). A brief clip can set the flag even if the clamped peak reads 32767. |

---

## Request Code Summary

| Command | Page | Cmd | Dir | Address (`wValue`) | Size | Notes |
|---|---|---|---|---|---|---|
| `MEAS.LEVELS` | 0x09 | 0x10 | GET | none (0) | GET var | All peaks, CPU loads, clip flags, active input count |
| `MEAS.CLIP_FLAGS` | 0x09 | 0x11 | GET | none (0) | GET 4 | Clip flags, not cleared |
| `MEAS.CLIP_CLEAR` | 0x09 | 0x12 | SET (zero-length) | none (0) | SET 0 | Clear clip flags |
| `SYS.SYSTEM_STATUS` | 0x01 | 0x40 | GET | none (0) | GET 16 | clk_sys, sample rate, core mV, temperature, CPU loads |
| `SYS.USB_AUDIO_STATUS` | 0x01 | 0x41 | GET | none (0) | GET 16 | USB packets, ring overruns, worst gap, alt, mounted |
| `SYS.PDM_STATS` | 0x01 | 0x44 | GET | none (0) | GET 16 | PDM ring and DMA counters |
| `SYS.SPDIF_TX_STATS` | 0x01 | 0x45 | GET | none (0) | GET 24 | USB audio packet gaps of 2 to 50 ms and output DMA starvations |
| `SYS.CORE1_MODE` | 0x01 | 0x23 | GET | none (0) | GET 1 | Core 1 mode |
