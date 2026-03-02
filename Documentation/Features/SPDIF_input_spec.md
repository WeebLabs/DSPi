# S/PDIF Input Specification

## Overview

DSPi supports switchable audio input: USB (default) or S/PDIF. When S/PDIF is the active source, received audio feeds through the identical DSP pipeline (preamp, loudness, master EQ, crossfeed, matrix mixer, per-output EQ, delay) as USB audio. The input source is controlled at runtime via three vendor USB control transfer commands:

- **`REQ_SET_AUDIO_SOURCE` (0x80)** — Switch between USB and S/PDIF input
- **`REQ_GET_AUDIO_SOURCE` (0x81)** — Query the current input source
- **`REQ_GET_SPDIF_IN_STATUS` (0x82)** — Query S/PDIF receiver state, sample rate, errors, and channel status

The S/PDIF receiver supports consumer-format IEC 60958-3 input at 44.1, 48, 88.2, 96, 176.4, and 192 kHz, with 24-bit audio precision.

---

## Vendor Commands

All commands use the standard DSPi vendor control transfer format:

- **bmRequestType:** `0x41` (Host→Device SET) or `0xC1` (Device→Host GET)
- **wIndex:** `2` (vendor interface number)
- **Timeout:** Recommended 1000 ms for GET commands, 2000 ms for SET_AUDIO_SOURCE (source switching can take up to 500 ms)

---

### REQ_SET_AUDIO_SOURCE (0x80)

**Direction:** Host → Device (SET)
**wValue:** 0 (unused)
**wIndex:** Vendor interface number (2)
**wLength:** 1

#### Request Data (1 byte)

| Offset | Size | Field | Values |
|--------|------|-------|--------|
| 0 | 1 | `source` | `0` = USB, `1` = S/PDIF |

#### Behavior

The firmware executes a multi-phase switch sequence:

1. **Mute** — All outputs are muted immediately. A 5 ms delay allows one block of silence to propagate through the pipeline.
2. **Start/Stop receiver** — S/PDIF RX hardware is started or stopped.
3. **Lock acquisition** (USB→SPDIF only) — The firmware waits up to 500 ms for the receiver to lock to the incoming signal. If lock is not achieved, the switch is aborted and the device remains on the previous source with mute restored to its prior state.
4. **Clock reconfiguration** — TX output clocks are adjusted to match the detected input sample rate.
5. **Unmute** — Outputs are unmuted (restoring the previous mute state).

#### Failure Cases

| Condition | Result |
|-----------|--------|
| No S/PDIF signal connected | Switch aborted, remains on USB, no error response (command still ACKs) |
| Signal present but unstable | Switch aborted after 500 ms timeout |
| Already on requested source | No-op, immediate return |
| Invalid source value (>1) | Ignored, no action |

#### Important Notes

- This command is **blocking** — the USB control transfer completes only after the switch sequence finishes (up to ~510 ms for USB→SPDIF). Set the USB transfer timeout accordingly.
- During the switch, audio output is briefly muted (~5-510 ms depending on direction and lock time).
- When switching to S/PDIF, USB audio data continues to arrive but is ignored. The host audio stack may report underruns.
- When switching back to USB, the device resumes consuming USB audio data immediately.
- The DSP pipeline state (EQ filter histories, crossfeed state, delay line contents) is **not** reset on source switch. This provides seamless transitions but means a brief "tail" of the previous source's audio may be heard through delay lines.

---

### REQ_GET_AUDIO_SOURCE (0x81)

**Direction:** Device → Host (GET)
**wValue:** 0 (unused)
**wIndex:** Vendor interface number (2)
**wLength:** 1

#### Response (1 byte)

| Offset | Size | Field | Values |
|--------|------|-------|--------|
| 0 | 1 | `source` | `0` = USB, `1` = S/PDIF |

#### Notes

- Returns the currently active input source.
- During a switch operation (REQ_SET_AUDIO_SOURCE in progress), returns the source that was active before the switch began.
- At boot, the source is always `0` (USB). The S/PDIF input setting is **not** persisted to flash.

---

### REQ_GET_SPDIF_IN_STATUS (0x82)

**Direction:** Device → Host (GET)
**wValue:** 0 (unused)
**wIndex:** Vendor interface number (2)
**wLength:** 20

#### Response (20 bytes)

| Offset | Size | Type | Field | Description |
|--------|------|------|-------|-------------|
| 0 | 4 | uint32_t | `state` | Receiver state (see below) |
| 4 | 4 | uint32_t | `sample_rate` | Detected sample rate in Hz, or 0 |
| 8 | 4 | uint32_t | `parity_err_count` | Parity errors since last lock |
| 12 | 5 | uint8_t[5] | `c_bits` | IEC 60958-3 channel status bytes 0-4 |
| 17 | 3 | — | (padding) | Struct alignment padding (ignore) |

All multi-byte fields are **little-endian** (native ARM Cortex-M byte order).

#### State Values

| Value | Name | Description |
|-------|------|-------------|
| 0 | `NO_SIGNAL` | No S/PDIF signal detected. Receiver is idle or scanning. |
| 1 | `ACQUIRING` | Signal detected, decoding in progress. Not yet stable enough for audio. |
| 2 | `LOCKED` | Receiver is locked and delivering audio. Sample rate is confirmed. |

#### Sample Rate Values

| `sample_rate` | Meaning |
|---------------|---------|
| 0 | Unknown / not yet detected |
| 44100 | 44.1 kHz (CD audio) |
| 48000 | 48 kHz (standard) |
| 88200 | 88.2 kHz (2x 44.1) |
| 96000 | 96 kHz (2x 48) |
| 176400 | 176.4 kHz (4x 44.1) |
| 192000 | 192 kHz (4x 48) |

The sample rate is derived from IEC 60958-3 channel status byte 3 when available, with a fallback to symbol-rate estimation. The rate is only valid when `state == LOCKED`.

#### Channel Status Bytes (c_bits)

The 5-byte `c_bits` array contains the first 5 bytes of the IEC 60958-3 channel status block, captured from the received S/PDIF stream. These are only meaningful when `state == LOCKED`.

| Byte | Typical Value | IEC 60958-3 Meaning |
|------|---------------|---------------------|
| 0 | 0x04 | Bit 0: consumer (0) / pro (1). Bit 1: PCM (0) / non-PCM (1). Bit 2: copy permitted. |
| 1 | 0x00 | Category code (general) |
| 2 | 0x00 | Source number / channel number |
| 3 | varies | Sample rate code (see table below) |
| 4 | varies | Word length / original sample rate |

**Byte 3 sample rate codes (IEC 60958-3):**

| Byte 3 | Rate |
|--------|------|
| 0x00 | 44.1 kHz |
| 0x02 | 48 kHz |
| 0x08 | 88.2 kHz |
| 0x0A | 96 kHz |
| 0x0C | 176.4 kHz |
| 0x0E | 192 kHz |

**Byte 4 word length (common values):**

| Byte 4 | Meaning |
|--------|---------|
| 0x00 | Not indicated / 20-bit max |
| 0x02 | 16-bit (max 20-bit) |
| 0x0B | 24-bit (max 24-bit) |

#### Parity Error Count

The `parity_err_count` field accumulates S/PDIF subframe parity errors detected since the last lock acquisition. It resets to zero each time the receiver transitions to LOCKED state. A non-zero count while locked indicates signal integrity issues (cable problems, EMI, marginal signal level).

---

## Receiver State Machine

```
                        ┌──────────────┐
          ┌────────────→│  NO_SIGNAL   │←───────────────────┐
          │             │  (state=0)   │                    │
          │             └──────┬───────┘                    │
          │                    │ edges detected,            │
          │                    │ rate estimated             │
          │                    ▼                            │
          │             ┌──────────────┐                    │
          │  timeout    │  ACQUIRING   │  timeout (100ms)   │
          │  (100ms)    │  (state=1)   │────────────────────┘
          │             └──────┬───────┘
          │                    │ 16 consecutive
          │                    │ valid blocks
          │                    ▼
          │             ┌──────────────┐
          │  signal     │   LOCKED     │
          └─────────────│  (state=2)   │
             lost       └──────────────┘
           (100ms)
```

### State Descriptions

**NO_SIGNAL (0):** The receiver is idle or actively scanning for an S/PDIF signal. A capture PIO program samples the GPIO pin looking for transitions. If no transitions are detected within 100 ms, the capture retries. When edges are detected, the minimum run length determines the symbol rate group and the receiver transitions to ACQUIRING.

**ACQUIRING (1):** The appropriate BMC decode PIO program is loaded and running. The receiver is accumulating S/PDIF blocks and validating sync patterns. Lock requires 16 consecutive blocks where >50% of subframe sync codes are valid. If validation fails for 100 ms, the receiver falls back to NO_SIGNAL.

**LOCKED (2):** Audio data is being delivered through the DSP pipeline. The sample rate has been confirmed via IEC 60958-3 channel status (or estimated from the symbol rate). A signal loss watchdog monitors DMA activity — if no DMA block completions occur within 100 ms, the receiver transitions back to NO_SIGNAL and the `spdif_in_lost_pending` flag is raised, causing the firmware to mute all outputs.

### Timing Characteristics

| Event | Duration |
|-------|----------|
| Signal detection (edges → ACQUIRING) | < 50 ms |
| Lock acquisition (ACQUIRING → LOCKED) | ~80-320 ms (16 blocks at 48 kHz = ~64 ms + overhead) |
| Total USB→SPDIF switch time | ~100-510 ms |
| Signal loss detection | ≤ 100 ms |
| SPDIF→USB switch time | < 10 ms |

---

## Audio Signal Path

When S/PDIF is the active input source, the received audio replaces USB audio at the entry point of the DSP pipeline. The entire downstream path is identical:

```
S/PDIF RX ──→ Preamp ──→ Loudness ──→ Master EQ ──→ Crossfeed ──→ Matrix Mixer ──→ Per-Output EQ ──→ Delay ──→ S/PDIF TX
  (24-bit)                                                                                                    ──→ PDM Sub
```

### Audio Precision

| Stage | RP2040 | RP2350 |
|-------|--------|--------|
| S/PDIF input | 24-bit | 24-bit |
| Internal representation | Q28 fixed-point | IEEE 754 float |
| S/PDIF output | 24-bit | 24-bit |

The S/PDIF receiver extracts the full 24-bit audio word from each subframe. On RP2350, samples are converted to float (÷ 8388608). On RP2040, samples are left-shifted to Q28 (the same fixed-point scale used for 24-bit USB input).

### Error Concealment

When a received subframe has its validity bit set (indicating the source marked it as invalid), the receiver performs **hold concealment** — the previous good sample value is repeated. This prevents clicks from occasional transmission errors.

### Processing Trigger

In USB mode, `process_audio_packet()` is called by the USB audio packet callback (~1 ms intervals). In S/PDIF mode, a 4 ms repeating timer checks the receiver FIFO; when at least 192 stereo pairs are available, the processing function is invoked. The sample count per invocation is capped at 192 (one S/PDIF block).

### TX Clock Synchronization

When receiving S/PDIF and outputting S/PDIF simultaneously, the transmitter and receiver are driven by independent clocks (the receiver follows the source's clock, while the transmitter uses the local oscillator). To prevent the output FIFO from slowly overflowing or underflowing, the firmware implements a feedback loop:

- **Target:** RX FIFO at 50% capacity (384 stereo pairs)
- **Hysteresis band:** +/- 192 stereo pairs (one block)
- **Action:** If FIFO level is above the band, TX clock is sped up by 1 fractional LSB. If below, slowed down. Within the band, TX clock is restored to nominal.
- **Resolution:** One fractional PIO clock divider LSB corresponds to ~50 ppm at typical operating frequencies, sufficient to track crystal oscillator drift.

This keeps the FIFO centered indefinitely without any audible artifacts.

---

## Application Patterns

### Basic Source Switching

```c
#include <libusb-1.0/libusb.h>

#define REQ_SET_AUDIO_SOURCE     0x80
#define REQ_GET_AUDIO_SOURCE     0x81
#define REQ_GET_SPDIF_IN_STATUS  0x82
#define VENDOR_INTF              2

#define AUDIO_SOURCE_USB   0
#define AUDIO_SOURCE_SPDIF 1

// Switch to S/PDIF input
uint8_t source = AUDIO_SOURCE_SPDIF;
int ret = libusb_control_transfer(handle,
    LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
    REQ_SET_AUDIO_SOURCE,
    0,              // wValue
    VENDOR_INTF,    // wIndex
    &source, 1,
    2000);          // 2s timeout — switch can take up to 510ms

if (ret < 0) {
    // Transfer error (timeout, device disconnected, etc.)
    fprintf(stderr, "Source switch failed: %s\n", libusb_error_name(ret));
}
```

### Query Current Source

```c
uint8_t current_source;
int ret = libusb_control_transfer(handle,
    LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
    REQ_GET_AUDIO_SOURCE,
    0, VENDOR_INTF,
    &current_source, 1,
    1000);

if (ret == 1) {
    printf("Input: %s\n", current_source ? "S/PDIF" : "USB");
}
```

### Poll Receiver Status

```c
typedef struct __attribute__((packed)) {
    uint32_t state;
    uint32_t sample_rate;
    uint32_t parity_err_count;
    uint8_t  c_bits[5];
    uint8_t  _pad[3];
} SpdifInStatus;

SpdifInStatus status;
int ret = libusb_control_transfer(handle,
    LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
    REQ_GET_SPDIF_IN_STATUS,
    0, VENDOR_INTF,
    (uint8_t *)&status, 20,
    1000);

if (ret == 20) {
    const char *state_names[] = {"No Signal", "Acquiring", "Locked"};
    printf("State: %s\n", state_names[status.state]);
    if (status.state == 2) {
        printf("Rate: %u Hz\n", status.sample_rate);
        printf("Parity errors: %u\n", status.parity_err_count);
        printf("Channel status: %02X %02X %02X %02X %02X\n",
               status.c_bits[0], status.c_bits[1], status.c_bits[2],
               status.c_bits[3], status.c_bits[4]);
    }
}
```

### Monitoring with Auto-Switch

```c
// Example: auto-switch to SPDIF when signal detected, back to USB on loss
void monitor_spdif_source(libusb_device_handle *handle) {
    uint8_t current_source;
    SpdifInStatus status;

    // Read current source
    libusb_control_transfer(handle,
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
        REQ_GET_AUDIO_SOURCE, 0, VENDOR_INTF,
        &current_source, 1, 1000);

    // Read receiver status
    libusb_control_transfer(handle,
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
        REQ_GET_SPDIF_IN_STATUS, 0, VENDOR_INTF,
        (uint8_t *)&status, 20, 1000);

    if (current_source == AUDIO_SOURCE_USB && status.state == 2) {
        // SPDIF signal present while on USB — offer to switch
        printf("S/PDIF signal detected at %u Hz. Switch? ", status.sample_rate);
    }

    if (current_source == AUDIO_SOURCE_SPDIF && status.state == 0) {
        // Signal lost while on SPDIF — switch back to USB
        uint8_t usb = AUDIO_SOURCE_USB;
        libusb_control_transfer(handle,
            LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
            REQ_SET_AUDIO_SOURCE, 0, VENDOR_INTF,
            &usb, 1, 2000);
        printf("S/PDIF signal lost, switched to USB\n");
    }
}
```

### Status Polling Interval

For responsive UI feedback, poll `REQ_GET_SPDIF_IN_STATUS` at 200-500 ms intervals. There is no interrupt or notification mechanism — the host must poll. The status query is lightweight (reads cached state, no hardware interaction).

Avoid polling faster than 100 ms as it generates unnecessary USB traffic. The receiver state machine operates on ~50-100 ms timescales, so faster polling provides no additional information.

### Interpreting Channel Status

The `c_bits` array allows control software to display source metadata:

```c
void print_source_info(const uint8_t c_bits[5]) {
    // Byte 0
    bool is_professional = c_bits[0] & 0x01;
    bool is_pcm = !(c_bits[0] & 0x02);
    bool copy_permitted = c_bits[0] & 0x04;

    printf("Format: %s %s%s\n",
           is_professional ? "Professional" : "Consumer",
           is_pcm ? "PCM" : "Non-PCM",
           copy_permitted ? ", copy OK" : "");

    // Byte 3: sample rate
    uint32_t rate = 0;
    switch (c_bits[3] & 0x0F) {
        case 0x00: rate = 44100;  break;
        case 0x02: rate = 48000;  break;
        case 0x08: rate = 88200;  break;
        case 0x0A: rate = 96000;  break;
        case 0x0C: rate = 176400; break;
        case 0x0E: rate = 192000; break;
    }
    if (rate) printf("Rate (ch status): %u Hz\n", rate);

    // Byte 4: word length
    switch (c_bits[4] & 0x0F) {
        case 0x02: printf("Word length: 16-bit\n"); break;
        case 0x04: printf("Word length: 20-bit\n"); break;
        case 0x0A: printf("Word length: 22-bit\n"); break;
        case 0x0B: printf("Word length: 24-bit\n"); break;
        default:   printf("Word length: not indicated\n"); break;
    }
}
```

---

## Platform Differences

| Feature | RP2040 | RP2350 |
|---------|--------|--------|
| S/PDIF RX PIO | PIO1 SM1 (shared with PDM) | PIO2 SM0 (dedicated) |
| RX DMA channels | CH4, CH5 | CH6, CH7 |
| Internal precision | Q28 fixed-point (32-bit) | IEEE 754 float |
| Supported input rates | 44.1, 48, 88.2, 96, 176.4, 192 kHz | Same |
| Input bit depth | 24-bit | 24-bit |

Both platforms expose the same vendor command interface and status struct format. Control software does not need to differentiate between platforms for S/PDIF input functionality.

---

## Interaction with Other Features

### Volume and Mute

The master volume and mute controls (`REQ_SET_PREAMP`, UAC1 volume/mute) apply identically to both USB and S/PDIF input. During source switching, the firmware temporarily forces mute; the pre-switch mute state is restored afterward.

### EQ, Crossfeed, Loudness

All DSP processing applies equally to S/PDIF input. Filter coefficients are recalculated if the S/PDIF source has a different sample rate than the previous source (e.g., switching from USB at 48 kHz to S/PDIF at 44.1 kHz triggers a full coefficient recomputation).

### Matrix Mixer

The matrix mixer operates on the 2 decoded S/PDIF input channels (left/right) identically to how it operates on the 2 USB input channels. All crosspoints, output enables, gains, and delays function the same regardless of input source.

### Sample Rate

When S/PDIF is the active input, the device's sample rate is determined by the source — not by the USB host. The `REQ_GET_STATUS` command (wValue=15) returns the current operating sample rate, which will reflect the S/PDIF source rate when in S/PDIF mode.

If the S/PDIF source changes rate while locked (e.g., a CD transport switches from 44.1 to 48 kHz), the firmware detects the change, mutes briefly, reconfigures all DSP filters for the new rate, and unmutes. This is automatic and requires no host intervention.

### USB Audio Stream

When S/PDIF is the active input source, the USB audio streaming interface remains active. USB audio packets continue to arrive and are silently discarded. The USB feedback endpoint continues to report the device's sample rate. From the USB host's perspective, the device behaves normally (no underrun errors are generated by the device, though the host audio stack may detect that its output is not being consumed at the expected rate).

### Flash Persistence

The input source selection is **not** saved to flash. The device always boots with USB as the active input. Control software must re-send `REQ_SET_AUDIO_SOURCE` after each device power cycle if S/PDIF input is desired.

---

## Backward Compatibility

- Firmware without S/PDIF input support will STALL on requests 0x80, 0x81, 0x82 (standard USB behavior for unsupported vendor requests). Control software should handle the STALL gracefully and hide S/PDIF input controls.
- Use `REQ_GET_PLATFORM` (0x7F) to check the firmware version. S/PDIF input is available in firmware v1.1.0 and later.
- No existing vendor commands are modified. All prior commands (EQ, matrix mixer, crossfeed, loudness, pin config, etc.) continue to function identically.

---

## Error Handling Recommendations

| Scenario | Recommended Application Behavior |
|----------|----------------------------------|
| `REQ_SET_AUDIO_SOURCE` returns success but status shows NO_SIGNAL | The switch was attempted but no signal was found. Inform user "No S/PDIF signal detected." The device reverted to the previous source. |
| Status shows LOCKED with high `parity_err_count` | Signal integrity issue. Warn user about cable quality or interference. |
| Status shows ACQUIRING for >2 seconds | Signal is present but unstable. Suggest checking the source device or cable. |
| `REQ_SET_AUDIO_SOURCE` times out (>2s) | The device may be hung in the lock acquisition phase. Retry once, then fall back to USB. |
| Status shows rate 0 while LOCKED | Unusual — channel status not yet captured. The `sample_rate` field from `REQ_GET_SPDIF_IN_STATUS` may lag by one block (~4 ms) after lock. Retry the status query. |

---

## Request Code Summary

| Code | Command | Direction | Data | Description |
|------|---------|-----------|------|-------------|
| 0x80 | `REQ_SET_AUDIO_SOURCE` | OUT | 1 byte: source (0=USB, 1=SPDIF) | Switch input source |
| 0x81 | `REQ_GET_AUDIO_SOURCE` | IN | 1 byte: source | Query current input source |
| 0x82 | `REQ_GET_SPDIF_IN_STATUS` | IN | 20 bytes: SpdifInStatus | Query receiver status |
