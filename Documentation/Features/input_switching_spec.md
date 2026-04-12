# Input Source Switching Specification

## 1. Overview

DSPi supports switchable audio input sources. The device defaults to USB audio input but can be switched at runtime to receive audio from an S/PDIF optical/coaxial input. The input source is selectable via vendor USB control transfers and is persisted in the user preset system.

Regardless of the selected input source, the received audio feeds through the identical DSP pipeline: preamp, loudness, master EQ, volume leveller, crossfeed, matrix mixer, per-output EQ, delay, and output gain. The input source affects only where the raw audio samples originate.

### Key characteristics

- **Two sources today:** USB (default) and S/PDIF. The enum is extensible for future I2S and ADAT inputs.
- **Non-blocking switch:** The SET command returns immediately. The actual hardware switch is deferred to the main loop.
- **Clean transitions:** Output is muted during the switch to prevent clicks and glitches.
- **Preset persistence:** The active input source is saved per-preset (SLOT_DATA_VERSION 13+).
- **USB stays enumerated:** Switching to S/PDIF does not cause USB re-enumeration.

### Signal chain position

The input source determines which hardware feeds PASS 1 of the DSP pipeline:

```
USB Audio Packets ──┐
                    ├──→ PASS 1: Per-Channel Preamp + Volume
S/PDIF RX ──────────┘         |
                          PASS 2: Master EQ
                               |
                          PASS 2.5: Volume Leveller
                               |
                          PASS 3: Crossfeed + Peak Metering
                               |
                          PASS 4: Matrix Mixing
                               |
                          PASS 5: Per-Output EQ + Delay + Output Gain
                               |
                          Output Encoding (S/PDIF TX, I2S, PDM)
```

---

## 2. InputSource Enum

The `InputSource` enum is defined in `audio_input.h`:

```c
typedef enum {
    INPUT_SOURCE_USB   = 0,
    INPUT_SOURCE_SPDIF = 1,
    // Future: INPUT_SOURCE_I2S = 2, INPUT_SOURCE_ADAT = 3
} InputSource;

#define INPUT_SOURCE_MAX    INPUT_SOURCE_SPDIF   // Highest valid value
```

| Value | Name | Description |
|-------|------|-------------|
| 0 | `INPUT_SOURCE_USB` | USB Audio Class 1 isochronous input (default) |
| 1 | `INPUT_SOURCE_SPDIF` | S/PDIF optical/coaxial receiver input |
| 2 | *(reserved)* | Future: I2S input |
| 3 | *(reserved)* | Future: ADAT input |

Values above `INPUT_SOURCE_MAX` (currently 1) are rejected by the firmware. The validation function is:

```c
static inline bool input_source_valid(uint8_t src) {
    return src <= INPUT_SOURCE_MAX;
}
```

---

## 3. Vendor Command Reference

All commands use USB EP0 vendor control transfers on the vendor interface (interface 2).

- **bmRequestType:** `0x41` (Host-to-Device SET) or `0xC1` (Device-to-Host GET)
- **wIndex:** `2` (vendor interface number)
- **Timeout:** 1000 ms recommended for all commands

### Command summary

| Code | Command | Direction | Payload | Description |
|------|---------|-----------|---------|-------------|
| 0xE0 | `REQ_SET_INPUT_SOURCE` | OUT | 1 byte | Switch input source |
| 0xE1 | `REQ_GET_INPUT_SOURCE` | IN | 1 byte | Query current input source |
| 0xE2 | `REQ_GET_SPDIF_RX_STATUS` | IN | 16 bytes | Query S/PDIF receiver state |
| 0xE3 | `REQ_GET_SPDIF_RX_CH_STATUS` | IN | 24 bytes | Query IEC 60958 channel status |
| 0xE4 | `REQ_SET_SPDIF_RX_PIN` | OUT (immediate) | wValue=pin, 1-byte response | Change SPDIF RX GPIO pin |
| 0xE5 | `REQ_GET_SPDIF_RX_PIN` | IN | 1 byte | Query SPDIF RX GPIO pin |

---

### 3.1 REQ_SET_INPUT_SOURCE (0xE0)

| Field | Value |
|-------|-------|
| **Direction** | Host -> Device (OUT) |
| **bRequest** | `0xE0` |
| **wValue** | Unused (0) |
| **wIndex** | Vendor interface number (2) |
| **wLength** | 1 |
| **Payload** | 1 byte: `InputSource` enum value |

#### Payload (1 byte)

| Offset | Size | Field | Values |
|--------|------|-------|--------|
| 0 | 1 | `source` | `0` = USB, `1` = S/PDIF |

#### Firmware behavior

1. Reads 1 byte from the vendor receive buffer.
2. Validates the value with `input_source_valid()`. Invalid values (> 1) are silently ignored.
3. If the value equals the current `active_input_source`, the command is a no-op.
4. Otherwise, sets `pending_input_source` and raises the `input_source_change_pending` flag.
5. The USB control transfer completes immediately (empty ACK). The actual hardware switch is deferred to the next main loop iteration.

**This command is non-blocking.** Unlike the older `REQ_SET_AUDIO_SOURCE` (0x80) from the legacy SPDIF input API, the 0xE0 command returns immediately. The app should poll `REQ_GET_SPDIF_RX_STATUS` (0xE2) to track the switch progress.

#### Failure behavior

| Condition | Result |
|-----------|--------|
| Invalid source value (> 1) | Silently ignored, no action |
| Already on requested source | No-op, immediate return |
| Payload too short (0 bytes) | Silently ignored |
| Multiple rapid switches | Last-write-wins (each new value overwrites `pending_input_source`) |

#### Raw USB transfer

```
bmRequestType: 0x41  (OUT | Vendor | Interface)
bRequest:      0xE0
wValue:        0x0000
wIndex:        0x0002
wLength:       0x0001
Data:          [0x01]    <-- Switch to S/PDIF
```

---

### 3.2 REQ_GET_INPUT_SOURCE (0xE1)

| Field | Value |
|-------|-------|
| **Direction** | Device -> Host (IN) |
| **bRequest** | `0xE1` |
| **wValue** | Unused (0) |
| **wIndex** | Vendor interface number (2) |
| **wLength** | 1 |
| **Response** | 1 byte: `InputSource` enum value |

#### Response (1 byte)

| Offset | Size | Field | Values |
|--------|------|-------|--------|
| 0 | 1 | `source` | `0` = USB, `1` = S/PDIF |

Returns the current `active_input_source`. This reflects the source that was most recently activated, not any pending switch. During the brief interval between a SET command and the main loop processing it, this may still return the old source.

#### Raw USB transfer

```
bmRequestType: 0xC1  (IN | Vendor | Interface)
bRequest:      0xE1
wValue:        0x0000
wIndex:        0x0002
wLength:       0x0001
Response:      [0x00]    <-- Currently on USB
```

---

### 3.3 REQ_GET_SPDIF_RX_STATUS (0xE2)

| Field | Value |
|-------|-------|
| **Direction** | Device -> Host (IN) |
| **bRequest** | `0xE2` |
| **wValue** | Unused (0) |
| **wIndex** | Vendor interface number (2) |
| **wLength** | 16 |
| **Response** | 16 bytes: `SpdifRxStatusPacket` |

#### Response (16 bytes)

| Offset | Size | Type | Field | Description |
|--------|------|------|-------|-------------|
| 0 | 1 | uint8_t | `state` | S/PDIF RX state machine (see below) |
| 1 | 1 | uint8_t | `input_source` | Current active `InputSource` enum |
| 2 | 1 | uint8_t | `lock_count` | Number of successful locks since activation |
| 3 | 1 | uint8_t | `loss_count` | Number of lock losses since activation |
| 4 | 4 | uint32_t | `sample_rate` | Detected sample rate in Hz (0 if not locked) |
| 8 | 4 | uint32_t | `parity_errors` | Cumulative parity error count |
| 12 | 2 | uint16_t | `fifo_fill_pct` | RX FIFO fill percentage (0-100) |
| 14 | 2 | uint16_t | `reserved` | Padding (ignore) |

All multi-byte fields are **little-endian** (native ARM Cortex-M byte order).

#### SpdifInputState values

| Value | Name | Description |
|-------|------|-------------|
| 0 | `INACTIVE` | RX hardware stopped (S/PDIF is not the active input) |
| 1 | `ACQUIRING` | Signal detected, waiting for lock (hardware started but not yet stable) |
| 2 | `LOCKED` | Receiving audio, sample rate confirmed |
| 3 | `RELOCKING` | Signal was lost, waiting for re-lock (output muted) |

#### Sample rate values

| `sample_rate` | Meaning |
|---------------|---------|
| 0 | Unknown / not yet detected |
| 44100 | 44.1 kHz |
| 48000 | 48 kHz |
| 88200 | 88.2 kHz |
| 96000 | 96 kHz |
| 176400 | 176.4 kHz |
| 192000 | 192 kHz |

The sample rate is only valid when `state == LOCKED`.

#### Packed struct (C)

```c
typedef struct __attribute__((packed)) {
    uint8_t  state;              // SpdifInputState enum
    uint8_t  input_source;       // Current active InputSource enum
    uint8_t  lock_count;         // Successful locks since activation
    uint8_t  loss_count;         // Lock losses since activation
    uint32_t sample_rate;        // Hz (0 if not locked)
    uint32_t parity_errors;      // Cumulative parity errors
    uint16_t fifo_fill_pct;      // 0-100
    uint16_t reserved;
} SpdifRxStatusPacket;           // 16 bytes
```

#### Hex byte layout example (locked at 48 kHz)

```
Offset: 00 01 02 03  04 05 06 07  08 09 0A 0B  0C 0D 0E 0F
Data:   02 01 03 00  80 BB 00 00  00 00 00 00  32 00 00 00
        ^  ^  ^  ^   ^---------   ^---------   ^---  ^---
        |  |  |  |   sample_rate  parity_err   fifo  rsv
        |  |  |  loss_count=0     =48000       =50%
        |  |  lock_count=3
        |  input_source=SPDIF
        state=LOCKED
```

---

### 3.4 REQ_GET_SPDIF_RX_CH_STATUS (0xE3)

| Field | Value |
|-------|-------|
| **Direction** | Device -> Host (IN) |
| **bRequest** | `0xE3` |
| **wValue** | Unused (0) |
| **wIndex** | Vendor interface number (2) |
| **wLength** | 24 |
| **Response** | 24 bytes: IEC 60958-3 channel status |

Returns the raw 24-byte (192-bit) IEC 60958-3 channel status block captured from the S/PDIF stream. Only meaningful when `SpdifInputState == LOCKED`. The first 5 bytes carry the most commonly used metadata:

| Byte | IEC 60958-3 Meaning |
|------|---------------------|
| 0 | Consumer/Pro, PCM/non-PCM, copy permission |
| 1 | Category code |
| 2 | Source/channel number |
| 3 | Sample rate code |
| 4 | Word length / original sample rate |
| 5-23 | Extended status (mostly zero for consumer sources) |

---

### 3.5 REQ_SET_SPDIF_RX_PIN (0xE4)

| Field | Value |
|-------|-------|
| **Direction** | Host -> Device (OUT, immediate response) |
| **bRequest** | `0xE4` |
| **wValue** | New GPIO pin number |
| **wIndex** | Vendor interface number (2) |
| **wLength** | 1 (response) |
| **Response** | 1 byte: status code |

Changes the GPIO pin used for S/PDIF receiver input. This is a device-level setting stored in the preset directory (not per-preset).

**This is an immediate-response SET command** -- the response byte is returned directly, not via the standard empty-ACK pattern. Use a device-to-host (IN) transfer type despite the SET semantics.

#### Status codes

| Value | Name | Description |
|-------|------|-------------|
| 0 | `SUCCESS` | Pin changed (or was already set to this value) |
| 1 | `INVALID_PIN` | Not a valid GPIO pin |
| 2 | `PIN_IN_USE` | Pin is already assigned to an output or other function |
| 3 | `OUTPUT_ACTIVE` | Cannot change while S/PDIF input is active |

**Important:** The pin cannot be changed while S/PDIF is the active input source (`active_input_source == INPUT_SOURCE_SPDIF`). Switch to USB first, change the pin, then switch back.

### 3.6 REQ_GET_SPDIF_RX_PIN (0xE5)

| Field | Value |
|-------|-------|
| **Direction** | Device -> Host (IN) |
| **bRequest** | `0xE5` |
| **wValue** | Unused (0) |
| **wIndex** | Vendor interface number (2) |
| **wLength** | 1 |
| **Response** | 1 byte: GPIO pin number |

Returns the GPIO pin currently configured for S/PDIF receiver input. Default: GPIO 11 (`PICO_SPDIF_RX_PIN_DEFAULT`).

---

## 4. Wire Format (Bulk Parameters)

The input source configuration is included in the bulk parameter transfer as `WireInputConfig`, a 16-byte packed struct appended to the end of `WireBulkParams`.

### 4.1 WireInputConfig struct (16 bytes)

| Byte offset | Size | Type | Field | Description |
|-------------|------|------|-------|-------------|
| 0 | 1 | uint8_t | `input_source` | `InputSource` enum (0=USB, 1=S/PDIF) |
| 1 | 1 | uint8_t | `spdif_rx_pin` | S/PDIF RX GPIO pin (informational) |
| 2 | 14 | uint8_t[14] | `reserved` | Must be zero. Future expansion. |

```c
typedef struct __attribute__((packed)) {
    uint8_t  input_source;       // InputSource enum (0=USB, 1=SPDIF)
    uint8_t  spdif_rx_pin;       // SPDIF RX GPIO pin (informational, SET does not apply)
    uint8_t  reserved[14];       // Future expansion (pad to 16 bytes)
} WireInputConfig;               // 16 bytes
```

**Important:** On bulk SET (0xA1), the `spdif_rx_pin` field is **informational only** and is NOT applied. Pin changes require the dedicated `REQ_SET_SPDIF_RX_PIN` (0xE4) command. The `input_source` field IS applied (deferred to main loop, same as the individual SET command).

### 4.2 Hex byte layout

```
Offset: 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F
Data:   01 0B 00 00 00 00 00 00 00 00 00 00 00 00 00 00
        ^  ^  ^----------------------------------------------
        |  |  reserved (zeros)
        |  spdif_rx_pin = GPIO 11
        input_source = SPDIF (1)
```

### 4.3 Position in WireBulkParams

The `WireInputConfig` section is the last section in the packet, appended after `WireMasterVolume`:

| Section | Size (bytes) | Cumulative offset |
|---------|-------------|-------------------|
| WireHeader | 16 | 0 |
| WireGlobalParams | 16 | 16 |
| WireCrossfeedParams | 16 | 32 |
| WireLegacyChannels | 16 | 48 |
| WireChannelDelays | 44 | 64 |
| WireCrosspoint[2][9] | 144 | 108 |
| WireOutputChannel[9] | 108 | 252 |
| WirePinConfig | 8 | 360 |
| WireBandParams[11][12] | 2112 | 368 |
| WireChannelNames | 352 | 2480 |
| WireI2SConfig | 16 | 2832 |
| WireLevellerConfig | 16 | 2848 |
| WirePreampConfig | 16 | 2864 |
| WireMasterVolume | 16 | 2880 |
| **WireInputConfig** | **16** | **2896** |
| **Total** | **2912** | |

### 4.4 Version compatibility

`WIRE_FORMAT_VERSION` = **7**. The `WireInputConfig` section was introduced in V7.

| Wire format version | WireInputConfig present | Behavior on SET |
|---------------------|-------------------------|-----------------|
| V2-V6 | No | Input source unchanged (stays at current value) |
| V7+ | Yes | `input_source` parsed and applied (deferred to main loop) |

**On GET (0xA0):** The firmware always populates `WireInputConfig` with current values and sets `format_version = 7`. The total response size is **2912 bytes**.

**On SET (0xA1):** If the incoming `format_version` < 7, the `WireInputConfig` section is not present and the input source remains at its current value. If `format_version` >= 7, the `input_source` is validated and (if different from current) queued for switching.

### 4.5 Bulk transfer commands

| Command | Code | Direction | Total size | Description |
|---------|------|-----------|------------|-------------|
| `REQ_GET_ALL_PARAMS` | 0xA0 | Device -> Host | 2912 bytes | Returns full `WireBulkParams` |
| `REQ_SET_ALL_PARAMS` | 0xA1 | Host -> Device | 2912 bytes | Applies full `WireBulkParams` to live state |

---

## 5. Preset Persistence

The input source selection is saved and restored as part of the user preset system.

### 5.1 Flash storage version

`SLOT_DATA_VERSION` = **13** (was 12). The `input_source` field was added in V13.

### 5.2 PresetSlot fields

The following fields are appended to the `PresetSlot` struct after `master_volume_db`:

| Field | Type | Size | Description |
|-------|------|------|-------------|
| `input_source` | uint8_t | 1 | `InputSource` enum (0=USB, 1=S/PDIF) |
| `input_source_padding[3]` | uint8_t[3] | 3 | Pad to 4-byte boundary |

### 5.3 Save behavior

When `REQ_PRESET_SAVE` (0x90) is issued, the current active input source is always saved:

```
slot->input_source = active_input_source;
```

### 5.4 Load behavior

When `REQ_PRESET_LOAD` (0x91) is issued:

| Slot version | Behavior |
|-------------|----------|
| V13+ | The `input_source` field is read. If it differs from the current `active_input_source` and is valid, a deferred switch is queued (`input_source_change_pending`). |
| V12 and earlier | The input source is left at its current value. No switch occurs. |
| Unconfigured (empty) slot | Factory defaults applied. Input source set to USB (0). |

The switch is deferred to the main loop (same mechanism as the `REQ_SET_INPUT_SOURCE` vendor command), so the preset load completes quickly and the mute/switch/unmute sequence runs asynchronously.

### 5.5 SPDIF RX pin storage

The S/PDIF RX pin is a **device-level** setting, not per-preset. It is stored in the `PresetDirectory` structure (flash sector 0), not in individual preset slots. Changing the RX pin via `REQ_SET_SPDIF_RX_PIN` (0xE4) triggers a deferred directory flash write.

### 5.6 Factory defaults

Applied when loading an empty slot or on factory reset (`REQ_FACTORY_RESET` / 0x53):

| Parameter | Default value |
|-----------|---------------|
| input_source | 0 (USB) |
| spdif_rx_pin | 11 (GPIO 11, stored in directory) |

---

## 6. Switching Sequences

### 6.1 USB to S/PDIF

```
App                          Firmware (USB IRQ)           Firmware (Main Loop)
 |                                |                            |
 |-- SET 0xE0 [0x01] ----------->|                            |
 |<-------- empty ACK -----------|                            |
 |                                |  pending_input_source = 1  |
 |                                |  input_source_change_pending = true
 |                                |                            |
 |                                |              main loop picks up flag:
 |                                |              1. usb_audio_drain_ring()
 |                                |              2. prepare_pipeline_reset() -> output muted
 |                                |              3. active_input_source = SPDIF
 |                                |              4. spdif_input_start() -> RX HW claimed
 |                                |              5. (output stays muted, waiting for lock)
 |                                |                            |
 |-- GET 0xE2 ------------------>|                            |
 |<-- state=ACQUIRING -----------|              ...polling for lock...
 |                                |                            |
 |                                |              SPDIF lock acquired:
 |                                |              6. spdif_input_check_rate_change()
 |                                |              7. Wait for FIFO fill (>= 25%)
 |                                |              8. complete_pipeline_reset() -> unmute
 |                                |                            |
 |-- GET 0xE2 ------------------>|                            |
 |<-- state=LOCKED, rate=48000 --|              Audio flowing:
 |                                |              SPDIF FIFO -> decode -> preamp -> DSP -> outputs
```

### 6.2 S/PDIF to USB

```
App                          Firmware (USB IRQ)           Firmware (Main Loop)
 |                                |                            |
 |-- SET 0xE0 [0x00] ----------->|                            |
 |<-------- empty ACK -----------|                            |
 |                                |  pending_input_source = 0  |
 |                                |  input_source_change_pending = true
 |                                |                            |
 |                                |              main loop picks up flag:
 |                                |              1. usb_audio_drain_ring()
 |                                |              2. prepare_pipeline_reset() -> output muted
 |                                |              3. spdif_input_stop() -> RX HW released
 |                                |              4. active_input_source = USB
 |                                |              5. usb_audio_flush_ring() -> discard stale data
 |                                |              6. complete_pipeline_reset() -> unmute
 |                                |                            |
 |-- GET 0xE1 ------------------>|                            |
 |<-- [0x00] (USB) --------------|              Audio flowing:
 |                                |              USB packets -> decode -> preamp -> DSP -> outputs
```

### 6.3 Timing

| Event | Duration |
|-------|----------|
| SET command to main loop pickup | < 1 ms (next main loop iteration) |
| Pipeline mute duration | ~5 ms (`PRESET_MUTE_SAMPLES` = 256 samples at 48 kHz) |
| S/PDIF lock acquisition | ~64-320 ms typical (library internal) |
| S/PDIF lock debounce | 100 ms (`SPDIF_RX_LOCK_DEBOUNCE_MS`) |
| FIFO fill to 25% | ~5-10 ms after lock |
| **Total USB -> S/PDIF** | **~80-450 ms silence** |
| **Total S/PDIF -> USB** | **~6-10 ms silence** |

---

## 7. Boot Behavior

The boot sequence involving input source is:

1. `preset_boot_load()` loads the startup preset from flash. If the slot is V13+ and contains `input_source = SPDIF`, the `input_source_change_pending` flag is set.
2. `spdif_input_init()` is called (initializes data structures, no PIO/DMA resources claimed).
3. If `active_input_source == INPUT_SOURCE_SPDIF` (set during preset load): `spdif_input_start()` is called to claim hardware and begin scanning.
4. Output stays muted until either:
   - S/PDIF lock is acquired and FIFO fills sufficiently (then unmute), or
   - USB audio arrives (if input source is USB, unmute happens normally).

If the saved preset specifies S/PDIF but no source is connected at boot, output remains silent indefinitely. The status will show `state = ACQUIRING`. Audio begins flowing as soon as a valid S/PDIF signal is connected.

---

## 8. USB Coexistence

When S/PDIF is the active input source:

| Aspect | Behavior |
|--------|----------|
| USB device enumeration | Unchanged. Device remains enumerated as a UAC1 audio device. |
| USB audio endpoint | Still accepts isochronous data from the host. |
| USB audio data | Accumulates in the ring buffer. Silently dropped when the ring is full. |
| SOF feedback endpoint | Continues operating (reports the device's output sample rate). |
| Host audio stack | May detect that output is not being consumed at the expected rate. |
| Switch back to USB | Ring buffer is flushed first (stale data discarded). |

The host operating system sees no difference in the USB device. There is no re-enumeration, no endpoint stall, and no alternate setting change. The only observable effect is that the audio data the host sends is not being consumed (because the DSP pipeline is processing S/PDIF audio instead).

---

## 9. Edge Cases

| Scenario | Firmware behavior |
|----------|-------------------|
| **Switch during playback** | Clean sequence: mute -> switch hardware -> unmute. No glitches. |
| **Switch to S/PDIF with no source connected** | Output stays muted indefinitely. Status shows `state = ACQUIRING`. Audio begins when a source is connected. |
| **S/PDIF signal lost while active** | State transitions to `RELOCKING`. Output muted immediately. Automatically unmutes when signal returns. |
| **Rate mismatch on switch** | Handled by `spdif_input_check_rate_change()`. All DSP filters recomputed for the new sample rate before unmuting. |
| **Multiple rapid switches** | Each SET overwrites `pending_input_source`. The main loop processes only the most recent value (last-write-wins). |
| **Switch while preset is loading** | The preset load's own mute period covers the switch. No double-mute. |
| **Boot with SPDIF preset, no source** | Output silence until a source is connected. Device is fully operational otherwise. |
| **SET with same source as current** | No-op. No mute, no hardware change. |
| **SET SPDIF RX pin while SPDIF active** | Rejected with status `OUTPUT_ACTIVE` (3). Switch to USB first. |

---

## 10. Example App Integration

This section provides practical code examples for integrating input source switching into a host application using libusb.

### 10.1 Constants and types

```c
#include <libusb-1.0/libusb.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

// Vendor command codes
#define REQ_SET_INPUT_SOURCE        0xE0
#define REQ_GET_INPUT_SOURCE        0xE1
#define REQ_GET_SPDIF_RX_STATUS     0xE2
#define REQ_GET_SPDIF_RX_CH_STATUS  0xE3
#define REQ_SET_SPDIF_RX_PIN        0xE4
#define REQ_GET_SPDIF_RX_PIN        0xE5

// Bulk transfer commands
#define REQ_GET_ALL_PARAMS          0xA0
#define REQ_SET_ALL_PARAMS          0xA1

// Vendor interface number
#define VENDOR_INTF                 2

// InputSource enum values
#define INPUT_SOURCE_USB            0
#define INPUT_SOURCE_SPDIF          1

// SpdifInputState enum values
#define SPDIF_STATE_INACTIVE        0
#define SPDIF_STATE_ACQUIRING       1
#define SPDIF_STATE_LOCKED          2
#define SPDIF_STATE_RELOCKING       3

// Status packet struct (matches firmware SpdifRxStatusPacket)
typedef struct __attribute__((packed)) {
    uint8_t  state;
    uint8_t  input_source;
    uint8_t  lock_count;
    uint8_t  loss_count;
    uint32_t sample_rate;
    uint32_t parity_errors;
    uint16_t fifo_fill_pct;
    uint16_t reserved;
} SpdifRxStatus;  // 16 bytes
```

### 10.2 Read current input source (GET 0xE1)

```c
int get_input_source(libusb_device_handle *dev, uint8_t *source_out) {
    int ret = libusb_control_transfer(dev,
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
        REQ_GET_INPUT_SOURCE,
        0,              // wValue (unused)
        VENDOR_INTF,    // wIndex
        source_out,     // 1-byte buffer
        1,              // wLength
        1000);          // timeout ms

    if (ret == 1) {
        printf("Current input source: %s\n",
               *source_out == INPUT_SOURCE_SPDIF ? "S/PDIF" : "USB");
        return 0;  // success
    }
    fprintf(stderr, "GET_INPUT_SOURCE failed: %s\n", libusb_error_name(ret));
    return -1;
}
```

### 10.3 Switch input source (SET 0xE0)

```c
int set_input_source(libusb_device_handle *dev, uint8_t source) {
    int ret = libusb_control_transfer(dev,
        LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
        REQ_SET_INPUT_SOURCE,
        0,              // wValue (unused)
        VENDOR_INTF,    // wIndex
        &source,        // 1-byte payload
        1,              // wLength
        1000);          // timeout ms

    if (ret == 1) {
        printf("Input source switch requested: %s\n",
               source == INPUT_SOURCE_SPDIF ? "S/PDIF" : "USB");
        return 0;
    }
    fprintf(stderr, "SET_INPUT_SOURCE failed: %s\n", libusb_error_name(ret));
    return -1;
}
```

### 10.4 Poll S/PDIF receiver status (GET 0xE2)

```c
int get_spdif_rx_status(libusb_device_handle *dev, SpdifRxStatus *status_out) {
    int ret = libusb_control_transfer(dev,
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
        REQ_GET_SPDIF_RX_STATUS,
        0,                            // wValue
        VENDOR_INTF,                  // wIndex
        (uint8_t *)status_out,        // 16-byte buffer
        sizeof(SpdifRxStatus),        // wLength = 16
        1000);

    if (ret == (int)sizeof(SpdifRxStatus)) {
        return 0;  // success
    }
    fprintf(stderr, "GET_SPDIF_RX_STATUS failed: %s\n", libusb_error_name(ret));
    return -1;
}

// Helper: human-readable state name
const char *spdif_state_name(uint8_t state) {
    switch (state) {
        case SPDIF_STATE_INACTIVE:  return "Inactive";
        case SPDIF_STATE_ACQUIRING: return "Acquiring";
        case SPDIF_STATE_LOCKED:    return "Locked";
        case SPDIF_STATE_RELOCKING: return "Re-locking";
        default:                    return "Unknown";
    }
}

void print_spdif_status(const SpdifRxStatus *s) {
    printf("  State:         %s (%u)\n", spdif_state_name(s->state), s->state);
    printf("  Input source:  %s\n", s->input_source ? "S/PDIF" : "USB");
    printf("  Lock count:    %u\n", s->lock_count);
    printf("  Loss count:    %u\n", s->loss_count);
    if (s->state == SPDIF_STATE_LOCKED) {
        printf("  Sample rate:   %u Hz\n", s->sample_rate);
        printf("  Parity errors: %u\n", s->parity_errors);
        printf("  FIFO fill:     %u%%\n", s->fifo_fill_pct);
    }
}
```

### 10.5 Complete switch with status polling

This example shows the full pattern for switching from USB to S/PDIF and waiting for the receiver to lock:

```c
int switch_to_spdif_and_wait(libusb_device_handle *dev, int timeout_ms) {
    // 1. Send the switch command
    if (set_input_source(dev, INPUT_SOURCE_SPDIF) != 0)
        return -1;

    // 2. Poll status until locked or timeout
    int elapsed = 0;
    const int poll_interval_ms = 200;  // poll every 200ms

    while (elapsed < timeout_ms) {
        // Platform-specific sleep (200ms)
        usleep(poll_interval_ms * 1000);
        elapsed += poll_interval_ms;

        SpdifRxStatus status;
        if (get_spdif_rx_status(dev, &status) != 0)
            return -1;

        switch (status.state) {
            case SPDIF_STATE_LOCKED:
                printf("S/PDIF locked at %u Hz after %d ms\n",
                       status.sample_rate, elapsed);
                return 0;  // success

            case SPDIF_STATE_ACQUIRING:
                printf("Acquiring S/PDIF signal... (%d ms)\n", elapsed);
                break;

            case SPDIF_STATE_INACTIVE:
                // Should not happen right after SET -- may indicate a problem
                fprintf(stderr, "Unexpected INACTIVE state\n");
                return -1;

            default:
                break;
        }
    }

    fprintf(stderr, "S/PDIF lock timeout after %d ms\n", timeout_ms);
    // Optionally switch back to USB on failure:
    // set_input_source(dev, INPUT_SOURCE_USB);
    return -1;
}

// Usage:
// switch_to_spdif_and_wait(dev, 2000);  // 2 second timeout
```

### 10.6 Build a source selection UI

Recommended UI layout for an input source selector:

```
 ┌──────────────────────────────────────────────────┐
 │  Input Source                                     │
 │                                                   │
 │  ( ) USB Audio                                    │
 │  (*) S/PDIF   [Locked 48000 Hz]                  │
 │                                                   │
 │  S/PDIF RX Pin: [GPIO 11]  [Change...]            │
 │                                                   │
 │  ┌── S/PDIF Status ──────────────────────────┐    │
 │  │  State:       Locked                       │    │
 │  │  Sample rate: 48000 Hz                     │    │
 │  │  Lock count:  3                            │    │
 │  │  Errors:      0 parity                     │    │
 │  │  FIFO:        52%                          │    │
 │  └────────────────────────────────────────────┘    │
 └──────────────────────────────────────────────────┘
```

Implementation steps:

1. **On app startup / device connect:**
   - `GET 0xE1` to read the current source, OR parse the `input_source` field from a bulk GET (0xA0).
   - `GET 0xE5` to read the S/PDIF RX pin.
   - `GET 0xE2` to read the initial S/PDIF status.
   - Set the radio button to match the current source.

2. **When the user selects a different source:**
   - `SET 0xE0` with the new source value.
   - Disable the radio buttons momentarily (prevent double-clicks).
   - Start polling `GET 0xE2` at 200 ms intervals.
   - When the status shows `LOCKED` (if switching to S/PDIF) or `INACTIVE` (if switching to USB), update the UI and re-enable the radio buttons.

3. **Background status polling (optional):**
   - While S/PDIF is the active source, poll `GET 0xE2` at 500 ms intervals to keep the status panel updated.
   - If `state` changes to `RELOCKING`, display a "Signal Lost" warning.
   - If `parity_errors` is increasing, display a "Signal Quality" warning.
   - Stop polling when USB is the active source (status will always be `INACTIVE`).

4. **Pin change flow:**
   - Verify current source is USB (GET 0xE1). If S/PDIF is active, prompt the user to switch to USB first.
   - `SET 0xE4` with `wValue = new_pin`. Check the 1-byte response for status code.
   - On success, update the displayed pin number.

### 10.7 Handle bulk params with WireInputConfig

When using bulk parameter transfers for configuration backup/restore, the `WireInputConfig` section must be handled correctly.

#### Reading (GET 0xA0)

```c
#define WIRE_INPUT_CONFIG_OFFSET  2896  // Byte offset in WireBulkParams
#define WIRE_BULK_PARAMS_SIZE     2912  // Total packet size (V7)

// After receiving the full bulk response:
uint8_t bulk_buf[WIRE_BULK_PARAMS_SIZE];
int ret = /* ... GET 0xA0 ... */;

// Check format version (byte 0 of header)
uint8_t format_version = bulk_buf[0];

if (format_version >= 7) {
    // WireInputConfig is present at offset 2896
    uint8_t input_source = bulk_buf[WIRE_INPUT_CONFIG_OFFSET];      // byte 0
    uint8_t spdif_rx_pin = bulk_buf[WIRE_INPUT_CONFIG_OFFSET + 1];  // byte 1

    printf("Input source: %s\n", input_source ? "S/PDIF" : "USB");
    printf("S/PDIF RX pin: GPIO %u\n", spdif_rx_pin);
} else {
    // V6 or earlier: no input config section. Assume USB.
    printf("Input source: USB (not in payload, firmware < V7)\n");
}
```

#### Writing (SET 0xA1)

```c
// When constructing a V7 bulk SET payload:
uint8_t bulk_buf[WIRE_BULK_PARAMS_SIZE];
memset(bulk_buf, 0, sizeof(bulk_buf));

// ... populate header, EQ, crossfeed, etc. ...

// Set format version to 7
bulk_buf[0] = 7;  // header.format_version

// Populate WireInputConfig at offset 2896
bulk_buf[WIRE_INPUT_CONFIG_OFFSET]     = INPUT_SOURCE_SPDIF;  // input_source
bulk_buf[WIRE_INPUT_CONFIG_OFFSET + 1] = 11;                  // spdif_rx_pin (informational only)
// bytes 2-15 are reserved (already zero from memset)

// Send the complete 2912-byte payload via SET 0xA1
```

**Important notes for bulk SET:**
- The `spdif_rx_pin` field is informational only. It is read on GET but not applied on SET.
- The `input_source` field IS applied on SET: it triggers the same deferred switch mechanism as `REQ_SET_INPUT_SOURCE` (0xE0).
- If `format_version` < 7 in a SET payload, the input source remains unchanged.
- Bulk SET always applies the input source regardless of any directory-level flags (there is no "include_input_source" flag).

---

## 11. Backward Compatibility

### Old apps with new firmware

Apps that do not know about the 0xE0-0xE5 commands will never send them. The device boots with USB as the default input source and remains there. All pre-existing vendor commands continue to function identically. Bulk GET responses grow from 2896 bytes (V6) to 2912 bytes (V7); old apps that request 2896 bytes via `wLength` will receive only that many bytes (USB control transfers truncate to the requested length).

### New apps with old firmware

Apps that send `REQ_SET_INPUT_SOURCE` (0xE0) or `REQ_GET_INPUT_SOURCE` (0xE1) to firmware that does not support them will receive a USB STALL response. Apps should handle this gracefully:

```c
// Probe for input switching support
uint8_t source;
int ret = libusb_control_transfer(dev,
    LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
    REQ_GET_INPUT_SOURCE, 0, VENDOR_INTF, &source, 1, 1000);

if (ret == 1) {
    // Input switching supported — show source selector UI
    show_input_source_selector(source);
} else {
    // Not supported — hide input source controls
    hide_input_source_selector();
}
```

### Old presets

Presets saved with `SLOT_DATA_VERSION` < 13 do not contain an `input_source` field. When loaded by V13+ firmware, the input source stays at its current value (no switch occurs).

### Old bulk transfer payloads

Bulk SET payloads with `format_version` < 7 do not contain the `WireInputConfig` section. When received by V7+ firmware, the input source is left unchanged.

### Migration from legacy SPDIF input API (0x80-0x82)

The earlier S/PDIF input API used different vendor command codes:

| Old command | Old code | New equivalent | New code | Notes |
|-------------|----------|----------------|----------|-------|
| `REQ_SET_AUDIO_SOURCE` | 0x80 | `REQ_SET_INPUT_SOURCE` | 0xE0 | New command is non-blocking (deferred) |
| `REQ_GET_AUDIO_SOURCE` | 0x81 | `REQ_GET_INPUT_SOURCE` | 0xE1 | Same semantics |
| `REQ_GET_SPDIF_IN_STATUS` | 0x82 | `REQ_GET_SPDIF_RX_STATUS` | 0xE2 | Different struct layout (16 bytes, not 20) |

Key differences from the legacy API:
- **0xE0 is non-blocking.** The old 0x80 blocked until the switch completed (up to 510 ms). The new 0xE0 returns immediately and the app polls 0xE2 for progress.
- **0xE2 returns a different struct.** The old 0x82 returned a 20-byte `SpdifInStatus` with `c_bits[5]`. The new 0xE2 returns a 16-byte `SpdifRxStatusPacket` with additional fields (`lock_count`, `loss_count`, `fifo_fill_pct`). Channel status bytes are now queried separately via 0xE3.
- **Persistence.** The old API did not persist the input source. The new API saves it per-preset (V13+).

---

## 12. Platform Notes

### Both platforms supported

Input source switching is available on both RP2040 and RP2350. The vendor commands, wire format, and preset storage are identical across platforms.

| Feature | RP2040 | RP2350 |
|---------|--------|--------|
| S/PDIF RX PIO | PIO1 SM2 (SM0=PDM, SM1=MCK occupied) | PIO2 SM0 (dedicated PIO block) |
| RX DMA IRQ | DMA_IRQ_0 (shared with I2S TX) | DMA_IRQ_0 (shared with I2S TX) |
| Internal precision | Q28 fixed-point | IEEE 754 float |
| Supported input rates | 44.1-192 kHz | 44.1-192 kHz |
| Input bit depth | 24-bit | 24-bit |
| Default RX pin | GPIO 11 | GPIO 11 |

Control software does not need to differentiate between platforms for input source functionality. The same vendor commands and struct layouts are used on both.

### CPU impact

Negligible. The input source check in the main loop is a single `volatile uint8_t` comparison. The S/PDIF polling path (FIFO read, clock servo) only runs when S/PDIF is the active source. When USB is active, there is zero overhead from the input source subsystem.

### BSS (RAM) impact

Minimal additions:

| Component | Size | Notes |
|-----------|------|-------|
| `active_input_source` | 1 byte | Volatile uint8_t |
| `pending_input_source` | 1 byte | Volatile uint8_t |
| `input_source_change_pending` | 1 byte | Volatile bool |
| `spdif_rx_pin` | 1 byte | uint8_t |
| Flash PresetSlot growth | 4 bytes | `input_source` + 3 padding |

---

## 13. Interaction with Other Features

### Volume and Mute

Master volume, per-channel preamp, and mute controls apply identically regardless of input source. During a source switch, the firmware temporarily mutes output; the pre-switch mute state is restored afterward.

### EQ, Crossfeed, Loudness, Leveller

All DSP processing applies equally to S/PDIF input. If the S/PDIF source has a different sample rate than the previous source (e.g., switching from USB at 48 kHz to S/PDIF at 96 kHz), all filter coefficients are recomputed for the new rate before unmuting.

### Matrix Mixer

The matrix mixer operates on the 2 decoded S/PDIF input channels (L/R) identically to how it operates on the 2 USB input channels. All crosspoints, output enables, gains, and delays function the same.

### Presets

The input source is now part of the preset state. Switching presets may cause an input source change if the loaded preset has a different source. The mute-on-load mechanism (`preset_loading` flag) covers both the preset parameter change and the input source switch.

### Output Type (S/PDIF TX, I2S, PDM)

Output configuration is completely independent of input source. All output types function identically whether the audio originates from USB or S/PDIF.

### Sample Rate

When S/PDIF is active, the device's operating sample rate is determined by the S/PDIF source, not the USB host. If the S/PDIF source changes rate while locked, the firmware detects the change, mutes, reconfigures all DSP filters, and unmutes automatically.

---

## 14. Request Code Summary

| Code | Command | Direction | Data | Description |
|------|---------|-----------|------|-------------|
| 0xE0 | `REQ_SET_INPUT_SOURCE` | OUT | 1 byte: source (0=USB, 1=SPDIF) | Switch input source (deferred) |
| 0xE1 | `REQ_GET_INPUT_SOURCE` | IN | 1 byte: source | Query current input source |
| 0xE2 | `REQ_GET_SPDIF_RX_STATUS` | IN | 16 bytes: SpdifRxStatusPacket | Query S/PDIF receiver status |
| 0xE3 | `REQ_GET_SPDIF_RX_CH_STATUS` | IN | 24 bytes: channel status | Query IEC 60958 channel status |
| 0xE4 | `REQ_SET_SPDIF_RX_PIN` | OUT (imm.) | wValue=pin, 1-byte response | Change S/PDIF RX GPIO pin |
| 0xE5 | `REQ_GET_SPDIF_RX_PIN` | IN | 1 byte: GPIO pin | Query S/PDIF RX GPIO pin |
