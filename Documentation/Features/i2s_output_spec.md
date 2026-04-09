# I2S Output Specification

## Overview

The DSPi firmware supports per-slot I2S output alongside S/PDIF. Each output slot (0–1 on RP2040, 0–3 on RP2350) can be individually switched between S/PDIF and I2S mode at runtime. I2S slots share a common BCK/LRCLK clock pair and can optionally generate a master clock (MCK). All output slots run on the same PIO block (PIO0) and can be synchronized to start sample-aligned.

The DSP pipeline is identical for both output types. Only the final consumer buffer encoding differs — S/PDIF uses BMC/NRZI encoding (16 bytes per stereo sample) while I2S uses left-justified raw PCM (8 bytes per stereo sample). From the host application's perspective, the audio processing chain is unchanged; only the physical output format and its pin configuration are different.

## Concepts

### Output Slot

A numbered position (0 to `NUM_SPDIF_INSTANCES - 1`) that drives one stereo audio pair. Each slot has an output type (S/PDIF or I2S), a data GPIO pin, a DMA channel, and a PIO state machine. The slot index maps directly to the SM and DMA channel numbers (slot 0 = SM0/DMA0, slot 1 = SM1/DMA1, etc.).

Each slot corresponds to a stereo pair of audio outputs:

| Slot | Outputs | RP2040 | RP2350 |
|------|---------|--------|--------|
| 0 | Out 1-2 | Yes | Yes |
| 1 | Out 3-4 | Yes | Yes |
| 2 | Out 5-6 | No | Yes |
| 3 | Out 7-8 | No | Yes |

### Output Type

Each slot is either S/PDIF (`OUTPUT_TYPE_SPDIF = 0`) or I2S (`OUTPUT_TYPE_I2S = 1`). The type determines which PIO program the slot's state machine runs and how audio samples are encoded into the consumer buffer.

**All slots default to S/PDIF at boot.** The preset system may restore a different configuration if the active preset was saved with I2S slots.

### Data Pin

Each slot has an independently assignable data GPIO pin. For S/PDIF slots this carries the BMC-encoded bitstream. For I2S slots this carries the serial audio data (MSB-first, left-justified 24-bit in 32-bit frames). The same `REQ_SET_OUTPUT_PIN` / `REQ_GET_OUTPUT_PIN` commands (0x7C/0x7D) work for both output types.

Default data pins:

| Slot | Default GPIO | Function |
|------|-------------|----------|
| 0 | 6 | Data for slot 0 |
| 1 | 7 | Data for slot 1 |
| 2 | 8 | Data for slot 2 (RP2350 only) |
| 3 | 9 | Data for slot 3 (RP2350 only) |

### Clock Pins (BCK / LRCLK)

I2S requires bit clock (BCK) and left/right clock (LRCLK) signals. These are shared across ALL I2S slots — all I2S outputs use the same BCK/LRCLK pair. The LRCLK pin is always BCK + 1 due to a PIO side-set hardware constraint and cannot be independently assigned.

| Signal | Default GPIO | Assignable | Notes |
|--------|-------------|------------|-------|
| BCK | 14 | Yes (via `REQ_SET_I2S_BCK_PIN`) | Requires no I2S outputs active to change |
| LRCLK | 15 | No — always BCK + 1 | PIO side-set constraint |

BCK and LRCLK are only driven when at least one I2S slot is active. When all slots are S/PDIF, these pins are idle. Both BCK and BCK+1 are reserved in the pin-in-use check, preventing any output data pin from being assigned to either.

### Master Clock (MCK)

An optional master clock output for DACs that require an external clock reference. MCK runs on a separate PIO state machine (PIO1 SM1) and can be independently enabled/disabled.

| Property | Value |
|----------|-------|
| Default GPIO | 13 |
| Default multiplier | 128x Fs |
| Supported multipliers | 128x or 256x |
| PIO block | PIO1 |
| State machine | SM1 |

MCK frequency = multiplier x sample_rate. The MCK frequency updates automatically when the device sample rate changes (e.g., 44.1 kHz to 48 kHz).

MCK jitter characteristics at sys_clk = 307.2 MHz:

| Sample Rate | Multiplier | MCK Freq | PIO Divider | Jitter |
|-------------|-----------|----------|-------------|--------|
| 48 kHz | 128x | 6.144 MHz | 25.0 (integer) | **Zero** |
| 48 kHz | 256x | 12.288 MHz | 12.5 (fractional) | Fractional |
| 96 kHz | 128x | 12.288 MHz | 12.5 (fractional) | Fractional |
| 44.1 kHz | 128x | 5.6448 MHz | 23.4375 (fractional) | Fractional |

**Recommendation:** Use 128x at 48 kHz for zero-jitter MCK. Most modern DACs (ES9038, PCM5102, AK4490) support 128x MCK. DACs that don't need MCK can leave it disabled entirely.

### I2S Audio Format

Standard I2S (Philips) format:

| Property | Value |
|----------|-------|
| Word length | 32 bits per channel |
| Audio data | 24-bit, left-justified (MSB at bit 31) |
| Bit order | MSB first |
| BCK per frame | 64 (32 per channel) |
| MSB delay | 1 BCK after LRCLK transition |
| LRCLK polarity | Low = Left, High = Right |

The 24-bit audio sample occupies the upper 24 bits of each 32-bit word. The lower 8 bits are zero. In firmware, this is computed as `sample << 8` where the input sample is a 24-bit signed integer in the range +/-8,388,607.

### I2S Clock Math

I2S bit timing at sys_clk = 307.2 MHz:

```
BCK      = Fs x 64            = 48000 x 64 = 3,072,000 Hz
PIO_clk  = BCK x 2            = 6,144,000 Hz  (2 PIO instructions per bit)
Divider  = sys_clk * 2 / Fs   = 307200000 * 2 / 48000 = 12800
         → integer 50, frac 0  → ZERO JITTER at 48 kHz
```

### Platform Constants

| Constant | RP2040 | RP2350 | Notes |
|----------|--------|--------|-------|
| `NUM_SPDIF_INSTANCES` | 2 | 4 | Max output slots (S/PDIF or I2S) |
| `OUTPUT_TYPE_SPDIF` | 0 | 0 | S/PDIF output mode |
| `OUTPUT_TYPE_I2S` | 1 | 1 | I2S output mode |
| `PICO_I2S_BCK_PIN` | 14 | 14 | Default BCK GPIO |
| `PICO_I2S_MCK_PIN` | 13 | 13 | Default MCK GPIO |

## Vendor Commands

All I2S commands use USB EP0 control transfers on the vendor interface (interface 2). I2S configuration commands use request codes `0xC0`–`0xC9`. The existing pin configuration commands (`0x7C`–`0x7D`) also apply to I2S data pins.

### Command Summary

| Command | Code | Direction | wValue | wLength | Description |
|---------|------|-----------|--------|---------|-------------|
| `REQ_SET_OUTPUT_TYPE` | `0xC0` | IN | `(type << 8) \| slot` | 1 | Switch slot between S/PDIF and I2S |
| `REQ_GET_OUTPUT_TYPE` | `0xC1` | IN | slot (0-3) | 1 | Get slot's current output type |
| `REQ_SET_I2S_BCK_PIN` | `0xC2` | IN | new_pin | 1 | Set BCK GPIO (LRCLK = pin + 1) |
| `REQ_GET_I2S_BCK_PIN` | `0xC3` | IN | 0 | 1 | Get current BCK GPIO |
| `REQ_SET_MCK_ENABLE` | `0xC4` | IN | 0 or 1 | 1 | Enable or disable MCK output |
| `REQ_GET_MCK_ENABLE` | `0xC5` | IN | 0 | 1 | Get MCK enabled state |
| `REQ_SET_MCK_PIN` | `0xC6` | IN | new_pin | 1 | Set MCK GPIO |
| `REQ_GET_MCK_PIN` | `0xC7` | IN | 0 | 1 | Get current MCK GPIO |
| `REQ_SET_MCK_MULTIPLIER` | `0xC8` | IN | 128 or 256 | 1 | Set MCK frequency multiplier |
| `REQ_GET_MCK_MULTIPLIER` | `0xC9` | IN | 0 | 1 | Get current MCK multiplier |

### Status Codes

I2S commands that return a status byte reuse the existing pin configuration status codes:

| Code | Name | Description |
|------|------|-------------|
| `0x00` | `PIN_CONFIG_SUCCESS` | Operation succeeded |
| `0x01` | `PIN_CONFIG_INVALID_PIN` | GPIO number is out of range, reserved, or invalid value |
| `0x02` | `PIN_CONFIG_PIN_IN_USE` | GPIO is already assigned to another output |
| `0x03` | `PIN_CONFIG_INVALID_OUTPUT` | Slot index is out of range |
| `0x04` | `PIN_CONFIG_OUTPUT_ACTIVE` | Cannot change pin/type while output is active |

### Reserved GPIO Pins

The following pins cannot be assigned to any output:

| GPIO | Reason |
|------|--------|
| 12 | UART TX (debug console) |
| 23-25 | Power management / LED |
| > 28 (RP2040) or > 29 (RP2350) | Out of range |

Additionally, when setting the BCK pin, both the BCK pin AND the BCK + 1 pin (LRCLK) must be valid and not in use by other outputs. MCK pin is checked against the same reserved list and output pin assignments.

---

### REQ_SET_OUTPUT_TYPE (0xC0)

Switch an output slot between S/PDIF and I2S mode.

**Transfer type:** Control IN (Device to Host)
**bmRequestType:** `0xC1` (Device-to-Host, Vendor, Interface)
**bRequest:** `0xC0`
**wValue:** `(new_type << 8) | slot_index`
- `slot_index`: 0 to `NUM_SPDIF_INSTANCES - 1`
- `new_type`: `0` = S/PDIF, `1` = I2S
**wIndex:** 2 (vendor interface)
**wLength:** 1

**Response:** 1-byte status code.

**Behavior:**
1. Validates slot index and type value
2. If the slot is already the requested type, returns success immediately (no-op)
3. Audio is muted for ~5 ms (256 samples at 48 kHz) to prevent clicks
4. For SPDIF-to-I2S: disables SPDIF instance, sets up I2S instance with same SM/DMA/pin, connects producer pool, enables I2S
5. For I2S-to-SPDIF: tears down I2S instance, re-sets up SPDIF instance, connects, enables
6. Updates the stored output type

**Notes:**
- The data pin for the slot is unchanged during a type switch
- All I2S slots share the same BCK/LRCLK pins (set via `REQ_SET_I2S_BCK_PIN`)
- This change is NOT automatically persisted to flash. Save the active preset to persist.

**Example (C/libusb):**
```c
// Switch slot 2 to I2S
uint8_t status;
uint16_t wValue = (OUTPUT_TYPE_I2S << 8) | 2;  // type=1, slot=2
int ret = libusb_control_transfer(handle,
    0xC1,       // bmRequestType: IN, Vendor, Interface
    0xC0,       // bRequest: REQ_SET_OUTPUT_TYPE
    wValue,     // wValue: (type << 8) | slot
    2,          // wIndex: vendor interface
    &status,    // data
    1,          // wLength
    1000);      // timeout ms
if (ret == 1 && status == 0x00) {
    // Switch succeeded
}
```

---

### REQ_GET_OUTPUT_TYPE (0xC1)

Get the current output type of a slot.

**wValue:** Slot index (0 to `NUM_SPDIF_INSTANCES - 1`)
**wLength:** 1

**Response:** 1 byte — `0` = S/PDIF, `1` = I2S.

**Example:**
```c
uint8_t type;
libusb_control_transfer(handle, 0xC1, 0xC1, slot, 2, &type, 1, 1000);
printf("Slot %d: %s\n", slot, type ? "I2S" : "S/PDIF");
```

---

### REQ_SET_I2S_BCK_PIN (0xC2)

Set the BCK (bit clock) GPIO pin. LRCLK is always BCK + 1.

**wValue:** New BCK GPIO pin number
**wLength:** 1

**Response:** 1-byte status code.

**Behavior:**
1. Validates that both `new_pin` and `new_pin + 1` are valid GPIO pins
2. If unchanged from current BCK pin, returns success (no-op)
3. Checks that NO output slots are currently configured as I2S. If any slot is I2S, returns `PIN_CONFIG_OUTPUT_ACTIVE`
4. Checks both pins are not in use by output data pins or MCK
5. Stores the new BCK pin value

**Notes:**
- The BCK pin change does NOT take effect until a slot is next switched to I2S mode
- Changing the BCK pin requires all slots to be in S/PDIF mode first. Workflow: switch all I2S slots to S/PDIF → change BCK pin → switch slots back to I2S

**Example:**
```c
uint8_t status;
libusb_control_transfer(handle, 0xC1, 0xC2, /*new_pin=*/16, 2, &status, 1, 1000);
// BCK will now be GPIO 16, LRCLK will be GPIO 17
```

---

### REQ_GET_I2S_BCK_PIN (0xC3)

**wValue:** 0
**wLength:** 1

**Response:** 1 byte — BCK GPIO pin number (default: 14).

---

### REQ_SET_MCK_ENABLE (0xC4)

Enable or disable the master clock output.

**wValue:** 0 = disable, 1 = enable
**wLength:** 1

**Response:** 1-byte status code (always `PIN_CONFIG_SUCCESS`).

**Behavior:**
- **Enable:** Configures the MCK PIO clock divider for the current sample rate and multiplier, starts the MCK output
- **Disable:** Stops the MCK PIO state machine; the MCK pin goes idle

**Notes:**
- MCK can be enabled independently of whether any I2S slots are active
- MCK frequency automatically adjusts when the sample rate changes
- MCK is off by default at boot

---

### REQ_GET_MCK_ENABLE (0xC5)

**Response:** 1 byte — `0` = disabled, `1` = enabled.

---

### REQ_SET_MCK_PIN (0xC6)

Set the MCK GPIO pin.

**wValue:** New MCK GPIO pin number
**wLength:** 1

**Response:** 1-byte status code.

**Notes:**
- MCK must be disabled before changing the pin
- To change: disable MCK → change pin → re-enable MCK

---

### REQ_GET_MCK_PIN (0xC7)

**Response:** 1 byte — GPIO pin number (default: 13).

---

### REQ_SET_MCK_MULTIPLIER (0xC8)

Set the MCK frequency multiplier.

**wValue:** 128 or 256
**wLength:** 1

**Response:** 1-byte status code.

**Notes:**
- 128x at 48 kHz gives an integer PIO divider (25.0) = zero jitter (recommended)
- 256x at 48 kHz gives a fractional divider (12.5) = slight jitter
- If MCK is currently enabled, the frequency changes immediately

---

### REQ_GET_MCK_MULTIPLIER (0xC9)

**Response:** 1 byte — `0` = 128x, `1` = 256x.

---

### REQ_SET_OUTPUT_PIN (0x7C) — Updated for I2S

The existing data pin change command now works with both S/PDIF and I2S slots.

**wValue:** `(new_pin << 8) | output_index`
**wLength:** 1

**Response:** 1-byte status code.

**Behavior for I2S slots:** The slot is briefly disabled, the data pin is changed via `audio_i2s_change_data_pin()`, and the slot is re-enabled. Clock pins (BCK/LRCLK) are NOT affected.

## Preset Persistence

I2S output configuration is included in the preset system. When a preset is saved, the current output types, BCK pin, MCK settings are captured. When loaded, they are restored.

### Data Stored in Presets

| Field | Size | Description |
|-------|------|-------------|
| `output_types[4]` | 4 bytes | Per-slot output type (0=S/PDIF, 1=I2S) |
| `i2s_bck_pin` | 1 byte | BCK GPIO pin number |
| `i2s_mck_pin` | 1 byte | MCK GPIO pin number |
| `i2s_mck_enabled` | 1 byte | MCK on/off (0 or 1) |
| `i2s_mck_multiplier` | 1 byte | MCK multiplier encoding: 0 = 128x, 1 = 256x (V11+); raw 128 or 0 (V9-V10) |

### Flash Storage Version

`SLOT_DATA_VERSION` is **11** (was 10). The I2S fields are appended at the end of the `PresetSlot` structure. Version 11 changes the `i2s_mck_multiplier` encoding from raw value (128 or 0) to enum-style (0 = 128x, 1 = 256x); internal storage is `uint16_t` holding the actual multiplier value.

**Backward compatibility:** Preset slots saved with version < 9 do not contain I2S configuration. When loaded, all output types default to S/PDIF and the BCK/MCK pins retain their default values. Slots saved with version 9-10 use the old MCK multiplier encoding (raw uint8_t: 128 = 128x, 0 = 256x due to uint8 wrap); V11+ uses the new encoding. Existing presets are unaffected by the upgrade.

**Factory reset** restores: all slots to S/PDIF, BCK = GPIO 14, MCK disabled, MCK pin = GPIO 13, MCK multiplier = 128x.

## Bulk Parameter Transfer

I2S configuration is included in the bulk parameter transfer (`REQ_GET_ALL_PARAMS` / `REQ_SET_ALL_PARAMS`).

### Wire Format Version

`WIRE_FORMAT_VERSION` is **5** (was 4). The `WireI2SConfig` section is appended at the end of `WireBulkParams`. Version 5 changes the `mck_multiplier` wire encoding from raw value to enum-style (0 = 128x, 1 = 256x).

### WireI2SConfig Structure

```c
#define WIRE_MAX_SPDIF_INSTANCES 4   // RP2350 max

typedef struct __attribute__((packed)) {
    uint8_t  output_types[WIRE_MAX_SPDIF_INSTANCES]; // Per-slot output type (0 or 1)
    uint8_t  bck_pin;                // BCK GPIO pin number
    uint8_t  mck_pin;                // MCK GPIO pin number
    uint8_t  mck_enabled;            // MCK on/off (0 or 1)
    uint8_t  mck_multiplier;         // 0 = 128x, 1 = 256x (V5+); raw 128 or 0 (V2-V4)
    uint8_t  reserved[8];            // Future use (must be 0)
} WireI2SConfig;                     // 16 bytes
```

### Complete WireBulkParams Layout

```c
typedef struct __attribute__((packed)) {
    WireHeader          header;          //   16 bytes  (offset    0)
    WireGlobalParams    global;          //   16 bytes  (offset   16)
    WireCrossfeedParams crossfeed;       //   16 bytes  (offset   32)
    WireLegacyChannels  legacy;          //   16 bytes  (offset   48)
    WireChannelDelays   delays;          //   44 bytes  (offset   64)
    WireCrosspoint      crosspoints[];   //  144 bytes  (offset  108)
    WireOutputChannel   outputs[];       //  108 bytes  (offset  252)
    WirePinConfig       pins;            //    8 bytes  (offset  360)
    WireBandParams      eq[];            // 2112 bytes  (offset  368)
    WireChannelNames    channel_names;   //  352 bytes  (offset 2480)
    WireI2SConfig       i2s_config;      //   16 bytes  (offset 2832)
} WireBulkParams;                        // Total: 2848 bytes
```

### Backward Compatibility

| Scenario | Behavior |
|----------|----------|
| V5 firmware, V2-V4 host sends SET | Accepted: older `format_version` allowed, missing sections get defaults; V2-V4 MCK multiplier uses old encoding (raw value) |
| V5 firmware, V5 host sends SET | Normal: all fields including I2S config applied with new MCK multiplier encoding (0 = 128x, 1 = 256x) |
| V5 firmware, V5 host sends GET | Returns full payload including I2S config with new MCK multiplier encoding |
| V4 firmware, V5 host sends SET | V4 firmware rejects: `format_version=5` not accepted |

## Application Integration Patterns

### Initialization Flow

When a host application connects to the device:

```
1. REQ_GET_PLATFORM (0x7F)            → Get platform, firmware version, output count
2. REQ_GET_ALL_PARAMS (0xA0)          → Read complete state including I2S config
3. Parse i2s_config section:
   - For each slot: read output_types[i] to know S/PDIF vs I2S
   - Read bck_pin to display BCK/LRCLK assignment
   - Read mck_enabled, mck_pin, mck_multiplier for MCK state
4. For each output slot:
   REQ_GET_OUTPUT_PIN (0x7D)          → Get current data pin
5. Display output configuration UI
```

All I2S state is included in the bulk parameter response. There is no need to issue separate `REQ_GET_OUTPUT_TYPE` / `REQ_GET_I2S_BCK_PIN` calls during initialization.

### Switching a Slot to I2S

Recommended order of operations:

```
1. (If no I2S slots exist yet and BCK pin needs changing)
   REQ_SET_I2S_BCK_PIN (0xC2)       → Set BCK pin before first I2S switch

2. (If DAC needs MCK and MCK is not yet enabled)
   REQ_SET_MCK_ENABLE (0xC4)         → Enable MCK output

3. REQ_SET_OUTPUT_TYPE (0xC0)         → Switch slot to I2S
   // Audio briefly mutes during switch (~5 ms)

4. Update UI to reflect the slot's new type
```

### Switching a Slot Back to S/PDIF

```
1. REQ_SET_OUTPUT_TYPE (0xC0)         → Switch slot back to S/PDIF

2. If no more I2S slots remain:
   (Optional) REQ_SET_MCK_ENABLE (0xC4) with wValue=0  → Disable MCK
```

### Changing the BCK Pin

All I2S slots must be switched to S/PDIF first:

```
1. For each slot that is I2S:
   REQ_SET_OUTPUT_TYPE (0xC0)         → Switch to S/PDIF temporarily

2. REQ_SET_I2S_BCK_PIN (0xC2)        → Set new BCK pin

3. For each slot that was I2S:
   REQ_SET_OUTPUT_TYPE (0xC0)         → Switch back to I2S
```

### Persisting I2S Configuration

I2S settings are part of the live state and are captured when saving a preset:

```
1. Configure outputs as desired (type, pins, MCK)
2. REQ_PRESET_SAVE (0x90)            → Save to a preset slot
   // All I2S config is included in the saved preset
3. On next boot, the startup preset will restore the I2S config
```

### Data Structures (C)

```c
// Output type identifiers
#define OUTPUT_TYPE_SPDIF  0
#define OUTPUT_TYPE_I2S    1

// Vendor command request codes
#define REQ_SET_OUTPUT_TYPE      0xC0
#define REQ_GET_OUTPUT_TYPE      0xC1
#define REQ_SET_I2S_BCK_PIN     0xC2
#define REQ_GET_I2S_BCK_PIN     0xC3
#define REQ_SET_MCK_ENABLE      0xC4
#define REQ_GET_MCK_ENABLE      0xC5
#define REQ_SET_MCK_PIN         0xC6
#define REQ_GET_MCK_PIN         0xC7
#define REQ_SET_MCK_MULTIPLIER  0xC8
#define REQ_GET_MCK_MULTIPLIER  0xC9

// Default pins
#define PICO_I2S_BCK_PIN   14   // LRCLK = 15
#define PICO_I2S_MCK_PIN   13
```

## Hardware Reference

### PIO Resource Map

| Resource | Function |
|----------|----------|
| PIO0 SM0-SM3 | Output slots (any mix of S/PDIF and I2S) |
| PIO0 instruction memory | S/PDIF NRZI (4 instr) + I2S 24-bit (8 instr) = 12/32 |
| PIO1 SM0 | PDM subwoofer output |
| PIO1 SM1 | MCK generator (when enabled) |
| PIO2 (RP2350) | S/PDIF input + ASRC |

### I2S Pin Wiring

```
GPIO 6  ─── Data (Slot 0)  ──→ DAC1 SDIN
GPIO 7  ─── Data (Slot 1)  ──→ DAC2 SDIN
GPIO 14 ─── BCK             ──→ DAC1 BCK, DAC2 BCK
GPIO 15 ─── LRCLK           ──→ DAC1 LRCLK, DAC2 LRCLK
GPIO 13 ─── MCK (optional)  ──→ DAC1 MCK, DAC2 MCK
```

All pins are assignable. The diagram above shows default assignments.

### BSS Memory Impact

| Platform | Before I2S | After I2S | Delta |
|----------|-----------|-----------|-------|
| RP2040 | ~127 KB | ~127 KB | +292 bytes |
| RP2350 | ~205 KB | ~206 KB | +528 bytes |

The I2S library adds I2S instance structs, MCK state, and PIO program offset caches. Consumer pool buffers are only allocated when a slot is switched to I2S.
