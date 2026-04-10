# Per-Channel Preamp Specification

## 1. Overview

The Per-Channel Preamp provides independent gain control for each USB input channel. Previously, a single global preamp value was applied identically to both USB L and R input channels. Now each input channel has its own preamp gain, enabling independent left/right level adjustment before any downstream processing.

### Key characteristics

- **Per-channel gain:** Each input channel (USB L, USB R) has an independent preamp gain value in dB.
- **First in the DSP chain:** Applied during PASS 1 (input conversion), before EQ, leveller, crossfeed, matrix mixing, and output processing.
- **Full backward compatibility:** Legacy single-value commands still work, setting all channels to the same value.
- **No firmware-side linking:** Linked mode (both channels adjust together) is an app-side concept. The firmware always stores and applies per-channel values independently.

### Signal chain position

The preamp is applied at **PASS 1** in the DSP pipeline:

```
USB Audio In
    |
PASS 1: Per-Channel Preamp + Volume  <-- HERE
    |
PASS 2: Master EQ (per-channel biquad/SVF filters)
    |
PASS 2.5: Volume Leveller
    |
PASS 3: Crossfeed + Master Peak Metering
    |
PASS 4: Matrix Mixing (fan-out to output channels)
    |
PASS 5: Per-Output EQ + Delay + Gain
    |
Output Encoding (S/PDIF, I2S, PDM)
```

Because the preamp is first in the chain, its gain affects all downstream processing. A +6 dB preamp boost will raise the signal level seen by EQ, the volume leveller, crossfeed, and all output channels.

### Channel indices

| Index | Channel |
|-------|---------|
| 0 | USB Left |
| 1 | USB Right |

The array is sized by `NUM_INPUT_CHANNELS` (currently 2), which is expandable for future S/PDIF input support.

---

## 2. Parameters

### 2.1 preamp_db (per channel)

| Property | Value |
|----------|-------|
| **Type** | `float` (IEEE 754 single-precision) |
| **Range** | No firmware-enforced range (any float accepted) |
| **Default** | 0.0 (unity gain) |
| **SET command (per-channel)** | `0xD0` (`REQ_SET_PREAMP_CH`) |
| **GET command (per-channel)** | `0xD1` (`REQ_GET_PREAMP_CH`) |
| **SET command (legacy, all channels)** | `0x44` (`REQ_SET_PREAMP`) |
| **GET command (legacy, channel 0)** | `0x45` (`REQ_GET_PREAMP`) |
| **Payload** | 4 bytes: little-endian IEEE 754 float (dB value) |

Controls the input gain for a specific channel. Negative values attenuate, positive values boost. The conversion from dB to a linear multiplier uses:

```
linear = powf(10.0f, db / 20.0f)
```

Common values:

| dB | Linear multiplier | Effect |
|----|-------------------|--------|
| -12.0 | 0.251 | Significant attenuation |
| -6.0 | 0.501 | Half voltage |
| 0.0 | 1.000 | Unity (default) |
| +6.0 | 1.995 | Double voltage |
| +12.0 | 3.981 | 4x voltage |

**No range clamping:** The firmware does not clamp dB values to any range. The app is responsible for enforcing sensible limits in its UI (e.g., -24 dB to +24 dB). However, NaN and infinity values are silently rejected (the preamp value is left unchanged) to prevent audio path corruption.

### Parameter summary

| Parameter | Type | Range | Default | SET | GET | Payload |
|-----------|------|-------|---------|-----|-----|---------|
| preamp_db (per-ch) | float | unrestricted | 0.0 | 0xD0 | 0xD1 | 4 bytes (LE float) |
| preamp_db (legacy) | float | unrestricted | 0.0 | 0x44 | 0x45 | 4 bytes (LE float) |

---

## 3. Vendor Command Reference

All commands use USB EP0 vendor control transfers on the vendor interface (interface 2). SET commands are host-to-device (OUT) with data in the payload. GET commands are device-to-host (IN) with the response in the data phase.

### 3.1 REQ_SET_PREAMP_CH (0xD0) -- Per-Channel SET

| Field | Value |
|-------|-------|
| **Direction** | Host -> Device (OUT) |
| **bRequest** | `0xD0` |
| **wValue** | Channel index (0 = USB L, 1 = USB R) |
| **wIndex** | Unused (0) |
| **wLength** | 4 |
| **Payload** | 4 bytes: IEEE 754 float, little-endian (dB value) |

**Firmware behavior:**
1. Validates `wValue` < `NUM_INPUT_CHANNELS`. If out of range, the command is silently ignored.
2. Reads 4 bytes from the vendor receive buffer as a float via `memcpy`.
3. Stores the dB value in `preamp_db[wValue]`.
4. Converts to linear gain: `preamp_linear[wValue] = powf(10.0f, db / 20.0f)`.
5. On RP2040, also computes a Q28 fixed-point representation for the audio callback.

**Example:** Set USB Right channel to +3.0 dB:
```
bRequest = 0xD0, wValue = 0x0001, wIndex = 0x0000, wLength = 4
Payload: [0x00, 0x00, 0x40, 0x40]   (3.0f in little-endian IEEE 754)
```

**Example:** Set USB Left channel to -6.0 dB:
```
bRequest = 0xD0, wValue = 0x0000, wIndex = 0x0000, wLength = 4
Payload: [0x00, 0x00, 0xC0, 0xC0]   (-6.0f in little-endian IEEE 754)
```

### 3.2 REQ_GET_PREAMP_CH (0xD1) -- Per-Channel GET

| Field | Value |
|-------|-------|
| **Direction** | Device -> Host (IN) |
| **bRequest** | `0xD1` |
| **wValue** | Channel index (0 = USB L, 1 = USB R) |
| **wIndex** | Unused (0) |
| **wLength** | 4 |
| **Response** | 4 bytes: IEEE 754 float, little-endian (dB value) |

**Firmware behavior:**
1. Validates `wValue` < `NUM_INPUT_CHANNELS`. If out of range, returns 0.0 dB.
2. Returns the dB value for the requested channel.

**Example:** Get USB Left channel preamp:
```
bRequest = 0xD1, wValue = 0x0000, wIndex = 0x0000, wLength = 4
Response: [0x00, 0x00, 0x40, 0x40]   (3.0f -- channel 0 is at +3.0 dB)
```

### 3.3 REQ_SET_PREAMP (0x44) -- Legacy SET (All Channels)

| Field | Value |
|-------|-------|
| **Direction** | Host -> Device (OUT) |
| **bRequest** | `0x44` |
| **wValue** | Unused (0) |
| **wIndex** | Unused (0) |
| **wLength** | 4 |
| **Payload** | 4 bytes: IEEE 754 float, little-endian (dB value) |

**Firmware behavior:**
1. Reads 4 bytes from the vendor receive buffer as a float via `memcpy`.
2. Sets ALL input channels to the same dB value: `preamp_db[i] = db` for all `i`.
3. Converts to linear and Q28 representations for all channels.

This command is retained for backward compatibility with apps that predate per-channel support. It is also useful for implementing linked mode (see section 7).

**Example:** Set all channels to +6.0 dB:
```
bRequest = 0x44, wValue = 0x0000, wIndex = 0x0000, wLength = 4
Payload: [0x00, 0x00, 0xC0, 0x40]   (6.0f in little-endian IEEE 754)
```

### 3.4 REQ_GET_PREAMP (0x45) -- Legacy GET (Channel 0 Only)

| Field | Value |
|-------|-------|
| **Direction** | Device -> Host (IN) |
| **bRequest** | `0x45` |
| **wValue** | Unused (0) |
| **wIndex** | Unused (0) |
| **wLength** | 4 |
| **Response** | 4 bytes: IEEE 754 float, little-endian (dB value) |

**Firmware behavior:**
1. Returns the dB value of **channel 0 only** (`preamp_db[0]`).

**Important:** This command does NOT return the value of channel 1. If channels have different preamp values, this command only reflects channel 0. Always use `REQ_GET_PREAMP_CH` (0xD1) with the appropriate channel index to get accurate per-channel state.

**Example response** (channel 0 at +6.0 dB):
```
[0x00, 0x00, 0xC0, 0x40]   (6.0f)
```

---

## 4. Wire Format (Bulk Parameters)

The per-channel preamp configuration is included in the bulk parameter transfer as `WirePreampConfig`, a 16-byte packed struct.

### WirePreampConfig struct (16 bytes)

| Byte offset | Size | Type | Field | Description |
|-------------|------|------|-------|-------------|
| 0 | 4 | float | `preamp_db[0]` | Channel 0 (USB L) preamp in dB |
| 4 | 4 | float | `preamp_db[1]` | Channel 1 (USB R) preamp in dB |
| 8 | 8 | uint8_t[8] | `reserved` | Padding, must be 0 |

All multi-byte fields are little-endian. Float fields are IEEE 754 single-precision.

```c
#define WIRE_MAX_INPUT_CHANNELS 2

typedef struct __attribute__((packed)) {
    float preamp_db[WIRE_MAX_INPUT_CHANNELS];  // Per-channel preamp (dB), 0=L, 1=R
    uint8_t reserved[8];                        // Pad to 16 bytes
} WirePreampConfig;                             // 16 bytes total
```

### Position in WireBulkParams

The `WirePreampConfig` is appended to the `WireBulkParams` struct (V6+):

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
| **WirePreampConfig** | **16** | **2864** |
| WireMasterVolume | 16 | 2880 |
| **Total** | **2896** | |

### Legacy field compatibility

The `WireGlobalParams` struct retains its existing `preamp_gain_db` float field. This legacy field is populated with channel 0's preamp value on GET, maintaining backward compatibility with older apps that parse `WireGlobalParams` directly.

On SET, if the incoming `format_version` is < 6:
- The legacy `WireGlobalParams.preamp_gain_db` value is applied to ALL input channels (same behavior as the old global preamp).

On SET, if the incoming `format_version` is >= 6:
- The `WirePreampConfig` section is parsed and per-channel values are applied.
- The legacy `WireGlobalParams.preamp_gain_db` field is ignored for preamp purposes.

### Version compatibility

| Wire format version | WirePreampConfig present | SET behavior |
|---------------------|--------------------------|--------------|
| V2-V5 | No | Legacy `preamp_gain_db` applied to all channels |
| V6+ | Yes | Per-channel values from `WirePreampConfig` applied |

When the firmware sends a GET (0xA0), `WirePreampConfig` is always populated with current per-channel values and `format_version` is set to the current version.

### Bulk transfer commands

| Command | Code | Direction | Description |
|---------|------|-----------|-------------|
| `REQ_GET_ALL_PARAMS` | 0xA0 | Device -> Host | Returns full `WireBulkParams` (2896 bytes) |
| `REQ_SET_ALL_PARAMS` | 0xA1 | Host -> Device | Applies full `WireBulkParams` to live state |

---

## 5. Preset Persistence

Per-channel preamp values are saved and restored as part of the user preset system.

### Flash storage version

The per-channel preamp fields were added to the `PresetSlot` struct at `SLOT_DATA_VERSION` 12.

### PresetSlot fields

| Field | Type | Size | Description |
|-------|------|------|-------------|
| `preamp_db_per_ch[NUM_INPUT_CHANNELS]` | float[2] | 8 | Per-channel preamp in dB |

The legacy `preamp_db` field is retained in the slot struct and is always populated with channel 0's value for backward compatibility.

### Save behavior

When `REQ_PRESET_SAVE` (0x90) is issued:
```
slot->preamp_db           = preamp_db[0];           // Legacy field = channel 0
slot->preamp_db_per_ch[0] = preamp_db[0];
slot->preamp_db_per_ch[1] = preamp_db[1];
```

### Load behavior

When `REQ_PRESET_LOAD` (0x91) is issued:

- **Slot version >= 12:** Per-channel preamp values are restored from `preamp_db_per_ch[]`.
- **Slot version < 12:** The single legacy `preamp_db` value is applied to ALL input channels.
- **Unconfigured (empty) slot:** Factory defaults applied (0.0 dB for all channels).

### Factory defaults

Applied when loading an empty slot, on factory reset (`REQ_FACTORY_RESET` / 0x53), or when loading a pre-V12 slot with no legacy preamp value:

| Parameter | Default value |
|-----------|---------------|
| preamp_db[0] (USB L) | 0.0 dB |
| preamp_db[1] (USB R) | 0.0 dB |

### Boot behavior

On device startup:
1. The preset system loads the startup slot (per the startup policy).
2. If the slot contains V12+ data, per-channel preamp values are restored.
3. If the slot is pre-V12, the single `preamp_db` value applies to all channels.
4. If the slot is empty, factory defaults (0.0 dB) are applied.
5. Linear gain and Q28 representations are computed for each channel.

---

## 6. Internal Representation

The firmware maintains three parallel representations for each input channel's preamp gain:

| Representation | Type | Used by | Notes |
|----------------|------|---------|-------|
| `preamp_db[ch]` | `float` | USB commands, persistence | Stored value, source of truth |
| `preamp_linear[ch]` | `float` | RP2350 audio callback | `powf(10.0f, db / 20.0f)` |
| `preamp_q28[ch]` | `int32_t` | RP2040 audio callback | Q28 fixed-point of linear value |

All three are updated atomically (per-channel) when a SET command is processed. ARM 32-bit aligned reads and writes are inherently atomic, so the audio callback always sees a consistent value for any single channel.

**Important:** The two channels are NOT atomically updated together. If you SET channel 0 and then SET channel 1 in rapid succession, there is a brief window where channel 0 has the new value and channel 1 still has the old value. This is imperceptible in practice.

---

## 7. App Integration Guide

### Step 1: Read the current preamp state

On app startup or when connecting to a device, read the preamp for each channel:

**Option A: Individual GET commands (2 transfers)**

```
GET 0xD1, wValue=0 -> preamp_db[0]  (4 bytes, float)
GET 0xD1, wValue=1 -> preamp_db[1]  (4 bytes, float)
```

**Option B: Bulk parameter GET (1 transfer, recommended)**

Issue `REQ_GET_ALL_PARAMS` (0xA0). Parse the `WirePreampConfig` at byte offset 2864 (16 bytes). Read `preamp_db[0]` at offset 0 and `preamp_db[1]` at offset 4 within the struct.

**Do NOT use the legacy GET (0x45) for reading preamp state** -- it only returns channel 0 and cannot tell you if the channels have different values.

### Step 2: Build the UI

Recommended UI controls:

| Control | Type | Notes |
|---------|------|-------|
| Left channel preamp | Slider or knob | Range: app-defined (e.g., -24 to +24 dB) |
| Right channel preamp | Slider or knob | Same range as left |
| Link toggle | Toggle switch | When ON, both sliders move together |
| dB readout | Numeric label | Display current value with 1 decimal place |

Place the L/R sliders side by side with a link icon/toggle between them. When linked, moving either slider updates both.

### Step 3: Implement linked and independent modes

**Linked mode (both channels adjust together):**

When the user adjusts either channel with linking enabled, set both channels to the same value. Two approaches:

*Approach A: Use legacy command (simplest)*
```
// User drags left slider to +3.0 dB with link enabled
float db = 3.0f;
uint8_t payload[4];
memcpy(payload, &db, 4);
usb_vendor_out(0x44, 0, 0, payload, 4);  // Sets BOTH channels to +3.0 dB
```

*Approach B: Send two per-channel commands*
```
float db = 3.0f;
uint8_t payload[4];
memcpy(payload, &db, 4);
usb_vendor_out(0xD0, 0, 0, payload, 4);  // USB L = +3.0 dB
usb_vendor_out(0xD0, 1, 0, payload, 4);  // USB R = +3.0 dB
```

**Independent mode (channels adjust separately):**

When the user adjusts a single channel:
```
// User drags right slider to -3.0 dB
float db = -3.0f;
uint8_t payload[4];
memcpy(payload, &db, 4);
usb_vendor_out(0xD0, 1, 0, payload, 4);  // USB R only, wValue=1
```

### Step 4: Handle preset load events

When the user loads a preset (`REQ_PRESET_LOAD` / 0x91), all DSP parameters change including preamp values. After a preset load, re-read both channels:

```
GET 0xD1, wValue=0 -> update left slider
GET 0xD1, wValue=1 -> update right slider
```

Or use a single bulk GET (0xA0) to refresh all parameters at once (recommended since preset load affects everything).

After reading, check if both channels have the same value to set the initial state of the link toggle.

### Step 5: Handle bulk transfers

If your app uses bulk parameter transfers for configuration backup/restore:

**Reading (GET 0xA0):**
1. Parse the full `WireBulkParams` response.
2. Check `header.format_version`. If < 6, the `WirePreampConfig` section is not present; read the legacy `WireGlobalParams.preamp_gain_db` field and use it for both channels.
3. If >= 6, read `WirePreampConfig` at offset 2864.

**Writing (SET 0xA1):**
1. Populate both `WirePreampConfig.preamp_db[0]` and `WirePreampConfig.preamp_db[1]`.
2. Also populate `WireGlobalParams.preamp_gain_db` with channel 0's value (for legacy compat).
3. Set `header.format_version` to the current version (>= 6).
4. Send the complete struct.

### Step 6: Endianness

All numeric values in the wire protocol are **little-endian**, which is the native byte order on both ARM (the device) and x86/x64 (most host platforms). On little-endian hosts, you can use `memcpy` directly between float variables and byte buffers without conversion.

---

## 8. Backward Compatibility

This section details how the per-channel preamp interoperates with older firmware versions and older apps.

### Old app, new firmware

An app that only knows about the legacy global preamp (commands 0x44/0x45) will continue to work:

| App action | Firmware behavior |
|------------|-------------------|
| `SET 0x44` with value X | Both channels set to X |
| `GET 0x45` | Returns channel 0's value |
| Bulk SET with V5 format | Legacy `preamp_gain_db` applied to all channels |
| Bulk GET | `WireGlobalParams.preamp_gain_db` = channel 0 (old app ignores the new section) |

The only limitation: an old app cannot set L and R to different values, and cannot read channel 1's value independently. But it will never see incorrect behavior.

### New app, old firmware

An app that sends the new per-channel commands (0xD0/0xD1) to firmware that does not support them:

| App action | Old firmware behavior |
|------------|----------------------|
| `SET 0xD0` | Unrecognized vendor command, silently ignored |
| `GET 0xD1` | Unrecognized vendor command, no response (transfer stalls or returns 0) |

**Recommendation:** New apps should probe for per-channel support by issuing `GET 0xD1, wValue=0` at startup. If the transfer succeeds and returns 4 bytes, per-channel preamp is supported. If it fails or returns 0 bytes, fall back to legacy commands (0x44/0x45) and hide the per-channel UI.

### Preset compatibility

| Scenario | Behavior |
|----------|----------|
| New firmware loads V12+ preset | Per-channel values restored |
| New firmware loads pre-V12 preset | Single `preamp_db` value applied to all channels |
| Old firmware loads V12+ preset | Old firmware ignores unrecognized fields; reads its own `preamp_db` field |

### Bulk transfer compatibility

| Scenario | Behavior |
|----------|----------|
| New firmware receives V5 bulk SET | Legacy `preamp_gain_db` applied to all channels |
| New firmware receives V6+ bulk SET | Per-channel values from `WirePreampConfig` applied |
| Old firmware receives V6+ bulk SET | Old firmware parses up to its known size; extra bytes ignored |
| New firmware sends bulk GET | Both `WireGlobalParams.preamp_gain_db` (= ch 0) and `WirePreampConfig` populated |

---

## 9. Platform Notes

### Both platforms supported

Per-channel preamp is available on both RP2040 and RP2350. The vendor commands, wire format, and preset storage are identical across platforms.

### RP2350 (float pipeline)

- Storage: `float preamp_db[NUM_INPUT_CHANNELS]`, `float preamp_linear[NUM_INPUT_CHANNELS]`
- Application in audio callback: `sample *= preamp_linear[ch]`
- Single-precision float multiply, negligible CPU cost.

### RP2040 (Q28 fixed-point pipeline)

- Storage: `float preamp_db[NUM_INPUT_CHANNELS]`, `float preamp_linear[NUM_INPUT_CHANNELS]`, `int32_t preamp_q28[NUM_INPUT_CHANNELS]`
- Application in audio callback: `sample = fast_mul_q28(sample, preamp_q28[ch])`
- The dB-to-linear conversion uses float (`powf`), but this only runs when the value changes (vendor command handler), not per-sample.

### BSS (RAM) impact

Negligible. The per-channel arrays add a few tens of bytes over the previous single-value storage:

| Component | Size | Notes |
|-----------|------|-------|
| `preamp_db[2]` | 8 bytes | Float dB values |
| `preamp_linear[2]` | 8 bytes | Float linear multipliers |
| `preamp_q28[2]` (RP2040 only) | 8 bytes | Q28 fixed-point multipliers |
| **Total** | **16-24 bytes** | |

### CPU impact

Zero additional cost compared to the previous single-preamp implementation. The audio callback was already applying a preamp multiply per sample; now it indexes into a per-channel array instead of using a single global value.
