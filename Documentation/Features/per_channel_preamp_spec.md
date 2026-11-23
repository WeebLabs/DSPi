# Per-Channel Preamp Specification

*Last updated: 2026-11-16 (vendor commands on the paged protocol of firmware 1.1.7, legacy all-channel commands retired)*

## 1. Overview

The Per-Channel Preamp provides independent gain control for each USB input channel. Previously, a single global preamp value was applied identically to both USB L and R input channels. Now each input channel has its own preamp gain, enabling independent left/right level adjustment before any downstream processing.

### Key characteristics

- **Per-channel gain:** Each input channel (USB L, USB R) has an independent preamp gain value in dB.
- **First in the DSP chain:** Applied during PASS 1 (input conversion), before EQ, leveller, crossfeed, matrix mixing, and output processing.
- **One command per channel:** `ROUTE.PREAMP` addresses one input channel. The legacy all-channel commands of firmware 1.1.6 (opcodes 0x44/0x45) are retired in 1.1.7.
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

The array is sized by `NUM_INPUT_CHANNELS`: **2 on RP2040**, **8 on RP2350** (inputs 0/1 are USB L/R; inputs 2-7 carry the extra channels in 8-channel USB input mode — see `usb_8ch_input_spec.md`). On RP2350 the per-channel preamp commands accept channel index 0-7; in the wire format, inputs 2-7 preamp lives in the V15 `WireInputExtConfig` tail section.

---

## 2. Parameters

### 2.1 preamp_db (per channel)

| Property | Value |
|----------|-------|
| **Type** | `float` (IEEE 754 single-precision) |
| **Range** | No firmware-enforced range (any float accepted) |
| **Default** | 0.0 (unity gain) |
| **Command** | `ROUTE.PREAMP` (page 0x05, command 0x40), GET and SET, input channel in `wValue` |
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

**No range clamping:** The firmware does not clamp dB values to any range. The app is responsible for enforcing sensible limits in its UI (e.g., -24 dB to +24 dB). However, a NaN or infinite value is refused with `VC_ERR_VALUE` (the SET STALLs and the preamp value is left unchanged) to prevent audio path corruption.

### Parameter summary

| Parameter | Type | Range | Default | Command | Payload |
|-----------|------|-------|---------|---------|---------|
| preamp_db (per-ch) | float | unrestricted (finite) | 0.0 | `ROUTE.PREAMP` | 4 bytes (LE float) |

---

## 3. Vendor Command Reference

The preamp uses one command of the paged protocol (see `../commands.md`). The page travels in the high byte of `wIndex` and the vendor interface (2) in the low byte, so every request carries `wIndex` 0x0502.

| Command | Page | Cmd | Dir | Address (`wValue`) | Size |
|---|---|---|---|---|---|
| `ROUTE.PREAMP` | 0x05 | 0x40 | GET, SET | input channel 0 to `NUM_INPUT_CHANNELS` - 1 | GET 4, SET 4 |

### 3.1 `ROUTE.PREAMP` SET

| Field | Value |
|-------|-------|
| **Direction** | Host -> Device (OUT, `bmRequestType` 0x41) |
| **bRequest** | `0x40` |
| **wValue** | Input channel index (0 = USB L, 1 = USB R) |
| **wIndex** | `0x0502` |
| **wLength** | 4 |
| **Payload** | 4 bytes: IEEE 754 float, little-endian (dB value) |

**Firmware behavior:**
1. The dispatcher checks `wValue` < `NUM_INPUT_CHANNELS` and the payload length of exactly 4. A bad channel STALLs with `VC_ERR_ADDRESS` and a wrong length with `VC_ERR_LENGTH`.
2. A NaN or infinite value STALLs with `VC_ERR_VALUE` and stores nothing.
3. Stores the dB value in `preamp_db[wValue]`.
4. Converts to linear gain: `preamp_linear[wValue] = powf(10.0f, db / 20.0f)`.
5. On RP2040, also computes a Q28 fixed-point representation for the audio callback.

**Example:** Set USB Right channel to +3.0 dB:
```
bRequest = 0x40, wValue = 0x0001, wIndex = 0x0502, wLength = 4
Payload: [0x00, 0x00, 0x40, 0x40]   (3.0f in little-endian IEEE 754)
```

**Example:** Set USB Left channel to -6.0 dB:
```
bRequest = 0x40, wValue = 0x0000, wIndex = 0x0502, wLength = 4
Payload: [0x00, 0x00, 0xC0, 0xC0]   (-6.0f in little-endian IEEE 754)
```

### 3.2 `ROUTE.PREAMP` GET

| Field | Value |
|-------|-------|
| **Direction** | Device -> Host (IN, `bmRequestType` 0xC1) |
| **bRequest** | `0x40` |
| **wValue** | Input channel index (0 = USB L, 1 = USB R) |
| **wIndex** | `0x0502` |
| **wLength** | 4 |
| **Response** | 4 bytes: IEEE 754 float, little-endian (dB value) |

**Firmware behavior:**
1. A `wValue` at or above `NUM_INPUT_CHANNELS` STALLs with `VC_ERR_ADDRESS`. Firmware 1.1.6 returned 0.0 dB instead.
2. Returns the dB value for the requested channel.

**Example:** Get USB Left channel preamp:
```
bRequest = 0x40, wValue = 0x0000, wIndex = 0x0502, wLength = 4
Response: [0x00, 0x00, 0x40, 0x40]   (3.0f -- channel 0 is at +3.0 dB)
```

### 3.3 Retired legacy commands

Firmware 1.1.6 also had an all-channel SET (opcode 0x44, set every input channel to one value) and a channel 0 GET (opcode 0x45). Both are retired in 1.1.7 and have no paged equivalent. A host sets each channel with `ROUTE.PREAMP` and reads each channel the same way.

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

When the firmware answers a `STORE.ALL_PARAMS` GET, `WirePreampConfig` is always populated with current per-channel values and `format_version` is set to the current version.

### Bulk transfer commands

| Command | Page | Cmd | Dir | Address (`wValue`) | Size | Description |
|---|---|---|---|---|---|---|
| `STORE.ALL_PARAMS` | 0x02 | 0x40 | GET, SET | none (0) | GET 6284, SET 6284 | GET returns the full `WireBulkParams` (2896 bytes at V6, 6284 at V36). SET applies it to live state |

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

When `STORE.PRESET_SAVE` is issued:
```
slot->preamp_db           = preamp_db[0];           // Legacy field = channel 0
slot->preamp_db_per_ch[0] = preamp_db[0];
slot->preamp_db_per_ch[1] = preamp_db[1];
```

### Load behavior

When `STORE.PRESET_LOAD` is issued:

- **Slot version >= 12:** Per-channel preamp values are restored from `preamp_db_per_ch[]`.
- **Slot version < 12:** The single legacy `preamp_db` value is applied to ALL input channels.
- **Unconfigured (empty) slot:** Factory defaults applied (0.0 dB for all channels).

### Factory defaults

Applied when loading an empty slot, on factory reset (`STORE.FACTORY_RESET`), or when loading a pre-V12 slot with no legacy preamp value:

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
GET ROUTE.PREAMP, wValue=0 -> preamp_db[0]  (4 bytes, float)
GET ROUTE.PREAMP, wValue=1 -> preamp_db[1]  (4 bytes, float)
```

**Option B: Bulk parameter GET (1 transfer, recommended)**

Issue a `STORE.ALL_PARAMS` GET. Parse the `WirePreampConfig` at byte offset 2864 (16 bytes). Read `preamp_db[0]` at offset 0 and `preamp_db[1]` at offset 4 within the struct.

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

When the user adjusts either channel with linking enabled, set both channels to the same value with one `ROUTE.PREAMP` SET per channel. There is no all-channel form in 1.1.7.

```
// usb_vendor_out(bRequest, wValue, wIndex, payload, len)
float db = 3.0f;
uint8_t payload[4];
memcpy(payload, &db, 4);
usb_vendor_out(0x40, 0, 0x0502, payload, 4);  // ROUTE.PREAMP, USB L = +3.0 dB
usb_vendor_out(0x40, 1, 0x0502, payload, 4);  // ROUTE.PREAMP, USB R = +3.0 dB
```

**Independent mode (channels adjust separately):**

When the user adjusts a single channel:
```
// User drags right slider to -3.0 dB
float db = -3.0f;
uint8_t payload[4];
memcpy(payload, &db, 4);
usb_vendor_out(0x40, 1, 0x0502, payload, 4);  // ROUTE.PREAMP, USB R only, wValue=1
```

### Step 4: Handle preset load events

When the user loads a preset (`STORE.PRESET_LOAD`), all DSP parameters change including preamp values. After a preset load, re-read both channels:

```
GET ROUTE.PREAMP, wValue=0 -> update left slider
GET ROUTE.PREAMP, wValue=1 -> update right slider
```

Or use a single `STORE.ALL_PARAMS` GET to refresh all parameters at once (recommended since preset load affects everything).

After reading, check if both channels have the same value to set the initial state of the link toggle.

### Step 5: Handle bulk transfers

If your app uses bulk parameter transfers for configuration backup/restore:

**Reading (`STORE.ALL_PARAMS` GET):**
1. Parse the full `WireBulkParams` response.
2. Check `header.format_version`. If < 6, the `WirePreampConfig` section is not present; read the legacy `WireGlobalParams.preamp_gain_db` field and use it for both channels.
3. If >= 6, read `WirePreampConfig` at offset 2864.

**Writing (`STORE.ALL_PARAMS` SET):**
1. Populate both `WirePreampConfig.preamp_db[0]` and `WirePreampConfig.preamp_db[1]`.
2. Also populate `WireGlobalParams.preamp_gain_db` with channel 0's value (for legacy compat).
3. Set `header.format_version` to the current version (>= 6).
4. Send the complete struct.

### Step 6: Endianness

All numeric values in the wire protocol are **little-endian**, which is the native byte order on both ARM (the device) and x86/x64 (most host platforms). On little-endian hosts, you can use `memcpy` directly between float variables and byte buffers without conversion.

---

## 8. Backward Compatibility

This section details how the per-channel preamp interoperates with older firmware versions and older apps.

### Hosts and firmware 1.1.7

Firmware 1.1.7 speaks only the paged protocol, so a host built for 1.1.6 cannot drive the preamp on it at all (every page 0 request is refused). The 1.1.6 behaviour below is kept for hosts that still support older firmware.

- On firmware 1.1.6, opcode 0x44 set every channel to one value and opcode 0x45 returned channel 0.
- Firmware that predates per-channel preamp ignored opcode 0xD0 and stalled or answered nothing to 0xD1. A host that supports such firmware probes 0xD1 with `wValue` 0 and hides the per-channel UI when it fails.
- On paged firmware there is nothing to probe. Every paged firmware has `ROUTE.PREAMP`, and a host that gates on the command table can check its bit in `SYS.PAGE_COMMANDS` for page 0x05.

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

Per-channel preamp is available on both RP2040 and RP2350. The vendor command, wire format, and preset storage are identical across platforms.

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
