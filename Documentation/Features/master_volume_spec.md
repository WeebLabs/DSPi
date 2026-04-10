# Master Volume Specification

## 1. Overview

The Master Volume is a device-side output ceiling that sets the maximum output level of the DSPi hardware. It acts as an attenuator (0 dB to -127 dB) with a dedicated mute sentinel, applied at the very end of the signal chain as a multiplier on the final output gain. It is completely independent of the USB host volume and does not affect any DSP processing stages.

### Key characteristics

- **Attenuation only:** Range is 0 dB (unity, full output) to -127 dB. No amplification is possible.
- **Mute sentinel:** The value -128.0 dB represents true silence (negative infinity). Any value at or below -128 is treated as mute.
- **DSP isolation:** Loudness compensation, Volume Leveller, Master EQ, Crossfeed, and Per-Output EQ are all completely unaffected by master volume. They continue to use the raw USB host volume for their computations.
- **Core 1 transparent:** Master volume is folded into the `vol_mul` multiplier before dispatching work to Core 1, so no Core 1 code changes were needed.

### Signal chain position

The Master Volume is applied at **PASS 5** (the output gain stage), after per-output EQ and delay:

```
USB Audio In
    |
PASS 1: Preamp + Volume (USB host volume applied here)
    |
PASS 2: Master EQ (per-channel biquad/SVF filters)
    |
PASS 2.5: Volume Leveller
    |
PASS 3: Crossfeed + Master Peak Metering
    |
PASS 4: Matrix Mixing (fan-out to output channels)
    |
PASS 5: Per-Output EQ + Delay + Output Gain  <-- MASTER VOLUME APPLIED HERE
    |                                              final = sample * output_gain * (host_volume * master_volume)
Output Encoding (S/PDIF, I2S, PDM)
```

The master volume multiplier is combined with the host volume and output gain into a single linear multiplier before sample processing. The effective formula is:

```
final_output = sample * output_gain * (host_volume * master_volume)
```

Where `master_volume` is the linear equivalent of the dB setting: `master_volume = 10^(master_volume_db / 20)`, or 0.0 if master_volume_db is -128 (mute).

### Independence from USB host volume

The USB host volume (controlled by the operating system's volume slider) and the master volume are two independent gain stages that multiply together:

| Master Volume | Host Volume | Effective Output |
|---------------|-------------|------------------|
| 0 dB (unity) | 0 dB (max) | 0 dB (full output) |
| 0 dB (unity) | -10 dB | -10 dB |
| -6 dB | 0 dB (max) | -6 dB |
| -6 dB | -10 dB | -16 dB |
| -128 dB (mute) | any | silence |

The master volume sets the ceiling. The host volume operates within whatever range the master volume allows.

Critically, the master volume does not change the value that loudness compensation sees. Loudness compensation is driven by the raw USB host volume position, so it continues to apply the correct equal-loudness contour regardless of the master volume setting.

---

## 2. Parameters

### 2.1 master_volume_db

| Property | Value |
|----------|-------|
| **Type** | `float` (IEEE 754 single-precision) |
| **Range** | -128.0 (mute) to 0.0 (unity) |
| **Default** | 0.0 (unity -- no attenuation) |
| **SET command** | `0xD2` (`REQ_SET_MASTER_VOLUME`) |
| **GET command** | `0xD3` (`REQ_GET_MASTER_VOLUME`) |
| **Payload** | 4 bytes: little-endian IEEE 754 float |

The master volume level in decibels. This is an attenuation-only control:

- **0.0 dB:** Unity gain. The device outputs at full potential, limited only by the host volume and per-output gain settings.
- **-6.0 dB:** Half voltage (approximately -6 dB power reduction).
- **-20.0 dB:** One-tenth voltage. Significant attenuation.
- **-127.0 dB:** Minimum non-mute attenuation. Effectively inaudible but not digital silence.
- **-128.0 dB:** Mute sentinel. True digital silence (output is zero).

Values below -128.0 are clamped to -128.0 (mute). Values above 0.0 are clamped to 0.0 (unity). NaN and infinity values are silently rejected (the master volume is left unchanged).

### 2.2 include_master_volume

| Property | Value |
|----------|-------|
| **Type** | `uint8_t` (bool) |
| **Range** | 0 (don't restore) or 1 (restore) |
| **Default** | 0 (don't restore on preset load) |
| **SET command** | `0xD4` (`REQ_SET_INCLUDE_MASTER_VOL`) |
| **GET command** | `0xD5` (`REQ_GET_INCLUDE_MASTER_VOL`) |
| **Payload** | 1 byte |

Controls whether loading a preset also restores the master volume from that preset's saved state. This is a **directory-level** flag (global setting, not per-preset), mirroring the `include_pins` pattern.

- **0 (default):** Preset loads do not change the master volume. The master volume behaves like a physical volume knob -- it stays where the user set it regardless of preset switching.
- **1:** Preset loads restore the master volume from the preset. Useful when different presets target different speaker setups at different maximum output levels.

This flag is stored in the preset directory (flash) and persists across reboots. Setting this flag triggers a deferred flash write via the main loop.

### Parameter summary

| Parameter | Type | Range | Default | SET | GET | Payload |
|-----------|------|-------|---------|-----|-----|---------|
| master_volume_db | float | -128.0 to 0.0 | 0.0 | 0xD2 | 0xD3 | 4 bytes (LE float) |
| include_master_volume | uint8_t | 0/1 | 0 | 0xD4 | 0xD5 | 1 byte |

### Constants

The firmware defines the following constants for master volume:

```c
#define MASTER_VOL_MUTE_DB   (-128.0f)  // Sentinel: true silence (negative infinity)
#define MASTER_VOL_MIN_DB    (-127.0f)  // Minimum non-mute attenuation
#define MASTER_VOL_MAX_DB    (0.0f)     // Unity gain (no attenuation)
```

---

## 3. Vendor Command Reference

All commands use USB EP0 vendor control transfers on the vendor interface (interface 2). SET commands are host-to-device (OUT) with data in the payload. GET commands are device-to-host (IN) with the response in the data phase. The `wValue` and `wIndex` fields are not used by master volume commands (ignored by firmware).

### 3.1 REQ_SET_MASTER_VOLUME (0xD2)

| Field | Value |
|-------|-------|
| **Direction** | Host -> Device (OUT) |
| **bRequest** | `0xD2` |
| **wValue** | Unused (0) |
| **wIndex** | Unused (0) |
| **wLength** | 4 |
| **Payload** | 4 bytes: IEEE 754 float, little-endian |

**Firmware behavior:**
1. Reads 4 bytes from the vendor receive buffer as a float via `memcpy`.
2. Clamps the value to [-128.0, 0.0].
3. Converts to a linear multiplier: `10^(db / 20)`, or 0.0 if the value is -128.0 (mute).
4. Stores the dB value and the linear multiplier in the live state.
5. The new master volume takes effect on the next audio callback iteration (typically within 1ms).

**Validation:** Minimum payload length = 4 bytes. Shorter payloads are silently ignored. Out-of-range values are clamped (not rejected).

**Example:** Set master volume to -6.0 dB:
```
bRequest = 0xD2, wValue = 0x0000, wIndex = 0x0000, wLength = 4
Payload: [0x00, 0x00, 0xC0, 0xC0]   (-6.0f in little-endian IEEE 754)
```

**Example:** Mute the output:
```
bRequest = 0xD2, wValue = 0x0000, wIndex = 0x0000, wLength = 4
Payload: [0x00, 0x00, 0x00, 0xC3]   (-128.0f in little-endian IEEE 754)
```

### 3.2 REQ_GET_MASTER_VOLUME (0xD3)

| Field | Value |
|-------|-------|
| **Direction** | Device -> Host (IN) |
| **bRequest** | `0xD3` |
| **wValue** | Unused (0) |
| **wIndex** | Unused (0) |
| **wLength** | 4 |
| **Response** | 4 bytes: IEEE 754 float, little-endian |

Returns the current master volume in dB. Returns -128.0 if muted, or a value in [-127.0, 0.0] otherwise.

**Example response** (master volume at -6.0 dB):
```
[0x00, 0x00, 0xC0, 0xC0]   (-6.0f)
```

**Example response** (master volume at unity):
```
[0x00, 0x00, 0x00, 0x00]   (0.0f)
```

**Example response** (muted):
```
[0x00, 0x00, 0x00, 0xC3]   (-128.0f)
```

### 3.3 REQ_SET_INCLUDE_MASTER_VOL (0xD4)

| Field | Value |
|-------|-------|
| **Direction** | Host -> Device (OUT) |
| **bRequest** | `0xD4` |
| **wValue** | Unused (0) |
| **wIndex** | Unused (0) |
| **wLength** | 1 |
| **Payload** | 1 byte: `0x00` = don't restore, nonzero = restore |

**Firmware behavior:**
1. Reads byte 0 from the vendor receive buffer.
2. Updates the `include_master_volume` flag in the directory cache.
3. Sets a pending flag for a deferred flash write on the next main loop iteration.

**Validation:** Minimum payload length = 1 byte. Shorter payloads are silently ignored.

**Example:** Enable master volume restore on preset load:
```
bRequest = 0xD4, wValue = 0x0000, wIndex = 0x0000, wLength = 1
Payload: [0x01]
```

### 3.4 REQ_GET_INCLUDE_MASTER_VOL (0xD5)

| Field | Value |
|-------|-------|
| **Direction** | Device -> Host (IN) |
| **bRequest** | `0xD5` |
| **wValue** | Unused (0) |
| **wIndex** | Unused (0) |
| **wLength** | 1 |
| **Response** | 1 byte: `0x00` = don't restore, `0x01` = restore |

**Example response** (include_master_volume disabled, default):
```
[0x00]
```

---

## 4. Wire Format (Bulk Parameters)

The Master Volume configuration is included in the bulk parameter transfer as `WireMasterVolume`, a 16-byte packed struct. This allows the master volume to be read or written as part of a single USB control transfer alongside all other DSP parameters.

### WireMasterVolume struct (16 bytes)

| Byte offset | Size | Type | Field | Description |
|-------------|------|------|-------|-------------|
| 0 | 4 | float | `master_volume_db` | -128.0 (mute) to 0.0 dB |
| 4 | 12 | uint8_t[12] | `reserved` | Must be 0. Pad to 16 bytes. |

All multi-byte fields are little-endian. The float field is IEEE 754 single-precision.

### Position in WireBulkParams

The `WireMasterVolume` section is appended after the `WirePreampConfig` section:

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
| **WireMasterVolume** | **16** | **2880** |
| **Total** | **2896** | |

### Version compatibility

The `WireMasterVolume` section was introduced in `WIRE_FORMAT_VERSION` 6.

| Wire format version | Master volume section present | Behavior on SET |
|---------------------|-------------------------------|-----------------|
| V1-V5 | No | Master volume defaults to 0.0 dB (unity) |
| V6+ | Yes | Master volume parsed and applied |

When the firmware receives a SET (0xA1) with `format_version` < 6, master volume is set to 0.0 dB (unity). The device behaves identically to pre-master-volume firmware.

When the firmware sends a GET (0xA0), the `WireMasterVolume` section is always populated with the current value and `format_version` is set to the current version.

**Bulk SET behavior:** The master volume from a bulk SET is always applied, regardless of the `include_master_volume` directory flag. The directory flag only governs preset loads. Bulk SET is a full state replacement and always applies all fields.

### Bulk transfer commands

| Command | Code | Direction | Description |
|---------|------|-----------|-------------|
| `REQ_GET_ALL_PARAMS` | 0xA0 | Device -> Host | Returns full `WireBulkParams` (2896 bytes) |
| `REQ_SET_ALL_PARAMS` | 0xA1 | Host -> Device | Applies full `WireBulkParams` to live state |

---

## 5. Preset Persistence

Master volume is saved and restored as part of the user preset system, with a directory-level flag controlling whether preset loads actually restore the master volume.

### Flash storage version

The master volume field was added to the `PresetSlot` struct at `SLOT_DATA_VERSION` 12.

### PresetSlot fields

The following field is appended to the `PresetSlot` struct:

| Field | Type | Size | Description |
|-------|------|------|-------------|
| `master_volume_db` | float | 4 | -128.0 (mute) to 0.0 dB |

### Save behavior

When `REQ_PRESET_SAVE` (0x90) is issued, the current master volume is **always** saved into the preset slot, regardless of the `include_master_volume` flag:

```
slot->master_volume_db = master_volume_db;
```

This ensures the preset always contains a complete snapshot of the device state. The `include_master_volume` flag only controls whether the saved value is restored on load.

### Load behavior

When `REQ_PRESET_LOAD` (0x91) is issued:

- **Slot version >= 12 AND `include_master_volume` == 1:** Master volume is restored from the slot data.
- **Slot version >= 12 AND `include_master_volume` == 0:** Master volume is left unchanged (the current live value persists).
- **Slot version < 12:** Master volume is left unchanged (the slot predates master volume support).
- **Unconfigured (empty) slot:** Factory defaults are applied; master volume is set to 0.0 dB only if `include_master_volume` == 1.

### Factory defaults

Applied on factory reset (`REQ_FACTORY_RESET` / 0x53):

| Parameter | Default value |
|-----------|---------------|
| master_volume_db | 0.0 (unity) |
| include_master_volume | 0 (don't restore) |

### Boot behavior

On device startup:
1. Master volume is initialized to 0.0 dB (unity).
2. The preset system loads the startup slot (per the startup policy).
3. If the slot contains V12+ data and `include_master_volume` is set, master volume is restored from the slot.
4. Otherwise, master volume remains at 0.0 dB.

### Preset directory response

The `REQ_PRESET_GET_DIR` (0x95) response now includes a 7th byte for the `include_master_volume` flag:

| Byte | Field | Description |
|------|-------|-------------|
| 0-1 | `slot_occupied` | Little-endian uint16_t bitmask (bit N = slot N occupied) |
| 2 | `startup_mode` | 0 = specified default, 1 = last active |
| 3 | `default_slot` | Default startup slot index (0-9) |
| 4 | `last_active_slot` | Last active slot index (0-9) |
| 5 | `include_pins` | 0 = don't restore pins on load, 1 = restore |
| 6 | `include_master_volume` | 0 = don't restore master volume on load, 1 = restore |

**Backward compatibility:** Apps requesting 6 bytes (the pre-V12 directory size) will receive only the first 6 bytes. USB control transfers naturally truncate to the requested `wLength`. No error occurs; the app simply does not see the new byte.

---

## 6. App Integration Guide

This section walks through how to add Master Volume support to a host application that communicates with the DSPi firmware over USB.

### Step 1: Read the current master volume

On app startup or when connecting to a device, read the master volume. You have two options:

**Option A: Individual GET command (1 transfer)**

```
GET 0xD3 -> master_volume_db  (4 bytes, float)
```

**Option B: Bulk parameter GET (1 transfer, recommended)**

Issue `REQ_GET_ALL_PARAMS` (0xA0). The response is a `WireBulkParams` struct (2896 bytes). Parse the `WireMasterVolume` at byte offset 2880 (the last 16 bytes). Check `header.format_version >= 6`; if the format version is older, master volume is not present and should be displayed as 0.0 dB.

Also read the directory flag:

```
GET 0xD5 -> include_master_volume  (1 byte)
```

Or parse byte 6 of the `REQ_PRESET_GET_DIR` (0x95) response (request `wLength = 7`).

### Step 2: Build the UI

Recommended UI layout:

**Volume slider:**
- Range: -127 dB to 0 dB (or a perceptual mapping, see below)
- A separate mute toggle button that sends -128.0
- Display "MUTE" when the value is -128, otherwise display the dB value (e.g., "-6.0 dB")
- Consider a logarithmic slider scale for better UX: most users operate in the -30 to 0 dB range

**Logarithmic slider mapping (recommended):**
```
// Slider position 0.0 to 1.0 -> dB value
// Uses a power curve for better perceptual distribution
float slider_to_db(float pos) {
    // pos = 0.0 -> -127 dB, pos = 1.0 -> 0 dB
    return -127.0f * powf(1.0f - pos, 3.0f);
}

float db_to_slider(float db) {
    // db = -127 -> 0.0, db = 0 -> 1.0
    return 1.0f - powf(-db / 127.0f, 1.0f / 3.0f);
}
```

**Include master volume toggle:**
- A checkbox or toggle in a settings/preferences area
- Label: "Restore master volume on preset load"
- Default: off
- When enabled, switching presets may change the output volume

### Step 3: Send SET commands on user interaction

When the user changes the volume slider, send the corresponding SET command immediately:

```c
// User drags slider to -12.0 dB
float vol = -12.0f;
uint8_t payload[4];
memcpy(payload, &vol, 4);  // little-endian on ARM/x86
usb_vendor_out(0xD2, payload, 4);

// User presses mute toggle
float mute = -128.0f;
memcpy(payload, &mute, 4);
usb_vendor_out(0xD2, payload, 4);

// User unmutes (restore previous volume)
float restore = -12.0f;  // app remembers the pre-mute value
memcpy(payload, &restore, 4);
usb_vendor_out(0xD2, payload, 4);

// User enables include_master_volume
uint8_t include = 1;
usb_vendor_out(0xD4, &include, 1);
```

**Important:** The firmware applies the new master volume value immediately (within the next audio callback, typically < 1ms). There is no pending/deferred mechanism for master volume itself -- unlike EQ coefficient updates, master volume is a single float that can be applied atomically. There is no acknowledgment; if the USB transfer completes successfully, the value was accepted.

### Step 4: Handle preset load events

When the user loads a preset (via `REQ_PRESET_LOAD` / 0x91), the master volume may or may not change depending on the `include_master_volume` flag. After a preset load, re-read the master volume to update the UI:

```
GET 0xD3 -> update volume slider / mute state
```

Or use a single bulk GET (0xA0) to refresh all parameters at once. This is the preferred approach since a preset load affects all DSP parameters.

### Step 5: Handle bulk transfers

If your app uses bulk parameter transfers for configuration backup/restore:

**Reading (GET 0xA0):**
1. Parse the full `WireBulkParams` response.
2. Check `header.format_version`. If < 6, the `WireMasterVolume` section is not present; display 0.0 dB.
3. If >= 6, read `WireMasterVolume` at offset 2880.

**Writing (SET 0xA1):**
1. Populate `WireMasterVolume` in the `WireBulkParams` struct.
2. Set `header.format_version` to the current version (6 or later).
3. Send the complete struct.
4. Bulk SET always applies master volume regardless of the `include_master_volume` directory flag.

### Step 6: Mute implementation pattern

The recommended pattern for implementing a mute toggle in the app:

```c
static float saved_master_vol = 0.0f;
static bool is_muted = false;

void on_mute_toggle() {
    if (is_muted) {
        // Unmute: restore saved volume
        set_master_volume(saved_master_vol);
        is_muted = false;
    } else {
        // Mute: save current volume, send mute sentinel
        GET master_volume -> saved_master_vol
        set_master_volume(-128.0f);
        is_muted = true;
    }
}

void on_connect() {
    float vol;
    GET 0xD3 -> vol;
    if (vol <= -128.0f) {
        is_muted = true;
        saved_master_vol = 0.0f;  // no way to know pre-mute value
    } else {
        is_muted = false;
        saved_master_vol = vol;
    }
}
```

Note: The firmware does not store a separate "pre-mute" value. If the app disconnects while muted and reconnects, it will read -128.0 and cannot recover the previous volume. The app should persist the pre-mute value locally if needed.

### Step 7: Endianness

All numeric values in the wire protocol are **little-endian**, which is the native byte order on both ARM (the device) and x86/x64 (most host platforms). On little-endian hosts, you can use `memcpy` directly between float variables and byte buffers without conversion. On big-endian hosts (rare), byte-swap is required for float fields.

---

## 7. Backward Compatibility

### Old apps with new firmware

Apps that do not know about master volume will never send `REQ_SET_MASTER_VOLUME` (0xD2). The master volume defaults to 0.0 dB (unity) on boot and remains there. The device behaves identically to pre-master-volume firmware. No existing vendor commands are affected.

### New apps with old firmware

Apps that send `REQ_SET_MASTER_VOLUME` (0xD2) to firmware that does not support it will receive a USB STALL response (the vendor command is unrecognized). Apps should handle this gracefully -- for example, by hiding the master volume UI or displaying it as unavailable.

Detection strategy: attempt `REQ_GET_MASTER_VOLUME` (0xD3). If the transfer completes successfully, master volume is supported. If it STALLs, it is not.

### Old presets

Presets saved with `SLOT_DATA_VERSION` < 12 do not contain a `master_volume_db` field. When loaded:
- The master volume is left unchanged (not reset to 0 dB), regardless of the `include_master_volume` flag.
- All other preset fields are restored normally.

### Old bulk transfer payloads

Bulk SET payloads with `format_version` < 6 do not contain the `WireMasterVolume` section. When received:
- The master volume is set to 0.0 dB (unity).
- This is a safe default: full output, no attenuation, matching pre-master-volume behavior.

### Preset directory size

The directory response grew from 6 to 7 bytes. Apps requesting the old 6-byte size continue to work because USB control transfers truncate to `wLength`. Apps that want the new byte must request `wLength >= 7`.

---

## 8. Platform Notes

### Both platforms supported

Master volume is available on both RP2040 and RP2350. The vendor commands, wire format, and preset storage are identical across platforms. The internal representation is a single float dB value and a precomputed linear multiplier.

### Implementation detail

The master volume is folded into the existing `vol_mul` computation at PASS 5:

```
vol_mul = output_gain[ch] * host_volume * master_volume_linear
```

This multiplication happens once per block per output channel (not per sample), so the CPU cost is negligible. On RP2040 (Q28 pipeline), the master volume linear multiplier is converted to Q28 before multiplication. On RP2350 (float pipeline), it remains as a float.

Because the master volume is folded into `vol_mul` before the Core 1 EQ worker dispatch, Core 1 receives the already-adjusted multiplier. No Core 1 code changes were necessary.

### CPU impact

Effectively zero. The master volume adds a single float multiply to the per-block `vol_mul` computation. This is negligible compared to the EQ, crossfeed, and other DSP operations.

### BSS (RAM) impact

Negligible. The master volume adds:
- 4 bytes for `master_volume_db` (float)
- 4 bytes for `master_volume_linear` (precomputed float multiplier)
- 1 byte for `include_master_volume` in the directory cache

### Sample rate handling

Master volume is a simple linear multiplier and is not affected by sample rate changes. No coefficient recomputation is needed when the sample rate changes.
