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

### 2.2 master_volume_mode

| Property | Value |
|----------|-------|
| **Type** | `uint8_t` |
| **Range** | 0 (independent) or 1 (with preset) |
| **Default** | 0 (independent) |
| **SET command** | `0xD4` (`REQ_SET_MASTER_VOLUME_MODE`) |
| **GET command** | `0xD5` (`REQ_GET_MASTER_VOLUME_MODE`) |
| **Payload** | 1 byte |

Selects how master volume is persisted across boots. This is a **directory-level** setting (global, not per-preset), mirroring `include_pins`.

- **Mode 0 — Independent (default).** Master volume is a stand-alone device setting, saved in the preset directory, applied at boot, and completely unaffected by preset save/load. The app persists a chosen value by issuing `REQ_SAVE_MASTER_VOLUME` (0xD6); the device then boots at that value on subsequent power-ups. Preset save still writes `master_volume_db` into the slot (so the same flash layout works for either mode), but the saved slot value is ignored on load. Behaves like a physical volume knob: preset switching never moves the knob.
- **Mode 1 — With preset.** Master volume is part of the preset. Saved with the preset, restored on preset load, same as other DSP parameters. Useful when different presets target different speaker setups at different maximum output levels.

Values outside `[0, 1]` are clamped to 0. Setting the mode triggers a deferred flash write via the main loop. The value itself persists in the preset directory across reboots.

### 2.3 saved master volume (independent storage)

The directory sector also carries a `master_volume_db` field — the value applied at boot when mode is 0. Updated by `REQ_SAVE_MASTER_VOLUME` (copies the current live `master_volume_db` into the field and flushes the directory). Readable via `REQ_GET_SAVED_MASTER_VOLUME`. The save command is accepted in both modes; in mode 1 the stored value is dormant until the user switches to mode 0.

### Parameter summary

| Parameter | Type | Range | Default | SET | GET | Payload |
|-----------|------|-------|---------|-----|-----|---------|
| master_volume_db (live) | float | -128.0 to 0.0 | 0.0 | 0xD2 | 0xD3 | 4 bytes (LE float) |
| master_volume_mode | uint8_t | 0/1 | 0 | 0xD4 | 0xD5 | 1 byte |
| saved master volume | float | -128.0 to 0.0 | 0.0 | 0xD6 (no payload, uses live) | 0xD7 | 4 bytes (LE float) |

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

### 3.3 REQ_SET_MASTER_VOLUME_MODE (0xD4)

| Field | Value |
|-------|-------|
| **Direction** | Host -> Device (OUT) |
| **bRequest** | `0xD4` |
| **wValue** | Unused (0) |
| **wIndex** | Unused (0) |
| **wLength** | 1 |
| **Payload** | 1 byte: `0x00` = independent (default), `0x01` = with preset |

**Firmware behavior:**
1. Reads byte 0 from the vendor receive buffer.
2. Clamps values outside `[0, 1]` to `0`.
3. Updates the `master_volume_mode` field in the directory cache.
4. Sets a pending flag for a deferred flash write on the next main loop iteration.

**Validation:** Minimum payload length = 1 byte. Shorter payloads are silently ignored.

**Example:** Switch to mode 1 (volume lives inside presets):
```
bRequest = 0xD4, wValue = 0x0000, wIndex = 0x0000, wLength = 1
Payload: [0x01]
```

### 3.4 REQ_GET_MASTER_VOLUME_MODE (0xD5)

| Field | Value |
|-------|-------|
| **Direction** | Device -> Host (IN) |
| **bRequest** | `0xD5` |
| **wValue** | Unused (0) |
| **wIndex** | Unused (0) |
| **wLength** | 1 |
| **Response** | 1 byte: `0x00` = independent, `0x01` = with preset |

**Example response** (default, mode 0):
```
[0x00]
```

### 3.5 REQ_SAVE_MASTER_VOLUME (0xD6)

| Field | Value |
|-------|-------|
| **Direction** | Device -> Host (IN) |
| **bRequest** | `0xD6` |
| **wValue** | Unused (0) |
| **wIndex** | Unused (0) |
| **wLength** | 1 |
| **Response** | 1 byte: status code (`PRESET_OK` = 0 on acceptance) |

Action-style command: copies the current live `master_volume_db` into the directory's independent storage field and flushes the directory sector to flash. This is the command that makes a chosen master volume survive a power cycle in mode 0. Follows the same request shape as `REQ_FACTORY_RESET` — an IN control transfer with a 1-byte status response, no OUT payload.

**Firmware behavior:**
1. Sets a pending flag for a deferred flash write on the next main loop iteration.
2. Returns `PRESET_OK` (0) immediately — the status confirms acceptance, not completion of the flash write. The subsequent main-loop dispatch captures the then-current live `master_volume_db` value and writes it to the directory.

**Accepted in both modes.** In mode 1 the stored value is dormant until the user switches to mode 0, so the app can issue this command regardless of mode without a pre-check. No error is surfaced for a "wrong mode" — the write simply becomes inert.

**Example:** Persist the currently active master volume:
```
bmRequestType = 0xC0, bRequest = 0xD6, wValue = 0x0000, wIndex = 0x0000, wLength = 1
Response: [0x00]   (PRESET_OK)
```

### 3.6 REQ_GET_SAVED_MASTER_VOLUME (0xD7)

| Field | Value |
|-------|-------|
| **Direction** | Device -> Host (IN) |
| **bRequest** | `0xD7` |
| **wValue** | Unused (0) |
| **wIndex** | Unused (0) |
| **wLength** | 4 |
| **Response** | 4 bytes: IEEE 754 float, little-endian |

Returns the value stored in the directory's independent master-volume field — i.e., the value the device would apply at boot in mode 0. Independent of the current live value and the current mode. Useful for showing "saved value" and "current value" side by side in the UI, or offering a "revert" button.

**Example response** (saved value is -12.0 dB):
```
[0x00, 0x00, 0x40, 0xC1]   (-12.0f)
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

**Bulk SET behavior:** The master volume from a bulk SET is always applied to the live `master_volume_db` global, regardless of `master_volume_mode`. Bulk SET is an in-memory state push — it does not touch the directory's independent storage field (that requires an explicit `REQ_SAVE_MASTER_VOLUME`). Mode 0 users who want a bulk-pushed value to survive a reboot must follow the bulk SET with a `REQ_SAVE_MASTER_VOLUME`.

### Bulk transfer commands

| Command | Code | Direction | Description |
|---------|------|-----------|-------------|
| `REQ_GET_ALL_PARAMS` | 0xA0 | Device -> Host | Returns full `WireBulkParams` (2896 bytes) |
| `REQ_SET_ALL_PARAMS` | 0xA1 | Host -> Device | Applies full `WireBulkParams` to live state |

---

## 5. Preset Persistence

Master volume has two persistence models selectable at runtime via `master_volume_mode`. In mode 0 (default) it lives in the preset directory sector, independent of presets. In mode 1 it lives in the individual preset slots, saved and restored with each preset.

### Flash storage version

- `PresetSlot.master_volume_db` was added at `SLOT_DATA_VERSION` 12. Still saved on every preset save (see below).
- `PresetDirectory.master_volume_db` — the independent storage — was added at directory `version = 2`. Directory v1 (pre-refactor) is migrated transparently on first boot of new firmware (see Section 7).

### PresetSlot fields

The following field is appended to the `PresetSlot` struct:

| Field | Type | Size | Description |
|-------|------|------|-------------|
| `master_volume_db` | float | 4 | -128.0 (mute) to 0.0 dB |

### PresetDirectory fields (relevant to master volume)

At `version = 2`:

| Field | Type | Size | Description |
|-------|------|------|-------------|
| `master_volume_mode` | uint8_t | 1 | 0 = independent, 1 = with preset |
| `master_volume_db` | float | 4 | Saved independent value (mode 0 source) |

### Save behavior (preset save)

When `REQ_PRESET_SAVE` (0x90) is issued, the current live master volume is **always** saved into the preset slot, regardless of mode:

```
slot->master_volume_db = master_volume_db;
```

This ensures the preset always contains a complete snapshot of the device state. In mode 0 the saved value is simply ignored on load; in mode 1 it's the source of truth on load.

### Save behavior (independent storage, mode 0)

When `REQ_SAVE_MASTER_VOLUME` (0xD6) is issued, the current live master volume is copied to `PresetDirectory.master_volume_db` and the directory sector is flushed to flash. This is the write that makes the chosen volume survive a power cycle in mode 0. Issuing this command in mode 1 is accepted but dormant (see Section 3.5).

### Load behavior (preset load)

When `REQ_PRESET_LOAD` (0x91) is issued, master volume is applied as follows:

- **Mode 1 AND slot version >= 12:** Master volume is restored from `slot->master_volume_db`.
- **Mode 1 AND slot version < 12:** Master volume falls back to `PresetDirectory.master_volume_db` (the slot predates master volume support, so there's nothing to restore).
- **Mode 0 (any slot version):** Master volume is **not** touched by the slot data. Live `master_volume_db` persists at whatever it currently is. (Factory-defaults paths explicitly re-apply `PresetDirectory.master_volume_db`; see below.)
- **Unconfigured (empty) slot:** Factory defaults are applied. The master-volume portion of factory defaults defers to the mode-aware helper, so mode 0 restores `PresetDirectory.master_volume_db` and mode 1 falls back to the same directory value (since there's no slot).

### Factory defaults

Applied on factory reset (`REQ_FACTORY_RESET` / 0x53) and on loading an empty or corrupt slot:

| Path | master_volume_db applied |
|------|--------------------------|
| Factory reset, mode 0 | `PresetDirectory.master_volume_db` (preserves user's saved independent value) |
| Factory reset, mode 1 | `PresetDirectory.master_volume_db` (fallback; defaults to 0.0 dB) |

The directory's `master_volume_mode` and `master_volume_db` fields are **not** reset by a factory reset — factory reset only touches live DSP state, not directory metadata.

Fresh-device directory defaults:

| Parameter | Default value |
|-----------|---------------|
| master_volume_mode | 0 (independent) |
| master_volume_db (directory) | 0.0 (unity) |

### Boot behavior

On device startup:
1. Live `master_volume_db` starts at 0.0 dB (unity) from the static initializer.
2. Directory is loaded. If directory is v1 (pre-refactor), it's migrated to v2 in-place; the old `include_master_volume` flag maps 1:1 to `master_volume_mode`, and `PresetDirectory.master_volume_db` defaults to 0.0 dB so boot-time audible behavior is unchanged for upgraded devices.
3. The preset system loads the startup slot (per the startup policy).
4. Master volume is applied based on mode:
   - **Mode 0:** `PresetDirectory.master_volume_db` is applied to live globals — the device comes up at the user's last-saved independent value.
   - **Mode 1 AND slot V12+:** `slot->master_volume_db` is applied.
   - **Mode 1 AND slot older:** `PresetDirectory.master_volume_db` is applied as fallback.

### Preset directory response

The `REQ_PRESET_GET_DIR` (0x95) response is still 7 bytes; byte [6] now carries `master_volume_mode` instead of `include_master_volume`. The numeric value space is unchanged (0 or 1):

| Byte | Field | Description |
|------|-------|-------------|
| 0-1 | `slot_occupied` | Little-endian uint16_t bitmask (bit N = slot N occupied) |
| 2 | `startup_mode` | 0 = specified default, 1 = last active |
| 3 | `default_slot` | Default startup slot index (0-9) |
| 4 | `last_active_slot` | Last active slot index (0-9) |
| 5 | `include_pins` | 0 = don't restore pins on load, 1 = restore |
| 6 | `master_volume_mode` | 0 = independent, 1 = with preset |

Apps requesting the old 6-byte size still work (USB truncates to `wLength`). Apps that want the mode byte must request `wLength >= 7`. The renamed field at byte [6] is numerically compatible with apps written for `include_master_volume`: value 1 still means "preset restores master volume" in both interpretations, and value 0 still means "preset does not restore" (with the new semantic that the device instead applies an independent stored value at boot).

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

Also read the mode and the saved independent value:

```
GET 0xD5 -> master_volume_mode          (1 byte; 0 = independent, 1 = with preset)
GET 0xD7 -> saved_master_volume_db      (4 bytes; float)
```

Or parse byte 6 of the `REQ_PRESET_GET_DIR` (0x95) response (request `wLength = 7`) for `master_volume_mode`.

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

**Master volume mode selector:**
- A two-state control in a settings/preferences area
- Option 0 (default): "Master volume is independent of presets"
- Option 1: "Master volume is part of each preset"
- Consider offering a "Save as boot default" button next to the volume slider. In mode 0 this button is the primary way to persist the current volume; in mode 1 it still works (writes to the independent storage) but is less meaningful until the user switches modes.
- A "Revert to saved" button can call `GET 0xD7` and then `SET 0xD2` to pull the stored value back into the live state.

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

// User toggles mode to "with preset"
uint8_t mode = 1;
usb_vendor_out(0xD4, &mode, 1);

// User clicks "Save as boot default" (mode 0 persistence).
// REQ_SAVE_MASTER_VOLUME is an IN-shaped action command like REQ_FACTORY_RESET:
// host issues a 1-byte IN transfer, device responds with status (0 = accepted).
uint8_t status;
usb_vendor_in(0xD6, &status, 1);

// User clicks "Revert to saved"
float saved;
usb_vendor_in(0xD7, &saved, 4);
usb_vendor_out(0xD2, &saved, 4);
```

**Important:** The firmware applies the new master volume value immediately (within the next audio callback, typically < 1ms). There is no pending/deferred mechanism for master volume itself -- unlike EQ coefficient updates, master volume is a single float that can be applied atomically. There is no acknowledgment; if the USB transfer completes successfully, the value was accepted.

### Step 4: Handle preset load events

When the user loads a preset (via `REQ_PRESET_LOAD` / 0x91), the master volume may or may not change depending on `master_volume_mode`. In mode 0 it is guaranteed not to change; in mode 1 it is replaced by the slot's saved value. After a preset load, re-read the master volume to update the UI:

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
4. Bulk SET always applies master volume to the live globals regardless of `master_volume_mode`. It does **not** update the independent storage field — if the user is in mode 0 and expects the pushed value to survive a reboot, follow the bulk SET with `REQ_SAVE_MASTER_VOLUME` (0xD6).

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

### Directory v1 → v2 migration

Devices running firmware built before the mode-flag refactor have a `PresetDirectory` with `version = 1` (containing `include_master_volume` as a `uint8_t` at the same byte offset now used by `master_volume_mode`). On first boot of new firmware:

1. `dir_load_cache` reads the directory's magic + version from the fixed 12-byte header.
2. If `version == 1`, the directory is read with a local `PresetDirectory_v1` struct, its v1-sized CRC is validated, and the field values are copied into the new v2 cache:
   - `master_volume_mode` = old `include_master_volume` (direct 1:1 map — `0` stays `0` meaning "independent" with default 0 dB, `1` stays `1` meaning "with preset")
   - `master_volume_db` = 0.0 dB (unity)
   - All other fields (slot names, startup mode, default slot, last active, include_pins, slot occupancy) copied verbatim
3. The migrated cache is flushed to flash as `version = 2`. The migration is transparent and one-shot.
4. If the v1 CRC fails to validate, the fresh-directory path runs — identical to a first-ever boot. User-visible directory state (slot names, startup config, slot occupancy) would be lost, but slot contents remain intact.

Audible behavior is unchanged across the upgrade: old `include_master_volume == 0` devices boot at 0 dB (unity) in both the old and new firmware; old `include_master_volume == 1` devices continue to restore master volume from each loaded preset.

### Old apps with new firmware

Apps that do not know about master volume will never send `REQ_SET_MASTER_VOLUME` (0xD2). Device-side boot behavior continues to be governed by `PresetDirectory.master_volume_db` (mode 0, unity by default) or the loaded preset (mode 1) — same as before on devices that were already in that state.

Apps written for the old `REQ_SET_INCLUDE_MASTER_VOL` / `REQ_GET_INCLUDE_MASTER_VOL` semantics (commands `0xD4` / `0xD5`) continue to work: the command IDs are unchanged, the payloads are still `uint8_t` in `[0, 1]`, and the numeric mapping is 1:1 (value `1` still means "preset restores master volume"; value `0` now means "use independent storage" with a default of 0 dB, which matches the pre-refactor "don't restore on load" boot behavior).

The new commands (`0xD6` `REQ_SAVE_MASTER_VOLUME`, `0xD7` `REQ_GET_SAVED_MASTER_VOLUME`) are additive — old apps that don't issue them are unaffected.

### New apps with old firmware

Apps that send `REQ_SET_MASTER_VOLUME` (0xD2) to firmware that does not support it will receive a USB STALL response. The same is true of the new `0xD6` / `0xD7` commands on firmware that predates them.

Detection strategy:
- **Master volume feature present?** Attempt `REQ_GET_MASTER_VOLUME` (0xD3). If success, feature is present.
- **Mode/save API present (vs. the old include-flag API)?** Attempt `REQ_GET_SAVED_MASTER_VOLUME` (0xD7). If it succeeds, the refactored API is in place and the app can use mode 0 independent persistence. If it STALLs, the firmware still speaks the old `include_master_volume` semantics at 0xD4/0xD5 — the app can fall back to treating the byte-6 field of `REQ_PRESET_GET_DIR` as a classic include flag.

### Old presets

Presets saved with `SLOT_DATA_VERSION` < 12 do not contain a `master_volume_db` field. When loaded:
- In mode 1 (per-preset): master volume falls back to `PresetDirectory.master_volume_db` (the independent storage value). Reasonable default — there's nothing to restore from the slot.
- In mode 0 (independent): master volume is not touched by the slot data anyway.

### Old bulk transfer payloads

Bulk SET payloads with `format_version` < 6 do not contain the `WireMasterVolume` section. When received, the master volume is set to 0.0 dB (unity) — unchanged from pre-refactor behavior.

### Preset directory size

The directory response is still 7 bytes. Apps requesting the old 6-byte size continue to work because USB control transfers truncate to `wLength`. Byte [6]'s meaning has been redefined (from `include_master_volume` to `master_volume_mode`), but the numeric range and 1:1 behavioral mapping make this transparent.

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
- 4 bytes for `master_volume_db` (float) — live global
- 4 bytes for `master_volume_linear` (precomputed float multiplier) — live global
- 4 bytes for `master_volume_q15` (precomputed Q15 multiplier, RP2040 path) — live global
- 1 byte for `master_volume_mode` in the directory cache
- 4 bytes for `master_volume_db` in the directory cache (independent storage)

### Sample rate handling

Master volume is a simple linear multiplier and is not affected by sample rate changes. No coefficient recomputation is needed when the sample rate changes.
