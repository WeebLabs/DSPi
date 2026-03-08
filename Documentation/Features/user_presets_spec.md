# User Presets Specification

## Overview

The DSPi firmware supports 10 user-configurable preset slots (0-9). Each preset captures the complete user-adjustable DSP state, allowing rapid switching between different audio configurations. Presets are stored in flash and persist across power cycles.

A preset is always active — there is no "no preset" state. Each slot can be either configured (has user data in flash) or unconfigured (loads factory defaults when selected). Slot 0 has the default name "Default".

A configurable startup policy determines which preset loads automatically on boot: either a fixed "default" slot or whichever slot was last active.

## Concepts

### Preset Slot

A numbered storage location (0-9) in flash that holds a complete DSP state snapshot. Slots can be unconfigured (never written or explicitly deleted — loads factory defaults when selected) or configured (contains valid user data).

### Active Slot

The slot that was most recently loaded or saved. Tracked in the preset directory as `last_active_slot`. Always a valid slot index (0-9) — a preset is always selected. On a fresh device, slot 0 is active by default.

### Preset Directory

A metadata sector in flash that stores:
- Which slots are occupied (bitmask)
- The name of each slot (32 bytes each)
- Startup configuration (mode + default slot)
- Which slot was last active
- Whether pin configuration is included in preset operations

### Startup Policy

Controls which preset loads on boot:

| Mode | Value | Behavior |
|------|-------|----------|
| Specified Default | 0 | Always loads the slot identified by `default_slot` |
| Last Active | 1 | Loads whichever slot was most recently loaded or saved |

The default mode is **Specified Default** with `default_slot = 0`.

If the target slot is empty or corrupt at boot, the device applies factory defaults while keeping the slot selected.

## DSP State Captured in a Preset

Each preset stores the following parameters:

| Category | Parameters | Notes |
|----------|-----------|-------|
| EQ | Per-channel filter recipes (type, freq, Q, gain) for all channels x 12 bands | Platform-dependent channel count |
| Preamp | Global preamp gain (dB) | |
| Bypass | Master EQ bypass flag | |
| Delays | Per-channel delay values (ms) | All channels including master |
| Channel Gain | Legacy per-channel gain (dB) for 3 channels | Backward compatibility |
| Channel Mute | Legacy per-channel mute for 3 channels | Backward compatibility |
| Loudness | Enabled flag, reference SPL, intensity percentage | |
| Crossfeed | Enabled, preset index, ITD enabled, custom frequency, custom feed level | |
| Matrix Mixer | All crosspoint routes (enabled, phase, gain) + all output states (enabled, mute, gain, delay) | |
| Pin Config | GPIO assignments for all outputs | Only restored if `include_pins` is set |

## Vendor Commands

All preset commands use USB EP0 control transfers on the vendor interface (interface 2). Commands use request codes `0x90`-`0x9A`.

### Command Summary

| Command | Code | Direction | wValue | wLength | Description |
|---------|------|-----------|--------|---------|-------------|
| `REQ_PRESET_SAVE` | `0x90` | IN | slot (0-9) | 1 | Save current live state to slot |
| `REQ_PRESET_LOAD` | `0x91` | IN | slot (0-9) | 1 | Load slot into live state |
| `REQ_PRESET_DELETE` | `0x92` | IN | slot (0-9) | 1 | Delete (erase) a preset slot |
| `REQ_PRESET_GET_NAME` | `0x93` | IN | slot (0-9) | 32 | Get slot name |
| `REQ_PRESET_SET_NAME` | `0x94` | OUT | slot (0-9) | 1-32 | Set slot name |
| `REQ_PRESET_GET_DIR` | `0x95` | IN | 0 | 6 | Get directory summary |
| `REQ_PRESET_SET_STARTUP` | `0x96` | OUT | 0 | 2 | Set startup configuration |
| `REQ_PRESET_GET_STARTUP` | `0x97` | IN | 0 | 3 | Get startup configuration |
| `REQ_PRESET_SET_INCLUDE_PINS` | `0x98` | OUT | 0 | 1 | Set pin-inclusion flag |
| `REQ_PRESET_GET_INCLUDE_PINS` | `0x99` | IN | 0 | 1 | Get pin-inclusion flag |
| `REQ_PRESET_GET_ACTIVE` | `0x9A` | IN | 0 | 1 | Get active slot index |

### Status Codes (returned by SAVE, LOAD, DELETE)

| Code | Name | Description |
|------|------|-------------|
| `0x00` | `PRESET_OK` | Operation succeeded |
| `0x01` | `PRESET_ERR_INVALID_SLOT` | Slot index >= 10 |
| `0x02` | `PRESET_ERR_SLOT_EMPTY` | *(reserved — load on empty slot now applies factory defaults)* |
| `0x03` | `PRESET_ERR_CRC` | Slot data failed integrity check |
| `0x04` | `PRESET_ERR_FLASH_WRITE` | Flash erase/program failed |

---

### REQ_PRESET_SAVE (0x90)

Save the current live DSP state into a preset slot.

**Transfer type:** Control IN (Device to Host)
**bmRequestType:** `0xC1` (Device-to-Host, Vendor, Interface)
**bRequest:** `0x90`
**wValue:** Slot index (0-9)
**wIndex:** Vendor interface number (2)
**wLength:** 1

**Response:** 1-byte status code.

**Behavior:**
1. Validates slot index (must be 0-9)
2. Snapshots all current live DSP parameters into a `PresetSlot` structure
3. Computes CRC32 over the data section
4. Erases and programs the flash sector for the given slot
5. Updates the preset directory: marks slot as occupied, sets `last_active_slot`
6. Returns `PRESET_OK` on success

**Notes:**
- Saving to an occupied slot overwrites it without confirmation
- The slot name is NOT affected by save (names are managed separately)
- If the slot previously had a name, the name is preserved
- Interrupts are briefly disabled during the flash write (~1-2 ms)

**Example (C/libusb):**
```c
uint8_t status;
int ret = libusb_control_transfer(handle,
    0xC1,       // bmRequestType: IN, Vendor, Interface
    0x90,       // bRequest: REQ_PRESET_SAVE
    slot,       // wValue: slot index (0-9)
    2,          // wIndex: vendor interface
    &status,    // data
    1,          // wLength
    1000);      // timeout ms
if (ret == 1 && status == 0x00) {
    // Save succeeded
}
```

---

### REQ_PRESET_LOAD (0x91)

Load a preset slot into the live DSP state.

**Transfer type:** Control IN (Device to Host)
**bmRequestType:** `0xC1`
**bRequest:** `0x91`
**wValue:** Slot index (0-9)
**wIndex:** 2
**wLength:** 1

**Response:** 1-byte status code.

**Behavior:**
1. Validates slot index
2. Engages audio mute (~5 ms / 256 samples at 48 kHz) to prevent clicks
3. If the slot is occupied: reads and validates the slot data from flash (CRC check), applies user data to live state. If `include_pins` is set, restores pin configuration (with validation)
4. If the slot is empty (unconfigured): applies factory defaults to live state
5. Recalculates all filter coefficients for the current sample rate
6. Updates delay sample counts
7. Triggers loudness and crossfeed recomputation
8. Updates `last_active_slot` in the directory
9. Audio resumes after the mute period

**Notes:**
- The mute is transparent to the host application
- Filter states (biquad/SVF accumulators) are reset as part of recalculation
- If the device is playing audio, there will be a brief (~5 ms) silence during the switch
- The host application should read back the new parameter values after load if it needs to update its UI

**Example (C/libusb):**
```c
uint8_t status;
libusb_control_transfer(handle, 0xC1, 0x91, slot, 2, &status, 1, 1000);
if (status == 0x00) {
    // Load succeeded - refresh UI by reading back parameters
}
```

---

### REQ_PRESET_DELETE (0x92)

Delete a preset slot, erasing its flash sector and marking it as unoccupied.

**Transfer type:** Control IN
**bmRequestType:** `0xC1`
**bRequest:** `0x92`
**wValue:** Slot index (0-9)
**wIndex:** 2
**wLength:** 1

**Response:** 1-byte status code.

**Behavior:**
1. Erases the flash sector for the given slot
2. Clears the occupied bit in the directory
3. The active slot selection is unchanged — if the deleted slot was active, it remains selected (loading it will yield factory defaults)
4. The current live DSP state is NOT affected (the device continues running with whatever parameters are currently active)

**Notes:**
- Deleting a slot does not affect its name in the directory. The name persists and can be reused if the slot is saved again. To clear the name, use `REQ_PRESET_SET_NAME` with an empty string.
- It is safe to delete the active slot; the device continues with the live state. The slot remains selected but is now unconfigured.
- Deleting a slot that is already empty succeeds silently (no error).

---

### REQ_PRESET_GET_NAME (0x93)

Get the 32-byte name of a preset slot.

**Transfer type:** Control IN
**bmRequestType:** `0xC1`
**bRequest:** `0x93`
**wValue:** Slot index (0-9)
**wIndex:** 2
**wLength:** 32

**Response:** 32 bytes, NUL-terminated ASCII string.

**Notes:**
- Empty/unnamed slots return 32 zero bytes
- Names can be set before the slot is occupied (i.e., you can name a slot before saving to it)
- Maximum name length is 31 characters (the 32nd byte is always NUL)
- Returns `false` (STALL) if slot index is invalid

**Example (C/libusb):**
```c
char name[32];
int ret = libusb_control_transfer(handle, 0xC1, 0x93, slot, 2,
                                   (uint8_t*)name, 32, 1000);
if (ret == 32) {
    printf("Slot %d: %s\n", slot, name);
}
```

---

### REQ_PRESET_SET_NAME (0x94)

Set the name of a preset slot.

**Transfer type:** Control OUT (Host to Device)
**bmRequestType:** `0x41` (Host-to-Device, Vendor, Interface)
**bRequest:** `0x94`
**wValue:** Slot index (0-9)
**wIndex:** 2
**wLength:** 1-32

**Payload:** Up to 32 bytes of ASCII name data. The firmware copies up to 31 characters and ensures NUL termination.

**Behavior:**
1. Copies the name into the directory cache
2. Writes the updated directory to flash
3. The slot does not need to be occupied

**Notes:**
- To clear a name, send a single NUL byte (or any empty payload)
- Names are stored in the directory sector, not in the preset slot itself
- Setting a name triggers a directory flash write

**Example (C/libusb):**
```c
const char *name = "Living Room EQ";
libusb_control_transfer(handle,
    0x41,                      // bmRequestType: OUT, Vendor, Interface
    0x94,                      // bRequest: REQ_PRESET_SET_NAME
    slot,                      // wValue: slot index
    2,                         // wIndex: vendor interface
    (uint8_t*)name,            // data
    strlen(name) + 1,          // wLength: include NUL terminator
    1000);
```

---

### REQ_PRESET_GET_DIR (0x95)

Get a summary of the entire preset directory.

**Transfer type:** Control IN
**bmRequestType:** `0xC1`
**bRequest:** `0x95`
**wValue:** 0
**wIndex:** 2
**wLength:** 6

**Response:** 6 bytes:

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 2 | `slot_occupied` | Little-endian uint16 bitmask (bit 0 = slot 0, ..., bit 9 = slot 9) |
| 2 | 1 | `startup_mode` | 0 = specified default, 1 = last active |
| 3 | 1 | `default_slot` | Slot loaded in "specified default" mode (0-9) |
| 4 | 1 | `last_active` | Last loaded/saved slot (always 0-9) |
| 5 | 1 | `include_pins` | Whether preset load restores pin config (0 or 1) |

**Notes:**
- This is the most efficient way to query the overall state of the preset system
- Use this on application startup to populate the preset list UI
- Follow up with `REQ_PRESET_GET_NAME` for each occupied slot to get names

**Example (C/libusb):**
```c
uint8_t dir[6];
libusb_control_transfer(handle, 0xC1, 0x95, 0, 2, dir, 6, 1000);

uint16_t occupied = dir[0] | (dir[1] << 8);
for (int i = 0; i < 10; i++) {
    if (occupied & (1 << i)) {
        printf("Slot %d: occupied\n", i);
    }
}
printf("Startup mode: %s\n", dir[2] ? "last active" : "specified default");
printf("Default slot: %d\n", dir[3]);
printf("Last active: %d\n", dir[4]);
printf("Include pins: %s\n", dir[5] ? "yes" : "no");
```

---

### REQ_PRESET_SET_STARTUP (0x96)

Configure the startup behavior.

**Transfer type:** Control OUT
**bmRequestType:** `0x41`
**bRequest:** `0x96`
**wValue:** 0
**wIndex:** 2
**wLength:** 2

**Payload:** 2 bytes:

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | `startup_mode` | 0 = specified default, 1 = last active |
| 1 | 1 | `default_slot` | Slot to load in "specified default" mode (0-9) |

**Behavior:**
1. Validates both values
2. Updates the directory and writes to flash
3. Returns status code (via empty control IN completion)

**Notes:**
- The `default_slot` value is always stored, even when `startup_mode` is "last active" (it serves as a fallback)
- Setting `startup_mode = 0` and `default_slot = 3` means: "always boot with slot 3"
- Setting `startup_mode = 1` means: "boot with whatever was last used"
- **Default behavior** on a fresh device: `startup_mode = 0`, `default_slot = 0` (load slot 0 on boot)

---

### REQ_PRESET_GET_STARTUP (0x97)

Get the current startup configuration.

**Transfer type:** Control IN
**bmRequestType:** `0xC1`
**bRequest:** `0x97`
**wValue:** 0
**wIndex:** 2
**wLength:** 3

**Response:** 3 bytes:

| Offset | Size | Field |
|--------|------|-------|
| 0 | 1 | `startup_mode` |
| 1 | 1 | `default_slot` |
| 2 | 1 | `last_active` |

---

### REQ_PRESET_SET_INCLUDE_PINS (0x98)

Set whether preset load/save operations include GPIO pin configuration.

**Transfer type:** Control OUT
**bmRequestType:** `0x41`
**bRequest:** `0x98`
**wValue:** 0
**wIndex:** 2
**wLength:** 1

**Payload:** 1 byte: `0` = exclude pins (default), `1` = include pins.

**Notes:**
- When disabled (default), pin configuration is a device-level setting independent of presets
- Pin data is always stored in the preset for completeness, but only restored on load when this flag is set
- Enabling this is useful when the same device is used with different physical output configurations
- The flag is persisted in the directory (survives power cycles)

---

### REQ_PRESET_GET_INCLUDE_PINS (0x99)

Get the current pin-inclusion flag.

**Transfer type:** Control IN
**bmRequestType:** `0xC1`
**bRequest:** `0x99`
**wValue:** 0
**wIndex:** 2
**wLength:** 1

**Response:** 1 byte: `0` or `1`.

---

### REQ_PRESET_GET_ACTIVE (0x9A)

Get the index of the currently active preset slot.

**Transfer type:** Control IN
**bmRequestType:** `0xC1`
**bRequest:** `0x9A`
**wValue:** 0
**wIndex:** 2
**wLength:** 1

**Response:** 1 byte: slot index (always 0-9).

**Notes:**
- A preset is always active — this never returns `0xFF`
- The active slot is updated whenever a preset is saved or loaded
- If the active slot is deleted, it remains selected (the slot is now unconfigured and will load factory defaults)

---

## Legacy Command Compatibility

The existing flash storage commands continue to work and redirect through the preset system:

| Legacy Command | Code | New Behavior |
|---------------|------|--------------|
| `REQ_SAVE_PARAMS` | `0x51` | Saves to the active preset slot. |
| `REQ_LOAD_PARAMS` | `0x52` | Reloads the active preset slot from flash (factory defaults if unconfigured). |
| `REQ_FACTORY_RESET` | `0x53` | Resets live state to factory defaults. Active slot unchanged. Does NOT erase any preset slots. |

## Firmware Migration

When the firmware is upgraded from a pre-preset version:

1. On first boot, `preset_boot_load()` finds no preset directory
2. It checks the legacy flash sector (last 4 KB) for the old `0x44535031` ("DSP1") magic
3. If found and CRC-valid, the legacy data is copied into preset slot 0
4. A new preset directory is created with:
   - Slot 0 marked as occupied, named "Migrated"
   - `startup_mode = 0` (specified default), `default_slot = 0`
   - `last_active_slot = 0`
5. The device boots with the migrated settings

This migration is transparent to the user. The device operates identically to before, but now supports the full preset system.

## Application Integration Patterns

### Initialization Flow

When a host application connects to the device:

```
1. REQ_PRESET_GET_DIR          -> Get occupied bitmask + startup config
2. For each occupied slot:
   REQ_PRESET_GET_NAME         -> Get the slot's name
3. REQ_PRESET_GET_ACTIVE       -> Determine which slot is currently loaded
4. Display the preset list with the active slot highlighted
```

### Saving the Current Configuration

```
1. (Optional) REQ_PRESET_SET_NAME  -> Set a descriptive name for the target slot
2. REQ_PRESET_SAVE                 -> Save live state to the slot
3. Update UI to show the slot as occupied
```

### Loading a Preset

```
1. REQ_PRESET_LOAD                 -> Load the slot (firmware handles mute)
2. Wait ~10 ms for the mute period to complete
3. Read back all parameter values to update UI:
   - REQ_GET_PREAMP
   - REQ_GET_BYPASS
   - REQ_GET_EQ_PARAM (for each channel/band)
   - REQ_GET_LOUDNESS, REQ_GET_LOUDNESS_REF, REQ_GET_LOUDNESS_INTENSITY
   - REQ_GET_CROSSFEED, REQ_GET_CROSSFEED_PRESET, etc.
   - REQ_GET_MATRIX_ROUTE (for each crosspoint)
   - REQ_GET_OUTPUT_ENABLE/GAIN/MUTE/DELAY (for each output)
```

### Managing Startup Behavior

```
// Set a specific default preset
REQ_PRESET_SET_STARTUP with [mode=0, default_slot=3]
// The device will always load slot 3 on boot

// Use "last active" mode
REQ_PRESET_SET_STARTUP with [mode=1, default_slot=0]
// The device will load whatever was last used
// default_slot=0 serves as fallback if last active is deleted
```

### Copying a Preset

There is no dedicated copy command. To copy preset A to preset B:

```
1. REQ_PRESET_LOAD slot=A        -> Load slot A into live state
2. REQ_PRESET_SAVE slot=B        -> Save live state to slot B
3. REQ_PRESET_SET_NAME slot=B    -> Optionally set a new name for the copy
4. REQ_PRESET_LOAD slot=A        -> Load slot A again to restore it as active
```

### Renaming a Preset

```
REQ_PRESET_SET_NAME with slot index and new name
// No need to load or save the preset; names are stored in the directory
```

### Deleting a Preset

```
1. Confirm with user (application responsibility)
2. REQ_PRESET_DELETE slot=N
3. (Optional) REQ_PRESET_SET_NAME slot=N with empty string to clear the name
4. Update UI to show the slot as empty
```

## Edge Cases and Behavior

| Scenario | Behavior |
|----------|----------|
| Save to occupied slot | Overwrites without confirmation |
| Load empty (unconfigured) slot | Applies factory defaults, sets slot as active |
| Load corrupt slot | Returns `PRESET_ERR_CRC` (0x03), live state unchanged |
| Delete active slot | Slot erased, active slot unchanged (now unconfigured), live state unchanged |
| Delete empty slot | Succeeds silently (erase + clear bit is idempotent) |
| Boot with specified default slot empty | Applies factory defaults, slot remains selected |
| Boot with last active slot empty | Applies factory defaults, slot remains selected |
| Boot with corrupt directory | Attempts legacy migration, then factory defaults with slot 0 |
| Set name on empty slot | Succeeds (name stored in directory, independent of slot data) |
| Factory reset | Live state reset to defaults, all presets remain intact, active slot unchanged |
| Pin config include_pins=0 (default) | Pin data stored in preset but NOT applied on load |
| Pin config include_pins=1 | Pin data validated and applied on load |
| Flash write fails | Returns `PRESET_ERR_FLASH_WRITE` (0x04), live state unchanged |
| Audio playing during preset load | Brief mute (~5 ms), then new preset takes effect |

## Flash Details

### Sector Layout

12 sectors (48 KB) reserved at the end of flash:

| Sector | Flash Offset (RP2040, 2MB) | Flash Offset (RP2350, 4MB) | Content |
|--------|----------------------------|----------------------------|---------|
| 0 | `0x1F4000` | `0x3F4000` | Preset Directory |
| 1 | `0x1F5000` | `0x3F5000` | Preset Slot 0 |
| 2 | `0x1F6000` | `0x3F6000` | Preset Slot 1 |
| 3 | `0x1F7000` | `0x3F7000` | Preset Slot 2 |
| 4 | `0x1F8000` | `0x3F8000` | Preset Slot 3 |
| 5 | `0x1F9000` | `0x3F9000` | Preset Slot 4 |
| 6 | `0x1FA000` | `0x3FA000` | Preset Slot 5 |
| 7 | `0x1FB000` | `0x3FB000` | Preset Slot 6 |
| 8 | `0x1FC000` | `0x3FC000` | Preset Slot 7 |
| 9 | `0x1FD000` | `0x3FD000` | Preset Slot 8 |
| 10 | `0x1FE000` | `0x3FE000` | Preset Slot 9 |
| 11 | `0x1FF000` | `0x3FF000` | Legacy sector (migration source) |

### Data Integrity

- Each slot and the directory have a CRC32 field (polynomial `0xEDB88320`)
- CRC covers the data section (everything after the 12-byte header: magic + version + reserved/slot_index + CRC)
- Magic numbers: Directory = `0x44535032` ("DSP2"), Slot = `0x44535033` ("DSP3"), Legacy = `0x44535031` ("DSP1")
- Slots include a `slot_index` field that is cross-checked against the expected position to detect misplaced data

### Flash Write Safety

- Interrupts are disabled during flash erase/program (typically ~1-2 ms)
- Audio output may briefly stall during this period (handled by the SPDIF buffer pool)
- A verification read-back after write checks that the magic number survived
- The directory is cached in RAM to minimize flash reads during normal operation

## RAM Impact

| Component | Size | Notes |
|-----------|------|-------|
| `dir_cache` (PresetDirectory) | ~340 bytes | Cached in BSS, loaded once at boot |
| `slot_buf` (PresetSlot, static) | ~1.6 KB (RP2040) / ~2.5 KB (RP2350) | Reused for each save operation |
| `write_buf` (sector scratch) | 4 KB | Page-aligned, used for flash writes |
| `preset_loading` + `preset_mute_counter` | 5 bytes | Mute-on-load control |
| **Total BSS increase** | **~6 KB (RP2040) / ~7 KB (RP2350)** | |
