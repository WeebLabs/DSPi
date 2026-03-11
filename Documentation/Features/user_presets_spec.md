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

### Channel Names

Per-channel user-configurable names stored in the live `channel_names[NUM_CHANNELS][PRESET_NAME_LEN]` global array. Each name is a 32-byte NUL-terminated ASCII string (max 31 visible characters).

**Storage lifecycle:**
- **Live state:** The `channel_names` array in `usb_audio.c` holds the current names. Individual names can be read/written via vendor commands `REQ_GET_CHANNEL_NAME` (0x9C) / `REQ_SET_CHANNEL_NAME` (0x9B). Changes are immediate in the live array but are NOT automatically persisted to flash.
- **Preset persistence:** When a preset is saved (`REQ_PRESET_SAVE`), the current `channel_names` array is copied into the `PresetSlot.channel_names` field. When loaded, names are restored from the slot. Channel names are always included in preset save/load — there is no opt-out flag (unlike pin config).
- **Bulk transfer:** Channel names are included in the `WireChannelNames` section of `WireBulkParams`, both for GET (0xA0) and SET (0xA1). The host receives and can modify channel names as part of the full state transfer.
- **Factory defaults:** When factory defaults are applied (empty slot load, factory reset, or first boot), all channels receive their default names via `get_default_channel_name()`.

**Default names by channel index:**

| Index | RP2040 | RP2350 |
|-------|--------|--------|
| 0 | USB L | USB L |
| 1 | USB R | USB R |
| 2 | SPDIF 1 L | SPDIF 1 L |
| 3 | SPDIF 1 R | SPDIF 1 R |
| 4 | SPDIF 2 L | SPDIF 2 L |
| 5 | SPDIF 2 R | SPDIF 2 R |
| 6 | PDM | SPDIF 3 L |
| 7 | — | SPDIF 3 R |
| 8 | — | SPDIF 4 L |
| 9 | — | SPDIF 4 R |
| 10 | — | PDM |

**Backward compatibility:** Preset slots saved with `SLOT_DATA_VERSION` < 8 do not contain channel names. When such a slot is loaded, all channels are assigned their default names.

### Startup Policy

Controls which preset loads on boot:

| Mode | Value | Behavior |
|------|-------|----------|
| Specified Default | 0 | Always loads the slot identified by `default_slot` |
| Last Active | 1 | Loads whichever slot was most recently loaded or saved |

The default mode is **Specified Default** with `default_slot = 0`.

If the target slot is empty or corrupt at boot, the device applies factory defaults while keeping the slot selected.

### Slot Names vs Channel Names

These are two distinct naming systems:

| | Slot Names (Preset Names) | Channel Names |
|---|---|---|
| **What they name** | Preset slots (0-9) | Audio channels (0 to NUM_CHANNELS-1) |
| **Where stored** | Preset Directory sector (flash metadata) | PresetSlot data (flash slot data) |
| **SET command** | `REQ_PRESET_SET_NAME` (0x94), wValue=slot | `REQ_SET_CHANNEL_NAME` (0x9B), wValue=channel |
| **GET command** | `REQ_PRESET_GET_NAME` (0x93), wValue=slot | `REQ_GET_CHANNEL_NAME` (0x9C), wValue=channel |
| **Persistence** | Immediate flash write on SET | Live in RAM only; persisted via `REQ_PRESET_SAVE` |
| **Per-preset?** | No — slot names are global | Yes — each preset stores its own channel names |
| **Affected by load** | No | Yes — names change when a preset is loaded |
| **Affected by save** | No | Yes — current names are captured in the saved slot |
| **In bulk transfer** | No | Yes — `WireChannelNames` section in `WireBulkParams` |
| **Max length** | 31 chars + NUL (32 bytes) | 31 chars + NUL (32 bytes) |

### Platform Constants

| Constant | RP2040 | RP2350 | Notes |
|----------|--------|--------|-------|
| `NUM_CHANNELS` | 7 | 11 | Total channels (2 input + outputs) |
| `NUM_OUTPUT_CHANNELS` | 5 | 9 | Output channels only |
| `NUM_INPUT_CHANNELS` | 2 | 2 | USB L/R |
| `NUM_SPDIF_INSTANCES` | 2 | 4 | S/PDIF stereo pairs |
| `NUM_PIN_OUTPUTS` | 3 | 5 | Configurable GPIO outputs (SPDIF + PDM) |
| `MAX_BANDS` | 12 | 12 | EQ bands per channel |
| `PRESET_NAME_LEN` | 32 | 32 | Bytes per name (slot or channel) |
| `PRESET_SLOTS` | 10 | 10 | Number of preset slots |
| `PLATFORM_ID` | 0 | 1 | Used in `REQ_GET_PLATFORM` and bulk params header |

**Determining platform at runtime:** Use `REQ_GET_PLATFORM` (0x7F) which returns a single byte (0=RP2040, 1=RP2350), or read `header.platform_id` / `header.num_channels` from a `REQ_GET_ALL_PARAMS` response. The valid channel index range for `REQ_SET_CHANNEL_NAME` / `REQ_GET_CHANNEL_NAME` is 0 to `header.num_channels - 1`.

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
| Channel Names | Per-channel 32-byte NUL-terminated names | Default names for slots with version < 8 |

## Vendor Commands

All preset commands use USB EP0 control transfers on the vendor interface (interface 2). Preset management commands use request codes `0x90`-`0x9C`. Bulk parameter transfer commands (`0xA0`-`0xA1`) are also documented here as they are closely related to the preset workflow.

### Command Summary

| Command | Code | Direction | wValue | wLength | Description |
|---------|------|-----------|--------|---------|-------------|
| `REQ_PRESET_SAVE` | `0x90` | IN | slot (0-9) | 1 | Save current live state to slot |
| `REQ_PRESET_LOAD` | `0x91` | IN | slot (0-9) | 1 | Load slot into live state |
| `REQ_PRESET_DELETE` | `0x92` | IN | slot (0-9) | 1 | Delete (erase) a preset slot |
| `REQ_PRESET_GET_NAME` | `0x93` | IN | slot (0-9) | 32 | Get slot name |
| `REQ_PRESET_SET_NAME` | `0x94` | OUT | slot (0-9) | 32 | Set slot name (fixed-size) |
| `REQ_PRESET_GET_DIR` | `0x95` | IN | 0 | 6 | Get directory summary |
| `REQ_PRESET_SET_STARTUP` | `0x96` | OUT | 0 | 2 | Set startup configuration |
| `REQ_PRESET_GET_STARTUP` | `0x97` | IN | 0 | 3 | Get startup configuration |
| `REQ_PRESET_SET_INCLUDE_PINS` | `0x98` | OUT | 0 | 1 | Set pin-inclusion flag |
| `REQ_PRESET_GET_INCLUDE_PINS` | `0x99` | IN | 0 | 1 | Get pin-inclusion flag |
| `REQ_PRESET_GET_ACTIVE` | `0x9A` | IN | 0 | 1 | Get active slot index |
| `REQ_SET_CHANNEL_NAME` | `0x9B` | OUT | channel index | 32 | Set channel name (fixed-size) |
| `REQ_GET_CHANNEL_NAME` | `0x9C` | IN | channel index | 32 | Get channel name |
| `REQ_GET_ALL_PARAMS` | `0xA0` | IN | 0 | 2832 | Get complete DSP state (multi-packet) |
| `REQ_SET_ALL_PARAMS` | `0xA1` | OUT | 0 | 2832 | Set complete DSP state (multi-packet) |

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
**wLength:** 32

**Payload:** Exactly 32 bytes — a NUL-terminated ASCII string zero-padded to `PRESET_NAME_LEN` (32 bytes). The firmware copies up to 31 characters and ensures NUL termination.

> **Important:** The host MUST always send exactly 32 bytes with `wLength = 32`. Short transfers cause USB disconnect and device reset on some host controllers. Zero-pad the buffer after the NUL terminator.

**Behavior:**
1. Copies the name into the directory cache
2. Writes the updated directory to flash
3. The slot does not need to be occupied

**Notes:**
- To clear a name, send a 32-byte buffer containing a single NUL byte followed by 31 zero bytes
- Names are stored in the directory sector, not in the preset slot itself
- Setting a name triggers a directory flash write

**Example (C/libusb):**
```c
char buf[32] = {0};
strncpy(buf, "Living Room EQ", 31);
libusb_control_transfer(handle,
    0x41,                      // bmRequestType: OUT, Vendor, Interface
    0x94,                      // bRequest: REQ_PRESET_SET_NAME
    slot,                      // wValue: slot index
    2,                         // wIndex: vendor interface
    (uint8_t*)buf,             // data: 32-byte zero-padded name
    32,                        // wLength: MUST be 32 (PRESET_NAME_LEN)
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
- This flag also gates pin application during `REQ_SET_ALL_PARAMS` (0xA1) bulk parameter transfers
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

### REQ_SET_CHANNEL_NAME (0x9B)

Set the user-configurable name of a channel.

**Transfer type:** Control OUT (Host to Device)
**bmRequestType:** `0x41` (Host-to-Device, Vendor, Interface)
**bRequest:** `0x9B`
**wValue:** Channel index (0 to NUM_CHANNELS-1)
**wIndex:** 2
**wLength:** 32

**Payload:** Exactly 32 bytes — a NUL-terminated ASCII string zero-padded to the full `PRESET_NAME_LEN` (32 bytes). The firmware copies up to 31 characters and ensures NUL termination.

> **Important:** The host MUST always send exactly 32 bytes with `wLength = 32`. Short transfers (e.g., sending only `strlen(name) + 1` bytes) cause USB disconnect and device reset on some host controllers (particularly certain xHCI implementations). Zero-pad the buffer after the NUL terminator. This matches the fixed-size convention used by `REQ_PRESET_SET_NAME` and the `WireChannelNames` section in bulk transfers.

**Behavior:**
1. Validates the channel index
2. Clears the existing name and copies the new name into `channel_names[ch]`
3. The name is stored in the live global array (not directly in flash)
4. To persist, save the current state to a preset slot with `REQ_PRESET_SAVE`

**Default names:**
- 0: "USB L", 1: "USB R"
- 2-3: "SPDIF 1 L/R", 4-5: "SPDIF 2 L/R"
- RP2350: 6-7: "SPDIF 3 L/R", 8-9: "SPDIF 4 L/R", 10: "PDM"
- RP2040: 6: "PDM"

**Example (C/libusb):**
```c
// Always send a fixed 32-byte zero-padded buffer
char buf[32] = {0};
strncpy(buf, "Front Left", 31);
libusb_control_transfer(handle,
    0x41,                      // bmRequestType: OUT, Vendor, Interface
    0x9B,                      // bRequest: REQ_SET_CHANNEL_NAME
    channel,                   // wValue: channel index
    2,                         // wIndex: vendor interface
    (uint8_t*)buf,             // data: 32-byte zero-padded name
    32,                        // wLength: MUST be 32 (PRESET_NAME_LEN)
    1000);
```

---

### REQ_GET_CHANNEL_NAME (0x9C)

Get the 32-byte name of a channel.

**Transfer type:** Control IN
**bmRequestType:** `0xC1`
**bRequest:** `0x9C`
**wValue:** Channel index (0 to NUM_CHANNELS-1)
**wIndex:** 2
**wLength:** 32

**Response:** 32 bytes, NUL-terminated ASCII string.

**Example (C/libusb):**
```c
char name[32];
int ret = libusb_control_transfer(handle, 0xC1, 0x9C, channel, 2,
                                   (uint8_t*)name, 32, 1000);
if (ret == 32) {
    printf("Channel %d: %s\n", channel, name);
}
```

---

### REQ_GET_ALL_PARAMS (0xA0)

Read the complete live DSP state in a single transfer (~2832 bytes). This is a multi-packet USB control transfer on EP0 (NOT a bulk endpoint transfer).

**Transfer type:** Control IN (multi-packet, ~45 packets of 64 bytes)
**bmRequestType:** `0xC1` (Device-to-Host, Vendor, Interface)
**bRequest:** `0xA0`
**wValue:** 0
**wIndex:** 2
**wLength:** `sizeof(WireBulkParams)` (2832)

**Response:** `WireBulkParams` structure (defined in `bulk_params.h`):

| Offset | Size | Section | Content |
|--------|------|---------|---------|
| 0 | 16 | Header | Format version (2), platform ID, channel counts, payload length, FW version |
| 16 | 16 | Global | Preamp gain, bypass, loudness enabled/ref SPL/intensity |
| 32 | 16 | Crossfeed | Enabled, preset, ITD, custom freq/feed level |
| 48 | 16 | Legacy channels | Per-channel gain (3) and mute (3) |
| 64 | 44 | Delays | Per-channel delay in ms (up to 11 channels, zero-padded) |
| 108 | 144 | Matrix crosspoints | 2 inputs × 9 outputs × 8 bytes (enabled, phase, gain) |
| 252 | 108 | Matrix outputs | 9 outputs × 12 bytes (enabled, mute, gain, delay) |
| 360 | 8 | Pin config | Number of pins + GPIO assignments (always included) |
| 368 | 2112 | EQ bands | 11 channels × 12 bands × 16 bytes (type, freq, Q, gain) |
| 2480 | 352 | Channel names | 11 channels × 32 bytes (NUL-terminated ASCII) |

**Notes:**
- All multi-byte fields are little-endian. All float fields are IEEE 754 single-precision at 4-byte-aligned offsets.
- Arrays are sized at platform maximums (RP2350: 11 channels, 9 outputs). Entries beyond the platform's actual count are zero-padded. Use `header.num_channels` and `header.num_output_channels` to determine valid entries.
- Pin configuration is always included in GET regardless of the `include_pins` setting.
- This is the preferred method for reading back parameters after a preset load or on app startup.

**Example (C/libusb):**
```c
uint8_t buf[2832];
int ret = libusb_control_transfer(handle,
    0xC1,       // bmRequestType: IN, Vendor, Interface
    0xA0,       // bRequest: REQ_GET_ALL_PARAMS
    0,          // wValue
    2,          // wIndex: vendor interface
    buf,        // data
    2832,       // wLength
    5000);      // timeout ms (longer for multi-packet)
if (ret == 2832) {
    WireBulkParams *params = (WireBulkParams *)buf;
    // Parse params->header, params->global, params->eq, params->channel_names, etc.
}
```

---

### REQ_SET_ALL_PARAMS (0xA1)

Apply a complete DSP state in a single transfer (~2832 bytes). This is a multi-packet USB control transfer on EP0 (NOT a bulk endpoint transfer).

**Transfer type:** Control OUT (multi-packet, ~45 packets of 64 bytes)
**bmRequestType:** `0x41` (Host-to-Device, Vendor, Interface)
**bRequest:** `0xA1`
**wValue:** 0
**wIndex:** 2
**wLength:** `sizeof(WireBulkParams)` (2832, must match exactly)

**Payload:** `WireBulkParams` structure (same layout as GET response above).

**Behavior:**
1. Firmware receives all packets into a 4 KB buffer via `usb_stream_transfer`
2. After the status phase completes, `bulk_params_pending` is set
3. Main loop processes the request (deferred from USB IRQ):
   a. Waits for Core 1 EQ worker to finish current work (if active)
   b. Engages audio mute (~5 ms / 256 samples at 48 kHz)
   c. Validates the header: `format_version`, `platform_id`, `num_channels`, `payload_length`
   d. Applies all parameters to live state
   e. If `include_pins` is set in the preset directory, applies pin configuration (with validation)
   f. Recalculates all filter coefficients for the current sample rate
   g. Updates delay sample counts
   h. Triggers loudness and crossfeed recomputation

**Validation errors:** If the header fails validation (wrong platform, wrong channel count, etc.), the parameters are silently not applied. There is no error response — the transfer itself succeeds at the USB level.

**Notes:**
- `wLength` must exactly equal `sizeof(WireBulkParams)` (2832 bytes); other values are rejected (STALL)
- The `platform_id` field must match the device's platform (RP2040=0, RP2350=1)
- The `num_channels` and `num_output_channels` fields must match the device's configuration
- Pin application is gated by the preset directory's `include_pins` flag (same as preset load)
- This command does NOT save to flash. To persist, follow with `REQ_PRESET_SAVE` or `REQ_SAVE_PARAMS`.
- This command does NOT change the active preset slot

**Example (C/libusb):**
```c
// Assume 'params' is a previously captured WireBulkParams
int ret = libusb_control_transfer(handle,
    0x41,                       // bmRequestType: OUT, Vendor, Interface
    0xA1,                       // bRequest: REQ_SET_ALL_PARAMS
    0,                          // wValue
    2,                          // wIndex: vendor interface
    (uint8_t*)&params,          // data
    sizeof(WireBulkParams),     // wLength: must be exactly 2832
    5000);                      // timeout ms
if (ret == sizeof(WireBulkParams)) {
    // Transfer accepted; parameters will be applied within ~5 ms
}
```

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

### Wire Format Version 2

`WIRE_FORMAT_VERSION` was bumped from 1 to 2 to add the `WireChannelNames` section to `WireBulkParams`. This is a **breaking change** for the bulk parameter commands:

- `sizeof(WireBulkParams)` changed from 2480 to 2832 bytes
- `REQ_GET_ALL_PARAMS` (0xA0): host must request 2832 bytes (was 2480)
- `REQ_SET_ALL_PARAMS` (0xA1): host must send exactly 2832 bytes (was 2480); the firmware rejects `wLength != sizeof(WireBulkParams)` with a STALL
- The firmware's `bulk_params_apply()` rejects payloads where `format_version != 2` or `payload_length != 2832`

Host applications built against format version 1 must be updated to include the channel names section. The simplest upgrade path: allocate a 2832-byte buffer, zero-initialize it, populate the existing fields as before, and leave the channel names section zeroed (the firmware will apply empty strings, which the host can then overwrite with `REQ_SET_CHANNEL_NAME` or by loading a preset).

## Application Integration Patterns

### Initialization Flow

When a host application connects to the device:

```
1. REQ_PRESET_GET_DIR          -> Get occupied bitmask + startup config
2. For each occupied slot:
   REQ_PRESET_GET_NAME         -> Get the slot's name
3. REQ_PRESET_GET_ACTIVE       -> Determine which slot is currently loaded
4. REQ_GET_ALL_PARAMS (0xA0)   -> Read all DSP parameters + channel names
                                  in one transfer (~2832 bytes)
5. Parse params->channel_names.names[] to populate channel label UI
6. Display the preset list with the active slot highlighted
```

**Note:** Channel names are included in the bulk parameter response (`WireChannelNames` at offset 2480, 352 bytes). There is no need to issue separate `REQ_GET_CHANNEL_NAME` calls during initialization — the bulk transfer provides all names in one shot.

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
3. Read back all parameter values + channel names to update UI:
   - REQ_GET_ALL_PARAMS (0xA0)     -> Single transfer, ~2832 bytes (preferred)
     Includes channel names in params->channel_names.names[]
   OR read individually:
   - REQ_GET_PREAMP
   - REQ_GET_BYPASS
   - REQ_GET_EQ_PARAM (for each channel/band)
   - REQ_GET_LOUDNESS, REQ_GET_LOUDNESS_REF, REQ_GET_LOUDNESS_INTENSITY
   - REQ_GET_CROSSFEED, REQ_GET_CROSSFEED_PRESET, etc.
   - REQ_GET_MATRIX_ROUTE (for each crosspoint)
   - REQ_GET_OUTPUT_ENABLE/GAIN/MUTE/DELAY (for each output)
   - REQ_GET_CHANNEL_NAME (for each channel)
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

### Renaming a Channel

```
1. REQ_SET_CHANNEL_NAME with channel index and 32-byte zero-padded name
2. Update channel label in UI
3. (Optional) REQ_PRESET_SAVE to persist the name change in the active slot
```

**Notes:**
- Channel names live in RAM and are NOT automatically written to flash
- To persist, the host must explicitly save the active preset after renaming
- To reset a channel to its default name, send the default string (e.g., "SPDIF 1 L") or load a preset that has default names
- Channel names are per-preset: loading a different preset may change the names
- All names can be read/written in one shot via `REQ_GET_ALL_PARAMS` / `REQ_SET_ALL_PARAMS`

### Complete Channel Name Workflow Example

```c
// === 1. Read all channel names (via bulk params) ===
uint8_t buf[2832];
int ret = libusb_control_transfer(handle, 0xC1, 0xA0, 0, 2, buf, 2832, 5000);
if (ret == 2832) {
    WireBulkParams *params = (WireBulkParams *)buf;
    int num_ch = params->header.num_channels;
    for (int ch = 0; ch < num_ch; ch++) {
        update_channel_label(ch, params->channel_names.names[ch]);
    }
}

// === 2. Read a single channel name ===
char name[32];
ret = libusb_control_transfer(handle, 0xC1, 0x9C, /*channel=*/2, 2,
                               (uint8_t*)name, 32, 1000);
// name now contains "SPDIF 1 L" (or user-set name)

// === 3. Rename a channel (always send fixed 32-byte buffer) ===
char new_name[32] = {0};
strncpy(new_name, "Front Left", 31);
libusb_control_transfer(handle, 0x41, 0x9B, /*channel=*/2, 2,
                         (uint8_t*)new_name, 32, 1000);
// Name is now live but NOT persisted to flash

// === 4. Persist by saving to the active preset ===
uint8_t active_slot = 0;
libusb_control_transfer(handle, 0xC1, 0x9A, 0, 2, &active_slot, 1, 1000);
uint8_t status;
libusb_control_transfer(handle, 0xC1, 0x90, active_slot, 2, &status, 1, 1000);
// Channel names are now saved in the preset slot

// === 5. After loading a different preset, re-read names ===
libusb_control_transfer(handle, 0xC1, 0x91, /*slot=*/3, 2, &status, 1, 1000);
// Wait for mute period
usleep(10000);
// Read back — channel names may have changed
ret = libusb_control_transfer(handle, 0xC1, 0xA0, 0, 2, buf, 2832, 5000);
// Update all channel labels from params->channel_names.names[]

// === 6. Rename via bulk SET (all names at once) ===
WireBulkParams *out = (WireBulkParams *)buf;
// ... populate all other fields ...
strncpy(out->channel_names.names[0], "Main L", 31);
strncpy(out->channel_names.names[1], "Main R", 31);
strncpy(out->channel_names.names[2], "Front L", 31);
strncpy(out->channel_names.names[3], "Front R", 31);
// etc.
ret = libusb_control_transfer(handle, 0x41, 0xA1, 0, 2,
                               (uint8_t*)out, 2832, 5000);
```

### Deleting a Preset

```
1. Confirm with user (application responsibility)
2. REQ_PRESET_DELETE slot=N
3. (Optional) REQ_PRESET_SET_NAME slot=N with empty string to clear the name
4. Update UI to show the slot as empty
```

## Data Structures

### Live State

```c
// usb_audio.c — live channel names, one per channel
char channel_names[NUM_CHANNELS][PRESET_NAME_LEN];  // 224 B (RP2040) / 352 B (RP2350)

// usb_audio.c — helper to populate default name for a channel
void get_default_channel_name(int ch, char *buf);  // declared in usb_audio.h
```

### Flash Storage (`PresetSlot` in `flash_storage.c`)

```c
// Added in SLOT_DATA_VERSION 8, at the end of PresetSlot (before closing brace):
char channel_names[NUM_CHANNELS][PRESET_NAME_LEN];  // V8
```

`SLOT_DATA_VERSION` is 8. When loading a slot with `version < 8`, the firmware calls `get_default_channel_name()` for each channel instead of reading the (nonexistent) field.

### Wire Format (`WireChannelNames` in `bulk_params.h`)

```c
#define WIRE_NAME_LEN  32   // Must match PRESET_NAME_LEN

typedef struct __attribute__((packed)) {
    char names[WIRE_MAX_CHANNELS][WIRE_NAME_LEN];  // 11 × 32 = 352 bytes
} WireChannelNames;
```

This section is appended to the end of `WireBulkParams` (after `eq`). `WIRE_FORMAT_VERSION` is 2.

**Collect** (`bulk_params_collect`): copies `channel_names[ch]` → `out->channel_names.names[ch]` for each valid channel. Remaining entries (beyond `NUM_CHANNELS`) are zero from the initial `memset`.

**Apply** (`bulk_params_apply`): copies `in->channel_names.names[ch]` → `channel_names[ch]` for each valid channel, then forces NUL termination at byte 31 (`channel_names[ch][PRESET_NAME_LEN - 1] = '\0'`).

### Complete `WireBulkParams` Struct (for host-side parsing)

The following C struct can be used directly by host applications to parse/build bulk parameter payloads. All fields are little-endian, packed with no padding. All `float` fields are IEEE 754 single-precision at 4-byte-aligned offsets within their section.

```c
#include <stdint.h>

#define WIRE_MAX_CHANNELS        11
#define WIRE_MAX_OUTPUT_CHANNELS  9
#define WIRE_MAX_INPUT_CHANNELS   2
#define WIRE_MAX_BANDS           12
#define WIRE_MAX_PIN_OUTPUTS      5
#define WIRE_NAME_LEN            32
#define WIRE_FORMAT_VERSION       2

#pragma pack(push, 1)

typedef struct {
    uint8_t  format_version;         // Must be 2
    uint8_t  platform_id;            // 0=RP2040, 1=RP2350
    uint8_t  num_channels;           // 7 (RP2040) or 11 (RP2350)
    uint8_t  num_output_channels;    // 5 (RP2040) or 9 (RP2350)
    uint8_t  num_input_channels;     // Always 2
    uint8_t  max_bands;              // Always 12
    uint16_t payload_length;         // sizeof(WireBulkParams) = 2832
    uint16_t fw_version_major;
    uint16_t fw_version_minor;
    uint32_t reserved;               // Always 0
} WireHeader;                        // 16 bytes, offset 0

typedef struct {
    float    preamp_gain_db;
    uint8_t  bypass;
    uint8_t  loudness_enabled;
    uint8_t  reserved[2];
    float    loudness_ref_spl;
    float    loudness_intensity_pct;
} WireGlobalParams;                  // 16 bytes, offset 16

typedef struct {
    uint8_t  enabled;
    uint8_t  preset;
    uint8_t  itd_enabled;
    uint8_t  reserved;
    float    custom_fc;
    float    custom_feed_db;
    uint32_t reserved2;
} WireCrossfeedParams;               // 16 bytes, offset 32

typedef struct {
    float    gain_db[3];
    uint8_t  mute[3];
    uint8_t  reserved;
} WireLegacyChannels;                // 16 bytes, offset 48

typedef struct {
    float    delay_ms[WIRE_MAX_CHANNELS];
} WireChannelDelays;                 // 44 bytes, offset 64

typedef struct {
    uint8_t  enabled;
    uint8_t  phase_invert;
    uint8_t  reserved[2];
    float    gain_db;
} WireCrosspoint;                    // 8 bytes each

typedef struct {
    uint8_t  enabled;
    uint8_t  mute;
    uint8_t  reserved[2];
    float    gain_db;
    float    delay_ms;
} WireOutputChannel;                 // 12 bytes each

typedef struct {
    uint8_t  num_pin_outputs;
    uint8_t  pins[WIRE_MAX_PIN_OUTPUTS];
    uint8_t  reserved[2];
} WirePinConfig;                     // 8 bytes, offset 360

typedef struct {
    uint8_t  type;                   // 0=flat, 1=peaking, 2=lowshelf,
                                     // 3=highshelf, 4=lowpass, 5=highpass
    uint8_t  reserved[3];
    float    freq;                   // Hz
    float    q;                      // Q factor
    float    gain_db;                // dB
} WireBandParams;                    // 16 bytes each

typedef struct {
    char     names[WIRE_MAX_CHANNELS][WIRE_NAME_LEN];
} WireChannelNames;                  // 352 bytes, offset 2480

typedef struct {
    WireHeader          header;                                          //     0: 16 B
    WireGlobalParams    global;                                          //    16: 16 B
    WireCrossfeedParams crossfeed;                                       //    32: 16 B
    WireLegacyChannels  legacy;                                          //    48: 16 B
    WireChannelDelays   delays;                                          //    64: 44 B
    WireCrosspoint      crosspoints[WIRE_MAX_INPUT_CHANNELS]
                                   [WIRE_MAX_OUTPUT_CHANNELS];           //   108: 144 B
    WireOutputChannel   outputs[WIRE_MAX_OUTPUT_CHANNELS];               //   252: 108 B
    WirePinConfig       pins;                                            //   360: 8 B
    WireBandParams      eq[WIRE_MAX_CHANNELS][WIRE_MAX_BANDS];           //   368: 2112 B
    WireChannelNames    channel_names;                                   //  2480: 352 B
} WireBulkParams;                    // Total: 2832 bytes

#pragma pack(pop)
```

**Parsing channel names from a bulk response:**

```c
WireBulkParams *params = (WireBulkParams *)buf;
int num_ch = params->header.num_channels;  // 7 or 11
for (int ch = 0; ch < num_ch; ch++) {
    // Each name is 32 bytes, NUL-terminated
    printf("Channel %d: %s\n", ch, params->channel_names.names[ch]);
}
// Entries beyond num_channels are zero-padded (ignore them)
```

### Size Summary

| Structure | RP2040 | RP2350 |
|-----------|--------|--------|
| `channel_names` (BSS) | 224 B (7 × 32) | 352 B (11 × 32) |
| `PresetSlot.channel_names` (flash) | 224 B | 352 B |
| `WireChannelNames` (wire) | 352 B (always max) | 352 B (always max) |
| `WireBulkParams` total | 2832 B | 2832 B |

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
| Set channel name without saving | Name is live in RAM but lost on reboot; persist with `REQ_PRESET_SAVE` |
| Load preset with version < 8 | Channel names reset to defaults (backward compatibility) |
| Channel name > 31 chars via SET | Truncated to 31 chars, byte 32 always NUL |
| Channel name via bulk SET | NUL enforced at position 31 after copy |
| GET channel name with invalid index | Returns STALL (no response) |
| SET channel name with invalid index | Silently ignored |
| Factory reset | Channel names reset to defaults |

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
| `slot_buf` (PresetSlot, static) | ~1.8 KB (RP2040) / ~2.8 KB (RP2350) | Reused for each save operation |
| `write_buf` (sector scratch) | 4 KB | Page-aligned, used for flash writes |
| `channel_names` | 224 B (RP2040) / 352 B (RP2350) | Live channel name array |
| `bulk_param_buf` | 4 KB | Shared GET/SET buffer (includes channel names section) |
| `preset_loading` + `preset_mute_counter` | 5 bytes | Mute-on-load control |
| **Total BSS increase** | **~6 KB (RP2040) / ~7 KB (RP2350)** | |
