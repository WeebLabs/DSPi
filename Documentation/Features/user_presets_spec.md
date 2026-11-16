# User Presets Specification

*Last updated: 2026-11-16 (vendor commands on the paged protocol of firmware 1.1.7; delete behavior, IO config mode and result codes corrected)*

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
- Whether the IO configuration (pins, output types) follows presets (`output_config_mode`)
- The master volume persistence mode

### Channel Names

Per-channel user-configurable names stored in the live `channel_names[NUM_CHANNELS][PRESET_NAME_LEN]` global array. Each name is a 32-byte NUL-terminated ASCII string (max 31 visible characters).

**Storage lifecycle:**
- **Live state:** The `channel_names` array in `usb_audio.c` holds the current names. Individual names can be read/written via the vendor command `ROUTE.CHANNEL_NAME`. Changes are immediate in the live array but are NOT automatically persisted to flash.
- **Preset persistence:** When a preset is saved (`STORE.PRESET_SAVE`), the current `channel_names` array is copied into the `PresetSlot.channel_names` field. When loaded, names are restored from the slot. Channel names are always included in preset save/load. There is no opt-out flag (unlike pin config).
- **Bulk transfer:** Channel names are included in the `WireChannelNames` section of `WireBulkParams`, both for `STORE.ALL_PARAMS` GET and SET. The host receives and can modify channel names as part of the full state transfer.
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
| **Command** | `STORE.PRESET_NAME`, wValue=slot | `ROUTE.CHANNEL_NAME`, wValue=channel |
| **Persistence** | Deferred flash write after each SET | Live in RAM only; persisted via `STORE.PRESET_SAVE` |
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
| `PLATFORM_ID` | 0 | 1 | Used in `SYS.PLATFORM` and bulk params header |

**Determining platform at runtime:** Use `SYS.PLATFORM`, whose byte 0 is the platform (0=RP2040, 1=RP2350), or read `header.platform_id` / `header.num_channels` from a `STORE.ALL_PARAMS` response. The valid channel index range for `ROUTE.CHANNEL_NAME` is 0 to `header.num_channels - 1`.

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
| Pin Config | GPIO assignments for all outputs | Only restored in with-preset IO mode (`output_config_mode` 1) |
| Channel Names | Per-channel 32-byte NUL-terminated names | Default names for slots with version < 8 |

## Vendor Commands

*Updated 2026-11-16 for the paged protocol of firmware 1.1.7. Firmware 1.1.6 used opcodes 0x90-0x9C and 0xA0-0xA1 for the same functions. The old to new mapping is in `vendor_protocol_migration_notes.md`.*

All preset commands are on the `STORE` page (0x02) of the paged protocol (see `../commands.md`), except the channel names, which live on the `ROUTE` page (0x05) with the other per-channel settings. The page travels in the high byte of `wIndex` and the vendor interface (2) in the low byte, so a `STORE` request carries `wIndex` 0x0202 and a `ROUTE` request 0x0502. `bRequest` is the command number. `wValue` is the address (slot or channel) and is 0 for a command with no address. Bulk parameter transfer (`STORE.ALL_PARAMS`) is also documented here because it is closely related to the preset workflow.

### Command Summary

| Command | Page | Cmd | Dir | Address (`wValue`) | Size | Description |
|---|---|---|---|---|---|---|
| `STORE.PRESET_SAVE` | 0x02 | 0x10 | SET (zero-length) | slot 0-9 | SET 0 | Save current live state to slot |
| `STORE.PRESET_LOAD` | 0x02 | 0x11 | SET (zero-length) | slot 0-9 | SET 0 | Load slot into live state |
| `STORE.PRESET_DELETE` | 0x02 | 0x12 | SET (zero-length) | slot 0-9 | SET 0 | Delete (erase) a preset slot |
| `STORE.PRESET_NAME` | 0x02 | 0x13 | GET, SET | slot 0-9 | GET 32, SET 1-32 | Slot name |
| `STORE.PRESET_ACTIVE` | 0x02 | 0x14 | GET | none (0) | GET 1 | Active slot index |
| `STORE.DIRECTORY` | 0x02 | 0x15 | GET | none (0) | GET 7 | Directory summary |
| `STORE.STARTUP` | 0x02 | 0x16 | GET, SET | none (0) | GET 2, SET 2 | Startup configuration |
| `STORE.OUTPUT_CONFIG_MODE` | 0x02 | 0x20 | GET, SET | none (0) | GET 1, SET 1 | IO config persistence mode (formerly the pin-inclusion flag) |
| `STORE.OUTPUT_CONFIG_SAVE` | 0x02 | 0x21 | SET (zero-length) | none (0) | SET 0 | Persist the live IO config (see `output_config_independent_load.md`) |
| `STORE.FACTORY_RESET` | 0x02 | 0x30 | SET (zero-length) | none (0) | SET 0 | Reset live state to factory defaults |
| `ROUTE.CHANNEL_NAME` | 0x05 | 0x31 | GET, SET | channel 0 to NUM_CHANNELS-1 | GET 32, SET 1-32 | Channel name |
| `STORE.ALL_PARAMS` | 0x02 | 0x40 | GET, SET | none (0) | GET 6136, SET 6136 | Complete DSP state (multi-packet). 6136 bytes at wire V32 |
| `STORE.ALL_PARAMS_CHUNK` | 0x02 | 0x41 | GET, SET | byte offset | GET var, SET var | The same state in pieces, USB only (see `bulk_params_chunking.md`) |

### Result Codes

Save, load, delete, the name and startup writes, and factory reset are deferred. The SET is validated at once and queued for the main loop. ACK means the request passed validation and was queued (`VC_OK`). It does not mean the flash write or the load has finished. A rejected SET STALLs, and the reason is readable from `SYS.LAST_RESULT` (UART and I2C return the code in the response status byte).

| Code | Name | When |
|------|------|------|
| `0x00` | `VC_OK` | Accepted and queued |
| `0x12` | `VC_ERR_LENGTH` | A payload on a zero-length SET, an empty or over-long name, or a `STORE.STARTUP` payload other than 2 bytes |
| `0x13` | `VC_ERR_ADDRESS` | Slot index >= 10, channel index out of range, or a non-zero `wValue` on a command with no address |
| `0x14` | `VC_ERR_VALUE` | `STORE.STARTUP` mode above 1 or slot above 9 |

Firmware 1.1.6 returned the old `PRESET_*` status byte in an IN data stage instead. `PRESET_ERR_INVALID_SLOT` (0x01) is now `VC_ERR_ADDRESS`. `PRESET_ERR_CRC` (0x03) and `PRESET_ERR_FLASH_WRITE` (0x04) arise only later in the main loop, so they never reach the host. A host that needs to confirm completion reads back (`STORE.DIRECTORY`, `STORE.PRESET_ACTIVE`, the parameters) or watches the notifications (`NOTIFY_EVT_PRESET_LOADED` followed by `NOTIFY_EVT_BULK_INVALIDATED` after a load).

---

### `STORE.PRESET_SAVE`

Save the current live DSP state into a preset slot.

**Transfer type:** Control OUT (Host to Device), zero-length
**bmRequestType:** `0x41` (Host-to-Device, Vendor, Interface)
**bRequest:** `0x10`
**wValue:** Slot index (0-9)
**wIndex:** `0x0202`
**wLength:** 0

**Response:** None. Success is the ACK of the status stage.

**Behavior:**
1. Validates the slot index (must be 0-9). A bad slot STALLs with `VC_ERR_ADDRESS`
2. Queues the save and ACKs
3. The main loop snapshots all current live DSP parameters into a `PresetSlot` structure
4. Computes CRC32 over the data section
5. Erases and programs the flash sector for the given slot
6. Updates the preset directory: marks slot as occupied, sets `last_active_slot`

**Notes:**
- Saving to an occupied slot overwrites it without confirmation
- The slot name is NOT affected by save (names are managed separately)
- If the slot previously had a name, the name is preserved
- Interrupts are briefly disabled during the flash write (~1-2 ms)

**Example (C/libusb):**
```c
int ret = libusb_control_transfer(handle,
    0x41,       // bmRequestType: OUT, Vendor, Interface
    0x10,       // bRequest: STORE.PRESET_SAVE
    slot,       // wValue: slot index (0-9)
    0x0202,     // wIndex: page STORE, vendor interface 2
    NULL,       // no data
    0,          // wLength: zero-length SET
    1000);      // timeout ms
if (ret == 0) {
    // Save accepted and queued
}
```

---

### `STORE.PRESET_LOAD`

Load a preset slot into the live DSP state.

**Transfer type:** Control OUT, zero-length
**bmRequestType:** `0x41`
**bRequest:** `0x11`
**wValue:** Slot index (0-9)
**wIndex:** `0x0202`
**wLength:** 0

**Response:** None. Success is the ACK of the status stage.

**Behavior:**
1. Validates the slot index. A bad slot STALLs with `VC_ERR_ADDRESS`
2. Queues the load and ACKs. The main loop then does the rest
3. Engages audio mute (~5 ms / 256 samples at 48 kHz) to prevent clicks
4. If the slot is occupied: reads and validates the slot data from flash (CRC check), applies user data to live state. In with-preset IO mode (`STORE.OUTPUT_CONFIG_MODE` 1), restores pin configuration (with validation)
5. If the slot is empty (unconfigured): applies factory defaults to live state
6. Recalculates all filter coefficients for the current sample rate
7. Updates delay sample counts
8. Triggers loudness and crossfeed recomputation
9. Updates `last_active_slot` in the directory
10. Audio resumes after the mute period, and the device pushes `NOTIFY_EVT_PRESET_LOADED` then `NOTIFY_EVT_BULK_INVALIDATED`

**Notes:**
- The mute is transparent to the host application
- Filter states (biquad/SVF accumulators) are reset as part of recalculation
- If the device is playing audio, there will be a brief (~5 ms) silence during the switch
- The host application should read back the new parameter values after load if it needs to update its UI

**Example (C/libusb):**
```c
int ret = libusb_control_transfer(handle, 0x41, 0x11, slot, 0x0202, NULL, 0, 1000);
if (ret == 0) {
    // Load queued - refresh UI by reading back parameters once it completes
}
```

---

### `STORE.PRESET_DELETE`

Delete a preset slot, erasing its flash sector and marking it as unoccupied.

**Transfer type:** Control OUT, zero-length
**bmRequestType:** `0x41`
**bRequest:** `0x12`
**wValue:** Slot index (0-9)
**wIndex:** `0x0202`
**wLength:** 0

**Response:** None. Success is the ACK of the status stage. A bad slot STALLs with `VC_ERR_ADDRESS`.

**Behavior:**
1. Erases the flash sector for the given slot
2. Clears the occupied bit and the slot name in the directory
3. The active slot selection is unchanged. If the deleted slot was active, it remains selected
4. If the deleted slot was the active slot, the live state is reset to factory defaults, as loading an empty slot would do (master volume and IO config follow their persistence modes). Deleting any other slot leaves the live state alone

**Notes:**
- Deleting a slot also clears its name.
- It is safe to delete the active slot. The slot remains selected but is now unconfigured.
- Deleting a slot that is already empty succeeds (no error).

---

### `STORE.PRESET_NAME` GET

Get the 32-byte name of a preset slot.

**Transfer type:** Control IN
**bmRequestType:** `0xC1`
**bRequest:** `0x13`
**wValue:** Slot index (0-9)
**wIndex:** `0x0202`
**wLength:** 32

**Response:** 32 bytes, NUL-terminated ASCII string.

**Notes:**
- Empty/unnamed slots return 32 zero bytes
- Names can be set before the slot is occupied (i.e., you can name a slot before saving to it)
- Maximum name length is 31 characters (the 32nd byte is always NUL)
- STALLs with `VC_ERR_ADDRESS` if the slot index is invalid

**Example (C/libusb):**
```c
char name[32];
int ret = libusb_control_transfer(handle, 0xC1, 0x13, slot, 0x0202,
                                   (uint8_t*)name, 32, 1000);
if (ret == 32) {
    printf("Slot %d: %s\n", slot, name);
}
```

---

### `STORE.PRESET_NAME` SET

Set the name of a preset slot.

**Transfer type:** Control OUT (Host to Device)
**bmRequestType:** `0x41` (Host-to-Device, Vendor, Interface)
**bRequest:** `0x13`
**wValue:** Slot index (0-9)
**wIndex:** `0x0202`
**wLength:** 1-32 (32 recommended)

**Payload:** 1 to 32 bytes of NUL-terminated ASCII. The firmware copies up to 31 characters and ensures NUL termination. An empty or longer payload STALLs with `VC_ERR_LENGTH`, and a bad slot with `VC_ERR_ADDRESS`.

> **Important:** Send exactly 32 bytes with `wLength = 32`, zero-padded after the NUL terminator. Short transfers have caused USB disconnect and device reset on some host controllers.

**Behavior:**
1. Stages the name and ACKs
2. The main loop copies the name into the directory cache and writes the updated directory to flash
3. The slot does not need to be occupied

**Notes:**
- To clear a name, send a 32-byte buffer containing a single NUL byte followed by 31 zero bytes
- Names are stored in the directory sector, not in the preset slot itself
- Setting a name triggers a deferred directory flash write

**Example (C/libusb):**
```c
char buf[32] = {0};
strncpy(buf, "Living Room EQ", 31);
libusb_control_transfer(handle,
    0x41,                      // bmRequestType: OUT, Vendor, Interface
    0x13,                      // bRequest: STORE.PRESET_NAME
    slot,                      // wValue: slot index
    0x0202,                    // wIndex: page STORE, vendor interface 2
    (uint8_t*)buf,             // data: 32-byte zero-padded name
    32,                        // wLength: 32 (PRESET_NAME_LEN)
    1000);
```

---

### `STORE.DIRECTORY`

Get a summary of the entire preset directory.

**Transfer type:** Control IN
**bmRequestType:** `0xC1`
**bRequest:** `0x15`
**wValue:** 0
**wIndex:** `0x0202`
**wLength:** 7 (a shorter request gets the leading bytes)

**Response:** 7 bytes:

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 2 | `slot_occupied` | Little-endian uint16 bitmask (bit 0 = slot 0, ..., bit 9 = slot 9) |
| 2 | 1 | `startup_mode` | 0 = specified default, 1 = last active |
| 3 | 1 | `default_slot` | Slot loaded in "specified default" mode (0-9) |
| 4 | 1 | `last_active` | Last loaded/saved slot (always 0-9) |
| 5 | 1 | `output_config_mode` | IO config persistence: 0 = independent, 1 = with preset (formerly `include_pins`) |
| 6 | 1 | `master_volume_mode` | 0 = independent, 1 = with preset (see `master_volume_spec.md`) |

**Notes:**
- This is the most efficient way to query the overall state of the preset system
- Use this on application startup to populate the preset list UI
- Follow up with `STORE.PRESET_NAME` for each occupied slot to get names

**Example (C/libusb):**
```c
uint8_t dir[7];
libusb_control_transfer(handle, 0xC1, 0x15, 0, 0x0202, dir, 7, 1000);

uint16_t occupied = dir[0] | (dir[1] << 8);
for (int i = 0; i < 10; i++) {
    if (occupied & (1 << i)) {
        printf("Slot %d: occupied\n", i);
    }
}
printf("Startup mode: %s\n", dir[2] ? "last active" : "specified default");
printf("Default slot: %d\n", dir[3]);
printf("Last active: %d\n", dir[4]);
printf("IO config with preset: %s\n", dir[5] ? "yes" : "no");
```

---

### `STORE.STARTUP` SET

Configure the startup behavior.

**Transfer type:** Control OUT
**bmRequestType:** `0x41`
**bRequest:** `0x16`
**wValue:** 0
**wIndex:** `0x0202`
**wLength:** 2

**Payload:** 2 bytes:

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | `startup_mode` | 0 = specified default, 1 = last active |
| 1 | 1 | `default_slot` | Slot to load in "specified default" mode (0-9) |

**Behavior:**
1. Validates both values. A mode above 1 or a slot above 9 STALLs with `VC_ERR_VALUE`, and a payload other than 2 bytes with `VC_ERR_LENGTH`
2. Queues the change and ACKs
3. The main loop updates the directory and writes it to flash

**Notes:**
- The `default_slot` value is always stored, even when `startup_mode` is "last active" (it serves as a fallback)
- Setting `startup_mode = 0` and `default_slot = 3` means: "always boot with slot 3"
- Setting `startup_mode = 1` means: "boot with whatever was last used"
- **Default behavior** on a fresh device: `startup_mode = 0`, `default_slot = 0` (load slot 0 on boot)

---

### `STORE.STARTUP` GET

Get the current startup configuration.

**Transfer type:** Control IN
**bmRequestType:** `0xC1`
**bRequest:** `0x16`
**wValue:** 0
**wIndex:** `0x0202`
**wLength:** 2

**Response:** 2 bytes, the same layout as the SET payload:

| Offset | Size | Field |
|--------|------|-------|
| 0 | 1 | `startup_mode` |
| 1 | 1 | `default_slot` |

The last active slot, which firmware 1.1.6 appended as a third byte, is byte 4 of `STORE.DIRECTORY`.

---

### `STORE.OUTPUT_CONFIG_MODE`

Set or get whether the physical IO configuration (output pins, output types, I2S MCK/BCK, S/PDIF RX pin, limiter settings) follows each preset or is stored on the device independently. This replaces the pin-inclusion flag of earlier firmware. The full model is in `output_config_independent_load.md`.

**bmRequestType:** `0x41` for SET, `0xC1` for GET
**bRequest:** `0x20`
**wValue:** 0
**wIndex:** `0x0202`
**wLength:** 1

**Payload / response:** 1 byte: `0` = independent (the IO config is a device-level setting, applied at boot only), `1` = with preset (default). A SET value above 1 stores 0.

**Notes:**
- Pin data is always stored in the preset for completeness, but only restored on load in with-preset mode
- The mode also gates pin application during `STORE.ALL_PARAMS` SET bulk parameter transfers
- In independent mode, `STORE.OUTPUT_CONFIG_SAVE` persists the live IO config to the device
- The mode is persisted in the directory (survives power cycles) and is byte 5 of `STORE.DIRECTORY`

---

### `STORE.PRESET_ACTIVE`

Get the index of the currently active preset slot.

**Transfer type:** Control IN
**bmRequestType:** `0xC1`
**bRequest:** `0x14`
**wValue:** 0
**wIndex:** `0x0202`
**wLength:** 1

**Response:** 1 byte: slot index (always 0-9).

**Notes:**
- A preset is always active. This never returns `0xFF`
- The active slot is updated whenever a preset is saved or loaded
- If the active slot is deleted, it remains selected (the slot is now unconfigured and will load factory defaults)

---

### `ROUTE.CHANNEL_NAME` SET

Set the user-configurable name of a channel.

**Transfer type:** Control OUT (Host to Device)
**bmRequestType:** `0x41` (Host-to-Device, Vendor, Interface)
**bRequest:** `0x31`
**wValue:** Channel index (0 to NUM_CHANNELS-1)
**wIndex:** `0x0502` (page ROUTE)
**wLength:** 1-32 (32 recommended)

**Payload:** 1 to 32 bytes of NUL-terminated ASCII. The firmware copies up to 31 characters and ensures NUL termination. An empty or longer payload STALLs with `VC_ERR_LENGTH`, and a bad channel with `VC_ERR_ADDRESS`. Firmware 1.1.6 accepted up to 64 bytes and ignored a bad channel.

> **Important:** Send exactly 32 bytes with `wLength = 32`, zero-padded after the NUL terminator. Short transfers (e.g., sending only `strlen(name) + 1` bytes) have caused USB disconnect and device reset on some host controllers (particularly certain xHCI implementations). This matches the fixed-size convention used by `STORE.PRESET_NAME` and the `WireChannelNames` section in bulk transfers.

**Behavior:**
1. The dispatcher validates the channel index and the payload length
2. Clears the existing name and copies the new name into `channel_names[ch]`
3. The name is stored in the live global array (not directly in flash)
4. To persist, save the current state to a preset slot with `STORE.PRESET_SAVE`

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
    0x31,                      // bRequest: ROUTE.CHANNEL_NAME
    channel,                   // wValue: channel index
    0x0502,                    // wIndex: page ROUTE, vendor interface 2
    (uint8_t*)buf,             // data: 32-byte zero-padded name
    32,                        // wLength: 32 (PRESET_NAME_LEN)
    1000);
```

---

### `ROUTE.CHANNEL_NAME` GET

Get the 32-byte name of a channel.

**Transfer type:** Control IN
**bmRequestType:** `0xC1`
**bRequest:** `0x31`
**wValue:** Channel index (0 to NUM_CHANNELS-1)
**wIndex:** `0x0502`
**wLength:** 32

**Response:** 32 bytes, NUL-terminated ASCII string. A bad channel STALLs with `VC_ERR_ADDRESS`.

**Example (C/libusb):**
```c
char name[32];
int ret = libusb_control_transfer(handle, 0xC1, 0x31, channel, 0x0502,
                                   (uint8_t*)name, 32, 1000);
if (ret == 32) {
    printf("Channel %d: %s\n", channel, name);
}
```

---

### `STORE.ALL_PARAMS` GET

Read the complete live DSP state in a single transfer. This is a multi-packet USB control transfer on EP0 (NOT a bulk endpoint transfer).

**Transfer type:** Control IN (multi-packet, 64 bytes per packet)
**bmRequestType:** `0xC1` (Device-to-Host, Vendor, Interface)
**bRequest:** `0x40`
**wValue:** 0
**wIndex:** `0x0202`
**wLength:** `sizeof(WireBulkParams)` (6136 at wire V32)

**Response:** `WireBulkParams` structure (defined in `bulk_params.h`). The table below is the V2 layout this section was first written against. Later versions append sections, and `bulk_params.h` holds the current layout.

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
- Pin configuration is always included in GET regardless of the IO config mode.
- This is the preferred method for reading back parameters after a preset load or on app startup.
- Windows hosts cannot move more than 4096 bytes in one control transfer and use `STORE.ALL_PARAMS_CHUNK` (see `bulk_params_chunking.md`).

**Example (C/libusb):**
```c
uint8_t buf[sizeof(WireBulkParams)];
int ret = libusb_control_transfer(handle,
    0xC1,       // bmRequestType: IN, Vendor, Interface
    0x40,       // bRequest: STORE.ALL_PARAMS
    0,          // wValue
    0x0202,     // wIndex: page STORE, vendor interface 2
    buf,        // data
    sizeof(buf),// wLength
    5000);      // timeout ms (longer for multi-packet)
if (ret == (int)sizeof(buf)) {
    WireBulkParams *params = (WireBulkParams *)buf;
    // Parse params->header, params->global, params->eq, params->channel_names, etc.
}
```

---

### `STORE.ALL_PARAMS` SET

Apply a complete DSP state in a single transfer. This is a multi-packet USB control transfer on EP0 (NOT a bulk endpoint transfer).

**Transfer type:** Control OUT (multi-packet, 64 bytes per packet)
**bmRequestType:** `0x41` (Host-to-Device, Vendor, Interface)
**bRequest:** `0x40`
**wValue:** 0
**wIndex:** `0x0202`
**wLength:** `sizeof(WireBulkParams)` (6136 at wire V32, must match exactly)

**Payload:** `WireBulkParams` structure (same layout as the GET response above).

**Behavior:**
1. A `wLength` other than `sizeof(WireBulkParams)` STALLs at SETUP with `VC_ERR_LENGTH`
2. Firmware receives all packets into the bulk buffer under the shared bulk lock. A lock held by another transport STALLs with `VC_ERR_BUSY`
3. The header is checked before the status stage: `format_version`, `platform_id`, `num_channels`, `num_input_channels`, `num_output_channels`, `payload_length`. A mismatch STALLs the status stage with `VC_ERR_VALUE` and nothing is applied
4. On success `bulk_params_pending` is set and the status stage is ACKed
5. Main loop processes the request:
   a. Waits for Core 1 EQ worker to finish current work (if active)
   b. Engages audio mute (~5 ms / 256 samples at 48 kHz)
   c. Applies all parameters to live state
   d. In with-preset IO mode, applies pin configuration (with validation)
   e. Recalculates all filter coefficients for the current sample rate
   f. Updates delay sample counts
   g. Triggers loudness and crossfeed recomputation

**Notes:**
- The `platform_id` field must match the device's platform (RP2040=0, RP2350=1)
- The `num_channels` and `num_output_channels` fields must match the device's configuration
- Pin application is gated by `STORE.OUTPUT_CONFIG_MODE` (same as preset load)
- Floats are stored as sent. The bulk SET does not apply the finite-value checks of the single-parameter commands
- This command does NOT save to flash. To persist, read `STORE.PRESET_ACTIVE` and follow with `STORE.PRESET_SAVE` to that slot. The 1.1.6 save-params opcode (0x51) is retired
- This command does NOT change the active preset slot

**Example (C/libusb):**
```c
// Assume 'params' is a previously captured WireBulkParams
int ret = libusb_control_transfer(handle,
    0x41,                       // bmRequestType: OUT, Vendor, Interface
    0x40,                       // bRequest: STORE.ALL_PARAMS
    0,                          // wValue
    0x0202,                     // wIndex: page STORE, vendor interface 2
    (uint8_t*)&params,          // data
    sizeof(WireBulkParams),     // wLength: must be exact
    5000);                      // timeout ms
if (ret == sizeof(WireBulkParams)) {
    // Transfer accepted; parameters will be applied within ~5 ms
}
```

---

## Retired and Replaced Commands

Earlier firmware had flash storage commands outside the preset set. Firmware 1.1.7 maps them as follows.

| 1.1.6 opcode | 1.1.7 | Behavior |
|---------------|------|--------------|
| `0x51` (save params) | Retired | Read `STORE.PRESET_ACTIVE`, then `STORE.PRESET_SAVE` to that slot |
| `0x52` | `STORE.OUTPUT_CONFIG_SAVE` | Persists the live IO config. The old load-params meaning of 0x52 was retired before 1.1.6. Use `STORE.PRESET_LOAD` on the active slot to reload it |
| `0x53` | `STORE.FACTORY_RESET` | Resets live state to factory defaults. Active slot unchanged. Does NOT erase any preset slots |

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
- The bulk GET (1.1.6 opcode 0xA0, now `STORE.ALL_PARAMS`): host must request 2832 bytes (was 2480)
- The bulk SET (1.1.6 opcode 0xA1, now `STORE.ALL_PARAMS`): host must send exactly 2832 bytes (was 2480); the firmware rejects `wLength != sizeof(WireBulkParams)` with a STALL
- The firmware's `bulk_params_apply()` rejects payloads where `format_version != 2` or `payload_length != 2832`

Host applications built against format version 1 must be updated to include the channel names section. The simplest upgrade path: allocate a 2832-byte buffer, zero-initialize it, populate the existing fields as before, and leave the channel names section zeroed (the firmware will apply empty strings, which the host can then overwrite with `ROUTE.CHANNEL_NAME` or by loading a preset).

## Application Integration Patterns

### Initialization Flow

When a host application connects to the device:

```
1. STORE.DIRECTORY             -> Get occupied bitmask + startup config
2. For each occupied slot:
   STORE.PRESET_NAME           -> Get the slot's name
3. STORE.PRESET_ACTIVE         -> Determine which slot is currently loaded
4. STORE.ALL_PARAMS (GET)      -> Read all DSP parameters + channel names
                                  in one transfer (6136 bytes at wire V32)
5. Parse params->channel_names.names[] to populate channel label UI
6. Display the preset list with the active slot highlighted
```

**Note:** Channel names are included in the bulk parameter response (`WireChannelNames` at offset 2480, 352 bytes). There is no need to issue separate `ROUTE.CHANNEL_NAME` calls during initialization, because the bulk transfer provides all names in one shot.

### Saving the Current Configuration

```
1. (Optional) STORE.PRESET_NAME    -> Set a descriptive name for the target slot
2. STORE.PRESET_SAVE               -> Save live state to the slot
3. Update UI to show the slot as occupied
```

### Loading a Preset

```
1. STORE.PRESET_LOAD               -> Load the slot (firmware handles mute)
2. Wait for NOTIFY_EVT_PRESET_LOADED, or ~10 ms for the mute period to complete
3. Read back all parameter values + channel names to update UI:
   - STORE.ALL_PARAMS (GET)        -> Single transfer (preferred)
     Includes channel names in params->channel_names.names[]
   OR read individually:
   - ROUTE.PREAMP (for each input channel)
   - FILTER.EQ_BYPASS
   - FILTER.BAND (for each channel/band)
   - DYN.LOUDNESS_ENABLE, DYN.LOUDNESS_REF, DYN.LOUDNESS_INTENSITY
   - FX.CROSSFEED_ENABLE, FX.CROSSFEED_PRESET, etc.
   - ROUTE.MATRIX_ROUTE (for each crosspoint)
   - ROUTE.OUTPUT_ENABLE/_GAIN/_MUTE/_DELAY (for each output)
   - ROUTE.CHANNEL_NAME (for each channel)
```

### Managing Startup Behavior

```
// Set a specific default preset
STORE.STARTUP SET with [mode=0, default_slot=3]
// The device will always load slot 3 on boot

// Use "last active" mode
STORE.STARTUP SET with [mode=1, default_slot=0]
// The device will load whatever was last used
// default_slot=0 serves as fallback if last active is deleted
```

### Copying a Preset

There is no dedicated copy command. To copy preset A to preset B:

```
1. STORE.PRESET_LOAD slot=A      -> Load slot A into live state
2. STORE.PRESET_SAVE slot=B      -> Save live state to slot B
3. STORE.PRESET_NAME slot=B      -> Optionally set a new name for the copy
4. STORE.PRESET_LOAD slot=A      -> Load slot A again to restore it as active
```

### Renaming a Preset

```
STORE.PRESET_NAME SET with slot index and new name
// No need to load or save the preset; names are stored in the directory
```

### Renaming a Channel

```
1. ROUTE.CHANNEL_NAME SET with channel index and 32-byte zero-padded name
2. Update channel label in UI
3. (Optional) STORE.PRESET_SAVE to persist the name change in the active slot
```

**Notes:**
- Channel names live in RAM and are NOT automatically written to flash
- To persist, the host must explicitly save the active preset after renaming
- To reset a channel to its default name, send the default string (e.g., "SPDIF 1 L") or load a preset that has default names
- Channel names are per-preset: loading a different preset may change the names
- All names can be read/written in one shot via `STORE.ALL_PARAMS` GET / SET

### Complete Channel Name Workflow Example

```c
// wIndex carries the page: STORE = 0x0202, ROUTE = 0x0502.
// === 1. Read all channel names (via bulk params) ===
uint8_t buf[sizeof(WireBulkParams)];
int ret = libusb_control_transfer(handle, 0xC1, 0x40, 0, 0x0202, buf, sizeof(buf), 5000);
if (ret == (int)sizeof(buf)) {
    WireBulkParams *params = (WireBulkParams *)buf;
    int num_ch = params->header.num_channels;
    for (int ch = 0; ch < num_ch; ch++) {
        update_channel_label(ch, params->channel_names.names[ch]);
    }
}

// === 2. Read a single channel name ===
char name[32];
ret = libusb_control_transfer(handle, 0xC1, 0x31, /*channel=*/2, 0x0502,
                               (uint8_t*)name, 32, 1000);   // ROUTE.CHANNEL_NAME
// name now contains "SPDIF 1 L" (or user-set name)

// === 3. Rename a channel (always send fixed 32-byte buffer) ===
char new_name[32] = {0};
strncpy(new_name, "Front Left", 31);
libusb_control_transfer(handle, 0x41, 0x31, /*channel=*/2, 0x0502,
                         (uint8_t*)new_name, 32, 1000);     // ROUTE.CHANNEL_NAME
// Name is now live but NOT persisted to flash

// === 4. Persist by saving to the active preset ===
uint8_t active_slot = 0;
libusb_control_transfer(handle, 0xC1, 0x14, 0, 0x0202, &active_slot, 1, 1000);  // STORE.PRESET_ACTIVE
libusb_control_transfer(handle, 0x41, 0x10, active_slot, 0x0202, NULL, 0, 1000); // STORE.PRESET_SAVE
// Channel names are saved in the preset slot once the main loop runs the save

// === 5. After loading a different preset, re-read names ===
libusb_control_transfer(handle, 0x41, 0x11, /*slot=*/3, 0x0202, NULL, 0, 1000); // STORE.PRESET_LOAD
// Wait for NOTIFY_EVT_PRESET_LOADED or the mute period
usleep(10000);
// Read back. Channel names may have changed
ret = libusb_control_transfer(handle, 0xC1, 0x40, 0, 0x0202, buf, sizeof(buf), 5000);
// Update all channel labels from params->channel_names.names[]

// === 6. Rename via bulk SET (all names at once) ===
WireBulkParams *out = (WireBulkParams *)buf;
// ... populate all other fields ...
strncpy(out->channel_names.names[0], "Main L", 31);
strncpy(out->channel_names.names[1], "Main R", 31);
strncpy(out->channel_names.names[2], "Front L", 31);
strncpy(out->channel_names.names[3], "Front R", 31);
// etc.
ret = libusb_control_transfer(handle, 0x41, 0x40, 0, 0x0202,
                               (uint8_t*)out, sizeof(WireBulkParams), 5000);  // STORE.ALL_PARAMS
```

### Deleting a Preset

```
1. Confirm with user (application responsibility)
2. STORE.PRESET_DELETE slot=N (this also clears the slot name)
3. Update UI to show the slot as empty
4. If slot N was active, re-read the live parameters: the device has reset them to factory defaults
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
| Load corrupt slot | Live state unchanged. The load was already ACKed when queued, so the CRC failure does not reach the host |
| Delete active slot | Slot erased and name cleared, active slot unchanged (now unconfigured), live state reset to factory defaults |
| Delete empty slot | Succeeds silently (erase + clear bit is idempotent) |
| Boot with specified default slot empty | Applies factory defaults, slot remains selected |
| Boot with last active slot empty | Applies factory defaults, slot remains selected |
| Boot with corrupt directory | Attempts legacy migration, then factory defaults with slot 0 |
| Set name on empty slot | Succeeds (name stored in directory, independent of slot data) |
| Factory reset | Live state reset to defaults, all presets remain intact, active slot unchanged |
| IO config mode 0 (independent) | Pin data stored in preset but NOT applied on load |
| IO config mode 1 (with preset, default) | Pin data validated and applied on load |
| Flash write fails | Live state unchanged. The request was already ACKed when queued, so the failure does not reach the host |
| Audio playing during preset load | Brief mute (~5 ms), then new preset takes effect |
| Set channel name without saving | Name is live in RAM but lost on reboot; persist with `STORE.PRESET_SAVE` |
| Load preset with version < 8 | Channel names reset to defaults (backward compatibility) |
| Channel name > 31 chars via SET | Truncated to 31 chars, byte 32 always NUL |
| Channel name via bulk SET | NUL enforced at position 31 after copy |
| GET channel name with invalid index | STALL with `VC_ERR_ADDRESS` |
| SET channel name with invalid index | STALL with `VC_ERR_ADDRESS`, nothing stored |
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
| `flash_stage` (flash staging buffer) | 4 KB (RP2040) / 8 KB (RP2350) | Page-aligned; `preset_save()` builds its slot here in place (separate `slot_buf` removed 2026-09-22) |
| `channel_names` | 224 B (RP2040) / 352 B (RP2350) | Live channel name array |
| `bulk_param_buf` | `sizeof(WireBulkParams)` (6,136 B at V32) | Shared GET/SET buffer (includes channel names section) |
| `preset_loading` + `preset_mute_counter` | 5 bytes | Mute-on-load control |
| **Total BSS increase** | **~6 KB (RP2040) / ~7 KB (RP2350)** | |
