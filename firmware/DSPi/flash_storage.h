#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

// Legacy result codes (used by flash_save_params / flash_load_params)
#define FLASH_OK            0
#define FLASH_ERR_WRITE     1
#define FLASH_ERR_NO_DATA   2
#define FLASH_ERR_CRC       3

// ============================================================================
// LEGACY API (now redirects through the preset system)
// ============================================================================

// Save current live state to the active preset slot (or slot 0 if none active).
int flash_save_params(void);

// Reload the active preset slot from flash.
int flash_load_params(void);

// Reset live state to factory defaults.  Does NOT erase presets.
// The active preset slot is unchanged (still selected, now running defaults).
void flash_factory_reset(void);

// ============================================================================
// PRESET API
// ============================================================================

// Save the current live DSP state into a preset slot (0-9).
// Updates last_active_slot in the directory.
// Returns PRESET_OK or PRESET_ERR_*.
uint8_t preset_save(uint8_t slot);

// Load a preset slot (0-9) into the live DSP state.
// If the slot is occupied, loads user data.  If empty, applies factory defaults.
// Triggers filter recalculation and delay update internally.
// Updates last_active_slot in the directory.
// Sets the preset_loading mute flag for glitch-free switching.
// Returns PRESET_OK or PRESET_ERR_*.
uint8_t preset_load(uint8_t slot);

// Delete a preset slot (0-9).  Erases the flash sector and clears the
// occupied bit.  The active slot selection is unchanged — if the deleted
// slot was active, it remains selected (loading it will yield factory defaults).
// Returns PRESET_OK or PRESET_ERR_INVALID_SLOT.
uint8_t preset_delete(uint8_t slot);

// Get the 32-byte name of a preset slot.  Copies into `name_out` (must be
// at least PRESET_NAME_LEN bytes).  Returns PRESET_OK or PRESET_ERR_*.
uint8_t preset_get_name(uint8_t slot, char *name_out);

// Set the name of a preset slot.  `name` is copied (up to 31 chars + NUL).
// The slot does not need to be occupied — names can be set before saving.
// Returns PRESET_OK or PRESET_ERR_*.
uint8_t preset_set_name(uint8_t slot, const char *name);

// Get a summary of the preset directory:
//   - slot_occupied:       16-bit bitmask (bit N = slot N occupied)
//   - startup_mode:        PRESET_STARTUP_SPECIFIED or PRESET_STARTUP_LAST_ACTIVE
//   - default_slot:        slot loaded in SPECIFIED mode (0-9)
//   - last_active:         last slot that was loaded/saved (always 0-9)
//   - include_pins:        whether preset load restores pin config (0/1)
//   - master_volume_mode:  MASTER_VOLUME_MODE_INDEPENDENT (0) or _WITH_PRESET (1)
void preset_get_directory(uint16_t *slot_occupied, uint8_t *startup_mode,
                          uint8_t *default_slot, uint8_t *last_active,
                          uint8_t *include_pins, uint8_t *master_volume_mode);

// Set startup behavior.
//   mode: PRESET_STARTUP_SPECIFIED or PRESET_STARTUP_LAST_ACTIVE
//   default_slot: which slot to load in SPECIFIED mode (0-9)
// Returns PRESET_OK or PRESET_ERR_INVALID_SLOT.
uint8_t preset_set_startup(uint8_t mode, uint8_t default_slot);

// Set whether preset load/save includes pin configuration.
void preset_set_include_pins(uint8_t include);

// Set the master-volume persistence mode (0 = independent, 1 = per-preset).
// Values outside the valid range are clamped to INDEPENDENT.
void preset_set_master_volume_mode(uint8_t mode);

// Copy the live master volume into the directory's independent field and
// persist.  Accepted regardless of current mode (dormant in mode 1).
// Returns PRESET_OK or PRESET_ERR_FLASH_WRITE.
uint8_t preset_save_master_volume(void);

// Read the directory's independent master-volume field (the value applied at
// boot in mode 0).  Does not affect live state.
float preset_get_saved_master_volume(void);

// Set the SPDIF RX input GPIO pin (device-level, stored in directory).
void preset_set_spdif_rx_pin(uint8_t pin);

// Get the currently active preset slot (always 0-9).
uint8_t preset_get_active(void);

// ============================================================================
// BOOT / MIGRATION
// ============================================================================

// Called once at boot.  Always selects a preset.  Loads the appropriate preset
// based on startup config; if the target slot is empty, applies factory defaults.
// If no preset directory exists, attempts legacy migration from the old
// single-sector flash format (copies into slot 0, sets as default).
// Always returns FLASH_OK.
int preset_boot_load(void);

// Audio mute flag — set by preset_load(), cleared by audio callback after
// the mute period expires.  The audio callback checks this flag and outputs
// silence while it is set.
extern volatile bool preset_loading;
extern volatile uint32_t preset_mute_counter;

// Number of samples to mute during preset switch (~5ms at 48kHz)
#define PRESET_MUTE_SAMPLES  256

#endif // FLASH_STORAGE_H
