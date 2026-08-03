#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "dac_hw_mute.h"        // DacHwMuteConfig (stored in PresetDirectory)
#include "control_surfaces.h"   // CsFlashConfig (stored in PresetDirectory)

// Legacy result codes (used by flash_save_params and the preset/boot API)
#define FLASH_OK            0
#define FLASH_ERR_WRITE     1
#define FLASH_ERR_NO_DATA   2
#define FLASH_ERR_CRC       3

// ============================================================================
// LEGACY API (now redirects through the preset system)
// ============================================================================

// Save current live state to the active preset slot (or slot 0 if none active).
int flash_save_params(void);

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
//   - output_config_mode:  OUTPUT_CONFIG_MODE_INDEPENDENT (0) or _WITH_PRESET (1)
//   - master_volume_mode:  MASTER_VOLUME_MODE_INDEPENDENT (0) or _WITH_PRESET (1)
void preset_get_directory(uint16_t *slot_occupied, uint8_t *startup_mode,
                          uint8_t *default_slot, uint8_t *last_active,
                          uint8_t *output_config_mode, uint8_t *master_volume_mode);

// Set startup behavior.
//   mode: PRESET_STARTUP_SPECIFIED or PRESET_STARTUP_LAST_ACTIVE
//   default_slot: which slot to load in SPECIFIED mode (0-9)
// Returns PRESET_OK or PRESET_ERR_INVALID_SLOT.
uint8_t preset_set_startup(uint8_t mode, uint8_t default_slot);

// Set the physical IO/output-config persistence mode (OUTPUT_CONFIG_MODE_*):
// whether output pins/types, I2S MCK/BCK and the SPDIF RX pin travel with
// presets (WITH_PRESET) or are stored device-global (INDEPENDENT).  Values
// outside the valid range clamp to INDEPENDENT.
void preset_set_output_config_mode(uint8_t mode);

// Copy the live physical IO config into the directory's device-global block and
// persist (the explicit "save output config" for INDEPENDENT mode).  Accepted
// in both modes; dormant in WITH_PRESET.  Returns PRESET_OK or
// PRESET_ERR_FLASH_WRITE.  Mirrors preset_save_master_volume().
uint8_t preset_save_output_config(void);

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

// DAC hardware mute config (board-level, directory-stored — V3+).  Both setter
// and getter are synchronous and main-loop only.  The setter assumes the
// caller has already validated the config (dac_hw_mute_set_config does this).
void preset_set_dac_hw_mute(const DacHwMuteConfig *cfg);
void preset_get_dac_hw_mute(DacHwMuteConfig *out);

// Selectable sys_clk (device-global, directory-stored; V17+).  vreg 0xFF means
// "use the mode's default voltage".  The setter is synchronous and main-loop
// only; the getter is safe to call before the clock is configured (XIP reads
// plus, at most, a directory migration write).  Either getter pointer may be NULL.
void preset_set_sys_clock(uint8_t mode, uint8_t vreg);
void preset_get_sys_clock(uint8_t *mode, uint8_t *vreg);

// External control-interface config (UART/I2C, board-level, directory-stored,
// V6+).  Setter is synchronous and main-loop only; a NULL pointer leaves that
// interface's stored config unchanged.  Getter copies out; either may be NULL.
void preset_set_ctrl_iface(const UartCtrlConfig *uart, const I2cCtrlConfig *i2c);
void preset_get_ctrl_iface(UartCtrlConfig *uart_out, I2cCtrlConfig *i2c_out);

// Control Surfaces bindings (board-level, directory-stored, V7+).  Getter
// copies out of the RAM cache; bindings persist only through
// preset_set_cs_all below (the REQ_CS_SAVE path).
void preset_get_cs_config(CsFlashConfig *out);

// Control Surfaces IR command table (board-level, directory-stored, V11+).
// Getter copies out of the RAM cache.  preset_set_cs_all persists bindings,
// IR commands and slot names together in one directory-sector write (the
// REQ_CS_SAVE path); synchronous and main-loop only, caller has validated
// all three.  Returns PRESET_OK or PRESET_ERR_*.
void preset_get_cs_ir_config(CsIrConfig *out);
uint8_t preset_set_cs_all(const CsFlashConfig *cfg, const CsIrConfig *ir,
                          const char (*names)[CS_NAME_LEN]);

// Control Surfaces slot names (board-level, directory-stored, V10+).  User
// labels for what each control slot is for; independent of the bindings
// (survive binding changes and slot clears).  Names persist only through
// preset_set_cs_all above; the getter copies CS_NAME_LEN bytes from the RAM
// cache into `name_out`.  Returns PRESET_OK or PRESET_ERR_*.
uint8_t preset_get_cs_name(uint8_t slot, char *name_out);

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

// Margin (ms) added on top of the configured DAC hardware-mute hold when
// flooring preset_mute_counter against a still-pending hold: preset_loading
// auto-clears when the counter expires (update_preset_mute_envelope), and if
// that happens while a deferred consumer of the flag is still waiting on
// dac_hw_mute_hold_elapsed(), the prefill handshake that owns the mute
// release never fires.  Both arming points enforce the floor:
// prepare_pipeline_reset() (main.c) and flash_mute_hold_samples()
// (flash_storage.c).  The margin covers the poll iterations between the hold
// elapsing and the consumer running, plus slop; samples only decrement while
// input is actually processed, so IRQ-off flash blackouts cost nothing.
#define PRESET_MUTE_HOLD_MARGIN_MS  120u

#endif // FLASH_STORAGE_H
