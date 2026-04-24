/*
 * flash_storage.c — Preset-based parameter persistence for DSPi
 *
 * Flash Layout (from end of flash, working backwards):
 *
 *   Sector 0  (-48 KB):  Preset Directory  (metadata, slot names, startup config)
 *   Sector 1  (-44 KB):  Preset Slot 0     (full DSP state snapshot)
 *   Sector 2  (-40 KB):  Preset Slot 1
 *   ...
 *   Sector 10 ( -8 KB):  Preset Slot 9
 *   Sector 11 ( -4 KB):  Legacy sector     (old single-preset format, kept for
 *                                            backward compat / migration)
 *
 * Each sector is 4 KB (FLASH_SECTOR_SIZE).  A preset slot stores the complete
 * user-configurable DSP state: EQ bands, preamp, delays, loudness, crossfeed,
 * matrix mixer, channel gains/mutes, and optionally pin assignments.
 *
 * The directory sector holds a 10-bit occupancy bitmask, 10 x 32-byte slot
 * names, startup configuration (which slot to load on boot), and the index
 * of the last-active slot.
 *
 * On boot, preset_boot_load() reads the directory and loads the appropriate
 * slot based on the startup policy.  If no directory exists (first boot after
 * firmware upgrade), it attempts to migrate the legacy single-sector data
 * into slot 0.
 */

#include "flash_storage.h"
#include "config.h"
#include "audio_input.h"
#include "spdif_input.h"
#include "dsp_pipeline.h"
#include "flash_clkdiv.h"
#include "usb_audio.h"
#include "crossfeed.h"
#include "pdm_generator.h"
#include "usb_feedback_controller.h"
#include "leveller.h"
#include "notify.h"

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include <string.h>
#include <math.h>    // powf(), isfinite() for master volume (db_to_linear() clamps at -60 dB)

// ============================================================================
// FLASH GEOMETRY
// ============================================================================

// Total reservation: 12 sectors (48 KB) at the end of flash.
// Sector 0 = directory, sectors 1-10 = preset slots, sector 11 = legacy.
#define PRESET_TOTAL_SECTORS    12
#define PRESET_BASE_OFFSET      (PICO_FLASH_SIZE_BYTES - (PRESET_TOTAL_SECTORS * FLASH_SECTOR_SIZE))

// Individual sector offsets (byte offset from start of flash)
#define DIR_SECTOR_OFFSET       (PRESET_BASE_OFFSET)
#define SLOT_SECTOR_OFFSET(n)   (PRESET_BASE_OFFSET + ((1 + (n)) * FLASH_SECTOR_SIZE))
#define LEGACY_SECTOR_OFFSET    (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

// XIP read pointers (memory-mapped flash)
#define DIR_ADDR                ((const PresetDirectory *)(XIP_BASE + DIR_SECTOR_OFFSET))
#define SLOT_ADDR(n)            ((const PresetSlot *)(XIP_BASE + SLOT_SECTOR_OFFSET(n)))
#define LEGACY_ADDR             ((const LegacyFlashStorage *)(XIP_BASE + LEGACY_SECTOR_OFFSET))

// Magic numbers — each distinct so we can tell sector types apart
#define DIR_MAGIC               0x44535032  // "DSP2"
#define SLOT_MAGIC              0x44535033  // "DSP3"
#define LEGACY_MAGIC            0x44535031  // "DSP1" (original format)

// Current data version for preset slot contents
#define SLOT_DATA_VERSION       13   // V13: input source selection

// ============================================================================
// ON-FLASH STRUCTURES
// ============================================================================

// Storage structure for matrix crosspoint (packed for flash)
typedef struct __attribute__((packed)) {
    uint8_t enabled;
    uint8_t phase_invert;
    uint8_t reserved[2];
    float gain_db;
} FlashMatrixCrosspoint;

// Storage structure for output channel (packed for flash)
typedef struct __attribute__((packed)) {
    uint8_t enabled;
    uint8_t mute;
    uint8_t reserved[2];
    float gain_db;
    float delay_ms;
} FlashOutputChannel;

// --- Preset Directory v1 (legacy — kept only for upgrade migration) ---
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;                        // == 1
    uint16_t reserved;
    uint32_t crc32;

    uint8_t  startup_mode;
    uint8_t  default_slot;
    uint8_t  last_active_slot;
    uint8_t  include_pins;

    uint16_t slot_occupied;
    uint8_t  include_master_volume;
    uint8_t  padding[1];
    char     slot_names[PRESET_SLOTS][PRESET_NAME_LEN];
} PresetDirectory_v1;

// --- Preset Directory v2 (current, sector 0) ---
typedef struct __attribute__((packed)) {
    uint32_t magic;                          // DIR_MAGIC
    uint16_t version;                        // Directory format version (2)
    uint16_t reserved;
    uint32_t crc32;                          // CRC over everything after this 12-byte header

    // Startup configuration
    uint8_t  startup_mode;                   // PRESET_STARTUP_SPECIFIED or _LAST_ACTIVE
    uint8_t  default_slot;                   // Slot to load in SPECIFIED mode (0-9)
    uint8_t  last_active_slot;               // Last loaded/saved slot (always 0-9)
    uint8_t  include_pins;                   // Whether preset load restores pin config

    // Slot metadata
    uint16_t slot_occupied;                  // Bitmask: bit N = slot N has valid data
    uint8_t  master_volume_mode;             // MASTER_VOLUME_MODE_INDEPENDENT or _WITH_PRESET (was include_master_volume)
    uint8_t  spdif_rx_pin;                   // SPDIF RX GPIO pin, device-level (was padding[1])
    float    master_volume_db;               // Independent master volume (mode 0 at boot)
    char     slot_names[PRESET_SLOTS][PRESET_NAME_LEN];  // 32-byte NUL-terminated names
} PresetDirectory;

#define DIR_VERSION_CURRENT  2

// --- Preset Slot (sectors 1-10) ---
typedef struct __attribute__((packed)) {
    uint32_t magic;                          // SLOT_MAGIC
    uint16_t version;                        // Data format version (matches SLOT_DATA_VERSION)
    uint16_t slot_index;                     // Which slot this is (sanity check)
    uint32_t crc32;                          // CRC over data section

    // ====== DSP State (same fields as legacy FlashStorage, same order) ======
    EqParamPacket filter_recipes[NUM_CHANNELS][MAX_BANDS];
    float preamp_db;
    uint8_t bypass;
    uint8_t padding[3];
    float delays_ms[NUM_CHANNELS];
    // Legacy per-channel gain/mute (V2)
    float channel_gain_db[3];
    uint8_t channel_mute[3];
    uint8_t padding2;
    // Loudness (V3)
    uint8_t loudness_enabled;
    uint8_t padding3[3];
    float loudness_ref_spl;
    float loudness_intensity_pct;
    // Crossfeed (V4)
    uint8_t crossfeed_enabled;
    uint8_t crossfeed_preset;
    uint8_t crossfeed_itd_enabled;
    uint8_t padding4;
    float crossfeed_custom_fc;
    float crossfeed_custom_feed_db;
    // Matrix mixer (V5)
    FlashMatrixCrosspoint matrix_crosspoints[NUM_INPUT_CHANNELS][NUM_OUTPUT_CHANNELS];
    FlashOutputChannel matrix_outputs[NUM_OUTPUT_CHANNELS];
    // Pin configuration (V6) — always stored, conditionally loaded
    uint8_t output_pins[NUM_PIN_OUTPUTS];
    uint8_t pin_padding[8 - NUM_PIN_OUTPUTS];
    // Channel names (V8)
    char channel_names[NUM_CHANNELS][PRESET_NAME_LEN];
    // I2S output configuration (V9)
    uint8_t output_types[4];     // Per-slot type: 0=S/PDIF, 1=I2S (padded to 4)
    uint8_t i2s_bck_pin;         // BCK GPIO; LRCLK = BCK + 1
    uint8_t i2s_mck_pin;         // MCK GPIO
    uint8_t i2s_mck_enabled;     // MCK on/off (0 or 1)
    uint8_t i2s_mck_multiplier;  // MCK = multiplier × Fs (128 or 256)
    // Volume Leveller (V10)
    uint8_t leveller_enabled;
    uint8_t leveller_speed;
    uint8_t leveller_lookahead;
    uint8_t leveller_padding;
    float   leveller_amount;
    float   leveller_max_gain_db;
    float   leveller_gate_threshold_db;
    // Per-channel preamp + Master volume (V12)
    float   preamp_db_per_ch[NUM_INPUT_CHANNELS];  // Per-input-channel preamp (dB)
    float   master_volume_db;                       // Device master volume (-128 mute, -127..0 dB)
    // Input source selection (V13)
    uint8_t input_source;            // InputSource enum (0=USB, 1=SPDIF)
    uint8_t input_source_padding[3]; // Pad to 4-byte boundary
} PresetSlot;

// --- Legacy single-sector format (for migration) ---
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t crc32;
    EqParamPacket filter_recipes[NUM_CHANNELS][MAX_BANDS];
    float preamp_db;
    uint8_t bypass;
    uint8_t padding[3];
    float delays_ms[NUM_CHANNELS];
    float channel_gain_db[3];
    uint8_t channel_mute[3];
    uint8_t padding2;
    uint8_t loudness_enabled;
    uint8_t padding3[3];
    float loudness_ref_spl;
    float loudness_intensity_pct;
    uint8_t crossfeed_enabled;
    uint8_t crossfeed_preset;
    uint8_t crossfeed_itd_enabled;
    uint8_t padding4;
    float crossfeed_custom_fc;
    float crossfeed_custom_feed_db;
    FlashMatrixCrosspoint matrix_crosspoints[NUM_INPUT_CHANNELS][NUM_OUTPUT_CHANNELS];
    FlashOutputChannel matrix_outputs[NUM_OUTPUT_CHANNELS];
    uint8_t output_pins[NUM_PIN_OUTPUTS];
    uint8_t pin_padding[8 - NUM_PIN_OUTPUTS];
} LegacyFlashStorage;

// ============================================================================
// EXTERNAL VARIABLES (defined in usb_audio.c / dsp_pipeline.c)
// ============================================================================

extern volatile float global_preamp_db[NUM_INPUT_CHANNELS];
extern volatile int32_t global_preamp_mul[NUM_INPUT_CHANNELS];
extern volatile float global_preamp_linear[NUM_INPUT_CHANNELS];
extern volatile float master_volume_db;
extern volatile float master_volume_linear;
extern volatile int32_t master_volume_q15;
// Defined in usb_audio.c — clamps, emits v1/v2 host notifications.
extern void update_master_volume(float db);
extern volatile float channel_gain_db[3];
extern volatile int32_t channel_gain_mul[3];
extern volatile bool channel_mute[3];
extern volatile bool loudness_enabled;
extern volatile float loudness_ref_spl;
extern volatile float loudness_intensity_pct;
extern volatile bool loudness_recompute_pending;
extern volatile CrossfeedConfig crossfeed_config;
extern volatile bool crossfeed_update_pending;
extern volatile LevellerConfig leveller_config;
extern volatile bool leveller_update_pending;
extern volatile bool leveller_reset_pending;
extern MatrixMixer matrix_mixer;
extern uint8_t output_pins[NUM_PIN_OUTPUTS];
extern char channel_names[NUM_CHANNELS][PRESET_NAME_LEN];
extern volatile uint32_t feedback_10_14;
extern volatile uint32_t nominal_feedback_10_14;
extern usb_feedback_ctrl_t fb_ctrl;

// ============================================================================
// MODULE STATE
// ============================================================================

// Audio mute flag for glitch-free preset switching
volatile bool preset_loading = false;
volatile uint32_t preset_mute_counter = 0;

// Forward declaration — defined in LEGACY API section
static void apply_factory_defaults(void);

// RAM-cached copy of the directory — updated on every directory write and
// loaded once at boot.  Avoids repeated flash reads for queries.
static PresetDirectory dir_cache;
static bool dir_cache_valid = false;

// Flash mute hold time in samples (rate-aware).
//
// A fixed sample count shrinks in real time at higher rates (e.g. 96 kHz),
// which can be too short to cover post-flash pipeline refill and envelope
// transitions.  Keep the hold at roughly 10 ms across rates with a floor
// that preserves existing behavior at 48 kHz.
static inline uint32_t flash_mute_hold_samples(void) {
    uint64_t samples = ((uint64_t)audio_state.freq * 10u + 999u) / 1000u;
    if (samples < 512u) samples = 512u;
    return (uint32_t)samples;
}

// ============================================================================
// CRC32 (polynomial 0xEDB88320, same as legacy implementation)
// ============================================================================

static uint32_t crc32(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
        }
    }
    return ~crc;
}

// ============================================================================
// dB-TO-LINEAR CONVERSION (no powf dependency in flash context)
// ============================================================================

// Compute 10^(db/20).  An earlier 4-term Taylor approximation here was
// catastrophically wrong beyond ~±10 dB (at -60 dB it returned ~58 instead
// of 0.001, producing loud/distorted output after preset load with any
// deeply-negative gain — see preamp/gain/matrix paths below).  powf is fine:
// this function runs from flash after XIP is up, no RAM-residency constraint.
static float db_to_linear(float db) {
    if (db <= -120.0f) return 0.0f;
    if (db >=  +80.0f) db = 80.0f;
    return powf(10.0f, db / 20.0f);
}

// ============================================================================
// LOW-LEVEL FLASH HELPERS
// ============================================================================

// Erase one sector and write data into it.
// `offset` is the byte offset from the start of flash (not XIP address).
// `data` and `len` specify the payload; it is zero-padded up to page alignment.
static int flash_write_sector(uint32_t offset, const void *data, size_t len) {
    // Round up to page boundary
    size_t write_size = (len + FLASH_PAGE_SIZE - 1) & ~(FLASH_PAGE_SIZE - 1);

    // Use a page-aligned scratch buffer (static to avoid large stack allocs)
    static uint8_t __attribute__((aligned(256))) write_buf[FLASH_SECTOR_SIZE];
    memset(write_buf, 0xFF, sizeof(write_buf));
    memcpy(write_buf, data, len);

    // NOTE: earlier versions drained the SPDIF RX FIFO here via a
    // `while (spdif_input_poll() > 0)` loop.  That triggered full DSP
    // pipeline processing (including a Core 1 work dispatch + wait) inside
    // flash_write_sector, which introduced a crash path on preset_save
    // with SPDIF as the active input source.  The pre-blackout drain now
    // lives only in prepare_flash_write_operation()'s settle loop, which
    // runs once per top-level flash operation; multi-write operations
    // (preset_save = slot + dir) rely on the SPDIF RX library's own
    // overflow handling during the brief inter-write window.
    //
    // Park Core 1 in RAM before quiescing XIP for flash erase/program.
    // Guarded: (a) victim_is_initialized handles first-boot (Core 1 not
    // launched yet) and launch-to-init race; (b) __get_current_exception
    // skips lockout in IRQ context (USB vendor handler) where SDK lock
    // internals are unsafe — IRQ callers rely on copy_to_ram build for
    // XIP safety (see CMakeLists.txt:38).
    bool do_lockout = multicore_lockout_victim_is_initialized(1)
                      && (__get_current_exception() == 0);
    if (do_lockout) multicore_lockout_start_blocking();

    uint32_t flags = save_and_disable_interrupts();
    dspi_flash_range_erase(offset, FLASH_SECTOR_SIZE);
    dspi_flash_range_program(offset, write_buf, write_size);
    restore_interrupts(flags);

    if (do_lockout) multicore_lockout_end_blocking();

    // Re-seed USB feedback controller after ~45ms interrupt blackout
    // (see preset_bugfix.md for details)
    fb_ctrl_reset(&fb_ctrl, nominal_feedback_10_14 << 2);
    feedback_10_14 = nominal_feedback_10_14;

    // Re-arm mute to cover SPDIF consumer pool refill (~4-8ms)
    preset_mute_counter = flash_mute_hold_samples();
    preset_loading = true;

    // Verify magic survived the write
    const uint32_t *verify = (const uint32_t *)(XIP_BASE + offset);
    const uint32_t *expected = (const uint32_t *)data;
    if (*verify != *expected) {
        return -1;  // Write verification failed
    }
    return 0;
}

// ============================================================================
// DIRECTORY MANAGEMENT
// ============================================================================

// Load the directory from flash into the RAM cache.
// Returns true if a valid directory was found.  Transparently migrates a v1
// directory to v2 on first boot of new firmware, preserving slot names,
// startup config, and the old include_master_volume flag (which maps 1:1 to
// master_volume_mode).  The independent master_volume_db defaults to
// MASTER_VOL_MAX_DB so boot-time audible behavior is unchanged post-upgrade.
static int dir_flush(void);  // forward decl — migration calls it
static bool dir_load_cache(void) {
    const PresetDirectory *flash_dir = DIR_ADDR;
    if (flash_dir->magic != DIR_MAGIC) {
        dir_cache_valid = false;
        return false;
    }

    if (flash_dir->version == DIR_VERSION_CURRENT) {
        // Current format — CRC covers everything after the 12-byte header.
        const uint8_t *data_start = (const uint8_t *)&flash_dir->startup_mode;
        size_t data_len = sizeof(PresetDirectory) - offsetof(PresetDirectory, startup_mode);
        if (crc32(data_start, data_len) != flash_dir->crc32) {
            dir_cache_valid = false;
            return false;
        }
        memcpy(&dir_cache, flash_dir, sizeof(dir_cache));
        dir_cache_valid = true;
        return true;
    }

    if (flash_dir->version == 1) {
        // Legacy v1 format — read with the old struct, validate old CRC,
        // then migrate to v2 in memory and flush.
        const PresetDirectory_v1 *v1 = (const PresetDirectory_v1 *)flash_dir;
        const uint8_t *v1_data_start = (const uint8_t *)&v1->startup_mode;
        size_t v1_data_len = sizeof(PresetDirectory_v1) - offsetof(PresetDirectory_v1, startup_mode);
        if (crc32(v1_data_start, v1_data_len) != v1->crc32) {
            dir_cache_valid = false;
            return false;
        }
        memset(&dir_cache, 0, sizeof(dir_cache));
        dir_cache.startup_mode       = v1->startup_mode;
        dir_cache.default_slot       = v1->default_slot;
        dir_cache.last_active_slot   = v1->last_active_slot;
        dir_cache.include_pins       = v1->include_pins;
        dir_cache.slot_occupied      = v1->slot_occupied;
        dir_cache.master_volume_mode = v1->include_master_volume
                                         ? MASTER_VOLUME_MODE_WITH_PRESET
                                         : MASTER_VOLUME_MODE_INDEPENDENT;
        dir_cache.master_volume_db   = MASTER_VOL_MAX_DB;
        memcpy(dir_cache.slot_names, v1->slot_names, sizeof(dir_cache.slot_names));
        dir_cache_valid = true;
        (void)dir_flush();  // persist as v2; if the flush fails, cache stays valid in RAM
        return true;
    }

    // Unknown future version — treat as invalid.
    dir_cache_valid = false;
    return false;
}

// Write the RAM-cached directory back to flash.
// Recomputes the CRC before writing.
static int dir_flush(void) {
    dir_cache.magic = DIR_MAGIC;
    dir_cache.version = DIR_VERSION_CURRENT;
    dir_cache.reserved = 0;
    // CRC covers everything after the 12-byte header (magic + version + reserved + crc32)
    const uint8_t *data_start = (const uint8_t *)&dir_cache.startup_mode;
    size_t data_len = sizeof(PresetDirectory) - offsetof(PresetDirectory, startup_mode);
    dir_cache.crc32 = crc32(data_start, data_len);

    if (flash_write_sector(DIR_SECTOR_OFFSET, &dir_cache, sizeof(dir_cache)) != 0) {
        return -1;
    }
    return 0;
}

// Ensure the directory cache is populated.  If no directory exists on flash,
// initialize a fresh one with factory-default settings.
static void dir_ensure(void) {
    if (dir_cache_valid) return;
    if (dir_load_cache()) return;

    // No valid directory — create a fresh one
    memset(&dir_cache, 0, sizeof(dir_cache));
    dir_cache.startup_mode = PRESET_STARTUP_SPECIFIED;
    dir_cache.default_slot = 0;
    dir_cache.last_active_slot = 0;     // Default to slot 0
    dir_cache.include_pins = 1;              // Include pins in preset load by default
    dir_cache.master_volume_mode = MASTER_VOLUME_MODE_INDEPENDENT;
    dir_cache.master_volume_db   = MASTER_VOL_MAX_DB;
    dir_cache.spdif_rx_pin = PICO_SPDIF_RX_PIN_DEFAULT;
    dir_cache.slot_occupied = 0;             // All slots empty
    // Slot 0 gets a default name; others are empty (already zeroed by memset)
    strncpy(dir_cache.slot_names[0], "Default", PRESET_NAME_LEN - 1);
    dir_cache_valid = true;
    // Don't flush yet — will be flushed on first preset save
}

// ============================================================================
// COLLECT / APPLY DSP STATE
// ============================================================================

// Snapshot the current live DSP state into a PresetSlot structure.
static void collect_live_state(PresetSlot *slot, uint8_t slot_index) {
    memset(slot, 0, sizeof(*slot));

    slot->magic = SLOT_MAGIC;
    slot->version = SLOT_DATA_VERSION;
    slot->slot_index = slot_index;

    // EQ
    memcpy(slot->filter_recipes, (void *)filter_recipes, sizeof(slot->filter_recipes));

    // Preamp — legacy field stores channel 0 for backward compat
    slot->preamp_db = global_preamp_db[0];

    // Bypass
    slot->bypass = bypass_master_eq ? 1 : 0;

    // Delays
    memcpy(slot->delays_ms, (void *)channel_delays_ms, sizeof(slot->delays_ms));

    // Legacy per-channel gain/mute
    memcpy(slot->channel_gain_db, (void *)channel_gain_db, sizeof(slot->channel_gain_db));
    for (int i = 0; i < 3; i++)
        slot->channel_mute[i] = channel_mute[i] ? 1 : 0;

    // Loudness
    slot->loudness_enabled = loudness_enabled ? 1 : 0;
    slot->loudness_ref_spl = loudness_ref_spl;
    slot->loudness_intensity_pct = loudness_intensity_pct;

    // Crossfeed
    slot->crossfeed_enabled = crossfeed_config.enabled ? 1 : 0;
    slot->crossfeed_preset = crossfeed_config.preset;
    slot->crossfeed_itd_enabled = crossfeed_config.itd_enabled ? 1 : 0;
    slot->crossfeed_custom_fc = crossfeed_config.custom_fc;
    slot->crossfeed_custom_feed_db = crossfeed_config.custom_feed_db;

    // Matrix mixer
    for (int in = 0; in < NUM_INPUT_CHANNELS; in++) {
        for (int out = 0; out < NUM_OUTPUT_CHANNELS; out++) {
            slot->matrix_crosspoints[in][out].enabled = matrix_mixer.crosspoints[in][out].enabled;
            slot->matrix_crosspoints[in][out].phase_invert = matrix_mixer.crosspoints[in][out].phase_invert;
            slot->matrix_crosspoints[in][out].gain_db = matrix_mixer.crosspoints[in][out].gain_db;
        }
    }
    for (int out = 0; out < NUM_OUTPUT_CHANNELS; out++) {
        slot->matrix_outputs[out].enabled = matrix_mixer.outputs[out].enabled;
        slot->matrix_outputs[out].mute = matrix_mixer.outputs[out].mute;
        slot->matrix_outputs[out].gain_db = matrix_mixer.outputs[out].gain_db;
        slot->matrix_outputs[out].delay_ms = matrix_mixer.outputs[out].delay_ms;
    }

    // Pin configuration (always stored, conditionally loaded)
    memcpy(slot->output_pins, output_pins, sizeof(slot->output_pins));

    // Channel names
    memcpy(slot->channel_names, channel_names, sizeof(slot->channel_names));

    // I2S configuration (V9)
    extern uint8_t output_types[];
    extern uint8_t i2s_bck_pin;
    extern uint8_t i2s_mck_pin;
    extern bool    i2s_mck_enabled;
    extern uint16_t i2s_mck_multiplier;
    memcpy(slot->output_types, output_types, NUM_SPDIF_INSTANCES);
    // Zero-pad remaining entries (RP2040 has 2 slots, array is 4)
    for (int i = NUM_SPDIF_INSTANCES; i < 4; i++) slot->output_types[i] = 0;
    slot->i2s_bck_pin = i2s_bck_pin;
    slot->i2s_mck_pin = i2s_mck_pin;
    slot->i2s_mck_enabled = i2s_mck_enabled ? 1 : 0;
    slot->i2s_mck_multiplier = (i2s_mck_multiplier == 256) ? 1 : 0;  // 0=128x, 1=256x

    // Volume Leveller (V10)
    slot->leveller_enabled = leveller_config.enabled ? 1 : 0;
    slot->leveller_speed = leveller_config.speed;
    slot->leveller_lookahead = leveller_config.lookahead ? 1 : 0;
    slot->leveller_amount = leveller_config.amount;
    slot->leveller_max_gain_db = leveller_config.max_gain_db;
    slot->leveller_gate_threshold_db = leveller_config.gate_threshold_db;

    // Per-channel preamp + Master volume (V12)
    for (int i = 0; i < NUM_INPUT_CHANNELS; i++)
        slot->preamp_db_per_ch[i] = global_preamp_db[i];
    slot->master_volume_db = master_volume_db;

    // Input source (V13)
    slot->input_source = active_input_source;

    // Compute CRC over the data section (everything after the 12-byte header)
    const uint8_t *data_start = (const uint8_t *)&slot->filter_recipes;
    size_t data_len = sizeof(PresetSlot) - offsetof(PresetSlot, filter_recipes);
    slot->crc32 = crc32(data_start, data_len);
}

// Apply a dB value to the live master volume globals.  NaN/Inf falls back
// to unity so a stale/garbage slot field can't silently mute the device.
// Delegates to update_master_volume() (in usb_audio.c) for the actual
// clamp + globals update + host notification, keeping a single canonical
// path for any master-volume change.
static void apply_master_volume_db(float db) {
    if (!isfinite(db)) db = MASTER_VOL_MAX_DB;
    update_master_volume(db);
}

// Decide which dB value to apply to master volume based on current mode.
//   mode 0 (independent): use dir_cache.master_volume_db, which was set
//     either by a prior REQ_SAVE_MASTER_VOLUME or by fresh-directory init.
//   mode 1 (per-preset):  use slot->master_volume_db if available (V12+).
//     Falls back to the directory value for older slots so we never leave
//     the live globals at a stale value from a previous load.
// `slot_or_null` may be NULL (e.g. factory-defaults path with no slot).
static void apply_master_volume_from_mode(const PresetSlot *slot_or_null) {
    float db;
    if (dir_cache.master_volume_mode == MASTER_VOLUME_MODE_WITH_PRESET
        && slot_or_null && slot_or_null->version >= 12) {
        db = slot_or_null->master_volume_db;
    } else {
        db = dir_cache.master_volume_db;
    }
    apply_master_volume_db(db);
}

// Apply a validated PresetSlot to the live DSP state.
// `include_pins` controls whether pin config is restored.
// Master volume is *not* touched here — callers invoke
// apply_master_volume_from_mode() separately after this returns, so mode
// changes don't require a preset reload to take effect.
// Does NOT trigger filter recalculation — caller must do that.
static void apply_slot_to_live(const PresetSlot *slot, bool include_pins) {
    // EQ
    memcpy((void *)filter_recipes, slot->filter_recipes, sizeof(filter_recipes));

    // Preamp — V12+ has per-channel values, older versions use single legacy field
    if (slot->version >= 12) {
        for (int i = 0; i < NUM_INPUT_CHANNELS; i++) {
            global_preamp_db[i] = slot->preamp_db_per_ch[i];
            float linear = db_to_linear(slot->preamp_db_per_ch[i]);
            global_preamp_mul[i] = (int32_t)(linear * (float)(1 << 28));
            global_preamp_linear[i] = linear;
        }
    } else {
        // Legacy: apply single preamp_db to all channels
        for (int i = 0; i < NUM_INPUT_CHANNELS; i++) {
            global_preamp_db[i] = slot->preamp_db;
            float linear = db_to_linear(slot->preamp_db);
            global_preamp_mul[i] = (int32_t)(linear * (float)(1 << 28));
            global_preamp_linear[i] = linear;
        }
    }

    // Master volume is applied separately by callers via
    // apply_master_volume_from_mode() so a mode change takes effect without
    // a preset reload.
    // Bypass
    bypass_master_eq = (slot->bypass != 0);

    // Delays
    memcpy((void *)channel_delays_ms, slot->delays_ms, sizeof(channel_delays_ms));

    // Legacy per-channel gain/mute
    for (int i = 0; i < 3; i++) {
        channel_gain_db[i] = slot->channel_gain_db[i];
        float g = db_to_linear(slot->channel_gain_db[i]);
        channel_gain_mul[i] = (int32_t)(g * 32768.0f);
        channel_mute[i] = (slot->channel_mute[i] != 0);
    }

    // Loudness
    loudness_enabled = (slot->loudness_enabled != 0);
    loudness_ref_spl = slot->loudness_ref_spl;
    loudness_intensity_pct = slot->loudness_intensity_pct;
    loudness_recompute_pending = true;

    // Crossfeed
    crossfeed_config.enabled = (slot->crossfeed_enabled != 0);
    crossfeed_config.preset = slot->crossfeed_preset;
    crossfeed_config.itd_enabled = (slot->crossfeed_itd_enabled != 0);
    crossfeed_config.custom_fc = slot->crossfeed_custom_fc;
    crossfeed_config.custom_feed_db = slot->crossfeed_custom_feed_db;
    crossfeed_update_pending = true;

    // Matrix mixer
    for (int in = 0; in < NUM_INPUT_CHANNELS; in++) {
        for (int out = 0; out < NUM_OUTPUT_CHANNELS; out++) {
            matrix_mixer.crosspoints[in][out].enabled = slot->matrix_crosspoints[in][out].enabled;
            matrix_mixer.crosspoints[in][out].phase_invert = slot->matrix_crosspoints[in][out].phase_invert;
            matrix_mixer.crosspoints[in][out].gain_db = slot->matrix_crosspoints[in][out].gain_db;
            matrix_mixer.crosspoints[in][out].gain_linear = db_to_linear(slot->matrix_crosspoints[in][out].gain_db);
        }
    }
    for (int out = 0; out < NUM_OUTPUT_CHANNELS; out++) {
        matrix_mixer.outputs[out].enabled = slot->matrix_outputs[out].enabled;
        matrix_mixer.outputs[out].mute = slot->matrix_outputs[out].mute;
        matrix_mixer.outputs[out].gain_db = slot->matrix_outputs[out].gain_db;
        matrix_mixer.outputs[out].gain_linear = db_to_linear(slot->matrix_outputs[out].gain_db);
        matrix_mixer.outputs[out].delay_ms = slot->matrix_outputs[out].delay_ms;
        channel_delays_ms[CH_OUT_1 + out] = slot->matrix_outputs[out].delay_ms;
    }

    // Pin configuration (conditional)
    if (include_pins) {
#if PICO_RP2350
        static const uint8_t default_pins[NUM_PIN_OUTPUTS] = {
            PICO_AUDIO_SPDIF_PIN, PICO_SPDIF_PIN_2,
            PICO_SPDIF_PIN_3, PICO_SPDIF_PIN_4, PICO_PDM_PIN
        };
#else
        static const uint8_t default_pins[NUM_PIN_OUTPUTS] = {
            PICO_AUDIO_SPDIF_PIN, PICO_SPDIF_PIN_2, PICO_PDM_PIN
        };
#endif
        for (int i = 0; i < NUM_PIN_OUTPUTS; i++) {
            uint8_t pin = slot->output_pins[i];
            bool valid = (pin <= 29) && (pin != 12) && !(pin >= 23 && pin <= 25);
#if !PICO_RP2350
            if (pin > 28) valid = false;
#endif
            output_pins[i] = valid ? pin : default_pins[i];
        }
    }

    // Channel names (V8+)
    if (slot->version >= 8) {
        memcpy(channel_names, slot->channel_names, sizeof(channel_names));
    } else {
        for (int ch = 0; ch < NUM_CHANNELS; ch++)
            get_default_channel_name(ch, channel_names[ch]);
    }

    // I2S configuration (V9+)
    {
        extern uint8_t output_types[];
        extern uint8_t i2s_bck_pin;
        extern uint8_t i2s_mck_pin;
        extern bool    i2s_mck_enabled;
        extern uint16_t i2s_mck_multiplier;
        if (slot->version >= 9) {
            memcpy(output_types, slot->output_types, NUM_SPDIF_INSTANCES);
            i2s_bck_pin = slot->i2s_bck_pin;
            i2s_mck_pin = slot->i2s_mck_pin;
            i2s_mck_enabled = (slot->i2s_mck_enabled != 0);
            if (slot->version >= 11) {
                // V11+: 0=128x, 1=256x
                i2s_mck_multiplier = (slot->i2s_mck_multiplier == 1) ? 256 : 128;
            } else {
                // V9-V10: raw value stored (128 or 0 for 256)
                i2s_mck_multiplier = (slot->i2s_mck_multiplier == 0) ? 256 : slot->i2s_mck_multiplier;
            }
        } else {
            // Default: all S/PDIF, no I2S/MCK
            memset(output_types, 0, NUM_SPDIF_INSTANCES);
            i2s_bck_pin = PICO_I2S_BCK_PIN;
            i2s_mck_pin = PICO_I2S_MCK_PIN;
            i2s_mck_enabled = false;
            i2s_mck_multiplier = 128;
        }
    }

    // Volume Leveller (V10+)
    if (slot->version >= 10) {
        leveller_config.enabled = (slot->leveller_enabled != 0);
        leveller_config.speed = slot->leveller_speed;
        leveller_config.lookahead = (slot->leveller_lookahead != 0);
        leveller_config.amount = slot->leveller_amount;
        leveller_config.max_gain_db = slot->leveller_max_gain_db;
        leveller_config.gate_threshold_db = slot->leveller_gate_threshold_db;
    } else {
        leveller_config.enabled = LEVELLER_DEFAULT_ENABLED;
        leveller_config.amount = LEVELLER_DEFAULT_AMOUNT;
        leveller_config.speed = LEVELLER_DEFAULT_SPEED;
        leveller_config.max_gain_db = LEVELLER_DEFAULT_MAX_GAIN_DB;
        leveller_config.lookahead = LEVELLER_DEFAULT_LOOKAHEAD;
        leveller_config.gate_threshold_db = LEVELLER_DEFAULT_GATE_DB;
    }
    leveller_update_pending = true;
    leveller_reset_pending = true;

    // Input source (V13+)
    if (slot->version >= 13) {
        uint8_t src = slot->input_source;
        if (input_source_valid(src) && src != active_input_source) {
            pending_input_source = src;
            __dmb();
            input_source_change_pending = true;
        }
    }
    // V12 and earlier: leave input source at current value (USB by default)
}

// ============================================================================
// SLOT VALIDATION
// ============================================================================

// Read and validate a preset slot from flash.
// Returns a pointer to the flash-mapped slot if valid, NULL otherwise.
static const PresetSlot *validate_slot(uint8_t slot) {
    const PresetSlot *s = SLOT_ADDR(slot);
    if (s->magic != SLOT_MAGIC) return NULL;
    if (s->slot_index != slot) return NULL;
    // CRC check
    const uint8_t *data_start = (const uint8_t *)&s->filter_recipes;
    size_t data_len = sizeof(PresetSlot) - offsetof(PresetSlot, filter_recipes);
    if (crc32(data_start, data_len) != s->crc32) return NULL;
    return s;
}

// ============================================================================
// PUBLIC PRESET API
// ============================================================================

uint8_t preset_save(uint8_t slot) {
    if (slot >= PRESET_SLOTS) return PRESET_ERR_INVALID_SLOT;

    dir_ensure();

    // Build the slot data from current live state
    static PresetSlot slot_buf;
    collect_live_state(&slot_buf, slot);

    // Engage mute before flash writes to prevent audio glitches
    preset_mute_counter = flash_mute_hold_samples();
    preset_loading = true;
    __dmb();

    // Write slot to flash
    if (flash_write_sector(SLOT_SECTOR_OFFSET(slot), &slot_buf, sizeof(slot_buf)) != 0) {
        return PRESET_ERR_FLASH_WRITE;
    }

    // Update directory: mark occupied, set last active
    dir_cache.slot_occupied |= (1u << slot);
    dir_cache.last_active_slot = slot;
    if (dir_flush() != 0) {
        return PRESET_ERR_FLASH_WRITE;
    }

    return PRESET_OK;
}

uint8_t preset_load(uint8_t slot) {
    if (slot >= PRESET_SLOTS) return PRESET_ERR_INVALID_SLOT;

    dir_ensure();

    // NOTE: muting is now handled by prepare_pipeline_reset() in the main
    // loop caller, which also waits for Core 1 idle before we modify state.

    // Bracket the wholesale state rewrite so per-field param_write calls
    // are suppressed; notify_end_bulk() emits a single BULK_INVALIDATED.
    // PRESET_LOADED is pushed here (ahead of the bulk) so the host sees
    // the two events in order: preset-loaded, then invalidate.
    notify_push_preset_loaded(slot);
    notify_begin_bulk(PARAM_SRC_PRESET);

    if (dir_cache.slot_occupied & (1u << slot)) {
        // Slot has user data — validate and load it
        const PresetSlot *s = validate_slot(slot);
        if (!s) {
            preset_loading = false;
            notify_end_bulk();
            return PRESET_ERR_CRC;
        }
        apply_slot_to_live(s, dir_cache.include_pins != 0);
        apply_master_volume_from_mode(s);
    } else {
        // Slot not configured — apply factory defaults
        apply_factory_defaults();
    }

    // Recalculate filters and delays for the current sample rate
    extern volatile AudioState audio_state;
    float rate = (float)audio_state.freq;
    dsp_recalculate_all_filters(rate);
    dsp_update_delay_samples(rate);

    // Zero all delay line buffers.  Without this, stale audio from the
    // previous preset's delay lines bleeds through — e.g. switching from
    // a 40ms delay to 0ms would replay ~40ms of old audio as the write
    // index wraps past the old data.
    extern
#if PICO_RP2350
    float delay_lines[NUM_DELAY_CHANNELS][MAX_DELAY_SAMPLES];
#else
    int32_t delay_lines[NUM_DELAY_CHANNELS][MAX_DELAY_SAMPLES];
#endif
    memset(delay_lines, 0, sizeof(delay_lines));

    // Transition Core 1 mode to match the new output enable state
    Core1Mode new_mode = derive_core1_mode();
    if (new_mode != core1_mode) {
        core1_mode = new_mode;
#if ENABLE_SUB
        pdm_set_enabled(new_mode == CORE1_MODE_PDM);
#endif
        __sev();  // Wake Core 1 to pick up mode change
    }

    // Update directory: set last active
    dir_cache.last_active_slot = slot;
    dir_flush();  // Best-effort; preset is already loaded even if dir write fails

    // Close the bulk bracket; emits one BULK_INVALIDATED with source=PRESET.
    notify_end_bulk();
    return PRESET_OK;
}

uint8_t preset_delete(uint8_t slot) {
    if (slot >= PRESET_SLOTS) return PRESET_ERR_INVALID_SLOT;

    dir_ensure();

    // NOTE: muting is now handled by prepare_pipeline_reset() in the main
    // loop caller.  The mute counter and preset_loading flag are set there.
    // See flash_write_sector() for why we no longer drain SPDIF RX FIFO
    // here (was causing preset_save/delete crashes via Core 1 dispatch
    // inside the flash blackout prep).
    __dmb();

    // Erase the slot's flash sector (same lockout guard as flash_write_sector)
    bool do_lockout = multicore_lockout_victim_is_initialized(1)
                      && (__get_current_exception() == 0);
    if (do_lockout) multicore_lockout_start_blocking();

    uint32_t flags = save_and_disable_interrupts();
    dspi_flash_range_erase(SLOT_SECTOR_OFFSET(slot), FLASH_SECTOR_SIZE);
    restore_interrupts(flags);

    if (do_lockout) multicore_lockout_end_blocking();

    // Re-seed feedback controller after interrupt blackout
    fb_ctrl_reset(&fb_ctrl, nominal_feedback_10_14 << 2);
    feedback_10_14 = nominal_feedback_10_14;
    preset_mute_counter = flash_mute_hold_samples();
    preset_loading = true;

    // Update directory — clear occupied bit and name, keep slot selected if active
    dir_cache.slot_occupied &= ~(1u << slot);
    memset(dir_cache.slot_names[slot], 0, PRESET_NAME_LEN);
    dir_flush();

    // If deleting the active slot, apply factory defaults to live state
    if (slot == dir_cache.last_active_slot) {
        preset_mute_counter = PRESET_MUTE_SAMPLES;
        preset_loading = true;
        __dmb();

        apply_factory_defaults();

        extern volatile AudioState audio_state;
        float rate = (float)audio_state.freq;
        dsp_recalculate_all_filters(rate);
        dsp_update_delay_samples(rate);

        // Transition Core 1 mode (factory defaults disable outputs 2+)
        Core1Mode new_mode = derive_core1_mode();
        if (new_mode != core1_mode) {
            core1_mode = new_mode;
#if ENABLE_SUB
            pdm_set_enabled(new_mode == CORE1_MODE_PDM);
#endif
            __sev();
        }
    }

    return PRESET_OK;
}

uint8_t preset_get_name(uint8_t slot, char *name_out) {
    if (slot >= PRESET_SLOTS) return PRESET_ERR_INVALID_SLOT;
    dir_ensure();
    memcpy(name_out, dir_cache.slot_names[slot], PRESET_NAME_LEN);
    return PRESET_OK;
}

uint8_t preset_set_name(uint8_t slot, const char *name) {
    if (slot >= PRESET_SLOTS) return PRESET_ERR_INVALID_SLOT;
    dir_ensure();

    // Copy name with guaranteed NUL termination
    memset(dir_cache.slot_names[slot], 0, PRESET_NAME_LEN);
    strncpy(dir_cache.slot_names[slot], name, PRESET_NAME_LEN - 1);

    if (dir_flush() != 0) {
        return PRESET_ERR_FLASH_WRITE;
    }
    return PRESET_OK;
}

void preset_get_directory(uint16_t *slot_occupied, uint8_t *startup_mode,
                          uint8_t *default_slot, uint8_t *last_active,
                          uint8_t *include_pins, uint8_t *master_volume_mode) {
    dir_ensure();
    *slot_occupied      = dir_cache.slot_occupied;
    *startup_mode       = dir_cache.startup_mode;
    *default_slot       = dir_cache.default_slot;
    *last_active        = dir_cache.last_active_slot;
    *include_pins       = dir_cache.include_pins;
    *master_volume_mode = dir_cache.master_volume_mode;
}

uint8_t preset_set_startup(uint8_t mode, uint8_t default_slot) {
    if (mode > PRESET_STARTUP_LAST_ACTIVE) return PRESET_ERR_INVALID_SLOT;
    if (default_slot >= PRESET_SLOTS) return PRESET_ERR_INVALID_SLOT;
    dir_ensure();

    dir_cache.startup_mode = mode;
    dir_cache.default_slot = default_slot;
    if (dir_flush() != 0) {
        return PRESET_ERR_FLASH_WRITE;
    }
    return PRESET_OK;
}

void preset_set_include_pins(uint8_t include) {
    dir_ensure();
    dir_cache.include_pins = include ? 1 : 0;
    dir_flush();
}

void preset_set_master_volume_mode(uint8_t mode) {
    if (mode > MASTER_VOLUME_MODE_WITH_PRESET) mode = MASTER_VOLUME_MODE_INDEPENDENT;
    dir_ensure();
    dir_cache.master_volume_mode = mode;
    dir_flush();
}

void preset_set_spdif_rx_pin(uint8_t pin) {
    dir_ensure();
    dir_cache.spdif_rx_pin = pin;
    dir_flush();
}

// Copy the live master volume into the directory's independent field and
// persist.  Accepted in both modes — in mode 1 the value is dormant until
// the user switches to mode 0.  Matches the deferred-flush machinery used
// for the other directory-writing setters.
uint8_t preset_save_master_volume(void) {
    dir_ensure();
    dir_cache.master_volume_db = master_volume_db;
    if (dir_flush() != 0) return PRESET_ERR_FLASH_WRITE;
    return PRESET_OK;
}

// Read back the directory's independent master volume field (the value that
// would apply at boot in mode 0).  Does not touch live globals.
float preset_get_saved_master_volume(void) {
    dir_ensure();
    return dir_cache.master_volume_db;
}

uint8_t preset_get_active(void) {
    dir_ensure();
    return dir_cache.last_active_slot;
}

// ============================================================================
// BOOT / MIGRATION
// ============================================================================

// Attempt to migrate legacy single-sector data (pre-preset firmware) into
// preset slot 0.  Called when no preset directory exists on flash.
static bool migrate_legacy(void) {
    const LegacyFlashStorage *legacy = LEGACY_ADDR;
    if (legacy->magic != LEGACY_MAGIC) return false;

    // Verify legacy CRC
    const uint8_t *data_start = (const uint8_t *)&legacy->filter_recipes;
    size_t data_len = sizeof(LegacyFlashStorage) - offsetof(LegacyFlashStorage, filter_recipes);
    if (crc32(data_start, data_len) != legacy->crc32) return false;

    // Build a PresetSlot from the legacy data.
    // The data section layout is identical, so we can memcpy the data portion.
    static PresetSlot slot_buf;
    memset(&slot_buf, 0, sizeof(slot_buf));
    slot_buf.magic = SLOT_MAGIC;
    slot_buf.version = legacy->version;
    slot_buf.slot_index = 0;

    // Copy data fields (identical layout from filter_recipes onward)
    memcpy(&slot_buf.filter_recipes, &legacy->filter_recipes,
           sizeof(LegacyFlashStorage) - offsetof(LegacyFlashStorage, filter_recipes));

    // Recompute CRC for the slot format
    const uint8_t *slot_data = (const uint8_t *)&slot_buf.filter_recipes;
    size_t slot_data_len = sizeof(PresetSlot) - offsetof(PresetSlot, filter_recipes);
    slot_buf.crc32 = crc32(slot_data, slot_data_len);

    // Write slot 0
    if (flash_write_sector(SLOT_SECTOR_OFFSET(0), &slot_buf, sizeof(slot_buf)) != 0) {
        return false;
    }

    // Create a fresh directory with slot 0 occupied and set as default
    memset(&dir_cache, 0, sizeof(dir_cache));
    dir_cache.startup_mode = PRESET_STARTUP_SPECIFIED;
    dir_cache.default_slot = 0;
    dir_cache.last_active_slot = 0;
    dir_cache.include_pins = 1;
    dir_cache.master_volume_mode = MASTER_VOLUME_MODE_INDEPENDENT;
    dir_cache.master_volume_db   = MASTER_VOL_MAX_DB;
    dir_cache.spdif_rx_pin = PICO_SPDIF_RX_PIN_DEFAULT;
    dir_cache.slot_occupied = 0x0001;  // Slot 0 occupied
    strncpy(dir_cache.slot_names[0], "Migrated", PRESET_NAME_LEN - 1);
    dir_cache_valid = true;

    if (dir_flush() != 0) {
        return false;
    }

    return true;
}

int preset_boot_load(void) {
    // Try to load the preset directory from flash
    if (dir_load_cache()) {
        // Restore device-level SPDIF RX pin from directory
        uint8_t rx_pin = dir_cache.spdif_rx_pin;
        if (rx_pin > 0 && rx_pin <= 29 && rx_pin != 12 &&
            !(rx_pin >= 23 && rx_pin <= 25)) {
            spdif_rx_pin = rx_pin;
        } else {
            spdif_rx_pin = PICO_SPDIF_RX_PIN_DEFAULT;
        }

        // Directory exists — determine which slot to load
        uint8_t target_slot;

        if (dir_cache.startup_mode == PRESET_STARTUP_LAST_ACTIVE) {
            target_slot = dir_cache.last_active_slot;
        } else {
            // PRESET_STARTUP_SPECIFIED (default)
            target_slot = dir_cache.default_slot;
        }

        // Clamp to valid range
        if (target_slot >= PRESET_SLOTS) {
            target_slot = dir_cache.default_slot;
            if (target_slot >= PRESET_SLOTS) target_slot = 0;
        }

        // Load the slot: user data if occupied, factory defaults if empty
        if ((dir_cache.slot_occupied & (1u << target_slot))) {
            const PresetSlot *s = validate_slot(target_slot);
            if (s) {
                apply_slot_to_live(s, dir_cache.include_pins != 0);
                apply_master_volume_from_mode(s);
            } else {
                // Corrupt data — fall back to factory defaults
                apply_factory_defaults();
            }
        } else {
            apply_factory_defaults();
        }

        dir_cache.last_active_slot = target_slot;
        return FLASH_OK;
    }

    // No directory — try legacy migration
    if (migrate_legacy()) {
        // Migration succeeded; slot 0 is now populated.  Load it.
        const PresetSlot *s = validate_slot(0);
        if (s) {
            apply_slot_to_live(s, false);  // Legacy migration: don't override pins
            apply_master_volume_from_mode(s);
        } else {
            apply_factory_defaults();
        }
        return FLASH_OK;
    }

    // First boot, no legacy data — initialize directory and use slot 0
    dir_ensure();
    dir_flush();
    apply_factory_defaults();
    return FLASH_OK;
}

// ============================================================================
// LEGACY API (redirects through preset system)
// ============================================================================

int flash_save_params(void) {
    dir_ensure();

    // Determine which slot to save into
    uint8_t slot = dir_cache.last_active_slot;
    if (slot >= PRESET_SLOTS) {
        // No active slot — use slot 0
        slot = 0;
    }

    uint8_t result = preset_save(slot);
    switch (result) {
        case PRESET_OK:             return FLASH_OK;
        case PRESET_ERR_FLASH_WRITE: return FLASH_ERR_WRITE;
        default:                    return FLASH_ERR_WRITE;
    }
}

int flash_load_params(void) {
    dir_ensure();

    uint8_t slot = dir_cache.last_active_slot;
    if (slot >= PRESET_SLOTS) {
        slot = 0;
    }

    uint8_t result = preset_load(slot);
    switch (result) {
        case PRESET_OK:            return FLASH_OK;
        case PRESET_ERR_CRC:       return FLASH_ERR_CRC;
        default:                   return FLASH_ERR_WRITE;
    }
}

// Reset the live DSP state to factory defaults.
// Does NOT modify the directory or active slot tracking.
static void apply_factory_defaults(void) {
    dsp_init_default_filters();

    // Preamp — reset all input channels to unity
    for (int i = 0; i < NUM_INPUT_CHANNELS; i++) {
        global_preamp_db[i]      = 0.0f;
        global_preamp_mul[i]     = (1 << 28);
        global_preamp_linear[i]  = 1.0f;
    }

    // Master volume — defer to the mode-aware helper so mode 0 restores the
    // directory's independent value instead of always stomping to unity.
    // (Mode 1 with slot_or_null=NULL also falls back to the directory value.)
    // Notifications are emitted via update_master_volume() inside
    // apply_master_volume_db().
    apply_master_volume_from_mode(NULL);

    // Bypass
    bypass_master_eq = false;

    // Per-channel gain and mute
    for (int i = 0; i < 3; i++) {
        channel_gain_db[i] = 0.0f;
        channel_gain_mul[i] = 32768;
        channel_mute[i] = false;
    }

    // Loudness
    loudness_enabled = false;
    loudness_ref_spl = 83.0f;
    loudness_intensity_pct = 100.0f;
    loudness_recompute_pending = true;

    // Crossfeed
    crossfeed_config.enabled = false;
    crossfeed_config.itd_enabled = true;
    crossfeed_config.preset = CROSSFEED_PRESET_DEFAULT;
    crossfeed_config.custom_fc = 700.0f;
    crossfeed_config.custom_feed_db = 4.5f;
    crossfeed_update_pending = true;

    // Matrix mixer: stereo pass-through on first pair
    memset(&matrix_mixer, 0, sizeof(matrix_mixer));
    matrix_mixer.crosspoints[0][0].enabled = 1;
    matrix_mixer.crosspoints[0][0].gain_db = 0.0f;
    matrix_mixer.crosspoints[0][0].gain_linear = 1.0f;
    matrix_mixer.crosspoints[1][1].enabled = 1;
    matrix_mixer.crosspoints[1][1].gain_db = 0.0f;
    matrix_mixer.crosspoints[1][1].gain_linear = 1.0f;
    matrix_mixer.outputs[0].enabled = 1;
    matrix_mixer.outputs[0].gain_linear = 1.0f;
    matrix_mixer.outputs[1].enabled = 1;
    matrix_mixer.outputs[1].gain_linear = 1.0f;
    for (int out = 2; out < NUM_OUTPUT_CHANNELS; out++) {
        matrix_mixer.outputs[out].enabled = 0;
        matrix_mixer.outputs[out].gain_linear = 1.0f;
    }

    // Reset pin configuration
    output_pins[0] = PICO_AUDIO_SPDIF_PIN;
    output_pins[1] = PICO_SPDIF_PIN_2;
#if PICO_RP2350
    output_pins[2] = PICO_SPDIF_PIN_3;
    output_pins[3] = PICO_SPDIF_PIN_4;
    output_pins[4] = PICO_PDM_PIN;
#else
    output_pins[2] = PICO_PDM_PIN;
#endif

    // Reset channel names to defaults
    for (int ch = 0; ch < NUM_CHANNELS; ch++)
        get_default_channel_name(ch, channel_names[ch]);

    // Reset I2S configuration
    {
        extern uint8_t output_types[];
        extern uint8_t i2s_bck_pin;
        extern uint8_t i2s_mck_pin;
        extern bool    i2s_mck_enabled;
        extern uint16_t i2s_mck_multiplier;
        memset(output_types, 0, NUM_SPDIF_INSTANCES);  // All S/PDIF
        i2s_bck_pin = PICO_I2S_BCK_PIN;
        i2s_mck_pin = PICO_I2S_MCK_PIN;
        i2s_mck_enabled = false;
        i2s_mck_multiplier = 128;
    }

    // Volume Leveller
    leveller_config.enabled = LEVELLER_DEFAULT_ENABLED;
    leveller_config.amount = LEVELLER_DEFAULT_AMOUNT;
    leveller_config.speed = LEVELLER_DEFAULT_SPEED;
    leveller_config.max_gain_db = LEVELLER_DEFAULT_MAX_GAIN_DB;
    leveller_config.lookahead = LEVELLER_DEFAULT_LOOKAHEAD;
    leveller_config.gate_threshold_db = LEVELLER_DEFAULT_GATE_DB;
    leveller_update_pending = true;
    leveller_reset_pending = true;

    // Input source: default to USB
    active_input_source = INPUT_SOURCE_USB;
}

void flash_factory_reset(void) {
    // Bracket so per-field writes in apply_factory_defaults() are suppressed
    // and the host sees exactly one BULK_INVALIDATED(source=FACTORY).
    notify_begin_bulk(PARAM_SRC_FACTORY);
    apply_factory_defaults();
    notify_end_bulk();
}
