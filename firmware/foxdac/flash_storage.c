#include "flash_storage.h"
#include "config.h"
#include "dsp_pipeline.h"
#include "usb_audio.h"

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#include <string.h>

// Flash configuration - use last 4KB sector
#define FLASH_STORAGE_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define FLASH_MAGIC 0x44535031  // "DSP1"
#define FLASH_VERSION 3

// Pointer to read flash via XIP
#define FLASH_STORAGE_ADDR (XIP_BASE + FLASH_STORAGE_OFFSET)

// Storage structure
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t crc32;
    // Data section
    EqParamPacket filter_recipes[NUM_CHANNELS][MAX_BANDS];
    float preamp_db;
    uint8_t bypass;
    uint8_t padding[3];  // Align to 4 bytes
    float delays_ms[NUM_CHANNELS];
    // V2: Per-channel gain and mute (output channels only)
    float channel_gain_db[3];
    uint8_t channel_mute[3];
    uint8_t padding2;  // Align to 4 bytes
    // V3: Loudness compensation
    uint8_t loudness_enabled;
    uint8_t padding3[3];
    float loudness_ref_spl;
    float loudness_intensity_pct;
} FlashStorage;

// External variables we need to access (defined in usb_audio.c)
extern volatile float global_preamp_db;
extern volatile int32_t global_preamp_mul;
extern volatile float channel_gain_db[3];
extern volatile int32_t channel_gain_mul[3];
extern volatile bool channel_mute[3];
extern volatile bool loudness_enabled;
extern volatile float loudness_ref_spl;
extern volatile float loudness_intensity_pct;
extern volatile bool loudness_recompute_pending;

// Simple CRC32 implementation (polynomial 0xEDB88320)
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

int flash_save_params(void) {
    // Build storage structure in RAM
    static FlashStorage storage;
    memset(&storage, 0, sizeof(storage));

    storage.magic = FLASH_MAGIC;
    storage.version = FLASH_VERSION;
    storage.reserved = 0;

    // Copy parameters
    memcpy(storage.filter_recipes, (void*)filter_recipes, sizeof(storage.filter_recipes));
    storage.preamp_db = global_preamp_db;
    storage.bypass = bypass_master_eq ? 1 : 0;
    memcpy(storage.delays_ms, (void*)channel_delays_ms, sizeof(storage.delays_ms));
    memcpy(storage.channel_gain_db, (void*)channel_gain_db, sizeof(storage.channel_gain_db));
    for (int i = 0; i < 3; i++) storage.channel_mute[i] = channel_mute[i] ? 1 : 0;
    storage.loudness_enabled = loudness_enabled ? 1 : 0;
    storage.loudness_ref_spl = loudness_ref_spl;
    storage.loudness_intensity_pct = loudness_intensity_pct;

    // Compute CRC over data section (everything after the header)
    const uint8_t *data_start = (const uint8_t *)&storage.filter_recipes;
    size_t data_len = sizeof(storage) - offsetof(FlashStorage, filter_recipes);
    storage.crc32 = crc32(data_start, data_len);

    // Round up to page size for writing
    size_t write_size = (sizeof(storage) + FLASH_PAGE_SIZE - 1) & ~(FLASH_PAGE_SIZE - 1);

    // Disable interrupts during flash operations
    uint32_t flags = save_and_disable_interrupts();

    // Erase the sector (required before writing)
    flash_range_erase(FLASH_STORAGE_OFFSET, FLASH_SECTOR_SIZE);

    // Program the data
    flash_range_program(FLASH_STORAGE_OFFSET, (const uint8_t *)&storage, write_size);

    // Restore interrupts
    restore_interrupts(flags);

    // Verify write by reading back and checking magic
    const FlashStorage *verify = (const FlashStorage *)FLASH_STORAGE_ADDR;
    if (verify->magic != FLASH_MAGIC) {
        return FLASH_ERR_WRITE;
    }

    return FLASH_OK;
}

int flash_load_params(void) {
    // Read from flash via XIP pointer
    const FlashStorage *storage = (const FlashStorage *)FLASH_STORAGE_ADDR;

    // Check magic number
    if (storage->magic != FLASH_MAGIC) {
        return FLASH_ERR_NO_DATA;
    }

    // Check version (for future compatibility)
    if (storage->version > FLASH_VERSION) {
        return FLASH_ERR_NO_DATA;  // Newer version, can't load
    }

    // Verify CRC
    const uint8_t *data_start = (const uint8_t *)&storage->filter_recipes;
    size_t data_len = sizeof(FlashStorage) - offsetof(FlashStorage, filter_recipes);
    uint32_t computed_crc = crc32(data_start, data_len);
    if (computed_crc != storage->crc32) {
        return FLASH_ERR_CRC;
    }

    // Load parameters into global variables
    memcpy((void*)filter_recipes, storage->filter_recipes, sizeof(filter_recipes));

    // Set preamp (need to compute multiplier too)
    global_preamp_db = storage->preamp_db;
    float linear = 1.0f;
    // Compute 10^(db/20) manually to avoid powf dependency issues
    if (storage->preamp_db != 0.0f) {
        // Use exp approximation: 10^x = e^(x * ln(10))
        // For small values, use simple approximation
        float db = storage->preamp_db;
        // Clamp to reasonable range
        if (db < -60.0f) db = -60.0f;
        if (db > 20.0f) db = 20.0f;
        // 10^(db/20) = e^(db * ln(10) / 20) = e^(db * 0.1151)
        float x = db * 0.1151292546f;  // ln(10)/20
        // Taylor series for e^x (good enough for our range)
        linear = 1.0f + x + x*x*0.5f + x*x*x*0.1666667f + x*x*x*x*0.0416667f;
        if (linear < 0.0f) linear = 0.0f;
    }
    global_preamp_mul = (int32_t)(linear * (float)(1 << 28));

    bypass_master_eq = (storage->bypass != 0);

    memcpy((void*)channel_delays_ms, storage->delays_ms, sizeof(channel_delays_ms));

    // V2: Per-channel gain and mute
    if (storage->version >= 2) {
        for (int i = 0; i < 3; i++) {
            channel_gain_db[i] = storage->channel_gain_db[i];
            // Compute 10^(db/20) using Taylor series (same as preamp above)
            float linear = 1.0f;
            float db = storage->channel_gain_db[i];
            if (db != 0.0f) {
                if (db < -60.0f) db = -60.0f;
                if (db > 20.0f) db = 20.0f;
                float x = db * 0.1151292546f;  // ln(10)/20
                linear = 1.0f + x + x*x*0.5f + x*x*x*0.1666667f + x*x*x*x*0.0416667f;
                if (linear < 0.0f) linear = 0.0f;
            }
            channel_gain_mul[i] = (int32_t)(linear * 32768.0f);
            channel_mute[i] = (storage->channel_mute[i] != 0);
        }
    }

    // V3: Loudness compensation
    if (storage->version >= 3) {
        loudness_enabled = (storage->loudness_enabled != 0);
        loudness_ref_spl = storage->loudness_ref_spl;
        loudness_intensity_pct = storage->loudness_intensity_pct;
        loudness_recompute_pending = true;
    }

    return FLASH_OK;
}

void flash_factory_reset(void) {
    // Simply reinitialize to defaults - does NOT erase flash
    dsp_init_default_filters();

    // Reset preamp to 0 dB
    global_preamp_db = 0.0f;
    global_preamp_mul = (1 << 28);  // Unity gain

    // Reset bypass
    bypass_master_eq = false;

    // Reset per-channel gain and mute
    for (int i = 0; i < 3; i++) {
        channel_gain_db[i] = 0.0f;
        channel_gain_mul[i] = 32768;  // Unity = 2^15
        channel_mute[i] = false;
    }

    // Reset loudness compensation
    loudness_enabled = false;
    loudness_ref_spl = 83.0f;
    loudness_intensity_pct = 100.0f;
    loudness_recompute_pending = true;
}
