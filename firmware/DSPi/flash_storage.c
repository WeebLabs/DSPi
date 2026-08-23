/*
 * flash_storage.c — Preset-based parameter persistence for DSPi
 *
 * Flash Layout (from end of flash, working backwards):
 *
 *   Sector 0:            Preset Directory  (metadata, slot names, startup config)
 *   Sectors 1..:         10 Preset Slots, SLOT_SECTORS each (2 sectors / 8 KB on
 *                        RP2350, 1 sector / 4 KB on RP2040) — full DSP snapshots
 *
 * Region = 1 + 10·SLOT_SECTORS sectors (21 on RP2350, 11 on RP2040).  A V21
 * slot is ~5.8 KB on RP2350 (17-channel EQ/names/crossover), which exceeds one
 * 4 KB sector — hence 2 sectors per slot there.  There is NO legacy sector:
 * the old single-preset format and its migration were removed at V21 (compat
 * broken intentionally; pre-V21 flash loads factory defaults).
 *
 * A preset slot stores the complete user-configurable DSP state: per-channel
 * EQ bands, preamp, delays, loudness, crossfeed, matrix mixer, crossover,
 * channel gains/mutes/names, and optionally pin assignments.
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
#include "crossover.h"
#include "flash_clkdiv.h"
#include "usb_audio.h"
#include "crossfeed.h"
#include "pdm_generator.h"
#include "usb_feedback_controller.h"
#include "leveller.h"
#include "loudness.h"     // LOUDNESS_DEFAULT_OUTPUT_MASK
#include "lg_sound_sync.h"
#include "adat_output.h"     // adat_output_config_enabled/_pin/_set_config (RP2350)
#include "upmix.h"           // upmix_config + UPMIX_DEFAULT_* (RP2350 only)
#include "notify.h"
#include "uart_control.h"    // uart_ctrl_owns_pin (io_pin_valid guard)
#include "i2c_control.h"     // i2c_ctrl_owns_pin (io_pin_valid guard)
#include "bulk_params.h"     // WireBulkParams offsets for user_mute notify
#include "dac_hw_mute.h"     // hold query for flash_mute_hold_samples floor

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/irq.h"            // DMA_IRQ_0 (selective flash blackout keep mask)
#include "hardware/structs/nvic.h"   // nvic_hw (selective flash blackout)
#if PICO_RP2350
#include "hardware/structs/sysinfo.h"
#endif
#include "hardware/clocks.h"  // GPIO_TO_GPOUT_CLOCK_HANDLE() — MCK pin migration
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include <string.h>
#include <math.h>    // powf(), isfinite() for master volume (db_to_linear() clamps at -60 dB)
#include <stdio.h>   // printf() for MCK migration warning

// ============================================================================
// FLASH GEOMETRY
// ============================================================================

// Preset region: 1 directory sector + 10 slots × SLOT_SECTORS, at the end of
// flash.  A slot spans TWO sectors (8 KB) on RP2350 — a V21 PresetSlot (unified
// 17-channel EQ/names/crossover) is ~5.8 KB and exceeds one 4 KB sector — but
// only ONE sector on RP2040, whose 7-channel slot stays ~2.3 KB (and whose RAM
// is too tight for an 8 KB flash scratch buffer).  (The old single-sector
// "legacy" migration is gone — V21 breaks compatibility intentionally; pre-V21
// data loads factory defaults.)
#if PICO_RP2350
#define SLOT_SECTORS            2   // ~5.8 KB slot
#else
#define SLOT_SECTORS            1   // ~2.3 KB slot
#endif
#define PRESET_TOTAL_SECTORS    (1 + 10 * SLOT_SECTORS)   // 21 (RP2350) / 11 (RP2040)
#define PRESET_BASE_OFFSET      (PICO_FLASH_SIZE_BYTES - (PRESET_TOTAL_SECTORS * FLASH_SECTOR_SIZE))
#define SLOT_BYTES              (SLOT_SECTORS * FLASH_SECTOR_SIZE)   // slot allocation

// Individual sector offsets (byte offset from start of flash)
#define DIR_SECTOR_OFFSET       (PRESET_BASE_OFFSET)
#define SLOT_SECTOR_OFFSET(n)   (PRESET_BASE_OFFSET + (1 + (n) * SLOT_SECTORS) * FLASH_SECTOR_SIZE)

// XIP read pointers (memory-mapped flash)
#define DIR_ADDR                ((const PresetDirectory *)(XIP_BASE + DIR_SECTOR_OFFSET))
#define SLOT_ADDR(n)            ((const PresetSlot *)(XIP_BASE + SLOT_SECTOR_OFFSET(n)))

// Magic numbers — each distinct so we can tell sector types apart
#define DIR_MAGIC               0x44535032  // "DSP2"
#define SLOT_MAGIC              0x44535033  // "DSP3"
#define LEGACY_MAGIC            0x44535031  // "DSP1" (original format)

// Current data version for preset slot contents.
//
//   V12: Per-channel preamp + master volume (struct grew)
//   V13: Input source + spdif_rx_pin (consumed V12 padding — same size)
//   V14: LG Sound Sync (consumed remaining padding — same size)
//   V15: User volume vol_index (consumed last padding byte — same size)
//   V16: Crossover bands appended (xover_recipes — struct grows). Older
//        versions retain their original on-disk size; the CRC validator
//        uses slot_data_size_for_version() to pick the right byte range
//        per stored version.
//   V17: I2S input pin + rate appended (i2s_rx_pin, i2s_input_rate;
//        struct grows by 2 bytes, same per-version CRC range mechanism
//        as V16).
//   V18: FilterType enum renumbered (first-order all-pass at 8, PEQ padding
//        9..31, crossover types shifted from 8..39 to 32..63). No struct
//        change — same on-disk size as V17. Pre-V18 slots are migrated on
//        load by remap_filter_type_pre_v18().
//   V19: First-order shelf PEQ types added (FILTER_LOWSHELF1=9,
//        FILTER_HIGHSHELF1=10). New enum values only; no struct change, no
//        renumber, no migration — same on-disk size as V17/V18.
//   V20: RP2350 8-channel USB input — matrix + preamp for inputs 2..7 (tail).
//   V21: Unified channel model — inputs are first-class channels (per-input PEQ
//        + metering, no "master"); NUM_CHANNELS grows (17 on RP2350); matrix +
//        preamp stored direct (all inputs inline).  Compatibility is broken
//        intentionally: ONLY V21 slots are accepted; pre-V21 slots fail
//        validation and load factory defaults (no migration).  The slot now
//        spans 2 flash sectors (SLOT_BYTES).
//   V22: I2S multichannel input appended (i2s_input_channels + i2s_rx_pin_ext[3];
//        struct grows by 4 bytes).  Backward-compatible tail-append: V21 slots
//        still load (the new fields default to unset) via the per-version CRC
//        range mechanism (slot_data_size_for_version).
//   V23: ADAT bulk output appended (adat_enabled + adat_pin; struct grows by 2
//        bytes).  Backward-compatible tail-append like V22: V21/V22 slots still
//        load (the new fields default to unset) via slot_data_size_for_version.
//   V24: Optional SPDIF inputs 2/3 appended (spdif_rx_enabled_ext +
//        spdif_rx_pin_ext[2]; struct grows by 3 bytes).  Backward-compatible
//        tail-append like V22/V23: older slots load with both extra inputs
//        disabled and their pins unset.
//   V25: Leveller channel masks appended (leveller_detector_mask +
//        leveller_apply_mask; struct grows by 2 bytes).  Backward-compatible
//        tail-append like V24: older slots load the all-channels default (0xFF).
//   V26: Loudness output mask appended (loudness_output_mask; struct grows by
//        2 bytes).  Backward-compatible tail-append like V25: older slots load
//        the all-outputs default (0xFFFF).
//   V27: Crossfeed output pair mask appended (crossfeed_output_pair_mask; struct
//        grows by 1 byte).  Backward-compatible tail-append like V26: older slots
//        load the pair-1-only default (0x01).
//   V28: I2S clock master/slave mode appended (i2s_clock_mode; struct grows by 1
//        byte).  Backward-compatible tail-append like V22..V27: older slots
//        still load via slot_data_size_for_version; the missing byte reads as 0
//        (= master), which is the correct default for older presets.
//   V29: I2S clock-pin mode + slave-pair BCK appended (i2s_clock_pin_mode +
//        i2s_bck_pin_slave; struct grows by 2 bytes).  Backward-compatible
//        tail-append like V22..V28: V21..V28 slots still load via
//        slot_data_size_for_version; the missing bytes read as 0 (= unified +
//        unset), the correct defaults for older presets.
//   V30: Linkwitz Transform per-band target Q appended (peq_qp_x512, Q*512;
//        struct grows by NUM_CHANNELS*MAX_BANDS*2 bytes).  Backward-compatible
//        tail-append like V22..V29: V21..V29 slots still load via
//        slot_data_size_for_version; the missing bytes read as 0 (= 0.707
//        default), the correct default for older presets.
//   V31: Psychoacoustic bass appended (psybass_enabled + reserved + output mask
//        + five floats; struct grows by 24 bytes).  Backward-compatible
//        tail-append like V22..V30: V21..V30 slots still load via
//        slot_data_size_for_version; older slots have no psybass data and load
//        the disabled/all-outputs defaults (see apply_slot_to_live).
//   V32: ADAT input config appended (adat_input_pin + adat_input_enabled +
//        adat_input_clock_mode; struct grows by 3 bytes).  Backward-compatible
//        tail-append like V22..V31: V21..V31 slots still load via
//        slot_data_size_for_version; older slots decode as disabled / pin unset
//        (0xFF) / master since io_config_from_slot gates the read on version >= 32.
//   V33: Stereo upmixer appended (upmix_enabled + center/surround modes +
//        reserved + ten floats; struct grows by 44 bytes; RP2350-only feature).
//        Backward-compatible tail-append like V22..V32: V21..V32 slots still load
//        via slot_data_size_for_version; older slots have no upmix data and load
//        the disabled defaults (apply is gated on version >= 33, RP2350 only).
//   V34: Upmixer presence bell claims the upmix reserved byte (int8, dB * 2;
//        struct size unchanged).  V33 slots always wrote 0 there, which decodes
//        as the 0 dB default, so V33 loads need no special handling.
//   V35: SPDIF input 4 pin appended (spdif_rx_pin4; struct grows by 1 byte).
//        Backward-compatible tail-append like V22..V33: V21..V34 slots still
//        load via slot_data_size_for_version and keep the device-level pin.
//        Input 4's enable bit is bit 2 of the existing V24 spdif_rx_enabled_ext
//        byte, which pre-V35 firmware masked to 0, so no gate is needed there.
#define SLOT_DATA_VERSION       35

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

// Device-global physical IO/output configuration (directory V4).  Single source
// of truth for output pins/types, I2S MCK/BCK and the SPDIF RX pin while
// output_config_mode == OUTPUT_CONFIG_MODE_INDEPENDENT.  Fixed-size (platform-
// independent) so the directory layout/CRC stays stable across RP2040/RP2350;
// only the first NUM_PIN_OUTPUTS / NUM_SPDIF_INSTANCES entries are meaningful.
typedef struct __attribute__((packed)) {
    uint8_t output_pins[8];          // GPIO per pin-output (SPDIF/I2S/PDM)
    uint8_t output_types[4];         // Per-slot: 0=S/PDIF, 1=I2S
    uint8_t i2s_bck_pin;             // BCK GPIO; LRCLK = BCK + 1
    uint8_t i2s_mck_pin;             // MCK GPIO
    uint8_t i2s_mck_enabled;         // MCK on/off (0 or 1)
    uint8_t i2s_mck_multiplier;      // 0 = 128x, 1 = 256x
    uint8_t spdif_rx_pin;            // SPDIF RX GPIO (device-level)
    uint8_t i2s_rx_pin;              // I2S RX data GPIO, stereo pair 0 (0 = unset → default)
    uint8_t i2s_input_rate_p1;       // I2S input rate, wire encoding PLUS ONE
                                     // (0 = unset → 48 kHz; 1=44100, 2=48000,
                                     // 3=96000).  +1 sentinel because old
                                     // directories carry zeros here and plain
                                     // encoding 0 would mean 44.1 kHz
    uint8_t i2s_input_channels;      // Active I2S input channels: 2/4/6/8.  Claimed from
                                     // the former reserved[0]; 0 in pre-V5 directories =
                                     // unset → 2 on apply
    uint8_t i2s_rx_pin_ext[3];       // I2S RX data GPIOs for stereo pairs 1..3 (DIR V5+;
                                     // 0 = unset).  Grows the struct by 3 bytes, so the
                                     // V4→V5 directory migration reads old configs via
                                     // FlashOutputConfig_v4
    uint8_t adat_enabled;            // ADAT bulk output enable (0/1) (DIR V8+).  Grows the
                                     // struct by 2 bytes, so the V7→V8 directory migration
                                     // reads old configs via FlashOutputConfig_v7
    uint8_t adat_pin;                // ADAT data GPIO (0 = unset → PICO_ADAT_PIN)
    uint8_t spdif_rx_enabled_ext;    // Optional SPDIF inputs enable mask (DIR V12+;
                                     // bit 0 = SPDIF2 .. bit 2 = SPDIF4; 0 = all disabled).
                                     // Grows the struct by 3 bytes, so pre-V12 directory
                                     // migrations read old configs via FlashOutputConfig_v11
    uint8_t spdif_rx_pin_ext[2];     // SPDIF RX 2/3 GPIOs (0 = unset; defaults 20/21).
                                     // SPDIF 4's pin is the V16 tail byte, not a third entry
                                     // here: widening this array in place would break the
                                     // prefix-memcpy migrations below.  Index all three
                                     // through cfg_spdif_ext_pin()
    uint8_t i2s_clock_mode;          // I2S clock: 0=master, 1=slave (DIR V13+).  Grows the
                                     // struct by 1 byte, so the V12→V13 directory migration
                                     // reads old configs via FlashOutputConfig_v12; 0 (master)
                                     // is the correct default for a zero-filled/missing byte
    uint8_t i2s_clock_pin_mode;      // I2S clock-pin mode: 0=unified, 1=split (DIR V14+).
                                     // Grows the struct by 2 bytes with i2s_bck_pin_slave, so
                                     // the V13→V14 migration reads old configs via
                                     // FlashOutputConfig_v13; 0 (unified) is correct for a
                                     // zero-filled/missing byte
    uint8_t i2s_bck_pin_slave;       // Slave-mode BCK GPIO (SPLIT only); LRCLK = +1
                                     // (0 = unset → PICO_I2S_BCK_PIN_SLAVE)
    uint8_t adat_input_pin;          // ADAT RX GPIO (DIR V15+; 0xFF = unset).  Grows the
                                     // struct by 3 bytes, so the V14→V15 directory migration
                                     // reads old configs via FlashOutputConfig_v14; the pin
                                     // is seeded to 0xFF (not the zero-fill 0) on that path
    uint8_t adat_input_enabled;      // ADAT input enable (0/1)
    uint8_t adat_input_clock_mode;   // ADAT input clock: 0=master, 1=slave
    uint8_t spdif_rx_pin4;           // SPDIF RX 4 GPIO (DIR V16+; 0 = unset → default 22).
                                     // Grows the struct by 1 byte, so the V15→V16 directory
                                     // migration reads old configs via FlashOutputConfig_v15
} FlashOutputConfig;                 // 35 bytes

// The SPDIF 2/3 pins live in spdif_rx_pin_ext[] (DIR V12) and SPDIF 4's in the
// V16 tail byte; this indexes all three as one 0-based ext array so callers can
// loop over SPDIF_RX_NUM_INPUTS - 1 entries.
static inline uint8_t *cfg_spdif_ext_pin(FlashOutputConfig *cfg, uint8_t i) {
    return (i < 2) ? &cfg->spdif_rx_pin_ext[i] : &cfg->spdif_rx_pin4;
}
static inline uint8_t cfg_spdif_ext_pin_get(const FlashOutputConfig *cfg, uint8_t i) {
    return (i < 2) ? cfg->spdif_rx_pin_ext[i] : cfg->spdif_rx_pin4;
}

// Historical 20-byte device-global IO config (directory V4), before the I2S
// multichannel input fields were appended.  Read only by the V4→V5 directory
// migration in load_directory().
typedef struct __attribute__((packed)) {
    uint8_t output_pins[8];
    uint8_t output_types[4];
    uint8_t i2s_bck_pin;
    uint8_t i2s_mck_pin;
    uint8_t i2s_mck_enabled;
    uint8_t i2s_mck_multiplier;
    uint8_t spdif_rx_pin;
    uint8_t i2s_rx_pin;
    uint8_t i2s_input_rate_p1;
    uint8_t reserved[1];
} FlashOutputConfig_v4;              // 20 bytes

// Historical 23-byte device-global IO config (directory V5-V7), before the
// ADAT bulk-output fields were appended.  Read only by the V5→V6, V6→V7 and
// V7→V8 directory migrations (whose snapshots embed this frozen type so their
// on-flash layout stays stable now that the live FlashOutputConfig has grown).
typedef struct __attribute__((packed)) {
    uint8_t output_pins[8];
    uint8_t output_types[4];
    uint8_t i2s_bck_pin;
    uint8_t i2s_mck_pin;
    uint8_t i2s_mck_enabled;
    uint8_t i2s_mck_multiplier;
    uint8_t spdif_rx_pin;
    uint8_t i2s_rx_pin;
    uint8_t i2s_input_rate_p1;
    uint8_t i2s_input_channels;
    uint8_t i2s_rx_pin_ext[3];
} FlashOutputConfig_v7;              // 23 bytes

// Historical 25-byte device-global IO config (directory V8-V11), before the
// optional SPDIF input 2/3 fields were appended.  Read by the V8..V11
// directory migrations; a strict prefix of the live FlashOutputConfig, so a
// prefix memcpy widens it (new tail bytes stay 0 = both inputs disabled).
typedef struct __attribute__((packed)) {
    uint8_t output_pins[8];
    uint8_t output_types[4];
    uint8_t i2s_bck_pin;
    uint8_t i2s_mck_pin;
    uint8_t i2s_mck_enabled;
    uint8_t i2s_mck_multiplier;
    uint8_t spdif_rx_pin;
    uint8_t i2s_rx_pin;
    uint8_t i2s_input_rate_p1;
    uint8_t i2s_input_channels;
    uint8_t i2s_rx_pin_ext[3];
    uint8_t adat_enabled;
    uint8_t adat_pin;
} FlashOutputConfig_v11;             // 25 bytes

// Historical 28-byte device-global IO config (directory V12), before the I2S
// clock master/slave mode byte was appended.  A strict prefix of the live
// FlashOutputConfig; read only by the V12→V13 directory migration (prefix
// memcpy widens it; the new tail byte stays 0 = master).
typedef struct __attribute__((packed)) {
    uint8_t output_pins[8];
    uint8_t output_types[4];
    uint8_t i2s_bck_pin;
    uint8_t i2s_mck_pin;
    uint8_t i2s_mck_enabled;
    uint8_t i2s_mck_multiplier;
    uint8_t spdif_rx_pin;
    uint8_t i2s_rx_pin;
    uint8_t i2s_input_rate_p1;
    uint8_t i2s_input_channels;
    uint8_t i2s_rx_pin_ext[3];
    uint8_t adat_enabled;
    uint8_t adat_pin;
    uint8_t spdif_rx_enabled_ext;
    uint8_t spdif_rx_pin_ext[2];
} FlashOutputConfig_v12;             // 28 bytes

// Frozen pre-V9 Control Surfaces layout (config format v1); read only by the
// V8→V9 directory migration (and the V7→V9 path, whose V7 flash also carries
// this old blob).  The live CsBinding / CsFlashConfig in control_surfaces.h
// have since grown (24-byte bindings with event/target/index, 16 slots), so
// these frozen copies keep the old on-flash geometry stable for the migration.
typedef struct __attribute__((packed)) {
    uint8_t type;
    uint8_t noun;
    uint8_t action;
    uint8_t flags;
    uint8_t gpio[2];
    uint8_t reserved[2];
    int16_t value;
    int16_t step;
    int16_t range_min;
    int16_t range_max;
} CsBinding_v1;                      // 16 bytes

typedef struct __attribute__((packed)) {
    uint8_t       version;          // == 1
    uint8_t       reserved[3];
    CsBinding_v1  bindings[8];
} CsFlashConfig_v1;                 // 132 bytes

// Frozen pre-V17 IR command table (config format v1, 8 sub-slots); read by
// every directory migration that predates V17.  IrCommand itself is unchanged,
// so only the sub-slot count differs from the live CsIrConfig.
typedef struct __attribute__((packed)) {
    uint8_t   version;              // == 1
    uint8_t   reserved[3];
    IrCommand cmds[8];
} CsIrConfig_v1;                    // 132 bytes

// Historical 29-byte device-global IO config (directory V13), before the I2S
// clock-pin mode fields were appended.  A strict prefix of the live
// FlashOutputConfig; read only by the V13→V14 directory migration (prefix
// memcpy widens it; the new tail bytes stay 0 = unified + unset).
typedef struct __attribute__((packed)) {
    uint8_t output_pins[8];
    uint8_t output_types[4];
    uint8_t i2s_bck_pin;
    uint8_t i2s_mck_pin;
    uint8_t i2s_mck_enabled;
    uint8_t i2s_mck_multiplier;
    uint8_t spdif_rx_pin;
    uint8_t i2s_rx_pin;
    uint8_t i2s_input_rate_p1;
    uint8_t i2s_input_channels;
    uint8_t i2s_rx_pin_ext[3];
    uint8_t adat_enabled;
    uint8_t adat_pin;
    uint8_t spdif_rx_enabled_ext;
    uint8_t spdif_rx_pin_ext[2];
    uint8_t i2s_clock_mode;
} FlashOutputConfig_v13;             // 29 bytes

// Historical 31-byte device-global IO config (directory V14), before the ADAT
// input fields were appended.  A strict prefix of the live FlashOutputConfig;
// read only by the V14→V15 directory migration (prefix memcpy widens it; the
// migration explicitly seeds adat_input_pin = 0xFF since the zero-fill tail
// would misread as GPIO 0).
typedef struct __attribute__((packed)) {
    uint8_t output_pins[8];
    uint8_t output_types[4];
    uint8_t i2s_bck_pin;
    uint8_t i2s_mck_pin;
    uint8_t i2s_mck_enabled;
    uint8_t i2s_mck_multiplier;
    uint8_t spdif_rx_pin;
    uint8_t i2s_rx_pin;
    uint8_t i2s_input_rate_p1;
    uint8_t i2s_input_channels;
    uint8_t i2s_rx_pin_ext[3];
    uint8_t adat_enabled;
    uint8_t adat_pin;
    uint8_t spdif_rx_enabled_ext;
    uint8_t spdif_rx_pin_ext[2];
    uint8_t i2s_clock_mode;
    uint8_t i2s_clock_pin_mode;
    uint8_t i2s_bck_pin_slave;
} FlashOutputConfig_v14;             // 31 bytes

// Historical 34-byte device-global IO config (directory V15), before the SPDIF
// input 4 pin was appended.  A strict prefix of the live FlashOutputConfig;
// read only by the V15→V16 directory migration (prefix memcpy widens it; the
// new tail byte stays 0 = unset, which resolves to the default GPIO).
typedef struct __attribute__((packed)) {
    uint8_t output_pins[8];
    uint8_t output_types[4];
    uint8_t i2s_bck_pin;
    uint8_t i2s_mck_pin;
    uint8_t i2s_mck_enabled;
    uint8_t i2s_mck_multiplier;
    uint8_t spdif_rx_pin;
    uint8_t i2s_rx_pin;
    uint8_t i2s_input_rate_p1;
    uint8_t i2s_input_channels;
    uint8_t i2s_rx_pin_ext[3];
    uint8_t adat_enabled;
    uint8_t adat_pin;
    uint8_t spdif_rx_enabled_ext;
    uint8_t spdif_rx_pin_ext[2];
    uint8_t i2s_clock_mode;
    uint8_t i2s_clock_pin_mode;
    uint8_t i2s_bck_pin_slave;
    uint8_t adat_input_pin;
    uint8_t adat_input_enabled;
    uint8_t adat_input_clock_mode;
} FlashOutputConfig_v15;             // 34 bytes

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

// --- Preset Directory v2 (legacy snapshot used by v2→v3 migration) ---
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t crc32;
    uint8_t  startup_mode;
    uint8_t  default_slot;
    uint8_t  last_active_slot;
    uint8_t  include_pins;
    uint16_t slot_occupied;
    uint8_t  master_volume_mode;
    uint8_t  spdif_rx_pin;
    float    master_volume_db;
    char     slot_names[PRESET_SLOTS][PRESET_NAME_LEN];
} PresetDirectory_v2;

// --- Preset Directory v3 (snapshot — kept only for v3→v4 migration) ---
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t crc32;
    uint8_t  startup_mode;
    uint8_t  default_slot;
    uint8_t  last_active_slot;
    uint8_t  include_pins;                    // → output_config_mode in V4
    uint16_t slot_occupied;
    uint8_t  master_volume_mode;
    uint8_t  spdif_rx_pin;
    float    master_volume_db;
    char     slot_names[PRESET_SLOTS][PRESET_NAME_LEN];
    DacHwMuteConfig dac_hw_mute;
} PresetDirectory_v3;

// --- Preset Directory v4 (current, sector 0) ---
// V3 appended the DacHwMuteConfig field (16 bytes) so users can configure a
// DAC hardware-mute GPIO that gets asserted during pipeline reset to
// suppress the audible click when I²S BCK/LRCLK stop mid-cycle.  The mute
// config is a board-level attribute (pin numbers + polarity + timing), not a
// listening profile, so it lives in the directory rather than per-preset.
//
// V4 repurposes the former `include_pins` byte as `output_config_mode` (same
// byte/offset, 1:1 value mapping → no migration of the value) and appends a
// device-global FlashOutputConfig block.  Together they let the device's whole
// physical IO config either travel with presets (WITH_PRESET, default) or be
// stored once here and applied at boot (INDEPENDENT) — mirroring the
// master-volume independent/with-preset mechanism.
//
// V6 appends the device-level external control-interface config (UART and I2C
// control ports).  Like the DAC hardware-mute block these are board-level
// attributes (which pins/baud/address the control link uses), not a listening
// profile, so they live device-global in the directory rather than per-preset.
//
// V7 appends the Control Surfaces binding config (which physical controls /
// indicators are wired to which GPIOs); board-level for the same reason.
//
// V8 grows the device-global output_config by 2 bytes (ADAT bulk output:
// adat_enabled + adat_pin); board-level, same INDEPENDENT/WITH_PRESET model as
// the rest of the physical IO config.
//
// V9 upgrades the embedded Control Surfaces config from format v1 to v2: each
// binding grows from 16 to 24 bytes (adds explicit event / target / index
// fields) and the slot count grows from 8 to 16, so cs_config grows from 132 to
// 388 bytes.  The V8→V9 migration copies each of the 8 old bindings' shared
// fields forward and zeroes the new ones; slots 8..15 start empty.
//
// V10 appends per-slot Control Surfaces names (16 x 32 bytes): user labels for
// what each physical control is for, set by the host app and readable by
// external MCUs / other hosts via REQ_GET_CS_NAME.  Slot metadata independent
// of the bindings (a name may exist before its binding and survives binding
// changes), hence a directory field rather than a cs_config format bump.
//
// V11 appends the Control Surfaces IR command table (learned remote-button
// commands for the CS_TYPE_IR component): a device-global CsIrConfig beside
// cs_config, board-level for the same reason.  All-zero = every sub-slot empty
// (protocol 0 = CS_IR_PROTO_NONE) = feature idle, so a fresh directory needs no
// seeding.  The V10->V11 migration copies every field forward and leaves the
// new block zeroed.
//
// V12 grows the embedded output_config by 3 bytes (optional SPDIF inputs 2/3:
// enable mask + pins), shifting every later member; pre-V12 migrations read the
// old layout through FlashOutputConfig_v11 / PresetDirectory_v11.  All-zero new
// bytes = both extra inputs disabled, pins unset (platform defaults on apply).
//
// V13 grows the device-global output_config by 1 byte (I2S clock master/slave
// mode: i2s_clock_mode), shifting every later member; the V12→V13 migration
// reads the old layout through FlashOutputConfig_v12 / PresetDirectory_v12.
// The all-zero new byte = master, the correct legacy default; same
// INDEPENDENT/WITH_PRESET model.
//
// V14 grows the device-global output_config by 2 bytes (I2S clock-pin mode:
// i2s_clock_pin_mode + i2s_bck_pin_slave); same INDEPENDENT/WITH_PRESET model.
//
// V15 grows the device-global output_config by 3 bytes (ADAT input:
// adat_input_pin + adat_input_enabled + adat_input_clock_mode); same
// INDEPENDENT/WITH_PRESET model.  The V14→V15 migration reads the old layout
// through FlashOutputConfig_v14 / PresetDirectory_v14 and seeds adat_input_pin
// to 0xFF (the zero-fill 0 would misread as GPIO 0).
//
// V16 grows the device-global output_config by 1 byte (SPDIF input 4 pin:
// spdif_rx_pin4); the zero-fill "unset" resolves to the platform default.
//
// V17 upgrades the embedded IR command table from format v1 to v2: the sub-slot
// count grows from 8 to 16, so cs_ir grows from 132 to 260 bytes.  It is the
// last directory member, so every earlier field keeps its offset; pre-V17
// migrations read the old block through CsIrConfig_v1 and widen it with
// cs_ir_from_v1(), leaving sub-slots 8..15 empty.
typedef struct __attribute__((packed)) {
    uint32_t magic;                          // DIR_MAGIC
    uint16_t version;                        // Directory format version (4)
    uint16_t reserved;
    uint32_t crc32;                          // CRC over everything after this 12-byte header

    // Startup configuration
    uint8_t  startup_mode;                   // PRESET_STARTUP_SPECIFIED or _LAST_ACTIVE
    uint8_t  default_slot;                   // Slot to load in SPECIFIED mode (0-9)
    uint8_t  last_active_slot;               // Last loaded/saved slot (always 0-9)
    uint8_t  output_config_mode;             // OUTPUT_CONFIG_MODE_* (was include_pins; 1↔WITH_PRESET, 0↔INDEPENDENT)

    // Slot metadata
    uint16_t slot_occupied;                  // Bitmask: bit N = slot N has valid data
    uint8_t  master_volume_mode;             // MASTER_VOLUME_MODE_INDEPENDENT or _WITH_PRESET (was include_master_volume)
    uint8_t  spdif_rx_pin;                   // SPDIF RX GPIO pin, device-level (legacy; superseded by output_config.spdif_rx_pin)
    float    master_volume_db;               // Independent master volume (mode 0 at boot)
    char     slot_names[PRESET_SLOTS][PRESET_NAME_LEN];  // 32-byte NUL-terminated names

    // V3 addition: DAC hardware mute pin configuration (board-level).
    DacHwMuteConfig dac_hw_mute;             // 16 bytes; enabled=0 by default

    // V4 addition: device-global physical IO config (INDEPENDENT mode store).
    // V5 grows it by 3 bytes (I2S multichannel input: i2s_rx_pin_ext[3]).
    // V8 grows it by 2 bytes (ADAT bulk output: adat_enabled + adat_pin).
    // V12 grows it by 3 bytes (optional SPDIF inputs 2/3: enable mask + pins).
    // V13 grows it by 1 byte (I2S clock master/slave mode: i2s_clock_mode).
    // V14 grows it by 2 bytes (I2S clock-pin mode: i2s_clock_pin_mode +
    // i2s_bck_pin_slave).
    // V15 grows it by 3 bytes (ADAT input: adat_input_pin + adat_input_enabled +
    // adat_input_clock_mode).
    // V16 grows it by 1 byte (SPDIF input 4 pin: spdif_rx_pin4).
    FlashOutputConfig output_config;         // 35 bytes

    // V6 addition: device-level external control-interface config (board-level).
    UartCtrlConfig uart_ctrl;                // 8 bytes; enabled=0 by default
    I2cCtrlConfig  i2c_ctrl;                 // 8 bytes; enabled=0 by default

    // V7 addition: Control Surfaces bindings (board-level).  All-zero =
    // every slot CS_TYPE_NONE = feature idle.  V9 grew this from 132 to 388
    // bytes (config format v2: 24-byte bindings, 16 slots).
    CsFlashConfig cs_config;                 // 388 bytes

    // V10 addition: per-slot Control Surfaces names (user labels, NUL-
    // terminated).  All-zero = unnamed; a fresh directory needs no seeding.
    char cs_names[CS_MAX_BINDINGS][CS_NAME_LEN];  // 512 bytes

    // V11 addition: Control Surfaces IR command table (learned remote-button
    // commands for the CS_TYPE_IR component).  Board-level / device-global like
    // cs_config; all-zero = every sub-slot empty (feature idle).  V17 grew this
    // from 132 to 260 bytes (config format v2: 16 sub-slots).
    CsIrConfig cs_ir;                        // 260 bytes
} PresetDirectory;

// Historical directory layout at V4, where output_config was the 20-byte
// FlashOutputConfig_v4.  Read only by the V4→V5 migration in load_directory();
// identical to PresetDirectory except the output_config type.
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t crc32;
    uint8_t  startup_mode;
    uint8_t  default_slot;
    uint8_t  last_active_slot;
    uint8_t  output_config_mode;
    uint16_t slot_occupied;
    uint8_t  master_volume_mode;
    uint8_t  spdif_rx_pin;
    float    master_volume_db;
    char     slot_names[PRESET_SLOTS][PRESET_NAME_LEN];
    DacHwMuteConfig dac_hw_mute;
    FlashOutputConfig_v4 output_config;      // 20 bytes
} PresetDirectory_v4;

// Historical directory layout at V5, before the V6 control-interface fields.
// Read only by the V5→V6 migration in load_directory(); identical to
// PresetDirectory except it lacks the trailing uart_ctrl / i2c_ctrl blocks.
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t crc32;
    uint8_t  startup_mode;
    uint8_t  default_slot;
    uint8_t  last_active_slot;
    uint8_t  output_config_mode;
    uint16_t slot_occupied;
    uint8_t  master_volume_mode;
    uint8_t  spdif_rx_pin;
    float    master_volume_db;
    char     slot_names[PRESET_SLOTS][PRESET_NAME_LEN];
    DacHwMuteConfig dac_hw_mute;
    FlashOutputConfig_v7 output_config;      // 23 bytes (frozen pre-ADAT layout)
} PresetDirectory_v5;

// Historical directory layout at V6, before the V7 Control Surfaces config.
// Read only by the V6→V7 migration in load_directory(); identical to
// PresetDirectory except it lacks the trailing cs_config block.
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t crc32;
    uint8_t  startup_mode;
    uint8_t  default_slot;
    uint8_t  last_active_slot;
    uint8_t  output_config_mode;
    uint16_t slot_occupied;
    uint8_t  master_volume_mode;
    uint8_t  spdif_rx_pin;
    float    master_volume_db;
    char     slot_names[PRESET_SLOTS][PRESET_NAME_LEN];
    DacHwMuteConfig dac_hw_mute;
    FlashOutputConfig_v7 output_config;      // 23 bytes (frozen pre-ADAT layout)
    UartCtrlConfig uart_ctrl;
    I2cCtrlConfig  i2c_ctrl;
} PresetDirectory_v6;

// Historical directory layout at V7, before the ADAT bulk-output fields grew
// the device-global output_config.  Read only by the V7→V9 migration in
// load_directory(); identical to PresetDirectory except output_config is the
// frozen 23-byte FlashOutputConfig_v7 (no adat_enabled / adat_pin) and cs_config
// is the frozen 132-byte CsFlashConfig_v1 (config format v1, 8 bindings).
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t crc32;
    uint8_t  startup_mode;
    uint8_t  default_slot;
    uint8_t  last_active_slot;
    uint8_t  output_config_mode;
    uint16_t slot_occupied;
    uint8_t  master_volume_mode;
    uint8_t  spdif_rx_pin;
    float    master_volume_db;
    char     slot_names[PRESET_SLOTS][PRESET_NAME_LEN];
    DacHwMuteConfig dac_hw_mute;
    FlashOutputConfig_v7 output_config;      // 23 bytes
    UartCtrlConfig uart_ctrl;
    I2cCtrlConfig  i2c_ctrl;
    CsFlashConfig_v1 cs_config;              // 132 bytes (frozen format v1)
} PresetDirectory_v7;

// Historical directory layout at V8, before the Control Surfaces config grew
// from format v1 to v2.  Read only by the V8→V9 migration in load_directory();
// identical to the current PresetDirectory except cs_config is the frozen
// 132-byte CsFlashConfig_v1 (8 x 16-byte bindings) rather than the 388-byte v2.
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t crc32;
    uint8_t  startup_mode;
    uint8_t  default_slot;
    uint8_t  last_active_slot;
    uint8_t  output_config_mode;
    uint16_t slot_occupied;
    uint8_t  master_volume_mode;
    uint8_t  spdif_rx_pin;
    float    master_volume_db;
    char     slot_names[PRESET_SLOTS][PRESET_NAME_LEN];
    DacHwMuteConfig dac_hw_mute;
    FlashOutputConfig_v11 output_config;     // 25 bytes (frozen pre-multi-SPDIF layout)
    UartCtrlConfig uart_ctrl;
    I2cCtrlConfig  i2c_ctrl;
    CsFlashConfig_v1 cs_config;              // 132 bytes (frozen format v1)
} PresetDirectory_v8;

// Historical directory layout at V9, before the per-slot Control Surfaces
// names were appended.  Read only by the V9→V10 migration in load_directory();
// identical to the current PresetDirectory except it lacks the trailing
// cs_names block.
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t crc32;
    uint8_t  startup_mode;
    uint8_t  default_slot;
    uint8_t  last_active_slot;
    uint8_t  output_config_mode;
    uint16_t slot_occupied;
    uint8_t  master_volume_mode;
    uint8_t  spdif_rx_pin;
    float    master_volume_db;
    char     slot_names[PRESET_SLOTS][PRESET_NAME_LEN];
    DacHwMuteConfig dac_hw_mute;
    FlashOutputConfig_v11 output_config;     // 25 bytes (frozen pre-multi-SPDIF layout)
    UartCtrlConfig uart_ctrl;
    I2cCtrlConfig  i2c_ctrl;
    CsFlashConfig cs_config;                 // 388 bytes (current format v2)
} PresetDirectory_v9;

// Historical directory layout at V10, before the IR command table was appended.
// Read only by the V10->V11 migration in load_directory(); identical to the
// current PresetDirectory except it lacks the trailing cs_ir block.
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t crc32;
    uint8_t  startup_mode;
    uint8_t  default_slot;
    uint8_t  last_active_slot;
    uint8_t  output_config_mode;
    uint16_t slot_occupied;
    uint8_t  master_volume_mode;
    uint8_t  spdif_rx_pin;
    float    master_volume_db;
    char     slot_names[PRESET_SLOTS][PRESET_NAME_LEN];
    DacHwMuteConfig dac_hw_mute;
    FlashOutputConfig_v11 output_config;     // 25 bytes (frozen pre-multi-SPDIF layout)
    UartCtrlConfig uart_ctrl;
    I2cCtrlConfig  i2c_ctrl;
    CsFlashConfig cs_config;                 // 388 bytes (current format v2)
    char cs_names[CS_MAX_BINDINGS][CS_NAME_LEN];  // 512 bytes
} PresetDirectory_v10;

// Historical directory layout at V11, before the device-global output_config
// grew by 3 bytes (optional SPDIF inputs 2/3).  Read only by the V11->V12
// migration in load_directory(); identical to the current PresetDirectory
// except output_config is the frozen 25-byte FlashOutputConfig_v11.
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t crc32;
    uint8_t  startup_mode;
    uint8_t  default_slot;
    uint8_t  last_active_slot;
    uint8_t  output_config_mode;
    uint16_t slot_occupied;
    uint8_t  master_volume_mode;
    uint8_t  spdif_rx_pin;
    float    master_volume_db;
    char     slot_names[PRESET_SLOTS][PRESET_NAME_LEN];
    DacHwMuteConfig dac_hw_mute;
    FlashOutputConfig_v11 output_config;     // 25 bytes (frozen pre-multi-SPDIF layout)
    UartCtrlConfig uart_ctrl;
    I2cCtrlConfig  i2c_ctrl;
    CsFlashConfig cs_config;                 // 388 bytes (current format v2)
    char cs_names[CS_MAX_BINDINGS][CS_NAME_LEN];  // 512 bytes
    CsIrConfig_v1 cs_ir;                     // 132 bytes (frozen format v1)
} PresetDirectory_v11;

// Historical directory layout at V12, before the device-global output_config
// grew by 1 byte (I2S clock master/slave mode).  Read only by the V12->V13
// migration in load_directory(); identical to the current PresetDirectory
// except output_config is the frozen 28-byte FlashOutputConfig_v12.
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t crc32;
    uint8_t  startup_mode;
    uint8_t  default_slot;
    uint8_t  last_active_slot;
    uint8_t  output_config_mode;
    uint16_t slot_occupied;
    uint8_t  master_volume_mode;
    uint8_t  spdif_rx_pin;
    float    master_volume_db;
    char     slot_names[PRESET_SLOTS][PRESET_NAME_LEN];
    DacHwMuteConfig dac_hw_mute;
    FlashOutputConfig_v12 output_config;     // 28 bytes (frozen pre-clock-mode layout)
    UartCtrlConfig uart_ctrl;
    I2cCtrlConfig  i2c_ctrl;
    CsFlashConfig cs_config;                 // 388 bytes (current format v2)
    char cs_names[CS_MAX_BINDINGS][CS_NAME_LEN];  // 512 bytes
    CsIrConfig_v1 cs_ir;                     // 132 bytes (frozen format v1)
} PresetDirectory_v12;

// Historical directory layout at V13, before the I2S clock-pin mode fields grew
// the device-global output_config.  Read only by the V13->V14 migration in
// load_directory(); identical to the current PresetDirectory except
// output_config is the frozen 29-byte FlashOutputConfig_v13 (no
// i2s_clock_pin_mode / i2s_bck_pin_slave).
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t crc32;
    uint8_t  startup_mode;
    uint8_t  default_slot;
    uint8_t  last_active_slot;
    uint8_t  output_config_mode;
    uint16_t slot_occupied;
    uint8_t  master_volume_mode;
    uint8_t  spdif_rx_pin;
    float    master_volume_db;
    char     slot_names[PRESET_SLOTS][PRESET_NAME_LEN];
    DacHwMuteConfig dac_hw_mute;
    FlashOutputConfig_v13 output_config;     // 29 bytes (frozen pre-clock-pin-mode layout)
    UartCtrlConfig uart_ctrl;
    I2cCtrlConfig  i2c_ctrl;
    CsFlashConfig cs_config;                 // 388 bytes (current format v2)
    char cs_names[CS_MAX_BINDINGS][CS_NAME_LEN];  // 512 bytes
    CsIrConfig_v1 cs_ir;                     // 132 bytes (frozen format v1)
} PresetDirectory_v13;

// Historical directory layout at V14, before the ADAT input fields grew the
// device-global output_config.  Read only by the V14->V15 migration in
// load_directory(); identical to the current PresetDirectory except
// output_config is the frozen 31-byte FlashOutputConfig_v14 (no adat_input_*).
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t crc32;
    uint8_t  startup_mode;
    uint8_t  default_slot;
    uint8_t  last_active_slot;
    uint8_t  output_config_mode;
    uint16_t slot_occupied;
    uint8_t  master_volume_mode;
    uint8_t  spdif_rx_pin;
    float    master_volume_db;
    char     slot_names[PRESET_SLOTS][PRESET_NAME_LEN];
    DacHwMuteConfig dac_hw_mute;
    FlashOutputConfig_v14 output_config;     // 31 bytes (frozen pre-ADAT-input layout)
    UartCtrlConfig uart_ctrl;
    I2cCtrlConfig  i2c_ctrl;
    CsFlashConfig cs_config;                 // 388 bytes (current format v2)
    char cs_names[CS_MAX_BINDINGS][CS_NAME_LEN];  // 512 bytes
    CsIrConfig_v1 cs_ir;                     // 132 bytes (frozen format v1)
} PresetDirectory_v14;

// --- Preset Directory v15 (kept only for upgrade migration) ---
// Identical to the live layout except output_config is the 34-byte
// FlashOutputConfig_v15 (no SPDIF input 4 pin).
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;                        // == 15
    uint16_t reserved;
    uint32_t crc32;

    uint8_t  startup_mode;
    uint8_t  default_slot;
    uint8_t  last_active_slot;
    uint8_t  output_config_mode;
    uint16_t slot_occupied;
    uint8_t  master_volume_mode;
    uint8_t  spdif_rx_pin;
    float    master_volume_db;
    char     slot_names[PRESET_SLOTS][PRESET_NAME_LEN];
    DacHwMuteConfig dac_hw_mute;
    FlashOutputConfig_v15 output_config;     // 34 bytes (frozen pre-SPDIF-4 layout)
    UartCtrlConfig uart_ctrl;
    I2cCtrlConfig  i2c_ctrl;
    CsFlashConfig cs_config;                 // 388 bytes (current format v2)
    char cs_names[CS_MAX_BINDINGS][CS_NAME_LEN];  // 512 bytes
    CsIrConfig_v1 cs_ir;                     // 132 bytes (frozen format v1)
} PresetDirectory_v15;

// --- Preset Directory v16 (kept only for upgrade migration) ---
// Identical to the live layout except cs_ir is the frozen 132-byte
// CsIrConfig_v1 (8 IR sub-slots) rather than the 260-byte v2.  output_config is
// still the live type only because nothing has grown it since V16; the size
// assert below pins that, and growing it means snapshotting a
// FlashOutputConfig_v16 here first.
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;                        // == 16
    uint16_t reserved;
    uint32_t crc32;

    uint8_t  startup_mode;
    uint8_t  default_slot;
    uint8_t  last_active_slot;
    uint8_t  output_config_mode;
    uint16_t slot_occupied;
    uint8_t  master_volume_mode;
    uint8_t  spdif_rx_pin;
    float    master_volume_db;
    char     slot_names[PRESET_SLOTS][PRESET_NAME_LEN];
    DacHwMuteConfig dac_hw_mute;
    FlashOutputConfig output_config;         // 35 bytes (current layout)
    UartCtrlConfig uart_ctrl;
    I2cCtrlConfig  i2c_ctrl;
    CsFlashConfig cs_config;                 // 388 bytes (current format v2)
    char cs_names[CS_MAX_BINDINGS][CS_NAME_LEN];  // 512 bytes
    CsIrConfig_v1 cs_ir;                     // 132 bytes (frozen format v1)
} PresetDirectory_v16;

// The V16->V17 migration copies everything ahead of cs_ir with one memcpy, so
// the two layouts must agree byte-for-byte up to that point.
_Static_assert(offsetof(PresetDirectory_v16, cs_ir) == offsetof(PresetDirectory, cs_ir),
               "V16 and V17 directories must share a byte-identical pre-cs_ir prefix");
// Pins the on-flash V16 geometry, which the offset check above cannot: it would
// still pass if a shared embedded struct grew on both sides at once.
_Static_assert(sizeof(PresetDirectory_v16) == 1443,
               "V16 directory geometry is frozen; snapshot any struct that grew");

#define DIR_VERSION_CURRENT  17

// The directory occupies exactly one flash sector; growth past it would
// silently overrun into preset slot 0.
_Static_assert(sizeof(PresetDirectory) <= FLASH_SECTOR_SIZE,
               "PresetDirectory must fit the single directory sector");

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
    // Matrix mixer — all inputs, direct (V21).
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
    // Per-channel preamp + Master volume — all inputs, direct (V21).
    float   preamp_db_per_ch[NUM_INPUT_CHANNELS];  // Per-input-channel preamp (dB)
    float   master_volume_db;                       // Device master volume (-128 mute, -127..0 dB)
    // Input source selection (V13) + SPDIF RX pin
    //
    // spdif_rx_pin claims one of V13's three padding bytes without changing
    // the slot struct size, so existing V13 slots remain CRC-valid: their
    // padding bytes were zero-initialised by collect_live_state's memset
    // (line ~484), and apply_slot_to_live treats spdif_rx_pin == 0 as
    // invalid and falls through to the live default — same effect as if
    // the byte never existed. Forward saves write the live pin and the
    // CRC is recomputed at save time.
    uint8_t input_source;            // InputSource enum (0=USB, 1=SPDIF)
    uint8_t spdif_rx_pin;            // SPDIF RX GPIO (0 = absent → use default)
    // LG Sound Sync per-preset toggle (V14).  Claims one of V13's two
    // padding bytes WITHOUT changing the slot struct size, so existing
    // V13 slots remain CRC-valid: their padding bytes were zero-initialised
    // by collect_live_state's memset, and apply_slot_to_live() gates the
    // read on `slot->version >= 14` so pre-V14 reads ignore this byte
    // entirely and fall through to LG_SOUND_SYNC_DEFAULT_ENABLED.  This
    // mirrors the V12→V13 trick where `spdif_rx_pin` consumed one of V12's
    // padding bytes — the established pattern for adding bytes without
    // breaking CRC compatibility on already-saved user presets.
    uint8_t lg_sound_sync_enabled;
    // User volume vol_index (V15).  Claims V14's last padding byte using
    // the same trick as lg_sound_sync_enabled — pre-V15 slots have it
    // zero-initialised, and apply_slot_to_live gates the read on
    // `slot->version >= 15` so pre-V15 loads do NOT touch user volume
    // (preserving the listening level the user had set, rather than
    // surprise-muting on legacy preset load).  Stored as vol_index
    // (range [0, CENTER_VOLUME_INDEX]) rather than float dB because the
    // audio path quantizes to integer dB anyway (apply_vol_index_to_audio
    // truncates the 8-bit fractional part of audio_state.volume), so
    // single-byte storage is lossless for the actual audio behavior; the
    // fractional dB that Windows uses for UAC1 GET_CUR roundtrip is
    // rebuilt from the index at load time.  THIS IS THE LAST AVAILABLE
    // PADDING BYTE — future preset additions will need either struct
    // growth (with explicit migration of pre-V15 slots) or directory-
    // level storage in the master-volume "independent" pattern.
    uint8_t user_vol_index;

    // Crossover bands (V16+).  Stored with the same EqParamPacket shape as
    // PEQ; `band` MUST be the wire band index (XOVER_BAND_BASE + i) for every
    // entry, because live-edit dispatch (main.c::eq_update_pending) routes
    // by the recipe's `band` field.  apply_slot_to_live() re-normalizes on
    // load defensively; migrate_legacy() initialises this section with
    // crossover defaults (FLAT type, fc=1000, band=XOVER_BAND_BASE+i) before
    // CRC.  See Documentation/Features/crossover_filters_spec.md.
    EqParamPacket xover_recipes[NUM_CHANNELS][MAX_XOVER_BANDS];

    // I2S input (V17+, struct grows by 2 bytes).  Same unset semantics as
    // spdif_rx_pin: i2s_rx_pin == 0 means "absent, use the live default".
    // i2s_input_rate uses the wire encoding (0=44100, 1=48000, 2=96000)
    // and is gated on version >= 17 in io_config_from_slot(), so the
    // missing-field case never decodes a bogus 44.1 kHz from a zero byte.
    uint8_t i2s_rx_pin;
    uint8_t i2s_input_rate;

    // I2S multichannel input (V22+, struct grows by 4 bytes).  Same 0 = unset
    // semantics as i2s_rx_pin; gated on version >= 22 in io_config_from_slot()
    // so pre-V22 slots never decode these as configured.
    uint8_t i2s_input_channels;      // 2/4/6/8 (0 = unset → device/live default)
    uint8_t i2s_rx_pin_ext[3];       // I2S RX data GPIOs, stereo pairs 1..3 (0 = unset)

    // ADAT bulk output (V23+, struct grows by 2 bytes).  Same 0 = unset
    // semantics as i2s_rx_pin for adat_pin; gated on version >= 23 in
    // io_config_from_slot() so pre-V23 slots never decode these as configured.
    // Fields are always stored (both platforms); RP2040 ignores them on apply.
    uint8_t adat_enabled;            // 0/1 configured ADAT enable
    uint8_t adat_pin;                // ADAT data GPIO (0 = unset → PICO_ADAT_PIN)

    // Optional SPDIF inputs 2/3 (V24+, struct grows by 3 bytes).  Same 0 =
    // unset semantics for the pins; gated on version >= 24 in
    // io_config_from_slot() so older slots load with both inputs disabled.
    // SPDIF 4 reuses bit 2 of this mask but stores its pin in the V35 tail
    // byte (this array is mid-struct and cannot grow without moving fields).
    uint8_t spdif_rx_enabled_ext;    // Enable mask: bit 0 = SPDIF2 .. bit 2 = SPDIF4
    uint8_t spdif_rx_pin_ext[2];     // SPDIF RX 2/3 GPIOs (0 = unset; defaults 20/21)

    // Leveller channel masks (V25+, struct grows by 2 bytes).  Gated on
    // version >= 25 in apply_slot_to_live(); older slots load the
    // all-channels default (0xFF).
    uint8_t leveller_detector_mask;
    uint8_t leveller_apply_mask;

    // Loudness output mask (V26+, struct grows by 2 bytes).  Gated on
    // version >= 26 in apply_slot_to_live(); older slots load the
    // all-outputs default (0xFFFF).
    uint16_t loudness_output_mask;

    // Crossfeed output pair mask (V27+, struct grows by 1 byte).  Gated on
    // version >= 27 in apply_slot_to_live(); older slots load the
    // pair-1-only default (0x01).
    uint8_t crossfeed_output_pair_mask;

    // I2S clock master/slave mode (V28+, struct grows by 1 byte).  Plain 0/1
    // (0=master, 1=slave); no +1 sentinel is needed because 0 (master) is the
    // correct default for a missing/zero-filled byte in pre-V28 slots.  Gated on
    // version >= 28 in io_config_from_slot() so older slots leave it at master.
    uint8_t i2s_clock_mode;          // 0=master, 1=slave

    // I2S clock-pin mode + slave-pair BCK (V29+, struct grows by 2 bytes).
    // i2s_clock_pin_mode is plain 0/1 (0=unified, 1=split); 0 (unified) is the
    // correct default for a missing/zero-filled byte, so no sentinel is needed.
    // i2s_bck_pin_slave uses the same 0 = unset convention as i2s_rx_pin
    // (falls back to the device baseline / PICO_I2S_BCK_PIN_SLAVE).  Both are
    // gated on version >= 29 in io_config_from_slot() so older slots leave the
    // live clock-pin mode untouched.
    uint8_t i2s_clock_pin_mode;      // 0=unified, 1=split
    uint8_t i2s_bck_pin_slave;       // slave-mode BCK GPIO (0 = unset → default)

    // Linkwitz Transform per-band target Q (V30+, struct grows by
    // NUM_CHANNELS*MAX_BANDS*2 bytes).  Stored as Q*512; 0 selects the 0.707
    // default.  Gated on version >= 30 in apply_slot_to_live(); older slots
    // have no qp data and load all-zero (= 0.707 default) for every band.
    uint16_t peq_qp_x512[NUM_CHANNELS][MAX_BANDS];

    // V31: psychoacoustic bass (one global config + output mask; see psybass.h)
    uint8_t  psybass_enabled;
    uint8_t  psybass_reserved;
    uint16_t psybass_output_mask;
    float    psybass_cutoff_hz;
    float    psybass_harmonics_db;
    float    psybass_drive_db;
    float    psybass_character_pct;
    float    psybass_original_db;

    // ADAT input (V32+, struct grows by 3 bytes).  Raw values: adat_input_pin
    // 0xFF = unset; adat_input_enabled 0/1; adat_input_clock_mode 0=master,
    // 1=slave.  Gated on version >= 32 in io_config_from_slot() so older slots
    // decode as disabled / pin unset / master (a zero-filled adat_input_pin
    // would misread as GPIO 0, hence the version gate rather than a 0 sentinel).
    // Fields are always stored (both platforms); RP2040 keeps them for
    // round-trips but never selects the source.
    uint8_t adat_input_pin;          // 0xFF = unset
    uint8_t adat_input_enabled;      // 0/1
    uint8_t adat_input_clock_mode;   // 0=master, 1=slave

    // Stereo upmixer (V33+, struct grows by 44 bytes; RP2350-only feature; see
    // upmix.h).  Layout mirrors UpmixConfigPacket: 4 bytes of flags/modes then
    // ten floats.  Fields are always stored (both platforms) for layout
    // uniformity; RP2040 writes zeros and never applies them.  Gated on version
    // >= 33 in apply_slot_to_live() so older slots load the disabled defaults.
    uint8_t upmix_enabled;           // 0/1
    uint8_t upmix_center_mode;       // UPMIX_CENTER_* (0..2; OFF = 2 added
                                     // without a layout change, so firmware
                                     // predating it clamps the byte to the
                                     // ADAPTIVE default rather than misreading)
    uint8_t upmix_surround_mode;     // UPMIX_SURROUND_* (0..2)
    int8_t  upmix_presence_q1;       // V34+: presence bell dB * 2 (was reserved,
                                     // always 0 in V33 slots = 0 dB default)
    float   upmix_strength_pct;
    float   upmix_center_width_pct;
    float   upmix_corr_threshold_pct;
    float   upmix_attack_ms;
    float   upmix_release_ms;
    float   upmix_detector_hpf_hz;
    float   upmix_surround_delay_ms;
    float   upmix_surround_hpf_hz;
    float   upmix_surround_lpf_hz;
    float   upmix_decorr_pct;

    // SPDIF input 4 GPIO (V35+, struct grows by 1 byte).  Same 0 = unset
    // semantics as spdif_rx_pin_ext[]; gated on version >= 35 in
    // io_config_from_slot() so older slots keep the device-level baseline.
    // The matching enable bit is bit 2 of spdif_rx_enabled_ext above, which
    // pre-V35 firmware always wrote as 0.
    uint8_t spdif_rx_pin4;
} PresetSlot;

// The whole slot must fit its 2-sector (8 KB) flash allocation.
_Static_assert(sizeof(PresetSlot) <= SLOT_BYTES,
               "PresetSlot must fit within its 2-sector flash allocation");

// (The pre-preset single-sector "legacy" format and its migration were removed
//  at V21 — compatibility is intentionally broken; boot loads factory defaults
//  when no valid V21 directory/slot is present.)

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
extern volatile uint16_t loudness_output_mask;
extern volatile CrossfeedConfig crossfeed_config;
extern volatile bool crossfeed_update_pending;
extern volatile LevellerConfig leveller_config;
extern volatile bool leveller_update_pending;
extern volatile bool leveller_reset_pending;
extern MatrixMixer matrix_mixer;
extern uint8_t output_pins[NUM_PIN_OUTPUTS];
// Physical IO config live globals (defined in usb_audio.c; spdif_rx_pin and
// active_input_source come from audio_input.h).  Owned by the output_config_mode
// mechanism below.
extern uint8_t  output_types[];
extern uint8_t  i2s_bck_pin;
extern uint8_t  i2s_mck_pin;
extern bool     i2s_mck_enabled;
extern uint16_t i2s_mck_multiplier;
extern volatile bool spdif_rx_pin_change_pending;
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
static inline void dir_apply_dac_hw_mute_defaults(void);  // defined below
static void io_config_defaults(FlashOutputConfig *cfg);    // defined below (IO config section)
static void ctrl_iface_defaults(UartCtrlConfig *u, I2cCtrlConfig *i);  // defined below
static void dir_sanitize_ctrl_iface(void);                            // defined below
static void dir_sanitize_cs_config(void);                             // defined below
static void dir_sanitize_cs_ir(void);                                 // defined below
static void cs_config_from_v1(CsFlashConfig *dst, const CsFlashConfig_v1 *src);  // defined below
static void cs_ir_from_v1(CsIrConfig *dst, const CsIrConfig_v1 *src);            // defined below
// Forward declaration — defined alongside validate_slot() in the SLOT
// VALIDATION section.  collect_live_state() and migrate_legacy() use it
// to compute the CRC byte range that matches whatever version they're
// writing, so the same range is used at save and read time.
static size_t slot_data_size_for_version(uint8_t version);

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
//
// When the DAC hardware-mute hold is still pending, size the counter to
// outlast it (same floor as prepare_pipeline_reset in main.c).  This re-arm
// runs after every flash write and is the last thing to touch
// preset_mute_counter before the completion path, so it must uphold the
// invariant on its own: on a non-streaming source the flash brackets skip
// the settle loop (nothing to drain), leaving the hold un-elapsed at
// completion; if the source then locks within the hold, its poll would burn
// a ~10 ms counter and auto-clear preset_loading before the prefill block
// (gated on dac_hw_mute_hold_elapsed) is allowed to run, and the mute
// release that block owns would never happen.  When the source was
// streaming, the settle loop already waited the hold out and this floor
// stays disengaged.
static inline uint32_t flash_mute_hold_samples(void) {
    uint32_t ms = 10u;
    if (!dac_hw_mute_hold_elapsed()) {
        ms += (uint32_t)dac_hw_mute_hold_ms() + PRESET_MUTE_HOLD_MARGIN_MS;
    }
    uint64_t samples = ((uint64_t)audio_state.freq * ms + 999u) / 1000u;
    if (samples < 512u) samples = 512u;
    return (uint32_t)samples;
}

// ============================================================================
// SELECTIVE FLASH IRQ BLACKOUT
// ============================================================================
//
// Masks at the NVIC instead of PRIMASK across the ~45 ms erase/program window,
// keeping exactly the two output DMA IRQ lines alive so every slot keeps
// clocking framed silence while XIP is unavailable.  Full rationale (why the
// clocks must not halt, what the wire carries, what stays masked and why):
// Documentation/current_architecture.md "Selective NVIC blackout" and
// Documentation/Features/silent_state_changes_spec.md section 4.
//
// Constraints a future edit must not break:
//   - Everything reachable from the two kept handlers must stay RAM-resident
//     and read no flash data; scripts/check_ram_placement.py enforces this
//     (FLASH_WINDOW_ROOTS in Check B2 is a hard failure).
//   - ICER/ISER are written directly, never via irq_set_mask_n_enabled():
//     the SDK helper clears pending bits on re-enable, and IRQs latched
//     during the window must survive to be serviced after it.
//   - (RP2350) NVIC word 1 must be masked too: UART/I2C control ISRs live
//     there and are flash-resident, so one firing mid-window faults.
//   - In IRQ context NVIC masking cannot guarantee the DMA handlers preempt
//     the current exception frame, so the PRIMASK fallback stays (defensive;
//     every runtime flash write is deferred to the main loop).
_Static_assert((DMA_IRQ_0 + PICO_AUDIO_SPDIF_DMA_IRQ) < 32 &&
               (DMA_IRQ_0 + PICO_AUDIO_I2S_DMA_IRQ) < 32,
               "output DMA IRQs must live in NVIC word 0");

#define FLASH_BLACKOUT_KEEP_MASK                              \
    ((1u << (DMA_IRQ_0 + PICO_AUDIO_SPDIF_DMA_IRQ)) |         \
     (1u << (DMA_IRQ_0 + PICO_AUDIO_I2S_DMA_IRQ)))

// The register shape differs per core: RP2040 (M0+) has scalar iser/icer,
// RP2350 (M33) has iser[2]/icer[2].  The keep mask lives entirely in word 0
// (asserted above); on RP2350 word 1 is masked wholesale.
#if PICO_RP2350
#define FLASH_BLACKOUT_ISER   (nvic_hw->iser[0])
#define FLASH_BLACKOUT_ICER   (nvic_hw->icer[0])
#define FLASH_BLACKOUT_ISER1  (nvic_hw->iser[1])
#define FLASH_BLACKOUT_ICER1  (nvic_hw->icer[1])
#else
#define FLASH_BLACKOUT_ISER  (nvic_hw->iser)
#define FLASH_BLACKOUT_ICER  (nvic_hw->icer)
#endif

static uint32_t flash_blackout_saved_iser = 0;
#if PICO_RP2350
static uint32_t flash_blackout_saved_iser1 = 0;
#endif
static uint32_t flash_blackout_primask = 0;
static bool     flash_blackout_used_primask = false;

// RAM-resident: they bracket calls that quiesce XIP, and keeping them out of
// flash removes any question about the return path.
static void __no_inline_not_in_flash_func(flash_irq_blackout_begin)(void) {
    if (__get_current_exception() != 0) {
        flash_blackout_used_primask = true;
        flash_blackout_primask = save_and_disable_interrupts();
        return;
    }
    flash_blackout_used_primask = false;
#if PICO_RP2350
    // Word 1 first, reverse of the restore order below.
    flash_blackout_saved_iser1 = FLASH_BLACKOUT_ISER1;
    FLASH_BLACKOUT_ICER1 = flash_blackout_saved_iser1;
#endif
    flash_blackout_saved_iser = FLASH_BLACKOUT_ISER;
    FLASH_BLACKOUT_ICER = flash_blackout_saved_iser & ~FLASH_BLACKOUT_KEEP_MASK;
    __dsb();
    __isb();
}

static void __no_inline_not_in_flash_func(flash_irq_blackout_end)(void) {
    if (flash_blackout_used_primask) {
        flash_blackout_used_primask = false;
        restore_interrupts(flash_blackout_primask);
        return;
    }
    // Writing 1s re-enables exactly what was enabled; pending bits latched
    // during the window are preserved.  Word 0 first: an IRQ enabled by the
    // first write can preempt before the second lands (the barriers do not
    // gate preemption), and a pending word-1 UART/I2C handler must find the
    // core IRQs already restored.
    FLASH_BLACKOUT_ISER = flash_blackout_saved_iser;
#if PICO_RP2350
    FLASH_BLACKOUT_ISER1 = flash_blackout_saved_iser1;
#endif
    __dsb();
    __isb();
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

// Erase the sector(s) covering a record and write data into them.
// `offset` is the byte offset from the start of flash (not XIP address).
// `data`/`len` specify the payload; it is zero-padded up to page alignment.
// Records up to SLOT_BYTES (a 2-sector preset slot) are supported; the erase
// rounds up to a full sector boundary so a >4 KB slot erases both its sectors.
static int flash_write_sector(uint32_t offset, const void *data, size_t len) {
    // Program size rounds to a page; erase size rounds to a sector.
    size_t write_size = (len + FLASH_PAGE_SIZE - 1)   & ~(FLASH_PAGE_SIZE - 1);
    size_t erase_size = (len + FLASH_SECTOR_SIZE - 1) & ~(FLASH_SECTOR_SIZE - 1);

    // Page-aligned scratch buffer sized for the largest record (a 2-sector slot).
    static uint8_t __attribute__((aligned(256))) write_buf[SLOT_BYTES];
    if (len > sizeof(write_buf)) return -1;   // defensive: record too large
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
    // internals are unsafe. IRQ-context saves are safe because Core 1's
    // entire execution set is RAM-resident (enforced by
    // scripts/check_ram_placement.py); core-1 lockout is still used when
    // not in IRQ context.
    bool do_lockout = multicore_lockout_victim_is_initialized(1)
                      && (__get_current_exception() == 0);
    if (do_lockout) multicore_lockout_start_blocking();

    // PDM's ring DMA free-runs through the window; with Core 1 parked nothing
    // refills it, so it would loop whatever modulator output is left in the
    // ring.  Fill it with true silence and force a lead re-anchor on resume
    // (see pdm_generator.c).  Only when we actually parked Core 1: if it keeps
    // running (its whole execution set is RAM-resident) it goes on filling the
    // ring itself, and a forced re-anchor would then be a gratuitous PDM phase
    // jump against the other slots.
    if (do_lockout) pdm_flash_silence();

    // Selective blackout: the output DMA IRQ lines stay live so every slot
    // keeps clocking framed silence for the whole erase/program window.
    flash_irq_blackout_begin();
    dspi_flash_range_erase(offset, erase_size);
    dspi_flash_range_program(offset, write_buf, write_size);
    flash_irq_blackout_end();

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
        // Defense against corrupt/hand-edited flash: bound-check the control
        // interface structs, resetting any implausible one to defaults.
        dir_sanitize_ctrl_iface();
        dir_sanitize_cs_config();
        dir_sanitize_cs_ir();
        dir_cache_valid = true;
        return true;
    }

    if (flash_dir->version == 16) {
        // V16 -> V17 migration.  V17 doubles the IR command table to 16
        // sub-slots (CsIrConfig format v2), growing the directory's trailing
        // cs_ir block from 132 to 260 bytes.  Everything before it is
        // byte-identical, so copy the whole prefix and widen cs_ir; the new
        // sub-slots start empty (CS_IR_PROTO_NONE).
        const PresetDirectory_v16 *v16 = (const PresetDirectory_v16 *)flash_dir;
        const uint8_t *v16_data_start = (const uint8_t *)&v16->startup_mode;
        size_t v16_data_len = sizeof(PresetDirectory_v16) - offsetof(PresetDirectory_v16, startup_mode);
        if (crc32(v16_data_start, v16_data_len) != v16->crc32) {
            dir_cache_valid = false;
            return false;
        }
        memset(&dir_cache, 0, sizeof(dir_cache));
        // Header excluded: dir_flush() restamps magic/version/crc, and copying
        // the old one would leave the cache reading V16 until it does.
        memcpy(&dir_cache.startup_mode, v16_data_start,
               offsetof(PresetDirectory_v16, cs_ir) - offsetof(PresetDirectory_v16, startup_mode));
        cs_ir_from_v1(&dir_cache.cs_ir, &v16->cs_ir);
        dir_sanitize_ctrl_iface();
        dir_sanitize_cs_config();
        dir_sanitize_cs_ir();
        dir_cache_valid = true;
        (void)dir_flush();   // persist at the current version
        return true;
    }

    if (flash_dir->version == 15) {
        // V15 -> V16 migration.  V16 grows the device-global output_config by
        // 1 byte (SPDIF input 4: spdif_rx_pin4).  Validate the v15 CRC, copy
        // every field forward, and widen the 34-byte output_config by prefix
        // memcpy; the new tail byte stays 0 from the memset, which is the
        // "unset" sentinel and resolves to PICO_SPDIF_RX_PIN4_DEFAULT on apply.
        const PresetDirectory_v15 *v15 = (const PresetDirectory_v15 *)flash_dir;
        const uint8_t *v15_data_start = (const uint8_t *)&v15->startup_mode;
        size_t v15_data_len = sizeof(PresetDirectory_v15) - offsetof(PresetDirectory_v15, startup_mode);
        if (crc32(v15_data_start, v15_data_len) != v15->crc32) {
            dir_cache_valid = false;
            return false;
        }
        memset(&dir_cache, 0, sizeof(dir_cache));
        dir_cache.startup_mode       = v15->startup_mode;
        dir_cache.default_slot       = v15->default_slot;
        dir_cache.last_active_slot   = v15->last_active_slot;
        dir_cache.output_config_mode = v15->output_config_mode;
        dir_cache.slot_occupied      = v15->slot_occupied;
        dir_cache.master_volume_mode = v15->master_volume_mode;
        dir_cache.spdif_rx_pin       = v15->spdif_rx_pin;
        dir_cache.master_volume_db   = v15->master_volume_db;
        memcpy(dir_cache.slot_names, v15->slot_names, sizeof(dir_cache.slot_names));
        dir_cache.dac_hw_mute        = v15->dac_hw_mute;
        memcpy(&dir_cache.output_config, &v15->output_config, sizeof(v15->output_config));
        dir_cache.uart_ctrl          = v15->uart_ctrl;
        dir_cache.i2c_ctrl           = v15->i2c_ctrl;
        dir_cache.cs_config          = v15->cs_config;
        memcpy(dir_cache.cs_names, v15->cs_names, sizeof(dir_cache.cs_names));
        cs_ir_from_v1(&dir_cache.cs_ir, &v15->cs_ir);
        dir_sanitize_ctrl_iface();
        dir_sanitize_cs_config();
        dir_sanitize_cs_ir();
        dir_cache_valid = true;
        (void)dir_flush();   // persist at the current version
        return true;
    }

    if (flash_dir->version == 14) {
        // V14 -> V15 migration.  V15 grows the device-global output_config by
        // 3 bytes (ADAT input: adat_input_pin + adat_input_enabled +
        // adat_input_clock_mode).  Validate the v14 CRC, copy every field
        // forward, and widen the 31-byte output_config by prefix memcpy; the new
        // tail bytes stay 0 from the memset (= disabled + master), then seed
        // adat_input_pin to 0xFF (the zero-fill 0 would misread as GPIO 0).
        const PresetDirectory_v14 *v14 = (const PresetDirectory_v14 *)flash_dir;
        const uint8_t *v14_data_start = (const uint8_t *)&v14->startup_mode;
        size_t v14_data_len = sizeof(PresetDirectory_v14) - offsetof(PresetDirectory_v14, startup_mode);
        if (crc32(v14_data_start, v14_data_len) != v14->crc32) {
            dir_cache_valid = false;
            return false;
        }
        memset(&dir_cache, 0, sizeof(dir_cache));
        dir_cache.startup_mode       = v14->startup_mode;
        dir_cache.default_slot       = v14->default_slot;
        dir_cache.last_active_slot   = v14->last_active_slot;
        dir_cache.output_config_mode = v14->output_config_mode;
        dir_cache.slot_occupied      = v14->slot_occupied;
        dir_cache.master_volume_mode = v14->master_volume_mode;
        dir_cache.spdif_rx_pin       = v14->spdif_rx_pin;
        dir_cache.master_volume_db   = v14->master_volume_db;
        memcpy(dir_cache.slot_names, v14->slot_names, sizeof(dir_cache.slot_names));
        dir_cache.dac_hw_mute        = v14->dac_hw_mute;
        memcpy(&dir_cache.output_config, &v14->output_config, sizeof(v14->output_config));
        dir_cache.output_config.adat_input_pin = 0xFF;   // unset (not the zero-fill 0)
        dir_cache.uart_ctrl          = v14->uart_ctrl;
        dir_cache.i2c_ctrl           = v14->i2c_ctrl;
        dir_cache.cs_config          = v14->cs_config;
        memcpy(dir_cache.cs_names, v14->cs_names, sizeof(dir_cache.cs_names));
        cs_ir_from_v1(&dir_cache.cs_ir, &v14->cs_ir);
        dir_sanitize_ctrl_iface();
        dir_sanitize_cs_config();
        dir_sanitize_cs_ir();
        dir_cache_valid = true;
        (void)dir_flush();   // persist at the current version
        return true;
    }

    if (flash_dir->version == 13) {
        // V13 -> V14 migration.  V14 grows the device-global output_config by
        // 2 bytes (I2S clock-pin mode: i2s_clock_pin_mode + i2s_bck_pin_slave).
        // Validate the v13 CRC, copy every field forward, and widen the 29-byte
        // output_config by prefix memcpy; the new bytes stay 0 (= unified +
        // unset) from the memset, which are the correct defaults.
        const PresetDirectory_v13 *v13 = (const PresetDirectory_v13 *)flash_dir;
        const uint8_t *v13_data_start = (const uint8_t *)&v13->startup_mode;
        size_t v13_data_len = sizeof(PresetDirectory_v13) - offsetof(PresetDirectory_v13, startup_mode);
        if (crc32(v13_data_start, v13_data_len) != v13->crc32) {
            dir_cache_valid = false;
            return false;
        }
        memset(&dir_cache, 0, sizeof(dir_cache));
        dir_cache.startup_mode       = v13->startup_mode;
        dir_cache.default_slot       = v13->default_slot;
        dir_cache.last_active_slot   = v13->last_active_slot;
        dir_cache.output_config_mode = v13->output_config_mode;
        dir_cache.slot_occupied      = v13->slot_occupied;
        dir_cache.master_volume_mode = v13->master_volume_mode;
        dir_cache.spdif_rx_pin       = v13->spdif_rx_pin;
        dir_cache.master_volume_db   = v13->master_volume_db;
        memcpy(dir_cache.slot_names, v13->slot_names, sizeof(dir_cache.slot_names));
        dir_cache.dac_hw_mute        = v13->dac_hw_mute;
        memcpy(&dir_cache.output_config, &v13->output_config, sizeof(v13->output_config));
        dir_cache.output_config.adat_input_pin = 0xFF;   // V15 field: unset (not zero-fill 0)
        dir_cache.uart_ctrl          = v13->uart_ctrl;
        dir_cache.i2c_ctrl           = v13->i2c_ctrl;
        dir_cache.cs_config          = v13->cs_config;
        memcpy(dir_cache.cs_names, v13->cs_names, sizeof(dir_cache.cs_names));
        cs_ir_from_v1(&dir_cache.cs_ir, &v13->cs_ir);
        dir_sanitize_ctrl_iface();
        dir_sanitize_cs_config();
        dir_sanitize_cs_ir();
        dir_cache_valid = true;
        (void)dir_flush();   // persist at the current version
        return true;
    }

    if (flash_dir->version == 12) {
        // V12 -> V14 migration.  V13 grew the device-global output_config by
        // 1 byte (I2S clock master/slave mode: i2s_clock_mode) and V14 by 2
        // more (I2S clock-pin mode).  Validate the v12 CRC, copy every field
        // forward, and widen the 28-byte output_config by prefix memcpy; the
        // new bytes stay 0 (= master + unified + unset) from the memset,
        // which are the correct defaults.
        const PresetDirectory_v12 *v12 = (const PresetDirectory_v12 *)flash_dir;
        const uint8_t *v12_data_start = (const uint8_t *)&v12->startup_mode;
        size_t v12_data_len = sizeof(PresetDirectory_v12) - offsetof(PresetDirectory_v12, startup_mode);
        if (crc32(v12_data_start, v12_data_len) != v12->crc32) {
            dir_cache_valid = false;
            return false;
        }
        memset(&dir_cache, 0, sizeof(dir_cache));
        dir_cache.startup_mode       = v12->startup_mode;
        dir_cache.default_slot       = v12->default_slot;
        dir_cache.last_active_slot   = v12->last_active_slot;
        dir_cache.output_config_mode = v12->output_config_mode;
        dir_cache.slot_occupied      = v12->slot_occupied;
        dir_cache.master_volume_mode = v12->master_volume_mode;
        dir_cache.spdif_rx_pin       = v12->spdif_rx_pin;
        dir_cache.master_volume_db   = v12->master_volume_db;
        memcpy(dir_cache.slot_names, v12->slot_names, sizeof(dir_cache.slot_names));
        dir_cache.dac_hw_mute        = v12->dac_hw_mute;
        memcpy(&dir_cache.output_config, &v12->output_config, sizeof(v12->output_config));
        dir_cache.output_config.adat_input_pin = 0xFF;   // V15 field: unset (not zero-fill 0)
        dir_cache.uart_ctrl          = v12->uart_ctrl;
        dir_cache.i2c_ctrl           = v12->i2c_ctrl;
        dir_cache.cs_config          = v12->cs_config;
        memcpy(dir_cache.cs_names, v12->cs_names, sizeof(dir_cache.cs_names));
        cs_ir_from_v1(&dir_cache.cs_ir, &v12->cs_ir);
        dir_sanitize_ctrl_iface();
        dir_sanitize_cs_config();
        dir_sanitize_cs_ir();
        dir_cache_valid = true;
        (void)dir_flush();   // persist at the current version
        return true;
    }

    if (flash_dir->version == 11) {
        // V11 -> V12 migration.  V12 grows the device-global output_config by
        // 3 bytes (optional SPDIF inputs 2/3).  Validate the v11 CRC, copy
        // every field forward, and widen the 25-byte output_config by prefix
        // memcpy; the new tail bytes stay zero from the memset (both extra
        // inputs disabled, pins unset = platform defaults on apply).
        const PresetDirectory_v11 *v11 = (const PresetDirectory_v11 *)flash_dir;
        const uint8_t *v11_data_start = (const uint8_t *)&v11->startup_mode;
        size_t v11_data_len = sizeof(PresetDirectory_v11) - offsetof(PresetDirectory_v11, startup_mode);
        if (crc32(v11_data_start, v11_data_len) != v11->crc32) {
            dir_cache_valid = false;
            return false;
        }
        memset(&dir_cache, 0, sizeof(dir_cache));
        dir_cache.startup_mode       = v11->startup_mode;
        dir_cache.default_slot       = v11->default_slot;
        dir_cache.last_active_slot   = v11->last_active_slot;
        dir_cache.output_config_mode = v11->output_config_mode;
        dir_cache.slot_occupied      = v11->slot_occupied;
        dir_cache.master_volume_mode = v11->master_volume_mode;
        dir_cache.spdif_rx_pin       = v11->spdif_rx_pin;
        dir_cache.master_volume_db   = v11->master_volume_db;
        memcpy(dir_cache.slot_names, v11->slot_names, sizeof(dir_cache.slot_names));
        dir_cache.dac_hw_mute        = v11->dac_hw_mute;
        memcpy(&dir_cache.output_config, &v11->output_config, sizeof(v11->output_config));
        dir_cache.output_config.adat_input_pin = 0xFF;   // V15 field: unset (not zero-fill 0)
        dir_cache.uart_ctrl          = v11->uart_ctrl;
        dir_cache.i2c_ctrl           = v11->i2c_ctrl;
        dir_cache.cs_config          = v11->cs_config;
        memcpy(dir_cache.cs_names, v11->cs_names, sizeof(dir_cache.cs_names));
        cs_ir_from_v1(&dir_cache.cs_ir, &v11->cs_ir);
        dir_sanitize_ctrl_iface();
        dir_sanitize_cs_config();
        dir_sanitize_cs_ir();
        dir_cache_valid = true;
        (void)dir_flush();   // persist at the current version
        return true;
    }

    if (flash_dir->version == 10) {
        // V10 -> V12 migration.  V11 appended the Control Surfaces IR command
        // table (132 bytes); V12 grew output_config (see the V11 branch).
        // Validate the v10 CRC, copy every field forward, and leave the new
        // blocks zeroed (every IR sub-slot empty, SPDIF 2/3 disabled).
        const PresetDirectory_v10 *v10 = (const PresetDirectory_v10 *)flash_dir;
        const uint8_t *v10_data_start = (const uint8_t *)&v10->startup_mode;
        size_t v10_data_len = sizeof(PresetDirectory_v10) - offsetof(PresetDirectory_v10, startup_mode);
        if (crc32(v10_data_start, v10_data_len) != v10->crc32) {
            dir_cache_valid = false;
            return false;
        }
        memset(&dir_cache, 0, sizeof(dir_cache));
        dir_cache.startup_mode       = v10->startup_mode;
        dir_cache.default_slot       = v10->default_slot;
        dir_cache.last_active_slot   = v10->last_active_slot;
        dir_cache.output_config_mode = v10->output_config_mode;
        dir_cache.slot_occupied      = v10->slot_occupied;
        dir_cache.master_volume_mode = v10->master_volume_mode;
        dir_cache.spdif_rx_pin       = v10->spdif_rx_pin;
        dir_cache.master_volume_db   = v10->master_volume_db;
        memcpy(dir_cache.slot_names, v10->slot_names, sizeof(dir_cache.slot_names));
        dir_cache.dac_hw_mute        = v10->dac_hw_mute;
        // 25-byte v11 config into the 28-byte field; SPDIF 2/3 bytes stay 0
        memcpy(&dir_cache.output_config, &v10->output_config, sizeof(v10->output_config));
        dir_cache.output_config.adat_input_pin = 0xFF;   // V15 field: unset (not zero-fill 0)
        dir_cache.uart_ctrl          = v10->uart_ctrl;
        dir_cache.i2c_ctrl           = v10->i2c_ctrl;
        dir_cache.cs_config          = v10->cs_config;   // already format v2
        memcpy(dir_cache.cs_names, v10->cs_names, sizeof(dir_cache.cs_names));
        // cs_ir stays zeroed from the memset (all sub-slots empty).
        dir_sanitize_ctrl_iface();
        dir_sanitize_cs_config();
        dir_sanitize_cs_ir();
        dir_cache_valid = true;
        (void)dir_flush();   // persist at the current version
        return true;
    }

    if (flash_dir->version == 9) {
        // V9 → V10 migration.  V10 appends the per-slot Control Surfaces
        // names (16 x 32 bytes).  Validate the v9 CRC, copy every field
        // forward, and leave the new block zeroed (all slots unnamed).
        const PresetDirectory_v9 *v9 = (const PresetDirectory_v9 *)flash_dir;
        const uint8_t *v9_data_start = (const uint8_t *)&v9->startup_mode;
        size_t v9_data_len = sizeof(PresetDirectory_v9) - offsetof(PresetDirectory_v9, startup_mode);
        if (crc32(v9_data_start, v9_data_len) != v9->crc32) {
            dir_cache_valid = false;
            return false;
        }
        memset(&dir_cache, 0, sizeof(dir_cache));
        dir_cache.startup_mode       = v9->startup_mode;
        dir_cache.default_slot       = v9->default_slot;
        dir_cache.last_active_slot   = v9->last_active_slot;
        dir_cache.output_config_mode = v9->output_config_mode;
        dir_cache.slot_occupied      = v9->slot_occupied;
        dir_cache.master_volume_mode = v9->master_volume_mode;
        dir_cache.spdif_rx_pin       = v9->spdif_rx_pin;
        dir_cache.master_volume_db   = v9->master_volume_db;
        memcpy(dir_cache.slot_names, v9->slot_names, sizeof(dir_cache.slot_names));
        dir_cache.dac_hw_mute        = v9->dac_hw_mute;
        // 25-byte v11 config into the 31-byte field; new tail bytes stay 0
        memcpy(&dir_cache.output_config, &v9->output_config, sizeof(v9->output_config));
        dir_cache.output_config.adat_input_pin = 0xFF;   // V15 field: unset (not zero-fill 0)
        dir_cache.uart_ctrl          = v9->uart_ctrl;
        dir_cache.i2c_ctrl           = v9->i2c_ctrl;
        dir_cache.cs_config          = v9->cs_config;   // already format v2
        // cs_ir stays zeroed from the memset (all sub-slots empty).
        dir_sanitize_ctrl_iface();
        dir_sanitize_cs_config();
        dir_sanitize_cs_ir();
        dir_cache_valid = true;
        (void)dir_flush();   // persist at the current version
        return true;
    }

    if (flash_dir->version == 8) {
        // V8 → V9 migration.  V9 upgrades the embedded Control Surfaces config
        // from format v1 (132 bytes: 8 x 16-byte bindings) to v2 (388 bytes:
        // 16 x 24-byte bindings).  Everything else is unchanged, so validate the
        // v8 CRC, copy every field forward, and translate cs_config field-by-
        // field via cs_config_from_v1().
        const PresetDirectory_v8 *v8 = (const PresetDirectory_v8 *)flash_dir;
        const uint8_t *v8_data_start = (const uint8_t *)&v8->startup_mode;
        size_t v8_data_len = sizeof(PresetDirectory_v8) - offsetof(PresetDirectory_v8, startup_mode);
        if (crc32(v8_data_start, v8_data_len) != v8->crc32) {
            dir_cache_valid = false;
            return false;
        }
        memset(&dir_cache, 0, sizeof(dir_cache));
        dir_cache.startup_mode       = v8->startup_mode;
        dir_cache.default_slot       = v8->default_slot;
        dir_cache.last_active_slot   = v8->last_active_slot;
        dir_cache.output_config_mode = v8->output_config_mode;
        dir_cache.slot_occupied      = v8->slot_occupied;
        dir_cache.master_volume_mode = v8->master_volume_mode;
        dir_cache.spdif_rx_pin       = v8->spdif_rx_pin;
        dir_cache.master_volume_db   = v8->master_volume_db;
        memcpy(dir_cache.slot_names, v8->slot_names, sizeof(dir_cache.slot_names));
        dir_cache.dac_hw_mute        = v8->dac_hw_mute;
        // 25-byte v11 config into the 31-byte field; new tail bytes stay 0
        memcpy(&dir_cache.output_config, &v8->output_config, sizeof(v8->output_config));
        dir_cache.output_config.adat_input_pin = 0xFF;   // V15 field: unset (not zero-fill 0)
        dir_cache.uart_ctrl          = v8->uart_ctrl;
        dir_cache.i2c_ctrl           = v8->i2c_ctrl;
        cs_config_from_v1(&dir_cache.cs_config, &v8->cs_config);
        dir_cache_valid = true;
        (void)dir_flush();   // persist at the current version
        return true;
    }

    if (flash_dir->version == 7) {
        // V7 → V9 migration.  V8 grew the device-global output_config by 2 bytes
        // (ADAT bulk output: adat_enabled + adat_pin) and V9 upgraded the
        // Control Surfaces config from format v1 to v2 (see the V8 branch).
        // Validate the v7 CRC, copy every field forward, widen the 23-byte v7
        // output_config into the 25-byte field (new adat bytes stay zero from
        // the memset, then adat_pin is seeded to the platform default with the
        // feature disabled), and translate cs_config via cs_config_from_v1().
        const PresetDirectory_v7 *v7 = (const PresetDirectory_v7 *)flash_dir;
        const uint8_t *v7_data_start = (const uint8_t *)&v7->startup_mode;
        size_t v7_data_len = sizeof(PresetDirectory_v7) - offsetof(PresetDirectory_v7, startup_mode);
        if (crc32(v7_data_start, v7_data_len) != v7->crc32) {
            dir_cache_valid = false;
            return false;
        }
        memset(&dir_cache, 0, sizeof(dir_cache));
        dir_cache.startup_mode       = v7->startup_mode;
        dir_cache.default_slot       = v7->default_slot;
        dir_cache.last_active_slot   = v7->last_active_slot;
        dir_cache.output_config_mode = v7->output_config_mode;
        dir_cache.slot_occupied      = v7->slot_occupied;
        dir_cache.master_volume_mode = v7->master_volume_mode;
        dir_cache.spdif_rx_pin       = v7->spdif_rx_pin;
        dir_cache.master_volume_db   = v7->master_volume_db;
        memcpy(dir_cache.slot_names, v7->slot_names, sizeof(dir_cache.slot_names));
        dir_cache.dac_hw_mute        = v7->dac_hw_mute;
        memcpy(&dir_cache.output_config, &v7->output_config, sizeof(v7->output_config));
        dir_cache.output_config.adat_input_pin = 0xFF;   // V15 field: unset (not zero-fill 0)
        dir_cache.output_config.adat_enabled = 0;
#if PICO_RP2350
        dir_cache.output_config.adat_pin = PICO_ADAT_PIN;
#else
        dir_cache.output_config.adat_pin = 0;
#endif
        dir_cache.uart_ctrl          = v7->uart_ctrl;
        dir_cache.i2c_ctrl           = v7->i2c_ctrl;
        cs_config_from_v1(&dir_cache.cs_config, &v7->cs_config);
        dir_cache_valid = true;
        (void)dir_flush();   // persist at the current version
        return true;
    }

    if (flash_dir->version == 6) {
        // V6 → V7 migration.  V7 appends the Control Surfaces config.
        // Validate the v6 CRC, copy every field forward, and leave the new
        // block zeroed (every slot CS_TYPE_NONE = feature idle).
        const PresetDirectory_v6 *v6 = (const PresetDirectory_v6 *)flash_dir;
        const uint8_t *v6_data_start = (const uint8_t *)&v6->startup_mode;
        size_t v6_data_len = sizeof(PresetDirectory_v6) - offsetof(PresetDirectory_v6, startup_mode);
        if (crc32(v6_data_start, v6_data_len) != v6->crc32) {
            dir_cache_valid = false;
            return false;
        }
        memset(&dir_cache, 0, sizeof(dir_cache));
        dir_cache.startup_mode       = v6->startup_mode;
        dir_cache.default_slot       = v6->default_slot;
        dir_cache.last_active_slot   = v6->last_active_slot;
        dir_cache.output_config_mode = v6->output_config_mode;
        dir_cache.slot_occupied      = v6->slot_occupied;
        dir_cache.master_volume_mode = v6->master_volume_mode;
        dir_cache.spdif_rx_pin       = v6->spdif_rx_pin;
        dir_cache.master_volume_db   = v6->master_volume_db;
        memcpy(dir_cache.slot_names, v6->slot_names, sizeof(dir_cache.slot_names));
        dir_cache.dac_hw_mute        = v6->dac_hw_mute;
        // 23-byte v7 config into the 28-byte field; adat + clock/clock-pin bytes stay 0 (unset/master/unified)
        memcpy(&dir_cache.output_config, &v6->output_config, sizeof(v6->output_config));
        dir_cache.output_config.adat_input_pin = 0xFF;   // V15 field: unset (not zero-fill 0)
        dir_cache.uart_ctrl          = v6->uart_ctrl;
        dir_cache.i2c_ctrl           = v6->i2c_ctrl;
        dir_cache.cs_config.version  = CS_CONFIG_VERSION;
        dir_cache_valid = true;
        (void)dir_flush();   // persist at the current version
        return true;
    }

    if (flash_dir->version == 5) {
        // V5 → V6 migration.  V6 appends the device-level control-interface
        // config (uart_ctrl / i2c_ctrl).  Validate the v5 CRC, copy every field
        // forward, and seed the new blocks with defaults (disabled).
        const PresetDirectory_v5 *v5 = (const PresetDirectory_v5 *)flash_dir;
        const uint8_t *v5_data_start = (const uint8_t *)&v5->startup_mode;
        size_t v5_data_len = sizeof(PresetDirectory_v5) - offsetof(PresetDirectory_v5, startup_mode);
        if (crc32(v5_data_start, v5_data_len) != v5->crc32) {
            dir_cache_valid = false;
            return false;
        }
        memset(&dir_cache, 0, sizeof(dir_cache));
        dir_cache.startup_mode       = v5->startup_mode;
        dir_cache.default_slot       = v5->default_slot;
        dir_cache.last_active_slot   = v5->last_active_slot;
        dir_cache.output_config_mode = v5->output_config_mode;
        dir_cache.slot_occupied      = v5->slot_occupied;
        dir_cache.master_volume_mode = v5->master_volume_mode;
        dir_cache.spdif_rx_pin       = v5->spdif_rx_pin;
        dir_cache.master_volume_db   = v5->master_volume_db;
        memcpy(dir_cache.slot_names, v5->slot_names, sizeof(dir_cache.slot_names));
        dir_cache.dac_hw_mute        = v5->dac_hw_mute;
        // 23-byte v7 config into the 28-byte field; adat + clock/clock-pin bytes stay 0 (unset/master/unified)
        memcpy(&dir_cache.output_config, &v5->output_config, sizeof(v5->output_config));
        dir_cache.output_config.adat_input_pin = 0xFF;   // V15 field: unset (not zero-fill 0)
        ctrl_iface_defaults(&dir_cache.uart_ctrl, &dir_cache.i2c_ctrl);
        dir_cache_valid = true;
        (void)dir_flush();   // persist at the current version
        return true;
    }

    if (flash_dir->version == 4) {
        // V4 → V5 migration.  V5 grows the device-global output_config by 3
        // bytes (I2S multichannel input: i2s_rx_pin_ext[3]).  Validate the v4
        // CRC, copy every field forward, and copy the 20-byte v4 output_config
        // into the 23-byte field — the new i2s_rx_pin_ext bytes stay zero
        // (= unset) from the memset, and i2s_input_channels inherits the old
        // reserved byte (0 = unset → 2 on apply).
        const PresetDirectory_v4 *v4 = (const PresetDirectory_v4 *)flash_dir;
        const uint8_t *v4_data_start = (const uint8_t *)&v4->startup_mode;
        size_t v4_data_len = sizeof(PresetDirectory_v4) - offsetof(PresetDirectory_v4, startup_mode);
        if (crc32(v4_data_start, v4_data_len) != v4->crc32) {
            dir_cache_valid = false;
            return false;
        }
        memset(&dir_cache, 0, sizeof(dir_cache));
        dir_cache.startup_mode       = v4->startup_mode;
        dir_cache.default_slot       = v4->default_slot;
        dir_cache.last_active_slot   = v4->last_active_slot;
        dir_cache.output_config_mode = v4->output_config_mode;
        dir_cache.slot_occupied      = v4->slot_occupied;
        dir_cache.master_volume_mode = v4->master_volume_mode;
        dir_cache.spdif_rx_pin       = v4->spdif_rx_pin;
        dir_cache.master_volume_db   = v4->master_volume_db;
        memcpy(dir_cache.slot_names, v4->slot_names, sizeof(dir_cache.slot_names));
        dir_cache.dac_hw_mute        = v4->dac_hw_mute;
        memcpy(&dir_cache.output_config, &v4->output_config, sizeof(v4->output_config));
        dir_cache.output_config.adat_input_pin = 0xFF;   // V15 field: unset (not zero-fill 0)
        ctrl_iface_defaults(&dir_cache.uart_ctrl, &dir_cache.i2c_ctrl);  // V6 blocks
        dir_cache_valid = true;
        (void)dir_flush();   // persist as V5
        return true;
    }

    if (flash_dir->version == 3) {
        // V3 → V4 migration.  Validate the v3 CRC, copy fields forward, and
        // seed the new device-global output_config from firmware IO defaults.
        // The former include_pins byte maps 1:1 onto output_config_mode
        // (1 → WITH_PRESET, 0 → INDEPENDENT).  In the default WITH_PRESET case
        // output_config is unused (boot applies the slot's IO).  Only a device
        // that had the non-default include_pins=0 lands in INDEPENDENT mode and
        // boots to default routing until the user re-saves via
        // REQ_SAVE_OUTPUT_CONFIG; the device-level SPDIF RX pin is carried into
        // the block so it survives regardless.
        const PresetDirectory_v3 *v3 = (const PresetDirectory_v3 *)flash_dir;
        const uint8_t *v3_data_start = (const uint8_t *)&v3->startup_mode;
        size_t v3_data_len = sizeof(PresetDirectory_v3) - offsetof(PresetDirectory_v3, startup_mode);
        if (crc32(v3_data_start, v3_data_len) != v3->crc32) {
            dir_cache_valid = false;
            return false;
        }
        memset(&dir_cache, 0, sizeof(dir_cache));
        dir_cache.startup_mode       = v3->startup_mode;
        dir_cache.default_slot       = v3->default_slot;
        dir_cache.last_active_slot   = v3->last_active_slot;
        dir_cache.output_config_mode = v3->include_pins;   // 1:1 value mapping
        dir_cache.slot_occupied      = v3->slot_occupied;
        dir_cache.master_volume_mode = v3->master_volume_mode;
        dir_cache.spdif_rx_pin       = v3->spdif_rx_pin;
        dir_cache.master_volume_db   = v3->master_volume_db;
        memcpy(dir_cache.slot_names, v3->slot_names, sizeof(dir_cache.slot_names));
        dir_cache.dac_hw_mute        = v3->dac_hw_mute;    // carry forward as-is
        io_config_defaults(&dir_cache.output_config);
        dir_cache.output_config.spdif_rx_pin = v3->spdif_rx_pin;  // keep device RX pin
        ctrl_iface_defaults(&dir_cache.uart_ctrl, &dir_cache.i2c_ctrl);  // V6 blocks
        dir_cache_valid = true;
        (void)dir_flush();
        return true;
    }

    if (flash_dir->version == 2) {
        // V2 → V3 migration.  Read with the old struct, validate the v2
        // CRC over the v2-sized data range, then copy fields forward
        // and zero-init the V3 dac_hw_mute config (feature off).  Flush
        // immediately so the next boot reads the V3 layout directly.
        const PresetDirectory_v2 *v2 = (const PresetDirectory_v2 *)flash_dir;
        const uint8_t *v2_data_start = (const uint8_t *)&v2->startup_mode;
        size_t v2_data_len = sizeof(PresetDirectory_v2) - offsetof(PresetDirectory_v2, startup_mode);
        if (crc32(v2_data_start, v2_data_len) != v2->crc32) {
            dir_cache_valid = false;
            return false;
        }
        memset(&dir_cache, 0, sizeof(dir_cache));
        dir_cache.startup_mode       = v2->startup_mode;
        dir_cache.default_slot       = v2->default_slot;
        dir_cache.last_active_slot   = v2->last_active_slot;
        dir_cache.output_config_mode = v2->include_pins;   // 1:1 value mapping
        dir_cache.slot_occupied      = v2->slot_occupied;
        dir_cache.master_volume_mode = v2->master_volume_mode;
        dir_cache.spdif_rx_pin       = v2->spdif_rx_pin;
        dir_cache.master_volume_db   = v2->master_volume_db;
        memcpy(dir_cache.slot_names, v2->slot_names, sizeof(dir_cache.slot_names));
        // dac_hw_mute: feature disabled, but pin/polarity/timing pre-set
        // to PCM5102A-friendly defaults so the user only has to flip
        // enabled=1 if their wiring matches the common case.
        dir_apply_dac_hw_mute_defaults();
        io_config_defaults(&dir_cache.output_config);      // V4 device-global IO
        dir_cache.output_config.spdif_rx_pin = v2->spdif_rx_pin;  // keep device RX pin
        ctrl_iface_defaults(&dir_cache.uart_ctrl, &dir_cache.i2c_ctrl);  // V6 blocks
        dir_cache_valid = true;
        (void)dir_flush();
        return true;
    }

    if (flash_dir->version == 1) {
        // Legacy v1 format — read with the old struct, validate old CRC,
        // then migrate forward to V3 in memory and flush.  Same field-copy
        // logic as the v2 path with v1's slightly different field set.
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
        dir_cache.output_config_mode = v1->include_pins;   // 1:1 value mapping
        dir_cache.slot_occupied      = v1->slot_occupied;
        dir_cache.master_volume_mode = v1->include_master_volume
                                         ? MASTER_VOLUME_MODE_WITH_PRESET
                                         : MASTER_VOLUME_MODE_INDEPENDENT;
        dir_cache.master_volume_db   = MASTER_VOL_DEFAULT_DB;
        memcpy(dir_cache.slot_names, v1->slot_names, sizeof(dir_cache.slot_names));
        // dac_hw_mute: feature disabled, but pin/polarity/timing pre-set
        // to PCM5102A-friendly defaults — see v2→v3 path above.
        dir_apply_dac_hw_mute_defaults();
        io_config_defaults(&dir_cache.output_config);      // V4 device-global IO (v1 has no RX pin)
        ctrl_iface_defaults(&dir_cache.uart_ctrl, &dir_cache.i2c_ctrl);  // V6 blocks
        dir_cache_valid = true;
        (void)dir_flush();  // persist as V4; if the flush fails, cache stays valid in RAM
        return true;
    }

    // Unknown future version — treat as invalid.
    dir_cache_valid = false;
    return false;
}

// Populate dir_cache.dac_hw_mute with factory defaults.  enabled = 0 so
// the feature stays off until the user explicitly turns it on, but pin /
// polarity / hold time get sensible defaults (GPIO 11, active-low,
// 5 ms hold — works out-of-the-box for the most common PCM5102A
// breakout).  Used by all three directory-init paths: fresh-flash
// (dir_ensure), v2→v3 migration, and v1→v3 migration.
static inline void dir_apply_dac_hw_mute_defaults(void) {
    dir_cache.dac_hw_mute.enabled    = 0;
    dir_cache.dac_hw_mute.active_low = DAC_HW_MUTE_DEFAULT_ACTIVE_LOW;
    dir_cache.dac_hw_mute.pin        = DAC_HW_MUTE_DEFAULT_PIN;
    dir_cache.dac_hw_mute.reserved0  = 0;
    dir_cache.dac_hw_mute.hold_ms    = DAC_HW_MUTE_DEFAULT_HOLD_MS;
    dir_cache.dac_hw_mute.release_ms = DAC_HW_MUTE_DEFAULT_RELEASE_MS;
    memset(dir_cache.dac_hw_mute.reserved, 0, sizeof(dir_cache.dac_hw_mute.reserved));
}

// Populate the control-interface configs with factory defaults (config.h pins /
// baud / address).  enabled = 0 so both stay off until the user turns them on.
// Either pointer may be NULL to leave that interface untouched.  Used by the
// fresh-flash, V5→V6 migration, and sanitize-on-load paths.
static void ctrl_iface_defaults(UartCtrlConfig *u, I2cCtrlConfig *i) {
    if (u) {
        u->enabled       = 0;
        u->tx_pin        = UART_CTRL_DEFAULT_TX_PIN;
        u->rx_pin        = UART_CTRL_DEFAULT_RX_PIN;
        u->notify_enable = 0;
        u->baud          = UART_CTRL_DEFAULT_BAUD;
    }
    if (i) {
        i->enabled = 0;
        i->sda_pin = I2C_CTRL_DEFAULT_SDA_PIN;
        i->scl_pin = I2C_CTRL_DEFAULT_SCL_PIN;
        i->address = I2C_CTRL_DEFAULT_ADDRESS;
        memset(i->reserved, 0, sizeof(i->reserved));
    }
}

// Bound-check the directory's control-interface structs; any implausible field
// resets that interface to defaults (disabled).  Deeper checks (pin mux,
// collisions) run elsewhere at apply time.
static void dir_sanitize_ctrl_iface(void) {
#if PICO_RP2350
    uint8_t max_pin = (sysinfo_hw->package_sel == 0) ? 47 : 29;
#else
    uint8_t max_pin = 29;
#endif
    UartCtrlConfig *u = &dir_cache.uart_ctrl;
    if (u->enabled > 1 || u->tx_pin > max_pin || u->rx_pin > max_pin ||
        u->baud < UART_CTRL_BAUD_MIN || u->baud > UART_CTRL_BAUD_MAX) {
        ctrl_iface_defaults(u, NULL);
    }
    // notify_enable claims the formerly-reserved byte, which old firmware
    // never validated; clamp a stray value instead of wiping the user's
    // whole stored config over a byte that used to be meaningless.
    if (u->notify_enable > 1) u->notify_enable = 0;
    I2cCtrlConfig *i = &dir_cache.i2c_ctrl;
    if (i->enabled > 1 || i->sda_pin > max_pin || i->scl_pin > max_pin ||
        i->address < I2C_CTRL_ADDRESS_MIN || i->address > I2C_CTRL_ADDRESS_MAX) {
        ctrl_iface_defaults(NULL, i);
    }
}

// Translate a frozen format-v1 Control Surfaces blob (8 x 16-byte bindings)
// into the current v2 layout (16 x 24-byte bindings).  Read only by the V7→V9
// and V8→V9 directory migrations.  Each old binding's shared fields carry
// forward verbatim; the v2-only fields (event / target / index / reserved)
// start zero, so a migrated button defaults to CS_EVT_PRESS and an untargeted
// noun to target/index 0.  The old two reserved bytes are dropped, and slots
// 8..15 start empty (CS_TYPE_NONE).  Full validation still happens at boot in
// control_surfaces_apply_binding.
static void cs_config_from_v1(CsFlashConfig *dst, const CsFlashConfig_v1 *src) {
    memset(dst, 0, sizeof(*dst));
    dst->version = CS_CONFIG_VERSION;
    for (int s = 0; s < 8; s++) {
        const CsBinding_v1 *o = &src->bindings[s];
        CsBinding *n = &dst->bindings[s];
        n->type      = o->type;
        n->noun      = o->noun;
        n->action    = o->action;
        n->flags     = o->flags;
        n->gpio[0]   = o->gpio[0];
        n->gpio[1]   = o->gpio[1];
        n->value     = o->value;
        n->step      = o->step;
        n->range_min = o->range_min;
        n->range_max = o->range_max;
        // event / target / index / reserved / reserved2 stay zero.
    }
    // slots 8..15 stay zero (CS_TYPE_NONE) from the memset.
}

// Widen a frozen format-v1 IR command table (8 sub-slots) to the current v2
// layout (16).  Read by every pre-V17 directory migration.  IrCommand is
// unchanged, so the learned commands copy verbatim; sub-slots 8..15 stay empty
// (CS_IR_PROTO_NONE) from the memset.
static void cs_ir_from_v1(CsIrConfig *dst, const CsIrConfig_v1 *src) {
    memset(dst, 0, sizeof(*dst));
    dst->version = CS_IR_CONFIG_VERSION;
    for (int s = 0; s < 8; s++) dst->cmds[s] = src->cmds[s];
}

// Bound-check the directory's Control Surfaces config.  An implausible blob
// version resets the whole block; an implausible binding resets that slot.
// Deeper checks (action masks, pin collisions) run at apply time in
// control_surfaces_apply_binding.
static void dir_sanitize_cs_config(void) {
    CsFlashConfig *c = &dir_cache.cs_config;
    if (c->version > CS_CONFIG_VERSION) {
        memset(c, 0, sizeof(*c));
        c->version = CS_CONFIG_VERSION;
        return;
    }
    for (int s = 0; s < CS_MAX_BINDINGS; s++) {
        CsBinding *b = &c->bindings[s];
        // Shallow enum-range check, same philosophy as the v1 layout: reset a
        // slot whose persisted type/noun/action/event is out of its enum range.
        // event joins the set as a v2 addition; target/index are channel/band
        // indices with no static bound here and are validated at apply time.
        if (b->type >= CS_TYPE_COUNT ||
            (b->type != CS_TYPE_NONE &&
             (b->noun >= CS_NOUN_COUNT || b->action >= CS_ACT_COUNT ||
              b->event >= CS_EVT_COUNT))) {
            memset(b, 0, sizeof(*b));
        }
    }
    // Normalize the version byte; pre-V7 migration paths leave it 0 (same
    // layout, all slots NONE).
    c->version = CS_CONFIG_VERSION;
    // Names (V10): guarantee NUL termination so hand-edited flash can never
    // leak an unterminated string to REQ_GET_CS_NAME readers.
    for (int s = 0; s < CS_MAX_BINDINGS; s++) {
        dir_cache.cs_names[s][CS_NAME_LEN - 1] = '\0';
    }
}

// Bound-check the directory's Control Surfaces IR command table.  An implausible
// blob version (or a dirty reserved field) resets the whole block; an
// implausible command resets that sub-slot.  Deeper checks run at apply time in
// control_surfaces_apply_ir_cmd.  Mirrors dir_sanitize_cs_config.
static void dir_sanitize_cs_ir(void) {
    CsIrConfig *c = &dir_cache.cs_ir;
    if (c->version > CS_IR_CONFIG_VERSION ||
        c->reserved[0] || c->reserved[1] || c->reserved[2]) {
        memset(c, 0, sizeof(*c));
        c->version = CS_IR_CONFIG_VERSION;
        return;
    }
    for (int s = 0; s < CS_MAX_IR_COMMANDS; s++) {
        IrCommand *cmd = &c->cmds[s];
        // Shallow enum-range check: reset a sub-slot whose persisted protocol is
        // out of range, or whose noun/action is out of range while occupied.
        // protocol == CS_IR_PROTO_NONE marks the sub-slot empty.
        if (cmd->protocol >= CS_IR_PROTO_COUNT ||
            (cmd->protocol != CS_IR_PROTO_NONE &&
             (cmd->noun >= CS_NOUN_COUNT || cmd->action >= CS_ACT_COUNT))) {
            memset(cmd, 0, sizeof(*cmd));
        }
    }
    // Normalize the version byte; a fresh/migrated all-zero block leaves it 0
    // (still idle, every sub-slot empty).
    c->version = CS_IR_CONFIG_VERSION;
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
    dir_cache.output_config_mode = OUTPUT_CONFIG_MODE_WITH_PRESET;  // IO travels with presets by default
    dir_cache.master_volume_mode = MASTER_VOLUME_MODE_INDEPENDENT;
    dir_cache.master_volume_db   = MASTER_VOL_DEFAULT_DB;
    dir_cache.spdif_rx_pin = PICO_SPDIF_RX_PIN_DEFAULT;
    dir_cache.slot_occupied = 0;             // All slots empty
    // Slot 0 gets a default name; others are empty (already zeroed by memset)
    strncpy(dir_cache.slot_names[0], "Default", PRESET_NAME_LEN - 1);
    // dac_hw_mute: feature disabled, but pin/polarity/timing pre-set
    // to PCM5102A-friendly defaults so users with the common DAC only
    // have to flip enabled=1.
    dir_apply_dac_hw_mute_defaults();
    io_config_defaults(&dir_cache.output_config);  // V4 device-global IO defaults
    ctrl_iface_defaults(&dir_cache.uart_ctrl, &dir_cache.i2c_ctrl);  // V6 control interfaces
    dir_cache_valid = true;
    // Don't flush yet — will be flushed on first preset save
}

// ============================================================================
// PHYSICAL IO / OUTPUT CONFIG
// ============================================================================
//
// The device's physical IO/routing — output pins, output types (SPDIF/I2S),
// I2S MCK/BCK, and the SPDIF RX pin — is owned by the output_config_mode
// mechanism, the exact analog of the master-volume independent/with-preset
// mechanism.  Whether it travels with presets (WITH_PRESET, default) or is a
// device-global value stored in the directory (INDEPENDENT) is selected by
// dir_cache.output_config_mode.  A single snapshot/apply path is shared by the
// per-slot and device-global sources so the two cannot diverge.  Input source
// (USB vs SPDIF) is NOT part of this block — it stays per-preset.

// True if `pin` is a usable output/RX GPIO on this platform.  (0 is allowed for
// output pins; the SPDIF RX path additionally rejects 0 as "absent".)
static bool io_pin_valid(uint8_t pin) {
    // GPIO 16/17 are assignable again (debug UART removed).  Live UART/I2C
    // control pins are rejected here so a stored or pushed IO config can
    // never steal the control link applying it (self-lockout guard); at
    // boot the interfaces are not yet up, so stored audio pins win and a
    // colliding control config stays down (visible via 0xF9).
    if (uart_ctrl_owns_pin(pin) || i2c_ctrl_owns_pin(pin)) return false;
#if PICO_RP2350
    uint8_t max_pin = (sysinfo_hw->package_sel == 0) ? 47 : 29;
    bool valid = (pin <= max_pin) && !(pin >= 23 && pin <= 25);
#else
    bool valid = (pin <= 28) && !(pin >= 23 && pin <= 25);
#endif
    return valid;
}

// Firmware IO factory defaults: all-SPDIF outputs, default pins, MCK off,
// default SPDIF RX pin.  Matches apply_factory_defaults' former IO block.
static void io_config_defaults(FlashOutputConfig *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    cfg->output_pins[0] = PICO_AUDIO_SPDIF_PIN;
    cfg->output_pins[1] = PICO_SPDIF_PIN_2;
#if PICO_RP2350
    cfg->output_pins[2] = PICO_SPDIF_PIN_3;
    cfg->output_pins[3] = PICO_SPDIF_PIN_4;
    cfg->output_pins[4] = PICO_PDM_PIN;
#else
    cfg->output_pins[2] = PICO_PDM_PIN;
#endif
    // output_types[] all 0 (S/PDIF) from memset
    cfg->i2s_bck_pin        = PICO_I2S_BCK_PIN;
    cfg->i2s_mck_pin        = PICO_I2S_MCK_PIN;
    cfg->i2s_mck_enabled    = 0;
    cfg->i2s_mck_multiplier = 0;             // 0 = 128x
    cfg->spdif_rx_pin       = PICO_SPDIF_RX_PIN_DEFAULT;
    // Optional SPDIF inputs 2..4: disabled, default pins.
    cfg->spdif_rx_enabled_ext = 0;
    for (uint8_t i = 1; i < SPDIF_RX_NUM_INPUTS; i++)
        *cfg_spdif_ext_pin(cfg, i - 1) = spdif_rx_pin_default_for_index(i);
    cfg->i2s_rx_pin         = PICO_I2S_RX_PIN_DEFAULT;
    cfg->i2s_input_rate_p1  = (uint8_t)(i2s_rate_encode(48000) + 1);
    cfg->i2s_input_channels = 2;   // stereo; i2s_rx_pin_ext[] left 0 (unset) by the memset
    // ADAT bulk output: disabled; default pin (0 on RP2040 where absent).
    cfg->adat_enabled = 0;
#if PICO_RP2350
    cfg->adat_pin = PICO_ADAT_PIN;
#else
    cfg->adat_pin = 0;
#endif
    cfg->i2s_clock_mode = I2S_CLOCK_MODE_MASTER;   // master (0) is the default
    cfg->i2s_clock_pin_mode = I2S_CLOCK_PIN_MODE_UNIFIED;  // unified (0) is the default
    cfg->i2s_bck_pin_slave  = PICO_I2S_BCK_PIN_SLAVE;
    // ADAT input: disabled, pin unset (0xFF), master clock.
    cfg->adat_input_pin        = 0xFF;
    cfg->adat_input_enabled    = 0;
    cfg->adat_input_clock_mode = ADAT_CLOCK_MODE_MASTER;
}

// Snapshot the live IO globals into cfg (for REQ_SAVE_OUTPUT_CONFIG).
static void io_config_from_live(FlashOutputConfig *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    for (int i = 0; i < NUM_PIN_OUTPUTS; i++)     cfg->output_pins[i]  = output_pins[i];
    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) cfg->output_types[i] = output_types[i];
    cfg->i2s_bck_pin        = i2s_bck_pin;
    cfg->i2s_mck_pin        = i2s_mck_pin;
    cfg->i2s_mck_enabled    = i2s_mck_enabled ? 1 : 0;
    cfg->i2s_mck_multiplier = (i2s_mck_multiplier == 256) ? 1 : 0;  // 0=128x, 1=256x
    cfg->spdif_rx_pin       = spdif_rx_pin;
    cfg->spdif_rx_enabled_ext = spdif_rx_enabled_ext;
    for (uint8_t i = 0; i < SPDIF_RX_NUM_INPUTS - 1; i++)
        *cfg_spdif_ext_pin(cfg, i) = spdif_rx_pin_ext[i];
    cfg->i2s_rx_pin         = i2s_rx_pin[0];
    cfg->i2s_input_rate_p1  = (uint8_t)(i2s_rate_encode(i2s_input_rate) + 1);
    cfg->i2s_input_channels = i2s_input_channels;
    for (int p = 0; p < 3; p++)
        cfg->i2s_rx_pin_ext[p] = (p + 1 < I2S_RX_MAX_PAIRS) ? i2s_rx_pin[p + 1] : 0;
    // ADAT bulk output (RP2350 only; zeros on RP2040 where the feature is absent).
#if PICO_RP2350
    cfg->adat_enabled = adat_output_config_enabled() ? 1 : 0;
    cfg->adat_pin     = adat_output_pin();
#else
    cfg->adat_enabled = 0;
    cfg->adat_pin     = 0;
#endif
    // I2S clock master/slave mode (both platforms).
    cfg->i2s_clock_mode = i2s_clock_mode;
    // I2S clock-pin mode + slave-pair BCK (both platforms).
    cfg->i2s_clock_pin_mode = i2s_clock_pin_mode;
    cfg->i2s_bck_pin_slave  = i2s_bck_pin_slave;
    // ADAT input (both platforms; RP2040 keeps state for round-trips).
    cfg->adat_input_pin        = adat_input_pin;
    cfg->adat_input_enabled    = adat_input_enabled ? 1 : 0;
    cfg->adat_input_clock_mode = adat_clock_mode;
}

// Extract a slot's IO config into cfg, honoring the slot's data version
// (output_pins V6+, output_types/I2S V9+ with the V11+ multiplier encoding,
// spdif_rx_pin V13+).  Fields the slot predates fall back to defaults; the
// SPDIF RX pin baselines from the device-level value (so a device-level pin set
// on a pre-V13 preset still survives) and a valid per-preset pin overrides it.
static void io_config_from_slot(const PresetSlot *slot, FlashOutputConfig *cfg) {
    io_config_defaults(cfg);
    cfg->spdif_rx_pin = dir_cache.output_config.spdif_rx_pin;  // device-level baseline
    if (slot->version >= 6) {
        for (int i = 0; i < NUM_PIN_OUTPUTS; i++) cfg->output_pins[i] = slot->output_pins[i];
    }
    if (slot->version >= 9) {
        for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) cfg->output_types[i] = slot->output_types[i];
        cfg->i2s_bck_pin     = slot->i2s_bck_pin;
        cfg->i2s_mck_pin     = slot->i2s_mck_pin;
        cfg->i2s_mck_enabled = slot->i2s_mck_enabled ? 1 : 0;
        // V11+ stores 0=128x/1=256x; V9-V10 stored raw (128, or 0 meaning 256).
        if (slot->version >= 11)
            cfg->i2s_mck_multiplier = (slot->i2s_mck_multiplier == 1) ? 1 : 0;
        else
            cfg->i2s_mck_multiplier = (slot->i2s_mck_multiplier == 0) ? 1 : 0;
    }
    if (slot->version >= 13 && slot->spdif_rx_pin != 0)
        cfg->spdif_rx_pin = slot->spdif_rx_pin;

    // Optional SPDIF inputs 2..4 (V24+; input 4's pin V35+): device-level
    // baseline, slot overrides.  Pins use the 0 = unset convention; the enable
    // mask has no unset sentinel in the slot, so it only overrides when the
    // slot actually carries it.
    cfg->spdif_rx_enabled_ext = dir_cache.output_config.spdif_rx_enabled_ext;
    for (uint8_t i = 0; i < SPDIF_RX_NUM_INPUTS - 1; i++)
        *cfg_spdif_ext_pin(cfg, i) =
            cfg_spdif_ext_pin_get(&dir_cache.output_config, i);
    if (slot->version >= 24) {
        cfg->spdif_rx_enabled_ext = slot->spdif_rx_enabled_ext;
        for (int i = 0; i < 2; i++)
            if (slot->spdif_rx_pin_ext[i] != 0)
                cfg->spdif_rx_pin_ext[i] = slot->spdif_rx_pin_ext[i];
    }
    if (slot->version >= 35 && slot->spdif_rx_pin4 != 0)
        cfg->spdif_rx_pin4 = slot->spdif_rx_pin4;

    // I2S input pin + rate (V17+): device-level baseline, slot overrides.
    cfg->i2s_rx_pin        = dir_cache.output_config.i2s_rx_pin;
    cfg->i2s_input_rate_p1 = dir_cache.output_config.i2s_input_rate_p1;
    if (slot->version >= 17) {
        if (slot->i2s_rx_pin != 0)
            cfg->i2s_rx_pin = slot->i2s_rx_pin;
        cfg->i2s_input_rate_p1 = (uint8_t)(slot->i2s_input_rate + 1);
    }

    // I2S multichannel input (V22+): device-level baseline, slot overrides.
    // 0 = unset (keep the baseline), matching the per-pin convention above.
    cfg->i2s_input_channels = dir_cache.output_config.i2s_input_channels;
    memcpy(cfg->i2s_rx_pin_ext, dir_cache.output_config.i2s_rx_pin_ext,
           sizeof(cfg->i2s_rx_pin_ext));
    if (slot->version >= 22) {
        if (slot->i2s_input_channels != 0)
            cfg->i2s_input_channels = slot->i2s_input_channels;
        for (int p = 0; p < 3; p++)
            if (slot->i2s_rx_pin_ext[p] != 0)
                cfg->i2s_rx_pin_ext[p] = slot->i2s_rx_pin_ext[p];
    }

    // ADAT bulk output (V23+): device-level baseline, slot overrides.
    // adat_pin == 0 = unset (keep the baseline; io_config_apply falls back to
    // PICO_ADAT_PIN), matching the per-pin convention above.
    cfg->adat_enabled = dir_cache.output_config.adat_enabled;
    cfg->adat_pin     = dir_cache.output_config.adat_pin;
    if (slot->version >= 23) {
        cfg->adat_enabled = slot->adat_enabled ? 1 : 0;
        if (slot->adat_pin != 0)
            cfg->adat_pin = slot->adat_pin;
    }

    // I2S clock master/slave mode (V28+): device-global baseline, slot overrides.
    // Live-vs-dormant split:
    //   - dormant (input != I2S): set i2s_clock_mode now so the next I2S start
    //     honors it; the main loop is not asked to rebuild anything.
    //   - live (input == I2S): pending ONLY.  Pre-setting the global here
    //     would make the load's own input restart adopt the new role while
    //     the output slots still hold the old clocking (transient dual-master
    //     on BCK/LRCLK); the main-loop handler flips the global and rebuilds
    //     input + output clocking atomically instead.
    {
        uint8_t m = (dir_cache.output_config.i2s_clock_mode <= 1)
                        ? dir_cache.output_config.i2s_clock_mode
                        : I2S_CLOCK_MODE_MASTER;
        if (slot->version >= 28 && slot->i2s_clock_mode <= 1)
            m = slot->i2s_clock_mode;
        cfg->i2s_clock_mode = m;
        if (m != i2s_clock_mode) {
            if (active_input_source == INPUT_SOURCE_I2S) {
                pending_i2s_clock_mode = m;
                __dmb();
                i2s_clock_mode_change_pending = true;
            } else {
                i2s_clock_mode = m;   // dormant/boot apply
            }
        }
    }

    // I2S clock-pin mode + slave-pair BCK (V29+): device-global baseline, slot
    // overrides.  Just resolve into cfg here; io_config_apply installs the live
    // globals (the pin mode has no pending mechanism, unlike the clock mode).
    {
        uint8_t pm = (dir_cache.output_config.i2s_clock_pin_mode <= 1)
                         ? dir_cache.output_config.i2s_clock_pin_mode
                         : I2S_CLOCK_PIN_MODE_UNIFIED;
        uint8_t sp = dir_cache.output_config.i2s_bck_pin_slave;  // 0 = unset → default
        if (slot->version >= 29) {
            if (slot->i2s_clock_pin_mode <= 1) pm = slot->i2s_clock_pin_mode;
            if (slot->i2s_bck_pin_slave != 0)  sp = slot->i2s_bck_pin_slave;
        }
        cfg->i2s_clock_pin_mode = pm;
        cfg->i2s_bck_pin_slave  = sp;
    }

    // ADAT input (V32+): device-global baseline, slot overrides.  adat_input_pin
    // 0xFF = unset (keep baseline); enable + clock mode override wholesale.  The
    // version gate keeps a pre-V32 slot's zero-filled tail from misreading as
    // GPIO 0 / disabled / master, defaulting to the device baseline instead.
    cfg->adat_input_pin        = dir_cache.output_config.adat_input_pin;
    cfg->adat_input_enabled    = dir_cache.output_config.adat_input_enabled;
    cfg->adat_input_clock_mode = dir_cache.output_config.adat_input_clock_mode;
    if (slot->version >= 32) {
        cfg->adat_input_enabled = slot->adat_input_enabled ? 1 : 0;
        if (slot->adat_input_clock_mode <= 1)
            cfg->adat_input_clock_mode = slot->adat_input_clock_mode;
        if (slot->adat_input_pin != 0xFF)
            cfg->adat_input_pin = slot->adat_input_pin;
    }
}

// Apply a FlashOutputConfig to the live IO globals.  Validates pins (invalid →
// platform default), checks MCK GPOUT-capability (falls back + disables MCK if
// the pin can't drive clk_gpoutN here), and — if the SPDIF RX pin changed while
// RX is the active input — schedules the running RX library's GPIO hot-swap.
// Sets output_types[]/i2s_* that the boot setup and the main loop's pipeline
// reset consume; it does NOT itself reconfigure hardware.  Input source is
// deliberately untouched.
static void io_config_apply(const FlashOutputConfig *cfg) {
    // Snapshot the BCK pair the running input actually uses before any clock /
    // clock-pin change below; used at the end to restart the input if it moved.
    uint8_t old_eff_bck = i2s_effective_bck_pin();

    static const uint8_t default_pins[NUM_PIN_OUTPUTS] = {
#if PICO_RP2350
        PICO_AUDIO_SPDIF_PIN, PICO_SPDIF_PIN_2, PICO_SPDIF_PIN_3, PICO_SPDIF_PIN_4, PICO_PDM_PIN
#else
        PICO_AUDIO_SPDIF_PIN, PICO_SPDIF_PIN_2, PICO_PDM_PIN
#endif
    };
    for (int i = 0; i < NUM_PIN_OUTPUTS; i++) {
        uint8_t pin = cfg->output_pins[i];
        output_pins[i] = io_pin_valid(pin) ? pin : default_pins[i];
    }

    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) output_types[i] = cfg->output_types[i];
    // Validate the stored/restored BCK before installing it raw (see
    // i2s_bck_pin_acceptable): BCK/LRCLK are clock OUTPUTS, so an invalid GPIO
    // can fault pio_gpio_init() and a collision with an output pin is driver
    // contention.  Output pins are applied above, so the conflict check sees the
    // final config; keep the live (known-valid) pin on failure.  The RX set
    // below is validated against whatever BCK ends up installed.
    if (i2s_bck_pin_acceptable(cfg->i2s_bck_pin)) {
        i2s_bck_pin = cfg->i2s_bck_pin;
    } else {
        printf("io_config: I2S BCK pin %u rejected (invalid/conflict); kept %u\n",
               (unsigned)cfg->i2s_bck_pin, (unsigned)i2s_bck_pin);
    }

    // I2S clock-pin mode + slave-pair BCK.  Installed here (after the master BCK
    // above, before the RX validation below) because that RX block reads the live
    // i2s_clock_pin_mode / i2s_bck_pin_slave to keep RX pins off the slave pair.
    // The pin mode has no pending mechanism (unlike the clock mode): apply it live.
    if (cfg->i2s_clock_pin_mode <= 1) i2s_clock_pin_mode = cfg->i2s_clock_pin_mode;
    {
        // 0 = unset → keep the live slave pin.  Reject an invalid GPIO or one that
        // overlaps the just-installed master BCK/LRCLK pair (both are clock lines);
        // keep the live pin on rejection, mirroring the master-BCK message style.
        uint8_t p = cfg->i2s_bck_pin_slave ? cfg->i2s_bck_pin_slave : i2s_bck_pin_slave;
        bool overlap = (p == i2s_bck_pin) || (p == (uint8_t)(i2s_bck_pin + 1)) ||
                       ((uint8_t)(p + 1) == i2s_bck_pin);
        if (i2s_bck_pin_acceptable(p) && !overlap) {
            i2s_bck_pin_slave = p;
        } else {
            printf("io_config: I2S slave BCK pin %u rejected (invalid/conflict); kept %u\n",
                   (unsigned)p, (unsigned)i2s_bck_pin_slave);
        }
    }

    // MCK pin must map to clk_gpoutN on this platform (see the V9 apply note);
    // otherwise reset to default and force MCK off so the change is visible.
    if (GPIO_TO_GPOUT_CLOCK_HANDLE(cfg->i2s_mck_pin, clk_sys) == clk_sys) {
        printf("Output-config MCK pin %u not GPOUT-capable on this platform; "
               "resetting to default %u and disabling MCK\n",
               (unsigned)cfg->i2s_mck_pin, (unsigned)PICO_I2S_MCK_PIN);
        i2s_mck_pin = PICO_I2S_MCK_PIN;
        i2s_mck_enabled = false;
    } else {
        i2s_mck_pin = cfg->i2s_mck_pin;
        i2s_mck_enabled = (cfg->i2s_mck_enabled != 0);
    }
    i2s_mck_multiplier = (cfg->i2s_mck_multiplier == 1) ? 256 : 128;

    // SPDIF RX pin (reject 0 = "absent"; hot-swap if it changed while RX active).
    if (cfg->spdif_rx_pin != 0 && io_pin_valid(cfg->spdif_rx_pin) &&
        cfg->spdif_rx_pin != spdif_rx_pin) {
        spdif_rx_pin = cfg->spdif_rx_pin;
        if (input_source_is_spdif(active_input_source) &&
            spdif_index_for_source(active_input_source) == 0)
            spdif_rx_pin_change_pending = true;
    }

    // Optional SPDIF inputs 2..4: pins first (0 = unset, platform default),
    // then the enable mask, so a stored pin+enable pair is validated against
    // the pin it arrives with.  A disabled input's pin is only a stored
    // preference; enabling is what must pass the conflict check.
    {
        for (uint8_t i = 0; i < SPDIF_RX_NUM_INPUTS - 1; i++) {
            uint8_t dflt = spdif_rx_pin_default_for_index((uint8_t)(i + 1));
            uint8_t stored = cfg_spdif_ext_pin_get(cfg, i);
            uint8_t pin = stored ? stored : dflt;
            if (!io_pin_valid(pin)) pin = dflt;
            if (pin != spdif_rx_pin_ext[i]) {
                spdif_rx_pin_ext[i] = pin;
                if (input_source_is_spdif(active_input_source) &&
                    spdif_index_for_source(active_input_source) == (uint8_t)(i + 1))
                    spdif_rx_pin_change_pending = true;
            }
        }
        // Two passes (disables, then enables) so a stored config that moves an
        // enable from one input to another's old pin still validates cleanly.
        uint8_t want = cfg->spdif_rx_enabled_ext & SPDIF_RX_ENABLED_EXT_MASK;
        for (uint8_t i = 1; i < SPDIF_RX_NUM_INPUTS; i++) {
            uint8_t bit = (uint8_t)(1u << (i - 1));
            if ((want & bit) || !spdif_input_enabled(i)) continue;
            if (input_source_is_spdif(active_input_source) &&
                spdif_index_for_source(active_input_source) == i) {
                printf("io_config: SPDIF input %u kept enabled (active source)\n",
                       (unsigned)(i + 1));
                continue;   // never disable the live input
            }
            spdif_rx_enabled_ext &= (uint8_t)~bit;
        }
        for (uint8_t i = 1; i < SPDIF_RX_NUM_INPUTS; i++) {
            uint8_t bit = (uint8_t)(1u << (i - 1));
            if (!(want & bit) || spdif_input_enabled(i)) continue;
            if (spdif_input_enable_acceptable(i)) {
                spdif_rx_enabled_ext |= bit;
            } else {
                printf("io_config: SPDIF input %u enable rejected (pin %u in use)\n",
                       (unsigned)(i + 1), (unsigned)spdif_rx_pin_for_index(i));
            }
        }
    }

    // I2S RX data pins (pair 0 + multichannel extras) and channel count, applied
    // as a validated SET (i2s_rx_pin_set_acceptable) so a stored config can't
    // bring two state machines up on one GPIO or on a clock pin.  0 = keep-live
    // per field; an invalid count keeps the live count.  BCK is applied above,
    // so the live i2s_bck_pin is the value the proposed pins must avoid.
    // Rejected as a unit (live retained) if inconsistent; a change restarts the
    // input so every pair re-syncs.
    {
        uint8_t proposed[I2S_RX_MAX_PAIRS];
        proposed[0] = (cfg->i2s_rx_pin != 0) ? cfg->i2s_rx_pin : i2s_rx_pin[0];
#if I2S_RX_MAX_PAIRS > 1
        for (int p = 1; p < I2S_RX_MAX_PAIRS; p++) {
            uint8_t pin = cfg->i2s_rx_pin_ext[p - 1];
            proposed[p] = (pin != 0) ? pin : i2s_rx_pin[p];
        }
#endif
        uint8_t ch = cfg->i2s_input_channels;
        uint8_t count = (ch == 2 || ch == 4 || ch == 6 || ch == 8)
                            ? ch : i2s_input_channels;
        if (count / 2 > I2S_RX_MAX_PAIRS) count = i2s_input_channels;

        if (i2s_rx_pin_set_acceptable(proposed, count / 2, i2s_bck_pin,
                                      (i2s_clock_pin_mode == I2S_CLOCK_PIN_MODE_SPLIT)
                                          ? i2s_bck_pin_slave : 0xFF)) {
            bool changed = false;
            for (int p = 0; p < I2S_RX_MAX_PAIRS; p++)
                if (proposed[p] != i2s_rx_pin[p]) {
                    i2s_rx_pin[p] = proposed[p];
                    changed = true;
                }
            if (count != i2s_input_channels) {
                i2s_input_channels = count;
                changed = true;
            }
            if (changed && active_input_source == INPUT_SOURCE_I2S)
                i2s_input_restart_pending = true;
        } else {
            printf("io_config: I2S RX pin/count config rejected (conflict); kept live\n");
        }
    }

    // I2S input rate (+1 sentinel: 0 = unset, leave live value alone).
    // No rate-change trigger here; the preset-load / bulk-apply handlers
    // in main.c compare i2s_input_rate against audio_state.freq after the
    // input restarts and defer the change themselves.
    if (cfg->i2s_input_rate_p1 != 0) {
        i2s_input_rate = i2s_rate_decode((uint8_t)(cfg->i2s_input_rate_p1 - 1));
    }

    // I2S clock master/slave mode (both platforms).  Live-vs-dormant split,
    // same as io_config_from_slot(): dormant applies the global directly;
    // live defers entirely to the main-loop handler, which flips the global
    // and rebuilds input + output clocking atomically (pre-setting it here
    // would let intermediate restarts adopt the new role against outputs
    // still holding the old clocking).  In WITH_PRESET the from_slot pass
    // already resolved the same value, so re-arming here is idempotent.
    if (cfg->i2s_clock_mode <= 1 && cfg->i2s_clock_mode != i2s_clock_mode) {
        if (active_input_source == INPUT_SOURCE_I2S) {
            pending_i2s_clock_mode = cfg->i2s_clock_mode;
            __dmb();
            i2s_clock_mode_change_pending = true;
        } else {
            i2s_clock_mode = cfg->i2s_clock_mode;   // dormant/boot apply
        }
    }

    // If the effective BCK pair moved (clock-pin mode / slave pin change) while
    // I2S is the live input, restart it so RX re-latches onto the new pair.
    // Redundant arming is cleared by the bracketed restore paths in main.c; this
    // covers paths without a bracket.  (If the clock-mode block above just
    // deferred a flip, its handler rebuilds everything and consumes this flag.)
    if (i2s_effective_bck_pin() != old_eff_bck && active_input_source == INPUT_SOURCE_I2S)
        i2s_input_restart_pending = true;

#if PICO_RP2350
    // ADAT bulk output: 0 = unset; a stored pin that fails the full ownership
    // check (adat_pin_acceptable, against the outputs applied above) falls back
    // to the default.  set_config only marks adat_output_config_dirty; the PIO
    // reconfigure is deferred to the main loop's pipeline-reset bracket, never
    // done here.  RP2040 ignores both fields (feature absent).
    {
        uint8_t pin = cfg->adat_pin ? cfg->adat_pin : PICO_ADAT_PIN;
        bool pin_ok = adat_pin_acceptable(pin);
        if (!pin_ok && pin != PICO_ADAT_PIN) {
            pin = PICO_ADAT_PIN;               // stored pin conflicts; try default
            pin_ok = adat_pin_acceptable(pin);
        }
        adat_output_set_config(cfg->adat_enabled != 0 && pin_ok, pin);
    }
#endif

#if PICO_RP2350
    // ADAT input pin + enable (RP2350).  adat_input_pin 0xFF = unset.  A stored
    // pin that differs from the live one and fails the ownership check falls back
    // to unset + disabled (an enabled input with no valid pin is inconsistent and
    // unselectable).  A disable of the live source is refused (mirrors the
    // SPDIF-ext precedent); a pin/enable change while ADAT is active arms a
    // deferred input restart.
    {
        uint8_t pin = cfg->adat_input_pin;
        uint8_t en  = cfg->adat_input_enabled ? 1 : 0;
        if (pin != 0xFF && pin != adat_input_pin && !adat_input_pin_acceptable(pin)) {
            printf("io_config: ADAT input pin %u rejected (invalid/conflict); unset\n",
                   (unsigned)pin);
            pin = 0xFF;
        }
        if (pin == 0xFF) en = 0;   // no valid pin -> cannot be enabled
        if (en == 0 && adat_input_enabled &&
            (active_input_source == INPUT_SOURCE_ADAT ||
             (input_source_change_pending &&
              pending_input_source == INPUT_SOURCE_ADAT))) {
            printf("io_config: ADAT input kept enabled (active source)\n");
            pin = adat_input_pin;
            en  = 1;
        }
        bool changed = (pin != adat_input_pin) || (en != adat_input_enabled);
        adat_input_pin     = pin;
        adat_input_enabled = en;
        if (changed && active_input_source == INPUT_SOURCE_ADAT)
            adat_input_restart_pending = true;
    }
#endif

    // ADAT input clock master/slave mode (both platforms; round-trips on RP2040).
    // Live-vs-dormant split like i2s_clock_mode: dormant writes the global
    // directly; live defers to the main-loop handler.
    if (cfg->adat_input_clock_mode <= 1 &&
        cfg->adat_input_clock_mode != adat_clock_mode) {
        if (active_input_source == INPUT_SOURCE_ADAT) {
            pending_adat_clock_mode = cfg->adat_input_clock_mode;
            __dmb();
            adat_clock_mode_change_pending = true;
        } else {
            adat_clock_mode = cfg->adat_input_clock_mode;   // dormant/boot apply
        }
    }
}

// Re-derive the live physical IO config for a preset *context* change — the
// exact analog of apply_master_volume_from_mode().  IO is sourced ONLY here;
// apply_slot_to_live() / apply_factory_defaults() no longer touch it.
//
//   WITH_PRESET: IO travels with the preset.  A configured slot supplies it; an
//     empty/factory-default context (slot_or_null == NULL) gets firmware
//     defaults — exactly as EQ resets to flat.
//   INDEPENDENT: IO is device-global.  Only a BOOT restore re-applies the saved
//     directory block; runtime context changes leave the live IO untouched, so
//     loading a preset never re-wires outputs.
//
// `slot_or_null` is the loaded slot, or NULL for an empty/factory context.
// `is_boot` is true only on the power-on restore path.
static void apply_output_config_from_mode(const PresetSlot *slot_or_null, bool is_boot) {
    FlashOutputConfig cfg;
    if (dir_cache.output_config_mode == OUTPUT_CONFIG_MODE_WITH_PRESET) {
        if (slot_or_null) io_config_from_slot(slot_or_null, &cfg);
        else              io_config_defaults(&cfg);
        io_config_apply(&cfg);
    } else if (is_boot) {
        io_config_apply(&dir_cache.output_config);
    }
    // INDEPENDENT + runtime: intentionally a no-op (live IO survives).
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
    slot->loudness_output_mask = loudness_output_mask;
    slot->loudness_ref_spl = loudness_ref_spl;
    slot->loudness_intensity_pct = loudness_intensity_pct;

    // Crossfeed
    slot->crossfeed_enabled = crossfeed_config.enabled ? 1 : 0;
    slot->crossfeed_preset = crossfeed_config.preset;
    slot->crossfeed_itd_enabled = crossfeed_config.itd_enabled ? 1 : 0;
    slot->crossfeed_output_pair_mask = crossfeed_config.output_pair_mask;
    slot->crossfeed_custom_fc = crossfeed_config.custom_fc;
    slot->crossfeed_custom_feed_db = crossfeed_config.custom_feed_db;

    // Matrix mixer — all inputs, direct.
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

    // Pin configuration (always stored; loaded per output_config_mode —
    // WITH_PRESET applies it, INDEPENDENT ignores it in favor of the directory).
    memcpy(slot->output_pins, output_pins, sizeof(slot->output_pins));

    // SPDIF RX pin: stored alongside output_pins, applied per output_config_mode.
    slot->spdif_rx_pin = spdif_rx_pin;

    // Optional SPDIF inputs 2..4: enable mask + pins (V24; input 4's pin V35),
    // same apply model.
    slot->spdif_rx_enabled_ext = spdif_rx_enabled_ext;
    slot->spdif_rx_pin_ext[0]  = spdif_rx_pin_ext[0];
    slot->spdif_rx_pin_ext[1]  = spdif_rx_pin_ext[1];
    slot->spdif_rx_pin4        = spdif_rx_pin_ext[2];

    // I2S input pin + rate (V17): same storage/apply model as spdif_rx_pin.
    slot->i2s_rx_pin = i2s_rx_pin[0];
    slot->i2s_input_rate = i2s_rate_encode(i2s_input_rate);

    // I2S multichannel input (V22): channel count + extra data pins.
    slot->i2s_input_channels = i2s_input_channels;
    for (int p = 0; p < 3; p++)
        slot->i2s_rx_pin_ext[p] = (p + 1 < I2S_RX_MAX_PAIRS) ? i2s_rx_pin[p + 1] : 0;

    // ADAT bulk output (V23): configured enable + data pin (RP2350 only;
    // zeros on RP2040 where the feature is absent).
#if PICO_RP2350
    slot->adat_enabled = adat_output_config_enabled() ? 1 : 0;
    slot->adat_pin     = adat_output_pin();
#else
    slot->adat_enabled = 0;
    slot->adat_pin     = 0;
#endif

    // I2S clock master/slave mode (V28): plain 0/1, same storage/apply model.
    slot->i2s_clock_mode = i2s_clock_mode;

    // I2S clock-pin mode + slave-pair BCK (V29): plain 0/1 + 0-unset pin.
    slot->i2s_clock_pin_mode = i2s_clock_pin_mode;
    slot->i2s_bck_pin_slave  = i2s_bck_pin_slave;

    // Linkwitz Transform per-band target Q (V30): Q*512, 0 = 0.707 default.
    memcpy(slot->peq_qp_x512, peq_qp_x512, sizeof(slot->peq_qp_x512));

    // Psychoacoustic bass (V31): one global config, enabled + mask + 5 floats.
    slot->psybass_enabled       = psybass_config.enabled ? 1 : 0;
    slot->psybass_reserved      = 0;
    slot->psybass_output_mask   = psybass_config.output_mask;
    slot->psybass_cutoff_hz     = psybass_config.cutoff_hz;
    slot->psybass_harmonics_db  = psybass_config.harmonics_db;
    slot->psybass_drive_db      = psybass_config.drive_db;
    slot->psybass_character_pct = psybass_config.character_pct;
    slot->psybass_original_db   = psybass_config.original_db;

    // ADAT input (V32): raw pin (0xFF unset) + enable + clock mode (both
    // platforms; RP2040 stores its default state for round-trips).
    slot->adat_input_pin        = adat_input_pin;
    slot->adat_input_enabled    = adat_input_enabled ? 1 : 0;
    slot->adat_input_clock_mode = adat_clock_mode;

    // Stereo upmixer (V33; presence byte V34): RP2350 stores the live config;
    // RP2040 has no upmix config, so it zero-fills (fields are round-tripped
    // but never applied).
#if PICO_RP2350
    slot->upmix_enabled            = upmix_config.enabled ? 1 : 0;
    slot->upmix_center_mode        = upmix_config.center_mode;
    slot->upmix_surround_mode      = upmix_config.surround_mode;
    slot->upmix_presence_q1        = upmix_presence_encode(upmix_config.presence_db);
    slot->upmix_strength_pct       = upmix_config.strength_pct;
    slot->upmix_center_width_pct   = upmix_config.center_width_pct;
    slot->upmix_corr_threshold_pct = upmix_config.corr_threshold_pct;
    slot->upmix_attack_ms          = upmix_config.attack_ms;
    slot->upmix_release_ms         = upmix_config.release_ms;
    slot->upmix_detector_hpf_hz    = upmix_config.detector_hpf_hz;
    slot->upmix_surround_delay_ms  = upmix_config.surround_delay_ms;
    slot->upmix_surround_hpf_hz    = upmix_config.surround_hpf_hz;
    slot->upmix_surround_lpf_hz    = upmix_config.surround_lpf_hz;
    slot->upmix_decorr_pct         = upmix_config.decorr_pct;
#else
    slot->upmix_enabled            = 0;
    slot->upmix_center_mode        = 0;
    slot->upmix_surround_mode      = 0;
    slot->upmix_presence_q1        = 0;
    slot->upmix_strength_pct       = 0.0f;
    slot->upmix_center_width_pct   = 0.0f;
    slot->upmix_corr_threshold_pct = 0.0f;
    slot->upmix_attack_ms          = 0.0f;
    slot->upmix_release_ms         = 0.0f;
    slot->upmix_detector_hpf_hz    = 0.0f;
    slot->upmix_surround_delay_ms  = 0.0f;
    slot->upmix_surround_hpf_hz    = 0.0f;
    slot->upmix_surround_lpf_hz    = 0.0f;
    slot->upmix_decorr_pct         = 0.0f;
#endif

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
    slot->leveller_detector_mask = leveller_config.detector_mask;
    slot->leveller_apply_mask = leveller_config.apply_mask;

    // Per-channel preamp + Master volume — all inputs, direct.
    for (int i = 0; i < NUM_INPUT_CHANNELS; i++)
        slot->preamp_db_per_ch[i] = global_preamp_db[i];
    slot->master_volume_db = master_volume_db;

    // Input source (V13)
    slot->input_source = active_input_source;

    // LG Sound Sync (V14) — per-preset feature gate.  Live state captured
    // here; the runtime fields (present/volume/muted) are deliberately NOT
    // saved because they're observations of an external device and have no
    // meaning across a preset save/load cycle.
    slot->lg_sound_sync_enabled = lg_sound_sync_get_enabled() ? 1 : 0;

    // User volume (V15) — vol_index in [0, CENTER_VOLUME_INDEX].  Same shift
    // arithmetic as audio_set_volume(): audio_state.volume is signed 8.8 dB
    // with the slider's zero point at the centre, so add CENTER*256, clamp,
    // and truncate the 8-bit fractional to recover the index that the audio
    // path actually uses (db_to_vol[] lookup).
    {
        int32_t vol = (int32_t)audio_state.volume + (int32_t)CENTER_VOLUME_INDEX * 256;
        if (vol < 0) vol = 0;
        int32_t hi = ((int32_t)CENTER_VOLUME_INDEX + 1) * 256 - 1;
        if (vol > hi) vol = hi;
        slot->user_vol_index = (uint8_t)((uint32_t)vol >> 8u);
    }

    // Crossover bands (V16+).  Copy the recipes; the wire band index
    // (XOVER_BAND_BASE + i) is already baked into xover_recipes[][] by
    // xover_init_default_filters() and the live-edit path in main.c, so no
    // normalization is needed here.
    memcpy(slot->xover_recipes, (void *)xover_recipes, sizeof(slot->xover_recipes));

    // Compute CRC over the data section using THIS version's byte range
    // (immutable for the version we're saving; the validator looks it up
    // again on load via slot_data_size_for_version()).
    const uint8_t *data_start = (const uint8_t *)&slot->filter_recipes;
    slot->crc32 = crc32(data_start, slot_data_size_for_version(SLOT_DATA_VERSION));
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

// Re-derive the live master volume for a given preset *context*.  This is the
// single source of truth for what master volume becomes whenever the active
// preset context changes (preset load, active-slot delete, boot).  Callers that
// only reset the DSP processing chain without switching context (factory reset)
// must NOT call it — they leave the master-volume ceiling intact.
//
//   mode 1 (per-preset): master volume travels with the preset.  A configured
//     V12+ slot carries its own value; any context without a per-preset value
//     (an empty slot, or a legacy pre-V12 preset) is "factory defaults" and so
//     gets the power-on default — exactly as EQ resets to flat, delays to zero.
//   mode 0 (independent): master volume is decoupled from presets.  Only a BOOT
//     restore re-applies the saved device-level value; runtime context changes
//     leave the live value untouched, honoring the console contract "loading a
//     preset never changes it".  See
//     Documentation/Features/master_volume_independent_load.md.
//
// `slot_or_null` is the loaded slot, or NULL for an empty/factory-default
// context.  `is_boot` is true only on the power-on restore path.
static void apply_master_volume_from_mode(const PresetSlot *slot_or_null,
                                          bool is_boot) {
    if (dir_cache.master_volume_mode == MASTER_VOLUME_MODE_WITH_PRESET) {
        bool slot_has_value = slot_or_null && slot_or_null->version >= 12;
        apply_master_volume_db(slot_has_value ? slot_or_null->master_volume_db
                                              : MASTER_VOL_DEFAULT_DB);
    } else if (is_boot) {
        apply_master_volume_db(dir_cache.master_volume_db);
    }
    // Independent mode + runtime: intentionally a no-op (live value survives).
}

// Translate a filter type value stored by pre-V18 firmware into the current
// FilterType numbering.  Pre-V18 stored crossover types at 8..39; V18 moved
// them to 32..63 (putting the new first-order all-pass at 8 and leaving 9..31
// as PEQ padding).  PEQ types 0..7 are unchanged, and no first-order all-pass
// existed before V18, so only the old crossover range shifts (+24).  The
// literals 8/39/24 are the frozen pre-V18 wire values, deliberately NOT the
// (now-renumbered) FILTER_* names.
static inline uint8_t remap_filter_type_pre_v18(uint8_t old_type) {
    if (old_type >= 8 && old_type <= 39) return (uint8_t)(old_type + 24);
    return old_type;
}

// Apply a validated PresetSlot to the live DSP state.
// Physical IO config (output pins/types, I2S MCK/BCK, SPDIF RX pin) and master
// volume are *not* touched here — callers invoke apply_output_config_from_mode()
// and apply_master_volume_from_mode() separately after this returns, so each is
// sourced per its own mode (per-preset vs device-global) without requiring a
// preset reload to take effect.
// Does NOT trigger filter recalculation — caller must do that.
static void apply_slot_to_live(const PresetSlot *slot) {
    // EQ
    memcpy((void *)filter_recipes, slot->filter_recipes, sizeof(filter_recipes));
    // Normalize the per-band bypass byte at the boundary.  Legacy presets
    // saved before this field existed stored 0 in that slot (the old
    // EqParamPacket.reserved); pre-existing 0 → not bypassed → safe.  The
    // explicit normalization defends against garbage from any future code
    // path or corrupted flash.  See Documentation/Features/band_bypass_spec.md.
    // Also normalize .channel/.band.  Presets saved before the
    // dsp_init_default_filters() channel/band fix (or by any path that
    // didn't fully populate the recipe) may have these at zero, which would
    // cause REQ_SET_BAND_BYPASS to misroute writes to slot (0,0).
    // V18 renumbered the FilterType value space; migrate type values from any
    // pre-V18 slot as they are loaded so existing presets keep their filters.
    bool pre_v18 = (slot->version < 18);
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        for (int b = 0; b < MAX_BANDS; b++) {
            filter_recipes[ch][b].bypass = (filter_recipes[ch][b].bypass == 1) ? 1 : 0;
            filter_recipes[ch][b].channel = ch;
            filter_recipes[ch][b].band = b;
            if (pre_v18)
                filter_recipes[ch][b].type = remap_filter_type_pre_v18(filter_recipes[ch][b].type);
        }
    }

    // Linkwitz Transform per-band target Q (V30): older slots have no qp data,
    // so restore all-zero (0 selects the 0.707 default) for them.
    if (slot->version >= 30)
        memcpy(peq_qp_x512, slot->peq_qp_x512, sizeof(peq_qp_x512));
    else
        memset(peq_qp_x512, 0, sizeof(peq_qp_x512));

    // Per-channel preamp — all inputs, direct.
    for (int i = 0; i < NUM_INPUT_CHANNELS; i++) {
        global_preamp_db[i] = slot->preamp_db_per_ch[i];
        float linear = db_to_linear(slot->preamp_db_per_ch[i]);
        global_preamp_mul[i] = (int32_t)(linear * (float)(1 << 28));
        global_preamp_linear[i] = linear;
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
    if (slot->version >= 26) loudness_output_mask = slot->loudness_output_mask;
    else loudness_output_mask = LOUDNESS_DEFAULT_OUTPUT_MASK;
    loudness_ref_spl = slot->loudness_ref_spl;
    loudness_intensity_pct = slot->loudness_intensity_pct;
    loudness_recompute_pending = true;

    // Crossfeed
    crossfeed_config.enabled = (slot->crossfeed_enabled != 0);
    crossfeed_config.preset = slot->crossfeed_preset;
    crossfeed_config.itd_enabled = (slot->crossfeed_itd_enabled != 0);
    crossfeed_config.output_pair_mask = (slot->version >= 27)
        ? (slot->crossfeed_output_pair_mask & ((1u << NUM_SPDIF_INSTANCES) - 1))
        : 0x01;
    crossfeed_config.custom_fc = slot->crossfeed_custom_fc;
    crossfeed_config.custom_feed_db = slot->crossfeed_custom_feed_db;
    crossfeed_update_pending = true;

    // Psychoacoustic bass (V31): older slots have no psybass data, so restore
    // the disabled/all-outputs defaults for them.  The pending flag is raised in
    // both branches so a load that turns psybass off unpublishes its coefficients.
    if (slot->version >= 31) {
        psybass_config.enabled       = (slot->psybass_enabled != 0);
        psybass_config.output_mask   = slot->psybass_output_mask;
        psybass_config.cutoff_hz     = slot->psybass_cutoff_hz;
        psybass_config.harmonics_db  = slot->psybass_harmonics_db;
        psybass_config.drive_db      = slot->psybass_drive_db;
        psybass_config.character_pct = slot->psybass_character_pct;
        psybass_config.original_db   = slot->psybass_original_db;
    } else {
        psybass_config.enabled       = false;
        psybass_config.output_mask   = PSYBASS_DEFAULT_OUTPUT_MASK;
        psybass_config.cutoff_hz     = PSYBASS_DEFAULT_CUTOFF;
        psybass_config.harmonics_db  = PSYBASS_DEFAULT_HARMONICS;
        psybass_config.drive_db      = PSYBASS_DEFAULT_DRIVE;
        psybass_config.character_pct = PSYBASS_DEFAULT_CHARACTER;
        psybass_config.original_db   = PSYBASS_DEFAULT_ORIGINAL;
    }
    psybass_update_pending = true;

    // Stereo upmixer (V33): RP2350-only.  V33+ slots restore the stored config
    // (modes clamped; enabled = nonzero); older slots load the disabled
    // defaults.  Range clamping of the floats happens downstream in
    // upmix_compute_coefficients (matching the psybass/leveller precedent of a
    // raw copy plus downstream clamp), so no NaN/range sanitization here.  The
    // pending flag is raised in both branches so a load that turns the upmixer
    // off unpublishes its coefficients.
#if PICO_RP2350
    if (slot->version >= 33) {
        upmix_config.enabled            = (slot->upmix_enabled != 0);
        upmix_config.center_mode        = upmix_clamp_center_mode(slot->upmix_center_mode);
        upmix_config.surround_mode      = (slot->upmix_surround_mode <= 2)
                                          ? slot->upmix_surround_mode : UPMIX_DEFAULT_SURROUND_MODE;
        upmix_config.strength_pct       = slot->upmix_strength_pct;
        upmix_config.center_width_pct   = slot->upmix_center_width_pct;
        upmix_config.corr_threshold_pct = slot->upmix_corr_threshold_pct;
        upmix_config.attack_ms          = slot->upmix_attack_ms;
        upmix_config.release_ms         = slot->upmix_release_ms;
        upmix_config.detector_hpf_hz    = slot->upmix_detector_hpf_hz;
        upmix_config.surround_delay_ms  = slot->upmix_surround_delay_ms;
        upmix_config.surround_hpf_hz    = slot->upmix_surround_hpf_hz;
        upmix_config.surround_lpf_hz    = slot->upmix_surround_lpf_hz;
        upmix_config.decorr_pct         = slot->upmix_decorr_pct;
        // V34 presence byte; V33 slots wrote 0 here, which decodes to the
        // 0 dB default, so no version gate is needed.
        upmix_config.presence_db        = upmix_presence_decode(slot->upmix_presence_q1);
    } else {
        upmix_config.enabled            = false;
        upmix_config.center_mode        = UPMIX_DEFAULT_CENTER_MODE;
        upmix_config.surround_mode      = UPMIX_DEFAULT_SURROUND_MODE;
        upmix_config.strength_pct       = UPMIX_DEFAULT_STRENGTH;
        upmix_config.center_width_pct   = UPMIX_DEFAULT_WIDTH;
        upmix_config.corr_threshold_pct = UPMIX_DEFAULT_THRESH;
        upmix_config.attack_ms          = UPMIX_DEFAULT_ATTACK;
        upmix_config.release_ms         = UPMIX_DEFAULT_RELEASE;
        upmix_config.detector_hpf_hz    = UPMIX_DEFAULT_DET_HPF;
        upmix_config.surround_delay_ms  = UPMIX_DEFAULT_SUR_DELAY;
        upmix_config.surround_hpf_hz    = UPMIX_DEFAULT_SUR_HPF;
        upmix_config.surround_lpf_hz    = UPMIX_DEFAULT_SUR_LPF;
        upmix_config.decorr_pct         = UPMIX_DEFAULT_DECORR;
        upmix_config.presence_db        = UPMIX_DEFAULT_PRESENCE;
    }
    upmix_update_pending = true;
#endif

    // Matrix mixer — all inputs, direct.
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

    // Pin configuration, output types, I2S MCK/BCK and the SPDIF RX pin are
    // applied by apply_output_config_from_mode() (per output_config_mode), not
    // here — see the PHYSICAL IO / OUTPUT CONFIG section above.

    // Channel names (V8+).  Pre-V8 slots predate output_types (V9+) and
    // input_source (V13+) too, so fall back to firmware boot defaults
    // (INPUT_SOURCE_USB, all-SPDIF outputs via NULL output_types).
    if (slot->version >= 8) {
        memcpy(channel_names, slot->channel_names, sizeof(channel_names));
    } else {
        for (int ch = 0; ch < NUM_CHANNELS; ch++)
            get_default_channel_name(ch, INPUT_SOURCE_USB, NULL, channel_names[ch]);
    }

    // I2S/output-type config moved to apply_output_config_from_mode() (above).

    // Volume Leveller (V10+)
    if (slot->version >= 10) {
        leveller_config.enabled = (slot->leveller_enabled != 0);
        leveller_config.speed = slot->leveller_speed;
        leveller_config.lookahead = (slot->leveller_lookahead != 0);
        leveller_config.amount = slot->leveller_amount;
        leveller_config.max_gain_db = slot->leveller_max_gain_db;
        leveller_config.gate_threshold_db = slot->leveller_gate_threshold_db;
        // Channel masks are V25+; older slots load the all-channels default.
        if (slot->version >= 25) {
            leveller_config.detector_mask = slot->leveller_detector_mask;
            leveller_config.apply_mask = slot->leveller_apply_mask;
        } else {
            leveller_config.detector_mask = LEVELLER_DEFAULT_DETECTOR_MASK;
            leveller_config.apply_mask = LEVELLER_DEFAULT_APPLY_MASK;
        }
    } else {
        leveller_config.enabled = LEVELLER_DEFAULT_ENABLED;
        leveller_config.amount = LEVELLER_DEFAULT_AMOUNT;
        leveller_config.speed = LEVELLER_DEFAULT_SPEED;
        leveller_config.max_gain_db = LEVELLER_DEFAULT_MAX_GAIN_DB;
        leveller_config.lookahead = LEVELLER_DEFAULT_LOOKAHEAD;
        leveller_config.gate_threshold_db = LEVELLER_DEFAULT_GATE_DB;
        leveller_config.detector_mask = LEVELLER_DEFAULT_DETECTOR_MASK;
        leveller_config.apply_mask = LEVELLER_DEFAULT_APPLY_MASK;
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

    // LG Sound Sync per-preset toggle (V14+).  Pre-V14 slots fall back to
    // the firmware default (off), so existing user presets continue to
    // behave exactly as they always did until the user explicitly enables
    // the feature and re-saves.  Using the public setter here is correct:
    //   - We're inside preset_load's notify_begin_bulk() bracket, so the
    //     PARAM_CHANGED it emits is suppressed (BULK_INVALIDATED at end
    //     covers it).
    //   - The setter's side-effects (demote+restore on disable, streak
    //     reset on enable) are exactly what we want when a preset load
    //     changes the gate.
    //   - lg_sound_sync_on_preset_loaded() below handles the streaks-reset
    //     case where the gate did NOT change but the TV state may have.
    bool lg_en = (slot->version >= 14)
                     ? (slot->lg_sound_sync_enabled != 0)
                     : (LG_SOUND_SYNC_DEFAULT_ENABLED != 0);
    /* Two calls, two responsibilities — keep both:
     *   set_enabled() handles the enable-bit transition and its side-
     *     effects (notify, streak reset on dis→en, demote+restore on
     *     en→dis).  No-op when the loaded slot's enabled matches live.
     *   on_preset_loaded() resets the detection streaks unconditionally
     *     so the next tick re-evaluates the signature even when the
     *     enable bit didn't change.  The TV may have changed state
     *     during the preset transition, so volume/muted observed
     *     before the load are non-authoritative.
     * Dropping either call introduces a subtle bug — the redundancy in
     * the en→dis case (both demote) is harmless and well worth the
     * clarity of having each function own its single concern. */
    lg_sound_sync_set_enabled(lg_en);
    lg_sound_sync_on_preset_loaded();

    // User volume (V15+).  Pre-V15 slots leave user volume UNTOUCHED — the
    // user wasn't expecting that preset to set their listening level when
    // they originally saved it, so don't surprise them on load.  Asymmetric
    // vs master volume, which always applies a value (independent or per-
    // preset) — but master volume's "default" is the directory's saved value,
    // which user volume doesn't have.  When this hook IS taken, route the
    // restore through update_user_volume() so vol_mul + the loudness
    // coefficient pointer + the LG cache invalidation + the v2 notify all
    // happen via the single funnel — same path REQ_SET_USER_VOLUME uses.
    // The notify's PARAM_CHANGED is suppressed by the surrounding bulk
    // bracket; BULK_INVALIDATED at notify_end_bulk() covers it.
    if (slot->version >= 15) {
        uint8_t idx = slot->user_vol_index;
        if (idx > CENTER_VOLUME_INDEX) idx = CENTER_VOLUME_INDEX;
        // Reconstruct integer dB from the saved index.  Lossless against the
        // index storage; fractional dB in audio_state.volume is rebuilt as
        // exactly N.0 dB (which is what apply_vol_index_to_audio uses anyway
        // since it truncates the fractional).
        float db = (float)((int32_t)idx - (int32_t)CENTER_VOLUME_INDEX);
        update_user_volume(db);
    }

    // Crossover bands (V16+).  Pre-V16 slots predate this feature; reset
    // crossovers to defaults rather than leaving whatever live values
    // existed (matches the model where preset load is a complete state
    // restore).  In either case re-normalize the wire band index
    // (XOVER_BAND_BASE + i) defensively; a stale local index leaking into pending_packet via
    // REQ_SET_BAND_BYPASS would misroute the next live edit into PEQ band 0.
    if (slot->version >= 16) {
        memcpy((void *)xover_recipes, slot->xover_recipes, sizeof(xover_recipes));
        for (int ch = 0; ch < NUM_CHANNELS; ch++) {
            for (int i = 0; i < MAX_XOVER_BANDS; i++) {
                xover_recipes[ch][i].channel = (uint8_t)ch;
                xover_recipes[ch][i].band    = (uint8_t)(XOVER_BAND_BASE + i);
                xover_recipes[ch][i].bypass  = (xover_recipes[ch][i].bypass == 1) ? 1 : 0;
                if (pre_v18)
                    xover_recipes[ch][i].type = remap_filter_type_pre_v18(xover_recipes[ch][i].type);
            }
        }
    } else {
        // V<16: no crossover data in the slot — apply defaults.
        xover_init_default_filters();
    }
}

// ============================================================================
// SLOT VALIDATION
// ============================================================================

// CRC byte range = the slot data section (filter_recipes .. end of the version's
// fields).  V21 ended at i2s_input_rate; V22 appended the I2S multichannel
// fields, so V21's range stops where those begin.  V23 appended the ADAT
// fields, so V22's range stops where those begin.  V24 appended the optional
// SPDIF inputs 2/3; V25 appended the leveller channel masks; V26 appended the
// loudness output mask; V27 appended the crossfeed output pair mask; V28
// appended i2s_clock_mode; V29 appended i2s_clock_pin_mode + i2s_bck_pin_slave;
// V30 appended peq_qp_x512; V31 appended psybass; V32 appended the ADAT input
// fields; V33 appended the stereo upmixer config; V35 appended the SPDIF input
// 4 pin, so each version's range stops where the next version's fields begin
// (a stored slot's CRC was computed without the fields its version predates).
#define SLOT_DATA_SIZE_V21 \
    (offsetof(PresetSlot, i2s_input_channels) - offsetof(PresetSlot, filter_recipes))
#define SLOT_DATA_SIZE_V22 \
    (offsetof(PresetSlot, adat_enabled) - offsetof(PresetSlot, filter_recipes))
#define SLOT_DATA_SIZE_V23 \
    (offsetof(PresetSlot, spdif_rx_enabled_ext) - offsetof(PresetSlot, filter_recipes))
#define SLOT_DATA_SIZE_V24 \
    (offsetof(PresetSlot, leveller_detector_mask) - offsetof(PresetSlot, filter_recipes))
#define SLOT_DATA_SIZE_V25 \
    (offsetof(PresetSlot, loudness_output_mask) - offsetof(PresetSlot, filter_recipes))
#define SLOT_DATA_SIZE_V26 \
    (offsetof(PresetSlot, crossfeed_output_pair_mask) - offsetof(PresetSlot, filter_recipes))
#define SLOT_DATA_SIZE_V27 \
    (offsetof(PresetSlot, i2s_clock_mode) - offsetof(PresetSlot, filter_recipes))
#define SLOT_DATA_SIZE_V28 \
    (offsetof(PresetSlot, i2s_clock_pin_mode) - offsetof(PresetSlot, filter_recipes))
#define SLOT_DATA_SIZE_V29 \
    (offsetof(PresetSlot, peq_qp_x512) - offsetof(PresetSlot, filter_recipes))
#define SLOT_DATA_SIZE_V30 \
    (offsetof(PresetSlot, psybass_enabled) - offsetof(PresetSlot, filter_recipes))
#define SLOT_DATA_SIZE_V31 \
    (offsetof(PresetSlot, adat_input_pin) - offsetof(PresetSlot, filter_recipes))
#define SLOT_DATA_SIZE_V32 \
    (offsetof(PresetSlot, upmix_enabled) - offsetof(PresetSlot, filter_recipes))
#define SLOT_DATA_SIZE_V33 \
    (offsetof(PresetSlot, spdif_rx_pin4) - offsetof(PresetSlot, filter_recipes))
// V34 claims the upmix reserved byte (presence); no size change.
#define SLOT_DATA_SIZE_V34 SLOT_DATA_SIZE_V33
#define SLOT_DATA_SIZE_V35 \
    (sizeof(PresetSlot) - offsetof(PresetSlot, filter_recipes))

// V21 broke compatibility (unified channel model); V22 (I2S multichannel input),
// V23 (ADAT bulk output), V24 (optional SPDIF inputs 2/3), V25 (leveller channel
// masks), V26 (loudness output mask), V27 (crossfeed output pair mask), V28
// (I2S clock master/slave mode), V29 (I2S clock-pin mode), V30 (Linkwitz
// Transform per-band target Q), V31 (psychoacoustic bass), V32 (ADAT input) and
// V33 (stereo upmixer) and V35 (SPDIF input 4 pin) are backward-compatible
// tail-appends; V34 (upmix presence) claims a reserved byte with no size
// change.  V21..V35 slots are all accepted (an older slot loads with the newer
// fields defaulted to unset) while older/unknown versions are invalidated and
// the slot loads factory defaults.
static size_t slot_data_size_for_version(uint8_t version) {
    switch (version) {
        case SLOT_DATA_VERSION:   // 35
            return SLOT_DATA_SIZE_V35;
        case 34:
            return SLOT_DATA_SIZE_V34;
        case 33:
            return SLOT_DATA_SIZE_V33;
        case 32:
            return SLOT_DATA_SIZE_V32;
        case 31:
            return SLOT_DATA_SIZE_V31;
        case 30:
            return SLOT_DATA_SIZE_V30;
        case 29:
            return SLOT_DATA_SIZE_V29;
        case 28:
            return SLOT_DATA_SIZE_V28;
        case 27:
            return SLOT_DATA_SIZE_V27;
        case 26:
            return SLOT_DATA_SIZE_V26;
        case 25:
            return SLOT_DATA_SIZE_V25;
        case 24:
            return SLOT_DATA_SIZE_V24;
        case 23:
            return SLOT_DATA_SIZE_V23;
        case 22:
            return SLOT_DATA_SIZE_V22;
        case 21:
            return SLOT_DATA_SIZE_V21;
        default:
            return 0;
    }
}

// Read and validate a preset slot from flash.
// Returns a pointer to the flash-mapped slot if valid, NULL otherwise.
static const PresetSlot *validate_slot(uint8_t slot) {
    const PresetSlot *s = SLOT_ADDR(slot);
    if (s->magic != SLOT_MAGIC) return NULL;
    if (s->slot_index != slot) return NULL;

    size_t data_len = slot_data_size_for_version(s->version);
    if (data_len == 0) return NULL;

    const uint8_t *data_start = (const uint8_t *)&s->filter_recipes;
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

    const PresetSlot *loaded_slot = NULL;
    if (dir_cache.slot_occupied & (1u << slot)) {
        // Slot has user data — validate and load it
        const PresetSlot *s = validate_slot(slot);
        if (!s) {
            preset_loading = false;
            notify_end_bulk();
            return PRESET_ERR_CRC;
        }
        apply_slot_to_live(s);
        loaded_slot = s;
    } else {
        // Slot not configured — apply factory defaults
        apply_factory_defaults();
    }
    // Runtime context switch: re-derive master volume AND physical IO config for
    // the loaded context (loaded_slot, or NULL for the empty/factory case).  In
    // each subsystem's independent mode this is a no-op so the live value
    // survives — loading a preset never changes a device-global setting.
    apply_master_volume_from_mode(loaded_slot, false);
    apply_output_config_from_mode(loaded_slot, false);

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

    // Erase the slot's full flash allocation (SLOT_BYTES = all SLOT_SECTORS,
    // 2 sectors on RP2350) so no stale data lingers in the second sector.
    // Same lockout guard as flash_write_sector.
    bool do_lockout = multicore_lockout_victim_is_initialized(1)
                      && (__get_current_exception() == 0);
    if (do_lockout) multicore_lockout_start_blocking();

    // Same treatment as flash_write_sector(): PDM ring to true silence (only
    // when Core 1 is parked), then a blackout that leaves the output DMA IRQ
    // lines alive so the slots keep clocking through the erase.
    if (do_lockout) pdm_flash_silence();

    flash_irq_blackout_begin();
    dspi_flash_range_erase(SLOT_SECTOR_OFFSET(slot), SLOT_BYTES);
    flash_irq_blackout_end();

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
        // flash_mute_hold_samples (not PRESET_MUTE_SAMPLES): this write
        // follows dir_flush()'s floored re-arm and must not lower the
        // counter below a pending hardware-mute hold (see the helper).
        preset_mute_counter = flash_mute_hold_samples();
        preset_loading = true;
        __dmb();

        apply_factory_defaults();
        // The active preset context is now empty — re-derive master volume and
        // physical IO the same way loading an empty preset does (with-preset =>
        // factory default, independent => untouched).
        apply_master_volume_from_mode(NULL, false);
        apply_output_config_from_mode(NULL, false);

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
                          uint8_t *output_config_mode, uint8_t *master_volume_mode) {
    dir_ensure();
    *slot_occupied      = dir_cache.slot_occupied;
    *startup_mode       = dir_cache.startup_mode;
    *default_slot       = dir_cache.default_slot;
    *last_active        = dir_cache.last_active_slot;
    *output_config_mode = dir_cache.output_config_mode;
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

void preset_set_output_config_mode(uint8_t mode) {
    if (mode > OUTPUT_CONFIG_MODE_WITH_PRESET) mode = OUTPUT_CONFIG_MODE_INDEPENDENT;
    dir_ensure();
    dir_cache.output_config_mode = mode;
    dir_flush();
}

void preset_set_master_volume_mode(uint8_t mode) {
    if (mode > MASTER_VOLUME_MODE_WITH_PRESET) mode = MASTER_VOLUME_MODE_INDEPENDENT;
    dir_ensure();
    dir_cache.master_volume_mode = mode;
    dir_flush();
}

// DAC hardware mute persistence.  Mirrors the output_config_mode / master_volume_mode
// setters: synchronous, main-loop only, writes the directory sector and
// blocks for the ~45 ms flash erase+program.  Caller (dac_hw_mute_set_config)
// must have already validated the config — no validation here.
void preset_set_dac_hw_mute(const DacHwMuteConfig *cfg) {
    if (!cfg) return;
    dir_ensure();
    memcpy(&dir_cache.dac_hw_mute, cfg, sizeof(dir_cache.dac_hw_mute));
    dir_flush();
}

void preset_get_dac_hw_mute(DacHwMuteConfig *out) {
    if (!out) return;
    dir_ensure();
    memcpy(out, &dir_cache.dac_hw_mute, sizeof(*out));
}

// Control-interface (UART/I2C) persistence.  Mirrors preset_set_dac_hw_mute:
// synchronous, main-loop only, writes the directory sector once.  A NULL
// pointer leaves that interface's stored config unchanged.  Callers must have
// already validated the configs; sanitize-on-load is only a corruption guard.
void preset_set_ctrl_iface(const UartCtrlConfig *uart, const I2cCtrlConfig *i2c) {
    if (!uart && !i2c) return;
    dir_ensure();
    if (uart) memcpy(&dir_cache.uart_ctrl, uart, sizeof(dir_cache.uart_ctrl));
    if (i2c)  memcpy(&dir_cache.i2c_ctrl,  i2c,  sizeof(dir_cache.i2c_ctrl));
    dir_flush();
}

void preset_get_ctrl_iface(UartCtrlConfig *uart_out, I2cCtrlConfig *i2c_out) {
    dir_ensure();
    if (uart_out) memcpy(uart_out, &dir_cache.uart_ctrl, sizeof(*uart_out));
    if (i2c_out)  memcpy(i2c_out,  &dir_cache.i2c_ctrl,  sizeof(*i2c_out));
}

// Control Surfaces persistence.  The getter reads the RAM cache; writes go
// through preset_set_cs_all below (bindings persist only via REQ_CS_SAVE).
void preset_get_cs_config(CsFlashConfig *out) {
    if (!out) return;
    dir_ensure();
    memcpy(out, &dir_cache.cs_config, sizeof(*out));
}

// Control Surfaces IR command table (V11).  Getter reads the RAM cache; setter
// persists bindings and the IR table together in one directory-sector write
// (the REQ_CS_SAVE path).  Caller has validated both blobs.
void preset_get_cs_ir_config(CsIrConfig *out) {
    if (!out) return;
    dir_ensure();
    memcpy(out, &dir_cache.cs_ir, sizeof(*out));
}

uint8_t preset_set_cs_all(const CsFlashConfig *cfg, const CsIrConfig *ir,
                          const char (*names)[CS_NAME_LEN]) {
    if (!cfg || !ir || !names) return PRESET_ERR_INVALID_SLOT;
    dir_ensure();
    memcpy(&dir_cache.cs_config, cfg, sizeof(dir_cache.cs_config));
    dir_cache.cs_config.version = CS_CONFIG_VERSION;
    memcpy(&dir_cache.cs_ir, ir, sizeof(dir_cache.cs_ir));
    dir_cache.cs_ir.version = CS_IR_CONFIG_VERSION;
    memcpy(dir_cache.cs_names, names, sizeof(dir_cache.cs_names));
    for (uint8_t s = 0; s < CS_MAX_BINDINGS; s++)
        dir_cache.cs_names[s][CS_NAME_LEN - 1] = '\0';
    if (dir_flush() != 0) {
        return PRESET_ERR_FLASH_WRITE;
    }
    return PRESET_OK;
}

// Control Surfaces slot names (V10).  Reads the RAM cache; the engine loads
// its live name table from here at boot and on REQ_CS_REVERT.
uint8_t preset_get_cs_name(uint8_t slot, char *name_out) {
    if (slot >= CS_MAX_BINDINGS) return PRESET_ERR_INVALID_SLOT;
    dir_ensure();
    memcpy(name_out, dir_cache.cs_names[slot], CS_NAME_LEN);
    return PRESET_OK;
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

// Snapshot the live physical IO config into the directory's device-global block
// and persist.  This is the explicit "save output config" action for INDEPENDENT
// mode (REQ_SAVE_OUTPUT_CONFIG); accepted in both modes — in WITH_PRESET it is
// dormant until the user switches to INDEPENDENT.  Mirrors
// preset_save_master_volume().  Also keeps the legacy device-level spdif_rx_pin
// field in sync so directory reads stay coherent.
uint8_t preset_save_output_config(void) {
    dir_ensure();
    io_config_from_live(&dir_cache.output_config);
    dir_cache.spdif_rx_pin = dir_cache.output_config.spdif_rx_pin;
    if (dir_flush() != 0) return PRESET_ERR_FLASH_WRITE;
    return PRESET_OK;
}

uint8_t preset_get_active(void) {
    dir_ensure();
    return dir_cache.last_active_slot;
}

// ============================================================================
// BOOT / MIGRATION
// ============================================================================

// Legacy single-sector migration was removed at V21 — compatibility is broken
// intentionally.  When no valid V21 directory exists on flash, the boot path
// falls to factory defaults rather than migrating old data.
static bool migrate_legacy(void) {
    return false;
}

int preset_boot_load(void) {
    // Try to load the preset directory from flash
    if (dir_load_cache()) {
        // The SPDIF RX pin (and the rest of the physical IO config) is restored
        // by apply_output_config_from_mode() below, per output_config_mode.

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
        const PresetSlot *boot_slot = NULL;
        if ((dir_cache.slot_occupied & (1u << target_slot))) {
            const PresetSlot *s = validate_slot(target_slot);
            if (s) {
                apply_slot_to_live(s);
                boot_slot = s;
            } else {
                // Corrupt data — fall back to factory defaults
                apply_factory_defaults();
            }
        } else {
            apply_factory_defaults();
        }
        // Boot restore: re-apply master volume AND physical IO per mode
        // (independent => saved directory value, with-preset => slot value,
        // NULL => firmware defaults).
        apply_master_volume_from_mode(boot_slot, true);
        apply_output_config_from_mode(boot_slot, true);

        dir_cache.last_active_slot = target_slot;
        return FLASH_OK;
    }

    // No directory — try legacy migration
    if (migrate_legacy()) {
        // Migration succeeded; slot 0 is now populated.  Load it.
        const PresetSlot *s = validate_slot(0);
        if (s) {
            apply_slot_to_live(s);
        } else {
            apply_factory_defaults();
        }
        apply_master_volume_from_mode(s, true);   // boot restore (s==NULL on fail)
        apply_output_config_from_mode(s, true);
        return FLASH_OK;
    }

    // First boot, no legacy data — initialize directory and use slot 0
    dir_ensure();
    dir_flush();
    apply_factory_defaults();
    apply_master_volume_from_mode(NULL, true);   // boot restore (default value)
    apply_output_config_from_mode(NULL, true);
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

// flash_load_params() (the legacy synchronous "revert to saved") was removed
// when its only caller — the deprecated REQ_LOAD_PARAMS (0x52) — was repurposed
// as REQ_SAVE_OUTPUT_CONFIG.  Hosts use the deferred, SPDIF-safe REQ_PRESET_LOAD
// (0x91) instead.

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

    // Master volume is intentionally NOT touched here.  This resets only the
    // DSP processing chain; the master-volume ceiling is owned exclusively by
    // apply_master_volume_from_mode(), which the preset-*context* callers
    // (preset_load / preset_delete / preset_boot_load) invoke after this
    // returns.  flash_factory_reset() deliberately does not, so a factory reset
    // leaves the ceiling intact.  See
    // Documentation/Features/master_volume_independent_load.md.

    // Vendor user mute — session-only flag (not persisted); clear on factory
    // reset so a previously-asserted vendor mute doesn't leave the device
    // silent with no UI handle on it.  When called from flash_factory_reset()
    // and preset_load() the surrounding bulk bracket suppresses this notify
    // and emits BULK_INVALIDATED instead; from preset_delete() (active-slot
    // branch) and preset_boot_load() the notify fires bare — same pattern as
    // the pre-existing master-volume / lg-sound-sync notifies in those paths.
    // Defensive guard avoids a no-op notify when already false.
    if (user_mute) {
        user_mute = false;
        uint8_t v = 0;
        notify_param_write(offsetof(WireBulkParams, user_volume.user_mute),
                           sizeof(uint8_t), &v);
    }

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
    loudness_output_mask = LOUDNESS_DEFAULT_OUTPUT_MASK;
    loudness_ref_spl = 87.0f;
    loudness_intensity_pct = 100.0f;
    loudness_recompute_pending = true;

    // Crossfeed
    crossfeed_config.enabled = false;
    crossfeed_config.itd_enabled = true;
    crossfeed_config.preset = CROSSFEED_PRESET_DEFAULT;
    crossfeed_config.output_pair_mask = 0x01;
    crossfeed_config.custom_fc = 700.0f;
    crossfeed_config.custom_feed_db = 4.5f;
    crossfeed_update_pending = true;

    // Psychoacoustic bass
    psybass_config.enabled       = false;
    psybass_config.output_mask   = PSYBASS_DEFAULT_OUTPUT_MASK;
    psybass_config.cutoff_hz     = PSYBASS_DEFAULT_CUTOFF;
    psybass_config.harmonics_db  = PSYBASS_DEFAULT_HARMONICS;
    psybass_config.drive_db      = PSYBASS_DEFAULT_DRIVE;
    psybass_config.character_pct = PSYBASS_DEFAULT_CHARACTER;
    psybass_config.original_db   = PSYBASS_DEFAULT_ORIGINAL;
    psybass_update_pending = true;

    // Stereo upmixer (RP2350-only): disabled, default engine params.
#if PICO_RP2350
    upmix_config.enabled            = false;
    upmix_config.center_mode        = UPMIX_DEFAULT_CENTER_MODE;
    upmix_config.surround_mode      = UPMIX_DEFAULT_SURROUND_MODE;
    upmix_config.strength_pct       = UPMIX_DEFAULT_STRENGTH;
    upmix_config.center_width_pct   = UPMIX_DEFAULT_WIDTH;
    upmix_config.corr_threshold_pct = UPMIX_DEFAULT_THRESH;
    upmix_config.attack_ms          = UPMIX_DEFAULT_ATTACK;
    upmix_config.release_ms         = UPMIX_DEFAULT_RELEASE;
    upmix_config.detector_hpf_hz    = UPMIX_DEFAULT_DET_HPF;
    upmix_config.surround_delay_ms  = UPMIX_DEFAULT_SUR_DELAY;
    upmix_config.surround_hpf_hz    = UPMIX_DEFAULT_SUR_HPF;
    upmix_config.surround_lpf_hz    = UPMIX_DEFAULT_SUR_LPF;
    upmix_config.decorr_pct         = UPMIX_DEFAULT_DECORR;
    upmix_config.presence_db        = UPMIX_DEFAULT_PRESENCE;
    upmix_update_pending = true;
#endif

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

    // Physical IO config (output pins/types, I2S MCK/BCK, SPDIF RX pin) is NOT
    // reset here — it is owned by apply_output_config_from_mode(), which the
    // preset-context callers invoke after this returns (WITH_PRESET → firmware
    // IO defaults; INDEPENDENT → device-global value preserved).  Mirrors how
    // master volume is left to apply_master_volume_from_mode().

    // Reset channel names to factory defaults (USB input, all-SPDIF outputs).
    // active_input_source is reset below; pass NULL for output_types so the
    // function uses its all-SPDIF fallback for naming purposes.
    for (int ch = 0; ch < NUM_CHANNELS; ch++)
        get_default_channel_name(ch, INPUT_SOURCE_USB, NULL, channel_names[ch]);

    // Volume Leveller
    leveller_config.enabled = LEVELLER_DEFAULT_ENABLED;
    leveller_config.amount = LEVELLER_DEFAULT_AMOUNT;
    leveller_config.speed = LEVELLER_DEFAULT_SPEED;
    leveller_config.max_gain_db = LEVELLER_DEFAULT_MAX_GAIN_DB;
    leveller_config.lookahead = LEVELLER_DEFAULT_LOOKAHEAD;
    leveller_config.gate_threshold_db = LEVELLER_DEFAULT_GATE_DB;
    leveller_config.detector_mask = LEVELLER_DEFAULT_DETECTOR_MASK;
    leveller_config.apply_mask = LEVELLER_DEFAULT_APPLY_MASK;
    leveller_update_pending = true;
    leveller_reset_pending = true;

    // Input source: default to USB
    active_input_source = INPUT_SOURCE_USB;

    // LG Sound Sync — reset to firmware default (off).  Run through the
    // public setter so any side-effects (demote, restore vol_mul) fire
    // correctly if the feature happened to be active at the moment of
    // factory reset.  Inside flash_factory_reset() this is bracketed by
    // notify_begin_bulk(PARAM_SRC_FACTORY) so the PARAM_CHANGED is
    // suppressed in favor of the single trailing BULK_INVALIDATED.
    lg_sound_sync_set_enabled(LG_SOUND_SYNC_DEFAULT_ENABLED != 0);
}

void flash_factory_reset(void) {
    // Bracket so per-field writes in apply_factory_defaults() are suppressed
    // and the host sees exactly one BULK_INVALIDATED(source=FACTORY).
    notify_begin_bulk(PARAM_SRC_FACTORY);
    apply_factory_defaults();
    // Physical IO: WITH_PRESET => reset to firmware defaults (matches the old
    // factory-reset behavior); INDEPENDENT => leave the device-global config
    // intact (a DSP factory reset shouldn't wipe device-level wiring — same
    // philosophy as leaving the master-volume ceiling intact).
    dir_ensure();
    apply_output_config_from_mode(NULL, false);
    notify_end_bulk();
}
