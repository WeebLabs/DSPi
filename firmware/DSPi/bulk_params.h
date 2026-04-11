#ifndef BULK_PARAMS_H
#define BULK_PARAMS_H

/*
 * bulk_params.h — Wire format for bulk parameter transfer
 *
 * Defines a platform-independent binary format for transferring the complete
 * DSP state in a single USB control transfer.  Used for:
 *   - App startup (GET: read all params in one shot)
 *   - Post-preset-load UI sync (GET: refresh after preset switch)
 *   - Configuration restore (SET: apply a saved state)
 *
 * All multi-byte fields are little-endian (native ARM).  All float fields
 * are IEEE 754 single-precision.  Floats are at 4-byte-aligned offsets.
 *
 * Variable-dimension arrays (channels, outputs) are sized at platform
 * maximums.  The header's num_channels / num_output_channels fields tell
 * the host how many entries are valid; remaining entries are zero-padded.
 */

#include <stdint.h>
#include <stdbool.h>

// Fixed maximums for the wire format (sized for the largest platform)
#define WIRE_MAX_CHANNELS        11   // RP2350 max
#define WIRE_MAX_OUTPUT_CHANNELS  9   // RP2350 max
#define WIRE_MAX_INPUT_CHANNELS   2   // Same on both
#define WIRE_MAX_BANDS           12   // Same on both
#define WIRE_MAX_PIN_OUTPUTS      5   // RP2350 max (4 SPDIF + 1 PDM)
#define WIRE_NAME_LEN            32   // Must match PRESET_NAME_LEN

#define WIRE_FORMAT_VERSION       7   // V7: input source configuration
#define WIRE_MAX_SPDIF_INSTANCES  4   // RP2350 max

// Platform IDs
#define WIRE_PLATFORM_RP2040      0
#define WIRE_PLATFORM_RP2350      1

// ============================================================================
// Section 1: Packet Header (16 bytes)
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t  format_version;         // WIRE_FORMAT_VERSION
    uint8_t  platform_id;            // WIRE_PLATFORM_RP2040 or _RP2350
    uint8_t  num_channels;           // Actual channel count (7 or 11)
    uint8_t  num_output_channels;    // Actual output count (5 or 9)
    uint8_t  num_input_channels;     // Always 2
    uint8_t  max_bands;              // Bands per channel in this payload (12)
    uint16_t payload_length;         // Total packet size including header
    uint16_t fw_version_major;       // Firmware version
    uint16_t fw_version_minor;
    uint32_t reserved;               // Zero, future flags
} WireHeader;                        // 16 bytes

// ============================================================================
// Section 2: Global Parameters (16 bytes)
// ============================================================================
typedef struct __attribute__((packed)) {
    float    preamp_gain_db;         // Preamp gain
    uint8_t  bypass;                 // Master EQ bypass (0/1)
    uint8_t  loudness_enabled;       // Loudness compensation (0/1)
    uint8_t  reserved[2];
    float    loudness_ref_spl;       // Reference SPL
    float    loudness_intensity_pct; // Intensity percentage
} WireGlobalParams;                  // 16 bytes

// ============================================================================
// Section 3: Crossfeed Parameters (16 bytes)
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t  enabled;                // Crossfeed on/off
    uint8_t  preset;                 // Preset index
    uint8_t  itd_enabled;            // ITD simulation on/off
    uint8_t  reserved;
    float    custom_fc;              // Custom crossover frequency (Hz)
    float    custom_feed_db;         // Custom feed level (dB)
    uint32_t reserved2;              // Future expansion
} WireCrossfeedParams;               // 16 bytes

// ============================================================================
// Section 4: Legacy Channel Gain/Mute (16 bytes)
// ============================================================================
typedef struct __attribute__((packed)) {
    float    gain_db[3];             // Per-channel gain
    uint8_t  mute[3];               // Per-channel mute (0/1)
    uint8_t  reserved;
} WireLegacyChannels;                // 16 bytes

// ============================================================================
// Section 5: Per-Channel Delays (44 bytes, fixed at WIRE_MAX_CHANNELS)
// ============================================================================
typedef struct __attribute__((packed)) {
    float    delay_ms[WIRE_MAX_CHANNELS];  // ms, zero-padded beyond num_channels
} WireChannelDelays;                 // 44 bytes

// ============================================================================
// Section 6: Matrix Crosspoint (8 bytes each)
// Layout: input 0 outputs 0..8, then input 1 outputs 0..8 (row-major)
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t  enabled;
    uint8_t  phase_invert;
    uint8_t  reserved[2];
    float    gain_db;
} WireCrosspoint;                    // 8 bytes

// ============================================================================
// Section 7: Matrix Output Channel (12 bytes each)
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t  enabled;
    uint8_t  mute;
    uint8_t  reserved[2];
    float    gain_db;
    float    delay_ms;
} WireOutputChannel;                 // 12 bytes

// ============================================================================
// Section 8: Pin Configuration (8 bytes)
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t  num_pin_outputs;            // 3 on RP2040, 5 on RP2350
    uint8_t  pins[WIRE_MAX_PIN_OUTPUTS]; // GPIO pin numbers, zero-padded
    uint8_t  reserved[2];
} WirePinConfig;                         // 8 bytes

// ============================================================================
// Section 9: EQ Band Parameters (16 bytes each)
// Layout: channel 0 bands 0..11, channel 1 bands 0..11, ... (row-major)
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t  type;                   // Filter type enum
    uint8_t  reserved[3];
    float    freq;                   // Hz
    float    q;                      // Q factor
    float    gain_db;                // dB
} WireBandParams;                    // 16 bytes

// ============================================================================
// Section 10: Channel Names (352 bytes)
// ============================================================================
typedef struct __attribute__((packed)) {
    char names[WIRE_MAX_CHANNELS][WIRE_NAME_LEN];
} WireChannelNames;                  // 352 bytes

// ============================================================================
// Section 11: I2S Configuration (16 bytes) — V3+
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t  output_types[WIRE_MAX_SPDIF_INSTANCES]; // Per-slot: 0=S/PDIF, 1=I2S
    uint8_t  bck_pin;                // BCK GPIO (LRCLK = BCK + 1)
    uint8_t  mck_pin;                // MCK GPIO
    uint8_t  mck_enabled;            // 0 = off, 1 = on
    uint8_t  mck_multiplier;         // 128 or 256
    uint8_t  reserved[8];            // Future expansion (must be 0)
} WireI2SConfig;                     // 16 bytes

// ============================================================================
// Section 12: Volume Leveller Configuration (16 bytes) — V4+
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t  enabled;                // 0/1
    uint8_t  speed;                  // 0=Slow, 1=Medium, 2=Fast
    uint8_t  lookahead;              // 0/1 (10ms lookahead delay)
    uint8_t  reserved;
    float    amount;                 // 0.0-100.0 (compression strength %)
    float    max_gain_db;            // 0.0-35.0 (max boost for quiet content)
    float    gate_threshold_db;      // -96.0-0.0 (silence gate level dBFS)
} WireLevellerConfig;                // 16 bytes

// ============================================================================
// Section 13: Per-Channel Preamp Configuration (16 bytes) — V6+
// ============================================================================
typedef struct __attribute__((packed)) {
    float    preamp_db[WIRE_MAX_INPUT_CHANNELS]; // Per-input-channel preamp (dB), 0=L, 1=R
    uint8_t  reserved[8];                        // Pad to 16 bytes (future input channels)
} WirePreampConfig;                              // 16 bytes

// ============================================================================
// Section 14: Master Volume (16 bytes) — V6+
// ============================================================================
typedef struct __attribute__((packed)) {
    float    master_volume_db;   // Device master volume: -128 (mute sentinel), -127..0 dB range
    uint8_t  reserved[12];       // Future expansion (pad to 16 bytes)
} WireMasterVolume;              // 16 bytes

// ============================================================================
// Section 15: Input Source Configuration (16 bytes) — V7+
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t  input_source;           // InputSource enum (0=USB, 1=SPDIF)
    uint8_t  spdif_rx_pin;          // SPDIF RX GPIO pin (informational, SET does not apply)
    uint8_t  reserved[14];           // Future expansion (pad to 16 bytes)
} WireInputConfig;                   // 16 bytes

// ============================================================================
// Complete Packet
// ============================================================================
typedef struct __attribute__((packed)) {
    WireHeader          header;                                          //   16
    WireGlobalParams    global;                                          //   16
    WireCrossfeedParams crossfeed;                                       //   16
    WireLegacyChannels  legacy;                                          //   16
    WireChannelDelays   delays;                                          //   44
    WireCrosspoint      crosspoints[WIRE_MAX_INPUT_CHANNELS][WIRE_MAX_OUTPUT_CHANNELS];  // 144
    WireOutputChannel   outputs[WIRE_MAX_OUTPUT_CHANNELS];               //  108
    WirePinConfig       pins;                                             //    8
    WireBandParams      eq[WIRE_MAX_CHANNELS][WIRE_MAX_BANDS];           // 2112
    WireChannelNames    channel_names;                                    //  352
    WireI2SConfig       i2s_config;                                       //   16
    WireLevellerConfig  leveller;                                          //   16
    WirePreampConfig    preamp;                                            //   16
    WireMasterVolume    master_volume;                                     //   16
    WireInputConfig     input_config;                                      //   16
} WireBulkParams;                    // Total: 2912 bytes

#define WIRE_BULK_PARAMS_SIZE  sizeof(WireBulkParams)

// Buffer size for USB stream transfer (must be power of 2, >= WIRE_BULK_PARAMS_SIZE)
#define WIRE_BULK_BUF_SIZE     4096

// Collect current live DSP state into wire format
void bulk_params_collect(WireBulkParams *out);

// Apply wire format to live DSP state.  Returns 0 on success, nonzero on error.
// Caller must recalculate filters and delays after this returns.
// If apply_pins is true, output pin assignments from the payload are applied.
int bulk_params_apply(const WireBulkParams *in, bool apply_pins);

#endif // BULK_PARAMS_H
