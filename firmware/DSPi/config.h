#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include <limits.h>

// ----------------------------------------------------------------------------
// GLOBAL VARIABLES
// ----------------------------------------------------------------------------
extern volatile int overruns;
extern volatile uint32_t pio_samples_dma;

// Buffer monitoring counters
extern volatile uint32_t pdm_ring_overruns;   // Core 0 couldn't push (ring full)
extern volatile uint32_t pdm_ring_underruns;  // Core 1 needed sample but ring empty
extern volatile uint32_t pdm_dma_overruns;    // Core 1 write caught up to DMA read
extern volatile uint32_t pdm_dma_underruns;   // Core 1 write fell behind DMA read
extern volatile uint32_t spdif_overruns;      // USB callback couldn't get buffer (pool full)
extern volatile uint32_t spdif_underruns;     // USB packet gap > 2ms (consumer likely starved)
extern volatile uint32_t usb_audio_packets;   // Debug: count of USB audio packets received
extern volatile uint32_t usb_audio_alt_set;   // Debug: last alt setting selected
extern volatile uint32_t usb_audio_mounted;   // Debug: audio mounted state

// USB audio feedback (SOF-measured, 10.14 fixed-point)
extern volatile uint32_t feedback_10_14;
extern volatile uint32_t nominal_feedback_10_14;

// ----------------------------------------------------------------------------
// CONFIGURATION
// ----------------------------------------------------------------------------

#define ENABLE_SUB 1

// S/PDIF Output Pins
#undef PICO_AUDIO_SPDIF_PIN
#define PICO_AUDIO_SPDIF_PIN   6    // S/PDIF 1 (Out 1-2)
#define PICO_SPDIF_PIN_2       7    // S/PDIF 2 (Out 3-4)
#if PICO_RP2350
#define PICO_SPDIF_PIN_3       8    // S/PDIF 3 (Out 5-6) — RP2350 only
#define PICO_SPDIF_PIN_4       9    // S/PDIF 4 (Out 7-8) — RP2350 only
#endif

// PDM Subwoofer Output Pin (PIO1)
#define PICO_PDM_PIN           10   // PDM sub (Out 9)

// Legacy aliases
#define PICO_AUDIO_SPDIF_SUB_PIN PICO_PDM_PIN

// Clip detection threshold — slightly above 1.0 to avoid false positives
// from float precision noise when 0 dBFS signals pass through biquad filters.
// +0.01 dB headroom: catches audible clipping, ignores float rounding artifacts.
#define CLIP_THRESH_F   1.001f              // RP2350 float path
#define CLIP_THRESH_Q28 ((1 << 28) + 268)   // RP2040 Q28 path (~0.001 in Q28)

#define FILTER_SHIFT 28

// PDM Configuration
#define PDM_OVERSAMPLE 256
#define PDM_DMA_BUFFER_SIZE 2048  // Doubled for more margin (was 1024)
#define PDM_DMA_RING_BITS 13      // log2(2048 * 4 bytes) = 13
#define PDM_PIO pio1
#define PDM_SM 0
#define PDM_CLIP_THRESH 29500  // ~90% modulation (was 26214 / 80%)

// PDM Sigma-Delta Tuning
// Dither mask: controls TPDF amplitude. Start small (0x1FF), increase if idle tones persist
#define PDM_DITHER_MASK 0x1FF
// Leakage shift: higher = less leakage. Applied once per audio sample.
// 16 gives ~1.4s time constant at 48kHz, safe for bass
#define PDM_LEAKAGE_SHIFT 16

// PDM Soft Start/Stop
#define PDM_FADE_IN_SHIFT   10                        // 2^10 = 1024 samples
#define PDM_FADE_IN_SAMPLES (1u << PDM_FADE_IN_SHIFT) // ~21ms at 48kHz

// SPDIF Buffer Configuration
#define AUDIO_BUFFER_COUNT    8   // Producer buffers per SPDIF instance
#define SPDIF_CONSUMER_BUFFER_COUNT 16  // Consumer buffers per SPDIF instance (DMA side)
#define AUDIO_BUFFER_SAMPLES  192

// DELAY CONFIGURATION
#if PICO_RP2350
#define MAX_DELAY_SAMPLES 4096   // 85ms at 48kHz
#else
#define MAX_DELAY_SAMPLES 2048   // 42ms at 48kHz
#endif
#define MAX_DELAY_MASK    (MAX_DELAY_SAMPLES - 1)

// Latency alignment (in samples - automatically adapts to sample rate)
// SPDIF path: watermark = AUDIO_BUFFER_COUNT/2 buffers
// PDM path: DMA buffer = PDM_DMA_BUFFER_SIZE/8 PCM samples
#define SPDIF_BUFFER_SAMPLES  384  // Consumer path depth at 50% fill (16 buffers * 48 samples * 50%)
#define PDM_BUFFER_SAMPLES    (PDM_DMA_BUFFER_SIZE / 8)                           // 256
#define SUB_ALIGN_SAMPLES     (SPDIF_BUFFER_SAMPLES - PDM_BUFFER_SAMPLES)         // 128

// ----------------------------------------------------------------------------
// VENDOR INTERFACE CONFIGURATION (WinUSB / WCID)
// ----------------------------------------------------------------------------

#define VENDOR_INTERFACE_NUMBER  2

// RESTORED: Dummy Endpoint for macOS compatibility
#define VENDOR_EP_IN        0x83
#define VENDOR_EP_SIZE      64
#define VENDOR_EP_INTERVAL  10

// Microsoft WCID Vendor Code
#define MS_VENDOR_CODE      0x01

// Vendor Request Commands (EP0 control transfers)
#define REQ_SET_EQ_PARAM    0x42
#define REQ_GET_EQ_PARAM    0x43
#define REQ_SET_PREAMP      0x44
#define REQ_GET_PREAMP      0x45
#define REQ_SET_BYPASS      0x46
#define REQ_GET_BYPASS      0x47
#define REQ_SET_DELAY       0x48
#define REQ_GET_DELAY       0x49
#define REQ_GET_STATUS      0x50
#define REQ_SAVE_PARAMS     0x51
#define REQ_LOAD_PARAMS     0x52
#define REQ_FACTORY_RESET   0x53
#define REQ_SET_CHANNEL_GAIN 0x54
#define REQ_GET_CHANNEL_GAIN 0x55
#define REQ_SET_CHANNEL_MUTE 0x56
#define REQ_GET_CHANNEL_MUTE 0x57
#define REQ_SET_LOUDNESS            0x58
#define REQ_GET_LOUDNESS            0x59
#define REQ_SET_LOUDNESS_REF        0x5A
#define REQ_GET_LOUDNESS_REF        0x5B
#define REQ_SET_LOUDNESS_INTENSITY  0x5C
#define REQ_GET_LOUDNESS_INTENSITY  0x5D
#define REQ_SET_CROSSFEED           0x5E
#define REQ_GET_CROSSFEED           0x5F
#define REQ_SET_CROSSFEED_PRESET    0x60
#define REQ_GET_CROSSFEED_PRESET    0x61
#define REQ_SET_CROSSFEED_FREQ      0x62
#define REQ_GET_CROSSFEED_FREQ      0x63
#define REQ_SET_CROSSFEED_FEED      0x64
#define REQ_GET_CROSSFEED_FEED      0x65
#define REQ_SET_CROSSFEED_ITD       0x66
#define REQ_GET_CROSSFEED_ITD       0x67

// Matrix Mixer Commands
#define REQ_SET_MATRIX_ROUTE        0x70
#define REQ_GET_MATRIX_ROUTE        0x71
#define REQ_SET_OUTPUT_ENABLE       0x72
#define REQ_GET_OUTPUT_ENABLE       0x73
#define REQ_SET_OUTPUT_GAIN         0x74
#define REQ_GET_OUTPUT_GAIN         0x75
#define REQ_SET_OUTPUT_MUTE         0x76
#define REQ_GET_OUTPUT_MUTE         0x77
#define REQ_SET_OUTPUT_DELAY        0x78
#define REQ_GET_OUTPUT_DELAY        0x79

// Core 1 Mode Query Commands
#define REQ_GET_CORE1_MODE          0x7A
#define REQ_GET_CORE1_CONFLICT      0x7B

// Pin Configuration Commands
#define REQ_SET_OUTPUT_PIN          0x7C
#define REQ_GET_OUTPUT_PIN          0x7D

// Device Identification Commands
#define REQ_GET_SERIAL              0x7E
#define REQ_GET_PLATFORM            0x7F

// Clip Detection Commands
#define REQ_CLEAR_CLIPS             0x83

// Preset System Commands
#define REQ_PRESET_SAVE             0x90
#define REQ_PRESET_LOAD             0x91
#define REQ_PRESET_DELETE           0x92
#define REQ_PRESET_GET_NAME         0x93
#define REQ_PRESET_SET_NAME         0x94
#define REQ_PRESET_GET_DIR          0x95
#define REQ_PRESET_SET_STARTUP      0x96
#define REQ_PRESET_GET_STARTUP      0x97
#define REQ_PRESET_SET_INCLUDE_PINS 0x98
#define REQ_PRESET_GET_INCLUDE_PINS 0x99
#define REQ_PRESET_GET_ACTIVE       0x9A
#define REQ_SET_CHANNEL_NAME        0x9B
#define REQ_GET_CHANNEL_NAME        0x9C

// Bulk parameter transfer
#define REQ_GET_ALL_PARAMS          0xA0
#define REQ_SET_ALL_PARAMS          0xA1

// I2S Output Configuration Commands
#define REQ_SET_OUTPUT_TYPE         0xC0
#define REQ_GET_OUTPUT_TYPE         0xC1
#define REQ_SET_I2S_BCK_PIN         0xC2
#define REQ_GET_I2S_BCK_PIN         0xC3
#define REQ_SET_MCK_ENABLE          0xC4
#define REQ_GET_MCK_ENABLE          0xC5
#define REQ_SET_MCK_PIN             0xC6
#define REQ_GET_MCK_PIN             0xC7
#define REQ_SET_MCK_MULTIPLIER      0xC8
#define REQ_GET_MCK_MULTIPLIER      0xC9

// Buffer statistics
#define REQ_GET_BUFFER_STATS        0xB0
#define REQ_RESET_BUFFER_STATS      0xB1
#define REQ_GET_USB_ERROR_STATS     0xB2
#define REQ_RESET_USB_ERROR_STATS   0xB3

// Volume Leveller Commands
#define REQ_SET_LEVELLER_ENABLE     0xB4
#define REQ_GET_LEVELLER_ENABLE     0xB5
#define REQ_SET_LEVELLER_AMOUNT     0xB6
#define REQ_GET_LEVELLER_AMOUNT     0xB7
#define REQ_SET_LEVELLER_SPEED      0xB8
#define REQ_GET_LEVELLER_SPEED      0xB9
#define REQ_SET_LEVELLER_MAX_GAIN   0xBA
#define REQ_GET_LEVELLER_MAX_GAIN   0xBB
#define REQ_SET_LEVELLER_LOOKAHEAD  0xBC
#define REQ_GET_LEVELLER_LOOKAHEAD  0xBD
#define REQ_SET_LEVELLER_GATE       0xBE
#define REQ_GET_LEVELLER_GATE       0xBF

// Per-Channel Preamp Commands
#define REQ_SET_PREAMP_CH           0xD0  // wValue = channel index (0=L, 1=R), payload = float dB
#define REQ_GET_PREAMP_CH           0xD1  // wValue = channel index, returns float dB

// Master Volume Commands
#define REQ_SET_MASTER_VOLUME       0xD2  // payload = float dB (-128 mute sentinel, -127..0 range)
#define REQ_GET_MASTER_VOLUME       0xD3  // returns float dB
#define REQ_SET_MASTER_VOLUME_MODE  0xD4  // payload = uint8_t mode (see MASTER_VOLUME_MODE_*)
#define REQ_GET_MASTER_VOLUME_MODE  0xD5  // returns uint8_t mode
#define REQ_SAVE_MASTER_VOLUME      0xD6  // no payload, stores live master vol to directory
#define REQ_GET_SAVED_MASTER_VOLUME 0xD7  // returns float dB from directory's independent field

// Master Volume Constants
#define MASTER_VOL_MUTE_DB          (-128.0f)  // Sentinel value: true -inf (mute)
#define MASTER_VOL_MIN_DB           (-127.0f)  // Minimum non-mute attenuation
#define MASTER_VOL_MAX_DB           (0.0f)     // Unity gain (no attenuation)
#define MASTER_VOL_DEFAULT_DB       (-20.0f)   // Power-on / fresh-device default

// Master Volume Persistence Modes
// Mode 0 (default): master volume is independent of presets. REQ_SAVE_MASTER_VOLUME
//   stores it in the directory sector; it's applied at boot and on factory reset.
//   Preset save/load does not touch it.
// Mode 1: master volume is part of the preset. Saved with the preset, restored on
//   preset load — same as the old include_master_volume=1 behavior.
#define MASTER_VOLUME_MODE_INDEPENDENT   0
#define MASTER_VOLUME_MODE_WITH_PRESET   1

// System
#define REQ_ENTER_BOOTLOADER        0xF0

// Preset configuration
#define PRESET_SLOTS                10
#define PRESET_NAME_LEN             32

// Preset startup modes
#define PRESET_STARTUP_SPECIFIED    0   // Load a specific default slot
#define PRESET_STARTUP_LAST_ACTIVE  1   // Load whichever slot was last active

// Preset status codes
#define PRESET_OK                   0x00
#define PRESET_ERR_INVALID_SLOT     0x01
#define PRESET_ERR_SLOT_EMPTY       0x02
#define PRESET_ERR_CRC              0x03
#define PRESET_ERR_FLASH_WRITE      0x04

// Platform IDs
#define PLATFORM_RP2040             0
#define PLATFORM_RP2350             1

// Firmware version (BCD encoded: major in high byte, minor.patch in low byte)
#define FW_VERSION_MAJOR            1
#define FW_VERSION_MINOR            1
#define FW_VERSION_PATCH            3
#define FW_VERSION_BCD              ((FW_VERSION_MAJOR << 8) | (FW_VERSION_MINOR << 4) | FW_VERSION_PATCH)

// Pin config status codes (shared by S/PDIF, I2S, and MCK pin commands)
#define PIN_CONFIG_SUCCESS          0x00
#define PIN_CONFIG_INVALID_PIN      0x01
#define PIN_CONFIG_PIN_IN_USE       0x02
#define PIN_CONFIG_INVALID_OUTPUT   0x03
#define PIN_CONFIG_OUTPUT_ACTIVE    0x04

// Output type identifiers
#define OUTPUT_TYPE_SPDIF           0
#define OUTPUT_TYPE_I2S             1

// I2S default pins
#define PICO_I2S_BCK_PIN            14   // BCK; LRCLK = BCK + 1 = GPIO 15
#define PICO_I2S_MCK_PIN            13   // Master clock (optional)

// Number of configurable outputs (SPDIF + PDM)
#if PICO_RP2350
#define NUM_SPDIF_INSTANCES         4
#define NUM_PIN_OUTPUTS             5   // 4 SPDIF + 1 PDM
#else
#define NUM_SPDIF_INSTANCES         2
#define NUM_PIN_OUTPUTS             3   // 2 SPDIF + 1 PDM
#endif

// USB Audio Feature Unit IDs
#define FEATURE_MUTE_CONTROL 1u
#define FEATURE_VOLUME_CONTROL 2u
#define ENDPOINT_FREQ_CONTROL 1u

// Channel Definitions
#define CH_MASTER_LEFT   0
#define CH_MASTER_RIGHT  1
#define CH_OUT_1         2   // S/PDIF 1 L
#define CH_OUT_2         3   // S/PDIF 1 R
#define CH_OUT_3         4   // S/PDIF 2 L
#define CH_OUT_4         5   // S/PDIF 2 R
#if PICO_RP2350
// RP2350: 11 channels (2 master + 8 SPDIF + 1 PDM)
#define CH_OUT_5         6   // S/PDIF 3 L
#define CH_OUT_6         7   // S/PDIF 3 R
#define CH_OUT_7         8   // S/PDIF 4 L
#define CH_OUT_8         9   // S/PDIF 4 R
#define CH_OUT_9_PDM     10  // PDM sub
#define NUM_OUTPUT_CHANNELS  9
#define NUM_CHANNELS     11
#else
// RP2040: 7 channels (2 master + 4 SPDIF + 1 PDM)
#define CH_OUT_5_PDM     6   // PDM sub
#define NUM_OUTPUT_CHANNELS  5
#define NUM_CHANNELS     7
#endif
#define MAX_BANDS        12

// Legacy aliases for backward compatibility
#define CH_OUT_LEFT      CH_OUT_1
#define CH_OUT_RIGHT     CH_OUT_2
#if PICO_RP2350
#define CH_OUT_SUB       CH_OUT_9_PDM
#else
#define CH_OUT_SUB       CH_OUT_5_PDM
#endif

// Matrix Mixer Configuration
#define NUM_INPUT_CHANNELS   2   // USB L/R (expandable to 4 for S/PDIF input)

// Core 1 Operating Mode
typedef enum {
    CORE1_MODE_IDLE      = 0,
    CORE1_MODE_PDM       = 1,
    CORE1_MODE_EQ_WORKER = 2,
} Core1Mode;

// Outputs assigned to Core 1 EQ worker
#if PICO_RP2350
#define CORE1_EQ_FIRST_OUTPUT  2
#define CORE1_EQ_LAST_OUTPUT   7   // S/PDIF pairs 2-4 = outputs 2-7
#else
#define CORE1_EQ_FIRST_OUTPUT  2
#define CORE1_EQ_LAST_OUTPUT   3   // S/PDIF pair 2 = outputs 2-3
#endif

// Core 1 EQ Worker Handshake
typedef struct {
    volatile bool     work_ready;
    volatile bool     work_done;
#if PICO_RP2350
    float           (*buf_out)[192];   // Pointer to buf_out array, set once at init
    uint32_t          sample_count;
    float             vol_mul;
    uint32_t          delay_write_idx;  // Snapshot for Core 1 delay processing
    int32_t          *spdif_out[3];     // Pairs 1-3 output buffers (NULL = skip)
#else
    int32_t         (*buf_out)[192];   // Pointer to buf_out array (Q28), set once at init
    uint32_t          sample_count;
    int32_t           vol_mul;         // Q15 master volume
    uint32_t          delay_write_idx;
    int32_t          *spdif_out[1];    // SPDIF pair 2 output buffer (NULL = skip)
#endif
} Core1EqWork;

// ----------------------------------------------------------------------------
// DATA STRUCTURES
// ----------------------------------------------------------------------------

// Matrix Mixer Crosspoint
typedef struct __attribute__((packed)) {
    uint8_t enabled;        // Route active
    uint8_t phase_invert;   // Polarity flip
    uint8_t reserved[2];
    float gain_db;          // -inf to +12dB
    float gain_linear;      // Pre-computed multiplier
} MatrixCrosspoint;

// Output Channel State
typedef struct __attribute__((packed)) {
    uint8_t enabled;        // Output active (saves CPU when disabled)
    uint8_t mute;           // Soft mute
    uint8_t reserved[2];
    float gain_db;          // Per-output gain (-inf to +12dB)
    float gain_linear;      // Pre-computed
    float delay_ms;         // Per-output delay
    int32_t delay_samples;  // Pre-computed from sample rate
} OutputChannel;

// Matrix Mixer
typedef struct {
    MatrixCrosspoint crosspoints[NUM_INPUT_CHANNELS][NUM_OUTPUT_CHANNELS];
    OutputChannel outputs[NUM_OUTPUT_CHANNELS];
} MatrixMixer;

// Matrix Route Packet (for vendor commands)
typedef struct __attribute__((packed)) {
    uint8_t input;          // 0-1 (USB L/R)
    uint8_t output;         // 0-8
    uint8_t enabled;        // 0 or 1
    uint8_t phase_invert;   // 0 or 1
    float gain_db;          // -inf to +12dB
} MatrixRoutePacket;

#if PICO_RP2350
typedef struct {
    // Biquad coefficients
    float b0, b1, b2, a1, a2;
    float s1, s2;                              // single-precision state (was double)

    // SVF coefficients and state
    float sva1, sva2, sva3;                    // integrator coefficients
    float svm0, svm1, svm2;                    // output mix coefficients
    float svic1eq, svic2eq;                    // integrator state
    uint32_t svf_type;                         // FilterType enum for inner loop specialization

    bool use_svf;                              // true = SVF path, false = biquad path
    bool bypass;
} Biquad;
#else
typedef struct {
    int32_t b0, b1, b2, a1, a2;
    int32_t s1, s2;
    bool bypass;
} Biquad;
#endif

enum FilterType {
    FILTER_FLAT = 0, FILTER_PEAKING = 1, FILTER_LOWSHELF = 2,
    FILTER_HIGHSHELF = 3, FILTER_LOWPASS = 4, FILTER_HIGHPASS = 5
};

typedef struct __attribute__((packed)) {
    uint8_t channel;
    uint8_t band;
    uint8_t type;
    uint8_t reserved;
    float freq;
    float Q;
    float gain_db;
} EqParamPacket;

typedef struct {
    uint16_t peaks[NUM_CHANNELS];
    uint8_t cpu0_load;
    uint8_t cpu1_load;
    uint16_t clip_flags;         // Per-channel clip latch bitmask (sticky, cleared by REQ_CLEAR_CLIPS)
} SystemStatusPacket;

// ----------------------------------------------------------------------------
// VENDOR COMMAND PACKET STRUCTURES
// ----------------------------------------------------------------------------

typedef struct __attribute__((packed)) {
    uint8_t cmd;
    uint8_t channel;
    uint8_t band;
    uint8_t reserved;
    uint8_t data[60];
} VendorCmdPacket;

typedef struct __attribute__((packed)) {
    uint8_t cmd;
    uint8_t result;
    uint8_t reserved[2];
    union {
        struct __attribute__((packed)) {
            uint16_t peaks[NUM_CHANNELS];
            uint8_t cpu0_load;
            uint8_t cpu1_load;
        } status;
        EqParamPacket eq_param;
        float preamp_db;
        float delay_ms;
        uint8_t bypass;
        uint8_t raw[60];
    };
} VendorRespPacket;

// Buffer Statistics Wire Format — fits in a single 64-byte control transfer
typedef struct __attribute__((packed)) {
    uint8_t consumer_free;       // [0-4] SPDIF buffers available for DMA
    uint8_t consumer_prepared;   // [0-4] SPDIF buffers queued for DMA playback
    uint8_t consumer_playing;    // [0-1] DMA in-flight
    uint8_t consumer_fill_pct;   // Consumer pipeline fill %
    uint8_t consumer_min_fill_pct; // Lowest consumer fill since last reset
    uint8_t consumer_max_fill_pct; // Highest consumer fill since last reset
    uint8_t pad[2];
} SpdifBufferStats;              // 8 bytes

typedef struct __attribute__((packed)) {
    uint8_t dma_fill_pct;        // DMA circular buffer fill %
    uint8_t dma_min_fill_pct;
    uint8_t dma_max_fill_pct;
    uint8_t ring_fill_pct;       // Software ring buffer fill %
    uint8_t ring_min_fill_pct;
    uint8_t ring_max_fill_pct;
    uint8_t pad[2];
} PdmBufferStats;                // 8 bytes

typedef struct __attribute__((packed)) {
    uint8_t num_spdif;           // NUM_SPDIF_INSTANCES (2 or 4)
    uint8_t flags;               // Bit 0: PDM active, Bit 1: audio streaming
    uint16_t sequence;           // Monotonic counter, wraps at 65535
    SpdifBufferStats spdif[4];   // Per-instance (unused zeroed)
    PdmBufferStats pdm;
} BufferStatsPacket;             // 4 + 32 + 8 = 44 bytes

extern uint8_t channel_band_counts[NUM_CHANNELS];
extern volatile SystemStatusPacket global_status;

// ----------------------------------------------------------------------------
// RP2350-SPECIFIC: Force time-critical functions into RAM
// RP2350 has different XIP cache behavior that causes audio underruns
// ----------------------------------------------------------------------------
#define DSP_TIME_CRITICAL __attribute__((section(".time_critical")))

// ----------------------------------------------------------------------------
// UTILS
// ----------------------------------------------------------------------------

#if !PICO_RP2350
static inline int32_t clip_s32(int32_t x) {
    if (x > INT32_MAX) return INT32_MAX;
    if (x < INT32_MIN) return INT32_MIN;
    return x;
}

static inline int32_t clip_s64_to_s32(int64_t x) {
    if (x > INT32_MAX) return INT32_MAX;
    if (x < INT32_MIN) return INT32_MIN;
    return (int32_t)x;
}

static inline int32_t clip_s24(int32_t x) {
    if (x > 0x7FFFFF) return 0x7FFFFF;
    if (x < -0x800000) return -0x800000;
    return x;
}

// Q15 fixed-point multiply using 16-bit partial products.
// Computes (sample * gain) >> 15 without 64-bit library call.
// Exact when the final result fits in int32 (always true for valid audio).
static inline int32_t fast_mul_q15(int32_t sample, int32_t gain) {
    int32_t sh = sample >> 16;
    uint32_t sl = (uint16_t)sample;
    int32_t gh = gain >> 16;
    uint32_t gl = (uint16_t)gain;

    int32_t hh = sh * gh;
    int32_t mid = sh * (int32_t)gl + (int32_t)sl * gh;
    uint32_t ll = sl * gl;

    return (int32_t)(((uint32_t)hh << 17) + ((uint32_t)mid << 1) + (ll >> 15));
}
#endif

#endif // CONFIG_H
