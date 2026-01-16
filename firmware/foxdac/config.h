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

// ----------------------------------------------------------------------------
// CONFIGURATION
// ----------------------------------------------------------------------------

#define ENABLE_SUB 1

#undef PICO_AUDIO_SPDIF_PIN
#define PICO_AUDIO_SPDIF_PIN 20

#define PICO_AUDIO_SPDIF_SUB_PIN 10

#define FILTER_SHIFT 28

// PDM Configuration
#define PDM_OVERSAMPLE 256
#define PDM_DMA_BUFFER_SIZE 2048  // Doubled for more margin (was 1024)
#define PDM_DMA_RING_BITS 13      // log2(2048 * 4 bytes) = 13
#define PDM_PIO pio1
#define PDM_SM 0
#define PDM_CLIP_THRESH 26214

// PDM Sigma-Delta Tuning
// Dither mask: controls TPDF amplitude. Start small (0x1FF), increase if idle tones persist
#define PDM_DITHER_MASK 0x1FF
// Leakage shift: higher = less leakage. Applied once per audio sample.
// 16 gives ~1.4s time constant at 48kHz, safe for bass
#define PDM_LEAKAGE_SHIFT 16

// DELAY CONFIGURATION
#define MAX_DELAY_SAMPLES 8192
#define MAX_DELAY_MASK    (MAX_DELAY_SAMPLES - 1)
#define SUB_ALIGN_MS      3.83f

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

// USB Audio Feature Unit IDs
#define FEATURE_MUTE_CONTROL 1u
#define FEATURE_VOLUME_CONTROL 2u
#define ENDPOINT_FREQ_CONTROL 1u

// Channel Definitions
#define CH_MASTER_LEFT  0
#define CH_MASTER_RIGHT 1
#define CH_OUT_LEFT     2
#define CH_OUT_RIGHT    3
#define CH_OUT_SUB      4
#define NUM_CHANNELS    5
#define MAX_BANDS       12

// ----------------------------------------------------------------------------
// DATA STRUCTURES
// ----------------------------------------------------------------------------

typedef struct {
    int32_t b0, b1, b2, a1, a2;
    int32_t s1, s2;
} Biquad;

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
    uint16_t peaks[5];
    uint8_t cpu0_load;
    uint8_t cpu1_load;
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
            uint16_t peaks[5];
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

extern const uint8_t channel_band_counts[NUM_CHANNELS];
extern volatile SystemStatusPacket global_status;

// ----------------------------------------------------------------------------
// UTILS
// ----------------------------------------------------------------------------

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

#endif // CONFIG_H
