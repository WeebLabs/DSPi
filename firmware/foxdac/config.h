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

// ----------------------------------------------------------------------------
// CONFIGURATION
// ----------------------------------------------------------------------------

#define ENABLE_SUB 1

// S/PDIF Output Pins (4 outputs on PIO0)
#undef PICO_AUDIO_SPDIF_PIN
#define PICO_AUDIO_SPDIF_PIN   6    // S/PDIF 1 (Out 1-2)
#define PICO_SPDIF_PIN_2       7    // S/PDIF 2 (Out 3-4)
#define PICO_SPDIF_PIN_3       8    // S/PDIF 3 (Out 5-6)
#define PICO_SPDIF_PIN_4       9    // S/PDIF 4 (Out 7-8)

// PDM Subwoofer Output Pin (PIO1)
#define PICO_PDM_PIN           10   // PDM sub (Out 9)

// Legacy aliases
#define PICO_AUDIO_SPDIF_SUB_PIN PICO_PDM_PIN

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

// SPDIF Buffer Configuration
#define AUDIO_BUFFER_COUNT    8   // Reduced from 8, compromise for lower latency
#define AUDIO_BUFFER_SAMPLES  192

// DELAY CONFIGURATION
// RP2350: 170ms max delay (8192 samples at 48kHz)
// RP2040: 85ms max delay (4096 samples) to fit 9 channels in RAM
#if PICO_RP2350
#define MAX_DELAY_SAMPLES 8192
#else
#define MAX_DELAY_SAMPLES 4096
#endif
#define MAX_DELAY_MASK    (MAX_DELAY_SAMPLES - 1)

// Latency alignment (in samples - automatically adapts to sample rate)
// SPDIF path: watermark = AUDIO_BUFFER_COUNT/2 buffers
// PDM path: DMA buffer = PDM_DMA_BUFFER_SIZE/8 PCM samples
#define SPDIF_BUFFER_SAMPLES  ((AUDIO_BUFFER_COUNT / 2) * AUDIO_BUFFER_SAMPLES)  // 384
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

// USB Audio Feature Unit IDs
#define FEATURE_MUTE_CONTROL 1u
#define FEATURE_VOLUME_CONTROL 2u
#define ENDPOINT_FREQ_CONTROL 1u

// Channel Definitions (11 total: 2 master + 9 outputs)
#define CH_MASTER_LEFT   0
#define CH_MASTER_RIGHT  1
#define CH_OUT_1         2   // S/PDIF 1 L (GPIO 20)
#define CH_OUT_2         3   // S/PDIF 1 R (GPIO 20)
#define CH_OUT_3         4   // S/PDIF 2 L (GPIO 21)
#define CH_OUT_4         5   // S/PDIF 2 R (GPIO 21)
#define CH_OUT_5         6   // S/PDIF 3 L (GPIO 22)
#define CH_OUT_6         7   // S/PDIF 3 R (GPIO 22)
#define CH_OUT_7         8   // S/PDIF 4 L (GPIO 23)
#define CH_OUT_8         9   // S/PDIF 4 R (GPIO 23)
#define CH_OUT_9_PDM     10  // PDM sub (GPIO 10)
#define NUM_CHANNELS     11
#define MAX_BANDS        12

// Legacy aliases for backward compatibility
#define CH_OUT_LEFT      CH_OUT_1
#define CH_OUT_RIGHT     CH_OUT_2
#define CH_OUT_SUB       CH_OUT_9_PDM

// Matrix Mixer Configuration
#define NUM_INPUT_CHANNELS   2   // USB L/R (expandable to 4 for S/PDIF input)
#define NUM_OUTPUT_CHANNELS  9   // Out 1-8 S/PDIF + Out 9 PDM

// Core 1 Operating Mode
typedef enum {
    CORE1_MODE_IDLE      = 0,
    CORE1_MODE_PDM       = 1,
    CORE1_MODE_EQ_WORKER = 2,
} Core1Mode;

// Outputs assigned to Core 1 EQ worker (S/PDIF pairs 2-4 = outputs 2-7)
#define CORE1_EQ_FIRST_OUTPUT  2
#define CORE1_EQ_LAST_OUTPUT   7

// Core 1 EQ Worker Handshake
typedef struct {
    volatile bool     work_ready;
    volatile bool     work_done;
    float           (*buf_out)[192];   // Pointer to buf_out array, set once at init
    uint32_t          sample_count;
    float             vol_mul;
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
    float b0, b1, b2, a1, a2;    // float coefficients
    double s1, s2;               // double state accumulators (Using inline DCP)
    bool bypass;                 // skip processing if true
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
#endif

#endif // CONFIG_H
