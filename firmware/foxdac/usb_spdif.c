/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <limits.h>

#include "pico/stdlib.h"
#include "pico/usb_device.h"
#include "pico/audio.h"
#include "pico/audio_spdif.h"
#include "pico/multicore.h"
#include "pico/unique_id.h"
#include "hardware/sync.h"
#include "hardware/irq.h"
#include "hardware/dma.h" 
#include "hardware/structs/dma.h" 
#include "hardware/structs/timer.h" 
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/watchdog.h"
#include "hardware/vreg.h" 
#include "lufa/AudioClassCommon.h"

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
#define PDM_DMA_BUFFER_SIZE 1024     
#define PDM_PIO pio1                 
#define PDM_SM 0

// PDM Stability Threshold (0.8 * 32768)
#define PDM_CLIP_THRESH 26214       

// DELAY CONFIGURATION
#define MAX_DELAY_SAMPLES 8192       // ~170ms @ 48kHz
#define MAX_DELAY_MASK    (MAX_DELAY_SAMPLES - 1)
#define SUB_ALIGN_MS      3.83f      // Fixed offset to align PDM with S/PDIF

// Standard USB Audio Request Codes
#define AUDIO_REQ_GetMin 0x82
#define AUDIO_REQ_GetMax 0x83
#define AUDIO_REQ_GetRes 0x84

// Vendor Requests
#define REQ_SET_EQ_PARAM 0x42
#define REQ_GET_EQ_PARAM 0x43
#define REQ_SET_PREAMP   0x44 
#define REQ_GET_PREAMP   0x45
#define REQ_SET_BYPASS   0x46
#define REQ_GET_BYPASS   0x47
#define REQ_SET_DELAY    0x48        
#define REQ_GET_DELAY    0x49        
#define REQ_GET_STATUS   0x50 

// Channel Definitions
#define CH_MASTER_LEFT  0
#define CH_MASTER_RIGHT 1
#define CH_OUT_LEFT     2
#define CH_OUT_RIGHT    3
#define CH_OUT_SUB      4
#define NUM_CHANNELS    5

#define MAX_BANDS       12

const uint8_t channel_band_counts[NUM_CHANNELS] = {
    10, 10, 2, 2, 2
};

enum FilterType { 
    FILTER_FLAT = 0, FILTER_PEAKING = 1, FILTER_LOWSHELF = 2, 
    FILTER_HIGHSHELF = 3, FILTER_LOWPASS = 4, FILTER_HIGHPASS = 5
};

// ----------------------------------------------------------------------------
// DATA STRUCTURES
// ----------------------------------------------------------------------------

typedef struct {
    int32_t b0, b1, b2, a1, a2;
    int32_t s1, s2; 
} Biquad;

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

Biquad filters[NUM_CHANNELS][MAX_BANDS];
EqParamPacket filter_recipes[NUM_CHANNELS][MAX_BANDS];
volatile SystemStatusPacket global_status = {0};

volatile bool eq_update_pending = false;
volatile EqParamPacket pending_packet;
volatile bool rate_change_pending = false;
volatile uint32_t pending_rate = 48000;

static volatile float global_preamp_db = 0.0f;
static volatile int32_t global_preamp_mul = 268435456; 
volatile bool bypass_master_eq = false;

volatile uint32_t pio_samples_dma = 0; 
volatile uint32_t pio_prev_samples_dma = 0;
int overruns = 0;

// Sync Variables
volatile uint64_t total_samples_produced = 0;
volatile uint64_t start_time_us = 0;
volatile bool sync_started = false;

// Delay Line State
int32_t delay_lines[3][MAX_DELAY_SAMPLES];
uint32_t delay_write_idx = 0;
float channel_delays_ms[NUM_CHANNELS] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
int32_t channel_delay_samples[3] = {0, 0, 0}; 

extern void usb_start_control_in_transfer(const struct usb_transfer_type *type);

// ----------------------------------------------------------------------------
// LOCK-FREE RING BUFFER
// ----------------------------------------------------------------------------
typedef struct {
    int32_t sample;
    bool reset;
} pdm_msg_t;

#define RING_SIZE 256
volatile pdm_msg_t pdm_ring[RING_SIZE];
volatile uint8_t pdm_head = 0; 
volatile uint8_t pdm_tail = 0; 

// ----------------------------------------------------------------------------
// UTILS
// ----------------------------------------------------------------------------

static uint32_t rng_state = 123456789;
static inline uint32_t fast_rand() {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return rng_state;
}

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

static inline int32_t fast_mul_q28(int32_t a, int32_t b) {
    int32_t ah = a >> 16;
    uint32_t al = a & 0xFFFF;
    int32_t bh = b >> 16;
    uint32_t bl = b & 0xFFFF;

    int32_t high = ah * bh; 
    int32_t mid1 = ah * bl; 
    int32_t mid2 = al * bh; 
    
    return (high << 4) + ((mid1 + mid2) >> 12);
}

// ----------------------------------------------------------------------------
// USB DESCRIPTORS
// ----------------------------------------------------------------------------

static char descriptor_str_vendor[] = "H3 & astanoev.com";
static char descriptor_str_product[] = "Pico DSP 2.1 (Sync Correct)";
static char descriptor_str_serial[17] = "0123456789ABCDEF"; 

static char *descriptor_strings[] = { descriptor_str_vendor, descriptor_str_product, descriptor_str_serial };

#define VENDOR_ID   0x2e8au
#define PRODUCT_ID  0xfeddu
#define AUDIO_OUT_ENDPOINT  0x01U
#define AUDIO_IN_ENDPOINT   0x82U

#undef AUDIO_SAMPLE_FREQ
#define AUDIO_SAMPLE_FREQ(frq) (uint8_t)(frq), (uint8_t)((frq >> 8)), (uint8_t)((frq >> 16))
#define FEATURE_MUTE_CONTROL 1u
#define FEATURE_VOLUME_CONTROL 2u
#define ENDPOINT_FREQ_CONTROL 1u

struct audio_device_config {
    struct usb_configuration_descriptor descriptor;
    struct usb_interface_descriptor ac_interface;
    struct __packed {
        USB_Audio_StdDescriptor_Interface_AC_t core;
        USB_Audio_StdDescriptor_InputTerminal_t input_terminal;
        USB_Audio_StdDescriptor_FeatureUnit_t feature_unit;
        USB_Audio_StdDescriptor_OutputTerminal_t output_terminal;
    } ac_audio;
    struct usb_interface_descriptor as_zero_interface;
    struct usb_interface_descriptor as_op_interface;
    struct __packed {
        USB_Audio_StdDescriptor_Interface_AS_t streaming;
        struct __packed {
            USB_Audio_StdDescriptor_Format_t core;
            USB_Audio_SampleFreq_t freqs[3];
        } format;
    } as_audio;
    struct __packed {
        struct usb_endpoint_descriptor_long core;
        USB_Audio_StdDescriptor_StreamEndpoint_Spc_t audio;
    } ep1;
    struct usb_endpoint_descriptor_long ep2;
};

static const struct audio_device_config audio_device_config = {
    .descriptor = {
        .bLength = sizeof(audio_device_config.descriptor), .bDescriptorType = DTYPE_Configuration,
        .wTotalLength = sizeof(audio_device_config), .bNumInterfaces = 2, .bConfigurationValue = 0x01,
        .iConfiguration = 0x00, .bmAttributes = 0x80, .bMaxPower = 0x32,
    },
    .ac_interface = {
        .bLength = sizeof(audio_device_config.ac_interface), .bDescriptorType = DTYPE_Interface,
        .bInterfaceNumber = 0x00, .bAlternateSetting = 0x00, .bNumEndpoints = 0x00,
        .bInterfaceClass = AUDIO_CSCP_AudioClass, .bInterfaceSubClass = AUDIO_CSCP_ControlSubclass,
        .bInterfaceProtocol = AUDIO_CSCP_ControlProtocol, .iInterface = 0x00,
    },
    .ac_audio = {
        .core = { .bLength = sizeof(audio_device_config.ac_audio.core), .bDescriptorType = AUDIO_DTYPE_CSInterface, .bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_Header, .bcdADC = VERSION_BCD(1, 0, 0), .wTotalLength = sizeof(audio_device_config.ac_audio), .bInCollection = 1, .bInterfaceNumbers = 1 },
        .input_terminal = { .bLength = sizeof(audio_device_config.ac_audio.input_terminal), .bDescriptorType = AUDIO_DTYPE_CSInterface, .bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_InputTerminal, .bTerminalID = 1, .wTerminalType = AUDIO_TERMINAL_STREAMING, .bAssocTerminal = 0, .bNrChannels = 2, .wChannelConfig = AUDIO_CHANNEL_LEFT_FRONT | AUDIO_CHANNEL_RIGHT_FRONT, .iChannelNames = 0, .iTerminal = 0 },
        .feature_unit = { .bLength = sizeof(audio_device_config.ac_audio.feature_unit), .bDescriptorType = AUDIO_DTYPE_CSInterface, .bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_Feature, .bUnitID = 2, .bSourceID = 1, .bControlSize = 1, .bmaControls = {AUDIO_FEATURE_MUTE | AUDIO_FEATURE_VOLUME, 0, 0}, .iFeature = 0 },
        .output_terminal = { .bLength = sizeof(audio_device_config.ac_audio.output_terminal), .bDescriptorType = AUDIO_DTYPE_CSInterface, .bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_OutputTerminal, .bTerminalID = 3, .wTerminalType = AUDIO_TERMINAL_OUT_SPEAKER, .bAssocTerminal = 0, .bSourceID = 2, .iTerminal = 0 },
    },
    .as_zero_interface = { .bLength = sizeof(audio_device_config.as_zero_interface), .bDescriptorType = DTYPE_Interface, .bInterfaceNumber = 0x01, .bAlternateSetting = 0x00, .bNumEndpoints = 0x00, .bInterfaceClass = AUDIO_CSCP_AudioClass, .bInterfaceSubClass = AUDIO_CSCP_AudioStreamingSubclass, .bInterfaceProtocol = AUDIO_CSCP_ControlProtocol, .iInterface = 0x00 },
    .as_op_interface = { .bLength = sizeof(audio_device_config.as_op_interface), .bDescriptorType = DTYPE_Interface, .bInterfaceNumber = 0x01, .bAlternateSetting = 0x01, .bNumEndpoints = 0x02, .bInterfaceClass = AUDIO_CSCP_AudioClass, .bInterfaceSubClass = AUDIO_CSCP_AudioStreamingSubclass, .bInterfaceProtocol = AUDIO_CSCP_ControlProtocol, .iInterface = 0x00 },
    .as_audio = {
        .streaming = { .bLength = sizeof(audio_device_config.as_audio.streaming), .bDescriptorType = AUDIO_DTYPE_CSInterface, .bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_General, .bTerminalLink = 1, .bDelay = 1, .wFormatTag = 1 },
        .format = {
            .core = { .bLength = sizeof(audio_device_config.as_audio.format), .bDescriptorType = AUDIO_DTYPE_CSInterface, .bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_FormatType, .bFormatType = 1, .bNrChannels = 2, .bSubFrameSize = 2, .bBitResolution = 16, .bSampleFrequencyType = count_of(audio_device_config.as_audio.format.freqs) },
            .freqs = { AUDIO_SAMPLE_FREQ(44100), AUDIO_SAMPLE_FREQ(48000), AUDIO_SAMPLE_FREQ(96000) },
        },
    },
    .ep1 = {
        .core = { .bLength = sizeof(audio_device_config.ep1.core), .bDescriptorType = DTYPE_Endpoint, .bEndpointAddress = AUDIO_OUT_ENDPOINT, .bmAttributes = 5, .wMaxPacketSize = 384, .bInterval = 1, .bRefresh = 0, .bSyncAddr = AUDIO_IN_ENDPOINT },
        .audio = { .bLength = sizeof(audio_device_config.ep1.audio), .bDescriptorType = AUDIO_DTYPE_CSEndpoint, .bDescriptorSubtype = AUDIO_DSUBTYPE_CSEndpoint_General, .bmAttributes = 1, .bLockDelayUnits = 0, .wLockDelay = 0 }
    },
    .ep2 = { .bLength = sizeof(audio_device_config.ep2), .bDescriptorType = 0x05, .bEndpointAddress = AUDIO_IN_ENDPOINT, .bmAttributes = 0x11, .wMaxPacketSize = 3, .bInterval = 0x01, .bRefresh = 2, .bSyncAddr = 0 },
};

static struct usb_interface ac_interface;
static struct usb_interface as_op_interface;
static struct usb_endpoint ep_op_out, ep_op_sync;

static const struct usb_device_descriptor boot_device_descriptor = {
    .bLength = 18, .bDescriptorType = 0x01, .bcdUSB = 0x0110, .bDeviceClass = 0x00, .bDeviceSubClass = 0x00, .bDeviceProtocol = 0x00, .bMaxPacketSize0 = 0x40, .idVendor = VENDOR_ID, .idProduct = PRODUCT_ID, .bcdDevice = 0x0200, .iManufacturer = 0x01, .iProduct = 0x02, .iSerialNumber = 0x03, .bNumConfigurations = 0x01,
};

const char *_get_descriptor_string(uint index) {
    if (index <= count_of(descriptor_strings)) return descriptor_strings[index - 1]; else return "";
}

static struct {
    uint32_t freq;
    int16_t volume;
    int16_t vol_mul;
    bool mute;
} audio_state = { .freq = 44100 };

static volatile uint8_t clock_176mhz = 0;
#define AUDIO_BUFFER_COUNT 8

// --- AUDIO POOLS ---
static struct audio_buffer_pool *producer_pool = NULL; // Main (Stereo S/PDIF)

// --- PDM STATE ---
// DMA Circular Buffer
uint32_t __attribute__((aligned(PDM_DMA_BUFFER_SIZE * 4))) pdm_dma_buffer[PDM_DMA_BUFFER_SIZE];
int pdm_dma_chan = -1;

volatile uint8_t usb_host_seen = 0;
static volatile uint32_t rate = 48000;

// Format Definitions
struct audio_format audio_format_48k = { .format = AUDIO_BUFFER_FORMAT_PCM_S16, .sample_freq = 48000, .channel_count = 2 };

// S/PDIF Config
struct audio_spdif_config config = { .pin = PICO_AUDIO_SPDIF_PIN, .dma_channel = 0, .pio_sm = 0 };
struct audio_buffer_format producer_format = { .format = &audio_format_48k, .sample_stride = 4 };

// ----------------------------------------------------------------------------
// VOLUME TABLES
// ----------------------------------------------------------------------------
uint16_t db_to_vol[91] = {
        0x0001, 0x0001, 0x0001, 0x0001, 0x0001, 0x0001, 0x0002, 0x0002, 0x0002, 0x0002, 0x0003, 0x0003, 0x0004, 0x0004, 0x0005, 0x0005,
        0x0006, 0x0007, 0x0008, 0x0009, 0x000a, 0x000b, 0x000d, 0x000e, 0x0010, 0x0012, 0x0014, 0x0017, 0x001a, 0x001d, 0x0020, 0x0024,
        0x0029, 0x002e, 0x0033, 0x003a, 0x0041, 0x0049, 0x0052, 0x005c, 0x0067, 0x0074, 0x0082, 0x0092, 0x00a4, 0x00b8, 0x00ce, 0x00e7,
        0x0104, 0x0124, 0x0147, 0x016f, 0x019c, 0x01ce, 0x0207, 0x0246, 0x028d, 0x02dd, 0x0337, 0x039b, 0x040c, 0x048a, 0x0518, 0x05b7,
        0x066a, 0x0732, 0x0813, 0x090f, 0x0a2a, 0x0b68, 0x0ccc, 0x0e5c, 0x101d, 0x1214, 0x1449, 0x16c3, 0x198a, 0x1ca7, 0x2026, 0x2413,
        0x287a, 0x2d6a, 0x32f5, 0x392c, 0x4026, 0x47fa, 0x50c3, 0x5a9d, 0x65ac, 0x7214, 0x7fff
};
#define CENTER_VOLUME_INDEX 91
#define ENCODE_DB(x) ((uint16_t)(int16_t)((x)*256))
#define MIN_VOLUME           ENCODE_DB(-CENTER_VOLUME_INDEX)
#define DEFAULT_VOLUME       ENCODE_DB(0)
#define MAX_VOLUME           ENCODE_DB(count_of(db_to_vol)-CENTER_VOLUME_INDEX)
#define VOLUME_RESOLUTION    ENCODE_DB(1)

// ----------------------------------------------------------------------------
// RAW PIO PROGRAM FOR PDM
// ----------------------------------------------------------------------------
static const uint16_t pio_pdm_instr[] = { 0x6001 }; // 0: out pins, 1
static const struct pio_program pio_pdm_program = { .instructions = pio_pdm_instr, .length = 1, .origin = -1 };

// ----------------------------------------------------------------------------
// CORE 1: PDM GENERATOR
// ----------------------------------------------------------------------------
void core1_pdm_entry() {
    int32_t local_pdm_err = 0;
    int32_t local_pdm_err2 = 0;
    uint32_t local_pdm_write = 0;
    uint32_t active_us_accumulator = 0;
    uint32_t sample_counter = 0;

    while (1) {
        while (pdm_head == pdm_tail) {
            __wfe(); 
        }
        
        pdm_msg_t msg = pdm_ring[pdm_tail];
        pdm_tail++; // Auto-wrap

        uint32_t start_time = timer_hw->timerawl;

        uint32_t read_addr = dma_hw->ch[pdm_dma_chan].read_addr;
        uint32_t current_read_idx = (read_addr - (uint32_t)pdm_dma_buffer) / 4;
        int32_t delta = (local_pdm_write - current_read_idx) & (PDM_DMA_BUFFER_SIZE - 1);
        if (delta > (PDM_DMA_BUFFER_SIZE / 2)) {
            local_pdm_write = (current_read_idx + 64) & (PDM_DMA_BUFFER_SIZE - 1);
        }

        if (msg.reset) {
            local_pdm_err = 0;
            local_pdm_err2 = 0;
            msg.sample = 0;
        }

        // TPDF Dither
        int32_t r1 = (int32_t)(fast_rand() & 2047);
        int32_t r2 = (int32_t)(fast_rand() & 2047);
        int32_t dither = r1 - r2;
        int32_t dithered_sample = msg.sample + dither;

        // Input Hard Limiter
        int32_t pcm_val = (dithered_sample >> 14); 
        if (pcm_val > PDM_CLIP_THRESH) pcm_val = PDM_CLIP_THRESH;
        if (pcm_val < -PDM_CLIP_THRESH) pcm_val = -PDM_CLIP_THRESH;

        int32_t target = pcm_val + 32768; 
        
        // 256x Oversampling
        for (int chunk = 0; chunk < 8; chunk++) {
            uint32_t pdm_word = 0;
            for (int k = 0; k < 32; k++) {
                int32_t fb_val = (local_pdm_err2 >= 0) ? 65535 : 0;
                if (local_pdm_err2 >= 0) pdm_word |= (1u << (31 - k));
                
                local_pdm_err += (target - fb_val);
                local_pdm_err2 += (local_pdm_err - fb_val);
            }
            pdm_dma_buffer[local_pdm_write] = pdm_word;
            local_pdm_write = (local_pdm_write + 1) & (PDM_DMA_BUFFER_SIZE - 1);
        }

        uint32_t end_time = timer_hw->timerawl;
        active_us_accumulator += (end_time - start_time);
        sample_counter++;

        if (sample_counter >= 48) {
            global_status.cpu1_load = (uint8_t)(active_us_accumulator / 10);
            active_us_accumulator = 0;
            sample_counter = 0;
        }
    }
}

// ----------------------------------------------------------------------------
// DSP LOGIC
// ----------------------------------------------------------------------------

static void compute_coefficients(EqParamPacket *p, Biquad *bq, float sample_rate) {
    bq->s1 = 0; bq->s2 = 0; 
    if (p->type == FILTER_FLAT || p->freq == 0 || sample_rate == 0) {
        bq->b0 = 1 << FILTER_SHIFT; bq->b1 = 0; bq->b2 = 0; bq->a1 = 0; bq->a2 = 0;
        return;
    }
    float omega = 2.0f * 3.1415926535f * p->freq / sample_rate;
    float sn = sinf(omega); float cs = cosf(omega);
    float alpha = sn / (2.0f * p->Q);
    float A = powf(10.0f, p->gain_db / 40.0f);
    float a0_f = 1.0f, a1_f = 0.0f, a2_f = 0.0f, b0_f = 1.0f, b1_f = 0.0f, b2_f = 0.0f;
    switch (p->type) {
        case FILTER_LOWPASS: b0_f = (1-cs)/2; b1_f = 1-cs; b2_f = (1-cs)/2; a0_f = 1+alpha; a1_f = -2*cs; a2_f = 1-alpha; break;
        case FILTER_HIGHPASS: b0_f = (1+cs)/2; b1_f = -(1+cs); b2_f = (1+cs)/2; a0_f = 1+alpha; a1_f = -2*cs; a2_f = 1-alpha; break;
        case FILTER_PEAKING: b0_f = 1+alpha*A; b1_f = -2*cs; b2_f = 1-alpha*A; a0_f = 1+alpha/A; a1_f = -2*cs; a2_f = 1-alpha/A; break;
        case FILTER_LOWSHELF: b0_f = A*((A+1)-(A-1)*cs+2*sqrtf(A)*alpha); b1_f = 2*A*((A-1)-(A+1)*cs); b2_f = A*((A+1)-(A-1)*cs-2*sqrtf(A)*alpha); a0_f = (A+1)+(A-1)*cs+2*sqrtf(A)*alpha; a1_f = -2*((A-1)+(A+1)*cs); a2_f = (A+1)+(A-1)*cs-2*sqrtf(A)*alpha; break;
        case FILTER_HIGHSHELF: b0_f = A*((A+1)+(A-1)*cs+2*sqrtf(A)*alpha); b1_f = -2*A*((A-1)+(A+1)*cs); b2_f = A*((A+1)+(A-1)*cs-2*sqrtf(A)*alpha); a0_f = (A+1)-(A-1)*cs+2*sqrtf(A)*alpha; a1_f = 2*((A-1)-(A+1)*cs); a2_f = (A+1)-(A-1)*cs-2*sqrtf(A)*alpha; break;
        default: break;
    }
    float scale = (float)(1LL << FILTER_SHIFT);
    bq->b0 = (int32_t)((b0_f / a0_f) * scale);
    bq->b1 = (int32_t)((b1_f / a0_f) * scale);
    bq->b2 = (int32_t)((b2_f / a0_f) * scale);
    bq->a1 = (int32_t)((a1_f / a0_f) * scale);
    bq->a2 = (int32_t)((a2_f / a0_f) * scale);
}

void init_default_filters() {
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        for (int b = 0; b < MAX_BANDS; b++) {
             filter_recipes[ch][b].type = FILTER_FLAT; filter_recipes[ch][b].freq = 1000.0f; filter_recipes[ch][b].Q = 0.707f; filter_recipes[ch][b].gain_db = 0.0f;
        }
    }
    EqParamPacket hp = { .type = FILTER_HIGHPASS, .freq = 80.0f, .Q = 0.707f, .gain_db = 0.0f };
    filter_recipes[CH_OUT_LEFT][0] = hp; filter_recipes[CH_OUT_RIGHT][0] = hp;
    EqParamPacket lp = { .type = FILTER_LOWPASS, .freq = 80.0f, .Q = 0.707f, .gain_db = 0.0f };
    filter_recipes[CH_OUT_SUB][0] = lp;
}

// Helper to convert milliseconds to samples safely
void update_delay_samples() {
    float rate = (float)audio_state.freq;
    
    // Calculate Channel 2 (Out Left)
    int32_t samples_l = (int32_t)(channel_delays_ms[CH_OUT_LEFT] * rate / 1000.0f);
    if (samples_l > MAX_DELAY_SAMPLES) samples_l = MAX_DELAY_SAMPLES;
    channel_delay_samples[0] = samples_l;

    // Calculate Channel 3 (Out Right)
    int32_t samples_r = (int32_t)(channel_delays_ms[CH_OUT_RIGHT] * rate / 1000.0f);
    if (samples_r > MAX_DELAY_SAMPLES) samples_r = MAX_DELAY_SAMPLES;
    channel_delay_samples[1] = samples_r;

    // Calculate Channel 4 (Out Sub) + Auto-Alignment (3.83ms)
    float sub_total_ms = channel_delays_ms[CH_OUT_SUB] + SUB_ALIGN_MS;
    int32_t samples_sub = (int32_t)(sub_total_ms * rate / 1000.0f);
    if (samples_sub > MAX_DELAY_SAMPLES) samples_sub = MAX_DELAY_SAMPLES;
    channel_delay_samples[2] = samples_sub;
}

void recalculate_all_filters(float sample_rate) {
    update_delay_samples(); // Recalculate offsets when rate changes
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        for (int b = 0; b < channel_band_counts[ch]; b++) {
            compute_coefficients(&filter_recipes[ch][b], &filters[ch][b], sample_rate);
        }
    }
}

// Optimized Channel Processing using __restrict
static inline int32_t process_channel_32(Biquad * __restrict biquads, int32_t input_32, uint8_t channel) {
    int32_t sample = input_32;
    uint8_t count = channel_band_counts[channel];
    for (int i = 0; i < count; i++) {
        Biquad *bq = &biquads[i];
        if (bq->a1 == 0 && bq->a2 == 0) continue;

        int32_t result = fast_mul_q28(bq->b0, sample) + bq->s1;
        bq->s1 = fast_mul_q28(bq->b1, sample) - fast_mul_q28(bq->a1, result) + bq->s2;
        bq->s2 = fast_mul_q28(bq->b2, sample) - fast_mul_q28(bq->a2, result);
        
        sample = clip_s32(result);
    }
    return sample;
}

// ----------------------------------------------------------------------------
// AUDIO CALLBACK
// ----------------------------------------------------------------------------

void __not_in_flash_func(usb_sof_irq)(void) { return; }

static void __not_in_flash_func(_as_audio_packet)(struct usb_endpoint *ep) {
    uint32_t start_time = timer_hw->timerawl;

    struct usb_buffer *usb_buffer = usb_current_out_packet_buffer(ep);
    if (!usb_buffer) { usb_start_empty_control_in_transfer_null_completion(); return; }

    struct audio_buffer* audio_buffer = NULL; 
    if (producer_pool) audio_buffer = take_audio_buffer(producer_pool, false);
    if (audio_buffer) audio_buffer->sample_count = usb_buffer->data_len / 4;
    
    uint32_t sample_count = usb_buffer->data_len / 4; 
    
    if (!sync_started) {
        start_time_us = time_us_64();
        sync_started = true;
    }
    total_samples_produced += sample_count;

    int16_t *in = (int16_t *) usb_buffer->data;
    int32_t vol_mul = audio_state.vol_mul;
    int32_t preamp = global_preamp_mul; 
    bool is_silent = true;
    int32_t silence_threshold = 1 << 20; 
    bool is_bypassed = bypass_master_eq;

    int32_t peak_ml = 0, peak_mr = 0, peak_ol = 0, peak_or = 0, peak_sub = 0;

    for (uint32_t i = 0; i < sample_count; i++) {
        int16_t raw_left_16 = in[i*2];
        int16_t raw_right_16 = in[i*2+1];
        
        if (abs(raw_left_16) > (silence_threshold >> 14) || abs(raw_right_16) > (silence_threshold >> 14)) is_silent = false;
        
        int32_t raw_left_32 = (int32_t)raw_left_16 << 14;
        int32_t raw_right_32 = (int32_t)raw_right_16 << 14;

        raw_left_32  = clip_s64_to_s32(((int64_t)raw_left_32 * preamp) >> 28);
        raw_right_32 = clip_s64_to_s32(((int64_t)raw_right_32 * preamp) >> 28);

        int32_t master_l_32, master_r_32;
        if (is_bypassed) { 
            master_l_32 = raw_left_32; 
            master_r_32 = raw_right_32; 
        } else {
            if (audio_buffer) {
                master_l_32 = process_channel_32(filters[CH_MASTER_LEFT], raw_left_32, CH_MASTER_LEFT);
                master_r_32 = process_channel_32(filters[CH_MASTER_RIGHT], raw_right_32, CH_MASTER_RIGHT);
            } else { 
                master_l_32 = 0; master_r_32 = 0; 
            }
        }

        if (abs(master_l_32) > peak_ml) peak_ml = abs(master_l_32);
        if (abs(master_r_32) > peak_mr) peak_mr = abs(master_r_32);

        int32_t sub_in_32 = (master_l_32 + master_r_32) >> 1;
        int32_t out_l_32 = 0, out_r_32 = 0, out_sub_32 = 0;

        if (audio_buffer) {
            out_l_32 = process_channel_32(filters[CH_OUT_LEFT], master_l_32, CH_OUT_LEFT);
            out_r_32 = process_channel_32(filters[CH_OUT_RIGHT], master_r_32, CH_OUT_RIGHT);
        }
#if ENABLE_SUB
        out_sub_32 = process_channel_32(filters[CH_OUT_SUB], sub_in_32, CH_OUT_SUB);
#endif

        if (abs(out_l_32) > peak_ol) peak_ol = abs(out_l_32);
        if (abs(out_r_32) > peak_or) peak_or = abs(out_r_32);
        if (abs(out_sub_32) > peak_sub) peak_sub = abs(out_sub_32);

        out_l_32   = (int32_t)(((int64_t)out_l_32 * vol_mul) >> 15);
        out_r_32   = (int32_t)(((int64_t)out_r_32 * vol_mul) >> 15);
        out_sub_32 = (int32_t)(((int64_t)out_sub_32 * vol_mul) >> 15);
        
        out_l_32 = clip_s32(out_l_32);
        out_r_32 = clip_s32(out_r_32);
        out_sub_32 = clip_s32(out_sub_32);

        // DELAY IMPLEMENTATION
        // Store current samples in the circular buffer
        delay_lines[0][delay_write_idx] = out_l_32;
        delay_lines[1][delay_write_idx] = out_r_32;
        delay_lines[2][delay_write_idx] = out_sub_32;

        // Read back delayed samples using fast bitwise masking
        // (x & MAX_DELAY_MASK) is equivalent to (x % MAX_DELAY_SAMPLES)
        int32_t delayed_l   = delay_lines[0][(delay_write_idx - channel_delay_samples[0]) & MAX_DELAY_MASK];
        int32_t delayed_r   = delay_lines[1][(delay_write_idx - channel_delay_samples[1]) & MAX_DELAY_MASK];
        int32_t delayed_sub = delay_lines[2][(delay_write_idx - channel_delay_samples[2]) & MAX_DELAY_MASK];

        // Increment pointer
        delay_write_idx = (delay_write_idx + 1) & MAX_DELAY_MASK;

        if (audio_buffer) {
            int16_t *out = (int16_t *) audio_buffer->buffer->bytes;
            out[i*2]     = (int16_t)(clip_s32(delayed_l + (1<<13)) >> 14);
            out[i*2+1]   = (int16_t)(clip_s32(delayed_r + (1<<13)) >> 14);
        }

#if ENABLE_SUB
        uint8_t next_head = pdm_head + 1;
        if (next_head != pdm_tail) {
            pdm_msg_t msg; 
            msg.sample = delayed_sub; 
            msg.reset = is_silent;
            pdm_ring[pdm_head] = msg;
            pdm_head = next_head; 
            __sev();
        } else {
            overruns++;
        }
#endif
    }

    global_status.peaks[0] = (uint16_t)(peak_ml >> 13);
    global_status.peaks[1] = (uint16_t)(peak_mr >> 13);
    global_status.peaks[2] = (uint16_t)(peak_ol >> 13);
    global_status.peaks[3] = (uint16_t)(peak_or >> 13);
    global_status.peaks[4] = (uint16_t)(peak_sub >> 13);

    if (audio_buffer) give_audio_buffer(producer_pool, audio_buffer);
    
    usb_grow_transfer(ep->current_transfer, 1);
    usb_packet_done(ep);

    uint32_t end_time = timer_hw->timerawl;
    global_status.cpu0_load = (uint8_t)((end_time - start_time) / 10);
}

// ----------------------------------------------------------------------------
// CONTROL LOGIC
// ----------------------------------------------------------------------------
static void __not_in_flash_func(_as_sync_packet)(struct usb_endpoint *ep) {
    struct usb_buffer *buffer = usb_current_in_packet_buffer(ep);
    buffer->data_len = 3;

    // --- TIME-BASED DRIFT COMPENSATION ---
    // Safe, simple, no library hacking.
    uint32_t feedback;
    
    if (sync_started && total_samples_produced > 48000) {
        uint64_t now_us = time_us_64();
        uint64_t elapsed_us = now_us - start_time_us;
        
        // Calculate theoretical samples we SHOULD have played
        uint64_t expected_samples = (elapsed_us * audio_state.freq) / 1000000;
        
        // Difference: (Actual Received) - (Expected Played)
        int32_t drift = (int32_t)(total_samples_produced - expected_samples);
        
        // Nominal (10.14 fixed point format for USB 1.0)
        uint32_t nominal = (audio_state.freq << 14u) / 1000;
        
        // Correction Factor:
        // Gain: 50 seems stable for quick lock.
        int32_t correction = drift * 50; 
        
        // Dampen the correction to prevent oscillation
        if (correction > 5000) correction = 5000;
        if (correction < -5000) correction = -5000;
        
        feedback = nominal - correction;
    } else {
        // Startup default
        feedback = audio_state.freq << 14u;
    }

    buffer->data[0] = feedback; 
    buffer->data[1] = feedback >> 8u; 
    buffer->data[2] = feedback >> 16u;
    
    usb_grow_transfer(ep->current_transfer, 1); 
    usb_packet_done(ep);
}

static const struct usb_transfer_type as_transfer_type = { .on_packet = _as_audio_packet, .initial_packet_count = 1 };
static const struct usb_transfer_type as_sync_transfer_type = { .on_packet = _as_sync_packet, .initial_packet_count = 1 };
static struct usb_transfer as_transfer;
static struct usb_transfer as_sync_transfer;

static struct audio_control_cmd { uint8_t cmd; uint8_t type; uint8_t cs; uint8_t cn; uint8_t unit; uint8_t len; } audio_control_cmd_t;
static void _audio_reconfigure();
static void perform_rate_change(uint32_t new_freq);
static void audio_set_volume(int16_t volume) {
    audio_state.volume = volume; volume += CENTER_VOLUME_INDEX * 256; if (volume < 0) volume = 0; if (volume >= count_of(db_to_vol) * 256) volume = count_of(db_to_vol) * 256 - 1; audio_state.vol_mul = db_to_vol[((uint16_t)volume) >> 8u];
}
static void _audio_reconfigure() { rate_change_pending = true; pending_rate = audio_state.freq; rate = audio_state.freq; }
static void update_pdm_clock(uint32_t freq) { float div = (float)clock_get_hz(clk_sys) / (float)(freq * PDM_OVERSAMPLE); pio_sm_set_clkdiv(PDM_PIO, PDM_SM, div); }
static void perform_rate_change(uint32_t new_freq) {
    switch (new_freq) { case 44100: case 48000: case 96000: break; default: new_freq = 44100; }
    if((new_freq == 48000 || new_freq == 96000) && clock_176mhz) { set_sys_clock_pll(1440000000, 6, 1); clock_176mhz = 0; }
    else if(new_freq == 44100 && !clock_176mhz) { set_sys_clock_pll(1236000000, 7, 1); clock_176mhz = 1; }
    
    // Reset sync state on rate change
    sync_started = false;
    total_samples_produced = 0;
    
    recalculate_all_filters((float)new_freq); update_pdm_clock(new_freq);
}

// ISR Callback
static void audio_cmd_packet(struct usb_endpoint *ep) {
    struct usb_buffer *buffer = usb_current_out_packet_buffer(ep);
    if (!buffer) { usb_start_empty_control_in_transfer_null_completion(); return; }

    if (audio_control_cmd_t.cmd == REQ_SET_EQ_PARAM && buffer->data_len >= sizeof(EqParamPacket)) {
        memcpy((void*)&pending_packet, buffer->data, sizeof(EqParamPacket)); eq_update_pending = true;
    }
    if (audio_control_cmd_t.cmd == REQ_SET_PREAMP && buffer->data_len >= 4) {
        float db; memcpy(&db, buffer->data, 4); global_preamp_db = db;
        float linear = powf(10.0f, db / 20.0f); global_preamp_mul = (int32_t)(linear * (float)(1<<28));
    }
    
    // NEW: Set Delay (Channel Index in wValue, Delay MS in data)
    if (audio_control_cmd_t.cmd == REQ_SET_DELAY && buffer->data_len >= 4) {
        uint8_t ch = audio_control_cmd_t.cn; // Channel from Setup packet
        if (ch < NUM_CHANNELS) {
            float ms; memcpy(&ms, buffer->data, 4);
            if (ms < 0) ms = 0;
            channel_delays_ms[ch] = ms;
            update_delay_samples(); // Recalculate immediately
        }
    }

    if (audio_control_cmd_t.cmd == REQ_SET_BYPASS && buffer->data_len >= 1) { bypass_master_eq = (buffer->data[0] != 0); }
    if (audio_control_cmd_t.cmd == AUDIO_REQ_SetCurrent && buffer->data_len >= audio_control_cmd_t.len) {
        if (audio_control_cmd_t.type == USB_REQ_TYPE_RECIPIENT_INTERFACE) {
            switch (audio_control_cmd_t.cs) { case FEATURE_MUTE_CONTROL: audio_state.mute = buffer->data[0]; break; case FEATURE_VOLUME_CONTROL: { int16_t vol; memcpy(&vol, buffer->data, 2); audio_set_volume(vol); } break; }
        } else if (audio_control_cmd_t.type == USB_REQ_TYPE_RECIPIENT_ENDPOINT && audio_control_cmd_t.cs == ENDPOINT_FREQ_CONTROL) {
            uint32_t new_freq; memcpy(&new_freq, buffer->data, 4); new_freq &= 0x00ffffffu; if (audio_state.freq != new_freq) { audio_state.freq = new_freq; _audio_reconfigure(); }
        }
    }
    usb_start_empty_control_in_transfer_null_completion();
}

static const struct usb_transfer_type _audio_cmd_transfer_type = { .on_packet = audio_cmd_packet, .initial_packet_count = 1 };
static bool as_set_alternate(struct usb_interface *interface, uint alt) { return alt < 2; }
static bool do_set_current(struct usb_setup_packet *setup) {
    if (setup->wLength && setup->wLength < 64) {
        audio_control_cmd_t.cmd = AUDIO_REQ_SetCurrent; audio_control_cmd_t.type = setup->bmRequestType & USB_REQ_TYPE_RECIPIENT_MASK; audio_control_cmd_t.len = (uint8_t) setup->wLength; audio_control_cmd_t.unit = setup->wIndex >> 8u; audio_control_cmd_t.cs = setup->wValue >> 8u; audio_control_cmd_t.cn = (uint8_t) setup->wValue; usb_start_control_out_transfer(&_audio_cmd_transfer_type); return true;
    } return false;
}

// Get MIX/MAX/RES volume parameters for Windows (Necessary to init)
static bool do_get_min_max_res(struct usb_setup_packet *setup) {
    if ((setup->bmRequestType & USB_REQ_TYPE_RECIPIENT_MASK) == USB_REQ_TYPE_RECIPIENT_INTERFACE) {
        // Check if the request is for Volume Control
        if ((setup->wValue >> 8u) == FEATURE_VOLUME_CONTROL) {
            int16_t val;
            switch (setup->bRequest) {
                case AUDIO_REQ_GetMin: val = MIN_VOLUME; break;
                case AUDIO_REQ_GetMax: val = MAX_VOLUME; break;
                case AUDIO_REQ_GetRes: val = VOLUME_RESOLUTION; break;
                default: return false;
            }
            // Send the 2-byte volume limit
            usb_start_tiny_control_in_transfer((uint16_t)val, 2);
            return true;
        }
    }
    return false;
}

static bool do_get_current(struct usb_setup_packet *setup) {
    if ((setup->bmRequestType & USB_REQ_TYPE_RECIPIENT_MASK) == USB_REQ_TYPE_RECIPIENT_INTERFACE) {
        switch (setup->wValue >> 8u) { case FEATURE_MUTE_CONTROL: usb_start_tiny_control_in_transfer(audio_state.mute, 1); return true; case FEATURE_VOLUME_CONTROL: usb_start_tiny_control_in_transfer(audio_state.volume, 2); return true; }
    } else if ((setup->bmRequestType & USB_REQ_TYPE_RECIPIENT_MASK) == USB_REQ_TYPE_RECIPIENT_ENDPOINT) { if ((setup->wValue >> 8u) == ENDPOINT_FREQ_CONTROL) { usb_start_tiny_control_in_transfer(audio_state.freq, 3); return true; } } return false;
}
bool _as_setup_request_handler(__unused struct usb_endpoint *ep, struct usb_setup_packet *setup) {
    if (USB_REQ_TYPE_TYPE_CLASS == (setup->bmRequestType & USB_REQ_TYPE_TYPE_MASK)) { switch (setup->bRequest) { case AUDIO_REQ_SetCurrent: return do_set_current(setup); case AUDIO_REQ_GetCurrent: return do_get_current(setup); default: break; } } return false;
}

static bool ac_setup_request_handler(__unused struct usb_interface *interface, struct usb_setup_packet *setup) {
    if ((setup->bmRequestType & USB_REQ_TYPE_TYPE_MASK) == USB_REQ_TYPE_TYPE_VENDOR) {
        if (setup->bRequest == REQ_SET_EQ_PARAM) { audio_control_cmd_t.cmd = REQ_SET_EQ_PARAM; usb_start_control_out_transfer(&_audio_cmd_transfer_type); return true; }
        if (setup->bRequest == REQ_SET_PREAMP && setup->wLength == 4) { audio_control_cmd_t.cmd = REQ_SET_PREAMP; usb_start_control_out_transfer(&_audio_cmd_transfer_type); return true; }
        if (setup->bRequest == REQ_GET_PREAMP) { uint32_t val; float current_db = global_preamp_db; memcpy(&val, &current_db, 4); usb_start_tiny_control_in_transfer(val, 4); return true; }
        
        // NEW: Set Delay
        if (setup->bRequest == REQ_SET_DELAY && setup->wLength == 4) { 
            audio_control_cmd_t.cmd = REQ_SET_DELAY; 
            audio_control_cmd_t.cn = (uint8_t)setup->wValue; // Channel Index
            usb_start_control_out_transfer(&_audio_cmd_transfer_type); 
            return true; 
        }
        
        // NEW: Get Delay
        if (setup->bRequest == REQ_GET_DELAY) {
            uint8_t ch = (uint8_t)setup->wValue;
            if (ch < NUM_CHANNELS) {
                uint32_t val; memcpy(&val, &channel_delays_ms[ch], 4);
                usb_start_tiny_control_in_transfer(val, 4);
                return true;
            }
        }

        if (setup->bRequest == REQ_SET_BYPASS && setup->wLength == 1) { audio_control_cmd_t.cmd = REQ_SET_BYPASS; usb_start_control_out_transfer(&_audio_cmd_transfer_type); return true; }
        if (setup->bRequest == REQ_GET_BYPASS) { usb_start_tiny_control_in_transfer(bypass_master_eq ? 1 : 0, 1); return true; }
        
        if (setup->bRequest == REQ_GET_STATUS) {
            uint32_t resp = 0;
            if (setup->wValue == 0) { resp = (uint32_t)global_status.peaks[0] | ((uint32_t)global_status.peaks[1] << 16); }
            else if (setup->wValue == 1) { resp = (uint32_t)global_status.peaks[2] | ((uint32_t)global_status.peaks[3] << 16); }
            else if (setup->wValue == 2) { resp = (uint32_t)global_status.peaks[4] | ((uint32_t)global_status.cpu0_load << 16) | ((uint32_t)global_status.cpu1_load << 24); }
            usb_start_tiny_control_in_transfer(resp, 4);
            return true;
        }

        if (setup->bRequest == REQ_GET_EQ_PARAM) {
            uint8_t channel = (setup->wValue >> 8) & 0xFF; uint8_t band = (setup->wValue >> 4) & 0x0F; uint8_t param = setup->wValue & 0x0F;
            if (channel < NUM_CHANNELS && band < channel_band_counts[channel]) {
                uint32_t val_to_send = 0; EqParamPacket *p = &filter_recipes[channel][band];
                switch (param) { case 0: val_to_send = (uint32_t)p->type; break; case 1: memcpy(&val_to_send, &p->freq, 4); break; case 2: memcpy(&val_to_send, &p->Q, 4); break; case 3: memcpy(&val_to_send, &p->gain_db, 4); break; }
                usb_start_tiny_control_in_transfer(val_to_send, 4); return true;
            }
        }
    }
    if (USB_REQ_TYPE_TYPE_CLASS == (setup->bmRequestType & USB_REQ_TYPE_TYPE_MASK)) {
            switch (setup->bRequest) {
                case AUDIO_REQ_SetCurrent:
                    return do_set_current(setup);
                case AUDIO_REQ_GetCurrent:
                    return do_get_current(setup);
                
                // HANDLE WINDOWS MIN/MAX/RES REQUEST
                case AUDIO_REQ_GetMin:
                case AUDIO_REQ_GetMax:
                case AUDIO_REQ_GetRes:
                    return do_get_min_max_res(setup);
                default: break;
            }
        }
        return false;
    }

void usb_sound_card_init() {
    usb_interface_init(&ac_interface, &audio_device_config.ac_interface, NULL, 0, true); ac_interface.setup_request_handler = ac_setup_request_handler;
    static struct usb_endpoint *const op_endpoints[] = { &ep_op_out, &ep_op_sync };
    usb_interface_init(&as_op_interface, &audio_device_config.as_op_interface, op_endpoints, count_of(op_endpoints), true);
    as_op_interface.set_alternate_handler = as_set_alternate; ep_op_out.setup_request_handler = _as_setup_request_handler; 
    as_transfer.type = &as_transfer_type; usb_set_default_transfer(&ep_op_out, &as_transfer);
    as_sync_transfer.type = &as_sync_transfer_type; usb_set_default_transfer(&ep_op_sync, &as_sync_transfer);
    static struct usb_interface *const boot_device_interfaces[] = { &ac_interface, &as_op_interface };
    __unused struct usb_device *device = usb_device_init(&boot_device_descriptor, &audio_device_config.descriptor, boot_device_interfaces, count_of(boot_device_interfaces), _get_descriptor_string);
    init_default_filters(); recalculate_all_filters(48000.0f); audio_set_volume(DEFAULT_VOLUME); _audio_reconfigure(); usb_device_start();
}

void setup_pdm_hw() {
    uint offset = pio_add_program(PDM_PIO, &pio_pdm_program);
    pio_sm_config c = pio_get_default_sm_config(); 
    sm_config_set_wrap(&c, offset, offset + (pio_pdm_program.length - 1));
    sm_config_set_out_pins(&c, PICO_AUDIO_SPDIF_SUB_PIN, 1);
    sm_config_set_out_shift(&c, true, true, 32); sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    pio_gpio_init(PDM_PIO, PICO_AUDIO_SPDIF_SUB_PIN); pio_sm_set_consecutive_pindirs(PDM_PIO, PDM_SM, PICO_AUDIO_SPDIF_SUB_PIN, 1, true);
    pio_sm_init(PDM_PIO, PDM_SM, offset, &c); update_pdm_clock(48000); pio_sm_set_enabled(PDM_PIO, PDM_SM, true);
    pdm_dma_chan = dma_claim_unused_channel(true); dma_channel_config dmac = dma_channel_get_default_config(pdm_dma_chan);
    channel_config_set_transfer_data_size(&dmac, DMA_SIZE_32); channel_config_set_read_increment(&dmac, true); channel_config_set_write_increment(&dmac, false);
    channel_config_set_dreq(&dmac, pio_get_dreq(PDM_PIO, PDM_SM, true)); channel_config_set_ring(&dmac, false, 12); 
    dma_channel_configure(pdm_dma_chan, &dmac, &PDM_PIO->txf[PDM_SM], pdm_dma_buffer, 0xFFFFFFFF, true);
}

void core0_init() {
    vreg_set_voltage(VREG_VOLTAGE_1_15);
    sleep_ms(10);
    set_sys_clock_pll(1440000000, 6, 1);

    watchdog_enable(8000, 1); gpio_init(23); gpio_set_dir(23, GPIO_OUT); gpio_put(23, 1); gpio_init(25); gpio_set_dir(25, GPIO_OUT);
    pico_get_unique_board_id_string(descriptor_str_serial, 17); bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;
    producer_pool = audio_new_producer_pool(&producer_format, AUDIO_BUFFER_COUNT, 192); audio_spdif_setup(&audio_format_48k, &config);
    audio_spdif_connect_extra(producer_pool, false, AUDIO_BUFFER_COUNT / 2, NULL);
#if ENABLE_SUB
    setup_pdm_hw();
    multicore_launch_core1(core1_pdm_entry);
#endif
    usb_sound_card_init(); irq_set_priority(DMA_IRQ_0 + PICO_AUDIO_SPDIF_DMA_IRQ, PICO_HIGHEST_IRQ_PRIORITY); audio_spdif_set_enabled(true);
}

int main(void) {
    set_sys_clock_pll(1536000000, 4, 2); 
    core0_init();
    while (1) {
        watchdog_update(); 
        if (eq_update_pending) { 
            EqParamPacket p = pending_packet; 
            eq_update_pending = false; 
            filter_recipes[p.channel][p.band] = p; 
            
            uint32_t flags = save_and_disable_interrupts();
            compute_coefficients(&p, &filters[p.channel][p.band], (float)audio_state.freq); 
            restore_interrupts(flags);
        }
        if (rate_change_pending) { uint32_t r = pending_rate; rate_change_pending = false; perform_rate_change(r); }
        __wfe();
    }
}
