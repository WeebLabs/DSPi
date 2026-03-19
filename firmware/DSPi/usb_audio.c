/*
 * USB Audio Implementation for DSPi
 * UAC1 Audio Streaming with DSP Pipeline
 * Uses pico-extras usb_device library with LUFA descriptor types
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/usb_device.h"
#include "pico/usb_device_private.h"
#include "pico/audio.h"
#include "pico/audio_spdif.h"
#include "hardware/sync.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/vreg.h"

#include "usb_audio.h"
#include "usb_descriptors.h"
#include "dsp_pipeline.h"
#include "dcp_inline.h"
#include "pdm_generator.h"
#include "flash_storage.h"
#include "loudness.h"
#include "crossfeed.h"
#include "bulk_params.h"
#include "pico/usb_stream_helper.h"

// ----------------------------------------------------------------------------
// GLOBALS
// ----------------------------------------------------------------------------

volatile AudioState audio_state = { .freq = 44100 };
volatile bool bypass_master_eq = false;
volatile SystemStatusPacket global_status = {0};

volatile bool eq_update_pending = false;
volatile EqParamPacket pending_packet;
volatile bool rate_change_pending = false;
volatile uint32_t pending_rate = 48000;
volatile bool bulk_params_pending = false;

// 4 KB aligned buffer shared between GET and SET bulk param transfers.
uint8_t __attribute__((aligned(4))) bulk_param_buf[WIRE_BULK_BUF_SIZE];

// Stream transfer state for multi-packet vendor control transfers.
static struct usb_stream_transfer _vendor_stream;
static struct usb_transfer _vendor_ack_transfer;

static struct usb_stream_transfer_funcs _vendor_stream_funcs = {
    .on_chunk = usb_stream_noop_on_chunk,
    .on_packet_complete = usb_stream_noop_on_packet_complete
};

// GET completion: data sent -> receive status-stage OUT ZLP from host
static void _vendor_get_complete(__unused struct usb_endpoint *ep,
                                 __unused struct usb_transfer *t) {
    usb_start_empty_transfer(usb_get_control_out_endpoint(), &_vendor_ack_transfer, NULL);
}

// SET status-stage ACK sent -> signal main loop to apply params
static void _vendor_set_ack_done(__unused struct usb_endpoint *ep,
                                 __unused struct usb_transfer *t) {
    bulk_params_pending = true;
}

// SET completion: data received -> send status-stage IN ZLP, then signal main loop
static void _vendor_set_complete(__unused struct usb_endpoint *ep,
                                 __unused struct usb_transfer *t) {
    usb_start_empty_transfer(usb_get_control_in_endpoint(), &_vendor_ack_transfer,
                             _vendor_set_ack_done);
}

volatile float global_preamp_db = 0.0f;
volatile int32_t global_preamp_mul = 268435456;
volatile float global_preamp_linear = 1.0f;

// Per-channel gain and mute (legacy 3-channel interface for flash compatibility)
volatile float channel_gain_db[3] = {0.0f, 0.0f, 0.0f};
volatile int32_t channel_gain_mul[3] = {32768, 32768, 32768};  // Unity = 2^15
volatile float channel_gain_linear[3] = {1.0f, 1.0f, 1.0f};
volatile bool channel_mute[3] = {false, false, false};

// Matrix Mixer State
MatrixMixer matrix_mixer = {0};

// Loudness compensation state
volatile bool loudness_enabled = false;
volatile float loudness_ref_spl = 83.0f;
volatile float loudness_intensity_pct = 100.0f;
volatile bool loudness_recompute_pending = false;

#if PICO_RP2350
static LoudnessSvfState loudness_state[2][LOUDNESS_BIQUAD_COUNT];  // [0]=Left, [1]=Right
#else
static Biquad loudness_biquads[2][LOUDNESS_BIQUAD_COUNT];  // [0]=Left, [1]=Right
#endif
static const LoudnessCoeffs *current_loudness_coeffs = NULL;

// Crossfeed state
volatile CrossfeedConfig crossfeed_config = {
    .enabled = false,
    .itd_enabled = true,
    .preset = CROSSFEED_PRESET_DEFAULT,
    .custom_fc = 700.0f,
    .custom_feed_db = 4.5f
};
volatile bool crossfeed_update_pending = false;
volatile bool crossfeed_bypassed = true;  // Fast bypass flag for audio callback
CrossfeedState crossfeed_state;

// Per-channel user-configurable names
char channel_names[NUM_CHANNELS][PRESET_NAME_LEN];

void get_default_channel_name(int ch, char *buf) {
    memset(buf, 0, PRESET_NAME_LEN);
#if PICO_RP2350
    static const char *defaults[] = {
        "USB L", "USB R",
        "SPDIF 1 L", "SPDIF 1 R", "SPDIF 2 L", "SPDIF 2 R",
        "SPDIF 3 L", "SPDIF 3 R", "SPDIF 4 L", "SPDIF 4 R",
        "PDM"
    };
#else
    static const char *defaults[] = {
        "USB L", "USB R",
        "SPDIF 1 L", "SPDIF 1 R", "SPDIF 2 L", "SPDIF 2 R",
        "PDM"
    };
#endif
    if (ch >= 0 && ch < NUM_CHANNELS) {
        strncpy(buf, defaults[ch], PRESET_NAME_LEN - 1);
    }
}

// Shared output buffer — file scope so Core 1 can access via pointer
#if PICO_RP2350
static float buf_out[NUM_OUTPUT_CHANNELS][192];
#else
static int32_t buf_out[NUM_OUTPUT_CHANNELS][192];
#endif

// Sync State
volatile uint64_t total_samples_produced = 0;
volatile uint64_t start_time_us = 0;
volatile bool sync_started = false;
static volatile uint64_t last_packet_time_us = 0;
static volatile uint8_t usb_input_bit_depth = 16;
#define AUDIO_GAP_THRESHOLD_US 50000  // 50ms - reset sync if packets stop this long

// Idle-time CPU load metering (Core 0)
static uint32_t cpu0_last_packet_end = 0;
static uint32_t cpu0_load_q8 = 0;         // EMA in Q8 fixed point (0-25600 = 0-100%)
static bool cpu0_load_primed = false;

// Fill-level servo: consumer fill for instance 0, read by usb_sof_irq()
volatile uint8_t spdif0_consumer_fill = 0;

// Buffer statistics watermark tracking
static void update_buffer_watermarks(void);
static void reset_buffer_watermarks(void);
static uint16_t buffer_stats_sequence = 0;
static uint8_t spdif_consumer_min_fill_pct[NUM_SPDIF_INSTANCES];
static uint8_t spdif_consumer_max_fill_pct[NUM_SPDIF_INSTANCES];
static uint8_t pdm_dma_min_fill_pct = 100;
static uint8_t pdm_dma_max_fill_pct = 0;
static uint8_t pdm_ring_min_fill_pct = 100;
static uint8_t pdm_ring_max_fill_pct = 0;

// Audio Pools (S/PDIF stereo pairs)
struct audio_buffer_pool *producer_pool_1 = NULL;  // S/PDIF 1 (Out 1-2)
struct audio_buffer_pool *producer_pool_2 = NULL;  // S/PDIF 2 (Out 3-4)
#if PICO_RP2350
struct audio_buffer_pool *producer_pool_3 = NULL;  // S/PDIF 3 (Out 5-6)
struct audio_buffer_pool *producer_pool_4 = NULL;  // S/PDIF 4 (Out 7-8)
#endif
struct audio_format audio_format_48k = { .format = AUDIO_BUFFER_FORMAT_PCM_S32, .sample_freq = 48000, .channel_count = 2 };

// Legacy aliases
#define producer_pool producer_pool_1
#define sub_producer_pool producer_pool_2

// ----------------------------------------------------------------------------
// USB INTERFACE / ENDPOINT OBJECTS
// ----------------------------------------------------------------------------

static struct usb_interface ac_interface;
static struct usb_interface as_op_interface;
static struct usb_interface vendor_interface;
static struct usb_endpoint ep_op_out, ep_op_sync;

// ----------------------------------------------------------------------------
// SYSTEM STATISTICS HELPERS
// ----------------------------------------------------------------------------

// Convert vreg voltage enum to millivolts
static uint16_t vreg_voltage_to_mv(enum vreg_voltage voltage) {
    // Voltage enum values map to specific voltages
    // See hardware/vreg.h for the full table
    static const uint16_t voltage_table[] = {
#if !PICO_RP2040
        550,  // VREG_VOLTAGE_0_55 = 0b00000
        600,  // VREG_VOLTAGE_0_60 = 0b00001
        650,  // VREG_VOLTAGE_0_65 = 0b00010
        700,  // VREG_VOLTAGE_0_70 = 0b00011
        750,  // VREG_VOLTAGE_0_75 = 0b00100
        800,  // VREG_VOLTAGE_0_80 = 0b00101
#endif
        850,  // VREG_VOLTAGE_0_85 = 0b00110
        900,  // VREG_VOLTAGE_0_90 = 0b00111
        950,  // VREG_VOLTAGE_0_95 = 0b01000
        1000, // VREG_VOLTAGE_1_00 = 0b01001
        1050, // VREG_VOLTAGE_1_05 = 0b01010
        1100, // VREG_VOLTAGE_1_10 = 0b01011
        1150, // VREG_VOLTAGE_1_15 = 0b01100
        1200, // VREG_VOLTAGE_1_20 = 0b01101
        1250, // VREG_VOLTAGE_1_25 = 0b01110
        1300, // VREG_VOLTAGE_1_30 = 0b01111
#if !PICO_RP2040
        1350, // VREG_VOLTAGE_1_35 = 0b10000
        1400, // VREG_VOLTAGE_1_40 = 0b10001
        1500, // VREG_VOLTAGE_1_50 = 0b10010
        1600, // VREG_VOLTAGE_1_60 = 0b10011
        1650, // VREG_VOLTAGE_1_65 = 0b10100
        1700, // VREG_VOLTAGE_1_70 = 0b10101
        1800, // VREG_VOLTAGE_1_80 = 0b10110
        1900, // VREG_VOLTAGE_1_90 = 0b10111
        2000, // VREG_VOLTAGE_2_00 = 0b11000
        2350, // VREG_VOLTAGE_2_35 = 0b11001
        2500, // VREG_VOLTAGE_2_50 = 0b11010
        2650, // VREG_VOLTAGE_2_65 = 0b11011
        2800, // VREG_VOLTAGE_2_80 = 0b11100
        3000, // VREG_VOLTAGE_3_00 = 0b11101
        3150, // VREG_VOLTAGE_3_15 = 0b11110
        3300, // VREG_VOLTAGE_3_30 = 0b11111
#endif
    };

#if PICO_RP2040
    // RP2040: offset by 6 entries (0.55-0.80V not available)
    uint8_t index = (voltage >= 6) ? (voltage - 6) : 0;
#else
    uint8_t index = voltage;
#endif

    if (index < sizeof(voltage_table) / sizeof(voltage_table[0])) {
        return voltage_table[index];
    }
    return 1100; // Default fallback
}

// Read ADC temperature sensor and return temperature in centi-degrees C
// Formula from SDK docs (same for RP2040/RP2350):
// T = 27 - (ADC_Voltage - 0.706) / 0.001721
static int16_t read_temperature_cdeg(void) {
    const float conversion_factor = 3.3f / 4095.0f;

    // Temperature sensor channel: auto-detects based on chip variant
    // RP2040, RP2350A (QFN-60): channel 4
    // RP2350B (QFN-80): channel 8
    adc_select_input(NUM_ADC_CHANNELS - 1);
    uint16_t adc_raw = adc_read();
    float voltage = adc_raw * conversion_factor;
    float temp_c = 27.0f - (voltage - 0.706f) / 0.001721f;

    return (int16_t)(temp_c * 100.0f); // Convert to centi-degrees
}

// ----------------------------------------------------------------------------
// VOLUME
// ----------------------------------------------------------------------------
#define CENTER_VOLUME_INDEX 60
static uint16_t db_to_vol[CENTER_VOLUME_INDEX + 1] = {
    // Index 0 = silent (slider bottom), index 1 = -59 dB, ..., index 60 = 0 dB
    0x0000, 0x0025, 0x0029, 0x002e, 0x0034, 0x003a, 0x0041, 0x0049,
    0x0052, 0x005c, 0x0068, 0x0074, 0x0082, 0x0092, 0x00a4, 0x00b8,
    0x00cf, 0x00e8, 0x0104, 0x0124, 0x0148, 0x0170, 0x019d, 0x01cf,
    0x0207, 0x0247, 0x028e, 0x02de, 0x0337, 0x039c, 0x040c, 0x048b,
    0x0519, 0x05b8, 0x066a, 0x0733, 0x0814, 0x0910, 0x0a2b, 0x0b68,
    0x0ccd, 0x0e5d, 0x101d, 0x1215, 0x1449, 0x16c3, 0x198a, 0x1ca8,
    0x2027, 0x2413, 0x287a, 0x2d6b, 0x32f5, 0x392d, 0x4027, 0x47fb,
    0x50c3, 0x5a9e, 0x65ad, 0x7215, 0x8000
};

#define ENCODE_DB(x) ((int16_t)((x)*256))
#define MIN_VOLUME           ENCODE_DB(-CENTER_VOLUME_INDEX)
#define DEFAULT_VOLUME       ENCODE_DB(0)
#define MAX_VOLUME           ENCODE_DB(0)
#define VOLUME_RESOLUTION    ENCODE_DB(1)

void audio_set_volume(int16_t volume) {
    audio_state.volume = volume;
    volume += CENTER_VOLUME_INDEX * 256;
    if (volume < 0) volume = 0;
    if (volume >= (CENTER_VOLUME_INDEX + 1) * 256) volume = (CENTER_VOLUME_INDEX + 1) * 256 - 1;
    uint8_t vol_index = ((uint16_t)volume) >> 8u;
    audio_state.vol_mul = db_to_vol[vol_index];

    // Update loudness compensation coefficients for this volume step
    if (loudness_enabled && loudness_active_table) {
        current_loudness_coeffs = loudness_active_table[vol_index];
    }
}

// ----------------------------------------------------------------------------
// AUDIO PROCESSING (called from USB audio packet callback)
// ----------------------------------------------------------------------------

static void __not_in_flash_func(process_audio_packet)(const uint8_t *data, uint16_t data_len) {
    uint32_t packet_start = time_us_32();

    // Detect SPDIF underrun: USB packets should arrive every ~1ms
    static uint32_t last_packet_time = 0;
    if (last_packet_time > 0 && !preset_loading) {
        uint32_t gap = packet_start - last_packet_time;
        if (gap > 2000 && gap < 50000) {  // 2ms-50ms gap = underrun
            spdif_underruns++;
        }
    }
    last_packet_time = packet_start;

    // Get audio buffers for S/PDIF outputs
#if PICO_RP2350
    struct audio_buffer* audio_buf[4] = {NULL, NULL, NULL, NULL};
    if (producer_pool_1) audio_buf[0] = take_audio_buffer(producer_pool_1, false);
    if (producer_pool_2) audio_buf[1] = take_audio_buffer(producer_pool_2, false);
    if (producer_pool_3) audio_buf[2] = take_audio_buffer(producer_pool_3, false);
    if (producer_pool_4) audio_buf[3] = take_audio_buffer(producer_pool_4, false);
#else
    struct audio_buffer* audio_buf[2] = {NULL, NULL};
    if (producer_pool_1) audio_buf[0] = take_audio_buffer(producer_pool_1, false);
    if (producer_pool_2) audio_buf[1] = take_audio_buffer(producer_pool_2, false);
#endif

    update_buffer_watermarks();

    const uint8_t bit_depth = usb_input_bit_depth;  // snapshot once — avoid double-read of volatile
    uint32_t bytes_per_frame = (bit_depth == 24) ? 6 : 4;
    uint32_t sample_count = data_len / bytes_per_frame;

    for (int b = 0; b < NUM_SPDIF_INSTANCES; b++) {
        if (audio_buf[b]) {
            audio_buf[b]->sample_count = sample_count;
        } else if (!preset_loading && (matrix_mixer.outputs[b*2].enabled || matrix_mixer.outputs[b*2+1].enabled)) {
            spdif_overruns++;
        }
    }

    uint64_t now_us = time_us_64();

    // Detect audio restart after gap - reset sync state and pre-fill pool
    if (sync_started && last_packet_time_us > 0 &&
        (now_us - last_packet_time_us) > AUDIO_GAP_THRESHOLD_US) {
        sync_started = false;
        total_samples_produced = 0;
        cpu0_load_primed = false;
        cpu0_load_q8 = 0;
    }
    last_packet_time_us = now_us;

    if (!sync_started) {
        start_time_us = now_us;
        sync_started = true;
    }
    total_samples_produced += sample_count;

#if PICO_RP2350
    // ------------------------------------------------------------------------
    // RP2350 FLOAT PIPELINE WITH MATRIX MIXER
    // ------------------------------------------------------------------------
    const float inv_32768 = 1.0f / 32768.0f;

    float vol_mul = audio_state.mute ? 0.0f : (float)audio_state.vol_mul * inv_32768;

    // Preset-switch mute: output silence while new parameters are being applied.
    // The counter is decremented each packet; audio resumes when it reaches zero.
    if (preset_loading) {
        vol_mul = 0.0f;
        if (preset_mute_counter > sample_count) {
            preset_mute_counter -= sample_count;
        } else {
            preset_mute_counter = 0;
            preset_loading = false;
        }
    }

    float preamp = global_preamp_linear;
    bool is_bypassed = bypass_master_eq;

    // Snapshot loudness state for this packet
    bool loud_on = loudness_enabled;
    const LoudnessCoeffs *loud_coeffs = current_loudness_coeffs;

    float peak_ml = 0, peak_mr = 0;

    // Pre-compute PDM scale factor
    const float pdm_scale = (float)(1 << 28);

    // Static buffers to avoid stack overflow (~8KB would be too much for stack)
    static float buf_l[192], buf_r[192];

    // ========== PASS 1: Input conversion + Preamp + Loudness ==========
    if (bit_depth == 24) {
        const uint8_t *p = (const uint8_t *)data;
        const float inv_8388608 = 1.0f / 8388608.0f;
        for (uint32_t i = 0; i < sample_count; i++) {
            int32_t left  = (int32_t)((uint32_t)p[2] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[0] << 8) >> 8;
            int32_t right = (int32_t)((uint32_t)p[5] << 24 | (uint32_t)p[4] << 16 | (uint32_t)p[3] << 8) >> 8;
            buf_l[i] = (float)left * inv_8388608 * preamp;
            buf_r[i] = (float)right * inv_8388608 * preamp;
            p += 6;
        }
    } else {
        const int16_t *in = (const int16_t *)data;
        for (uint32_t i = 0; i < sample_count; i++) {
            buf_l[i] = (float)in[i*2] * inv_32768 * preamp;
            buf_r[i] = (float)in[i*2+1] * inv_32768 * preamp;
        }
    }

    // Loudness compensation (SVF shelf filters)
    if (loud_on && loud_coeffs) {
        for (uint32_t i = 0; i < sample_count; i++) {
            float raw_left = buf_l[i];
            float raw_right = buf_r[i];
            for (int j = 0; j < LOUDNESS_BIQUAD_COUNT; j++) {
                const LoudnessCoeffs *lc = &loud_coeffs[j];
                if (lc->bypass) continue;
                LoudnessSvfState *st = &loudness_state[0][j];
                float v3 = raw_left - st->ic2eq;
                float v1 = lc->sva1 * st->ic1eq + lc->sva2 * v3;
                float v2 = st->ic2eq + lc->sva2 * st->ic1eq + lc->sva3 * v3;
                st->ic1eq = 2.0f * v1 - st->ic1eq;
                st->ic2eq = 2.0f * v2 - st->ic2eq;
                raw_left = lc->svm0 * raw_left + lc->svm1 * v1 + lc->svm2 * v2;
            }
            for (int j = 0; j < LOUDNESS_BIQUAD_COUNT; j++) {
                const LoudnessCoeffs *lc = &loud_coeffs[j];
                if (lc->bypass) continue;
                LoudnessSvfState *st = &loudness_state[1][j];
                float v3 = raw_right - st->ic2eq;
                float v1 = lc->sva1 * st->ic1eq + lc->sva2 * v3;
                float v2 = st->ic2eq + lc->sva2 * st->ic1eq + lc->sva3 * v3;
                st->ic1eq = 2.0f * v1 - st->ic1eq;
                st->ic2eq = 2.0f * v2 - st->ic2eq;
                raw_right = lc->svm0 * raw_right + lc->svm1 * v1 + lc->svm2 * v2;
            }
            buf_l[i] = raw_left;
            buf_r[i] = raw_right;
        }
    }

    // ========== PASS 2: Master EQ (Block-Based) ==========
    if (!is_bypassed) {
        if (!channel_bypassed[CH_MASTER_LEFT]) {
            dsp_process_channel_block(filters[CH_MASTER_LEFT], buf_l, sample_count, CH_MASTER_LEFT);
        }
        if (!channel_bypassed[CH_MASTER_RIGHT]) {
            dsp_process_channel_block(filters[CH_MASTER_RIGHT], buf_r, sample_count, CH_MASTER_RIGHT);
        }
    }

    // ========== PASS 3: Crossfeed + Master Peaks ==========
    bool do_crossfeed = !crossfeed_bypassed;

    // Crossfeed is sample-by-sample (internal state), combined with peak tracking
    for (uint32_t i = 0; i < sample_count; i++) {
        float ml = buf_l[i], mr = buf_r[i];
        float abs_ml = fabsf(ml); if (abs_ml > peak_ml) peak_ml = abs_ml;
        float abs_mr = fabsf(mr); if (abs_mr > peak_mr) peak_mr = abs_mr;
        if (do_crossfeed) {
            crossfeed_process_stereo(&crossfeed_state, &ml, &mr);
            buf_l[i] = ml; buf_r[i] = mr;
        }
    }

    // ========== PASS 4: Matrix Mixing (block-based, output-major) ==========
    // Snapshot crosspoint coefficients and process one output at a time
    for (int out = 0; out < NUM_OUTPUT_CHANNELS; out++) {
        if (!matrix_mixer.outputs[out].enabled) {
            memset(buf_out[out], 0, sample_count * sizeof(float));
            continue;
        }

        // Load crosspoint config once per output (not per sample)
        float gain_l = 0.0f, gain_r = 0.0f;
        MatrixCrosspoint *xp_l = &matrix_mixer.crosspoints[0][out];
        MatrixCrosspoint *xp_r = &matrix_mixer.crosspoints[1][out];
        if (xp_l->enabled) gain_l = xp_l->phase_invert ? -xp_l->gain_linear : xp_l->gain_linear;
        if (xp_r->enabled) gain_r = xp_r->phase_invert ? -xp_r->gain_linear : xp_r->gain_linear;

        float *dst = buf_out[out];
        if (gain_l != 0.0f && gain_r != 0.0f) {
            for (uint32_t i = 0; i < sample_count; i++)
                dst[i] = buf_l[i] * gain_l + buf_r[i] * gain_r;
        } else if (gain_l != 0.0f) {
            for (uint32_t i = 0; i < sample_count; i++)
                dst[i] = buf_l[i] * gain_l;
        } else if (gain_r != 0.0f) {
            for (uint32_t i = 0; i < sample_count; i++)
                dst[i] = buf_r[i] * gain_r;
        } else {
            memset(dst, 0, sample_count * sizeof(float));
        }
    }

    // ========== PASS 5-7: Per-Output EQ + Gain + Delay + Output ==========
    if (core1_mode == CORE1_MODE_EQ_WORKER) {
        // --- Dual-core path: Core 1 handles EQ+delay+SPDIF for outputs 2-7 ---

        // Dispatch to Core 1
        core1_eq_work.sample_count = sample_count;
        core1_eq_work.vol_mul = vol_mul;
        core1_eq_work.delay_write_idx = delay_write_idx;
        core1_eq_work.spdif_out[0] = audio_buf[1] ? (int32_t *)audio_buf[1]->buffer->bytes : NULL;
        core1_eq_work.spdif_out[1] = audio_buf[2] ? (int32_t *)audio_buf[2]->buffer->bytes : NULL;
        core1_eq_work.spdif_out[2] = audio_buf[3] ? (int32_t *)audio_buf[3]->buffer->bytes : NULL;
        core1_eq_work.work_done = false;
        __dmb();
        core1_eq_work.work_ready = true;
        __sev();

        // Core 0: EQ + gain for outputs 0-1
        for (int out = 0; out < CORE1_EQ_FIRST_OUTPUT; out++) {
            if (!matrix_mixer.outputs[out].enabled) continue;
            if (!matrix_mixer.outputs[out].mute) {
                uint8_t eq_ch = CH_OUT_1 + out;
                if (!channel_bypassed[eq_ch]) {
                    dsp_process_channel_block(filters[eq_ch], buf_out[out], sample_count, eq_ch);
                }
            }
            float gain = matrix_mixer.outputs[out].mute ? 0.0f
                         : matrix_mixer.outputs[out].gain_linear * vol_mul;
            if (gain == 0.0f) {
                memset(buf_out[out], 0, sample_count * sizeof(float));
            } else if (gain != 1.0f) {
                float *dst = buf_out[out];
                for (uint32_t i = 0; i < sample_count; i++)
                    dst[i] *= gain;
            }
        }

        // Core 0: Delay for outputs 0-1
        if (any_delay_active) {
            for (int out = 0; out < CORE1_EQ_FIRST_OUTPUT; out++) {
                int32_t dly = channel_delay_samples[out];
                if (dly <= 0) continue;
                float *dst = buf_out[out];
                float *dline = delay_lines[out];
                uint32_t widx = delay_write_idx;
                for (uint32_t i = 0; i < sample_count; i++) {
                    dline[widx] = dst[i];
                    dst[i] = dline[(widx - dly) & MAX_DELAY_MASK];
                    widx = (widx + 1) & MAX_DELAY_MASK;
                }
            }
        }

        // Core 0: Peaks for outputs 0..CORE1_EQ_FIRST_OUTPUT-1
        for (int out = 0; out < CORE1_EQ_FIRST_OUTPUT; out++) {
            float peak = 0;
            for (uint32_t i = 0; i < sample_count; i++) {
                float a = fabsf(buf_out[out][i]);
                if (a > peak) peak = a;
            }
            global_status.peaks[CH_OUT_1 + out] = (uint16_t)(fminf(1.0f, peak) * 32767.0f);
            if (peak > CLIP_THRESH_F) global_status.clip_flags |= (1u << (CH_OUT_1 + out));
        }
        // PDM is inactive in EQ_WORKER mode
        global_status.peaks[CH_OUT_SUB] = 0;

        // Core 0: S/PDIF for pair 0
        if (audio_buf[0]) {
            int left_ch = 0, right_ch = 1;
            if (!matrix_mixer.outputs[left_ch].enabled && !matrix_mixer.outputs[right_ch].enabled) {
                memset(audio_buf[0]->buffer->bytes, 0, sample_count * 8);
            } else {
                int32_t *out_ptr = (int32_t *)audio_buf[0]->buffer->bytes;
                for (uint32_t i = 0; i < sample_count; i++) {
                    float dl = fmaxf(-1.0f, fminf(1.0f, buf_out[0][i]));
                    float dr = fmaxf(-1.0f, fminf(1.0f, buf_out[1][i]));
                    out_ptr[i*2]   = (int32_t)(dl * 8388607.0f);
                    out_ptr[i*2+1] = (int32_t)(dr * 8388607.0f);
                }
            }
        }

        // Wait for Core 1 (EQ + delay + S/PDIF for outputs 2-7)
        while (!core1_eq_work.work_done) {
            __wfe();
        }
        __dmb();

        // Update shared delay write index (both cores used same base)
        if (any_delay_active) {
            delay_write_idx = (delay_write_idx + sample_count) & MAX_DELAY_MASK;
        }
    } else {
        // --- Single-core path: all outputs on Core 0 ---

        // EQ + gain
        for (int out = 0; out < NUM_OUTPUT_CHANNELS; out++) {
            if (!matrix_mixer.outputs[out].enabled) continue;
            if (!matrix_mixer.outputs[out].mute) {
                uint8_t eq_ch = CH_OUT_1 + out;
                if (!channel_bypassed[eq_ch]) {
                    dsp_process_channel_block(filters[eq_ch], buf_out[out], sample_count, eq_ch);
                }
            }
            float gain = matrix_mixer.outputs[out].mute ? 0.0f
                         : matrix_mixer.outputs[out].gain_linear * vol_mul;
            if (gain == 0.0f) {
                memset(buf_out[out], 0, sample_count * sizeof(float));
            } else if (gain != 1.0f) {
                float *dst = buf_out[out];
                for (uint32_t i = 0; i < sample_count; i++)
                    dst[i] *= gain;
            }
        }

        // Delay
        if (any_delay_active) {
            for (int out = 0; out < NUM_OUTPUT_CHANNELS; out++) {
                int32_t dly = channel_delay_samples[out];
                if (dly <= 0) continue;
                float *dst = buf_out[out];
                float *dline = delay_lines[out];
                uint32_t widx = delay_write_idx;
                for (uint32_t i = 0; i < sample_count; i++) {
                    dline[widx] = dst[i];
                    dst[i] = dline[(widx - dly) & MAX_DELAY_MASK];
                    widx = (widx + 1) & MAX_DELAY_MASK;
                }
            }
            delay_write_idx = (delay_write_idx + sample_count) & MAX_DELAY_MASK;
        }

        // Peaks for all SPDIF outputs
        for (int out = 0; out < NUM_SPDIF_INSTANCES * 2; out++) {
            float peak = 0;
            for (uint32_t i = 0; i < sample_count; i++) {
                float a = fabsf(buf_out[out][i]);
                if (a > peak) peak = a;
            }
            global_status.peaks[CH_OUT_1 + out] = (uint16_t)(fminf(1.0f, peak) * 32767.0f);
            if (peak > CLIP_THRESH_F) global_status.clip_flags |= (1u << (CH_OUT_1 + out));
        }

        // S/PDIF conversion
        for (int pair = 0; pair < 4; pair++) {
            if (!audio_buf[pair]) continue;
            int left_ch = pair * 2;
            int right_ch = pair * 2 + 1;
            if (!matrix_mixer.outputs[left_ch].enabled && !matrix_mixer.outputs[right_ch].enabled) {
                memset(audio_buf[pair]->buffer->bytes, 0, sample_count * 8);
                continue;
            }
            int32_t *out_ptr = (int32_t *)audio_buf[pair]->buffer->bytes;
            for (uint32_t i = 0; i < sample_count; i++) {
                float dl = fmaxf(-1.0f, fminf(1.0f, buf_out[left_ch][i]));
                float dr = fmaxf(-1.0f, fminf(1.0f, buf_out[right_ch][i]));
                out_ptr[i*2]     = (int32_t)(dl * 8388607.0f);
                out_ptr[i*2+1]   = (int32_t)(dr * 8388607.0f);
            }
        }

#if ENABLE_SUB
        if (matrix_mixer.outputs[NUM_OUTPUT_CHANNELS-1].enabled) {
            float peak_sub = 0;
            for (uint32_t i = 0; i < sample_count; i++) {
                float abs_sub = fabsf(buf_out[NUM_OUTPUT_CHANNELS-1][i]);
                if (abs_sub > peak_sub) peak_sub = abs_sub;
            }
            global_status.peaks[CH_OUT_SUB] = (uint16_t)(fminf(1.0f, peak_sub) * 32767.0f);
            if (peak_sub > CLIP_THRESH_F) global_status.clip_flags |= (1u << CH_OUT_SUB);
            for (uint32_t i = 0; i < sample_count; i++) {
                int32_t pdm_sample_q28 = (int32_t)(buf_out[NUM_OUTPUT_CHANNELS-1][i] * pdm_scale);
                pdm_push_sample(pdm_sample_q28, false);
            }
        } else {
            global_status.peaks[CH_OUT_SUB] = 0;
        }
#endif
    }

    // Write input peaks
    global_status.peaks[0] = (uint16_t)(fminf(1.0f, peak_ml) * 32767.0f);
    global_status.peaks[1] = (uint16_t)(fminf(1.0f, peak_mr) * 32767.0f);
    if (peak_ml > CLIP_THRESH_F) global_status.clip_flags |= (1u << CH_MASTER_LEFT);
    if (peak_mr > CLIP_THRESH_F) global_status.clip_flags |= (1u << CH_MASTER_RIGHT);

#else
    // ------------------------------------------------------------------------
    // RP2040 BLOCK-BASED FIXED-POINT PIPELINE WITH MATRIX MIXER
    // 2 SPDIF stereo pairs + PDM sub, dual-core EQ worker
    // ------------------------------------------------------------------------
    int32_t vol_mul = audio_state.mute ? 0 : audio_state.vol_mul;

    // Preset-switch mute: output silence while new parameters are being applied.
    if (preset_loading) {
        vol_mul = 0;
        if (preset_mute_counter > sample_count) {
            preset_mute_counter -= sample_count;
        } else {
            preset_mute_counter = 0;
            preset_loading = false;
        }
    }

    int32_t preamp = global_preamp_mul;
    bool is_bypassed = bypass_master_eq;

    // Snapshot loudness state for this packet
    bool loud_on = loudness_enabled;
    const LoudnessCoeffs *loud_coeffs = current_loudness_coeffs;

    int32_t peak_ml = 0, peak_mr = 0;

    // Static buffers for block processing
    static int32_t buf_l[192], buf_r[192];

    // ========== PASS 1: Input conversion + Preamp + Loudness ==========
    if (bit_depth == 24) {
        const uint8_t *p = (const uint8_t *)data;
        for (uint32_t i = 0; i < sample_count; i++) {
            // 24-bit -> Q28: left-justify to [31:8] then >>2 = net <<6
            int32_t raw_left_32  = (int32_t)((uint32_t)p[2] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[0] << 8) >> 2;
            int32_t raw_right_32 = (int32_t)((uint32_t)p[5] << 24 | (uint32_t)p[4] << 16 | (uint32_t)p[3] << 8) >> 2;
            buf_l[i] = fast_mul_q28(raw_left_32, preamp);
            buf_r[i] = fast_mul_q28(raw_right_32, preamp);
            p += 6;
        }
    } else {
        const int16_t *in = (const int16_t *)data;
        for (uint32_t i = 0; i < sample_count; i++) {
            int32_t raw_left_32 = (int32_t)in[i*2] << 14;
            int32_t raw_right_32 = (int32_t)in[i*2+1] << 14;
            buf_l[i] = fast_mul_q28(raw_left_32, preamp);
            buf_r[i] = fast_mul_q28(raw_right_32, preamp);
        }
    }

    // Loudness compensation (per-sample — biquad state coupling)
    if (loud_on && loud_coeffs) {
        for (uint32_t i = 0; i < sample_count; i++) {
            int32_t raw_left_32 = buf_l[i];
            int32_t raw_right_32 = buf_r[i];
            for (int j = 0; j < LOUDNESS_BIQUAD_COUNT; j++) {
                const LoudnessCoeffs *lc = &loud_coeffs[j];
                if (lc->bypass) continue;
                Biquad *bq = &loudness_biquads[0][j];
                int32_t result = fast_mul_q28(lc->b0, raw_left_32) + bq->s1;
                bq->s1 = fast_mul_q28(lc->b1, raw_left_32)
                        - fast_mul_q28(lc->a1, result) + bq->s2;
                bq->s2 = fast_mul_q28(lc->b2, raw_left_32)
                        - fast_mul_q28(lc->a2, result);
                raw_left_32 = result;
            }
            for (int j = 0; j < LOUDNESS_BIQUAD_COUNT; j++) {
                const LoudnessCoeffs *lc = &loud_coeffs[j];
                if (lc->bypass) continue;
                Biquad *bq = &loudness_biquads[1][j];
                int32_t result = fast_mul_q28(lc->b0, raw_right_32) + bq->s1;
                bq->s1 = fast_mul_q28(lc->b1, raw_right_32)
                        - fast_mul_q28(lc->a1, result) + bq->s2;
                bq->s2 = fast_mul_q28(lc->b2, raw_right_32)
                        - fast_mul_q28(lc->a2, result);
                raw_right_32 = result;
            }
            buf_l[i] = raw_left_32;
            buf_r[i] = raw_right_32;
        }
    }

    // ========== PASS 2: Master EQ (Block-Based) ==========
    if (!is_bypassed) {
        if (!channel_bypassed[CH_MASTER_LEFT])
            dsp_process_channel_block(filters[CH_MASTER_LEFT], buf_l, sample_count, CH_MASTER_LEFT);
        if (!channel_bypassed[CH_MASTER_RIGHT])
            dsp_process_channel_block(filters[CH_MASTER_RIGHT], buf_r, sample_count, CH_MASTER_RIGHT);
    }

    // ========== PASS 3: Crossfeed + Master Peaks ==========
    for (uint32_t i = 0; i < sample_count; i++) {
        int32_t ml = buf_l[i], mr = buf_r[i];
        if (abs(ml) > peak_ml) peak_ml = abs(ml);
        if (abs(mr) > peak_mr) peak_mr = abs(mr);
        if (!crossfeed_bypassed) {
            crossfeed_process_stereo(&crossfeed_state, &ml, &mr);
            buf_l[i] = ml; buf_r[i] = mr;
        }
    }

    // ========== PASS 4: Matrix Mixing (block-based, output-major) ==========
    for (int out = 0; out < NUM_OUTPUT_CHANNELS; out++) {
        if (!matrix_mixer.outputs[out].enabled) {
            memset(buf_out[out], 0, sample_count * sizeof(int32_t));
            continue;
        }

        MatrixCrosspoint *xp_l = &matrix_mixer.crosspoints[0][out];
        MatrixCrosspoint *xp_r = &matrix_mixer.crosspoints[1][out];
        int32_t gain_l_q15 = xp_l->enabled ? (int32_t)((xp_l->phase_invert ? -xp_l->gain_linear : xp_l->gain_linear) * 32768.0f) : 0;
        int32_t gain_r_q15 = xp_r->enabled ? (int32_t)((xp_r->phase_invert ? -xp_r->gain_linear : xp_r->gain_linear) * 32768.0f) : 0;

        int32_t *dst = buf_out[out];
        if (gain_l_q15 != 0 && gain_r_q15 != 0) {
            for (uint32_t i = 0; i < sample_count; i++)
                dst[i] = fast_mul_q15(buf_l[i], gain_l_q15) + fast_mul_q15(buf_r[i], gain_r_q15);
        } else if (gain_l_q15 != 0) {
            for (uint32_t i = 0; i < sample_count; i++)
                dst[i] = fast_mul_q15(buf_l[i], gain_l_q15);
        } else if (gain_r_q15 != 0) {
            for (uint32_t i = 0; i < sample_count; i++)
                dst[i] = fast_mul_q15(buf_r[i], gain_r_q15);
        } else {
            memset(dst, 0, sample_count * sizeof(int32_t));
        }
    }

    // ========== PASS 5-7: Per-Output EQ + Gain + Delay + Output ==========
    // PDM output index
    int pdm_out = NUM_OUTPUT_CHANNELS - 1;

    if (core1_mode == CORE1_MODE_EQ_WORKER) {
        // --- Dual-core path: Core 0 handles pair 1, Core 1 handles pair 2 ---

        // Dispatch to Core 1
        core1_eq_work.sample_count = sample_count;
        core1_eq_work.vol_mul = vol_mul;
        core1_eq_work.delay_write_idx = delay_write_idx;
        core1_eq_work.spdif_out[0] = audio_buf[1] ? (int32_t *)audio_buf[1]->buffer->bytes : NULL;
        core1_eq_work.work_done = false;
        __dmb();
        core1_eq_work.work_ready = true;
        __sev();

        // Core 0: EQ + gain for outputs 0-1 (SPDIF pair 1)
        for (int out = 0; out < CORE1_EQ_FIRST_OUTPUT; out++) {
            if (!matrix_mixer.outputs[out].enabled) continue;
            if (!matrix_mixer.outputs[out].mute) {
                uint8_t eq_ch = CH_OUT_1 + out;
                if (!is_bypassed && !channel_bypassed[eq_ch])
                    dsp_process_channel_block(filters[eq_ch], buf_out[out], sample_count, eq_ch);
            }
            int32_t gain = matrix_mixer.outputs[out].mute ? 0
                           : (int32_t)(matrix_mixer.outputs[out].gain_linear * (float)vol_mul);
            if (gain == 0) {
                memset(buf_out[out], 0, sample_count * sizeof(int32_t));
            } else {
                int32_t *dst = buf_out[out];
                for (uint32_t i = 0; i < sample_count; i++)
                    dst[i] = fast_mul_q15(dst[i], gain);
            }
        }

        // Core 0: Delay for outputs 0-1
        if (any_delay_active) {
            for (int out = 0; out < CORE1_EQ_FIRST_OUTPUT; out++) {
                int32_t dly = channel_delay_samples[out];
                if (dly <= 0) continue;
                int32_t *dst = buf_out[out];
                int32_t *dline = delay_lines[out];
                uint32_t widx = delay_write_idx;
                for (uint32_t i = 0; i < sample_count; i++) {
                    dline[widx] = dst[i];
                    dst[i] = dline[(widx - dly) & MAX_DELAY_MASK];
                    widx = (widx + 1) & MAX_DELAY_MASK;
                }
            }
        }

        // Core 0: Peaks for outputs 0..CORE1_EQ_FIRST_OUTPUT-1
        for (int out = 0; out < CORE1_EQ_FIRST_OUTPUT; out++) {
            int32_t peak = 0;
            for (uint32_t i = 0; i < sample_count; i++) {
                int32_t a = abs(buf_out[out][i]);
                if (a > peak) peak = a;
            }
            global_status.peaks[CH_OUT_1 + out] = (uint16_t)(peak >> 13);
            if (peak > CLIP_THRESH_Q28) global_status.clip_flags |= (1u << (CH_OUT_1 + out));
        }
        // PDM is inactive in EQ_WORKER mode
        global_status.peaks[CH_OUT_SUB] = 0;

        // Core 0: S/PDIF conversion for pair 1
        if (audio_buf[0]) {
            if (!matrix_mixer.outputs[0].enabled && !matrix_mixer.outputs[1].enabled) {
                memset(audio_buf[0]->buffer->bytes, 0, sample_count * 8);
            } else {
                int32_t *out_ptr = (int32_t *)audio_buf[0]->buffer->bytes;
                for (uint32_t i = 0; i < sample_count; i++) {
                    out_ptr[i*2]   = clip_s24((buf_out[0][i] + (1 << 5)) >> 6);
                    out_ptr[i*2+1] = clip_s24((buf_out[1][i] + (1 << 5)) >> 6);
                }
            }
        }

        // Wait for Core 1 (EQ + delay + S/PDIF for outputs 2-3)
        while (!core1_eq_work.work_done) {
            __wfe();
        }
        __dmb();

        // Update shared delay write index
        if (any_delay_active) {
            delay_write_idx = (delay_write_idx + sample_count) & MAX_DELAY_MASK;
        }
    } else {
        // --- Single-core path: all outputs on Core 0 ---
        uint32_t saved_delay_write_idx = delay_write_idx;

        // EQ + gain (block-based)
        for (int out = 0; out < NUM_OUTPUT_CHANNELS; out++) {
            if (!matrix_mixer.outputs[out].enabled) continue;
            if (!matrix_mixer.outputs[out].mute) {
                uint8_t eq_ch = CH_OUT_1 + out;
                if (!is_bypassed && !channel_bypassed[eq_ch])
                    dsp_process_channel_block(filters[eq_ch], buf_out[out], sample_count, eq_ch);
            }
            int32_t gain = matrix_mixer.outputs[out].mute ? 0
                           : (int32_t)(matrix_mixer.outputs[out].gain_linear * (float)vol_mul);
            if (gain == 0) {
                memset(buf_out[out], 0, sample_count * sizeof(int32_t));
            } else {
                int32_t *dst = buf_out[out];
                for (uint32_t i = 0; i < sample_count; i++)
                    dst[i] = fast_mul_q15(dst[i], gain);
            }
        }

        // Delay (all outputs use same base write index)
        if (any_delay_active) {
            for (int out = 0; out < NUM_OUTPUT_CHANNELS; out++) {
                int32_t dly = channel_delay_samples[out];
                if (dly <= 0) continue;
                int32_t *dst = buf_out[out];
                int32_t *dline = delay_lines[out];
                uint32_t widx = saved_delay_write_idx;
                for (uint32_t i = 0; i < sample_count; i++) {
                    dline[widx] = dst[i];
                    dst[i] = dline[(widx - dly) & MAX_DELAY_MASK];
                    widx = (widx + 1) & MAX_DELAY_MASK;
                }
            }
            delay_write_idx = (saved_delay_write_idx + sample_count) & MAX_DELAY_MASK;
        }

        // Peaks for all SPDIF outputs
        for (int out = 0; out < NUM_SPDIF_INSTANCES * 2; out++) {
            int32_t peak = 0;
            for (uint32_t i = 0; i < sample_count; i++) {
                int32_t a = abs(buf_out[out][i]);
                if (a > peak) peak = a;
            }
            global_status.peaks[CH_OUT_1 + out] = (uint16_t)(peak >> 13);
            if (peak > CLIP_THRESH_Q28) global_status.clip_flags |= (1u << (CH_OUT_1 + out));
        }

        // S/PDIF conversion (2 stereo pairs)
        for (int pair = 0; pair < NUM_SPDIF_INSTANCES; pair++) {
            if (!audio_buf[pair]) continue;
            int left_ch = pair * 2;
            int right_ch = pair * 2 + 1;
            if (!matrix_mixer.outputs[left_ch].enabled && !matrix_mixer.outputs[right_ch].enabled) {
                memset(audio_buf[pair]->buffer->bytes, 0, sample_count * 8);
                continue;
            }
            int32_t *out_ptr = (int32_t *)audio_buf[pair]->buffer->bytes;
            for (uint32_t i = 0; i < sample_count; i++) {
                out_ptr[i*2]   = clip_s24((buf_out[left_ch][i] + (1 << 5)) >> 6);
                out_ptr[i*2+1] = clip_s24((buf_out[right_ch][i] + (1 << 5)) >> 6);
            }
        }

#if ENABLE_SUB
        // PDM sub output
        if (matrix_mixer.outputs[pdm_out].enabled) {
            int32_t peak_sub = 0;
            for (uint32_t i = 0; i < sample_count; i++) {
                int32_t abs_sub = abs(buf_out[pdm_out][i]);
                if (abs_sub > peak_sub) peak_sub = abs_sub;
            }
            global_status.peaks[CH_OUT_SUB] = (uint16_t)(peak_sub >> 13);
            if (peak_sub > CLIP_THRESH_Q28) global_status.clip_flags |= (1u << CH_OUT_SUB);
            for (uint32_t i = 0; i < sample_count; i++) {
                pdm_push_sample(buf_out[pdm_out][i], false);
            }
        } else {
            global_status.peaks[CH_OUT_SUB] = 0;
        }
#endif
    }

    // Write input peaks
    global_status.peaks[0] = (uint16_t)(peak_ml >> 13);
    global_status.peaks[1] = (uint16_t)(peak_mr >> 13);
    if (peak_ml > CLIP_THRESH_Q28) global_status.clip_flags |= (1u << CH_MASTER_LEFT);
    if (peak_mr > CLIP_THRESH_Q28) global_status.clip_flags |= (1u << CH_MASTER_RIGHT);
#endif

    // Return all buffers
#if PICO_RP2350
    for (int b = 0; b < 4; b++) {
        if (audio_buf[b]) {
            struct audio_buffer_pool *pool = (b == 0) ? producer_pool_1 :
                                              (b == 1) ? producer_pool_2 :
                                              (b == 2) ? producer_pool_3 : producer_pool_4;
            give_audio_buffer(pool, audio_buf[b]);
        }
    }
#else
    if (audio_buf[0]) give_audio_buffer(producer_pool_1, audio_buf[0]);
    if (audio_buf[1]) give_audio_buffer(producer_pool_2, audio_buf[1]);
#endif

    uint32_t packet_end = time_us_32();

    if (cpu0_load_primed) {
        uint32_t busy_us = packet_end - packet_start;
        uint32_t idle_us = packet_start - cpu0_last_packet_end;
        if (idle_us > 2000) idle_us = 0;   // clamp during USB gaps

        uint32_t total_us = busy_us + idle_us;
        if (total_us > 0) {
            uint32_t inst_q8 = (busy_us * 25600) / total_us;
            cpu0_load_q8 = cpu0_load_q8 - (cpu0_load_q8 >> 3) + (inst_q8 >> 3);
        }
        global_status.cpu0_load = (uint8_t)((cpu0_load_q8 + 128) >> 8);
    } else {
        cpu0_load_primed = true;
    }
    cpu0_last_packet_end = packet_end;
}

// ----------------------------------------------------------------------------
// USB AUDIO PACKET CALLBACKS (pico-extras usb_device)
// ----------------------------------------------------------------------------

static void __not_in_flash_func(_as_audio_packet)(struct usb_endpoint *ep) {
    assert(ep->current_transfer);
    struct usb_buffer *usb_buffer = usb_current_out_packet_buffer(ep);

    usb_audio_packets++;
    process_audio_packet(usb_buffer->data, usb_buffer->data_len);

    // keep on truckin'
    usb_grow_transfer(ep->current_transfer, 1);
    usb_packet_done(ep);
}

static void __not_in_flash_func(_as_sync_packet)(struct usb_endpoint *ep) {
    assert(ep->current_transfer);
    struct usb_buffer *buffer = usb_current_in_packet_buffer(ep);
    assert(buffer->data_max >= 3);
    buffer->data_len = 3;

    // Use SOF-measured feedback; fall back to pre-computed nominal
    uint32_t fb = feedback_10_14;
    if (fb == 0) fb = nominal_feedback_10_14;

    buffer->data[0] = fb;
    buffer->data[1] = fb >> 8u;
    buffer->data[2] = fb >> 16u;

    usb_grow_transfer(ep->current_transfer, 1);
    usb_packet_done(ep);
}

static const struct usb_transfer_type as_transfer_type = {
    .on_packet = _as_audio_packet,
    .initial_packet_count = 1,
};

static const struct usb_transfer_type as_sync_transfer_type = {
    .on_packet = _as_sync_packet,
    .initial_packet_count = 1,
};

static struct usb_transfer as_transfer;
static struct usb_transfer as_sync_transfer;

// ----------------------------------------------------------------------------
// UAC1 AUDIO CONTROL REQUEST HANDLERS
// ----------------------------------------------------------------------------

static struct audio_control_cmd {
    uint8_t cmd;
    uint8_t type;
    uint8_t cs;
    uint8_t cn;
    uint8_t unit;
    uint8_t len;
} audio_control_cmd_t;

static void _audio_reconfigure(void) {
    rate_change_pending = true;
    pending_rate = audio_state.freq;
}

static bool do_get_current(struct usb_setup_packet *setup) {
    if ((setup->bmRequestType & USB_REQ_TYPE_RECIPIENT_MASK) == USB_REQ_TYPE_RECIPIENT_INTERFACE) {
        switch (setup->wValue >> 8u) {
        case FEATURE_MUTE_CONTROL:
            usb_start_tiny_control_in_transfer(audio_state.mute, 1);
            return true;
        case FEATURE_VOLUME_CONTROL:
            usb_start_tiny_control_in_transfer(audio_state.volume, 2);
            return true;
        }
    } else if ((setup->bmRequestType & USB_REQ_TYPE_RECIPIENT_MASK) == USB_REQ_TYPE_RECIPIENT_ENDPOINT) {
        if ((setup->wValue >> 8u) == ENDPOINT_FREQ_CONTROL) {
            usb_start_tiny_control_in_transfer(audio_state.freq, 3);
            return true;
        }
    }
    return false;
}

static bool do_get_minimum(struct usb_setup_packet *setup) {
    if ((setup->bmRequestType & USB_REQ_TYPE_RECIPIENT_MASK) == USB_REQ_TYPE_RECIPIENT_INTERFACE) {
        switch (setup->wValue >> 8u) {
        case FEATURE_VOLUME_CONTROL:
            usb_start_tiny_control_in_transfer(MIN_VOLUME, 2);
            return true;
        }
    }
    return false;
}

static bool do_get_maximum(struct usb_setup_packet *setup) {
    if ((setup->bmRequestType & USB_REQ_TYPE_RECIPIENT_MASK) == USB_REQ_TYPE_RECIPIENT_INTERFACE) {
        switch (setup->wValue >> 8u) {
        case FEATURE_VOLUME_CONTROL:
            usb_start_tiny_control_in_transfer(MAX_VOLUME, 2);
            return true;
        }
    }
    return false;
}

static bool do_get_resolution(struct usb_setup_packet *setup) {
    if ((setup->bmRequestType & USB_REQ_TYPE_RECIPIENT_MASK) == USB_REQ_TYPE_RECIPIENT_INTERFACE) {
        switch (setup->wValue >> 8u) {
        case FEATURE_VOLUME_CONTROL:
            usb_start_tiny_control_in_transfer(VOLUME_RESOLUTION, 2);
            return true;
        }
    }
    return false;
}

static void audio_cmd_packet(struct usb_endpoint *ep) {
    assert(audio_control_cmd_t.cmd == AUDIO_REQ_SetCurrent);
    struct usb_buffer *buffer = usb_current_out_packet_buffer(ep);
    audio_control_cmd_t.cmd = 0;
    if (buffer->data_len >= audio_control_cmd_t.len) {
        if (audio_control_cmd_t.type == USB_REQ_TYPE_RECIPIENT_INTERFACE) {
            switch (audio_control_cmd_t.cs) {
            case FEATURE_MUTE_CONTROL:
                audio_state.mute = buffer->data[0];
                break;
            case FEATURE_VOLUME_CONTROL:
                audio_set_volume(*(int16_t *) buffer->data);
                break;
            }
        } else if (audio_control_cmd_t.type == USB_REQ_TYPE_RECIPIENT_ENDPOINT) {
            if (audio_control_cmd_t.cs == ENDPOINT_FREQ_CONTROL) {
                uint32_t new_freq = (*(uint32_t *) buffer->data) & 0x00ffffffu;
                if (audio_state.freq != new_freq) {
                    audio_state.freq = new_freq;
                    _audio_reconfigure();
                }
            }
        }
    }
    usb_start_empty_control_in_transfer_null_completion();
}

static const struct usb_transfer_type _audio_cmd_transfer_type = {
    .on_packet = audio_cmd_packet,
    .initial_packet_count = 1,
};

static bool do_set_current(struct usb_setup_packet *setup) {
    if (setup->wLength && setup->wLength < 64) {
        audio_control_cmd_t.cmd = AUDIO_REQ_SetCurrent;
        audio_control_cmd_t.type = setup->bmRequestType & USB_REQ_TYPE_RECIPIENT_MASK;
        audio_control_cmd_t.len = (uint8_t) setup->wLength;
        audio_control_cmd_t.unit = setup->wIndex >> 8u;
        audio_control_cmd_t.cs = setup->wValue >> 8u;
        audio_control_cmd_t.cn = (uint8_t) setup->wValue;
        usb_start_control_out_transfer(&_audio_cmd_transfer_type);
        return true;
    }
    return false;
}

// Forward declaration — Console app sends vendor requests with wIndex=0 (interface recipient),
// so they arrive here on the AC interface and must be forwarded to the vendor handler.
static bool vendor_setup_request_handler(struct usb_interface *interface, struct usb_setup_packet *setup);

static bool ac_setup_request_handler(__unused struct usb_interface *interface, struct usb_setup_packet *setup) {
    setup = __builtin_assume_aligned(setup, 4);

    // Forward vendor-type requests to the vendor handler (Console sends wIndex=0)
    if (USB_REQ_TYPE_TYPE_VENDOR == (setup->bmRequestType & USB_REQ_TYPE_TYPE_MASK)) {
        return vendor_setup_request_handler(interface, setup);
    }

    if (USB_REQ_TYPE_TYPE_CLASS == (setup->bmRequestType & USB_REQ_TYPE_TYPE_MASK)) {
        switch (setup->bRequest) {
        case AUDIO_REQ_SetCurrent:
            return do_set_current(setup);
        case AUDIO_REQ_GetCurrent:
            return do_get_current(setup);
        case AUDIO_REQ_GetMinimum:
            return do_get_minimum(setup);
        case AUDIO_REQ_GetMaximum:
            return do_get_maximum(setup);
        case AUDIO_REQ_GetResolution:
            return do_get_resolution(setup);
        default:
            break;
        }
    }
    return false;
}

static bool _as_setup_request_handler(__unused struct usb_endpoint *ep, struct usb_setup_packet *setup) {
    setup = __builtin_assume_aligned(setup, 4);
    if (USB_REQ_TYPE_TYPE_CLASS == (setup->bmRequestType & USB_REQ_TYPE_TYPE_MASK)) {
        switch (setup->bRequest) {
        case AUDIO_REQ_SetCurrent:
            return do_set_current(setup);
        case AUDIO_REQ_GetCurrent:
            return do_get_current(setup);
        case AUDIO_REQ_GetMinimum:
            return do_get_minimum(setup);
        case AUDIO_REQ_GetMaximum:
            return do_get_maximum(setup);
        case AUDIO_REQ_GetResolution:
            return do_get_resolution(setup);
        default:
            break;
        }
    }
    return false;
}

static bool as_set_alternate(struct usb_interface *interface, uint alt) {
    assert(interface == &as_op_interface);
    usb_audio_alt_set = alt;
    if (alt == 2) {
        usb_input_bit_depth = 24;
    } else {
        usb_input_bit_depth = 16;
    }
    return alt < 3;
}

// ----------------------------------------------------------------------------
// VENDOR INTERFACE HANDLER (DSPi commands via EP0 control transfers)
// ----------------------------------------------------------------------------

// Buffer for vendor SET requests
static uint8_t vendor_rx_buf[64];
static uint8_t vendor_last_request = 0;
static uint16_t vendor_last_wValue = 0;

// Derive Core 1 mode from current output enable state
Core1Mode derive_core1_mode(void) {
    // PDM output (last) takes priority — checked first
    if (matrix_mixer.outputs[NUM_OUTPUT_CHANNELS - 1].enabled)
        return CORE1_MODE_PDM;
    // Any of outputs 2-7 enabled → EQ worker
    for (int out = CORE1_EQ_FIRST_OUTPUT; out <= CORE1_EQ_LAST_OUTPUT; out++) {
        if (matrix_mixer.outputs[out].enabled)
            return CORE1_MODE_EQ_WORKER;
    }
    return CORE1_MODE_IDLE;
}

static void vendor_cmd_packet(struct usb_endpoint *ep) {
    struct usb_buffer *buffer = usb_current_out_packet_buffer(ep);

    if (buffer->data_len > 0 && buffer->data_len <= sizeof(vendor_rx_buf)) {
        memcpy(vendor_rx_buf, buffer->data, buffer->data_len);
    }

    // Process command based on saved request info
    switch (vendor_last_request) {
        case REQ_SET_EQ_PARAM:
            if (buffer->data_len >= sizeof(EqParamPacket)) {
                memcpy((void*)&pending_packet, vendor_rx_buf, sizeof(EqParamPacket));
                if (pending_packet.channel < NUM_CHANNELS &&
                    pending_packet.band < channel_band_counts[pending_packet.channel]) {
                    eq_update_pending = true;
                }
            }
            break;

        case REQ_SET_PREAMP:
            if (buffer->data_len >= 4) {
                float db;
                memcpy(&db, vendor_rx_buf, 4);
                global_preamp_db = db;
                float linear = powf(10.0f, db / 20.0f);
                global_preamp_mul = (int32_t)(linear * (float)(1<<28));
                global_preamp_linear = linear;
            }
            break;

        case REQ_SET_DELAY: {
            uint8_t ch = vendor_last_wValue & 0xFF;
            if (ch < NUM_CHANNELS && buffer->data_len >= 4) {
                float ms;
                memcpy(&ms, vendor_rx_buf, 4);
                if (ms < 0) ms = 0;
                channel_delays_ms[ch] = ms;
                dsp_update_delay_samples((float)audio_state.freq);
            }
            break;
        }

        case REQ_SET_BYPASS:
            if (buffer->data_len >= 1) {
                bypass_master_eq = (vendor_rx_buf[0] != 0);
            }
            break;

        case REQ_SET_CHANNEL_GAIN: {
            uint8_t ch = vendor_last_wValue & 0xFF;
            if (ch < 3 && buffer->data_len >= 4) {
                float db;
                memcpy(&db, vendor_rx_buf, 4);
                channel_gain_db[ch] = db;
                float linear = powf(10.0f, db / 20.0f);
                channel_gain_mul[ch] = (int32_t)(linear * 32768.0f);
                channel_gain_linear[ch] = linear;
            }
            break;
        }

        case REQ_SET_CHANNEL_MUTE: {
            uint8_t ch = vendor_last_wValue & 0xFF;
            if (ch < 3 && buffer->data_len >= 1) {
                channel_mute[ch] = (vendor_rx_buf[0] != 0);
            }
            break;
        }

        case REQ_SET_LOUDNESS:
            if (buffer->data_len >= 1) {
                loudness_enabled = (vendor_rx_buf[0] != 0);
                if (loudness_enabled && loudness_active_table) {
                    // Re-select coefficients for current volume
                    int16_t vol = audio_state.volume + CENTER_VOLUME_INDEX * 256;
                    if (vol < 0) vol = 0;
                    if (vol >= (CENTER_VOLUME_INDEX + 1) * 256) vol = (CENTER_VOLUME_INDEX + 1) * 256 - 1;
                    current_loudness_coeffs = loudness_active_table[((uint16_t)vol) >> 8u];
                } else {
                    current_loudness_coeffs = NULL;
                }
            }
            break;

        case REQ_SET_LOUDNESS_REF:
            if (buffer->data_len >= 4) {
                float val;
                memcpy(&val, vendor_rx_buf, 4);
                if (val < 40.0f) val = 40.0f;
                if (val > 100.0f) val = 100.0f;
                loudness_ref_spl = val;
                loudness_recompute_pending = true;
            }
            break;

        case REQ_SET_LOUDNESS_INTENSITY:
            if (buffer->data_len >= 4) {
                float val;
                memcpy(&val, vendor_rx_buf, 4);
                if (val < 0.0f) val = 0.0f;
                if (val > 200.0f) val = 200.0f;
                loudness_intensity_pct = val;
                loudness_recompute_pending = true;
            }
            break;

        case REQ_SET_CROSSFEED:
            if (buffer->data_len >= 1) {
                crossfeed_config.enabled = (vendor_rx_buf[0] != 0);
                crossfeed_update_pending = true;
            }
            break;

        case REQ_SET_CROSSFEED_PRESET:
            if (buffer->data_len >= 1) {
                uint8_t preset = vendor_rx_buf[0];
                if (preset <= CROSSFEED_PRESET_CUSTOM) {
                    crossfeed_config.preset = preset;
                    crossfeed_update_pending = true;
                }
            }
            break;

        case REQ_SET_CROSSFEED_FREQ:
            if (buffer->data_len >= 4) {
                float val;
                memcpy(&val, vendor_rx_buf, 4);
                if (val < CROSSFEED_FREQ_MIN) val = CROSSFEED_FREQ_MIN;
                if (val > CROSSFEED_FREQ_MAX) val = CROSSFEED_FREQ_MAX;
                crossfeed_config.custom_fc = val;
                if (crossfeed_config.preset == CROSSFEED_PRESET_CUSTOM) {
                    crossfeed_update_pending = true;
                }
            }
            break;

        case REQ_SET_CROSSFEED_FEED:
            if (buffer->data_len >= 4) {
                float val;
                memcpy(&val, vendor_rx_buf, 4);
                if (val < CROSSFEED_FEED_MIN) val = CROSSFEED_FEED_MIN;
                if (val > CROSSFEED_FEED_MAX) val = CROSSFEED_FEED_MAX;
                crossfeed_config.custom_feed_db = val;
                if (crossfeed_config.preset == CROSSFEED_PRESET_CUSTOM) {
                    crossfeed_update_pending = true;
                }
            }
            break;

        case REQ_SET_CROSSFEED_ITD:
            if (buffer->data_len >= 1) {
                crossfeed_config.itd_enabled = (vendor_rx_buf[0] != 0);
                crossfeed_update_pending = true;
            }
            break;

        // Matrix Mixer Commands
        case REQ_SET_MATRIX_ROUTE:
            if (buffer->data_len >= sizeof(MatrixRoutePacket)) {
                MatrixRoutePacket pkt;
                memcpy(&pkt, vendor_rx_buf, sizeof(pkt));
                if (pkt.input < NUM_INPUT_CHANNELS && pkt.output < NUM_OUTPUT_CHANNELS) {
                    MatrixCrosspoint *xp = &matrix_mixer.crosspoints[pkt.input][pkt.output];
                    xp->enabled = pkt.enabled;
                    xp->phase_invert = pkt.phase_invert;
                    xp->gain_db = pkt.gain_db;
                    // Compute linear gain
                    xp->gain_linear = powf(10.0f, pkt.gain_db / 20.0f);
                }
            }
            break;

        case REQ_SET_OUTPUT_ENABLE: {
            uint8_t out = vendor_last_wValue & 0xFF;
            if (out < NUM_OUTPUT_CHANNELS && buffer->data_len >= 1) {
                bool want_enable = (vendor_rx_buf[0] != 0);

                // Mutual exclusion interlock: PDM vs EQ worker outputs
                // Core 1 can only do one: PDM or EQ worker (outputs 2+ on both platforms)
                if (want_enable) {
                    bool is_pdm = (out == NUM_OUTPUT_CHANNELS - 1);
                    bool is_core1_eq = (out >= CORE1_EQ_FIRST_OUTPUT && out <= CORE1_EQ_LAST_OUTPUT);

                    if (is_pdm) {
                        for (int i = CORE1_EQ_FIRST_OUTPUT; i <= CORE1_EQ_LAST_OUTPUT; i++) {
                            if (matrix_mixer.outputs[i].enabled) goto skip_enable;
                        }
                    } else if (is_core1_eq) {
                        if (matrix_mixer.outputs[NUM_OUTPUT_CHANNELS - 1].enabled) goto skip_enable;
                    }
                }

                matrix_mixer.outputs[out].enabled = want_enable ? 1 : 0;

                // Determine new Core 1 mode and transition
                Core1Mode new_mode = derive_core1_mode();
                if (new_mode != core1_mode) {
                    core1_mode = new_mode;
#if ENABLE_SUB
                    pdm_set_enabled(new_mode == CORE1_MODE_PDM);
#endif
                    __sev();  // Wake Core 1 to pick up mode change
                }
            }
            skip_enable:
            break;
        }

        case REQ_SET_OUTPUT_GAIN: {
            uint8_t out = vendor_last_wValue & 0xFF;
            if (out < NUM_OUTPUT_CHANNELS && buffer->data_len >= 4) {
                float db;
                memcpy(&db, vendor_rx_buf, 4);
                matrix_mixer.outputs[out].gain_db = db;
                matrix_mixer.outputs[out].gain_linear = powf(10.0f, db / 20.0f);
            }
            break;
        }

        case REQ_SET_OUTPUT_MUTE: {
            uint8_t out = vendor_last_wValue & 0xFF;
            if (out < NUM_OUTPUT_CHANNELS && buffer->data_len >= 1) {
                matrix_mixer.outputs[out].mute = vendor_rx_buf[0];
            }
            break;
        }

        case REQ_SET_OUTPUT_DELAY: {
            uint8_t out = vendor_last_wValue & 0xFF;
            if (out < NUM_OUTPUT_CHANNELS && buffer->data_len >= 4) {
                float ms;
                memcpy(&ms, vendor_rx_buf, 4);
                if (ms < 0) ms = 0;
                matrix_mixer.outputs[out].delay_ms = ms;
                // Update the channel delay used by DSP pipeline
                channel_delays_ms[CH_OUT_1 + out] = ms;
                dsp_update_delay_samples((float)audio_state.freq);
            }
            break;
        }

        // --- Preset SET commands ---

        case REQ_PRESET_SET_NAME: {
            // wValue = slot index, payload = up to 32 bytes of name
            uint8_t slot = vendor_last_wValue & 0xFF;
            if (buffer->data_len > 0) {
                // Ensure NUL termination
                char name[PRESET_NAME_LEN];
                memset(name, 0, sizeof(name));
                size_t copy_len = buffer->data_len < (PRESET_NAME_LEN - 1)
                                ? buffer->data_len : (PRESET_NAME_LEN - 1);
                memcpy(name, vendor_rx_buf, copy_len);
                preset_set_name(slot, name);
            }
            break;
        }

        case REQ_PRESET_SET_STARTUP: {
            // payload: byte 0 = startup_mode, byte 1 = default_slot
            if (buffer->data_len >= 2) {
                preset_set_startup(vendor_rx_buf[0], vendor_rx_buf[1]);
            }
            break;
        }

        case REQ_PRESET_SET_INCLUDE_PINS: {
            // payload: byte 0 = include_pins (0 or 1)
            if (buffer->data_len >= 1) {
                preset_set_include_pins(vendor_rx_buf[0]);
            }
            break;
        }

        case REQ_SET_CHANNEL_NAME: {
            // wValue = channel index, payload = 1-32 bytes of name
            uint8_t ch = vendor_last_wValue & 0xFF;
            if (ch < NUM_CHANNELS && buffer->data_len > 0) {
                memset(channel_names[ch], 0, PRESET_NAME_LEN);
                size_t copy_len = buffer->data_len < (PRESET_NAME_LEN - 1)
                                ? buffer->data_len : (PRESET_NAME_LEN - 1);
                memcpy(channel_names[ch], vendor_rx_buf, copy_len);
            }
            break;
        }
    }

    usb_start_empty_control_in_transfer_null_completion();
}

static const struct usb_transfer_type _vendor_cmd_transfer_type = {
    .on_packet = vendor_cmd_packet,
    .initial_packet_count = 1,
};

// Helper: write data into the control IN buffer and send
static void vendor_send_response(const void *data, uint len) {
    struct usb_buffer *buffer = usb_current_in_packet_buffer(usb_get_control_in_endpoint());
    memcpy(buffer->data, data, len);
    buffer->data_len = len;
    usb_start_single_buffer_control_in_transfer();
}

// Runtime pin configuration
#if PICO_RP2350
uint8_t output_pins[NUM_PIN_OUTPUTS] = {
    PICO_AUDIO_SPDIF_PIN, PICO_SPDIF_PIN_2,
    PICO_SPDIF_PIN_3, PICO_SPDIF_PIN_4, PICO_PDM_PIN
};
#else
uint8_t output_pins[NUM_PIN_OUTPUTS] = {
    PICO_AUDIO_SPDIF_PIN, PICO_SPDIF_PIN_2, PICO_PDM_PIN
};
#endif

audio_spdif_instance_t *spdif_instance_ptrs[NUM_SPDIF_INSTANCES];

// Pin validation helpers
static bool is_valid_gpio_pin(uint8_t pin) {
    if (pin == 12) return false;                // UART TX
    if (pin >= 23 && pin <= 25) return false;   // Power/LED
#if PICO_RP2350
    return pin <= 29;
#else
    return pin <= 28;
#endif
}

static bool is_pin_in_use(uint8_t pin, uint8_t exclude) {
    for (int i = 0; i < NUM_PIN_OUTPUTS; i++) {
        if (i == exclude) continue;
        if (output_pins[i] == pin) return true;
    }
    return false;
}

// ----------------------------------------------------------------------------
// BUFFER STATISTICS HELPERS
// ----------------------------------------------------------------------------

static uint count_pool_free(audio_buffer_pool_t *pool) {
    uint32_t save = spin_lock_blocking(pool->free_list_spin_lock);
    uint count = audio_buffer_list_count(pool->free_list);
    spin_unlock(pool->free_list_spin_lock, save);
    return count;
}

static uint count_pool_prepared(audio_buffer_pool_t *pool) {
    uint32_t save = spin_lock_blocking(pool->prepared_list_spin_lock);
    uint count = audio_buffer_list_count(pool->prepared_list);
    spin_unlock(pool->prepared_list_spin_lock, save);
    return count;
}

static void reset_buffer_watermarks(void) {
    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        spdif_consumer_min_fill_pct[i] = 100;
        spdif_consumer_max_fill_pct[i] = 0;
    }
    pdm_dma_min_fill_pct = 100;
    pdm_dma_max_fill_pct = 0;
    pdm_ring_min_fill_pct = 100;
    pdm_ring_max_fill_pct = 0;
}

static void update_buffer_watermarks(void) {
    uint consumer_capacity = SPDIF_CONSUMER_BUFFER_COUNT;

    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        audio_spdif_instance_t *inst = spdif_instance_ptrs[i];
        uint cons_prepared = count_pool_prepared(inst->consumer_pool);
        uint playing = (inst->playing_buffer != NULL) ? 1 : 0;
        uint fill = cons_prepared + playing;
        uint8_t cons_pct = (uint8_t)(fill * 100 / consumer_capacity);
        if (cons_pct < spdif_consumer_min_fill_pct[i]) spdif_consumer_min_fill_pct[i] = cons_pct;
        if (cons_pct > spdif_consumer_max_fill_pct[i]) spdif_consumer_max_fill_pct[i] = cons_pct;
        if (i == 0) spdif0_consumer_fill = (uint8_t)fill;
    }

    if (pdm_enabled) {
        uint8_t dma_pct = pdm_get_dma_fill_pct();
        if (dma_pct < pdm_dma_min_fill_pct) pdm_dma_min_fill_pct = dma_pct;
        if (dma_pct > pdm_dma_max_fill_pct) pdm_dma_max_fill_pct = dma_pct;

        uint8_t ring_pct = pdm_get_ring_fill_pct();
        if (ring_pct < pdm_ring_min_fill_pct) pdm_ring_min_fill_pct = ring_pct;
        if (ring_pct > pdm_ring_max_fill_pct) pdm_ring_max_fill_pct = ring_pct;
    }
}

static bool vendor_setup_request_handler(__unused struct usb_interface *interface, struct usb_setup_packet *setup) {
    setup = __builtin_assume_aligned(setup, 4);

    if (!(setup->bmRequestType & USB_DIR_IN)) {
        // Host -> Device (SET requests)
        vendor_last_request = setup->bRequest;
        vendor_last_wValue = setup->wValue;

        // Large control OUT: bulk parameter SET
        if (setup->bRequest == REQ_SET_ALL_PARAMS &&
            setup->wLength == sizeof(WireBulkParams)) {
            usb_stream_setup_transfer(&_vendor_stream, &_vendor_stream_funcs,
                                      bulk_param_buf, WIRE_BULK_BUF_SIZE,
                                      sizeof(WireBulkParams), _vendor_set_complete);
            _vendor_stream.ep = usb_get_control_out_endpoint();
            usb_start_transfer(usb_get_control_out_endpoint(), &_vendor_stream.core);
            return true;
        }

        if (setup->wLength && setup->wLength <= sizeof(vendor_rx_buf)) {
            usb_start_control_out_transfer(&_vendor_cmd_transfer_type);
            return true;
        }
        return false;

    } else {
        // Device -> Host (GET requests)
        static uint8_t resp_buf[64];

        switch (setup->bRequest) {
            case REQ_GET_PREAMP: {
                float current_db = global_preamp_db;
                memcpy(resp_buf, &current_db, 4);
                vendor_send_response(resp_buf, 4);
                return true;
            }

            case REQ_GET_DELAY: {
                uint8_t ch = (uint8_t)setup->wValue;
                if (ch < NUM_CHANNELS) {
                    memcpy(resp_buf, (void*)&channel_delays_ms[ch], 4);
                    vendor_send_response(resp_buf, 4);
                    return true;
                }
                return false;
            }

            case REQ_GET_BYPASS: {
                resp_buf[0] = bypass_master_eq ? 1 : 0;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_CHANNEL_GAIN: {
                uint8_t ch = (uint8_t)setup->wValue;
                if (ch < 3) {
                    memcpy(resp_buf, (void*)&channel_gain_db[ch], 4);
                    vendor_send_response(resp_buf, 4);
                    return true;
                }
                return false;
            }

            case REQ_GET_CHANNEL_MUTE: {
                uint8_t ch = (uint8_t)setup->wValue;
                if (ch < 3) {
                    resp_buf[0] = channel_mute[ch] ? 1 : 0;
                    vendor_send_response(resp_buf, 1);
                    return true;
                }
                return false;
            }

            case REQ_GET_LOUDNESS: {
                resp_buf[0] = loudness_enabled ? 1 : 0;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_LOUDNESS_REF: {
                float val = loudness_ref_spl;
                memcpy(resp_buf, &val, 4);
                vendor_send_response(resp_buf, 4);
                return true;
            }

            case REQ_GET_LOUDNESS_INTENSITY: {
                float val = loudness_intensity_pct;
                memcpy(resp_buf, &val, 4);
                vendor_send_response(resp_buf, 4);
                return true;
            }

            case REQ_GET_CROSSFEED: {
                resp_buf[0] = crossfeed_config.enabled ? 1 : 0;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_CROSSFEED_PRESET: {
                resp_buf[0] = crossfeed_config.preset;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_CROSSFEED_FREQ: {
                float val = crossfeed_config.custom_fc;
                memcpy(resp_buf, &val, 4);
                vendor_send_response(resp_buf, 4);
                return true;
            }

            case REQ_GET_CROSSFEED_FEED: {
                float val = crossfeed_config.custom_feed_db;
                memcpy(resp_buf, &val, 4);
                vendor_send_response(resp_buf, 4);
                return true;
            }

            case REQ_GET_CROSSFEED_ITD: {
                resp_buf[0] = crossfeed_config.itd_enabled ? 1 : 0;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_STATUS: {
                if (setup->wValue == 9) {
                    // Combined status: all peaks + CPU load + clip flags
                    // RP2350: 26 bytes (11 peaks × 2 + 2 CPU + 2 clip)
                    // RP2040: 18 bytes (7 peaks × 2 + 2 CPU + 2 clip)
                    for (int i = 0; i < NUM_CHANNELS; i++) {
                        resp_buf[i*2]     = global_status.peaks[i] & 0xFF;
                        resp_buf[i*2 + 1] = global_status.peaks[i] >> 8;
                    }
                    resp_buf[NUM_CHANNELS * 2]     = global_status.cpu0_load;
                    resp_buf[NUM_CHANNELS * 2 + 1] = global_status.cpu1_load;
                    resp_buf[NUM_CHANNELS * 2 + 2] = global_status.clip_flags & 0xFF;
                    resp_buf[NUM_CHANNELS * 2 + 3] = global_status.clip_flags >> 8;
                    vendor_send_response(resp_buf, NUM_CHANNELS * 2 + 4);
                    return true;
                }

                uint32_t resp = 0;
                switch (setup->wValue) {
                    case 0: resp = (uint32_t)global_status.peaks[0] | ((uint32_t)global_status.peaks[1] << 16); break;
                    case 1: resp = (uint32_t)global_status.peaks[2] | ((uint32_t)global_status.peaks[3] << 16); break;
                    case 2: resp = (uint32_t)global_status.peaks[4] | ((uint32_t)global_status.cpu0_load << 16) | ((uint32_t)global_status.cpu1_load << 24); break;
                    case 3: resp = pdm_ring_overruns; break;
                    case 4: resp = pdm_ring_underruns; break;
                    case 5: resp = pdm_dma_overruns; break;
                    case 6: resp = pdm_dma_underruns; break;
                    case 7: resp = spdif_overruns; break;
                    case 8: resp = spdif_underruns; break;
                    case 10: resp = usb_audio_packets; break;
                    case 11: resp = usb_audio_alt_set; break;
                    case 12: resp = usb_audio_mounted; break;
                    case 13: resp = clock_get_hz(clk_sys); break;  // System clock frequency in Hz
                    case 14: resp = vreg_voltage_to_mv(vreg_get_voltage()); break;  // Core voltage in mV
                    case 15: resp = audio_state.freq; break;  // Sample rate in Hz
                    case 16: resp = (uint32_t)read_temperature_cdeg(); break;  // Temperature in centi-degrees C
                }
                usb_start_tiny_control_in_transfer(resp, 4);
                return true;
            }

            case REQ_SAVE_PARAMS: {
                int result = flash_save_params();
                usb_start_tiny_control_in_transfer(result, 1);
                return true;
            }

            case REQ_LOAD_PARAMS: {
                // flash_load_params() routes through preset_load(), which
                // handles filter recalculation, delay updates, and mute
                // internally — no need to duplicate here.
                int result = flash_load_params();
                usb_start_tiny_control_in_transfer(result, 1);
                return true;
            }

            case REQ_FACTORY_RESET: {
                flash_factory_reset();
                dsp_recalculate_all_filters((float)audio_state.freq);
                dsp_update_delay_samples((float)audio_state.freq);
                loudness_recompute_pending = true;
                usb_start_tiny_control_in_transfer(FLASH_OK, 1);
                return true;
            }

            case REQ_GET_EQ_PARAM: {
                uint8_t channel = (setup->wValue >> 8) & 0xFF;
                uint8_t band = (setup->wValue >> 4) & 0x0F;
                uint8_t param = setup->wValue & 0x0F;
                if (channel < NUM_CHANNELS && band < channel_band_counts[channel]) {
                    uint32_t val_to_send = 0;
                    EqParamPacket *p = &filter_recipes[channel][band];
                    switch (param) {
                        case 0: val_to_send = (uint32_t)p->type; break;
                        case 1: memcpy(&val_to_send, &p->freq, 4); break;
                        case 2: memcpy(&val_to_send, &p->Q, 4); break;
                        case 3: memcpy(&val_to_send, &p->gain_db, 4); break;
                    }
                    usb_start_tiny_control_in_transfer(val_to_send, 4);
                    return true;
                }
                return false;
            }

            // Matrix Mixer GET commands
            case REQ_GET_MATRIX_ROUTE: {
                // wValue = (input << 8) | output
                uint8_t input = (setup->wValue >> 8) & 0xFF;
                uint8_t output = setup->wValue & 0xFF;
                if (input < NUM_INPUT_CHANNELS && output < NUM_OUTPUT_CHANNELS) {
                    MatrixCrosspoint *xp = &matrix_mixer.crosspoints[input][output];
                    MatrixRoutePacket pkt = {
                        .input = input,
                        .output = output,
                        .enabled = xp->enabled,
                        .phase_invert = xp->phase_invert,
                        .gain_db = xp->gain_db
                    };
                    memcpy(resp_buf, &pkt, sizeof(pkt));
                    vendor_send_response(resp_buf, sizeof(pkt));
                    return true;
                }
                return false;
            }

            case REQ_GET_OUTPUT_ENABLE: {
                uint8_t out = (uint8_t)setup->wValue;
                if (out < NUM_OUTPUT_CHANNELS) {
                    resp_buf[0] = matrix_mixer.outputs[out].enabled;
                    vendor_send_response(resp_buf, 1);
                    return true;
                }
                return false;
            }

            case REQ_GET_OUTPUT_GAIN: {
                uint8_t out = (uint8_t)setup->wValue;
                if (out < NUM_OUTPUT_CHANNELS) {
                    memcpy(resp_buf, &matrix_mixer.outputs[out].gain_db, 4);
                    vendor_send_response(resp_buf, 4);
                    return true;
                }
                return false;
            }

            case REQ_GET_OUTPUT_MUTE: {
                uint8_t out = (uint8_t)setup->wValue;
                if (out < NUM_OUTPUT_CHANNELS) {
                    resp_buf[0] = matrix_mixer.outputs[out].mute;
                    vendor_send_response(resp_buf, 1);
                    return true;
                }
                return false;
            }

            case REQ_GET_OUTPUT_DELAY: {
                uint8_t out = (uint8_t)setup->wValue;
                if (out < NUM_OUTPUT_CHANNELS) {
                    memcpy(resp_buf, &matrix_mixer.outputs[out].delay_ms, 4);
                    vendor_send_response(resp_buf, 4);
                    return true;
                }
                return false;
            }

            case REQ_GET_CORE1_MODE: {
                resp_buf[0] = (uint8_t)core1_mode;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_CORE1_CONFLICT: {
                // wValue = proposed output index to enable
                // Returns 1 if enabling it would conflict, 0 if OK
                uint8_t out = (uint8_t)setup->wValue;
                uint8_t conflict = 0;
                if (out < NUM_OUTPUT_CHANNELS) {
                    bool is_pdm = (out == NUM_OUTPUT_CHANNELS - 1);
                    bool is_core1_eq = (out >= CORE1_EQ_FIRST_OUTPUT && out <= CORE1_EQ_LAST_OUTPUT);
                    if (is_pdm) {
                        for (int i = CORE1_EQ_FIRST_OUTPUT; i <= CORE1_EQ_LAST_OUTPUT; i++) {
                            if (matrix_mixer.outputs[i].enabled) { conflict = 1; break; }
                        }
                    } else if (is_core1_eq) {
                        if (matrix_mixer.outputs[NUM_OUTPUT_CHANNELS - 1].enabled) conflict = 1;
                    }
                }
                resp_buf[0] = conflict;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_SET_OUTPUT_PIN: {
                // wValue = (new_pin << 8) | output_index
                uint8_t out_idx = setup->wValue & 0xFF;
                uint8_t new_pin = (setup->wValue >> 8) & 0xFF;
                uint8_t status;

                if (out_idx >= NUM_PIN_OUTPUTS) {
                    status = PIN_CONFIG_INVALID_OUTPUT;
                } else if (!is_valid_gpio_pin(new_pin)) {
                    status = PIN_CONFIG_INVALID_PIN;
                } else if (is_pin_in_use(new_pin, out_idx)) {
                    status = PIN_CONFIG_PIN_IN_USE;
                } else if (new_pin == output_pins[out_idx]) {
                    // No-op: pin unchanged
                    status = PIN_CONFIG_SUCCESS;
                } else if (out_idx < NUM_SPDIF_INSTANCES) {
                    // SPDIF output: disable → change pin → re-enable
                    audio_spdif_instance_t *inst = spdif_instance_ptrs[out_idx];
                    audio_spdif_set_enabled(inst, false);
                    audio_spdif_change_pin(inst, new_pin);
                    audio_spdif_set_enabled(inst, true);
                    output_pins[out_idx] = new_pin;
                    status = PIN_CONFIG_SUCCESS;
                } else {
                    // PDM output (out_idx == 4): must be disabled first
                    if (pdm_enabled || core1_mode == CORE1_MODE_PDM) {
                        status = PIN_CONFIG_OUTPUT_ACTIVE;
                    } else {
                        pdm_change_pin(new_pin);
                        output_pins[out_idx] = new_pin;
                        status = PIN_CONFIG_SUCCESS;
                    }
                }

                resp_buf[0] = status;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_OUTPUT_PIN: {
                uint8_t out_idx = (uint8_t)setup->wValue;
                if (out_idx < NUM_PIN_OUTPUTS) {
                    resp_buf[0] = output_pins[out_idx];
                    vendor_send_response(resp_buf, 1);
                    return true;
                }
                return false;
            }

            case REQ_GET_SERIAL: {
                memcpy(resp_buf, usb_descriptor_str_serial, 16);
                vendor_send_response(resp_buf, 16);
                return true;
            }

            case REQ_GET_PLATFORM: {
                resp_buf[0] = PLATFORM_RP2040;
#if PICO_RP2350
                resp_buf[0] = PLATFORM_RP2350;
#endif
                resp_buf[1] = (uint8_t)(FW_VERSION_BCD >> 8);    // major
                resp_buf[2] = (uint8_t)(FW_VERSION_BCD & 0xFF);  // minor.patch BCD
                resp_buf[3] = NUM_OUTPUT_CHANNELS;
                vendor_send_response(resp_buf, 4);
                return true;
            }

            case REQ_CLEAR_CLIPS: {
                // Read-then-clear: return the clip flags that were set, then reset
                uint16_t flags = global_status.clip_flags;
                global_status.clip_flags = 0;
                usb_start_tiny_control_in_transfer(flags, 2);
                return true;
            }

            // --- Preset Commands ---

            case REQ_PRESET_SAVE: {
                // wValue = slot index (0-9).  Saves current live state.
                uint8_t slot = (uint8_t)setup->wValue;
                uint8_t result = preset_save(slot);
                resp_buf[0] = result;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_PRESET_LOAD: {
                // wValue = slot index (0-9).  Loads slot into live state.
                uint8_t slot = (uint8_t)setup->wValue;
                uint8_t result = preset_load(slot);
                resp_buf[0] = result;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_PRESET_DELETE: {
                // wValue = slot index (0-9).  Erases the slot.
                uint8_t slot = (uint8_t)setup->wValue;
                uint8_t result = preset_delete(slot);
                resp_buf[0] = result;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_PRESET_GET_NAME: {
                // wValue = slot index.  Returns 32-byte NUL-terminated name.
                uint8_t slot = (uint8_t)setup->wValue;
                char name[PRESET_NAME_LEN];
                uint8_t result = preset_get_name(slot, name);
                if (result == PRESET_OK) {
                    memcpy(resp_buf, name, PRESET_NAME_LEN);
                    vendor_send_response(resp_buf, PRESET_NAME_LEN);
                    return true;
                }
                return false;
            }

            case REQ_PRESET_GET_DIR: {
                // Returns 6-byte directory summary:
                //   [0-1] slot_occupied bitmask (little-endian u16)
                //   [2]   startup_mode
                //   [3]   default_slot
                //   [4]   last_active_slot
                //   [5]   include_pins
                uint16_t occupied;
                uint8_t mode, def_slot, last_active, inc_pins;
                preset_get_directory(&occupied, &mode, &def_slot,
                                     &last_active, &inc_pins);
                resp_buf[0] = occupied & 0xFF;
                resp_buf[1] = occupied >> 8;
                resp_buf[2] = mode;
                resp_buf[3] = def_slot;
                resp_buf[4] = last_active;
                resp_buf[5] = inc_pins;
                vendor_send_response(resp_buf, 6);
                return true;
            }

            case REQ_PRESET_GET_STARTUP: {
                // Returns 3 bytes: startup_mode, default_slot, last_active
                uint16_t occupied;
                uint8_t mode, def_slot, last_active, inc_pins;
                preset_get_directory(&occupied, &mode, &def_slot,
                                     &last_active, &inc_pins);
                resp_buf[0] = mode;
                resp_buf[1] = def_slot;
                resp_buf[2] = last_active;
                vendor_send_response(resp_buf, 3);
                return true;
            }

            case REQ_PRESET_GET_INCLUDE_PINS: {
                uint16_t occupied;
                uint8_t mode, def_slot, last_active, inc_pins;
                preset_get_directory(&occupied, &mode, &def_slot,
                                     &last_active, &inc_pins);
                resp_buf[0] = inc_pins;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_PRESET_GET_ACTIVE: {
                resp_buf[0] = preset_get_active();
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_CHANNEL_NAME: {
                uint8_t ch = setup->wValue & 0xFF;
                if (ch < NUM_CHANNELS) {
                    vendor_send_response(channel_names[ch], PRESET_NAME_LEN);
                    return true;
                }
                return false;
            }

            case REQ_GET_ALL_PARAMS: {
                bulk_params_collect((WireBulkParams *)bulk_param_buf);
                uint32_t len = sizeof(WireBulkParams);
                if (setup->wLength < len) len = setup->wLength;
                usb_stream_setup_transfer(&_vendor_stream, &_vendor_stream_funcs,
                                          bulk_param_buf, WIRE_BULK_BUF_SIZE, len,
                                          _vendor_get_complete);
                bool need_zlp = (len > 0) && ((len & 63u) == 0);
                if (need_zlp) usb_grow_transfer(&_vendor_stream.core, 1);
                _vendor_stream.ep = usb_get_control_in_endpoint();
                usb_start_transfer(usb_get_control_in_endpoint(), &_vendor_stream.core);
                return true;
            }

            case REQ_GET_BUFFER_STATS: {
                BufferStatsPacket pkt;
                memset(&pkt, 0, sizeof(pkt));
                pkt.num_spdif = NUM_SPDIF_INSTANCES;
                pkt.flags = (pdm_enabled ? 0x01 : 0) | (sync_started ? 0x02 : 0);
                pkt.sequence = buffer_stats_sequence++;

                uint consumer_capacity = SPDIF_CONSUMER_BUFFER_COUNT;

                for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
                    audio_spdif_instance_t *inst = spdif_instance_ptrs[i];
                    pkt.spdif[i].consumer_free = count_pool_free(inst->consumer_pool);
                    pkt.spdif[i].consumer_prepared = count_pool_prepared(inst->consumer_pool);
                    pkt.spdif[i].consumer_playing = (inst->playing_buffer != NULL) ? 1 : 0;
                    uint cons_fill = pkt.spdif[i].consumer_prepared + pkt.spdif[i].consumer_playing;
                    pkt.spdif[i].consumer_fill_pct = (uint8_t)(cons_fill * 100 / consumer_capacity);
                    pkt.spdif[i].consumer_min_fill_pct = spdif_consumer_min_fill_pct[i];
                    pkt.spdif[i].consumer_max_fill_pct = spdif_consumer_max_fill_pct[i];
                }

                if (pdm_enabled) {
                    pkt.pdm.dma_fill_pct = pdm_get_dma_fill_pct();
                    pkt.pdm.dma_min_fill_pct = pdm_dma_min_fill_pct;
                    pkt.pdm.dma_max_fill_pct = pdm_dma_max_fill_pct;
                    pkt.pdm.ring_fill_pct = pdm_get_ring_fill_pct();
                    pkt.pdm.ring_min_fill_pct = pdm_ring_min_fill_pct;
                    pkt.pdm.ring_max_fill_pct = pdm_ring_max_fill_pct;
                }

                memcpy(resp_buf, &pkt, sizeof(pkt));
                vendor_send_response(resp_buf, sizeof(pkt));
                return true;
            }

            case REQ_RESET_BUFFER_STATS: {
                uint16_t flags = setup->wValue;
                if (flags & 0x01) {
                    reset_buffer_watermarks();
                }
                resp_buf[0] = 1;
                vendor_send_response(resp_buf, 1);
                return true;
            }
        }

        return false;
    }
}

// ----------------------------------------------------------------------------
// DEVICE-LEVEL SETUP REQUEST HANDLER (WCID / MS OS descriptors)
// ----------------------------------------------------------------------------

static bool device_setup_request_handler(struct usb_device *dev, struct usb_setup_packet *setup) {
    (void)dev;
    setup = __builtin_assume_aligned(setup, 4);

    // Intercept GET_DESCRIPTOR(String, 0xEE) — MS OS String Descriptor
    // The default string handler only converts ASCII→UTF-16, but the MS OS
    // string descriptor is a fixed 18-byte binary blob that must be returned raw.
    if (!(setup->bmRequestType & USB_REQ_TYPE_TYPE_MASK) &&
        (setup->bmRequestType & USB_DIR_IN) &&
        setup->bRequest == USB_REQUEST_GET_DESCRIPTOR &&
        setup->wValue == 0x03EE) {
        uint16_t len = setup->wLength < MS_OS_STRING_DESC_LEN ? setup->wLength : MS_OS_STRING_DESC_LEN;
        vendor_send_response(ms_os_string_descriptor, len);
        return true;
    }

    // Handle Microsoft OS vendor requests (WCID compat ID and ext props)
    if ((setup->bmRequestType & USB_REQ_TYPE_TYPE_MASK) == USB_REQ_TYPE_TYPE_VENDOR &&
        setup->bRequest == MS_VENDOR_CODE) {
        switch (setup->wIndex) {
            case 0x0004: {
                uint16_t len = setup->wLength < MS_COMPAT_ID_DESC_LEN ? setup->wLength : MS_COMPAT_ID_DESC_LEN;
                vendor_send_response(ms_compat_id_descriptor, len);
                return true;
            }
            case 0x0005: {
                uint16_t len = setup->wLength < MS_EXT_PROP_DESC_LEN ? setup->wLength : MS_EXT_PROP_DESC_LEN;
                vendor_send_response(ms_ext_prop_descriptor, len);
                return true;
            }
        }
    }

    return false;
}

// ----------------------------------------------------------------------------
// STRING DESCRIPTOR CALLBACK (with MS OS string at index 0xEE)
// ----------------------------------------------------------------------------

static const char *_get_descriptor_string(uint index) {
    if (index >= 1 && index <= count_of(descriptor_strings)) {
        return descriptor_strings[index - 1];
    }
    return "";
}

// ----------------------------------------------------------------------------
// INIT
// ----------------------------------------------------------------------------

// S/PDIF Instances
static audio_spdif_instance_t spdif_instance_1 = {0};  // Out 1-2
static audio_spdif_instance_t spdif_instance_2 = {0};  // Out 3-4
#if PICO_RP2350
static audio_spdif_instance_t spdif_instance_3 = {0};  // Out 5-6
static audio_spdif_instance_t spdif_instance_4 = {0};  // Out 7-8
#endif

struct audio_spdif_config spdif_config_1 = {
    .pin = PICO_AUDIO_SPDIF_PIN,  // GPIO 6
    .dma_channel = 0,
    .pio_sm = 0,
    .pio = PICO_AUDIO_SPDIF_PIO,
    .dma_irq = PICO_AUDIO_SPDIF_DMA_IRQ,
};

struct audio_spdif_config spdif_config_2 = {
    .pin = PICO_SPDIF_PIN_2,  // GPIO 7
    .dma_channel = 1,
    .pio_sm = 1,
    .pio = PICO_AUDIO_SPDIF_PIO,
    .dma_irq = PICO_AUDIO_SPDIF_DMA_IRQ,
};

#if PICO_RP2350
struct audio_spdif_config spdif_config_3 = {
    .pin = PICO_SPDIF_PIN_3,  // GPIO 8
    .dma_channel = 2,
    .pio_sm = 2,
    .pio = PICO_AUDIO_SPDIF_PIO,
    .dma_irq = PICO_AUDIO_SPDIF_DMA_IRQ,
};

struct audio_spdif_config spdif_config_4 = {
    .pin = PICO_SPDIF_PIN_4,  // GPIO 9
    .dma_channel = 3,
    .pio_sm = 3,
    .pio = PICO_AUDIO_SPDIF_PIO,
    .dma_irq = PICO_AUDIO_SPDIF_DMA_IRQ,
};
#endif

struct audio_buffer_format producer_format = { .format = &audio_format_48k, .sample_stride = 8 };

// Legacy aliases
#define spdif_instance spdif_instance_1
#define spdif_sub_instance spdif_instance_2
#define config spdif_config_1
#define sub_config spdif_config_2

// Initialize matrix mixer with default stereo pass-through
static void matrix_init_defaults(void) {
    memset(&matrix_mixer, 0, sizeof(matrix_mixer));

    // Stereo pass-through on first S/PDIF pair (Out 1-2)
    matrix_mixer.crosspoints[0][0].enabled = 1;     // L→Out1
    matrix_mixer.crosspoints[0][0].gain_db = 0.0f;
    matrix_mixer.crosspoints[0][0].gain_linear = 1.0f;

    matrix_mixer.crosspoints[1][1].enabled = 1;     // R→Out2
    matrix_mixer.crosspoints[1][1].gain_db = 0.0f;
    matrix_mixer.crosspoints[1][1].gain_linear = 1.0f;

    // Enable first stereo pair only by default
    matrix_mixer.outputs[0].enabled = 1;
    matrix_mixer.outputs[0].gain_linear = 1.0f;
    matrix_mixer.outputs[1].enabled = 1;
    matrix_mixer.outputs[1].gain_linear = 1.0f;

    // All other outputs disabled by default (saves CPU)
    for (int out = 2; out < NUM_OUTPUT_CHANNELS; out++) {
        matrix_mixer.outputs[out].enabled = 0;
        matrix_mixer.outputs[out].gain_linear = 1.0f;
    }
}

void usb_sound_card_init(void) {
    // Initialize matrix mixer defaults
    matrix_init_defaults();
    reset_buffer_watermarks();

    // S/PDIF Setup (this must happen before USB init to claim DMA channels)
    producer_pool_1 = audio_new_producer_pool(&producer_format, AUDIO_BUFFER_COUNT, 192);
    producer_pool_2 = audio_new_producer_pool(&producer_format, AUDIO_BUFFER_COUNT, 192);
#if PICO_RP2350
    producer_pool_3 = audio_new_producer_pool(&producer_format, AUDIO_BUFFER_COUNT, 192);
    producer_pool_4 = audio_new_producer_pool(&producer_format, AUDIO_BUFFER_COUNT, 192);
#endif

    // Setup S/PDIF instances
    audio_spdif_setup(&spdif_instance_1, &audio_format_48k, &spdif_config_1);
    audio_spdif_connect_extra(&spdif_instance_1, producer_pool_1, false, SPDIF_CONSUMER_BUFFER_COUNT, NULL);

    audio_spdif_setup(&spdif_instance_2, &audio_format_48k, &spdif_config_2);
    audio_spdif_connect_extra(&spdif_instance_2, producer_pool_2, false, SPDIF_CONSUMER_BUFFER_COUNT, NULL);

#if PICO_RP2350
    audio_spdif_setup(&spdif_instance_3, &audio_format_48k, &spdif_config_3);
    audio_spdif_connect_extra(&spdif_instance_3, producer_pool_3, false, SPDIF_CONSUMER_BUFFER_COUNT, NULL);

    audio_spdif_setup(&spdif_instance_4, &audio_format_48k, &spdif_config_4);
    audio_spdif_connect_extra(&spdif_instance_4, producer_pool_4, false, SPDIF_CONSUMER_BUFFER_COUNT, NULL);
#endif

    // Populate instance pointer array for pin config commands
    spdif_instance_ptrs[0] = &spdif_instance_1;
    spdif_instance_ptrs[1] = &spdif_instance_2;
#if PICO_RP2350
    spdif_instance_ptrs[2] = &spdif_instance_3;
    spdif_instance_ptrs[3] = &spdif_instance_4;
#endif

    irq_set_priority(DMA_IRQ_0 + PICO_AUDIO_SPDIF_DMA_IRQ, PICO_HIGHEST_IRQ_PRIORITY);

    // Start all outputs synchronized
#if PICO_RP2350
    audio_spdif_instance_t *spdif_all[] = {
        &spdif_instance_1, &spdif_instance_2, &spdif_instance_3, &spdif_instance_4
    };
    audio_spdif_enable_sync(spdif_all, 4);
#else
    audio_spdif_instance_t *spdif_all[] = {
        &spdif_instance_1, &spdif_instance_2
    };
    audio_spdif_enable_sync(spdif_all, 2);
#endif

    // Initialize pico-extras USB device with 3 interfaces: AC, AS, Vendor

    // Audio Control interface
    usb_interface_init(&ac_interface, &audio_device_config.ac_interface, NULL, 0, true);
    ac_interface.setup_request_handler = ac_setup_request_handler;

    // Audio Streaming interface with OUT + sync endpoints
    static struct usb_endpoint *const op_endpoints[] = {
        &ep_op_out, &ep_op_sync
    };
    usb_interface_init(&as_op_interface, &audio_device_config.as_op_interface, op_endpoints, count_of(op_endpoints), true);
    as_op_interface.set_alternate_handler = as_set_alternate;
    ep_op_out.setup_request_handler = _as_setup_request_handler;
    as_transfer.type = &as_transfer_type;
    usb_set_default_transfer(&ep_op_out, &as_transfer);
    as_sync_transfer.type = &as_sync_transfer_type;
    usb_set_default_transfer(&ep_op_sync, &as_sync_transfer);

    // Vendor interface (control-only, no endpoints)
    usb_interface_init(&vendor_interface, &audio_device_config.vendor_interface, NULL, 0, true);
    vendor_interface.setup_request_handler = vendor_setup_request_handler;

    // Initialize USB device
    static struct usb_interface *const boot_device_interfaces[] = {
        &ac_interface,
        &as_op_interface,
        &vendor_interface,
    };
    struct usb_device *device = usb_device_init(&boot_device_descriptor, &audio_device_config.descriptor,
        boot_device_interfaces, count_of(boot_device_interfaces),
        _get_descriptor_string);
    assert(device);
    device->setup_request_handler = device_setup_request_handler;

    // Initialize DSP
    dsp_init_default_filters();
    dsp_recalculate_all_filters(48000.0f);
    audio_set_volume(DEFAULT_VOLUME);
    _audio_reconfigure();

    // Initialize Core 1 EQ worker pointer to shared output buffer
    core1_eq_work.buf_out = buf_out;

    // Initialize ADC for temperature sensor
    adc_init();
    adc_set_temp_sensor_enabled(true);

    usb_device_start();
}
