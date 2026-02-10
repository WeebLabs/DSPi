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

static Biquad loudness_biquads[2][LOUDNESS_BIQUAD_COUNT];  // [0]=Left, [1]=Right
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

// Shared output buffer — file scope so Core 1 can access via pointer
static float buf_out[NUM_OUTPUT_CHANNELS][192];

// Sync State
volatile uint64_t total_samples_produced = 0;
volatile uint64_t start_time_us = 0;
volatile bool sync_started = false;
static volatile uint64_t last_packet_time_us = 0;
#define AUDIO_GAP_THRESHOLD_US 50000  // 50ms - reset sync if packets stop this long

// Audio Pools (4 S/PDIF stereo pairs)
struct audio_buffer_pool *producer_pool_1 = NULL;  // S/PDIF 1 (Out 1-2)
struct audio_buffer_pool *producer_pool_2 = NULL;  // S/PDIF 2 (Out 3-4)
struct audio_buffer_pool *producer_pool_3 = NULL;  // S/PDIF 3 (Out 5-6)
struct audio_buffer_pool *producer_pool_4 = NULL;  // S/PDIF 4 (Out 7-8)
struct audio_format audio_format_48k = { .format = AUDIO_BUFFER_FORMAT_PCM_S16, .sample_freq = 48000, .channel_count = 2 };

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
#define CENTER_VOLUME_INDEX 91
static uint16_t db_to_vol[91] = {
    0x0001, 0x0001, 0x0001, 0x0001, 0x0001, 0x0001, 0x0002, 0x0002, 0x0002, 0x0002, 0x0003, 0x0003, 0x0004, 0x0004, 0x0005, 0x0005,
    0x0006, 0x0007, 0x0008, 0x0009, 0x000a, 0x000b, 0x000d, 0x000e, 0x0010, 0x0012, 0x0014, 0x0017, 0x001a, 0x001d, 0x0020, 0x0024,
    0x0029, 0x002e, 0x0033, 0x003a, 0x0041, 0x0049, 0x0052, 0x005c, 0x0067, 0x0074, 0x0082, 0x0092, 0x00a4, 0x00b8, 0x00ce, 0x00e7,
    0x0104, 0x0124, 0x0147, 0x016f, 0x019c, 0x01ce, 0x0207, 0x0246, 0x028d, 0x02dd, 0x0337, 0x039b, 0x040c, 0x048a, 0x0518, 0x05b7,
    0x066a, 0x0732, 0x0813, 0x090f, 0x0a2a, 0x0b68, 0x0ccc, 0x0e5c, 0x101d, 0x1214, 0x1449, 0x16c3, 0x198a, 0x1ca7, 0x2026, 0x2413,
    0x287a, 0x2d6a, 0x32f5, 0x392c, 0x4026, 0x47fa, 0x50c3, 0x5a9d, 0x65ac, 0x7214, 0x7fff
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
    if (volume >= 91 * 256) volume = 91 * 256 - 1;
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
    uint32_t start_time = time_us_32();
    uint32_t core1_wait_us = 0;

    // Detect SPDIF underrun: USB packets should arrive every ~1ms
    static uint32_t last_packet_time = 0;
    if (last_packet_time > 0) {
        uint32_t gap = start_time - last_packet_time;
        if (gap > 2000 && gap < 50000) {  // 2ms-50ms gap = underrun
            spdif_underruns++;
        }
    }
    last_packet_time = start_time;

    // Get audio buffers for all 4 S/PDIF outputs
    struct audio_buffer* audio_buf[4] = {NULL, NULL, NULL, NULL};
    if (producer_pool_1) audio_buf[0] = take_audio_buffer(producer_pool_1, false);
    if (producer_pool_2) audio_buf[1] = take_audio_buffer(producer_pool_2, false);
    if (producer_pool_3) audio_buf[2] = take_audio_buffer(producer_pool_3, false);
    if (producer_pool_4) audio_buf[3] = take_audio_buffer(producer_pool_4, false);

    uint32_t sample_count = data_len / 4;  // 2 channels * 2 bytes per sample

    for (int b = 0; b < 4; b++) {
        if (audio_buf[b]) {
            audio_buf[b]->sample_count = sample_count;
        } else if (matrix_mixer.outputs[b*2].enabled || matrix_mixer.outputs[b*2+1].enabled) {
            spdif_overruns++;
        }
    }

    uint64_t now_us = time_us_64();

    // Detect audio restart after gap - reset sync state and pre-fill pool
    if (sync_started && last_packet_time_us > 0 &&
        (now_us - last_packet_time_us) > AUDIO_GAP_THRESHOLD_US) {
        sync_started = false;
        total_samples_produced = 0;

        // Pre-fill with 2 silent buffers to prevent underrun on restart
        for (int i = 0; i < 2; i++) {
            struct audio_buffer *sb = take_audio_buffer(producer_pool_1, false);
            if (sb) {
                int16_t *out = (int16_t *)sb->buffer->bytes;
                for (uint32_t j = 0; j < 192; j++) {
                    out[j * 2] = 0;
                    out[j * 2 + 1] = 0;
                }
                sb->sample_count = 192;
                give_audio_buffer(producer_pool_1, sb);
            }
        }
    }
    last_packet_time_us = now_us;

    if (!sync_started) {
        start_time_us = now_us;
        sync_started = true;
    }
    total_samples_produced += sample_count;

    const int16_t *in = (const int16_t *)data;

#if PICO_RP2350
    // ------------------------------------------------------------------------
    // RP2350 FLOAT PIPELINE WITH MATRIX MIXER
    // ------------------------------------------------------------------------
    const float inv_32768 = 1.0f / 32768.0f;

    float vol_mul = (float)audio_state.vol_mul * inv_32768;
    float preamp = global_preamp_linear;
    bool is_bypassed = bypass_master_eq;

    // Snapshot loudness state for this packet
    bool loud_on = loudness_enabled;
    const LoudnessCoeffs *loud_coeffs = current_loudness_coeffs;

    float peak_ml = 0, peak_mr = 0, peak_ol = 0, peak_or = 0, peak_sub = 0;

    // Pre-compute PDM scale factor
    const float pdm_scale = (float)(1 << 28);

    // Static buffers to avoid stack overflow (~8KB would be too much for stack)
    static float buf_l[192], buf_r[192];

    // ========== PASS 1: Input conversion + Preamp + Loudness ==========
    for (uint32_t i = 0; i < sample_count; i++) {
        buf_l[i] = (float)in[i*2] * inv_32768 * preamp;
        buf_r[i] = (float)in[i*2+1] * inv_32768 * preamp;
    }

    // Loudness compensation
    if (loud_on && loud_coeffs) {
        for (uint32_t i = 0; i < sample_count; i++) {
            float raw_left = buf_l[i];
            float raw_right = buf_r[i];
            for (int j = 0; j < LOUDNESS_BIQUAD_COUNT; j++) {
                const LoudnessCoeffs *lc = &loud_coeffs[j];
                if (lc->bypass) continue;
                Biquad *bq = &loudness_biquads[0][j];
                double rd = dcp_dadd(dcp_f2d(lc->b0 * raw_left), bq->s1);
                float rf = dcp_d2f(rd);
                float v1 = lc->b1 * raw_left - lc->a1 * rf;
                bq->s1 = dcp_dadd(dcp_f2d(v1), bq->s2);
                bq->s2 = dcp_f2d(lc->b2 * raw_left - lc->a2 * rf);
                raw_left = rf;
            }
            for (int j = 0; j < LOUDNESS_BIQUAD_COUNT; j++) {
                const LoudnessCoeffs *lc = &loud_coeffs[j];
                if (lc->bypass) continue;
                Biquad *bq = &loudness_biquads[1][j];
                double rd = dcp_dadd(dcp_f2d(lc->b0 * raw_right), bq->s1);
                float rf = dcp_d2f(rd);
                float v1 = lc->b1 * raw_right - lc->a1 * rf;
                bq->s1 = dcp_dadd(dcp_f2d(v1), bq->s2);
                bq->s2 = dcp_f2d(lc->b2 * raw_right - lc->a2 * rf);
                raw_right = rf;
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
        if (!matrix_mixer.outputs[out].enabled) continue;

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

    // ========== PASS 5: Per-Output EQ + Gain (block-based, dual-core) ==========
    if (core1_mode == CORE1_MODE_EQ_WORKER) {
        // Dispatch outputs CORE1_EQ_FIRST_OUTPUT..CORE1_EQ_LAST_OUTPUT to Core 1
        core1_eq_work.sample_count = sample_count;
        core1_eq_work.vol_mul = vol_mul;
        core1_eq_work.work_done = false;
        __dmb();
        core1_eq_work.work_ready = true;
        __sev();

        // Core 0: process outputs 0..CORE1_EQ_FIRST_OUTPUT-1
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

        // Also process PDM output (last) on Core 0 if enabled
        {
            int out = NUM_OUTPUT_CHANNELS - 1;
            if (matrix_mixer.outputs[out].enabled) {
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
        }

        // Wait for Core 1 to finish outputs CORE1_EQ_FIRST_OUTPUT..CORE1_EQ_LAST_OUTPUT
        uint32_t wait_start = time_us_32();
        while (!core1_eq_work.work_done) {
            __wfe();
        }
        __dmb();
        core1_wait_us = time_us_32() - wait_start;
    } else {
        // Single-core: process all outputs on Core 0
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
    }

    // ========== PASS 6: Delay (only outputs with non-zero delay) ==========
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

    // ========== PASS 7: Output to S/PDIF + PDM ==========
    // Peaks (first stereo pair only)
    for (uint32_t i = 0; i < sample_count; i++) {
        float abs_ol = fabsf(buf_out[0][i]); if (abs_ol > peak_ol) peak_ol = abs_ol;
        float abs_or = fabsf(buf_out[1][i]); if (abs_or > peak_or) peak_or = abs_or;
    }

    // S/PDIF conversion — skip pairs where neither output is enabled
    for (int pair = 0; pair < 4; pair++) {
        if (!audio_buf[pair]) continue;
        int left_ch = pair * 2;
        int right_ch = pair * 2 + 1;
        if (!matrix_mixer.outputs[left_ch].enabled && !matrix_mixer.outputs[right_ch].enabled) {
            // Pair disabled — write silence so S/PDIF DMA has valid data
            memset(audio_buf[pair]->buffer->bytes, 0, sample_count * 4);
            continue;
        }
        int16_t *out_ptr = (int16_t *)audio_buf[pair]->buffer->bytes;
        for (uint32_t i = 0; i < sample_count; i++) {
            float dl = fmaxf(-1.0f, fminf(1.0f, buf_out[left_ch][i]));
            float dr = fmaxf(-1.0f, fminf(1.0f, buf_out[right_ch][i]));
            out_ptr[i*2]     = (int16_t)(dl * 32767.0f);
            out_ptr[i*2+1]   = (int16_t)(dr * 32767.0f);
        }
    }

#if ENABLE_SUB
    if (matrix_mixer.outputs[NUM_OUTPUT_CHANNELS-1].enabled) {
        for (uint32_t i = 0; i < sample_count; i++) {
            float abs_sub = fabsf(buf_out[NUM_OUTPUT_CHANNELS-1][i]);
            if (abs_sub > peak_sub) peak_sub = abs_sub;
        }
        for (uint32_t i = 0; i < sample_count; i++) {
            int32_t pdm_sample_q28 = (int32_t)(buf_out[NUM_OUTPUT_CHANNELS-1][i] * pdm_scale);
            pdm_push_sample(pdm_sample_q28, false);
        }
    }
#endif

    // Convert peaks 0.0-1.0 to Q15-ish uint16 for status report
    global_status.peaks[0] = (uint16_t)(fminf(1.0f, peak_ml) * 32767.0f);
    global_status.peaks[1] = (uint16_t)(fminf(1.0f, peak_mr) * 32767.0f);
    global_status.peaks[2] = (uint16_t)(fminf(1.0f, peak_ol) * 32767.0f);
    global_status.peaks[3] = (uint16_t)(fminf(1.0f, peak_or) * 32767.0f);
    global_status.peaks[4] = (uint16_t)(fminf(1.0f, peak_sub) * 32767.0f);

#else
    // ------------------------------------------------------------------------
    // RP2040 FIXED-POINT PIPELINE WITH MATRIX MIXER
    // Note: Matrix mixer on RP2040 only supports first 3 outputs (L/R/Sub)
    // due to CPU constraints. Full 9-output matrix requires RP2350.
    // ------------------------------------------------------------------------
    int32_t vol_mul = audio_state.vol_mul;
    int32_t preamp = global_preamp_mul;
    bool is_bypassed = bypass_master_eq;

    // Snapshot loudness state for this packet
    bool loud_on = loudness_enabled;
    const LoudnessCoeffs *loud_coeffs = current_loudness_coeffs;

    int32_t peak_ml = 0, peak_mr = 0, peak_ol = 0, peak_or = 0, peak_sub = 0;

    for (uint32_t i = 0; i < sample_count; i++) {
        int16_t raw_left_16 = in[i*2];
        int16_t raw_right_16 = in[i*2+1];

        int32_t raw_left_32 = (int32_t)raw_left_16 << 14;
        int32_t raw_right_32 = (int32_t)raw_right_16 << 14;

        raw_left_32  = clip_s64_to_s32(((int64_t)raw_left_32 * preamp) >> 28);
        raw_right_32 = clip_s64_to_s32(((int64_t)raw_right_32 * preamp) >> 28);

        // Loudness compensation
        if (loud_on && loud_coeffs) {
            for (int j = 0; j < LOUDNESS_BIQUAD_COUNT; j++) {
                const LoudnessCoeffs *lc = &loud_coeffs[j];
                if (lc->bypass) continue;
                Biquad *bq = &loudness_biquads[0][j];
                int32_t result = (int32_t)(((int64_t)lc->b0 * raw_left_32) >> 28) + bq->s1;
                bq->s1 = (int32_t)(((int64_t)lc->b1 * raw_left_32) >> 28)
                        - (int32_t)(((int64_t)lc->a1 * result) >> 28) + bq->s2;
                bq->s2 = (int32_t)(((int64_t)lc->b2 * raw_left_32) >> 28)
                        - (int32_t)(((int64_t)lc->a2 * result) >> 28);
                raw_left_32 = result;
            }
            for (int j = 0; j < LOUDNESS_BIQUAD_COUNT; j++) {
                const LoudnessCoeffs *lc = &loud_coeffs[j];
                if (lc->bypass) continue;
                Biquad *bq = &loudness_biquads[1][j];
                int32_t result = (int32_t)(((int64_t)lc->b0 * raw_right_32) >> 28) + bq->s1;
                bq->s1 = (int32_t)(((int64_t)lc->b1 * raw_right_32) >> 28)
                        - (int32_t)(((int64_t)lc->a1 * result) >> 28) + bq->s2;
                bq->s2 = (int32_t)(((int64_t)lc->b2 * raw_right_32) >> 28)
                        - (int32_t)(((int64_t)lc->a2 * result) >> 28);
                raw_right_32 = result;
            }
        }

        int32_t master_l_32, master_r_32;
        if (is_bypassed) {
            master_l_32 = raw_left_32;
            master_r_32 = raw_right_32;
        } else {
            master_l_32 = channel_bypassed[CH_MASTER_LEFT] ? raw_left_32 :
                          dsp_process_channel(filters[CH_MASTER_LEFT], raw_left_32, CH_MASTER_LEFT);
            master_r_32 = channel_bypassed[CH_MASTER_RIGHT] ? raw_right_32 :
                          dsp_process_channel(filters[CH_MASTER_RIGHT], raw_right_32, CH_MASTER_RIGHT);
        }

        if (abs(master_l_32) > peak_ml) peak_ml = abs(master_l_32);
        if (abs(master_r_32) > peak_mr) peak_mr = abs(master_r_32);

        // Crossfeed
        if (!crossfeed_bypassed) {
            crossfeed_process_stereo(&crossfeed_state, &master_l_32, &master_r_32);
        }

        // RP2040: Simple 3-output routing (L/R/Sub) using legacy gain/mute arrays
        int32_t sub_in_32 = (master_l_32 + master_r_32) >> 1;
        int32_t out_l_32 = 0, out_r_32 = 0, out_sub_32 = 0;

        out_l_32 = channel_bypassed[CH_OUT_LEFT] ? master_l_32 :
                   dsp_process_channel(filters[CH_OUT_LEFT], master_l_32, CH_OUT_LEFT);
        out_r_32 = channel_bypassed[CH_OUT_RIGHT] ? master_r_32 :
                   dsp_process_channel(filters[CH_OUT_RIGHT], master_r_32, CH_OUT_RIGHT);
#if ENABLE_SUB
        out_sub_32 = channel_bypassed[CH_OUT_SUB] ? sub_in_32 :
                     dsp_process_channel(filters[CH_OUT_SUB], sub_in_32, CH_OUT_SUB);
#endif

        // Per-channel gain and mute (legacy 3-channel)
        out_l_32   = channel_mute[0] ? 0 : (int32_t)(((int64_t)out_l_32 * channel_gain_mul[0]) >> 15);
        out_r_32   = channel_mute[1] ? 0 : (int32_t)(((int64_t)out_r_32 * channel_gain_mul[1]) >> 15);
        out_sub_32 = channel_mute[2] ? 0 : (int32_t)(((int64_t)out_sub_32 * channel_gain_mul[2]) >> 15);

        if (abs(out_l_32) > peak_ol) peak_ol = abs(out_l_32);
        if (abs(out_r_32) > peak_or) peak_or = abs(out_r_32);
        if (abs(out_sub_32) > peak_sub) peak_sub = abs(out_sub_32);

        // Master volume
        out_l_32   = (int32_t)(((int64_t)out_l_32 * vol_mul) >> 15);
        out_r_32   = (int32_t)(((int64_t)out_r_32 * vol_mul) >> 15);
        out_sub_32 = (int32_t)(((int64_t)out_sub_32 * vol_mul) >> 15);

        out_l_32 = clip_s32(out_l_32);
        out_r_32 = clip_s32(out_r_32);
        out_sub_32 = clip_s32(out_sub_32);

        // Delay (only first 3 channels on RP2040)
        delay_lines[0][delay_write_idx] = out_l_32;
        delay_lines[1][delay_write_idx] = out_r_32;
        delay_lines[2][delay_write_idx] = out_sub_32;

        int32_t delayed_l   = delay_lines[0][(delay_write_idx - channel_delay_samples[0]) & MAX_DELAY_MASK];
        int32_t delayed_r   = delay_lines[1][(delay_write_idx - channel_delay_samples[1]) & MAX_DELAY_MASK];
        int32_t delayed_sub = delay_lines[2][(delay_write_idx - channel_delay_samples[2]) & MAX_DELAY_MASK];

        delay_write_idx = (delay_write_idx + 1) & MAX_DELAY_MASK;

        // Output to first S/PDIF pair
        if (audio_buf[0]) {
            int16_t *out = (int16_t *) audio_buf[0]->buffer->bytes;
            out[i*2]     = (int16_t)(clip_s32(delayed_l + (1<<13)) >> 14);
            out[i*2+1]   = (int16_t)(clip_s32(delayed_r + (1<<13)) >> 14);
        }

#if ENABLE_SUB
        pdm_push_sample(delayed_sub, false);

        // Sub S/PDIF output (second pair, mono duplicated)
        if (audio_buf[1]) {
            int16_t *sub_out = (int16_t *) audio_buf[1]->buffer->bytes;
            int16_t sub_s16 = (int16_t)(clip_s32(delayed_sub + (1<<13)) >> 14);
            sub_out[i*2]     = sub_s16;
            sub_out[i*2+1]   = sub_s16;
        }
#endif
    }

    global_status.peaks[0] = (uint16_t)(peak_ml >> 13);
    global_status.peaks[1] = (uint16_t)(peak_mr >> 13);
    global_status.peaks[2] = (uint16_t)(peak_ol >> 13);
    global_status.peaks[3] = (uint16_t)(peak_or >> 13);
    global_status.peaks[4] = (uint16_t)(peak_sub >> 13);
#endif

    // Return all buffers
    for (int b = 0; b < 4; b++) {
        if (audio_buf[b]) {
            struct audio_buffer_pool *pool = (b == 0) ? producer_pool_1 :
                                              (b == 1) ? producer_pool_2 :
                                              (b == 2) ? producer_pool_3 : producer_pool_4;
            give_audio_buffer(pool, audio_buf[b]);
        }
    }

    uint32_t end_time = time_us_32();
    global_status.cpu0_load = (uint8_t)((end_time - start_time - core1_wait_us) / 10);
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

    // 10.14 fixed-point feedback: nominal sample rate
    uint feedback = (audio_state.freq << 14u) / 1000u;

    buffer->data[0] = feedback;
    buffer->data[1] = feedback >> 8u;
    buffer->data[2] = feedback >> 16u;

    // keep on truckin'
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
    return alt < 2;
}

// ----------------------------------------------------------------------------
// VENDOR INTERFACE HANDLER (DSPi commands via EP0 control transfers)
// ----------------------------------------------------------------------------

// Buffer for vendor SET requests
static uint8_t vendor_rx_buf[64];
static uint8_t vendor_last_request = 0;
static uint16_t vendor_last_wValue = 0;

// Derive Core 1 mode from current output enable state
static Core1Mode derive_core1_mode(void) {
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
                    if (vol >= 91 * 256) vol = 91 * 256 - 1;
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

                // Mutual exclusion interlock: PDM vs outputs 2-7
                if (want_enable) {
                    bool is_pdm = (out == NUM_OUTPUT_CHANNELS - 1);
                    bool is_core1_eq = (out >= CORE1_EQ_FIRST_OUTPUT && out <= CORE1_EQ_LAST_OUTPUT);

                    if (is_pdm) {
                        // Refuse PDM if any output 2-7 is enabled
                        for (int i = CORE1_EQ_FIRST_OUTPUT; i <= CORE1_EQ_LAST_OUTPUT; i++) {
                            if (matrix_mixer.outputs[i].enabled) goto skip_enable;
                        }
                    } else if (is_core1_eq) {
                        // Refuse output 2-7 if PDM is enabled
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

static bool vendor_setup_request_handler(__unused struct usb_interface *interface, struct usb_setup_packet *setup) {
    setup = __builtin_assume_aligned(setup, 4);

    if (!(setup->bmRequestType & USB_DIR_IN)) {
        // Host -> Device (SET requests)
        vendor_last_request = setup->bRequest;
        vendor_last_wValue = setup->wValue;

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
                    // Combined status: all peaks + CPU in one 12-byte transfer
                    resp_buf[0] = global_status.peaks[0] & 0xFF;
                    resp_buf[1] = global_status.peaks[0] >> 8;
                    resp_buf[2] = global_status.peaks[1] & 0xFF;
                    resp_buf[3] = global_status.peaks[1] >> 8;
                    resp_buf[4] = global_status.peaks[2] & 0xFF;
                    resp_buf[5] = global_status.peaks[2] >> 8;
                    resp_buf[6] = global_status.peaks[3] & 0xFF;
                    resp_buf[7] = global_status.peaks[3] >> 8;
                    resp_buf[8] = global_status.peaks[4] & 0xFF;
                    resp_buf[9] = global_status.peaks[4] >> 8;
                    resp_buf[10] = global_status.cpu0_load;
                    resp_buf[11] = global_status.cpu1_load;
                    vendor_send_response(resp_buf, 12);
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
                int result = flash_load_params();
                if (result == FLASH_OK) {
                    dsp_recalculate_all_filters((float)audio_state.freq);
                    dsp_update_delay_samples((float)audio_state.freq);
                    loudness_recompute_pending = true;
                }
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
    if (index == 0xEE) {
        // Return NULL — we handle MS OS string via the device setup handler
        // Actually, we need to handle string 0xEE specially
        return NULL;
    }
    if (index >= 1 && index <= count_of(descriptor_strings)) {
        return descriptor_strings[index - 1];
    }
    return "";
}

// ----------------------------------------------------------------------------
// INIT
// ----------------------------------------------------------------------------

// S/PDIF Instances (4 stereo pairs on PIO0 SM0-3)
static audio_spdif_instance_t spdif_instance_1 = {0};  // Out 1-2, GPIO 6
static audio_spdif_instance_t spdif_instance_2 = {0};  // Out 3-4, GPIO 7
static audio_spdif_instance_t spdif_instance_3 = {0};  // Out 5-6, GPIO 8
static audio_spdif_instance_t spdif_instance_4 = {0};  // Out 7-8, GPIO 9

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

struct audio_buffer_format producer_format = { .format = &audio_format_48k, .sample_stride = 4 };

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

    // S/PDIF Setup (this must happen before USB init to claim DMA channels)
    producer_pool_1 = audio_new_producer_pool(&producer_format, AUDIO_BUFFER_COUNT, 192);
    producer_pool_2 = audio_new_producer_pool(&producer_format, AUDIO_BUFFER_COUNT, 192);
    producer_pool_3 = audio_new_producer_pool(&producer_format, AUDIO_BUFFER_COUNT, 192);
    producer_pool_4 = audio_new_producer_pool(&producer_format, AUDIO_BUFFER_COUNT, 192);

    // Setup 4 S/PDIF instances on PIO0 SM0-3
    audio_spdif_setup(&spdif_instance_1, &audio_format_48k, &spdif_config_1);
    audio_spdif_connect_extra(&spdif_instance_1, producer_pool_1, false, AUDIO_BUFFER_COUNT / 2, NULL);

    audio_spdif_setup(&spdif_instance_2, &audio_format_48k, &spdif_config_2);
    audio_spdif_connect_extra(&spdif_instance_2, producer_pool_2, false, AUDIO_BUFFER_COUNT / 2, NULL);

    audio_spdif_setup(&spdif_instance_3, &audio_format_48k, &spdif_config_3);
    audio_spdif_connect_extra(&spdif_instance_3, producer_pool_3, false, AUDIO_BUFFER_COUNT / 2, NULL);

    audio_spdif_setup(&spdif_instance_4, &audio_format_48k, &spdif_config_4);
    audio_spdif_connect_extra(&spdif_instance_4, producer_pool_4, false, AUDIO_BUFFER_COUNT / 2, NULL);

    irq_set_priority(DMA_IRQ_0 + PICO_AUDIO_SPDIF_DMA_IRQ, PICO_HIGHEST_IRQ_PRIORITY);

    // Start all 4 outputs synchronized
    audio_spdif_instance_t *spdif_all[] = {
        &spdif_instance_1, &spdif_instance_2, &spdif_instance_3, &spdif_instance_4
    };
    audio_spdif_enable_sync(spdif_all, 4);

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
