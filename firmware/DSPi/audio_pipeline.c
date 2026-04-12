/*
 * audio_pipeline.c — Input-agnostic DSP pipeline for DSPi
 *
 * Extracted from usb_audio.c: process_input_block() and associated
 * pipeline state (loudness filter state, preset mute envelope, CPU
 * metering, buffer watermarks).
 *
 * All output slot state DEFINITIONS remain in usb_audio.c; this file
 * accesses them via extern declarations in usb_audio.h.
 */

#include "audio_pipeline.h"
#include "usb_audio.h"
#include "config.h"
#include "dsp_pipeline.h"
#include "loudness.h"
#include "crossfeed.h"
#include "leveller.h"
#include "flash_storage.h"
#include "pdm_generator.h"
#include "pico/audio.h"
#include "pico/audio_spdif.h"
#include "pico/audio_i2s_multi.h"
#include "hardware/timer.h"
#include "hardware/sync.h"
#include <math.h>
#include <string.h>

// spdif0_consumer_fill is defined in usb_audio.c and read by main.c
extern volatile uint8_t spdif0_consumer_fill;

// ----------------------------------------------------------------------------
// PIPELINE STATE (moved from usb_audio.c)
// ----------------------------------------------------------------------------

// Loudness compensation filter state
#if PICO_RP2350
static LoudnessSvfState loudness_state[2][LOUDNESS_BIQUAD_COUNT];  // [0]=Left, [1]=Right
#else
static Biquad loudness_biquads[2][LOUDNESS_BIQUAD_COUNT];  // [0]=Left, [1]=Right
#endif

// Crossfeed state
CrossfeedState crossfeed_state;

// Volume Leveller state
LevellerCoeffs leveller_coeffs;
LevellerState leveller_state;

// Shared output buffer — file scope so Core 1 can access via pointer
#if PICO_RP2350
float buf_out[NUM_OUTPUT_CHANNELS][192];
#else
int32_t buf_out[NUM_OUTPUT_CHANNELS][192];
#endif

// Shared input buffers — file scope for pipeline access
#if PICO_RP2350
float buf_l[192], buf_r[192];
#else
int32_t buf_l[192], buf_r[192];
#endif

// Budget-based CPU load metering (Core 0)
// Measures busy_us / budget_us where budget = sample_count / sample_rate.
// Immune to bursty delivery (SPDIF RX DMA delivers 192-sample blocks every ~4ms).
static uint32_t cpu0_load_q8 = 0;         // EMA in Q8 fixed point (0-25600 = 0-100%)

// Buffer statistics watermark tracking
uint16_t buffer_stats_sequence = 0;
uint8_t spdif_consumer_min_fill_pct[NUM_SPDIF_INSTANCES];
uint8_t spdif_consumer_max_fill_pct[NUM_SPDIF_INSTANCES];
uint8_t pdm_dma_min_fill_pct = 100;
uint8_t pdm_dma_max_fill_pct = 0;
uint8_t pdm_ring_min_fill_pct = 100;
uint8_t pdm_ring_max_fill_pct = 0;

// Forward declarations for internal helpers
static void update_buffer_watermarks(void);
static inline void update_slot0_fill_fast(void);

// ----------------------------------------------------------------------------
// PRESET MUTE SMOOTHING
// ----------------------------------------------------------------------------

// Preset mute smoothing
//
// Flash-backed operations (save/load/delete, directory writes) drive
// `preset_loading` to force a temporary mute. A hard step between full-scale
// and zero can produce an audible pop on some DAC chains, so we apply a short
// envelope around the mute gate.
//
// The envelope runs in packet context (process_audio_packet) and advances by
// `sample_count` each call, giving a time-based transition that is consistent
// across 44.1/48/96 kHz.
#define PRESET_MUTE_TRANSITION_MS 8u
static float preset_mute_smooth_gain = 1.0f;  // 1.0 = full level, 0.0 = muted

static inline uint32_t preset_mute_transition_samples(uint32_t sample_rate_hz) {
    uint64_t samples = ((uint64_t)sample_rate_hz * PRESET_MUTE_TRANSITION_MS + 999u) / 1000u;
    if (samples < 1u) samples = 1u;
    if (samples > UINT32_MAX) samples = UINT32_MAX;
    return (uint32_t)samples;
}

static inline float update_preset_mute_envelope(uint32_t sample_count, uint32_t sample_rate_hz) {
    // Latch current mute state for THIS packet so the final muted packet
    // remains fully in the fade-out direction even when the counter expires.
    bool mute_active_for_packet = preset_loading;

    if (mute_active_for_packet) {
        if (preset_mute_counter > sample_count) {
            preset_mute_counter -= sample_count;
        } else {
            preset_mute_counter = 0;
            preset_loading = false;
        }
    }

    float target = mute_active_for_packet ? 0.0f : 1.0f;
    if (sample_count == 0) {
        preset_mute_smooth_gain = target;
        return preset_mute_smooth_gain;
    }

    float step = (float)sample_count / (float)preset_mute_transition_samples(sample_rate_hz);
    if (step > 1.0f) step = 1.0f;

    if (preset_mute_smooth_gain < target) {
        preset_mute_smooth_gain += step;
        if (preset_mute_smooth_gain > target) preset_mute_smooth_gain = target;
    } else if (preset_mute_smooth_gain > target) {
        preset_mute_smooth_gain -= step;
        if (preset_mute_smooth_gain < target) preset_mute_smooth_gain = target;
    }

    return preset_mute_smooth_gain;
}

// ----------------------------------------------------------------------------
// CPU METERING RESET
// ----------------------------------------------------------------------------

void pipeline_reset_cpu_metering(void) {
    cpu0_load_q8 = 0;
}

// ----------------------------------------------------------------------------
// Input-agnostic DSP pipeline: loudness, EQ, leveller, crossfeed, matrix
// mixer, per-output EQ/gain/delay, output encoding, buffer return, CPU
// metering.  Reads from file-scope buf_l[]/buf_r[] (filled by caller).
// ----------------------------------------------------------------------------
void __not_in_flash_func(process_input_block)(uint32_t sample_count) {
    uint32_t packet_start = time_us_32();

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

    update_slot0_fill_fast();
    // Watermark tracking is diagnostic-only; run at lower cadence to keep
    // the packet callback lean under heavy DSP/output load.
    static uint8_t watermark_div = 0;
    if ((++watermark_div & 0x07u) == 0) {
        update_buffer_watermarks();
    }

    uint32_t sample_rate_hz = audio_state.freq;
    float preset_mute_gain = update_preset_mute_envelope(sample_count, sample_rate_hz);

    for (int b = 0; b < NUM_SPDIF_INSTANCES; b++) {
        if (audio_buf[b]) {
            audio_buf[b]->sample_count = sample_count;
        } else if (!preset_loading && (matrix_mixer.outputs[b*2].enabled || matrix_mixer.outputs[b*2+1].enabled)) {
            spdif_overruns++;
        }
    }

#if PICO_RP2350
    // ------------------------------------------------------------------------
    // RP2350 FLOAT PIPELINE WITH MATRIX MIXER
    // ------------------------------------------------------------------------
    const float inv_32768 = 1.0f / 32768.0f;

    // vol_mul: USB host volume (raw) — used for loudness compensation only.
    // vol_mul_master: host volume × master volume — used for output gain.
    // Keeping them separate ensures master volume never affects loudness curves.
    float vol_mul = audio_state.mute ? 0.0f : (float)audio_state.vol_mul * inv_32768;
    vol_mul *= preset_mute_gain;
    float vol_mul_master = vol_mul * master_volume_linear;

    bool is_bypassed = bypass_master_eq;

    // Snapshot loudness state for this packet
    bool loud_on = loudness_enabled;
    const LoudnessCoeffs *loud_coeffs = current_loudness_coeffs;

    float peak_ml = 0, peak_mr = 0;

    // Pre-compute PDM scale factor
    const float pdm_scale = (float)(1 << 28);

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

    // ========== PASS 2.5: Volume Leveller ==========
    if (!leveller_bypassed) {
        leveller_process_block(&leveller_state, &leveller_coeffs,
                               (const LevellerConfig *)&leveller_config,
                               buf_l, buf_r, sample_count);
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
        core1_eq_work.vol_mul = vol_mul_master;  // Core 1 uses master-scaled volume
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
            // Output gain uses vol_mul_master (host vol × master vol)
            float gain = matrix_mixer.outputs[out].mute ? 0.0f
                         : matrix_mixer.outputs[out].gain_linear * vol_mul_master;
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
            // Output gain uses vol_mul_master (host vol × master vol)
            float gain = matrix_mixer.outputs[out].mute ? 0.0f
                         : matrix_mixer.outputs[out].gain_linear * vol_mul_master;
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
    // vol_mul: USB host volume (raw Q15) — used for loudness compensation only.
    // vol_mul_master: host volume × master volume (Q15) — used for output gain.
    int32_t vol_mul = audio_state.mute ? 0 : audio_state.vol_mul;
    int32_t preset_mute_gain_q15 = (int32_t)(preset_mute_gain * 32768.0f + 0.5f);
    if (preset_mute_gain_q15 < 0) preset_mute_gain_q15 = 0;
    if (preset_mute_gain_q15 > 32768) preset_mute_gain_q15 = 32768;
    vol_mul = fast_mul_q15(vol_mul, preset_mute_gain_q15);
    int32_t vol_mul_master = fast_mul_q15(vol_mul, master_volume_q15);

    bool is_bypassed = bypass_master_eq;

    // Snapshot loudness state for this packet
    bool loud_on = loudness_enabled;
    const LoudnessCoeffs *loud_coeffs = current_loudness_coeffs;

    int32_t peak_ml = 0, peak_mr = 0;

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

    // ========== PASS 2.5: Volume Leveller ==========
    if (!leveller_bypassed) {
        leveller_process_block(&leveller_state, &leveller_coeffs,
                               (const LevellerConfig *)&leveller_config,
                               buf_l, buf_r, sample_count);
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
        core1_eq_work.vol_mul = vol_mul_master;  // Core 1 uses master-scaled volume
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
            // Output gain uses vol_mul_master (host vol × master vol, Q15)
            int32_t gain = matrix_mixer.outputs[out].mute ? 0
                           : (int32_t)(matrix_mixer.outputs[out].gain_linear * (float)vol_mul_master);
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
            // Output gain uses vol_mul_master (host vol × master vol, Q15)
            int32_t gain = matrix_mixer.outputs[out].mute ? 0
                           : (int32_t)(matrix_mixer.outputs[out].gain_linear * (float)vol_mul_master);
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

    // Budget-based CPU metering: compare processing time against the time
    // budget for sample_count samples.  Immune to bursty calling patterns
    // (SPDIF RX DMA delivers 192-sample blocks every ~4ms at 48kHz; the old
    // idle-time approach clamped that 4ms gap to zero → permanent 100%).
    {
        uint32_t busy_us = packet_end - packet_start;
        uint32_t budget_us = (uint32_t)((uint64_t)sample_count * 1000000u / sample_rate_hz);
        if (budget_us > 0) {
            uint32_t inst_q8 = (busy_us * 25600) / budget_us;
            if (inst_q8 > 25600) inst_q8 = 25600;  // cap at 100%
            cpu0_load_q8 = cpu0_load_q8 - (cpu0_load_q8 >> 3) + (inst_q8 >> 3);
        }
        global_status.cpu0_load = (uint8_t)((cpu0_load_q8 + 128) >> 8);
    }
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

void get_slot_consumer_stats(uint slot, uint *cons_free, uint *cons_prepared, uint *playing) {
    // Output-type switches teardown/setup pools in main-loop context while USB
    // control requests may still run in IRQ context. Avoid dereferencing pool
    // pointers during that transition window.
    if (output_type_switch_in_progress) {
        *cons_free = SPDIF_CONSUMER_BUFFER_COUNT;
        *cons_prepared = 0;
        *playing = 0;
        return;
    }

    if (output_types[slot] == OUTPUT_TYPE_I2S) {
        audio_i2s_instance_t *inst = i2s_instance_ptrs[slot];
        if (!inst || !inst->consumer_pool) {
            *cons_free = 0;
            *cons_prepared = 0;
            *playing = 0;
            return;
        }
        *cons_free = count_pool_free(inst->consumer_pool);
        *cons_prepared = count_pool_prepared(inst->consumer_pool);
        *playing = (inst->playing_buffer != NULL) ? 1 : 0;
    } else {
        audio_spdif_instance_t *inst = spdif_instance_ptrs[slot];
        if (!inst || !inst->consumer_pool) {
            *cons_free = 0;
            *cons_prepared = 0;
            *playing = 0;
            return;
        }
        *cons_free = count_pool_free(inst->consumer_pool);
        *cons_prepared = count_pool_prepared(inst->consumer_pool);
        *playing = (inst->playing_buffer != NULL) ? 1 : 0;
    }
}

uint get_slot_consumer_fill(uint slot) {
    // See get_slot_consumer_stats(): never touch per-slot pools while a type
    // switch is mutating ownership/state.
    if (output_type_switch_in_progress) {
        return 0;
    }

    uint cons_free = SPDIF_CONSUMER_BUFFER_COUNT;

    if (output_types[slot] == OUTPUT_TYPE_I2S) {
        audio_i2s_instance_t *inst = i2s_instance_ptrs[slot];
        if (inst && inst->consumer_pool) {
            cons_free = count_pool_free(inst->consumer_pool);
        }
    } else {
        audio_spdif_instance_t *inst = spdif_instance_ptrs[slot];
        if (inst && inst->consumer_pool) {
            cons_free = count_pool_free(inst->consumer_pool);
        }
    }

    if (cons_free > SPDIF_CONSUMER_BUFFER_COUNT) cons_free = SPDIF_CONSUMER_BUFFER_COUNT;
    return SPDIF_CONSUMER_BUFFER_COUNT - cons_free;
}

// Servo-critical: update slot-0 fill every packet with minimal work.
static inline void update_slot0_fill_fast(void) {
    spdif0_consumer_fill = (uint8_t)get_slot_consumer_fill(0);
}

void reset_buffer_watermarks(void) {
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
        uint fill = get_slot_consumer_fill(i);
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
