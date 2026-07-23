/*
 * audio_pipeline.c — Input-agnostic DSP pipeline for DSPi
 *
 * Extracted from usb_audio.c: process_input_block() and associated
 * pipeline state (preset mute envelope, CPU metering, buffer watermarks).
 *
 * All output slot state DEFINITIONS remain in usb_audio.c; this file
 * accesses them via extern declarations in usb_audio.h.
 */

#include "audio_pipeline.h"
#include "usb_audio.h"
#include "audio_input.h"
#include "spdif_input.h"
#include "i2s_input.h"
#include "config.h"
#include "dsp_pipeline.h"
#include "crossover.h"
#include "loudness.h"
#include "crossfeed.h"
#include "leveller.h"
#include "flash_storage.h"
#include "pdm_generator.h"
#include "siggen.h"
#include "upmix.h"
#include "adat_output.h"
#include "output_s24.h"
#include "loopback.h"   // DSPI_LOOPBACK slot-0 capture tap (self-guarded; empty otherwise)
#include "pico/audio.h"
#include "pico/audio_spdif.h"
#include "pico/audio_i2s_multi.h"
#include "hardware/timer.h"
#include "hardware/sync.h"
#include "hardware/dma.h"
#include <math.h>
#include <string.h>

// Defined with the buffer statistics helpers below.
static bool output_rings_pre_give(uint32_t frames);

// spdif0_consumer_fill is defined in usb_audio.c and read by main.c
extern volatile uint8_t spdif0_consumer_fill;

// ----------------------------------------------------------------------------
// PIPELINE STATE (moved from usb_audio.c)
// ----------------------------------------------------------------------------

// Loudness compensation filter state lives in loudness.c
// (loudness_output_state[NUM_OUTPUT_CHANNELS], shared with the Core 1 worker).

// Crossfeed per-pair state and published coefficients live in crossfeed.c
// (crossfeed_pair_state[NUM_SPDIF_INSTANCES], current_crossfeed_coeffs).

// Volume Leveller state
LevellerCoeffs leveller_coeffs;
LevellerState leveller_state;

// Shared output buffer — file scope so Core 1 can access via pointer
#if PICO_RP2350
float buf_out[NUM_OUTPUT_CHANNELS][192];
#else
int32_t buf_out[NUM_OUTPUT_CHANNELS][192];
#endif

// Shared input buffers — file scope for pipeline access.  `input_bufs[]` lets
// the per-input EQ + metering and the matrix address any input channel
// uniformly (input k -> input_bufs[k]).  Inputs 0/1 are the stereo bus
// (buf_l/buf_r, shared by every source); higher inputs carry audio only in a
// multichannel USB alt.
#if PICO_RP2350
float buf_l[192], buf_r[192];
float buf_in_ext[NUM_INPUT_CHANNELS - NUM_STEREO_INPUTS][192];   // inputs 2..7
float *const input_bufs[NUM_INPUT_CHANNELS] = {
    buf_l, buf_r,
    buf_in_ext[0], buf_in_ext[1], buf_in_ext[2],
    buf_in_ext[3], buf_in_ext[4], buf_in_ext[5],
};
#else
int32_t buf_l[192], buf_r[192];
int32_t *const input_bufs[NUM_INPUT_CHANNELS] = { buf_l, buf_r };
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
// OUTPUT VOLUME RAMPING (host vol × master vol × preset_mute_gain)
// ----------------------------------------------------------------------------
//
// USB host volume changes (and mute toggles, master volume changes,
// preset_mute transitions) all funnel into a single composite gain that is
// applied after per-output EQ.  Snapping that gain at packet boundaries
// produces audible clicks because a typical 1 dB host step is a ~12% linear
// jump.  We hold the previous packet's ending value and linearly interpolate
// to the new target across the packet (sample 0 = prev, sample sample_count =
// new target), giving a click-free ramp shared by every output.  Both Core 0
// and Core 1 work from the same start+step, so per-output gains stay
// proportional and output slots remain phase-aligned.
#if PICO_RP2350
static float vol_mul_master_prev = 0.0f;
#else
static int32_t vol_mul_master_prev_q15 = 0;
#endif

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
// Input-agnostic DSP pipeline: per-input EQ, leveller, matrix mixer,
// per-pair crossfeed, per-output EQ/gain/loudness/delay, output encoding, buffer return,
// CPU metering.  Reads from file-scope buf_l[]/buf_r[] (filled by caller).
// ----------------------------------------------------------------------------

// Single source of truth for the active input channel count (see header).
// RAM-resident so process_input_block() can call it without an XIP stall.
uint8_t __not_in_flash_func(active_input_channel_count)(void) {
    int n;
    if (active_input_source == INPUT_SOURCE_USB)
        n = usb_input_channels;
    else if (active_input_source == INPUT_SOURCE_I2S)
        n = i2s_input_channels;
    else if (active_input_source == INPUT_SOURCE_ADAT)
        n = 8;                            // ADAT is always 8 channels
    else
        n = NUM_STEREO_INPUTS;            // S/PDIF (and any future stereo source)
    if (n > NUM_INPUT_CHANNELS) n = NUM_INPUT_CHANNELS;
    return (uint8_t)n;
}

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

    // Pre-give gate, hoisted ABOVE every output side effect (PDM pushes
    // mid-function, slot gives and the ADAT push at the end): a rejected
    // block must vanish from EVERY output together, PDM included, or the
    // outputs acquire a permanent content offset. Space only grows between
    // here and the writes, so early admission stays valid.
    bool rings_admit = output_rings_pre_give(sample_count);

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

    // vol_mul_master_target: composite output gain target for THIS packet
    //   (host volume × preset_mute_gain × master volume).
    // We linearly ramp from vol_mul_master_prev (last packet's ending value)
    // to the new target across sample_count samples; in steady state the step
    // is zero and the gain loops fall back to the original constant-gain
    // fast-path (no extra per-sample work).
    //
    // Mute sources:
    //   1. UAC1 host MUTE control (audio_state.mute) — USB-gated so the OS
    //      mute key can't silence SPDIF playback.  audio_state.vol_mul itself
    //      is already frozen at the last USB-active value because
    //      audio_set_volume() bails before touching it when source != USB.
    //   2. REQ_SET_USER_MUTE vendor channel (user_mute) — always honored, no
    //      input-source guard.  Symmetric with REQ_SET_USER_VOLUME's
    //      always-apply contract: an external control surface that mutes via
    //      the vendor channel expects audio to actually go silent.
    //
    // Snapshot both flags (and active_input_source via host_active) into
    // locals so the OR is over a consistent view — protects against any
    // future move of vendor handlers off the main loop.  Today these all
    // live on Core 0 so a torn read is impossible, but the snapshot is one
    // ldr per packet and pays for itself in clarity.
    bool s_uac_mute  = audio_state.mute;
    bool s_user_mute = user_mute;
    bool host_active = (active_input_source == INPUT_SOURCE_USB);
    bool muted = (s_uac_mute && host_active) || s_user_mute;
    float vol_mul_target = muted
                           ? 0.0f : (float)audio_state.vol_mul * inv_32768;
    vol_mul_target *= preset_mute_gain;
    float vol_mul_master_target = vol_mul_target * master_volume_linear;
    float vol_mul_master_start  = vol_mul_master_prev;
    float vol_mul_master_step   = (sample_count > 0)
        ? (vol_mul_master_target - vol_mul_master_start) / (float)sample_count
        : 0.0f;
    // Only snap prev forward when this packet actually carried audio. A
    // zero-length packet would otherwise eat the delta and force the next real
    // packet to hard-snap from start==target, undoing the ramp.
    if (sample_count > 0) {
        vol_mul_master_prev = vol_mul_master_target;
    }

    bool is_bypassed = bypass_master_eq;

    // Active input count for this packet (see active_input_channel_count()):
    // the USB alt's channel count, the I2S input channel count, or the stereo
    // pair for S/PDIF.  Each active input gets its own PEQ, then the leveller
    // (channel-count agnostic, mask-driven), then the matrix.  The matrix
    // iterates n_active_inputs, so buf_in_ext (inputs 2..7) is read ONLY when
    // those inputs are actually active; stale samples can never leak into
    // stereo / S/PDIF output.  (Crossfeed runs per output pair post-matrix,
    // so it is input-count agnostic and no longer bypassed in multichannel.)
    int n_active_inputs = active_input_channel_count();

    // Loudness snapshot for this packet: one coefficient pointer + output
    // mask, shared with Core 1 via core1_eq_work so both cores apply the
    // same view.  Compensation runs per output (post-gain) in PASS 5-7 on
    // the outputs selected by loudness_output_mask; input-count agnostic.
    const LoudnessCoeffs *loud_coeffs = loudness_enabled ? current_loudness_coeffs : NULL;
    uint16_t loud_mask = loudness_output_mask;

    // Crossfeed snapshot for this packet: published coefficient pointer
    // (NULL = disabled) + output pair mask, shared with Core 1 via
    // core1_eq_work.  Runs per output pair post-matrix (PASS 4.5).
    const CrossfeedCoeffs *xf_coeffs = (const CrossfeedCoeffs *)current_crossfeed_coeffs;
    uint8_t xf_mask = crossfeed_config.output_pair_mask;

    // Psychoacoustic bass snapshot for this packet: published coefficient
    // pointer (NULL = disabled) + output mask, shared with Core 1 via
    // core1_eq_work.  Runs per output pre-crossover in PASS 5-7 (it must see
    // the low band before any high-pass crossover removes it).
    const PsybassCoeffs *pb_coeffs = (const PsybassCoeffs *)current_psybass_coeffs;
    uint16_t pb_mask = psybass_config.output_mask;

    // Pre-compute PDM scale factor
    const float pdm_scale = (float)(1 << 28);

    // ========== PASS 2: Per-Input EQ + Metering ==========
    // Each active input channel gets its own PEQ (the generalized "master EQ"),
    // then a post-EQ peak/clip meter into global_status.  Runs only for active
    // inputs (n_active_inputs); inputs above that are zeroed so the host shows
    // no stale activity.
    for (int k = 0; k < n_active_inputs; k++) {
        float *ibuf = input_bufs[k];
        if (!is_bypassed && !channel_bypassed[k]) {
            dsp_process_channel_block(filters[k], ibuf, sample_count, k);
        }
        float pk = 0.0f;
        for (uint32_t i = 0; i < sample_count; i++) {
            float a = fabsf(ibuf[i]); if (a > pk) pk = a;
        }
        global_status.peaks[k] = (uint16_t)(fminf(1.0f, pk) * 32767.0f);
        if (pk > CLIP_THRESH_F) global_status.clip_flags |= (1u << k);
    }
    for (int k = n_active_inputs; k < NUM_INPUT_CHANNELS; k++)
        global_status.peaks[k] = 0;

    // ========== PASS 2.5: Volume Leveller ==========
    // Channel-count agnostic: detector/apply masks select input channels,
    // one linked gain preserves the mix balance.  With lookahead on, ALL
    // active inputs are delayed identically inside the leveller, so
    // inter-channel (and therefore inter-output-slot) alignment holds.
    if (!leveller_bypassed) {
        leveller_process_block(&leveller_state, &leveller_coeffs,
                               (const LevellerConfig *)&leveller_config,
                               input_bufs, (uint32_t)n_active_inputs,
                               sample_count);
    }

    // ========== PASS 3: Stereo Upmixer ==========
    // Derives Centre/Ls/Rs from the stereo bus into the otherwise-idle
    // multichannel input rows (buf_in_ext[0..2] = source rows 2..4) and raises
    // the matrix source count so they become ordinary routable sources.  Runs
    // only when the active input is the plain stereo pair; in multichannel
    // modes those rows carry real inputs and the upmixer parks (state reset,
    // source count unchanged).  Steering is pure gain on C/L/R (zero latency);
    // the surround Haas delay is identical for every slot fed from the same
    // source row, so inter-output-slot alignment is preserved.
    int n_matrix_sources = n_active_inputs;
    const UpmixCoeffs *um_coeffs = (const UpmixCoeffs *)current_upmix_coeffs;
    if (um_coeffs && n_active_inputs == NUM_STEREO_INPUTS) {
        upmix_process_block(um_coeffs, buf_l, buf_r,
                            buf_in_ext[UPMIX_ROW_C - NUM_STEREO_INPUTS],
                            buf_in_ext[UPMIX_ROW_LS - NUM_STEREO_INPUTS],
                            buf_in_ext[UPMIX_ROW_RS - NUM_STEREO_INPUTS],
                            sample_count);
        n_matrix_sources = NUM_STEREO_INPUTS + um_coeffs->n_derived;

        // Derived-row metering: rows 2..4 carry C/Ls/Rs while upmixing (the
        // PASS 2 zeroing loop above cleared them).  The extracted centre can
        // legitimately reach +3 dBFS on hot correlated content, so the host
        // must be able to see it; clip flags share the channel bit space.
        for (int k = 0; k < um_coeffs->n_derived; k++) {
            int row = NUM_STEREO_INPUTS + k;
            const float *dbuf = input_bufs[row];
            float pk = 0.0f;
            for (uint32_t i = 0; i < sample_count; i++) {
                float a = fabsf(dbuf[i]); if (a > pk) pk = a;
            }
            global_status.peaks[row] = (uint16_t)(fminf(1.0f, pk) * 32767.0f);
            if (pk > CLIP_THRESH_F) global_status.clip_flags |= (1u << row);
        }
    } else {
        upmix_park();
    }

    // ========== PASS 4: Matrix Mixing (block-based, output-major) ==========
    // Generalized over n_matrix_sources (2 for stereo / S/PDIF / I2S, 4/6/8
    // for multichannel USB, 3 or 5 with the upmixer active).  For each output,
    // snapshot the active (source buffer, signed gain) pairs ONCE, then run
    // the sample loop.  Sources are gated by n_matrix_sources, not by
    // crosspoint enables, so buf_in_ext rows are read only when they carry
    // real input or upmix-derived audio.  Every output uses the same
    // sample_count and per-sample index, so inter-output sample alignment is
    // preserved exactly (CLAUDE.md hard rule).
    for (int out = 0; out < NUM_OUTPUT_CHANNELS; out++) {
        float *dst = buf_out[out];
        if (!matrix_mixer.outputs[out].enabled) {
            memset(dst, 0, sample_count * sizeof(float));
            continue;
        }

        // Snapshot active crosspoints for this output (once, not per sample)
        const float *src[NUM_INPUT_CHANNELS];
        float        gain[NUM_INPUT_CHANNELS];
        int n = 0;
        for (int in = 0; in < n_matrix_sources; in++) {
            MatrixCrosspoint *xp = &matrix_mixer.crosspoints[in][out];
            if (!xp->enabled) continue;
            float g = xp->phase_invert ? -xp->gain_linear : xp->gain_linear;
            if (g == 0.0f) continue;
            src[n]  = input_bufs[in];
            gain[n] = g;
            n++;
        }

        if (n == 0) {
            memset(dst, 0, sample_count * sizeof(float));
            continue;
        }

        // First active input writes; remaining inputs accumulate.
        const float *s0 = src[0];
        float g0 = gain[0];
        for (uint32_t i = 0; i < sample_count; i++)
            dst[i] = s0[i] * g0;
        for (int k = 1; k < n; k++) {
            const float *sk = src[k];
            float gk = gain[k];
            for (uint32_t i = 0; i < sample_count; i++)
                dst[i] += sk[i] * gk;
        }
    }

    // Test-signal injection: replaces the matrix mix on generator channels
    // before per-output processing and the Core 1 dispatch, so every output
    // slot still advances by the same sample_count (alignment preserved).
    if (siggen_running)
        siggen_render(buf_out, sample_count, sample_rate_hz);

    // ========== PASS 5-7: Per-Output EQ + Gain + Delay + Output ==========
    // Slot finalization mode for THIS packet (see output_s24.h): in-place S24
    // staging while ADAT consumes it, fused convert+interleave otherwise.
    // Sampled once so both cores and the ADAT push agree within the packet
    // (resync runs on this same thread, so the value cannot change mid-call).
    bool finalize_s24 = adat_output_is_active();

    if (core1_mode == CORE1_MODE_EQ_WORKER) {
        // --- Dual-core path: Core 1 handles EQ+delay+SPDIF for outputs 2-7 ---

        // Dispatch to Core 1 — both cores share the same vol ramp params so
        // outputs assigned to either core stay phase-aligned.
        core1_eq_work.finalize_s24 = finalize_s24;
        core1_eq_work.sample_count = sample_count;
        core1_eq_work.vol_mul_start = vol_mul_master_start;
        core1_eq_work.vol_mul_step  = vol_mul_master_step;
        core1_eq_work.delay_write_idx = delay_write_idx;
        core1_eq_work.loud_coeffs = loud_coeffs;
        core1_eq_work.loud_mask = loud_mask;
        core1_eq_work.xfeed_coeffs = xf_coeffs;
        core1_eq_work.xfeed_mask = xf_mask;
        core1_eq_work.psybass_coeffs = pb_coeffs;
        core1_eq_work.psybass_mask = pb_mask;
        core1_eq_work.spdif_out[0] = audio_buf[1] ? (int32_t *)audio_buf[1]->buffer->bytes : NULL;
        core1_eq_work.spdif_out[1] = audio_buf[2] ? (int32_t *)audio_buf[2]->buffer->bytes : NULL;
        core1_eq_work.spdif_out[2] = audio_buf[3] ? (int32_t *)audio_buf[3]->buffer->bytes : NULL;
        core1_eq_work.work_done = false;
        __dmb();
        core1_eq_work.work_ready = true;
        __sev();

        // ========== PASS 4.5: Crossfeed (per output pair, pre-EQ) ==========
        // Core 0 owns pair 0; Core 1 runs pairs 1-3 inside eq_worker_loop.
        // Pre-EQ so per-output (headphone) EQ shapes the post-crossfeed signal.
        crossfeed_process_pairs(xf_coeffs, xf_mask, 0, 0, buf_out, sample_count);

        // Core 0: EQ + gain for outputs 0-1
        for (int out = 0; out < CORE1_EQ_FIRST_OUTPUT; out++) {
            if (!matrix_mixer.outputs[out].enabled) {
                loudness_reset_output_state(&loudness_output_state[out]);
                psybass_reset_output_state(&psybass_output_state[out]);
                continue;
            }
            // Psychoacoustic bass on masked outputs, pre-crossover (must see
            // the low band before any high-pass crossover removes it).
            // Skipped-and-cleared when masked off, muted, or RAW.
            if (pb_coeffs && ((pb_mask >> out) & 1u)
                && !matrix_mixer.outputs[out].mute
                && !(siggen_raw_mask & (1u << out))) {
                psybass_process_output_block(pb_coeffs, &psybass_output_state[out],
                                             buf_out[out], sample_count);
            } else {
                psybass_reset_output_state(&psybass_output_state[out]);
            }
            if (!matrix_mixer.outputs[out].mute && !(siggen_raw_mask & (1u << out))) {
                uint8_t eq_ch = CH_OUT_1 + out;
                if (!channel_xover_bypassed[eq_ch])
                    xover_process_channel_block(xover_filters[eq_ch], buf_out[out], sample_count);
                if (!channel_bypassed[eq_ch]) {
                    dsp_process_channel_block(filters[eq_ch], buf_out[out], sample_count, eq_ch);
                }
            }
            // Output gain ramps from gain_start → gain_start + N*step across the
            // packet (host vol × master vol × preset_mute, scaled by per-output
            // matrix gain).  In steady state step==0 and we drop into the same
            // constant-gain branches as before — no per-sample overhead.
            float matrix_gain = matrix_mixer.outputs[out].gain_linear;
            float gain_start = matrix_mixer.outputs[out].mute ? 0.0f
                               : matrix_gain * vol_mul_master_start;
            float gain_step  = matrix_mixer.outputs[out].mute ? 0.0f
                               : matrix_gain * vol_mul_master_step;
            if (gain_step == 0.0f) {
                if (gain_start == 0.0f) {
                    memset(buf_out[out], 0, sample_count * sizeof(float));
                } else if (gain_start != 1.0f) {
                    float *dst = buf_out[out];
                    for (uint32_t i = 0; i < sample_count; i++)
                        dst[i] *= gain_start;
                }
            } else {
                float *dst = buf_out[out];
                float gain = gain_start;
                for (uint32_t i = 0; i < sample_count; i++) {
                    dst[i] *= gain;
                    gain += gain_step;
                }
            }

            // Volume-keyed loudness on masked outputs, post-gain (loudness
            // only boosts at low volume, so headroom is maximal here).
            // Skipped-and-cleared when masked off, muted to zero this
            // packet, or carrying a RAW test signal.
            if (loud_coeffs && ((loud_mask >> out) & 1u)
                && !(siggen_raw_mask & (1u << out))
                && !(gain_start == 0.0f && gain_step == 0.0f)) {
                loudness_process_output_block(loud_coeffs,
                                              &loudness_output_state[out],
                                              buf_out[out], sample_count);
            } else {
                loudness_reset_output_state(&loudness_output_state[out]);
            }
        }

        // PDM is inactive in EQ_WORKER mode and owned by neither core's
        // output loop; keep its loudness/psybass state cleared so the first
        // packet after a switch back to single-core starts clean.
        loudness_reset_output_state(&loudness_output_state[NUM_OUTPUT_CHANNELS - 1]);
        psybass_reset_output_state(&psybass_output_state[NUM_OUTPUT_CHANNELS - 1]);

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

        // Core 0: finalize outputs 0-1 (see output_s24.h).  ADAT active:
        // convert to S24 in place, even with no slot buffer, so ADAT always
        // sees converted samples; otherwise fuse convert+interleave and skip
        // the staging pass.  Then S/PDIF pair 0.
        if (finalize_s24) {
            output_block_to_s24_inplace(buf_out[0], sample_count);
            output_block_to_s24_inplace(buf_out[1], sample_count);
        }
        if (audio_buf[0]) {
            int left_ch = 0, right_ch = 1;
            if (!matrix_mixer.outputs[left_ch].enabled && !matrix_mixer.outputs[right_ch].enabled) {
                memset(audio_buf[0]->buffer->bytes, 0, sample_count * 8);
            } else {
                int32_t *out_ptr = (int32_t *)audio_buf[0]->buffer->bytes;
                if (finalize_s24)
                    output_pair_interleave_s24(out_ptr, buf_out[0], buf_out[1], sample_count);
                else
                    output_pair_convert_interleave(out_ptr, buf_out[0], buf_out[1], sample_count);
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

        // ========== PASS 4.5: Crossfeed (per output pair, pre-EQ) ==========
        crossfeed_process_pairs(xf_coeffs, xf_mask, 0, NUM_SPDIF_INSTANCES - 1,
                                buf_out, sample_count);

        // EQ + gain (per-sample vol ramp, see Core 0 dual-core branch above for
        // rationale; steady-state step==0 falls back to constant-gain path).
        for (int out = 0; out < NUM_OUTPUT_CHANNELS; out++) {
            if (!matrix_mixer.outputs[out].enabled) {
                loudness_reset_output_state(&loudness_output_state[out]);
                psybass_reset_output_state(&psybass_output_state[out]);
                continue;
            }
            // Psychoacoustic bass, pre-crossover (see dual-core branch above).
            if (pb_coeffs && ((pb_mask >> out) & 1u)
                && !matrix_mixer.outputs[out].mute
                && !(siggen_raw_mask & (1u << out))) {
                psybass_process_output_block(pb_coeffs, &psybass_output_state[out],
                                             buf_out[out], sample_count);
            } else {
                psybass_reset_output_state(&psybass_output_state[out]);
            }
            if (!matrix_mixer.outputs[out].mute && !(siggen_raw_mask & (1u << out))) {
                uint8_t eq_ch = CH_OUT_1 + out;
                if (!channel_xover_bypassed[eq_ch])
                    xover_process_channel_block(xover_filters[eq_ch], buf_out[out], sample_count);
                if (!channel_bypassed[eq_ch]) {
                    dsp_process_channel_block(filters[eq_ch], buf_out[out], sample_count, eq_ch);
                }
            }
            float matrix_gain = matrix_mixer.outputs[out].gain_linear;
            float gain_start = matrix_mixer.outputs[out].mute ? 0.0f
                               : matrix_gain * vol_mul_master_start;
            float gain_step  = matrix_mixer.outputs[out].mute ? 0.0f
                               : matrix_gain * vol_mul_master_step;
            if (gain_step == 0.0f) {
                if (gain_start == 0.0f) {
                    memset(buf_out[out], 0, sample_count * sizeof(float));
                } else if (gain_start != 1.0f) {
                    float *dst = buf_out[out];
                    for (uint32_t i = 0; i < sample_count; i++)
                        dst[i] *= gain_start;
                }
            } else {
                float *dst = buf_out[out];
                float gain = gain_start;
                for (uint32_t i = 0; i < sample_count; i++) {
                    dst[i] *= gain;
                    gain += gain_step;
                }
            }

            // Volume-keyed loudness on masked outputs, post-gain (see
            // dual-core Core 0 branch above for rationale).
            if (loud_coeffs && ((loud_mask >> out) & 1u)
                && !(siggen_raw_mask & (1u << out))
                && !(gain_start == 0.0f && gain_step == 0.0f)) {
                loudness_process_output_block(loud_coeffs,
                                              &loudness_output_state[out],
                                              buf_out[out], sample_count);
            } else {
                loudness_reset_output_state(&loudness_output_state[out]);
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

        // Finalize outputs 0-7 (see output_s24.h).  ADAT active: convert to
        // S24 in place, even with no slot buffer, so ADAT always sees
        // converted samples; otherwise fuse convert+interleave per pair and
        // skip the staging pass.
        if (finalize_s24) {
            for (int out = 0; out < NUM_SPDIF_INSTANCES * 2; out++)
                output_block_to_s24_inplace(buf_out[out], sample_count);
        }
        for (int pair = 0; pair < 4; pair++) {
            if (!audio_buf[pair]) continue;
            int left_ch = pair * 2;
            int right_ch = pair * 2 + 1;
            if (!matrix_mixer.outputs[left_ch].enabled && !matrix_mixer.outputs[right_ch].enabled) {
                memset(audio_buf[pair]->buffer->bytes, 0, sample_count * 8);
                continue;
            }
            int32_t *out_ptr = (int32_t *)audio_buf[pair]->buffer->bytes;
            if (finalize_s24)
                output_pair_interleave_s24(out_ptr, buf_out[left_ch], buf_out[right_ch], sample_count);
            else
                output_pair_convert_interleave(out_ptr, buf_out[left_ch], buf_out[right_ch], sample_count);
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
            if (rings_admit) {
                for (uint32_t i = 0; i < sample_count; i++) {
                    int32_t pdm_sample_q28 = (int32_t)(buf_out[NUM_OUTPUT_CHANNELS-1][i] * pdm_scale);
                    pdm_push_sample(pdm_sample_q28, false);
                }
            }
        } else {
            global_status.peaks[CH_OUT_SUB] = 0;
        }
#endif
    }

    // (Per-input peaks/clip are written in PASS 2, above.)

#else
    // ------------------------------------------------------------------------
    // RP2040 BLOCK-BASED FIXED-POINT PIPELINE WITH MATRIX MIXER
    // 2 SPDIF stereo pairs + PDM sub, dual-core EQ worker
    // ------------------------------------------------------------------------
    // Composite output gain target for THIS packet (host vol × preset_mute ×
    // master vol, Q15).  See RP2350 branch above for the ramp rationale; on
    // RP2040 the start/step values are int32 Q15 to keep the inner gain loop
    // pure-integer.  Steady-state step==0 falls back to the original
    // constant-gain branch — no per-sample overhead when nothing is moving.
    // See RP2350 branch above for mute-source rationale (UAC1 USB-gated +
    // vendor always-applies, ORed) and the snapshot-into-locals discipline.
    bool s_uac_mute  = audio_state.mute;
    bool s_user_mute = user_mute;
    bool host_active = (active_input_source == INPUT_SOURCE_USB);
    bool muted = (s_uac_mute && host_active) || s_user_mute;
    int32_t vol_mul = muted ? 0 : audio_state.vol_mul;
    int32_t preset_mute_gain_q15 = (int32_t)(preset_mute_gain * 32768.0f + 0.5f);
    if (preset_mute_gain_q15 < 0) preset_mute_gain_q15 = 0;
    if (preset_mute_gain_q15 > 32768) preset_mute_gain_q15 = 32768;
    vol_mul = fast_mul_q15(vol_mul, preset_mute_gain_q15);
    int32_t vol_mul_master_target = fast_mul_q15(vol_mul, master_volume_q15);
    int32_t vol_mul_master_start_q15 = vol_mul_master_prev_q15;
    int32_t vol_mul_master_step_q15 = (sample_count > 0)
        ? (vol_mul_master_target - vol_mul_master_start_q15) / (int32_t)sample_count
        : 0;
    // Only snap prev forward when this packet actually carried audio (see
    // matching guard in RP2350 branch above for rationale).
    if (sample_count > 0) {
        vol_mul_master_prev_q15 = vol_mul_master_target;
    }

    bool is_bypassed = bypass_master_eq;

    // Loudness snapshot for this packet: one coefficient pointer + output
    // mask, shared with Core 1 via core1_eq_work (see RP2350 branch above).
    // Compensation runs per output (post-gain) in PASS 5-7.
    const LoudnessCoeffs *loud_coeffs = loudness_enabled ? current_loudness_coeffs : NULL;
    uint16_t loud_mask = loudness_output_mask;

    // Crossfeed snapshot for this packet (see RP2350 branch above): runs per
    // output pair post-matrix, shared with Core 1 via core1_eq_work.
    const CrossfeedCoeffs *xf_coeffs = (const CrossfeedCoeffs *)current_crossfeed_coeffs;
    uint8_t xf_mask = crossfeed_config.output_pair_mask;

    // Psychoacoustic bass snapshot for this packet (see RP2350 branch above):
    // runs per output pre-crossover, shared with Core 1 via core1_eq_work.
    const PsybassCoeffs *pb_coeffs = (const PsybassCoeffs *)current_psybass_coeffs;
    uint16_t pb_mask = psybass_config.output_mask;

    // ========== PASS 2: Per-Input EQ + Metering ========== (RP2040: 2 inputs)
    for (int k = 0; k < NUM_INPUT_CHANNELS; k++) {
        int32_t *ibuf = input_bufs[k];
        if (!is_bypassed && !channel_bypassed[k]) {
            dsp_process_channel_block(filters[k], ibuf, sample_count, k);
        }
        int32_t pk = 0;
        for (uint32_t i = 0; i < sample_count; i++) {
            int32_t a = abs(ibuf[i]); if (a > pk) pk = a;
        }
        global_status.peaks[k] = (uint16_t)(pk >> 13);
        if (pk > CLIP_THRESH_Q28) global_status.clip_flags |= (1u << k);
    }

    // ========== PASS 2.5: Volume Leveller ========== (masks select L/R)
    if (!leveller_bypassed) {
        leveller_process_block(&leveller_state, &leveller_coeffs,
                               (const LevellerConfig *)&leveller_config,
                               input_bufs, NUM_INPUT_CHANNELS, sample_count);
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

    // Test-signal injection: replaces the matrix mix on generator channels
    // before per-output processing and the Core 1 dispatch, so every output
    // slot still advances by the same sample_count (alignment preserved).
    if (siggen_running)
        siggen_render(buf_out, sample_count, sample_rate_hz);

    // ========== PASS 5-7: Per-Output EQ + Gain + Delay + Output ==========
    // PDM output index
    int pdm_out = NUM_OUTPUT_CHANNELS - 1;

    if (core1_mode == CORE1_MODE_EQ_WORKER) {
        // --- Dual-core path: Core 0 handles pair 1, Core 1 handles pair 2 ---

        // Dispatch to Core 1 — both cores share the same vol ramp params so
        // outputs assigned to either core stay phase-aligned.
        core1_eq_work.sample_count = sample_count;
        core1_eq_work.vol_mul_start = vol_mul_master_start_q15;
        core1_eq_work.vol_mul_step  = vol_mul_master_step_q15;
        core1_eq_work.delay_write_idx = delay_write_idx;
        core1_eq_work.loud_coeffs = loud_coeffs;
        core1_eq_work.loud_mask = loud_mask;
        core1_eq_work.xfeed_coeffs = xf_coeffs;
        core1_eq_work.xfeed_mask = xf_mask;
        core1_eq_work.psybass_coeffs = pb_coeffs;
        core1_eq_work.psybass_mask = pb_mask;
        core1_eq_work.spdif_out[0] = audio_buf[1] ? (int32_t *)audio_buf[1]->buffer->bytes : NULL;
        core1_eq_work.work_done = false;
        __dmb();
        core1_eq_work.work_ready = true;
        __sev();

        // ========== PASS 4.5: Crossfeed (per output pair, pre-EQ) ==========
        // Core 0 owns pair 0; Core 1 runs pair 1 inside eq_worker_loop.
        crossfeed_process_pairs(xf_coeffs, xf_mask, 0, 0, buf_out, sample_count);

        // Core 0: EQ + gain for outputs 0-1 (SPDIF pair 1)
        for (int out = 0; out < CORE1_EQ_FIRST_OUTPUT; out++) {
            if (!matrix_mixer.outputs[out].enabled) {
                loudness_reset_output_state(&loudness_output_state[out]);
                psybass_reset_output_state(&psybass_output_state[out]);
                continue;
            }
            // Psychoacoustic bass on masked outputs, pre-crossover (must see
            // the low band before any high-pass crossover removes it).
            // Skipped-and-cleared when masked off, muted, or RAW.
            if (pb_coeffs && ((pb_mask >> out) & 1u)
                && !matrix_mixer.outputs[out].mute
                && !(siggen_raw_mask & (1u << out))) {
                psybass_process_output_block(pb_coeffs, &psybass_output_state[out],
                                             buf_out[out], sample_count);
            } else {
                psybass_reset_output_state(&psybass_output_state[out]);
            }
            if (!matrix_mixer.outputs[out].mute && !(siggen_raw_mask & (1u << out))) {
                uint8_t eq_ch = CH_OUT_1 + out;
                if (!channel_xover_bypassed[eq_ch])
                    xover_process_channel_block(xover_filters[eq_ch], buf_out[out], sample_count);
                if (!channel_bypassed[eq_ch])
                    dsp_process_channel_block(filters[eq_ch], buf_out[out], sample_count, eq_ch);
            }
            // Per-sample vol ramp; step==0 in steady state → constant-gain path.
            float matrix_gain_f = matrix_mixer.outputs[out].gain_linear;
            int32_t gain_start = matrix_mixer.outputs[out].mute ? 0
                                 : (int32_t)(matrix_gain_f * (float)vol_mul_master_start_q15);
            int32_t gain_step  = matrix_mixer.outputs[out].mute ? 0
                                 : (int32_t)(matrix_gain_f * (float)vol_mul_master_step_q15);
            if (gain_step == 0) {
                if (gain_start == 0) {
                    memset(buf_out[out], 0, sample_count * sizeof(int32_t));
                } else {
                    int32_t *dst = buf_out[out];
                    for (uint32_t i = 0; i < sample_count; i++)
                        dst[i] = fast_mul_q15(dst[i], gain_start);
                }
            } else {
                int32_t *dst = buf_out[out];
                int32_t gain = gain_start;
                for (uint32_t i = 0; i < sample_count; i++) {
                    dst[i] = fast_mul_q15(dst[i], gain);
                    gain += gain_step;
                }
            }

            // Volume-keyed loudness on masked outputs, post-gain (loudness
            // only boosts at low volume, so Q28 headroom is maximal here).
            // Skipped-and-cleared when masked off, muted to zero this
            // packet, or carrying a RAW test signal.
            if (loud_coeffs && ((loud_mask >> out) & 1u)
                && !(siggen_raw_mask & (1u << out))
                && !(gain_start == 0 && gain_step == 0)) {
                loudness_process_output_block(loud_coeffs,
                                              &loudness_output_state[out],
                                              buf_out[out], sample_count);
            } else {
                loudness_reset_output_state(&loudness_output_state[out]);
            }
        }

        // PDM is inactive in EQ_WORKER mode and owned by neither core's
        // output loop; keep its loudness/psybass state cleared so the first
        // packet after a switch back to single-core starts clean.
        loudness_reset_output_state(&loudness_output_state[NUM_OUTPUT_CHANNELS - 1]);
        psybass_reset_output_state(&psybass_output_state[NUM_OUTPUT_CHANNELS - 1]);

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

        // ========== PASS 4.5: Crossfeed (per output pair, pre-EQ) ==========
        crossfeed_process_pairs(xf_coeffs, xf_mask, 0, NUM_SPDIF_INSTANCES - 1,
                                buf_out, sample_count);

        // EQ + gain (block-based, per-sample vol ramp; step==0 in steady state
        // → constant-gain path with no extra per-sample work).
        for (int out = 0; out < NUM_OUTPUT_CHANNELS; out++) {
            if (!matrix_mixer.outputs[out].enabled) {
                loudness_reset_output_state(&loudness_output_state[out]);
                psybass_reset_output_state(&psybass_output_state[out]);
                continue;
            }
            // Psychoacoustic bass, pre-crossover (see dual-core branch above).
            if (pb_coeffs && ((pb_mask >> out) & 1u)
                && !matrix_mixer.outputs[out].mute
                && !(siggen_raw_mask & (1u << out))) {
                psybass_process_output_block(pb_coeffs, &psybass_output_state[out],
                                             buf_out[out], sample_count);
            } else {
                psybass_reset_output_state(&psybass_output_state[out]);
            }
            if (!matrix_mixer.outputs[out].mute && !(siggen_raw_mask & (1u << out))) {
                uint8_t eq_ch = CH_OUT_1 + out;
                if (!channel_xover_bypassed[eq_ch])
                    xover_process_channel_block(xover_filters[eq_ch], buf_out[out], sample_count);
                if (!channel_bypassed[eq_ch])
                    dsp_process_channel_block(filters[eq_ch], buf_out[out], sample_count, eq_ch);
            }
            float matrix_gain_f = matrix_mixer.outputs[out].gain_linear;
            int32_t gain_start = matrix_mixer.outputs[out].mute ? 0
                                 : (int32_t)(matrix_gain_f * (float)vol_mul_master_start_q15);
            int32_t gain_step  = matrix_mixer.outputs[out].mute ? 0
                                 : (int32_t)(matrix_gain_f * (float)vol_mul_master_step_q15);
            if (gain_step == 0) {
                if (gain_start == 0) {
                    memset(buf_out[out], 0, sample_count * sizeof(int32_t));
                } else {
                    int32_t *dst = buf_out[out];
                    for (uint32_t i = 0; i < sample_count; i++)
                        dst[i] = fast_mul_q15(dst[i], gain_start);
                }
            } else {
                int32_t *dst = buf_out[out];
                int32_t gain = gain_start;
                for (uint32_t i = 0; i < sample_count; i++) {
                    dst[i] = fast_mul_q15(dst[i], gain);
                    gain += gain_step;
                }
            }

            // Volume-keyed loudness on masked outputs, post-gain (see
            // dual-core Core 0 branch above for rationale).
            if (loud_coeffs && ((loud_mask >> out) & 1u)
                && !(siggen_raw_mask & (1u << out))
                && !(gain_start == 0 && gain_step == 0)) {
                loudness_process_output_block(loud_coeffs,
                                              &loudness_output_state[out],
                                              buf_out[out], sample_count);
            } else {
                loudness_reset_output_state(&loudness_output_state[out]);
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
            if (rings_admit) {
                for (uint32_t i = 0; i < sample_count; i++) {
                    pdm_push_sample(buf_out[pdm_out][i], false);
                }
            }
        } else {
            global_status.peaks[CH_OUT_SUB] = 0;
        }
#endif
    }

    // (Per-input peaks/clip are written in PASS 2, above.)
#endif

#ifdef DSPI_LOOPBACK
    // Loopback capture tap (debug build): copy slot 0's finalized, interleaved
    // 24-bit output into the capture ring just before the buffer is handed to
    // the output DMA.  By here slot 0 is fully written for every pipeline
    // variant (RP2350 dual/single-core, RP2040 dual/single-core), including the
    // silence (memset) branch.  Read-only — does not perturb inter-slot phase.
    if (audio_buf[0]) {
        loopback_push_slot0((const int32_t *)audio_buf[0]->buffer->bytes, sample_count);
    }
#endif

    // rings_admit was decided at the top of this function, before the PDM
    // pushes; monotone space keeps it valid here.

    // Return all buffers
#if PICO_RP2350
    for (int b = 0; b < 4; b++) {
        if (audio_buf[b]) {
            struct audio_buffer_pool *pool = (b == 0) ? producer_pool_1 :
                                              (b == 1) ? producer_pool_2 :
                                              (b == 2) ? producer_pool_3 : producer_pool_4;
            if (rings_admit) give_audio_buffer(pool, audio_buf[b]);
            else queue_free_audio_buffer(pool, audio_buf[b]);
        }
    }

    // ADAT bulk output tap: in finalize_s24 mode buf_out[0..7] hold in-place
    // S24 for every pipeline variant here (see output_s24.h), and pushing
    // AFTER the (blocking) gives keeps the ADAT ring lead bounded by the
    // slot-0 consumer fill; see adat_output.c.  The snapshot gate keeps the
    // push and the conversion mode consistent within the packet.
    if (finalize_s24 && rings_admit)
        adat_output_push_block((const out_s24_t (*)[192])buf_out, sample_count);
#else
    if (rings_admit) {
        if (audio_buf[0]) give_audio_buffer(producer_pool_1, audio_buf[0]);
        if (audio_buf[1]) give_audio_buffer(producer_pool_2, audio_buf[1]);
    } else {
        if (audio_buf[0]) queue_free_audio_buffer(producer_pool_1, audio_buf[0]);
        if (audio_buf[1]) queue_free_audio_buffer(producer_pool_2, audio_buf[1]);
    }
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

// Pre-give gate, once per block: (1) underrun recovery, deficit measured
// against the worst enabled slot, one identical skip everywhere; then
// (2) group admission, true only when EVERY enabled slot can accept the
// WHOLE block. Space only grows between this check and the writes, so an
// admitted block can never be partially written; a rejected block is
// dropped uniformly before any ring (or the ADAT mirror) sees it, which
// is what keeps wr_frames, and therefore alignment, identical across
// slots at the writable cap.
DSP_TIME_CRITICAL
static bool output_rings_pre_give(uint32_t frames) {
    uint32_t max_deficit = 0;
    for (uint i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        int32_t deficit = 0;
        if (output_types[i] == OUTPUT_TYPE_I2S) {
            audio_i2s_instance_t *inst = i2s_instance_ptrs[i];
            if (!inst || !inst->ring || !inst->enabled) continue;
            deficit = (int32_t)(audio_i2s_ring_consumed_frames(inst)
                                - inst->wr_frames);
        } else {
            audio_spdif_instance_t *inst = spdif_instance_ptrs[i];
            if (!inst || !inst->ring || !inst->enabled) continue;
            deficit = (int32_t)(audio_spdif_ring_consumed_frames(inst)
                                - inst->wr_frames);
        }
        if (deficit > 0 && (uint32_t)deficit > max_deficit)
            max_deficit = (uint32_t)deficit;
    }
    if (max_deficit != 0) {
        uint32_t skip = ((max_deficit + 192u + 47u) / 48u) * 48u;
        for (uint i = 0; i < NUM_SPDIF_INSTANCES; i++) {
            if (output_types[i] == OUTPUT_TYPE_I2S) {
                audio_i2s_instance_t *inst = i2s_instance_ptrs[i];
                if (inst && inst->ring && inst->enabled)
                    audio_i2s_ring_skip(inst, skip);
            } else {
                audio_spdif_instance_t *inst = spdif_instance_ptrs[i];
                if (inst && inst->ring && inst->enabled)
                    audio_spdif_ring_skip(inst, skip);
            }
        }
        extern volatile int overruns;
        extern volatile bool preset_loading;
        if (!preset_loading) overruns++;
    }

    // Group admission.
    for (uint i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        int32_t fill;
        if (output_types[i] == OUTPUT_TYPE_I2S) {
            audio_i2s_instance_t *inst = i2s_instance_ptrs[i];
            if (!inst || !inst->ring || !inst->enabled) continue;
            fill = (int32_t)(inst->wr_frames - audio_i2s_ring_consumed_frames(inst));
            if (fill < 0) fill = 0;
            if ((uint32_t)fill + frames >
                PICO_AUDIO_I2S_RING_FRAMES - PICO_AUDIO_I2S_ERASE_AHEAD)
                return false;
        } else {
            audio_spdif_instance_t *inst = spdif_instance_ptrs[i];
            if (!inst || !inst->ring || !inst->enabled) continue;
            fill = (int32_t)(inst->wr_frames - audio_spdif_ring_consumed_frames(inst));
            if (fill < 0) fill = 0;
            if ((uint32_t)fill + frames >
                PICO_AUDIO_SPDIF_RING_FRAMES - PICO_AUDIO_SPDIF_ERASE_AHEAD)
                return false;
        }
    }
    return true;
}

// Idle silence pump: with outputs enabled but no producer running (USB
// pause, source gaps), keep the ring silent ahead of the reader so the
// free-running DMA can never loop stale audio. The fill gate makes it a
// no-op while streaming (fill holds near 8 buckets).
DSP_TIME_CRITICAL
void output_rings_idle_pump(void) {
    if (output_type_switch_in_progress) return;
    // Re-anchor here as well as in the give path: with no gives flowing
    // (source switch, host stream stopped) the writer must still track the
    // reader, or the deficit grows without bound. No-op when fill >= 0.
    (void)output_rings_pre_give(0);
    for (uint i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        // Signed fill: a reader-ahead deficit must ENGAGE the pump, not
        // read as a huge unsigned fill and disarm it.
        if (output_types[i] == OUTPUT_TYPE_I2S) {
            audio_i2s_instance_t *inst = i2s_instance_ptrs[i];
            if (inst && inst->ring && inst->enabled &&
                (int32_t)(inst->wr_frames - audio_i2s_ring_consumed_frames(inst)) < 240)
                audio_i2s_ring_silence_pump(inst, 192u);
        } else {
            audio_spdif_instance_t *inst = spdif_instance_ptrs[i];
            if (inst && inst->ring && inst->enabled &&
                (int32_t)(inst->wr_frames - audio_spdif_ring_consumed_frames(inst)) < 240)
                audio_spdif_ring_silence_pump(inst, 192u);
        }
    }
}

// Flash blackouts stall the main loop (and so the idle pump) for ~45 ms,
// longer than the erase-ahead margin; pre-stamp a full ring of silence so
// the wire stays clean throughout. Called from the flash-write bracket.
void output_rings_prestamp_full_silence(void) {
    if (output_type_switch_in_progress) return;
    for (uint i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        if (output_types[i] == OUTPUT_TYPE_I2S) {
            audio_i2s_instance_t *inst = i2s_instance_ptrs[i];
            if (inst && inst->ring && inst->enabled)
                audio_i2s_ring_silence_pump(inst, PICO_AUDIO_I2S_RING_FRAMES);
        } else {
            audio_spdif_instance_t *inst = spdif_instance_ptrs[i];
            if (inst && inst->ring && inst->enabled)
                audio_spdif_ring_silence_pump(inst, PICO_AUDIO_SPDIF_RING_FRAMES);
        }
    }
}

// One "bucket" = 48 frames (the old consumer-buffer size); the vendor
// stats wire format and every prefill threshold keep bucket units.
#define SLOT_FILL_FRAMES_PER_BUCKET 48u
#define SLOT_FILL_MAX_BUCKETS       SPDIF_CONSUMER_BUFFER_COUNT

// Live ring fill in frames for a slot; returns false when no valid reading
// exists (type switch in progress, slot not set up).
static bool slot_ring_fill(uint slot, uint32_t *fill_frames) {
    if (output_type_switch_in_progress) return false;
    if (output_types[slot] == OUTPUT_TYPE_I2S) {
        audio_i2s_instance_t *inst = i2s_instance_ptrs[slot];
        if (!inst || !inst->ring) return false;
        *fill_frames = audio_i2s_ring_fill_frames(inst);
    } else {
        audio_spdif_instance_t *inst = spdif_instance_ptrs[slot];
        if (!inst || !inst->ring) return false;
        *fill_frames = audio_spdif_ring_fill_frames(inst);
    }
    // Reader ahead of writer (producer idle) is a deficit, not a huge fill.
    if ((int32_t)*fill_frames < 0) *fill_frames = 0;
    return true;
}

void get_slot_consumer_stats(uint slot, uint *cons_free, uint *cons_prepared, uint *playing) {
    // Bucket mapping keeps REQ_GET_BUFFER_STATS byte-compatible:
    // prepared = fill buckets, free = capacity minus fill, playing = enabled.
    uint32_t fill;
    if (!slot_ring_fill(slot, &fill)) {
        *cons_free = output_type_switch_in_progress ? SLOT_FILL_MAX_BUCKETS : 0;
        *cons_prepared = 0;
        *playing = 0;
        return;
    }
    uint buckets = fill / SLOT_FILL_FRAMES_PER_BUCKET;
    if (buckets > SLOT_FILL_MAX_BUCKETS) buckets = SLOT_FILL_MAX_BUCKETS;
    *cons_prepared = buckets;
    *cons_free = SLOT_FILL_MAX_BUCKETS - buckets;
    if (output_types[slot] == OUTPUT_TYPE_I2S) {
        *playing = (i2s_instance_ptrs[slot] && i2s_instance_ptrs[slot]->enabled) ? 1 : 0;
    } else {
        *playing = (spdif_instance_ptrs[slot] && spdif_instance_ptrs[slot]->enabled) ? 1 : 0;
    }
}

DSP_TIME_CRITICAL
uint get_slot_consumer_fill(uint slot) {
    // Bucket units (0..16) so every prefill threshold keeps its meaning.
    uint32_t fill;
    if (!slot_ring_fill(slot, &fill)) return 0;
    uint buckets = fill / SLOT_FILL_FRAMES_PER_BUCKET;
    return (buckets > SLOT_FILL_MAX_BUCKETS) ? SLOT_FILL_MAX_BUCKETS : buckets;
}

// Live ring fill in frames for the vendor stats path (sample-granular).
// Returns false when no valid reading exists.
bool get_slot_consumer_fill_frames(uint slot, uint32_t *frames) {
    return slot_ring_fill(slot, frames);
}

// Fractional fill for the clock servo, in bucket units, from the ring's
// cumulative-count snapshot pair: produced and consumed frame totals are
// latched together at the end of each block write, so the measurement is
// taken at a constant production phase and the block-arrival sawtooth
// never enters it. Negative = no valid reading this tick.
DSP_TIME_CRITICAL
float get_slot_consumer_fill_frac(uint slot) {
    if (output_type_switch_in_progress) return -1.0f;

    uint32_t produced, consumed;
    if (output_types[slot] == OUTPUT_TYPE_I2S) {
        audio_i2s_instance_t *inst = i2s_instance_ptrs[slot];
        if (!inst || !inst->ring) return -1.0f;
        produced = inst->snap_produced_frames;
        consumed = inst->snap_consumed_frames;
    } else {
        audio_spdif_instance_t *inst = spdif_instance_ptrs[slot];
        if (!inst || !inst->ring) return -1.0f;
        produced = inst->snap_produced_frames;
        consumed = inst->snap_consumed_frames;
    }
    return (float)(int32_t)(produced - consumed) *
           (1.0f / (float)SLOT_FILL_FRAMES_PER_BUCKET);
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

DSP_TIME_CRITICAL
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

// ----------------------------------------------------------------------------
// TEST-SIGNAL PUMP
// ----------------------------------------------------------------------------
//
// process_input_block() is normally driven by whichever input source delivers
// samples.  When the generator is running and no source is streaming, nothing
// would call it and the outputs would drain to silence, so the main loop
// calls this pump instead.  It feeds zero-input blocks, paced by the slot-0
// consumer fill level (the same signal the USB feedback servo and the SPDIF
// prefill logic use), topping up to half-full.  Every block goes through the
// full pipeline, so inter-slot alignment and the delay-line write index
// advance exactly as they do for a real input source.
void siggen_pump(void) {
    if (!siggen_running) return;

    // Never pump while the pipeline is owned by someone else: an input
    // source actively delivering blocks, a pending source change, an output
    // type switch mutating the pools, or a flash operation (preset mute).
    //
    // preset_loading needs care: it is only ever cleared by the block path
    // consuming preset_mute_counter (preset_mute_gain_step), which needs
    // blocks to flow.  In USB mode with no host stream the pump is the only
    // block source, so refusing on preset_loading would deadlock - boot (or
    // a stream-restart resync) leaves preset_loading latched until the next
    // host stream, and a started generator sits in FADE_IN forever, silent.
    // Pump through the mute instead: blocks render muted until the counter
    // clears, then the generator fades in.  (Actual preset loads stop the
    // generator via siggen_stop_immediate, so !siggen_running already gates
    // those.)  Non-USB sources keep the refusal: their preset_loading
    // doubles as the lock-acquisition / prefill handshake signal
    // (main-loop SPDIF/I2S blocks) and must not be consumed by the pump.
    if (preset_loading && active_input_source != INPUT_SOURCE_USB) return;
    if (input_source_change_pending ||
        output_type_switch_in_progress || producer_pool_1 == NULL)
        return;
    bool streaming =
        (active_input_source == INPUT_SOURCE_USB && usb_audio_stream_active()) ||
        (input_source_is_spdif(active_input_source) &&
         spdif_input_get_state() == SPDIF_INPUT_LOCKED) ||
        (active_input_source == INPUT_SOURCE_I2S &&
         i2s_input_get_state() == I2S_INPUT_RUNNING);
    if (streaming) return;

    // Bounded top-up per call to keep the main loop responsive; the loop
    // spins fast enough to sustain any supported rate.
    for (int b = 0; b < 2; b++) {
        if (get_slot_consumer_fill(0) >= SPDIF_CONSUMER_BUFFER_COUNT / 2)
            break;
        int n_in = active_input_channel_count();
        for (int k = 0; k < n_in; k++)
            memset(input_bufs[k], 0, AUDIO_BUFFER_SAMPLES * sizeof(input_bufs[0][0]));
        process_input_block(AUDIO_BUFFER_SAMPLES);
    }
}
