/*
 * spdif_input.c — S/PDIF receiver integration for DSPi
 *
 * Integrates the pico_spdif_rx library into the DSPi audio pipeline.
 * Handles:
 *   - RX lifecycle (start/stop tied to input source switching)
 *   - Audio extraction: FIFO → decode → preamp → buf_l/buf_r → process_input_block
 *   - Clock servo: PI controller adjusting output PIO dividers
 *   - Lock/unlock event handling with mute transitions
 *   - Rate change detection
 */

#include "spdif_input.h"
#include "audio_input.h"
#include "audio_pipeline.h"
#include "asrc.h"
#include "config.h"
#include "dsp_pipeline.h"
#include "usb_audio.h"
#include "spdif_rx.h"

#include "hardware/sync.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "pico/audio_spdif.h"
#include "pico/audio_i2s_multi.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

// Clock servo: rate-based + proportional fill trim (mirrors USB feedback controller)
// The library's measured actual sample rate provides the base divider.
// FIFO fill level provides a small proportional trim to prevent drift.
// Fill trim biases the rounding of the PIO divider between adjacent integer
// values, dithering across LSB boundaries to achieve sub-LSB rate matching.
// With output prefill at 50% (8 buffers), worst-case PIO quantization drift
// (~156 ppm at 96kHz) takes ~19s to exhaust headroom, so gentle gains suffice.
#define SERVO_FILL_KP         0.0005f   // Fill-level proportional gain
#define SERVO_UPDATE_INTERVAL 1000      // Main loop iterations between servo updates (~20ms)

// ============================================================================
// STATE
// ============================================================================

static volatile SpdifInputState spdif_state = SPDIF_INPUT_INACTIVE;

// Flags set by DMA IRQ callbacks (minimal ISR work — main loop handles)
static volatile bool spdif_rx_stable_flag = false;
static volatile bool spdif_rx_lost_flag = false;
static volatile uint32_t spdif_rx_detected_rate = 0;

// Statistics
static uint8_t spdif_lock_count = 0;
static uint8_t spdif_loss_count = 0;

// Clock servo state
static uint32_t servo_skip_counter = 0;

// Debounce: after lock, wait this many main-loop polls with sufficient
// FIFO fill before declaring ready to unmute
static uint32_t lock_debounce_polls = 0;
#define LOCK_DEBOUNCE_THRESHOLD  8  // ~8 polls ≈ a few ms at main loop rate

// ============================================================================
// RATE MAPPING
// ============================================================================

static uint32_t spdif_freq_to_hz(spdif_rx_samp_freq_t freq) {
    switch (freq) {
        case SAMP_FREQ_44100:  return 44100;
        case SAMP_FREQ_48000:  return 48000;
        case SAMP_FREQ_88200:  return 88200;
        case SAMP_FREQ_96000:  return 96000;
        case SAMP_FREQ_176400: return 176400;
        case SAMP_FREQ_192000: return 192000;
        default:               return 0;
    }
}

// Check if a detected rate is supported by DSPi
static bool is_supported_rate(uint32_t rate_hz) {
    return (rate_hz == 44100 || rate_hz == 48000 || rate_hz == 96000);
}

// ============================================================================
// CALLBACKS (called from DMA IRQ — must be minimal)
// ============================================================================

static volatile uint32_t dbg_stable_count = 0;
static volatile uint32_t dbg_lost_count = 0;
static volatile spdif_rx_samp_freq_t dbg_last_freq = SAMP_FREQ_NONE;

static void on_stable_callback(spdif_rx_samp_freq_t freq) {
    dbg_stable_count++;
    dbg_last_freq = freq;
    spdif_rx_detected_rate = spdif_freq_to_hz(freq);
    __dmb();
    spdif_rx_stable_flag = true;
}

static void on_lost_stable_callback(void) {
    dbg_lost_count++;
    spdif_rx_lost_flag = true;
}

// ============================================================================
// CLOCK SERVO HELPERS
// ============================================================================


// Cached base dividers (16.8 fixed-point, same format as the SPDIF TX library).
// Set once when lock is acquired; servo applies fractional adjustments to these.
static uint32_t spdif_tx_base_divider = 0;  // sys_clk / sample_freq
static uint32_t i2s_tx_base_divider = 0;    // sys_clk * 2 / sample_freq

// Compute and cache the base dividers for all output types at a given sample rate.
static void servo_cache_base_dividers(uint32_t sample_freq) {
    uint32_t sys_clk = clock_get_hz(clk_sys);
    // SPDIF TX: divider = sys_clk / sample_freq (16.8 fixed-point)
    spdif_tx_base_divider = sys_clk / sample_freq +
                            (sys_clk % sample_freq != 0);
    // I2S TX: divider = sys_clk * 2 / sample_freq (ceiling division, matches I2S library)
    uint64_t i2s_num = (uint64_t)sys_clk * 2;
    i2s_tx_base_divider = (uint32_t)((i2s_num + sample_freq - 1) / sample_freq);
}

// Apply a divider (16.8 fixed-point) to a PIO SM
static inline void set_divider(PIO pio, uint sm, uint32_t div_16_8) {
    pio_sm_set_clkdiv_int_frac(pio, sm, div_16_8 >> 8, div_16_8 & 0xFF);
}

// Apply a fractional adjustment to all output PIO dividers.
// delta is a small signed fraction: positive = slow down outputs, negative = speed up.
// Uses cached base dividers so each output type keeps its correct rate ratio.
// Last written dividers — skip PIO writes when unchanged
static uint32_t last_spdif_div = 0;
static uint32_t last_i2s_div = 0;
static uint32_t last_mck_div = 0;

// ============================================================================
// PUBLIC API
// ============================================================================

void spdif_input_init(void) {
    spdif_state = SPDIF_INPUT_INACTIVE;
}

void spdif_input_start(void) {
    // Guard against double-start (would panic on resource re-claim)
    if (spdif_state != SPDIF_INPUT_INACTIVE) return;

    // PIO SM assignment:
    //   RP2040: PIO1 SM2 (SM0=PDM, SM1=MCK occupied)
    //   RP2350: PIO2 SM0 (dedicated PIO block)
#if PICO_RP2350
    const uint rx_pio_sm = 0;
#else
    const uint rx_pio_sm = 2;
#endif

    spdif_rx_config_t cfg = {
        .data_pin = spdif_rx_pin,
        .pio_sm = rx_pio_sm,
        .dma_channel0 = PICO_SPDIF_RX_DMA_CH0,
        .dma_channel1 = PICO_SPDIF_RX_DMA_CH1,
        .alarm_pool = alarm_pool_get_default(),
        .flags = SPDIF_RX_FLAGS_ALL,
    };

    // Set callbacks BEFORE start — the library can fire on_stable from DMA
    // IRQ during spdif_rx_start() if a source is connected and locks fast.
    spdif_rx_set_callback_on_stable(on_stable_callback);
    spdif_rx_set_callback_on_lost_stable(on_lost_stable_callback);

    // Hold the SPDIF TX library's DMA IRQ refcount so that
    // audio_spdif_set_enabled(false) during pipeline resets doesn't
    // disable the entire DMA_IRQ_1 line while we still need it.
    audio_spdif_irq_refcount_adjust(PICO_SPDIF_RX_DMA_IRQ, +1);

    spdif_rx_start(&cfg);

    spdif_state = SPDIF_INPUT_ACQUIRING;
    spdif_lock_count = 0;
    spdif_loss_count = 0;
    spdif_rx_stable_flag = false;
    spdif_rx_lost_flag = false;
    spdif_rx_detected_rate = 0;
    servo_skip_counter = 0;
    spdif_tx_base_divider = 0;
    i2s_tx_base_divider = 0;
    lock_debounce_polls = 0;

    printf("SPDIF RX: started on GPIO %u\n", spdif_rx_pin);
}

void spdif_input_stop(void) {
    if (spdif_state != SPDIF_INPUT_INACTIVE) {
        spdif_rx_end();
        // Release the DMA IRQ refcount hold we took in start()
        audio_spdif_irq_refcount_adjust(PICO_SPDIF_RX_DMA_IRQ, -1);
        spdif_state = SPDIF_INPUT_INACTIVE;
        spdif_rx_detected_rate = 0;
        printf("SPDIF RX: stopped\n");
    }
}

SpdifInputState spdif_input_get_state(void) {
    return spdif_state;
}

uint32_t spdif_input_get_sample_rate(void) {
    return spdif_rx_detected_rate;
}

bool spdif_input_check_rate_change(void) {
    uint32_t rate = spdif_rx_detected_rate;
    if (rate == 0 || !is_supported_rate(rate)) return false;
    if (rate != audio_state.freq) {
        // Signal rate change to main loop (same mechanism as USB rate change)
        pending_rate = rate;
        __dmb();
        rate_change_pending = true;
        return true;
    }
    return false;
}

// ============================================================================
// MAIN-LOOP POLL (called every iteration when SPDIF is active input)
// ============================================================================

// Format conversion + per-input preamp (one stereo sample).
//
// raw_l/r are signed int32 with 24-bit audio left-justified to bit 31
// (≈ ±2^31 full-scale). The natural output format of both the SPDIF FIFO
// subframe extraction below AND the ASRC, so the same helper feeds either.
//
// preamp_* are passed in by the caller, NOT read from globals here, so the
// callers can hoist the volatile-global reads out of the per-sample loop —
// otherwise the loop would issue two memory loads per iteration. This is
// what the original (pre-ASRC) loop did inline; we preserve it.
//
// Q-format note (RP2040): the legacy path right-shifts by 4 and the rest of
// the pipeline treats buf_l/r at that scale. We replicate it bit-for-bit
// so non-ASRC builds remain byte-identical to today and the downstream Q28
// EQ math is undisturbed.
#if PICO_RP2350
typedef float spdif_preamp_t;
#else
typedef int32_t spdif_preamp_t;
#endif

DSP_TIME_CRITICAL static inline void spdif_apply_preamp_one(uint32_t       idx,
                                                            int32_t        raw_l,
                                                            int32_t        raw_r,
                                                            spdif_preamp_t preamp_l,
                                                            spdif_preamp_t preamp_r)
{
#if PICO_RP2350
    const float inv_2147483648 = 1.0f / 2147483648.0f;
    buf_l[idx] = (float)raw_l * inv_2147483648 * preamp_l;
    buf_r[idx] = (float)raw_r * inv_2147483648 * preamp_r;
#else
    int32_t q_l = raw_l >> 4;
    int32_t q_r = raw_r >> 4;
    buf_l[idx] = fast_mul_q28(q_l, preamp_l);
    buf_r[idx] = fast_mul_q28(q_r, preamp_r);
#endif
}

#if SPDIF_USE_ASRC
#define ASRC_STAGING_SAMPLES 192u

/* Last fs_pipe_nom passed to asrc_init(). When audio_state.freq changes
 * (rate-change machinery has fired), we re-init ASRC. Tracked here, not
 * inside asrc.c, so that asrc.c stays focused on the ratio servo and
 * doesn't reach into pipeline globals. */
static uint32_t asrc_last_fs = 0;

static int32_t asrc_in_l[ASRC_STAGING_SAMPLES];
static int32_t asrc_in_r[ASRC_STAGING_SAMPLES];
static int32_t asrc_out_l[ASRC_STAGING_SAMPLES];
static int32_t asrc_out_r[ASRC_STAGING_SAMPLES];
static int32_t asrc_pending_l[ASRC_STAGING_SAMPLES];
static int32_t asrc_pending_r[ASRC_STAGING_SAMPLES];
static uint32_t asrc_pending_count = 0;

static inline void asrc_clear_pending_input(void) {
    asrc_pending_count = 0;
}
#endif

DSP_TIME_CRITICAL
uint32_t spdif_input_poll(void) {
    if (spdif_state == SPDIF_INPUT_INACTIVE)
        return 0;

    // --- Handle lock loss event ---
    if (spdif_rx_lost_flag) {
        spdif_rx_lost_flag = false;
        if (spdif_loss_count < 255) spdif_loss_count++;
        spdif_state = SPDIF_INPUT_RELOCKING;
        spdif_rx_detected_rate = 0;
        servo_skip_counter = 0;
        lock_debounce_polls = 0;
#if SPDIF_USE_ASRC
        /* Drop history & re-mute. The next lock acquire will re-init. */
        asrc_reset();
        asrc_clear_pending_input();
#endif
        // Muting is handled by the main loop (sets preset_loading flag)
        return 0;
    }

    // --- Handle lock acquisition event ---
    if (spdif_rx_stable_flag) {
        spdif_rx_stable_flag = false;
        if (spdif_lock_count < 255) spdif_lock_count++;

        uint32_t rate = spdif_rx_detected_rate;
        if (rate == 0 || !is_supported_rate(rate)) {
            // Unsupported rate — stay in relocking, output muted
            spdif_state = SPDIF_INPUT_RELOCKING;
            return 0;
        }

        // Prepare for locked state
        spdif_state = SPDIF_INPUT_LOCKED;
        servo_skip_counter = 0;
        servo_cache_base_dividers(rate);
        lock_debounce_polls = 0;
#if SPDIF_USE_ASRC
        /* Snapshot pipeline rate (might still be the OLD rate at this
         * point if rate-change is pending — the servo loop will catch up
         * once perform_rate_change() runs and audio_state.freq updates). */
        asrc_init(audio_state.freq);
        asrc_clear_pending_input();
        asrc_last_fs = audio_state.freq;
#endif

        // Rate change will be handled by main loop via spdif_input_check_rate_change()
        return 0;
    }

    // --- Only process audio when locked ---
    if (spdif_state != SPDIF_INPUT_LOCKED)
        return 0;

#if SPDIF_USE_ASRC
    /* If perform_rate_change() has changed audio_state.freq since our
     * last init, re-init ASRC at the new pipeline rate. This is the
     * single point where rate transitions land in ASRC mode — no extra
     * hook in main.c is needed. */
    if (audio_state.freq != asrc_last_fs) {
        asrc_init(audio_state.freq);
        asrc_clear_pending_input();
        asrc_last_fs = audio_state.freq;
    }
#endif

    // Lock debounce: wait for FIFO to build up before processing
    if (lock_debounce_polls < LOCK_DEBOUNCE_THRESHOLD) {
        lock_debounce_polls++;
        return 0;
    }

    // --- Read FIFO and feed pipeline ---
    uint32_t fifo_count = spdif_rx_get_fifo_count();
    // Need at least one stereo pair (2 subframes)
#if SPDIF_USE_ASRC
    if (fifo_count < 2 && asrc_pending_count == 0)
        return 0;
#else
    if (fifo_count < 2)
        return 0;
#endif

    // Read up to 192 stereo samples per poll (matching buf_l/buf_r size)
    uint32_t max_subframes = 192 * 2;  // 192 stereo samples = 384 subframes
    uint32_t need = fifo_count;
    if (need > max_subframes) need = max_subframes;
    need &= ~1u;  // Round down to stereo pairs

    /* Hoist preamp loads out of the per-sample loop. global_preamp_* are
     * volatile (vendor commands write them from another context); without
     * this the compiler would reload them every iteration. */
#if PICO_RP2350
    const spdif_preamp_t preamp_l = global_preamp_linear[0];
    const spdif_preamp_t preamp_r = global_preamp_linear[1];
#else
    const spdif_preamp_t preamp_l = global_preamp_mul[0];
    const spdif_preamp_t preamp_r = global_preamp_mul[1];
#endif

#if SPDIF_USE_ASRC
    /* ---------------- ASRC PATH ----------------
     * Preserve input continuity across calls: when R < 1.0, ASRC can
     * produce out_max samples while consuming fewer input samples. Those
     * unconsumed samples have already been destructively read from the
     * RX FIFO, so carry them forward and prepend them to the next call. */
    uint32_t in_idx = 0;
    for (; in_idx < asrc_pending_count && in_idx < ASRC_STAGING_SAMPLES; in_idx++) {
        asrc_in_l[in_idx] = asrc_pending_l[in_idx];
        asrc_in_r[in_idx] = asrc_pending_r[in_idx];
    }
    asrc_pending_count = 0;

    uint32_t got = 0;
    uint32_t fifo_need = need;
    uint32_t fifo_room = (ASRC_STAGING_SAMPLES - in_idx) * 2u;
    if (fifo_need > fifo_room) fifo_need = fifo_room;
    fifo_need &= ~1u;

    while (got < fifo_need && in_idx < ASRC_STAGING_SAMPLES) {
        uint32_t *buf;
        uint32_t n = spdif_rx_read_fifo(&buf, fifo_need - got);
        if (n == 0) break;
        for (uint32_t i = 0; i + 1 < n && in_idx < ASRC_STAGING_SAMPLES; i += 2) {
            asrc_in_l[in_idx] = (int32_t)((buf[i]     & 0x0FFFFFF0u) << 4);
            asrc_in_r[in_idx] = (int32_t)((buf[i + 1] & 0x0FFFFFF0u) << 4);
            in_idx++;
        }
        got += n;
    }

    if (in_idx == 0) return 0;

    unsigned in_consumed = 0;
    unsigned n_out = asrc_process(asrc_in_l, asrc_in_r, in_idx,
                                  asrc_out_l, asrc_out_r, ASRC_STAGING_SAMPLES,
                                  &in_consumed);
    if (in_consumed > in_idx) in_consumed = in_idx;

    asrc_pending_count = in_idx - in_consumed;
    for (uint32_t i = 0; i < asrc_pending_count; i++) {
        asrc_pending_l[i] = asrc_in_l[in_consumed + i];
        asrc_pending_r[i] = asrc_in_r[in_consumed + i];
    }

    if (n_out == 0) return 0;

    for (unsigned i = 0; i < n_out; i++) {
        spdif_apply_preamp_one(i, asrc_out_l[i], asrc_out_r[i],
                               preamp_l, preamp_r);
    }
    process_input_block(n_out);
    return n_out;
#else
    /* ---------------- LEGACY DIVIDER-SERVO PATH ----------------
     * Identical to pre-ASRC behaviour — output PIO dividers track input
     * rate (handled by spdif_input_update_clock_servo() below); samples
     * pass straight from the FIFO through preamp/format into buf_l/buf_r. */
    uint32_t got = 0;
    uint32_t sample_idx = 0;

    while (got < need) {
        uint32_t *buf;
        uint32_t n = spdif_rx_read_fifo(&buf, need - got);
        if (n == 0) break;  // Underrun or wrap boundary

        for (uint32_t i = 0; i + 1 < n; i += 2) {
            // Extract 24-bit signed audio from SPDIF word format:
            // bits[27:4] = 24-bit audio, bits[31:28] = VUCP
            // Shift left 4 to sign-extend into int32 full-scale
            int32_t raw_l = (int32_t)((buf[i]     & 0x0FFFFFF0u) << 4);
            int32_t raw_r = (int32_t)((buf[i + 1] & 0x0FFFFFF0u) << 4);
            spdif_apply_preamp_one(sample_idx, raw_l, raw_r,
                                   preamp_l, preamp_r);
            sample_idx++;
        }
        got += n;
    }

    if (sample_idx > 0) {
        process_input_block(sample_idx);
    }

    return sample_idx;
#endif
}

// ============================================================================
// CLOCK SERVO
// ============================================================================

void spdif_input_update_clock_servo(void) {
    if (spdif_state != SPDIF_INPUT_LOCKED || spdif_tx_base_divider == 0)
        return;

    // Rate-limit: crystal drift is slow (ppm), no need to run every iteration.
    if (++servo_skip_counter < SERVO_UPDATE_INTERVAL) return;
    servo_skip_counter = 0;

    // Library's measured actual input rate. Used by both the legacy divider
    // servo (as the primary control) AND the ASRC servo (as the rate
    // feedforward). Sanity-bounded the same way in both modes.
    float actual_freq = spdif_rx_get_samp_freq_actual();
    if (actual_freq < 20000.0f || actual_freq > 200000.0f) return;

#if SPDIF_USE_ASRC
    /* -------- ASRC servo path --------
     * Output PIO dividers are pinned at the nominal value for
     * audio_state.freq (set once on lock acquire by
     * servo_cache_base_dividers()). We never touch them again — that's
     * the whole point of ASRC mode: outputs run on the host crystal.
     * Drive the ratio servo from rate FF + consumer-pool fill error.
     */
    uint8_t consumer_fill = get_slot_consumer_fill(0);
    asrc_servo_tick(consumer_fill, actual_freq);
    return;
#else
    /* -------- LEGACY DIVIDER-SERVO PATH (unchanged) -------- */

    uint32_t sys_clk = clock_get_hz(clk_sys);
    // No ceiling — precise float division lets the fill trim dither between
    // adjacent integer divider values to achieve sub-LSB rate matching.
    float spdif_div_f = (float)sys_clk / actual_freq;
    float i2s_div_f   = (float)sys_clk * 2.0f / actual_freq;
    (void)i2s_div_f;  // unused — i2s divider derived from spdif divider × 2 below

    // -----------------------------------------------------------------------
    // Loop B: Consumer fill trim (proportional only, mirrors USB servo)
    // Uses output consumer buffer fill — the direct measure of whether
    // outputs are consuming too fast or too slow. The RX FIFO only shows
    // input-vs-pipeline rate, NOT pipeline-vs-output rate.
    // -----------------------------------------------------------------------
    uint8_t consumer_fill = get_slot_consumer_fill(0);  // Slot 0 as reference
    int32_t fill_error = (int32_t)consumer_fill - 8;    // Target 50% of 16 buffers

    float fill_trim = 0.0f;
    if (fill_error > 2 || fill_error < -2) {
        // Positive error (overfull) → negative trim → reduce divider → speed up outputs
        fill_trim = -(float)fill_error / 16.0f * SERVO_FILL_KP;
    }

    // -----------------------------------------------------------------------
    // Apply: rate-based divider + fill trim
    // I2S divider is forced to exactly 2× SPDIF divider to prevent independent
    // rounding from causing the two output types to drift apart over time.
    // -----------------------------------------------------------------------
    uint32_t spdif_div = (uint32_t)(spdif_div_f * (1.0f + fill_trim) + 0.5f);
    uint32_t i2s_div   = spdif_div * 2;

    // Skip PIO writes if dividers haven't changed
    if (spdif_div == last_spdif_div && i2s_div == last_i2s_div) return;
    last_spdif_div = spdif_div;
    last_i2s_div = i2s_div;

    extern struct audio_spdif_instance *spdif_instance_ptrs[];
    extern struct audio_i2s_instance *i2s_instance_ptrs[];
    extern uint8_t output_types[];

    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        if (output_types[i] == OUTPUT_TYPE_SPDIF && spdif_instance_ptrs[i]) {
            set_divider(spdif_instance_ptrs[i]->pio,
                        spdif_instance_ptrs[i]->pio_sm, spdif_div);
        } else if (output_types[i] == OUTPUT_TYPE_I2S && i2s_instance_ptrs[i]) {
            set_divider(i2s_instance_ptrs[i]->pio,
                        i2s_instance_ptrs[i]->pio_sm, i2s_div);
        }
    }

    // MCK servo: keep master clock frequency-locked to the servoed I2S data rate.
    // MCK divider (16.8) = sys_clk × 128 / (actual_freq × multiplier)
    extern bool i2s_mck_enabled;
    extern uint16_t i2s_mck_multiplier;
    if (i2s_mck_enabled && i2s_mck_multiplier > 0) {
        float mck_div_f = (float)sys_clk * 128.0f / (actual_freq * (float)i2s_mck_multiplier);
        uint32_t mck_div = (uint32_t)(mck_div_f * (1.0f + fill_trim) + 0.5f);
        if (mck_div != last_mck_div) {
            last_mck_div = mck_div;
            audio_i2s_mck_set_divider(mck_div);
        }
    }
#endif  /* SPDIF_USE_ASRC */
}

// ============================================================================
// STATUS QUERIES
// ============================================================================

void spdif_input_get_status(SpdifRxStatusPacket *out) {
    memset(out, 0, sizeof(*out));
    out->state = (uint8_t)spdif_state;
    out->input_source = active_input_source;
    out->lock_count = spdif_lock_count;
    out->loss_count = spdif_loss_count;
    out->sample_rate = spdif_rx_detected_rate;

    if (spdif_state != SPDIF_INPUT_INACTIVE) {
        out->parity_errors = spdif_rx_get_parity_err_count();
        uint32_t fifo_count = spdif_rx_get_fifo_count();
        out->fifo_fill_pct = (uint16_t)((fifo_count * 100u) / SPDIF_RX_FIFO_SIZE);
    }

    // DEBUG: pack diagnostic info into reserved + sample_rate fields.
    // Byte 14 (low): library internal state (0=NO_SIGNAL, 1=WAITING_STABLE, 2=STABLE)
    // Byte 15 (high): bits [7:4]=stable_cb count (0-15), bits [3:0]=lost_cb count (0-15)
    {
        spdif_rx_state_t lib_state = spdif_rx_get_state();
        uint8_t lo = (uint8_t)lib_state;
        uint8_t hi = ((dbg_stable_count & 0xF) << 4) | (dbg_lost_count & 0xF);
        out->reserved = (uint16_t)lo | ((uint16_t)hi << 8);
    }
    // When not locked, report the library's detected frequency in sample_rate
    // (overrides the 0 from spdif_rx_detected_rate when we haven't processed on_stable)
    if (spdif_state != SPDIF_INPUT_LOCKED) {
        spdif_rx_samp_freq_t lib_freq = spdif_rx_get_samp_freq();
        out->sample_rate = (uint32_t)lib_freq;  // raw enum value (44100/48000/96000 or 0)
    }
}

void spdif_input_get_channel_status(uint8_t *out_24_bytes) {
    if (spdif_state != SPDIF_INPUT_INACTIVE) {
        spdif_rx_get_c_bits(out_24_bytes, 24, 0);
    } else {
        memset(out_24_bytes, 0, 24);
    }
}
