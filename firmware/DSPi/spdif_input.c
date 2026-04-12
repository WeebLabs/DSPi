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
// Fill trim gain: must be strong enough to dither between adjacent PIO
// divider values (~8 Hz step at 48kHz) to achieve sub-LSB rate matching.
#define SERVO_FILL_KP       0.001f    // Fill-level proportional gain
#define SERVO_FILL_DEADBAND (SPDIF_BLOCK_SIZE / 2)  // Tight deadband — allow active dithering
#define SERVO_UPDATE_INTERVAL 250     // Main loop iterations between servo updates (~5ms)

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

        // Rate change will be handled by main loop via spdif_input_check_rate_change()
        return 0;
    }

    // --- Only process audio when locked ---
    if (spdif_state != SPDIF_INPUT_LOCKED)
        return 0;

    // Lock debounce: wait for FIFO to build up before processing
    if (lock_debounce_polls < LOCK_DEBOUNCE_THRESHOLD) {
        lock_debounce_polls++;
        return 0;
    }

    // --- Read FIFO and feed pipeline ---
    uint32_t fifo_count = spdif_rx_get_fifo_count();
    // Need at least one stereo pair (2 subframes)
    if (fifo_count < 2)
        return 0;

    // Read up to 192 stereo samples per poll (matching buf_l/buf_r size)
    uint32_t max_subframes = 192 * 2;  // 192 stereo samples = 384 subframes
    uint32_t need = fifo_count;
    if (need > max_subframes) need = max_subframes;
    need &= ~1u;  // Round down to stereo pairs

    // Apply preamp and fill buf_l/buf_r
#if PICO_RP2350
    float preamp_l = global_preamp_linear[0];
    float preamp_r = global_preamp_linear[1];
    const float inv_2147483648 = 1.0f / 2147483648.0f;
#else
    int32_t preamp_l = global_preamp_mul[0];
    int32_t preamp_r = global_preamp_mul[1];
#endif

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

#if PICO_RP2350
            // Float: int32 full-scale → [-1.0, 1.0] with preamp
            buf_l[sample_idx] = (float)raw_l * inv_2147483648 * preamp_l;
            buf_r[sample_idx] = (float)raw_r * inv_2147483648 * preamp_r;
#else
            // Q28: int32 full-scale >> 4 → Q28, then apply preamp
            int32_t q28_l = raw_l >> 4;
            int32_t q28_r = raw_r >> 4;
            buf_l[sample_idx] = fast_mul_q28(q28_l, preamp_l);
            buf_r[sample_idx] = fast_mul_q28(q28_r, preamp_r);
#endif
            sample_idx++;
        }
        got += n;
    }

    if (sample_idx > 0) {
        process_input_block(sample_idx);
    }

    return sample_idx;
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

    // -----------------------------------------------------------------------
    // Loop A: Rate-based divider (primary control)
    // Use the library's measured actual input sample rate to compute the
    // ideal output divider directly. No IIR needed — the library already
    // averages over 8 blocks (~32ms at 48kHz).
    // -----------------------------------------------------------------------
    float actual_freq = spdif_rx_get_samp_freq_actual();
    if (actual_freq < 20000.0f || actual_freq > 200000.0f) return;  // Sanity check

    uint32_t sys_clk = clock_get_hz(clk_sys);
    // No ceiling — precise float division lets the fill trim dither between
    // adjacent integer divider values to achieve sub-LSB rate matching.
    float spdif_div_f = (float)sys_clk / actual_freq;
    float i2s_div_f   = (float)sys_clk * 2.0f / actual_freq;

    // -----------------------------------------------------------------------
    // Loop B: Consumer fill trim (proportional only, mirrors USB servo)
    // Uses output consumer buffer fill — the direct measure of whether
    // outputs are consuming too fast or too slow. The RX FIFO only shows
    // input-vs-pipeline rate, NOT pipeline-vs-output rate.
    // -----------------------------------------------------------------------
    uint8_t consumer_fill = get_slot_consumer_fill(0);  // Slot 0 as reference
    int32_t fill_error = (int32_t)consumer_fill - 8;    // Target 50% of 16 buffers

    float fill_trim = 0.0f;
    if (fill_error > 1 || fill_error < -1) {
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
