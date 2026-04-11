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

// Clock servo PI controller gains (tunable firmware constants)
#define SERVO_KP            0.0005f   // Proportional gain
#define SERVO_KI            0.000005f // Integral gain
#define SERVO_MAX_ADJUST    0.005f    // Max fractional divider adjustment
#define SERVO_DEADBAND      (2 * SPDIF_BLOCK_SIZE)  // Ignore small FIFO deviations

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
static float servo_integral = 0.0f;
static uint32_t servo_base_divider = 0;  // Base divider for current rate (16.8 fixed-point)

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

static void on_stable_callback(spdif_rx_samp_freq_t freq) {
    spdif_rx_detected_rate = spdif_freq_to_hz(freq);
    __dmb();
    spdif_rx_stable_flag = true;
}

static void on_lost_stable_callback(void) {
    spdif_rx_lost_flag = true;
}

// ============================================================================
// CLOCK SERVO HELPERS
// ============================================================================

// Compute the base SPDIF TX clock divider for a given sample rate.
// Returns the divider in 16.8 fixed-point format (same as pio_sm_set_clkdiv_int_frac).
static uint32_t compute_base_divider(uint32_t sample_freq) {
    uint32_t system_clock_frequency = clock_get_hz(clk_sys);
    // Same formula as SPDIF TX library's update_pio_frequency()
    uint32_t divider = system_clock_frequency / sample_freq +
                       (system_clock_frequency % sample_freq != 0);
    return divider;
}

// Apply a fractional adjustment to all SPDIF/I2S output PIO dividers.
// delta is a small signed fraction: positive = slow down outputs, negative = speed up.
static void apply_servo_adjustment(float delta) {
    if (servo_base_divider == 0) return;

    // Clamp
    if (delta > SERVO_MAX_ADJUST) delta = SERVO_MAX_ADJUST;
    if (delta < -SERVO_MAX_ADJUST) delta = -SERVO_MAX_ADJUST;

    // Convert base divider from integer to float, apply fractional adjustment
    float base_f = (float)servo_base_divider;
    float adjusted = base_f * (1.0f + delta);
    if (adjusted < 256.0f) adjusted = 256.0f;  // Minimum divider = 1.0

    // Convert back to int.frac8 format
    uint32_t div_fixed = (uint32_t)(adjusted + 0.5f);
    uint16_t div_int = div_fixed >> 8;
    uint8_t div_frac = div_fixed & 0xFF;

    // Apply to all active SPDIF output instances
    extern struct audio_spdif_instance *spdif_instance_ptrs[];
    extern uint8_t output_types[];
    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        if (output_types[i] == OUTPUT_TYPE_SPDIF && spdif_instance_ptrs[i]) {
            pio_sm_set_clkdiv_int_frac(
                spdif_instance_ptrs[i]->pio,
                spdif_instance_ptrs[i]->pio_sm,
                div_int, div_frac
            );
        }
    }

    // Also apply to I2S outputs if any
    extern struct audio_i2s_instance *i2s_instance_ptrs[];
    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        if (output_types[i] == OUTPUT_TYPE_I2S && i2s_instance_ptrs[i]) {
            pio_sm_set_clkdiv_int_frac(
                i2s_instance_ptrs[i]->pio,
                i2s_instance_ptrs[i]->pio_sm,
                div_int, div_frac
            );
        }
    }
}

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
    //   RP2040: PIO1 SM2 (SM0=PDM, SM1=MCK — both occupied)
    //   RP2350: PIO2 SM0 (entire PIO block is dedicated to SPDIF RX)
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

    spdif_rx_start(&cfg);

    spdif_state = SPDIF_INPUT_ACQUIRING;
    spdif_lock_count = 0;
    spdif_loss_count = 0;
    spdif_rx_stable_flag = false;
    spdif_rx_lost_flag = false;
    spdif_rx_detected_rate = 0;
    servo_integral = 0.0f;
    servo_base_divider = 0;
    lock_debounce_polls = 0;

    printf("SPDIF RX: started on GPIO %u\n", spdif_rx_pin);
}

void spdif_input_stop(void) {
    if (spdif_state != SPDIF_INPUT_INACTIVE) {
        spdif_rx_end();
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
        servo_integral = 0.0f;
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
        servo_integral = 0.0f;
        servo_base_divider = compute_base_divider(rate);
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
    if (spdif_state != SPDIF_INPUT_LOCKED || servo_base_divider == 0)
        return;

    uint32_t fifo_count = spdif_rx_get_fifo_count();
    uint32_t target_fill = SPDIF_RX_FIFO_SIZE / 2;

    int32_t error = (int32_t)fifo_count - (int32_t)target_fill;

    // Deadband: ignore small deviations around target
    if (error > -(int32_t)SERVO_DEADBAND && error < (int32_t)SERVO_DEADBAND)
        return;

    float error_f = (float)error / (float)SPDIF_RX_FIFO_SIZE;

    // PI controller
    servo_integral += error_f * SERVO_KI;
    if (servo_integral > SERVO_MAX_ADJUST) servo_integral = SERVO_MAX_ADJUST;
    if (servo_integral < -SERVO_MAX_ADJUST) servo_integral = -SERVO_MAX_ADJUST;

    float adjust = error_f * SERVO_KP + servo_integral;

    // Positive error = FIFO filling up = outputs too slow = reduce divider (speed up)
    // Negative error = FIFO draining = outputs too fast = increase divider (slow down)
    apply_servo_adjustment(-adjust);
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
}

void spdif_input_get_channel_status(uint8_t *out_24_bytes) {
    if (spdif_state != SPDIF_INPUT_INACTIVE) {
        spdif_rx_get_c_bits(out_24_bytes, 24, 0);
    } else {
        memset(out_24_bytes, 0, 24);
    }
}
