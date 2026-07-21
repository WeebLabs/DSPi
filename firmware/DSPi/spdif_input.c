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
#include "adat_output.h"
#include "input_servo.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

// Clock servo: rate-based + proportional fill trim (mirrors USB feedback controller).
// The divider math and PIO/MCK writes live in input_servo.c (shared with the
// ADAT input's slave clock mode); this module owns lock gating, rate limiting,
// and the library's rate measurement.
// With output prefill at 50% (8 buffers), worst-case PIO quantization drift
// (~156 ppm at 96kHz) takes ~19s to exhaust headroom, so gentle gains suffice.
#define SERVO_UPDATE_INTERVAL 1000      // Main loop iterations between servo updates (~20ms)

// Long-window rate measurement (mirrors the ADAT slave dual-anchor scheme in
// adat_input.c). The library's samp_freq_actual is an 8-block moving average
// of DMA IRQ intervals; the average telescopes to two IRQ timestamps only
// ~32 ms apart (16 ms at 96 kHz), so a few us of IRQ latency jitter is
// 100+ ppm of rate noise. Fed raw to the servo that slams the 16.8 output
// dividers (1 LSB = 156 ppm at 48 kHz) across several LSBs every tick, which
// bounces the consumer fill and frequency-modulates the outgoing SPDIF
// carrier. Instead: count decoded blocks (exactly 192 frames each while
// stable) against the completing IRQ's entry timestamp, via the coherent
// {count, time} pair the library publishes (spdif_rx_get_block_ref). Anchors
// rotate every SERVO_LONG_HALF_US, so the steady-state span is 8-16 s and
// endpoint jitter is well under 1 ppm. The count-based rate is trusted once
// the span reaches SERVO_LONG_MIN_US (~10 ppm noise at 1 s, still far below
// one divider LSB); an IIR-smoothed library estimate bridges the first
// second after lock.
#define SERVO_LONG_MIN_US   1000000ull  // min anchor span for count-based rate
#define SERVO_LONG_HALF_US  8000000ull  // anchor rotation half-period
#define SERVO_SMOOTH_ALPHA  0.25f       // bridge-estimate IIR coefficient

// ============================================================================
// STATE
// ============================================================================

static volatile SpdifInputState spdif_state = SPDIF_INPUT_INACTIVE;

// GPIO the RX library was started on.  Recorded at start so stop() releases
// the right pad even if the live pin config changed while running (the RX
// library's teardown does not reset the GPIO function itself).
static uint8_t spdif_active_data_pin = 0xFF;

// Flags set by DMA IRQ callbacks (minimal ISR work — main loop handles)
static volatile bool spdif_rx_stable_flag = false;
static volatile bool spdif_rx_lost_flag = false;
static volatile uint32_t spdif_rx_detected_rate = 0;

// Statistics
static uint8_t spdif_lock_count = 0;
static uint8_t spdif_loss_count = 0;

// Clock servo state
static uint32_t servo_skip_counter = 0;
static uint64_t servo_long_us[2];       // dual-anchor timestamps
static uint32_t servo_long_blocks[2];   // dual-anchor decoded-block counts
static bool     servo_long_valid = false;
static float    servo_hz_long = 0.0f;   // count-based long-window rate
static float    servo_hz_smooth = 0.0f; // IIR-smoothed bridge estimate

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

static void __not_in_flash_func(on_stable_callback)(spdif_rx_samp_freq_t freq) {
    dbg_stable_count++;
    dbg_last_freq = freq;
    spdif_rx_detected_rate = spdif_freq_to_hz(freq);
    __dmb();
    spdif_rx_stable_flag = true;
}

static void __not_in_flash_func(on_lost_stable_callback)(void) {
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

// Re-anchor the long-window rate measurement and clear the bridge estimate.
// Call when lock is (re)acquired; the library republishes the {count, time}
// pair every block IRQ, so the snapshot read here is at most one block old.
static void servo_meas_arm(void) {
    uint32_t blocks;
    uint64_t t_us;
    spdif_rx_get_block_ref(&blocks, &t_us);
    servo_long_us[0] = servo_long_us[1] = t_us;
    servo_long_blocks[0] = servo_long_blocks[1] = blocks;
    servo_long_valid = false;
    servo_hz_long = 0.0f;
    servo_hz_smooth = 0.0f;
}

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

    // Run on the GPIO of whichever SPDIF input (1/2/3) is the active source
    spdif_active_data_pin = spdif_rx_active_pin();

    spdif_rx_config_t cfg = {
        .data_pin = spdif_active_data_pin,
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
    servo_long_valid = false;  // real re-anchor happens at lock (servo_meas_arm)
    input_servo_reset();  // force a full divider rewrite after (re)lock
    lock_debounce_polls = 0;

    printf("SPDIF RX: started on GPIO %u\n", spdif_active_data_pin);
}

void spdif_input_stop(void) {
    if (spdif_state != SPDIF_INPUT_INACTIVE) {
        spdif_rx_end();
        // Release the DMA IRQ refcount hold we took in start()
        audio_spdif_irq_refcount_adjust(PICO_SPDIF_RX_DMA_IRQ, -1);
        // The library's teardown leaves the pad muxed to PIO input; reset it
        // so a pin/input switch actually frees the old GPIO (mirrors
        // i2s_input_stop).
        if (spdif_active_data_pin != 0xFF) {
            gpio_set_function(spdif_active_data_pin, GPIO_FUNC_NULL);
            gpio_set_dir(spdif_active_data_pin, GPIO_IN);
            spdif_active_data_pin = 0xFF;
        }
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
        servo_meas_arm();
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
    // Live-tail read (ADAT-ring style): count and consume the words the
    // in-flight DMA block transfer has already written, not just completed
    // blocks. Block-granular release (384 subframes = 4 consumer buffers in
    // one poll, every 4 ms at 48 kHz) made the consumer fill sawtooth across
    // 4 buffers at block rate even with a perfectly stable servo; word-
    // granular delivery moves the fill smoothly like the ADAT DMA ring does.
    uint32_t fifo_count = spdif_rx_get_fifo_count_live();

    // Batch into ~1 ms granules (the pipeline's native USB-packet cadence).
    // Word-granular availability is what keeps the consumer fill smooth, but
    // processing the few frames that land per main-loop iteration lets
    // process_input_block()'s fixed cost (producer pool takes/gives, gain
    // ramp + filter state setup, core metering) dominate: 54% CPU0 measured
    // on hardware at 48 kHz from sliver-sized calls, 9% batched. Waiting for
    // a ~1 ms granule restores the USB-like call profile and also keeps every
    // consumer buffer carrying a full payload, so the 16-buffer fill count
    // keeps meaning ~16 ms of real cushion.
    uint32_t min_subframes = (audio_state.freq / 1000) * 2;
    if (min_subframes < 2) min_subframes = 2;
    if (fifo_count < min_subframes)
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
        uint32_t n = spdif_rx_read_fifo_live(&buf, need - got);
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
            // Q28: int32 full-scale >> 2 -> Q28, then apply preamp.
            // Matches the RP2040 USB 24-bit path (net 24-bit sample << 6),
            // so the later Q28 -> int24 output shift preserves unity gain.
            int32_t q28_l = raw_l >> 2;
            int32_t q28_r = raw_r >> 2;
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

DSP_TIME_CRITICAL
void spdif_input_update_clock_servo(void) {
    if (spdif_state != SPDIF_INPUT_LOCKED || spdif_tx_base_divider == 0)
        return;

    // Signal-loss pending but not yet processed by the poll (the flag is set
    // from IRQ, so it can land between this iteration's poll and servo
    // calls). The newest decoded blocks may then be partial or corrupt and
    // the FIFO is draining; skip rather than servo on either. The poll drops
    // lock on the next iteration.
    if (spdif_rx_lost_flag) return;

    // Rate-limit: crystal drift is slow (ppm), no need to run every iteration.
    if (++servo_skip_counter < SERVO_UPDATE_INTERVAL) return;
    servo_skip_counter = 0;

    // Bridge estimate: IIR-smoothed library measurement, only used until the
    // long window is valid (see the SERVO_LONG_* comment for why the raw
    // value is too jittery to apply directly).
    float lib_hz = spdif_rx_get_samp_freq_actual();
    if (servo_hz_smooth == 0.0f) servo_hz_smooth = lib_hz;
    else servo_hz_smooth += SERVO_SMOOTH_ALPHA * (lib_hz - servo_hz_smooth);

    // Long window: exact decoded-block count between two IRQ-timestamped
    // anchors (dual-anchor rotation, adat_input.c pattern). A lock drop
    // re-arms the anchors, so counts never span a library block_count reset.
    uint32_t blocks;
    uint64_t t_us;
    spdif_rx_get_block_ref(&blocks, &t_us);
    if (t_us - servo_long_us[1] >= SERVO_LONG_HALF_US) {
        servo_long_us[0] = servo_long_us[1];
        servo_long_blocks[0] = servo_long_blocks[1];
        servo_long_us[1] = t_us;
        servo_long_blocks[1] = blocks;
    }
    uint64_t span = t_us - servo_long_us[0];
    if (span >= SERVO_LONG_MIN_US) {
        uint32_t delta_blocks = blocks - servo_long_blocks[0];
        servo_hz_long = (float)delta_blocks * (float)(SPDIF_BLOCK_SIZE / 2) *
                        1e6f / (float)span;
        servo_long_valid = true;
    }

    // Divider math and PIO/MCK writes are shared (input_servo.c).
    input_servo_apply(servo_long_valid ? servo_hz_long : servo_hz_smooth);
}

uint32_t spdif_input_current_tx_divider(void) {
    return (spdif_state == SPDIF_INPUT_LOCKED) ? input_servo_current_divider() : 0;
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
        // Live count: with live-tail consumption the completed-block count
        // reads ~0 permanently, which would make this stat meaningless. Only
        // probe the DMA while LOCKED; during acquisition the channels may be
        // running the library's capture phase into a different buffer and the
        // in-flight probe would report garbage.
        uint32_t fifo_count = (spdif_state == SPDIF_INPUT_LOCKED)
                                  ? spdif_rx_get_fifo_count_live()
                                  : spdif_rx_get_fifo_count();
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
