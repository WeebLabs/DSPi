/*
 * i2s_input.c — I2S receiver integration for DSPi (Pico-master)
 *
 * Mirrors spdif_input.c minus the servo and lock state machine. Because
 * BCK/LRCLK and the RX SM all derive from sys_clk via identical 24.8
 * dividers, input rate ≡ output rate by construction; no clock servo is
 * needed. Startup follows the same prefill-then-unmute pattern as SPDIF.
 *
 * Phase 2.5: clock-domain ownership is fully library-managed. This file
 * only deals with the RX instance lifecycle; the library's election runs
 * inside audio_i2s_rx_setup/teardown to install or uninstall the phantom
 * clkout-only SM as needed (used when no I2S TX output is registered).
 *
 * Hardware layout:
 *   RP2040: phantom on pio0/SM2 (BCK/LRCLK), RX on pio1/SM2 (DIN)
 *   RP2350: phantom on pio2/SM1 (BCK/LRCLK), RX on pio2/SM0 (DIN)
 *   DMA: channel 7 (free on both platforms in worst-case mixes)
 */

#include "i2s_input.h"
#include "audio_input.h"
#include "audio_pipeline.h"
#include "config.h"
#include "dsp_pipeline.h"
#include "usb_audio.h"

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "pico/audio_i2s_multi.h"
#include "pico/stdlib.h"

#include <stdio.h>
#include <string.h>

// ============================================================================
// HARDWARE ASSIGNMENT
// ============================================================================

// (Phantom clkout-only placement is configured by usb_sound_card_init in
//  usb_audio.c via audio_i2s_clock_domain_configure — see
//  I2S_CLOCK_DOMAIN_PHANTOM_PIO_IDX / I2S_CLOCK_DOMAIN_PHANTOM_SM in
//  config.h. The library handles all phantom lifecycle internally.)

// RX SM placement (samples DIN)
#if PICO_RP2350
#define I2S_INPUT_RX_PIO_IDX       2u
#define I2S_INPUT_RX_SM            0u
#else
#define I2S_INPUT_RX_PIO_IDX       1u
#define I2S_INPUT_RX_SM            2u
#endif

// DMA channel for the RX ring
#define I2S_INPUT_DMA_CHANNEL      7u

// Ring size (power of 2). 2048 words × 4 bytes = 8 KB = ~10 ms at 48 kHz
#define I2S_INPUT_RING_WORDS       2048u
#define I2S_INPUT_RING_BYTES       (I2S_INPUT_RING_WORDS * 4u)

// Default DIN pin (canonical definition in audio_input.h; this guard is
// just paranoia in case the include order changes).
#ifndef PICO_I2S_DIN_PIN_DEFAULT
#error "PICO_I2S_DIN_PIN_DEFAULT must be defined in audio_input.h"
#endif

// ============================================================================
// STATE
// ============================================================================

static volatile I2sInputState i2s_state = I2S_INPUT_INACTIVE;
static audio_i2s_rx_instance_t i2s_rx_inst;

// Caller-allocated ring buffer aligned to its size (DMA write-ring constraint)
static uint32_t i2s_ring[I2S_INPUT_RING_WORDS]
    __attribute__((aligned(I2S_INPUT_RING_BYTES)));

// DIN pin — device-level setting (lives in audio_input.c eventually like
// spdif_rx_pin; for Phase 2 keep here, expose via getter).
uint8_t i2s_din_pin = PICO_I2S_DIN_PIN_DEFAULT;

// ============================================================================
// PUBLIC API
// ============================================================================

void i2s_input_init(void) {
    i2s_state = I2S_INPUT_INACTIVE;
    memset(&i2s_rx_inst, 0, sizeof(i2s_rx_inst));
}

void i2s_input_start(void) {
    if (i2s_state != I2S_INPUT_INACTIVE) return;  // Idempotent

    uint32_t fs = audio_state.freq;

    // Set up the RX instance. The library's internal election runs at
    // the end of audio_i2s_rx_setup; if no I2S TX is registered, it
    // installs the phantom clkout-only SM to drive BCK/LRCLK. If a TX
    // is already running as master, the RX simply attaches as observer.
    audio_i2s_rx_config_t cfg = {
        .din_pin = i2s_din_pin,
        .pio_sm = I2S_INPUT_RX_SM,
        .pio = I2S_INPUT_RX_PIO_IDX,
        .dma_channel = I2S_INPUT_DMA_CHANNEL,
        .slot_count = 2,
        .slot_width_bits = 32,
        .reserved = {0, 0},
        .ring = i2s_ring,
        .ring_size_words = I2S_INPUT_RING_WORDS,
    };
    if (!audio_i2s_rx_setup(&i2s_rx_inst, &cfg, fs)) {
        printf("I2S input start: rx_setup failed\n");
        return;
    }

    // Enable RX (drains FIFO, arms DMA, starts SM at entry point — the
    // SM self-syncs on the next LRCLK low edge).
    audio_i2s_rx_set_enabled(&i2s_rx_inst, true);

    i2s_state = I2S_INPUT_STARTING;
    printf("I2S input: started (DIN GPIO %u @ %u Hz)\n",
           (uint)i2s_din_pin, (uint)fs);
}

void i2s_input_stop(void) {
    if (i2s_state == I2S_INPUT_INACTIVE) return;

    // Library's internal election runs during teardown; if this was the
    // last I2S participant, the phantom (if any) is uninstalled.
    audio_i2s_rx_teardown(&i2s_rx_inst);

    i2s_state = I2S_INPUT_INACTIVE;
    printf("I2S input: stopped\n");
}

I2sInputState i2s_input_get_state(void) {
    return i2s_state;
}

bool i2s_input_get_lock(void) {
    return i2s_state == I2S_INPUT_ACTIVE;
}

void i2s_input_set_frequency(uint32_t fs) {
    if (i2s_state == I2S_INPUT_INACTIVE) return;

    // The library's audio_i2s_update_all_frequencies (called from
    // perform_rate_change) updates phantom + all TX/RX dividers in lockstep.
    // This per-instance call kept for direct callers, in case a future
    // path wants to set the I2S input rate independently of outputs.
    bool was_enabled = i2s_rx_inst.active;
    if (was_enabled) audio_i2s_rx_set_enabled(&i2s_rx_inst, false);
    audio_i2s_rx_set_frequency(&i2s_rx_inst, fs);
    if (was_enabled) audio_i2s_rx_set_enabled(&i2s_rx_inst, true);
}

void i2s_input_mark_active(void) {
    if (i2s_state == I2S_INPUT_STARTING) {
        i2s_state = I2S_INPUT_ACTIVE;
    }
}

// ============================================================================
// MAIN-LOOP POLL — called every iteration when I2S is active input
// ============================================================================

DSP_TIME_CRITICAL
uint32_t i2s_input_poll(void) {
    if (i2s_state == I2S_INPUT_INACTIVE) return 0;
    if (!i2s_rx_inst.active) return 0;

    // Words available in ring = (write_idx - read_idx) modulo ring_size.
    uint32_t write_idx = audio_i2s_rx_get_write_idx(&i2s_rx_inst);
    uint32_t mask = i2s_rx_inst.ring_size_words - 1u;
    uint32_t available = (write_idx - i2s_rx_inst.read_idx_words) & mask;

    // Match SPDIF's natural ~192-sample batching pattern.  Without this gate
    // the main loop polls fast enough that each call only sees 2-10 stereo
    // pairs in the ring, and process_input_block() runs its full per-call
    // fixed overhead (pool takes, mute envelope, watermark, DSP setup) for
    // a tiny payload — pinning the CPU load reading at ~70% even though
    // the actual work is small.  Holding off until 192 stereo pairs are
    // accumulated amortises the fixed overhead across the same payload
    // size SPDIF naturally delivers, dropping the load reading into the
    // SPDIF-equivalent range.  4 ms input latency at 48 kHz — matches the
    // SPDIF path's IEC 60958 block cadence so feature parity is exact.
    //
    // Overflow guard: if the ring exceeds 75% of capacity (e.g. consumer
    // starved during a flash blackout), drain immediately rather than
    // waiting for the next batch boundary.
    const uint32_t target_batch_words   = 192u * 2u;
    const uint32_t overflow_words       = (i2s_rx_inst.ring_size_words * 3u) / 4u;
    if (available < target_batch_words && available < overflow_words) return 0;

    // Cap at 192 stereo frames to fit buf_l/buf_r (192 samples each)
    if (available > target_batch_words) available = target_batch_words;
    available &= ~1u;  // round down to stereo pairs

    // Apply preamp; preamp[0]=L, preamp[1]=R (matches USB and SPDIF input)
#if PICO_RP2350
    float preamp_l = global_preamp_linear[0];
    float preamp_r = global_preamp_linear[1];
    const float inv_2147483648 = 1.0f / 2147483648.0f;
#else
    int32_t preamp_l = global_preamp_mul[0];
    int32_t preamp_r = global_preamp_mul[1];
#endif

    uint32_t sample_idx = 0;
    uint32_t r = i2s_rx_inst.read_idx_words;

    for (uint32_t w = 0; w + 1u < available; w += 2u) {
        // Audio is left-justified MSB-first in 32-bit slots: bits[31:8] are
        // the 24-bit signed sample, bits[7:0] are zero. Masking off the low
        // byte preserves sign in the upper bits.
        int32_t raw_l = (int32_t)(i2s_rx_inst.ring[r] & 0xFFFFFF00u);
        int32_t raw_r = (int32_t)(i2s_rx_inst.ring[(r + 1u) & mask] & 0xFFFFFF00u);
        r = (r + 2u) & mask;

#if PICO_RP2350
        buf_l[sample_idx] = (float)raw_l * inv_2147483648 * preamp_l;
        buf_r[sample_idx] = (float)raw_r * inv_2147483648 * preamp_r;
#else
        // Q28: shift right 4 to map int32 full-scale to Q28, then scale.
        int32_t q28_l = raw_l >> 4;
        int32_t q28_r = raw_r >> 4;
        buf_l[sample_idx] = fast_mul_q28(q28_l, preamp_l);
        buf_r[sample_idx] = fast_mul_q28(q28_r, preamp_r);
#endif
        sample_idx++;
    }

    i2s_rx_inst.read_idx_words = r;

    if (sample_idx > 0) {
        process_input_block(sample_idx);
    }
    return sample_idx;
}

// ============================================================================
// STATUS QUERY
// ============================================================================

void i2s_input_get_status(I2sInputStatusPacket *out) {
    if (!out) return;
    memset(out, 0, sizeof(*out));
    out->state = (uint8_t)i2s_state;
    out->input_source = active_input_source;
    out->din_pin = i2s_din_pin;
    out->sample_rate = audio_state.freq;
    out->ring_size_words = (uint16_t)I2S_INPUT_RING_WORDS;

    if (i2s_rx_inst.active) {
        uint32_t write_idx = audio_i2s_rx_get_write_idx(&i2s_rx_inst);
        uint32_t mask = i2s_rx_inst.ring_size_words - 1u;
        uint32_t available = (write_idx - i2s_rx_inst.read_idx_words) & mask;
        out->ring_fill_words = available;
    }
}
