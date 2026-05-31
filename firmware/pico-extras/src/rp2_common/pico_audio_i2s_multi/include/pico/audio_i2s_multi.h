/*
 * Copyright (c) 2026 WeebLabs
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_AUDIO_I2S_MULTI_H
#define _PICO_AUDIO_I2S_MULTI_H

#include "pico/audio.h"
#include "hardware/pio.h"

/** \file audio_i2s_multi.h
 *  \defgroup pico_audio_i2s_multi pico_audio_i2s_multi
 *  Multi-instance I2S audio output using the PIO
 *
 * This library uses the \ref pio system to implement multiple independent
 * I2S audio output interfaces. Each instance drives one stereo pair on its
 * own data pin, sharing BCK/LRCLK clock signals via PIO side-set.
 *
 * Follows the same architectural patterns as pico_audio_spdif_multi:
 * instance-based API, shared DMA IRQ handler, reference-counted IRQ
 * enable/disable, and synchronized multi-instance start.
 *
 * Audio format: 24-bit samples left-justified in 32-bit I2S frames.
 * Standard Philips I2S timing: MSB-first, 1 BCK delay after LRCLK edge.
 */

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// Configuration defaults
// ---------------------------------------------------------------------------

#ifndef PICO_AUDIO_I2S_DMA_IRQ
#ifdef PICO_AUDIO_SPDIF_DMA_IRQ
#define PICO_AUDIO_I2S_DMA_IRQ PICO_AUDIO_SPDIF_DMA_IRQ
#else
#define PICO_AUDIO_I2S_DMA_IRQ 1
#endif
#endif

/** Consumer buffer format identifier for I2S (raw PCM, not BMC-encoded) */
#define AUDIO_BUFFER_FORMAT_PIO_I2S 1301

/** Maximum number of I2S instances that can be registered */
#define PICO_AUDIO_I2S_MAX_INSTANCES 4

/** Samples per DMA transfer — matches SPDIF for pipeline compatibility */
#define PICO_AUDIO_I2S_DMA_SAMPLE_COUNT 48u

/** Bytes per stereo frame in the I2S consumer buffer (2 x int32). Must not exceed
 *  PICO_AUDIO_SPDIF_CONSUMER_FRAME_BYTES, which sizes the shared per-slot pool. */
#define PICO_AUDIO_I2S_CONSUMER_FRAME_BYTES 8u

// ---------------------------------------------------------------------------
// Instance structure
// ---------------------------------------------------------------------------

/** \brief Per-instance state for an I2S output
 * \ingroup pico_audio_i2s_multi
 *
 * Caller allocates and zero-initializes before passing to audio_i2s_setup().
 */
typedef struct audio_i2s_instance {
    // Hardware config (set in setup, immutable after)
    PIO pio;
    uint8_t pio_sm;
    uint8_t dma_channel;
    uint8_t dma_irq;            // 0 or 1
    uint8_t data_pin;           // Serial audio data GPIO
    uint8_t clock_pin_base;     // BCK GPIO; LRCLK = clock_pin_base + 1
    bool    clock_master;       // true = drives BCK/LRCLK, false = data only

    // Runtime state
    audio_buffer_t *playing_buffer;
    uint32_t freq;
    bool enabled;

    // DMA word tracking for USB feedback endpoint
    volatile uint32_t words_consumed;       // Total DMA words consumed (incremented in DMA IRQ)
    uint32_t current_transfer_words;        // DMA word count of current transfer

    // Per-instance audio pipeline
    audio_format_t consumer_format;
    audio_buffer_format_t consumer_buffer_format;
    audio_buffer_t silence_buffer;
    mem_buffer_t silence_mem;                   // static backing for silence_buffer (no heap)
    uint8_t silence_data[PICO_AUDIO_I2S_DMA_SAMPLE_COUNT * PICO_AUDIO_I2S_CONSUMER_FRAME_BYTES];
    audio_buffer_pool_t *consumer_pool;         // shared per-slot static pool (assigned by caller)

    // Embedded connection (uses container_of to recover instance pointer)
    struct producer_pool_blocking_give_connection connection;
} audio_i2s_instance_t;

// ---------------------------------------------------------------------------
// Configuration structure
// ---------------------------------------------------------------------------

/** \brief Configuration for an I2S output instance
 * \ingroup pico_audio_i2s_multi
 */
typedef struct audio_i2s_config {
    uint8_t data_pin;           // Serial audio data GPIO
    uint8_t clock_pin_base;     // BCK GPIO; LRCLK = clock_pin_base + 1
    uint8_t dma_channel;
    uint8_t pio_sm;
    uint8_t pio;                // PIO block index (0, 1, or 2 on RP2350)
    uint8_t dma_irq;            // DMA IRQ index (0 or 1)
    bool    clock_master;       // true = drive BCK/LRCLK (master), false = data only (slave)
} audio_i2s_config_t;

// ---------------------------------------------------------------------------
// Public API — mirrors pico_audio_spdif_multi
// ---------------------------------------------------------------------------

/** \brief Set up an I2S audio output instance
 * \ingroup pico_audio_i2s_multi
 *
 * Initializes the PIO state machine with the 24-bit I2S program, claims the
 * DMA channel, and registers the instance in the shared IRQ handler.
 *
 * All instances must share the same DMA IRQ line (asserted).
 * All instances on the same PIO block share the same BCK/LRCLK pins
 * (PIO side-set constraint).
 *
 * \param inst   Caller-allocated, zero-initialized instance
 * \param intended_audio_format  Desired audio format (48kHz stereo S32)
 * \param config Hardware configuration
 */
const audio_format_t *audio_i2s_setup(audio_i2s_instance_t *inst,
                                       const audio_format_t *intended_audio_format,
                                       const audio_i2s_config_t *config);

/** \brief Connect a producer pool to an I2S instance with extra options
 * \ingroup pico_audio_i2s_multi
 *
 * Re-formats the caller's consumer pool for I2S and establishes the
 * producer-to-consumer connection. The connection callback left-shifts 24-bit
 * samples into 32-bit I2S frames (MSB-aligned).
 *
 * \param inst           The I2S instance
 * \param producer       The producer buffer pool (PCM_S32, stride 8)
 * \param buffer_on_give If true, buffer on give side
 * \param consumer_pool  Caller-owned consumer pool (re-formatted for I2S here);
 *                       enables one static pool to be reused across output types
 * \param connection     Optional custom connection (NULL for default)
 */
bool audio_i2s_connect_extra(audio_i2s_instance_t *inst,
                              audio_buffer_pool_t *producer,
                              bool buffer_on_give, audio_buffer_pool_t *consumer_pool,
                              audio_connection_t *connection);

/** \brief Enable or disable an I2S output instance
 * \ingroup pico_audio_i2s_multi
 *
 * Uses reference-counted DMA IRQ enable/disable to support multiple instances.
 *
 * \param inst    The I2S instance
 * \param enabled true to enable, false to disable
 */
void audio_i2s_set_enabled(audio_i2s_instance_t *inst, bool enabled);

/** \brief Change the data pin of an I2S output instance
 * \ingroup pico_audio_i2s_multi
 *
 * The instance must be disabled before calling this function.
 * Aborts any stale DMA, releases the old data pin to high-Z,
 * reinitializes the PIO state machine with the new data pin,
 * and restores the clock divider.
 *
 * Clock pins (BCK/LRCLK) are NOT affected.
 *
 * \param inst    The I2S instance (must be disabled)
 * \param new_pin The new data GPIO pin number
 */
void audio_i2s_change_data_pin(audio_i2s_instance_t *inst, uint new_pin);

/** \brief Enable multiple I2S instances with synchronized PIO start
 * \ingroup pico_audio_i2s_multi
 *
 * Primes DMA for all instances, then starts all PIO state machines
 * simultaneously using pio_enable_sm_mask_in_sync().
 *
 * \param instances Array of pointers to initialized instances
 * \param count     Number of instances
 */
void audio_i2s_enable_sync(audio_i2s_instance_t *instances[], uint count);

/** \brief Tear down an I2S instance, releasing all hardware resources
 * \ingroup pico_audio_i2s_multi
 *
 * Disables the instance, aborts DMA, unclaims the DMA channel and PIO SM,
 * releases GPIO pins, and removes the instance from the IRQ handler registry.
 *
 * Used when switching an output slot from I2S back to S/PDIF.
 * The instance struct is zeroed and can be re-used with audio_i2s_setup().
 *
 * \param inst The I2S instance to tear down
 */
void audio_i2s_teardown(audio_i2s_instance_t *inst);

/** \brief Atomically update clock dividers for all I2S instances and restart in sync
 * \ingroup pico_audio_i2s_multi
 *
 * Stops all active I2S state machines, updates every registered instance's
 * clock divider, and restarts all in sync. Called from perform_rate_change()
 * to avoid the brief master/slave divider mismatch that occurs with lazy
 * per-instance updates.
 *
 * \param sample_freq New sample rate in Hz
 */
void audio_i2s_update_all_frequencies(uint32_t sample_freq);

/** \brief Get the index of the current I2S clock master (-1 if none)
 * \ingroup pico_audio_i2s_multi
 */
int8_t audio_i2s_get_clock_master_index(void);

// ---------------------------------------------------------------------------
// MCK (Master Clock) generator API
// ---------------------------------------------------------------------------
//
// MCK is generated by one of the four hardware CLK_GPOUTn clock outputs —
// no PIO state machine is consumed.  The GPIO must map to a CLK_GPOUTn on
// the current platform; use the SDK macro GPIO_TO_GPOUT_CLOCK_HANDLE() to
// validate before passing pins into this API:
//
//   if (GPIO_TO_GPOUT_CLOCK_HANDLE(pin, clk_sys) == clk_sys) { /* invalid */ }
//
// AUXSRC is fixed to clk_sys so the divider math is a single equation
// (sys_clk × 256 / (Fs × multiplier)) across both platforms.
//
// Lifecycle: setup() captures the pin without driving it; the GPOUTn block
// is configured on the first set_enabled(true) call.  Always call
// update_frequency() at least once before set_enabled(true) on a cold start
// so a sane divider is loaded — enabling with no divider is refused.

/** \brief Set up the MCK generator (records GPIO; does not drive the pin)
 * \ingroup pico_audio_i2s_multi
 *
 * The pin is captured but no hardware is configured until the first
 * audio_i2s_mck_set_enabled(true).  Caller must subsequently call
 * audio_i2s_mck_update_frequency() at least once before enabling, so
 * the GPOUTn block has a valid divider to load.
 *
 * \param pin GPIO for MCK output (must be CLK_GPOUTn-capable on the platform)
 */
void audio_i2s_mck_setup(uint pin);

/** \brief Enable or disable the MCK output
 * \ingroup pico_audio_i2s_multi
 *
 * On enable, configures the CLK_GPOUTn block (AUXSRC = clk_sys, last
 * computed 24.8 divider) and routes the pad mux.  On disable, disconnects
 * the pad mux so the pin floats; the generator continues running
 * internally (negligible quiescent draw — the SDK has no public API to
 * stop a CLK_GPOUTn generator).
 */
void audio_i2s_mck_set_enabled(bool enabled);

/** \brief Update MCK frequency for a new sample rate
 * \ingroup pico_audio_i2s_multi
 *
 * Computes the 24.8 divider as div = sys_clk × 256 / (sample_freq × multiplier).
 * If MCK is currently enabled, the divider is hot-loaded glitchlessly via
 * clock_gpio_init_int_frac8().  Otherwise the divider is stored and applied
 * on the next set_enabled(true).
 *
 * Ordering: always call this *before* set_enabled(true) on a cold start —
 * an enable with no divider configured is refused.
 *
 * \param sample_freq Sample rate in Hz (e.g., 48000)
 * \param multiplier  MCK multiplier: 128 or 256
 */
void audio_i2s_mck_update_frequency(uint32_t sample_freq, uint32_t multiplier);

/** \brief Change the MCK GPIO pin
 * \ingroup pico_audio_i2s_multi
 *
 * MCK must be disabled before calling.  The new pin takes effect on the
 * next audio_i2s_mck_set_enabled(true) — clock_gpio_init_int_frac8() at
 * that point routes the pad mux to whichever CLK_GPOUTn block the new pin
 * maps to.
 *
 * \param new_pin New GPIO for MCK output (must be CLK_GPOUTn-capable)
 */
void audio_i2s_mck_change_pin(uint new_pin);

/** \brief Get the current MCK GPIO pin (0xFF if setup() never called) */
uint8_t audio_i2s_mck_get_pin(void);

/** \brief Check if MCK is currently enabled */
bool audio_i2s_mck_is_enabled(void);

/** \brief Set the MCK divider directly (24.8 fixed-point)
 * \ingroup pico_audio_i2s_multi
 *
 * Used by the SPDIF input clock servo to keep MCK frequency-locked to
 * the servoed input data rate.  Bypasses the (sample_freq × multiplier)
 * computation in audio_i2s_mck_update_frequency().
 *
 * \param div_24_8 Clock divider in 24.8 fixed-point format
 */
void audio_i2s_mck_set_divider(uint32_t div_24_8);

/** \brief Atomically synchronize MCK library state with a desired tuple
 * \ingroup pico_audio_i2s_multi
 *
 * After preset / bulk-params apply paths update the firmware's "user
 * preference" globals (`i2s_mck_pin`, `i2s_mck_enabled`, …) the library's
 * internal state can be stale.  The vendor-command path keeps the two in
 * sync via audio_i2s_mck_change_pin() / audio_i2s_mck_set_enabled(); the
 * apply paths historically did not, which left MCK driving the wrong pin
 * after a preset load that customised mck_pin.
 *
 * This helper is the apply-path equivalent: it sequences
 *   1. (if pin changed) disable-then-change_pin (change_pin asserts !running),
 *   2. update_frequency for the requested Fs × multiplier (idempotent reload
 *      if already running on the same pin),
 *   3. set_enabled to the requested state,
 * so the library always matches the (pin, enabled, Fs, multiplier) tuple
 * the caller passes in.  Idempotent — calling twice with the same args is
 * a no-op on the second call.
 *
 * Caller is responsible for ensuring `pin` is CLK_GPOUTn-capable on the
 * current platform (use GPIO_TO_GPOUT_CLOCK_HANDLE() to validate).  If
 * audio_i2s_mck_setup() has not been called yet, this is a no-op.
 *
 * \param pin         Desired MCK GPIO (must map to CLK_GPOUTn)
 * \param enabled     Desired enabled state
 * \param sample_freq Sample rate in Hz (for divider math)
 * \param multiplier  MCK multiplier (128 or 256)
 */
void audio_i2s_mck_apply_state(uint pin, bool enabled,
                               uint32_t sample_freq, uint32_t multiplier);

#ifdef __cplusplus
}
#endif

#endif // _PICO_AUDIO_I2S_MULTI_H
