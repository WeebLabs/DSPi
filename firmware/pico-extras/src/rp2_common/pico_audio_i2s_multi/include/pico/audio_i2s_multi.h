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
    audio_buffer_pool_t *consumer_pool;

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
 * Creates the consumer buffer pool and establishes the producer-to-consumer
 * connection. The connection callback left-shifts 24-bit samples into 32-bit
 * I2S frames (MSB-aligned).
 *
 * \param inst           The I2S instance
 * \param producer       The producer buffer pool (PCM_S32, stride 8)
 * \param buffer_on_give If true, buffer on give side
 * \param buffer_count   Number of consumer buffers to allocate
 * \param connection     Optional custom connection (NULL for default)
 */
bool audio_i2s_connect_extra(audio_i2s_instance_t *inst,
                              audio_buffer_pool_t *producer,
                              bool buffer_on_give, uint buffer_count,
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

/** \brief Set up the MCK generator on a PIO state machine
 * \ingroup pico_audio_i2s_multi
 *
 * Loads the MCK toggle program and claims the specified SM.
 * MCK starts disabled; call audio_i2s_mck_set_enabled(true) to start.
 *
 * Intended for PIO1 SM1 (SM0 is PDM output).
 *
 * \param pio PIO block (typically pio1)
 * \param sm  State machine index (typically 1)
 * \param pin GPIO for MCK output
 */
void audio_i2s_mck_setup(PIO pio, uint sm, uint pin);

/** \brief Enable or disable the MCK output
 * \ingroup pico_audio_i2s_multi
 */
void audio_i2s_mck_set_enabled(bool enabled);

/** \brief Update MCK frequency for a new sample rate
 * \ingroup pico_audio_i2s_multi
 *
 * Computes the PIO clock divider for the given sample rate and multiplier.
 * If MCK is currently enabled, the frequency changes immediately.
 *
 * \param sample_freq Sample rate in Hz (e.g., 48000)
 * \param multiplier  MCK multiplier: 128 or 256
 */
void audio_i2s_mck_update_frequency(uint32_t sample_freq, uint32_t multiplier);

/** \brief Change the MCK GPIO pin
 * \ingroup pico_audio_i2s_multi
 *
 * MCK must be disabled before calling. The new pin takes effect
 * on the next audio_i2s_mck_set_enabled(true).
 *
 * \param new_pin New GPIO for MCK output
 */
void audio_i2s_mck_change_pin(uint new_pin);

/** \brief Get the current MCK GPIO pin */
uint8_t audio_i2s_mck_get_pin(void);

/** \brief Check if MCK is currently enabled */
bool audio_i2s_mck_is_enabled(void);

#ifdef __cplusplus
}
#endif

#endif // _PICO_AUDIO_I2S_MULTI_H
