/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_AUDIO_SPDIF_H
#define _PICO_AUDIO_SPDIF_H

#include "pico/audio.h"

/** \file audio_spdif.h
 *  \defgroup pico_audio_spdif pico_audio_spdif
 *  S/PDIF audio output using the PIO
 *
 * This library uses the \ref pio system to implement a S/PDIF audio interface.
 * Multiple instances can operate concurrently on independent PIO SMs/DMA
 * channels/GPIO pins.
 */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PICO_AUDIO_SPDIF_DMA_IRQ
#ifdef PICO_AUDIO_DMA_IRQ
#define PICO_AUDIO_SPDIF_DMA_IRQ PICO_AUDIO_DMA_IRQ
#else
#define PICO_AUDIO_SPDIF_DMA_IRQ 0
#endif
#endif

#ifndef PICO_AUDIO_SPDIF_PIO
#ifdef PICO_AUDIO_PIO
#define PICO_AUDIO_SPDIF_PIO PICO_AUDIO_PIO
#else
#define PICO_AUDIO_SPDIF_PIO 0
#endif
#endif

#ifndef PICO_AUDIO_SPDIF_MAX_CHANNELS
#ifdef PICO_AUDIO_MAX_CHANNELS
#define PICO_AUDIO_SPDIF_MAX_CHANNELS PICO_AUDIO_MAX_CHANNELS
#else
#define PICO_AUDIO_SPDIF_MAX_CHANNELS 2u
#endif
#endif

#ifndef PICO_AUDIO_SPDIF_BUFFERS_PER_CHANNEL
#ifdef PICO_AUDIO_BUFFERS_PER_CHANNEL
#define PICO_AUDIO_SPDIF_BUFFERS_PER_CHANNEL PICO_AUDIO_BUFFERS_PER_CHANNEL
#else
#define PICO_AUDIO_SPDIF_BUFFERS_PER_CHANNEL 3u
#endif
#endif

// fixed by S/PDIF
#define PICO_AUDIO_SPDIF_BLOCK_SAMPLE_COUNT 192u

// Allow use of pico_audio driver without actually doing anything much
#ifndef PICO_AUDIO_SPDIF_NOOP
#ifdef PICO_AUDIO_NOOP
#define PICO_AUDIO_SPDIF_NOOP PICO_AUDIO_NOOP
#else
#define PICO_AUDIO_SPDIF_NOOP 0
#endif
#endif

#ifndef PICO_AUDIO_SPDIF_MONO_INPUT
#define PICO_AUDIO_SPDIF_MONO_INPUT 0
#endif

#ifndef PICO_AUDIO_SPDIF_PIN
#define PICO_AUDIO_SPDIF_PIN 0
#endif

#define AUDIO_BUFFER_FORMAT_PIO_SPDIF 1300

#define PICO_AUDIO_SPDIF_MAX_INSTANCES 4

#include "hardware/pio.h"

/** \brief Per-instance state for an S/PDIF output
 * \ingroup audio_spdif
 */
typedef struct audio_spdif_instance {
    // Hardware config (set in setup, immutable after)
    PIO pio;
    uint8_t pio_sm;
    uint8_t dma_channel;
    uint8_t dma_irq;            // 0 or 1
    uint8_t pin;

    // Runtime state
    audio_buffer_t *playing_buffer;
    uint32_t freq;
    bool enabled;

    // Per-instance audio pipeline
    audio_format_t consumer_format;
    audio_buffer_format_t consumer_buffer_format;
    audio_buffer_t silence_buffer;
    audio_buffer_pool_t *consumer_pool;

    // Embedded connection
    struct producer_pool_blocking_give_connection connection;
} audio_spdif_instance_t;

/** \brief Configuration for an S/PDIF output instance
 * \ingroup audio_spdif
 */
typedef struct audio_spdif_config {
    uint8_t pin;
    uint8_t dma_channel;
    uint8_t pio_sm;
    uint8_t pio;        // PIO block index (0, 1, or 2 on RP2350)
    uint8_t dma_irq;    // DMA IRQ index (0 or 1)
} audio_spdif_config_t;

/** \brief Set up an S/PDIF audio output instance
 * \ingroup audio_spdif
 *
 * \param inst The instance to initialize (caller-allocated, zero-initialized)
 * \param intended_audio_format The desired audio format
 * \param config The hardware configuration to apply
 */
const audio_format_t *audio_spdif_setup(audio_spdif_instance_t *inst,
                                        const audio_format_t *intended_audio_format,
                                        const audio_spdif_config_t *config);

/** \brief Connect a producer pool to an S/PDIF instance with a pass-through connection
 * \ingroup audio_spdif
 */
bool audio_spdif_connect_thru(audio_spdif_instance_t *inst,
                              audio_buffer_pool_t *producer,
                              audio_connection_t *connection);

/** \brief Connect a producer pool to an S/PDIF instance using the default connection
 * \ingroup audio_spdif
 */
bool audio_spdif_connect(audio_spdif_instance_t *inst, audio_buffer_pool_t *producer);

/** \brief Connect a producer pool to an S/PDIF instance with extra options
 * \ingroup audio_spdif
 *
 * \param inst The S/PDIF instance
 * \param producer The producer buffer pool
 * \param buffer_on_give If true, buffer on give side
 * \param buffer_count Number of consumer buffers to allocate
 * \param connection Optional custom connection (NULL for default)
 */
bool audio_spdif_connect_extra(audio_spdif_instance_t *inst,
                               audio_buffer_pool_t *producer,
                               bool buffer_on_give, uint buffer_count,
                               audio_connection_t *connection);

/** \brief Enable or disable an S/PDIF output instance
 * \ingroup audio_spdif
 *
 * \param inst The S/PDIF instance
 * \param enabled true to enable, false to disable
 */
void audio_spdif_set_enabled(audio_spdif_instance_t *inst, bool enabled);

/** \brief Enable multiple S/PDIF instances with synchronized PIO start
 * \ingroup audio_spdif
 *
 * Primes DMA for all instances, then starts all PIO state machines
 * simultaneously using pio_enable_sm_mask_in_sync(). SMs on the same
 * PIO block start on the exact same clock cycle.
 *
 * \param instances Array of pointers to initialized instances
 * \param count Number of instances
 */
void audio_spdif_enable_sync(audio_spdif_instance_t *instances[], uint count);

#ifdef __cplusplus
}
#endif

#endif //_AUDIO_SPDIF_H
