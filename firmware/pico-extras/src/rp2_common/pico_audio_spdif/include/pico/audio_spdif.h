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
 * This library uses the \ref pio system to implement a S/PDIF audio interface
 *
 * \todo Must be more we need to say here.
 * \todo certainly need an example
 *
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

#if !(PICO_AUDIO_SPDIF_DMA_IRQ == 0 || PICO_AUDIO_SPDIF_DMA_IRQ == 1)
#error PICO_AUDIO_SPDIF_DMA_IRQ must be 0 or 1
#endif

#if !(PICO_AUDIO_SPDIF_PIO == 0 || PICO_AUDIO_SPDIF_PIO == 1)
#error PICO_AUDIO_SPDIF_PIO ust be 0 or 1
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
//#warning PICO_AUDIO_SPDIF_PIN should be defined when using AUDIO_SPDIF
#define PICO_AUDIO_SPDIF_PIN 0
#endif

#define AUDIO_BUFFER_FORMAT_PIO_SPDIF 1300

// todo this needs to come from a build config
/** \brief Base configuration structure used when setting up
 * \ingroup audio_spdif
 */
typedef struct audio_spdif_config {
    uint8_t pin;
    uint8_t dma_channel;
    uint8_t pio_sm;
} audio_spdif_config_t;

extern const audio_spdif_config_t audio_spdif_default_config;

/** \brief Set up system to output S/PDIF audio
 * \ingroup audio_spdif
 *
 * \param intended_audio_format \todo
 * \param config The configuration to apply.
 */
const audio_format_t *audio_spdif_setup(const audio_format_t *intended_audio_format,
                                               const audio_spdif_config_t *config);


/** \brief \todo
 * \ingroup audio_spdif
 *
 * \param producer
 * \param connection
 */
bool audio_spdif_connect_thru(audio_buffer_pool_t *producer, audio_connection_t *connection);


/** \brief \todo
 * \ingroup audio_spdif
 *
 * \param producer
 */
bool audio_spdif_connect(audio_buffer_pool_t *producer);


/** \brief \todo
 * \ingroup audio_spdif
 *
 * \param producer
 */
bool audio_spdif_connect_s8(audio_buffer_pool_t *producer);
bool audio_spdif_connect_extra(audio_buffer_pool_t *producer, bool buffer_on_give, uint buffer_count,
                               audio_connection_t *connection);

/** \brief \todo
 * \ingroup audio_spdif
 *
 * \param producer
 * \param buffer_on_give
 * \param buffer_count
 * \param samples_per_buffer
 * \param connection
 * \return
 */
bool audio_spdif_connect_extra(audio_buffer_pool_t *producer, bool buffer_on_give, uint buffer_count,
                               audio_connection_t *connection);


/** \brief Set up system to output S/PDIF audio
 * \ingroup audio_spdif
 *
 * \param enabled true to enable S/PDIF audio, false to disable.
 */
void audio_spdif_set_enabled(bool enabled);

#ifdef __cplusplus
}
#endif

#endif //_AUDIO_SPDIF_H
