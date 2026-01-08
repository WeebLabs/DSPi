/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_AUDIO_I2S_H
#define _PICO_AUDIO_I2S_H

#include "pico/audio.h"

/** \file audio_i2s.h
 *  \defgroup pico_audio_i2s pico_audio_i2s
 *  I2S audio output using the PIO
 *
 * This library uses the \ref hardware_pio system to implement a I2S audio interface
 *
 * \todo Must be more we need to say here.
 * \todo certainly need an example
 *
 */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PICO_AUDIO_I2S_DMA_IRQ
#ifdef PICO_AUDIO_DMA_IRQ
#define PICO_AUDIO_I2S_DMA_IRQ PICO_AUDIO_DMA_IRQ
#else
#define PICO_AUDIO_I2S_DMA_IRQ 0
#endif
#endif

#ifndef PICO_AUDIO_I2S_PIO
#ifdef PICO_AUDIO_PIO
#define PICO_AUDIO_I2S_PIO PICO_AUDIO_PIO
#else
#define PICO_AUDIO_I2S_PIO 0
#endif
#endif

#if !(PICO_AUDIO_I2S_DMA_IRQ == 0 || PICO_AUDIO_I2S_DMA_IRQ == 1)
#error PICO_AUDIO_I2S_DMA_IRQ must be 0 or 1
#endif

#if !(PICO_AUDIO_I2S_PIO == 0 || PICO_AUDIO_I2S_PIO == 1)
#error PICO_AUDIO_I2S_PIO ust be 0 or 1
#endif

#ifndef PICO_AUDIO_I2S_MAX_CHANNELS
#ifdef PICO_AUDIO_MAX_CHANNELS
#define PICO_AUDIO_I2S_MAX_CHANNELS PICO_AUDIO_MAX_CHANNELS
#else
#define PICO_AUDIO_I2S_MAX_CHANNELS 2u
#endif
#endif

#ifndef PICO_AUDIO_I2S_BUFFERS_PER_CHANNEL
#ifdef PICO_AUDIO_BUFFERS_PER_CHANNEL
#define PICO_AUDIO_I2S_BUFFERS_PER_CHANNEL PICO_AUDIO_BUFFERS_PER_CHANNEL
#else
#define PICO_AUDIO_I2S_BUFFERS_PER_CHANNEL 3u
#endif
#endif

#ifndef PICO_AUDIO_I2S_BUFFER_SAMPLE_LENGTH
#ifdef PICO_AUDIO_BUFFER_SAMPLE_LENGTH
#define PICO_AUDIO_I2S_BUFFER_SAMPLE_LENGTH PICO_AUDIO_BUFFER_SAMPLE_LENGTH
#else
#define PICO_AUDIO_I2S_BUFFER_SAMPLE_LENGTH 576u
#endif
#endif

#ifndef PICO_AUDIO_I2S_SILENCE_BUFFER_SAMPLE_LENGTH
#ifdef PICO_AUDIO_I2S_SILENCE_BUFFER_SAMPLE_LENGTH
#define PICO_AUDIO_I2S_SILENCE_BUFFER_SAMPLE_LENGTH PICO_AUDIO_SILENCE_BUFFER_SAMPLE_LENGTH
#else
#define PICO_AUDIO_I2S_SILENCE_BUFFER_SAMPLE_LENGTH 256u
#endif
#endif

// Allow use of pico_audio driver without actually doing anything much
#ifndef PICO_AUDIO_I2S_NOOP
#ifdef PICO_AUDIO_NOOP
#define PICO_AUDIO_I2S_NOOP PICO_AUDIO_NOOP
#else
#define PICO_AUDIO_I2S_NOOP 0
#endif
#endif

#ifndef PICO_AUDIO_I2S_MONO_INPUT
#define PICO_AUDIO_I2S_MONO_INPUT 0
#endif
#ifndef PICO_AUDIO_I2S_MONO_OUTPUT
#define PICO_AUDIO_I2S_MONO_OUTPUT 0
#endif

#ifndef PICO_AUDIO_I2S_DATA_PIN
//#warning PICO_AUDIO_I2S_DATA_PIN should be defined when using AUDIO_I2S
#define PICO_AUDIO_I2S_DATA_PIN 28
#endif

#ifndef PICO_AUDIO_I2S_CLOCK_PIN_BASE
//#warning PICO_AUDIO_I2S_CLOCK_PIN_BASE should be defined when using AUDIO_I2S
#define PICO_AUDIO_I2S_CLOCK_PIN_BASE 26
#endif

// todo this needs to come from a build config
/** \brief Base configuration structure used when setting up
 * \ingroup pico_audio_i2s
 */
typedef struct audio_i2s_config {
    uint8_t data_pin;
    uint8_t clock_pin_base;
    uint8_t dma_channel;
    uint8_t pio_sm;
} audio_i2s_config_t;

/** \brief Set up system to output I2S audio
 * \ingroup pico_audio_i2s
 *
 * \param intended_audio_format \todo
 * \param config The configuration to apply.
 */
const audio_format_t *audio_i2s_setup(const audio_format_t *intended_audio_format,
                                               const audio_i2s_config_t *config);


/** \brief \todo
 * \ingroup pico_audio_i2s
 *
 * \param producer
 * \param connection
 */
bool audio_i2s_connect_thru(audio_buffer_pool_t *producer, audio_connection_t *connection);


/** \brief \todo
 * \ingroup pico_audio_i2s
 *
 * \param producer
 *
 *  todo make a common version (or a macro) .. we don't want to pull in unnecessary code by default
 */
bool audio_i2s_connect(audio_buffer_pool_t *producer);


/** \brief \todo
 * \ingroup pico_audio_i2s
 *
 * \param producer
 */
bool audio_i2s_connect_s8(audio_buffer_pool_t *producer);
bool audio_i2s_connect_extra(audio_buffer_pool_t *producer, bool buffer_on_give, uint buffer_count, uint samples_per_buffer, audio_connection_t *connection);

/** \brief \todo
 * \ingroup pico_audio_i2s
 *
 * \param producer
 * \param buffer_on_give
 * \param buffer_count
 * \param samples_per_buffer
 * \param connection
 * \return
 */
bool audio_i2s_connect_extra(audio_buffer_pool_t *producer, bool buffer_on_give, uint buffer_count,
                                 uint samples_per_buffer, audio_connection_t *connection);


/** \brief Set up system to output I2S audio
 * \ingroup pico_audio_i2s
 *
 * \param enable true to enable I2S audio, false to disable.
 */
void audio_i2s_set_enabled(bool enabled);

#ifdef __cplusplus
}
#endif

#endif //_AUDIO_I2S_H
