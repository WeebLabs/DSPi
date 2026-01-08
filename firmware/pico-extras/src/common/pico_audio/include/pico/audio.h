/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_AUDIO_H
#define _PICO_AUDIO_H

#include "pico.h"
#include "pico/util/buffer.h"
#include "hardware/sync.h"

#ifdef __cplusplus
extern "C" {
#endif

/** \file audio.h
*  \defgroup pico_audio pico_audio
 *
 * Common API for audio output
 *
 */

// PICO_CONFIG: SPINLOCK_ID_AUDIO_FREE_LIST_LOCK, Spinlock number for the audio free list, min=0, max=31, default=6, group=audio
#ifndef SPINLOCK_ID_AUDIO_FREE_LIST_LOCK
#define SPINLOCK_ID_AUDIO_FREE_LIST_LOCK 6
#endif

// PICO_CONFIG: SPINLOCK_ID_AUDIO_PREPARED_LISTS_LOCK, Spinlock number for the audio prepared list, min=0, max=31, default=7, group=audio
#ifndef SPINLOCK_ID_AUDIO_PREPARED_LISTS_LOCK
#define SPINLOCK_ID_AUDIO_PREPARED_LISTS_LOCK 7
#endif

// PICO_CONFIG: PICO_AUDIO_NOOP, Enable/disable audio by forcing NOOPS, type=bool, default=0, group=audio
#ifndef PICO_AUDIO_NOOP
#define PICO_AUDIO_NOOP 0
#endif


#define AUDIO_BUFFER_FORMAT_PCM_S16 1          ///< signed 16bit PCM
#define AUDIO_BUFFER_FORMAT_PCM_S8 2           ///< signed 8bit PCM
#define AUDIO_BUFFER_FORMAT_PCM_U16 3          ///< unsigned 16bit PCM
#define AUDIO_BUFFER_FORMAT_PCM_U8 4           ///< unsigned 16bit PCM

/** \brief Audio format definition
 */
typedef struct audio_format {
    uint32_t sample_freq;      ///< Sample frequency in Hz
    uint16_t format;           ///< Audio format \ref audio_formats
    uint16_t channel_count;    ///< Number of channels
} audio_format_t;

/** \brief Audio buffer format definition
 */
typedef struct audio_buffer_format {
    const audio_format_t *format;      ///< Audio format
    uint16_t sample_stride;                 ///< Sample stride
} audio_buffer_format_t;

/** \brief Audio buffer definition
 */
typedef struct audio_buffer {
    mem_buffer_t *buffer;
    const audio_buffer_format_t *format;
    uint32_t sample_count;
    uint32_t max_sample_count;
    uint32_t user_data; // only valid while the user has the buffer
    // private - todo make an internal version
    struct audio_buffer *next;
} audio_buffer_t;

typedef struct audio_connection audio_connection_t;

typedef struct audio_buffer_pool {
    enum {
        ac_producer, ac_consumer
    } type;
    const audio_format_t *format;
    // private
    audio_connection_t *connection;
    spin_lock_t *free_list_spin_lock;
    // ----- begin protected by free_list_spin_lock -----
    audio_buffer_t *free_list;
    spin_lock_t *prepared_list_spin_lock;
    audio_buffer_t *prepared_list;
    audio_buffer_t *prepared_list_tail;
} audio_buffer_pool_t;

typedef struct audio_connection audio_connection_t;

struct audio_connection {
    audio_buffer_t *(*producer_pool_take)(audio_connection_t *connection, bool block);

    void (*producer_pool_give)(audio_connection_t *connection, audio_buffer_t *buffer);

    audio_buffer_t *(*consumer_pool_take)(audio_connection_t *connection, bool block);

    void (*consumer_pool_give)(audio_connection_t *connection, audio_buffer_t *buffer);

    audio_buffer_pool_t *producer_pool;
    audio_buffer_pool_t *consumer_pool;
};

/*! \brief Allocate and initialise an audio producer pool
 *  \ingroup pico_audio
 *
 * \param format Format of the audio buffer
 * \param buffer_count \todo
 * \param buffer_sample_count \todo
 * \return Pointer to an audio_buffer_pool
 */
audio_buffer_pool_t *audio_new_producer_pool(audio_buffer_format_t *format, int buffer_count,
                                                         int buffer_sample_count);

/*! \brief Allocate and initialise an audio consumer pool
 *  \ingroup pico_audio
 *
 * \param format Format of the audio buffer
 * \param buffer_count
 * \param buffer_sample_count
 * \return Pointer to an audio_buffer_pool
 */
audio_buffer_pool_t *audio_new_consumer_pool(audio_buffer_format_t *format, int buffer_count,
                                                         int buffer_sample_count);

/*! \brief Allocate and initialise an audio wrapping buffer
 *  \ingroup pico_audio
 *
 * \param format Format of the audio buffer
 * \param buffer \todo
 * \return Pointer to an audio_buffer
 */
audio_buffer_t *audio_new_wrapping_buffer(audio_buffer_format_t *format, mem_buffer_t *buffer);

/*! \brief Allocate and initialise an new audio buffer
 *  \ingroup pico_audio
 *
 * \param format Format of the audio buffer
 * \param buffer_sample_count \todo
 * \return Pointer to an audio_buffer
 */
audio_buffer_t *audio_new_buffer(audio_buffer_format_t *format, int buffer_sample_count);

/*! \brief Initialise an audio buffer
 *  \ingroup pico_audio
 *
 * \param audio_buffer Pointer to an audio_buffer
 * \param format Format of the audio buffer
 * \param buffer_sample_count \todo
 */
void audio_init_buffer(audio_buffer_t *audio_buffer, audio_buffer_format_t *format, int buffer_sample_count);

/*! \brief \todo
 *  \ingroup pico_audio
 *
 * \param ac \todo
 * \param buffer \todo
 * \return Pointer to an audio_buffer
 */
void give_audio_buffer(audio_buffer_pool_t *ac, audio_buffer_t *buffer);

/*! \brief \todo
 *  \ingroup pico_audio
 *
 * \return Pointer to an audio_buffer
 */
audio_buffer_t *take_audio_buffer(audio_buffer_pool_t *ac, bool block);

/*! \brief \todo
 *  \ingroup pico_audio
 *
 */
static inline void release_audio_buffer(audio_buffer_pool_t *ac, audio_buffer_t *buffer) {
    buffer->sample_count = 0;
    give_audio_buffer(ac, buffer);
}

/*! \brief \todo
 *  \ingroup pico_audio
 *
 * todo we are currently limited to 4095+1 input samples
 * step is fraction of an input sample per output sample * 0x1000 and should be < 0x1000 i.e. we we are up-sampling (otherwise results are undefined)
 */
void audio_upsample(int16_t *input, int16_t *output, uint output_count, uint32_t step);

/*! \brief \todo
 *  \ingroup pico_audio
 * similar but the output buffer is word aligned, and we output an even number of samples.. this is slightly faster than the above
 * todo we are currently limited to 4095+1 input samples
 * step is fraction of an input sample per output sample * 0x1000 and should be < 0x1000 i.e. we we are up-sampling (otherwise results are undefined)
 */
void audio_upsample_words(int16_t *input, int16_t *output_aligned, uint output_word_count, uint32_t step);

/*! \brief \todo
 *  \ingroup pico_audio
 */
void audio_upsample_double(int16_t *input, int16_t *output, uint output_count, uint32_t step);

/*! \brief \todo
 *  \ingroup pico_audio
 */
void audio_complete_connection(audio_connection_t *connection, audio_buffer_pool_t *producer,
                                      audio_buffer_pool_t *consumer);

/*! \brief \todo
 *  \ingroup pico_audio
 */
audio_buffer_t *get_free_audio_buffer(audio_buffer_pool_t *context, bool block);

/*! \brief \todo
 *  \ingroup pico_audio
 */
void queue_free_audio_buffer(audio_buffer_pool_t *context, audio_buffer_t *ab);

/*! \brief \todo
 *  \ingroup pico_audio
 */
audio_buffer_t *get_full_audio_buffer(audio_buffer_pool_t *context, bool block);

/*! \brief \todo
 *  \ingroup pico_audio
 */
void queue_full_audio_buffer(audio_buffer_pool_t *context, audio_buffer_t *ab);

/*! \brief \todo
 *  \ingroup pico_audio
 *
 * generally an pico_audio connection uses 3 of the defaults and does the hard work in one of them
 */
void consumer_pool_give_buffer_default(audio_connection_t *connection, audio_buffer_t *buffer);

/*! \brief \todo
 *  \ingroup pico_audio
 */
audio_buffer_t *consumer_pool_take_buffer_default(audio_connection_t *connection, bool block);

/*! \brief \todo
 *  \ingroup pico_audio
 */
void producer_pool_give_buffer_default(audio_connection_t *connection, audio_buffer_t *buffer);

/*! \brief \todo
 *  \ingroup pico_audio
 */
audio_buffer_t *producer_pool_take_buffer_default(audio_connection_t *connection, bool block);

enum audio_correction_mode {
    none,
    fixed_dither,
    dither,
    noise_shaped_dither,
};

struct buffer_copying_on_consumer_take_connection {
    struct audio_connection core;
    audio_buffer_t *current_producer_buffer;
    uint32_t current_producer_buffer_pos;
};

struct producer_pool_blocking_give_connection {
    audio_connection_t core;
    audio_buffer_t *current_consumer_buffer;
    uint32_t current_consumer_buffer_pos;
};

/*! \brief \todo
 *  \ingroup pico_audio
 */
audio_buffer_t *mono_to_mono_consumer_take(audio_connection_t *connection, bool block);

/*! \brief \todo
 *  \ingroup pico_audio
 */
audio_buffer_t *mono_s8_to_mono_consumer_take(audio_connection_t *connection, bool block);

/*! \brief \todo
 *  \ingroup pico_audio
 */
audio_buffer_t *stereo_to_stereo_consumer_take(audio_connection_t *connection, bool block);

/*! \brief \todo
 *  \ingroup pico_audio
 */
audio_buffer_t *mono_to_stereo_consumer_take(audio_connection_t *connection, bool block);

/*! \brief \todo
 *  \ingroup pico_audio
 */
audio_buffer_t *mono_s8_to_stereo_consumer_take(audio_connection_t *connection, bool block);

/*! \brief \todo
 *  \ingroup pico_audio
 */
void stereo_to_stereo_producer_give(audio_connection_t *connection, audio_buffer_t *buffer);

// not worth a separate header for now
typedef struct __packed pio_audio_channel_config {
    uint8_t base_pin;
    uint8_t dma_channel;
    uint8_t pio_sm;
} pio_audio_channel_config_t;

#ifdef __cplusplus
}
#endif

#endif //_AUDIO_H
