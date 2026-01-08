/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <cstring>
#include "pico/audio.h"
#include "pico/sample_conversion.h"

// ======================
// == DEBUGGING =========

#define ENABLE_AUDIO_ASSERTIONS

#ifdef ENABLE_AUDIO_ASSERTIONS
#define audio_assert(x) assert(x)
#else
#define audio_assert(x) (void)0
#endif

inline static audio_buffer_t *list_remove_head(audio_buffer_t **phead) {
    audio_buffer_t *ab = *phead;

    if (ab) {
        *phead = ab->next;
        ab->next = NULL;
    }

    return ab;
}

inline static audio_buffer_t *list_remove_head_with_tail(audio_buffer_t **phead,
                                                              audio_buffer_t **ptail) {
    audio_buffer_t *ab = *phead;

    if (ab) {
        *phead = ab->next;

        if (!ab->next) {
            audio_assert(*ptail == ab);
            *ptail = NULL;
        } else {
            ab->next = NULL;
        }
    }

    return ab;
}

inline static void list_prepend(audio_buffer_t **phead, audio_buffer_t *ab) {
    audio_assert(ab->next == NULL);
    audio_assert(ab != *phead);
    ab->next = *phead;
    *phead = ab;
}

// todo add a tail for these already sorted lists as we generally insert on the end
inline static void list_append_with_tail(audio_buffer_t **phead, audio_buffer_t **ptail,
                                         audio_buffer_t *ab) {
    audio_assert(ab->next == NULL);
    audio_assert(ab != *phead);
    audio_assert(ab != *ptail);

    if (!*phead) {
        audio_assert(!*ptail);
        *ptail = ab;
        // insert at the beginning
        list_prepend(phead, ab);
    } else {
        // insert at end
        (*ptail)->next = ab;
        *ptail = ab;
    }
}

audio_buffer_t *get_free_audio_buffer(audio_buffer_pool_t *context, bool block) {
    audio_buffer_t *ab;

    do {
        uint32_t save = spin_lock_blocking(context->free_list_spin_lock);
        ab = list_remove_head(&context->free_list);
        spin_unlock(context->free_list_spin_lock, save);
        if (ab || !block) break;
        __wfe();
    } while (true);
    return ab;
}

void queue_free_audio_buffer(audio_buffer_pool_t *context, audio_buffer_t *ab) {
    assert(!ab->next);
    uint32_t save = spin_lock_blocking(context->free_list_spin_lock);
    list_prepend(&context->free_list, ab);
    spin_unlock(context->free_list_spin_lock, save);
    __sev();
}

audio_buffer_t *get_full_audio_buffer(audio_buffer_pool_t *context, bool block) {
    audio_buffer_t *ab;

    do {
        uint32_t save = spin_lock_blocking(context->prepared_list_spin_lock);
        ab = list_remove_head_with_tail(&context->prepared_list, &context->prepared_list_tail);
        spin_unlock(context->prepared_list_spin_lock, save);
        if (ab || !block) break;
        __wfe();
    } while (true);
    return ab;
}

void queue_full_audio_buffer(audio_buffer_pool_t *context, audio_buffer_t *ab) {
    assert(!ab->next);
    uint32_t save = spin_lock_blocking(context->prepared_list_spin_lock);
    list_append_with_tail(&context->prepared_list, &context->prepared_list_tail, ab);
    spin_unlock(context->prepared_list_spin_lock, save);
    __sev();
}

void producer_pool_give_buffer_default(audio_connection_t *connection, audio_buffer_t *buffer) {
    queue_full_audio_buffer(connection->producer_pool, buffer);
}

audio_buffer_t *producer_pool_take_buffer_default(audio_connection_t *connection, bool block) {
    return get_free_audio_buffer(connection->producer_pool, block);
}

void consumer_pool_give_buffer_default(audio_connection_t *connection, audio_buffer_t *buffer) {
    queue_free_audio_buffer(connection->consumer_pool, buffer);
}

audio_buffer_t *consumer_pool_take_buffer_default(audio_connection_t *connection, bool block) {
    return get_full_audio_buffer(connection->consumer_pool, block);
}

static audio_connection_t connection_default = {
        .producer_pool_take = producer_pool_take_buffer_default,
        .producer_pool_give = producer_pool_give_buffer_default,
        .consumer_pool_take = consumer_pool_take_buffer_default,
        .consumer_pool_give = consumer_pool_give_buffer_default,
};

audio_buffer_t *audio_new_buffer(audio_buffer_format_t *format, int buffer_sample_count) {
    audio_buffer_t *buffer = (audio_buffer_t *) calloc(1, sizeof(audio_buffer_t));
    audio_init_buffer(buffer, format, buffer_sample_count);
    return buffer;
}

void audio_init_buffer(audio_buffer_t *audio_buffer, audio_buffer_format_t *format, int buffer_sample_count) {
    audio_buffer->format = format;
    audio_buffer->buffer = pico_buffer_alloc(buffer_sample_count * format->sample_stride);
    audio_buffer->max_sample_count = buffer_sample_count;
    audio_buffer->sample_count = 0;
}

audio_buffer_pool_t *
audio_new_buffer_pool(audio_buffer_format_t *format, int buffer_count, int buffer_sample_count) {
    audio_buffer_pool_t *ac = (audio_buffer_pool_t *) calloc(1, sizeof(audio_buffer_pool_t));
    audio_buffer_t *audio_buffers = buffer_count ? (audio_buffer_t *) calloc(buffer_count,
                                                                                       sizeof(audio_buffer_t)) : 0;
    ac->format = format->format;
    for (int i = 0; i < buffer_count; i++) {
        audio_init_buffer(audio_buffers + i, format, buffer_sample_count);
        audio_buffers[i].next = i != buffer_count - 1 ? &audio_buffers[i + 1] : NULL;
    }
    // todo one per channel?
    ac->free_list_spin_lock = spin_lock_init(SPINLOCK_ID_AUDIO_FREE_LIST_LOCK);
    ac->free_list = audio_buffers;
    ac->prepared_list_spin_lock = spin_lock_init(SPINLOCK_ID_AUDIO_PREPARED_LISTS_LOCK);
    ac->prepared_list = NULL;
    ac->prepared_list_tail = NULL;
    ac->connection = &connection_default;
    return ac;
}

audio_buffer_t *audio_new_wrapping_buffer(audio_buffer_format_t *format, mem_buffer_t *buffer) {
    audio_buffer_t *audio_buffer = (audio_buffer_t *) calloc(1, sizeof(audio_buffer_t));
    if (audio_buffer) {
        audio_buffer->format = format;
        audio_buffer->buffer = buffer;
        audio_buffer->max_sample_count = buffer->size / format->sample_stride;
        audio_buffer->sample_count = 0;
        audio_buffer->next = 0;
    }
    return audio_buffer;

}

audio_buffer_pool_t *
audio_new_producer_pool(audio_buffer_format_t *format, int buffer_count, int buffer_sample_count) {
    audio_buffer_pool_t *ac = audio_new_buffer_pool(format, buffer_count, buffer_sample_count);
    ac->type = audio_buffer_pool::ac_producer;
    return ac;
}

audio_buffer_pool_t *
audio_new_consumer_pool(audio_buffer_format_t *format, int buffer_count, int buffer_sample_count) {
    audio_buffer_pool_t *ac = audio_new_buffer_pool(format, buffer_count, buffer_sample_count);
    ac->type = audio_buffer_pool::ac_consumer;
    return ac;
}

void audio_complete_connection(audio_connection_t *connection, audio_buffer_pool_t *producer_pool,
                               audio_buffer_pool_t *consumer_pool) {
    assert(producer_pool->type == audio_buffer_pool::ac_producer);
    assert(consumer_pool->type == audio_buffer_pool::ac_consumer);
    producer_pool->connection = connection;
    consumer_pool->connection = connection;
    connection->producer_pool = producer_pool;
    connection->consumer_pool = consumer_pool;
}

void give_audio_buffer(audio_buffer_pool_t *ac, audio_buffer_t *buffer) {
    buffer->user_data = 0;
    assert(ac->connection);
    if (ac->type == audio_buffer_pool::ac_producer)
        ac->connection->producer_pool_give(ac->connection, buffer);
    else
        ac->connection->consumer_pool_give(ac->connection, buffer);
}

audio_buffer_t *take_audio_buffer(audio_buffer_pool_t *ac, bool block) {
    assert(ac->connection);
    if (ac->type == audio_buffer_pool::ac_producer)
        return ac->connection->producer_pool_take(ac->connection, block);
    else
        return ac->connection->consumer_pool_take(ac->connection, block);
}

// todo rename this - this is s16 to s16
audio_buffer_t *mono_to_mono_consumer_take(audio_connection_t *connection, bool block) {
    return consumer_pool_take<Mono<FmtS16>, Mono<FmtS16>>(connection, block);
}

// todo rename this - this is s16 to s16
audio_buffer_t *stereo_to_stereo_consumer_take(audio_connection_t *connection, bool block) {
    return consumer_pool_take<Stereo<FmtS16>, Stereo<FmtS16>>(connection, block);
}

// todo rename this - this is s16 to s16
audio_buffer_t *mono_to_stereo_consumer_take(audio_connection_t *connection, bool block) {
    return consumer_pool_take<Stereo<FmtS16>, Mono<FmtS16>>(connection, block);
}

audio_buffer_t *mono_s8_to_mono_consumer_take(audio_connection_t *connection, bool block) {
    return consumer_pool_take<Mono<FmtS16>, Mono<FmtS8>>(connection, block);
}

audio_buffer_t *mono_s8_to_stereo_consumer_take(audio_connection_t *connection, bool block) {
    return consumer_pool_take<Stereo<FmtS16>, Mono<FmtS8>>(connection, block);
}

void stereo_to_stereo_producer_give(audio_connection_t *connection, audio_buffer_t *buffer) {
    return producer_pool_blocking_give<Stereo<FmtS16>, Stereo<FmtS16>>(connection, buffer);
}
