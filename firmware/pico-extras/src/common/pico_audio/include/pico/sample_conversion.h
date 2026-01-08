/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SOFTWARE_SAMPLE_CONVERSION_H
#define SOFTWARE_SAMPLE_CONVERSION_H

#include <algorithm>
#include <cstring>
#include "pico/audio.h"
#include "pico/util/buffer.h"

template<typename _sample_t>
struct FmtDetails {
public:
    static const uint channel_count = 1;
    static const uint frame_stride = channel_count * sizeof(_sample_t);
    typedef _sample_t sample_t;
};

typedef struct : public FmtDetails<uint8_t> {
} FmtU8;

typedef struct : public FmtDetails<int8_t> {
} FmtS8;

typedef struct : public FmtDetails<uint16_t> {
} FmtU16;

typedef struct : public FmtDetails<int16_t> {
} FmtS16;

// Multi channel is just N samples back to back
template<typename Fmt, uint ChannelCount>
struct MultiChannelFmt {
    static const uint channel_count = ChannelCount;
    static const uint frame_stride = ChannelCount * Fmt::frame_stride;
    typedef typename Fmt::sample_t sample_t;
};

// define Mono<X> details as one channel
template<typename Fmt> using Mono = MultiChannelFmt<Fmt, 1>;

// define Stereo<X> details as two channels
template<typename Fmt> using Stereo = MultiChannelFmt<Fmt, 2>;

template<typename ToFmt, typename FromFmt>
struct sample_converter {
    static typename ToFmt::sample_t convert_sample(const typename FromFmt::sample_t &sample);
};

// noop conversion

template<typename Fmt>
struct sample_converter<Fmt, Fmt> {
    static typename Fmt::sample_t convert_sample(const typename Fmt::sample_t &sample) {
        return sample;
    }
};

// converters to S16
template<>
struct sample_converter<FmtS16, FmtU16> {
    static int16_t convert_sample(const uint16_t &sample) {
        return sample ^ 0x8000u;
    }
};

template<>
struct sample_converter<FmtS16, FmtS8> {
    static int16_t convert_sample(const int8_t &sample) {
        return sample << 8u;
    }
};

template<>
struct sample_converter<FmtS16, FmtU8> {
    static int16_t convert_sample(const uint8_t &sample) {
        return (sample << 8u) ^ 0x8000u;
    }
};

// converters to U16

template<>
struct sample_converter<FmtU16, FmtS8> {
    static uint16_t convert_sample(const int8_t &sample) {
        return (sample << 8u) ^ 0x8000u;
    }
};

template<>
struct sample_converter<FmtU16, FmtU8> {
    static uint16_t convert_sample(const uint8_t &sample) {
        return sample << 8u;
    }
};

template<>
struct sample_converter<FmtU16, FmtS16> {
    static uint16_t convert_sample(const int16_t &sample) {
        return sample ^ 0x8000u;
    }
};

// converters to S8

template<>
struct sample_converter<FmtS8, FmtU16> {
    static int8_t convert_sample(const uint16_t &sample) {
        return (sample ^ 0x8000u) >> 8u;
    }
};

template<>
struct sample_converter<FmtS8, FmtU8> {
    static int8_t convert_sample(const uint8_t &sample) {
        return sample ^ 0x80;
    }
};

template<>
struct sample_converter<FmtS8, FmtS16> {
    static int8_t convert_sample(const int16_t &sample) {
        return sample >> 8u;
    }
};

// converters to U8

template<>
struct sample_converter<FmtU8, FmtU16> {
    static uint8_t convert_sample(const uint16_t &sample) {
        return sample >> 8u;
    }
};

template<>
struct sample_converter<FmtU8, FmtS8> {
    static uint8_t convert_sample(const int8_t &sample) {
        return sample ^ 0x80;
    }
};

template<>
struct sample_converter<FmtU8, FmtS16> {
    static uint8_t convert_sample(const int16_t &sample) {
        return (sample ^ 0x8000u) >> 8u;
    }
};

// template type for doing sample conversion
template<typename ToFmt, typename FromFmt>
struct converting_copy {
    static void copy(typename ToFmt::sample_t *dest, const typename FromFmt::sample_t *src, uint sample_count);
};

// Efficient copies of same sample type

template<class Fmt, uint ChannelCount>
struct converting_copy<MultiChannelFmt<Fmt, ChannelCount>, MultiChannelFmt<Fmt, ChannelCount>> {
    static void copy(typename MultiChannelFmt<Fmt, ChannelCount>::sample_t *dest,
                     const typename MultiChannelFmt<Fmt, ChannelCount>::sample_t *src,
                     uint sample_count) {
        memcpy((void *) dest, (const void *) src, sample_count * MultiChannelFmt<Fmt, ChannelCount>::frame_stride);
    }
};

// N channel to N channel
template<typename ToFmt, typename FromFmt, uint NumChannels>
struct converting_copy<MultiChannelFmt<ToFmt, NumChannels>, MultiChannelFmt<FromFmt, NumChannels>> {
    static void copy(typename ToFmt::sample_t *dest, const typename FromFmt::sample_t *src, uint sample_count) {
        for (uint i = 0; i < sample_count * NumChannels; i++) {
            *dest++ = sample_converter<ToFmt, FromFmt>::convert_sample(*src++);
        }
    }
};


// mono->stereo conversion
template<typename ToFmt, typename FromFmt>
struct converting_copy<Stereo<ToFmt>, Mono<FromFmt>> {
    static void copy(typename ToFmt::sample_t *dest, const typename FromFmt::sample_t *src, uint sample_count) {
        for (; sample_count; sample_count--) {
            typename ToFmt::sample_t mono_sample = sample_converter<ToFmt, FromFmt>::convert_sample(*src++);
            *dest++ = mono_sample;
            *dest++ = mono_sample;
        }
    }
};

// stereo->mono conversion
template<typename ToFmt, typename FromFmt>
struct converting_copy<Mono<ToFmt>, Stereo<FromFmt>> {
    static void copy(typename ToFmt::sample_t *dest, const typename FromFmt::sample_t *src, uint sample_count) {
        for (; sample_count; sample_count--) {
            // average first in case precision is better in source
            typename FromFmt::sample_t averaged_sample = (src[0] + src[1]) / 2;
            src += 2;
            *dest++ = sample_converter<ToFmt, FromFmt>::convert_sample(averaged_sample);
        }
    }
};

template<typename ToFmt, typename FromFmt>
audio_buffer_t *consumer_pool_take(audio_connection_t *connection, bool block) {
    struct buffer_copying_on_consumer_take_connection *cc = (struct buffer_copying_on_consumer_take_connection *) connection;
    // for now we block until we have all the data in consumer buffers
    audio_buffer_t *buffer = get_free_audio_buffer(cc->core.consumer_pool, block);
    if (!buffer) return NULL;
    assert(buffer->format->sample_stride == ToFmt::frame_stride);

    uint32_t pos = 0;
    while (pos < buffer->max_sample_count) {
        if (!cc->current_producer_buffer) {
            cc->current_producer_buffer = get_full_audio_buffer(cc->core.producer_pool, block);
            if (!cc->current_producer_buffer) {
                assert(!block);
                if (!pos) {
                    queue_free_audio_buffer(cc->core.consumer_pool, buffer);
                    return NULL;
                }
                break;
            }
            assert(cc->current_producer_buffer->format->format->channel_count == FromFmt::channel_count);
            assert(cc->current_producer_buffer->format->sample_stride == FromFmt::frame_stride);
            cc->current_producer_buffer_pos = 0;
        }
        uint sample_count = std::min(buffer->max_sample_count - pos,
                                     cc->current_producer_buffer->sample_count - cc->current_producer_buffer_pos);
        converting_copy<ToFmt, FromFmt>::copy(
                ((typename ToFmt::sample_t *) buffer->buffer->bytes) + pos * ToFmt::channel_count,
                ((typename FromFmt::sample_t *) cc->current_producer_buffer->buffer->bytes) +
                cc->current_producer_buffer_pos * FromFmt::channel_count,
                sample_count);
        pos += sample_count;
        cc->current_producer_buffer_pos += sample_count;
        if (cc->current_producer_buffer_pos == cc->current_producer_buffer->sample_count) {
            queue_free_audio_buffer(cc->core.producer_pool, cc->current_producer_buffer);
            cc->current_producer_buffer = NULL;
        }
    }
    buffer->sample_count = pos;
    return buffer;
}

template<typename ToFmt, typename FromFmt>
void producer_pool_blocking_give(audio_connection_t *connection, audio_buffer_t *buffer) {
    struct producer_pool_blocking_give_connection *pbc = (struct producer_pool_blocking_give_connection *) connection;
    // for now we block until we have all the data in consumer buffers
    uint32_t pos = 0;
    while (pos < buffer->sample_count) {
        if (!pbc->current_consumer_buffer) {
            pbc->current_consumer_buffer = get_free_audio_buffer(pbc->core.consumer_pool, true);
            pbc->current_consumer_buffer_pos = 0;
        }
        uint sample_count = std::min(buffer->sample_count - pos,
                                     pbc->current_consumer_buffer->max_sample_count - pbc->current_consumer_buffer_pos);
        assert(buffer->format->sample_stride == FromFmt::frame_stride);
        assert(buffer->format->format->channel_count == FromFmt::channel_count);
        converting_copy<ToFmt, FromFmt>::copy(
                ((typename ToFmt::sample_t *) pbc->current_consumer_buffer->buffer->bytes) +
                pbc->current_consumer_buffer_pos * ToFmt::channel_count,
                ((typename FromFmt::sample_t *) buffer->buffer->bytes) + pos * FromFmt::channel_count, sample_count);
        pos += sample_count;
        pbc->current_consumer_buffer_pos += sample_count;
        if (pbc->current_consumer_buffer_pos == pbc->current_consumer_buffer->max_sample_count) {
            pbc->current_consumer_buffer->sample_count = pbc->current_consumer_buffer->max_sample_count;
            queue_full_audio_buffer(pbc->core.consumer_pool, pbc->current_consumer_buffer);
            pbc->current_consumer_buffer = NULL;
        }
    }
    // todo this should be a connection configuration (or a seaparate connection type)
#ifdef BLOCKING_GIVE_SYNCHRONIZE_BUFFERS
    if (pbc->current_consumer_buffer) {
        pbc->current_consumer_buffer->sample_count = pbc->current_consumer_buffer_pos;
        queue_full_audio_buffer(pbc->core.consumer_pool, pbc->current_consumer_buffer);
        pbc->current_consumer_buffer = NULL;
    }
#endif
    assert(pos == buffer->sample_count);
    queue_free_audio_buffer(pbc->core.producer_pool, buffer);
}

#endif //SOFTWARE_SAMPLE_CONVERSION_H
