/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <cstdio>

#include "pico/sample_conversion.h"
#include "pico/audio_spdif/sample_encoding.h"
#include "pico/audio_spdif.h"
#include "hardware/gpio.h"

static_assert(8 == sizeof(spdif_subframe_t), "");

// subframe within SPDIF
typedef struct : public FmtDetails<spdif_subframe_t> {
} FmtSPDIF;

template<typename FromFmt>
struct converting_copy<Stereo<FmtSPDIF>, Stereo<FromFmt>> {
    static void copy(FmtSPDIF::sample_t *dest, const typename FromFmt::sample_t *src, uint sample_count) {
        for (uint i = 0; i < sample_count * 2; i++) {
            spdif_update_subframe(dest++, sample_converter<FmtS16 , FromFmt>::convert_sample(*src++));
        }
    }
};

template<typename FromFmt>
struct converting_copy<Stereo<FmtSPDIF>, Mono<FromFmt>> {
    static void copy(FmtSPDIF::sample_t *dest, const typename FromFmt::sample_t *src, uint sample_count) {
        for (uint i = 0; i < sample_count; i++) {
            int16_t sample = sample_converter<FmtS16 ,FromFmt>::convert_sample(*src++);
            spdif_update_subframe(dest++, sample);
            spdif_update_subframe(dest++, sample);
        }
    }
};


void stereo_to_spdif_producer_give(audio_connection_t *connection, audio_buffer_t *buffer) {
    producer_pool_blocking_give<Stereo<FmtSPDIF>, Stereo<FmtS16>>(connection, buffer);
}

void mono_to_spdif_producer_give(audio_connection_t *connection, audio_buffer_t *buffer) {
    producer_pool_blocking_give<Stereo<FmtSPDIF>, Mono<FmtS16>>(connection, buffer);
}




