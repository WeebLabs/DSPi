/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_AUDIO_SPDIF_SAMPLE_ENCODING_H
#define _PICO_AUDIO_SPDIF_SAMPLE_ENCODING_H

#include "pico/audio.h"

#ifdef __cplusplus
extern "C" {
#endif

void mono_to_spdif_producer_give(audio_connection_t *connection, audio_buffer_t *buffer);
void stereo_to_spdif_producer_give(audio_connection_t *connection, audio_buffer_t *buffer);

typedef struct {
    uint32_t l;
    uint32_t h;
} spdif_subframe_t;

extern uint32_t spdif_lookup[256];

static inline void spdif_update_subframe(spdif_subframe_t *subframe, int16_t sample) {
    // the subframe is partially initialized, so we need to insert the sample
    // bits and update the parity
    uint32_t sl = spdif_lookup[(uint8_t)sample];
    uint32_t sh = spdif_lookup[(uint8_t)(sample>>8u)];
    subframe->l = (subframe->l & 0xffffffu) | (sl << 24u);
    uint32_t ph = subframe->h >> 24u;
    uint32_t h = (((uint16_t)sh) << 8u) |
                 (((uint16_t)sl) >> 8u);
    uint32_t p = (sl>>16u)^(sh>>16u);
    p = p ^ ((__mul_instruction(ph&0x2a,0x2a) >> 6u) & 1u);
    subframe->h = h | ((ph&0x7f) << 24u) | (p << 31u);
}

#ifdef __cplusplus
}
#endif

#endif //SOFTWARE_SAMPLE_ENCODING_H
