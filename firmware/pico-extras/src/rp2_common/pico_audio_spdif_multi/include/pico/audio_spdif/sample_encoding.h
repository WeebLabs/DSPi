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
void stereo_to_spdif_producer_give_s32(audio_connection_t *connection, audio_buffer_t *buffer);

typedef struct {
    uint32_t l;
    uint32_t h;
} spdif_subframe_t;

extern uint32_t spdif_lookup[256];

static inline void spdif_update_subframe(spdif_subframe_t *subframe, int32_t sample) {
    // Encode 24-bit audio (bits [23:0] of sample) into SPDIF subframe.
    // 3 lookup accesses — one per byte of 24-bit sample.
    uint32_t s0 = spdif_lookup[(uint8_t)sample];           // byte 0 → subframe bits 4-11
    uint32_t s1 = spdif_lookup[(uint8_t)(sample >> 8u)];   // byte 1 → subframe bits 12-19
    uint32_t s2 = spdif_lookup[(uint8_t)(sample >> 16u)];  // byte 2 → subframe bits 20-27

    // l[7:0]=preamble preserved, l[23:8]=s0 BMC, l[31:24]=s1 low BMC
    subframe->l = (subframe->l & 0xffu)
               | (((uint16_t)s0) << 8u)
               | (s1 << 24u);

    uint32_t ph = subframe->h >> 24u;
    // h[7:0]=s1 high BMC, h[23:8]=s2 BMC
    uint32_t h = (((uint16_t)s1) >> 8u)
              | (((uint16_t)s2) << 8u);

    uint32_t p = (s0 >> 16u) ^ (s1 >> 16u) ^ (s2 >> 16u);
    p = p ^ ((__mul_instruction(ph & 0x2a, 0x2a) >> 6u) & 1u);
    subframe->h = h | ((ph & 0x7f) << 24u) | (p << 31u);
}

#ifdef __cplusplus
}
#endif

#endif //SOFTWARE_SAMPLE_ENCODING_H
