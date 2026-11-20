/*
 * Copyright (c) 2026 WeebLabs
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// S/PDIF ring word stamping for audio_spdif_bmc.pio (one word per subframe).
// Plain C with no SDK dependency so tools/spdif_ring_stamp_test builds the
// same loops on the host. Word format: spdif_pio_bmc_encoder_spec.md.

#ifndef _PICO_AUDIO_SPDIF_RING_STAMP_H
#define _PICO_AUDIO_SPDIF_RING_STAMP_H

#include <stdint.h>

#define SPDIF_STAMP_BLOCK_FRAMES  192u   // IEC 60958 block
#define SPDIF_STAMP_CS_FRAMES     40u    // channel-status bits carried; 0 after

// Preamble as [1:0] lead and [3:2] trail low-loop counts.
#define SPDIF_WORD_PRE_X  (2u | (0u << 2))
#define SPDIF_WORD_PRE_Y  (1u | (1u << 2))
#define SPDIF_WORD_PRE_Z  (0u | (2u << 2))
#define SPDIF_WORD_C_SHIFT 30u

// Always inline: the silence path runs from RAM during flash blackout.
#define SPDIF_STAMP_INLINE static inline __attribute__((always_inline))

SPDIF_STAMP_INLINE uint32_t spdif_stamp_sample(int32_t s) {
    return ((uint32_t)s << 4) & 0x0FFFFFF0u;
}

SPDIF_STAMP_INLINE uint32_t spdif_stamp_cs(const uint8_t *cs, uint32_t phase) {
    return (uint32_t)((cs[phase >> 3] >> (phase & 7u)) & 1u) << SPDIF_WORD_C_SHIFT;
}

// Frames until the next ring wrap or IEC phase boundary (40, 192), capped at n.
SPDIF_STAMP_INLINE uint32_t spdif_stamp_seg(uint32_t pos, uint32_t phase,
                                            uint32_t n, uint32_t ring_frames) {
    uint32_t len = ring_frames - pos;
    uint32_t pb = (phase < SPDIF_STAMP_CS_FRAMES) ? SPDIF_STAMP_CS_FRAMES - phase
                                                  : SPDIF_STAMP_BLOCK_FRAMES - phase;
    if (pb < len) len = pb;
    if (n < len) len = n;
    return len;
}

// Stamp n audio frames from interleaved L/R at absolute frames [w, w+n).
// ring_frames must be a power of two; ring holds 2 words per frame.
SPDIF_STAMP_INLINE void spdif_stamp_audio(uint32_t *ring, uint32_t ring_frames,
                                          const uint8_t *cs, uint32_t w,
                                          const int32_t *lr, uint32_t n) {
    uint32_t phase = w % SPDIF_STAMP_BLOCK_FRAMES;
    while (n) {
        uint32_t pos = w & (ring_frames - 1u);
        uint32_t len = spdif_stamp_seg(pos, phase, n, ring_frames);
        uint32_t *p = ring + pos * 2u;
        if (phase >= SPDIF_STAMP_CS_FRAMES) {
            for (uint32_t i = 0; i < len; i++) {
                p[0] = spdif_stamp_sample(lr[0]) | SPDIF_WORD_PRE_X;
                p[1] = spdif_stamp_sample(lr[1]) | SPDIF_WORD_PRE_Y;
                p += 2;
                lr += 2;
            }
        } else {
            for (uint32_t i = 0; i < len; i++) {
                uint32_t ph = phase + i;
                uint32_t c = spdif_stamp_cs(cs, ph);
                p[0] = spdif_stamp_sample(lr[0]) | c |
                       (ph == 0 ? SPDIF_WORD_PRE_Z : SPDIF_WORD_PRE_X);
                p[1] = spdif_stamp_sample(lr[1]) | c | SPDIF_WORD_PRE_Y;
                p += 2;
                lr += 2;
            }
        }
        w += len;
        n -= len;
        phase += len;
        if (phase == SPDIF_STAMP_BLOCK_FRAMES) phase = 0;
    }
}

// Stamp n silence frames at absolute frames [e, e+n).
SPDIF_STAMP_INLINE void spdif_stamp_silence(uint32_t *ring, uint32_t ring_frames,
                                            const uint8_t *cs, uint32_t e, uint32_t n) {
    uint32_t phase = e % SPDIF_STAMP_BLOCK_FRAMES;
    while (n) {
        uint32_t pos = e & (ring_frames - 1u);
        uint32_t len = spdif_stamp_seg(pos, phase, n, ring_frames);
        uint32_t *p = ring + pos * 2u;
        if (phase >= SPDIF_STAMP_CS_FRAMES) {
            for (uint32_t i = 0; i < len; i++) {
                p[0] = SPDIF_WORD_PRE_X;
                p[1] = SPDIF_WORD_PRE_Y;
                p += 2;
            }
        } else {
            for (uint32_t i = 0; i < len; i++) {
                uint32_t ph = phase + i;
                uint32_t c = spdif_stamp_cs(cs, ph);
                p[0] = c | (ph == 0 ? SPDIF_WORD_PRE_Z : SPDIF_WORD_PRE_X);
                p[1] = c | SPDIF_WORD_PRE_Y;
                p += 2;
            }
        }
        e += len;
        n -= len;
        phase += len;
        if (phase == SPDIF_STAMP_BLOCK_FRAMES) phase = 0;
    }
}

#endif
