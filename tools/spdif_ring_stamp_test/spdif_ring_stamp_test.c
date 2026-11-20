// Host test: the real ring stamping loops (ring_stamp.h) against a naive
// per-frame reference, over random positions, lengths, ring wrap and every
// IEC phase boundary. Build and run: tools/spdif_ring_stamp_test/run.sh
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/audio_spdif/ring_stamp.h"

#define RING_FRAMES 512u

static const uint8_t cs[5] = {0x04, 0x00, 0x00, 0x02, 0x0B};

static uint32_t ref_cs(uint32_t phase) {
    return phase < 40 ? (uint32_t)((cs[phase / 8] >> (phase % 8)) & 1u) : 0u;
}

// Word format restated from the spec, independent of the header's helpers.
static void ref_frame(uint32_t *ring, uint32_t f, int32_t l, int32_t r) {
    uint32_t phase = f % 192u, c = ref_cs(phase) << 30;
    uint32_t x = 2u | (0u << 2), y = 1u | (1u << 2), z = 0u | (2u << 2);
    uint32_t pos = f & (RING_FRAMES - 1u);
    ring[pos * 2] = (((uint32_t)l & 0xFFFFFFu) << 4) | c | (phase == 0 ? z : x);
    ring[pos * 2 + 1] = (((uint32_t)r & 0xFFFFFFu) << 4) | c | y;
}

static uint32_t rng_state = 12345u;
static uint32_t rnd(void) {
    rng_state ^= rng_state << 13; rng_state ^= rng_state >> 17; rng_state ^= rng_state << 5;
    return rng_state;
}

int main(void) {
    static uint32_t a[RING_FRAMES * 2], b[RING_FRAMES * 2];
    static int32_t lr[RING_FRAMES * 2];
    unsigned fails = 0, runs = 0;
    for (int it = 0; it < 200000; it++) {
        uint32_t start;
        switch (it % 4) {
        case 0: start = rnd(); break;                                  // anywhere, incl. near 2^32
        case 1: start = (rnd() % 64u) * 192u + (rnd() % 3u) * 20u + 38u; break;  // around 40
        case 2: start = (rnd() % 64u) * 192u + 190u - (rnd() % 4u); break;        // around 192/0
        default: start = (rnd() % 64u) * RING_FRAMES + RING_FRAMES - 1u - (rnd() % 8u); break; // wrap
        }
        uint32_t n = (it % 8 == 0) ? rnd() % (RING_FRAMES + 1u) : rnd() % 300u;
        int silence = it & 1;
        for (uint32_t i = 0; i < RING_FRAMES * 2; i++) a[i] = b[i] = rnd();
        for (uint32_t i = 0; i < n * 2; i++) lr[i] = (int32_t)rnd();
        if (silence) {
            spdif_stamp_silence(a, RING_FRAMES, cs, start, n);
            for (uint32_t i = 0; i < n; i++) ref_frame(b, start + i, 0, 0);
        } else {
            spdif_stamp_audio(a, RING_FRAMES, cs, start, lr, n);
            for (uint32_t i = 0; i < n; i++) ref_frame(b, start + i, lr[i * 2], lr[i * 2 + 1]);
        }
        runs++;
        if (memcmp(a, b, sizeof a) != 0) {
            if (fails++ < 5) printf("FAIL: %s start=%u n=%u\n", silence ? "silence" : "audio", start, n);
        }
    }
    printf("%u runs, %u failures\n", runs, fails);
    return fails ? 1 : 0;
}
