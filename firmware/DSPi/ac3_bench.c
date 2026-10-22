/*
 * ac3_bench.c; on-device AC-3 decode bench.  See ac3_bench.h.
 */

#include "ac3_bench.h"
#include "ac3/ac3_decoder.h"
#include "ac3_bench_frames.h"
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include <string.h>

static Ac3BenchResult result;
static volatile uint32_t requested_passes = 0;

// Frames are copied out of flash first: in production the deframer hands
// the decoder a RAM buffer, and XIP reads would distort the timing.
static uint8_t frame_buf[AC3_MAX_FRAME_BYTES + 4] __attribute__((aligned(4)));
static Ac3Decoder dec;

static uint32_t fnv1a(uint32_t h, const void *p, size_t n) {
    const uint8_t *b = p;
    for (size_t i = 0; i < n; i++) { h ^= b[i]; h *= 16777619u; }
    return h;
}

void ac3_bench_request(uint32_t passes) {
    requested_passes = passes ? passes : 10;
    result.state = 1;
}

void ac3_bench_get(Ac3BenchResult *out) {
    memcpy(out, &result, sizeof(*out));
}

void ac3_bench_service(void) {
    uint32_t passes = requested_passes;
    if (!passes) return;
    requested_passes = 0;

    memset(&result, 0, sizeof(result));
    result.state = 1;
#if defined(DSPI_AC3_RAM_HOT)
    result.ram_hot = 1;
#endif
    result.frame_count = AC3_BENCH_FRAME_COUNT;
    result.passes = passes;
    result.expected = AC3_BENCH_CHECKSUM;
    result.sys_clk_hz = clock_get_hz(clk_sys);
    result.min_frame_us = 0xFFFFFFFFu;

    // The checksum covers the interleaved DSPi-ordered output exactly as
    // tools/ac3_test/ac3_test.c computes it, one pass only (the overlap
    // history makes later passes differ from the host's single decode).
    static float inter[AC3_BLOCK_SAMPLES * AC3_CH_COUNT];
    uint32_t sum = 2166136261u;

    uint32_t t_run0 = time_us_32();
    for (uint32_t p = 0; p < passes; p++) {
        ac3_decoder_init(&dec);
        const uint8_t *src = ac3_bench_frames;
        bool checksum_pass = (p == 0);
        for (int f = 0; f < AC3_BENCH_FRAME_COUNT; f++) {
            uint32_t len = ac3_bench_frame_len[f];
            memcpy(frame_buf, src, len);
            src += len;

            uint32_t t0 = time_us_32();
            if (ac3_decoder_frame_begin(&dec, frame_buf)) { result.errors++; continue; }
            uint32_t t1 = time_us_32();
            result.header_us += t1 - t0;
            for (int b = 0; b < AC3_BLOCKS_PER_FRAME; b++) {
                uint32_t tb = time_us_32();
                if (ac3_decoder_block(&dec)) { result.errors++; break; }
                result.block_us += time_us_32() - tb;
                if (checksum_pass) {
                    for (int ch = 0; ch < AC3_CH_COUNT; ch++) {
                        const float *s = ac3_decoder_channel(&dec, ch);
                        for (int i = 0; i < AC3_BLOCK_SAMPLES; i++)
                            inter[i * AC3_CH_COUNT + ch] = s ? s[i] : 0.0f;
                    }
                    sum = fnv1a(sum, inter, sizeof(inter));
                }
            }
            uint32_t frame_us = time_us_32() - t0;
            if (frame_us < result.min_frame_us) result.min_frame_us = frame_us;
            if (frame_us > result.max_frame_us) result.max_frame_us = frame_us;
        }
    }
    result.total_us = time_us_32() - t_run0;
    result.checksum = sum;
    result.checksum_ok = (sum == AC3_BENCH_CHECKSUM) ? 1 : 0;
    result.state = 2;
}
