/*
 * matrix_mix.h: RP2350 matrix mixer kernel (PASS 4), one output per call.
 * See "Matrix Mixing" in Documentation/current_architecture.md.
 */
#ifndef MATRIX_MIX_H
#define MATRIX_MIX_H

#include <stdint.h>

#define MATRIX_MIX_MAX_SRC 8

// dst = sum over k of src[k][i] * gain[k], for 1 <= n <= MATRIX_MIX_MAX_SRC.
void matrix_mix_output(float *dst, const float *const *src, const float *gain,
                       int n, uint32_t count);

// One pass over up to four sources, summed in source order; ACC adds onto dst.
// The k order must match a plain sequential sum, or results stop being bit-exact.
static inline __attribute__((always_inline))
void matrix_mix_pass(float *__restrict d, const float *const *s, const float *g,
                     uint32_t count, const int N, const int ACC, const int U) {
    const float *p0 = s[0], *p1 = N > 1 ? s[1] : 0, *p2 = N > 2 ? s[2] : 0, *p3 = N > 3 ? s[3] : 0;
    const float g0 = g[0], g1 = N > 1 ? g[1] : 0.0f, g2 = N > 2 ? g[2] : 0.0f, g3 = N > 3 ? g[3] : 0.0f;
    uint32_t i = 0;
    if (U > 1) {
        for (; i + (uint32_t)U <= count; i += (uint32_t)U) {
#pragma GCC unroll 4
            for (int u = 0; u < U; u++) {
                float a = ACC ? d[i + u] + p0[i + u] * g0 : p0[i + u] * g0;
                if (N > 1) a += p1[i + u] * g1;
                if (N > 2) a += p2[i + u] * g2;
                if (N > 3) a += p3[i + u] * g3;
                d[i + u] = a;
            }
        }
    }
#pragma GCC unroll 1
    for (; i < count; i++) {
        float a = ACC ? d[i] + p0[i] * g0 : p0[i] * g0;
        if (N > 1) a += p1[i] * g1;
        if (N > 2) a += p2[i] * g2;
        if (N > 3) a += p3[i] * g3;
        d[i] = a;
    }
}

static inline __attribute__((always_inline))
void matrix_mix_kernel(float *__restrict dst, const float *const *src, const float *gain,
                       int n, uint32_t count) {
    // Unroll factors are sized against the RP2350 .data budget; see "Matrix Mixer" in the architecture doc.
    switch (n) {
    case 1: matrix_mix_pass(dst, src, gain, count, 1, 0, 4); return;
    case 2: matrix_mix_pass(dst, src, gain, count, 2, 0, 4); return;
    case 3: matrix_mix_pass(dst, src, gain, count, 3, 0, 1); return;
    default: break;
    }
    // Four or more: a four-source write pass, then accumulate the rest.
    matrix_mix_pass(dst, src, gain, count, 4, 0, 1);
    switch (n) {
    case 5: matrix_mix_pass(dst, src + 4, gain + 4, count, 1, 1, 1); break;
    case 6: matrix_mix_pass(dst, src + 4, gain + 4, count, 2, 1, 1); break;
    case 7: matrix_mix_pass(dst, src + 4, gain + 4, count, 3, 1, 1); break;
    case 8: matrix_mix_pass(dst, src + 4, gain + 4, count, 4, 1, 1); break;
    default: break;
    }
}

#endif // MATRIX_MIX_H
