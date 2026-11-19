#ifndef DSP_SVF_H
#define DSP_SVF_H

#include "config.h"

#if PICO_RP2350

// Arm classes for the second-order SVF kernel.  Sections fuse only with a
// partner of the same class, so every fused section keeps the op count of its
// specialized single-kernel arm.
typedef enum {
    DSP_SVF_ARM_LPHP = 0,   // FILTER_LOWPASS / FILTER_HIGHPASS
    DSP_SVF_ARM_PEAK,       // FILTER_PEAKING / FILTER_NOTCH / FILTER_ALLPASS
    DSP_SVF_ARM_GENERIC,    // shelves, Linkwitz Transform, and any other type
} DspSvfArm;

static inline DspSvfArm dsp_svf_arm_class(const Filter *f) {
    switch (f->filter_type) {
        case FILTER_LOWPASS:
        case FILTER_HIGHPASS:
            return DSP_SVF_ARM_LPHP;
        case FILTER_PEAKING:
        case FILTER_NOTCH:
        case FILTER_ALLPASS:
            return DSP_SVF_ARM_PEAK;
        default:
            return DSP_SVF_ARM_GENERIC;
    }
}

// Loop state carries h1 = ic1eq/2 (with A1 = 2*a1) and updates ic2eq by
// 2*g*v1; Filter keeps the standard ic1eq/ic2eq meaning, converted only at
// kernel entry and exit, so crossover state save/restore is unaffected.
#define SVF_LOAD(f, n) \
    const float A1_##n = (f)->sva1 + (f)->sva1, a2_##n = (f)->sva2; \
    const float g_##n = (f)->g, G2_##n = (f)->g + (f)->g; \
    const float m0_##n = (f)->svm0, m1_##n = (f)->svm1, m2_##n = (f)->svm2; \
    float h1_##n = 0.5f * (f)->svic1eq, ic2_##n = (f)->svic2eq; \
    float v1_##n, v2_##n, v3_##n, gv_##n

#define SVF_STORE(f, n) \
    (f)->svic1eq = h1_##n + h1_##n; \
    (f)->svic2eq = ic2_##n;

// Per-sample arms: section n maps input x to output y.
#define SVF_BP(x, n) \
    v3_##n = (x) - ic2_##n; \
    v1_##n = A1_##n * h1_##n + a2_##n * v3_##n; \
    h1_##n = v1_##n - h1_##n;
#define SVF_PEAK(x, y, n) SVF_BP(x, n) \
    ic2_##n = ic2_##n + G2_##n * v1_##n; \
    y = (x) + m1_##n * v1_##n;
#define SVF_V2(x, n) SVF_BP(x, n) \
    gv_##n = g_##n * v1_##n; \
    v2_##n = ic2_##n + gv_##n; \
    ic2_##n = v2_##n + gv_##n;
#define SVF_LP(x, y, n)    SVF_V2(x, n) y = v2_##n;
#define SVF_HP(x, y, n)    SVF_V2(x, n) y = (x) + m1_##n * v1_##n - v2_##n;
#define SVF_SHELF(x, y, n) SVF_V2(x, n) y = (x) * m0_##n + m1_##n * v1_##n + m2_##n * v2_##n;

// Unrolled-by-4 sweep of sp[0..count) plus the remainder; STEP(s, ...) is one sample.
#define SVF_UNROLL4(STEP, ...) do { \
    uint32_t blk_ = count >> 2; \
    while (blk_ > 0) { \
        STEP(sp[0], __VA_ARGS__) STEP(sp[1], __VA_ARGS__) \
        STEP(sp[2], __VA_ARGS__) STEP(sp[3], __VA_ARGS__) \
        sp += 4; blk_--; \
    } \
    blk_ = count & 0x3; \
    while (blk_ > 0) { STEP(sp[0], __VA_ARGS__) sp++; blk_--; } \
} while (0)

#define SVF_S1(s, ARM)        { x = (s); ARM(x, y, 0) (s) = y; }
#define SVF_S2(s, ARM0, ARM1) { x = (s); ARM0(x, y, 0) ARM1(y, z, 1) (s) = z; }
#define SVF1_S(s, OUT)        { x = (s); v1 = a2 * x + A1 * h1; h1 = v1 - h1; (s) = (OUT); }

// Forced inline: an out-of-line copy lands in flash (XIP) and fails the RAM placement check.
#define SVF_KERNEL static inline __attribute__((always_inline))

SVF_KERNEL void dsp_svf_first_order(Filter * __restrict f, float * __restrict samples, uint32_t count) {
    // One-pole TPT SVF, same half-state ic1 form as the second-order kernel.
    const float A1 = f->sva1 + f->sva1, a2 = f->sva2;
    const float m0 = f->svm0, m1 = f->svm1, m2 = f->svm2;
    float h1 = 0.5f * f->svic1eq;
    float *sp = samples;
    float x, v1;

    switch (f->filter_type) {
        case FILTER_LOWPASS1:   SVF_UNROLL4(SVF1_S, v1); break;
        case FILTER_HIGHPASS1:  SVF_UNROLL4(SVF1_S, x - v1); break;
        case FILTER_ALLPASS1:   SVF_UNROLL4(SVF1_S, v1 + v1 - x); break;
        case FILTER_HIGHSHELF1: SVF_UNROLL4(SVF1_S, x + m2 * (x - v1)); break;
        case FILTER_LOWSHELF1:  SVF_UNROLL4(SVF1_S, x + m1 * v1); break;
        default:                SVF_UNROLL4(SVF1_S, x * m0 + m1 * v1 + m2 * (x - v1)); break;
    }

    f->svic1eq = h1 + h1;
}

SVF_KERNEL void dsp_svf_second_order(Filter * __restrict f, float * __restrict samples, uint32_t count) {
    SVF_LOAD(f, 0);
    float *sp = samples;
    float x, y;

    // Per-type specialization: eliminates zero-multiplies in inner loop
    switch (f->filter_type) {
        case FILTER_LOWPASS:  SVF_UNROLL4(SVF_S1, SVF_LP); break;
        case FILTER_HIGHPASS: SVF_UNROLL4(SVF_S1, SVF_HP); break;
        case FILTER_PEAKING:
        case FILTER_NOTCH:
        case FILTER_ALLPASS:  SVF_UNROLL4(SVF_S1, SVF_PEAK); break;
        default:              SVF_UNROLL4(SVF_S1, SVF_SHELF); break;
    }

    SVF_STORE(f, 0);
}

// Fused two-section SVF cascade: each sample is loaded once, pushed through
// both sections in registers, and stored once.  Callers must pair sections of
// equal dsp_svf_arm_class; each section runs its single-kernel arm verbatim, so
// a fused sweep is bit-identical to two sequential single sweeps.
SVF_KERNEL void dsp_svf_second_order_x2(Filter * __restrict f0, Filter * __restrict f1,
                                           float * __restrict samples, uint32_t count) {
    SVF_LOAD(f0, 0);
    SVF_LOAD(f1, 1);
    float *sp = samples;
    float x, y, z;

    DspSvfArm arm0 = dsp_svf_arm_class(f0);
    if (arm0 == DSP_SVF_ARM_PEAK) {
        SVF_UNROLL4(SVF_S2, SVF_PEAK, SVF_PEAK);
    } else if (arm0 == DSP_SVF_ARM_LPHP) {
        bool hp0 = (f0->filter_type == FILTER_HIGHPASS);
        bool hp1 = (f1->filter_type == FILTER_HIGHPASS);
        if (!hp0 && !hp1)     SVF_UNROLL4(SVF_S2, SVF_LP, SVF_LP);
        else if (!hp0)        SVF_UNROLL4(SVF_S2, SVF_LP, SVF_HP);
        else if (!hp1)        SVF_UNROLL4(SVF_S2, SVF_HP, SVF_LP);
        else                  SVF_UNROLL4(SVF_S2, SVF_HP, SVF_HP);
    } else {
        SVF_UNROLL4(SVF_S2, SVF_SHELF, SVF_SHELF);
    }

    SVF_STORE(f0, 0);
    SVF_STORE(f1, 1);
}

#endif // PICO_RP2350

#endif // DSP_SVF_H
