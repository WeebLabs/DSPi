#ifndef DSP_BIQUAD_H
#define DSP_BIQUAD_H

#include "config.h"

#if PICO_RP2350
static inline void dsp_biquad_first_order(Filter * __restrict f, float * __restrict samples, uint32_t count) {
    // Load biquad coefficients
    // Assumption is that b2 and a2 are 0.0f
    float b0 = f->b0, b1 = f->b1;
    float a1 = f->a1;
    float s1 = f->s1, s2 = f->s2;
    float *sp = samples;
    float x, y, t;
    uint32_t blk_count = count >> 2; // unroll loops by 4

    switch(f->filter_type) {
        case FILTER_ALLPASS1:
            //optimise b1=1.0f, (b0=a1)
            while(blk_count > 0) {
                x = sp[0];
                y = b0 * x + s1;
                s1 = x - a1 * y;
                sp[0] = y;

                x = sp[1];
                y = b0 * x + s1;
                s1 = x - a1 * y;
                sp[1] = y;

                x = sp[2];
                y = b0 * x + s1;
                s1 = x - a1 * y;
                sp[2] = y;

                x = sp[3];
                y = b0 * x + s1;
                s1 = x - a1 * y;
                sp[3] = y;

                sp += 4;
                blk_count--;
            }

            blk_count = count & 0x3;
            while(blk_count > 0) {
                x = *sp;
                y = b0 * x + s1;
                s1 = x - a1 * y;
                *sp++ = y;
                blk_count--;
            }
            break;

        case FILTER_LOWPASS1:
            //optimise b0=b1
            while(blk_count > 0) {
                x = sp[0];
                t = b0 * x;
                y = t + s1;
                s1 = t - a1 * y;
                sp[0] = y;

                x = sp[1];
                t = b0 * x;
                y = t + s1;
                s1 = t - a1 * y;
                sp[1] = y;

                x = sp[2];
                t = b0 * x;
                y = t + s1;
                s1 = t - a1 * y;
                sp[2] = y;

                x = sp[3];
                t = b0 * x;
                y = t + s1;
                s1 = t - a1 * y;
                sp[3] = y;

                sp += 4;
                blk_count--;
            }

            blk_count = count & 0x3;
            while(blk_count > 0) {
                x = *sp;
                t = b0 * x;
                y = t + s1;
                s1 = t - a1 * y;
                *sp++ = y;
                blk_count--;
            }
            break;

        default:
            while(blk_count > 0) {
                x = sp[0];
                y = b0 * x + s1;
                s1 = b1 * x - a1 * y;
                sp[0] = y;

                x = sp[1];
                y = b0 * x + s1;
                s1 = b1 * x - a1 * y;
                sp[1] = y;

                x = sp[2];
                y = b0 * x + s1;
                s1 = b1 * x - a1 * y;
                sp[2] = y;

                x = sp[3];
                y = b0 * x + s1;
                s1 = b1 * x - a1 * y;
                sp[3] = y;

                sp += 4;
                blk_count--;
            }

            blk_count = count & 0x3;
            while(blk_count > 0) {
                x = *sp;
                y = b0 * x + s1;
                s1 = b1 * x - a1 * y;
                *sp++ = y;
                blk_count--;
            }
            break;
    }

    f->s1 = s1;
    f->s2 = 0.0f;
}

static inline void dsp_biquad_second_order(Filter * __restrict f, float * __restrict samples, uint32_t count) {
    // Load biquad coefficients
    float b0 = f->b0, b1 = f->b1, b2 = f->b2;
    float a1 = f->a1, a2 = f->a2;
    float s1 = f->s1, s2 = f->s2;
    float *sp = samples;
    float x, y;
    uint32_t blk_count = count >> 2; // unroll loops by 4

    switch(f->filter_type) {
        case FILTER_LOWPASS:
        case FILTER_HIGHPASS:
            //optimise b0 = b2
            while(blk_count > 0) {
                x = sp[0];
                y = b0 * x + s1;
                s1 = b1 * x - a1 * y + s2;
                s2 = b0 * x - a2 * y;
                sp[0] = y;

                x = sp[1];
                y = b0 * x + s1;
                s1 = b1 * x - a1 * y + s2;
                s2 = b0 * x - a2 * y;
                sp[1] = y;

                x = sp[2];
                y = b0 * x + s1;
                s1 = b1 * x - a1 * y + s2;
                s2 = b0 * x - a2 * y;
                sp[2] = y;

                x = sp[3];
                y = b0 * x + s1;
                s1 = b1 * x - a1 * y + s2;
                s2 = b0 * x - a2 * y;
                sp[3] = y;

                sp += 4;
                blk_count--;
            }

            blk_count = count & 0x3;
            while(blk_count > 0) {
                x = *sp;
                y = b0 * x + s1;
                s1 = b1 * x - a1 * y + s2;
                s2 = b0 * x - a2 * y;
                *sp++ = y;
                blk_count--;
            }
            break;

        case FILTER_PEAKING:
            // optimise b1 = a1
            while(blk_count > 0) {
                x = sp[0];
                y = b0 * x + s1;
                s1 = b1 * (x - y) + s2;
                s2 = b2 * x - a2 * y;
                sp[0] = y;

                x = sp[1];
                y = b0 * x + s1;
                s1 = b1 * (x - y) + s2;
                s2 = b2 * x - a2 * y;
                sp[1] = y;

                x = sp[2];
                y = b0 * x + s1;
                s1 = b1 * (x - y) + s2;
                s2 = b2 * x - a2 * y;
                sp[2] = y;

                x = sp[3];
                y = b0 * x + s1;
                s1 = b1 * (x - y) + s2;
                s2 = b2 * x - a2 * y;
                sp[3] = y;

                sp += 4;
                blk_count--;
            }

            blk_count = count & 0x3;
            while(blk_count > 0) {
                x = *sp;
                y = b0 * x + s1;
                s1 = b1 * (x - y) + s2;
                s2 = b2 * x - a2 * y;
                *sp++ = y;
                blk_count--;
            }
            break;

        case FILTER_ALLPASS:
            // optimise b1 = a1
            // optimise a0 = b2, a0 is normalised to 1.0f so b2 is implicitly 1.0f
            while(blk_count > 0) {
                x = sp[0];
                y = b0 * x + s1;
                s1 = b1 * (x - y) + s2;
                s2 = x - a2 * y;
                sp[0] = y;

                x = sp[1];
                y = b0 * x + s1;
                s1 = b1 * (x - y) + s2;
                s2 = x - a2 * y;
                sp[1] = y;

                x = sp[2];
                y = b0 * x + s1;
                s1 = b1 * (x - y) + s2;
                s2 = x - a2 * y;
                sp[2] = y;

                x = sp[3];
                y = b0 * x + s1;
                s1 = b1 * (x - y) + s2;
                s2 = x - a2 * y;
                sp[3] = y;

                sp += 4;
                blk_count--;
            }

            blk_count = count & 0x3;
            while(blk_count > 0) {
                x = *sp;
                y = b0 * x + s1;
                s1 = b1 * (x - y) + s2;
                s2 = x - a2 * y;
                *sp++ = y;
                blk_count--;
            }
            break;

        case FILTER_NOTCH:
            // optimise b0=b2, b1=a1
           while(blk_count > 0) {
                x = sp[0];
                y = b0 * x + s1;
                s1 = b1 * (x - y) + s2;
                s2 = b0 * x - a2 * y;
                sp[0] = y;

                x = sp[1];
                y = b0 * x + s1;
                s1 = b1 * (x - y) + s2;
                s2 = b0 * x - a2 * y;
                sp[1] = y;

                x = sp[2];
                y = b0 * x + s1;
                s1 = b1 * (x - y) + s2;
                s2 = b0 * x - a2 * y;
                sp[2] = y;

                x = sp[3];
                y = b0 * x + s1;
                s1 = b1 * (x - y) + s2;
                s2 = b0 * x - a2 * y;
                sp[3] = y;

                sp += 4;
                blk_count--;
            }

            blk_count = count & 0x3;
            while(blk_count > 0) {
                x = *sp;
                y = b0 * x + s1;
                s1 = b1 * (x - y) + s2;
                s2 = b0 * x - a2 * y;
                *sp++ = y;
                blk_count--;
            }
            break;

        default:
            while(blk_count > 0) {
                x = sp[0];
                y = b0 * x + s1;
                s1 = b1 * x - a1 * y + s2;
                s2 = b2 * x - a2 * y;
                sp[0] = y;

                x = sp[1];
                y = b0 * x + s1;
                s1 = b1 * x - a1 * y + s2;
                s2 = b2 * x - a2 * y;
                sp[1] = y;

                x = sp[2];
                y = b0 * x + s1;
                s1 = b1 * x - a1 * y + s2;
                s2 = b2 * x - a2 * y;
                sp[2] = y;

                x = sp[3];
                y = b0 * x + s1;
                s1 = b1 * x - a1 * y + s2;
                s2 = b2 * x - a2 * y;
                sp[3] = y;

                sp += 4;
                blk_count--;
            }

            blk_count = count & 0x3;
            while(blk_count > 0) {
                x = *sp;
                y = b0 * x + s1;
                s1 = b1 * x - a1 * y + s2;
                s2 = b2 * x - a2 * y;
                *sp++ = y;
                blk_count--;
            }
            break;
    }

    f->s1 = s1;
    f->s2 = s2;
}

// Second-order LP/HP sections satisfy b0 == b2 exactly; every second-order
// crossover biquad section is one of these two types (crossover.c).
static inline bool dsp_biquad_is_lphp(const Filter *f) {
    return f->filter_type == FILTER_LOWPASS || f->filter_type == FILTER_HIGHPASS;
}

// Fused two-section cascade: each sample is loaded once, pushed through both
// sections in registers, and stored once.  The arm is chosen once per call and
// each arm reuses the matching single-kernel expressions verbatim, so a fused
// sweep is bit-identical to two sequential single sweeps.

// Both sections optimise b1 = a1 (8 ops/section/sample vs 9 generic).
static inline __attribute__((always_inline))
void dsp_biquad_x2_peak(Filter * __restrict f0, Filter * __restrict f1,
                        float * __restrict samples, uint32_t count) {
    float b0_0 = f0->b0, b1_0 = f0->b1, b2_0 = f0->b2;
    float a1_0 = f0->a1, a2_0 = f0->a2;
    float s1_0 = f0->s1, s2_0 = f0->s2;
    float b0_1 = f1->b0, b1_1 = f1->b1, b2_1 = f1->b2;
    float a1_1 = f1->a1, a2_1 = f1->a2;
    float s1_1 = f1->s1, s2_1 = f1->s2;
    float *sp = samples;
    float x, y, z;
    uint32_t blk_count = count >> 2; // unroll loops by 4

    while(blk_count > 0) {
        x = sp[0];
        y = b0_0 * x + s1_0;
        s1_0 = b1_0 * (x - y) + s2_0;
        s2_0 = b2_0 * x - a2_0 * y;
        z = b0_1 * y + s1_1;
        s1_1 = b1_1 * (y - z) + s2_1;
        s2_1 = b2_1 * y - a2_1 * z;
        sp[0] = z;

        x = sp[1];
        y = b0_0 * x + s1_0;
        s1_0 = b1_0 * (x - y) + s2_0;
        s2_0 = b2_0 * x - a2_0 * y;
        z = b0_1 * y + s1_1;
        s1_1 = b1_1 * (y - z) + s2_1;
        s2_1 = b2_1 * y - a2_1 * z;
        sp[1] = z;

        x = sp[2];
        y = b0_0 * x + s1_0;
        s1_0 = b1_0 * (x - y) + s2_0;
        s2_0 = b2_0 * x - a2_0 * y;
        z = b0_1 * y + s1_1;
        s1_1 = b1_1 * (y - z) + s2_1;
        s2_1 = b2_1 * y - a2_1 * z;
        sp[2] = z;

        x = sp[3];
        y = b0_0 * x + s1_0;
        s1_0 = b1_0 * (x - y) + s2_0;
        s2_0 = b2_0 * x - a2_0 * y;
        z = b0_1 * y + s1_1;
        s1_1 = b1_1 * (y - z) + s2_1;
        s2_1 = b2_1 * y - a2_1 * z;
        sp[3] = z;

        sp += 4;
        blk_count--;
    }

    blk_count = count & 0x3;
    while(blk_count > 0) {
        x = *sp;
        y = b0_0 * x + s1_0;
        s1_0 = b1_0 * (x - y) + s2_0;
        s2_0 = b2_0 * x - a2_0 * y;
        z = b0_1 * y + s1_1;
        s1_1 = b1_1 * (y - z) + s2_1;
        s2_1 = b2_1 * y - a2_1 * z;
        *sp++ = z;
        blk_count--;
    }

    f0->s1 = s1_0;
    f0->s2 = s2_0;
    f1->s1 = s1_1;
    f1->s2 = s2_1;
}

// Both sections optimise b0 = b2, so the b0 product serves y and s2; LP may mix with HP.
static inline __attribute__((always_inline))
void dsp_biquad_x2_lphp(Filter * __restrict f0, Filter * __restrict f1,
                        float * __restrict samples, uint32_t count) {
    float b0_0 = f0->b0, b1_0 = f0->b1, b2_0 = f0->b2;
    float a1_0 = f0->a1, a2_0 = f0->a2;
    float s1_0 = f0->s1, s2_0 = f0->s2;
    float b0_1 = f1->b0, b1_1 = f1->b1, b2_1 = f1->b2;
    float a1_1 = f1->a1, a2_1 = f1->a2;
    float s1_1 = f1->s1, s2_1 = f1->s2;
    float *sp = samples;
    float x, y, z, t0, t1;
    uint32_t blk_count = count >> 2; // unroll loops by 4

    while(blk_count > 0) {
        x = sp[0];
        t0 = b0_0 * x;
        y = t0 + s1_0;
        s1_0 = b1_0 * x - a1_0 * y + s2_0;
        s2_0 = t0 - a2_0 * y;
        t1 = b0_1 * y;
        z = t1 + s1_1;
        s1_1 = b1_1 * y - a1_1 * z + s2_1;
        s2_1 = t1 - a2_1 * z;
        sp[0] = z;

        x = sp[1];
        t0 = b0_0 * x;
        y = t0 + s1_0;
        s1_0 = b1_0 * x - a1_0 * y + s2_0;
        s2_0 = t0 - a2_0 * y;
        t1 = b0_1 * y;
        z = t1 + s1_1;
        s1_1 = b1_1 * y - a1_1 * z + s2_1;
        s2_1 = t1 - a2_1 * z;
        sp[1] = z;

        x = sp[2];
        t0 = b0_0 * x;
        y = t0 + s1_0;
        s1_0 = b1_0 * x - a1_0 * y + s2_0;
        s2_0 = t0 - a2_0 * y;
        t1 = b0_1 * y;
        z = t1 + s1_1;
        s1_1 = b1_1 * y - a1_1 * z + s2_1;
        s2_1 = t1 - a2_1 * z;
        sp[2] = z;

        x = sp[3];
        t0 = b0_0 * x;
        y = t0 + s1_0;
        s1_0 = b1_0 * x - a1_0 * y + s2_0;
        s2_0 = t0 - a2_0 * y;
        t1 = b0_1 * y;
        z = t1 + s1_1;
        s1_1 = b1_1 * y - a1_1 * z + s2_1;
        s2_1 = t1 - a2_1 * z;
        sp[3] = z;

        sp += 4;
        blk_count--;
    }

    blk_count = count & 0x3;
    while(blk_count > 0) {
        x = *sp;
        t0 = b0_0 * x;
        y = t0 + s1_0;
        s1_0 = b1_0 * x - a1_0 * y + s2_0;
        s2_0 = t0 - a2_0 * y;
        t1 = b0_1 * y;
        z = t1 + s1_1;
        s1_1 = b1_1 * y - a1_1 * z + s2_1;
        s2_1 = t1 - a2_1 * z;
        *sp++ = z;
        blk_count--;
    }

    f0->s1 = s1_0;
    f0->s2 = s2_0;
    f1->s1 = s1_1;
    f1->s2 = s2_1;
}

// Generic 9-op form for any other pair of second-order sections.
static inline __attribute__((always_inline))
void dsp_biquad_x2_generic(Filter * __restrict f0, Filter * __restrict f1,
                           float * __restrict samples, uint32_t count) {
    float b0_0 = f0->b0, b1_0 = f0->b1, b2_0 = f0->b2;
    float a1_0 = f0->a1, a2_0 = f0->a2;
    float s1_0 = f0->s1, s2_0 = f0->s2;
    float b0_1 = f1->b0, b1_1 = f1->b1, b2_1 = f1->b2;
    float a1_1 = f1->a1, a2_1 = f1->a2;
    float s1_1 = f1->s1, s2_1 = f1->s2;
    float *sp = samples;
    float x, y, z;
    uint32_t blk_count = count >> 2; // unroll loops by 4

    while(blk_count > 0) {
        x = sp[0];
        y = b0_0 * x + s1_0;
        s1_0 = b1_0 * x - a1_0 * y + s2_0;
        s2_0 = b2_0 * x - a2_0 * y;
        z = b0_1 * y + s1_1;
        s1_1 = b1_1 * y - a1_1 * z + s2_1;
        s2_1 = b2_1 * y - a2_1 * z;
        sp[0] = z;

        x = sp[1];
        y = b0_0 * x + s1_0;
        s1_0 = b1_0 * x - a1_0 * y + s2_0;
        s2_0 = b2_0 * x - a2_0 * y;
        z = b0_1 * y + s1_1;
        s1_1 = b1_1 * y - a1_1 * z + s2_1;
        s2_1 = b2_1 * y - a2_1 * z;
        sp[1] = z;

        x = sp[2];
        y = b0_0 * x + s1_0;
        s1_0 = b1_0 * x - a1_0 * y + s2_0;
        s2_0 = b2_0 * x - a2_0 * y;
        z = b0_1 * y + s1_1;
        s1_1 = b1_1 * y - a1_1 * z + s2_1;
        s2_1 = b2_1 * y - a2_1 * z;
        sp[2] = z;

        x = sp[3];
        y = b0_0 * x + s1_0;
        s1_0 = b1_0 * x - a1_0 * y + s2_0;
        s2_0 = b2_0 * x - a2_0 * y;
        z = b0_1 * y + s1_1;
        s1_1 = b1_1 * y - a1_1 * z + s2_1;
        s2_1 = b2_1 * y - a2_1 * z;
        sp[3] = z;

        sp += 4;
        blk_count--;
    }

    blk_count = count & 0x3;
    while(blk_count > 0) {
        x = *sp;
        y = b0_0 * x + s1_0;
        s1_0 = b1_0 * x - a1_0 * y + s2_0;
        s2_0 = b2_0 * x - a2_0 * y;
        z = b0_1 * y + s1_1;
        s1_1 = b1_1 * y - a1_1 * z + s2_1;
        s2_1 = b2_1 * y - a2_1 * z;
        *sp++ = z;
        blk_count--;
    }

    f0->s1 = s1_0;
    f0->s2 = s2_0;
    f1->s1 = s1_1;
    f1->s2 = s2_1;
}

// Arm dispatch for one pair. dsp_cascade_block calls the arms itself (see dsp_pair_runs.c).
static inline void dsp_biquad_second_order_x2(Filter * __restrict f0, Filter * __restrict f1,
                                              float * __restrict samples, uint32_t count) {
    if (f0->filter_type == FILTER_PEAKING && f1->filter_type == FILTER_PEAKING)
        dsp_biquad_x2_peak(f0, f1, samples, count);
    else if (dsp_biquad_is_lphp(f0) && dsp_biquad_is_lphp(f1))
        dsp_biquad_x2_lphp(f0, f1, samples, count);
    else
        dsp_biquad_x2_generic(f0, f1, samples, count);
}

#endif // PICO_RP2350

#endif // DSP_BIQUAD_H
