#pragma once

// ----------------------------------------------------------------------------
// Float -> S24 finalization of output blocks (RP2350).
//
// Two modes, chosen once per packet from the ADAT-active snapshot (both cores
// use the same snapshot via core1_eq_work.finalize_s24):
//
// ADAT active: after the last float consumer (EQ/gain, loudness, delay, peak
// metering), each output channel row of buf_out is converted IN PLACE to a
// clamped 24-bit sample held in an int32.  The slot interleave and the ADAT
// encoder both read this integer form, so the clamp/scale runs exactly once
// per sample per channel instead of once per consumer.
//
// ADAT inactive: the staging pass is skipped and each slot pair is converted
// and interleaved in one fused pass (the rows stay float), avoiding the
// second memory pass when nothing else consumes the integer form.  Both modes
// produce bit-identical slot bytes; only the number of passes differs.
//
// The rows are declared float; every integer access to them must go through
// out_s24_t (may_alias) so GCC preserves cross-type access ordering.
// ----------------------------------------------------------------------------

#include <stdint.h>
#include <math.h>

typedef int32_t out_s24_t __attribute__((may_alias));

// Largest |x| in a block. Keep fmaxf: it compiles to VMAXNM, while an
// if (a > pk) compare pays a VMRS flag stall per sample. NaNs are ignored.
static inline float block_peak_abs(const float *buf, uint32_t n) {
    float pk = 0.0f;
    for (uint32_t i = 0; i < n; i++)
        pk = fmaxf(pk, fabsf(buf[i]));
    return pk;
}

// Returns the row's pre-clamp peak |x|, so metering needs no separate pass.
static inline float output_block_to_s24_inplace(float *buf, uint32_t n) {
    out_s24_t *dst = (out_s24_t *)buf;
    float pk = 0.0f;
    for (uint32_t i = 0; i < n; i++) {
        float v = buf[i];
        pk = fmaxf(pk, fabsf(v));
        float x = fmaxf(-1.0f, fminf(1.0f, v));
        dst[i] = (int32_t)(x * 8388607.0f);
    }
    return pk;
}

// Integer interleave of two rows already finalized by
// output_block_to_s24_inplace() (ADAT-active mode).
static inline void output_pair_interleave_s24(int32_t *out_ptr,
                                              const float *l, const float *r,
                                              uint32_t n) {
    const out_s24_t *sl = (const out_s24_t *)l;
    const out_s24_t *sr = (const out_s24_t *)r;
    for (uint32_t i = 0; i < n; i++) {
        out_ptr[i*2]   = sl[i];
        out_ptr[i*2+1] = sr[i];
    }
}

// Fused convert + interleave of two float rows (ADAT-inactive mode; the rows
// are left untouched). Writes each row's pre-clamp peak |x| to pk[0..1].
static inline void output_pair_convert_interleave(int32_t *out_ptr,
                                                  const float *l, const float *r,
                                                  uint32_t n, float pk[2]) {
    float pl = 0.0f, pr = 0.0f;
    for (uint32_t i = 0; i < n; i++) {
        float vl = l[i], vr = r[i];
        pl = fmaxf(pl, fabsf(vl));
        pr = fmaxf(pr, fabsf(vr));
        float dl = fmaxf(-1.0f, fminf(1.0f, vl));
        float dr = fmaxf(-1.0f, fminf(1.0f, vr));
        out_ptr[i*2]   = (int32_t)(dl * 8388607.0f);
        out_ptr[i*2+1] = (int32_t)(dr * 8388607.0f);
    }
    pk[0] = pl;
    pk[1] = pr;
}
