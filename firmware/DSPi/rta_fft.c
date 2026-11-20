// ----------------------------------------------------------------------------
// Spectrum analyser kernel.  See rta_fft.h for the interface and
// Documentation/Features/spectrum_analyser_spec.md sections 3.4 and 4 for
// the design.  Constants come from rta_tables.h (scripts/gen_rta_tables.py).
// ----------------------------------------------------------------------------

#include "rta_fft.h"
#include "rta_tables.h"

#include <string.h>

// ---------------------------------------------------------------------------
// Format primitives.  Everything below the FFT loops is written once and the
// arithmetic differences live here, so the two builds cannot drift apart.
// ---------------------------------------------------------------------------

#if RTA_SAMPLE_FLOAT

typedef float    rta_acc_t;     // butterfly accumulator
typedef float    rta_tw_t;      // twiddle element
typedef float    rta_binpow_t;  // one bin's power

// Float needs no overflow scaling, so the butterflies and split run at unit
// gain and the finish takes the whole 1/n out with an exact power of two.
#define RTA_STAGE_Q4(x)  (x)
#define RTA_STAGE_Q2(x)  (x)
#define RTA_ISQRT2(x)    (x)
#define RTA_Q2(x)        ((x) * 0.5f)   // part of the split formula, not scaling
#define RTA_BAND_SCALE  7.9826261f    // 1 / 0.125272059, full-scale sine -> 1.0

static inline rta_sample_t rta_st(rta_acc_t v) { return v; }

// 4-term Blackman-Harris on X[k-3..k+3], scaled to a 0.5 centre tap so a
// bin-centred sine keeps Hann's bin level.  Hann leaked loud bass into 250-500 Hz.
static inline rta_acc_t rta_window(rta_acc_t x0, rta_acc_t x1, rta_acc_t x2, rta_acc_t x3,
                                   rta_acc_t x4, rta_acc_t x5, rta_acc_t x6) {
    return 0.5f * x3 - 0.340271777f * (x2 + x4)
         + 0.098452962f * (x1 + x5) - 0.008139373f * (x0 + x6);
}

static inline rta_binpow_t rta_bin_power(rta_acc_t wr, rta_acc_t wi) {
    return wr * wr + wi * wi;
}

#else

typedef int32_t  rta_acc_t;
typedef int16_t  rta_tw_t;
typedef uint32_t rta_binpow_t;

// Round half up: the plain arithmetic shift would bias every stage downwards.
#define RTA_STAGE_Q4(x) (((x) + 2) >> 2)
#define RTA_STAGE_Q2(x) (((x) + 1) >> 1)
#define RTA_Q2(x)     (((x) + 1) >> 1)
#define RTA_ISQRT2(x) ((((x) * 23170) + 16384) >> 15)
#define RTA_BAND_SCALE  7.43439986e-9f  // 1 / (32768^2 * 0.125272059)

static inline rta_sample_t rta_st(rta_acc_t v) {
    if (v > 32767) v = 32767;
    if (v < -32768) v = -32768;
    return (rta_sample_t)v;
}

// Same window in Q15.  Worst-case accumulator is 1.50e9, inside int32.
static inline rta_acc_t rta_window(rta_acc_t x0, rta_acc_t x1, rta_acc_t x2, rta_acc_t x3,
                                   rta_acc_t x4, rta_acc_t x5, rta_acc_t x6) {
    return (16384 * x3 - 11150 * (x2 + x4) + 3226 * (x1 + x5)
            - 267 * (x0 + x6) + 16384) >> 15;
}

// Window output reaches 45685, so squares are taken unsigned (sum <= 4.18e9).
static inline rta_binpow_t rta_bin_power(rta_acc_t wr, rta_acc_t wi) {
    const uint32_t ur = (uint32_t)(wr < 0 ? -wr : wr);
    const uint32_t ui = (uint32_t)(wi < 0 ? -wi : wi);
    return ur * ur + ui * ui;
}

#endif

// v * W, W = (w[0], w[1]) = (cos, -sin).  v must already carry the stage's
// scaling, otherwise the Q15 products overflow int32.  Both twiddle loads come
// before the stores: in float they may alias, which forces a reload otherwise.
static inline void rta_twmul(rta_acc_t vr, rta_acc_t vi, const rta_tw_t *w,
                             rta_sample_t *outr, rta_sample_t *outi) {
#if RTA_SAMPLE_FLOAT
    const float wr = w[0], wi = w[1];
    *outr = vr * wr - vi * wi;
    *outi = vr * wi + vi * wr;
#else
    int32_t wr = w[0], wi = w[1];
    *outr = rta_st((vr * wr - vi * wi + 16384) >> 15);
    *outi = rta_st((vr * wi + vi * wr + 16384) >> 15);
#endif
}

// ---------------------------------------------------------------------------
// Tables
// ---------------------------------------------------------------------------

const RtaBandTable *rta_band_table(uint32_t sample_rate_hz, uint8_t order) {
    if (order < RTA_ORDER_MIN || order > RTA_ORDER_MAX) return NULL;

    int r = -1;
    for (int i = 0; i < (int)(sizeof(rta_base_rate_hz) / sizeof(rta_base_rate_hz[0])); i++) {
        if (rta_base_rate_hz[i] == sample_rate_hz) { r = i; break; }
    }
    if (r < 0) return NULL;

    const uint8_t oi = (uint8_t)(order - RTA_ORDER_MIN);
    return &rta_tables_fast[r][oi];
}

uint16_t rta_band_centre_hz(uint8_t b) {
    return (b < RTA_MAX_BANDS) ? rta_band_centre_tab[b] : 0u;
}

// ---------------------------------------------------------------------------
// log2 and level bytes.  The per-bin path must stay integer-only on RP2040, so
// both formats share one fixed-point log2 and differ only in how they reach it.
// ---------------------------------------------------------------------------

// f12: fractional part of a 1.f mantissa in Q12.  Returns log2(1+f) in Q8.
static inline int32_t rta_log2_frac_q8(uint32_t f12) {
    const uint32_t i = f12 >> 7;
    const uint32_t r = f12 & 127u;
    const uint32_t lo = rta_log2_tab[i];
    const uint32_t hi = rta_log2_tab[i + 1];
    return (int32_t)((lo + (((hi - lo) * r) >> 7)) >> 8);
}

#if !RTA_SAMPLE_FLOAT
// ARMv6-M has no CLZ instruction, so this is a libgcc call on RP2040, not one
// cycle.  It still beats a float log per bin by a wide margin.
static int32_t rta_log2_q8_u32(uint32_t v) {
    const int32_t e = 31 - (int32_t)__builtin_clz(v);
    const uint32_t m = (e >= 12) ? (v >> (e - 12)) : (v << (12 - e));
    return (e * 256) + rta_log2_frac_q8(m & 0xFFFu);
}
#endif

// 0.5 dB steps: level = 243 + 20*log10(2)*log2(power), 6.0206 in Q8 = 1541.
// Both factors are Q8, so the product needs a 16-bit shift, not 8.
static inline uint8_t rta_level_from_log2_q8(int32_t log2_q8) {
    const int32_t lv = RTA_LEVEL_ZERO_DBFS + ((log2_q8 * 1541 + 32768) >> 16);
    if (lv <= 0) return 0;
    if (lv >= 255) return 255;
    return (uint8_t)lv;
}

uint8_t rta_level_from_power(float power) {
    union { float f; uint32_t u; } b;
    if (!(power > 0.0f)) return 0;
    b.f = power;
    const int32_t e = (int32_t)((b.u >> 23) & 0xFFu) - 127;
    if (e < -64) return 0;   // below the wire floor, and keeps denormals out
    return rta_level_from_log2_q8((e * 256) + rta_log2_frac_q8((b.u >> 11) & 0xFFFu));
}

// One bin's level from its power in the format's native units.  scale is the
// float build's 16 / n^2 (see rta_fft_finish_step); Q15 ignores it.
static inline uint8_t rta_bin_level(rta_binpow_t p, float scale) {
#if RTA_SAMPLE_FLOAT
    return rta_level_from_power(scale * p);
#else
    (void)scale;
    // p is |4*Xw|^2 in Q15 LSBs squared, so the normalised power is p / 2^26.
    if (p == 0u) return 0;
    return rta_level_from_log2_q8(rta_log2_q8_u32(p) - (26 << 8));
#endif
}

// ---------------------------------------------------------------------------
// Complex FFT: radix-4 decimation in frequency, with one radix-2 stage first
// when the complex length is not a power of four.  Blocks of length L stride
// the twiddle table by RTA_TW_N / L, so one table serves every order.
// ---------------------------------------------------------------------------

// Two real samples share one complex slot, so the packed value reaches sqrt(2)
// full scale and any later rotation would clip int16.  Scaling by 1/sqrt(2)
// here keeps every stage inside the format; rta_split puts the factor back.
#if !RTA_SAMPLE_FLOAT
static void rta_prescale(rta_sample_t *buf, uint16_t i0, uint16_t i1) {
    for (uint16_t i = i0; i < i1; i++) {
        buf[i] = rta_st(RTA_ISQRT2((rta_acc_t)buf[i]));
    }
}
#endif

// Butterflies j0..j1-1 of the stage (m / 2 in all).
static void rta_stage_r2(rta_sample_t *c, uint16_t m, uint16_t j0, uint16_t j1) {
    const uint16_t half = (uint16_t)(m >> 1);
    const uint16_t tstride = (uint16_t)(RTA_TW_N / m);
    for (uint16_t j = j0; j < j1; j++) {
        const uint16_t i0 = (uint16_t)(j << 1);
        const uint16_t i1 = (uint16_t)((j + half) << 1);
        const rta_acc_t x0r = c[i0], x0i = c[i0 + 1];
        const rta_acc_t x1r = c[i1], x1i = c[i1 + 1];
        c[i0]     = rta_st(RTA_STAGE_Q2(x0r + x1r));
        c[i0 + 1] = rta_st(RTA_STAGE_Q2(x0i + x1i));
        const rta_acc_t ur = RTA_STAGE_Q2(x0r - x1r), ui = RTA_STAGE_Q2(x0i - x1i);
        if (j == 0) {
            c[i1] = rta_st(ur);
            c[i1 + 1] = rta_st(ui);
        } else {
            rta_twmul(ur, ui, &rta_tw[2u * (uint32_t)(j * tstride)], &c[i1], &c[i1 + 1]);
        }
    }
}

// Butterflies q0..q1-1 of the stage (m / 4 in all), q = block * (L / 4) + j.
static void rta_stage_r4(rta_sample_t *c, uint16_t L, uint16_t q0, uint16_t q1) {
    const uint16_t quarter = (uint16_t)(L >> 2);
    const uint16_t tstride = (uint16_t)(RTA_TW_N / L);
    const uint16_t step = (uint16_t)(quarter << 1);
    uint16_t base = (uint16_t)((q0 / quarter) * L);
    uint16_t j = (uint16_t)(q0 % quarter);
    for (uint16_t q = q0; q < q1; q++) {
        const uint16_t i0 = (uint16_t)((base + j) << 1);
        const uint16_t i1 = (uint16_t)(i0 + step);
        const uint16_t i2 = (uint16_t)(i1 + step);
        const uint16_t i3 = (uint16_t)(i2 + step);
        const rta_acc_t x0r = c[i0], x0i = c[i0 + 1];
        const rta_acc_t x1r = c[i1], x1i = c[i1 + 1];
        const rta_acc_t x2r = c[i2], x2i = c[i2 + 1];
        const rta_acc_t x3r = c[i3], x3i = c[i3 + 1];
        const rta_acc_t t0r = x0r + x2r, t0i = x0i + x2i;
        const rta_acc_t t1r = x0r - x2r, t1i = x0i - x2i;
        const rta_acc_t t2r = x1r + x3r, t2i = x1i + x3i;
        const rta_acc_t t3r = x1r - x3r, t3i = x1i - x3i;

        // u1 = t1 - i*t3 and u3 = t1 + i*t3 for the e^-jw transform.
        const rta_acc_t u0r = RTA_STAGE_Q4(t0r + t2r), u0i = RTA_STAGE_Q4(t0i + t2i);
        const rta_acc_t u1r = RTA_STAGE_Q4(t1r + t3i), u1i = RTA_STAGE_Q4(t1i - t3r);
        const rta_acc_t u2r = RTA_STAGE_Q4(t0r - t2r), u2i = RTA_STAGE_Q4(t0i - t2i);
        const rta_acc_t u3r = RTA_STAGE_Q4(t1r - t3i), u3i = RTA_STAGE_Q4(t1i + t3r);

        c[i0] = rta_st(u0r);
        c[i0 + 1] = rta_st(u0i);
        if (j == 0) {
            c[i1] = rta_st(u1r); c[i1 + 1] = rta_st(u1i);
            c[i2] = rta_st(u2r); c[i2 + 1] = rta_st(u2i);
            c[i3] = rta_st(u3r); c[i3 + 1] = rta_st(u3i);
        } else {
            const uint32_t t = (uint32_t)j * tstride;
            rta_twmul(u1r, u1i, &rta_tw[2u * t], &c[i1], &c[i1 + 1]);
            rta_twmul(u2r, u2i, &rta_tw[4u * t], &c[i2], &c[i2 + 1]);
            rta_twmul(u3r, u3i, &rta_tw[6u * t], &c[i3], &c[i3 + 1]);
        }
        if (++j == quarter) { j = 0; base = (uint16_t)(base + L); }
    }
}

// Position p holds frequency index rta_perm_index(p): base-4 digit reversal,
// with the leading radix-2 digit moving to the bottom when mlog is odd.
static uint16_t rta_perm_index(uint16_t p, uint8_t mlog) {
    uint16_t lo = p, q = 0;
    uint8_t nd = (uint8_t)(mlog >> 1);
    if (mlog & 1u) {
        q = (uint16_t)(p >> (mlog - 1));
        lo = (uint16_t)(p & ((1u << (mlog - 1)) - 1u));
    }
    uint16_t r = 0;
    for (uint8_t i = 0; i < nd; i++) {
        r = (uint16_t)((r << 2) | (lo & 3u));
        lo = (uint16_t)(lo >> 2);
    }
    return (mlog & 1u) ? (uint16_t)((r << 1) | q) : r;
}

// Cycle-following permutation, resumable between cycles: units are start
// positions p0..p1-1, and seen (cleared before the first) marks placed slots.
static void rta_permute(rta_sample_t *c, uint8_t mlog, uint32_t *seen, uint16_t p0, uint16_t p1) {
    for (uint16_t p = p0; p < p1; p++) {
        if (seen[p >> 5] & (1u << (p & 31u))) continue;
        seen[p >> 5] |= 1u << (p & 31u);
        rta_sample_t tr = c[2 * p], ti = c[2 * p + 1];
        uint16_t cur = p;
        for (;;) {
            const uint16_t nxt = rta_perm_index(cur, mlog);
            if (nxt == p) { c[2 * p] = tr; c[2 * p + 1] = ti; break; }
            const rta_sample_t sr = c[2 * nxt], si = c[2 * nxt + 1];
            c[2 * nxt] = tr; c[2 * nxt + 1] = ti;
            tr = sr; ti = si;
            seen[nxt >> 5] |= 1u << (nxt & 31u);
            cur = nxt;
        }
    }
}

// Real split: X[k] = S - i*W*D, X[m-k] = conj(S + i*W*D), where S and D are the
// half sum and half difference of Z[k] and conj(Z[m-k]).  In Q15 the 1/sqrt(2)
// that rta_prescale took out comes back here, completing the 1/n scaling.
// Units k0..k1-1 of the pass (m / 2 + 1 in all): unit 0 is slot 0, unit k
// pairs bins k and m - k.
static void rta_split(rta_sample_t *c, uint8_t order, uint16_t k0, uint16_t k1) {
    const uint16_t m = (uint16_t)(1u << (order - 1));
    const uint16_t sp_stride = (uint16_t)(1u << (RTA_ORDER_MAX - order));

    if (k0 == 0) {   // X[0] and the Nyquist bin are both real and share slot 0.
        const rta_acc_t zr = c[0], zi = c[1];
        c[0] = rta_st(RTA_ISQRT2(zr + zi));
        c[1] = rta_st(RTA_ISQRT2(zr - zi));
        k0 = 1;
    }

    for (uint16_t k = k0; k < k1; k++) {
        const uint16_t kk = (uint16_t)(m - k);
        const rta_acc_t ar = c[2 * k], ai = c[2 * k + 1];
        const rta_acc_t br = c[2 * kk], bi = -(rta_acc_t)c[2 * kk + 1];
        const rta_acc_t sr = RTA_Q2(ar + br), si = RTA_Q2(ai + bi);
        const rta_acc_t dr = RTA_Q2(ar - br), di = RTA_Q2(ai - bi);
        const rta_tw_t *w = &rta_split_tw[2u * (uint32_t)(k * sp_stride)];

        // P = i * W * D
#if RTA_SAMPLE_FLOAT
        const float wr = w[0], wi = w[1];
        const rta_acc_t pr = -(dr * wi + di * wr);
        const rta_acc_t pi = dr * wr - di * wi;
#else
        const int32_t wr = w[0], wi = w[1];
        const rta_acc_t pr = -((dr * wi + di * wr + 16384) >> 15);
        const rta_acc_t pi = (dr * wr - di * wi + 16384) >> 15;
#endif
        c[2 * k] = rta_st(RTA_ISQRT2(sr - pr));
        c[2 * k + 1] = rta_st(RTA_ISQRT2(si - pi));
        if (kk != k) {
            c[2 * kk] = rta_st(RTA_ISQRT2(sr + pr));
            c[2 * kk + 1] = rta_st(-RTA_ISQRT2(si + pi));
        }
    }
}

bool rta_fft_step(rta_sample_t *buf, uint8_t order, RtaFftCursor *c, uint16_t max_units) {
    if (order < RTA_ORDER_MIN || order > RTA_ORDER_MAX) { c->stage = 0; c->pos = 0; return true; }
    if (max_units == 0) max_units = 1;

    const uint8_t mlog = (uint8_t)(order - 1);
    const uint16_t m = (uint16_t)(1u << mlog);
    const uint8_t has_r2 = (uint8_t)(mlog & 1u);
    const uint8_t n_bfly = (uint8_t)(has_r2 + (mlog >> 1));
    // Stage 0 prescales (Q15 only; float starts at 1), 1..n_bfly are butterfly
    // stages, then the permutation and the real split.
    uint8_t s = c->stage;
#if RTA_SAMPLE_FLOAT
    if (s == 0) c->stage = s = 1;
#endif
    uint16_t total;
    if (s == 0)                       total = (uint16_t)(1u << order);
    else if (s <= n_bfly)             total = (has_r2 && s == 1) ? (uint16_t)(m >> 1) : (uint16_t)(m >> 2);
    else if (s == n_bfly + 1)         total = m;
    else                              total = (uint16_t)((m >> 1) + 1);

    const uint16_t from = c->pos;
    const uint16_t to = (uint32_t)from + max_units < total ? (uint16_t)(from + max_units) : total;
    if (s == 0) {
#if !RTA_SAMPLE_FLOAT
        rta_prescale(buf, from, to);
#endif
    } else if (s <= n_bfly) {
        if (has_r2 && s == 1) {
            rta_stage_r2(buf, m, from, to);
        } else {
            const uint8_t k = (uint8_t)(s - 1 - has_r2);
            rta_stage_r4(buf, (uint16_t)((m >> has_r2) >> (2 * k)), from, to);
        }
    } else if (s == n_bfly + 1) {
        if (from == 0) memset(c->seen, 0, sizeof(c->seen));
        rta_permute(buf, mlog, c->seen, from, to);
    } else {
        rta_split(buf, order, from, to);
    }

    if (to < total) { c->pos = to; return false; }
    c->pos = 0;
    if (s == n_bfly + 2) { c->stage = 0; return true; }
    c->stage = (uint8_t)(s + 1);
    return false;
}

// ---------------------------------------------------------------------------
// Window, power, bands
// ---------------------------------------------------------------------------

// X[i] for i in [-3, n_bins + 3].  Slot 0 holds the real X[0] and the real
// Nyquist bin; the spectrum is conjugate-symmetric about both.
static inline void rta_bin_at(const rta_sample_t *buf, int32_t i, int32_t n_bins,
                              rta_acc_t *re, rta_acc_t *im) {
    rta_acc_t sign = 1;
    if (i < 0) { i = -i; sign = -1; }
    else if (i > n_bins) { i = 2 * n_bins - i; sign = -1; }
    if (i == 0 || i == n_bins) { *re = buf[i ? 1 : 0]; *im = 0; return; }
    *re = buf[2 * i];
    *im = sign * (rta_acc_t)buf[2 * i + 1];
}

void rta_fft_finish_begin(RtaFinishCursor *c) {
    memset(c, 0, sizeof(*c));
}

bool rta_fft_finish_step(RtaFinishCursor *c, const rta_sample_t *buf, uint8_t order,
                         const RtaBandTable *table, float *band_power, uint8_t *bins,
                         uint16_t max_bins) {
    if (order < RTA_ORDER_MIN || order > RTA_ORDER_MAX) return true;
    const uint16_t n_bins = (uint16_t)(1u << (order - 1));

    uint8_t n_bands = 0;
    if (band_power && table) {
        n_bands = table->n_bands;
        if (n_bands > RTA_MAX_BANDS) n_bands = RTA_MAX_BANDS;
    }

    uint32_t end = (uint32_t)c->next_bin + max_bins;
    if (end > n_bins || (!bins && !n_bands)) end = n_bins;

    // Float bins are n = 2 * n_bins times the 1/n-scaled spectrum (see the
    // macros at the top), so power carries n^2.  Powers of two keep this exact.
#if RTA_SAMPLE_FLOAT
    const float pscale = 1.0f / (float)(1u << (2u * order));
    const float bin_scale = 16.0f * pscale;
#else
    const float bin_scale = 0.0f;
#endif

    if (bins || n_bands) {
        uint8_t b0 = c->band;
        for (uint16_t k = c->next_bin; k < end; k++) {
            rta_acc_t wr, wi;
            if (k >= 4 && k + 4 <= n_bins) {
                // Interior: all 7 taps are plain complex slots 1..n_bins-1.
                const rta_sample_t *x = &buf[2 * k - 6];
                wr = rta_window(x[0], x[2], x[4], x[6], x[8], x[10], x[12]);
                wi = rta_window(x[1], x[3], x[5], x[7], x[9], x[11], x[13]);
            } else {
                rta_acc_t re[7], im[7];
                for (int32_t j = 0; j < 7; j++) {
                    rta_bin_at(buf, (int32_t)k + j - 3, n_bins, &re[j], &im[j]);
                }
                wr = rta_window(re[0], re[1], re[2], re[3], re[4], re[5], re[6]);
                wi = rta_window(im[0], im[1], im[2], im[3], im[4], im[5], im[6]);
            }
            const rta_binpow_t p = rta_bin_power(wr, wi);
            if (bins) bins[k] = rta_bin_level(p, bin_scale);

            if (n_bands) {
                while (b0 < n_bands && table->hi[b0] < k) b0++;
                for (uint8_t b = b0; b < n_bands && table->lo[b] <= k; b++) {
                    if (k <= table->hi[b]) c->acc[b] += p;
                }
            }
        }
        c->band = b0;
    }
    c->next_bin = (uint16_t)end;
    if (end < n_bins) return false;

    // No table: bands read the floor rather than stack garbage.
    if (band_power) {
        memset(band_power, 0, RTA_MAX_BANDS * sizeof(float));
        for (uint8_t b = 0; b < n_bands; b++) {
#if RTA_SAMPLE_FLOAT
            band_power[b] = c->acc[b] * (RTA_BAND_SCALE * pscale);
#else
            band_power[b] = (float)c->acc[b] * RTA_BAND_SCALE;
#endif
        }
    }
    return true;
}

void rta_fft_finish(const rta_sample_t *buf, uint8_t order,
                    const RtaBandTable *table, float *band_power, uint8_t *bins) {
    RtaFinishCursor c;
    rta_fft_finish_begin(&c);
    (void)rta_fft_finish_step(&c, buf, order, table, band_power, bins, UINT16_MAX);
}
