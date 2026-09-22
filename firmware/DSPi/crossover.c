/*
 * crossover.c — High-order crossover filters (LR / BW / Bessel).
 *
 * Each user-visible "band" is a cascade of up to 4 biquad sections. The
 * cascade decomposes by family:
 *
 *   Linkwitz-Riley_{2N} = (BW_N)² (two cascaded BW_N filters)
 *       LR2 = single biquad (double real pole)
 *       LR4 = 2 biquads (BW2 doubled)
 *       LR8 = 4 biquads (BW4 doubled)
 *
 *   Butterworth_N: N/2 biquads (even N) or 1 first-order + (N-1)/2 biquads (odd N)
 *
 *   Bessel_N: N/2 biquads (only even N supported: 2, 4, 6, 8)
 *
 * Each section is built from an analog 2nd-order (or 1st-order) prototype via
 * the bilinear transform with frequency prewarping. The output filter coeffs
 * feed straight into the existing per-platform kernel (TDF2 on RP2040, hybrid
 * SVF/TDF2 on RP2350). First-order sections always use TDF2 — the Cytomic SVF
 * is fundamentally 2nd-order topology.
 *
 * Cascade ordering: low-Q sections first, for TDF2 numerical stability
 * (Oppenheim & Schafer). For odd-order BW, the 1st-order section is placed
 * first regardless of Q (its effective Q = 0.5 ≤ any biquad).
 *
 * See Documentation/Features/crossover_filters_spec.md.
 */

#include <math.h>
#include <string.h>

#include "crossover.h"
#include "config.h"
#include "dsp_pipeline.h"   // Filter layout, FILTER_SHIFT, FILTER_LOWPASS/HIGHPASS
#include "dsp_cascade.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// =============================================================================
// Storage
// =============================================================================

XoverFilter   xover_filters[NUM_OUTPUT_CHANNELS][MAX_XOVER_BANDS];
EqParamPacket xover_recipes[NUM_CHANNELS][MAX_XOVER_BANDS];
bool          output_xover_bypassed[NUM_OUTPUT_CHANNELS];

// =============================================================================
// Filter-type → metadata
// =============================================================================

// Indexed by (type - FILTER_XOVER_FIRST). Order in this table MUST match the
// FilterType enum's crossover ordering exactly.
typedef struct {
    uint8_t family;        // XoverFamily
    uint8_t order;         // 1..8
    uint8_t is_highpass;   // 0 = LP, 1 = HP
    uint8_t num_sections;  // 1..4
} XoverTypeMeta;

// Compile-time guard: this table is indexed by (type - FILTER_XOVER_FIRST),
// so every value in the [FIRST..LAST] range MUST have exactly one entry. A
// future enum reorder that drifts table length from enum span would silently
// misroute filter design.
#define XOVER_TYPE_COUNT (FILTER_XOVER_LAST - FILTER_XOVER_FIRST + 1)

static const XoverTypeMeta xover_type_table[] = {
    // FILTER_LR2_LP=32 .. FILTER_LR8_HP=39
    { XOVER_FAMILY_LR,  2, 0, 1 },  // LR2 LP
    { XOVER_FAMILY_LR,  2, 1, 1 },  // LR2 HP
    { XOVER_FAMILY_LR,  4, 0, 2 },  // LR4 LP
    { XOVER_FAMILY_LR,  4, 1, 2 },  // LR4 HP
    { XOVER_FAMILY_LR,  6, 0, 4 },  // LR6 LP — (BW3)² = 2 first-order + 2 biquads
    { XOVER_FAMILY_LR,  6, 1, 4 },  // LR6 HP
    { XOVER_FAMILY_LR,  8, 0, 4 },  // LR8 LP
    { XOVER_FAMILY_LR,  8, 1, 4 },  // LR8 HP

    // FILTER_BW1_LP=40 .. FILTER_BW8_HP=55
    { XOVER_FAMILY_BW,  1, 0, 1 },  // BW1 LP
    { XOVER_FAMILY_BW,  1, 1, 1 },  // BW1 HP
    { XOVER_FAMILY_BW,  2, 0, 1 },  // BW2 LP
    { XOVER_FAMILY_BW,  2, 1, 1 },  // BW2 HP
    { XOVER_FAMILY_BW,  3, 0, 2 },  // BW3 LP
    { XOVER_FAMILY_BW,  3, 1, 2 },  // BW3 HP
    { XOVER_FAMILY_BW,  4, 0, 2 },  // BW4 LP
    { XOVER_FAMILY_BW,  4, 1, 2 },  // BW4 HP
    { XOVER_FAMILY_BW,  5, 0, 3 },  // BW5 LP
    { XOVER_FAMILY_BW,  5, 1, 3 },  // BW5 HP
    { XOVER_FAMILY_BW,  6, 0, 3 },  // BW6 LP
    { XOVER_FAMILY_BW,  6, 1, 3 },  // BW6 HP
    { XOVER_FAMILY_BW,  7, 0, 4 },  // BW7 LP
    { XOVER_FAMILY_BW,  7, 1, 4 },  // BW7 HP
    { XOVER_FAMILY_BW,  8, 0, 4 },  // BW8 LP
    { XOVER_FAMILY_BW,  8, 1, 4 },  // BW8 HP

    // FILTER_BES2_LP=56 .. FILTER_BES8_HP=63
    { XOVER_FAMILY_BES, 2, 0, 1 },  // Bes2 LP
    { XOVER_FAMILY_BES, 2, 1, 1 },  // Bes2 HP
    { XOVER_FAMILY_BES, 4, 0, 2 },  // Bes4 LP
    { XOVER_FAMILY_BES, 4, 1, 2 },  // Bes4 HP
    { XOVER_FAMILY_BES, 6, 0, 3 },  // Bes6 LP
    { XOVER_FAMILY_BES, 6, 1, 3 },  // Bes6 HP
    { XOVER_FAMILY_BES, 8, 0, 4 },  // Bes8 LP
    { XOVER_FAMILY_BES, 8, 1, 4 },  // Bes8 HP
};

_Static_assert(sizeof(xover_type_table) / sizeof(xover_type_table[0]) == XOVER_TYPE_COUNT,
               "xover_type_table size must match the FilterType crossover enum span");

bool xover_filter_meta(uint8_t type, XoverFilterMeta *out) {
    if (out) memset(out, 0, sizeof(*out));
    if (type < FILTER_XOVER_FIRST || type > FILTER_XOVER_LAST) return false;
    const XoverTypeMeta *t = &xover_type_table[type - FILTER_XOVER_FIRST];
    if (out) {
        out->family       = t->family;
        out->order        = t->order;
        out->is_highpass  = t->is_highpass;
        out->num_sections = t->num_sections;
    }
    return true;
}

// =============================================================================
// Analog prototype pole tables
//
// All poles given as (σ_n, ω_n) normalized to unit cutoff (σ_n² + ω_n² ≈ 1 for
// Butterworth, varying for Bessel). Cascade order: low-Q first.
// =============================================================================

typedef struct {
    float sigma;   // Real part magnitude of left-half-plane pole
    float omega;   // Imaginary part magnitude
} AnalogPolePair;

// Butterworth: poles equally spaced on unit semi-circle. For order N, the
// conjugate-pair angles measured from the negative real axis are
//   θ_k = π·(2k - 1) / (2N), k = 1 .. ceil(N/2)
// giving σ_n = cos(θ_k), ω_n = sin(θ_k). Pairs are listed in ascending θ,
// which is ascending Q (Q_k = 1 / (2·σ_n_k)), so they're already in
// stability-preferred cascade order.
//
// For odd N the LAST pole is real (-1), implemented as a 1st-order section
// placed BEFORE the biquad pairs by design_butterworth() — its conceptual
// Q is 0.5, lower than any biquad.
static void bw_pole_pair(uint8_t order, uint8_t pair_idx, AnalogPolePair *out) {
    // pair_idx counts only complex pairs (not the real pole on odd orders).
    // For order N, there are floor(N/2) pairs. Pair k uses angle:
    //   θ = π·(2k + (N & 1) + 1) / (2N) ... too cute. Use the direct form:
    // The first complex pair (lowest Q) sits at the angle nearest the
    // negative-real axis, advancing toward ±jω with each subsequent pair.
    float theta;
    if (order & 1u) {
        // Odd order: real pole at θ=π/2 from negative-real axis is the "last"
        // (most-imaginary) which is actually purely real. Complex pair angles
        // step from (π/N) outward.
        theta = (float)M_PI * (float)(pair_idx + 1) / (float)order;
    } else {
        // Even order: complex pairs at (2k - 1) · π / (2N)
        theta = (float)M_PI * (float)(2 * pair_idx + 1) / (float)(2 * order);
    }
    out->sigma = cosf(theta);   // sigma > 0 since θ ∈ (0, π/2)
    out->omega = sinf(theta);
}

// Bessel (-3 dB normalized) pole tables. Values verified against scipy's
// `signal.bessel(N, 1.0, btype='lowpass', analog=True, norm='mag')` to
// 5 decimal places; cross-referenced with Williams/Taylor "Electronic
// Filter Design Handbook" and Analog Devices MT-201.  Ordered low-Q → high-Q.
//
// Each row is a conjugate pair (σ_n, ω_n).  Only even orders supported.
// Validated by tools/filter_tester/test_crossover.py (sweep against scipy
// as independent reference).
static const AnalogPolePair bessel2_pairs[1] = {
    { 1.10160f, 0.63601f },                          // Q ≈ 0.577
};
static const AnalogPolePair bessel4_pairs[2] = {
    { 1.37007f, 0.41025f },                          // Q ≈ 0.522
    { 0.99521f, 1.25711f },                          // Q ≈ 0.806
};
static const AnalogPolePair bessel6_pairs[3] = {
    { 1.57149f, 0.32090f },                          // Q ≈ 0.510
    { 1.38186f, 0.97147f },                          // Q ≈ 0.611
    { 0.93066f, 1.66186f },                          // Q ≈ 1.023
};
static const AnalogPolePair bessel8_pairs[4] = {
    { 1.75741f, 0.27287f },                          // Q ≈ 0.506
    { 1.63694f, 0.82280f },                          // Q ≈ 0.560
    { 1.37384f, 1.38836f },                          // Q ≈ 0.711
    { 0.89287f, 1.99833f },                          // Q ≈ 1.226 — was 0.90932, 1.95710 (table typo, caught by test_crossover.py)
};

static const AnalogPolePair *bessel_table(uint8_t order, uint8_t *count) {
    switch (order) {
        case 2: *count = 1; return bessel2_pairs;
        case 4: *count = 2; return bessel4_pairs;
        case 6: *count = 3; return bessel6_pairs;
        case 8: *count = 4; return bessel8_pairs;
        default: *count = 0; return NULL;
    }
}

// =============================================================================
// Per-section design (bilinear transform with frequency prewarping)
//
// Given a normalized analog pole pair (σ_n, ω_n) and a target digital cutoff
// fc Hz at sample rate Fs Hz, build a filter section. Frequency prewarping is
// applied at the FILTER level: ω_a = 2·Fs·tan(π·fc/Fs). Per-section poles are
// then σ = σ_n · ω_a, ω = ω_n · ω_a. The bilinear transform with K = 2·Fs
// produces the z-domain coefficients.
// =============================================================================

// Zero state + set bq as unity-passthrough (used for bypassed sections).
static void filter_set_passthrough(Filter *f) {
    memset(f, 0, sizeof(*f));
#if PICO_RP2350
    f->b0 = 1.0f;
    f->use_svf = false;
    f->filter_type = FILTER_FLAT;
    f->bypass = true;
#else
    f->b0 = 1 << FILTER_SHIFT;
    f->bypass = true;
#endif
}

#if PICO_RP2350
// RP2350: choose between SVF (low fc) and TDF2 biquad per section. Section
// state (s1/s2 or svic1eq/svic2eq) is cleared whenever use_svf changes,
// matching the existing PEQ convention.
static void filter_assign_2nd_order_rp2350(Filter *f,
                                           float sigma, float omega,
                                           bool is_hp, float Fs, float fc) {
    bool was_svf = f->use_svf;

    // Per-section SVF/TDF2 decision. For our scope the section radius (and
    // therefore the section's effective frequency relative to fc) is always
    // close to 1, so we apply the same filter-level fc/Fs gate the PEQ uses.
    f->use_svf = (fc < (Fs / 7.5f));

    if (was_svf != f->use_svf) {
        f->s1 = 0.0f; f->s2 = 0.0f;
        f->svic1eq = 0.0f; f->svic2eq = 0.0f;
    }
    f->first_order = false;   // 2nd-order section uses the 2-pole SVF path

    if(is_hp)
        f->filter_type = FILTER_HIGHPASS;
    else
        f->filter_type = FILTER_LOWPASS;

    if (f->use_svf) {
        // Cytomic SVF (TPT). For a 2nd-order section with prewarped pole
        // (σ, ω), the equivalent (f₀, Q) parameters are:
        //   ω₀ = √(σ² + ω²)        (analog rad/s)
        //   Q  = ω₀ / (2·σ)
        // SVF expects digital g = tan(π · f₀_digital / Fs). f₀_digital is
        // the digital image of ω₀; since the whole filter is prewarped at
        // fc, ω₀ = ω_a · √(σ_n² + ω_n²), and digital f₀ = atan(ω₀/(2·Fs))·Fs/π.
        // Equivalently g = ω₀ / (2·Fs), which IS the prewarped g. We use
        // this direct form to avoid a redundant atan→tan round-trip.
        float w0 = sqrtf(sigma * sigma + omega * omega);   // analog rad/s
        float Q  = w0 / (2.0f * sigma);
        float g  = w0 / (2.0f * Fs);
        float k  = 1.0f / Q;

        float sva1 = 1.0f / (1.0f + g * (g + k));
        float sva2 = g * sva1;
        float sva3 = g * sva2;

        f->sva1 = sva1;
        f->sva2 = sva2;
        f->sva3 = sva3;
        f->g = g;
        if (is_hp) {
            f->svm0 = 1.0f; f->svm1 = -k; f->svm2 = -1.0f;
        } else {
            f->svm0 = 0.0f; f->svm1 = 0.0f; f->svm2 = 1.0f;
        }
        // Fallback biquad coeffs (unused in SVF path, keep sane)
        f->b0 = 1.0f; f->b1 = 0.0f; f->b2 = 0.0f;
        f->a1 = 0.0f; f->a2 = 0.0f;
        f->bypass = false;
        return;
    }

    // TDF2 path on RP2350: bilinear transform directly.
    float K  = 2.0f * Fs;
    float A  = 2.0f * sigma;             // coefficient of s in (s+σ)²+ω²
    float B  = sigma * sigma + omega * omega;
    float A0 = K * K + A * K + B;
    float A1 = 2.0f * (B - K * K);
    float A2 = K * K - A * K + B;

    float inv = 1.0f / A0;
    if (is_hp) {
        f->b0 = (K * K) * inv;
        f->b1 = (-2.0f * K * K) * inv;
        f->b2 = (K * K) * inv;
    } else {
        f->b0 = B * inv;
        f->b1 = (2.0f * B) * inv;
        f->b2 = B * inv;
    }
    f->a1 = A1 * inv;
    f->a2 = A2 * inv;
    f->sva1 = 0.0f; f->sva2 = 0.0f; f->sva3 = 0.0f;
    f->svm0 = 0.0f; f->svm1 = 0.0f; f->svm2 = 0.0f;
    f->bypass = false;
}

static void filter_assign_1st_order_rp2350(Filter *f,
                                           float sigma_real,
                                           bool is_hp, float Fs, float fc) {
    // First-order section: pole at -sigma_real (already denormalised).
    // Follows the same hybrid rule as the 2nd-order path: one-pole SVF below
    // Fs/7.5, degenerate TDF2 biquad above.  Section state is cleared when the
    // topology changes (matching the 2nd-order convention).
    bool was_svf = f->use_svf;
    f->use_svf = (fc < (Fs / 7.5f));
    if (was_svf != f->use_svf) {
        f->s1 = 0.0f; f->s2 = 0.0f;
        f->svic1eq = 0.0f; f->svic2eq = 0.0f;
    }

    f->first_order = true;

    if(is_hp)
        f->filter_type = FILTER_HIGHPASS1;
    else
        f->filter_type = FILTER_LOWPASS1;

    if (f->use_svf) {
        // One-pole TPT SVF.  g = sigma_real/(2*Fs) is the prewarped one-pole
        // gain (same direct form as the 2nd-order section; avoids an atan->tan
        // round-trip), yielding a pole identical to the biquad form.  The
        // 1/(1+g) reciprocal is folded into sva1 so the loop is multiply-only.
        float g    = sigma_real / (2.0f * Fs);
        float sva1 = 1.0f / (1.0f + g);
        f->sva1 = sva1;
        f->sva2 = g * sva1;
        f->sva3 = 0.0f;
        if (is_hp) {
            f->svm0 = 0.0f; f->svm1 = 0.0f; f->svm2 = 1.0f;   // out = in - lp
        } else {
            f->svm0 = 0.0f; f->svm1 = 1.0f; f->svm2 = 0.0f;   // out = lp
        }
        f->b0 = 1.0f; f->b1 = 0.0f; f->b2 = 0.0f; f->a1 = 0.0f; f->a2 = 0.0f;
        f->bypass = false;
        return;
    }

    float K  = 2.0f * Fs;
    float A0 = K + sigma_real;
    float A1 = sigma_real - K;
    float inv = 1.0f / A0;

    if (is_hp) {
        f->b0 = K * inv;
        f->b1 = -K * inv;
    } else {
        f->b0 = sigma_real * inv;
        f->b1 = sigma_real * inv;
    }
    f->b2 = 0.0f;
    f->a1 = A1 * inv;
    f->a2 = 0.0f;
    f->sva1 = 0.0f; f->sva2 = 0.0f; f->sva3 = 0.0f;
    f->svm0 = 0.0f; f->svm1 = 0.0f; f->svm2 = 0.0f;
    f->bypass = false;
}
#else
// RP2040: Q28 fixed-point TDF2 biquad. Same bilinear math, scaled into Q28.
static void filter_assign_2nd_order_rp2040(Filter *f,
                                           float sigma, float omega,
                                           bool is_hp, float Fs) {
    float K  = 2.0f * Fs;
    float A  = 2.0f * sigma;
    float B  = sigma * sigma + omega * omega;
    float A0 = K * K + A * K + B;
    float A1 = 2.0f * (B - K * K);
    float A2 = K * K - A * K + B;

    float scale = (float)(1LL << FILTER_SHIFT);
    float inv   = scale / A0;

    if (is_hp) {
        f->b0 = (int32_t)((K * K) * inv);
        f->b1 = (int32_t)((-2.0f * K * K) * inv);
        f->b2 = (int32_t)((K * K) * inv);
    } else {
        f->b0 = (int32_t)(B * inv);
        f->b1 = (int32_t)((2.0f * B) * inv);
        f->b2 = (int32_t)(B * inv);
    }
    f->a1 = (int32_t)(A1 * inv);
    f->a2 = (int32_t)(A2 * inv);
    f->bypass = false;
}

static void filter_assign_1st_order_rp2040(Filter *f,
                                           float sigma_real,
                                           bool is_hp, float Fs) {
    float K  = 2.0f * Fs;
    float A0 = K + sigma_real;
    float A1 = sigma_real - K;
    float scale = (float)(1LL << FILTER_SHIFT);
    float inv   = scale / A0;

    if (is_hp) {
        f->b0 = (int32_t)(K * inv);
        f->b1 = (int32_t)(-K * inv);
    } else {
        f->b0 = (int32_t)(sigma_real * inv);
        f->b1 = (int32_t)(sigma_real * inv);
    }
    f->b2 = 0;
    f->a1 = (int32_t)(A1 * inv);
    f->a2 = 0;
    f->bypass = false;
}
#endif

// Thin per-platform wrappers so the family-design code below stays unified.
//
// IMPORTANT: the analog LP→HP transform `s → ωc/s` inverts each pole through
// the unit circle. For a conjugate pair (-σ_n ± jω_n), the corresponding HP
// pair is (-σ_n/r² ± jω_n/r²) where r² = σ_n² + ω_n². For Butterworth and
// Linkwitz-Riley, every prototype pole lies exactly on the unit circle
// (r² = 1) so the reciprocal coincides with the original and a numerator-only
// HP path is correct. For Bessel, r² varies per section (~1.6 .. ~4.6
// depending on order and section index), so the LP poles are NOT the HP
// poles — we must apply the reciprocal explicitly here. The transform is a
// no-op for r² = 1, so applying it unconditionally is correct for all
// families and required by Bessel.
static void section_emit_2nd_order(Filter *f, float sigma_n, float omega_n,
                                   float omega_a, bool is_hp,
                                   float Fs, float fc) {
    if (is_hp) {
        float r2 = sigma_n * sigma_n + omega_n * omega_n;
        if (r2 > 0.0f) {
            sigma_n /= r2;
            omega_n /= r2;
        }
        // r2 == 0 would mean a pole at the origin, which no supported family
        // produces; the guard avoids a divide-by-zero if a future caller
        // misuses this helper.
    }
    float sigma = sigma_n * omega_a;
    float omega = omega_n * omega_a;
#if PICO_RP2350
    filter_assign_2nd_order_rp2350(f, sigma, omega, is_hp, Fs, fc);
#else
    (void)fc;
    filter_assign_2nd_order_rp2040(f, sigma, omega, is_hp, Fs);
#endif
}

static void section_emit_1st_order(Filter *f, float sigma_n,
                                   float omega_a, bool is_hp, float Fs,
                                   float fc) {
    if (is_hp && sigma_n > 0.0f) {
        // LP→HP on a real-pole 1st-order: pole at -σ_n → pole at -1/σ_n.
        // For Butterworth (σ_n = 1) the reciprocal is identity, so this is
        // a no-op for BW1/3/5/7. Kept for symmetry with the 2nd-order path
        // so a future family with a real pole off the unit circle (e.g.
        // odd-order Bessel) is handled correctly without surprise.
        sigma_n = 1.0f / sigma_n;
    }
    float sigma = sigma_n * omega_a;
#if PICO_RP2350
    filter_assign_1st_order_rp2350(f, sigma, is_hp, Fs, fc);
#else
    (void)fc;
    filter_assign_1st_order_rp2040(f, sigma, is_hp, Fs);
#endif
}

// =============================================================================
// Family-level design (cascade assembly)
// =============================================================================

// Build a Butterworth cascade of `order` into `band` (overwrites num_sections,
// sections, bypass). For odd order, places the 1st-order section first.
static void design_butterworth(XoverFilter *band,
                               uint8_t order, bool is_hp,
                               float omega_a, float Fs, float fc) {
    uint8_t idx = 0;
    if (order & 1u) {
        // Real pole at σ_n = 1 (Butterworth N has its real pole on the unit
        // circle at -1).
        section_emit_1st_order(&band->sections[idx++], 1.0f, omega_a, is_hp, Fs, fc);
    }
    uint8_t pairs = order / 2;
    for (uint8_t p = 0; p < pairs; p++) {
        AnalogPolePair pole;
        bw_pole_pair(order, p, &pole);
        section_emit_2nd_order(&band->sections[idx++],
                               pole.sigma, pole.omega,
                               omega_a, is_hp, Fs, fc);
    }
    band->num_sections = idx;
}

// LR_{2N} = cascade of two BW_N filters. Implementation: design BW_N once,
// then duplicate every section. For LR2 (= BW1²), the canonical realisation
// is a single biquad with a double real pole at -1, equivalent to two
// cascaded 1st-order sections — we use the single-biquad form for tighter
// state and one less per-sample multiply.
//
// For odd-N BW (LR6 = BW3²), BW_N includes a 1st-order section in addition
// to the biquad(s). We emit BW_N's sections in their canonical "1st-order
// first, then biquads in ascending angle" order, then duplicate the whole
// cascade. Resulting layout for each supported LR:
//   LR2 → [biq]
//   LR4 → [biq0, biq0_dup]
//   LR6 → [1st, biq0, 1st_dup, biq0_dup]
//   LR8 → [biq0, biq1, biq0_dup, biq1_dup]
static void design_linkwitz_riley(XoverFilter *band,
                                  uint8_t order_lr, bool is_hp,
                                  float omega_a, float Fs, float fc) {
    if (order_lr == 2) {
        // (BW1)² → one 2nd-order section with σ_n=1, ω_n=0.
        section_emit_2nd_order(&band->sections[0],
                               1.0f, 0.0f, omega_a, is_hp, Fs, fc);
        band->num_sections = 1;
        return;
    }
    // LR_{2N} = (BW_N)² for N ∈ {2, 3, 4} → LR4/LR6/LR8.
    uint8_t bw_order = order_lr / 2;
    uint8_t pairs    = bw_order / 2;
    bool    has_1st  = (bw_order & 1u) != 0;
    uint8_t idx      = 0;

    // ---- Emit BW_N's sections (1st-order first, then biquads) ----
    if (has_1st) {
        // Butterworth real pole at σ_n = 1.
        section_emit_1st_order(&band->sections[idx++], 1.0f, omega_a, is_hp, Fs, fc);
    }
    for (uint8_t p = 0; p < pairs; p++) {
        AnalogPolePair pole;
        bw_pole_pair(bw_order, p, &pole);
        section_emit_2nd_order(&band->sections[idx++],
                               pole.sigma, pole.omega,
                               omega_a, is_hp, Fs, fc);
    }

    // ---- Duplicate the BW_N cascade for (BW_N)² ----
    // Same coefficients, fresh state on each duplicate so the two cascaded
    // copies don't share internal state.
    uint8_t bw_section_count = idx;
    for (uint8_t i = 0; i < bw_section_count; i++) {
        Filter copy = band->sections[i];
        copy.s1 = copy.s2 = 0;
#if PICO_RP2350
        copy.svic1eq = copy.svic2eq = 0.0f;
#endif
        band->sections[idx++] = copy;
    }
    band->num_sections = idx;
}

static void design_bessel(XoverFilter *band,
                          uint8_t order, bool is_hp,
                          float omega_a, float Fs, float fc) {
    uint8_t count = 0;
    const AnalogPolePair *table = bessel_table(order, &count);
    if (!table || count == 0) {
        band->num_sections = 0;
        band->bypass = true;
        return;
    }
    for (uint8_t p = 0; p < count; p++) {
        section_emit_2nd_order(&band->sections[p],
                               table[p].sigma, table[p].omega,
                               omega_a, is_hp, Fs, fc);
    }
    band->num_sections = count;
}

// =============================================================================
// Public: design one band
// =============================================================================

void xover_design_filter(const EqParamPacket *recipe,
                         XoverFilter *band,
                         float sample_rate) {
    if (!band) return;

    // Reset bypass before deciding. Bypass-byte normalisation matches PEQ
    // (== 1 strictly bypasses; any other value leaves active).
    bool user_bypass = (recipe && recipe->bypass == 1);

    // Snapshot per-section filter state before the passthrough reset below
    // wipes it. A live redesign (e.g. the user dragging fc) must keep the
    // cascade's memory or the output steps discontinuously on every update;
    // PEQ has the same convention (dsp_compute_coefficients preserves s1/s2
    // across coefficient updates, resetting only on an SVF/TDF2 path change).
#if PICO_RP2350
    float old_s1[MAX_XOVER_BAND_SECTIONS], old_s2[MAX_XOVER_BAND_SECTIONS];
    float old_ic1[MAX_XOVER_BAND_SECTIONS], old_ic2[MAX_XOVER_BAND_SECTIONS];
    bool  old_svf[MAX_XOVER_BAND_SECTIONS];
    bool  old_first_order[MAX_XOVER_BAND_SECTIONS];
#else
    int32_t old_s1[MAX_XOVER_BAND_SECTIONS], old_s2[MAX_XOVER_BAND_SECTIONS];
#endif
    uint8_t old_active = (band->num_sections <= MAX_XOVER_BAND_SECTIONS)
                         ? band->num_sections : 0;
    for (uint8_t i = 0; i < old_active; i++) {
        old_s1[i] = band->sections[i].s1;
        old_s2[i] = band->sections[i].s2;
#if PICO_RP2350
        old_ic1[i] = band->sections[i].svic1eq;
        old_ic2[i] = band->sections[i].svic2eq;
        old_svf[i] = band->sections[i].use_svf;
        old_first_order[i] = band->sections[i].first_order;
#endif
    }

    // Mark all sections passthrough first, so any unused slots are inert
    // even if the design fails partway.
    for (uint8_t i = 0; i < MAX_XOVER_BAND_SECTIONS; i++) {
        filter_set_passthrough(&band->sections[i]);
    }
    band->num_sections = 0;
    band->bypass = true;

    if (!recipe || user_bypass || sample_rate <= 0.0f) return;

    XoverFilterMeta meta;
    if (!xover_filter_meta(recipe->type, &meta)) return;   // Non-crossover type → bypassed

    // Clamp fc to a safe range — same as PEQ.
    float fc = recipe->freq;
    if (fc < 10.0f) fc = 10.0f;
    if (fc > sample_rate * 0.45f) fc = sample_rate * 0.45f;

    float omega_a = 2.0f * sample_rate *
                    tanf((float)M_PI * fc / sample_rate);   // prewarp

    switch (meta.family) {
        case XOVER_FAMILY_BW:
            design_butterworth(band, meta.order, meta.is_highpass != 0,
                               omega_a, sample_rate, fc);
            break;
        case XOVER_FAMILY_LR:
            design_linkwitz_riley(band, meta.order, meta.is_highpass != 0,
                                  omega_a, sample_rate, fc);
            break;
        case XOVER_FAMILY_BES:
            design_bessel(band, meta.order, meta.is_highpass != 0,
                          omega_a, sample_rate, fc);
            break;
        default:
            return;  // band stays bypassed
    }

    if (band->num_sections > 0 && band->num_sections <= MAX_XOVER_BAND_SECTIONS) {
        band->bypass = false;
        // Restore surviving sections' state (the passthrough reset above
        // zeroed it). A section that was bypassed, dropped out of the new
        // cascade, or changed topology (SVF/TDF2 path, or 1st/2nd-order via
        // first_order) stays zeroed, matching the PEQ path-change convention.
        uint8_t keep = (band->num_sections < old_active)
                       ? band->num_sections : old_active;
        for (uint8_t i = 0; i < keep; i++) {
#if PICO_RP2350
            if (band->sections[i].use_svf != old_svf[i] ||
                band->sections[i].first_order != old_first_order[i]) continue;
            band->sections[i].svic1eq = old_ic1[i];
            band->sections[i].svic2eq = old_ic2[i];
#endif
            band->sections[i].s1 = old_s1[i];
            band->sections[i].s2 = old_s2[i];
        }
    }
}

// =============================================================================
// Init / recompute / bypass helpers
// =============================================================================

void xover_update_output_bypass(uint8_t out) {
    if (out >= NUM_OUTPUT_CHANNELS) return;
    bool all = true;
    for (uint8_t b = 0; b < MAX_XOVER_BANDS; b++) {
        if (!xover_filters[out][b].bypass) { all = false; break; }
    }
    output_xover_bypassed[out] = all;
}

void xover_init_default_filters(void) {
    memset(xover_filters, 0, sizeof(xover_filters));
    memset(xover_recipes, 0, sizeof(xover_recipes));
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        for (int i = 0; i < MAX_XOVER_BANDS; i++) {
            EqParamPacket *r = &xover_recipes[ch][i];
            r->channel  = (uint8_t)ch;
            // CRITICAL: store the WIRE band index (XOVER_BAND_BASE + i), not
            // the local index. Live-edit dispatch in main.c reads p.band to
            // route the update; if a stale local-index leaked into
            // pending_packet via REQ_SET_BAND_BYPASS we'd misroute the
            // write to PEQ band 0. See crossover_filters_spec.md for the
            // full discussion.
            r->band     = (uint8_t)(XOVER_BAND_BASE + i);
            r->type     = FILTER_FLAT;
            r->bypass   = 0;
            r->freq     = 1000.0f;
            r->Q        = 0.707f;
            r->gain_db  = 0.0f;
        }
    }
    for (int out = 0; out < NUM_OUTPUT_CHANNELS; out++) {
        for (int i = 0; i < MAX_XOVER_BANDS; i++) {
            // Design with FILTER_FLAT yields a bypassed band.
            xover_design_filter(&xover_recipes[CH_OUT_1 + out][i],
                                &xover_filters[out][i], 0.0f);
            xover_filters[out][i].bypass = true;
        }
        output_xover_bypassed[out] = true;
    }
}

void xover_recalculate_all(float sample_rate) {
    for (int out = 0; out < NUM_OUTPUT_CHANNELS; out++) {
        for (int i = 0; i < MAX_XOVER_BANDS; i++) {
            xover_design_filter(&xover_recipes[CH_OUT_1 + out][i],
                                &xover_filters[out][i],
                                sample_rate);
        }
        xover_update_output_bypass((uint8_t)out);
    }
}

// =============================================================================
// Processing kernel
//
// Each band is a cascade of section_count Filters.
// =============================================================================

#if PICO_RP2350
DSP_TIME_CRITICAL
void xover_process_channel_block(XoverFilter *bands,
                                 float * __restrict samples,
                                 uint32_t sample_count) {
    for (uint8_t b = 0; b < MAX_XOVER_BANDS; b++) {
        XoverFilter *band = &bands[b];
        if (band->bypass || band->num_sections == 0) continue;
        // Defensive upper bound on num_sections — see RP2040 wrapper below
        // for rationale.  Without this a corrupt num_sections would step
        // past the sections[] array on the float path too.
        uint8_t n = band->num_sections;
        if (n > MAX_XOVER_BAND_SECTIONS) n = MAX_XOVER_BAND_SECTIONS;
        dsp_cascade_block(band->sections, n, samples, sample_count);
    }
}
#else
// RP2040: delegates to a shared assembly entry point that walks a fixed
// section count. The asm body is identical to dsp_process_channel_block()'s
// inner loop, just driven from num_sections instead of
// channel_band_counts[channel]. See dsp_process_rp2040.S.
extern void dsp_process_band_cascade_block(Filter * __restrict sections,
                                           int32_t * __restrict samples,
                                           uint32_t sample_count,
                                           uint32_t num_sections);

DSP_TIME_CRITICAL
void xover_process_channel_block(XoverFilter *bands,
                                 int32_t * __restrict samples,
                                 uint32_t sample_count) {
    for (uint8_t b = 0; b < MAX_XOVER_BANDS; b++) {
        XoverFilter *band = &bands[b];
        if (band->bypass || band->num_sections == 0) continue;
        // Defensive upper bound: the assembly entry trusts the loop count
        // and would walk past the cascade array if num_sections were corrupt.
        // Storage contract limits it to [1..MAX_XOVER_BAND_SECTIONS]; the
        // check belt-and-suspenders flash/RAM corruption.
        if (band->num_sections > MAX_XOVER_BAND_SECTIONS) continue;
        dsp_process_band_cascade_block(band->sections, samples,
                                       sample_count, band->num_sections);
    }
}
#endif
