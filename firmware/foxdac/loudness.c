#include <math.h>
#include <string.h>
#include "loudness.h"

// Double-buffered loudness coefficient tables
LoudnessCoeffs loudness_tables[2][LOUDNESS_VOL_STEPS][LOUDNESS_BIQUAD_COUNT];
LoudnessCoeffs (*loudness_active_table)[LOUDNESS_BIQUAD_COUNT] = NULL;

// Track which buffer is active (0 or 1)
static uint8_t active_buf = 0;

// ----------------------------------------------------------------------------
// ISO 226:2003 Constants for evaluation frequencies
// We only need 2 frequencies: ~50 Hz (low shelf) and ~10 kHz (high shelf)
// Constants from ISO 226:2003 Table 1
// ----------------------------------------------------------------------------

// ISO 226 Table 1 constants for 50 Hz
// f=50: Tf=44.0, αf=0.432, Lu=80.4
#define ISO_50_TF   44.0f
#define ISO_50_AF   0.432f
#define ISO_50_LU   80.4f

// ISO 226 Table 1 constants for 10000 Hz
// f=10000: Tf=13.9, αf=0.301, Lu=17.8
#define ISO_10K_TF  13.9f
#define ISO_10K_AF  0.301f
#define ISO_10K_LU  17.8f

// ----------------------------------------------------------------------------
// ISO 226:2003 SPL calculation (Equations 1-2 from the standard)
//
// Af = 4.47e-3 × (10^(0.025×Ln) - 1.15) + (0.4 × 10^((Tf+Lu)/10 - 9))^αf
// Lp = (10/αf) × log10(Af) - Lu + 94
// ----------------------------------------------------------------------------

static float iso226_spl(float Tf, float af, float Lu, float phon) {
    // Threshold term: (0.4 * 10^((Tf+Lu)/10 - 9))^af
    float B = 0.4f * powf(10.0f, (Tf + Lu) / 10.0f - 9.0f);
    float threshold = powf(B, af);

    // Af = phon-dependent term + threshold
    float Af = 4.47e-3f * (powf(10.0f, 0.025f * phon) - 1.15f) + threshold;

    // Clamp to avoid log of negative/zero
    if (Af < 1e-10f) Af = 1e-10f;

    // Lp = (10/αf) * log10(Af) - Lu + 94
    return (10.0f / af) * log10f(Af) - Lu + 94.0f;
}

// Compute loudness compensation gain at a given frequency for a volume step
// Returns the gain in dB that should be applied
static float loudness_compensation_db(float Tf, float af, float Lu,
                                       float ref_spl, float effective_phon,
                                       float intensity_pct) {
    // At reference level, no compensation
    if (effective_phon >= ref_spl) return 0.0f;

    // SPL at this frequency for the reference phon level
    float spl_ref = iso226_spl(Tf, af, Lu, ref_spl);

    // SPL at this frequency for the effective (reduced) phon level
    float spl_eff = iso226_spl(Tf, af, Lu, effective_phon);

    // The equal-loudness contour difference minus the actual volume change
    // compensation = [L(f, eff_phon) - L(f, ref_phon)] - (eff_phon - ref_phon)
    // This gives how much MORE (or less) the SPL drops at this frequency
    // compared to the flat attenuation at 1 kHz
    float flat_change = effective_phon - ref_spl;  // negative (volume reduced)
    float freq_change = spl_eff - spl_ref;         // how much SPL actually changes at this freq
    float compensation = freq_change - flat_change; // positive = need boost

    // Scale by intensity
    compensation *= (intensity_pct / 100.0f);

    return compensation;
}

// ----------------------------------------------------------------------------
// RBJ Shelf Biquad Coefficient Computation
// Same equations as dsp_compute_coefficients() in dsp_pipeline.c
// ----------------------------------------------------------------------------

static void compute_shelf_coeffs(float freq, float Q, float gain_db,
                                  int is_high_shelf, float sample_rate,
                                  LoudnessCoeffs *out) {
    // Identity if no gain
    if (fabsf(gain_db) < 0.01f) {
#if PICO_RP2350
        out->b0 = 1.0f; out->b1 = 0.0f; out->b2 = 0.0f;
        out->a1 = 0.0f; out->a2 = 0.0f;
#else
        out->b0 = 1 << FILTER_SHIFT; out->b1 = 0; out->b2 = 0;
        out->a1 = 0; out->a2 = 0;
#endif
        return;
    }

    float omega = 2.0f * 3.1415926535f * freq / sample_rate;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2.0f * Q);
    float A = powf(10.0f, gain_db / 40.0f);
    float sqrtA = sqrtf(A);

    float a0_f, a1_f, a2_f, b0_f, b1_f, b2_f;

    if (is_high_shelf) {
        b0_f = A * ((A + 1) + (A - 1) * cs + 2 * sqrtA * alpha);
        b1_f = -2 * A * ((A - 1) + (A + 1) * cs);
        b2_f = A * ((A + 1) + (A - 1) * cs - 2 * sqrtA * alpha);
        a0_f = (A + 1) - (A - 1) * cs + 2 * sqrtA * alpha;
        a1_f = 2 * ((A - 1) - (A + 1) * cs);
        a2_f = (A + 1) - (A - 1) * cs - 2 * sqrtA * alpha;
    } else {
        b0_f = A * ((A + 1) - (A - 1) * cs + 2 * sqrtA * alpha);
        b1_f = 2 * A * ((A - 1) - (A + 1) * cs);
        b2_f = A * ((A + 1) - (A - 1) * cs - 2 * sqrtA * alpha);
        a0_f = (A + 1) + (A - 1) * cs + 2 * sqrtA * alpha;
        a1_f = -2 * ((A - 1) + (A + 1) * cs);
        a2_f = (A + 1) + (A - 1) * cs - 2 * sqrtA * alpha;
    }

#if PICO_RP2350
    float inv_a0 = 1.0f / a0_f;
    out->b0 = b0_f * inv_a0;
    out->b1 = b1_f * inv_a0;
    out->b2 = b2_f * inv_a0;
    out->a1 = a1_f * inv_a0;
    out->a2 = a2_f * inv_a0;
#else
    float scale = (float)(1LL << FILTER_SHIFT);
    out->b0 = (int32_t)((b0_f / a0_f) * scale);
    out->b1 = (int32_t)((b1_f / a0_f) * scale);
    out->b2 = (int32_t)((b2_f / a0_f) * scale);
    out->a1 = (int32_t)((a1_f / a0_f) * scale);
    out->a2 = (int32_t)((a2_f / a0_f) * scale);
#endif
}

// ----------------------------------------------------------------------------
// Table Recomputation (called from main loop, not time-critical)
// ----------------------------------------------------------------------------

void loudness_recompute_table(float ref_spl, float intensity_pct, float sample_rate) {
    if (sample_rate < 1.0f) sample_rate = 48000.0f;

    // Clamp ref_spl to valid range
    if (ref_spl < 40.0f) ref_spl = 40.0f;
    if (ref_spl > 100.0f) ref_spl = 100.0f;

    // Write into the INACTIVE buffer
    uint8_t write_buf = 1 - active_buf;

    // Low shelf: fc=200 Hz, Q=0.707
    // High shelf: fc=6000 Hz, Q=0.707
    static const float shelf_freq[2] = { 200.0f, 6000.0f };
    static const float shelf_Q = 0.707f;

    for (int vol_idx = 0; vol_idx < LOUDNESS_VOL_STEPS; vol_idx++) {
        // Volume in dB: index 0 = -90 dB, index 90 = 0 dB
        float vol_db = (float)(vol_idx - 90);

        // Effective phon = ref_spl + vol_db, clamped to [20, ref_spl]
        float effective_phon = ref_spl + vol_db;
        if (effective_phon < 20.0f) effective_phon = 20.0f;
        if (effective_phon > ref_spl) effective_phon = ref_spl;

        // Low shelf gain from ISO 226 at 50 Hz
        float low_gain_db = loudness_compensation_db(
            ISO_50_TF, ISO_50_AF, ISO_50_LU,
            ref_spl, effective_phon, intensity_pct);

        // High shelf gain from ISO 226 at 10 kHz
        float high_gain_db = loudness_compensation_db(
            ISO_10K_TF, ISO_10K_AF, ISO_10K_LU,
            ref_spl, effective_phon, intensity_pct);

        // Compute low shelf biquad coefficients
        compute_shelf_coeffs(shelf_freq[0], shelf_Q, low_gain_db,
                           0, sample_rate,
                           &loudness_tables[write_buf][vol_idx][0]);

        // Compute high shelf biquad coefficients
        compute_shelf_coeffs(shelf_freq[1], shelf_Q, high_gain_db,
                           1, sample_rate,
                           &loudness_tables[write_buf][vol_idx][1]);
    }

    // Atomic swap: update active table pointer
    active_buf = write_buf;
    loudness_active_table = loudness_tables[active_buf];
}
