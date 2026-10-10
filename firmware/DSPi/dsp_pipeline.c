#include <math.h>
#include <string.h>
#include "dsp_pipeline.h"
#include "dsp_svf.h"
#include "dsp_biquad.h"
#include "dsp_cascade.h"
#include "dcp_inline.h"
#include "crossover.h"

static inline bool is_filter_flat(const EqParamPacket *p) {
    if (p->type == FILTER_FLAT) return true;
    // Any type outside the PEQ block (see filter_is_peq_type / the FilterType
    // value-space contract in config.h) is not a valid PEQ filter — bypass the
    // band rather than producing garbage. The most important case this defends
    // against is a crossover filter type (FILTER_XOVER_FIRST..FILTER_XOVER_LAST)
    // ending up in a PEQ band slot via a host bug or a mis-routed
    // REQ_SET_EQ_PARAM / bulk apply / preset load: on the RP2350 SVF path the
    // unknown type leaves the output mix coefficients zeroed (svm0=svm1=svm2=0),
    // and the band would output silence at low fc. Treat as bypass instead,
    // recipe round-trips. (An undefined value in the reserved PEQ padding falls
    // through to a unity passthrough via the coefficient switch default.)
    if (!filter_is_peq_type(p->type)) return true;
    if (p->freq <= 0.0f) return true;

    // Linkwitz Transform carries the target frequency fp in gain_db (Hz);
    // a non-positive fp is unconfigured/invalid, treat as flat.
    if (p->type == FILTER_LINKWITZ_TRANSFORM && p->gain_db <= 0.0f) return true;

    // Peaking/shelf with ~0dB gain is effectively flat
    if (p->type == FILTER_PEAKING ||
        p->type == FILTER_LOWSHELF ||
        p->type == FILTER_HIGHSHELF ||
        p->type == FILTER_LOWSHELF1 ||
        p->type == FILTER_HIGHSHELF1) {
        if (fabsf(p->gain_db) < 0.01f) return true;
    }
    return false;
}

Filter filters[NUM_CHANNELS][MAX_BANDS];
EqParamPacket filter_recipes[NUM_CHANNELS][MAX_BANDS];

// Linkwitz Transform target Q per PEQ band, fixed-point Q*512 (0 = 0.707
// default).  Parallel to filter_recipes because EqParamPacket has no spare
// field; only read when the band's type is FILTER_LINKWITZ_TRANSFORM.
uint16_t peq_qp_x512[NUM_CHANNELS][MAX_BANDS];
float channel_delays_ms[NUM_CHANNELS] = {0};  // All 11 channels initialized to 0
bool channel_bypassed[NUM_CHANNELS];

// Delay Line State (all output channels on both platforms)
// RP2350: float, 170ms max delay (8192 samples), 9 channels
// RP2040: int32_t, 50ms software cap (4096 samples hardware), 5 channels
#if PICO_RP2350
float delay_lines[NUM_DELAY_CHANNELS][MAX_DELAY_SAMPLES];
#else
int32_t delay_lines[NUM_DELAY_CHANNELS][MAX_DELAY_SAMPLES];
#endif
uint32_t delay_write_idx = 0;
int32_t channel_delay_samples[NUM_DELAY_CHANNELS] = {0};
bool any_delay_active = false;

// Every channel (inputs + outputs) has 10 active PEQ bands.  Range-designator
// init keeps this correct as NUM_CHANNELS changes (17 on RP2350, 7 on RP2040).
uint8_t channel_band_counts[NUM_CHANNELS] = {[0 ... NUM_CHANNELS-1] = 10};

#if !PICO_RP2350
DSP_TIME_CRITICAL int32_t fast_mul_q28(int32_t a, int32_t b) {
    int32_t ah = a >> 16;
    uint32_t al = a & 0xFFFF;
    int32_t bh = b >> 16;
    uint32_t bl = b & 0xFFFF;

    int32_t high = ah * bh;
    int32_t mid1 = ah * bl;
    int32_t mid2 = al * bh;

    return (high << 4) + ((mid1 + mid2) >> 12);
}
#endif

void dsp_compute_coefficients(EqParamPacket *p, Filter *f, float sample_rate) {
    // 0xFF-safe interpretation: bypass byte must be exactly 1 to bypass.
    // Any other value (0, 0xFF padding from legacy hosts, garbage) leaves
    // the band active.  See Documentation/Features/band_bypass_spec.md.
    bool user_bypass = (p->bypass == 1);

    if (user_bypass || is_filter_flat(p) || sample_rate == 0) {
        f->bypass = true;
#if PICO_RP2350
        f->b0 = 1.0f; f->b1 = 0.0f; f->b2 = 0.0f; f->a1 = 0.0f; f->a2 = 0.0f;
        f->sva1 = 0.0f; f->sva2 = 0.0f; f->sva3 = 0.0f;
        f->svm0 = 0.0f; f->svm1 = 0.0f; f->svm2 = 0.0f;
        f->use_svf = false;
        f->first_order = false;
#else
        f->b0 = 1 << FILTER_SHIFT; f->b1 = 0; f->b2 = 0; f->a1 = 0; f->a2 = 0;
#endif
        return;
    }

    f->bypass = false;

    // Input validation
    if (p->Q < 0.1f) p->Q = 0.1f;
    if (p->Q > 20.0f) p->Q = 20.0f;
    if (p->freq < 10.0f) p->freq = 10.0f;
    if (p->freq > sample_rate * 0.45f) p->freq = sample_rate * 0.45f;

    // Linkwitz Transform: zeros at the driver's measured rolloff (freq = f0,
    // Q = Q0), poles at the target alignment.  fp rides in gain_db (Hz, gain
    // is unused by this type), Qp in peq_qp_x512[] as Q*512 (0 = 0.707).
    // Both corners clamp to 0.15*Fs: that bounds every normalized biquad
    // coefficient below 6.4, inside the RP2040 Q28 range of +/-8, and is far
    // above any physical use of the filter.  Clamps stay local (no recipe
    // write-back) so the stored fp round-trips to the host unmodified.
    const bool is_lt = (p->type == FILTER_LINKWITZ_TRANSFORM);
    float lt_fp = 0.0f, lt_qp = 0.707f;
    if (is_lt) {
        if (p->freq > sample_rate * 0.15f) p->freq = sample_rate * 0.15f;
        lt_fp = p->gain_db;
        if (lt_fp < 10.0f) lt_fp = 10.0f;
        if (lt_fp > sample_rate * 0.15f) lt_fp = sample_rate * 0.15f;
        uint16_t qpx = (p->channel < NUM_CHANNELS && p->band < MAX_BANDS)
                           ? peq_qp_x512[p->channel][p->band] : 0;
        if (qpx != 0) {
            lt_qp = (float)qpx * (1.0f / 512.0f);
            if (lt_qp < 0.1f) lt_qp = 0.1f;
            if (lt_qp > 20.0f) lt_qp = 20.0f;
        }
    }

    // gain_db holds fp for LT; skip the dB conversion (it could produce inf).
    float A = is_lt ? 1.0f : powf(10.0f, p->gain_db / 40.0f);

#if PICO_RP2350
    // SVF/biquad crossover decision + state reset on path change
    bool was_svf = f->use_svf;
    bool was_first_order = f->first_order;
    f->use_svf = (p->freq < (sample_rate / 7.5f));
    f->filter_type = p->type;
    // LT has two corner frequencies; both must sit below the SVF threshold,
    // otherwise fall back to the exact-bilinear biquad.
    if (is_lt && lt_fp >= (sample_rate / 7.5f)) f->use_svf = false;
    // First-order types (all-pass, low/high shelf) are genuine 1st-order
    // sections.  They can't use the 2nd-order SVF, but they DO follow the same
    // hybrid rule: a one-pole SVF below Fs/7.5 (svf_first_order) and a
    // degenerate TDF2 biquad above.
    const bool is_first_order = (p->type == FILTER_LOWPASS1 ||
                                 p->type == FILTER_HIGHPASS1 ||
                                 p->type == FILTER_ALLPASS1 ||
                                 p->type == FILTER_LOWSHELF1 ||
                                 p->type == FILTER_HIGHSHELF1);
    f->first_order = is_first_order;
    if (was_svf != f->use_svf || was_first_order != f->first_order) {
        f->s1 = 0.0f; f->s2 = 0.0f;
        f->svic1eq = 0.0f; f->svic2eq = 0.0f;
    }

    if (f->use_svf) {
        if (f->first_order) {
            // One-pole TPT SVF (1st-order types).  The 1/(1+g) reciprocal is
            // folded into sva1 so the inner loop is multiply-only; svic2eq is
            // unused (kept 0).  lp = v1, hp = in - v1.
            float g = tanf(3.1415926535f * p->freq / sample_rate);
            float svm0_f = 0.0f, svm1_f = 0.0f, svm2_f = 0.0f;
            switch (p->type) {
                case FILTER_HIGHPASS1:  // out = in - v1
                    svm0_f = 0.0f; svm1_f = 0.0f; svm2_f = 1.0f; break;
                case FILTER_LOWPASS1:   // out = v1
                    svm0_f = 0.0f; svm1_f = 1.0f; svm2_f = 0.0f; break;
                case FILTER_ALLPASS1:   // out = 2*lp - in
                    svm0_f = 0.0f; svm1_f = 1.0f; svm2_f = -1.0f; break;
                case FILTER_LOWSHELF1:  // 1st-order shelf prewarps by A, not sqrt(A)
                    g = g / A;
                    svm0_f = 1.0f; svm1_f = A * A - 1.0f; svm2_f = 0.0f; break;
                case FILTER_HIGHSHELF1:
                    g = g * A;
                    svm0_f = 1.0f; svm1_f = 0.0f; svm2_f = A * A - 1.0f; break;
                default: break;
            }
            float sva1_f = 1.0f / (1.0f + g);
            float sva2_f = g * sva1_f;
            f->sva1 = sva1_f; f->sva2 = sva2_f; f->sva3 = 0.0f;
            f->svm0 = svm0_f; f->svm1 = svm1_f; f->svm2 = svm2_f;
            f->g = g;
            // Fallback biquad coeffs (unused in SVF path, keep sane)
            f->b0 = 1.0f; f->b1 = 0.0f; f->b2 = 0.0f; f->a1 = 0.0f; f->a2 = 0.0f;
            return;
        }

        // SVF coefficients (Simper, "SvfLinearTrapAllOutputs", Cytomic 2021)
        // Shelf k = 1/Q matches RBJ Audio-EQ-Cookbook response exactly.
        float g = tanf(3.1415926535f * p->freq / sample_rate);
        float k = 1.0f / p->Q;

        switch (p->type) {
            case FILTER_PEAKING:
                k = 1.0f / (p->Q * A);
                break;
            case FILTER_LOWSHELF: {
                float sqrtA = sqrtf(A);
                g = g / sqrtA;
                break;
            }
            case FILTER_HIGHSHELF: {
                float sqrtA = sqrtf(A);
                g = g * sqrtA;
                break;
            }
            case FILTER_LINKWITZ_TRANSFORM:
                // SVF is tuned at the pole pair (target alignment).
                g = tanf(3.1415926535f * lt_fp / sample_rate);
                k = 1.0f / lt_qp;
                break;
            default: break;
        }

        float sva1_f = 1.0f / (1.0f + g * (g + k));
        float sva2_f = g * sva1_f;
        float sva3_f = g * sva2_f;

        float svm0_f = 0.0f, svm1_f = 0.0f, svm2_f = 0.0f;
        switch (p->type) {
            case FILTER_LOWPASS:   svm0_f = 0.0f; svm1_f = 0.0f;            svm2_f = 1.0f;       break;
            case FILTER_HIGHPASS:  svm0_f = 1.0f; svm1_f = -k;              svm2_f = -1.0f;      break;
            case FILTER_PEAKING:   svm0_f = 1.0f; svm1_f = k*(A*A - 1.0f);  svm2_f = 0.0f;       break;
            case FILTER_LOWSHELF:  svm0_f = 1.0f; svm1_f = k*(A - 1.0f);    svm2_f = A*A - 1.0f; break;
            case FILTER_HIGHSHELF: svm0_f = A*A;  svm1_f = k*(1.0f-A)*A;    svm2_f = 1.0f - A*A; break;
            case FILTER_NOTCH:     svm0_f = 1.0f; svm1_f = -1*k;            svm2_f = 0.0f;       break;
            case FILTER_ALLPASS:   svm0_f = 1.0f; svm1_f = -2.0*k;          svm2_f = 0.0f;       break;
            case FILTER_LINKWITZ_TRANSFORM: {
                // H = hp + (g0/(Q0*gp))*bp + (g0/gp)^2*lp with hp = in - k*v1 - v2,
                // bp = v1, lp = v2; zeros prewarped at f0, poles at fp.
                float g0 = tanf(3.1415926535f * p->freq / sample_rate);
                float r = g0 / g;
                svm0_f = 1.0f;
                svm1_f = r / p->Q - k;
                svm2_f = r * r - 1.0f;
                break;
            }
            default: break;
        }

        f->sva1 = sva1_f; f->sva2 = sva2_f; f->sva3 = sva3_f;
        f->svm0 = svm0_f; f->svm1 = svm1_f; f->svm2 = svm2_f;
        f->g = g;

        // Also compute biquad coefficients as fallback (not used in SVF path)
        f->b0 = 1.0f; f->b1 = 0.0f; f->b2 = 0.0f; f->a1 = 0.0f; f->a2 = 0.0f;
        return;
    }

    // Clear SVF coefficients for biquad path
    f->sva1 = 0.0f; f->sva2 = 0.0f; f->sva3 = 0.0f;
    f->svm0 = 0.0f; f->svm1 = 0.0f; f->svm2 = 0.0f;
#endif

    float omega = 2.0f * 3.1415926535f * p->freq / sample_rate;
    float sn = sinf(omega); float cs = cosf(omega);
    float alpha = sn / (2.0f * p->Q);
    float a0_f = 1.0f, a1_f = 0.0f, a2_f = 0.0f, b0_f = 1.0f, b1_f = 0.0f, b2_f = 0.0f;
    switch (p->type) {
        case FILTER_LOWPASS: b0_f = (1-cs)/2; b1_f = 1-cs; b2_f = (1-cs)/2; a0_f = 1+alpha; a1_f = -2*cs; a2_f = 1-alpha; break;
        case FILTER_HIGHPASS: b0_f = (1+cs)/2; b1_f = -(1+cs); b2_f = (1+cs)/2; a0_f = 1+alpha; a1_f = -2*cs; a2_f = 1-alpha; break;
        case FILTER_PEAKING: b0_f = 1+alpha*A; b1_f = -2*cs; b2_f = 1-alpha*A; a0_f = 1+alpha/A; a1_f = -2*cs; a2_f = 1-alpha/A; break;
        case FILTER_LOWSHELF: b0_f = A*((A+1)-(A-1)*cs+2*sqrtf(A)*alpha); b1_f = 2*A*((A-1)-(A+1)*cs); b2_f = A*((A+1)-(A-1)*cs-2*sqrtf(A)*alpha); a0_f = (A+1)+(A-1)*cs+2*sqrtf(A)*alpha; a1_f = -2*((A-1)+(A+1)*cs); a2_f = (A+1)+(A-1)*cs-2*sqrtf(A)*alpha; break;
        case FILTER_HIGHSHELF: b0_f = A*((A+1)+(A-1)*cs+2*sqrtf(A)*alpha); b1_f = -2*A*((A-1)+(A+1)*cs); b2_f = A*((A+1)+(A-1)*cs-2*sqrtf(A)*alpha); a0_f = (A+1)-(A-1)*cs+2*sqrtf(A)*alpha; a1_f = 2*((A-1)-(A+1)*cs); a2_f = (A+1)-(A-1)*cs-2*sqrtf(A)*alpha; break;
        case FILTER_NOTCH: b0_f = 1.0f; b1_f = -2*cs; b2_f = 1.0f; a0_f = 1+alpha; a1_f=-2*cs; a2_f=1-alpha; break;
        case FILTER_ALLPASS: b0_f = 1-alpha; b1_f = -2*cs; b2_f = 1+alpha; a0_f = 1 + alpha; a1_f = -2*cs; a2_f = 1 - alpha; break;
        case FILTER_ALLPASS1: {
            // First-order all-pass. H(z) = (a + z^-1)/(1 + a*z^-1),
            // a = (tan(pi*fc/Fs) - 1)/(tan(pi*fc/Fs) + 1). Flat magnitude,
            // phase 0 → -180° (-90° at fc). Degenerate biquad (b2 = a2 = 0);
            // runs unchanged on RP2040 (Q28) and RP2350 (float, TDF2 forced above).
            float ta = tanf(3.1415926535f * p->freq / sample_rate);
            float ap = (ta - 1.0f) / (ta + 1.0f);
            b0_f = ap;   b1_f = 1.0f; b2_f = 0.0f;
            a0_f = 1.0f; a1_f = ap;   a2_f = 0.0f;
            break;
        }
        case FILTER_LOWSHELF1: {
            // First-order low shelf (degenerate biquad, b2 = a2 = 0): DC gain
            // A^2, unity at Nyquist.  RBJ-equivalent 1st-order form.
            b0_f = (A * sn) + 1.0f + cs; b1_f = (A * sn) - 1.0f - cs; b2_f = 0.0f;
            a0_f = (sn / A) + 1.0f + cs; a1_f = (sn / A) - 1.0f - cs; a2_f = 0.0f;
            break;
        }
        case FILTER_HIGHSHELF1: {
            // First-order high shelf: unity at DC, gain A^2 at Nyquist.
            b0_f = sn + A + (A * cs); b1_f = sn - A - (A * cs); b2_f = 0.0f;
            a0_f = sn + (1.0f / A) + (cs / A); a1_f = sn - (1.0f / A) - (cs / A); a2_f = 0.0f;
            break;
        }
        case FILTER_LINKWITZ_TRANSFORM: {
            // Bilinear transform, each corner prewarped independently:
            // zeros from (f0, Q0) = freq/Q, poles from (fp, Qp).
            // DC gain = (g0/gp)^2, unity toward Nyquist.
            float g0 = tanf(3.1415926535f * p->freq / sample_rate);
            float gp = tanf(3.1415926535f * lt_fp / sample_rate);
            b0_f = 1.0f + g0 / p->Q + g0 * g0;
            b1_f = 2.0f * (g0 * g0 - 1.0f);
            b2_f = 1.0f - g0 / p->Q + g0 * g0;
            a0_f = 1.0f + gp / lt_qp + gp * gp;
            a1_f = 2.0f * (gp * gp - 1.0f);
            a2_f = 1.0f - gp / lt_qp + gp * gp;
            break;
        }
        case FILTER_LOWPASS1: {
            b0_f = sn; b1_f = sn; b2_f = 0.0f;
            a0_f = sn + 1.0f + cs; a1_f = sn - 1.0f - cs; a2_f = 0.0f;
            break;
        }
        case FILTER_HIGHPASS1: {
            b0_f = 1.0f + cs; b1_f = -1.0f - cs; b2_f = 0.0f;
            a0_f = sn + 1.0f + cs; a1_f = sn - 1.0f - cs; a2_f = 0.0f;
            break;
        }
        default: break;
    }

#if PICO_RP2350
    // Float storage
    float inv_a0 = 1.0f / a0_f;
    f->b0 = b0_f * inv_a0;
    f->b1 = b1_f * inv_a0;
    f->b2 = b2_f * inv_a0;
    f->a1 = a1_f * inv_a0;
    f->a2 = a2_f * inv_a0;
#else
    // Q28 Fixed Point Storage
    float scale = (float)(1LL << FILTER_SHIFT);
    f->b0 = (int32_t)((b0_f / a0_f) * scale);
    f->b1 = (int32_t)((b1_f / a0_f) * scale);
    f->b2 = (int32_t)((b2_f / a0_f) * scale);
    f->a1 = (int32_t)((a1_f / a0_f) * scale);
    f->a2 = (int32_t)((a2_f / a0_f) * scale);
#endif
}

void dsp_init_default_filters() {
    memset(filters, 0, sizeof(filters));
    memset(channel_delays_ms, 0, sizeof(channel_delays_ms));
    memset(peq_qp_x512, 0, sizeof(peq_qp_x512));

    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        channel_bypassed[ch] = true;
        for (int b = 0; b < MAX_BANDS; b++) {
            filters[ch][b].bypass = true;
#if PICO_RP2350
            filters[ch][b].b0 = 1.0f;
            filters[ch][b].use_svf = false;
            filters[ch][b].filter_type = FILTER_FLAT;
            filters[ch][b].svic1eq = 0.0f;
            filters[ch][b].svic2eq = 0.0f;
#else
            filters[ch][b].b0 = 1 << FILTER_SHIFT;
#endif
            filter_recipes[ch][b].channel = ch;
            filter_recipes[ch][b].band = b;
            filter_recipes[ch][b].type = FILTER_FLAT;
            filter_recipes[ch][b].freq = 1000.0f;
            filter_recipes[ch][b].Q = 0.707f;
            filter_recipes[ch][b].gain_db = 0.0f;
        }
    }

    // Crossover bands default-init.  Writes wire-band-index (XOVER_BAND_BASE + i)
    // into each recipe's `band` field; see crossover_filters_spec.md.
    xover_init_default_filters();
}

void dsp_update_delay_samples(float sample_rate) {
    // Update delay samples for all 9 output channels
    // Delay values come from the matrix mixer OutputChannel.delay_ms
    // This function is called when sample rate changes or delays are updated

    any_delay_active = false;
    for (int out = 0; out < NUM_DELAY_CHANNELS; out++) {
        // Get delay_ms from the corresponding EQ channel (CH_OUT_1 + out)
        float delay_ms = channel_delays_ms[CH_OUT_1 + out];

        // PDM sub needs alignment compensation (last delay channel)
        if (out == NUM_DELAY_CHANNELS - 1) {
            float align_ms = (float)SUB_ALIGN_SAMPLES / sample_rate * 1000.0f;
            delay_ms += align_ms;
        }

        // Cap at MAX-1: a delay of exactly MAX_DELAY_SAMPLES reads the slot just
        // written, i.e. zero delay.  Sub hits the cap 128 samples early (see doc).
        int32_t samples = (int32_t)(delay_ms * sample_rate / 1000.0f);
        if (samples > MAX_DELAY_SAMPLES - 1) samples = MAX_DELAY_SAMPLES - 1;
        if (samples < 0) samples = 0;
        channel_delay_samples[out] = samples;

        if (samples > 0) any_delay_active = true;
    }
}

void dsp_recalculate_all_filters(float sample_rate) {
    dsp_update_delay_samples(sample_rate);
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        bool all_bypassed = true;
        for (int b = 0; b < channel_band_counts[ch]; b++) {
            dsp_compute_coefficients(&filter_recipes[ch][b], &filters[ch][b], sample_rate);
            if (!filters[ch][b].bypass) {
                all_bypassed = false;
            }
        }
        channel_bypassed[ch] = all_bypassed;
    }
    // Rebuild all crossover sections at the new Fs.  Section state is reset
    // on SVF/biquad path change inside xover_design_filter(), same convention
    // as PEQ above.
    xover_recalculate_all(sample_rate);
}

#if PICO_RP2350
DSP_TIME_CRITICAL
float dsp_process_channel(Filter * __restrict filters, float input, uint8_t channel) {
    float sample = input;
    uint8_t count = channel_band_counts[channel];
    for (int i = 0; i < count; i++) {
        Filter *f = &filters[i];
        if (f->bypass) continue;

        if (f->use_svf) {
            if (f->first_order) {
                // One-pole TPT SVF (1st-order). lp = v1, hp = sample - v1.
                float v1 = f->sva2 * sample + f->sva1 * f->svic1eq;
                f->svic1eq = 2.0f * v1 - f->svic1eq;
                sample = f->svm0 * sample + f->svm1 * v1 + f->svm2 * (sample - v1);
            } else {
                float v3 = sample - f->svic2eq;
                float v1 = f->sva1 * f->svic1eq + f->sva2 * v3;
                float v2 = f->svic2eq + f->sva2 * f->svic1eq + f->sva3 * v3;
                f->svic1eq = 2.0f * v1 - f->svic1eq;
                f->svic2eq = 2.0f * v2 - f->svic2eq;
                sample = f->svm0 * sample + f->svm1 * v1 + f->svm2 * v2;
            }
        } else {
            float out = f->b0 * sample + f->s1;
            f->s1 = f->b1 * sample - f->a1 * out + f->s2;
            f->s2 = f->b2 * sample - f->a2 * out;
            sample = out;
        }
    }
    return sample;
}

// A/B knob: 1 fuses shelf/Linkwitz SVF pairs (the generic arm), 0 leaves them
// as singles.  Both settings are bit-identical; this only trades RAM text for
// speed on the generic arm.
#define DSPI_FUSE_GENERIC_SVF_PAIRS 1

DSP_TIME_CRITICAL
void dsp_cascade_block(Filter * __restrict sections, uint32_t num_sections,
                       float * __restrict samples, uint32_t count) {
    uint32_t i = 0;

    while (i < num_sections) {
        Filter *f = &sections[i];
        if (f->bypass) { i++; continue; }

        // Bypassed sections are skipped without breaking the pair around them;
        // section order is preserved exactly.
        uint32_t j = i + 1;
        while (j < num_sections && sections[j].bypass) j++;

        // Second-order sections fuse only with a partner on the same path AND
        // in the same kernel arm, so each keeps its specialized op count and
        // still saves a load/store per sample.  A cross-arm pair would force
        // the generic mix on a specialized section and cost more than it saves.
        if (!f->first_order && j < num_sections) {
            Filter *fn = &sections[j];
            if (!fn->first_order && fn->use_svf == f->use_svf) {
                if (!f->use_svf) {
                    dsp_biquad_second_order_x2(f, fn, samples, count);
                    i = j + 1;
                    continue;
                }
                DspSvfArm arm = dsp_svf_arm_class(f);
                if (arm == dsp_svf_arm_class(fn) &&
                    (DSPI_FUSE_GENERIC_SVF_PAIRS || arm != DSP_SVF_ARM_GENERIC)) {
                    dsp_svf_second_order_x2(f, fn, samples, count);
                    i = j + 1;
                    continue;
                }
            }
        }

        if (f->use_svf) {
            if (f->first_order)
                dsp_svf_first_order(f, samples, count);
            else
                dsp_svf_second_order(f, samples, count);
        } else {
            if (f->first_order)
                dsp_biquad_first_order(f, samples, count);
            else
                dsp_biquad_second_order(f, samples, count);
        }
        i++;
    }
}

DSP_TIME_CRITICAL
void dsp_process_channel_block(Filter * __restrict filters, float * __restrict samples,
                               uint32_t count, uint8_t channel) {
    dsp_cascade_block(filters, channel_band_counts[channel], samples, count);
}
#else
// RP2040: Per-sample implemented in dsp_process_rp2040.S
extern int32_t dsp_process_channel(Filter * __restrict biquads, int32_t input_32, uint8_t channel);

// RP2040: Block-based biquad implemented in dsp_process_rp2040.S (assembly)
extern void dsp_process_channel_block(Filter * __restrict biquads, int32_t * __restrict samples,
                                      uint32_t count, uint8_t channel);
#endif
