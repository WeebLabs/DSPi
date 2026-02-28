#include <math.h>
#include <string.h>
#include "dsp_pipeline.h"
#include "dcp_inline.h"

static inline bool is_filter_flat(const EqParamPacket *p) {
    if (p->type == FILTER_FLAT) return true;
    if (p->freq <= 0.0f) return true;

    // Peaking/shelf with ~0dB gain is effectively flat
    if (p->type == FILTER_PEAKING ||
        p->type == FILTER_LOWSHELF ||
        p->type == FILTER_HIGHSHELF) {
        if (fabsf(p->gain_db) < 0.01f) return true;
    }
    return false;
}

Biquad filters[NUM_CHANNELS][MAX_BANDS];
EqParamPacket filter_recipes[NUM_CHANNELS][MAX_BANDS];
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

uint8_t channel_band_counts[NUM_CHANNELS] = {
#if PICO_RP2350
    // Master L, Master R, Out1-9 (11 channels total)
    10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10
#else
    // RP2040: 7 channels (2 master + 4 SPDIF + 1 PDM)
    10, 10, 10, 10, 10, 10, 10
#endif
};

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

void dsp_compute_coefficients(EqParamPacket *p, Biquad *bq, float sample_rate) {
    if (is_filter_flat(p) || sample_rate == 0) {
        bq->bypass = true;
#if PICO_RP2350
        bq->b0 = 1.0f; bq->b1 = 0.0f; bq->b2 = 0.0f; bq->a1 = 0.0f; bq->a2 = 0.0f;
        bq->sva1 = 0.0f; bq->sva2 = 0.0f; bq->sva3 = 0.0f; bq->svm0 = 0.0f; bq->svm1 = 0.0f; bq->svm2 = 0.0f;
#else
        bq->b0 = 1 << FILTER_SHIFT; bq->b1 = 0; bq->b2 = 0; bq->a1 = 0; bq->a2 = 0;
#endif
        return;
    }

    bq->bypass = false;

    //biquad coefficients
    float omega = 2.0f * 3.1415926535f * p->freq / sample_rate;
    float sn = sinf(omega); float cs = cosf(omega);
    float alpha = sn / (2.0f * p->Q);
    float A = powf(10.0f, p->gain_db / 40.0f);
    float a0_f = 1.0f, a1_f = 0.0f, a2_f = 0.0f, b0_f = 1.0f, b1_f = 0.0f, b2_f = 0.0f;
    switch (p->type) {
        case FILTER_LOWPASS: b0_f = (1-cs)/2; b1_f = 1-cs; b2_f = (1-cs)/2; a0_f = 1+alpha; a1_f = -2*cs; a2_f = 1-alpha; break;
        case FILTER_HIGHPASS: b0_f = (1+cs)/2; b1_f = -(1+cs); b2_f = (1+cs)/2; a0_f = 1+alpha; a1_f = -2*cs; a2_f = 1-alpha; break;
        case FILTER_PEAKING: b0_f = 1+alpha*A; b1_f = -2*cs; b2_f = 1-alpha*A; a0_f = 1+alpha/A; a1_f = -2*cs; a2_f = 1-alpha/A; break;
        case FILTER_LOWSHELF: b0_f = A*((A+1)-(A-1)*cs+2*sqrtf(A)*alpha); b1_f = 2*A*((A-1)-(A+1)*cs); b2_f = A*((A+1)-(A-1)*cs-2*sqrtf(A)*alpha); a0_f = (A+1)+(A-1)*cs+2*sqrtf(A)*alpha; a1_f = -2*((A-1)+(A+1)*cs); a2_f = (A+1)+(A-1)*cs-2*sqrtf(A)*alpha; break;
        case FILTER_HIGHSHELF: b0_f = A*((A+1)+(A-1)*cs+2*sqrtf(A)*alpha); b1_f = -2*A*((A-1)+(A+1)*cs); b2_f = A*((A+1)+(A-1)*cs-2*sqrtf(A)*alpha); a0_f = (A+1)-(A-1)*cs+2*sqrtf(A)*alpha; a1_f = 2*((A-1)-(A+1)*cs); a2_f = (A+1)-(A-1)*cs-2*sqrtf(A)*alpha; break;
        default: break;
    }

    //state variable filter coefficients
    float svm0_f = 0.0f, svm1_f = 0.0f, svm2_f = 0.0f;
    float sva1_f = 0.0f, sva2_f = 0.0f, sva3_f = 0.0f;

    //A is the value as used in the biquad previously
    float g = tanf((p->freq / sample_rate) * 3.1415926535f);
    float k = (p->type == FILTER_PEAKING) ? 1.0f / (p->Q*A) : 1.0f / p->Q;

    switch (p->type) {
        case FILTER_LOWPASS:   svm0_f = 0.0f; svm1_f =  0.0f;     svm2_f =  1.0f;   sva1_f = 1.0f/(1.0f + g*(g + k)); sva2_f = g * sva1_f; sva3_f = g * sva2_f; break;
        case FILTER_HIGHPASS:  svm0_f = 1.0f; svm1_f = -k;        svm2_f = -1.0f;   sva1_f = 1.0f/(1.0f + g*(g + k)); sva2_f = g * sva1_f; sva3_f = g * sva2_f; break;
        case FILTER_PEAKING:   svm0_f = 1.0f; svm1_f = k*(A*A-1); svm2_f = 0.0f;    sva1_f = 1.0f/(1.0f + g*(g + k)); sva2_f = g * sva1_f; sva3_f = g * sva2_f; break;
        case FILTER_LOWSHELF:  svm0_f = 1.0f; svm1_f = k*(A-1);   svm2_f = A*A - 1; sva1_f = 1.0f/(1.0f + g*(g + k)); sva2_f = g * sva1_f; sva3_f = g * sva2_f; break;
        case FILTER_HIGHSHELF: svm0_f = A*A;  svm1_f = k*(1-A)*A; svm2_f = 1 - A*A; sva1_f = 1.0f/(1.0f + g*(g + k)); sva2_f = g * sva1_f; sva3_f = g * sva2_f; break;
        default: break;
    }

#if PICO_RP2350
    // Float storage for state variable filter coefficients
    bq->sva1 = sva1_f;
    bq->sva2 = sva2_f;
    bq->sva3 = sva3_f;
    bq->svm0 = svm0_f;
    bq->svm1 = svm1_f;
    bq->svm2 = svm2_f;

    // Float storage for biquad coefficients
    float inv_a0 = 1.0f / a0_f;
    bq->b0 = b0_f * inv_a0;
    bq->b1 = b1_f * inv_a0;
    bq->b2 = b2_f * inv_a0;
    bq->a1 = a1_f * inv_a0;
    bq->a2 = a2_f * inv_a0;
#else
    // Q28 Fixed Point Storage
    float scale = (float)(1LL << FILTER_SHIFT);
    bq->b0 = (int32_t)((b0_f / a0_f) * scale);
    bq->b1 = (int32_t)((b1_f / a0_f) * scale);
    bq->b2 = (int32_t)((b2_f / a0_f) * scale);
    bq->a1 = (int32_t)((a1_f / a0_f) * scale);
    bq->a2 = (int32_t)((a2_f / a0_f) * scale);
#endif
}

void dsp_init_default_filters() {
    memset(filters, 0, sizeof(filters));
    memset(channel_delays_ms, 0, sizeof(channel_delays_ms));

    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        channel_bypassed[ch] = true;
        for (int b = 0; b < MAX_BANDS; b++) {
            filters[ch][b].bypass = true;
#if PICO_RP2350
            filters[ch][b].b0 = 1.0f;
#else
            filters[ch][b].b0 = 1 << FILTER_SHIFT;
#endif
            filter_recipes[ch][b].type = FILTER_FLAT;
            filter_recipes[ch][b].freq = 1000.0f;
            filter_recipes[ch][b].Q = 0.707f;
            filter_recipes[ch][b].gain_db = 0.0f;
        }
    }

    // Default: highpass on S/PDIF outputs (main speakers)
    EqParamPacket hp = { .type = FILTER_HIGHPASS, .freq = 80.0f, .Q = 0.707f, .gain_db = 0.0f };
#if PICO_RP2350
    for (int out = CH_OUT_1; out <= CH_OUT_8; out++) {
#else
    for (int out = CH_OUT_1; out <= CH_OUT_4; out++) {
#endif
        filter_recipes[out][0] = hp;
    }

    // Default: lowpass on PDM sub output
    EqParamPacket lp = { .type = FILTER_LOWPASS, .freq = 80.0f, .Q = 0.707f, .gain_db = 0.0f };
    filter_recipes[CH_OUT_SUB][0] = lp;
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

        int32_t samples = (int32_t)(delay_ms * sample_rate / 1000.0f);
#if !PICO_RP2350
        int32_t max_cap = (int32_t)(MAX_DELAY_MS_CAP * sample_rate / 1000.0f);
        if (samples > max_cap) samples = max_cap;
#endif
        if (samples > MAX_DELAY_SAMPLES) samples = MAX_DELAY_SAMPLES;
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
}

#if PICO_RP2350
DSP_TIME_CRITICAL
float dsp_process_channel(Biquad * __restrict biquads, float input, uint8_t channel) {
    float sample = input;
    uint8_t count = channel_band_counts[channel];
    for (int i = 0; i < count; i++) {
        Biquad *bq = &biquads[i];
        if (bq->bypass) continue;

        // Mixed Precision: Float Multiplies, Double Accumulation
        // y[n] = b0*x[n] + s1[n-1]
        float result_f = dcp_dadd_d2f(dcp_f2d(bq->b0 * sample), bq->s1);

        // s1[n] = b1*x[n] - a1*y[n] + s2[n-1]
        float val1 = bq->b1 * sample - bq->a1 * result_f;
        bq->s1 = dcp_dadd(dcp_f2d(val1), bq->s2);

        // s2[n] = b2*x[n] - a2*y[n]
        float val2 = bq->b2 * sample - bq->a2 * result_f;
        bq->s2 = dcp_f2d(val2);

        sample = result_f;
    }
    return sample;
}

DSP_TIME_CRITICAL
void dsp_process_channel_block(Biquad * __restrict biquads, float * __restrict samples,
                               uint32_t count, uint8_t channel) {
    uint8_t num_bands = channel_band_counts[channel];

    // Process each biquad across all samples (coefficients loaded once per filter)
    for (int band = 0; band < num_bands; band++) {
        Biquad *bq = &biquads[band];
        if (bq->bypass) continue;

        // Load coefficients once for all samples
        float b0 = bq->b0;
        float b1 = bq->b1;
        float b2 = bq->b2;
        float a1 = bq->a1;
        float a2 = bq->a2;
        double s1 = bq->s1;
        double s2 = bq->s2;

        // Process all samples with this biquad
        for (uint32_t i = 0; i < count; i++) {
            float sample = samples[i];

            // y[n] = b0*x[n] + s1[n-1]
            float result_f = dcp_dadd_d2f(dcp_f2d(b0 * sample), s1);

            // s1[n] = b1*x[n] - a1*y[n] + s2[n-1]
            float val1 = b1 * sample - a1 * result_f;
            s1 = dcp_dadd(dcp_f2d(val1), s2);

            // s2[n] = b2*x[n] - a2*y[n]
            s2 = dcp_f2d(b2 * sample - a2 * result_f);

            samples[i] = result_f;
        }

        // Store state back
        bq->s1 = s1;
        bq->s2 = s2;
    }
}

// See https://www.cytomic.com/files/dsp/SvfLinearTrapOptimised2.pdf
DSP_TIME_CRITICAL
void dsp_process_channel_block_svf(Biquad * __restrict biquads, float * __restrict samples,
                               uint32_t count, uint8_t channel) {
    uint8_t num_bands = channel_band_counts[channel];

    // vars for all svf filters
    register float two_f = 2.0f;  // compiler not putting this in its own register otherwise
    float v1, v2, v3;

    // Process each filter across all samples (coefficients loaded once per filter)
    for (int band = 0; band < num_bands; band++) {
        Biquad *bq = &biquads[band];
        if (bq->bypass) continue;

        //state variable filter coefficients
        float a1 = bq->sva1;
        float a2 = bq->sva2;
        float a3 = bq->sva3;
        float m0 = bq->svm0;
        float m1 = bq->svm1;
        float m2 = bq->svm2;

        //state variable filter internal state
        float ic1eq = bq->svic1eq;
        float ic2eq = bq->svic2eq;

        float *sample = samples;

        // Process all samples with this state variable filter
        for (uint32_t i = 0; i < count; i++) {
           float s = *sample;
           v3 = s - ic2eq;
           v1 = a1 * ic1eq + a2 * v3;
           v2 = ic2eq + a2 * ic1eq + a3 * v3;
           ic1eq = two_f * v1 - ic1eq;
           ic2eq = two_f * v2 - ic2eq;
           *sample++ = m0 * s + m1 * v1 + m2 * v2;
        }

        // Store state back
        bq->svic1eq = ic1eq;
        bq->svic2eq = ic2eq;
    }
}

DSP_TIME_CRITICAL
void dsp_process_channel_block_single(Biquad * __restrict biquads, float * __restrict samples,
                               uint32_t count, uint8_t channel) {
    uint8_t num_bands = channel_band_counts[channel];

    // Process each biquad across all samples (coefficients loaded once per filter)
    for (int band = 0; band < num_bands; band++) {
        Biquad *bq = &biquads[band];
        if (bq->bypass) continue;

        // Load coefficients once for all samples
        float b0 = bq->b0;
        float b1 = bq->b1;
        float b2 = bq->b2;
        float a1 = bq->a1;
        float a2 = bq->a2;
        float s1 = dcp_d2f(bq->s1);     //TODO - if the single precision filter is
        float s2 = dcp_d2f(bq->s2);     //good enough, change these state vars to floats

        float *sample = samples;
        float result_f, val1;

        // Process all samples with this biquad
        for (uint32_t i = 0; i < count; i++) {
            float sample = samples[i];

            /* y[n] = b0*x[n] + s1[n-1] */
            result_f = b0 * sample + s1;

            /* s1[n] = b1*x[n] - a1*y[n] + s2[n-1] */
            val1 = b1 * sample - a1 * result_f;
            s1 = val1 + s2;

            /* s2[n] = b2*x[n] - a2*y[n] */
            s2 = b2 * sample - a2 * result_f;

            samples[i] = result_f;
        }

        // Store state back
        bq->s1 = dcp_f2d(s1);
        bq->s2 = dcp_f2d(s2);
    }
}

#else
// RP2040: Per-sample implemented in dsp_process_rp2040.S
extern int32_t dsp_process_channel(Biquad * __restrict biquads, int32_t input_32, uint8_t channel);

// RP2040: Block-based biquad implemented in dsp_process_rp2040.S (assembly)
extern void dsp_process_channel_block(Biquad * __restrict biquads, int32_t * __restrict samples,
                                      uint32_t count, uint8_t channel);
#endif
