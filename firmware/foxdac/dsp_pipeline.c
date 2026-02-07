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
float channel_delays_ms[NUM_CHANNELS] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
bool channel_bypassed[NUM_CHANNELS];

// Delay Line State
#if PICO_RP2350
float delay_lines[3][MAX_DELAY_SAMPLES];
#else
int32_t delay_lines[3][MAX_DELAY_SAMPLES];
#endif
uint32_t delay_write_idx = 0;
int32_t channel_delay_samples[3] = {0, 0, 0}; 

uint8_t channel_band_counts[NUM_CHANNELS] = {
#if PICO_RP2350
    10, 10, 10, 10, 10
#else
    10, 10, 2, 2, 2
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
#else
        bq->b0 = 1 << FILTER_SHIFT; bq->b1 = 0; bq->b2 = 0; bq->a1 = 0; bq->a2 = 0;
#endif
        return;
    }

    bq->bypass = false;

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

#if PICO_RP2350
    // Float storage
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
    EqParamPacket hp = { .type = FILTER_HIGHPASS, .freq = 80.0f, .Q = 0.707f, .gain_db = 0.0f };
    filter_recipes[CH_OUT_LEFT][0] = hp; filter_recipes[CH_OUT_RIGHT][0] = hp;
    EqParamPacket lp = { .type = FILTER_LOWPASS, .freq = 80.0f, .Q = 0.707f, .gain_db = 0.0f };
    filter_recipes[CH_OUT_SUB][0] = lp;
}

void dsp_update_delay_samples(float sample_rate) {
    int32_t samples_l = (int32_t)(channel_delays_ms[CH_OUT_LEFT] * sample_rate / 1000.0f);
    if (samples_l > MAX_DELAY_SAMPLES) samples_l = MAX_DELAY_SAMPLES;
    channel_delay_samples[0] = samples_l;

    int32_t samples_r = (int32_t)(channel_delays_ms[CH_OUT_RIGHT] * sample_rate / 1000.0f);
    if (samples_r > MAX_DELAY_SAMPLES) samples_r = MAX_DELAY_SAMPLES;
    channel_delay_samples[1] = samples_r;

    // Sub: add alignment compensation (converts sample difference to ms at current rate)
    float align_ms = (float)SUB_ALIGN_SAMPLES / sample_rate * 1000.0f;
    float sub_total_ms = channel_delays_ms[CH_OUT_SUB] + align_ms;
    int32_t samples_sub = (int32_t)(sub_total_ms * sample_rate / 1000.0f);
    if (samples_sub > MAX_DELAY_SAMPLES) samples_sub = MAX_DELAY_SAMPLES;
    channel_delay_samples[2] = samples_sub;
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
        double result_d = dcp_dadd(dcp_f2d(bq->b0 * sample), bq->s1);
        float result_f = dcp_d2f(result_d);

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
#else
// RP2040: Implemented in dsp_process_rp2040.S for maximum performance
extern int32_t dsp_process_channel(Biquad * __restrict biquads, int32_t input_32, uint8_t channel);
#endif