#include <math.h>
#include <string.h>
#include "dsp_pipeline.h"

Biquad filters[NUM_CHANNELS][MAX_BANDS];
EqParamPacket filter_recipes[NUM_CHANNELS][MAX_BANDS];
float channel_delays_ms[NUM_CHANNELS] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// Delay Line State
int32_t delay_lines[3][MAX_DELAY_SAMPLES];
uint32_t delay_write_idx = 0;
int32_t channel_delay_samples[3] = {0, 0, 0}; 

const uint8_t channel_band_counts[NUM_CHANNELS] = {
    10, 10, 2, 2, 2
};

int32_t fast_mul_q28(int32_t a, int32_t b) {
    int32_t ah = a >> 16;
    uint32_t al = a & 0xFFFF;
    int32_t bh = b >> 16;
    uint32_t bl = b & 0xFFFF;

    int32_t high = ah * bh; 
    int32_t mid1 = ah * bl; 
    int32_t mid2 = al * bh; 
    
    return (high << 4) + ((mid1 + mid2) >> 12);
}

void dsp_compute_coefficients(EqParamPacket *p, Biquad *bq, float sample_rate) {
    bq->s1 = 0; bq->s2 = 0; 
    if (p->type == FILTER_FLAT || p->freq == 0 || sample_rate == 0) {
        bq->b0 = 1 << FILTER_SHIFT; bq->b1 = 0; bq->b2 = 0; bq->a1 = 0; bq->a2 = 0;
        return;
    }
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
    float scale = (float)(1LL << FILTER_SHIFT);
    bq->b0 = (int32_t)((b0_f / a0_f) * scale);
    bq->b1 = (int32_t)((b1_f / a0_f) * scale);
    bq->b2 = (int32_t)((b2_f / a0_f) * scale);
    bq->a1 = (int32_t)((a1_f / a0_f) * scale);
    bq->a2 = (int32_t)((a2_f / a0_f) * scale);
}

void dsp_init_default_filters() {
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        for (int b = 0; b < MAX_BANDS; b++) {
             filter_recipes[ch][b].type = FILTER_FLAT; filter_recipes[ch][b].freq = 1000.0f; filter_recipes[ch][b].Q = 0.707f; filter_recipes[ch][b].gain_db = 0.0f;
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
        for (int b = 0; b < channel_band_counts[ch]; b++) {
            dsp_compute_coefficients(&filter_recipes[ch][b], &filters[ch][b], sample_rate);
        }
    }
}

int32_t dsp_process_channel(Biquad * __restrict biquads, int32_t input_32, uint8_t channel) {
    int32_t sample = input_32;
    uint8_t count = channel_band_counts[channel];
    for (int i = 0; i < count; i++) {
        Biquad *bq = &biquads[i];
        if (bq->a1 == 0 && bq->a2 == 0) continue;

        int32_t result = fast_mul_q28(bq->b0, sample) + bq->s1;
        bq->s1 = fast_mul_q28(bq->b1, sample) - fast_mul_q28(bq->a1, result) + bq->s2;
        bq->s2 = fast_mul_q28(bq->b2, sample) - fast_mul_q28(bq->a2, result);
        
        sample = clip_s32(result);
    }
    return sample;
}