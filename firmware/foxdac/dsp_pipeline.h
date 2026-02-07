#ifndef DSP_PIPELINE_H
#define DSP_PIPELINE_H

#include "config.h"

// Filter storage
extern Biquad filters[NUM_CHANNELS][MAX_BANDS];
extern EqParamPacket filter_recipes[NUM_CHANNELS][MAX_BANDS];
extern float channel_delays_ms[NUM_CHANNELS];
extern bool channel_bypassed[NUM_CHANNELS];  // true if all bands in channel are flat

// Delay Lines
#if PICO_RP2350
extern float delay_lines[3][MAX_DELAY_SAMPLES];
#else
extern int32_t delay_lines[3][MAX_DELAY_SAMPLES];
#endif
extern uint32_t delay_write_idx;
extern int32_t channel_delay_samples[3];

// API
void dsp_init_default_filters(void);
void dsp_compute_coefficients(EqParamPacket *p, Biquad *bq, float sample_rate);
void dsp_recalculate_all_filters(float sample_rate);
void dsp_update_delay_samples(float sample_rate);

// Optimized processing function
#if PICO_RP2350
float dsp_process_channel(Biquad * __restrict biquads, float input, uint8_t channel);
#else
int32_t dsp_process_channel(Biquad * __restrict biquads, int32_t input_32, uint8_t channel);
#endif

// Math helper
int32_t fast_mul_q28(int32_t a, int32_t b);

#endif // DSP_PIPELINE_H