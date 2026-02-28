#ifndef DSP_PIPELINE_H
#define DSP_PIPELINE_H

#include "config.h"

// Filter storage
extern Biquad filters[NUM_CHANNELS][MAX_BANDS];
extern EqParamPacket filter_recipes[NUM_CHANNELS][MAX_BANDS];
extern float channel_delays_ms[NUM_CHANNELS];
extern bool channel_bypassed[NUM_CHANNELS];  // true if all bands in channel are flat

// Delay Lines — all 9 output channels on both platforms
// RP2350: float, 170ms max delay (8192 samples)
// RP2040: int32_t, 42ms max delay (2048 samples)
#define NUM_DELAY_CHANNELS NUM_OUTPUT_CHANNELS
#if PICO_RP2350
extern float delay_lines[NUM_DELAY_CHANNELS][MAX_DELAY_SAMPLES];
#else
extern int32_t delay_lines[NUM_DELAY_CHANNELS][MAX_DELAY_SAMPLES];
#endif
extern uint32_t delay_write_idx;
extern int32_t channel_delay_samples[NUM_DELAY_CHANNELS];
extern bool any_delay_active;  // True if any channel has non-zero delay

// API
void dsp_init_default_filters(void);
void dsp_compute_coefficients(EqParamPacket *p, Biquad *bq, float sample_rate);
void dsp_recalculate_all_filters(float sample_rate);
void dsp_update_delay_samples(float sample_rate);

// Optimized processing function
#if PICO_RP2350
float dsp_process_channel(Biquad * __restrict biquads, float input, uint8_t channel);
void dsp_process_channel_block(Biquad * __restrict biquads, float * __restrict samples,
                               uint32_t count, uint8_t channel);
void dsp_process_channel_block_svf(Biquad * __restrict biquads, float * __restrict samples,
                               uint32_t count, uint8_t channel);
void dsp_process_channel_block_single(Biquad * __restrict biquads, float * __restrict samples,
                               uint32_t count, uint8_t channel);
#else
int32_t dsp_process_channel(Biquad * __restrict biquads, int32_t input_32, uint8_t channel);
void dsp_process_channel_block(Biquad * __restrict biquads, int32_t * __restrict samples,
                               uint32_t count, uint8_t channel);
#endif

// Math helper
int32_t fast_mul_q28(int32_t a, int32_t b);

#endif // DSP_PIPELINE_H
