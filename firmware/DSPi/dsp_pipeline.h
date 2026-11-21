#ifndef DSP_PIPELINE_H
#define DSP_PIPELINE_H

#include <string.h>
#include "config.h"

// Filter storage
extern Filter filters[NUM_CHANNELS][MAX_BANDS];
extern EqParamPacket filter_recipes[NUM_CHANNELS][MAX_BANDS];
// Linkwitz Transform target Q per PEQ band (Q*512, 0 = 0.707 default);
// parallel to filter_recipes, read only for FILTER_LINKWITZ_TRANSFORM bands.
extern uint16_t peq_qp_x512[NUM_CHANNELS][MAX_BANDS];
extern float channel_delays_ms[NUM_CHANNELS];
extern bool channel_bypassed[NUM_CHANNELS];  // true if all bands in channel are flat

// Delay Lines — all 9 output channels on both platforms
// RP2350: float, 2048-sample ring (42 ms at 48 kHz)
// RP2040: int32_t, 1024-sample ring (21 ms at 48 kHz)
#define NUM_DELAY_CHANNELS NUM_OUTPUT_CHANNELS
#if PICO_RP2350
extern float delay_lines[NUM_DELAY_CHANNELS][MAX_DELAY_SAMPLES];
#else
extern int32_t delay_lines[NUM_DELAY_CHANNELS][MAX_DELAY_SAMPLES];
#endif
extern uint32_t delay_write_idx;
extern int32_t channel_delay_samples[NUM_DELAY_CHANNELS];
extern bool any_delay_active;  // True if any channel has non-zero delay
extern bool delay_parked[NUM_DELAY_CHANNELS];

#if PICO_RP2350
typedef float delay_sample_t;
#else
typedef int32_t delay_sample_t;
#endif

// One packet of output delay. A disabled output zeroes its line once, then skips:
// zeros written per packet would not do, a later longer delay reads older slots.
static inline __attribute__((always_inline))
void delay_run_output(int out, delay_sample_t *dst, uint32_t widx, uint32_t n, bool enabled) {
    int32_t dly = channel_delay_samples[out];
    if (dly <= 0) return;
    delay_sample_t *dline = delay_lines[out];
    if (!enabled) {
        if (!delay_parked[out]) {
            memset(dline, 0, sizeof(delay_lines[0]));
            delay_parked[out] = true;
        }
        return;
    }
    delay_parked[out] = false;
    for (uint32_t i = 0; i < n; i++) {
        dline[widx] = dst[i];
        dst[i] = dline[(widx - dly) & MAX_DELAY_MASK];
        widx = (widx + 1) & MAX_DELAY_MASK;
    }
}

// API
void dsp_init_default_filters(void);
void dsp_compute_coefficients(EqParamPacket *p, Filter *f, float sample_rate);
void dsp_recalculate_all_filters(float sample_rate);
void dsp_update_delay_samples(float sample_rate);

// Optimized processing function
#if PICO_RP2350
void dsp_process_channel_block(Filter * __restrict filters, float * __restrict samples,
                               uint32_t count, uint8_t channel);
#else
int32_t dsp_process_channel(Filter * __restrict filters, int32_t input_32, uint8_t channel);
void dsp_process_channel_block(Filter * __restrict filters, int32_t * __restrict samples,
                               uint32_t count, uint8_t channel);
#endif

// Math helper
int32_t fast_mul_q28(int32_t a, int32_t b);

#endif // DSP_PIPELINE_H