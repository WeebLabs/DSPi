// Compiles the real dsp_pipeline.c (and through it dsp_svf.h / dsp_biquad.h)
// for the host, with every external symbol prefixed by KPFX. Built by run.sh.
#include "config.h"
#undef DSP_TIME_CRITICAL
#define DSP_TIME_CRITICAL

#define PFX2(p, n) p##_##n
#define PFX1(p, n) PFX2(p, n)
#define K(n) PFX1(KPFX, n)

#define filters                     K(filters)
#define filter_recipes              K(filter_recipes)
#define peq_qp_x512                 K(peq_qp_x512)
#define channel_delays_ms           K(channel_delays_ms)
#define channel_bypassed            K(channel_bypassed)
#define delay_lines                 K(delay_lines)
#define delay_write_idx             K(delay_write_idx)
#define channel_delay_samples       K(channel_delay_samples)
#define any_delay_active            K(any_delay_active)
#define channel_band_counts         K(channel_band_counts)
#define dsp_compute_coefficients    K(dsp_compute_coefficients)
#define dsp_init_default_filters    K(dsp_init_default_filters)
#define dsp_update_delay_samples    K(dsp_update_delay_samples)
#define dsp_recalculate_all_filters K(dsp_recalculate_all_filters)
#define dsp_process_channel_block   K(dsp_process_channel_block)
#define dsp_cascade_block           K(dsp_cascade_block)

#include "dsp_pipeline.c"
#include "dsp_pair_runs.c"   // out-of-line pair runs dsp_cascade_block calls

void K(svf1)(Filter *f, float *s, uint32_t n)             { dsp_svf_first_order(f, s, n); }
void K(svf2)(Filter *f, float *s, uint32_t n)             { dsp_svf_second_order(f, s, n); }
void K(svf2x2)(Filter *a, Filter *b, float *s, uint32_t n) { dsp_svf_second_order_x2(a, b, s, n); }
void K(bq1)(Filter *f, float *s, uint32_t n)              { dsp_biquad_first_order(f, s, n); }
void K(bq2)(Filter *f, float *s, uint32_t n)              { dsp_biquad_second_order(f, s, n); }
void K(bq2x2)(Filter *a, Filter *b, float *s, uint32_t n)  { dsp_biquad_second_order_x2(a, b, s, n); }
