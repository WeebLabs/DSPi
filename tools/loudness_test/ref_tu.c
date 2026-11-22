// The pre-change table loudness.c (fetched by run.sh), every external symbol prefixed ref_.
#include "loudness.h"
#define loudness_output_state       ref_loudness_output_state
#define loudness_tables             ref_loudness_tables
#define loudness_active_table       ref_loudness_active_table
#define loudness_recompute_table    ref_loudness_recompute_table
#define loudness_recompute_begin    ref_loudness_recompute_begin
#define loudness_recompute_take_done ref_loudness_take_done
#define bg_job_register             ref_bg_job_register
extern LoudnessCoeffs ref_loudness_tables[2][61][LOUDNESS_BIQUAD_COUNT];
extern LoudnessCoeffs (*ref_loudness_active_table)[LOUDNESS_BIQUAD_COUNT];
void ref_loudness_recompute_table(float ref_spl, float intensity_pct, float sample_rate);
void ref_loudness_recompute_begin(float ref_spl, float intensity_pct, float sample_rate);
bool ref_loudness_take_done(void);
#define sva3 g   // the old source stored g*sva2 in sva3; that field is now g
#include REF_SRC
void ref_bg_job_register(BgJob *job) { (void)job; }
