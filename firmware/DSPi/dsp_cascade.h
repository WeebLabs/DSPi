#ifndef DSP_CASCADE_H
#define DSP_CASCADE_H

#include "config.h"

#if PICO_RP2350
// Runs a filter cascade over a sample block, fusing each pair of adjacent
// active second-order sections on the same path into one buffer sweep.
// Out-of-line and shared by the PEQ and crossover kernels so the unrolled
// bodies are emitted into RAM once.  Defined in dsp_pipeline.c; see
// Documentation/current_architecture.md.
void dsp_cascade_block(Filter * __restrict sections, uint32_t num_sections,
                       float * __restrict samples, uint32_t count);

// Run every consecutive pair of one arm from s[0], an active section with an active
// partner of the same arm, and return the sections consumed. Defined in dsp_pair_runs.c.
uint32_t dsp_svf_x2_peak_run(Filter * __restrict s, uint32_t n, float * __restrict samples, uint32_t count);
uint32_t dsp_svf_x2_shelf_run(Filter * __restrict s, uint32_t n, float * __restrict samples, uint32_t count);
uint32_t dsp_biquad_x2_generic_run(Filter * __restrict s, uint32_t n, float * __restrict samples, uint32_t count);
#endif // PICO_RP2350

#endif // DSP_CASCADE_H
