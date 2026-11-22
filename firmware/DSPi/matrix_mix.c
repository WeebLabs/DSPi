/*
 * matrix_mix.c: out-of-line home of the RP2350 matrix kernel (matrix_mix.h).
 * Its own TU so the kernel's code and FP contraction setting do not depend on process_input_block().
 */
#include "config.h"
#include "matrix_mix.h"

#if PICO_RP2350
void __noinline DSP_TIME_CRITICAL matrix_mix_output(float *dst, const float *const *src,
                                                     const float *gain, int n, uint32_t count) {
    matrix_mix_kernel(dst, src, gain, n, count);
}
#endif
