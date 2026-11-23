/* Host stand-in: the RP2040 Q28 multiply, the firmware's own, an exact one, or an int64 one for the wide build. */
#pragma once
#include <stdint.h>
#if defined(SP_Q28_WIDE)
static inline int64_t fast_mul_q28(int64_t a, int64_t b) { return (a * b) >> 28; }
#elif defined(SP_EXACT_MUL)
static inline int32_t fast_mul_q28(int32_t a, int32_t b) { return (int32_t)(((int64_t)a * b) >> 28); }
#else
static inline int32_t fast_mul_q28(int32_t a, int32_t b) {   /* copy of firmware/DSPi/dsp_pipeline.c */
    int32_t ah = a >> 16; uint32_t al = a & 0xFFFF; int32_t bh = b >> 16; uint32_t bl = b & 0xFFFF;
    int32_t high = ah * bh, mid1 = ah * bl, mid2 = al * bh;
    return (high << 4) + ((mid1 + mid2) >> 12);
}
#endif
