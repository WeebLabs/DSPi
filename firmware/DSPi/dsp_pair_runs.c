#include "dsp_svf.h"
#include "dsp_biquad.h"
#include "dsp_cascade.h"

#if PICO_RP2350
// Each pair arm whose loop schedules badly inside dsp_cascade_block runs here, out of
// line, one call per run of same-arm pairs. See "FP contraction" in current_architecture.md.

static inline uint32_t next_active(const Filter *s, uint32_t k, uint32_t n) {
    while (k < n && s[k].bypass) k++;
    return k;
}

static inline bool svf_pair_ok(const Filter *a, const Filter *b, DspSvfArm arm) {
    return !a->first_order && !b->first_order && a->use_svf && b->use_svf &&
           dsp_svf_arm_class(a) == arm && dsp_svf_arm_class(b) == arm;
}
static inline bool svf_peak_ok(const Filter *a, const Filter *b)  { return svf_pair_ok(a, b, DSP_SVF_ARM_PEAK); }
static inline bool svf_shelf_ok(const Filter *a, const Filter *b) { return svf_pair_ok(a, b, DSP_SVF_ARM_GENERIC); }

static inline bool bq_generic_ok(const Filter *a, const Filter *b) {
    return !a->first_order && !b->first_order && !a->use_svf && !b->use_svf &&
           !(a->filter_type == FILTER_PEAKING && b->filter_type == FILTER_PEAKING) &&
           !(dsp_biquad_is_lphp(a) && dsp_biquad_is_lphp(b));
}

// The continue test must pair exactly as dsp_cascade_block would, or output stops being
// bit-identical to the unfused cascade.
#define PAIR_RUN(name, ARM, OK) \
DSP_TIME_CRITICAL uint32_t name(Filter * __restrict s, uint32_t n, \
                                float * __restrict samples, uint32_t count) { \
    uint32_t i = 0; \
    for (;;) { \
        uint32_t j = next_active(s, i + 1, n); \
        ARM(&s[i], &s[j], samples, count); \
        i = next_active(s, j + 1, n); \
        uint32_t k = next_active(s, i + 1, n); \
        if (k >= n || !OK(&s[i], &s[k])) return i; \
    } \
}

PAIR_RUN(dsp_svf_x2_peak_run, dsp_svf_x2_peak, svf_peak_ok)
PAIR_RUN(dsp_svf_x2_shelf_run, dsp_svf_x2_shelf, svf_shelf_ok)
PAIR_RUN(dsp_biquad_x2_generic_run, dsp_biquad_x2_generic, bq_generic_ok)
#endif // PICO_RP2350
