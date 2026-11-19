/*
 * eq_prof.h: cycle-exact EQ cascade profiler (DSPI_EQ_PROF bench builds only, RP2350).
 * Times every dsp_cascade_block() call from its callers with the per-core DWT
 * cycle counter; read through DIAG.EQ_PROF. Never in release builds.
 */
#ifndef EQ_PROF_H
#define EQ_PROF_H

#if defined(DSPI_EQ_PROF) && PICO_RP2350

#include <stdint.h>
#include <stdbool.h>
#include "hardware/structs/m33.h"
#include "hardware/structs/sio.h"

typedef struct {
    volatile uint64_t cycles;     // total cycles inside timed calls
    volatile uint32_t calls;
    volatile uint32_t samples;    // sum of per-call sample counts
    volatile uint32_t min_cycles; // cheapest call since the last min reset (interrupt-free bound)
    volatile uint32_t min_samples;
    volatile bool     reset_min;  // set by the host, honoured by the owning core
    volatile bool     dwt_on;
} EqProfCore;

extern EqProfCore eq_prof[2];

// DWT is per core, so each core enables its own counter on its first timed call.
#define EQ_PROF_START() \
    EqProfCore *_ep = &eq_prof[sio_hw->cpuid & 1]; \
    if (!_ep->dwt_on) { m33_hw->demcr |= M33_DEMCR_TRCENA_BITS; \
                        m33_hw->dwt_ctrl |= M33_DWT_CTRL_CYCCNTENA_BITS; _ep->dwt_on = true; } \
    uint32_t _ep_t0 = m33_hw->dwt_cyccnt

#define EQ_PROF_END(n) do { \
    uint32_t _ep_d = m33_hw->dwt_cyccnt - _ep_t0; \
    _ep->cycles += _ep_d; _ep->calls++; _ep->samples += (n); \
    if (_ep->reset_min || _ep_d < _ep->min_cycles) { \
        _ep->min_cycles = _ep_d; _ep->min_samples = (n); _ep->reset_min = false; } \
} while (0)

#else
#define EQ_PROF_START() do { } while (0)
#define EQ_PROF_END(n)  do { } while (0)
#endif

#endif // EQ_PROF_H
