#pragma once

// ----------------------------------------------------------------------------
// Spectrum analyser kernel: real FFT, frequency-domain Blackman-Harris, band sums, and
// band sums.  Platform-neutral so the
// host harness (tools/rta_test) compiles this same code natively against a
// numpy oracle.  Protocol and engine live in rta.h / rta.c; design notes in
// Documentation/Features/spectrum_analyser_spec.md.
// ----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>

// Sample format is selected by RTA_SAMPLE_FLOAT so the host harness can build
// both variants; firmware derives it from the platform.
#ifndef RTA_SAMPLE_FLOAT
#  if PICO_RP2350
#    define RTA_SAMPLE_FLOAT 1
#  else
#    define RTA_SAMPLE_FLOAT 0
#  endif
#endif

#if RTA_SAMPLE_FLOAT
typedef float    rta_sample_t;   // linear, full scale = +/-1.0
typedef float    rta_bandacc_t;  // band power accumulator
#else
typedef int16_t  rta_sample_t;   // Q15, full scale = +/-32767
typedef uint64_t rta_bandacc_t;  // band power accumulator (exact integer sums)
#endif

// RTA_RAM_FUNC marks kernel code that may run from the audio path.  The host
// build (-DRTA_HOST) has no such placement.
#ifdef RTA_HOST
#  define RTA_RAM_FUNC
#else
#  include "config.h"
#  define RTA_RAM_FUNC DSP_TIME_CRITICAL
#endif

#define RTA_ORDER_MIN    8
#define RTA_ORDER_MAX    10
#define RTA_MAX_POINTS   (1u << RTA_ORDER_MAX)
#define RTA_MAX_BANDS    37

// Wire level byte: 0.5 dB steps, 243 = 0 dBFS, 255 = +6 dBFS, 0 = floor.
#define RTA_LEVEL_ZERO_DBFS  243

// Band edge table for one (sample rate, order) combination.  lo/hi are
// inclusive bin indices into the N/2 magnitude bins.  A band without a bin at this
// order is empty (lo > hi) and reads the floor.
typedef struct {
    uint32_t sample_rate_hz;
    uint8_t  order;
    uint8_t  n_bands;          // bands valid at this rate (34 or 37)
    uint8_t  reserved0;
    uint8_t  reserved;
    uint16_t lo[RTA_MAX_BANDS];
    uint16_t hi[RTA_MAX_BANDS];
} RtaBandTable;

// Band table, or NULL for an unsupported rate/order.
const RtaBandTable *rta_band_table(uint32_t sample_rate_hz, uint8_t order);

// Nominal third-octave centre frequency of band index b (IEC 61260), for caps.
uint16_t rta_band_centre_hz(uint8_t b);

// ---------------------------------------------------------------------------
// Real FFT, in place, resumable.
//
// Input: n = 1 << order real samples in buf[0..n-1].
// Output: n/2 complex bins packed as (re, im) pairs in buf[0..n-1], with the
// purely real Nyquist bin X[n/2] stored in the imaginary slot of X[0].
// Scaling: the fixed-point kernel halves every stage so the result carries a
// 1/n factor; the float kernel applies the same 1/n so both formats agree.
//
// Zero the cursor before the first call. Each call does at most max_units units
// of the current stage (a prescale sample, a butterfly, a permutation cycle
// start or a split bin pair) and returns true once the whole transform is done.
// Results do not depend on max_units.
// ---------------------------------------------------------------------------
typedef struct {
    uint8_t  stage;
    uint16_t pos;                          // units of the current stage already done
    uint32_t seen[RTA_MAX_POINTS / 64];    // permutation: slots already placed
} RtaFftCursor;

bool rta_fft_step(rta_sample_t *buf, uint8_t order, RtaFftCursor *c, uint16_t max_units);

// ---------------------------------------------------------------------------
// Post-transform: frequency-domain Blackman-Harris, bin power, band sums, level bytes.
//
// band_power: table->n_bands linear powers, normalised so a full-scale sine
//             reads 1.0 in the band that contains it.  May be NULL.
// bins:       n/2 level bytes, bin magnitude normalised so a full-scale sine
//             at a bin centre reads RTA_LEVEL_ZERO_DBFS.  May be NULL.
//
// The per-bin path must be integer-only on RP2040 (CLZ log2), because a
// float log per bin would cost more than the transform itself.
// ---------------------------------------------------------------------------
void rta_fft_finish(const rta_sample_t *buf, uint8_t order,
                    const RtaBandTable *table, float *band_power, uint8_t *bins);

// The same pass, resumable in bin chunks: about 1 us per bin on RP2350, so the
// whole pass in one main-loop step stalls every other main-loop task. Begin,
// then step until it returns true; band_power is written on that last call.
typedef struct {
    uint16_t      next_bin;
    uint8_t       band;          // first band whose range can still hold next_bin
    rta_bandacc_t acc[RTA_MAX_BANDS];
} RtaFinishCursor;

void rta_fft_finish_begin(RtaFinishCursor *c);
bool rta_fft_finish_step(RtaFinishCursor *c, const rta_sample_t *buf, uint8_t order,
                         const RtaBandTable *table, float *band_power, uint8_t *bins,
                         uint16_t max_bins);

// Linear power (1.0 = 0 dBFS) to wire level byte; clamps to 0..255.
uint8_t rta_level_from_power(float power);
