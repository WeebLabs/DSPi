/*
 * input_servo.h; shared clock servo for externally clocked inputs
 * (SPDIF input, ADAT input in slave clock mode, I2S slave).
 *
 * Callers own lock gating, rate limiting, and the input rate measurement;
 * this module owns the servo terms and actuates the soft VCXO (system PLL
 * trim, soft_vcxo.c). Output PIO dividers stay at their nominal values.
 */

#ifndef INPUT_SERVO_H
#define INPUT_SERVO_H

#include <stdbool.h>
#include <stdint.h>
#include "hardware/timer.h"

// Servo tick period: the old 1000-pass tick at the measured ~75 us quiet pass, so
// the per-tick gains in input_servo.c run no faster than they were bench-verified.
// See current_architecture.md "Clock Servo".
#define INPUT_SERVO_TICK_US  75000u

// True once a tick period has elapsed since *last_us, which it then restarts.
// Unsigned elapsed time stays wrap-safe across any gap (at most one tick late).
static __force_inline bool input_servo_tick_due(uint32_t *last_us) {
    uint32_t now = time_us_32();
    if ((uint32_t)(now - *last_us) < INPUT_SERVO_TICK_US) return false;
    *last_us = now;
    return true;
}

// Track the measured input rate: ppm feed-forward plus a fill trim from
// the given output slot's ring fill (fill_slot < 0 = rate term only).
// Callers must not apply until the pipeline runs at the detected rate
// (audio_state.freq matches), or the feed-forward is computed against the
// wrong nominal divider.
void input_servo_apply(float actual_freq, int fill_slot);

// Zero the integrator and park the VCXO at nominal. Call only when tracking
// starts or stops: a relock holds the last-good trim, and a call on the
// pipeline-reset path would park a locked VCXO mid-lock.
void input_servo_reset(void);

#include "config.h"
#if DSPI_CLOCK_DIAG
// Per-tick servo snapshot for the CLKDIAG dump.  ppm fields are the last
// computed term; min/max total ppm are windowed (reset on read).  Written
// only from input_servo_apply (plain stores); read from the main loop.
typedef struct {
    float    actual_freq;   // measured input rate fed in
    float    ppm_ff;        // feed-forward ppm
    float    ppm_total;     // total commanded ppm (ff + fill)
    float    ppm_min;       // min total ppm since last read
    float    ppm_max;       // max total ppm since last read
    float    integral_ppm;  // integrator value, in ppm
    int32_t  fill_error;    // consumer fill minus target, in buckets
    int32_t  fill_slot;     // slot used for fill term (-1 = rate only)
    uint32_t ticks;         // total servo ticks that actuated (cumulative)
} InputServoDiag;

// Fill *out and reset the windowed min/max total ppm.  Main-loop context.
void input_servo_get_diag(InputServoDiag *out);
#endif // DSPI_CLOCK_DIAG

#endif // INPUT_SERVO_H
