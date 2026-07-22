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

#include <stdint.h>

// Track the measured input rate: ppm feed-forward plus a fill trim from
// the given output slot's consumer pool (fill_slot < 0 = rate term only).
// Callers must not apply until the pipeline runs at the detected rate
// (audio_state.freq matches), or the feed-forward is computed against the
// wrong nominal divider.
void input_servo_apply(float actual_freq, int fill_slot);

// Zero the integrator and park the VCXO at nominal. Call when tracking
// starts or stops and when lock is (re)acquired.
void input_servo_reset(void);

#endif // INPUT_SERVO_H
