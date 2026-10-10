/*
 * input_servo.h; shared output-clock servo actuation for externally
 * clocked inputs (SPDIF input, ADAT input in slave clock mode).
 *
 * Callers own lock gating, rate limiting, and the input rate measurement;
 * this module owns the divider math and PIO/MCK writes.
 */

#ifndef INPUT_SERVO_H
#define INPUT_SERVO_H

#include <stdbool.h>
#include <stdint.h>

// Servo all output slots (SPDIF/I2S types), the ADAT output, and MCK to the
// measured input rate: rate-based dividers plus a proportional trim from
// slot 0's consumer fill. Returns the SPDIF-format divider written (16.8),
// or 0 if actual_freq failed the sanity check. Skips PIO writes when the
// dividers are unchanged.
uint32_t input_servo_apply(float actual_freq);

// Rewrite the cached dividers to every slot, the ADAT output and MCK, for
// slots re-initialized at nominal while the servo stays locked. Returns false
// (and writes nothing) when nothing has been applied since the last reset.
bool input_servo_reapply(void);

// Clear the written-divider cache so the next apply performs a full rewrite.
// Call when (re)acquiring lock or after outputs restart at nominal dividers.
void input_servo_reset(void);

// Last SPDIF-format divider written (0 if none since reset). Callers gate
// this on their own lock state for *_current_tx_divider() semantics.
uint32_t input_servo_current_divider(void);

#endif // INPUT_SERVO_H
