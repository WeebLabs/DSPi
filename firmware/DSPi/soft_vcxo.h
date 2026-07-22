/*
 * soft_vcxo.h; fractional-N trim of the system PLL ("soft VCXO").
 *
 * Replaces per-output PIO divider trimming for input clock tracking: a
 * ppm-level nudge of sys_clk moves every derived output clock (SPDIF TX,
 * I2S TX, ADAT TX, MCK, PDM) together, so inter-slot alignment is
 * preserved by construction. See soft_vcxo.c for the mechanism.
 */

#ifndef SOFT_VCXO_H
#define SOFT_VCXO_H

// One-time hardware bring-up (DMA channels, DMA timer, PWM pacer). Inert
// when sys_clk is not the 307.2 MHz / fbdiv 128 plan the constants assume.
void soft_vcxo_init(void);

// Command a sys_clk offset in ppm (clamped to +-SOFT_VCXO_MAX_PPM);
// 0 parks the PLL at exactly nominal. Cheap; callable at servo rate.
void soft_vcxo_set_ppm(float ppm);

// Last commanded (post-clamp) offset, for status/debug.
float soft_vcxo_current_ppm(void);

#include "config.h"
#if DSPI_CLOCK_DIAG
#include <stdint.h>
#include <stdbool.h>

// Snapshot of the VCXO command path internals for the CLKDIAG dump.  min/max
// commanded ppm are windowed: soft_vcxo_get_diag() resets them on read.
typedef struct {
    float    last_ppm;      // last post-clamp commanded ppm
    float    min_ppm;       // min commanded ppm since last read
    float    max_ppm;       // max commanded ppm since last read
    uint32_t set_calls;     // total soft_vcxo_set_ppm calls (cumulative)
    uint32_t clamp_sats;    // times a command hit +-MAX_PPM (cumulative)
    uint32_t sign_changes;  // command sign flips (cumulative)
    uint32_t pulse_hz;      // effective pulse rate sys/(clkdiv*wrap), 0=parked
    uint32_t pwm_wrap;      // PWM period (wrap+1); up to 0x10000
    uint16_t pwm_div;       // programmed PWM clkdiv
    uint8_t  fbdiv0;        // commanded excursion fbdiv (127/128/129)
    bool     parked;        // true when the last command parked the loop
} SoftVcxoDiag;

// Fill *out and reset the windowed min/max ppm.  Cheap; main-loop context.
void soft_vcxo_get_diag(SoftVcxoDiag *out);
#endif // DSPI_CLOCK_DIAG

#endif // SOFT_VCXO_H
