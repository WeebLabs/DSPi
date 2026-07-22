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

#endif // SOFT_VCXO_H
