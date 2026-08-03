// Shared audio clock divider base for all output slots.
#ifndef AUDIO_CLOCK_DIV_H
#define AUDIO_CLOCK_DIV_H

#include "hardware/clocks.h"

// 16.8 divider for a 256*Fs PIO clock, rounded to nearest.  Every audio
// output clock (SPDIF, ADAT, I2S BCK, MCK, PDM) must derive from this value
// by exact power-of-two scaling: independent rounding would give slots
// different ppm errors and let them drift apart at fractional sys clocks.
static inline uint32_t audio_base_divider_16_8(uint32_t sample_freq) {
    uint32_t sys = clock_get_hz(clk_sys);
    return (sys + sample_freq / 2u) / sample_freq;
}

#endif
