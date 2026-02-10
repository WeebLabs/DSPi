#ifndef PDM_GENERATOR_H
#define PDM_GENERATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

// Enable/disable PDM hardware (PIO + DMA) from Core 0
void pdm_set_enabled(bool enabled);

// Functions exposed to main.c
void pdm_setup_hw(void);
void pdm_core1_entry(void);
void pdm_update_clock(uint32_t freq);
void pdm_push_sample(int32_t sample, bool reset);

// Core 1 mode and EQ worker state (written by Core 0, read by Core 1)
extern volatile Core1Mode core1_mode;
extern Core1EqWork core1_eq_work;

#endif // PDM_GENERATOR_H
