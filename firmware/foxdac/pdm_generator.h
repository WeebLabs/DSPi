#ifndef PDM_GENERATOR_H
#define PDM_GENERATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

// Functions exposed to main.c
void pdm_setup_hw(void);
void pdm_core1_entry(void);
void pdm_update_clock(uint32_t freq);
void pdm_push_sample(int32_t sample, bool reset);

#endif // PDM_GENERATOR_H
