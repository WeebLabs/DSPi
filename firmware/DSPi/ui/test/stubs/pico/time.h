#pragma once
#include <stdint.h>
extern uint64_t sim_us;
static inline uint64_t time_us_64(void) { return sim_us; }
static inline uint32_t time_us_32(void) { return (uint32_t)sim_us; }
