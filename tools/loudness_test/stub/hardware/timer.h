// Host stand-in for the Pico SDK timer; the loudness job never reads real time.
#pragma once
#include <stdint.h>
static inline uint32_t time_us_32(void) { return 0; }
