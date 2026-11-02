#ifndef STUB_PICO_TIME_H
#define STUB_PICO_TIME_H
#include <stdint.h>
extern uint64_t stub_now_us;
static inline uint64_t time_us_64(void) { return stub_now_us; }
#endif
