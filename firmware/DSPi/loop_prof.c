/*
 * loop_prof.c; bench-only main-loop section profiler. See loop_prof.h.
 */
#include "loop_prof.h"
#include "hardware/timer.h"

static uint32_t worst_us[LOOP_PROF_SECTIONS];
static uint32_t section_start_us;
static uint8_t  section = 0xFF;

void loop_prof_mark(uint8_t next) {
    uint32_t now = time_us_32();
    if (section < LOOP_PROF_SECTIONS && now - section_start_us > worst_us[section])
        worst_us[section] = now - section_start_us;
    section = next;
    section_start_us = now;
}

uint32_t loop_prof_take(uint8_t n) {
    if (n >= LOOP_PROF_SECTIONS) return 0;
    uint32_t v = worst_us[n];
    worst_us[n] = 0;
    return v;
}
