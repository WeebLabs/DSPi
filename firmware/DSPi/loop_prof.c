/*
 * loop_prof.c; bench-only main-loop profiler. See loop_prof.h.
 */
#include "loop_prof.h"
#include "hardware/timer.h"

_Static_assert(LOOP_PROF_STATUS_BASE + LOOP_PROF_SECTIONS <= LOOP_PROF_JOB_BASE &&
               LOOP_PROF_JOB_BASE + BG_JOB_COUNT <= LOOP_PROF_OPCODE_BASE,
               "loop_prof status ranges overlap");

static uint32_t section_worst_us[LOOP_PROF_SECTIONS];
static uint32_t job_worst_us[BG_JOB_COUNT];
static uint32_t opcode_worst_us[256];
static uint32_t section_start_us;
static uint8_t  section = 0xFF;

void loop_prof_mark(uint8_t next) {
    uint32_t now = time_us_32();
    if (section < LOOP_PROF_SECTIONS && now - section_start_us > section_worst_us[section])
        section_worst_us[section] = now - section_start_us;
    section = next;
    section_start_us = now;
}

void loop_prof_job(uint8_t job, uint32_t us) {
    if (job < BG_JOB_COUNT && us > job_worst_us[job]) job_worst_us[job] = us;
}

void loop_prof_opcode(uint8_t opcode, uint32_t us) {
    if (us > opcode_worst_us[opcode]) opcode_worst_us[opcode] = us;
}

static uint32_t take(uint32_t *slot) {
    uint32_t v = *slot;
    *slot = 0;
    return v;
}

uint32_t loop_prof_take(uint16_t i) {
    if (i >= LOOP_PROF_OPCODE_BASE && i < LOOP_PROF_OPCODE_BASE + 256u)
        return take(&opcode_worst_us[i - LOOP_PROF_OPCODE_BASE]);
    if (i >= LOOP_PROF_JOB_BASE && i < LOOP_PROF_JOB_BASE + BG_JOB_COUNT)
        return take(&job_worst_us[i - LOOP_PROF_JOB_BASE]);
    if (i >= LOOP_PROF_STATUS_BASE && i < LOOP_PROF_STATUS_BASE + LOOP_PROF_SECTIONS)
        return take(&section_worst_us[i - LOOP_PROF_STATUS_BASE]);
    return 0;
}
