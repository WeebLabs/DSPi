/*
 * bg_job.c; background job runner and step budget. See bg_job.h.
 */
#include "bg_job.h"
#include "output_latency.h"
#include "usb_audio.h"   // audio_state

// A quiet pass (housekeeping plus one delivery) costs about this much (RP2350,
// measured 2026-10-24; current_architecture.md "Step budget").
#define BG_PASS_FLOOR_US    120u
#define BG_BUDGET_MIN_US     50u
#define BG_BUDGET_MAX_US    300u

volatile uint32_t bg_over_budget_count = 0;

static BgJob *jobs[BG_JOB_COUNT];
static uint8_t next_job;

#ifdef DSPI_LOOP_PROF
volatile uint16_t bg_test_pace = 1;
#endif

void bg_job_register(BgJob *job) {
    if ((unsigned)job->id < BG_JOB_COUNT) jobs[job->id] = job;
}

static void bg_step_account(BgJobId job, uint32_t elapsed_us) {
    if (elapsed_us > bg_step_budget_us() + BG_OVERRUN_MARGIN_US) bg_over_budget_count++;
#ifdef DSPI_LOOP_PROF
    loop_prof_job((uint8_t)job, elapsed_us);
#else
    (void)job;
#endif
}

void bg_run_one(void) {
#ifdef DSPI_LOOP_PROF
    static uint16_t skipped;
    if (bg_test_pace > 1 && ++skipped < bg_test_pace) return;
    skipped = 0;
#endif
    for (uint8_t k = 0; k < BG_JOB_COUNT; k++) {
        uint8_t i = (uint8_t)((next_job + k) % BG_JOB_COUNT);
        BgJob *job = jobs[i];
        if (!job || !job->active) continue;
        next_job = (uint8_t)((i + 1) % BG_JOB_COUNT);
        uint32_t t0 = time_us_32();
        uint32_t deadline = t0 + bg_step_budget_us();
#ifdef DSPI_LOOP_PROF
        if (bg_test_pace > 1) deadline = t0;   // one unit per step
#endif
        if (job->step(job->ctx, deadline)) job->active = false;
        bg_step_account(job->id, time_us_32() - t0);
        return;
    }
}

uint32_t bg_step_budget_us(void) {
    uint32_t fs = audio_state.freq ? (uint32_t)audio_state.freq : 48000u;
    uint32_t t = output_ring_target_frames, b = input_batch_frames;
    uint32_t slack_us = t > b ? (uint32_t)((uint64_t)(t - b) * 1000000u / fs) : 0;
    // Half of what one delivery gap can absorb: a step shares its gap with the
    // rest of a pass and with interrupts.
    uint32_t floor_us = BG_PASS_FLOOR_US + BG_IRQ_FLOOR_US;
    uint32_t budget = slack_us > floor_us ? (slack_us - floor_us) / 2u : 0;
    if (budget < BG_BUDGET_MIN_US) budget = BG_BUDGET_MIN_US;
    if (budget > BG_BUDGET_MAX_US) budget = BG_BUDGET_MAX_US;
    return budget;
}

