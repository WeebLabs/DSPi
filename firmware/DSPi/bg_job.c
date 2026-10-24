/*
 * bg_job.c; main-loop step budget. See bg_job.h and
 * Documentation/current_architecture.md "Step budget".
 */
#include "bg_job.h"
#include "output_latency.h"
#include "usb_audio.h"   // audio_state

// A quiet pass (housekeeping plus one delivery) costs about this much.
#define BG_PASS_FLOOR_US     75u
#define BG_BUDGET_MIN_US     50u
#define BG_BUDGET_MAX_US    300u

volatile uint32_t bg_over_budget_count = 0;

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

void bg_step_account(BgJobId job, uint32_t elapsed_us) {
    if (elapsed_us > bg_step_budget_us() + BG_IRQ_FLOOR_US) bg_over_budget_count++;
#ifdef DSPI_LOOP_PROF
    loop_prof_job((uint8_t)job, elapsed_us);
#else
    (void)job;
#endif
}
