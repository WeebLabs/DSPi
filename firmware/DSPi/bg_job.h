/*
 * bg_job.h; long main-loop work run in budgeted steps between audio deliveries
 *
 * Core 0 main loop only. Design, budgets and measurements: current_architecture.md
 * "Step budget and background jobs" and Features/main_loop_bg_jobs_plan.md.
 */
#ifndef BG_JOB_H
#define BG_JOB_H

#include <stdbool.h>
#include <stdint.h>
#include "hardware/timer.h"
#include "loop_prof.h"   // BgJobId

// One step: work until deadline_us (at least one unit, so it always advances)
// and return true when the job is finished. A step must not restart its own job.
typedef bool (*BgStepFn)(void *ctx, uint32_t deadline_us);

typedef struct {
    BgJobId       id;
    BgStepFn      step;
    void         *ctx;
    volatile bool active;
} BgJob;

// Make a job known to bg_run_one(); idempotent, before its first start.
void bg_job_register(BgJob *job);

// Start, restart or stop a job. Restarting is the owner's business: reset its
// cursor, then start. Inline so the audio path can start a job without a call.
static inline void bg_job_start(BgJob *job)        { job->active = true; }
static inline void bg_job_cancel(BgJob *job)       { job->active = false; }
static inline bool bg_job_active(const BgJob *job) { return job->active; }

// True when a unit expected to cost unit_us would end past the deadline. Check
// before each unit, with the last cost of the same kind of unit as the estimate.
static inline bool bg_out_of_time(uint32_t deadline_us, uint32_t unit_us) {
    return (int32_t)(time_us_32() + unit_us - deadline_us) >= 0;
}

// Estimate for a kind of unit not yet measured (its first run is often cold
// XIP code): assumed not to fit, so it always starts a step of its own.
#define BG_UNIT_UNMEASURED  0xFFFFu

// One step of the next active job, round-robin. Call between audio deliveries.
void bg_run_one(void);

// One interrupt landing (the S/PDIF receiver's block check is the longest),
// set aside when deriving the budget.
#define BG_IRQ_FLOOR_US        60u
// An overrun counts past budget plus two landings: under real use a step
// absorbs two about once in 10 s (current_architecture.md "Step budget").
#define BG_OVERRUN_MARGIN_US  110u

// Longest one step may run at the live latency option and rate, in us.
uint32_t bg_step_budget_us(void);

// Steps over budget + BG_OVERRUN_MARGIN_US since boot (REQ_GET_STATUS 31);
// on DSPI_LOOP_PROF builds each job's worst step is kept too.
extern volatile uint32_t bg_over_budget_count;

#endif // BG_JOB_H
