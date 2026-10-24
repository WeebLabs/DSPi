/*
 * bg_job.h; budget for long main-loop work done in steps between audio deliveries
 *
 * Phase 1 of Documentation/Features/main_loop_bg_jobs_plan.md: the step budget
 * and overrun accounting. The step runner itself is Phase 2.
 */
#ifndef BG_JOB_H
#define BG_JOB_H

#include <stdint.h>
#include "loop_prof.h"   // BgJobId

// Interrupts land inside any step; an overrun is only counted past this margin.
#define BG_IRQ_FLOOR_US  60u

// Longest one step may run at the live latency option and rate, in us.
uint32_t bg_step_budget_us(void);

// Record one step's duration: counts an overrun (all builds) and, on
// DSPI_LOOP_PROF builds, the job's worst step.
void bg_step_account(BgJobId job, uint32_t elapsed_us);

// Steps over budget + BG_IRQ_FLOOR_US since boot (REQ_GET_STATUS 31).
extern volatile uint32_t bg_over_budget_count;

#endif // BG_JOB_H
