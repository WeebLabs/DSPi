#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "pico/time.h"
typedef enum { BG_JOB_UI, BG_JOB_COUNT } BgJobId;
typedef bool (*BgStepFn)(void *ctx, uint32_t deadline_us);
typedef struct { BgJobId id; BgStepFn step; void *ctx; volatile bool active; } BgJob;
void bg_job_register(BgJob *job);
static inline void bg_job_start(BgJob *job)  { job->active = true; }
static inline bool bg_out_of_time(uint32_t deadline_us, uint32_t unit_us) {
    return (int32_t)(time_us_32() + unit_us - deadline_us) >= 0; }
void bg_run_one(void);
