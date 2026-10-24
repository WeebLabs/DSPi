/*
 * loop_prof.h; main-loop section, background-step and vendor-handler profiler
 *
 * The section and job lists below are also parsed by tools/latency_bench/soak.py,
 * so keep each X(...) entry on its own line. Everything but the job ids
 * compiles to nothing unless DSPI_LOOP_PROF is set.
 */
#ifndef LOOP_PROF_H
#define LOOP_PROF_H

#include <stdint.h>

// Main-loop sections in loop order: LOOP_PROF(id) closes the running one.
#define LOOP_PROF_SECTION_LIST(X)            \
    X(LP_HEAD,          "head")              \
    X(LP_USB_VENDOR,    "usb+vendor")        \
    X(LP_NOTIFY,        "notify")            \
    X(LP_TRANSPORTS,    "uart/lg/dac")       \
    X(LP_DELIVERY_HOST, "delivery_host")     \
    X(LP_CS_TICK,       "control_surfaces")  \
    X(LP_SIGGEN,        "siggen_service")    \
    X(LP_IDLE_PUMP,     "idle_pump")         \
    X(LP_RTA,           "rta")               \
    X(LP_INPUT,         "audio_inputs+dsp")  \
    X(LP_BG_JOBS,       "bg_jobs")           \
    X(LP_DELIVERY_BG,   "delivery_bg")       \
    X(LP_FLASH_SETS,    "flash_sets")        \
    X(LP_EQ_UPDATES,    "eq_updates")        \
    X(LP_COEFF_UPDATES, "coeff_updates")     \
    X(LP_DEFERRED_OPS,  "deferred_ops")      \
    X(LP_DELIVERY_END,  "delivery_end")

// Background steps accounted against the step budget (bg_job.h).
#define BG_JOB_LIST(X)                       \
    X(BG_JOB_RTA_STEP,  "rta_step")          \
    X(BG_JOB_LOUDNESS,  "loudness_rebuild")  \
    X(BG_JOB_CS_REVERT, "cs_revert")

#define LOOP_PROF_ENUM_ENTRY(id, name) id,
typedef enum { LOOP_PROF_SECTION_LIST(LOOP_PROF_ENUM_ENTRY) LOOP_PROF_SECTIONS } LoopProfSection;
typedef enum { BG_JOB_LIST(LOOP_PROF_ENUM_ENTRY) BG_JOB_COUNT } BgJobId;
#undef LOOP_PROF_ENUM_ENTRY

// REQ_GET_STATUS wValue bases; each read returns the worst time in us since the
// last read of that index, and clears it.
#define LOOP_PROF_STATUS_BASE    64    // + LoopProfSection
#define LOOP_PROF_JOB_BASE       96    // + BgJobId
#define LOOP_PROF_OPCODE_BASE   256    // + vendor bRequest
// Bench pacing, not a read: wValue 512 + n (n >= 1) makes each background step
// do one unit, on one pass in n, so a host can observe a job mid-flight.
#define LOOP_PROF_BG_PACE_BASE  512

#ifdef DSPI_LOOP_PROF
void loop_prof_mark(uint8_t section);
void loop_prof_job(uint8_t job, uint32_t us);
void loop_prof_opcode(uint8_t opcode, uint32_t us);
uint32_t loop_prof_take(uint16_t status_index);
extern volatile uint16_t bg_test_pace;   // bg_job.c; 1 = normal pacing
#define LOOP_PROF(id) loop_prof_mark(id)
#else
#define LOOP_PROF(id) ((void)0)
#endif

#endif // LOOP_PROF_H
