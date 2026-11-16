/*
 * loop_prof.h; main-loop section, background-step and vendor-handler profiler
 *
 * Hosts read it through DIAG.LOOP_PROF_* (vendor/pages/diag.def). The section and
 * job lists are also parsed by tools/latency_bench/loop_prof.py, so keep each
 * X(...) entry on its own line. Only the job ids exist without DSPI_LOOP_PROF.
 */
#ifndef LOOP_PROF_H
#define LOOP_PROF_H

#include <stdbool.h>
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

#ifdef DSPI_LOOP_PROF
// Every value is the worst time in us since boot or loop_prof_reset(); reads never clear.
void loop_prof_mark(uint8_t section);
void loop_prof_job(uint8_t job, uint32_t us);
void loop_prof_command(uint16_t id, uint32_t us);     // id = (page << 8) | cmd
uint32_t loop_prof_section_worst(uint8_t section);
uint32_t loop_prof_job_worst(uint8_t job);
bool loop_prof_command_worst(uint16_t id, uint32_t *us);   // false: not built
void loop_prof_reset(void);
// bg_job.c. Above 1, each background step does one unit, on one pass in n,
// so a host can observe a job mid-flight.
extern volatile uint16_t bg_test_pace;
#define LOOP_PROF(id) loop_prof_mark(id)
#else
#define LOOP_PROF(id) ((void)0)
#endif

#endif // LOOP_PROF_H
