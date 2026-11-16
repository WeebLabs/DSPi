/*
 * loop_prof.c; bench-only main-loop profiler. See loop_prof.h.
 */
#include <string.h>
#include "loop_prof.h"
#include "hardware/timer.h"
#include "vc_core.h"   // vc_cmd_index(), VC_PAGE_LIST

// Commands built into this image, counted from the same .def files as the page
// tables. Each VC_PAGE line leaves a marker, so a page missing here fails below.
#define VC_PAGE(m, num, rev, doc) lp_page_##m,
#define VC_CMD(c, name, g, s, ak, gl, smin, smax, fl, a, doc) VC_IDS_CAT(lp_cmd_, __COUNTER__),
#define VC_RESERVED(c, note)
enum {
#include "pages/sys.def"
#include "pages/store.def"
#include "pages/input.def"
#include "pages/output.def"
#include "pages/route.def"
#include "pages/filter.def"
#include "pages/dyn.def"
#include "pages/fx.def"
#include "pages/meas.def"
#include "pages/cs.def"
#include "pages/xport.def"
#include "pages/clock.def"
#include "pages/diag.def"
    LP_PAGES_AND_CMDS
};
#undef VC_PAGE
#undef VC_CMD
#undef VC_RESERVED

#define X(m) _Static_assert(lp_page_##m >= 0, "loop_prof.c must count every page");
VC_PAGE_LIST(X)
#undef X
#define X(m) + 1
enum { LP_PAGES = 0 VC_PAGE_LIST(X) };
#undef X
#define LOOP_PROF_COMMANDS (LP_PAGES_AND_CMDS - LP_PAGES)

static uint32_t section_worst_us[LOOP_PROF_SECTIONS];
static uint32_t job_worst_us[BG_JOB_COUNT];
static uint32_t command_worst_us[LOOP_PROF_COMMANDS];   // by vc_cmd_index()
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

void loop_prof_command(uint16_t id, uint32_t us) {
    int i = vc_cmd_index(id);
    if (i >= 0 && i < LOOP_PROF_COMMANDS && us > command_worst_us[i]) command_worst_us[i] = us;
}

uint32_t loop_prof_section_worst(uint8_t s) {
    return s < LOOP_PROF_SECTIONS ? section_worst_us[s] : 0;
}

uint32_t loop_prof_job_worst(uint8_t job) {
    return job < BG_JOB_COUNT ? job_worst_us[job] : 0;
}

bool loop_prof_command_worst(uint16_t id, uint32_t *us) {
    int i = vc_cmd_index(id);
    if (i < 0 || i >= LOOP_PROF_COMMANDS) return false;
    *us = command_worst_us[i];
    return true;
}

void loop_prof_reset(void) {
    memset(section_worst_us, 0, sizeof(section_worst_us));
    memset(job_worst_us, 0, sizeof(job_worst_us));
    memset(command_worst_us, 0, sizeof(command_worst_us));
}
