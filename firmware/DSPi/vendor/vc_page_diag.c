/*
 * vc_page_diag.c: DIAG page (0xF0). Command set: vendor_command_overhaul_plan.md section 5.13.
 */

#include "vc_core.h"
#include "config.h"
#include "clock_diag.h"
#include "loopback.h"
#include "loop_prof.h"

#if DSPI_CLOCK_DIAG
// VC_F_USB_ONLY is a choice for a bench readout: the packet fits the 256-byte
// reply buffer (VC_REPLY_MAX) that UART and I2C also use.
static VcResult diag_clock_diag_get(VcCtx *c) {
    ClockDiagPacket diag;
    clock_diag_get_snapshot(&diag);
    vc_reply(c, &diag, sizeof(diag));
    return VC_OK;
}
_Static_assert(sizeof(ClockDiagPacket) <= VC_REPLY_MAX, "DIAG.CLOCK_DIAG exceeds the reply buffer");
#endif

#ifdef DSPI_LOOPBACK
static VcResult diag_loopback_stats_get(VcCtx *c) {
    vc_out_u32(c, 0, loopback_get_overflow_count());
    vc_out_u32(c, 4, loopback_get_underrun_count());
    return VC_OK;
}
#endif

#ifdef DSPI_LOOP_PROF
static VcResult diag_lp_section_get(VcCtx *c) {
    if (c->addr >= LOOP_PROF_SECTIONS) return VC_ERR_ADDRESS;
    vc_out_u32(c, 0, loop_prof_section_worst((uint8_t)c->addr));
    return VC_OK;
}

static VcResult diag_lp_job_get(VcCtx *c) {
    if (c->addr >= BG_JOB_COUNT) return VC_ERR_ADDRESS;
    vc_out_u32(c, 0, loop_prof_job_worst((uint8_t)c->addr));
    return VC_OK;
}

static VcResult diag_lp_command_get(VcCtx *c) {
    uint32_t us;
    if (!loop_prof_command_worst(c->addr, &us)) return VC_ERR_ADDRESS;
    vc_out_u32(c, 0, us);
    return VC_OK;
}

static VcResult diag_lp_bg_pace_get(VcCtx *c) {
    vc_out_u16(c, 0, bg_test_pace);
    return VC_OK;
}

static VcResult diag_lp_bg_pace_set(VcCtx *c) {
    uint16_t n = vc_in_u16(c, 0);
    bg_test_pace = n ? n : 1;
    return VC_OK;
}

static VcResult diag_lp_reset_set(VcCtx *c) {
    (void)c;
    loop_prof_reset();
    return VC_OK;
}
#endif

#define VC_PAGE_DEF "pages/diag.def"
#include "vc_page_table.inc"
