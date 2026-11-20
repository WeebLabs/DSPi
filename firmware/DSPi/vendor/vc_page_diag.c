/*
 * vc_page_diag.c: DIAG page (0xF0). Command set: vendor_command_overhaul_plan.md section 5.13.
 */

#include "vc_core.h"
#include "config.h"
#include "clock_diag.h"
#include "loopback.h"
#include "loop_prof.h"
#include "eq_prof.h"
#include "wirecap.h"

#if DSPI_CLOCK_DIAG
// Served on every transport. The assert below keeps it inside the UART and I2C reply buffer.
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

#ifdef DSPI_EQ_PROF
// Each core's u64 total is reread until stable, since the owning core may be mid-update.
static VcResult diag_eq_prof_get(VcCtx *c) {
    for (int k = 0; k < 2; k++) {
        uint64_t cyc; uint32_t calls, smp;
        do { cyc = eq_prof[k].cycles; calls = eq_prof[k].calls; smp = eq_prof[k].samples; }
        while (cyc != eq_prof[k].cycles);
        uint16_t o = (uint16_t)(k * 24);
        vc_out_u32(c, o, (uint32_t)cyc);
        vc_out_u32(c, (uint16_t)(o + 4), (uint32_t)(cyc >> 32));
        vc_out_u32(c, (uint16_t)(o + 8), calls);
        vc_out_u32(c, (uint16_t)(o + 12), smp);
        vc_out_u32(c, (uint16_t)(o + 16), eq_prof[k].min_cycles);
        vc_out_u32(c, (uint16_t)(o + 20), eq_prof[k].min_samples);
    }
    return VC_OK;
}

static VcResult diag_eq_prof_reset_min_set(VcCtx *c) {
    (void)c;
    eq_prof[0].reset_min = true;
    eq_prof[1].reset_min = true;
    return VC_OK;
}
#endif

#ifdef DSPI_WIRECAP
static VcResult wirecap_vc(WirecapResult r) {
    switch (r) {
        case WIRECAP_OK:        return VC_OK;
        case WIRECAP_ERR_STATE: return VC_ERR_STATE;
        case WIRECAP_ERR_VALUE: return VC_ERR_VALUE;
        default:                return VC_ERR_BUSY;
    }
}

static VcResult diag_wirecap_arm_set(VcCtx *c) {
    return wirecap_vc(wirecap_arm(vc_in_u8(c, 0), vc_in_u8(c, 1), vc_in_u16(c, 2),
                                  vc_in_u8(c, 4), vc_in_u32(c, 6)));
}

static VcResult diag_wirecap_status_get(VcCtx *c) {
    wirecap_status(c->out);
    return VC_OK;
}

static VcResult diag_wirecap_read_get(VcCtx *c) {
    c->out_len = wirecap_read((uint32_t)c->addr * 256u, c->out, 256);
    return c->out_len ? VC_OK : VC_ERR_STATE;
}
_Static_assert(VC_REPLY_MAX >= 256, "DIAG.WIRECAP_READ chunks are 256 bytes");

static VcResult diag_wirecap_clkgen_get(VcCtx *c) {
    wirecap_clkgen_status(c->out);
    return VC_OK;
}

static VcResult diag_wirecap_clkgen_set(VcCtx *c) {
    return wirecap_vc(wirecap_clkgen_set(vc_in_u8(c, 0) != 0, vc_in_u32(c, 1)));
}

static VcResult diag_wirecap_glitch_set(VcCtx *c) {
    return wirecap_vc(wirecap_clkgen_glitch(vc_in_u8(c, 0), vc_in_u8(c, 1)));
}

static VcResult diag_wirecap_rate_set(VcCtx *c) {
    return wirecap_vc(wirecap_request_rate(vc_in_u32(c, 0)));
}
#endif

#define VC_PAGE_DEF "pages/diag.def"
#include "vc_page_table.inc"
