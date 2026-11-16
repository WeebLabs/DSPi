/*
 * vc_page_diag.c: DIAG page (0xF0). Command set: vendor_command_overhaul_plan.md section 5.13.
 */

#include "vc_core.h"
#include "config.h"
#include "clock_diag.h"
#include "loopback.h"

#if DSPI_CLOCK_DIAG
// The packet outgrows the 132-byte UART and I2C reply cap, hence VC_F_USB_ONLY.
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

#define VC_PAGE_DEF "pages/diag.def"
#include "vc_page_table.inc"
