/*
 * vc_bulk.c: VC_F_BULK routing for paged commands over the bulk_param_buf
 * lock. The lock itself and the page-0 chunk sessions still live in
 * vendor_commands.c, because a page-0 case body reads the owner directly.
 */

#include "vc_internal.h"
#include "usb_audio.h"
#include "bulk_params.h"

_Static_assert(WIRE_BULK_BUF_SIZE <= 0xFFFFu, "bulk length must fit a u16 wLength");

uint8_t *vc_bulk_buf(void) { return bulk_param_buf; }
uint16_t vc_bulk_cap(void) { return (uint16_t)WIRE_BULK_BUF_SIZE; }

// USB holds the lock from SETUP to the status ACK (vc_usb.c); the SETUP-stage
// cleanup in tud_vendor_control_xfer_cb drops it if the host abandons a transfer.
bool vc_bulk_usb_acquire(void) { return vendor_bulk_try_acquire(CTRL_SOURCE_USB); }
void vc_bulk_usb_release(void) { vendor_bulk_release(CTRL_SOURCE_USB); }

// PHASE 3 HOOK: UART and I2C stream bulk payloads only for page 0 until
// vendor_is_bulk_set() and their zero-copy reply paths become page-aware.
VcResult vc_bulk_ext_check(CtrlSource src, bool is_get, const VcCmd *e) {
    (void)is_get;
    if ((e->flags & VC_F_BULK) && src != CTRL_SOURCE_USB) return VC_ERR_BLOCKED;
    return VC_OK;
}
