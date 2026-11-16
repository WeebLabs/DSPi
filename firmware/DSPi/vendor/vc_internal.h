/*
 * vc_internal.h: state shared between vendor_commands.c and vendor/ only.
 * Not for page files or transports.
 */

#ifndef VC_INTERNAL_H
#define VC_INTERNAL_H

#include <stdint.h>
#include <stdbool.h>
#include "tusb.h"
#include "vc_core.h"

#define VC_USB_RX_MAX 64u

// Defined in vendor_commands.c. The one EP0 SET receive buffer both USB paths use.
uint8_t *vendor_usb_rx_buf(void);

// Defined in vendor_commands.c. While a USB control transfer holds a shared
// buffer (EP0 receive, or a multi-packet reply), external dispatch answers BUSY.
void vendor_usb_busy_begin(void);
void vendor_usb_busy_end(void);
bool vendor_usb_busy(void);

// vc_usb.c: every stage of a vendor request whose page is not 0.
bool vc_usb_control_xfer(uint8_t rhport, uint8_t stage, tusb_control_request_t const *req);

// vc_usb.c: reboot into the bootloader once the current status stage completes.
void vc_usb_bootloader_after_status(void);

// vc_bulk.c: VC_F_BULK routing against the bulk_param_buf lock.
uint8_t *vc_bulk_buf(void);
uint16_t vc_bulk_cap(void);
bool     vc_bulk_usb_acquire(void);
void     vc_bulk_usb_release(void);
VcResult vc_bulk_ext_check(CtrlSource src, bool is_get, const VcCmd *e);

#endif // VC_INTERNAL_H
