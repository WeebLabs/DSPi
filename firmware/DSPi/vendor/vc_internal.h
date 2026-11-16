/*
 * vc_internal.h: state shared between the vendor/ core files only.
 * Not for page files or transports.
 */

#ifndef VC_INTERNAL_H
#define VC_INTERNAL_H

#include <stdint.h>
#include <stdbool.h>
#include "tusb.h"
#include "vc_core.h"

#define VC_USB_RX_MAX 64u   // EP0 receive buffer for a non-bulk SET (vc_usb.c)

// vc_core.c. While a USB control transfer holds a shared buffer (EP0 receive,
// or a multi-packet reply), external dispatch answers BUSY.
void vendor_usb_busy_begin(void);
void vendor_usb_busy_end(void);
bool vendor_usb_busy(void);

// vc_usb.c: reboot into the bootloader once the current status stage completes.
void vc_usb_bootloader_after_status(void);

// vc_bulk.c. True while USB owns the bulk_param_buf lock.
bool vendor_bulk_usb_holds(void);

// vc_bulk.c: VC_F_BULK routing against the bulk_param_buf lock.
uint8_t *vc_bulk_buf(void);
uint16_t vc_bulk_cap(void);
bool     vc_bulk_usb_acquire(void);
void     vc_bulk_usb_release(void);
VcResult vc_bulk_ext_check(CtrlSource src, uint16_t id, const VcCmd *e);

// vc_bulk.c: STORE.ALL_PARAMS over UART and I2C. They return the wire status
// byte, since losing the bulk lock is a transport condition, not a command result.
uint8_t vc_bulk_ext_get(CtrlSource src, uint8_t page, uint8_t cmd, uint16_t addr,
                        const uint8_t **out, uint16_t *out_len);
uint8_t vc_bulk_ext_set(CtrlSource src, uint8_t page, uint8_t cmd, uint16_t addr,
                        uint16_t len);

// vc_bulk.c: USB stages of a VC_F_BULK request. setup checks and takes the lock,
// window is where the payload or reply lives, failed and ack drop it.
VcResult vc_bulk_usb_setup(uint16_t id, bool is_get, uint16_t wValue, uint16_t wLength);
uint8_t *vc_bulk_usb_window(uint16_t id, uint16_t wValue, uint16_t wLength, uint16_t *cap);
void     vc_bulk_usb_failed(uint16_t id, bool is_get, bool at_setup);
void     vc_bulk_usb_ack(uint16_t id, bool is_get);

// vc_bulk.c: STORE handlers' bulk work. The check mirrors bulk_params_apply()'s header gate.
VcResult vc_bulk_params_check(const uint8_t *buf);
VcResult vc_bulk_chunk_get(VcCtx *c);
VcResult vc_bulk_chunk_set(VcCtx *c);

// vc_bulk.c: USB chunk sessions. continues() tells the SETUP cleanup to keep one open.
void usb_chunk_sessions_close(void);
bool vc_bulk_chunk_continues(uint16_t id, bool is_get);

#endif // VC_INTERNAL_H
