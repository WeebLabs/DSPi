/*
 * vendor_commands.h — Vendor USB control request handlers for DSPi
 *
 * Phase 2: adapted for TinyUSB.  The previous pico-extras `struct usb_interface`
 * / `struct usb_setup_packet` API is replaced by TinyUSB's stage-based
 * `tud_control_xfer` flow.  The DSPi UAC1 class driver (`usb_audio.c`) routes
 * vendor-class vendor-interface requests here via `vendor_control_xfer_cb`.
 */

#ifndef VENDOR_COMMANDS_H
#define VENDOR_COMMANDS_H

#include "config.h"
#include <stdint.h>
#include <stdbool.h>
#include "hardware/vreg.h"
#include "tusb.h"

// Overrides TinyUSB's weak tud_vendor_control_xfer_cb (usbd.c:82).  TinyUSB
// routes ALL vendor-type control transfers here directly (usbd.c:727-730),
// bypassing class drivers, regardless of wIndex/recipient.
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *req);

// ----------------------------------------------------------------------------
// Transport-neutral dispatch (orchestrator)
//
// The vendor command surface is reachable from USB (TinyUSB EP0 path above)
// and from the external control transports (UART, I2C target).  External
// transports parse their wire frames into the same bRequest/wValue/wIndex/
// wLength shape and call vendor_dispatch_get/set from MAIN-LOOP context only.
// New commands added to the SET/GET switches are automatically reachable
// from every transport; nothing per-transport to implement.
// ----------------------------------------------------------------------------

typedef enum {
    CTRL_SOURCE_USB  = 0,
    CTRL_SOURCE_UART = 1,
    CTRL_SOURCE_I2C  = 2,
    CTRL_SOURCE_GPIO = 3,   // internal Control Surfaces engine (main loop); not a
                            // wire transport.  The Control Surfaces tick dispatches
                            // through vendor_dispatch_get/set with this source so its
                            // writes are tagged PARAM_SRC_GPIO.
} CtrlSource;

typedef enum {
    CTRL_DISPATCH_OK          = 0,  // handled; GET response captured
    CTRL_DISPATCH_ERROR       = 1,  // handler rejected (USB equivalent: STALL)
    CTRL_DISPATCH_BLOCKED     = 2,  // USB-only command refused on external transport
    CTRL_DISPATCH_BULK_LOCKED = 3,  // bulk_param_buf owned by another transport
    CTRL_DISPATCH_BUSY        = 4,  // USB control SET in flight; retry next poll
} CtrlDispatchResult;

// The page is the wIndex high byte (vc_page_of).  Page 0 runs the legacy
// opcode switches; any other page goes to vc_dispatch() in vendor/vc_core.c.

// Dispatch a GET from an external transport.  wLength caps the response size
// (0 = uncapped).  On CTRL_DISPATCH_OK, *resp_data/*resp_len point at the
// response; the storage is static and stays valid until the next dispatch
// from ANY transport, so consume or copy before returning to the poll loop.
// A REQ_GET_ALL_PARAMS response points into bulk_param_buf and holds the bulk
// lock; the caller MUST vendor_bulk_release() when done streaming it.
CtrlDispatchResult vendor_dispatch_get(CtrlSource src, uint8_t bRequest,
                                       uint16_t wValue, uint16_t wIndex, uint16_t wLength,
                                       const uint8_t **resp_data, uint16_t *resp_len);

// Dispatch a SET from an external transport.  payload/wLength <= 64 bytes,
// except REQ_SET_ALL_PARAMS where the caller must already hold the bulk lock
// and have streamed the payload into bulk_param_buf (payload arg ignored);
// on OK the apply is handed to the main loop and the lock is released.
CtrlDispatchResult vendor_dispatch_set(CtrlSource src, uint8_t bRequest,
                                       uint16_t wValue, uint16_t wIndex,
                                       const uint8_t *payload, uint16_t wLength);

// bulk_param_buf ownership.  Serializes USB / UART / I2C use of the single
// shared bulk buffer.  External owners go stale after 500 ms unless they
// refresh with vendor_bulk_touch() while actively streaming; a dead USB
// owner (cable pull mid-EP0) is reaped after 3 s.  IRQ-safe.
bool vendor_bulk_try_acquire(CtrlSource src);
void vendor_bulk_release(CtrlSource src);
void vendor_bulk_touch(CtrlSource src);

// True if (bRequest, wLength) is a bulk SET whose payload belongs in
// bulk_param_buf.  Single definition shared by USB and both transports.
bool vendor_is_bulk_set(uint8_t bRequest, uint16_t wLength);

// Map a dispatch result to the CTRL_STATUS_* wire code (shared by UART/I2C).
uint8_t ctrl_status_from_dispatch(CtrlDispatchResult r);

// Wire-level status codes shared by the external transports (byte 0 of every
// UART/I2C response frame).
#define CTRL_STATUS_OK           0x00
#define CTRL_STATUS_BUSY         0x01  // retry the whole request shortly
#define CTRL_STATUS_ERROR        0x02  // unknown command or bad parameter
#define CTRL_STATUS_BLOCKED      0x03  // USB-only command (interface self-config)
#define CTRL_STATUS_BULK_LOCKED  0x04  // bulk buffer busy; retry
#define CTRL_STATUS_CRC_ERROR    0x05  // UART frame failed CRC16
#define CTRL_STATUS_OVERSIZE     0x06  // non-bulk payload exceeded 64 bytes
#define CTRL_STATUS_FRAME_ERROR  0x07  // malformed or truncated frame

// System statistics helpers
uint16_t vreg_voltage_to_mv(enum vreg_voltage voltage);
int16_t read_temperature_cdeg(void);

// Pin validation helpers (vendor/vc_pins.c)
#include "vc_pins.h"

// MCK encode/decode for wire and flash persistence
uint8_t  mck_encode(uint16_t val);
uint16_t mck_decode(uint8_t raw);

// (Removed: is_mck_multiplier_supported_for_rate / sanitize_mck_multiplier_for_rate.
//  CLK_GPOUTn-driven MCK no longer needs the 96 kHz × 256× clamp — see the
//  comment block above mck_encode in vendor_commands.c.)

// Ring buffer accessor (audio_ring is static in usb_audio.c)
uint32_t usb_audio_ring_overrun_count(void);

#endif // VENDOR_COMMANDS_H
