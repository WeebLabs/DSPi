/*
 * vc_ctrl.h: vendor command entry points for the external control transports
 * (UART, I2C target) and internal callers, the shared bulk_param_buf lock and
 * the wire status codes. Transport spec: Documentation/Features/control_interfaces_spec.md.
 */

#ifndef VC_CTRL_H
#define VC_CTRL_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    CTRL_SOURCE_USB  = 0,
    CTRL_SOURCE_UART = 1,
    CTRL_SOURCE_I2C  = 2,
    CTRL_SOURCE_GPIO = 3,   // Control Surfaces engine (main loop), not a wire transport
} CtrlSource;

// Command results (plan section 3.3). On UART and I2C a paged request's status
// byte carries the code itself; USB hosts read it from SYS.LAST_RESULT.
typedef enum {
    VC_OK               = 0x00,
    VC_ERR_UNKNOWN      = 0x10,
    VC_ERR_DIRECTION    = 0x11,
    VC_ERR_LENGTH       = 0x12,
    VC_ERR_ADDRESS      = 0x13,
    VC_ERR_VALUE        = 0x14,
    VC_ERR_UNSUPPORTED  = 0x15,
    VC_ERR_BLOCKED      = 0x16,
    VC_ERR_BUSY         = 0x17,
    VC_ERR_STATE        = 0x18,
    VC_ERR_PIN_INVALID  = 0x19,
    VC_ERR_PIN_IN_USE   = 0x1A,
} VcResult;

// Largest non-bulk GET reply on any transport; the UART and I2C copy buffers match it.
#define VC_REPLY_MAX 256u

// Transport-level status codes, for conditions where the command never ran.
// They share byte 0 of every UART/I2C response with the VcResult codes above.
#define CTRL_STATUS_OK           0x00
#define CTRL_STATUS_BUSY         0x01  // a USB transfer holds the shared buffers; retry
#define CTRL_STATUS_ERROR        0x02  // page 0 only: legacy refusal
#define CTRL_STATUS_BLOCKED      0x03  // page 0 only: legacy USB-only refusal
#define CTRL_STATUS_BULK_LOCKED  0x04  // bulk buffer owned by another transport; retry
#define CTRL_STATUS_CRC_ERROR    0x05  // UART frame failed CRC16
#define CTRL_STATUS_OVERSIZE     0x06  // payload larger than the transport can buffer
#define CTRL_STATUS_FRAME_ERROR  0x07  // malformed or truncated frame

// Worth retrying the same request after a short wait.
static inline bool ctrl_status_transient(uint8_t st) {
    return st == CTRL_STATUS_BUSY || st == CTRL_STATUS_BULK_LOCKED || st == VC_ERR_BUSY;
}

// UART and I2C dispatch, main loop only. Returns the wire status byte. On OK the
// GET reply at *resp (capped to wLength, 0 = uncapped) is valid only until the next
// dispatch from ANY transport. A ctrl_is_bulk_get() reply lives in bulk_param_buf
// and holds src's bulk lock until the transport has streamed it and releases it.
uint8_t ctrl_dispatch_get(CtrlSource src, uint8_t bRequest, uint16_t wValue,
                          uint16_t wIndex, uint16_t wLength,
                          const uint8_t **resp, uint16_t *resp_len);

// A SET for which vendor_is_bulk_set() is true must already sit in bulk_param_buf
// under src's lock (payload ignored); on OK the main-loop apply owns the buffer
// and the lock is released. Any other payload is at most 64 bytes.
uint8_t ctrl_dispatch_set(CtrlSource src, uint8_t bRequest, uint16_t wValue,
                          uint16_t wIndex, const uint8_t *payload, uint16_t wLength);

// Bulk requests the transports stream through bulk_param_buf. Both are tiny and
// table-free: vendor_is_bulk_set() runs in the I2C target ISR.
bool vendor_is_bulk_set(uint16_t wIndex, uint8_t bRequest, uint16_t wLength);
bool ctrl_is_bulk_get(uint16_t wIndex, uint8_t bRequest);

// bulk_param_buf ownership across USB, UART and I2C. External owners go stale
// after 500 ms without vendor_bulk_touch(), a USB owner after 3 s. IRQ-safe.
bool vendor_bulk_try_acquire(CtrlSource src);
void vendor_bulk_release(CtrlSource src);
void vendor_bulk_touch(CtrlSource src);

// SYS.ENTER_BOOTLOADER over UART or I2C: the transport reboots once its reply is out.
void ctrl_reboot_request(CtrlSource src);
bool ctrl_reboot_pending(CtrlSource src);

// ---- Coarse results for internal callers still on the page-0 opcodes ----
typedef enum {
    CTRL_DISPATCH_OK          = 0,
    CTRL_DISPATCH_ERROR       = 1,
    CTRL_DISPATCH_BLOCKED     = 2,
    CTRL_DISPATCH_BULK_LOCKED = 3,
    CTRL_DISPATCH_BUSY        = 4,
} CtrlDispatchResult;

// ctrl_dispatch_get/set with the status folded to a CtrlDispatchResult.
CtrlDispatchResult vendor_dispatch_get(CtrlSource src, uint8_t bRequest,
                                       uint16_t wValue, uint16_t wIndex, uint16_t wLength,
                                       const uint8_t **resp_data, uint16_t *resp_len);
CtrlDispatchResult vendor_dispatch_set(CtrlSource src, uint8_t bRequest,
                                       uint16_t wValue, uint16_t wIndex,
                                       const uint8_t *payload, uint16_t wLength);

uint8_t ctrl_status_from_dispatch(CtrlDispatchResult r);
CtrlDispatchResult ctrl_dispatch_from_status(uint8_t st);

#endif // VC_CTRL_H
