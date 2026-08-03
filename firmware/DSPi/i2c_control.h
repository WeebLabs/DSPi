/*
 * i2c_control.h - I2C target (slave) control transport for DSPi
 *
 * Exposes the whole vendor-command surface to an external I2C controller
 * (MCU, SBC) through the transport-neutral dispatcher in vendor_commands.c.
 * The device is target-only: it never initiates bus traffic.
 *
 * Wire format (all multi-byte fields little-endian):
 *
 *   Request (one I2C write transaction, 8-byte header + optional payload):
 *     [type][bReq][wValL][wValH][wIdxL][wIdxH][wLenL][wLenH][payload...]
 *     type 0x01 = SET (payload of wLen bytes follows the header)
 *     type 0x02 = GET (no payload; wLen caps the response size, 0 = full)
 *
 *   Response (I2C read transactions, resumable at byte granularity):
 *     [status][lenL][lenH][payload...]   then 0xFF padding beyond the end
 *     status is a CTRL_STATUS_* code; len is 0 unless status == OK on a GET.
 *
 * A read before the main loop has dispatched the request returns a BUSY
 * frame [0x01, 0, 0]; the controller retries the read (a repeated-START
 * write+read lands here on the first read, then succeeds on the retry).
 * The response survives across read transactions until fully consumed, so
 * controllers without repeated-START can read it in chunks.  A new request
 * write discards any unconsumed response.
 *
 * The ISR only moves bytes between the I2C FIFOs and module buffers; frame
 * dispatch runs from i2c_ctrl_poll() in the main loop.  Clock stretching is
 * bounded to ISR latency at FIFO boundaries; the device never stretches
 * while waiting on application state (that is what the BUSY frame is for).
 *
 * Interface self-configuration (REQ_SET_I2C_CONFIG / REQ_SET_UART_CONFIG)
 * is refused on this transport; it is reachable over USB only.
 */

#ifndef I2C_CONTROL_H
#define I2C_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

#define I2C_CTRL_TYPE_SET    0x01
#define I2C_CTRL_TYPE_GET    0x02
#define I2C_CTRL_HEADER_LEN  8
#define I2C_CTRL_RESP_HDR_LEN 3

// Boot-time bring-up from the persisted directory config.  Main loop only.
// Returns the PIN_CONFIG_* validation status.
uint8_t i2c_ctrl_init(const I2cCtrlConfig *cfg);

// Re-derives the IP's clk_sys-based timing counts; call after a sys-clock
// switch.  No-op when the interface is down.
void i2c_ctrl_reclock(void);

// Tear down, validate, reconfigure.  Returns PIN_CONFIG_* status; on
// validation failure the previous config is restored best-effort.
// Main-loop context only.
uint8_t i2c_ctrl_apply(const I2cCtrlConfig *cfg);

// Validation only, no side effects.  Returns PIN_CONFIG_* status.
uint8_t i2c_ctrl_validate(const I2cCtrlConfig *cfg);

// Main-loop tick: dispatch a completed request frame and park its response.
// Bounded: at most one dispatch per call.
void i2c_ctrl_poll(void);

// True iff the interface is live and `pin` is its SDA or SCL GPIO.
// Consulted by pin_used_by_fixed_peripheral() in vendor_commands.c.
bool i2c_ctrl_owns_pin(uint8_t pin);

// True when the peripheral is up and listening at the configured address.
bool i2c_ctrl_is_live(void);

#endif // I2C_CONTROL_H
