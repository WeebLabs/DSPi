/*
 * uart_control.h - UART control transport for DSPi
 *
 * Lets an external MCU drive the entire vendor-command surface over a UART.
 * The transport-neutral dispatcher (ctrl_dispatch_get/set) does the real
 * work; this module only frames bytes on the wire and pumps them.  Nothing
 * here ever blocks or busy-waits: the ISR moves bytes into a ring, and all
 * parsing, dispatch and TX happen in uart_ctrl_poll() from the main loop, so
 * the audio pipeline is never touched.
 *
 * Wire protocol (fixed 8N1, CRC-protected).  Every frame starts with the sync
 * byte 0xA5 followed by a type byte:
 *
 *   Request  SET : A5 01 bReq wValL wValH wIdxL wIdxH wLenL wLenH payload[wLen] crcL crcH
 *   Request  GET : A5 02 bReq wValL wValH wIdxL wIdxH wLenL wLenH crcL crcH
 *                  (no payload; wLen caps the response size, 0 = full)
 *   Response SET : A5 81 status lenL lenH crcL crcH                 (len always 0)
 *   Response GET : A5 82 status lenL lenH payload[len] crcL crcH    (payload only when status==OK)
 *   Notification : A5 40 00 lenL lenH packet[len] crcL crcH         (device-initiated)
 *
 * Notification frames carry the same v2 notify packet the USB interrupt
 * endpoint (EP 0x83) delivers, verbatim (see notify.h and the notification
 * protocol spec).  They are pushed only when UartCtrlConfig.notify_enable is
 * set and only at frame boundaries while the link is otherwise idle, so a
 * response is never split by one; clients must accept a 0x40 frame at any
 * point between frames.  A gap in the packets' sequence bytes means events
 * were dropped for this consumer; recover with a full STORE.ALL_PARAMS read.
 *
 * CRC is CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no final
 * XOR) computed over every byte after the sync byte (the type byte through the
 * end of the payload) and transmitted little-endian.  It is accumulated
 * incrementally byte-by-byte in both directions; there is never a lump-sum pass.
 *
 * The status byte is a CTRL_STATUS_* transport code or, for a request that
 * reached a command, its VcResult code (vendor/vc_ctrl.h).
 */

#ifndef UART_CONTROL_H
#define UART_CONTROL_H

#include "config.h"
#include <stdint.h>
#include <stdbool.h>

// Boot-time bring-up from the persisted config.  Same as uart_ctrl_apply but
// without rollback; call once before the main loop starts.  Returns the
// PIN_CONFIG_* validation status (recorded for XPORT.STATUS).
uint8_t uart_ctrl_init(const UartCtrlConfig *cfg);

// Reconfigure at runtime (main-loop context only).  Tears the current
// interface down first so a pin-in-use check does not see our own pins, then
// validates and, if valid and enabled, brings the new config up.  On a
// validation failure the previous config is restored best-effort.  Returns a
// PIN_CONFIG_* status code.
uint8_t uart_ctrl_apply(const UartCtrlConfig *cfg);

// Validate a config with no side effects.  Returns a PIN_CONFIG_* status code.
uint8_t uart_ctrl_validate(const UartCtrlConfig *cfg);

// Main-loop tick: drain the RX ring, parse frames, dispatch, pump TX.  Never
// blocks; safe to call every loop iteration.
void uart_ctrl_poll(void);

// The config last applied (its pins are claimed only while live). Returns live.
bool uart_ctrl_live_config(UartCtrlConfig *out);

// True iff the peripheral is up and listening.
bool uart_ctrl_is_live(void);

#endif // UART_CONTROL_H
