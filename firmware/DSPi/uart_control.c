/*
 * uart_control.c - UART control transport for DSPi
 *
 * See uart_control.h for the wire protocol.  Design rule: nothing here ever
 * blocks or busy-waits.  The exclusive UART IRQ only drains the RX FIFO into a
 * ring; every parse, dispatch and TX byte is produced from uart_ctrl_poll() in
 * main-loop context, so the real-time audio path is never delayed.
 */

#include "uart_control.h"
#include "vc_ctrl.h"
#include "vc_pins.h"
#include "usb_audio.h"
#include "bulk_params.h"
#include "notify.h"
#include "config.h"

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "pico/bootrom.h"
#include <string.h>

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

#define UART_SYNC            0xA5u
#define UART_TYPE_SET        0x01u
#define UART_TYPE_GET        0x02u
#define UART_TYPE_NOTIFY     0x40u  // device->host async notification frame
#define UART_RESP_SET        0x81u
#define UART_RESP_GET        0x82u

#define RX_RING_SIZE         256u                 // power of two
#define RX_RING_MASK         (RX_RING_SIZE - 1u)
#define PAYLOAD_MAX          64u                  // non-bulk SET payload cap
#define TX_COPY_MAX          VC_REPLY_MAX         // GET response copy buffer

#define FRAME_TIMEOUT_US     100000u              // inter-byte mid-frame timeout
#define DISPATCH_RETRY_US    50000u               // transient-status retry window

// ---------------------------------------------------------------------------
// Peripheral / live-config state
// ---------------------------------------------------------------------------

static uart_inst_t   *g_uart = NULL;
static uint           g_irq = 0;
static bool           g_irq_installed = false;
static bool           g_is_live = false;
static UartCtrlConfig g_live;

// ---------------------------------------------------------------------------
// RX ring (single-producer ISR, single-consumer poll)
// ---------------------------------------------------------------------------

static volatile uint8_t  rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head = 0;   // written by ISR only
static volatile uint16_t rx_tail = 0;   // written by consumer only
static volatile uint32_t rx_overrun = 0;

// ---------------------------------------------------------------------------
// Parser FSM state
// ---------------------------------------------------------------------------

typedef enum {
    RX_WAIT_SYNC = 0,
    RX_TYPE,
    RX_HEADER,   // 7 header bytes
    RX_PAYLOAD,  // SET only
    RX_CRC,      // 2 CRC bytes
} RxState;

static RxState  rx_state = RX_WAIT_SYNC;
static bool     rx_is_get = false;
static uint8_t  hdr[7];
static uint8_t  hdr_pos = 0;
static uint8_t  bReq = 0;
static uint16_t wValue = 0, wIndex = 0, wLen = 0;
static uint16_t payload_pos = 0;
static bool     discard = false;         // consume+CRC without storing
static bool     to_bulk = false;         // stream payload into bulk_param_buf
static uint8_t  deferred_status = CTRL_STATUS_OK;
static uint16_t rx_crc = 0xFFFF;
static uint8_t  crc_pos = 0;
static uint8_t  crc_rx_lo = 0;
static bool     parser_holds_bulk = false;  // parser owns the bulk lock

static uint8_t  payload_buf[PAYLOAD_MAX];
static uint8_t  tx_copy[TX_COPY_MAX];
// Notification packet staging: decoupled from tx_copy so response and
// notification lifetimes can never entangle.  Sized for the largest v2
// notify packet (12 + 52).
static uint8_t  notify_pkt[64];
static uint32_t last_byte_time = 0;

// ---------------------------------------------------------------------------
// Pending dispatch (one request in flight)
// ---------------------------------------------------------------------------

static struct {
    bool     active;
    bool     is_get;
    uint8_t  bReq;
    uint16_t wValue, wIndex, wLen;
    uint32_t retry_start;
} pending;

// ---------------------------------------------------------------------------
// TX state machine (pumped only from poll)
// ---------------------------------------------------------------------------

typedef enum {
    TX_SYNC = 0, TX_TYPE, TX_STATUS, TX_LEN_L, TX_LEN_H,
    TX_PAYLOAD, TX_CRC_L, TX_CRC_H,
} TxState;

static struct {
    bool           active;
    bool           bulk;      // payload is bulk_param_buf, holds the bulk lock
    TxState        state;
    uint8_t        type;
    uint8_t        status;
    uint16_t       len;
    uint16_t       pos;
    uint16_t       crc;
    const uint8_t *payload;
} tx;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// CRC16-CCITT-FALSE, one byte at a time.  Table-driven: a bulk SET at
// 1 Mbaud spends ~50 cycles/byte on the bitwise form, which adds up inside
// the main loop; the 512-byte table lives in .rodata (flash XIP on RP2040).
static uint16_t crc16_step(uint16_t crc, uint8_t b) {
    static const uint16_t crc16_table[256] = {
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
        0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
        0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
        0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
        0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
        0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
        0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
        0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
        0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
        0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
        0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
        0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
        0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
        0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
        0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
        0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
        0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
        0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
        0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
        0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
        0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
        0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
        0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
        0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
        0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
        0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
        0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
        0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
        0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
        0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
        0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
    };
    return (uint16_t)((crc << 8) ^ crc16_table[(crc >> 8) ^ b]);
}

// UARTx instance index for a GPIO from its pin-mux position: 0 = uart0, 1 = uart1.
static inline uint8_t uart_instance_index(uint8_t pin) {
    return (uint8_t)((((pin >> 2) + 1) >> 1) & 1);
}

static void release_parser_bulk(void) {
    if (parser_holds_bulk) {
        vendor_bulk_release(CTRL_SOURCE_UART);
        parser_holds_bulk = false;
    }
}

// Reset all parser/TX state; release any bulk lock we hold (parser or TX).
static void reset_proto_state(void) {
    release_parser_bulk();
    if (tx.bulk) {
        vendor_bulk_release(CTRL_SOURCE_UART);
        tx.bulk = false;
    }
    rx_state = RX_WAIT_SYNC;
    hdr_pos = 0;
    payload_pos = 0;
    discard = false;
    to_bulk = false;
    crc_pos = 0;
    pending.active = false;
    tx.active = false;
    rx_head = 0;
    rx_tail = 0;
}

// Abort a partially-received frame on inter-byte timeout; send nothing.
static void abort_frame(void) {
    release_parser_bulk();
    rx_state = RX_WAIT_SYNC;
    hdr_pos = 0;
    payload_pos = 0;
    discard = false;
    to_bulk = false;
    crc_pos = 0;
}

// Arm the TX state machine.  Payload is emitted only when status == OK.
static void start_tx(uint8_t type, uint8_t status,
                     const uint8_t *payload, uint16_t len, bool bulk) {
    tx.type    = type;
    tx.status  = status;
    tx.len     = (status == CTRL_STATUS_OK) ? len : 0;
    tx.payload = payload;
    tx.bulk    = bulk;
    tx.crc     = 0xFFFF;
    tx.state   = TX_SYNC;
    tx.pos     = 0;
    tx.active  = true;
}

static inline void start_tx_status(uint8_t type, uint8_t status) {
    start_tx(type, status, NULL, 0, false);
}

// ---------------------------------------------------------------------------
// RX IRQ: drain the FIFO into the ring (never blocks, never parses)
// ---------------------------------------------------------------------------

static void uart_ctrl_irq(void) {
    while (uart_is_readable(g_uart)) {
        uint8_t b = (uint8_t)uart_get_hw(g_uart)->dr;
        uint16_t next = (uint16_t)((rx_head + 1) & RX_RING_MASK);
        if (next == rx_tail) { rx_overrun++; continue; }  // full: drop
        rx_ring[rx_head] = b;
        rx_head = next;
    }
}

static int ring_pop(void) {
    if (rx_tail == rx_head) return -1;
    uint8_t b = rx_ring[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1) & RX_RING_MASK);
    return (int)b;
}

// ---------------------------------------------------------------------------
// Frame parsing
// ---------------------------------------------------------------------------

// SET header parsed: decide the payload destination (bulk / normal / discard).
static void header_complete(void) {
    bReq   = hdr[0];
    wValue = (uint16_t)hdr[1] | ((uint16_t)hdr[2] << 8);
    wIndex = (uint16_t)hdr[3] | ((uint16_t)hdr[4] << 8);
    wLen   = (uint16_t)hdr[5] | ((uint16_t)hdr[6] << 8);
    discard = false;
    to_bulk = false;
    payload_pos = 0;
    deferred_status = CTRL_STATUS_OK;

    if (rx_is_get) { crc_pos = 0; rx_state = RX_CRC; return; }  // GET: no payload
    if (wLen == 0) { crc_pos = 0; rx_state = RX_CRC; return; }

    if (vendor_is_bulk_set(wIndex, bReq, wLen)) {
        if (vendor_bulk_try_acquire(CTRL_SOURCE_UART)) {
            parser_holds_bulk = true;
            to_bulk = true;                    // stream straight into bulk_param_buf
        } else {
            discard = true;                    // buffer busy; answer after the frame
            deferred_status = CTRL_STATUS_BULK_LOCKED;
        }
    } else if (wLen > PAYLOAD_MAX) {
        discard = true;
        deferred_status = CTRL_STATUS_OVERSIZE;
    }
    rx_state = RX_PAYLOAD;
}

// A full frame arrived; `good` is the CRC verdict.  Either respond directly
// (errors / discarded frames) or hand the request to the dispatch retry loop.
static void frame_complete(bool good) {
    uint8_t resp_type = rx_is_get ? UART_RESP_GET : UART_RESP_SET;

    if (!good) {
        release_parser_bulk();
        discard = false;
        start_tx_status(resp_type, CTRL_STATUS_CRC_ERROR);
        return;
    }
    if (discard) {
        // Nothing was acquired (bulk-locked never acquired; oversize is non-bulk).
        start_tx_status(resp_type, deferred_status);
        discard = false;
        return;
    }
    // Good CRC on a real frame: dispatch it (retried across polls if transient).
    pending.is_get      = rx_is_get;
    pending.bReq        = bReq;
    pending.wValue      = wValue;
    pending.wIndex      = wIndex;
    pending.wLen        = wLen;
    pending.retry_start = time_us_32();
    pending.active      = true;
}

static void feed_byte(uint8_t b) {
    last_byte_time = time_us_32();
    switch (rx_state) {
        case RX_WAIT_SYNC:
            if (b == UART_SYNC) rx_state = RX_TYPE;
            break;

        case RX_TYPE:
            if (b == UART_TYPE_SET || b == UART_TYPE_GET) {
                rx_is_get = (b == UART_TYPE_GET);
                rx_crc = crc16_step(0xFFFF, b);
                hdr_pos = 0;
                rx_state = RX_HEADER;
            } else {
                rx_state = RX_WAIT_SYNC;  // noise (includes reserved 0x40): resync
            }
            break;

        case RX_HEADER:
            hdr[hdr_pos++] = b;
            rx_crc = crc16_step(rx_crc, b);
            if (hdr_pos == 7) header_complete();
            break;

        case RX_PAYLOAD:
            rx_crc = crc16_step(rx_crc, b);
            if (!discard) {
                if (to_bulk) bulk_param_buf[payload_pos] = b;
                else         payload_buf[payload_pos] = b;
            }
            payload_pos++;
            if (payload_pos == wLen) { crc_pos = 0; rx_state = RX_CRC; }
            break;

        case RX_CRC:
            if (crc_pos == 0) {
                crc_rx_lo = b;
                crc_pos = 1;
            } else {
                uint16_t rec = (uint16_t)crc_rx_lo | ((uint16_t)b << 8);
                rx_state = RX_WAIT_SYNC;
                frame_complete(rec == rx_crc);
            }
            break;
    }
}

// Consume ring bytes until a frame completes (sets pending/tx), the ring
// empties, or the per-poll byte budget runs out.  Only called when no
// response is transmitting and none is pending.  The cap bounds worst-case
// parse time per main-loop iteration (a full 256-byte ring after a stall
// would otherwise parse in one burst right when the loop is catching up).
static void parse_ring(void) {
    int c;
    int budget = 128;
    while (budget-- > 0 && !tx.active && !pending.active &&
           (c = ring_pop()) >= 0)
        feed_byte((uint8_t)c);
}

// ---------------------------------------------------------------------------
// Dispatch
// ---------------------------------------------------------------------------

static void try_dispatch(void) {
    uint8_t st;
    const uint8_t *rd = NULL;
    uint16_t rl = 0;

    if (pending.is_get) {
        st = ctrl_dispatch_get(CTRL_SOURCE_UART, pending.bReq, pending.wValue,
                               pending.wIndex, pending.wLen, &rd, &rl);
    } else {
        // A bulk SET payload already sits in bulk_param_buf under our lock.
        st = ctrl_dispatch_set(CTRL_SOURCE_UART, pending.bReq, pending.wValue,
                               pending.wIndex, payload_buf, pending.wLen);
    }

    // Transient results retry internally for a short window before giving up.
    if (ctrl_status_transient(st) &&
        (uint32_t)(time_us_32() - pending.retry_start) < DISPATCH_RETRY_US)
        return;  // keep pending; retry on the next poll

    uint8_t resp_type = pending.is_get ? UART_RESP_GET : UART_RESP_SET;

    // Bulk SET lock: the dispatcher releases it on OK; we release on any
    // other result (including a timed-out BUSY that never touched the lock).
    if (!pending.is_get) {
        if (parser_holds_bulk && st != CTRL_STATUS_OK)
            vendor_bulk_release(CTRL_SOURCE_UART);
        parser_holds_bulk = false;
    }

    pending.active = false;

    if (pending.is_get && st == CTRL_STATUS_OK) {
        if (ctrl_is_bulk_get(pending.wIndex, pending.bReq)) {
            // Zero-copy stream from bulk_param_buf; the dispatcher handed us the
            // bulk lock, released when the last CRC byte goes out.
            start_tx(resp_type, CTRL_STATUS_OK, rd, rl, true);
        } else {
            // The dispatcher's reply is only valid until the next dispatch from
            // any transport, so snapshot it now. It never exceeds TX_COPY_MAX.
            uint16_t n = (rl > TX_COPY_MAX) ? TX_COPY_MAX : rl;
            memcpy(tx_copy, rd, n);
            start_tx(resp_type, CTRL_STATUS_OK, tx_copy, n, false);
        }
    } else {
        start_tx_status(resp_type, st);
    }
}

// ---------------------------------------------------------------------------
// TX pump (main-loop only; never waits on the FIFO)
// ---------------------------------------------------------------------------

static void pump_tx(void) {
    while (tx.active && uart_is_writable(g_uart)) {
        uint8_t out;
        switch (tx.state) {
            case TX_SYNC:
                out = UART_SYNC;  // sync byte is outside the CRC
                tx.state = TX_TYPE;
                break;
            case TX_TYPE:
                out = tx.type;    tx.crc = crc16_step(tx.crc, out); tx.state = TX_STATUS; break;
            case TX_STATUS:
                out = tx.status;  tx.crc = crc16_step(tx.crc, out); tx.state = TX_LEN_L; break;
            case TX_LEN_L:
                out = (uint8_t)(tx.len & 0xFF); tx.crc = crc16_step(tx.crc, out); tx.state = TX_LEN_H; break;
            case TX_LEN_H:
                out = (uint8_t)(tx.len >> 8);   tx.crc = crc16_step(tx.crc, out);
                tx.pos = 0;
                tx.state = tx.len ? TX_PAYLOAD : TX_CRC_L;
                break;
            case TX_PAYLOAD:
                out = tx.payload[tx.pos];  tx.crc = crc16_step(tx.crc, out); tx.pos++;
                if (tx.pos == tx.len) tx.state = TX_CRC_L;
                break;
            case TX_CRC_L:
                out = (uint8_t)(tx.crc & 0xFF); tx.state = TX_CRC_H; break;
            case TX_CRC_H:
                out = (uint8_t)(tx.crc >> 8);
                uart_get_hw(g_uart)->dr = out;   // last byte of the frame
                tx.active = false;
                if (tx.bulk) { vendor_bulk_release(CTRL_SOURCE_UART); tx.bulk = false; }
                continue;
            default:
                tx.active = false;
                continue;
        }
        uart_get_hw(g_uart)->dr = out;
    }
}

// ---------------------------------------------------------------------------
// Bring-up / teardown
// ---------------------------------------------------------------------------

static void up(void) {
    uint8_t tx_pin = g_live.tx_pin;
    uint8_t rx_pin = g_live.rx_pin;
    uint8_t idx = uart_instance_index(tx_pin);
    g_uart = idx ? uart1 : uart0;
    g_irq  = idx ? UART1_IRQ : UART0_IRQ;

    uart_init(g_uart, g_live.baud);
    uart_set_format(g_uart, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(g_uart, false, false);
    uart_set_fifo_enabled(g_uart, true);
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    gpio_pull_up(rx_pin);   // keep RX idle-high when no external MCU is attached

    while (uart_is_readable(g_uart)) (void)uart_get_hw(g_uart)->dr;  // drain stale RX

    reset_proto_state();

    irq_set_exclusive_handler(g_irq, uart_ctrl_irq);
    irq_set_priority(g_irq, 0xC0);
    irq_set_enabled(g_irq, true);
    g_irq_installed = true;
    uart_set_irq_enables(g_uart, true, false);  // RX + RX-timeout, no TX IRQ

    g_is_live = true;
    notify_consumer_set_active(NOTIFY_CONSUMER_UART,
                               g_live.notify_enable != 0);
}

static void down(void) {
    notify_consumer_set_active(NOTIFY_CONSUMER_UART, false);
    if (g_irq_installed) {
        uart_set_irq_enables(g_uart, false, false);
        irq_set_enabled(g_irq, false);
        irq_remove_handler(g_irq, uart_ctrl_irq);
        g_irq_installed = false;
    }
    if (g_uart) uart_deinit(g_uart);

    gpio_set_function(g_live.tx_pin, GPIO_FUNC_NULL);
    gpio_disable_pulls(g_live.tx_pin);
    gpio_set_function(g_live.rx_pin, GPIO_FUNC_NULL);
    gpio_disable_pulls(g_live.rx_pin);

    reset_proto_state();
    g_is_live = false;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

uint8_t uart_ctrl_validate(const UartCtrlConfig *cfg) {
    if (!cfg) return PIN_CONFIG_INVALID_PARAM;
    if (cfg->enabled > 1) return PIN_CONFIG_INVALID_PARAM;
    if (cfg->notify_enable > 1) return PIN_CONFIG_INVALID_PARAM;
    if (cfg->baud < UART_CTRL_BAUD_MIN || cfg->baud > UART_CTRL_BAUD_MAX)
        return PIN_CONFIG_INVALID_PARAM;
    // Mux rules hold even while disabled; ownership only once enabled (vc_pins.c).
    PinRequest rq = { PIN_FN_UART_CTRL, 0, { cfg->tx_pin, cfg->rx_pin } };
    return pin_check(pin_state_live(), &rq, cfg->enabled ? PIN_CHECK_CLAIM : PIN_CHECK_STORE);
}

// Returns the validation status so boot code can record it for
// XPORT.STATUS without validating twice.
uint8_t uart_ctrl_init(const UartCtrlConfig *cfg) {
    g_irq_installed = false;
    g_is_live = false;
    g_uart = NULL;
    tx.bulk = false;
    tx.active = false;
    parser_holds_bulk = false;
    reset_proto_state();

    if (!cfg) return PIN_CONFIG_INVALID_PARAM;
    g_live = *cfg;

    uint8_t st = uart_ctrl_validate(cfg);
    if (st == PIN_CONFIG_SUCCESS && cfg->enabled) up();
    return st;
}

uint8_t uart_ctrl_apply(const UartCtrlConfig *cfg) {
    if (!cfg) return PIN_CONFIG_INVALID_PARAM;

    UartCtrlConfig prev = g_live;
    bool prev_up = g_is_live;

    if (g_is_live) down();  // free our own pins before validating the new config

    uint8_t st = uart_ctrl_validate(cfg);
    if (st == PIN_CONFIG_SUCCESS) {
        g_live = *cfg;
        if (cfg->enabled) up();
        return st;
    }

    // Validation failed: best-effort restore the previous live config.
    if (prev_up) {
        g_live = prev;
        up();
    }
    return st;
}

void uart_ctrl_poll(void) {
    if (!g_is_live) return;

    // Keep the bulk lock fresh whenever we hold it (receiving, pending, or TX).
    if (parser_holds_bulk || tx.bulk)
        vendor_bulk_touch(CTRL_SOURCE_UART);

    // Inter-byte timeout: drop a stalled partial frame, send nothing.
    if (rx_state != RX_WAIT_SYNC &&
        (uint32_t)(time_us_32() - last_byte_time) > FRAME_TIMEOUT_US)
        abort_frame();

    if (tx.active) pump_tx();

    // SYS.ENTER_BOOTLOADER: reboot once its reply has left the shift register.
    if (ctrl_reboot_pending(CTRL_SOURCE_UART)) {
        if (!tx.active && !(uart_get_hw(g_uart)->fr & UART_UARTFR_BUSY_BITS))
            reset_usb_boot(0, 0);
        return;
    }

    if (pending.active) try_dispatch();

    // Parse new bytes only when the transport is otherwise idle.
    if (!tx.active && !pending.active) parse_ring();

    // Notification frames go out only when the transport is STILL idle
    // after parsing, so request/response traffic always takes precedence
    // (a saturating requester can starve notifications; the ring's
    // force-drop plus the packet seq gap covers that, and the client
    // re-reads full state).  The packet is copied into notify_pkt by the
    // peek, so the ring entry is committed immediately; the TX pump then
    // streams the frame at FIFO pace with no blocking.
    if (g_live.notify_enable && !tx.active && !pending.active) {
        uint16_t n = notify_peek_next_for(NOTIFY_CONSUMER_UART,
                                          notify_pkt, sizeof(notify_pkt));
        if (n) {
            notify_commit_pop_for(NOTIFY_CONSUMER_UART);
            start_tx(UART_TYPE_NOTIFY, CTRL_STATUS_OK, notify_pkt, n, false);
            pump_tx();   // start pushing bytes this same poll
        }
    }
}

bool uart_ctrl_live_config(UartCtrlConfig *out) {
    *out = g_live;
    return g_is_live;
}

bool uart_ctrl_is_live(void) {
    return g_is_live;
}
