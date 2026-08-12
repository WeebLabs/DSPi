/*
 * i2c_control.c - I2C target (slave) control transport for DSPi
 *
 * Built on pico_i2c_slave (per-event ISR callbacks).  The event handler
 * only moves bytes; parsing state is a few counters, and the vendor-command
 * dispatch runs from i2c_ctrl_poll() in the main loop.  See i2c_control.h
 * for the wire format.
 */

#include "i2c_control.h"
#include "vendor_commands.h"
#include "usb_audio.h"
#include "bulk_params.h"
#include "control_surfaces.h"           // CsMacro sizes resp_small
#include "control_surfaces_display.h"   // display instance exclusion

#include <string.h>
#include "pico/stdlib.h"
#include "pico/i2c_slave.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"

// Below USB and the audio DMA IRQs; control traffic must never delay audio.
#define I2C_CTRL_IRQ_PRIORITY  0xC0

// How long poll() re-tries a dispatch that came back BUSY / BULK_LOCKED
// before parking that status for the controller to see.
#define I2C_CTRL_RETRY_US      50000u

// ----------------------------------------------------------------------------
// Module state
// ----------------------------------------------------------------------------

static I2cCtrlConfig live_cfg;      // current configuration (enabled may be 1
                                    // while `live` is false after a boot-time
                                    // pin collision)
static bool live = false;           // peripheral armed
static i2c_inst_t *inst = NULL;

// --- request accumulation (written by ISR) ---
static volatile uint8_t  rx_header[I2C_CTRL_HEADER_LEN];
static volatile uint8_t  rx_payload[64];
static volatile uint32_t rx_count;          // bytes received this transaction
static volatile bool     rx_bulk;           // payload streams into bulk_param_buf
static volatile bool     rx_discard;        // consume payload without storing
static volatile uint8_t  rx_discard_status; // status to park after a discarded frame
static volatile bool     frame_ready;       // complete frame awaiting dispatch

// --- parked response (written by poll, drained by ISR) ---
static uint8_t  resp_hdr[I2C_CTRL_RESP_HDR_LEN];
// Sized with _ext_resp_copy and the UART TX_COPY_MAX for the largest
// non-bulk GET (the 132-byte CsMacro); the three must move together.
static uint8_t  resp_small[sizeof(CsMacro)];   // copy of non-bulk GET payloads
static const uint8_t *resp_payload;         // resp_small or bulk_param_buf
static volatile uint16_t resp_len;
static volatile bool     resp_ready;
static volatile bool     resp_bulk;         // holding the bulk lock while parked
static volatile uint32_t tx_cursor;         // position in [hdr|payload] stream
static volatile uint32_t busy_cursor;       // position in the 3-byte BUSY frame

// --- frame under dispatch (poll context only) ---
// poll() takes ownership of a completed frame inside a critical section so
// the ISR's new-request cleanup can never release a bulk lock or rewrite a
// buffer the dispatch is still using.
static bool     cur_valid;          // a frame is held for (re-)dispatch
static uint8_t  cur_hdr[I2C_CTRL_HEADER_LEN];
static uint8_t  cur_payload[64];
static bool     cur_bulk;           // frame streamed into bulk_param_buf (lock held)
static bool     cur_discard;
static uint8_t  cur_discard_status;
static bool     retry_active;
static uint32_t retry_start;

// wLen from a header, little-endian.
static inline uint16_t hdr_wlen(const volatile uint8_t *h) {
    return (uint16_t)(h[6] | (h[7] << 8));
}

// ----------------------------------------------------------------------------
// ISR event handler.  Bounded work only: FIFO moves and counter updates.
// ----------------------------------------------------------------------------

static void park_response(uint8_t status, const uint8_t *payload,
                          uint16_t len, bool bulk);

// Drop any parked response (a new request supersedes it).
static void resp_drop_from_isr(void) {
    if (resp_ready && resp_bulk) vendor_bulk_release(CTRL_SOURCE_I2C);
    resp_ready = false;
    resp_bulk  = false;
    tx_cursor  = 0;
}

static void on_receive_byte(uint8_t b) {
    // First byte of a new request: discard any unconsumed response and any
    // undispatched previous frame (controller is expected to serialize).
    if (rx_count == 0) {
        resp_drop_from_isr();
        if (frame_ready && rx_bulk) vendor_bulk_release(CTRL_SOURCE_I2C);
        frame_ready = false;
        rx_bulk = false;
        rx_discard = false;
        rx_discard_status = CTRL_STATUS_OK;
    }

    if (rx_count < I2C_CTRL_HEADER_LEN) {
        rx_header[rx_count++] = b;
        if (rx_count == I2C_CTRL_HEADER_LEN) {
            // Header complete: choose where the payload goes.
            uint8_t  type = rx_header[0];
            uint8_t  breq = rx_header[1];
            uint16_t wlen = hdr_wlen(rx_header);
            if (type == I2C_CTRL_TYPE_SET && vendor_is_bulk_set(breq, wlen)) {
                if (vendor_bulk_try_acquire(CTRL_SOURCE_I2C)) {
                    rx_bulk = true;
                } else {
                    rx_discard = true;
                    rx_discard_status = CTRL_STATUS_BULK_LOCKED;
                }
            } else if (type == I2C_CTRL_TYPE_SET && wlen > sizeof(rx_payload)) {
                rx_discard = true;
                rx_discard_status = CTRL_STATUS_OVERSIZE;
            }
        }
        return;
    }

    uint32_t pidx = rx_count - I2C_CTRL_HEADER_LEN;
    if (rx_bulk) {
        if (pidx < sizeof(WireBulkParams)) bulk_param_buf[pidx] = b;
    } else if (!rx_discard && pidx < sizeof(rx_payload)) {
        rx_payload[pidx] = b;
    }
    rx_count++;
}

static void on_request(void) {
    // Batch-fill the TX FIFO; the hardware stretches SCL until the first
    // byte lands, so each RD_REQ costs one ISR latency at most.
    while (i2c_get_write_available(inst) > 0) {
        uint8_t b = 0xFF;   // padding beyond the frame end
        if (resp_ready) {
            uint32_t total = (uint32_t)I2C_CTRL_RESP_HDR_LEN + resp_len;
            if (tx_cursor < I2C_CTRL_RESP_HDR_LEN) {
                b = resp_hdr[tx_cursor];
            } else if (tx_cursor < total) {
                b = resp_payload[tx_cursor - I2C_CTRL_RESP_HDR_LEN];
            }
            if (tx_cursor < total) tx_cursor++;
        } else {
            // Nothing parked yet: structured BUSY frame, then padding.
            static const uint8_t busy_frame[I2C_CTRL_RESP_HDR_LEN] =
                { CTRL_STATUS_BUSY, 0, 0 };
            if (busy_cursor < sizeof(busy_frame)) b = busy_frame[busy_cursor++];
        }
        i2c_write_byte_raw(inst, b);
    }
}

static void on_finish(void) {
    busy_cursor = 0;

    // End of a write transaction: promote a complete frame for dispatch.
    if (rx_count > 0) {
        uint8_t  type = rx_header[0];
        uint16_t wlen = (rx_count >= I2C_CTRL_HEADER_LEN) ? hdr_wlen(rx_header) : 0;
        uint32_t expect = I2C_CTRL_HEADER_LEN +
                          ((type == I2C_CTRL_TYPE_SET) ? wlen : 0);
        if (rx_count == expect &&
            (type == I2C_CTRL_TYPE_SET || type == I2C_CTRL_TYPE_GET)) {
            frame_ready = true;
        } else {
            // Truncated, overlong, or unknown type: report and reset.
            if (rx_bulk) { vendor_bulk_release(CTRL_SOURCE_I2C); rx_bulk = false; }
            park_response(rx_discard ? rx_discard_status : CTRL_STATUS_FRAME_ERROR,
                          NULL, 0, false);
        }
        rx_count = 0;
    }

    // End of a read transaction: consume the response once fully drained.
    if (resp_ready &&
        tx_cursor >= (uint32_t)I2C_CTRL_RESP_HDR_LEN + resp_len) {
        resp_drop_from_isr();
    }
}

static void i2c_ctrl_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
        case I2C_SLAVE_RECEIVE:
            while (i2c_get_read_available(i2c) > 0)
                on_receive_byte(i2c_read_byte_raw(i2c));
            break;
        case I2C_SLAVE_REQUEST:
            on_request();
            break;
        case I2C_SLAVE_FINISH:
            on_finish();
            break;
    }
}

// ----------------------------------------------------------------------------
// Main-loop dispatch
// ----------------------------------------------------------------------------

// Park a response for the controller.  Small GET payloads are copied so the
// dispatcher's static buffers can be reused by other transports; a bulk GET
// keeps its pointer into bulk_param_buf under the bulk lock.
static void park_response(uint8_t status, const uint8_t *payload,
                          uint16_t len, bool bulk) {
    if (!bulk && len > 0) {
        if (len > sizeof(resp_small)) len = sizeof(resp_small);
        memcpy(resp_small, payload, len);
        payload = resp_small;
    }
    uint32_t save = save_and_disable_interrupts();
    resp_hdr[0]  = status;
    resp_hdr[1]  = (uint8_t)(len & 0xFF);
    resp_hdr[2]  = (uint8_t)(len >> 8);
    resp_payload = payload;
    resp_len     = (status == CTRL_STATUS_OK) ? len : 0;
    resp_bulk    = bulk;
    tx_cursor    = 0;
    __dmb();
    resp_ready   = true;
    restore_interrupts(save);
}

// Parked-bulk-response watchdog state (poll context only).
static uint32_t bulk_resp_cursor_seen;
static uint32_t bulk_resp_progress_time;

void i2c_ctrl_poll(void) {
    if (!live) return;

    // Keep a slow controller's bulk read from being reaped as stale, but
    // only while it is actually draining the response.  A controller that
    // requests a bulk GET and then never reads it must not pin the shared
    // bulk lock forever; after 1 s without read progress the parked
    // response is dropped and the lock freed (the controller re-issues).
    if (resp_ready && resp_bulk) {
        uint32_t cur = tx_cursor;
        if (cur != bulk_resp_cursor_seen || bulk_resp_progress_time == 0) {
            bulk_resp_cursor_seen = cur;
            bulk_resp_progress_time = time_us_32();
            vendor_bulk_touch(CTRL_SOURCE_I2C);
        } else if ((uint32_t)(time_us_32() - bulk_resp_progress_time) > 1000000u) {
            uint32_t save = save_and_disable_interrupts();
            resp_drop_from_isr();
            restore_interrupts(save);
            bulk_resp_progress_time = 0;
        } else {
            vendor_bulk_touch(CTRL_SOURCE_I2C);
        }
    } else {
        bulk_resp_progress_time = 0;
    }
    if (cur_valid && cur_bulk) vendor_bulk_touch(CTRL_SOURCE_I2C);

    // Claim a completed frame from the ISR.  Ownership (including a bulk
    // lock held for a streamed 0xA1 payload) moves to cur_* here, so a new
    // request arriving mid-dispatch cannot pull state out from under us.
    // Only the bytes the frame actually carries are copied; the IRQ-off
    // window stays sub-microsecond for GETs and short SETs.
    if (!cur_valid) {
        if (!frame_ready) return;
        uint32_t save = save_and_disable_interrupts();
        memcpy(cur_hdr, (const void *)rx_header, sizeof(cur_hdr));
        cur_bulk = rx_bulk;
        cur_discard = rx_discard;
        cur_discard_status = rx_discard_status;
        if (cur_hdr[0] == I2C_CTRL_TYPE_SET && !cur_bulk && !cur_discard) {
            uint16_t plen = hdr_wlen(cur_hdr);
            if (plen > sizeof(cur_payload)) plen = sizeof(cur_payload);
            if (plen) memcpy(cur_payload, (const void *)rx_payload, plen);
        }
        rx_bulk = false;
        frame_ready = false;
        restore_interrupts(save);
        cur_valid = true;
        retry_active = false;
    }

    uint8_t  type = cur_hdr[0];
    uint8_t  breq = cur_hdr[1];
    uint16_t wval = (uint16_t)(cur_hdr[2] | (cur_hdr[3] << 8));
    uint16_t widx = (uint16_t)(cur_hdr[4] | (cur_hdr[5] << 8));
    uint16_t wlen = hdr_wlen(cur_hdr);

    if (cur_discard) {
        // Payload was consumed but not stored (oversize / bulk lock busy).
        cur_valid = false;
        park_response(cur_discard_status, NULL, 0, false);
        return;
    }

    CtrlDispatchResult r;
    const uint8_t *resp_data = NULL;
    uint16_t rlen = 0;
    bool bulk_get = (type == I2C_CTRL_TYPE_GET && breq == REQ_GET_ALL_PARAMS);

    if (type == I2C_CTRL_TYPE_SET) {
        r = vendor_dispatch_set(CTRL_SOURCE_I2C, breq, wval, widx,
                                cur_bulk ? NULL : cur_payload, wlen);
    } else {
        r = vendor_dispatch_get(CTRL_SOURCE_I2C, breq, wval, widx, wlen,
                                &resp_data, &rlen);
    }

    // Transient contention: retry quietly for a while before surfacing it.
    if (r == CTRL_DISPATCH_BUSY || r == CTRL_DISPATCH_BULK_LOCKED) {
        if (!retry_active) {
            retry_active = true;
            retry_start = time_us_32();
            return;
        }
        if ((uint32_t)(time_us_32() - retry_start) < I2C_CTRL_RETRY_US) return;
    }
    retry_active = false;
    cur_valid = false;

    if (cur_bulk && r != CTRL_DISPATCH_OK) {
        // Dispatch did not take ownership of the streamed payload.
        vendor_bulk_release(CTRL_SOURCE_I2C);
        cur_bulk = false;
    }

    if (r == CTRL_DISPATCH_OK && type == I2C_CTRL_TYPE_GET) {
        park_response(CTRL_STATUS_OK, resp_data, rlen, bulk_get);
    } else {
        park_response(ctrl_status_from_dispatch(r), NULL, 0, false);
    }
}

// ----------------------------------------------------------------------------
// Bring-up / teardown / validation
// ----------------------------------------------------------------------------

// I2C function mux: SDA on even GPIOs, SCL on odd; instance from bit 1.
static inline uint8_t i2c_instance_index(uint8_t pin) { return (pin >> 1) & 1; }

uint8_t i2c_ctrl_validate(const I2cCtrlConfig *cfg) {
    if (!cfg) return PIN_CONFIG_INVALID_PARAM;
    if (cfg->enabled > 1) return PIN_CONFIG_INVALID_PARAM;
    if (cfg->address < I2C_CTRL_ADDRESS_MIN || cfg->address > I2C_CTRL_ADDRESS_MAX)
        return PIN_CONFIG_INVALID_PARAM;
    if (!is_valid_gpio_pin(cfg->sda_pin) || !is_valid_gpio_pin(cfg->scl_pin))
        return PIN_CONFIG_INVALID_PIN;
    if (!cfg->enabled) return PIN_CONFIG_SUCCESS;
    if ((cfg->sda_pin & 1) != 0 || (cfg->scl_pin & 1) != 1)
        return PIN_CONFIG_INVALID_PIN;
    if (i2c_instance_index(cfg->sda_pin) != i2c_instance_index(cfg->scl_pin))
        return PIN_CONFIG_INVALID_PIN;
    // The CS display holds the other hardware instance as I2C master; a
    // target config on the same instance (even on different pins) would
    // reset that block out from under it.
    if (cs_display_live_instance() == (int)i2c_instance_index(cfg->sda_pin))
        return PIN_CONFIG_PIN_IN_USE;
    uint8_t st = ctrl_iface_check_pin(cfg->sda_pin);
    if (st == PIN_CONFIG_SUCCESS) st = ctrl_iface_check_pin(cfg->scl_pin);
    return st;
}

static void i2c_ctrl_down(void) {
    if (!live) return;
    i2c_slave_deinit(inst);
    i2c_deinit(inst);
    gpio_set_function(live_cfg.sda_pin, GPIO_FUNC_NULL);
    gpio_set_function(live_cfg.scl_pin, GPIO_FUNC_NULL);
    gpio_disable_pulls(live_cfg.sda_pin);
    gpio_disable_pulls(live_cfg.scl_pin);
    live = false;
    inst = NULL;
    // Reset transport state; drop any lock we still hold.
    if ((frame_ready || rx_count > 0) && rx_bulk) vendor_bulk_release(CTRL_SOURCE_I2C);
    if (cur_valid && cur_bulk) vendor_bulk_release(CTRL_SOURCE_I2C);
    if (resp_ready && resp_bulk) vendor_bulk_release(CTRL_SOURCE_I2C);
    rx_count = 0; rx_bulk = false; rx_discard = false;
    frame_ready = false; resp_ready = false; resp_bulk = false;
    cur_valid = false; cur_bulk = false;
    tx_cursor = 0; busy_cursor = 0; retry_active = false;
}

static void i2c_ctrl_up(const I2cCtrlConfig *cfg) {
    uint8_t idx = i2c_instance_index(cfg->sda_pin);
    inst = (idx == 0) ? i2c0 : i2c1;
    // Baud here only seeds timing registers; as a target we follow the
    // controller's clock (Fast-mode, 400 kHz, is the supported ceiling).
    i2c_init(inst, 400 * 1000);
    gpio_set_function(cfg->sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(cfg->scl_pin, GPIO_FUNC_I2C);
    // Weak internal pull-ups; real buses want external 2.2k to 10k resistors.
    gpio_pull_up(cfg->sda_pin);
    gpio_pull_up(cfg->scl_pin);
    i2c_slave_init(inst, cfg->address, i2c_ctrl_slave_handler);
    irq_set_priority(I2C0_IRQ + idx, I2C_CTRL_IRQ_PRIORITY);
    live = true;
}

uint8_t i2c_ctrl_apply(const I2cCtrlConfig *cfg) {
    if (!cfg) return PIN_CONFIG_INVALID_PARAM;
    I2cCtrlConfig prev = live_cfg;
    bool was_live = live;
    i2c_ctrl_down();
    uint8_t status = i2c_ctrl_validate(cfg);
    if (status == PIN_CONFIG_SUCCESS) {
        live_cfg = *cfg;
        if (cfg->enabled) i2c_ctrl_up(cfg);
    } else if (was_live && i2c_ctrl_validate(&prev) == PIN_CONFIG_SUCCESS) {
        // Keep the old interface running rather than dropping the link.
        live_cfg = prev;
        i2c_ctrl_up(&prev);
    }
    return status;
}

// Returns the validation status so boot code can record it for
// REQ_GET_CTRL_IFACE_STATUS.  On failure the interface stays down but keeps
// its stored config; the status command exposes live=false to the host.
uint8_t i2c_ctrl_init(const I2cCtrlConfig *cfg) {
    live = false;
    if (!cfg) return PIN_CONFIG_INVALID_PARAM;
    live_cfg = *cfg;
    uint8_t st = i2c_ctrl_validate(cfg);
    if (st == PIN_CONFIG_SUCCESS && cfg->enabled) i2c_ctrl_up(cfg);
    return st;
}

bool i2c_ctrl_owns_pin(uint8_t pin) {
    return live && (pin == live_cfg.sda_pin || pin == live_cfg.scl_pin);
}

bool i2c_ctrl_is_live(void) { return live; }

int i2c_ctrl_live_instance(void) {
    if (!live) return -1;
    return (inst == i2c1) ? 1 : 0;
}
