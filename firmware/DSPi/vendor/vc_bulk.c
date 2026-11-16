/*
 * vc_bulk.c: the bulk_param_buf lock shared by USB, UART and I2C, VC_F_BULK
 * routing on every transport, and the USB chunk sessions of STORE.ALL_PARAMS_CHUNK.
 */

#include "vc_internal.h"
#include "config.h"
#include "usb_audio.h"
#include "bulk_params.h"
#include "hardware/sync.h"
#include "pico/time.h"

_Static_assert(WIRE_BULK_BUF_SIZE <= 0xFFFFu, "bulk length must fit a u16 wLength");
_Static_assert(sizeof(WireBulkParams) <= WIRE_BULK_BUF_SIZE, "chunk offsets index bulk_param_buf");

#define VC_BULK_PARAMS_LEN ((uint16_t)sizeof(WireBulkParams))

uint8_t *vc_bulk_buf(void) { return bulk_param_buf; }
uint16_t vc_bulk_cap(void) { return (uint16_t)WIRE_BULK_BUF_SIZE; }

// ---- bulk_param_buf lock ----
// One owner at a time; after a SET is handed over, bulk_params_pending guards the
// buffer until the main loop has applied it. Owner is CtrlSource + 1, 0 = free.
#define BULK_OWNER_NONE  0u
#define BULK_OWNER_USB   ((uint8_t)(CTRL_SOURCE_USB + 1))
static volatile uint8_t bulk_buf_owner = BULK_OWNER_NONE;
static uint32_t bulk_owner_since;

static inline uint8_t bulk_owner_for(CtrlSource src) { return (uint8_t)(src + 1); }

// The I2C target ISR acquires when a bulk SET header arrives, racing the main loop.
// USB gets a longer stale window because EP0 has no refresh hook.
bool vendor_bulk_try_acquire(CtrlSource src) {
    uint32_t save = save_and_disable_interrupts();
    if (bulk_buf_owner != BULK_OWNER_NONE) {
        uint32_t limit = (bulk_buf_owner == BULK_OWNER_USB) ? 3000000u : 500000u;
        if ((uint32_t)(time_us_32() - bulk_owner_since) > limit)
            bulk_buf_owner = BULK_OWNER_NONE;
    }
    bool ok = (bulk_buf_owner == BULK_OWNER_NONE && !bulk_params_pending);
    if (ok) {
        bulk_buf_owner = bulk_owner_for(src);
        bulk_owner_since = time_us_32();
    }
    restore_interrupts(save);
    return ok;
}

void vendor_bulk_release(CtrlSource src) {
    uint32_t save = save_and_disable_interrupts();
    if (bulk_buf_owner == bulk_owner_for(src)) bulk_buf_owner = BULK_OWNER_NONE;
    restore_interrupts(save);
}

// A slow transport refreshes the stale timer while it streams (a full bulk
// transfer takes about 0.5 s at 115200 baud).
void vendor_bulk_touch(CtrlSource src) {
    if (bulk_buf_owner == bulk_owner_for(src)) bulk_owner_since = time_us_32();
}

bool vendor_bulk_usb_holds(void) { return bulk_buf_owner == BULK_OWNER_USB; }

// The transports route a SET payload into bulk_param_buf on this answer and the
// dispatcher reads it back from there, so both must use this one function.
// A paged length the handler refuses still lands here, so the result code matches USB.
bool vendor_is_bulk_set(uint16_t wIndex, uint8_t bRequest, uint16_t wLength) {
    uint16_t id = (uint16_t)((vc_page_of(0x41, wIndex) << 8) | bRequest);
    return id == VC_STORE_ALL_PARAMS && wLength != 0 && wLength <= WIRE_BULK_BUF_SIZE;
}

bool ctrl_is_bulk_get(uint16_t wIndex, uint8_t bRequest) {
    return (uint16_t)((vc_page_of(0xC1, wIndex) << 8) | bRequest) == VC_STORE_ALL_PARAMS;
}

// USB holds the lock from SETUP to the status ACK (vc_usb.c); the SETUP-stage
// cleanup in tud_vendor_control_xfer_cb drops it if the host abandons a transfer.
bool vc_bulk_usb_acquire(void) { return vendor_bulk_try_acquire(CTRL_SOURCE_USB); }
void vc_bulk_usb_release(void) { vendor_bulk_release(CTRL_SOURCE_USB); }

// UART and I2C stream only STORE.ALL_PARAMS (ctrl_dispatch_get/set supply the
// buffer); internal callers have no bulk buffer. USB-only bulk commands never get here.
VcResult vc_bulk_ext_check(CtrlSource src, uint16_t id, const VcCmd *e) {
    if (!(e->flags & VC_F_BULK) || src == CTRL_SOURCE_USB) return VC_OK;
    if ((src == CTRL_SOURCE_UART || src == CTRL_SOURCE_I2C) && id == VC_STORE_ALL_PARAMS)
        return VC_OK;
    return VC_ERR_BLOCKED;
}

// On OK the reply stays in bulk_param_buf under src's lock for the transport to stream.
uint8_t vc_bulk_ext_get(CtrlSource src, uint8_t page, uint8_t cmd, uint16_t addr,
                        const uint8_t **out, uint16_t *out_len) {
    uint16_t id = (uint16_t)((page << 8) | cmd);
    const VcCmd *e = NULL;
    VcResult r = vc_check(src, true, page, cmd, addr, 0, &e);
    if (r == VC_OK) {
        if (!vendor_bulk_try_acquire(src)) return CTRL_STATUS_BULK_LOCKED;
        r = vc_run(src, true, e, id, addr, NULL, 0, bulk_param_buf, vc_bulk_cap(), out, out_len);
        if (r != VC_OK) vendor_bulk_release(src);
    }
    vc_record(src, true, id, addr, r);
    return (uint8_t)r;
}

// The payload is already in bulk_param_buf under src's lock. The handler sets
// bulk_params_pending on OK, so the lock is dropped only then; on failure the
// transport releases it, keeping release and its own state in one place.
uint8_t vc_bulk_ext_set(CtrlSource src, uint8_t page, uint8_t cmd, uint16_t addr,
                        uint16_t len) {
    uint16_t id = (uint16_t)((page << 8) | cmd);
    const VcCmd *e = NULL;
    VcResult r = vc_check(src, false, page, cmd, addr, len, &e);
    if (r == VC_OK) {
        // Refreshed with the owner check, so the 500 ms stale reclaim in the I2C
        // ISR cannot take the buffer before the handler sets bulk_params_pending.
        uint32_t save = save_and_disable_interrupts();
        bool owned = (bulk_buf_owner == bulk_owner_for(src));
        if (owned) bulk_owner_since = time_us_32();
        restore_interrupts(save);
        if (!owned) return CTRL_STATUS_BULK_LOCKED;
        r = vc_run(src, false, e, id, addr, bulk_param_buf, len, NULL, 0, NULL, NULL);
        if (r == VC_OK) vendor_bulk_release(src);
    }
    vc_record(src, false, id, addr, r);
    return (uint8_t)r;
}

// ---- Chunk sessions ----
// A session spans several EP0 transfers and keeps the USB bulk lock between
// them. Any other request reaching tud_vendor_control_xfer_cb closes it
// (see vc_bulk_chunk_continues).

static bool     usb_chunk_get_open = false;   // snapshot parked in bulk_param_buf
static bool     usb_chunk_get_done = false;   // final chunk queued; release at ACK
static bool     usb_chunk_set_open = false;
static uint16_t usb_chunk_set_received = 0;   // next expected byte offset

void usb_chunk_sessions_close(void) {
    usb_chunk_get_open = false;
    usb_chunk_get_done = false;
    usb_chunk_set_open = false;
    usb_chunk_set_received = 0;
}

bool vc_bulk_chunk_continues(uint16_t id, bool is_get) {
    if (id != VC_STORE_ALL_PARAMS_CHUNK) return false;
    return is_get ? usb_chunk_get_open : usb_chunk_set_open;
}

// Offset 0 (re)opens a session under the lock; later offsets must continue the
// session the lock still guards.
static VcResult vc_chunk_open(bool is_get) {
    if (!vendor_bulk_usb_holds() && !vendor_bulk_try_acquire(CTRL_SOURCE_USB))
        return VC_ERR_BUSY;
    usb_chunk_sessions_close();
    if (is_get) usb_chunk_get_open = true;
    else        usb_chunk_set_open = true;
    return VC_OK;
}

static bool vc_chunk_live(bool is_get) {
    return vendor_bulk_usb_holds() && (is_get ? usb_chunk_get_open : usb_chunk_set_open);
}

// The GET handler replies in place: vc_bulk_usb_window() points c->out at the
// requested offset of the snapshot.
VcResult vc_bulk_chunk_get(VcCtx *c) {
    uint16_t off = c->addr;
    if (off >= VC_BULK_PARAMS_LEN) return VC_ERR_ADDRESS;
    if (c->out != bulk_param_buf + off) return VC_ERR_BLOCKED;
    if (c->out_cap == 0) return VC_ERR_LENGTH;
    if (off == 0) {
        VcResult r = vc_chunk_open(true);
        if (r != VC_OK) return r;
        bulk_params_collect((WireBulkParams *)bulk_param_buf);
    } else if (!vc_chunk_live(true)) {
        return VC_ERR_STATE;   // no snapshot: restart at offset 0
    }
    usb_chunk_get_done = ((uint32_t)off + c->out_cap >= VC_BULK_PARAMS_LEN);
    vendor_bulk_touch(CTRL_SOURCE_USB);
    c->out_len = c->out_cap;
    return VC_OK;
}

// SETUP stage of a chunk SET: the payload then lands at bulk_param_buf + off.
static VcResult vc_chunk_set_begin(uint16_t off, uint16_t n) {
    if (off >= VC_BULK_PARAMS_LEN) return VC_ERR_ADDRESS;
    if ((uint32_t)off + n > VC_BULK_PARAMS_LEN) return VC_ERR_LENGTH;
    if (off == 0) {
        VcResult r = vc_chunk_open(false);
        if (r != VC_OK) return r;
    } else if (!vc_chunk_live(false) || off != usb_chunk_set_received) {
        return VC_ERR_STATE;   // out of order or session lost: restart at offset 0
    }
    vendor_bulk_touch(CTRL_SOURCE_USB);
    return VC_OK;
}

// DATA stage of a chunk SET. The final chunk hands the buffer to the main-loop apply.
VcResult vc_bulk_chunk_set(VcCtx *c) {
    uint16_t off = c->addr;
    if (c->in != bulk_param_buf + off) return VC_ERR_BLOCKED;
    if (!vc_chunk_live(false) || off != usb_chunk_set_received) return VC_ERR_STATE;
    usb_chunk_set_received = (uint16_t)(off + c->in_len);
    vendor_bulk_touch(CTRL_SOURCE_USB);
    if (usb_chunk_set_received < VC_BULK_PARAMS_LEN) return VC_OK;
    usb_chunk_sessions_close();
    VcResult r = vc_bulk_params_check(bulk_param_buf);
    if (r != VC_OK) return r;
    bulk_params_pending = true;   // guards bulk_param_buf from here; the lock drops at ACK
    return VC_OK;
}

// The header checks bulk_params_apply() makes, so a mismatch fails the request.
VcResult vc_bulk_params_check(const uint8_t *buf) {
    const WireBulkParams *p = (const WireBulkParams *)buf;
#if PICO_RP2350
    if (p->header.platform_id != WIRE_PLATFORM_RP2350) return VC_ERR_VALUE;
#else
    if (p->header.platform_id != WIRE_PLATFORM_RP2040) return VC_ERR_VALUE;
#endif
    if (p->header.num_channels != NUM_CHANNELS ||
        p->header.num_input_channels != NUM_INPUT_CHANNELS ||
        p->header.num_output_channels != NUM_OUTPUT_CHANNELS ||
        p->header.format_version != WIRE_FORMAT_VERSION ||
        p->header.payload_length != sizeof(WireBulkParams))
        return VC_ERR_VALUE;
    return VC_OK;
}

// ---- USB stage hooks for VC_F_BULK commands (vc_usb.c) ----

VcResult vc_bulk_usb_setup(uint16_t id, bool is_get, uint16_t wValue, uint16_t wLength) {
    if (id == VC_STORE_ALL_PARAMS_CHUNK)
        return is_get ? VC_OK : vc_chunk_set_begin(wValue, wLength);   // a GET opens in its handler
    if (id == VC_STORE_ALL_PARAMS && !is_get && wLength != VC_BULK_PARAMS_LEN)
        return VC_ERR_LENGTH;
    return vc_bulk_usb_acquire() ? VC_OK : VC_ERR_BUSY;
}

uint8_t *vc_bulk_usb_window(uint16_t id, uint16_t wValue, uint16_t wLength, uint16_t *cap) {
    if (id != VC_STORE_ALL_PARAMS_CHUNK) {
        *cap = vc_bulk_cap();
        return bulk_param_buf;
    }
    if (wValue >= VC_BULK_PARAMS_LEN) {
        *cap = 0;
        return bulk_param_buf;
    }
    uint16_t room = (uint16_t)(VC_BULK_PARAMS_LEN - wValue);
    *cap = wLength < room ? wLength : room;
    return bulk_param_buf + wValue;
}

// A refused chunk SET at SETUP keeps its session, so the host can resend the right offset.
void vc_bulk_usb_failed(uint16_t id, bool is_get, bool at_setup) {
    if (id == VC_STORE_ALL_PARAMS_CHUNK) {
        if (!is_get && at_setup) return;
        usb_chunk_sessions_close();
    }
    vc_bulk_usb_release();
}

void vc_bulk_usb_ack(uint16_t id, bool is_get) {
    if (id == VC_STORE_ALL_PARAMS_CHUNK) {
        if (is_get ? !usb_chunk_get_done : usb_chunk_set_open) return;   // session continues
        usb_chunk_sessions_close();
    }
    vc_bulk_usb_release();
}
