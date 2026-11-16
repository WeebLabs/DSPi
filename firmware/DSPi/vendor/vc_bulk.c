/*
 * vc_bulk.c: VC_F_BULK routing for paged commands over the bulk_param_buf
 * lock, and the USB chunk sessions shared by STORE.ALL_PARAMS_CHUNK and the
 * page-0 0xA2/0xA3 opcodes. The lock itself still lives in vendor_commands.c.
 */

#include "vc_internal.h"
#include "config.h"
#include "usb_audio.h"
#include "bulk_params.h"

_Static_assert(WIRE_BULK_BUF_SIZE <= 0xFFFFu, "bulk length must fit a u16 wLength");
_Static_assert(sizeof(WireBulkParams) <= WIRE_BULK_BUF_SIZE, "chunk offsets index bulk_param_buf");

#define VC_BULK_PARAMS_LEN ((uint16_t)sizeof(WireBulkParams))

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

// ---- Chunk sessions ----
// A session spans several EP0 transfers and keeps the USB bulk lock between
// them. Only a request on the path that opened it continues it; anything else
// reaching tud_vendor_control_xfer_cb closes it (see vc_bulk_chunk_continues).

bool     usb_chunk_get_open = false;   // snapshot parked in bulk_param_buf
bool     usb_chunk_get_done = false;   // final chunk queued; release at ACK
bool     usb_chunk_set_open = false;
uint16_t usb_chunk_set_received = 0;   // next expected byte offset
static bool vc_chunk_paged = false;    // the open session belongs to STORE.ALL_PARAMS_CHUNK

void usb_chunk_sessions_close(void) {
    usb_chunk_get_open = false;
    usb_chunk_get_done = false;
    usb_chunk_set_open = false;
    usb_chunk_set_received = 0;
    vc_chunk_paged = false;
}

bool vc_bulk_chunk_continues(uint8_t page, bool is_get, uint8_t bRequest) {
    if (page == 0) {
        return !vc_chunk_paged &&
               ((bRequest == REQ_GET_ALL_PARAMS_CHUNK && usb_chunk_get_open) ||
                (bRequest == REQ_SET_ALL_PARAMS_CHUNK && usb_chunk_set_open));
    }
    if ((uint16_t)((page << 8) | bRequest) != VC_STORE_ALL_PARAMS_CHUNK) return false;
    return vc_chunk_paged && (is_get ? usb_chunk_get_open : usb_chunk_set_open);
}

// Offset 0 (re)opens a session under the lock; later offsets must continue the
// paged session the lock still guards.
static VcResult vc_chunk_open(bool is_get) {
    if (!vendor_bulk_usb_holds() && !vendor_bulk_try_acquire(CTRL_SOURCE_USB))
        return VC_ERR_BUSY;
    usb_chunk_sessions_close();
    vc_chunk_paged = true;
    if (is_get) usb_chunk_get_open = true;
    else        usb_chunk_set_open = true;
    return VC_OK;
}

static bool vc_chunk_live(bool is_get) {
    return vc_chunk_paged && vendor_bulk_usb_holds() &&
           (is_get ? usb_chunk_get_open : usb_chunk_set_open);
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
