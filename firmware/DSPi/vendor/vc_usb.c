/*
 * vc_usb.c: EP0 front end for paged vendor requests (page != 0). Runs in
 * tud_task (main loop). Stage rules: plan section 8.5.
 */

#include "vc_internal.h"
#include "pico/bootrom.h"

static bool vc_usb_reboot_pending;

void vc_usb_bootloader_after_status(void) { vc_usb_reboot_pending = true; }

static bool vc_usb_is_bulk(uint8_t page, uint8_t cmd) {
    const VcCmd *e = vc_find_cmd(vc_find_page(page), cmd);
    return e && (e->flags & VC_F_BULK);
}

static bool vc_usb_setup(uint8_t rhport, tusb_control_request_t const *req,
                         uint8_t page, bool is_get, uint16_t id) {
    const VcCmd *e = NULL;
    uint16_t in_len = is_get ? 0 : req->wLength;
    // Cleared first: a reboot request whose status stage never completed must not fire later.
    vc_usb_reboot_pending = false;

    VcResult r = vc_check(CTRL_SOURCE_USB, is_get, page, req->bRequest,
                          req->wValue, in_len, &e);
    if (r != VC_OK) {
        vc_record(CTRL_SOURCE_USB, is_get, id, req->wValue, r);
        return false;
    }
    bool bulk = (e->flags & VC_F_BULK) != 0;
    if (bulk && !vc_bulk_usb_acquire()) {
        vc_record(CTRL_SOURCE_USB, is_get, id, req->wValue, VC_ERR_BUSY);
        return false;
    }

    if (is_get) {
        uint8_t *buf = bulk ? vc_bulk_buf() : vc_reply_buf();
        uint16_t cap = bulk ? vc_bulk_cap() : (uint16_t)VC_REPLY_MAX;
        const uint8_t *out;
        uint16_t len;
        r = vc_run(CTRL_SOURCE_USB, true, e, id, req->wValue, NULL, 0, buf, cap, &out, &len);
        vc_record(CTRL_SOURCE_USB, true, id, req->wValue, r);
        if (r != VC_OK) {
            if (bulk) vc_bulk_usb_release();
            return false;
        }
        if (len > req->wLength) len = req->wLength;
        // TinyUSB reads later packets from the buffer after this returns.
        if (!bulk && len > CFG_TUD_ENDPOINT0_SIZE) vendor_usb_busy_begin();
        return tud_control_xfer(rhport, req, (void *)out, len);
    }

    if (in_len == 0) {
        // Zero-length SET: the handler runs now, so failure can still STALL.
        r = vc_run(CTRL_SOURCE_USB, false, e, id, req->wValue, NULL, 0, NULL, 0, NULL, NULL);
        vc_record(CTRL_SOURCE_USB, false, id, req->wValue, r);
        if (r != VC_OK) return false;
        return tud_control_status(rhport, req);
    }

    uint8_t *rx = bulk ? vc_bulk_buf() : vendor_usb_rx_buf();
    vendor_usb_busy_begin();
    return tud_control_xfer(rhport, req, rx, req->wLength);
}

static bool vc_usb_set_data(tusb_control_request_t const *req, uint8_t page, uint16_t id) {
    const VcCmd *e = NULL;
    VcResult r = vc_check(CTRL_SOURCE_USB, false, page, req->bRequest,
                          req->wValue, req->wLength, &e);
    bool bulk = (r == VC_OK) && (e->flags & VC_F_BULK);
    if (r == VC_OK) {
        const uint8_t *in = bulk ? vc_bulk_buf() : vendor_usb_rx_buf();
        r = vc_run(CTRL_SOURCE_USB, false, e, id, req->wValue, in, req->wLength,
                   NULL, 0, NULL, NULL);
    }
    vendor_usb_busy_end();
    vc_record(CTRL_SOURCE_USB, false, id, req->wValue, r);
    if (r != VC_OK && bulk) vc_bulk_usb_release();
    return r == VC_OK;   // false STALLs the status stage
}

bool vc_usb_control_xfer(uint8_t rhport, uint8_t stage, tusb_control_request_t const *req) {
    uint8_t page = vc_page_of(req->bmRequestType, req->wIndex);
    bool is_get = req->bmRequestType_bit.direction == TUSB_DIR_IN;
    uint16_t id = (uint16_t)((page << 8) | req->bRequest);

    if (stage == CONTROL_STAGE_SETUP) return vc_usb_setup(rhport, req, page, is_get, id);

    if (stage == CONTROL_STAGE_DATA) {
        return is_get ? true : vc_usb_set_data(req, page, id);
    }

    if (stage == CONTROL_STAGE_ACK) {
        vendor_usb_busy_end();
        if (vc_usb_is_bulk(page, req->bRequest)) vc_bulk_usb_release();
        if (!is_get && vc_usb_reboot_pending) {
            vc_usb_reboot_pending = false;
            reset_usb_boot(0, 0);
        }
    }
    return true;
}
