/*
 * vc_core.c: page directory, lookup, validation, dispatch and LAST_RESULT
 * for the paged vendor protocol, and the UART/I2C entry points (vc_ctrl.h).
 * Cold path, runs from flash.
 */

#include "vc_core.h"
#include "vc_internal.h"
#include "config.h"
#include "audio_input.h"
#include "control_surfaces.h"
#include "notify.h"
#include "pico/time.h"
#ifdef DSPI_LOOP_PROF
#include "loop_prof.h"
#endif

#define X(m) extern const VcPage vc_page_##m;
VC_PAGE_LIST(X)
#undef X

#define X(m) &vc_page_##m,
const VcPage *const vc_page_dir[] = { VC_PAGE_LIST(X) };
#undef X
const uint8_t vc_page_dir_len = (uint8_t)(sizeof(vc_page_dir) / sizeof(vc_page_dir[0]));

// Two pages with one number fail as duplicate cases; page 0 is never a page.
#define X(m) case VC_PAGE_##m:
__attribute__((unused)) static void vc_page_number_check_(int n) {
    switch (n) {
    VC_PAGE_LIST(X)
    default: break;
    }
}
#undef X
#define X(m) _Static_assert(VC_PAGE_##m != 0, "page 0 is reserved");
VC_PAGE_LIST(X)
#undef X

_Static_assert(VC_SET_MAX <= VC_USB_RX_MAX, "non-bulk SET must fit the EP0 receive buffer");

static uint8_t vc_reply_storage[VC_REPLY_MAX] __attribute__((aligned(4)));

typedef struct {
    uint16_t id;
    uint16_t addr;
    uint8_t  is_get;
    uint8_t  result;
} VcLastResult;
static VcLastResult vc_last[VC_NUM_SOURCES];

uint8_t *vc_reply_buf(void) { return vc_reply_storage; }

const VcPage *vc_find_page(uint8_t page) {
    for (uint8_t i = 0; i < vc_page_dir_len; i++) {
        const VcPage *p = vc_page_dir[i];
        if (p->page == page) return p->count ? p : NULL;
    }
    return NULL;
}

const VcCmd *vc_find_cmd(const VcPage *p, uint8_t cmd) {
    if (!p) return NULL;
    for (uint8_t i = 0; i < p->count; i++)
        if (p->cmds[i].cmd == cmd) return &p->cmds[i];
    return NULL;
}

#ifdef DSPI_LOOP_PROF
int vc_cmd_index(uint16_t id) {
    int base = 0;
    for (uint8_t i = 0; i < vc_page_dir_len; i++) {
        const VcPage *p = vc_page_dir[i];
        if (p->page == (uint8_t)(id >> 8)) {
            for (uint8_t k = 0; k < p->count; k++)
                if (p->cmds[k].cmd == (uint8_t)id) return base + k;
            return -1;
        }
        base += p->count;
    }
    return -1;
}
#endif

static bool vc_addr_ok(uint8_t kind, uint16_t a, bool is_get) {
    uint8_t hi = (uint8_t)(a >> 8), lo = (uint8_t)a;
    switch (kind) {
        case VC_A_NONE:          return a == 0;
        case VC_A_CHANNEL:       return a < NUM_CHANNELS;
        case VC_A_INPUT:         return a < NUM_INPUT_CHANNELS;
        case VC_A_OUTPUT:        return a < NUM_OUTPUT_CHANNELS;
        case VC_A_OUTPUT_ALL:    return a < NUM_OUTPUT_CHANNELS || (!is_get && a == 0xFF);
        case VC_A_SLOT:          return a < NUM_SPDIF_INSTANCES;
        case VC_A_PIN_OUTPUT:    return a < NUM_PIN_OUTPUTS;
        case VC_A_PRESET:        return a < PRESET_SLOTS;
        case VC_A_CH_BAND:       return hi < NUM_CHANNELS;
        case VC_A_IN_OUT:        return hi < NUM_INPUT_CHANNELS && lo < NUM_OUTPUT_CHANNELS;
        case VC_A_SOURCE:        return a <= INPUT_SOURCE_MAX;
        case VC_A_SPDIF_RX:      return a < SPDIF_RX_NUM_INPUTS;
        case VC_A_I2S_PAIR:      return a < I2S_RX_MAX_PAIRS;
        case VC_A_ROLE:          return a <= 1;
        case VC_A_CS_SLOT:       return a < CS_MAX_BINDINGS;
        case VC_A_CS_SUB:        return a < CS_MAX_IR_COMMANDS;
        case VC_A_CS_GROUP:      return a < CS_MAX_GROUPS;
        case VC_A_CS_MACRO:      return a < CS_MAX_MACROS;
        case VC_A_CS_MACRO_STEP: return hi < CS_MAX_MACRO_STEPS && lo < CS_MAX_MACROS;
        case VC_A_CS_PAGE:       return a < CS_MAX_DISPLAY_PAGES;
        case VC_A_CS_NOUN:       return a < CS_NOUN_COUNT;
        case VC_A_PAGE:          return a <= 0xFF;
        case VC_A_PAGE_CMD:
        case VC_A_OFFSET:
        case VC_A_INDEX:         return true;
        case VC_A_CS_SLOT_TYPE:  return lo < CS_MAX_BINDINGS && hi != CS_TYPE_NONE && hi < CS_TYPE_COUNT;
        default:                 return false;
    }
}

ParamSource vc_param_src(CtrlSource src) {
    switch (src) {
        case CTRL_SOURCE_UART: return PARAM_SRC_UART;
        case CTRL_SOURCE_I2C:  return PARAM_SRC_I2C;
        case CTRL_SOURCE_GPIO: return PARAM_SRC_GPIO;
        default:               return PARAM_SRC_HOST_SET;
    }
}

VcResult vc_check(CtrlSource src, bool is_get, uint8_t page, uint8_t cmd,
                  uint16_t addr, uint16_t in_len, const VcCmd **entry) {
    const VcCmd *e = vc_find_cmd(vc_find_page(page), cmd);
    if (!e) return VC_ERR_UNKNOWN;
    if (!(is_get ? e->get : e->set)) return VC_ERR_DIRECTION;

    if (src != CTRL_SOURCE_USB) {
        if (e->flags & VC_F_USB_ONLY) return VC_ERR_BLOCKED;
        if (!is_get && (e->flags & VC_F_SET_USB_ONLY)) return VC_ERR_BLOCKED;
    }
    VcResult r = vc_bulk_ext_check(src, (uint16_t)((page << 8) | cmd), e);
    if (r != VC_OK) return r;

    if (!is_get) {
        if (e->flags & VC_F_BULK) {
            if (in_len == 0 || in_len > vc_bulk_cap()) return VC_ERR_LENGTH;
        } else if (in_len < e->set_min || in_len > e->set_max) {
            return VC_ERR_LENGTH;
        }
    }
    if (!vc_addr_ok(e->addr_kind, addr, is_get)) return VC_ERR_ADDRESS;
    *entry = e;
    return VC_OK;
}

VcResult vc_run(CtrlSource src, bool is_get, const VcCmd *e, uint16_t id,
                uint16_t addr, const uint8_t *in, uint16_t in_len,
                uint8_t *out_buf, uint16_t out_cap,
                const uint8_t **out, uint16_t *out_len) {
    VcCtx c = {
        .src = src, .arg = e->arg, .id = id, .addr = addr,
        .in = in_len ? in : NULL, .in_len = in_len,
        .out = out_buf, .out_cap = out_cap, .out_len = 0,
    };
    bool fixed = is_get && e->get_len != VC_LEN_VAR;
    if (fixed) memset(out_buf, 0, e->get_len < out_cap ? e->get_len : out_cap);

#ifdef DSPI_LOOP_PROF
    uint32_t t0 = time_us_32();
#endif
    notify_set_source(vc_param_src(src));
    VcResult r = (is_get ? e->get : e->set)(&c);
    notify_set_source(PARAM_SRC_UNKNOWN);
#ifdef DSPI_LOOP_PROF
    loop_prof_command(id, time_us_32() - t0);
#endif

    if (out) {
        uint16_t len = fixed ? e->get_len : c.out_len;
        if (len > out_cap) len = out_cap;
        *out = (is_get && r == VC_OK) ? out_buf : NULL;
        *out_len = (is_get && r == VC_OK) ? len : 0;
    }
    return r;
}

void vc_record(CtrlSource src, bool is_get, uint16_t id, uint16_t addr, VcResult r) {
    if ((unsigned)src >= VC_NUM_SOURCES) return;
    if (is_get && id == VC_SYS_LAST_RESULT) return;   // reading it must not overwrite it
    vc_last[src] = (VcLastResult){ .id = id, .addr = addr,
                                   .is_get = is_get ? 1 : 0, .result = (uint8_t)r };
}

void vc_last_result(CtrlSource src, uint8_t out[8]) {
    memset(out, 0, 8);
    if ((unsigned)src >= VC_NUM_SOURCES) return;
    const VcLastResult *l = &vc_last[src];
    out[0] = (uint8_t)l->id;   out[1] = (uint8_t)(l->id >> 8);
    out[2] = l->is_get;        out[3] = l->result;
    out[4] = (uint8_t)l->addr; out[5] = (uint8_t)(l->addr >> 8);
}

VcResult vc_dispatch(CtrlSource src, bool is_get, uint8_t page, uint8_t cmd,
                     uint16_t addr, const uint8_t *in, uint16_t in_len,
                     const uint8_t **out, uint16_t *out_len) {
    uint16_t id = (uint16_t)((page << 8) | cmd);
    const VcCmd *e = NULL;
    VcResult r;
    if (out) { *out = NULL; *out_len = 0; }

    // A USB transfer may hold vendor_usb_rx_buf or the reply buffer mid-stream.
    if (src != CTRL_SOURCE_USB && vendor_usb_busy()) {
        r = VC_ERR_BUSY;
    } else {
        r = vc_check(src, is_get, page, cmd, addr, in_len, &e);
        if (r == VC_OK)
            r = vc_run(src, is_get, e, id, addr, in, in_len,
                       vc_reply_storage, VC_REPLY_MAX, out, out_len);
    }
    vc_record(src, is_get, id, addr, r);
    return r;
}

VcResult vc_set(CtrlSource src, uint16_t id, uint16_t addr,
                const void *payload, uint16_t len) {
    return vc_dispatch(src, false, (uint8_t)(id >> 8), (uint8_t)id, addr,
                       (const uint8_t *)payload, len, NULL, NULL);
}

VcResult vc_get(CtrlSource src, uint16_t id, uint16_t addr,
                const uint8_t **out, uint16_t *out_len) {
    return vc_dispatch(src, true, (uint8_t)(id >> 8), (uint8_t)id, addr,
                       NULL, 0, out, out_len);
}

// ---- USB in-flight guard ----
// Set from SETUP until a USB SET's data stage or a multi-packet reply is done.
// Stale after 100 ms in case the host aborts the transfer mid-flight.
static volatile bool usb_busy;
static uint32_t usb_busy_since;

void vendor_usb_busy_begin(void) {
    usb_busy_since = time_us_32();
    usb_busy = true;
}
void vendor_usb_busy_end(void) { usb_busy = false; }
bool vendor_usb_busy(void) {
    if (!usb_busy) return false;
    if ((uint32_t)(time_us_32() - usb_busy_since) > 100000u) {
        usb_busy = false;
        return false;
    }
    return true;
}

// ---- External transports (UART, I2C) and internal callers ----

uint8_t ctrl_dispatch_get(CtrlSource src, uint8_t bRequest, uint16_t wValue,
                          uint16_t wIndex, uint16_t wLength,
                          const uint8_t **resp, uint16_t *resp_len) {
    *resp = NULL;
    *resp_len = 0;
    if (src == CTRL_SOURCE_USB) return VC_ERR_BLOCKED;   // USB has its own front end
    if (vendor_usb_busy()) return CTRL_STATUS_BUSY;

    uint8_t page = vc_page_of(0xC1, wIndex);
    uint8_t st = ctrl_is_bulk_get(wIndex, bRequest)
        ? vc_bulk_ext_get(src, page, bRequest, wValue, resp, resp_len)
        : (uint8_t)vc_dispatch(src, true, page, bRequest, wValue, NULL, 0, resp, resp_len);
    if (st == CTRL_STATUS_OK && wLength && *resp_len > wLength) *resp_len = wLength;
    return st;
}

uint8_t ctrl_dispatch_set(CtrlSource src, uint8_t bRequest, uint16_t wValue,
                          uint16_t wIndex, const uint8_t *payload, uint16_t wLength) {
    if (src == CTRL_SOURCE_USB) return VC_ERR_BLOCKED;
    if (vendor_usb_busy()) return CTRL_STATUS_BUSY;

    uint8_t page = vc_page_of(0x41, wIndex);
    if (vendor_is_bulk_set(wIndex, bRequest, wLength))
        return vc_bulk_ext_set(src, page, bRequest, wValue, wLength);
    // vc_check() refuses a payload above set_max before any handler reads it.
    return (uint8_t)vc_dispatch(src, false, page, bRequest, wValue, payload, wLength, NULL, NULL);
}

// ---- Deferred reboot for SYS.ENTER_BOOTLOADER over UART and I2C ----
static volatile uint8_t ctrl_reboot_mask;

void ctrl_reboot_request(CtrlSource src) { ctrl_reboot_mask |= (uint8_t)(1u << src); }
bool ctrl_reboot_pending(CtrlSource src) { return (ctrl_reboot_mask >> src) & 1u; }
