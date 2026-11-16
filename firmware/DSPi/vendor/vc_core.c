/*
 * vc_core.c: page directory, lookup, validation, dispatch and LAST_RESULT
 * for the paged vendor protocol. Cold path, runs from flash.
 */

#include "vc_core.h"
#include "vc_internal.h"
#include "config.h"
#include "audio_input.h"
#include "control_surfaces.h"
#include "notify.h"

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
        default:                 return false;
    }
}

static ParamSource vc_param_src(CtrlSource src) {
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
    VcResult r = vc_bulk_ext_check(src, is_get, e);
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

    notify_set_source(vc_param_src(src));
    VcResult r = (is_get ? e->get : e->set)(&c);
    notify_set_source(PARAM_SRC_UNKNOWN);

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

// Phase 3 extends the UART/I2C status byte to carry the VcResult itself.
CtrlDispatchResult vc_to_ctrl_dispatch(VcResult r) {
    switch (r) {
        case VC_OK:          return CTRL_DISPATCH_OK;
        case VC_ERR_BLOCKED: return CTRL_DISPATCH_BLOCKED;
        case VC_ERR_BUSY:    return CTRL_DISPATCH_BUSY;
        default:             return CTRL_DISPATCH_ERROR;
    }
}
