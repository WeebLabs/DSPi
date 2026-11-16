/*
 * vc_core.h: table-driven vendor command dispatcher (paged protocol).
 *
 * Design and rules: Documentation/Features/vendor_command_overhaul_plan.md
 * sections 3 and 8. Page files include this header only.
 */

#ifndef VC_CORE_H
#define VC_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "vc_ctrl.h"            // CtrlSource, VcResult
#include "config.h"             // PIN_CONFIG_*
#include "notify.h"             // ParamSource
#include "vc_ids.h"

#define VC_PROTOCOL_MAJOR 1
#define VC_PROTOCOL_MINOR 0

#define VC_SET_MAX       64u       // largest non-bulk SET payload
#define VC_LEN_VAR       0xFFFFu   // GET_LEN for a variable-length reply
#define VC_NUM_SOURCES   (CTRL_SOURCE_GPIO + 1)

// The numeric values reach hosts through SYS.COMMAND_INFO: append only.
typedef enum {
    VC_A_NONE = 0, VC_A_CHANNEL, VC_A_INPUT, VC_A_OUTPUT, VC_A_OUTPUT_ALL,
    VC_A_SLOT, VC_A_PIN_OUTPUT, VC_A_PRESET, VC_A_CH_BAND, VC_A_IN_OUT,
    VC_A_SOURCE, VC_A_SPDIF_RX, VC_A_I2S_PAIR, VC_A_ROLE, VC_A_CS_SLOT,
    VC_A_CS_SUB, VC_A_CS_GROUP, VC_A_CS_MACRO, VC_A_CS_MACRO_STEP, VC_A_CS_PAGE,
    VC_A_CS_NOUN, VC_A_PAGE, VC_A_PAGE_CMD, VC_A_OFFSET, VC_A_INDEX,
    VC_A_COUNT
} VcAddrKind;

// Bit values reach hosts through SYS.COMMAND_INFO: never renumber.
#define VC_F_USB_ONLY      0x0001u   // enforced: both directions refused off USB
#define VC_F_SET_USB_ONLY  0x0002u   // enforced: SET refused off USB
#define VC_F_BULK          0x0004u   // enforced: payload or reply lives in bulk_param_buf
#define VC_F_DEFERRED      0x0008u   // informational from here down
#define VC_F_FLASH         0x0010u
#define VC_F_RESTART       0x0020u
#define VC_F_KEEPALIVE     0x0040u

typedef struct {
    CtrlSource     src;       // transport of this request
    uint8_t        arg;       // per-entry constant from the table
    uint16_t       id;        // (page << 8) | cmd
    uint16_t       addr;      // wValue, already range-checked for the address kind
    const uint8_t *in;        // SET payload (NULL when in_len is 0)
    uint16_t       in_len;
    uint8_t       *out;       // GET reply buffer, zeroed for a fixed-length reply
    uint16_t       out_cap;   // capacity of out
    uint16_t       out_len;   // set by variable-length GET handlers only
} VcCtx;

typedef VcResult (*VcHandler)(VcCtx *c);

typedef struct {
    uint8_t   cmd;
    uint8_t   addr_kind;      // VcAddrKind
    uint8_t   set_min;        // SET payload bounds (ignored for VC_F_BULK)
    uint8_t   set_max;
    uint16_t  get_len;        // fixed reply length, VC_LEN_VAR, or 0 with no GET
    uint16_t  flags;          // VC_F_*
    uint8_t   arg;
    VcHandler get;            // NULL = no GET
    VcHandler set;            // NULL = no SET
} VcCmd;

typedef struct {
    uint8_t      page;
    uint8_t      revision;
    uint8_t      count;       // 0 = page not implemented on this build
    const VcCmd *cmds;
} VcPage;

// The page directory, ascending by page number (vc_core.c).
extern const VcPage *const vc_page_dir[];
extern const uint8_t vc_page_dir_len;

// The single place a request's page is derived. Every transport calls this.
static inline uint8_t vc_page_of(uint8_t bmRequestType, uint16_t wIndex) {
    (void)bmRequestType;
    return (uint8_t)(wIndex >> 8);
}

const VcPage *vc_find_page(uint8_t page);   // NULL unless implemented
const VcCmd  *vc_find_cmd(const VcPage *p, uint8_t cmd);

#ifdef DSPI_LOOP_PROF
// Position of a built command across the page directory, -1 when not built.
// Keys the profiler's per-command storage (loop_prof.c); never sent to a host.
int vc_cmd_index(uint16_t id);
#endif

// Full dispatch for UART, I2C and internal callers: checks, notify bracket,
// handler, LAST_RESULT. A GET reply in *out stays valid until the next dispatch.
VcResult vc_dispatch(CtrlSource src, bool is_get, uint8_t page, uint8_t cmd,
                     uint16_t addr, const uint8_t *in, uint16_t in_len,
                     const uint8_t **out, uint16_t *out_len);

// Typed wrappers for internal callers.
VcResult vc_set(CtrlSource src, uint16_t id, uint16_t addr,
                const void *payload, uint16_t len);
VcResult vc_get(CtrlSource src, uint16_t id, uint16_t addr,
                const uint8_t **out, uint16_t *out_len);

// Dispatch steps, split so the USB front end can reject at SETUP and run later.
VcResult vc_check(CtrlSource src, bool is_get, uint8_t page, uint8_t cmd,
                  uint16_t addr, uint16_t in_len, const VcCmd **entry);
VcResult vc_run(CtrlSource src, bool is_get, const VcCmd *e, uint16_t id,
                uint16_t addr, const uint8_t *in, uint16_t in_len,
                uint8_t *out_buf, uint16_t out_cap,
                const uint8_t **out, uint16_t *out_len);
void     vc_record(CtrlSource src, bool is_get, uint16_t id, uint16_t addr,
                   VcResult r);

// SYS.LAST_RESULT layout for one transport.
void vc_last_result(CtrlSource src, uint8_t out[8]);

// The shared static reply buffer (VC_REPLY_MAX bytes).
uint8_t *vc_reply_buf(void);

// The notification tag for a request's transport. Handlers that tag a deferred
// notification themselves must use this so it matches the dispatcher's bracket.
ParamSource vc_param_src(CtrlSource src);

// PIN_CONFIG_* (config.h) to the result a pin SET reports.
static inline VcResult vc_from_pin_status(uint8_t status) {
    switch (status) {
        case PIN_CONFIG_SUCCESS:        return VC_OK;
        case PIN_CONFIG_INVALID_PIN:    return VC_ERR_PIN_INVALID;
        case PIN_CONFIG_PIN_IN_USE:     return VC_ERR_PIN_IN_USE;
        case PIN_CONFIG_INVALID_OUTPUT: return VC_ERR_ADDRESS;
        case PIN_CONFIG_OUTPUT_ACTIVE:  return VC_ERR_STATE;
        default:                        return VC_ERR_VALUE;
    }
}

// ---- Payload and reply helpers for handlers (little-endian, unaligned-safe) ----

static inline uint8_t vc_in_u8(const VcCtx *c, uint16_t off) { return c->in[off]; }
static inline uint16_t vc_in_u16(const VcCtx *c, uint16_t off) {
    return (uint16_t)(c->in[off] | (c->in[off + 1] << 8));
}
static inline uint32_t vc_in_u32(const VcCtx *c, uint16_t off) {
    uint32_t v; memcpy(&v, c->in + off, 4); return v;
}
static inline float vc_in_f32(const VcCtx *c, uint16_t off) {
    float v; memcpy(&v, c->in + off, 4); return v;
}

static inline void vc_out_u8(VcCtx *c, uint16_t off, uint8_t v) { c->out[off] = v; }
static inline void vc_out_u16(VcCtx *c, uint16_t off, uint16_t v) {
    c->out[off] = (uint8_t)v; c->out[off + 1] = (uint8_t)(v >> 8);
}
static inline void vc_out_u32(VcCtx *c, uint16_t off, uint32_t v) { memcpy(c->out + off, &v, 4); }
static inline void vc_out_f32(VcCtx *c, uint16_t off, float v) { memcpy(c->out + off, &v, 4); }

// NaN passes through unclamped; handlers that must refuse it check first.
static inline float vc_clampf(float v, float lo, float hi) {
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    return v;
}

// Copy a whole reply; also sets out_len, so it serves variable-length replies.
static inline void vc_reply(VcCtx *c, const void *data, uint16_t len) {
    if (len > c->out_cap) len = c->out_cap;
    memcpy(c->out, data, len);
    c->out_len = len;
}

#endif // VC_CORE_H
