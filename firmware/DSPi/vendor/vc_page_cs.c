/*
 * vc_page_cs.c: CS page (0x0A). Command set: vendor_command_overhaul_plan.md section 5.10.
 * Preview SETs stage into the same single-deep cs_set_*_pending handoffs as the
 * legacy opcodes; the main loop applies them and posts the result to cs_last_status.
 */

#include "vc_core.h"
#include "config.h"
#include "control_surfaces.h"
#include "notify.h"
#include "hardware/sync.h"

_Static_assert(sizeof(CsCapsHeader) == 52, "CS.CAPS size");
_Static_assert(sizeof(CsNounDesc) == 12, "CS.NOUN_DESC size");
_Static_assert(sizeof(CsStatusPacket) == 41, "CS.STATUS size");
_Static_assert(sizeof(CsExtStatusPacket) == 24, "CS.EXT_STATUS size");
_Static_assert(sizeof(CsBinding) == 24, "CS.BINDING size");
_Static_assert(sizeof(IrCommand) == 16, "CS.IR_CMD size");
_Static_assert(sizeof(CsGroup) == 40, "CS.GROUP size");
_Static_assert(sizeof(CsMacro) == 132, "CS.MACRO size");
_Static_assert(sizeof(CsMacroHeaderWire) == 36, "CS.MACRO_HEADER size");
_Static_assert(sizeof(CsMacroStep) == 12, "CS.MACRO_STEP size");
_Static_assert(sizeof(CsDisplayCfg) == 12, "CS.DISPLAY_CONFIG size");
_Static_assert(sizeof(CsDisplayPage) == 4, "CS.DISPLAY_PAGE size");
_Static_assert(sizeof(CsDisplayStatus) == 8, "CS.DISPLAY_STATUS size");
_Static_assert(3 * CS_MAX_BINDINGS == 48, "CS.AUX_ALL size");

// cs_last_slot tags keep the result channel's slot spaces apart (control_surfaces.h).
#define CS_TAG_IR     0x80u
#define CS_TAG_GROUP  0x40u
#define CS_TAG_MACRO  0x60u
#define CS_TAG_DISP   0x50u
#define CS_TAG_GLOBAL 0xFFu

static void cs_report(uint8_t status, uint8_t slot) {
    cs_last_status = status;
    cs_last_slot = slot;
}

// ---- Status and capabilities ----

static VcResult cs_caps_get(VcCtx *c) {
    memcpy(c->out, control_surfaces_caps_header(), sizeof(CsCapsHeader));
    return VC_OK;
}

static VcResult cs_noun_desc_get(VcCtx *c) {
    const CsNounDesc *d = control_surfaces_noun_desc((uint8_t)c->addr);
    if (d == NULL) return VC_ERR_ADDRESS;
    memcpy(c->out, d, sizeof(CsNounDesc));
    return VC_OK;
}

static VcResult cs_status_get(VcCtx *c) {
    CsStatusPacket pkt;
    control_surfaces_get_status(&pkt);
    memcpy(c->out, &pkt, sizeof(pkt));
    return VC_OK;
}

static VcResult cs_ext_status_get(VcCtx *c) {
    CsExtStatusPacket pkt;
    control_surfaces_get_ext_status(&pkt);
    memcpy(c->out, &pkt, sizeof(pkt));
    return VC_OK;
}

// ---- Save and revert ----

static VcResult cs_save_set(VcCtx *c) {
    (void)c;
    if (cs_save_pending || cs_revert_pending) {
        cs_report(CS_STATUS_BUSY, CS_TAG_GLOBAL);
        return VC_ERR_BUSY;
    }
    cs_report(CS_STATUS_PENDING, CS_TAG_GLOBAL);
    __dmb();
    cs_save_pending = true;
    return VC_OK;
}

static VcResult cs_revert_set(VcCtx *c) {
    (void)c;
    if (cs_save_pending || cs_revert_pending) {
        cs_report(CS_STATUS_BUSY, CS_TAG_GLOBAL);
        return VC_ERR_BUSY;
    }
    cs_report(CS_STATUS_PENDING, CS_TAG_GLOBAL);
    __dmb();
    cs_revert_pending = true;
    return VC_OK;
}

// ---- Bindings and slot names ----

static VcResult cs_binding_get(VcCtx *c) {
    const CsBinding *b = control_surfaces_get_binding((uint8_t)c->addr);
    if (b == NULL) return VC_ERR_ADDRESS;
    memcpy(c->out, b, sizeof(CsBinding));
    return VC_OK;
}

static VcResult cs_binding_set(VcCtx *c) {
    uint8_t slot = (uint8_t)c->addr;
    if (cs_set_binding_pending) {
        cs_report(CS_STATUS_BUSY, slot);
        return VC_ERR_BUSY;
    }
    memcpy((void *)&cs_set_binding_val, c->in, sizeof(CsBinding));
    cs_set_binding_slot = slot;
    cs_report(CS_STATUS_PENDING, slot);
    __dmb();
    cs_set_binding_pending = true;
    return VC_OK;
}

static VcResult cs_slot_name_get(VcCtx *c) {
    const char *name = control_surfaces_get_name((uint8_t)c->addr);
    if (name == NULL) return VC_ERR_ADDRESS;
    memcpy(c->out, name, CS_NAME_LEN);
    return VC_OK;
}

static VcResult cs_slot_name_set(VcCtx *c) {
    uint8_t slot = (uint8_t)c->addr;
    if (cs_set_name_pending) {
        cs_report(CS_STATUS_BUSY, slot);
        return VC_ERR_BUSY;
    }
    // The last byte stays NUL, so a full 32-byte payload loses its final character.
    memset(cs_set_name_val, 0, CS_NAME_LEN);
    uint16_t n = c->in_len < (CS_NAME_LEN - 1) ? c->in_len : (CS_NAME_LEN - 1);
    memcpy(cs_set_name_val, c->in, n);
    cs_set_name_slot = slot;
    cs_report(CS_STATUS_PENDING, slot);
    __dmb();
    cs_set_name_pending = true;
    return VC_OK;
}

// ---- IR commands and learn ----

static VcResult cs_ir_cmd_get(VcCtx *c) {
    const IrCommand *ir = control_surfaces_get_ir_cmd((uint8_t)c->addr);
    if (ir == NULL) return VC_ERR_ADDRESS;
    memcpy(c->out, ir, sizeof(IrCommand));
    return VC_OK;
}

static VcResult cs_ir_cmd_set(VcCtx *c) {
    uint8_t sub = (uint8_t)c->addr;
    if (cs_set_ir_cmd_pending) {
        cs_report(CS_STATUS_BUSY, (uint8_t)(CS_TAG_IR | sub));
        return VC_ERR_BUSY;
    }
    memcpy((void *)&cs_set_ir_cmd_val, c->in, sizeof(IrCommand));
    cs_set_ir_cmd_slot = sub;
    cs_report(CS_STATUS_PENDING, (uint8_t)(CS_TAG_IR | sub));
    __dmb();
    cs_set_ir_cmd_pending = true;
    return VC_OK;
}

static VcResult cs_ir_learn_status_get(VcCtx *c) {
    control_surfaces_get_ir_learn(c->out);
    return VC_OK;
}

// ARG 1 arms, 0 cancels. A running revert refuses without touching cs_last_status.
static VcResult cs_ir_learn_set(VcCtx *c) {
    uint8_t rc = control_surfaces_ir_learn_control(c->arg);
    if (rc == CS_STATUS_BUSY) return VC_ERR_BUSY;
    if (rc == CS_STATUS_NO_IR) {
        cs_report(CS_STATUS_NO_IR, CS_TAG_GLOBAL);
        return VC_ERR_STATE;
    }
    return VC_OK;
}

// ---- Groups ----

static VcResult cs_group_get(VcCtx *c) {
    const CsGroup *g = control_surfaces_get_group((uint8_t)c->addr);
    if (g == NULL) return VC_ERR_ADDRESS;
    memcpy(c->out, g, sizeof(CsGroup));
    return VC_OK;
}

static VcResult cs_group_set(VcCtx *c) {
    uint8_t idx = (uint8_t)c->addr;
    if (cs_set_group_pending) {
        cs_report(CS_STATUS_BUSY, (uint8_t)(CS_TAG_GROUP | idx));
        return VC_ERR_BUSY;
    }
    memcpy((void *)&cs_set_group_val, c->in, sizeof(CsGroup));
    cs_set_group_slot = idx;
    cs_report(CS_STATUS_PENDING, (uint8_t)(CS_TAG_GROUP | idx));
    __dmb();
    cs_set_group_pending = true;
    return VC_OK;
}

// ---- Macros ----

static VcResult cs_macro_get(VcCtx *c) {
    const CsMacro *m = control_surfaces_get_macro((uint8_t)c->addr);
    if (m == NULL) return VC_ERR_ADDRESS;
    memcpy(c->out, m, sizeof(CsMacro));
    return VC_OK;
}

// The header as MACRO_HEADER SET takes it; the SET rejects nonzero reserved bytes.
static VcResult cs_macro_header_get(VcCtx *c) {
    const CsMacro *m = control_surfaces_get_macro((uint8_t)c->addr);
    if (m == NULL) return VC_ERR_ADDRESS;
    CsMacroHeaderWire h;
    memset(&h, 0, sizeof(h));
    memcpy(h.name, m->name, CS_NAME_LEN);
    h.step_count = m->step_count;
    memcpy(c->out, &h, sizeof(h));
    return VC_OK;
}

static VcResult cs_macro_header_set(VcCtx *c) {
    uint8_t idx = (uint8_t)c->addr;
    if (cs_set_macro_hdr_pending) {
        cs_report(CS_STATUS_BUSY, (uint8_t)(CS_TAG_MACRO | idx));
        return VC_ERR_BUSY;
    }
    memcpy((void *)&cs_set_macro_hdr_val, c->in, sizeof(CsMacroHeaderWire));
    cs_set_macro_hdr_slot = idx;
    cs_report(CS_STATUS_PENDING, (uint8_t)(CS_TAG_MACRO | idx));
    __dmb();
    cs_set_macro_hdr_pending = true;
    return VC_OK;
}

static VcResult cs_macro_step_get(VcCtx *c) {
    const CsMacro *m = control_surfaces_get_macro((uint8_t)c->addr);
    if (m == NULL) return VC_ERR_ADDRESS;
    memcpy(c->out, &m->steps[c->addr >> 8], sizeof(CsMacroStep));
    return VC_OK;
}

static VcResult cs_macro_step_set(VcCtx *c) {
    uint8_t idx = (uint8_t)c->addr;
    if (cs_set_macro_step_pending) {
        cs_report(CS_STATUS_BUSY, (uint8_t)(CS_TAG_MACRO | idx));
        return VC_ERR_BUSY;
    }
    memcpy((void *)&cs_set_macro_step_val, c->in, sizeof(CsMacroStep));
    cs_set_macro_step_slot = idx;
    cs_set_macro_step_idx = (uint8_t)(c->addr >> 8);
    cs_report(CS_STATUS_PENDING, (uint8_t)(CS_TAG_MACRO | idx));
    __dmb();
    cs_set_macro_step_pending = true;
    return VC_OK;
}

static VcResult cs_macro_fire_set(VcCtx *c) {
    uint8_t idx = (uint8_t)c->addr;
    uint8_t rc = control_surfaces_macro_fire(idx);
    if (rc == CS_STATUS_BUSY) return VC_ERR_BUSY;   // revert running; status untouched
    if (rc != PIN_CONFIG_SUCCESS) {
        cs_report(rc, (uint8_t)(CS_TAG_MACRO | idx));
        return VC_ERR_VALUE;
    }
    return VC_OK;
}

static VcResult cs_macro_cancel_set(VcCtx *c) {
    (void)c;
    control_surfaces_macro_cancel();
    return VC_OK;
}

// ---- Aux outputs ----

static VcResult cs_aux_state_get(VcCtx *c) {
    if (!control_surfaces_aux_up((uint8_t)c->addr)) return VC_ERR_STATE;
    vc_out_u8(c, 0, control_surfaces_aux_state((uint8_t)c->addr));
    return VC_OK;
}

static VcResult cs_aux_level_get(VcCtx *c) {
    if (!control_surfaces_aux_up((uint8_t)c->addr)) return VC_ERR_STATE;
    vc_out_u16(c, 0, control_surfaces_aux_level((uint8_t)c->addr));
    return VC_OK;
}

static VcResult cs_aux_all_get(VcCtx *c) {
    for (uint8_t i = 0; i < CS_MAX_BINDINGS; i++) {
        vc_out_u8(c, i, control_surfaces_aux_state(i));
        vc_out_u16(c, (uint16_t)(CS_MAX_BINDINGS + 2 * i), control_surfaces_aux_level(i));
    }
    return VC_OK;
}

// ARG 0 is AUX_STATE, 1 is AUX_LEVEL. Applied at once: no flash, no dirty flag.
static VcResult cs_aux_set(VcCtx *c) {
    uint8_t slot = (uint8_t)c->addr;
    // A revert in progress refuses without touching cs_last_status (it reports PENDING).
    if (control_surfaces_busy()) return VC_ERR_BUSY;
    uint8_t  before_st = control_surfaces_aux_state(slot);
    uint16_t before_lv = control_surfaces_aux_level(slot);
    bool ok = c->arg ? control_surfaces_set_aux_level(slot, vc_in_u16(c, 0))
                     : control_surfaces_set_aux_state(slot, vc_in_u8(c, 0) ? 1 : 0);
    if (!ok) {
        cs_report(CS_STATUS_INVALID_AUX, slot);
        return VC_ERR_STATE;
    }
    uint8_t  st = control_surfaces_aux_state(slot);
    uint16_t lv = control_surfaces_aux_level(slot);
    if (st != before_st || lv != before_lv)
        notify_push_cs_aux(slot, st, lv, vc_param_src(c->src));
    return VC_OK;
}

// ---- Display ----

static VcResult cs_display_config_get(VcCtx *c) {
    memcpy(c->out, &control_surfaces_display_flash()->cfg, sizeof(CsDisplayCfg));
    return VC_OK;
}

static VcResult cs_display_config_set(VcCtx *c) {
    if (cs_set_disp_cfg_pending) {
        cs_report(CS_STATUS_BUSY, CS_TAG_DISP);
        return VC_ERR_BUSY;
    }
    memcpy((void *)&cs_set_disp_cfg_val, c->in, sizeof(CsDisplayCfg));
    cs_report(CS_STATUS_PENDING, CS_TAG_DISP);
    __dmb();
    cs_set_disp_cfg_pending = true;
    return VC_OK;
}

static VcResult cs_display_caps_get(VcCtx *c) {
    vc_out_u8(c, 0, CS_MAX_DISPLAY_PAGES);
    vc_out_u8(c, 1, CS_DISP_MODEL_COUNT);
    return VC_OK;
}

static VcResult cs_display_page_get(VcCtx *c) {
    const CsDisplayPage *p = control_surfaces_get_display_page((uint8_t)c->addr);
    if (p == NULL) return VC_ERR_ADDRESS;
    memcpy(c->out, p, sizeof(CsDisplayPage));
    return VC_OK;
}

static VcResult cs_display_page_set(VcCtx *c) {
    uint8_t idx = (uint8_t)c->addr;
    if (cs_set_disp_page_pending) {
        cs_report(CS_STATUS_BUSY, (uint8_t)(CS_TAG_DISP | idx));
        return VC_ERR_BUSY;
    }
    memcpy((void *)&cs_set_disp_page_val, c->in, sizeof(CsDisplayPage));
    cs_set_disp_page_slot = idx;
    cs_report(CS_STATUS_PENDING, (uint8_t)(CS_TAG_DISP | idx));
    __dmb();
    cs_set_disp_page_pending = true;
    return VC_OK;
}

static VcResult cs_display_status_get(VcCtx *c) {
    CsDisplayStatus st;
    control_surfaces_get_display_status(&st);
    memcpy(c->out, &st, sizeof(st));
    return VC_OK;
}

#define VC_PAGE_DEF "pages/cs.def"
#include "vc_page_table.inc"
