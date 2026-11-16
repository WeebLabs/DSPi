/*
 * vc_page_filter.c: FILTER page. Command set: vendor_command_overhaul_plan.md section 5.6.
 * Band index map (PEQ 0..count-1, crossover 20..23 on outputs): crossover_filters_spec.md.
 */

#include <math.h>
#include <stddef.h>
#include "vc_core.h"
#include "config.h"
#include "usb_audio.h"
#include "dsp_pipeline.h"
#include "crossover.h"
#include "bulk_params.h"
#include "notify.h"

// The stored recipe for a band, or NULL for the reserved gap and input-channel crossover bands.
static const EqParamPacket *filter_recipe(uint16_t addr) {
    uint8_t ch = (uint8_t)(addr >> 8), band = (uint8_t)addr;
    if (band < channel_band_counts[ch]) return &filter_recipes[ch][band];
    if (band >= XOVER_BAND_BASE && band < XOVER_BAND_BASE + MAX_XOVER_BANDS && ch >= CH_OUT_1)
        return &xover_recipes[ch][band - XOVER_BAND_BASE];
    return NULL;
}

static uint16_t filter_stored_qp(uint16_t addr) {
    uint8_t ch = (uint8_t)(addr >> 8), band = (uint8_t)addr;
    return band < channel_band_counts[ch] ? peq_qp_x512[ch][band] : 0;
}

// One pending slot feeds the main-loop apply (main.c, eq_update_pending). A second
// write before it is consumed would overwrite the first, so it is refused instead.
static VcResult filter_stage(const EqParamPacket *p, uint16_t qp) {
    if (eq_update_pending) return VC_ERR_BUSY;
    memcpy((void *)&pending_packet, p, sizeof(EqParamPacket));
    pending_eq_qp_x512 = qp;
    eq_update_pending = true;
    return VC_OK;
}

static VcResult filter_band_get(VcCtx *c) {
    const EqParamPacket *p = filter_recipe(c->addr);
    if (!p) return VC_ERR_ADDRESS;
    vc_out_u8(c, 0, p->type);
    vc_out_u8(c, 1, p->bypass == 1 ? 1 : 0);
    vc_out_u16(c, 2, filter_stored_qp(c->addr));
    vc_out_f32(c, 4, p->freq);
    vc_out_f32(c, 8, p->Q);
    vc_out_f32(c, 12, p->gain_db);
    return VC_OK;
}

// The band field must be the wire index: main.c routes on it (PEQ versus crossover).
static VcResult filter_band_set(VcCtx *c) {
    if (!filter_recipe(c->addr)) return VC_ERR_ADDRESS;
    EqParamPacket p = {
        .channel = (uint8_t)(c->addr >> 8),
        .band    = (uint8_t)c->addr,
        .type    = vc_in_u8(c, 0),
        .bypass  = vc_in_u8(c, 1) == 1 ? 1 : 0,
        .freq    = vc_in_f32(c, 4),
        .Q       = vc_in_f32(c, 8),
        .gain_db = vc_in_f32(c, 12),
    };
    if (!isfinite(p.freq) || !isfinite(p.Q) || !isfinite(p.gain_db)) return VC_ERR_VALUE;
    bool is_peq = p.band < channel_band_counts[p.channel];
    return filter_stage(&p, is_peq ? vc_in_u16(c, 2) : 0);
}

static VcResult filter_band_bypass_get(VcCtx *c) {
    const EqParamPacket *p = filter_recipe(c->addr);
    if (!p) return VC_ERR_ADDRESS;
    vc_out_u8(c, 0, p->bypass == 1 ? 1 : 0);
    return VC_OK;
}

static VcResult filter_band_bypass_set(VcCtx *c) {
    const EqParamPacket *stored = filter_recipe(c->addr);
    if (!stored) return VC_ERR_ADDRESS;
    EqParamPacket p = *stored;
    p.channel = (uint8_t)(c->addr >> 8);
    p.band    = (uint8_t)c->addr;
    p.bypass  = vc_in_u8(c, 0) == 1 ? 1 : 0;
    return filter_stage(&p, filter_stored_qp(c->addr));
}

static VcResult filter_eq_bypass_get(VcCtx *c) {
    vc_out_u8(c, 0, bypass_master_eq ? 1 : 0);
    return VC_OK;
}

static VcResult filter_eq_bypass_set(VcCtx *c) {
    bypass_master_eq = (vc_in_u8(c, 0) != 0);
    uint8_t v = bypass_master_eq ? 1 : 0;
    notify_param_write(offsetof(WireBulkParams, global.bypass), 1, &v);
    return VC_OK;
}

#define VC_PAGE_DEF "pages/filter.def"
#include "vc_page_table.inc"
