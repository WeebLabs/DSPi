/*
 * vc_page_dyn.c: DYN page. Command set: vendor_command_overhaul_plan.md section 5.7.
 */

#include <math.h>
#include <stddef.h>
#include "vc_core.h"
#include "config.h"
#include "usb_audio.h"
#include "limiter.h"
#include "leveller.h"
#include "loudness.h"
#include "bulk_params.h"
#include "notify.h"

// ---- Limiter (ARG is the LIMITER_PARAM_* index) ----

static VcResult dyn_limiter_get(VcCtx *c) {
    float v;
    if (!limiter_get_param((uint8_t)c->addr, c->arg, &v)) return VC_ERR_ADDRESS;
    vc_out_f32(c, 0, v);
    return VC_OK;
}

// limiter_set_param() owns clamping, group ganging, the pending flag and notifications.
static VcResult dyn_limiter_set(VcCtx *c) {
    float v = vc_in_f32(c, 0);
    if (v != v) return VC_ERR_VALUE;
    if (!limiter_set_param((uint8_t)c->addr, c->arg, v)) return VC_ERR_ADDRESS;
    return VC_OK;
}

static VcResult dyn_limiter_meters_get(VcCtx *c) {
    for (uint8_t k = 0; k < NUM_OUTPUT_CHANNELS; k++)
        vc_out_u16(c, (uint16_t)(2 * k), limiter_meter_centidb(k));
    c->out_len = 2 * NUM_OUTPUT_CHANNELS;
    return VC_OK;
}

static VcResult dyn_limiter_status_get(VcCtx *c) {
    vc_out_u8(c, 0, limiter_is_engaged() ? 1 : 0);
    vc_out_u8(c, 1, LIMITER_DELAY);
    vc_out_u8(c, 2, LIMITER_BLOCK);
    vc_out_u8(c, 3, NUM_OUTPUT_CHANNELS);
    return VC_OK;
}

// ---- Leveller ----

static VcResult dyn_lev_enable_get(VcCtx *c) {
    vc_out_u8(c, 0, leveller_config.enabled ? 1 : 0);
    return VC_OK;
}

static VcResult dyn_lev_enable_set(VcCtx *c) {
    leveller_config.enabled = (vc_in_u8(c, 0) != 0);
    leveller_update_pending = true;
    leveller_reset_pending = true;
    uint8_t v = leveller_config.enabled ? 1 : 0;
    notify_param_write(offsetof(WireBulkParams, leveller.enabled), 1, &v);
    return VC_OK;
}

static VcResult dyn_lev_amount_get(VcCtx *c) {
    vc_out_f32(c, 0, leveller_config.amount);
    return VC_OK;
}

static VcResult dyn_lev_amount_set(VcCtx *c) {
    float raw = vc_in_f32(c, 0);
    if (!isfinite(raw)) return VC_ERR_VALUE;
    float val = vc_clampf(raw, LEVELLER_AMOUNT_MIN, LEVELLER_AMOUNT_MAX);
    leveller_config.amount = val;
    leveller_update_pending = true;
    notify_param_write(offsetof(WireBulkParams, leveller.amount), sizeof(float), &val);
    return VC_OK;
}

static VcResult dyn_lev_speed_get(VcCtx *c) {
    vc_out_u8(c, 0, leveller_config.speed);
    return VC_OK;
}

static VcResult dyn_lev_speed_set(VcCtx *c) {
    uint8_t spd = vc_in_u8(c, 0);
    if (spd >= LEVELLER_SPEED_COUNT) return VC_ERR_VALUE;
    leveller_config.speed = spd;
    leveller_update_pending = true;
    notify_param_write(offsetof(WireBulkParams, leveller.speed), 1, &spd);
    return VC_OK;
}

static VcResult dyn_lev_max_gain_get(VcCtx *c) {
    vc_out_f32(c, 0, leveller_config.max_gain_db);
    return VC_OK;
}

static VcResult dyn_lev_max_gain_set(VcCtx *c) {
    float raw = vc_in_f32(c, 0);
    if (!isfinite(raw)) return VC_ERR_VALUE;
    float val = vc_clampf(raw, LEVELLER_MAX_GAIN_MIN, LEVELLER_MAX_GAIN_MAX);
    leveller_config.max_gain_db = val;
    leveller_update_pending = true;
    notify_param_write(offsetof(WireBulkParams, leveller.max_gain_db), sizeof(float), &val);
    return VC_OK;
}

static VcResult dyn_lev_lookahead_get(VcCtx *c) {
    vc_out_u8(c, 0, leveller_config.lookahead ? 1 : 0);
    return VC_OK;
}

static VcResult dyn_lev_lookahead_set(VcCtx *c) {
    leveller_config.lookahead = (vc_in_u8(c, 0) != 0);
    leveller_update_pending = true;
    leveller_reset_pending = true;   // clears the delay buffer
    uint8_t v = leveller_config.lookahead ? 1 : 0;
    notify_param_write(offsetof(WireBulkParams, leveller.lookahead), 1, &v);
    return VC_OK;
}

static VcResult dyn_lev_gate_get(VcCtx *c) {
    vc_out_f32(c, 0, leveller_config.gate_threshold_db);
    return VC_OK;
}

static VcResult dyn_lev_gate_set(VcCtx *c) {
    float raw = vc_in_f32(c, 0);
    if (!isfinite(raw)) return VC_ERR_VALUE;
    float val = vc_clampf(raw, LEVELLER_GATE_MIN, LEVELLER_GATE_MAX);
    leveller_config.gate_threshold_db = val;
    leveller_update_pending = true;
    notify_param_write(offsetof(WireBulkParams, leveller.gate_threshold_db), sizeof(float), &val);
    return VC_OK;
}

static VcResult dyn_lev_masks_get(VcCtx *c) {
    vc_out_u8(c, 0, leveller_config.detector_mask);
    vc_out_u8(c, 1, leveller_config.apply_mask);
    return VC_OK;
}

// No reset: masks switch glitch-free while the rings keep running.
static VcResult dyn_lev_masks_set(VcCtx *c) {
    leveller_config.detector_mask = vc_in_u8(c, 0);
    leveller_config.apply_mask = vc_in_u8(c, 1);
    leveller_update_pending = true;
    uint8_t dm = leveller_config.detector_mask;
    uint8_t am = leveller_config.apply_mask;
    notify_param_write(offsetof(WireBulkParams, leveller.detector_mask), 1, &dm);
    notify_param_write(offsetof(WireBulkParams, leveller.apply_mask), 1, &am);
    return VC_OK;
}

// ---- Loudness ----

static VcResult dyn_loud_enable_get(VcCtx *c) {
    vc_out_u8(c, 0, loudness_enabled ? 1 : 0);
    return VC_OK;
}

// Coefficients follow effective_vol_index, whichever owner drives vol_mul (USB, LG Sound Sync).
static VcResult dyn_loud_enable_set(VcCtx *c) {
    loudness_enabled = (vc_in_u8(c, 0) != 0);
    if (loudness_enabled && loudness_active_table) {
        uint8_t idx = effective_vol_index;
        if (idx > CENTER_VOLUME_INDEX) idx = CENTER_VOLUME_INDEX;
        current_loudness_coeffs = loudness_active_table[idx];
    } else {
        current_loudness_coeffs = NULL;
    }
    uint8_t v = loudness_enabled ? 1 : 0;
    notify_param_write(offsetof(WireBulkParams, global.loudness_enabled), 1, &v);
    return VC_OK;
}

static VcResult dyn_loud_ref_get(VcCtx *c) {
    vc_out_f32(c, 0, loudness_ref_spl);
    return VC_OK;
}

static VcResult dyn_loud_ref_set(VcCtx *c) {
    float raw = vc_in_f32(c, 0);
    if (!isfinite(raw)) return VC_ERR_VALUE;
    float val = vc_clampf(raw, LOUDNESS_REF_SPL_MIN, LOUDNESS_REF_SPL_MAX);
    loudness_ref_spl = val;
    loudness_recompute_pending = true;
    notify_param_write(offsetof(WireBulkParams, global.loudness_ref_spl), sizeof(float), &val);
    return VC_OK;
}

static VcResult dyn_loud_intensity_get(VcCtx *c) {
    vc_out_f32(c, 0, loudness_intensity_pct);
    return VC_OK;
}

static VcResult dyn_loud_intensity_set(VcCtx *c) {
    float raw = vc_in_f32(c, 0);
    if (!isfinite(raw)) return VC_ERR_VALUE;
    float val = vc_clampf(raw, LOUDNESS_INTENSITY_MIN, LOUDNESS_INTENSITY_MAX);
    loudness_intensity_pct = val;
    loudness_recompute_pending = true;
    notify_param_write(offsetof(WireBulkParams, global.loudness_intensity_pct), sizeof(float), &val);
    return VC_OK;
}

static VcResult dyn_loud_mask_get(VcCtx *c) {
    vc_out_u16(c, 0, loudness_output_mask);
    return VC_OK;
}

// No recompute: the mask does not affect the coefficient table.
static VcResult dyn_loud_mask_set(VcCtx *c) {
    loudness_output_mask = vc_in_u16(c, 0);
    uint16_t m = loudness_output_mask;
    notify_param_write(offsetof(WireBulkParams, global.loudness_output_mask), 2, (const uint8_t *)&m);
    return VC_OK;
}

#define VC_PAGE_DEF "pages/dyn.def"
#include "vc_page_table.inc"
