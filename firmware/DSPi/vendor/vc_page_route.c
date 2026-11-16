/*
 * vc_page_route.c: ROUTE page. Command set: vendor_command_overhaul_plan.md section 5.5.
 */

#include <math.h>
#include <stddef.h>
#include "vc_core.h"
#include "config.h"
#include "usb_audio.h"
#include "dsp_pipeline.h"
#include "flash_storage.h"
#include "bulk_params.h"
#include "notify.h"
#include "pdm_generator.h"
#include "hardware/sync.h"

// ---- Matrix ----

static VcResult route_matrix_get(VcCtx *c) {
    const MatrixCrosspoint *xp = &matrix_mixer.crosspoints[c->addr >> 8][c->addr & 0xFF];
    vc_out_u8(c, 0, xp->enabled);
    vc_out_u8(c, 1, xp->phase_invert);
    vc_out_f32(c, 4, xp->gain_db);
    return VC_OK;
}

static VcResult route_matrix_set(VcCtx *c) {
    uint8_t input = (uint8_t)(c->addr >> 8), output = (uint8_t)c->addr;
    uint8_t enabled = vc_in_u8(c, 0), phase = vc_in_u8(c, 1);
    float gain_db = vc_in_f32(c, 4);
    if (!isfinite(gain_db)) return VC_ERR_VALUE;

    MatrixCrosspoint *xp = &matrix_mixer.crosspoints[input][output];
    xp->enabled = enabled;
    xp->phase_invert = phase;
    xp->gain_db = gain_db;
    xp->gain_linear = powf(10.0f, gain_db / 20.0f);

    WireCrosspoint wxp;
    memset(&wxp, 0, sizeof(wxp));
    wxp.enabled = enabled ? 1 : 0;
    wxp.phase_invert = phase ? 1 : 0;
    wxp.gain_db = gain_db;
    uint16_t off = (uint16_t)(offsetof(WireBulkParams, crosspoints)
        + ((uint16_t)input * WIRE_MAX_OUTPUT_CHANNELS + output) * sizeof(WireCrosspoint));
    notify_param_write(off, sizeof(WireCrosspoint), &wxp);
    return VC_OK;
}

// ---- Per-output controls ----

static uint16_t route_out_wire_off(uint8_t out, size_t field) {
    return (uint16_t)(offsetof(WireBulkParams, outputs)
                      + (uint16_t)out * sizeof(WireOutputChannel) + field);
}

// Core 1 runs either the PDM output or the EQ worker outputs, never both.
static bool route_enable_conflicts(uint8_t out) {
    bool is_pdm = (out == NUM_OUTPUT_CHANNELS - 1);
    bool is_core1_eq = (out >= CORE1_EQ_FIRST_OUTPUT && out <= CORE1_EQ_LAST_OUTPUT);
    if (is_pdm) {
        for (int i = CORE1_EQ_FIRST_OUTPUT; i <= CORE1_EQ_LAST_OUTPUT; i++)
            if (matrix_mixer.outputs[i].enabled) return true;
    } else if (is_core1_eq) {
        if (matrix_mixer.outputs[NUM_OUTPUT_CHANNELS - 1].enabled) return true;
    }
    return false;
}

static VcResult route_out_enable_get(VcCtx *c) {
    vc_out_u8(c, 0, matrix_mixer.outputs[c->addr].enabled);
    return VC_OK;
}

static VcResult route_out_enable_set(VcCtx *c) {
    uint8_t out = (uint8_t)c->addr;
    bool want_enable = (vc_in_u8(c, 0) != 0);
    if (want_enable && route_enable_conflicts(out)) return VC_ERR_STATE;

    matrix_mixer.outputs[out].enabled = want_enable ? 1 : 0;

    Core1Mode new_mode = derive_core1_mode();
    if (new_mode != core1_mode) {
        core1_mode = new_mode;
#if ENABLE_SUB
        pdm_set_enabled(new_mode == CORE1_MODE_PDM);
#endif
        __sev();
    }

    uint8_t v = matrix_mixer.outputs[out].enabled ? 1 : 0;
    notify_param_write(route_out_wire_off(out, offsetof(WireOutputChannel, enabled)), 1, &v);
    return VC_OK;
}

static VcResult route_out_gain_get(VcCtx *c) {
    vc_out_f32(c, 0, matrix_mixer.outputs[c->addr].gain_db);
    return VC_OK;
}

static VcResult route_out_gain_set(VcCtx *c) {
    uint8_t out = (uint8_t)c->addr;
    float db = vc_in_f32(c, 0);
    if (!isfinite(db)) return VC_ERR_VALUE;
    matrix_mixer.outputs[out].gain_db = db;
    matrix_mixer.outputs[out].gain_linear = powf(10.0f, db / 20.0f);
    notify_param_write(route_out_wire_off(out, offsetof(WireOutputChannel, gain_db)),
                       sizeof(float), &db);
    return VC_OK;
}

static VcResult route_out_mute_get(VcCtx *c) {
    vc_out_u8(c, 0, matrix_mixer.outputs[c->addr].mute);
    return VC_OK;
}

static VcResult route_out_mute_set(VcCtx *c) {
    uint8_t out = (uint8_t)c->addr;
    uint8_t v = vc_in_u8(c, 0);
    matrix_mixer.outputs[out].mute = v;
    notify_param_write(route_out_wire_off(out, offsetof(WireOutputChannel, mute)), 1, &v);
    return VC_OK;
}

static VcResult route_out_delay_get(VcCtx *c) {
    vc_out_f32(c, 0, matrix_mixer.outputs[c->addr].delay_ms);
    return VC_OK;
}

static VcResult route_out_delay_set(VcCtx *c) {
    uint8_t out = (uint8_t)c->addr;
    float ms = vc_in_f32(c, 0);
    if (!isfinite(ms)) return VC_ERR_VALUE;
    if (ms < 0) ms = 0;
    matrix_mixer.outputs[out].delay_ms = ms;
    channel_delays_ms[CH_OUT_1 + out] = ms;
    dsp_update_delay_samples((float)audio_state.freq);

    notify_param_write(route_out_wire_off(out, offsetof(WireOutputChannel, delay_ms)),
                       sizeof(float), &ms);
    notify_param_write((uint16_t)(offsetof(WireBulkParams, delays.delay_ms)
                                  + (CH_OUT_1 + out) * sizeof(float)),
                       sizeof(float), &ms);
    return VC_OK;
}

static VcResult route_out_conflict_get(VcCtx *c) {
    vc_out_u8(c, 0, route_enable_conflicts((uint8_t)c->addr) ? 1 : 0);
    return VC_OK;
}

// ---- Per-channel delay and name ----

static VcResult route_ch_delay_get(VcCtx *c) {
    vc_out_f32(c, 0, channel_delays_ms[c->addr]);
    return VC_OK;
}

static VcResult route_ch_delay_set(VcCtx *c) {
    uint8_t ch = (uint8_t)c->addr;
    float ms = vc_in_f32(c, 0);
    if (!isfinite(ms)) return VC_ERR_VALUE;
    if (ms < 0) ms = 0;
    channel_delays_ms[ch] = ms;
    dsp_update_delay_samples((float)audio_state.freq);
    notify_param_write((uint16_t)(offsetof(WireBulkParams, delays.delay_ms) + ch * sizeof(float)),
                       sizeof(float), &ms);
    return VC_OK;
}

static VcResult route_ch_name_get(VcCtx *c) {
    memcpy(c->out, channel_names[c->addr], PRESET_NAME_LEN);
    return VC_OK;
}

static VcResult route_ch_name_set(VcCtx *c) {
    uint8_t ch = (uint8_t)c->addr;
    memset(channel_names[ch], 0, PRESET_NAME_LEN);
    size_t copy_len = c->in_len < (PRESET_NAME_LEN - 1) ? c->in_len : (PRESET_NAME_LEN - 1);
    memcpy(channel_names[ch], c->in, copy_len);
    notify_param_write((uint16_t)(offsetof(WireBulkParams, channel_names.names)
                                  + (uint16_t)ch * WIRE_NAME_LEN),
                       WIRE_NAME_LEN, channel_names[ch]);
    return VC_OK;
}

// ---- Preamp ----

static VcResult route_preamp_get(VcCtx *c) {
    vc_out_f32(c, 0, global_preamp_db[c->addr]);
    return VC_OK;
}

static VcResult route_preamp_set(VcCtx *c) {
    float db = vc_in_f32(c, 0);
    if (!isfinite(db)) return VC_ERR_VALUE;
    update_preamp((uint8_t)c->addr, db);
    return VC_OK;
}

// ---- Master volume ----

static VcResult route_master_vol_get(VcCtx *c) {
    vc_out_f32(c, 0, master_volume_db);
    return VC_OK;
}

static VcResult route_master_vol_set(VcCtx *c) {
    float db = vc_in_f32(c, 0);
    if (!isfinite(db)) return VC_ERR_VALUE;
    // The bracket lets NOTIFY_SUPPRESS_HOST_ECHO filter host-originated echoes.
    notify_master_vol_host_initiated = true;
    update_master_volume(db);
    notify_master_vol_host_initiated = false;
    return VC_OK;
}

static VcResult route_master_mode_get(VcCtx *c) {
    uint16_t occupied;
    uint8_t startup, def_slot, last_active, inc_pins, mv_mode;
    preset_get_directory(&occupied, &startup, &def_slot, &last_active, &inc_pins, &mv_mode);
    vc_out_u8(c, 0, mv_mode);
    return VC_OK;
}

// The directory flash write runs on the main loop.
static VcResult route_master_mode_set(VcCtx *c) {
    uint8_t m = vc_in_u8(c, 0);
    if (m > MASTER_VOLUME_MODE_WITH_PRESET) m = MASTER_VOLUME_MODE_INDEPENDENT;
    flash_set_master_volume_mode_val = m;
    __dmb();
    flash_set_master_volume_mode_pending = true;
    return VC_OK;
}

static VcResult route_master_save_set(VcCtx *c) {
    (void)c;
    flash_save_master_volume_pending = true;
    __dmb();
    return VC_OK;
}

static VcResult route_master_saved_get(VcCtx *c) {
    vc_out_f32(c, 0, preset_get_saved_master_volume());
    return VC_OK;
}

// ---- User volume and mute ----

static VcResult route_user_vol_get(VcCtx *c) {
    vc_out_f32(c, 0, (float)audio_state.volume / 256.0f);
    return VC_OK;
}

static VcResult route_user_vol_set(VcCtx *c) {
    float db = vc_in_f32(c, 0);
    if (!isfinite(db)) return VC_ERR_VALUE;
    update_user_volume(db);
    return VC_OK;
}

static VcResult route_user_mute_get(VcCtx *c) {
    vc_out_u8(c, 0, user_mute ? 1 : 0);
    return VC_OK;
}

static VcResult route_user_mute_set(VcCtx *c) {
    user_mute = (vc_in_u8(c, 0) != 0);
    uint8_t v = user_mute ? 1 : 0;
    notify_param_write(offsetof(WireBulkParams, user_volume.user_mute), sizeof(uint8_t), &v);
    return VC_OK;
}

#define VC_PAGE_DEF "pages/route.def"
#include "vc_page_table.inc"
