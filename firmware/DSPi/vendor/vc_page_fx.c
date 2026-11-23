/*
 * vc_page_fx.c: FX page (0x08). Command set: vendor_command_overhaul_plan.md section 5.8.
 * Clamps, pending flags and notifications match the 1.1.6 behaviour of each setting.
 */

#include <stddef.h>
#include <math.h>
#include "vc_core.h"
#include "config.h"
#include "usb_audio.h"
#include "crossfeed.h"
#include "subharm.h"
#include "psybass.h"
#include "tube.h"
#include "spatial.h"
#include "upmix.h"
#include "audio_pipeline.h"   // active_input_channel_count() for the upmix status
#include "bulk_params.h"
#include "notify.h"

// ---- Crossfeed ----

static VcResult fx_crossfeed_enable_get(VcCtx *c) {
    c->out[0] = crossfeed_config.enabled ? 1 : 0;
    return VC_OK;
}

static VcResult fx_crossfeed_enable_set(VcCtx *c) {
    crossfeed_config.enabled = (c->in[0] != 0);
    crossfeed_update_pending = true;
    uint8_t v = crossfeed_config.enabled ? 1 : 0;
    notify_param_write(offsetof(WireBulkParams, crossfeed.enabled), 1, &v);
    return VC_OK;
}

static VcResult fx_crossfeed_preset_get(VcCtx *c) {
    c->out[0] = crossfeed_config.preset;
    return VC_OK;
}

static VcResult fx_crossfeed_preset_set(VcCtx *c) {
    uint8_t preset = c->in[0];
    if (preset > CROSSFEED_PRESET_CUSTOM) return VC_ERR_VALUE;
    crossfeed_config.preset = preset;
    crossfeed_update_pending = true;
    notify_param_write(offsetof(WireBulkParams, crossfeed.preset), 1, &preset);
    return VC_OK;
}

// ARG 0 = custom cutoff, 1 = custom feed. Both only recompute under the custom preset.
static VcResult fx_crossfeed_float_get(VcCtx *c) {
    vc_out_f32(c, 0, c->arg ? crossfeed_config.custom_feed_db : crossfeed_config.custom_fc);
    return VC_OK;
}

static VcResult fx_crossfeed_float_set(VcCtx *c) {
    float val = vc_in_f32(c, 0);
    if (!isfinite(val)) return VC_ERR_VALUE;
    if (c->arg == 0) {
        val = vc_clampf(val, CROSSFEED_FREQ_MIN, CROSSFEED_FREQ_MAX);
        crossfeed_config.custom_fc = val;
    } else {
        val = vc_clampf(val, CROSSFEED_FEED_MIN, CROSSFEED_FEED_MAX);
        crossfeed_config.custom_feed_db = val;
    }
    if (crossfeed_config.preset == CROSSFEED_PRESET_CUSTOM) {
        crossfeed_update_pending = true;
    }
    notify_param_write(c->arg ? offsetof(WireBulkParams, crossfeed.custom_feed_db)
                              : offsetof(WireBulkParams, crossfeed.custom_fc),
                       sizeof(float), &val);
    return VC_OK;
}

static VcResult fx_crossfeed_itd_get(VcCtx *c) {
    c->out[0] = crossfeed_config.itd_enabled ? 1 : 0;
    return VC_OK;
}

static VcResult fx_crossfeed_itd_set(VcCtx *c) {
    crossfeed_config.itd_enabled = (c->in[0] != 0);
    crossfeed_update_pending = true;
    uint8_t v = crossfeed_config.itd_enabled ? 1 : 0;
    notify_param_write(offsetof(WireBulkParams, crossfeed.itd_enabled), 1, &v);
    return VC_OK;
}

static VcResult fx_crossfeed_outputs_get(VcCtx *c) {
    c->out[0] = crossfeed_config.output_pair_mask;
    return VC_OK;
}

// The pipeline reads the mask live each packet, so no recompute is raised.
static VcResult fx_crossfeed_outputs_set(VcCtx *c) {
    uint8_t v = c->in[0] & ((1u << NUM_SPDIF_INSTANCES) - 1);
    crossfeed_config.output_pair_mask = v;
    notify_param_write(offsetof(WireBulkParams, crossfeed.output_pair_mask), 1, &v);
    return VC_OK;
}

// ---- Float parameter tables shared by subharm and psybass ----

typedef struct {
    volatile float *field;
    float           min, max;
    uint16_t        wire;      // WireBulkParams offset for the notification
} FxFloatParam;

static VcResult fx_float_get(VcCtx *c, const FxFloatParam *p) {
    vc_out_f32(c, 0, *p->field);
    return VC_OK;
}

static VcResult fx_float_set(VcCtx *c, const FxFloatParam *p, volatile bool *pending) {
    float raw = vc_in_f32(c, 0);
    if (!isfinite(raw)) return VC_ERR_VALUE;
    float val = vc_clampf(raw, p->min, p->max);
    *p->field = val;
    *pending = true;
    notify_param_write(p->wire, sizeof(float), &val);
    return VC_OK;
}

// ---- Subharmonic synth ----

enum { FX_SH_LOW, FX_SH_HIGH, FX_SH_TOP, FX_SH_BOOST, FX_SH_DEPTH, FX_SH_HOLD, FX_SH_CEILING };

#define FX_SH(f) ((uint16_t)offsetof(WireBulkParams, subharm.f))
static const FxFloatParam fx_subharm_floats[] = {
    [FX_SH_LOW]     = { &subharm_config.low_db,         SUBHARM_LEVEL_MIN,   SUBHARM_LEVEL_MAX,   FX_SH(low_db) },
    [FX_SH_HIGH]    = { &subharm_config.high_db,        SUBHARM_LEVEL_MIN,   SUBHARM_LEVEL_MAX,   FX_SH(high_db) },
    [FX_SH_TOP]     = { &subharm_config.top_db,         SUBHARM_LEVEL_MIN,   SUBHARM_LEVEL_MAX,   FX_SH(top_db) },
    [FX_SH_BOOST]   = { &subharm_config.boost_db,       SUBHARM_BOOST_MIN,   SUBHARM_BOOST_MAX,   FX_SH(boost_db) },
    [FX_SH_DEPTH]   = { &subharm_config.select_depth,   SUBHARM_DEPTH_MIN,   SUBHARM_DEPTH_MAX,   FX_SH(select_depth) },
    [FX_SH_HOLD]    = { &subharm_config.select_hold_ms, SUBHARM_HOLD_MIN,    SUBHARM_HOLD_MAX,    FX_SH(select_hold_ms) },
    [FX_SH_CEILING] = { &subharm_config.ceiling_db,     SUBHARM_CEILING_MIN, SUBHARM_CEILING_MAX, FX_SH(ceiling_db) },
};
#undef FX_SH

static VcResult fx_subharm_float_get(VcCtx *c) {
    return fx_float_get(c, &fx_subharm_floats[c->arg]);
}

static VcResult fx_subharm_float_set(VcCtx *c) {
    return fx_float_set(c, &fx_subharm_floats[c->arg], &subharm_update_pending);
}

static VcResult fx_subharm_enable_get(VcCtx *c) {
    c->out[0] = subharm_config.enabled ? 1 : 0;
    return VC_OK;
}

static VcResult fx_subharm_enable_set(VcCtx *c) {
    subharm_config.enabled = (c->in[0] != 0);
    subharm_update_pending = true;
    uint8_t v = subharm_config.enabled ? 1 : 0;
    notify_param_write(offsetof(WireBulkParams, subharm.enabled), 1, &v);
    return VC_OK;
}

static VcResult fx_subharm_mask_get(VcCtx *c) {
    vc_out_u16(c, 0, subharm_config.output_mask);
    return VC_OK;
}

// Read live each packet, so no recompute is raised.
static VcResult fx_subharm_mask_set(VcCtx *c) {
    subharm_config.output_mask = vc_in_u16(c, 0);
    uint16_t m = subharm_config.output_mask;
    notify_param_write(offsetof(WireBulkParams, subharm.output_mask), 2, (const uint8_t *)&m);
    return VC_OK;
}

static VcResult fx_subharm_select_get(VcCtx *c) {
    c->out[0] = subharm_config.select_mode;
    return VC_OK;
}

static VcResult fx_subharm_select_set(VcCtx *c) {
    uint8_t m = c->in[0];
    if (m > SUBHARM_SELECT_MODE_MAX) m = SUBHARM_SELECT_MODE_MAX;
    subharm_config.select_mode = m;
    subharm_update_pending = true;
    notify_param_write(offsetof(WireBulkParams, subharm.select_mode), 1, &m);
    return VC_OK;
}

static VcResult fx_subharm_link_get(VcCtx *c) {
    c->out[0] = subharm_config.link_pairs ? 1 : 0;
    return VC_OK;
}

// Read live each packet, so no recompute is raised.
static VcResult fx_subharm_link_set(VcCtx *c) {
    subharm_config.link_pairs = (c->in[0] != 0);
    uint8_t v = subharm_config.link_pairs ? 1 : 0;
    notify_param_write(offsetof(WireBulkParams, subharm.link_pairs), 1, &v);
    return VC_OK;
}

static VcResult fx_subharm_solo_get(VcCtx *c) {
    c->out[0] = subharm_config.solo ? 1 : 0;
    return VC_OK;
}

// Solo has no wire offset and no notify, so a saved config can never boot in it.
static VcResult fx_subharm_solo_set(VcCtx *c) {
    subharm_config.solo = (c->in[0] != 0);
    return VC_OK;
}

// Computed from a snapshot of the live config, not a cached value a SET could race.
static VcResult fx_subharm_headroom_get(VcCtx *c) {
    SubharmConfig cfg = subharm_config;
    vc_out_f32(c, 0, subharm_headroom_db(&cfg));
    return VC_OK;
}

static VcResult fx_subharm_meter_get(VcCtx *c) {
    for (uint8_t k = 0; k < NUM_OUTPUT_CHANNELS; k++)
        vc_out_u16(c, (uint16_t)(2 * k), subharm_meter_u16(k));
    c->out_len = 2 * NUM_OUTPUT_CHANNELS;
    return VC_OK;
}

// ---- Psychoacoustic bass ----

enum { FX_PB_CUTOFF, FX_PB_HARMONICS, FX_PB_DRIVE, FX_PB_CHARACTER, FX_PB_ORIGINAL };

#define FX_PB(f) ((uint16_t)offsetof(WireBulkParams, psybass.f))
static const FxFloatParam fx_psybass_floats[] = {
    [FX_PB_CUTOFF]    = { &psybass_config.cutoff_hz,     PSYBASS_CUTOFF_MIN,    PSYBASS_CUTOFF_MAX,    FX_PB(cutoff_hz) },
    [FX_PB_HARMONICS] = { &psybass_config.harmonics_db,  PSYBASS_HARMONICS_MIN, PSYBASS_HARMONICS_MAX, FX_PB(harmonics_db) },
    [FX_PB_DRIVE]     = { &psybass_config.drive_db,      PSYBASS_DRIVE_MIN,     PSYBASS_DRIVE_MAX,     FX_PB(drive_db) },
    [FX_PB_CHARACTER] = { &psybass_config.character_pct, PSYBASS_CHARACTER_MIN, PSYBASS_CHARACTER_MAX, FX_PB(character_pct) },
    [FX_PB_ORIGINAL]  = { &psybass_config.original_db,   PSYBASS_ORIGINAL_MIN,  PSYBASS_ORIGINAL_MAX,  FX_PB(original_db) },
};
#undef FX_PB

static VcResult fx_psybass_float_get(VcCtx *c) {
    return fx_float_get(c, &fx_psybass_floats[c->arg]);
}

static VcResult fx_psybass_float_set(VcCtx *c) {
    return fx_float_set(c, &fx_psybass_floats[c->arg], &psybass_update_pending);
}

static VcResult fx_psybass_enable_get(VcCtx *c) {
    c->out[0] = psybass_config.enabled ? 1 : 0;
    return VC_OK;
}

static VcResult fx_psybass_enable_set(VcCtx *c) {
    psybass_config.enabled = (c->in[0] != 0);
    psybass_update_pending = true;
    uint8_t v = psybass_config.enabled ? 1 : 0;
    notify_param_write(offsetof(WireBulkParams, psybass.enabled), 1, &v);
    return VC_OK;
}

static VcResult fx_psybass_mask_get(VcCtx *c) {
    vc_out_u16(c, 0, psybass_config.output_mask);
    return VC_OK;
}

// Read live each packet, so no recompute is raised.
static VcResult fx_psybass_mask_set(VcCtx *c) {
    psybass_config.output_mask = vc_in_u16(c, 0);
    uint16_t m = psybass_config.output_mask;
    notify_param_write(offsetof(WireBulkParams, psybass.output_mask), 2, (const uint8_t *)&m);
    return VC_OK;
}

// ---- Tube preamp (ARG = TUBE_PARAM_*) ----

static VcResult fx_tube_get(VcCtx *c) {
    float v;
    if (!tube_get_param(c->arg, &v)) return VC_ERR_UNKNOWN;
    vc_out_f32(c, 0, v);
    return VC_OK;
}

// tube_set_param owns clamping, the pending flag and every notification. It
// ignores NaN, which this page rejects instead.
static VcResult fx_tube_set(VcCtx *c) {
    float v = vc_in_f32(c, 0);
    if (isnan(v)) return VC_ERR_VALUE;
    return tube_set_param(c->arg, v) ? VC_OK : VC_ERR_UNKNOWN;
}

// ---- Stereo upmixer (RP2350 only) ----

#if PICO_RP2350
// Profile addressing: wValue is the profile; the dispatcher does not range-check INDEX.
#define FX_UPMIX_PROFILE(c) ((c)->addr < UPMIX_NUM_PROFILES ? (uint8_t)(c)->addr : 0xFF)

static VcResult fx_upmix_config_get(VcCtx *c) {
    uint8_t p = FX_UPMIX_PROFILE(c);
    if (p == 0xFF) return VC_ERR_ADDRESS;
    UpmixConfigPacket pkt;
    upmix_config_to_packet(p, &pkt);
    memcpy(c->out, &pkt, UPMIX_PACKET_V1_SIZE);
    return VC_OK;
}

static bool fx_upmix_route_ok(const UpmixRoute *r) {
    return !(r->out_mask & (uint16_t)~UPMIX_OUT_MASK) && !(r->flags & (uint8_t)~UPMIX_ROUTE_INVERT);
}

// The packet is 80 bytes, past VC_SET_MAX (64), so it travels as v1's 44 bytes
// here and the v2 tail in UPMIX_CONFIG_EXT; each keeps the other's bytes.
static VcResult fx_upmix_config_set(VcCtx *c) {
    uint8_t p = FX_UPMIX_PROFILE(c);
    if (p == 0xFF) return VC_ERR_ADDRESS;
    UpmixConfigPacket pkt;
    memcpy(&pkt, c->in, UPMIX_PACKET_V1_SIZE);
    const float f[] = { pkt.strength_pct, pkt.center_width_pct, pkt.corr_threshold_pct,
                        pkt.attack_ms, pkt.release_ms, pkt.detector_hpf_hz,
                        pkt.surround_delay_ms, pkt.surround_hpf_hz, pkt.surround_lpf_hz,
                        pkt.decorr_pct };
    for (unsigned i = 0; i < sizeof(f) / sizeof(f[0]); i++)
        if (!isfinite(f[i])) return VC_ERR_VALUE;
    upmix_packet_to_config(p, &pkt, UPMIX_PKT_HEAD);
    upmix_update_pending = true;
    upmix_notify_config(p);
    return VC_OK;
}

static VcResult fx_upmix_ext_get(VcCtx *c) {
    uint8_t p = FX_UPMIX_PROFILE(c);
    if (p == 0xFF) return VC_ERR_ADDRESS;
    UpmixConfigPacket pkt;
    upmix_config_to_packet(p, &pkt);
    memcpy(c->out, (const uint8_t *)&pkt + UPMIX_PACKET_V1_SIZE, sizeof(pkt) - UPMIX_PACKET_V1_SIZE);
    return VC_OK;
}

// Out-of-range sources, counts, masks and flags are refused, not clamped.
static VcResult fx_upmix_ext_set(VcCtx *c) {
    uint8_t p = FX_UPMIX_PROFILE(c);
    if (p == 0xFF) return VC_ERR_ADDRESS;
    UpmixConfigPacket pkt;
    upmix_config_to_packet(p, &pkt);
    memcpy((uint8_t *)&pkt + UPMIX_PACKET_V1_SIZE, c->in, sizeof(pkt) - UPMIX_PACKET_V1_SIZE);
    if (!isfinite(pkt.back_offset_ms) || pkt.src_a >= NUM_INPUT_CHANNELS || pkt.src_b >= NUM_INPUT_CHANNELS ||
        (pkt.active_counts & (uint8_t)~UPMIX_COUNTS_MASK)) return VC_ERR_VALUE;
    for (int g = 0; g < UPMIX_NUM_GEN; g++)
        if (!fx_upmix_route_ok(&pkt.routes[g])) return VC_ERR_VALUE;
    upmix_packet_to_config(p, &pkt, UPMIX_PKT_TAIL);
    upmix_update_pending = true;
    upmix_notify_config(p);
    return VC_OK;
}

static VcResult fx_upmix_status_get(VcCtx *c) {
    UpmixStatus st;
    uint16_t out_en = 0;
    for (int o = 0; o < NUM_OUTPUT_CHANNELS; o++)
        if (matrix_mixer.outputs[o].enabled) out_en |= (uint16_t)(1u << o);
    upmix_get_status(&st, active_input_channel_count(), out_en);
    memcpy(c->out, &st, sizeof(st));
    return VC_OK;
}

// wValue = (profile << 8) | generated channel.
static volatile UpmixRoute *fx_upmix_route_at(const VcCtx *c) {
    uint8_t p = (uint8_t)(c->addr >> 8), g = (uint8_t)c->addr;
    return (p < UPMIX_NUM_PROFILES && g < UPMIX_NUM_GEN) ? &upmix_profiles[p].routes[g] : NULL;
}

static VcResult fx_upmix_route_get(VcCtx *c) {
    volatile UpmixRoute *r = fx_upmix_route_at(c);
    if (!r) return VC_ERR_ADDRESS;
    UpmixRoute v = { r->out_mask, r->trim_q1, r->flags };
    memcpy(c->out, &v, sizeof(v));
    return VC_OK;
}

static VcResult fx_upmix_route_set(VcCtx *c) {
    volatile UpmixRoute *r = fx_upmix_route_at(c);
    if (!r) return VC_ERR_ADDRESS;
    UpmixRoute v;
    memcpy(&v, c->in, sizeof(v));
    if (!fx_upmix_route_ok(&v)) return VC_ERR_VALUE;
    r->out_mask = v.out_mask;
    r->trim_q1 = v.trim_q1 < UPMIX_TRIM_MIN_Q1 ? UPMIX_TRIM_MIN_Q1
               : (v.trim_q1 > UPMIX_TRIM_MAX_Q1 ? UPMIX_TRIM_MAX_Q1 : v.trim_q1);
    r->flags = v.flags;
    upmix_update_pending = true;
    upmix_notify_config((uint8_t)(c->addr >> 8));
    return VC_OK;
}

static VcResult fx_upmix_param_get(VcCtx *c) {
    uint8_t p = FX_UPMIX_PROFILE(c);
    if (p == 0xFF) return VC_ERR_ADDRESS;
    volatile const UpmixConfig *u = &upmix_profiles[p];
    float v = 0.0f;
    switch (c->arg) {
        case UPMIX_PARAM_ENABLED:       v = u->enabled ? 1.0f : 0.0f; break;
        case UPMIX_PARAM_CENTER_MODE:   v = (float)u->center_mode; break;
        case UPMIX_PARAM_SURROUND_MODE: v = (float)u->surround_mode; break;
        case UPMIX_PARAM_STRENGTH:      v = u->strength_pct; break;
        case UPMIX_PARAM_CENTER_WIDTH:  v = u->center_width_pct; break;
        case UPMIX_PARAM_THRESHOLD:     v = u->corr_threshold_pct; break;
        case UPMIX_PARAM_ATTACK:        v = u->attack_ms; break;
        case UPMIX_PARAM_RELEASE:       v = u->release_ms; break;
        case UPMIX_PARAM_DET_HPF:       v = u->detector_hpf_hz; break;
        case UPMIX_PARAM_SUR_DELAY:     v = u->surround_delay_ms; break;
        case UPMIX_PARAM_SUR_HPF:       v = u->surround_hpf_hz; break;
        case UPMIX_PARAM_SUR_LPF:       v = u->surround_lpf_hz; break;
        case UPMIX_PARAM_DECORR:        v = u->decorr_pct; break;
        case UPMIX_PARAM_PRESENCE:      v = u->presence_db; break;
        case UPMIX_PARAM_SRC_A:         v = (float)u->src_a; break;
        case UPMIX_PARAM_SRC_B:         v = (float)u->src_b; break;
        case UPMIX_PARAM_ACTIVE_COUNTS: v = (float)u->active_counts; break;
        case UPMIX_PARAM_BACK_OFFSET:   v = u->back_offset_ms; break;
        default:                        return VC_ERR_UNKNOWN;
    }
    vc_out_f32(c, 0, v);
    return VC_OK;
}

// Mode, enable, source and count values are rounded; float ranges are clamped
// at recompute.  A source or count no profile can hold is refused.
static VcResult fx_upmix_param_set(VcCtx *c) {
    uint8_t p = FX_UPMIX_PROFILE(c);
    if (p == 0xFF) return VC_ERR_ADDRESS;
    volatile UpmixConfig *u = &upmix_profiles[p];
    float v = vc_in_f32(c, 0);
    if (!isfinite(v)) return VC_ERR_VALUE;
    // Saturate before rounding: lrintf of a value past long is unspecified.
    long m = v <= -1000.0f ? -1000 : (v >= 1000.0f ? 1000 : lrintf(v));
    switch (c->arg) {
        case UPMIX_PARAM_ENABLED:
            u->enabled = (v != 0.0f);
            break;
        case UPMIX_PARAM_CENTER_MODE:
            u->center_mode = upmix_clamp_center_mode(m);
            break;
        case UPMIX_PARAM_SURROUND_MODE:
            u->surround_mode = (m < 0) ? 0 : (m > 2) ? 2 : (uint8_t)m;
            break;
        case UPMIX_PARAM_STRENGTH:      u->strength_pct = v; break;
        case UPMIX_PARAM_CENTER_WIDTH:  u->center_width_pct = v; break;
        case UPMIX_PARAM_THRESHOLD:     u->corr_threshold_pct = v; break;
        case UPMIX_PARAM_ATTACK:        u->attack_ms = v; break;
        case UPMIX_PARAM_RELEASE:       u->release_ms = v; break;
        case UPMIX_PARAM_DET_HPF:       u->detector_hpf_hz = v; break;
        case UPMIX_PARAM_SUR_DELAY:     u->surround_delay_ms = v; break;
        case UPMIX_PARAM_SUR_HPF:       u->surround_hpf_hz = v; break;
        case UPMIX_PARAM_SUR_LPF:       u->surround_lpf_hz = v; break;
        case UPMIX_PARAM_DECORR:        u->decorr_pct = v; break;
        case UPMIX_PARAM_PRESENCE:      u->presence_db = v; break;
        case UPMIX_PARAM_SRC_A:
        case UPMIX_PARAM_SRC_B:
            if (m < 0 || m >= NUM_INPUT_CHANNELS) return VC_ERR_VALUE;
            if (c->arg == UPMIX_PARAM_SRC_A) u->src_a = (uint8_t)m; else u->src_b = (uint8_t)m;
            break;
        case UPMIX_PARAM_ACTIVE_COUNTS:
            if (m < 0 || (m & ~(long)UPMIX_COUNTS_MASK)) return VC_ERR_VALUE;
            u->active_counts = (uint8_t)m;
            break;
        case UPMIX_PARAM_BACK_OFFSET:   u->back_offset_ms = v; break;
        default:                        return VC_ERR_UNKNOWN;
    }
    upmix_update_pending = true;
    upmix_notify_config(p);
    return VC_OK;
}
#endif

// ---- Stereo spatializer (both platforms) ----

static VcResult fx_spatial_config_get(VcCtx *c) {
    SpatialConfigPacket p;
    spatial_config_to_packet(&p);
    memcpy(c->out, &p, sizeof(p));
    return VC_OK;
}

static VcResult fx_spatial_config_set(VcCtx *c) {
    SpatialConfigPacket p;
    memcpy(&p, c->in, sizeof(p));
    return spatial_set_config(&p) ? VC_OK : VC_ERR_VALUE;
}

static VcResult fx_spatial_u8_get(VcCtx *c) {
    c->out[0] = (uint8_t)spatial_get_param(c->arg);
    return VC_OK;
}

static VcResult fx_spatial_u8_set(VcCtx *c) {
    uint8_t v = c->in[0];
    if (c->arg == SPATIAL_PARAM_ENABLED) v = v != 0;
    return spatial_set_param(c->arg, (float)v) ? VC_OK : VC_ERR_VALUE;
}

static VcResult fx_spatial_f32_get(VcCtx *c) {
    vc_out_f32(c, 0, spatial_get_param(c->arg));
    return VC_OK;
}

static VcResult fx_spatial_f32_set(VcCtx *c) {
    return spatial_set_param(c->arg, vc_in_f32(c, 0)) ? VC_OK : VC_ERR_VALUE;
}

#define VC_PAGE_DEF "pages/fx.def"
#include "vc_page_table.inc"
