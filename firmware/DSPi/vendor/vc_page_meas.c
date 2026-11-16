/*
 * vc_page_meas.c: MEAS page (0x09). Command set: vendor_command_overhaul_plan.md section 5.9.
 * Replies match the legacy opcodes byte for byte; RTA_BINS replies stop at the reply buffer.
 */

#include "vc_core.h"
#include "config.h"
#include "audio_pipeline.h"
#include "notify.h"
#include "rta.h"
#include "siggen.h"

// ---- Meters and clip flags ----

static VcResult meas_levels_get(VcCtx *c) {
    for (int i = 0; i < NUM_CHANNELS; i++)
        vc_out_u16(c, (uint16_t)(i * 2), global_status.peaks[i]);
    uint16_t off = NUM_CHANNELS * 2;
    c->out[off + 0] = global_status.cpu0_load;
    c->out[off + 1] = global_status.cpu1_load;
    vc_out_u32(c, (uint16_t)(off + 2), global_status.clip_flags);
    c->out[off + 6] = active_input_channel_count();
    c->out_len = (uint16_t)(off + 7);
    return VC_OK;
}

static VcResult meas_clip_flags_get(VcCtx *c) {
    vc_out_u32(c, 0, global_status.clip_flags);
    return VC_OK;
}

static VcResult meas_clip_clear_set(VcCtx *c) {
    (void)c;
    global_status.clip_flags = 0;
    return VC_OK;
}

// ---- Spectrum analyser ----

static VcResult meas_rta_config_get(VcCtx *c) {
    RtaConfig cfg;
    rta_get_config(&cfg);
    memcpy(c->out, &cfg, sizeof(cfg));
    return VC_OK;
}

static VcResult meas_rta_config_set(VcCtx *c) {
    return rta_set_config(c->in, c->in_len, vc_param_src(c->src)) ? VC_OK : VC_ERR_VALUE;
}

static VcResult meas_rta_caps_get(VcCtx *c) {
    RtaCaps caps;
    rta_get_caps(&caps);
    memcpy(c->out, &caps, sizeof(caps));
    return VC_OK;
}

// Chunk n here is the engine's chunk n + 1 (its chunk 0 never existed).
static VcResult meas_rta_band_centres_get(VcCtx *c) {
    if (c->addr >= 0xFF) return VC_ERR_ADDRESS;
    uint16_t n = rta_get_caps_centres((uint8_t)(c->addr + 1), c->out);
    if (n == 0) return VC_ERR_ADDRESS;
    c->out_len = n;
    return VC_OK;
}

// Any band or bin read is a keepalive, even one that is then refused.
static VcResult meas_rta_bands_get(VcCtx *c) {
    rta_note_read();
    if (c->addr > 0xFF) return VC_ERR_ADDRESS;
    RtaBandFrame frame;
    if (!rta_get_band_frame((uint8_t)c->addr, &frame)) return VC_ERR_ADDRESS;
    memcpy(c->out, &frame, sizeof(frame));
    return VC_OK;
}

// No lock: the frame repeats its seq as the final byte for the host to compare.
static VcResult meas_rta_bins_get(VcCtx *c) {
    rta_note_read();
    uint16_t flen = 0;
    const uint8_t *frame = rta_bin_frame(&flen);
    if (frame == NULL || flen == 0) return VC_ERR_STATE;
    if (c->addr >= flen) return VC_ERR_ADDRESS;
    vc_reply(c, frame + c->addr, (uint16_t)(flen - c->addr));
    return VC_OK;
}

static VcResult meas_rta_status_get(VcCtx *c) {
    RtaStatus st;
    rta_get_status(&st);
    memcpy(c->out, &st, sizeof(st));
    return VC_OK;
}

// Runs in bulk_param_buf under the bulk lock. An empty live mask answers zero
// bytes, so a host can tell "nothing live" from a refusal.
static VcResult meas_rta_bands_all_get(VcCtx *c) {
    rta_note_read();
    uint16_t mask = rta_live_mask();
    uint16_t len = 0;
    for (uint8_t i = 0; i < RTA_MAX_TRACKED; i++) {
        if (!(mask & (1u << i))) continue;
        if (len + sizeof(RtaBandFrame) > c->out_cap) return VC_ERR_STATE;
        RtaBandFrame *dst = (RtaBandFrame *)&c->out[len];
        if (!rta_get_band_frame(i, dst)) memset(dst, 0, sizeof(*dst));
        len = (uint16_t)(len + sizeof(*dst));
    }
    c->out_len = len;
    return VC_OK;
}

static VcResult meas_rta_control_set(VcCtx *c) {
    return rta_control(c->arg) ? VC_OK : VC_ERR_UNKNOWN;
}

// ---- Signal generator ----

static VcResult meas_siggen_config_get(VcCtx *c) {
    SiggenConfig tmp;
    siggen_get_config(&tmp);
    memcpy(c->out, &tmp, sizeof(tmp));
    return VC_OK;
}

static VcResult meas_siggen_config_set(VcCtx *c) {
    return siggen_stage_config(c->in, c->in_len) ? VC_OK : VC_ERR_VALUE;
}

static VcResult meas_siggen_status_get(VcCtx *c) {
    SiggenStatus tmp;
    siggen_get_status(&tmp);
    memcpy(c->out, &tmp, sizeof(tmp));
    return VC_OK;
}

static VcResult meas_siggen_caps_get(VcCtx *c) {
    memcpy(c->out, siggen_caps_header(), sizeof(SiggenCapsHeader));
    return VC_OK;
}

static VcResult meas_siggen_type_desc_get(VcCtx *c) {
    if (c->addr > 0xFF) return VC_ERR_ADDRESS;
    const SiggenTypeDesc *d = siggen_caps_type((uint8_t)c->addr);
    if (d == NULL) return VC_ERR_ADDRESS;
    memcpy(c->out, d, sizeof(*d));
    return VC_OK;
}

// START is refused until a valid config has been applied; STOP and STOP_NOW always succeed.
static VcResult meas_siggen_control_set(VcCtx *c) {
    return siggen_control(c->arg) ? VC_OK : VC_ERR_STATE;
}

_Static_assert(sizeof(RtaConfig) == 12 && sizeof(RtaCaps) == 16 && sizeof(RtaStatus) == 24 &&
               sizeof(RtaBandFrame) == 82, "MEAS RTA reply sizes are in pages/meas.def");
_Static_assert(sizeof(SiggenConfig) == 36 && sizeof(SiggenStatus) == 16 &&
               sizeof(SiggenCapsHeader) == 8 && sizeof(SiggenTypeDesc) == 62,
               "MEAS siggen reply sizes are in pages/meas.def");
_Static_assert(2 * NUM_CHANNELS + 7 <= VC_REPLY_MAX, "MEAS.LEVELS must fit the reply buffer");

#define VC_PAGE_DEF "pages/meas.def"
#include "vc_page_table.inc"
