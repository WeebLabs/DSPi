/*
 * USB Feedback Controller: Q16.16 rate + fill servo
 *
 * Pure module: no Pico SDK dependencies, no hardware access.
 * All hardware interaction happens in the caller (uac1_driver_sof).
 */

#include "usb_feedback_controller.h"

// These run every USB SOF in ISR context; keep them in RAM under XIP builds.
#define FB_TIME_CRITICAL __attribute__((section(".time_critical.fb_ctrl")))

// ---------------------------------------------------------------------------
// Init / Reset
// ---------------------------------------------------------------------------

FB_TIME_CRITICAL
static void long_window_clear(usb_feedback_ctrl_t *ctrl) {
    ctrl->long_seeded = false;
    ctrl->long_valid  = false;
}

void fb_ctrl_init(usb_feedback_ctrl_t *ctrl) {
    ctrl->rate_estimate_q16     = 0;
    ctrl->nominal_rate_q16      = 0;
    ctrl->rate_long_q16         = 0;
    ctrl->frame_total           = 0;
    long_window_clear(ctrl);
    ctrl->fill_error_filtered   = 0;
    ctrl->servo_q16             = 0;
    ctrl->feedback_out_q16      = 0;
    ctrl->holdoff_remaining     = 0;
    ctrl->rate_valid            = false;
    ctrl->stream_active         = false;
    ctrl->need_baseline         = false;
    ctrl->last_frame            = 0;
    ctrl->last_total_words      = 0;
    ctrl->diag_updates          = 0;
    ctrl->diag_merged_windows   = 0;
    ctrl->diag_rejected         = 0;
    ctrl->diag_long_reanchors   = 0;
    ctrl->diag_fb_min_q16       = 0;
    ctrl->diag_fb_max_q16       = 0;
}

void fb_ctrl_reset(usb_feedback_ctrl_t *ctrl, uint32_t nominal_rate_q16) {
    ctrl->nominal_rate_q16      = nominal_rate_q16;
    ctrl->rate_estimate_q16     = nominal_rate_q16;
    long_window_clear(ctrl);
    ctrl->rate_valid            = true;
    ctrl->fill_error_filtered   = 0;
    ctrl->servo_q16             = 0;
    ctrl->holdoff_remaining     = FB_HOLDOFF_UPDATES;
    ctrl->feedback_out_q16      = nominal_rate_q16;
    ctrl->diag_fb_min_q16       = nominal_rate_q16;
    ctrl->diag_fb_max_q16       = nominal_rate_q16;
    ctrl->stream_active         = true;
    ctrl->need_baseline         = true;
}

void fb_ctrl_stream_stop(usb_feedback_ctrl_t *ctrl) {
    ctrl->stream_active         = false;
    ctrl->rate_valid            = false;
    long_window_clear(ctrl);
    ctrl->fill_error_filtered   = 0;
    ctrl->servo_q16             = 0;
    ctrl->holdoff_remaining     = 0;
    ctrl->feedback_out_q16      = ctrl->nominal_rate_q16;
}

// ---------------------------------------------------------------------------
// Long-window rate (Loop A primary)
// ---------------------------------------------------------------------------

FB_TIME_CRITICAL
static void long_window_seed(usb_feedback_ctrl_t *ctrl, uint32_t words) {
    ctrl->long_frames[0] = ctrl->long_frames[1] = ctrl->frame_total;
    ctrl->long_words[0]  = ctrl->long_words[1]  = words;
    ctrl->long_seeded = true;
}

FB_TIME_CRITICAL
static void long_window_drop(usb_feedback_ctrl_t *ctrl) {
    long_window_clear(ctrl);
    ctrl->diag_long_reanchors++;
}

// Call only on a clean window (see FB_LONG_ANCHOR_TOL_Q16): words is both a
// candidate anchor and the endpoint of this update's long-window sample.
FB_TIME_CRITICAL
static void long_window_update(usb_feedback_ctrl_t *ctrl, uint32_t words,
                               uint32_t rate_shift) {
    if (!ctrl->long_seeded) {
        long_window_seed(ctrl, words);
        return;
    }
    // den = span * words per frame (4 or 2) must stay below 2^16 for the 32-bit
    // remainder step. Rotation waits for a clean window, so a late one can push
    // the span past that; rotate early then, and drop only after a real gap.
    uint32_t span_max = 0xFFFFu >> (14 - rate_shift);
    uint32_t span = ctrl->frame_total - ctrl->long_frames[0];
    if (ctrl->frame_total - ctrl->long_frames[1] >= FB_LONG_HALF_FRAMES || span > span_max) {
        ctrl->long_frames[0] = ctrl->long_frames[1];
        ctrl->long_words[0]  = ctrl->long_words[1];
        ctrl->long_frames[1] = ctrl->frame_total;
        ctrl->long_words[1]  = words;
        span = ctrl->frame_total - ctrl->long_frames[0];
        if (span > span_max) {
            long_window_drop(ctrl);
            return;
        }
    }
    if (span < FB_LONG_MIN_FRAMES)
        return;

    uint32_t den = span << (14 - rate_shift);
    uint32_t delta = words - ctrl->long_words[0];
    uint32_t q = delta / den;
    if (q >> 15) {                  // garbage delta; q << 16 would wrap
        long_window_drop(ctrl);
        return;
    }
    uint32_t r = delta - q * den;
    uint32_t rate = (q << 16) + (((r << 16) + (den >> 1)) / den);

    int32_t offset = (int32_t)(rate - ctrl->nominal_rate_q16);
    int32_t sane = (int32_t)(ctrl->nominal_rate_q16 >> FB_LONG_SANE_SHIFT);
    if (offset > sane || offset < -sane) {
        long_window_drop(ctrl);
        return;
    }
    ctrl->rate_long_q16 = rate;
    ctrl->long_valid = true;
}

// ---------------------------------------------------------------------------
// SOF update (called every 1ms from USB IRQ)
// ---------------------------------------------------------------------------

FB_TIME_CRITICAL
void fb_ctrl_sof_update(usb_feedback_ctrl_t *ctrl,
                        uint32_t frame_count,
                        uint32_t current_total_words,
                        uint32_t rate_shift,
                        uint32_t consumer_fill_q16) {
    if (!ctrl->stream_active || !ctrl->rate_valid)
        return;

    frame_count &= FB_FRAME_MASK;

    // First call after reset: latch both anchors only. last_total_words is
    // stale here, so a delta would overflow the rate path.
    if (ctrl->need_baseline) {
        ctrl->last_frame = frame_count;
        ctrl->last_total_words = current_total_words;
        ctrl->frame_total = 0;
        long_window_clear(ctrl);    // an ISR inside fb_ctrl_reset may have seeded it
        ctrl->need_baseline = false;
        return;
    }

    // Decimate on the USB frame number, not on callback count: a SOF IRQ held
    // off past 1 ms merges two SOFs into one callback.
    uint32_t elapsed = (frame_count - ctrl->last_frame) & FB_FRAME_MASK;
    if (elapsed < FB_UPDATE_FRAMES)
        return;

    uint32_t delta_words = current_total_words - ctrl->last_total_words;
    ctrl->last_frame = frame_count;
    ctrl->last_total_words = current_total_words;
    ctrl->frame_total += elapsed;

    ctrl->diag_updates++;
    if (elapsed > FB_UPDATE_FRAMES)
        ctrl->diag_merged_windows++;

    if (delta_words == 0)
        return;  // DMA stalled, skip this cycle

    // -----------------------------------------------------------------------
    // Loop A bridge: short-window rate (IIR, α = 1/16, rounded)
    //
    // Normalize by the true frame span. Do not reject one half of a late-anchor
    // pair: its twin then goes uncancelled (see FB_RATE_SANE_SHIFT).
    // -----------------------------------------------------------------------
    bool clean = false;
    if ((delta_words >> (30 - rate_shift)) == 0) {   // keeps the shift below in range
        uint32_t rate_raw_q16 = (elapsed == FB_UPDATE_FRAMES)
            ? delta_words << rate_shift
            : ((delta_words << (rate_shift + 2)) + elapsed / 2) / elapsed;
        int32_t raw_offset = (int32_t)(rate_raw_q16 - ctrl->nominal_rate_q16);
        int32_t sane = (int32_t)(ctrl->nominal_rate_q16 >> FB_RATE_SANE_SHIFT);
        if (raw_offset <= sane && raw_offset >= -sane) {
            int32_t rate_error = (int32_t)(rate_raw_q16 - ctrl->rate_estimate_q16);
            clean = elapsed == FB_UPDATE_FRAMES &&
                    rate_error <= FB_LONG_ANCHOR_TOL_Q16 &&
                    rate_error >= -FB_LONG_ANCHOR_TOL_Q16;
            ctrl->rate_estimate_q16 += (uint32_t)round_div_pow2_s32(rate_error, FB_IIR_SHIFT);
        } else {
            ctrl->diag_rejected++;
        }
    } else {
        ctrl->diag_rejected++;
    }

    // Loop A primary: long-window count rate, fed only from clean windows.
    if (clean)
        long_window_update(ctrl, current_total_words, rate_shift);

    // -----------------------------------------------------------------------
    // Loop B: Fill-level servo (dual-slope proportional, direct measurement)
    //
    // Uses the slot ring fill directly. That is immune to the counter ratchet
    // that breaks epoch-relative produced/consumed tracking across producer
    // stalls.
    // -----------------------------------------------------------------------
    int32_t servo_q16 = 0;

    if (ctrl->holdoff_remaining > 0) {
        ctrl->holdoff_remaining--;
    } else {
        // Fill error in Q16.16 buffer-counts. Positive = overfull.
        int32_t fill_error_q16 = (int32_t)consumer_fill_q16 - FB_FILL_TARGET_Q16;

        // IIR filter: same α=1/16 as rate path
        int32_t fe_delta = fill_error_q16 - ctrl->fill_error_filtered;
        ctrl->fill_error_filtered += round_div_pow2_s32(fe_delta, FB_IIR_SHIFT);

        // Overfull → negative correction → host sends less.
        int32_t e = ctrl->fill_error_filtered;
        int64_t corr = (int64_t)FB_FILL_KP_Q16 * e;
        if (e > FB_FILL_KNEE_Q16)
            corr += (int64_t)FB_FILL_KP2_Q16 * (e - FB_FILL_KNEE_Q16);
        else if (e < -FB_FILL_KNEE_Q16)
            corr += (int64_t)FB_FILL_KP2_Q16 * (e + FB_FILL_KNEE_Q16);
        int32_t servo_raw = (int32_t)-(corr >> 16);

        if (servo_raw > FB_SERVO_CLAMP_Q16)
            servo_raw = FB_SERVO_CLAMP_Q16;
        if (servo_raw < -FB_SERVO_CLAMP_Q16)
            servo_raw = -FB_SERVO_CLAMP_Q16;

        servo_q16 = servo_raw;
    }
    ctrl->servo_q16 = servo_q16;

    // -----------------------------------------------------------------------
    // Sum rate + servo, clamp to nominal ± 1.0 sample/frame
    // -----------------------------------------------------------------------
    uint32_t rate_q16 = ctrl->long_valid ? ctrl->rate_long_q16 : ctrl->rate_estimate_q16;
    int32_t fb_out = (int32_t)rate_q16 + servo_q16;
    int32_t nom = (int32_t)ctrl->nominal_rate_q16;

    if (fb_out > nom + FB_OUTER_CLAMP_Q16)
        fb_out = nom + FB_OUTER_CLAMP_Q16;
    if (fb_out < nom - FB_OUTER_CLAMP_Q16)
        fb_out = nom - FB_OUTER_CLAMP_Q16;

    ctrl->feedback_out_q16 = (uint32_t)fb_out;
    if ((uint32_t)fb_out < ctrl->diag_fb_min_q16) ctrl->diag_fb_min_q16 = (uint32_t)fb_out;
    if ((uint32_t)fb_out > ctrl->diag_fb_max_q16) ctrl->diag_fb_max_q16 = (uint32_t)fb_out;
}

// ---------------------------------------------------------------------------
// Endpoint serialization: Q16.16 → 10.14
// ---------------------------------------------------------------------------

FB_TIME_CRITICAL
uint32_t fb_ctrl_get_10_14(const usb_feedback_ctrl_t *ctrl) {
    uint32_t q16 = ctrl->feedback_out_q16;
    if (q16 == 0)
        return 0;  // Caller should fall back to nominal
    // Rounded shift: (q16 + 2) >> 2
    return (q16 + 2) >> 2;
}

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------

void fb_ctrl_get_diag(usb_feedback_ctrl_t *ctrl, fb_ctrl_diag_t *out) {
    out->feedback_q16     = ctrl->feedback_out_q16;
    out->fb_min_q16       = ctrl->diag_fb_min_q16;
    out->fb_max_q16       = ctrl->diag_fb_max_q16;
    out->rate_iir_q16     = ctrl->rate_estimate_q16;
    out->rate_long_q16    = ctrl->long_valid ? ctrl->rate_long_q16 : 0;
    out->long_span_frames = ctrl->long_seeded
                            ? ctrl->frame_total - ctrl->long_frames[0] : 0;
    out->servo_q16        = ctrl->servo_q16;
    out->fill_error_q16   = ctrl->fill_error_filtered;
    out->updates          = ctrl->diag_updates;
    out->merged_windows   = ctrl->diag_merged_windows;
    out->rejected         = ctrl->diag_rejected;
    out->long_reanchors   = ctrl->diag_long_reanchors;
    ctrl->diag_fb_min_q16 = ctrl->diag_fb_max_q16 = ctrl->feedback_out_q16;
}
