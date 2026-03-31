/*
 * USB Feedback Controller — Q16.16 rate + fill servo
 *
 * Pure module: no Pico SDK dependencies, no hardware access.
 * All hardware interaction happens in the caller (usb_sof_irq).
 */

#include "usb_feedback_controller.h"

// ---------------------------------------------------------------------------
// Init / Reset
// ---------------------------------------------------------------------------

void fb_ctrl_init(usb_feedback_ctrl_t *ctrl) {
    ctrl->rate_estimate_q16     = 0;
    ctrl->nominal_rate_q16      = 0;
    ctrl->fill_error_filtered   = 0;
    ctrl->feedback_out_q16      = 0;
    ctrl->holdoff_remaining     = 0;
    ctrl->rate_valid            = false;
    ctrl->stream_active         = false;
    ctrl->need_baseline         = false;
    ctrl->sof_count             = 0;
    ctrl->last_total_words      = 0;
}

void fb_ctrl_reset(usb_feedback_ctrl_t *ctrl, uint32_t nominal_rate_q16) {
    ctrl->nominal_rate_q16      = nominal_rate_q16;
    ctrl->rate_estimate_q16     = nominal_rate_q16;
    ctrl->rate_valid            = true;
    ctrl->fill_error_filtered   = 0;
    ctrl->holdoff_remaining     = FB_HOLDOFF_UPDATES;
    ctrl->feedback_out_q16      = nominal_rate_q16;
    ctrl->stream_active         = true;
    ctrl->need_baseline         = true;
    ctrl->sof_count             = 0;
}

void fb_ctrl_stream_stop(usb_feedback_ctrl_t *ctrl) {
    ctrl->stream_active         = false;
    ctrl->rate_valid            = false;
    ctrl->fill_error_filtered   = 0;
    ctrl->holdoff_remaining     = 0;
    ctrl->sof_count             = 0;
    ctrl->feedback_out_q16      = ctrl->nominal_rate_q16;
}

// ---------------------------------------------------------------------------
// SOF update (called every 1ms from USB IRQ)
// ---------------------------------------------------------------------------

void fb_ctrl_sof_update(usb_feedback_ctrl_t *ctrl,
                        uint32_t current_total_words,
                        uint32_t rate_shift,
                        uint8_t consumer_fill) {
    if (!ctrl->stream_active || !ctrl->rate_valid)
        return;

    ctrl->sof_count++;
    if ((ctrl->sof_count & 0x3) != 0)   // Every 4 SOFs (bRefresh=2)
        return;

    // First measurement cycle after reset: capture DMA baseline only.
    // last_total_words is stale — computing a delta would overflow the rate path.
    if (ctrl->need_baseline) {
        ctrl->last_total_words = current_total_words;
        ctrl->need_baseline = false;
        return;
    }

    // -----------------------------------------------------------------------
    // Loop A: Rate measurement (IIR, α = 1/16, rounded)
    // -----------------------------------------------------------------------
    uint32_t delta_words = current_total_words - ctrl->last_total_words;
    ctrl->last_total_words = current_total_words;

    if (delta_words == 0)
        return;  // DMA stalled — skip this cycle

    uint32_t rate_raw_q16 = delta_words << rate_shift;

    int32_t rate_error = (int32_t)(rate_raw_q16 - ctrl->rate_estimate_q16);
    ctrl->rate_estimate_q16 += (uint32_t)round_div_pow2_s32(rate_error, FB_IIR_SHIFT);

    // -----------------------------------------------------------------------
    // Loop B: Fill-level servo (proportional, direct measurement)
    //
    // Uses the consumer buffer fill count directly — immune to the counter
    // ratchet effect that breaks epoch-relative produced/consumed tracking
    // during producer pool backpressure stalls.
    // -----------------------------------------------------------------------
    int32_t servo_q16 = 0;

    if (ctrl->holdoff_remaining > 0) {
        ctrl->holdoff_remaining--;
    } else {
        // Fill error in Q16.16 buffer-counts for smooth IIR filtering.
        // Positive error = overfull, negative = underfull.
        int32_t fill_error_q16 = ((int32_t)consumer_fill - FB_FILL_TARGET) << 16;

        // IIR filter: same α=1/16 as rate path
        int32_t fe_delta = fill_error_q16 - ctrl->fill_error_filtered;
        ctrl->fill_error_filtered += round_div_pow2_s32(fe_delta, FB_IIR_SHIFT);

        // Proportional servo: overfull → negative correction → host sends less
        int32_t servo_raw = -((int64_t)FB_FILL_KP_Q16 * ctrl->fill_error_filtered >> 16);

        if (servo_raw > FB_SERVO_CLAMP_Q16)
            servo_raw = FB_SERVO_CLAMP_Q16;
        if (servo_raw < -FB_SERVO_CLAMP_Q16)
            servo_raw = -FB_SERVO_CLAMP_Q16;

        servo_q16 = servo_raw;
    }

    // -----------------------------------------------------------------------
    // Sum rate + servo, clamp to nominal ± 1.0 sample/frame
    // -----------------------------------------------------------------------
    int32_t fb_out = (int32_t)ctrl->rate_estimate_q16 + servo_q16;
    int32_t nom = (int32_t)ctrl->nominal_rate_q16;

    if (fb_out > nom + FB_OUTER_CLAMP_Q16)
        fb_out = nom + FB_OUTER_CLAMP_Q16;
    if (fb_out < nom - FB_OUTER_CLAMP_Q16)
        fb_out = nom - FB_OUTER_CLAMP_Q16;

    ctrl->feedback_out_q16 = (uint32_t)fb_out;
}

// ---------------------------------------------------------------------------
// Endpoint serialization: Q16.16 → 10.14
// ---------------------------------------------------------------------------

uint32_t fb_ctrl_get_10_14(const usb_feedback_ctrl_t *ctrl) {
    uint32_t q16 = ctrl->feedback_out_q16;
    if (q16 == 0)
        return 0;  // Caller should fall back to nominal
    // Rounded shift: (q16 + 2) >> 2
    return (q16 + 2) >> 2;
}
