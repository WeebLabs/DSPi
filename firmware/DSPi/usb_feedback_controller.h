/*
 * USB Feedback Controller — Q16.16 rate + fill servo
 *
 * Pure module with no Pico SDK dependencies.
 * Loop A: rounded IIR rate estimator (α=1/16, Q16.16).
 * Loop B: proportional fill-level servo using direct consumer fill measurement.
 */

#ifndef USB_FEEDBACK_CONTROLLER_H
#define USB_FEEDBACK_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------------------------
// Q16.16 helpers
// ---------------------------------------------------------------------------

// Signed nearest-integer division by 2^n (half-away-from-zero).
// Uses int64_t internally to avoid -INT32_MIN overflow.
static inline int32_t round_div_pow2_s32(int32_t x, uint32_t n) {
    int64_t xi = x;
    int64_t bias = 1ll << (n - 1);
    return xi >= 0 ? (int32_t)((xi + bias) >> n)
                   : (int32_t)(-(((-xi) + bias) >> n));
}

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

// Fill servo: direct consumer buffer fill (0-16 buffers)
#define FB_FILL_TARGET             8       // 50% of 16 consumer buffers
#define FB_FILL_KP_Q16             4096    // old Kp=1024 in 10.14 → 1024<<2 in Q16.16

// Servo clamp: ±0.5 sample/frame in Q16.16 (matches old ±8192 in 10.14)
#define FB_SERVO_CLAMP_Q16         32768

// Total feedback clamp: ±1.0 sample/frame in Q16.16
#define FB_OUTER_CLAMP_Q16         65536

// IIR shift (α = 1/16 at 4ms update → τ ≈ 64ms)
#define FB_IIR_SHIFT               4

// Holdoff: number of valid 4ms updates required before servo is armed
#define FB_HOLDOFF_UPDATES         2

// ---------------------------------------------------------------------------
// Controller state
// ---------------------------------------------------------------------------

typedef struct {
    // Rate estimator (Loop A)
    uint32_t rate_estimate_q16;     // IIR-filtered device clock rate
    uint32_t nominal_rate_q16;      // Pre-computed nominal for current Fs

    // Fill servo (Loop B) — direct measurement
    int32_t  fill_error_filtered;   // IIR-filtered fill error (in buffer counts, Q16.16)

    // Output
    uint32_t feedback_out_q16;      // Final feedback value (Q16.16)

    // Gating / validity
    uint8_t  holdoff_remaining;     // Updates remaining before servo arms
    bool     rate_valid;            // Rate estimator has been seeded
    bool     stream_active;         // Audio stream is currently active
    bool     need_baseline;         // First post-reset cycle: capture last_total_words only

    // SOF decimation
    uint32_t sof_count;             // Counts SOF events for 4-SOF decimation
    uint32_t last_total_words;      // Previous DMA word total for delta
} usb_feedback_ctrl_t;

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

// Initialize controller to power-on defaults.
void fb_ctrl_init(usb_feedback_ctrl_t *ctrl);

// Reset controller state and reseed at nominal rate.
void fb_ctrl_reset(usb_feedback_ctrl_t *ctrl, uint32_t nominal_rate_q16);

// Mark stream as deactivated (alt setting 0).
void fb_ctrl_stream_stop(usb_feedback_ctrl_t *ctrl);

// Called every SOF (1ms). Performs 4-SOF decimated measurement and update.
// current_total_words: sub-buffer-precise DMA word count from slot 0.
// rate_shift: 12 for SPDIF, 13 for I2S.
// consumer_fill: current consumer buffer fill count for slot 0 (0-16).
void fb_ctrl_sof_update(usb_feedback_ctrl_t *ctrl,
                        uint32_t current_total_words,
                        uint32_t rate_shift,
                        uint8_t consumer_fill);

// Get the current feedback value as 10.14 fixed-point for the USB endpoint.
// Returns 0 if the controller has never been reset (caller should use nominal).
uint32_t fb_ctrl_get_10_14(const usb_feedback_ctrl_t *ctrl);

#endif // USB_FEEDBACK_CONTROLLER_H
