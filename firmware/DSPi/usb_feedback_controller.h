/*
 * USB Feedback Controller: Q16.16 rate + fill servo
 *
 * Pure module with no Pico SDK dependencies.
 * Loop A: long-window (8-16 s) count-based rate; a rounded IIR (α=1/16) bridges startup.
 * Loop B: dual-slope proportional fill servo on the direct consumer fill.
 * Design and tuning: Documentation/current_architecture.md "Asynchronous Feedback Endpoint".
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

// Fill servo: direct consumer buffer fill (0-16 buffers of 48 frames).
// Inner slope 1/256 sample/frame per buffer keeps packet and processing jitter out
// of the feedback; past the knee the slope rises to 1/16 so real displacements clear fast.
#define FB_FILL_TARGET             8       // 50% of 16 consumer buffers
#define FB_FILL_KP_Q16             256     // inner slope
#define FB_FILL_KP2_Q16            3840    // added slope beyond the knee (total 4096)
#define FB_FILL_KNEE_Q16           (1 << 16)   // ±1 buffer

// Servo clamp: ±0.5 sample/frame in Q16.16 (matches old ±8192 in 10.14)
#define FB_SERVO_CLAMP_Q16         32768

// Total feedback clamp: ±1.0 sample/frame in Q16.16
#define FB_OUTER_CLAMP_Q16         65536

// IIR shift (α = 1/16 at 4ms update → τ ≈ 64ms)
#define FB_IIR_SHIFT               4

// Holdoff: number of valid 4ms updates required before servo is armed
#define FB_HOLDOFF_UPDATES         2

// Update cadence in USB frames (bRefresh=2). rate_shift scales exactly this
// span; a longer (merged-SOF) window is normalized by its true frame count.
#define FB_UPDATE_FRAMES           4

// USB SOF frame numbers are 11 bits and wrap every 2048 ms.
#define FB_FRAME_MASK              0x7FFu

// Rate-sample plausibility: ±nominal >> 1 (50%). A genuine anchor up to ~1 ms late
// reaches 25%, so this leaves margin; rejecting one half of a pair breaks cancellation.
#define FB_RATE_SANE_SHIFT         1

// Long-window rate: dual anchors rotate every HALF frames, so the span is 8-16 s.
// 2 * HALF * 4 words must stay below 65536 (the remainder division's limit).
#define FB_LONG_HALF_FRAMES        8000
#define FB_LONG_MIN_FRAMES         1000    // trust the long rate after 1 s
#define FB_LONG_SANE_SHIFT         7       // long rate within ±nominal >> 7 (0.78%)

// A long-window anchor or sample is taken only from a window whose short rate is
// within this of the IIR (0.5 sample/frame): a late SOF ISR would bias 8-16 s.
#define FB_LONG_ANCHOR_TOL_Q16     (1 << 15)

// ---------------------------------------------------------------------------
// Controller state
// ---------------------------------------------------------------------------

typedef struct {
    // Rate estimator (Loop A)
    uint32_t rate_estimate_q16;     // IIR-filtered device clock rate (startup bridge)
    uint32_t nominal_rate_q16;      // Pre-computed nominal for current Fs
    uint32_t rate_long_q16;         // Long-window rate, valid when long_valid
    uint32_t frame_total;           // USB frames since reset, extended past the 11-bit wrap
    uint32_t long_frames[2];        // Long-window anchors: [0] older, [1] newer
    uint32_t long_words[2];
    bool     long_seeded;           // Anchors hold a clean sample
    bool     long_valid;            // rate_long_q16 is in use

    // Fill servo (Loop B), direct measurement
    int32_t  fill_error_filtered;   // IIR-filtered fill error (in buffer counts, Q16.16)
    int32_t  servo_q16;             // Last servo term (diagnostics)

    // Diagnostics: counters are cumulative since init, min/max since fb_ctrl_get_diag
    uint32_t diag_updates;
    uint32_t diag_merged_windows;   // windows longer than FB_UPDATE_FRAMES
    uint32_t diag_rejected;         // short-window samples outside the plausibility band
    uint32_t diag_long_reanchors;   // long window discarded (gap or implausible rate)
    uint32_t diag_fb_min_q16;
    uint32_t diag_fb_max_q16;

    // Output
    uint32_t feedback_out_q16;      // Final feedback value (Q16.16)

    // Gating / validity
    uint8_t  holdoff_remaining;     // Updates remaining before servo arms
    bool     rate_valid;            // Rate estimator has been seeded
    bool     stream_active;         // Audio stream is currently active
    bool     need_baseline;         // First post-reset cycle: capture last_total_words only

    // Measurement window anchors, latched together in the same SOF call
    uint32_t last_frame;            // USB frame number at the last anchor
    uint32_t last_total_words;      // DMA word total at the last anchor
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

// Called every SOF (1ms). Updates once per FB_UPDATE_FRAMES of USB frame
// number and divides by the true span, so a merged (missed) SOF cannot skew it.
// frame_count: the SOF's 11-bit USB frame number.
// current_total_words: sub-buffer-precise DMA word count from slot 0.
// rate_shift: 12 for SPDIF, 13 for I2S.
// consumer_fill_q16: slot 0 fill in Q16.16 buffer units (frames * 65536
// / 48); sample-granular, from the ring's coherent snapshot pair.
void fb_ctrl_sof_update(usb_feedback_ctrl_t *ctrl,
                        uint32_t frame_count,
                        uint32_t current_total_words,
                        uint32_t rate_shift,
                        uint32_t consumer_fill_q16);

// Get the current feedback value as 10.14 fixed-point for the USB endpoint.
// Returns 0 if the controller has never been reset (caller should use nominal).
uint32_t fb_ctrl_get_10_14(const usb_feedback_ctrl_t *ctrl);

typedef struct {
    uint32_t feedback_q16;          // controller output (the wire value is nominal in non-USB modes)
    uint32_t fb_min_q16;
    uint32_t fb_max_q16;
    uint32_t rate_iir_q16;
    uint32_t rate_long_q16;         // 0 while the long window is not valid
    uint32_t long_span_frames;      // 0 while unseeded
    int32_t  servo_q16;
    int32_t  fill_error_q16;        // filtered
    uint32_t updates;
    uint32_t merged_windows;
    uint32_t rejected;
    uint32_t long_reanchors;
} fb_ctrl_diag_t;

// Snapshot for diagnostics and restart the min/max window. Main-loop context;
// tearing against the SOF ISR is acceptable.
void fb_ctrl_get_diag(usb_feedback_ctrl_t *ctrl, fb_ctrl_diag_t *out);

#endif // USB_FEEDBACK_CONTROLLER_H
