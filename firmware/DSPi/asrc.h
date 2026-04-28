/*
 * asrc.h — Optional polyphase-FIR ASRC for the SPDIF input path.
 *
 * Built only when SPDIF_USE_ASRC is non-zero (see config.h). When the
 * macro is 0 the implementations below are empty stubs so non-ASRC
 * builds remain bit-for-bit identical to the legacy PIO-divider servo
 * path.
 *
 * Architecture (drift-tracking mode):
 *   SPDIF_RX_FIFO ─► asrc_process() ─► (preamp + format) ─► process_input_block()
 *                       ▲
 *                       │ ratio
 *                  asrc_servo_tick(consumer_fill, fs_in_meas)
 *
 * Pipeline Fs continues to follow the detected SPDIF rate via the existing
 * rate-change machinery (audio_state.freq, perform_rate_change()). This
 * module only compensates for the small ppm offset between the SPDIF
 * transmitter's crystal and the host's crystal, by trimming the resampling
 * ratio around 1.0 to keep the output consumer pool centered at 50% fill.
 *
 * Output PIO dividers are pinned at the nominal rate when ASRC is enabled,
 * so all four output instances stay phase-coherent — no slot drift is
 * introduced by clock recovery (the inviolable hard constraint).
 *
 * Quality (measured by the coefficient generator at design time):
 *   RP2350 (256 phases × 64-tap float, cubic-Hermite inter-phase):
 *     stop-band  ≥ 106 dB, pass-band ripple ≤ 0.04 dB,
 *     pass-band  to 0.85 × input_Nyquist (~20.4 kHz at Fs=48 kHz).
 *   RP2040 (256 phases × 32-tap Q31, linear inter-phase):
 *     stop-band  ≥ 104 dB, pass-band ripple ≤ 0.04 dB,
 *     pass-band  to 0.70 × input_Nyquist (~16.8 kHz at Fs=48 kHz).
 *
 * The narrower RP2040 pass-band is dictated by the M0+ compute budget;
 * see asrc.c and the SNR/CPU table in config.h.
 */
#ifndef ASRC_H
#define ASRC_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Caller binds the pipeline rate the ASRC will produce its output at.
 * Called once at boot and again whenever audio_state.freq changes
 * (rate-change handler). Resets all internal state to a clean post-reset
 * configuration. */
void asrc_init(uint32_t fs_pipe_nom);

/* Discard history, zero phase, exact 1.0 ratio, mute. Called at SPDIF
 * lock loss / lock acquire / rate change / input-source switch. After
 * reset, the next ASRC_TAPS input samples populate the history without
 * producing output (priming). */
void asrc_reset(void);

/* Run the polyphase resampler on a stereo input chunk. Caller passes raw
 * sign-extended 24-bit samples in int32 (left-justified to bit 31, low
 * byte zero — the same format the SPDIF FIFO loop uses today before the
 * preamp/format conversion). The function consumes input as needed and
 * writes up to `out_max` output samples in the same int32 left-justified
 * format. Output count is capped at `out_max`; any residual phase state
 * is preserved across calls. The pipeline's per-block cap is 192 frames,
 * which is always passed as out_max so process_input_block() never sees
 * a count larger than its working buffer.
 *
 * `in_consumed_out`, when non-NULL, receives the number of input samples
 * actually consumed. This can be less than `n_in` when R < 1.0 or when
 * `out_max` is reached, so callers that read from a destructive FIFO must
 * carry the remaining input into the next call.
 *
 * Returns the number of output samples produced. Returns 0 if not yet
 * primed or muted (indicating the caller must NOT push these samples
 * through the pipeline this tick).
 */
unsigned asrc_process(const int32_t *in_l,
                      const int32_t *in_r,
                      unsigned       n_in,
                      int32_t       *out_l,
                      int32_t       *out_r,
                      unsigned       out_max,
                      unsigned      *in_consumed_out);

/* Servo update: feed forward the SPDIF rate measurement plus a proportional
 * trim from the consumer-pool fill error. Called from spdif_input.c on the
 * existing 1000-iteration servo cadence (~20 ms). LPF on the committed
 * ratio gives τ ≈ 1.3 s, well below pitch-modulation audibility. */
void asrc_servo_tick(uint8_t consumer_fill, float fs_in_meas);

/* ---------------- Telemetry / introspection ---------------- */

/* (R − 1.0) × 1e6, signed. Reflects the latest committed (LPF'd) ratio. */
int32_t asrc_get_ratio_ppm(void);

/* Current state flags for the host UI / bench tests. Bit 0: primed,
 * bit 1: muted, bits 2-7: reserved. */
uint8_t asrc_get_lock_flags(void);

/* Cumulative count of times the ASRC ran out of input mid-call (input
 * FIFO drained faster than expected) since asrc_init. Diagnostic only. */
uint32_t asrc_get_underrun_count(void);

#ifdef __cplusplus
}
#endif

#endif /* ASRC_H */
