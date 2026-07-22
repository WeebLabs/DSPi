/*
 * input_servo.c; shared clock servo for externally clocked inputs
 *
 * Actuates the soft VCXO (system PLL trim, soft_vcxo.c) instead of output
 * PIO dividers: every output clock (SPDIF/I2S/ADAT TX, MCK, PDM) derives
 * from sys_clk, so one ppm command tracks them all and inter-slot
 * alignment is preserved by construction. The old divider servo could not
 * touch PDM at all and had a ~156 ppm control quantum (one 16.8 divider
 * LSB at 48 kHz); the VCXO quantum is a single ~3.8 ns phase nudge.
 *
 * Two control terms, mirroring the USB feedback servo:
 *   Loop A: ppm feed-forward from the measured input rate, computed
 *           against the programmed nominal divider (ceil, identical to
 *           the TX library) so divider rounding is cancelled here rather
 *           than left to the integrator.
 *   Loop B: consumer-fill trim; a proportional term (continuous beyond a
 *           2-buffer deadband) for disturbances plus a slow clamped
 *           integrator, gated on nonzero fill error, that holds the fill
 *           at 8 of 16 buffers to within the +-1 buffer quantization.
 * The total command is slew-limited per tick before actuation (see the
 * Loop B gain comment for the hardware findings behind the tuning).
 *
 * The I2S slave servo shares this module too (it passes the first
 * SPDIF-type slot as fill reference): with no divider writes left, the
 * old reason for its separate implementation is gone.
 */

#include "input_servo.h"
#include "soft_vcxo.h"
#include "config.h"
#include "audio_pipeline.h"
#include "usb_audio.h"

#include "hardware/clocks.h"

// Loop B gains, retuned 2026-07-22 after the first hardware capture.
//
// The original gains were carried over from the divider servo, where the
// 156 ppm-per-LSB actuator quantization hid almost all integrator motion
// (sub-LSB wander never reached the outputs). On the continuous VCXO the
// same gains expressed directly: the bench log showed the integrator
// limit-cycling rail to rail (-15..+50 ppm, ~100 s period, ~10 ppm/s
// slews) on ZERO true error, driven purely by the +-1 buffer fill
// quantization, and the external DAC receiver PLL unlocked on the slews.
//
// Structure note: fill responds to a rate trim through an integration
// (ppm -> fill slew), so integral action on the quantized fill can never
// fully settle; a micro limit cycle is inherent. The tuning therefore
// aims to make that cycle harmless rather than pretend to remove it:
//   - the integrator only accumulates on a NONZERO fill error, so it
//     cannot pump while the fill sits centred;
//   - its clamp bounds the cycle amplitude to +-10 ppm (also enough
//     margin for a few percent of VCXO tune-ratio error on a worst-case
//     +-100 ppm source);
//   - a global slew limiter caps the commanded ppm change per tick, so
//     the carrier never steps faster than a receiver PLL tracks.
#define SERVO_FILL_DEADBAND      2      // buffers; P engages beyond this
#define SERVO_FILL_KP_PPM        1.5f   // ppm per buffer beyond the deadband
#define SERVO_FILL_KI_PPM        0.005f // ppm per tick per buffer (ferr != 0)
#define SERVO_FILL_ICLAMP_PPM    10.0f
#define SERVO_SLEW_PPM_PER_TICK  0.5f   // ~25 ppm/s at the ~20 ms tick

// Integrator state and last slew-limited command, both in ppm.
static float fill_integral_ppm = 0.0f;
static float last_cmd_ppm = 0.0f;

#if DSPI_CLOCK_DIAG
// Diagnostic snapshot (see input_servo_get_diag).  Plain stores from the
// servo tick; windowed min/max total ppm reset on read.
static InputServoDiag diag_servo = { .fill_slot = -1 };
#endif

DSP_TIME_CRITICAL
void input_servo_apply(float actual_freq, int fill_slot) {
    if (actual_freq < 20000.0f || actual_freq > 200000.0f) return;

    uint32_t freq_nom = audio_state.freq;
    if (freq_nom == 0) return;
    uint32_t sys_clk = clock_get_hz(clk_sys);   // nominal by convention

    // Programmed SPDIF-type divider: ceil(sys/freq), bit-identical to
    // update_pio_frequency() in the TX library. Outputs actually run at
    // sys/div, so referencing div (not freq_nom) folds the divider
    // rounding residual into the feed-forward.
    uint32_t div = sys_clk / freq_nom + (sys_clk % freq_nom != 0);
    float ppm = ((actual_freq * (float)div) / (float)sys_clk - 1.0f) * 1e6f;

    // Feed-forward far outside VCXO authority means the caller applied
    // before the pipeline followed the detected rate; do not integrate on
    // that transient.
    if (ppm > 2000.0f || ppm < -2000.0f) return;

#if DSPI_CLOCK_DIAG
    float diag_ff = ppm;
    int32_t diag_fill_error = 0;
#endif

    if (fill_slot >= 0) {
        // Positive error (overfull) means consumers lag; speed sys_clk up.
        int32_t fill_error = (int32_t)get_slot_consumer_fill((uint)fill_slot) - 8;
#if DSPI_CLOCK_DIAG
        diag_fill_error = fill_error;
#endif

        // P: continuous beyond the deadband (no jump at the edge).
        float p_ppm = 0.0f;
        if (fill_error > SERVO_FILL_DEADBAND)
            p_ppm = (float)(fill_error - SERVO_FILL_DEADBAND) * SERVO_FILL_KP_PPM;
        else if (fill_error < -SERVO_FILL_DEADBAND)
            p_ppm = (float)(fill_error + SERVO_FILL_DEADBAND) * SERVO_FILL_KP_PPM;

        // I: only on a nonzero error, so a centred fill cannot pump it.
#if !(DSPI_CLOCK_DIAG && SOFT_VCXO_DIAG_FREEZE_INTEGRATOR)
        if (fill_error != 0) {
            fill_integral_ppm += (float)fill_error * SERVO_FILL_KI_PPM;
            if (fill_integral_ppm >  SERVO_FILL_ICLAMP_PPM) fill_integral_ppm =  SERVO_FILL_ICLAMP_PPM;
            if (fill_integral_ppm < -SERVO_FILL_ICLAMP_PPM) fill_integral_ppm = -SERVO_FILL_ICLAMP_PPM;
        }
#endif

        ppm += p_ppm + fill_integral_ppm;
    }

    // Slew limiter: bound the commanded step per tick so the output
    // carriers never move faster than downstream receiver PLLs track.
    if (ppm > last_cmd_ppm + SERVO_SLEW_PPM_PER_TICK)
        ppm = last_cmd_ppm + SERVO_SLEW_PPM_PER_TICK;
    else if (ppm < last_cmd_ppm - SERVO_SLEW_PPM_PER_TICK)
        ppm = last_cmd_ppm - SERVO_SLEW_PPM_PER_TICK;
    last_cmd_ppm = ppm;

#if DSPI_CLOCK_DIAG
    diag_servo.actual_freq  = actual_freq;
    diag_servo.ppm_ff       = diag_ff;
    diag_servo.ppm_total    = ppm;
    diag_servo.integral_ppm = fill_integral_ppm;
    diag_servo.fill_error   = diag_fill_error;
    diag_servo.fill_slot    = fill_slot;
    if (ppm < diag_servo.ppm_min) diag_servo.ppm_min = ppm;
    if (ppm > diag_servo.ppm_max) diag_servo.ppm_max = ppm;
    diag_servo.ticks++;
#endif

    soft_vcxo_set_ppm(ppm);
}

void input_servo_reset(void) {
    fill_integral_ppm = 0.0f;
    last_cmd_ppm = 0.0f;
    soft_vcxo_set_ppm(0.0f);
}

#if DSPI_CLOCK_DIAG
void input_servo_get_diag(InputServoDiag *out) {
    *out = diag_servo;
    diag_servo.ppm_min = diag_servo.ppm_total;
    diag_servo.ppm_max = diag_servo.ppm_total;
}
#endif
