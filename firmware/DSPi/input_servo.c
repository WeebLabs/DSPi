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
 *   Loop B: consumer-fill trim; a deadbanded (|error| > 2 buffers)
 *           proportional term for disturbances plus a slow clamped
 *           integrator that centres the fill on exactly 8 of 16 buffers.
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

#define SERVO_FILL_KP  0.0005f   // Fill-level proportional gain

// Slow fill-centering integrator. The proportional term is deadbanded
// (|error| > 2 buffers), so without an integral term the fill would park
// wherever the enable transient leaves it inside the deadband. The
// integrator accumulates the residual ppm (estimator bias, feed-forward
// gain error) and pulls the fill onto exactly 8 buffers.
//
// Gain: the trim-to-fill plant is a single integrator (Fs/48 buffers per
// second per unit trim), so pure integral action limit-cycles; KI keeps
// that cycle at ~1 buffer amplitude, the same magnitude as the normal
// production/consumption wobble.
//
// Clamp: +-50 ppm of authority. The feed-forward absorbs divider rounding
// exactly, so the integrator only ever holds small residuals; the clamp
// bounds windup from transient bogus fill reads (type switches report 0).
#define SERVO_FILL_KI     2.0e-7f  // per tick per buffer of fill error
#define SERVO_FILL_ICLAMP 5.0e-5f  // +-50 ppm

// Integrator state, as a dimensionless rate trim (1e-6 = 1 ppm).
static float fill_trim_integral = 0.0f;

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

        float fill_trim = 0.0f;
        if (fill_error > 2 || fill_error < -2)
            fill_trim = (float)fill_error / 16.0f * SERVO_FILL_KP;

        // Centering integrator: always active so the fill converges on 8
        // buffers instead of parking inside the deadband.
#if !(DSPI_CLOCK_DIAG && SOFT_VCXO_DIAG_FREEZE_INTEGRATOR)
        fill_trim_integral += (float)fill_error * SERVO_FILL_KI;
        if (fill_trim_integral >  SERVO_FILL_ICLAMP) fill_trim_integral =  SERVO_FILL_ICLAMP;
        if (fill_trim_integral < -SERVO_FILL_ICLAMP) fill_trim_integral = -SERVO_FILL_ICLAMP;
#endif

        ppm += (fill_trim + fill_trim_integral) * 1e6f;
    }

#if DSPI_CLOCK_DIAG
    diag_servo.actual_freq  = actual_freq;
    diag_servo.ppm_ff       = diag_ff;
    diag_servo.ppm_total    = ppm;
    diag_servo.integral_ppm = fill_trim_integral * 1e6f;
    diag_servo.fill_error   = diag_fill_error;
    diag_servo.fill_slot    = fill_slot;
    if (ppm < diag_servo.ppm_min) diag_servo.ppm_min = ppm;
    if (ppm > diag_servo.ppm_max) diag_servo.ppm_max = ppm;
    diag_servo.ticks++;
#endif

    soft_vcxo_set_ppm(ppm);
}

void input_servo_reset(void) {
    fill_trim_integral = 0.0f;
    soft_vcxo_set_ppm(0.0f);
}

#if DSPI_CLOCK_DIAG
void input_servo_get_diag(InputServoDiag *out) {
    *out = diag_servo;
    diag_servo.ppm_min = diag_servo.ppm_total;
    diag_servo.ppm_max = diag_servo.ppm_total;
}
#endif
