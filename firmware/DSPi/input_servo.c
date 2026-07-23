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
 *   Loop B: consumer-fill trim on the SAMPLE-GRANULAR fractional fill
 *           (get_slot_consumer_fill_frac), IIR-smoothed; a plain PI that
 *           converges the fill onto 8.0 of 16 buffers, with a second P
 *           slope beyond +-2 buffers for real displacements.
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

// Loop B gains, re-derived 2026-07-23 for the sample-granular fill signal
// (get_slot_consumer_fill_frac; see that function for what it measures).
//
// History: the divider-servo gains, expressed on the continuous VCXO,
// limit-cycled the integrator rail to rail (-15..+50 ppm, ~10 ppm/s
// slews) on zero true error because the fill was quantized to whole
// buffers; external DAC receiver PLLs unlocked on the slews. A first
// retune bounded that cycle (gated I, +-10 ppm clamp, slew limiter).
// The fractional fill removes the quantizer itself: after the IIR the
// error is continuous, so a plain PI converges to a fixed point and the
// integrator no longer needs an error gate.
//
// Plant: 1 ppm of rate trim slews the fill by Fs*1e-6 samples/s, i.e.
// ~0.001 buffer/s per ppm at 48 kHz. KP sets the correction bandwidth
// (2 ppm/buffer -> ~0.002 rad/s); KI holds the static residual and its
// zero sits below that crossover for damping (0.0025 ppm/s per buffer).
// The genuine ~1-buffer production/consumption ripple is smoothed by the
// IIR to ~+-0.2 buffer, so KP contributes well under 0.5 ppm of command
// noise, further shaped by the slew limiter. A second P slope engages
// beyond +-2 buffers so real displacements recover faster; transients
// bigger than that arrive via resets, which re-centre the fill anyway.
#define SERVO_FILL_ALPHA         0.125f  // fill IIR (~8 ticks, ~160 ms)
#define SERVO_FILL_KP_PPM        2.0f    // ppm per buffer of smoothed error
#define SERVO_FILL_KP2_PPM       4.0f    // added slope beyond +-2 buffers
#define SERVO_FILL_KI_PPM        5.0e-5f // ppm per tick per buffer
#define SERVO_FILL_ICLAMP_PPM    3.0f    // bounds the static residual held
#define SERVO_SLEW_PPM_PER_TICK  0.5f    // ~25 ppm/s at the ~20 ms tick

// Smoothed fill (buffer units), integrator and last slew-limited command.
static float fill_smooth = 8.0f;
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
        float fill = get_slot_consumer_fill_frac((uint)fill_slot);
        if (fill < 0.0f) {
            // No valid reading (type switch in progress); hold everything.
            soft_vcxo_set_ppm(last_cmd_ppm);
            return;
        }
        fill_smooth += SERVO_FILL_ALPHA * (fill - fill_smooth);
        float fill_error = fill_smooth - 8.0f;
#if DSPI_CLOCK_DIAG
        diag_fill_error = (int32_t)fill_error;
#endif

        // P on the smoothed continuous error; second slope for real
        // displacements beyond +-2 buffers.
        float p_ppm = fill_error * SERVO_FILL_KP_PPM;
        if (fill_error > 2.0f)
            p_ppm += (fill_error - 2.0f) * SERVO_FILL_KP2_PPM;
        else if (fill_error < -2.0f)
            p_ppm += (fill_error + 2.0f) * SERVO_FILL_KP2_PPM;

        // I: continuous error, no gate needed (see the gain comment).
#if !(DSPI_CLOCK_DIAG && SOFT_VCXO_DIAG_FREEZE_INTEGRATOR)
        fill_integral_ppm += fill_error * SERVO_FILL_KI_PPM;
        if (fill_integral_ppm >  SERVO_FILL_ICLAMP_PPM) fill_integral_ppm =  SERVO_FILL_ICLAMP_PPM;
        if (fill_integral_ppm < -SERVO_FILL_ICLAMP_PPM) fill_integral_ppm = -SERVO_FILL_ICLAMP_PPM;
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
    fill_smooth = 8.0f;   // seed at target so the IIR has no start transient
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
