/*
 * input_servo.c; shared output-clock servo actuation
 *
 * Extracted from the SPDIF input servo so ADAT input (slave clock mode) can
 * reuse it unchanged. Two control terms, mirroring the USB feedback servo:
 *   Loop A: the measured input rate sets the ideal output dividers directly.
 *   Loop B: slot 0 consumer fill provides a small proportional trim that
 *           dithers the divider rounding across LSB boundaries for sub-LSB
 *           rate matching (fill target 50% = 8 of 16 buffers, deadband 2).
 * The I2S divider is forced to exactly 2x the SPDIF divider so independent
 * rounding cannot make the two output types drift apart.
 *
 * The I2S slave servo is NOT built on this: edge-locked I2S output slots
 * consume at exactly the external rate, so it must trim from the first
 * SPDIF-type slot only and never writes I2S/MCK dividers (see i2s_input.c).
 *
 * FUTURE FIX (belt and braces, not yet implemented): slew-limit the applied
 * divider to +-1 LSB per call. With the long-window estimators upstream
 * (ADAT slave, SPDIF input) the rate term is already sub-LSB stable, so the
 * limit would rarely engage, but it would hard-bound output clock wander
 * (DAC PLL kindness) for every current and future caller regardless of
 * estimator quality. Needs care at (re)lock: the first apply after
 * input_servo_reset() must still jump straight to the target divider, not
 * slew from the nominal one. See "Clock Servo" in
 * Documentation/current_architecture.md.
 */

#include "input_servo.h"
#include "config.h"
#include "audio_pipeline.h"

#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "pico/audio_spdif.h"
#include "pico/audio_i2s_multi.h"
#include "adat_output.h"

#define SERVO_FILL_KP  0.0005f   // Fill-level proportional gain

// Last written dividers; skip PIO writes when unchanged
static uint32_t last_spdif_div = 0;
static uint32_t last_i2s_div = 0;
static uint32_t last_mck_div = 0;

// Apply a divider (16.8 fixed-point) to a PIO SM
static inline void set_divider(PIO pio, uint sm, uint32_t div_16_8) {
    pio_sm_set_clkdiv_int_frac(pio, sm, div_16_8 >> 8, div_16_8 & 0xFF);
}

DSP_TIME_CRITICAL
uint32_t input_servo_apply(float actual_freq) {
    if (actual_freq < 20000.0f || actual_freq > 200000.0f) return 0;

    uint32_t sys_clk = clock_get_hz(clk_sys);
    // No ceiling; precise float division lets the fill trim dither between
    // adjacent integer divider values to achieve sub-LSB rate matching.
    float spdif_div_f = (float)sys_clk / actual_freq;

    uint8_t consumer_fill = get_slot_consumer_fill(0);  // Slot 0 as reference
    int32_t fill_error = (int32_t)consumer_fill - 8;    // Target 50% of 16 buffers

    float fill_trim = 0.0f;
    if (fill_error > 2 || fill_error < -2) {
        // Positive error (overfull) → negative trim → reduce divider → speed up outputs
        fill_trim = -(float)fill_error / 16.0f * SERVO_FILL_KP;
    }

    uint32_t spdif_div = (uint32_t)(spdif_div_f * (1.0f + fill_trim) + 0.5f);
    uint32_t i2s_div   = spdif_div * 2;

    if (spdif_div == last_spdif_div && i2s_div == last_i2s_div)
        return spdif_div;
    last_spdif_div = spdif_div;
    last_i2s_div = i2s_div;

    extern struct audio_spdif_instance *spdif_instance_ptrs[];
    extern struct audio_i2s_instance *i2s_instance_ptrs[];
    extern uint8_t output_types[];

    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        if (output_types[i] == OUTPUT_TYPE_SPDIF && spdif_instance_ptrs[i]) {
            set_divider(spdif_instance_ptrs[i]->pio,
                        spdif_instance_ptrs[i]->pio_sm, spdif_div);
        } else if (output_types[i] == OUTPUT_TYPE_I2S && i2s_instance_ptrs[i]) {
            set_divider(i2s_instance_ptrs[i]->pio,
                        i2s_instance_ptrs[i]->pio_sm, i2s_div);
        }
    }

#if PICO_RP2350
    // ADAT bulk output runs the same 256*Fs PIO clock as the S/PDIF TX SMs;
    // apply the identical divider so it stays rate-locked to the slots.
    adat_output_servo_divider(spdif_div);
#endif

    // MCK servo: keep master clock frequency-locked to the servoed I2S data
    // rate. MCK is driven by CLK_GPOUTn, so the divider is the direct 24.8
    // form: div_24.8 = sys_clk * 256 / (actual_freq * multiplier).
    extern bool i2s_mck_enabled;
    extern uint16_t i2s_mck_multiplier;
    if (i2s_mck_enabled && i2s_mck_multiplier > 0) {
        float mck_div_f = (float)sys_clk * 256.0f / (actual_freq * (float)i2s_mck_multiplier);
        uint32_t mck_div = (uint32_t)(mck_div_f * (1.0f + fill_trim) + 0.5f);
        if (mck_div != last_mck_div) {
            last_mck_div = mck_div;
            audio_i2s_mck_set_divider(mck_div);
        }
    }

    return spdif_div;
}

void input_servo_reset(void) {
    last_spdif_div = 0;
    last_i2s_div = 0;
    last_mck_div = 0;
}

uint32_t input_servo_current_divider(void) {
    return last_spdif_div;
}
