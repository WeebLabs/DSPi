/*
 * soft_vcxo.c; fractional-N trim of the system PLL ("soft VCXO").
 *
 * Mechanism: a DMA channel pair pulses pll_sys FBDIV between 128 and
 * 128 +- 1 for one DMA-timer period (150 sys cycles, ~0.49 us) per PWM
 * wrap; the PLL loop filter smooths the pulse train into a clean average
 * frequency offset. Each pulse inserts f_ref * t_pulse = 5.86 VCO cycles
 * (1.17 sys cycles) of phase, so the offset is proportional to the pulse
 * rate: 1 ppm per 262.144 Hz of PWM. The control quantum is a single
 * ~3.8 ns phase nudge, versus ~156 ppm per PIO divider LSB at 48 kHz for
 * the old divider servo.
 *
 * Topology (zero IRQs, zero steady-state CPU):
 *   PACE  (1 transfer, DREQ = PWM wrap): writes the fbdivs[] address to
 *         PULSE's read-address trigger alias, starting it.
 *   PULSE (2 transfers, DREQ = DMA timer): streams {excursion, nominal}
 *         into pll_sys fbdiv_int, then CHAIN_TOs PACE to re-arm it
 *         (trans_count reloads on every trigger), closing the loop.
 *
 * The tune ratio is analytic (timer fraction, f_ref, postdiv only) and
 * sits inside the input servo's feedback loop, so no calibration pass is
 * needed; residual gain error is absorbed by the fill terms.
 *
 * RP2350 needs an ACCESSCTRL grant for DMA writes to PLL_SYS. RP2040 has
 * no ACCESSCTRL (APB is open to the DMA master) but fbdiv dithering there
 * is pending hardware verification.
 */

#include "soft_vcxo.h"
#include "config.h"

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pll.h"
#include "hardware/pwm.h"
#if PICO_RP2350
#include "hardware/structs/accessctrl.h"
// hardware/pll.h macro-aliases pll_sys to pll_sys_hw, which would mangle
// the accessctrl_hw->pll_sys register name below.
#undef pll_sys
#endif

#define SOFT_VCXO_FBDIV_NOM   128u
#define SOFT_VCXO_SYS_HZ      307200000u
#define SOFT_VCXO_MAX_PPM     300.0f

// Pulses per second for 1 ppm: extra sys cycles per pulse =
// f_ref(12e6) * t_pulse(150 / 307.2e6) / postdiv1(5) = 1.171875, so
// 1 ppm (307.2 Hz) needs 307.2 / 1.171875 = 262.144 pulses/s.
#define SOFT_VCXO_HZ_PER_PPM  262.144f

// Below ~30 Hz (~0.11 ppm) park instead; the servo integrator dithers
// across the floor, so finer static resolution buys nothing.
#define SOFT_VCXO_MIN_HZ      30.0f

// {excursion, nominal} pair the PULSE channel streams into fbdiv_int.
// Must stay in RAM (.data): the pulse train keeps running through flash
// blackout windows where XIP reads would stall it.
static uint32_t fbdivs[2] = {SOFT_VCXO_FBDIV_NOM, SOFT_VCXO_FBDIV_NOM};

// Constant source word for PACE (PULSE's read address).
static uint32_t fbdivs_addr = (uint32_t)(uintptr_t)fbdivs;

static bool  vcxo_ready = false;
static float vcxo_ppm = 0.0f;

#if DSPI_CLOCK_DIAG
// Diagnostic aggregates (see soft_vcxo_get_diag).  Written only from
// soft_vcxo_set_ppm (plain stores, no locking); read from the main loop.
static float    diag_min_ppm = 0.0f;
static float    diag_max_ppm = 0.0f;
static uint32_t diag_set_calls = 0;
static uint32_t diag_clamp_sats = 0;
static uint32_t diag_sign_changes = 0;
static uint32_t diag_pulse_hz = 0;
static uint32_t diag_pwm_wrap = 0x10000u;
static uint16_t diag_pwm_div = 128;
static uint8_t  diag_fbdiv0 = SOFT_VCXO_FBDIV_NOM;
static bool     diag_parked = true;
static int8_t   diag_last_sign = 0;
#endif

void soft_vcxo_init(void) {
    // The tune constants assume the 307.2 MHz (fbdiv 128, postdiv 5/1)
    // clock plan; stay inert on the RP2350 low-clock boot fallback.
    if (clock_get_hz(clk_sys) != SOFT_VCXO_SYS_HZ) return;

#if PICO_RP2350
    // Grant the DMA master write access to PLL_SYS (0xACCE = passwd).
    accessctrl_hw->pll_sys = 0xacce0000u | 0xffu;
#endif

    dma_channel_claim(SOFT_VCXO_DMA_PULSE);
    dma_channel_claim(SOFT_VCXO_DMA_PACE);
    dma_timer_claim(SOFT_VCXO_DMA_TIMER);
    // Pulse width = one timer period = 150 sys cycles.
    dma_timer_set_fraction(SOFT_VCXO_DMA_TIMER, 1, 150);

    dma_channel_config c = dma_channel_get_default_config(SOFT_VCXO_DMA_PULSE);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, dma_get_timer_dreq(SOFT_VCXO_DMA_TIMER));
    channel_config_set_high_priority(&c, true);
    channel_config_set_chain_to(&c, SOFT_VCXO_DMA_PACE);
    dma_channel_configure(SOFT_VCXO_DMA_PULSE, &c,
                          &pll_sys_hw->fbdiv_int, fbdivs, 2, false);

    c = dma_channel_get_default_config(SOFT_VCXO_DMA_PACE);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pwm_get_dreq(SOFT_VCXO_PWM_SLICE));
    channel_config_set_high_priority(&c, true);
    dma_channel_configure(SOFT_VCXO_DMA_PACE, &c,
                          &dma_hw->ch[SOFT_VCXO_DMA_PULSE].al3_read_addr_trig,
                          &fbdivs_addr, 1, false);

    // The slice drives no GPIO; only its wrap DREQ is used. Start slow;
    // parked pulses write the nominal fbdiv, which is a no-op.
    pwm_config pc = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&pc, 128);
    pwm_config_set_wrap(&pc, 0xffff);
    pwm_init(SOFT_VCXO_PWM_SLICE, &pc, true);

    dma_channel_start(SOFT_VCXO_DMA_PACE);
    vcxo_ready = true;
}

DSP_TIME_CRITICAL
void soft_vcxo_set_ppm(float ppm) {
    if (!vcxo_ready) return;
#if DSPI_CLOCK_DIAG && SOFT_VCXO_DIAG_FORCE_PARK
    // A/B: force every command to 0 so the servo still runs but the PLL never
    // moves.  Persistent cutouts with this on exonerate the VCXO (H1/H5).
    ppm = 0.0f;
#endif
    if (ppm >  SOFT_VCXO_MAX_PPM) { ppm =  SOFT_VCXO_MAX_PPM;
#if DSPI_CLOCK_DIAG
        diag_clamp_sats++;
#endif
    } else if (ppm < -SOFT_VCXO_MAX_PPM) { ppm = -SOFT_VCXO_MAX_PPM;
#if DSPI_CLOCK_DIAG
        diag_clamp_sats++;
#endif
    }
    vcxo_ppm = ppm;

#if DSPI_CLOCK_DIAG
    diag_set_calls++;
    if (ppm < diag_min_ppm) diag_min_ppm = ppm;
    if (ppm > diag_max_ppm) diag_max_ppm = ppm;
    int8_t sign = (ppm > 0.0f) ? 1 : (ppm < 0.0f ? -1 : 0);
    if (sign != 0 && diag_last_sign != 0 && sign != diag_last_sign)
        diag_sign_changes++;
    if (sign != 0) diag_last_sign = sign;
#endif

    float freq = (ppm >= 0.0f ? ppm : -ppm) * SOFT_VCXO_HZ_PER_PPM;
    if (freq < SOFT_VCXO_MIN_HZ) {
        fbdivs[0] = SOFT_VCXO_FBDIV_NOM;    // park: pulses become no-ops
#if DSPI_CLOCK_DIAG
        diag_parked = true;
        diag_fbdiv0 = SOFT_VCXO_FBDIV_NOM;
        diag_pulse_hz = 0;
#endif
        return;
    }
    fbdivs[0] = (ppm > 0.0f) ? SOFT_VCXO_FBDIV_NOM + 1u
                             : SOFT_VCXO_FBDIV_NOM - 1u;

    // Highest-resolution clkdiv/wrap pair for the requested pulse rate.
    // Worst case at MAX_PPM is ~78.6 kHz (wrap ~3908), duty ~4%.
    uint32_t div = 1;
    uint32_t wrap = (uint32_t)((float)SOFT_VCXO_SYS_HZ / freq);
    while (wrap > 0x10000u && div < 128u) { wrap >>= 1; div <<= 1; }
    if (wrap > 0x10000u) wrap = 0x10000u;
    if (wrap < 4u) wrap = 4u;
    pwm_set_clkdiv_int_frac4(SOFT_VCXO_PWM_SLICE, (uint8_t)div, 0);
    pwm_set_wrap(SOFT_VCXO_PWM_SLICE, (uint16_t)(wrap - 1u));

#if DSPI_CLOCK_DIAG
    diag_parked = false;
    diag_fbdiv0 = (uint8_t)fbdivs[0];
    diag_pwm_div = (uint16_t)div;
    diag_pwm_wrap = wrap;
    diag_pulse_hz = SOFT_VCXO_SYS_HZ / (div * wrap);
#endif
}

float soft_vcxo_current_ppm(void) {
    return vcxo_ppm;
}

#if DSPI_CLOCK_DIAG
void soft_vcxo_get_diag(SoftVcxoDiag *out) {
    out->last_ppm     = vcxo_ppm;
    out->min_ppm      = diag_min_ppm;
    out->max_ppm      = diag_max_ppm;
    out->set_calls    = diag_set_calls;
    out->clamp_sats   = diag_clamp_sats;
    out->sign_changes = diag_sign_changes;
    out->pulse_hz     = diag_pulse_hz;
    out->pwm_div      = diag_pwm_div;
    out->pwm_wrap     = diag_pwm_wrap;
    out->fbdiv0       = diag_fbdiv0;
    out->parked       = diag_parked;
    // Reset the windowed min/max to the current command so the next dump
    // reflects only the intervening interval.
    diag_min_ppm = vcxo_ppm;
    diag_max_ppm = vcxo_ppm;
}
#endif
