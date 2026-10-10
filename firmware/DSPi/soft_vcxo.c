/*
 * soft_vcxo.c; fractional-N trim of the system PLL ("soft VCXO").
 *
 * Mechanism: a DMA channel pair pulses pll_sys FBDIV between its nominal N
 * and N +- 1 for one DMA-timer period (SOFT_VCXO_PULSE_CYCLES sys cycles)
 * per PWM wrap; the PLL loop filter smooths the pulse train into a clean
 * average frequency offset. Each pulse inserts PULSE_CYCLES / N sys cycles
 * of phase (1.17 at the 307.2 MHz plan, N = 128), so the offset is
 * proportional to the pulse rate: 1 ppm per sys_MHz * N / PULSE_CYCLES Hz
 * (262.144 Hz at 307.2 MHz). N, sys_clk and the tune ratio are read from
 * the live PLL at init, so any clock plan works.
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

#define SOFT_VCXO_MAX_PPM      300.0f
// Pulse width: one DMA-timer period, in sys cycles.
#define SOFT_VCXO_PULSE_CYCLES 150u
// Commands below this park instead; the servo integrator dithers across the
// floor, so finer static resolution buys nothing.
#define SOFT_VCXO_PARK_PPM     0.115f

// Clock plan read from pll_sys at init.
static uint32_t vcxo_fbdiv_nom;
static uint32_t vcxo_sys_hz;
static float    vcxo_hz_per_ppm;   // pulse rate per ppm of sys_clk

// {excursion, nominal} pair the PULSE channel streams into fbdiv_int.
// Must stay in RAM: the pulse train keeps running through flash blackout
// windows where XIP reads would stall it.
static uint32_t fbdivs[2];

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
static uint16_t diag_pwm_div = 255;
static uint8_t  diag_fbdiv0;
static bool     diag_parked = true;
static int8_t   diag_last_sign = 0;
#endif

void soft_vcxo_init(void) {
    // Stay inert unless clk_sys is pll_sys at its programmed rate (the
    // RP2350 low-clock boot fallback is not).
    uint32_t refdiv = pll_sys_hw->cs & PLL_CS_REFDIV_BITS;
    uint32_t fbdiv  = pll_sys_hw->fbdiv_int & PLL_FBDIV_INT_BITS;
    uint32_t pd1 = (pll_sys_hw->prim & PLL_PRIM_POSTDIV1_BITS) >> PLL_PRIM_POSTDIV1_LSB;
    uint32_t pd2 = (pll_sys_hw->prim & PLL_PRIM_POSTDIV2_BITS) >> PLL_PRIM_POSTDIV2_LSB;
    uint32_t sys = clock_get_hz(clk_sys);
    if (refdiv == 0 || pd1 == 0 || pd2 == 0 ||
        (uint64_t)XOSC_HZ / refdiv * fbdiv / (pd1 * pd2) != sys)
        return;
    vcxo_fbdiv_nom = fbdiv;
    vcxo_sys_hz = sys;
    vcxo_hz_per_ppm = (float)sys * 1e-6f * (float)fbdiv / (float)SOFT_VCXO_PULSE_CYCLES;
    fbdivs[0] = fbdivs[1] = fbdiv;
#if DSPI_CLOCK_DIAG
    diag_fbdiv0 = (uint8_t)fbdiv;
#endif

#if PICO_RP2350
    // Grant the DMA master write access to PLL_SYS (0xACCE = passwd).
    accessctrl_hw->pll_sys = 0xacce0000u | 0xffu;
#endif

    dma_channel_claim(SOFT_VCXO_DMA_PULSE);
    dma_channel_claim(SOFT_VCXO_DMA_PACE);
    dma_timer_claim(SOFT_VCXO_DMA_TIMER);
    dma_timer_set_fraction(SOFT_VCXO_DMA_TIMER, 1, SOFT_VCXO_PULSE_CYCLES);

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
    pwm_config_set_clkdiv_int(&pc, 255);
    pwm_config_set_wrap(&pc, 0xffff);
    pwm_init(SOFT_VCXO_PWM_SLICE, &pc, true);

    dma_channel_start(SOFT_VCXO_DMA_PACE);
    vcxo_ready = true;
}

DSP_TIME_CRITICAL
void soft_vcxo_set_ppm(float ppm) {
    if (!vcxo_ready) return;
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

    float mag = (ppm >= 0.0f) ? ppm : -ppm;
    if (mag < SOFT_VCXO_PARK_PPM) {
        fbdivs[0] = vcxo_fbdiv_nom;    // park: pulses become no-ops
#if DSPI_CLOCK_DIAG
        diag_parked = true;
        diag_fbdiv0 = (uint8_t)vcxo_fbdiv_nom;
        diag_pulse_hz = 0;
#endif
        return;
    }
    fbdivs[0] = (ppm > 0.0f) ? vcxo_fbdiv_nom + 1u : vcxo_fbdiv_nom - 1u;

    // Smallest integer clkdiv that fits the period in 16 bits gives the
    // finest wrap resolution. The park floor keeps the period within reach
    // of clkdiv 255 for any PLL feedback divider of 79 or more.
    float period = (float)vcxo_sys_hz / (mag * vcxo_hz_per_ppm);
    uint32_t div = (uint32_t)(period / 65536.0f) + 1u;
    if (div > 255u) div = 255u;
    uint32_t wrap = (uint32_t)(period / (float)div + 0.5f);
    if (wrap > 0x10000u) wrap = 0x10000u;
    if (wrap < 4u) wrap = 4u;
    pwm_set_clkdiv_int_frac4(SOFT_VCXO_PWM_SLICE, (uint8_t)div, 0);
    pwm_set_wrap(SOFT_VCXO_PWM_SLICE, (uint16_t)(wrap - 1u));

#if DSPI_CLOCK_DIAG
    diag_parked = false;
    diag_fbdiv0 = (uint8_t)fbdivs[0];
    diag_pwm_div = (uint16_t)div;
    diag_pwm_wrap = wrap;
    diag_pulse_hz = vcxo_sys_hz / (div * wrap);
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
