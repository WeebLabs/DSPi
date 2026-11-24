// soft_vcxo_shaped.c: crystal-paced, noise-shaped soft VCXO (RP2350, DSPI_VCXO_SHAPED), same
// API as soft_vcxo.c. A PIO pacer timed from XOSC on GPIO 25 writes one FBDIV byte per 5 XOSC
// periods from a RAM ring a sigma-delta keeps filled. Mechanism, hazards and constants:
// Documentation/Features/soft_vcxo_spec.md section 8.

#include "soft_vcxo.h"
#include "config.h"

#if SOFT_VCXO_SHAPED

#include <string.h>
#include <arm_acle.h>

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/pll.h"
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "hardware/structs/accessctrl.h"
#include "hardware/structs/m33.h"
// hardware/pll.h macro-aliases pll_sys, which would mangle accessctrl_hw->pll_sys.
#undef pll_sys

#include "soft_vcxo_pacer.pio.h"

#define VX_NOM            128u
#define VX_SYS_HZ         307200000u
#define VX_XOSC_HZ        12000000u

// One decision per 6 slots: full scale (a pulse every decision) is 1e6 / (128 * 6)
// = 1302 ppm. The clamp sits at 0.77 of it, where the modulator stays bounded.
#define VX_SLOTS_PER_DEC  6
#define VX_ONE            (1 << 24)                 // one pulse per decision, Q24
#define VX_HALF           (1 << 23)
#define VX_PPM_TO_Q24     (1e-6f * 128.0f * (float)VX_SLOTS_PER_DEC * (float)VX_ONE)
#define VX_MAX_PPM        1000.0f
// Below this the pacer parks; in practice only the servo's explicit 0 lands here.
#define VX_PARK_PPM       0.001f

// 8 KB ring = 3.41 ms of slots at 2.4 MB/s, several times the worst normal gap
// between producer deliveries. Must stay power-of-two sized and aligned (DMA wrap).
#define VX_RING_BITS      13
#define VX_RING_BYTES     (1u << VX_RING_BITS)
#define VX_RING_MASK      (VX_RING_BYTES - 1u)
#define VX_RING_WMASK     (VX_RING_BYTES / 4u - 1u)
#define VX_STEP_BYTES     12u      // two decisions as three aligned words
#define VX_GUARD_BYTES    64u      // never refill right up to the DMA read point
#define VX_FILL_CAP_BYTES 2400u    // one call refills at most 1 ms of slots
#define VX_FILL_PERIOD_US 200u     // refill in batches (480 B) rather than on every delivery
// Lead after a start or a replay: 100 us of slack against preemption mid-fill. At a
// start these are 128s, so a ragged first slot after GPOUT3 starts only rewrites 128.
#define VX_START_PAD      240u
#define VX_RESYNC_BYTES   240u
// Stop drain: well past the ~10 bytes in flight (feed, TX FIFO, OSR, RX FIFO, writer).
#define VX_DRAIN_US       50u
// Clean window 44..57 measured for this byte-fed path (45..58 for the 32-bit bench variant).
#define VX_DEFAULT_D      50u

#define VX_PIO            SOFT_VCXO_PACER_PIO
#define VX_SM             SOFT_VCXO_PACER_SM

_Static_assert(VX_RING_BITS <= 15, "DMA ring wrap is at most 32 KB");
_Static_assert(VX_FILL_CAP_BYTES % VX_STEP_BYTES == 0, "fill works in whole steps");
_Static_assert(VX_FILL_PERIOD_US * 12u / 5u < VX_FILL_CAP_BYTES, "a due refill must fit one call");
_Static_assert(VX_START_PAD % 4 == 0 && VX_RESYNC_BYTES % 4 == 0, "steps start word-aligned");

// The DMA reads this with XIP down (flash blackouts), so it must stay in RAM.
// Every byte must be 127, 128 or 129 at all times: the feed can fetch any of them.
static uint32_t vcxo_ring[VX_RING_BYTES / 4]
    __attribute__((aligned(VX_RING_BYTES), section(".bss.aligned.vcxo_ring")));

static uint8_t  vx_state = SOFT_VCXO_ST_INERT;
static uint     vx_offset;
static uint8_t  vx_d = VX_DEFAULT_D;

// Servo command (soft_vcxo_set_ppm), consumed by soft_vcxo_service.
static float    servo_ppm = 0.0f;
static int32_t  servo_u = 0;
static bool     servo_run = false;

// Modulator and ring writer state.
static int32_t  sd_i1, sd_i2;
static uint32_t fill_w;        // next ring byte to write (word-aligned)
static uint32_t fill_lead;     // unread bytes ahead of the DMA after the last fill
static uint32_t fill_t;        // time_us_32() of the last fill
static uint32_t drain_t;
static int32_t  run_u;

#if DSPI_CLOCK_DIAG
static bool     force_on = false;
static float    force_ppm = 0.0f;
static int32_t  force_u = 0;
static bool     force_run = false;
static bool     restart_req = false;

static float    diag_min_ppm, diag_max_ppm;
static uint32_t diag_set_calls, diag_clamp_sats, diag_sign_changes;
static int8_t   diag_last_sign;
static uint32_t diag_starts, diag_stops, diag_fill_calls, diag_sat_hits;
static uint64_t diag_decisions, diag_fill_cycles;
static int64_t  diag_net;
static uint32_t diag_lead_min = VX_RING_BYTES, diag_gap_max;
static int32_t  diag_i1_peak, diag_i2_peak;
static uint32_t diag_overtaken;
#endif
static uint32_t diag_replays;

static inline float vx_clamp(float ppm) {
    if (ppm > VX_MAX_PPM) return VX_MAX_PPM;
    if (ppm < -VX_MAX_PPM) return -VX_MAX_PPM;
    return ppm;
}

void soft_vcxo_init(void) {
    // Only the 12 MHz / N=128 / 307.2 MHz plan has a measured clean pacer window.
    uint32_t refdiv = pll_sys_hw->cs & PLL_CS_REFDIV_BITS;
    uint32_t fbdiv  = pll_sys_hw->fbdiv_int & PLL_FBDIV_INT_BITS;
    uint32_t pd1 = (pll_sys_hw->prim & PLL_PRIM_POSTDIV1_BITS) >> PLL_PRIM_POSTDIV1_LSB;
    uint32_t pd2 = (pll_sys_hw->prim & PLL_PRIM_POSTDIV2_BITS) >> PLL_PRIM_POSTDIV2_LSB;
    if (XOSC_HZ != VX_XOSC_HZ || refdiv != 1 || fbdiv != VX_NOM || pd1 * pd2 != 5 ||
        clock_get_hz(clk_sys) != VX_SYS_HZ)
        return;
    if (!pio_can_add_program(VX_PIO, &vcxo_pacer_program))
        return;

    accessctrl_hw->pll_sys = 0xacce0000u | 0xffu;   // admit the DMA master to PLL_SYS

    dma_channel_claim(SOFT_VCXO_DMA_WRITER);
    dma_channel_claim(SOFT_VCXO_DMA_FEED);
    pio_sm_claim(VX_PIO, VX_SM);
    vx_offset = pio_add_program(VX_PIO, &vcxo_pacer_program);
    memset(vcxo_ring, VX_NOM, VX_RING_BYTES);

    // XOSC on GPOUT3. The clean delay window was measured at 12 mA, fast slew.
    clock_gpio_init(SOFT_VCXO_REF_PIN, CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, 1);
    gpio_set_drive_strength(SOFT_VCXO_REF_PIN, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_slew_rate(SOFT_VCXO_REF_PIN, GPIO_SLEW_RATE_FAST);
    gpio_set_input_enabled(SOFT_VCXO_REF_PIN, true);
    hw_clear_bits(&clocks_hw->clk[clk_gpout3].ctrl, CLOCKS_CLK_GPOUT3_CTRL_ENABLE_BITS);

#if DSPI_CLOCK_DIAG
    m33_hw->demcr |= M33_DEMCR_TRCENA_BITS;
    m33_hw->dwt_ctrl |= M33_DWT_CTRL_CYCCNTENA_BITS;
#endif
    vx_state = SOFT_VCXO_ST_PARKED;
}

// Second-order three-level sigma-delta, one decision per call. Clamped
// integrators bound it at any input; inside +-1000 ppm they never engage.
static __force_inline int32_t sd_step(int32_t *i1, int32_t *i2, int32_t u) {
    // Same as (i2 > HALF) - (i2 < -HALF): one unsigned compare tests both rails.
    int32_t m = -(int32_t)((uint32_t)(*i2 + VX_HALF) > (uint32_t)VX_ONE);
    int32_t y = ((*i2 >> 31) | 1) & m;
    int32_t yq = y * VX_ONE;
    *i1 = __ssat(*i1 + u - yq, 28);
    *i2 = __ssat(*i2 + *i1 - yq, 29);
    return y;
}

// Top the ring up to just behind the DMA read point. A gap longer than the
// last lead means the DMA replayed stale bytes; the writer then restarts just ahead of it.
DSP_TIME_CRITICAL __attribute__((noinline))
static void vx_fill(int32_t u) {
#if DSPI_CLOCK_DIAG
    uint32_t c0 = m33_hw->dwt_cyccnt;
#endif
    uint32_t now = time_us_32();
    uint32_t rpos = dma_hw->ch[SOFT_VCXO_DMA_FEED].read_addr & VX_RING_MASK;
    uint32_t gap = now - fill_t;
    uint32_t lead = (fill_w - rpos) & VX_RING_MASK;
    uint32_t steps = 0;
    int32_t net = 0;
#if DSPI_CLOCK_DIAG
    if (gap > diag_gap_max && gap < 1000000u) diag_gap_max = gap;
#endif
    if (gap >= 1000000u || gap * 12u >= fill_lead * 5u || lead > fill_lead) {
        diag_replays++;
        fill_w = (rpos + VX_RESYNC_BYTES) & VX_RING_MASK & ~3u;
        lead = (fill_w - rpos) & VX_RING_MASK;
    }
#if DSPI_CLOCK_DIAG
    if (lead < diag_lead_min) diag_lead_min = lead;
#endif
    if (lead < VX_RING_BYTES - VX_GUARD_BYTES) {
        uint32_t room = VX_RING_BYTES - VX_GUARD_BYTES - lead;
        if (room > VX_FILL_CAP_BYTES) room = VX_FILL_CAP_BYTES;
        steps = room / VX_STEP_BYTES;

        int32_t i1 = sd_i1, i2 = sd_i2;
        uint32_t wi = fill_w >> 2;
        for (uint32_t k = 0; k < steps; k++) {
            int32_t y0 = sd_step(&i1, &i2, u);
            int32_t y1 = sd_step(&i1, &i2, u);
            vcxo_ring[wi & VX_RING_WMASK]        = 0x80808000u | (uint32_t)(int32_t)(VX_NOM + y0);
            vcxo_ring[(wi + 1u) & VX_RING_WMASK] = 0x80008080u | ((uint32_t)(int32_t)(VX_NOM + y1) << 16);
            vcxo_ring[(wi + 2u) & VX_RING_WMASK] = 0x80808080u;
            wi += 3u;
            net += y0 + y1;
        }
        __compiler_memory_barrier();
        // Detection only: the DMA passing the first byte this pass wrote means part
        // of it landed behind the DMA; the next call's replay logic resyncs.
        uint32_t moved = (dma_hw->ch[SOFT_VCXO_DMA_FEED].read_addr - rpos) & VX_RING_MASK;
        if (moved > lead) {
#if DSPI_CLOCK_DIAG
            diag_overtaken++;
#else
            diag_replays++;
#endif
        }
        sd_i1 = i1;
        sd_i2 = i2;
        fill_w = (wi << 2) & VX_RING_MASK;
        fill_lead = lead + steps * VX_STEP_BYTES;
        fill_t = now;
    }
    run_u = u;
#if DSPI_CLOCK_DIAG
    diag_fill_calls++;
    diag_decisions += 2u * steps;
    diag_net += net;
    int32_t a1 = sd_i1 < 0 ? -sd_i1 : sd_i1, a2 = sd_i2 < 0 ? -sd_i2 : sd_i2;
    if (a1 > diag_i1_peak) diag_i1_peak = a1;
    if (a2 > diag_i2_peak) diag_i2_peak = a2;
    if (steps && (a1 >= (1 << 27) - 1 || a2 >= (1 << 28) - 1)) diag_sat_hits++;
    diag_fill_cycles += m33_hw->dwt_cyccnt - c0;
#else
    (void)net;
#endif
}

// PARKED -> RUNNING. The SM starts only once the feed has filled its TX FIFO,
// so no slot can stall on an empty FIFO (an off-phase excursion resets the chip).
static void __attribute__((noinline)) vx_start(int32_t u) {
    PIO pio = VX_PIO;
    memset(vcxo_ring, VX_NOM, VX_RING_BYTES);
    __dmb();
    sd_i1 = sd_i2 = 0;
    fill_w = VX_START_PAD;
    fill_lead = VX_START_PAD;

    pio_sm_config c = vcxo_pacer_program_get_default_config(vx_offset);
    sm_config_set_in_pins(&c, SOFT_VCXO_REF_PIN);
    sm_config_set_in_shift(&c, false, false, 32);
    sm_config_set_out_shift(&c, true, false, 8);
    sm_config_set_clkdiv_int_frac(&c, 1, 0);
    pio_sm_init(pio, VX_SM, vx_offset, &c);
    // X = d through the FIFO with autopull off, then empty the OSR: a leftover
    // d in it would be the first value written to FBDIV.
    pio_sm_put(pio, VX_SM, vx_d);
    pio_sm_exec(pio, VX_SM, pio_encode_pull(false, true));
    pio_sm_exec(pio, VX_SM, pio_encode_mov(pio_x, pio_osr));
    pio_sm_exec(pio, VX_SM, pio_encode_out(pio_null, 32));
    sm_config_set_out_shift(&c, true, true, 8);
    pio->sm[VX_SM].shiftctrl = c.shiftctrl;
    pio_sm_clear_fifos(pio, VX_SM);

    dma_channel_config cw = dma_channel_get_default_config(SOFT_VCXO_DMA_WRITER);
    channel_config_set_transfer_data_size(&cw, DMA_SIZE_32);
    channel_config_set_read_increment(&cw, false);
    channel_config_set_write_increment(&cw, false);
    channel_config_set_dreq(&cw, pio_get_dreq(pio, VX_SM, false));
    channel_config_set_high_priority(&cw, true);
    dma_channel_configure(SOFT_VCXO_DMA_WRITER, &cw, &pll_sys_hw->fbdiv_int, &pio->rxf[VX_SM],
                          dma_encode_endless_transfer_count(), true);

    // ENDLESS with read wrap: a late CPU replays old bytes, it never starves the SM.
    dma_channel_config cf = dma_channel_get_default_config(SOFT_VCXO_DMA_FEED);
    channel_config_set_transfer_data_size(&cf, DMA_SIZE_8);
    channel_config_set_read_increment(&cf, true);
    channel_config_set_write_increment(&cf, false);
    channel_config_set_ring(&cf, false, VX_RING_BITS);
    channel_config_set_dreq(&cf, pio_get_dreq(pio, VX_SM, true));
    channel_config_set_high_priority(&cf, true);
    dma_channel_configure(SOFT_VCXO_DMA_FEED, &cf, &pio->txf[VX_SM], vcxo_ring,
                          dma_encode_endless_transfer_count(), true);

    uint32_t t0 = time_us_32();
    while (!pio_sm_is_tx_fifo_full(pio, VX_SM)) {
        if (time_us_32() - t0 > 100u) {
            // Feed never arrived and nothing has reached FBDIV. Go inert rather
            // than retry this busy-wait on every delivery.
            hw_clear_bits(&dma_hw->ch[SOFT_VCXO_DMA_FEED].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
            hw_clear_bits(&dma_hw->ch[SOFT_VCXO_DMA_WRITER].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
            dma_channel_abort(SOFT_VCXO_DMA_FEED);
            dma_channel_abort(SOFT_VCXO_DMA_WRITER);
            pll_sys_hw->fbdiv_int = VX_NOM;
            pio_sm_clear_fifos(pio, VX_SM);
            servo_ppm = 0.0f;
            vx_state = SOFT_VCXO_ST_INERT;
            return;
        }
    }

    fill_t = time_us_32();
    vx_fill(u);
    hw_set_bits(&clocks_hw->clk[clk_gpout3].ctrl, CLOCKS_CLK_GPOUT3_CTRL_ENABLE_BITS);
    pio_sm_set_enabled(pio, VX_SM, true);
    vx_state = SOFT_VCXO_ST_RUNNING;
#if DSPI_CLOCK_DIAG
    diag_lead_min = VX_RING_BYTES;   // the short start lead is not a late refill
    diag_starts++;
#endif
}

// RUNNING -> DRAINING: from here the feed can only fetch 128s.
static void __attribute__((noinline)) vx_begin_stop(void) {
    memset(vcxo_ring, VX_NOM, VX_RING_BYTES);
    __dmb();
    drain_t = time_us_32();
    vx_state = SOFT_VCXO_ST_DRAINING;
}

// DRAINING -> PARKED once only 128s can still be in flight. The two aborts are
// not simultaneous; that is safe only because both channels carry 128s by now.
static void __attribute__((noinline)) vx_finish_stop(void) {
    PIO pio = VX_PIO;
    pio_sm_set_enabled(pio, VX_SM, false);
    hw_clear_bits(&dma_hw->ch[SOFT_VCXO_DMA_FEED].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
    hw_clear_bits(&dma_hw->ch[SOFT_VCXO_DMA_WRITER].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
    dma_channel_abort(SOFT_VCXO_DMA_FEED);
    dma_channel_abort(SOFT_VCXO_DMA_WRITER);
    pll_sys_hw->fbdiv_int = VX_NOM;
    pio_sm_clear_fifos(pio, VX_SM);
    hw_clear_bits(&clocks_hw->clk[clk_gpout3].ctrl, CLOCKS_CLK_GPOUT3_CTRL_ENABLE_BITS);
    run_u = 0;
    vx_state = SOFT_VCXO_ST_PARKED;
#if DSPI_CLOCK_DIAG
    restart_req = false;
    diag_stops++;
#endif
}

DSP_TIME_CRITICAL
void soft_vcxo_service(void) {
    int32_t u = servo_u;
    bool run = servo_run;
#if DSPI_CLOCK_DIAG
    if (force_on) { u = force_u; run = force_run; }
#endif
    switch (vx_state) {
    case SOFT_VCXO_ST_PARKED:
        if (run) vx_start(u);
        break;
    case SOFT_VCXO_ST_RUNNING:
#if DSPI_CLOCK_DIAG
        if (restart_req) run = false;
#endif
        if (!run) vx_begin_stop();
        else if ((uint32_t)(time_us_32() - fill_t) >= VX_FILL_PERIOD_US) vx_fill(u);
        break;
    case SOFT_VCXO_ST_DRAINING:
        if ((uint32_t)(time_us_32() - drain_t) >= VX_DRAIN_US) vx_finish_stop();
        break;
    default:
        break;
    }
}

DSP_TIME_CRITICAL
void soft_vcxo_set_ppm(float ppm) {
    if (vx_state == SOFT_VCXO_ST_INERT) return;
#if DSPI_CLOCK_DIAG
    if (ppm > VX_MAX_PPM || ppm < -VX_MAX_PPM) diag_clamp_sats++;
#endif
    ppm = vx_clamp(ppm);
    servo_ppm = ppm;
    servo_u = (int32_t)(ppm * VX_PPM_TO_Q24);
    servo_run = (ppm >= VX_PARK_PPM || ppm <= -VX_PARK_PPM);
#if DSPI_CLOCK_DIAG
    diag_set_calls++;
    if (ppm < diag_min_ppm) diag_min_ppm = ppm;
    if (ppm > diag_max_ppm) diag_max_ppm = ppm;
    int8_t sign = (ppm > 0.0f) ? 1 : (ppm < 0.0f ? -1 : 0);
    if (sign != 0 && diag_last_sign != 0 && sign != diag_last_sign)
        diag_sign_changes++;
    if (sign != 0) diag_last_sign = sign;
#endif
}

float soft_vcxo_current_ppm(void) {
    return servo_ppm;
}

#if DSPI_CLOCK_DIAG
void soft_vcxo_get_diag(SoftVcxoDiag *out) {
    bool running = (vx_state == SOFT_VCXO_ST_RUNNING);
    int32_t u = running ? run_u : 0;
    out->last_ppm     = servo_ppm;
    out->min_ppm      = diag_min_ppm;
    out->max_ppm      = diag_max_ppm;
    out->set_calls    = diag_set_calls;
    out->clamp_sats   = diag_clamp_sats;
    out->sign_changes = diag_sign_changes;
    // Mean pulse rate: |u| pulses per decision at XOSC / 30 decisions per second.
    out->pulse_hz     = (uint32_t)(((uint64_t)(u < 0 ? -u : u) * (VX_XOSC_HZ / 30u)) >> 24);
    out->pwm_wrap     = 0;
    out->pwm_div      = 0;
    out->fbdiv0       = (uint8_t)(u > 0 ? VX_NOM + 1u : u < 0 ? VX_NOM - 1u : VX_NOM);
    out->parked       = !running;
    diag_min_ppm = servo_ppm;
    diag_max_ppm = servo_ppm;
}

// tools/vcxo_shaped_bench.py decodes this layout (STATUS_FMT); change both together.
_Static_assert(sizeof(SoftVcxoShapedDiag) == 104, "DIAG.VCXO_STATUS layout changed");

void soft_vcxo_shaped_get_diag(SoftVcxoShapedDiag *out) {
    memset(out, 0, sizeof(*out));
    out->version         = 1;
    out->state           = vx_state;
    out->delay_d         = vx_d;
    out->forced          = force_on ? 1u : 0u;
    out->force_ppm       = force_ppm;
    out->servo_ppm       = servo_ppm;
    out->u_q24           = (vx_state == SOFT_VCXO_ST_RUNNING) ? run_u : 0;
    out->run_ppm         = (float)out->u_q24 / VX_PPM_TO_Q24;
    out->starts          = diag_starts;
    out->stops           = diag_stops;
    out->replays         = diag_replays;
    out->fill_calls      = diag_fill_calls;
    out->decisions       = diag_decisions;
    out->net_pulses      = diag_net;
    out->fill_cycles     = diag_fill_cycles;
    out->lead_min_bytes  = diag_lead_min;
    out->fill_gap_max_us = diag_gap_max;
    out->i1_peak         = diag_i1_peak;
    out->i2_peak         = diag_i2_peak;
    out->sat_hits        = diag_sat_hits;
    out->fbdiv_live      = (uint16_t)(pll_sys_hw->fbdiv_int & 0xffffu);
    out->sm_enabled      = (VX_PIO->ctrl >> VX_SM) & 1u;
    out->dma_busy        = (dma_channel_is_busy(SOFT_VCXO_DMA_WRITER) ? 1u : 0u) |
                           (dma_channel_is_busy(SOFT_VCXO_DMA_FEED) ? 2u : 0u);
    out->ring_bytes      = VX_RING_BYTES;
    out->overtaken       = diag_overtaken;
    diag_lead_min = VX_RING_BYTES;
    diag_gap_max = 0;
    diag_i1_peak = diag_i2_peak = 0;
}

void soft_vcxo_force(bool on, float ppm) {
    ppm = vx_clamp(ppm);
    force_ppm = ppm;
    force_u = (int32_t)(ppm * VX_PPM_TO_Q24);
    force_run = (ppm >= VX_PARK_PPM || ppm <= -VX_PARK_PPM);
    force_on = on;
}

bool soft_vcxo_get_force(float *ppm) {
    *ppm = force_ppm;
    return force_on;
}

bool soft_vcxo_set_delay(uint8_t d) {
    if (d < SOFT_VCXO_D_MIN || d > SOFT_VCXO_D_MAX) return false;
    vx_d = d;
    if (vx_state == SOFT_VCXO_ST_RUNNING) restart_req = true;
    return true;
}

uint8_t soft_vcxo_get_delay(void) {
    return vx_d;
}
#endif // DSPI_CLOCK_DIAG

#endif // SOFT_VCXO_SHAPED
