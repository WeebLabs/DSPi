// Soft-VCXO profiler: drives the firmware's soft_vcxo.c on the DSPi clock
// plan and measures the real sys_clk offset with fc0 against the crystal.
// Send 'g' over USB CDC to run the suite; results print as CSV-ish lines.

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pll.h"
#include "hardware/structs/clocks.h"
#include "hardware/sync.h"
#include "hardware/vreg.h"
#include "../../firmware/DSPi/config.h"
#include "../../firmware/DSPi/soft_vcxo.h"

#define NOM_HZ   307200000.0
#define FBDIV_N  128.0

// One fc0 gate (~32.8 ms) of pll_sys against clk_ref, in 1/32 kHz units.
static uint32_t fc0_raw(void) {
    while (clocks_hw->fc0.status & CLOCKS_FC0_STATUS_RUNNING_BITS) tight_loop_contents();
    clocks_hw->fc0.ref_khz  = clock_get_hz(clk_ref) / 1000u;
    clocks_hw->fc0.interval = 15;
    clocks_hw->fc0.min_khz  = 0;
    clocks_hw->fc0.max_khz  = 0xffffffffu;
    clocks_hw->fc0.src      = CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY;
    while (!(clocks_hw->fc0.status & CLOCKS_FC0_STATUS_DONE_BITS)) tight_loop_contents();
    uint32_t r = clocks_hw->fc0.result;
    clocks_hw->fc0.src = 0;
    return r;
}

typedef struct { double ppm, min_ppm, max_ppm; } meas_t;

static double raw_to_ppm(double raw) { return (raw * 31.25 - NOM_HZ) / NOM_HZ * 1e6; }

static meas_t measure(int gates) {
    uint64_t sum = 0;
    uint32_t lo = UINT32_MAX, hi = 0;
    for (int i = 0; i < gates; i++) {
        uint32_t r = fc0_raw();
        sum += r;
        if (r < lo) lo = r;
        if (r > hi) hi = r;
    }
    meas_t m = { raw_to_ppm((double)sum / gates), raw_to_ppm(lo), raw_to_ppm(hi) };
    return m;
}

// Core 1 lock poller: counts pll_sys CS reads with LOCK clear while armed.
static volatile bool poll_run = false;
static volatile uint32_t poll_reads = 0, poll_unlocked = 0;

static void core1_main(void) {
    while (true) {
        if (!poll_run) { tight_loop_contents(); continue; }
        uint32_t reads = 0, unl = 0;
        while (poll_run) {
            if (!(pll_sys_hw->cs & PLL_CS_LOCK_BITS)) unl++;
            reads++;
        }
        poll_reads = reads;
        poll_unlocked = unl;
    }
}

static uint32_t pulse_hz_now(void) {
    SoftVcxoDiag d;
    soft_vcxo_get_diag(&d);
    return d.parked ? 0 : d.pulse_hz;
}

static void set_width(uint32_t w) { dma_timer_set_fraction(SOFT_VCXO_DMA_TIMER, 1, (uint16_t)w); }

static double baseline_ppm;

static void run_suite(void) {
    printf("# pll_sys cs=0x%08lx pwr=0x%08lx fbdiv=%lu prim=0x%08lx sys=%lu ref=%lu\n",
           (unsigned long)pll_sys_hw->cs, (unsigned long)pll_sys_hw->pwr,
           (unsigned long)pll_sys_hw->fbdiv_int, (unsigned long)pll_sys_hw->prim,
           (unsigned long)clock_get_hz(clk_sys), (unsigned long)clock_get_hz(clk_ref));

    soft_vcxo_set_ppm(0.0f);
    set_width(150);
    sleep_ms(200);
    meas_t b = measure(60);
    baseline_ppm = b.ppm;
    printf("BASE,ppm=%.4f,min=%.3f,max=%.3f\n", b.ppm, b.min_ppm, b.max_ppm);

    // T1: static gain at the shipped 150-cycle width.
    // 1200 checks the clamp: it should read back as the clamp value.
    static const float cmds[] = { 0.5f, 1, 2, 5, 10, 20, 30, 50, 100, 200, 300,
                                  400, 500, 600, 800, 1000, 1200 };
    for (int s = 1; s >= -1; s -= 2) {
        for (unsigned i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
            float c = s * cmds[i];
            soft_vcxo_set_ppm(c);
            sleep_ms(150);
            meas_t m = measure(32);
            double rel = m.ppm - baseline_ppm;
            printf("SWEEP,cmd=%.2f,meas=%.4f,gain=%.4f,min=%.3f,max=%.3f,pulse_hz=%lu\n",
                   c, rel, rel / c, m.min_ppm - baseline_ppm, m.max_ppm - baseline_ppm,
                   (unsigned long)pulse_hz_now());
        }
    }
    soft_vcxo_set_ppm(0.0f);
    sleep_ms(200);

    // T2: phase per pulse vs pulse width at a fixed pulse rate (100 ppm at W=150).
    static const uint16_t pos_w[] = { 64, 96, 128, 150, 192, 256, 384, 512 };
    static const uint16_t neg_w[] = { 128, 150, 192, 256, 384, 512 };
    for (int s = 1; s >= -1; s -= 2) {
        const uint16_t *ws = (s > 0) ? pos_w : neg_w;
        unsigned nw = (s > 0) ? sizeof(pos_w) / 2 : sizeof(neg_w) / 2;
        for (unsigned i = 0; i < nw; i++) {
            set_width(ws[i]);
            soft_vcxo_set_ppm(s * 100.0f);
            sleep_ms(150);
            meas_t m = measure(32);
            double rel = m.ppm - baseline_ppm;
            uint32_t ph = pulse_hz_now();
            double cyc = rel * 1e-6 * NOM_HZ / (double)ph;   // sys cycles per pulse
            double ideal = s * ws[i] / FBDIV_N;
            printf("WIDTH,sign=%d,w=%u,ref_periods=%.2f,meas=%.4f,pulse_hz=%lu,cyc_per_pulse=%.4f,ideal=%.4f,ratio=%.4f\n",
                   s, ws[i], ws[i] / 25.6, rel, (unsigned long)ph, cyc, ideal, cyc / ideal);
            soft_vcxo_set_ppm(0.0f);
            sleep_ms(50);
        }
    }
    set_width(150);

    // T3: lock bit under maximum pulse rate, and fc0 with the APB being hammered.
    static const float lk[] = { 1000.0f, -1000.0f, 300.0f, -300.0f, 30.0f };
    for (unsigned i = 0; i < sizeof(lk) / sizeof(lk[0]); i++) {
        soft_vcxo_set_ppm(lk[i]);
        sleep_ms(150);
        meas_t quiet = measure(32);
        poll_run = true;
        meas_t busy = measure(32);
        poll_run = false;
        sleep_ms(5);
        printf("LOCK,cmd=%.1f,meas_quiet=%.4f,meas_polled=%.4f,reads=%lu,unlocked=%lu\n",
               lk[i], quiet.ppm - baseline_ppm, busy.ppm - baseline_ppm,
               (unsigned long)poll_reads, (unsigned long)poll_unlocked);
    }

    // T4: hold -800 ppm (a slow Level II source) for 20 s from park, one reading per second.
    soft_vcxo_set_ppm(0.0f);
    sleep_ms(300);
    soft_vcxo_set_ppm(-800.0f);
    absolute_time_t t0 = get_absolute_time();
    for (int i = 0; i < 20; i++) {
        meas_t m = measure(30);
        printf("HOLD,t_ms=%lld,meas=%.4f,min=%.3f,max=%.3f\n",
               absolute_time_diff_us(t0, get_absolute_time()) / 1000,
               m.ppm - baseline_ppm, m.min_ppm - baseline_ppm, m.max_ppm - baseline_ppm);
    }
    soft_vcxo_set_ppm(0.0f);
    printf("DONE\n");
}

int main(void) {
    // Same plan as the DSPi RP2040 boot: 1.15 V, VCO 1536 MHz / 5 / 1.
    vreg_set_voltage(VREG_VOLTAGE_1_15);
    sleep_ms(10);
    set_sys_clock_pll(1536000000, 5, 1);
    stdio_init_all();
    soft_vcxo_init();
    multicore_launch_core1(core1_main);

    while (true) {
        int c = getchar_timeout_us(1000000);
        if (c == 'g') run_suite();
        else printf("READY\n");
    }
}
