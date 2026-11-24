// FBDIV pulse-count bench (RP2350, 307.2 MHz plan). A PIO TDC samples the
// crystal on a GPOUT pin to read the sys/ref phase to 1 VCO cycle; pulses come
// from a DMA-timer table (the real topology) or a PIO pacer (exact width/phase).
// Line commands over USB CDC; see run_bench.py.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "hardware/vreg.h"
#include "hardware/watchdog.h"
#include "hardware/structs/xosc.h"
#include "hardware/spi.h"
#include "hardware/resets.h"
#include "hardware/structs/hstx_ctrl.h"
#include "hardware/structs/hstx_fifo.h"
#include "hardware/structs/accessctrl.h"
#include "hardware/structs/m33.h"
#include "hardware/pll.h"
// hardware/pll.h aliases pll_sys, which mangles accessctrl_hw->pll_sys.
#undef pll_sys
#include "tdc.pio.h"

#define SM_TDC     0
#define SM_PACE    1
#define CH_PULSE   0
#define CH_PACER   1
#define CH_LD0     2    // 2..5 paced load channels
#define CH_MEMCPY  6
#define NOMINAL    128u
#define MAXF       700

static uint ref_pin = 25;
static uint tdc_off, pacer_off;
static int dir = 0;            // change of s per inserted VCO cycle, set by K
static int fixH = 65;          // 0: fit H per snapshot
static uint32_t settle_us = 300;
static uint32_t pulse_w = 128;
static uint8_t u_of[128];

static uint32_t tab_p[2] = { NOMINAL + 1, NOMINAL };
static uint32_t tab_m[2] = { NOMINAL - 1, NOMINAL };
static uint32_t dtab[4000 + 4];

static uint32_t cap_w[MAXF * 4];
static uint32_t cap_t[MAXF];

static inline uint32_t cyc(void) { return m33_hw->dwt_cyccnt; }
static inline int wrap128(int x) { return ((x + 64) & 127) - 64; }

static uint32_t rng = 0x12345678u;
static inline uint32_t xrand(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }

// ---------------------------------------------------------------- watchdog breadcrumbs
// Scratch 0..3 (4..7 belong to the boot ROM): stage+command, PLL CS, clk_sys selected
// plus FBDIV readback, and a stage-specific context word. Printed after a watchdog reboot.

#define CRUMB_MAGIC 0xb0d10000u
enum { ST_IDLE = 1, ST_CMD, ST_CAPTURE, ST_XBURST, ST_CHAIN_RUN, ST_CHAIN_WAIT, ST_CHAIN_STOP,
       ST_FC0, ST_FIRE, ST_SETUP };
static uint8_t crumb_cmd;
static uint32_t boot_crumb[4];
static bool boot_wd;

static inline void crumb(uint32_t stage, uint32_t ctx) {
    watchdog_hw->scratch[0] = CRUMB_MAGIC | (stage << 8) | crumb_cmd;
    watchdog_hw->scratch[1] = pll_sys_hw->cs;
    watchdog_hw->scratch[2] = clocks_hw->clk[clk_sys].selected | (pll_sys_hw->fbdiv_int << 16);
    watchdog_hw->scratch[3] = ctx;
    watchdog_update();
}

// ---------------------------------------------------------------- TDC

static bool rx_word(uint32_t *out) {
    uint32_t n = 0;
    while (pio0_hw->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB + SM_TDC)))
        if (++n > 4000000u) return false;
    *out = pio0_hw->rxf[SM_TDC];
    return true;
}

static uint32_t cap_timeouts = 0;

// n consecutive frames after one discarded frame; returns frame gaps != 512 (or -1).
static int capture(uint32_t n) {
    watchdog_update();
    uint32_t irq = save_and_disable_interrupts();
    int bad = 0;
    uint32_t d;
    for (int i = 0; i < 4 && bad == 0; i++) if (!rx_word(&d)) bad = -1;
    for (uint32_t f = 0; f < n && bad == 0; f++) {
        if (!rx_word(&cap_w[4 * f])) { bad = -1; break; }
        cap_t[f] = cyc();
        for (int i = 1; i < 4; i++) if (!rx_word(&cap_w[4 * f + i])) { bad = -1; break; }
    }
    restore_interrupts(irq);
    if (bad < 0) { cap_timeouts++; return -1; }
    for (uint32_t f = 1; f < n; f++) {
        uint32_t g = cap_t[f] - cap_t[f - 1];
        if (g < 448 || g > 576) bad++;
    }
    return bad;
}

typedef struct { int s, H, err, amb; } fit_t;

static fit_t decode(const uint32_t *w) {
    uint8_t ones[128], known[128];
    memset(ones, 0, sizeof ones);
    memset(known, 0, sizeof known);
    int tot = 0;
    for (int j = 0; j < 128; j++) {
        int b = (w[j >> 5] >> (j & 31)) & 1;
        known[u_of[j]]++;
        ones[u_of[j]] += b;
        tot += b;
    }
    fit_t f = { -1, 0, 1 << 30, 0 };
    int hlo = fixH ? fixH : 48, hhi = fixH ? fixH : 80;
    for (int H = hlo; H <= hhi; H++) {
        int ko = 0, oo = 0;
        for (int p = 0; p < H; p++) { ko += known[p]; oo += ones[p]; }
        for (int s = 0; s < 128; s++) {
            // With H fixed, the sample just past the falling edge is ignored: it flickers.
            int e = (ko - oo) + (tot - oo) - (fixH ? ones[(s + H) & 127] : 0);
            if (e < f.err) { f.err = e; f.s = s; f.H = H; f.amb = 1; }
            else if (e == f.err) f.amb++;
            int q = (s + H) & 127;
            ko += known[q] - known[s];
            oo += ones[q] - ones[s];
        }
    }
    return f;
}

static uint32_t meas_fail = 0;
static uint32_t meas_n = 64;
static uint32_t qh[6];          // |step residual| in tenths: <.1 <.2 <.3 <.4 <.5, and fails

#define FX 1024
#define FX_WRAP (128 * FX)

// Settled phase in 1/1024 VCO cycle: mean over meas_n frames. Edge samples
// flicker frame to frame, so a single frame can be off by one. -1 on failure.
static int meas(void) {
    for (int tries = 0; tries < 4; tries++) {
        if (capture(meas_n) != 0) continue;
        int s0 = decode(&cap_w[0]).s, acc = 0;
        for (uint32_t i = 1; i < meas_n; i++) acc += wrap128(decode(&cap_w[4 * i]).s - s0);
        return (s0 * FX + acc * FX / (int)meas_n + FX_WRAP) % FX_WRAP;
    }
    meas_fail++;
    return -1;
}

static inline int sround(int fx) { return ((fx + FX / 2) / FX) & 127; }

// Wrapped difference b - a in VCO cycles, rounded; logs the rounding residual.
static int dstep(int a, int b, bool *ok) {
    if (a < 0 || b < 0) { *ok = false; qh[5]++; return 0; }
    int d = (b - a) & (FX_WRAP - 1);
    if (d >= FX_WRAP / 2) d -= FX_WRAP;
    int r = d >= 0 ? (d + FX / 2) / FX : -((-d + FX / 2) / FX);
    int res = d - r * FX;
    if (res < 0) res = -res;
    int q = res * 10 / FX;
    qh[q > 4 ? 4 : q]++;
    *ok = res < 4 * FX / 10;
    return r;
}

static void print_qh(const char *tag) {
    printf("QH,%s,lt0.1=%lu,lt0.2=%lu,lt0.3=%lu,lt0.4=%lu,lt0.5=%lu,fail=%lu\n", tag,
           (unsigned long)qh[0], (unsigned long)qh[1], (unsigned long)qh[2], (unsigned long)qh[3],
           (unsigned long)qh[4], (unsigned long)qh[5]);
    memset(qh, 0, sizeof qh);
}

static void tdc_start(void) {
    pio_sm_set_enabled(pio0, SM_TDC, false);
    pio_sm_clear_fifos(pio0, SM_TDC);
    pio_sm_restart(pio0, SM_TDC);
    pio_sm_put(pio0, SM_TDC, 127);
    pio_sm_exec(pio0, SM_TDC, pio_encode_pull(false, true));
    pio_sm_exec(pio0, SM_TDC, pio_encode_mov(pio_y, pio_osr));
    pio_sm_exec(pio0, SM_TDC, pio_encode_jmp(tdc_off));
    pio_sm_set_enabled(pio0, SM_TDC, true);
}

static void ref_pin_setup(uint pin) {
    clock_gpio_init(pin, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, 1);
    gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);
    gpio_set_input_enabled(pin, true);
}

static void pio_setup(void) {
    tdc_off = pio_add_program(pio0, &tdc_program);
    pacer_off = pio_add_program(pio0, &pacer_program);
    pio_sm_config c = tdc_program_get_default_config(tdc_off);
    sm_config_set_in_pins(&c, ref_pin);
    // RX not joined: the TX FIFO is needed once at start to load Y.
    sm_config_set_in_shift(&c, true, true, 32);
    sm_config_set_mov_status(&c, STATUS_RX_LESSTHAN, 1);
    sm_config_set_clkdiv_int_frac(&c, 1, 0);
    pio_sm_init(pio0, SM_TDC, tdc_off, &c);

    c = pacer_program_get_default_config(pacer_off);
    sm_config_set_out_shift(&c, true, false, 32);
    sm_config_set_in_shift(&c, true, false, 32);
    sm_config_set_clkdiv_int_frac(&c, 1, 0);
    pio_sm_init(pio0, SM_PACE, pacer_off, &c);
    pio_sm_set_enabled(pio0, SM_PACE, true);
    tdc_start();

    for (int j = 0; j < 128; j++) u_of[j] = (uint8_t)((15 * j) & 127);
}

// ---------------------------------------------------------------- pulses

static void pulse_dma_setup(void) {
    accessctrl_hw->pll_sys = 0xacce0000u | 0xffu;
    dma_channel_claim(CH_PULSE);
    dma_channel_claim(CH_PACER);
    dma_timer_claim(0);
    dma_timer_set_fraction(0, 1, (uint16_t)pulse_w);
    dma_channel_config c = dma_channel_get_default_config(CH_PULSE);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, dma_get_timer_dreq(0));
    channel_config_set_high_priority(&c, true);
    dma_channel_configure(CH_PULSE, &c, &pll_sys_hw->fbdiv_int, tab_p, 2, false);

    c = dma_channel_get_default_config(CH_PACER);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio0, SM_PACE, false));
    channel_config_set_high_priority(&c, true);
    dma_channel_configure(CH_PACER, &c, &pll_sys_hw->fbdiv_int, &pio0_hw->rxf[SM_PACE],
                          dma_encode_transfer_count(0x0fffffffu), true);
}

static bool tab_safe(const uint32_t *tab, uint32_t n) {
    if (pulse_w >= 120) return true;
    for (uint32_t i = 0; i < n; i++) if (tab[i] < NOMINAL) return false;
    return true;
}

static bool noirq = false;
static bool quietwait = false;
static int trig_d = -1;         // >= 0: trigger this many cycles after a TDC frame's last word

// Each triggered timer-paced sequence moves the DMA timer 0 tick 2 sys cycles
// later (measured, see run_bench logs), so the write phase is 10*ntrig - s.
static uint32_t ntrig = 0;
static inline int tphase(int sfx) { return (int)((10u * ntrig - (uint32_t)sround(sfx)) & 127u); }

static void fire_tab(const uint32_t *tab, uint32_t n) {
    ntrig++;
    uint32_t st = (noirq || quietwait || trig_d >= 0) ? save_and_disable_interrupts() : 0;
    if (trig_d >= 0) {
        uint32_t w, t;
        for (int i = 0; i < 4; i++) rx_word(&w);     // finish any queued frame
        for (int i = 0; i < 4; i++) rx_word(&w);     // then one whole fresh frame
        t = cyc();
        while (cyc() - t < (uint32_t)trig_d) ;
    }
    dma_channel_transfer_from_buffer_now(CH_PULSE, tab, n);
    if (quietwait) {        // keep the CPU off the bus until the table has run out
        uint32_t t = cyc();
        while (cyc() - t < (n + 2) * pulse_w + 64) ;
    }
    for (uint32_t t = cyc(); dma_channel_is_busy(CH_PULSE); )
        if (cyc() - t > 300000000u) { crumb(ST_FIRE, 0xdead0001u); dma_channel_abort(CH_PULSE); break; }
    if (noirq || quietwait || trig_d >= 0) restore_interrupts(st);
}

static int lead = 0;            // no-op 128 entries before the excursion

// Pulse table in SRAM8 (scratch X): no code or stack lives there while core 1 is off.
static uint32_t t_scr[8] __attribute__((section(".scratch_x.pulsetab")));
static bool tab_scratch = false;
static int zpace_d = -1;        // >= 0: fire_sign uses the PIO pacer at this d instead
static void fire_pacer(uint32_t d, uint32_t w, uint32_t exc);

static void fire_sign(int sg) {
    if (zpace_d >= 0) {
        if (sg) fire_pacer((uint32_t)zpace_d, 128, NOMINAL + sg);
        return;
    }
    static uint32_t t_main[8];
    uint32_t *t = tab_scratch ? t_scr : t_main;
    for (int i = 0; i < 8; i++) t[i] = NOMINAL;
    t[lead] = NOMINAL + sg;
    fire_tab(t, (uint32_t)lead + 2);
}

static void fire_pacer(uint32_t d, uint32_t w, uint32_t exc) {
    if (exc < NOMINAL && w < 120) return;
    uint32_t c0 = dma_hw->ch[CH_PACER].transfer_count & 0x0fffffffu;
    pio_sm_put_blocking(pio0, SM_PACE, d | ((w - 4) << 16));
    pio_sm_put_blocking(pio0, SM_PACE, exc);
    pio_sm_put_blocking(pio0, SM_PACE, NOMINAL);
    for (uint32_t t = cyc(); c0 - (dma_hw->ch[CH_PACER].transfer_count & 0x0fffffffu) < 2; )
        if (cyc() - t > 30000000u) { crumb(ST_FIRE, 0xdead0002u); break; }
}

// ---------------------------------------------------------------- load

static volatile uint32_t hammer_on = 0, hammer_sink;
static uint32_t ld_src[4] __attribute__((aligned(16))) = { 1, 2, 3, 4 };
static uint32_t ld_ring[4][64] __attribute__((aligned(256)));
static uint32_t mc_src[1024] __attribute__((aligned(4096)));
static uint32_t mc_dst;
static uint32_t load_mask = 0;

static volatile uint32_t spin_word;
static uint32_t c1_src[1024], c1_dst[1024], c0_src[1024], c0_dst[1024];

// hammer_on 1: poll pll_sys CS (APB). 2: poll a RAM word. 3: memcpy 4 KB in a loop.
static bool core1_up = false;
static void rec_core1(int mode);

static void core1_main(void) {
    m33_hw->demcr |= M33_DEMCR_TRCENA_BITS;
    m33_hw->dwt_ctrl |= M33_DWT_CTRL_CYCCNTENA_BITS;
    while (true) {
        uint32_t a = 0;
        if (hammer_on == 10 || hammer_on == 11) { rec_core1((int)hammer_on); hammer_on = 0; }
        if (hammer_on == 1) while (hammer_on == 1) a += pll_sys_hw->cs;
        if (hammer_on == 2) while (hammer_on == 2) a += spin_word;
        if (hammer_on == 3) while (hammer_on == 3) { memcpy(c1_dst, c1_src, sizeof c1_dst); a++; }
        hammer_sink = a;
    }
}

// bit0: 4 paced channels, bit1: core 1 polls pll_sys CS, bit2: unpaced memcpy,
// bit3: paced channels at high priority, bit4: core 1 polls a RAM word,
// bit5: core 1 memcpy loop, bit6: core 0 memcpy plus a CDC line during crystal bursts.
static void set_load(uint32_t mask) {
    for (int i = 0; i < 4; i++) dma_channel_abort(CH_LD0 + i);
    dma_channel_abort(CH_MEMCPY);
    hammer_on = 0;
    pwm_set_enabled(0, false);
    load_mask = mask;
    if (mask & 1) {
        static const uint16_t div[3] = { 1024, 768, 1536 };
        for (int t = 0; t < 3; t++) dma_timer_set_fraction(1 + t, 1, div[t]);
        pwm_config pc = pwm_get_default_config();
        pwm_config_set_wrap(&pc, 1023);
        pwm_init(0, &pc, true);
        for (int i = 0; i < 4; i++) {
            uint ch = CH_LD0 + i;
            dma_channel_config c = dma_channel_get_default_config(ch);
            channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
            channel_config_set_high_priority(&c, (mask & 8) != 0);
            channel_config_set_dreq(&c, i < 3 ? dma_get_timer_dreq(1 + i) : pwm_get_dreq(0));
            if (i < 2) {        // RAM to RAM
                channel_config_set_read_increment(&c, false);
                channel_config_set_write_increment(&c, true);
                channel_config_set_ring(&c, true, 8);
                dma_channel_configure(ch, &c, ld_ring[i], &ld_src[i],
                                      dma_encode_endless_transfer_count(), true);
            } else {            // RAM to idle PIO SM TX FIFO
                channel_config_set_read_increment(&c, true);
                channel_config_set_write_increment(&c, false);
                channel_config_set_ring(&c, false, 8);
                dma_channel_configure(ch, &c, &pio0_hw->txf[i], ld_ring[i],
                                      dma_encode_endless_transfer_count(), true);
            }
        }
    }
    // Core 1 is held in reset unless a load needs it: an idle spin is itself SRAM load.
    if (mask & (2 | 16 | 32)) {
        if (!core1_up) { multicore_launch_core1(core1_main); core1_up = true; }
        hammer_on = (mask & 2) ? 1 : (mask & 32) ? 3 : 2;
    } else if (core1_up) {
        multicore_reset_core1();
        core1_up = false;
    }
    if (mask & 4) {
        dma_channel_config c = dma_channel_get_default_config(CH_MEMCPY);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_read_increment(&c, true);
        channel_config_set_write_increment(&c, false);
        channel_config_set_ring(&c, false, 12);
        dma_channel_configure(CH_MEMCPY, &c, &mc_dst, mc_src,
                              dma_encode_endless_transfer_count(), true);
    }
}

// ---------------------------------------------------------------- fc0

static uint32_t fc0_raw(void) {
    crumb(ST_FC0, 0);
    while (clocks_hw->fc0.status & CLOCKS_FC0_STATUS_RUNNING_BITS) tight_loop_contents();
    clocks_hw->fc0.ref_khz  = clock_get_hz(clk_ref) / 1000u;
    clocks_hw->fc0.interval = 15;
    clocks_hw->fc0.min_khz  = 0;
    clocks_hw->fc0.max_khz  = 0xffffffffu;
    clocks_hw->fc0.src      = CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY;
    // Sit off the bus for most of the ~33 ms gate: polling the APB is itself bus load.
    uint32_t t = cyc();
    while (cyc() - t < 9800000u) ;
    while (!(clocks_hw->fc0.status & CLOCKS_FC0_STATUS_DONE_BITS)) tight_loop_contents();
    uint32_t r = clocks_hw->fc0.result;
    clocks_hw->fc0.src = 0;
    return r;
}

static double fc0_mean(int gates) {
    uint64_t s = 0;
    for (int i = 0; i < gates; i++) s += fc0_raw();
    return (double)s / gates * 31.25;
}

static uint32_t ring_tab[32] __attribute__((aligned(128)));

// Continuous unipolar stream, one pulse per sp slots (sp = 8, 16 or 32).
static void cmd_fc0(int sp, int sign, int gates) {
    int bits = sp == 8 ? 5 : sp == 16 ? 6 : 7;
    if (sp != 8 && sp != 16 && sp != 32) { printf("ERR sp\n"); return; }
    for (int i = 0; i < sp; i++) ring_tab[i] = NOMINAL;
    ring_tab[0] = NOMINAL + sign;
    double base = fc0_mean(gates);
    dma_channel_config c = dma_channel_get_default_config(CH_PULSE);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_ring(&c, false, bits);
    channel_config_set_dreq(&c, dma_get_timer_dreq(0));
    channel_config_set_high_priority(&c, true);
    dma_channel_configure(CH_PULSE, &c, &pll_sys_hw->fbdiv_int, ring_tab,
                          dma_encode_endless_transfer_count(), true);
    sleep_ms(50);
    double on = fc0_mean(gates);
    dma_channel_abort(CH_PULSE);
    pll_sys_hw->fbdiv_int = NOMINAL;
    double after = fc0_mean(gates);
    dma_channel_cleanup(CH_PULSE);
    dma_channel_unclaim(CH_PULSE);
    dma_channel_unclaim(CH_PACER);
    dma_timer_unclaim(0);
    dma_channel_abort(CH_PACER);
    pulse_dma_setup();
    // Pulses are spaced in sys cycles, so their real rate scales with the offset.
    double rate = 307.2e6 / (pulse_w * (double)sp) * on / ((base + after) / 2);
    double dppm = (on - (base + after) / 2) / 307.2e6 * 1e6;
    double vco = dppm * 1e-6 * 1536e6 / rate;
    printf("FC0,sp=%d,sign=%d,gates=%d,base_hz=%.2f,on_hz=%.2f,after_hz=%.2f,pulse_hz=%.1f,dppm=%.5f,vco_per_pulse=%.6f,load=%lu\n",
           sp, sign, gates, base, on, after, rate, dppm, vco, (unsigned long)load_mask);
}


// ---------------------------------------------------------------- crystal pacer (PIO1)

#define CH_XFEED   8
#define CH_XW      9
#define XMAX       4000

static uint xp_off, xpf_off;
static int xp_d = 0, xp_n = 30, xp_edge = 0;    // edge 0: rising (PIO1), 1: falling (PIO2)
static PIO xp_pio;
static uint32_t xtab[XMAX];
static uint32_t xring[1] __attribute__((aligned(4)));

static inline uint32_t xword(int v) { return (uint32_t)v | (NOMINAL << 8) | ((uint32_t)(xp_n - 7) << 16); }

static PIO xw_pio;
static uint xw_sm;
static uint32_t xw_rearms = 0;
static inline uint32_t xw_count(void);

static void xw_arm(void) {
    xw_pio = xp_pio;
    xw_sm = 0;
    dma_channel_abort(CH_XW);
    dma_channel_config cc = dma_channel_get_default_config(CH_XW);
    channel_config_set_transfer_data_size(&cc, DMA_SIZE_32);
    channel_config_set_read_increment(&cc, false);
    channel_config_set_write_increment(&cc, false);
    channel_config_set_dreq(&cc, pio_get_dreq(xw_pio, xw_sm, false));
    channel_config_set_high_priority(&cc, true);
    dma_channel_configure(CH_XW, &cc, &pll_sys_hw->fbdiv_int, &xw_pio->rxf[xw_sm],
                          dma_encode_transfer_count(0x0fffffffu), true);
}

// CH_XW counts down from 2^28 - 1; re-arm it before it can run dry inside a long soak.
static void xw_refresh(void) {
    if (xw_count() > 0x08000000u) return;
    dma_channel_abort(CH_XW);
    dma_channel_config cc = dma_channel_get_default_config(CH_XW);
    channel_config_set_transfer_data_size(&cc, DMA_SIZE_32);
    channel_config_set_read_increment(&cc, false);
    channel_config_set_write_increment(&cc, false);
    channel_config_set_dreq(&cc, pio_get_dreq(xw_pio, xw_sm, false));
    channel_config_set_high_priority(&cc, true);
    dma_channel_configure(CH_XW, &cc, &pll_sys_hw->fbdiv_int, &xw_pio->rxf[xw_sm],
                          dma_encode_transfer_count(0x0fffffffu), true);
    xw_rearms++;
}

// Only call while both pacers are idle (stalled on pull with their TX FIFOs empty).
static void xp_config(int d, int n, int edge) {
    xp_d = d < 0 ? 0 : d > 31 ? 31 : d;
    xp_n = n < 7 ? 7 : n;
    xp_edge = edge ? 1 : 0;
    xp_pio = xp_edge ? pio2 : pio1;
    pio_sm_set_enabled(xp_pio, 0, false);
    pio_sm_exec(xp_pio, 0, pio_encode_set(pio_x, (uint)xp_d));
    pio_sm_set_enabled(xp_pio, 0, true);
    xw_arm();
}

static void xp_sm_init(PIO pio, uint off, const pio_program_t *prog) {
    pio_sm_config c = pio == pio1 ? xpacer_program_get_default_config(off)
                                  : xpacerf_program_get_default_config(off);
    (void)prog;
    sm_config_set_in_pins(&c, ref_pin);
    sm_config_set_out_shift(&c, true, false, 32);
    sm_config_set_in_shift(&c, true, false, 32);
    sm_config_set_clkdiv_int_frac(&c, 1, 0);
    pio_sm_init(pio, 0, off, &c);
    pio_sm_set_enabled(pio, 0, true);
}

static void xp_setup(void) {
    xp_off = pio_add_program(pio1, &xpacer_program);
    xpf_off = pio_add_program(pio2, &xpacerf_program);
    xp_sm_init(pio1, xp_off, &xpacer_program);
    xp_sm_init(pio2, xpf_off, &xpacerf_program);
    dma_channel_claim(CH_XFEED);
    dma_channel_claim(CH_XW);
    xp_config(0, 30, 0);
}

static void xfeed_start(const uint32_t *tab, uint32_t n, bool ring) {
    dma_channel_config cc = dma_channel_get_default_config(CH_XFEED);
    channel_config_set_transfer_data_size(&cc, DMA_SIZE_32);
    channel_config_set_read_increment(&cc, true);
    channel_config_set_write_increment(&cc, false);
    if (ring) channel_config_set_ring(&cc, false, 2);
    channel_config_set_dreq(&cc, pio_get_dreq(xp_pio, 0, true));
    dma_channel_configure(CH_XFEED, &cc, &xp_pio->txf[0], tab,
                          ring ? dma_encode_endless_transfer_count() : n, true);
}

static inline uint32_t xw_count(void) {
    return dma_hw->ch[CH_XW].transfer_count & 0x0fffffffu; }

static int xkind = 0;            // 0 crystal PIO pacer, 1 SPI1 chain, 2 HSTX chain, 3 6-instr pacer
static int cdelay = -1;         // chain pacers: < 0 writes FBDIV directly, else PIO delay
static bool chain_burst(uint32_t n);

// Runs one burst of n slots and waits until all 2n FBDIV writes are done.
static bool x6_burst(uint32_t n);

static bool xburst(uint32_t n) {
    if (xkind == 0 || xkind == 3 || cdelay >= 0) xw_refresh();
    if (xkind == 3) return x6_burst(n);
    if (xkind) return chain_burst(n);
    uint32_t c0 = xw_count();
    crumb(ST_XBURST, n);
    xfeed_start(xtab, n, false);
    uint32_t t0 = cyc(), lim = (uint32_t)((uint64_t)n * (uint32_t)xp_n * 26u) + 3000000u;
    static uint32_t k;
    while (c0 - xw_count() < 2 * n) {
        if (load_mask & 64) memcpy(c0_dst, c0_src, 512);
        if (cyc() - t0 > lim) { crumb(ST_XBURST, 0xdead0003u); return false; }
    }
    if (load_mask & 64) printf("#cdc %08lx burst traffic line padding padding padding\n", (unsigned long)k++);
    return true;
}

static double sd_i1, sd_i2;
static int xverbose = 0;        // print the first n bursts of each g command

// Second-order three-level sigma-delta: u = pulses per slot.
static int sd_step(double u) {
    int y = sd_i2 > 0.5 ? 1 : sd_i2 < -0.5 ? -1 : 0;
    sd_i1 += u - y;
    sd_i2 += sd_i1 - y;
    return y;
}

// Crystal-paced bursts: n slots each. mode 0 random, 1 all +, 2 all -, 3 none,
// 4 alternating, 6 sigma-delta at arg/100 ppm (u from the 1536 MHz VCO).
static void cmd_X(int nb, int n, int mode, int arg) {
    if (!dir) { printf("ERR dir\n"); return; }
    if (n < 1 || n > XMAX) { printf("ERR size\n"); return; }
    double u = arg / 100.0 * 1e-6 * 1536e6 / 5.0 / (12e6 / xp_n);
    uint32_t dh[65] = { 0 }, dother = 0, fails = 0, unlock = 0, lockn = 0, nz = 0, uncertain = 0, tmo = 0;
    int64_t sumdev = 0, sumabs = 0;
    uint32_t pb_n[16] = { 0 }, pb_abs[16] = { 0 };
    uint64_t pulses = 0, plus = 0;
    uint32_t t_start = time_us_32();
    int s = meas();
    for (int b = 0; b < nb; b++) {
        if (s < 0) { s = meas(); fails++; continue; }
        int net = 0, np = 0;
        for (int j = 0; j < n; j++) {
            int sg = mode == 1 ? 1 : mode == 2 ? -1 : mode == 3 ? 0 : mode == 4 ? (j & 1 ? -1 : 1)
                   : mode == 6 ? sd_step(u) : ((xrand() >> 7) & 1 ? 1 : -1);
            xtab[j] = xword((int)NOMINAL + sg);
            net += sg;
            if (sg) np++;
            if (sg > 0) plus++;
        }
        int s0 = s;
        if (!xburst((uint32_t)n)) { tmo++; break; }
        busy_wait_us(settle_us);
        uint32_t cs = pll_sys_hw->cs;
        if (!(cs & PLL_CS_LOCK_BITS)) unlock++;
        if (cs & PLL_CS_LOCK_N_BITS) lockn++;
        int s1 = meas();
        pulses += np;
        if (s1 < 0) { fails++; s = meas(); continue; }
        bool ok;
        int dev = wrap128(dstep(s0, s1, &ok) - dir * 5 * net);
        if (!ok) uncertain++;
        if (dev >= -32 && dev <= 32) dh[dev + 32]++; else dother++;
        sumdev += dev;
        sumabs += dev < 0 ? -dev : dev;
        if (dev) nz++;
        if (b < xverbose) printf("XB,%d,net=%d,np=%d,s0=%d,s1=%d,ds=%d,dev=%d\n", b, net, np, s0, s1,
                                 dstep(s0, s1, &ok), dev);
        int pb = sround(s0) >> 3;
        pb_n[pb]++;
        pb_abs[pb] += dev < 0 ? -dev : dev;
        s = s1;
        if ((b % 20000) == 19999)
            printf("XPROG,b=%d,pulses=%llu,sumabs=%lld,nz=%lu,t_s=%lu\n", b + 1, (unsigned long long)pulses,
                   (long long)sumabs, (unsigned long)nz, (unsigned long)((time_us_32() - t_start) / 1000000u));
    }
    print_qh("X");
    printf("XS,kind=%d,cdelay=%d,edge=%d,d=%d,N=%d,mode=%d,arg=%d,load=%lu,bursts=%d,slots=%d,pulses=%llu,plus=%llu,sumdev=%lld,sumabs=%lld,nz=%lu,fails=%lu,unlock=%lu,lockn=%lu,other=%lu,uncertain=%lu,timeout=%lu,t_s=%lu\n",
           xkind, cdelay, xp_edge, xp_d, xp_n, mode, arg, (unsigned long)load_mask, nb, n, (unsigned long long)pulses,
           (unsigned long long)plus, (long long)sumdev, (long long)sumabs, (unsigned long)nz,
           (unsigned long)fails, (unsigned long)unlock, (unsigned long)lockn, (unsigned long)dother,
           (unsigned long)uncertain, (unsigned long)tmo, (unsigned long)((time_us_32() - t_start) / 1000000u));
    printf("XH,d=%d,mode=%d,load=%lu", xp_d, mode, (unsigned long)load_mask);
    for (int i = 0; i < 65; i++) if (dh[i]) printf(",%d:%lu", i - 32, (unsigned long)dh[i]);
    printf("\nXPB,d=%d,mode=%d,load=%lu", xp_d, mode, (unsigned long)load_mask);
    for (int i = 0; i < 16; i++) printf(",%d:%lu/%lu", i * 8, (unsigned long)pb_n[i], (unsigned long)pb_abs[i]);
    printf("\n");
}

// Continuous crystal-paced unipolar stream measured with fc0 (rate is exactly 12 MHz / N).
static void cmd_xfc0(int sign, int gates) {
    double base = fc0_mean(gates);
    xring[0] = xword((int)NOMINAL + sign);
    xfeed_start(xring, 0, true);
    sleep_ms(50);
    double on = fc0_mean(gates);
    dma_channel_abort(CH_XFEED);
    sleep_ms(2);
    double after = fc0_mean(gates);
    double rate = 12e6 / xp_n;
    double dhz = on - (base + after) / 2;
    printf("XFC0,edge=%d,d=%d,N=%d,sign=%d,gates=%d,base_hz=%.2f,on_hz=%.2f,after_hz=%.2f,pulse_hz=%.3f,dppm=%.5f,vco_per_pulse=%.6f,load=%lu,fbdiv_after=%lu\n",
           xp_edge, xp_d, xp_n, sign, gates, base, on, after, rate, dhz / 307.2e6 * 1e6, dhz * 5.0 / rate,
           (unsigned long)load_mask, (unsigned long)pll_sys_hw->fbdiv_int);
}


// ---------------------------------------------------------------- chain pacers (SPI1 / HSTX)
// X (DREQ = peripheral TX) writes one dummy word into the peripheral FIFO and chains to
// Y, which writes one table word (FBDIV value) to FBDIV or to the PIO delay stage and
// chains back to X. The peripheral consumes one word per 20 cycles of its 48 MHz clock.

#define CH_SX      10
#define CH_SY      11
#define CPRE       16           // leading no-op words while the peripheral FIFO fills
#define CPAD       24
#define CSLOTS     6            // slots of 5 ref periods per pulse: 400k pulses/s

#define CGUARD     2048         // 128-filled words after any table: a runaway Y writes no-ops
#define CMAXN      1000         // largest chain burst in pulses
static uint32_t ctab[CMAXN * CSLOTS + CPRE + CPAD + CGUARD];
static uint32_t guard_val = NOMINAL;    // hazard test only: what a runaway Y would write
static uint32_t cdummy;
static uint xd_off;
static bool xd_loaded = false;

static void chain_periph_start(void) {
    if (xkind == 1) {
        spi_init(spi1, 12000000);
        spi_hw_t *h = spi_get_hw(spi1);
        h->cr1 = 0;
        // 5-bit Motorola frames, SPH=1 (back to back), bit clock clk_peri/4: 20 clk_peri per word.
        h->cr0 = (4u << SPI_SSPCR0_DSS_LSB) | SPI_SSPCR0_SPH_BITS;
        h->cpsr = 4;
        h->cr1 = SPI_SSPCR1_SSE_BITS;
    } else {
        unreset_block_num_wait_blocking(RESET_HSTX);
        clock_configure(clk_hstx, 0, CLOCKS_CLK_HSTX_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, 48000000, 48000000);
        hstx_ctrl_hw->csr = 0;
        hstx_ctrl_hw->csr = (20u << HSTX_CTRL_CSR_N_SHIFTS_LSB) | (1u << HSTX_CTRL_CSR_SHIFT_LSB) |
                            (1u << HSTX_CTRL_CSR_CLKDIV_LSB) | HSTX_CTRL_CSR_EN_BITS;
    }
}

static void keep_start(void);
static void keep_stop(void);

static void chain_setup(int kind, int delay, bool reinit) {
    if (kind != xkind) reinit = true;
    if (reinit) keep_stop();
    dma_channel_abort(CH_SX);
    dma_channel_abort(CH_SY);
    xkind = kind;
    cdelay = delay;
    if (!xd_loaded) {
        xd_off = pio_add_program(pio2, &xdelay_program);
        pio_sm_config c = xdelay_program_get_default_config(xd_off);
        sm_config_set_clkdiv_int_frac(&c, 1, 0);
        pio_sm_init(pio2, 1, xd_off, &c);
        pio_sm_set_enabled(pio2, 1, true);
        dma_channel_claim(CH_SX);
        dma_channel_claim(CH_SY);
        xd_loaded = true;
    }
    if (kind == 0) { xp_config(xp_d, xp_n, xp_edge); return; }
    if (delay >= 0) {
        pio_sm_set_enabled(pio2, 1, false);
        pio_sm_exec(pio2, 1, pio_encode_set(pio_x, (uint)(delay > 31 ? 31 : delay)));
        pio_sm_set_enabled(pio2, 1, true);
        xw_pio = pio2;
        xw_sm = 1;
        dma_channel_abort(CH_XW);
        dma_channel_config cc = dma_channel_get_default_config(CH_XW);
        channel_config_set_transfer_data_size(&cc, DMA_SIZE_32);
        channel_config_set_read_increment(&cc, false);
        channel_config_set_write_increment(&cc, false);
        channel_config_set_dreq(&cc, pio_get_dreq(pio2, 1, false));
        channel_config_set_high_priority(&cc, true);
        dma_channel_configure(CH_XW, &cc, &pll_sys_hw->fbdiv_int, &pio2_hw->rxf[1],
                              dma_encode_transfer_count(0x0fffffffu), true);
    }
    // Re-initialising the peripheral re-rolls its word phase against the crystal.
    if (reinit) { chain_periph_start(); keep_start(); }
}

static void chain_run(const uint32_t *tab) {
    keep_stop();
    volatile void *pdst = xkind == 1 ? (volatile void *)&spi_get_hw(spi1)->dr
                                     : (volatile void *)&hstx_fifo_hw->fifo;
    dma_channel_config cx = dma_channel_get_default_config(CH_SX);
    channel_config_set_transfer_data_size(&cx, DMA_SIZE_32);
    channel_config_set_read_increment(&cx, false);
    channel_config_set_write_increment(&cx, false);
    channel_config_set_dreq(&cx, xkind == 1 ? spi_get_dreq(spi1, true) : DREQ_HSTX);
    channel_config_set_high_priority(&cx, true);
    channel_config_set_chain_to(&cx, CH_SY);
    dma_channel_configure(CH_SX, &cx, pdst, &cdummy, 1, false);
    dma_channel_config cy = dma_channel_get_default_config(CH_SY);
    channel_config_set_transfer_data_size(&cy, DMA_SIZE_32);
    channel_config_set_read_increment(&cy, true);
    channel_config_set_write_increment(&cy, false);
    channel_config_set_high_priority(&cy, true);
    channel_config_set_chain_to(&cy, CH_SX);
    dma_channel_configure(CH_SY, &cy, cdelay >= 0 ? (volatile void *)&pio2_hw->txf[1]
                                                  : (volatile void *)&pll_sys_hw->fbdiv_int,
                          tab, 1, false);
    dma_channel_start(CH_SX);
}

static uint32_t chain_tmo = 0;
static int stopmode = 1;
#define CH_KEEP    12
static bool keepalive = false;  // keep the peripheral FIFO fed between bursts: fixed word phase

static void keep_start(void) {
    if (!keepalive || !xkind || xkind == 3) return;
    volatile void *pdst = xkind == 1 ? (volatile void *)&spi_get_hw(spi1)->dr
                                     : (volatile void *)&hstx_fifo_hw->fifo;
    dma_channel_config ck = dma_channel_get_default_config(CH_KEEP);
    channel_config_set_transfer_data_size(&ck, DMA_SIZE_32);
    channel_config_set_read_increment(&ck, false);
    channel_config_set_write_increment(&ck, false);
    channel_config_set_dreq(&ck, xkind == 1 ? spi_get_dreq(spi1, true) : DREQ_HSTX);
    dma_channel_configure(CH_KEEP, &ck, pdst, &cdummy, dma_encode_endless_transfer_count(), true);
}

static void keep_stop(void) {
    if (dma_channel_is_busy(CH_KEEP)) dma_channel_abort(CH_KEEP);
}         // 0: abort both (phase 2 method), 1: unchain Y, then wait
static uint32_t chain_runaway = 0, chain_stop_tmo = 0;

static void chain_stop(void) {
    crumb(ST_CHAIN_STOP, dma_hw->ch[CH_SY].read_addr);
    if (stopmode == 0) {
        dma_channel_abort(CH_SX);
        dma_channel_abort(CH_SY);
    } else {
        uint32_t c = dma_hw->ch[CH_SY].al1_ctrl;
        dma_hw->ch[CH_SY].al1_ctrl = (c & ~DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS) | (CH_SY << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB);
        uint32_t t = cyc();
        while (dma_channel_is_busy(CH_SX) || dma_channel_is_busy(CH_SY))
            if (cyc() - t > 3072000u) { chain_stop_tmo++; dma_channel_abort(CH_SX); dma_channel_abort(CH_SY); break; }
    }
    keep_start();
    // A chain that keeps running after the stop walks into the 128-filled guard area.
    uint32_t a0 = dma_hw->ch[CH_SY].read_addr;
    crumb(ST_CHAIN_STOP, 0x5000000u | (a0 & 0xffffffu));
    busy_wait_us(20);
    if (dma_hw->ch[CH_SY].read_addr != a0) {
        chain_runaway++;
        uint32_t c = dma_hw->ch[CH_SY].al1_ctrl;
        dma_hw->ch[CH_SY].al1_ctrl = (c & ~DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS) | (CH_SY << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB);
        busy_wait_us(5);
        dma_channel_abort(CH_SX);
        dma_channel_abort(CH_SY);
    }
}

// The table ends in CPAD no-op words, so stopping the chain there can never cut an excursion.
static bool chain_burst(uint32_t n) {
    if (n > CMAXN) return false;
    uint32_t len = CPRE + n * CSLOTS;
    for (uint32_t i = 0; i < len + CPAD; i++) ctab[i] = NOMINAL;
    for (uint32_t i = len + CPAD; i < len + CPAD + CGUARD; i++) ctab[i] = guard_val;
    for (uint32_t j = 0; j < n; j++) ctab[CPRE + j * CSLOTS] = xtab[j] & 0xffu;
    crumb(ST_CHAIN_RUN, n);
    chain_run(ctab);
    uint32_t t0 = cyc(), lim = len * 200u + 3000000u;
    static uint32_t k;
    uintptr_t stop = (uintptr_t)&ctab[len + CPAD / 2];
    uint32_t it = 0;
    while (dma_hw->ch[CH_SY].read_addr < stop) {
        if (load_mask & 64) memcpy(c0_dst, c0_src, 512);
        if ((++it & 0xfff) == 0) crumb(ST_CHAIN_WAIT, dma_hw->ch[CH_SY].read_addr - (uint32_t)(uintptr_t)ctab);
        if (cyc() - t0 > lim) { crumb(ST_CHAIN_WAIT, 0xdead0004u); chain_tmo++; chain_stop(); return false; }
    }
    chain_stop();
    busy_wait_us(5);
    if (load_mask & 64) printf("#cdc %08lx burst traffic line padding padding padding\n", (unsigned long)k++);
    return true;
}

// Words per second the chain actually moves (Y into a RAM sink), over ~0.1 s.
static void chain_rate(void) {
    static uint32_t sink[1];
    for (uint32_t i = 0; i < CMAXN * CSLOTS; i++) ctab[i] = NOMINAL;
    int keep = cdelay;
    cdelay = -1;
    chain_run(ctab);
    dma_channel_set_write_addr(CH_SY, sink, false);
    busy_wait_us(1000);
    uint32_t a0 = dma_hw->ch[CH_SY].read_addr, t0 = cyc();
    busy_wait_us(1000);
    uint32_t a1 = dma_hw->ch[CH_SY].read_addr, t1 = cyc();
    chain_stop();
    cdelay = keep;
    double words = (a1 - a0) / 4.0, secs = (t1 - t0) / 307.2e6;
    printf("CHAINRATE,kind=%d,words=%.0f,cycles=%lu,words_per_s=%.1f,clk_peri=%lu,clk_hstx=%lu,clk_usb=%lu\n",
           xkind, words, (unsigned long)(t1 - t0), words / secs, (unsigned long)clock_get_hz(clk_peri),
           (unsigned long)clock_get_hz(clk_hstx), (unsigned long)clock_get_hz(clk_usb));
}

static uint32_t cring[8] __attribute__((aligned(32)));

// Continuous chain stream, one pulse per 8 slots (300k/s), measured with fc0.
static void cmd_chain_fc0(int sign, int gates) {
    if (xkind != 1 && xkind != 2) { printf("ERR kind\n"); return; }
    double base = fc0_mean(gates);
    for (int i = 0; i < 8; i++) cring[i] = NOMINAL;
    cring[0] = NOMINAL + sign;
    chain_run(cring);
    dma_channel_config cy = dma_get_channel_config(CH_SY);
    channel_config_set_ring(&cy, false, 5);
    dma_channel_set_config(CH_SY, &cy, false);
    sleep_ms(50);
    double on = fc0_mean(gates);
    chain_stop();
    sleep_ms(2);
    double after = fc0_mean(gates);
    double rate = 2.4e6 / 8, dhz = on - (base + after) / 2;
    printf("CFC0,kind=%d,cdelay=%d,sign=%d,gates=%d,base_hz=%.2f,on_hz=%.2f,after_hz=%.2f,pulse_hz=%.1f,vco_per_pulse=%.6f,load=%lu,fbdiv_after=%lu\n",
           xkind, cdelay, sign, gates, base, on, after, rate, dhz * 5.0 / rate, (unsigned long)load_mask,
           (unsigned long)pll_sys_hw->fbdiv_int);
}

// ---------------------------------------------------------------- 6-instruction pacer (PIO1 SM1)

static uint x6_off;
static bool x6_loaded = false;
static int x6_d = 40;

static void x6_config(int d) {
    if (!x6_loaded) {
        x6_off = pio_add_program(pio1, &xpacer6_program);
        x6_loaded = true;
    }
    x6_d = d;
    pio_sm_config c = xpacer6_program_get_default_config(x6_off);
    sm_config_set_in_pins(&c, ref_pin);
    sm_config_set_out_shift(&c, true, true, 32);
    sm_config_set_in_shift(&c, true, false, 32);
    sm_config_set_clkdiv_int_frac(&c, 1, 0);
    pio_sm_set_enabled(pio1, 1, false);
    pio_sm_init(pio1, 1, x6_off, &c);
    pio_sm_put(pio1, 1, (uint32_t)d);
    pio_sm_exec(pio1, 1, pio_encode_out(pio_x, 32));
    pio_sm_set_enabled(pio1, 1, true);
    xw_pio = pio1;
    xw_sm = 1;
    dma_channel_abort(CH_XW);
    dma_channel_config cc = dma_channel_get_default_config(CH_XW);
    channel_config_set_transfer_data_size(&cc, DMA_SIZE_32);
    channel_config_set_read_increment(&cc, false);
    channel_config_set_write_increment(&cc, false);
    channel_config_set_dreq(&cc, pio_get_dreq(pio1, 1, false));
    channel_config_set_high_priority(&cc, true);
    dma_channel_configure(CH_XW, &cc, &pll_sys_hw->fbdiv_int, &pio1_hw->rxf[1],
                          dma_encode_transfer_count(0x0fffffffu), true);
}

// One table word per 5-ref slot, CSLOTS slots per pulse; leading no-op absorbs the stalled SM.
static bool x6_burst(uint32_t n) {
    uint32_t len = 2 + n * CSLOTS + 4;
    for (uint32_t i = 0; i < len; i++) ctab[i] = NOMINAL;
    for (uint32_t j = 0; j < n; j++) ctab[2 + j * CSLOTS] = xtab[j] & 0xffu;
    uint32_t c0 = xw_count();
    crumb(ST_XBURST, 0x6000u | n);
    dma_channel_config cc = dma_channel_get_default_config(CH_XFEED);
    channel_config_set_transfer_data_size(&cc, DMA_SIZE_32);
    channel_config_set_read_increment(&cc, true);
    channel_config_set_write_increment(&cc, false);
    channel_config_set_dreq(&cc, pio_get_dreq(pio1, 1, true));
    dma_channel_configure(CH_XFEED, &cc, &pio1_hw->txf[1], ctab, len, true);
    uint32_t t0 = cyc(), lim = len * 200u + 3000000u;
    static uint32_t k;
    while (c0 - xw_count() < len) {
        if (load_mask & 64) memcpy(c0_dst, c0_src, 512);
        if (cyc() - t0 > lim) { crumb(ST_XBURST, 0xdead0006u); return false; }
    }
    if (load_mask & 64) printf("#cdc %08lx burst traffic line padding padding padding\n", (unsigned long)k++);
    return true;
}

// ---------------------------------------------------------------- byte-ring stream (firmware path)
// xpacer6b on PIO1 SM1. Feed: DMA_SIZE_8 from a power-of-two byte ring (read ring wrap,
// ENDLESS) into TX1. Writer: RX1 to FBDIV, HIGH_PRIORITY, ENDLESS. The CPU filler writes
// sigma-delta decisions ahead of the DMA read pointer; a late filler replays stale bytes.

#define BR_MAX 4096
static uint8_t bring[BR_MAX] __attribute__((aligned(BR_MAX)));
static uint32_t br_n = 4096;            // ring size in bytes (power of two)
static uint br6_off;
static bool br6_loaded = false, br_on = false;
static int br_mode = 0;                 // 0 park, 1 sigma-delta, 2 all +, 3 all -, 4 random
static int32_t br_u = 0, br_i1 = 0, br_i2 = 0;   // Q16 sigma-delta state and input
static int32_t br_dither = 0;                     // Q16 quantizer dither amplitude
static int64_t br_net = 0;              // sum of (byte - 128) over bytes the DMA has read
static uint64_t br_r = 0, br_w = 0;     // absolute read / write byte indices
static uint32_t br_ph = 0;              // br_w % br_sp
static uint32_t br_sp = 6;              // slots per decision (6: 400k decisions/s)
static uint32_t br_last_pos = 0;
static uint64_t br_last_us = 0;
static int32_t br_sum = 0;              // sum of (byte - 128) over the whole ring
static uint64_t br_pulses = 0, br_decisions = 0, br_late = 0, br_laps = 0;
static uint64_t br_fill_cyc = 0, br_fill_calls = 0, br_acc_cyc = 0;   // write part / bench accounting part
#define BR_AHEAD 64                     // bytes left between the DMA and the first write
#define BR_BEHIND 8                     // bytes kept clear behind the DMA

static int br_decide(void) {
    switch (br_mode) {
    case 1: {
        // Optional dither: a uniform offset of +-br_dither/65536 on the quantizer input.
        int32_t q = br_i2;
        if (br_dither) q += (int32_t)(xrand() % (uint32_t)(2 * br_dither + 1)) - br_dither;
        int y = q > 32768 ? 1 : q < -32768 ? -1 : 0;
        br_i1 += br_u - (y << 16);
        br_i2 += br_i1 - (y << 16);
        // Never active inside the +-1000 ppm clamp; guards against a runaway input.
        if (br_i1 > (4 << 16)) br_i1 = 4 << 16; else if (br_i1 < -(4 << 16)) br_i1 = -(4 << 16);
        if (br_i2 > (8 << 16)) br_i2 = 8 << 16; else if (br_i2 < -(8 << 16)) br_i2 = -(8 << 16);
        return y;
    }
    case 2: return 1;
    case 3: return -1;
    case 4: return (xrand() >> 7) & 1 ? 1 : -1;
    case 5: { static int a5; a5 ^= 1; return a5 ? 1 : -1; }
    case 6: {   // alternating, with every 130th decision a 0 in place of a -1 (net +1 per 130)
        static int a6, n6;
        a6 ^= 1;
        if (++n6 == 130) { n6 = 0; if (!a6) return 0; }
        return a6 ? 1 : -1;
    }
    default: return 0;
    }
}

// u in pulses per decision (Q16): ppm * 1536e6 / 5 / decision rate, clamped to 0.9 of full scale.
static void br_set_ppm(double ppm) {
    double u = ppm * 1e-6 * 1536e6 / 5.0 / (2.4e6 / br_sp);
    if (u > 0.9) u = 0.9; else if (u < -0.9) u = -0.9;
    br_u = (int32_t)(u * 65536.0 + (u >= 0 ? 0.5 : -0.5));
}

// One filler pass: account for bytes the DMA has read since the last pass, then
// write fresh bytes from BR_AHEAD past the read pointer up to a ring length ahead.
static void br_fill(void) {
    uint32_t irq = save_and_disable_interrupts();
    uint32_t c0 = cyc();
    uint64_t now = time_us_64();
    uint32_t pos = (dma_hw->ch[CH_XFEED].read_addr - (uint32_t)(uintptr_t)bring) & (br_n - 1);
    uint32_t delta = (pos - br_last_pos) & (br_n - 1);
    // Consumption is 2.4 M bytes/s from the crystal, so elapsed time fixes the lap count.
    double est = (double)(now - br_last_us) * 2.4;
    int64_t laps = (int64_t)((est - delta) / br_n + 0.5);
    if (laps < 0) laps = 0;
    if (laps) { br_net += laps * (int64_t)br_sum; br_laps += (uint64_t)laps; }
    for (uint32_t i = 0; i < delta; i++) br_net += (int)bring[(br_last_pos + i) & (br_n - 1)] - 128;
    br_r += (uint64_t)laps * br_n + delta;
    br_last_pos = pos;
    br_last_us = now;
    uint32_t c1 = cyc();
    br_acc_cyc += c1 - c0;
    uint64_t start = br_r + BR_AHEAD, end = br_r + br_n - BR_BEHIND;
    if (br_w < start) { if (br_mode) br_late++; br_w = start; br_ph = (uint32_t)(br_w % br_sp); }
    uint32_t mask = br_n - 1;
    bool first = true;
    while (br_w < end) {
        // The DMA keeps moving during a long pass (accounting included): never write
        // within BR_AHEAD of it. Checked before the first byte and every 8 bytes.
        if (first || (br_w & 7) == 0) {
            first = false;
            uint32_t pn = (dma_hw->ch[CH_XFEED].read_addr - (uint32_t)(uintptr_t)bring) & mask;
            uint64_t rn = br_r + ((pn - br_last_pos) & mask);
            if (br_w < rn + BR_AHEAD) { if (br_mode) br_late++; br_w = rn + BR_AHEAD; br_ph = (uint32_t)(br_w % br_sp); }
        }
        int v = 0;
        if (br_ph == 0) {
            v = br_decide();
            br_decisions++;
            if (v) br_pulses++;
        }
        if (++br_ph == br_sp) br_ph = 0;
        uint8_t *p = &bring[br_w & mask];
        br_sum += v - ((int)*p - 128);
        *p = (uint8_t)(128 + v);
        br_w++;
    }
    br_fill_cyc += cyc() - c1;
    br_fill_calls++;
    restore_interrupts(irq);
}

static void br_sm_init(int d) {
    if (!br6_loaded) { br6_off = pio_add_program(pio1, &xpacer6b_program); br6_loaded = true; }
    pio_sm_config c = xpacer6b_program_get_default_config(br6_off);
    sm_config_set_in_pins(&c, ref_pin);
    sm_config_set_out_shift(&c, true, true, 8);
    sm_config_set_in_shift(&c, true, false, 32);
    sm_config_set_clkdiv_int_frac(&c, 1, 0);
    pio_sm_set_enabled(pio1, 1, false);
    pio_sm_init(pio1, 1, br6_off, &c);
    pio_sm_put(pio1, 1, (uint32_t)d);
    pio_sm_exec(pio1, 1, pio_encode_out(pio_x, 32));
    pio_sm_set_enabled(pio1, 1, true);
}

// Start from parked: ring all 128, writer then feed started, filler takes over.
static void br_start(void) {
    pll_sys_hw->fbdiv_int = NOMINAL;
    for (uint32_t i = 0; i < br_n; i++) bring[i] = NOMINAL;
    br_sum = 0;
    xw_pio = pio1;
    xw_sm = 1;
    dma_channel_abort(CH_XW);
    dma_channel_config cc = dma_channel_get_default_config(CH_XW);
    channel_config_set_transfer_data_size(&cc, DMA_SIZE_32);
    channel_config_set_read_increment(&cc, false);
    channel_config_set_write_increment(&cc, false);
    channel_config_set_dreq(&cc, pio_get_dreq(pio1, 1, false));
    channel_config_set_high_priority(&cc, true);
    dma_channel_configure(CH_XW, &cc, &pll_sys_hw->fbdiv_int, &pio1_hw->rxf[1],
                          dma_encode_endless_transfer_count(), true);
    uint32_t bits = 0;
    while ((1u << bits) < br_n) bits++;
    cc = dma_channel_get_default_config(CH_XFEED);
    channel_config_set_transfer_data_size(&cc, DMA_SIZE_8);
    channel_config_set_read_increment(&cc, true);
    channel_config_set_write_increment(&cc, false);
    channel_config_set_ring(&cc, false, bits);
    channel_config_set_dreq(&cc, pio_get_dreq(pio1, 1, true));
    dma_channel_configure(CH_XFEED, &cc, &pio1_hw->txf[1], bring, dma_encode_endless_transfer_count(), true);
    br_last_pos = (dma_hw->ch[CH_XFEED].read_addr - (uint32_t)(uintptr_t)bring) & (br_n - 1);
    br_last_us = time_us_64();
    br_r = br_last_pos;
    br_w = br_r;
    br_ph = (uint32_t)(br_w % br_sp);
    br_on = true;
}

// Park and drain: only 128s are written until a full ring of them has been read,
// so the last FBDIV writes are 128. Returns false on timeout.
static bool br_park(void) {
    int keep = br_mode;
    br_mode = 0;
    br_fill();
    uint64_t target = br_w + br_n + 16;
    uint32_t t0 = cyc();
    while (br_r < target) {
        busy_wait_us(50);
        br_fill();
        if (cyc() - t0 > 307200000u) { br_mode = keep; return false; }
    }
    br_mode = keep;
    return true;
}

// Stop from parked: feed off, let the PIO drain its FIFO of 128s, then the writer off.
static void br_stop(void) {
    br_park();
    dma_channel_abort(CH_XFEED);
    busy_wait_us(20);
    dma_channel_abort(CH_XW);
    pll_sys_hw->fbdiv_int = NOMINAL;
    br_on = false;
}

static uint64_t br_cp = 0, br_cp_bad = 0, br_cp_unc = 0, br_unlock = 0;
static int64_t br_cp_sumabs = 0;
static int br_s = -1;
static int64_t br_net0 = 0;

// Checkpoint: park, measure, compare s with 5 x net of everything read, resume.
static int br_check(void) {
    br_park();
    busy_wait_us(settle_us);
    if (!(pll_sys_hw->cs & PLL_CS_LOCK_BITS)) br_unlock++;
    int s1 = meas();
    int dev = 0;
    if (br_s >= 0 && s1 >= 0) {
        bool ok;
        int64_t net = br_net - br_net0;
        dev = wrap128(dstep(br_s, s1, &ok) - (int)((dir * 5 * net) % 128));
        if (!ok) br_cp_unc++;
        if (dev) { br_cp_bad++; br_cp_sumabs += dev < 0 ? -dev : dev; }
        br_cp++;
    }
    br_s = s1;
    br_net0 = br_net;
    return dev;
}

// Soak: mode, ppm x 100 (mode 1), target pulses, checkpoint every cp decisions,
// gaps random 0..gmax us between filler passes, optional stall list in ms.
static void cmd_bsoak(int mode, int ppm100, int mpulses_k, int cp, int gmax, int stalls) {
    if (!br_on) br_start();
    br_mode = 0;
    br_set_ppm(ppm100 / 100.0);
    br_i1 = br_i2 = 0;
    br_s = -1;
    br_check();
    br_cp = br_cp_bad = br_cp_unc = br_unlock = 0;
    br_cp_sumabs = 0;
    uint64_t p0 = br_pulses, d0 = br_decisions, late0 = br_late, laps0 = br_laps;
    uint64_t ac0 = br_acc_cyc;
    uint64_t fc0c = br_fill_cyc, fcalls0 = br_fill_calls, target = (uint64_t)mpulses_k * 1000u;
    static const int stall_ms[3] = { 5, 50, 500 };
    int next_stall = 0;
    uint32_t t_start = time_us_32();
    br_mode = mode;
    uint64_t last_cp = br_decisions;
    static uint32_t k;
    while (br_pulses - p0 < target) {
        br_fill();
        uint32_t g = gmax ? xrand() % (uint32_t)gmax : 0;
        if (load_mask & 64) {
            uint32_t t = time_us_32();
            while (time_us_32() - t < g) memcpy(c0_dst, c0_src, 256);
            if ((k++ & 63) == 0) printf("#cdc %08lx filler traffic line padding padding\n", (unsigned long)k);
        } else {
            busy_wait_us(g);
        }
        // Stalls: at 1/4, 1/2 and 3/4 of the run when enabled.
        if (stalls && next_stall < 3 && br_pulses - p0 > target * (uint64_t)(next_stall + 1) / 4) {
            crumb(ST_XBURST, 0x57a1u + (uint32_t)next_stall);
            uint32_t ms = (uint32_t)stall_ms[next_stall++];
            uint32_t t = time_us_32();
            while (time_us_32() - t < ms * 1000u) { watchdog_update(); }
        }
        if (br_decisions - last_cp >= (uint64_t)cp) {
            int dev = br_check();
            if (dev && br_cp_bad <= 20) printf("BCPDEV,cp=%llu,dev=%d\n", (unsigned long long)br_cp, dev);
            br_mode = mode;
            last_cp = br_decisions;
        }
    }
    br_mode = 0;
    br_check();
    uint64_t dec = br_decisions - d0, calls = br_fill_calls - fcalls0;
    double cpd = dec ? (double)(br_fill_cyc - fc0c) / (double)dec : 0;
    printf("BS,mode=%d,ppm=%.2f,u=%.5f,ring=%lu,gmax=%d,stalls=%d,load=%lu,d=%d,pulses=%llu,decisions=%llu,checkpoints=%llu,cp_bad=%llu,cp_sumabs=%lld,uncertain=%llu,unlock=%llu,late=%llu,laps=%llu,fill_calls=%llu,cyc_per_decision=%.1f,core_pct_at_400k=%.3f,bench_accounting_cyc_per_decision=%.1f,t_s=%lu\n",
           mode, ppm100 / 100.0, br_u / 65536.0, (unsigned long)br_n, gmax, stalls, (unsigned long)load_mask, x6_d,
           (unsigned long long)(br_pulses - p0), (unsigned long long)dec, (unsigned long long)br_cp,
           (unsigned long long)br_cp_bad, (long long)br_cp_sumabs, (unsigned long long)br_cp_unc,
           (unsigned long long)br_unlock, (unsigned long long)(br_late - late0),
           (unsigned long long)(br_laps - laps0), (unsigned long long)calls, cpd, cpd * 400e3 / 307.2e6 * 100.0,
           dec ? (double)(br_acc_cyc - ac0) / (double)dec : 0.0, (unsigned long)((time_us_32() - t_start) / 1000000u));
}

// Debug: k + pulses placed directly in the ring ahead of the DMA, then park and check.
static void cmd_bk(int k) {
    if (!br_on) br_start();
    br_mode = 0;
    br_park();
    busy_wait_us(settle_us);
    int s0 = meas();
    int64_t n0 = br_net;
    uint32_t irq = save_and_disable_interrupts();
    br_fill();
    uint32_t posnow0 = (dma_hw->ch[CH_XFEED].read_addr - (uint32_t)(uintptr_t)bring) & (br_n - 1);
    uint64_t p = br_r + ((posnow0 - br_last_pos) & (br_n - 1)) + 64;
    uint32_t posnow = (dma_hw->ch[CH_XFEED].read_addr - (uint32_t)(uintptr_t)bring) & (br_n - 1);
    printf("BKPOS,pos=%lu,br_r_mod=%lu,p_mod=%lu,br_w_minus_r=%lld\n", (unsigned long)posnow,
           (unsigned long)(br_r & (br_n - 1)), (unsigned long)(p & (br_n - 1)), (long long)(br_w - br_r));
    for (int i = 0; i < k; i++, p += 6) {
        uint8_t *q = &bring[p & (br_n - 1)];
        br_sum += 1 - ((int)*q - 128);
        *q = NOMINAL + 1;
    }
    restore_interrupts(irq);
    busy_wait_us(500);
    int sm = meas();
    int64_t nm = br_net;
    br_fill();
    int64_t nm2 = br_net;
    busy_wait_us(500);
    int sm2 = meas();
    br_park();
    busy_wait_us(settle_us);
    int s1 = meas();
    bool ok;
    int ds = dstep(s0, s1, &ok);
    printf("BKMID,s0=%d,s_mid=%d,net_before_fill=%lld,net_after_fill=%lld,s_mid2=%d,s1=%d\n", s0, sm,
           (long long)(nm - n0), (long long)(nm2 - n0), sm2, s1);
    printf("BK,k=%d,net_acc=%lld,ds=%d,expect=%d,dev=%d\n", k, (long long)(br_net - n0), ds,
           (int)((5 * k) % 128), wrap128(ds - 5 * k));
}

// Debug: FBDIV readback histogram and raw RX words while all-plus bytes stream.
static void cmd_bdbg(void) {
    if (!br_on) br_start();
    br_mode = 2;
    br_fill();
    uint32_t h[4] = { 0 }, other = 0, ov = 0;
    for (int i = 0; i < 20000; i++) {
        uint32_t v = pll_sys_hw->fbdiv_int;
        if (v >= 127 && v <= 129) h[v - 127]++; else { other++; ov = v; }
        if ((i & 255) == 0) br_fill();
    }
    printf("BDBG,fbdiv127=%lu,128=%lu,129=%lu,other=%lu,last_other=0x%lx,xw_busy=%d,feed_busy=%d,pc=%lu,flevel=0x%08lx,fdebug=0x%08lx\n",
           (unsigned long)h[0], (unsigned long)h[1], (unsigned long)h[2], (unsigned long)other, (unsigned long)ov,
           dma_channel_is_busy(CH_XW), dma_channel_is_busy(CH_XFEED), (unsigned long)pio_sm_get_pc(pio1, 1),
           (unsigned long)pio1_hw->flevel, (unsigned long)pio1_hw->fdebug);
    br_mode = 0;
    br_park();
}

// Start/stop cycles: each runs n decisions of mode at ppm, then stops and checks.
static void cmd_bcycle(int cycles, int mode, int ppm100, int ndec) {
    if (br_on) br_stop();
    br_s = meas();
    br_cp = br_cp_bad = br_cp_unc = br_unlock = 0;
    br_cp_sumabs = 0;
    uint64_t nonnom = 0, p0 = br_pulses;
    for (int c = 0; c < cycles; c++) {
        br_net0 = br_net;
        br_start();
        br_mode = mode;
        br_set_ppm(ppm100 / 100.0);
        uint64_t d0 = br_decisions;
        while (br_decisions - d0 < (uint64_t)ndec) { br_fill(); busy_wait_us(xrand() % 200u); }
        br_mode = 0;
        br_stop();
        if (pll_sys_hw->fbdiv_int != NOMINAL) nonnom++;
        busy_wait_us(settle_us);
        if (!(pll_sys_hw->cs & PLL_CS_LOCK_BITS)) br_unlock++;
        int s1 = meas();
        bool ok;
        int dev = wrap128(dstep(br_s, s1, &ok) - (int)((dir * 5 * (br_net - br_net0)) % 128));
        if (dev) { br_cp_bad++; br_cp_sumabs += dev < 0 ? -dev : dev; }
        if (!ok) br_cp_unc++;
        br_cp++;
        br_s = s1;
    }
    printf("BC,cycles=%d,mode=%d,ppm=%.2f,ndec=%d,load=%lu,pulses=%llu,bad=%llu,sumabs=%lld,uncertain=%llu,unlock=%llu,fbdiv_not_128_after_stop=%llu\n",
           cycles, mode, ppm100 / 100.0, ndec, (unsigned long)load_mask, (unsigned long long)(br_pulses - p0),
           (unsigned long long)br_cp_bad, (long long)br_cp_sumabs, (unsigned long long)br_cp_unc,
           (unsigned long long)br_unlock, (unsigned long long)nonnom);
}

// ---------------------------------------------------------------- TIE recorder (core 1)
// Core 1 takes every TDC frame (600 kHz) and estimates the phase of the ref square wave's
// fundamental over the 128 residues. Stored full-rate or via a CIC5 decimating by 8.
// Output units: VCO cycles x 256.

#include <math.h>
#define REC_MAX 40000
static int32_t rec_buf[REC_MAX];
static volatile uint32_t rec_n = 0, rec_got = 0, rec_skips = 0, rec_lost = 0, rec_done = 0, rec_maxgap = 0;
static float dft_x[16][256], dft_y[16][256];   // per (word, byte) sums of cos / sin
static float rot_c[2048], rot_s[2048];          // reference angles in 1/16 residue steps

static void dft_init(void) {
    for (int wb = 0; wb < 16; wb++)
        for (int v = 0; v < 256; v++) {
            float x = 0, y = 0;
            for (int k = 0; k < 8; k++)
                if (v & (1 << k)) {
                    int j = wb * 8 + k;
                    float a = 2.0f * (float)M_PI * (float)u_of[j] / 128.0f;
                    x += cosf(a);
                    y += sinf(a);
                }
            dft_x[wb][v] = x;
            dft_y[wb][v] = y;
        }
    for (int i = 0; i < 2048; i++) {
        float a = 2.0f * (float)M_PI * (float)i / 2048.0f;
        rot_c[i] = cosf(a);
        rot_s[i] = sinf(a);
    }
}

static inline void dft_xy(const uint32_t *w, float *x, float *y) {
    float sx = 0, sy = 0;
    for (int i = 0; i < 4; i++) {
        uint32_t v = w[i];
        for (int b = 0; b < 4; b++, v >>= 8) {
            sx += dft_x[i * 4 + b][v & 255];
            sy += dft_y[i * 4 + b][v & 255];
        }
    }
    *x = sx;
    *y = sy;
}

static bool rec_frame(uint32_t *w, uint32_t *t) {
    for (int i = 0; i < 4; i++) {
        uint32_t n = 0;
        while (pio0_hw->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB + SM_TDC)))
            if (++n > 1000000u) return false;
        w[i] = pio0_hw->rxf[SM_TDC];
        if (i == 0) *t = m33_hw->dwt_cyccnt;
    }
    return true;
}

// Phase tracker state: reference in 1/16 residue (absolute, unwrapped).
static int32_t trk_ref;

static inline int32_t trk_phase(const uint32_t *w) {
    float x, y;
    dft_xy(w, &x, &y);
    int idx = trk_ref & 2047;
    float c = rot_c[idx], sn = rot_s[idx];
    float xr = x * c + y * sn, yr = y * c - x * sn;
    float d = yr / xr;                                  // tan of the angle from the reference
    float a = d - d * d * d * (1.0f / 3.0f);            // atan for |d| < 0.1
    float res = a * (128.0f / (2.0f * (float)M_PI));    // residues
    int32_t ph = trk_ref * 16 + (int32_t)lrintf(res * 256.0f);   // VCO x 256
    int32_t step = (int32_t)lrintf(res * 16.0f);
    if (step) trk_ref += step;
    return ph;
}

static void trk_seed(const uint32_t *w) {
    float x, y;
    dft_xy(w, &x, &y);
    float a = atan2f(y, x);
    if (a < 0) a += 2.0f * (float)M_PI;
    trk_ref = (int32_t)lrintf(a * 2048.0f / (2.0f * (float)M_PI));
}

static void rec_core1(int mode) {
    uint32_t w[4], t, tl = 0, n = rec_n;
    int64_t i1 = 0, i2 = 0, i3 = 0, i4 = 0, i5 = 0, z1 = 0, z2 = 0, z3 = 0, z4 = 0, z5 = 0;
    int cnt = 0;
    rec_got = rec_skips = rec_lost = rec_maxgap = 0;
    for (int k = 0; k < 2; k++) if (!rec_frame(w, &t)) { rec_done = 1; return; }
    trk_seed(w);
    if (!rec_frame(w, &tl)) { rec_done = 1; return; }
    uint32_t out = 0;
    int32_t x0 = trk_phase(w);
    while (out < n) {
        if (!rec_frame(w, &t)) break;
        uint32_t gap = t - tl;
        tl = t;
        if (gap > rec_maxgap) rec_maxgap = gap;
        if (gap > 700) rec_skips++;
        int32_t x = trk_phase(w) - x0;
        if (mode == 11) {
            rec_buf[out++] = x;
        } else {
            i1 += x; i2 += i1; i3 += i2; i4 += i3; i5 += i4;
            if (++cnt == 8) {
                cnt = 0;
                int64_t c1 = i5 - z1; z1 = i5;
                int64_t c2 = c1 - z2; z2 = c1;
                int64_t c3 = c2 - z3; z3 = c2;
                int64_t c4 = c3 - z4; z4 = c3;
                int64_t c5 = c4 - z5; z5 = c4;
                rec_buf[out++] = (int32_t)(c5 >> 15);   // VCO x 256 (gain 8^5 removed)
            }
        }
        rec_got++;
    }
    rec_done = 1;
}

// Runs one recording on core 1 while core 0 keeps the byte stream fed (if running).
// mode 10: n samples at 75 kHz (CIC), mode 11: n samples at 600 kHz.
static void rec_run(int mode, uint32_t n) {
    if (n > REC_MAX) n = REC_MAX;
    if (!core1_up) { multicore_launch_core1(core1_main); core1_up = true; }
    hammer_on = 0;
    busy_wait_us(100);
    rec_n = n;
    rec_done = 0;
    if (br_on) br_fill();
    uint64_t late0 = br_late;
    hammer_on = (uint32_t)mode;
    uint32_t t0 = time_us_32();
    static uint32_t k;
    while (!rec_done) {
        if (br_on) br_fill();
        watchdog_update();
        uint32_t t = time_us_32();
        if (load_mask & 64) {
            while (time_us_32() - t < 60) memcpy(c0_dst, c0_src, 256);
            if ((k++ & 255) == 0) printf("#cdc %08lx recorder traffic line padding\n", (unsigned long)k);
        } else {
            while (time_us_32() - t < 60) ;
        }
        if (time_us_32() - t0 > 3000000u) break;
    }
    printf("REC,mode=%d,n=%lu,got=%lu,skips=%lu,lost=%lu,maxgap=%lu,filler_late=%llu,done=%lu\n", mode,
           (unsigned long)n, (unsigned long)rec_got, (unsigned long)rec_skips, (unsigned long)rec_lost,
           (unsigned long)rec_maxgap, (unsigned long long)(br_late - late0), (unsigned long)rec_done);
}

static void rec_dump(uint32_t n, const char *tag) {
    if (n > REC_MAX) n = REC_MAX;
    int32_t prev = 0;
    for (uint32_t i = 0; i < n; i += 32) {
        printf("RD,%s,%lu", tag, (unsigned long)i);
        for (uint32_t j = i; j < i + 32 && j < n; j++) { printf(",%ld", (long)(rec_buf[j] - prev)); prev = rec_buf[j]; }
        printf("\n");
        if ((i & 1023) == 0) watchdog_update();
    }
    printf("RDEND,%s\n", tag);
}

// DFT tracker versus template decode on 700 captured frames: prints both per frame.
static void rec_validate(int print) {
    if (capture(MAXF) != 0) { printf("ERR cap\n"); return; }
    trk_seed(&cap_w[0]);
    int32_t p0 = trk_phase(&cap_w[0]);
    int s0 = decode(&cap_w[0]).s, sacc = 0, sprev = s0;
    double sum = 0, sum2 = 0, mx = 0;
    for (int f = 0; f < MAXF; f++) {
        int32_t p = trk_phase(&cap_w[4 * f]) - p0;
        int s = decode(&cap_w[4 * f]).s;
        sacc += wrap128(s - sprev);
        sprev = s;
        double d = p / 256.0 - sacc;
        sum += d; sum2 += d * d;
        if (fabs(d) > mx) mx = fabs(d);
        if (print) printf("RV,%d,%.3f,%d\n", f, p / 256.0, sacc);
    }
    double m = sum / MAXF;
    printf("RVAL,frames=%d,mean_diff_vco=%.4f,rms_diff_vco=%.4f,max_abs_diff_vco=%.3f\n", MAXF, m,
           sqrt(sum2 / MAXF - m * m), mx);
}

// ---------------------------------------------------------------- today's scheme, emulated
// Same topology and arithmetic as firmware/DSPi/soft_vcxo.c: PACE (PWM wrap DREQ) writes
// the table address to PULSE's read-address trigger; PULSE streams {exc, 128} paced by a
// DMA timer at 1/150 and chains back to PACE.

#define CH_EPULSE 13
#define CH_EPACE  14
#define EM_SLICE  1
#define EM_TIMER  0
static uint32_t em_fbdivs[2] = { NOMINAL, NOMINAL };
static uint32_t em_addr;
static bool em_on = false;

static void em_set_ppm(float ppm) {
    const float hz_per_ppm = 307.2e6f * 1e-6f * 128.0f / 150.0f;
    float mag = ppm >= 0 ? ppm : -ppm;
    if (mag < 0.115f) { em_fbdivs[0] = NOMINAL; return; }
    em_fbdivs[0] = ppm > 0 ? NOMINAL + 1 : NOMINAL - 1;
    float period = 307.2e6f / (mag * hz_per_ppm);
    uint32_t div = (uint32_t)(period / 65536.0f) + 1u;
    if (div > 255u) div = 255u;
    uint32_t wrap = (uint32_t)(period / (float)div + 0.5f);
    if (wrap > 0x10000u) wrap = 0x10000u;
    if (wrap < 4u) wrap = 4u;
    pwm_set_clkdiv_int_frac4(EM_SLICE, (uint8_t)div, 0);
    pwm_set_wrap(EM_SLICE, (uint16_t)(wrap - 1u));
    printf("EM,ppm=%.3f,div=%lu,wrap=%lu,pulse_hz=%.3f\n", ppm, (unsigned long)div, (unsigned long)wrap,
           307.2e6 / ((double)div * wrap));
}

static void em_start(float ppm) {
    em_fbdivs[0] = em_fbdivs[1] = NOMINAL;
    em_addr = (uint32_t)(uintptr_t)em_fbdivs;
    dma_timer_set_fraction(EM_TIMER, 1, 150);
    dma_channel_config c = dma_channel_get_default_config(CH_EPULSE);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, dma_get_timer_dreq(EM_TIMER));
    channel_config_set_high_priority(&c, true);
    channel_config_set_chain_to(&c, CH_EPACE);
    dma_channel_configure(CH_EPULSE, &c, &pll_sys_hw->fbdiv_int, em_fbdivs, 2, false);
    c = dma_channel_get_default_config(CH_EPACE);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pwm_get_dreq(EM_SLICE));
    channel_config_set_high_priority(&c, true);
    dma_channel_configure(CH_EPACE, &c, &dma_hw->ch[CH_EPULSE].al3_read_addr_trig, &em_addr, 1, false);
    pwm_config pc = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&pc, 255);
    pwm_config_set_wrap(&pc, 0xffff);
    pwm_init(EM_SLICE, &pc, true);
    dma_channel_start(CH_EPACE);
    em_set_ppm(ppm);
    em_on = true;
}

// Park first (pulses become 128 writes), then stop the pacer and both channels.
static void em_stop(void) {
    em_fbdivs[0] = NOMINAL;
    busy_wait_us(20000);
    pwm_set_enabled(EM_SLICE, false);
    busy_wait_us(1000);
    uint32_t c = dma_hw->ch[CH_EPULSE].al1_ctrl;
    dma_hw->ch[CH_EPULSE].al1_ctrl = (c & ~DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS) | (CH_EPULSE << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB);
    busy_wait_us(100);
    dma_channel_abort(CH_EPACE);
    dma_channel_abort(CH_EPULSE);
    pll_sys_hw->fbdiv_int = NOMINAL;
    dma_timer_set_fraction(0, 1, (uint16_t)pulse_w);
    em_on = false;
}

// One jitter case: scheme 0 parked, 1 today's, 2 shaped stream; then both recordings.
static void cmd_jit(int scheme, int ppm1000, int nlong, int nshort) {
    float ppm = ppm1000 / 1000.0f;
    if (em_on) em_stop();
    if (scheme >= 2) {
        if (!br_on) br_start();
        br_i1 = br_i2 = 0;
        br_set_ppm(ppm);
        br_mode = scheme == 2 ? 1 : scheme == 3 ? 5 : 6;
    } else if (br_on) {
        br_stop();
    }
    if (scheme == 1) em_start(ppm);
    uint32_t t = time_us_32();
    while (time_us_32() - t < 200000u) { if (br_on) br_fill(); busy_wait_us(50); watchdog_update(); }
    printf("JIT,scheme=%d,ppm=%.3f,load=%lu,dither=%ld,sp=%lu\n", scheme, ppm, (unsigned long)load_mask, (long)br_dither, (unsigned long)br_sp);
    rec_run(10, (uint32_t)nlong);
    rec_dump((uint32_t)nlong, "L");
    // The dump starves the filler for seconds: refill and settle before the next record.
    t = time_us_32();
    while (time_us_32() - t < 50000u) { if (br_on) br_fill(); busy_wait_us(50); watchdog_update(); }
    rec_run(11, (uint32_t)nshort);
    rec_dump((uint32_t)nshort, "S");
    if (scheme == 1) em_stop();
    if (scheme >= 2) { br_mode = 0; br_park(); }
}

// ---------------------------------------------------------------- write-timing probe
// Redirects the FBDIV writer of a pacer into the tstamp SM, so the PLL is untouched,
// and histograms the spacing of consecutive writes in sys cycles.

static uint ts_off;
static bool ts_loaded = false;
static uint32_t ts_buf[4096];

static void cmd_probe(int kind, int n) {
    if (n > 4096) n = 4096;
    if (!ts_loaded) {
        ts_off = pio_add_program(pio2, &tstamp_program);
        pio_sm_config c = tstamp_program_get_default_config(ts_off);
        sm_config_set_in_shift(&c, false, true, 32);
        sm_config_set_mov_status(&c, STATUS_TX_LESSTHAN, 1);
        sm_config_set_clkdiv_int_frac(&c, 1, 0);
        pio_sm_init(pio2, 2, ts_off, &c);
        pio_sm_set_enabled(pio2, 2, true);
        ts_loaded = true;
    }
    pio_sm_clear_fifos(pio2, 2);
    volatile uint32_t *dst = &pio2_hw->txf[2];
    uint32_t total = (uint32_t)n + 64;
    crumb(ST_SETUP, 0x9000u | (uint32_t)kind);
    if (kind == 0) {
        for (uint32_t i = 0; i < total / 2 + 8 && i < XMAX; i++) xtab[i] = xword(NOMINAL);
        dma_channel_set_write_addr(CH_XW, dst, false);
        xfeed_start(xtab, total / 2 + 8 < XMAX ? total / 2 + 8 : XMAX, false);
    } else {
        for (uint32_t i = 0; i < total + CPRE + CGUARD; i++) ctab[i] = NOMINAL;
        int keep = cdelay;
        cdelay = -1;
        chain_run(ctab);
        dma_channel_set_write_addr(CH_SY, dst, false);
        cdelay = keep;
    }
    uint32_t irq = save_and_disable_interrupts(), got = 0, t0 = cyc();
    while (got < (uint32_t)n && cyc() - t0 < 300000000u) {
        if (!(pio2_hw->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB + 2)))) ts_buf[got++] = pio2_hw->rxf[2];
        if ((cyc() & 0xfffff) == 0) watchdog_update();
    }
    restore_interrupts(irq);
    if (kind == 0) {
        for (uint32_t t = cyc(); dma_channel_is_busy(CH_XFEED) && cyc() - t < 300000000u; ) ;
        busy_wait_us(3000);
        xw_arm();
    } else {
        chain_stop();
    }
    // Histogram of spacing in sys cycles (3 dX + 4), skipping the FIFO fill transient.
    static uint32_t h[1100];
    uint32_t mn = 0xffffffffu, mx = 0;
    memset(h, 0, sizeof h);
    int skip = kind ? 24 : 4;
    for (uint32_t i = skip; i + 1 < got; i++) {
        uint32_t c = 3u * (ts_buf[i] - ts_buf[i + 1]) + 4u;
        if (c < mn) mn = c;
        if (c > mx) mx = c;
        h[c < 1099 ? c : 1099]++;
    }
    printf("PROBE,kind=%d,load=%lu,words=%lu,min=%lu,max=%lu,hist", kind, (unsigned long)load_mask,
           (unsigned long)got, (unsigned long)mn, (unsigned long)mx);
    for (int i = 0; i < 1100; i++) if (h[i]) printf(",%d:%lu", i, (unsigned long)h[i]);
    printf("\n");
}

// ---------------------------------------------------------------- tests

static void cmd_crumb(void) {
    printf("BOOT,watchdog=%d,magic=%d,stage=%lu,cmd=%c,pll_cs=0x%08lx,clk_sys_sel=0x%lx,fbdiv=%lu,ctx=0x%08lx\n",
           boot_wd, (boot_crumb[0] & 0xffff0000u) == CRUMB_MAGIC, (unsigned long)((boot_crumb[0] >> 8) & 0xff),
           (char)(boot_crumb[0] & 0xff) ? (char)(boot_crumb[0] & 0xff) : '-', (unsigned long)boot_crumb[1],
           (unsigned long)(boot_crumb[2] & 0xffff), (unsigned long)(boot_crumb[2] >> 16), (unsigned long)boot_crumb[3]);
    printf("CHAINSTAT,timeouts=%lu,runaway=%lu,stop_timeouts=%lu,stopmode=%d,xw_count=%lu,xw_rearms=%lu\n",
           (unsigned long)chain_tmo, (unsigned long)chain_runaway, (unsigned long)chain_stop_tmo, stopmode,
           (unsigned long)xw_count(), (unsigned long)xw_rearms);
}

static void cmd_info(void) {
    cmd_crumb();
    printf("INFO,cs=0x%08lx,fbdiv=%lu,prim=0x%08lx,sys=%lu,ref=%lu,pin=%u,dir=%d,H=%d,settle=%lu,W=%lu,load=%lu,meas_fail=%lu,timeouts=%lu\n",
           (unsigned long)pll_sys_hw->cs, (unsigned long)pll_sys_hw->fbdiv_int,
           (unsigned long)pll_sys_hw->prim, (unsigned long)clock_get_hz(clk_sys),
           (unsigned long)clock_get_hz(clk_ref), ref_pin, dir, fixH,
           (unsigned long)settle_us, (unsigned long)pulse_w, (unsigned long)load_mask,
           (unsigned long)meas_fail, (unsigned long)cap_timeouts);
}

// Raw dump of n consecutive frames, optionally right after a timer-table pulse.
static void cmd_raw(int n, int sign) {
    if (n > MAXF) n = MAXF;
    uint32_t t0 = cyc();
    if (sign) dma_channel_transfer_from_buffer_now(CH_PULSE, sign > 0 ? tab_p : tab_m, 2);
    int bad = capture(n);
    printf("RAWHDR,n=%d,sign=%d,bad=%d\n", n, sign, bad);
    for (int f = 0; f < n; f++) {
        fit_t ft = decode(&cap_w[4 * f]);
        printf("RAW,%d,%lu,%08lx,%08lx,%08lx,%08lx,%d,%d,%d,%d\n", f,
               (unsigned long)(cap_t[f] - t0), (unsigned long)cap_w[4 * f], (unsigned long)cap_w[4 * f + 1],
               (unsigned long)cap_w[4 * f + 2], (unsigned long)cap_w[4 * f + 3], ft.s, ft.H, ft.err, ft.amb);
    }
    printf("RAWEND\n");
}

// Stability: m captures of 600 frames, compared bitwise to the very first frame.
static void cmd_stab(int m) {
    uint32_t ref[4];
    uint32_t same = 0, one = 0, more = 0, total = 0, badcap = 0;
    uint32_t shist[128] = { 0 }, errhist[8] = { 0 }, ambhist[8] = { 0 };
    for (int k = 0; k < m; k++) {
        if (capture(600) != 0) { badcap++; continue; }
        if (total == 0) memcpy(ref, cap_w, sizeof ref);
        for (int f = 0; f < 600; f++) {
            int d = 0;
            for (int i = 0; i < 4; i++) d += __builtin_popcount(cap_w[4 * f + i] ^ ref[i]);
            if (d == 0) same++; else if (d == 1) one++; else more++;
            total++;
            if ((f & 15) == 0) {
                fit_t ft = decode(&cap_w[4 * f]);
                shist[ft.s & 127]++;
                errhist[ft.err > 7 ? 7 : ft.err]++;
                ambhist[ft.amb > 7 ? 7 : ft.amb]++;
            }
        }
    }
    printf("STAB,frames=%lu,identical=%lu,one_bit=%lu,more=%lu,badcap=%lu,ref=%08lx %08lx %08lx %08lx\n",
           (unsigned long)total, (unsigned long)same, (unsigned long)one, (unsigned long)more,
           (unsigned long)badcap, (unsigned long)ref[0], (unsigned long)ref[1],
           (unsigned long)ref[2], (unsigned long)ref[3]);
    printf("STABS");
    for (int i = 0; i < 128; i++) if (shist[i]) printf(",s%d=%lu", i, (unsigned long)shist[i]);
    printf("\nSTABERR");
    for (int i = 0; i < 8; i++) printf(",%lu", (unsigned long)errhist[i]);
    printf("\nSTABAMB");
    for (int i = 0; i < 8; i++) printf(",%lu", (unsigned long)ambhist[i]);
    printf("\n");
}

// Step response: raw delta-s per frame after a single timer-table pulse.
static const uint16_t a_idx[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 15, 20, 25, 30, 40, 50, 60, 80,
                                  100, 120, 150, 180, 240, 300, 360, 420, 480, 540, 600, 660, 699 };
#define NAI (sizeof a_idx / sizeof a_idx[0])

static void cmd_step(int reps, int sign, int pre_us) {
    int64_t sum[NAI] = { 0 };
    int mn[NAI], mx[NAI];
    uint32_t eqf[NAI] = { 0 }, n = 0, fin_hist[11] = { 0 }, badcap = 0;
    double tsum[NAI] = { 0 };
    for (unsigned i = 0; i < NAI; i++) { mn[i] = 999; mx[i] = -999; }
    for (int r = 0; r < reps; r++) {
        int s0 = meas();
        if (s0 < 0) { badcap++; continue; }
        s0 = sround(s0);
        uint32_t t0 = cyc();
        dma_channel_transfer_from_buffer_now(CH_PULSE, sign > 0 ? tab_p : tab_m, 2);
        if (pre_us) busy_wait_us(pre_us);
        if (capture(MAXF) != 0) { badcap++; busy_wait_us(2000); continue; }
        int fin = wrap128(decode(&cap_w[4 * (MAXF - 1)]).s - s0);
        fin_hist[(fin + 5 < 0) ? 0 : (fin + 5 > 10 ? 10 : fin + 5)]++;
        for (unsigned i = 0; i < NAI; i++) {
            int d = wrap128(decode(&cap_w[4 * a_idx[i]]).s - s0);
            sum[i] += d;
            if (d < mn[i]) mn[i] = d;
            if (d > mx[i]) mx[i] = d;
            if (d == fin) eqf[i]++;
            tsum[i] += (double)(cap_t[a_idx[i]] - t0) / 307.2;
        }
        n++;
        busy_wait_us(1000);
    }
    printf("STEP,sign=%d,pre_us=%d,n=%lu,badcap=%lu,final_hist(-5..5)", sign, pre_us,
           (unsigned long)n, (unsigned long)badcap);
    for (int i = 0; i < 11; i++) printf(",%lu", (unsigned long)fin_hist[i]);
    printf("\n");
    for (unsigned i = 0; i < NAI && n; i++)
        printf("STEPI,sign=%d,pre_us=%d,frame=%u,t_us=%.2f,mean=%.3f,min=%d,max=%d,eq_final=%lu\n",
               sign, pre_us, a_idx[i], tsum[i] / n, (double)sum[i] / n, mn[i], mx[i],
               (unsigned long)eqf[i]);
}

// Count buckets: 3,4,5,6,7, other, meas-fail.
typedef uint32_t hist7_t[7];

static void hist_add(uint32_t *h, int cnt, bool ok) {
    if (!ok) { h[6]++; return; }
    if (cnt >= 3 && cnt <= 7) h[cnt - 3]++; else h[5]++;
}

static hist7_t bh[2][128];

// Test B: timer-table single pulses in a +,-,+ cycle (net +5, so the phase
// walks through all 128 values); each pulse is binned by its measured phase.
static void cmd_B(int reps) {
    if (!dir) { printf("ERR dir\n"); return; }
    memset(bh, 0, sizeof bh);
    uint32_t t_start = time_us_32();
    int s = meas();
    // +,-,+ plus a random extra + now and then, so no deterministic orbit can
    // lock out a phase.
    int k3 = 0;
    for (uint32_t it = 0; it < 400000u; it++) {
        int sg = k3 == 1 ? -1 : 1;
        k3 = (k3 + 1) % 3;
        if (k3 == 0 && (xrand() & 3) == 0) k3 = 2;
        int s0 = s;
        bool ok;
        int ph = s0 >= 0 ? (zpace_d >= 0 ? ((5 * zpace_d - sround(s0)) & 127) : tphase(s0)) : 0;
        fire_sign(sg);
        busy_wait_us(settle_us);
        s = meas();
        int cnt = sg * dir * dstep(s0, s, &ok);
        if (s0 >= 0) hist_add(bh[sg > 0 ? 0 : 1][ph], cnt, ok);
        if (s0 >= 0 && (cnt != 5 || !ok))
            printf("BEV,it=%lu,sign=%d,phase=%d,s0=%d,cnt=%d,ok=%d,t_us=%lu\n", (unsigned long)it, sg,
                   ph, sround(s0), cnt, ok, (unsigned long)time_us_32());
        if (s < 0) s = meas();
        if ((it % 3000) == 2999) {
            uint32_t mn = 0xffffffffu;
            for (int g = 0; g < 2; g++)
                for (int p = 0; p < 128; p++) {
                    uint32_t *h = bh[g][p], t = h[0] + h[1] + h[2] + h[3] + h[4] + h[5] + h[6];
                    if (t < mn) mn = t;
                }
            printf("BPROG,it=%lu,min_per_phase=%lu,t_s=%lu\n", (unsigned long)(it + 1),
                   (unsigned long)mn, (unsigned long)((time_us_32() - t_start) / 1000000u));
            if (mn >= (uint32_t)reps) break;
        }
    }
    for (int sg = 0; sg < 2; sg++)
        for (int p = 0; p < 128; p++) {
            uint32_t *h = bh[sg][p];
            uint32_t tot = h[0] + h[1] + h[2] + h[3] + h[4] + h[5] + h[6];
            if (!tot) continue;
            printf("BT,phase=%d,sign=%c,n=%lu,c3=%lu,c4=%lu,c5=%lu,c6=%lu,c7=%lu,other=%lu,fail=%lu\n",
                   p, sg ? '-' : '+', (unsigned long)tot, (unsigned long)h[0], (unsigned long)h[1],
                   (unsigned long)h[2], (unsigned long)h[3], (unsigned long)h[4],
                   (unsigned long)h[5], (unsigned long)h[6]);
        }
    print_qh("B");
    printf("BEND,meas_fail=%lu\n", (unsigned long)meas_fail);
}

// Test C: PIO-paced single pulses of width W; phase = 5*d - s swept uniformly.
static void cmd_C(int W, int sign, int n) {
    if (!dir) { printf("ERR dir\n"); return; }
    if (sign < 0 && W < 120) { printf("ERR danger\n"); return; }
    memset(bh, 0, sizeof bh);
    int s = meas();
    int64_t csum = 0;
    uint32_t cn = 0;
    for (int i = 0; i < n; i++) {
        int target = i & 127;
        if (s < 0) { s = meas(); continue; }
        int si = sround(s);
        int d = (77 * (target + si)) & 127;     // 77 = 5^-1 mod 128
        int ph = (5 * d - si) & 127;
        fire_pacer((uint32_t)d, (uint32_t)W, NOMINAL + sign);
        busy_wait_us(settle_us);
        int s1 = meas();
        bool ok;
        int cnt = sign * dir * dstep(s, s1, &ok);
        hist_add(bh[0][ph], cnt, ok);
        if (ok) { csum += cnt; cn++; }
        s = s1;
    }
    uint32_t tot[7] = { 0 };
    for (int p = 0; p < 128; p++) {
        uint32_t *h = bh[0][p];
        for (int k = 0; k < 7; k++) tot[k] += h[k];
        printf("CT,W=%d,sign=%c,phase=%d,c3=%lu,c4=%lu,c5=%lu,c6=%lu,c7=%lu,other=%lu,fail=%lu\n",
               W, sign > 0 ? '+' : '-', p, (unsigned long)h[0], (unsigned long)h[1],
               (unsigned long)h[2], (unsigned long)h[3], (unsigned long)h[4],
               (unsigned long)h[5], (unsigned long)h[6]);
    }
    print_qh("C");
    printf("CS,W=%d,sign=%c,n=%lu,mean=%.5f,c3=%lu,c4=%lu,c5=%lu,c6=%lu,c7=%lu,other=%lu,fail=%lu,load=%lu\n",
           W, sign > 0 ? '+' : '-', (unsigned long)cn, cn ? (double)csum / cn : 0.0,
           (unsigned long)tot[0], (unsigned long)tot[1], (unsigned long)tot[2], (unsigned long)tot[3],
           (unsigned long)tot[4], (unsigned long)tot[5], (unsigned long)tot[6], (unsigned long)load_mask);
}

// Test D/E: bursts of np pulses, one per sp slots. mode 0 mixed, 1 all +, 2 all -,
// 3 none, 4 alternating +,-, 5 alternating plus two + at the end: net +10 VCO
// cancels the trigger's 10 VCO tick shift, so a parked phase stays put.
// park >= 0: before each burst, single pulses step the phase back to -s == park.
static void cmd_D(int nb, int np, int sp, int mode, int park) {
    if (!dir) { printf("ERR dir\n"); return; }
    if (np > 1000 || sp < 2 || sp > 16 || np * sp > 4000) { printf("ERR size\n"); return; }
    uint32_t len = (uint32_t)(np * sp + lead) + 1;
    for (uint32_t i = 0; i < len; i++) dtab[i] = NOMINAL;
    uint32_t dh[65] = { 0 }, dother = 0, fails = 0, unlock = 0, nz = 0, uncertain = 0;
    int64_t sumdev = 0, sumabs = 0;
    uint32_t pb_n[16] = { 0 }, pb_abs[16] = { 0 };
    int32_t pb_sum[16] = { 0 };
    uint64_t pulses = 0, plus = 0;
    uint32_t t_start = time_us_32();
    int s = meas();
    for (int b = 0; b < nb; b++) {
        if (park >= 0 && s >= 0) {
            // Step the phase toward park with single + pulses (5 per pulse).
            for (int k = 0; k < 130 && tphase(s) != park; k++) {
                fire_sign(1);
                busy_wait_us(settle_us);
                s = meas();
                if (s < 0) break;
            }
        }
        if (s < 0) { s = meas(); fails++; continue; }
        int net = 0;
        for (int j = 0; j < np; j++) {
            int sg = mode == 1 ? 1 : mode == 2 ? -1 : mode == 3 ? 0 : mode == 4 ? (j & 1 ? -1 : 1)
                   : mode == 5 ? ((j & 1) && j < np - 2 ? -1 : 1)
                   : ((xrand() >> 7) & 1 ? 1 : -1);
            dtab[lead + j * sp] = NOMINAL + sg;
            net += sg;
            if (sg > 0) plus++;
        }
        int s0 = s;
        int ph0 = tphase(s0);
        fire_tab(dtab, len);
        busy_wait_us(settle_us);
        if (!(pll_sys_hw->cs & PLL_CS_LOCK_BITS)) unlock++;
        int s1 = meas();
        pulses += np;
        if (s1 < 0) { fails++; s = meas(); continue; }
        bool ok;
        int dev = wrap128(dstep(s0, s1, &ok) - dir * 5 * net);
        if (!ok) uncertain++;
        if (dev >= -32 && dev <= 32) dh[dev + 32]++; else dother++;
        sumdev += dev;
        sumabs += dev < 0 ? -dev : dev;
        if (dev) nz++;
        int pb = ph0 >> 3;
        pb_n[pb]++;
        pb_sum[pb] += dev;
        pb_abs[pb] += dev < 0 ? -dev : dev;
        s = s1;
        if ((b % 500) == 499)
            printf("DPROG,b=%d,pulses=%llu,sumdev=%lld,sumabs=%lld,nz=%lu,fails=%lu,t_s=%lu\n", b + 1,
                   (unsigned long long)pulses, (long long)sumdev, (long long)sumabs,
                   (unsigned long)nz, (unsigned long)fails,
                   (unsigned long)((time_us_32() - t_start) / 1000000u));
    }
    print_qh("D");
    printf("DS,mode=%d,np=%d,sp=%d,park=%d,load=%lu,settle=%lu,bursts=%d,pulses=%llu,plus=%llu,sumdev=%lld,sumabs=%lld,nz=%lu,fails=%lu,unlock=%lu,other=%lu,uncertain=%lu\n",
           mode, np, sp, park, (unsigned long)load_mask, (unsigned long)settle_us, nb,
           (unsigned long long)pulses, (unsigned long long)plus, (long long)sumdev,
           (long long)sumabs, (unsigned long)nz, (unsigned long)fails, (unsigned long)unlock,
           (unsigned long)dother, (unsigned long)uncertain);
    printf("DH,mode=%d,sp=%d,load=%lu", mode, sp, (unsigned long)load_mask);
    for (int i = 0; i < 65; i++) if (dh[i]) printf(",%d:%lu", i - 32, (unsigned long)dh[i]);
    printf("\nDPB,mode=%d,sp=%d,load=%lu", mode, sp, (unsigned long)load_mask);
    for (int i = 0; i < 16; i++)
        printf(",%d:%lu/%ld/%lu", i * 8, (unsigned long)pb_n[i], (long)pb_sum[i], (unsigned long)pb_abs[i]);
    printf("\n");
}

// Per-pulse log: mode 0 alternating, 1 all +, 2 all -, 3 null (all-128 table), 4 +,-,+.
static void cmd_Z(int n, int mode) {
    int s = meas();
    for (int i = 0; i < n; i++) {
        int sg = mode == 1 ? 1 : mode == 2 ? -1 : mode == 3 ? 0 : mode == 4 ? (i % 3 == 1 ? -1 : 1)
               : (i & 1 ? -1 : 1);
        fire_sign(sg);
        busy_wait_us(settle_us);
        int s1 = meas();
        bool ok;
        int r = dstep(s, s1, &ok);
        printf("Z,%d,%d,%d,%d,%d,%d,%lu,%lu\n", i, sg, s, s1, r, ok, (unsigned long)time_us_32(),
               (unsigned long)ntrig);
        s = s1;
    }
    print_qh("Z");
}

// Timer-path count vs phase with randomized history: each trial steps s to a
// random target with PIO-paced + pulses (never miscounted), then fires one
// timer-table pulse of sign sg. Prints one line per trial.
static void cmd_rand(int n, int sg) {
    int s = meas();
    for (int i = 0; i < n; i++) {
        int target = (int)(xrand() & 127);
        for (int k = 0; k < 140 && s >= 0 && sround(s) != target; k++) {
            fire_pacer(0, 128, NOMINAL + 1);
            busy_wait_us(settle_us);
            s = meas();
        }
        if (s < 0) { s = meas(); continue; }
        int s0 = s;
        int g = sg ? sg : ((xrand() >> 9) & 1 ? 1 : -1);
        fire_sign(g);
        busy_wait_us(settle_us);
        s = meas();
        bool ok;
        int cnt = g * dir * dstep(s0, s, &ok);
        printf("RT,%d,%d,%d,%d,%d\n", i, g, s0, cnt, ok);
    }
    print_qh("RT");
}

// ---------------------------------------------------------------- main

static void handle(char *line) {
    int a = 0, b = 0, c = 0, d = 0, e = -1;
    char cmd = line[0];
    sscanf(line + 1, "%d %d %d %d %d", &a, &b, &c, &d, &e);
    crumb_cmd = (uint8_t)cmd;
    crumb(ST_CMD, 0);
    switch (cmd) {
    case 'I': cmd_info(); break;
    case 'a': keepalive = a != 0; if (!keepalive) keep_stop(); else keep_start(); printf("OK keepalive=%d\n", keepalive); break;
    case 'z': if (br_on) br_stop(); x6_d = a; br_n = (uint32_t)(b ? b : 4096); br_sm_init(a);
              printf("OK bytering d=%d ring=%lu\n", a, (unsigned long)br_n); break;
    case 'E': cmd_bsoak(a, b, c, d, e < 0 ? 300 : e, 0); break;
    case 'f': cmd_bsoak(a, b, c, d, e < 0 ? 300 : e, 1); break;
    case 'o': cmd_bk(a); break;
    case 'n': cmd_bdbg(); break;
    case '#': cmd_jit(a, b, c ? c : 37500, d ? d : 32768); break;
    case '%': if (br_on) br_stop(); br_sp = (uint32_t)(a < 2 ? 2 : a); printf("OK sp=%lu\n", (unsigned long)br_sp); break;
    case '!': br_dither = a; printf("OK dither=%ld\n", (long)br_dither); break;
    case '@': rec_validate(a); break;
    case 'u': {     // byte-ring consumption rate over ~10 ms, sampled every 50 us
        if (!br_on) br_start();
        uint32_t last = dma_hw->ch[CH_XFEED].read_addr, tot = 0, t0 = time_us_32(), mx = 0;
        for (int i = 0; i < 200; i++) {
            busy_wait_us(50);
            uint32_t a2 = dma_hw->ch[CH_XFEED].read_addr, dlt = (a2 - last) & (br_n - 1);
            if (dlt > mx) mx = dlt;
            tot += dlt;
            last = a2;
        }
        uint32_t el = time_us_32() - t0;
        printf("BRATE,bytes=%lu,us=%lu,bytes_per_us=%.4f,max_step=%lu,pc=%lu,fdebug=0x%08lx\n", (unsigned long)tot,
               (unsigned long)el, (double)tot / el, (unsigned long)mx, (unsigned long)pio_sm_get_pc(pio1, 1),
               (unsigned long)pio1_hw->fdebug);
        br_fill();
        break;
    }
    case 'd': {     // raw: k + bytes into a running 128 stream, no filler, wait, measure
        if (!br_on) br_start();
        br_mode = 0;
        br_park();
        busy_wait_us(200);
        int s0 = meas();
        uint32_t pos = (dma_hw->ch[CH_XFEED].read_addr - (uint32_t)(uintptr_t)bring) & (br_n - 1);
        uint32_t base = (pos + 64) & (br_n - 1), h129 = 0, n = 0;
        for (int i = 0; i < a; i++) bring[(base + 6 * (uint32_t)i) & (br_n - 1)] = NOMINAL + 1;
        uint32_t t = time_us_32();
        while (time_us_32() - t < (uint32_t)(b ? b : 200)) { n++; if (pll_sys_hw->fbdiv_int == 129) h129++; }
        for (int i = 0; i < a; i++) bring[(base + 6 * (uint32_t)i) & (br_n - 1)] = NOMINAL;
        busy_wait_us(3000);
        int s1 = meas();
        bool ok;
        printf("BRAW,k=%d,s0=%d,s1=%d,ds=%d,expect=%d,reads129=%lu/%lu\n", a, s0, s1, dstep(s0, s1, &ok),
               wrap128(5 * a), (unsigned long)h129, (unsigned long)n);
        br_fill();
        break;
    }
    case 't': {     // fc0 on a byte-ring stream left to replay (mode a): VCO cycles per pulse
        if (!br_on) br_start();
        double base = fc0_mean(20);
        br_mode = a;
        br_fill();
        busy_wait_us(3000);
        br_fill();
        int32_t sum = br_sum;
        double on = fc0_mean(20);
        br_mode = 0;
        br_park();
        double rate = 2.4e6 * (double)sum / (double)br_n;
        printf("BFC0,mode=%d,ring_sum=%ld,pulse_rate=%.1f,base=%.2f,on=%.2f,vco_per_pulse=%.5f\n", a, (long)sum, rate,
               base, on, rate ? (on - base) * 5.0 / rate : 0.0);
        break;
    }
    case 's': printf("CLK,sys_ctrl=0x%08lx,sys_sel=0x%lx,sys_div=0x%lx,ref_ctrl=0x%08lx,ref_sel=0x%lx,pll_cs=0x%08lx,pll_pwr=0x%08lx,pll_fb=%lu,pll_prim=0x%08lx,xosc_status=0x%08lx\n",
                     (unsigned long)clocks_hw->clk[clk_sys].ctrl, (unsigned long)clocks_hw->clk[clk_sys].selected,
                     (unsigned long)clocks_hw->clk[clk_sys].div, (unsigned long)clocks_hw->clk[clk_ref].ctrl,
                     (unsigned long)clocks_hw->clk[clk_ref].selected, (unsigned long)pll_sys_hw->cs,
                     (unsigned long)pll_sys_hw->pwr, (unsigned long)pll_sys_hw->fbdiv_int,
                     (unsigned long)pll_sys_hw->prim, (unsigned long)xosc_hw->status); break;
    case 'l': cmd_bcycle(a, b, c, d ? d : 2000); break;
    case 'y': xkind = 3; x6_config(a); printf("OK kind=3 x6_d=%d\n", x6_d); break;
    case 'p': cmd_probe(a, b ? b : 2000); break;
    case 'v': xverbose = a; printf("OK verbose=%d\n", xverbose); break;
    case 'm': cmd_chain_fc0(a, b ? b : 60); break;
    case 'e': guard_val = (uint32_t)a; printf("OK guard=%lu\n", (unsigned long)guard_val); break;
    case 'j': stopmode = a; printf("OK stopmode=%d\n", stopmode); break;
    case 'R': cmd_raw(a ? a : 16, b); break;
    case 'S': cmd_stab(a ? a : 10); break;
    case 'A': cmd_step(a ? a : 50, b ? b : 1, c); break;
    case 'K': dir = a; printf("OK dir=%d\n", dir); break;
    case 'H': fixH = a; printf("OK H=%d\n", fixH); break;
    case 'W': settle_us = (uint32_t)a; printf("OK settle=%lu\n", (unsigned long)settle_us); break;
    case 'T': pulse_w = (uint32_t)a; dma_timer_set_fraction(0, 1, (uint16_t)a); printf("OK W=%d\n", a); break;
    case 'P': {
        int s0 = meas();
        const uint32_t *t = a > 0 ? tab_p : tab_m;
        if (!tab_safe(t, 2)) { printf("ERR danger\n"); break; }
        fire_sign(a > 0 ? 1 : -1);
        busy_wait_us(settle_us);
        int s1 = meas();
        bool ok;
        int r = dstep(s0, s1, &ok);
        printf("PULSE,sign=%d,s0=%.3f,s1=%.3f,ds=%d,ok=%d\n", a, s0 / 1024.0, s1 / 1024.0, r, ok);
        break;
    }
    case 'Q': {     // pacer single pulse: d W sign
        int s0 = meas();
        fire_pacer((uint32_t)a, (uint32_t)b, NOMINAL + (c > 0 ? 1 : -1));
        busy_wait_us(settle_us);
        int s1 = meas();
        bool ok;
        int r = dstep(s0, s1, &ok);
        printf("PACE,d=%d,W=%d,sign=%d,s0=%.3f,s1=%.3f,ds=%d,ok=%d\n", a, b, c, s0 / 1024.0, s1 / 1024.0, r, ok);
        break;
    }
    case 'M': printf("MEAS,s=%.3f\n", meas() / 1024.0); break;
    case 'N': meas_n = (uint32_t)(a < 1 ? 1 : a > MAXF ? MAXF : a); printf("OK n=%lu\n", (unsigned long)meas_n); break;
    case 'B': cmd_B(a ? a : 200); break;
    case 'C': cmd_C(a, b, c); break;
    case 'D': cmd_D(a, b, c, d, e); break;
    case 'Z': cmd_Z(a, b); break;
    case 'Y': trig_d = a; printf("OK trig_d=%d\n", trig_d); break;
    case 'V': {     // count vs trigger offset: V reps d_lo d_hi d_step
        for (int dd = b; dd <= c; dd += (d ? d : 1)) {
            trig_d = dd;
            uint32_t h[2][7] = { { 0 } };
            int s = meas();
            for (int r = 0; r < 3 * a; r++) {
                int sg = (r % 3 == 1) ? -1 : 1;
                int s0 = s;
                bool ok;
                fire_sign(sg);
                busy_wait_us(settle_us);
                s = meas();
                hist_add(h[sg > 0 ? 0 : 1], sg * dir * dstep(s0, s, &ok), ok);
                if (s < 0) s = meas();
            }
            printf("VT,d=%d,p4=%lu,p5=%lu,p6=%lu,po=%lu,m4=%lu,m5=%lu,m6=%lu,mo=%lu,phase_end=%d\n", dd,
                   (unsigned long)h[0][1], (unsigned long)h[0][2], (unsigned long)h[0][3],
                   (unsigned long)(h[0][0] + h[0][4] + h[0][5] + h[0][6]),
                   (unsigned long)h[1][1], (unsigned long)h[1][2], (unsigned long)h[1][3],
                   (unsigned long)(h[1][0] + h[1][4] + h[1][5] + h[1][6]), (128 - sround(s)) & 127);
        }
        trig_d = -1;
        break;
    }
    case 'w': {     // tick offset vs TDC frame: w n fires(0/1) ; CH7 timer-paced probe
        static uint32_t u_src, u_dst;
        dma_channel_config cc = dma_channel_get_default_config(7);
        channel_config_set_transfer_data_size(&cc, DMA_SIZE_32);
        channel_config_set_read_increment(&cc, false);
        channel_config_set_write_increment(&cc, false);
        channel_config_set_dreq(&cc, dma_get_timer_dreq(0));
        dma_channel_configure(7, &cc, &u_dst, &u_src, 1, false);
        for (int i = 0; i < a; i++) {
            uint32_t irq = save_and_disable_interrupts(), w;
            for (int k = 0; k < 8; k++) rx_word(&w);
            uint32_t t0 = cyc();
            dma_channel_set_trans_count(7, 1, true);
            while (dma_channel_is_busy(7)) ;
            uint32_t t1 = cyc();
            restore_interrupts(irq);
            printf("TICK,%d,%lu\n", i, (unsigned long)((t1 - t0) % 128u));
            if (b) { fire_sign(0); busy_wait_us(50); }
        }
        break;
    }
    case 'U': {     // DMA timer 0 period: n paced RAM-to-RAM transfers timed by CYCCNT
        static uint32_t u_src, u_dst;
        uint32_t n = a ? (uint32_t)a : 1000000u;
        dma_channel_config cc = dma_channel_get_default_config(7);
        channel_config_set_transfer_data_size(&cc, DMA_SIZE_32);
        channel_config_set_read_increment(&cc, false);
        channel_config_set_write_increment(&cc, false);
        channel_config_set_dreq(&cc, dma_get_timer_dreq(0));
        dma_channel_configure(7, &cc, &u_dst, &u_src, 1, true);
        while (dma_channel_is_busy(7)) ;
        uint32_t irq = save_and_disable_interrupts();
        dma_channel_set_trans_count(7, 1, true);
        while (dma_channel_is_busy(7)) ;
        uint32_t t0 = cyc();
        dma_channel_set_trans_count(7, n, true);
        while (dma_channel_is_busy(7)) ;
        uint32_t t1 = cyc();
        restore_interrupts(irq);
        printf("TIMERPERIOD,n=%lu,cycles=%lu,period=%.6f\n", (unsigned long)n, (unsigned long)(t1 - t0),
               (double)(t1 - t0) / n);
        break;
    }
    case 'r': cmd_rand(a, b); break;
    case 'x': xp_config(a, b ? b : xp_n, c); printf("OK xd=%d xN=%d edge=%d\n", xp_d, xp_n, xp_edge); break;
    case 'g': cmd_X(a, b, c, d); break;
    case 'h': cmd_xfc0(a, b ? b : 60); break;
    case 'i': sd_i1 = sd_i2 = 0; printf("OK sd reset\n"); break;
    case 'c': chain_setup(a, b, (c & 2) != 0); if (c & 1) chain_rate(); printf("OK kind=%d cdelay=%d\n", xkind, cdelay); break;
    case 'b': printf("REBOOT\n"); sleep_ms(100); watchdog_reboot(0, 0, 10); break;
    case 'O': zpace_d = a; printf("OK zpace_d=%d\n", zpace_d); break;
    case 'J': lead = a < 0 ? 0 : a > 6 ? 6 : a; printf("OK lead=%d\n", lead); break;
    case 'k': tab_scratch = a != 0; printf("OK tab_scratch=%d addr=%p\n", tab_scratch, (void *)t_scr); break;
    case 'q': quietwait = a != 0; printf("OK quietwait=%d\n", quietwait); break;
    case 'X': noirq = a != 0; printf("OK noirq=%d\n", noirq); break;
    case 'L': set_load((uint32_t)a); printf("OK load=%d\n", a); break;
    case 'F': cmd_fc0(a, b, c ? c : 60); break;
    case 'G':
        gpio_set_function(ref_pin, GPIO_FUNC_NULL);
        ref_pin = (uint)a;
        ref_pin_setup(ref_pin);
        pio_sm_set_in_pins(pio0, SM_TDC, ref_pin);
        printf("OK pin=%u\n", ref_pin);
        break;
    default: printf("ERR cmd\n"); break;
    }
    printf("DONE\n");
}

int main(void) {
    boot_wd = watchdog_caused_reboot();
    for (int i = 0; i < 4; i++) boot_crumb[i] = watchdog_hw->scratch[i];
    for (int i = 0; i < 4; i++) watchdog_hw->scratch[i] = 0;
    vreg_set_voltage(VREG_VOLTAGE_1_15);
    busy_wait_ms(10);
    set_sys_clock_hz(307200000, false);
    stdio_init_all();
    m33_hw->demcr |= M33_DEMCR_TRCENA_BITS;
    m33_hw->dwt_ctrl |= M33_DWT_CTRL_CYCCNTENA_BITS;
    ref_pin_setup(ref_pin);
    pio_setup();
    pulse_dma_setup();
    xp_setup();
    dft_init();
    for (int i = 0; i < 1024; i++) mc_src[i] = i;
    watchdog_enable(4000, true);

    char line[96];
    int n = 0;
    while (true) {
        crumb(ST_IDLE, 0);
        int ch = getchar_timeout_us(1000000);
        if (ch == PICO_ERROR_TIMEOUT) { if (n == 0) printf("READY\n"); continue; }
        if (ch == '\r' || ch == '\n') {
            line[n] = 0;
            if (n) handle(line);
            n = 0;
        } else if (n < (int)sizeof line - 1) {
            line[n++] = (char)ch;
        }
    }
}
