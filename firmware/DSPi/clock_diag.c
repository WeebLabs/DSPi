/*
 * clock_diag.c; bench clock / audio-path diagnostics (see clock_diag.h).
 *
 * Entirely gated by DSPI_CLOCK_DIAG.  Everything here runs from the main loop
 * (never an IRQ), so the translation unit lives in flash; the only IRQ-context
 * cost is the TX libraries' pre-existing per-instance starvation increment,
 * which this module just enables and reads.
 */

#include "clock_diag.h"

#if DSPI_CLOCK_DIAG

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "hardware/clocks.h"
#include "hardware/structs/clocks.h"
#include "hardware/dma.h"
#include "hardware/structs/dma_debug.h"
#include "hardware/pwm.h"
#include "hardware/pll.h"
#include "hardware/regs/pll.h"
#include "hardware/timer.h"

#include "config.h"
#include "soft_vcxo.h"
#include "input_servo.h"
#include "spdif_input.h"
#include "audio_pipeline.h"
#include "audio_input.h"
#include "usb_audio.h"
#include "pdm_generator.h"
#include "pico/audio_spdif.h"
#include "pico/audio_i2s_multi.h"

// Shared globals owned elsewhere.
extern uint8_t output_types[];
extern audio_spdif_instance_t *spdif_instance_ptrs[];
extern audio_i2s_instance_t   *i2s_instance_ptrs[];
extern volatile bool output_type_switch_in_progress;

#define VCXO_SYS_NOM_HZ 307200000u

// ---------------------------------------------------------------------------
// Windowed aggregates (reset each dump)
// ---------------------------------------------------------------------------

// fc0 measured sys_clk (non-blocking state machine).
static uint8_t  fc_state = 0;         // 0 = kick pending, 1 = measurement running
static uint32_t fc_last_hz = 0;
static uint32_t fc_min_hz = 0;
static uint32_t fc_max_hz = 0;
static uint32_t fc_count = 0;

// Main-loop gap.
static uint32_t loop_prev_us = 0;
static uint32_t loop_gap_max_us = 0;

// Per-slot min consumer fill and last-dump ring consumed-words totals.
static uint8_t  slot_min_fill[NUM_SPDIF_INSTANCES];
static uint32_t slot_prev_words[NUM_SPDIF_INSTANCES];

// PDM min fill.
static uint8_t  pdm_min_fill = 100;

// PWM pacer counter movement evidence.
static uint16_t pwm_prev_ctr = 0;
static uint32_t pwm_moves = 0;

static uint32_t last_dump_us = 0;

// Latest once-per-second snapshot, served read-only by REQ_GET_CLOCK_DIAG.
static ClockDiagPacket g_snapshot;

// ppm -> centi-ppm (ppm*100) with symmetric rounding.
static inline int32_t ppm_to_cppm(float v) {
    return (int32_t)(v * 100.0f + (v >= 0.0f ? 0.5f : -0.5f));
}

// Hz -> milli-Hz (Hz*1000), clamped non-negative (rates fit uint32 to 4.29 MHz).
static inline uint32_t hz_to_mhz(float v) {
    if (v < 0.0f) v = 0.0f;
    return (uint32_t)(v * 1000.0f + 0.5f);
}

// ---------------------------------------------------------------------------
// Float formatting (no printf %f in this build; UART stdio, minimal libc)
// ---------------------------------------------------------------------------

// Signed ppm with two decimals, e.g. "-12.34".
static void fmt_ppm(char *s, float v) {
    long x = (long)(v * 100.0f + (v >= 0.0f ? 0.5f : -0.5f));
    long a = (x < 0) ? -x : x;
    snprintf(s, 12, "%s%ld.%02ld", (x < 0) ? "-" : "", a / 100, a % 100);
}

// Positive-leaning Hz with three decimals, e.g. "48000.050".
static void fmt_hz(char *s, float v) {
    long w = (long)v;
    long m = (long)((v - (float)w) * 1000.0f + (v >= 0.0f ? 0.5f : -0.5f));
    if (m < 0) m = -m;
    if (m >= 1000) { w += 1; m -= 1000; }
    snprintf(s, 20, "%ld.%03ld", w, m);
}

// sys_clk offset in ppm*100 vs the 307.2 MHz nominal, as an integer.
static long hz_to_ppm100(uint32_t hz) {
    long diff = (long)hz - (long)VCXO_SYS_NOM_HZ;
    return diff * 1000 / 3072;   // ppm*100 = diff / 307.2 * 100
}

// ---------------------------------------------------------------------------
// fc0 frequency-counter service (non-blocking)
// ---------------------------------------------------------------------------
// fc0 is otherwise unused in the tree (only the SDK's frequency_count_khz and
// rosc calibration touch it, neither of which this firmware invokes), so we
// drive the registers directly rather than busy-wait like the old bench code.
static void fc_service(void) {
    if (fc_state == 0) {
        if (clocks_hw->fc0.status & CLOCKS_FC0_STATUS_RUNNING_BITS) return;
        clocks_hw->fc0.ref_khz  = clock_get_hz(clk_ref) / 1000u;
        clocks_hw->fc0.interval = 15;   // gate = 2^15 us ~= 32.8 ms
        clocks_hw->fc0.min_khz  = 0;
        clocks_hw->fc0.max_khz  = 0xffffffffu;
        clocks_hw->fc0.src      = CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY;
        fc_state = 1;
        return;
    }
    if (!(clocks_hw->fc0.status & CLOCKS_FC0_STATUS_DONE_BITS)) return;
    // result[24:5] = kHz, result[4:0] = 1/32 kHz fraction (31.25 Hz LSB).
    uint32_t result = clocks_hw->fc0.result;
    uint32_t hz = (uint32_t)(((uint64_t)result * 1000ull) >> 5);
    fc_last_hz = hz;
    if (fc_count == 0 || hz < fc_min_hz) fc_min_hz = hz;
    if (fc_count == 0 || hz > fc_max_hz) fc_max_hz = hz;
    fc_count++;
    clocks_hw->fc0.src = 0;   // CLOCKS_FC0_SRC_VALUE_NULL; re-kick next pass
    fc_state = 0;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void clock_diag_init(void) {
    // Enable the TX libraries' per-instance consumer-starvation monitors; the
    // increment already exists in each DMA IRQ path (a silence buffer is
    // substituted when the consumer pool is empty), gated on these flags.
    audio_spdif_set_starvation_monitoring(true);
    audio_spdif_reset_dma_starvations();
    audio_i2s_set_starvation_monitoring(true);
    audio_i2s_reset_dma_starvations();

    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        slot_min_fill[i] = SPDIF_CONSUMER_BUFFER_COUNT;
        slot_prev_words[i] = 0;
    }
    loop_prev_us = time_us_32();
    last_dump_us = loop_prev_us;
    pwm_prev_ctr = pwm_hw->slice[SOFT_VCXO_PWM_SLICE].ctr;
}

void clock_diag_sample(void) {
    fc_service();

    // Main-loop gap.
    uint32_t now = time_us_32();
    uint32_t gap = now - loop_prev_us;
    loop_prev_us = now;
    if (gap > loop_gap_max_us) loop_gap_max_us = gap;

    // Per-slot min consumer fill (skip the type-switch window; the pool
    // pointers are being mutated and the fill read is not meaningful).
    if (!output_type_switch_in_progress) {
        for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
            uint f = get_slot_consumer_fill((uint)i);
            if (f < slot_min_fill[i]) slot_min_fill[i] = (uint8_t)f;
        }
        if (pdm_enabled) {
            uint8_t rf = pdm_get_ring_fill_pct();
            if (rf < pdm_min_fill) pdm_min_fill = rf;
        }
    }

    // PWM-pacer counter movement (H5 loop-alive evidence, sampled ms apart).
    uint16_t ctr = pwm_hw->slice[SOFT_VCXO_PWM_SLICE].ctr;
    if (ctr != pwm_prev_ctr) { pwm_moves++; pwm_prev_ctr = ctr; }
}

void clock_diag_poll(void) {
    uint32_t now = time_us_32();
    if (now - last_dump_us < 1000000u) return;
    last_dump_us = now;

    char a[20], b[20], c[20], d[20];

    // Snapshot header (served by REQ_GET_CLOCK_DIAG).
    g_snapshot.version   = 1;
#if PICO_RP2350
    g_snapshot.platform  = PLATFORM_RP2350;
#else
    g_snapshot.platform  = PLATFORM_RP2040;
#endif
    g_snapshot.num_slots = NUM_SPDIF_INSTANCES;

    // --- sysclk: fc0-measured sys_clk and its ppm excursion, PLL lock ---
    {
        long p_last = hz_to_ppm100(fc_last_hz);
        long p_min  = hz_to_ppm100(fc_min_hz);
        long p_max  = hz_to_ppm100(fc_max_hz);
        uint32_t lock = (pll_sys_hw->cs & PLL_CS_LOCK_BITS) ? 1u : 0u;
        printf("CLKDIAG sysclk: hz=%lu min=%lu max=%lu ppm=%ld.%02ld[%ld.%02ld..%ld.%02ld] n=%lu lock=%lu\n",
               (unsigned long)fc_last_hz, (unsigned long)fc_min_hz, (unsigned long)fc_max_hz,
               p_last / 100, labs(p_last) % 100,
               p_min / 100, labs(p_min) % 100, p_max / 100, labs(p_max) % 100,
               (unsigned long)fc_count, (unsigned long)lock);

        g_snapshot.pll_lock       = (uint8_t)lock;
        g_snapshot.sysclk_last_hz = fc_last_hz;
        g_snapshot.sysclk_min_hz  = fc_min_hz;
        g_snapshot.sysclk_max_hz  = fc_max_hz;
        g_snapshot.sysclk_count   = fc_count;
    }

    // --- vcxo: command path + DMA/PWM loop-alive evidence ---
    {
        SoftVcxoDiag v;
        soft_vcxo_get_diag(&v);
        fmt_ppm(a, v.last_ppm); fmt_ppm(b, v.min_ppm); fmt_ppm(c, v.max_ppm);

        const uint pulse = SOFT_VCXO_DMA_PULSE;
        const uint pace  = SOFT_VCXO_DMA_PACE;
        uint32_t pulse_ctrl = dma_hw->ch[pulse].al1_ctrl;
        uint32_t pace_ctrl  = dma_hw->ch[pace].al1_ctrl;

        printf("CLKDIAG vcxo: ppm=%s[%s..%s] park=%u fbdiv0=%u pulseHz=%lu div=%u wrap=%u set=%lu clamp=%lu sflip=%lu fbdivLive=%lu cs=%lu\n",
               a, b, c, v.parked ? 1u : 0u, (unsigned)v.fbdiv0,
               (unsigned long)v.pulse_hz, (unsigned)v.pwm_div, (unsigned)v.pwm_wrap,
               (unsigned long)v.set_calls, (unsigned long)v.clamp_sats,
               (unsigned long)v.sign_changes,
               (unsigned long)(pll_sys_hw->fbdiv_int & 0xffffu),
               (unsigned long)((pll_sys_hw->cs & PLL_CS_LOCK_BITS) ? 1u : 0u));

        printf("CLKDIAG vcxoloop: PULSE busy=%lu en=%lu tcr=%lu rd=0x%08lx | PACE busy=%lu en=%lu tcr=%lu ctdreq=%lu | pwmctr=%u moves=%lu\n",
               (unsigned long)((pulse_ctrl & DMA_CH0_CTRL_TRIG_BUSY_BITS) ? 1u : 0u),
               (unsigned long)((pulse_ctrl & DMA_CH0_CTRL_TRIG_EN_BITS) ? 1u : 0u),
               (unsigned long)dma_debug_hw->ch[pulse].dbg_tcr,
               (unsigned long)dma_hw->ch[pulse].read_addr,
               (unsigned long)((pace_ctrl & DMA_CH0_CTRL_TRIG_BUSY_BITS) ? 1u : 0u),
               (unsigned long)((pace_ctrl & DMA_CH0_CTRL_TRIG_EN_BITS) ? 1u : 0u),
               (unsigned long)dma_debug_hw->ch[pace].dbg_tcr,
               (unsigned long)dma_debug_hw->ch[pace].dbg_ctdreq,
               (unsigned)pwm_hw->slice[SOFT_VCXO_PWM_SLICE].ctr,
               (unsigned long)pwm_moves);

        g_snapshot.vcxo_last_cppm    = ppm_to_cppm(v.last_ppm);
        g_snapshot.vcxo_min_cppm     = ppm_to_cppm(v.min_ppm);
        g_snapshot.vcxo_max_cppm     = ppm_to_cppm(v.max_ppm);
        g_snapshot.vcxo_pulse_hz     = v.pulse_hz;
        g_snapshot.vcxo_pwm_wrap     = v.pwm_wrap;
        g_snapshot.vcxo_pwm_div      = v.pwm_div;
        g_snapshot.vcxo_fbdiv_live   = (uint16_t)(pll_sys_hw->fbdiv_int & 0xffffu);
        g_snapshot.vcxo_set_calls    = v.set_calls;
        g_snapshot.vcxo_clamp_sats   = v.clamp_sats;
        g_snapshot.vcxo_sign_changes = v.sign_changes;
        g_snapshot.vcxo_fbdiv0       = v.fbdiv0;
        g_snapshot.vcxo_parked       = v.parked ? 1u : 0u;

        g_snapshot.loop_pulse_busy = (pulse_ctrl & DMA_CH0_CTRL_TRIG_BUSY_BITS) ? 1u : 0u;
        g_snapshot.loop_pulse_en   = (pulse_ctrl & DMA_CH0_CTRL_TRIG_EN_BITS) ? 1u : 0u;
        g_snapshot.loop_pace_busy  = (pace_ctrl & DMA_CH0_CTRL_TRIG_BUSY_BITS) ? 1u : 0u;
        g_snapshot.loop_pace_en    = (pace_ctrl & DMA_CH0_CTRL_TRIG_EN_BITS) ? 1u : 0u;
        g_snapshot.loop_pulse_tcr  = dma_debug_hw->ch[pulse].dbg_tcr;
        g_snapshot.loop_pace_tcr   = dma_debug_hw->ch[pace].dbg_tcr;
        g_snapshot.loop_pace_ctdreq = dma_debug_hw->ch[pace].dbg_ctdreq;
        g_snapshot.loop_pwm_moves  = pwm_moves;
        g_snapshot.loop_pwm_ctr    = (uint16_t)pwm_hw->slice[SOFT_VCXO_PWM_SLICE].ctr;
    }

    // --- servo: input_servo term breakdown ---
    {
        InputServoDiag s;
        input_servo_get_diag(&s);
        fmt_hz(a, s.actual_freq);
        fmt_ppm(b, s.ppm_ff); fmt_ppm(c, s.ppm_total); fmt_ppm(d, s.integral_ppm);
        char lo[12], hi[12];
        fmt_ppm(lo, s.ppm_min); fmt_ppm(hi, s.ppm_max);
        printf("CLKDIAG servo: in=%s ff=%s tot=%s[%s..%s] int=%s ferr=%ld slot=%ld ticks=%lu\n",
               a, b, c, lo, hi, d, (long)s.fill_error, (long)s.fill_slot,
               (unsigned long)s.ticks);

        g_snapshot.servo_actual_mhz = hz_to_mhz(s.actual_freq);
        g_snapshot.servo_ff_cppm    = ppm_to_cppm(s.ppm_ff);
        g_snapshot.servo_total_cppm = ppm_to_cppm(s.ppm_total);
        g_snapshot.servo_min_cppm   = ppm_to_cppm(s.ppm_min);
        g_snapshot.servo_max_cppm   = ppm_to_cppm(s.ppm_max);
        g_snapshot.servo_int_cppm   = ppm_to_cppm(s.integral_ppm);
        g_snapshot.servo_fill_error = s.fill_error;
        g_snapshot.servo_fill_slot  = s.fill_slot;
        g_snapshot.servo_ticks      = s.ticks;
    }

    // --- slots: per-output fill/min, starvation, DMA throughput ---
    {
        char line[256];
        int off = snprintf(line, sizeof(line), "CLKDIAG slots:");
        for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
            uint fill = output_type_switch_in_progress ? 0 : get_slot_consumer_fill((uint)i);
            uint32_t starv, words;
            const char *ty;
            // Ring path: consumed words = unwrapped DMA read pointer; starv
            // counts FRAMES of silence exposed (unit change from buffers).
            if (output_types[i] == OUTPUT_TYPE_I2S) {
                ty = "i2s";
                starv = audio_i2s_get_dma_starvations_instance((uint)i);
                audio_i2s_instance_t *inst = i2s_instance_ptrs[i];
                words = (inst && inst->ring) ? audio_i2s_ring_consumed_words(inst) : 0;
            } else {
                ty = "spd";
                starv = audio_spdif_get_dma_starvations_instance((uint)i);
                audio_spdif_instance_t *inst = spdif_instance_ptrs[i];
                words = (inst && inst->ring) ? audio_spdif_ring_consumed_words(inst) : 0;
            }
            uint32_t dwc = words - slot_prev_words[i];
            slot_prev_words[i] = words;

            g_snapshot.slot_fill[i]     = (uint8_t)fill;
            g_snapshot.slot_min_fill[i] = slot_min_fill[i];
            g_snapshot.slot_is_i2s[i]   = (output_types[i] == OUTPUT_TYPE_I2S) ? 1u : 0u;
            g_snapshot.slot_starv[i]    = starv;
            g_snapshot.slot_dwc[i]      = dwc;

            off += snprintf(line + off, sizeof(line) - off,
                            " s%d=%s %u/min%u starv=%lu dwc=%lu", i, ty,
                            fill, (unsigned)slot_min_fill[i],
                            (unsigned long)starv, (unsigned long)dwc);
            if (off >= (int)sizeof(line) - 1) break;
        }
        printf("%s\n", line);
    }

    // --- pdm: fill and ring/dma starvation (silence-insert underruns) ---
    g_snapshot.pdm_enabled = pdm_enabled ? 1u : 0u;
    if (pdm_enabled) {
        uint8_t rf = pdm_get_ring_fill_pct();
        uint8_t df = pdm_get_dma_fill_pct();
        printf("CLKDIAG pdm: ringfill=%u/min%u dmafill=%u ringur=%lu dmaur=%lu\n",
               (unsigned)rf, (unsigned)pdm_min_fill, (unsigned)df,
               (unsigned long)pdm_ring_underruns, (unsigned long)pdm_dma_underruns);
        g_snapshot.pdm_ring_fill = rf;
        g_snapshot.pdm_ring_min  = pdm_min_fill;
        g_snapshot.pdm_dma_fill  = df;
        g_snapshot.pdm_ring_ur   = pdm_ring_underruns;
        g_snapshot.pdm_dma_ur    = pdm_dma_underruns;
    } else {
        g_snapshot.pdm_ring_fill = 0;
        g_snapshot.pdm_ring_min  = 0;
        g_snapshot.pdm_dma_fill  = 0;
        g_snapshot.pdm_ring_ur   = 0;
        g_snapshot.pdm_dma_ur    = 0;
    }

    // --- input: SPDIF RX lock/rate/errors + estimator ---
    {
        SpdifRxStatusPacket st;
        spdif_input_get_status(&st);
        SpdifServoDiag sd;
        spdif_input_get_servo_diag(&sd);
        fmt_hz(a, sd.hz_long); fmt_hz(b, sd.hz_smooth);
        printf("CLKDIAG input: src=%u st=%u rate=%lu lk=%u ls=%u par=%lu fifo=%u%% hzL=%s hzS=%s use=%c span=%lums\n",
               (unsigned)active_input_source, (unsigned)st.state,
               (unsigned long)st.sample_rate, (unsigned)st.lock_count,
               (unsigned)st.loss_count, (unsigned long)st.parity_errors,
               (unsigned)st.fifo_fill_pct, a, b, sd.which_long ? 'L' : 'S',
               (unsigned long)sd.span_ms);

        g_snapshot.in_src        = active_input_source;
        g_snapshot.in_state      = st.state;
        g_snapshot.in_lock       = st.lock_count;
        g_snapshot.in_loss       = st.loss_count;
        g_snapshot.in_rate       = st.sample_rate;
        g_snapshot.in_parity     = st.parity_errors;
        g_snapshot.in_fifo_pct   = st.fifo_fill_pct;
        g_snapshot.in_which_long = sd.which_long ? 1u : 0u;
        g_snapshot._rsv          = 0;
        g_snapshot.in_hz_long_mhz = hz_to_mhz(sd.hz_long);
        g_snapshot.in_span_ms    = sd.span_ms;
    }

    // --- loop: worst main-loop gap since last dump ---
    printf("CLKDIAG loop: gapmax=%luus\n", (unsigned long)loop_gap_max_us);
    g_snapshot.loop_gapmax_us = loop_gap_max_us;

    // Reset windowed aggregates.
    fc_min_hz = fc_max_hz = fc_last_hz;
    fc_count = 0;
    loop_gap_max_us = 0;
    pwm_moves = 0;
    pdm_min_fill = 100;
    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++)
        slot_min_fill[i] = (uint8_t)(output_type_switch_in_progress
                                     ? SPDIF_CONSUMER_BUFFER_COUNT
                                     : get_slot_consumer_fill((uint)i));
}

void clock_diag_get_snapshot(ClockDiagPacket *out) {
    // Plain copy of the latest 1 Hz latch.  Tearing is acceptable for
    // diagnostics; the control transfer and the poll both run on core 0's
    // main loop, so in practice they never interleave.
    *out = g_snapshot;
}

#endif // DSPI_CLOCK_DIAG
