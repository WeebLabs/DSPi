/*
 * soft_vcxo.h; fractional-N trim of the system PLL ("soft VCXO").
 *
 * Replaces per-output PIO divider trimming for input clock tracking: a
 * ppm-level nudge of sys_clk moves every derived output clock (SPDIF TX,
 * I2S TX, ADAT TX, MCK, PDM) together, so inter-slot alignment is
 * preserved by construction. See soft_vcxo.c for the mechanism.
 */

#ifndef SOFT_VCXO_H
#define SOFT_VCXO_H

#include "config.h"

// One-time hardware bring-up (DMA channels, DMA timer, PWM pacer; the shaped
// build claims its PIO pacer and GPIO 25 instead); reads the
// clock plan from pll_sys. Inert unless clk_sys runs from pll_sys. A runtime
// sys_clk change must stop the pulse train first: it rewrites fbdiv_int.
void soft_vcxo_init(void);

// Command a sys_clk offset in ppm (clamped to +-SOFT_VCXO_MAX_PPM);
// 0 parks the PLL at exactly nominal. Cheap; callable at servo rate.
void soft_vcxo_set_ppm(float ppm);

// Last commanded (post-clamp) offset, for status/debug.
float soft_vcxo_current_ppm(void);

// Shaped actuator (SOFT_VCXO_SHAPED): refills the pulse ring and steps the
// start/park sequence. Core 0 only, at every producer delivery; a gap longer
// than the ring (about 3.4 ms) replays stale pulses. No-op otherwise.
#if SOFT_VCXO_SHAPED
void soft_vcxo_service(void);

enum { SOFT_VCXO_ST_INERT, SOFT_VCXO_ST_PARKED, SOFT_VCXO_ST_RUNNING, SOFT_VCXO_ST_DRAINING };

// Pacer delay range that keeps every slot at exactly 5 XOSC periods.
#define SOFT_VCXO_D_MIN  40u
#define SOFT_VCXO_D_MAX  59u
#else
static inline void soft_vcxo_service(void) {}
#endif

#if DSPI_CLOCK_DIAG
#include <stdint.h>
#include <stdbool.h>

// Snapshot of the VCXO command path internals for the CLKDIAG dump.  min/max
// commanded ppm are windowed: soft_vcxo_get_diag() resets them on read.
typedef struct {
    float    last_ppm;      // last post-clamp commanded ppm
    float    min_ppm;       // min commanded ppm since last read
    float    max_ppm;       // max commanded ppm since last read
    uint32_t set_calls;     // total soft_vcxo_set_ppm calls (cumulative)
    uint32_t clamp_sats;    // times a command hit +-MAX_PPM (cumulative)
    uint32_t sign_changes;  // command sign flips (cumulative)
    uint32_t pulse_hz;      // effective pulse rate sys/(clkdiv*wrap), 0=parked
    uint32_t pwm_wrap;      // PWM period (wrap+1); up to 0x10000
    uint16_t pwm_div;       // programmed PWM clkdiv
    uint8_t  fbdiv0;        // commanded excursion fbdiv (127/128/129)
    bool     parked;        // true when the last command parked the loop
} SoftVcxoDiag;

// Fill *out and reset the windowed min/max ppm.  Cheap; main-loop context.
void soft_vcxo_get_diag(SoftVcxoDiag *out);

#if SOFT_VCXO_SHAPED
// Bench view of the shaped actuator, served as DIAG.VCXO_STATUS (layout in
// soft_vcxo_spec.md section 8.7). Counters are cumulative; fields marked
// windowed restart on every read.
typedef struct __attribute__((packed)) {
    uint8_t  version;          // 1
    uint8_t  state;            // SoftVcxoShapedState
    uint8_t  delay_d;          // pacer delay X
    uint8_t  forced;           // 1 while DIAG.VCXO_FORCE overrides the servo
    float    force_ppm;
    float    servo_ppm;        // last post-clamp soft_vcxo_set_ppm command
    float    run_ppm;          // command the modulator is running
    int32_t  u_q24;            // modulator input, pulses per decision in Q24
    uint32_t starts;
    uint32_t stops;
    uint32_t replays;          // fills that found the DMA past the written ring
    uint32_t fill_calls;
    uint64_t decisions;        // modulator decisions written
    int64_t  net_pulses;       // sum of decisions (+1 / 0 / -1)
    uint64_t fill_cycles;      // core-0 DWT cycles spent filling
    uint32_t lead_min_bytes;   // windowed: least unread ring content seen
    uint32_t fill_gap_max_us;  // windowed: longest gap between running fills
    int32_t  i1_peak;          // windowed |integrator 1|, Q24
    int32_t  i2_peak;          // windowed |integrator 2|, Q24
    uint32_t sat_hits;         // integrator clamp engagements
    uint16_t fbdiv_live;
    uint8_t  sm_enabled;
    uint8_t  dma_busy;         // bit 0 writer, bit 1 feed
    uint32_t ring_bytes;
    uint64_t fc0_sum;          // clock_diag fc0 results summed, 1/32 kHz units
    uint32_t fc0_gates;        // results in fc0_sum (about 32.8 ms each)
    uint32_t overtaken;        // fills the DMA caught up with before they finished
} SoftVcxoShapedDiag;


void soft_vcxo_shaped_get_diag(SoftVcxoShapedDiag *out);
// Run a fixed trim in place of the servo's command (on = false releases it).
void soft_vcxo_force(bool on, float ppm);
bool soft_vcxo_get_force(float *ppm);
// New pacer delay; a running pacer parks and restarts on it. False if out of range.
bool soft_vcxo_set_delay(uint8_t d);
uint8_t soft_vcxo_get_delay(void);
#endif // SOFT_VCXO_SHAPED
#endif // DSPI_CLOCK_DIAG

#endif // SOFT_VCXO_H
