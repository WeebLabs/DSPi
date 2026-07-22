/*
 * clock_diag.h; bench clock / audio-path diagnostics.
 *
 * A serial-console instrument (all lines prefixed "CLKDIAG ") that lets us
 * separate the failure classes behind DAC cutouts while the soft VCXO tracks
 * an input: sys_clk excursions (H1), servo command oscillation (H2), transient
 * consumer starvation (H3), TX-path/main-loop stalls (H4), and a wedged VCXO
 * DMA/PWM loop (H5).  Gated entirely by DSPI_CLOCK_DIAG (config.h); compiled
 * out to no-ops when 0.  Console printf only; no wire/notify/version surface.
 */

#ifndef CLOCK_DIAG_H
#define CLOCK_DIAG_H

#include "config.h"

#if DSPI_CLOCK_DIAG

#include <stdint.h>

// Latched once-per-second snapshot of every CLKDIAG field, for the read-only
// REQ_GET_CLOCK_DIAG vendor command (no console backend exists in this build).
// Packed, little-endian, decoded host-side (scratchpad/clkdiag_poll.py).  ppm
// fields are centi-ppm (ppm*100); frequencies flagged _mhz are milli-Hz
// (Hz*1000).  Per-slot arrays are sized NUM_SPDIF_INSTANCES; num_slots reports
// the live count.  Tearing is acceptable (diagnostics; filled from the main
// loop, memcpy'd from the control transfer).
typedef struct __attribute__((packed)) {
    uint8_t  version;            // format version (=1)
    uint8_t  platform;           // PLATFORM_RP2040 / PLATFORM_RP2350
    uint8_t  num_slots;          // NUM_SPDIF_INSTANCES
    uint8_t  pll_lock;           // sys PLL locked

    // sysclk (fc0-measured), windowed since last poll
    uint32_t sysclk_last_hz;
    uint32_t sysclk_min_hz;
    uint32_t sysclk_max_hz;
    uint32_t sysclk_count;

    // vcxo command path
    int32_t  vcxo_last_cppm;     // last post-clamp commanded ppm*100
    int32_t  vcxo_min_cppm;      // windowed min
    int32_t  vcxo_max_cppm;      // windowed max
    uint32_t vcxo_pulse_hz;      // effective pulse rate (0=parked)
    uint32_t vcxo_pwm_wrap;
    uint16_t vcxo_pwm_div;
    uint16_t vcxo_fbdiv_live;    // pll_sys_hw->fbdiv_int & 0xffff
    uint32_t vcxo_set_calls;     // cumulative
    uint32_t vcxo_clamp_sats;    // cumulative
    uint32_t vcxo_sign_changes;  // cumulative
    uint8_t  vcxo_fbdiv0;        // commanded excursion fbdiv (127/128/129)
    uint8_t  vcxo_parked;

    // vcxo DMA / PWM loop-alive evidence
    uint8_t  loop_pulse_busy;
    uint8_t  loop_pulse_en;
    uint8_t  loop_pace_busy;
    uint8_t  loop_pace_en;
    uint32_t loop_pulse_tcr;
    uint32_t loop_pace_tcr;
    uint32_t loop_pace_ctdreq;
    uint32_t loop_pwm_moves;     // windowed
    uint16_t loop_pwm_ctr;

    // servo term breakdown
    uint32_t servo_actual_mhz;   // measured input rate *1000
    int32_t  servo_ff_cppm;
    int32_t  servo_total_cppm;
    int32_t  servo_min_cppm;     // windowed
    int32_t  servo_max_cppm;     // windowed
    int32_t  servo_int_cppm;     // integrator, ppm*100
    int32_t  servo_fill_error;
    int32_t  servo_fill_slot;
    uint32_t servo_ticks;        // cumulative actuating ticks

    // per-slot (NUM_SPDIF_INSTANCES)
    uint8_t  slot_fill[NUM_SPDIF_INSTANCES];
    uint8_t  slot_min_fill[NUM_SPDIF_INSTANCES];  // windowed
    uint8_t  slot_is_i2s[NUM_SPDIF_INSTANCES];    // 1=i2s, 0=spdif
    uint32_t slot_starv[NUM_SPDIF_INSTANCES];     // cumulative DMA starvations
    uint32_t slot_dwc[NUM_SPDIF_INSTANCES];       // words consumed this window

    // pdm
    uint8_t  pdm_enabled;
    uint8_t  pdm_ring_fill;
    uint8_t  pdm_ring_min;       // windowed
    uint8_t  pdm_dma_fill;
    uint32_t pdm_ring_ur;        // cumulative
    uint32_t pdm_dma_ur;         // cumulative

    // input (SPDIF RX + estimator)
    uint8_t  in_src;
    uint8_t  in_state;
    uint8_t  in_lock;            // lock_count
    uint8_t  in_loss;            // loss_count
    uint32_t in_rate;            // detected Hz
    uint32_t in_parity;          // cumulative parity errors
    uint16_t in_fifo_pct;
    uint8_t  in_which_long;      // 1=long-window estimate applied
    uint8_t  _rsv;
    uint32_t in_hz_long_mhz;     // long-window rate *1000
    uint32_t in_span_ms;         // long-window anchor span

    // main loop
    uint32_t loop_gapmax_us;     // windowed worst gap
} ClockDiagPacket;

// Copy the latest 1 Hz snapshot.  Main-loop / control-transfer context.
void clock_diag_get_snapshot(ClockDiagPacket *out);

// One-time bring-up: enable the TX-library starvation monitors and seed the
// timing baselines.  Call once after soft_vcxo_init().
void clock_diag_init(void);

// Cheap per-iteration sampler: services the non-blocking fc0 frequency-counter
// state machine, tracks the max main-loop gap, min consumer fill per slot, and
// PWM-pacer counter movement.  Call once near the top of every main-loop pass.
void clock_diag_sample(void);

// Once-per-second multi-line dump.  Call from the main loop; internally
// throttled.  Resets the windowed aggregates after printing.
void clock_diag_poll(void);

#else

static inline void clock_diag_init(void) {}
static inline void clock_diag_sample(void) {}
static inline void clock_diag_poll(void) {}

#endif // DSPI_CLOCK_DIAG

#endif // CLOCK_DIAG_H
