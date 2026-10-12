/*
 * Copyright (c) 2026 WeebLabs
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_AUDIO_I2S_MULTI_H
#define _PICO_AUDIO_I2S_MULTI_H

#include "pico.h"
#include "hardware/pio.h"

/** \file audio_i2s_multi.h
 *  \defgroup pico_audio_i2s_multi pico_audio_i2s_multi
 *  Multi-instance I2S audio output using the PIO
 *
 * This library uses the \ref pio system to implement multiple independent
 * I2S audio output interfaces. Each instance drives one stereo pair on its
 * own data pin, sharing BCK/LRCLK clock signals via PIO side-set.
 *
 * Follows the same architectural patterns as pico_audio_spdif_multi:
 * instance-based API, one free-running DMA ring per slot with no IRQs, and
 * synchronized multi-instance start.
 *
 * Audio format: 24-bit samples left-justified in 32-bit I2S frames.
 * Standard Philips I2S timing: MSB-first, 1 BCK delay after LRCLK edge.
 */

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// Configuration defaults
// ---------------------------------------------------------------------------


/** Maximum number of I2S instances that can be registered */
#define PICO_AUDIO_I2S_MAX_INSTANCES 4


// ---------------------------------------------------------------------------
// Instance structure
// ---------------------------------------------------------------------------

/** \brief Per-instance state for an I2S output
 * \ingroup pico_audio_i2s_multi
 *
 * Caller allocates and zero-initializes before passing to audio_i2s_setup().
 */
typedef struct audio_i2s_instance {
    // Hardware config (set in setup, immutable after)
    PIO pio;
    uint8_t pio_sm;
    uint8_t dma_channel;
    uint8_t dma_reload_channel; // RP2040 only: free-running reload pair partner
    uint8_t data_pin;           // Serial audio data GPIO
    uint8_t clock_pin_base;     // BCK GPIO; LRCLK = clock_pin_base + 1
    bool    clock_master;       // true = drives BCK/LRCLK, false = data only
    bool    external_clock;     // true = wait-driven on external BCK/LRCLK inputs
                                // (clock_master must be false; divider fixed 1.0)

    // Runtime state
    uint32_t freq;
    bool enabled;
    uint8_t instance_index;     // Stable slot identity (== dma_channel)

    // Ring state; same model as the S/PDIF library (see audio_spdif.c
    // "Ring output model"). Frame counters are monotonic since ring reset.
    int32_t *ring;                      // slot ring base (caller-owned)
    volatile uint32_t wr_frames;        // frames of real audio written
    uint32_t erased_frames;             // silence stamped through this total
    volatile uint32_t underrun_frames;  // silence frames exposed to the wire
    volatile uint32_t drop_frames;      // producer frames dropped on ring-full

    // Consumed-side unwrap state (mutated only in the consumed getter).
    uint32_t last_read_addr;
    uint32_t last_read_time_us;          // selects whole laps across long polls
    volatile uint32_t consumed_words;   // words; USB feedback only
    volatile uint32_t consumed_frames;  // frames; ALL fill/deficit math
    uint8_t consumed_word_rem;          // mid-frame carry between reads

    // Coherent produced/consumed pair latched after each block write.
    volatile uint32_t snap_produced_frames;
    volatile uint32_t snap_consumed_frames;
    // Fill immediately before the most recent successful block write.
    // UINT32_MAX means no successful write has occurred since ring reset.
    volatile uint32_t recovery_prewrite_fill_frames;
} audio_i2s_instance_t;

// Ring geometry: shares the slot's 16 KB store with the S/PDIF instance
// (frames are 8 bytes here vs 16, so I2S uses the first half). Identical
// frame capacity keeps fill/latency semantics uniform across output types.
#define PICO_AUDIO_I2S_RING_FRAMES  1024u
#define PICO_AUDIO_I2S_RING_BYTES   (PICO_AUDIO_I2S_RING_FRAMES * 2u * 4u)
#define PICO_AUDIO_I2S_ERASE_AHEAD  192u

// ---------------------------------------------------------------------------
// Configuration structure
// ---------------------------------------------------------------------------

/** \brief Configuration for an I2S output instance
 * \ingroup pico_audio_i2s_multi
 */
typedef struct audio_i2s_config {
    uint8_t data_pin;           // Serial audio data GPIO
    uint8_t clock_pin_base;     // BCK GPIO; LRCLK = clock_pin_base + 1
    uint8_t dma_channel;
    uint8_t pio_sm;
    uint8_t pio;                // PIO block index (0, 1, or 2 on RP2350)
    bool    clock_master;       // true = drive BCK/LRCLK (master), false = data only (slave)
    bool    external_clock;     // true = slave to EXTERNAL BCK/LRCLK (pads are inputs;
                                // clock_master ignored).  Wait-driven at divider 1.0;
                                // rate changes do not touch these SMs' dividers.
    uint8_t dma_reload_channel; // RP2040 only: reload partner (ignored on RP2350)
    void   *ring_base;          // caller-owned per-slot ring storage (shared with
                                // the slot's S/PDIF instance), aligned to the
                                // S/PDIF ring size; I2S uses the first
                                // PICO_AUDIO_I2S_RING_BYTES of it
} audio_i2s_config_t;

// ---------------------------------------------------------------------------
// Ring output API (same model as pico_audio_spdif_multi)
// ---------------------------------------------------------------------------

// Encode a block of interleaved S32 L/R frames into the slot ring
// (<<8 | pad pattern), extend the erase-ahead silence region, latch the
// coherent produced/consumed snapshot pair.
void audio_i2s_ring_write_s32(audio_i2s_instance_t *inst,
                              const int32_t *interleaved_lr, uint32_t frames);

// Monotonic DMA words consumed since ring reset. Elapsed time selects any
// complete ring laps between calls; the DMA address supplies the remainder.
uint32_t audio_i2s_ring_consumed_words(audio_i2s_instance_t *inst);

// Frames consumed since ring reset. Wrap-coherent with wr_frames at 2^32;
// the words counter is NOT (divided down it wraps early), so all fill and
// deficit arithmetic must use this one.
uint32_t audio_i2s_ring_consumed_frames(audio_i2s_instance_t *inst);

static inline uint32_t audio_i2s_ring_produced_frames(audio_i2s_instance_t *inst) {
    return inst->wr_frames;
}

// Current fill in frames (produced minus consumed).
uint32_t audio_i2s_ring_fill_frames(audio_i2s_instance_t *inst);

// Reset ring pointers and accounting. DMA must be stopped.
void audio_i2s_ring_reset(audio_i2s_instance_t *inst);

// Group re-anchor primitive; the caller applies the SAME skip to every
// enabled slot of both output types so inter-slot alignment holds.
void audio_i2s_ring_skip(audio_i2s_instance_t *inst, uint32_t frames);

// Extend the erased (silence) region toward one full ring ahead of the
// reader, bounded by max_frames per call. Keeps a stalled producer from
// ever looping stale ring content; cheap no-op while streaming.
void audio_i2s_ring_silence_pump(audio_i2s_instance_t *inst, uint32_t max_frames);

// ---------------------------------------------------------------------------
// Public API — mirrors pico_audio_spdif_multi
// ---------------------------------------------------------------------------

/** \brief Set up an I2S audio output instance
 * \ingroup pico_audio_i2s_multi
 *
 * Initializes the PIO state machine with the 24-bit I2S program, claims the
 * DMA channel, resets the slot ring, registers the instance and programs the
 * nominal divider for \p sample_freq on every I2S SM of the block. Audio
 * reaches the instance only through audio_i2s_ring_write_s32().
 *
 * All instances on the same PIO block share the same BCK/LRCLK pins
 * (PIO side-set constraint).
 *
 * \param inst        Caller-allocated, zero-initialized instance
 * \param sample_freq Output sample rate in Hz
 * \param config      Hardware configuration
 */
void audio_i2s_setup(audio_i2s_instance_t *inst, uint32_t sample_freq,
                     const audio_i2s_config_t *config);

/** \brief Enable or disable an I2S output instance
 * \ingroup pico_audio_i2s_multi
 *
 * Uses reference-counted DMA IRQ enable/disable to support multiple instances.
 *
 * \param inst    The I2S instance
 * \param enabled true to enable, false to disable
 */
void audio_i2s_set_enabled(audio_i2s_instance_t *inst, bool enabled);

/** \brief Change the data pin of an I2S output instance
 * \ingroup pico_audio_i2s_multi
 *
 * The instance must be disabled before calling this function.
 * Aborts any stale DMA, releases the old data pin to high-Z,
 * reinitializes the PIO state machine with the new data pin,
 * and restores the clock divider.
 *
 * Clock pins (BCK/LRCLK) are NOT affected.
 *
 * \param inst    The I2S instance (must be disabled)
 * \param new_pin The new data GPIO pin number
 */
void audio_i2s_change_data_pin(audio_i2s_instance_t *inst, uint new_pin);

/** \brief Move an I2S output's data line onto a new GPIO without releasing the old one
 * \ingroup pico_audio_i2s_multi
 *
 * Same as audio_i2s_change_data_pin() except the old data pin is left
 * untouched, for callers that release outgoing pins themselves before claiming
 * incoming ones.
 *
 * \param inst    The I2S instance (must be disabled)
 * \param new_pin The new data GPIO pin number
 */
void audio_i2s_attach_data_pin(audio_i2s_instance_t *inst, uint new_pin);

/** \brief Enable multiple I2S instances with synchronized PIO start
 * \ingroup pico_audio_i2s_multi
 *
 * Primes DMA for all instances, then starts all PIO state machines
 * simultaneously using pio_enable_sm_mask_in_sync().
 *
 * \param instances Array of pointers to initialized instances
 * \param count     Number of instances
 */
void audio_i2s_enable_sync(audio_i2s_instance_t *instances[], uint count);

/** \brief Prepare-only half of audio_i2s_enable_sync
 * \ingroup pico_audio_i2s_multi
 *
 * Rewinds each SM to its program entry, primes DMA / IRQ refcounts and
 * marks the instances enabled without starting the SMs; returns the SM
 * mask for the shared PIO block so the caller can perform one combined
 * pio_enable_sm_mask_in_sync together with the SPDIF slots (used by the
 * I2S clock-slave path to gate the start on an external LRCLK edge).
 */
uint32_t audio_i2s_enable_sync_prepare(audio_i2s_instance_t *instances[], uint count);

/** \brief Reset DMA-starvation counters
 * \ingroup pico_audio_i2s_multi
 */
void audio_i2s_reset_dma_starvations(void);

/** \brief Get total DMA-starvation events across all instances
 * \ingroup pico_audio_i2s_multi
 */
uint32_t audio_i2s_get_dma_starvations(void);

/** \brief Get DMA-starvation events for one instance index (0..3)
 * \ingroup pico_audio_i2s_multi
 *
 * Index is the slot index (== dma_channel, same convention as the S/PDIF
 * library's instance_index).
 */
uint32_t audio_i2s_get_dma_starvations_instance(uint index);

/** \brief Tear down an I2S instance, releasing all hardware resources
 * \ingroup pico_audio_i2s_multi
 *
 * Disables the instance, aborts DMA, unclaims the DMA channel and PIO SM,
 * releases GPIO pins, and removes the instance from the registry.
 *
 * Used when switching an output slot from I2S back to S/PDIF.
 * The instance struct is zeroed and can be re-used with audio_i2s_setup().
 *
 * \param inst The I2S instance to tear down
 */
void audio_i2s_teardown(audio_i2s_instance_t *inst);

/** \brief Atomically update clock dividers for all I2S instances and restart in sync
 * \ingroup pico_audio_i2s_multi
 *
 * Stops all active I2S state machines, updates every registered instance's
 * clock divider, and restarts all in sync. Called from perform_rate_change()
 * so master and slave SMs never run at different dividers.
 *
 * \param sample_freq New sample rate in Hz
 */
void audio_i2s_update_all_frequencies(uint32_t sample_freq);

/** \brief Get the index of the current I2S clock master (-1 if none)
 * \ingroup pico_audio_i2s_multi
 */
int8_t audio_i2s_get_clock_master_index(void);

// ---------------------------------------------------------------------------
// External-clock framing-slip flag
// ---------------------------------------------------------------------------

// PIO irq flag raised by audio_i2s_dataout_extclk.pio when its per-frame
// LRCLK verification fails (external BCK glitch or LRCLK phase jump).  The
// program re-frames itself; software must treat the flag like a clock loss
// (the slipped slot is no longer sample-aligned with internally clocked
// slots until the next synchronized output start).
#define AUDIO_I2S_EXTCLK_SLIP_IRQ 7u

/** \brief Read and clear the external-clock framing-slip flag
 * \ingroup pico_audio_i2s_multi
 *
 * True if any external-clock I2S output SM flagged a framing slip since the
 * last call.  Also cleared by audio_i2s_enable_sync_prepare() for extclk
 * instances (a restart IS the slip handling).
 */
bool audio_i2s_extclk_framing_slipped(void);

// ---------------------------------------------------------------------------
// MCK (Master Clock) generator API
// ---------------------------------------------------------------------------
//
// MCK is generated by one of the four hardware CLK_GPOUTn clock outputs —
// no PIO state machine is consumed.  The GPIO must map to a CLK_GPOUTn on
// the current platform; use the SDK macro GPIO_TO_GPOUT_CLOCK_HANDLE() to
// validate before passing pins into this API:
//
//   if (GPIO_TO_GPOUT_CLOCK_HANDLE(pin, clk_sys) == clk_sys) { /* invalid */ }
//
// AUXSRC is fixed to clk_sys so the divider math is a single equation
// (sys_clk × 256 / (Fs × multiplier)) across both platforms.
//
// Lifecycle: setup() captures the pin without driving it; the GPOUTn block
// is configured on the first set_enabled(true) call.  Always call
// update_frequency() at least once before set_enabled(true) on a cold start
// so a sane divider is loaded — enabling with no divider is refused.

/** \brief Set up the MCK generator (records GPIO; does not drive the pin)
 * \ingroup pico_audio_i2s_multi
 *
 * The pin is captured but no hardware is configured until the first
 * audio_i2s_mck_set_enabled(true).  Caller must subsequently call
 * audio_i2s_mck_update_frequency() at least once before enabling, so
 * the GPOUTn block has a valid divider to load.
 *
 * \param pin GPIO for MCK output (must be CLK_GPOUTn-capable on the platform)
 */
void audio_i2s_mck_setup(uint pin);

/** \brief Enable or disable the MCK output
 * \ingroup pico_audio_i2s_multi
 *
 * On enable, configures the CLK_GPOUTn block (AUXSRC = clk_sys, last
 * computed 24.8 divider) and routes the pad mux.  On disable, disconnects
 * the pad mux so the pin floats; the generator continues running
 * internally (negligible quiescent draw — the SDK has no public API to
 * stop a CLK_GPOUTn generator).
 */
void audio_i2s_mck_set_enabled(bool enabled);

/** \brief Update MCK frequency for a new sample rate
 * \ingroup pico_audio_i2s_multi
 *
 * Computes the 24.8 divider as div = sys_clk × 256 / (sample_freq × multiplier).
 * If MCK is currently enabled, the divider is hot-loaded glitchlessly via
 * clock_gpio_init_int_frac8().  Otherwise the divider is stored and applied
 * on the next set_enabled(true).
 *
 * Ordering: always call this *before* set_enabled(true) on a cold start —
 * an enable with no divider configured is refused.
 *
 * \param sample_freq Sample rate in Hz (e.g., 48000)
 * \param multiplier  MCK multiplier: 128 or 256
 */
void audio_i2s_mck_update_frequency(uint32_t sample_freq, uint32_t multiplier);

/** \brief Change the MCK GPIO pin
 * \ingroup pico_audio_i2s_multi
 *
 * MCK must be disabled before calling.  The new pin takes effect on the
 * next audio_i2s_mck_set_enabled(true) — clock_gpio_init_int_frac8() at
 * that point routes the pad mux to whichever CLK_GPOUTn block the new pin
 * maps to.
 *
 * \param new_pin New GPIO for MCK output (must be CLK_GPOUTn-capable)
 */
void audio_i2s_mck_change_pin(uint new_pin);

/** \brief Get the current MCK GPIO pin (0xFF if setup() never called) */
uint8_t audio_i2s_mck_get_pin(void);

/** \brief Check if MCK is currently enabled */
bool audio_i2s_mck_is_enabled(void);

/** \brief Atomically synchronize MCK library state with a desired tuple
 * \ingroup pico_audio_i2s_multi
 *
 * After preset / bulk-params apply paths update the firmware's "user
 * preference" globals (`i2s_mck_pin`, `i2s_mck_enabled`, …) the library's
 * internal state can be stale.  The vendor-command path keeps the two in
 * sync via audio_i2s_mck_change_pin() / audio_i2s_mck_set_enabled(); the
 * apply paths historically did not, which left MCK driving the wrong pin
 * after a preset load that customised mck_pin.
 *
 * This helper is the apply-path equivalent: it sequences
 *   1. (if pin changed) disable-then-change_pin (change_pin asserts !running),
 *   2. update_frequency for the requested Fs × multiplier (idempotent reload
 *      if already running on the same pin),
 *   3. set_enabled to the requested state,
 * so the library always matches the (pin, enabled, Fs, multiplier) tuple
 * the caller passes in.  Idempotent — calling twice with the same args is
 * a no-op on the second call.
 *
 * Caller is responsible for ensuring `pin` is CLK_GPOUTn-capable on the
 * current platform (use GPIO_TO_GPOUT_CLOCK_HANDLE() to validate).  If
 * audio_i2s_mck_setup() has not been called yet, this is a no-op.
 *
 * \param pin         Desired MCK GPIO (must map to CLK_GPOUTn)
 * \param enabled     Desired enabled state
 * \param sample_freq Sample rate in Hz (for divider math)
 * \param multiplier  MCK multiplier (128 or 256)
 */
void audio_i2s_mck_apply_state(uint pin, bool enabled,
                               uint32_t sample_freq, uint32_t multiplier);

#ifdef __cplusplus
}
#endif

#endif // _PICO_AUDIO_I2S_MULTI_H
