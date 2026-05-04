/*
 * Copyright (c) 2026 WeebLabs
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_AUDIO_I2S_MULTI_H
#define _PICO_AUDIO_I2S_MULTI_H

#include "pico/audio.h"
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
 * instance-based API, shared DMA IRQ handler, reference-counted IRQ
 * enable/disable, and synchronized multi-instance start.
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

#ifndef PICO_AUDIO_I2S_DMA_IRQ
#ifdef PICO_AUDIO_SPDIF_DMA_IRQ
#define PICO_AUDIO_I2S_DMA_IRQ PICO_AUDIO_SPDIF_DMA_IRQ
#else
#define PICO_AUDIO_I2S_DMA_IRQ 1
#endif
#endif

/** Consumer buffer format identifier for I2S (raw PCM, not BMC-encoded) */
#define AUDIO_BUFFER_FORMAT_PIO_I2S 1301

/** Maximum number of I2S instances that can be registered */
#define PICO_AUDIO_I2S_MAX_INSTANCES 4

/** Samples per DMA transfer — matches SPDIF for pipeline compatibility */
#define PICO_AUDIO_I2S_DMA_SAMPLE_COUNT 48u

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
    uint8_t dma_irq;            // 0 or 1
    uint8_t data_pin;           // Serial audio data GPIO
    uint8_t clock_pin_base;     // BCK GPIO; LRCLK = clock_pin_base + 1
    bool    clock_master;       // true = drives BCK/LRCLK, false = data only

    // Runtime state
    audio_buffer_t *playing_buffer;
    uint32_t freq;
    bool enabled;

    // DMA word tracking for USB feedback endpoint
    volatile uint32_t words_consumed;       // Total DMA words consumed (incremented in DMA IRQ)
    uint32_t current_transfer_words;        // DMA word count of current transfer

    // Per-instance audio pipeline
    audio_format_t consumer_format;
    audio_buffer_format_t consumer_buffer_format;
    audio_buffer_t silence_buffer;
    audio_buffer_pool_t *consumer_pool;

    // Embedded connection (uses container_of to recover instance pointer)
    struct producer_pool_blocking_give_connection connection;
} audio_i2s_instance_t;

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
    uint8_t dma_irq;            // DMA IRQ index (0 or 1)
    bool    clock_master;       // [DEPRECATED, ignored] Election is internal:
                                // lowest-index registered instance becomes
                                // master; field retained only for ABI continuity.
} audio_i2s_config_t;

// ---------------------------------------------------------------------------
// Public API — mirrors pico_audio_spdif_multi
// ---------------------------------------------------------------------------

/** \brief Set up an I2S audio output instance
 * \ingroup pico_audio_i2s_multi
 *
 * Initializes the PIO state machine with the 24-bit I2S program, claims the
 * DMA channel, and registers the instance in the shared IRQ handler.
 *
 * All instances must share the same DMA IRQ line (asserted).
 * All instances on the same PIO block share the same BCK/LRCLK pins
 * (PIO side-set constraint).
 *
 * \param inst   Caller-allocated, zero-initialized instance
 * \param intended_audio_format  Desired audio format (48kHz stereo S32)
 * \param config Hardware configuration
 */
const audio_format_t *audio_i2s_setup(audio_i2s_instance_t *inst,
                                       const audio_format_t *intended_audio_format,
                                       const audio_i2s_config_t *config);

/** \brief Connect a producer pool to an I2S instance with extra options
 * \ingroup pico_audio_i2s_multi
 *
 * Creates the consumer buffer pool and establishes the producer-to-consumer
 * connection. The connection callback left-shifts 24-bit samples into 32-bit
 * I2S frames (MSB-aligned).
 *
 * \param inst           The I2S instance
 * \param producer       The producer buffer pool (PCM_S32, stride 8)
 * \param buffer_on_give If true, buffer on give side
 * \param buffer_count   Number of consumer buffers to allocate
 * \param connection     Optional custom connection (NULL for default)
 */
bool audio_i2s_connect_extra(audio_i2s_instance_t *inst,
                              audio_buffer_pool_t *producer,
                              bool buffer_on_give, uint buffer_count,
                              audio_connection_t *connection);

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

/** \brief Tear down an I2S instance, releasing all hardware resources
 * \ingroup pico_audio_i2s_multi
 *
 * Disables the instance, aborts DMA, unclaims the DMA channel and PIO SM,
 * releases GPIO pins, and removes the instance from the IRQ handler registry.
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
 * to avoid the brief master/slave divider mismatch that occurs with lazy
 * per-instance updates.
 *
 * \param sample_freq New sample rate in Hz
 */
void audio_i2s_update_all_frequencies(uint32_t sample_freq);

/** \brief Get the index of the current I2S clock master (-1 if none)
 * \ingroup pico_audio_i2s_multi
 */
int8_t audio_i2s_get_clock_master_index(void);

// ---------------------------------------------------------------------------
// I2S clock domain — read-only views of the current BCK/LRCLK source
// ---------------------------------------------------------------------------
//
// The clock domain is the (BCK, LRCLK, MCK, sample_freq) tuple that all
// I2S TX and RX state machines share. Today the clocks are folded into the
// first registered TX instance running audio_i2s_clkout (master mode);
// future work may split them onto a dedicated SM or operate in slave mode.
//
// Callers that need to observe BCK/LRCLK from another PIO block (e.g. the
// I2S input subsystem) should query these accessors rather than reaching
// into the TX instance struct, so the abstraction can evolve without
// breaking call sites.

/** \brief I2S clock domain mode
 * \ingroup pico_audio_i2s_multi
 */
typedef enum {
    AUDIO_I2S_CLOCK_MODE_MASTER = 0,  ///< Pico drives BCK/LRCLK (and optional MCK)
    AUDIO_I2S_CLOCK_MODE_SLAVE  = 1,  ///< External device drives BCK/LRCLK; PIO observes
} audio_i2s_clock_mode_t;

/** \brief Returns true iff a clock domain is currently active (any TX instance registered) */
bool audio_i2s_clock_domain_is_active(void);

/** \brief Current clock domain mode (always MASTER until slave-mode support lands) */
audio_i2s_clock_mode_t audio_i2s_clock_domain_mode(void);

/** \brief Absolute GPIO number of the BCK pin, or 0xFF if no clock domain is active */
uint8_t audio_i2s_clock_domain_bck_pin(void);

/** \brief Absolute GPIO number of the LRCLK pin (BCK + 1), or 0xFF if inactive */
uint8_t audio_i2s_clock_domain_lrclk_pin(void);

/** \brief Sample rate the clock domain is currently locked to, or 0 if inactive */
uint32_t audio_i2s_clock_domain_sample_freq(void);

// ---------------------------------------------------------------------------
// Clock domain configuration and bulk-change transactions
// ---------------------------------------------------------------------------
//
// The library tracks an I2S clock domain and runs an internal election on
// every TX/RX setup/teardown to decide which SM drives BCK/LRCLK:
//   - ≥1 TX registered: lowest-index TX runs `audio_i2s_clkout` (combined
//     clocks + first-slot data). Other TX SMs run `audio_i2s_dataout`.
//     RX SMs (when present) observe BCK/LRCLK via absolute `wait gpio`.
//   - 0 TX, ≥1 RX: a phantom SM (configured via _configure below) runs
//     `audio_i2s_clkout_only` to drive BCK/LRCLK on RX's behalf.
//   - 0 TX, 0 RX: clock domain is idle; BCK/LRCLK are not driven.
//
// Callers do not specify master/slave roles directly; the library picks
// the cheapest layout. The `clock_master` field in audio_i2s_config_t is
// ignored.

/** \brief Configure the clock domain — call once at boot, and again whenever
 *         the BCK pin changes via vendor command.
 *
 * Stores the global BCK pin and the location reserved for the phantom SM
 * (used only when a clkout-only fallback is needed). Idempotent; safe to
 * call when no I2S participants are active.
 *
 * \param bck_pin           BCK GPIO; LRCLK is bck_pin + 1.
 * \param phantom_pio_idx   PIO block (0/1/2) for the phantom SM.
 * \param phantom_sm        State machine index (0-3) for the phantom SM.
 */
void audio_i2s_clock_domain_configure(uint8_t bck_pin,
                                       uint8_t phantom_pio_idx,
                                       uint8_t phantom_sm);

/** \brief Begin a bulk-change transaction.
 *
 * Inside a transaction, audio_i2s_setup/teardown and audio_i2s_rx_setup/
 * teardown register changes without immediately re-running the election.
 * Election runs once at the matching commit. Nested transactions are
 * supported (depth-counter); election runs only when depth returns to 0.
 */
void audio_i2s_clock_domain_begin_transaction(void);

/** \brief Commit a bulk-change transaction.
 *
 * If this matches the outermost begin and any setup/teardown ran inside
 * the transaction, the election runs and applies role/phantom changes.
 * Sync-restarts any TX SMs whose role changed.
 */
void audio_i2s_clock_domain_commit_transaction(void);

// ---------------------------------------------------------------------------
// Clocks-only master — drives BCK/LRCLK without any audio data output
// ---------------------------------------------------------------------------
//
// Used when I2S input is selected and no I2S TX output is configured. Mutually
// exclusive with TX-master mode: caller must ensure no audio_i2s_instance has
// clock_master=true before starting clocks-only.
//
// All four functions are no-ops if called in an inappropriate state.

/** \brief Start the clocks-only master SM driving BCK/LRCLK from sys_clk
 * \param pio_idx  PIO block (0/1/2) hosting the SM
 * \param sm       State machine index (0-3)
 * \param bck_pin  BCK GPIO; LRCLK is bck_pin+1 (PIO side-set constraint)
 * \param fs       Sample rate in Hz
 * \return true on success, false if a TX master is active or sm/pin invalid
 */
bool audio_i2s_clock_domain_clkout_only_start(uint8_t pio_idx, uint8_t sm,
                                               uint8_t bck_pin, uint32_t fs);

/** \brief Stop the clocks-only master SM and release BCK/LRCLK */
void audio_i2s_clock_domain_clkout_only_stop(void);

/** \brief True iff the clocks-only master SM is currently running */
bool audio_i2s_clock_domain_clkout_only_is_active(void);

/** \brief Update the clocks-only master divider for a new sample rate */
void audio_i2s_clock_domain_clkout_only_set_frequency(uint32_t fs);

// ---------------------------------------------------------------------------
// I2S RX (input) — captures DIN synchronously with the clock domain
// ---------------------------------------------------------------------------
//
// The RX SM observes BCK/LRCLK via absolute `wait gpio` so it can live on any
// PIO block. The DMA writes captured samples into a caller-allocated ring
// buffer in continuous wrap mode (single DMA channel, `set_ring` on the
// write side; the consumer polls dma write_addr to know how many new samples
// are available).
//
// Frame format: 32-bit-per-channel I2S, 24-bit audio left-justified MSB-first
// (matches audio_i2s_clkout). Audio occupies bits [31:8] of each pushed word.
//
// Slot count (2 for stereo I2S, 4/8 for TDM) is stored in the instance for
// future extensibility; Phase 2 supports stereo only.

/** \brief Per-instance state for an I2S input
 * \ingroup pico_audio_i2s_multi
 */
typedef struct audio_i2s_rx_instance {
    PIO pio;                          // PIO block hosting the RX SM
    uint8_t pio_sm;
    uint8_t dma_channel;              // Single ring-wrapped DMA channel
    uint8_t din_pin;                  // Audio data input GPIO
    uint8_t slot_count;               // 2 (stereo I2S); 4/8 reserved for TDM
    uint8_t slot_width_bits;          // 32 (24-in-32 left-justified default)
    uint8_t reserved;
    uint32_t freq;                    // Current sample rate
    bool active;                      // SM enabled and DMA armed

    // Caller-owned ring buffer (aligned to ring size).
    uint32_t *ring;                   // Word-typed for stride convenience
    uint32_t  ring_size_words;        // Power of 2 — DMA write-ring wraps here
    uint8_t   ring_log2_bytes;        // log2 of ring size in bytes

    // Tracking
    uint32_t read_idx_words;          // Consumer's read position in ring
} audio_i2s_rx_instance_t;

/** \brief Configuration for an I2S input instance
 * \ingroup pico_audio_i2s_multi
 */
typedef struct audio_i2s_rx_config {
    uint8_t din_pin;                  // Audio data input GPIO
    uint8_t pio_sm;                   // State machine index (0-3)
    uint8_t pio;                      // PIO block index (0/1/2)
    uint8_t dma_channel;              // DMA channel for ring writes
    uint8_t slot_count;               // 2 = stereo I2S
    uint8_t slot_width_bits;          // 32 = 24-in-32 left-justified default
    uint8_t reserved[2];
    uint32_t *ring;                   // Caller-allocated, aligned to ring size
    uint32_t  ring_size_words;        // Power of 2 (e.g. 2048 = 8 KB)
} audio_i2s_rx_config_t;

/** \brief Set up the I2S input instance
 *
 * Claims the PIO SM and DMA channel, loads the input PIO program, patches
 * absolute BCK/LRCLK pin operands, and configures DMA for ring-wrapped
 * continuous write. Does not enable the SM — call audio_i2s_rx_set_enabled().
 *
 * The clock domain must already be driving BCK/LRCLK (either a TX master
 * or audio_i2s_clock_domain_clkout_only_start()) before the SM is enabled,
 * but the order of setup vs clock domain start is not constrained.
 *
 * \return true on success
 */
bool audio_i2s_rx_setup(audio_i2s_rx_instance_t *inst,
                         const audio_i2s_rx_config_t *config,
                         uint32_t sample_freq);

/** \brief Enable/disable the I2S RX SM
 *
 * Enable: drains RX FIFO, arms DMA, jumps SM to entry point, enables SM.
 * Disable: stops SM, aborts DMA.
 */
void audio_i2s_rx_set_enabled(audio_i2s_rx_instance_t *inst, bool enabled);

/** \brief Update the RX SM divider for a new sample rate
 *
 * Caller should disable, call this, then re-enable. The first stereo frame
 * after re-enable should be discarded by the consumer (handled in poll()).
 */
void audio_i2s_rx_set_frequency(audio_i2s_rx_instance_t *inst, uint32_t fs);

/** \brief Tear down the I2S RX instance and release all hardware */
void audio_i2s_rx_teardown(audio_i2s_rx_instance_t *inst);

/** \brief Current DMA write position as a word index into the ring (0..ring_size_words-1) */
uint32_t audio_i2s_rx_get_write_idx(const audio_i2s_rx_instance_t *inst);

// ---------------------------------------------------------------------------
// MCK (Master Clock) generator API
// ---------------------------------------------------------------------------
// MCK is generated by a hardware CLK_GPOUT block (no PIO state machine
// required).  The chosen GPIO determines which clk_gpout instance is used;
// see GPIO_TO_GPOUT_CLOCK_HANDLE in pico-sdk/hardware/clocks.h for the
// pin-to-clock mapping.  On RP2040 the only DSPi-friendly GPOUT is GPIO 21
// (gpout0); GPIOs 23-25 are also gpout-capable but reserved for power/LED.
// RP2350 additionally exposes gpout0/gpout1 on GPIOs 13/15.

/** \brief Capture the MCK GPIO pin (no hardware action yet)
 * \ingroup pico_audio_i2s_multi
 *
 * Records the pin number so subsequent set_enabled(true) calls know where
 * to route the clk_gpout output.  MCK starts disabled — call
 * audio_i2s_mck_update_frequency() to set the divider, then
 * audio_i2s_mck_set_enabled(true) to start.
 *
 * \param pin GPIO that maps to a clk_gpout instance (validate via
 *            GPIO_TO_GPOUT_CLOCK_HANDLE macro before calling).
 */
void audio_i2s_mck_setup(uint pin);

/** \brief Enable or disable the MCK output
 * \ingroup pico_audio_i2s_multi
 *
 * On enable, configures the clk_gpout block (AUXSRC = clk_sys, current
 * divider) and routes the GPIO pad mux to the clock output function.
 * On disable, returns the GPIO pad to high-Z.  audio_i2s_mck_update_frequency
 * must have been called at least once before the first enable.
 */
void audio_i2s_mck_set_enabled(bool enabled);

/** \brief Update MCK frequency for a new sample rate
 * \ingroup pico_audio_i2s_multi
 *
 * Computes the clk_gpout divider for the given sample rate and multiplier.
 * If MCK is currently enabled, the divider register is updated live (the
 * clk_gpout block accepts glitchless divider changes).
 *
 * Divider math: divider_24.8 = sys_clk × 256 / (sample_freq × multiplier).
 * At 307.2 MHz sys_clk:
 *   48 kHz × 128× → divider 50.0 (integer, clean)
 *   48 kHz × 256× → divider 25.0 (integer, clean — was fractional 12.5 with PIO)
 *   96 kHz × 128× → divider 25.0 (integer, clean — was fractional 12.5 with PIO)
 *   96 kHz × 256× → divider 12.5 (still fractional)
 *
 * \param sample_freq Sample rate in Hz (e.g. 48000)
 * \param multiplier  MCK multiplier: 128 or 256
 */
void audio_i2s_mck_update_frequency(uint32_t sample_freq, uint32_t multiplier);

/** \brief Change the MCK GPIO pin
 * \ingroup pico_audio_i2s_multi
 *
 * MCK must be disabled before calling.  The new pin takes effect on the
 * next audio_i2s_mck_set_enabled(true).
 *
 * \param new_pin New GPIO; must be a clk_gpout-capable pin.
 */
void audio_i2s_mck_change_pin(uint new_pin);

/** \brief Get the current MCK GPIO pin */
uint8_t audio_i2s_mck_get_pin(void);

/** \brief Check if MCK is currently enabled */
bool audio_i2s_mck_is_enabled(void);

/** \brief Set the MCK clk_gpout divider directly (24.8 fixed-point)
 * \ingroup pico_audio_i2s_multi
 *
 * Used by the SPDIF input clock servo to keep MCK frequency-locked
 * to the servoed I2S data rate.  The 24.8 format matches the value
 * written to CLK_GPOUTn_DIV (16-bit integer in [23:8] + 8-bit fraction
 * in [7:0]).  Caller must compute as: sys_clk × 256 / target_MCK_hz.
 *
 * \param div_24_8 Clock divider in 24.8 fixed-point format
 */
void audio_i2s_mck_set_divider(uint32_t div_24_8);

#ifdef __cplusplus
}
#endif

#endif // _PICO_AUDIO_I2S_MULTI_H
