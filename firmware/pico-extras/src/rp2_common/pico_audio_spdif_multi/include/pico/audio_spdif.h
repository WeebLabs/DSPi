/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_AUDIO_SPDIF_H
#define _PICO_AUDIO_SPDIF_H

#include "pico/audio.h"

/** \file audio_spdif.h
 *  \defgroup pico_audio_spdif pico_audio_spdif
 *  S/PDIF audio output using the PIO
 *
 * This library uses the \ref pio system to implement a S/PDIF audio interface.
 * Multiple instances can operate concurrently on independent PIO SMs/DMA
 * channels/GPIO pins.
 */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PICO_AUDIO_SPDIF_DMA_IRQ
#ifdef PICO_AUDIO_DMA_IRQ
#define PICO_AUDIO_SPDIF_DMA_IRQ PICO_AUDIO_DMA_IRQ
#else
#define PICO_AUDIO_SPDIF_DMA_IRQ 0
#endif
#endif

#ifndef PICO_AUDIO_SPDIF_PIO
#ifdef PICO_AUDIO_PIO
#define PICO_AUDIO_SPDIF_PIO PICO_AUDIO_PIO
#else
#define PICO_AUDIO_SPDIF_PIO 0
#endif
#endif

#ifndef PICO_AUDIO_SPDIF_MAX_CHANNELS
#ifdef PICO_AUDIO_MAX_CHANNELS
#define PICO_AUDIO_SPDIF_MAX_CHANNELS PICO_AUDIO_MAX_CHANNELS
#else
#define PICO_AUDIO_SPDIF_MAX_CHANNELS 2u
#endif
#endif

#ifndef PICO_AUDIO_SPDIF_BUFFERS_PER_CHANNEL
#ifdef PICO_AUDIO_BUFFERS_PER_CHANNEL
#define PICO_AUDIO_SPDIF_BUFFERS_PER_CHANNEL PICO_AUDIO_BUFFERS_PER_CHANNEL
#else
#define PICO_AUDIO_SPDIF_BUFFERS_PER_CHANNEL 3u
#endif
#endif

// fixed by S/PDIF
#define PICO_AUDIO_SPDIF_BLOCK_SAMPLE_COUNT 192u

// DMA transfer granularity — must divide PICO_AUDIO_SPDIF_BLOCK_SAMPLE_COUNT evenly
#ifndef PICO_AUDIO_SPDIF_DMA_SAMPLE_COUNT
#define PICO_AUDIO_SPDIF_DMA_SAMPLE_COUNT 48u
#endif

// Bytes per stereo frame in the S/PDIF consumer buffer (2 x sizeof(spdif_subframe_t)).
// This is the largest output-type consumer stride, so it sizes the shared per-slot
// consumer pool's data blocks (I2S, stride 8, under-fills them). Asserted against the
// real type in audio_spdif.c.
#define PICO_AUDIO_SPDIF_CONSUMER_FRAME_BYTES 16u

// Allow use of pico_audio driver without actually doing anything much
#ifndef PICO_AUDIO_SPDIF_NOOP
#ifdef PICO_AUDIO_NOOP
#define PICO_AUDIO_SPDIF_NOOP PICO_AUDIO_NOOP
#else
#define PICO_AUDIO_SPDIF_NOOP 0
#endif
#endif

#ifndef PICO_AUDIO_SPDIF_MONO_INPUT
#define PICO_AUDIO_SPDIF_MONO_INPUT 0
#endif

#ifndef PICO_AUDIO_SPDIF_PIN
#define PICO_AUDIO_SPDIF_PIN 0
#endif

#define AUDIO_BUFFER_FORMAT_PIO_SPDIF 1300

#define PICO_AUDIO_SPDIF_MAX_INSTANCES 4

#include "hardware/pio.h"

/** \brief Per-instance state for an S/PDIF output
 * \ingroup audio_spdif
 */
// Ring geometry: one encoded ring per slot replaces the consumer pool.
// Power of 2 (hardware read-address wrap on RP2350); 1024 frames = 21.3 ms
// at 48 kHz. The writer keeps ERASE_AHEAD frames of valid silence stamped
// ahead of real audio so a stalled producer plays silence, never a loop of
// stale ring content.
#define PICO_AUDIO_SPDIF_RING_FRAMES     1024u
#define PICO_AUDIO_SPDIF_RING_BYTES      (PICO_AUDIO_SPDIF_RING_FRAMES * 2u * 8u)
#define PICO_AUDIO_SPDIF_ERASE_AHEAD     192u   // one IEC block of silence margin

typedef struct audio_spdif_instance {
    // Hardware config (set in setup, immutable after)
    PIO pio;
    uint8_t pio_sm;
    uint8_t dma_channel;
    uint8_t dma_reload_channel; // RP2040 only: free-running reload pair partner
    uint8_t dma_irq;            // 0 or 1; kept for the shared refcount API (RX hold)
    uint8_t pin;
    uint8_t instance_index;     // Stable slot identity (== dma_channel)

    // Runtime state
    uint32_t freq;
    bool enabled;

    // Ring state. Frame counters are monotonic since the last ring reset;
    // uint32 wrap is harmless (all consumers use differences).
    uint32_t *ring;                     // slot ring base (caller-owned words)
    volatile uint32_t wr_frames;        // frames of real audio written
    uint32_t erased_frames;             // silence stamped through this total
    volatile uint32_t underrun_frames;  // silence frames exposed to the wire
    volatile uint32_t drop_frames;      // producer frames dropped on ring-full

    // Consumed-side unwrap state; mutated only inside
    // audio_spdif_ring_consumed_words() under a brief IRQ-off section.
    uint32_t last_read_addr;
    uint32_t last_read_time_us;          // selects whole laps across long polls
    volatile uint32_t consumed_words;   // words; USB feedback only
    volatile uint32_t consumed_frames;  // frames; ALL fill/deficit math
    uint8_t consumed_word_rem;          // mid-frame carry between reads

    // Cumulative-count snapshot latched at the end of each block write:
    // a coherent (constant-production-phase) observation pair for the
    // clock servo, free of the block-arrival sawtooth.
    volatile uint32_t snap_produced_frames;
    volatile uint32_t snap_consumed_frames;
    // Fill immediately before the most recent successful block write.
    // UINT32_MAX means no successful write has occurred since ring reset.
    volatile uint32_t recovery_prewrite_fill_frames;

    // Producer-side formats + embedded connection (kept until the direct
    // ring-write migration removes the producer pools)
    audio_format_t consumer_format;
    audio_buffer_format_t consumer_buffer_format;
    struct producer_pool_blocking_give_connection connection;
} audio_spdif_instance_t;

/** \brief Configuration for an S/PDIF output instance
 * \ingroup audio_spdif
 */
typedef struct audio_spdif_config {
    uint8_t pin;
    uint8_t dma_channel;
    uint8_t dma_reload_channel; // RP2040 only: reload partner (ignored on RP2350)
    uint8_t pio_sm;
    uint8_t pio;        // PIO block index (0, 1, or 2 on RP2350)
    uint8_t dma_irq;    // DMA IRQ index (0 or 1)
    void   *ring_base;  // caller-owned per-slot ring storage,
                        // PICO_AUDIO_SPDIF_RING_BYTES, aligned to its size;
                        // shared with the slot's I2S instance across retypes
} audio_spdif_config_t;

/** \brief Set up an S/PDIF audio output instance
 * \ingroup audio_spdif
 *
 * \param inst The instance to initialize (caller-allocated, zero-initialized)
 * \param intended_audio_format The desired audio format
 * \param config The hardware configuration to apply
 */
const audio_format_t *audio_spdif_setup(audio_spdif_instance_t *inst,
                                        const audio_format_t *intended_audio_format,
                                        const audio_spdif_config_t *config);

/** \brief Connect a producer pool to an S/PDIF instance with extra options
 * \ingroup audio_spdif
 *
 * \param inst The S/PDIF instance
 * \param producer The producer buffer pool
 * \param buffer_on_give If true, buffer on give side
 * \param consumer_pool Caller-owned consumer pool (re-formatted for S/PDIF here);
 *                      enables one static pool to be reused across output types
 * \param connection Optional custom connection (NULL for default)
 */
bool audio_spdif_connect_extra(audio_spdif_instance_t *inst,
                               audio_buffer_pool_t *producer,
                               bool buffer_on_give, audio_buffer_pool_t *consumer_pool,
                               audio_connection_t *connection);

/** \brief Enable or disable an S/PDIF output instance
 * \ingroup audio_spdif
 *
 * \param inst The S/PDIF instance
 * \param enabled true to enable, false to disable
 */
void audio_spdif_set_enabled(audio_spdif_instance_t *inst, bool enabled);

/** \brief Change the GPIO pin of an S/PDIF output instance
 * \ingroup audio_spdif
 *
 * The instance must be disabled before calling this function.
 * Aborts any stale DMA, releases the old pin to high-Z, reinitializes the
 * PIO state machine with the new pin, and restores the clock divider.
 *
 * \param inst The S/PDIF instance (must be disabled)
 * \param new_pin The new GPIO pin number
 */
void audio_spdif_change_pin(audio_spdif_instance_t *inst, uint new_pin);

/** \brief Fully tear down an S/PDIF output instance for output-type switching
 * \ingroup audio_spdif
 *
 * Counterpart to audio_spdif_setup() and mirror of audio_i2s_teardown().
 * Releases the DMA channel, PIO SM, pin, and registry slot so this slot's DMA
 * channel can be re-claimed by its I2S instance.  The caller-owned consumer
 * pool is detached but not freed (re-formatted on the next connect).
 *
 * \param inst The S/PDIF instance to tear down
 */
void audio_spdif_teardown(audio_spdif_instance_t *inst);

// ---------------------------------------------------------------------------
// Ring output API
// ---------------------------------------------------------------------------

/** \brief Encode a block of interleaved S32 L/R frames into the slot ring.
 *
 * Stamps IEC preamble/channel-status per frame from the absolute frame
 * index (phase = total frames mod 192), then extends the erase-ahead
 * silence region. Frames beyond the writable window are dropped and
 * counted in drop_frames. Latches the cumulative produced/consumed
 * snapshot pair on return.
 */
void audio_spdif_ring_write_s32(audio_spdif_instance_t *inst,
                                const int32_t *interleaved_lr, uint32_t frames);

/** \brief Monotonic DMA words consumed since the last ring reset.
 *
 * Unwraps the DMA read pointer. Elapsed time selects any whole ring laps
 * between calls; the hardware pointer supplies the exact in-ring remainder.
 */
uint32_t audio_spdif_ring_consumed_words(audio_spdif_instance_t *inst);

// Frames consumed since ring reset. Wrap-coherent with wr_frames at 2^32;
// the words counter is NOT (divided down it wraps early), so all fill and
// deficit arithmetic must use this one.
uint32_t audio_spdif_ring_consumed_frames(audio_spdif_instance_t *inst);

/** \brief Frames of real audio written since the last ring reset. */
static inline uint32_t audio_spdif_ring_produced_frames(audio_spdif_instance_t *inst) {
    return inst->wr_frames;
}

/** \brief Current fill in frames (produced minus consumed). */
uint32_t audio_spdif_ring_fill_frames(audio_spdif_instance_t *inst);

/** \brief Reset ring pointers and accounting. DMA must be stopped. */
void audio_spdif_ring_reset(audio_spdif_instance_t *inst);

/** \brief Group re-anchor primitive: advance the write position by
 * `frames` without writing audio (the skipped region already carries
 * erase-ahead silence). Counted in underrun_frames. The caller applies
 * the SAME skip to every enabled slot so inter-slot alignment holds.
 */
void audio_spdif_ring_skip(audio_spdif_instance_t *inst, uint32_t frames);

// Extend the erased (silence) region toward one full ring ahead of the
// reader, bounded by max_frames per call. Keeps a stalled producer from
// ever looping stale ring content; cheap no-op while streaming.
void audio_spdif_ring_silence_pump(audio_spdif_instance_t *inst, uint32_t max_frames);

/** \brief Enable multiple S/PDIF instances with synchronized PIO start
 * \ingroup audio_spdif
 *
 * Primes DMA for all instances, then starts all PIO state machines
 * simultaneously using pio_enable_sm_mask_in_sync(). SMs on the same
 * PIO block start on the exact same clock cycle.
 *
 * \param instances Array of pointers to initialized instances
 * \param count Number of instances
 */
void audio_spdif_enable_sync(audio_spdif_instance_t *instances[], uint count);

/*! \brief Prepare-only half of audio_spdif_enable_sync
 * \ingroup pico_audio_spdif_multi
 *
 * Primes DMA / IRQ refcounts and marks the instances enabled without
 * starting the SMs; returns the SM mask for the shared PIO block so the
 * caller can perform one combined pio_enable_sm_mask_in_sync (used by the
 * I2S clock-slave path to gate the start on an external LRCLK edge).
 */
uint32_t audio_spdif_enable_sync_prepare(audio_spdif_instance_t *instances[], uint count);

/*! \brief Eagerly program the instance's PIO clock divider for a sample rate
 * \ingroup pico_audio_spdif_multi
 *
 * Recomputes and writes the nominal divider for \p sample_freq and updates
 * the instance's rate bookkeeping (inst->freq) and the IEC 60958-3 channel
 * status rate byte.  The normal divider update is lazy (wrap_consumer_take,
 * gated on a rate mismatch against inst->freq); callers that trim the SM
 * divider behind the library's back (input clock servos) use this to restore
 * nominal even when the rate value is unchanged.
 */
void audio_spdif_apply_pio_frequency(audio_spdif_instance_t *inst, uint32_t sample_freq);

/** \brief Enable/disable DMA-starvation monitoring
 * \ingroup audio_spdif
 *
 * When enabled, the driver counts consumer-empty DMA starts (silence fallback).
 * Intended for dropout diagnostics during active USB streaming.
 */
void audio_spdif_set_starvation_monitoring(bool enabled);

/** \brief Reset DMA-starvation counters
 * \ingroup audio_spdif
 */
void audio_spdif_reset_dma_starvations(void);

/** \brief Get total DMA-starvation events across all instances
 * \ingroup audio_spdif
 */
uint32_t audio_spdif_get_dma_starvations(void);

/** \brief Get DMA-starvation events for one instance index (0..3)
 * \ingroup audio_spdif
 */
uint32_t audio_spdif_get_dma_starvations_instance(uint index);

/** \brief Adjust the DMA IRQ enable reference count for a given IRQ line.
 * \ingroup audio_spdif
 *
 * External subsystems sharing the same DMA IRQ line (e.g. SPDIF RX) must
 * call this to prevent audio_spdif_set_enabled(false) from disabling the
 * entire IRQ line while the external subsystem still needs it.
 *
 * \param dma_irq  DMA IRQ index (0 or 1)
 * \param delta    +1 to hold the IRQ enabled, -1 to release
 */
void audio_spdif_irq_refcount_adjust(uint dma_irq, int delta);

#ifdef __cplusplus
}
#endif

#endif //_AUDIO_SPDIF_H
