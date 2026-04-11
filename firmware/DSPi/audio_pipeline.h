/*
 * audio_pipeline.h — Input-agnostic DSP pipeline for DSPi
 *
 * Extracted from usb_audio.c: process_input_block() and associated
 * pipeline state (loudness filter state, crossfeed, leveller, preset
 * mute envelope, CPU metering, buffer watermarks).
 */

#ifndef AUDIO_PIPELINE_H
#define AUDIO_PIPELINE_H

#include "config.h"
#include "pico/audio.h"

// Generic DSP pipeline entry point — processes buf_l/buf_r through
// loudness, EQ, leveller, crossfeed, matrix mixer, per-output
// EQ/gain/delay, and output encoding.
// buf_l[] and buf_r[] must be filled by the caller before invoking.
void process_input_block(uint32_t sample_count);

// Reset CPU load metering state — called on audio gap detection
void pipeline_reset_cpu_metering(void);

// Shared input sample buffers (filled by active input source)
#if PICO_RP2350
extern float buf_l[192], buf_r[192];
extern float buf_out[NUM_OUTPUT_CHANNELS][192];
#else
extern int32_t buf_l[192], buf_r[192];
extern int32_t buf_out[NUM_OUTPUT_CHANNELS][192];
#endif

// Buffer statistics helpers (used by vendor_commands.c and pipeline)
uint get_slot_consumer_fill(uint slot);
void get_slot_consumer_stats(uint slot, uint *cons_free,
                             uint *cons_prepared, uint *playing);
void reset_buffer_watermarks(void);

// Buffer watermark state (read by vendor GET handlers)
extern uint16_t buffer_stats_sequence;
extern uint8_t spdif_consumer_min_fill_pct[];
extern uint8_t spdif_consumer_max_fill_pct[];
extern uint8_t pdm_dma_min_fill_pct;
extern uint8_t pdm_dma_max_fill_pct;
extern uint8_t pdm_ring_min_fill_pct;
extern uint8_t pdm_ring_max_fill_pct;

#endif // AUDIO_PIPELINE_H
