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

// Number of active input channels for the current input source: the USB alt's
// channel count, the I2S input channel count, or the stereo pair for S/PDIF;
// clamped to NUM_INPUT_CHANNELS.  Single source of truth for the DSP pipeline's
// input dimension AND the host-visible status (REQ_GET_STATUS), so the two can
// never disagree about how many inputs are live.
uint8_t active_input_channel_count(void);

// Reset CPU load metering state — called on audio gap detection
void pipeline_reset_cpu_metering(void);

// Shared input sample buffers (filled by active input source)
#if PICO_RP2350
extern float buf_l[192], buf_r[192];
extern float buf_out[NUM_OUTPUT_CHANNELS][192];
// Extra input channels 2..7 for multichannel input modes (inputs 0/1 remain
// buf_l/buf_r, shared with every input source).  Written by the 8-channel USB
// deinterleave AND the multichannel I2S deinterleave; read by the matrix only
// when n_active_inputs > 2 (active_input_channel_count()), so stale contents can
// never leak into stereo or S/PDIF processing.
extern float buf_in_ext[NUM_INPUT_CHANNELS - NUM_STEREO_INPUTS][192];
#else
extern int32_t buf_l[192], buf_r[192];
extern int32_t buf_out[NUM_OUTPUT_CHANNELS][192];
#endif

// Buffer statistics helpers (used by vendor_commands.c and pipeline)
uint get_slot_consumer_fill(uint slot);
// Sample-granular fill in buffer units (0..16) for the clock servo;
// negative when no valid reading exists (type switch, slot not running).
float get_slot_consumer_fill_frac(uint slot);
// Ring stale-audio protection: main-loop idle pump and the pre-flash
// full-silence stamp (see audio_pipeline.c).
void output_rings_idle_pump(void);
void output_rings_prestamp_full_silence(void);
// Live sample-granular fill in frames; false = no valid reading.
bool get_slot_consumer_fill_frames(uint slot, uint32_t *frames);
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
