/*
 * audio_pipeline.h — Input-agnostic DSP pipeline for DSPi
 *
 * Extracted from usb_audio.c: process_input_block() and associated
 * pipeline state (loudness filter state, crossfeed, leveller, preset
 * mute envelope, CPU metering, buffer watermarks).
 */

#ifndef AUDIO_PIPELINE_H
#define AUDIO_PIPELINE_H

#include "pico.h"
#include "config.h"

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

// ---------------------------------------------------------------------------
// Preset/reset mute envelope: observation and control
//
// The envelope is the final output gain applied to every slot (see
// update_preset_mute_envelope in audio_pipeline.c).  These entry points let
// the pipeline-reset bracket in main.c fade the wire to silence BEFORE it
// stops any clock, and prove it got there, instead of arming a mute and
// tearing down in the same breath.  All are main-thread only.
// ---------------------------------------------------------------------------

// Drive the envelope to zero for the next `samples` samples of audio without
// touching `preset_loading` (which doubles as the input prefill-handshake
// signal).  Refreshable: each call restarts the countdown, so a waiter calls
// it every iteration; if the waiter disappears the request expires on its own
// and audio fades back up.
void pipeline_request_soft_mute(uint32_t samples);

// Like pipeline_request_soft_mute() but never shortens a longer outstanding
// request, so a second requester cannot cut another's dwell short.
void pipeline_hold_soft_mute(uint32_t samples);

// Drop the request from pipeline_request_soft_mute() immediately.  Used once
// `preset_loading` has taken ownership of the mute.
void pipeline_clear_soft_mute_request(void);

// True once a processed packet has actually rendered the envelope down to
// zero, i.e. the fade-out is complete in the audio that has been produced,
// not merely armed.
bool pipeline_mute_is_silent(void);

// Force the envelope (and the per-packet ramp's starting value) to zero.  For
// the case where no producer is running: the envelope only advances when a
// packet is processed, so elapsed time alone can never complete a fade.
void pipeline_latch_mute_silence(void);

// Longest configured per-output delay, in samples.  The mute gain is applied
// ahead of the delay lines, so a fade-out is only truly on the wire once this
// many samples have been pushed through behind it.
uint32_t pipeline_max_active_delay_samples(void);

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
// Steady-state slot ring fill (8 buckets of 48 frames): the underrun re-anchor
// target, and the post-restart hold before the fade back up.
#define OUTPUT_RING_TARGET_FILL_FRAMES (SPDIF_CONSUMER_BUFFER_COUNT / 2u * 48u)
// Ring stale-audio protection (see audio_pipeline.c): main-loop idle pump and
// a one-shot full-ring silence stamp.
void output_rings_idle_pump(void);
void output_rings_stamp_silence_ahead(void);
// Stall bracket: while open, a RAM-resident timer alarm keeps every enabled
// ring stamped with framed silence ahead of its reader. Open only where no
// ring is written, reset or torn down. Nestable.
void output_rings_stall_init(void);
void output_rings_stall_begin(void);
void output_rings_stall_end(void);
// NVIC bit of the stall alarm, for the flash blackout keep mask (0 before init).
uint32_t output_rings_stall_nvic_mask(void);
// Fade drain: latch write positions when the mute envelope reaches zero, then
// poll until every enabled reader has played extra_frames past them.
void output_rings_silence_mark_arm(uint32_t extra_frames);
bool output_rings_silence_mark_reached(void);
#if PICO_RP2350
// Signed ring fill in frames (negative = reader ahead of writer); 0 when the
// slot is not running.  RAM-resident; the ADAT output anchors to it.
int32_t output_ring_fill_signed(uint slot);
#endif
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
