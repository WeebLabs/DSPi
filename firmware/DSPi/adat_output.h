#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "output_s24.h"

// ----------------------------------------------------------------------------
// ADAT bulk output (RP2350 only)
//
// Streams all 8 post-gain output channels as one ADAT lightpipe frame per
// sample on a dedicated PIO1 state machine.  Only valid at 44.1/48 kHz; the
// stream auto-suspends at higher rates and resumes (if configured enabled)
// when the rate returns.  See Documentation/Features/adat_output_spec.md.
// ----------------------------------------------------------------------------

// Returned by REQ_GET_ADAT_STATUS (8 bytes, packed).
typedef struct __attribute__((packed)) {
    uint8_t  enabled;       // configured enable (persisted intent)
    uint8_t  active;        // stream currently running
    uint8_t  pin;           // configured data GPIO
    uint8_t  rate_ok;       // current sample rate is 44.1/48 kHz
    uint16_t resync_count;  // stream restarts since boot
    uint16_t slip_count;    // emergency local resyncs since boot (should stay 0)
} AdatStatus;

#if PICO_RP2350

// Config intent (vendor commands / flash apply).  Only records the values and
// sets adat_output_config_dirty on change; hardware is reconfigured from the
// main loop inside a pipeline-reset bracket, never in ISR context.
void adat_output_set_config(bool enabled, uint8_t pin);

// Set when config changed and a muted apply is still owed.  Cleared by
// adat_output_resync(); the main loop services it when no other reset path
// (preset load, type switch) has already done so.
extern volatile bool adat_output_config_dirty;

void    adat_output_init(void);            // boot: templates + defaults, no HW
void    adat_output_get_status(AdatStatus *out);
bool    adat_output_config_enabled(void);
uint8_t adat_output_pin(void);
bool    adat_output_is_active(void);

// Wire format of the bulk output (shared PIO SM / DMA / ring; mutually
// exclusive).  TDM8: DATA on its pin, FSYNC on pin + 1, BCLK on its own pin;
// 8 x 32-bit slots, FSYNC-aligned MSB (TI TDM bus mode, e.g. TAD5242).  Both
// setters only record intent and set adat_output_config_dirty.  The enable
// flag (adat_output_set_config) covers whichever format is selected.
#define BULK_FORMAT_ADAT  0
#define BULK_FORMAT_TDM8  1
void    bulk_output_set_format(uint8_t format);
uint8_t bulk_output_format(void);
void    tdm_output_set_pins(uint8_t data_pin, uint8_t bclk_pin);
uint8_t tdm_output_data_pin(void);
uint8_t tdm_output_bclk_pin(void);
// True if `pin` is currently claimed by the enabled bulk output (ADAT data
// pin, or any of the three TDM8 pins).
bool    bulk_output_owns_pin(uint8_t pin);

// Restore-path acceptability check for the ADAT data GPIO (full ownership
// check; ADAT's own claim never blocks).  Defined in vendor_commands.c.
bool adat_pin_acceptable(uint8_t pin);

// Rate policy: record the new rate, stop the stream if it left 44.1/48 kHz.
// Called from perform_rate_change() inside the pipeline-reset bracket; the
// complete_pipeline_reset() that follows restarts the stream via resync when
// the rate is valid again.
void adat_output_on_rate_change(uint32_t freq);

// Stop the stream (outputs draining / disruptive work ahead).
void adat_output_stream_stop(void);

// Full stream (re)start against current config: reconfigure pin/divider,
// prefill the alignment cushion, re-arm DMA, start the SM.  Called at the end
// of every synchronized output restart so the ADAT-to-slot offset is
// re-established at exactly ADAT_ALIGN_LEAD_FRAMES each epoch.
void adat_output_resync(void);

// Mirrors the DMA-starvation monitoring state (usb_audio.c SOF/alt handling):
// while true, silence insertion is slaved 1:1 to slot 0's starvation counter;
// while false (host stream stopped) the cushion free-runs on silence.
void adat_output_set_stream_active(bool active);

// Drop any accumulated slot-0 starvation backlog without inserting silence
// for it.  Called at the end of a flash bracket: during the erase/program
// window the slots keep clocking (selective NVIC blackout) and count
// starvations, but ADAT is not behind by that amount; its own ring lapped
// and it emitted a frame per sample clock the whole time.  Mirroring the
// backlog would therefore shift the ADAT-to-slot offset in the wrong
// direction; the synchronized restart (or the input prefill's) re-canonical-
// izes the offset instead.
void adat_output_rebaseline_starvations(void);

// SPDIF-input clock servo hook: apply the same 16.8 divider the S/PDIF TX SMs
// run (ADAT's PIO clock is likewise 256*Fs).  0 clears the override.
void adat_output_servo_divider(uint32_t div_16_8);

// Main-loop service: slaved/idle silence top-up and deferred local resync.
void adat_output_task(void);

// DSP push: encode and enqueue sample_count frames from the 8 finalized
// output channels, already converted to S24 in place in buf_out (see
// output_s24.h).  Called from process_input_block() after the slot gives;
// no-op while the stream is down.
void adat_output_push_block(const out_s24_t (*bufs)[192], uint32_t sample_count);

#ifdef DSPI_ENABLE_WIFI
// Loan of the (otherwise idle) ring storage to the NET FLAC decoder
// (net_flac.c has the invariants).  Core 0 main-loop context only.
//   adat_output_ring_loanable()  true while the bulk output is neither enabled
//                                nor running and no loan is out
//   adat_output_ring_loan()      returns the ring (and its size) or NULL; while
//                                loaned, adat_output_resync() calls `evict`
//                                before it would start the output, and keeps the
//                                output off unless the loan was returned
//   adat_output_ring_return()    end the loan
bool  adat_output_ring_loanable(void);
void *adat_output_ring_loan(uint32_t *bytes, void (*evict)(void));
void  adat_output_ring_return(void);
#endif

#endif // PICO_RP2350
