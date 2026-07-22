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

// Main-loop service: slaved/idle silence top-up and deferred local resync.
void adat_output_task(void);

// DSP push: encode and enqueue sample_count frames from the 8 finalized
// output channels, already converted to S24 in place in buf_out (see
// output_s24.h).  Called from process_input_block() after the slot gives;
// no-op while the stream is down.
void adat_output_push_block(const out_s24_t (*bufs)[192], uint32_t sample_count);

#endif // PICO_RP2350
