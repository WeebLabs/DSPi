/*
 * DSPi Main - USB Audio Device
 * USB Audio with DSP processing and S/PDIF output
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/unique_id.h"
#include "hardware/watchdog.h"
#include "hardware/vreg.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/timer.h"

// Local headers
#include "config.h"
#include "audio_input.h"
#include "audio_pipeline.h"
#include "spdif_input.h"
#include "soft_vcxo.h"
#include "clock_diag.h"
#include "i2s_input.h"
#include "spdif_rx.h"
#include "lg_sound_sync.h"
#include "dac_hw_mute.h"
#include "dsp_pipeline.h"
#include "crossover.h"
#include "flash_clkdiv.h"
#include "flash_storage.h"
#include "pico/audio_i2s_multi.h"
#include "adat_output.h"
#include "adat_input.h"
#include "pdm_generator.h"
#include "siggen.h"
#include "upmix.h"
#include "usb_audio.h"
#include "notify.h"
#include "uart_control.h"
#include "i2c_control.h"
#include "control_surfaces.h"
#include "loudness.h"
#include "crossfeed.h"
#include "leveller.h"
#include "bulk_params.h"
#include "pico/audio_spdif.h"
#include "usb_feedback_controller.h"
#include "usb_descriptors.h"
#include "tusb.h"

// ----------------------------------------------------------------------------
// GLOBAL DEFINITIONS
// ----------------------------------------------------------------------------

// USB audio feedback controller (Q16.16 internal, 10.14 wire)
usb_feedback_ctrl_t fb_ctrl;

// Legacy endpoint-facing values (written by controller, read by sync packet handler)
volatile uint32_t feedback_10_14 = 0;
volatile uint32_t nominal_feedback_10_14 = 0;
volatile bool output_type_switch_in_progress = false;

// Consumer fill level for slot 0 — written by usb_audio.c, read by SOF handler
extern volatile uint8_t spdif0_consumer_fill;

// NOTE: The per-SOF feedback servo tick now lives in the UAC1 class driver
// (usb_audio.c:uac1_driver_sof) which TinyUSB dispatches from the USB IRQ
// whenever an SOF arrives.  No standalone usb_sof_irq() is needed.

volatile int overruns = 0;  // Legacy - kept for compatibility
volatile uint32_t pio_samples_dma = 0;

// Buffer monitoring counters
volatile uint32_t pdm_ring_overruns = 0;   // Core 0 couldn't push (ring full)
volatile uint32_t pdm_ring_underruns = 0;  // Core 1 needed sample but ring empty
volatile uint32_t pdm_dma_overruns = 0;    // Core 1 write caught up to DMA read
volatile uint32_t pdm_dma_underruns = 0;   // Core 1 write fell behind DMA read
volatile uint32_t spdif_overruns = 0;      // USB callback couldn't get buffer (pool full)
volatile uint32_t spdif_underruns = 0;     // USB packet gap > 2ms (consumer likely starved)
volatile uint32_t usb_audio_packets = 0;   // Debug: count of USB audio packets received
volatile uint32_t usb_audio_alt_set = 0;   // Debug: last alt setting selected
volatile uint32_t usb_audio_mounted = 0;   // Debug: audio mounted state
static volatile uint8_t clock_176mhz = 0;

#include "pico/audio.h"
extern struct audio_format audio_format_48k;
extern MatrixMixer matrix_mixer;

// Volume Leveller globals (defined in usb_audio.c)
extern volatile LevellerConfig leveller_config;
extern volatile bool leveller_update_pending;
extern volatile bool leveller_reset_pending;
extern volatile bool leveller_bypassed;
extern LevellerCoeffs leveller_coeffs;
extern LevellerState leveller_state;

static void reset_usb_feedback_loop(void);
static void prepare_pipeline_reset(uint32_t mute_samples);
static void complete_pipeline_reset(void);

// Forward declaration — definition further down.  perform_rate_change() needs
// to check this to avoid tearing down a SPDIF lock-acquisition prefill in
// progress.
static bool spdif_prefilling;

// (Previously a sanitize_mck_multiplier_for_rate() helper lived here that
// force-clamped 96 kHz × 256× to 128× because the old PIO-toggle MCK had
// a 6.25 fractional divider in that combo.  CLK_GPOUTn gives 12.5 there —
// still fractional but stable on real hardware — so the clamp has been
// removed.  See audio_i2s_multi.c MCK section for the full divider table.)

// I2S input role election: the input SM is the clock master only when no
// output slot is I2S; otherwise the lowest-index I2S output drives BCK and
// LRCLK (process_type_switches) and the input slaves to those pads.
// In I2S clock-slave mode an EXTERNAL master owns BCK/LRCLK, so the input
// is never the master (i2s_input_start derives the external role itself).
static bool i2s_input_should_be_master(void) {
    if (i2s_clock_mode == I2S_CLOCK_MODE_SLAVE) return false;
    extern uint8_t output_types[];
    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        if (output_types[i] == OUTPUT_TYPE_I2S) return false;
    }
    return true;
}

// Rewrite every SPDIF-type slot's PIO divider to nominal for the given rate.
// The input clock servos (SPDIF input, I2S slave) trim these SMs directly and
// never update the library's inst->freq bookkeeping, so the library's lazy
// wrap_consumer_take update is blind to the trim and won't fire when the
// pipeline rate value is unchanged; without an explicit restore the trim
// outlives its servo.  That matters beyond an off-nominal carrier: ADAT
// resyncs to nominal inside complete_pipeline_reset()/enable_outputs_in_sync()
// (the input servo dividers read 0 once the input is stopped), and a
// nominal-ADAT vs trimmed-SPDIF split makes ADAT drift against the slots it
// mirrors, eroding its alignment cushion until the slip machinery forces a
// resync.  All SPDIF slots get the same value, so inter-slot alignment is
// unaffected.  I2S-type slots are excluded: their SMs run other programs
// (extclk slots at divider 1.0) and are rebuilt by their own paths.
static void restore_nominal_spdif_dividers(uint32_t sample_freq) {
    extern uint8_t output_types[];
    extern audio_spdif_instance_t *spdif_instance_ptrs[];
    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        if (output_types[i] == OUTPUT_TYPE_SPDIF && spdif_instance_ptrs[i]) {
            audio_spdif_apply_pio_frequency(spdif_instance_ptrs[i], sample_freq);
        }
    }
}

// defer_output_to_input_prefill: when true, the caller's input prefill handshake
// (the main-loop I2S block) will drain, re-enable outputs in sync, and own the
// DAC-mute release after this returns, so perform_rate_change() must NOT run
// complete_pipeline_reset() (which would re-enable + release the mute early and
// double-drain).  Set by I2S bring-up / runtime I2S rate change / source-switch
// INTO I2S — the last of which is why this is an explicit parameter rather than
// an active_input_source check (active is still the OLD source there).
static void perform_rate_change(uint32_t new_freq, bool defer_output_to_input_prefill) {
    switch (new_freq) { case 44100: case 48000: case 96000: break; default: new_freq = 44100; }

    // Engage mute and wait for Core 1 EQ worker to drain before touching
    // filter coefficients or PIO dividers. Without this bracket, old-rate
    // consumer-pool buffers would play at the new PIO bit-clock for ~16ms
    // (audible pitch shift + resync click).
    prepare_pipeline_reset(PRESET_MUTE_SAMPLES);

    // I2S input bracket: stop across the rate change, restart at the new
    // rate below. Covers the clock-master divider change and re-phases the
    // slave role against the restarted TX master.
    bool i2s_input_was_running =
        (active_input_source == INPUT_SOURCE_I2S &&
         i2s_input_get_state() != I2S_INPUT_INACTIVE);
    if (i2s_input_was_running) {
        i2s_input_stop();
    }

    // Update the audio format so pico_audio_spdif can update the PIO divider
    audio_format_48k.sample_freq = new_freq;

    // Keep the global rate coherent for device-initiated changes (SPDIF
    // lock, I2S rate select). The USB host path already wrote it; for the
    // other sources everything downstream (loudness/crossfeed/leveller
    // recompute handlers, REQ_GET_STATUS) reads audio_state.freq.
    audio_state.freq = new_freq;

#if PICO_RP2350
    // RP2350: 307.2MHz fixed (VCO 1536 / 5 / 1) — no clock switching
#else
    // RP2040: 307.2MHz fixed (VCO 1536 / 5 / 1) — no clock switching
#endif
    // Reset sync
    extern volatile bool sync_started;
    extern volatile uint64_t total_samples_produced;
    sync_started = false;
    total_samples_produced = 0;

    // Pre-compute nominal feedback and reset controller
    nominal_feedback_10_14 = ((uint64_t)new_freq << 14) / 1000;
    feedback_10_14 = nominal_feedback_10_14;
    reset_usb_feedback_loop();

    dsp_recalculate_all_filters((float)new_freq);
    loudness_recompute_pending = true;
    crossfeed_update_pending = true;  // Recalculate crossfeed coefficients for new sample rate
    leveller_update_pending = true;   // Recalculate leveller coefficients for new sample rate
    psybass_update_pending = true;    // Recalculate psybass coefficients for new sample rate
#if PICO_RP2350
    upmix_update_pending = true;      // Recalculate upmixer coefficients for new sample rate
#endif
    pdm_update_clock(new_freq);

#if PICO_RP2350
    // ADAT rate policy: suspend above 48 kHz, divider refresh otherwise.  The
    // complete_pipeline_reset() below restarts the stream when the rate is
    // valid and the output is configured enabled.
    adat_output_on_rate_change(new_freq);
    // ADAT input mirrors the policy: master mode retunes its RX divider and
    // re-syncs (or parks above 48 kHz); slave mode is a no-op (the detected
    // wire rate drove this change, so its divider is already nominal).
    adat_input_on_rate_change(new_freq);
#endif

    // Atomically update all I2S instances and restart in sync (avoids brief
    // master/slave divider mismatch from lazy per-instance callbacks)
    audio_i2s_update_all_frequencies(new_freq);

    // SPDIF TX dividers: restore nominal eagerly, even when new_freq equals
    // the old rate.  The library's lazy update is gated on a rate mismatch
    // against inst->freq, which an input clock servo's trim never creates,
    // so an equal-rate call through here (e.g. a source switch away from a
    // servoed input) would otherwise leave the trim in place.  The mute
    // bracket above covers the write, and complete_pipeline_reset() (or the
    // caller's prefill block) drains and restarts in sync afterwards.
    restore_nominal_spdif_dividers(new_freq);

    // Update MCK frequency for new sample rate (if enabled).  No per-rate
    // multiplier sanitization — CLK_GPOUTn handles every Fs × multiplier
    // combination this firmware supports.
    extern bool i2s_mck_enabled;
    extern uint16_t i2s_mck_multiplier;
    if (i2s_mck_enabled) {
        audio_i2s_mck_update_frequency(new_freq, i2s_mck_multiplier);
    }

    // Drain all consumer pools (old-rate audio) and restart outputs in sync
    // at the new PIO divider. The SPDIF wrap_consumer_take path would
    // otherwise update the divider lazily mid-stream with old-rate audio
    // still queued in each consumer pool.
    //
    // Exception: if the SPDIF lock-acquisition block is currently prefilling
    // the pools (spdif_prefilling == true), complete_pipeline_reset() would
    // drain the half-filled pools and re-enable outputs against an empty
    // pool — the exact underrun/pop the prefill handshake exists to prevent.
    // Let the prefill block's own enable_outputs_in_sync() restart outputs
    // when the 50 % fill threshold is reached instead.
    //
    // Likewise skip when the eventual input is I2S (defer_output_to_input_prefill):
    // the main-loop I2S prefill block will drain, re-enable in sync, and own the
    // DAC-mute release.  This is what makes the source-switch-INTO-I2S case
    // correct (active_input_source is still the OLD source here, so
    // complete_pipeline_reset()'s own I2S guard could not fire) and avoids a
    // redundant drain for the runtime / bring-up I2S cases.
    if (!spdif_prefilling && !defer_output_to_input_prefill) {
        complete_pipeline_reset();
    }

    // Restart I2S input at the new rate (master divider derives from
    // audio_state.freq inside i2s_input_start)
    if (i2s_input_was_running && active_input_source == INPUT_SOURCE_I2S) {
        i2s_input_start(i2s_input_should_be_master());
    }
}

// Bring up I2S input and hand off to the main-loop prefill handshake.
//
// Precondition: the caller has already engaged the mute (prepare_pipeline_reset)
// and set active_input_source = INPUT_SOURCE_I2S. This applies the selected
// rate and MCK, starts the input, and deliberately does NOT enable outputs:
// it leaves preset_loading set so the main-loop I2S block drains the outputs,
// fills the consumer pools to 50%, and starts them in sync (mirroring the
// SPDIF lock-acquisition prefill, minus the lock wait).
//
// When the selected rate differs from the live pipeline rate, perform_rate_change
// is used: it must run its full reset so SPDIF TX dividers are reprogrammed
// (they update via instance teardown/restart). That briefly enables outputs
// emitting muted silence; the main-loop block then re-drains and prefills.
static void i2s_input_bringup_prefill(void) {
    bool master = i2s_input_should_be_master();

    // Clock-slave mode: the external master defines the rate; stay at the
    // current pipeline rate until the lock machinery detects the real one
    // (i2s_slave_check_rate_change arms the change before the prefill).
    if (i2s_clock_mode != I2S_CLOCK_MODE_SLAVE &&
        i2s_input_rate != audio_state.freq) {
        // Eventual input is I2S; the main-loop I2S prefill block owns the
        // output drain/re-enable/mute-release, so defer rather than letting
        // complete_pipeline_reset() release the mute before that drain.
        perform_rate_change(i2s_input_rate, true);
    }

    extern bool i2s_mck_enabled;
    extern uint16_t i2s_mck_multiplier;
    if (i2s_mck_enabled && master) {
        // External source may need MCK; master role implies no I2S outputs,
        // so MCK is not already running for an output. Divider before enable.
        audio_i2s_mck_update_frequency(i2s_input_rate, i2s_mck_multiplier);
        audio_i2s_mck_set_enabled(true);
    }

    i2s_input_start(master);
}

// Reset an SPDIF instance's software queue state so it can restart in phase with
// other SPDIF instances after output-type switching.
static void spdif_reset_consumer_pipeline(audio_spdif_instance_t *inst) {
    // Return any partially filled producer->consumer staging buffer.
    if (inst->connection.current_consumer_buffer) {
        queue_free_audio_buffer(inst->consumer_pool, inst->connection.current_consumer_buffer);
        inst->connection.current_consumer_buffer = NULL;
    }
    inst->connection.current_consumer_buffer_pos = 0;

    // Drain prepared buffers back to free so we don't resume with stale backlog.
    for (;;) {
        audio_buffer_t *ab = get_full_audio_buffer(inst->consumer_pool, false);
        if (!ab) break;
        queue_free_audio_buffer(inst->consumer_pool, ab);
    }

    // Restart IEC60958 block framing from position 0 on next DMA prime.
    inst->subframe_position = 0;
}

static void i2s_reset_consumer_pipeline(audio_i2s_instance_t *inst) {
    // Return any partially filled producer->consumer staging buffer.
    if (inst->connection.current_consumer_buffer) {
        queue_free_audio_buffer(inst->consumer_pool, inst->connection.current_consumer_buffer);
        inst->connection.current_consumer_buffer = NULL;
    }
    inst->connection.current_consumer_buffer_pos = 0;

    // Drain prepared buffers back to free so we don't resume with stale backlog.
    for (;;) {
        audio_buffer_t *ab = get_full_audio_buffer(inst->consumer_pool, false);
        if (!ab) break;
        queue_free_audio_buffer(inst->consumer_pool, ab);
    }
}

// Forward declarations (defined later in this file)
static void prepare_pipeline_reset(uint32_t mute_samples);
static void complete_pipeline_reset(void);
static void drain_and_disable_outputs(void);
static void enable_outputs_in_sync(void);
static uint32_t samples_for_duration_ms(uint32_t sample_rate_hz, uint32_t duration_ms);

// SPDIF input prefill: outputs disabled while consumer buffers fill to 50%
static bool spdif_prefilling = false;

// I2S input prefill: same handshake as SPDIF (drain outputs, fill consumer
// pools to 50%, then enable in sync) but with no lock wait, since the I2S
// input is synchronous to our own clock domain and runs as soon as started.
// Cleared by prepare_pipeline_reset() so every disruptive op restarts the
// handshake cleanly. Driven by the main-loop I2S block.
static bool i2s_prefilling = false;

// ADAT input prefill: SPDIF-style lock-gated handshake in both clock modes
// (frame sync must be verified before audio flows). Cleared by
// prepare_pipeline_reset(). Driven by the main-loop ADAT block (RP2350).
static bool adat_prefilling = false;

// ---------------------------------------------------------------------------
// process_type_switches — unified output type transition handler
//
// Handles any combination of SPDIF↔I2S slot changes atomically with correct
// I2S master/slave election.  Used by three callers:
//   1. Vendor command (output_type_change_mask from USB ISR)
//   2. Boot (slots loaded from preset that need I2S)
//   3. Preset load (slots whose type differs between old and new preset)
//
// Two-pass approach:
//   Pass 1: Teardown all outgoing types (I2S→SPDIF or SPDIF→I2S transitions)
//   Pass 2: Setup new types with master election, then restart all in sync
//
// change_mask: bitmask of slots that need a type change (bit N = slot N)
// new_types[]: desired output type per slot (only slots in change_mask are read)
// ---------------------------------------------------------------------------
static void process_type_switches(uint8_t change_mask, const uint8_t new_types[]) {
    if (change_mask == 0) return;

    extern uint8_t output_types[];
    extern audio_spdif_instance_t *spdif_instance_ptrs[];
    extern audio_i2s_instance_t *i2s_instance_ptrs[];
    extern uint8_t output_pins[];
    extern struct audio_buffer_pool *producer_pools[];
    extern struct audio_buffer_pool *slot_consumer_pools[];  // shared per-slot static pools
    extern bool i2s_mck_enabled;
    extern uint16_t i2s_mck_multiplier;

    uint8_t current_types[NUM_SPDIF_INSTANCES];
    uint8_t target_types[NUM_SPDIF_INSTANCES];
    memcpy(current_types, output_types, NUM_SPDIF_INSTANCES);
    memcpy(target_types, current_types, NUM_SPDIF_INSTANCES);

    // Snapshot requested targets for this batch so ISR updates that arrive
    // mid-switch are handled in the NEXT batch.
    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        if (change_mask & (1u << i)) {
            uint8_t req = new_types[i];
            if (req <= OUTPUT_TYPE_I2S) {
                target_types[i] = req;
            }
        }
    }

    // Deterministic master policy: lowest-index active I2S slot is master.
    // In I2S clock-slave mode NO slot is master; every I2S slot runs the
    // external-clock program against the externally driven BCK/LRCLK.
    const bool want_extclk = i2s_slave_mode_active();
    int target_master_slot = -1;
    if (!want_extclk) {
        for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
            if (target_types[i] == OUTPUT_TYPE_I2S) {
                target_master_slot = i;
                break;
            }
        }
    }

    bool any_change = false;
    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        if (target_types[i] != current_types[i]) {
            any_change = true;
            break;
        }
    }
    // Same-type I2S slots still count as a change when their live clocking
    // (master election / external-clock mode / BCK pin) no longer matches
    // the target; clock-mode and input-source transitions, and bulk/preset
    // restores that install a new clock pin/pin mode without a type change,
    // call in with an unchanged type map precisely to trigger this rebuild.
    // The effective BCK tracks the clock-pin mode: SPLIT + slave clocking
    // waits on the slave pair instead of the master pair.
    if (!any_change) {
        for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
            if (target_types[i] != OUTPUT_TYPE_I2S) continue;
            audio_i2s_instance_t *inst = i2s_instance_ptrs[i];
            if (!inst || !inst->consumer_pool) continue;
            if (inst->external_clock != want_extclk ||
                inst->clock_master != (i == target_master_slot) ||
                inst->clock_pin_base != i2s_effective_bck_pin()) {
                any_change = true;
                break;
            }
        }
    }
    if (!any_change) return;

    output_type_switch_in_progress = true;
    __dmb();
    const bool usb_irq_was_enabled = irq_is_enabled(USBCTRL_IRQ);
    irq_set_enabled(USBCTRL_IRQ, false);

    usb_audio_drain_ring();
    prepare_pipeline_reset(PRESET_MUTE_SAMPLES);

    // If the caller entered with the DAC hardware mute RELEASED (e.g. a
    // BCK-only restore rebuild discovered after a completed reset already
    // released it on the USB path), the assert above armed a fresh hold;
    // wait it out so the teardown below never stops clocks before the DAC's
    // analog ramp.  Instant for every pre-gated caller (hold already
    // elapsed), so existing paths are unaffected.
    while (!dac_hw_mute_hold_elapsed()) tight_loop_contents();

    // Suspend SPDIF RX across the type-switch window.  The DMA IRQ disable
    // below kills DMA_IRQ_1 servicing (which RX shares with SPDIF TX), and
    // the alarm-pool decode-timeout alarms in pico_spdif_rx use a separate
    // timer IRQ that can fire mid-transition and access PIO/DMA state we
    // are mutating.  Stop cleanly here; restart at the end if it was running.
    // If a caller already stopped RX (e.g. preset_load_pending across the
    // flash blackout), state==INACTIVE and we leave it that way — caller
    // is responsible for restart.
    bool rx_was_running = (input_source_is_spdif(active_input_source) &&
                           spdif_input_get_state() != SPDIF_INPUT_INACTIVE);
    if (rx_was_running) {
        spdif_input_stop();
        spdif_prefilling = false;
    }

    // Suspend I2S input too: type switches can move BCK/LRCLK ownership
    // between PIO blocks and change the input's master/slave role.  Restart
    // with a re-elected role at the end.
    bool i2s_rx_was_running = (active_input_source == INPUT_SOURCE_I2S &&
                               i2s_input_get_state() != I2S_INPUT_INACTIVE);
    if (i2s_rx_was_running) {
        i2s_input_stop();
    }

    // Prevent DMA IRQ handlers from touching registries while we teardown/setup
    // instances and mutate hardware ownership.
    const uint spdif_dma_irq_num = DMA_IRQ_0 + PICO_AUDIO_SPDIF_DMA_IRQ;
    const uint i2s_dma_irq_num = DMA_IRQ_0 + PICO_AUDIO_I2S_DMA_IRQ;
    irq_set_enabled(spdif_dma_irq_num, false);
    if (i2s_dma_irq_num != spdif_dma_irq_num) {
        irq_set_enabled(i2s_dma_irq_num, false);
    }

    // Quiesce ALL currently active outputs before any teardown/setup work.
    // Type switching can repurpose SMs/channels and master-clock ownership;
    // doing that while other slots still run DMA/PIO is unsafe and can crash.
    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        if (current_types[i] == OUTPUT_TYPE_I2S) {
            audio_i2s_instance_t *inst = i2s_instance_ptrs[i];
            if (!inst || !inst->consumer_pool) continue;
            if (inst->enabled) {
                audio_i2s_set_enabled(inst, false);
            }
            dma_irqn_set_channel_enabled(inst->dma_irq, inst->dma_channel, false);
            dma_channel_abort(inst->dma_channel);
            if (inst->playing_buffer) {
                give_audio_buffer(inst->consumer_pool, inst->playing_buffer);
                inst->playing_buffer = NULL;
            }
            dma_irqn_acknowledge_channel(inst->dma_irq, inst->dma_channel);
        } else {
            audio_spdif_instance_t *inst = spdif_instance_ptrs[i];
            if (!inst || !inst->consumer_pool) continue;
            if (inst->enabled) {
                audio_spdif_set_enabled(inst, false);
            }
            dma_irqn_set_channel_enabled(inst->dma_irq, inst->dma_channel, false);
            dma_channel_abort(inst->dma_channel);
            if (inst->playing_buffer) {
                give_audio_buffer(inst->consumer_pool, inst->playing_buffer);
                inst->playing_buffer = NULL;
            }
            dma_irqn_acknowledge_channel(inst->dma_irq, inst->dma_channel);
        }
    }

    // ---- Pass 1: Teardown outgoing types ----
    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        if (target_types[i] == current_types[i]) continue;  // No type change

        if (current_types[i] == OUTPUT_TYPE_I2S) {
            // I2S → SPDIF: teardown the I2S instance (releases DMA channel i
            // and PIO SM i; the SPDIF instance is fully re-set-up in Pass 2).
            audio_i2s_teardown(i2s_instance_ptrs[i]);
        } else {
            // SPDIF → I2S: fully tear down the SPDIF instance so it releases
            // DMA channel i (and PIO SM i) for this slot's I2S instance to
            // claim in Pass 2.  SPDIF and I2S now share one DMA channel per
            // output slot (channel == slot index), freeing the high channels
            // for input use.
            audio_spdif_teardown(spdif_instance_ptrs[i]);
        }
    }

    // ---- Pass 2: Setup final types and enforce deterministic master/slave roles ----
    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        bool had_i2s = (current_types[i] == OUTPUT_TYPE_I2S);
        bool want_i2s = (target_types[i] == OUTPUT_TYPE_I2S);

        if (want_i2s) {
            bool want_master = (i == target_master_slot);
            bool need_rebuild = !had_i2s;

            if (had_i2s) {
                audio_i2s_instance_t *inst = i2s_instance_ptrs[i];
                if (!inst->consumer_pool || inst->clock_master != want_master ||
                    inst->external_clock != want_extclk ||
                    inst->clock_pin_base != i2s_effective_bck_pin()) {
                    if (inst->enabled) {
                        audio_i2s_set_enabled(inst, false);
                    }
                    audio_i2s_teardown(inst);
                    need_rebuild = true;
                }
            }

            if (need_rebuild) {
                audio_i2s_config_t i2s_cfg = {
                    .data_pin = output_pins[i],
                    .clock_pin_base = i2s_effective_bck_pin(),
                    // Shared with this slot's S/PDIF instance: one DMA channel
                    // per output slot (channel index == slot index).  The
                    // outgoing S/PDIF instance released channel i in Pass 1.
                    .dma_channel = i,
                    .pio_sm = i,
                    .pio = PICO_AUDIO_SPDIF_PIO,
                    .dma_irq = PICO_AUDIO_I2S_DMA_IRQ,
                    .clock_master = want_master,
                    .external_clock = want_extclk,
                };
                audio_i2s_setup(i2s_instance_ptrs[i], &audio_format_48k, &i2s_cfg);
                // Re-formats the slot's shared static consumer pool for I2S (no alloc).
                audio_i2s_connect_extra(i2s_instance_ptrs[i], producer_pools[i],
                                        false, slot_consumer_pools[i], NULL);
                if (had_i2s) {
                    printf("Slot %d %s I2S master\n", i, want_master ? "promoted to" : "demoted to");
                }
            }
        } else if (had_i2s) {
            // I2S → SPDIF: the I2S teardown in Pass 1 released DMA channel i
            // and PIO SM i.  Fully re-set-up the S/PDIF instance on the same
            // channel/SM (mirrors boot), reclaiming the shared resources.
            audio_spdif_instance_t *spdif_inst = spdif_instance_ptrs[i];
            audio_spdif_config_t spdif_cfg = {
                .pin = output_pins[i],
                .dma_channel = i,
                .pio_sm = i,
                .pio = PICO_AUDIO_SPDIF_PIO,
                .dma_irq = PICO_AUDIO_SPDIF_DMA_IRQ,
            };
            audio_spdif_setup(spdif_inst, &audio_format_48k, &spdif_cfg);
            // Re-formats the slot's shared static consumer pool back to S/PDIF
            // (re-points buffers + re-fills IEC-60958 framing), re-wires the
            // connection, and re-applies the current sample-rate divider via
            // update_pio_frequency — no alloc/free. The pool was last formatted
            // for I2S.
            audio_spdif_connect_extra(spdif_inst, producer_pools[i], false,
                                      slot_consumer_pools[i], NULL);
            memset(i2s_instance_ptrs[i], 0, sizeof(audio_i2s_instance_t));
        }
    }

    // Regenerate default channel names for slots whose type is changing.
    // Only overwrite names that still match the OLD default — user customisations
    // are preserved by string-inequality.  RAM-only; persisted on REQ_PRESET_SAVE.
    for (int slot = 0; slot < NUM_SPDIF_INSTANCES; slot++) {
        if (current_types[slot] == target_types[slot]) continue;
        for (int side = 0; side < 2; side++) {
            int ch = CH_OUT_1 + slot * 2 + side;
            char old_default[PRESET_NAME_LEN];
            char new_default[PRESET_NAME_LEN];
            get_default_channel_name(ch, active_input_source, current_types, old_default);
            get_default_channel_name(ch, active_input_source, target_types, new_default);
            if (strcmp(old_default, new_default) == 0) continue;
            if (strcmp(channel_names[ch], old_default) != 0) continue;
            memcpy(channel_names[ch], new_default, PRESET_NAME_LEN);
            notify_param_write(
                (uint16_t)(offsetof(WireBulkParams, channel_names.names) + ch * WIRE_NAME_LEN),
                WIRE_NAME_LEN, channel_names[ch]);
        }
    }

    memcpy(output_types, target_types, NUM_SPDIF_INSTANCES);

    // Start/stop MCK based on whether any slot is now I2S, or I2S is the
    // active input (the external source may need MCK regardless of the
    // output types).  In I2S clock-slave mode MCK is forced off: a locally
    // generated MCK would be asynchronous to the external BCK/LRCLK, which
    // is invalid for downstream converters.
    bool any_i2s = (active_input_source == INPUT_SOURCE_I2S);
    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        if (output_types[i] == OUTPUT_TYPE_I2S) { any_i2s = true; break; }
    }
    if (want_extclk) {
        audio_i2s_mck_set_enabled(false);
    } else if (any_i2s && i2s_mck_enabled) {
        // Set divider BEFORE enabling the GPOUTn block so MCK starts at the
        // correct frequency.  Reversed order would briefly run MCK at the
        // previous divider, causing a transient PLL relock chirp on
        // connected DACs.  Matches the REQ_SET_MCK_ENABLE vendor command
        // order.  No multiplier sanitization needed — CLK_GPOUTn handles
        // every Fs × multiplier combination.
        audio_i2s_mck_update_frequency(audio_state.freq, i2s_mck_multiplier);
        audio_i2s_mck_set_enabled(true);
    } else if (!any_i2s) {
        audio_i2s_mck_set_enabled(false);
    }

    // Restart all outputs in sync (handles both SPDIF and I2S instances)
    complete_pipeline_reset();

    __dmb();
    output_type_switch_in_progress = false;
    if (usb_irq_was_enabled) {
        irq_set_enabled(USBCTRL_IRQ, true);
    }

    // Restart SPDIF RX if we suspended it above.  If active_input_source
    // changed during the switch (rare — driven by deferred input_source
    // change), skip restart — the input-source handler will manage it.
    if (rx_was_running &&
        input_source_is_spdif(active_input_source) &&
        !input_source_change_pending) {
        spdif_input_start();
    }

    // Restart I2S input with a freshly elected role: the final output types
    // decide whether the input SM is the clock master (no I2S outputs) or
    // slaves to the new output master's BCK/LRCLK.
    if (i2s_rx_was_running &&
        active_input_source == INPUT_SOURCE_I2S &&
        !input_source_change_pending) {
        i2s_input_start(i2s_input_should_be_master());
    }

    printf("Type switch complete: mask=0x%02x\n", change_mask);
}

// Rebuild every I2S output slot's clocking without changing types: called
// when a transition crosses the I2S clock-slave boundary (mode change, or a
// source switch into/out of slave-clocked I2S), so the slots swap between
// the internal clkout/dataout programs and the external-clock program, and
// after bulk/preset restores, which can install a new i2s_bck_pin with an
// unchanged type map.  process_type_switches detects the clocking mismatch
// (master election / external-clock mode / BCK pin) against the live
// instances and does all the heavy lifting (quiesce, rebuild, MCK policy,
// synchronized restart); with nothing mismatched it returns without
// disruption.  No-op when no slot is I2S.
static void rebuild_i2s_output_clocking(void) {
    extern uint8_t output_types[];
    uint8_t mask = 0;
    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        if (output_types[i] == OUTPUT_TYPE_I2S) mask |= (uint8_t)(1u << i);
    }
    if (mask) process_type_switches(mask, output_types);
}

// ---------------------------------------------------------------------------
// process_pin_changes — deferred output data-pin reassignment for SPDIF/I2S
//
// Reconfigures the data GPIO of one or more output slots, then restarts ALL
// slots in sync via complete_pipeline_reset() so inter-slot sample alignment
// is preserved (CLAUDE.md invariant) — the moved slot re-enters in phase with
// the rest.  Structure mirrors process_type_switches: mute, quiesce, mutate
// pins while disabled, synchronized restart.  Deferred from the vendor command
// so the soft mute + DAC hardware-mute hold run without blocking the USB ISR.
//
// mask: bit N = slot N's output_pins[N] differs from its live data pin.
// ---------------------------------------------------------------------------
static void process_pin_changes(uint8_t mask) {
    if (mask == 0) return;

    extern uint8_t output_types[];
    extern uint8_t output_pins[];
    extern audio_spdif_instance_t *spdif_instance_ptrs[];
    extern audio_i2s_instance_t *i2s_instance_ptrs[];

    usb_audio_drain_ring();
    prepare_pipeline_reset(PRESET_MUTE_SAMPLES);

    // Suspend SPDIF RX across the reconfiguration: its decode-timeout alarm can
    // fire mid-mutation and touch shared DMA/PIO state.  Restart at the end if
    // it was running.  Mirrors process_type_switches.
    bool rx_was_running = (input_source_is_spdif(active_input_source) &&
                           spdif_input_get_state() != SPDIF_INPUT_INACTIVE);
    if (rx_was_running) {
        spdif_input_stop();
        spdif_prefilling = false;
    }

    // Disable every slot before mutating any pin — change_pin asserts !enabled.
    drain_and_disable_outputs();

    // Apply the new pin to each flagged slot while it is disabled.  Skip if it
    // already matches the live pin (e.g. a set-then-revert before this ran).
    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        if (!(mask & (1u << i))) continue;
        if (output_types[i] == OUTPUT_TYPE_I2S) {
            audio_i2s_instance_t *inst = i2s_instance_ptrs[i];
            if (inst && inst->consumer_pool && inst->data_pin != output_pins[i])
                audio_i2s_change_data_pin(inst, output_pins[i]);
        } else {
            audio_spdif_instance_t *inst = spdif_instance_ptrs[i];
            if (inst && inst->consumer_pool && inst->pin != output_pins[i])
                audio_spdif_change_pin(inst, output_pins[i]);
        }
    }

    // Synchronized restart of all slots (also resets USB feedback and releases
    // the DAC hardware mute on USB input; for SPDIF input the lingering
    // preset_loading hands release to the lock-acquisition prefill block).
    // I2S input needs no suspension here: output data-pin moves never change
    // its master/slave role, and complete_pipeline_reset's trailing
    // i2s_input_resync() re-phases a slave against the restarted TX master.
    complete_pipeline_reset();

    // Restart SPDIF RX if we suspended it (unless the source changed mid-op).
    if (rx_was_running &&
        input_source_is_spdif(active_input_source) &&
        !input_source_change_pending) {
        spdif_input_start();
    }

    printf("Pin change complete: mask=0x%02x\n", mask);
}

// Reset USB async feedback loop state after disruptive output pipeline events
// (type switch, global resync, stream activation).
static void reset_usb_feedback_loop(void) {
    fb_ctrl_reset(&fb_ctrl, nominal_feedback_10_14 << 2);
    feedback_10_14 = nominal_feedback_10_14;
}

// ---------------------------------------------------------------------------
// Two-phase pipeline reset API
//
// Any operation that disrupts output pipeline phase alignment (stream
// start/restart, output type switch) must bracket the disruptive work:
//
//   prepare_pipeline_reset(PRESET_MUTE_SAMPLES);
//   ... type-specific teardown / setup ...
//   complete_pipeline_reset();
//
// For simple cases (stream restart) with no work between phases, call
// both back-to-back.  Only non-disruptive RAM-only operations should call
// prepare_pipeline_reset() alone.
// ---------------------------------------------------------------------------

// Phase 1: prepare for disruptive pipeline work.
// Waits for Core 1 EQ worker to finish, arms the audio soft-mute
// envelope, then asserts the DAC hardware mute (if configured) and
// holds for the user-configured hold_ms.  The order is intentional:
// the software mute state is visible before disruptive work begins,
// and the hardware mute gives the DAC chip's analog output time to
// ramp down before the caller stops BCK/LRCLK.  Together they cover
// both failure modes — data-path discontinuity and analog DC-step on
// clock cessation.  Hardware mute is a no-op when disabled (zero-cost
// when not configured).
static void prepare_pipeline_reset(uint32_t mute_samples) {
    if (core1_mode == CORE1_MODE_EQ_WORKER) {
        while (core1_eq_work.work_ready && !core1_eq_work.work_done)
            tight_loop_contents();
        __dmb();
    }
    preset_mute_counter = mute_samples;
    preset_loading = true;
    // Cancel any in-progress I2S/ADAT prefill: this disruptive op will
    // re-trigger the handshake via preset_loading once it completes. (No
    // effect on the SPDIF path, which manages spdif_prefilling at its own
    // call sites.)
    i2s_prefilling = false;
    adat_prefilling = false;
    __dmb();
    dac_hw_mute_assert();

    // preset_loading must outlive any pending hardware-mute hold.  The soft
    // envelope (update_preset_mute_envelope) auto-clears preset_loading after
    // preset_mute_counter samples, and the SPDIF/ADAT/I2S-slave prefill
    // blocks gate their drain on dac_hw_mute_hold_elapsed() while their poll
    // keeps feeding the pipeline; after a fast re-lock (or a pin-swap input
    // restart) with a long configured hold, a small counter expires before
    // the drain is allowed to run, the prefill handshake never fires, and
    // dac_hw_mute_release() is never called: stuck mute.  So when a hold is
    // still pending here, floor the counter to outlast it (hold + margin for
    // the post-hold poll iterations).  Reachable only with a fresh hold: the
    // synchronous reset handlers pre-gate on pipeline_reset_ready(), so by
    // the time their body runs prepare, the hold has elapsed and this is
    // skipped — USB-path mute durations are unchanged.  The flash brackets
    // are covered separately: their settle loop waits the hold out before
    // the blackout when the source is streaming, and flash_write_sector()'s
    // trailing re-arm (flash_mute_hold_samples) applies this same floor for
    // the non-streaming case, where the hold is still pending at completion.
    if (!dac_hw_mute_hold_elapsed()) {
        uint32_t floor_samples = samples_for_duration_ms(
            audio_state.freq,
            (uint32_t)dac_hw_mute_hold_ms() + PRESET_MUTE_HOLD_MARGIN_MS);
        if (preset_mute_counter < floor_samples) {
            preset_mute_counter = floor_samples;
        }
    }
}

// Non-blocking pre-clock-stop barrier for the synchronous reset handlers.
//
// These handlers (preset load, factory reset, bulk params, rate change,
// stream restart, output-type switch, output-pin change, input source switch)
// stop the output
// clocks in the SAME main-loop iteration that they start — so they cannot
// busy-wait the DAC hardware-mute hold without stalling the loop (starving
// usb_audio_drain_ring() / SPDIF polling).  Instead each such handler gates
// its body on this helper:
//
//     if (some_pending && pipeline_reset_ready()) {
//         some_pending = false;
//         ... usb_audio_drain_ring(); prepare_pipeline_reset(N); ...
//         ... disruptive work + complete_pipeline_reset() ...
//     }
//
// While the hold has not elapsed the body is skipped and the pending flag is
// left set, so the main loop falls through and keeps servicing audio; the
// handler retries next iteration.  dac_hw_mute_assert() is idempotent (it does
// not re-arm/extend the hold once asserted), so calling the gate every
// iteration during the wait is safe and cheap and the deadline never slips.
// Returns true immediately when the feature is disabled or hold_ms == 0.
//
// IMPORTANT — the gate engages ONLY the DAC hardware mute, NOT the soft-mute
// flag (preset_loading).  preset_loading also triggers the SPDIF lock-
// acquisition block, which runs EARLIER in the main loop; if the gate held
// preset_loading true across the wait, that block would fire
// drain_and_disable_outputs() on the very iteration the hold elapses — before
// the handler body's complete_pipeline_reset() — leaving spdif_prefilling set
// so the prefill path re-enables a second time with no teardown between, which
// double-starts the SPDIF DMA and breaks inter-slot alignment.  By engaging
// only the hardware mute here, the body's own prepare_pipeline_reset() sets
// preset_loading at the proper time (right before teardown) exactly as it did
// before this feature, so the SPDIF block keeps its original ordering (it
// reacts on the NEXT iteration, after the body's complete).  The hardware mute
// alone is what must lead the clock-stop; every other output keeps playing
// until the body runs.  The body's prepare also fences Core 1 after the body's
// final usb_audio_drain_ring().  The body's complete_pipeline_reset() (or the
// SPDIF lock-block release) owns the matching dac_hw_mute_release().
static bool pipeline_reset_ready(void) {
    dac_hw_mute_assert();
    return dac_hw_mute_hold_elapsed();
}

// Per-slot teardown: stop the output PIO SM, mask the channel's DMA IRQ,
// abort the DMA, return the playing_buffer to the consumer pool, drain the
// consumer pipeline, then re-arm the channel IRQ.  Safe to call with main
// interrupts ENABLED, via a layered protection chain:
//
//   1. audio_*_set_enabled(inst, false) halts the PIO state machine,
//      stopping DREQ-driven DMA progress.  This is NOT the IRQ-skip
//      gate — the shared DMA IRQ handlers in audio_spdif.c:412-442 and
//      audio_i2s_multi.c:504-524 do NOT read inst->enabled; they gate
//      on dma_irqn_get_channel_status() (i.e. the post-mask `ints`
//      register).  The `enabled` flag governs whether the audio path
//      produces into the producer pool, not whether the IRQ handler
//      services completions.
//
//   2. dma_irqn_set_channel_enabled(false) masks this channel's IRQ
//      bit in irq_ctrl[irq_index].inte.  After this point the shared
//      handler reads dma_irqn_get_channel_status as 0 for this channel
//      and skips it — even if the underlying raw completion bit fires.
//      THIS is the actual race protection.
//
//   3. dma_channel_abort() is a HW-level stop with a busy-wait for
//      completion.  Critically: there is a brief window between step 1
//      (SM stop) and step 2 (IRQ mask) where the IRQ handler CAN
//      still fire if the last DMA word completed at exactly that
//      instant.  In that window the racing handler may have called
//      give_audio_buffer(playing_buffer) + audio_start_dma_transfer()
//      — populating playing_buffer again and starting a fresh DMA.
//      The abort here kills any such re-started DMA cleanly.
//
//   4. The `if (inst->playing_buffer) give_audio_buffer(...)` check
//      AFTER the abort is NOT redundant — it handles the race in (3).
//      Removing it would leak a buffer and (on re-enable) play stale
//      audio.  Future maintainers: do not "simplify" this.
//
//   5. *_reset_consumer_pipeline() drains the prepared list back to
//      the free list.  Safe because the channel IRQ is masked (the
//      only writer to this consumer pool from IRQ context); the spin-
//      lock inside the pool ops is the cross-with-main-thread guard.
//
//   6. dma_irqn_acknowledge_channel() clears any stale `ints` bit set
//      during the (3) race window BEFORE re-arming the line, so no
//      spurious post-reset interrupt fires.
//
// Concurrent producers that DON'T race here:
//   - USB audio class ISR (usb_audio.c:1286 → usb_audio_ring_push) only
//     pushes to the SPSC audio_ring.  Never touches consumer pools.
//   - USB SOF ISR (usb_audio.c:1315-1363) reads inst->words_consumed +
//     inst->current_transfer_words; these are written only from the
//     DMA IRQ handler.  With the channel IRQ masked, they're stable.
//   - Core 1 is idle: prepare_pipeline_reset() spin-waited for
//     work_done, and preset_loading=true blocks new dispatch from
//     process_audio_packet.  PDM mode (if active) operates on its own
//     ring/DMA and never touches output pools.
static void teardown_output_slot(int slot_idx) {
    extern uint8_t output_types[];
    extern audio_spdif_instance_t *spdif_instance_ptrs[];
    extern audio_i2s_instance_t *i2s_instance_ptrs[];

    if (output_types[slot_idx] == OUTPUT_TYPE_I2S) {
        audio_i2s_instance_t *inst = i2s_instance_ptrs[slot_idx];
        if (!inst || !inst->consumer_pool) return;

        if (inst->enabled) audio_i2s_set_enabled(inst, false);
        dma_irqn_set_channel_enabled(inst->dma_irq, inst->dma_channel, false);
        dma_channel_abort(inst->dma_channel);
        if (inst->playing_buffer) {
            give_audio_buffer(inst->consumer_pool, inst->playing_buffer);
            inst->playing_buffer = NULL;
        }
        i2s_reset_consumer_pipeline(inst);
        dma_irqn_acknowledge_channel(inst->dma_irq, inst->dma_channel);
        dma_irqn_set_channel_enabled(inst->dma_irq, inst->dma_channel, true);
    } else {
        audio_spdif_instance_t *inst = spdif_instance_ptrs[slot_idx];
        if (!inst || !inst->consumer_pool) return;

        if (inst->enabled) audio_spdif_set_enabled(inst, false);
        dma_irqn_set_channel_enabled(inst->dma_irq, inst->dma_channel, false);
        dma_channel_abort(inst->dma_channel);
        if (inst->playing_buffer) {
            give_audio_buffer(inst->consumer_pool, inst->playing_buffer);
            inst->playing_buffer = NULL;
        }
        spdif_reset_consumer_pipeline(inst);
        dma_irqn_acknowledge_channel(inst->dma_irq, inst->dma_channel);
        dma_irqn_set_channel_enabled(inst->dma_irq, inst->dma_channel, true);
    }
}

// In I2S clock-slave mode, gate the synchronized output start on an external
// LRCLK falling edge so the SPDIF-vs-I2S inter-slot offset is identical
// across every reset: SPDIF slots begin emitting sample 0 within nanoseconds
// of the gate edge (all priming is hoisted before the gate; only the
// pio_enable_sm_mask_in_sync write follows it), and the external-clock I2S
// program self-frames on the next falling edge having discarded exactly one
// frame (see audio_i2s_dataout_extclk.pio), landing both types on the same
// sample index.  Bounded wait (worst case one frame period, 22.7 us at
// 44.1 kHz, plus margin); on timeout (external clocks absent) it just
// returns; the slave lock machinery re-runs the synchronized start when
// clocks return.  Callers skip the gate when no output slot is I2S (SPDIF
// inter-slot alignment needs only the mask start; input-to-output phase is
// not sample-indexed).  Called with interrupts disabled.
static void i2s_slave_gate_on_lrclk(void) {
    if (!i2s_slave_mode_active()) return;
    // Poll the pin the running RX session was patched with, not the live
    // global (they diverge briefly across a deferred BCK pin change).
    const uint lrclk = (uint)i2s_input_active_bck_pin() + 1;
    const uint64_t deadline = time_us_64() + 35;
    while (!gpio_get(lrclk)) { if (time_us_64() > deadline) return; }
    while (gpio_get(lrclk))  { if (time_us_64() > deadline) return; }
}

// Start every prepared output slot SM in ONE pio_enable_sm_mask_in_sync
// write.  Shared by complete_pipeline_reset (Phase 2) and
// enable_outputs_in_sync; there is deliberately no other start path for
// output slots, so mixed-type start behavior cannot diverge again.
//
// Why one mask write: both output types live on PICO_AUDIO_SPDIF_PIO
// (asserted below), so a combined mask starts SPDIF and I2S SM clocks in
// the same sys_clk cycle AND resets their fractional clkdiv accumulators
// together.  The previous non-slave path issued two group starts
// (audio_spdif_enable_sync then audio_i2s_enable_sync) with the I2S DMA
// priming between them, which left a fixed few-microsecond inter-type
// start skew and unsynchronized divider phase at fractional rates.
// Single-type configs are unchanged (the mask degenerates to one group).
//
// In I2S clock-slave mode the write is additionally gated on an external
// LRCLK edge after both types are primed (only when an extclk I2S slot
// exists; up to ~35 us with IRQs off) so the gate-to-start latency cannot
// smear the extclk program's one-frame discard across a frame boundary
// at 96 kHz.
//
// Caller must hold interrupts disabled across this call: the DMAs primed
// by the prepare calls sit stalled against their stopped SMs (the 8-word
// PIO TX FIFO absorbs the head of the transfer), and a stale DMA IRQ or
// SOF firing between prime and start could complete an extra transfer for
// one type before the clocks run, producing a 1-frame inter-type skew.
static void start_output_slots_in_sync(audio_spdif_instance_t *spdif_sync[], uint spdif_count,
                                       audio_i2s_instance_t *i2s_sync[], uint i2s_count) {
    uint32_t mask = 0;
    PIO out_pio = NULL;
    if (spdif_count) {
        mask |= audio_spdif_enable_sync_prepare(spdif_sync, spdif_count);
        out_pio = spdif_sync[0]->pio;
    }
    if (i2s_count) {
        assert(!out_pio || i2s_sync[0]->pio == out_pio);
        mask |= audio_i2s_enable_sync_prepare(i2s_sync, i2s_count);
        out_pio = i2s_sync[0]->pio;
        if (i2s_slave_mode_active()) i2s_slave_gate_on_lrclk();
    }
    if (mask) pio_enable_sm_mask_in_sync(out_pio, mask);
}

// Three-phase pipeline reset.  The IRQ-disabled critical section in
// Phase 2 is intentionally tiny; only the synchronized PIO SM start
// needs atomicity (preserves CLAUDE.md's slot-alignment invariant).
// Phase 1 (per-slot teardown) and Phase 3 (USB feedback reset) run with
// interrupts enabled.
//
// Why keep blackout small: USB audio class ISRs continue to drain
// packets into the audio_ring throughout Phase 1, eliminating a ~1 ms
// USB starvation window that previously compounded the audible I2S DAC
// click on input-source switches.
//
// Phase 2's save_and_disable_interrupts wraps the prime-and-start helper
// (start_output_slots_in_sync); see its comment for why prime and start
// must be atomic.  Do NOT shrink this critical section; the regression
// would only surface on installations actually running both output types.
//
// KNOWN RACE (B1, lg_sound_sync-style benign): Phase 3's
// reset_usb_feedback_loop() performs ~8 non-atomic field writes to
// fb_ctrl, which the SOF ISR reads/writes every 1 ms.  An SOF firing
// mid-reset can observe a transiently inconsistent fb_ctrl struct
// (e.g. rate_valid=true with stale last_total_words) and compute a
// garbage delta.  Impact is bounded by FB_OUTER_CLAMP_Q16: at most one
// wire-feedback packet (1 ms) is off by ±1 sample/frame — well within
// the UAC1 host's normal jitter envelope.  Verified inaudible in
// listening tests.  Moving Phase 3 back inside the Phase 2 bracket
// would close the race (cost: ~8 extra stores in the IRQ-disabled
// section).  Left outside intentionally to keep blackout minimal; if
// fb_ctrl gains additional writers or the clamps are tightened, this
// decision should be revisited.
static void complete_pipeline_reset(void) {
    extern uint8_t output_types[];
    extern audio_spdif_instance_t *spdif_instance_ptrs[];
    extern audio_i2s_instance_t *i2s_instance_ptrs[];

    audio_spdif_instance_t *spdif_sync[NUM_SPDIF_INSTANCES];
    audio_i2s_instance_t *i2s_sync[NUM_SPDIF_INSTANCES];
    uint spdif_count = 0;
    uint i2s_count = 0;

    // Phase 1: per-slot teardown — interrupts enabled.
    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        teardown_output_slot(i);
        if (output_types[i] == OUTPUT_TYPE_I2S) {
            audio_i2s_instance_t *inst = i2s_instance_ptrs[i];
            if (inst && inst->consumer_pool) i2s_sync[i2s_count++] = inst;
        } else {
            audio_spdif_instance_t *inst = spdif_instance_ptrs[i];
            if (inst && inst->consumer_pool) spdif_sync[spdif_count++] = inst;
        }
    }

    // Phase 2: tiny IRQ-disabled section; synchronized single-mask PIO
    // start for every slot of both types (see start_output_slots_in_sync).
    uint32_t flags = save_and_disable_interrupts();
    start_output_slots_in_sync(spdif_sync, spdif_count, i2s_sync, i2s_count);
    restore_interrupts(flags);

    // Phase 3: USB feedback reset.  See B1 note in block comment above
    // for the (bounded) race with the SOF ISR.
    reset_usb_feedback_loop();

    // Phase 4: begin hardware-mute release.  Order is critical: clocks
    // must be running (Phase 2 completed) BEFORE the mute pin deasserts.
    // If release_ms > 0, dac_hw_mute_release() leaves the pin asserted
    // and returns; dac_hw_mute_tick() deasserts it later so the input
    // pipeline keeps draining while the DAC remains muted.
    //
    // EXCEPTION: I2S, SPDIF, or ADAT input with a pending prefill
    // (preset_loading still set): the main-loop prefill block for that source
    // (gated on preset_loading) will DRAIN and disable the outputs again after
    // this returns, then re-enable in sync and own the release (right after its
    // enable_outputs_in_sync()).  Releasing here would un-mute before that
    // drain stops the clocks, and with the default release_ms == 0 the pin
    // deasserts immediately, so the clock-stop click is fully exposed.  Defer
    // to the prefill block in those cases.  (USB has no such prefill drain,
    // so it releases here as before.  The prefill blocks always run while
    // their source is active and preset_loading is set; SPDIF and ADAT hold
    // the mute until lock, which is the intended mute-until-lock behavior, so
    // the mute is never left stuck asserted.  INPUT_SOURCE_ADAT is defined on
    // both platforms but never active on RP2040, so no #if is needed.)
    // input_source_is_spdif() matches the prefill block's own gate, covering
    // SPDIF inputs 2/3 as well; a bare == INPUT_SOURCE_SPDIF here once left
    // those two sources releasing early through this same path.
    if (!(preset_loading &&
          (active_input_source == INPUT_SOURCE_I2S ||
           input_source_is_spdif(active_input_source) ||
           active_input_source == INPUT_SOURCE_ADAT))) {
        dac_hw_mute_release();
    }

    // Phase 5: re-phase a slave-role I2S input.  The synchronized start in
    // Phase 2 rewinds the I2S TX clock master to its PIO entry point, which
    // resets LRCLK phase under a running slave input SM; without a resync
    // the input would misframe and swap L/R permanently.  No-op unless the
    // input is RUNNING in the slave role.
    i2s_input_resync();

#if PICO_RP2350
    // Phase 6: restart the ADAT bulk output against current config.  ADAT is
    // deliberately NOT in the Phase 2 sync start: its alignment comes from
    // stream tracking behind a fixed silence cushion, so restarting it here
    // re-establishes the constant ADAT-to-slot offset each epoch without
    // widening the IRQ-disabled section.
    adat_output_resync();
#endif
}

// Disable all outputs, abort DMA, drain consumer pipelines. Outputs stay
// disabled so consumer buffers can be prefilled before starting playback.
// Counterpart: enable_outputs_in_sync().
//
// Like complete_pipeline_reset(), runs the per-slot teardown with main
// interrupts ENABLED — see teardown_output_slot() for the safety argument.
static void drain_and_disable_outputs(void) {
    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        teardown_output_slot(i);
    }
#if PICO_RP2350
    adat_output_stream_stop();
#endif
}

// Enable all outputs in sync. Call after consumer buffers have been prefilled.
// Counterpart: drain_and_disable_outputs().
static void enable_outputs_in_sync(void) {
    extern uint8_t output_types[];
    extern audio_spdif_instance_t *spdif_instance_ptrs[];
    extern audio_i2s_instance_t *i2s_instance_ptrs[];

    audio_spdif_instance_t *spdif_sync[NUM_SPDIF_INSTANCES];
    audio_i2s_instance_t *i2s_sync[NUM_SPDIF_INSTANCES];
    uint spdif_count = 0;
    uint i2s_count = 0;

    uint32_t flags = save_and_disable_interrupts();

    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        if (output_types[i] == OUTPUT_TYPE_I2S) {
            audio_i2s_instance_t *inst = i2s_instance_ptrs[i];
            if (inst && inst->consumer_pool) i2s_sync[i2s_count++] = inst;
        } else {
            audio_spdif_instance_t *inst = spdif_instance_ptrs[i];
            if (inst && inst->consumer_pool) spdif_sync[spdif_count++] = inst;
        }
    }

    // Synchronized single-mask start for both types; slave-mode LRCLK
    // gating is handled inside (see start_output_slots_in_sync).
    start_output_slots_in_sync(spdif_sync, spdif_count, i2s_sync, i2s_count);

    restore_interrupts(flags);

    // Re-phase a slave-role I2S input after the TX master restart (see
    // complete_pipeline_reset Phase 5 for the rationale)
    i2s_input_resync();

#if PICO_RP2350
    adat_output_resync();
#endif
}

// Flash writes disable interrupts for tens of milliseconds. Even when DSP
// parameters are unchanged (e.g. preset save or directory writes), that
// blackout can leave output consumer pools underfilled and inter-slot phase
// skewed.  Bracket every deferred flash write with these helpers so audio
// always resumes from a deterministic, synchronized state.
//
// Additional anti-pop handling:
//  1) Use a rate-aware mute window (ms-based, not fixed sample count).
//  2) Allow a short pre-flash settle period so muted packets are actually
//     rendered to the outputs before interrupts are blacked out by flash ops.
// Settle must exceed envelope ramp (~8ms) + consumer pipeline drain (~16ms @ 48kHz).
// Premute must exceed settle + flash write (~45ms) + margin so the mute counter
// never expires before the pipeline is reset.  A configured DAC hardware-mute
// hold above ~75 ms burns through this while the settle loop services the
// input, but flash_write_sector()'s trailing re-arm restores preset_loading
// after every write (with the pending-hold floor from flash_mute_hold_samples),
// so the completion path always sees the flag set regardless of hold length.
#define FLASH_WRITE_PREMUTE_MS       120u
#define FLASH_WRITE_FADE_SETTLE_US   30000u

static uint32_t samples_for_duration_ms(uint32_t sample_rate_hz, uint32_t duration_ms) {
    uint64_t samples = ((uint64_t)sample_rate_hz * (uint64_t)duration_ms + 999u) / 1000u;
    if (samples < PRESET_MUTE_SAMPLES) samples = PRESET_MUTE_SAMPLES;
    if (samples > UINT32_MAX) samples = UINT32_MAX;
    return (uint32_t)samples;
}

// Tracks whether prepare_flash_write_operation() tore down SPDIF RX and
// therefore owes the complete_flash_write_operation_* helpers a restart.
// Static file-scope; flash operations are serialized via the main loop.
static bool spdif_suspended_for_flash = false;

// Same contract for I2S input.
static bool i2s_suspended_for_flash = false;

static void prepare_flash_write_operation(void) {
    // Drain the current input source once before the mute engages so the
    // envelope starts from the freshest possible state.
    if (active_input_source == INPUT_SOURCE_USB) {
        usb_audio_drain_ring();
    } else if (input_source_is_spdif(active_input_source)) {
        spdif_input_poll();
    } else if (active_input_source == INPUT_SOURCE_I2S) {
        i2s_input_poll();
    }
#if PICO_RP2350
    else if (active_input_source == INPUT_SOURCE_ADAT) {
        adat_input_poll();
    }
#endif

    prepare_pipeline_reset(samples_for_duration_ms(audio_state.freq,
                                                   FLASH_WRITE_PREMUTE_MS));

    // During the fade-out settle window, keep servicing the active input
    // source so the output consumer pools stay fed (with muted samples)
    // until the envelope reaches zero.  Prior to this fix, the settle loop
    // only drained the USB ring — for SPDIF input the RX FIFO filled and
    // overflowed and the output pools drained, producing post-save pops.
    //
    // The loop also absorbs the DAC hardware-mute hold armed by
    // prepare_pipeline_reset() above: it runs until BOTH the settle window
    // and dac_hw_mute_hold_elapsed() are satisfied, so the DAC has its full
    // ramp time before complete_flash_write_operation_*() stops the clocks.
    // Flash writes are inherently blocking (≈45 ms IRQ-off), so unlike the
    // deferred main-loop reset handlers there is nothing to yield to here —
    // folding the hold into the existing settle wait adds no blocking beyond
    // max(settle, hold), and keeps the pipeline fed throughout.  When not
    // streaming the loop is skipped: there is no audio, so no clock-stop
    // thump and no hold to honor.
    extern volatile bool sync_started;
    bool usb_streaming   = (active_input_source == INPUT_SOURCE_USB) && sync_started;
    bool spdif_streaming = (input_source_is_spdif(active_input_source)) &&
                           (spdif_input_get_state() == SPDIF_INPUT_LOCKED);
    bool i2s_streaming   = (active_input_source == INPUT_SOURCE_I2S) &&
                           (i2s_input_get_state() == I2S_INPUT_RUNNING);
    bool adat_streaming  = false;
#if PICO_RP2350
    adat_streaming = (active_input_source == INPUT_SOURCE_ADAT) &&
                     (adat_input_get_state() == ADAT_INPUT_LOCKED);
#endif
    if (usb_streaming || spdif_streaming || i2s_streaming || adat_streaming) {
        uint64_t start_us = time_us_64();
        while ((time_us_64() - start_us) < FLASH_WRITE_FADE_SETTLE_US
               || !dac_hw_mute_hold_elapsed()) {
            if (active_input_source == INPUT_SOURCE_USB) {
                usb_audio_drain_ring();
            } else if (input_source_is_spdif(active_input_source)) {
                spdif_input_poll();
#if PICO_RP2350
            } else if (active_input_source == INPUT_SOURCE_ADAT) {
                adat_input_poll();
#endif
            } else {
                i2s_input_poll();
            }
            tight_loop_contents();
        }
    }

    // Final flush just before the ~45 ms flash blackout.  For USB, drain
    // any residuals from the ring.
    if (active_input_source == INPUT_SOURCE_USB) {
        usb_audio_drain_ring();
    }

    // For SPDIF: fully tear down the RX pipeline before the flash blackout.
    // The alternative (keeping SPDIF RX running while IRQs are disabled for
    // ~45 ms × N flash writes) causes decode-timeout alarms to fire on the
    // blackout edge, tearing down DMA/PIO asynchronously while preset_save
    // is between its two flash writes.  That race crashes the core.  Stop
    // cleanly here; complete_flash_write_operation_*() restarts.
    if (input_source_is_spdif(active_input_source) &&
        spdif_input_get_state() != SPDIF_INPUT_INACTIVE) {
        spdif_input_stop();
        spdif_prefilling = false;
        spdif_suspended_for_flash = true;
    }

    // For I2S: stop before the blackout too.  The ring DMA itself is
    // IRQ-less and would survive, but on resume the accumulated backlog
    // and a slave SM that free-ran against stalled consumers are simpler
    // to reason about as a clean stop/restart, matching the SPDIF pattern.
    if (active_input_source == INPUT_SOURCE_I2S &&
        i2s_input_get_state() != I2S_INPUT_INACTIVE) {
        i2s_input_stop();
        i2s_suspended_for_flash = true;
    }

    // ADAT is deliberately NOT stopped for the blackout: its RX is an
    // IRQ-less free-running DMA ring (no decode-timeout alarms to race the
    // blackout edge, unlike SPDIF), and the software poll re-acquires frame
    // sync after the ring laps.  preset_loading is still set from
    // prepare_pipeline_reset() above, so the main-loop ADAT lock block
    // re-runs the drain/prefill/enable handshake once LOCKED returns.
}

// Restart SPDIF RX if prepare_flash_write_operation() tore it down.  Must
// be called from both full and light completion paths so every flash op
// symmetrically restarts what it suspended.
static void resume_spdif_after_flash(void) {
    if (!spdif_suspended_for_flash) return;
    spdif_suspended_for_flash = false;

    // If the active source changed during the flash write (e.g. preset load
    // that carries USB as the input source), don't restart — the pending
    // input_source_change_pending handler would immediately stop it again.
    if (!input_source_is_spdif(active_input_source)) return;
    if (input_source_change_pending) return;

    // preset_loading=true is still set from prepare_pipeline_reset(); the
    // main loop's SPDIF lock-acquisition block will fire when lock returns
    // and run the normal drain+prefill handshake.
    spdif_input_start();
}

// Restart I2S input if prepare_flash_write_operation() tore it down.  Same
// symmetry contract as resume_spdif_after_flash().
static void resume_i2s_after_flash(void) {
    if (!i2s_suspended_for_flash) return;
    i2s_suspended_for_flash = false;

    if (active_input_source != INPUT_SOURCE_I2S) return;
    if (input_source_change_pending) return;

    i2s_input_start(i2s_input_should_be_master());
}

// Source-split release of the hardware mute that prepare_pipeline_reset()
// asserted: the single canonical home of the rule deciding whether a completion
// path may deassert the DAC mute itself, or must leave it for the SPDIF
// lock-acquisition prefill block to release.
//
//   - USB input: the output PIO clocks are live when this runs (the light/
//     metadata path never stopped them; complete_pipeline_reset() on the full
//     path just restarted them in sync), so the pin can deassert now.  The soft
//     envelope is still settling, so the DAC un-mutes into silence/ramp, not a
//     step.
//   - SPDIF input: output must stay muted until valid SPDIF audio flows again.
//     The lock-acquisition prefill block (main loop) re-enables outputs after
//     re-lock and owns the matching dac_hw_mute_release(); deasserting here
//     would un-mute the DAC into pre-lock silence.
//   - I2S input: same conclusion as SPDIF, but for a subtler reason.
//     prepare_flash_write_operation() stops the I2S input and leaves
//     preset_loading set, so the main-loop I2S prefill block (gated on
//     preset_loading) DRAINS and disables the outputs AFTER this completion
//     returns, then re-enables them in sync and owns the matching
//     dac_hw_mute_release() (right after enable_outputs_in_sync()).  Deasserting
//     here would un-mute BEFORE that drain stops the output clocks — exposing
//     the exact clock-stop click the hardware mute exists to suppress.
//
// So only USB releases here (its outputs genuinely stay live through the light
// path).  Completion paths that run a full pipeline reset
// (complete_flash_write_operation_full()) release implicitly inside
// complete_pipeline_reset() and need not call this.  It exists for completion
// paths that keep outputs running through the operation — today the
// light/metadata flash path on USB — where the release is otherwise easy to
// forget (its omission once left the DAC silent indefinitely; see git 833a51a).
static inline void release_hw_mute_if_outputs_live(void) {
    if (active_input_source == INPUT_SOURCE_USB) {
        dac_hw_mute_release();
    }
}

// Full completion path: drain/restart all output consumer pipelines and reset
// feedback state. Use this for operations that materially affect runtime audio
// continuity (preset save/delete and legacy save command compatibility path).
static void complete_flash_write_operation_full(void) {
    // Restart SPDIF RX if prepare_flash_write_operation() suspended it.
    // The lock-acquisition block in the main loop will drain outputs and
    // run the prefill handshake once RX re-locks.
    resume_spdif_after_flash();

    // Restart I2S input likewise.  Unlike SPDIF it has no lock handshake:
    // the input falls through to complete_pipeline_reset() below (the
    // non-SPDIF branch), whose trailing i2s_input_resync() re-phases a
    // slave against the freshly restarted TX master.
    resume_i2s_after_flash();

    if (input_source_is_spdif(active_input_source)) {
        // For SPDIF input: the pre-flash settle loop pre-filled the output
        // consumer pools with muted samples.  The blackout stopped DMA
        // chaining after ~one buffer, leaving the remaining pool buffers
        // intact to play out silence as DMA resumes.  We skip
        // complete_pipeline_reset() — draining the pools here would force
        // outputs to restart against an empty pool, causing pops and
        // uneven inter-slot fill.  The hardware mute is likewise left asserted
        // (the lock-acquisition prefill path releases it after re-lock — the
        // SPDIF half of release_hw_mute_if_outputs_live()'s rule).
        reset_usb_feedback_loop();
        return;
    }

    // USB input: feedback loop and USB ring handle blackout recovery; a full
    // pipeline reset is safe and keeps inter-slot phase synchronized.
    complete_pipeline_reset();
}

// Light completion path: keep the mute envelope active, but skip a full output
// pipeline rebuild. Suitable for metadata-only flash writes (names/startup flags)
// where DSP/output topology is unchanged.
static inline void complete_flash_write_operation_light(void) {
    // Symmetry with prepare_flash_write_operation(): restart SPDIF RX and
    // I2S input if they were suspended for the blackout.
    resume_spdif_after_flash();
    resume_i2s_after_flash();

    // Release the DAC hardware mute that prepare_flash_write_operation()
    // asserted (USB input only; for SPDIF the lock-acquisition prefill path
    // owns it).  See release_hw_mute_if_outputs_live() for the full rule —
    // omitting this once left the DAC silent indefinitely after a metadata-only
    // write while EMC was enabled on USB input.
    release_hw_mute_if_outputs_live();
}

void core0_init() {
    // LED setup
    gpio_init(25); gpio_set_dir(25, GPIO_OUT);

#if PICO_RP2350
    // Enable flush-to-zero and default-NaN for audio processing.
    // Prevents denormal performance penalty in SVF/biquad state decay.
    {
        uint32_t fpscr;
        __asm__ volatile("vmrs %0, fpscr" : "=r"(fpscr));
        fpscr |= (1 << 24) | (1 << 25);  // FZ + DN bits
        __asm__ volatile("vmsr fpscr, %0" : : "r"(fpscr));
    }

    // RP2350: 307.2MHz (VCO 1536 / 5 / 1) — integer SPDIF/I2S dividers at 48kHz
    vreg_set_voltage(VREG_VOLTAGE_1_15);
    busy_wait_ms(10);

    if (!set_sys_clock_hz(307200000, false)) {
        set_sys_clock_hz(150000000, false);
    }

    // Drop flash clock from ROM default (~102 MHz) to sys_clk/6 ≈ 51.2 MHz
    // for parity with RP2040.  Subsequent flash ops go through the wrappers
    // in flash_clkdiv.c which restore this after each erase/program.
    dspi_flash_apply_clkdiv();
#else
    vreg_set_voltage(VREG_VOLTAGE_1_15);
    busy_wait_ms(10);
    // 307.2MHz -> VCO 1536 MHz / 5 / 1 — integer SPDIF/I2S dividers at 48kHz
    set_sys_clock_pll(1536000000, 5, 1);
#endif

    gpio_init(23); gpio_set_dir(23, GPIO_OUT); gpio_put(23, 1);

    pico_get_unique_board_id_string(usb_descriptor_str_serial, 17);

    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

    // Soft VCXO (system PLL trim): claims its explicit DMA channels, DMA
    // timer 0 and PWM pacer slice before any dynamic DMA claims below.
    soft_vcxo_init();

    // Bench clock/audio-path diagnostics: enable the TX starvation monitors
    // and seed the timing baselines.  No-op when DSPI_CLOCK_DIAG is 0.
    clock_diag_init();

    // [CRITICAL FIX]
    // Initialize USB/SPDIF *BEFORE* PDM.
    // SPDIF requires DMA Channel 0 (hardcoded in config).
    // If PDM inits first, it steals Ch 0 via dma_claim_unused_channel(), causing SPDIF to panic/crash.
    usb_sound_card_init();

#if PICO_RP2350
    // Templates only, no hardware; must precede the first adat_output_resync()
    // (reachable via process_type_switches -> complete_pipeline_reset below).
    adat_output_init();
    adat_input_init();
#endif

    // Initialize feedback controller and nominal rate
    fb_ctrl_init(&fb_ctrl);
    nominal_feedback_10_14 = ((uint64_t)audio_state.freq << 14) / 1000;
    feedback_10_14 = nominal_feedback_10_14;

    // Assert USB SOF cannot be preempted by DMA IRQs — required for
    // the non-atomic multi-field read in usb_sof_irq() to be safe.
    assert(NVIC_GetPriority(USBCTRL_IRQ) <= NVIC_GetPriority(DMA_IRQ_0 + PICO_AUDIO_SPDIF_DMA_IRQ));
    assert(NVIC_GetPriority(USBCTRL_IRQ) <= NVIC_GetPriority(DMA_IRQ_0 + PICO_AUDIO_I2S_DMA_IRQ));

    // Load preset from flash.  Always selects a preset (factory defaults if
    // the target slot is empty).  Migrates legacy data on first boot.
    preset_boot_load();

    // DAC hardware mute init.  Must come AFTER preset_boot_load() so the
    // directory's persisted config (mute pin assignments, polarity, hold
    // time) is available; the config arrives via preset_get_dac_hw_mute()
    // which reads dir_cache.  Claims GPIOs and drives them to un-muted
    // level per polarity.  No-op when feature is disabled in flash
    // (factory-fresh state).
    {
        DacHwMuteConfig hw;
        preset_get_dac_hw_mute(&hw);
        dac_hw_mute_init(&hw);
    }

    // Sync MCK library state with the just-loaded globals.  usb_sound_card_init()
    // (above) called audio_i2s_mck_setup() with the boot-default pin; if the
    // preset specifies a different mck_pin or wants mck_enabled=true, the
    // library would otherwise drive the wrong pin (or fail to start) once
    // process_type_switches() below tries to enable MCK.  This is the
    // sole boot-time apply-path sync point.
    {
        extern uint8_t  i2s_mck_pin;
        extern bool     i2s_mck_enabled;
        extern uint16_t i2s_mck_multiplier;
        audio_i2s_mck_apply_state(i2s_mck_pin, i2s_mck_enabled,
                                  audio_state.freq, i2s_mck_multiplier);
    }

    {
        uint32_t flags = save_and_disable_interrupts();
        dsp_recalculate_all_filters(48000.0f);
        dsp_update_delay_samples(48000.0f);
        restore_interrupts(flags);

        // Apply output type + pin configuration from preset (before Core 1 starts).
        // usb_sound_card_init() created all slots as SPDIF; convert any that the
        // preset saved as I2S using process_type_switches() for correct master election.
        {
            extern uint8_t output_types[];
            extern uint8_t output_pins[];
            extern audio_spdif_instance_t *spdif_instance_ptrs[];

            // Build change mask for slots that need I2S + apply SPDIF pin changes
            uint8_t boot_mask = 0;
            uint8_t boot_types[NUM_SPDIF_INSTANCES];
            for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
                boot_types[i] = output_types[i];
                if (output_types[i] == OUTPUT_TYPE_I2S) {
                    boot_mask |= (1u << i);
                } else {
                    // SPDIF slot — apply pin config if changed
                    if (output_pins[i] != spdif_instance_ptrs[i]->pin) {
                        audio_spdif_set_enabled(spdif_instance_ptrs[i], false);
                        audio_spdif_change_pin(spdif_instance_ptrs[i], output_pins[i]);
                        audio_spdif_set_enabled(spdif_instance_ptrs[i], true);
                    }
                }
            }

            // Temporarily mark all slots as SPDIF (they were set up as SPDIF at init).
            // process_type_switches() compares against output_types[] to detect changes.
            for (int i = 0; i < NUM_SPDIF_INSTANCES; i++)
                output_types[i] = OUTPUT_TYPE_SPDIF;

            if (boot_mask)
                process_type_switches(boot_mask, boot_types);
        }
    }

#if PICO_RP2350
    // Start the ADAT bulk output if the loaded config enables it.  Boot paths
    // that ran complete_pipeline_reset() above already did this; idempotent.
    adat_output_resync();
#endif

    // Initial loudness table computation (uses loaded or default params)
    loudness_recompute_table(loudness_ref_spl, loudness_intensity_pct, 48000.0f);
    if (loudness_enabled && loudness_active_table) {
        audio_set_volume(audio_state.volume);  // Re-select loudness coefficients
    }

    // Initial volume leveller setup (uses loaded or default params)
    leveller_compute_coefficients(&leveller_coeffs, (const LevellerConfig *)&leveller_config, 48000.0f);
    leveller_reset_state(&leveller_state);
    leveller_bypassed = !leveller_config.enabled;

    // Initial psychoacoustic bass setup (uses loaded or default params)
    psybass_apply_config((const PsybassConfig *)&psybass_config, 48000.0f);

#if PICO_RP2350
    // Initial upmixer setup (uses loaded or default params)
    upmix_apply_config((const UpmixConfig *)&upmix_config, 48000.0f);
#endif

#if ENABLE_SUB
    {
        extern uint8_t output_pins[];
        pdm_setup_hw(output_pins[NUM_PIN_OUTPUTS - 1]);
    }

    // Determine initial Core 1 mode from output enables (may have been loaded from flash)
    if (matrix_mixer.outputs[NUM_OUTPUT_CHANNELS - 1].enabled) {
        core1_mode = CORE1_MODE_PDM;
        pdm_set_enabled(true);
    } else {
        // Check if any outputs 2-7 are enabled for EQ worker
        bool any_eq_output = false;
        for (int i = CORE1_EQ_FIRST_OUTPUT; i <= CORE1_EQ_LAST_OUTPUT; i++) {
            if (matrix_mixer.outputs[i].enabled) { any_eq_output = true; break; }
        }
        core1_mode = any_eq_output ? CORE1_MODE_EQ_WORKER : CORE1_MODE_IDLE;
        pdm_set_enabled(false);
    }

    multicore_launch_core1(pdm_core1_entry);
#endif

    // Initialize SPDIF RX subsystem (no PIO/DMA resources claimed yet)
    spdif_input_init();

    // Initialize I2S RX subsystem (no PIO/DMA resources claimed yet)
    i2s_input_init();

    // Initialize the LG Sound Sync state machine.  Must come AFTER
    // preset_boot_load() (which sets s_enabled from the loaded slot via
    // apply_slot_to_live → lg_sound_sync_set_enabled) so the streaks/
    // last-decoded fields are zeroed without clobbering the just-loaded
    // user preference.  s_enabled is intentionally not reset here.
    //
    // Note on notification ordering: any notify_param_write() calls
    // emitted from preset_boot_load()'s apply path land in the still-
    // zeroed notify ring before notify_init() runs, then are silently
    // discarded when notify_init() resets head/tail. This is the
    // established pattern (every per-preset setting has the same
    // shape) and is benign — the post-init shadow is rebaselined
    // from final live state via bulk_params_collect, so the host
    // never sees a stale value. If a future change moves any
    // ring-consumer to run before notify_init(), preset_boot_load()
    // should be wrapped in notify_begin_bulk(PARAM_SRC_PRESET) to
    // collapse the otherwise-many discarded entries into one tagged
    // BULK_INVALIDATED. */
    lg_sound_sync_init();

    // If the loaded preset has SPDIF as input source, start RX hardware.
    // Output remains muted until lock is acquired (handled in main loop).
    //
    // IMPORTANT: prepare_pipeline_reset() must run BEFORE spdif_input_start()
    // here so preset_loading=true is set when the lock-acquisition block
    // (line ~834) evaluates on the first post-lock main-loop iteration.
    // Without this, the block's precondition (`preset_loading && !prefilling`)
    // is never satisfied, so drain_and_disable_outputs() → prefill →
    // enable_outputs_in_sync() never runs; outputs instead stream from
    // whatever fill state usb_sound_card_init() left behind, producing the
    // "feedback targeting lower fill" asymmetry between boot-with-SPDIF and
    // the runtime USB→SPDIF switch.  The runtime path already calls this;
    // boot was the odd one out.
    if (input_source_is_spdif(active_input_source)) {
        prepare_pipeline_reset(PRESET_MUTE_SAMPLES);
        spdif_input_start();
    } else if (active_input_source == INPUT_SOURCE_I2S) {
        // Defensive parity with the SPDIF branch.  In practice
        // apply_slot_to_live() defers the source via
        // input_source_change_pending, so boot-into-I2S normally runs
        // through the main-loop switch handler instead; this covers any
        // path that sets the source directly before we get here.
        //
        // Bring up the input with the rate applied but outputs left muted;
        // the main-loop I2S block prefills the consumer pools to 50% then
        // enables in sync (same handshake as the runtime switch).
        prepare_pipeline_reset(PRESET_MUTE_SAMPLES);
        i2s_input_bringup_prefill();
    }
#if PICO_RP2350
    else if (active_input_source == INPUT_SOURCE_ADAT) {
        // Same parity for boot-into-ADAT: outputs stay muted until the
        // receiver locks; the main-loop ADAT block owns drain/prefill/enable.
        prepare_pipeline_reset(PRESET_MUTE_SAMPLES);
        // Master mode: the device is the rate authority (shared with I2S
        // master mode); apply the selected rate before starting.  Slave mode
        // detects the wire rate and arms its own change after lock.
        if (adat_clock_mode == ADAT_CLOCK_MODE_MASTER &&
            i2s_input_rate != audio_state.freq) {
            perform_rate_change(i2s_input_rate, true);
            dsp_update_delay_samples((float)i2s_input_rate);
        }
        adat_input_start();
    }
#endif

    // Baseline the notification shadow from the fully-initialised live state.
    // Must come after preset_boot_load() / apply_factory_defaults() so any
    // subsequent param_write call sees a truthful baseline and only emits
    // notifications on real changes.
    notify_init();

    // External control interfaces (UART / I2C target).  Deliberately last:
    // after preset_boot_load() (persisted config available), after every
    // audio pin claim above (a stored control config whose pins now collide
    // is quietly kept down, visible via REQ_GET_CTRL_IFACE_STATUS), and
    // after notify_init() so the UART bring-up's notify-consumer activation
    // is not wiped by the consumer-table reset.  Both interfaces ship
    // disabled.
    {
        UartCtrlConfig ucfg;
        I2cCtrlConfig  icfg;
        preset_get_ctrl_iface(&ucfg, &icfg);
        // Record the boot validation results so REQ_GET_CTRL_IFACE_STATUS is
        // truthful when a stored config's pins now collide (live stays 0).
        ctrl_uart_last_status = uart_ctrl_init(&ucfg);
        ctrl_i2c_last_status  = i2c_ctrl_init(&icfg);
    }

    // Control Surfaces (physical controls/indicators on user GPIOs).  Last of
    // all: after every other pin claim (a stored binding whose pins now
    // collide is kept down, visible via REQ_GET_CS_STATUS) and after
    // notify_init so its dispatched writes notify normally.  Pot/switch
    // boot-sync dispatches fire from the first main-loop ticks, not here.
    control_surfaces_init();
}

int main(void) {
    // Initial LED on to show we're alive
    gpio_init(25); gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, 1);

#if !PICO_RP2350
    set_sys_clock_pll(1536000000, 4, 2);
#endif

    core0_init();

    // Enable watchdog
    watchdog_enable(8000, 1);

    while (1) {
        // Update watchdog
        watchdog_update();

        // Bench diagnostics: cheap per-iteration sampling (fc0 service, loop
        // gap, min consumer fill, PWM-pacer movement).  No-op when the flag
        // is 0.
        clock_diag_sample();

        // TinyUSB device task — processes enumeration, control transfers, and
        // deferred bus events.  Must be called at least once per main-loop
        // iteration.  Audio RX and feedback tx happen from USB IRQ via our
        // UAC1 class driver callbacks, so tud_task() is not latency-critical
        // for the audio stream itself.
        tud_task();

        // Fire any queued device→host notifications to EP 0x83.  Emit is
        // deferred from update_master_volume() to here so we never call
        // usbd_edpt_xfer from within a control-transfer DATA stage.
        usb_notify_tick();

        // External control transports: parse any complete UART/I2C frames
        // and dispatch them through the shared vendor-command surface.
        // Both are cheap no-ops while disabled and never block; heavy work
        // (bulk apply, flash) stays on the existing deferred paths below.
        uart_ctrl_poll();
        i2c_ctrl_poll();

        // LG Sound Sync detection tick — internally throttled and
        // gated on (feature enabled && SPDIF input && SPDIF locked).
        // Cheap on the not-applicable path; safe to call every loop.
        lg_sound_sync_tick();

        // DAC hardware-mute deadline check — releases async test pulses
        // and post-clock-restart release holds on schedule.  Two loads +
        // branches when no deadline is in flight (the steady state).
        dac_hw_mute_tick();

        // Control Surfaces poll: debounce buttons/switches, decode encoders,
        // read pots, drive LEDs.  Internally throttled to 1 kHz; immediate
        // no-op while no binding is active.
        control_surfaces_tick();

        // Test-signal generator: apply staged configs and emit deferred
        // notifications, then drive the pipeline with generator blocks when
        // no input source is streaming.  Both are immediate no-ops while the
        // generator is idle.
        siggen_service();
        siggen_pump();

        // Drain USB audio ring — highest priority (only when USB is active input).
        // USB ISR pushes raw packets into the ring; we run the full DSP
        // pipeline here in main-loop context instead of USB IRQ context.
        // In non-USB modes, defensively flush so any packet pushed by the ISR
        // in the brief window straddling an active_input_source change can't
        // sit stale until the next USB→x switch.
        if (active_input_source == INPUT_SOURCE_USB) {
            usb_audio_drain_ring();
        } else {
            usb_audio_flush_ring();
        }

        // Poll SPDIF input when active
        if (input_source_is_spdif(active_input_source)) {
            SpdifInputState rx_state = spdif_input_get_state();

            // Handle lock acquisition: drain outputs, prefill, then start.
            // Gated on dac_hw_mute_hold_elapsed() so the DAC hardware-mute
            // hold armed by prepare_pipeline_reset() (on the USB->SPDIF
            // switch, boot-into-SPDIF, or re-lock) completes BEFORE
            // drain_and_disable_outputs() stops the output clocks.  In the
            // normal case lock takes far longer than hold_ms so this is
            // already satisfied; the guard only adds a few non-blocking
            // poll iterations on an instant re-lock.  Returns true with
            // zero latency when the feature is disabled.
            if (rx_state == SPDIF_INPUT_LOCKED && preset_loading && !spdif_prefilling
                    && dac_hw_mute_hold_elapsed()) {
                spdif_input_check_rate_change();
                // Disable outputs and drain consumer buffers so they can
                // be prefilled with real audio before playback begins.
                drain_and_disable_outputs();
                preset_loading = false;
                preset_mute_counter = 0;
                spdif_prefilling = true;
            }

            // Prefill: wait for consumer buffers to reach 50% before enabling outputs
            if (spdif_prefilling) {
                if (get_slot_consumer_fill(0) >= SPDIF_CONSUMER_BUFFER_COUNT / 2) {
                    enable_outputs_in_sync();
                    spdif_prefilling = false;
                    // Release the DAC hardware mute that prepare_pipeline_reset()
                    // asserted on the USB→SPDIF switch (or boot-into-SPDIF, or
                    // re-lock after lock loss).  The SPDIF lock-acquisition flow
                    // intentionally skips complete_pipeline_reset() so output stays
                    // muted until lock + prefill complete; without this release the
                    // XSMT pin would stay asserted indefinitely and the DAC's analog
                    // stage would never un-mute.  Order matches complete_pipeline_reset()
                    // Phase 4: clocks running (enable_outputs_in_sync above) BEFORE
                    // mute release begins, so either immediate deassert or the
                    // delayed release_ms deadline happens with valid BCK/LRCLK.
                    dac_hw_mute_release();
                }
            }

            // Handle lock loss: mute output immediately
            if (rx_state == SPDIF_INPUT_RELOCKING && !preset_loading && !spdif_prefilling) {
                prepare_pipeline_reset(PRESET_MUTE_SAMPLES);
                spdif_prefilling = false;
                pipeline_reset_cpu_metering();
            }

            // Read FIFO audio and feed DSP pipeline
            spdif_input_poll();

            // Adjust output PIO dividers to track SPDIF input clock
            spdif_input_update_clock_servo();
        }

#if PICO_RP2350
        // Poll ADAT input when active.  Both clock modes run the SPDIF-style
        // lock-gated flow: frame sync must be found and verified before audio
        // flows, and outputs prefill against real input audio.  Master mode
        // simply has no rate detection or servo behind the same states.
        else if (active_input_source == INPUT_SOURCE_ADAT) {
            AdatInputState ad_state = adat_input_get_state();

            // Lock acquired: drain outputs, prefill, then start (same gating
            // as the SPDIF block; see its comment for the DAC-mute hold).
            // In slave mode a detected-rate mismatch defers a rate change
            // instead; prefill re-triggers once the pipeline rate matches.
            if (ad_state == ADAT_INPUT_LOCKED && preset_loading && !adat_prefilling
                    && dac_hw_mute_hold_elapsed()) {
                if (!adat_input_check_rate_change()) {
                    drain_and_disable_outputs();
                    preset_loading = false;
                    preset_mute_counter = 0;
                    adat_prefilling = true;
                }
            }

            // The LOCKED gate matters (mirrors the I2S slave prefill): never
            // enable outputs against a receiver that lost sync mid-prefill.
            if (adat_prefilling && ad_state == ADAT_INPUT_LOCKED &&
                get_slot_consumer_fill(0) >= SPDIF_CONSUMER_BUFFER_COUNT / 2) {
                enable_outputs_in_sync();
                adat_prefilling = false;
                // Release the DAC hardware mute now that clocks are running
                // (Phase 4 ordering, same as the SPDIF prefill path).
                dac_hw_mute_release();
            }

            // Sync or signal loss: mute. No receiver restart is needed
            // (unlike the I2S slave's stalled wait-driven SMs): the RX PIO
            // free-runs and the poll re-acquires frame sync in software.
            if (ad_state == ADAT_INPUT_RELOCKING && !preset_loading) {
                prepare_pipeline_reset(PRESET_MUTE_SAMPLES);
                pipeline_reset_cpu_metering();
            }

            // Rate machine + sync search + decode + pipeline feed
            adat_input_poll();

            // Slave mode + LOCKED only; cheap no-op otherwise
            adat_input_update_clock_servo();
        }
#endif

        // Poll I2S input when active.
        else if (active_input_source == INPUT_SOURCE_I2S &&
                 i2s_clock_mode == I2S_CLOCK_MODE_SLAVE) {
            // Clock-slave mode: SPDIF-style lock-gated flow.  The external
            // master owns BCK/LRCLK, so outputs stay muted/drained until the
            // measured external rate locks; the input keeps producing during
            // the prefill (its clocks never stop), so the pools fill with
            // real audio like the SPDIF path.
            i2s_slave_poll();
            I2sSlaveState sl_state = i2s_slave_get_state();

            if (sl_state == I2S_SLAVE_LOCKED && preset_loading && !i2s_prefilling
                    && dac_hw_mute_hold_elapsed()) {
                if (!i2s_slave_check_rate_change()) {
                    drain_and_disable_outputs();
                    preset_loading = false;
                    preset_mute_counter = 0;
                    i2s_prefilling = true;
                }
                // else: the deferred rate change restarts the input; prefill
                // re-triggers when it re-locks at the new pipeline rate.
            }

            // The LOCKED gate matters: if clocks drop mid-prefill the fill
            // must not re-enable outputs against a possibly misframed RX;
            // the RELOCKING restart below re-frames everything first.  This
            // enable is a deliberate second start after the one inside any
            // preceding complete_pipeline_reset(): the reset's start may
            // have been ungated (clocks absent), so the gated re-enable
            // here is what actually re-establishes the inter-type offset.
            // Do not "optimize" it away.
            if (i2s_prefilling && sl_state == I2S_SLAVE_LOCKED &&
                get_slot_consumer_fill(0) >= SPDIF_CONSUMER_BUFFER_COUNT / 2) {
                enable_outputs_in_sync();
                i2s_prefilling = false;
                // Release the DAC hardware mute now that clocks are running
                // (Phase 4 ordering, same as the SPDIF prefill path).
                dac_hw_mute_release();
            }

            // Clock loss or external rate change: mute (if not already
            // muted), then ALWAYS restart the receiver so its SMs (possibly
            // stalled mid-word) re-frame on the returning clocks; the
            // prefill's enable_outputs_in_sync() re-frames the external-clock
            // TX SMs the same way.  The restart must not be skipped while
            // preset_loading is set (startup mute hold, armed rate change):
            // RELOCKING could otherwise re-acquire straight to LOCKED with a
            // misframed receiver.  Covers loss during an in-flight prefill
            // too (prepare_pipeline_reset clears i2s_prefilling).  One-shot:
            // the restart re-arms measurement into ACQUIRING.
            if (sl_state == I2S_SLAVE_RELOCKING) {
                if (!preset_loading) {
                    prepare_pipeline_reset(PRESET_MUTE_SAMPLES);
                    pipeline_reset_cpu_metering();
                }
                i2s_input_stop();
                i2s_input_start(i2s_input_should_be_master());
            }

            // Process audio only when LOCKED (mirrors the SPDIF poll's lock
            // gate): pre-lock the measured rate may not match the pipeline
            // rate, and producing would also drain the soft-mute counter
            // before the prefill drain runs.  The RX DMA ring free-runs and
            // self-wraps meanwhile; the prefill starts from freshly drained
            // pools, so nothing stale is played.
            if (sl_state == I2S_SLAVE_LOCKED) {
                i2s_input_poll();
            }
            i2s_slave_update_clock_servo();
        }
        // Master clock mode: no lock handling and no clock servo (the input
        // is synchronous to our own clock domain), but it DOES use the same
        // prefill handshake as SPDIF so outputs start against a 50% consumer
        // fill instead of whatever low level the startup transient leaves.
        // Trigger is preset_loading (set by every disruptive op via
        // prepare_pipeline_reset); there is no lock to wait for, so prefill
        // begins as soon as the DAC-mute hold has elapsed.
        else if (active_input_source == INPUT_SOURCE_I2S) {
            bool i2s_master = i2s_input_is_clock_master();

            if (preset_loading && !i2s_prefilling && dac_hw_mute_hold_elapsed()) {
                // Disable outputs and drain consumer pools so they can be
                // prefilled before playback begins.
                drain_and_disable_outputs();
                preset_loading = false;
                preset_mute_counter = 0;
                i2s_prefilling = true;
            }

            if (i2s_prefilling &&
                get_slot_consumer_fill(0) >= SPDIF_CONSUMER_BUFFER_COUNT / 2) {
                enable_outputs_in_sync();
                i2s_prefilling = false;
                // Release the DAC hardware mute now that clocks are running
                // (Phase 4 ordering, same as the SPDIF prefill path).
                dac_hw_mute_release();
            } else if (i2s_prefilling && !i2s_master) {
                // Slave role: the drain above stopped the I2S output clock
                // master that supplies the input's BCK/LRCLK, so the input
                // produces no samples and i2s_input_poll() cannot fill the
                // pools. Synthesize a silent block instead; real audio resumes
                // after enable_outputs_in_sync() restarts the clock master and
                // re-phases the input ring (i2s_input_resync). Master role
                // self-clocks and fills with real audio via the poll below.
                i2s_input_prefill_silence(192);
            }

            i2s_input_poll();
        }

        // Handle deferred flash SET commands (fire-and-forget, no result).
        // Atomic snapshot: briefly disable IRQs to copy payload + clear flag,
        // preventing the USB ISR from overwriting payload mid-read.
        {
            extern volatile bool flash_set_name_pending;
            if (flash_set_name_pending) {
                char name[PRESET_NAME_LEN];
                uint8_t slot;
                uint32_t f = save_and_disable_interrupts();
                extern uint8_t flash_set_name_slot;
                extern char flash_set_name_buf[];
                slot = flash_set_name_slot;
                memcpy(name, flash_set_name_buf, PRESET_NAME_LEN);
                flash_set_name_pending = false;
                restore_interrupts(f);
                prepare_flash_write_operation();
                uint8_t status = preset_set_name(slot, name);
                complete_flash_write_operation_light();
                if (status != PRESET_OK) {
                    printf("preset_set_name failed: slot=%u err=%u\n",
                           (unsigned)slot, (unsigned)status);
                }
            }

            extern volatile bool flash_set_startup_pending;
            if (flash_set_startup_pending) {
                uint8_t mode, slot;
                uint32_t f = save_and_disable_interrupts();
                extern uint8_t flash_set_startup_mode;
                extern uint8_t flash_set_startup_slot;
                mode = flash_set_startup_mode;
                slot = flash_set_startup_slot;
                flash_set_startup_pending = false;
                restore_interrupts(f);
                prepare_flash_write_operation();
                uint8_t status = preset_set_startup(mode, slot);
                complete_flash_write_operation_light();
                if (status != PRESET_OK) {
                    printf("preset_set_startup failed: mode=%u slot=%u err=%u\n",
                           (unsigned)mode, (unsigned)slot, (unsigned)status);
                }
            }

            extern volatile bool flash_set_output_config_mode_pending;
            if (flash_set_output_config_mode_pending) {
                uint8_t val;
                uint32_t f = save_and_disable_interrupts();
                extern uint8_t flash_set_output_config_mode_val;
                val = flash_set_output_config_mode_val;
                flash_set_output_config_mode_pending = false;
                restore_interrupts(f);
                prepare_flash_write_operation();
                preset_set_output_config_mode(val);
                complete_flash_write_operation_light();
            }

            extern volatile bool flash_save_output_config_pending;
            if (flash_save_output_config_pending) {
                uint32_t f = save_and_disable_interrupts();
                flash_save_output_config_pending = false;
                restore_interrupts(f);
                prepare_flash_write_operation();
                preset_save_output_config();
                complete_flash_write_operation_light();
            }

            extern volatile bool flash_set_master_volume_mode_pending;
            if (flash_set_master_volume_mode_pending) {
                uint8_t val;
                uint32_t f = save_and_disable_interrupts();
                extern uint8_t flash_set_master_volume_mode_val;
                val = flash_set_master_volume_mode_val;
                flash_set_master_volume_mode_pending = false;
                restore_interrupts(f);
                prepare_flash_write_operation();
                preset_set_master_volume_mode(val);
                complete_flash_write_operation_light();
            }

            extern volatile bool flash_save_master_volume_pending;
            if (flash_save_master_volume_pending) {
                uint32_t f = save_and_disable_interrupts();
                flash_save_master_volume_pending = false;
                restore_interrupts(f);
                prepare_flash_write_operation();
                preset_save_master_volume();
                complete_flash_write_operation_light();
            }

            // UART / I2C control-interface config (USB-only SETs, deferred).
            // Live apply first (GPIO/IRQ work, no flash); persist to the
            // directory only when the new config validated, so a bad SET
            // can never clobber a good stored config.
            if (ctrl_set_uart_pending) {
                UartCtrlConfig cfg;
                uint32_t f = save_and_disable_interrupts();
                memcpy(&cfg, (const void *)&ctrl_set_uart_val, sizeof(cfg));
                ctrl_set_uart_pending = false;
                restore_interrupts(f);
                uint8_t status = uart_ctrl_apply(&cfg);
                ctrl_uart_last_status = status;
                if (status == PIN_CONFIG_SUCCESS) {
                    prepare_flash_write_operation();
                    preset_set_ctrl_iface(&cfg, NULL);
                    complete_flash_write_operation_light();
                }
            }

            if (ctrl_set_i2c_pending) {
                I2cCtrlConfig cfg;
                uint32_t f = save_and_disable_interrupts();
                memcpy(&cfg, (const void *)&ctrl_set_i2c_val, sizeof(cfg));
                ctrl_set_i2c_pending = false;
                restore_interrupts(f);
                uint8_t status = i2c_ctrl_apply(&cfg);
                ctrl_i2c_last_status = status;
                if (status == PIN_CONFIG_SUCCESS) {
                    prepare_flash_write_operation();
                    preset_set_ctrl_iface(NULL, &cfg);
                    complete_flash_write_operation_light();
                }
            }

            // Control Surfaces binding SET (deferred).  Live-only preview:
            // apply does the GPIO/ADC claims but no flash; a successful apply
            // just marks the live config dirty.  REQ_CS_SAVE persists the
            // whole config later; REQ_CS_REVERT discards the preview.
            if (cs_set_binding_pending) {
                CsBinding b;
                uint8_t slot;
                uint32_t f = save_and_disable_interrupts();
                memcpy(&b, (const void *)&cs_set_binding_val, sizeof(b));
                slot = cs_set_binding_slot;
                cs_set_binding_pending = false;
                restore_interrupts(f);
                uint8_t status = control_surfaces_apply_binding(slot, &b);
                cs_last_status = status;
                cs_last_slot = slot;
                if (status == PIN_CONFIG_SUCCESS) {
                    control_surfaces_set_dirty(true);
                }
            }

            // Control Surfaces slot-name SET (deferred).  Live-only preview
            // like the binding SET: update the live name table, mark dirty,
            // no flash.  Slot range was validated at the vendor handler.
            if (cs_set_name_pending) {
                char name[CS_NAME_LEN];
                uint8_t slot;
                uint32_t f = save_and_disable_interrupts();
                memcpy(name, cs_set_name_val, sizeof(name));
                slot = cs_set_name_slot;
                cs_set_name_pending = false;
                restore_interrupts(f);
                uint8_t status = control_surfaces_apply_name(slot, name);
                cs_last_status = status;
                cs_last_slot = slot;
                if (status == PIN_CONFIG_SUCCESS) {
                    control_surfaces_set_dirty(true);
                }
            }

            // Control Surfaces IR-command SET (deferred).  Live-only preview
            // like the binding SET: apply the command, mark dirty on success,
            // no flash.  Sub-slot is reported as 0x80 | slot in cs_last_slot.
            if (cs_set_ir_cmd_pending) {
                IrCommand c;
                uint8_t slot;
                uint32_t f = save_and_disable_interrupts();
                memcpy(&c, (const void *)&cs_set_ir_cmd_val, sizeof(c));
                slot = cs_set_ir_cmd_slot;
                cs_set_ir_cmd_pending = false;
                restore_interrupts(f);
                uint8_t status = control_surfaces_apply_ir_cmd(slot, &c);
                cs_last_status = status;
                cs_last_slot = 0x80 | slot;
                if (status == PIN_CONFIG_SUCCESS) {
                    control_surfaces_set_dirty(true);
                }
            }

            // Control Surfaces SAVE (deferred).  Persist the whole live
            // config (bindings + IR commands + slot names) in one directory
            // flash write, then clear the dirty preview flag on success.
            if (cs_save_pending) {
                uint32_t f = save_and_disable_interrupts();
                cs_save_pending = false;
                restore_interrupts(f);
                prepare_flash_write_operation();
                uint8_t rc = preset_set_cs_all(control_surfaces_config(),
                                               control_surfaces_ir_config(),
                                               control_surfaces_names());
                complete_flash_write_operation_light();
                cs_last_status = (rc == PRESET_OK) ? PIN_CONFIG_SUCCESS
                                                   : CS_STATUS_FLASH_ERROR;
                cs_last_slot = 0xFF;
                if (rc == PRESET_OK) {
                    control_surfaces_set_dirty(false);
                }
            }

            // Control Surfaces REVERT (deferred).  Re-apply the stored config
            // from the directory RAM cache (GPIO reclaims, no flash) and drop
            // the dirty preview flag.
            if (cs_revert_pending) {
                uint32_t f = save_and_disable_interrupts();
                cs_revert_pending = false;
                restore_interrupts(f);
                control_surfaces_revert();
                control_surfaces_set_dirty(false);
                cs_last_status = PIN_CONFIG_SUCCESS;
                cs_last_slot = 0xFF;
            }

            // DAC hardware mute config update (deferred from USB ISR).
            // dac_hw_mute_set_config does validation, applies live pin
            // claims, writes the directory (~45 ms flash), and emits the
            // wire-format notify so other connected hosts see the new
            // state.  prepare_flash_write_operation brackets so SPDIF RX
            // and other peripherals survive the flash blackout.
            extern volatile bool flash_set_dac_hw_mute_pending;
            extern DacHwMuteConfig flash_set_dac_hw_mute_val;
            if (flash_set_dac_hw_mute_pending) {
                DacHwMuteConfig hw;
                uint32_t f = save_and_disable_interrupts();
                memcpy(&hw, (const void *)&flash_set_dac_hw_mute_val, sizeof(hw));
                flash_set_dac_hw_mute_pending = false;
                restore_interrupts(f);
                prepare_flash_write_operation();
                (void)dac_hw_mute_set_config(&hw);
                complete_flash_write_operation_light();
            }

            // DAC hardware mute test pulse — starts asynchronously and
            // releases via dac_hw_mute_tick() on the main loop's normal
            // cadence.  A synchronous busy-wait here would block the
            // audio drain and starve SPDIF/I²S outputs (~48 ms producer-
            // pool depth at 48 kHz) — see dac_hw_mute.c for the design
            // note.
            extern volatile bool dac_hw_mute_test_pending;
            if (dac_hw_mute_test_pending) {
                uint32_t f = save_and_disable_interrupts();
                dac_hw_mute_test_pending = false;
                restore_interrupts(f);
                (void)dac_hw_mute_test_start();
            }
        }

        // Handle EQ / crossover band parameter updates from USB.  The
        // pending_packet's `band` field is the wire band index (already
        // normalized at the vendor handler boundary — see
        // crossover_filters_spec.md):
        //   0..channel_band_counts-1 → PEQ
        //   XOVER_BAND_BASE..XOVER_BAND_BASE+MAX_XOVER_BANDS-1 → crossover band (band - XOVER_BAND_BASE)
        // Anything else is invalid and would have been rejected upstream.
        if (eq_update_pending) {
            EqParamPacket p = pending_packet;
            uint16_t qpx = pending_eq_qp_x512;
            eq_update_pending = false;

            bool is_xover = (p.band >= XOVER_BAND_BASE &&
                             p.band < (XOVER_BAND_BASE + MAX_XOVER_BANDS));
            uint8_t local = is_xover ? (uint8_t)(p.band - XOVER_BAND_BASE) : p.band;

            if (is_xover) {
                xover_recipes[p.channel][local] = p;
                // Notification offset points into the WireCrossoverConfig
                // section, NOT eq[].  Apps receiving PARAM_CHANGED can
                // distinguish the section by offset; bulk readers see the
                // same byte position they got from REQ_GET_ALL_PARAMS.
                WireBandParams wbp;
                memset(&wbp, 0, sizeof(wbp));
                wbp.type    = (uint8_t)p.type;
                wbp.bypass  = (p.bypass == 1) ? 1 : 0;
                wbp.freq    = p.freq;
                wbp.q       = p.Q;
                wbp.gain_db = p.gain_db;
                uint16_t off = (uint16_t)(offsetof(WireBulkParams, crossovers)
                    + offsetof(WireCrossoverConfig, bands)
                    + ((uint16_t)p.channel * WIRE_MAX_XOVER_BANDS + local) * sizeof(WireBandParams));
                notify_param_write(off, sizeof(WireBandParams), &wbp);
            } else {
                // Store the LT target Qp before filter_recipes so it is in
                // place ahead of the dsp_compute_coefficients call below.
                peq_qp_x512[p.channel][p.band] = qpx;
                filter_recipes[p.channel][p.band] = p;
                WireBandParams wbp;
                memset(&wbp, 0, sizeof(wbp));
                wbp.type    = (uint8_t)p.type;
                wbp.bypass  = (p.bypass == 1) ? 1 : 0;
                wbp.freq    = p.freq;
                wbp.q       = p.Q;
                wbp.gain_db = p.gain_db;
                // LT bands carry Qp (Q*512, LE) in the otherwise-zero reserved
                // bytes so notification readers can recover the 4th parameter.
                if (p.type == FILTER_LINKWITZ_TRANSFORM) {
                    wbp.reserved[0] = (uint8_t)(qpx & 0xFF);
                    wbp.reserved[1] = (uint8_t)((qpx >> 8) & 0xFF);
                }
                uint16_t off = (uint16_t)(offsetof(WireBulkParams, eq)
                    + ((uint16_t)p.channel * WIRE_MAX_BANDS + p.band) * sizeof(WireBandParams));
                notify_param_write(off, sizeof(WireBandParams), &wbp);
            }

            // If updating a Core 1 output's channel, wait for Core 1 to
            // finish current work before modifying coefficients.  Applies
            // equally to PEQ and crossover — both run on Core 1 for the
            // same output range when EQ_WORKER mode is active.
            bool is_core1_channel = (p.channel >= (CH_OUT_1 + CORE1_EQ_FIRST_OUTPUT) &&
                                     p.channel <= (CH_OUT_1 + CORE1_EQ_LAST_OUTPUT));
            if (is_core1_channel && core1_mode == CORE1_MODE_EQ_WORKER) {
                while (core1_eq_work.work_ready && !core1_eq_work.work_done) {
                    tight_loop_contents();
                }
                __dmb();
            }

            uint32_t flags = save_and_disable_interrupts();
            if (is_xover) {
                xover_design_filter(&p, &xover_filters[p.channel][local],
                                    (float)audio_state.freq);
                xover_update_channel_bypass(p.channel);
            } else {
                dsp_compute_coefficients(&p, &filters[p.channel][p.band],
                                        (float)audio_state.freq);
                // Recalculate channel bypass flag (PEQ side)
                bool all_bypassed = true;
                for (int b = 0; b < channel_band_counts[p.channel]; b++) {
                    if (!filters[p.channel][b].bypass) {
                        all_bypassed = false;
                        break;
                    }
                }
                channel_bypassed[p.channel] = all_bypassed;
            }
            restore_interrupts(flags);
        }

        // Handle sample rate changes.  Gated on the non-blocking DAC
        // hardware-mute hold; perform_rate_change()'s own
        // prepare_pipeline_reset() then engages the soft mute + Core 1 fence
        // and re-asserts the (already-held) hardware mute idempotently.
        if (rate_change_pending && pipeline_reset_ready()) {
            uint32_t r = pending_rate;
            rate_change_pending = false;
            usb_audio_drain_ring();  // Process old-rate packets before clock switch
            // For I2S/ADAT input the main-loop prefill block owns the output
            // restart + mute release; defer so complete_pipeline_reset()
            // doesn't release the mute before that block's drain (for ADAT,
            // before the receiver has even re-locked).  USB/SPDIF restart here.
            perform_rate_change(r, active_input_source == INPUT_SOURCE_I2S ||
                                   active_input_source == INPUT_SOURCE_ADAT);
        }

        // Handle loudness table recomputation
        if (loudness_recompute_pending) {
            loudness_recompute_pending = false;
            loudness_recompute_table(loudness_ref_spl, loudness_intensity_pct, (float)audio_state.freq);
            // Re-key the active coefficient pointer at the current
            // vol_index.  effective_vol_index is set in lock-step with
            // vol_mul by apply_vol_index_to_audio(), so this picks up
            // whatever volume owner is currently active (USB host slider,
            // LG Sound Sync, …) without disturbing vol_mul itself.  The
            // previous formulation (audio_set_volume(audio_state.volume))
            // early-returned during SPDIF playback and left the coefficient
            // pointer dangling at the pre-recompute table, an audible
            // defect when the user adjusted ref SPL or intensity while
            // playing through SPDIF.
            if (loudness_enabled && loudness_active_table) {
                uint8_t idx = effective_vol_index;
                if (idx > CENTER_VOLUME_INDEX) idx = CENTER_VOLUME_INDEX;
                current_loudness_coeffs = loudness_active_table[idx];
            }
        }

        // Handle crossfeed coefficient updates: compute into the inactive
        // buffer and publish (NULL = disabled); pair states are reset by
        // the pipeline whenever a pair is skipped.
        if (crossfeed_update_pending) {
            crossfeed_update_pending = false;
            crossfeed_apply_config((const CrossfeedConfig *)&crossfeed_config, (float)audio_state.freq);
        }

        // Handle psychoacoustic bass coefficient updates: same double-buffer
        // publish model as crossfeed (NULL = disabled); per-output states are
        // reset by the pipeline whenever an output is skipped.
        if (psybass_update_pending) {
            psybass_update_pending = false;
            psybass_apply_config((const PsybassConfig *)&psybass_config, (float)audio_state.freq);
        }

#if PICO_RP2350
        // Handle upmixer coefficient updates: same double-buffer publish
        // model (NULL = disabled); processing state resets via upmix_park()
        // whenever the pipeline pass is not running.
        if (upmix_update_pending) {
            upmix_update_pending = false;
            upmix_apply_config((const UpmixConfig *)&upmix_config, (float)audio_state.freq);
        }
#endif

        // Handle volume leveller coefficient updates
        if (leveller_update_pending) {
            leveller_update_pending = false;
            leveller_compute_coefficients(&leveller_coeffs, (const LevellerConfig *)&leveller_config, (float)audio_state.freq);
            if (leveller_reset_pending) {
                leveller_reset_pending = false;
                leveller_reset_state(&leveller_state);
            }
            leveller_bypassed = !leveller_config.enabled;
        }

        // Handle USB stream restart (alt 0 -> alt > 0): re-lock all active output
        // pipelines so consumer fill/phase starts aligned after host re-prime.
        {
            extern volatile bool stream_restart_resync_pending;
            if (stream_restart_resync_pending && pipeline_reset_ready()) {
                stream_restart_resync_pending = false;
                __dmb();

                usb_audio_drain_ring();   // Process remaining packets
                usb_audio_flush_ring();   // Discard stale data from previous stream

                // Engage the soft mute and fence Core 1 (the drains above
                // re-dispatch it) before the synchronized teardown/restart.
                // The gate held the DAC hardware mute; this re-asserts it
                // idempotently (hold not re-armed).
                prepare_pipeline_reset(PRESET_MUTE_SAMPLES);
                complete_pipeline_reset();
                printf("USB stream restart: outputs resynced\n");
            }
        }

        // Handle deferred preset operations.
        // These were moved out of the USB IRQ to avoid:
        //  - 45ms interrupt blackout from flash writes inside an ISR
        //  - Missing pipeline reset after preset_load (stale consumer buffers
        //    with old DSP parameters would play out for ~24ms)
        //  - Delay line bleed-through when delay length changes between presets
        {
            extern volatile bool preset_load_pending;
            extern volatile bool save_params_pending;
            extern volatile bool preset_save_pending;
            extern volatile uint8_t pending_preset_load_slot;
            extern volatile uint8_t pending_preset_save_slot;

            if (preset_load_pending && pipeline_reset_ready()) {
                preset_load_pending = false;
                __dmb();

                // Generator state is strictly transient: a preset load always
                // silences it (audio is already muted here, so no fade).
                siggen_stop_immediate(SIGGEN_STOP_PRESET);

                extern uint8_t output_types[];

                // Snapshot current output types BEFORE load so we can detect
                // which slots need hardware reconfiguration afterward.
                uint8_t old_types[NUM_SPDIF_INSTANCES];
                memcpy(old_types, output_types, NUM_SPDIF_INSTANCES);

                usb_audio_drain_ring();
                // Engage the soft mute and fence Core 1 after the drain.  The
                // gate held the DAC hardware mute; this re-asserts it
                // idempotently (hold not re-armed).
                prepare_pipeline_reset(PRESET_MUTE_SAMPLES);

                // Tear down SPDIF RX across the flash write (preset_load's
                // dir_flush does a ~45 ms blackout).  Leaving RX running
                // during the blackout lets decode-timeout alarms fire on the
                // post-blackout edge, racing with the downstream pipeline.
                // See prepare_flash_write_operation() for the same pattern.
                bool suspended_spdif = false;
                if (input_source_is_spdif(active_input_source) &&
                    spdif_input_get_state() != SPDIF_INPUT_INACTIVE) {
                    spdif_input_stop();
                    spdif_prefilling = false;
                    suspended_spdif = true;
                }

                // Same for I2S input: the loaded preset may change output
                // types (input role re-election) and the flash blackout
                // stalls the consumer side.  Restarted once below.
                bool suspended_i2s = false;
                if (active_input_source == INPUT_SOURCE_I2S &&
                    i2s_input_get_state() != I2S_INPUT_INACTIVE) {
                    i2s_input_stop();
                    suspended_i2s = true;
                }

                // Apply the new preset: overwrites all DSP state (EQ, delays,
                // matrix, gains, output_types[]), recalculates filter coefficients,
                // transitions Core 1 mode, and writes the directory to flash.
                preset_load(pending_preset_load_slot);

                // SPDIF RX restart is deferred until after process_type_switches
                // (or the no-type-change branch) below.  Restarting here would
                // race with process_type_switches' own RX management — RX
                // would be torn down and re-acquired twice, doubling the audible
                // glitch on a preset switch that also flips an output type.

                // Sync MCK library state with the freshly-applied preset
                // globals.  Handles three transitions in one call:
                //   • mck_pin changed → disable, change_pin, (re-)enable
                //   • mck_enabled flipped → start or stop on current pin
                //   • mck_enabled unchanged but multiplier/Fs changed →
                //     glitchless DIV reload
                // Any subsequent process_type_switches() may still gate
                // MCK off if no I2S slots end up active, which is the
                // pre-existing "auto-disable when no I2S" behaviour.
                {
                    extern uint8_t  i2s_mck_pin;
                    extern bool     i2s_mck_enabled;
                    extern uint16_t i2s_mck_multiplier;
                    audio_i2s_mck_apply_state(i2s_mck_pin, i2s_mck_enabled,
                                              audio_state.freq, i2s_mck_multiplier);
                }

                // Build change mask for slots whose type changed
                uint8_t change_mask = 0;
                for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
                    if (output_types[i] != old_types[i])
                        change_mask |= (1u << i);
                }

                if (change_mask) {
                    // preset_load() already wrote new types to output_types[].
                    // Restore old types so process_type_switches() sees the
                    // delta correctly (it compares against output_types[]).
                    // RX is INACTIVE here (we suspended it pre-flash), so
                    // process_type_switches' own RX management is a no-op
                    // for this caller — we restart RX once below.
                    uint8_t new_types[NUM_SPDIF_INSTANCES];
                    memcpy(new_types, output_types, NUM_SPDIF_INSTANCES);
                    memcpy(output_types, old_types, NUM_SPDIF_INSTANCES);
                    process_type_switches(change_mask, new_types);
                } else if (input_source_is_spdif(active_input_source)) {
                    // SPDIF input: don't drain/re-enable the output pipeline.
                    // The pool holds muted samples queued by the preset-mute
                    // window; draining them would force outputs to restart
                    // against empty pools and produce pops/uneven fill.  Let
                    // DMA resume chaining after the flash blackout and play
                    // out silence until the mute envelope fades back in.
                    reset_usb_feedback_loop();
                } else {
                    // No type changes — just resync pipelines
                    complete_pipeline_reset();
                }

                // BCK-only restore: a bulk/preset restore can install a new
                // i2s_bck_pin with an unchanged type map, which the mask above
                // cannot see.  rebuild_i2s_output_clocking() re-invokes
                // process_type_switches over the I2S slots; its clocking
                // mismatch detection (master election / external-clock mode /
                // BCK pin) rebuilds only when something actually differs, so
                // this is a cheap no-op in the common case (including when the
                // type switch above already rebuilt everything).
                rebuild_i2s_output_clocking();

                // Restart SPDIF RX once, after all type-switch / pipeline
                // work is done.  Skipped if the preset switched the input
                // source — input_source_change_pending will manage RX
                // when its handler fires below.
                if (suspended_spdif &&
                    input_source_is_spdif(active_input_source) &&
                    !input_source_change_pending) {
                    spdif_input_start();
                }

                // Restart I2S input once, with a freshly elected role (the
                // loaded preset may have changed the output types).  If the
                // loaded preset selects a different I2S rate, defer a rate
                // change; perform_rate_change's own bracket restarts the
                // input at the new rate.
                if (suspended_i2s &&
                    active_input_source == INPUT_SOURCE_I2S &&
                    !input_source_change_pending) {
                    i2s_input_start(i2s_input_should_be_master());
                    // io_config_apply() may have flagged a pin hot-swap
                    // (i2s_rx_pin_change_pending) OR a full restart for a
                    // multichannel / channel-count change (i2s_input_restart_
                    // pending) while applying the slot; this restart already
                    // used the new config, so consume BOTH to avoid a redundant
                    // deferred restart (and its extra mute).
                    i2s_rx_pin_change_pending = false;
                    i2s_input_restart_pending = false;
                    // Master mode only: in slave mode the stored rate is
                    // dormant (the external master defines the rate; the lock
                    // machinery re-rates as needed).  Check the EFFECTIVE
                    // mode: this apply may itself have deferred a flip to
                    // slave, in which case the live global still reads
                    // master here.
                    uint8_t eff_mode = i2s_clock_mode_change_pending
                                           ? pending_i2s_clock_mode
                                           : i2s_clock_mode;
                    if (eff_mode != I2S_CLOCK_MODE_SLAVE &&
                        i2s_input_rate != audio_state.freq) {
                        pending_rate = i2s_input_rate;
                        __dmb();
                        rate_change_pending = true;
                    }
                }
            }

            if (save_params_pending) {
                save_params_pending = false;
                __dmb();

                // Legacy REQ_SAVE_PARAMS compatibility path.  Keep this on the
                // same robust flash-write flow as preset save.
                prepare_flash_write_operation();
                int status = flash_save_params();
                complete_flash_write_operation_full();
                if (status != FLASH_OK) {
                    printf("flash_save_params failed: err=%d\n", status);
                }
            }

            if (preset_save_pending) {
                preset_save_pending = false;
                __dmb();

                // Even though save does not modify DSP parameters, it performs
                // two flash writes (slot + directory), each with long interrupt
                // blackout.  Always do a full post-write resync so outputs
                // cannot remain in a skewed/underfilled state.
                prepare_flash_write_operation();
                uint8_t status = preset_save(pending_preset_save_slot);
                complete_flash_write_operation_full();
                if (status != PRESET_OK) {
                    printf("preset_save failed: slot=%u err=%u\n",
                           (unsigned)pending_preset_save_slot, (unsigned)status);
                }
            }

            extern volatile uint16_t preset_delete_mask;
            if (preset_delete_mask) {
                // Atomically snapshot and clear the mask so new deletes
                // arriving during processing are captured in the next pass.
                uint32_t flags = save_and_disable_interrupts();
                uint16_t mask = preset_delete_mask;
                preset_delete_mask = 0;
                restore_interrupts(flags);

                extern uint8_t output_types[];

                // Snapshot output types before deletes — if the active
                // slot is deleted, apply_factory_defaults() resets
                // output_types[] to all-SPDIF without a hardware switch.
                uint8_t old_types[NUM_SPDIF_INSTANCES];
                memcpy(old_types, output_types, NUM_SPDIF_INSTANCES);

                // Single prepare/complete bracket around all deletes —
                // each preset_delete() does its own flash erase internally.
                prepare_flash_write_operation();
                for (int slot = 0; slot < PRESET_SLOTS; slot++) {
                    if (mask & (1u << slot)) {
                        preset_delete(slot);
                    }
                }

                // Check if output types changed (active slot deleted →
                // factory defaults → all SPDIF).  If so, do a proper
                // hardware type switch instead of just a pipeline reset.
                uint8_t change_mask = 0;
                for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
                    if (output_types[i] != old_types[i])
                        change_mask |= (1u << i);
                }

                if (change_mask) {
                    uint8_t new_types[NUM_SPDIF_INSTANCES];
                    memcpy(new_types, output_types, NUM_SPDIF_INSTANCES);
                    memcpy(output_types, old_types, NUM_SPDIF_INSTANCES);
                    process_type_switches(change_mask, new_types);
                } else {
                    complete_flash_write_operation_full();
                }

                // BCK-only restore: a bulk/preset restore can install a new
                // i2s_bck_pin with an unchanged type map, which the mask above
                // cannot see.  rebuild_i2s_output_clocking() re-invokes
                // process_type_switches over the I2S slots; its clocking
                // mismatch detection (master election / external-clock mode /
                // BCK pin) rebuilds only when something actually differs, so
                // this is a cheap no-op in the common case (including when the
                // type switch above already rebuilt everything).
                rebuild_i2s_output_clocking();
            }

            extern volatile bool factory_reset_pending;
            if (factory_reset_pending && pipeline_reset_ready()) {
                factory_reset_pending = false;
                __dmb();

                // Transient generator state: factory reset silences it.
                siggen_stop_immediate(SIGGEN_STOP_PRESET);

                extern uint8_t output_types[];

                // Snapshot current output types before reset clears them to
                // all-SPDIF so we can detect I2S→SPDIF transitions.
                uint8_t old_types[NUM_SPDIF_INSTANCES];
                memcpy(old_types, output_types, NUM_SPDIF_INSTANCES);

                usb_audio_drain_ring();
                // Engage the soft mute and fence Core 1 after the drain before
                // mutating shared DSP state (delay-line zeroing below touches
                // buffers Core 1 reads).  Gate held the DAC hardware mute;
                // idempotent re-assert (hold not re-armed).
                prepare_pipeline_reset(PRESET_MUTE_SAMPLES);

                // Tear down SPDIF RX across the reset (recalc + delay-line
                // zeroing mutate state the live RX path consumes; its decode-
                // timeout alarm IRQ can fire mid-mutation).  Same hazard/guard
                // as preset_load_pending and bulk_params_pending.  Restarted
                // at the end (or by process_type_switches if types changed and
                // RX is still down — it leaves caller-stopped RX alone).
                bool suspended_spdif = false;
                if (input_source_is_spdif(active_input_source) &&
                    spdif_input_get_state() != SPDIF_INPUT_INACTIVE) {
                    spdif_input_stop();
                    spdif_prefilling = false;
                    suspended_spdif = true;
                }

                // Same teardown for I2S input.  flash_factory_reset() forces
                // the input source back to USB, so the restart guard below
                // naturally skips; the suspension still matters for the
                // mutation window itself.
                bool suspended_i2s = false;
                if (active_input_source == INPUT_SOURCE_I2S &&
                    i2s_input_get_state() != I2S_INPUT_INACTIVE) {
                    i2s_input_stop();
                    suspended_i2s = true;
                }

                flash_factory_reset();
                dsp_recalculate_all_filters((float)audio_state.freq);
                dsp_update_delay_samples((float)audio_state.freq);
                loudness_recompute_pending = true;
                crossfeed_update_pending = true;

                // Zero delay lines to prevent stale audio bleed-through
                extern
#if PICO_RP2350
                float delay_lines[NUM_DELAY_CHANNELS][MAX_DELAY_SAMPLES];
#else
                int32_t delay_lines[NUM_DELAY_CHANNELS][MAX_DELAY_SAMPLES];
#endif
                memset(delay_lines, 0, sizeof(delay_lines));

                // Transition Core 1 mode to match new output enable state
                Core1Mode new_mode = derive_core1_mode();
                if (new_mode != core1_mode) {
                    core1_mode = new_mode;
#if ENABLE_SUB
                    pdm_set_enabled(new_mode == CORE1_MODE_PDM);
#endif
                    __sev();
                }

                // Check if output types changed (factory defaults = all SPDIF)
                uint8_t change_mask = 0;
                for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
                    if (output_types[i] != old_types[i])
                        change_mask |= (1u << i);
                }

                if (change_mask) {
                    uint8_t new_types[NUM_SPDIF_INSTANCES];
                    memcpy(new_types, output_types, NUM_SPDIF_INSTANCES);
                    memcpy(output_types, old_types, NUM_SPDIF_INSTANCES);
                    process_type_switches(change_mask, new_types);
                } else {
                    complete_pipeline_reset();
                }

                // BCK-only restore: a bulk/preset restore can install a new
                // i2s_bck_pin with an unchanged type map, which the mask above
                // cannot see.  rebuild_i2s_output_clocking() re-invokes
                // process_type_switches over the I2S slots; its clocking
                // mismatch detection (master election / external-clock mode /
                // BCK pin) rebuilds only when something actually differs, so
                // this is a cheap no-op in the common case (including when the
                // type switch above already rebuilt everything).
                rebuild_i2s_output_clocking();

                // Restart SPDIF RX if we suspended it above (skip if an input-
                // source change is pending — that handler manages RX).
                if (suspended_spdif &&
                    input_source_is_spdif(active_input_source) &&
                    !input_source_change_pending) {
                    spdif_input_start();
                }

                // I2S input mirror (normally skipped: factory reset forces
                // the input source to USB).  Defensive parity with the preset/
                // bulk mirrors: if this ever runs, consume any restore-raised
                // I2S restart flags so the deferred handler doesn't re-restart.
                if (suspended_i2s &&
                    active_input_source == INPUT_SOURCE_I2S &&
                    !input_source_change_pending) {
                    i2s_input_start(i2s_input_should_be_master());
                    i2s_rx_pin_change_pending = false;
                    i2s_input_restart_pending = false;
                }
            }
        }

        // Handle output type change (deferred from USB ISR — needs heap allocation)
        {
            extern volatile uint8_t output_type_change_mask;
            extern volatile uint8_t pending_output_types[];

            // Gated on the non-blocking DAC hardware-mute hold.
            // process_type_switches()'s own prepare_pipeline_reset() then
            // engages the soft mute + Core 1 fence and re-asserts the
            // (already-held) hardware mute; its teardown runs after the hold.
            if (output_type_change_mask && pipeline_reset_ready()) {
                uint8_t mask = output_type_change_mask;
                output_type_change_mask = 0;
                __dmb();
                process_type_switches(mask, (const uint8_t *)pending_output_types);
            }
        }

#if PICO_RP2350
        // ADAT bulk-output service: silence top-up every pass; deferred config
        // apply (vendor enable/pin change) through the same muted-restart
        // bracket the other output config paths use.  Reset paths that already
        // ran complete_pipeline_reset() cleared the dirty flag on the way.
        // Serviced BEFORE process_pin_changes so an ADAT disable releases its
        // GPIO before a queued slot re-pin can be installed on it.
        adat_output_task();
        if (adat_output_config_dirty && pipeline_reset_ready()) {
            prepare_pipeline_reset(PRESET_MUTE_SAMPLES);
            complete_pipeline_reset();
        }
#endif

        // Handle deferred output data-pin reassignment (SPDIF/I2S slots).
        // Gated on the DAC hardware-mute hold like the other reset handlers;
        // process_pin_changes() mutes, repins, and restarts all slots in sync.
        {
            extern volatile uint8_t output_pin_change_mask;
            if (output_pin_change_mask && pipeline_reset_ready()) {
                uint8_t mask = output_pin_change_mask;
                output_pin_change_mask = 0;
                __dmb();
                process_pin_changes(mask);
            }
        }

        // Handle bulk parameter SET (deferred from USB IRQ)
        if (bulk_params_pending && pipeline_reset_ready()) {
            bulk_params_pending = false;

            extern uint8_t output_types[];

            // Snapshot current output types BEFORE apply so we can detect
            // which slots need hardware reconfiguration afterward.  Without
            // this, bulk SET would change output_types[] in RAM while the
            // SPDIF/I2S hardware stayed in its previous configuration —
            // slots flipped to I2S would keep emitting biphase-mark on the
            // data pin, audible as loud noise on the wired-up I2S receiver.
            uint8_t old_types[NUM_SPDIF_INSTANCES];
            memcpy(old_types, output_types, NUM_SPDIF_INSTANCES);

            usb_audio_drain_ring();  // Process before full state swap
            // Engage the soft mute and fence Core 1 after the drain before
            // bulk_params_apply() mutates coefficients / matrix state Core 1
            // reads.  Gate held the DAC hardware mute; idempotent re-assert
            // (hold not re-armed).
            prepare_pipeline_reset(PRESET_MUTE_SAMPLES);

            // Tear down SPDIF RX across the full-state swap.  bulk_params_apply()
            // + dsp_recalculate_all_filters() mutate the very coefficients/matrix
            // the live SPDIF input path consumes, and pico_spdif_rx's decode-
            // timeout alarm IRQ can fire mid-mutation and touch PIO/DMA state we
            // are reconfiguring — the same hazard preset_load_pending guards. Left
            // running, this races the swap and faults the core (8s watchdog reset).
            // Restarted once at the end (or by the input-source handler if the
            // bulk payload changed the source).
            bool suspended_spdif = false;
            if (input_source_is_spdif(active_input_source) &&
                spdif_input_get_state() != SPDIF_INPUT_INACTIVE) {
                spdif_input_stop();
                spdif_prefilling = false;
                suspended_spdif = true;
            }

            // Same teardown for I2S input: the bulk payload can change
            // output types (role re-election), the BCK pin, the data pin
            // and the I2S rate.  Restarted once at the end.
            bool suspended_i2s = false;
            if (active_input_source == INPUT_SOURCE_I2S &&
                i2s_input_get_state() != I2S_INPUT_INACTIVE) {
                i2s_input_stop();
                suspended_i2s = true;
            }

            // Apply the received parameters.  Pin config is applied only in
            // with-preset mode (output_config_mode == 1); in independent mode the
            // device-global IO is left to apply_output_config_from_mode / the
            // explicit REQ_SAVE_OUTPUT_CONFIG, so a bulk push doesn't stomp it.
            uint16_t _occ; uint8_t _m, _d, _la, oc_mode, _mv_mode;
            preset_get_directory(&_occ, &_m, &_d, &_la, &oc_mode, &_mv_mode);
            int err = bulk_params_apply((const WireBulkParams *)bulk_param_buf,
                                        oc_mode == OUTPUT_CONFIG_MODE_WITH_PRESET);
            if (err == 0) {
                float rate = (float)audio_state.freq;
                dsp_recalculate_all_filters(rate);
                dsp_update_delay_samples(rate);

                // Sync MCK library state with the freshly-applied bulk
                // globals — same rationale and three-transition coverage
                // as the preset_load_pending block above.
                {
                    extern uint8_t  i2s_mck_pin;
                    extern bool     i2s_mck_enabled;
                    extern uint16_t i2s_mck_multiplier;
                    audio_i2s_mck_apply_state(i2s_mck_pin, i2s_mck_enabled,
                                              audio_state.freq, i2s_mck_multiplier);
                }

                // Transition Core 1 mode to match new output enable state
                Core1Mode new_mode = derive_core1_mode();
                if (new_mode != core1_mode) {
                    core1_mode = new_mode;
#if ENABLE_SUB
                    pdm_set_enabled(new_mode == CORE1_MODE_PDM);
#endif
                    __sev();
                }

                // Build change mask for slots whose type changed
                uint8_t change_mask = 0;
                for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
                    if (output_types[i] != old_types[i])
                        change_mask |= (1u << i);
                }

                if (change_mask) {
                    // bulk_params_apply() already wrote new types to output_types[].
                    // Restore old types so process_type_switches() sees the
                    // delta correctly (it compares against output_types[]).
                    uint8_t new_types[NUM_SPDIF_INSTANCES];
                    memcpy(new_types, output_types, NUM_SPDIF_INSTANCES);
                    memcpy(output_types, old_types, NUM_SPDIF_INSTANCES);
                    process_type_switches(change_mask, new_types);
                } else if (input_source_is_spdif(active_input_source)) {
                    // SPDIF input: don't drain/re-enable the output pipeline.
                    // Same rationale as the preset_load_pending path — draining
                    // mid-prefill would force outputs to restart against empty
                    // pools.  Just reset the feedback loop.
                    reset_usb_feedback_loop();
                } else {
                    // No type changes — resync pipelines so the post-mute
                    // restart is sample-aligned across all slots.
                    complete_pipeline_reset();
                }

                // BCK-only restore: a bulk/preset restore can install a new
                // i2s_bck_pin with an unchanged type map, which the mask above
                // cannot see.  rebuild_i2s_output_clocking() re-invokes
                // process_type_switches over the I2S slots; its clocking
                // mismatch detection (master election / external-clock mode /
                // BCK pin) rebuilds only when something actually differs, so
                // this is a cheap no-op in the common case (including when the
                // type switch above already rebuilt everything).
                rebuild_i2s_output_clocking();
            }

            // Restart SPDIF RX if we suspended it above (outside the err==0
            // guard so a rejected payload still restores RX).  Skip if the
            // bulk payload queued an input-source change — that handler owns
            // RX restart then.  Mirrors preset_load_pending.
            if (suspended_spdif &&
                input_source_is_spdif(active_input_source) &&
                !input_source_change_pending) {
                spdif_input_start();
            }

            // I2S input mirror: restart with a freshly elected role.  A bulk
            // apply may have raised i2s_rx_pin_change_pending (pin) or
            // i2s_input_restart_pending (channel-count / multichannel); the
            // restart here already picks up the new config, so clear BOTH to
            // avoid a redundant deferred restart (and its extra mute).
            if (suspended_i2s &&
                active_input_source == INPUT_SOURCE_I2S &&
                !input_source_change_pending) {
                i2s_input_start(i2s_input_should_be_master());
                i2s_rx_pin_change_pending = false;
                i2s_input_restart_pending = false;
                // Master mode only: in slave mode the stored rate is dormant
                // (the external master defines the rate; the lock machinery
                // re-rates as needed).  Check the EFFECTIVE mode: this apply
                // may itself have deferred a flip to slave, in which case the
                // live global still reads master here.
                uint8_t eff_mode = i2s_clock_mode_change_pending
                                       ? pending_i2s_clock_mode
                                       : i2s_clock_mode;
                if (eff_mode != I2S_CLOCK_MODE_SLAVE &&
                    i2s_input_rate != audio_state.freq) {
                    pending_rate = i2s_input_rate;
                    __dmb();
                    rate_change_pending = true;
                }
            }
        }

        // Handle deferred I2S clock-mode change (master <-> slave).  Runs
        // BEFORE the input-source switch so a combined apply (preset / bulk
        // params changing both) records the mode dormantly first and the
        // source switch below brings everything up in one reset.  Only a
        // live I2S source needs work: restart the input with the new role,
        // rebuild the I2S output slots' clocking (process_type_switches also
        // applies the slave-mode MCK policy), and handle MCK for the
        // no-I2S-output case here.
        if (i2s_clock_mode_change_pending) {
            if (active_input_source != INPUT_SOURCE_I2S) {
                // Dormant apply: the mode only matters while I2S is the
                // source, so just record it for the next switch into I2S.
                i2s_clock_mode_change_pending = false;
                __dmb();
                if (i2s_clock_mode != pending_i2s_clock_mode) {
                    i2s_clock_mode = pending_i2s_clock_mode;
                    uint8_t wire_mode = i2s_clock_mode;
                    notify_param_write(offsetof(WireBulkParams,
                                                input_config.i2s_clock_mode),
                                       1, &wire_mode);
                }
            } else if (pending_i2s_clock_mode == i2s_clock_mode) {
                // Redundant request (e.g. a slave-then-master toggle pair
                // consumed in one go): the live hardware already matches;
                // consume without a spurious muted rebuild.  Every path that
                // arms the flag with an unchanged value relies on this (the
                // preset/bulk apply paths defer without pre-setting the
                // global, so a real change always differs here).
                i2s_clock_mode_change_pending = false;
                __dmb();
            } else if (pipeline_reset_ready()) {
                i2s_clock_mode_change_pending = false;
                __dmb();
                prepare_pipeline_reset(PRESET_MUTE_SAMPLES);
                if (i2s_input_get_state() != I2S_INPUT_INACTIVE) {
                    i2s_input_stop();
                }
                i2s_clock_mode = pending_i2s_clock_mode;
                rebuild_i2s_output_clocking();

                extern bool i2s_mck_enabled;
                extern uint16_t i2s_mck_multiplier;
                if (i2s_clock_mode == I2S_CLOCK_MODE_SLAVE) {
                    // External master owns all clocks; a locally generated
                    // MCK would be asynchronous to them.
                    audio_i2s_mck_set_enabled(false);
                    // Drop any pending stored-rate change that raced the flip
                    // (e.g. armed by a restore mirror or vendor 0xED just
                    // before this handler ran): in slave mode the lock
                    // machinery re-detects and re-arms the external rate.
                    rate_change_pending = false;
                } else {
                    // Back to master mode: return to the selected rate.  When
                    // the rates differ, perform_rate_change restores nominal
                    // dividers itself; when they match it is skipped, so
                    // restore the slave servo's divider trim explicitly (see
                    // restore_nominal_spdif_dividers).
                    if (i2s_input_rate != audio_state.freq) {
                        perform_rate_change(i2s_input_rate, true);
                    } else {
                        restore_nominal_spdif_dividers(audio_state.freq);
                    }
                    if (i2s_mck_enabled && i2s_input_should_be_master()) {
                        audio_i2s_mck_update_frequency(i2s_input_rate,
                                                       i2s_mck_multiplier);
                        audio_i2s_mck_set_enabled(true);
                    }
                }

                i2s_input_start(i2s_input_should_be_master());
                // The I2S main-loop block owns the drain/prefill/enable and
                // the DAC-mute release (preset_loading is set).

                // Notify at apply time (mirrors the input-source pattern).
                uint8_t wire_mode = i2s_clock_mode;
                notify_param_write(offsetof(WireBulkParams,
                                            input_config.i2s_clock_mode),
                                   1, &wire_mode);
            }
        }

        // Handle deferred ADAT clock-mode change.  Much lighter than the I2S
        // flip: the receiver is receive-only, so outputs need no structural
        // rebuild; restart the receiver in the new mode and restore the rate
        // authority.
        if (adat_clock_mode_change_pending) {
            if (pending_adat_clock_mode == adat_clock_mode) {
                adat_clock_mode_change_pending = false;
                __dmb();
            } else if (active_input_source != INPUT_SOURCE_ADAT) {
                // Dormant flip: nothing running, just adopt the mode.
                adat_clock_mode_change_pending = false;
                __dmb();
                adat_clock_mode = pending_adat_clock_mode;
                uint8_t wire_mode_p1 = (uint8_t)(adat_clock_mode + 1);
                notify_param_write(offsetof(WireBulkParams,
                                            input_config.adat_clock_mode_p1),
                                   1, &wire_mode_p1);
            } else if (pipeline_reset_ready()) {
                adat_clock_mode_change_pending = false;
                __dmb();
#if PICO_RP2350
                prepare_pipeline_reset(PRESET_MUTE_SAMPLES);
                adat_input_stop();
                adat_clock_mode = pending_adat_clock_mode;
                if (adat_clock_mode == ADAT_CLOCK_MODE_SLAVE) {
                    // The wire rate is re-detected after lock; drop any
                    // stored-rate change that raced the flip (mirrors the
                    // I2S clock-mode handler's clear).
                    rate_change_pending = false;
                } else {
                    // Back to master: return to the selected device rate and
                    // clear the slave servo's divider trim (see the I2S
                    // handler's identical branch for the rationale).
                    if (i2s_input_rate != audio_state.freq) {
                        perform_rate_change(i2s_input_rate, true);
                    } else {
                        restore_nominal_spdif_dividers(audio_state.freq);
                    }
                }
                adat_input_start();
                // The ADAT main-loop block owns drain/prefill/enable and the
                // DAC-mute release (preset_loading is set).

                uint8_t wire_mode_p1 = (uint8_t)(adat_clock_mode + 1);
                notify_param_write(offsetof(WireBulkParams,
                                            input_config.adat_clock_mode_p1),
                                   1, &wire_mode_p1);
#endif
            }
        }

        // Handle deferred input source switch
        if (input_source_change_pending) {
            uint8_t new_source = pending_input_source;
            uint8_t old_source = active_input_source;
            // Selectable, not just valid: a switch into a disabled SPDIF 2/3
            // (e.g. from a stored preset in INDEPENDENT mode) is consumed as
            // a no-op instead of bringing up an unclaimed GPIO.
            bool real_switch = (new_source != old_source) && input_source_selectable(new_source);

            if (!real_switch) {
                // No-op request (same / invalid source): consume without
                // engaging the mute so a redundant switch can't arm the
                // hardware mute and leave it asserted forever.
                input_source_change_pending = false;
                __dmb();
            } else if (pipeline_reset_ready()) {
                input_source_change_pending = false;
                __dmb();

                usb_audio_drain_ring();
                // Engage the soft mute and fence Core 1 after the drain before
                // the source teardown.  Gate held the DAC hardware mute;
                // idempotent re-assert (hold not re-armed).
                prepare_pipeline_reset(PRESET_MUTE_SAMPLES);

                // Stop old source hardware.  A SPDIF-to-SPDIF input switch
                // takes this stop branch and the is-spdif start branch below:
                // full stop/reset/start on the new input's GPIO, outputs
                // muted until the new source locks.
                if (input_source_is_spdif(old_source)) {
                    spdif_input_stop();
                    spdif_prefilling = false;

                    // Restore nominal output PIO dividers — the clock servo
                    // adjusts them during SPDIF input and they must be reset
                    // to prevent garbled audio on the new input source.
                    // perform_rate_change() recalculates all PIO dividers,
                    // filter coefficients, and resets the feedback loop.
                    // When the new source is I2S, go straight to its selected
                    // rate so the switch costs a single reset instead of two.
                    // In slave clock mode the stored I2S rate is dormant
                    // (auto-detected after lock), so stay at the current rate.
                    uint32_t target_rate = audio_state.freq;
                    if (new_source == INPUT_SOURCE_USB) {
                        target_rate = usb_audio_get_selected_rate();
                    } else if (new_source == INPUT_SOURCE_I2S &&
                               i2s_clock_mode != I2S_CLOCK_MODE_SLAVE) {
                        target_rate = i2s_input_rate;
                    } else if (new_source == INPUT_SOURCE_ADAT &&
                               adat_clock_mode != ADAT_CLOCK_MODE_SLAVE) {
                        // ADAT master mode shares the I2S device rate authority
                        target_rate = i2s_input_rate;
                    }
                    // Switching INTO I2S: defer the output restart + mute release
                    // to the I2S prefill block (active_input_source is still
                    // SPDIF here, so complete_pipeline_reset()'s I2S guard can't
                    // fire).  Switching INTO another SPDIF input: defer likewise,
                    // so the SPDIF lock/prefill block owns the restart and the
                    // outputs stay muted until the new input locks (same flow as
                    // USB to SPDIF).  Switching INTO ADAT: defer to its
                    // lock/prefill block the same way.  Switching to USB:
                    // complete_pipeline_reset() runs here.
                    perform_rate_change(target_rate,
                                        new_source == INPUT_SOURCE_I2S ||
                                        new_source == INPUT_SOURCE_ADAT ||
                                        input_source_is_spdif(new_source));
                    dsp_update_delay_samples((float)target_rate);

                    // Reset DSP state to prevent stale SPDIF data leaking
                    leveller_reset_pending = true;
                    pipeline_reset_cpu_metering();
                } else if (old_source == INPUT_SOURCE_I2S) {
                    i2s_input_stop();

                    // Slave-mode clock servo may have trimmed the SPDIF TX
                    // dividers to the departed external master's rate; the
                    // conditional perform_rate_change below skips equal-rate
                    // switches, so restore nominal explicitly (see
                    // restore_nominal_spdif_dividers; ADAT resyncs to nominal
                    // in the reset below and would drift against trimmed
                    // slots).  No-op cost in master mode, and harmless before
                    // a SPDIF target (its servo re-trims after lock).
                    restore_nominal_spdif_dividers(audio_state.freq);

                    // USB endpoint rate is retained while another source is
                    // active; apply it now without letting USB retune I2S live.
                    if (new_source == INPUT_SOURCE_USB) {
                        uint32_t target_rate = usb_audio_get_selected_rate();
                        if (target_rate != audio_state.freq) {
                            perform_rate_change(target_rate, false);
                            dsp_update_delay_samples((float)target_rate);
                        }
                    }

                    // MCK was running for the external source; stop it when
                    // no I2S output still needs it.
                    if (i2s_mck_enabled && i2s_input_should_be_master()) {
                        audio_i2s_mck_set_enabled(false);
                    }

                    // Reset DSP state to prevent stale I2S data leaking
                    leveller_reset_pending = true;
                    pipeline_reset_cpu_metering();
                }
#if PICO_RP2350
                else if (old_source == INPUT_SOURCE_ADAT) {
                    adat_input_stop();
                    adat_prefilling = false;

                    // Slave-mode servo may have trimmed the output dividers to
                    // the departed external clock; restore nominal (same
                    // rationale as the I2S branch above).
                    restore_nominal_spdif_dividers(audio_state.freq);

                    // USB endpoint rate is retained while another source is
                    // active; apply it now (mirrors the I2S branch).
                    if (new_source == INPUT_SOURCE_USB) {
                        uint32_t target_rate = usb_audio_get_selected_rate();
                        if (target_rate != audio_state.freq) {
                            perform_rate_change(target_rate, false);
                            dsp_update_delay_samples((float)target_rate);
                        }
                    }

                    // Reset DSP state to prevent stale ADAT data leaking
                    leveller_reset_pending = true;
                    pipeline_reset_cpu_metering();
                }
#endif

                // Regenerate input-channel default names for the new source —
                // ALL input channels, including the multichannel extras (2..7),
                // so 4/6/8-ch I2S and USB inputs relabel ("USB 3" -> "I2S 3"),
                // not just the stereo pair.  Custom names are preserved by
                // string-inequality.  RAM-only; persisted on REQ_PRESET_SAVE.
                {
                    extern uint8_t output_types[];
                    for (int ch = 0; ch < NUM_INPUT_CHANNELS; ch++) {
                        char old_default[PRESET_NAME_LEN];
                        char new_default[PRESET_NAME_LEN];
                        get_default_channel_name(ch, old_source, output_types, old_default);
                        get_default_channel_name(ch, new_source, output_types, new_default);
                        if (strcmp(old_default, new_default) == 0) continue;
                        if (strcmp(channel_names[ch], old_default) != 0) continue;
                        memcpy(channel_names[ch], new_default, PRESET_NAME_LEN);
                        notify_param_write(
                            (uint16_t)(offsetof(WireBulkParams, channel_names.names) + ch * WIRE_NAME_LEN),
                            WIRE_NAME_LEN, channel_names[ch]);
                    }
                }

                active_input_source = new_source;

                // Drop any pending rate change armed for the OLD source (e.g.
                // a USB SET_CUR that landed after this iteration's rate-change
                // handler already ran, or a detector arm that raced the
                // switch).  Left set, it would fire next iteration and retune
                // the pipeline under the NEW source; the original decorative-
                // USB bug through a side door.  Every path below re-derives
                // the rate itself: the SPDIF-source branch already called
                // perform_rate_change, SPDIF/I2S targets re-arm via their
                // lock/prefill machinery, and the USB target re-checks the
                // retained endpoint selection after its reset.  Mirrors the
                // clock-mode flip handler's clear.  Placed after the source
                // flip so an IRQ arm during the teardown above is also caught;
                // from here on, only the new source can legitimately arm.
                rate_change_pending = false;

                // Notify the LG Sound Sync module of the source change.
                // On a switch away from SPDIF it demotes to absent without
                // touching vol_mul (the audio_set_volume() thaw below
                // handles vol_mul on USB transitions); on a switch into
                // SPDIF it re-arms the streaks for fresh detection.
                lg_sound_sync_on_input_source_change(active_input_source);

                // I2S output slots change clocking at the slave-mode boundary
                // (internal clkout/dataout <-> external-clock program), and
                // MCK policy flips with them.  Rebuild whenever a source
                // switch crosses into or out of slave-clocked I2S.  active
                // source is already the NEW one, so process_type_switches
                // elects the correct clocking; the extra reset it performs
                // stays muted (preset_loading is set).
                if (i2s_clock_mode == I2S_CLOCK_MODE_SLAVE &&
                    (old_source == INPUT_SOURCE_I2S || new_source == INPUT_SOURCE_I2S)) {
                    rebuild_i2s_output_clocking();
                }

                // Start new source hardware
                if (input_source_is_spdif(new_source)) {
                    spdif_input_start();
                    // Don't complete_pipeline_reset yet — output stays muted
                    // until SPDIF lock is acquired (handled in polling block below)
                } else if (new_source == INPUT_SOURCE_I2S) {
                    // Apply rate + MCK and start the input, but do NOT enable
                    // outputs here.  The mute is already engaged (the
                    // prepare_pipeline_reset above), so the main-loop I2S block
                    // drains the outputs, prefills the consumer pools to 50%,
                    // and starts them in sync.  This gives I2S the same startup
                    // fill margin as USB/SPDIF instead of whatever low level the
                    // transient leaves.
                    i2s_input_bringup_prefill();
                }
#if PICO_RP2350
                else if (new_source == INPUT_SOURCE_ADAT) {
                    // Master mode: apply the selected device rate (shared with
                    // I2S master mode) if the old-source branch above did not
                    // already retune to it.  Slave mode detects the wire rate
                    // and arms its own change after lock.
                    if (adat_clock_mode == ADAT_CLOCK_MODE_MASTER &&
                        i2s_input_rate != audio_state.freq) {
                        perform_rate_change(i2s_input_rate, true);
                        dsp_update_delay_samples((float)i2s_input_rate);
                    }
                    adat_input_start();
                    // Outputs stay muted until frame sync locks; the main-loop
                    // ADAT block owns drain/prefill/enable + the mute release.
                }
#endif
                else {
                    // Switching to USB: flush stale ring data, complete reset
                    usb_audio_flush_ring();
                    complete_pipeline_reset();

                    // Thaw the cached host volume.  audio_set_volume() bails
                    // when source != USB, so any host SET_CUR Volume requests
                    // received during SPDIF mode were recorded into
                    // audio_state.volume but never applied to vol_mul or the
                    // loudness coefficient pointer.  Re-applying here brings
                    // the live gain path in line with what Windows last sent.
                    audio_set_volume(audio_state.volume);

                    // Close the switch-window race: a host SET_CUR that
                    // landed after the old-source branch read the retained
                    // selection but before active_input_source flipped above
                    // was recorded without arming (USB was not the active
                    // source yet), and the switch applied the older value.
                    // Re-check now that USB owns the pipeline; the deferred
                    // handler performs the retune.
                    uint32_t usb_rate = usb_audio_get_selected_rate();
                    if (usb_rate != audio_state.freq) {
                        pending_rate = usb_rate;
                        __dmb();
                        rate_change_pending = true;
                    }
                }

                printf("Input source: %u -> %u\n",
                       (unsigned)old_source, (unsigned)new_source);

                // Notify host that the active input source has changed.
                // Source tag carries through from the SET dispatcher if
                // this came from REQ_SET_INPUT_SOURCE.
                uint8_t wire_src = (uint8_t)active_input_source;
                notify_param_write(offsetof(WireBulkParams, input_config.input_source),
                                   1, &wire_src);
                // The active input channel count can change with the source
                // (USB alt count / I2S count / stereo for S/PDIF).  Push the
                // input-format event so a host driving relayout off it reacts to
                // the switch, mirroring the USB-alt-change path.
                notify_push_input_format(active_input_channel_count());
            }
        }

        // Handle deferred SPDIF RX pin hot-swap. Set when the ACTIVE SPDIF
        // input's pin is updated (by vendor command, bulk params apply, or
        // preset load) while that input is the live source; restart picks up
        // the new GPIO via spdif_rx_active_pin(). RX library teardown is too
        // heavy for USB ISR context, so we run stop/start here.
        // Persistence is now slot-scoped (REQ_PRESET_SAVE) — no flash
        // write happens here.
        extern volatile bool spdif_rx_pin_change_pending;
        if (spdif_rx_pin_change_pending) {
            spdif_rx_pin_change_pending = false;
            if (input_source_is_spdif(active_input_source) &&
                spdif_input_get_state() != SPDIF_INPUT_INACTIVE) {
                spdif_input_stop();
                spdif_prefilling = false;
                // Restart on the new pin; outputs stay muted until lock
                // is acquired (handled by the SPDIF polling block above).
                prepare_pipeline_reset(PRESET_MUTE_SAMPLES);
                spdif_input_start();
            }
        }

        // Handle deferred ADAT input restart: pin hot-swap while ADAT is the
        // live source (same shape as the SPDIF pin swap above).  When ADAT
        // is not the active source the new pin is only a stored preference.
        if (adat_input_restart_pending) {
            adat_input_restart_pending = false;
#if PICO_RP2350
            if (active_input_source == INPUT_SOURCE_ADAT &&
                adat_input_get_state() != ADAT_INPUT_INACTIVE) {
                adat_input_stop();
                adat_prefilling = false;
                prepare_pipeline_reset(PRESET_MUTE_SAMPLES);
                adat_input_start();
            }
#endif
        }

        // Handle deferred I2S RX restarts: data-pin hot-swap and full
        // restart after a BCK pin change in the input-master role.  Gated on
        // pipeline_reset_ready() like every other reset handler.  Outputs are
        // left muted after the restart; the main-loop I2S block above prefills
        // the consumer pools to 50% and enables them in sync, so no explicit
        // complete_pipeline_reset() is needed here.
        if ((i2s_rx_pin_change_pending || i2s_input_restart_pending) &&
            pipeline_reset_ready()) {
            i2s_rx_pin_change_pending = false;
            i2s_input_restart_pending = false;
            if (active_input_source == INPUT_SOURCE_I2S &&
                i2s_input_get_state() != I2S_INPUT_INACTIVE) {
                prepare_pipeline_reset(PRESET_MUTE_SAMPLES);
                i2s_input_stop();
                i2s_input_start(i2s_input_should_be_master());
            }
        }

        // Bench diagnostics: once-per-second CLKDIAG console dump (internally
        // throttled).  No-op when DSPI_CLOCK_DIAG is 0.
        clock_diag_poll();

        // LED heartbeat - toggle every ~1000 iterations
        static uint32_t loop_counter = 0;
        if (++loop_counter >= 1000) {
            loop_counter = 0;
            gpio_xor_mask(1u << 25);
        }
    }
}
