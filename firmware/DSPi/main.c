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
#include "spdif_rx.h"
#include "lg_sound_sync.h"
#include "dac_hw_mute.h"
#include "dsp_pipeline.h"
#include "flash_clkdiv.h"
#include "flash_storage.h"
#include "pico/audio_i2s_multi.h"
#include "pdm_generator.h"
#include "usb_audio.h"
#include "notify.h"
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

static void perform_rate_change(uint32_t new_freq) {
    switch (new_freq) { case 44100: case 48000: case 96000: break; default: new_freq = 44100; }

    // Engage mute and wait for Core 1 EQ worker to drain before touching
    // filter coefficients or PIO dividers. Without this bracket, old-rate
    // consumer-pool buffers would play at the new PIO bit-clock for ~16ms
    // (audible pitch shift + resync click).
    prepare_pipeline_reset(PRESET_MUTE_SAMPLES);

    // Update the audio format so pico_audio_spdif can update the PIO divider
    audio_format_48k.sample_freq = new_freq;

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
    pdm_update_clock(new_freq);

    // Atomically update all I2S instances and restart in sync (avoids brief
    // master/slave divider mismatch from lazy per-instance callbacks)
    audio_i2s_update_all_frequencies(new_freq);

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
    if (!spdif_prefilling) {
        complete_pipeline_reset();
    }
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

// SPDIF input prefill: outputs disabled while consumer buffers fill to 50%
static bool spdif_prefilling = false;

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
    extern uint8_t i2s_bck_pin;
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

    bool any_change = false;
    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        if (target_types[i] != current_types[i]) {
            any_change = true;
            break;
        }
    }
    if (!any_change) return;

    output_type_switch_in_progress = true;
    __dmb();
    const bool usb_irq_was_enabled = irq_is_enabled(USBCTRL_IRQ);
    irq_set_enabled(USBCTRL_IRQ, false);

    // Deterministic master policy: lowest-index active I2S slot is master.
    int target_master_slot = -1;
    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        if (target_types[i] == OUTPUT_TYPE_I2S) {
            target_master_slot = i;
            break;
        }
    }

    usb_audio_drain_ring();
    prepare_pipeline_reset(PRESET_MUTE_SAMPLES);

    // Suspend SPDIF RX across the type-switch window.  The DMA IRQ disable
    // below kills DMA_IRQ_1 servicing (which RX shares with SPDIF TX), and
    // the alarm-pool decode-timeout alarms in pico_spdif_rx use a separate
    // timer IRQ that can fire mid-transition and access PIO/DMA state we
    // are mutating.  Stop cleanly here; restart at the end if it was running.
    // If a caller already stopped RX (e.g. preset_load_pending across the
    // flash blackout), state==INACTIVE and we leave it that way — caller
    // is responsible for restart.
    bool rx_was_running = (active_input_source == INPUT_SOURCE_SPDIF &&
                           spdif_input_get_state() != SPDIF_INPUT_INACTIVE);
    if (rx_was_running) {
        spdif_input_stop();
        spdif_prefilling = false;
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
            // I2S → SPDIF: teardown the I2S instance
            audio_i2s_teardown(i2s_instance_ptrs[i]);
        } else {
            // SPDIF → I2S: disable and unclaim the SPDIF SM
            audio_spdif_instance_t *spdif_inst = spdif_instance_ptrs[i];
            audio_spdif_set_enabled(spdif_inst, false);
            dma_irqn_set_channel_enabled(spdif_inst->dma_irq, spdif_inst->dma_channel, false);
            dma_channel_abort(spdif_inst->dma_channel);
            if (spdif_inst->playing_buffer) {
                give_audio_buffer(spdif_inst->consumer_pool, spdif_inst->playing_buffer);
                spdif_inst->playing_buffer = NULL;
            }
            spdif_reset_consumer_pipeline(spdif_inst);
            dma_irqn_acknowledge_channel(spdif_inst->dma_irq, spdif_inst->dma_channel);
            pio_sm_unclaim(spdif_inst->pio, spdif_inst->pio_sm);
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
                if (!inst->consumer_pool || inst->clock_master != want_master) {
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
                    .clock_pin_base = i2s_bck_pin,
                    .dma_channel = i + 8,
                    .pio_sm = i,
                    .pio = PICO_AUDIO_SPDIF_PIO,
                    .dma_irq = PICO_AUDIO_I2S_DMA_IRQ,
                    .clock_master = want_master,
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
            // Setup SPDIF on slot where I2S was torn down
            audio_spdif_instance_t *spdif_inst = spdif_instance_ptrs[i];
            pio_sm_claim(spdif_inst->pio, spdif_inst->pio_sm);
            audio_spdif_change_pin(spdif_inst, output_pins[i]);
            // Re-formats the slot's shared static consumer pool back to S/PDIF
            // (re-points buffers + re-fills IEC-60958 framing) and re-wires the
            // connection — no alloc/free. The pool was last formatted for I2S.
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
            int ch = NUM_INPUT_CHANNELS + slot * 2 + side;
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

    // Start/stop MCK based on whether any slot is now I2S
    bool any_i2s = false;
    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        if (output_types[i] == OUTPUT_TYPE_I2S) { any_i2s = true; break; }
    }
    if (any_i2s && i2s_mck_enabled) {
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
        active_input_source == INPUT_SOURCE_SPDIF &&
        !input_source_change_pending) {
        spdif_input_start();
    }

    printf("Type switch complete: mask=0x%02x\n", change_mask);
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
    bool rx_was_running = (active_input_source == INPUT_SOURCE_SPDIF &&
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
    complete_pipeline_reset();

    // Restart SPDIF RX if we suspended it (unless the source changed mid-op).
    if (rx_was_running &&
        active_input_source == INPUT_SOURCE_SPDIF &&
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
    __dmb();
    dac_hw_mute_assert();
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

// Three-phase pipeline reset.  The IRQ-disabled critical section in
// Phase 2 is intentionally tiny — only the synchronized PIO SM start
// needs atomicity (preserves CLAUDE.md's slot-alignment invariant for
// single-type configs).  Phase 1 (per-slot teardown) and Phase 3 (USB
// feedback reset) run with interrupts enabled.
//
// Why keep blackout small: USB audio class ISRs continue to drain
// packets into the audio_ring throughout Phase 1, eliminating a ~1 ms
// USB starvation window that previously compounded the audible I2S DAC
// click on input-source switches.
//
// Phase 2 outer save_and_disable_interrupts wraps BOTH library calls
// (audio_spdif_enable_sync + audio_i2s_enable_sync).  Although each
// library has its own inner save_and_disable_interrupts around its
// pio_enable_sm_mask_in_sync call, the outer wrap exists to bracket
// the cross-type SPDIF<->I2S boundary: without it, a stale DMA IRQ or
// SOF could fire between the two enable_sync calls and either disturb
// USB feedback baselining or let one type's just-primed DMA complete
// an extra transfer before the other type's clocks start, producing a
// 1-frame inter-type skew.  Do NOT split this critical section across
// the two calls "to shrink it further" — it would silently break
// mixed-output configs and the regression would only surface on
// installations actually running both output types.
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

    // Phase 2: tiny IRQ-disabled section — synchronized PIO start.
    // See block comment above for why this wraps BOTH enable_sync calls.
    uint32_t flags = save_and_disable_interrupts();
    if (spdif_count) audio_spdif_enable_sync(spdif_sync, spdif_count);
    if (i2s_count) audio_i2s_enable_sync(i2s_sync, i2s_count);
    restore_interrupts(flags);

    // Phase 3: USB feedback reset.  See B1 note in block comment above
    // for the (bounded) race with the SOF ISR.
    reset_usb_feedback_loop();

    // Phase 4: begin hardware-mute release.  Order is critical: clocks
    // must be running (Phase 2 completed) BEFORE the mute pin deasserts.
    // If release_ms > 0, dac_hw_mute_release() leaves the pin asserted
    // and returns; dac_hw_mute_tick() deasserts it later so the input
    // pipeline keeps draining while the DAC remains muted.
    dac_hw_mute_release();
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

    if (spdif_count) audio_spdif_enable_sync(spdif_sync, spdif_count);
    if (i2s_count) audio_i2s_enable_sync(i2s_sync, i2s_count);

    restore_interrupts(flags);
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
// never expires before the pipeline is reset.
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

static void prepare_flash_write_operation(void) {
    // Drain the current input source once before the mute engages so the
    // envelope starts from the freshest possible state.
    if (active_input_source == INPUT_SOURCE_USB) {
        usb_audio_drain_ring();
    } else if (active_input_source == INPUT_SOURCE_SPDIF) {
        spdif_input_poll();
    }

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
    bool spdif_streaming = (active_input_source == INPUT_SOURCE_SPDIF) &&
                           (spdif_input_get_state() == SPDIF_INPUT_LOCKED);
    if (usb_streaming || spdif_streaming) {
        uint64_t start_us = time_us_64();
        while ((time_us_64() - start_us) < FLASH_WRITE_FADE_SETTLE_US
               || !dac_hw_mute_hold_elapsed()) {
            if (active_input_source == INPUT_SOURCE_USB) {
                usb_audio_drain_ring();
            } else {
                spdif_input_poll();
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
    if (active_input_source == INPUT_SOURCE_SPDIF &&
        spdif_input_get_state() != SPDIF_INPUT_INACTIVE) {
        spdif_input_stop();
        spdif_prefilling = false;
        spdif_suspended_for_flash = true;
    }
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
    if (active_input_source != INPUT_SOURCE_SPDIF) return;
    if (input_source_change_pending) return;

    // preset_loading=true is still set from prepare_pipeline_reset(); the
    // main loop's SPDIF lock-acquisition block will fire when lock returns
    // and run the normal drain+prefill handshake.
    spdif_input_start();
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
//
// Completion paths that run a full pipeline reset on USB
// (complete_flash_write_operation_full()) release implicitly inside
// complete_pipeline_reset() and need not call this.  It exists for completion
// paths that keep outputs running through the operation — today the
// light/metadata flash path — where the release is otherwise easy to forget
// (its omission once left the DAC silent indefinitely; see git 833a51a).
static inline void release_hw_mute_if_outputs_live(void) {
    if (active_input_source != INPUT_SOURCE_SPDIF) {
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

    if (active_input_source == INPUT_SOURCE_SPDIF) {
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
    // Symmetry with prepare_flash_write_operation(): restart SPDIF RX if
    // it was suspended for the blackout.
    resume_spdif_after_flash();

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

    // [CRITICAL FIX]
    // Initialize USB/SPDIF *BEFORE* PDM.
    // SPDIF requires DMA Channel 0 (hardcoded in config).
    // If PDM inits first, it steals Ch 0 via dma_claim_unused_channel(), causing SPDIF to panic/crash.
    usb_sound_card_init();

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

    // Initial loudness table computation (uses loaded or default params)
    loudness_recompute_table(loudness_ref_spl, loudness_intensity_pct, 48000.0f);
    if (loudness_enabled && loudness_active_table) {
        audio_set_volume(audio_state.volume);  // Re-select loudness coefficients
    }

    // Initial volume leveller setup (uses loaded or default params)
    leveller_compute_coefficients(&leveller_coeffs, (const LevellerConfig *)&leveller_config, 48000.0f);
    leveller_reset_state(&leveller_state);
    leveller_bypassed = !leveller_config.enabled;

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
    if (active_input_source == INPUT_SOURCE_SPDIF) {
        prepare_pipeline_reset(PRESET_MUTE_SAMPLES);
        spdif_input_start();
    }

    // Baseline the notification shadow from the fully-initialised live state.
    // Must come after preset_boot_load() / apply_factory_defaults() so any
    // subsequent param_write call sees a truthful baseline and only emits
    // notifications on real changes.
    notify_init();
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

        // LG Sound Sync detection tick — internally throttled and
        // gated on (feature enabled && SPDIF input && SPDIF locked).
        // Cheap on the not-applicable path; safe to call every loop.
        lg_sound_sync_tick();

        // DAC hardware-mute deadline check — releases async test pulses
        // and post-clock-restart release holds on schedule.  Two loads +
        // branches when no deadline is in flight (the steady state).
        dac_hw_mute_tick();

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
        if (active_input_source == INPUT_SOURCE_SPDIF) {
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

        // Handle EQ parameter updates from USB
        if (eq_update_pending) {
            EqParamPacket p = pending_packet;
            eq_update_pending = false;
            filter_recipes[p.channel][p.band] = p;

            // Shadow-notify the host of the new band params.  The wire
            // struct layout (WireBandParams: type, reserved[3], freq, Q, gain_db)
            // differs from EqParamPacket, so we marshal into a temp.
            {
                WireBandParams wbp;
                memset(&wbp, 0, sizeof(wbp));
                wbp.type = (uint8_t)p.type;
                wbp.bypass = (p.bypass == 1) ? 1 : 0;
                wbp.freq = p.freq;
                wbp.q = p.Q;
                wbp.gain_db = p.gain_db;
                uint16_t off = (uint16_t)(offsetof(WireBulkParams, eq)
                    + ((uint16_t)p.channel * WIRE_MAX_BANDS + p.band) * sizeof(WireBandParams));
                notify_param_write(off, sizeof(WireBandParams), &wbp);
            }

            // If updating a Core 1 EQ channel, wait for Core 1 to finish
            // current work before modifying coefficients
            bool is_core1_channel = (p.channel >= (CH_OUT_1 + CORE1_EQ_FIRST_OUTPUT) &&
                                     p.channel <= (CH_OUT_1 + CORE1_EQ_LAST_OUTPUT));
            if (is_core1_channel && core1_mode == CORE1_MODE_EQ_WORKER) {
                // Spin-wait until Core 1 is idle (work_done or no work dispatched)
                while (core1_eq_work.work_ready && !core1_eq_work.work_done) {
                    tight_loop_contents();
                }
                __dmb();
            }

            uint32_t flags = save_and_disable_interrupts();
            dsp_compute_coefficients(&p, &filters[p.channel][p.band], (float)audio_state.freq);

            // Recalculate channel bypass flag
            bool all_bypassed = true;
            for (int b = 0; b < channel_band_counts[p.channel]; b++) {
                if (!filters[p.channel][b].bypass) {
                    all_bypassed = false;
                    break;
                }
            }
            channel_bypassed[p.channel] = all_bypassed;

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
            perform_rate_change(r);
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

        // Handle crossfeed coefficient updates
        if (crossfeed_update_pending) {
            crossfeed_update_pending = false;
            crossfeed_compute_coefficients(&crossfeed_state, (const CrossfeedConfig *)&crossfeed_config, (float)audio_state.freq);
            // Update bypass flag atomically
            crossfeed_bypassed = !crossfeed_config.enabled;
        }

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
                if (active_input_source == INPUT_SOURCE_SPDIF &&
                    spdif_input_get_state() != SPDIF_INPUT_INACTIVE) {
                    spdif_input_stop();
                    spdif_prefilling = false;
                    suspended_spdif = true;
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
                } else if (active_input_source == INPUT_SOURCE_SPDIF) {
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

                // Restart SPDIF RX once, after all type-switch / pipeline
                // work is done.  Skipped if the preset switched the input
                // source — input_source_change_pending will manage RX
                // when its handler fires below.
                if (suspended_spdif &&
                    active_input_source == INPUT_SOURCE_SPDIF &&
                    !input_source_change_pending) {
                    spdif_input_start();
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
            }

            extern volatile bool factory_reset_pending;
            if (factory_reset_pending && pipeline_reset_ready()) {
                factory_reset_pending = false;
                __dmb();

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
                if (active_input_source == INPUT_SOURCE_SPDIF &&
                    spdif_input_get_state() != SPDIF_INPUT_INACTIVE) {
                    spdif_input_stop();
                    spdif_prefilling = false;
                    suspended_spdif = true;
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

                // Restart SPDIF RX if we suspended it above (skip if an input-
                // source change is pending — that handler manages RX).
                if (suspended_spdif &&
                    active_input_source == INPUT_SOURCE_SPDIF &&
                    !input_source_change_pending) {
                    spdif_input_start();
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
            if (active_input_source == INPUT_SOURCE_SPDIF &&
                spdif_input_get_state() != SPDIF_INPUT_INACTIVE) {
                spdif_input_stop();
                spdif_prefilling = false;
                suspended_spdif = true;
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
                } else if (active_input_source == INPUT_SOURCE_SPDIF) {
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
            }

            // Restart SPDIF RX if we suspended it above (outside the err==0
            // guard so a rejected payload still restores RX).  Skip if the
            // bulk payload queued an input-source change — that handler owns
            // RX restart then.  Mirrors preset_load_pending.
            if (suspended_spdif &&
                active_input_source == INPUT_SOURCE_SPDIF &&
                !input_source_change_pending) {
                spdif_input_start();
            }
        }

        // Handle deferred input source switch
        if (input_source_change_pending) {
            uint8_t new_source = pending_input_source;
            uint8_t old_source = active_input_source;
            bool real_switch = (new_source != old_source) && input_source_valid(new_source);

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

                // Stop old source hardware
                if (old_source == INPUT_SOURCE_SPDIF) {
                    spdif_input_stop();
                    spdif_prefilling = false;

                    // Restore nominal output PIO dividers — the clock servo
                    // adjusts them during SPDIF input and they must be reset
                    // to prevent garbled audio on the new input source.
                    // perform_rate_change() recalculates all PIO dividers,
                    // filter coefficients, and resets the feedback loop.
                    perform_rate_change(audio_state.freq);
                    dsp_update_delay_samples((float)audio_state.freq);

                    // Reset DSP state to prevent stale SPDIF data leaking
                    leveller_reset_pending = true;
                    pipeline_reset_cpu_metering();
                }

                // Regenerate input-channel default names for the new source.
                // Custom names are preserved by string-inequality.  RAM-only;
                // persisted on REQ_PRESET_SAVE.
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

                // Notify the LG Sound Sync module of the source change.
                // On a switch away from SPDIF it demotes to absent without
                // touching vol_mul (the audio_set_volume() thaw below
                // handles vol_mul on USB transitions); on a switch into
                // SPDIF it re-arms the streaks for fresh detection.
                lg_sound_sync_on_input_source_change(active_input_source);

                // Start new source hardware
                if (new_source == INPUT_SOURCE_SPDIF) {
                    spdif_input_start();
                    // Don't complete_pipeline_reset yet — output stays muted
                    // until SPDIF lock is acquired (handled in polling block below)
                } else {
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
                }

                printf("Input source: %u -> %u\n",
                       (unsigned)old_source, (unsigned)new_source);

                // Notify host that the active input source has changed.
                // Source tag carries through from the SET dispatcher if
                // this came from REQ_SET_INPUT_SOURCE.
                uint8_t wire_src = (uint8_t)active_input_source;
                notify_param_write(offsetof(WireBulkParams, input_config.input_source),
                                   1, &wire_src);
            }
        }

        // Handle deferred SPDIF RX pin hot-swap. Set when spdif_rx_pin is
        // updated (by vendor command, bulk params apply, or preset load)
        // while INPUT_SOURCE_SPDIF is active. RX library teardown is too
        // heavy for USB ISR context, so we run stop/start here.
        // Persistence is now slot-scoped (REQ_PRESET_SAVE) — no flash
        // write happens here.
        extern volatile bool spdif_rx_pin_change_pending;
        if (spdif_rx_pin_change_pending) {
            spdif_rx_pin_change_pending = false;
            if (active_input_source == INPUT_SOURCE_SPDIF &&
                spdif_input_get_state() != SPDIF_INPUT_INACTIVE) {
                spdif_input_stop();
                spdif_prefilling = false;
                // Restart on the new pin; outputs stay muted until lock
                // is acquired (handled by the SPDIF polling block above).
                prepare_pipeline_reset(PRESET_MUTE_SAMPLES);
                spdif_input_start();
            }
        }

        // LED heartbeat - toggle every ~1000 iterations
        static uint32_t loop_counter = 0;
        if (++loop_counter >= 1000) {
            loop_counter = 0;
            gpio_xor_mask(1u << 25);
        }
    }
}
