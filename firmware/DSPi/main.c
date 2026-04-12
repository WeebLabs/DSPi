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
#include "dsp_pipeline.h"
#include "flash_storage.h"
#include "pico/audio_i2s_multi.h"
#include "pdm_generator.h"
#include "usb_audio.h"
#include "loudness.h"
#include "crossfeed.h"
#include "leveller.h"
#include "bulk_params.h"
#include "pico/audio_spdif.h"
#include "usb_feedback_controller.h"

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

// USB SOF IRQ — measures device clock vs host clock for async feedback
void __not_in_flash_func(usb_sof_irq)(void) {
    extern audio_spdif_instance_t *spdif_instance_ptrs[];
    extern uint8_t output_types[];

    // During output-type reconfiguration, slot ownership and DMA assignment are
    // transiently inconsistent by design (teardown/setup in progress). Skip this
    // frame's feedback sample to avoid reading partially transitioned state.
    if (output_type_switch_in_progress) return;

    // Read DMA word count from slot 0 (SPDIF or I2S)
    volatile uint32_t *p_words_consumed;
    uint32_t xfer_words;
    uint8_t dma_ch;
    uint8_t slot0_type = output_types[0];
    uint32_t rate_shift;

    if (slot0_type == OUTPUT_TYPE_I2S) {
        extern audio_i2s_instance_t *i2s_instance_ptrs[];
        audio_i2s_instance_t *inst = i2s_instance_ptrs[0];
        p_words_consumed = &inst->words_consumed;
        xfer_words = inst->current_transfer_words;
        dma_ch = inst->dma_channel;
        rate_shift = 13;      // I2S: << (16-3)
    } else {
        audio_spdif_instance_t *inst = spdif_instance_ptrs[0];
        p_words_consumed = &inst->words_consumed;
        xfer_words = inst->current_transfer_words;
        dma_ch = inst->dma_channel;
        rate_shift = 12;      // SPDIF: << (16-4)
    }

    // Sub-buffer-precise DMA word total for slot 0
    uint32_t remaining = dma_channel_hw_addr(dma_ch)->transfer_count;
    uint32_t current_total = *p_words_consumed + (xfer_words - remaining);

    fb_ctrl_sof_update(&fb_ctrl, current_total, rate_shift, spdif0_consumer_fill);

    // Publish to endpoint-facing variables
    uint32_t fb_10_14 = fb_ctrl_get_10_14(&fb_ctrl);
    if (fb_10_14)
        feedback_10_14 = fb_10_14;
}

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

// 96 kHz + 256x (24.576 MHz MCK) is unstable on current hardware/clocking, so
// force 128x whenever that combination is encountered from persisted state.
static void sanitize_mck_multiplier_for_rate(uint32_t sample_rate_hz) {
    extern uint16_t i2s_mck_multiplier;
    if (sample_rate_hz >= 96000u && i2s_mck_multiplier == 256u) {
        i2s_mck_multiplier = 128u;
        printf("MCK 256x not supported at %lu Hz; forcing 128x\n",
               (unsigned long)sample_rate_hz);
    }
}

static void perform_rate_change(uint32_t new_freq) {
    switch (new_freq) { case 44100: case 48000: case 96000: break; default: new_freq = 44100; }

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

    // Update MCK frequency for new sample rate (if enabled)
    extern bool i2s_mck_enabled;
    extern uint16_t i2s_mck_multiplier;
    if (i2s_mck_enabled) {
        sanitize_mck_multiplier_for_rate(new_freq);
        audio_i2s_mck_update_frequency(new_freq, i2s_mck_multiplier);
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
                audio_i2s_connect_extra(i2s_instance_ptrs[i], producer_pools[i],
                                        false, SPDIF_CONSUMER_BUFFER_COUNT, NULL);
                if (had_i2s) {
                    printf("Slot %d %s I2S master\n", i, want_master ? "promoted to" : "demoted to");
                }
            }
        } else if (had_i2s) {
            // Setup SPDIF on slot where I2S was torn down
            audio_spdif_instance_t *spdif_inst = spdif_instance_ptrs[i];
            pio_sm_claim(spdif_inst->pio, spdif_inst->pio_sm);
            audio_spdif_change_pin(spdif_inst, output_pins[i]);
            audio_complete_connection(&spdif_inst->connection.core,
                                      producer_pools[i],
                                      spdif_inst->consumer_pool);
            memset(i2s_instance_ptrs[i], 0, sizeof(audio_i2s_instance_t));
        }
    }

    memcpy(output_types, target_types, NUM_SPDIF_INSTANCES);

    // Start/stop MCK based on whether any slot is now I2S
    bool any_i2s = false;
    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        if (output_types[i] == OUTPUT_TYPE_I2S) { any_i2s = true; break; }
    }
    if (any_i2s && i2s_mck_enabled) {
        sanitize_mck_multiplier_for_rate(audio_state.freq);
        audio_i2s_mck_set_enabled(true);
        audio_i2s_mck_update_frequency(audio_state.freq, i2s_mck_multiplier);
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

    printf("Type switch complete: mask=0x%02x\n", change_mask);
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
// Waits for Core 1 EQ worker to finish, then engages the audio mute.
static void prepare_pipeline_reset(uint32_t mute_samples) {
    if (core1_mode == CORE1_MODE_EQ_WORKER) {
        while (core1_eq_work.work_ready && !core1_eq_work.work_done)
            tight_loop_contents();
        __dmb();
    }
    preset_mute_counter = mute_samples;
    preset_loading = true;
    __dmb();
}

// Phase 2: drain all consumer pipelines, restart outputs in sync, reset
// USB feedback.  The entire sequence runs with interrupts disabled so that
// USB audio packets cannot produce into already-drained pools (which would
// create a persistent inter-slot fill offset).
static void complete_pipeline_reset(void) {
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

            i2s_reset_consumer_pipeline(inst);
            dma_irqn_acknowledge_channel(inst->dma_irq, inst->dma_channel);
            dma_irqn_set_channel_enabled(inst->dma_irq, inst->dma_channel, true);

            i2s_sync[i2s_count++] = inst;
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

            spdif_reset_consumer_pipeline(inst);
            dma_irqn_acknowledge_channel(inst->dma_irq, inst->dma_channel);
            dma_irqn_set_channel_enabled(inst->dma_irq, inst->dma_channel, true);

            spdif_sync[spdif_count++] = inst;
        }
    }

    if (spdif_count) {
        audio_spdif_enable_sync(spdif_sync, spdif_count);
    }
    if (i2s_count) {
        audio_i2s_enable_sync(i2s_sync, i2s_count);
    }

    reset_usb_feedback_loop();

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

static void prepare_flash_write_operation(void) {
    usb_audio_drain_ring();
    prepare_pipeline_reset(samples_for_duration_ms(audio_state.freq,
                                                   FLASH_WRITE_PREMUTE_MS));

    // If stream is active, give the mute envelope time to ramp down before
    // flash write lockout halts packet processing.
    extern volatile bool sync_started;
    if (sync_started) {
        uint64_t start_us = time_us_64();
        while ((time_us_64() - start_us) < FLASH_WRITE_FADE_SETTLE_US) {
            usb_audio_drain_ring();
            tight_loop_contents();
        }
    }

    // Consume any packets that arrived during settle so flash starts from
    // the quietest possible output state.
    usb_audio_drain_ring();
}

// Full completion path: drain/restart all output consumer pipelines and reset
// feedback state. Use this for operations that materially affect runtime audio
// continuity (preset save/delete and legacy save command compatibility path).
static void complete_flash_write_operation_full(void) {
    complete_pipeline_reset();
}

// Light completion path: keep the mute envelope active, but skip a full output
// pipeline rebuild. Suitable for metadata-only flash writes (names/startup flags)
// where DSP/output topology is unchanged.
static inline void complete_flash_write_operation_light(void) {
    // Intentionally empty.
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

    // If the loaded preset has SPDIF as input source, start RX hardware.
    // Output remains muted until lock is acquired (handled in main loop).
    if (active_input_source == INPUT_SOURCE_SPDIF) {
        spdif_input_start();
    }
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

        // Drain USB audio ring — highest priority (only when USB is active input).
        // USB ISR pushes raw packets into the ring; we run the full DSP
        // pipeline here in main-loop context instead of USB IRQ context.
        if (active_input_source == INPUT_SOURCE_USB) {
            usb_audio_drain_ring();
        }

        // Poll SPDIF input when active
        if (active_input_source == INPUT_SOURCE_SPDIF) {
            SpdifInputState rx_state = spdif_input_get_state();

            // Handle lock acquisition: check for rate change, then unmute
            if (rx_state == SPDIF_INPUT_LOCKED && preset_loading) {
                // Rate change check before unmuting
                spdif_input_check_rate_change();
                // Unmute once FIFO has built up sufficiently
                if (spdif_rx_get_fifo_count() >= SPDIF_RX_FIFO_SIZE / 4) {
                    complete_pipeline_reset();
                }
            }

            // Handle lock loss: mute output immediately
            if (rx_state == SPDIF_INPUT_RELOCKING && !preset_loading) {
                prepare_pipeline_reset(PRESET_MUTE_SAMPLES);
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

            extern volatile bool flash_set_include_pins_pending;
            if (flash_set_include_pins_pending) {
                uint8_t val;
                uint32_t f = save_and_disable_interrupts();
                extern uint8_t flash_set_include_pins_val;
                val = flash_set_include_pins_val;
                flash_set_include_pins_pending = false;
                restore_interrupts(f);
                prepare_flash_write_operation();
                preset_set_include_pins(val);
                complete_flash_write_operation_light();
            }

            extern volatile bool flash_set_include_master_vol_pending;
            if (flash_set_include_master_vol_pending) {
                uint8_t val;
                uint32_t f = save_and_disable_interrupts();
                extern uint8_t flash_set_include_master_vol_val;
                val = flash_set_include_master_vol_val;
                flash_set_include_master_vol_pending = false;
                restore_interrupts(f);
                prepare_flash_write_operation();
                preset_set_include_master_volume(val);
                complete_flash_write_operation_light();
            }
        }

        // Handle EQ parameter updates from USB
        if (eq_update_pending) {
            EqParamPacket p = pending_packet;
            eq_update_pending = false;
            filter_recipes[p.channel][p.band] = p;

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

        // Handle sample rate changes
        if (rate_change_pending) {
            uint32_t r = pending_rate;
            rate_change_pending = false;
            usb_audio_drain_ring();  // Process old-rate packets before clock switch
            perform_rate_change(r);
        }

        // Handle loudness table recomputation
        if (loudness_recompute_pending) {
            loudness_recompute_pending = false;
            loudness_recompute_table(loudness_ref_spl, loudness_intensity_pct, (float)audio_state.freq);
            // Update coefficient pointer for current volume
            if (loudness_enabled && loudness_active_table) {
                audio_set_volume(audio_state.volume);
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
            if (stream_restart_resync_pending) {
                stream_restart_resync_pending = false;
                __dmb();

                usb_audio_drain_ring();   // Process remaining packets
                usb_audio_flush_ring();   // Discard stale data from previous stream

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

            if (preset_load_pending) {
                preset_load_pending = false;
                __dmb();

                extern uint8_t output_types[];

                // Snapshot current output types BEFORE load so we can detect
                // which slots need hardware reconfiguration afterward.
                uint8_t old_types[NUM_SPDIF_INSTANCES];
                memcpy(old_types, output_types, NUM_SPDIF_INSTANCES);

                usb_audio_drain_ring();
                prepare_pipeline_reset(PRESET_MUTE_SAMPLES);

                // Apply the new preset: overwrites all DSP state (EQ, delays,
                // matrix, gains, output_types[]), recalculates filter coefficients,
                // transitions Core 1 mode, and writes the directory to flash.
                preset_load(pending_preset_load_slot);

                // Presets can carry persisted raw MCK=0 (256x). Clamp invalid
                // 96 kHz combinations and apply the effective MCK divider now
                // so no-type-change loads still update clock state.
                {
                    extern bool i2s_mck_enabled;
                    extern uint16_t i2s_mck_multiplier;
                    if (i2s_mck_enabled) {
                        sanitize_mck_multiplier_for_rate(audio_state.freq);
                        audio_i2s_mck_update_frequency(audio_state.freq, i2s_mck_multiplier);
                    }
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
                    uint8_t new_types[NUM_SPDIF_INSTANCES];
                    memcpy(new_types, output_types, NUM_SPDIF_INSTANCES);
                    memcpy(output_types, old_types, NUM_SPDIF_INSTANCES);
                    process_type_switches(change_mask, new_types);
                } else {
                    // No type changes — just resync pipelines
                    complete_pipeline_reset();
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
            if (factory_reset_pending) {
                factory_reset_pending = false;
                __dmb();

                extern uint8_t output_types[];

                // Snapshot current output types before reset clears them to
                // all-SPDIF so we can detect I2S→SPDIF transitions.
                uint8_t old_types[NUM_SPDIF_INSTANCES];
                memcpy(old_types, output_types, NUM_SPDIF_INSTANCES);

                usb_audio_drain_ring();
                prepare_pipeline_reset(PRESET_MUTE_SAMPLES);

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
            }
        }

        // Handle output type change (deferred from USB ISR — needs heap allocation)
        {
            extern volatile uint8_t output_type_change_mask;
            extern volatile uint8_t pending_output_types[];

            if (output_type_change_mask) {
                uint8_t mask = output_type_change_mask;
                output_type_change_mask = 0;
                __dmb();
                process_type_switches(mask, (const uint8_t *)pending_output_types);
            }
        }

        // Handle bulk parameter SET (deferred from USB IRQ)
        if (bulk_params_pending) {
            bulk_params_pending = false;

            usb_audio_drain_ring();  // Process before full state swap
            prepare_pipeline_reset(PRESET_MUTE_SAMPLES);

            // Apply the received parameters (pin config gated by include_pins setting)
            uint16_t _occ; uint8_t _m, _d, _la, inc_pins, _inc_mv;
            preset_get_directory(&_occ, &_m, &_d, &_la, &inc_pins, &_inc_mv);
            int err = bulk_params_apply((const WireBulkParams *)bulk_param_buf, inc_pins != 0);
            if (err == 0) {
                float rate = (float)audio_state.freq;
                dsp_recalculate_all_filters(rate);
                dsp_update_delay_samples(rate);

                // Bulk apply can also update persisted MCK settings. Keep MCK
                // clock state coherent even when output types are unchanged.
                {
                    extern bool i2s_mck_enabled;
                    extern uint16_t i2s_mck_multiplier;
                    if (i2s_mck_enabled) {
                        sanitize_mck_multiplier_for_rate(audio_state.freq);
                        audio_i2s_mck_update_frequency(audio_state.freq, i2s_mck_multiplier);
                    }
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
            }
        }

        // Handle deferred input source switch
        if (input_source_change_pending) {
            input_source_change_pending = false;
            __dmb();

            uint8_t new_source = pending_input_source;
            uint8_t old_source = active_input_source;

            if (new_source != old_source && input_source_valid(new_source)) {
                usb_audio_drain_ring();
                prepare_pipeline_reset(PRESET_MUTE_SAMPLES);

                // Stop old source hardware
                if (old_source == INPUT_SOURCE_SPDIF) {
                    spdif_input_stop();

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

                active_input_source = new_source;

                // Start new source hardware
                if (new_source == INPUT_SOURCE_SPDIF) {
                    spdif_input_start();
                    // Don't complete_pipeline_reset yet — output stays muted
                    // until SPDIF lock is acquired (handled in polling block below)
                } else {
                    // Switching to USB: flush stale ring data, complete reset
                    usb_audio_flush_ring();
                    complete_pipeline_reset();
                }

                printf("Input source: %u -> %u\n",
                       (unsigned)old_source, (unsigned)new_source);
            }
        }

        // Handle deferred SPDIF RX pin directory update
        if (flash_set_spdif_rx_pin_pending) {
            flash_set_spdif_rx_pin_pending = false;
            prepare_flash_write_operation();
            preset_set_spdif_rx_pin(spdif_rx_pin);
            complete_flash_write_operation_light();
        }

        // LED heartbeat - toggle every ~1000 iterations
        static uint32_t loop_counter = 0;
        if (++loop_counter >= 1000) {
            loop_counter = 0;
            gpio_xor_mask(1u << 25);
        }
    }
}
