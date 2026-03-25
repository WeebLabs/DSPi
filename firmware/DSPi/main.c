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
#include "dsp_pipeline.h"
#include "flash_storage.h"
#include "pico/audio_i2s_multi.h"
#include "pdm_generator.h"
#include "usb_audio.h"
#include "loudness.h"
#include "crossfeed.h"
#include "bulk_params.h"
#include "pico/audio_spdif.h"

// ----------------------------------------------------------------------------
// GLOBAL DEFINITIONS
// ----------------------------------------------------------------------------

// USB audio feedback (SOF-measured, 10.14 fixed-point)
volatile uint32_t feedback_10_14 = 0;
volatile uint32_t nominal_feedback_10_14 = 0;

// SOF handler state for feedback measurement
static uint32_t sof_count = 0;
static uint32_t fb_last_total = 0;
static uint32_t fb_accum = 0;
volatile uint32_t feedback_reset_value = 0;

// Fill-level servo (Loop B): proportional correction based on consumer buffer fill
extern volatile uint8_t spdif0_consumer_fill;
#define FILL_TARGET      8       // 50% of 16 consumer buffers
#define FILL_SERVO_KP    1024    // 10.14 units per buffer of error, keeps fill within ±1 of target
#define FILL_SERVO_CLAMP 8192    // ±0.5 samples in 10.14 — prevents servo dominating rate
#define FB_OUTER_CLAMP   16384   // ±1 sample in 10.14 from nominal

// USB SOF IRQ — measures device clock vs host clock for async feedback
void __not_in_flash_func(usb_sof_irq)(void) {
    extern audio_spdif_instance_t *spdif_instance_ptrs[];
    extern uint8_t output_types[];

    // Read DMA word count from instance 0 (SPDIF or I2S)
    volatile uint32_t *p_words_consumed;
    uint32_t xfer_words;
    uint8_t dma_ch;
    uint8_t slot0_type = output_types[0];

    if (slot0_type == OUTPUT_TYPE_I2S) {
        extern audio_i2s_instance_t *i2s_instance_ptrs[];
        audio_i2s_instance_t *inst = i2s_instance_ptrs[0];
        p_words_consumed = &inst->words_consumed;
        xfer_words = inst->current_transfer_words;
        dma_ch = inst->dma_channel;
    } else {
        audio_spdif_instance_t *inst = spdif_instance_ptrs[0];
        p_words_consumed = &inst->words_consumed;
        xfer_words = inst->current_transfer_words;
        dma_ch = inst->dma_channel;
    }

    // Read total DMA words consumed by instance 0 (sub-buffer precision)
    uint32_t remaining = dma_channel_hw_addr(dma_ch)->transfer_count;
    uint32_t current_total = *p_words_consumed + (xfer_words - remaining);

    // Handle reset request from rate change
    if (feedback_reset_value) {
        fb_accum = feedback_reset_value;
        feedback_reset_value = 0;
        fb_last_total = current_total;
        sof_count = 0;
    }

    sof_count++;
    if ((sof_count & 0x3) == 0) {  // Every 4 SOFs (bRefresh=2 → 2^2=4)
        uint32_t delta_words = current_total - fb_last_total;
        fb_last_total = current_total;

        if (delta_words > 0) {
            // 10.14 format: delta_words / 4_SOFs / words_per_sample * 2^14
            // SPDIF: 4 words/sample → shift = 10 (2^14 / 16 = 1024)
            // I2S:   2 words/sample → shift = 11 (2^14 / 8  = 2048)
            uint32_t shift = (slot0_type == OUTPUT_TYPE_I2S) ? 11 : 10;
            uint32_t raw = delta_words << shift;

            if (fb_accum == 0) {
                fb_accum = raw;
            } else {
                int32_t error = (int32_t)raw - (int32_t)fb_accum;
                fb_accum += error >> 3;  // IIR α≈0.125, τ≈32ms
            }

            // Loop B: proportional fill-level servo
            // Underfull (fill < target) → negative error → positive servo → host sends more
            int32_t fill_error = (int32_t)spdif0_consumer_fill - FILL_TARGET;
            int32_t fb_servo = -(fill_error * FILL_SERVO_KP);
            if (fb_servo > FILL_SERVO_CLAMP)  fb_servo = FILL_SERVO_CLAMP;
            if (fb_servo < -FILL_SERVO_CLAMP) fb_servo = -FILL_SERVO_CLAMP;

            // Sum rate + fill, clamp to nominal ± 1 sample
            int32_t fb_out = (int32_t)fb_accum + fb_servo;
            int32_t nom = (int32_t)nominal_feedback_10_14;
            if (fb_out > nom + FB_OUTER_CLAMP) fb_out = nom + FB_OUTER_CLAMP;
            if (fb_out < nom - FB_OUTER_CLAMP) fb_out = nom - FB_OUTER_CLAMP;

            feedback_10_14 = (uint32_t)fb_out;
        }
    }
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

static void perform_rate_change(uint32_t new_freq) {
    switch (new_freq) { case 44100: case 48000: case 96000: break; default: new_freq = 44100; }

    // Update the audio format so pico_audio_spdif can update the PIO divider
    audio_format_48k.sample_freq = new_freq;

#if PICO_RP2350
    // RP2350: Dynamic clock switching for optimal performance and integer PIO dividers
    // 48kHz family -> 307.2MHz (PIO divider 6400 = 25.0 exact integer)
    // 44.1kHz family -> 264.6MHz (44100 * 6000)
    uint32_t target_freq = (new_freq == 44100) ? 264600000 : 307200000;

    // Only change if needed to avoid glitches
    if (clock_get_hz(clk_sys) != target_freq) {
        vreg_set_voltage(VREG_VOLTAGE_1_20);
        busy_wait_us(100);
        set_sys_clock_hz(target_freq, false);
    }
#else
    // RP2040: Change system clock for optimal S/PDIF timing
    // 307.2MHz gives integer PIO divider (25.0) for 48kHz SPDIF
    if((new_freq == 48000 || new_freq == 96000) && clock_176mhz) {
        // 307.2MHz -> VCO 1536 MHz / 5 / 1
        set_sys_clock_pll(1536000000, 5, 1);
        clock_176mhz = 0;
    }
    else if(new_freq == 44100 && !clock_176mhz) {
        // 264.6MHz (44100 * 6000) -> VCO 1058.4 MHz
        set_sys_clock_pll(1058400000, 4, 1);
        clock_176mhz = 1;
    }
#endif
    // Reset sync
    extern volatile bool sync_started;
    extern volatile uint64_t total_samples_produced;
    sync_started = false;
    total_samples_produced = 0;

    // Pre-compute nominal feedback and reset SOF measurement
    nominal_feedback_10_14 = ((uint64_t)new_freq << 14) / 1000;
    feedback_10_14 = nominal_feedback_10_14;
    feedback_reset_value = nominal_feedback_10_14;

    dsp_recalculate_all_filters((float)new_freq);
    loudness_recompute_pending = true;
    crossfeed_update_pending = true;  // Recalculate crossfeed coefficients for new sample rate
    pdm_update_clock(new_freq);

    // Update MCK frequency for new sample rate (if enabled)
    extern bool i2s_mck_enabled;
    extern uint8_t i2s_mck_multiplier;
    if (i2s_mck_enabled) {
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

// Reset USB async feedback loop state after disruptive output pipeline events
// (type switch + global resync), so SOF estimation restarts from nominal.
static void reset_usb_feedback_loop(void) {
    feedback_reset_value = nominal_feedback_10_14;
    feedback_10_14 = nominal_feedback_10_14;
}

// Re-lock consumer fill across all active outputs after any type switch.
static void resync_active_output_pipelines(void) {
    extern uint8_t output_types[];
    extern audio_spdif_instance_t *spdif_instance_ptrs[];
    extern audio_i2s_instance_t *i2s_instance_ptrs[];

    audio_spdif_instance_t *spdif_sync[NUM_SPDIF_INSTANCES];
    audio_i2s_instance_t *i2s_sync[NUM_SPDIF_INSTANCES];
    uint spdif_count = 0;
    uint i2s_count = 0;

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

    // RP2350: 307.2MHz gives integer PIO divider (25.0) for 48kHz SPDIF
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    busy_wait_ms(10);

    // VCO 1536MHz / 5 = 307.2MHz
    if (!set_sys_clock_hz(307200000, false)) {
        set_sys_clock_hz(150000000, false);
    }
#else
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    busy_wait_ms(10);
    // Initial 307.2MHz -> VCO 1536 MHz / 5 / 1
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

    // Initialize nominal feedback for default sample rate
    nominal_feedback_10_14 = ((uint64_t)audio_state.freq << 14) / 1000;
    feedback_10_14 = nominal_feedback_10_14;

    // Load preset from flash.  Always selects a preset (factory defaults if
    // the target slot is empty).  Migrates legacy data on first boot.
    preset_boot_load();
    {
        uint32_t flags = save_and_disable_interrupts();
        dsp_recalculate_all_filters(48000.0f);
        dsp_update_delay_samples(48000.0f);
        restore_interrupts(flags);

        // Apply SPDIF pin configuration (before Core 1 starts)
        extern uint8_t output_pins[];
        extern audio_spdif_instance_t *spdif_instance_ptrs[];
        for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
            if (output_pins[i] != spdif_instance_ptrs[i]->pin) {
                audio_spdif_set_enabled(spdif_instance_ptrs[i], false);
                audio_spdif_change_pin(spdif_instance_ptrs[i], output_pins[i]);
                audio_spdif_set_enabled(spdif_instance_ptrs[i], true);
            }
        }
    }

    // Initial loudness table computation (uses loaded or default params)
    loudness_recompute_table(loudness_ref_spl, loudness_intensity_pct, 48000.0f);
    if (loudness_enabled && loudness_active_table) {
        audio_set_volume(audio_state.volume);  // Re-select loudness coefficients
    }

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

        // Handle USB stream restart (alt 0 -> alt > 0): re-lock all active output
        // pipelines so consumer fill/phase starts aligned after host re-prime.
        {
            extern volatile bool stream_restart_resync_pending;
            if (stream_restart_resync_pending) {
                stream_restart_resync_pending = false;
                __dmb();

                // Wait for Core 1 to finish current work before touching pipelines.
                if (core1_mode == CORE1_MODE_EQ_WORKER) {
                    while (core1_eq_work.work_ready && !core1_eq_work.work_done) {
                        tight_loop_contents();
                    }
                    __dmb();
                }

                preset_mute_counter = 256;
                preset_loading = true;
                __dmb();

                resync_active_output_pipelines();
                reset_usb_feedback_loop();
                printf("USB stream restart: outputs resynced\n");
            }
        }

        // Handle output type change (deferred from USB ISR — needs heap allocation)
        {
            extern volatile bool output_type_change_pending;
            extern volatile uint8_t pending_output_slot;
            extern volatile uint8_t pending_output_type;

            if (output_type_change_pending) {
                output_type_change_pending = false;
                __dmb();

                uint8_t slot = pending_output_slot;
                uint8_t new_type = pending_output_type;

                extern uint8_t output_types[];
                extern audio_spdif_instance_t *spdif_instance_ptrs[];
                extern audio_i2s_instance_t *i2s_instance_ptrs[];
                extern uint8_t output_pins[];
                extern uint8_t i2s_bck_pin;

                // Mute audio during switch (~5ms)
                preset_mute_counter = 256;
                preset_loading = true;
                __dmb();

                // Wait for Core 1 to finish current work
                if (core1_mode == CORE1_MODE_EQ_WORKER) {
                    while (core1_eq_work.work_ready && !core1_eq_work.work_done)
                        tight_loop_contents();
                    __dmb();
                }

                if (new_type == OUTPUT_TYPE_I2S && output_types[slot] == OUTPUT_TYPE_SPDIF) {
                    // --- S/PDIF → I2S ---
                    audio_spdif_instance_t *spdif_inst = spdif_instance_ptrs[slot];

                    // Disable SPDIF and silence its DMA IRQ
                    audio_spdif_set_enabled(spdif_inst, false);
                    dma_irqn_set_channel_enabled(spdif_inst->dma_irq, spdif_inst->dma_channel, false);
                    dma_channel_abort(spdif_inst->dma_channel);
                    if (spdif_inst->playing_buffer) {
                        give_audio_buffer(spdif_inst->consumer_pool, spdif_inst->playing_buffer);
                        spdif_inst->playing_buffer = NULL;
                    }
                    spdif_reset_consumer_pipeline(spdif_inst);

                    // Unclaim SM so I2S can claim it (DMA channel stays claimed/dormant)
                    pio_sm_unclaim(spdif_inst->pio, spdif_inst->pio_sm);

                    // Set up I2S with same SM but a DIFFERENT DMA channel
                    // (SPDIF uses 0-3, I2S uses 8-11 to avoid handler conflict)
                    audio_i2s_config_t i2s_cfg = {
                        .data_pin = output_pins[slot],
                        .clock_pin_base = i2s_bck_pin,
                        .dma_channel = slot + 8,
                        .pio_sm = slot,
                        .pio = PICO_AUDIO_SPDIF_PIO,
                        .dma_irq = PICO_AUDIO_I2S_DMA_IRQ,
                    };

                    extern struct audio_buffer_pool *producer_pools[];
                    audio_i2s_setup(i2s_instance_ptrs[slot], &audio_format_48k, &i2s_cfg);
                    audio_i2s_connect_extra(i2s_instance_ptrs[slot], producer_pools[slot],
                                            false, SPDIF_CONSUMER_BUFFER_COUNT, NULL);

                    output_types[slot] = OUTPUT_TYPE_I2S;
                    resync_active_output_pipelines();
                    reset_usb_feedback_loop();
                    printf("Slot %d switched to I2S (DMA ch %d, outputs resynced)\n", slot, slot + 8);

                } else if (new_type == OUTPUT_TYPE_SPDIF && output_types[slot] == OUTPUT_TYPE_I2S) {
                    // --- I2S → S/PDIF ---
                    audio_spdif_instance_t *spdif_inst = spdif_instance_ptrs[slot];

                    // Tear down I2S (releases its DMA channel + SM)
                    audio_i2s_teardown(i2s_instance_ptrs[slot]);
                    memset(i2s_instance_ptrs[slot], 0, sizeof(audio_i2s_instance_t));

                    // Reclaim SM for SPDIF
                    pio_sm_claim(spdif_inst->pio, spdif_inst->pio_sm);

                    // Reinit SPDIF SM (handles PIO program, clock divider, DMA IRQ re-enable)
                    audio_spdif_change_pin(spdif_inst, output_pins[slot]);

                    // Reconnect producer pool to SPDIF consumer pool
                    // (audio_i2s_connect_extra replaced the connection; restore it)
                    extern struct audio_buffer_pool *producer_pools[];
                    audio_complete_connection(&spdif_inst->connection.core,
                                              producer_pools[slot],
                                              spdif_inst->consumer_pool);

                    output_types[slot] = OUTPUT_TYPE_SPDIF;
                    resync_active_output_pipelines();
                    reset_usb_feedback_loop();
                    printf("Slot %d switched to S/PDIF (outputs resynced)\n", slot);
                }
            }
        }

        // Handle bulk parameter SET (deferred from USB IRQ)
        if (bulk_params_pending) {
            bulk_params_pending = false;

            // Wait for Core 1 to finish current work
            if (core1_mode == CORE1_MODE_EQ_WORKER) {
                while (core1_eq_work.work_ready && !core1_eq_work.work_done) {
                    tight_loop_contents();
                }
                __dmb();
            }

            // Mute audio during parameter swap
            preset_mute_counter = PRESET_MUTE_SAMPLES;
            preset_loading = true;
            __dmb();

            // Apply the received parameters (pin config gated by include_pins setting)
            uint16_t _occ; uint8_t _m, _d, _la, inc_pins;
            preset_get_directory(&_occ, &_m, &_d, &_la, &inc_pins);
            int err = bulk_params_apply((const WireBulkParams *)bulk_param_buf, inc_pins != 0);
            if (err == 0) {
                float rate = (float)audio_state.freq;
                dsp_recalculate_all_filters(rate);
                dsp_update_delay_samples(rate);

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

        // LED heartbeat - toggle every ~1000 iterations
        static uint32_t loop_counter = 0;
        if (++loop_counter >= 1000) {
            loop_counter = 0;
            gpio_xor_mask(1u << 25);
        }
    }
}
