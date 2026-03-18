/*
 * DSPi Main - USB Audio Device
 * USB Audio with DSP processing and S/PDIF output
 */

#include <stdio.h>
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
#define FILL_TARGET      2       // 50% of consumer capacity (4 buffers)
#define FILL_SERVO_KP    1024    // 10.14 units per buffer of error, tau ~6s at max
#define FILL_SERVO_CLAMP 8192    // ±0.5 samples in 10.14 — prevents servo dominating rate
#define FB_OUTER_CLAMP   16384   // ±1 sample in 10.14 from nominal

// USB SOF IRQ — measures device clock vs host clock for async feedback
void __not_in_flash_func(usb_sof_irq)(void) {
    extern audio_spdif_instance_t *spdif_instance_ptrs[];
    audio_spdif_instance_t *inst = spdif_instance_ptrs[0];

    // Read total DMA words consumed by instance 0 (sub-buffer precision)
    uint32_t remaining = dma_channel_hw_addr(inst->dma_channel)->transfer_count;
    uint32_t current_total = inst->words_consumed
                           + (inst->current_transfer_words - remaining);

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
            // 10.14 format: delta_words / 4_SOFs / 4_words_per_sample * 2^14
            //             = delta_words * (2^14 / 16) = delta_words << 10
            uint32_t raw = delta_words << 10;

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
        vreg_set_voltage(VREG_VOLTAGE_1_15);
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
    vreg_set_voltage(VREG_VOLTAGE_1_15);
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
