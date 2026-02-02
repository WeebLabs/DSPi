/*
 * DSPi Main - TinyUSB Audio Device
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
#include "hardware/timer.h"
#include "tusb.h"

// Local headers
#include "config.h"
#include "dsp_pipeline.h"
#include "flash_storage.h"
#include "pdm_generator.h"
#include "usb_audio.h"
#include "usb_descriptors.h"

// ----------------------------------------------------------------------------
// GLOBAL DEFINITIONS
// ----------------------------------------------------------------------------
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

static void perform_rate_change(uint32_t new_freq) {
    switch (new_freq) { case 44100: case 48000: case 96000: break; default: new_freq = 44100; }

    // Update the audio format so pico_audio_spdif can update the PIO divider
    audio_format_48k.sample_freq = new_freq;

#if PICO_RP2350
    // RP2350: Dynamic clock switching for optimal performance and integer PIO dividers
    // 48kHz family -> 288MHz (48000 * 6000)
    // 44.1kHz family -> 264.6MHz (44100 * 6000)
    uint32_t target_freq = (new_freq == 44100) ? 264600000 : 288000000;
    
    // Only change if needed to avoid glitches
    if (clock_get_hz(clk_sys) != target_freq) {
        // User requested 1.1V even for high clock
        vreg_set_voltage(VREG_VOLTAGE_1_10); 
        busy_wait_us(100);
        set_sys_clock_hz(target_freq, false);
    }
#else
    // RP2040: Change system clock for optimal S/PDIF timing
    if((new_freq == 48000 || new_freq == 96000) && clock_176mhz) {
        set_sys_clock_pll(1440000000, 6, 1);
        clock_176mhz = 0;
    }
    else if(new_freq == 44100 && !clock_176mhz) {
        set_sys_clock_pll(1236000000, 7, 1);
        clock_176mhz = 1;
    }
#endif
    // Reset sync
    extern volatile bool sync_started;
    extern volatile uint64_t total_samples_produced;
    sync_started = false;
    total_samples_produced = 0;

    dsp_recalculate_all_filters((float)new_freq);
    pdm_update_clock(new_freq);
}

void core0_init() {
    // LED setup
    gpio_init(25); gpio_set_dir(25, GPIO_OUT);

#if PICO_RP2350
    // RP2350: Use set_sys_clock_hz for proper clock tracking
    vreg_set_voltage(VREG_VOLTAGE_1_10);
    busy_wait_ms(10);

    // Target 288MHz for 48kHz audio start
    if (!set_sys_clock_hz(288000000, false)) {
        // Fall back to 150MHz if 288MHz not achievable
        set_sys_clock_hz(150000000, false);
    }
#else
    vreg_set_voltage(VREG_VOLTAGE_1_15);
    busy_wait_ms(10);
    set_sys_clock_pll(1440000000, 6, 1);
#endif

    gpio_init(23); gpio_set_dir(23, GPIO_OUT); gpio_put(23, 1);

    pico_get_unique_board_id_string(descriptor_str_serial, 17);

    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

    // [CRITICAL FIX]
    // Initialize USB/SPDIF *BEFORE* PDM.
    // SPDIF requires DMA Channel 0 (hardcoded in config).
    // If PDM inits first, it steals Ch 0 via dma_claim_unused_channel(), causing SPDIF to panic/crash.
    usb_sound_card_init();

    // Try to load saved parameters from flash
    // If successful, this overwrites the defaults set by usb_sound_card_init()
    if (flash_load_params() == FLASH_OK) {
        dsp_recalculate_all_filters(48000.0f);
        dsp_update_delay_samples(48000.0f);
    }

#if ENABLE_SUB
    pdm_setup_hw();

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

    // Brief LED indication that init is complete
    gpio_put(25, 1);
    busy_wait_ms(500);

    // Enable watchdog
    watchdog_enable(8000, 1);

    while (1) {
        // TinyUSB task - must be called regularly
        tud_task();

        // Update watchdog
        watchdog_update();

        // Update audio feedback for drift correction
        update_audio_feedback();

        // Handle EQ parameter updates from USB
        if (eq_update_pending) {
            EqParamPacket p = pending_packet;
            eq_update_pending = false;
            filter_recipes[p.channel][p.band] = p;

            uint32_t flags = save_and_disable_interrupts();
            dsp_compute_coefficients(&p, &filters[p.channel][p.band], (float)audio_state.freq);
            restore_interrupts(flags);
        }

        // Handle sample rate changes
        if (rate_change_pending) {
            uint32_t r = pending_rate;
            rate_change_pending = false;
            perform_rate_change(r);
        }

        // LED heartbeat - toggle every ~1000 iterations
        static uint32_t loop_counter = 0;
        if (++loop_counter >= 1000) {
            loop_counter = 0;
            gpio_xor_mask(1u << 25);
        }
    }
}
