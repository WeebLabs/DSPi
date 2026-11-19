/*
 * audio_input.c — Input source state for DSPi
 *
 * Global definitions for the input source abstraction layer.
 * Phase 2 adds SPDIF RX lifecycle functions here.
 */

#include "audio_input.h"
#include "input_capture_arena.h"
#include "pico/stdlib.h"

// Shared capture storage for the mutually exclusive input receivers.  The
// explicit section keeps it out of COMMON, where its alignment would cost
// more padding than the overlay saves.
InputCaptureArena __attribute__((section(".bss.input_capture_arena")))
    input_capture_arena;
static InputArenaOwner arena_owner = INPUT_ARENA_FREE;

void input_arena_claim(InputArenaOwner owner) {
    // A second live owner would write another receiver's ring; fail loudly at
    // the start that broke the exclusivity invariant, not later in the audio.
    if (arena_owner != INPUT_ARENA_FREE && arena_owner != owner) {
        panic("input capture arena busy: owner %d, claim %d",
              (int)arena_owner, (int)owner);
    }
    arena_owner = owner;
}

void input_arena_release(InputArenaOwner owner) {
    if (arena_owner == owner) arena_owner = INPUT_ARENA_FREE;
}

InputArenaOwner input_arena_get_owner(void) {
    return arena_owner;
}

// Active input source — default to USB
volatile uint8_t active_input_source = INPUT_SOURCE_USB;

// SPDIF RX GPIO pin — device-level setting (not per-preset)
uint8_t spdif_rx_pin = PICO_SPDIF_RX_PIN_DEFAULT;

// Optional SPDIF inputs 2..4: GPIOs.  An input outside the block count reserves no
// GPIO and is absent from the source list.
uint8_t spdif_rx_pin_ext[SPDIF_RX_NUM_INPUTS - 1] = {
    PICO_SPDIF_RX_PIN2_DEFAULT,
    PICO_SPDIF_RX_PIN3_DEFAULT,
    PICO_SPDIF_RX_PIN4_DEFAULT,
};

// Input blocks: S/PDIF on with one input, I2S off (input_enable_plan.md section 14).
uint8_t spdif_rx_enabled = 1;
uint8_t spdif_rx_count = 1;
uint8_t i2s_input_enabled = 0;

// I2S RX serial-data pins, one per stereo pair.  [0] is the always-present
// stereo pair; [1..3] (RP2350) are placeholders the host assigns when enabling
// >2-channel I2S input.  Defaults are the contiguous block GPIO 1/2/3/4 (pairs
// 0/1/2/3): all free of every default assignment, so enabling 4/6/8-channel
// input out of the box never self-collides.  They are still expected to be
// overridden per board, since multichannel input requires explicit wiring of
// each ADC data line.
uint8_t i2s_rx_pin[I2S_RX_MAX_PAIRS] = {
    PICO_I2S_RX_PIN_DEFAULT,            // pair 0 = GPIO 1
#if I2S_RX_MAX_PAIRS > 1
#ifdef DSPI_PLATFORM_PINS
    PICO_I2S_RX_PIN2_DEFAULT, PICO_I2S_RX_PIN3_DEFAULT, PICO_I2S_RX_PIN4_DEFAULT,
#else
    2, 3, 4,                            // pairs 1/2/3 = GPIO 2/3/4
#endif
#endif
};

// Active I2S input channel count (2/4/6/8 on RP2350, 2 on RP2040)
uint8_t i2s_input_channels = 2;

// Selected sample rate while I2S input is active (device is the rate
// authority in I2S input mode)
uint32_t i2s_input_rate = 48000;

// I2S clock mode (master by default; slave only meaningful with I2S input)
uint8_t i2s_clock_mode = I2S_CLOCK_MODE_MASTER;

// Deferred input source switch
volatile bool input_source_change_pending = false;
volatile uint8_t pending_input_source = INPUT_SOURCE_USB;

// Deferred I2S clock-mode apply (handled in main loop)
volatile bool i2s_clock_mode_change_pending = false;
volatile uint8_t pending_i2s_clock_mode = I2S_CLOCK_MODE_MASTER;

// Deferred I2S RX hot-swaps (handled in main loop)
volatile bool i2s_rx_pin_change_pending = false;
volatile bool i2s_input_restart_pending = false;

// ADAT input (RP2350 only; the state exists on both platforms so vendor
// GET/persistence round-trips work everywhere). Disabled by default; no free
// default GPIO remains, so the pin ships unset and must be assigned by the
// host before the input can be enabled. Same reserve-nothing-while-disabled
// model as the optional SPDIF inputs.
uint8_t adat_input_enabled = 0;
uint8_t adat_input_pin = 0xFF;
uint8_t adat_clock_mode = ADAT_CLOCK_MODE_MASTER;

// Deferred ADAT input applies (handled in main loop)
volatile bool adat_clock_mode_change_pending = false;
volatile uint8_t pending_adat_clock_mode = ADAT_CLOCK_MODE_MASTER;
volatile bool adat_input_restart_pending = false;
