/*
 * audio_input.h — Input source abstraction for DSPi
 *
 * Defines the input source enum and switching infrastructure.
 * Currently supports USB and S/PDIF inputs; designed for future
 * extensibility to I2S and ADAT without restructuring.
 */

#ifndef AUDIO_INPUT_H
#define AUDIO_INPUT_H

#include <stdint.h>
#include <stdbool.h>

// Input source identifiers (extensible — leave gaps for future types)
typedef enum {
    INPUT_SOURCE_USB   = 0,
    INPUT_SOURCE_SPDIF = 1,
    // Future: INPUT_SOURCE_I2S = 2, INPUT_SOURCE_ADAT = 3
} InputSource;

#define INPUT_SOURCE_MAX    INPUT_SOURCE_SPDIF   // Highest valid value

// Default SPDIF RX GPIO pin
#define PICO_SPDIF_RX_PIN_DEFAULT  11

// SPDIF RX lock debounce — firmware constant, not configurable via vendor command.
// After the library reports lock, wait this many ms before unmuting output.
#define SPDIF_RX_LOCK_DEBOUNCE_MS  100

// Current active input source (definition in audio_input.c)
extern volatile uint8_t active_input_source;

// SPDIF RX pin (device-level setting, stored in PresetDirectory)
extern uint8_t spdif_rx_pin;

// Deferred input source switch (set by vendor command, handled in main loop)
extern volatile bool input_source_change_pending;
extern volatile uint8_t pending_input_source;

// Validate an input source value
static inline bool input_source_valid(uint8_t src) {
    return src <= INPUT_SOURCE_MAX;
}

#endif // AUDIO_INPUT_H
