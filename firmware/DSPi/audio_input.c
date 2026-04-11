/*
 * audio_input.c — Input source state for DSPi
 *
 * Global definitions for the input source abstraction layer.
 * Phase 2 adds SPDIF RX lifecycle functions here.
 */

#include "audio_input.h"

// Active input source — default to USB
volatile uint8_t active_input_source = INPUT_SOURCE_USB;

// SPDIF RX GPIO pin — device-level setting (not per-preset)
uint8_t spdif_rx_pin = PICO_SPDIF_RX_PIN_DEFAULT;

// Deferred input source switch
volatile bool input_source_change_pending = false;
volatile uint8_t pending_input_source = INPUT_SOURCE_USB;
