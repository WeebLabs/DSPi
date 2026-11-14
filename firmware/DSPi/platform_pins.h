#ifndef PLATFORM_PINS_H
#define PLATFORM_PINS_H

// Pin hooks for a platform other than the RP2040 and RP2350. Only a build that defines
// DSPI_PLATFORM_PINS includes this file; it then supplies dspi_platform_pins.h, which defines every
// default pin macro, and implements the functions below. The RP2040 and RP2350 builds never see it.

#include <stdbool.h>
#include <stdint.h>

// What a pin is for, so a platform can apply that function's rule.
enum {
    DSPI_PIN_FN_ANY,            // no function-specific rule: is_valid_gpio_pin()
    DSPI_PIN_FN_OUTPUT,         // an S/PDIF or I2S output slot's data pin
    DSPI_PIN_FN_PDM,
    DSPI_PIN_FN_I2S_CLK,        // BCK or LRCLK of the master (or unified) clock pair
    DSPI_PIN_FN_I2S_CLK_SLAVE,  // BCK or LRCLK of the SPLIT-mode slave clock pair
    DSPI_PIN_FN_MCK,
    DSPI_PIN_FN_ADAT_OUT,
    DSPI_PIN_FN_ADAT_IN,
    DSPI_PIN_FN_SPDIF_RX,
    DSPI_PIN_FN_I2S_RX,
    DSPI_PIN_FN_DAC_MUTE,
    DSPI_PIN_FN_UART_CTRL,      // pair: TX, RX
    DSPI_PIN_FN_I2C_CTRL,       // pair: SDA, SCL
    DSPI_PIN_FN_CS_DISPLAY,     // pair: SDA, SCL
};

bool dspi_platform_pin_valid(uint8_t fn, uint8_t pin);
// In place of the RP2040/RP2350 mux rules for a UART, I2C or display pin pair.
bool dspi_platform_pin_pair_valid(uint8_t fn, uint8_t a, uint8_t b);
// A control-surface binding's pin, by binding type (CS_TYPE_*), after DSPi's own checks.
bool dspi_platform_cs_pin_valid(uint8_t cs_type, uint8_t pin);

#include "dspi_platform_pins.h"

#endif
