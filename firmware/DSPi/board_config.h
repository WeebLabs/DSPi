#ifndef DSPI_BOARD_CONFIG_H
#define DSPI_BOARD_CONFIG_H

#include "pico.h"

#include <stdbool.h>
#include <stdint.h>

#if PICO_RP2350 && !PICO_RP2350A
#define DSPI_CONTROL_ADC_FIRST_PIN 40
#else
#define DSPI_CONTROL_ADC_FIRST_PIN 26
#endif
#define DSPI_CONTROL_ADC_LAST_PIN (DSPI_CONTROL_ADC_FIRST_PIN + 2)

static inline bool dspi_board_gpio_is_reserved(uint8_t pin) {
#if defined(OLIMEX_RP2350_XXL)
    // GPIO 8 is the PSRAM chip-select on the shared QSPI bus.
    return pin == 8;
#else
    (void)pin;
    return false;
#endif
}

static inline bool dspi_gpio_is_valid(uint8_t pin) {
    if (dspi_board_gpio_is_reserved(pin)) return false;
    if (pin >= 23 && pin <= 25) return false;

#if PICO_RP2350
    return pin <= 29;
#else
    return pin <= 28;
#endif
}

static inline bool dspi_control_adc_pin_is_valid(uint8_t pin) {
    return pin >= DSPI_CONTROL_ADC_FIRST_PIN &&
           pin <= DSPI_CONTROL_ADC_LAST_PIN &&
           !dspi_board_gpio_is_reserved(pin);
}

#endif
