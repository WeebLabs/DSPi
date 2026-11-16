/*
 * vc_pins.c: GPIO validation and conflict checks shared by the vendor
 * commands and the bulk/preset restore paths.
 */

#include "vc_pins.h"
#include "usb_audio.h"
#include "audio_input.h"
#include "spdif_input.h"
#include "i2s_input.h"
#include "dac_hw_mute.h"
#include "adat_output.h"
#include "adat_input.h"
#include "uart_control.h"
#include "i2c_control.h"
#include "control_surfaces.h"
#include "config.h"

// Platform defaults for each pin output, in output_pins[] order (same values
// as the initializer in usb_audio.c). OUTPUT pin SETs map PIN_RESET_TO_DEFAULT through it.
const uint8_t default_output_pins[NUM_PIN_OUTPUTS] = {
#if PICO_RP2350
    PICO_AUDIO_SPDIF_PIN, PICO_SPDIF_PIN_2,
    PICO_SPDIF_PIN_3, PICO_SPDIF_PIN_4, PICO_PDM_PIN
#else
    PICO_AUDIO_SPDIF_PIN, PICO_SPDIF_PIN_2, PICO_PDM_PIN
#endif
};

bool is_valid_gpio_pin(uint8_t pin) {
#ifdef DSPI_PLATFORM_PINS
    return dspi_platform_pin_valid(DSPI_PIN_FN_ANY, pin);
#else
    // GPIO 16/17 are general-purpose again since the debug UART was removed;
    // when the UART control interface claims them, is_pin_in_use() covers it.
    if (pin >= 23 && pin <= 25) return false;   // Power/LED
#if PICO_RP2350
    return pin <= 29;
#else
    return pin <= 28;
#endif
#endif
}

bool pin_used_by_fixed_peripheral_except_mck(uint8_t pin, uint8_t exclude_output) {
    for (int i = 0; i < NUM_PIN_OUTPUTS; i++) {
        if (i == exclude_output) continue;
        if (output_pins[i] == pin) return true;
    }
#if PICO_RP2350
    // ADAT data pin: claimed only while ADAT is config-enabled (mirrors MCK).
    if (adat_output_config_enabled() && pin == adat_output_pin()) return true;
    // ADAT input pin: claimed only while the input is enabled. Direction-agnostic;
    // the TX-pin-sharing exception lives at the ADAT-input validation sites.
    if (adat_input_enabled && pin == adat_input_pin) return true;
#endif
    if (pin == spdif_rx_pin) return true;                     // SPDIF RX input 1 (always claimed)
    // Optional SPDIF inputs: a pin is claimed only while that input is
    // enabled; a disabled input's stored pin is invisible to conflict checks.
    for (uint8_t i = 1; i < SPDIF_RX_NUM_INPUTS; i++)
        if (spdif_input_enabled(i) && pin == spdif_rx_pin_for_index(i)) return true;
    if (dac_hw_mute_owns_pin(pin)) return true;               // DAC hardware-mute
    if (uart_ctrl_owns_pin(pin)) return true;                 // UART control (if live)
    if (i2c_ctrl_owns_pin(pin)) return true;                  // I2C control (if live)
    if (control_surfaces_owns_pin(pin)) return true;         // Control Surfaces (live bindings)
    return false;
}

bool pin_used_by_fixed_peripheral(uint8_t pin, uint8_t exclude_output) {
    if (i2s_mck_enabled && pin == i2s_mck_pin) return true;   // MCK (if enabled)
    return pin_used_by_fixed_peripheral_except_mck(pin, exclude_output);
}

bool is_pin_in_use(uint8_t pin, uint8_t exclude) {
    if (pin_used_by_fixed_peripheral(pin, exclude)) return true;
    // I2S BCK/LRCLK are reserved only while the clocks run (any I2S output, or
    // I2S input active). In SPLIT clock-pin mode both pairs count, since the
    // dormant pair is one deferred mode flip away from being driven.
    bool i2s_clocks_in_use = (active_input_source == INPUT_SOURCE_I2S);
    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        if (output_types[i] == OUTPUT_TYPE_I2S) {
            i2s_clocks_in_use = true;
            break;  // All I2S slots share the same BCK/LRCLK
        }
    }
    if (i2s_clocks_in_use && i2s_clock_pin_claimed(pin)) return true;
    // I2S RX data pins for every active (by count) stereo pair.
    for (int p = 0; p < i2s_input_channels / 2 && p < I2S_RX_MAX_PAIRS; p++)
        if (pin == i2s_rx_pin[p]) return true;
    return false;
}

bool i2s_rx_pair_pin_taken(uint8_t pin, uint8_t exclude_pair, uint8_t n_pairs) {
    for (int q = 0; q < n_pairs && q < I2S_RX_MAX_PAIRS; q++)
        if (q != exclude_pair && i2s_rx_pin[q] == pin)
            return true;
    return false;
}

uint8_t check_i2s_rx_pin(uint8_t pin, uint8_t pair, uint8_t n_pairs) {
    if (!PIN_VALID_FOR(DSPI_PIN_FN_I2S_RX, pin))              return PIN_CONFIG_INVALID_PIN;
    if (i2s_clock_pin_claimed(pin))                           return PIN_CONFIG_PIN_IN_USE;
    if (pin_used_by_fixed_peripheral(pin, 0xFF))             return PIN_CONFIG_PIN_IN_USE;
    if (i2s_rx_pair_pin_taken(pin, pair, n_pairs))           return PIN_CONFIG_PIN_IN_USE;
    return PIN_CONFIG_SUCCESS;
}

// While idx is disabled its own pin is not reserved, so there is no self-conflict.
// The restore paths and the SPDIF input enable command share this check.
bool spdif_input_enable_acceptable(uint8_t idx) {
    if (idx == 0 || idx >= SPDIF_RX_NUM_INPUTS) return false;
    uint8_t pin = spdif_rx_pin_for_index(idx);
    return PIN_VALID_FOR(DSPI_PIN_FN_SPDIF_RX, pin) && !is_pin_in_use(pin, 0xFF);
}

// The restore paths reject an inconsistent pushed or stored I2S RX config as a
// unit with this before it can reach i2s_input_start(). bck2_pin is the SPLIT
// mode slave clock base, or 0xFF; data pins must avoid both clock pairs.
bool i2s_rx_pin_set_acceptable(const uint8_t *pins, uint8_t n_pairs,
                               uint8_t bck_pin, uint8_t bck2_pin) {
    if (n_pairs > I2S_RX_MAX_PAIRS) return false;
    for (uint8_t p = 0; p < n_pairs; p++) {
        uint8_t pin = pins[p];
        if (!PIN_VALID_FOR(DSPI_PIN_FN_I2S_RX, pin)) return false;
        if (pin == bck_pin || pin == (uint8_t)(bck_pin + 1)) return false;
        if (bck2_pin != 0xFF &&
            (pin == bck2_pin || pin == (uint8_t)(bck2_pin + 1))) return false;
        if (pin_used_by_fixed_peripheral(pin, 0xFF)) return false;
        for (uint8_t q = 0; q < p; q++)
            if (pins[q] == pin) return false;   // mutual distinctness
    }
    return true;
}

// BCK/LRCLK are push-pull outputs, so a collision is driver contention. BCK vs
// RX data pins is deliberately not checked here: i2s_rx_pin_set_acceptable()
// covers it, and checking against not-yet-restored RX pins would reject valid configs.
#ifdef DSPI_PLATFORM_PINS
static bool i2s_bck_pair_acceptable(uint8_t fn, uint8_t bck_pin) {
#else
bool i2s_bck_pin_acceptable(uint8_t bck_pin) {
#endif
    uint8_t lrck = (uint8_t)(bck_pin + 1);
    if (!PIN_VALID_FOR(fn, bck_pin) || !PIN_VALID_FOR(fn, lrck)) return false;
    if (pin_used_by_fixed_peripheral(bck_pin, 0xFF)) return false;
    if (pin_used_by_fixed_peripheral(lrck, 0xFF)) return false;
    return true;
}
#ifdef DSPI_PLATFORM_PINS
bool i2s_bck_pin_acceptable(uint8_t bck_pin) {
    return i2s_bck_pair_acceptable(DSPI_PIN_FN_I2S_CLK, bck_pin);
}
bool i2s_bck_slave_pin_acceptable(uint8_t bck_pin) {
    return i2s_bck_pair_acceptable(DSPI_PIN_FN_I2S_CLK_SLAVE, bck_pin);
}
#endif

// Ignores MCK's own claim so a kept pin passes. The later BCK check covers the
// clock pair against MCK; RX data pins check MCK.
bool i2s_mck_pin_acceptable(uint8_t pin) {
    return PIN_VALID_FOR(DSPI_PIN_FN_MCK, pin) && !pin_used_by_fixed_peripheral_except_mck(pin, 0xFF);
}

#if PICO_RP2350
// ADAT is a push-pull output, so any owned pin is driver contention; ADAT's own
// current claim never blocks it.
bool adat_pin_acceptable(uint8_t pin) {
    if (!PIN_VALID_FOR(DSPI_PIN_FN_ADAT_OUT, pin)) return false;
    if (adat_output_config_enabled() && pin == adat_output_pin()) return true;
    return !is_pin_in_use(pin, 0xFF);
}

// The RX only listens, so sharing the ADAT TX pin is the supported loopback
// self-test, but only while ADAT output is enabled.
bool adat_input_pin_acceptable(uint8_t pin) {
    if (!PIN_VALID_FOR(DSPI_PIN_FN_ADAT_IN, pin)) return false;
    if (adat_output_config_enabled() && pin == adat_output_pin()) return true;
#ifdef DSPI_PLATFORM_PINS
    // A platform build's ADAT input may share S/PDIF input 1's pin, its one receiver input; DSPi runs
    // one input source at a time.
    if (pin == spdif_rx_pin) return true;
#endif
    return !is_pin_in_use(pin, 0xFF);
}
#endif

// The I2S clock pair counts as ALWAYS claimed: control pins are long-lived, and
// a clash would otherwise surface only when I2S starts and stomps the control link.
uint8_t ctrl_iface_check_pin(uint8_t pin) {
    if (!is_valid_gpio_pin(pin))                              return PIN_CONFIG_INVALID_PIN;
    if (i2s_clock_pin_claimed(pin))                           return PIN_CONFIG_PIN_IN_USE;
    if (is_pin_in_use(pin, 0xFF))                             return PIN_CONFIG_PIN_IN_USE;
    return PIN_CONFIG_SUCCESS;
}
