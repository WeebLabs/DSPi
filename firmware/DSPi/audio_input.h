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
    INPUT_SOURCE_USB    = 0,
    INPUT_SOURCE_SPDIF  = 1,   // SPDIF input 1 (always enabled)
    INPUT_SOURCE_I2S    = 2,
    INPUT_SOURCE_ADAT   = 3,   // 8-channel ADAT input (RP2350 only, disabled until enabled by host)
    INPUT_SOURCE_SPDIF2 = 4,   // Optional SPDIF input 2 (disabled until enabled by host)
    INPUT_SOURCE_SPDIF3 = 5,   // Optional SPDIF input 3 (disabled until enabled by host)
    INPUT_SOURCE_SPDIF4 = 6,   // Optional SPDIF input 4 (disabled until enabled by host)
} InputSource;

#define INPUT_SOURCE_MAX    INPUT_SOURCE_SPDIF4   // Highest valid value

// Number of selectable SPDIF inputs. Input 0 (INPUT_SOURCE_SPDIF) is the
// always-present one; inputs 1..3 (SPDIF2..SPDIF4) are optional and share the
// single RX PIO state machine; only the active one ever claims its GPIO.
#define SPDIF_RX_NUM_INPUTS 4

// The optional inputs must stay contiguous from INPUT_SOURCE_SPDIF2: the
// index<->source helpers below are arithmetic, not a lookup table.
_Static_assert(INPUT_SOURCE_MAX == INPUT_SOURCE_SPDIF2 + SPDIF_RX_NUM_INPUTS - 2,
               "optional SPDIF sources must be contiguous and end at INPUT_SOURCE_MAX");

// Default SPDIF RX GPIO pin.  GPIO 5 sits just below the output-pin
// neighborhood (SPDIF outs on 6-9, PDM on 10) and is unused by any
// default output, leaving GPIO 11 free for the DAC hardware-mute
// default (see DAC_HW_MUTE_DEFAULT_PIN in dac_hw_mute.h).
#define PICO_SPDIF_RX_PIN_DEFAULT  5

// Default GPIOs for the optional SPDIF inputs 2, 3 and 4.  All are free of
// every default assignment on RP2350; GPIO 21 is the RP2040 MCK default, so
// enable-time validation rejects the clash if MCK is enabled there.
#define PICO_SPDIF_RX_PIN2_DEFAULT 20
#define PICO_SPDIF_RX_PIN3_DEFAULT 21
#define PICO_SPDIF_RX_PIN4_DEFAULT 22

// Default I2S RX data GPIO (stereo pair 0).  The four data-pin defaults are the
// contiguous block GPIO 1/2/3/4 (pairs 0/1/2/3), all unused by any default
// assignment (SPDIF RX 5, outputs 6-9, PDM 10, DAC mute 11, BCK 14, LRCLK 15,
// MCK 21 on RP2040 / 13 on RP2350, slave clock pair 12/13 on RP2040 / 26/27 on
// RP2350), so enabling 4/6/8-channel input out of the box never self-collides.
// Real boards override these per wiring.
#define PICO_I2S_RX_PIN_DEFAULT    1

// Maximum I2S RX stereo pairs.  Each pair is one PIO state machine + one DMA
// ring + one serial-data pin, sharing the single BCK/LRCLK.  RP2350 fans out to
// 4 pairs (8 channels), matching its 8 input channels and the freed DMA budget;
// RP2040's unified channel model has only the stereo pair and no spare PIO SM /
// DMA channels for more, so it stays at one pair.
#if PICO_RP2350
#define I2S_RX_MAX_PAIRS    4
#else
#define I2S_RX_MAX_PAIRS    1
#endif
#define I2S_RX_MAX_CHANNELS (I2S_RX_MAX_PAIRS * 2)

// SPDIF RX lock debounce — firmware constant, not configurable via vendor command.
// After the library reports lock, wait this many ms before unmuting output.
#define SPDIF_RX_LOCK_DEBOUNCE_MS  100

// Current active input source (definition in audio_input.c)
extern volatile uint8_t active_input_source;

// True while something is actually feeding process_input_block(): a locked or
// running source, or the siggen idle pump when it would really run.  Defined
// in main.c beside the pipeline-settle logic that is its other consumer.
bool pipeline_producer_is_streaming(void);

// SPDIF RX pin (device-level setting, stored in PresetDirectory)
extern uint8_t spdif_rx_pin;

// GPIOs for the optional SPDIF inputs 2..4 ([0] = SPDIF2 ... [2] = SPDIF4).
// Same persistence model as spdif_rx_pin.  A disabled input's pin is only a
// stored preference: it is not reserved against other functions and is never
// claimed in hardware until the input is enabled AND selected.
extern uint8_t spdif_rx_pin_ext[SPDIF_RX_NUM_INPUTS - 1];

// Enable mask for the optional SPDIF inputs: bit 0 = SPDIF2 ... bit 2 = SPDIF4.
// 0 by default; the extra inputs are absent from the source list and invisible
// to pin-conflict validation.  SPDIF input 1 is always enabled.
extern uint8_t spdif_rx_enabled_ext;

// All bits of spdif_rx_enabled_ext that name a real input; anything wider
// arriving from the wire or flash is masked off before use.
#define SPDIF_RX_ENABLED_EXT_MASK  ((uint8_t)((1u << (SPDIF_RX_NUM_INPUTS - 1)) - 1))

// I2S RX serial-data pins, one per stereo pair, each independently
// configurable (same per-pin persistence model as spdif_rx_pin).  [0] is the
// always-present stereo pair (default PICO_I2S_RX_PIN_DEFAULT); [1..3] are used
// only when i2s_input_channels selects more than 2 channels and are assigned by
// the host (multichannel input requires wiring one ADC data line per pair).
extern uint8_t i2s_rx_pin[I2S_RX_MAX_PAIRS];

// Active I2S input channel count: 2/4/6/8 on RP2350, always 2 on RP2040.
// Determines how many stereo pairs (SMs / DMA rings / data pins) the input
// claims.  Changing it requires a full input restart (i2s_input_restart_pending).
extern uint8_t i2s_input_channels;

// Selected sample rate for I2S input (device is the rate authority in
// I2S input mode; 44100 / 48000 / 96000)
extern uint32_t i2s_input_rate;

// Deferred input source switch (set by vendor command, handled in main loop)
extern volatile bool input_source_change_pending;
extern volatile uint8_t pending_input_source;

// Deferred I2S RX hot-swaps (set by vendor commands / bulk apply, handled
// in main loop): data-pin change, and full restart after a BCK pin change
// while the input SM is the clock master
extern volatile bool i2s_rx_pin_change_pending;
extern volatile bool i2s_input_restart_pending;

// I2S clock mode: who owns BCK/LRCLK while I2S is the input source.
// MASTER (default) = the device generates BCK/LRCLK and is the rate
// authority (selected via REQ_SET_INPUT_RATE).  SLAVE = an external master
// drives BCK/LRCLK (both pins become inputs); the rate is auto-detected
// from the external clocks and every output is slaved to them (I2S slots
// edge-driven, SPDIF/ADAT servo rate-matched, MCK forced off).  Has no
// effect while the input source is not I2S.
typedef enum {
    I2S_CLOCK_MODE_MASTER = 0,
    I2S_CLOCK_MODE_SLAVE  = 1,
} I2sClockMode;

extern uint8_t i2s_clock_mode;

// Deferred clock-mode apply: a mode change while I2S input is live needs a
// full input restart plus an output-side rebuild (I2S output SMs switch
// between clkout/dataout and the external-clock program), handled in the
// main loop.  The handler copies pending_i2s_clock_mode into i2s_clock_mode
// at apply time; boot/preset paths may also write the global directly when
// nothing I2S is live (dormant apply).
extern volatile bool i2s_clock_mode_change_pending;
extern volatile uint8_t pending_i2s_clock_mode;

// I2S clock-pin mode: whether both clock modes share one BCK/LRCLK pair
// (UNIFIED, legacy behavior) or each mode has its own (SPLIT: master mode
// drives i2s_bck_pin, slave mode listens on i2s_bck_pin_slave).  LRCLK =
// BCK + 1 in both modes (PIO side-set constraint).  The slave pair is fully
// dormant in UNIFIED mode.  See Documentation/Features/clock_pins_spec.md.
typedef enum {
    I2S_CLOCK_PIN_MODE_UNIFIED = 0,
    I2S_CLOCK_PIN_MODE_SPLIT   = 1,
} I2sClockPinMode;

extern uint8_t i2s_clock_pin_mode;   // defined in usb_audio.c
extern uint8_t i2s_bck_pin_slave;    // slave-mode BCK (SPLIT only); LRCLK = +1

// The BCK pin the current clock mode actually uses (LRCLK = return + 1).
// Every hardware consumer (TX clock_pin_base, RX start snapshot, rebuild
// change-detection) reads this instead of i2s_bck_pin so SPLIT mode routes
// the slave role onto its own pair.  Callers that rebuild after a deferred
// clock-mode flip read it AFTER the handler updates i2s_clock_mode, so the
// result tracks the mode the rebuild is for.
static inline uint8_t i2s_effective_bck_pin(void) {
    extern uint8_t i2s_bck_pin;
    return (i2s_clock_pin_mode == I2S_CLOCK_PIN_MODE_SPLIT &&
            i2s_clock_mode == I2S_CLOCK_MODE_SLAVE) ? i2s_bck_pin_slave
                                                    : i2s_bck_pin;
}

// True if `pin` is one of the configured I2S clock GPIOs: the master/unified
// BCK/LRCLK pair, plus the slave pair when SPLIT mode is configured.  Used by
// the "clock pins are always claimed" validation sites so nothing else can be
// assigned onto a pair the next mode switch will drive or listen on.
static inline bool i2s_clock_pin_claimed(uint8_t pin) {
    extern uint8_t i2s_bck_pin;
    if (pin == i2s_bck_pin || pin == (uint8_t)(i2s_bck_pin + 1)) return true;
    if (i2s_clock_pin_mode == I2S_CLOCK_PIN_MODE_SPLIT &&
        (pin == i2s_bck_pin_slave || pin == (uint8_t)(i2s_bck_pin_slave + 1)))
        return true;
    return false;
}

// True when I2S slave clocking is in force (SLAVE selected AND I2S is the
// active input source)
static inline bool i2s_slave_mode_active(void) {
    return i2s_clock_mode == I2S_CLOCK_MODE_SLAVE &&
           active_input_source == INPUT_SOURCE_I2S;
}

// ADAT input clock mode: who owns the clock domain of the incoming stream.
// MASTER (default) = the far end locks to DSPi's ADAT output, so the return
// stream is already in our clock domain: no rate detection, no servo; the
// device is the rate authority via REQ_SET_INPUT_RATE (shared with I2S
// master mode). SLAVE = external gear owns the clock; the wire rate is
// auto-detected and every output is servo rate-matched to it, exactly like
// SPDIF input. Has no effect while the input source is not ADAT.
typedef enum {
    ADAT_CLOCK_MODE_MASTER = 0,
    ADAT_CLOCK_MODE_SLAVE  = 1,
} AdatClockMode;

// ADAT input config (defined in audio_input.c; persists like the optional
// SPDIF inputs: disabled by default, pin 0xFF = unset, GPIO claimed only
// while ADAT is the active source). RP2040 keeps the state for wire/preset
// round-trips but can never select the source.
extern uint8_t adat_input_enabled;
extern uint8_t adat_input_pin;
extern uint8_t adat_clock_mode;

// Deferred ADAT applies (handled in main loop): clock-mode flip, and input
// restart after an enable/pin change while ADAT is the active source.
extern volatile bool adat_clock_mode_change_pending;
extern volatile uint8_t pending_adat_clock_mode;
extern volatile bool adat_input_restart_pending;

// True when ADAT slave clocking is in force (SLAVE selected AND ADAT is the
// active input source)
static inline bool adat_slave_mode_active(void) {
    return adat_clock_mode == ADAT_CLOCK_MODE_SLAVE &&
           active_input_source == INPUT_SOURCE_ADAT;
}

// Structurally valid input source value. ADAT (3) is structurally valid on
// both platforms so presets round-trip; selectability is gated separately.
static inline bool input_source_valid(uint8_t src) {
    return src <= INPUT_SOURCE_MAX;
}

// True for any of the SPDIF inputs
static inline bool input_source_is_spdif(uint8_t src) {
    return src == INPUT_SOURCE_SPDIF ||
           (src >= INPUT_SOURCE_SPDIF2 && src <= INPUT_SOURCE_MAX);
}

// SPDIF input index (0..SPDIF_RX_NUM_INPUTS-1) for a source; 0 for non-SPDIF
static inline uint8_t spdif_index_for_source(uint8_t src) {
    if (src >= INPUT_SOURCE_SPDIF2 && src <= INPUT_SOURCE_MAX)
        return (uint8_t)(src - INPUT_SOURCE_SPDIF2 + 1);
    return 0;
}

// InputSource value for a SPDIF input index (0..SPDIF_RX_NUM_INPUTS-1)
static inline uint8_t spdif_source_for_index(uint8_t idx) {
    if (idx == 0 || idx >= SPDIF_RX_NUM_INPUTS) return INPUT_SOURCE_SPDIF;
    return (uint8_t)(INPUT_SOURCE_SPDIF2 + idx - 1);
}

// Configured GPIO for a SPDIF input index (0..SPDIF_RX_NUM_INPUTS-1)
static inline uint8_t spdif_rx_pin_for_index(uint8_t idx) {
    return (idx == 0) ? spdif_rx_pin : spdif_rx_pin_ext[idx - 1];
}

// Factory GPIO for a SPDIF input index; the reset-to-default target for
// REQ_SET_SPDIF_RX_PIN and the fallback for an unset stored pin.
static inline uint8_t spdif_rx_pin_default_for_index(uint8_t idx) {
    switch (idx) {
        case 1:  return PICO_SPDIF_RX_PIN2_DEFAULT;
        case 2:  return PICO_SPDIF_RX_PIN3_DEFAULT;
        case 3:  return PICO_SPDIF_RX_PIN4_DEFAULT;
        default: return PICO_SPDIF_RX_PIN_DEFAULT;
    }
}

// True if SPDIF input `idx` is enabled (index 0 is always enabled)
static inline bool spdif_input_enabled(uint8_t idx) {
    return idx == 0 || ((spdif_rx_enabled_ext >> (idx - 1)) & 1u) != 0;
}

// Valid AND currently offered in the source list (disabled SPDIF 2/3 and
// disabled/unsupported ADAT are not)
static inline bool input_source_selectable(uint8_t src) {
    if (!input_source_valid(src)) return false;
    if (src == INPUT_SOURCE_ADAT) {
#if PICO_RP2350
        return adat_input_enabled != 0 && adat_input_pin != 0xFF;
#else
        return false;   // no PIO/DMA/channel budget on RP2040
#endif
    }
    if (!input_source_is_spdif(src)) return true;
    return spdif_input_enabled(spdif_index_for_source(src));
}

// GPIO the RX library should run on for the currently-active SPDIF source
static inline uint8_t spdif_rx_active_pin(void) {
    return spdif_rx_pin_for_index(spdif_index_for_source(active_input_source));
}

// Validate a proposed I2S RX data-pin set (the first `n_pairs` entries of
// `pins[]` are the pairs that will be active) against the effective `bck_pin`:
// each active pin must be a valid GPIO, not a clock pin (BCK/LRCLK), not used by
// a fixed peripheral, and mutually distinct.  The bulk/preset restore paths use
// this to reject an inconsistent pushed/stored I2S config as a unit before it
// can reach i2s_input_start().  Defined in vendor_commands.c (alongside the
// other pin helpers); declared here so the restore paths can call it without
// pulling in the TinyUSB-heavy vendor_commands.h.
// bck2_pin: secondary (slave-pair) clock base when SPLIT clock-pin mode is in
// force, or 0xFF for none.
bool i2s_rx_pin_set_acceptable(const uint8_t *pins, uint8_t n_pairs,
                               uint8_t bck_pin, uint8_t bck2_pin);

// True if `bck_pin` (LRCLK = bck_pin + 1) is acceptable as the I2S clock pair:
// both valid GPIOs and neither colliding with a fixed peripheral.  The
// bulk/preset restore paths use this to reject a pushed/stored BCK they would
// otherwise install raw (BCK/LRCLK are clock OUTPUTS — a collision is driver
// contention and an invalid GPIO can fault pio_gpio_init()).  Defined in
// vendor_commands.c; declared here for the same reason as above.
bool i2s_bck_pin_acceptable(uint8_t bck_pin);

// True if a restored, enabled MCK may drive `pin` (no other fixed-peripheral
// owner; MCK's own claim ignored).  Defined in vendor_commands.c.
bool i2s_mck_pin_acceptable(uint8_t pin);

// True if SPDIF input `idx` (1..SPDIF_RX_NUM_INPUTS-1) could be enabled right now: its configured
// GPIO is valid and not claimed by any other function.  Defined in
// vendor_commands.c (alongside the other pin helpers); declared here so the
// bulk/preset restore paths can validate a stored enable before applying it.
bool spdif_input_enable_acceptable(uint8_t idx);

// True if `pin` could be the ADAT input (RX) GPIO right now: valid GPIO and
// either the ADAT output's configured pin (the supported loopback self-test,
// since the RX only listens) or not claimed by any other function.  Defined in
// vendor_commands.c (RP2350 only); declared here so the bulk/preset restore
// paths can validate a stored ADAT input pin before applying it.
bool adat_input_pin_acceptable(uint8_t pin);

// I2S input rate wire/flash encoding (1 byte): 0 = 44100, 1 = 48000,
// 2 = 96000. Unknown values decode to 48000.
static inline uint8_t i2s_rate_encode(uint32_t hz) {
    return (hz == 44100) ? 0 : ((hz == 96000) ? 2 : 1);
}
static inline uint32_t i2s_rate_decode(uint8_t enc) {
    return (enc == 0) ? 44100 : ((enc == 2) ? 96000 : 48000);
}

#endif // AUDIO_INPUT_H
