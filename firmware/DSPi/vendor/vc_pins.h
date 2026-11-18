/*
 * vc_pins.h: the one place every pin decision is made (vc_pins.c). SET handlers,
 * deferred applies, the restore paths and the availability queries all call these
 * checks, so a host query can never disagree with a SET. Design and rules:
 * Documentation/Features/pin_availability_query_spec.md. Status returns are PIN_CONFIG_*.
 */

#ifndef VC_PINS_H
#define VC_PINS_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "audio_input.h"        // SPDIF_RX_NUM_INPUTS, I2S_RX_MAX_PAIRS
#include "control_surfaces.h"   // CS_MAX_BINDINGS

// A pin's validity for one function. Without DSPI_PLATFORM_PINS the function is
// ignored; a platform build applies its own rule (platform_pins.h).
#ifdef DSPI_PLATFORM_PINS
#define PIN_VALID_FOR(fn, pin) dspi_platform_pin_valid(fn, pin)
#else
#define PIN_VALID_FOR(fn, pin) is_valid_gpio_pin(pin)
#endif

// Platform default GPIO of each pin output, in output_pins[] order.
extern const uint8_t default_output_pins[NUM_PIN_OUTPUTS];

bool is_valid_gpio_pin(uint8_t pin);

// Pin functions. The values reach hosts (SYS.PIN_CLAIMS byte 1, the SYS.PIN_STATE
// substitution mask): append only, never renumber.
enum {
    PIN_FN_NONE      = 0,
    PIN_FN_OUTPUT    = 1,    // index = output slot
    PIN_FN_PDM       = 2,
    PIN_FN_I2S_CLK   = 3,    // index 0 master or unified pair, 1 slave pair; role 1 = LRCLK
    PIN_FN_MCK       = 4,
    PIN_FN_ADAT_TX   = 5,
    PIN_FN_ADAT_RX   = 6,
    PIN_FN_SPDIF_RX  = 7,    // index = S/PDIF input
    PIN_FN_I2S_RX    = 8,    // index = stereo pair
    PIN_FN_DAC_MUTE  = 9,
    PIN_FN_UART_CTRL = 10,   // role 0 TX, 1 RX
    PIN_FN_I2C_CTRL  = 11,   // role 0 SDA, 1 SCL
    PIN_FN_CS        = 12,   // index = binding slot
    PIN_FN_COUNT
};

#define PIN_CLAIM_ACTIVE  0u   // the function is in use (live or pending) and holds the pin
#define PIN_CLAIM_STORED  1u   // a stored preference of a function not in use; blocks nothing

// One claim, laid out as the SYS.PIN_CLAIMS wire entry. meta: bits 1:0 state,
// bits 3:2 role, bits 7:4 CS binding type.
typedef struct {
    uint8_t gpio;
    uint8_t fn;
    uint8_t idx;
    uint8_t meta;
} PinClaim;

static inline uint8_t pin_claim_state(const PinClaim *c) { return c->meta & 3u; }
static inline uint8_t pin_claim_role(const PinClaim *c)  { return (c->meta >> 2) & 3u; }
static inline uint8_t pin_claim_cs_type(const PinClaim *c) { return c->meta >> 4; }

#if PICO_RP2350
#define PIN_ADAT_CLAIMS 2
#else
#define PIN_ADAT_CLAIMS 0
#endif

// Every claim a quiet device can list: SYS.PIN_CLAIMS must hold this many.
#define PIN_CLAIMS_STEADY_MAX (NUM_PIN_OUTPUTS + PIN_ADAT_CLAIMS + SPDIF_RX_NUM_INPUTS + \
                               I2S_RX_MAX_PAIRS + 4 + 1 + 1 + 2 + 2 + 2 * CS_MAX_BINDINGS)
// A Control Surfaces revert can hold the outgoing and the reloading bindings at once.
#define PIN_STATE_MAX_CLAIMS  (PIN_CLAIMS_STEADY_MAX + 2 * CS_MAX_BINDINGS)

typedef struct PinState {
    uint8_t  count;
    bool     overflow;          // a claim was dropped; checks then refuse every pin
    PinClaim claims[PIN_STATE_MAX_CLAIMS];
} PinState;

void pin_state_reset(PinState *st);
void pin_state_add(PinState *st, uint8_t gpio, uint8_t fn, uint8_t idx,
                   uint8_t role, uint8_t state, uint8_t cs_type);

// Every pin-relevant setting of the device's audio IO, as one consistent set.
// UART, I2C, Control Surfaces and (outside a bulk push) the DAC mute are not
// carried: their modules add their own live claims.
typedef struct {
    uint8_t output_pins[NUM_PIN_OUTPUTS];          // last entry is PDM
    uint8_t output_types[NUM_SPDIF_INSTANCES];
    uint8_t pdm_enabled;
    uint8_t input_source;
    uint8_t bck_pin[2];                            // [0] master or unified, [1] slave
    uint8_t clock_pin_mode;
    uint8_t clock_mode;
    uint8_t mck_enabled;
    uint8_t mck_pin;
    uint8_t spdif_rx_pin[SPDIF_RX_NUM_INPUTS];
    uint8_t spdif_rx_enabled_ext;
    uint8_t i2s_rx_pin[I2S_RX_MAX_PAIRS];
    uint8_t i2s_rx_pairs;
    uint8_t adat_tx_enabled, adat_tx_pin;          // RP2350 only; zero elsewhere
    uint8_t adat_rx_enabled, adat_rx_pin;          // adat_rx_pin 0xFF = unset
    uint8_t has_dac_mute;                          // bulk push only: the blob carries it
    uint8_t dac_mute_enabled, dac_mute_pin;
} PinConfig;

// Live globals into a PinConfig. Pending switches are not applied.
void pin_config_capture(PinConfig *cfg);
// Factory pin set, the base a failed boot restore falls back to.
void pin_config_defaults(PinConfig *cfg);

// ---- Checks ----

typedef enum {
    PIN_CHECK_STORE = 0,    // validity only: a stored pin of a function not in use
    PIN_CHECK_CLAIM = 1,    // validity plus conflicts against every active claim
} PinCheckMode;

// All of one function's pins as a unit. pin[1] is 0xFF for a one-pin function and is
// derived (BCK + 1) for the I2S clock pair.
typedef struct {
    uint8_t fn;
    uint8_t idx;
    uint8_t pin[2];
} PinRequest;

uint8_t pin_check(const PinState *st, const PinRequest *rq, PinCheckMode mode);

// True if `pin` would clash with an active claim of another function (sharing
// exceptions applied). Used by Control Surfaces, which checks its own family itself.
bool pin_conflicts(const PinState *st, uint8_t fn, uint8_t idx, uint8_t pin);

// Pin-pair rules shared with the CS display. pin_pair_group: the UART or I2C instance a
// GPIO muxes to. pin_role_valid: the GPIO can be role 0 (TX, SDA) or 1 (RX, SCL).
uint8_t pin_pair_group(uint8_t fn, uint8_t pin);
bool    pin_role_valid(uint8_t fn, uint8_t role, uint8_t pin);
// The I2C instance the other I2C user holds live (CS display for fn I2C_CTRL, the I2C
// target for fn CS), or -1.
int     pin_i2c_instance_taken(const PinState *st, uint8_t fn);

// The live state: every module's claims, with pending output type, input source and
// clock mode switches counted as in use. One static buffer; not reentrant.
const PinState *pin_state_live(void);
bool pin_fn_in_use(const PinState *st, uint8_t fn, uint8_t idx);

// A live refusal of every pin for this function (PIN_CONFIG_OUTPUT_ACTIVE), else success.
uint8_t pin_fn_status(uint8_t fn, uint8_t idx);

// A pin SET against the live state: function status, then the check in the mode the
// function's use selects.
uint8_t pin_check_set(uint8_t fn, uint8_t idx, uint8_t pin0, uint8_t pin1);

// ---- Start checks: a function about to begin using its stored pins ----

uint8_t pin_start_check(uint8_t fn, uint8_t idx);
// Output slot `slot` becomes `type`, or the input source becomes `source`, or the clock
// mode or clock-pin mode changes. Each refuses before anything is queued.
uint8_t pin_start_check_output_type(uint8_t slot, uint8_t type);
uint8_t pin_start_check_source(uint8_t source);
uint8_t pin_start_check_i2s_rx_pairs(uint8_t pairs);
uint8_t pin_start_check_clock(uint8_t clock_pin_mode, uint8_t clock_mode);
uint8_t pin_start_check_pdm(void);

// ---- Restore: bulk push, preset load, factory reset, boot ----

#define PIN_VALIDATE_BOOT        0x01u   // control interfaces and CS are not up yet
#define PIN_VALIDATE_FIXED_PINS  0x02u   // IO pins are not part of this restore

typedef struct {
    uint16_t substituted;      // bit n: pin function n was replaced or forced off
    bool     source_reverted;  // the proposed input source could not start
    bool     pdm_disabled;     // the proposed PDM enable could not claim its pin
} PinValidateResult;

// Correct `cfg` in place so every function in use can claim its pins, by the fixed
// priority order and fallback rule of the spec (section 7). `base` is the live config
// at run time and the factory set at boot.
void pin_config_validate(PinConfig *cfg, const PinConfig *base, uint8_t flags,
                         PinValidateResult *res);

// Set the live globals from a validated config, arming the deferred hot-swap and
// restart flags; the output clock mode, rates and multipliers are the caller's.
void pin_config_install(const PinConfig *cfg);

// Validate against the live base (or the factory set with PIN_VALIDATE_BOOT), install,
// switch PDM off and drop a pending input switch as the result says, and note it.
void pin_restore_apply(PinConfig *cfg, uint8_t flags, PinValidateResult *res);

// The substitution mask of the last restore, for SYS.PIN_STATE.
void     pin_restore_note(const PinValidateResult *res);
uint16_t pin_restore_substituted(void);

// ---- Host queries ----

// Availability reply (spec section 8.2). Up to two groups of two roles.
typedef struct PinAvail {
    uint8_t  status;           // VcResult-space function status, or CS_STATUS_*
    uint8_t  flags;
    uint8_t  groups;
    uint8_t  roles;
    uint64_t valid[2][2];
    uint64_t free[2][2];
    uint64_t shareable;        // CS buttons only
} PinAvail;

#define PIN_AVAIL_IN_USE      0x01u
#define PIN_AVAIL_PAIR_PLUS1  0x02u
#define PIN_AVAIL_UNASSIGNED  0x04u
#define PIN_AVAIL_START_OK    0x08u

void     pin_avail(uint8_t fn, uint8_t idx, PinAvail *out);
// cs_layout: the fixed 80-byte CS reply (four mask pairs laid out for 2 x 2, then the
// shareable mask). Returns the reply length, 0 if cap is short.
uint16_t pin_avail_serialize(const PinAvail *a, uint32_t generation, uint8_t *out, uint16_t cap,
                             bool cs_layout);

// The generation counter (spec section 9). sync recomputes it from the live state;
// with push set, a change raises NOTIFY_EVT_PIN_MAP_CHANGED.
uint32_t pin_generation_sync(bool push);

// GPIOs that exist and are assignable to at least one function.
uint64_t pin_gpio_mask(void);
uint8_t  pin_gpio_count(void);

#endif // VC_PINS_H
