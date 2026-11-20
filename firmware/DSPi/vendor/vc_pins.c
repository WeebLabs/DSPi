/*
 * vc_pins.c: every pin decision (vc_pins.h). A function holds its pins only while
 * in use; checks run against a claim list built from the live or proposed config.
 * Design: Documentation/Features/pin_availability_query_spec.md.
 */

#include <string.h>
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
#include "pdm_generator.h"
#include "notify.h"
#include "vc_ctrl.h"            // VcResult codes for the availability status
#include "config.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"

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

_Static_assert(sizeof(PinClaim) == 4, "PinClaim is the 4-byte SYS.PIN_CLAIMS entry");
_Static_assert(PIN_STATE_MAX_CLAIMS <= 255, "PinState.count is a uint8_t");

bool is_valid_gpio_pin(uint8_t pin) {
#ifdef DSPI_PLATFORM_PINS
    return dspi_platform_pin_valid(DSPI_PIN_FN_ANY, pin);
#else
    if (pin >= 23 && pin <= 25) return false;   // Power/LED
#if PICO_RP2350
    return pin <= 29;
#else
    return pin <= 28;
#endif
#endif
}

// ---------------------------------------------------------------------------
// Claim lists
// ---------------------------------------------------------------------------

void pin_state_reset(PinState *st) {
    st->count = 0;
    st->overflow = false;
}

void pin_state_add(PinState *st, uint8_t gpio, uint8_t fn, uint8_t idx,
                   uint8_t role, uint8_t state, uint8_t cs_type) {
    if (st->count >= PIN_STATE_MAX_CLAIMS) {
        st->overflow = true;
        return;
    }
    PinClaim *c = &st->claims[st->count++];
    c->gpio = gpio;
    c->fn = fn;
    c->idx = idx;
    c->meta = (uint8_t)((state & 3u) | ((role & 3u) << 2) | ((cs_type & 0xFu) << 4));
}

static bool claim_active_on(const PinState *st, uint8_t fn, uint8_t gpio) {
    for (uint8_t i = 0; i < st->count; i++) {
        const PinClaim *c = &st->claims[i];
        if (c->fn == fn && c->gpio == gpio && pin_claim_state(c) == PIN_CLAIM_ACTIVE) return true;
    }
    return false;
}

bool pin_fn_in_use(const PinState *st, uint8_t fn, uint8_t idx) {
    for (uint8_t i = 0; i < st->count; i++) {
        const PinClaim *c = &st->claims[i];
        if (c->fn == fn && c->idx == idx && pin_claim_state(c) == PIN_CLAIM_ACTIVE) return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Configs and their claims
// ---------------------------------------------------------------------------

void pin_config_capture(PinConfig *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    memcpy(cfg->output_pins, output_pins, NUM_PIN_OUTPUTS);
    memcpy(cfg->output_types, output_types, NUM_SPDIF_INSTANCES);
    cfg->pdm_enabled = matrix_mixer.outputs[NUM_OUTPUT_CHANNELS - 1].enabled ? 1 : 0;
    cfg->input_source = active_input_source;
    cfg->bck_pin[0] = i2s_bck_pin;
    cfg->bck_pin[1] = i2s_bck_pin_slave;
    cfg->clock_pin_mode = i2s_clock_pin_mode;
    cfg->clock_mode = i2s_clock_mode;
    cfg->mck_enabled = i2s_mck_enabled ? 1 : 0;
    cfg->mck_pin = i2s_mck_pin;
    for (uint8_t i = 0; i < SPDIF_RX_NUM_INPUTS; i++) cfg->spdif_rx_pin[i] = spdif_rx_pin_for_index(i);
    cfg->spdif_rx_enabled = spdif_rx_enabled ? 1 : 0;
    cfg->spdif_rx_count = spdif_rx_count;
    memcpy(cfg->i2s_rx_pin, i2s_rx_pin, I2S_RX_MAX_PAIRS);
    cfg->i2s_rx_enabled = i2s_input_enabled ? 1 : 0;
    cfg->i2s_rx_pairs = (uint8_t)(i2s_input_channels / 2);
#if PICO_RP2350
    cfg->adat_tx_enabled = adat_output_config_enabled() ? 1 : 0;
    cfg->adat_tx_pin = adat_output_pin();
    cfg->adat_rx_enabled = adat_input_enabled ? 1 : 0;
    cfg->adat_rx_pin = adat_input_pin;
#else
    cfg->adat_rx_pin = 0xFF;
#endif
    DacHwMuteConfig hw;
    dac_hw_mute_get_config(&hw);
    cfg->dac_mute_enabled = hw.enabled ? 1 : 0;
    cfg->dac_mute_pin = hw.pin;
}

void pin_config_defaults(PinConfig *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    memcpy(cfg->output_pins, default_output_pins, NUM_PIN_OUTPUTS);
    cfg->input_source = INPUT_SOURCE_USB;
    cfg->bck_pin[0] = PICO_I2S_BCK_PIN;
    cfg->bck_pin[1] = PICO_I2S_BCK_PIN_SLAVE;
    cfg->mck_pin = PICO_I2S_MCK_PIN;
    for (uint8_t i = 0; i < SPDIF_RX_NUM_INPUTS; i++)
        cfg->spdif_rx_pin[i] = spdif_rx_pin_default_for_index(i);
    cfg->spdif_rx_enabled = 1;
    cfg->spdif_rx_count = 1;
    for (uint8_t p = 0; p < I2S_RX_MAX_PAIRS; p++)
        cfg->i2s_rx_pin[p] = (uint8_t)(PICO_I2S_RX_PIN_DEFAULT + p);
    cfg->i2s_rx_pairs = 1;
#if PICO_RP2350
    cfg->adat_tx_pin = PICO_ADAT_PIN;
#endif
    cfg->adat_rx_pin = 0xFF;
    cfg->dac_mute_pin = DAC_HW_MUTE_PIN_NONE;
}

void pin_config_admit_source(PinConfig *c, uint8_t src) {
    if (src == INPUT_SOURCE_I2S) {
        c->i2s_rx_enabled = 1;
    } else if (input_source_is_spdif(src)) {
        uint8_t need = (uint8_t)(spdif_index_for_source(src) + 1);
        c->spdif_rx_enabled = 1;
        if (c->spdif_rx_count < need) c->spdif_rx_count = need;
    } else if (src == INPUT_SOURCE_ADAT && c->adat_rx_pin != 0xFF) {
        c->adat_rx_enabled = 1;
    }
}

bool pin_config_source_ok(const PinConfig *c) {
    uint8_t src = c->input_source;
    if (src == INPUT_SOURCE_I2S) return c->i2s_rx_enabled != 0;
    if (input_source_is_spdif(src))
        return c->spdif_rx_enabled != 0 && spdif_index_for_source(src) < c->spdif_rx_count;
    if (src == INPUT_SOURCE_ADAT) return c->adat_rx_enabled != 0 && c->adat_rx_pin != 0xFF;
    return true;
}

// The clock pair is reserved while the I2S input block is on, selected or not, so a
// switch to I2S never needs a pin it does not already hold.
static bool cfg_clocks_run(const PinConfig *c) {
    if (c->i2s_rx_enabled) return true;
    for (uint8_t i = 0; i < NUM_SPDIF_INSTANCES; i++)
        if (c->output_types[i] == OUTPUT_TYPE_I2S) return true;
    return false;
}

static uint8_t cfg_clock_pair(const PinConfig *c) {
    return (c->clock_pin_mode == I2S_CLOCK_PIN_MODE_SPLIT &&
            c->clock_mode == I2S_CLOCK_MODE_SLAVE) ? 1 : 0;
}

// The single definition of "in use" for every function a PinConfig carries.
static bool cfg_in_use(const PinConfig *c, uint8_t fn, uint8_t idx) {
    // PDM, MCK and the ADAT pair have one instance. Without this, check_started()'s
    // walk over every index sees them as new at idx 1..3 and clashes them with their
    // own idx 0 claim, so every enable was refused as PIN_IN_USE.
    if (idx != 0 && (fn == PIN_FN_PDM || fn == PIN_FN_MCK ||
                     fn == PIN_FN_ADAT_TX || fn == PIN_FN_ADAT_RX)) return false;
    switch (fn) {
        case PIN_FN_OUTPUT:   return true;
        case PIN_FN_PDM:      return c->pdm_enabled != 0;
        case PIN_FN_I2S_CLK:  return cfg_clocks_run(c) && cfg_clock_pair(c) == idx;
        case PIN_FN_MCK:      return c->mck_enabled != 0;
        case PIN_FN_ADAT_TX:  return c->adat_tx_enabled != 0;
        case PIN_FN_ADAT_RX:  return c->adat_rx_enabled != 0 && c->adat_rx_pin != 0xFF;
        case PIN_FN_SPDIF_RX: return c->spdif_rx_enabled != 0 && idx < c->spdif_rx_count;
        case PIN_FN_I2S_RX:   return c->i2s_rx_enabled != 0 && idx < c->i2s_rx_pairs;
        case PIN_FN_DAC_MUTE: return c->has_dac_mute && c->dac_mute_enabled &&
                                     c->dac_mute_pin != DAC_HW_MUTE_PIN_NONE;
        default:              return false;
    }
}

// The configs a claim list is built from: the live one, then optional views of it with
// queued switches applied. A function in use in any of them is active.
typedef struct {
    const PinConfig *v[3];
} CfgViews;

static bool in_use_any(const CfgViews *cv, uint8_t fn, uint8_t idx) {
    for (int k = 0; k < 3; k++)
        if (cv->v[k] && cfg_in_use(cv->v[k], fn, idx)) return true;
    return false;
}

static void add_fn(PinState *st, const CfgViews *cv,
                   uint8_t fn, uint8_t idx, uint8_t pin0, uint8_t pin1) {
    uint8_t state = in_use_any(cv, fn, idx) ? PIN_CLAIM_ACTIVE : PIN_CLAIM_STORED;
    if (pin0 != 0xFF) pin_state_add(st, pin0, fn, idx, 0, state, 0);
    if (pin1 != 0xFF) pin_state_add(st, pin1, fn, idx, 1, state, 0);
}

// Claims of every function a PinConfig carries, pins from the first view.
static void add_config_claims(PinState *st, const CfgViews *cv) {
    const PinConfig *a = cv->v[0];
    for (uint8_t i = 0; i < NUM_SPDIF_INSTANCES; i++)
        add_fn(st, cv, PIN_FN_OUTPUT, i, a->output_pins[i], 0xFF);
    add_fn(st, cv, PIN_FN_PDM, 0, a->output_pins[NUM_PIN_OUTPUTS - 1], 0xFF);
    for (uint8_t r = 0; r < 2; r++)
        add_fn(st, cv, PIN_FN_I2S_CLK, r, a->bck_pin[r], (uint8_t)(a->bck_pin[r] + 1));
    add_fn(st, cv, PIN_FN_MCK, 0, a->mck_pin, 0xFF);
#if PICO_RP2350
    add_fn(st, cv, PIN_FN_ADAT_TX, 0, a->adat_tx_pin, 0xFF);
    add_fn(st, cv, PIN_FN_ADAT_RX, 0, a->adat_rx_pin, 0xFF);
#endif
    for (uint8_t i = 0; i < SPDIF_RX_NUM_INPUTS; i++)
        add_fn(st, cv, PIN_FN_SPDIF_RX, i, a->spdif_rx_pin[i], 0xFF);
    for (uint8_t p = 0; p < I2S_RX_MAX_PAIRS; p++)
        add_fn(st, cv, PIN_FN_I2S_RX, p, a->i2s_rx_pin[p], 0xFF);
    if (a->has_dac_mute && a->dac_mute_pin != DAC_HW_MUTE_PIN_NONE)
        add_fn(st, cv, PIN_FN_DAC_MUTE, 0, a->dac_mute_pin, 0xFF);
}

static void add_ctrl_claims(PinState *st) {
    UartCtrlConfig u;
    bool u_live = uart_ctrl_live_config(&u);
    pin_state_add(st, u.tx_pin, PIN_FN_UART_CTRL, 0, 0, u_live ? PIN_CLAIM_ACTIVE : PIN_CLAIM_STORED, 0);
    pin_state_add(st, u.rx_pin, PIN_FN_UART_CTRL, 0, 1, u_live ? PIN_CLAIM_ACTIVE : PIN_CLAIM_STORED, 0);
    I2cCtrlConfig i;
    bool i_live = i2c_ctrl_live_config(&i);
    pin_state_add(st, i.sda_pin, PIN_FN_I2C_CTRL, 0, 0, i_live ? PIN_CLAIM_ACTIVE : PIN_CLAIM_STORED, 0);
    pin_state_add(st, i.scl_pin, PIN_FN_I2C_CTRL, 0, 1, i_live ? PIN_CLAIM_ACTIVE : PIN_CLAIM_STORED, 0);
}

static void add_dac_mute_claim(PinState *st) {
    DacHwMuteConfig hw;
    dac_hw_mute_get_config(&hw);
    if (hw.pin == DAC_HW_MUTE_PIN_NONE) return;
    pin_state_add(st, hw.pin, PIN_FN_DAC_MUTE, 0, 0,
                  dac_hw_mute_owns_pin(hw.pin) ? PIN_CLAIM_ACTIVE : PIN_CLAIM_STORED, 0);
}

// Hardware still driving or listening on a pin its config has let go of: a dormant
// clock-mode change, an ADAT reroute or a PDM fade-out not yet applied.
static void add_hw_claims(PinState *st, const PinConfig *live) {
    uint8_t bases[2] = { i2s_outputs_live_bck_pin(), i2s_input_live_bck_pin() };
    for (int k = 0; k < 2; k++) {
        uint8_t b = bases[k];
        if (b == 0xFF || claim_active_on(st, PIN_FN_I2S_CLK, b)) continue;
        uint8_t idx = (b == live->bck_pin[1] && b != live->bck_pin[0]) ? 1 : 0;
        pin_state_add(st, b, PIN_FN_I2S_CLK, idx, 0, PIN_CLAIM_ACTIVE, 0);
        pin_state_add(st, (uint8_t)(b + 1), PIN_FN_I2S_CLK, idx, 1, PIN_CLAIM_ACTIVE, 0);
    }
    if (pdm_hw_is_active() && !claim_active_on(st, PIN_FN_PDM, pdm_get_pin()))
        pin_state_add(st, pdm_get_pin(), PIN_FN_PDM, 0, 0, PIN_CLAIM_ACTIVE, 0);
#if PICO_RP2350
    uint8_t ah = adat_output_hw_pin();
    if (ah != 0xFF && !claim_active_on(st, PIN_FN_ADAT_TX, ah))
        pin_state_add(st, ah, PIN_FN_ADAT_TX, 0, 0, PIN_CLAIM_ACTIVE, 0);
#endif
}

static void capture_pending(PinConfig *p, const PinConfig *live) {
    *p = *live;
    for (uint8_t i = 0; i < NUM_SPDIF_INSTANCES; i++)
        if (output_type_change_mask & (1u << i)) p->output_types[i] = pending_output_types[i];
    if (input_source_change_pending) p->input_source = pending_input_source;
    if (i2s_clock_mode_change_pending) p->clock_mode = pending_i2s_clock_mode;
}

// Every live claim, with the given IO config views standing in for the live one.
static void build_live_state(PinState *st, const CfgViews *cv) {
    pin_state_reset(st);
    add_config_claims(st, cv);
    add_hw_claims(st, cv->v[0]);
    add_dac_mute_claim(st);
    add_ctrl_claims(st);
    control_surfaces_add_pin_claims(st);
}

static PinState s_live_state;

const PinState *pin_state_live(void) {
    PinConfig live, pending;
    pin_config_capture(&live);
    capture_pending(&pending, &live);
    CfgViews cv = { { &live, &pending, NULL } };
    build_live_state(&s_live_state, &cv);
    return &s_live_state;
}

// ---------------------------------------------------------------------------
// The check
// ---------------------------------------------------------------------------

static uint8_t platform_fn(uint8_t fn, uint8_t idx) {
#ifdef DSPI_PLATFORM_PINS
    switch (fn) {
        case PIN_FN_OUTPUT:    return DSPI_PIN_FN_OUTPUT;
        case PIN_FN_PDM:       return DSPI_PIN_FN_PDM;
        case PIN_FN_I2S_CLK:   return idx ? DSPI_PIN_FN_I2S_CLK_SLAVE : DSPI_PIN_FN_I2S_CLK;
        case PIN_FN_MCK:       return DSPI_PIN_FN_MCK;
        case PIN_FN_ADAT_TX:   return DSPI_PIN_FN_ADAT_OUT;
        case PIN_FN_ADAT_RX:   return DSPI_PIN_FN_ADAT_IN;
        case PIN_FN_SPDIF_RX:  return DSPI_PIN_FN_SPDIF_RX;
        case PIN_FN_I2S_RX:    return DSPI_PIN_FN_I2S_RX;
        case PIN_FN_DAC_MUTE:  return DSPI_PIN_FN_DAC_MUTE;
        case PIN_FN_UART_CTRL: return DSPI_PIN_FN_UART_CTRL;
        case PIN_FN_I2C_CTRL:  return DSPI_PIN_FN_I2C_CTRL;
        default:               return DSPI_PIN_FN_ANY;
    }
#else
    (void)fn; (void)idx;
    return 0;
#endif
}

static bool single_valid(uint8_t fn, uint8_t idx, uint8_t pin) {
#ifdef DSPI_WIRECAP
    // Bench rig: the board-internal GPIO 24/25 may carry the I2S clocks (wirecap_test_rig.md).
    if (fn == PIN_FN_I2S_CLK && (pin == 24 || pin == 25)) return true;
#endif
    if (pin == 0xFF || !is_valid_gpio_pin(pin)) return false;
#ifdef DSPI_PLATFORM_PINS
    if (!dspi_platform_pin_valid(platform_fn(fn, idx), pin)) return false;
#else
    (void)platform_fn(fn, idx);
#endif
    if (fn == PIN_FN_MCK && GPIO_TO_GPOUT_CLOCK_HANDLE(pin, clk_sys) == clk_sys) return false;
    return true;
}

// UART: TX at pin%4 == 0, RX at pin%4 == 1, one instance. I2C and the CS display:
// SDA even, SCL odd, one instance. A platform build supplies both rules.
uint8_t pin_pair_group(uint8_t fn, uint8_t pin) {
    if (fn == PIN_FN_UART_CTRL) return (uint8_t)((((pin >> 2) + 1) >> 1) & 1);
    return (uint8_t)((pin >> 1) & 1);
}

bool pin_role_valid(uint8_t fn, uint8_t role, uint8_t pin) {
    if (!single_valid(fn, 0, pin)) return false;
#ifdef DSPI_PLATFORM_PINS
    (void)role;
    return true;
#else
    if (fn == PIN_FN_UART_CTRL) return (pin & 3u) == role;
    return (pin & 1u) == role;
#endif
}

static bool pair_valid(uint8_t fn, uint8_t a, uint8_t b) {
    if (!pin_role_valid(fn, 0, a) || !pin_role_valid(fn, 1, b)) return false;
#ifdef DSPI_PLATFORM_PINS
    uint8_t pfn = (fn == PIN_FN_UART_CTRL) ? DSPI_PIN_FN_UART_CTRL :
                  (fn == PIN_FN_I2C_CTRL)  ? DSPI_PIN_FN_I2C_CTRL : DSPI_PIN_FN_CS_DISPLAY;
    return dspi_platform_pin_pair_valid(pfn, a, b);
#else
    return pin_pair_group(fn, a) == pin_pair_group(fn, b);
#endif
}

static bool request_valid(const PinRequest *rq) {
    switch (rq->fn) {
        case PIN_FN_I2S_CLK:
            return rq->pin[0] < 0xFE && single_valid(rq->fn, rq->idx, rq->pin[0]) &&
                   single_valid(rq->fn, rq->idx, (uint8_t)(rq->pin[0] + 1));
        case PIN_FN_UART_CTRL:
        case PIN_FN_I2C_CTRL:
            return pair_valid(rq->fn, rq->pin[0], rq->pin[1]);
        default:
            return single_valid(rq->fn, rq->idx, rq->pin[0]);
    }
}

// The one sanctioned overlap between functions: the ADAT receiver may listen on the
// ADAT transmitter's pin (loopback self-test); a platform build may also put it on
// S/PDIF input 1's pin, its one receiver input.
static bool may_share(uint8_t fn, uint8_t idx, const PinClaim *c) {
    if ((fn == PIN_FN_ADAT_RX && c->fn == PIN_FN_ADAT_TX) ||
        (fn == PIN_FN_ADAT_TX && c->fn == PIN_FN_ADAT_RX)) return true;
#ifdef DSPI_PLATFORM_PINS
    if ((fn == PIN_FN_ADAT_RX && c->fn == PIN_FN_SPDIF_RX && c->idx == 0) ||
        (fn == PIN_FN_SPDIF_RX && idx == 0 && c->fn == PIN_FN_ADAT_RX)) return true;
#else
    (void)idx;
#endif
    return false;
}

// A function never conflicts with itself. The two I2S clock pairs are one function:
// only one is ever driven, and a pair swap releases the old pair first. Control
// Surfaces resolves conflicts inside its own family (button sharing, PWM slices).
static bool same_family(uint8_t fn, uint8_t idx, const PinClaim *c) {
    if (c->fn != fn) return false;
    return fn == PIN_FN_I2S_CLK || fn == PIN_FN_CS || c->idx == idx;
}

static int instance_of_live(const PinState *st, uint8_t fn, uint8_t cs_type) {
    for (uint8_t i = 0; i < st->count; i++) {
        const PinClaim *c = &st->claims[i];
        if (pin_claim_state(c) != PIN_CLAIM_ACTIVE || c->fn != fn || pin_claim_role(c) != 0) continue;
        if (fn == PIN_FN_CS && pin_claim_cs_type(c) != cs_type) continue;
        return pin_pair_group(PIN_FN_I2C_CTRL, c->gpio);
    }
    return -1;
}

bool pin_conflicts(const PinState *st, uint8_t fn, uint8_t idx, uint8_t pin) {
    if (st->overflow) return true;
    for (uint8_t i = 0; i < st->count; i++) {
        const PinClaim *c = &st->claims[i];
        if (c->gpio != pin || pin_claim_state(c) != PIN_CLAIM_ACTIVE) continue;
        if (same_family(fn, idx, c) || may_share(fn, idx, c)) continue;
        return true;
    }
    return false;
}

// The I2C target and the CS display are both I2C blocks: one instance each, even on
// different pins, or one resets the other out from under it.
int pin_i2c_instance_taken(const PinState *st, uint8_t fn) {
    return (fn == PIN_FN_I2C_CTRL) ? instance_of_live(st, PIN_FN_CS, CS_TYPE_DISPLAY)
                                   : instance_of_live(st, PIN_FN_I2C_CTRL, 0);
}

uint8_t pin_check(const PinState *st, const PinRequest *rq, PinCheckMode mode) {
    PinRequest r = *rq;
    if (r.fn == PIN_FN_I2S_CLK) r.pin[1] = (uint8_t)(r.pin[0] + 1);
    if (!request_valid(&r)) return PIN_CONFIG_INVALID_PIN;
    if (mode == PIN_CHECK_STORE) return PIN_CONFIG_SUCCESS;
    for (int k = 0; k < 2; k++) {
        if (r.pin[k] == 0xFF) continue;
        if (pin_conflicts(st, r.fn, r.idx, r.pin[k])) return PIN_CONFIG_PIN_IN_USE;
    }
    if (r.fn == PIN_FN_I2C_CTRL &&
        pin_i2c_instance_taken(st, r.fn) == (int)pin_pair_group(r.fn, r.pin[0]))
        return PIN_CONFIG_PIN_IN_USE;
    return PIN_CONFIG_SUCCESS;
}

// ---------------------------------------------------------------------------
// Live function status and SET helpers
// ---------------------------------------------------------------------------

static bool pdm_running(void) {
    return pdm_enabled || core1_mode == CORE1_MODE_PDM || pdm_hw_is_active();
}

// The pair the I2S outputs run on now (live or pending type), or 0xFF.
static uint8_t outputs_clock_pair(void) {
    bool any = false;
    for (uint8_t i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        uint8_t t = (output_type_change_mask & (1u << i)) ? pending_output_types[i] : output_types[i];
        if (t == OUTPUT_TYPE_I2S || output_types[i] == OUTPUT_TYPE_I2S) any = true;
    }
    if (!any) return 0xFF;
    uint8_t mode = i2s_clock_mode_change_pending ? pending_i2s_clock_mode : i2s_clock_mode;
    return (i2s_clock_pin_mode == I2S_CLOCK_PIN_MODE_SPLIT && mode == I2S_CLOCK_MODE_SLAVE) ? 1 : 0;
}

uint8_t pin_fn_status(uint8_t fn, uint8_t idx) {
    switch (fn) {
        case PIN_FN_PDM:     return pdm_running() ? PIN_CONFIG_OUTPUT_ACTIVE : PIN_CONFIG_SUCCESS;
        case PIN_FN_MCK:     return i2s_mck_enabled ? PIN_CONFIG_OUTPUT_ACTIVE : PIN_CONFIG_SUCCESS;
        case PIN_FN_I2S_CLK: return outputs_clock_pair() == idx ? PIN_CONFIG_OUTPUT_ACTIVE
                                                                 : PIN_CONFIG_SUCCESS;
        default:             return PIN_CONFIG_SUCCESS;
    }
}

uint8_t pin_check_set(uint8_t fn, uint8_t idx, uint8_t pin0, uint8_t pin1) {
    uint8_t s = pin_fn_status(fn, idx);
    if (s != PIN_CONFIG_SUCCESS) return s;
    const PinState *st = pin_state_live();
    PinRequest rq = { fn, idx, { pin0, pin1 } };
    return pin_check(st, &rq, pin_fn_in_use(st, fn, idx) ? PIN_CHECK_CLAIM : PIN_CHECK_STORE);
}

// ---------------------------------------------------------------------------
// Start checks
// ---------------------------------------------------------------------------

// One past the highest index any function a PinConfig carries can have.
#define PIN_FN_MAX_INDEX ((SPDIF_RX_NUM_INPUTS > I2S_RX_MAX_PAIRS) ? SPDIF_RX_NUM_INPUTS : I2S_RX_MAX_PAIRS)
_Static_assert(PIN_FN_MAX_INDEX >= NUM_SPDIF_INSTANCES && PIN_FN_MAX_INDEX >= 2,
               "start checks walk every index of every function");

// The configured pins of (fn, idx) in a PinConfig.
static PinRequest cfg_request(const PinConfig *c, uint8_t fn, uint8_t idx) {
    PinRequest r = { fn, idx, { 0xFF, 0xFF } };
    switch (fn) {
        case PIN_FN_OUTPUT:   r.pin[0] = c->output_pins[idx]; break;
        case PIN_FN_PDM:      r.pin[0] = c->output_pins[NUM_PIN_OUTPUTS - 1]; break;
        case PIN_FN_I2S_CLK:  r.pin[0] = c->bck_pin[idx]; break;
        case PIN_FN_MCK:      r.pin[0] = c->mck_pin; break;
        case PIN_FN_ADAT_TX:  r.pin[0] = c->adat_tx_pin; break;
        case PIN_FN_ADAT_RX:  r.pin[0] = c->adat_rx_pin; break;
        case PIN_FN_SPDIF_RX: r.pin[0] = c->spdif_rx_pin[idx]; break;
        case PIN_FN_I2S_RX:   r.pin[0] = c->i2s_rx_pin[idx]; break;
        case PIN_FN_DAC_MUTE: r.pin[0] = c->dac_mute_pin; break;
        default: break;
    }
    return r;
}

// Every function the change brings into use must claim its pins against the state
// the change would produce. Functions already in use, or queued to be, are not re-judged.
static uint8_t check_started(const PinConfig *before, const PinConfig *pending,
                             const PinConfig *after) {
    static PinState st;
    CfgViews cv = { { before, pending, after } };
    build_live_state(&st, &cv);
    static const uint8_t fns[] = { PIN_FN_PDM, PIN_FN_I2S_CLK, PIN_FN_MCK, PIN_FN_ADAT_TX,
                                   PIN_FN_ADAT_RX, PIN_FN_SPDIF_RX, PIN_FN_I2S_RX };
    for (unsigned f = 0; f < sizeof(fns); f++) {
        for (uint8_t idx = 0; idx < PIN_FN_MAX_INDEX; idx++) {
            if (!cfg_in_use(after, fns[f], idx)) continue;
            if (cfg_in_use(before, fns[f], idx) || cfg_in_use(pending, fns[f], idx)) continue;
            PinRequest rq = cfg_request(after, fns[f], idx);
            uint8_t s = pin_check(&st, &rq, PIN_CHECK_CLAIM);
            if (s != PIN_CONFIG_SUCCESS) return s;
        }
    }
    return PIN_CONFIG_SUCCESS;
}

// The edit lands on top of whatever switches are already queued.
static uint8_t start_with(void (*edit)(PinConfig *, uint8_t, uint8_t), uint8_t a, uint8_t b) {
    PinConfig before, pending, after;
    pin_config_capture(&before);
    capture_pending(&pending, &before);
    after = pending;
    edit(&after, a, b);
    return check_started(&before, &pending, &after);
}

static void edit_enable(PinConfig *c, uint8_t fn, uint8_t idx) {
    switch (fn) {
        case PIN_FN_PDM:      c->pdm_enabled = 1; break;
        case PIN_FN_MCK:      c->mck_enabled = 1; break;
        case PIN_FN_ADAT_TX:  c->adat_tx_enabled = 1; break;
        case PIN_FN_ADAT_RX:  c->adat_rx_enabled = 1; break;
        default: break;
    }
}
static void edit_type(PinConfig *c, uint8_t slot, uint8_t type) { c->output_types[slot] = type; }
static void edit_spdif_rx(PinConfig *c, uint8_t en, uint8_t count) {
    c->spdif_rx_enabled = en;
    c->spdif_rx_count = count;
}
static void edit_i2s_rx(PinConfig *c, uint8_t en, uint8_t pairs) {
    c->i2s_rx_enabled = en;
    c->i2s_rx_pairs = pairs;
}
static void edit_clock(PinConfig *c, uint8_t pin_mode, uint8_t mode) {
    c->clock_pin_mode = pin_mode;
    c->clock_mode = mode;
}

uint8_t pin_start_check(uint8_t fn, uint8_t idx) {
    // A function with a pin that can never work is refused as invalid, not in use.
    PinConfig c;
    pin_config_capture(&c);
    PinRequest rq = cfg_request(&c, fn, idx);
    if (fn == PIN_FN_I2S_CLK) rq.pin[1] = (uint8_t)(rq.pin[0] + 1);
    if (!request_valid(&rq)) return PIN_CONFIG_INVALID_PIN;
    return start_with(edit_enable, fn, idx);
}

uint8_t pin_start_check_output_type(uint8_t slot, uint8_t type) {
    return start_with(edit_type, slot, type);
}
uint8_t pin_start_check_spdif_rx(uint8_t enabled, uint8_t count) {
    return start_with(edit_spdif_rx, enabled, count);
}
uint8_t pin_start_check_i2s_rx(uint8_t enabled, uint8_t pairs) {
    return start_with(edit_i2s_rx, enabled, pairs);
}
uint8_t pin_start_check_pdm(void) { return pin_start_check(PIN_FN_PDM, 0); }

uint8_t pin_start_check_clock(uint8_t clock_pin_mode, uint8_t clock_mode) {
    return start_with(edit_clock, clock_pin_mode, clock_mode);
}

// ---------------------------------------------------------------------------
// Restore validator
// ---------------------------------------------------------------------------

typedef struct {
    PinConfig         *cfg;
    const PinConfig   *base;
    uint8_t            flags;
    PinState           st;          // claims accepted so far, by priority
    PinValidateResult *res;
    bool               abandon;
} Vctx;

static void set_pin(PinConfig *c, uint8_t fn, uint8_t idx, uint8_t pin) {
    switch (fn) {
        case PIN_FN_OUTPUT:   c->output_pins[idx] = pin; break;
        case PIN_FN_PDM:      c->output_pins[NUM_PIN_OUTPUTS - 1] = pin; break;
        case PIN_FN_I2S_CLK:  c->bck_pin[idx] = pin; break;
        case PIN_FN_MCK:      c->mck_pin = pin; break;
        case PIN_FN_ADAT_TX:  c->adat_tx_pin = pin; break;
        case PIN_FN_ADAT_RX:  c->adat_rx_pin = pin; break;
        case PIN_FN_SPDIF_RX: c->spdif_rx_pin[idx] = pin; break;
        case PIN_FN_I2S_RX:   c->i2s_rx_pin[idx] = pin; break;
        case PIN_FN_DAC_MUTE: c->dac_mute_pin = pin; break;
        default: break;
    }
}

static uint8_t default_pin(uint8_t fn, uint8_t idx) {
    switch (fn) {
        case PIN_FN_OUTPUT:   return default_output_pins[idx];
        case PIN_FN_PDM:      return default_output_pins[NUM_PIN_OUTPUTS - 1];
        case PIN_FN_I2S_CLK:  return idx ? PICO_I2S_BCK_PIN_SLAVE : PICO_I2S_BCK_PIN;
        case PIN_FN_MCK:      return PICO_I2S_MCK_PIN;
#if PICO_RP2350
        case PIN_FN_ADAT_TX:  return PICO_ADAT_PIN;
#endif
        case PIN_FN_SPDIF_RX: return spdif_rx_pin_default_for_index(idx);
        case PIN_FN_I2S_RX:   return (uint8_t)(PICO_I2S_RX_PIN_DEFAULT + idx);
        default:              return 0xFF;   // ADAT RX and the DAC mute have no default pin
    }
}

static void force_off(PinConfig *c, uint8_t fn, uint8_t idx) {
    switch (fn) {
        case PIN_FN_PDM:      c->pdm_enabled = 0; break;
        case PIN_FN_MCK:      c->mck_enabled = 0; break;
        case PIN_FN_ADAT_TX:  c->adat_tx_enabled = 0; break;
        case PIN_FN_ADAT_RX:  c->adat_rx_enabled = 0; break;
        case PIN_FN_SPDIF_RX:   // the count is cut below idx; input 1 switches the block off
            if (idx == 0) c->spdif_rx_enabled = 0;
            else if (c->spdif_rx_count > idx) c->spdif_rx_count = idx;
            break;
        case PIN_FN_I2S_RX:   c->i2s_rx_enabled = 0; break;
        case PIN_FN_DAC_MUTE: c->dac_mute_enabled = 0; break;
        default: break;
    }
}

static void accept(Vctx *v, uint8_t fn, uint8_t idx) {
    PinRequest r = cfg_request(v->cfg, fn, idx);
    if (fn == PIN_FN_I2S_CLK) r.pin[1] = (uint8_t)(r.pin[0] + 1);
    for (int k = 0; k < 2; k++)
        if (r.pin[k] != 0xFF) pin_state_add(&v->st, r.pin[k], fn, idx, (uint8_t)k, PIN_CLAIM_ACTIVE, 0);
}

static uint8_t try_claim(Vctx *v, uint8_t fn, uint8_t idx) {
    PinRequest r = cfg_request(v->cfg, fn, idx);
    return pin_check(&v->st, &r, PIN_CHECK_CLAIM);
}

static void note(Vctx *v, uint8_t fn) { v->res->substituted |= (uint16_t)(1u << fn); }

// A stored pin of a function not in use is judged on validity alone.
static void store_only(Vctx *v, uint8_t fn, uint8_t idx) {
    PinRequest r = cfg_request(v->cfg, fn, idx);
    if (r.pin[0] == 0xFF && (fn == PIN_FN_ADAT_RX || fn == PIN_FN_DAC_MUTE)) return;   // unset
    if (pin_check(&v->st, &r, PIN_CHECK_STORE) == PIN_CONFIG_SUCCESS) return;
    set_pin(v->cfg, fn, idx, default_pin(fn, idx));
    note(v, fn);
}

// A function with no off state: its own pin, then the default, then the base pin.
// If none can claim, the whole pin section falls back to the base.
static bool claim_no_off(Vctx *v, uint8_t fn, uint8_t idx) {
    if (try_claim(v, fn, idx) == PIN_CONFIG_SUCCESS) { accept(v, fn, idx); return true; }
    if (!(v->flags & PIN_VALIDATE_FIXED_PINS)) {
        PinRequest base = cfg_request(v->base, fn, idx);
        uint8_t cands[2] = { default_pin(fn, idx), base.pin[0] };
        for (int k = 0; k < 2; k++) {
            if (cands[k] == 0xFF) continue;
            set_pin(v->cfg, fn, idx, cands[k]);
            if (try_claim(v, fn, idx) == PIN_CONFIG_SUCCESS) {
                note(v, fn);
                accept(v, fn, idx);
                return true;
            }
        }
    }
    v->abandon = true;
    return false;
}

// A function with an enable flag keeps its chosen pin as a stored preference and is
// switched off; moving it would drive a pin nobody chose. A pin that can never work is
// replaced by the default, which keeps the function on if it claims.
static void claim_or_off(Vctx *v, uint8_t fn, uint8_t idx) {
    uint8_t s = try_claim(v, fn, idx);
    if (s == PIN_CONFIG_SUCCESS) { accept(v, fn, idx); return; }
    note(v, fn);
    if (s == PIN_CONFIG_INVALID_PIN && default_pin(fn, idx) != 0xFF &&
        !(v->flags & PIN_VALIDATE_FIXED_PINS)) {
        set_pin(v->cfg, fn, idx, default_pin(fn, idx));
        if (try_claim(v, fn, idx) == PIN_CONFIG_SUCCESS) { accept(v, fn, idx); return; }
    }
    force_off(v->cfg, fn, idx);
}

static void unit(Vctx *v, uint8_t fn, uint8_t idx, bool can_switch_off) {
    if (v->abandon) return;
    if (!cfg_in_use(v->cfg, fn, idx)) { store_only(v, fn, idx); return; }
    if (can_switch_off) claim_or_off(v, fn, idx);
    else                claim_no_off(v, fn, idx);
}

// The input the device listens to now, or at boot the one it is about to start, cannot
// be switched off by a restore. For S/PDIF that covers every input up to it in the count.
static bool input_kept(const Vctx *v, uint8_t fn, uint8_t idx) {
    uint8_t src = (v->flags & PIN_VALIDATE_BOOT) ? v->cfg->input_source : v->base->input_source;
    if (fn == PIN_FN_ADAT_RX) return src == INPUT_SOURCE_ADAT;
    if (fn == PIN_FN_SPDIF_RX) return input_source_is_spdif(src) && idx <= spdif_index_for_source(src);
    if (fn == PIN_FN_I2S_RX) return src == INPUT_SOURCE_I2S;
    return false;
}

// Claims the clock pair and the counted receive pins as one set; on any failure the
// state is rolled back to `mark` and false is returned.
static bool claim_i2s_set(Vctx *v, uint8_t pair, bool with_clock) {
    uint8_t mark = v->st.count;
    bool ok = !with_clock || try_claim(v, PIN_FN_I2S_CLK, pair) == PIN_CONFIG_SUCCESS;
    if (ok && with_clock) accept(v, PIN_FN_I2S_CLK, pair);
    for (uint8_t p = 0; p < v->cfg->i2s_rx_pairs && ok; p++) {
        if (try_claim(v, PIN_FN_I2S_RX, p) == PIN_CONFIG_SUCCESS) accept(v, PIN_FN_I2S_RX, p);
        else ok = false;
    }
    if (!ok) v->st.count = mark;
    return ok;
}

// The I2S input block is one unit: its clock pair (unless the outputs already need it)
// and its counted receive pins. A unit that cannot claim switches the block off. When the
// device is listening to I2S each pin falls back like a no-off function instead.
static void i2s_units(Vctx *v) {
    bool outputs_need = false;
    for (uint8_t i = 0; i < NUM_SPDIF_INSTANCES; i++)
        if (v->cfg->output_types[i] == OUTPUT_TYPE_I2S) outputs_need = true;
    uint8_t pair = cfg_clock_pair(v->cfg);
    bool rx_on = v->cfg->i2s_rx_enabled && !v->abandon;
    bool taken = false;   // the in-use pair and counted receive pins are already accepted

    if (rx_on && !outputs_need) {
        taken = claim_i2s_set(v, pair, true);
    } else if (rx_on) {
        unit(v, PIN_FN_I2S_CLK, pair, false);
        taken = !v->abandon && claim_i2s_set(v, pair, false);
    }
    if (rx_on && !taken && !v->abandon) {
        if (input_kept(v, PIN_FN_I2S_RX, 0)) {
            // A kept input has no off state: each pin tries its default, then the base pin.
            if (!outputs_need) unit(v, PIN_FN_I2S_CLK, pair, false);
            for (uint8_t p = 0; p < v->cfg->i2s_rx_pairs; p++) unit(v, PIN_FN_I2S_RX, p, false);
            taken = !v->abandon;
        } else {
            force_off(v->cfg, PIN_FN_I2S_RX, 0);
            note(v, PIN_FN_I2S_RX);
        }
    }
    for (uint8_t r = 0; r < 2; r++) {
        if (taken && r == pair) continue;
        if (rx_on && outputs_need && r == pair) continue;   // judged above
        unit(v, PIN_FN_I2S_CLK, r, false);
    }
    for (uint8_t p = 0; p < I2S_RX_MAX_PAIRS; p++) {
        if (taken && p < v->cfg->i2s_rx_pairs) continue;
        unit(v, PIN_FN_I2S_RX, p, false);
    }
}

void pin_config_validate(PinConfig *cfg, const PinConfig *base, uint8_t flags,
                         PinValidateResult *res) {
    static Vctx v;
    memset(res, 0, sizeof(*res));
    v.cfg = cfg;
    v.base = base;
    v.flags = flags;
    v.res = res;
    v.abandon = false;
    pin_state_reset(&v.st);

    // Claims the restore does not carry come first. At boot nothing else is up yet.
    if (!(flags & PIN_VALIDATE_BOOT)) {
        add_ctrl_claims(&v.st);
        control_surfaces_add_pin_claims(&v.st);
        if (!cfg->has_dac_mute) add_dac_mute_claim(&v.st);
        // Stored claims never block; drop them so only active ones remain.
        uint8_t n = 0;
        for (uint8_t i = 0; i < v.st.count; i++)
            if (pin_claim_state(&v.st.claims[i]) == PIN_CLAIM_ACTIVE) v.st.claims[n++] = v.st.claims[i];
        v.st.count = n;
    }

    if (cfg->adat_rx_enabled && cfg->adat_rx_pin == 0xFF) {   // an input with no pin cannot run
        cfg->adat_rx_enabled = 0;
        note(&v, PIN_FN_ADAT_RX);
    }

    for (uint8_t i = 0; i < NUM_SPDIF_INSTANCES; i++) unit(&v, PIN_FN_OUTPUT, i, false);
    unit(&v, PIN_FN_PDM, 0, true);
    unit(&v, PIN_FN_SPDIF_RX, 0, !input_kept(&v, PIN_FN_SPDIF_RX, 0));
    unit(&v, PIN_FN_MCK, 0, true);
    // Kept S/PDIF inputs are a prefix and claim before I2S, which can switch off; the rest
    // follow ascending, so a clash cuts the count at the first input that cannot claim.
    uint8_t sp = 1;
    for (; sp < SPDIF_RX_NUM_INPUTS && input_kept(&v, PIN_FN_SPDIF_RX, sp); sp++)
        unit(&v, PIN_FN_SPDIF_RX, sp, false);
    i2s_units(&v);
    for (; sp < SPDIF_RX_NUM_INPUTS; sp++)
        unit(&v, PIN_FN_SPDIF_RX, sp, true);
    if (cfg->has_dac_mute) unit(&v, PIN_FN_DAC_MUTE, 0, true);
#if PICO_RP2350
    unit(&v, PIN_FN_ADAT_TX, 0, true);
    unit(&v, PIN_FN_ADAT_RX, 0, !input_kept(&v, PIN_FN_ADAT_RX, 0));
#endif

    if (v.abandon) {
        // The base is consistent by construction: the live config, or the factory set.
        // PDM's enable is a routing setting, so it only ever drops to the base state.
        PinConfig proposed = *cfg;
        *cfg = *base;
        cfg->has_dac_mute = proposed.has_dac_mute;
        cfg->pdm_enabled = proposed.pdm_enabled && base->pdm_enabled;
        res->substituted = (uint16_t)(((1u << PIN_FN_COUNT) - 1u) & ~1u);
        // The factory pin set serves any source, so a boot keeps the stored one.
        if (flags & PIN_VALIDATE_BOOT) {
            cfg->input_source = proposed.input_source;
            pin_config_admit_source(cfg, cfg->input_source);
            if (!pin_config_source_ok(cfg)) cfg->input_source = INPUT_SOURCE_USB;
        }
        res->source_reverted = (proposed.input_source != cfg->input_source);
        res->pdm_disabled = (proposed.pdm_enabled && !cfg->pdm_enabled);
        return;
    }
    // A proposed source whose block a clash switched off or cut reverts to the base one.
    if (!pin_config_source_ok(cfg)) {
        cfg->input_source = (flags & PIN_VALIDATE_BOOT) ? INPUT_SOURCE_USB : base->input_source;
        res->source_reverted = true;
    }
    res->pdm_disabled = (res->substituted & (1u << PIN_FN_PDM)) && cfg->pdm_enabled == 0;
}

// Globals and deferred flags only: the hardware follows in the caller's pipeline-reset
// bracket, which releases every outgoing pin before it claims any.
void pin_config_install(const PinConfig *c) {
    extern volatile bool spdif_rx_pin_change_pending;
    const uint8_t src = active_input_source;
    const uint8_t old_eff_bck = i2s_effective_bck_pin();

    memcpy(output_pins, c->output_pins, NUM_PIN_OUTPUTS);
    memcpy(output_types, c->output_types, NUM_SPDIF_INSTANCES);

    for (uint8_t i = 0; i < SPDIF_RX_NUM_INPUTS; i++) {
        if (c->spdif_rx_pin[i] == spdif_rx_pin_for_index(i)) continue;
        if (i == 0) spdif_rx_pin = c->spdif_rx_pin[0];
        else        spdif_rx_pin_ext[i - 1] = c->spdif_rx_pin[i];
        if (input_source_is_spdif(src) && spdif_index_for_source(src) == i)
            spdif_rx_pin_change_pending = true;
    }
    spdif_rx_enabled = c->spdif_rx_enabled ? 1 : 0;
    spdif_rx_count = c->spdif_rx_count;
    i2s_input_enabled = c->i2s_rx_enabled ? 1 : 0;

    i2s_mck_pin = c->mck_pin;
    i2s_mck_enabled = c->mck_enabled != 0;
    i2s_bck_pin = c->bck_pin[0];
    i2s_bck_pin_slave = c->bck_pin[1];
    i2s_clock_pin_mode = c->clock_pin_mode;

    bool rx_changed = (uint8_t)(c->i2s_rx_pairs * 2) != i2s_input_channels;
    for (uint8_t p = 0; p < I2S_RX_MAX_PAIRS; p++) {
        if (c->i2s_rx_pin[p] != i2s_rx_pin[p]) rx_changed = true;
        i2s_rx_pin[p] = c->i2s_rx_pin[p];
    }
    i2s_input_channels = (uint8_t)(c->i2s_rx_pairs * 2);
    if ((rx_changed || i2s_effective_bck_pin() != old_eff_bck) && src == INPUT_SOURCE_I2S)
        i2s_input_restart_pending = true;

#if PICO_RP2350
    // Unchanged ADAT config is not rewritten, so a no-op restore never restarts it.
    if (adat_output_config_enabled() != (c->adat_tx_enabled != 0) || adat_output_pin() != c->adat_tx_pin)
        adat_output_set_config(c->adat_tx_enabled != 0, c->adat_tx_pin);
    if (adat_input_pin != c->adat_rx_pin || adat_input_enabled != c->adat_rx_enabled) {
        adat_input_pin = c->adat_rx_pin;
        adat_input_enabled = c->adat_rx_enabled;
        if (src == INPUT_SOURCE_ADAT) adat_input_restart_pending = true;
    }
#endif
}

void pin_restore_apply(PinConfig *cfg, uint8_t flags, PinValidateResult *res) {
    PinConfig base;
    if (flags & PIN_VALIDATE_BOOT) pin_config_defaults(&base);
    else                           pin_config_capture(&base);
    // The caller has already written the restored matrix, so the base PDM state comes
    // from the running device instead.
    if (!(flags & PIN_VALIDATE_BOOT)) base.pdm_enabled = pdm_running() ? 1 : 0;
    base.has_dac_mute = cfg->has_dac_mute;
    pin_config_validate(cfg, &base, flags, res);
    pin_config_install(cfg);
    if (res->pdm_disabled) matrix_mixer.outputs[NUM_OUTPUT_CHANNELS - 1].enabled = 0;
    if (res->source_reverted) input_source_change_pending = false;
    // A queued switch to an input this restore switched off is dropped here, not later.
    if (input_source_change_pending && !input_source_selectable(pending_input_source))
        input_source_change_pending = false;
    pin_restore_note(res);
}

static uint16_t s_last_substituted;

void pin_restore_note(const PinValidateResult *res) { s_last_substituted = res->substituted; }
uint16_t pin_restore_substituted(void) { return s_last_substituted; }

// ---------------------------------------------------------------------------
// Host queries
// ---------------------------------------------------------------------------

uint8_t pin_gpio_count(void) {
#ifdef NUM_BANK0_GPIOS
    return (uint8_t)NUM_BANK0_GPIOS;
#else
    return 30;
#endif
}

uint64_t pin_gpio_mask(void) {
    uint64_t m = 0;
    for (uint8_t g = 0; g < pin_gpio_count() && g < 64; g++)
        if (is_valid_gpio_pin(g)) m |= (uint64_t)1 << g;
    return m;
}


static uint8_t avail_status(uint8_t fn, uint8_t idx, const PinState *st) {
    if (st->overflow) return VC_ERR_BUSY;
    return pin_fn_status(fn, idx) == PIN_CONFIG_SUCCESS ? VC_OK : VC_ERR_STATE;
}

// The pins a function holds now, for the START_OK and UNASSIGNED flags.
static PinRequest live_request(uint8_t fn, uint8_t idx) {
    PinRequest r = { fn, idx, { 0xFF, 0xFF } };
    if (fn == PIN_FN_UART_CTRL) {
        UartCtrlConfig u;
        (void)uart_ctrl_live_config(&u);
        r.pin[0] = u.tx_pin;
        r.pin[1] = u.rx_pin;
    } else if (fn == PIN_FN_I2C_CTRL) {
        I2cCtrlConfig i;
        (void)i2c_ctrl_live_config(&i);
        r.pin[0] = i.sda_pin;
        r.pin[1] = i.scl_pin;
    } else {
        PinConfig c;
        pin_config_capture(&c);
        c.has_dac_mute = 1;
        r = cfg_request(&c, fn, idx);
    }
    return r;
}

void pin_avail(uint8_t fn, uint8_t idx, PinAvail *a) {
    memset(a, 0, sizeof(*a));
    const PinState *st = pin_state_live();
    a->status = avail_status(fn, idx, st);
    if (pin_fn_in_use(st, fn, idx)) a->flags |= PIN_AVAIL_IN_USE;
    if (fn == PIN_FN_I2S_CLK) a->flags |= PIN_AVAIL_PAIR_PLUS1;
    PinRequest cur = live_request(fn, idx);
    if (cur.pin[0] == 0xFF) a->flags |= PIN_AVAIL_UNASSIGNED;
    else if (pin_check(st, &cur, PIN_CHECK_CLAIM) == PIN_CONFIG_SUCCESS) a->flags |= PIN_AVAIL_START_OK;

    uint8_t n = pin_gpio_count();
    if (n > 64) n = 64;
    if (fn == PIN_FN_UART_CTRL || fn == PIN_FN_I2C_CTRL) {
#ifdef DSPI_PLATFORM_PINS
        a->groups = 1;
#else
        a->groups = 2;
#endif
        a->roles = 2;
        int taken = (fn == PIN_FN_I2C_CTRL) ? pin_i2c_instance_taken(st, fn) : -1;
        for (uint8_t g = 0; g < n; g++) {
            for (uint8_t r = 0; r < 2; r++) {
                if (!pin_role_valid(fn, r, g)) continue;
                uint8_t grp = (a->groups == 2) ? pin_pair_group(fn, g) : 0;
                uint64_t bit = (uint64_t)1 << g;
                a->valid[grp][r] |= bit;
                if (!pin_conflicts(st, fn, 0, g) && taken != (int)pin_pair_group(fn, g))
                    a->free[grp][r] |= bit;
            }
        }
        return;
    }
    a->groups = 1;
    a->roles = 1;
    for (uint8_t g = 0; g < n; g++) {
        PinRequest rq = { fn, idx, { g, 0xFF } };
        uint64_t bit = (uint64_t)1 << g;
        if (pin_check(st, &rq, PIN_CHECK_STORE) != PIN_CONFIG_SUCCESS) continue;
        a->valid[0][0] |= bit;
        if (pin_check(st, &rq, PIN_CHECK_CLAIM) == PIN_CONFIG_SUCCESS) a->free[0][0] |= bit;
    }
}

static void put_u64(uint8_t *p, uint64_t v) {
    for (int i = 0; i < 8; i++) p[i] = (uint8_t)(v >> (8 * i));
}

uint16_t pin_avail_serialize(const PinAvail *a, uint32_t generation, uint8_t *out, uint16_t cap,
                             bool cs_layout) {
    uint8_t pairs = cs_layout ? 4 : (uint8_t)(a->groups * a->roles);
    uint16_t len = (uint16_t)(8 + 16u * pairs + (cs_layout ? 8 : 0));
    if (len > cap) return 0;
    memset(out, 0, len);
    out[0] = a->status;
    out[1] = a->flags;
    out[2] = a->groups;
    out[3] = a->roles;
    memcpy(out + 4, &generation, 4);
    uint8_t stride = cs_layout ? 2 : a->roles;
    for (uint8_t g = 0; g < a->groups; g++) {
        for (uint8_t r = 0; r < a->roles; r++) {
            uint8_t *p = out + 8 + 16u * (uint16_t)(g * stride + r);
            put_u64(p, a->valid[g][r]);
            put_u64(p + 8, a->free[g][r]);
        }
    }
    if (cs_layout) put_u64(out + 72, a->shareable);
    return len;
}

// ---------------------------------------------------------------------------
// Generation counter
// ---------------------------------------------------------------------------

static uint32_t s_generation;
static uint32_t s_hash;
static bool     s_hash_valid;
static bool     s_notify_owed;

// Covers everything a reply depends on: the claims and every live function status.
static uint32_t state_hash(const PinState *st) {
    uint32_t h = 2166136261u;                       // FNV-1a
    const uint8_t *p = (const uint8_t *)st->claims;
    for (uint16_t i = 0; i < (uint16_t)st->count * sizeof(PinClaim); i++) h = (h ^ p[i]) * 16777619u;
    h = (h ^ st->count) * 16777619u;
    h = (h ^ (st->overflow ? 1u : 0u)) * 16777619u;
    uint8_t status[4] = { pin_fn_status(PIN_FN_PDM, 0), pin_fn_status(PIN_FN_MCK, 0),
                          pin_fn_status(PIN_FN_I2S_CLK, 0), pin_fn_status(PIN_FN_I2S_CLK, 1) };
    for (int i = 0; i < 4; i++) h = (h ^ status[i]) * 16777619u;
    return h;
}

uint32_t pin_generation_sync(bool push) {
    uint32_t h = state_hash(pin_state_live());
    if (!s_hash_valid) {
        s_hash = h;
        s_hash_valid = true;
    } else if (h != s_hash) {
        s_hash = h;
        s_generation++;
        s_notify_owed = true;
    }
    // A change first seen by a host query is still announced from the main loop.
    if (push && s_notify_owed) {
        s_notify_owed = false;
        notify_push_pin_map_changed(s_generation);
    }
    return s_generation;
}
