/*
 * vc_page_clock.c: CLOCK page. Command set: vendor_command_overhaul_plan.md section 5.12.
 * Clock pin rules (pair distinctness, SPLIT mode): Documentation/Features/clock_pins_spec.md.
 */

#include "vc_core.h"
#include "vc_pins.h"
#include "config.h"
#include "usb_audio.h"
#include "audio_input.h"
#include "i2s_input.h"
#include "bulk_params.h"
#include "notify.h"
#include "pico/audio_i2s_multi.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"

static bool any_i2s_output(void) {
    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++)
        if (output_types[i] == OUTPUT_TYPE_I2S) return true;
    return false;
}

static uint8_t effective_i2s_clock_mode(void) {
    return i2s_clock_mode_change_pending ? pending_i2s_clock_mode : i2s_clock_mode;
}

// ---- I2S clock pins and modes ----

static VcResult clock_bck_pin_get(VcCtx *c) {
    c->out[0] = c->addr ? i2s_bck_pin_slave : i2s_bck_pin;
    return VC_OK;
}

static VcResult clock_bck_pin_set(VcCtx *c) {
    uint8_t role = (uint8_t)c->addr;
    uint8_t new_pin = vc_in_u8(c, 0);
    if (new_pin == PIN_RESET_TO_DEFAULT) new_pin = role ? PICO_I2S_BCK_PIN_SLAVE : PICO_I2S_BCK_PIN;
    if (!PIN_VALID_FOR(role ? DSPI_PIN_FN_I2S_CLK_SLAVE : DSPI_PIN_FN_I2S_CLK, new_pin) ||
        !PIN_VALID_FOR(role ? DSPI_PIN_FN_I2S_CLK_SLAVE : DSPI_PIN_FN_I2S_CLK, new_pin + 1))
        return VC_ERR_PIN_INVALID;
    if (new_pin == (role ? i2s_bck_pin_slave : i2s_bck_pin)) return VC_OK;

    // Adjacent bases overlap through the shared LRCLK. The master pair checks the
    // dormant slave pair only in SPLIT mode; entering SPLIT re-validates.
    uint8_t other = role ? i2s_bck_pin : i2s_bck_pin_slave;
    bool overlaps = (new_pin == other) || (new_pin == (uint8_t)(other + 1)) ||
                    ((uint8_t)(new_pin + 1) == other);
    if (role == 0 && i2s_clock_pin_mode != I2S_CLOCK_PIN_MODE_SPLIT) overlaps = false;
    // I2S output slots run on role 0 except in SPLIT + slave clocking, where they use role 1.
    bool split_slave = (i2s_clock_pin_mode == I2S_CLOCK_PIN_MODE_SPLIT &&
                        effective_i2s_clock_mode() == I2S_CLOCK_MODE_SLAVE);
    if (any_i2s_output() && (role == 1) == split_slave) return VC_ERR_STATE;
    if (overlaps || is_pin_in_use(new_pin, 0xFF) || is_pin_in_use(new_pin + 1, 0xFF))
        return VC_ERR_PIN_IN_USE;

    if (role == 1) {
        i2s_bck_pin_slave = new_pin;
        if (active_input_source == INPUT_SOURCE_I2S && split_slave) i2s_input_restart_pending = true;
        notify_param_write(offsetof(WireBulkParams, i2s_config.bck_pin_slave), 1, &i2s_bck_pin_slave);
    } else {
        i2s_bck_pin = new_pin;
        if (active_input_source == INPUT_SOURCE_I2S && !split_slave) i2s_input_restart_pending = true;
        notify_param_write(offsetof(WireBulkParams, i2s_config.bck_pin), 1, &i2s_bck_pin);
    }
    return VC_OK;
}

static VcResult clock_pin_mode_get(VcCtx *c) {
    c->out[0] = i2s_clock_pin_mode;
    return VC_OK;
}

// Only a live pair swap (slave clocking in force or pending) needs hardware work;
// every other change is a dormant store.
static VcResult clock_pin_mode_set(VcCtx *c) {
    uint8_t new_mode = vc_in_u8(c, 0);
    if (new_mode > I2S_CLOCK_PIN_MODE_SPLIT) return VC_ERR_VALUE;
    if (new_mode == i2s_clock_pin_mode) return VC_OK;

    bool live_swap = (effective_i2s_clock_mode() == I2S_CLOCK_MODE_SLAVE);
    // Entering SPLIT: the dormant slave pair must already be distinct and unclaimed,
    // since a later clock-mode flip adopts it without rechecking.
    bool slave_pair_bad = new_mode == I2S_CLOCK_PIN_MODE_SPLIT &&
        ((i2s_bck_pin_slave == i2s_bck_pin) ||
         (i2s_bck_pin_slave == (uint8_t)(i2s_bck_pin + 1)) ||
         ((uint8_t)(i2s_bck_pin_slave + 1) == i2s_bck_pin) ||
         is_pin_in_use(i2s_bck_pin_slave, 0xFF) ||
         is_pin_in_use((uint8_t)(i2s_bck_pin_slave + 1), 0xFF));
    if (live_swap && any_i2s_output()) return VC_ERR_STATE;
    if (slave_pair_bad) return VC_ERR_PIN_IN_USE;

    i2s_clock_pin_mode = new_mode;
    if (live_swap && active_input_source == INPUT_SOURCE_I2S) i2s_input_restart_pending = true;
    uint8_t wire_mode_p1 = (uint8_t)(i2s_clock_pin_mode + 1);
    notify_param_write(offsetof(WireBulkParams, i2s_config.clock_pin_mode_p1), 1, &wire_mode_p1);
    return VC_OK;
}

// Reports the live mode; a pending change shows once the main loop applies it.
static VcResult clock_i2s_mode_get(VcCtx *c) {
    c->out[0] = i2s_clock_mode;
    return VC_OK;
}

// Applied and notified on the main loop: the switch restarts I2S RX and rebuilds the I2S outputs.
static VcResult clock_i2s_mode_set(VcCtx *c) {
    uint8_t v = vc_in_u8(c, 0);
    if (v > I2S_CLOCK_MODE_SLAVE) return VC_ERR_VALUE;
    if (v == i2s_clock_mode && !i2s_clock_mode_change_pending) return VC_OK;
    pending_i2s_clock_mode = v;
    __dmb();
    i2s_clock_mode_change_pending = true;
    return VC_OK;
}

_Static_assert(sizeof(I2sSlaveStatusPacket) == 16, "CLOCK.I2S_SLAVE_STATUS is 16 bytes");

static VcResult clock_i2s_slave_status_get(VcCtx *c) {
    I2sSlaveStatusPacket st;
    i2s_slave_get_status(&st);
    memcpy(c->out, &st, sizeof(st));
    return VC_OK;
}

// ---- MCK ----

static VcResult clock_mck_enable_get(VcCtx *c) {
    c->out[0] = i2s_mck_enabled ? 1 : 0;
    return VC_OK;
}

static VcResult clock_mck_enable_set(VcCtx *c) {
    bool enable = vc_in_u8(c, 0) != 0;
    if (enable && !i2s_mck_enabled) {
        // MCK claims its pin only while enabled, so another function may hold it now.
        if (is_pin_in_use(i2s_mck_pin, 0xFF)) return VC_ERR_PIN_IN_USE;
        // Load the divider before enabling GPOUT, or the DAC sees a stale clock and relocks.
        audio_i2s_mck_update_frequency(audio_state.freq, i2s_mck_multiplier);
        audio_i2s_mck_set_enabled(true);
        i2s_mck_enabled = true;
    } else if (!enable && i2s_mck_enabled) {
        audio_i2s_mck_set_enabled(false);
        i2s_mck_enabled = false;
    }
    uint8_t v = i2s_mck_enabled ? 1 : 0;
    notify_param_write(offsetof(WireBulkParams, i2s_config.mck_enabled), 1, &v);
    return VC_OK;
}

static VcResult clock_mck_pin_get(VcCtx *c) {
    c->out[0] = i2s_mck_pin;
    return VC_OK;
}

static VcResult clock_mck_pin_set(VcCtx *c) {
    uint8_t new_pin = vc_in_u8(c, 0);
    uint8_t status;
    if (new_pin == PIN_RESET_TO_DEFAULT) new_pin = PICO_I2S_MCK_PIN;
    if (!PIN_VALID_FOR(DSPI_PIN_FN_MCK, new_pin)) {
        status = PIN_CONFIG_INVALID_PIN;
    } else if (GPIO_TO_GPOUT_CLOCK_HANDLE(new_pin, clk_sys) == clk_sys) {
        status = PIN_CONFIG_INVALID_PIN;   // MCK is a CLK_GPOUTn output; clk_sys means no GPOUT here
    } else if (i2s_mck_enabled) {
        status = PIN_CONFIG_OUTPUT_ACTIVE;
    } else if (is_pin_in_use(new_pin, 0xFF)) {
        status = PIN_CONFIG_PIN_IN_USE;
    } else if (new_pin == i2s_mck_pin) {
        status = PIN_CONFIG_SUCCESS;
    } else {
        audio_i2s_mck_change_pin(new_pin);
        i2s_mck_pin = new_pin;
        notify_param_write(offsetof(WireBulkParams, i2s_config.mck_pin), 1, &i2s_mck_pin);
        status = PIN_CONFIG_SUCCESS;
    }
    return vc_from_pin_status(status);
}

static VcResult clock_mck_mult_get(VcCtx *c) {
    c->out[0] = mck_encode(i2s_mck_multiplier);
    return VC_OK;
}

static VcResult clock_mck_mult_set(VcCtx *c) {
    uint8_t raw = vc_in_u8(c, 0);
    if (raw > 1) return VC_ERR_VALUE;
    i2s_mck_multiplier = mck_decode(raw);
    if (i2s_mck_enabled) audio_i2s_mck_update_frequency(audio_state.freq, i2s_mck_multiplier);
    uint8_t wire_mult = mck_encode(i2s_mck_multiplier);
    notify_param_write(offsetof(WireBulkParams, i2s_config.mck_multiplier), 1, &wire_mult);
    return VC_OK;
}

#define VC_PAGE_DEF "pages/clock.def"
#include "vc_page_table.inc"
