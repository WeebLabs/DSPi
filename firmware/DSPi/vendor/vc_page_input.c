/*
 * vc_page_input.c: INPUT page. Command set: vendor_command_overhaul_plan.md section 5.3.
 * Pin and enable SETs keep the 1.1.6 checks; PIN_CONFIG_* refusals become VC_ERR_* results.
 */

#include "vc_core.h"
#include "vc_pins.h"
#include "config.h"
#include "usb_audio.h"
#include "audio_input.h"
#include "audio_pipeline.h"
#include "spdif_input.h"
#include "soft_vcxo.h"
#include "lg_sound_sync.h"
#include "input_source_names.h"
#include "adat_input.h"
#include "adat_output.h"
#include "bulk_params.h"
#include "notify.h"
#include "hardware/sync.h"

extern volatile bool spdif_rx_pin_change_pending;

// ---- Source ----

static VcResult input_source_get(VcCtx *c) {
    c->out[0] = active_input_source;
    return VC_OK;
}

// Notification is emitted at apply time (main.c), so it reports the active source.
static VcResult input_source_set(VcCtx *c) {
    uint8_t src = vc_in_u8(c, 0);
    if (!input_source_valid(src)) return VC_ERR_VALUE;
    if (src == active_input_source) return VC_OK;
#if !PICO_RP2350
    if (src == INPUT_SOURCE_ADAT) return VC_ERR_UNSUPPORTED;
#endif
    if (!input_source_selectable(src)) return VC_ERR_STATE;   // a disabled optional input
    pending_input_source = src;
    __dmb();
    input_source_change_pending = true;
    return VC_OK;
}

static VcResult input_status_get(VcCtx *c) {
    vc_out_u8(c, 0, active_input_source);
    vc_out_u8(c, 1, active_input_channel_count());
    vc_out_u32(c, 4, spdif_input_measured_rate_mhz());
    float p = soft_vcxo_current_ppm() * 100.0f;
    vc_out_u32(c, 8, (uint32_t)(int32_t)(p + (p >= 0.0f ? 0.5f : -0.5f)));
    return VC_OK;
}

static VcResult input_source_name_get_h(VcCtx *c) {
    if (!input_source_name_get((uint8_t)c->addr, (char *)c->out)) return VC_ERR_ADDRESS;
    return VC_OK;
}

static VcResult input_source_name_set_h(VcCtx *c) {
    if (!input_source_name_stage((uint8_t)c->addr, c->in, c->in_len, vc_param_src(c->src)))
        return VC_ERR_ADDRESS;
    return VC_OK;
}

// ---- S/PDIF RX ----

_Static_assert(sizeof(SpdifRxStatusPacket) == 16, "INPUT.SPDIF_RX_STATUS is 16 bytes");
_Static_assert(2 + SPDIF_RX_NUM_INPUTS == 6, "INPUT.SPDIF_RX_CONFIG is 6 bytes");

static VcResult input_spdif_rx_status_get(VcCtx *c) {
    SpdifRxStatusPacket st;
    spdif_input_get_status(&st);
    memcpy(c->out, &st, sizeof(st));
    return VC_OK;
}

static VcResult input_spdif_rx_ch_status_get(VcCtx *c) {
    spdif_input_get_channel_status(c->out);
    return VC_OK;
}

static VcResult input_spdif_rx_pin_get(VcCtx *c) {
    c->out[0] = spdif_rx_pin_for_index((uint8_t)c->addr);
    return VC_OK;
}

// RAM-only like OUTPUT.PIN; the hot swap of the active input runs on the main loop.
static VcResult input_spdif_rx_pin_set(VcCtx *c) {
    uint8_t index = (uint8_t)c->addr;
    uint8_t new_pin = vc_in_u8(c, 0);
    if (new_pin == PIN_RESET_TO_DEFAULT) new_pin = spdif_rx_pin_default_for_index(index);
    if (!PIN_VALID_FOR(DSPI_PIN_FN_SPDIF_RX, new_pin)) return VC_ERR_PIN_INVALID;
    if (new_pin == spdif_rx_pin_for_index(index)) return VC_OK;
    // A disabled optional input stores its pin as a preference; enable re-checks it.
    if (spdif_input_enabled(index) && is_pin_in_use(new_pin, 0xFF)) return VC_ERR_PIN_IN_USE;

    if (index == 0) spdif_rx_pin = new_pin;
    else            spdif_rx_pin_ext[index - 1] = new_pin;
    if (input_source_is_spdif(active_input_source) &&
        spdif_index_for_source(active_input_source) == index)
        spdif_rx_pin_change_pending = true;
    uint16_t off = (index == 0)
        ? (uint16_t)offsetof(WireBulkParams, input_config.spdif_rx_pin)
        : (uint16_t)(offsetof(WireBulkParams, input_config.spdif_rx_pin_ext) + (index - 1));
    notify_param_write(off, 1, &new_pin);
    return VC_OK;
}

static VcResult input_spdif_rx_enable_get(VcCtx *c) {
    if (c->addr == 0) return VC_ERR_ADDRESS;   // input 0 is always enabled
    c->out[0] = spdif_input_enabled((uint8_t)c->addr) ? 1 : 0;
    return VC_OK;
}

static VcResult input_spdif_rx_enable_set(VcCtx *c) {
    uint8_t index = (uint8_t)c->addr;
    uint8_t enable = vc_in_u8(c, 0) ? 1 : 0;
    if (index == 0) return VC_ERR_ADDRESS;
    if (enable == (spdif_input_enabled(index) ? 1 : 0)) return VC_OK;

    if (enable) {
        uint8_t pin = spdif_rx_pin_for_index(index);
        if (!PIN_VALID_FOR(DSPI_PIN_FN_SPDIF_RX, pin)) return VC_ERR_PIN_INVALID;
        if (is_pin_in_use(pin, 0xFF)) return VC_ERR_PIN_IN_USE;
        spdif_rx_enabled_ext |= (uint8_t)(1u << (index - 1));
    } else {
        // The host must switch the source away first.
        bool is_active = input_source_is_spdif(active_input_source) &&
                         spdif_index_for_source(active_input_source) == index;
        bool is_pending = input_source_change_pending &&
                          pending_input_source == spdif_source_for_index(index);
        if (is_active || is_pending) return VC_ERR_STATE;
        spdif_rx_enabled_ext &= (uint8_t)~(1u << (index - 1));
    }
    uint8_t enc = (uint8_t)(spdif_rx_enabled_ext + 1);   // wire field is the mask plus one
    notify_param_write(offsetof(WireBulkParams, input_config.spdif_rx_enabled_ext_p1), 1, &enc);
    return VC_OK;
}

static VcResult input_spdif_rx_config_get(VcCtx *c) {
    c->out[0] = SPDIF_RX_NUM_INPUTS;
    c->out[1] = (uint8_t)((spdif_rx_enabled_ext << 1) | 1);
    for (uint8_t i = 0; i < SPDIF_RX_NUM_INPUTS; i++)
        c->out[2 + i] = spdif_rx_pin_for_index(i);
    return VC_OK;
}

// ---- I2S RX ----

static VcResult input_i2s_rx_pin_get(VcCtx *c) {
    c->out[0] = i2s_rx_pin[c->addr];
    return VC_OK;
}

static VcResult input_i2s_rx_pin_set(VcCtx *c) {
    uint8_t pair = (uint8_t)c->addr;
    uint8_t new_pin = vc_in_u8(c, 0);
    // Defaults are the contiguous block from pair 0's default (audio_input.c).
    if (new_pin == PIN_RESET_TO_DEFAULT) new_pin = (uint8_t)(PICO_I2S_RX_PIN_DEFAULT + pair);
    if (new_pin == i2s_rx_pin[pair]) return VC_OK;
    uint8_t st = check_i2s_rx_pin(new_pin, pair, (uint8_t)(i2s_input_channels / 2));
    if (st != PIN_CONFIG_SUCCESS) return vc_from_pin_status(st);

    i2s_rx_pin[pair] = new_pin;
    if (active_input_source == INPUT_SOURCE_I2S) {
        // Only stereo pair 0 may hot-swap; any other change restarts every pair in sync.
        if (pair == 0 && i2s_input_channels <= 2)
            i2s_rx_pin_change_pending = true;
        else
            i2s_input_restart_pending = true;
    }
    uint16_t off = (pair == 0)
        ? (uint16_t)offsetof(WireBulkParams, input_config.i2s_rx_pin)
        : (uint16_t)(offsetof(WireBulkParams, input_config.i2s_rx_pin_ext) + (pair - 1));
    notify_param_write(off, 1, &i2s_rx_pin[pair]);
    return VC_OK;
}

static VcResult input_i2s_rx_channels_get(VcCtx *c) {
    c->out[0] = i2s_input_channels;
    return VC_OK;
}

static VcResult input_i2s_rx_channels_set(VcCtx *c) {
    uint8_t count = vc_in_u8(c, 0);
    if (count != 2 && count != 4 && count != 6 && count != 8) return VC_ERR_VALUE;
    if (count / 2 > I2S_RX_MAX_PAIRS) return VC_ERR_UNSUPPORTED;

    // Each newly active pair's parked pin is checked against the NEW pair count.
    uint8_t old_pairs = (uint8_t)(i2s_input_channels / 2);
    uint8_t new_pairs = (uint8_t)(count / 2);
    for (uint8_t p = old_pairs; p < new_pairs; p++) {
        uint8_t st = check_i2s_rx_pin(i2s_rx_pin[p], p, new_pairs);
        if (st != PIN_CONFIG_SUCCESS) return vc_from_pin_status(st);
    }
    if (count != i2s_input_channels) {
        i2s_input_channels = count;
        if (active_input_source == INPUT_SOURCE_I2S) {
            i2s_input_restart_pending = true;
            notify_push_input_format(active_input_channel_count());
        }
        notify_param_write(offsetof(WireBulkParams, input_config.i2s_input_channels),
                           1, &i2s_input_channels);
    }
    return VC_OK;
}

static VcResult input_i2s_rx_rate_get(VcCtx *c) {
    vc_out_u32(c, 0, i2s_input_rate);
    return VC_OK;
}

// Stored always; a live rate change is armed only while the device is the rate
// authority (I2S master or ADAT master input). Slave modes take the external rate.
static VcResult input_i2s_rx_rate_set(VcCtx *c) {
    uint32_t rate = vc_in_u32(c, 0);
    if (rate != 44100 && rate != 48000 && rate != 96000) return VC_ERR_VALUE;
    i2s_input_rate = rate;
    uint8_t enc = i2s_rate_encode(rate);
    notify_param_write(offsetof(WireBulkParams, input_config.i2s_input_rate), 1, &enc);
    bool rate_authority_active =
        (active_input_source == INPUT_SOURCE_I2S && !i2s_slave_mode_active()) ||
        (active_input_source == INPUT_SOURCE_ADAT && adat_clock_mode == ADAT_CLOCK_MODE_MASTER);
    if (rate_authority_active && rate != audio_state.freq) {
        pending_rate = rate;
        __dmb();
        rate_change_pending = true;
    }
    return VC_OK;
}

// ---- ADAT RX ----

#if PICO_RP2350
_Static_assert(sizeof(AdatInputStatusPacket) == 20, "INPUT.ADAT_RX_STATUS is 20 bytes");

// The RX only listens, so sharing the ADAT TX pin (loopback self-test) is allowed.
static bool adat_rx_pin_taken(uint8_t pin) {
    return pin != adat_output_pin() && is_pin_in_use(pin, 0xFF)
#ifdef DSPI_PLATFORM_PINS
           && pin != spdif_rx_pin     // shared receiver pin (adat_input_pin_acceptable())
#endif
           ;
}

static VcResult input_adat_rx_enable_get(VcCtx *c) {
    c->out[0] = adat_input_enabled;
    return VC_OK;
}

static VcResult input_adat_rx_enable_set(VcCtx *c) {
    uint8_t value = vc_in_u8(c, 0);
    if (value > 1) return VC_ERR_VALUE;
    if (value == (adat_input_enabled ? 1 : 0)) {
        // Unchanged; the notify below still runs.
    } else if (value) {
        uint8_t pin = adat_input_pin;
        if (pin == 0xFF || !PIN_VALID_FOR(DSPI_PIN_FN_ADAT_IN, pin)) return VC_ERR_PIN_INVALID;
        if (adat_rx_pin_taken(pin)) return VC_ERR_PIN_IN_USE;
        adat_input_enabled = 1;
    } else {
        bool is_active  = (active_input_source == INPUT_SOURCE_ADAT);
        bool is_pending = input_source_change_pending &&
                          pending_input_source == INPUT_SOURCE_ADAT;
        if (is_active || is_pending) return VC_ERR_STATE;
        adat_input_enabled = 0;
    }
    uint8_t enc = (uint8_t)(adat_input_enabled + 1);   // wire field is the enable plus one
    notify_param_write(offsetof(WireBulkParams, input_config.adat_input_enabled_p1), 1, &enc);
    return VC_OK;
}

static VcResult input_adat_rx_pin_get(VcCtx *c) {
    c->out[0] = adat_input_pin;
    return VC_OK;
}

static VcResult input_adat_rx_pin_set(VcCtx *c) {
    uint8_t pin = vc_in_u8(c, 0);
    if (pin == PIN_RESET_TO_DEFAULT) {
        // No free default GPIO exists, so reset means unset, which an enabled input cannot be.
        if (adat_input_enabled) return VC_ERR_STATE;
        adat_input_pin = 0xFF;
    } else if (!PIN_VALID_FOR(DSPI_PIN_FN_ADAT_IN, pin)) {
        return VC_ERR_PIN_INVALID;
    } else if (pin == adat_input_pin) {
        // Unchanged; the notify below still runs.
    } else if (adat_rx_pin_taken(pin)) {
        return VC_ERR_PIN_IN_USE;
    } else {
        adat_input_pin = pin;
        if (active_input_source == INPUT_SOURCE_ADAT) adat_input_restart_pending = true;
    }
    uint8_t wire_pin = (adat_input_pin == 0xFF) ? 0 : adat_input_pin;   // wire 0 = unset
    notify_param_write(offsetof(WireBulkParams, input_config.adat_input_pin), 1, &wire_pin);
    return VC_OK;
}

// Reports the live mode; a pending change shows once the main loop applies it.
static VcResult input_adat_rx_clock_mode_get(VcCtx *c) {
    c->out[0] = adat_clock_mode;
    return VC_OK;
}

static VcResult input_adat_rx_clock_mode_set(VcCtx *c) {
    uint8_t v = vc_in_u8(c, 0);
    if (v > 1) return VC_ERR_VALUE;
    if (v == adat_clock_mode && !adat_clock_mode_change_pending) return VC_OK;
    pending_adat_clock_mode = v;
    __dmb();
    adat_clock_mode_change_pending = true;
    return VC_OK;
}

static VcResult input_adat_rx_status_get(VcCtx *c) {
    AdatInputStatusPacket s;
    adat_input_get_status(&s);
    memcpy(c->out, &s, sizeof(s));
    return VC_OK;
}
#endif

// ---- LG Sound Sync ----

_Static_assert(sizeof(LgSoundSyncStatus) == 16, "INPUT.LG_SOUND_SYNC_STATUS is 16 bytes");

static VcResult input_lg_enable_get(VcCtx *c) {
    c->out[0] = lg_sound_sync_get_enabled() ? 1u : 0u;
    return VC_OK;
}

// Persisted with the preset, like the other per-preset toggles.
static VcResult input_lg_enable_set(VcCtx *c) {
    lg_sound_sync_set_enabled(vc_in_u8(c, 0) != 0);
    return VC_OK;
}

static VcResult input_lg_status_get(VcCtx *c) {
    LgSoundSyncStatus st;
    lg_sound_sync_get_status(&st);
    memcpy(c->out, &st, sizeof(st));
    return VC_OK;
}

#define VC_PAGE_DEF "pages/input.def"
#include "vc_page_table.inc"
