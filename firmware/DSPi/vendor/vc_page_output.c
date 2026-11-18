/*
 * vc_page_output.c: OUTPUT page. Command set: vendor_command_overhaul_plan.md section 5.4.
 * Type and pin moves only queue work: the main loop applies them inside the
 * muted pipeline reset that keeps every output slot in phase.
 */

#include "vc_core.h"
#include "vc_pins.h"
#include "config.h"
#include "usb_audio.h"
#include "audio_input.h"
#include "output_latency.h"
#include "pdm_generator.h"
#include "adat_output.h"
#include "dac_hw_mute.h"
#include "bulk_params.h"
#include "notify.h"
#include "hardware/sync.h"

extern DacHwMuteConfig flash_set_dac_hw_mute_val;

// ---- Type and pins ----

static VcResult output_type_get(VcCtx *c) {
    c->out[0] = output_types[c->addr];
    return VC_OK;
}

// Notified at request time against the eventual type; the main loop applies it.
// A slot that brings the I2S clocks up checks the clock pair first.
static VcResult output_type_set(VcCtx *c) {
    uint8_t slot = (uint8_t)c->addr;
    uint8_t new_type = vc_in_u8(c, 0);
    if (new_type > OUTPUT_TYPE_I2S) return VC_ERR_VALUE;
    if (new_type != output_types[slot]) {
        uint8_t st = pin_start_check_output_type(slot, new_type);
        if (st != PIN_CONFIG_SUCCESS) return vc_from_pin_status(st);
        pending_output_types[slot] = new_type;
        __dmb();
        output_type_change_mask |= (uint8_t)(1u << slot);
    }
    notify_param_write((uint16_t)(offsetof(WireBulkParams, i2s_config.output_types) + slot),
                       1, &new_type);
    return VC_OK;
}

static VcResult output_pin_get(VcCtx *c) {
    c->out[0] = output_pins[c->addr];
    return VC_OK;
}

// A stopped PDM stores its pin; only a running one claims it (vc_pins.c).
static VcResult output_pin_set(VcCtx *c) {
    uint8_t out_idx = (uint8_t)c->addr;
    uint8_t new_pin = vc_in_u8(c, 0);
    if (new_pin == PIN_RESET_TO_DEFAULT) new_pin = default_output_pins[out_idx];
    bool is_pdm = out_idx >= NUM_SPDIF_INSTANCES;
    if (new_pin != output_pins[out_idx]) {
        uint8_t status = pin_check_set(is_pdm ? PIN_FN_PDM : PIN_FN_OUTPUT,
                                       is_pdm ? 0 : out_idx, new_pin, 0xFF);
        if (status != PIN_CONFIG_SUCCESS) return vc_from_pin_status(status);
        if (!is_pdm) {
            // RAM-only; process_pin_changes() restarts the slot in phase with the others.
            output_pins[out_idx] = new_pin;
            output_pin_change_mask |= (uint8_t)(1u << out_idx);
        } else {
            pdm_change_pin(new_pin);
            output_pins[out_idx] = new_pin;
        }
    }
    notify_param_write((uint16_t)(offsetof(WireBulkParams, pins.pins) + out_idx),
                       1, &output_pins[out_idx]);
    return VC_OK;
}

static VcResult output_pin_avail_get(VcCtx *c) {
    PinAvail a;
    bool is_pdm = c->addr >= NUM_SPDIF_INSTANCES;
    pin_avail(is_pdm ? PIN_FN_PDM : PIN_FN_OUTPUT, is_pdm ? 0 : (uint8_t)c->addr, &a);
    c->out_len = pin_avail_serialize(&a, pin_generation_sync(false), c->out, c->out_cap, false);
    return VC_OK;
}

// ---- Latency ----

_Static_assert(sizeof(OutputLatencyStatus) == 12, "OUTPUT.LATENCY_STATUS is 12 bytes");
_Static_assert(sizeof(OutputLatencyOptionWire) == 8, "OUTPUT.LATENCY_OPTIONS entries are 8 bytes");

static inline uint16_t frames_to_us(uint32_t frames, uint32_t fs) {
    return (uint16_t)(((uint64_t)frames * 1000000u + fs / 2u) / fs);
}

// A queued request reads back at once, so SET then GET round-trips before the main loop runs.
static VcResult output_latency_get_h(VcCtx *c) {
    c->out[0] = flash_set_output_latency_pending ? flash_set_output_latency_id
                                                 : output_latency_requested();
    return VC_OK;
}

// The main loop owns the flash write and the output restart. Re-sending the stored option is a no-op.
static VcResult output_latency_set_h(VcCtx *c) {
    uint8_t id = vc_in_u8(c, 0);
    if (!output_latency_find(id)) return VC_ERR_VALUE;
    if (id == output_latency_requested() && !flash_set_output_latency_pending) return VC_OK;
    flash_set_output_latency_id = id;
    __dmb();
    flash_set_output_latency_pending = true;
    return VC_OK;
}

static VcResult output_latency_status_get(VcCtx *c) {
    uint32_t fs = audio_state.freq ? audio_state.freq : 48000u;
    OutputLatencyStatus st = {0};
    st.requested_id  = output_latency_requested();
    st.effective_id  = output_latency_effective();
    st.effective_us  = frames_to_us(output_ring_target_frames, fs);
    st.target_frames = (uint16_t)output_ring_target_frames;
    st.batch_frames  = (uint16_t)input_batch_frames;
    st.pending       = flash_set_output_latency_pending ? 1 : 0;
    st.option_count  = output_latency_option_count;
    memcpy(c->out, &st, sizeof(st));
    return VC_OK;
}

static VcResult output_latency_options_get(VcCtx *c) {
    uint32_t fs = audio_state.freq ? audio_state.freq : 48000u;
    uint16_t n = 0;
    for (uint8_t i = 0; i < output_latency_option_count; i++) {
        if (n + sizeof(OutputLatencyOptionWire) > c->out_cap) break;
        const OutputLatencyOption *o = &output_latency_options[i];
        const OutputLatencyOption *run = output_latency_resolve(o->id, active_input_source);
        OutputLatencyOptionWire e = {
            .id = o->id,
            .flags = (uint8_t)(o->flags | (run == o ? OUTPUT_LATENCY_WIRE_AVAILABLE : 0)),
            .latency_us = o->latency_us,
            .batch_us = o->batch_us,
            .effective_us = frames_to_us(output_latency_target_frames(run, fs), fs),
        };
        memcpy(c->out + n, &e, sizeof(e));
        n += sizeof(e);
    }
    c->out_len = n;
    return VC_OK;
}

// ---- ADAT TX ----

#if PICO_RP2350
_Static_assert(sizeof(AdatStatus) == 8, "OUTPUT.ADAT_TX_STATUS is 8 bytes");

static VcResult output_adat_tx_enable_get(VcCtx *c) {
    c->out[0] = adat_output_config_enabled() ? 1 : 0;
    return VC_OK;
}

static VcResult output_adat_tx_enable_set(VcCtx *c) {
    uint8_t value = vc_in_u8(c, 0);
    if (value > 1) return VC_ERR_VALUE;
    if (value == (adat_output_config_enabled() ? 1 : 0)) {
        // Unchanged; the notify below still runs.
    } else if (value) {
        uint8_t st = pin_start_check(PIN_FN_ADAT_TX, 0);
        if (st != PIN_CONFIG_SUCCESS) return vc_from_pin_status(st);
        adat_output_set_config(true, adat_output_pin());
    } else {
        adat_output_set_config(false, adat_output_pin());
    }
    uint8_t v = adat_output_config_enabled() ? 1 : 0;
    notify_param_write(offsetof(WireBulkParams, adat_config.enabled), 1, &v);
    return VC_OK;
}

static VcResult output_adat_tx_pin_get(VcCtx *c) {
    c->out[0] = adat_output_pin();
    return VC_OK;
}

// A re-route is allowed while enabled; the deferred apply moves the stream under mute.
// A disabled output stores its pin; enable checks it.
static VcResult output_adat_tx_pin_set(VcCtx *c) {
    uint8_t pin = vc_in_u8(c, 0);
    if (pin == PIN_RESET_TO_DEFAULT) pin = PICO_ADAT_PIN;
    uint8_t st = pin_check_set(PIN_FN_ADAT_TX, 0, pin, 0xFF);
    if (st != PIN_CONFIG_SUCCESS) return vc_from_pin_status(st);
    if (pin != adat_output_pin()) adat_output_set_config(adat_output_config_enabled(), pin);
    uint8_t p = adat_output_pin();
    notify_param_write(offsetof(WireBulkParams, adat_config.pin), 1, &p);
    return VC_OK;
}

static VcResult output_adat_tx_pin_avail_get(VcCtx *c) {
    PinAvail a;
    pin_avail(PIN_FN_ADAT_TX, 0, &a);
    c->out_len = pin_avail_serialize(&a, pin_generation_sync(false), c->out, c->out_cap, false);
    return VC_OK;
}

static VcResult output_adat_tx_status_get(VcCtx *c) {
    AdatStatus s;
    adat_output_get_status(&s);
    memcpy(c->out, &s, sizeof(s));
    return VC_OK;
}
#endif

// ---- DAC hardware mute ----

static VcResult output_dac_mute_config_get(VcCtx *c) {
    DacHwMuteConfig hw;
    dac_hw_mute_get_config(&hw);
    memcpy(c->out, &hw, sizeof(hw));
    return VC_OK;
}

// Scalar fields and pin validity the deferred dac_hw_mute_set_config() would refuse
// are refused here. Ownership still resolves on the main loop; read back to confirm.
static VcResult output_dac_mute_config_set(VcCtx *c) {
    DacHwMuteConfig cfg;
    memcpy(&cfg, c->in, sizeof(cfg));
    if (!dac_hw_mute_scalars_valid(&cfg)) return VC_ERR_VALUE;
    if (cfg.pin != DAC_HW_MUTE_PIN_NONE) {
        PinRequest rq = { PIN_FN_DAC_MUTE, 0, { cfg.pin, 0xFF } };
        if (pin_check(pin_state_live(), &rq, PIN_CHECK_STORE) != PIN_CONFIG_SUCCESS)
            return VC_ERR_PIN_INVALID;
    }
    memcpy((void *)&flash_set_dac_hw_mute_val, &cfg, sizeof(cfg));
    __dmb();
    flash_set_dac_hw_mute_pending = true;
    return VC_OK;
}

static VcResult output_dac_mute_pin_avail_get(VcCtx *c) {
    PinAvail a;
    pin_avail(PIN_FN_DAC_MUTE, 0, &a);
    c->out_len = pin_avail_serialize(&a, pin_generation_sync(false), c->out, c->out_cap, false);
    return VC_OK;
}

// The pulse busy-waits, so it runs from the main loop after the reply.
static VcResult output_dac_mute_test_set(VcCtx *c) {
    (void)c;
    DacHwMuteConfig hw;
    dac_hw_mute_get_config(&hw);
    if (hw.enabled == 0) return VC_ERR_STATE;
    dac_hw_mute_test_pending = true;
    __dmb();
    return VC_OK;
}

#define VC_PAGE_DEF "pages/output.def"
#include "vc_page_table.inc"
