/*
 * bulk_params.c — Bulk parameter collect/apply for DSPi
 *
 * Snapshots the entire live DSP state into the wire format (GET), or
 * applies a received wire-format payload back to live state (SET).
 *
 * bulk_params_apply() writes to the same globals as
 * apply_slot_to_live() in flash_storage.c but does NOT recalculate
 * filters or delays — the caller must do that.
 */

#include "bulk_params.h"
#include "config.h"
#include "dsp_pipeline.h"
#include "usb_audio.h"
#include "crossfeed.h"
#include "leveller.h"

#include <string.h>

// External variables (defined in usb_audio.c)
extern volatile float global_preamp_db;
extern volatile int32_t global_preamp_mul;
extern volatile float global_preamp_linear;
extern volatile float channel_gain_db[3];
extern volatile int32_t channel_gain_mul[3];
extern volatile float channel_gain_linear[3];
extern volatile bool channel_mute[3];
extern volatile bool loudness_enabled;
extern volatile float loudness_ref_spl;
extern volatile float loudness_intensity_pct;
extern volatile bool loudness_recompute_pending;
extern volatile CrossfeedConfig crossfeed_config;
extern volatile bool crossfeed_update_pending;
extern volatile LevellerConfig leveller_config;
extern volatile bool leveller_update_pending;
extern volatile bool leveller_reset_pending;
extern MatrixMixer matrix_mixer;
extern uint8_t output_pins[NUM_PIN_OUTPUTS];

// ============================================================================
// dB-TO-LINEAR CONVERSION (duplicated from flash_storage.c — it's static there)
// ============================================================================

static float db_to_linear(float db) {
    if (db == 0.0f) return 1.0f;
    if (db < -60.0f) db = -60.0f;
    if (db > 20.0f) db = 20.0f;
    float x = db * 0.1151292546f;  // ln(10)/20
    float linear = 1.0f + x + x*x*0.5f + x*x*x*0.1666667f + x*x*x*x*0.0416667f;
    return (linear < 0.0f) ? 0.0f : linear;
}

// ============================================================================
// COLLECT: Live state → wire format
// ============================================================================

void bulk_params_collect(WireBulkParams *out) {
    memset(out, 0, sizeof(*out));

    // Header
    out->header.format_version = WIRE_FORMAT_VERSION;
#if PICO_RP2350
    out->header.platform_id = WIRE_PLATFORM_RP2350;
#else
    out->header.platform_id = WIRE_PLATFORM_RP2040;
#endif
    out->header.num_channels = NUM_CHANNELS;
    out->header.num_output_channels = NUM_OUTPUT_CHANNELS;
    out->header.num_input_channels = NUM_INPUT_CHANNELS;
    out->header.max_bands = MAX_BANDS;
    out->header.payload_length = sizeof(WireBulkParams);
    out->header.fw_version_major = FW_VERSION_MAJOR;
    out->header.fw_version_minor = FW_VERSION_MINOR;

    // Global params
    out->global.preamp_gain_db = global_preamp_db;
    out->global.bypass = bypass_master_eq ? 1 : 0;
    out->global.loudness_enabled = loudness_enabled ? 1 : 0;
    out->global.loudness_ref_spl = loudness_ref_spl;
    out->global.loudness_intensity_pct = loudness_intensity_pct;

    // Crossfeed
    out->crossfeed.enabled = crossfeed_config.enabled ? 1 : 0;
    out->crossfeed.preset = crossfeed_config.preset;
    out->crossfeed.itd_enabled = crossfeed_config.itd_enabled ? 1 : 0;
    out->crossfeed.custom_fc = crossfeed_config.custom_fc;
    out->crossfeed.custom_feed_db = crossfeed_config.custom_feed_db;

    // Legacy channels
    for (int i = 0; i < 3; i++) {
        out->legacy.gain_db[i] = channel_gain_db[i];
        out->legacy.mute[i] = channel_mute[i] ? 1 : 0;
    }

    // Delays
    for (int i = 0; i < NUM_CHANNELS; i++) {
        out->delays.delay_ms[i] = channel_delays_ms[i];
    }

    // Matrix crosspoints
    for (int in = 0; in < NUM_INPUT_CHANNELS; in++) {
        for (int o = 0; o < NUM_OUTPUT_CHANNELS; o++) {
            out->crosspoints[in][o].enabled = matrix_mixer.crosspoints[in][o].enabled;
            out->crosspoints[in][o].phase_invert = matrix_mixer.crosspoints[in][o].phase_invert;
            out->crosspoints[in][o].gain_db = matrix_mixer.crosspoints[in][o].gain_db;
        }
    }

    // Matrix outputs
    for (int o = 0; o < NUM_OUTPUT_CHANNELS; o++) {
        out->outputs[o].enabled = matrix_mixer.outputs[o].enabled;
        out->outputs[o].mute = matrix_mixer.outputs[o].mute;
        out->outputs[o].gain_db = matrix_mixer.outputs[o].gain_db;
        out->outputs[o].delay_ms = matrix_mixer.outputs[o].delay_ms;
    }

    // Pin config (always included in GET regardless of include_pins setting)
    out->pins.num_pin_outputs = NUM_PIN_OUTPUTS;
    for (int i = 0; i < NUM_PIN_OUTPUTS; i++) {
        out->pins.pins[i] = output_pins[i];
    }

    // EQ bands
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        for (int b = 0; b < MAX_BANDS; b++) {
            out->eq[ch][b].type = filter_recipes[ch][b].type;
            out->eq[ch][b].freq = filter_recipes[ch][b].freq;
            out->eq[ch][b].q = filter_recipes[ch][b].Q;
            out->eq[ch][b].gain_db = filter_recipes[ch][b].gain_db;
        }
    }

    // Channel names
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        memcpy(out->channel_names.names[ch], channel_names[ch], PRESET_NAME_LEN);
    }

    // I2S configuration (V3)
    {
        extern uint8_t output_types[];
        extern uint8_t i2s_bck_pin;
        extern uint8_t i2s_mck_pin;
        extern bool    i2s_mck_enabled;
        extern uint8_t i2s_mck_multiplier;
        memset(&out->i2s_config, 0, sizeof(out->i2s_config));
        memcpy(out->i2s_config.output_types, output_types, NUM_SPDIF_INSTANCES);
        out->i2s_config.bck_pin = i2s_bck_pin;
        out->i2s_config.mck_pin = i2s_mck_pin;
        out->i2s_config.mck_enabled = i2s_mck_enabled ? 1 : 0;
        out->i2s_config.mck_multiplier = i2s_mck_multiplier;
    }

    // Volume Leveller (V4+)
    out->leveller.enabled = leveller_config.enabled ? 1 : 0;
    out->leveller.speed = leveller_config.speed;
    out->leveller.lookahead = leveller_config.lookahead ? 1 : 0;
    out->leveller.amount = leveller_config.amount;
    out->leveller.max_gain_db = leveller_config.max_gain_db;
    out->leveller.gate_threshold_db = leveller_config.gate_threshold_db;
}

// ============================================================================
// APPLY: Wire format → live state
// ============================================================================

int bulk_params_apply(const WireBulkParams *in, bool apply_pins) {
    // Validate header (accept V2-V4 for backward compat)
    // V2: lacks I2S + leveller sections, V3: lacks leveller section
    if (in->header.format_version < 2 || in->header.format_version > WIRE_FORMAT_VERSION)
        return -1;

#if PICO_RP2350
    if (in->header.platform_id != WIRE_PLATFORM_RP2350)
        return -2;
#else
    if (in->header.platform_id != WIRE_PLATFORM_RP2040)
        return -2;
#endif

    if (in->header.num_channels != NUM_CHANNELS)
        return -3;
    if (in->header.num_output_channels != NUM_OUTPUT_CHANNELS)
        return -3;
    // Accept V2 (2832 bytes, no I2S section) or V3 (2848 bytes, with I2S)
    uint16_t v2_size = sizeof(WireBulkParams) - sizeof(WireI2SConfig);
    if (in->header.payload_length != sizeof(WireBulkParams) &&
        in->header.payload_length != v2_size)
        return -4;

    // Global params
    global_preamp_db = in->global.preamp_gain_db;
    float linear = db_to_linear(in->global.preamp_gain_db);
    global_preamp_mul = (int32_t)(linear * (float)(1 << 28));
    global_preamp_linear = linear;

    bypass_master_eq = (in->global.bypass != 0);

    loudness_enabled = (in->global.loudness_enabled != 0);
    loudness_ref_spl = in->global.loudness_ref_spl;
    loudness_intensity_pct = in->global.loudness_intensity_pct;
    loudness_recompute_pending = true;

    // Crossfeed
    crossfeed_config.enabled = (in->crossfeed.enabled != 0);
    crossfeed_config.preset = in->crossfeed.preset;
    crossfeed_config.itd_enabled = (in->crossfeed.itd_enabled != 0);
    crossfeed_config.custom_fc = in->crossfeed.custom_fc;
    crossfeed_config.custom_feed_db = in->crossfeed.custom_feed_db;
    crossfeed_update_pending = true;

    // Legacy channels
    for (int i = 0; i < 3; i++) {
        channel_gain_db[i] = in->legacy.gain_db[i];
        float g = db_to_linear(in->legacy.gain_db[i]);
        channel_gain_mul[i] = (int32_t)(g * 32768.0f);
        channel_gain_linear[i] = g;
        channel_mute[i] = (in->legacy.mute[i] != 0);
    }

    // Delays
    for (int i = 0; i < NUM_CHANNELS; i++) {
        channel_delays_ms[i] = in->delays.delay_ms[i];
    }

    // Matrix crosspoints
    for (int inp = 0; inp < NUM_INPUT_CHANNELS; inp++) {
        for (int o = 0; o < NUM_OUTPUT_CHANNELS; o++) {
            matrix_mixer.crosspoints[inp][o].enabled = in->crosspoints[inp][o].enabled;
            matrix_mixer.crosspoints[inp][o].phase_invert = in->crosspoints[inp][o].phase_invert;
            matrix_mixer.crosspoints[inp][o].gain_db = in->crosspoints[inp][o].gain_db;
            matrix_mixer.crosspoints[inp][o].gain_linear = db_to_linear(in->crosspoints[inp][o].gain_db);
        }
    }

    // Matrix outputs
    for (int o = 0; o < NUM_OUTPUT_CHANNELS; o++) {
        matrix_mixer.outputs[o].enabled = in->outputs[o].enabled;
        matrix_mixer.outputs[o].mute = in->outputs[o].mute;
        matrix_mixer.outputs[o].gain_db = in->outputs[o].gain_db;
        matrix_mixer.outputs[o].gain_linear = db_to_linear(in->outputs[o].gain_db);
        matrix_mixer.outputs[o].delay_ms = in->outputs[o].delay_ms;
        channel_delays_ms[CH_OUT_1 + o] = in->outputs[o].delay_ms;
    }

    // Pin config
    if (apply_pins) {
#if PICO_RP2350
        static const uint8_t default_pins[NUM_PIN_OUTPUTS] = {
            PICO_AUDIO_SPDIF_PIN, PICO_SPDIF_PIN_2,
            PICO_SPDIF_PIN_3, PICO_SPDIF_PIN_4, PICO_PDM_PIN
        };
#else
        static const uint8_t default_pins[NUM_PIN_OUTPUTS] = {
            PICO_AUDIO_SPDIF_PIN, PICO_SPDIF_PIN_2, PICO_PDM_PIN
        };
#endif
        for (int i = 0; i < NUM_PIN_OUTPUTS; i++) {
            uint8_t pin = in->pins.pins[i];
            bool valid = (pin <= 29) && (pin != 12) && !(pin >= 23 && pin <= 25);
#if !PICO_RP2350
            if (pin > 28) valid = false;
#endif
            output_pins[i] = valid ? pin : default_pins[i];
        }
    }

    // EQ bands
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        for (int b = 0; b < MAX_BANDS; b++) {
            filter_recipes[ch][b].channel = ch;
            filter_recipes[ch][b].band = b;
            filter_recipes[ch][b].type = in->eq[ch][b].type;
            filter_recipes[ch][b].freq = in->eq[ch][b].freq;
            filter_recipes[ch][b].Q = in->eq[ch][b].q;
            filter_recipes[ch][b].gain_db = in->eq[ch][b].gain_db;
        }
    }

    // Channel names
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        memcpy(channel_names[ch], in->channel_names.names[ch], PRESET_NAME_LEN);
        channel_names[ch][PRESET_NAME_LEN - 1] = '\0';  // Enforce NUL termination
    }

    // I2S configuration (V3 payloads only — V2 payloads skip this)
    if (in->header.format_version >= 3 &&
        in->header.payload_length >= sizeof(WireBulkParams)) {
        extern uint8_t output_types[];
        extern uint8_t i2s_bck_pin;
        extern uint8_t i2s_mck_pin;
        extern bool    i2s_mck_enabled;
        extern uint8_t i2s_mck_multiplier;
        memcpy(output_types, in->i2s_config.output_types, NUM_SPDIF_INSTANCES);
        i2s_bck_pin = in->i2s_config.bck_pin;
        i2s_mck_pin = in->i2s_config.mck_pin;
        i2s_mck_enabled = (in->i2s_config.mck_enabled != 0);
        i2s_mck_multiplier = in->i2s_config.mck_multiplier;
    }

    // Volume Leveller (V4+ payloads only)
    if (in->header.format_version >= 4) {
        leveller_config.enabled = (in->leveller.enabled != 0);
        leveller_config.speed = in->leveller.speed;
        leveller_config.lookahead = (in->leveller.lookahead != 0);
        leveller_config.amount = in->leveller.amount;
        leveller_config.max_gain_db = in->leveller.max_gain_db;
        leveller_config.gate_threshold_db = in->leveller.gate_threshold_db;
    } else {
        // V2/V3 payload: apply defaults
        leveller_config.enabled = LEVELLER_DEFAULT_ENABLED;
        leveller_config.amount = LEVELLER_DEFAULT_AMOUNT;
        leveller_config.speed = LEVELLER_DEFAULT_SPEED;
        leveller_config.max_gain_db = LEVELLER_DEFAULT_MAX_GAIN_DB;
        leveller_config.lookahead = LEVELLER_DEFAULT_LOOKAHEAD;
        leveller_config.gate_threshold_db = LEVELLER_DEFAULT_GATE_DB;
    }
    leveller_update_pending = true;
    leveller_reset_pending = true;

    return 0;
}
