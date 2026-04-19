/*
 * vendor_commands.c — Vendor USB control request handlers for DSPi
 *
 * Phase 2 (TinyUSB): the legacy pico-extras setup/packet handlers have been
 * replaced by TinyUSB's stage-based tud_control_xfer flow.  All 30+ command
 * handlers inside the SET and GET switch statements are unchanged — they
 * continue to consume `vendor_rx_buf` / produce responses via
 * `vendor_send_response()`, which now trampolines into `tud_control_xfer()`.
 *
 * All state DEFINITIONS remain in usb_audio.c; this file accesses them
 * via extern declarations in usb_audio.h.
 */

#include "vendor_commands.h"
#include "usb_audio.h"
#include "audio_input.h"
#include "spdif_input.h"
#include "audio_pipeline.h"
#include "config.h"
#include "dsp_pipeline.h"
#include "flash_storage.h"
#include "loudness.h"
#include "crossfeed.h"
#include "leveller.h"
#include "bulk_params.h"
#include "pdm_generator.h"
#include "usb_descriptors.h"
#include "tusb.h"
#include "pico/audio_spdif.h"
#include "pico/audio_i2s_multi.h"
#include "hardware/adc.h"
#include "hardware/vreg.h"
#include "hardware/clocks.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"

// ----------------------------------------------------------------------------
// VENDOR-INTERNAL STATIC STATE
// ----------------------------------------------------------------------------

// SET payload buffer (wLength <= 64 for regular SET; bulk SET uses bulk_param_buf).
static uint8_t vendor_rx_buf[64];
static uint8_t vendor_last_request = 0;
static uint16_t vendor_last_wValue = 0;
static uint16_t vendor_last_wLength = 0;

// Captured during SETUP so vendor_send_response() can issue tud_control_xfer()
// without every GET case having to plumb rhport + request through.
static uint8_t _vendor_rhport;
static tusb_control_request_t const *_vendor_current_req;

// Internal helpers exported via vendor_send_response() replacement.
static void vendor_send_response(const void *data, uint16_t len);

// Forward declarations: SET data-stage and GET dispatch.
static void vendor_handle_set_data(tusb_control_request_t const *req);
static bool vendor_handle_get(tusb_control_request_t const *req);

// Shim type preserved from the pico-extras era so the SET handler case
// bodies (which still read `buffer->data_len` and `buffer->data`) compile
// unmodified against TinyUSB's already-buffered control data.
typedef struct {
    uint8_t *data;
    uint16_t data_len;
} vendor_buffer_t;

// ----------------------------------------------------------------------------
// SYSTEM STATISTICS HELPERS (moved from usb_audio.c)
// ----------------------------------------------------------------------------

// Convert vreg voltage enum to millivolts
uint16_t vreg_voltage_to_mv(enum vreg_voltage voltage) {
    // Voltage enum values map to specific voltages
    // See hardware/vreg.h for the full table
    static const uint16_t voltage_table[] = {
#if !PICO_RP2040
        550,  // VREG_VOLTAGE_0_55 = 0b00000
        600,  // VREG_VOLTAGE_0_60 = 0b00001
        650,  // VREG_VOLTAGE_0_65 = 0b00010
        700,  // VREG_VOLTAGE_0_70 = 0b00011
        750,  // VREG_VOLTAGE_0_75 = 0b00100
        800,  // VREG_VOLTAGE_0_80 = 0b00101
#endif
        850,  // VREG_VOLTAGE_0_85 = 0b00110
        900,  // VREG_VOLTAGE_0_90 = 0b00111
        950,  // VREG_VOLTAGE_0_95 = 0b01000
        1000, // VREG_VOLTAGE_1_00 = 0b01001
        1050, // VREG_VOLTAGE_1_05 = 0b01010
        1100, // VREG_VOLTAGE_1_10 = 0b01011
        1150, // VREG_VOLTAGE_1_15 = 0b01100
        1200, // VREG_VOLTAGE_1_20 = 0b01101
        1250, // VREG_VOLTAGE_1_25 = 0b01110
        1300, // VREG_VOLTAGE_1_30 = 0b01111
#if !PICO_RP2040
        1350, // VREG_VOLTAGE_1_35 = 0b10000
        1400, // VREG_VOLTAGE_1_40 = 0b10001
        1500, // VREG_VOLTAGE_1_50 = 0b10010
        1600, // VREG_VOLTAGE_1_60 = 0b10011
        1650, // VREG_VOLTAGE_1_65 = 0b10100
        1700, // VREG_VOLTAGE_1_70 = 0b10101
        1800, // VREG_VOLTAGE_1_80 = 0b10110
        1900, // VREG_VOLTAGE_1_90 = 0b10111
        2000, // VREG_VOLTAGE_2_00 = 0b11000
        2350, // VREG_VOLTAGE_2_35 = 0b11001
        2500, // VREG_VOLTAGE_2_50 = 0b11010
        2650, // VREG_VOLTAGE_2_65 = 0b11011
        2800, // VREG_VOLTAGE_2_80 = 0b11100
        3000, // VREG_VOLTAGE_3_00 = 0b11101
        3150, // VREG_VOLTAGE_3_15 = 0b11110
        3300, // VREG_VOLTAGE_3_30 = 0b11111
#endif
    };

#if PICO_RP2040
    // RP2040: offset by 6 entries (0.55-0.80V not available)
    uint8_t index = (voltage >= 6) ? (voltage - 6) : 0;
#else
    uint8_t index = voltage;
#endif

    if (index < sizeof(voltage_table) / sizeof(voltage_table[0])) {
        return voltage_table[index];
    }
    return 1100; // Default fallback
}

// Read ADC temperature sensor and return temperature in centi-degrees C
// Formula from SDK docs (same for RP2040/RP2350):
// T = 27 - (ADC_Voltage - 0.706) / 0.001721
int16_t read_temperature_cdeg(void) {
    const float conversion_factor = 3.3f / 4095.0f;

    // Temperature sensor channel: auto-detects based on chip variant
    // RP2040, RP2350A (QFN-60): channel 4
    // RP2350B (QFN-80): channel 8
    adc_select_input(NUM_ADC_CHANNELS - 1);
    uint16_t adc_raw = adc_read();
    float voltage = adc_raw * conversion_factor;
    float temp_c = 27.0f - (voltage - 0.706f) / 0.001721f;

    return (int16_t)(temp_c * 100.0f); // Convert to centi-degrees
}

// ----------------------------------------------------------------------------
// CORE 1 MODE DERIVATION (moved from usb_audio.c)
// ----------------------------------------------------------------------------

// derive_core1_mode lives in usb_audio.c (moved during Phase 1 migration).

// ----------------------------------------------------------------------------
// MCK HELPERS (moved from usb_audio.c)
// ----------------------------------------------------------------------------

// Encode/decode for wire and flash persistence: 0 = 128x, 1 = 256x
uint8_t  mck_encode(uint16_t val) { return (val == 256) ? 1 : 0; }
uint16_t mck_decode(uint8_t raw)  { return (raw == 1) ? 256 : 128; }

// 96 kHz + 256x requires a 24.576 MHz MCK derived from a highly fractional
// divider on the current fixed sys_clk plan. On real hardware this mode has
// proven unreliable (lock loss / silence), so clamp to 128x for 96 kHz+.
bool is_mck_multiplier_supported_for_rate(uint16_t mult, uint32_t sample_rate_hz) {
    return !(mult == 256u && sample_rate_hz >= 96000u);
}

void sanitize_mck_multiplier_for_rate(uint32_t sample_rate_hz) {
    if (sample_rate_hz >= 96000u && i2s_mck_multiplier == 256u) {
        i2s_mck_multiplier = 128u;
        printf("MCK 256x not supported at %lu Hz; forcing 128x\n",
               (unsigned long)sample_rate_hz);
    }
}

// ----------------------------------------------------------------------------
// PIN VALIDATION HELPERS (moved from usb_audio.c)
// ----------------------------------------------------------------------------

bool is_valid_gpio_pin(uint8_t pin) {
    if (pin == 12) return false;                // UART TX
    if (pin >= 23 && pin <= 25) return false;   // Power/LED
#if PICO_RP2350
    return pin <= 29;
#else
    return pin <= 28;
#endif
}

bool is_pin_in_use(uint8_t pin, uint8_t exclude) {
    for (int i = 0; i < NUM_PIN_OUTPUTS; i++) {
        if (i == exclude) continue;
        if (output_pins[i] == pin) return true;
    }
    // Also check I2S BCK and LRCLK pins if any slot is I2S
    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        if (output_types[i] == OUTPUT_TYPE_I2S) {
            if (pin == i2s_bck_pin || pin == (i2s_bck_pin + 1)) return true;
            break;  // All I2S slots share the same BCK/LRCLK
        }
    }
    // Check MCK pin if enabled
    if (i2s_mck_enabled && pin == i2s_mck_pin) return true;
    // Check SPDIF RX input pin
    if (pin == spdif_rx_pin) return true;
    return false;
}

// ----------------------------------------------------------------------------
// VENDOR SET HANDLER (moved from usb_audio.c)
// ----------------------------------------------------------------------------

static void vendor_handle_set_data(tusb_control_request_t const *req) {
    (void)req;
    // Shim for legacy handler bodies below.  The real payload was delivered
    // into vendor_rx_buf by tud_control_xfer() during the DATA stage; we
    // synthesize a local struct with the same shape as the pico-extras
    // `buffer` the handlers used to read, so the case bodies stay unchanged.
    vendor_buffer_t _buf = { vendor_rx_buf, vendor_last_wLength };
    vendor_buffer_t *buffer = &_buf;
    (void)buffer;

    // Process command based on saved request info
    switch (vendor_last_request) {
        case REQ_SET_EQ_PARAM:
            if (buffer->data_len >= sizeof(EqParamPacket)) {
                memcpy((void*)&pending_packet, vendor_rx_buf, sizeof(EqParamPacket));
                if (pending_packet.channel < NUM_CHANNELS &&
                    pending_packet.band < channel_band_counts[pending_packet.channel]) {
                    eq_update_pending = true;
                }
            }
            break;

        case REQ_SET_PREAMP:
            // Legacy: sets ALL input channels to the same preamp value.
            // Payload: 4 bytes (float dB).
            if (buffer->data_len >= 4) {
                float db;
                memcpy(&db, vendor_rx_buf, 4);
                for (int ch = 0; ch < NUM_INPUT_CHANNELS; ch++)
                    update_preamp(ch, db);
            }
            break;

        case REQ_SET_PREAMP_CH: {
            // Per-channel preamp.  wValue = input channel index (0=L, 1=R).
            // Payload: 4 bytes (float dB).
            uint8_t ch = vendor_last_wValue & 0xFF;
            if (ch < NUM_INPUT_CHANNELS && buffer->data_len >= 4) {
                float db;
                memcpy(&db, vendor_rx_buf, 4);
                update_preamp(ch, db);
            }
            break;
        }

        case REQ_SET_MASTER_VOLUME:
            // Set device-side master volume ceiling.
            // Payload: 4 bytes (float dB).  -128 = mute, -127..0 = attenuation range.
            if (buffer->data_len >= 4) {
                float db;
                memcpy(&db, vendor_rx_buf, 4);
                // Bracket the call so NOTIFY_SUPPRESS_HOST_ECHO (in usb_audio.c)
                // can filter out host-originated echoes if enabled.
                notify_master_vol_host_initiated = true;
                update_master_volume(db);
                notify_master_vol_host_initiated = false;
            }
            break;

        case REQ_SET_DELAY: {
            uint8_t ch = vendor_last_wValue & 0xFF;
            if (ch < NUM_CHANNELS && buffer->data_len >= 4) {
                float ms;
                memcpy(&ms, vendor_rx_buf, 4);
                if (ms < 0) ms = 0;
                channel_delays_ms[ch] = ms;
                dsp_update_delay_samples((float)audio_state.freq);
            }
            break;
        }

        case REQ_SET_BYPASS:
            if (buffer->data_len >= 1) {
                bypass_master_eq = (vendor_rx_buf[0] != 0);
            }
            break;

        case REQ_SET_CHANNEL_GAIN: {
            uint8_t ch = vendor_last_wValue & 0xFF;
            if (ch < 3 && buffer->data_len >= 4) {
                float db;
                memcpy(&db, vendor_rx_buf, 4);
                channel_gain_db[ch] = db;
                float linear = powf(10.0f, db / 20.0f);
                channel_gain_mul[ch] = (int32_t)(linear * 32768.0f);
                channel_gain_linear[ch] = linear;
            }
            break;
        }

        case REQ_SET_CHANNEL_MUTE: {
            uint8_t ch = vendor_last_wValue & 0xFF;
            if (ch < 3 && buffer->data_len >= 1) {
                channel_mute[ch] = (vendor_rx_buf[0] != 0);
            }
            break;
        }

        case REQ_SET_LOUDNESS:
            if (buffer->data_len >= 1) {
                loudness_enabled = (vendor_rx_buf[0] != 0);
                if (loudness_enabled && loudness_active_table) {
                    // Re-select coefficients for current volume
                    int16_t vol = audio_state.volume + CENTER_VOLUME_INDEX * 256;
                    if (vol < 0) vol = 0;
                    if (vol >= (CENTER_VOLUME_INDEX + 1) * 256) vol = (CENTER_VOLUME_INDEX + 1) * 256 - 1;
                    current_loudness_coeffs = loudness_active_table[((uint16_t)vol) >> 8u];
                } else {
                    current_loudness_coeffs = NULL;
                }
            }
            break;

        case REQ_SET_LOUDNESS_REF:
            if (buffer->data_len >= 4) {
                float val;
                memcpy(&val, vendor_rx_buf, 4);
                if (val < 40.0f) val = 40.0f;
                if (val > 100.0f) val = 100.0f;
                loudness_ref_spl = val;
                loudness_recompute_pending = true;
            }
            break;

        case REQ_SET_LOUDNESS_INTENSITY:
            if (buffer->data_len >= 4) {
                float val;
                memcpy(&val, vendor_rx_buf, 4);
                if (val < 0.0f) val = 0.0f;
                if (val > 200.0f) val = 200.0f;
                loudness_intensity_pct = val;
                loudness_recompute_pending = true;
            }
            break;

        case REQ_SET_CROSSFEED:
            if (buffer->data_len >= 1) {
                crossfeed_config.enabled = (vendor_rx_buf[0] != 0);
                crossfeed_update_pending = true;
            }
            break;

        case REQ_SET_CROSSFEED_PRESET:
            if (buffer->data_len >= 1) {
                uint8_t preset = vendor_rx_buf[0];
                if (preset <= CROSSFEED_PRESET_CUSTOM) {
                    crossfeed_config.preset = preset;
                    crossfeed_update_pending = true;
                }
            }
            break;

        case REQ_SET_CROSSFEED_FREQ:
            if (buffer->data_len >= 4) {
                float val;
                memcpy(&val, vendor_rx_buf, 4);
                if (val < CROSSFEED_FREQ_MIN) val = CROSSFEED_FREQ_MIN;
                if (val > CROSSFEED_FREQ_MAX) val = CROSSFEED_FREQ_MAX;
                crossfeed_config.custom_fc = val;
                if (crossfeed_config.preset == CROSSFEED_PRESET_CUSTOM) {
                    crossfeed_update_pending = true;
                }
            }
            break;

        case REQ_SET_CROSSFEED_FEED:
            if (buffer->data_len >= 4) {
                float val;
                memcpy(&val, vendor_rx_buf, 4);
                if (val < CROSSFEED_FEED_MIN) val = CROSSFEED_FEED_MIN;
                if (val > CROSSFEED_FEED_MAX) val = CROSSFEED_FEED_MAX;
                crossfeed_config.custom_feed_db = val;
                if (crossfeed_config.preset == CROSSFEED_PRESET_CUSTOM) {
                    crossfeed_update_pending = true;
                }
            }
            break;

        case REQ_SET_CROSSFEED_ITD:
            if (buffer->data_len >= 1) {
                crossfeed_config.itd_enabled = (vendor_rx_buf[0] != 0);
                crossfeed_update_pending = true;
            }
            break;

        // Volume Leveller Commands
        case REQ_SET_LEVELLER_ENABLE:
            if (buffer->data_len >= 1) {
                leveller_config.enabled = (vendor_rx_buf[0] != 0);
                leveller_update_pending = true;
                leveller_reset_pending = true;  // Reset state when toggling
            }
            break;

        case REQ_SET_LEVELLER_AMOUNT:
            if (buffer->data_len >= 4) {
                float val;
                memcpy(&val, vendor_rx_buf, 4);
                if (val < LEVELLER_AMOUNT_MIN) val = LEVELLER_AMOUNT_MIN;
                if (val > LEVELLER_AMOUNT_MAX) val = LEVELLER_AMOUNT_MAX;
                leveller_config.amount = val;
                leveller_update_pending = true;
            }
            break;

        case REQ_SET_LEVELLER_SPEED:
            if (buffer->data_len >= 1) {
                uint8_t spd = vendor_rx_buf[0];
                if (spd < LEVELLER_SPEED_COUNT) {
                    leveller_config.speed = spd;
                    leveller_update_pending = true;
                }
            }
            break;

        case REQ_SET_LEVELLER_MAX_GAIN:
            if (buffer->data_len >= 4) {
                float val;
                memcpy(&val, vendor_rx_buf, 4);
                if (val < LEVELLER_MAX_GAIN_MIN) val = LEVELLER_MAX_GAIN_MIN;
                if (val > LEVELLER_MAX_GAIN_MAX) val = LEVELLER_MAX_GAIN_MAX;
                leveller_config.max_gain_db = val;
                leveller_update_pending = true;
            }
            break;

        case REQ_SET_LEVELLER_LOOKAHEAD:
            if (buffer->data_len >= 1) {
                leveller_config.lookahead = (vendor_rx_buf[0] != 0);
                leveller_update_pending = true;
                leveller_reset_pending = true;  // Clear delay buffer on toggle
            }
            break;

        case REQ_SET_LEVELLER_GATE:
            if (buffer->data_len >= 4) {
                float val;
                memcpy(&val, vendor_rx_buf, 4);
                if (val < LEVELLER_GATE_MIN) val = LEVELLER_GATE_MIN;
                if (val > LEVELLER_GATE_MAX) val = LEVELLER_GATE_MAX;
                leveller_config.gate_threshold_db = val;
                leveller_update_pending = true;
            }
            break;

        // Matrix Mixer Commands
        case REQ_SET_MATRIX_ROUTE:
            if (buffer->data_len >= sizeof(MatrixRoutePacket)) {
                MatrixRoutePacket pkt;
                memcpy(&pkt, vendor_rx_buf, sizeof(pkt));
                if (pkt.input < NUM_INPUT_CHANNELS && pkt.output < NUM_OUTPUT_CHANNELS) {
                    MatrixCrosspoint *xp = &matrix_mixer.crosspoints[pkt.input][pkt.output];
                    xp->enabled = pkt.enabled;
                    xp->phase_invert = pkt.phase_invert;
                    xp->gain_db = pkt.gain_db;
                    // Compute linear gain
                    xp->gain_linear = powf(10.0f, pkt.gain_db / 20.0f);
                }
            }
            break;

        case REQ_SET_OUTPUT_ENABLE: {
            uint8_t out = vendor_last_wValue & 0xFF;
            if (out < NUM_OUTPUT_CHANNELS && buffer->data_len >= 1) {
                bool want_enable = (vendor_rx_buf[0] != 0);

                // Mutual exclusion interlock: PDM vs EQ worker outputs
                // Core 1 can only do one: PDM or EQ worker (outputs 2+ on both platforms)
                if (want_enable) {
                    bool is_pdm = (out == NUM_OUTPUT_CHANNELS - 1);
                    bool is_core1_eq = (out >= CORE1_EQ_FIRST_OUTPUT && out <= CORE1_EQ_LAST_OUTPUT);

                    if (is_pdm) {
                        for (int i = CORE1_EQ_FIRST_OUTPUT; i <= CORE1_EQ_LAST_OUTPUT; i++) {
                            if (matrix_mixer.outputs[i].enabled) goto skip_enable;
                        }
                    } else if (is_core1_eq) {
                        if (matrix_mixer.outputs[NUM_OUTPUT_CHANNELS - 1].enabled) goto skip_enable;
                    }
                }

                matrix_mixer.outputs[out].enabled = want_enable ? 1 : 0;

                // Determine new Core 1 mode and transition
                Core1Mode new_mode = derive_core1_mode();
                if (new_mode != core1_mode) {
                    core1_mode = new_mode;
#if ENABLE_SUB
                    pdm_set_enabled(new_mode == CORE1_MODE_PDM);
#endif
                    __sev();  // Wake Core 1 to pick up mode change
                }
            }
            skip_enable:
            break;
        }

        case REQ_SET_OUTPUT_GAIN: {
            uint8_t out = vendor_last_wValue & 0xFF;
            if (out < NUM_OUTPUT_CHANNELS && buffer->data_len >= 4) {
                float db;
                memcpy(&db, vendor_rx_buf, 4);
                matrix_mixer.outputs[out].gain_db = db;
                matrix_mixer.outputs[out].gain_linear = powf(10.0f, db / 20.0f);
            }
            break;
        }

        case REQ_SET_OUTPUT_MUTE: {
            uint8_t out = vendor_last_wValue & 0xFF;
            if (out < NUM_OUTPUT_CHANNELS && buffer->data_len >= 1) {
                matrix_mixer.outputs[out].mute = vendor_rx_buf[0];
            }
            break;
        }

        case REQ_SET_OUTPUT_DELAY: {
            uint8_t out = vendor_last_wValue & 0xFF;
            if (out < NUM_OUTPUT_CHANNELS && buffer->data_len >= 4) {
                float ms;
                memcpy(&ms, vendor_rx_buf, 4);
                if (ms < 0) ms = 0;
                matrix_mixer.outputs[out].delay_ms = ms;
                // Update the channel delay used by DSP pipeline
                channel_delays_ms[CH_OUT_1 + out] = ms;
                dsp_update_delay_samples((float)audio_state.freq);
            }
            break;
        }

        // --- Preset SET commands ---

        case REQ_PRESET_SET_NAME: {
            // Deferred to main loop — flash write in dir_flush() is too
            // slow for USB IRQ context.  Copy payload to pending buffer.
            uint8_t slot = vendor_last_wValue & 0xFF;
            if (buffer->data_len > 0) {
                memset(flash_set_name_buf, 0, PRESET_NAME_LEN);
                size_t copy_len = buffer->data_len < (PRESET_NAME_LEN - 1)
                                ? buffer->data_len : (PRESET_NAME_LEN - 1);
                memcpy(flash_set_name_buf, vendor_rx_buf, copy_len);
                flash_set_name_slot = slot;
                __dmb();
                flash_set_name_pending = true;
            }
            break;
        }

        case REQ_PRESET_SET_STARTUP: {
            // Deferred to main loop — flash write in dir_flush().
            if (buffer->data_len >= 2) {
                flash_set_startup_mode = vendor_rx_buf[0];
                flash_set_startup_slot = vendor_rx_buf[1];
                __dmb();
                flash_set_startup_pending = true;
            }
            break;
        }

        case REQ_PRESET_SET_INCLUDE_PINS: {
            // Deferred to main loop — flash write in dir_flush().
            if (buffer->data_len >= 1) {
                flash_set_include_pins_val = vendor_rx_buf[0];
                __dmb();
                flash_set_include_pins_pending = true;
            }
            break;
        }

        case REQ_SET_INCLUDE_MASTER_VOL: {
            // Set whether preset load restores master volume.
            // Deferred to main loop — flash write in dir_flush().
            // Payload: 1 byte (0 = don't restore, 1 = restore on load).
            if (buffer->data_len >= 1) {
                flash_set_include_master_vol_val = vendor_rx_buf[0];
                __dmb();
                flash_set_include_master_vol_pending = true;
            }
            break;
        }

        case REQ_SET_CHANNEL_NAME: {
            // wValue = channel index, payload = 1-32 bytes of name
            uint8_t ch = vendor_last_wValue & 0xFF;
            if (ch < NUM_CHANNELS && buffer->data_len > 0) {
                memset(channel_names[ch], 0, PRESET_NAME_LEN);
                size_t copy_len = buffer->data_len < (PRESET_NAME_LEN - 1)
                                ? buffer->data_len : (PRESET_NAME_LEN - 1);
                memcpy(channel_names[ch], vendor_rx_buf, copy_len);
            }
            break;
        }

        case REQ_SET_INPUT_SOURCE: {
            // Payload: 1 byte = InputSource enum value.
            // Deferred to main loop — actual source switch requires
            // pipeline reset and (Phase 2) hardware start/stop.
            if (buffer->data_len >= 1) {
                uint8_t src = vendor_rx_buf[0];
                if (input_source_valid(src) && src != active_input_source) {
                    pending_input_source = src;
                    __dmb();
                    input_source_change_pending = true;
                }
            }
            break;
        }
    }

    // TinyUSB auto-sends the status-stage ZLP after control_xfer_cb returns
    // true from CONTROL_STAGE_DATA; no explicit empty-IN transfer needed.
}

// ----------------------------------------------------------------------------
// VENDOR RESPONSE HELPER
// ----------------------------------------------------------------------------

// Send a control IN response from the current vendor GET handler.
// Stashes the request + rhport at SETUP so case bodies can stay unchanged.
static void vendor_send_response(const void *data, uint16_t len) {
    tud_control_xfer(_vendor_rhport, _vendor_current_req, (void *)data, len);
}

// Legacy compatibility shim for the pico-extras helper that sent a small
// integer value as a control IN response in one call.  Several handlers use
// it for scalar responses (status/enum/flag values).  Buffer must outlive
// the EP0 transfer — static storage is fine since control xfers are
// serialized by TinyUSB.
static uint32_t _vendor_scalar_resp;
static inline void usb_start_tiny_control_in_transfer(uint32_t val, uint16_t len) {
    _vendor_scalar_resp = val;
    vendor_send_response(&_vendor_scalar_resp, len);
}

// ----------------------------------------------------------------------------
// VENDOR GET DISPATCH
// ----------------------------------------------------------------------------

static bool vendor_handle_get(tusb_control_request_t const *req) {
    // Shim to let legacy case bodies reference `setup->...`.
    tusb_control_request_t const *setup = req;
    (void)setup;

    {
        // Device -> Host (GET requests)
        static uint8_t resp_buf[64];

        switch (setup->bRequest) {
            case REQ_GET_PREAMP: {
                // Legacy: returns channel 0's preamp value (backward compat)
                float current_db = global_preamp_db[0];
                memcpy(resp_buf, &current_db, 4);
                vendor_send_response(resp_buf, 4);
                return true;
            }

            case REQ_GET_PREAMP_CH: {
                // Per-channel preamp GET.  wValue = input channel index.
                uint8_t ch = (uint8_t)setup->wValue;
                if (ch < NUM_INPUT_CHANNELS) {
                    float current_db = global_preamp_db[ch];
                    memcpy(resp_buf, &current_db, 4);
                    vendor_send_response(resp_buf, 4);
                    return true;
                }
                return false;
            }

            case REQ_GET_MASTER_VOLUME: {
                // Returns device master volume in dB (-128 = mute, -127..0 range)
                float db = master_volume_db;
                memcpy(resp_buf, &db, 4);
                vendor_send_response(resp_buf, 4);
                return true;
            }

            case REQ_GET_DELAY: {
                uint8_t ch = (uint8_t)setup->wValue;
                if (ch < NUM_CHANNELS) {
                    memcpy(resp_buf, (void*)&channel_delays_ms[ch], 4);
                    vendor_send_response(resp_buf, 4);
                    return true;
                }
                return false;
            }

            case REQ_GET_BYPASS: {
                resp_buf[0] = bypass_master_eq ? 1 : 0;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_CHANNEL_GAIN: {
                uint8_t ch = (uint8_t)setup->wValue;
                if (ch < 3) {
                    memcpy(resp_buf, (void*)&channel_gain_db[ch], 4);
                    vendor_send_response(resp_buf, 4);
                    return true;
                }
                return false;
            }

            case REQ_GET_CHANNEL_MUTE: {
                uint8_t ch = (uint8_t)setup->wValue;
                if (ch < 3) {
                    resp_buf[0] = channel_mute[ch] ? 1 : 0;
                    vendor_send_response(resp_buf, 1);
                    return true;
                }
                return false;
            }

            case REQ_GET_LOUDNESS: {
                resp_buf[0] = loudness_enabled ? 1 : 0;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_LOUDNESS_REF: {
                float val = loudness_ref_spl;
                memcpy(resp_buf, &val, 4);
                vendor_send_response(resp_buf, 4);
                return true;
            }

            case REQ_GET_LOUDNESS_INTENSITY: {
                float val = loudness_intensity_pct;
                memcpy(resp_buf, &val, 4);
                vendor_send_response(resp_buf, 4);
                return true;
            }

            case REQ_GET_CROSSFEED: {
                resp_buf[0] = crossfeed_config.enabled ? 1 : 0;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_CROSSFEED_PRESET: {
                resp_buf[0] = crossfeed_config.preset;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_CROSSFEED_FREQ: {
                float val = crossfeed_config.custom_fc;
                memcpy(resp_buf, &val, 4);
                vendor_send_response(resp_buf, 4);
                return true;
            }

            case REQ_GET_CROSSFEED_FEED: {
                float val = crossfeed_config.custom_feed_db;
                memcpy(resp_buf, &val, 4);
                vendor_send_response(resp_buf, 4);
                return true;
            }

            case REQ_GET_CROSSFEED_ITD: {
                resp_buf[0] = crossfeed_config.itd_enabled ? 1 : 0;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            // Volume Leveller GET commands
            case REQ_GET_LEVELLER_ENABLE: {
                resp_buf[0] = leveller_config.enabled ? 1 : 0;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_LEVELLER_AMOUNT: {
                float val = leveller_config.amount;
                memcpy(resp_buf, &val, 4);
                vendor_send_response(resp_buf, 4);
                return true;
            }

            case REQ_GET_LEVELLER_SPEED: {
                resp_buf[0] = leveller_config.speed;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_LEVELLER_MAX_GAIN: {
                float val = leveller_config.max_gain_db;
                memcpy(resp_buf, &val, 4);
                vendor_send_response(resp_buf, 4);
                return true;
            }

            case REQ_GET_LEVELLER_LOOKAHEAD: {
                resp_buf[0] = leveller_config.lookahead ? 1 : 0;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_LEVELLER_GATE: {
                float val = leveller_config.gate_threshold_db;
                memcpy(resp_buf, &val, 4);
                vendor_send_response(resp_buf, 4);
                return true;
            }

            case REQ_GET_STATUS: {
                if (setup->wValue == 9) {
                    // Combined status: all peaks + CPU load + clip flags
                    // RP2350: 26 bytes (11 peaks × 2 + 2 CPU + 2 clip)
                    // RP2040: 18 bytes (7 peaks × 2 + 2 CPU + 2 clip)
                    for (int i = 0; i < NUM_CHANNELS; i++) {
                        resp_buf[i*2]     = global_status.peaks[i] & 0xFF;
                        resp_buf[i*2 + 1] = global_status.peaks[i] >> 8;
                    }
                    resp_buf[NUM_CHANNELS * 2]     = global_status.cpu0_load;
                    resp_buf[NUM_CHANNELS * 2 + 1] = global_status.cpu1_load;
                    resp_buf[NUM_CHANNELS * 2 + 2] = global_status.clip_flags & 0xFF;
                    resp_buf[NUM_CHANNELS * 2 + 3] = global_status.clip_flags >> 8;
                    vendor_send_response(resp_buf, NUM_CHANNELS * 2 + 4);
                    return true;
                }

                uint32_t resp = 0;
                switch (setup->wValue) {
                    case 0: resp = (uint32_t)global_status.peaks[0] | ((uint32_t)global_status.peaks[1] << 16); break;
                    case 1: resp = (uint32_t)global_status.peaks[2] | ((uint32_t)global_status.peaks[3] << 16); break;
                    case 2: resp = (uint32_t)global_status.peaks[4] | ((uint32_t)global_status.cpu0_load << 16) | ((uint32_t)global_status.cpu1_load << 24); break;
                    case 3: resp = pdm_ring_overruns; break;
                    case 4: resp = pdm_ring_underruns; break;
                    case 5: resp = pdm_dma_overruns; break;
                    case 6: resp = pdm_dma_underruns; break;
                    case 7: resp = spdif_overruns; break;
                    case 8: resp = spdif_underruns; break;
                    case 10: resp = usb_audio_packets; break;
                    case 11: resp = usb_audio_alt_set; break;
                    case 12: resp = usb_audio_mounted; break;
                    case 13: resp = clock_get_hz(clk_sys); break;  // System clock frequency in Hz
                    case 14: resp = vreg_voltage_to_mv(vreg_get_voltage()); break;  // Core voltage in mV
                    case 15: resp = audio_state.freq; break;  // Sample rate in Hz
                    case 16: resp = (uint32_t)read_temperature_cdeg(); break;  // Temperature in centi-degrees C
                    case 17: resp = audio_spdif_get_dma_starvations(); break;  // Total SPDIF DMA starvations
                    case 18: resp = audio_spdif_get_dma_starvations_instance(0); break;  // SPDIF instance 0
                    case 19: resp = audio_spdif_get_dma_starvations_instance(1); break;  // SPDIF instance 1
                    case 20: resp = audio_spdif_get_dma_starvations_instance(2); break;  // SPDIF instance 2
                    case 21: resp = audio_spdif_get_dma_starvations_instance(3); break;  // SPDIF instance 3
                    case 22: resp = usb_audio_ring_overrun_count(); break;  // USB audio ring overruns
                }
                usb_start_tiny_control_in_transfer(resp, 4);
                return true;
            }

            case REQ_SAVE_PARAMS: {
                // Legacy command retained for compatibility, but deferred to
                // main loop to avoid flash write blackout in IRQ context.
                save_params_pending = true;
                __dmb();
                usb_start_tiny_control_in_transfer(FLASH_OK, 1);  // Accepted
                return true;
            }

            case REQ_LOAD_PARAMS: {
                // flash_load_params() routes through preset_load(), which
                // handles filter recalculation, delay updates, and mute
                // internally — no need to duplicate here.
                int result = flash_load_params();
                usb_start_tiny_control_in_transfer(result, 1);
                return true;
            }

            case REQ_FACTORY_RESET: {
                // Deferred to main loop — same pipeline protection as preset
                // load/save/delete: mute, Core 1 sync, delay line zero, and
                // output type switch if the live config had I2S outputs.
                factory_reset_pending = true;
                __dmb();
                usb_start_tiny_control_in_transfer(FLASH_OK, 1);
                return true;
            }

            case REQ_GET_EQ_PARAM: {
                uint8_t channel = (setup->wValue >> 8) & 0xFF;
                uint8_t band = (setup->wValue >> 4) & 0x0F;
                uint8_t param = setup->wValue & 0x0F;
                if (channel < NUM_CHANNELS && band < channel_band_counts[channel]) {
                    uint32_t val_to_send = 0;
                    EqParamPacket *p = &filter_recipes[channel][band];
                    switch (param) {
                        case 0: val_to_send = (uint32_t)p->type; break;
                        case 1: memcpy(&val_to_send, &p->freq, 4); break;
                        case 2: memcpy(&val_to_send, &p->Q, 4); break;
                        case 3: memcpy(&val_to_send, &p->gain_db, 4); break;
                    }
                    usb_start_tiny_control_in_transfer(val_to_send, 4);
                    return true;
                }
                return false;
            }

            // Matrix Mixer GET commands
            case REQ_GET_MATRIX_ROUTE: {
                // wValue = (input << 8) | output
                uint8_t input = (setup->wValue >> 8) & 0xFF;
                uint8_t output = setup->wValue & 0xFF;
                if (input < NUM_INPUT_CHANNELS && output < NUM_OUTPUT_CHANNELS) {
                    MatrixCrosspoint *xp = &matrix_mixer.crosspoints[input][output];
                    MatrixRoutePacket pkt = {
                        .input = input,
                        .output = output,
                        .enabled = xp->enabled,
                        .phase_invert = xp->phase_invert,
                        .gain_db = xp->gain_db
                    };
                    memcpy(resp_buf, &pkt, sizeof(pkt));
                    vendor_send_response(resp_buf, sizeof(pkt));
                    return true;
                }
                return false;
            }

            case REQ_GET_OUTPUT_ENABLE: {
                uint8_t out = (uint8_t)setup->wValue;
                if (out < NUM_OUTPUT_CHANNELS) {
                    resp_buf[0] = matrix_mixer.outputs[out].enabled;
                    vendor_send_response(resp_buf, 1);
                    return true;
                }
                return false;
            }

            case REQ_GET_OUTPUT_GAIN: {
                uint8_t out = (uint8_t)setup->wValue;
                if (out < NUM_OUTPUT_CHANNELS) {
                    memcpy(resp_buf, &matrix_mixer.outputs[out].gain_db, 4);
                    vendor_send_response(resp_buf, 4);
                    return true;
                }
                return false;
            }

            case REQ_GET_OUTPUT_MUTE: {
                uint8_t out = (uint8_t)setup->wValue;
                if (out < NUM_OUTPUT_CHANNELS) {
                    resp_buf[0] = matrix_mixer.outputs[out].mute;
                    vendor_send_response(resp_buf, 1);
                    return true;
                }
                return false;
            }

            case REQ_GET_OUTPUT_DELAY: {
                uint8_t out = (uint8_t)setup->wValue;
                if (out < NUM_OUTPUT_CHANNELS) {
                    memcpy(resp_buf, &matrix_mixer.outputs[out].delay_ms, 4);
                    vendor_send_response(resp_buf, 4);
                    return true;
                }
                return false;
            }

            case REQ_GET_CORE1_MODE: {
                resp_buf[0] = (uint8_t)core1_mode;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_CORE1_CONFLICT: {
                // wValue = proposed output index to enable
                // Returns 1 if enabling it would conflict, 0 if OK
                uint8_t out = (uint8_t)setup->wValue;
                uint8_t conflict = 0;
                if (out < NUM_OUTPUT_CHANNELS) {
                    bool is_pdm = (out == NUM_OUTPUT_CHANNELS - 1);
                    bool is_core1_eq = (out >= CORE1_EQ_FIRST_OUTPUT && out <= CORE1_EQ_LAST_OUTPUT);
                    if (is_pdm) {
                        for (int i = CORE1_EQ_FIRST_OUTPUT; i <= CORE1_EQ_LAST_OUTPUT; i++) {
                            if (matrix_mixer.outputs[i].enabled) { conflict = 1; break; }
                        }
                    } else if (is_core1_eq) {
                        if (matrix_mixer.outputs[NUM_OUTPUT_CHANNELS - 1].enabled) conflict = 1;
                    }
                }
                resp_buf[0] = conflict;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_SET_OUTPUT_PIN: {
                // wValue = (new_pin << 8) | output_index
                uint8_t out_idx = setup->wValue & 0xFF;
                uint8_t new_pin = (setup->wValue >> 8) & 0xFF;
                uint8_t status;

                if (out_idx >= NUM_PIN_OUTPUTS) {
                    status = PIN_CONFIG_INVALID_OUTPUT;
                } else if (!is_valid_gpio_pin(new_pin)) {
                    status = PIN_CONFIG_INVALID_PIN;
                } else if (is_pin_in_use(new_pin, out_idx)) {
                    status = PIN_CONFIG_PIN_IN_USE;
                } else if (new_pin == output_pins[out_idx]) {
                    // No-op: pin unchanged
                    status = PIN_CONFIG_SUCCESS;
                } else if (out_idx < NUM_SPDIF_INSTANCES) {
                    // Output slot: disable → change pin → re-enable
                    if (output_types[out_idx] == OUTPUT_TYPE_I2S) {
                        audio_i2s_instance_t *inst = i2s_instance_ptrs[out_idx];
                        audio_i2s_set_enabled(inst, false);
                        audio_i2s_change_data_pin(inst, new_pin);
                        audio_i2s_set_enabled(inst, true);
                    } else {
                        audio_spdif_instance_t *inst = spdif_instance_ptrs[out_idx];
                        audio_spdif_set_enabled(inst, false);
                        audio_spdif_change_pin(inst, new_pin);
                        audio_spdif_set_enabled(inst, true);
                    }
                    output_pins[out_idx] = new_pin;
                    status = PIN_CONFIG_SUCCESS;
                } else {
                    // PDM output (out_idx == 4): must be disabled first
                    if (pdm_enabled || core1_mode == CORE1_MODE_PDM) {
                        status = PIN_CONFIG_OUTPUT_ACTIVE;
                    } else {
                        pdm_change_pin(new_pin);
                        output_pins[out_idx] = new_pin;
                        status = PIN_CONFIG_SUCCESS;
                    }
                }

                resp_buf[0] = status;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_OUTPUT_PIN: {
                uint8_t out_idx = (uint8_t)setup->wValue;
                if (out_idx < NUM_PIN_OUTPUTS) {
                    resp_buf[0] = output_pins[out_idx];
                    vendor_send_response(resp_buf, 1);
                    return true;
                }
                return false;
            }

            case REQ_GET_SERIAL: {
                memcpy(resp_buf, usb_descriptor_str_serial, 16);
                vendor_send_response(resp_buf, 16);
                return true;
            }

            case REQ_GET_PLATFORM: {
                resp_buf[0] = PLATFORM_RP2040;
#if PICO_RP2350
                resp_buf[0] = PLATFORM_RP2350;
#endif
                resp_buf[1] = (uint8_t)(FW_VERSION_BCD >> 8);    // major
                resp_buf[2] = (uint8_t)(FW_VERSION_BCD & 0xFF);  // minor.patch BCD
                resp_buf[3] = NUM_OUTPUT_CHANNELS;
                vendor_send_response(resp_buf, 4);
                return true;
            }

            case REQ_CLEAR_CLIPS: {
                // Read-then-clear: return the clip flags that were set, then reset
                uint16_t flags = global_status.clip_flags;
                global_status.clip_flags = 0;
                usb_start_tiny_control_in_transfer(flags, 2);
                return true;
            }

            // --- Preset Commands ---

            case REQ_PRESET_SAVE: {
                // Deferred to main loop: flash writes must not run in IRQ
                // context (45ms interrupt blackout per sector).  The main loop
                // brackets save with prepare/complete_pipeline_reset() so audio
                // resumes from a fully resynced output pipeline after blackout.
                // Response is fire-and-forget: "accepted".
                uint8_t slot = (uint8_t)setup->wValue;
                if (slot >= PRESET_SLOTS) {
                    resp_buf[0] = PRESET_ERR_INVALID_SLOT;
                } else {
                    pending_preset_save_slot = slot;
                    preset_save_pending = true;
                    __dmb();
                    resp_buf[0] = PRESET_OK;
                }
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_PRESET_LOAD: {
                // Deferred to main loop so that:
                //  - Flash reads and dir_flush don't run in IRQ context
                //  - The operation is bracketed with prepare/complete_pipeline_reset
                //    to drain stale consumer buffers and resync outputs
                //  - Delay lines are zeroed to prevent old audio bleed-through
                // Response is fire-and-forget: "accepted".  The host can poll
                // REQ_PRESET_GET_ACTIVE to confirm the load completed.
                uint8_t slot = (uint8_t)setup->wValue;
                if (slot >= PRESET_SLOTS) {
                    resp_buf[0] = PRESET_ERR_INVALID_SLOT;
                } else {
                    pending_preset_load_slot = slot;
                    preset_load_pending = true;
                    __dmb();
                    resp_buf[0] = PRESET_OK;
                }
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_PRESET_DELETE: {
                // Deferred to main loop: flash erase runs with interrupts
                // disabled for ~45ms.  If the deleted slot is the active slot,
                // factory defaults are applied — which needs pipeline reset
                // to flush stale buffers processed with the old parameters.
                uint8_t slot = (uint8_t)setup->wValue;
                if (slot >= PRESET_SLOTS) {
                    resp_buf[0] = PRESET_ERR_INVALID_SLOT;
                } else {
                    preset_delete_mask |= (1u << slot);
                    __dmb();
                    resp_buf[0] = PRESET_OK;
                }
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_PRESET_GET_NAME: {
                // wValue = slot index.  Returns 32-byte NUL-terminated name.
                uint8_t slot = (uint8_t)setup->wValue;
                char name[PRESET_NAME_LEN];
                uint8_t result = preset_get_name(slot, name);
                if (result == PRESET_OK) {
                    memcpy(resp_buf, name, PRESET_NAME_LEN);
                    vendor_send_response(resp_buf, PRESET_NAME_LEN);
                    return true;
                }
                return false;
            }

            case REQ_PRESET_GET_DIR: {
                // Returns 7-byte directory summary:
                //   [0-1] slot_occupied bitmask (little-endian u16)
                //   [2]   startup_mode
                //   [3]   default_slot
                //   [4]   last_active_slot
                //   [5]   include_pins
                //   [6]   include_master_volume  (V12+, old apps request 6 bytes)
                uint16_t occupied;
                uint8_t mode, def_slot, last_active, inc_pins, inc_master_vol;
                preset_get_directory(&occupied, &mode, &def_slot,
                                     &last_active, &inc_pins, &inc_master_vol);
                resp_buf[0] = occupied & 0xFF;
                resp_buf[1] = occupied >> 8;
                resp_buf[2] = mode;
                resp_buf[3] = def_slot;
                resp_buf[4] = last_active;
                resp_buf[5] = inc_pins;
                resp_buf[6] = inc_master_vol;
                vendor_send_response(resp_buf, 7);
                return true;
            }

            case REQ_PRESET_GET_STARTUP: {
                // Returns 3 bytes: startup_mode, default_slot, last_active
                uint16_t occupied;
                uint8_t mode, def_slot, last_active, inc_pins, inc_master_vol;
                preset_get_directory(&occupied, &mode, &def_slot,
                                     &last_active, &inc_pins, &inc_master_vol);
                resp_buf[0] = mode;
                resp_buf[1] = def_slot;
                resp_buf[2] = last_active;
                vendor_send_response(resp_buf, 3);
                return true;
            }

            case REQ_PRESET_GET_INCLUDE_PINS: {
                uint16_t occupied;
                uint8_t mode, def_slot, last_active, inc_pins, inc_master_vol;
                preset_get_directory(&occupied, &mode, &def_slot,
                                     &last_active, &inc_pins, &inc_master_vol);
                resp_buf[0] = inc_pins;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_INCLUDE_MASTER_VOL: {
                // Returns whether preset load restores master volume (0/1).
                uint16_t occupied;
                uint8_t mode, def_slot, last_active, inc_pins, inc_master_vol;
                preset_get_directory(&occupied, &mode, &def_slot,
                                     &last_active, &inc_pins, &inc_master_vol);
                resp_buf[0] = inc_master_vol;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_PRESET_GET_ACTIVE: {
                resp_buf[0] = preset_get_active();
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_CHANNEL_NAME: {
                uint8_t ch = setup->wValue & 0xFF;
                if (ch < NUM_CHANNELS) {
                    vendor_send_response(channel_names[ch], PRESET_NAME_LEN);
                    return true;
                }
                return false;
            }

            case REQ_GET_ALL_PARAMS: {
                bulk_params_collect((WireBulkParams *)bulk_param_buf);
                uint16_t len = sizeof(WireBulkParams);
                if (setup->wLength < len) len = setup->wLength;
                // tud_control_xfer handles EP0 chunking (including trailing ZLP
                // on exact-multiple-of-64 transfers) internally.
                return tud_control_xfer(_vendor_rhport, _vendor_current_req,
                                         bulk_param_buf, len);
            }

            case REQ_GET_BUFFER_STATS: {
                BufferStatsPacket pkt;
                memset(&pkt, 0, sizeof(pkt));
                pkt.num_spdif = NUM_SPDIF_INSTANCES;
                pkt.flags = (pdm_enabled ? 0x01 : 0) | (sync_started ? 0x02 : 0);
                pkt.sequence = buffer_stats_sequence++;

                uint consumer_capacity = SPDIF_CONSUMER_BUFFER_COUNT;

                for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
                    uint cons_free, cons_prepared, playing;
                    get_slot_consumer_stats(i, &cons_free, &cons_prepared, &playing);
                    pkt.spdif[i].consumer_free = (uint8_t)cons_free;
                    pkt.spdif[i].consumer_prepared = (uint8_t)cons_prepared;
                    pkt.spdif[i].consumer_playing = (uint8_t)playing;
                    // Fill % is based on total occupied buffers (capacity - free), so it
                    // includes hidden staging buffers (e.g., I2S partial-assembly buffer).
                    uint cons_fill = (cons_free > consumer_capacity) ? 0 : (consumer_capacity - cons_free);
                    pkt.spdif[i].consumer_fill_pct = (uint8_t)(cons_fill * 100 / consumer_capacity);
                    pkt.spdif[i].consumer_min_fill_pct = spdif_consumer_min_fill_pct[i];
                    pkt.spdif[i].consumer_max_fill_pct = spdif_consumer_max_fill_pct[i];
                }

                if (pdm_enabled) {
                    pkt.pdm.dma_fill_pct = pdm_get_dma_fill_pct();
                    pkt.pdm.dma_min_fill_pct = pdm_dma_min_fill_pct;
                    pkt.pdm.dma_max_fill_pct = pdm_dma_max_fill_pct;
                    pkt.pdm.ring_fill_pct = pdm_get_ring_fill_pct();
                    pkt.pdm.ring_min_fill_pct = pdm_ring_min_fill_pct;
                    pkt.pdm.ring_max_fill_pct = pdm_ring_max_fill_pct;
                }

                memcpy(resp_buf, &pkt, sizeof(pkt));
                vendor_send_response(resp_buf, sizeof(pkt));
                return true;
            }

            case REQ_RESET_BUFFER_STATS: {
                uint16_t flags = setup->wValue;
                if (flags & 0x01) {
                    reset_buffer_watermarks();
                }
                resp_buf[0] = 1;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_USB_ERROR_STATS: {
                // TinyUSB does not expose per-category USB error counters.
                // Return all zeros so the host app keeps working; re-plumb if
                // a future TinyUSB version surfaces these via dcd_event or
                // equivalent.
                typedef struct __attribute__((packed)) {
                    uint32_t total;
                    uint32_t crc;
                    uint32_t bitstuff;
                    uint32_t rx_overflow;
                    uint32_t rx_timeout;
                    uint32_t data_seq;
                } UsbErrorStatsPacket;

                UsbErrorStatsPacket pkt = {0};
                memcpy(resp_buf, &pkt, sizeof(pkt));
                vendor_send_response(resp_buf, sizeof(pkt));
                return true;
            }

            case REQ_RESET_USB_ERROR_STATS: {
                // No-op under TinyUSB (see REQ_GET_USB_ERROR_STATS).
                resp_buf[0] = 1;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            // ----------------------------------------------------------------
            // System Commands (0xF0+)
            // ----------------------------------------------------------------

            case REQ_ENTER_BOOTLOADER: {
                // Send response before rebooting so the host sees success
                resp_buf[0] = 1;
                vendor_send_response(resp_buf, 1);
                // Brief delay to let the USB response complete
                busy_wait_ms(100);
                reset_usb_boot(0, 0);
                // Never returns
            }

            // ----------------------------------------------------------------
            // I2S / MCK Configuration Commands (0xC0-0xC9)
            // ----------------------------------------------------------------

            case REQ_SET_OUTPUT_TYPE: {
                // wValue = (new_type << 8) | slot_index
                //
                // Type switching is DEFERRED to the main loop because it involves
                // heap allocation (consumer pool creation) which cannot safely run
                // in USB ISR context (malloc uses a spin lock that can deadlock if
                // the main loop is also in malloc).
                //
                uint8_t slot = setup->wValue & 0xFF;
                uint8_t new_type = (setup->wValue >> 8) & 0xFF;
                uint8_t status;

                if (slot >= NUM_SPDIF_INSTANCES) {
                    status = PIN_CONFIG_INVALID_OUTPUT;
                } else if (new_type > 1) {
                    status = PIN_CONFIG_INVALID_PIN;
                } else if (new_type == output_types[slot]) {
                    status = PIN_CONFIG_SUCCESS;  // No-op
                } else {
                    // Defer to main loop — per-slot bitmask supports
                    // back-to-back requests without dropping any
                    extern volatile uint8_t output_type_change_mask;
                    extern volatile uint8_t pending_output_types[];
                    pending_output_types[slot] = new_type;
                    __dmb();
                    output_type_change_mask |= (1u << slot);
                    status = PIN_CONFIG_SUCCESS;
                }

                resp_buf[0] = status;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_OUTPUT_TYPE: {
                uint8_t slot = (uint8_t)setup->wValue;
                if (slot < NUM_SPDIF_INSTANCES) {
                    resp_buf[0] = output_types[slot];
                    vendor_send_response(resp_buf, 1);
                    return true;
                }
                return false;
            }

            case REQ_SET_I2S_BCK_PIN: {
                uint8_t new_pin = (uint8_t)setup->wValue;
                uint8_t status;

                if (!is_valid_gpio_pin(new_pin) || !is_valid_gpio_pin(new_pin + 1)) {
                    status = PIN_CONFIG_INVALID_PIN;
                } else if (new_pin == i2s_bck_pin) {
                    status = PIN_CONFIG_SUCCESS;  // No-op
                } else {
                    // Reject if any slot is currently I2S
                    bool any_i2s = false;
                    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
                        if (output_types[i] == OUTPUT_TYPE_I2S) { any_i2s = true; break; }
                    }
                    if (any_i2s) {
                        status = PIN_CONFIG_OUTPUT_ACTIVE;
                    } else if (is_pin_in_use(new_pin, 0xFF) || is_pin_in_use(new_pin + 1, 0xFF)) {
                        status = PIN_CONFIG_PIN_IN_USE;
                    } else {
                        i2s_bck_pin = new_pin;
                        status = PIN_CONFIG_SUCCESS;
                    }
                }
                resp_buf[0] = status;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_I2S_BCK_PIN: {
                resp_buf[0] = i2s_bck_pin;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_SET_MCK_ENABLE: {
                bool enable = (setup->wValue != 0);
                if (enable && !i2s_mck_enabled) {
                    sanitize_mck_multiplier_for_rate(audio_state.freq);
                    audio_i2s_mck_update_frequency(audio_state.freq, i2s_mck_multiplier);
                    audio_i2s_mck_set_enabled(true);
                    i2s_mck_enabled = true;
                } else if (!enable && i2s_mck_enabled) {
                    audio_i2s_mck_set_enabled(false);
                    i2s_mck_enabled = false;
                }
                resp_buf[0] = PIN_CONFIG_SUCCESS;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_MCK_ENABLE: {
                resp_buf[0] = i2s_mck_enabled ? 1 : 0;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_SET_MCK_PIN: {
                uint8_t new_pin = (uint8_t)setup->wValue;
                uint8_t status;
                if (!is_valid_gpio_pin(new_pin)) {
                    status = PIN_CONFIG_INVALID_PIN;
                } else if (i2s_mck_enabled) {
                    status = PIN_CONFIG_OUTPUT_ACTIVE;
                } else if (is_pin_in_use(new_pin, 0xFF)) {
                    status = PIN_CONFIG_PIN_IN_USE;
                } else if (new_pin == i2s_mck_pin) {
                    status = PIN_CONFIG_SUCCESS;
                } else {
                    audio_i2s_mck_change_pin(new_pin);
                    i2s_mck_pin = new_pin;
                    status = PIN_CONFIG_SUCCESS;
                }
                resp_buf[0] = status;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_MCK_PIN: {
                resp_buf[0] = i2s_mck_pin;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_SET_MCK_MULTIPLIER: {
                // Wire encoding: 0 = 128x, 1 = 256x
                uint16_t raw = setup->wValue;
                if (raw > 1) { resp_buf[0] = PIN_CONFIG_INVALID_PIN; vendor_send_response(resp_buf, 1); return true; }
                uint16_t mult = (raw == 1) ? 256 : 128;

                if (!is_mck_multiplier_supported_for_rate(mult, audio_state.freq)) {
                    printf("Rejected MCK %ux at %lu Hz (unsupported)\n",
                           (unsigned)mult, (unsigned long)audio_state.freq);
                    resp_buf[0] = PIN_CONFIG_INVALID_PIN;
                    vendor_send_response(resp_buf, 1);
                    return true;
                }
                i2s_mck_multiplier = mult;
                if (i2s_mck_enabled) {
                    audio_i2s_mck_update_frequency(audio_state.freq, i2s_mck_multiplier);
                }
                resp_buf[0] = PIN_CONFIG_SUCCESS;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_MCK_MULTIPLIER: {
                sanitize_mck_multiplier_for_rate(audio_state.freq);
                resp_buf[0] = mck_encode(i2s_mck_multiplier);
                vendor_send_response(resp_buf, 1);
                return true;
            }

            // ---- Audio Input Source Commands ----

            case REQ_GET_INPUT_SOURCE: {
                resp_buf[0] = active_input_source;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_SET_SPDIF_RX_PIN: {
                // Immediate-response SET (same pattern as SET_I2S_BCK_PIN).
                // wValue = new GPIO pin number.
                uint8_t new_pin = (uint8_t)setup->wValue;
                uint8_t status;
                if (!is_valid_gpio_pin(new_pin)) {
                    status = PIN_CONFIG_INVALID_PIN;
                } else if (new_pin == spdif_rx_pin) {
                    status = PIN_CONFIG_SUCCESS;  // No-op
                } else if (active_input_source == INPUT_SOURCE_SPDIF) {
                    status = PIN_CONFIG_OUTPUT_ACTIVE;  // Can't change while RX active
                } else if (is_pin_in_use(new_pin, 0xFF)) {
                    status = PIN_CONFIG_PIN_IN_USE;
                } else {
                    spdif_rx_pin = new_pin;
                    flash_set_spdif_rx_pin_pending = true;
                    status = PIN_CONFIG_SUCCESS;
                }
                resp_buf[0] = status;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_SPDIF_RX_PIN: {
                resp_buf[0] = spdif_rx_pin;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_SPDIF_RX_STATUS: {
                SpdifRxStatusPacket status;
                spdif_input_get_status(&status);
                vendor_send_response(&status, sizeof(status));
                return true;
            }

            case REQ_GET_SPDIF_RX_CH_STATUS: {
                uint8_t ch_status[24];
                spdif_input_get_channel_status(ch_status);
                vendor_send_response(ch_status, 24);
                return true;
            }
        }

        return false;
    }
}

// ----------------------------------------------------------------------------
// PUBLIC ENTRY POINT — overrides TinyUSB's weak tud_vendor_control_xfer_cb
// (usbd.c:82).  TinyUSB calls this directly for every vendor-type request
// (usbd.c:727-730), bypassing class drivers.  Dispatches SETUP/DATA/ACK.
// ----------------------------------------------------------------------------

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                tusb_control_request_t const *req) {
    // Stash rhport + request so vendor_send_response() can complete the xfer
    // without every case body having to plumb them through.
    _vendor_rhport       = rhport;
    _vendor_current_req  = req;

    if (stage == CONTROL_STAGE_SETUP) {
        if (req->bmRequestType_bit.direction == TUSB_DIR_IN) {
            // GET path — dispatches into the legacy switch.
            return vendor_handle_get(req);
        }

        // SET path — schedule DATA stage receive.
        vendor_last_request = req->bRequest;
        vendor_last_wValue  = req->wValue;
        vendor_last_wLength = req->wLength;

        if (req->bRequest == REQ_SET_ALL_PARAMS &&
            req->wLength == sizeof(WireBulkParams)) {
            // Large bulk SET — tud_control_xfer handles EP0 chunking.
            return tud_control_xfer(rhport, req, bulk_param_buf, req->wLength);
        }

        if (req->wLength == 0) {
            // Zero-length SET — no DATA stage; ACK immediately.
            return tud_control_status(rhport, req);
        }

        if (req->wLength <= sizeof(vendor_rx_buf)) {
            return tud_control_xfer(rhport, req, vendor_rx_buf, req->wLength);
        }

        // Oversized SET we can't handle — STALL.
        return false;
    }

    if (stage == CONTROL_STAGE_DATA) {
        // SET data received — run the legacy command dispatcher.
        if (req->bmRequestType_bit.direction == TUSB_DIR_OUT) {
            vendor_handle_set_data(req);
        }
        return true;
    }

    if (stage == CONTROL_STAGE_ACK) {
        // Status-stage completed.  Signal the main loop to apply bulk SET.
        if (req->bmRequestType_bit.direction == TUSB_DIR_OUT &&
            req->bRequest == REQ_SET_ALL_PARAMS) {
            bulk_params_pending = true;
        }
    }
    return true;
}
