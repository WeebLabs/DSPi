/*
 * USB Audio Implementation for DSPi
 * UAC1 Audio Streaming with DSP Pipeline
 * Uses pico-extras usb_device library with LUFA descriptor types
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/usb_device.h"
#include "pico/usb_device_private.h"
#include "pico/audio.h"
#include "pico/audio_spdif.h"
#include "hardware/sync.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/vreg.h"

#include "usb_audio.h"
#include "usb_descriptors.h"
#include "dsp_pipeline.h"
#include "dcp_inline.h"
#include "pdm_generator.h"
#include "flash_storage.h"
#include "loudness.h"
#include "crossfeed.h"

// ----------------------------------------------------------------------------
// GLOBALS
// ----------------------------------------------------------------------------

volatile AudioState audio_state = { .freq = 44100 };
volatile bool bypass_master_eq = false;
volatile SystemStatusPacket global_status = {0};

volatile bool eq_update_pending = false;
volatile EqParamPacket pending_packet;
volatile bool rate_change_pending = false;
volatile uint32_t pending_rate = 48000;

volatile float global_preamp_db = 0.0f;
volatile int32_t global_preamp_mul = 268435456;
volatile float global_preamp_linear = 1.0f;

// Per-channel gain and mute (output channels: L=0, R=1, Sub=2)
volatile float channel_gain_db[3] = {0.0f, 0.0f, 0.0f};
volatile int32_t channel_gain_mul[3] = {32768, 32768, 32768};  // Unity = 2^15
volatile float channel_gain_linear[3] = {1.0f, 1.0f, 1.0f};
volatile bool channel_mute[3] = {false, false, false};

// Loudness compensation state
volatile bool loudness_enabled = false;
volatile float loudness_ref_spl = 83.0f;
volatile float loudness_intensity_pct = 100.0f;
volatile bool loudness_recompute_pending = false;

static Biquad loudness_biquads[2][LOUDNESS_BIQUAD_COUNT];  // [0]=Left, [1]=Right
static const LoudnessCoeffs *current_loudness_coeffs = NULL;

// Crossfeed state
volatile CrossfeedConfig crossfeed_config = {
    .enabled = false,
    .itd_enabled = true,
    .preset = CROSSFEED_PRESET_DEFAULT,
    .custom_fc = 700.0f,
    .custom_feed_db = 4.5f
};
volatile bool crossfeed_update_pending = false;
volatile bool crossfeed_bypassed = true;  // Fast bypass flag for audio callback
CrossfeedState crossfeed_state;

// Sync State
volatile uint64_t total_samples_produced = 0;
volatile uint64_t start_time_us = 0;
volatile bool sync_started = false;
static volatile uint64_t last_packet_time_us = 0;
#define AUDIO_GAP_THRESHOLD_US 50000  // 50ms - reset sync if packets stop this long

// Audio Pool
struct audio_buffer_pool *producer_pool = NULL;
struct audio_format audio_format_48k = { .format = AUDIO_BUFFER_FORMAT_PCM_S16, .sample_freq = 48000, .channel_count = 2 };

// ----------------------------------------------------------------------------
// USB INTERFACE / ENDPOINT OBJECTS
// ----------------------------------------------------------------------------

static struct usb_interface ac_interface;
static struct usb_interface as_op_interface;
static struct usb_interface vendor_interface;
static struct usb_endpoint ep_op_out, ep_op_sync;

// ----------------------------------------------------------------------------
// SYSTEM STATISTICS HELPERS
// ----------------------------------------------------------------------------

// Convert vreg voltage enum to millivolts
static uint16_t vreg_voltage_to_mv(enum vreg_voltage voltage) {
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
static int16_t read_temperature_cdeg(void) {
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
// VOLUME
// ----------------------------------------------------------------------------
#define CENTER_VOLUME_INDEX 91
static uint16_t db_to_vol[91] = {
    0x0001, 0x0001, 0x0001, 0x0001, 0x0001, 0x0001, 0x0002, 0x0002, 0x0002, 0x0002, 0x0003, 0x0003, 0x0004, 0x0004, 0x0005, 0x0005,
    0x0006, 0x0007, 0x0008, 0x0009, 0x000a, 0x000b, 0x000d, 0x000e, 0x0010, 0x0012, 0x0014, 0x0017, 0x001a, 0x001d, 0x0020, 0x0024,
    0x0029, 0x002e, 0x0033, 0x003a, 0x0041, 0x0049, 0x0052, 0x005c, 0x0067, 0x0074, 0x0082, 0x0092, 0x00a4, 0x00b8, 0x00ce, 0x00e7,
    0x0104, 0x0124, 0x0147, 0x016f, 0x019c, 0x01ce, 0x0207, 0x0246, 0x028d, 0x02dd, 0x0337, 0x039b, 0x040c, 0x048a, 0x0518, 0x05b7,
    0x066a, 0x0732, 0x0813, 0x090f, 0x0a2a, 0x0b68, 0x0ccc, 0x0e5c, 0x101d, 0x1214, 0x1449, 0x16c3, 0x198a, 0x1ca7, 0x2026, 0x2413,
    0x287a, 0x2d6a, 0x32f5, 0x392c, 0x4026, 0x47fa, 0x50c3, 0x5a9d, 0x65ac, 0x7214, 0x7fff
};

#define ENCODE_DB(x) ((int16_t)((x)*256))
#define MIN_VOLUME           ENCODE_DB(-CENTER_VOLUME_INDEX)
#define DEFAULT_VOLUME       ENCODE_DB(0)
#define MAX_VOLUME           ENCODE_DB(0)
#define VOLUME_RESOLUTION    ENCODE_DB(1)

void audio_set_volume(int16_t volume) {
    audio_state.volume = volume;
    volume += CENTER_VOLUME_INDEX * 256;
    if (volume < 0) volume = 0;
    if (volume >= 91 * 256) volume = 91 * 256 - 1;
    uint8_t vol_index = ((uint16_t)volume) >> 8u;
    audio_state.vol_mul = db_to_vol[vol_index];

    // Update loudness compensation coefficients for this volume step
    if (loudness_enabled && loudness_active_table) {
        current_loudness_coeffs = loudness_active_table[vol_index];
    }
}

// ----------------------------------------------------------------------------
// AUDIO PROCESSING (called from USB audio packet callback)
// ----------------------------------------------------------------------------

static void __not_in_flash_func(process_audio_packet)(const uint8_t *data, uint16_t data_len) {
    uint32_t start_time = time_us_32();

    // Detect SPDIF underrun: USB packets should arrive every ~1ms
    static uint32_t last_packet_time = 0;
    if (last_packet_time > 0) {
        uint32_t gap = start_time - last_packet_time;
        if (gap > 2000 && gap < 50000) {  // 2ms-50ms gap = underrun
            spdif_underruns++;
        }
    }
    last_packet_time = start_time;

    struct audio_buffer* audio_buffer = NULL;
    if (producer_pool) audio_buffer = take_audio_buffer(producer_pool, false);

    uint32_t sample_count = data_len / 4;  // 2 channels * 2 bytes per sample

    if (audio_buffer) {
        audio_buffer->sample_count = sample_count;
    } else {
        spdif_overruns++;
    }

    uint64_t now_us = time_us_64();

    // Detect audio restart after gap - reset sync state and pre-fill pool
    if (sync_started && last_packet_time_us > 0 &&
        (now_us - last_packet_time_us) > AUDIO_GAP_THRESHOLD_US) {
        sync_started = false;
        total_samples_produced = 0;

        // Pre-fill with 2 silent buffers to prevent underrun on restart
        for (int i = 0; i < 2; i++) {
            struct audio_buffer *sb = take_audio_buffer(producer_pool, false);
            if (sb) {
                int16_t *out = (int16_t *)sb->buffer->bytes;
                for (uint32_t j = 0; j < 192; j++) {
                    out[j * 2] = 0;
                    out[j * 2 + 1] = 0;
                }
                sb->sample_count = 192;
                give_audio_buffer(producer_pool, sb);
            }
        }
    }
    last_packet_time_us = now_us;

    if (!sync_started) {
        start_time_us = now_us;
        sync_started = true;
    }
    total_samples_produced += sample_count;

    const int16_t *in = (const int16_t *)data;

#if PICO_RP2350
    // ------------------------------------------------------------------------
    // RP2350 FLOAT PIPELINE (Phase 3)
    // ------------------------------------------------------------------------
    const float inv_32768 = 1.0f / 32768.0f;  // Multiply instead of divide

    float vol_mul = (float)audio_state.vol_mul * inv_32768;
    float preamp = global_preamp_linear;
    bool is_bypassed = bypass_master_eq;

    float gain_l = channel_gain_linear[0];
    float gain_r = channel_gain_linear[1];
    float gain_sub = channel_gain_linear[2];
    bool mute_l = channel_mute[0];
    bool mute_r = channel_mute[1];
    bool mute_sub = channel_mute[2];

    // Snapshot loudness state for this packet
    bool loud_on = loudness_enabled;
    const LoudnessCoeffs *loud_coeffs = current_loudness_coeffs;

    float peak_ml = 0, peak_mr = 0, peak_ol = 0, peak_or = 0, peak_sub = 0;

    for (uint32_t i = 0; i < sample_count; i++) {
        // Input: 16-bit PCM -> Float Normalized (-1.0 to 1.0)
        float raw_left = (float)in[i*2] * inv_32768;
        float raw_right = (float)in[i*2+1] * inv_32768;

        // Preamp
        raw_left *= preamp;
        raw_right *= preamp;

        // Loudness compensation (after preamp, before master EQ)
        if (loud_on && loud_coeffs) {
            for (int j = 0; j < LOUDNESS_BIQUAD_COUNT; j++) {
                const LoudnessCoeffs *lc = &loud_coeffs[j];
                if (lc->bypass) continue;
                Biquad *bq = &loudness_biquads[0][j];
                double rd = dcp_dadd(dcp_f2d(lc->b0 * raw_left), bq->s1);
                float rf = dcp_d2f(rd);
                float v1 = lc->b1 * raw_left - lc->a1 * rf;
                bq->s1 = dcp_dadd(dcp_f2d(v1), bq->s2);
                bq->s2 = dcp_f2d(lc->b2 * raw_left - lc->a2 * rf);
                raw_left = rf;
            }
            for (int j = 0; j < LOUDNESS_BIQUAD_COUNT; j++) {
                const LoudnessCoeffs *lc = &loud_coeffs[j];
                if (lc->bypass) continue;
                Biquad *bq = &loudness_biquads[1][j];
                double rd = dcp_dadd(dcp_f2d(lc->b0 * raw_right), bq->s1);
                float rf = dcp_d2f(rd);
                float v1 = lc->b1 * raw_right - lc->a1 * rf;
                bq->s1 = dcp_dadd(dcp_f2d(v1), bq->s2);
                bq->s2 = dcp_f2d(lc->b2 * raw_right - lc->a2 * rf);
                raw_right = rf;
            }
        }

        // Master EQ
        float master_l, master_r;
        if (is_bypassed) {
            master_l = raw_left;
            master_r = raw_right;
        } else {
            if (audio_buffer) {
                master_l = channel_bypassed[CH_MASTER_LEFT] ? raw_left :
                           dsp_process_channel(filters[CH_MASTER_LEFT], raw_left, CH_MASTER_LEFT);
                master_r = channel_bypassed[CH_MASTER_RIGHT] ? raw_right :
                           dsp_process_channel(filters[CH_MASTER_RIGHT], raw_right, CH_MASTER_RIGHT);
            } else {
                master_l = 0.0f; master_r = 0.0f;
            }
        }

        float abs_ml = fabsf(master_l); if (abs_ml > peak_ml) peak_ml = abs_ml;
        float abs_mr = fabsf(master_r); if (abs_mr > peak_mr) peak_mr = abs_mr;

        // Crossfeed (after Master EQ, before Output EQ)
        if (!crossfeed_bypassed) {
            crossfeed_process_stereo(&crossfeed_state, &master_l, &master_r);
        }

        // Subwoofer Mix
        float sub_in = (master_l + master_r) * 0.5f;
        float out_l = 0.0f, out_r = 0.0f, out_sub = 0.0f;

        if (audio_buffer) {
            out_l = channel_bypassed[CH_OUT_LEFT] ? master_l :
                    dsp_process_channel(filters[CH_OUT_LEFT], master_l, CH_OUT_LEFT);
            out_r = channel_bypassed[CH_OUT_RIGHT] ? master_r :
                    dsp_process_channel(filters[CH_OUT_RIGHT], master_r, CH_OUT_RIGHT);
        }
#if ENABLE_SUB
        out_sub = channel_bypassed[CH_OUT_SUB] ? sub_in :
                  dsp_process_channel(filters[CH_OUT_SUB], sub_in, CH_OUT_SUB);
#endif

        // Per-channel Gain & Mute
        out_l   = mute_l ? 0.0f : (out_l * gain_l);
        out_r   = mute_r ? 0.0f : (out_r * gain_r);
        out_sub = mute_sub ? 0.0f : (out_sub * gain_sub);

        float abs_ol = fabsf(out_l); if (abs_ol > peak_ol) peak_ol = abs_ol;
        float abs_or = fabsf(out_r); if (abs_or > peak_or) peak_or = abs_or;
        float abs_sub = fabsf(out_sub); if (abs_sub > peak_sub) peak_sub = abs_sub;

        // Master Volume
        out_l   *= vol_mul;
        out_r   *= vol_mul;
        out_sub *= vol_mul;

        // Delay
        delay_lines[0][delay_write_idx] = out_l;
        delay_lines[1][delay_write_idx] = out_r;
        delay_lines[2][delay_write_idx] = out_sub;

        float delayed_l   = delay_lines[0][(delay_write_idx - channel_delay_samples[0]) & MAX_DELAY_MASK];
        float delayed_r   = delay_lines[1][(delay_write_idx - channel_delay_samples[1]) & MAX_DELAY_MASK];
        float delayed_sub = delay_lines[2][(delay_write_idx - channel_delay_samples[2]) & MAX_DELAY_MASK];

        delay_write_idx = (delay_write_idx + 1) & MAX_DELAY_MASK;

        // Output: Float -> 16-bit PCM (S/PDIF)
        if (audio_buffer) {
            int16_t *out = (int16_t *) audio_buffer->buffer->bytes;
            // Hard clip to [-1.0, 1.0] then scale
            float dl = fmaxf(-1.0f, fminf(1.0f, delayed_l));
            float dr = fmaxf(-1.0f, fminf(1.0f, delayed_r));
            out[i*2]     = (int16_t)(dl * 32767.0f);
            out[i*2+1]   = (int16_t)(dr * 32767.0f);
        }

#if ENABLE_SUB
        // PDM expects int32 Q28-like range
        int32_t pdm_sample_q28 = (int32_t)(delayed_sub * (float)(1<<28));
        pdm_push_sample(pdm_sample_q28, false);
#endif
    }

    // Convert peaks 0.0-1.0 to Q15-ish uint16 for status report
    global_status.peaks[0] = (uint16_t)(fminf(1.0f, peak_ml) * 32767.0f);
    global_status.peaks[1] = (uint16_t)(fminf(1.0f, peak_mr) * 32767.0f);
    global_status.peaks[2] = (uint16_t)(fminf(1.0f, peak_ol) * 32767.0f);
    global_status.peaks[3] = (uint16_t)(fminf(1.0f, peak_or) * 32767.0f);
    global_status.peaks[4] = (uint16_t)(fminf(1.0f, peak_sub) * 32767.0f);

#else
    // ------------------------------------------------------------------------
    // RP2040 FIXED-POINT PIPELINE (Legacy)
    // ------------------------------------------------------------------------
    int32_t vol_mul = audio_state.vol_mul;
    int32_t preamp = global_preamp_mul;
    bool is_bypassed = bypass_master_eq;

    // Snapshot loudness state for this packet
    bool loud_on = loudness_enabled;
    const LoudnessCoeffs *loud_coeffs = current_loudness_coeffs;

    int32_t peak_ml = 0, peak_mr = 0, peak_ol = 0, peak_or = 0, peak_sub = 0;

    for (uint32_t i = 0; i < sample_count; i++) {
        int16_t raw_left_16 = in[i*2];
        int16_t raw_right_16 = in[i*2+1];

        int32_t raw_left_32 = (int32_t)raw_left_16 << 14;
        int32_t raw_right_32 = (int32_t)raw_right_16 << 14;

        raw_left_32  = clip_s64_to_s32(((int64_t)raw_left_32 * preamp) >> 28);
        raw_right_32 = clip_s64_to_s32(((int64_t)raw_right_32 * preamp) >> 28);

        // Loudness compensation (after preamp, before master EQ)
        if (loud_on && loud_coeffs) {
            for (int j = 0; j < LOUDNESS_BIQUAD_COUNT; j++) {
                const LoudnessCoeffs *lc = &loud_coeffs[j];
                if (lc->bypass) continue;
                Biquad *bq = &loudness_biquads[0][j];
                int32_t result = (int32_t)(((int64_t)lc->b0 * raw_left_32) >> 28) + bq->s1;
                bq->s1 = (int32_t)(((int64_t)lc->b1 * raw_left_32) >> 28)
                        - (int32_t)(((int64_t)lc->a1 * result) >> 28) + bq->s2;
                bq->s2 = (int32_t)(((int64_t)lc->b2 * raw_left_32) >> 28)
                        - (int32_t)(((int64_t)lc->a2 * result) >> 28);
                raw_left_32 = result;
            }
            for (int j = 0; j < LOUDNESS_BIQUAD_COUNT; j++) {
                const LoudnessCoeffs *lc = &loud_coeffs[j];
                if (lc->bypass) continue;
                Biquad *bq = &loudness_biquads[1][j];
                int32_t result = (int32_t)(((int64_t)lc->b0 * raw_right_32) >> 28) + bq->s1;
                bq->s1 = (int32_t)(((int64_t)lc->b1 * raw_right_32) >> 28)
                        - (int32_t)(((int64_t)lc->a1 * result) >> 28) + bq->s2;
                bq->s2 = (int32_t)(((int64_t)lc->b2 * raw_right_32) >> 28)
                        - (int32_t)(((int64_t)lc->a2 * result) >> 28);
                raw_right_32 = result;
            }
        }

        int32_t master_l_32, master_r_32;
        if (is_bypassed) {
            master_l_32 = raw_left_32;
            master_r_32 = raw_right_32;
        } else {
            if (audio_buffer) {
                master_l_32 = channel_bypassed[CH_MASTER_LEFT] ? raw_left_32 :
                              dsp_process_channel(filters[CH_MASTER_LEFT], raw_left_32, CH_MASTER_LEFT);
                master_r_32 = channel_bypassed[CH_MASTER_RIGHT] ? raw_right_32 :
                              dsp_process_channel(filters[CH_MASTER_RIGHT], raw_right_32, CH_MASTER_RIGHT);
            } else {
                master_l_32 = 0; master_r_32 = 0;
            }
        }

        if (abs(master_l_32) > peak_ml) peak_ml = abs(master_l_32);
        if (abs(master_r_32) > peak_mr) peak_mr = abs(master_r_32);

        // Crossfeed (after Master EQ, before Output EQ)
        if (!crossfeed_bypassed) {
            crossfeed_process_stereo(&crossfeed_state, &master_l_32, &master_r_32);
        }

        int32_t sub_in_32 = (master_l_32 + master_r_32) >> 1;
        int32_t out_l_32 = 0, out_r_32 = 0, out_sub_32 = 0;

        if (audio_buffer) {
            out_l_32 = channel_bypassed[CH_OUT_LEFT] ? master_l_32 :
                       dsp_process_channel(filters[CH_OUT_LEFT], master_l_32, CH_OUT_LEFT);
            out_r_32 = channel_bypassed[CH_OUT_RIGHT] ? master_r_32 :
                       dsp_process_channel(filters[CH_OUT_RIGHT], master_r_32, CH_OUT_RIGHT);
        }
#if ENABLE_SUB
        out_sub_32 = channel_bypassed[CH_OUT_SUB] ? sub_in_32 :
                     dsp_process_channel(filters[CH_OUT_SUB], sub_in_32, CH_OUT_SUB);
#endif

        // Per-channel gain and mute
        out_l_32   = channel_mute[0] ? 0 : (int32_t)(((int64_t)out_l_32 * channel_gain_mul[0]) >> 15);
        out_r_32   = channel_mute[1] ? 0 : (int32_t)(((int64_t)out_r_32 * channel_gain_mul[1]) >> 15);
        out_sub_32 = channel_mute[2] ? 0 : (int32_t)(((int64_t)out_sub_32 * channel_gain_mul[2]) >> 15);

        if (abs(out_l_32) > peak_ol) peak_ol = abs(out_l_32);
        if (abs(out_r_32) > peak_or) peak_or = abs(out_r_32);
        if (abs(out_sub_32) > peak_sub) peak_sub = abs(out_sub_32);

        // Master volume
        out_l_32   = (int32_t)(((int64_t)out_l_32 * vol_mul) >> 15);
        out_r_32   = (int32_t)(((int64_t)out_r_32 * vol_mul) >> 15);
        out_sub_32 = (int32_t)(((int64_t)out_sub_32 * vol_mul) >> 15);

        out_l_32 = clip_s32(out_l_32);
        out_r_32 = clip_s32(out_r_32);
        out_sub_32 = clip_s32(out_sub_32);

        // DELAY
        delay_lines[0][delay_write_idx] = out_l_32;
        delay_lines[1][delay_write_idx] = out_r_32;
        delay_lines[2][delay_write_idx] = out_sub_32;

        int32_t delayed_l   = delay_lines[0][(delay_write_idx - channel_delay_samples[0]) & MAX_DELAY_MASK];
        int32_t delayed_r   = delay_lines[1][(delay_write_idx - channel_delay_samples[1]) & MAX_DELAY_MASK];
        int32_t delayed_sub = delay_lines[2][(delay_write_idx - channel_delay_samples[2]) & MAX_DELAY_MASK];

        delay_write_idx = (delay_write_idx + 1) & MAX_DELAY_MASK;

        if (audio_buffer) {
            int16_t *out = (int16_t *) audio_buffer->buffer->bytes;
            out[i*2]     = (int16_t)(clip_s32(delayed_l + (1<<13)) >> 14);
            out[i*2+1]   = (int16_t)(clip_s32(delayed_r + (1<<13)) >> 14);
        }

#if ENABLE_SUB
        pdm_push_sample(delayed_sub, false);
#endif
    }

    global_status.peaks[0] = (uint16_t)(peak_ml >> 13);
    global_status.peaks[1] = (uint16_t)(peak_mr >> 13);
    global_status.peaks[2] = (uint16_t)(peak_ol >> 13);
    global_status.peaks[3] = (uint16_t)(peak_or >> 13);
    global_status.peaks[4] = (uint16_t)(peak_sub >> 13);
#endif

    if (audio_buffer) give_audio_buffer(producer_pool, audio_buffer);

    uint32_t end_time = time_us_32();
    global_status.cpu0_load = (uint8_t)((end_time - start_time) / 10);
}

// ----------------------------------------------------------------------------
// USB AUDIO PACKET CALLBACKS (pico-extras usb_device)
// ----------------------------------------------------------------------------

static void __not_in_flash_func(_as_audio_packet)(struct usb_endpoint *ep) {
    assert(ep->current_transfer);
    struct usb_buffer *usb_buffer = usb_current_out_packet_buffer(ep);

    usb_audio_packets++;
    process_audio_packet(usb_buffer->data, usb_buffer->data_len);

    // keep on truckin'
    usb_grow_transfer(ep->current_transfer, 1);
    usb_packet_done(ep);
}

static void __not_in_flash_func(_as_sync_packet)(struct usb_endpoint *ep) {
    assert(ep->current_transfer);
    struct usb_buffer *buffer = usb_current_in_packet_buffer(ep);
    assert(buffer->data_max >= 3);
    buffer->data_len = 3;

    // 10.14 fixed-point feedback: nominal sample rate
    uint feedback = (audio_state.freq << 14u) / 1000u;

    buffer->data[0] = feedback;
    buffer->data[1] = feedback >> 8u;
    buffer->data[2] = feedback >> 16u;

    // keep on truckin'
    usb_grow_transfer(ep->current_transfer, 1);
    usb_packet_done(ep);
}

static const struct usb_transfer_type as_transfer_type = {
    .on_packet = _as_audio_packet,
    .initial_packet_count = 1,
};

static const struct usb_transfer_type as_sync_transfer_type = {
    .on_packet = _as_sync_packet,
    .initial_packet_count = 1,
};

static struct usb_transfer as_transfer;
static struct usb_transfer as_sync_transfer;

// ----------------------------------------------------------------------------
// UAC1 AUDIO CONTROL REQUEST HANDLERS
// ----------------------------------------------------------------------------

static struct audio_control_cmd {
    uint8_t cmd;
    uint8_t type;
    uint8_t cs;
    uint8_t cn;
    uint8_t unit;
    uint8_t len;
} audio_control_cmd_t;

static void _audio_reconfigure(void) {
    rate_change_pending = true;
    pending_rate = audio_state.freq;
}

static bool do_get_current(struct usb_setup_packet *setup) {
    if ((setup->bmRequestType & USB_REQ_TYPE_RECIPIENT_MASK) == USB_REQ_TYPE_RECIPIENT_INTERFACE) {
        switch (setup->wValue >> 8u) {
        case FEATURE_MUTE_CONTROL:
            usb_start_tiny_control_in_transfer(audio_state.mute, 1);
            return true;
        case FEATURE_VOLUME_CONTROL:
            usb_start_tiny_control_in_transfer(audio_state.volume, 2);
            return true;
        }
    } else if ((setup->bmRequestType & USB_REQ_TYPE_RECIPIENT_MASK) == USB_REQ_TYPE_RECIPIENT_ENDPOINT) {
        if ((setup->wValue >> 8u) == ENDPOINT_FREQ_CONTROL) {
            usb_start_tiny_control_in_transfer(audio_state.freq, 3);
            return true;
        }
    }
    return false;
}

static bool do_get_minimum(struct usb_setup_packet *setup) {
    if ((setup->bmRequestType & USB_REQ_TYPE_RECIPIENT_MASK) == USB_REQ_TYPE_RECIPIENT_INTERFACE) {
        switch (setup->wValue >> 8u) {
        case FEATURE_VOLUME_CONTROL:
            usb_start_tiny_control_in_transfer(MIN_VOLUME, 2);
            return true;
        }
    }
    return false;
}

static bool do_get_maximum(struct usb_setup_packet *setup) {
    if ((setup->bmRequestType & USB_REQ_TYPE_RECIPIENT_MASK) == USB_REQ_TYPE_RECIPIENT_INTERFACE) {
        switch (setup->wValue >> 8u) {
        case FEATURE_VOLUME_CONTROL:
            usb_start_tiny_control_in_transfer(MAX_VOLUME, 2);
            return true;
        }
    }
    return false;
}

static bool do_get_resolution(struct usb_setup_packet *setup) {
    if ((setup->bmRequestType & USB_REQ_TYPE_RECIPIENT_MASK) == USB_REQ_TYPE_RECIPIENT_INTERFACE) {
        switch (setup->wValue >> 8u) {
        case FEATURE_VOLUME_CONTROL:
            usb_start_tiny_control_in_transfer(VOLUME_RESOLUTION, 2);
            return true;
        }
    }
    return false;
}

static void audio_cmd_packet(struct usb_endpoint *ep) {
    assert(audio_control_cmd_t.cmd == AUDIO_REQ_SetCurrent);
    struct usb_buffer *buffer = usb_current_out_packet_buffer(ep);
    audio_control_cmd_t.cmd = 0;
    if (buffer->data_len >= audio_control_cmd_t.len) {
        if (audio_control_cmd_t.type == USB_REQ_TYPE_RECIPIENT_INTERFACE) {
            switch (audio_control_cmd_t.cs) {
            case FEATURE_MUTE_CONTROL:
                audio_state.mute = buffer->data[0];
                break;
            case FEATURE_VOLUME_CONTROL:
                audio_set_volume(*(int16_t *) buffer->data);
                break;
            }
        } else if (audio_control_cmd_t.type == USB_REQ_TYPE_RECIPIENT_ENDPOINT) {
            if (audio_control_cmd_t.cs == ENDPOINT_FREQ_CONTROL) {
                uint32_t new_freq = (*(uint32_t *) buffer->data) & 0x00ffffffu;
                if (audio_state.freq != new_freq) {
                    audio_state.freq = new_freq;
                    _audio_reconfigure();
                }
            }
        }
    }
    usb_start_empty_control_in_transfer_null_completion();
}

static const struct usb_transfer_type _audio_cmd_transfer_type = {
    .on_packet = audio_cmd_packet,
    .initial_packet_count = 1,
};

static bool do_set_current(struct usb_setup_packet *setup) {
    if (setup->wLength && setup->wLength < 64) {
        audio_control_cmd_t.cmd = AUDIO_REQ_SetCurrent;
        audio_control_cmd_t.type = setup->bmRequestType & USB_REQ_TYPE_RECIPIENT_MASK;
        audio_control_cmd_t.len = (uint8_t) setup->wLength;
        audio_control_cmd_t.unit = setup->wIndex >> 8u;
        audio_control_cmd_t.cs = setup->wValue >> 8u;
        audio_control_cmd_t.cn = (uint8_t) setup->wValue;
        usb_start_control_out_transfer(&_audio_cmd_transfer_type);
        return true;
    }
    return false;
}

// Forward declaration — Console app sends vendor requests with wIndex=0 (interface recipient),
// so they arrive here on the AC interface and must be forwarded to the vendor handler.
static bool vendor_setup_request_handler(struct usb_interface *interface, struct usb_setup_packet *setup);

static bool ac_setup_request_handler(__unused struct usb_interface *interface, struct usb_setup_packet *setup) {
    setup = __builtin_assume_aligned(setup, 4);

    // Forward vendor-type requests to the vendor handler (Console sends wIndex=0)
    if (USB_REQ_TYPE_TYPE_VENDOR == (setup->bmRequestType & USB_REQ_TYPE_TYPE_MASK)) {
        return vendor_setup_request_handler(interface, setup);
    }

    if (USB_REQ_TYPE_TYPE_CLASS == (setup->bmRequestType & USB_REQ_TYPE_TYPE_MASK)) {
        switch (setup->bRequest) {
        case AUDIO_REQ_SetCurrent:
            return do_set_current(setup);
        case AUDIO_REQ_GetCurrent:
            return do_get_current(setup);
        case AUDIO_REQ_GetMinimum:
            return do_get_minimum(setup);
        case AUDIO_REQ_GetMaximum:
            return do_get_maximum(setup);
        case AUDIO_REQ_GetResolution:
            return do_get_resolution(setup);
        default:
            break;
        }
    }
    return false;
}

static bool _as_setup_request_handler(__unused struct usb_endpoint *ep, struct usb_setup_packet *setup) {
    setup = __builtin_assume_aligned(setup, 4);
    if (USB_REQ_TYPE_TYPE_CLASS == (setup->bmRequestType & USB_REQ_TYPE_TYPE_MASK)) {
        switch (setup->bRequest) {
        case AUDIO_REQ_SetCurrent:
            return do_set_current(setup);
        case AUDIO_REQ_GetCurrent:
            return do_get_current(setup);
        case AUDIO_REQ_GetMinimum:
            return do_get_minimum(setup);
        case AUDIO_REQ_GetMaximum:
            return do_get_maximum(setup);
        case AUDIO_REQ_GetResolution:
            return do_get_resolution(setup);
        default:
            break;
        }
    }
    return false;
}

static bool as_set_alternate(struct usb_interface *interface, uint alt) {
    assert(interface == &as_op_interface);
    usb_audio_alt_set = alt;
    return alt < 2;
}

// ----------------------------------------------------------------------------
// VENDOR INTERFACE HANDLER (DSPi commands via EP0 control transfers)
// ----------------------------------------------------------------------------

// Buffer for vendor SET requests
static uint8_t vendor_rx_buf[64];
static uint8_t vendor_last_request = 0;
static uint16_t vendor_last_wValue = 0;

static void vendor_cmd_packet(struct usb_endpoint *ep) {
    struct usb_buffer *buffer = usb_current_out_packet_buffer(ep);

    if (buffer->data_len > 0 && buffer->data_len <= sizeof(vendor_rx_buf)) {
        memcpy(vendor_rx_buf, buffer->data, buffer->data_len);
    }

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
            if (buffer->data_len >= 4) {
                float db;
                memcpy(&db, vendor_rx_buf, 4);
                global_preamp_db = db;
                float linear = powf(10.0f, db / 20.0f);
                global_preamp_mul = (int32_t)(linear * (float)(1<<28));
                global_preamp_linear = linear;
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
                    if (vol >= 91 * 256) vol = 91 * 256 - 1;
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
    }

    usb_start_empty_control_in_transfer_null_completion();
}

static const struct usb_transfer_type _vendor_cmd_transfer_type = {
    .on_packet = vendor_cmd_packet,
    .initial_packet_count = 1,
};

// Helper: write data into the control IN buffer and send
static void vendor_send_response(const void *data, uint len) {
    struct usb_buffer *buffer = usb_current_in_packet_buffer(usb_get_control_in_endpoint());
    memcpy(buffer->data, data, len);
    buffer->data_len = len;
    usb_start_single_buffer_control_in_transfer();
}

static bool vendor_setup_request_handler(__unused struct usb_interface *interface, struct usb_setup_packet *setup) {
    setup = __builtin_assume_aligned(setup, 4);

    if (!(setup->bmRequestType & USB_DIR_IN)) {
        // Host -> Device (SET requests)
        vendor_last_request = setup->bRequest;
        vendor_last_wValue = setup->wValue;

        if (setup->wLength && setup->wLength <= sizeof(vendor_rx_buf)) {
            usb_start_control_out_transfer(&_vendor_cmd_transfer_type);
            return true;
        }
        return false;

    } else {
        // Device -> Host (GET requests)
        static uint8_t resp_buf[64];

        switch (setup->bRequest) {
            case REQ_GET_PREAMP: {
                float current_db = global_preamp_db;
                memcpy(resp_buf, &current_db, 4);
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

            case REQ_GET_STATUS: {
                if (setup->wValue == 9) {
                    // Combined status: all peaks + CPU in one 12-byte transfer
                    resp_buf[0] = global_status.peaks[0] & 0xFF;
                    resp_buf[1] = global_status.peaks[0] >> 8;
                    resp_buf[2] = global_status.peaks[1] & 0xFF;
                    resp_buf[3] = global_status.peaks[1] >> 8;
                    resp_buf[4] = global_status.peaks[2] & 0xFF;
                    resp_buf[5] = global_status.peaks[2] >> 8;
                    resp_buf[6] = global_status.peaks[3] & 0xFF;
                    resp_buf[7] = global_status.peaks[3] >> 8;
                    resp_buf[8] = global_status.peaks[4] & 0xFF;
                    resp_buf[9] = global_status.peaks[4] >> 8;
                    resp_buf[10] = global_status.cpu0_load;
                    resp_buf[11] = global_status.cpu1_load;
                    vendor_send_response(resp_buf, 12);
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
                }
                usb_start_tiny_control_in_transfer(resp, 4);
                return true;
            }

            case REQ_SAVE_PARAMS: {
                int result = flash_save_params();
                usb_start_tiny_control_in_transfer(result, 1);
                return true;
            }

            case REQ_LOAD_PARAMS: {
                int result = flash_load_params();
                if (result == FLASH_OK) {
                    dsp_recalculate_all_filters((float)audio_state.freq);
                    dsp_update_delay_samples((float)audio_state.freq);
                    loudness_recompute_pending = true;
                }
                usb_start_tiny_control_in_transfer(result, 1);
                return true;
            }

            case REQ_FACTORY_RESET: {
                flash_factory_reset();
                dsp_recalculate_all_filters((float)audio_state.freq);
                dsp_update_delay_samples((float)audio_state.freq);
                loudness_recompute_pending = true;
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
        }

        return false;
    }
}

// ----------------------------------------------------------------------------
// DEVICE-LEVEL SETUP REQUEST HANDLER (WCID / MS OS descriptors)
// ----------------------------------------------------------------------------

static bool device_setup_request_handler(struct usb_device *dev, struct usb_setup_packet *setup) {
    (void)dev;
    setup = __builtin_assume_aligned(setup, 4);

    // Handle Microsoft OS vendor requests (WCID compat ID and ext props)
    if ((setup->bmRequestType & USB_REQ_TYPE_TYPE_MASK) == USB_REQ_TYPE_TYPE_VENDOR &&
        setup->bRequest == MS_VENDOR_CODE) {
        switch (setup->wIndex) {
            case 0x0004: {
                uint16_t len = setup->wLength < MS_COMPAT_ID_DESC_LEN ? setup->wLength : MS_COMPAT_ID_DESC_LEN;
                vendor_send_response(ms_compat_id_descriptor, len);
                return true;
            }
            case 0x0005: {
                uint16_t len = setup->wLength < MS_EXT_PROP_DESC_LEN ? setup->wLength : MS_EXT_PROP_DESC_LEN;
                vendor_send_response(ms_ext_prop_descriptor, len);
                return true;
            }
        }
    }

    return false;
}

// ----------------------------------------------------------------------------
// STRING DESCRIPTOR CALLBACK (with MS OS string at index 0xEE)
// ----------------------------------------------------------------------------

static const char *_get_descriptor_string(uint index) {
    if (index == 0xEE) {
        // Return NULL — we handle MS OS string via the device setup handler
        // Actually, we need to handle string 0xEE specially
        return NULL;
    }
    if (index >= 1 && index <= count_of(descriptor_strings)) {
        return descriptor_strings[index - 1];
    }
    return "";
}

// ----------------------------------------------------------------------------
// INIT
// ----------------------------------------------------------------------------

// S/PDIF Config
static audio_spdif_instance_t spdif_instance = {0};
struct audio_spdif_config config = {
    .pin = PICO_AUDIO_SPDIF_PIN,
    .dma_channel = 0,
    .pio_sm = 0,
    .pio = PICO_AUDIO_SPDIF_PIO,
    .dma_irq = PICO_AUDIO_SPDIF_DMA_IRQ,
};
struct audio_buffer_format producer_format = { .format = &audio_format_48k, .sample_stride = 4 };

void usb_sound_card_init(void) {
    // S/PDIF Setup (this must happen before USB init to claim DMA channel 0)
    producer_pool = audio_new_producer_pool(&producer_format, AUDIO_BUFFER_COUNT, 192);

    audio_spdif_setup(&spdif_instance, &audio_format_48k, &config);
    audio_spdif_connect_extra(&spdif_instance, producer_pool, false, AUDIO_BUFFER_COUNT / 2, NULL);

    irq_set_priority(DMA_IRQ_0 + PICO_AUDIO_SPDIF_DMA_IRQ, PICO_HIGHEST_IRQ_PRIORITY);
    audio_spdif_set_enabled(&spdif_instance, true);

    // Initialize pico-extras USB device with 3 interfaces: AC, AS, Vendor

    // Audio Control interface
    usb_interface_init(&ac_interface, &audio_device_config.ac_interface, NULL, 0, true);
    ac_interface.setup_request_handler = ac_setup_request_handler;

    // Audio Streaming interface with OUT + sync endpoints
    static struct usb_endpoint *const op_endpoints[] = {
        &ep_op_out, &ep_op_sync
    };
    usb_interface_init(&as_op_interface, &audio_device_config.as_op_interface, op_endpoints, count_of(op_endpoints), true);
    as_op_interface.set_alternate_handler = as_set_alternate;
    ep_op_out.setup_request_handler = _as_setup_request_handler;
    as_transfer.type = &as_transfer_type;
    usb_set_default_transfer(&ep_op_out, &as_transfer);
    as_sync_transfer.type = &as_sync_transfer_type;
    usb_set_default_transfer(&ep_op_sync, &as_sync_transfer);

    // Vendor interface (control-only, no endpoints)
    usb_interface_init(&vendor_interface, &audio_device_config.vendor_interface, NULL, 0, true);
    vendor_interface.setup_request_handler = vendor_setup_request_handler;

    // Initialize USB device
    static struct usb_interface *const boot_device_interfaces[] = {
        &ac_interface,
        &as_op_interface,
        &vendor_interface,
    };
    struct usb_device *device = usb_device_init(&boot_device_descriptor, &audio_device_config.descriptor,
        boot_device_interfaces, count_of(boot_device_interfaces),
        _get_descriptor_string);
    assert(device);
    device->setup_request_handler = device_setup_request_handler;

    // Initialize DSP
    dsp_init_default_filters();
    dsp_recalculate_all_filters(48000.0f);
    audio_set_volume(DEFAULT_VOLUME);
    _audio_reconfigure();

    // Initialize ADC for temperature sensor
    adc_init();
    adc_set_temp_sensor_enabled(true);

    usb_device_start();
}
