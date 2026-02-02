/*
 * TinyUSB Audio Implementation for DSPi
 * UAC2 Audio Streaming with DSP Pipeline
 */

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "tusb.h"
#include "usb_audio.h"

// UAC2 Audio Class Request Codes
#define AUDIO_CS_REQ_CUR    0x01
#define AUDIO_CS_REQ_RANGE  0x02

// UAC2 Entity IDs (must match usb_descriptors.c)
#define UAC2_ENTITY_CLOCK           0x04
#define UAC2_ENTITY_FEATURE_UNIT    0x02
#include "usb_descriptors.h"
#include "dsp_pipeline.h"
#include "pdm_generator.h"
#include "flash_storage.h"
#include "pico/audio_spdif.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"

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

// Per-channel gain and mute (output channels: L=0, R=1, Sub=2)
volatile float channel_gain_db[3] = {0.0f, 0.0f, 0.0f};
volatile int32_t channel_gain_mul[3] = {32768, 32768, 32768};  // Unity = 2^15
volatile bool channel_mute[3] = {false, false, false};

// Sync State
volatile uint64_t total_samples_produced = 0;
volatile uint64_t start_time_us = 0;
volatile bool sync_started = false;
static volatile uint64_t last_packet_time_us = 0;
#define AUDIO_GAP_THRESHOLD_US 50000  // 50ms - reset sync if packets stop this long

// Audio Pool
struct audio_buffer_pool *producer_pool = NULL;
struct audio_format audio_format_48k = { .format = AUDIO_BUFFER_FORMAT_PCM_S16, .sample_freq = 48000, .channel_count = 2 };

// Audio receive buffer
static uint8_t audio_rx_buffer[384];  // Max packet size

// ----------------------------------------------------------------------------
// DELAY LINES (moved from dsp_pipeline.c to here or externed?)
// Actually delay_lines are defined in dsp_pipeline.c. usb_audio.c uses them?
// No, usb_audio.c accesses them via extern in dsp_pipeline.h?
// Let's check dsp_pipeline.h
// It says: extern int32_t delay_lines[3][MAX_DELAY_SAMPLES];
// So I need to update dsp_pipeline.h FIRST to conditionalize the type.
// I missed that. I will cancel this and update dsp_pipeline.h first.


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
    audio_state.vol_mul = db_to_vol[((uint16_t)volume) >> 8u];
}

// ----------------------------------------------------------------------------
// VENDOR INTERFACE HELPERS
// ----------------------------------------------------------------------------

bool vendor_interface_ready(void) {
    return tud_ready();
}

bool vendor_queue_response(const VendorRespPacket *resp) {
    // Deprecated with Control-Only interface
    (void)resp;
    return false;
}

// ----------------------------------------------------------------------------
// AUDIO PROCESSING (called from TinyUSB audio callback)
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
    float vol_mul = (float)audio_state.vol_mul / 32768.0f; // Q15 -> float
    float preamp = global_preamp_db == 0.0f ? 1.0f : powf(10.0f, global_preamp_db / 20.0f);
    bool is_bypassed = bypass_master_eq;

    float peak_ml = 0, peak_mr = 0, peak_ol = 0, peak_or = 0, peak_sub = 0;

    for (uint32_t i = 0; i < sample_count; i++) {
        // Input: 16-bit PCM -> Float Normalized (-1.0 to 1.0)
        float raw_left = (float)in[i*2] / 32768.0f;
        float raw_right = (float)in[i*2+1] / 32768.0f;

        // Preamp
        raw_left *= preamp;
        raw_right *= preamp;

        // Master EQ
        float master_l, master_r;
        if (is_bypassed) {
            master_l = raw_left;
            master_r = raw_right;
        } else {
            if (audio_buffer) {
                master_l = dsp_process_channel(filters[CH_MASTER_LEFT], raw_left, CH_MASTER_LEFT);
                master_r = dsp_process_channel(filters[CH_MASTER_RIGHT], raw_right, CH_MASTER_RIGHT);
            } else {
                master_l = 0.0f; master_r = 0.0f;
            }
        }

        if (fabsf(master_l) > peak_ml) peak_ml = fabsf(master_l);
        if (fabsf(master_r) > peak_mr) peak_mr = fabsf(master_r);

        // Subwoofer Mix
        float sub_in = (master_l + master_r) * 0.5f;
        float out_l = 0.0f, out_r = 0.0f, out_sub = 0.0f;

        if (audio_buffer) {
            out_l = dsp_process_channel(filters[CH_OUT_LEFT], master_l, CH_OUT_LEFT);
            out_r = dsp_process_channel(filters[CH_OUT_RIGHT], master_r, CH_OUT_RIGHT);
        }
#if ENABLE_SUB
        out_sub = dsp_process_channel(filters[CH_OUT_SUB], sub_in, CH_OUT_SUB);
#endif

        // Per-channel Gain & Mute
        float gain_l = powf(10.0f, channel_gain_db[0] / 20.0f);
        float gain_r = powf(10.0f, channel_gain_db[1] / 20.0f);
        float gain_sub = powf(10.0f, channel_gain_db[2] / 20.0f);

        out_l   = channel_mute[0] ? 0.0f : (out_l * gain_l);
        out_r   = channel_mute[1] ? 0.0f : (out_r * gain_r);
        out_sub = channel_mute[2] ? 0.0f : (out_sub * gain_sub);

        if (fabsf(out_l) > peak_ol) peak_ol = fabsf(out_l);
        if (fabsf(out_r) > peak_or) peak_or = fabsf(out_r);
        if (fabsf(out_sub) > peak_sub) peak_sub = fabsf(out_sub);

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
        // PDM expects int32 Q28-like range (or at least significant magnitude for dither)
        // Convert back to int32 scale for PDM generator
        int32_t pdm_sample = (int32_t)(fmaxf(-1.0f, fminf(1.0f, delayed_sub)) * 32768.0f * 16384.0f); // Scale up to near INT32 range
        // Wait, PDM generator logic expects "pcm_val >> 14".
        // Original: out_sub_32 (Q28 approx).
        // pdm_push_sample(delayed_sub, false);
        // Let's match original scale: Q28 -> 1.0 = 2^28.
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

    int32_t peak_ml = 0, peak_mr = 0, peak_ol = 0, peak_or = 0, peak_sub = 0;

    for (uint32_t i = 0; i < sample_count; i++) {
        int16_t raw_left_16 = in[i*2];
        int16_t raw_right_16 = in[i*2+1];

        int32_t raw_left_32 = (int32_t)raw_left_16 << 14;
        int32_t raw_right_32 = (int32_t)raw_right_16 << 14;

        raw_left_32  = clip_s64_to_s32(((int64_t)raw_left_32 * preamp) >> 28);
        raw_right_32 = clip_s64_to_s32(((int64_t)raw_right_32 * preamp) >> 28);

        int32_t master_l_32, master_r_32;
        if (is_bypassed) {
            master_l_32 = raw_left_32;
            master_r_32 = raw_right_32;
        } else {
            if (audio_buffer) {
                master_l_32 = dsp_process_channel(filters[CH_MASTER_LEFT], raw_left_32, CH_MASTER_LEFT);
                master_r_32 = dsp_process_channel(filters[CH_MASTER_RIGHT], raw_right_32, CH_MASTER_RIGHT);
            } else {
                master_l_32 = 0; master_r_32 = 0;
            }
        }

        if (abs(master_l_32) > peak_ml) peak_ml = abs(master_l_32);
        if (abs(master_r_32) > peak_mr) peak_mr = abs(master_r_32);

        int32_t sub_in_32 = (master_l_32 + master_r_32) >> 1;
        int32_t out_l_32 = 0, out_r_32 = 0, out_sub_32 = 0;

        if (audio_buffer) {
            out_l_32 = dsp_process_channel(filters[CH_OUT_LEFT], master_l_32, CH_OUT_LEFT);
            out_r_32 = dsp_process_channel(filters[CH_OUT_RIGHT], master_r_32, CH_OUT_RIGHT);
        }
#if ENABLE_SUB
        out_sub_32 = dsp_process_channel(filters[CH_OUT_SUB], sub_in_32, CH_OUT_SUB);
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
// TINYUSB AUDIO CALLBACKS
// ----------------------------------------------------------------------------

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf) {
    (void)rhport;

    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t entityId = TU_U16_HIGH(p_request->wIndex);

    // Clock Source entity - sample rate control (UAC2)
    if (entityId == UAC2_ENTITY_CLOCK) {
        if (ctrlSel == AUDIO_CS_CTRL_SAM_FREQ) {
            // UAC2 uses 4-byte sample rate
            uint32_t new_freq;
            memcpy(&new_freq, buf, 4);

            if (audio_state.freq != new_freq) {
                audio_state.freq = new_freq;
                rate_change_pending = true;
                pending_rate = new_freq;
            }
            return true;
        }
    }

    // Feature Unit controls
    if (entityId == UAC2_ENTITY_FEATURE_UNIT) {
        switch (ctrlSel) {
            case AUDIO_FU_CTRL_MUTE:
                audio_state.mute = buf[0];
                return true;

            case AUDIO_FU_CTRL_VOLUME:
                if (channelNum == 0) {  // Master channel
                    int16_t vol;
                    memcpy(&vol, buf, 2);
                    audio_set_volume(vol);
                }
                return true;
        }
    }

    return false;
}

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    (void)rhport;

    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t entityId = TU_U16_HIGH(p_request->wIndex);

    // Clock Source entity - sample rate control (UAC2)
    if (entityId == UAC2_ENTITY_CLOCK) {
        if (ctrlSel == AUDIO_CS_CTRL_SAM_FREQ) {
            switch (p_request->bRequest) {
                case AUDIO_CS_REQ_CUR: {
                    // Return current sample rate (4 bytes for UAC2)
                    uint32_t freq = audio_state.freq;
                    return tud_control_xfer(rhport, p_request, &freq, 4);
                }
                case AUDIO_CS_REQ_RANGE: {
                    // UAC2 Range format: wNumSubRanges (2) + array of { dMIN, dMAX, dRES } (12 bytes each)
                    // We support 2 sample rates: 44100 and 48000
                    static const uint8_t sample_rate_range[] = {
                        // wNumSubRanges = 2
                        0x02, 0x00,
                        // Subrange 1: 44100 Hz
                        0x44, 0xAC, 0x00, 0x00,  // dMIN = 44100
                        0x44, 0xAC, 0x00, 0x00,  // dMAX = 44100
                        0x00, 0x00, 0x00, 0x00,  // dRES = 0
                        // Subrange 2: 48000 Hz
                        0x80, 0xBB, 0x00, 0x00,  // dMIN = 48000
                        0x80, 0xBB, 0x00, 0x00,  // dMAX = 48000
                        0x00, 0x00, 0x00, 0x00,  // dRES = 0
                    };
                    return tud_control_xfer(rhport, p_request, (void*)sample_rate_range,
                                            TU_MIN(p_request->wLength, sizeof(sample_rate_range)));
                }
            }
        }
        if (ctrlSel == AUDIO_CS_CTRL_CLK_VALID) {
            // Clock is always valid
            static const uint8_t clk_valid = 1;
            return tud_control_xfer(rhport, p_request, (void*)&clk_valid, 1);
        }
    }

    // Feature Unit controls
    if (entityId == UAC2_ENTITY_FEATURE_UNIT) {
        switch (ctrlSel) {
            case AUDIO_FU_CTRL_MUTE:
                return tud_control_xfer(rhport, p_request, (void*)&audio_state.mute, 1);

            case AUDIO_FU_CTRL_VOLUME:
                switch (p_request->bRequest) {
                    case AUDIO_CS_REQ_CUR:
                        return tud_control_xfer(rhport, p_request, (void*)&audio_state.volume, 2);
                    case AUDIO_CS_REQ_RANGE: {
                        // UAC2 Volume Range: wNumSubRanges (2) + { wMIN, wMAX, wRES } (6 bytes each)
                        static uint8_t vol_range[8];
                        int16_t min_vol = MIN_VOLUME;
                        int16_t max_vol = MAX_VOLUME;
                        int16_t res_vol = VOLUME_RESOLUTION;
                        vol_range[0] = 0x01; vol_range[1] = 0x00;  // wNumSubRanges = 1
                        memcpy(&vol_range[2], &min_vol, 2);
                        memcpy(&vol_range[4], &max_vol, 2);
                        memcpy(&vol_range[6], &res_vol, 2);
                        return tud_control_xfer(rhport, p_request, vol_range,
                                                TU_MIN(p_request->wLength, sizeof(vol_range)));
                    }
                }
                break;
        }
    }

    (void)channelNum;
    return false;
}

// Invoked when audio class specific set request received for an EP
// In UAC2, sample rate is handled via Clock Source entity, not endpoint
bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf) {
    (void)rhport;
    (void)p_request;
    (void)buf;
    // No endpoint-specific controls needed for UAC2
    return false;
}

// Invoked when audio class specific get request received for an EP
bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    (void)rhport;
    (void)p_request;
    // No endpoint-specific controls needed for UAC2
    return false;
}

// Invoked when set interface (alt setting) is called
bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    (void)rhport;
    // Track alt setting for debug
    uint8_t alt = (uint8_t)p_request->wValue;
    usb_audio_alt_set = alt;
    return true;
}

// Invoked when audio streaming interface is closed (alt 0)
bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    (void)rhport;
    (void)p_request;
    return true;
}

// Invoked when audio data received from host
bool tud_audio_rx_done_pre_read_cb(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting) {
    (void)rhport;
    (void)func_id;
    (void)ep_out;
    (void)cur_alt_setting;

    if (n_bytes_received > 0 && n_bytes_received <= sizeof(audio_rx_buffer)) {
        // Read audio data from TinyUSB
        uint16_t bytes_read = tud_audio_read(audio_rx_buffer, n_bytes_received);
        if (bytes_read > 0) {
            usb_audio_packets++;  // Debug counter
            process_audio_packet(audio_rx_buffer, bytes_read);
        }
    }

    return true;
}

// Invoked for feedback endpoint - configure for manual fixed-point control
void tud_audio_feedback_params_cb(uint8_t func_id, uint8_t alt_itf, audio_feedback_params_t* feedback_param) {
    (void)func_id;
    (void)alt_itf;

    // Use disabled method - we'll set feedback manually via tud_audio_n_fb_set()
    // This gives us direct control over drift correction
    feedback_param->method = AUDIO_FEEDBACK_METHOD_DISABLED;
}

// Update feedback value (called from main loop or timer)
void update_audio_feedback(void) {
    usb_audio_mounted = tud_audio_mounted() ? 1 : 0;
    if (!usb_audio_mounted) return;

    // For full-speed USB, feedback is in 10.14 fixed-point format
    // Represents samples per frame (1ms at full-speed)
    // 48000 Hz = 48 samples/frame = 48 << 14 = 786432
    // 44100 Hz = 44.1 samples/frame = 44.1 * 16384 = 722534

    uint32_t nominal_10_14 = ((uint64_t)audio_state.freq << 14) / 1000;

    if (sync_started && total_samples_produced > audio_state.freq) {
        uint64_t now_us = time_us_64();
        uint64_t elapsed_us = now_us - start_time_us;
        uint64_t expected_samples = ((uint64_t)elapsed_us * audio_state.freq) / 1000000;
        int64_t drift = (int64_t)total_samples_produced - (int64_t)expected_samples;

        // Apply proportional correction: ~0.1% adjustment per sample of drift
        // Scale drift to 10.14 fixed point correction
        int32_t correction = (int32_t)((drift * 16) / 10);  // ~0.1% per sample drift
        if (correction > 8192) correction = 8192;    // Cap at ~0.5 samples/frame
        if (correction < -8192) correction = -8192;

        // If we've produced MORE than expected, tell host to send LESS (subtract)
        // If we've produced LESS than expected, tell host to send MORE (add)
        uint32_t feedback = nominal_10_14 - (uint32_t)correction;
        tud_audio_n_fb_set(0, feedback);
    } else {
        tud_audio_n_fb_set(0, nominal_10_14);
    }
}

// ----------------------------------------------------------------------------
// VENDOR CONTROL TRANSFER CALLBACK (for vendor commands and WCID)
// ----------------------------------------------------------------------------

// Buffer for vendor SET requests
static uint8_t vendor_rx_buf[64];
static uint8_t vendor_last_request = 0;
static uint16_t vendor_last_wValue = 0;

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request) {
    // First check for Microsoft OS descriptor requests
    if ((request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR) &&
        (request->bRequest == MS_VENDOR_CODE)) {
        return handle_ms_vendor_request(rhport, stage, request);
    }

    // Only handle SETUP stage for data-out transfers, DATA stage for data-in
    if (request->bmRequestType_bit.direction == TUSB_DIR_OUT) {
        // Host -> Device (SET requests)

        if (stage == CONTROL_STAGE_SETUP) {
            // Save request info for DATA stage
            vendor_last_request = request->bRequest;
            vendor_last_wValue = request->wValue;
            // Prepare to receive data
            return tud_control_xfer(rhport, request, vendor_rx_buf, request->wLength);
        }

        if (stage == CONTROL_STAGE_DATA) {
            // Data received, process command
            switch (vendor_last_request) {
                case REQ_SET_EQ_PARAM:
                    if (request->wLength >= sizeof(EqParamPacket)) {
                        memcpy((void*)&pending_packet, vendor_rx_buf, sizeof(EqParamPacket));
                        eq_update_pending = true;
                    }
                    break;

                case REQ_SET_PREAMP:
                    if (request->wLength >= 4) {
                        float db;
                        memcpy(&db, vendor_rx_buf, 4);
                        global_preamp_db = db;
                        float linear = powf(10.0f, db / 20.0f);
                        global_preamp_mul = (int32_t)(linear * (float)(1<<28));
                    }
                    break;

                case REQ_SET_DELAY: {
                    uint8_t ch = vendor_last_wValue & 0xFF;
                    if (ch < NUM_CHANNELS && request->wLength >= 4) {
                        float ms;
                        memcpy(&ms, vendor_rx_buf, 4);
                        if (ms < 0) ms = 0;
                        channel_delays_ms[ch] = ms;
                        dsp_update_delay_samples((float)audio_state.freq);
                    }
                    break;
                }

                case REQ_SET_BYPASS:
                    if (request->wLength >= 1) {
                        bypass_master_eq = (vendor_rx_buf[0] != 0);
                    }
                    break;

                case REQ_SET_CHANNEL_GAIN: {
                    uint8_t ch = vendor_last_wValue & 0xFF;
                    if (ch < 3 && request->wLength >= 4) {
                        float db;
                        memcpy(&db, vendor_rx_buf, 4);
                        channel_gain_db[ch] = db;
                        float linear = powf(10.0f, db / 20.0f);
                        channel_gain_mul[ch] = (int32_t)(linear * 32768.0f);
                    }
                    break;
                }

                case REQ_SET_CHANNEL_MUTE: {
                    uint8_t ch = vendor_last_wValue & 0xFF;
                    if (ch < 3 && request->wLength >= 1) {
                        channel_mute[ch] = (vendor_rx_buf[0] != 0);
                    }
                    break;
                }
            }
            return true;
        }

        return true;

    } else {
        // Device -> Host (GET requests)

        if (stage != CONTROL_STAGE_SETUP) return true;

        static uint8_t resp_buf[64];

        switch (request->bRequest) {
            case REQ_GET_PREAMP: {
                float current_db = global_preamp_db;
                memcpy(resp_buf, &current_db, 4);
                return tud_control_xfer(rhport, request, resp_buf, 4);
            }

            case REQ_GET_DELAY: {
                uint8_t ch = (uint8_t)request->wValue;
                if (ch < NUM_CHANNELS) {
                    memcpy(resp_buf, (void*)&channel_delays_ms[ch], 4);
                    return tud_control_xfer(rhport, request, resp_buf, 4);
                }
                return false;
            }

            case REQ_GET_BYPASS: {
                resp_buf[0] = bypass_master_eq ? 1 : 0;
                return tud_control_xfer(rhport, request, resp_buf, 1);
            }

            case REQ_GET_CHANNEL_GAIN: {
                uint8_t ch = (uint8_t)request->wValue;
                if (ch < 3) {
                    memcpy(resp_buf, (void*)&channel_gain_db[ch], 4);
                    return tud_control_xfer(rhport, request, resp_buf, 4);
                }
                return false;
            }

            case REQ_GET_CHANNEL_MUTE: {
                uint8_t ch = (uint8_t)request->wValue;
                if (ch < 3) {
                    resp_buf[0] = channel_mute[ch] ? 1 : 0;
                    return tud_control_xfer(rhport, request, resp_buf, 1);
                }
                return false;
            }

            case REQ_GET_STATUS: {
                if (request->wValue == 9) {
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
                    return tud_control_xfer(rhport, request, resp_buf, 12);
                }

                uint32_t resp = 0;
                switch (request->wValue) {
                    case 0: resp = (uint32_t)global_status.peaks[0] | ((uint32_t)global_status.peaks[1] << 16); break;
                    case 1: resp = (uint32_t)global_status.peaks[2] | ((uint32_t)global_status.peaks[3] << 16); break;
                    case 2: resp = (uint32_t)global_status.peaks[4] | ((uint32_t)global_status.cpu0_load << 16) | ((uint32_t)global_status.cpu1_load << 24); break;
                    case 3: resp = pdm_ring_overruns; break;
                    case 4: resp = pdm_ring_underruns; break;
                    case 5: resp = pdm_dma_overruns; break;
                    case 6: resp = pdm_dma_underruns; break;
                    case 7: resp = spdif_overruns; break;
                    case 8: resp = spdif_underruns; break;
                    case 10: resp = usb_audio_packets; break;  // Debug: USB audio packet count
                    case 11: resp = usb_audio_alt_set; break;  // Debug: last alt setting
                    case 12: resp = usb_audio_mounted; break;  // Debug: audio mounted state
                    case 13: resp = usb_config_requests; break; // Debug: config descriptor requests
                }
                memcpy(resp_buf, &resp, 4);
                return tud_control_xfer(rhport, request, resp_buf, 4);
            }

            case REQ_SAVE_PARAMS: {
                int result = flash_save_params();
                resp_buf[0] = result;
                return tud_control_xfer(rhport, request, resp_buf, 1);
            }

            case REQ_LOAD_PARAMS: {
                int result = flash_load_params();
                if (result == FLASH_OK) {
                    dsp_recalculate_all_filters((float)audio_state.freq);
                    dsp_update_delay_samples((float)audio_state.freq);
                }
                resp_buf[0] = result;
                return tud_control_xfer(rhport, request, resp_buf, 1);
            }

            case REQ_FACTORY_RESET: {
                flash_factory_reset();
                dsp_recalculate_all_filters((float)audio_state.freq);
                dsp_update_delay_samples((float)audio_state.freq);
                resp_buf[0] = FLASH_OK;
                return tud_control_xfer(rhport, request, resp_buf, 1);
            }

            case REQ_GET_EQ_PARAM: {
                uint8_t channel = (request->wValue >> 8) & 0xFF;
                uint8_t band = (request->wValue >> 4) & 0x0F;
                uint8_t param = request->wValue & 0x0F;
                if (channel < NUM_CHANNELS && band < channel_band_counts[channel]) {
                    uint32_t val_to_send = 0;
                    EqParamPacket *p = &filter_recipes[channel][band];
                    switch (param) {
                        case 0: val_to_send = (uint32_t)p->type; break;
                        case 1: memcpy(&val_to_send, &p->freq, 4); break;
                        case 2: memcpy(&val_to_send, &p->Q, 4); break;
                        case 3: memcpy(&val_to_send, &p->gain_db, 4); break;
                    }
                    memcpy(resp_buf, &val_to_send, 4);
                    return tud_control_xfer(rhport, request, resp_buf, 4);
                }
                return false;
            }
        }

        return false;
    }
}

// ----------------------------------------------------------------------------
// INIT
// ----------------------------------------------------------------------------

// S/PDIF Config
struct audio_spdif_config config = { .pin = PICO_AUDIO_SPDIF_PIN, .dma_channel = 0, .pio_sm = 0 };
struct audio_buffer_format producer_format = { .format = &audio_format_48k, .sample_stride = 4 };

static void _audio_reconfigure(void) {
    rate_change_pending = true;
    pending_rate = audio_state.freq;
}

void usb_sound_card_init(void) {
    // S/PDIF Setup (this must happen before TinyUSB init to claim DMA channel 0)
    producer_pool = audio_new_producer_pool(&producer_format, AUDIO_BUFFER_COUNT, 192);

    audio_spdif_setup(&audio_format_48k, &config);
    audio_spdif_connect_extra(producer_pool, false, AUDIO_BUFFER_COUNT / 2, NULL);

    irq_set_priority(DMA_IRQ_0 + PICO_AUDIO_SPDIF_DMA_IRQ, PICO_HIGHEST_IRQ_PRIORITY);
    audio_spdif_set_enabled(true);

    // Initialize TinyUSB
    tud_init(BOARD_TUD_RHPORT);

    // Initialize DSP
    dsp_init_default_filters();
    dsp_recalculate_all_filters(48000.0f);
    audio_set_volume(DEFAULT_VOLUME);
    _audio_reconfigure();
}
