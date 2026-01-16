#include <string.h>
#include <math.h>
#include "usb_audio.h"
#include "usb_descriptors.h"
#include "dsp_pipeline.h"
#include "pdm_generator.h"
#include "pico/audio_spdif.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"

// External USB control endpoint (defined in pico-sdk usb_device.c)
extern struct usb_endpoint usb_control_in;

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

static volatile float global_preamp_db = 0.0f;
static volatile int32_t global_preamp_mul = 268435456;

// Sync State
volatile uint64_t total_samples_produced = 0;
volatile uint64_t start_time_us = 0;
volatile bool sync_started = false;

// Audio Pool
struct audio_buffer_pool *producer_pool = NULL;
struct audio_format audio_format_48k = { .format = AUDIO_BUFFER_FORMAT_PCM_S16, .sample_freq = 48000, .channel_count = 2 };

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

#define ENCODE_DB(x) ((uint16_t)(int16_t)((x)*256))
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
    return true; // Always ready with control transfers
}

bool vendor_queue_response(const VendorRespPacket *resp) {
    // Deprecated with Control-Only interface
    return false;
}

// ----------------------------------------------------------------------------
// AUDIO CALLBACK
// ----------------------------------------------------------------------------

void __not_in_flash_func(usb_sof_irq)(void) { return; }

static void __not_in_flash_func(_as_audio_packet)(struct usb_endpoint *ep) {
    uint32_t start_time = timer_hw->timerawl;

    struct usb_buffer *usb_buffer = usb_current_out_packet_buffer(ep);
    if (!usb_buffer) { usb_start_empty_control_in_transfer_null_completion(); return; }

    struct audio_buffer* audio_buffer = NULL;
    if (producer_pool) audio_buffer = take_audio_buffer(producer_pool, false);
    if (audio_buffer) audio_buffer->sample_count = usb_buffer->data_len / 4;
    
    uint32_t sample_count = usb_buffer->data_len / 4;
    
    if (!sync_started) {
        start_time_us = time_us_64();
        sync_started = true;
    }
    total_samples_produced += sample_count;

    int16_t *in = (int16_t *) usb_buffer->data;
    int32_t vol_mul = audio_state.vol_mul;
    int32_t preamp = global_preamp_mul;
    bool is_silent = true;
    int32_t silence_threshold = 1 << 20;
    bool is_bypassed = bypass_master_eq;

    int32_t peak_ml = 0, peak_mr = 0, peak_ol = 0, peak_or = 0, peak_sub = 0;

    for (uint32_t i = 0; i < sample_count; i++) {
        int16_t raw_left_16 = in[i*2];
        int16_t raw_right_16 = in[i*2+1];
        
        if (abs(raw_left_16) > (silence_threshold >> 14) || abs(raw_right_16) > (silence_threshold >> 14)) is_silent = false;
        
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

        if (abs(out_l_32) > peak_ol) peak_ol = abs(out_l_32);
        if (abs(out_r_32) > peak_or) peak_or = abs(out_r_32);
        if (abs(out_sub_32) > peak_sub) peak_sub = abs(out_sub_32);

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
        pdm_push_sample(delayed_sub, is_silent);
#endif
    }

    global_status.peaks[0] = (uint16_t)(peak_ml >> 13);
    global_status.peaks[1] = (uint16_t)(peak_mr >> 13);
    global_status.peaks[2] = (uint16_t)(peak_ol >> 13);
    global_status.peaks[3] = (uint16_t)(peak_or >> 13);
    global_status.peaks[4] = (uint16_t)(peak_sub >> 13);

    if (audio_buffer) give_audio_buffer(producer_pool, audio_buffer);
    
    usb_grow_transfer(ep->current_transfer, 1);
    usb_packet_done(ep);

    uint32_t end_time = timer_hw->timerawl;
    global_status.cpu0_load = (uint8_t)((end_time - start_time) / 10);
}

// ----------------------------------------------------------------------------
// FEEDBACK & DRIFT CORRECTION
// ----------------------------------------------------------------------------

static void __not_in_flash_func(_as_sync_packet)(struct usb_endpoint *ep) {
    struct usb_buffer *buffer = usb_current_in_packet_buffer(ep);
    buffer->data_len = 3;

    uint32_t nominal = (audio_state.freq << 14u) / 1000u;
    uint32_t feedback;

    if (sync_started && total_samples_produced > audio_state.freq) {
        uint64_t now_us = time_us_64();
        uint64_t elapsed_us = now_us - start_time_us;
        uint64_t expected_samples = (elapsed_us * audio_state.freq) / 1000000;
        int32_t drift = (int32_t)(total_samples_produced - expected_samples);
        
        int32_t correction = (drift * 50) / 1000;
        
        if (correction > 500) correction = 500;
        if (correction < -500) correction = -500;
        
        feedback = nominal - correction;
    } else {
        feedback = nominal;
    }

    buffer->data[0] = feedback;
    buffer->data[1] = feedback >> 8u;
    buffer->data[2] = feedback >> 16u;
    
    usb_grow_transfer(ep->current_transfer, 1);
    usb_packet_done(ep);
}

// ----------------------------------------------------------------------------
// CONTROL REQUESTS
// ----------------------------------------------------------------------------

static const struct usb_transfer_type as_transfer_type = { .on_packet = _as_audio_packet, .initial_packet_count = 1 };
static const struct usb_transfer_type as_sync_transfer_type = { .on_packet = _as_sync_packet, .initial_packet_count = 1 };
static struct usb_transfer as_transfer;
static struct usb_transfer as_sync_transfer;

static struct audio_control_cmd { uint8_t cmd; uint8_t type; uint8_t cs; uint8_t cn; uint8_t unit; uint8_t len; } audio_control_cmd_t;

static void _audio_reconfigure();

static void audio_cmd_packet(struct usb_endpoint *ep) {
    struct usb_buffer *buffer = usb_current_out_packet_buffer(ep);
    if (!buffer) { usb_start_empty_control_in_transfer_null_completion(); return; }

    // Vendor commands (Shared by Interface 0 (macOS) and Interface 2 (WinUSB))
    if (audio_control_cmd_t.cmd == REQ_SET_EQ_PARAM && buffer->data_len >= sizeof(EqParamPacket)) {
        memcpy((void*)&pending_packet, buffer->data, sizeof(EqParamPacket));
        eq_update_pending = true;
    }
    if (audio_control_cmd_t.cmd == REQ_SET_PREAMP && buffer->data_len >= 4) {
        float db;
        memcpy(&db, buffer->data, 4);
        global_preamp_db = db;
        float linear = powf(10.0f, db / 20.0f);
        global_preamp_mul = (int32_t)(linear * (float)(1<<28));
    }
    if (audio_control_cmd_t.cmd == REQ_SET_DELAY && buffer->data_len >= 4) {
        uint8_t ch = audio_control_cmd_t.cn;
        if (ch < NUM_CHANNELS) {
            float ms;
            memcpy(&ms, buffer->data, 4);
            if (ms < 0) ms = 0;
            channel_delays_ms[ch] = ms;
            dsp_update_delay_samples((float)audio_state.freq);
        }
    }
    if (audio_control_cmd_t.cmd == REQ_SET_BYPASS && buffer->data_len >= 1) {
        bypass_master_eq = (buffer->data[0] != 0);
    }

    // Audio class commands
    if (audio_control_cmd_t.cmd == AUDIO_REQ_SetCurrent && buffer->data_len >= audio_control_cmd_t.len) {
        if (audio_control_cmd_t.type == USB_REQ_TYPE_RECIPIENT_INTERFACE) {
            switch (audio_control_cmd_t.cs) {
                case FEATURE_MUTE_CONTROL: audio_state.mute = buffer->data[0]; break;
                case FEATURE_VOLUME_CONTROL: { int16_t vol; memcpy(&vol, buffer->data, 2); audio_set_volume(vol); } break;
            }
        } else if (audio_control_cmd_t.type == USB_REQ_TYPE_RECIPIENT_ENDPOINT && audio_control_cmd_t.cs == 1u) {
            uint32_t new_freq; memcpy(&new_freq, buffer->data, 4); new_freq &= 0x00ffffffu;
            if (audio_state.freq != new_freq) { audio_state.freq = new_freq; _audio_reconfigure(); }
        }
    }
    usb_start_empty_control_in_transfer_null_completion();
}

static const struct usb_transfer_type _audio_cmd_transfer_type = { .on_packet = audio_cmd_packet, .initial_packet_count = 1 };

static bool as_set_alternate(struct usb_interface *interface, uint alt) { return alt < 2; }

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
        if ((setup->wValue >> 8u) == 1u) {
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

// ----------------------------------------------------------------------------
// SETUP REQUEST HANDLERS
// ----------------------------------------------------------------------------

bool _as_setup_request_handler(__unused struct usb_endpoint *ep, struct usb_setup_packet *setup) {
    if (USB_REQ_TYPE_TYPE_CLASS == (setup->bmRequestType & USB_REQ_TYPE_TYPE_MASK)) {
        switch (setup->bRequest) {
            case AUDIO_REQ_SetCurrent:    return do_set_current(setup);
            case AUDIO_REQ_GetCurrent:    return do_get_current(setup);
            case AUDIO_REQ_GetMinimum:    return do_get_minimum(setup);
            case AUDIO_REQ_GetMaximum:    return do_get_maximum(setup);
            case AUDIO_REQ_GetResolution: return do_get_resolution(setup);
            default: break;
        }
    }
    return false;
}

static bool handle_vendor_request(struct usb_setup_packet *setup) {
    if (setup->bRequest == REQ_SET_EQ_PARAM) {
        audio_control_cmd_t.cmd = REQ_SET_EQ_PARAM;
        usb_start_control_out_transfer(&_audio_cmd_transfer_type);
        return true;
    }
    if (setup->bRequest == REQ_SET_PREAMP && setup->wLength == 4) {
        audio_control_cmd_t.cmd = REQ_SET_PREAMP;
        usb_start_control_out_transfer(&_audio_cmd_transfer_type);
        return true;
    }
    if (setup->bRequest == REQ_GET_PREAMP) {
        uint32_t val;
        float current_db = global_preamp_db;
        memcpy(&val, &current_db, 4);
        usb_start_tiny_control_in_transfer(val, 4);
        return true;
    }
    
    if (setup->bRequest == REQ_SET_DELAY && setup->wLength == 4) {
        audio_control_cmd_t.cmd = REQ_SET_DELAY;
        audio_control_cmd_t.cn = (uint8_t)setup->wValue;
        usb_start_control_out_transfer(&_audio_cmd_transfer_type);
        return true;
    }
    if (setup->bRequest == REQ_GET_DELAY) {
        uint8_t ch = (uint8_t)setup->wValue;
        if (ch < NUM_CHANNELS) {
            uint32_t val;
            memcpy(&val, &channel_delays_ms[ch], 4);
            usb_start_tiny_control_in_transfer(val, 4);
            return true;
        }
    }

    if (setup->bRequest == REQ_SET_BYPASS && setup->wLength == 1) {
        audio_control_cmd_t.cmd = REQ_SET_BYPASS;
        usb_start_control_out_transfer(&_audio_cmd_transfer_type);
        return true;
    }
    if (setup->bRequest == REQ_GET_BYPASS) {
        usb_start_tiny_control_in_transfer(bypass_master_eq ? 1 : 0, 1);
        return true;
    }
    
    if (setup->bRequest == REQ_GET_STATUS) {
        uint32_t resp = 0;
        if (setup->wValue == 0) {
            resp = (uint32_t)global_status.peaks[0] | ((uint32_t)global_status.peaks[1] << 16);
        } else if (setup->wValue == 1) {
            resp = (uint32_t)global_status.peaks[2] | ((uint32_t)global_status.peaks[3] << 16);
        } else if (setup->wValue == 2) {
            resp = (uint32_t)global_status.peaks[4] | ((uint32_t)global_status.cpu0_load << 16) | ((uint32_t)global_status.cpu1_load << 24);
        }
        usb_start_tiny_control_in_transfer(resp, 4);
        return true;
    }

    if (setup->bRequest == REQ_GET_EQ_PARAM) {
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
    }
    return false;
}

static bool ac_setup_request_handler(__unused struct usb_interface *interface, struct usb_setup_packet *setup) {
    // Legacy support for macOS (Interface 0)
    if ((setup->bmRequestType & USB_REQ_TYPE_TYPE_MASK) == USB_REQ_TYPE_TYPE_VENDOR) {
        return handle_vendor_request(setup);
    }

    // Handle audio class requests
    if (USB_REQ_TYPE_TYPE_CLASS == (setup->bmRequestType & USB_REQ_TYPE_TYPE_MASK)) {
        switch (setup->bRequest) {
            case AUDIO_REQ_SetCurrent:    return do_set_current(setup);
            case AUDIO_REQ_GetCurrent:    return do_get_current(setup);
            case AUDIO_REQ_GetMinimum:    return do_get_minimum(setup);
            case AUDIO_REQ_GetMaximum:    return do_get_maximum(setup);
            case AUDIO_REQ_GetResolution: return do_get_resolution(setup);
            default: break;
        }
    }
    return false;
}

// Vendor Interface Handler (WinUSB) - Interface 2
static bool vendor_setup_request_handler(__unused struct usb_interface *interface, struct usb_setup_packet *setup) {
    // Only accept Vendor requests targeting this Interface
    if ((setup->bmRequestType & USB_REQ_TYPE_TYPE_MASK) == USB_REQ_TYPE_TYPE_VENDOR &&
        (setup->bmRequestType & USB_REQ_TYPE_RECIPIENT_MASK) == USB_REQ_TYPE_RECIPIENT_INTERFACE) {
        return handle_vendor_request(setup);
    }
    return false;
}

// ----------------------------------------------------------------------------
// DEVICE-LEVEL SETUP HANDLER FOR WCID
// ----------------------------------------------------------------------------

// Microsoft OS String Descriptor - raw bytes ready to send
static const uint8_t ms_os_string_descriptor[] = {
    0x12,                   // bLength (18 bytes)
    0x03,                   // bDescriptorType (STRING)
    'M', 0, 'S', 0,         // qwSignature "MSFT100"
    'F', 0, 'T', 0,
    '1', 0, '0', 0,
    '0', 0,
    MS_VENDOR_CODE,         // bMS_VendorCode
    0x00                    // bPad
};

// Microsoft Compatible ID Feature Descriptor
static const uint8_t ms_compat_id_descriptor[] = {
    // Header (16 bytes)
    0x28, 0x00, 0x00, 0x00, // dwLength (40 bytes total) - LE
    0x00, 0x01,             // bcdVersion (1.00) - LE
    0x04, 0x00,             // wIndex (0x0004 = Compatible ID) - LE
    0x01,                   // bCount (1 section)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // reserved (7 bytes)
    // Function Section (24 bytes)
    VENDOR_INTERFACE_NUMBER, // bFirstInterfaceNumber
    0x01,                   // reserved (must be 1)
    'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,  // compatibleID (8 bytes)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // subCompatibleID (8 bytes)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00  // reserved (6 bytes)
};

// Microsoft Extended Properties Feature Descriptor
static const uint8_t ms_ext_prop_descriptor[] = {
    // Header (10 bytes)
    0x8E, 0x00, 0x00, 0x00, // dwLength (142 bytes) - LE
    0x00, 0x01,             // bcdVersion (1.00) - LE
    0x05, 0x00,             // wIndex (0x0005 = Extended Properties) - LE
    0x01, 0x00,             // wCount (1 property) - LE
    // Property Section (132 bytes)
    0x84, 0x00, 0x00, 0x00, // dwSize (132 bytes) - LE
    0x01, 0x00, 0x00, 0x00, // dwPropertyDataType (1 = REG_SZ) - LE
    0x28, 0x00,             // wPropertyNameLength (40 bytes) - LE
    // Property name: "DeviceInterfaceGUID" in UTF-16LE (40 bytes with null terminator)
    'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00,
    'I', 0x00, 'n', 0x00, 't', 0x00, 'e', 0x00, 'r', 0x00, 'f', 0x00,
    'a', 0x00, 'c', 0x00, 'e', 0x00, 'G', 0x00, 'U', 0x00, 'I', 0x00,
    'D', 0x00, 0x00, 0x00,
    // dwPropertyDataLength (78 bytes) - LE
    0x4E, 0x00, 0x00, 0x00,
    // Property data: "{88BAE032-5A81-49F0-BC3D-A4FF138216D6}" in UTF-16LE (78 bytes with null)
    '{', 0x00, '8', 0x00, '8', 0x00, 'B', 0x00, 'A', 0x00, 'E', 0x00, '0', 0x00, '3', 0x00,
    '2', 0x00, '-', 0x00, '5', 0x00, 'A', 0x00, '8', 0x00, '1', 0x00, '-', 0x00, '4', 0x00,
    '9', 0x00, 'F', 0x00, '0', 0x00, '-', 0x00, 'B', 0x00, 'C', 0x00, '3', 0x00, 'D', 0x00,
    '-', 0x00, 'A', 0x00, '4', 0x00, 'F', 0x00, 'F', 0x00, '1', 0x00, '3', 0x00, '8', 0x00,
    '2', 0x00, '1', 0x00, '6', 0x00, 'D', 0x00, '6', 0x00, '}', 0x00, 0x00, 0x00
};

static bool device_setup_request_handler(struct usb_device *dev, struct usb_setup_packet *setup) {
    (void)dev;
    
    // Handle GET_DESCRIPTOR for MS OS String Descriptor (index 0xEE)
    if (setup->bmRequestType == 0x80 &&
        setup->bRequest == 0x06 &&
        setup->wValue == 0x03EE) {
        
        struct usb_buffer *buffer = usb_current_in_packet_buffer(&usb_control_in);
        uint16_t len = sizeof(ms_os_string_descriptor);
        if (len > setup->wLength) len = setup->wLength;
        memcpy(buffer->data, ms_os_string_descriptor, len);
        buffer->data_len = len;
        usb_start_single_buffer_control_in_transfer();
        return true;
    }
    
    // Handle Microsoft OS Descriptor vendor requests
    if ((setup->bmRequestType == 0xC0 || setup->bmRequestType == 0xC1) &&
        setup->bRequest == MS_VENDOR_CODE) {
        
        const uint8_t *desc = NULL;
        uint16_t desc_len = 0;
        
        switch (setup->wIndex) {
            case 0x0004:  // Compatible ID Feature Descriptor
                desc = ms_compat_id_descriptor;
                desc_len = sizeof(ms_compat_id_descriptor);
                break;
                
            case 0x0005:  // Extended Properties Feature Descriptor
                desc = ms_ext_prop_descriptor;
                desc_len = sizeof(ms_ext_prop_descriptor);
                break;
        }
        
        if (desc && desc_len > 0) {
            struct usb_buffer *buffer = usb_current_in_packet_buffer(&usb_control_in);
            uint16_t len = desc_len;
            if (len > setup->wLength) len = setup->wLength;
            memcpy(buffer->data, desc, len);
            buffer->data_len = len;
            usb_start_single_buffer_control_in_transfer();
            return true;
        }
    }
    
    return false;  // Let default handler process other requests
}

// ----------------------------------------------------------------------------
// INIT
// ----------------------------------------------------------------------------
static struct usb_interface ac_interface;
static struct usb_interface as_op_interface;
static struct usb_interface vendor_interface;
static struct usb_endpoint ep_op_out, ep_op_sync;

// S/PDIF Config
struct audio_spdif_config config = { .pin = PICO_AUDIO_SPDIF_PIN, .dma_channel = 0, .pio_sm = 0 };
struct audio_buffer_format producer_format = { .format = &audio_format_48k, .sample_stride = 4 };
#define AUDIO_BUFFER_COUNT 8

static void _audio_reconfigure() {
    rate_change_pending = true;
    pending_rate = audio_state.freq;
}

void usb_sound_card_init() {
    // S/PDIF Setup
    producer_pool = audio_new_producer_pool(&producer_format, AUDIO_BUFFER_COUNT, 192);
    audio_spdif_setup(&audio_format_48k, &config);
    audio_spdif_connect_extra(producer_pool, false, AUDIO_BUFFER_COUNT / 2, NULL);
    irq_set_priority(DMA_IRQ_0 + PICO_AUDIO_SPDIF_DMA_IRQ, PICO_HIGHEST_IRQ_PRIORITY);
    audio_spdif_set_enabled(true);

    // USB Setup - Audio Control Interface
    usb_interface_init(&ac_interface, &audio_device_config.ac_interface, NULL, 0, true);
    ac_interface.setup_request_handler = ac_setup_request_handler;
    
    // Audio Streaming Interface
    static struct usb_endpoint *const op_endpoints[] = { &ep_op_out, &ep_op_sync };
    usb_interface_init(&as_op_interface, &audio_device_config.as_op_interface, op_endpoints, count_of(op_endpoints), true);
    as_op_interface.set_alternate_handler = as_set_alternate;
    ep_op_out.setup_request_handler = _as_setup_request_handler;
    as_transfer.type = &as_transfer_type;
    usb_set_default_transfer(&ep_op_out, &as_transfer);
    as_sync_transfer.type = &as_sync_transfer_type;
    usb_set_default_transfer(&ep_op_sync, &as_sync_transfer);
    
    // Vendor Interface (WinUSB - Control Only)
    // No endpoints to initialize
    usb_interface_init(&vendor_interface, &audio_device_config.vendor_interface, NULL, 0, true);
    vendor_interface.setup_request_handler = vendor_setup_request_handler;
    
    // Device init with all 3 interfaces
    static struct usb_interface *const boot_device_interfaces[] = {
        &ac_interface,
        &as_op_interface,
        &vendor_interface
    };
    
    struct usb_device *device = usb_device_init(
        &boot_device_descriptor,
        &audio_device_config.descriptor,
        boot_device_interfaces,
        count_of(boot_device_interfaces),
        _get_descriptor_string
    );
    
    // Set device-level setup handler for WCID requests
    device->setup_request_handler = device_setup_request_handler;
    
    dsp_init_default_filters();
    dsp_recalculate_all_filters(48000.0f);
    audio_set_volume(DEFAULT_VOLUME);
    _audio_reconfigure();
    usb_device_start();
}
