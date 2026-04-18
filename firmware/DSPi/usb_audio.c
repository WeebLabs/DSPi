/*
 * USB Audio Implementation for DSPi
 * UAC1 Audio Streaming with DSP Pipeline
 *
 * Phase 1: migrated from pico-extras usb_device → TinyUSB.  TinyUSB's built-in
 * audio class driver is UAC2-only (audio_device.c:1576 rejects bInterfaceProtocol
 * != V2), so DSPi registers its own UAC1 class driver via usbd_app_driver_get_cb().
 * The vendor control interface has been dropped temporarily and will return in
 * Phase 2.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/audio.h"
#include "pico/audio_spdif.h"
#include "pico/audio_i2s_multi.h"
#include "hardware/sync.h"
#include "pico/bootrom.h"
#include "hardware/irq.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/vreg.h"

#include "tusb.h"
#include "device/usbd_pvt.h"
#include "class/audio/audio.h"

#include "usb_audio.h"
#include "audio_pipeline.h"
#include "usb_descriptors.h"
#include "dsp_pipeline.h"
#include "dcp_inline.h"
#include "pdm_generator.h"
#include "flash_storage.h"
#include "loudness.h"
#include "crossfeed.h"
#include "leveller.h"
#include "bulk_params.h"
#include "usb_audio_ring.h"
#include "usb_feedback_controller.h"
#include "vendor_commands.h"

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
volatile bool bulk_params_pending = false;

// Output type switching — deferred to main loop (needs heap allocation).
// Per-slot bitmask supports back-to-back requests without dropping any.
volatile uint8_t output_type_change_mask = 0;                   // Bit N = slot N has pending change
volatile uint8_t pending_output_types[NUM_SPDIF_INSTANCES];     // New type per slot
// USB stream restart (alt 0 -> alt > 0) — deferred to main loop for safe pipeline re-lock
volatile bool stream_restart_resync_pending = false;

// Preset operations — deferred to main loop so that:
//  1. Flash writes (preset_save/delete/dir_flush) don't run in USB IRQ context,
//     avoiding a ~45ms interrupt blackout inside an ISR.
//  2. preset_load can be bracketed with prepare_pipeline_reset() /
//     complete_pipeline_reset() to drain stale consumer buffers and resync
//     all outputs.  Without this, buffers containing audio processed with the
//     OLD preset's parameters play out for ~24ms after the new preset is applied.
//  3. Delay line contents from the old preset are zeroed, preventing stale audio
//     from bleeding through when delay length changes.
volatile bool preset_load_pending = false;
volatile uint8_t pending_preset_load_slot = 0;
volatile bool save_params_pending = false;   // Legacy REQ_SAVE_PARAMS (deferred)
volatile bool preset_save_pending = false;
volatile uint8_t pending_preset_save_slot = 0;
volatile uint16_t preset_delete_mask = 0;  // Bitmask of slots pending delete
volatile bool factory_reset_pending = false;

// SPSC ring buffer: USB audio ISR pushes raw packets, main loop consumes
// and runs the DSP pipeline.  Placed in RAM for flash-operation safety.
static usb_audio_ring_t __not_in_flash("audio_ring") audio_ring;

// USB packet arrival timestamp for gap detection.  File-scope (not function-
// local) so it can be reset to 0 on stream lifecycle transitions in
// as_set_alternate() and usb_audio_flush_ring().
static volatile uint32_t audio_ring_last_push_us = 0;

// Deferred fire-and-forget flash SET commands.
// Separate pending flags per command type prevent cross-command clobbering.
// Same-command back-to-back is last-writer-wins (correct for idempotent settings).
// Known limitation: SET_NAME for different slots in rapid succession can lose
// one update.  In practice, host apps serialize preset edits.
volatile bool flash_set_name_pending = false;
uint8_t flash_set_name_slot = 0;
char    flash_set_name_buf[PRESET_NAME_LEN];

volatile bool flash_set_startup_pending = false;
uint8_t flash_set_startup_mode = 0;
uint8_t flash_set_startup_slot = 0;

volatile bool flash_set_include_pins_pending = false;
uint8_t flash_set_include_pins_val = 0;

// Deferred include_master_volume directory update (flash write must happen on main loop)
volatile bool flash_set_include_master_vol_pending = false;
uint8_t flash_set_include_master_vol_val = 0;

// 4 KB aligned buffer shared between GET and SET bulk param transfers.
uint8_t __attribute__((aligned(4))) bulk_param_buf[WIRE_BULK_BUF_SIZE];

// Per-input-channel preamp gain.  Indexed by input channel (0=USB L, 1=USB R).
// Arrays sized by NUM_INPUT_CHANNELS so adding future inputs (e.g. S/PDIF)
// only requires changing that constant.
volatile float global_preamp_db[NUM_INPUT_CHANNELS]      = {[0 ... NUM_INPUT_CHANNELS-1] = 0.0f};
volatile int32_t global_preamp_mul[NUM_INPUT_CHANNELS]    = {[0 ... NUM_INPUT_CHANNELS-1] = 268435456};  // Unity = 1<<28 (Q28)
volatile float global_preamp_linear[NUM_INPUT_CHANNELS]   = {[0 ... NUM_INPUT_CHANNELS-1] = 1.0f};

// Master volume — device-side ceiling on all output.  Applied post-output-gain,
// does NOT affect loudness compensation, leveller, or any other DSP stage.
// Range: MASTER_VOL_MIN_DB (-127) to MASTER_VOL_MAX_DB (0), with
// MASTER_VOL_MUTE_DB (-128) as sentinel for true silence.
volatile float master_volume_db       = MASTER_VOL_MAX_DB;   // 0 dB = unity (no attenuation)
volatile float master_volume_linear   = 1.0f;
volatile int32_t master_volume_q15    = 32768;                // Unity in Q15 (for RP2040 path)

// Per-channel gain and mute (legacy 3-channel interface for flash compatibility)
volatile float channel_gain_db[3] = {0.0f, 0.0f, 0.0f};
volatile int32_t channel_gain_mul[3] = {32768, 32768, 32768};  // Unity = 2^15
volatile float channel_gain_linear[3] = {1.0f, 1.0f, 1.0f};
volatile bool channel_mute[3] = {false, false, false};

// Matrix Mixer State
MatrixMixer matrix_mixer = {0};

// Loudness compensation state
volatile bool loudness_enabled = false;
volatile float loudness_ref_spl = 83.0f;
volatile float loudness_intensity_pct = 100.0f;
volatile bool loudness_recompute_pending = false;

const LoudnessCoeffs *current_loudness_coeffs = NULL;

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

// Volume Leveller state
volatile LevellerConfig leveller_config = {
    .enabled = LEVELLER_DEFAULT_ENABLED,
    .amount = LEVELLER_DEFAULT_AMOUNT,
    .speed = LEVELLER_DEFAULT_SPEED,
    .max_gain_db = LEVELLER_DEFAULT_MAX_GAIN_DB,
    .lookahead = LEVELLER_DEFAULT_LOOKAHEAD,
    .gate_threshold_db = LEVELLER_DEFAULT_GATE_DB
};
volatile bool leveller_update_pending = false;
volatile bool leveller_reset_pending = false;
volatile bool leveller_bypassed = true;  // Fast bypass flag for audio callback

// Per-channel user-configurable names
char channel_names[NUM_CHANNELS][PRESET_NAME_LEN];

void get_default_channel_name(int ch, char *buf) {
    memset(buf, 0, PRESET_NAME_LEN);
#if PICO_RP2350
    static const char *defaults[] = {
        "USB L", "USB R",
        "SPDIF 1 L", "SPDIF 1 R", "SPDIF 2 L", "SPDIF 2 R",
        "SPDIF 3 L", "SPDIF 3 R", "SPDIF 4 L", "SPDIF 4 R",
        "PDM"
    };
#else
    static const char *defaults[] = {
        "USB L", "USB R",
        "SPDIF 1 L", "SPDIF 1 R", "SPDIF 2 L", "SPDIF 2 R",
        "PDM"
    };
#endif
    if (ch >= 0 && ch < NUM_CHANNELS) {
        strncpy(buf, defaults[ch], PRESET_NAME_LEN - 1);
    }
}

// ---------------------------------------------------------------------------
// Preamp & Master Volume helpers
// ---------------------------------------------------------------------------

// Update a single input channel's preamp gain from a dB value.
// Computes both float (RP2350) and Q28 (RP2040) representations so the
// audio callback can read the correct format without conversion.
void update_preamp(uint8_t ch, float db) {
    if (!isfinite(db)) return;  // Reject NaN/Inf — would propagate through entire audio path
    global_preamp_db[ch] = db;
    float linear = powf(10.0f, db / 20.0f);
    global_preamp_mul[ch]    = (int32_t)(linear * (float)(1 << 28));
    global_preamp_linear[ch] = linear;
}

// Update the device-side master volume from a dB value.
// Clamps to [MASTER_VOL_MUTE_DB .. MASTER_VOL_MAX_DB].
// MASTER_VOL_MUTE_DB (-128) is a sentinel meaning true silence (−∞ dB).
void update_master_volume(float db) {
    if (!isfinite(db)) return;  // Reject NaN/Inf — would zero-out or corrupt all output
    if (db < MASTER_VOL_MUTE_DB) db = MASTER_VOL_MUTE_DB;
    if (db > MASTER_VOL_MAX_DB)  db = MASTER_VOL_MAX_DB;
    master_volume_db = db;
    if (db <= MASTER_VOL_MUTE_DB) {
        // Mute sentinel — true silence
        master_volume_linear = 0.0f;
        master_volume_q15    = 0;
    } else {
        float linear = powf(10.0f, db / 20.0f);
        master_volume_linear = linear;
        master_volume_q15    = (int32_t)(linear * 32768.0f);
    }
}

// Sync State
volatile uint64_t total_samples_produced = 0;
volatile uint64_t start_time_us = 0;
volatile bool sync_started = false;
static volatile uint64_t last_packet_time_us = 0;
static volatile uint8_t usb_input_bit_depth = 16;
#define AUDIO_GAP_THRESHOLD_US 50000  // 50ms - reset sync if packets stop this long

// Consumer fill for instance 0 — used by watermark monitoring only
// (no longer part of the active feedback path).
volatile uint8_t spdif0_consumer_fill = 0;

// Audio Pools (S/PDIF stereo pairs)
struct audio_buffer_pool *producer_pool_1 = NULL;  // S/PDIF 1 (Out 1-2)
struct audio_buffer_pool *producer_pool_2 = NULL;  // S/PDIF 2 (Out 3-4)
#if PICO_RP2350
struct audio_buffer_pool *producer_pool_3 = NULL;  // S/PDIF 3 (Out 5-6)
struct audio_buffer_pool *producer_pool_4 = NULL;  // S/PDIF 4 (Out 7-8)
#endif
struct audio_format audio_format_48k = { .format = AUDIO_BUFFER_FORMAT_PCM_S32, .sample_freq = 48000, .channel_count = 2 };

// Legacy aliases
#define producer_pool producer_pool_1
#define sub_producer_pool producer_pool_2

// ----------------------------------------------------------------------------
// VOLUME
// ----------------------------------------------------------------------------
static uint16_t db_to_vol[CENTER_VOLUME_INDEX + 1] = {
    // Index 0 = silent (slider bottom), index 1 = -59 dB, ..., index 60 = 0 dB
    0x0000, 0x0025, 0x0029, 0x002e, 0x0034, 0x003a, 0x0041, 0x0049,
    0x0052, 0x005c, 0x0068, 0x0074, 0x0082, 0x0092, 0x00a4, 0x00b8,
    0x00cf, 0x00e8, 0x0104, 0x0124, 0x0148, 0x0170, 0x019d, 0x01cf,
    0x0207, 0x0247, 0x028e, 0x02de, 0x0337, 0x039c, 0x040c, 0x048b,
    0x0519, 0x05b8, 0x066a, 0x0733, 0x0814, 0x0910, 0x0a2b, 0x0b68,
    0x0ccd, 0x0e5d, 0x101d, 0x1215, 0x1449, 0x16c3, 0x198a, 0x1ca8,
    0x2027, 0x2413, 0x287a, 0x2d6b, 0x32f5, 0x392d, 0x4027, 0x47fb,
    0x50c3, 0x5a9e, 0x65ad, 0x7215, 0x8000
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
    if (volume >= (CENTER_VOLUME_INDEX + 1) * 256) volume = (CENTER_VOLUME_INDEX + 1) * 256 - 1;
    uint8_t vol_index = ((uint16_t)volume) >> 8u;
    audio_state.vol_mul = db_to_vol[vol_index];

    // Update loudness compensation coefficients for this volume step
    if (loudness_enabled && loudness_active_table) {
        current_loudness_coeffs = loudness_active_table[vol_index];
    }
}

// ----------------------------------------------------------------------------
// USB-specific wrapper: byte decode + gap detection, then pipeline
// (process_input_block is in audio_pipeline.c)
// ----------------------------------------------------------------------------
static void __not_in_flash_func(process_audio_packet)(const uint8_t *data, uint16_t data_len) {
    // USB format snapshot
    const uint8_t bit_depth = usb_input_bit_depth;  // snapshot once — avoid double-read of volatile
    uint32_t bytes_per_frame = (bit_depth == 24) ? 6 : 4;
    uint32_t sample_count = data_len / bytes_per_frame;

    // USB-specific gap detection + sync tracking
    // NOTE: USB packet gap detection has moved to _as_audio_packet() (ISR
    // context) where it measures actual packet arrival timing rather than
    // main-loop processing timing.  See audio_ring_last_push_us.
    uint64_t now_us = time_us_64();
    if (sync_started && last_packet_time_us > 0 &&
        (now_us - last_packet_time_us) > AUDIO_GAP_THRESHOLD_US) {
        sync_started = false;
        total_samples_produced = 0;
        pipeline_reset_cpu_metering();
    }
    last_packet_time_us = now_us;
    if (!sync_started) {
        start_time_us = now_us;
        sync_started = true;
    }
    total_samples_produced += sample_count;

    // PASS 1: USB byte decode → buf_l/buf_r + preamp
#if PICO_RP2350
    {
        float preamp_l = global_preamp_linear[0];
        float preamp_r = global_preamp_linear[1];
        const float inv_32768 = 1.0f / 32768.0f;
        if (bit_depth == 24) {
            const uint8_t *p = (const uint8_t *)data;
            const float inv_8388608 = 1.0f / 8388608.0f;
            for (uint32_t i = 0; i < sample_count; i++) {
                int32_t left  = (int32_t)((uint32_t)p[2] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[0] << 8) >> 8;
                int32_t right = (int32_t)((uint32_t)p[5] << 24 | (uint32_t)p[4] << 16 | (uint32_t)p[3] << 8) >> 8;
                buf_l[i] = (float)left * inv_8388608 * preamp_l;
                buf_r[i] = (float)right * inv_8388608 * preamp_r;
                p += 6;
            }
        } else {
            const int16_t *in = (const int16_t *)data;
            for (uint32_t i = 0; i < sample_count; i++) {
                buf_l[i] = (float)in[i*2] * inv_32768 * preamp_l;
                buf_r[i] = (float)in[i*2+1] * inv_32768 * preamp_r;
            }
        }
    }
#else
    {
        int32_t preamp_l = global_preamp_mul[0];
        int32_t preamp_r = global_preamp_mul[1];
        if (bit_depth == 24) {
            const uint8_t *p = (const uint8_t *)data;
            for (uint32_t i = 0; i < sample_count; i++) {
                // 24-bit -> Q28: left-justify to [31:8] then >>2 = net <<6
                int32_t raw_left_32  = (int32_t)((uint32_t)p[2] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[0] << 8) >> 2;
                int32_t raw_right_32 = (int32_t)((uint32_t)p[5] << 24 | (uint32_t)p[4] << 16 | (uint32_t)p[3] << 8) >> 2;
                buf_l[i] = fast_mul_q28(raw_left_32, preamp_l);
                buf_r[i] = fast_mul_q28(raw_right_32, preamp_r);
                p += 6;
            }
        } else {
            const int16_t *in = (const int16_t *)data;
            for (uint32_t i = 0; i < sample_count; i++) {
                int32_t raw_left_32 = (int32_t)in[i*2] << 14;
                int32_t raw_right_32 = (int32_t)in[i*2+1] << 14;
                buf_l[i] = fast_mul_q28(raw_left_32, preamp_l);
                buf_r[i] = fast_mul_q28(raw_right_32, preamp_r);
            }
        }
    }
#endif

    process_input_block(sample_count);
}

// ----------------------------------------------------------------------------
// USB AUDIO RING BUFFER — PUBLIC WRAPPERS
// ----------------------------------------------------------------------------

// Drain all pending packets from the ring, running the DSP pipeline for
// each.  Called as the first operation in the main loop and before any
// disruptive deferred operation (rate change, output type switch, etc.).
void usb_audio_drain_ring(void) {
    usb_audio_slot_t *slot;
    while ((slot = usb_audio_ring_peek(&audio_ring)) != NULL) {
        process_audio_packet(slot->data, slot->data_len);
        usb_audio_ring_consume(&audio_ring);
    }
}

// Discard all pending ring data and reset gap-detection timestamp.
// Used on stream stop/start transitions to flush stale packets from a
// previous stream.
void usb_audio_flush_ring(void) {
    usb_audio_ring_flush(&audio_ring);
    audio_ring_last_push_us = 0;
}

// Ring overrun accessor (audio_ring is static)
uint32_t usb_audio_ring_overrun_count(void) {
    return audio_ring.overrun_count;
}

// Derive Core 1 mode from current output enable state.
// (Moved from vendor_commands.c during the Phase 1 TinyUSB migration — still
// needed by main.c and flash_storage.c for preset-load Core 1 transitions.)
Core1Mode derive_core1_mode(void) {
    if (matrix_mixer.outputs[NUM_OUTPUT_CHANNELS - 1].enabled)
        return CORE1_MODE_PDM;
    for (int out = CORE1_EQ_FIRST_OUTPUT; out <= CORE1_EQ_LAST_OUTPUT; out++) {
        if (matrix_mixer.outputs[out].enabled)
            return CORE1_MODE_EQ_WORKER;
    }
    return CORE1_MODE_IDLE;
}

// ----------------------------------------------------------------------------
// UAC1 CLASS DRIVER (custom TinyUSB class driver, registered via
// usbd_app_driver_get_cb). TinyUSB's built-in audio class driver is UAC2-only
// (audio_device.c:1576), so we provide our own UAC1 implementation without
// patching vendored SDK code.
// ----------------------------------------------------------------------------

// Endpoint buffers.  Must live in RAM; reused across every transfer.
// Audio data OUT: sized for worst-case (24-bit 96 kHz + 1 jitter sample).
// Feedback IN: 4 bytes (actual payload 3, DCD requires 4-byte iso alloc).
static uint8_t __attribute__((aligned(4))) __not_in_flash("audio_scratch") ep_out_buf[AUDIO_EP_MAX_PKT];
static uint8_t __attribute__((aligned(4))) __not_in_flash("audio_scratch") ep_fb_buf[4];

// Control request scratch for SET_CUR data stage (1-3 bytes payload).
static uint8_t uac1_ctrl_buf[8];

// Class driver state (AC+AS audio function + vendor interface).
static struct {
    uint8_t ac_itf;
    uint8_t as_itf;
    uint8_t vendor_itf;      // 0xFF = not claimed
    uint8_t cur_alt;         // Current AS alt setting (0, 1, or 2)
    bool    ep_data_open;
    bool    ep_fb_open;
    // Deferred SET_CUR context (captured at SETUP, applied at DATA)
    uint8_t pending_cs;
    uint8_t pending_recipient;
    uint8_t pending_len;
} uac1 = { .vendor_itf = 0xFF };

extern usb_feedback_ctrl_t fb_ctrl;

// Forward decls.
static void uac1_driver_init(void);
static bool uac1_driver_deinit(void);
static void uac1_driver_reset(uint8_t rhport);
static uint16_t uac1_driver_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len);
static bool uac1_driver_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *req);
static bool uac1_driver_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes);
static void uac1_driver_sof(uint8_t rhport, uint32_t frame_count);

static const usbd_class_driver_t uac1_driver = {
    .name            = "DSPi_UAC1",
    .init            = uac1_driver_init,
    .deinit          = uac1_driver_deinit,
    .reset           = uac1_driver_reset,
    .open            = uac1_driver_open,
    .control_xfer_cb = uac1_driver_control_xfer_cb,
    .xfer_cb         = uac1_driver_xfer_cb,
    .sof             = uac1_driver_sof,
};

// TinyUSB looks up this weak symbol during tud_init(). Returning our driver
// here makes TinyUSB dispatch all interface/endpoint events for our AC+AS
// interfaces through our callbacks.
usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count) {
    *driver_count = 1;
    return &uac1_driver;
}

// ----------------------------------------------------------------------------
// Class driver implementation
// ----------------------------------------------------------------------------

static void uac1_driver_init(void) {
    memset(&uac1, 0, sizeof(uac1));
}

static bool uac1_driver_deinit(void) {
    return true;
}

static void uac1_driver_reset(uint8_t rhport) {
    (void)rhport;
    uac1.ep_data_open = false;
    uac1.ep_fb_open = false;
    uac1.cur_alt = 0;
    uac1.vendor_itf = 0xFF;
    usb_audio_alt_set = 0;
}

static uint16_t uac1_driver_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len) {
    // Second call path: vendor interface (class 0xFF, 0 EPs, no CS descriptors).
    // TinyUSB calls open() again with this interface after we've claimed AC+AS.
    if (itf_desc->bInterfaceClass == 0xFF &&
        itf_desc->bAlternateSetting == 0 &&
        itf_desc->bNumEndpoints == 0) {
        uac1.vendor_itf = itf_desc->bInterfaceNumber;
        return tu_desc_len(itf_desc);  // 9 bytes
    }

    // Claim the UAC1 AC interface (don't check bInterfaceProtocol — UAC1 uses 0x00).
    TU_VERIFY(itf_desc->bInterfaceClass == TUSB_CLASS_AUDIO);
    TU_VERIFY(itf_desc->bInterfaceSubClass == AUDIO_SUBCLASS_CONTROL);
    TU_VERIFY(itf_desc->bAlternateSetting == 0);

    uac1.ac_itf = itf_desc->bInterfaceNumber;

    uint8_t const *p_desc = (uint8_t const *)itf_desc;
    uint8_t const *p_end  = p_desc + max_len;
    uint16_t drv_len = 0;

    // Skip AC standard interface descriptor.
    drv_len += tu_desc_len(p_desc);
    p_desc += tu_desc_len(p_desc);

    // Walk AC class-specific descriptors (header, input terminal, feature unit, output terminal).
    while (p_desc < p_end && tu_desc_type(p_desc) == TUSB_DESC_CS_INTERFACE) {
        drv_len += tu_desc_len(p_desc);
        p_desc += tu_desc_len(p_desc);
    }

    // Walk all AS alt settings (0, 1, 2) and their endpoints.
    // Reserve worst-case DPRAM for the data OUT and feedback IN endpoints.
#ifdef TUP_DCD_EDPT_ISO_ALLOC
    bool allocated_out = false;
    bool allocated_fb = false;
#endif

    while (p_desc < p_end && tu_desc_type(p_desc) == TUSB_DESC_INTERFACE) {
        tusb_desc_interface_t const *as = (tusb_desc_interface_t const *)p_desc;
        if (as->bInterfaceClass != TUSB_CLASS_AUDIO ||
            as->bInterfaceSubClass != AUDIO_SUBCLASS_STREAMING) {
            break;
        }
        uac1.as_itf = as->bInterfaceNumber;

        drv_len += tu_desc_len(p_desc);
        p_desc += tu_desc_len(p_desc);

        // Consume this alt's class-specific + endpoint descriptors up to the
        // next standard interface descriptor or end.
        while (p_desc < p_end && tu_desc_type(p_desc) != TUSB_DESC_INTERFACE) {
#ifdef TUP_DCD_EDPT_ISO_ALLOC
            if (tu_desc_type(p_desc) == TUSB_DESC_ENDPOINT) {
                tusb_desc_endpoint_t const *ep = (tusb_desc_endpoint_t const *)p_desc;
                if (ep->bmAttributes.xfer == TUSB_XFER_ISOCHRONOUS) {
                    uint16_t mps = tu_edpt_packet_size(ep);
                    uint8_t  ep_addr = ep->bEndpointAddress;
                    if (ep_addr == AUDIO_OUT_ENDPOINT && !allocated_out) {
                        usbd_edpt_iso_alloc(rhport, ep_addr, AUDIO_EP_MAX_PKT);
                        allocated_out = true;
                    } else if (ep_addr == AUDIO_IN_ENDPOINT && !allocated_fb) {
                        (void)mps;
                        usbd_edpt_iso_alloc(rhport, ep_addr, 4);
                        allocated_fb = true;
                    }
                }
            }
#endif
            drv_len += tu_desc_len(p_desc);
            p_desc += tu_desc_len(p_desc);
        }
    }

    // Enable SOF events for our driver (needed for feedback servo tick).
    usbd_sof_enable(rhport, SOF_CONSUMER_AUDIO, true);

    return drv_len;
}

// Arm a fresh data OUT xfer.  Called once on alt>0 activation and from xfer_cb
// completion.
static inline void uac1_arm_data_out(uint8_t rhport) {
    usbd_edpt_xfer(rhport, AUDIO_OUT_ENDPOINT, ep_out_buf, AUDIO_EP_MAX_PKT);
}

// Arm a fresh feedback IN xfer with the current 10.14 feedback value.
static inline void uac1_arm_feedback(uint8_t rhport) {
    uint32_t fb = feedback_10_14;
    if (fb == 0) fb = nominal_feedback_10_14;
    ep_fb_buf[0] = (uint8_t)(fb & 0xFF);
    ep_fb_buf[1] = (uint8_t)((fb >> 8) & 0xFF);
    ep_fb_buf[2] = (uint8_t)((fb >> 16) & 0xFF);
    ep_fb_buf[3] = 0;
    usbd_edpt_xfer(rhport, AUDIO_IN_ENDPOINT, ep_fb_buf, 3);
}

// Open isochronous endpoints for the specified alt (1 or 2).
static bool uac1_open_stream_eps(uint8_t rhport, uint8_t alt) {
    if (alt != 1 && alt != 2) return false;

    const uint8_t *data_ep = usb_audio_data_ep_desc[alt - 1];
    const uint8_t *fb_ep   = usb_audio_fb_ep_desc[alt - 1];

#ifdef TUP_DCD_EDPT_ISO_ALLOC
    TU_ASSERT(usbd_edpt_iso_activate(rhport, (tusb_desc_endpoint_t const *)data_ep));
    TU_ASSERT(usbd_edpt_iso_activate(rhport, (tusb_desc_endpoint_t const *)fb_ep));
#else
    TU_ASSERT(usbd_edpt_open(rhport, (tusb_desc_endpoint_t const *)data_ep));
    TU_ASSERT(usbd_edpt_open(rhport, (tusb_desc_endpoint_t const *)fb_ep));
#endif
    uac1.ep_data_open = true;
    uac1.ep_fb_open = true;

    uac1_arm_data_out(rhport);
    uac1_arm_feedback(rhport);
    return true;
}

static void uac1_close_stream_eps(uint8_t rhport) {
    if (uac1.ep_data_open) {
        usbd_edpt_close(rhport, AUDIO_OUT_ENDPOINT);
        uac1.ep_data_open = false;
    }
    if (uac1.ep_fb_open) {
        usbd_edpt_close(rhport, AUDIO_IN_ENDPOINT);
        uac1.ep_fb_open = false;
    }
}

// Apply a new AS alt setting (0, 1, or 2). Mirrors the old as_set_alternate().
static bool uac1_apply_alt(uint8_t rhport, uint8_t alt) {
    if (alt > 2) return false;

    uint32_t prev_alt = usb_audio_alt_set;
    usb_audio_alt_set = alt;
    uac1.cur_alt = alt;
    usb_input_bit_depth = (alt == 2) ? 24 : 16;

    bool active = (alt > 0);
    audio_spdif_set_starvation_monitoring(active);
    audio_ring_last_push_us = 0;

    if (active) {
        if (prev_alt == 0) {
            audio_spdif_reset_dma_starvations();
            stream_restart_resync_pending = true;
            __dmb();
        }
        // Open (or reopen if bit-depth changed) isochronous endpoints.
        uac1_close_stream_eps(rhport);
        if (!uac1_open_stream_eps(rhport, alt)) return false;
    } else {
        uac1_close_stream_eps(rhport);
        if (prev_alt > 0) {
            fb_ctrl_stream_stop(&fb_ctrl);
            feedback_10_14 = nominal_feedback_10_14;
        }
    }
    return true;
}

// ----------------------------------------------------------------------------
// Class + standard control requests
// ----------------------------------------------------------------------------

// UAC1 feature unit (entity 2) — mute + master volume.
static bool uac1_handle_fu_get(uint8_t rhport, tusb_control_request_t const *req) {
    uint8_t cs = TU_U16_HIGH(req->wValue);
    switch (req->bRequest) {
        case UAC1_REQ_GET_CUR:
            if (cs == UAC1_FU_CTRL_MUTE) {
                static uint8_t m;
                m = audio_state.mute ? 1 : 0;
                return tud_control_xfer(rhport, req, &m, 1);
            }
            if (cs == UAC1_FU_CTRL_VOLUME) {
                static int16_t v;
                v = audio_state.volume;
                return tud_control_xfer(rhport, req, &v, 2);
            }
            break;
        case UAC1_REQ_GET_MIN:
            if (cs == UAC1_FU_CTRL_VOLUME) {
                static int16_t v = MIN_VOLUME;
                return tud_control_xfer(rhport, req, &v, 2);
            }
            break;
        case UAC1_REQ_GET_MAX:
            if (cs == UAC1_FU_CTRL_VOLUME) {
                static int16_t v = MAX_VOLUME;
                return tud_control_xfer(rhport, req, &v, 2);
            }
            break;
        case UAC1_REQ_GET_RES:
            if (cs == UAC1_FU_CTRL_VOLUME) {
                static int16_t v = VOLUME_RESOLUTION;
                return tud_control_xfer(rhport, req, &v, 2);
            }
            break;
    }
    return false;
}

static bool uac1_handle_ep_get(uint8_t rhport, tusb_control_request_t const *req) {
    uint8_t cs = TU_U16_HIGH(req->wValue);
    if (req->bRequest == UAC1_REQ_GET_CUR && cs == UAC1_EP_CTRL_SAMPLING_FREQ) {
        static uint8_t freq_bytes[3];
        uint32_t f = audio_state.freq;
        freq_bytes[0] = (uint8_t)(f & 0xFF);
        freq_bytes[1] = (uint8_t)((f >> 8) & 0xFF);
        freq_bytes[2] = (uint8_t)((f >> 16) & 0xFF);
        return tud_control_xfer(rhport, req, freq_bytes, 3);
    }
    return false;
}

static bool uac1_driver_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *req) {
    // Note: vendor-type requests never reach this callback.  TinyUSB routes
    // them directly to tud_vendor_control_xfer_cb (usbd.c:727-730) without
    // consulting class drivers.  See vendor_commands.c for the vendor dispatch.

    if (stage == CONTROL_STAGE_SETUP) {
        // --- Standard requests on our interfaces ---
        if (req->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD) {
            if (req->bRequest == TUSB_REQ_SET_INTERFACE) {
                uint8_t itf = TU_U16_LOW(req->wIndex);
                uint8_t alt = TU_U16_LOW(req->wValue);
                if (itf == uac1.ac_itf) {
                    // AC interface only has alt 0
                    if (alt != 0) return false;
                    return tud_control_status(rhport, req);
                }
                if (itf == uac1.as_itf) {
                    if (!uac1_apply_alt(rhport, alt)) return false;
                    return tud_control_status(rhport, req);
                }
                if (itf == uac1.vendor_itf) {
                    // Vendor interface has only alt 0
                    if (alt != 0) return false;
                    return tud_control_status(rhport, req);
                }
                return false;
            }
            if (req->bRequest == TUSB_REQ_GET_INTERFACE) {
                uint8_t itf = TU_U16_LOW(req->wIndex);
                static uint8_t alt_resp;
                if (itf == uac1.ac_itf) {
                    alt_resp = 0;
                } else if (itf == uac1.as_itf) {
                    alt_resp = uac1.cur_alt;
                } else if (itf == uac1.vendor_itf) {
                    alt_resp = 0;
                } else {
                    return false;
                }
                return tud_control_xfer(rhport, req, &alt_resp, 1);
            }
            return false;
        }

        // --- Class requests ---
        if (req->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS) {
            uint8_t recipient = req->bmRequestType_bit.recipient;
            bool is_get = (req->bmRequestType_bit.direction == TUSB_DIR_IN);

            if (recipient == TUSB_REQ_RCPT_INTERFACE) {
                uint8_t itf      = TU_U16_LOW(req->wIndex);
                uint8_t entityID = TU_U16_HIGH(req->wIndex);
                if (itf != uac1.ac_itf) return false;
                if (entityID != UAC1_FEATURE_UNIT_ID) return false;

                if (is_get) return uac1_handle_fu_get(rhport, req);

                // SET_CUR: schedule data stage.
                if (req->bRequest == UAC1_REQ_SET_CUR) {
                    uint16_t len = req->wLength;
                    if (len == 0 || len > sizeof(uac1_ctrl_buf)) return false;
                    uac1.pending_cs        = TU_U16_HIGH(req->wValue);
                    uac1.pending_recipient = TUSB_REQ_RCPT_INTERFACE;
                    uac1.pending_len       = (uint8_t)len;
                    return tud_control_xfer(rhport, req, uac1_ctrl_buf, len);
                }
                return false;
            }

            if (recipient == TUSB_REQ_RCPT_ENDPOINT) {
                uint8_t ep = TU_U16_LOW(req->wIndex);
                if (ep != AUDIO_OUT_ENDPOINT) return false;

                if (is_get) return uac1_handle_ep_get(rhport, req);

                if (req->bRequest == UAC1_REQ_SET_CUR) {
                    uint16_t len = req->wLength;
                    if (len == 0 || len > sizeof(uac1_ctrl_buf)) return false;
                    uac1.pending_cs        = TU_U16_HIGH(req->wValue);
                    uac1.pending_recipient = TUSB_REQ_RCPT_ENDPOINT;
                    uac1.pending_len       = (uint8_t)len;
                    return tud_control_xfer(rhport, req, uac1_ctrl_buf, len);
                }
                return false;
            }
            return false;
        }

        return false;
    }

    if (stage == CONTROL_STAGE_DATA) {
        // Apply SET_CUR payload captured at SETUP.
        if (req->bmRequestType_bit.type != TUSB_REQ_TYPE_CLASS) return true;
        if (uac1.pending_recipient == TUSB_REQ_RCPT_INTERFACE) {
            if (uac1.pending_cs == UAC1_FU_CTRL_MUTE) {
                audio_state.mute = uac1_ctrl_buf[0];
            } else if (uac1.pending_cs == UAC1_FU_CTRL_VOLUME) {
                int16_t v;
                memcpy(&v, uac1_ctrl_buf, sizeof(v));
                audio_set_volume(v);
            }
        } else if (uac1.pending_recipient == TUSB_REQ_RCPT_ENDPOINT) {
            if (uac1.pending_cs == UAC1_EP_CTRL_SAMPLING_FREQ) {
                uint32_t new_freq = (uint32_t)uac1_ctrl_buf[0]
                                  | ((uint32_t)uac1_ctrl_buf[1] << 8)
                                  | ((uint32_t)uac1_ctrl_buf[2] << 16);
                if (new_freq != audio_state.freq) {
                    audio_state.freq = new_freq;
                    rate_change_pending = true;
                    pending_rate = new_freq;
                }
            }
        }
        uac1.pending_recipient = 0;
        return true;
    }

    return true;
}

// ----------------------------------------------------------------------------
// Endpoint transfer completion — audio RX producer + feedback re-arm.
// Runs in USB IRQ context (rp2040/rp2350 DCD fires xfer_cb synchronously).
// ----------------------------------------------------------------------------

static bool __not_in_flash_func(uac1_driver_xfer_cb)(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes) {
    if (ep_addr == AUDIO_OUT_ENDPOINT) {
        if (result == XFER_RESULT_SUCCESS && xferred_bytes > 0) {
            usb_audio_packets++;

            // Gap detection at actual packet arrival time (preserves the same
            // fault-counting semantics as the old _as_audio_packet callback).
            uint32_t now = time_us_32();
            if (audio_ring_last_push_us > 0 && !preset_loading) {
                uint32_t gap = now - audio_ring_last_push_us;
                if (gap > 2000 && gap < 50000) {
                    spdif_underruns++;
                }
            }
            audio_ring_last_push_us = now;

            usb_audio_ring_push(&audio_ring, ep_out_buf,
                                xferred_bytes > 0xFFFFu ? 0xFFFFu : (uint16_t)xferred_bytes);
        }
        // Re-arm regardless of this frame's success — iso transfers fail-open.
        if (uac1.ep_data_open) uac1_arm_data_out(rhport);
        return true;
    }

    if (ep_addr == AUDIO_IN_ENDPOINT) {
        // Feedback packet transmitted; publish the current value and re-arm.
        if (uac1.ep_fb_open) uac1_arm_feedback(rhport);
        return true;
    }

    return false;
}

// ----------------------------------------------------------------------------
// SOF tick — measures device clock vs host clock, drives the Q16.16 feedback
// servo.  Replaces the old usb_sof_irq() that lived in main.c.
// ----------------------------------------------------------------------------

static void __not_in_flash_func(uac1_driver_sof)(uint8_t rhport, uint32_t frame_count) {
    (void)rhport;
    (void)frame_count;

    extern audio_spdif_instance_t *spdif_instance_ptrs[];
    extern volatile bool output_type_switch_in_progress;

    // Skip during output-type reconfiguration (slot ownership is transiently
    // inconsistent; reading DMA state could crash).
    if (output_type_switch_in_progress) return;

    volatile uint32_t *p_words_consumed;
    uint32_t xfer_words;
    uint8_t dma_ch;
    uint8_t slot0_type = output_types[0];
    uint32_t rate_shift;

    if (slot0_type == OUTPUT_TYPE_I2S) {
        audio_i2s_instance_t *inst = i2s_instance_ptrs[0];
        p_words_consumed = &inst->words_consumed;
        xfer_words = inst->current_transfer_words;
        dma_ch = inst->dma_channel;
        rate_shift = 13;
    } else {
        audio_spdif_instance_t *inst = spdif_instance_ptrs[0];
        p_words_consumed = &inst->words_consumed;
        xfer_words = inst->current_transfer_words;
        dma_ch = inst->dma_channel;
        rate_shift = 12;
    }

    uint32_t remaining = dma_channel_hw_addr(dma_ch)->transfer_count;
    uint32_t current_total = *p_words_consumed + (xfer_words - remaining);

    fb_ctrl_sof_update(&fb_ctrl, current_total, rate_shift, spdif0_consumer_fill);

    uint32_t fb_10_14 = fb_ctrl_get_10_14(&fb_ctrl);
    if (fb_10_14) feedback_10_14 = fb_10_14;
}

// Runtime pin configuration
#if PICO_RP2350
uint8_t output_pins[NUM_PIN_OUTPUTS] = {
    PICO_AUDIO_SPDIF_PIN, PICO_SPDIF_PIN_2,
    PICO_SPDIF_PIN_3, PICO_SPDIF_PIN_4, PICO_PDM_PIN
};
#else
uint8_t output_pins[NUM_PIN_OUTPUTS] = {
    PICO_AUDIO_SPDIF_PIN, PICO_SPDIF_PIN_2, PICO_PDM_PIN
};
#endif

audio_spdif_instance_t *spdif_instance_ptrs[NUM_SPDIF_INSTANCES];

// ---------------------------------------------------------------------------
// OutputSlot — per-slot output type management (S/PDIF or I2S)
// ---------------------------------------------------------------------------

// Per-slot output type: OUTPUT_TYPE_SPDIF (0) or OUTPUT_TYPE_I2S (1)
uint8_t output_types[NUM_SPDIF_INSTANCES] = {0};  // All S/PDIF by default

// I2S instances — statically allocated, activated when a slot switches to I2S
static audio_i2s_instance_t i2s_instance_1 = {0};
static audio_i2s_instance_t i2s_instance_2 = {0};
#if PICO_RP2350
static audio_i2s_instance_t i2s_instance_3 = {0};
static audio_i2s_instance_t i2s_instance_4 = {0};
#endif

// Indexed arrays for both instance types (populated in usb_sound_card_init)
audio_i2s_instance_t *i2s_instance_ptrs[NUM_SPDIF_INSTANCES];
struct audio_buffer_pool *producer_pools[NUM_SPDIF_INSTANCES];

// I2S clock configuration
uint8_t i2s_bck_pin = PICO_I2S_BCK_PIN;     // BCK GPIO; LRCLK = BCK + 1
uint8_t i2s_mck_pin = PICO_I2S_MCK_PIN;     // MCK GPIO
bool    i2s_mck_enabled = false;             // MCK enabled state
// MCK multiplier: actual value (128 or 256).
// Wire/flash format uses uint8_t where 256 wraps to 0 — encode/decode at boundaries only.
uint16_t i2s_mck_multiplier = 128;


// ----------------------------------------------------------------------------
// INIT
// ----------------------------------------------------------------------------

// S/PDIF Instances
static audio_spdif_instance_t spdif_instance_1 = {0};  // Out 1-2
static audio_spdif_instance_t spdif_instance_2 = {0};  // Out 3-4
#if PICO_RP2350
static audio_spdif_instance_t spdif_instance_3 = {0};  // Out 5-6
static audio_spdif_instance_t spdif_instance_4 = {0};  // Out 7-8
#endif

struct audio_spdif_config spdif_config_1 = {
    .pin = PICO_AUDIO_SPDIF_PIN,  // GPIO 6
    .dma_channel = 0,
    .pio_sm = 0,
    .pio = PICO_AUDIO_SPDIF_PIO,
    .dma_irq = PICO_AUDIO_SPDIF_DMA_IRQ,
};

struct audio_spdif_config spdif_config_2 = {
    .pin = PICO_SPDIF_PIN_2,  // GPIO 7
    .dma_channel = 1,
    .pio_sm = 1,
    .pio = PICO_AUDIO_SPDIF_PIO,
    .dma_irq = PICO_AUDIO_SPDIF_DMA_IRQ,
};

#if PICO_RP2350
struct audio_spdif_config spdif_config_3 = {
    .pin = PICO_SPDIF_PIN_3,  // GPIO 8
    .dma_channel = 2,
    .pio_sm = 2,
    .pio = PICO_AUDIO_SPDIF_PIO,
    .dma_irq = PICO_AUDIO_SPDIF_DMA_IRQ,
};

struct audio_spdif_config spdif_config_4 = {
    .pin = PICO_SPDIF_PIN_4,  // GPIO 9
    .dma_channel = 3,
    .pio_sm = 3,
    .pio = PICO_AUDIO_SPDIF_PIO,
    .dma_irq = PICO_AUDIO_SPDIF_DMA_IRQ,
};
#endif

struct audio_buffer_format producer_format = { .format = &audio_format_48k, .sample_stride = 8 };

// Legacy aliases
#define spdif_instance spdif_instance_1
#define spdif_sub_instance spdif_instance_2
#define config spdif_config_1
#define sub_config spdif_config_2

// Initialize matrix mixer with default stereo pass-through
static void matrix_init_defaults(void) {
    memset(&matrix_mixer, 0, sizeof(matrix_mixer));

    // Stereo pass-through on first S/PDIF pair (Out 1-2)
    matrix_mixer.crosspoints[0][0].enabled = 1;     // L→Out1
    matrix_mixer.crosspoints[0][0].gain_db = 0.0f;
    matrix_mixer.crosspoints[0][0].gain_linear = 1.0f;

    matrix_mixer.crosspoints[1][1].enabled = 1;     // R→Out2
    matrix_mixer.crosspoints[1][1].gain_db = 0.0f;
    matrix_mixer.crosspoints[1][1].gain_linear = 1.0f;

    // Enable first stereo pair only by default
    matrix_mixer.outputs[0].enabled = 1;
    matrix_mixer.outputs[0].gain_linear = 1.0f;
    matrix_mixer.outputs[1].enabled = 1;
    matrix_mixer.outputs[1].gain_linear = 1.0f;

    // All other outputs disabled by default (saves CPU)
    for (int out = 2; out < NUM_OUTPUT_CHANNELS; out++) {
        matrix_mixer.outputs[out].enabled = 0;
        matrix_mixer.outputs[out].gain_linear = 1.0f;
    }
}

void usb_sound_card_init(void) {
    // Initialize matrix mixer defaults
    matrix_init_defaults();
    reset_buffer_watermarks();

    // S/PDIF Setup (this must happen before USB init to claim DMA channels)
    producer_pool_1 = audio_new_producer_pool(&producer_format, AUDIO_BUFFER_COUNT, 192);
    producer_pool_2 = audio_new_producer_pool(&producer_format, AUDIO_BUFFER_COUNT, 192);
#if PICO_RP2350
    producer_pool_3 = audio_new_producer_pool(&producer_format, AUDIO_BUFFER_COUNT, 192);
    producer_pool_4 = audio_new_producer_pool(&producer_format, AUDIO_BUFFER_COUNT, 192);
#endif

    // Setup S/PDIF instances
    audio_spdif_setup(&spdif_instance_1, &audio_format_48k, &spdif_config_1);
    audio_spdif_connect_extra(&spdif_instance_1, producer_pool_1, false, SPDIF_CONSUMER_BUFFER_COUNT, NULL);

    audio_spdif_setup(&spdif_instance_2, &audio_format_48k, &spdif_config_2);
    audio_spdif_connect_extra(&spdif_instance_2, producer_pool_2, false, SPDIF_CONSUMER_BUFFER_COUNT, NULL);

#if PICO_RP2350
    audio_spdif_setup(&spdif_instance_3, &audio_format_48k, &spdif_config_3);
    audio_spdif_connect_extra(&spdif_instance_3, producer_pool_3, false, SPDIF_CONSUMER_BUFFER_COUNT, NULL);

    audio_spdif_setup(&spdif_instance_4, &audio_format_48k, &spdif_config_4);
    audio_spdif_connect_extra(&spdif_instance_4, producer_pool_4, false, SPDIF_CONSUMER_BUFFER_COUNT, NULL);
#endif

    // Populate instance pointer arrays for pin/type config commands
    spdif_instance_ptrs[0] = &spdif_instance_1;
    spdif_instance_ptrs[1] = &spdif_instance_2;
#if PICO_RP2350
    spdif_instance_ptrs[2] = &spdif_instance_3;
    spdif_instance_ptrs[3] = &spdif_instance_4;
#endif

    // I2S instance pointers (instances are dormant until a slot is switched to I2S)
    i2s_instance_ptrs[0] = &i2s_instance_1;
    i2s_instance_ptrs[1] = &i2s_instance_2;
#if PICO_RP2350
    i2s_instance_ptrs[2] = &i2s_instance_3;
    i2s_instance_ptrs[3] = &i2s_instance_4;
#endif

    // Indexed producer pool array for type-switching convenience
    producer_pools[0] = producer_pool_1;
    producer_pools[1] = producer_pool_2;
#if PICO_RP2350
    producer_pools[2] = producer_pool_3;
    producer_pools[3] = producer_pool_4;
#endif

    // MCK generator setup on PIO1 SM1 (starts disabled)
    audio_i2s_mck_setup(pio1, 1, i2s_mck_pin);

    irq_set_priority(DMA_IRQ_0 + PICO_AUDIO_SPDIF_DMA_IRQ, PICO_HIGHEST_IRQ_PRIORITY);
    irq_set_priority(DMA_IRQ_0 + PICO_AUDIO_I2S_DMA_IRQ, PICO_HIGHEST_IRQ_PRIORITY);

    // Start all outputs synchronized
#if PICO_RP2350
    audio_spdif_instance_t *spdif_all[] = {
        &spdif_instance_1, &spdif_instance_2, &spdif_instance_3, &spdif_instance_4
    };
    audio_spdif_enable_sync(spdif_all, 4);
#else
    audio_spdif_instance_t *spdif_all[] = {
        &spdif_instance_1, &spdif_instance_2
    };
    audio_spdif_enable_sync(spdif_all, 2);
#endif

    // Initialize TinyUSB device stack.  The UAC1 class driver registered via
    // usbd_app_driver_get_cb() will own the AC + AS interfaces.
    tud_init(0);

    // Initialize DSP
    dsp_init_default_filters();
    dsp_recalculate_all_filters(48000.0f);
    audio_set_volume(DEFAULT_VOLUME);
    rate_change_pending = true;
    pending_rate = audio_state.freq;

    // Initialize Core 1 EQ worker pointer to shared output buffer
    core1_eq_work.buf_out = buf_out;

    // Initialize ADC for temperature sensor
    adc_init();
    adc_set_temp_sensor_enabled(true);
}
