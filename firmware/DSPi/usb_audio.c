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

#include "usb_audio.h"
#include "vendor_commands.h"
#include "usb_descriptors.h"
#include "dsp_pipeline.h"
#include "dcp_inline.h"
#include "pdm_generator.h"
#include "flash_storage.h"
#include "loudness.h"
#include "crossfeed.h"
#include "leveller.h"
#include "bulk_params.h"
#include "pico/usb_stream_helper.h"
#include "usb_audio_ring.h"
#include "usb_feedback_controller.h"

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

#if PICO_RP2350
static LoudnessSvfState loudness_state[2][LOUDNESS_BIQUAD_COUNT];  // [0]=Left, [1]=Right
#else
static Biquad loudness_biquads[2][LOUDNESS_BIQUAD_COUNT];  // [0]=Left, [1]=Right
#endif
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
CrossfeedState crossfeed_state;

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
LevellerCoeffs leveller_coeffs;
LevellerState leveller_state;

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

// Shared output buffer — file scope so Core 1 can access via pointer
#if PICO_RP2350
static float buf_out[NUM_OUTPUT_CHANNELS][192];
#else
static int32_t buf_out[NUM_OUTPUT_CHANNELS][192];
#endif

// Sync State
volatile uint64_t total_samples_produced = 0;
volatile uint64_t start_time_us = 0;
volatile bool sync_started = false;
static volatile uint64_t last_packet_time_us = 0;
static volatile uint8_t usb_input_bit_depth = 16;
#define AUDIO_GAP_THRESHOLD_US 50000  // 50ms - reset sync if packets stop this long

// Idle-time CPU load metering (Core 0)
static uint32_t cpu0_last_packet_end = 0;
static uint32_t cpu0_load_q8 = 0;         // EMA in Q8 fixed point (0-25600 = 0-100%)
static bool cpu0_load_primed = false;

// Consumer fill for instance 0 — used by watermark monitoring only
// (no longer part of the active feedback path).
volatile uint8_t spdif0_consumer_fill = 0;

// Buffer statistics watermark tracking
static void update_buffer_watermarks(void);
static inline void update_slot0_fill_fast(void);
uint16_t buffer_stats_sequence = 0;
uint8_t spdif_consumer_min_fill_pct[NUM_SPDIF_INSTANCES];
uint8_t spdif_consumer_max_fill_pct[NUM_SPDIF_INSTANCES];
uint8_t pdm_dma_min_fill_pct = 100;
uint8_t pdm_dma_max_fill_pct = 0;
uint8_t pdm_ring_min_fill_pct = 100;
uint8_t pdm_ring_max_fill_pct = 0;

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
// USB INTERFACE / ENDPOINT OBJECTS
// ----------------------------------------------------------------------------

static struct usb_interface ac_interface;
static struct usb_interface as_op_interface;
static struct usb_interface vendor_interface;
static struct usb_endpoint ep_op_out, ep_op_sync;

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
// AUDIO PROCESSING (called from USB audio packet callback)
// ----------------------------------------------------------------------------

// Preset mute smoothing
//
// Flash-backed operations (save/load/delete, directory writes) drive
// `preset_loading` to force a temporary mute. A hard step between full-scale
// and zero can produce an audible pop on some DAC chains, so we apply a short
// envelope around the mute gate.
//
// The envelope runs in packet context (process_audio_packet) and advances by
// `sample_count` each call, giving a time-based transition that is consistent
// across 44.1/48/96 kHz.
#define PRESET_MUTE_TRANSITION_MS 8u
static float preset_mute_smooth_gain = 1.0f;  // 1.0 = full level, 0.0 = muted

static inline uint32_t preset_mute_transition_samples(uint32_t sample_rate_hz) {
    uint64_t samples = ((uint64_t)sample_rate_hz * PRESET_MUTE_TRANSITION_MS + 999u) / 1000u;
    if (samples < 1u) samples = 1u;
    if (samples > UINT32_MAX) samples = UINT32_MAX;
    return (uint32_t)samples;
}

static inline float update_preset_mute_envelope(uint32_t sample_count, uint32_t sample_rate_hz) {
    // Latch current mute state for THIS packet so the final muted packet
    // remains fully in the fade-out direction even when the counter expires.
    bool mute_active_for_packet = preset_loading;

    if (mute_active_for_packet) {
        if (preset_mute_counter > sample_count) {
            preset_mute_counter -= sample_count;
        } else {
            preset_mute_counter = 0;
            preset_loading = false;
        }
    }

    float target = mute_active_for_packet ? 0.0f : 1.0f;
    if (sample_count == 0) {
        preset_mute_smooth_gain = target;
        return preset_mute_smooth_gain;
    }

    float step = (float)sample_count / (float)preset_mute_transition_samples(sample_rate_hz);
    if (step > 1.0f) step = 1.0f;

    if (preset_mute_smooth_gain < target) {
        preset_mute_smooth_gain += step;
        if (preset_mute_smooth_gain > target) preset_mute_smooth_gain = target;
    } else if (preset_mute_smooth_gain > target) {
        preset_mute_smooth_gain -= step;
        if (preset_mute_smooth_gain < target) preset_mute_smooth_gain = target;
    }

    return preset_mute_smooth_gain;
}

static void __not_in_flash_func(process_audio_packet)(const uint8_t *data, uint16_t data_len) {
    uint32_t packet_start = time_us_32();

    // NOTE: USB packet gap detection has moved to _as_audio_packet() (ISR
    // context) where it measures actual packet arrival timing rather than
    // main-loop processing timing.  See audio_ring_last_push_us.

    // Get audio buffers for S/PDIF outputs
#if PICO_RP2350
    struct audio_buffer* audio_buf[4] = {NULL, NULL, NULL, NULL};
    if (producer_pool_1) audio_buf[0] = take_audio_buffer(producer_pool_1, false);
    if (producer_pool_2) audio_buf[1] = take_audio_buffer(producer_pool_2, false);
    if (producer_pool_3) audio_buf[2] = take_audio_buffer(producer_pool_3, false);
    if (producer_pool_4) audio_buf[3] = take_audio_buffer(producer_pool_4, false);
#else
    struct audio_buffer* audio_buf[2] = {NULL, NULL};
    if (producer_pool_1) audio_buf[0] = take_audio_buffer(producer_pool_1, false);
    if (producer_pool_2) audio_buf[1] = take_audio_buffer(producer_pool_2, false);
#endif

    update_slot0_fill_fast();
    // Watermark tracking is diagnostic-only; run at lower cadence to keep
    // the packet callback lean under heavy DSP/output load.
    static uint8_t watermark_div = 0;
    if ((++watermark_div & 0x07u) == 0) {
        update_buffer_watermarks();
    }

    const uint8_t bit_depth = usb_input_bit_depth;  // snapshot once — avoid double-read of volatile
    uint32_t bytes_per_frame = (bit_depth == 24) ? 6 : 4;
    uint32_t sample_count = data_len / bytes_per_frame;
    uint32_t sample_rate_hz = audio_state.freq;
    float preset_mute_gain = update_preset_mute_envelope(sample_count, sample_rate_hz);

    for (int b = 0; b < NUM_SPDIF_INSTANCES; b++) {
        if (audio_buf[b]) {
            audio_buf[b]->sample_count = sample_count;
        } else if (!preset_loading && (matrix_mixer.outputs[b*2].enabled || matrix_mixer.outputs[b*2+1].enabled)) {
            spdif_overruns++;
        }
    }

    uint64_t now_us = time_us_64();

    // Detect audio restart after gap - reset sync state and pre-fill pool
    if (sync_started && last_packet_time_us > 0 &&
        (now_us - last_packet_time_us) > AUDIO_GAP_THRESHOLD_US) {
        sync_started = false;
        total_samples_produced = 0;
        cpu0_load_primed = false;
        cpu0_load_q8 = 0;
    }
    last_packet_time_us = now_us;

    if (!sync_started) {
        start_time_us = now_us;
        sync_started = true;
    }
    total_samples_produced += sample_count;

#if PICO_RP2350
    // ------------------------------------------------------------------------
    // RP2350 FLOAT PIPELINE WITH MATRIX MIXER
    // ------------------------------------------------------------------------
    const float inv_32768 = 1.0f / 32768.0f;

    // vol_mul: USB host volume (raw) — used for loudness compensation only.
    // vol_mul_master: host volume × master volume — used for output gain.
    // Keeping them separate ensures master volume never affects loudness curves.
    float vol_mul = audio_state.mute ? 0.0f : (float)audio_state.vol_mul * inv_32768;
    vol_mul *= preset_mute_gain;
    float vol_mul_master = vol_mul * master_volume_linear;

    // Per-input-channel preamp (snapshot for this block)
    float preamp_l = global_preamp_linear[0];
    float preamp_r = global_preamp_linear[1];
    bool is_bypassed = bypass_master_eq;

    // Snapshot loudness state for this packet
    bool loud_on = loudness_enabled;
    const LoudnessCoeffs *loud_coeffs = current_loudness_coeffs;

    float peak_ml = 0, peak_mr = 0;

    // Pre-compute PDM scale factor
    const float pdm_scale = (float)(1 << 28);

    // Static buffers to avoid stack overflow (~8KB would be too much for stack)
    static float buf_l[192], buf_r[192];

    // ========== PASS 1: Input conversion + Preamp + Loudness ==========
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

    // Loudness compensation (SVF shelf filters)
    if (loud_on && loud_coeffs) {
        for (uint32_t i = 0; i < sample_count; i++) {
            float raw_left = buf_l[i];
            float raw_right = buf_r[i];
            for (int j = 0; j < LOUDNESS_BIQUAD_COUNT; j++) {
                const LoudnessCoeffs *lc = &loud_coeffs[j];
                if (lc->bypass) continue;
                LoudnessSvfState *st = &loudness_state[0][j];
                float v3 = raw_left - st->ic2eq;
                float v1 = lc->sva1 * st->ic1eq + lc->sva2 * v3;
                float v2 = st->ic2eq + lc->sva2 * st->ic1eq + lc->sva3 * v3;
                st->ic1eq = 2.0f * v1 - st->ic1eq;
                st->ic2eq = 2.0f * v2 - st->ic2eq;
                raw_left = lc->svm0 * raw_left + lc->svm1 * v1 + lc->svm2 * v2;
            }
            for (int j = 0; j < LOUDNESS_BIQUAD_COUNT; j++) {
                const LoudnessCoeffs *lc = &loud_coeffs[j];
                if (lc->bypass) continue;
                LoudnessSvfState *st = &loudness_state[1][j];
                float v3 = raw_right - st->ic2eq;
                float v1 = lc->sva1 * st->ic1eq + lc->sva2 * v3;
                float v2 = st->ic2eq + lc->sva2 * st->ic1eq + lc->sva3 * v3;
                st->ic1eq = 2.0f * v1 - st->ic1eq;
                st->ic2eq = 2.0f * v2 - st->ic2eq;
                raw_right = lc->svm0 * raw_right + lc->svm1 * v1 + lc->svm2 * v2;
            }
            buf_l[i] = raw_left;
            buf_r[i] = raw_right;
        }
    }

    // ========== PASS 2: Master EQ (Block-Based) ==========
    if (!is_bypassed) {
        if (!channel_bypassed[CH_MASTER_LEFT]) {
            dsp_process_channel_block(filters[CH_MASTER_LEFT], buf_l, sample_count, CH_MASTER_LEFT);
        }
        if (!channel_bypassed[CH_MASTER_RIGHT]) {
            dsp_process_channel_block(filters[CH_MASTER_RIGHT], buf_r, sample_count, CH_MASTER_RIGHT);
        }
    }

    // ========== PASS 2.5: Volume Leveller ==========
    if (!leveller_bypassed) {
        leveller_process_block(&leveller_state, &leveller_coeffs,
                               (const LevellerConfig *)&leveller_config,
                               buf_l, buf_r, sample_count);
    }

    // ========== PASS 3: Crossfeed + Master Peaks ==========
    bool do_crossfeed = !crossfeed_bypassed;

    // Crossfeed is sample-by-sample (internal state), combined with peak tracking
    for (uint32_t i = 0; i < sample_count; i++) {
        float ml = buf_l[i], mr = buf_r[i];
        float abs_ml = fabsf(ml); if (abs_ml > peak_ml) peak_ml = abs_ml;
        float abs_mr = fabsf(mr); if (abs_mr > peak_mr) peak_mr = abs_mr;
        if (do_crossfeed) {
            crossfeed_process_stereo(&crossfeed_state, &ml, &mr);
            buf_l[i] = ml; buf_r[i] = mr;
        }
    }

    // ========== PASS 4: Matrix Mixing (block-based, output-major) ==========
    // Snapshot crosspoint coefficients and process one output at a time
    for (int out = 0; out < NUM_OUTPUT_CHANNELS; out++) {
        if (!matrix_mixer.outputs[out].enabled) {
            memset(buf_out[out], 0, sample_count * sizeof(float));
            continue;
        }

        // Load crosspoint config once per output (not per sample)
        float gain_l = 0.0f, gain_r = 0.0f;
        MatrixCrosspoint *xp_l = &matrix_mixer.crosspoints[0][out];
        MatrixCrosspoint *xp_r = &matrix_mixer.crosspoints[1][out];
        if (xp_l->enabled) gain_l = xp_l->phase_invert ? -xp_l->gain_linear : xp_l->gain_linear;
        if (xp_r->enabled) gain_r = xp_r->phase_invert ? -xp_r->gain_linear : xp_r->gain_linear;

        float *dst = buf_out[out];
        if (gain_l != 0.0f && gain_r != 0.0f) {
            for (uint32_t i = 0; i < sample_count; i++)
                dst[i] = buf_l[i] * gain_l + buf_r[i] * gain_r;
        } else if (gain_l != 0.0f) {
            for (uint32_t i = 0; i < sample_count; i++)
                dst[i] = buf_l[i] * gain_l;
        } else if (gain_r != 0.0f) {
            for (uint32_t i = 0; i < sample_count; i++)
                dst[i] = buf_r[i] * gain_r;
        } else {
            memset(dst, 0, sample_count * sizeof(float));
        }
    }

    // ========== PASS 5-7: Per-Output EQ + Gain + Delay + Output ==========
    if (core1_mode == CORE1_MODE_EQ_WORKER) {
        // --- Dual-core path: Core 1 handles EQ+delay+SPDIF for outputs 2-7 ---

        // Dispatch to Core 1
        core1_eq_work.sample_count = sample_count;
        core1_eq_work.vol_mul = vol_mul_master;  // Core 1 uses master-scaled volume
        core1_eq_work.delay_write_idx = delay_write_idx;
        core1_eq_work.spdif_out[0] = audio_buf[1] ? (int32_t *)audio_buf[1]->buffer->bytes : NULL;
        core1_eq_work.spdif_out[1] = audio_buf[2] ? (int32_t *)audio_buf[2]->buffer->bytes : NULL;
        core1_eq_work.spdif_out[2] = audio_buf[3] ? (int32_t *)audio_buf[3]->buffer->bytes : NULL;
        core1_eq_work.work_done = false;
        __dmb();
        core1_eq_work.work_ready = true;
        __sev();

        // Core 0: EQ + gain for outputs 0-1
        for (int out = 0; out < CORE1_EQ_FIRST_OUTPUT; out++) {
            if (!matrix_mixer.outputs[out].enabled) continue;
            if (!matrix_mixer.outputs[out].mute) {
                uint8_t eq_ch = CH_OUT_1 + out;
                if (!channel_bypassed[eq_ch]) {
                    dsp_process_channel_block(filters[eq_ch], buf_out[out], sample_count, eq_ch);
                }
            }
            // Output gain uses vol_mul_master (host vol × master vol)
            float gain = matrix_mixer.outputs[out].mute ? 0.0f
                         : matrix_mixer.outputs[out].gain_linear * vol_mul_master;
            if (gain == 0.0f) {
                memset(buf_out[out], 0, sample_count * sizeof(float));
            } else if (gain != 1.0f) {
                float *dst = buf_out[out];
                for (uint32_t i = 0; i < sample_count; i++)
                    dst[i] *= gain;
            }
        }

        // Core 0: Delay for outputs 0-1
        if (any_delay_active) {
            for (int out = 0; out < CORE1_EQ_FIRST_OUTPUT; out++) {
                int32_t dly = channel_delay_samples[out];
                if (dly <= 0) continue;
                float *dst = buf_out[out];
                float *dline = delay_lines[out];
                uint32_t widx = delay_write_idx;
                for (uint32_t i = 0; i < sample_count; i++) {
                    dline[widx] = dst[i];
                    dst[i] = dline[(widx - dly) & MAX_DELAY_MASK];
                    widx = (widx + 1) & MAX_DELAY_MASK;
                }
            }
        }

        // Core 0: Peaks for outputs 0..CORE1_EQ_FIRST_OUTPUT-1
        for (int out = 0; out < CORE1_EQ_FIRST_OUTPUT; out++) {
            float peak = 0;
            for (uint32_t i = 0; i < sample_count; i++) {
                float a = fabsf(buf_out[out][i]);
                if (a > peak) peak = a;
            }
            global_status.peaks[CH_OUT_1 + out] = (uint16_t)(fminf(1.0f, peak) * 32767.0f);
            if (peak > CLIP_THRESH_F) global_status.clip_flags |= (1u << (CH_OUT_1 + out));
        }
        // PDM is inactive in EQ_WORKER mode
        global_status.peaks[CH_OUT_SUB] = 0;

        // Core 0: S/PDIF for pair 0
        if (audio_buf[0]) {
            int left_ch = 0, right_ch = 1;
            if (!matrix_mixer.outputs[left_ch].enabled && !matrix_mixer.outputs[right_ch].enabled) {
                memset(audio_buf[0]->buffer->bytes, 0, sample_count * 8);
            } else {
                int32_t *out_ptr = (int32_t *)audio_buf[0]->buffer->bytes;
                for (uint32_t i = 0; i < sample_count; i++) {
                    float dl = fmaxf(-1.0f, fminf(1.0f, buf_out[0][i]));
                    float dr = fmaxf(-1.0f, fminf(1.0f, buf_out[1][i]));
                    out_ptr[i*2]   = (int32_t)(dl * 8388607.0f);
                    out_ptr[i*2+1] = (int32_t)(dr * 8388607.0f);
                }
            }
        }

        // Wait for Core 1 (EQ + delay + S/PDIF for outputs 2-7)
        while (!core1_eq_work.work_done) {
            __wfe();
        }
        __dmb();

        // Update shared delay write index (both cores used same base)
        if (any_delay_active) {
            delay_write_idx = (delay_write_idx + sample_count) & MAX_DELAY_MASK;
        }
    } else {
        // --- Single-core path: all outputs on Core 0 ---

        // EQ + gain
        for (int out = 0; out < NUM_OUTPUT_CHANNELS; out++) {
            if (!matrix_mixer.outputs[out].enabled) continue;
            if (!matrix_mixer.outputs[out].mute) {
                uint8_t eq_ch = CH_OUT_1 + out;
                if (!channel_bypassed[eq_ch]) {
                    dsp_process_channel_block(filters[eq_ch], buf_out[out], sample_count, eq_ch);
                }
            }
            // Output gain uses vol_mul_master (host vol × master vol)
            float gain = matrix_mixer.outputs[out].mute ? 0.0f
                         : matrix_mixer.outputs[out].gain_linear * vol_mul_master;
            if (gain == 0.0f) {
                memset(buf_out[out], 0, sample_count * sizeof(float));
            } else if (gain != 1.0f) {
                float *dst = buf_out[out];
                for (uint32_t i = 0; i < sample_count; i++)
                    dst[i] *= gain;
            }
        }

        // Delay
        if (any_delay_active) {
            for (int out = 0; out < NUM_OUTPUT_CHANNELS; out++) {
                int32_t dly = channel_delay_samples[out];
                if (dly <= 0) continue;
                float *dst = buf_out[out];
                float *dline = delay_lines[out];
                uint32_t widx = delay_write_idx;
                for (uint32_t i = 0; i < sample_count; i++) {
                    dline[widx] = dst[i];
                    dst[i] = dline[(widx - dly) & MAX_DELAY_MASK];
                    widx = (widx + 1) & MAX_DELAY_MASK;
                }
            }
            delay_write_idx = (delay_write_idx + sample_count) & MAX_DELAY_MASK;
        }

        // Peaks for all SPDIF outputs
        for (int out = 0; out < NUM_SPDIF_INSTANCES * 2; out++) {
            float peak = 0;
            for (uint32_t i = 0; i < sample_count; i++) {
                float a = fabsf(buf_out[out][i]);
                if (a > peak) peak = a;
            }
            global_status.peaks[CH_OUT_1 + out] = (uint16_t)(fminf(1.0f, peak) * 32767.0f);
            if (peak > CLIP_THRESH_F) global_status.clip_flags |= (1u << (CH_OUT_1 + out));
        }

        // S/PDIF conversion
        for (int pair = 0; pair < 4; pair++) {
            if (!audio_buf[pair]) continue;
            int left_ch = pair * 2;
            int right_ch = pair * 2 + 1;
            if (!matrix_mixer.outputs[left_ch].enabled && !matrix_mixer.outputs[right_ch].enabled) {
                memset(audio_buf[pair]->buffer->bytes, 0, sample_count * 8);
                continue;
            }
            int32_t *out_ptr = (int32_t *)audio_buf[pair]->buffer->bytes;
            for (uint32_t i = 0; i < sample_count; i++) {
                float dl = fmaxf(-1.0f, fminf(1.0f, buf_out[left_ch][i]));
                float dr = fmaxf(-1.0f, fminf(1.0f, buf_out[right_ch][i]));
                out_ptr[i*2]     = (int32_t)(dl * 8388607.0f);
                out_ptr[i*2+1]   = (int32_t)(dr * 8388607.0f);
            }
        }

#if ENABLE_SUB
        if (matrix_mixer.outputs[NUM_OUTPUT_CHANNELS-1].enabled) {
            float peak_sub = 0;
            for (uint32_t i = 0; i < sample_count; i++) {
                float abs_sub = fabsf(buf_out[NUM_OUTPUT_CHANNELS-1][i]);
                if (abs_sub > peak_sub) peak_sub = abs_sub;
            }
            global_status.peaks[CH_OUT_SUB] = (uint16_t)(fminf(1.0f, peak_sub) * 32767.0f);
            if (peak_sub > CLIP_THRESH_F) global_status.clip_flags |= (1u << CH_OUT_SUB);
            for (uint32_t i = 0; i < sample_count; i++) {
                int32_t pdm_sample_q28 = (int32_t)(buf_out[NUM_OUTPUT_CHANNELS-1][i] * pdm_scale);
                pdm_push_sample(pdm_sample_q28, false);
            }
        } else {
            global_status.peaks[CH_OUT_SUB] = 0;
        }
#endif
    }

    // Write input peaks
    global_status.peaks[0] = (uint16_t)(fminf(1.0f, peak_ml) * 32767.0f);
    global_status.peaks[1] = (uint16_t)(fminf(1.0f, peak_mr) * 32767.0f);
    if (peak_ml > CLIP_THRESH_F) global_status.clip_flags |= (1u << CH_MASTER_LEFT);
    if (peak_mr > CLIP_THRESH_F) global_status.clip_flags |= (1u << CH_MASTER_RIGHT);

#else
    // ------------------------------------------------------------------------
    // RP2040 BLOCK-BASED FIXED-POINT PIPELINE WITH MATRIX MIXER
    // 2 SPDIF stereo pairs + PDM sub, dual-core EQ worker
    // ------------------------------------------------------------------------
    // vol_mul: USB host volume (raw Q15) — used for loudness compensation only.
    // vol_mul_master: host volume × master volume (Q15) — used for output gain.
    int32_t vol_mul = audio_state.mute ? 0 : audio_state.vol_mul;
    int32_t preset_mute_gain_q15 = (int32_t)(preset_mute_gain * 32768.0f + 0.5f);
    if (preset_mute_gain_q15 < 0) preset_mute_gain_q15 = 0;
    if (preset_mute_gain_q15 > 32768) preset_mute_gain_q15 = 32768;
    vol_mul = fast_mul_q15(vol_mul, preset_mute_gain_q15);
    int32_t vol_mul_master = fast_mul_q15(vol_mul, master_volume_q15);

    // Per-input-channel preamp (snapshot for this block)
    int32_t preamp_l = global_preamp_mul[0];
    int32_t preamp_r = global_preamp_mul[1];
    bool is_bypassed = bypass_master_eq;

    // Snapshot loudness state for this packet
    bool loud_on = loudness_enabled;
    const LoudnessCoeffs *loud_coeffs = current_loudness_coeffs;

    int32_t peak_ml = 0, peak_mr = 0;

    // Static buffers for block processing
    static int32_t buf_l[192], buf_r[192];

    // ========== PASS 1: Input conversion + Preamp + Loudness ==========
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

    // Loudness compensation (per-sample — biquad state coupling)
    if (loud_on && loud_coeffs) {
        for (uint32_t i = 0; i < sample_count; i++) {
            int32_t raw_left_32 = buf_l[i];
            int32_t raw_right_32 = buf_r[i];
            for (int j = 0; j < LOUDNESS_BIQUAD_COUNT; j++) {
                const LoudnessCoeffs *lc = &loud_coeffs[j];
                if (lc->bypass) continue;
                Biquad *bq = &loudness_biquads[0][j];
                int32_t result = fast_mul_q28(lc->b0, raw_left_32) + bq->s1;
                bq->s1 = fast_mul_q28(lc->b1, raw_left_32)
                        - fast_mul_q28(lc->a1, result) + bq->s2;
                bq->s2 = fast_mul_q28(lc->b2, raw_left_32)
                        - fast_mul_q28(lc->a2, result);
                raw_left_32 = result;
            }
            for (int j = 0; j < LOUDNESS_BIQUAD_COUNT; j++) {
                const LoudnessCoeffs *lc = &loud_coeffs[j];
                if (lc->bypass) continue;
                Biquad *bq = &loudness_biquads[1][j];
                int32_t result = fast_mul_q28(lc->b0, raw_right_32) + bq->s1;
                bq->s1 = fast_mul_q28(lc->b1, raw_right_32)
                        - fast_mul_q28(lc->a1, result) + bq->s2;
                bq->s2 = fast_mul_q28(lc->b2, raw_right_32)
                        - fast_mul_q28(lc->a2, result);
                raw_right_32 = result;
            }
            buf_l[i] = raw_left_32;
            buf_r[i] = raw_right_32;
        }
    }

    // ========== PASS 2: Master EQ (Block-Based) ==========
    if (!is_bypassed) {
        if (!channel_bypassed[CH_MASTER_LEFT])
            dsp_process_channel_block(filters[CH_MASTER_LEFT], buf_l, sample_count, CH_MASTER_LEFT);
        if (!channel_bypassed[CH_MASTER_RIGHT])
            dsp_process_channel_block(filters[CH_MASTER_RIGHT], buf_r, sample_count, CH_MASTER_RIGHT);
    }

    // ========== PASS 2.5: Volume Leveller ==========
    if (!leveller_bypassed) {
        leveller_process_block(&leveller_state, &leveller_coeffs,
                               (const LevellerConfig *)&leveller_config,
                               buf_l, buf_r, sample_count);
    }

    // ========== PASS 3: Crossfeed + Master Peaks ==========
    for (uint32_t i = 0; i < sample_count; i++) {
        int32_t ml = buf_l[i], mr = buf_r[i];
        if (abs(ml) > peak_ml) peak_ml = abs(ml);
        if (abs(mr) > peak_mr) peak_mr = abs(mr);
        if (!crossfeed_bypassed) {
            crossfeed_process_stereo(&crossfeed_state, &ml, &mr);
            buf_l[i] = ml; buf_r[i] = mr;
        }
    }

    // ========== PASS 4: Matrix Mixing (block-based, output-major) ==========
    for (int out = 0; out < NUM_OUTPUT_CHANNELS; out++) {
        if (!matrix_mixer.outputs[out].enabled) {
            memset(buf_out[out], 0, sample_count * sizeof(int32_t));
            continue;
        }

        MatrixCrosspoint *xp_l = &matrix_mixer.crosspoints[0][out];
        MatrixCrosspoint *xp_r = &matrix_mixer.crosspoints[1][out];
        int32_t gain_l_q15 = xp_l->enabled ? (int32_t)((xp_l->phase_invert ? -xp_l->gain_linear : xp_l->gain_linear) * 32768.0f) : 0;
        int32_t gain_r_q15 = xp_r->enabled ? (int32_t)((xp_r->phase_invert ? -xp_r->gain_linear : xp_r->gain_linear) * 32768.0f) : 0;

        int32_t *dst = buf_out[out];
        if (gain_l_q15 != 0 && gain_r_q15 != 0) {
            for (uint32_t i = 0; i < sample_count; i++)
                dst[i] = fast_mul_q15(buf_l[i], gain_l_q15) + fast_mul_q15(buf_r[i], gain_r_q15);
        } else if (gain_l_q15 != 0) {
            for (uint32_t i = 0; i < sample_count; i++)
                dst[i] = fast_mul_q15(buf_l[i], gain_l_q15);
        } else if (gain_r_q15 != 0) {
            for (uint32_t i = 0; i < sample_count; i++)
                dst[i] = fast_mul_q15(buf_r[i], gain_r_q15);
        } else {
            memset(dst, 0, sample_count * sizeof(int32_t));
        }
    }

    // ========== PASS 5-7: Per-Output EQ + Gain + Delay + Output ==========
    // PDM output index
    int pdm_out = NUM_OUTPUT_CHANNELS - 1;

    if (core1_mode == CORE1_MODE_EQ_WORKER) {
        // --- Dual-core path: Core 0 handles pair 1, Core 1 handles pair 2 ---

        // Dispatch to Core 1
        core1_eq_work.sample_count = sample_count;
        core1_eq_work.vol_mul = vol_mul_master;  // Core 1 uses master-scaled volume
        core1_eq_work.delay_write_idx = delay_write_idx;
        core1_eq_work.spdif_out[0] = audio_buf[1] ? (int32_t *)audio_buf[1]->buffer->bytes : NULL;
        core1_eq_work.work_done = false;
        __dmb();
        core1_eq_work.work_ready = true;
        __sev();

        // Core 0: EQ + gain for outputs 0-1 (SPDIF pair 1)
        for (int out = 0; out < CORE1_EQ_FIRST_OUTPUT; out++) {
            if (!matrix_mixer.outputs[out].enabled) continue;
            if (!matrix_mixer.outputs[out].mute) {
                uint8_t eq_ch = CH_OUT_1 + out;
                if (!is_bypassed && !channel_bypassed[eq_ch])
                    dsp_process_channel_block(filters[eq_ch], buf_out[out], sample_count, eq_ch);
            }
            // Output gain uses vol_mul_master (host vol × master vol, Q15)
            int32_t gain = matrix_mixer.outputs[out].mute ? 0
                           : (int32_t)(matrix_mixer.outputs[out].gain_linear * (float)vol_mul_master);
            if (gain == 0) {
                memset(buf_out[out], 0, sample_count * sizeof(int32_t));
            } else {
                int32_t *dst = buf_out[out];
                for (uint32_t i = 0; i < sample_count; i++)
                    dst[i] = fast_mul_q15(dst[i], gain);
            }
        }

        // Core 0: Delay for outputs 0-1
        if (any_delay_active) {
            for (int out = 0; out < CORE1_EQ_FIRST_OUTPUT; out++) {
                int32_t dly = channel_delay_samples[out];
                if (dly <= 0) continue;
                int32_t *dst = buf_out[out];
                int32_t *dline = delay_lines[out];
                uint32_t widx = delay_write_idx;
                for (uint32_t i = 0; i < sample_count; i++) {
                    dline[widx] = dst[i];
                    dst[i] = dline[(widx - dly) & MAX_DELAY_MASK];
                    widx = (widx + 1) & MAX_DELAY_MASK;
                }
            }
        }

        // Core 0: Peaks for outputs 0..CORE1_EQ_FIRST_OUTPUT-1
        for (int out = 0; out < CORE1_EQ_FIRST_OUTPUT; out++) {
            int32_t peak = 0;
            for (uint32_t i = 0; i < sample_count; i++) {
                int32_t a = abs(buf_out[out][i]);
                if (a > peak) peak = a;
            }
            global_status.peaks[CH_OUT_1 + out] = (uint16_t)(peak >> 13);
            if (peak > CLIP_THRESH_Q28) global_status.clip_flags |= (1u << (CH_OUT_1 + out));
        }
        // PDM is inactive in EQ_WORKER mode
        global_status.peaks[CH_OUT_SUB] = 0;

        // Core 0: S/PDIF conversion for pair 1
        if (audio_buf[0]) {
            if (!matrix_mixer.outputs[0].enabled && !matrix_mixer.outputs[1].enabled) {
                memset(audio_buf[0]->buffer->bytes, 0, sample_count * 8);
            } else {
                int32_t *out_ptr = (int32_t *)audio_buf[0]->buffer->bytes;
                for (uint32_t i = 0; i < sample_count; i++) {
                    out_ptr[i*2]   = clip_s24((buf_out[0][i] + (1 << 5)) >> 6);
                    out_ptr[i*2+1] = clip_s24((buf_out[1][i] + (1 << 5)) >> 6);
                }
            }
        }

        // Wait for Core 1 (EQ + delay + S/PDIF for outputs 2-3)
        while (!core1_eq_work.work_done) {
            __wfe();
        }
        __dmb();

        // Update shared delay write index
        if (any_delay_active) {
            delay_write_idx = (delay_write_idx + sample_count) & MAX_DELAY_MASK;
        }
    } else {
        // --- Single-core path: all outputs on Core 0 ---
        uint32_t saved_delay_write_idx = delay_write_idx;

        // EQ + gain (block-based)
        for (int out = 0; out < NUM_OUTPUT_CHANNELS; out++) {
            if (!matrix_mixer.outputs[out].enabled) continue;
            if (!matrix_mixer.outputs[out].mute) {
                uint8_t eq_ch = CH_OUT_1 + out;
                if (!is_bypassed && !channel_bypassed[eq_ch])
                    dsp_process_channel_block(filters[eq_ch], buf_out[out], sample_count, eq_ch);
            }
            // Output gain uses vol_mul_master (host vol × master vol, Q15)
            int32_t gain = matrix_mixer.outputs[out].mute ? 0
                           : (int32_t)(matrix_mixer.outputs[out].gain_linear * (float)vol_mul_master);
            if (gain == 0) {
                memset(buf_out[out], 0, sample_count * sizeof(int32_t));
            } else {
                int32_t *dst = buf_out[out];
                for (uint32_t i = 0; i < sample_count; i++)
                    dst[i] = fast_mul_q15(dst[i], gain);
            }
        }

        // Delay (all outputs use same base write index)
        if (any_delay_active) {
            for (int out = 0; out < NUM_OUTPUT_CHANNELS; out++) {
                int32_t dly = channel_delay_samples[out];
                if (dly <= 0) continue;
                int32_t *dst = buf_out[out];
                int32_t *dline = delay_lines[out];
                uint32_t widx = saved_delay_write_idx;
                for (uint32_t i = 0; i < sample_count; i++) {
                    dline[widx] = dst[i];
                    dst[i] = dline[(widx - dly) & MAX_DELAY_MASK];
                    widx = (widx + 1) & MAX_DELAY_MASK;
                }
            }
            delay_write_idx = (saved_delay_write_idx + sample_count) & MAX_DELAY_MASK;
        }

        // Peaks for all SPDIF outputs
        for (int out = 0; out < NUM_SPDIF_INSTANCES * 2; out++) {
            int32_t peak = 0;
            for (uint32_t i = 0; i < sample_count; i++) {
                int32_t a = abs(buf_out[out][i]);
                if (a > peak) peak = a;
            }
            global_status.peaks[CH_OUT_1 + out] = (uint16_t)(peak >> 13);
            if (peak > CLIP_THRESH_Q28) global_status.clip_flags |= (1u << (CH_OUT_1 + out));
        }

        // S/PDIF conversion (2 stereo pairs)
        for (int pair = 0; pair < NUM_SPDIF_INSTANCES; pair++) {
            if (!audio_buf[pair]) continue;
            int left_ch = pair * 2;
            int right_ch = pair * 2 + 1;
            if (!matrix_mixer.outputs[left_ch].enabled && !matrix_mixer.outputs[right_ch].enabled) {
                memset(audio_buf[pair]->buffer->bytes, 0, sample_count * 8);
                continue;
            }
            int32_t *out_ptr = (int32_t *)audio_buf[pair]->buffer->bytes;
            for (uint32_t i = 0; i < sample_count; i++) {
                out_ptr[i*2]   = clip_s24((buf_out[left_ch][i] + (1 << 5)) >> 6);
                out_ptr[i*2+1] = clip_s24((buf_out[right_ch][i] + (1 << 5)) >> 6);
            }
        }

#if ENABLE_SUB
        // PDM sub output
        if (matrix_mixer.outputs[pdm_out].enabled) {
            int32_t peak_sub = 0;
            for (uint32_t i = 0; i < sample_count; i++) {
                int32_t abs_sub = abs(buf_out[pdm_out][i]);
                if (abs_sub > peak_sub) peak_sub = abs_sub;
            }
            global_status.peaks[CH_OUT_SUB] = (uint16_t)(peak_sub >> 13);
            if (peak_sub > CLIP_THRESH_Q28) global_status.clip_flags |= (1u << CH_OUT_SUB);
            for (uint32_t i = 0; i < sample_count; i++) {
                pdm_push_sample(buf_out[pdm_out][i], false);
            }
        } else {
            global_status.peaks[CH_OUT_SUB] = 0;
        }
#endif
    }

    // Write input peaks
    global_status.peaks[0] = (uint16_t)(peak_ml >> 13);
    global_status.peaks[1] = (uint16_t)(peak_mr >> 13);
    if (peak_ml > CLIP_THRESH_Q28) global_status.clip_flags |= (1u << CH_MASTER_LEFT);
    if (peak_mr > CLIP_THRESH_Q28) global_status.clip_flags |= (1u << CH_MASTER_RIGHT);
#endif

    // Return all buffers
#if PICO_RP2350
    for (int b = 0; b < 4; b++) {
        if (audio_buf[b]) {
            struct audio_buffer_pool *pool = (b == 0) ? producer_pool_1 :
                                              (b == 1) ? producer_pool_2 :
                                              (b == 2) ? producer_pool_3 : producer_pool_4;
            give_audio_buffer(pool, audio_buf[b]);
        }
    }
#else
    if (audio_buf[0]) give_audio_buffer(producer_pool_1, audio_buf[0]);
    if (audio_buf[1]) give_audio_buffer(producer_pool_2, audio_buf[1]);
#endif

    uint32_t packet_end = time_us_32();

    if (cpu0_load_primed) {
        uint32_t busy_us = packet_end - packet_start;
        uint32_t idle_us = packet_start - cpu0_last_packet_end;
        if (idle_us > 2000) idle_us = 0;   // clamp during USB gaps

        uint32_t total_us = busy_us + idle_us;
        if (total_us > 0) {
            uint32_t inst_q8 = (busy_us * 25600) / total_us;
            cpu0_load_q8 = cpu0_load_q8 - (cpu0_load_q8 >> 3) + (inst_q8 >> 3);
        }
        global_status.cpu0_load = (uint8_t)((cpu0_load_q8 + 128) >> 8);
    } else {
        cpu0_load_primed = true;
    }
    cpu0_last_packet_end = packet_end;
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

// ----------------------------------------------------------------------------
// USB AUDIO PACKET CALLBACKS (pico-extras usb_device)
// ----------------------------------------------------------------------------

static void __not_in_flash_func(_as_audio_packet)(struct usb_endpoint *ep) {
    assert(ep->current_transfer);
    struct usb_buffer *usb_buffer = usb_current_out_packet_buffer(ep);

    usb_audio_packets++;

    // USB packet gap detection — runs at ISR arrival time (not main-loop
    // processing time) to avoid false positives from ring queue delay.
    // audio_ring_last_push_us is file-scope so it can be reset on stream
    // lifecycle transitions in as_set_alternate() and usb_audio_flush_ring().
    {
        uint32_t now = time_us_32();
        if (audio_ring_last_push_us > 0 && !preset_loading) {
            uint32_t gap = now - audio_ring_last_push_us;
            if (gap > 2000 && gap < 50000) {
                spdif_underruns++;
            }
        }
        audio_ring_last_push_us = now;
    }

    // Push raw packet into ring for main-loop DSP processing.
    // Ring-full drops are counted separately from spdif_overruns
    // (different fault class: ring backpressure vs pool pressure).
    usb_audio_ring_push(&audio_ring, usb_buffer->data, usb_buffer->data_len);

    usb_grow_transfer(ep->current_transfer, 1);
    usb_packet_done(ep);
}

static void __not_in_flash_func(_as_sync_packet)(struct usb_endpoint *ep) {
    assert(ep->current_transfer);
    struct usb_buffer *buffer = usb_current_in_packet_buffer(ep);
    assert(buffer->data_max >= 3);
    buffer->data_len = 3;

    // Use SOF-measured feedback; fall back to pre-computed nominal
    uint32_t fb = feedback_10_14;
    if (fb == 0) fb = nominal_feedback_10_14;

    buffer->data[0] = fb;
    buffer->data[1] = fb >> 8u;
    buffer->data[2] = fb >> 16u;

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
    if (alt >= 3) return false;

    uint32_t prev_alt = usb_audio_alt_set;
    usb_audio_alt_set = alt;
    if (alt == 2) {
        usb_input_bit_depth = 24;
    } else {
        usb_input_bit_depth = 16;
    }

    // Arm/disarm SPDIF starvation diagnostics with stream state.
    // Reset only on inactive->active transition so changing 16/24-bit alt
    // doesn't clear counters mid-stream.
    bool active = (alt > 0);
    audio_spdif_set_starvation_monitoring(active);

    // Reset gap-detection timestamp on stream transitions to prevent false
    // underrun counts from stale timestamps across stream lifecycle events.
    audio_ring_last_push_us = 0;

    if (active && prev_alt == 0) {
        audio_spdif_reset_dma_starvations();
        stream_restart_resync_pending = true;
        __dmb();
    } else if (!active && prev_alt > 0) {
        // Stream deactivation: force controller invalid, publish nominal
        extern usb_feedback_ctrl_t fb_ctrl;
        fb_ctrl_stream_stop(&fb_ctrl);
    }

    return true;
}

// Ring overrun accessor (audio_ring is static; vendor_commands.c calls this)
uint32_t usb_audio_ring_overrun_count(void) {
    return audio_ring.overrun_count;
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
// BUFFER STATISTICS HELPERS
// ----------------------------------------------------------------------------

static uint count_pool_free(audio_buffer_pool_t *pool) {
    uint32_t save = spin_lock_blocking(pool->free_list_spin_lock);
    uint count = audio_buffer_list_count(pool->free_list);
    spin_unlock(pool->free_list_spin_lock, save);
    return count;
}

static uint count_pool_prepared(audio_buffer_pool_t *pool) {
    uint32_t save = spin_lock_blocking(pool->prepared_list_spin_lock);
    uint count = audio_buffer_list_count(pool->prepared_list);
    spin_unlock(pool->prepared_list_spin_lock, save);
    return count;
}

void get_slot_consumer_stats(uint slot, uint *cons_free, uint *cons_prepared, uint *playing) {
    // Output-type switches teardown/setup pools in main-loop context while USB
    // control requests may still run in IRQ context. Avoid dereferencing pool
    // pointers during that transition window.
    if (output_type_switch_in_progress) {
        *cons_free = SPDIF_CONSUMER_BUFFER_COUNT;
        *cons_prepared = 0;
        *playing = 0;
        return;
    }

    if (output_types[slot] == OUTPUT_TYPE_I2S) {
        audio_i2s_instance_t *inst = i2s_instance_ptrs[slot];
        if (!inst || !inst->consumer_pool) {
            *cons_free = 0;
            *cons_prepared = 0;
            *playing = 0;
            return;
        }
        *cons_free = count_pool_free(inst->consumer_pool);
        *cons_prepared = count_pool_prepared(inst->consumer_pool);
        *playing = (inst->playing_buffer != NULL) ? 1 : 0;
    } else {
        audio_spdif_instance_t *inst = spdif_instance_ptrs[slot];
        if (!inst || !inst->consumer_pool) {
            *cons_free = 0;
            *cons_prepared = 0;
            *playing = 0;
            return;
        }
        *cons_free = count_pool_free(inst->consumer_pool);
        *cons_prepared = count_pool_prepared(inst->consumer_pool);
        *playing = (inst->playing_buffer != NULL) ? 1 : 0;
    }
}

uint get_slot_consumer_fill(uint slot) {
    // See get_slot_consumer_stats(): never touch per-slot pools while a type
    // switch is mutating ownership/state.
    if (output_type_switch_in_progress) {
        return 0;
    }

    uint cons_free = SPDIF_CONSUMER_BUFFER_COUNT;

    if (output_types[slot] == OUTPUT_TYPE_I2S) {
        audio_i2s_instance_t *inst = i2s_instance_ptrs[slot];
        if (inst && inst->consumer_pool) {
            cons_free = count_pool_free(inst->consumer_pool);
        }
    } else {
        audio_spdif_instance_t *inst = spdif_instance_ptrs[slot];
        if (inst && inst->consumer_pool) {
            cons_free = count_pool_free(inst->consumer_pool);
        }
    }

    if (cons_free > SPDIF_CONSUMER_BUFFER_COUNT) cons_free = SPDIF_CONSUMER_BUFFER_COUNT;
    return SPDIF_CONSUMER_BUFFER_COUNT - cons_free;
}

// Servo-critical: update slot-0 fill every packet with minimal work.
static inline void update_slot0_fill_fast(void) {
    spdif0_consumer_fill = (uint8_t)get_slot_consumer_fill(0);
}

void reset_buffer_watermarks(void) {
    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        spdif_consumer_min_fill_pct[i] = 100;
        spdif_consumer_max_fill_pct[i] = 0;
    }
    pdm_dma_min_fill_pct = 100;
    pdm_dma_max_fill_pct = 0;
    pdm_ring_min_fill_pct = 100;
    pdm_ring_max_fill_pct = 0;
}

static void update_buffer_watermarks(void) {
    uint consumer_capacity = SPDIF_CONSUMER_BUFFER_COUNT;

    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        uint fill = get_slot_consumer_fill(i);
        uint8_t cons_pct = (uint8_t)(fill * 100 / consumer_capacity);
        if (cons_pct < spdif_consumer_min_fill_pct[i]) spdif_consumer_min_fill_pct[i] = cons_pct;
        if (cons_pct > spdif_consumer_max_fill_pct[i]) spdif_consumer_max_fill_pct[i] = cons_pct;
        if (i == 0) spdif0_consumer_fill = (uint8_t)fill;
    }

    if (pdm_enabled) {
        uint8_t dma_pct = pdm_get_dma_fill_pct();
        if (dma_pct < pdm_dma_min_fill_pct) pdm_dma_min_fill_pct = dma_pct;
        if (dma_pct > pdm_dma_max_fill_pct) pdm_dma_max_fill_pct = dma_pct;

        uint8_t ring_pct = pdm_get_ring_fill_pct();
        if (ring_pct < pdm_ring_min_fill_pct) pdm_ring_min_fill_pct = ring_pct;
        if (ring_pct > pdm_ring_max_fill_pct) pdm_ring_max_fill_pct = ring_pct;
    }
}


// ----------------------------------------------------------------------------
// DEVICE-LEVEL SETUP REQUEST HANDLER (WCID / MS OS descriptors)
// ----------------------------------------------------------------------------

static bool device_setup_request_handler(struct usb_device *dev, struct usb_setup_packet *setup) {
    (void)dev;
    setup = __builtin_assume_aligned(setup, 4);

    // Intercept GET_DESCRIPTOR(String, 0xEE) — MS OS String Descriptor
    // The default string handler only converts ASCII→UTF-16, but the MS OS
    // string descriptor is a fixed 18-byte binary blob that must be returned raw.
    if (!(setup->bmRequestType & USB_REQ_TYPE_TYPE_MASK) &&
        (setup->bmRequestType & USB_DIR_IN) &&
        setup->bRequest == USB_REQUEST_GET_DESCRIPTOR &&
        setup->wValue == 0x03EE) {
        uint16_t len = setup->wLength < MS_OS_STRING_DESC_LEN ? setup->wLength : MS_OS_STRING_DESC_LEN;
        vendor_send_response(ms_os_string_descriptor, len);
        return true;
    }

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
    if (index >= 1 && index <= count_of(descriptor_strings)) {
        return descriptor_strings[index - 1];
    }
    return "";
}

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

    // Initialize Core 1 EQ worker pointer to shared output buffer
    core1_eq_work.buf_out = buf_out;

    // Initialize ADC for temperature sensor
    adc_init();
    adc_set_temp_sensor_enabled(true);

    usb_device_start();
}
