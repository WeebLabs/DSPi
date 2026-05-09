/*
 * USB Audio Interface Header for DSPi
 * pico-extras usb_device UAC1 implementation
 */

#ifndef USB_AUDIO_H
#define USB_AUDIO_H

#include "config.h"
#include "pico/types.h"
#include "loudness.h"
#include "leveller.h"

// ----------------------------------------------------------------------------
// AUDIO STATE (exposed to Main)
// ----------------------------------------------------------------------------

typedef struct {
    uint32_t freq;
    int16_t volume;
    int16_t vol_mul;
    bool mute;
} AudioState;

extern volatile AudioState audio_state;
extern volatile bool bypass_master_eq;

// Per-channel gain and mute (output channels only: L, R, Sub)
extern volatile float channel_gain_db[3];
extern volatile int32_t channel_gain_mul[3];
extern volatile float channel_gain_linear[3];
extern volatile bool channel_mute[3];

// Per-input-channel preamp gain (indexed by input channel: 0=USB L, 1=USB R)
extern volatile float global_preamp_db[NUM_INPUT_CHANNELS];
extern volatile int32_t global_preamp_mul[NUM_INPUT_CHANNELS];
extern volatile float global_preamp_linear[NUM_INPUT_CHANNELS];

// Master volume — device-side ceiling on all output (does not affect DSP stages)
extern volatile float master_volume_db;
extern volatile float master_volume_linear;
extern volatile int32_t master_volume_q15;

// Vendor-channel user mute (REQ_SET_USER_MUTE).  Separate from audio_state.mute
// because the two have different gating semantics: UAC1 mute is USB-gated by the
// audio pipeline (the OS mute key must not silence SPDIF playback), whereas the
// vendor channel always applies — symmetric with REQ_SET_USER_VOLUME's
// always-apply contract.  Pipeline ORs them: muted = (audio_state.mute &&
// host_active) || user_mute.  audio_state.volume IS shared with UAC1 because the
// volume value itself is source-agnostic (the gating is inside the apply call,
// not the field), so no parallel field is needed for volume.
extern volatile bool user_mute;

// Loudness compensation
extern volatile bool loudness_enabled;
extern volatile float loudness_ref_spl;
extern volatile float loudness_intensity_pct;
extern volatile bool loudness_recompute_pending;
// Pointer-typed `volatile` (qualifier on the pointer itself, not the
// pointee) — the audio pipeline reads this every packet, and any write
// from apply_vol_index_to_audio() must not be hoisted/cached across
// the read by the optimiser even when both sides live in the same TU.
extern const LoudnessCoeffs *volatile current_loudness_coeffs;

// The vol_index most recently applied to vol_mul + loudness coefficients.
// Single source of truth for "what user-perceived volume is active right
// now", used by the loudness re-enable and loudness-table-recompute paths
// so they re-key the coefficient pointer at the *actual* current vol_index
// — not the USB-cached audio_state.volume, which would be wrong when an
// alternative volume owner (LG Sound Sync) is driving vol_mul on SPDIF
// input.  Initialised to 0 (silent) at boot — matches the BSS-zero state
// of vol_mul before audio_set_volume() runs on first USB enumeration.
extern volatile uint8_t effective_vol_index;

// Crossfeed
#include "crossfeed.h"
extern volatile CrossfeedConfig crossfeed_config;
extern volatile bool crossfeed_update_pending;
extern volatile bool crossfeed_bypassed;
extern CrossfeedState crossfeed_state;

// Volume Leveller
extern volatile LevellerConfig leveller_config;
extern volatile bool leveller_update_pending;
extern volatile bool leveller_reset_pending;
extern volatile bool leveller_bypassed;
extern LevellerCoeffs leveller_coeffs;
extern LevellerState leveller_state;

// Matrix Mixer
extern MatrixMixer matrix_mixer;

// ----------------------------------------------------------------------------
// EQ UPDATE FLAGS (for main loop to handle)
// ----------------------------------------------------------------------------

extern volatile bool eq_update_pending;
extern volatile EqParamPacket pending_packet;
extern volatile bool rate_change_pending;
extern volatile uint32_t pending_rate;
extern volatile bool bulk_params_pending;
extern volatile bool output_type_switch_in_progress;
extern uint8_t bulk_param_buf[];
extern char channel_names[NUM_CHANNELS][PRESET_NAME_LEN];
// Compute the canonical default name for a channel given input source and output slot types.
// `output_types` may be NULL when `ch` is an input channel or PDM.
void get_default_channel_name(int ch, uint8_t input_source,
                              const uint8_t *output_types, char *buf);

// Core 1 mode derivation (used by preset load and bulk params)
Core1Mode derive_core1_mode(void);

// ----------------------------------------------------------------------------
// OUTPUT SLOT STATE
// ----------------------------------------------------------------------------

extern uint8_t output_pins[];
extern uint8_t output_types[];
extern struct audio_spdif_instance *spdif_instance_ptrs[];
extern struct audio_i2s_instance *i2s_instance_ptrs[];
extern struct audio_buffer_pool *producer_pools[];

// Producer pool aliases (individual named, used by pipeline)
extern struct audio_buffer_pool *producer_pool_1;
extern struct audio_buffer_pool *producer_pool_2;
#if PICO_RP2350
extern struct audio_buffer_pool *producer_pool_3;
extern struct audio_buffer_pool *producer_pool_4;
#endif

// I2S clock configuration
extern uint8_t i2s_bck_pin;
extern uint8_t i2s_mck_pin;
extern bool    i2s_mck_enabled;
extern uint16_t i2s_mck_multiplier;

// ----------------------------------------------------------------------------
// DEFERRED COMMAND FLAGS (written by vendor SET, read by main loop)
// ----------------------------------------------------------------------------

extern volatile bool flash_set_name_pending;
extern uint8_t flash_set_name_slot;
extern char    flash_set_name_buf[];

extern volatile bool flash_set_startup_pending;
extern uint8_t flash_set_startup_mode;
extern uint8_t flash_set_startup_slot;

extern volatile bool flash_set_include_pins_pending;
extern uint8_t flash_set_include_pins_val;

extern volatile bool flash_set_master_volume_mode_pending;
extern uint8_t flash_set_master_volume_mode_val;
extern volatile bool flash_save_master_volume_pending;

extern volatile bool flash_set_spdif_rx_pin_pending;

extern volatile bool save_params_pending;
extern volatile bool preset_save_pending;
extern volatile bool preset_load_pending;
extern volatile bool factory_reset_pending;
extern volatile uint8_t pending_preset_load_slot;
extern volatile uint8_t pending_preset_save_slot;
extern volatile uint16_t preset_delete_mask;

extern volatile uint8_t output_type_change_mask;
extern volatile uint8_t pending_output_types[];
extern volatile bool stream_restart_resync_pending;

// Sync state (used by vendor GET handler for buffer stats)
extern volatile bool sync_started;

// ----------------------------------------------------------------------------
// VOLUME CONSTANT (shared by audio_set_volume and vendor command handlers)
// ----------------------------------------------------------------------------

#define CENTER_VOLUME_INDEX 60

// ----------------------------------------------------------------------------
// SHARED HELPERS (defined in usb_audio.c, called from vendor_commands.c)
// ----------------------------------------------------------------------------

void update_preamp(uint8_t ch, float db);
void update_master_volume(float db);

// Update the user-perceived volume — same quantity the UAC1 host slider drives.
// Writes audio_state.volume in 8.8-fixed-point dB encoding so a subsequent UAC1
// GET_CUR roundtrips the same value, then unconditionally calls
// apply_vol_index_to_audio() so vol_mul AND the loudness-coefficient pointer
// follow.  Bypasses the input-source guard inside audio_set_volume() —
// vendor/hardware-control callers want the change applied immediately, not
// silently cached for a future SPDIF→USB switch.  Caller-side conventions
// (e.g. "Console only writes during non-USB input") are not enforced here.
void update_user_volume(float db);

// Apply a vol_index in [0..CENTER_VOLUME_INDEX] to the live audio path.
// Updates audio_state.vol_mul (consumed click-free by the audio pipeline's
// per-packet ramp) and refreshes current_loudness_coeffs when loudness is
// enabled.  Does NOT touch audio_state.volume — the caller owns the question
// of whether the host's reported volume value is changing or merely being
// re-applied.
//
// Single funnel for every owner of the user-perceived volume (USB host
// slider via audio_set_volume(), LG Sound Sync via lg_sound_sync.c, etc.) so
// loudness compensation always tracks the same quantity that drives vol_mul.
//
// Main-loop only.  vol_mul is a 16-bit field that on RP2040 is not single-
// instruction atomic relative to the audio pipeline reading it; the existing
// pipeline code tolerates that races through the per-packet ramp, but the
// design intent is for this to be called from the main thread.
void apply_vol_index_to_audio(uint8_t vol_index);

// ----------------------------------------------------------------------------
// API
// ----------------------------------------------------------------------------

void usb_sound_card_init(void);
void audio_set_volume(int16_t volume);

// Push a master-volume change to the host over the interrupt IN endpoint.
// Called internally from update_master_volume(); vendor_commands.c brackets
// its update_master_volume() call with notify_master_vol_host_initiated so
// the echo-suppression #define (NOTIFY_SUPPRESS_HOST_ECHO) can distinguish
// host-originated from device-originated changes.  Only sets pending state;
// the actual xfer fires from usb_notify_tick() (main loop) or xfer_cb.
void usb_notify_master_volume(float db);
extern volatile bool notify_master_vol_host_initiated;

// Drain any pending device→host notifications to the interrupt IN endpoint.
// Called once per iteration from the main loop in main.c.
void usb_notify_tick(void);

// USB audio ring buffer — main-loop entry points for decoupled DSP processing
void usb_audio_drain_ring(void);   // Process all pending USB audio packets
void usb_audio_flush_ring(void);   // Discard stale ring data + reset gap timestamp

// Exposed in usb_descriptors.h (populated by main.c from the chip unique ID).

#endif // USB_AUDIO_H
