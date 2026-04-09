/*
 * USB Audio Interface Header for DSPi
 * pico-extras usb_device UAC1 implementation
 */

#ifndef USB_AUDIO_H
#define USB_AUDIO_H

#include "config.h"

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

// Loudness compensation
extern volatile bool loudness_enabled;
extern volatile float loudness_ref_spl;
extern volatile float loudness_intensity_pct;
extern volatile bool loudness_recompute_pending;

// Crossfeed
#include "crossfeed.h"
extern volatile CrossfeedConfig crossfeed_config;
extern volatile bool crossfeed_update_pending;
extern volatile bool crossfeed_bypassed;
extern CrossfeedState crossfeed_state;

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
void get_default_channel_name(int ch, char *buf);

// Core 1 mode derivation (used by preset load and bulk params)
Core1Mode derive_core1_mode(void);

// ----------------------------------------------------------------------------
// API
// ----------------------------------------------------------------------------

void usb_sound_card_init(void);
void audio_set_volume(int16_t volume);

// USB audio ring buffer — main-loop entry points for decoupled DSP processing
void usb_audio_drain_ring(void);   // Process all pending USB audio packets
void usb_audio_flush_ring(void);   // Discard stale ring data + reset gap timestamp

// Expose serial string buffer for main.c to write unique board ID
extern char *usb_descriptor_str_serial;

#endif // USB_AUDIO_H
