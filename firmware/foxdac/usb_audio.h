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

// Preamp
extern volatile float global_preamp_linear;

// ----------------------------------------------------------------------------
// EQ UPDATE FLAGS (for main loop to handle)
// ----------------------------------------------------------------------------

extern volatile bool eq_update_pending;
extern volatile EqParamPacket pending_packet;
extern volatile bool rate_change_pending;
extern volatile uint32_t pending_rate;

// ----------------------------------------------------------------------------
// API
// ----------------------------------------------------------------------------

void usb_sound_card_init(void);
void audio_set_volume(int16_t volume);

// Expose serial string buffer for main.c to write unique board ID
extern char *usb_descriptor_str_serial;

#endif // USB_AUDIO_H
