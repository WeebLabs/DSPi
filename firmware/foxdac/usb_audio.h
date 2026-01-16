#ifndef USB_AUDIO_H
#define USB_AUDIO_H

#include "config.h"
#include "pico/usb_device.h"

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

// ----------------------------------------------------------------------------
// VENDOR INTERFACE API
// ----------------------------------------------------------------------------

// Queue a response packet to be sent on the vendor IN endpoint
// Returns true if queued successfully, false if queue full
bool vendor_queue_response(const VendorRespPacket *resp);

// Check if vendor interface is connected and ready
bool vendor_interface_ready(void);

#endif // USB_AUDIO_H
