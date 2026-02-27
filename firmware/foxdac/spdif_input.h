/*
 * S/PDIF Input Receiver — RP2350 only
 *
 * Always-running receiver that scans for S/PDIF signal at boot.
 * Status is always queryable regardless of active audio source.
 * Switchable USB/SPDIF input via vendor commands.
 */

#ifndef SPDIF_INPUT_H
#define SPDIF_INPUT_H

#include <stdint.h>
#include <stdbool.h>

// Receiver states (matches vendor command 0x82 spec)
typedef enum {
    SPDIF_IN_NO_SIGNAL  = 0,
    SPDIF_IN_ACQUIRING  = 1,
    SPDIF_IN_LOCKED     = 2
} SpdifInState;

// Audio source selection
typedef enum {
    AUDIO_SOURCE_USB   = 0,
    AUDIO_SOURCE_SPDIF = 1
} AudioSource;

// Status struct (matches vendor command 0x82 response — 20 bytes)
typedef struct __attribute__((packed)) {
    uint32_t state;
    uint32_t sample_rate;
    uint32_t parity_err_count;
    uint8_t  c_bits[5];
    uint8_t  _pad[3];
} SpdifInStatus;

// Lifecycle — receiver starts automatically and runs continuously
void spdif_input_init(uint8_t data_pin);
void spdif_input_poll(void);  // Called from main loop (handles deferred state transitions)

// State queries (always valid — receiver runs in background)
SpdifInState spdif_input_get_state(void);
uint32_t spdif_input_get_sample_rate(void);
void spdif_input_get_status(SpdifInStatus *out);

// Audio data consumption (called from process_audio_packet in SPDIF mode)
uint32_t spdif_input_read_samples(float *buf_l, float *buf_r, uint32_t max_samples);
uint32_t spdif_input_get_fifo_count(void);

// TX clock feedback (called from process_audio_packet in SPDIF mode)
void spdif_input_adjust_tx_clock(void);

// Source switching (called from vendor command handler)
bool spdif_input_switch_source(AudioSource new_source);
AudioSource spdif_input_get_source(void);

#endif // SPDIF_INPUT_H
