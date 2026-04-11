/*
 * spdif_input.h — S/PDIF receiver integration for DSPi
 *
 * State machine, audio extraction, clock servo, and status query
 * functions for the pico_spdif_rx library integration.
 */

#ifndef SPDIF_INPUT_H
#define SPDIF_INPUT_H

#include <stdint.h>
#include <stdbool.h>

// SPDIF RX input state machine
typedef enum {
    SPDIF_INPUT_INACTIVE   = 0,   // RX hardware stopped (not selected as input)
    SPDIF_INPUT_ACQUIRING  = 1,   // Waiting for initial signal lock
    SPDIF_INPUT_LOCKED     = 2,   // Receiving and processing audio
    SPDIF_INPUT_RELOCKING  = 3,   // Signal lost, waiting for re-lock
} SpdifInputState;

// SPDIF RX status packet (returned by REQ_GET_SPDIF_RX_STATUS, 16 bytes)
typedef struct __attribute__((packed)) {
    uint8_t  state;              // SpdifInputState enum
    uint8_t  input_source;       // Current active InputSource enum
    uint8_t  lock_count;         // Number of successful locks since activation
    uint8_t  loss_count;         // Number of lock losses since activation
    uint32_t sample_rate;        // Detected sample rate in Hz (0 if not locked)
    uint32_t parity_errors;      // Cumulative parity error count
    uint16_t fifo_fill_pct;      // RX FIFO fill percentage (0-100)
    uint16_t reserved;
} SpdifRxStatusPacket;           // 16 bytes

// Initialize SPDIF RX subsystem (called once at boot, no HW claimed)
void spdif_input_init(void);

// Start SPDIF RX hardware (called when switching to SPDIF input)
void spdif_input_start(void);

// Stop SPDIF RX hardware (called when switching away from SPDIF input)
void spdif_input_stop(void);

// Main-loop poll: check lock/unlock events, read FIFO, feed pipeline.
// Returns number of stereo samples processed (0 if no data).
uint32_t spdif_input_poll(void);

// Clock servo: adjust output PIO dividers to track SPDIF input clock.
// Called from main loop after spdif_input_poll().
void spdif_input_update_clock_servo(void);

// Get current SPDIF input state
SpdifInputState spdif_input_get_state(void);

// Get detected sample rate (0 if not locked)
uint32_t spdif_input_get_sample_rate(void);

// Get status packet for vendor command response (0xE2)
void spdif_input_get_status(SpdifRxStatusPacket *out);

// Get IEC 60958 channel status bytes for vendor command response (0xE3)
// out must point to at least 24 bytes
void spdif_input_get_channel_status(uint8_t *out_24_bytes);

// Called by main loop when lock event fires and rate differs from current
// Returns true if rate change was needed
bool spdif_input_check_rate_change(void);

#endif // SPDIF_INPUT_H
