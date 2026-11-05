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

// Stop the receiver for a stop-and-restart of the same source (flash write, reconfig).
// Unlike stop, the VCXO keeps its trim, so running output clocks do not step.
void spdif_input_suspend(void);

// Main-loop poll: check lock/unlock events, read FIFO, feed pipeline.
// Returns number of stereo samples processed (0 if no data).
uint32_t spdif_input_poll(void);

// Drop the input received but not yet delivered (whole stereo pairs). Prefill
// start only: nothing has been rendered since the outputs stopped, so the
// stream simply starts later; see current_architecture.md "Output Latency".
void spdif_input_discard_queued(void);

// Clock servo: trim the soft VCXO to track the SPDIF input clock.
// Called from main loop after spdif_input_poll().
void spdif_input_update_clock_servo(void);

// Lock-time seed. Ready once LOCKED with SERVO_LONG_MIN_US of rate window; the
// seed then steps the VCXO to the measured rate, so call it only with outputs stopped.
bool spdif_input_seed_ready(void);
void spdif_input_seed_servo(void);

// Measured input rate in mHz from the long window, 0 until one is valid.
uint32_t spdif_input_measured_rate_mhz(void);

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

#include "config.h"
#if DSPI_CLOCK_DIAG
// Rate-estimator internals for the CLKDIAG dump. which_long is true once a
// long-window rate exists (from the lock-time seed onward).
typedef struct {
    float    hz_long;      // count-based long-window rate estimate
    bool     which_long;   // true = hz_long valid
    uint32_t span_ms;      // long-window anchor span in ms
} SpdifServoDiag;

void spdif_input_get_servo_diag(SpdifServoDiag *out);
#endif // DSPI_CLOCK_DIAG

#endif // SPDIF_INPUT_H
