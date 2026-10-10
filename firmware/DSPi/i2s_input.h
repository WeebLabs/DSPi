/*
 * i2s_input.h - I2S receiver integration for DSPi
 *
 * In MASTER clock mode (default) the input is synchronous to the device's
 * own clock domain, so there is no clock servo, no rate detection and no
 * lock state machine. Two roles exist:
 *
 *   clock master - no output slot is I2S; the input SM drives BCK/LRCLK
 *                  via side-set while sampling data
 *   slave        - at least one output slot is I2S; the input SM samples
 *                  data against the BCK/LRCLK pads driven by the output
 *                  clock master
 *
 * In SLAVE clock mode (i2s_clock_mode == I2S_CLOCK_MODE_SLAVE) an EXTERNAL
 * master drives BCK/LRCLK and both pins are inputs. Every pair runs the
 * wait-driven slave program against the external pads, the sample rate is
 * auto-detected from the DMA word rate, and the i2s_slave_* API provides a
 * SPDIF-style lock state machine plus a clock servo that rate-matches the
 * SPDIF/ADAT outputs to the external clock domain.
 *
 * Reuses the SPDIF RX PIO state machine and DMA channels, which are free
 * whenever SPDIF input is inactive (inputs are switched, never mixed).
 */

#ifndef I2S_INPUT_H
#define I2S_INPUT_H

#include <stdint.h>
#include <stdbool.h>

// Default data pin: PICO_I2S_RX_PIN_DEFAULT in audio_input.h

// I2S RX state. No lock concept: clocks are ours, so the input is either
// running or not selected.
typedef enum {
    I2S_INPUT_INACTIVE = 0,   // Hardware stopped (not selected as input)
    I2S_INPUT_RUNNING  = 1,   // Receiving and processing audio
} I2sInputState;

// Initialize the subsystem (called once at boot, no hardware claimed)
void i2s_input_init(void);

// Start I2S RX hardware in the given role. Claims the SPDIF RX PIO SM and
// DMA channels; caller must ensure SPDIF RX is inactive.
void i2s_input_start(bool clock_master);

// Stop I2S RX hardware and release all claimed resources
void i2s_input_stop(void);

// Re-phase a running slave-role input after the I2S TX clock master has
// been restarted (which resets LRCLK phase). No-op unless RUNNING in the
// slave role. Called at the end of complete_pipeline_reset() and
// enable_outputs_in_sync().
void i2s_input_resync(void);

// Main-loop poll: drain the DMA ring, apply preamp, feed the pipeline.
// Returns number of stereo frames processed.
uint32_t i2s_input_poll(void);

// Push one silent block through the pipeline to prefill the output consumer
// pools. Used only during a slave-role prefill, where the input is clocked by
// an I2S output and so cannot supply samples while the outputs are drained.
void i2s_input_prefill_silence(uint32_t frames);

// Get current state
I2sInputState i2s_input_get_state(void);

// True if RUNNING in the clock-master role
bool i2s_input_is_clock_master(void);

// BCK GPIO of the running session (snapshot taken at start); the live
// i2s_bck_pin global when the input is stopped. LRCLK is this + 1.
uint8_t i2s_input_active_bck_pin(void);

// ============================================================================
// External-clock slave mode (i2s_clock_mode == I2S_CLOCK_MODE_SLAVE)
// ============================================================================

// Lock state while the input runs in the external-clock slave role.
// Mirrors SpdifInputState semantics.
typedef enum {
    I2S_SLAVE_INACTIVE  = 0,   // not in slave role (or input stopped)
    I2S_SLAVE_ACQUIRING = 1,   // measuring external clocks, no lock yet
    I2S_SLAVE_RELOCKING = 2,   // clocks lost or rate changed; output muted
    I2S_SLAVE_LOCKED    = 3,   // locked to a supported external rate
} I2sSlaveState;

// Wire status packet for REQ_GET_I2S_SLAVE_STATUS (16 bytes)
typedef struct __attribute__((packed)) {
    uint8_t  state;           // I2sSlaveState
    uint8_t  clock_mode;      // live i2s_clock_mode
    uint8_t  lock_count;      // locks since boot (saturates at 255)
    uint8_t  loss_count;      // losses since boot (saturates at 255)
    uint32_t detected_rate;   // snapped Hz (44100/48000/96000), 0 unless LOCKED
    uint32_t measured_hz;     // raw measured external rate, 0 when no clocks
    uint8_t  slip_count;      // framing slips since boot (saturates at 255);
                              // each one also increments loss_count via the
                              // relock it forces
    uint8_t  reserved[3];
} I2sSlaveStatusPacket;

// Main-loop poll while the input runs in the slave role: accumulates the
// DMA word count, measures the external rate, and drives the lock state
// machine. Cheap no-op in other roles.
void i2s_slave_poll(void);

// Adjust SPDIF/ADAT output PIO dividers to track the measured external
// rate (rate loop + consumer-fill trim). Call every main-loop iteration
// while LOCKED; rate-limits internally.
void i2s_slave_update_clock_servo(void);

// Arm a deferred pipeline rate change if the locked external rate differs
// from audio_state.freq. Returns true if a change was armed.
bool i2s_slave_check_rate_change(void);

I2sSlaveState i2s_slave_get_state(void);
uint32_t i2s_slave_get_detected_rate(void);

void i2s_slave_get_status(I2sSlaveStatusPacket *out);

#endif // I2S_INPUT_H
