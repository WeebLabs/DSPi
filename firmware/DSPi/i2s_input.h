/*
 * i2s_input.h — I2S receiver integration for DSPi (Pico-master)
 *
 * High-level subsystem mirroring spdif_input.{c,h}. The Pico generates
 * BCK/LRCLK from sys_clk via PIO, so input rate = output rate (same
 * crystal, same divider math) and no clock servo is required. The
 * subsystem orchestrates:
 *   - Clock-domain start (clocks-only master if no I2S TX is configured)
 *   - RX SM + DMA ring setup/teardown
 *   - Per-poll ring drain → preamp → buf_l/buf_r → process_input_block
 *   - Source-switching lifecycle (start, stop, set_pin, set_frequency)
 *
 * Phase 2 constraint: I2S input is mutually exclusive with I2S TX outputs
 * (caller responsible — vendor commands enforce). The clocks-only master
 * SM is always used; no live BCK/LRCLK ownership handoff between TX/RX.
 */

#ifndef I2S_INPUT_H
#define I2S_INPUT_H

#include <stdint.h>
#include <stdbool.h>

// I2S input state machine
typedef enum {
    I2S_INPUT_INACTIVE = 0,   // Subsystem stopped (input source != I2S)
    I2S_INPUT_STARTING = 1,   // SM started, waiting for prefill to reach 50%
    I2S_INPUT_ACTIVE   = 2,   // Streaming
} I2sInputState;

// Status packet returned by REQ_GET_I2S_INPUT_STATUS (16 bytes for symmetry
// with SpdifRxStatusPacket; useful during bring-up).
typedef struct __attribute__((packed)) {
    uint8_t  state;              // I2sInputState
    uint8_t  input_source;       // Active InputSource
    uint8_t  din_pin;
    uint8_t  reserved0;
    uint32_t sample_rate;        // Current Fs (Pico-master, == audio_state.freq)
    uint32_t ring_fill_words;    // Words available in ring (write - read)
    uint16_t ring_size_words;
    uint16_t reserved1;
} I2sInputStatusPacket;          // 16 bytes

// Initialize subsystem (called once at boot; no HW claimed yet)
void i2s_input_init(void);

// Start hardware (called when switching to I2S input).
// Starts clocks-only master SM (drives BCK/LRCLK), claims RX SM + DMA,
// arms ring, enters STARTING state. Outputs remain muted until prefill
// completes (handled by main-loop polling).
void i2s_input_start(void);

// Stop hardware (called when switching away from I2S input).
// Tears down RX SM/DMA, stops clocks-only master, releases pins.
void i2s_input_stop(void);

// Main-loop poll: drain ring → apply preamp → fill buf_l/buf_r → call
// process_input_block(). Returns number of stereo samples processed.
uint32_t i2s_input_poll(void);

// Get current state
I2sInputState i2s_input_get_state(void);

// True when state == I2S_INPUT_ACTIVE (Pico-master can't lose lock)
bool i2s_input_get_lock(void);

// Update SM divider on a sample-rate change. Called from perform_rate_change.
void i2s_input_set_frequency(uint32_t fs);

// Mark prefill complete (called by main-loop polling block once consumer
// pools fill to 50%). Promotes STARTING → ACTIVE.
void i2s_input_mark_active(void);

// Status packet for vendor command response
void i2s_input_get_status(I2sInputStatusPacket *out);

#endif // I2S_INPUT_H
