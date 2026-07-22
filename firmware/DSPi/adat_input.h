/*
 * adat_input.h; ADAT lightpipe 8-channel input (RP2350 only)
 *
 * A new selectable input source (INPUT_SOURCE_ADAT): 8 channels of 24-bit
 * audio at 44.1/48 kHz from one TOSLINK receiver. No SMUX/96k.
 *
 * Two clock modes (adat_clock_mode, audio_input.h):
 *   MASTER: the incoming stream is locked to our own clock domain (the far
 *           end syncs to DSPi's ADAT output). No rate detection, no servo;
 *           the device rate authority is REQ_SET_INPUT_RATE, as in I2S
 *           master mode. Above 48 kHz the input parks with rate_ok = false.
 *   SLAVE:  external gear owns the clock. The wire rate is acquired by
 *           probing the exact 48/44.1 kHz decoder timings and requiring a
 *           valid frame-header run. Once locked, RX DMA word timing supplies
 *           the fine servo reference and all outputs track it via
 *           input_servo_apply(), exactly like SPDIF input. Never parked:
 *           rate_ok is always true (a >48 kHz device rate at switch-in is
 *           resolved by the deferred rate change after lock, under the
 *           switch-in mute).
 *
 * The receiver itself is identical in both modes: a PIO NRZI decoder
 * (adat_input.pio) streams decoded bits into a DMA ring; the main-loop poll
 * finds the frame sync, verifies the header per frame, unstuffs 8 channels
 * and feeds process_input_block().
 */

#ifndef ADAT_INPUT_H
#define ADAT_INPUT_H

#include <stdint.h>
#include <stdbool.h>

// Lock state. SYNCING = rate known (or ours), searching for / verifying the
// frame sync header. RELOCKING mutes outputs exactly like a SPDIF lock loss.
typedef enum {
    ADAT_INPUT_INACTIVE  = 0,   // hardware stopped (not selected as input)
    ADAT_INPUT_ACQUIRING = 1,   // slave: probing 48/44.1 timing; master: waiting for a valid device rate
    ADAT_INPUT_SYNCING   = 2,   // searching for frame sync
    ADAT_INPUT_LOCKED    = 3,   // decoding audio
    ADAT_INPUT_RELOCKING = 4,   // signal or rate lost; output muted
} AdatInputState;

// Wire status packet for REQ_GET_ADAT_INPUT_STATUS (20 bytes)
typedef struct __attribute__((packed)) {
    uint8_t  state;          // AdatInputState
    uint8_t  clock_mode;     // live adat_clock_mode (0 master / 1 slave)
    uint8_t  enabled;        // configured enable
    uint8_t  pin;            // configured RX GPIO (0xFF = unset)
    uint8_t  rate_ok;        // 0 = master mode parked (device rate > 48 kHz); always 1 in slave mode
    uint8_t  lock_count;     // locks since boot (saturates at 255)
    uint8_t  loss_count;     // lock losses since boot (saturates at 255)
    uint8_t  slip_count;     // losses caused by header verification failure
    uint16_t header_err;     // cumulative header mismatches (wraps)
    uint16_t reserved;
    uint32_t detected_rate;  // Hz while LOCKED (master mode: device rate), else 0
    uint32_t measured_hz;    // slave: raw measured wire rate, 0 in master mode
} AdatInputStatusPacket;

#if PICO_RP2350

// Boot-time init; no hardware claimed.
void adat_input_init(void);

// Start/stop the receiver. Claims PIO1 SM2, DMA channel 15 and the RX pin's
// input enable; caller (main-loop source switch) guarantees single-start.
void adat_input_start(void);
void adat_input_stop(void);

// Main-loop poll: rate machine, sync search, frame decode, pipeline feed.
// Returns frames delivered to process_input_block().
uint32_t adat_input_poll(void);

// Servo outputs to the recovered wire rate (slave mode + LOCKED only; cheap
// no-op otherwise). Call every main-loop iteration; rate-limits internally.
void adat_input_update_clock_servo(void);

// Arm a deferred pipeline rate change if the locked wire rate differs from
// audio_state.freq (slave mode). Returns true if a change was armed.
bool adat_input_check_rate_change(void);

// Device rate changed (called from perform_rate_change). Master mode: retune
// the RX cell period and re-sync, or park with rate_ok = false above 48 kHz.
void adat_input_on_rate_change(uint32_t freq);

AdatInputState adat_input_get_state(void);
uint32_t adat_input_get_detected_rate(void);
void adat_input_get_status(AdatInputStatusPacket *out);

#endif // PICO_RP2350

#endif // ADAT_INPUT_H
