#ifndef SIGGEN_H
#define SIGGEN_H

/*
 * siggen.h -- Onboard test signal generator.
 *
 * Synthesizes measurement and diagnostic signals directly into the per-output
 * mix buffers (buf_out[]) between the matrix-mix pass and the per-output
 * processing pass of process_input_block().  Selected output channels have
 * their routed audio replaced by the generated signal; unselected channels
 * play program audio untouched.  Downstream stages (crossover, PEQ, output
 * gain, master volume, delay, encode) run unchanged, so inter-slot sample
 * alignment is preserved by construction.
 *
 * RAW mode bypasses the per-channel crossover + PEQ for generator channels
 * (output trim, master volume, mute and delay still apply).
 *
 * When no input source is streaming, siggen_pump() drives the pipeline
 * itself, paced by the slot-0 consumer fill level.
 *
 * State is strictly transient: never persisted, always off at boot, stopped
 * by preset load and factory reset.
 *
 * See Documentation/Features/test_signals_spec.md for the full protocol.
 */

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

// ---------------------------------------------------------------------------
// Signal catalogue
// ---------------------------------------------------------------------------

typedef enum {
    SIGGEN_SINE        = 0,   // p1 = freq Hz
    SIGGEN_SQUARE      = 1,   // p1 = freq Hz (polyBLEP band-limited)
    SIGGEN_WHITE       = 2,   // white noise, uniform
    SIGGEN_PINK        = 3,   // pink noise (-3 dB/oct)
    SIGGEN_SWEEP_LOG   = 4,   // p1 = f1, p2 = f2; duration_ms = sweep length
    SIGGEN_SWEEP_LIN   = 5,   // p1 = f1, p2 = f2; duration_ms = sweep length
    SIGGEN_SWEEP_STEP  = 6,   // p1 = f1, p2 = f2, p3 = steps/octave, p4 = dwell ms
    SIGGEN_IMPULSE     = 7,   // p1 = period ms; single-sample unit impulses
    SIGGEN_CLICKS_ALT  = 8,   // p1 = period ms; alternating-polarity clicks
    SIGGEN_POLARITY    = 9,   // p1 = pulse width ms, p2 = period ms; positive lobe
    SIGGEN_TONE_BURST  = 10,  // p1 = freq, p2 = on cycles, p3 = off cycles, p4 = edge cycles
    SIGGEN_TONE_PAIR   = 11,  // p1 = f1, p2 = f2, p3 = amplitude ratio A1/A2
                              //   SMPTE IMD: 60/7000/4; CCIF IMD: 19000/20000/1
    SIGGEN_MULTITONE   = 12,  // p1 = tone count, p2 = f_lo, p3 = f_hi (log-spaced)
    SIGGEN_ISP         = 13,  // p1 = pattern: 0 = fs/4 @45deg (+3.01 dBTP),
                              //               1 = fs/6 @30deg (+1.25 dBTP)
    SIGGEN_CHANNEL_ID  = 14,  // channel-count blip melody; typically with WALK
    SIGGEN_TYPE_COUNT
} SiggenType;

// Config flags
#define SIGGEN_FLAG_RAW     0x01  // bypass per-channel crossover + PEQ
#define SIGGEN_FLAG_DECORR  0x02  // noise: independent generator per channel
#define SIGGEN_FLAG_WALK    0x04  // play masked channels one at a time, advancing per cycle

// Multitone oscillator budget (per-platform CPU headroom)
#if PICO_RP2350
#define SIGGEN_MULTITONE_MAX  16
#else
#define SIGGEN_MULTITONE_MAX  8
#endif

// Fade applied on start/stop/config swap of continuous signals (ms)
#define SIGGEN_FADE_MS  5

// ---------------------------------------------------------------------------
// Wire structures (packed, little-endian, floats IEEE-754 LE)
// ---------------------------------------------------------------------------

#define SIGGEN_CFG_VERSION  1

// MEAS.SIGGEN_CONFIG payload and reply.
// Timing model per type:
//   continuous (SINE/SQUARE/WHITE/PINK/TONE_PAIR/MULTITONE/ISP):
//     duration_ms = total play time, 0 = until stopped; repeat/gap unused.
//   sweeps (LOG/LIN/STEP): duration_ms = one sweep (must be > 0);
//     repeat = sweep count (0 = infinite), gap_ms = silence between sweeps.
//   patterns (IMPULSE/CLICKS_ALT/POLARITY/TONE_BURST/CHANNEL_ID):
//     repeat = pattern periods (0 = infinite), gap_ms adds extra silence
//     per period, duration_ms unused.
typedef struct __attribute__((packed)) {
    uint8_t  version;       // SIGGEN_CFG_VERSION
    uint8_t  signal_type;   // SiggenType
    uint16_t channel_mask;  // output-channel select, bit i = output i
    uint16_t invert_mask;   // polarity-inverted subset of channel_mask
    uint8_t  flags;         // SIGGEN_FLAG_*
    uint8_t  reserved0;
    float    level_db;      // peak level dBFS, -120..0
    uint32_t duration_ms;
    uint16_t repeat;
    uint16_t gap_ms;
    float    p1, p2, p3, p4;
} SiggenConfig;             // 36 bytes

// Generator run state (SiggenStatus.state)
typedef enum {
    SIGGEN_STATE_IDLE     = 0,
    SIGGEN_STATE_FADE_IN  = 1,
    SIGGEN_STATE_RUN      = 2,
    SIGGEN_STATE_GAP      = 3,
    SIGGEN_STATE_FADE_OUT = 4,
} SiggenState;

// MEAS.SIGGEN_STATUS reply
typedef struct __attribute__((packed)) {
    uint8_t  version;        // SIGGEN_CFG_VERSION
    uint8_t  state;          // SiggenState
    uint8_t  signal_type;    // active/last SiggenType
    uint8_t  active_channel; // WALK: current output channel, else 0xFF
    uint32_t elapsed_ms;     // time since start
    uint16_t cycles_done;    // completed sweeps / pattern periods
    uint8_t  stop_reason;    // SIGGEN_STOP_* of last stop
    uint8_t  reserved0;
    float    current_freq;   // instantaneous sweep frequency, 0 if n/a
} SiggenStatus;              // 16 bytes

// Stop reasons (status + notification)
#define SIGGEN_STOP_NONE       0
#define SIGGEN_STOP_HOST       1  // CONTROL stop
#define SIGGEN_STOP_COMPLETED  2  // duration/repeat exhausted
#define SIGGEN_STOP_PRESET     3  // preset load / factory reset
#define SIGGEN_STOP_RECONFIG   4  // SET_CONFIG while running (restart)

// Control actions: MEAS.SIGGEN_STOP, MEAS.SIGGEN_START, MEAS.SIGGEN_STOP_NOW
#define SIGGEN_CTL_STOP        0  // fade out, then idle
#define SIGGEN_CTL_START       1  // (re)start with the applied config
#define SIGGEN_CTL_STOP_NOW    2  // immediate hard stop, no fade

// MEAS.SIGGEN_CAPS returns SiggenCapsHeader; MEAS.SIGGEN_TYPE_DESC at address
// 0..type_count-1 returns that type's SiggenTypeDesc.
typedef struct __attribute__((packed)) {
    uint8_t  version;
    uint8_t  type_count;          // SIGGEN_TYPE_COUNT
    uint8_t  output_channels;     // NUM_OUTPUT_CHANNELS
    uint8_t  multitone_max;       // SIGGEN_MULTITONE_MAX
    uint16_t valid_channel_mask;  // (1 << NUM_OUTPUT_CHANNELS) - 1
    uint16_t reserved0;
} SiggenCapsHeader;               // 8 bytes

// Param semantic codes for SiggenParamDesc.semantic
#define SIGGEN_PARAM_UNUSED   0
#define SIGGEN_PARAM_FREQ_HZ  1
#define SIGGEN_PARAM_MS       2
#define SIGGEN_PARAM_CYCLES   3
#define SIGGEN_PARAM_COUNT    4
#define SIGGEN_PARAM_RATIO    5
#define SIGGEN_PARAM_PATTERN  6

typedef struct __attribute__((packed)) {
    uint8_t semantic;             // SIGGEN_PARAM_*
    float   min, max, def;
} SiggenParamDesc;                // 13 bytes

// Timing model codes for SiggenTypeDesc.timing_model
#define SIGGEN_TIMING_CONTINUOUS  0
#define SIGGEN_TIMING_SWEEP       1
#define SIGGEN_TIMING_PATTERN     2

typedef struct __attribute__((packed)) {
    uint8_t         id;           // SiggenType
    char            name[8];      // NUL-padded short name
    uint8_t         timing_model; // SIGGEN_TIMING_*
    SiggenParamDesc p[4];
} SiggenTypeDesc;                 // 62 bytes

// ---------------------------------------------------------------------------
// Vendor-command surface (handlers in vendor/vc_page_meas.c)
// ---------------------------------------------------------------------------

// Validate + stage a config from a wire payload.  Returns false on invalid
// payload (bad version/type, empty channel mask, zero sweep duration);
// out-of-range or NaN level/params are clamped to the caps ranges rather
// than rejected.  If running, the generator restarts (fade-out, swap,
// fade-in) via siggen_service().  Never auto-starts from idle.
bool siggen_stage_config(const void *payload, uint16_t len);

// CONTROL action (SIGGEN_CTL_*).  Returns false on unknown action or on
// START with no valid config applied.
bool siggen_control(uint8_t action);

// Fill wire responses.
void siggen_get_config(SiggenConfig *out);
void siggen_get_status(SiggenStatus *out);
const SiggenCapsHeader *siggen_caps_header(void);
const SiggenTypeDesc  *siggen_caps_type(uint8_t index);   // NULL if out of range

// ---------------------------------------------------------------------------
// Pipeline integration (audio_pipeline.c / pdm_generator.c / main.c)
// ---------------------------------------------------------------------------

// Fast pipeline gate: true while the generator contributes samples
// (FADE_IN/RUN/GAP/FADE_OUT).  Checked once per block.
extern volatile bool siggen_running;

// Output channels whose crossover + PEQ are bypassed this block (RAW mode).
// Bit i = output i.  Zero whenever the generator is idle.  Read by the
// per-output EQ gating on both cores; only written from Core 0 main-loop
// context between blocks.
extern volatile uint32_t siggen_raw_mask;

// Render into the per-output mix buffers; called from process_input_block()
// after matrix mixing, before per-output processing and the Core 1 dispatch.
// Writes only channels in (channel_mask & enabled outputs).
#if PICO_RP2350
void siggen_render(float (*bufs)[AUDIO_BUFFER_SAMPLES], uint32_t sample_count,
                   uint32_t sample_rate_hz);
#else
void siggen_render(int32_t (*bufs)[AUDIO_BUFFER_SAMPLES], uint32_t sample_count,
                   uint32_t sample_rate_hz);
#endif

// Main-loop service: applies staged configs and emits deferred notifications.
void siggen_service(void);

// Main-loop pump: when the generator is running and no input source is
// streaming, synthesizes zero-input blocks through process_input_block(),
// paced by the slot-0 consumer fill level.  Implemented in audio_pipeline.c
// (needs the input buffers and consumer-fill internals).
void siggen_pump(void);

// Hard-stop without fade and clear staged state.  For preset load / factory
// reset paths (audio is already muted there).  Safe to call when idle.
void siggen_stop_immediate(uint8_t stop_reason);

#endif // SIGGEN_H
