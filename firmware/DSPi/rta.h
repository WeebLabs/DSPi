#pragma once

/*
 * rta.h -- Onboard spectrum analyser (RTA / FFT).
 *
 * One FFT engine, pointed at any set of channels on the input side (after
 * the per-input PEQ) or the output side (after gain and delay, exactly what
 * the slots transmit).  The audio path only copies the current channel's
 * samples into a capture buffer and continuously updates the selected bass
 * banks; the transform runs from the main loop in
 * bounded steps.  FFT frames are visited in rotation; bass CPU scales with selected channels.
 *
 * Transient: off at boot, never persisted, stops itself when nobody reads.
 * See Documentation/Features/spectrum_analyser_spec.md for the protocol.
 */

#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "rta_fft.h"
#include "rta_bass.h"
#include "notify.h"

// ---------------------------------------------------------------------------
// Protocol constants
// ---------------------------------------------------------------------------

#define RTA_CFG_VERSION       3
#define RTA_IDLE_TIMEOUT_MS   5000

#define RTA_TAP_INPUT   0     // after per-input PEQ, before the matrix
#define RTA_TAP_OUTPUT  1     // after gain + delay, before encode
#define RTA_TAP_NONE    0xFF

#define RTA_FLAG_MANUAL 0x01  // CONTROL owns run state: no auto-start, no auto-off

#define RTA_STATE_IDLE          0
#define RTA_STATE_CAPTURING     1
#define RTA_STATE_TRANSFORMING  2

#define RTA_CTL_STOP       0
#define RTA_CTL_START      1
#define RTA_CTL_RESET_AVG  2

#define RTA_CH_NONE        0xFF

// Channels tracked at once: the larger of the two tap widths.
#if NUM_OUTPUT_CHANNELS > NUM_INPUT_CHANNELS
#define RTA_MAX_TRACKED  NUM_OUTPUT_CHANNELS
#else
#define RTA_MAX_TRACKED  NUM_INPUT_CHANNELS
#endif

#define RTA_BIN_FRAME_MAX  (16 + RTA_MAX_POINTS / 2 + 1)

// ---------------------------------------------------------------------------
// Wire structures (packed, little-endian)
// ---------------------------------------------------------------------------

typedef struct __attribute__((packed)) {
    uint8_t  version;          // RTA_CFG_VERSION
    uint8_t  tap;              // RTA_TAP_*
    uint16_t channel_mask;     // bit i = channel i at that tap; 0 = STALL
    uint8_t  fft_order;        // RTA_ORDER_MIN..RTA_ORDER_MAX
    uint8_t  reserved0;
    uint16_t avg_ms;           // EMA ms; 0 disables extra averaging (bass retains detector smoothing)
    uint8_t  peak_decay_db_s;  // 0 = peak hold off
    uint8_t  flags;            // RTA_FLAG_*
    uint16_t reserved;
} RtaConfig;
_Static_assert(sizeof(RtaConfig) == 12, "RtaConfig wire size");

typedef struct __attribute__((packed)) {
    uint8_t  version;
    uint8_t  input_channels;
    uint8_t  output_channels;
    uint8_t  fft_order_min;
    uint8_t  fft_order_max;
    uint8_t  fft_order_default;
    uint8_t  bass_bands;       // continuous bands, starting at index 0
    uint8_t  max_bands;
    uint8_t  level_zero;       // RTA_LEVEL_ZERO_DBFS
    uint8_t  dynamic_range_db;
    uint16_t idle_timeout_ms;
    uint16_t max_bin_frame;
    uint16_t bass_dynamic_range_db; // conservative usable bass range, both formats
} RtaCaps;
_Static_assert(sizeof(RtaCaps) == 16, "RtaCaps wire size");

typedef struct __attribute__((packed)) {
    uint8_t  version;
    uint8_t  channel;
    uint8_t  seq;              // per-channel frame counter
    uint8_t  n_bands;
    uint16_t age_ms;           // since last frame; 0xFFFF = never
    uint16_t reserved;
    uint8_t  avg[RTA_MAX_BANDS];
    uint8_t  peak[RTA_MAX_BANDS];
} RtaBandFrame;
_Static_assert(sizeof(RtaBandFrame) == 82, "RtaBandFrame wire size");

typedef struct __attribute__((packed)) {
    uint8_t  version;
    uint8_t  channel;
    uint8_t  seq;              // repeated as the frame's final byte
    uint8_t  fft_order;
    uint32_t sample_rate_hz;
    uint16_t n_bins;           // frame is 16 + n_bins + 1 bytes
    uint16_t reserved[3];
} RtaBinFrameHeader;
_Static_assert(sizeof(RtaBinFrameHeader) == 16, "RtaBinFrameHeader wire size");

typedef struct __attribute__((packed)) {
    uint8_t  version;
    uint8_t  state;            // RTA_STATE_*
    uint8_t  tap;
    uint8_t  channel;          // being captured / transformed; RTA_CH_NONE when idle
    uint8_t  reserved0;
    uint8_t  live_count;
    uint16_t live_mask;
    uint16_t frames_per_s;
    uint16_t busy_us_per_s;    // main-loop transform time, EMA
    uint16_t last_frame_us;    // wall time of the last complete transform
    uint16_t idle_ms;          // since the last data read; 0xFFFF = never
    uint32_t sample_rate_hz;
    uint8_t  first_band;       // 0 with supported bass bank, 0xFF for unsupported rate
    uint8_t  reserved1;
    uint16_t bass_busy_us_per_s; // summed tap time on both cores; saturates at 65535
} RtaStatus;
_Static_assert(sizeof(RtaStatus) == 24, "RtaStatus wire size");

// ---------------------------------------------------------------------------
// Vendor-command surface (handlers in vendor_commands.c)
// ---------------------------------------------------------------------------

// Validate and stage a config.  False = STALL (bad version/length/tap/order,
// or an empty mask at that tap).  Applied by rta_service(), which announces
// it with NOTIFY_EVT_RTA_CONFIG tagged with src.
bool rta_set_config(const void *payload, uint16_t len, ParamSource src);
void rta_get_config(RtaConfig *out);
void rta_get_caps(RtaCaps *out);
// Band centre table chunk (wValue 1..): up to 32 uint16 Hz values; returns
// the byte count written, 0 for an out-of-range chunk.
uint16_t rta_get_caps_centres(uint8_t chunk, uint8_t *out);
// Latest band frame for a channel at the applied tap.  False = bad channel.
bool rta_get_band_frame(uint8_t channel, RtaBandFrame *out);
// Published bin frame (stable storage; validate seq == final byte).
const uint8_t *rta_bin_frame(uint16_t *len);
void rta_get_status(RtaStatus *out);
bool rta_control(uint8_t action);
// Any band or bin read: keepalive for auto-off, auto-start unless MANUAL.
void rta_note_read(void);
// Channels selected AND live at the applied tap (for GET_BANDS_ALL).
uint16_t rta_live_mask(void);

// ---------------------------------------------------------------------------
// Pipeline integration
// ---------------------------------------------------------------------------

#if PICO_RP2350
typedef float   rta_pipe_t;
#else
typedef int32_t rta_pipe_t;    // Q28
#endif

// Per-packet tap descriptor.  Written by Core 0 in rta_packet_begin(),
// before the Core 1 dispatch barrier, so both cores use one view.  Only the
// core that owns the tapped row copies samples this packet; `produced` is
// the one field it writes back, read by Core 0 after the join.  A row that
// is not tapped leaves it at 0, which is how the engine notices a channel
// went dead mid-frame.
typedef struct {
    uint8_t  tap;              // RTA_TAP_* or RTA_TAP_NONE
    uint8_t  row;
    uint16_t wr;
    uint16_t take;             // samples to copy this packet, 0 = none
    volatile uint16_t produced;
    uint8_t bass_tap;
    uint16_t bass_mask;
    volatile uint32_t bass_busy_us[RTA_MAX_TRACKED];
} RtaPacketView;

extern RtaPacketView rta_view;
extern rta_sample_t  rta_buf[RTA_MAX_POINTS];

// Core 0: top of process_input_block() and after the Core 1 join.
void rta_packet_begin(uint32_t sample_count);
void rta_packet_end(uint32_t sample_count);

// Owning core, inside the existing meter loops.  One RAM-resident function
// so the call sites do not each carry the Q28 conversion loop.
void rta_tap(uint8_t tap, uint8_t row, const rta_pipe_t *buf, uint32_t n);

// Main loop: apply staged config, fill liveness, auto-off. The transform of a
// filled frame runs as a background job (bg_job.h), started by rta_packet_end().
void rta_service(void);

// Discard the frame in flight and restart capture.  Rate change, input
// source switch, preset load, factory reset.
void rta_restart(void);

void rta_init(void);
