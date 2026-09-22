#ifndef LIMITER_H
#define LIMITER_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

// Per-output brickwall lookahead peak limiter.  Timing proof, link groups and
// the silent engage protocol: Documentation/Features/output_limiter_spec.md.

// Parameter indices (REQ_LIMITER wValue low byte; high byte = output).
enum {
    LIMITER_PARAM_ENABLED = 0,
    LIMITER_PARAM_THRESHOLD_DB,
    LIMITER_PARAM_RELEASE_MS,
    LIMITER_PARAM_LINK_GROUP,
    LIMITER_NUM_PARAMS
};
#define LIMITER_GET_METER       0x80   // read-only: NUM_OUTPUT_CHANNELS x uint16 GR, 0.01 dB
#define LIMITER_GET_STATUS      0x81   // read-only: engaged, lookahead, block, num_outputs
#define LIMITER_ALL_OUTPUTS     0xFF   // SET only: apply to every output

// The delay must be exactly two blocks for the no-overshoot guarantee, and
// a power of two so ring segments never straddle the wrap (spec 2.1).
#define LIMITER_BLOCK           16
#define LIMITER_DELAY           (2 * LIMITER_BLOCK)
#define LIMITER_MAX_BOUNDS      (AUDIO_BUFFER_SAMPLES / LIMITER_BLOCK)

#define LIMITER_THRESHOLD_MIN  -30.0f  // RP2040 divide keeps 0.1 % precision down to here
#define LIMITER_THRESHOLD_MAX    0.0f
#define LIMITER_RELEASE_MIN     10.0f
#define LIMITER_RELEASE_MAX   1000.0f
#define LIMITER_LINK_GROUP_MAX     4

#define LIMITER_DEFAULT_THRESHOLD -1.0f
#define LIMITER_DEFAULT_RELEASE  100.0f

typedef struct {
    bool     enabled;
    uint8_t  link_group;      // 0 = unlinked
    float    threshold_db;
    float    release_ms;
} LimiterOutputConfig;

// Live configuration + main-loop recompute flag.  SETs go through
// limiter_set_param(); the audio path only reads published coefficients.
extern volatile LimiterOutputConfig limiter_config[NUM_OUTPUT_CHANNELS];
extern volatile bool limiter_update_pending;

#if PICO_RP2350
typedef float   lm_sample_t;
#else
typedef int32_t lm_sample_t;
#endif

bool limiter_set_param(uint8_t output, uint8_t index, float value);
bool limiter_get_param(uint8_t output, uint8_t index, float *value);

// Gain reduction applied in the last packet, 0.01 dB units (0 = none).
uint16_t limiter_meter_centidb(uint8_t output);

// Restore the defaults (all off) without publishing; caller raises pending.
void limiter_config_defaults(void);

// Recompute and publish coefficients (main loop).  Publishes NULL when no
// output is enabled, which is what asks for the delay to be released.
void limiter_apply_config(float sample_rate);

// Engage state: "wants" follows the published config, "engaged" is whether
// the lookahead delay is in the signal path right now.
bool limiter_wants_engaged(void);
bool limiter_is_engaged(void);
uint32_t limiter_latency_samples(void);

// The pipeline will switch on its next packet without any fade: the last
// LIMITER_DELAY samples were silent.  packet_count lets the main loop tell a
// stalled producer from a slow fade.
bool limiter_switch_ready(void);
uint32_t limiter_packet_count(void);

// Main thread, only while no block is being produced: switch the delay
// directly.  With audio flowing the pipeline switches it under a fade.
void limiter_force_engage(bool on);

// Clear every delay ring and limiter state (main thread, Core 1 idle).
void limiter_reset_all(void);

// Pipeline hooks.  packet_begin runs on Core 0 before the Core 1 dispatch;
// `silent` = the composite output gain is zero for the whole packet.
// process_outputs runs on each core for the outputs it owns; `core` picks
// its side of the per-packet meeting when a link group spans both cores.
void limiter_packet_begin(uint32_t n, bool silent, bool dual_core);
void limiter_process_outputs(int first, int last,
                             lm_sample_t (*buf_out)[AUDIO_BUFFER_SAMPLES],
                             uint32_t n, int core);
void limiter_packet_end(uint32_t n);

#endif // LIMITER_H
