/*
 * output_latency.h; selectable output latency options
 *
 * A fixed table of options, each a slot ring fill target plus the input batch
 * it needs, and the live state derived from the selected one. Design, limits
 * and measurements: Documentation/current_architecture.md "Output Latency".
 */

#ifndef OUTPUT_LATENCY_H
#define OUTPUT_LATENCY_H

#include <stdbool.h>
#include <stdint.h>

// Option IDs are the wire and flash values. 0 is never valid, so a zeroed
// store reads as unset and takes the default.
// The two low-latency options batch stream inputs at 0.33 ms and need a
// stream input; they differ only in margin (current_architecture.md).
typedef enum {
    OUTPUT_LATENCY_0P67MS = 1,
    OUTPUT_LATENCY_1MS    = 2,
    OUTPUT_LATENCY_2MS    = 3,
    OUTPUT_LATENCY_3MS    = 4,
    OUTPUT_LATENCY_4MS    = 5,
    OUTPUT_LATENCY_6MS    = 6,
    OUTPUT_LATENCY_8MS    = 7,
} OutputLatencyId;

#define OUTPUT_LATENCY_DEFAULT  OUTPUT_LATENCY_4MS

// Needs a sample-stream input (S/PDIF, I2S, ADAT). USB delivers whole 1 ms
// packets, so a target under one packet would run dry between them.
#define OUTPUT_LATENCY_FLAG_STREAM_ONLY  0x01u

typedef struct {
    uint8_t  id;          // OutputLatencyId
    uint8_t  flags;       // OUTPUT_LATENCY_FLAG_*
    uint16_t latency_us;  // slot ring fill target
    uint16_t batch_us;    // frames per pipeline call from a stream input
} OutputLatencyOption;

extern const OutputLatencyOption output_latency_options[];
extern const uint8_t output_latency_option_count;

// Live state of the effective option at the current rate, published together by
// output_latency_apply() and read on the audio hot paths.
extern volatile uint32_t output_ring_target_frames;  // slot ring fill target
extern volatile uint32_t input_batch_frames;         // a stream input waits for this many
extern volatile uint32_t output_ring_block_cap;      // no producer delivers more per call

// NULL for an unknown ID.
const OutputLatencyOption *output_latency_find(uint8_t id);

// The option a source runs for a request: a stream-only request on USB runs as
// the lowest standard option. An unknown request runs as the default.
const OutputLatencyOption *output_latency_resolve(uint8_t id, uint8_t source);

// Ring fill target, in frames, an option gives at a rate (ring ceiling applied).
uint32_t output_latency_target_frames(const OutputLatencyOption *opt, uint32_t fs);

// Stored request (an unknown ID records the default) and the option in force.
void output_latency_request(uint8_t id);
uint8_t output_latency_requested(void);
uint8_t output_latency_effective(void);

// Re-derive the live state for the request, rate and active source. Every slot
// shares the result, so call it only inside a pipeline reset bracket (or before
// audio starts): the restart then re-anchors every ring at the new target together.
void output_latency_apply(uint32_t fs, uint8_t source);

#endif // OUTPUT_LATENCY_H
