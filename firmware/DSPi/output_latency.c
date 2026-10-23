/*
 * output_latency.c; selectable output latency options
 *
 * See output_latency.h and Documentation/current_architecture.md "Output Latency".
 */

#include "output_latency.h"
#include "audio_input.h"
#include "config.h"
#include "pico/audio_spdif.h"
#include "pico/audio_i2s_multi.h"

#include <stddef.h>

#define LOW_BATCH_US    333u    // 16 frames at 48 kHz
#define STD_BATCH_US   1000u    // one USB packet

// Ascending latency: output_latency_resolve() falls back to the first entry
// without OUTPUT_LATENCY_FLAG_STREAM_ONLY.
const OutputLatencyOption output_latency_options[] = {
    { OUTPUT_LATENCY_0P67MS, OUTPUT_LATENCY_FLAG_STREAM_ONLY,  667u, LOW_BATCH_US },
    { OUTPUT_LATENCY_1MS,    OUTPUT_LATENCY_FLAG_STREAM_ONLY, 1000u, LOW_BATCH_US },
    { OUTPUT_LATENCY_2MS,    0,                               2000u, STD_BATCH_US },
    { OUTPUT_LATENCY_3MS,    0,                               3000u, STD_BATCH_US },
    { OUTPUT_LATENCY_4MS,    0,                               4000u, STD_BATCH_US },
    { OUTPUT_LATENCY_6MS,    0,                               6000u, STD_BATCH_US },
    { OUTPUT_LATENCY_8MS,    0,                               8000u, STD_BATCH_US },
};
const uint8_t output_latency_option_count =
    (uint8_t)(sizeof(output_latency_options) / sizeof(output_latency_options[0]));

volatile uint32_t output_ring_target_frames;
volatile uint32_t input_batch_frames;
volatile uint32_t output_ring_block_cap;

static uint8_t requested_id = OUTPUT_LATENCY_DEFAULT;
static uint8_t effective_id = OUTPUT_LATENCY_DEFAULT;

#define US_TO_FRAMES(us, fs)  ((uint32_t)(((uint64_t)(us) * (fs) + 500000u) / 1000000u))
#define BATCH_FRAMES(us, fs)  ((uint32_t)(((uint64_t)(us) * (fs) + 999999u) / 1000000u))
// A USB packet can carry one frame over nominal, and the stream readers use the
// same frame of slack to catch up after a late poll.
#define BLOCK_CAP(us, fs)     (BATCH_FRAMES(us, fs) + 1u)
// One block must still fit above the target inside the writable span, or group
// admission would refuse blocks in steady state.
#define RING_CEILING(us, fs)  (PICO_AUDIO_SPDIF_RING_FRAMES - PICO_AUDIO_SPDIF_ERASE_AHEAD - \
                               BLOCK_CAP(us, fs))

// The smallest option of each batch size binds: each target must exceed one
// block at the lowest rate, and fit the ring at the highest.
_Static_assert(US_TO_FRAMES(667u, 44100u) > BLOCK_CAP(LOW_BATCH_US, 44100u),
               "0.67 ms target must exceed one input block");
_Static_assert(US_TO_FRAMES(2000u, 44100u) > BLOCK_CAP(STD_BATCH_US, 44100u),
               "2 ms target must exceed one input block");
_Static_assert(US_TO_FRAMES(2000u, 96000u) <= RING_CEILING(STD_BATCH_US, 96000u),
               "2 ms target does not fit the slot ring at 96 kHz");
_Static_assert(BLOCK_CAP(STD_BATCH_US, 96000u) <= AUDIO_BUFFER_SAMPLES,
               "block cap exceeds the pipeline buffers");
_Static_assert(sizeof(output_latency_options) / sizeof(output_latency_options[0]) *
               sizeof(OutputLatencyOptionWire) <= 64,
               "option table reply exceeds one EP0 packet (vendor resp_buf)");
_Static_assert(PICO_AUDIO_I2S_RING_FRAMES - PICO_AUDIO_I2S_ERASE_AHEAD ==
               PICO_AUDIO_SPDIF_RING_FRAMES - PICO_AUDIO_SPDIF_ERASE_AHEAD,
               "the ceiling assumes one writable span for both output types");

const OutputLatencyOption *output_latency_find(uint8_t id) {
    for (uint8_t i = 0; i < output_latency_option_count; i++)
        if (output_latency_options[i].id == id) return &output_latency_options[i];
    return NULL;
}

const OutputLatencyOption *output_latency_resolve(uint8_t id, uint8_t source) {
    const OutputLatencyOption *opt = output_latency_find(id);
    if (!opt) opt = output_latency_find(OUTPUT_LATENCY_DEFAULT);
    if ((opt->flags & OUTPUT_LATENCY_FLAG_STREAM_ONLY) && source == INPUT_SOURCE_USB) {
        for (uint8_t i = 0; i < output_latency_option_count; i++)
            if (!(output_latency_options[i].flags & OUTPUT_LATENCY_FLAG_STREAM_ONLY))
                return &output_latency_options[i];
    }
    return opt;
}

uint32_t output_latency_target_frames(const OutputLatencyOption *opt, uint32_t fs) {
    uint32_t want = US_TO_FRAMES(opt->latency_us, fs);
    uint32_t ceiling = RING_CEILING(opt->batch_us, fs);
    return want < ceiling ? want : ceiling;
}

void output_latency_request(uint8_t id) {
    requested_id = output_latency_find(id) ? id : OUTPUT_LATENCY_DEFAULT;
}

uint8_t output_latency_requested(void) { return requested_id; }
uint8_t output_latency_effective(void) { return effective_id; }

void output_latency_apply(uint32_t fs, uint8_t source) {
    if (fs == 0) fs = 48000u;
    const OutputLatencyOption *opt = output_latency_resolve(requested_id, source);
    effective_id = opt->id;
    input_batch_frames = BATCH_FRAMES(opt->batch_us, fs);
    output_ring_block_cap = BLOCK_CAP(opt->batch_us, fs);
    output_ring_target_frames = output_latency_target_frames(opt, fs);
}
