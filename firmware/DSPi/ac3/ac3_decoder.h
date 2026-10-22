/*
 * ac3_decoder.h; DSPi wrapper around the ported liba52 AC-3 decoder.
 *
 * Static-state float decoder: one Ac3Decoder holds everything (about
 * 17 KB), nothing is allocated.  Output is six DSPi-ordered channels
 * (L R C LFE Ls Rs) at 256 samples per block, six blocks per frame, full
 * scale = 1.0.  See Documentation/Features/ac3_dts_decoding_plan.md.
 */

#ifndef AC3_DECODER_H
#define AC3_DECODER_H

#include <stdint.h>
#include <stdbool.h>
#include "a52.h"
#include "a52_internal.h"

#define AC3_MAX_FRAME_BYTES    3840
#define AC3_HEADER_BYTES       7      // bytes a52_syncinfo() needs
#define AC3_BLOCKS_PER_FRAME   6
#define AC3_BLOCK_SAMPLES      256
#define AC3_FRAME_SAMPLES      (AC3_BLOCKS_PER_FRAME * AC3_BLOCK_SAMPLES)

// DSPi channel order for the decoded set (matches the plan's inputs 0-5).
enum {
    AC3_CH_L = 0, AC3_CH_R, AC3_CH_C, AC3_CH_LFE, AC3_CH_LS, AC3_CH_RS,
    AC3_CH_COUNT
};

typedef struct {
    a52_state_t state;
    // Six 256-sample output channels followed by six overlap histories.
    float samples[AC3_BLOCK_SAMPLES * 12] __attribute__((aligned(16)));
    int   flags;          // a52 output flags of the current frame
    int   sample_rate;
    int   bit_rate;
    int   frame_bytes;
    uint8_t drc_mode;     // 0 off, 1 as sent (default), 2 half the gain change in dB
    bool  dialnorm_apply; // scale by 10^((dialnorm - 31) / 20); default off here,
                          // the firmware sets it from ac3_config every frame
} Ac3Decoder;

void ac3_decoder_init(Ac3Decoder *d);

// Parse a sync frame header (AC3_HEADER_BYTES).  Returns the frame length in
// bytes, or 0 if the bytes are not a valid AC-3 sync frame.
int ac3_decoder_frame_info(const uint8_t *hdr, int *flags,
                           int *sample_rate, int *bit_rate);

// Start decoding a complete frame.  The buffer must stay valid until the
// last block is decoded and must have 4 bytes of readable slack after the
// frame (the bit reader fetches whole 32-bit words).  Returns 0 on success.
int ac3_decoder_frame_begin(Ac3Decoder *d, const uint8_t *frame);

// Decode the next block of the current frame.  Returns 0 on success.
int ac3_decoder_block(Ac3Decoder *d);

// Decoded samples of one DSPi channel for the last block, or NULL when the
// current frame does not carry that channel (a 2/0 stream has no C, LFE,
// Ls or Rs).  A single surround channel is returned for both Ls and Rs.
const float *ac3_decoder_channel(const Ac3Decoder *d, int ch);

#endif // AC3_DECODER_H
