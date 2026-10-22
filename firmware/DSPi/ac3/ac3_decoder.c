/*
 * ac3_decoder.c; DSPi wrapper around the ported liba52 AC-3 decoder.
 * GPLv2-or-later, as the library it wraps.
 */

#include "ac3_decoder.h"
#include <string.h>
#include <math.h>

// Dialnorm field -> gain that brings dialogue to -31 dBFS: 10^((n - 31) / 20),
// with 0 meaning -31 dB (unity), as the AC-3 standard defines.
static const float dialnorm_gain[32] = { 1.000000000e+00f, 3.162277660e-02f, 3.548133892e-02f, 3.981071706e-02f, 4.466835922e-02f, 5.011872336e-02f, 5.623413252e-02f, 6.309573445e-02f, 7.079457844e-02f, 7.943282347e-02f, 8.912509381e-02f, 1.000000000e-01f, 1.122018454e-01f, 1.258925412e-01f, 1.412537545e-01f, 1.584893192e-01f, 1.778279410e-01f, 1.995262315e-01f, 2.238721139e-01f, 2.511886432e-01f, 2.818382931e-01f, 3.162277660e-01f, 3.548133892e-01f, 3.981071706e-01f, 4.466835922e-01f, 5.011872336e-01f, 5.623413252e-01f, 6.309573445e-01f, 7.079457844e-01f, 7.943282347e-01f, 8.912509381e-01f, 1.000000000e+00f };

// Half-strength dynamic range: half the gain change in dB is the square root
// of the linear gain.  Called by a52_block() per dynrng word.
static level_t drc_half(level_t range, void *data) {
    (void)data;
    return sqrtf(range);
}

// The decoder is asked for its widest layout; a52_frame() narrows it to
// what the stream carries (a 2/0 stream comes back as A52_STEREO).
#define AC3_REQUEST_FLAGS   (A52_3F2R | A52_LFE)

void ac3_decoder_init(Ac3Decoder *d) {
    memset(d, 0, sizeof(*d));
    a52_init_static(&d->state, d->samples);
    d->drc_mode = 1;
    d->dialnorm_apply = false;
}

int ac3_decoder_frame_info(const uint8_t *hdr, int *flags,
                           int *sample_rate, int *bit_rate) {
    return a52_syncinfo(hdr, flags, sample_rate, bit_rate);
}

int ac3_decoder_frame_begin(Ac3Decoder *d, const uint8_t *frame) {
    int flags = AC3_REQUEST_FLAGS;
    int sample_rate, bit_rate;
    int len = a52_syncinfo(frame, &flags, &sample_rate, &bit_rate);
    if (len == 0) return -1;
    flags = AC3_REQUEST_FLAGS;
    level_t level = 1.0f;
    if (a52_frame(&d->state, frame, &flags, &level, 0.0f)) return -1;
    // a52_frame() re-arms dynamic range every frame, so the mode must be
    // re-applied every frame too.
    if (d->drc_mode == 0)      a52_dynrng(&d->state, NULL, NULL);
    else if (d->drc_mode == 2) a52_dynrng(&d->state, drc_half, NULL);
    // Dialnorm scales the output level; dynrng starts from level each block
    // that carries a dynrng word and otherwise keeps this value, so both get it.
    if (d->dialnorm_apply) {
        float g = dialnorm_gain[d->state.dialnorm & 31];
        d->state.level *= g;
        d->state.dynrng *= g;
    }
    d->flags = flags;
    d->sample_rate = sample_rate;
    d->bit_rate = bit_rate;
    d->frame_bytes = len;
    return 0;
}

int ac3_decoder_block(Ac3Decoder *d) {
    return a52_block(&d->state) ? -1 : 0;
}

// liba52 lays the LFE out first (when present), then the full-bandwidth
// channels in the order its output mode defines.  Index of each DSPi
// channel within the full-bandwidth set, -1 when absent.
static const int8_t fbw_index[11][AC3_CH_COUNT] = {
    //            L   R   C  LFE  Ls  Rs
    /* CHANNEL */ { 0,  1, -1, -1, -1, -1},
    /* MONO    */ {-1, -1,  0, -1, -1, -1},
    /* STEREO  */ { 0,  1, -1, -1, -1, -1},
    /* 3F      */ { 0,  2,  1, -1, -1, -1},
    /* 2F1R    */ { 0,  1, -1, -1,  2,  2},
    /* 3F1R    */ { 0,  2,  1, -1,  3,  3},
    /* 2F2R    */ { 0,  1, -1, -1,  2,  3},
    /* 3F2R    */ { 0,  2,  1, -1,  3,  4},
    /* CHANNEL1*/ {-1, -1,  0, -1, -1, -1},
    /* CHANNEL2*/ {-1, -1,  0, -1, -1, -1},
    /* DOLBY   */ { 0,  1, -1, -1, -1, -1},
};

const float *ac3_decoder_channel(const Ac3Decoder *d, int ch) {
    if (ch < 0 || ch >= AC3_CH_COUNT) return NULL;
    int mode = d->flags & A52_CHANNEL_MASK;
    bool lfe = (d->flags & A52_LFE) != 0;
    if (ch == AC3_CH_LFE) return lfe ? d->samples : NULL;
    if (mode > A52_DOLBY) return NULL;
    int idx = fbw_index[mode][ch];
    if (idx < 0) return NULL;
    return d->samples + AC3_BLOCK_SAMPLES * (idx + (lfe ? 1 : 0));
}
