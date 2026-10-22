/*
 * ac3_stream.h; AC-3 over IEC 61937: deframer, decoded ring and the
 * just-in-time block scheduler.  SDK-free so tools/ac3_test/stream_test.c
 * runs it on the host against a direct decode.
 *
 * Positions are carrier frame counts since reset.  A burst anchored at
 * carrier frame A plays from A + AC3_STREAM_DELAY; block k of that frame is
 * decoded when its start is within AC3_STREAM_LEAD frames of the drain.
 * See Documentation/Features/ac3_dts_decoding_plan.md section 6.
 */

#ifndef AC3_STREAM_H
#define AC3_STREAM_H

#include <stdint.h>
#include <stdbool.h>
#include "iec61937.h"
#include "ac3/ac3_decoder.h"

#define AC3_STREAM_RING_FRAMES  512u
#define AC3_STREAM_RING_MASK    (AC3_STREAM_RING_FRAMES - 1u)
#define AC3_STREAM_FRAME_BYTES  (AC3_MAX_FRAME_BYTES + 4)   // bit reader reads whole words
#define AC3_STREAM_SLOTS        2
#define AC3_STREAM_DELAY        1536u   // anchor to playback, one burst period
#define AC3_STREAM_LEAD         256u    // decode when due within this; ring holds 512

typedef struct {
    uint32_t anchor;   // carrier frame index of the burst's Pa
    uint16_t bytes;    // payload bytes captured (Pd / 8)
    uint8_t  ready;    // captured, waiting to decode
} Ac3StreamSlot;

typedef struct {
    uint8_t (*frames)[AC3_STREAM_FRAME_BYTES];
    float   (*ring)[AC3_CH_COUNT];
    Ac3Decoder *dec;
    Ac3StreamSlot slot[AC3_STREAM_SLOTS];
    // Capture (hot path): payload words still to store, 0 = not capturing.
    uint16_t cap_left;
    uint16_t cap_pos;
    uint8_t  cap_slot;
    uint32_t pending_anchor;
    // Decode (main-loop service).
    int8_t   dec_slot;      // slot being decoded, -1 = none
    uint8_t  next_block;
    bool     active;        // an AC-3 frame has been queued since reset
    uint32_t pos;           // carrier frames drained so far (ring read position)
    // Counters, saturating where a host shows them.
    uint32_t frames_decoded;
    uint32_t frames_dropped;   // no free slot, or stale before decode
    uint32_t blocks_late;      // decoded after the drain had passed them
    uint32_t decode_errors;    // frame or block the decoder rejected
} Ac3Stream;

void ac3_stream_init(Ac3Stream *s, uint8_t (*frames)[AC3_STREAM_FRAME_BYTES],
                     float (*ring)[AC3_CH_COUNT], Ac3Decoder *dec);
// Drop queued frames, zero the ring, restart positions and the decoder.
void ac3_stream_reset(Ac3Stream *s);

// Cold halves of the feed: a detector phase change (preamble, Pc, Pd) and
// the end of a payload capture.  RAM-placed in the firmware.
void ac3_stream_capture_event(Ac3Stream *s, const Iec61937Detector *d,
                              uint8_t phase_before, uint32_t frame_pos);
void ac3_stream_capture_done(Ac3Stream *s);

// Feed one receiver word through the detector and the capture.  frame_pos
// is the carrier frame index of this word.  Returns the detector's hold.
static inline bool ac3_stream_feed(Ac3Stream *s, Iec61937Detector *d,
                                   uint32_t word, uint32_t frame_pos) {
    uint8_t before = d->phase;
    bool hold = iec61937_feed(d, word);
    if (s->cap_left) {
        // Payload words carry the first bitstream byte in the high half.
        uint16_t w = (uint16_t)(word >> 12);
        uint8_t *dst = s->frames[s->cap_slot] + s->cap_pos;
        dst[0] = (uint8_t)(w >> 8);
        dst[1] = (uint8_t)w;
        s->cap_pos += 2;
        if (--s->cap_left == 0) ac3_stream_capture_done(s);
    } else if (__builtin_expect(before != d->phase, 0)) {
        ac3_stream_capture_event(s, d, before, frame_pos);
    }
    return hold;
}

// Copy n frames from the ring at the read position into six channel
// buffers (DSPi order), scaled by gain, zeroing the ring behind the read.
void ac3_stream_drain(Ac3Stream *s, uint32_t n, float *const out[AC3_CH_COUNT],
                      const float gain[AC3_CH_COUNT]);
// Advance the read position without output (decode inactive).
static inline void ac3_stream_skip(Ac3Stream *s, uint32_t n) { s->pos += n; }

// Decode up to max_blocks blocks whose playback start is within
// AC3_STREAM_LEAD of the read position, writing them into the ring.
// Returns blocks decoded.  Main-loop service, flash-resident.
int ac3_stream_decode_due(Ac3Stream *s, int max_blocks);

#endif // AC3_STREAM_H
