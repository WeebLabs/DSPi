/*
 * ac3_stream.c; see ac3_stream.h.
 */

#include "ac3_stream.h"
#include <string.h>

#if PICO_RP2350
#include "config.h"
#include "input_capture_arena.h"
#define AC3_STREAM_HOT DSP_TIME_CRITICAL
_Static_assert(INPUT_ARENA_AC3_FRAME_BYTES == AC3_STREAM_FRAME_BYTES, "arena frame size");
_Static_assert(INPUT_ARENA_AC3_RING_FRAMES == AC3_STREAM_RING_FRAMES, "arena ring size");
_Static_assert(INPUT_ARENA_AC3_CHANNELS == AC3_CH_COUNT, "arena ring channels");
#else
#define AC3_STREAM_HOT
#endif

void ac3_stream_init(Ac3Stream *s, uint8_t (*frames)[AC3_STREAM_FRAME_BYTES],
                     float (*ring)[AC3_CH_COUNT], Ac3Decoder *dec) {
    memset(s, 0, sizeof(*s));
    s->frames = frames;
    s->ring = ring;
    s->dec = dec;
    ac3_stream_reset(s);
}

void ac3_stream_reset(Ac3Stream *s) {
    memset(s->slot, 0, sizeof(s->slot));
    memset(s->ring, 0, sizeof(float) * AC3_CH_COUNT * AC3_STREAM_RING_FRAMES);
    s->cap_left = 0;
    s->cap_pos = 0;
    s->cap_slot = 0;
    s->dec_slot = -1;
    s->next_block = 0;
    s->active = false;
    // pos and pending_anchor survive: a preamble straddling the reset must
    // still anchor its frame in the coordinates the poll keeps feeding.
    ac3_decoder_init(s->dec);
}

AC3_STREAM_HOT
void ac3_stream_capture_event(Ac3Stream *s, const Iec61937Detector *d,
                              uint8_t phase_before, uint32_t frame_pos) {
    if (phase_before == 0 && d->phase == 1) {
        // Pb just matched: this frame holds the preamble.
        s->pending_anchor = frame_pos;
        return;
    }
    if (!(phase_before == 2 && d->phase == 0)) return;
    // Pd just captured.  Only AC-3 bursts with a sane length are queued.
    if (d->data_type != IEC61937_TYPE_AC3) return;
    // Pc bit 7 means the source knows the payload is bad: leave it silent.
    if (d->pc_error) { s->decode_errors++; return; }
    uint32_t bytes = d->pd / 8u;
    if (bytes < AC3_HEADER_BYTES || bytes > AC3_MAX_FRAME_BYTES) return;
    int free_slot = -1;
    for (int i = 0; i < AC3_STREAM_SLOTS; i++)
        if (!s->slot[i].ready && i != s->dec_slot) { free_slot = i; break; }
    if (free_slot < 0) { s->frames_dropped++; return; }
    s->cap_slot = (uint8_t)free_slot;
    s->cap_pos = 0;
    s->cap_left = (uint16_t)((d->pd + 15u) / 16u);
    s->slot[free_slot].anchor = s->pending_anchor;
    s->slot[free_slot].bytes = (uint16_t)bytes;
}

AC3_STREAM_HOT
void ac3_stream_capture_done(Ac3Stream *s) {
    s->slot[s->cap_slot].ready = 1;
    s->active = true;
}

AC3_STREAM_HOT
void ac3_stream_drain(Ac3Stream *s, uint32_t n, float *const out[AC3_CH_COUNT],
                      const float gain[AC3_CH_COUNT]) {
    uint32_t pos = s->pos;
    for (uint32_t i = 0; i < n; i++) {
        float *slot = s->ring[(pos + i) & AC3_STREAM_RING_MASK];
        for (int ch = 0; ch < AC3_CH_COUNT; ch++) {
            out[ch][i] = slot[ch] * gain[ch];
            slot[ch] = 0.0f;
        }
    }
    s->pos = pos + n;
}

// Some sources send the AC-3 bytes swapped within each 16-bit word; the
// sync word tells which, and the decoder wants big-endian.
static void fix_byte_order(uint8_t *buf, uint32_t bytes) {
    if (buf[0] == 0x77 && buf[1] == 0x0B) {
        for (uint32_t i = 0; i + 1 < bytes; i += 2) {
            uint8_t t = buf[i]; buf[i] = buf[i + 1]; buf[i + 1] = t;
        }
    }
}

int ac3_stream_decode_due(Ac3Stream *s, int max_blocks) {
    int done = 0;
    uint32_t pos = s->pos;

    if (s->dec_slot < 0) {
        int best = -1;
        for (int i = 0; i < AC3_STREAM_SLOTS; i++) {
            if (!s->slot[i].ready) continue;
            if (best < 0 || (int32_t)(s->slot[i].anchor - s->slot[best].anchor) < 0) best = i;
        }
        if (best < 0) return 0;
        Ac3StreamSlot *sl = &s->slot[best];
        uint32_t s0 = sl->anchor + AC3_STREAM_DELAY;
        if ((int32_t)(s0 + AC3_FRAME_SAMPLES - pos) <= 0) {
            // Whole frame already behind the drain (a long stall): drop it.
            sl->ready = 0;
            s->frames_dropped++;
            return 0;
        }
        if ((int32_t)(s0 - (pos + AC3_STREAM_LEAD)) > 0) return 0;   // not due yet
        uint8_t *buf = s->frames[best];
        fix_byte_order(buf, sl->bytes);
        // The sync check alone passes a frame mangled by a FIFO overrun, and
        // liba52 would then unpack it as full-scale noise: insist on the CRCs.
        int flags, sr, br;
        int len = ac3_decoder_frame_info(buf, &flags, &sr, &br);
        if (len == 0 || len > (int)sl->bytes || a52_crc(buf, len) ||
            ac3_decoder_frame_begin(s->dec, buf)) {
            sl->ready = 0;
            s->decode_errors++;
            return 0;
        }
        s->dec_slot = (int8_t)best;
        s->next_block = 0;
    }

    while (s->dec_slot >= 0 && done < max_blocks) {
        Ac3StreamSlot *sl = &s->slot[s->dec_slot];
        uint32_t sk = sl->anchor + AC3_STREAM_DELAY + (uint32_t)s->next_block * AC3_BLOCK_SAMPLES;
        if ((int32_t)(sk - (pos + AC3_STREAM_LEAD)) > 0) break;   // not due yet
        if (ac3_decoder_block(s->dec)) {
            s->decode_errors++;
            sl->ready = 0;
            s->dec_slot = -1;
            break;
        }
        done++;
        // The drain may already be inside this block after a stall; write
        // what is still ahead of it and count the rest as late.
        uint32_t skip = ((int32_t)(pos - sk) > 0) ? (pos - sk) : 0;
        if (skip >= AC3_BLOCK_SAMPLES) {
            s->blocks_late++;
        } else {
            for (int ch = 0; ch < AC3_CH_COUNT; ch++) {
                const float *src = ac3_decoder_channel(s->dec, ch);
                for (uint32_t i = skip; i < AC3_BLOCK_SAMPLES; i++)
                    s->ring[(sk + i) & AC3_STREAM_RING_MASK][ch] = src ? src[i] : 0.0f;
            }
            if (skip) s->blocks_late++;
        }
        if (++s->next_block == AC3_BLOCKS_PER_FRAME) {
            sl->ready = 0;
            s->dec_slot = -1;
            s->frames_decoded++;
        }
    }
    return done;
}
