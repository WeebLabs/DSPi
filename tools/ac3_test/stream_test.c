/*
 * stream_test.c; host test of firmware/DSPi/ac3_stream.c.  Wraps an .ac3
 * file into an IEC 61937 word stream, runs it through the detector, the
 * capture, the drain and the just-in-time scheduler the way the S/PDIF poll
 * and the main-loop service do, and checks the output against a direct
 * decode delayed by exactly AC3_STREAM_DELAY frames.
 *
 * stream_test <in.ac3> [--swap] [--stall N] [--chunk N]
 *   --swap    byte-swap the payload words (sources that send 0x770B)
 *   --stall N skip the service for N polls once, mid-stream (late blocks)
 *   --chunk N frames per simulated poll (default 48)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../../firmware/DSPi/ac3_stream.h"

static int failures = 0;
#define CHECK(c, ...) do { if (!(c)) { failures++; printf("  FAIL %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

#define LEAD_WORDS 64   // 32 carrier frames before the first burst
static uint32_t word_of(uint16_t payload) { return (uint32_t)payload << 12; }

static uint8_t *read_file(const char *path, size_t *len) {
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); exit(1); }
    fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
    uint8_t *buf = calloc(1, (size_t)n + 4);
    if (fread(buf, 1, (size_t)n, f) != (size_t)n) exit(1);
    fclose(f);
    *len = (size_t)n;
    return buf;
}

// Direct decode: interleaved six channels, one frame of 1536 samples each.
static float *direct_decode(const uint8_t *buf, size_t len, int *nframes) {
    static Ac3Decoder d;
    ac3_decoder_init(&d);
    size_t cap = 64, pos = 0; int frames = 0;
    float *out = malloc(cap * AC3_FRAME_SAMPLES * AC3_CH_COUNT * sizeof(float));
    while (pos + AC3_HEADER_BYTES <= len) {
        int flags, sr, br;
        int n = ac3_decoder_frame_info(buf + pos, &flags, &sr, &br);
        if (!n) { pos++; continue; }
        if (pos + (size_t)n > len) break;
        if ((size_t)frames == cap) { cap *= 2; out = realloc(out, cap * AC3_FRAME_SAMPLES * AC3_CH_COUNT * sizeof(float)); }
        if (ac3_decoder_frame_begin(&d, buf + pos)) { pos += n; continue; }
        for (int b = 0; b < AC3_BLOCKS_PER_FRAME; b++) {
            ac3_decoder_block(&d);
            float *dst = out + ((size_t)frames * AC3_FRAME_SAMPLES + b * AC3_BLOCK_SAMPLES) * AC3_CH_COUNT;
            for (int ch = 0; ch < AC3_CH_COUNT; ch++) {
                const float *src = ac3_decoder_channel(&d, ch);
                for (int i = 0; i < AC3_BLOCK_SAMPLES; i++) dst[i * AC3_CH_COUNT + ch] = src ? src[i] : 0.0f;
            }
        }
        frames++;
        pos += n;
    }
    *nframes = frames;
    return out;
}

// Build the IEC 61937 carrier: one burst per frame, payload then zero
// stuffing to the 1536-frame period.  Returns words (2 per carrier frame).
static uint16_t *build_carrier(const uint8_t *buf, size_t len, int nframes, int swap, size_t *nwords) {
    // PCM-ish lead-in, the bursts, then two silent periods so the last
    // frame's playback window lies inside the carrier.
    size_t total = (size_t)(nframes + 2) * AC3_FRAME_SAMPLES * 2 + LEAD_WORDS;
    uint16_t *w = calloc(total, sizeof(uint16_t));
    size_t n = 0;
    for (int i = 0; i < LEAD_WORDS; i++) w[n++] = (uint16_t)(1000 + i);
    size_t pos = 0; int f = 0;
    while (f < nframes) {
        int flags, sr, br;
        int fl = ac3_decoder_frame_info(buf + pos, &flags, &sr, &br);
        if (!fl) { pos++; continue; }
        size_t start = n;
        w[n++] = IEC61937_PA; w[n++] = IEC61937_PB;
        w[n++] = IEC61937_TYPE_AC3; w[n++] = (uint16_t)(fl * 8);
        for (int i = 0; i < fl; i += 2) {
            uint16_t v = (uint16_t)((buf[pos + i] << 8) | buf[pos + i + 1]);
            if (swap) v = (uint16_t)((v >> 8) | (v << 8));
            w[n++] = v;
        }
        while (n < start + AC3_FRAME_SAMPLES * 2) w[n++] = 0;
        pos += fl; f++;
    }
    n += 2 * AC3_FRAME_SAMPLES * 2;
    *nwords = n;
    return w;
}

int main(int argc, char **argv) {
    if (argc < 2) { fprintf(stderr, "stream_test in.ac3 [--swap] [--stall N] [--chunk N]\n"); return 2; }
    int swap = 0, stall = 0, chunk = 48;
    for (int i = 2; i < argc; i++) {
        if (!strcmp(argv[i], "--swap")) swap = 1;
        else if (!strcmp(argv[i], "--stall") && i + 1 < argc) stall = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--chunk") && i + 1 < argc) chunk = atoi(argv[++i]);
    }
    size_t len; uint8_t *buf = read_file(argv[1], &len);
    int nframes; float *ref = direct_decode(buf, len, &nframes);
    size_t nwords; uint16_t *carrier = build_carrier(buf, len, nframes, swap, &nwords);
    size_t carrier_frames = nwords / 2;

    static uint8_t frames[AC3_STREAM_SLOTS][AC3_STREAM_FRAME_BYTES];
    static float ring[AC3_STREAM_RING_FRAMES][AC3_CH_COUNT];
    static Ac3Decoder dec;
    Ac3Stream st; Iec61937Detector det;
    iec61937_reset(&det);
    ac3_stream_init(&st, frames, ring, &dec);

    float *out = calloc(carrier_frames * AC3_CH_COUNT, sizeof(float));
    float gain[AC3_CH_COUNT] = {1, 1, 1, 1, 1, 1};
    size_t fpos = 0; int poll = 0, blocks = 0, stalled = 0;
    while (fpos < carrier_frames) {
        size_t n = carrier_frames - fpos; if (n > (size_t)chunk) n = chunk;
        // Poll: feed every word, then drain the batch.
        for (size_t i = 0; i < n; i++) {
            ac3_stream_feed(&st, &det, word_of(carrier[(fpos + i) * 2]), (uint32_t)(fpos + i));
            ac3_stream_feed(&st, &det, word_of(carrier[(fpos + i) * 2 + 1]), (uint32_t)(fpos + i));
        }
        float *o[AC3_CH_COUNT]; float tmp[AC3_CH_COUNT][192];
        for (int ch = 0; ch < AC3_CH_COUNT; ch++) o[ch] = tmp[ch];
        ac3_stream_drain(&st, (uint32_t)n, o, gain);
        for (size_t i = 0; i < n; i++)
            for (int ch = 0; ch < AC3_CH_COUNT; ch++) out[(fpos + i) * AC3_CH_COUNT + ch] = tmp[ch][i];
        fpos += n; poll++;
        // Service: at most two blocks per pass, unless stalled.
        if (stall && poll == 200) stalled = stall;
        if (stalled) { stalled--; continue; }
        blocks += ac3_stream_decode_due(&st, 2);
    }

    // Compare: carrier frame t (t >= lead-in + DELAY) holds direct sample
    // t - lead - DELAY.  Count mismatching samples.
    size_t lead = LEAD_WORDS / 2, mism = 0, compared = 0;
    double maxd = 0;
    // With a stall, compare only up to it: the dropped frame desynchronises
    // the decoder's dither generator from the direct decode's, so later
    // samples legitimately differ.
    int long_stall = (size_t)stall * chunk > AC3_STREAM_LEAD;   // beyond what the lead absorbs
    size_t limit = long_stall ? (size_t)200 * chunk : carrier_frames;
    for (size_t t = lead + AC3_STREAM_DELAY; t < carrier_frames && t < limit; t++) {
        size_t r = t - lead - AC3_STREAM_DELAY;
        if (r >= (size_t)nframes * AC3_FRAME_SAMPLES) break;
        for (int ch = 0; ch < AC3_CH_COUNT; ch++) {
            float a = out[t * AC3_CH_COUNT + ch], b = ref[r * AC3_CH_COUNT + ch];
            compared++;
            if (a != b) { mism++; double d = fabs(a - b); if (d > maxd) maxd = d; }
        }
    }
    // Before the first playback position the output must be silent.
    size_t pre_nonzero = 0;
    for (size_t t = 0; t < lead + AC3_STREAM_DELAY && t < carrier_frames; t++)
        for (int ch = 0; ch < AC3_CH_COUNT; ch++) if (out[t * AC3_CH_COUNT + ch] != 0.0f) pre_nonzero++;

    printf("frames=%d carrier=%zu polls=%d blocks=%d decoded=%u dropped=%u late=%u errors=%u\n",
           nframes, carrier_frames, poll, blocks, st.frames_decoded, st.frames_dropped,
           st.blocks_late, st.decode_errors);
    printf("compared=%zu mismatches=%zu maxdiff=%g pre_delay_nonzero=%zu\n", compared, mism, maxd, pre_nonzero);
    if (!long_stall) {
        CHECK(mism == 0, "output differs from the delayed direct decode");
        CHECK(st.blocks_late == 0 && st.frames_dropped == 0 && st.decode_errors == 0, "scheduler counters");
        CHECK(st.frames_decoded == (uint32_t)nframes, "decoded %u of %d frames", st.frames_decoded, nframes);
    } else {
        CHECK(st.blocks_late > 0 || st.frames_dropped > 0, "stall produced no late blocks");
        CHECK(mism == 0, "output before the stall differs");
        CHECK(st.decode_errors == 0, "decode errors after the stall");
        CHECK(st.frames_decoded + st.frames_dropped == (uint32_t)nframes,
              "did not recover: decoded %u dropped %u of %d", st.frames_decoded, st.frames_dropped, nframes);
    }
    CHECK(pre_nonzero == 0, "output before the anchored delay is not silent");
    printf(failures ? "FAIL\n" : "PASS\n");
    return failures ? 1 : 0;
}
