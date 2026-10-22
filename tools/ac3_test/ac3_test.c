/*
 * ac3_test.c; host harness for the firmware's ported AC-3 decoder
 * (firmware/DSPi/ac3).  Decodes an .ac3 file to interleaved float32 in DSPi
 * channel order (L R C LFE Ls Rs, zeros for channels the stream lacks),
 * prints a checksum the device bench compares against, and checks the
 * generated IMDCT tables against a recomputation.  See run.sh.
 *
 * Build: see run.sh (the decoder sources are compiled with
 * -ffp-contract=off so host and device round identically).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <time.h>
#include "../../firmware/DSPi/ac3/ac3_decoder.h"

// FNV-1a over the float bit patterns of every output sample, in DSPi order.
static uint32_t fnv1a(uint32_t h, const void *p, size_t n) {
    const uint8_t *b = p;
    for (size_t i = 0; i < n; i++) { h ^= b[i]; h *= 16777619u; }
    return h;
}

static uint8_t *read_file(const char *path, size_t *len) {
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return NULL; }
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    // 4 bytes of slack: the bit reader fetches whole words past the frame.
    uint8_t *buf = calloc(1, (size_t)n + 4);
    if (fread(buf, 1, (size_t)n, f) != (size_t)n) { perror(path); free(buf); fclose(f); return NULL; }
    fclose(f);
    *len = (size_t)n;
    return buf;
}

// Decode every sync frame; returns frames decoded.  out may be NULL.
static int decode_stream(Ac3Decoder *d, const uint8_t *buf, size_t len, FILE *out,
                         uint32_t *checksum, int max_frames, int *errors) {
    static Ac3Decoder dec_static;
    (void)d;
    Ac3Decoder *dec = &dec_static;
    ac3_decoder_init(dec);
    size_t pos = 0;
    int frames = 0;
    uint32_t h = 2166136261u;
    *errors = 0;
    while (pos + AC3_HEADER_BYTES <= len && (max_frames <= 0 || frames < max_frames)) {
        int flags, sr, br;
        int n = ac3_decoder_frame_info(buf + pos, &flags, &sr, &br);
        if (n == 0) { pos++; continue; }
        if (pos + (size_t)n > len) break;
        if (ac3_decoder_frame_begin(dec, buf + pos)) { (*errors)++; pos += n; continue; }
        for (int b = 0; b < AC3_BLOCKS_PER_FRAME; b++) {
            if (ac3_decoder_block(dec)) { (*errors)++; break; }
            float inter[AC3_BLOCK_SAMPLES * AC3_CH_COUNT];
            for (int ch = 0; ch < AC3_CH_COUNT; ch++) {
                const float *src = ac3_decoder_channel(dec, ch);
                for (int i = 0; i < AC3_BLOCK_SAMPLES; i++)
                    inter[i * AC3_CH_COUNT + ch] = src ? src[i] : 0.0f;
            }
            h = fnv1a(h, inter, sizeof(inter));
            if (out) fwrite(inter, sizeof(inter), 1, out);
        }
        frames++;
        pos += n;
    }
    *checksum = h;
    return frames;
}

static int cmd_decode(int argc, char **argv) {
    if (argc < 2) { fprintf(stderr, "decode <in.ac3> [out.f32] [--no-drc] [--frames N]\n"); return 2; }
    const char *outpath = NULL;
    int no_drc = 0, max_frames = 0;
    for (int i = 2; i < argc; i++) {
        if (!strcmp(argv[i], "--no-drc")) no_drc = 1;
        else if (!strcmp(argv[i], "--frames") && i + 1 < argc) max_frames = atoi(argv[++i]);
        else outpath = argv[i];
    }
    size_t len;
    uint8_t *buf = read_file(argv[1], &len);
    if (!buf) return 1;
    FILE *out = outpath ? fopen(outpath, "wb") : NULL;
    if (outpath && !out) { perror(outpath); return 1; }
    static Ac3Decoder d;
    ac3_decoder_init(&d);
    d.drc = !no_drc;
    uint32_t sum; int errors;
    // decode_stream re-inits; carry the drc choice through a global flag.
    extern int g_no_drc; g_no_drc = no_drc;
    int frames = decode_stream(&d, buf, len, out, &sum, max_frames, &errors);
    if (out) fclose(out);
    int flags, sr, br;
    ac3_decoder_frame_info(buf, &flags, &sr, &br);
    printf("frames=%d errors=%d sample_rate=%d bit_rate=%d mode=%d lfe=%d checksum=0x%08x\n",
           frames, errors, sr, br, flags & A52_CHANNEL_MASK, (flags & A52_LFE) ? 1 : 0, sum);
    free(buf);
    return errors ? 1 : 0;
}

int g_no_drc = 0;

// Reference recomputation of the generated tables (liba52's a52_imdct_init).
static double bessel_i0(double x) {
    double b = 1; int i = 100;
    do b = b * x / (i * i) + 1; while (--i);
    return b;
}

typedef struct { float real, imag; } cplx;
extern const float *test_tables_window(void);

static int cmd_tables(void) {
    // The tables are static in imdct.c; re-include the header here to read them.
    #define sample_t float
    typedef struct complex_s { sample_t real; sample_t imag; } complex_t;
    #include "../../firmware/DSPi/ac3/imdct_tables.h"
    #undef sample_t
    int bad = 0;
    double local[256], sum = 0;
    for (int i = 0; i < 256; i++) {
        sum += bessel_i0(i * (256 - i) * (5 * M_PI / 256) * (5 * M_PI / 256));
        local[i] = sum;
    }
    sum++;
    for (int i = 0; i < 256; i++)
        if (a52_imdct_window[i] != (float)sqrt(local[i] / sum)) bad++;
    for (int i = 0; i < 3; i++)  if (roots16[i]  != (float)cos((M_PI / 8)  * (i + 1))) bad++;
    for (int i = 0; i < 7; i++)  if (roots32[i]  != (float)cos((M_PI / 16) * (i + 1))) bad++;
    for (int i = 0; i < 15; i++) if (roots64[i]  != (float)cos((M_PI / 32) * (i + 1))) bad++;
    for (int i = 0; i < 31; i++) if (roots128[i] != (float)cos((M_PI / 64) * (i + 1))) bad++;
    for (int i = 0; i < 64; i++) {
        int k = fftorder[i] / 2 + 64;
        if (pre1[i].real != (float)cos((M_PI / 256) * (k - 0.25))) bad++;
        if (pre1[i].imag != (float)sin((M_PI / 256) * (k - 0.25))) bad++;
    }
    for (int i = 64; i < 128; i++) {
        int k = fftorder[i] / 2 + 64;
        if (pre1[i].real != (float)-cos((M_PI / 256) * (k - 0.25))) bad++;
        if (pre1[i].imag != (float)-sin((M_PI / 256) * (k - 0.25))) bad++;
    }
    for (int i = 0; i < 64; i++) {
        if (post1[i].real != (float)cos((M_PI / 256) * (i + 0.5))) bad++;
        if (post1[i].imag != (float)sin((M_PI / 256) * (i + 0.5))) bad++;
        int k = fftorder[i] / 4;
        if (pre2[i].real != (float)cos((M_PI / 128) * (k - 0.25))) bad++;
        if (pre2[i].imag != (float)sin((M_PI / 128) * (k - 0.25))) bad++;
    }
    for (int i = 0; i < 32; i++) {
        if (post2[i].real != (float)cos((M_PI / 128) * (i + 0.5))) bad++;
        if (post2[i].imag != (float)sin((M_PI / 128) * (i + 0.5))) bad++;
    }
    printf("tables: %d mismatching entries\n", bad);
    return bad ? 1 : 0;
}

static int cmd_bench(int argc, char **argv) {
    if (argc < 2) { fprintf(stderr, "bench <in.ac3> [passes]\n"); return 2; }
    int passes = argc > 2 ? atoi(argv[2]) : 20;
    size_t len;
    uint8_t *buf = read_file(argv[1], &len);
    if (!buf) return 1;
    static Ac3Decoder d;
    uint32_t sum; int errors, frames = 0;
    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    for (int p = 0; p < passes; p++) frames += decode_stream(&d, buf, len, NULL, &sum, 0, &errors);
    clock_gettime(CLOCK_MONOTONIC, &t1);
    double us = (t1.tv_sec - t0.tv_sec) * 1e6 + (t1.tv_nsec - t0.tv_nsec) / 1e3;
    printf("host: %d frames in %.0f us, %.1f us/frame\n", frames, us, us / frames);
    free(buf);
    return 0;
}

int main(int argc, char **argv) {
    if (argc < 2) { fprintf(stderr, "usage: ac3_test decode|tables|bench ...\n"); return 2; }
    if (!strcmp(argv[1], "decode")) return cmd_decode(argc - 1, argv + 1);
    if (!strcmp(argv[1], "tables")) return cmd_tables();
    if (!strcmp(argv[1], "bench")) return cmd_bench(argc - 1, argv + 1);
    fprintf(stderr, "unknown command %s\n", argv[1]);
    return 2;
}
