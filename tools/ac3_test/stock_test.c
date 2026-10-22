/*
 * stock_test.c; decodes an .ac3 file with an unmodified liba52 checkout and
 * writes the same interleaved float32 layout as ac3_test, so the port can be
 * checked bit-exact against upstream.  Requires the stock sources; see
 * run_stock.sh.  Output layout: liba52 order (LFE, then full-bandwidth
 * channels), remapped here to DSPi order L R C LFE Ls Rs.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "a52.h"

static const int8_t fbw_index[11][6] = {
    { 0,  1, -1, -1, -1, -1}, {-1, -1,  0, -1, -1, -1}, { 0,  1, -1, -1, -1, -1},
    { 0,  2,  1, -1, -1, -1}, { 0,  1, -1, -1,  2,  2}, { 0,  2,  1, -1,  3,  3},
    { 0,  1, -1, -1,  2,  3}, { 0,  2,  1, -1,  3,  4}, {-1, -1,  0, -1, -1, -1},
    {-1, -1,  0, -1, -1, -1}, { 0,  1, -1, -1, -1, -1},
};

int main(int argc, char **argv) {
    if (argc < 3) { fprintf(stderr, "stock_test in.ac3 out.f32\n"); return 2; }
    FILE *f = fopen(argv[1], "rb");
    if (!f) { perror(argv[1]); return 1; }
    fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
    uint8_t *buf = calloc(1, n + 4);
    if (fread(buf, 1, n, f) != (size_t)n) return 1;
    fclose(f);
    FILE *out = fopen(argv[2], "wb");
    if (!out) { perror(argv[2]); return 1; }

    a52_state_t *st = a52_init();
    sample_t *samples = a52_samples(st);
    long pos = 0; int frames = 0;
    while (pos + 7 <= n) {
        int flags, sr, br;
        int len = a52_syncinfo(buf + pos, &flags, &sr, &br);
        if (!len) { pos++; continue; }
        if (pos + len > n) break;
        flags = A52_3F2R | A52_LFE;
        level_t level = 1.0f;
        if (a52_frame(st, buf + pos, &flags, &level, 0.0f)) { fprintf(stderr, "frame error\n"); return 1; }
        int mode = flags & A52_CHANNEL_MASK, lfe = (flags & A52_LFE) != 0;
        for (int b = 0; b < 6; b++) {
            if (a52_block(st)) { fprintf(stderr, "block error\n"); return 1; }
            float inter[256 * 6];
            for (int ch = 0; ch < 6; ch++) {
                const float *src = NULL;
                if (ch == 3) src = lfe ? samples : NULL;
                else if (fbw_index[mode][ch] >= 0) src = samples + 256 * (fbw_index[mode][ch] + lfe);
                for (int i = 0; i < 256; i++) inter[i * 6 + ch] = src ? src[i] : 0.0f;
            }
            fwrite(inter, sizeof(inter), 1, out);
        }
        frames++;
        pos += len;
    }
    fclose(out);
    printf("stock: frames=%d\n", frames);
    return 0;
}
