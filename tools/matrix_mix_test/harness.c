/* Host check: matrix_mix_kernel() against a frozen sequential reference, bit for bit. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "matrix_mix.h"

#define LEN 192
#define PAD 8
static float in[MATRIX_MIX_MAX_SRC][LEN];

// Frozen copy of the pre-kernel PASS 4 loop: first source writes, the rest accumulate.
static void ref_mix(float *dst, const float *const *src, const float *gain, int n, uint32_t count) {
    for (uint32_t i = 0; i < count; i++) dst[i] = src[0][i] * gain[0];
    for (int k = 1; k < n; k++)
        for (uint32_t i = 0; i < count; i++) dst[i] = dst[i] + src[k][i] * gain[k];
}

static uint32_t rng = 12345;
static float frand(void) { rng = rng * 1664525u + 1013904223u; return (float)(int32_t)rng / 2147483648.0f; }

int main(void) {
    int fails = 0, cases = 0;
    for (int r = 0; r < MATRIX_MIX_MAX_SRC; r++)
        for (int i = 0; i < LEN; i++) in[r][i] = frand();
    for (int n = 1; n <= MATRIX_MIX_MAX_SRC; n++) {
        for (uint32_t count = 0; count <= LEN; count++) {
            for (int trial = 0; trial < 3; trial++) {
                const float *src[MATRIX_MIX_MAX_SRC]; float gain[MATRIX_MIX_MAX_SRC];
                int perm[MATRIX_MIX_MAX_SRC];
                for (int k = 0; k < MATRIX_MIX_MAX_SRC; k++) perm[k] = k;
                for (int k = MATRIX_MIX_MAX_SRC - 1; k > 0; k--) {
                    rng = rng * 1664525u + 1013904223u; int j = (int)((rng >> 8) % (uint32_t)(k + 1));
                    int t = perm[k]; perm[k] = perm[j]; perm[j] = t;
                }
                for (int k = 0; k < n; k++) { src[k] = in[perm[k]]; gain[k] = frand() * 4.0f; }
                float a[LEN + PAD], b[LEN + PAD];
                for (int i = 0; i < LEN + PAD; i++) { a[i] = 1234.5f; b[i] = 1234.5f; }
                ref_mix(a, src, gain, n, count);
                matrix_mix_kernel(b, src, gain, n, count);
                cases++;
                if (memcmp(a, b, sizeof a) != 0) {
                    if (fails++ < 10) {
                        for (int i = 0; i < LEN + PAD; i++)
                            if (memcmp(&a[i], &b[i], 4)) { printf("FAIL n=%d count=%u i=%d ref=%.9g new=%.9g\n", n, count, i, a[i], b[i]); break; }
                    }
                }
                for (int i = (int)count; i < LEN + PAD; i++)
                    if (b[i] != 1234.5f) { if (fails++ < 10) printf("FAIL n=%d count=%u wrote past count at %d\n", n, count, i); break; }
            }
        }
    }
    printf("%s: %d cases, %d failures\n", fails ? "FAIL" : "PASS", cases, fails);
    return fails != 0;
}
