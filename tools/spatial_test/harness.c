/* Host harness: compiles the real spatial.c (float, or Q28 with the firmware or an exact multiply)
 * into a shared library driven from Python through ctypes. */
#include <math.h>
#include <stdlib.h>
#include "spatial.c"

HMatrix matrix_mixer;
volatile uint32_t siggen_raw_mask;

#if PICO_RP2350
static float h_buf[NUM_OUTPUT_CHANNELS][AUDIO_BUFFER_SAMPLES];
#define TO_SP(x)   ((sp_t)(x))
#define FROM_SP(x) ((double)(x))
#else
static sp_t h_buf[NUM_OUTPUT_CHANNELS][AUDIO_BUFFER_SAMPLES];
#define TO_SP(x)   ((sp_t)llrint((x) * 268435456.0))
#define FROM_SP(x) ((double)(x) / 268435456.0)
#endif

int h_is_float(void) { return PICO_RP2350; }
int h_pairs(void) { return NUM_SPDIF_INSTANCES; }

void h_init(void) {
    for (int k = 0; k < NUM_OUTPUT_CHANNELS; k++) matrix_mixer.outputs[k].enabled = true;
    siggen_raw_mask = 0;
    memset(spatial_pair_state, 0, sizeof(spatial_pair_state));
    spatial_config_defaults(&spatial_config);
}

/* f: width, bass, shelf, span, atten, mid, trim. */
void h_config(int enabled, int mode, int mask, const float *f, float fs) {
    spatial_config.enabled = enabled != 0;
    spatial_config.mode = (uint8_t)mode;
    spatial_config.pair_mask = (uint8_t)mask;
    spatial_config.width_pct = f[0]; spatial_config.bass_width_pct = f[1]; spatial_config.shelf_hz = f[2];
    spatial_config.span_deg = f[3]; spatial_config.atten_db = f[4]; spatial_config.mid_pct = f[5];
    spatial_config.trim_db = f[6];
    spatial_apply_config(fs);
}

void h_enable_output(int out, int en) { matrix_mixer.outputs[out].enabled = en != 0; }
void h_raw(uint32_t mask) { siggen_raw_mask = mask; }

/* One packet on pair `pair` (outputs 2p, 2p+1); other outputs pass untouched. */
void h_packet(int pair, double *L, double *R, int n) {
    for (int i = 0; i < n; i++) { h_buf[2 * pair][i] = TO_SP(L[i]); h_buf[2 * pair + 1][i] = TO_SP(R[i]); }
    spatial_process_pairs((const SpatialCoeffs *)current_spatial_coeffs, spatial_config.pair_mask,
                          pair, pair, h_buf, (uint32_t)n);
    for (int i = 0; i < n; i++) { L[i] = FROM_SP(h_buf[2 * pair][i]); R[i] = FROM_SP(h_buf[2 * pair + 1][i]); }
}

/* Pairs first..last with an explicit mask; buf holds 2 * pairs channels of n samples each. */
void h_process(int first, int last, int mask, double *buf, int n) {
    for (int k = 2 * first; k <= 2 * last + 1; k++)
        for (int i = 0; i < n; i++) h_buf[k][i] = TO_SP(buf[k * n + i]);
    spatial_process_pairs((const SpatialCoeffs *)current_spatial_coeffs, (uint32_t)mask, first, last,
                          h_buf, (uint32_t)n);
    for (int k = 2 * first; k <= 2 * last + 1; k++)
        for (int i = 0; i < n; i++) buf[k * n + i] = FROM_SP(h_buf[k][i]);
}

/* Coefficients as doubles: g, gm, a5, ah, d (samples), fade_len, w1, dwb, aw, t. */
void h_coeffs(double *o) {
    const SpatialCoeffs *c = (const SpatialCoeffs *)current_spatial_coeffs;
#if PICO_RP2350
    o[4] = c->d;
#else
    o[4] = c->d / 65536.0;
#endif
    o[0] = FROM_SP(c->g); o[1] = FROM_SP(c->gm); o[2] = FROM_SP(c->a5); o[3] = FROM_SP(c->ah);
    o[5] = c->fade_len; o[6] = FROM_SP(c->w1); o[7] = FROM_SP(c->dwb); o[8] = FROM_SP(c->aw); o[9] = FROM_SP(c->t);
}

/* Codec: decode a packet, re-encode; returns the mask and mode after decoding. */
void h_codec(const uint8_t *in, uint8_t *out) {
    SpatialConfigPacket p;
    memcpy(&p, in, sizeof(p));
    spatial_packet_to_config(&p);
    spatial_config_to_packet((SpatialConfigPacket *)out);
}

/* Notify offsets for every field, against the documented wire offsets (commands.md). Returns failures. */
int h_notify_check(void) {
    h_ntf_n = 0;
    spatial_notify_config();
    const uint16_t want[] = { 6252, 6253, 6254, 6256, 6260, 6264, 6268, 6272, 6276, 6280 };
    int bad = h_ntf_n != (int)(sizeof(want) / sizeof(want[0]));
    for (int i = 0; i < h_ntf_n && !bad; i++) bad += h_ntf_off[i] != want[i];
    return bad;
}
