/*
 * Output Limiter: brickwall lookahead peak limiter per output.  No-overshoot
 * proof, link groups, cross-core meeting and the silent engage protocol:
 * Documentation/Features/output_limiter_spec.md.
 */

#include <float.h>
#include <math.h>
#include <string.h>
#include "limiter.h"
#include "bulk_params.h"   // WireBulkParams offsets for change notifications
#include "notify.h"
#include "usb_audio.h"     // matrix_mixer
#include "hardware/sync.h"

volatile LimiterOutputConfig limiter_config[NUM_OUTPUT_CHANNELS] = {
    [0 ... NUM_OUTPUT_CHANNELS - 1] = {
        .enabled      = false,
        .link_group   = 0,
        .threshold_db = LIMITER_DEFAULT_THRESHOLD,
        .release_ms   = LIMITER_DEFAULT_RELEASE,
    },
};
volatile bool limiter_update_pending = false;

// Gains are Q30 on RP2040; levels are raw Q28 samples, where 24-bit full
// scale at the output is 2^29 (the Q28 ">> 6" output conversion).
#if PICO_RP2350
typedef float    lm_gain_t;
#define LM_UNITY 1.0f
#define LM_G_UNITY 1.0f
#else
typedef uint32_t lm_gain_t;
#define LM_UNITY (1u << 30)
#define LM_G_UNITY 32768             // applied gain, Q15
#define LM_FS_RAW 536870912.0f        // 2^29
#endif

typedef struct {
#if PICO_RP2350
    float    thr[NUM_OUTPUT_CHANNELS];   // linear
    float    rel[NUM_OUTPUT_CHANNELS];   // per-block release multiplier
#else
    uint32_t thr[NUM_OUTPUT_CHANNELS];   // raw level
    uint32_t thr2[NUM_OUTPUT_CHANNELS];  // thr << 1: numerator of the Q15 divide
    uint32_t rho[NUM_OUTPUT_CHANNELS];   // per-block release multiplier minus 1, Q30
#endif
    uint16_t link_mask[NUM_OUTPUT_CHANNELS];  // enabled outputs in this group; 0 = unlinked
    uint16_t enabled_mask;
} LimiterCoeffs;

typedef struct {
    lm_sample_t ring[LIMITER_DELAY];
    lm_gain_t   p;          // running peak of the input block in progress
    lm_gain_t   h_prev;     // hard target of the last completed block
    lm_gain_t   env;        // this output's own envelope (before linking)
    lm_gain_t   rel;        // release (rel / rho) cached for the drain after leaving
#if PICO_RP2350
    float       g, step;    // gain ramp over the output block being emitted
    float       g_last;     // ramp end target (applied gain at the last boundary)
    float       meter;      // deepest applied gain in the last packet
#else
    int32_t     g, step;    // Q15
    int32_t     g_last;     // Q15
    int32_t     meter;      // Q15
#endif
    bool        ring_dirty; // ring may hold non-zero samples
    bool        gain_active; // limiting, or draining back to unity after leaving
} LimiterOutputState;

// Written by Core 0 before the Core 1 dispatch; the dispatch __dmb publishes it.
typedef struct {
    const LimiterCoeffs *c;
    uint16_t part_mask;     // outputs limited this packet
    uint16_t ring_mask;     // outputs delayed this packet (superset of part_mask)
    uint16_t drain_mask;    // left the limited set, releasing their own gain to unity
    uint8_t  pos;           // stream position mod LIMITER_DELAY at packet start
    uint8_t  nb;            // block boundaries inside this packet
    bool     engaged;
    bool     xcore;         // a live link group spans both cores this packet
} LimiterPacket;

static LimiterOutputState lm_state[NUM_OUTPUT_CHANNELS];
static lm_gain_t lm_env[NUM_OUTPUT_CHANNELS][LIMITER_MAX_BOUNDS];
static LimiterPacket lm_pk;
static volatile uint8_t lm_arrived[2];

static LimiterCoeffs lm_coeff_bufs[2];
static uint8_t lm_coeff_idx = 0;
static volatile const LimiterCoeffs *current_limiter_coeffs = NULL;

static bool     lm_engaged = false;
static uint8_t  lm_pos = 0;
static uint32_t lm_silent_run = 0;
static volatile uint32_t lm_packets = 0;

_Static_assert((LIMITER_DELAY & (LIMITER_DELAY - 1)) == 0, "ring wrap uses a mask");
_Static_assert(NUM_OUTPUT_CHANNELS <= 16, "output masks are 16-bit");

static inline float lm_clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// ---------------------------------------------------------------------------
// Parameters
// ---------------------------------------------------------------------------

static uint16_t lm_wire_off(uint8_t out, uint16_t field_off) {
    return (uint16_t)(offsetof(WireBulkParams, limiter)
                      + out * sizeof(WireLimiterOutput) + field_off);
}

static void lm_set_one(uint8_t out, uint8_t index, float value) {
    volatile LimiterOutputConfig *cfg = &limiter_config[out];
    switch (index) {
    case LIMITER_PARAM_ENABLED: {
        cfg->enabled = (value != 0.0f);
        uint8_t b = cfg->enabled ? 1 : 0;
        notify_param_write(lm_wire_off(out, offsetof(WireLimiterOutput, enabled)), 1, &b);
        break;
    }
    case LIMITER_PARAM_THRESHOLD_DB: {
        float v = lm_clampf(value, LIMITER_THRESHOLD_MIN, LIMITER_THRESHOLD_MAX);
        cfg->threshold_db = v;
        notify_param_write(lm_wire_off(out, offsetof(WireLimiterOutput, threshold_db)), 4, &v);
        break;
    }
    case LIMITER_PARAM_RELEASE_MS: {
        float v = lm_clampf(value, LIMITER_RELEASE_MIN, LIMITER_RELEASE_MAX);
        cfg->release_ms = v;
        notify_param_write(lm_wire_off(out, offsetof(WireLimiterOutput, release_ms)), 4, &v);
        break;
    }
    case LIMITER_PARAM_LINK_GROUP: {
        uint8_t gr = (uint8_t)(lm_clampf(value, 0.0f, (float)LIMITER_LINK_GROUP_MAX) + 0.5f);
        cfg->link_group = gr;
        notify_param_write(lm_wire_off(out, offsetof(WireLimiterOutput, link_group)), 1, &gr);
        break;
    }
    }
}

bool limiter_set_param(uint8_t output, uint8_t index, float value) {
    if (index >= LIMITER_NUM_PARAMS) return false;
    if (output >= NUM_OUTPUT_CHANNELS && output != LIMITER_ALL_OUTPUTS) return false;
    if (value != value) return true;   // NaN: ignore, never store

    if (output == LIMITER_ALL_OUTPUTS) {
        for (uint8_t k = 0; k < NUM_OUTPUT_CHANNELS; k++) lm_set_one(k, index, value);
    } else {
        lm_set_one(output, index, value);
    }
    limiter_update_pending = true;
    return true;
}

bool limiter_get_param(uint8_t output, uint8_t index, float *value) {
    if (output >= NUM_OUTPUT_CHANNELS) return false;
    const volatile LimiterOutputConfig *cfg = &limiter_config[output];
    switch (index) {
    case LIMITER_PARAM_ENABLED:      *value = cfg->enabled ? 1.0f : 0.0f; break;
    case LIMITER_PARAM_THRESHOLD_DB: *value = cfg->threshold_db; break;
    case LIMITER_PARAM_RELEASE_MS:   *value = cfg->release_ms; break;
    case LIMITER_PARAM_LINK_GROUP:   *value = (float)cfg->link_group; break;
    default: return false;
    }
    return true;
}

uint16_t limiter_meter_centidb(uint8_t output) {
    if (output >= NUM_OUTPUT_CHANNELS) return 0;
#if PICO_RP2350
    float g = lm_state[output].meter;
#else
    float g = (float)lm_state[output].meter * (1.0f / 32768.0f);
#endif
    if (g >= 1.0f) return 0;
    if (g < 1e-6f) return 12000;       // 120 dB: display floor
    float cdb = -2000.0f * log10f(g);
    return (uint16_t)(cdb > 65535.0f ? 65535.0f : cdb + 0.5f);
}

void limiter_config_defaults(void) {
    for (int k = 0; k < NUM_OUTPUT_CHANNELS; k++) {
        limiter_config[k].enabled      = false;
        limiter_config[k].link_group   = 0;
        limiter_config[k].threshold_db = LIMITER_DEFAULT_THRESHOLD;
        limiter_config[k].release_ms   = LIMITER_DEFAULT_RELEASE;
    }
}

// ---------------------------------------------------------------------------
// Coefficients
// ---------------------------------------------------------------------------

void limiter_apply_config(float sample_rate) {
    LimiterCoeffs *c = &lm_coeff_bufs[lm_coeff_idx ^ 1];
    memset(c, 0, sizeof(*c));
    if (sample_rate < 1.0f) sample_rate = 48000.0f;

    for (int k = 0; k < NUM_OUTPUT_CHANNELS; k++) {
        volatile LimiterOutputConfig *cfg = &limiter_config[k];
        // Bulk and preset restores store raw values; sanitize them in place
        // so a later GET reports what the audio path actually uses.
        float t_db = cfg->threshold_db, r_ms = cfg->release_ms;
        if (t_db != t_db) t_db = LIMITER_DEFAULT_THRESHOLD;
        if (r_ms != r_ms) r_ms = LIMITER_DEFAULT_RELEASE;
        t_db = lm_clampf(t_db, LIMITER_THRESHOLD_MIN, LIMITER_THRESHOLD_MAX);
        r_ms = lm_clampf(r_ms, LIMITER_RELEASE_MIN, LIMITER_RELEASE_MAX);
        cfg->threshold_db = t_db;
        cfg->release_ms = r_ms;
        if (cfg->link_group > LIMITER_LINK_GROUP_MAX) cfg->link_group = LIMITER_LINK_GROUP_MAX;

        float thr = powf(10.0f, t_db / 20.0f);
        float x = (float)LIMITER_BLOCK / (r_ms * 1e-3f * sample_rate);
#if PICO_RP2350
        c->thr[k] = thr;
        c->rel[k] = expf(x);
#else
        c->thr[k]  = (uint32_t)(thr * LM_FS_RAW);
        c->thr2[k] = c->thr[k] << 1;
        c->rho[k]  = (uint32_t)(expm1f(x) * (float)LM_UNITY + 0.5f);
#endif
        if (cfg->enabled) c->enabled_mask |= (uint16_t)(1u << k);
    }
    for (int k = 0; k < NUM_OUTPUT_CHANNELS; k++) {
        uint8_t gr = limiter_config[k].link_group;
        if (!((c->enabled_mask >> k) & 1u) || gr == 0) continue;
        for (int m = 0; m < NUM_OUTPUT_CHANNELS; m++) {
            if (((c->enabled_mask >> m) & 1u) && limiter_config[m].link_group == gr)
                c->link_mask[k] |= (uint16_t)(1u << m);
        }
    }

    if (c->enabled_mask) {
        lm_coeff_idx ^= 1;
        current_limiter_coeffs = c;
    } else {
        current_limiter_coeffs = NULL;
    }
}

// ---------------------------------------------------------------------------
// Engage state
// ---------------------------------------------------------------------------

static inline __attribute__((always_inline)) void lm_reset_gain(LimiterOutputState *st) {
    st->p = 0;
    st->h_prev = LM_UNITY;
    st->env = LM_UNITY;
#if PICO_RP2350
    st->g = 1.0f; st->step = 0.0f; st->g_last = 1.0f; st->meter = 1.0f;
#else
    st->g = 32768; st->step = 0; st->g_last = 32768; st->meter = 32768;
#endif
    st->gain_active = false;
}

static inline __attribute__((always_inline)) void lm_clear_output(LimiterOutputState *st) {
    memset(st->ring, 0, sizeof(st->ring));
    st->ring_dirty = false;
    lm_reset_gain(st);
}

DSP_TIME_CRITICAL
void limiter_reset_all(void) {
    for (int k = 0; k < NUM_OUTPUT_CHANNELS; k++) lm_clear_output(&lm_state[k]);
}

bool limiter_wants_engaged(void) { return current_limiter_coeffs != NULL; }
bool limiter_switch_ready(void)   { return lm_silent_run >= LIMITER_DELAY; }
uint32_t limiter_packet_count(void) { return lm_packets; }
bool limiter_is_engaged(void)    { return lm_engaged; }
uint32_t limiter_latency_samples(void) { return lm_engaged ? LIMITER_DELAY : 0; }

void limiter_force_engage(bool on) {
    limiter_reset_all();
    lm_engaged = on;
}

// ---------------------------------------------------------------------------
// Pipeline hooks
// ---------------------------------------------------------------------------

// An output joining mid-stream has 32 unmeasured samples in its ring.  Hold
// it at the gain the ring's peak needs until block decisions take over.
static inline __attribute__((always_inline))
void lm_join(LimiterOutputState *st, const LimiterCoeffs *c, int k) {
#if PICO_RP2350
    float pk = 0.0f;
    for (int i = 0; i < LIMITER_DELAY; i++) pk = fmaxf(pk, fabsf(st->ring[i]));
    float h = c->thr[k] / fmaxf(pk, c->thr[k]);
    st->g = st->g_last = st->meter = h;
    st->step = 0.0f;
#else
    uint32_t pk = 0;
    for (int i = 0; i < LIMITER_DELAY; i++) {
        int32_t v = st->ring[i];
        uint32_t a = v < 0 ? 0u - (uint32_t)v : (uint32_t)v;
        if (a > pk) pk = a;
    }
    uint32_t h = (pk > c->thr[k]) ? (c->thr2[k] / ((pk + 16383u) >> 14)) << 15 : LM_UNITY;
    st->g = st->g_last = st->meter = (int32_t)(h >> 15);
    st->step = 0;
#endif
    st->p = pk;
    st->h_prev = h;
    st->env = h;
}

DSP_TIME_CRITICAL
void limiter_packet_begin(uint32_t n, bool silent, bool dual_core) {
    const LimiterCoeffs *c = (const LimiterCoeffs *)current_limiter_coeffs;
    bool want = (c != NULL);

    // Switch only once the rings can hold nothing but zeros (spec 2.3).
    if (want != lm_engaged && lm_silent_run >= LIMITER_DELAY) {
        limiter_reset_all();
        lm_engaged = want;
    }
    if (silent) {
        if (lm_silent_run < LIMITER_DELAY) lm_silent_run += n;
    } else {
        lm_silent_run = 0;
    }

    uint16_t processed = dual_core ? (uint16_t)((1u << (CORE1_EQ_LAST_OUTPUT + 1)) - 1u)
                                   : (uint16_t)((1u << NUM_OUTPUT_CHANNELS) - 1u);
    uint16_t ring = 0, part = 0;
    if (lm_engaged) {
        for (int k = 0; k < NUM_OUTPUT_CHANNELS; k++)
            if (matrix_mixer.outputs[k].enabled) ring |= (uint16_t)(1u << k);
        ring &= processed;
        // RAW test signals are limited too: RAW skips the crossover, which is
        // exactly when a driver most needs the protection.
        if (c) part = ring & c->enabled_mask;
    }

    // Core 1 is idle here, so touching its outputs' state is safe.  An
    // output leaving the limited set drains: its ring was measured, so it
    // keeps its own gain and releases to unity instead of stepping.
    uint16_t drain = 0;
    for (int k = 0; k < NUM_OUTPUT_CHANNELS; k++) {
        LimiterOutputState *st = &lm_state[k];
        if (!((ring >> k) & 1u)) {
            if (st->ring_dirty) lm_clear_output(st);
        } else if ((part >> k) & 1u) {
#if PICO_RP2350
            st->rel = c->rel[k];
#else
            st->rel = c->rho[k];
#endif
            if (!st->gain_active) lm_join(st, c, k);
            st->gain_active = true;
        } else if (st->gain_active) {
            drain |= (uint16_t)(1u << k);
        }
    }

    bool xcore = false;
    if (dual_core && part) {
        const uint16_t c0 = (uint16_t)((1u << CORE1_EQ_FIRST_OUTPUT) - 1u);
        const uint16_t c1 = (uint16_t)(processed & ~c0);
        for (int k = 0; k < CORE1_EQ_FIRST_OUTPUT; k++)
            if (((part & c0) >> k) & 1u && (c->link_mask[k] & part & c1)) xcore = true;
    }

    lm_pk.c = c;
    lm_pk.part_mask = part;
    lm_pk.ring_mask = ring;
    lm_pk.drain_mask = drain;
    lm_pk.pos = lm_pos;
    lm_pk.nb = (uint8_t)(((lm_pos & (LIMITER_BLOCK - 1)) + n) / LIMITER_BLOCK);
    lm_pk.engaged = lm_engaged;
    lm_pk.xcore = xcore;
    lm_arrived[0] = 0;
    lm_arrived[1] = 0;
    lm_packets++;
}

DSP_TIME_CRITICAL
void limiter_packet_end(uint32_t n) {
    lm_pos = (uint8_t)((lm_pos + n) & (LIMITER_DELAY - 1));
}

// Each core publishes its envelopes, then waits for the other's.
static inline __attribute__((always_inline)) void lm_meet(int core) {
    __dmb();
    lm_arrived[core] = 1;
    __sev();
    while (!lm_arrived[core ^ 1]) __wfe();
    __dmb();
}

// Delay only: RAW test signals and outputs whose own limiter is off.
static inline __attribute__((always_inline)) void lm_ring_only(LimiterOutputState *st, lm_sample_t *x,
                                uint32_t n, uint32_t pos) {
    uint32_t q = pos & (LIMITER_DELAY - 1);
    uint32_t i = 0;
    while (i < n) {
        uint32_t seg = LIMITER_DELAY - q;
        if (seg > n - i) seg = n - i;
        lm_sample_t *r = st->ring + q;
        lm_sample_t *xp = x + i;
        for (uint32_t k = 0; k < seg; k++) {
            lm_sample_t y = r[k];
            r[k] = xp[k];
            xp[k] = y;
        }
        i += seg;
        q = (q + seg) & (LIMITER_DELAY - 1);
    }
    st->ring_dirty = true;
}

#if PICO_RP2350

// Peak per block, then E = min(E * r, 1, h_prev, h).  h = thr / max(p, thr)
// is exactly 1.0 below threshold, so no compare is needed.
static inline __attribute__((always_inline)) void lm_detect(LimiterOutputState *st, const float *x, uint32_t n,
                             uint32_t ph, float thr, float rel, float *env_out) {
    float p = st->p, hp = st->h_prev, e = st->env;
    uint32_t i = 0, b = 0, left = LIMITER_BLOCK - ph;
    while (i < n) {
        uint32_t seg = (n - i < left) ? n - i : left;
        const float *xp = x + i;
        for (uint32_t k = 0; k < seg; k++) p = fmaxf(p, fabsf(xp[k]));
        i += seg;
        if (seg == left) {
            float h = thr / fmaxf(p, thr);
            e = fminf(fminf(e * rel, 1.0f), fminf(hp, h));
            env_out[b++] = e;
            hp = h;
            p = 0.0f;
            left = LIMITER_BLOCK;
        } else {
            left -= seg;
        }
    }
    st->p = p; st->h_prev = hp; st->env = e;
}

// A block boundary restarts the ramp at the previous target, so the output
// block about to be emitted runs G[b-1] -> G[b] exactly (spec 2.1).
static inline __attribute__((always_inline)) void lm_apply(LimiterOutputState *st, float *x, uint32_t n,
                            uint32_t pos, const float *G) {
    float g = st->g, step = st->step, gl = st->g_last, gmin = gl;
    uint32_t q = pos & (LIMITER_DELAY - 1);
    uint32_t i = 0, b = 0, left = LIMITER_BLOCK - (pos & (LIMITER_BLOCK - 1));
    while (i < n) {
        uint32_t seg = (n - i < left) ? n - i : left;
        float *r = st->ring + q;
        float *xp = x + i;
        for (uint32_t k = 0; k < seg; k++) {
            float y = r[k];
            r[k] = xp[k];
            xp[k] = y * g;
            g += step;
        }
        i += seg;
        q = (q + seg) & (LIMITER_DELAY - 1);
        if (seg == left) {
            float gn = G[b++];
            g = gl;
            step = (gn - gl) * (1.0f / LIMITER_BLOCK);
            gl = gn;
            gmin = fminf(gmin, gn);
            left = LIMITER_BLOCK;
        } else {
            left -= seg;
        }
    }
    st->g = g; st->step = step; st->g_last = gl; st->meter = gmin;
    st->ring_dirty = true;
}

#else  // RP2040

// y = x * g / 2^15 for 0 <= g <= 32768, exact floor, two 16-bit multiplies.
static inline __attribute__((always_inline)) int32_t lm_mul_q15(int32_t x, int32_t g) {
    int32_t sh = x >> 16;
    uint32_t sl = (uint32_t)x & 0xFFFFu;
    return (int32_t)((uint32_t)(sh * g) << 1) + (int32_t)((sl * (uint32_t)g) >> 15);
}

// Same as the float detect.  h is a 32/32 hardware divide giving Q15; the
// denominator is rounded up so h rounds down and never overshoots.
static inline __attribute__((always_inline)) void lm_detect(LimiterOutputState *st, const int32_t *x, uint32_t n,
                             uint32_t ph, uint32_t thr, uint32_t thr2, uint32_t rho,
                             uint32_t *env_out) {
    uint32_t p = st->p, hp = st->h_prev, e = st->env;
    uint32_t i = 0, b = 0, left = LIMITER_BLOCK - ph;
    while (i < n) {
        uint32_t seg = (n - i < left) ? n - i : left;
        const int32_t *xp = x + i;
        for (uint32_t k = 0; k < seg; k++) {
            int32_t v = xp[k];
            uint32_t s = (uint32_t)(v >> 31);
            uint32_t a = ((uint32_t)v ^ s) - s;
            if (a > p) p = a;
        }
        i += seg;
        if (seg == left) {
            uint32_t h = (p > thr) ? (thr2 / ((p + 16383u) >> 14)) << 15 : LM_UNITY;
            uint32_t er = e + (uint32_t)(((uint64_t)e * rho) >> 30);
            if (er > LM_UNITY) er = LM_UNITY;
            if (hp < er) er = hp;
            if (h < er) er = h;
            e = er;
            env_out[b++] = e;
            hp = h;
            p = 0;
            left = LIMITER_BLOCK;
        } else {
            left -= seg;
        }
    }
    st->p = p; st->h_prev = hp; st->env = e;
}

// Q15 ramp; the arithmetic shift floors the step so the ramp never rises
// above the straight line between the two targets.
static inline __attribute__((always_inline)) void lm_apply(LimiterOutputState *st, int32_t *x, uint32_t n,
                            uint32_t pos, const uint32_t *G) {
    int32_t g = st->g, step = st->step, gl = st->g_last, gmin = gl;
    uint32_t q = pos & (LIMITER_DELAY - 1);
    uint32_t i = 0, b = 0, left = LIMITER_BLOCK - (pos & (LIMITER_BLOCK - 1));
    while (i < n) {
        uint32_t seg = (n - i < left) ? n - i : left;
        int32_t *r = st->ring + q;
        int32_t *xp = x + i;
        for (uint32_t k = 0; k < seg; k++) {
            int32_t y = r[k];
            r[k] = xp[k];
            xp[k] = lm_mul_q15(y, g);
            g += step;
        }
        i += seg;
        q = (q + seg) & (LIMITER_DELAY - 1);
        if (seg == left) {
            int32_t gn = (int32_t)(G[b++] >> 15);
            g = gl;
            step = (gn - gl) >> 4;
            gl = gn;
            if (gn < gmin) gmin = gn;
            left = LIMITER_BLOCK;
        } else {
            left -= seg;
        }
    }
    st->g = g; st->step = step; st->g_last = gl; st->meter = gmin;
    st->ring_dirty = true;
}
_Static_assert(LIMITER_BLOCK == 16, "RP2040 ramp step divides by shifting 4");

#endif

DSP_TIME_CRITICAL
void limiter_process_outputs(int first, int last,
                             lm_sample_t (*buf_out)[AUDIO_BUFFER_SAMPLES],
                             uint32_t n, int core) {
    if (!lm_pk.engaged) return;
    const LimiterCoeffs *c = lm_pk.c;
    const uint16_t range = (uint16_t)(((1u << (last + 1)) - 1u) & ~((1u << first) - 1u));
    const uint16_t part = lm_pk.part_mask & range;
    const uint16_t drain = lm_pk.drain_mask & range;
    const uint32_t pos = lm_pk.pos;
    const uint32_t ph = pos & (LIMITER_BLOCK - 1);
    const uint32_t nb = lm_pk.nb;

    // A draining output measures with no threshold, so its envelope only
    // releases; c may be NULL by then, hence the cached release.
    for (int k = first; k <= last; k++) {
        LimiterOutputState *st = &lm_state[k];
#if PICO_RP2350
        if ((part >> k) & 1u)
            lm_detect(st, buf_out[k], n, ph, c->thr[k], st->rel, lm_env[k]);
        else if ((drain >> k) & 1u)
            lm_detect(st, buf_out[k], n, ph, FLT_MAX, st->rel, lm_env[k]);
#else
        if ((part >> k) & 1u)
            lm_detect(st, buf_out[k], n, ph, c->thr[k], c->thr2[k], st->rel, lm_env[k]);
        else if ((drain >> k) & 1u)
            lm_detect(st, buf_out[k], n, ph, UINT32_MAX, 0, st->rel, lm_env[k]);
#endif
    }

    // The other core must not return before both sides arrive, so meet
    // unconditionally whenever the packet asked for it.
    if (lm_pk.xcore) lm_meet(core);

    for (int k = first; k <= last; k++) {
        LimiterOutputState *st = &lm_state[k];
        if (!((lm_pk.ring_mask >> k) & 1u)) continue;
        if ((drain >> k) & 1u) {
            lm_apply(st, buf_out[k], n, pos, lm_env[k]);
            if (st->env == LM_UNITY && st->g_last == LM_G_UNITY && st->step == 0)
                lm_reset_gain(st);
            continue;
        }
        if (!((part >> k) & 1u)) {
            lm_ring_only(st, buf_out[k], n, pos);
            continue;
        }
        const lm_gain_t *G = lm_env[k];
        lm_gain_t Gl[LIMITER_MAX_BOUNDS];
        uint16_t members = (uint16_t)(c->link_mask[k] & lm_pk.part_mask & ~(1u << k));
        if (members) {
            for (uint32_t b = 0; b < nb; b++) Gl[b] = lm_env[k][b];
            for (int m = 0; m < NUM_OUTPUT_CHANNELS; m++) {
                if (!((members >> m) & 1u)) continue;
                for (uint32_t b = 0; b < nb; b++)
                    if (lm_env[m][b] < Gl[b]) Gl[b] = lm_env[m][b];
            }
            G = Gl;
        }
        lm_apply(st, buf_out[k], n, pos, G);
    }
}
