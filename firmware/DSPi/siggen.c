/*
 * siggen.c -- Onboard test signal generator.
 *
 * Architecture: a per-block segment planner (state machine: fade-in, run,
 * gap, fade-out, cycle/repeat/walk sequencing) drives per-type synth kernels.
 * The planner advances all timing exactly once per block and emits segments
 * [offset, length, gain ramp, active mask]; kernels then fill each segment.
 * This keeps sequencing platform-independent; only the sample kernels fork
 * between RP2350 float and RP2040 Q28 fixed point.
 *
 * All entry points run in Core 0 main-loop context (vendor dispatch, render
 * inside process_input_block, service).  The only cross-core traffic is
 * Core 1 reading siggen_raw_mask, written between blocks.
 *
 * Rate-dependent constants are precomputed at apply time for 44.1/48/96 kHz
 * so the render path does no float math on RP2040 and no libm calls anywhere.
 */

#include "siggen.h"
#include "usb_audio.h"
#include "notify.h"
#include <math.h>
#include <string.h>

extern MatrixMixer matrix_mixer;

// ---------------------------------------------------------------------------
// Platform sample/gain primitives
// ---------------------------------------------------------------------------

#if PICO_RP2350
typedef float samp_t;
typedef float gain_t;
#define GAIN_ONE 1.0f
static inline samp_t samp_scale(samp_t s, gain_t g) { return s * g; }
#else
typedef int32_t samp_t;
typedef int32_t gain_t;                    // Q30
#define GAIN_ONE (1 << 30)
static inline int32_t mul_q30(int32_t a, int32_t b) {
    return (int32_t)(((int64_t)a * b) >> 30);
}
static inline samp_t samp_scale(samp_t s, gain_t g) { return mul_q30(s, g); }
#endif

// ---------------------------------------------------------------------------
// Sine kernel: phase in uint32 turns (1 turn = 2^32), odd 7th-order
// polynomial on the folded quarter wave.  THD ~ -139 dB (validated offline).
// ---------------------------------------------------------------------------

#if PICO_RP2350
DSP_TIME_CRITICAL
static float osc_sin(uint32_t phase) {
    float x = (float)(int32_t)phase * (1.0f / 4294967296.0f);   // -0.5..0.5 turn
    if (x > 0.25f)       x = 0.5f - x;
    else if (x < -0.25f) x = -0.5f - x;
    float z = x * x;
    return x * (6.2831695157f + z * (-41.3379844969f
              + z * (81.3720013363f + z * -71.3162390433f)));
}
#else
// Q30 multiply via 16x16 decomposition, dropping the low x low term (error
// under 4 LSB Q30, validated offline).  Single-cycle multiplies on the M0+;
// roughly 5x cheaper than the __aeabi_lmul path, which matters for the
// multitone oscillator bank.  Operands must satisfy |a*b| < 2^61 as usual;
// callers here keep |a| <= 2^30 and |b| < 2^31.
DSP_TIME_CRITICAL
static int32_t mul_q30_fast(int32_t a, int32_t b) {
    int32_t  ah = a >> 16;
    uint32_t al = a & 0xFFFF;
    int32_t  bh = b >> 16;
    uint32_t bl = b & 0xFFFF;
    return ((ah * bh) << 2)
         + ((int32_t)(ah * bl) >> 14)
         + ((int32_t)(al * bh) >> 14);
}

// Q30 result (+-1.0 = +-2^30); u = folded phase as Q30 quarter-turn fraction.
DSP_TIME_CRITICAL
static int32_t osc_sin(uint32_t phase) {
    int32_t p = (int32_t)phase;
    if (p > (1 << 30))        p = (int32_t)((1u << 31) - (uint32_t)p);
    else if (p < -(1 << 30))  p = (int32_t)(-(int64_t)(1u << 31) - p);
    int32_t z = mul_q30_fast(p, p);
    int32_t acc = -4673781;                          // d7 Q30
    acc = 85324728   + mul_q30_fast(z, acc);         // d5
    acc = -693536295 + mul_q30_fast(z, acc);         // d3
    acc = 1686625474 + mul_q30_fast(z, acc);         // d1
    return mul_q30_fast(p, acc);
}
#endif

// ---------------------------------------------------------------------------
// Configuration + derived state
// ---------------------------------------------------------------------------

#define SIGGEN_NUM_RATES 3
static const uint32_t siggen_rates[SIGGEN_NUM_RATES] = { 44100, 48000, 96000 };

// Timing model per type (indexed by SiggenType)
static const uint8_t type_timing[SIGGEN_TYPE_COUNT] = {
    [SIGGEN_SINE]       = SIGGEN_TIMING_CONTINUOUS,
    [SIGGEN_SQUARE]     = SIGGEN_TIMING_CONTINUOUS,
    [SIGGEN_WHITE]      = SIGGEN_TIMING_CONTINUOUS,
    [SIGGEN_PINK]       = SIGGEN_TIMING_CONTINUOUS,
    [SIGGEN_SWEEP_LOG]  = SIGGEN_TIMING_SWEEP,
    [SIGGEN_SWEEP_LIN]  = SIGGEN_TIMING_SWEEP,
    [SIGGEN_SWEEP_STEP] = SIGGEN_TIMING_SWEEP,
    [SIGGEN_IMPULSE]    = SIGGEN_TIMING_PATTERN,
    [SIGGEN_CLICKS_ALT] = SIGGEN_TIMING_PATTERN,
    [SIGGEN_POLARITY]   = SIGGEN_TIMING_PATTERN,
    [SIGGEN_TONE_BURST] = SIGGEN_TIMING_PATTERN,
    [SIGGEN_TONE_PAIR]  = SIGGEN_TIMING_CONTINUOUS,
    [SIGGEN_MULTITONE]  = SIGGEN_TIMING_CONTINUOUS,
    [SIGGEN_ISP]        = SIGGEN_TIMING_CONTINUOUS,
    [SIGGEN_CHANNEL_ID] = SIGGEN_TIMING_PATTERN,
};

// Rate-dependent derived parameters, one row per supported rate
typedef struct {
    uint32_t osc_inc[SIGGEN_MULTITONE_MAX]; // tone increments (turns/sample Q32)
    uint64_t sweep_inc0;        // sweep start increment, turns/sample Q48
    uint64_t sweep_dinc;        // linear sweep: increment delta per sample, Q48
    int32_t  sweep_eps_q31;     // log sweep: relative growth per sample, Q31
    uint32_t step_dwell;        // stepped sweep: samples per step
    uint32_t step_ratio_q30;    // stepped sweep: per-step frequency ratio, Q30
    uint32_t cycle_samples;     // one sweep / pattern period / walk dwell (0 = none)
    uint32_t gap_samples;       // inter-cycle silence
    uint32_t duration_samples;  // continuous total (0 = infinite)
    uint32_t fade_samples;      // start/stop fade
    uint32_t attack_samples;    // windowed-cycle edge ramps
    uint32_t release_samples;
    uint32_t burst_on, burst_edge;   // tone burst: on-time and edge samples
    uint32_t pulse_samples;          // polarity lobe width
    uint32_t id_blip, id_blip_gap, id_tail;  // channel-ID timing
    // envelope phase per sample, precomputed reciprocals so the kernels
    // avoid a per-sample 64-bit division
    uint32_t pulse_env_step;         // half turn across the polarity lobe
    uint32_t burst_env_step;         // quarter turn across a burst edge
    uint32_t id_env_step;            // half turn across a blip
} SiggenDerived;

static SiggenConfig  cfg;                    // applied config
static SiggenConfig  cfg_next;               // staged swap while running
static bool          cfg_valid   = false;
static bool          swap_pending = false;
static SiggenDerived drv_tab[SIGGEN_NUM_RATES];
static uint32_t      phase_init[SIGGEN_MULTITONE_MAX];  // multitone Schroeder phases
static uint8_t       mt_count;               // active multitone oscillators

#if PICO_RP2350
static float   amp;            // peak level, linear
static float   amp_tone;       // per-tone level (multitone)
static float   amp_pair[2];    // tone-pair levels
#else
static int32_t amp;            // peak level, full scale SAMPLE_FULL_SCALE_Q28
static int32_t amp_tone;
static int32_t amp_pair[2];
#endif

// ---------------------------------------------------------------------------
// Runtime state
// ---------------------------------------------------------------------------

volatile bool     siggen_running  = false;
volatile uint32_t siggen_raw_mask = 0;

// Engine states are IDLE/RUN/GAP only; the start/stop fade is an overlay
// ramp (fade_dir/fade_level) on top of them, so it correctly covers the
// beginning of the first cycle and the tail of the last one.  Status reports
// FADE_IN/FADE_OUT derived from fade_dir for the wire protocol.
static uint8_t  eng_state = SIGGEN_STATE_IDLE;
static uint8_t  rate_idx  = 1;               // index into siggen_rates
static uint32_t seg_remaining;               // samples left in current segment
static uint32_t cycle_len;                   // current RUN cycle length (0 = unbounded)
static uint32_t cycle_pos;                   // position within RUN cycle
static int8_t   fade_dir;                    // +1 rising, -1 falling, 0 steady
static uint32_t fade_level, fade_len;        // overlay ramp position 0..fade_len
static uint32_t duration_remaining;          // continuous total (0 = none/inf tracked off)
static bool     duration_limited;
static uint64_t elapsed_samples;
static uint32_t cycles_done;
static uint32_t cycles_target;               // 0 = infinite
static uint8_t  walk_pos;                    // bit index of current walk channel
static bool     walk_on;
static uint8_t  stop_reason = SIGGEN_STOP_NONE;
static volatile bool notify_pending = false;
static uint8_t  notify_state, notify_reason;

// Per-type synth state
static uint32_t osc_phase[SIGGEN_MULTITONE_MAX];
static uint64_t sweep_inc;                   // current sweep increment Q48
static uint64_t sweep_phase;                 // sweep phase accumulator Q48 turns
static uint32_t step_pos;                    // stepped sweep: samples into dwell
static int32_t  click_sign;                  // alternating clicks: +1 / -1
static uint32_t rng_state[NUM_OUTPUT_CHANNELS];
#if PICO_RP2350
static float    pink_s[NUM_OUTPUT_CHANNELS][3];
#else
static int32_t  pink_s[NUM_OUTPUT_CHANNELS][3];   // Q24
#endif
static uint8_t  isp_idx;
// Channel-ID pitch increments, precomputed per rate and output channel so a
// walk advance needs no float math in the render path
static uint32_t id_inc[SIGGEN_NUM_RATES][NUM_OUTPUT_CHANNELS];

// ISP patterns: sample-peak-normalized sequences with known true-peak overs
static const int8_t isp_pat0[4] = { 1, 1, -1, -1 };       // fs/4 @45deg, +3.01 dBTP
static const int8_t isp_pat1[6] = { 1, 1, 0, -1, -1, 0 }; // fs/6 @30deg, +1.25 dBTP

// Channel-ID pitches: C major pentatonic from C5; channels 5+ an octave up
static const float id_pitch[5] = { 523.25f, 587.33f, 659.25f, 783.99f, 880.00f };

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------

static inline const SiggenDerived *drv(void) { return &drv_tab[rate_idx]; }

static inline uint32_t rng_next(uint32_t *s) {
    uint32_t x = *s;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    return *s = x;
}

static inline uint8_t mask_lowest_bit(uint32_t m) {
    for (uint8_t i = 0; i < 32; i++) if (m & (1u << i)) return i;
    return 0;
}

static inline uint8_t mask_popcount(uint32_t m) {
    uint8_t n = 0;
    while (m) { n += m & 1u; m >>= 1; }
    return n;
}

// Next set bit at or above `from`, wrapping
static inline uint8_t mask_next_bit(uint32_t m, uint8_t from) {
    for (uint8_t i = 0; i < 32; i++) {
        uint8_t b = (uint8_t)((from + i) % 32);
        if (m & (1u << b)) return b;
    }
    return 0;
}

// ---------------------------------------------------------------------------
// Synth kernels.  Each fills dst[0..n) with the signal at full configured
// level; the planner applies fades/windows afterwards.  seg_base = cycle_pos
// at dst[0] for kernels that need intra-cycle position.
// ---------------------------------------------------------------------------

DSP_TIME_CRITICAL
static void synth_sine(samp_t *dst, uint32_t n) {
    uint32_t ph = osc_phase[0], inc = drv()->osc_inc[0];
    for (uint32_t i = 0; i < n; i++) {
        ph += inc;
#if PICO_RP2350
        dst[i] = osc_sin(ph) * amp;
#else
        dst[i] = (int32_t)(((int64_t)osc_sin(ph) * amp) >> 30);
#endif
    }
    osc_phase[0] = ph;
}

// polyBLEP edge correction for the square, one-sided around each transition
#if PICO_RP2350
DSP_TIME_CRITICAL
static float blep(uint32_t ph, uint32_t inc) {
    if (ph < inc) {                       // just after upward edge
        float t = (float)ph / (float)inc;
        return t + t - t * t - 1.0f;
    }
    if (ph > (uint32_t)-(int32_t)inc) {   // just before upward edge
        float t = ((float)ph - 4294967296.0f) / (float)inc;
        return t * t + t + t + 1.0f;
    }
    return 0.0f;
}
#else
DSP_TIME_CRITICAL
static int32_t blep(uint32_t ph, uint32_t inc) {   // Q30
    if (ph < inc) {
        int32_t t = (int32_t)(((uint64_t)ph << 30) / inc);
        return t + t - mul_q30(t, t) - GAIN_ONE;
    }
    if (ph > (uint32_t)-(int32_t)inc) {
        int32_t t = -(int32_t)(((uint64_t)(0u - ph) << 30) / inc);
        return mul_q30(t, t) + t + t + GAIN_ONE;
    }
    return 0;
}
#endif

DSP_TIME_CRITICAL
static void synth_square(samp_t *dst, uint32_t n) {
    uint32_t ph = osc_phase[0], inc = drv()->osc_inc[0];
    for (uint32_t i = 0; i < n; i++) {
        ph += inc;
#if PICO_RP2350
        float v = (ph < 0x80000000u) ? 1.0f : -1.0f;
        v += blep(ph, inc) - blep(ph + 0x80000000u, inc);
        if (v > 1.0f) v = 1.0f; else if (v < -1.0f) v = -1.0f;
        dst[i] = v * amp;
#else
        int32_t v = (ph < 0x80000000u) ? GAIN_ONE : -GAIN_ONE;
        int64_t acc = (int64_t)v + blep(ph, inc) - blep(ph + 0x80000000u, inc);
        if (acc > GAIN_ONE) acc = GAIN_ONE; else if (acc < -GAIN_ONE) acc = -GAIN_ONE;
        dst[i] = (int32_t)(((int64_t)(int32_t)acc * amp) >> 30);
#endif
    }
    osc_phase[0] = ph;
}

DSP_TIME_CRITICAL
static void synth_white(samp_t *dst, uint32_t n, uint8_t ch) {
    uint32_t *rs = &rng_state[ch];
    for (uint32_t i = 0; i < n; i++) {
        int32_t r = (int32_t)rng_next(rs);
#if PICO_RP2350
        dst[i] = (float)r * (1.0f / 2147483648.0f) * amp;
#else
        dst[i] = (int32_t)(((int64_t)r * amp) >> 31);
#endif
    }
}

// Paul Kellet economy pink filter; output normalized by 1/8 (peak ~7.1 on
// uniform white input, validated offline).
DSP_TIME_CRITICAL
static void synth_pink(samp_t *dst, uint32_t n, uint8_t ch) {
    uint32_t *rs = &rng_state[ch];
#if PICO_RP2350
    float b0 = pink_s[ch][0], b1 = pink_s[ch][1], b2 = pink_s[ch][2];
    for (uint32_t i = 0; i < n; i++) {
        float w = (float)(int32_t)rng_next(rs) * (1.0f / 2147483648.0f);
        b0 = 0.99765f * b0 + w * 0.0990460f;
        b1 = 0.96300f * b1 + w * 0.2965164f;
        b2 = 0.57000f * b2 + w * 1.0526913f;
        dst[i] = (b0 + b1 + b2 + w * 0.1848f) * 0.125f * amp;
    }
    pink_s[ch][0] = b0; pink_s[ch][1] = b1; pink_s[ch][2] = b2;
#else
    // States Q24; white input Q24; poles/gains Q30 (validated offline)
    int32_t b0 = pink_s[ch][0], b1 = pink_s[ch][1], b2 = pink_s[ch][2];
    for (uint32_t i = 0; i < n; i++) {
        int32_t w = (int32_t)rng_next(rs) >> 7;     // Q24
        b0 = (int32_t)(((int64_t)1071218531 * b0 + (int64_t)106349833  * w) >> 30);
        b1 = (int32_t)(((int64_t)1034013377 * b1 + (int64_t)318382060  * w) >> 30);
        b2 = (int32_t)(((int64_t)612032840  * b2 + (int64_t)1130318677 * w) >> 30);
        int64_t sum = (int64_t)b0 + b1 + b2 + (((int64_t)198427489 * w) >> 30);
        dst[i] = (int32_t)((sum * amp) >> 27);      // Q24 -> Q28 with /8 normalize
    }
    pink_s[ch][0] = b0; pink_s[ch][1] = b1; pink_s[ch][2] = b2;
#endif
}

// Sweeps: phase/increment in Q48 turns; per-sample increment update by mode.
DSP_TIME_CRITICAL
static void synth_sweep(samp_t *dst, uint32_t n, uint8_t type) {
    const SiggenDerived *d = drv();
    uint64_t ph = sweep_phase, inc = sweep_inc;
    for (uint32_t i = 0; i < n; i++) {
        ph += inc;
#if PICO_RP2350
        dst[i] = osc_sin((uint32_t)(ph >> 16)) * amp;
#else
        dst[i] = (int32_t)(((int64_t)osc_sin((uint32_t)(ph >> 16)) * amp) >> 30);
#endif
        if (type == SIGGEN_SWEEP_LOG) {
            inc += (uint64_t)(((int64_t)(inc >> 16) * d->sweep_eps_q31) >> 15);
        } else if (type == SIGGEN_SWEEP_LIN) {
            inc += d->sweep_dinc;
        } else {                                    // stepped
            if (++step_pos >= d->step_dwell) {
                step_pos = 0;
                inc = (uint64_t)(((inc >> 16) * d->step_ratio_q30) >> 14);
            }
        }
    }
    sweep_phase = ph; sweep_inc = inc;
}

// Impulse / alternating clicks: one sample at cycle start
DSP_TIME_CRITICAL
static void synth_click(samp_t *dst, uint32_t n, uint32_t seg_base, bool alternate) {
    memset(dst, 0, n * sizeof(samp_t));
    if (seg_base == 0 && n > 0) {
        samp_t v = amp;
        if (alternate && click_sign < 0) v = -v;
        dst[0] = v;
    }
}

// Polarity pulse: single positive half-sine lobe at cycle start
DSP_TIME_CRITICAL
static void synth_polarity(samp_t *dst, uint32_t n, uint32_t seg_base) {
    const SiggenDerived *d = drv();
    uint32_t w = d->pulse_samples;
    uint32_t estep = d->pulse_env_step;
    for (uint32_t i = 0; i < n; i++) {
        uint32_t pos = seg_base + i;
        if (pos < w) {
            // phase 0..0.5 turn across the lobe
            uint32_t ph = pos * estep;
#if PICO_RP2350
            dst[i] = osc_sin(ph) * amp;
#else
            dst[i] = (int32_t)(((int64_t)osc_sin(ph) * amp) >> 30);
#endif
        } else {
            dst[i] = 0;
        }
    }
}

// Tone burst: sine with raised-cosine edges inside the on-window
DSP_TIME_CRITICAL
static void synth_burst(samp_t *dst, uint32_t n, uint32_t seg_base) {
    const SiggenDerived *d = drv();
    uint32_t on = d->burst_on, edge = d->burst_edge;
    uint32_t ph = osc_phase[0], inc = drv()->osc_inc[0];
    for (uint32_t i = 0; i < n; i++) {
        uint32_t pos = seg_base + i;
        if (pos >= on) { dst[i] = 0; continue; }
        ph += inc;
        samp_t s;
#if PICO_RP2350
        s = osc_sin(ph) * amp;
#else
        s = (int32_t)(((int64_t)osc_sin(ph) * amp) >> 30);
#endif
        if (edge > 0 && (pos < edge || pos >= on - edge)) {
            // env = sin^2 of a quarter-turn ramp: raised cosine
            uint32_t ep = (pos < edge) ? pos : (on - 1 - pos);
            uint32_t eph = ep * d->burst_env_step;
#if PICO_RP2350
            float e = osc_sin(eph);
            s *= e * e;
#else
            int32_t e = osc_sin(eph);
            s = mul_q30(s, mul_q30(e, e));
#endif
        }
        dst[i] = s;
    }
    osc_phase[0] = ph;
}

DSP_TIME_CRITICAL
static void synth_tone_pair(samp_t *dst, uint32_t n) {
    const SiggenDerived *d = drv();
    uint32_t p0 = osc_phase[0], p1 = osc_phase[1];
    uint32_t i0 = d->osc_inc[0], i1 = d->osc_inc[1];
    for (uint32_t i = 0; i < n; i++) {
        p0 += i0; p1 += i1;
#if PICO_RP2350
        dst[i] = osc_sin(p0) * amp_pair[0] + osc_sin(p1) * amp_pair[1];
#else
        int64_t acc = (int64_t)osc_sin(p0) * amp_pair[0]
                    + (int64_t)osc_sin(p1) * amp_pair[1];
        dst[i] = (int32_t)(acc >> 30);
#endif
    }
    osc_phase[0] = p0; osc_phase[1] = p1;
}

DSP_TIME_CRITICAL
static void synth_multitone(samp_t *dst, uint32_t n) {
    const SiggenDerived *d = drv();
    for (uint32_t i = 0; i < n; i++) {
#if PICO_RP2350
        float acc = 0.0f;
        for (uint8_t k = 0; k < mt_count; k++) {
            osc_phase[k] += d->osc_inc[k];
            acc += osc_sin(osc_phase[k]);
        }
        dst[i] = acc * amp_tone;
#else
        int64_t acc = 0;
        for (uint8_t k = 0; k < mt_count; k++) {
            osc_phase[k] += d->osc_inc[k];
            acc += osc_sin(osc_phase[k]);
        }
        dst[i] = (int32_t)((acc * amp_tone) >> 30);
#endif
    }
}

DSP_TIME_CRITICAL
static void synth_isp(samp_t *dst, uint32_t n) {
    const int8_t *pat = (cfg.p1 >= 1.0f) ? isp_pat1 : isp_pat0;
    uint8_t len = (cfg.p1 >= 1.0f) ? 6 : 4;
    uint8_t idx = isp_idx;
    for (uint32_t i = 0; i < n; i++) {
        int8_t v = pat[idx];
        if (++idx >= len) idx = 0;
#if PICO_RP2350
        dst[i] = (float)v * amp;
#else
        dst[i] = v * amp;
#endif
    }
    isp_idx = idx;
}

// Channel-ID blips: (channel index + 1) enveloped sine blips at the channel's
// pentatonic pitch, then a tail pause.  Walks the mask one channel per cycle.
DSP_TIME_CRITICAL
static void synth_channel_id(samp_t *dst, uint32_t n, uint32_t seg_base) {
    const SiggenDerived *d = drv();
    uint32_t blip = d->id_blip, unit = d->id_blip + d->id_blip_gap;
    uint32_t nblips = (uint32_t)walk_pos + 1;
    uint32_t active_span = nblips * unit;
    uint32_t ph = osc_phase[0], inc = id_inc[rate_idx][walk_pos];
    for (uint32_t i = 0; i < n; i++) {
        uint32_t pos = seg_base + i;
        uint32_t rel = pos % unit;
        if (pos >= active_span || rel >= blip) { dst[i] = 0; continue; }
        ph += inc;
        // full-hump envelope: sin(0..0.5 turn) across the blip
        uint32_t eph = rel * d->id_env_step;
        samp_t s;
#if PICO_RP2350
        float e = osc_sin(eph);
        s = osc_sin(ph) * amp * e * e;
#else
        int32_t e = osc_sin(eph);
        s = (int32_t)(((int64_t)osc_sin(ph) * amp) >> 30);
        s = mul_q30(s, mul_q30(e, e));
#endif
        dst[i] = s;
    }
    osc_phase[0] = ph;
}

// ---------------------------------------------------------------------------
// Cycle bookkeeping
// ---------------------------------------------------------------------------

// Current RUN cycle length; CHANNEL_ID depends on the walk channel
DSP_TIME_CRITICAL
static uint32_t current_cycle_samples(void) {
    const SiggenDerived *d = drv();
    if (cfg.signal_type == SIGGEN_CHANNEL_ID) {
        uint32_t unit = d->id_blip + d->id_blip_gap;
        return ((uint32_t)walk_pos + 1) * unit - d->id_blip_gap + d->id_tail;
    }
    return d->cycle_samples;
}

// Reset per-cycle synth state (phases, counters) for a deterministic cycle
DSP_TIME_CRITICAL
static void synth_reset_cycle(void) {
    const SiggenDerived *d = drv();
    switch (cfg.signal_type) {
    case SIGGEN_SWEEP_LOG:
    case SIGGEN_SWEEP_LIN:
    case SIGGEN_SWEEP_STEP:
        sweep_phase = 0; sweep_inc = d->sweep_inc0; step_pos = 0;
        break;
    case SIGGEN_CLICKS_ALT:
        click_sign = -click_sign;
        break;
    default:
        break;
    }
    osc_phase[0] = 0; osc_phase[1] = 0;
    if (cfg.signal_type == SIGGEN_MULTITONE)
        memcpy(osc_phase, phase_init, sizeof(uint32_t) * mt_count);
    isp_idx = 0;
}

// ---------------------------------------------------------------------------
// Planner: advance the state machine across one block, emitting segments
// ---------------------------------------------------------------------------

typedef struct {
    uint16_t off, len;
    uint32_t mask;          // channels carrying signal this segment
    gain_t   g0, g_step;    // linear gain ramp across the segment
    bool     silent;        // gap: write zeros instead of synth
    uint32_t cycle_base;    // cycle_pos at segment start (for position kernels)
} SegPlan;

#define SIGGEN_MAX_SEGS 16

static uint32_t eff_mask_cached;    // set per block in siggen_render

// Gain of the intra-cycle window (sweeps and walked continuous) at pos
DSP_TIME_CRITICAL
static gain_t window_gain_at(uint32_t pos) {
    const SiggenDerived *d = drv();
    bool windowed = (type_timing[cfg.signal_type] == SIGGEN_TIMING_SWEEP) ||
                    (type_timing[cfg.signal_type] == SIGGEN_TIMING_CONTINUOUS && walk_on);
    if (!windowed || cycle_len == 0) return GAIN_ONE;
    if (pos < d->attack_samples) {
#if PICO_RP2350
        return (float)pos / (float)d->attack_samples;
#else
        return (int32_t)(((uint64_t)pos << 30) / d->attack_samples);
#endif
    }
    if (cycle_len > d->release_samples && pos >= cycle_len - d->release_samples) {
        uint32_t rem = cycle_len - pos;
#if PICO_RP2350
        return (float)rem / (float)d->release_samples;
#else
        return (int32_t)(((uint64_t)rem << 30) / d->release_samples);
#endif
    }
    return GAIN_ONE;
}

// Gain of the start/stop overlay fade at ramp position `level`
DSP_TIME_CRITICAL
static gain_t fade_gain_of(uint32_t level) {
    if (level >= fade_len) return GAIN_ONE;
#if PICO_RP2350
    return (float)level / (float)fade_len;
#else
    return (int32_t)(((uint64_t)level << 30) / fade_len);
#endif
}

// Samples until the next gain-ramp breakpoint within the current segment
DSP_TIME_CRITICAL
static uint32_t next_boundary(uint32_t max_len) {
    uint32_t lim = max_len;
    if (seg_remaining > 0 && seg_remaining < lim) lim = seg_remaining;
    if (fade_dir > 0 && fade_len > fade_level) {
        uint32_t to_top = fade_len - fade_level;
        if (to_top < lim) lim = to_top;
    } else if (fade_dir < 0 && fade_level > 0) {
        if (fade_level < lim) lim = fade_level;
    }
    if (eng_state == SIGGEN_STATE_RUN) {
        // duration-limited continuous: breakpoint where the tail fade starts
        if (duration_limited && fade_dir == 0 && duration_remaining > fade_len) {
            uint32_t to_fade = duration_remaining - fade_len;
            if (to_fade < lim) lim = to_fade;
        }
        if (cycle_len > 0) {
            const SiggenDerived *d = drv();
            if (cycle_pos < d->attack_samples) {
                uint32_t to_end = d->attack_samples - cycle_pos;
                if (to_end < lim) lim = to_end;
            } else if (cycle_len > d->release_samples) {
                uint32_t rel_start = cycle_len - d->release_samples;
                if (cycle_pos < rel_start) {
                    uint32_t to_rel = rel_start - cycle_pos;
                    if (to_rel < lim) lim = to_rel;
                }
            }
        }
    }
    return lim ? lim : 1;
}

// Runs from the render path; only writes flags, so RAM-resident and cheap.
// With a swap pending the generator stays "running" so the pump keeps
// feeding blocks; siggen_service() relaunches with the staged config.
DSP_TIME_CRITICAL
static void mark_stopped(uint8_t reason) {
    eng_state = SIGGEN_STATE_IDLE;
    fade_dir = 0;
    if (!swap_pending) {
        siggen_running = false;
        siggen_raw_mask = 0;
    }
    stop_reason = reason;
    notify_state = SIGGEN_STATE_IDLE;
    notify_reason = reason;
    notify_pending = true;
}

// Handle end of the current engine segment; returns false when the engine
// went idle (block remainder should be silence)
DSP_TIME_CRITICAL
static bool segment_finished(void) {
    switch (eng_state) {
    case SIGGEN_STATE_RUN:
        if (cycle_len == 0) return true;    // unbounded, duration-driven
        cycles_done++;
        if (cycles_target > 0 && cycles_done >= cycles_target) {
            mark_stopped(SIGGEN_STOP_COMPLETED);
            return false;
        }
        if (walk_on) walk_pos = mask_next_bit(cfg.channel_mask, (uint8_t)(walk_pos + 1));
        if (drv()->gap_samples > 0) {
            eng_state = SIGGEN_STATE_GAP;
            seg_remaining = drv()->gap_samples;
        } else {
            synth_reset_cycle();
            cycle_pos = 0;
            cycle_len = current_cycle_samples();
            seg_remaining = cycle_len;
        }
        return true;

    case SIGGEN_STATE_GAP:
        synth_reset_cycle();
        eng_state = SIGGEN_STATE_RUN;
        cycle_pos = 0;
        cycle_len = current_cycle_samples();
        seg_remaining = cycle_len;
        return true;

    default:
        return false;
    }
}

// Build the segment plan for n samples; advances all engine timing.
// A pending config swap is applied by siggen_service() in cold context; the
// remainder of this block plays silence.
DSP_TIME_CRITICAL
static uint8_t plan_block(uint32_t n, SegPlan *plan) {
    uint8_t nseg = 0;
    uint32_t pos = 0;
    while (pos < n && nseg < SIGGEN_MAX_SEGS) {
        if (eng_state == SIGGEN_STATE_IDLE) break;
        uint32_t len = next_boundary(n - pos);
        SegPlan *s = &plan[nseg++];
        s->off = (uint16_t)pos;
        s->len = (uint16_t)len;
        s->silent = (eng_state == SIGGEN_STATE_GAP);
        s->cycle_base = cycle_pos;
        s->mask = walk_on ? (eff_mask_cached & (1u << walk_pos)) : eff_mask_cached;

        // endpoint-sampled combined gain ramp (fade overlay x cycle window)
        uint32_t f0 = fade_level;
        uint32_t f1 = fade_level;
        if (fade_dir > 0)      f1 = (f1 + len > fade_len) ? fade_len : f1 + len;
        else if (fade_dir < 0) f1 = (f1 > len) ? f1 - len : 0;
        gain_t g0 = samp_scale(fade_gain_of(f0), window_gain_at(cycle_pos));
        gain_t g1 = samp_scale(fade_gain_of(f1), window_gain_at(cycle_pos + len));
#if PICO_RP2350
        s->g0 = g0; s->g_step = (g1 - g0) / (float)len;
#else
        s->g0 = g0; s->g_step = (int32_t)(((int64_t)g1 - g0) / (int32_t)len);
#endif

        // advance timing
        pos += len;
        elapsed_samples += len;
        fade_level = f1;
        if (fade_dir > 0 && fade_level >= fade_len) {
            fade_dir = 0;
        } else if (fade_dir < 0 && fade_level == 0) {
            mark_stopped(stop_reason ? stop_reason : SIGGEN_STOP_HOST);
            continue;                     // engine idle; remainder is silence
        }
        if (eng_state == SIGGEN_STATE_RUN) {
            cycle_pos += len;
            if (duration_limited) {
                duration_remaining = (duration_remaining > len)
                                     ? duration_remaining - len : 0;
                // begin the tail fade so it completes right at duration end
                if (fade_dir == 0 && duration_remaining <= fade_len) {
                    stop_reason = SIGGEN_STOP_COMPLETED;
                    fade_dir = -1;
                }
            }
        }
        if (seg_remaining > 0) {
            seg_remaining -= len;
            if (seg_remaining == 0 && !segment_finished())
                continue;                 // engine stopped; remainder is silence
        }
    }
    return nseg;
}

// ---------------------------------------------------------------------------
// Render
// ---------------------------------------------------------------------------

DSP_TIME_CRITICAL
static void synth_segment(samp_t *dst, const SegPlan *s, uint8_t ch) {
    switch (cfg.signal_type) {
    case SIGGEN_SINE:       synth_sine(dst, s->len); break;
    case SIGGEN_SQUARE:     synth_square(dst, s->len); break;
    case SIGGEN_WHITE:      synth_white(dst, s->len, ch); break;
    case SIGGEN_PINK:       synth_pink(dst, s->len, ch); break;
    case SIGGEN_SWEEP_LOG:
    case SIGGEN_SWEEP_LIN:
    case SIGGEN_SWEEP_STEP: synth_sweep(dst, s->len, cfg.signal_type); break;
    case SIGGEN_IMPULSE:    synth_click(dst, s->len, s->cycle_base, false); break;
    case SIGGEN_CLICKS_ALT: synth_click(dst, s->len, s->cycle_base, true); break;
    case SIGGEN_POLARITY:   synth_polarity(dst, s->len, s->cycle_base); break;
    case SIGGEN_TONE_BURST: synth_burst(dst, s->len, s->cycle_base); break;
    case SIGGEN_TONE_PAIR:  synth_tone_pair(dst, s->len); break;
    case SIGGEN_MULTITONE:  synth_multitone(dst, s->len); break;
    case SIGGEN_ISP:        synth_isp(dst, s->len); break;
    case SIGGEN_CHANNEL_ID: synth_channel_id(dst, s->len, s->cycle_base); break;
    default:                memset(dst, 0, s->len * sizeof(samp_t)); break;
    }
}

DSP_TIME_CRITICAL
#if PICO_RP2350
void siggen_render(float (*bufs)[AUDIO_BUFFER_SAMPLES], uint32_t sample_count,
                   uint32_t sample_rate_hz)
#else
void siggen_render(int32_t (*bufs)[AUDIO_BUFFER_SAMPLES], uint32_t sample_count,
                   uint32_t sample_rate_hz)
#endif
{
    if (!siggen_running || sample_count == 0) return;

    // pick nearest precomputed rate row (44.1/48/96)
    uint8_t ri = 1;
    for (uint8_t i = 0; i < SIGGEN_NUM_RATES; i++)
        if (siggen_rates[i] == sample_rate_hz) { ri = i; break; }
    rate_idx = ri;

    uint32_t enabled = 0;
    for (int o = 0; o < NUM_OUTPUT_CHANNELS; o++)
        if (matrix_mixer.outputs[o].enabled) enabled |= 1u << o;
    uint32_t eff = cfg.channel_mask & enabled & ((1u << NUM_OUTPUT_CHANNELS) - 1);
    eff_mask_cached = eff;

    SegPlan plan[SIGGEN_MAX_SEGS];
    uint8_t nseg = plan_block(sample_count, plan);

    uint32_t pos_covered = (nseg > 0) ? (uint32_t)(plan[nseg-1].off + plan[nseg-1].len) : 0;

    for (uint8_t si = 0; si < nseg; si++) {
        const SegPlan *s = &plan[si];
        uint32_t chans = s->mask;
        // eff channels not carrying signal this segment get silence
        uint32_t silent_ch = eff & ~chans;
        for (int o = 0; o < NUM_OUTPUT_CHANNELS; o++)
            if (silent_ch & (1u << o))
                memset(&bufs[o][s->off], 0, s->len * sizeof(samp_t));
        if (!chans) continue;

        bool decorr = (cfg.flags & SIGGEN_FLAG_DECORR) &&
                      (cfg.signal_type == SIGGEN_WHITE || cfg.signal_type == SIGGEN_PINK);

        if (s->silent) {
            for (int o = 0; o < NUM_OUTPUT_CHANNELS; o++)
                if (chans & (1u << o))
                    memset(&bufs[o][s->off], 0, s->len * sizeof(samp_t));
            continue;
        }

        uint8_t primary = mask_lowest_bit(chans);

        if (decorr) {
            for (int o = 0; o < NUM_OUTPUT_CHANNELS; o++) {
                if (!(chans & (1u << o))) continue;
                samp_t *dst = &bufs[o][s->off];
                synth_segment(dst, s, (uint8_t)o);
                if (s->g0 != GAIN_ONE || s->g_step != 0) {
                    gain_t g = s->g0;
                    for (uint32_t i = 0; i < s->len; i++) {
                        dst[i] = samp_scale(dst[i], g);
                        g += s->g_step;
                    }
                }
                if (cfg.invert_mask & (1u << o))
                    for (uint32_t i = 0; i < s->len; i++) dst[i] = -dst[i];
            }
            continue;
        }

        samp_t *pdst = &bufs[primary][s->off];
        synth_segment(pdst, s, primary);
        if (s->g0 != GAIN_ONE || s->g_step != 0) {
            gain_t g = s->g0;
            for (uint32_t i = 0; i < s->len; i++) {
                pdst[i] = samp_scale(pdst[i], g);
                g += s->g_step;
            }
        }
        bool prim_inv = (cfg.invert_mask >> primary) & 1u;
        for (int o = 0; o < NUM_OUTPUT_CHANNELS; o++) {
            if (o == primary || !(chans & (1u << o))) continue;
            samp_t *dst = &bufs[o][s->off];
            bool inv = ((cfg.invert_mask >> o) & 1u) != prim_inv;
            if (inv)
                for (uint32_t i = 0; i < s->len; i++) dst[i] = -pdst[i];
            else
                memcpy(dst, pdst, s->len * sizeof(samp_t));
        }
        if (prim_inv)
            for (uint32_t i = 0; i < s->len; i++) pdst[i] = -pdst[i];
    }

    // engine went idle mid-block: silence the tail of every eff channel
    if (pos_covered < sample_count) {
        for (int o = 0; o < NUM_OUTPUT_CHANNELS; o++)
            if (eff & (1u << o))
                memset(&bufs[o][pos_covered],
                       0, (sample_count - pos_covered) * sizeof(samp_t));
    }
}

// ---------------------------------------------------------------------------
// Control / config (cold, flash-resident)
// ---------------------------------------------------------------------------

// Compute one rate row of derived parameters from cfg
static void compute_derived(uint32_t fs, SiggenDerived *d) {
    memset(d, 0, sizeof(*d));
    const double two32 = 4294967296.0;
    const double two48 = 281474976710656.0;
    float nyq = 0.45f * (float)fs;

    float f1 = cfg.p1, f2 = cfg.p2;
    d->fade_samples = (uint32_t)((uint64_t)fs * SIGGEN_FADE_MS / 1000u);
    if (d->fade_samples < 8) d->fade_samples = 8;
    d->gap_samples = (uint32_t)((uint64_t)fs * cfg.gap_ms / 1000u);

    switch (cfg.signal_type) {
    case SIGGEN_SINE:
    case SIGGEN_SQUARE: {
        float f = f1;
        if (f < 1.0f) f = 1.0f;
        if (f > nyq) f = nyq;
        d->osc_inc[0] = (uint32_t)((double)f / fs * two32);
        break;
    }
    case SIGGEN_SWEEP_LOG:
    case SIGGEN_SWEEP_LIN:
    case SIGGEN_SWEEP_STEP: {
        if (f1 < 1.0f) f1 = 1.0f;
        if (f2 > nyq) f2 = nyq;
        if (f2 <= f1) f2 = f1 + 1.0f;
        uint32_t dur = (uint32_t)((uint64_t)fs * cfg.duration_ms / 1000u);
        if (dur < d->fade_samples * 2) dur = d->fade_samples * 2;
        d->cycle_samples = dur;
        d->sweep_inc0 = (uint64_t)((double)f1 / fs * two48);
        if (cfg.signal_type == SIGGEN_SWEEP_LOG) {
            double eps = log((double)f2 / f1) / (double)dur;   // per-sample growth
            d->sweep_eps_q31 = (int32_t)(eps * 2147483648.0);
        } else if (cfg.signal_type == SIGGEN_SWEEP_LIN) {
            d->sweep_dinc = (uint64_t)(((double)f2 - f1) / fs * two48 / dur);
        } else {
            float spo = cfg.p3;
            if (spo < 1.0f) spo = 3.0f;
            if (spo > 24.0f) spo = 24.0f;
            float dwell_ms = cfg.p4;
            if (dwell_ms < 20.0f) dwell_ms = 250.0f;
            d->step_dwell = (uint32_t)((uint64_t)fs * (uint32_t)dwell_ms / 1000u);
            d->step_ratio_q30 = (uint32_t)(pow(2.0, 1.0 / spo) * 1073741824.0);
            // stepped sweep length: steps to cover f1..f2 at dwell each
            double steps = log2((double)f2 / f1) * spo;
            d->cycle_samples = (uint32_t)((steps + 1.0) * d->step_dwell);
        }
        d->attack_samples = d->release_samples = d->fade_samples;
        break;
    }
    case SIGGEN_IMPULSE:
    case SIGGEN_CLICKS_ALT: {
        float period = f1;
        if (!(period >= 10.0f)) period = 500.0f;    // catches NaN too
        d->cycle_samples = (uint32_t)((uint64_t)fs * (uint32_t)period / 1000u);
        break;
    }
    case SIGGEN_POLARITY: {
        float pw = f1;
        if (pw < 1.0f) pw = 5.0f;
        float period = cfg.p2;
        if (period < pw * 2.0f) period = 500.0f;
        d->pulse_samples = (uint32_t)((uint64_t)fs * (uint32_t)(pw * 10.0f) / 10000u);
        if (d->pulse_samples < 4) d->pulse_samples = 4;
        d->pulse_env_step = (1u << 31) / d->pulse_samples;
        d->cycle_samples = (uint32_t)((uint64_t)fs * (uint32_t)period / 1000u);
        if (d->cycle_samples < d->pulse_samples * 2)
            d->cycle_samples = d->pulse_samples * 2;
        break;
    }
    case SIGGEN_TONE_BURST: {
        float f = f1;
        if (f < 10.0f) f = 1000.0f;
        if (f > nyq) f = nyq;
        d->osc_inc[0] = (uint32_t)((double)f / fs * two32);
        float on_cyc = cfg.p2 >= 1.0f ? cfg.p2 : 8.0f;
        float off_cyc = cfg.p3 >= 0.0f ? cfg.p3 : 8.0f;
        float edge_cyc = cfg.p4;
        if (edge_cyc < 0.0f) edge_cyc = 2.0f;
        d->burst_on = (uint32_t)(on_cyc * fs / f);
        if (d->burst_on < 8) d->burst_on = 8;
        d->burst_edge = (uint32_t)(edge_cyc * fs / f);
        if (d->burst_edge * 2 > d->burst_on) d->burst_edge = d->burst_on / 2;
        d->burst_env_step = d->burst_edge ? (1u << 30) / d->burst_edge : 0;
        d->cycle_samples = d->burst_on + (uint32_t)(off_cyc * fs / f);
        break;
    }
    case SIGGEN_TONE_PAIR: {
        if (f1 < 1.0f) f1 = 60.0f;
        if (f2 < 1.0f) f2 = 7000.0f;
        if (f1 > nyq) f1 = nyq;
        if (f2 > nyq) f2 = nyq;
        d->osc_inc[0] = (uint32_t)((double)f1 / fs * two32);
        d->osc_inc[1] = (uint32_t)((double)f2 / fs * two32);
        break;
    }
    case SIGGEN_MULTITONE: {
        float lo = cfg.p2 >= 1.0f ? cfg.p2 : 20.0f;
        float hi = cfg.p3 > lo ? cfg.p3 : 20000.0f;
        if (hi > nyq) hi = nyq;
        for (uint8_t k = 0; k < mt_count; k++) {
            double f = (mt_count > 1)
                ? lo * pow((double)hi / lo, (double)k / (mt_count - 1))
                : lo;
            d->osc_inc[k] = (uint32_t)(f / fs * two32);
        }
        break;
    }
    case SIGGEN_CHANNEL_ID: {
        float blip_ms = (cfg.p1 >= 30.0f && cfg.p1 <= 1000.0f) ? cfg.p1 : 120.0f;
        d->id_blip     = (uint32_t)((uint64_t)fs * (uint32_t)blip_ms / 1000u);
        d->id_blip_gap = (uint32_t)((uint64_t)fs * 100u / 1000u);
        d->id_tail     = (uint32_t)((uint64_t)fs * 700u / 1000u);
        d->id_env_step = (1u << 31) / d->id_blip;
        // pitch is per walk channel; increment refreshed in apply/walk via
        // synth_reset_cycle -> uses osc_inc[0], set below for channel 0.
        break;
    }
    default:
        break;
    }

    // continuous types: duration governs total play time; walked continuous
    // uses duration as the per-channel dwell
    if (type_timing[cfg.signal_type] == SIGGEN_TIMING_CONTINUOUS) {
        uint32_t durs = (uint32_t)((uint64_t)fs * cfg.duration_ms / 1000u);
        if (walk_on) {
            if (durs < d->fade_samples * 2) durs = 2 * fs;   // default 2 s dwell
            d->cycle_samples = durs;
            d->attack_samples = d->release_samples = d->fade_samples;
        } else {
            // fade-in plus tail fade need at least 2x fade to be honored
            if (durs > 0 && durs < d->fade_samples * 2)
                durs = d->fade_samples * 2;
            d->duration_samples = durs;                      // 0 = infinite
        }
    }

    // Defensive floors: a zero cycle would turn a cycle-timed type into an
    // unbounded one, and sub-millisecond pattern cycles would exhaust the
    // per-block segment plan.  1 ms is well below any audible pattern rate.
    if (d->step_dwell == 0 && cfg.signal_type == SIGGEN_SWEEP_STEP)
        d->step_dwell = fs / 100u;
    if (d->cycle_samples > 0 &&
        type_timing[cfg.signal_type] == SIGGEN_TIMING_PATTERN) {
        uint32_t min_cycle = fs / 1000u;
        if (d->cycle_samples < min_cycle) d->cycle_samples = min_cycle;
    }
}

// Precompute CHANNEL_ID pitch increments for every channel and rate
static void fill_id_pitch_table(void) {
    for (uint8_t r = 0; r < SIGGEN_NUM_RATES; r++) {
        for (uint8_t ch = 0; ch < NUM_OUTPUT_CHANNELS; ch++) {
            float f = id_pitch[ch % 5] * ((ch >= 5) ? 2.0f : 1.0f);
            id_inc[r][ch] =
                (uint32_t)((double)f / siggen_rates[r] * 4294967296.0);
        }
    }
}

// Push the deferred notification immediately (cold context only).  Called
// before any transition that would overwrite the single pending slot, so
// back-to-back transitions in one main-loop iteration each reach the host.
static void flush_notify(void) {
    if (notify_pending) {
        notify_pending = false;
        notify_push_siggen_state(notify_state, notify_reason,
                                 cfg.signal_type,
                                 walk_on ? walk_pos : 0xFF);
    }
}

static void start_engine(void) {
    flush_notify();

    // level -> linear amplitude
    float db = cfg.level_db;
    if (db > 0.0f) db = 0.0f;
    if (db < -120.0f) db = -120.0f;
    float lin = powf(10.0f, db / 20.0f);

    walk_on = (cfg.flags & SIGGEN_FLAG_WALK) ||
              (cfg.signal_type == SIGGEN_CHANNEL_ID);
    walk_pos = mask_lowest_bit(cfg.channel_mask);

    mt_count = 0;
    if (cfg.signal_type == SIGGEN_MULTITONE) {
        int c = (int)cfg.p1;
        if (c < 2) c = 10;
        if (c > SIGGEN_MULTITONE_MAX) c = SIGGEN_MULTITONE_MAX;
        mt_count = (uint8_t)c;
        // Schroeder phases bound the crest factor
        for (uint8_t k = 0; k < mt_count; k++) {
            double frac = -((double)k * (k - 1)) / (2.0 * mt_count);
            frac -= floor(frac);
            phase_init[k] = (uint32_t)(frac * 4294967296.0);
        }
    }

#if PICO_RP2350
    amp = lin;
    amp_tone = mt_count ? lin / (float)mt_count : lin;
    {
        float r = (cfg.signal_type == SIGGEN_TONE_PAIR && cfg.p3 > 0.01f)
                  ? cfg.p3 : 1.0f;
        amp_pair[0] = lin * r / (1.0f + r);
        amp_pair[1] = lin / (1.0f + r);
    }
#else
    amp = (int32_t)(lin * (float)SAMPLE_FULL_SCALE_Q28);
    amp_tone = mt_count ? amp / mt_count : amp;
    {
        float r = (cfg.signal_type == SIGGEN_TONE_PAIR && cfg.p3 > 0.01f)
                  ? cfg.p3 : 1.0f;
        amp_pair[0] = (int32_t)(lin * r / (1.0f + r) * (float)SAMPLE_FULL_SCALE_Q28);
        amp_pair[1] = (int32_t)(lin / (1.0f + r) * (float)SAMPLE_FULL_SCALE_Q28);
    }
#endif

    for (uint8_t r = 0; r < SIGGEN_NUM_RATES; r++)
        compute_derived(siggen_rates[r], &drv_tab[r]);
    if (cfg.signal_type == SIGGEN_CHANNEL_ID) fill_id_pitch_table();

    // Resolve the live pipeline rate before latching rate-dependent state;
    // rate_idx is otherwise only refreshed by siggen_render while running.
    rate_idx = 1;
    for (uint8_t r = 0; r < SIGGEN_NUM_RATES; r++)
        if (siggen_rates[r] == audio_state.freq) { rate_idx = r; break; }

    // reset run state
    for (int ch = 0; ch < NUM_OUTPUT_CHANNELS; ch++) {
        rng_state[ch] = 0x9E3779B9u * (uint32_t)(ch + 1) + 0x7F4A7C15u;
        pink_s[ch][0] = pink_s[ch][1] = pink_s[ch][2] = 0;
    }
    click_sign = -1;   // synth_reset_cycle below toggles: first click positive
    elapsed_samples = 0;
    cycles_done = 0;
    duration_limited = false;
    duration_remaining = 0;

    uint8_t timing = type_timing[cfg.signal_type];
    if (timing == SIGGEN_TIMING_CONTINUOUS && !walk_on) {
        cycles_target = 0;
        if (cfg.duration_ms > 0) duration_limited = true;
    } else if (walk_on) {
        uint8_t wl = mask_popcount(cfg.channel_mask);
        cycles_target = (cfg.repeat > 0 && wl > 0)
                        ? (uint32_t)cfg.repeat * wl : 0;
    } else {
        cycles_target = cfg.repeat;
    }

    synth_reset_cycle();

    // duration_remaining latched at start from the active rate row
    if (duration_limited)
        duration_remaining = drv_tab[rate_idx].duration_samples
                           ? drv_tab[rate_idx].duration_samples : 0;
    if (duration_remaining == 0) duration_limited = false;

    // fade-in overlays the start of the first cycle.  Pattern-timing types
    // start impulsively by design (a fade would mute the first click), so
    // they start at full level; the stop fade still applies.
    fade_len = drv_tab[rate_idx].fade_samples;
    if (timing == SIGGEN_TIMING_PATTERN) {
        fade_level = fade_len;
        fade_dir = 0;
    } else {
        fade_level = 0;
        fade_dir = 1;
    }
    eng_state = SIGGEN_STATE_RUN;
    cycle_pos = 0;
    cycle_len = current_cycle_samples();
    seg_remaining = cycle_len;
    stop_reason = SIGGEN_STOP_NONE;

    siggen_raw_mask = (cfg.flags & SIGGEN_FLAG_RAW)
                      ? (cfg.channel_mask & ((1u << NUM_OUTPUT_CHANNELS) - 1)) : 0;
    siggen_running = true;

    notify_state = SIGGEN_STATE_RUN;
    notify_reason = SIGGEN_STOP_NONE;
    notify_pending = true;
}

static const SiggenTypeDesc caps_types[SIGGEN_TYPE_COUNT];   // defined below

bool siggen_stage_config(const void *payload, uint16_t len) {
    if (len < sizeof(SiggenConfig)) return false;
    SiggenConfig c;
    memcpy(&c, payload, sizeof(c));
    if (c.version != SIGGEN_CFG_VERSION) return false;
    if (c.signal_type >= SIGGEN_TYPE_COUNT) return false;
    uint16_t valid = (uint16_t)((1u << NUM_OUTPUT_CHANNELS) - 1);
    if ((c.channel_mask & valid) == 0) return false;
    c.channel_mask &= valid;
    c.invert_mask &= c.channel_mask;
    if (!(c.level_db <= 0.0f && c.level_db >= -120.0f)) {
        // clamp NaN/out-of-range rather than reject; NaN fails both compares
        c.level_db = (c.level_db > 0.0f) ? 0.0f : -120.0f;
    }
    // Sanitize p1..p4 against the caps table: NaN becomes 0 (0 selects the
    // type default downstream); nonzero values clamp to the advertised range.
    {
        float pv[4] = { c.p1, c.p2, c.p3, c.p4 };
        for (int k = 0; k < 4; k++) {
            const SiggenParamDesc *pd = &caps_types[c.signal_type].p[k];
            if (pd->semantic == SIGGEN_PARAM_UNUSED) { pv[k] = 0.0f; continue; }
            if (!(pv[k] == pv[k])) pv[k] = 0.0f;             // NaN check
            else if (pv[k] != 0.0f) {
                if (pv[k] < pd->min) pv[k] = pd->min;
                if (pv[k] > pd->max) pv[k] = pd->max;
            }
        }
        c.p1 = pv[0]; c.p2 = pv[1]; c.p3 = pv[2]; c.p4 = pv[3];
    }
    uint8_t timing = type_timing[c.signal_type];
    if (timing == SIGGEN_TIMING_SWEEP && c.duration_ms == 0) return false;

    if (siggen_running) {
        // fade out, then siggen_service() relaunches with the new config
        cfg_next = c;
        swap_pending = true;
        stop_reason = SIGGEN_STOP_RECONFIG;
        fade_dir = -1;
    } else {
        cfg = c;
    }
    cfg_valid = true;
    return true;
}

bool siggen_control(uint8_t action) {
    switch (action) {
    case SIGGEN_CTL_START:
        if (!cfg_valid) return false;
        if (siggen_running) {
            // restart: fade out, then relaunch.  Keep an already-staged
            // config (SET_CONFIG then START must not resurrect the old one).
            if (!swap_pending) cfg_next = cfg;
            swap_pending = true;
            stop_reason = SIGGEN_STOP_RECONFIG;
            fade_dir = -1;
        } else {
            start_engine();
        }
        return true;
    case SIGGEN_CTL_STOP:
        if (!siggen_running) return true;
        swap_pending = false;
        if (eng_state == SIGGEN_STATE_GAP) {
            mark_stopped(SIGGEN_STOP_HOST);   // gap is silent; no fade needed
            flush_notify();
        } else {
            stop_reason = SIGGEN_STOP_HOST;
            fade_dir = -1;
        }
        return true;
    case SIGGEN_CTL_STOP_NOW:
        swap_pending = false;
        if (siggen_running) {
            mark_stopped(SIGGEN_STOP_HOST);
            flush_notify();
        }
        return true;
    default:
        return false;
    }
}

void siggen_stop_immediate(uint8_t reason) {
    swap_pending = false;
    if (siggen_running) {
        mark_stopped(reason);
        flush_notify();
    }
}

void siggen_get_config(SiggenConfig *out) {
    if (cfg_valid) {
        *out = cfg;
    } else {
        memset(out, 0, sizeof(*out));
        out->version = SIGGEN_CFG_VERSION;
    }
}

void siggen_get_status(SiggenStatus *out) {
    memset(out, 0, sizeof(*out));
    out->version = SIGGEN_CFG_VERSION;
    // wire state derives the fade overlay: rising = FADE_IN, falling = FADE_OUT
    if (eng_state == SIGGEN_STATE_IDLE)      out->state = SIGGEN_STATE_IDLE;
    else if (fade_dir > 0)                   out->state = SIGGEN_STATE_FADE_IN;
    else if (fade_dir < 0)                   out->state = SIGGEN_STATE_FADE_OUT;
    else                                     out->state = eng_state;
    out->signal_type = cfg.signal_type;
    out->active_channel = (siggen_running && walk_on) ? walk_pos : 0xFF;
    uint32_t fs = siggen_rates[rate_idx];
    out->elapsed_ms = (uint32_t)(elapsed_samples * 1000u / fs);
    out->cycles_done = (uint16_t)((cycles_done > 0xFFFF) ? 0xFFFF : cycles_done);
    out->stop_reason = stop_reason;
    uint8_t tm = type_timing[cfg.signal_type];
    if (siggen_running && tm == SIGGEN_TIMING_SWEEP)
        out->current_freq = (float)((double)sweep_inc * fs / 281474976710656.0);
}

void siggen_service(void) {
    // relaunch after a fade-out with a staged config (SET_CONFIG or START
    // while running); done here so the render path never calls cold code
    if (swap_pending && eng_state == SIGGEN_STATE_IDLE) {
        swap_pending = false;
        cfg = cfg_next;
        start_engine();
    }
    flush_notify();
}

// ---------------------------------------------------------------------------
// Capabilities
// ---------------------------------------------------------------------------

static const SiggenCapsHeader caps_header = {
    .version = SIGGEN_CFG_VERSION,
    .type_count = SIGGEN_TYPE_COUNT,
    .output_channels = NUM_OUTPUT_CHANNELS,
    .multitone_max = SIGGEN_MULTITONE_MAX,
    .valid_channel_mask = (uint16_t)((1u << NUM_OUTPUT_CHANNELS) - 1),
    .reserved0 = 0,
};

#define PD_NONE               { SIGGEN_PARAM_UNUSED, 0, 0, 0 }
#define PD(sem, mn, mx, df)   { (sem), (mn), (mx), (df) }

static const SiggenTypeDesc caps_types[SIGGEN_TYPE_COUNT] = {
    { SIGGEN_SINE, "sine", SIGGEN_TIMING_CONTINUOUS,
      { PD(SIGGEN_PARAM_FREQ_HZ, 1, 40000, 1000), PD_NONE, PD_NONE, PD_NONE } },
    { SIGGEN_SQUARE, "square", SIGGEN_TIMING_CONTINUOUS,
      { PD(SIGGEN_PARAM_FREQ_HZ, 1, 40000, 100), PD_NONE, PD_NONE, PD_NONE } },
    { SIGGEN_WHITE, "white", SIGGEN_TIMING_CONTINUOUS,
      { PD_NONE, PD_NONE, PD_NONE, PD_NONE } },
    { SIGGEN_PINK, "pink", SIGGEN_TIMING_CONTINUOUS,
      { PD_NONE, PD_NONE, PD_NONE, PD_NONE } },
    { SIGGEN_SWEEP_LOG, "swp-log", SIGGEN_TIMING_SWEEP,
      { PD(SIGGEN_PARAM_FREQ_HZ, 1, 40000, 20), PD(SIGGEN_PARAM_FREQ_HZ, 1, 40000, 20000),
        PD_NONE, PD_NONE } },
    { SIGGEN_SWEEP_LIN, "swp-lin", SIGGEN_TIMING_SWEEP,
      { PD(SIGGEN_PARAM_FREQ_HZ, 1, 40000, 20), PD(SIGGEN_PARAM_FREQ_HZ, 1, 40000, 20000),
        PD_NONE, PD_NONE } },
    { SIGGEN_SWEEP_STEP, "swp-stp", SIGGEN_TIMING_SWEEP,
      { PD(SIGGEN_PARAM_FREQ_HZ, 1, 40000, 20), PD(SIGGEN_PARAM_FREQ_HZ, 1, 40000, 20000),
        PD(SIGGEN_PARAM_COUNT, 1, 24, 3), PD(SIGGEN_PARAM_MS, 20, 10000, 250) } },
    { SIGGEN_IMPULSE, "impulse", SIGGEN_TIMING_PATTERN,
      { PD(SIGGEN_PARAM_MS, 10, 60000, 500), PD_NONE, PD_NONE, PD_NONE } },
    { SIGGEN_CLICKS_ALT, "clk-alt", SIGGEN_TIMING_PATTERN,
      { PD(SIGGEN_PARAM_MS, 10, 60000, 500), PD_NONE, PD_NONE, PD_NONE } },
    { SIGGEN_POLARITY, "polarty", SIGGEN_TIMING_PATTERN,
      { PD(SIGGEN_PARAM_MS, 1, 100, 5), PD(SIGGEN_PARAM_MS, 10, 60000, 500),
        PD_NONE, PD_NONE } },
    { SIGGEN_TONE_BURST, "burst", SIGGEN_TIMING_PATTERN,
      { PD(SIGGEN_PARAM_FREQ_HZ, 10, 40000, 1000), PD(SIGGEN_PARAM_CYCLES, 1, 1000, 8),
        PD(SIGGEN_PARAM_CYCLES, 0, 1000, 8), PD(SIGGEN_PARAM_CYCLES, 0, 100, 2) } },
    { SIGGEN_TONE_PAIR, "tonpair", SIGGEN_TIMING_CONTINUOUS,
      { PD(SIGGEN_PARAM_FREQ_HZ, 1, 40000, 60), PD(SIGGEN_PARAM_FREQ_HZ, 1, 40000, 7000),
        PD(SIGGEN_PARAM_RATIO, 0.1f, 10, 4), PD_NONE } },
    { SIGGEN_MULTITONE, "multi", SIGGEN_TIMING_CONTINUOUS,
      { PD(SIGGEN_PARAM_COUNT, 2, SIGGEN_MULTITONE_MAX, 10),
        PD(SIGGEN_PARAM_FREQ_HZ, 1, 40000, 20), PD(SIGGEN_PARAM_FREQ_HZ, 1, 40000, 20000),
        PD_NONE } },
    { SIGGEN_ISP, "isp", SIGGEN_TIMING_CONTINUOUS,
      { PD(SIGGEN_PARAM_PATTERN, 0, 1, 0), PD_NONE, PD_NONE, PD_NONE } },
    { SIGGEN_CHANNEL_ID, "chan-id", SIGGEN_TIMING_PATTERN,
      { PD(SIGGEN_PARAM_MS, 30, 1000, 120), PD_NONE, PD_NONE, PD_NONE } },
};

const SiggenCapsHeader *siggen_caps_header(void) { return &caps_header; }

const SiggenTypeDesc *siggen_caps_type(uint8_t index) {
    return (index < SIGGEN_TYPE_COUNT) ? &caps_types[index] : NULL;
}
