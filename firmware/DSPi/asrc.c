/*
 * asrc.c — Polyphase ASRC for SPDIF input drift tracking.
 *
 * See asrc.h for the public API and the architectural overview.
 *
 * ALGORITHM
 * =========
 * Polyphase windowed-sinc FIR with inter-phase interpolation.
 *
 *  - The prototype is a Kaiser-windowed sinc, designed offline by
 *    scripts/gen_asrc_coeffs.py and frozen as a C header. Each polyphase
 *    "phase" is normalized to unity DC gain so that any linear/Hermite
 *    blend of phases preserves DC pass-through.
 *
 *  - Per output sample, the resampler advances a 32-bit fractional phase
 *    accumulator by the committed ratio R (Q32.32, ≈ 1.0 ± a few hundred
 *    ppm for drift tracking). The high bits of phase_frac index a
 *    polyphase row; the low bits are the inter-phase blend coefficient.
 *
 *  - When phase_frac wraps in 32 bits, that signals that the resampler
 *    has crossed an input-sample boundary and an extra input must be
 *    pulled into the history. Otherwise exactly one input is consumed
 *    per output (R is always close to 1.0 in this design).
 *
 *  - RP2350 uses cubic-Hermite (Catmull-Rom) inter-phase blending —
 *    256-phase Hermite gives a noise floor << prototype stop-band, so
 *    overall performance is set by the prototype.
 *
 *  - RP2040 uses linear inter-phase blending. M0+ has no SMULL or DSP
 *    extension, so per-tap arithmetic is software-emulated 32×32→32-low
 *    (~12 cycles per MAC). 32 taps × 2 channels × 2 (linear blend) at
 *    48 kHz fits comfortably inside a single core.
 *
 * SERVO
 * =====
 * Rate feedforward (Fs_in_meas / Fs_pipe_nom) plus a proportional fill
 * trim, both passed through a single-pole LPF on the committed ratio.
 * Time constant τ ≈ 1.3 s — far below pitch-modulation audibility yet
 * fast enough that consumer-pool fill never drifts past its 16-buffer
 * margin given crystal-pair offsets in the ±200 ppm range. See the
 * "### Servo" section in the design plan and asrc.h for the full math.
 *
 * BOUNDARY NOTE
 * =============
 * At an input-sample boundary (phase_idx wrapping L-1 → 0 within the
 * same output), the next polyphase row is conceptually "phase 0 of the
 * next input" and would require a one-sample history shift to be exact.
 * We use the simpler `(phase_idx + 1) % L` lookup against the current
 * history, which is bounded-error: the boundary case occurs only when
 * the full 32-bit phase_frac wraps, i.e. ~once per 10⁴ outputs at
 * 100 ppm offset, and the resulting artifact is well below the
 * prototype stop-band.
 */

#include "asrc.h"

#if SPDIF_USE_ASRC

#include "dsp_pipeline.h"
#include "pico.h"

#include <math.h>
#include <string.h>

/* ---------------- Coefficient table selection ---------------- */
/* Flat-prototype layout (libsamplerate-style):
 *   ASRC_COEFFS[BASE_OFFSET + base + k*L + 0]      = phase[base][k]
 *   ASRC_COEFFS[BASE_OFFSET + base + k*L + 1]      = "next polyphase column" —
 *      at base = L-1, this is phase[0][k+1], which is the algebraically-
 *      correct boundary form needed when the linear/Hermite blend reaches
 *      across the input-sample boundary (phase[0] of next input convolved
 *      against current history is identical to phase[0][k+1] convolved at
 *      the same tap index, by the polyphase L:1 decomposition identity).
 *
 *   Hermite uses offsets {-1, 0, +1, +2} — the 2-element padding at each
 *   end of the table makes those reads always safe (out-of-band → 0).
 */
#if PICO_RP2350
  #include "asrc_coeffs_rp2350.h"
  #define ASRC_PHASES         ASRC_COEFFS_RP2350_PHASES
  #define ASRC_TAPS           ASRC_COEFFS_RP2350_TAPS
  #define ASRC_COEFFS         asrc_coeffs_rp2350
  #define ASRC_BASE_OFFSET    ASRC_COEFFS_RP2350_BASE_OFFSET
  typedef float  sample_t;
  typedef float  coef_t;
#else
  #include "asrc_coeffs_rp2040.h"
  #define ASRC_PHASES         ASRC_COEFFS_RP2040_PHASES
  #define ASRC_TAPS           ASRC_COEFFS_RP2040_TAPS
  #define ASRC_COEFFS         asrc_coeffs_rp2040
  #define ASRC_BASE_OFFSET    ASRC_COEFFS_RP2040_BASE_OFFSET
  typedef int32_t sample_t;
  typedef int32_t coef_t;
#endif

/* Sanity: phase count must be a power of two so phase_idx falls in the
 * top log2(N) bits of phase_frac and modular arithmetic is a simple AND. */
_Static_assert((ASRC_PHASES & (ASRC_PHASES - 1)) == 0,
               "ASRC_PHASES must be a power of two");

#define ASRC_PHASE_MASK   (ASRC_PHASES - 1u)
#define ASRC_PHASE_SHIFT  (32u - __builtin_ctz(ASRC_PHASES))   /* e.g. 24 for L=256 */
#define ASRC_SUB_MASK     ((1u << ASRC_PHASE_SHIFT) - 1u)

/* History circular buffer: one extra slot allows look-one-ahead patterns
 * from later iterations without modulo-add at the read site (we never
 * use the extra slot in the current implementation, but it's free and
 * eliminates a class of off-by-one bugs). */
#define ASRC_HIST_SIZE    (ASRC_TAPS + 1)

/* ---------------- Servo constants ---------------- */
/* (R - 1) * 2^32 = ratio_q32_32 - 2^32. ±5000 ppm clamp. */
#define ASRC_RATIO_MIN_Q   ((int64_t)(0.995 * 4294967296.0))
#define ASRC_RATIO_MAX_Q   ((int64_t)(1.005 * 4294967296.0))
#define ASRC_RATIO_ONE_Q   ((int64_t)4294967296LL)

/* Q32 KP for proportional fill trim. 6.25e-6 per buffer × 2^32 ratio
 * scale ≈ 26843. With max |fill_err| = 8, P-authority is ±50 ppm. */
#define ASRC_KP_FILL_Q     ((int64_t)26843)

/* Q32 KI for integrator. Must be small relative to KP so the integrator
 * tracks SUSTAINED rate-measurement bias, not transients. 1000 per tick
 * × 1 buffer × 50 ticks/sec accumulates ~12 ppm/sec at fill_err = 1.
 * A persistent ±200 ppm bias settles in ~17 seconds. */
#define ASRC_KI_FILL_Q     ((int64_t)1000)
/* Anti-windup clamp on the integrator term: ±400 ppm = ±400e-6 × 2^32. */
#define ASRC_INTG_MAX_Q    ((int64_t)1717987)
#define ASRC_INTG_MIN_Q    (-ASRC_INTG_MAX_Q)

#define ASRC_FILL_TARGET   8           /* 50 % of 16-buffer consumer pool */
#define ASRC_FILL_DEADBAND 1           /* |fill_err| <= 1 → freeze integrator */

/* LPF α = 1/64 per tick. Tick cadence is set by the caller (servo runs
 * on the existing ~20 ms main-loop cadence), giving τ ≈ 1.3 s. */
#define ASRC_LPF_SHIFT     6           /* α = 1 / (1 << ASRC_LPF_SHIFT) */
#define ASRC_HOLDOFF_TICKS 2           /* matches USB feedback FB_HOLDOFF_UPDATES */

/* ---------------- Internal state ---------------- */

/* Coefficients are RAM-placed on RP2040 (the Q30 table is 32 KB — twice the
 * 16 KB XIP cache, so a flash placement would thrash on the per-output
 * phase sweep), flash-resident on RP2350 (one phase = 256 B easily fits in
 * the 16 KB cache). Either way the access is read-only and DSP-time-critical;
 * we just compile against whatever ASRC_COEFFS resolves to. */

static struct {
    /* Pipeline rate, frozen at init/rate-change. */
    uint32_t fs_pipe_nom;

    /* Phase accumulator and committed ratio (R × 2^32 in int64). */
    int64_t  ratio_q32_32;
    int64_t  ratio_target_q32_32;
    uint32_t phase_frac;

    /* History (per channel) — newest sample at hist[(hist_idx-1) mod size]. */
    sample_t hist_l[ASRC_HIST_SIZE];
    sample_t hist_r[ASRC_HIST_SIZE];
    uint32_t hist_idx;
    uint32_t hist_filled;     /* 0..ASRC_TAPS */

    /* Servo / lifecycle. */
    int64_t  fill_integrator_q;  /* PI integrator on fill error (Q32 ratio units) */
    uint8_t  holdoff;            /* down-counts to 0 after reset */
    bool     primed;
    bool     muted;

    /* Telemetry. */
    int32_t  ratio_ppm;
    uint32_t underrun_count;
} asrc_st;

/* ---------------- Math helpers ---------------- */

#if !PICO_RP2350
/* Q30 × Q30 → Q30 on M0+ (ARMv6-M has only `MULS` — 32×32→32-low — no SMULL).
 *
 * Decompose a = ah:al, b = bh:bl with ah/bh signed-16 and al/bl unsigned-16.
 * Then a·b = ah·bh·2³² + (ah·bl + al·bh)·2¹⁶ + al·bl, so
 *      (a·b) ≫ 30 = (ah·bh)·2² + (ah·bl + al·bh) ≫ 14 + (al·bl) ≫ 30.
 * The last term is bounded by 65535²/2³⁰ ≈ 4 LSBs, well below all relevant
 * noise floors, so we drop it.
 *
 * Why Q30 and not Q31:
 *   For full Q31 inputs (ah, bh up to ±32767 and al, bl up to 65535), the
 *   sum (ah·bl + al·bh) can reach ±4·10⁹, which overflows int32. The naïve
 *   Q31 helper (in our earlier draft) silently wrapped on near-full-scale
 *   audio, producing constant metallic distortion. Halving the operand
 *   range to Q30 caps each cross-term at ~1.07e9 and the sum at ~2.14e9,
 *   which fits inside the int32 range with no headroom needed.
 *
 *   Q30 still gives ~180 dB of dynamic range, and our coefficient-table
 *   normalization (per-phase sum = 1.0, max central-tap ≤ ~0.95 in audio
 *   terms) leaves ~0.05 of headroom before saturation. Inputs/outputs are
 *   converted Q31↔Q30 by a single shift at the asrc.c boundary; the rest
 *   of the firmware still sees Q31 left-justified ints. */
static inline int32_t fast_mul_q30(int32_t a, int32_t b) {
    int32_t ah = a >> 16;
    int32_t al = (int32_t)((uint32_t)a & 0xFFFFu);
    int32_t bh = b >> 16;
    int32_t bl = (int32_t)((uint32_t)b & 0xFFFFu);
    int32_t hi  = ah * bh;
    int32_t md  = ah * bl + al * bh;
    return (hi << 2) + (md >> 14);
}
#endif

/* ---------------- Polyphase convolution (per channel) ---------------- */

/* Index helper: the convolution sums phase[p][m] × hist[newest - m] for m
 * in [0, ASRC_TAPS). Hist is circular of size ASRC_HIST_SIZE; we hand-
 * unwind the wrap with a single conditional branch in the inner loop. */
static inline uint32_t hist_read_idx(uint32_t newest, uint32_t m) {
    /* newest is hist_idx - 1 already modular; subtract m and add size. */
    return (newest - m + ASRC_HIST_SIZE) % ASRC_HIST_SIZE;
}

#if PICO_RP2350
/* RP2350: float, cubic-Hermite (Catmull-Rom) inter-phase blend over the
 * flat-prototype layout. Coefficients for the four blended phases at any
 * given output sample sit at adjacent positions in the flat array
 * (offsets -1, 0, +1, +2 from `base + k*L`), so the table access pattern
 * is one cache line per tap — much better than four separate phase rows
 * in a 2D layout.
 *
 * Catmull-Rom basis functions for s ∈ [0, 1):
 *   h_-1 = -s/2 + s² - s³/2
 *   h_0  =  1   - 5s²/2 + 3s³/2
 *   h_+1 =  s/2 + 2s² - 3s³/2
 *   h_+2 = -s²/2 + s³/2
 * Sum identically 1 (DC pass-through). */
static __attribute__((always_inline)) inline float
asrc_convolve_hermite_one(const sample_t *hist,
                          uint32_t       newest,
                          uint32_t       phase_idx,
                          float          s)
{
    const float s2 = s * s;
    const float s3 = s2 * s;
    const float h_m1 = -0.5f * s + s2 - 0.5f * s3;
    const float h_0  =  1.0f - 2.5f * s2 + 1.5f * s3;
    const float h_p1 =  0.5f * s + 2.0f * s2 - 1.5f * s3;
    const float h_p2 = -0.5f * s2 + 0.5f * s3;

    /* Pointer to the prototype "anchor" for this output: position `base`
     * within the flat array, with the +ASRC_BASE_OFFSET skipping the
     * leading zero pad that lets us read at -1 without a bounds check. */
    const coef_t *base_ptr = &ASRC_COEFFS[ASRC_BASE_OFFSET + phase_idx];

    float acc = 0.0f;
    for (uint32_t k = 0; k < ASRC_TAPS; k++) {
        const coef_t *p = base_ptr + k * ASRC_PHASES;
        /* p[-1] = phase[base-1][k] (or pre-pad zero at base=0)
         * p[ 0] = phase[base  ][k]
         * p[+1] = phase[base+1][k]   — at base=L-1 this resolves to
         *                              phase[0][k+1], the boundary-
         *                              correct "phase 0 of next input"
         *                              against current history.
         * p[+2] = phase[base+2][k]   — analogously at base=L-2 / L-1. */
        const float c_eff = h_m1 * p[-1] + h_0 * p[0]
                          + h_p1 * p[ 1] + h_p2 * p[2];
        acc += c_eff * hist[hist_read_idx(newest, k)];
    }
    return acc;
}
#else
/* RP2040: Q30 fixed-point, linear inter-phase blend over the flat layout.
 * Per tap k we read two adjacent prototype elements (`p[0]`, `p[1]`) which
 * are the two polyphase coefficients the blend interpolates between. At
 * base = L-1 the `+1` access naturally lands on phase[0][k+1] — the
 * algebraically-correct boundary form (see flat-layout comment above). */
static __attribute__((always_inline)) inline int32_t
asrc_convolve_linear_one(const sample_t *hist,
                         uint32_t        newest,
                         uint32_t        phase_idx,
                         int32_t         sub_q30)
{
    const coef_t *base_ptr = &ASRC_COEFFS[ASRC_BASE_OFFSET + phase_idx];

    int64_t acc = 0;
    for (uint32_t k = 0; k < ASRC_TAPS; k++) {
        const coef_t *p = base_ptr + k * ASRC_PHASES;
        const int32_t coef0 = p[0];
        const int32_t coef1 = p[1];   /* "next polyphase column" — see comment above */
        const int32_t delta = coef1 - coef0;
        const int32_t c_eff = coef0 + fast_mul_q30(sub_q30, delta);
        const int32_t s     = hist[hist_read_idx(newest, k)];
        acc += (int64_t)fast_mul_q30(c_eff, s);
    }
    /* Saturation: bounded by ±2^30 in Q30 for unity-DC FIR with ±1.0 input. */
    const int64_t sat_hi =  (int64_t)1 << 30;
    const int64_t sat_lo = -(int64_t)1 << 30;
    if (acc > sat_hi - 1) acc = sat_hi - 1;
    if (acc < sat_lo)     acc = sat_lo;
    return (int32_t)acc;
}
#endif

/* ---------------- Public API ---------------- */

void asrc_init(uint32_t fs_pipe_nom) {
    asrc_st.fs_pipe_nom = fs_pipe_nom ? fs_pipe_nom : 48000u;
    asrc_reset();
}

void asrc_reset(void) {
    /* Wipe history; mark unprimed and muted. Subsequent input pushes will
     * fill the kernel and the first servo tick after will lift the mute. */
    memset(asrc_st.hist_l, 0, sizeof(asrc_st.hist_l));
    memset(asrc_st.hist_r, 0, sizeof(asrc_st.hist_r));
    asrc_st.hist_idx     = 0;
    asrc_st.hist_filled  = 0;

    asrc_st.phase_frac          = 0;
    asrc_st.ratio_q32_32        = ASRC_RATIO_ONE_Q;
    asrc_st.ratio_target_q32_32 = ASRC_RATIO_ONE_Q;
    asrc_st.fill_integrator_q   = 0;
    asrc_st.holdoff             = ASRC_HOLDOFF_TICKS;
    asrc_st.primed              = false;
    asrc_st.muted               = true;
    asrc_st.ratio_ppm           = 0;
}

/* Convert raw int32 left-justified 24-bit input (the format the SPDIF FIFO
 * loop already produces in spdif_input.c) into the platform's native
 * pipeline sample type. Inverse conversion happens at the ASRC output. */
static inline sample_t asrc_in_to_native(int32_t raw_int32) {
#if PICO_RP2350
    /* Convert sign-extended Q31 audio into [-1.0, 1.0] float. */
    return (float)raw_int32 * (1.0f / 2147483648.0f);
#else
    /* RP2040 internal storage is Q30 (one bit narrower than Q31 — see
     * fast_mul_q30 explanation above). Down-shift the input by one bit
     * with simple sign-preserving arithmetic shift; the boundary cast
     * back to Q31 is a left-shift. The 1-bit precision loss at the I/O
     * boundary is negligible (~−180 dB). */
    return raw_int32 >> 1;
#endif
}

static inline int32_t asrc_native_to_out(sample_t s) {
#if PICO_RP2350
    if (s >  1.0f) s =  1.0f;
    if (s < -1.0f) s = -1.0f;
    return (int32_t)(s * 2147483648.0f);
#else
    /* Q30 → Q31 with clipping at the Q30 max (which is ≈0.5 of Q31 full
     * scale, so the << 1 cannot itself overflow the int32 sign bit). */
    if (s >=  ((int32_t)1 << 30))      return INT32_MAX;
    if (s <= -((int32_t)1 << 30) - 1)  return INT32_MIN;
    return s << 1;
#endif
}

/* Push one raw input pair into the circular history. */
static inline void asrc_push_input(int32_t in_l_raw, int32_t in_r_raw) {
    asrc_st.hist_l[asrc_st.hist_idx] = asrc_in_to_native(in_l_raw);
    asrc_st.hist_r[asrc_st.hist_idx] = asrc_in_to_native(in_r_raw);
    asrc_st.hist_idx = (asrc_st.hist_idx + 1u) % ASRC_HIST_SIZE;
    if (asrc_st.hist_filled < ASRC_TAPS) {
        asrc_st.hist_filled++;
        if (asrc_st.hist_filled >= ASRC_TAPS) {
            asrc_st.primed = true;
        }
    }
}

unsigned __not_in_flash_func(asrc_process)(const int32_t *in_l,
                                           const int32_t *in_r,
                                           unsigned       n_in,
                                           int32_t       *out_l,
                                           int32_t       *out_r,
                                           unsigned       out_max,
                                           unsigned      *in_consumed_out)
{
    if (in_consumed_out) *in_consumed_out = 0;

    if (asrc_st.muted) {
        /* While muted, still ingest inputs to populate history so we are
         * primed by the time the servo lifts the mute. Drop output. */
        for (unsigned i = 0; i < n_in; i++) {
            asrc_push_input(in_l[i], in_r[i]);
        }
        if (in_consumed_out) *in_consumed_out = n_in;
        return 0;
    }

    if (n_in == 0 || out_max == 0) return 0;

    unsigned in_consumed = 0;
    unsigned out_count   = 0;

    /* Pre-compute the per-output ratio decomposition; it doesn't change
     * mid-block (servo updates run between blocks, not inside them). */
    const uint32_t int_part  = (uint32_t)(asrc_st.ratio_q32_32 >> 32);
    const uint32_t frac_part = (uint32_t)(asrc_st.ratio_q32_32 & 0xFFFFFFFFu);

    while (out_count < out_max) {
        /* Predict how many inputs this output will consume by advancing
         * phase. For R close to 1.0, this is normally 1, occasionally 2. */
        const uint32_t new_phase = asrc_st.phase_frac + frac_part;
        unsigned       consume   = int_part + (new_phase < asrc_st.phase_frac ? 1u : 0u);

        /* If we don't have enough input left in the caller's buffer, stop
         * here and let the next call provide more. State is preserved. */
        if (in_consumed + consume > n_in) break;

        /* Pull `consume` samples into history. */
        for (unsigned i = 0; i < consume; i++) {
            asrc_push_input(in_l[in_consumed], in_r[in_consumed]);
            in_consumed++;
        }

        asrc_st.phase_frac = new_phase;

        if (!asrc_st.primed) continue;     /* still warming up — no output yet */

        /* newest = hist_idx - 1 (mod size). */
        const uint32_t newest    = (asrc_st.hist_idx + ASRC_HIST_SIZE - 1u) % ASRC_HIST_SIZE;
        const uint32_t phase_idx = (asrc_st.phase_frac >> ASRC_PHASE_SHIFT) & ASRC_PHASE_MASK;
        const uint32_t sub_bits  = asrc_st.phase_frac & ASRC_SUB_MASK;

#if PICO_RP2350
        const float sub = (float)sub_bits * (1.0f / (float)(1ULL << ASRC_PHASE_SHIFT));
        const sample_t y_l = asrc_convolve_hermite_one(asrc_st.hist_l, newest, phase_idx, sub);
        const sample_t y_r = asrc_convolve_hermite_one(asrc_st.hist_r, newest, phase_idx, sub);
#else
        /* Promote sub_bits (24-bit if N_PHASES = 256) to Q30 for the linear
         * blend. Phase fraction sub_bits is in [0, 2^ASRC_PHASE_SHIFT); we
         * want sub ∈ [0, 1) in Q30 = sub_bits << (30 - ASRC_PHASE_SHIFT). */
        const int32_t sub_q30 = (int32_t)(sub_bits << (30 - ASRC_PHASE_SHIFT));
        const sample_t y_l = asrc_convolve_linear_one(asrc_st.hist_l, newest, phase_idx, sub_q30);
        const sample_t y_r = asrc_convolve_linear_one(asrc_st.hist_r, newest, phase_idx, sub_q30);
#endif

        out_l[out_count] = asrc_native_to_out(y_l);
        out_r[out_count] = asrc_native_to_out(y_r);
        out_count++;
    }

    if (in_consumed_out) *in_consumed_out = in_consumed;

    /* Telemetry: if we left input on the caller's plate (out_max not yet
     * reached but ran out of input), it isn't an underrun — we just need
     * more input next call. The "underrun" counter is reserved for cases
     * where input arrives but the pipeline can't accept more outputs;
     * that's tracked by spdif_overruns elsewhere. */
    return out_count;
}

void asrc_servo_tick(uint8_t consumer_fill, float fs_in_meas) {
    if (asrc_st.fs_pipe_nom == 0) return;
    if (fs_in_meas < 20000.0f || fs_in_meas > 220000.0f) return;  /* sanity */

    /* --- Rate feedforward in Q32.32 --- */
    const double r_ff = (double)fs_in_meas / (double)asrc_st.fs_pipe_nom;
    int64_t r_ff_q = (int64_t)(r_ff * 4294967296.0);

    /* --- Fill error and proportional + integral terms ---
     *
     * The proportional term gives fast response to short-term fill drift.
     * The integral term absorbs SUSTAINED bias in r_ff_q — without it,
     * any persistent rate-measurement error larger than KP × max fill
     * (= 50 ppm) saturates the proportional term, leaves the residual
     * uncorrected, and slowly drains or fills the consumer pool until
     * the DAC sees an empty/garbage stream. Adding KI lets the loop
     * absorb a few hundred ppm of measurement bias over ~10 seconds.
     */
    int32_t fill_err = (int32_t)consumer_fill - ASRC_FILL_TARGET;

    int64_t r_fb_q = 0;
    if (fill_err >  ASRC_FILL_DEADBAND ||
        fill_err < -ASRC_FILL_DEADBAND) {
        r_fb_q = (int64_t)ASRC_KP_FILL_Q * fill_err;

        /* Integrate fill error. Sign convention: fill below target
         * (fill_err < 0) → ASRC needs to produce faster → R smaller →
         * ratio adjustment NEGATIVE → integrator NEGATIVE. So the
         * integrator polarity is the same as the proportional term. */
        asrc_st.fill_integrator_q += (int64_t)ASRC_KI_FILL_Q * fill_err;
        if (asrc_st.fill_integrator_q > ASRC_INTG_MAX_Q)
            asrc_st.fill_integrator_q = ASRC_INTG_MAX_Q;
        if (asrc_st.fill_integrator_q < ASRC_INTG_MIN_Q)
            asrc_st.fill_integrator_q = ASRC_INTG_MIN_Q;
    }
    /* Inside the deadband we freeze the integrator (don't reset it) —
     * its value reflects the long-term bias and should persist. */

    /* --- Sum, clamp (anti-windup on the final ratio target) --- */
    int64_t r_target = r_ff_q + r_fb_q + asrc_st.fill_integrator_q;
    if (r_target > ASRC_RATIO_MAX_Q) r_target = ASRC_RATIO_MAX_Q;
    if (r_target < ASRC_RATIO_MIN_Q) r_target = ASRC_RATIO_MIN_Q;
    asrc_st.ratio_target_q32_32 = r_target;

    /* --- Holdoff: first ASRC_HOLDOFF_TICKS cycles populate the LPF
     *     directly (skip the slow chase from the reset 1.0). --- */
    if (asrc_st.holdoff > 0) {
        asrc_st.ratio_q32_32 = r_target;
        asrc_st.holdoff--;
    } else {
        /* --- Single-pole LPF: ratio += (target − ratio) >> ASRC_LPF_SHIFT */
        int64_t delta = r_target - asrc_st.ratio_q32_32;
        asrc_st.ratio_q32_32 += delta >> ASRC_LPF_SHIFT;
    }

    /* Mute lifts as soon as priming completes AND holdoff is done.
     * Checked unconditionally (not gated by holdoff>0) so that priming
     * completing AFTER holdoff has expired still unmutes correctly —
     * otherwise a slow priming would leave us silent forever. */
    if (asrc_st.holdoff == 0 && asrc_st.primed && asrc_st.muted) {
        asrc_st.muted = false;
    }

    /* Telemetry: ppm = (R - 1.0) × 1e6, computed from the committed ratio. */
    int64_t delta_q = asrc_st.ratio_q32_32 - ASRC_RATIO_ONE_Q;
    asrc_st.ratio_ppm = (int32_t)((delta_q * 1000000) >> 32);
}

int32_t  asrc_get_ratio_ppm(void)   { return asrc_st.ratio_ppm; }
uint32_t asrc_get_underrun_count(void) { return asrc_st.underrun_count; }

uint8_t asrc_get_lock_flags(void) {
    uint8_t f = 0;
    if (asrc_st.primed) f |= 0x01u;
    if (asrc_st.muted)  f |= 0x02u;
    return f;
}

#else  /* SPDIF_USE_ASRC == 0 — empty stubs keep link surface stable */

void     asrc_init(uint32_t fs_pipe_nom)            { (void)fs_pipe_nom; }
void     asrc_reset(void)                           { }
unsigned asrc_process(const int32_t *in_l, const int32_t *in_r, unsigned n_in,
                      int32_t *out_l, int32_t *out_r, unsigned out_max,
                      unsigned *in_consumed_out) {
    (void)in_l; (void)in_r; (void)out_l; (void)out_r; (void)out_max;
    if (in_consumed_out) *in_consumed_out = n_in;
    return 0;
}
void     asrc_servo_tick(uint8_t consumer_fill, float fs_in_meas) {
    (void)consumer_fill; (void)fs_in_meas;
}
int32_t  asrc_get_ratio_ppm(void)        { return 0; }
uint8_t  asrc_get_lock_flags(void)       { return 0; }
uint32_t asrc_get_underrun_count(void)   { return 0; }

#endif /* SPDIF_USE_ASRC */
