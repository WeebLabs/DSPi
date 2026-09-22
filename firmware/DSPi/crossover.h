#ifndef CROSSOVER_H
#define CROSSOVER_H

/*
 * crossover.h — Per-output high-order crossover filters.
 *
 * One crossover "band" = one user-visible filter of a chosen family / order /
 * shape. Internally that filter is a cascade of up to 4 biquad sections (one
 * 1st-order section + up to three biquads for high-order Butterworth; pure
 * biquad cascades otherwise). The cascade is run in-place on the per-output
 * sample buffer between the matrix mixer (PASS 4) and the per-output PEQ
 * (PASS 5+).
 *
 * Wire-protocol band addressing:
 *     bands 0..9    active PEQ (filter_recipes[][])
 *     bands 10..19  reserved gap; rejected by handlers today
 *     bands 20..23  crossover (xover_recipes[][band - XOVER_BAND_BASE])
 *     bands 24+     rejected
 *
 * Recipes span NUM_CHANNELS to match the flash slot and wire layouts; the
 * designed cascades and bypass flags are output-indexed (ch - CH_OUT_1).
 * Crossover bands on input channels are rejected at the handler boundary.
 *
 * See Documentation/Features/crossover_filters_spec.md.
 */

#include "config.h"
#include "dsp_pipeline.h"   // Filter type + per-platform kernel API

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// Filter family / metadata
// ---------------------------------------------------------------------------

typedef enum {
    XOVER_FAMILY_NONE = 0,   // FILTER_FLAT and any non-crossover type
    XOVER_FAMILY_LR   = 1,   // Linkwitz-Riley (orders 2, 4, 8 supported)
    XOVER_FAMILY_BW   = 2,   // Butterworth (orders 1..8 supported)
    XOVER_FAMILY_BES  = 3,   // Bessel (orders 2, 4, 6, 8 supported)
} XoverFamily;

typedef struct {
    uint8_t family;        // XoverFamily
    uint8_t order;         // 1..8
    uint8_t is_highpass;   // 0 = LP, 1 = HP
    uint8_t num_sections;  // 1..4
} XoverFilterMeta;

// Decode a FilterType into family / order / shape / section count.
// Returns true for types in [FILTER_XOVER_FIRST, FILTER_XOVER_LAST];
// returns false (and zero-fills *out) for FILTER_FLAT, PEQ types, or any
// value outside the crossover range.
bool xover_filter_meta(uint8_t type, XoverFilterMeta *out);

// ---------------------------------------------------------------------------
// Storage
// ---------------------------------------------------------------------------

// A single crossover band = an ordered cascade of biquad sections.
// `num_sections` ∈ [0..MAX_XOVER_BAND_SECTIONS]; 0 means the band is bypassed
// (also reflected in `bypass`). All sections are pre-baked Filters; the
// existing per-platform kernel (TDF2 on RP2040, hybrid SVF/TDF2 on RP2350)
// runs each section as-is.
#define MAX_XOVER_BAND_SECTIONS 4

typedef struct {
    Filter   sections[MAX_XOVER_BAND_SECTIONS];
    uint8_t  num_sections;
    bool     bypass;         // True ⇒ skip entire band (whole-cascade bypass)
} XoverFilter;

// xover_recipes mirrors filter_recipes (EqParamPacket, channel-indexed; Q and
// gain_db ignored for crossover types). xover_filters holds the designed
// cascades and is indexed by OUTPUT (0..NUM_OUTPUT_CHANNELS-1), not channel.
extern XoverFilter   xover_filters[NUM_OUTPUT_CHANNELS][MAX_XOVER_BANDS];
extern EqParamPacket xover_recipes[NUM_CHANNELS][MAX_XOVER_BANDS];

// Fast-path "no work" flag per OUTPUT: true when every band is bypassed or
// designed-to-flat, so the whole crossover stage is skipped for that output.
extern bool output_xover_bypassed[NUM_OUTPUT_CHANNELS];

// ---------------------------------------------------------------------------
// Init / design / recompute
// ---------------------------------------------------------------------------

// Initialise xover_recipes/xover_filters to defaults:
//   type=FILTER_FLAT, freq=1000.0f, Q=0.707f, gain_db=0.0f, bypass=0
//   channel = ch, band = XOVER_BAND_BASE + i (wire band index; critical: see
//   spec, "Band-field normalization", to prevent live-edit misrouting).
// Called from dsp_init_default_filters() and apply_factory_defaults().
void xover_init_default_filters(void);

// Design one band: read recipe, populate sections + num_sections + bypass.
// Idempotent. Section state is preserved across redesigns so live edits do
// not click; state is reset when a section's SVF/TDF2 path changes, when the
// band is bypassed or re-enabled, or when the section drops out of the new
// cascade (same convention as dsp_compute_coefficients()).
void xover_design_filter(const EqParamPacket *recipe,
                         XoverFilter *band,
                         float sample_rate);

// Rebuild all crossover sections at a new sample rate. Called from
// dsp_recalculate_all_filters().
void xover_recalculate_all(float sample_rate);

// Recompute output_xover_bypassed[out] after writing one band. Takes an
// OUTPUT index, not a channel. Cheap O(MAX_XOVER_BANDS).
void xover_update_output_bypass(uint8_t out);

// ---------------------------------------------------------------------------
// Processing kernel
// ---------------------------------------------------------------------------

// Block-based crossover processing for one output channel. Mirrors the
// dsp_process_channel_block() API. Called from the audio pipeline between
// PASS 4 (matrix mixer) and PASS 5 (per-output PEQ).
#if PICO_RP2350
void xover_process_channel_block(XoverFilter *bands,
                                 float * __restrict samples,
                                 uint32_t sample_count);
#else
void xover_process_channel_block(XoverFilter *bands,
                                 int32_t * __restrict samples,
                                 uint32_t sample_count);
#endif

#ifdef __cplusplus
}
#endif

#endif // CROSSOVER_H
