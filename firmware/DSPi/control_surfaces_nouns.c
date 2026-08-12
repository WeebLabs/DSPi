/*
 * control_surfaces_nouns.c; the noun catalog for Control Surfaces.
 *
 * One row per noun in cs_noun_table (kind, unit, action mask, range,
 * target addressing), one case per noun in cs_noun_get (live read in
 * natural units) and cs_noun_dispatch (map an absolute target onto the
 * existing vendor command a host would send).  The engine in
 * control_surfaces.c is unit- and noun-agnostic; everything
 * parameter-specific lives here.
 *
 * Main-loop context only.  Live reads touch main-loop-owned state (or
 * volatile globals published by the audio path); dispatches go through
 * vendor_dispatch_set/get with CTRL_SOURCE_GPIO, so validation, deferred
 * apply, and PARAM_SRC_GPIO notifications come from the shared handlers.
 */

#include "control_surfaces.h"
#include "control_surfaces_display.h"
#include "config.h"
#include "vendor_commands.h"
#include "usb_audio.h"
#include "audio_input.h"
#include "flash_storage.h"
#include "spdif_input.h"
#include "lg_sound_sync.h"
#include "siggen.h"
#include "crossover.h"
#include "audio_pipeline.h"
#include "dsp_pipeline.h"
#include "psybass.h"
#include "upmix.h"
#include "loudness.h"
#if PICO_RP2350
#include "adat_output.h"
#endif

#include <math.h>

// USB input sync state (usb_audio.c); true while the host stream is running.
extern volatile bool sync_started;

// PEQ filter types run 0..FILTER_HIGHSHELF1 contiguously (crossover types
// start at FILTER_XOVER_FIRST = 32 and are not front-panel material).
// FILTER_LINKWITZ_TRANSFORM (11) is deliberately excluded from cycling: its
// gain_db field carries a frequency and it needs a 4th parameter no CS noun
// edits; it is host-configurable only.
#define CS_PEQ_TYPE_COUNT  (FILTER_HIGHSHELF1 + 1)

// ---------------------------------------------------------------------------
// Action-mask groups
// ---------------------------------------------------------------------------
#define CS_CONT_RW  (CS_ACT_BIT(CS_ACT_ADJUST) | CS_ACT_BIT(CS_ACT_STEP) | \
                     CS_ACT_BIT(CS_ACT_INC) | CS_ACT_BIT(CS_ACT_DEC) | \
                     CS_ACT_BIT(CS_ACT_SET) | CS_ACT_BIT(CS_ACT_IND_ABOVE) | \
                     CS_ACT_BIT(CS_ACT_IND_LEVEL))
#define CS_BOOL_RW  (CS_ACT_BIT(CS_ACT_TOGGLE) | CS_ACT_BIT(CS_ACT_SET) | \
                     CS_ACT_BIT(CS_ACT_FOLLOW) | CS_ACT_BIT(CS_ACT_MOMENTARY) | \
                     CS_ACT_BIT(CS_ACT_IND_EQUALS))
#define CS_ENUM_RW  (CS_ACT_BIT(CS_ACT_STEP) | CS_ACT_BIT(CS_ACT_INC) | \
                     CS_ACT_BIT(CS_ACT_DEC) | CS_ACT_BIT(CS_ACT_SET) | \
                     CS_ACT_BIT(CS_ACT_IND_EQUALS))
#define CS_BOOL_RO  CS_ACT_BIT(CS_ACT_IND_EQUALS)
#define CS_ENUM_RO  CS_ACT_BIT(CS_ACT_IND_EQUALS)
#define CS_CONT_RO  (CS_ACT_BIT(CS_ACT_IND_ABOVE) | CS_ACT_BIT(CS_ACT_IND_LEVEL))

// ADAT output exists only on RP2350; an empty action mask marks the noun
// unavailable so bindings are rejected and hosts grey it out, while the
// noun keeps its wire value on both platforms.
#if PICO_RP2350
#define CS_ADAT_ACTS  CS_BOOL_RO
#else
#define CS_ADAT_ACTS  0
#endif

// The stereo upmixer is RP2350-only; same greyed-out pattern as ADAT.  The
// range/enum literals in the RP2040 rows mirror upmix.h (whose limits are
// compiled out there) so both platforms publish identical descriptors.
#if PICO_RP2350
#define CS_UPMIX_ACTS(group)     group
#define CS_UPMIX_CENTER_MODES    (UPMIX_CENTER_OFF + 1)
#define CS_UPMIX_SURROUND_MODES  (UPMIX_SURROUND_ADAPTIVE + 1)
#define CS_UPMIX_PRES_MIN        UPMIX_PRESENCE_MIN
#define CS_UPMIX_PRES_MAX        UPMIX_PRESENCE_MAX
#else
#define CS_UPMIX_ACTS(group)     0
#define CS_UPMIX_CENTER_MODES    3       // Passive / Logic / Off
#define CS_UPMIX_SURROUND_MODES  3       // Off / Passive / Logic
#define CS_UPMIX_PRES_MIN        -12.0f  // mirror UPMIX_PRESENCE_MIN/MAX
#define CS_UPMIX_PRES_MAX         12.0f
#endif

// Output-delay front-panel ceiling: the full delay ring at 48 kHz, floored
// to whole ms (21 ms on RP2040, 42 ms on RP2350), pre-encoded as 8.8.  At
// 96 kHz the pipeline clamps in samples, same as a host-set delay; the
// stored ms value round-trips unclamped.
#define CS_DELAY_MAX_MS_Q8  ((int16_t)((MAX_DELAY_SAMPLES / 48) * 256))

// Loudness intensity accepts up to 200 % over the wire, but the int16 8.8
// value fields top out at 127.99, so the front-panel span stops there.
#define CS_LOUDNESS_INTENSITY_MAX  127.0f

// Q(8.8) helper for table literals; rounds to nearest (a plain cast would
// truncate, encoding Q 0.1 as 25 instead of the documented 26).  Must stay a
// constant expression: the table below is a static initializer.
#define Q8(x)  ((int16_t)((x) * 256 + ((x) >= 0 ? 0.5f : -0.5f)))

// ---------------------------------------------------------------------------
// The catalog.  Ranges are conservative front-panel spans, inside whatever
// the underlying handler accepts.
// ---------------------------------------------------------------------------
const CsNounDesc cs_noun_table[CS_NOUN_COUNT] = {
    [CS_NOUN_USER_VOLUME]     = { CS_KIND_CONTINUOUS, 0, CS_CONT_RW,
                                  (int16_t)(-CENTER_VOLUME_INDEX * 256), 0,
                                  CS_UNIT_DB, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_MASTER_VOLUME]   = { CS_KIND_CONTINUOUS, 0, CS_CONT_RW,
                                  Q8(-127), 0, CS_UNIT_DB, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_USER_MUTE]       = { CS_KIND_BOOL, 0, CS_BOOL_RW, 0, 0,
                                  CS_UNIT_NONE, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_LOUDNESS]        = { CS_KIND_BOOL, 0, CS_BOOL_RW, 0, 0,
                                  CS_UNIT_NONE, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_CROSSFEED]       = { CS_KIND_BOOL, 0, CS_BOOL_RW, 0, 0,
                                  CS_UNIT_NONE, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_LEVELLER]        = { CS_KIND_BOOL, 0, CS_BOOL_RW, 0, 0,
                                  CS_UNIT_NONE, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_PRESET]          = { CS_KIND_ENUM, PRESET_SLOTS, CS_ENUM_RW, 0, 0,
                                  CS_UNIT_NONE, CS_TARGET_NONE, 0, CS_NDF_DEFERRED },
    [CS_NOUN_INPUT_SOURCE]    = { CS_KIND_ENUM, INPUT_SOURCE_MAX + 1, CS_ENUM_RW, 0, 0,
                                  CS_UNIT_NONE, CS_TARGET_NONE, 0, CS_NDF_DEFERRED },
    [CS_NOUN_CLIP]            = { CS_KIND_BOOL, 0,
                                  CS_ACT_BIT(CS_ACT_TRIGGER) | CS_ACT_BIT(CS_ACT_IND_EQUALS),
                                  0, 0, CS_UNIT_NONE, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_EQ_BYPASS]       = { CS_KIND_BOOL, 0, CS_BOOL_RW, 0, 0,
                                  CS_UNIT_NONE, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_LG_SYNC]         = { CS_KIND_BOOL, 0, CS_BOOL_RW, 0, 0,
                                  CS_UNIT_NONE, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_CROSSFEED_PRESET] = { CS_KIND_ENUM, CROSSFEED_PRESET_CUSTOM + 1, CS_ENUM_RW,
                                  0, 0, CS_UNIT_NONE, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_CROSSFEED_ITD]   = { CS_KIND_BOOL, 0, CS_BOOL_RW, 0, 0,
                                  CS_UNIT_NONE, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_LEVELLER_AMOUNT] = { CS_KIND_CONTINUOUS, 0, CS_CONT_RW,
                                  0, Q8(100), CS_UNIT_PERCENT, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_LEVELLER_SPEED]  = { CS_KIND_ENUM, LEVELLER_SPEED_COUNT, CS_ENUM_RW, 0, 0,
                                  CS_UNIT_NONE, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_LEVELLER_LOOKAHEAD] = { CS_KIND_BOOL, 0, CS_BOOL_RW, 0, 0,
                                  CS_UNIT_NONE, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_PREAMP]          = { CS_KIND_CONTINUOUS, 0, CS_CONT_RW,
                                  Q8(-24), Q8(24), CS_UNIT_DB,
                                  CS_TARGET_INPUT_CH, NUM_INPUT_CHANNELS, 0 },
    [CS_NOUN_OUTPUT_GAIN]     = { CS_KIND_CONTINUOUS, 0, CS_CONT_RW,
                                  Q8(-60), Q8(12), CS_UNIT_DB,
                                  CS_TARGET_OUTPUT_CH, NUM_OUTPUT_CHANNELS, 0 },
    [CS_NOUN_OUTPUT_MUTE]     = { CS_KIND_BOOL, 0, CS_BOOL_RW, 0, 0,
                                  CS_UNIT_NONE, CS_TARGET_OUTPUT_CH, NUM_OUTPUT_CHANNELS, 0 },
    [CS_NOUN_OUTPUT_ENABLE]   = { CS_KIND_BOOL, 0, CS_BOOL_RW, 0, 0,
                                  CS_UNIT_NONE, CS_TARGET_OUTPUT_CH, NUM_OUTPUT_CHANNELS, 0 },
    [CS_NOUN_FILTER_FREQ]     = { CS_KIND_CONTINUOUS, 0, CS_CONT_RW,
                                  20, 20000, CS_UNIT_HZ,
                                  CS_TARGET_DSP_BAND, NUM_CHANNELS, CS_NDF_DEFERRED },
    [CS_NOUN_FILTER_GAIN]     = { CS_KIND_CONTINUOUS, 0, CS_CONT_RW,
                                  Q8(-24), Q8(24), CS_UNIT_DB,
                                  CS_TARGET_DSP_BAND, NUM_CHANNELS, CS_NDF_DEFERRED },
    [CS_NOUN_FILTER_Q]        = { CS_KIND_CONTINUOUS, 0, CS_CONT_RW,
                                  Q8(0.1f), Q8(10), CS_UNIT_Q,
                                  CS_TARGET_DSP_BAND, NUM_CHANNELS, CS_NDF_DEFERRED },
    [CS_NOUN_FILTER_TYPE]     = { CS_KIND_ENUM, CS_PEQ_TYPE_COUNT, CS_ENUM_RW, 0, 0,
                                  CS_UNIT_NONE, CS_TARGET_DSP_BAND, NUM_CHANNELS,
                                  CS_NDF_DEFERRED },
    [CS_NOUN_FILTER_BYPASS]   = { CS_KIND_BOOL, 0, CS_BOOL_RW, 0, 0,
                                  CS_UNIT_NONE, CS_TARGET_DSP_BAND, NUM_CHANNELS,
                                  CS_NDF_DEFERRED },
    [CS_NOUN_SIGGEN]          = { CS_KIND_BOOL, 0, CS_BOOL_RW, 0, 0,
                                  CS_UNIT_NONE, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_DAC_MUTE_TEST]   = { CS_KIND_BOOL, 0, CS_ACT_BIT(CS_ACT_TRIGGER), 0, 0,
                                  CS_UNIT_NONE, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_CLIP_CH]         = { CS_KIND_BOOL, 0, CS_BOOL_RO, 0, 0,
                                  CS_UNIT_NONE, CS_TARGET_DSP_CH, NUM_CHANNELS, 0 },
    [CS_NOUN_LEVEL]           = { CS_KIND_CONTINUOUS, 0, CS_CONT_RO,
                                  Q8(-60), 0, CS_UNIT_DB,
                                  CS_TARGET_DSP_CH, NUM_CHANNELS, 0 },
    [CS_NOUN_SPDIF_LOCK]      = { CS_KIND_BOOL, 0, CS_BOOL_RO, 0, 0,
                                  CS_UNIT_NONE, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_SAMPLE_RATE]     = { CS_KIND_ENUM, 3, CS_ENUM_RO, 0, 0,
                                  CS_UNIT_NONE, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_USB_STREAMING]   = { CS_KIND_BOOL, 0, CS_BOOL_RO, 0, 0,
                                  CS_UNIT_NONE, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_ADAT_ACTIVE]     = { CS_KIND_BOOL, 0, CS_ADAT_ACTS, 0, 0,
                                  CS_UNIT_NONE, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_LG_PRESENT]      = { CS_KIND_BOOL, 0, CS_BOOL_RO, 0, 0,
                                  CS_UNIT_NONE, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_LG_MUTED]        = { CS_KIND_BOOL, 0, CS_BOOL_RO, 0, 0,
                                  CS_UNIT_NONE, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_UPMIX]           = { CS_KIND_BOOL, 0, CS_UPMIX_ACTS(CS_BOOL_RW), 0, 0,
                                  CS_UNIT_NONE, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_UPMIX_CENTER_MODE] = { CS_KIND_ENUM, CS_UPMIX_CENTER_MODES,
                                  CS_UPMIX_ACTS(CS_ENUM_RW), 0, 0,
                                  CS_UNIT_NONE, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_UPMIX_SURROUND_MODE] = { CS_KIND_ENUM, CS_UPMIX_SURROUND_MODES,
                                  CS_UPMIX_ACTS(CS_ENUM_RW), 0, 0,
                                  CS_UNIT_NONE, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_UPMIX_STRENGTH]  = { CS_KIND_CONTINUOUS, 0, CS_UPMIX_ACTS(CS_CONT_RW),
                                  0, Q8(100), CS_UNIT_PERCENT, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_UPMIX_WIDTH]     = { CS_KIND_CONTINUOUS, 0, CS_UPMIX_ACTS(CS_CONT_RW),
                                  0, Q8(100), CS_UNIT_PERCENT, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_UPMIX_PRESENCE]  = { CS_KIND_CONTINUOUS, 0, CS_UPMIX_ACTS(CS_CONT_RW),
                                  Q8(CS_UPMIX_PRES_MIN), Q8(CS_UPMIX_PRES_MAX),
                                  CS_UNIT_DB, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_PSYBASS]         = { CS_KIND_BOOL, 0, CS_BOOL_RW, 0, 0,
                                  CS_UNIT_NONE, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_PSYBASS_CUTOFF]  = { CS_KIND_CONTINUOUS, 0, CS_CONT_RW,
                                  (int16_t)PSYBASS_CUTOFF_MIN, (int16_t)PSYBASS_CUTOFF_MAX,
                                  CS_UNIT_HZ, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_PSYBASS_HARMONICS] = { CS_KIND_CONTINUOUS, 0, CS_CONT_RW,
                                  Q8(PSYBASS_HARMONICS_MIN), Q8(PSYBASS_HARMONICS_MAX),
                                  CS_UNIT_DB, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_PSYBASS_DRIVE]   = { CS_KIND_CONTINUOUS, 0, CS_CONT_RW,
                                  Q8(PSYBASS_DRIVE_MIN), Q8(PSYBASS_DRIVE_MAX),
                                  CS_UNIT_DB, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_PSYBASS_CHARACTER] = { CS_KIND_CONTINUOUS, 0, CS_CONT_RW,
                                  Q8(PSYBASS_CHARACTER_MIN), Q8(PSYBASS_CHARACTER_MAX),
                                  CS_UNIT_PERCENT, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_PSYBASS_ORIGINAL] = { CS_KIND_CONTINUOUS, 0, CS_CONT_RW,
                                  Q8(PSYBASS_ORIGINAL_MIN), Q8(PSYBASS_ORIGINAL_MAX),
                                  CS_UNIT_DB, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_OUTPUT_DELAY]    = { CS_KIND_CONTINUOUS, 0, CS_CONT_RW,
                                  0, CS_DELAY_MAX_MS_Q8, CS_UNIT_MS,
                                  CS_TARGET_OUTPUT_CH, NUM_OUTPUT_CHANNELS, 0 },
    [CS_NOUN_PRESET_RELOAD]   = { CS_KIND_BOOL, 0, CS_ACT_BIT(CS_ACT_TRIGGER), 0, 0,
                                  CS_UNIT_NONE, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_LOUDNESS_SPL]    = { CS_KIND_CONTINUOUS, 0, CS_CONT_RW,
                                  Q8(LOUDNESS_REF_SPL_MIN), Q8(LOUDNESS_REF_SPL_MAX),
                                  CS_UNIT_DB, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_LOUDNESS_INTENSITY] = { CS_KIND_CONTINUOUS, 0, CS_CONT_RW,
                                  Q8(LOUDNESS_INTENSITY_MIN), Q8(CS_LOUDNESS_INTENSITY_MAX),
                                  CS_UNIT_PERCENT, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_INPUT_LEVEL_MAX] = { CS_KIND_CONTINUOUS, 0, CS_CONT_RO,
                                  Q8(-60), 0, CS_UNIT_DB,
                                  CS_TARGET_NONE, 0, 0 },
    // SET fires macro `value`; IND_EQUALS lights while it runs.  Stepping
    // through macros would fire them while browsing, so INC/DEC stay out.
    [CS_NOUN_MACRO]           = { CS_KIND_ENUM, CS_MAX_MACROS,
                                  CS_ACT_BIT(CS_ACT_SET) | CS_ACT_BIT(CS_ACT_IND_EQUALS),
                                  0, 0, CS_UNIT_NONE, CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_CPU_LOAD]        = { CS_KIND_CONTINUOUS, 0, CS_CONT_RO,
                                  0, Q8(100), CS_UNIT_PERCENT,
                                  CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_DISPLAY_PAGE]    = { CS_KIND_ENUM, CS_MAX_DISPLAY_PAGES,
                                  CS_ENUM_RW, 0, 0, CS_UNIT_NONE,
                                  CS_TARGET_NONE, 0, 0 },
    [CS_NOUN_DISPLAY_EDIT]    = { CS_KIND_BOOL, 0, CS_BOOL_RW, 0, 0,
                                  CS_UNIT_NONE, CS_TARGET_NONE, 0, 0 },
    // Virtual: the engine resolves the shown page at event time; scalar
    // fields must be zero (enforced in cs_validate / cs_validate_ir_cmd).
    [CS_NOUN_PAGE_VALUE]      = { CS_KIND_ENUM, 1,
                                  CS_ACT_BIT(CS_ACT_STEP) | CS_ACT_BIT(CS_ACT_INC) |
                                  CS_ACT_BIT(CS_ACT_DEC) | CS_ACT_BIT(CS_ACT_TOGGLE),
                                  0, 0, CS_UNIT_NONE, CS_TARGET_NONE, 0, 0 },
};

// ---------------------------------------------------------------------------
// Filter recipe access (FILTER_* nouns).  PEQ bands live in filter_recipes,
// crossover bands (XOVER_BAND_BASE..+3) in xover_recipes; both are applied
// deferred by the EQ handler, so the engine steps these nouns from a shadow.
// ---------------------------------------------------------------------------

static const EqParamPacket *cs_filter_recipe(uint8_t ch, uint8_t band) {
    if (band >= XOVER_BAND_BASE)
        return &xover_recipes[ch][band - XOVER_BAND_BASE];
    return &filter_recipes[ch][band];
}

// Read-modify-write one field of a filter through REQ_SET_EQ_PARAM.  The
// packet is built from the live recipe so the untouched fields round-trip.
// The EQ handlers share one pending_packet slot; while an apply is still
// owed, report BUSY so the engine retries instead of overwriting it.
static bool cs_filter_rmw(uint8_t noun, uint8_t ch, uint8_t band, float value) {
    if (eq_update_pending) return false;
    EqParamPacket p = *cs_filter_recipe(ch, band);
    p.channel = ch;
    p.band = band;
    p.bypass = (p.bypass == 1) ? 1 : 0;
    switch (noun) {
        case CS_NOUN_FILTER_FREQ:   p.freq = value;            break;
        // gain_db holds fp in Hz for LT; a dB clamp would corrupt it, so
        // report the adjust handled without touching the recipe.
        case CS_NOUN_FILTER_GAIN:
            if (p.type == FILTER_LINKWITZ_TRANSFORM) return true;
            p.gain_db = value;         break;
        case CS_NOUN_FILTER_Q:      p.Q = value;               break;
        case CS_NOUN_FILTER_TYPE:   p.type = (uint8_t)value;   break;
        case CS_NOUN_FILTER_BYPASS: p.bypass = (value >= 0.5f) ? 1 : 0; break;
        default: return true;
    }
    CtrlDispatchResult r = vendor_dispatch_set(CTRL_SOURCE_GPIO, REQ_SET_EQ_PARAM,
                                               0, 0, (const uint8_t *)&p, sizeof(p));
    return r != CTRL_DISPATCH_BUSY;
}

// ---------------------------------------------------------------------------
// Live reads
// ---------------------------------------------------------------------------

float cs_noun_get(uint8_t noun, uint8_t target, uint8_t index) {
    switch (noun) {
        case CS_NOUN_USER_VOLUME:   return (float)audio_state.volume * (1.0f / 256.0f);
        case CS_NOUN_MASTER_VOLUME: {
            float db = master_volume_db;   // -128 mute sentinel steps up from the floor
            return (db < MASTER_VOL_MIN_DB) ? MASTER_VOL_MIN_DB : db;
        }
        case CS_NOUN_USER_MUTE:     return user_mute ? 1.0f : 0.0f;
        case CS_NOUN_LOUDNESS:      return loudness_enabled ? 1.0f : 0.0f;
        case CS_NOUN_CROSSFEED:     return crossfeed_config.enabled ? 1.0f : 0.0f;
        case CS_NOUN_LEVELLER:      return leveller_config.enabled ? 1.0f : 0.0f;
        case CS_NOUN_PRESET:        return (float)preset_get_active();
        case CS_NOUN_INPUT_SOURCE:  return (float)active_input_source;
        case CS_NOUN_CLIP:          return global_status.clip_flags ? 1.0f : 0.0f;
        case CS_NOUN_EQ_BYPASS:     return bypass_master_eq ? 1.0f : 0.0f;
        case CS_NOUN_LG_SYNC: {
            LgSoundSyncStatus st;
            lg_sound_sync_get_status(&st);
            return st.enabled ? 1.0f : 0.0f;
        }
        case CS_NOUN_CROSSFEED_PRESET: return (float)crossfeed_config.preset;
        case CS_NOUN_CROSSFEED_ITD:    return crossfeed_config.itd_enabled ? 1.0f : 0.0f;
        case CS_NOUN_LEVELLER_AMOUNT:  return leveller_config.amount;
        case CS_NOUN_LEVELLER_SPEED:   return (float)leveller_config.speed;
        case CS_NOUN_LEVELLER_LOOKAHEAD: return leveller_config.lookahead ? 1.0f : 0.0f;
        case CS_NOUN_PREAMP:        return global_preamp_db[target];
        case CS_NOUN_OUTPUT_GAIN:   return matrix_mixer.outputs[target].gain_db;
        case CS_NOUN_OUTPUT_MUTE:   return matrix_mixer.outputs[target].mute ? 1.0f : 0.0f;
        case CS_NOUN_OUTPUT_ENABLE: return matrix_mixer.outputs[target].enabled ? 1.0f : 0.0f;
        case CS_NOUN_FILTER_FREQ:   return cs_filter_recipe(target, index)->freq;
        // LT bands report fp (Hz) here since gain_db is repurposed; writes are blocked.
        case CS_NOUN_FILTER_GAIN:   return cs_filter_recipe(target, index)->gain_db;
        case CS_NOUN_FILTER_Q:      return cs_filter_recipe(target, index)->Q;
        case CS_NOUN_FILTER_TYPE:   return (float)cs_filter_recipe(target, index)->type;
        case CS_NOUN_FILTER_BYPASS:
            return (cs_filter_recipe(target, index)->bypass == 1) ? 1.0f : 0.0f;
        case CS_NOUN_SIGGEN: {
            SiggenStatus st;
            siggen_get_status(&st);
            return (st.state != SIGGEN_STATE_IDLE) ? 1.0f : 0.0f;
        }
        case CS_NOUN_DAC_MUTE_TEST: return 0.0f;
        case CS_NOUN_CLIP_CH:
            return (global_status.clip_flags & (1u << target)) ? 1.0f : 0.0f;
        case CS_NOUN_LEVEL: {
            uint16_t pk = global_status.peaks[target];
            if (pk == 0) return -60.0f;
            float db = 20.0f * log10f((float)pk * (1.0f / 32767.0f));
            return (db < -60.0f) ? -60.0f : db;
        }
        case CS_NOUN_INPUT_LEVEL_MAX: {
            // peaks[] freezes at its last value when block delivery stops
            // (nothing clears it), so a dead source must read as silence or
            // an amp-trigger LED would latch on forever.
            if (!pipeline_producer_is_streaming()) return -60.0f;
            // Inputs occupy peaks[0..n); inactive channels read 0, so the
            // max over the active count is the live signal-presence level.
            uint16_t pk = 0;
            uint8_t n = active_input_channel_count();
            for (uint8_t ch = 0; ch < n; ch++)
                if (global_status.peaks[ch] > pk) pk = global_status.peaks[ch];
            if (pk == 0) return -60.0f;
            float db = 20.0f * log10f((float)pk * (1.0f / 32767.0f));
            return (db < -60.0f) ? -60.0f : db;
        }
        case CS_NOUN_SPDIF_LOCK:
            return (spdif_input_get_state() == SPDIF_INPUT_LOCKED) ? 1.0f : 0.0f;
        case CS_NOUN_SAMPLE_RATE:
            // Enum index for IND_EQUALS; an unrecognised rate matches nothing.
            switch (audio_state.freq) {
                case 44100: return 0.0f;
                case 48000: return 1.0f;
                case 96000: return 2.0f;
                default:    return 255.0f;
            }
        case CS_NOUN_USB_STREAMING:
            return (active_input_source == INPUT_SOURCE_USB && sync_started) ? 1.0f : 0.0f;
#if PICO_RP2350
        case CS_NOUN_ADAT_ACTIVE: {
            AdatStatus st;
            adat_output_get_status(&st);
            return (st.active && st.rate_ok) ? 1.0f : 0.0f;
        }
#else
        case CS_NOUN_ADAT_ACTIVE: return 0.0f;
#endif
        case CS_NOUN_LG_PRESENT:
        case CS_NOUN_LG_MUTED: {
            LgSoundSyncStatus st;
            lg_sound_sync_get_status(&st);
            if (noun == CS_NOUN_LG_PRESENT) return st.present ? 1.0f : 0.0f;
            return (st.present && st.muted) ? 1.0f : 0.0f;
        }
#if PICO_RP2350
        case CS_NOUN_UPMIX:               return upmix_config.enabled ? 1.0f : 0.0f;
        case CS_NOUN_UPMIX_CENTER_MODE:   return (float)upmix_config.center_mode;
        case CS_NOUN_UPMIX_SURROUND_MODE: return (float)upmix_config.surround_mode;
        case CS_NOUN_UPMIX_STRENGTH:      return upmix_config.strength_pct;
        case CS_NOUN_UPMIX_WIDTH:         return upmix_config.center_width_pct;
        case CS_NOUN_UPMIX_PRESENCE:      return upmix_config.presence_db;
#else
        case CS_NOUN_UPMIX:
        case CS_NOUN_UPMIX_CENTER_MODE:
        case CS_NOUN_UPMIX_SURROUND_MODE:
        case CS_NOUN_UPMIX_STRENGTH:
        case CS_NOUN_UPMIX_WIDTH:
        case CS_NOUN_UPMIX_PRESENCE:      return 0.0f;
#endif
        case CS_NOUN_PSYBASS:           return psybass_config.enabled ? 1.0f : 0.0f;
        case CS_NOUN_PSYBASS_CUTOFF:    return psybass_config.cutoff_hz;
        case CS_NOUN_PSYBASS_HARMONICS: return psybass_config.harmonics_db;
        case CS_NOUN_PSYBASS_DRIVE:     return psybass_config.drive_db;
        case CS_NOUN_PSYBASS_CHARACTER: return psybass_config.character_pct;
        case CS_NOUN_PSYBASS_ORIGINAL:  return psybass_config.original_db;
        case CS_NOUN_OUTPUT_DELAY:      return matrix_mixer.outputs[target].delay_ms;
        case CS_NOUN_PRESET_RELOAD:     return 0.0f;
        // Both read live: the SET handler stores the value immediately and
        // only the table rebuild is deferred to the main loop.
        case CS_NOUN_LOUDNESS_SPL:       return loudness_ref_spl;
        case CS_NOUN_LOUDNESS_INTENSITY: return loudness_intensity_pct;
        // 255 while idle so an IND_EQUALS comparand never matches.
        case CS_NOUN_MACRO:              return (float)cs_macro_running_index();
        case CS_NOUN_CPU_LOAD:           return (float)global_status.cpu0_load;
        // 255 with no shown page, same never-matches convention.
        case CS_NOUN_DISPLAY_PAGE:       return (float)cs_display_current_page();
        case CS_NOUN_DISPLAY_EDIT:       return cs_display_edit_armed() ? 1.0f : 0.0f;
        default: return 0.0f;
    }
}

// ---------------------------------------------------------------------------
// Dispatch
// ---------------------------------------------------------------------------

bool cs_noun_dispatch(uint8_t noun, uint8_t target, uint8_t index, float value) {
    const uint8_t *rd; uint16_t rl;
    CtrlDispatchResult r;
    switch (noun) {
        case CS_NOUN_USER_VOLUME:
        case CS_NOUN_MASTER_VOLUME:
        case CS_NOUN_LEVELLER_AMOUNT:
        case CS_NOUN_LOUDNESS_SPL:
        case CS_NOUN_LOUDNESS_INTENSITY: {
            float f = value;
            uint8_t req = (noun == CS_NOUN_USER_VOLUME)   ? REQ_SET_USER_VOLUME
                        : (noun == CS_NOUN_MASTER_VOLUME) ? REQ_SET_MASTER_VOLUME
                        : (noun == CS_NOUN_LEVELLER_AMOUNT) ? REQ_SET_LEVELLER_AMOUNT
                        : (noun == CS_NOUN_LOUDNESS_SPL)  ? REQ_SET_LOUDNESS_REF
                                                          : REQ_SET_LOUDNESS_INTENSITY;
            r = vendor_dispatch_set(CTRL_SOURCE_GPIO, req, 0, 0,
                                    (const uint8_t *)&f, sizeof(f));
            break;
        }
        case CS_NOUN_PREAMP:
        case CS_NOUN_OUTPUT_GAIN:
        case CS_NOUN_OUTPUT_DELAY: {
            float f = value;
            uint8_t req = (noun == CS_NOUN_PREAMP)      ? REQ_SET_PREAMP_CH
                        : (noun == CS_NOUN_OUTPUT_GAIN) ? REQ_SET_OUTPUT_GAIN
                                                        : REQ_SET_OUTPUT_DELAY;
            r = vendor_dispatch_set(CTRL_SOURCE_GPIO, req, target, 0,
                                    (const uint8_t *)&f, sizeof(f));
            break;
        }
        case CS_NOUN_PSYBASS_CUTOFF:
        case CS_NOUN_PSYBASS_HARMONICS:
        case CS_NOUN_PSYBASS_DRIVE:
        case CS_NOUN_PSYBASS_CHARACTER:
        case CS_NOUN_PSYBASS_ORIGINAL: {
            // Per-parameter float SETs; nouns are contiguous from CUTOFF.
            static const uint8_t psybass_req[] = {
                REQ_SET_PSYBASS_CUTOFF, REQ_SET_PSYBASS_HARMONICS,
                REQ_SET_PSYBASS_DRIVE, REQ_SET_PSYBASS_CHARACTER,
                REQ_SET_PSYBASS_ORIGINAL,
            };
            float f = value;
            r = vendor_dispatch_set(CTRL_SOURCE_GPIO,
                                    psybass_req[noun - CS_NOUN_PSYBASS_CUTOFF],
                                    0, 0, (const uint8_t *)&f, sizeof(f));
            break;
        }
#if PICO_RP2350
        case CS_NOUN_UPMIX:
        case CS_NOUN_UPMIX_CENTER_MODE:
        case CS_NOUN_UPMIX_SURROUND_MODE:
        case CS_NOUN_UPMIX_STRENGTH:
        case CS_NOUN_UPMIX_WIDTH:
        case CS_NOUN_UPMIX_PRESENCE: {
            // One float-per-parameter command; wValue selects the parameter.
            // Nouns are contiguous from UPMIX; enable/mode floats are rounded
            // and clamped by the handler.
            static const uint8_t upmix_param[] = {
                UPMIX_PARAM_ENABLED, UPMIX_PARAM_CENTER_MODE,
                UPMIX_PARAM_SURROUND_MODE, UPMIX_PARAM_STRENGTH,
                UPMIX_PARAM_CENTER_WIDTH, UPMIX_PARAM_PRESENCE,
            };
            float f = value;
            r = vendor_dispatch_set(CTRL_SOURCE_GPIO, REQ_UPMIX_SET_PARAM,
                                    upmix_param[noun - CS_NOUN_UPMIX], 0,
                                    (const uint8_t *)&f, sizeof(f));
            break;
        }
#endif
        case CS_NOUN_PRESET_RELOAD:
            // Reload the active preset from flash, discarding unsaved live
            // edits; the same deferred, pipeline-safe path the PRESET noun
            // takes.  Device-global state (master volume in independent
            // mode, output config, CS bindings) is untouched.
            r = vendor_dispatch_get(CTRL_SOURCE_GPIO, REQ_PRESET_LOAD,
                                    (uint16_t)preset_get_active(), 0, 1, &rd, &rl);
            break;
        case CS_NOUN_PRESET:
            // Same deferred, pipeline-safe path a host load takes.
            r = vendor_dispatch_get(CTRL_SOURCE_GPIO, REQ_PRESET_LOAD,
                                    (uint16_t)value, 0, 1, &rd, &rl);
            break;
        case CS_NOUN_CLIP:
            r = vendor_dispatch_get(CTRL_SOURCE_GPIO, REQ_CLEAR_CLIPS,
                                    0, 0, 4, &rd, &rl);
            break;
        case CS_NOUN_FILTER_FREQ:
        case CS_NOUN_FILTER_GAIN:
        case CS_NOUN_FILTER_Q:
        case CS_NOUN_FILTER_TYPE:
            return cs_filter_rmw(noun, target, index, value);
        case CS_NOUN_FILTER_BYPASS: {
            // Direct band-bypass command; no RMW needed, but it funnels
            // through the same pending_packet as REQ_SET_EQ_PARAM.
            if (eq_update_pending) return false;
            uint8_t v = (value >= 0.5f) ? 1 : 0;
            r = vendor_dispatch_set(CTRL_SOURCE_GPIO, REQ_SET_BAND_BYPASS,
                                    (uint16_t)((target << 8) | index), 0, &v, 1);
            break;
        }
        case CS_NOUN_SIGGEN:
            r = vendor_dispatch_get(CTRL_SOURCE_GPIO, REQ_SIGGEN_CONTROL,
                                    (value >= 0.5f) ? SIGGEN_CTL_START : SIGGEN_CTL_STOP,
                                    0, 1, &rd, &rl);
            break;
        case CS_NOUN_MACRO:
            // Direct call, not a vendor command: the sequencer itself then
            // dispatches every step through the shared command surface.
            (void)control_surfaces_macro_fire((uint8_t)value);
            return true;
        case CS_NOUN_DISPLAY_PAGE:
            cs_display_select_page((uint8_t)value);
            return true;
        case CS_NOUN_DISPLAY_EDIT:
            cs_display_set_edit(value >= 0.5f);
            return true;
        case CS_NOUN_PAGE_VALUE:
            // Never dispatched directly; the engine routes it at the event
            // sources.  A stray dispatch is a safe no-op.
            return true;
        case CS_NOUN_DAC_MUTE_TEST:
            r = vendor_dispatch_get(CTRL_SOURCE_GPIO, REQ_TEST_DAC_HW_MUTE,
                                    0, 0, 1, &rd, &rl);
            break;
        case CS_NOUN_OUTPUT_MUTE:
        case CS_NOUN_OUTPUT_ENABLE: {
            uint8_t v = (value >= 0.5f) ? 1 : 0;
            uint8_t req = (noun == CS_NOUN_OUTPUT_MUTE) ? REQ_SET_OUTPUT_MUTE
                                                        : REQ_SET_OUTPUT_ENABLE;
            r = vendor_dispatch_set(CTRL_SOURCE_GPIO, req, target, 0, &v, 1);
            break;
        }
        default: {
            // Single-byte SETs (bool / enum nouns).
            uint8_t v = (uint8_t)value;
            uint8_t req;
            switch (noun) {
                case CS_NOUN_USER_MUTE:        req = REQ_SET_USER_MUTE;          break;
                case CS_NOUN_LOUDNESS:         req = REQ_SET_LOUDNESS;           break;
                case CS_NOUN_CROSSFEED:        req = REQ_SET_CROSSFEED;          break;
                case CS_NOUN_LEVELLER:         req = REQ_SET_LEVELLER_ENABLE;    break;
                case CS_NOUN_INPUT_SOURCE:     req = REQ_SET_INPUT_SOURCE;       break;
                case CS_NOUN_EQ_BYPASS:        req = REQ_SET_BYPASS;             break;
                case CS_NOUN_LG_SYNC:          req = REQ_SET_LG_SOUND_SYNC_ENABLE; break;
                case CS_NOUN_CROSSFEED_PRESET: req = REQ_SET_CROSSFEED_PRESET;   break;
                case CS_NOUN_CROSSFEED_ITD:    req = REQ_SET_CROSSFEED_ITD;      break;
                case CS_NOUN_LEVELLER_SPEED:   req = REQ_SET_LEVELLER_SPEED;     break;
                case CS_NOUN_LEVELLER_LOOKAHEAD: req = REQ_SET_LEVELLER_LOOKAHEAD; break;
                case CS_NOUN_PSYBASS:          req = REQ_SET_PSYBASS;            break;
                default:                       return true;   // read-only noun
            }
            r = vendor_dispatch_set(CTRL_SOURCE_GPIO, req, 0, 0, &v, 1);
            break;
        }
    }
    return r != CTRL_DISPATCH_BUSY;
}

// ---------------------------------------------------------------------------
// Target validation
// ---------------------------------------------------------------------------

// One resolved channel (+ band for DSP_BAND nouns); the per-member check
// behind grouped bindings and the core of the single-target check below.
uint8_t cs_noun_validate_target_ch(uint8_t noun, uint8_t ch, uint8_t index) {
    const CsNounDesc *nd = &cs_noun_table[noun];
    switch (nd->target_kind) {
        case CS_TARGET_INPUT_CH:
        case CS_TARGET_OUTPUT_CH:
        case CS_TARGET_DSP_CH:
            if (ch >= nd->target_count || index != 0)
                return CS_STATUS_INVALID_TARGET;
            return PIN_CONFIG_SUCCESS;
        case CS_TARGET_DSP_BAND: {
            if (ch >= nd->target_count) return CS_STATUS_INVALID_TARGET;
            bool peq = (index < channel_band_counts[ch]);
            // Crossover bands exist on output channels only; the EQ handler
            // rejects them on inputs, so reject at bind time too.
            bool xover = (index >= XOVER_BAND_BASE &&
                          index < XOVER_BAND_BASE + MAX_XOVER_BANDS &&
                          ch >= CH_OUT_1);
            // Gain/Q/type only exist on PEQ bands; frequency and bypass also
            // apply to the crossover bands (a sub-crossover knob).
            if (noun == CS_NOUN_FILTER_FREQ || noun == CS_NOUN_FILTER_BYPASS)
                return (peq || xover) ? PIN_CONFIG_SUCCESS : CS_STATUS_INVALID_TARGET;
            return peq ? PIN_CONFIG_SUCCESS : CS_STATUS_INVALID_TARGET;
        }
        default:
            return CS_STATUS_INVALID_TARGET;
    }
}

uint8_t cs_noun_validate_target(const CsBinding *b) {
    const CsNounDesc *nd = &cs_noun_table[b->noun];
    if (nd->target_kind == CS_TARGET_NONE) {
        // Strict: untargeted nouns must carry zeros so a host bug is
        // caught at bind time, not silently ignored.
        if (b->target != 0 || b->index != 0) return CS_STATUS_INVALID_TARGET;
        return PIN_CONFIG_SUCCESS;
    }
    return cs_noun_validate_target_ch(b->noun, b->target, b->index);
}
