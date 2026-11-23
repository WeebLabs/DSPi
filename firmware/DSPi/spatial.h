#ifndef SPATIAL_H
#define SPATIAL_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "config.h"

// Stereo spatializer: width or crosstalk cancellation on selected output pairs,
// both platforms, zero latency.  Design: Documentation/Features/stereo_spatializer_spec.md.

#define SPATIAL_MODE_WIDTH  0
#define SPATIAL_MODE_XTC    1
#define SPATIAL_MODE_MAX    1

#define SPATIAL_WIDTH_MIN     0.0f    // percent, both width fields
#define SPATIAL_WIDTH_MAX   200.0f
#define SPATIAL_SHELF_MIN   100.0f    // Hz
#define SPATIAL_SHELF_MAX  1500.0f
#define SPATIAL_SPAN_MIN      5.0f    // speaker half-angle, degrees
#define SPATIAL_SPAN_MAX     30.0f
#define SPATIAL_ATTEN_MIN     2.5f    // dB per pass; 2.5 dB is G_MAX = 0.75
#define SPATIAL_ATTEN_MAX    20.0f
#define SPATIAL_MID_MIN       0.0f    // percent
#define SPATIAL_MID_MAX     100.0f
#define SPATIAL_TRIM_MIN    -12.0f    // dB; never above 0 (RP2040 headroom, spec 7.3)
#define SPATIAL_TRIM_MAX      0.0f

#define SPATIAL_DEFAULT_MODE        SPATIAL_MODE_WIDTH
#define SPATIAL_DEFAULT_PAIRS       0x01
#define SPATIAL_DEFAULT_WIDTH       125.0f
#define SPATIAL_DEFAULT_BASS_WIDTH  150.0f
#define SPATIAL_DEFAULT_SHELF       700.0f
#define SPATIAL_DEFAULT_SPAN         10.0f
#define SPATIAL_DEFAULT_ATTEN         3.0f   // a DSPi choice, not a published RACE figure
#define SPATIAL_DEFAULT_MID         100.0f
#define SPATIAL_DEFAULT_TRIM          0.0f

#define SPATIAL_XTC_HP_HZ     250.0f  // fixed loop band (RACE's published edges)
#define SPATIAL_XTC_LP_HZ    5000.0f
#define SPATIAL_HEAD_RADIUS_M  0.0875f
#define SPATIAL_SOUND_M_S    343.0f
#define SPATIAL_FADE_MS       10.0f
#define SPATIAL_RING           32     // samples; holds 30 degrees at 96 kHz plus the interpolation tap
#define SPATIAL_PAIR_MASK  ((uint8_t)((1u << NUM_SPDIF_INSTANCES) - 1u))

typedef struct {
    bool enabled;
    uint8_t mode;          // SPATIAL_MODE_*
    uint8_t pair_mask;     // bit p = output pair p (outputs 2p, 2p+1)
    float width_pct;
    float bass_width_pct;
    float shelf_hz;
    float span_deg;
    float atten_db;
    float mid_pct;
    float trim_db;
} SpatialConfig;

// Wire image (FX.SPATIAL_CONFIG, bulk section, preset slot): 32 bytes, spec 8.2.
typedef struct __attribute__((packed)) {
    uint8_t enabled;
    uint8_t mode;
    uint8_t pair_mask;
    uint8_t reserved;      // zero on GET, ignored on SET
    float width_pct;
    float bass_width_pct;
    float shelf_hz;
    float span_deg;
    float atten_db;
    float mid_pct;
    float trim_db;
} SpatialConfigPacket;
_Static_assert(sizeof(SpatialConfigPacket) == 32, "SpatialConfigPacket wire size");

// Parameter ids, in FX.SPATIAL_ENABLE..FX.SPATIAL_TRIM order (command = 0xB1 + id).
enum {
    SPATIAL_PARAM_ENABLED = 0,
    SPATIAL_PARAM_MODE,
    SPATIAL_PARAM_PAIRS,
    SPATIAL_PARAM_WIDTH,
    SPATIAL_PARAM_BASS_WIDTH,
    SPATIAL_PARAM_SHELF,
    SPATIAL_PARAM_SPAN,
    SPATIAL_PARAM_ATTEN,
    SPATIAL_PARAM_MID,
    SPATIAL_PARAM_TRIM,
    SPATIAL_PARAM_COUNT
};

#if PICO_RP2350
typedef float sp_t;
#elif defined(SP_Q28_WIDE)
typedef int64_t sp_t;      // host harness only: Q28 that cannot wrap, the reference for int32
#else
typedef int32_t sp_t;      // Q28
#endif

// Published coefficient set: both modes, always present while the stage exists.
typedef struct {
    uint32_t fs;           // the rate it was computed for
    uint16_t fade_len;     // samples in a full fade
    uint8_t enabled, mode;
    sp_t fade_step;        // 1 / fade_len
    sp_t w1, dwb, aw;      // width: w - 1, wb - w, shelf one-pole coefficient
    sp_t g, gm;            // XTC: g and gm, each times the band normalisation k
    sp_t a5, ah;           // XTC loop one-poles: 5 kHz, and 250 Hz inside the high-pass
    sp_t t;                // trim, linear, at most 1
    sp_t d;                // XTC delay in samples (Q16.16 on RP2040), at least 1
} SpatialCoeffs;

typedef struct {
    sp_t lp, hl;           // 5 kHz low-pass, 250 Hz low-pass of the high-pass
    sp_t ring[SPATIAL_RING];
} SpatialXtcPath;

typedef struct {
    uint32_t fs;
    uint16_t fade;         // 0..fade_len
    uint8_t mode;          // running mode
    uint8_t widx;
    bool dirty;            // holds state that must be cleared before reuse
    bool primed;           // ramp values below are live
    bool m_dirty;          // the mid recursion holds state
    uint16_t ramp;         // samples left in the running gain ramp (spec 6)
    uint16_t m_tail;       // samples the mid filters keep running after gm settles at 0
    sp_t w1, dwb, aw, g, gm, t, d;   // values in use
    sp_t tw1, tdwb, taw, tg, tgm, tt; // ramp targets
    sp_t sw1, sdwb, saw, sg, sgm, st; // per-sample ramp steps
    sp_t lpw;              // width shelf state
    SpatialXtcPath s, m;
} SpatialPairState;

extern volatile SpatialConfig spatial_config;
extern volatile bool spatial_update_pending;
extern volatile const SpatialCoeffs *current_spatial_coeffs;
extern SpatialPairState spatial_pair_state[NUM_SPDIF_INSTANCES];

void spatial_config_defaults(volatile SpatialConfig *cfg);
void spatial_compute_coefficients(SpatialCoeffs *c, const SpatialConfig *cfg, float fs);

// Recompute and publish current_spatial_coeffs (never NULL afterwards).  Main loop.
void spatial_apply_config(float fs);

// Wire codec.  Decoding sanitizes for bulk and flash: non-finite floats take the
// default, an unknown mode loads as width with the stage off, the mask is masked.
void spatial_config_to_packet(SpatialConfigPacket *out);
void spatial_packet_to_config(const SpatialConfigPacket *in);

// Vendor access: SETs clamp, store, raise the pending flag and notify.  A mode
// above SPATIAL_MODE_MAX or a non-finite value returns false and changes nothing.
bool spatial_set_param(uint8_t id, float v);
float spatial_get_param(uint8_t id);
bool spatial_set_config(const SpatialConfigPacket *in);
void spatial_notify_config(void);

// Runs the stage on output pairs first_pair..last_pair with a per-packet snapshot
// of coefficients and mask, on the core that owns those pairs (spec 7.1).
void spatial_process_pairs(const SpatialCoeffs *c, uint32_t pair_mask, int first_pair, int last_pair,
                           sp_t (*buf_out)[AUDIO_BUFFER_SAMPLES], uint32_t n);

#endif // SPATIAL_H
