/*
 * ui_types.h; data the pages draw from. Pure data, no hardware, so the whole
 * render path can be exercised on a host (see test/).
 */
#ifndef UI_TYPES_H
#define UI_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#define UI_PRESETS        10
#define UI_EQ_MAX_BANDS   10
#define UI_EQ_COLS        92
#define UI_FX_COUNT       6

typedef enum { UI_PG_NOW, UI_PG_EQ, UI_PG_FX, UI_PG_PRESET, UI_PG_SYS, UI_PG_COUNT } UiPage;

typedef struct {
    uint8_t type;        // DSPi FilterType value
    bool    bypass;
    float   freq, gain_db, q;
} UiEqBand;

typedef struct {
    float   vol_db, vol_lo_db, vol_hi_db;
    bool    mute, streaming, clip;
    uint8_t rate_idx;    // 0 = 44.1k, 1 = 48k, 2 = 96k
    uint8_t preset;      // active preset slot
    uint8_t cpu_pct;
    float   lvl_db[2];   // output peak, dB
    bool    fx_on[UI_FX_COUNT];
} UiLive;

typedef struct {
    uint8_t  nb;                         // active (non-flat) bands
    uint8_t  ch;                         // DSP channel the bands were read from
    uint8_t  slot[UI_EQ_MAX_BANDS];      // band slot index of each listed band
    UiEqBand band[UI_EQ_MAX_BANDS];
    int8_t   curve_q[UI_EQ_COLS];        // response in 0.25 dB units
    bool     curve_valid;
} UiEq;

typedef struct {
    UiPage  page;
    bool    in_menu;
    uint8_t cursor;
    UiLive  live;
    UiEq    eq;
    char    preset_name[UI_PRESETS][13];
    uint8_t preset_valid[UI_PRESETS];    // 1 = stored preset exists
    float   meter[2];                    // 0..1 bar position
    float   peak[2];                     // 0..1 peak-hold position
    uint8_t disp_level, led_level;       // 0..3
    char    toast[20];
    bool    toast_on;
    uint8_t hold_pct;                    // button hold progress toward long press, 0 = idle
    uint32_t now_ms;
} UiView;

#endif
