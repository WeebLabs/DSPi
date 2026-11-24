/*
 * ui_eq.h; approximate magnitude response of the listed PEQ bands, for the
 * EQ page graph. Computed incrementally (a few columns per background step)
 * so it never holds the main loop. RBJ cookbook biquads; first-order types use
 * the analogue prototype. This is a display aid, not DSPi's exact filter math.
 */
#ifndef UI_EQ_H
#define UI_EQ_H

#include "ui_types.h"

// DSPi FilterType values this module understands (mirrors config.h; ui.c
// static-asserts the equivalence at build time).
#define EQT_FLAT        0
#define EQT_PEAKING     1
#define EQT_LOWSHELF    2
#define EQT_HIGHSHELF   3
#define EQT_LOWPASS     4
#define EQT_HIGHPASS    5
#define EQT_NOTCH       6
#define EQT_ALLPASS     7
#define EQT_ALLPASS1    8
#define EQT_LOWSHELF1   9
#define EQT_HIGHSHELF1  10
#define EQT_LINKWITZ    11
#define EQT_LOWPASS1    12
#define EQT_HIGHPASS1   13

#define UI_EQ_F_LO   20.0f
#define UI_EQ_F_HI   20000.0f

void ui_eq_calc_begin(UiEq *eq, float fs);
bool ui_eq_calc_step(UiEq *eq, int max_cols);     // true once the whole curve is done
void ui_eq_calc_all(UiEq *eq, float fs);          // blocking helper (host tests)
int  ui_eq_col_for_freq(float f_hz);              // graph column of a frequency, clamped

#endif
