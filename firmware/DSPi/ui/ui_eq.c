#include "ui_eq.h"
#include <math.h>

#define PI_F 3.14159265f

// |H|^2 = (nS - nB*p + nC*p^2) / (dS - dB*p + dC*p^2) with p = sin^2(w/2).
// This form keeps the DC gain exact; the cos(w) expansion cancels
// catastrophically in float for low frequencies and high-pass shapes.
typedef struct {
    uint8_t kind;                 // 0 biquad, 1 analogue LP1, 2 analogue HP1
    float   nS, nB, nC, dS, dB, dC;
    float   fc;
} Sec;

static Sec   s_sec[UI_EQ_MAX_BANDS];
static int   s_nsec;
static float s_fs;
static int   s_col;
static bool  s_active;

static void set_sec(Sec *s, float b0, float b1, float b2, float a0, float a1, float a2) {
    s->kind = 0;
    float sb = b0 + b1 + b2, sa = a0 + a1 + a2;
    s->nS = sb * sb;
    s->nB = 4.0f * (b0 * b1 + 4.0f * b0 * b2 + b1 * b2);
    s->nC = 16.0f * b0 * b2;
    s->dS = sa * sa;
    s->dB = 4.0f * (a0 * a1 + 4.0f * a0 * a2 + a1 * a2);
    s->dC = 16.0f * a0 * a2;
}

static bool build_sec(const UiEqBand *b, float fs, Sec *out) {
    if (b->bypass) return false;
    float f0 = b->freq;
    if (f0 < 5.0f) f0 = 5.0f;
    if (f0 > fs * 0.45f) f0 = fs * 0.45f;
    float q = b->q > 0.05f ? b->q : 0.7071f;
    float w0 = 2.0f * PI_F * f0 / fs;
    float cw = cosf(w0), sw = sinf(w0);
    float alpha = sw / (2.0f * q);
    float A = powf(10.0f, b->gain_db / 40.0f);
    float sqA = sqrtf(A);

    switch (b->type) {
    case EQT_PEAKING:
        set_sec(out, 1 + alpha * A, -2 * cw, 1 - alpha * A, 1 + alpha / A, -2 * cw, 1 - alpha / A);
        return true;
    case EQT_LOWSHELF1: alpha = sw / (2.0f * 0.7071f);
        /* fallthrough */
    case EQT_LOWSHELF:
        set_sec(out,
                A * ((A + 1) - (A - 1) * cw + 2 * sqA * alpha),
                2 * A * ((A - 1) - (A + 1) * cw),
                A * ((A + 1) - (A - 1) * cw - 2 * sqA * alpha),
                (A + 1) + (A - 1) * cw + 2 * sqA * alpha,
                -2 * ((A - 1) + (A + 1) * cw),
                (A + 1) + (A - 1) * cw - 2 * sqA * alpha);
        return true;
    case EQT_HIGHSHELF1: alpha = sw / (2.0f * 0.7071f);
        /* fallthrough */
    case EQT_HIGHSHELF:
        set_sec(out,
                A * ((A + 1) + (A - 1) * cw + 2 * sqA * alpha),
                -2 * A * ((A - 1) + (A + 1) * cw),
                A * ((A + 1) + (A - 1) * cw - 2 * sqA * alpha),
                (A + 1) - (A - 1) * cw + 2 * sqA * alpha,
                2 * ((A - 1) - (A + 1) * cw),
                (A + 1) - (A - 1) * cw - 2 * sqA * alpha);
        return true;
    case EQT_LOWPASS:
        set_sec(out, (1 - cw) / 2, 1 - cw, (1 - cw) / 2, 1 + alpha, -2 * cw, 1 - alpha);
        return true;
    case EQT_HIGHPASS:
        set_sec(out, (1 + cw) / 2, -(1 + cw), (1 + cw) / 2, 1 + alpha, -2 * cw, 1 - alpha);
        return true;
    case EQT_NOTCH:
        set_sec(out, 1, -2 * cw, 1, 1 + alpha, -2 * cw, 1 - alpha);
        return true;
    case EQT_LOWPASS1:  out->kind = 1; out->fc = f0; return true;
    case EQT_HIGHPASS1: out->kind = 2; out->fc = f0; return true;
    default:            return false;     // flat, all-pass, Linkwitz: no magnitude change drawn
    }
}

int ui_eq_col_for_freq(float f) {
    if (f < UI_EQ_F_LO) f = UI_EQ_F_LO;
    if (f > UI_EQ_F_HI) f = UI_EQ_F_HI;
    float t = log10f(f / UI_EQ_F_LO) / 3.0f;            // 20 Hz..20 kHz = 3 decades
    int c = (int)(t * (UI_EQ_COLS - 1) + 0.5f);
    return c < 0 ? 0 : (c >= UI_EQ_COLS ? UI_EQ_COLS - 1 : c);
}

void ui_eq_calc_begin(UiEq *eq, float fs) {
    s_fs = fs > 8000.0f ? fs : 48000.0f;
    s_nsec = 0;
    for (int i = 0; i < eq->nb && i < UI_EQ_MAX_BANDS; i++) {
        Sec s;
        if (build_sec(&eq->band[i], s_fs, &s)) s_sec[s_nsec++] = s;
    }
    s_col = 0;
    s_active = true;
    eq->curve_valid = false;
}

static int8_t eval_col(int col) {
    float f = UI_EQ_F_LO * powf(UI_EQ_F_HI / UI_EQ_F_LO, (float)col / (float)(UI_EQ_COLS - 1));
    float w = 2.0f * PI_F * f / s_fs;
    float sh = sinf(0.5f * w);
    float p = sh * sh, p2 = p * p;
    float prod = 1.0f;
    for (int i = 0; i < s_nsec; i++) {
        const Sec *s = &s_sec[i];
        if (s->kind == 0) {
            float num = s->nS - s->nB * p + s->nC * p2;
            float den = s->dS - s->dB * p + s->dC * p2;
            if (den < 1e-12f) den = 1e-12f;
            if (num < 0.0f) num = 0.0f;
            prod *= num / den;
        } else {
            float x = f / s->fc;
            x *= x;
            prod *= (s->kind == 1) ? 1.0f / (1.0f + x) : x / (1.0f + x);
        }
    }
    if (prod < 1e-9f) prod = 1e-9f;
    float q = 40.0f * log10f(prod);                     // 10*log10 in dB, x4 for quarter-dB
    if (q > 96.0f) q = 96.0f;
    if (q < -96.0f) q = -96.0f;
    return (int8_t)lrintf(q);
}

bool ui_eq_calc_step(UiEq *eq, int max_cols) {
    if (!s_active) return true;
    while (max_cols-- > 0 && s_col < UI_EQ_COLS) {
        int c = s_col++;
        eq->curve_q[c] = eval_col(c);
    }
    if (s_col >= UI_EQ_COLS) {
        eq->curve_valid = true;
        s_active = false;
        return true;
    }
    return false;
}

void ui_eq_calc_all(UiEq *eq, float fs) {
    ui_eq_calc_begin(eq, fs);
    while (!ui_eq_calc_step(eq, 16)) { }
}
