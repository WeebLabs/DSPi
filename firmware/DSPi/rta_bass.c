/* Continuous low-rate bass RTA. No sample buffers, no changes to audio. */
#include "rta_bass.h"
#include <string.h>
#include <math.h>
#include "rta_bass_tables.h"

void rta_bass_configure(RtaBassCoeffs *c, uint32_t rate, uint16_t avg_ms) {
    memcpy(c, &bass_tables[rate == 44100 ? 0 : rate == 96000 ? 2 : 1], sizeof(*c));
    // One continuous power EMA, shared with the published average. The
    // intrinsic minimum is one period; longer user averaging replaces it.
    if (avg_ms) {
        float fs = (float)rate / ((float)c->decimation * 8.0f);
        float envelope = -expm1f(-1000.0f / (fs * avg_ms));
        for (int b = 0; b < RTA_BASS_BANDS; b++) {
#if RTA_SAMPLE_FLOAT
            if (envelope < c->band[b].envelope) c->band[b].envelope = envelope;
#else
            int32_t q = (int32_t)(envelope * 268435456.0f + 0.5f);
            if (q < c->band[b].envelope) c->band[b].envelope = q;
#endif
        }
    }
}

#if RTA_SAMPLE_FLOAT
/* trunc(clamp(f, +/-4) * scale), NaN -> 0; scale must be a power of two
 * so the product is exact. VCVT saturates and maps NaN to 0 by definition. */
static inline int32_t bass_input(float f, float scale, int32_t lim) {
#if defined(__arm__) && defined(__ARM_FP) /* AArch32 only; AArch64 hosts fall back */
    float t = f * scale;
    int32_t v;
    __asm__("vcvt.s32.f32 %0, %0" : "+t"(t));
    __asm__("vmov %0, %1" : "=r"(v) : "t"(t));
#else
    int32_t v = f != f ? 0 : f >= 4.0f ? lim : f <= -4.0f ? -lim : (int32_t)(f * scale);
#endif
    v = v > lim ? lim : v;
    return v < -lim ? -lim : v;
}
#else
/* All fixed-point products use 64-bit intermediates; the resulting Q27
 * signals have +/-16 headroom. Saturation prevents wrap under hot inputs. */
static inline int32_t sat(int64_t v) {
    return v > INT32_MAX ? INT32_MAX : v < INT32_MIN ? INT32_MIN : (int32_t)v;
}
#endif

void RTA_RAM_FUNC rta_bass_push(const RtaBassCoeffs *c, RtaBassChannel *s,
#if RTA_SAMPLE_FLOAT
                              const float *samples,
#else
                              const int32_t *samples,
#endif
                              uint32_t n) {
#if RTA_SAMPLE_FLOAT
    /* Hot state in locals, written back after the loop. The Q27 path keeps
     * it in memory: locals grow the RP2040 RAM function. */
    uint32_t i0 = s->integrator[0], i1 = s->integrator[1], i2 = s->integrator[2];
    uint32_t count = s->count, decimation = c->decimation;
    // count never wraps inside the loop, so one left at or past a lowered decimation would stall forever.
    if (count >= decimation) count = 0;
    const float scale = (float)(1u << (27 - c->cic_shift));
    const int32_t lim = (int32_t)(1u << (29 - c->cic_shift));
#endif
    for (uint32_t i = 0; i < n; i++) {
        /* CIC output is always Q27. Choose input scale to leave +/-4
         * headroom after the maximum CIC gain. Integer integrators avoid
         * catastrophic cancellation of floating-point running sums. */
#if RTA_SAMPLE_FLOAT
        uint32_t v = (uint32_t)bass_input(samples[i], scale, lim);
        i0 += v; i1 += i0; i2 += i1; v = i2;
        if (++count != decimation) continue;
        count = 0;
#else
        /* Samples reach 0 dBFS at 2^29, so int32 range is the float +/-4. */
        uint32_t v = (uint32_t)(samples[i] >> (2 + c->cic_shift));
        for (int j = 0; j < 3; j++) { s->integrator[j] += v; v = s->integrator[j]; }
        if (++s->count != c->decimation) continue;
        s->count = 0;
#endif
        for (int j = 0; j < 3; j++) { uint32_t old = v; v -= s->comb[j]; s->comb[j] = old; }
#if RTA_SAMPLE_FLOAT
        float x = (float)(int32_t)v * (1.0f / 134217728.0f);
#else
        int32_t x = (int32_t)v;
#endif
        for (int j = 0; j < RTA_BASS_SOS; j++) {
            const RtaBassSos *q = &c->lowpass[j];
#if RTA_SAMPLE_FLOAT
            float bx = q->b0*x;
            float y = bx + s->lp[j][0];
            s->lp[j][0] = q->b1*x - q->a1*y + s->lp[j][1];
            s->lp[j][1] = bx - q->a2*y;
#else
            int64_t bx = (int64_t)q->b0*x;
            int32_t y = sat((bx + (int64_t)s->lp[j][0]*268435456) >> 28);
            s->lp[j][0] = sat(((int64_t)q->b1*x - (int64_t)q->a1*y +
                              (int64_t)s->lp[j][1]*268435456) >> 28);
            s->lp[j][1] = sat((bx - (int64_t)q->a2*y) >> 28);
#endif
            x = y;
        }
        if (++s->low_count != 8) continue;
        s->low_count = 0;
        for (int b = 0; b < RTA_BASS_BANDS; b++) {
            const RtaBassBandCoeffs *q = &c->band[b];
            rta_bass_value_t (*z)[2] = s->band[b].z;
            rta_bass_value_t y = x;
            /* Generated gains keep every cascade prefix at <=1 peak gain. */
            for (int k = 0; k < RTA_BASS_BAND_SOS; k++) {
                const RtaBassBandSos *h = &q->sos[k];
#if RTA_SAMPLE_FLOAT
                float bx = h->b0*y;
                y = bx + z[k][0];
                z[k][0] = z[k][1] - h->a1*y;
                z[k][1] = -bx - h->a2*y;
#else
                int64_t bx = (int64_t)h->b0*y;
                y = sat((bx + (int64_t)z[k][0]*268435456) >> 28);
                z[k][0] = sat(((int64_t)z[k][1]*268435456 - (int64_t)h->a1*y) >> 28);
                z[k][1] = sat((-bx - (int64_t)h->a2*y) >> 28);
#endif
            }
#if RTA_SAMPLE_FLOAT
            s->band[b].power += q->envelope * (2*y*y - s->band[b].power);
#else
            /* Q48 power. Limit only the detector above +12 dBFS; this
             * bounds the 64-bit EMA product even under filter overload.
             * Shift before multiplication retains ~96 dB power precision. */
            int32_t yp = y > 536870912 ? 536870912 : y < -536870912 ? -536870912 : y;
            int64_t p = ((int64_t)yp*yp) >> 6;
            int64_t delta = ((p - s->band[b].power) >> 16) * q->envelope;
            s->band[b].power += delta >> 12;
            if (s->band[b].power < 0) s->band[b].power = 0;
#endif
        }
    }
#if RTA_SAMPLE_FLOAT
    s->integrator[0] = i0; s->integrator[1] = i1; s->integrator[2] = i2;
    s->count = (uint8_t)count;
#endif
}

float rta_bass_power(const RtaBassCoeffs *c, const RtaBassChannel *s, uint8_t b) {
#if RTA_SAMPLE_FLOAT
    float p = s->band[b].power;
#else
    float p = (float)s->band[b].power * (1.0f/140737488355328.0f);
#endif
    return p * c->band[b].compensation;
}
