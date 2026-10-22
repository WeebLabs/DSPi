/*
 * iec61937.h; IEC 61937 data-burst detector for the S/PDIF input.
 *
 * Header-only and SDK-free so the host harness (tools/iec61937_test) compiles
 * the same code the firmware runs.  Phase 0 of the AC-3 / DTS plan: see
 * Documentation/Features/ac3_dts_decoding_plan.md and the architecture doc's
 * "IEC 61937 Bitstream Detection" section.
 */

#ifndef IEC61937_H
#define IEC61937_H

#include <stdint.h>
#include <stdbool.h>

#define IEC61937_PA               0xF872u
#define IEC61937_PB               0x4E1Fu

// Pc data types (bits 4:0) this firmware recognises.
#define IEC61937_TYPE_NULL        0
#define IEC61937_TYPE_AC3         1
#define IEC61937_TYPE_PAUSE       3
#define IEC61937_TYPE_DTS1        11
#define IEC61937_TYPE_DTS2        12
#define IEC61937_TYPE_DTS3        13
#define IEC61937_TYPE_NONE        0xFF   // no burst seen yet

// Reported format (SpdifBitstreamStatus.format, NOTIFY_EVT_SPDIF_FORMAT).
#define IEC61937_FORMAT_PCM       0
#define IEC61937_FORMAT_AC3       1
#define IEC61937_FORMAT_DTS       2
#define IEC61937_FORMAT_OTHER     3      // burst of another type, or non-audio flag only

// Words (not frames) a burst keeps the input muted after its preamble.  Two
// DTS type III periods (2048 frames each) is the longest legal gap between
// preambles; 8192 words = 4096 frames covers it with margin at every rate.
#define IEC61937_HOLD_WORDS       8192u

typedef struct {
    uint16_t prev;         // previous payload word (preamble spans two words)
    uint8_t  phase;        // 0 = scanning, 1 = next word is Pc, 2 = next is Pd
    uint8_t  data_type;    // Pc bits 4:0 of the last burst, IEC61937_TYPE_NONE if none
    uint8_t  pc_error;     // Pc bit 7 (error flag) of the last burst
    uint8_t  format;       // IEC61937_FORMAT_* derived from data_type
    uint16_t pd;           // Pd of the last burst (length in bits for AC-3, bytes for DTS)
    uint16_t period;       // frames between the last two preambles (0 = unknown)
    uint32_t hold;         // words of mute remaining (0 = not holding)
    uint32_t since_pa;     // words since the last Pa, for period measurement
    uint32_t burst_count;  // preambles seen since reset
} Iec61937Detector;

static inline void iec61937_reset(Iec61937Detector *d) {
    d->prev = 0;
    d->phase = 0;
    d->data_type = IEC61937_TYPE_NONE;
    d->pc_error = 0;
    d->format = IEC61937_FORMAT_PCM;
    d->pd = 0;
    d->period = 0;
    d->hold = 0;
    d->since_pa = 0;
    d->burst_count = 0;
}

// Null and pause bursts carry no data type of their own; they keep the
// format of the data burst they follow so a host does not see PCM flicker.
static inline uint8_t iec61937_format_for(uint8_t data_type, uint8_t current) {
    switch (data_type) {
        case IEC61937_TYPE_AC3:  return IEC61937_FORMAT_AC3;
        case IEC61937_TYPE_DTS1:
        case IEC61937_TYPE_DTS2:
        case IEC61937_TYPE_DTS3: return IEC61937_FORMAT_DTS;
        case IEC61937_TYPE_NULL:
        case IEC61937_TYPE_PAUSE:
            return current == IEC61937_FORMAT_PCM ? IEC61937_FORMAT_OTHER : current;
        default:                 return IEC61937_FORMAT_OTHER;
    }
}

// The rare path (a preamble, then its Pc and Pd) is kept out of line so the
// per-word feed stays a few instructions when inlined into a -O3 loop; the
// including file may define IEC61937_COLD_ATTR to place it in RAM.
#ifndef IEC61937_COLD_ATTR
#define IEC61937_COLD_ATTR __attribute__((noinline))
#endif
IEC61937_COLD_ATTR
static void iec61937_event(Iec61937Detector *d, uint16_t w) {
    if (d->phase == 1) {
        d->data_type = (uint8_t)(w & 0x1Fu);
        d->pc_error  = (uint8_t)((w >> 7) & 1u);
        d->format    = iec61937_format_for(d->data_type, d->format);
        d->phase = 2;
    } else if (d->phase == 2) {
        d->pd = w;
        d->phase = 0;
    } else {
        // since_pa counted the words from the previous Pa up to and including
        // this Pb, so the Pa-to-Pa distance is one word less.
        uint32_t words = d->since_pa - 1u;
        if (d->burst_count > 0 && words / 2u <= 0xFFFFu) d->period = (uint16_t)(words / 2u);
        d->since_pa = 1;
        d->burst_count++;
        d->hold = IEC61937_HOLD_WORDS;
        d->phase = 1;
    }
}

// Feed one S/PDIF word (24-bit audio at bits 27:4; the IEC 61937 payload is
// its top 16 bits).  Returns true while the input must stay muted.  Pa/Pb
// are matched across consecutive words whatever their subframe parity, so a
// receiver that pairs frames one word late still detects the burst.
static inline bool iec61937_feed(Iec61937Detector *d, uint32_t spdif_word) {
    uint16_t w = (uint16_t)(spdif_word >> 12);
    d->since_pa++;
    if (d->hold) d->hold--;
    if (__builtin_expect(d->phase != 0 || (d->prev == IEC61937_PA && w == IEC61937_PB), 0))
        iec61937_event(d, w);
    d->prev = w;
    return d->hold != 0;
}

#endif // IEC61937_H
