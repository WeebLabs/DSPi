/*
 * iec61937_test.c; host harness for the firmware's IEC 61937 burst detector
 * (firmware/DSPi/iec61937.h).  Synthesises S/PDIF word streams and checks the
 * mute hold, format reporting and period measurement.
 *
 * cc -O2 -o iec61937_test iec61937_test.c && ./iec61937_test
 */

#include <stdio.h>
#include <stdlib.h>
#include "../../firmware/DSPi/iec61937.h"

static int failures = 0;

#define CHECK(cond, ...) do { \
    if (!(cond)) { failures++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); \
                   printf(__VA_ARGS__); printf("\n"); } } while (0)

// A payload word sits at bits 27:12 of the receiver's 32-bit word.
static uint32_t word(uint16_t payload) { return (uint32_t)payload << 12; }

// Feed one stereo frame; returns the detector's mute verdict after the pair.
static bool frame(Iec61937Detector *d, uint16_t l, uint16_t r) {
    iec61937_feed(d, word(l));
    return iec61937_feed(d, word(r));
}

// One burst period: preamble, payload words, zero stuffing to the period.
// pa_in_right shifts the whole burst by one word (receiver paired late).
static void burst(Iec61937Detector *d, uint8_t data_type, uint16_t pd,
                  uint32_t payload_words, uint32_t period_frames,
                  bool pa_in_right, uint32_t *muted_frames) {
    uint32_t total_words = period_frames * 2;
    uint16_t stream[8192];
    uint32_t n = 0;
    if (pa_in_right) stream[n++] = 0;
    stream[n++] = IEC61937_PA;
    stream[n++] = IEC61937_PB;
    stream[n++] = (uint16_t)(data_type & 0x1F);
    stream[n++] = pd;
    for (uint32_t i = 0; i < payload_words && n < total_words; i++) stream[n++] = (uint16_t)(0x1234 + i);
    while (n < total_words) stream[n++] = 0;
    for (uint32_t i = 0; i + 1 < total_words; i += 2) {
        if (frame(d, stream[i], stream[i + 1])) (*muted_frames)++;
    }
}

static void test_pcm_never_mutes(void) {
    printf("pcm_never_mutes\n");
    Iec61937Detector d;
    iec61937_reset(&d);
    uint32_t muted = 0;
    srand(1);
    for (uint32_t i = 0; i < 200000; i++) {
        if (frame(&d, (uint16_t)rand(), (uint16_t)rand())) muted++;
    }
    // Pa in L then Pb in R by chance is 2^-32 per frame; a random stream must
    // never trip the detector.  Also exercise Pa alone and Pb alone.
    frame(&d, IEC61937_PA, 0x0000);
    frame(&d, 0x0000, IEC61937_PB);
    frame(&d, IEC61937_PB, IEC61937_PA);
    CHECK(muted == 0, "random PCM muted %u frames", muted);
    CHECK(d.hold == 0 && d.burst_count == 0, "stray Pa/Pb counted as burst");
    CHECK(d.format == IEC61937_FORMAT_PCM, "format %u", d.format);
}

static void test_ac3_stream(void) {
    printf("ac3_stream\n");
    Iec61937Detector d;
    iec61937_reset(&d);
    uint32_t muted = 0;
    // 448 kbps AC-3 at 48 kHz: 1792 bytes = 896 payload words, period 1536.
    for (int i = 0; i < 10; i++) burst(&d, IEC61937_TYPE_AC3, 1792 * 8, 896, 1536, false, &muted);
    CHECK(muted == 10 * 1536, "muted %u of %u frames", muted, 10 * 1536);
    CHECK(d.format == IEC61937_FORMAT_AC3, "format %u", d.format);
    CHECK(d.data_type == IEC61937_TYPE_AC3, "data type %u", d.data_type);
    CHECK(d.pd == 1792 * 8, "pd %u", d.pd);
    CHECK(d.period == 1536, "period %u", d.period);
    CHECK(d.burst_count == 10, "burst_count %u", d.burst_count);
    CHECK(d.pc_error == 0, "pc_error set");

    // Stream ends: the hold runs from the last preamble, so PCM resumes
    // HOLD_WORDS minus the period's own words after the last burst.
    uint32_t still = 0;
    uint32_t first_pcm = 0;
    for (uint32_t i = 0; i < 6000; i++) {
        bool m = frame(&d, 0x0100, 0x0200);
        if (m) still++; else if (!first_pcm) first_pcm = i;
    }
    uint32_t expect = (IEC61937_HOLD_WORDS - (1536 * 2 - 2)) / 2 - 1;
    CHECK(still == expect, "hold released after %u frames (expected %u)", still, expect);
    CHECK(first_pcm == expect, "PCM resumed at frame %u", first_pcm);
}

static void test_dts_types_and_pause(void) {
    printf("dts_types_and_pause\n");
    Iec61937Detector d;
    iec61937_reset(&d);
    uint32_t muted = 0;
    burst(&d, IEC61937_TYPE_DTS1, 2012, 1006, 512, false, &muted);
    CHECK(d.format == IEC61937_FORMAT_DTS, "type I format %u", d.format);
    burst(&d, IEC61937_TYPE_DTS2, 4024, 2012, 1024, false, &muted);
    CHECK(d.period == 512, "period after type I->II %u", d.period);
    burst(&d, IEC61937_TYPE_DTS3, 8048, 4024, 2048, false, &muted);
    CHECK(d.period == 1024, "period after type II->III %u", d.period);
    CHECK(d.format == IEC61937_FORMAT_DTS, "type III format %u", d.format);
    // Pause bursts keep the last data format and keep the mute.
    for (int i = 0; i < 4; i++) burst(&d, IEC61937_TYPE_PAUSE, 0, 0, 2048, false, &muted);
    CHECK(d.format == IEC61937_FORMAT_DTS, "pause changed format to %u", d.format);
    CHECK(d.hold != 0, "pause burst did not hold the mute");
    CHECK(muted == 512 + 1024 + 2048 + 4 * 2048, "muted %u", muted);
}

static void test_other_and_error_flag(void) {
    printf("other_and_error_flag\n");
    Iec61937Detector d;
    iec61937_reset(&d);
    uint32_t muted = 0;
    // Pause with no preceding data burst reports OTHER, not PCM.
    burst(&d, IEC61937_TYPE_PAUSE, 0, 0, 1024, false, &muted);
    CHECK(d.format == IEC61937_FORMAT_OTHER, "lone pause format %u", d.format);
    // MPEG-2 AAC (type 7) is a bitstream this firmware does not name.
    burst(&d, 7, 100, 50, 1024, false, &muted);
    CHECK(d.format == IEC61937_FORMAT_OTHER, "AAC format %u", d.format);
    // Pc error flag (bit 7) is captured with the data type.
    burst(&d, (uint8_t)(0x80 | IEC61937_TYPE_AC3), 100, 50, 1536, false, &muted);
    CHECK(d.data_type == IEC61937_TYPE_AC3, "data type with error flag %u", d.data_type);
    CHECK(d.pc_error == 0, "error flag must come from Pc bit 7, got %u", d.pc_error);
    // Feed Pc with bit 7 set explicitly (burst() masks the type to 5 bits).
    frame(&d, IEC61937_PA, IEC61937_PB);
    frame(&d, 0x0080 | IEC61937_TYPE_AC3, 1234);
    CHECK(d.pc_error == 1, "error flag not captured");
    CHECK(muted > 0, "never muted");
}

static void test_misaligned_pairs(void) {
    printf("misaligned_pairs\n");
    Iec61937Detector d;
    iec61937_reset(&d);
    uint32_t muted = 0;
    for (int i = 0; i < 4; i++) burst(&d, IEC61937_TYPE_AC3, 1536 * 8, 768, 1536, true, &muted);
    CHECK(d.burst_count == 4, "burst_count %u with Pa in the right subframe", d.burst_count);
    CHECK(d.format == IEC61937_FORMAT_AC3, "format %u", d.format);
    CHECK(d.data_type == IEC61937_TYPE_AC3, "data type %u", d.data_type);
    CHECK(d.pd == 1536 * 8, "pd %u", d.pd);
    CHECK(d.period == 1536, "period %u", d.period);
    // The frame holding the late Pa is only recognised one frame later; the
    // poll zeroes it retroactively (spdif_input.c), so the detector may miss
    // exactly that one.
    CHECK(muted == 4 * 1536 - 1, "muted %u", muted);
}

static void test_reset_clears(void) {
    printf("reset_clears\n");
    Iec61937Detector d;
    iec61937_reset(&d);
    uint32_t muted = 0;
    burst(&d, IEC61937_TYPE_AC3, 100, 50, 1536, false, &muted);
    iec61937_reset(&d);
    CHECK(d.hold == 0 && d.format == IEC61937_FORMAT_PCM && d.data_type == IEC61937_TYPE_NONE
          && d.burst_count == 0 && d.period == 0, "reset left state behind");
    CHECK(!frame(&d, 0, 0), "muted after reset");
}

int main(void) {
    test_pcm_never_mutes();
    test_ac3_stream();
    test_dts_types_and_pause();
    test_other_and_error_flag();
    test_misaligned_pairs();
    test_reset_clears();
    if (failures) {
        printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    printf("ALL PASS\n");
    return 0;
}
