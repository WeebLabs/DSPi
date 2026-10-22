/*
 * input_capture_arena.h; shared storage for the input capture buffers.
 *
 * The SPDIF RX FIFO, the I2S RX rings and the ADAT RX ring are mutually
 * exclusive (one input source runs at a time), so they overlay one arena.
 * See Documentation/current_architecture.md "Input Capture Arena" for the
 * ownership invariant and the per-platform sizes.
 */

#ifndef INPUT_CAPTURE_ARENA_H
#define INPUT_CAPTURE_ARENA_H

#include <stdint.h>
#include "pico.h"

// Geometry mirrors the owning modules' own ring constants; each module
// static-asserts its constants against these, so a size change there breaks
// the build instead of silently truncating a member.
#if PICO_RP2350
#define INPUT_ARENA_ALIGN        8192u
#define INPUT_ARENA_ADAT_WORDS   2048u
#define INPUT_ARENA_I2S_PAIRS    4u
#define INPUT_ARENA_I2S_WORDS    2048u
#else
#define INPUT_ARENA_ALIGN        4096u
#define INPUT_ARENA_I2S_PAIRS    1u
#define INPUT_ARENA_I2S_WORDS    1024u
#endif
#define INPUT_ARENA_SPDIF_WORDS  3072u
#if PICO_RP2350
// AC-3 decode (ac3_stream.h static-asserts these against its own sizes):
// two frame buffers with the bit reader's 4-byte slack, and the decoded ring.
#define INPUT_ARENA_AC3_FRAME_BYTES  3844u
#define INPUT_ARENA_AC3_RING_FRAMES  512u
#define INPUT_ARENA_AC3_CHANNELS     6u
#endif

// S/PDIF member: the receiver FIFO first (it carries the DMA alignment),
// then, on RP2350, the AC-3 decoder's buffers, which only exist while the
// S/PDIF receiver owns the arena.  32,264 B on RP2350, under the I2S member.
typedef struct {
    uint32_t fifo[INPUT_ARENA_SPDIF_WORDS];
#if PICO_RP2350
    uint8_t  ac3_frames[2][INPUT_ARENA_AC3_FRAME_BYTES] __attribute__((aligned(4)));
    float    ac3_ring[INPUT_ARENA_AC3_RING_FRAMES][INPUT_ARENA_AC3_CHANNELS];
#endif
} InputArenaSpdif;

// Alignment is the largest DMA write-wrap requirement of any member.  Each
// I2S row is its own wrap region, so row bytes == arena alignment is what
// keeps every row self-aligned; do not shrink a row without re-checking it.
typedef union {
#if PICO_RP2350
    uint32_t adat_ring[INPUT_ARENA_ADAT_WORDS];
#endif
    uint32_t i2s_ring[INPUT_ARENA_I2S_PAIRS][INPUT_ARENA_I2S_WORDS];
    InputArenaSpdif spdif;
} __attribute__((aligned(INPUT_ARENA_ALIGN))) InputCaptureArena;

extern InputCaptureArena input_capture_arena;

typedef enum {
    INPUT_ARENA_FREE = 0,
    INPUT_ARENA_SPDIF,
    INPUT_ARENA_I2S,
    INPUT_ARENA_ADAT,
} InputArenaOwner;

// Claim panics unless the arena is free or already held by the same owner:
// two live receivers would write the same bytes.  Release by a non-owner is
// a no-op.  USB claims nothing.
void input_arena_claim(InputArenaOwner owner);
void input_arena_release(InputArenaOwner owner);
InputArenaOwner input_arena_get_owner(void);

#endif // INPUT_CAPTURE_ARENA_H
