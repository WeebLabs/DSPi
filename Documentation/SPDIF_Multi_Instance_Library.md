# pico_audio_spdif_multi: Multi-Instance S/PDIF Output Library

*Last updated: 2026-10-12*

## Table of Contents

1. [Overview](#overview)
2. [Architecture](#architecture)
3. [Hardware Requirements](#hardware-requirements)
4. [Build Integration](#build-integration)
5. [Compile-Time Configuration](#compile-time-configuration)
6. [Data Structures](#data-structures)
7. [API Reference](#api-reference)
8. [Encoders](#encoders)
9. [Usage Guide](#usage-guide)
10. [Synchronized Startup](#synchronized-startup)
11. [Sample Rate Setting](#sample-rate-setting)
12. [Underrun Handling](#underrun-handling)
13. [Resource Allocation Map](#resource-allocation-map)
14. [Constraints and Limitations](#constraints-and-limitations)
15. [Internals](#internals)
16. [Troubleshooting](#troubleshooting)

---

## Overview

`pico_audio_spdif_multi` is a PIO-based S/PDIF (IEC 60958) transmitter for the RP2040 and RP2350. Each output instance owns one ring of encoded frames in RAM. A DMA channel reads that ring forever, wrapping in hardware, and feeds a PIO state machine that drives one GPIO. The caller writes blocks of 24-bit stereo samples into the ring, and the library encodes them on the way in.

Nothing in the output path takes an interrupt. Fill level, consumed frames and underrun detection are all pointer arithmetic on the DMA read address. The writer keeps a margin of correctly framed silence ahead of the audio, and a silence pump extends it to a full ring while no audio is written. A stalled producer therefore plays clean silence and never a loop of stale audio.

Up to 4 instances run at once. Instances on the same PIO block can be started on the same system clock cycle, which is what keeps DSPi's output slots sample-aligned.

### Key Features

- Up to 4 independent S/PDIF outputs, each with its own PIO SM, DMA channel, GPIO pin and ring
- Free-running DMA ring per instance, with no DMA interrupts and no buffer queues
- Caller-owned ring storage, so the same RAM can be reused by another output type on the same slot
- Two interchangeable encoders, the PIO biphase-mark encoder by default and a software NRZI encoder for tight PIO instruction memory
- Erase-ahead silence margin and a silence pump, so underruns and stalls play framed silence
- Group re-anchor primitive (`audio_spdif_ring_skip()`) that lets the caller recover from underruns without moving one slot against another
- Synchronized start of all instances on a block, with a prepare-only half so the caller can start other SMs in the same mask write
- 24-bit audio with IEC 60958-3 consumer channel status, including the sample-rate byte

### Files

| File | Role |
|------|------|
| `include/pico/audio_spdif.h` | Public header. Types, ring geometry macros, function declarations. |
| `include/pico/audio_spdif/sample_encoding.h` | `spdif_subframe_t`, the shared `spdif_lookup` table and the inline software encoder `spdif_update_subframe()`. |
| `audio_spdif.c` | Setup, ring writer, consumed-side accounting, DMA arm and stop, encoder selection, synchronized start. |
| `audio_spdif_bmc.pio` | 16-instruction program that does the biphase-mark encoding in the PIO. |
| `audio_spdif.pio` | 4-instruction NRZI program fed with software-encoded words. |
| `CMakeLists.txt` | INTERFACE library target `pico_audio_spdif_multi`. |

All paths are relative to `firmware/pico-extras/src/rp2_common/pico_audio_spdif_multi/`.

---

## Architecture

```
  int32 L/R block                      slot ring (caller-owned)            free-running DMA       PIO SM
  (24-bit in [23:0])                   1024 frames, 16 KB                  ENDLESS (RP2350) or     biphase-mark
        |                                                                  data + reload (RP2040)  line output
        v                              +------------------------------+          |                  |
  audio_spdif_ring_write_s32() ------> | audio | audio | silence ...  | -------> DMA ---> TX FIFO ---> GPIO
  encodes, stamps IEC block phase      +------------------------------+
                                         ^ reader        ^ writer   ^ erase-ahead edge
```

Each instance has its own ring, DMA channel (plus a reload partner on RP2040), PIO state machine and pin. Shared across all instances are the software encoder lookup table, one loaded PIO program per PIO block, the IEC channel-status table, and the instance registry used by the starvation getters.

### Data Flow

1. The caller hands a block of interleaved L/R `int32_t` samples to `audio_spdif_ring_write_s32()`.
2. The writer encodes each frame into the ring at its absolute frame position, stamping the IEC 60958 preamble and channel-status bit from that position modulo 192.
3. The writer then extends a margin of encoded silence ahead of the new audio (`PICO_AUDIO_SPDIF_ERASE_AHEAD`, 192 frames).
4. The DMA channel reads the ring continuously into the PIO TX FIFO, wrapping at the ring boundary in hardware. It is paced only by the SM's DREQ.
5. The PIO SM shifts the words out as a biphase-mark S/PDIF signal at 256 PIO cycles per stereo frame.
6. Readers of the fill level ask the library for consumed frames. The library unwraps the DMA read address into a monotonic counter and subtracts it from the frames written.

---

## Hardware Requirements

Each S/PDIF instance requires the following.

| Resource | Count per Instance | Notes |
|----------|--------------------|-------|
| GPIO pin | 1 | Any GPIO. Driven by the PIO side-set. |
| PIO state machine | 1 | On any PIO block. Every instance on a block shares that block's loaded program. |
| DMA channel | 1 | Unique. Its number is also the instance's stable statistics index. |
| DMA reload channel | 1 on RP2040, 0 on RP2350 | RP2040 has no ENDLESS transfer count, so a partner channel rewrites the data channel's count each ring pass. |
| Ring storage | `PICO_AUDIO_SPDIF_RING_BYTES` (16 KB) | Caller-owned RAM, aligned to its own size. |
| DMA IRQ line | none | The output path takes no interrupts. |

### Pin Electrical Characteristics

The S/PDIF coaxial specification (IEC 60958-3) calls for 0.5 V peak-to-peak into 75 ohms. The PIO output is 3.3 V CMOS. For a proper coaxial output use a resistive divider or a series resistor with a pulse transformer or capacitor-coupled stage. For a short run to a DAC or optical transmitter on the same board, a direct connection usually works.

---

## Build Integration

### CMakeLists.txt

Link against the INTERFACE target.

```cmake
target_link_libraries(your_app
    pico_audio_spdif_multi
    # ... other libraries
)
```

The target generates headers for both PIO programs and links `hardware_dma`, `hardware_pio`, `hardware_irq`, `hardware_clocks` and `hardware_sync`. It does not depend on `pico_audio`.

The implementation includes `audio_clock_div.h`, which the application must provide on its include path. DSPi provides it in `firmware/DSPi/`. It defines `audio_base_divider_16_8()`, the shared base divider that every DSPi output clock derives from.

### Include

```c
#include "pico/audio_spdif.h"
```

---

## Compile-Time Configuration

| Macro | Value | Overridable | Description |
|-------|-------|-------------|-------------|
| `PICO_AUDIO_SPDIF_PIO` | `PICO_AUDIO_PIO`, else `0` | Yes | Convenience default for `audio_spdif_config_t.pio`. The library itself does not read it. DSPi sets it to 0. |
| `PICO_AUDIO_SPDIF_PIN` | `0` | Yes | Convenience default for `audio_spdif_config_t.pin`. DSPi sets it to 6. |
| `PICO_AUDIO_SPDIF_MAX_INSTANCES` | `4` | No | Size of the instance registry. Asserted in setup. |
| `PICO_AUDIO_SPDIF_BLOCK_SAMPLE_COUNT` | `192` | No | IEC 60958 block length in frames (channel-status period). Fixed by the standard. |
| `PICO_AUDIO_SPDIF_RING_FRAMES` | `1024` | No | Ring capacity in frames. Must be a power of two (static assert). 21.3 ms at 48 kHz. |
| `PICO_AUDIO_SPDIF_RING_BYTES` | `16384` | No | Ring size in bytes, 16 bytes per frame. Also the required alignment of `ring_base`. |
| `PICO_AUDIO_SPDIF_ERASE_AHEAD` | `192` | No | Frames of encoded silence the writer keeps ahead of real audio. |

---

## Data Structures

### `audio_spdif_instance_t`

Allocate one per output, with static storage duration and zero-initialized. The library manages every field after `audio_spdif_setup()`. Callers may read the counters but should not write them.

```c
typedef struct audio_spdif_instance {
    // Hardware config (set in setup, immutable after)
    PIO pio;
    uint8_t pio_sm;
    uint8_t dma_channel;
    uint8_t dma_reload_channel; // RP2040 only: free-running reload pair partner
    uint8_t pin;
    uint8_t instance_index;     // Stable slot identity (== dma_channel)

    // Runtime state
    uint32_t freq;
    bool enabled;
    bool pio_bmc;

    // Ring state
    uint32_t *ring;                     // slot ring base (caller-owned words)
    volatile uint32_t wr_frames;        // frames of real audio written
    uint32_t erased_frames;             // silence stamped through this total
    volatile uint32_t underrun_frames;  // silence frames exposed to the wire
    volatile uint32_t drop_frames;      // producer frames dropped on ring-full

    // Consumed-side unwrap state
    uint32_t last_read_addr;
    uint32_t last_read_time_us;
    volatile uint32_t consumed_words;   // words; USB feedback only
    volatile uint32_t consumed_frames;  // frames; ALL fill/deficit math
    uint8_t consumed_word_rem;

    volatile uint32_t snap_produced_frames;
    volatile uint32_t snap_consumed_frames;
    volatile uint32_t recovery_prewrite_fill_frames;
} audio_spdif_instance_t;
```

(Header comments shortened here.)

| Field | Meaning |
|-------|---------|
| `freq` | Rate the SM divider is programmed for. Written by setup and `audio_spdif_apply_pio_frequency()`. |
| `enabled` | True from `audio_spdif_set_enabled(true)` or `audio_spdif_enable_sync_prepare()` until disable or teardown. |
| `pio_bmc` | Word format of this instance's ring. True for the PIO encoder, false for software NRZI words. Fixed at setup from the block's loaded program. |
| `ring` | Ring base taken from `config->ring_base`. NULL after teardown, which callers use as "not set up". |
| `wr_frames` | Frames of audio written since the last ring reset, including frames advanced by `audio_spdif_ring_skip()`. |
| `erased_frames` | Absolute frame position up to which silence has been stamped. |
| `underrun_frames` | Frames advanced by `audio_spdif_ring_skip()`. Reported by the starvation getters. |
| `drop_frames` | Frames rejected by the writer because the block did not fit. |
| `consumed_words`, `consumed_frames` | Monotonic DMA consumption since the last ring reset. Fill and deficit arithmetic must use `consumed_frames`, because `consumed_words` wraps four times sooner. |
| `snap_produced_frames`, `snap_consumed_frames` | Produced and consumed totals latched together at the end of each block write. They are sampled at a constant production phase, which is what DSPi's clock servo observes. |
| `recovery_prewrite_fill_frames` | Fill immediately before the most recent successful write. `UINT32_MAX` until the first write after a reset. DSPi uses it to size underrun re-anchors so PDM stays aligned. |

All frame counters are `uint32_t` and wrap harmlessly, because every consumer uses differences.

### `audio_spdif_config_t`

Hardware configuration passed to `audio_spdif_setup()`.

```c
typedef struct audio_spdif_config {
    uint8_t pin;
    uint8_t dma_channel;
    uint8_t dma_reload_channel; // RP2040 only: reload partner (ignored on RP2350)
    uint8_t pio_sm;
    uint8_t pio;        // PIO block index (0, 1, or 2 on RP2350)
    void   *ring_base;  // caller-owned per-slot ring storage,
                        // PICO_AUDIO_SPDIF_RING_BYTES, aligned to its size;
                        // shared with the slot's I2S instance across retypes
} audio_spdif_config_t;
```

- `pin`: any valid GPIO.
- `dma_channel`: unique. It becomes `instance_index`, so it should stay fixed for a given output across teardown and setup.
- `dma_reload_channel`: RP2040 only, unique, claimed and released alongside `dma_channel`.
- `pio_sm`: 0 to 3, unique within its block.
- `pio`: 0 or 1 on RP2040, 0 to 2 on RP2350. Any other value panics.
- `ring_base`: non-NULL and aligned to `PICO_AUDIO_SPDIF_RING_BYTES`. Both are asserted. The library never frees it.

---

## API Reference

### Lifecycle

#### `audio_spdif_setup`

```c
void audio_spdif_setup(audio_spdif_instance_t *inst, uint32_t sample_freq,
                       const audio_spdif_config_t *config);
```

Initializes an instance. It must be called before any other function on that instance.

**What it does**

1. Builds the shared NRZI lookup table on the first call.
2. Copies the hardware config into the instance and sets `instance_index` to `dma_channel`.
3. Takes the ring from `config->ring_base` and asserts its alignment.
4. Fixes the instance's word format (`pio_bmc`). It uses the block's loaded encoder if a program is resident, else the block's requested encoder.
5. Resets the ring, which stamps the whole ring with silence in that format.
6. Initializes the GPIO for the PIO block and claims the SM.
7. Loads the selected program if the block has none. It panics with a message naming the program and block if the program does not fit.
8. Initializes the SM and programs the nominal divider for `sample_freq`.
9. Claims the DMA channel, plus the reload channel on RP2040. The DMA itself is configured at each arm.
10. Adds the instance to the registry.

The instance is left disabled. Audio reaches it only through `audio_spdif_ring_write_s32()`.

#### `audio_spdif_set_enabled`

```c
void audio_spdif_set_enabled(audio_spdif_instance_t *inst, bool enabled);
```

Starts or stops one instance. A call with the current state is a no-op.

- **Enable** clears the SM FIFOs, arms the DMA at the ring position matching `consumed_words`, then enables the SM. The reader therefore resumes exactly where it stopped, or at frame 0 after a ring reset.
- **Disable** stops the SM first, then stops the DMA.

To start several instances aligned, use `audio_spdif_enable_sync()` instead.

#### `audio_spdif_change_pin`

```c
void audio_spdif_change_pin(audio_spdif_instance_t *inst, uint new_pin);
```

Moves a disabled instance (asserted) to a new GPIO. It releases the old pin to high-Z, but only while the pin is still muxed to this instance's PIO, then does everything `audio_spdif_attach_pin()` does.

#### `audio_spdif_attach_pin`

```c
void audio_spdif_attach_pin(audio_spdif_instance_t *inst, uint new_pin);
```

Same as `audio_spdif_change_pin()` without touching the old pin. It is for callers that release all outgoing pins themselves before claiming incoming ones, so a pin swap between two outputs cannot release a pin the other output just took. It stops any stale DMA, initializes the new pin, re-initializes the SM with the block's loaded program and restores the divider. The instance must be disabled (asserted).

#### `audio_spdif_teardown`

```c
void audio_spdif_teardown(audio_spdif_instance_t *inst);
```

Releases everything setup claimed so the same SM and DMA channel can be claimed by another output type on the same slot. In order, it disables the instance (or stops the DMA if already disabled), sets `ring` to NULL, releases the pin, unclaims the DMA channel or channels and the SM, and removes the instance from the registry. If it was the last S/PDIF instance on its block, it also removes the block's program, so the next setup can load whichever encoder is selected by then. The ring storage is untouched and can be passed to the next setup.

### Ring Output

#### `audio_spdif_ring_write_s32`

```c
void audio_spdif_ring_write_s32(audio_spdif_instance_t *inst,
                                const int32_t *interleaved_lr, uint32_t frames);
```

Encodes `frames` interleaved L/R samples into the ring. Each sample carries 24-bit audio in bits [23:0]. Higher bits are ignored.

The write is all-or-nothing. If the block does not fit in the writable window (`RING_FRAMES - ERASE_AHEAD - fill`), nothing is written, `drop_frames` grows by `frames`, and only the servo snapshot is refreshed. A partial write would advance this instance by a different amount than its peers. Callers that run several instances must check that every instance has room before writing any of them (see [Underrun Handling](#underrun-handling)).

On success it stamps each frame with its IEC block phase, extends the silence margin to `ERASE_AHEAD` frames past the new audio, records `recovery_prewrite_fill_frames`, and latches the `snap_*` pair.

The writer works whether or not the instance is enabled, which is how a ring is prefilled before a synchronized start.

#### `audio_spdif_ring_consumed_words` and `audio_spdif_ring_consumed_frames`

```c
uint32_t audio_spdif_ring_consumed_words(audio_spdif_instance_t *inst);
uint32_t audio_spdif_ring_consumed_frames(audio_spdif_instance_t *inst);
```

Monotonic DMA consumption since the last ring reset, in 32-bit words (4 per frame) and in frames. Each call unwraps the hardware read address under a short interrupt-off section. Use frames for all fill and deficit arithmetic. The words counter wraps at 2^32 words, which is not coherent with `wr_frames`.

#### `audio_spdif_ring_produced_frames` and `audio_spdif_ring_fill_frames`

```c
static inline uint32_t audio_spdif_ring_produced_frames(audio_spdif_instance_t *inst);
uint32_t audio_spdif_ring_fill_frames(audio_spdif_instance_t *inst);
```

Produced is `wr_frames`. Fill is produced minus consumed. When the reader has overtaken the writer the fill is negative, so read it as `int32_t` before comparing.

#### `audio_spdif_ring_reset`

```c
void audio_spdif_ring_reset(audio_spdif_instance_t *inst);
```

Re-anchors all accounting at ring position 0 and restarts IEC block framing from frame 0. It also points the stopped DMA's read address back at the ring base and stamps the whole ring with silence in the instance's current format. The DMA must be stopped. Use it after disabling an instance and before a synchronized restart, for example after a rate change.

#### `audio_spdif_ring_skip`

```c
void audio_spdif_ring_skip(audio_spdif_instance_t *inst, uint32_t frames);
```

The group re-anchor primitive. It advances the write position by `frames` without writing audio, counts them in `underrun_frames`, and restamps silence through the new margin. The skipped region already holds silence or is restamped, so the reader plays silence across it. To keep instances aligned, the caller applies the same skip to every enabled instance.

#### `audio_spdif_ring_silence_pump`

```c
void audio_spdif_ring_silence_pump(audio_spdif_instance_t *inst, uint32_t max_frames);
```

Extends the silence region toward one full ring ahead of the reader, stamping at most `max_frames` per call. It only stamps frames past the last written audio, so pending audio always plays exactly once. While audio is streaming it is a near no-op. Call it periodically when no producer is running, and from a timer when the caller's main loop may stall for longer than a ring lap.

### Encoder Selection

#### `audio_spdif_select_encoder`

```c
void audio_spdif_select_encoder(uint pio_index, bool pio_bmc);
```

Requests the encoder for a PIO block. True selects the 16-instruction PIO biphase-mark encoder, false the 4-instruction NRZI program with software words. The request takes effect at the next program load on that block, which is the first `audio_spdif_setup()` after the block's last teardown. Instances already set up keep their format. The default for every block is the PIO encoder.

#### `audio_spdif_loaded_encoder`

```c
int audio_spdif_loaded_encoder(uint pio_index);
```

Returns 1 if the PIO encoder is loaded on the block, 0 for the NRZI program, and -1 if no S/PDIF program is loaded (no instance set up on that block).

### Synchronized Start

#### `audio_spdif_enable_sync`

```c
void audio_spdif_enable_sync(audio_spdif_instance_t *instances[], uint count);
```

Starts `count` instances (1 to `PICO_AUDIO_SPDIF_MAX_INSTANCES`) on the same system clock cycle. It runs `audio_spdif_enable_sync_prepare()` and then a single `pio_enable_sm_mask_in_sync()` with interrupts disabled.

#### `audio_spdif_enable_sync_prepare`

```c
uint32_t audio_spdif_enable_sync_prepare(audio_spdif_instance_t *instances[], uint count);
```

The prepare-only half. It rewinds every SM (clears its FIFOs, restarts it, and jumps to the loaded program's entry point), arms every DMA, and marks the instances enabled without starting any SM. It returns the SM mask for the block. The caller then performs one `pio_enable_sm_mask_in_sync()` that can also include other SMs on the same block. DSPi uses this to start S/PDIF and I2S slots in one write. All instances must be on the same PIO block (asserted).

### Rate

#### `audio_spdif_apply_pio_frequency`

```c
void audio_spdif_apply_pio_frequency(audio_spdif_instance_t *inst, uint32_t sample_freq);
```

Writes the nominal divider for `sample_freq` to the instance's SM, updates `inst->freq`, and sets the channel-status sample-rate byte. Nothing in the library retunes on its own, so a rate change must call this for every set-up instance and then restart the outputs in sync (see [Sample Rate Setting](#sample-rate-setting)).

### Diagnostics

```c
void     audio_spdif_reset_dma_starvations(void);
uint32_t audio_spdif_get_dma_starvations(void);
uint32_t audio_spdif_get_dma_starvations_instance(uint index);
```

The total getter sums `underrun_frames` over all registered instances. The per-instance getter returns `underrun_frames` for the registered instance whose `instance_index` matches, or 0 if none does. The reset clears `underrun_frames` and `drop_frames` on every registered instance. All values are in frames.

---

## Encoders

Every frame occupies four 32-bit ring words (two per subframe) in both formats, and both programs take exactly 256 PIO cycles per frame. The ring geometry, DMA accounting, divider and start sequence are therefore identical whichever encoder is loaded.

| | PIO encoder (default) | Software NRZI |
|---|---|---|
| Program | `audio_spdif_bmc`, 16 instructions | `audio_spdif`, 4 instructions |
| Word A | Control word. Preamble as lead and trail low-run counts in bits [2:0] and [5:3]. | Preamble and the first part of the NRZI-coded subframe. |
| Word B | Data word. Sample in [23:0], V in [24], U in [25], C in [26]. | Rest of the NRZI-coded subframe, including the parity bit. |
| Parity | Generated by the PIO from the line level | Computed in software |
| CPU cost per frame | Four stores and two masks | Six table lookups and two parity computations |

The PIO encoder is used unless its 16 instructions do not fit beside the other programs on the block. In DSPi that happens in exactly one configuration. When an external I2S master clocks the device and at least one output slot is I2S, PIO0 holds the 26-instruction I2S external-clock program, so the S/PDIF slots switch to the NRZI program. `process_type_switches()` in `main.c` makes that choice before any teardown or setup, and treats an encoder change as a full output rebuild. The rule, word format, program listing and equivalence proof are in `Documentation/Features/spdif_pio_bmc_encoder_spec.md`.

Both entry points emit two low cycles before the first preamble bit, so a synchronized start lands on the same frame edge with either program.

---

## Usage Guide

### Two Outputs With Aligned Start

```c
#include "pico/audio_spdif.h"

static uint8_t __attribute__((aligned(PICO_AUDIO_SPDIF_RING_BYTES)))
    rings[2][PICO_AUDIO_SPDIF_RING_BYTES];

static audio_spdif_instance_t out_a, out_b;   // static, so zero-initialized

static const audio_spdif_config_t cfg_a = {
    .pin = 6, .dma_channel = 0, .dma_reload_channel = 6,   // reload used on RP2040 only
    .pio_sm = 0, .pio = 0, .ring_base = rings[0],
};
static const audio_spdif_config_t cfg_b = {
    .pin = 7, .dma_channel = 1, .dma_reload_channel = 7,
    .pio_sm = 1, .pio = 0, .ring_base = rings[1],
};

void outputs_init(void) {
    audio_spdif_setup(&out_a, 48000, &cfg_a);
    audio_spdif_setup(&out_b, 48000, &cfg_b);
    audio_spdif_instance_t *all[] = { &out_a, &out_b };
    audio_spdif_enable_sync(all, 2);   // both rings start as silence
}
```

### Feeding Blocks

Write every instance from the same block, and only when every instance can take the whole block. Otherwise one instance advances while another drops, and they stop being aligned.

```c
#define TARGET_FILL 384u   // re-anchor cushion, 8 ms at 48 kHz

void push_block(const int32_t *lr_a, const int32_t *lr_b, uint32_t frames) {
    audio_spdif_instance_t *all[] = { &out_a, &out_b };

    // A reader overtook its writer: re-anchor every instance by one identical skip.
    uint32_t deficit = 0;
    for (int i = 0; i < 2; i++) {
        int32_t d = (int32_t)(audio_spdif_ring_consumed_frames(all[i]) -
                              audio_spdif_ring_produced_frames(all[i]));
        if (d > (int32_t)deficit) deficit = (uint32_t)d;
    }
    if (deficit)
        for (int i = 0; i < 2; i++) audio_spdif_ring_skip(all[i], deficit + TARGET_FILL);

    // Admit the block only if it fits everywhere.
    for (int i = 0; i < 2; i++) {
        int32_t fill = (int32_t)audio_spdif_ring_fill_frames(all[i]);
        if (fill < 0) fill = 0;
        if ((uint32_t)fill + frames >
            PICO_AUDIO_SPDIF_RING_FRAMES - PICO_AUDIO_SPDIF_ERASE_AHEAD)
            return;
    }
    audio_spdif_ring_write_s32(&out_a, lr_a, frames);
    audio_spdif_ring_write_s32(&out_b, lr_b, frames);
}
```

When no producer is running, call `audio_spdif_ring_silence_pump()` on each enabled instance from the main loop so the reader never laps into stale content.

### How DSPi Uses It

- **Slots.** Each output slot has a fixed 16 KB ring in `slot_ring_store[]` (`usb_audio.c`), placed in the `.slot_rings` section at the top of RAM and not zeroed at boot. The slot's S/PDIF instance and its I2S instance (`pico_audio_i2s_multi`) share that store and the slot's SM and DMA channel. `audio_spdif_teardown()` and `audio_i2s_teardown()` hand them across on an output-type switch.
- **Producing.** `process_input_block()` (`audio_pipeline.c`) fills one 192-frame staging buffer per slot (`slot_stage[]`) from either core. `output_rings_pre_give()` runs the underrun re-anchor and the group admission across every enabled slot of both types and the PDM queue. If the block is admitted, `output_rings_write_block()` copies each staging buffer into its slot ring with `audio_spdif_ring_write_s32()` or `audio_i2s_ring_write_s32()`, chosen by `output_types[]`. Rings that are set up but not yet enabled are written too, so they prefill before a synchronized start. A rejected block is written nowhere.
- **Idle and stalls.** `output_rings_idle_pump()` runs the silence pump from the main loop while no producer runs. Around waits that can exceed a ring lap (flash writes, the DAC mute hold), the output-ring stall bracket runs `output_rings_stamp_silence_ahead()` from a 1 ms timer alarm. See "Output Ring Stall Bracket" in `current_architecture.md`.
- **Starting.** `start_output_slots_in_sync()` (`main.c`) is the only start path. It ORs the masks from `audio_spdif_enable_sync_prepare()` and `audio_i2s_enable_sync_prepare()` and starts every slot in one `pio_enable_sm_mask_in_sync()`. `audio_spdif_enable_sync()` is used once, at boot in `usb_sound_card_init()`.

---

## Synchronized Startup

### Why It Matters

DSPi's output slots feed separate DACs that together form a multichannel system. Any start offset between slots is a fixed time skew between channels, which breaks imaging, crossover summing and phase-coherent measurement. The firmware treats inter-slot alignment as inviolable.

### How It Works

`audio_spdif_enable_sync_prepare()` first rewinds every SM to its program entry with empty FIFOs. A stopped SM keeps a partial output shift register, while a re-initialized one starts empty, so without the rewind slots could start part of a frame apart. It then arms each DMA. The arm prefetches the head of each ring into the SM's TX FIFO and then stalls on DREQ. The FIFO prefetch is already counted as consumed, which is why the rewind has to come first.

A single `pio_enable_sm_mask_in_sync()` then starts every SM in the mask on the same system clock cycle and resets their fractional divider accumulators together. From that cycle on, every SM runs from the same clock with the same divider, so the slots stay in step for as long as they run.

### Rules

- All instances in one call must be on the same PIO block. This is asserted.
- Instances must be disabled, with their DMA stopped, before prepare.
- Prefill the rings first if the start should play audio rather than silence. The reader of each instance starts at its `consumed_words` position, which is frame 0 after a ring reset.
- Hold interrupts disabled from prepare to the mask write when mixing with other SMs, so nothing runs between priming one group and starting the clocks.
- After the start every instance is `enabled`. Use `audio_spdif_set_enabled(inst, false)` to stop one.

---

## Sample Rate Setting

The PIO runs at 256 times the sample rate. `audio_spdif_setup()` and `audio_spdif_apply_pio_frequency()` program the divider from `audio_base_divider_16_8(sample_freq)`, which is `round(clk_sys / sample_freq)` read as a 16.8 fixed-point divider. Every DSPi output clock (S/PDIF, I2S BCK, MCK, ADAT, PDM) derives from this one value by power-of-two scaling, so all outputs share the same rounding error and stay locked to each other.

At DSPi's default 307.2 MHz system clock the dividers come out as follows.

| Rate | Divider | Actual rate |
|------|---------|-------------|
| 44100 Hz | 27.2109 (6966/256) | 44099.91 Hz (-2 ppm) |
| 48000 Hz | 25.0 | 48000 Hz exact |
| 96000 Hz | 12.5 | 96000 Hz exact |

The library only ever programs nominal dividers. Input clock tracking in DSPi trims the system PLL itself, which moves every output together.

The channel-status sample-rate byte is set to the IEC 60958-3 code for 44.1, 48 or 96 kHz, and to "not indicated" for any other rate. The channel-status table is shared by all instances, so all S/PDIF outputs always advertise the same rate.

### Changing Rate

Nothing retunes lazily. A rate change in DSPi (`perform_rate_change()` in `main.c`) runs these steps.

1. Mute and fence the pipeline.
2. Set `output_rate_hz`, the rate every slot set up from then on uses.
3. Call `audio_i2s_update_all_frequencies()` for the I2S slots and `audio_spdif_apply_pio_frequency()` for every set-up S/PDIF slot.
4. Disable every slot, reset every ring (dropping old-rate audio and restarting IEC framing), and restart all slots in one synchronized start. The rings play silence until the producer refills them, unless an input's prefill handshake fills them before the start.

Changing the divider of one running instance on its own is not a supported operation, because that instance would drift against the others.

---

## Underrun Handling

The ring has three regions ahead of the reader. Real audio runs up to `wr_frames`. Encoded silence runs from there to `erased_frames`, at least `ERASE_AHEAD` (192) frames further. Beyond that is content from an earlier lap.

**Producer stalls briefly.** The reader plays out the remaining audio, then the silence margin. The S/PDIF stream stays valid and receivers keep lock.

**Producer stops.** The caller runs `audio_spdif_ring_silence_pump()` so the silence region keeps growing to a full ring ahead of the reader. Stale content from an earlier lap would not only replay old audio but also break IEC block framing, because 1024 is not a multiple of 192. Each replayed lap would sit 64 frames out of block step and could make receivers unlock.

**Reader overtakes writer.** When production resumes, consumed frames exceed `wr_frames` by some deficit. The caller measures the worst deficit across all enabled instances and calls `audio_spdif_ring_skip()` with the same length on every one. Each write position jumps past the silence the reader played, plus a cushion, and every instance moves by the same amount. The skipped frames are counted in `underrun_frames`.

**Ring full.** The writer refuses any block that does not fit entirely and counts it in `drop_frames`. Callers that pre-check every instance, as DSPi does, never reach this case.

The library does nothing on its own when the reader overtakes the writer. The DMA keeps running and the reader keeps playing the silence stamped ahead of it. Recovery happens on the caller's next block.

---

## Resource Allocation Map

### DSPi

All output slots live on PIO0, the block named by `PICO_AUDIO_SPDIF_PIO`. A slot's I2S instance reuses the same SM, DMA channel and ring.

| Slot | Outputs | PIO | SM | DMA | Reload DMA (RP2040) | Default GPIO | Ring |
|------|---------|-----|----|-----|---------------------|--------------|------|
| 0 | 1-2 | PIO0 | 0 | 0 | 6 | 6 | `slot_ring_store[0]` |
| 1 | 3-4 | PIO0 | 1 | 1 | 7 | 7 | `slot_ring_store[1]` |
| 2 | 5-6 (RP2350) | PIO0 | 2 | 2 | none | 8 | `slot_ring_store[2]` |
| 3 | 7-8 (RP2350) | PIO0 | 3 | 3 | none | 9 | `slot_ring_store[3]` |

Reload channels are `OUTPUT_SLOT_RELOAD_DMA_BASE + slot` (`config.h`). Ring storage totals 32 KB on RP2040 and 64 KB on RP2350.

PIO0 instruction memory is shared with the I2S output programs.

| Configuration | PIO0 contents | S/PDIF encoder |
|---------------|---------------|----------------|
| No I2S slots | S/PDIF 16 | PIO |
| I2S slots, internal clocking | S/PDIF 16 + I2S clkout 8 + dataout 8 = 32 | PIO |
| I2S slots, external clock master | S/PDIF 4 + I2S extclk 26 = 30 | Software NRZI |

The library claims no DMA IRQ line and no timer. The stall-bracket alarm belongs to DSPi.

---

## Constraints and Limitations

### Hard Constraints

1. **At most 4 instances.** Asserted in setup.
2. **Ring storage.** `PICO_AUDIO_SPDIF_RING_BYTES`, aligned to its size, valid for as long as the instance is set up. The DMA wraps the read address by masking low address bits, so misalignment would read outside the ring.
3. **Same block for a synchronized start.** `audio_spdif_enable_sync_prepare()` asserts that every instance shares one PIO block. Instances on different blocks cannot be started on the same cycle by this library.
4. **One encoder per block.** All S/PDIF instances on a block share one loaded program and one ring word format. The encoder changes only when the block's last instance is torn down and the next one is set up.
5. **Pin changes only while disabled.** Asserted in both pin functions.
6. **DMA must be stopped for a ring reset.** The reset rewrites the DMA read address.
7. **All-or-nothing writes.** The writer never splits a block. Multi-instance callers must admit a block only when every instance has room.
8. **Static instance structs.** The registry holds pointers until teardown.
9. **Input format.** 24-bit samples in bits [23:0] of interleaved L/R `int32_t`. Validity and user bits are always 0. Channel status is fixed to consumer, PCM, copy permitted, 24-bit word length, plus the rate byte.

### Soft Constraints

10. **Latency.** The ring holds 21.3 ms at 48 kHz. Usable depth is 832 frames after the silence margin. DSPi runs at a 384-frame target (8 ms at 48 kHz).
11. **IEC phase at counter wrap.** The block phase is the absolute frame count modulo 192. When `wr_frames` wraps at 2^32 (about 24.8 hours of uninterrupted streaming at 48 kHz, restarted by any ring reset) the phase steps by 64 frames once. Receivers resynchronize on the next Z preamble.
12. **PIO instruction memory.** In DSPi's internal-clock I2S configuration PIO0 has no free instruction, so any new PIO0 program needs the NRZI encoder or a smaller I2S program set.

---

## Internals

### Ring Geometry

A frame is two `spdif_subframe_t` entries (`l` and `h` words), 16 bytes in all. Frame `n` sits at ring index `n & (RING_FRAMES - 1)`. The IEC block phase is `n % 192`, carried as a running counter inside each stamping loop. It cannot be derived from ring position, because 192 does not divide 1024.

### Free-Running DMA

`ring_dma_arm()` configures the data channel for 32-bit reads with read increment, a read-address ring of `RING_BYTES`, the SM's TX DREQ and high priority. On RP2350 it triggers with the ENDLESS transfer count. On RP2040 the data channel runs `RING_WORDS` transfers and chains to the reload channel, which writes `RING_WORDS` back into the data channel's count-and-trigger register. The reload source word is a non-const RAM variable, so the chain keeps running while flash is unavailable. The arm sets the read address and the unwrap anchor together under an interrupt-off section, so the next consumed read sees zero delta.

`ring_dma_stop()` on RP2040 first points the data channel's chain at itself to break the loop, then aborts both channels and waits for them to go idle. On RP2350 it aborts the channel and clears the transfer count so a later normal-mode user of the channel does not inherit ENDLESS mode.

### Consumed-Side Accounting

`audio_spdif_ring_consumed_words()` reads the DMA read address and adds the forward delta modulo the ring size. A poll that comes more than half a ring late could miss whole laps, so the function also checks elapsed time against the programmed rate and adds any whole laps that time says occurred. The address supplies the exact in-ring remainder. The lap correction only runs while the instance is enabled and its SM is actually running, so the window between prepare and a gated start cannot inject phantom laps. A fast path skips the 64-bit divide whenever less than half a ring of time has passed. A remainder byte carries partial frames between calls so `consumed_frames` advances exactly once per frame.

### RAM Placement

The writer, skip, silence pump and the consumed and fill readers are marked `SPDIF_TIME_CRITICAL` (section `.time_critical`) on both platforms. The stamping helpers are forced inline into them. They run from the RAM-resident pipeline and from DSPi's stall alarm, which must keep working while flash is unavailable. `scripts/check_ram_placement.py` enforces this for DSPi builds.

### Shared Global State

| Variable | Purpose |
|----------|---------|
| `spdif_lookup[256]` | NRZI encoding table for the software encoder. Built on the first setup. Non-static because `sample_encoding.h` reads it. |
| `spdif_channel_status[5]` | IEC 60958-3 consumer channel status. Byte 3 is the rate code. Shared by all instances. |
| `pio_program_offset[3]` | Load offset of the S/PDIF program per PIO block, or -1 when none is loaded. |
| `spdif_pio_encoder_requested[3]` | Encoder requested per block by `audio_spdif_select_encoder()`. Defaults to the PIO encoder. |
| `spdif_pio_encoder_loaded[3]` | Encoder actually loaded per block. |
| `spdif_instances[]`, `spdif_instance_count` | Registry of set-up instances, used by the starvation getters and to decide when a block's program can be released. |
| `spdif_ring_words_const` | RP2040 only. Reload source word for the free-running DMA pair. |

---

## Troubleshooting

### No audio output

1. Check the GPIO matches the hardware and that the instance was started with `audio_spdif_enable_sync()` or `audio_spdif_set_enabled()`.
2. Check that blocks are being admitted. A rising `drop_frames` means the caller wrote while the ring was full. A flat `wr_frames` means nothing is being written.
3. A receiver that locks but plays silence usually means the writer is behind the reader and the caller is not re-anchoring with `audio_spdif_ring_skip()`.

### Clicks or silence gaps

1. Read the starvation counters. They are in frames, and each underrun adds the full re-anchor length.
2. Check the producer completes every block within its period, and that main-loop waits longer than a ring lap (21.3 ms at 48 kHz, 10.7 ms at 96 kHz) keep a silence pump running from a timer.

### Receiver loses lock after a stall

The reader has replayed an earlier lap, and that lap's IEC block framing is out of step. Keep silence stamped ahead of the reader through the stall with `audio_spdif_ring_silence_pump()`.

### One output is offset from another

Start all instances with one synchronized start, and write every block to every instance or to none. Never write one instance on its own, and never change one instance's divider on its own.

### Panic "S/PDIF ... program does not fit in PIO*n*"

The block's instruction memory is full. In DSPi this means the encoder selection did not drop to the NRZI program before the I2S external-clock program was loaded. Elsewhere, move the instance to another block or select the NRZI encoder with `audio_spdif_select_encoder()` before the block's first setup.

### Assertion failure in `audio_spdif_setup`

`spdif_instance_count < PICO_AUDIO_SPDIF_MAX_INSTANCES` means a fifth instance was set up, or an instance was set up twice without a teardown. An alignment assertion means `ring_base` is not aligned to `PICO_AUDIO_SPDIF_RING_BYTES`.

### DMA channel conflicts

The library claims its channels at setup and panics through the SDK if one is already claimed. In DSPi the slot channels are claimed in `usb_sound_card_init()` before the USB stack starts, and the remaining channel numbers are fixed in `config.h`.
