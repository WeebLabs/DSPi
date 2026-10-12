# Buffer Fill Level Statistics

*Last updated: 2026-10-12*

## Overview

Real-time fill level monitoring for the output slot rings (S/PDIF or I2S, one per slot) and the PDM buffers in the DSPi firmware. Enables host applications to diagnose audio glitches, near-miss underruns, and pipeline health via USB vendor commands.

Each slot's fill is the number of frames written into its ring and not yet read by the free-running output DMA. The wire format predates the rings and reports in "buckets" of 48 frames on a 16-bucket (768-frame) scale. The same bucket unit drives every prefill threshold and the clock servo target, so a reading here means the same thing as the firmware's own decisions. The pipeline stages each block in a fixed per-slot buffer before copying it into the rings, and that staging is always empty between blocks, so it is not reported.

## Wire Format

### BufferStatsPacket (44 bytes)

Fits in a single 64-byte USB control transfer — no streaming transfer complexity needed.

```
Offset  Size  Field
0       1     num_spdif          NUM_SPDIF_INSTANCES (2 on RP2040, 4 on RP2350)
1       1     flags              Bit 0: PDM active, Bit 1: audio streaming
2       2     sequence           Monotonic counter (wraps at 65535)
4       32    spdif[4]           Per-instance SPDIF stats (unused entries zeroed)
36      8     pdm                PDM buffer stats
```

### SpdifBufferStats (8 bytes per instance)

```
Offset  Size  Field
0       1     consumer_free          [0-16] 16 minus consumer_prepared
1       1     consumer_prepared      [0-16] ring fill in 48-frame buckets (clamped)
2       1     consumer_playing       [0-1] slot output enabled
3       1     consumer_fill_pct      consumer_prepared as % of 16 buckets
4       1     consumer_min_fill_pct  Lowest consumer_fill_pct since last reset
5       1     consumer_max_fill_pct  Highest consumer_fill_pct since last reset
6-7     2     fill_centi_pct         Sample-granular fill, 0.01% of 768 frames (u16 LE)
```

`fill_centi_pct` can exceed 10000 because a ring holds more than 768 frames. It saturates at 65535. All slot fields read zero while a slot is not set up. During an output-type switch `consumer_free` reads 16 and the rest read zero.

### PdmBufferStats (8 bytes)

```
Offset  Size  Field
0       1     dma_fill_pct       DMA circular buffer fill percentage
1       1     dma_min_fill_pct   Lowest DMA fill since last reset
2       1     dma_max_fill_pct   Highest DMA fill since last reset
3       1     ring_fill_pct      Software ring buffer fill percentage
4       1     ring_min_fill_pct  Lowest ring fill since last reset
5       1     ring_max_fill_pct  Highest ring fill since last reset
6-7     2     pad                Reserved (zero)
```

## Vendor Commands

### REQ_GET_BUFFER_STATS (0xB0)

- **Direction:** GET (IN)
- **wValue:** Unused
- **wLength:** 44
- **Response:** 44-byte `BufferStatsPacket`

Returns a snapshot of all buffer fill levels and watermarks. The `sequence` counter increments on each call, allowing the host to detect missed polls.

### REQ_RESET_BUFFER_STATS (0xB1)

- **Direction:** GET (IN)
- **wValue:** Reset flags (bit 0: reset watermarks)
- **wLength:** 1
- **Response:** 1-byte success (0x01)

Resets min watermarks to 100 and max watermarks to 0 for all SPDIF instances and PDM.

## Fill Percentage Formulas

### Slot Ring Fill

```
consumer_prepared = min(fill_frames / 48, 16)
consumer_fill_pct = consumer_prepared * 100 / 16
fill_centi_pct    = min(fill_frames * 10000 / 768, 65535)
```

- **Scale:** 16 buckets of 48 frames (`SPDIF_CONSUMER_BUFFER_COUNT`), 768 frames in all
- **Healthy range:** around 50% (the servo target is 8 buckets, 384 frames, 8 ms at 48 kHz)
- **Warning:** Sustained 0% indicates underrun risk. 100% means the ring is at or beyond the top of the scale

### PDM DMA Fill

```
fill_pct = ((write_idx - read_idx) & (PDM_DMA_BUFFER_SIZE - 1)) * 100 / PDM_DMA_BUFFER_SIZE
```

- **Capacity:** 2048 words (DMA circular buffer)
- **Healthy range:** ~12.5% (TARGET_LEAD = 256 words ahead of DMA read)
- **Warning:** >50% triggers underrun recovery in firmware; <2% risks DMA overrun

### PDM Ring Fill

```
fill_pct = ((head - tail) & 0xFF) * 100 / RING_SIZE
```

- **Capacity:** 256 entries (uint8_t wrap-around)
- **Healthy range:** 0-10% (typically 0-2 entries queued)
- **Warning:** High fill indicates Core 1 PDM processing is falling behind

## Watermark Behavior

- Watermarks track the minimum and maximum fill percentages observed since the last reset
- Updated on every eighth pipeline block in `process_input_block()` (about every 8 ms)
- Slot ring watermarks tracked per slot, whichever output type the slot has
- Only updated while the pipeline is processing blocks (watermarks freeze when the input stops)
- PDM watermarks only update while PDM is enabled
- Watermarks reset via `REQ_RESET_BUFFER_STATS` (0xB1) with wValue bit 0 set
- Also reset at firmware initialization (`usb_sound_card_init()`)

## Flags Field

| Bit | Name | Description |
|-----|------|-------------|
| 0 | PDM active | PDM output is enabled (`pdm_enabled` flag) |
| 1 | Audio streaming | USB audio packets are being received (`sync_started` flag) |
| 2-7 | Reserved | Always 0 |

## Application Patterns

### Periodic Polling

Poll `REQ_GET_BUFFER_STATS` every 100-500ms to build a time series. Use the `sequence` counter to detect missed polls (gap > 1 between consecutive reads).

### Glitch Investigation

1. Reset watermarks via `REQ_RESET_BUFFER_STATS`
2. Reproduce the audio glitch
3. Read `REQ_GET_BUFFER_STATS` — check `consumer_min_fill_pct` for near-zero values indicating underrun

### Health Dashboard

| Metric | Good | Warning | Critical |
|--------|------|---------|----------|
| Consumer fill_pct | 25-75% | <25% or >75% | 0% or 100% |
| PDM DMA fill_pct | 8-20% | <5% or >30% | >50% (underrun recovery) |
| PDM ring fill_pct | 0-10% | >20% | >50% (Core 1 overloaded) |
| consumer_min_fill_pct | >0% | 0% | Sustained 0% |

## Implementation Details

### Ring Fill Reading

The fill is the ring's produced frame count minus its consumed frame count (`audio_spdif_ring_fill_frames()` or `audio_i2s_ring_fill_frames()`). The consumed count is unwrapped from the DMA read address under a brief interrupt-off section, so a reading costs one register read and a few arithmetic operations. A reader that has overtaken the writer reads as zero fill.

### PDM Write Index

`pdm_stats_write_idx` is a `volatile uint32_t` in `pdm_generator.c`, written by Core 1 after each 8-chunk sigma-delta pass, read by Core 0 for stats. Single-word writes are atomic on ARM Cortex-M0+/M33 — no synchronization needed.

### Overhead

Watermark update runs on every eighth pipeline block. Overhead is one ring fill reading per slot plus the PDM DMA address read, negligible against the per-block processing budget.

### BSS Impact

~18 bytes total across both platforms (consumer watermark arrays, sequence counter, pdm_stats_write_idx).
