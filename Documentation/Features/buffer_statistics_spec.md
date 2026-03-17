# Buffer Fill Level Statistics

## Overview

Real-time buffer fill level monitoring for SPDIF consumer (DMA-side) pools and PDM buffers in the DSPi firmware. Enables host applications to diagnose audio glitches, near-miss underruns, and pipeline health via USB vendor commands.

Producer (USB-side) pool stats are not tracked because `producer_pool_blocking_give` returns the producer buffer to the free list synchronously within `give_audio_buffer()` — the producer pool is always fully free between USB packets, so instantaneous and watermark values would always read 0%.

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
0       1     consumer_free          [0-4] SPDIF buffers available for DMA
1       1     consumer_prepared      [0-4] SPDIF buffers queued for DMA playback
2       1     consumer_playing       [0-1] DMA in-flight buffer
3       1     consumer_fill_pct      Consumer pipeline fill %
4       1     consumer_min_fill_pct  Lowest consumer fill since last reset
5       1     consumer_max_fill_pct  Highest consumer fill since last reset
6-7     2     pad                    Reserved (zero)
```

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

### SPDIF Consumer Fill (DMA side)

```
consumer_fill_pct = (consumer_prepared + consumer_playing) * 100 / (AUDIO_BUFFER_COUNT / 2)
```

- **Capacity:** `AUDIO_BUFFER_COUNT / 2` = 4 consumer buffers per instance
- **Healthy range:** 25-75% (1-3 buffers queued)
- **Warning:** Sustained 0% indicates underrun risk; sustained 100% indicates overrun risk

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
- Updated once per USB audio packet (~1ms) in `process_audio_packet()`
- Consumer SPDIF pool watermarks tracked per instance
- Only updated while audio is streaming (watermarks freeze when USB audio stops)
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

### List Counting

`audio_buffer_list_count()` is a `static inline` function in `pico/audio.h` that traverses a buffer linked list under the caller's spinlock. Lists have max 4 nodes (consumer pool size); traversal takes ~50ns.

### PDM Write Index

`pdm_stats_write_idx` is a `volatile uint32_t` in `pdm_generator.c`, written by Core 1 after each 8-chunk sigma-delta pass, read by Core 0 for stats. Single-word writes are atomic on ARM Cortex-M0+/M33 — no synchronization needed.

### Overhead

Watermark update runs once per USB packet (~1ms). Overhead: ~1-2us total (4-8 spinlock-protected list traversals of max 4 nodes each for consumer pools, plus PDM DMA address read). Negligible vs the 500-800us packet processing budget.

### BSS Impact

~18 bytes total across both platforms (consumer watermark arrays, sequence counter, pdm_stats_write_idx).
