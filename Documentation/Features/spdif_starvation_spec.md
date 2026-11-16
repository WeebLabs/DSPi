# SPDIF Starvation Metric Spec

*Last updated: 2026-11-16 (read through `SYS.SPDIF_TX_STATS` on the paged protocol of firmware 1.1.7)*

## 1. Purpose

This document explains how a host app can read and use the **SPDIF DMA starvation** metric.

In plain language: each S/PDIF output slot plays from a ring that its DMA reads continuously. If the output reader catches up with the audio the pipeline has written, it plays the framed silence kept ahead of the audio instead. The firmware then moves every slot's write position forward by the same amount so the slots stay aligned, and it adds that number of frames to this counter.

This metric is useful for diagnosing occasional single dropouts that are not always visible in older counters.

## 2. What The Counter Means

A starvation event means:

1. The output reader of at least one enabled slot overtook that slot's writer.
2. The reader played framed silence for the missing span.
3. On the next pipeline block the firmware re-anchored every enabled slot's write position by one identical skip.
4. Each S/PDIF slot's counter grew by the skip length in frames.

This is a **direct output-side starvation** signal. The counters are in **frames**, not events, so one dropout typically adds hundreds. Only slots typed S/PDIF report here. A slot typed I2S reads 0 for its index.

It is different from:

- `spdif_underruns` (offset 0 of `SYS.SPDIF_TX_STATS`): USB packet-gap heuristic.
- The 1.1.6 `GET_STATUS` selector 7, which always read 0, is retired with no replacement.

## 3. USB API

Firmware 1.1.7 reports every S/PDIF output counter in one struct read on the paged protocol (`../commands.md`). It replaces the 1.1.6 `GET_STATUS` selectors 8 and 17 to 21.

| Command | Page | Cmd | Dir | Address (`wValue`) | Size |
|---|---|---|---|---|---|
| `SYS.SPDIF_TX_STATS` | 0x01 | 0x45 | GET | none (0) | GET 24 |

On USB that is `bmRequestType` `0xC1`, `bRequest` `0x45`, `wValue` 0, `wIndex` `0x0102` (page 0x01 in the high byte, interface 2 in the low byte), `wLength` 24.

### 3.1 Reply layout

All fields are little-endian `uint32_t`.

| Offset | Field |
|---|---|
| 0 | `spdif_underruns`, the USB packet-gap heuristic |
| 4 | Total SPDIF DMA starvations (all SPDIF instances combined) |
| 8 | SPDIF instance 0 starvations (Output 1/2) |
| 12 | SPDIF instance 1 starvations (Output 3/4) |
| 16 | SPDIF instance 2 starvations (Output 5/6, RP2350 only, 0 on RP2040) |
| 20 | SPDIF instance 3 starvations (Output 7/8, RP2350 only, 0 on RP2040) |

The current sample rate is the u32 at offset 4 of `SYS.SYSTEM_STATUS` (page 0x01, command 0x40, 16 bytes).

## 4. Counter Lifecycle

1. Monitoring is enabled when USB audio streaming becomes active (`alt > 0`).
2. Counters reset automatically when stream transitions from inactive to active (`alt 0 -> alt 1/2`).
3. Counters are not reset when changing between 16-bit and 24-bit active alt settings.
4. Counters are not persistent across reboot.

## 5. App Integration (Step-by-Step)

1. Read `SYS.SPDIF_TX_STATS` once per second while stream is active.
2. Keep the previous total (offset 4) and compute `delta` each poll.
3. If `delta > 0`, the per-instance fields of the same reply (offsets 8 to 20) show which output starved.
4. Log timestamp, total delta, per-instance values, current sample rate (`SYS.SYSTEM_STATUS` offset 4), and the packet-gap counter (offset 0).
5. Show total and per-output counters in your diagnostics UI.

Use unsigned wrap-safe delta:

```c
uint32_t delta = current - previous; // uint32_t wrap-safe arithmetic
```

## 6. Minimal Host Pseudocode

```c
// Paged GET: bmRequestType 0xC1, bRequest = cmd, wIndex = (page << 8) | 2.
static uint32_t u32_at(const uint8_t *b, int off) {
    return (uint32_t)b[off] | ((uint32_t)b[off + 1] << 8) |
           ((uint32_t)b[off + 2] << 16) | ((uint32_t)b[off + 3] << 24);
}

void poll_starvation(Device dev) {
    static bool has_prev = false;
    static uint32_t prev_total = 0;

    uint8_t tx[24], sys[16];
    control_in(dev, 0xC1, 0x45, 0, 0x0102, tx, 24);    // SYS.SPDIF_TX_STATS
    uint32_t total = u32_at(tx, 4);
    if (has_prev) {
        uint32_t delta = total - prev_total;
        if (delta > 0) {
            control_in(dev, 0xC1, 0x40, 0, 0x0102, sys, 16);  // SYS.SYSTEM_STATUS
            log_event(total, delta,
                      u32_at(tx, 8), u32_at(tx, 12), u32_at(tx, 16), u32_at(tx, 20),
                      u32_at(tx, 0),     // spdif_underruns
                      u32_at(sys, 4));   // sample rate Hz
        }
    }
    prev_total = total;
    has_prev = true;
}
```

## 7. UI Guidance

Show these fields at minimum:

1. `SPDIF DMA Starvations (Total)`
2. `Out1/2`, `Out3/4`, `Out5/6`, `Out7/8` starvation counters
3. `Last delta` (how many new starved frames since last poll)
4. `Last event timestamp`

Add a warning indicator when `delta > 0`.

## 8. How To Interpret Results

1. Audible dropout and starvation delta increase together:
   starvation is confirmed as a real output-buffer miss.
2. Audible dropout with no starvation delta:
   issue is likely elsewhere (clock/relock, transport, analog chain, or other timing path).
3. Starvation increases only on one instance:
   focus investigation on that output slot/type-switch history and fill behavior.

## 9. Compatibility Notes

Every paged firmware (1.1.7 and later) has `SYS.SPDIF_TX_STATS`. Firmware 1.1.6 and older read these counters through `GET_STATUS` selectors and does not answer the paged protocol. Values may stay at zero for a long time on a healthy link.

Recommended behavior in app:

1. Treat all-zero values as "no detected starvation" (not as guaranteed unsupported).
2. Keep feature visible but non-blocking.
3. Include firmware version/platform in logs when possible.
