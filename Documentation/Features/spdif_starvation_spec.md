# SPDIF Starvation Metric Spec

*Last updated: 2026-10-12*

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

- `spdif_underruns` (`REQ_GET_STATUS wValue=8`): USB packet-gap heuristic.
- `REQ_GET_STATUS wValue=7` is retired and always reads 0.

## 3. USB API

Use the existing vendor status request:

- `bmRequestType`: `0xC1` (IN, Vendor, Interface)
- `bRequest`: `0x50` (`REQ_GET_STATUS`)
- `wIndex`: `2` (vendor interface)
- `wLength`: `4`

Response format is a little-endian `uint32_t`.

### 3.1 New `wValue` mappings

- `17`: total SPDIF DMA starvations (all SPDIF instances combined)
- `18`: SPDIF instance 0 starvations (Output 1/2)
- `19`: SPDIF instance 1 starvations (Output 3/4)
- `20`: SPDIF instance 2 starvations (Output 5/6, RP2350 only)
- `21`: SPDIF instance 3 starvations (Output 7/8, RP2350 only)

## 4. Counter Lifecycle

1. Monitoring is enabled when USB audio streaming becomes active (`alt > 0`).
2. Counters reset automatically when stream transitions from inactive to active (`alt 0 -> alt 1/2`).
3. Counters are not reset when changing between 16-bit and 24-bit active alt settings.
4. Counters are not persistent across reboot.

## 5. App Integration (Step-by-Step)

1. Start polling `wValue=17` once per second while stream is active.
2. Keep previous sample and compute `delta` each poll.
3. If `delta > 0`, immediately read `wValue=18..21` to locate which output starved.
4. Log timestamp, total delta, per-instance values, current sample rate (`wValue=15`), and the packet-gap counter (`wValue=8`).
5. Show total and per-output counters in your diagnostics UI.

Use unsigned wrap-safe delta:

```c
uint32_t delta = current - previous; // uint32_t wrap-safe arithmetic
```

## 6. Minimal Host Pseudocode

```c
uint32_t get_status_u32(Device dev, uint16_t wValue) {
    // control IN transfer, 4-byte response
    // bmRequestType=0xC1, bRequest=0x50, wValue=wValue, wIndex=2, wLength=4
    uint8_t b[4] = control_in(dev, 0xC1, 0x50, wValue, 2, 4);
    return (uint32_t)b[0] |
           ((uint32_t)b[1] << 8) |
           ((uint32_t)b[2] << 16) |
           ((uint32_t)b[3] << 24);
}

void poll_starvation(Device dev) {
    static bool has_prev = false;
    static uint32_t prev_total = 0;

    uint32_t total = get_status_u32(dev, 17);
    if (has_prev) {
        uint32_t delta = total - prev_total;
        if (delta > 0) {
            uint32_t s0 = get_status_u32(dev, 18);
            uint32_t s1 = get_status_u32(dev, 19);
            uint32_t s2 = get_status_u32(dev, 20);
            uint32_t s3 = get_status_u32(dev, 21);
            uint32_t spdif_underruns = get_status_u32(dev, 8);
            uint32_t sample_rate_hz  = get_status_u32(dev, 15);
            log_event(total, delta, s0, s1, s2, s3,
                      spdif_underruns, sample_rate_hz);
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

Older firmware may not support this metric. If unsupported, values may stay at zero indefinitely.

Recommended behavior in app:

1. Treat all-zero values as "no detected starvation" (not as guaranteed unsupported).
2. Keep feature visible but non-blocking.
3. Include firmware version/platform in logs when possible.
