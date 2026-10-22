# DSPi AC-3 / DTS Bitstream Decoding Plan

*Version 0.1, 2026-10-22. Planning document; nothing implemented. RP2350 only. Branch `ac3-dts-decode`.*

This document records the practicality assessment, the cost estimates and the phased plan for decoding AC-3 and DTS core bitstreams received on the S/PDIF input. It is a plan, not a protocol specification. Command byte layouts, notify packet formats and preset fields are decided in Phase 3 and will be written into a separate spec at that point.

---

## 1. Summary

AC-3 decoding is practical on RP2350. The estimated cost is 7 to 12 % of core 0 at 307.2 MHz and about 20 KB of new RAM. DTS core decoding is feasible but costs roughly twice the CPU and three times the RAM, so it follows AC-3. RP2040 is excluded. It has no FPU and about 45 KB of free RAM, and core 0 already runs the Q28 pipeline sample by sample.

The decoder runs as sliced background work on core 0's main loop, the same way the spectrum analyser FFT does. Decoded 5.1 audio enters the pipeline as six ordinary input channels with a fixed delay anchored to the IEC 61937 burst start. The receiver, the soft VCXO and the output path are untouched.

The CPU figures are estimates from operation counts. Phase 1 measures them on hardware before any integration work is spent.

## 2. What S/PDIF can carry

A bitstream on S/PDIF is an IEC 61937 data burst. The carrier is an ordinary two-channel 16-bit PCM frame stream at the audio rate. The payload sits in the top 16 bits of each 24-bit subframe word. A burst starts with a four-word preamble.

| Word | Name | Value |
|---|---|---|
| 0 (left) | Pa | 0xF872 |
| 1 (right) | Pb | 0x4E1F |
| 2 (left) | Pc | burst info; bits 4:0 = data type, bit 7 = error flag |
| 3 (right) | Pd | payload length in bits (AC-3) or bytes (DTS) |

Relevant data types are 1 (AC-3), 11 (DTS type I, 512-sample burst), 12 (DTS type II, 1024), 13 (DTS type III, 2048), 0 (null) and 3 (pause). Bursts repeat at exactly the frame length in carrier samples, so an AC-3 burst repeats every 1536 frames and the payload occupies the first part of each period, with zero stuffing after it. At 640 kbps the AC-3 payload is 2560 bytes, which is about 42 % of the period.

Only AC-3 (up to 640 kbps) and DTS core (up to 1509 kbps) travel this way. E-AC-3, TrueHD, DTS-HD and DTS:X are HDMI-only. DTS 96/24 and DTS-ES decode as their 48 kHz 5.1 core. AC-3 at 32 kHz would need receiver support for that rate and is out of scope.

The channel-status non-audio bit (byte 0, bit 1) is set by most but not all sources. Detection must use the preamble, with the status bit as a confirming hint only.

## 3. Current hazard

When a bitstream arrives today the firmware plays it as PCM. That is full-scale noise on every output. Phase 0 fixes this on its own and should ship before any decoder exists.

## 4. Receiver and pipeline fit

- The S/PDIF receiver already delivers bit-exact 24-bit words (`(word & 0x0FFFFFF0) << 4` in `spdif_input_poll()`), so the 16-bit payload is `(word >> 12) & 0xFFFF`. Channel status is readable through `spdif_rx_get_c_bits()`.
- The soft VCXO trims the sys PLL to the carrier. Decoded samples map one-to-one onto carrier samples, so the rate logic needs no change and `process_input_block()` keeps receiving the carrier's sample count.
- `active_input_channel_count()` gains a case for decoded S/PDIF returning 6. The channel order is L, R, C, LFE, Ls, Rs in inputs 0 to 5. A 2/0 AC-3 stream delivers L and R only, with inputs 2 to 5 zeroed, so a matrix configured for stereo keeps working.
- Transitions between PCM and bitstream, and between 2 and 6 active inputs, change the input delay. They must go through the existing input resync path, the same one an input source switch uses, so output slot alignment is never at risk. All decoded channels pass through one ring with one delay, so they are aligned with each other by construction.

## 5. Where the decoder runs

Core 1 is owned outright by the sigma-delta modulator whenever the PDM sub output is enabled (`derive_core1_mode()`), and 5.1 users plausibly run a sub. Core 1 is therefore not the home for the decoder.

Core 0's main loop already runs the RTA FFT as sliced work, and the S/PDIF poll batches input in about 1 ms granules against a multi-hundred-frame output cushion. The decoder decodes one 256-sample AC-3 block per main-loop pass, about 0.5 ms at 307.2 MHz, which fits inside that cushion. Decode time is metered separately from `cpu0_load`, with a busy-microseconds-per-second counter like the RTA's.

Core 1 gap-filling in EQ-worker and idle modes remains a later optimisation, not part of this plan.

## 6. Scheduling and latency

The whole frame is buffered before decoding starts, so decoding is just-in-time against a fixed anchor.

- The deframer records the carrier sample position of each Pa as the frame's anchor.
- Decoded sample `i` of frame N is played at `anchor_N + period + i`, where `period` is 1536 for AC-3 and 512, 1024 or 2048 for DTS types I to III.
- Block k of frame N is decoded one main-loop pass before it is due. At 640 kbps the payload lands by about 13 ms into a 32 ms period and the whole frame decodes in about 3 ms of CPU spread over the period, so the margin is wide.
- Pause bursts, null bursts, sync loss and CRC failures zero-fill the affected period.

Added input latency is one burst period. AC-3 adds 32 ms. DTS adds 10.7, 21.3 or 42.7 ms by burst type. Both are fixed and deterministic per format.

A flash write stalls XIP code for about 44 ms, which exceeds the decode margin, so a preset save produces a zero-filled gap of one frame on the decoded input. The silent-state-change machinery already fades those operations, so this is accepted.

## 7. Memory

RP2350 has about 76 KB free after boot (`.data` 96 KB, `.bss` 286 KB, slot rings 64 KB, of 512 KB).

The S/PDIF FIFO (12 KB), the I2S rings (32 KB) and the ADAT RX ring (8 KB) already overlay one 32 KB union, the input capture arena. During S/PDIF input only 12 KB of it is in use. The spare 20 KB holds the AC-3 double frame buffer (2 x 3840 B) and a 512-frame, 6-channel, 32-bit decoded ring (12 KB), 19.7 KB in all, as part of the S/PDIF member of the union. That adds nothing to BSS.

The decoder working state must coexist with the live FIFO, so it is genuinely new RAM. AC-3 and DTS never decode at the same time, so their states overlay in one union.

| Build | New RAM | Free afterwards |
|---|---|---|
| AC-3 only | ~17-20 KB state | ~55 KB |
| AC-3 + DTS core | ~45-50 KB state (union) | ~25 KB |

The ADAT output ring (30 KB) is not counted as reclaimable, because ADAT output can run alongside S/PDIF input. Making it part of the overlay is a product decision that would return about 30 KB to the DTS case.

Hot loops (IMDCT, mantissa unpack, QMF synthesis) are RAM-pinned at an estimated 6 to 8 KB of `.data`. The `DATA_BUDGET` in `scripts/check_ram_placement.py` rises accordingly, and the checker runs after every hot-path change.

## 8. CPU estimates

One core at 307.2 MHz, derived from operation counts with the measured M33 FP occupancy of 2 to 2.5 cycles per fused multiply-add. Unmeasured.

| Item | AC-3 5.1 (640 kbps) | DTS core 5.1 (1509 kbps) |
|---|---|---|
| Transform / synthesis | ~14 M cycles/s (6 IMDCT-512 per block) | ~15-29 M cycles/s (32-band QMF, 512-tap prototype) |
| Mantissa / Huffman / bit allocation | ~12 M cycles/s | ~10 M cycles/s |
| Remainder (header, coupling, gain, copy) | ~3 M cycles/s | ~3 M cycles/s |
| Total | ~29 M cycles/s, 7-12 % | ~40-50 M cycles/s, 12-20 % |
| Budget to plan against | 15 % | 25 % |

At the 150 MHz fallback clock every figure doubles.

## 9. Decoder sources and licensing

- **liba52** (GPLv2 or later) for AC-3. Small, C, float build, self-contained IMDCT. Port needs static state instead of `malloc`, const tables instead of init-time generation, and `-ffp-contract=off` evaluated per the existing kernel measurements.
- **libdca** (GPLv2 or later) for DTS core. Larger code and Huffman tables, flash-resident under XIP, hot synthesis loop RAM-pinned.
- The repo is GPLv3, so both are licence-compatible. FFmpeg's decoders are LGPL and also compatible but drag in libavcodec infrastructure, so they are not used.
- The AC-3 essential patents expired in 2017. The original DTS core patents appear to be past term but must be confirmed before DTS ships. Neither "Dolby Digital" nor "DTS" may be used as a product name without a trademark licence, so the feature is called AC-3 and DTS core decoding. This is not legal advice.

## 10. Phases

### Phase 0. Bitstream detection and mute

- Parse the Pa/Pb preamble and the non-audio status bit in `spdif_input_poll()`.
- Report the input format (PCM, AC-3, DTS, unknown bitstream) in the input status and through a notify event. New events need both the push and the `notify_peek_next_for` case.
- Mute the pipeline input while a bitstream is present.
- Standalone value. Ships independently of everything below.

### Phase 1. Decoder spike and bench (gate)

- Port liba52 into a static-state, const-table float module under `firmware/DSPi/`.
- Host harness under `tools/` decodes reference AC-3 vectors (generated with FFmpeg's encoder from known multichannel test signals) and compares against FFmpeg's decoder.
- Device bench build decodes frames from a flash-resident array in a loop and meters cycles per frame with `time_us_32()`.
- Exit criterion: measured cost inside the 15 % budget and state inside the RAM estimate. Otherwise the plan is revised here.

### Phase 2. Integration

- IEC 61937 deframer in the S/PDIF poll, writing complete frames into the double buffer in the arena.
- Just-in-time block scheduling against the anchored delay, into the decoded ring.
- Ring drain into inputs 0 to 5 at carrier pace, zero fill on pause, null and loss.
- Dialnorm applied. Dynamic range compression exposed as a setting.
- PCM/bitstream and 2/6-channel transitions routed through the input resync path.
- Decoder busy meter.
- Verification by `DSPI_LOOPBACK` capture for the fixed delay and inter-channel alignment, plus the host harness for sample accuracy.

### Phase 3. Control surface

- Vendor commands for decoder enable, DRC mode and dialnorm handling.
- Status GET with format, channel mode, sample rate, bitrate and error counters.
- Notify event for format changes, both halves implemented.
- Preset persistence with `SLOT_DATA_VERSION` and `WIRE_FORMAT_VERSION` bumps, and the ESP32 front-panel client's exact-length readback checks re-verified.
- Console support and the protocol spec written at this point.

### Phase 4. DTS core

- Port libdca behind the same deframer for burst types 11 to 13.
- Overlay its state with the AC-3 state.
- Re-run the Phase 1 bench against the 25 % budget.
- Decide type III (2048-sample burst) support on the RAM measured then.

## 11. Out of scope

- AC-3 at 32 kHz.
- Raw bitstream pass-through to an output slot for an external receiver. It needs the non-audio bit on TX and an untouched 16-bit path, and is a separate feature.
- Core 1 gap-filling for the decoder.
- RP2040.
