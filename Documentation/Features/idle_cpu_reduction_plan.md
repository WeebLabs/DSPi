# Idle CPU reduction plan (RP2350, stereo I2S)

*Drafted 2026-11-21. Agreed between two independent reviews (Opus and Fable). Updated the same day with a measured per-stage profile. Steps 1 and 2 committed (12f733e), step 3 committed (aef4548). Step 4 done (uncommitted). Step 5 not started.*

## Goal

Lower the core 0 load for the stereo-only configuration (outputs 1-2 enabled, no PEQ, I2S on all four slots, 48 kHz USB stereo) without raising the USB batch size. One pipeline call per 1 ms, 48-frame packet stays.

Baseline: 18,561 cycles per packet, 6.0%. After step 1: about 17,500, 5.7%. After step 2: about 16,400, 5.3%. After step 3: about 15,700, 5.1%. After step 4: about 13,200, 4.3%. Target: about 11,300 cycles, about 3.7% (step 1 delivered a third of its estimate).

## Where the time goes (measured)

Hardware DWT profile, 2026-11-21, host streaming digital zero. Full data in `bench_results/idle_profile_2026-11-21/results.md`.

| Stage | Cycles | % of 1 ms |
|---|---|---|
| Ring writes, 4 slots (1,274 per slot) | 5,419 | 1.76 |
| Meter + finalize, 8 outputs | 4,563 | 1.49 |
| Matrix (2 copies, 7 disabled-output memsets) | 1,881 | 0.61 |
| Crossfeed + subharm while disabled (state resets) | 1,478 | 0.48 |
| Per-output loop, 9 outputs (resets, gain) | 1,262 | 0.41 |
| Input pass (peaks, RTA taps) | 1,077 | 0.35 |
| pre_give (8 position reads) | 960 | 0.31 |
| Delay pass on the disabled PDM sub | 866 | 0.28 |
| Everything else | 1,055 | 0.34 |

Most of the cost is per-slot and per-output housekeeping. Three of the four slots and seven of the nine outputs carry nothing in this configuration and still pay every packet.

Conditions that matter when reading the meter:
- It only updates while blocks flow. With the stream closed it freezes at a stale value (10% was observed).
- An open Console analyser adds bass-bank work inside the window.
- S/PDIF slots cost about 0.25 points more than I2S (ring writes 6,204 vs 5,419).

## Steps

Re-run the profile (`bench_results/idle_profile_2026-11-21`) after each step and record the measured saving.

### 1. I2S ring writer codegen (done: 835 cycles measured, estimate was 2,500)

`audio_i2s_ring_write_s32`: split the data write and the erase-ahead fill into at most two contiguous runs each, with the ring pointer hoisted into a local, so there is no per-frame index mask and no `inst->ring` reload. The erase becomes a plain word fill. No behaviour change, and output bytes must be identical. The S/PDIF twin is a separate decision.

Result: ring writes 5,419 to 4,584 cycles (1,274 to 1,065 per slot). Bytes are proven identical by a host test over 2 million randomized spans (`bench_results/idle_profile_2026-11-21/equiv`). The audio loop went from 13 to 10 instructions per frame and the pad loop from 9 to 4 (paired stores). GCC will not pair the audio loads, and a `memcpy` pair variant spilled to the stack, so further codegen gains would need assembly. The bigger lever is format, not codegen. If finalize wrote I2S-ready words for I2S slots, the ring write would become a plain copy. The stage is shared with S/PDIF, so that is a design change for a later step.

### 2. Skip clean feature resets (done: 1,097 cycles measured)

Subharm (9 x 160 B per packet) and crossfeed reset their state on every packet while disabled. Give each per-output (subharm) and per-pair (crossfeed) state a dirty flag. The flag is set only inside the process functions and is cleared only by the reset itself, so the reset runs only when dirty. Any other writer of that state, such as subharm's seed-from-partner or a coefficient swap, must also mark it dirty. Psybass, tube and loudness can take the same flag, but they are smaller, so that part is optional.

Result: subharm already had the flag. Its `active` field is set by process entry, the linked-partner mirror and the seed copy, and cleared only by reset, and process entry resets an inactive state. `subharm_park_output_state()` therefore resets only an active state, used for ineligible outputs and the dual-core PDM sub. Crossfeed was left alone, because its state is 16 bytes per pair and a flag would add hidden state for a few cycles. Measured: the crossfeed + subharm stage went from 1,478 to 381 cycles, and dual-core Core 1 work from 7,701 to 7,000. Functional check on hardware: with a 50 Hz host tone, subharm on gave meter ~196 on outputs 1-2, off read 0 on every output (the reset still fires), and on again restarted normally. Psybass, tube and loudness are not done.

### 3. Delay pass on disabled outputs (done: 649 cycles measured, estimate was 850)

The PDM sub always carries a positive alignment delay (`dsp_pipeline.c`, sub alignment), so `any_delay_active` is permanently true and the disabled sub is delayed every packet. When an output goes disabled, zero its whole delay line once (one flag per output), then skip it. "Flushed by recent zeros" is not safe. The shared `delay_write_idx` keeps advancing, so a later read at `widx - dly` could land on slots written before the disable. A `dly` change while disabled is safe under the zero-once rule. An enabled output always runs the delay. The delay pass exists in six pipeline branches (`audio_pipeline.c` single-core, dual-core and RP2040 paths) plus the two Core 1 workers in `pdm_generator.c`. The rule therefore goes in one shared helper, and the enable state is the same per-packet snapshot on both cores.

Result: `delay_run_output()` in `dsp_pipeline.h` replaces all six delay loops (the four in `audio_pipeline.c` and the two Core 1 workers). The enable state is read live, like every other stage. A packet where the enable flips mid-pass costs at most one packet of undelayed audio on the output being disabled. Measured: the limiter + delay stage went from 866 to 217 cycles, single-core total 17,060 to 16,348. The dual config (outputs 1-8) does not run the PDM sub's delay, so it is unchanged within noise. Re-enable test on hardware, both configs: output 1 disabled with tone in its line, then re-enabled at 40 ms with the host streaming zeros. Its meter read 0 across 400+ polls. A line that was not zeroed would have played the old tone. Visible change: a disabled output no longer plays out its delay tail.

### 4. Fewer position reads, and no peak scans on disabled pairs (done: 2,511 cycles measured, estimate was 2,500)

- `output_rings_pre_give` reads each slot's consumed count once and reuses it for both the deficit and the admission check. A stale snapshot only overestimates fill, which is conservative. After a recovery skip, use the updated `wr_frames` with the same consumed value. The ring write keeps its own read, because `snap_produced/snap_consumed` are the clock servo's observable.
- `audio_i2s_ring_consumed_frames` takes one IRQ-off section instead of two.
- `output_pairs_meter_finalize`: a pair with both outputs disabled reports peak 0 instead of scanning `buf_out`. This applies only in the non-s24 branch. In ADAT (`finalize_s24`) mode the peak comes free from the in-place convert and matches what ADAT sends, so that branch is untouched. Visible change: a disabled output no longer shows a delay or limiter tail on its meter. Nothing from it reaches a slot.

Result: pre_give went from 960 to 574 cycles and finalize from 4,558 to 2,498. The ring writes also dropped about 12 cycles per slot from the single critical section. Single-core stereo I2S total went from 16,348 to 13,837, and the S/PDIF stereo total is 15,410. The S/PDIF library got the same single critical section as I2S. On hardware, 15 s of streaming showed no underruns, drops or starvation, and with a tone on outputs 1-2 the disabled outputs read 0. The meter change is not visible in practice, because step 3 already keeps disabled rows at zero.

### 5. Silent I2S slot skip (about 1,800 cycles after step 1)

When a slot's pair is disabled and its ring already holds only the pad pattern, the write advances `wr_frames`, `erased_frames` and the snapshots, but skips the data and erase stores. A zero sample encodes to exactly `I2S_PAD_PATTERN`, so the wire is bit-identical, and the counters advance identically on all slots, so alignment holds.

Conditions:
- The library keeps a trailing-pad-frame counter. Ring reset stamps the full ring and sets it. Any non-silent write clears it. `ring_skip` and `silence_pump` only write pad, so they preserve it.
- "Silent" is a per-pair mask returned by the finalize branch that memsets the stage. It is never re-derived from matrix enables at write time. RP2040, other pipeline variants and Core 1 pairs default to not-silent unless the mask is handed back after `work_done`.
- The stage memset stays, so `slot_stage` remains valid (the loopback tap reads `slot_stage[0]`).
- The consumed read, drop check, `recovery_prewrite_fill_frames` and `snap_*` are unchanged.
- I2S only. S/PDIF silence encodes the 192-frame channel-status phase, and the 512-frame ring is not a multiple of 192.
- Not applied to the live pair (float peak 0 does not imply stage 0, and it only pays in digital silence).

### Verification gate

Steps 1 to 4 must produce identical slot bytes before and after, except for the documented meter change in step 4. Step 3 additionally needs a re-enable test: enable the sub after a long disabled period and check that no stale audio comes out. Step 5 lands only after a wirecap bit-compare of all four slots plus a loopback alignment check across:
- output type switch, forced underrun recovery, preset load, rate change
- enabling and disabling pair 2 mid-stream
- I2S clock-slave mode
- USB stream stop and restart (idle pump and re-anchor while the pad counter is live)
- a flash operation (preset save or rename, via `output_rings_stamp_silence_ahead`)
- cold boot into the stereo-only config
- signal generator routed to an output in a disabled pair
- a dual-core config (outputs 1-8), where Core 1 pairs take the not-silent default and must stay bit-identical

## Follow-up (not part of stereo idle)

Dual-core config (outputs 1-8): Core 0 spends 4,824 cycles per packet waiting for Core 1, because Core 1 carries three pairs (7,701 cycles) against Core 0's one (3,117). The four ring writes (5,419) then run serially on Core 0 after the wait. Rebalancing the pairs, or letting Core 1 write its own pairs' rings after the shared admission decision, is a separate investigation.

## Rejected or deferred

- Folding output gain into the convert: changes rounding, so slot bytes differ, and limiter, delay and RTA all read post-gain `buf_out`.
- Skipping the matrix memsets for disabled outputs (part of the 1,881): deferred. Finalize, delay, RTA taps and ADAT all read disabled `buf_out`, so a "known zero" flag would be hidden state with many readers.
- The 64-bit budget divide in the CPU meter: it runs after `packet_end`, so it is off-meter.
- Increasing the USB batch size: excluded by requirement.
