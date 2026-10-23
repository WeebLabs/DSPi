# Low-latency mode: cutting the per-call pipeline overhead

*Written 2026-10-23 as a handover note. Updated the same day: Fixes 1, 2 and 3 implemented and verified (see "Status and measured results" and "Fix 3 integrated"). All code is uncommitted on local branch `bench/input-batch`.*

## TL;DR

A low-latency mode means feeding the DSP pipeline smaller batches of audio (for example 16 frames instead of 48) and holding less audio in the output rings. Smaller batches cost more CPU because every call to `process_input_block()` pays a fixed setup cost before it touches a single sample. On RP2350 with a realistic 4-way active setup, that fixed cost is about 39 us per call on core 0. At 16 frames that adds about 7.7 percentage points of core 0 load over today's 48-frame batches.

The fixed cost is not mostly DSP. It breaks down into four things, in rough order of size:

1. Core 0 sitting idle while it waits for core 1, because core 1 handles 6 of the 8 outputs.
2. Filter setup. Every filter section costs about 57 cycles per call before its first sample, mostly re-deciding things that never change between calls.
3. Ring bookkeeping. The output-ring code reads each ring's DMA position about 13 times per call.
4. Disabled features clearing their state on every call, even when it is already clear.

Fixing all four should bring the 4-way fixed cost from about 39 us to the low teens, which makes the 16-frame penalty roughly 2 to 3 points instead of 7.7. Unrolling the sample loops for a fixed 16-frame size does **not** help (section "Why unrolling does not help").

## Status and measured results (2026-10-23)

**What is in the code.**
- **Fix 1 is implemented** for crossfeed, psybass, tube and subharmonic. Loudness was left alone: its reset only zeroes a few integrators and costs almost nothing. Subharmonic reuses its existing `active` byte. The other three gained one.
- **Fix 2 is implemented** in the narrow form: one consumed read per slot shared by the two admission loops, the per-call slot-0 refresh deleted, and the unused `extern` in `main.c` removed. The ring writes still read for themselves.
- **The six filter kernels are `always_inline`.** A second caller in `dsp_pipeline.c` made GCC emit out-of-line copies in flash, which the RAM-placement check caught. This changes nothing in normal builds, where only one caller exists.
- **Fix 3 exists only as a bench prototype**: a precomputed kernel sequence of pointers into `filters[][]` and `xover_filters[][]`, built once when enabled (`0x6F` wValue `0x0800 | 1`). It is not wired into the filter-edit paths, so it is not production code.

**Checks.**
- Idle-reset invariant: 501,364 checks during 120 random toggles of crossfeed, subharmonic, psybass, tube, their masks and output mutes, with 0 inactive states that were not clean. Skipping an idle reset is therefore a no-op, so output is unchanged.
- Fix 3 exactness: on all 8 outputs of the 4-way setup, 64 blocks of noise through the current cascade and through the plan gave bit-identical output and bit-identical final filter state. The plan pairs across band boundaries.
- Normal builds of both platforms are clean and pass `scripts/check_ram_placement.py`. The bench build exceeds the RAM-code budget because the prototype carries a second inlined copy of every kernel. That is expected and bench-only.
- Not tested on hardware: underrun recovery after Fix 2 (it needs the stuck-rings fix first).

**Results (RP2350, 4-way active, core 0 load in %, profiled builds).** Profiled builds carry about 1.7 us of checkpoint overhead per call, identically in every variant.

| Variant | 48 fr (1 ms) | 24 fr | 16 fr | 12 fr | Fixed us/call | us/frame | Core 1 fixed |
|---|---|---|---|---|---|---|---|
| Before | 32.52 | 36.36 | 40.27 | 44.09 | 38.6 | 5.97 | 22.5 |
| Fix 1 + 2 | 31.85 | 35.26 | 38.74 | 42.17 | 34.4 | 5.92 | 19.7 |
| Fix 1 + 2 + 3 (prototype) | 30.65 | 33.29 | 35.95 | 38.63 | 26.6 | 5.83 | 13.0 |

The penalty for 16-frame batches over 48-frame batches falls from 7.75 points to 6.89 with Fixes 1 and 2, and to 5.30 with Fix 3.

On factory defaults, core 0 fixed cost fell from 21.8 to 17.2 us per call (load at 16 frames 14.65% to 12.92%). Fix 3 does nothing there because every filter is flat and bypassed.

**Where each fix landed (fixed us per call, before / Fix 1+2 / Fix 1+2+3):**
- Entry ring admission: 4.45 / 2.91 / 2.91. Fix 2 saved 1.5 us, as the review predicted.
- Crossfeed + subharmonic, both off: factory defaults 4.79 / 1.20, 4-way core 0 1.44 / 0.62, 4-way core 1 3.49 / 1.05.
- State resets, limiter, delay (4-way core 0): 0.79 / 0.38.
- Output EQ loop, core 1: 16.95 / 16.59 / 9.84. Core 0 outputs: 5.37 / 5.15 / 3.33. Input EQ: 3.62 / 3.68 / 2.64.
- Wait for core 1: 14.41 / 13.11 / 8.18.

**Fix 3 split, measured before building it.** With zero samples the current cascade costs 54 to 56 cycles per section, and the plan costs about 30. So the plan removes about 45% of the fixed filter cost, and kernel entry (loading coefficients and state, storing state, the inner arm switch) is the other half. At 16 samples the plan is 12 to 13% faster per output. The earlier estimate that core 1's 17 us of filter setup could fall to 4 to 6 us was too optimistic. It fell to 9.8.

**Review of this plan (2026-10-23), and what the measurements said.**
- Subharmonic already carries the flag. Correct, and used.
- Loudness can stay as is. Correct.
- Fix 2 saves 1 to 2 us, not 2 to 2.5. Correct: 1.5 us.
- Fix 3 must be pointers into the existing filter storage plus a precomputed kernel tag, never copied `Filter` structs. Correct, and the prototype is built that way. For production, rebuild a channel's plan inside the existing guarded edit path in `main.c` (wait for core 1 idle, write with interrupts off) rather than inventing a pointer-swap publish. There is no such publish for filters today.
- Pairing may cross band boundaries. Correct, and verified bit-exact on hardware.
- Fix 3 is RP2350-only. Correct: the RP2040 cascade is assembly with no dispatch to remove.
- Measure the dispatch/entry split first. Done, see above.
- The wait for core 1 stops at zero and is core 1's time minus core 0's parallel segment, so not every core 1 saving reaches core 0. Correct. After Fix 3 core 1's fixed cost is 13.0 us against core 0's parallel segment of about 5.4, leaving a fixed wait of 8.2 us. That is larger than the review's 3 to 4 us because core 1 did not fall as far as estimated.
- About 11 us of core 0 fixed cost is untouched by Fixes 1 to 3 (ring writes, matrix, mute envelope, peaks, dispatch, remaining entry). Confirmed: those stages did not move.

### Fix 3 integrated (2026-10-23)

The plan now replaces the cascade in production on RP2350 (`dsp_plan.h`; architecture doc "Filter Plans (RP2350)"). Plans live in `dsp_plan[][]` as pointers into the existing filter storage. The guarded single-band edit in `main.c` rebuilds the edited channel inside its Core-1-idle, interrupts-off window, and `dsp_recalculate_all_filters()` and `xover_init_default_filters()` rebuild every channel at their end. `dsp_cascade_block()`, `dsp_cascade.h` and the RP2350 versions of `dsp_process_channel_block()` and `xover_process_channel_block()` are gone. The bench build keeps the old cascade only as a reference.

Verified on hardware:
- Bit-identical output and final filter state against the old cascade on all 8 outputs, for the 4-way profile and for a randomized configuration of up to 19 sections per output.
- Plan coherence (`0x6F` `0x0800`): 0 stored plans differed from a fresh build across 300 random live edits (151 PEQ, 83 crossover, 42 band bypass, 11 master bypass, 9 bulk applies, 4 preset loads), with audio streaming throughout.
- Idle-reset invariant still 0 violations (503,038 checks).
- Normal builds of both platforms pass `scripts/check_ram_placement.py` (its root list now expects `dsp_plan_run` on RP2350). RP2040 is byte-identical. RP2350 RAM code is 1,088 B smaller and BSS 5.7 KB larger.

| Variant (4-way, core 0 %) | 48 fr | 24 fr | 16 fr | 12 fr | Fixed us/call | Core 1 fixed |
|---|---|---|---|---|---|---|
| Before | 32.52 | 36.36 | 40.27 | 44.09 | 38.6 | 22.5 |
| Fix 1 + 2 | 31.85 | 35.26 | 38.74 | 42.17 | 34.4 | 19.7 |
| Fix 1 + 2 + 3 integrated | 30.59 | 33.18 | 35.77 | 38.39 | 25.9 | 12.6 |

The 16-versus-48 penalty is now 5.18 points, down from 7.75. Factory defaults: 9.70 / 11.37 / 13.05 / 14.73% at 48 / 24 / 16 / 12 frames, fixed 16.8 us (was 21.8). The integrated build is marginally faster than the prototype because the bench switch's branch is gone.

Found along the way, not fixed: `dsp_init_default_filters()` resets each band recipe's type, frequency, Q and gain but not its `bypass` byte, so factory defaults and empty-preset loads keep any per-band bypass set earlier. Pre-existing and unrelated to the plans. Fixed 2026-10-23 (uncommitted).

**What this means for the next step.** Core 0's fixed cost is now 25.9 us with Fix 3 integrated. Of that, 7.9 us is waiting for core 1, so Fix 4 (balancing outputs 4/4 between the cores) is now the largest remaining lever. 
## Background: why batch size costs CPU

Time per call is `fixed + per_frame * k`, where k is frames per call. This was confirmed on hardware: across 18 batch sizes the fit is within 1 us everywhere. CPU load is time divided by the real-time length of the audio, so load = `fixed / k * Fs + per_frame * Fs`. There is no knee. Halving k doubles the fixed share. The only way to make small batches cheap is to make `fixed` small.

Measured fits (RP2350, 307.2 MHz, 48 kHz, release/v1.1.6 + bench build):

| Config | Core | Fixed per call | Per frame |
|---|---|---|---|
| Factory defaults | 0 | 21.5 us | 1.66 us |
| 4-way active | 0 | 38.7 us | 5.95 us |
| 4-way active | 1 | 22.8 us | 4.19 us |

"4-way active" means stereo USB in, all 8 S/PDIF outputs on (so the core 1 EQ worker runs), PDM off, 10 peaking PEQ bands on both inputs and every output, an LR4 4-way crossover, and 1.5 ms delay on outputs 1-2. Its exact parameter blob is `bench_results/batch_bench/config_4way.bin`.

## Where the fixed cost goes (measured)

Per-stage cycle profile at 48, 24, 16 and 12 frames, each stage fitted to `fixed + per_frame * k`. Stage sums matched the measured busy time to within 0.1 us. The checkpoints add about 1.7 us per call, so these figures run slightly high. Raw data: `bench_results/batch_bench/stage_profile.json`.

**4-way active, core 0 (fixed us per call):**

| Stage | Fixed | Notes |
|---|---|---|
| Wait for core 1 | 14.4 | Plus 2.4 us per frame. Core 0 idles about 40% of its pipeline time at 48 frames. |
| Output EQ/gain loop, outputs 1-2 | 5.4 | Filter setup for 2 outputs plus disabled-feature resets. |
| Entry: ring admission | 4.4 | `output_rings_pre_give()` and `update_slot0_fill_fast()`. Ring position reads. |
| Input EQ + meters | 3.6 | 20 filter sections. |
| Ring writes | 2.8 | Plus 0.64 us per frame (see "Separate items"). |
| Matrix | 1.8 | |
| Crossfeed + subharmonic | 1.4 | Both features are OFF. This is pure reset cost. |
| Mute envelope + snapshots | 1.2 | |
| Output peaks + finalize | 1.1 | |
| Limiter setup + core 1 dispatch | 1.0 | |
| Everything else | 1.4 | |

**4-way active, core 1 (fixed us per call), total 22.5:**

| Stage | Fixed |
|---|---|
| Output EQ/gain loop, outputs 3-8 | 17.0 |
| Crossfeed + subharmonic (both OFF) | 3.5 |
| Output peaks | 1.3 |
| Limiter, delay, finalize | 0.7 |

**Factory defaults (single-core path), notable stages:** entry ring admission 4.4, crossfeed + subharmonic (both OFF) 4.8, output peaks + finalize 2.9, ring writes 2.8. About 8 of the 21.5 us is avoidable bookkeeping.

**Key reading of the numbers.** The core 0 to core 1 handshake itself is cheap: dispatch 1.0 us plus about 0.6 us wake latency. The 14.4 us "wait" is core 1's larger fixed workload showing up on core 0's timeline. Anything that shortens core 1's work per call comes straight off core 0's wait.

## The fixes, in order of payoff for effort

### Fix 1: stop clearing disabled features on every call

*Status 2026-10-23: implemented and verified (see above).*

**What happens now.** When crossfeed, subharmonic, psychoacoustic bass, tube or loudness is not processing an output (feature off, output masked off, output muted, RAW test signal, or gain zero for loudness), the pipeline calls that feature's reset function for that output on every call. `subharm_reset_output_state()` memsets 160 bytes per output. With everything off on the single-core path that is 4.8 us per call.

**The change.** Keep one "dirty" flag per output per feature. Set it whenever the feature's process function runs for that output. Call the reset only when the flag is set, then clear the flag. Keep the reset function itself unchanged.

**Every call site to update.** Miss one and a feature can come back on with stale state.
- `crossfeed_process_pairs()` in `crossfeed.c` (per pair, `crossfeed_reset_pair_state`).
- `subharm_process_outputs()` in `subharm.c` (per output).
- The output loops that reset psybass, tube and loudness. There are separate copies for the RP2350 core 0 dual-core path, the RP2350 core 0 single-core path, the RP2040 dual-core and single-core paths (all in `audio_pipeline.c`), and the core 1 workers for RP2350 and RP2040 (`eq_worker_loop` in `pdm_generator.c`).
- The four PDM-output resets that run every call in the RP2350 dual-core path ("PDM is inactive in EQ_WORKER mode").

**Why it is safe.**
- Output is bit-identical as long as only the process and reset functions write the state. Checked 2026-10-23: nothing else writes `subharm_output_state`, `psybass_output_state`, `tube_output_state`, `loudness_output_state` or `crossfeed_pair_state`, apart from subharmonic's `sh_seed_from_partner()`, which runs only on the processing path.
- Something else clearing state while the flag says dirty is harmless (one extra reset). The dangerous direction, state dirty while the flag says clean, can only come from a missed process call site.
- Each output's state belongs to one core at a time. Core-mode switches already happen with core 1 idle, so the flags need no locking. Keep each flag next to the state it guards.
- No change to sample counts, so slot alignment is untouched.

**Expected saving.** About 4.8 us per call on factory defaults. On 4-way, about 1.4 us on core 0 and 3.5 us on core 1, and the core 1 share comes off core 0's wait.

**Verify.** Bit-exact output before and after on the `DSPI_LOOPBACK` build. Also toggle each feature on and off mid-stream and confirm no transient.

### Fix 2: read each ring's position once in the admission check

*Status 2026-10-23: implemented; saves 1.5 us. Recovery not yet re-tested on hardware.*

**What happens now.** `audio_spdif_ring_consumed_frames()` (and the I2S twin) reads the DMA read address and the timer with interrupts disabled, does lap accounting, and disables interrupts again. Per call it runs about 13 times. Four of those are in the underrun loop of `output_rings_pre_give()`, four in its admission loop, one in `update_slot0_fill_fast()`, and four inside the ring writes.

**The change.**
- In `output_rings_pre_give()`, read consumed once per slot in the underrun loop and reuse that value in the admission loop.
- Remove the per-call `update_slot0_fill_fast()`. Its value (`spdif0_consumer_fill`) is only used by the USB feedback code as a fallback when slot 0 has no ring, where it is 0 anyway, and `update_buffer_watermarks()` refreshes it every 8 calls already.

**Do NOT carry the snapshot into the ring writes.** This is the trap. `audio_spdif_ring_write_s32()` computes the fill just before writing and stores it as `recovery_prewrite_fill_frames`. That value drives the sample-exact underrun recovery that keeps PDM aligned with the S/PDIF slots, and the write also latches the coherent produced/consumed pair the clock servo uses. A reading from the start of the call is stale by the audio played during the DSP, often hundreds of microseconds. Recovery would then skip the wrong number of frames and shift PDM against the slots, which violates the slot-alignment hard constraint. The writes must keep their own fresh reads.

**Why the narrow form is safe.** Between the underrun loop and the admission loop the DMA advances at most about a frame. Reusing the earlier value makes fill read up to one frame high, so admission becomes slightly more cautious. That only matters at the ring's capacity limit, far above the normal fill, and the decision applies identically to every slot, so alignment holds. The underrun skip only moves `wr_frames`, so the cached consumed value stays valid across it. Lap detection tolerates fewer reads by design (it uses elapsed time to count whole laps).

**Expected saving.** Entry goes from 9 ring reads to 4. Estimate 2 to 2.5 us per call.

**Verify.** Re-profile stage 0. Force an underrun and confirm PDM still lines up with the slots after recovery. Do that only after the stuck-rings bug below is fixed, or the test cannot recover.

### Fix 3: a precomputed filter plan per channel

*Status 2026-10-23: integrated on RP2350 (see "Fix 3 integrated"). The design notes below are superseded where the review section corrects them (pointer storage, guarded rebuild, cross-band pairing).*

**What happens now.** `dsp_cascade_block()` walks each channel's section list on every call. It skips bypassed sections, decides which neighbours pair into the fused x2 kernels, classifies each SVF section's arm, and dispatches. The filters almost never change, so the answer is the same every call. On top of that, each output enters the cascade up to three times (one per active crossover band via `xover_process_channel_block()`, then the PEQ). All kernels are already inlined into `dsp_cascade_block`, whose prologue pushes 7 integer registers and `d8-d13`, so register saves are per cascade call, not per section.

Measured cost is about 57 cycles per section per call (fixed) against about 15 cycles per section per sample. Core 1's filter setup alone is about 17 us per call in the 4-way setup.

**The change.** When a channel's filters change (EQ edit, crossover edit, rate change, preset load), build a flat plan for that output: crossover sections followed by PEQ sections, bypassed ones removed, pairing and kernel arm already chosen. Per call, run the plan straight through in one cascade. What is left per section is loading 5 or 6 coefficients and 2 states, the sample loop, and storing 2 states.

**Hazards.**
- The plan must be rebuilt everywhere filters change, and the rebuild must not race the pipeline. Follow the existing coefficient-publish pattern (build a new plan, publish it with one pointer swap that both cores see per packet).
- Section order and pairing must stay exactly as today, so output stays bit-identical. Test bit-exactness against the current cascade like the earlier fused-kernel work (memory: "EQ kernel opts HW A/B").

**Expected saving.** The floor is roughly 10 to 15 cycles per section instead of 57. Core 1's 17 us could fall to perhaps 4 to 6 us, and most of that comes off core 0's wait. This is an estimate from cycle counts. The kernel work in August 2026 showed cost proxies mislead on this core, so A/B it on hardware before believing it.

### Fix 4: balance outputs between the cores

**What happens now.** Core 1 runs outputs 3-8 (`CORE1_EQ_FIRST_OUTPUT` 2 to `CORE1_EQ_LAST_OUTPUT` 7). Core 0 runs inputs, matrix and outputs 1-2, then waits. It waits 14.4 us per call plus 2.4 us per frame.

**The change.** Split the S/PDIF output pairs 2 and 2 between the cores, so each core does 4 outputs.

**Hazards.** This touches which slot staging buffers each core finalizes (`core1_eq_work.spdif_out[]`), the PDM versus EQ-worker interlock in the output-enable handler, the limiter's cross-core link groups, and the RP2040 split (2 S/PDIF pairs only). Every output must still advance by the same sample count per packet, which the shared `sample_count` already guarantees.

**Expected saving.** It cuts both fixed and per-frame cost. Rough estimate: 4-way core 0 load at 16 frames drops from 40% to around 30%. It is the biggest single lever and also the biggest change.

## Why unrolling does not help

Unrolling a fixed 16-sample loop removes the loop counter, compare and branch per sample. On the RP2350 M33 the FPU is the bottleneck (VFMA occupies it for about 2 to 2.5 cycles), and the integer loop instructions already run alongside it, so there is almost nothing to recover. Interleaving and latency-hiding transforms were tried in August 2026 and bought about 0%. The per-call cost is coefficient and state movement, dispatch decisions, bookkeeping and core imbalance, and unrolling touches none of those. A compile-time block size would only let the compiler drop odd-length tail handling and count checks, worth a few cycles per section.

## Separate items found along the way

- **Outputs stay silent after an overload (bug, must fix first).** If core 0 falls behind long enough to empty the output rings, `rings_admit` stays false and the rings never restart while the host keeps streaming. Only a few seconds at USB alt 0 clears it. A low-latency mode with a smaller ring target makes this far more likely. A fix task was proposed on 2026-10-22 (`overload-ring-recovery`). Seen in the bench build; the ring path is shared with release builds.
- **Ring write per-frame cost.** Ring writes cost 0.64 us per frame regardless of batch size (about 3% CPU), partly because every audio frame written also stamps a frame of silence one IEC block ahead (`PICO_AUDIO_SPDIF_ERASE_AHEAD` 192). Worth a look on its own.
- **Overload threshold.** The pipeline overloads at about 90 to 97% measured load, not 100%, because main-loop work outside `process_input_block()` is not metered. Plan headroom with that gap.

## Other limits a low-latency mode must change

These are not overhead, but they set floors on latency:
- `OUTPUT_RING_TARGET_FILL_FRAMES` is 384 (8 ms at 48 kHz). It is also the servo target and the underrun re-anchor point.
- A static assert requires the ring target to exceed `AUDIO_BUFFER_SAMPLES` (192), because one input call can deliver up to 192 frames. A low-latency mode must cap deliveries near the batch size.
- `SUB_ALIGN_SAMPLES` is the ring target minus `PDM_BUFFER_SAMPLES` (256). A target below 256 frames makes it negative, so PDM needs a smaller buffer or the mode excludes PDM. Slot alignment must hold either way.
- S/PDIF input batches by time (`freq / 1000`), but I2S and ADAT use a fixed 48 frames (`I2S_INPUT_MIN_BLOCK`, `ADAT_INPUT_MIN_BLOCK`). Make them consistent, preferably by time.

## Tooling (how to reproduce)

- **Bench build.** `cmake -S firmware -B build-rp2350-batchbench -DPICO_PLATFORM=rp2350-arm-s -DPICO_BOARD=pico2 -DDSPI_BATCH_BENCH=ON`. All bench code is under `#ifdef DSPI_BATCH_BENCH` except the USB decode loops, which write at `buf_*[base + i]` with `base` 0 in normal builds.
- **Vendor GET 0x6F** (bench builds only, see `firmware/DSPi/batch_bench.h`). wValue `0x8000 | k` sets the batch size k (0 means one call per USB packet) and clears counters. wValue 0 reads the 24-byte `BatchBenchStats`. wValue `0x4000 | core` reads 16 x u32 per-stage cycle sums for that core (RP2350 only).
- **Stage checkpoints.** `BENCH_PROF(core, i)` in `process_input_block()` and `eq_worker_loop()`. The section names are in `STAGES_C0` and `STAGES_C1` in `tools/batch_bench/stage_profile.py`. Core 0 sections 5 to 9 have different contents on the single-core and dual-core paths.
- **Scripts.** `tools/batch_bench/batch_bench.py` (snapshot, run sweep, restore) and `tools/batch_bench/stage_profile.py <out_dir> 48,24,16,12 [window_s]`. Method notes in `tools/batch_bench/README.md`. Results in `bench_results/batch_bench/`.
- **Python.** Use `/opt/homebrew/bin/python3.14`, which has pyusb, numpy and sounddevice. The `~/venv` Python lacks sounddevice.
- **Flashing.** `REQ_ENTER_BOOTLOADER` (0xF0) is an IN request (GET). Then `picotool load -x <uf2>`.
- **Overload recovery in tests.** After any overload, stop the host stream for about 6 s (alt 0) before measuring again. A quick close and reopen is not enough. `batch_bench.py` detects this from ring fill and delivered frames and does it automatically.
- **Report.** https://claude.ai/artifact/GhyxH2Q7GjZ9p52qvsV1nr (sweep charts and table).

## Suggested order of work

1. Fix the stuck-rings bug (blocks any honest low-latency testing).
2. Fix 1 and Fix 2. Small, safe, bit-exact. Re-profile at 48, 24, 16 and 12.
3. Fix 3 with a hardware A/B and a bit-exactness check.
4. Fix 4 if the remaining fixed cost is still too high.
5. Then the low-latency mode itself: smaller ring target, capped delivery size, PDM buffer, time-based batching for every input.
