# Matrix Mixer and Post-Matrix Kernel FP Optimisation Plan (draft)

*Revision 3, reviewed by Fable over two rounds and approved 2026-11-22. Status: implemented and hardware-measured 2026-11-22, uncommitted.*

## Results (2026-11-22)

RP2350, 48 kHz, 307.2 MHz, siggen pump, cycles from a DWT profiler applied identically to every build. Raw data and scripts are in `bench_results/matrix_fp_2026-11-22/` (`results.md` there has every table). Baseline is HEAD 3b78c5d.

| Region | Before | After | Change |
|---|---|---|---|
| Matrix, dense 8x8 | 33,733 cyc/packet | 17,075 | -49 % |
| Matrix, 3 or 4 crosspoints per output | 14,352 | 7,679 | -47 % |
| Matrix, stereo default | 1,931 | 1,798 | -7 % |
| Loudness, per output-sample, both cores | 61 to 62 cyc | 39 | -35 to -37 % |
| Psybass, per output-sample, both cores | 81 to 84 cyc | 63 to 64 | -22 to -25 % |
| Core 1 work, 8 outputs | 52,260 | 40,640 | -22 % |
| Core 0 packet, stereo default | 11.0 % CPU | 9.6 % | -12 % |
| Core 0 packet, dense 8x8 dual core | 33.5 % CPU | 24.5 % | -27 % |

Each step was measured on its own and kept:

- Step 1 (fused matrix): -43 to -49 % on dense routing, -5 % on stereo default.
- Step 2 (out of line, contraction off): moving out of line alone changed under 3 %; contraction off gave loudness -16 to -19 % and psybass -17 to -20 %.
- Step 2b (half-state SVF): loudness a further -36 % (both shelves fused per sample), psybass a further -6 %.

Deviations from the plan:

- **Profiler.** A scratch patch (`bench_results/.../patch/apply.py`) applied the same timers to each build, instead of extending `eq_prof.h` in the tree. Dense configs used a bench-only override that forces 8 active inputs, because ADAT and I2S input pins collided with the outputs on this board.
- **RAM budget.** It set the final shapes. The tree started with 672 B of `.data` headroom. The matrix kernel unrolls 4x only for n = 1 and 2. Loudness unrolls 2x, since 4x measured about 14 % faster on loudness but would have overrun the budget by 176 B. Final `.data` is 100,072 of 100,352 B.
- **Loudness bypass.** A bypassed shelf now stores identity coefficients (`svm0 = 1`), so the fused loop also covers the one-shelf case without a second code path.
- **Host builds.** `config.h` guards `DSP_TIME_CRITICAL` with `#ifndef` so host harnesses can define it empty.

Verification: `tools/matrix_mix_test` (bit-exact, n = 1..8, lengths 0..192), `tools/fx_kernel_test` (new loudness and psybass error vs double -135.2 / -138.4 dB, old -134.6 / -137.0 dB, block-split bit-exact), `tools/loudness_test` and `tools/eq_kernel_test` pass. On-device 1 kHz output peaks match baseline. The RP2040 build is instruction-identical to HEAD. `check_ram_placement.py` shows only the two pre-existing literal-pool FAILs (`audio_*_ring_consumed_frames`), which HEAD has too.

Not covered: a loopback capture of the final build. The output meters cannot resolve LSB-level changes, so arithmetic correctness rests on the host harnesses.


## Background

A review of PASS 4 (matrix mixing) in `firmware/DSPi/audio_pipeline.c` raised three points.

1. It contains a VFMA that might be faster without.
2. The 32 FPU registers are unused. Input samples, output accumulators or gains could live in registers instead of RAM, and there is a lot of read-modify-write of RAM.
3. The per-crosspoint accumulate loop could become a `switch` on the active crosspoint count, so the gains can be held in registers.

## Findings

- **The current RP2350 matrix loop.** For each output, a first pass writes `dst = s0*g0` (`vldmia; vmul; cmp; vstmia; bne`). Each further crosspoint runs its own pass (`vldr dst; vldmia src; vfma; vstmia; cmp; bne`). Not unrolled. The gain is already in a register per pass, so point 3 is really about fusing passes. Per sample, an output with n crosspoints costs n source loads, n-1 reloads of `dst`, n stores and n iterations of loop overhead.
- **Cost estimate (unmeasured).** About 6 to 8 cycles per sample for the first pass and 9 to 11 for each accumulate pass. At 48 kHz (48-sample packets) and 307.2 MHz: stereo default (2 crosspoints, n=1 each) ~0.25 % of a frame, dense 8x9 ~9 %.
- **Why fusion pays despite "loads/stores overlap FPU work".** That claim (current_architecture.md, "FP contraction") holds where the FPU is saturated, at 9 to 13 FP ops per load/store pair. The matrix is the opposite: one FP op per three memory ops plus a per-sample branch. It is memory and branch bound. Expected fused cost is ~3.5 to 4 cycles per crosspoint-sample plus ~2/n, roughly half for n >= 2 (dense 8x9 from ~9 % to ~4 to 5 %). For n = 1 only the unroll helps, under 0.1 % CPU. **The common stereo case will not move measurably. The value is in multichannel and crossover-style routing.**
- **RP2040 is already fused** (2 inputs, one pass per output, three cases). Out of scope.
- **The matrix VFMA is minor.** n-1 VFMA + 1 VMUL is roughly equal to n VMUL + n-1 VADD on the M33.
- **The larger contraction issue is the loudness and psybass kernels.** `process_input_block()` has 47 fused ops (31 `vfma`, 4 `vfms`, 12 `vfnms`). One is the matrix. The other 46 are two inlined copies each of `loudness_process_output_block()` (7) and `psybass_process_output_block()` (16), which are `static inline` in their headers.
- **The Core 1 worker is in `pdm_generator.c`**, not `audio_pipeline.c`. `eq_worker_loop()` expands both kernels again (23 fused ops) and that file builds at the default `-O2` with contraction on. In worker mode outputs 2 to 7 run there.
- **Loudness and psybass still use the pre-2026-11-19 SVF form** (`ic1 = 2*v1 - ic1`, three-term output mix, coefficients read through `lc->` inside the loop). De-contracted, the loudness shelf is ~17 FP ops per sample against 13 for the `SVF_SHELF` arm in `dsp_svf.h`.

## Scope

RP2350 only. RP2040 code, Q28 paths and `dsp_process_rp2040.S` are untouched, including the RP2040 inline loudness and psybass kernels.

## Step 0: Measurement (small)

- Extend `eq_prof.h` under the existing `DSPI_EQ_PROF` option with three more per-core slots (matrix, loudness, psybass), and extend or version the `DIAG.EQ_PROF` payload. No new knob or DIAG id.
- Control rows to catch whole-function re-optimisation on stages the change did not touch: a 10-band EQ cascade row, and whole-packet cost from `cpu0_load_q8` (Q8 resolution). No nested whole-function timer, because `EQ_PROF_START()` declares fixed local names.
- Deciding configurations, 48 kHz, 307.2 MHz, siggen pump: dense 8x9 (8-channel USB input), stereo into 8 outputs crossover-style (including an n=2 summed sub), loudness on all outputs, psybass on all outputs. Each run in single-core and Core 1 worker mode where it applies.
- No-regression row: stereo default routing.
- Reference build measured first and last, as in the 2026-11-19 EQ A/B.

## Step 1: Fused matrix kernel (independent)

- New out-of-line `DSP_TIME_CRITICAL` function `matrix_mix_output()` in a new TU `matrix_mix.c`, added and flagged `-O3 -ffp-contract=off` in the RP2350-only block of `CMakeLists.txt`, with the kernel body in `matrix_mix.h` so a host harness can compile it. Out of line so that the change does not re-optimise `process_input_block()` and so that the kernel's contraction setting is fixed independently of Step 2.
- The caller keeps today's crosspoint snapshot (n, `src[]`, `gain[]`) and keeps the disabled-output and n = 0 `memset` paths exactly as they are.
- `dst` and sources marked `restrict` (separate arrays).
- Shape, chosen up front for RAM text:
  - N = 1 and 2: 4x unrolled single pass, scalar tail (44.1 kHz packets are 44 or 45 samples).
  - N = 3 and 4: single pass, 2x unroll or none (decide on static size).
  - N > 4: a write pass over the first 4 crosspoints, then accumulate passes of up to 4. One extra `dst` reload and store per four crosspoints.
  - Target growth under ~1.2 KB RAM text. State the actual figure.
- Summation as sequential statements (`acc = s0*g0; acc += s1*g1; ...`) in k = 0..n-1 order, in a contract-off TU. Output is identical to v1.1.6 for n = 1 and changes at the LSB for n >= 2 (the VFMA becomes a separate multiply and add).
- Host harness `tools/matrix_mix_test/`, built with explicit `-ffp-contract=off` (Apple clang defaults to `on`): new kernel against a frozen reference loop written the same way, bit for bit, n = 1..8, lengths 0..192 including 44/45, random gains including negatives. Assert every output is written for exactly `sample_count` samples.
- Rejected: input-major blocking across outputs. Rows are 768 bytes apart, beyond `vstr`'s offset range, so each output needs its own pointer; sparse routing needs a second code shape; and it only saves source loads, not stores.
- Rejected: `memcpy` for n = 1 with unity gain. Saves under 0.1 % and differs for denormals under FZ.

## Step 2: Loudness and psybass kernels, out of line and de-contracted

- Move the RP2350 `loudness_process_output_block()` and `psybass_process_output_block()` out of line as `DSP_TIME_CRITICAL` functions, following the `tube.h` pattern, so one copy serves `audio_pipeline.c` (Core 0) and `pdm_generator.c` (Core 1). Three inlined copies of each collapse to one, so RAM text goes down. Call cost is one call per output per packet.
- Placement: in `loudness.c` and `psybass.c`, with `-O3 -ffp-contract=off` on those files set in the RP2350-only block of `CMakeLists.txt` (beside `rta_bass.c`), so RP2040 objects are unchanged by construction. Coefficient design code also de-contracts and coefficients move by an LSB. That matches `tools/loudness_test/run.sh`, which already compiles the live `loudness.c` with `-ffp-contract=off`, and it cannot affect cycle counts.
- Static check, then hardware A/B against Step 0 rows, both cores.
- Confirm `tools/loudness_test` still passes after the move.
- Precision: filter response within 0.01 dB and noise floor equivalent, not byte-identical loopback.
- `crossfeed.c`, `leveller.c`, `upmix.c` are not affected and stay separate later candidates. `tube.c` is already out of line and de-contracted.

## Step 2b: SVF half-state rewrite for loudness and psybass

- Apply the 2026-11-19 half-state rewrite and hoist coefficients to locals. The arm macros (`SVF_V2`, `SVF_SHELF`, `SVF_LP`, `SVF_HP`) in `dsp_svf.h` can be reused. `SVF_LOAD` / `SVF_STORE` cannot, because they read `g`, `svic1eq` and `svic2eq` off one `Filter`, while loudness and psybass split coefficients and state across two structs. The kernels declare those locals themselves.
- The coefficient structs change (add `g`, drop `sva3`), so the design functions and `tools/loudness_test` are touched. Per-output state size is unchanged. Coefficients are derived at run time and are not in any wire or flash format (confirm with one grep before starting).
- Keep stored state in standard `ic1eq`/`ic2eq` meaning so reset and bypass handling are unchanged.
- Host test against a frozen model of the current kernel: error against a double-precision reference under the same +1.5 dB gate the EQ harness uses, and state continuity across block boundaries.
- Measured separately from Step 2. Do not quote the EQ kernels' 14 to 24 % as the expected gain.

## Order

Step 0, then Step 1, then Step 2, then Step 2b, each built and measured on its own.

## Constraints

- Output slot alignment: every output still uses the same `sample_count` and index. No change to timing, ring writes or slot order.
- New kernels are `DSP_TIME_CRITICAL` and pass `check_ram_placement.py`.
- Build both platforms clean. RP2040 should show no change in the matrix, loudness or psybass code.
- Update the CMake flag comment and `Documentation/current_architecture.md` (FP contraction paragraph, which lists loudness and psybass as still contracted, plus the matrix mixing and loudness/psybass sections), with timestamps.
- No commit without explicit instruction.

## Acceptance

- Host harnesses pass.
- Matrix faster on n >= 2 configurations and not slower on n = 1.
- Each of Steps 2 and 2b kept only if it measures faster on both cores with precision equivalent.
- Control rows within noise.
- RAM text change stated and within the agreed budget.
