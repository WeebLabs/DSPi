# Main-Loop Background Jobs: Plan

*Drafted 2026-10-24 on branch `output-latency` (after dbfb8ca), jointly by two reviewers who checked each other's claims against the code. Phases 1 to 4 implemented 2026-10-24 (see the status sections at the end); RP2040 measurements outstanding.*

## TL;DR

Core 0's main loop both feeds audio into the DSP pipeline and runs everything else (USB requests, the analyser, Control Surfaces, deferred configuration changes). The time between two audio deliveries is how long the output rings can go unfed. At the 0.67 ms latency option that budget is roughly 250 to 330 us, so any single piece of work longer than that risks an audible underrun.

The current branch already fixed the structure that matters most: audio is delivered twice per pass, so the worst gap is the longer half of a pass, not the whole pass. It also split three long tasks (analyser finish, loudness rebuild, Control Surfaces reload) by hand into resumable steps. That approach is sound, not a band-aid. Its weakness is that every split is a fixed granule tuned once on RP2350 at one clock, each module invented its own cursor, and no build notices when a step outgrows its slot.

The plan replaces those hand-tuned splits with one small **step runner**. A long task registers as a job whose step function does work until a deadline and reports whether it is done. The main loop runs one step between audio deliveries. The deadline comes from the live latency option, so the same code self-tunes on RP2040, RP2350 and every system clock. A counter in every build records any step that overran, and the bench tools fail a run when it moves. It is deliberately not a scheduler with priorities, not a "yield to audio" call inside long code, and not an interrupt-driven pipeline.

## 1. Assessment of the current state

**Sound.**
- The three-way input split (`input_service_handshakes()`, `pipeline_service_producer()`, `input_service_servos()`) and the two delivery points in `main.c`. Servos rate-limit by call count, so they rightly run once per pass.
- Flash and reset-bracketed operations already sit outside the budget by design. `prepare_flash_write_operation()` fades, keeps the producer fed, then stamps a full ring of framed silence ahead, so a blackout costs silence, never alignment.
- The analyser phase machine (`rta.c` `step()`) and the loudness double buffer (`loudness_recompute_step()`) both publish atomically.

**Fragile.**
- Granules are constants: `LOUDNESS_STEPS_PER_CALL 8` ("about 90 us on RP2350"), `RTA_FINISH_BINS_PER_STEP 64`, and one radix-4 butterfly stage per `rta_fft_step()` call (about 140 us at order 10). RP2040 runs `compute_vol_step()` in software float, and the selectable system clock (307.2 / 384 / 480 MHz) moves every number. No RP2040 measurements exist.
- The first half-pass is itself a sum: `tud_task()` (a vendor handler such as a bulk collect, 100 to 170 us), the Control Surfaces tick and an analyser step all precede the first delivery. One order-10 stage landing with one bulk handler already exceeds the 0.67 ms budget; the soaks passed because they rarely coincide.
- The second half-pass runs every pending deferred flag back to back. Bracketed ones are fine; a burst of small unbracketed applies is unmeasured.
- Interrupts add a floor of about 53 us to whatever step they land in.
- `loop_prof` sections are positional (`loop_prof.h`), so moving a loop item silently renumbers what `soak.py` reads.
- The Control Surfaces revert is still one block of about 170 to 350 us, roughly half of it cold XIP flash misses.

## 2. Alternatives considered and rejected

- **A "yield to audio" helper called inside long code.** Rejected. `process_input_block()` reads coefficient tables, masks, output types and matrix state that a handler may be half-way through updating, and the Core 1 EQ handshake (`Core1EqWork`, config.h) exists to give both cores one consistent view per block. A yield inside `bulk_params_apply()` or `preset_load()` would feed torn state to the pipeline. The main stack is 2 KB, which also rules out stackful coroutines and makes a nested pipeline frame on top of a deep handler a stack risk.
- **Timer- or interrupt-driven delivery.** The only option that removes the main loop from the latency path, and by far the most expensive. Every `*_pending` flag and payload becomes a cross-context handoff, the Core 1 spin-wait in `audio_pipeline.c` would run at interrupt priority above USB and S/PDIF RX, and the reset brackets depend on main-loop sequencing. A separate project, not part of this plan.
- **Background work on Core 1.** Core 1 is either the PDM generator or the EQ worker and spins on its work handshake; there is no room without reworking that handshake.
- **Pinning the revert code to RAM.** Buys about 100 us once per revert for a permanent RAM cost under the tight RP2040 budget. Slicing is cheaper.
- **A priority or deadline scheduler.** Overkill. There is exactly one deadline (the next delivery) and the loop order is fixed. "A delivery between every unbounded segment" gives the same guarantee with no scheduler.

## 3. Design: the `bg_job` step runner

### API sketch

```c
// bg_job.h
typedef bool (*BgStepFn)(void *ctx, uint32_t deadline_us);  // true = finished
typedef struct {
    uint8_t  id;          // BgJobId, from loop_prof.h (profiler attribution)
    BgStepFn step;
    void    *ctx;
} BgJob;

void     bg_job_start(BgJob *job);         // restarts if already active
void     bg_job_cancel(BgJob *job);
bool     bg_job_active(const BgJob *job);
void     bg_run_one(void);                 // one step of the next active job, round-robin
uint32_t bg_step_budget_us(void);          // from the live output-latency option
extern volatile uint32_t bg_over_budget_count;   // all builds, REQ_GET_STATUS 31
```

- **Steps spend to a deadline.** `bg_run_one()` passes `deadline_us = now + bg_step_budget_us()`, and a step loops until it finishes or `(int32_t)(time_us_32() - deadline_us) >= 0` (wrap-safe). Loudness becomes "build volume steps until the deadline", which makes the granule self-tuning across platforms and clocks.
- **Indivisible quanta.** Work that cannot be split below a unit (one butterfly group) is checked by measurement rather than trusted. `rta_fft_step()` gains a group sub-cursor inside its existing stage cursor, so an order-10 radix-4 stage also spends to the deadline.
- **Over-budget counter.** `bg_run_one()` times each step and increments `bg_over_budget_count` when a step exceeds `budget + BG_IRQ_FLOOR_US` (60 us, a named constant whose derivation lives in this document), so the counter flags genuine overruns rather than interrupt landings. It is a new `REQ_GET_STATUS` index (31); no existing scalar GET changes length, because an external UART client checks exact lengths. `REQ_GET_STATUS` 29 (worst delivery gap) stays the ground truth; the counter only attributes.
- **Budget.** `bg_step_budget_us()` returns 100 us for the two low-latency options and 300 us otherwise, derived as (target minus batch minus the quiet-pass floor of about 75 us minus the interrupt floor of about 53 us), halved. The table and its derivation go in `current_architecture.md` "Output Latency".
- **Job stats.** Jobs that keep their own accounting (the analyser's `frame_busy_us`, `busy_us_per_s`) still get it: `bg_run_one()` hands the measured step time back to the step, or the step times itself. Those statistics must not be lost.
- **Restart and context.** `bg_job_start()` from a vendor handler is safe because all of this runs on Core 0 in one thread. Every job must tolerate a restart mid-build (loudness already does; the revert must release before it reloads, which its step order gives).
- **Brackets.** `bg_run_one()` keeps running during reset brackets (`preset_loading`, output type switch). Bracketed operations already pay in silence, and starving jobs there only delays the analyser and loudness swap for no gain.

### Loop shape

Move the analyser step out of the first half-pass into `bg_run_one()`:

```
tud_task, notify, transports, CS tick, siggen service, idle pump   (half-pass A)
deliver
bg_run_one()                                                       (one bounded step)
deferred block                                                     (half-pass B)
deliver
```

Try this with two deliveries first. The deferred block is empty on most passes, so a third delivery between `bg_run_one()` and the deferred block may be unnecessary; the quiet-pass measurement decides. An extra delivery is a few loads when nothing is ready, but measure it before keeping it.

### Profiler names

`loop_prof.h` keeps the main-loop sections as an enum with a name table, plus the `BgJobId` enum and its name table, each guarded by a `_Static_assert` that table length equals enum count. `soak.py` parses the header, so the firmware and the bench can never drift. Under `DSPI_LOOP_PROF`, `bg_run_one()` records a per-job worst keyed by id, and `tud_task` vendor dispatch records a per-opcode worst.

## 4. Control Surfaces revert as a job

**Steps.** Per-slot teardown, then the button and IR reset, then `cs_load_stored_bindings(true)` and `(false)` per slot, then IR and names, then `cs_bindings_changed()` once. The tick is safe between steps because torn-down slots are inactive.

**The gate lives in the CS module.** `control_surfaces_busy()` is true while the revert job runs. The entry surface is too wide to audit from `vendor_commands.c` (deferred applies in `main.c`, direct SETs, the GPIO noun path in `control_surfaces_nouns.c`, the UART and I2C transports), so the check goes at the top of every `control_surfaces_apply_*()`, `control_surfaces_fire_macro()`, IR learn and aux state call, returning `CS_STATUS_BUSY`. `main.c` skips its whole CS deferred block while busy, leaving the flags set; today those handlers clear the flag before calling, so this is a small reordering.

**Reads answer from the stored image.** No CS GET newly fails, because the ESP32 front panel does strict exact-length readback and a failed GET would break its sync. The revert converges on the stored image, so while busy, `REQ_GET_CS_BINDING` and the IR, group, macro, name and display-page GETs answer from the directory copy. That needs per-slot stored accessors in `flash_storage.c` (today only the whole 388-byte `preset_get_cs_config()` and the IR equivalent exist), which also reduces stack use in handlers. Status GETs stay live; they already carry PENDING.

**Rejected alternative.** Shadow-and-swap is not possible because the revert claims GPIO, PWM, I2C and IR hardware. A half-shadow (validate into shadow, claim live) buys nothing because `cs_validate()` checks pin conflicts against live claims.

## 5. Vendor handlers

Rule: a handler that exceeds the budget moves its work into the deferred block or a job. It never yields. Measure before moving anything. Known candidates:
- `REQ_GET_ALL_PARAMS` runs `bulk_params_collect()` in the handler on the first chunk and on the non-chunked path. If it exceeds budget, the collect becomes a job that fills `bulk_param_buf`, and the first chunk answers busy-retry.
- CS status GETs that copy `CsFlashConfig` from the directory (cold XIP plus a large copy).
- `REQ_RTA_GET_BINS` (a frame copy, probably fine) and handlers that read directory structures such as `preset_get_name()` (cold misses, probably short).

## 6. Phases

1. **Measurement.** Add `bg_over_budget_count` (GET_STATUS 31), the named profiler sections and job ids, and the per-opcode worst. Make `soak.py` assert on the counter and on GET_STATUS 29. Add an RP2040 profiling build and record RP2040 numbers for every option.
2. **Runner, then loudness and analyser.** Write `bg_job.c/.h` (about 80 lines). Migrate loudness first (replaces the `loudness_recompute_*` driving code in `main.c`; delete `LOUDNESS_STEPS_PER_CALL`; keep the blocking `loudness_recompute_table()` for boot). Then make the analyser `step()` a job; `rta_service()` keeps config apply and auto-off. Add the butterfly group sub-cursor. `RTA_FINISH_BINS_PER_STEP` survives only as a maximum. Build both platforms, run `latency_bench` and the soaks.
3. **Control Surfaces revert and stored load as a job,** with the module-owned gate and stored-image reads from section 4.
4. **Deferred block and handlers.** Using the profiler, count unbracketed applies per pass and per-opcode handler time; move only what measurement shows over budget. Add a "one small apply per pass" rule only if bursts show up.

## 7. Acceptance criteria

On both platforms:
- The three soaks (S/PDIF 0.67 ms, S/PDIF 1 ms, USB 2 ms) leave `bg_over_budget_count` unchanged, keep GET_STATUS 29 under (target minus batch) with margin, and keep the lowest pre-write fill at 0.67 ms above zero.
- The `4way` profile at 0.67 ms, where the fixed pass is longest.
- I2S input at 96 kHz at 0.67 ms, the tightest case the bench can drive.
- The `latency_bench` alignment check (playing slots within one frame for at least 98 percent of readings) still passes, because the change moves when blocks are given.
- Quiet-pass cost before and after, from the profiler, under a stated ceiling.
- A torn-read check: poll `GET_CS_BINDING` continuously across a revert and assert every reply equals the stored image.
- RP2040 on every option it offers. If RP2040 cannot meet the 0.67 ms budget with a quiet pass, mark that option unavailable on RP2040 through the option table's flags rather than chasing it.

## 8. Do not

- Slice bracketed operations (preset load, flash writes, rate change, output type switch); they must stay atomic and already run under a fade plus a silence stamp.
- Add a yield into any handler or pipeline path.
- Move the pipeline into an interrupt on this branch.
- Touch `usb_device.c`.
- Feed any ring independently of the shared write index. The runner only decides when `pipeline_service_producer()` runs, never how.

## Phase 1 status (2026-10-24)

Implemented, uncommitted:
- `bg_job.c/.h`: `bg_step_budget_us()`, `bg_step_account()`, `bg_over_budget_count` (`REQ_GET_STATUS` 31). The Phase 2 runner will call `bg_step_account()` itself; until then the analyser step, the loudness step and the Control Surfaces revert account themselves.
- `loop_prof.h`: sections and job ids as X-macro lists, so the enum and the names come from one list and cannot disagree. This replaces the `_Static_assert` on table length proposed in section 3. `tools/latency_bench/loop_prof.py` parses the lists and the status bases.
- Per-opcode vendor handler worst under `DSPI_LOOP_PROF` (`vendor_handle_get()` and `vendor_handle_set_data()` are the single funnel for every transport).
- `soak.py`: a verdict with exit status 1 on failure (no underruns outside host stalls, no refused blocks, no step over budget, every delivery gap under target minus batch, pre-write fill never zero), plus whole-run profiler worsts.
- RP2040 profiling build: `build-rp2040-loopprof`, built but not yet run. The RP2040 measurements are deferred (no RP2040 board on the bench 2026-10-24); take them before Phase 2 settles `BG_PASS_FLOOR_US` or the RP2040 option availability.

Differences from the draft:
- The budget follows the formula in section 3 rather than the round 100 / 300 us: 99 us at 0.67 ms, 266 us at 1 ms, 300 us (the cap) from 2 ms. The draft's 100 us for the 1 ms option did not match its own derivation.
- The boot-time loudness build (`loudness_recompute_table()`) is not accounted, because it is not a main-loop step.

### RP2350 measurements (2026-10-24, 307.2 MHz, `build-rp2350-loopprof`)

15-minute soaks, same action seed as the earlier runs, `bench_results/bg_phase1/rp2350`. No underruns and no refused blocks in any run; all three fail the new checks, which is the point of Phase 1.

| Soak | Over-budget steps | Worst gap (limit) | Lowest pre-write fill | Failed checks |
|---|---|---|---|---|
| S/PDIF, 0.67 ms (budget 99 us) | 1036, in 621 of 900 s | 406 us (333) | 0 frames, twice | budget, gap, fill |
| S/PDIF, 1 ms (budget 266 us) | 3 | 472 us (666) | 13 frames | budget |
| USB, 2 ms (budget 300 us) | 1 | 411 us (1000) | 37 frames | budget |

Worst step per job: analyser 208 us, loudness rebuild 155 to 173 us, Control Surfaces revert 370 us. What this means for the later phases:
- **Analyser** (Phase 2): nearly all of the 1036 overruns at 0.67 ms. One order-10 radix-4 stage plus an interrupt lands above 159 us (budget plus floor) every few frames, so the group sub-cursor is required, not optional.
- **Loudness** (Phase 2): eight volume steps take up to 173 us, over the 0.67 ms limit. Spending to a deadline fixes it.
- **Revert** (Phase 3): the only overrun at 1 ms and 2 ms, and the cause of every 0.67 ms second whose gap passed the limit or whose fill touched zero.
- **Quiet pass**: the baseline action profile measures a worst gap of about 121 us with nothing happening (about 110 us of it is processing one 16-frame block), not the 75 us `BG_PASS_FLOOR_US` assumes. With the measured value the formula gives (333 - 121 - 60) / 2 = 76 us at 0.67 ms rather than 99 us. Settle the constant once the RP2040 numbers are in.
- **Vendor handlers** (Phase 4 candidates): `REQ_GET_ALL_PARAMS` 181 us, `REQ_SIGGEN_CONTROL` 107 to 145 us, `REQ_RTA_GET_BANDS` 115 us (polled every 50 ms by Console, so it is the most frequent large cost). Every other opcode stays under 85 us.
- **Coefficient updates**: a loudness reference or intensity change spends up to 194 us in the coefficient-update section, separate from the table rebuild. Measure it as its own item in Phase 4.

## Phase 2 status (2026-10-24)

Implemented, uncommitted:
- `bg_job.c/.h`: `BgJob`, `bg_job_register()`, inline `bg_job_start()` / `bg_job_cancel()` (the audio path starts the analyser job without a call), `bg_run_one()` round-robin after the first delivery (profiler section `bg_jobs`), and `bg_out_of_time(deadline, estimate)`.
- Loudness rebuild as a job: builds volume steps until the deadline, `loudness_recompute_take_done()` replaces the step's return value for the re-key, `LOUDNESS_STEPS_PER_CALL` deleted, boot build unchanged.
- Analyser transform as a job, started by `rta_packet_end()`, cancelled by `stop_engine()`; `rta_service()` keeps config, liveness and auto-off. `RtaFftCursor` makes every FFT stage resumable (prescale samples, butterflies, permutation cycle starts with the placed-slot bitmap in the cursor, split bin pairs); `test_rta.py` proves 7-unit chunks bit-identical to whole stages.

Found and fixed during Phase 2:
- **The permutation was one 140 to 150 us unit.** With steps spending to a 300 us deadline, every frame ended one step at about 440 us (15,000 to 30,000 overruns per soak). Resumable between cycles, a 16-start chunk costs about 32 us.
- **Checking the deadline after a unit lets a step end one unit late**, which with an interrupt crossed the margin. Steps now check before each unit with an estimate: the analyser keeps the last chunk cost per FFT stage and for the finish (costs differ 8 to 34 us by stage); the loudness rebuild uses its last step.
- **One interrupt margin was too tight for the counter.** Under the soak's interrupt load a step absorbs two landings about once in 10 s (up to 79 us over budget, gaps still under 222 us). The counter margin is now `BG_OVERRUN_MARGIN_US` (110 us); the budget still sets aside one landing (`BG_IRQ_FLOOR_US`, 60 us).
- **`BG_PASS_FLOOR_US` is the measured 120 us**, not the drafted 75 us, so budgets are 76 / 243 / 300 us at 0.67 ms / 1 ms / 2 ms and above.

### RP2350 results (`bench_results/bg_phase2`)

`latency_bench` on the first Phase 2 build: 42 of 42 points pass on the stock configuration and 42 of 42 under the 4-way profile, rate-follow and rapid-switch checks pass (the later fixes change only when analyser work runs, which the bench does not exercise). 15-minute soaks on the final build:

| Soak | Underruns | Over-budget steps | Worst gap max / p99 (limit) | Lowest fill (min / median) |
|---|---|---|---|---|
| S/PDIF, 0.67 ms | 0 | 3 (Phase 1: 1036) | 480 / 219 us (333) | 0 / 10 frames |
| S/PDIF, 1 ms | 0 | 3 (Phase 1: 3) | 416 / 381 us (666) | 16 / 18 frames |
| USB, 2 ms | 0 | 1 (Phase 1: 1) | 531 / 495 us (1000) | 31 / 41 frames |

Every remaining overrun, every gap over the limit and the one zero-fill second are Control Surfaces reverts (seconds 280, 439 and 529 of the seed), which is Phase 3. The analyser holds 44 to 45 frames per second at every option, unchanged. At 1 ms and 2 ms the p99 gap rose (Phase 1: 240 / 278 us) because the budget lets a step run longer; it stays well inside the limit, which is what the budget is derived from.

## Phase 3 status (2026-10-24)

Implemented, uncommitted:
- The Control Surfaces reload is one sequence of units (`cs_reload_unit()`, phases `CS_RL_*`), run straight through at boot and as a background job by `control_surfaces_revert_begin()`. The old one-call `control_surfaces_revert()` and `cs_load_stored()` are gone. The stored bindings and IR table are read in place (`preset_stored_cs_config()`, `preset_stored_cs_ir_config()`, replacing the copy-out getters).
- `control_surfaces_busy()`: every public apply, aux write, macro fire and IR learn returns `CS_STATUS_BUSY` (or false) while the job runs, and `main.c` holds its whole CS deferred block, so queued requests run afterwards in order.
- Reads during the job: bindings and IR commands from the stored image, aux state and level projected; groups, macros, display tables and names change within one unit and stay live. This narrows section 4's "groups, macros, names and display pages from the directory copy": making those reset and reload atomic avoided both the extra accessors and the display seeding mismatch (pages seed on attach when the stored table is empty).
- `tools/dspi_test` group `control_surfaces`, and a bench pacing knob (`REQ_GET_STATUS` 512+n on `DSPI_LOOP_PROF` builds) so the torn-read test can land reads mid-revert.

Found and fixed during Phase 3:
- **Unmeasured units.** The first revert after boot runs cold code; estimates start at `BG_UNIT_UNMEASURED`, so a kind of unit not yet seen always starts its own step.
- **Estimates per binding type.** One estimate per phase let a costly slot (a display attach is about 120 us cold) follow a cheap empty slot (about 1 us) past the deadline; estimates are now per phase and binding type.

### RP2350 results (`bench_results/bg_phase3`)

`control_surfaces` QA group: 3 of 3 pass; 196 binding reads landed mid-revert across 20 paced reverts, all equal to the stored image. Full suite on the release build: the 10 known failures plus `rta_bin_frame_seq_consistency`, which fails on the current host path, not the firmware (USB control transfers are 3 ms each behind the bench KVM's hub since 2026-10-24 03:16, so a 9-transfer bin read cannot finish inside one 22 ms frame; the same read took 5 to 9 ms before). 15-minute soaks, all three passing every check:

| Soak | Underruns | Over-budget steps | Worst gap max / p99 (limit) | Lowest fill (min / median) |
|---|---|---|---|---|
| S/PDIF, 0.67 ms | 0 | 0 (Phase 1: 1036) | 254 / 221 us (333) | 5 / 11 frames |
| S/PDIF, 1 ms | 0 | 0 (Phase 1: 3) | 393 / 388 us (666) | 14 / 19 frames |
| USB, 2 ms | 0 | 0 (Phase 1: 1) | 515 / 500 us (1000) | 34 / 39 frames |

Worst step per job: analyser 155 / 304 / 319 us, loudness rebuild 120 / 247 / 299 us, revert 152 / 232 / 230 us, each inside budget plus margin (186 / 353 / 410 us). The 0.67 ms option now passes the same real-use soak as the others. Still to do: Phase 4 (vendor handlers and the deferred block) and the RP2040 measurements.

## Phase 4 status (2026-10-24)

**What the measurements showed.** The Phase 3 soaks' whole-run profiler, read on the USB soak where the S/PDIF receiver's ~55 us interrupt does not inflate every figure, ranks the work that is not yet a job:

| Item | Worst | How often |
|---|---|---|
| `REQ_GET_ALL_PARAMS` (bulk collect, 1.9 KB of field copies) | 185 us (107 cold, about 80 warm on a quiet board) | on demand |
| `REQ_RTA_GET_BANDS` | 116 us, only when a read auto-starts the analyser (`arm()` designs the bass bank); a normal read copies 82 bytes | rare |
| `REQ_SIGGEN_CONTROL` | 107 us | rare |
| Deferred block, `flash_sets` | about 170 us, a Control Surfaces binding add (cold) | per bind |
| Control Surfaces tick | 127 to 154 us with pots bound and moving | while a control moves |
| `coeff_updates` | 93 to 112 us | per filter edit |

The action profile at 0.67 ms puts every action but one within about 40 us of the quiet pass; only the bulk read stands out (+86 us). No deferred-block burst appeared: its largest item is a single binding add.

**What was changed, and why not more.** Each item above is a single bounded piece of work that fits under the 333 us gap limit on its own. The hazard was that two of them could share one segment between deliveries: host commands and the Control Surfaces tick sat in the same segment (a bulk read on top of a turning knob could reach about 370 us), and the background step shared its segment with the deferred block. So the fix is structural rather than per handler: `pipeline_service_producer(true)` now runs after host commands and after the background step as well (four delivery points per pass), and the worst gap is the longest single segment. Making the bulk collect a job was considered and not done: the UART and I2C transports would cope (both retry `CTRL_DISPATCH_BUSY`), but USB would need the control data stage answered later from outside the request callback, a change to the most fragile path in the firmware for a rare read that already fits on its own. If a future option tightens the limit below about 250 us, that is the item to revisit, and RAM placement alone would not be enough (about 80 us warm).

### RP2350 results (`bench_results/bg_phase4`)

Quiet pass (action profile baseline) 127 us, against 130 us with two delivery points, so the extra deliveries cost nothing measurable. 15-minute soaks, all three passing every check with no overruns:

| Soak | Worst gap, Phase 3 / Phase 4 | p99, Phase 3 / Phase 4 | Limit | Lowest fill (median) |
|---|---|---|---|---|
| S/PDIF, 0.67 ms | 254 / 266 us | 221 / 209 us | 333 us | 5 frames (10) |
| S/PDIF, 1 ms | 393 / 376 us | 388 / 373 us | 666 us | 15 frames (20) |
| USB, 2 ms | 515 / 325 us | 500 / 323 us | 1000 us | 34 frames (39) |

The largest gain is on USB, where a 300 us background step no longer shares a segment with block processing. At 0.67 ms the median and p99 fell; the maximum moved by 12 us, within run-to-run spread.

`latency_bench` (stock configuration, every source and option, with the extra checks): 42 of 42 points pass, the slot-alignment proxy reads at least 99.6 % of samples within one frame on every point, and the rate-follow, source-fallback and rapid-switch checks pass. `control_surfaces` QA group: 3 of 3.
