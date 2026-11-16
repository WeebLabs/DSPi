# Audio loopback: outstanding test failures

*Status: ROOT-CAUSED to the host (macOS coreaudiod input-engine runaway);
firmware and device fully exonerated. Harness self-heal fix recommended in
8c, not yet implemented. Last updated 2026-08-02 evening (RP2350, firmware
v1.1.5 loopback build, wire v27).*

*Command names updated to the paged vendor protocol (firmware 1.1.7) on 2026-11-16. The findings and
their dates are unchanged.*

Four tests in the `audio` group failed under full `run.py` invocations and
passed under every controlled sequence. This document records how that was
root-caused, because the failure signature will recur on any macOS rig.

**The failure mechanism** (sections 8a / 8a-2): a stochastic host-side event
puts coreaudiod's input engine for the capture device into a runaway where it
zero-fills a growing tail of every ~10.6 ms IO cycle, REPLACING real samples
one-for-one. The corruption grows at roughly 1.8 %/s, sometimes re-anchors
and parks, and clears only on a CoreAudio nominal-rate change (engine
reconfigure); PortAudio stream cycling does not clear it. Neither PortAudio
nor any device counter reports anything.

**Neither the firmware's DSP math nor its USB capture delivery is
implicated.** Every clean measurement agrees with reference math to the limit
of the rig, and the counter/timing evidence in 8a-2 rules the device out as
the source of the zeros.

---

## 1. The four tests

| Test | Symptom under a full run | Typical error |
|---|---|---|
| `peq_linkwitz_transform` | magnitude vs the firmware's own LT design | 5.5 to 15.6 dB (varies) |
| `xo_all_band_slots` | all four crossover band slots wrong | 31 to 39 dB |
| `xo_two_band_cascade` | BW2 HP300 + LP4000 cascade | 11 to 31 dB |
| `upmix_centre_off_passthrough` | L/R not bit-exact with centre engine OFF | residual -12 to -18 dBFS, scale 0.29 to 0.67 |

Errors **vary between runs** for the same test. They are not a fixed offset.

---

## 2. Run history

| Run | Condition | Result | Failures |
|---|---|---|---|
| 1 | DSPi Console open (contending for the device) | 66/105 | 21, spread across PEQ, crossover, integrity |
| 2 | Console closed | **87/105, 0 FAIL** | none (18 skipped: 44.1 kHz unreachable) |
| 3 | CoreAudio HAL rate switching added | 101/105 | the 4 |
| 4 | universal capture retry added | 89/105 | the 4, plus all 12 of the 44.1 kHz variants |
| 5 | `--audio-rates 44100` (44.1 kHz only) | 84/88 | the 4 |
| 6 | full run, capture-dump instrumentation (2026-08-02 06:25) | 171/175 | the 4, all at 48 kHz; 44.1 kHz variants passed |
| 7 | identical rerun of 6, ~2 h later | **175/175, 0 FAIL** | none |
| 8 | rerun with in_underflow + block-size logging | 175/175 | none |
| 9 | identical rerun, 9 minutes after 8 | 112/175 | 63: tripped early in the 44.1 kHz block and never recovered (no further rate switch until the end) |

Run 1's 21 failures were entirely **DSPi Console contention** and are not part of
this problem. Closing it produced a clean sweep in run 2. Always confirm no other
application holds the device before interpreting a failure.

Run 4 is the only run in which the 44.1 kHz variants failed. In run 3 they
passed, and in run 5 (44.1 kHz as the only rate) they passed. The distinguishing
feature of run 4 is that the 44.1 kHz block ran **after** a mid-run rate switch
from 48 kHz. This has not been investigated and may be a separate issue.

---

## 3. What is established

### The firmware measures correctly

With the device uncontended, every measurement is essentially exact:

| Measurement | Result | Threshold |
|---|---|---|
| Flat-path residual, 48 kHz | -101.1 dBFS | -80 |
| Flat-path residual, 44.1 kHz | -141.9 dBFS | -80 |
| Path gain (fitted scale) | exactly -1.00000 | 1.0 +-0.5 dB |
| Noise floor (silence played) | -400 dBFS (digital silence) | -100 |
| THD, 1 kHz | 0.0000% | < 0.1% |
| Filter magnitude error, median of 61 configs | 0.001 dB | 0.7 / 1.0 |
| Filter magnitude error, worst of 61 configs | 0.214 dB (see section 7) | 0.7 |
| Output delay at 44.1 kHz | 220 samples exactly | 220 +-2 |
| Clock drift between the two CoreAudio devices | 0 samples over 5.1 s (0.0 ppm) | n/a |

A dedicated 65-test filter-type run passed **twice, 65/65**, covering every PEQ
type in the enum, the Linkwitz Transform, and all 36 crossover types.

### Structural coverage passes consistently

Alignment, output-type switching (8 of 8 with `lag=0 corr=1.00`), routing, phase
invert, gain, mute, delay, clipping, loudness, crossfeed, leveller, psybass and
the loudness output mask all pass in full runs.

---

## 4. Where they fail and where they pass

| Context | Tests run | The four |
|---|---|---|
| Full `run.py`, 48 kHz | 105 | **FAIL** |
| Full `run.py`, 44.1 kHz | 88 | **FAIL** |
| 65-test filter-type run (x2) | 65 | PASS |
| 17-test 44.1 kHz subset run | 17 | PASS |
| 22-test sequential probe of all non-filter tests | 22 | PASS |
| `output_clip_limit` then `peq_linkwitz_transform`, x3 | 2 | PASS (0.001 dB) |
| `xo_all_band_slots` alone in a fresh process | 1 | PASS |
| `peq_linkwitz_transform` driven by hand, x4 | 1 | PASS (0.000 dB) |

The trigger is therefore **not** in the test bodies. It is something about a full
`run.py` invocation that none of the reconstructions reproduce.

---

## 5. Hypotheses ruled out

Each was refuted by direct measurement, not by argument. Do not re-litigate these
without new evidence.

| Hypothesis | How it was refuted |
|---|---|
| Host-side sample-rate conversion between the two CoreAudio devices | Residual energy **falls** with frequency (-29 dB at 0-500 Hz, -78 dB at 12-20 kHz). SRC interpolation error rises with frequency. |
| Clock drift between the playback and capture functions | A tone burst at the start and end of a 6 s capture had **identical** delay (5704 samples both times): 0 samples drift, 0.0 ppm. |
| The stereo upmixer corrupting L/R | `FX.UPMIX_STATUS` (then `UPMIX_GET_STATUS`) read `active=0, parked_reason=1` (disabled) while the errors persisted. |
| The onboard signal generator injecting a tone | `SiggenStatus.state = 0` (IDLE) and all output peak meters read zero with no host audio. Note byte 0 of that struct is `version`, not a run flag. |
| Lost EQ writes (`eq_update_pending` is a single slot, no queue) | Wrote four EQ params back-to-back and read all four back: **0 of 4 lost**, at 0 ms, 5 ms and 30 ms spacing. |
| `output_clip_limit` leaving state that breaks the next test | Ran the pair back-to-back three times: **all pass at 0.001 dB**. Also contradicted by an earlier bisect where the LT test failed running first. |
| Capture servo time-base warp fixed by more pre-roll | Errors were 0.00 dB at 0.10 s, 0.50 s and 1.50 s of pre-roll; the failure was not reproducing at the time, so this is untested rather than refuted. |
| Device state leakage (wrong filter left applied) | Read back band type, frequency and gain after each write: always correct. `xo_all_band_slots` sets band 20 to type 42 and reads back 42 while measuring 31 dB error. |
| Preset-dependent state (user had input PEQs configured) | `loopback_baseline_clean` passes, and one of its assertions is `input EQ bypass == 1`. Input EQ is provably neutralised. |

---

## 6. Observations not yet explained

1. **The failure only appears under `run.py`.** Candidate differences from the
   reconstructions: the `Runner` wrapper's liveness poll (`wait_ready`) after
   every mutating test; total duration (5 to 9 minutes versus 1 to 3); the number
   of stream open/close cycles (roughly 350 in a full run).

2. **Elevated THD in full runs.** `output_clip_limit` reports its *clean*
   reference THD as 1.06% to 2.67% in full runs but **0.000%** in isolation.
   `psybass_harmonics` reports 4.86% to 5.53% THD with psybass **off** in full
   runs versus 0.001% isolated. Something nonlinear is present during a full run.
   A sequential probe of all 22 non-filter tests, measuring flat-path THD after
   each, found **0.0000% throughout**, so the source is not any single one of
   those tests.

3. **The glitch counters never move** during these failures, so the capture is
   not dropping or inserting frames. Whatever corrupts the measurement does so
   without a servo underrun or ring overflow.

4. **Only absolute-reference tests fail.** Tests comparing two measurements
   against each other (`loudness_output_mask` on versus off, `psybass_harmonics`
   THD delta) pass in the same runs. A systematic distortion cancels in a
   difference but wrecks a comparison against an external reference.

---

## 7. Not a bug: the two mild filter outliers

`peq_lowpass_lo` (0.214 dB) and `peq_highpass_hi` (0.096 dB) are *within*
tolerance and reproduce at similar values. Their error is a pure function of how
far the filter has attenuated the signal:

| Expected response | `peq_lowpass_lo` max error |
|---|---|
| 0 to -20 dB | 0.000 dB |
| -20 to -40 dB | 0.002 dB |
| -40 to -60 dB | 0.016 dB |
| -60 to -80 dB | 0.132 dB |
| below -80 dB | 0.214 dB |

Converting each error into the contaminating signal that would produce it gives a
consistent floor of about **-114 dBFS** for `peq_lowpass_lo` and **-135 dBFS**
for `peq_highpass_hi`. The 21 dB gap is explained by where each test's worst
point falls: a log sweep carries equal energy per octave, so per-bin energy is
about 26 dB lower at 17 kHz (where `lowpass_lo` bottoms out) than at 40 Hz (where
`highpass_hi` does).

This is not quantisation (24-bit noise spread over ~24000 FFT bins is about
-193 dBFS per bin). The likely mechanism is spectral leakage: `measure_transfer`
takes an unwindowed `rfft` with only a 5 ms taper, so the roughly 0 dB passband
bleeds into deeply attenuated bins.

**Known inconsistency:** the crossover tests mask comparison to where the
response is above `XO_MAG_FLOOR_DB` (-60 dB); the PEQ tests apply no floor and
compare everywhere. Applying the same floor to the PEQ tests would bring these to
0.016 dB and 0.002 dB. Not done, because it changes what the tests claim to
verify and deserves a deliberate decision.

---

## 8. What the instrumentation found (2026-08-02)

The section-8 plan from the previous revision was executed: every capture in a
full run is now dumped raw (section 8b), and run 6 tripped with the recorder
on. Of the three candidate outcomes, the answer is **corrupted signal**, and
the corruption is fully characterised.

### 8a. The mechanism: periodic zero bursts replace real samples

From the run-6 dumps (259 captures, both rates):

1. **Contaminated captures contain literal zero runs.** Identical on L and R
   (dead-mask agreement 100%). Between the zero runs the signal fits the
   expected (filtered) reference at the exact right sample positions; the
   local cross-correlation lag stays 0 through the whole capture. So the
   zeros REPLACE real samples one-for-one; nothing is inserted or delayed.
   This is why the doc's earlier clock-drift check (identical burst delay at
   both ends of a 6 s capture) passed while the captures were corrupt.

2. **Zero-run lengths are quantised to 16 samples** (within one capture:
   k*16 minus a constant 1..3, e.g. {191, 207, 223} or {13, 29}), and run
   STARTS repeat every 480/528 samples (whole USB frames), pattern period
   exactly 1536 samples = 32 ms.

3. **Onset and growth.** The episode began mid-run, during
   `leveller_boost`'s leveller-on capture: first as 6-sample gaps every
   1536, then growing LINEARLY at ~1.8 %/s of stream time for the next 24 s
   until 42.7 % of every capture was zeros. The gap fraction is what each
   failing test saw:

   | capture | t (s) | zeros |
   |---|---|---|
   | leveller_boost (on) | +17 | 0.1 % |
   | output_clip_limit (clean ref; the "elevated THD") | +18 | 2.7 % |
   | peq_linkwitz_transform | +21 | 4.4 % |
   | xo_all_band_slots (4 captures) | +23..29 | 6.4 -> 15.5 % |
   | xo_two_band_cascade | +31 | 17.4 % |
   | upmix_centre_off_passthrough | +37 | 33.7 % |
   | psybass_harmonics | +40 | 40.5 % |
   | loopback_integrity_44k1 (after rate switch) | +47 | **0.0 %** |

   The 44.1 kHz rate switch (a full pipeline reset on the device AND a
   CoreAudio engine reconfigure on the host) cleared it instantly; the whole
   44.1 kHz block and the final return to 48 kHz were pristine (residual
   -141 dBFS).

4. **This explains every prior observation.** A zero-gap fraction g fits a
   sweep as scale ~ (1-g) with residual = rms*sqrt(g(1-g)); the measured
   scale/residual pairs match that model to 0.1 dB on all of upmix (0.63 /
   -17.4), clip (0.977 / -25.5) and psybass (0.593 / -17.2). The residual
   spectrum follows the sweep's own 1/f energy (the old "SRC ruled out"
   evidence), difference-based tests cancel it, THD reads elevated because
   gap splatter lands in harmonic bins, and read-backs are all correct
   because device state genuinely is correct.

5. **Neither side's error counters move.** Device: loopback overflow AND
   underrun counters frozen through the entire episode (bit-identical
   before/after every contaminated capture); SPDIF DMA starvations, USB
   audio ring overruns: no correlated movement. Host: PortAudio reported no
   input_overflow / output_underflow on any attempt (single attempt each).

### 8a-2. Verdict from run 9 (tripped with full flag instrumentation)

Run 9 tripped inside the 44.1 kHz block and stayed corrupted for ~40
captures, all with the new instrumentation recording. The evidence now
localises the fault to the HOST's input path:

1. **PortAudio never knew.** `in_underflow` (and every other PortAudio
   status flag) was 0 on every contaminated capture, single attempt each.
   The zeros arrive at PortAudio as ordinary data from CoreAudio.

2. **The gap cadence is fixed in WALL TIME, not device samples.** At 48 kHz
   gaps start every 480/528 samples (10/11 ms); at 44.1 kHz every 441/485
   samples (10/11 ms). That period (~10.6 ms) matches the host IO cycle:
   the PortAudio input callback's maximum block was 512 frames at 48 kHz
   and 467 at 44.1 kHz (same ~10.6 ms). One zero run sits at the tail of
   each host IO cycle, its length growing steadily; at 44.1 kHz the lengths
   take every integer value (the 16-sample quantisation seen at 48 kHz was
   incidental, not structural).

3. **The device cannot be the source of a 14-72 % shortfall.** Its loopback
   ring holds 1024 frames; a sustained shortfall of that size would
   overflow it within ~50 ms and increment the overflow counter thousands
   of times per second. Both loopback counters were bit-identical before
   and after every contaminated capture, as were the USB audio ring
   overruns, SPDIF DMA starvations, and spdif over/underruns.

4. **The playback side delivered everything.** The real samples on both
   sides of every gap sit at their exact nominal positions (local
   cross-correlation lag flat at 0): the content "under" each gap is what
   was replaced. A drop on the OUT side would splice content and step the
   lag; that never happens.

5. **Mid-episode partial recovery.** In run 9 the gap fraction grew
   0 -> 72 % over ~20 captures, then snapped back to ~14 % and PARKED there
   for the rest of the block: consistent with the host engine re-anchoring
   its timeline once drift exceeded a threshold, then drifting again.

Conclusion: macOS coreaudiod's input engine for the DSPi capture device
progressively starves and zero-fills the tail of each ~10.6 ms IO cycle.
The runaway state lives in the engine (which macOS keeps running between
PortAudio clients, so stream close/reopen does not clear it) and resets on
a nominal-rate change, which reconfigures the engine.

### 8a-3. System-log correlation (unified log)

Note: `log` is a zsh builtin; use `/usr/bin/log` or every query silently
returns nothing (this cost the first search attempt).

Whenever the loopback tests are running, coreaudiod logs, about every 2 s
(roughly once per capture):

    HALS_OverloadMessage: Overload possibly due to HAL client proc
    exceeding io cycle budget.   (cause: ClientHALIODurationExceededBudget)

Cross-referencing thread IDs shows these overloads are raised on the DSPi
CAPTURE (input) IO context ("IOWorkLoopInit 10009 ...:4", the first context
our client starts). The analytics payload reports HAL_client_IO_duration of
19-25 ms against the IO cycle budget, and the IO cycle is 512 frames at
48 kHz / 467 frames at 44.1 kHz; both match the measured zero-gap cadence
exactly, confirming the gaps are whole engine IO cycles.

The correlation is a chronic condition, not a discrete trigger:

- Overloads fire at ~0.5 Hz through CLEAN runs and tripped runs alike, and
  start/stop exactly with our client's activity (they ceased at run end
  07:35:06 and reappeared for a 30 s verification probe at 07:38).
- At the two precisely-known onset instants (06:25:30 for run 6, 07:30:54.5
  for run 9) there is NO unique event: no kernel/AppleUSBAudio/IOUSB log
  line, no coreaudiod resync or error beyond the routine overloads, and
  coreaudiod's own "num_continuous_silent_io_cycles" analytic stays 0 even
  while it is demonstrably inserting silence.

So the client (Python + PortAudio processing ~2x over the 10.6 ms budget on
every input cycle) keeps the engine permanently on the edge; the trip is
the stochastic moment the engine's lateness handling degrades into the
discard-and-zero-fill regime, and its own telemetry never records the
degradation. This also explains the earlier Wi-Fi finding on this host
(scans adding scheduler latency produced the same overload pathway).

### 8b. Instrumentation now in the harness

- `audio.play_record()` records every attempt's PortAudio status flags,
  including `input_underflow` (PortAudio's "delivered silence because no
  real data was available" flag, previously unmonitored and exactly this
  signature), plus input callback block sizes, into `audio.LAST_RUN`.
- With `DSPI_AUDIO_DUMP=<dir>` set, `_capture()` writes every capture
  (excitation + raw capture + glitch counters + spdif over/underruns +
  clk_sys + DMA starvations + usb ring overruns + live rate + test name) as
  sequential `.npz` files.
- Offline analysers live in the session scratchpad (`analyze_dump.py`,
  `zero_runs.py`, `scale_all.py`, `verdict.py`): global/local lag profiles,
  windowed gain, zero-run signatures, per-capture verdict tables.

### 8c. Reproduction statistics and the recommended fix

The trip is stochastic. Full runs: 5 of 9 tripped (runs 3, 4, 5, 6, 9;
runs 2, 7, 8 clean). Controlled sequences have NEVER tripped: 400
consecutive flat-path stream-cycle captures clean; 20+ iterations of the
phase-4/5 test bodies clean; the switch tests added, still clean. The trip
point moves (run 6: mid-leveller at 48 kHz; run 9: early PEQ block at
44.1 kHz), so no test is the trigger; it is an asynchronous host event.

Since the fault is a host (coreaudiod) input-engine runaway, the suite now
detects and self-heals rather than failing 63 tests (IMPLEMENTED):

1. **Detect**: `audio.capture_zero_gaps()` scans each capture, bounded to
   the excitation's aligned span, for runs of >= 4 exact zeros whose 64
   neighbouring samples on BOTH sides are live. Validated against the three
   dump archives: 0 false positives on ~430 clean captures (including the
   switch-stress tails, where the loopback servo's post-excitation re-prime
   legitimately alternates silence and stale bursts, and all steep-filter
   captures), and it fires from the FIRST onset capture (0.1 % corruption)
   through the whole growth phase. The late "parked" form (signal arrives
   ~0.9 s late, no interior zeros) is not detected here, but never arises
   once the growth phase is healed, and fails the tests' own corr/presence
   checks if a run somehow starts inside it.
2. **Heal**: `_capture()` bounces the CoreAudio nominal rate (other rate
   and back, waiting for the device to follow each move) and retries.
   Max 2 heals per capture; a capture still gapped after that raises
   RuntimeError naming this document.
3. **Report**: heal events are counted in `_HEAL_EVENTS`, printed inline as
   they happen, and summarised in rate_switch_round_trip's notes.

Covered by selftest section 31 (detector semantics, no hardware needed).

Prevention (attacks the chronic overloads from 8a-3 rather than the
symptom): give the input engine a bigger IO cycle so the Python client fits
its budget. Measured on this machine, 15 capture cycles (20 s) per setting,
overload lines counted in the unified log, worst flat-path residual
-141.9 dBFS at every setting:

| blocksize (frames) | cycle budget | overloads / 20 s |
|---|---|---|
| default (512) | 10.6 ms | 14 |
| 1024 | 21.3 ms | 7-8 (also 8 while tripped) |
| **1536 (applied)** | 32 ms | **0** |
| 2048 | 42.7 ms | 0 |
| 4096 | 85 ms | 0 |

1536 is the smallest measured-clean size (the client's IO work is 19-25 ms,
so 1024's budget sits inside it) and is the default via
`DSPI_AUDIO_BLOCKSIZE` (0 restores the PortAudio-chosen 512).

Caveat that shaped the design: the first full run with `blocksize=1536`
hung in PortAudio's stop path (main thread wedged in AudioOutputUnitStop,
an IO thread in PA's startStopCallback -> AudioUnitGetProperty) within ~10
stream stops, against zero hangs in thousands of stops at the default size
the same day; a 200-cycle open/stop stress at 1536 then passed, so the
deadlock is rare and racy, plausibly widened by the longer IO cycle.
Because of it, ALL stream IO now runs in a disposable capture worker
subprocess (`python3 -m tools.dspi_test.audio --worker`, length-prefixed
pickles over stdin/stdout): the parent never opens a stream, a wedged
worker is killed and respawned with the capture retried once, and a worker
that wedges twice in a row raises AudioUnavailable (test SKIPs, run
continues). Covered by selftest section 32.

Two additional live observations (2026-08-02 afternoon):

- The runaway also trips OUTSIDE test runs: the engine was found already
  corrupted (~40 % zeros, effective real rate ~28 kHz) hours after the last
  run, presumably from other clients' use of the device. Any capture taken
  through a tripped engine is garbage regardless of harness settings, which
  is why the zero-run detector + heal remains necessary on top of the
  blocksize fix.
- Opening a stream at a DIFFERENT latency class (latency="low") reset the
  tripped engine the same way a nominal-rate bounce does: any IO-buffer
  reconfiguration appears to rebuild the engine state. Either works as the
  heal action; the rate bounce is the one verified twice in full runs.

The 4-test failure signature of runs 3-6 and the 63-test signature of run 9
are the same fault at different onset times; which tests fail is purely a
function of when the runaway starts and how big the deficit is by the time
each test measures.

---

## 9. Reproducing

```bash
# Fails: the four tests, both rates (about 9 minutes)
python3 -m tools.dspi_test.run --group audio --report r.md --json r.json

# Fails: same four, 44.1 kHz only (about 4 minutes)
python3 -m tools.dspi_test.run --group audio --audio-rates 44100

# Passes: the same tests in isolation
#   see the scratchpad runners used during the investigation, or drive the
#   registry directly:
#     from tools.dspi_test.framework import REGISTRY, Check
#     by = {tc.name: tc for tc in REGISTRY if tc.group == "audio"}
#     by["xo_all_band_slots"].fn(dev, prof, Check())
```

Before interpreting any failure:

1. **Close DSPi Console** and any other audio application. Contention alone
   produced 21 false failures in run 1.
2. Confirm the device is uncontended: output peak meters should read zero with no
   host audio playing.
3. Prefer a snapshot and restore around any manual diagnostic
   (`lifecycle.capture` / `lifecycle.restore_live`), or the device is left in the
   neutralised state rather than the user's preset.

---

## 10. Related

- `tools/dspi_test/test_harness.md` sections 9 and 9a: how the measurement works
  and how rates are handled.
- `Documentation/current_architecture.md`, "USB Audio Loopback Capture": the tap,
  the rate-matching servo, and the glitch counters (`DIAG.LOOPBACK_STATS`).
- `tools/dspi_test/selftest.py`: no-hardware checks of the harness's own logic,
  including that every capture call routes through `_capture()`.
