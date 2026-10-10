# Bench Checklist: SPDIF RX Overhaul Merge (Phase 4)

*Written 2026-10-11 for images built from `006b3de` on `merge/spdif-rx-overhaul`.*

This is the hardware verification the merge plan (`spdif_rx_overhaul_merge_plan.md`, Phase 4) calls for. Nothing in the merge has run on hardware yet. The RP2040 output-ring path and RP2040 PLL dithering have never run on hardware at all, so treat every RP2040 step as a first run.

Work through the sections in order. Earlier sections catch the faults that would make later ones meaningless. Record each result next to its checkbox, with numbers where the step asks for them.

## 0. Images and tools

All images are in `custom-builds/`.

| Image | Use |
|---|---|
| `merge-baseline-3272007/DSPi-<chip>-baseline-3272007-loopback.uf2` | Section 1 only: the pre-merge reference run |
| `merge-bench-006b3de/DSPi-<chip>-merge-006b3de.uf2` | Normal firmware: sections 3, 4, 5, 6, 9, 10, 12, 13 |
| `merge-bench-006b3de/DSPi-<chip>-merge-006b3de-loopback.uf2` | Loopback test suite: section 2, and the soak in section 11 |
| `merge-bench-006b3de/DSPi-<chip>-merge-006b3de-clkdiag.uf2` | Clock diagnostics: sections 7 and 8 |

Every merge image reports `v1.1.6-beta4-35-g006b3de` as its build stamp. Before each section, confirm which image is running:

```bash
python3 tools/dspi_poke/dspi_poke.py get REQ_GET_BUILD_INFO 64
```

Equipment the sections assume:
- A logic analyser with at least 4 channels and SPDIF and I2S decoders (sections 4 and 12).
- An external SPDIF receiver with a visible lock indicator, and a PCM5102 board on I2S with SCK grounded (section 5).
- A DAC that runs from MCK, for example a PCM5122 or an ES9038 board (section 9).
- An ADAT receiver (RP2350 only, section 10).
- An SPDIF source, an ADAT source, and an external I2S master such as an Amanero (sections 7 and 8).

## 1. Pre-merge baseline (Phase 0, still owed)

Flash the baseline loopback image for each chip and record the reference run. Later sections compare against it.

```bash
python3 -m tools.dspi_test.run --all --report baseline_<chip>.md --json baseline_<chip>.json
```

- [ ] RP2350 baseline report saved. Totals: ___ PASS / ___ FAIL / ___ SKIP.
- [ ] RP2040 baseline report saved. Totals: ___ PASS / ___ FAIL / ___ SKIP.

## 2. Test suite on the merge

Flash the merge loopback image and run the same command, saving to `merge_<chip>.*`.

- [ ] RP2350: no test that passed on the baseline fails on the merge.
- [ ] RP2040: no test that passed on the baseline fails on the merge.

The suite counts `GET_STATUS` values 17 to 21 differently now. They report frames of silence instead of silence buffers, so any test asserting on their absolute values needs reading in that light.

## 3. Boot and basic playback (normal image)

- [ ] RP2350 boots, enumerates over USB, and plays USB audio on all four slots, PDM and ADAT.
- [ ] RP2040 boots, enumerates, and plays on both slots and PDM. This is the first time the RP2040 ring path has run.
- [ ] RP2040 with one slot switched to I2S plays correctly on both types.
- [ ] Neither chip reboots or faults over five minutes of playback. A watchdog reset shows up as re-enumeration.

## 4. Alignment matrix (normal image)

This is the inviolable constraint, so it gets the most care. Set the signal generator to single-sample impulses (type 7, `SIGGEN_IMPULSE`, period 100 ms) routed to every output. Capture all output pins at once on the logic analyser. For each pair of outputs, measure the frame offset at which the same impulse appears.

Record the offsets once at the start. Repeat after each operation below and compare.

- SPDIF and I2S slots must show **identical** offsets before and after every operation.
- ADAT should sit a constant 96 frames behind the slots by design.
- PDM's offset is a known open defect (about 200 samples early, and drifting). Record it but do not fail on it.

| Operation | Slots unchanged | ADAT constant | PDM offset (record only) |
|---|---|---|---|
| Starting point | | | |
| Reboot | | | |
| Preset load | | | |
| Preset save | | | |
| Active preset delete | | | |
| Factory reset | | | |
| Bulk SET (Console) | | | |
| Preset rename (metadata flash write) | | | |
| Control Surface save | | | |
| Input source switch (USB to SPDIF and back) | | | |
| Rate change 48 to 44.1 to 96 kHz | | | |
| Output type switch (SPDIF to I2S and back) | | | |
| Output pin change | | | |
| MCK enable, multiplier change | | | |
| USB alt change (RP2350: 2ch to 8ch to 2ch) | | | |
| I2S slave unlock and relock | | | |
| ADAT slave unlock and relock (RP2350) | | | |

- [ ] RP2350 matrix complete with no slot or ADAT failures.
- [ ] RP2040 matrix complete with no slot failures (no ADAT).

## 5. Flash writes keep receivers locked (normal image)

The stall alarm should keep every output playing correctly framed silence through flash writes. Connect the external SPDIF receiver to one slot and the PCM5102 to an I2S slot. Then perform 20 preset renames, 10 Control Surface saves and 5 preset saves, at both 48 and 96 kHz.

- [ ] The SPDIF receiver's lock indicator never drops.
- [ ] The PCM5102 never wedges into full-scale noise. That was the field bug the silent-state work targeted.
- [ ] Audio resumes after every write without a click.

## 6. Other stalls (normal image)

- [ ] An output type switch while the DAC hardware mute has a hold time configured keeps the SPDIF receiver locked.
- [ ] A PDM pin move (Console) keeps the SPDIF receiver locked.

## 7. Clock tracking (clkdiag image)

For each source below, play for at least 130 seconds and log with:

```bash
python3 tools/clkdiag_poll.py 130
```

Pass means:
- **Starvation.** The `starv` counters stay at zero after lock.
- **Fill.** The fill settles near 50% (8 buckets).
- **Trim.** The commanded ppm settles to the source's offset, and the 1-second swing stays within about 1 ppm.

| Source | RP2350 | RP2040 |
|---|---|---|
| SPDIF in, 44.1 kHz | | |
| SPDIF in, 48 kHz | | |
| SPDIF in, 96 kHz | | |
| ADAT slave, 48 kHz | | n/a |
| I2S slave (Amanero), 48 kHz | | |
| I2S slave (Amanero), 44.1 kHz | | |

RP2040 PLL dithering has never run on hardware. If the RP2040 `vcxo` line shows commands but `sysclk` ppm does not follow, stop and report it, because the soft VCXO is then inert on RP2040.

- [ ] Every cell passes, or the failure is recorded with its `clkdiag.log`.

## 8. Relock holds the trim (clkdiag image)

For SPDIF in, ADAT slave and I2S slave, pull the source cable for about 2 seconds while logging, then reconnect.

- [ ] Each input relocks without a reboot.
- [ ] The commanded ppm resumes near its pre-pull value instead of restarting from 0. This is the 2e change.
- [ ] No starvation after relock.

## 9. MCK at 44.1 kHz (normal image)

MCK used to run about 144 ppm away from BCK at 44.1 kHz; 2g fixed that.

- [ ] An MCK-clocked DAC plays 10 minutes at 44.1 kHz (both 128× and 256× MCK) with no periodic dropouts or re-locks.
- [ ] Optional: a frequency counter shows MCK / BCK at exactly 2 (128×) or 4 (256×).

## 10. ADAT output (normal image, RP2350)

- [ ] The ADAT receiver stays locked through the flash writes of section 5.
- [ ] The ADAT offset (section 4 method) stays constant after host underruns: pause and resume the host stream ten times.
- [ ] The ADAT offset stays constant across five pipeline resets (preset loads).

## 11. Soak (loopback image)

The old 6.2-hour counter-coherence failure is the target. Play continuous USB audio for at least 6.5 hours.

- [ ] Fill stays steady, which you can watch in the Console buffer stats.
- [ ] `GET_STATUS` starvation values 17 to 21 do not climb.
- [ ] A loopback suite run after the soak still passes.

## 12. Underrun recovery (normal image)

Force underruns by stalling the host: pause and resume playback, and switch the system output device away and back.

- [ ] After every recovery the slot offsets are unchanged (section 4 method).
- [ ] Audio returns cleanly each time.

## 13. Behaviour changes users will see

- [ ] RP2040: binding a PWM LED or aux PWM output to GPIO 14 or 15 is refused with `CS_STATUS_PWM_CONFLICT`. Slice 7 belongs to the soft VCXO.
- [ ] DSPi Console connects, and its buffer stats show sample-granular fill (`fill_centi_pct`).
- [ ] `REQ_GET_CLOCK_DIAG` (0x82) returns zero bytes on the normal image and a full packet on the clkdiag image.
- [ ] If an ESP32 front panel is available: it syncs and shows no communication fault.

## Afterwards

Record the results in the merge plan's Progress section under "Phase 4 results". Any section-4 slot failure, or any section-5 receiver unlock, blocks the merge from landing. Phase 3 (producer-pool removal) starts only after this checklist passes.
