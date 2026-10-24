# latency_bench

Checks that every selectable output latency (the slot ring fill target, vendor
opcode `0x03`) runs without underruns or refused blocks, on every source the
bench can drive without external hardware. Design: `Documentation/current_architecture.md`
"Output Latency".

## What it drives

| Source | How | Rates |
|---|---|---|
| `usb48` | Loops -60 dBFS noise to the DSPi from the host | 48 kHz (CoreAudio will not move the output endpoint off 48 kHz) |
| `i2s44`, `i2s48`, `i2s96` | I2S input in clock-master mode; nothing needs to be wired, the input still runs on the device clock | 44.1 / 48 / 96 kHz |
| `siggen` | USB selected with no host stream, so the test-signal pump is the only producer | the current rate |
| `spdif` | S/PDIF input 1, only if something locks within 4 s | the detected rate |

The run holds the user mute on throughout. An undriven I2S data pin reads as
full-scale noise, and mute is a gain stage that leaves ring timing unchanged.

## What it measures

For each latency option (all seven by default, `--options` to narrow), the window
starts the moment the change applies, so the post-reset transient counts:

- underrun recoveries and blocks refused by ring admission (`REQ_GET_STATUS` 26 and 27)
- the lowest fill seen just before any write (28), which is the real margin
- live per-slot fill every 100 ms: the steady-state mean over the last 10 s, and the
  largest spread between playing slots (an alignment proxy: slots that share a write
  index and a synchronized start should read the same fill)
- USB ring overruns, USB packet gaps, PDM underruns and the CPU meters

A point passes with no underruns, no refused blocks, a positive minimum, a
steady-state mean between one block below and one block (plus 2 frames of sampling noise) above `target - block/2` (sources anchored by a prefill threshold land up to a block high, and I2S in master mode has no servo to pull them back), and at least 98% of fill readings
with the playing slots within 1 frame of each other (the stats GET reads the slots in
turn with interrupts enabled, so an interrupt between two reads shows as a rare 2-3
frame spread; the maximum is still reported). `--checks` adds a rate-follow check (8 ms at 48 kHz, then
96 kHz, which must clamp to its 351-frame ceiling, then back) and 20 rapid
2 ms / 8 ms switches with the spread read after each.

## Running

Use a Python with pyusb, numpy and sounddevice, from the repo root.

```bash
python3 tools/latency_bench/latency_bench.py snapshot --out bench_results/latency
```

```bash
python3 tools/latency_bench/latency_bench.py run --out bench_results/latency --profile user --checks
```

```bash
python3 tools/latency_bench/latency_bench.py run --out bench_results/latency --profile 4way
```

```bash
python3 tools/latency_bench/latency_bench.py restore --out bench_results/latency
```

`--profile 4way` loads an empty preset slot and applies the heavy 4-way profile
from `tools/batch_bench` (both cores busy). `restore` reloads the original slot,
pushes the captured parameters back and restores the original latency.

## Soak

`soak.py` runs one source at one option for many minutes while the host behaves
like a busy DSPi Console session, which steady-state windows cannot reproduce:

- the spectrum analyser is read every 50 ms, rotating through the outputs;
- every 1 to 3 s a random action runs, drawn from the shared live-parameter
  catalogue (`tools/dspi_test/actions.py`, also used by the stress test) plus
  feature toggles, full parameter reads, analyser reconfiguration, signal
  generator bursts and Control Surfaces bindings (input types only: pots on the
  floating ADC pins, whose noise drives real dispatches, and buttons that never
  target a mute noun);
- once a second it reads underruns, refused blocks, USB ring overruns, the
  lowest pre-write fill and the worst input delivery gap (`REQ_GET_STATUS`
  26-29), USB packet gaps of 2 to 50 ms, the packet count and the worst packet
  gap of any length (8, 10, 30), background steps over budget (31), and the
  host player's underflow flags. On a `DSPI_CLOCK_DIAG` build it also records
  the USB feedback range for the second (`REQ_GET_CLOCK_DIAG`).

Any second with an underrun, refused block, host underflow, a delivery gap over
300 us or a USB packet gap over 5 ms is logged with
the actions of the preceding 2.5 s, so a stall points at its cause. The run
refuses to start over an unsaved Control Surfaces preview, holds the user mute,
and restores the parameters, analyser config, latency and source at the end.

The summary ends with a verdict, and the script exits with status 1 when any
check fails:

- no underruns, except in seconds the host stopped sending (a USB packet gap
  over 5 ms), which are reported separately as host stalls;
- no refused blocks, and no background step over budget;
- every second's worst delivery gap under the option's target minus its batch;
- the lowest pre-write fill never zero.

On a `DSPI_LOOP_PROF` build the summary also holds the whole run's worst
main-loop sections, background steps and vendor handlers by opcode.

```bash
python3 tools/latency_bench/soak.py --out bench_results/latency_options/soak --source spdif --option 1 --minutes 15
```

`--profile-actions N` instead times every action alone N times with the analyser
held stopped, giving the main-loop cost of each host operation. On a
`DSPI_LOOP_PROF` build it also reports the costliest sections, steps and opcodes.
Section and job names come from `firmware/DSPi/loop_prof.h` (`loop_prof.py`).
