# latency_bench

Checks that every selectable output latency (the slot ring fill target,
`OUTPUT.LATENCY`) runs without underruns or refused blocks, on every source the
bench can drive without external hardware. Design: `Documentation/current_architecture.md`
"Output Latency". The scripts speak the paged vendor protocol (firmware 1.1.7 and
later) and name commands as `PAGE.NAME`, as in `Documentation/commands.md`.

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

- underrun recoveries and blocks refused by ring admission (`SYS.PIPELINE_STATS`)
- the lowest fill seen just before any write (also `SYS.PIPELINE_STATS`), which is the real margin
- live per-slot fill every 100 ms (`SYS.BUFFER_STATS`): the steady-state mean over the last 10 s,
  and the largest spread between playing slots (an alignment proxy: slots that share a write
  index and a synchronized start should read the same fill)
- USB ring overruns (`SYS.USB_AUDIO_STATUS`), USB packet gaps over 2 ms (`SYS.SPDIF_TX_STATS`,
  whose first field counts them), PDM underruns (`SYS.PDM_STATS`) and the CPU meters
  (`SYS.SYSTEM_STATUS`)

The window starts with `SYS.STATS_RESET` (bit 0), which clears the watermarks and
worst-gap fields. The option is set with `OUTPUT.LATENCY` and the script waits on
`OUTPUT.LATENCY_STATUS` until it has applied. An unknown option ID is refused.

A point passes with no underruns, no refused blocks, a positive minimum, a
steady-state mean between one block below and one block (plus 2 frames of sampling noise) above `target - block/2` (sources anchored by a prefill threshold land up to a block high, and I2S in master mode has no servo to pull them back), and at least 98% of fill readings
with the playing slots within 1 frame of each other (the stats GET reads the slots in
turn with interrupts enabled, so an interrupt between two reads shows as a rare 2-3
frame spread; the maximum is still reported). `--checks` adds a rate-follow check (8 ms at 48 kHz, then
96 kHz, which must clamp to its 351-frame ceiling, then back) and 20 rapid
2 ms / 8 ms switches with the spread read after each.

## Running

Use a Python with pyusb and numpy, from the repo root. The `usb48` source and a
`--source usb` soak also need sounddevice to play to the device. Without it
`usb48` is skipped and a USB soak refuses to start, before either changes anything.

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

`snapshot` only reads. `run` switches the input source (`INPUT.SOURCE`, and
`INPUT.I2S_RX_RATE` for the I2S sources), writes each latency option and holds
`ROUTE.USER_MUTE`. At the end it puts back the source and the mute, not the latency
or the I2S rate, and reports whether the parameters changed. `--profile 4way` loads an
empty preset slot (`STORE.PRESET_LOAD`) and applies the heavy 4-way profile that
`tools/batch_bench` also uses (both cores busy), written with `FILTER.BAND`,
`ROUTE.MATRIX_ROUTE`, `ROUTE.OUTPUT_ENABLE` and `ROUTE.CHANNEL_DELAY`. `restore` reloads the original slot,
pushes the captured parameters back and restores the original latency. The `siggen`
source starts the signal generator (`MEAS.SIGGEN_START`) on outputs 1 to 8.

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
- once a second it reads underruns, refused blocks, the lowest pre-write fill,
  the worst input delivery gap and background steps over budget
  (`SYS.PIPELINE_STATS`), USB ring overruns, the packet count and the worst packet
  gap of any length (`SYS.USB_AUDIO_STATUS`), USB packet gaps of 2 to 50 ms
  (`SYS.SPDIF_TX_STATS`) and the host player's underflow flags, then resets the
  watermarks (`SYS.STATS_RESET`). On a `DSPI_CLOCK_DIAG` build it also records
  the USB feedback range for the second (`DIAG.CLOCK_DIAG`, absent from other builds).

Any second with an underrun, refused block, host underflow, a delivery gap over
300 us or a USB packet gap over 5 ms is logged with
the actions of the preceding 2.5 s, so a stall points at its cause. An action the
device refuses is counted with " (stalled)" after its name. The run
refuses to start over an unsaved Control Surfaces preview (`CS.STATUS`), holds the user mute,
and restores the parameters, analyser config (`MEAS.RTA_CONFIG`), latency and source at the end.
Its actions include signal generator bursts (`MEAS.SIGGEN_START`) and `CS.REVERT`, so run it
only on a bench where both are acceptable.

The summary ends with a verdict, and the script exits with status 1 when any
check fails:

- no underruns, except in seconds the host stopped sending (a USB packet gap
  over 5 ms), which are reported separately as host stalls;
- no refused blocks, and no background step over budget;
- every second's worst delivery gap under the option's target minus its batch;
- the lowest pre-write fill never zero.

On a `DSPI_LOOP_PROF` build the summary also holds the whole run's worst
main-loop sections, background steps and vendor handlers by command
(`DIAG.LOOP_PROF_SECTION`, `_JOB` and `_COMMAND`, cleared by `DIAG.LOOP_PROF_RESET`).

```bash
python3 tools/latency_bench/soak.py --out bench_results/latency_options/soak --source spdif --option 1 --minutes 15
```

`--profile-actions N` instead times every action alone N times with the analyser
held stopped, giving the main-loop cost of each host operation. On a
`DSPI_LOOP_PROF` build it also reports the costliest sections, steps and commands.
Section and job names come from `firmware/DSPi/loop_prof.h` (`loop_prof.py`).

## Loop profiler

`loop_prof.py` reads the main-loop profiler on a `DSPI_LOOP_PROF` build. It resets
the worsts, waits, then prints the worst time of each section, each background job
and the costliest commands by `PAGE.NAME`. Reads never clear. `--pace N` makes each
background step run on one pass in N for the window, then sets it back to 1. Other
builds have no `DIAG.LOOP_PROF_*` commands, and the script says so and exits.

```bash
python3 tools/latency_bench/loop_prof.py --seconds 5 --top 15
```

## Shared helpers

`paged.py` reads each telemetry struct in one transfer and names the fields as the
firmware does. `SYS.SPDIF_TX_STATS` field `spdif_underruns` is the USB packet gap
count, reported as `usb_packet_gaps`.
