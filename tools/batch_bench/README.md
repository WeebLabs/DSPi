# batch_bench

Measures how DSP pipeline CPU load changes with input batch size: the number of
frames handed to `process_input_block()` per call. USB audio is the test bed.

## How it works

The firmware must be built with `-DDSPI_BATCH_BENCH=ON`. That build stages USB
input and runs the pipeline in exact k-frame calls, carrying any remainder into
the next packet, so every call is uniform. k is set at runtime (1 to 96; 0 means
one call per USB packet, the release behaviour). The build also adds up exact
busy time on each core, read back through vendor GET `0x6F` (`BatchBenchStats`
in `firmware/DSPi/batch_bench.h`).

CPU percentage is busy time divided by the real-time duration of the frames
processed. That is the same definition the stock meter uses, but summed exactly
over the window instead of smoothed and rounded to whole percent.

## Running

Use a Python with pyusb, numpy and sounddevice, from the repo root.

```bash
cmake -S firmware -B build-rp2350-batchbench -DPICO_PLATFORM=rp2350-arm-s -DPICO_BOARD=pico2 -DDSPI_BATCH_BENCH=ON
```

```bash
python3 tools/batch_bench/batch_bench.py snapshot --out bench_results/batch_bench
```

```bash
python3 tools/batch_bench/batch_bench.py run --out bench_results/batch_bench --profile 4way
```

```bash
python3 tools/batch_bench/batch_bench.py restore --out bench_results/batch_bench
```

`run` loads an empty preset slot to get factory DSP defaults, which writes the
preset directory. It then applies the profile and plays seeded white noise at
-60 dBFS into the DSPi. After every step it re-reads the full parameter blob and
aborts if any byte differs from the reference, which is saved as
`config_<profile>.bin`. Each pass sweeps k downward and then back up, and stops
descending once core 0 passes 92 %.

`restore` only works when the firmware's wire format matches the snapshot's.

## Profiles

- `minimal`: factory DSP defaults.
- `4way`: stereo USB in, all S/PDIF outputs on, PDM off (so the core 1 EQ worker
  runs), 10 peaking PEQ bands on both inputs and every output, an LR4 4-way
  crossover, and 1.5 ms delay on outputs 1-2.
