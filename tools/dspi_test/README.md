# dspi_test: DSPi firmware QA suite

**TL;DR.** This is the host-side test suite for the DSPi firmware. It talks to a DSPi over USB, sends
it vendor commands, and checks every answer: values round-trip, ranges clamp, bad requests are refused
with the right reason, settings persist, and the device never crashes or hangs. It speaks only the paged
vendor protocol of firmware 1.1.7 and later. A run puts the device back the way it found it.

By default the suite checks the control plane only, because release firmware has no USB capture
endpoint. An optional audio group (`--audio`) also measures real signal when DSPi runs the
`DSPI_LOOPBACK` build. [`test_harness.md`](test_harness.md) is the full guide to that audio rig. This
README is the quick start.

## Requirements

```
pip install pyusb           # plus `brew install libusb` on macOS
```

- A DSPi on USB (VID `0x2E8B`, PID `0xFEAA`), not held by another app. Close DSPi Console first.
- **Firmware 1.1.7 or later.** The harness reads `SYS.PROTOCOL_INFO` before anything else. Older
  firmware does not answer it, and the harness then stops with a `ProtocolError` saying it needs
  1.1.7 or later. It sends such firmware nothing else, because 1.1.6 would run a paged command as
  whichever old opcode shares its number.
- Run from the repo root, so the command registry `Documentation/protocol/dspi_commands.json` is found.

## Run

```bash
# Default run: every group except audio. No flash writes.
python3 -m tools.dspi_test.run

# Add the flash-writing tests (presets, names, startup, output-config and master volume saves).
# Erases are budget-capped (default 30) and every change is put back.
python3 -m tools.dspi_test.run --allow-flash

# Everything: audio group, flash tests and the factory-reset gate.
python3 -m tools.dspi_test.run --all

# Some groups only, with reports.
python3 -m tools.dspi_test.run --group eq,volume --report report.md --json report.json

# Heavier stress and fuzz.
python3 -m tools.dspi_test.run --stress 500

# List the tests, or write the catalog. Neither needs a device.
python3 -m tools.dspi_test.run --list
python3 -m tools.dspi_test.run --catalog catalog.md
```

The exit code is 0 only when no test FAILs or ERRORs.

### Options

| Flag | Effect |
|---|---|
| `--group G[,G...]` | Run only these groups. Default: every group except `audio` |
| `--allow-flash` | Run the flash-writing tests, capped at `--flash-cap` erases |
| `--flash-cap N` | Hard cap on flash erases for the run (default 30) |
| `--allow-factory-reset` | Gate for factory-reset tests. No current test needs it: `STORE.FACTORY_RESET` is only sent in shapes the firmware refuses |
| `--audio` | Include the audio loopback group |
| `--audio-rates HZ[,HZ...]` | Rates for the audio group (default `48000,44100`) |
| `--all` | Same as `--audio --allow-flash --allow-factory-reset`. The audio group runs first |
| `--stress N` | Iterations for the stress and fuzz tests (default 0, a light pass) |
| `--report PATH.md`, `--json PATH` | Write reports |
| `--list`, `--catalog PATH.md` | List the tests, or write the catalog from test metadata, and exit |
| `--no-restore` | Skip the final restore (debugging only) |
| `--quiet` | No per-test console lines |

## How the tests are organised

There are two families of test modules, with different jobs.

- **`paged_*` modules: protocol conformance, one per page.** They check each page against the
  specification in `Documentation/commands.md`. Discovery data matches the registry, every reply has
  its documented shape, values round-trip, and wrong lengths, addresses, directions and non-finite
  floats are refused with the documented `VC_*` code. `paged_page0` proves page 0 answers nothing.
  `paged_pins` checks the pin availability queries against what the real pin SETs do.
- **The other modules: feature behaviour.** Clamps and ranges, interactions between features, bulk
  parameter round trips, stress and fuzz, the spectrum analyser, preset isolation, pin moves and the
  audio loopback. They assume the protocol works and test what the device does with it.

A new test goes in the family that matches its purpose. A shape or refusal check belongs in the page's
`paged_*` module, and a behaviour check belongs in the feature module.

### Groups

From `python3 -m tools.dspi_test.run --list` (201 tests in 28 groups, plus the opt-in `audio` group).

| Group | Tests | Group | Tests |
|---|---|---|---|
| `identity` | 4 | `paged_sys` | 18 |
| `eq` | 2 | `paged_store` | 12 |
| `dynamics` | 15 | `paged_input` | 13 |
| `outputs` | 9 | `paged_output` | 5 |
| `volume` | 3 | `paged_clock` | 5 |
| `inputs` | 11 | `paged_xport` | 2 |
| `diagnostics` | 3 | `paged_route` | 16 |
| `presets` | 5 | `paged_filter` | 7 |
| `crosscut` | 7 | `paged_dyn` | 6 |
| `stress` | 3 | `paged_fx` | 8 |
| `rta` | 8 | `paged_meas` | 10 |
| `control_surfaces` | 3 (opt-in) | `paged_cs` | 7 |
| `audio` | opt-in, `--audio` | `paged_cs_nouns` | 1 |
| | | `paged_diag` | 6 |
| | | `paged_page0` | 2 |
| | | `paged_pins` | 10 |

### Opt-in tests and gates

- **`audio`.** Runs only with `--audio`, `--all` or an explicit `--group audio`. Needs the
  `DSPI_LOOPBACK` firmware and `sounddevice`. See [Audio loopback](#audio-loopback-optional).
- **Bench-build `DIAG` tests.** `DIAG.CLOCK_DIAG`, `DIAG.LOOPBACK_STATS` and `DIAG.LOOP_PROF_*` exist
  only in the `DSPI_CLOCK_DIAG`, `DSPI_LOOPBACK` and `DSPI_LOOP_PROF` builds. Their shape tests SKIP
  on a release build.
- **`rta_input_tap_usb_sine`** plays a tone to the DSPi over USB, so it SKIPs without `sounddevice`.
  Any firmware build works.
- **Flash tests** SKIP without `--allow-flash`.
- **`SYS.ENTER_BOOTLOADER`** is never sent. Its quarantine test is a permanent SKIP.

## How a run stays safe

- **Snapshot and restore.** Before the first test the harness reads the whole live state with
  `STORE.ALL_PARAMS`, plus the preset directory, startup settings, pins, output types and names. After
  the run it writes the bulk state back and checks it byte for byte. Tests that move pins, output types,
  the input source or MCK put those back themselves.
- **Flash budget.** Each flash test declares its erase cost. The runner enforces the cap and never
  loops a flash writer. Preset tests use one scratch slot and restore it.
- **Liveness.** After every mutating test the runner polls `SYS.PLATFORM` (`wait_ready`). A device
  that never answers again aborts the run.
- **Re-enumeration.** Some operations block control transfers briefly, and the host may reset the
  bus. The transport retries timeouts and re-acquires the handle. The report counts re-enumerations.

## Writing a test

Tests call the paged protocol by name. `dev.pget(cmd, addr=0, length=None)` and
`dev.pset(cmd, data=b"", addr=0)` take a `PAGE.NAME` (or a numeric ID). The typed forms are
`pget_u8/u16/u32/i32/f32/str` and `pset_u8/u16/u32/i32/f32`. A refused request raises `PagedStall`,
whose `result_name` is the `VC_*` code read back from `SYS.LAST_RESULT`.

```python
from ..framework import test
from ..helpers import expect_stall, float_roundtrip


@test("mygroup", mutating=True)
def my_output_gain(dev, profile, chk):
    """ROUTE.OUTPUT_GAIN round-trips on output 0 and refuses an output that does not exist."""
    float_roundtrip(dev, chk, "ROUTE.OUTPUT_GAIN", -6.0, addr=0)
    missing = profile.num_output_channels
    expect_stall(chk, lambda: dev.pset_f32("ROUTE.OUTPUT_GAIN", 0.0, missing),
                 "VC_ERR_ADDRESS", "output past the last")
```

Register the module in `tests/__init__.py`. Follow these rules.

- **Use the shared helpers** in `helpers.py` before writing your own. They cover float and bool round
  trips and clamps, `expect_stall` and `expect_result` for expected result codes, non-finite and
  wrong-length refusals, `wait_for` for deferred work, `set_retry_busy` for `VC_ERR_BUSY`,
  `FILTER.BAND` and `ROUTE.MATRIX_ROUTE` packing, and GPIO choice from the firmware's valid-pin rule.
  `telemetry.py` reads the `SYS`, `INPUT`, `MEAS` and `DIAG` telemetry structs, and `tests/_cs.py`
  holds the Control Surfaces support.
- **Expect the exact code.** Assert a refusal with `expect_stall(chk, fn, "VC_ERR_...", msg)`, not a
  bare STALL. It retries because `SYS.LAST_RESULT` is shared by every USB host.
- **Mark what the test does.** `mutating=True` if it changes device state. `flash=N` (erase cost) if it
  writes flash, which also gates it behind `--allow-flash`.
- **Restore what the snapshot cannot.** The final restore covers live DSP state. Put back flash
  contents, pins, output types and the input source in a `finally`.
- **Use `pget_ready` only where success is required.** It retries through a busy window, which would
  hide an expected refusal.

## Audio loopback (optional)

The `audio` group measures real audio. It plays a signal into the DSPi USB output, lets the DSP process
it, and captures output slot 0 from DSPi's own USB capture input. The `DSPI_LOOPBACK` build taps slot 0
after all DSP and streams it back, so no recorder or wiring is needed. The results are checked against
the firmware's filter math (`tools/filter_tester/compare_filter.py`). The chain is digital on one clock,
so the capture is a fixed-latency, near bit-exact copy of slot 0.

Flash DSPi with the `DSPI_LOOPBACK` build, then install the extra packages.

```
pip install sounddevice numpy scipy     # macOS also: brew install portaudio
```

```bash
# Bring-up: list host audio devices, then a raw loopback tone with level, THD and noise.
python3 -m tools.dspi_test.audio --list
python3 -m tools.dspi_test.audio --probe

# The loopback suite.
python3 -m tools.dspi_test.run --audio --group audio
```

The group checks the following.

- **Integrity.** `loopback_integrity`: unity gain, low noise and THD, near bit-exact reproduction.
- **PEQ.** Each PEQ type against the RBJ reference within 0.7 dB, on both sides of the RP2350
  SVF/biquad boundary, and the first-order all-pass phase.
- **Crossovers.** Linkwitz-Riley, Butterworth and Bessel at orders 1 to 8, LP and HP, against the scipy
  reference within 1 dB, and LR4 LP plus HP summing flat.
- **Output controls.** Output gain, mute, master and user volume, preamp, matrix routing and phase
  invert, and per-output delay to the exact sample.
- **Alignment.** Slot 0 left and right stay sample-aligned, also after an input source switch and an
  output type switch. The capture taps one slot, so inter-slot alignment needs multichannel capture.
- **Output types.** Slot 0 as I2S measures clean audio, and repeated S/PDIF to I2S switches recover.
- **Full chain.** Stacked PEQ bands, loudness, crossfeed, leveller and clipping at full scale.

The captured slot is probed once per session. macOS asks for microphone access to the DSPi capture
input on the first run. Output and capture are both found by the name `DSPi` and told apart by
direction. Adjust `audio.py` if your OS names them differently.

## Selftest

```bash
python3 -m tools.dspi_test.selftest
```

No device needed. It checks the harness's own logic against fakes of the firmware. A section whose
optional module (`numpy`, `sounddevice`) is missing reports SKIP and the run still exits 0. It does
need `pyusb` to import. The current result is OK 170, SKIP 1, FAILURES 0.

## Golden regression

`golden/` holds a byte-for-byte record of the firmware's answers to a fixed script of paged requests,
and `python3 -m tools.dspi_test.golden.capture compare --ref ...` reports every reply that changed. The
reference is tied back to firmware 1.1.6. See [`golden/README.md`](golden/README.md).

## Current results

On the bench RP2350 with firmware 1.1.7 beta 1 on 2026-11-17, `--allow-flash --flash-cap 60`: 179 of 192
pass, 1 fails, 12 SKIP. Five of the skips are flash tests past the erase cap, because the whole suite
needs more than 60 erases. Their three groups pass on their own (41 of 41).

- **The failure** is `rta.rta_pink_noise_flat`, which fails about half the time. Every output's
  graded bands sit within 1 to 2 dB, in 0.5 dB steps, and one step of noise takes an output to 2.5 dB
  against the 2 dB limit. The tolerance is tight for the measurement, not a firmware fault.
- **The other skips** are the bootloader quarantine (1), `rta_input_tap_usb_sine` without
  `sounddevice` (1) and the bench-build `DIAG` tests (5). `pdiag_clock_diag_shape` passes on a
  `DSPI_CLOCK_DIAG` build.

## Layout

| File | Role |
|---|---|
| `device.py` | `DspiDevice`: USB transport, `pget`/`pset` and typed forms, `protocol_info`, `last_result`, `paged_commands`, `resolve`, `wait_ready`/`is_alive`, `raw_request` (page 0 test only), and the exceptions `Stall`, `PagedStall`, `Disconnected`, `Timeout`, `ProtocolError` |
| `profile.py` | `PlatformProfile`: counts and ceilings probed from the device |
| `lifecycle.py` | Snapshot and restore |
| `framework.py` | Test registry, `Check` assertions, `Runner`, reports, catalog |
| `helpers.py` | Shared test helpers |
| `telemetry.py` | Readers for the telemetry structs, also used by `tools/latency_bench` |
| `actions.py` | Live-parameter action catalogue shared by the stress test and the latency soak |
| `audio.py`, `coreaudio.py` | Audio measurement engine and macOS sample-rate control |
| `run.py` | CLI entry point |
| `selftest.py` | No-hardware checks of the harness |
| `tests/` | Test modules, one group each. `tests/_cs.py` is shared Control Surfaces support |
| `golden/` | Golden regression |

The Windows `wIndex` check is a separate script, `tools/windex_page_test.py` (procedure in
`Documentation/Features/windex_page_test.md`).
