# dspi_test — DSPi firmware control-plane QA suite

A repeatable, host-executable test harness that exercises **every** DSPi vendor
command over USB and asserts the device never crashes, hangs, stalls
unexpectedly, or returns wrong/out-of-range results. This is the standardized
firmware-stability gate.

It is **control-plane only by default**: it verifies command round-trips, parameter
ranges, validation behavior (STALL / in-band error / silent no-op / clamp), flash
persistence, cross-feature state, and device liveness. The default firmware has no USB
capture endpoint, so the core suite does not measure audio. An **optional hardware
audio-loopback group** (`--audio`) adds real signal measurement when DSPi is flashed
with the `DSPI_LOOPBACK` build, which exposes output slot 0 as a USB capture input
(no external recorder or wiring); see [Audio loopback](#audio-loopback-optional).

> **New here?** [`test_harness.md`](test_harness.md) is the full, explain-everything guide
> to the audio-loopback rig: the DSPI_LOOPBACK firmware, setup, every test and parameter,
> and troubleshooting. This README is the quick-start.

## Requirements

```
pip install pyusb           # plus `brew install libusb` on macOS
```
A DSPi plugged in (VID `0x2E8B`, PID `0xFEAA`), not held exclusively by another
app. Run from the repo root so `config.h` opcode parsing works.

## Run

```bash
# COMPLETE SUITE — run everything in one command (audio loopback + flash +
# factory reset). This is the one to use if you just want "run all the tests".
# The audio group runs first so its auto-probe sees a pristine device. Needs DSPi
# flashed with the DSPI_LOOPBACK build + sounddevice for the audio part (those tests
# SKIP, never fail, if absent):
python3 -m tools.dspi_test.run --all

# Full non-flash run (safe, fast, no flash wear):
python3 -m tools.dspi_test.run

# Include flash-writing tests (preset save/load/delete, names, startup,
# output-config mode/save, master-volume save, DAC-mute config). Budget-capped & restored:
python3 -m tools.dspi_test.run --allow-flash

# Also run the one-shot factory-reset test:
python3 -m tools.dspi_test.run --allow-flash --allow-factory-reset

# Only some groups; write reports:
python3 -m tools.dspi_test.run --group eq,volume --report report.md --json report.json

# Heavier stress/fuzz:
python3 -m tools.dspi_test.run --stress 500

# List tests / write the catalog without touching the device:
python3 -m tools.dspi_test.run --list
python3 -m tools.dspi_test.run --catalog catalog.md
```

Exit code is `0` only if there are no `FAIL`/`ERROR` results.

### Options

| Flag | Effect |
|---|---|
| `--all` | **Complete suite:** audio loopback + flash + factory reset, in one command (= `--audio --allow-flash --allow-factory-reset`). Audio runs first. |
| `--group G[,G...]` | Run only these groups (default: all except `audio`). |
| `--audio` | Include the hardware audio-loopback group (needs the DSPI_LOOPBACK firmware + sounddevice). |
| `--allow-flash` | Enable flash-writing tests (capped at `--flash-cap`, default 30 erases). |
| `--allow-factory-reset` | Enable the one-shot factory-reset test. |
| `--flash-cap N` | Hard cap on flash erase cycles for the run. |
| `--stress N` | Iteration count for stress/fuzz tests. |
| `--report PATH.md` / `--json PATH` | Write reports. |
| `--catalog PATH.md` | Emit the test catalog (from metadata) and exit. |
| `--no-restore` | Skip the final snapshot restore (debugging only). |
| `--quiet` | Suppress per-test console lines. |

## Groups

`identity` · `eq` · `dynamics` · `outputs` · `volume` · `inputs` · `diagnostics`
· `presets` · `crosscut` · `stress` · `rta` (spectrum analyser; needs the
siggen, one test also needs `--audio`) · `audio` (opt-in, `--audio`)

## Audio loopback (optional)

The `audio` group is the one part of the suite that measures **real audio**. It plays a
signal out the DSPi USB audio output, lets the DSP process it, and captures **output
slot 0** from DSPi's own USB capture input (the `DSPI_LOOPBACK` firmware build taps
slot 0 after all DSP and streams it back — no external recorder or wiring), then
verifies the measured result against the firmware's filter math
(`tools/filter_tester/compare_filter.py`). The chain is single-clock and digital, so the
capture is a fixed-latency, near bit-exact copy of slot 0.

It is excluded from the default run. **Flash DSPi with the `DSPI_LOOPBACK` build** (the
default firmware has no capture endpoint), then enable with `--audio` (or
`--group audio`) and install the extra deps:

```
pip install sounddevice numpy scipy     # macOS also: brew install portaudio
```

Then:

```bash
# Bring-up: enumerate host audio devices, then a raw loopback tone + level/THD/noise:
python3 -m tools.dspi_test.audio --list
python3 -m tools.dspi_test.audio --probe

# Run the loopback suite (integrity baseline + PEQ frequency response):
python3 -m tools.dspi_test.run --audio --group audio
```

What it checks:

- **`loopback_integrity`** — flat path: signal reaches the DSPi capture at unity gain,
  low noise floor and THD, and near bit-exact reproduction.
- **`peq_*`** — sets each PEQ type (peaking, shelves, low/high pass, notch, first-order
  shelves) on the target output and asserts the measured magnitude response matches the
  RBJ reference within 0.7 dB, spanning both sides of the RP2350 SVF/biquad boundary.
- **`loopback_allpass_phase`** — first-order all-pass: flat magnitude and correct phase shape.
- **`xo_*`** — sets each crossover type (Linkwitz-Riley, Butterworth, Bessel; orders 1-8;
  LP and HP) on a crossover band and asserts the measured magnitude response matches the
  scipy reference (mirrors `tools/filter_tester/test_crossover.py`) within 1 dB over the
  measurable region.
- **`xo_lr4_complementary_sum`** — LR4 LP+HP measured in one capture sum to flat magnitude
  (the defining Linkwitz-Riley property).
- **output controls** — `output_gain_level` (level tracks set dB), `output_mute_silences`
  (mute -> silence), `level_controls` (master volume / user volume / preamp scale by the set
  dB), `matrix_routing` (crosspoint enable), `matrix_phase_invert` (polarity flips), and
  `output_delay` (a per-output delay shifts that leg by exactly the set sample count).
- **alignment** — `slot_lr_alignment` (the slot's L/R are sample-aligned) and that this
  survives pipeline-reset operations: `alignment_after_input_switch` (USB->S/PDIF->USB) and
  `alignment_after_output_type_switch` (S/PDIF->I2S->S/PDIF). Note: the capture taps one
  slot (slot 0), so this checks INTRA-slot L/R; full inter-slot alignment needs multichannel capture.
- **output types** — `output_type_i2s_audio` measures real audio with slot 0 as **I2S**
  (unity, low THD, near bit-exact, L/R aligned); since the tap is pre-encoder, a dead I2S
  output path stalls the producer and the tone vanishes, so a clean tone proves the I2S
  consumer is draining. `output_type_switch_stress` cycles slot 0 SPDIF<->I2S several times
  and re-measures after each switch (signal present, unity level, L/R aligned), exercising the
  shared-DMA-channel teardown/re-setup path. Both SKIP if the I2S switch is unavailable.
- **full chain / dynamics** — `multiband_eq` (stacked PEQ bands = sum of their responses),
  `loudness_shape` (loudness boosts bass/treble at low volume), `crossfeed_bleed` (one channel
  bleeds an attenuated copy into the opposite), `leveller_boost` (a quiet signal is lifted
  within the max-gain ceiling), and `output_clip_limit` (driving past 0 dBFS clamps at full
  scale and raises THD instead of wrapping).

The captured slot (always slot 0) is **auto-probed** once per session. macOS prompts for
microphone access for the DSPi capture input on first run. Both the output and the capture
are matched by the name substring `DSPi`; the harness disambiguates them by direction
(output vs input channels). Adjust in `audio.py` if your OS names them differently.

## How it stays safe

- **Snapshot/restore.** Before any test, the full live DSP state is captured via
  `REQ_GET_ALL_PARAMS` (plus directory metadata). After the run it is restored
  via `REQ_SET_ALL_PARAMS` and verified byte-for-byte. Mutating tests also
  restore hardware resources (pins, output types, input source, MCK) inline.
- **Flash minimization.** Each flash-writing test declares its erase cost; the
  run is hard-capped and never loops a flash writer. Preset lifecycle uses one
  scratch slot (the highest unoccupied) and restores it.
- **Liveness sentinel.** After every mutating test the device is polled
  (`REQ_GET_PLATFORM`); a device that never recovers aborts the run.
- **Resilience.** The transport retries transient timeouts and **re-acquires the
  handle on re-enumeration** (some heavy ops briefly disable the USB control IRQ
  long enough that the host resets the bus — recoverable, and the count is
  reported).
- **Exclusions.** `0xF0 ENTER_BOOTLOADER` is never sent (would drop the device
  to BOOTSEL); it appears as a documented SKIP. (`0x52` is no longer excluded —
  the dead `LOAD_PARAMS` was repurposed as `REQ_SAVE_OUTPUT_CONFIG`, a safe
  action covered by `output_config_save`.)

## Paged protocol and the translation layer

*Added 2026-11-16 for the vendor command overhaul (`Documentation/Features/vendor_command_overhaul_plan.md`).*

`DspiDevice` speaks both protocols. Paged commands go through `dev.pget(cmd, addr=0, length=None)` and
`dev.pset(cmd, data=b"", addr=0)`, where `cmd` is a name such as `"FILTER.BAND"` or a numeric ID such as
`0x0610`. Names resolve through `Documentation/protocol/dspi_commands.json`, then through the plan
numbering in `Documentation/protocol/legacy_map.json`. A refused paged request raises `PagedStall` (a
`Stall`) whose `result_name` is the `VC_*` code read back from `SYS.LAST_RESULT`. `dev.protocol_info()`
returns `None` on firmware without the paged dispatcher, and every paged request is refused locally on
such firmware, because 1.1.6 would run it as the old opcode with the same number.

The legacy `dev.get(OP.X, ...)` and `dev.set(...)` calls have two modes:

```bash
DSPI_TEST_PROTOCOL=old        python3 -m tools.dspi_test.run   # default: old opcodes on page 0
DSPI_TEST_PROTOCOL=translate  python3 -m tools.dspi_test.run   # old calls become paged commands
python3 -m tools.dspi_test.legacy_map --check                  # validate the mapping data
```

In translate mode `legacy_map.py` follows `legacy_map.json` (its `_schema` key documents the format).
Write-as-read commands become SETs and answer the 1.1.6 status byte. A refused OUT write is swallowed,
as 1.1.6 ACKed it, and logged in `dev.swallowed`. `legacy_map.golden_transport` replays the golden
capture through the same layer:
`python -m tools.dspi_test.golden.capture compare ... --transport tools.dspi_test.legacy_map:golden_transport`.
The layer and the `old` mode are removed in plan Phase 6. The JSON stays as migration data.

`DSPI_TEST_PROTOCOL=hybrid` (or `protocol="hybrid"`) is for the page migration, while page 0 still
answers. An old call is translated only when every paged command it would use exists on the attached
device in the needed direction; otherwise it goes to page 0 exactly as in `old` mode. The decision is
per opcode, per split selector, and per `GET_STATUS` selector. Custom adapters declare what they touch
in the `uses` list of their JSON entry. What exists is read from `SYS.PAGE_LIST` and
`SYS.PAGE_COMMANDS` once per handle (not from the registry), so reconnect or make a new `DspiDevice`
after flashing. A request that would use no paged command at all stays on page 0.
`dev.hybrid_counts` counts calls per old opcode name, `dev.hybrid_summary()` gives a one-line summary,
and the summary is printed at exit. For the golden replay use
`--transport tools.dspi_test.legacy_map:golden_transport_hybrid`.

`tests/paged_sys.py` holds native tests for the `SYS` discovery commands. They SKIP on firmware without
the paged dispatcher.

## Layout

| File | Role |
|---|---|
| `device.py` | `DspiDevice` USB transport; typed get/set; paged `pget`/`pset`; STALL/Timeout/Disconnect handling; opcode map. |
| `legacy_map.py` | Temporary translation of 1.1.6 requests into paged commands, and its `--check`. |
| `profile.py` | `PlatformProfile` — probes the device for counts/ceilings (platform-adaptive). |
| `lifecycle.py` | Snapshot & restore. |
| `framework.py` | Test registry, `Check` assertions, `Runner`, reports, catalog. |
| `helpers.py` | Reusable round-trip / clamp / bool / NaN patterns. |
| `run.py` | CLI entry point. |
| `tests/` | One module per command group. |

## Adding a test

```python
from ..device import OP
from ..framework import test

@test("mygroup", mutating=True, flash=0)
def my_check(dev, profile, chk):
    """One-line description (shown in the catalog)."""
    dev.set_f32(OP.SET_SOMETHING, 1.5)
    chk.approx(dev.get_f32(OP.GET_SOMETHING), 1.5, 1e-3, "round-trip")
    chk.stalls(lambda: dev.get(OP.GET_SOMETHING, 4, wvalue=999), "bad index STALLs")
```
Set `mutating=True` if it changes device state, and `flash=N` (erase cost) if it
writes flash. The global snapshot restores live state; restore hardware
resources (pins/types/input) inline in a `finally`.
