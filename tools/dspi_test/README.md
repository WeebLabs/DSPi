# dspi_test — DSPi firmware control-plane QA suite

A repeatable, host-executable test harness that exercises **every** DSPi vendor
command over USB and asserts the device never crashes, hangs, stalls
unexpectedly, or returns wrong/out-of-range results. This is the standardized
firmware-stability gate.

It is **control-plane only**: it verifies command round-trips, parameter ranges,
validation behavior (STALL / in-band error / silent no-op / clamp), flash
persistence, cross-feature state, and device liveness. It does **not** measure
audio (there is no USB capture endpoint on the device).

## Requirements

```
pip install pyusb           # plus `brew install libusb` on macOS
```
A DSPi plugged in (VID `0x2E8B`, PID `0xFEAA`), not held exclusively by another
app. Run from the repo root so `config.h` opcode parsing works.

## Run

```bash
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
| `--group G[,G...]` | Run only these groups (default: all). |
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
· `presets` · `crosscut` · `stress`

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

## Layout

| File | Role |
|---|---|
| `device.py` | `DspiDevice` USB transport; typed get/set; STALL/Timeout/Disconnect handling; opcode map. |
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
