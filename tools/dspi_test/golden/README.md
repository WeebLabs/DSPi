# Golden capture of the 1.1.6 vendor protocol

A byte-for-byte record of how firmware 1.1.6 answers its vendor requests, used to prove that the
vendor command overhaul changes nothing but the addressing (plan section 7.5,
`Documentation/Features/vendor_command_overhaul_plan.md`).

The capture reads every readable opcode at every meaningful `wValue` (plus a few out-of-range
ones, so STALL and zero replies are recorded too), runs a write script that moves every settable
parameter to a distinct non-default value, reads everything again, and puts the device back the
way it found it.

## Files

| File | What it is |
|---|---|
| `script_rp2350.json`, `script_rp2040.json` | The replayed requests, as data in legacy terms: `op` (REQ_ name without the prefix), `code`, `dir` (`R` IN read, `W` OUT write, `WR` write-as-read), `wvalue`, `payload` hex, `len` (IN length). Optional `settle` says how long to wait after a write (`delay_ms`, `poll: ready`, `poll: cs`) |
| `script.py` | Generates the two script files. Run `python -m tools.dspi_test.golden.script` after editing it. The JSON is the frozen artifact |
| `legacy_opcodes.json` | The 1.1.6 opcode table, frozen so the tool keeps working after `config.h` is renumbered |
| `capture.py` | Capture, compare and selfcheck |
| `mask.json` | Byte ranges that legitimately vary (meters, counters, temperature, serial, versions), each with a reason |
| `rp2350_1.1.6.json` | The reference capture |

## Sequence

1. Read everything (state 0) and check preconditions.
2. Baseline: `SET_ALL_PARAMS` with the baseline blob (the device's own state on capture, the
   reference's blob on compare), then stop and reset the transient engines (siggen, RTA, subharm solo,
   clip latch, buffer watermarks).
3. Read everything (state A).
4. Run the write script for the selected tiers, recording each write-as-read reply and the
   settled `GET_CS_STATUS` after each Control Surfaces SET.
5. Read everything (state B).
6. Restore from state 0 and read everything again. Any unmasked difference from state 0 is reported.

## Use

Run from the repo root with a Python that has pyusb. Close DSPi Console and any other host first:
the tool refuses to run while another host is polling (`--allow-other-hosts` overrides).

```bash
python -m tools.dspi_test.golden.capture capture --out tools/dspi_test/golden/rp2350_1.1.6.json --allow-flash --allow-reconfig
python -m tools.dspi_test.golden.capture compare --ref tools/dspi_test/golden/rp2350_1.1.6.json --allow-flash --allow-reconfig
python -m tools.dspi_test.golden.capture capture --selfcheck     # lists bytes that change run to run
```

`compare` must use the same tiers as the reference, exits 1 on any unmasked difference and prints
each one as section, request, byte offset, expected and got. `--transport module:function`
replaces the legacy transport (signature `(dev, direction, code, wvalue, data)`), which is how a
later phase replays this script through the paged translation layer.

## Tiers

- **default**: live DSP state only. No flash, no pins. Restored by the bulk SET.
- **flash** (`--allow-flash`): Control Surfaces previews, device and source names, a preset name on
  slot 9, startup, both persistence modes, `SAVE_MASTER_VOLUME`. Restored by writing the old values
  back, then `CS_SAVE`.
- **reconfig** (`--allow-reconfig`, RP2350): input-side and storage-only IO settings that touch no
  driven pin while USB is the source: input rate, I2S and ADAT clock modes, ADAT input, optional
  S/PDIF input enable and pins, I2S RX pins and channel count, BCK pins, clock-pin mode, MCK
  multiplier and pin, ADAT output pin. Needs USB as source, no I2S slot, and output-config mode 1,
  because the bulk SET is the backstop for pins the firmware will not re-set one at a time.

## Excluded, and why

- `ENTER_BOOTLOADER`, `FACTORY_RESET`: destructive.
- `SET_UART_CONFIG`, `SET_I2C_CONFIG`, DAC hardware mute config and test: can cut a control link or
  pulse a board pin.
- Output pin moves, output type switches, MCK and ADAT output enable, input source switches away from
  USB: they drive pins or restart the outputs.
- `OUTPUT_LATENCY` SET, `SET_ALL_PARAMS(_CHUNK)` as a scripted write: a directory flash write plus an
  output restart, and the bulk SET is the restore mechanism.
- `PRESET_SAVE`, `PRESET_LOAD`, `PRESET_DELETE`, `SAVE_PARAMS`: they move `last_active_slot` or
  overwrite a preset, which no command can put back.
- `SAVE_OUTPUT_CONFIG`, `CS_SAVE` as a scripted write: no readback shows the stored block.
- `CS_REVERT`: in 1.1.6 it leaves every stored non-button binding down with `PIN_IN_USE`, because
  `control_surfaces_owns_pin()` counts a slot's own stored pins as taken while the revert runs.
- `SET_CS_AUX_STATE`, `SET_CS_AUX_LEVEL`: they need a live aux output binding, which drives a GPIO.
- `SIGGEN_CONTROL` start: it would play a sweep on the outputs.
- Later-offset `GET_ALL_PARAMS_CHUNK` reads: any other host's request tears the chunk session down.
- `GET_STATUS` `wValue` 64 and up: loop-profiler builds treat some of them as commands.
- The bench-only `0x4F` wIndex echo.

The siggen config cannot be put back to its boot value (all zero), because `SIGGEN_SET_CONFIG`
rejects it. After the first run following a boot it is left at a 1 kHz, -20 dBFS sine, staged and
never started.
