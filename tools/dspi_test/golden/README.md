# Golden regression of the paged vendor protocol

A byte-for-byte record of how the firmware answers a fixed script of paged vendor requests. Run it
against new firmware to see every reply that changed. It began as the proof that the vendor command
overhaul changed nothing but the addressing (plan section 7.5,
`Documentation/Features/vendor_command_overhaul_plan.md`), and it stays as a regression baseline.

The script reads every readable command at every meaningful address, deliberate out-of-range ones
included. It then moves every settable parameter it can safely touch to a distinct non-default value,
reads everything again, and puts the device back the way it found it.

## Files

| File | What it is |
|---|---|
| `script_rp2350.json`, `script_rp2040.json` | The requests, as data. `cmd` (`PAGE.NAME`), `dir` (`G` GET, `S` SET), `addr` (`wValue`), `len` for a GET, `payload` hex for a SET. `settle` says how long to wait after a request (`delay_ms`, `poll: ready`, `poll: cs`). `origin: native` marks reads added in the paged protocol (see Provenance) |
| `reference_rp2350.json` | The reference. Every reply, each refusal as its `VC_*` result code, plus firmware, build string, git state and provenance |
| `mask.json` | Byte ranges that legitimately vary (meters, counters, temperature, serial, versions), each with a reason |
| `capture.py` | Capture, compare and selfcheck |

There is no script generator. The JSON is the artifact. Edit it by hand (one request per line), then
recapture.

## Provenance

The RP2350 script is the 1.1.6 golden script converted request for request through
`Documentation/protocol/legacy_map.json`. The conversion was checked on hardware: replaying the old
script through the translation layer on 1.1.7 beta 1 sent exactly these requests. Requests the layer
answered without the device (retired opcodes, unknown `GET_STATUS` selectors) were dropped. So were
duplicate reads, such as several old status selectors that read the same struct, and the reads the layer
made only to build a SET payload. Reads marked `origin: native` have no 1.1.6 counterpart: discovery
(`SYS.PROTOCOL_INFO`, `SYS.PAGE_LIST`, `SYS.PAGE_COMMANDS`, `SYS.COMMAND_INFO` of every command,
`SYS.LAST_RESULT` after a refused read) and the GETs no 1.1.6 read reached.

The chain back to 1.1.6 ran on 2026-11-16 on one RP2350.

1. `rp2350_1.1.6.json` was captured on firmware 1.1.6.
2. Firmware 1.1.7 beta 1, replayed through the translation layer, matched it with 0 unexplained
   differences. The 2 intended ones were `ROUTE.OUTPUT_ENABLE_CONFLICT` of output 9 in states A and B,
   which 1.1.6 answered with 00 and which is now `VC_ERR_ADDRESS`.
3. In the same session `reference_rp2350.json` was captured natively. Every translated request in it
   matched the paged reply recorded during step 2, outside the mask.

The `provenance` block of the reference holds the counts and hashes. The translation layer
(`tools/dspi_test/legacy_map.py`) was deleted afterwards, together with the 1.1.6 capture, the 1.1.6
opcode script and its opcode table. All of them are in commit edf11eb and can be recovered from it.

```bash
git show edf11eb:tools/dspi_test/golden/rp2350_1.1.6.json      # the 1.1.6 capture
git show edf11eb:tools/dspi_test/golden/script_rp2350.json     # the 1.1.6 opcode script
```

The RP2040 script was converted by the same mapping but has never run on an RP2040, and there is no
RP2040 reference. Capture one on an RP2040 before using `compare` there.

## Sequence

1. Read everything (initial state) and check preconditions.
2. Baseline: `STORE.ALL_PARAMS` with the baseline blob (the device's own state on capture, the
   reference's blob on compare), then stop and reset the transient engines (siggen, RTA, subharm solo,
   clip latch, buffer watermarks).
3. Read everything (state A).
4. Run the writes of the selected tiers, recording each result and the settled `CS.STATUS` after each
   Control Surfaces SET.
5. Read everything (state B).
6. Restore from the initial state and read everything again. Any unmasked difference is reported
   under "restore".

## Use

Run from the repo root with a Python that has pyusb. Close DSPi Console and any other host first. The
tool refuses to run while another host is polling (`--allow-other-hosts` overrides), and it refuses
firmware without the paged protocol (no `SYS.PROTOCOL_INFO`).

```bash
python -m tools.dspi_test.golden.capture compare --ref tools/dspi_test/golden/reference_rp2350.json --allow-flash --allow-reconfig
python -m tools.dspi_test.golden.capture capture --out tools/dspi_test/golden/reference_rp2350.json --allow-flash --allow-reconfig
python -m tools.dspi_test.golden.capture capture --selfcheck --allow-flash --allow-reconfig
```

`compare` must use the same tiers as the reference. It exits 1 on any unmasked difference and prints
each one as section, request, byte offset, expected and got. It refuses a reference made from a
different script file (`--force` overrides). `--expect-build` refuses a device whose build string does
not start with the given text. `capture --provenance FILE` stores a JSON object in the reference.

`--selfcheck` runs the whole sequence twice and lists every byte that changed between the runs, marking
any that the mask does not cover.

The tool does not need a freshly booted device. The two bytes that depend on what ran earlier in the
boot are masked: the last-touched slot in `CS.STATUS` everywhere, and the last Control Surfaces result
(`CS.STATUS` byte 0) in state A and in the restore check, before the script's own writes define it.
The siggen history bytes of `MEAS.SIGGEN_STATUS` are masked too.

## Tiers

- **default**: live DSP state only. No flash, no pins. Restored by the bulk SET.
- **flash** (`--allow-flash`): Control Surfaces previews, device and source names, a preset name on
  slot 9, startup, output-config and master volume modes, `ROUTE.MASTER_VOLUME_SAVE`. Restored by
  writing the old values back, then `CS.SAVE`.
- **reconfig** (`--allow-reconfig`, RP2350): input-side and storage-only IO settings that touch no
  driven pin while USB is the source: I2S input rate, I2S and ADAT clock modes, ADAT input, optional
  S/PDIF input enable and pins, I2S RX pins and channel count, BCK pins, clock-pin mode, MCK multiplier
  and pin, ADAT output pin. Needs USB as source, no I2S slot, and output-config mode 1, because the
  bulk SET is the backstop for pins the firmware will not re-set one at a time.

## Excluded, and why

- `SYS.ENTER_BOOTLOADER`, `STORE.FACTORY_RESET`: destructive.
- `XPORT.UART_CONFIG`, `XPORT.I2C_CONFIG` SETs, `OUTPUT.DAC_MUTE_CONFIG` SET, `OUTPUT.DAC_MUTE_TEST`: can
  cut a control link or pulse a board pin.
- `OUTPUT.PIN`, `OUTPUT.TYPE`, `CLOCK.MCK_ENABLE` and `OUTPUT.ADAT_TX_ENABLE` SETs, and input source
  switches away from USB: they drive pins or restart the outputs.
- `OUTPUT.LATENCY` SET: a directory flash write plus an output restart.
- `STORE.ALL_PARAMS` and `STORE.ALL_PARAMS_CHUNK` as scripted SETs: the bulk SET is the restore
  mechanism.
- `STORE.PRESET_SAVE`, `STORE.PRESET_LOAD`, `STORE.PRESET_DELETE`: they move the last active slot or
  overwrite a preset, which no command can put back.
- `STORE.OUTPUT_CONFIG_SAVE`, `CS.SAVE` as scripted SETs: no readback shows the stored block.
- `CS.REVERT`: it leaves every stored non-button binding down with `PIN_IN_USE`, because
  `control_surfaces_owns_pin()` counts a slot's own stored pins as taken while the revert runs.
- `CS.AUX_STATE`, `CS.AUX_LEVEL` SETs: they need a live aux output binding, which drives a GPIO.
- `CS.IR_LEARN_ARM`: it leaves the live receiver in learn mode. The script only cancels.
- `MEAS.SIGGEN_START`: it would play a sweep on the outputs.
- The ten `FX.UPMIX_*` parameter SETs that the 1.1.6 script never used: `FX.UPMIX_CONFIG` already
  moves every field.
- `STORE.ALL_PARAMS_CHUNK` reads past offset 0: any other host's request tears the chunk session down.
- `DIAG` commands other than the two reads the 1.1.6 script made: bench builds only. A release build
  answers `VC_ERR_UNKNOWN`.

The siggen config cannot be put back to its boot value (all zero), because `MEAS.SIGGEN_CONFIG`
refuses it. After the first run following a boot it is left at a 1 kHz, -20 dBFS sine, staged and never
started. The restore report says so.

## When to recapture

Recapture with `capture --out` when the firmware changes a reply on purpose, when a command is added
to the script, or when a device-global setting the script does not write changes on the bench device
(UART config, stored Control Surfaces bindings, presets, pins). `SYS.PAGE_LIST` and
`SYS.COMMAND_INFO` are in the script, so adding or changing any command shows up as a difference.
Before recapturing, run `compare` and check that every difference is one you meant. Then record what
you checked with `--provenance`.
