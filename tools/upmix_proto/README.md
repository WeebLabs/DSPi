# Upmix surround leak prototype

Offline float32 model of `firmware/DSPi/upmix.c` (adaptive surround) plus a
switchable proposed engine, measured stem-through on synthetic test material.

- `upmix_model.py`: `baseline_run()` is the port of `upmix_process_block()`.
  `engine()` is the proposed engine. With every flag off it is bit-exact to the port.
- `signals.py`: deterministic test material (seeded, nothing downloaded).
- `run_eval.py`: runs every configuration, writes `out/results.csv` and the WAVs,
  and prints the summary table.

Needs numpy, scipy and numba in one interpreter. Neither of the stock
interpreters on this machine has all three, so use a scratch venv:

    /opt/homebrew/bin/python3.14 -m venv /tmp/upmix_venv
    /tmp/upmix_venv/bin/pip install numpy scipy numba
    cd tools/upmix_proto && /tmp/upmix_venv/bin/python run_eval.py --sweep

`--sweep` adds the parameter sensitivity rows. `--nowav` skips the audio files.
WAVs are 5 channels in the order L', R', C, Ls, Rs at 48 kHz float32.
`*_in.wav` is the stereo input.

Summary columns are dB. `leak` is a direct stem's level in the louder surround,
relative to its louder front channel through the same 300 Hz to 7 kHz band
limit. `ret` is ambience surround power relative to baseline. `n*` is the leak
change net of the ambience change, so a plain gain cut scores 0. `pStd` and
`pRate` are the std and 10 ms step RMS of the effective surround gain.

## Firmware port check

`harness/harness.py` compiles the working-tree `firmware/DSPi/upmix.c` and the
`HEAD` version natively against the stubs in `harness/stubs`. It checks that
non-adaptive paths are bit-identical to `HEAD`, measures Ls/Rs latency with an
impulse, compares the C sample by sample with `upmix_model.port_engine()`, and
measures the `run_side.py` metrics from the compiled C, stem-through. It writes
`out/harness_report.txt` and `out/harness_metrics.csv`.

    /tmp/upmix_venv/bin/python harness/harness.py

`harness/split_check.py` needs only numpy. It checks that `upmix_begin()` /
`upmix_render()` reproduce the committed engine bit for bit at chunk sizes 1, 7
and 32, across every mode, mode switches and packet sizes.

    python3 harness/split_check.py [rev]
