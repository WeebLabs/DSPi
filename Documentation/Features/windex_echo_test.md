# wIndex Echo Test (Windows)

*Last updated: 2026-11-15*

## What this checks

The new vendor protocol puts a page number in the high byte of `wIndex`. This test proves that Windows delivers that byte to the device unchanged. It is the gate for Phase 1 of `vendor_command_overhaul_plan.md`.

It takes about five minutes.

## What you need

- A Windows 10 or 11 PC.
- A DSPi and a USB cable. Either board works.
- The bench firmware for that board:
  - RP2350: `build-rp2350-windexecho/DSPi/DSPi.uf2`
  - RP2040: `build-rp2040-windexecho/DSPi/DSPi.uf2`
- The script `tools/windex_echo_test.py`.
- Python 3.9 or newer on the PC.

The bench firmware is firmware 1.1.6 plus one extra request (0x4F) that reports back what it received. Everything else behaves as normal, so the Console still works with it.

## Steps

1. **Flash the bench firmware.** Hold BOOTSEL, plug the board in, and copy the UF2 onto the drive that appears. The board reboots as a DSPi.
2. **Close anything that talks to the DSPi.** Quit DSPi Console and any other tool that holds the device.
3. **Install the two Python packages.** In a terminal:

   ```
   pip install pyusb libusb-package
   ```

4. **Run the script.**

   ```
   python windex_echo_test.py
   ```

5. **Send back the whole output.** Copy everything the script prints.

## Reading the result

The last four lines are the summary.

```
RESULT
  interface recipient, high byte intact : PASS
  interface recipient, all fields exact : yes
  device recipient, high byte intact    : PASS
  device recipient, all fields exact    : yes
```

| Line | Meaning |
|---|---|
| interface recipient, high byte intact | The one that matters. PASS means the plan goes ahead as written |
| interface recipient, all fields exact | "no" is acceptable. Windows may rewrite the low byte with the interface number, which the plan never relies on |
| device recipient, high byte intact | The fallback addressing. Only matters if the first line fails |
| device recipient, all fields exact | Informational |

If the first line says FAIL, the per-row output above it shows what the device saw for each request. Send it all back and the plan is revised before any more work is done.

## If the script cannot talk to the device

| Message | Cause | Fix |
|---|---|---|
| `No DSPi found` | Board not enumerated, or held by another app | Replug. Quit DSPi Console |
| `Echo request failed` | The normal firmware is flashed, not the bench build | Repeat step 1 with the `windexecho` UF2 |
| `Echo request failed` with an access or "not supported" error | Interface 2 is not bound to WinUSB | Open Device Manager and check that the DSPi vendor interface shows under "Universal Serial Bus devices". The firmware asks Windows to bind WinUSB automatically, so a replug normally fixes it |

## Optional extra check

If the Windows Console is to hand, the same request can be sent through its own WinUSB call path: `bmRequestType` 0xC1, `bRequest` 0x4F, `wValue` 0xA55A, `wIndex` 0x7F02, length 16. Bytes 4 and 5 of the reply are the `wIndex` the device saw, low byte first. Byte 5 should read 0x7F.

## Afterwards

Flash the normal firmware again when you are done. The bench build is not a release.
