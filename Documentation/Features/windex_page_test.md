# wIndex Page Test (Windows)

*Last updated: 2026-11-16*

## TL;DR

Firmware 1.1.7 puts a page number in the high byte of `wIndex` on every vendor request. This test proves that Windows passes that byte to the DSPi unchanged. It uses the normal 1.1.7 firmware, not a special build. The device keeps a record of the last request it received (`SYS.LAST_RESULT`), so the script sends requests to several pages and then asks the device which page it saw.

It takes about five minutes. It is the Windows gate in section 11.1 of `vendor_command_overhaul_plan.md`.

## What the test sends

Every test request is harmless. It uses command number 0xEE, which no page defines in any build, so the device refuses it and changes nothing.

For each recipient the script does this:

1. Reads `SYS.PROTOCOL_INFO` (page 0x01, command 0x10). The reply must start with `DSPi`.
2. Sends the same read to page 0x00. It must be refused. That shows the device really tells pages apart.
3. For each of the pages 0x02, 0x55, 0x7F, 0xAA, 0xF1 and 0xFF, sends three requests to command 0xEE: a read, an empty write and a 4-byte write. Each one has its own `wValue`. Each one is refused on purpose.
4. After each request, reads `SYS.LAST_RESULT` and checks that the page, command, `wValue` and direction match what was sent.

It runs this twice. First with interface recipient requests (`0xC1` / `0x41`), which is what the protocol uses. Then with device recipient requests (`0xC0` / `0x40`), which is the fallback if Windows ever changes the byte.

The script refuses to run on firmware 1.1.6 or older. That firmware ignores the page byte and would treat the test requests as old commands. On such a board it prints the firmware version and stops without sending anything else.

## What you need

- A Windows 10 or 11 PC.
- A DSPi and a USB cable. Either board works.
- Firmware 1.1.7 beta 1 or later on the DSPi. From this repo:
  - RP2350: `build-rp2350/DSPi/DSPi.uf2`
  - RP2040: `build-rp2040/DSPi/DSPi.uf2`
- The script `tools/windex_page_test.py`. It is a single file and needs nothing else from the repo.
- Python 3.9 or newer.

## Steps

1. **Flash the firmware.** Hold BOOTSEL, plug the board in, and copy the UF2 onto the drive that appears. The board reboots as a DSPi.
2. **Close anything that talks to the DSPi.** Quit DSPi Console and any other tool that might hold the device. Only one program can read `SYS.LAST_RESULT` meaningfully at a time.
3. **Install the two Python packages.** In a terminal:

   ```
   pip install pyusb libusb-package
   ```

4. **Run the script** from the folder that holds it.

   ```
   python windex_page_test.py
   ```

5. **Send back the whole output.** Copy everything the script prints, including the rows above the summary.

The exit code is 0 when the interface recipient line passes, and 1 otherwise.

## Reading the result

The last four lines are the summary.

```
RESULT
  interface recipient, page byte intact : PASS
  interface recipient, all fields exact : yes
  device recipient, page byte intact    : PASS
  device recipient, all fields exact    : yes
```

| Line | Meaning |
|---|---|
| interface recipient, page byte intact | The one that matters. PASS means the device saw every page exactly as sent. The protocol goes ahead as designed |
| interface recipient, all fields exact | "yes" means the command number, direction and result code also matched. "no" with a PASS above it is worth reporting but does not block |
| device recipient, page byte intact | The fallback addressing. It only matters if the first line fails |
| device recipient, all fields exact | Informational |

Each row above the summary shows one request. It shows what was sent and what the device recorded. A FAIL row shows the page the device actually saw, which tells us what Windows did to the byte. If the first summary line says FAIL, send everything back before any host is ported.

## Troubleshooting

| Message | Cause | Fix |
|---|---|---|
| `No DSPi found` | Board not enumerated, or held by another app | Replug. Quit DSPi Console |
| `This DSPi runs firmware 1.1.6` (or another old version) | The board still has old firmware | Repeat step 1 with a 1.1.7 UF2 |
| `FAIL: the DSPi did not answer the protocol probe on either recipient` | Either the board runs firmware the script does not recognise, or Windows changed the page byte on every request | Check which firmware is flashed. If it is 1.1.7 beta 1 or later, this is a real failure: send the output back |
| An access, "not supported" or "entity not found" error | The DSPi vendor interface is not bound to WinUSB | Open Device Manager and check that the DSPi vendor interface shows under "Universal Serial Bus devices". The firmware asks Windows to bind WinUSB automatically, so a replug normally fixes it |
| `LAST_RESULT unreadable` on some rows | The device stopped answering, or another program sent requests at the same time | Close every other DSPi program and run again |
| `pyusb is missing` | Step 3 was skipped | Run the `pip install` line |

## What to send back

- The whole output of the script.
- The Windows version (the script prints it on its first line).
- Which board you used (RP2040 or RP2350).

## Afterwards

Nothing to undo. The test changes no settings, and the firmware is the normal 1.1.7 build.
