# dspi_poke

Send one vendor request to a connected DSPi and see the reply.

It speaks the paged vendor protocol of firmware 1.1.7 and later. Use it to try a command that has no Console control yet, or to see exactly why the device refuses a request.

## Install

```
pip install pyusb
# macOS:
brew install libusb
```

## How it names commands

A command is written `PAGE.NAME`, for example `ROUTE.OUTPUT_GAIN`. Names are not case-sensitive. A name alone works when only one page has it. You can also give the page and command as numbers (`5.0x21`, `route.0x21`), the 16-bit ID (`0x0521`), or a command byte with `--page`.

Names, directions, address kinds and sizes come from `Documentation/protocol/dspi_commands.json`. The script finds it by walking up from its own folder or the working directory. Pass `--registry PATH` when running it from elsewhere. Numeric forms work without the file.

`--addr` sets `wValue`, the address (which output, channel, band or slot). `wIndex` comes from the page. `--windex` overrides it for experiments.

## Safety check

The tool first sends GET `SYS.PROTOCOL_INFO`. Firmware 1.1.6 and older does not answer it, and the tool stops. That firmware ignores the page and would run whatever old opcode shares the command number, factory reset included. `--allow-old-firmware` sends the request anyway.

## Examples

List commands, optionally filtered by a piece of the name:

```
$ ./dspi_poke.py list route.output
0x05 ROUTE  rev 1  Matrix, per-output controls, delays, channel names, preamp, master and user volume
  0x0520  ROUTE.OUTPUT_ENABLE           GS  OUTPUT        get 1    set 1     Output enable; ...
  0x0521  ROUTE.OUTPUT_GAIN             GS  OUTPUT        get 4    set 4     Output gain, f32 dB
  ...
```

Identity, the protocol version and the pages on the device:

```
$ ./dspi_poke.py info
protocol 1.0, 12 page(s), flags 0x00, largest SET 64 B, largest GET 256 B
firmware v1.1.7-beta1 on RP2350, 9 outputs, build v1.1.6-10-g518298a-dirty 2026-11-16
pages on the device:
  0x01  SYS     rev 1   Discovery, identity, device name, last result, bootloader, ...
  ...
  0xF0  DIAG    absent on this build (registry rev 1)

$ ./dspi_poke.py get SYS.PLATFORM
GET SYS.PLATFORM (0x0120)  wValue=0x0000  wIndex=0x0102  length=7
  received 7 byte(s):
  [0000]  01 01 17 09 01 07 01                             .......
```

Read a float at an address, then write the same value back:

```
$ ./dspi_poke.py get ROUTE.OUTPUT_GAIN --addr 0 --as float
GET ROUTE.OUTPUT_GAIN (0x0521)  wValue=0x0000 (OUTPUT)  wIndex=0x0502  length=4
  received 4 byte(s):
  0

$ ./dspi_poke.py set ROUTE.OUTPUT_GAIN --addr 0 --float 0
SET ROUTE.OUTPUT_GAIN (0x0521)  wValue=0x0000 (OUTPUT)  wIndex=0x0502  payload=4 byte(s)
  [0000]  00 00 00 00                                      ....
  device accepted 4 byte(s).
```

A zero-length action:

```
$ ./dspi_poke.py set MEAS.CLIP_CLEAR
SET MEAS.CLIP_CLEAR (0x0912)  wValue=0x0000  wIndex=0x0902  payload=0 byte(s)
  device accepted 0 byte(s).
```

A refused request. The tool reads `SYS.LAST_RESULT` and prints the reason:

```
$ ./dspi_poke.py get ROUTE.OUTPUT_GAIN --addr 99
GET ROUTE.OUTPUT_GAIN (0x0521)  wValue=0x0063 (OUTPUT)  wIndex=0x0502  length=4
  STALL: VC_ERR_ADDRESS (0x13): wValue out of range
```

A request the registry already rules out never reaches the device:

```
$ ./dspi_poke.py get MEAS.CLIP_CLEAR
MEAS.CLIP_CLEAR has no GET handler (it is SET only).  Use `set`, or --no-check to send the GET anyway.
```

`--no-check` also skips the SET length check, so you can send a deliberately wrong length and watch the device return `VC_ERR_LENGTH`.

## Discovery

- `info PAGE` reads `SYS.PAGE_COMMANDS` and lists what the device implements on that page. It marks commands that are absent on this build or missing from the registry.
- `info CMD` reads `SYS.COMMAND_INFO` and shows the directions, address kind, sizes and flags the device reports. It notes any difference from the registry.

Run `./dspi_poke.py help` for the full guide, including every payload and reply format.
