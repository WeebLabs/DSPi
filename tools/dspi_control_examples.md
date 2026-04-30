# DSPi Raspberry Pi Control Examples

These scripts run on a Raspberry Pi or similar Linux host connected to a Pico or
Pico 2 running DSPi firmware with I2C and UART control enabled.

The scripts are deliberately simple examples. They use only the Python standard
library and the Option A control frame:

```text
request:  "D" "C" 0x01 dir bRequest wValueLE wIndexLE wLengthLE payload...
response: "D" "C" 0x81 status lengthLE payload...
```

## Build DSPi Firmware

The default DSPi firmware leaves I2C and UART disabled with priority `-1`.
Build a transport-enabled image before wiring the Raspberry Pi:

```bash
cmake -S firmware -B firmware/build-rp2350-control -DPICO_BOARD=pico2 \
  -DDSPI_CONTROL_I2C_PRIORITY=20 \
  -DDSPI_CONTROL_UART_PRIORITY=10
cmake --build firmware/build-rp2350-control --config Release
```

Use `-DPICO_BOARD=pico` for RP2040. Override the pin/address definitions only
if your hardware needs different wiring:

```bash
-DDSPI_CONTROL_I2C_ADDR=0x2A
-DDSPI_CONTROL_I2C_SDA_PIN=4
-DDSPI_CONTROL_I2C_SCL_PIN=5
-DDSPI_CONTROL_UART_TX_PIN=12
-DDSPI_CONTROL_UART_RX_PIN=13
-DDSPI_CONTROL_UART_BAUD=1000000
```

## I2C Wiring

Default DSPi I2C settings are address `0x2A`, SDA GPIO 4, SCL GPIO 5, 400 kHz.

```text
Raspberry Pi GPIO2/SDA1 pin 3  -> DSPi GPIO4/SDA
Raspberry Pi GPIO3/SCL1 pin 5  -> DSPi GPIO5/SCL
Raspberry Pi GND               -> DSPi GND
```

Enable I2C on Raspberry Pi OS with `raspi-config` or `dtparam=i2c_arm=on`.
The I2C example writes one request frame, then reads the response header plus
the requested payload length in a single I2C read transaction.  This matters
because the DSPi target treats STOP after a partial response read as the end of
that response.

Run:

```bash
python3 tools/dspi_i2c_control_test.py --device /dev/i2c-1 --address 0x2a
```

The I2C script waits 1 ms between writing a request and reading the response by
default. Use `--turnaround-delay 0.005` to increase that pause if small scalar
responses show malformed response headers while larger reads pass.

## UART Wiring

Default DSPi UART settings are 1,000,000 baud, DSPi TX GPIO 12, DSPi RX GPIO 13.

```text
Raspberry Pi GPIO14/TXD pin 8  -> DSPi GPIO13/RX
Raspberry Pi GPIO15/RXD pin 10 -> DSPi GPIO12/TX
Raspberry Pi GND               -> DSPi GND
```

Disable the Raspberry Pi serial console on the selected UART before testing.
Choose alternate DSPi UART pins if GPIO 13 is needed for I2S MCK.

Run:

```bash
python3 tools/dspi_uart_control_test.py --device /dev/serial0 --baud 1000000
```

## Checks Performed

Each script:

- Reads `REQ_GET_PLATFORM`.
- Performs a low-risk `REQ_SET_MASTER_VOLUME` / `REQ_GET_MASTER_VOLUME` round
  trip by setting master volume to `-37.0 dB`, reading it back, and restoring
  the original value.
- Reads `REQ_GET_ALL_PARAMS` and validates the `WireBulkParams` header.
- Sends a bad frame version and expects transport status `BAD_FRAME`.

Each check prints `Testing`, `Executing`, and `Result` lines. A failed check is
recorded and the script continues to the next check where the transport remains
usable. The final summary reports the number of failed and skipped checks, and
the process exits non-zero if any check failed.

By default the master-volume write/read target is `-37.0 dB`. Override it with
`--master-volume-target` if the bench setup needs a different temporary value.
If the original value is already the requested target, the script chooses a
nearby value so the check still proves that the device changed state. The
script restores the original value after the changed-value readback.

Use 3.3 V signalling only. Do not connect 5 V to Pico GPIO.
