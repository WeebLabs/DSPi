# Device Identification Specification

## Overview

DSPi exposes two vendor USB control transfer commands for device identification:

- **`REQ_GET_SERIAL` (0x7E)** — Returns the board's unique flash ID as a 16-byte ASCII hex string
- **`REQ_GET_PLATFORM` (0x7F)** — Returns platform type, firmware version, and output count

These complement the standard USB `iSerialNumber` string descriptor (already populated from the same flash unique ID at boot) by making identification data available over the vendor control interface to already-connected software.

## USB Enumeration Context

Each DSPi device already exposes a unique serial number via the standard USB `iSerialNumber` string descriptor. This is set at boot from `pico_get_unique_board_id_string()` (see `main.c`). Operating systems use VID + PID + serial to distinguish multiple identical devices.

The vendor commands below provide the same serial plus additional metadata (platform, firmware version, output count) to control software that communicates over the vendor interface.

---

## REQ_GET_SERIAL (0x7E)

**Direction:** Device → Host (GET)
**wValue:** 0 (unused)
**wIndex:** Vendor interface number
**wLength:** 16

### Response (16 bytes)

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 16 | `serial` | ASCII hex unique board ID (null-padded if shorter) |

The serial string is the same value as the USB `iSerialNumber` descriptor — the Pico's flash unique ID formatted as lowercase hex (e.g., `"E46058388B1A2E2C"`).

### Example

```
Request:  bmRequestType=0xC1, bRequest=0x7E, wValue=0, wIndex=2, wLength=16
Response: 45 34 36 30 35 38 33 38 38 42 31 41 32 45 32 43
          "E46058388B1A2E2C"
```

---

## REQ_GET_PLATFORM (0x7F)

**Direction:** Device → Host (GET)
**wValue:** 0 (unused)
**wIndex:** Vendor interface number
**wLength:** 4

### Response (4 bytes)

| Offset | Size | Field | Values |
|--------|------|-------|--------|
| 0 | 1 | `platform` | `0` = RP2040, `1` = RP2350 |
| 1 | 1 | `fw_version_hi` | BCD major version (e.g., `0x01` for v1.x.x) |
| 2 | 1 | `fw_version_lo` | BCD minor.patch (e.g., `0x09` for v_.0.9) |
| 3 | 1 | `num_outputs` | Number of output channels (compile-time `NUM_OUTPUT_CHANNELS`) |

### Firmware Version Decoding

The firmware version is BCD-encoded in 2 bytes:

```
major = byte[1]
minor = byte[2] >> 4
patch = byte[2] & 0x0F
```

Each component ranges 0–9. Examples:

| Bytes | Version |
|-------|---------|
| `01 09` | v1.0.9 |
| `02 31` | v2.3.1 |
| `01 10` | v1.1.0 |

### Platform Capability Matrix

| Feature | RP2040 (`platform=0`) | RP2350 (`platform=1`) |
|---------|----------------------|----------------------|
| Output channels | 9 | 9 |
| EQ precision | Fixed-point (32-bit) | Float + DCP double |
| Core 1 EQ worker | No | Yes |
| Max delay | 85ms (4096 samples) | 170ms (8192 samples) |

### Example

```
Request:  bmRequestType=0xC1, bRequest=0x7F, wValue=0, wIndex=2, wLength=4
Response: 01 01 09 09
          platform=RP2350, fw=v1.0.9, outputs=9
```

---

## Application Patterns

### Multi-Device Discovery

```c
// Pseudocode: enumerate all DSPi devices and identify each
for each usb_device with VID/PID matching DSPi:
    // Serial is available at enumeration via iSerialNumber
    serial = device.serial_number  // from USB descriptor

    // After opening the vendor interface, get extended info
    platform_info = vendor_get(REQ_GET_PLATFORM, length=4)
    platform  = platform_info[0]
    fw_major  = platform_info[1]
    fw_minor  = platform_info[2] >> 4
    fw_patch  = platform_info[2] & 0x0F
    n_outputs = platform_info[3]

    printf("Device %s: %s, fw v%d.%d.%d, %d outputs\n",
           serial,
           platform == 1 ? "RP2350" : "RP2040",
           fw_major, fw_minor, fw_patch,
           n_outputs)
```

### Feature Adaptation

```c
// Adapt UI/behavior based on platform capabilities
if (platform == PLATFORM_RP2350) {
    enable_eq_worker_status()     // Core 1 EQ worker available
    set_max_delay_slider(170.0)   // 170ms max
} else {
    disable_eq_worker_status()
    set_max_delay_slider(85.0)    // 85ms max
}
```

### libusb Example (C)

```c
#include <libusb-1.0/libusb.h>

#define REQ_GET_SERIAL   0x7E
#define REQ_GET_PLATFORM 0x7F
#define VENDOR_INTF      2

// GET serial
uint8_t serial[16];
int ret = libusb_control_transfer(handle,
    LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
    REQ_GET_SERIAL,
    0,              // wValue
    VENDOR_INTF,    // wIndex
    serial, 16,
    1000);          // timeout ms
printf("Serial: %.16s\n", serial);

// GET platform info
uint8_t info[4];
ret = libusb_control_transfer(handle,
    LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
    REQ_GET_PLATFORM,
    0, VENDOR_INTF,
    info, 4, 1000);
printf("Platform: %s, FW: v%d.%d.%d, Outputs: %d\n",
       info[0] ? "RP2350" : "RP2040",
       info[1], info[2] >> 4, info[2] & 0x0F,
       info[3]);
```

---

## Backward Compatibility

- Devices running firmware without these commands will STALL on requests 0x7E/0x7F (standard USB behavior for unsupported vendor requests). Control software should handle the STALL/timeout gracefully and treat it as "identification unavailable."
- The existing `iSerialNumber` USB descriptor is unchanged and remains the primary device selection mechanism during enumeration.
- No existing vendor commands are modified.

## Request Code Map (for reference)

| Code | Command |
|------|---------|
| 0x7A | `REQ_GET_CORE1_MODE` |
| 0x7B | `REQ_GET_CORE1_CONFLICT` |
| 0x7C | `REQ_SET_OUTPUT_PIN` |
| 0x7D | `REQ_GET_OUTPUT_PIN` |
| 0x7E | `REQ_GET_SERIAL` |
| 0x7F | `REQ_GET_PLATFORM` |
