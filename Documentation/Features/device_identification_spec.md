# Device Identification Specification

*Last updated: 2026-11-16 (commands on the paged protocol of firmware 1.1.7)*

## Overview

DSPi exposes three vendor commands for device identification. All three are GETs on the `SYS` page of the paged protocol (see `../commands.md`).

| Command | Page | Cmd | Dir | Address (`wValue`) | Size | Returns |
|---|---|---|---|---|---|---|
| `SYS.SERIAL` | 0x01 | 0x21 | GET | none (0) | GET 16 | The board's unique flash ID as a 16-byte ASCII hex string |
| `SYS.PLATFORM` | 0x01 | 0x20 | GET | none (0) | GET 7 | Platform type, firmware version, and output count |
| `SYS.BUILD_INFO` | 0x01 | 0x22 | GET | none (0) | GET 64 | The git describe stamp and build date of the running firmware (provenance for humans, never compared by software) |

Each request uses `bmRequestType` 0xC1, `bRequest` = the command number and `wIndex` 0x0102 (page 0x01 in the high byte, vendor interface 2 in the low byte). A non-zero `wValue` STALLs with `VC_ERR_ADDRESS`. Firmware 1.1.6 and older answered the same bytes to opcodes 0x7E, 0x7F and 0x80 with `wIndex` 2.

These complement the standard USB `iSerialNumber` string descriptor (already populated from the same flash unique ID at boot) by making identification data available over the vendor control interface to already-connected software.

## USB Enumeration Context

Each DSPi device already exposes a unique serial number via the standard USB `iSerialNumber` string descriptor. This is set at boot from `pico_get_unique_board_id_string()` (see `main.c`). Operating systems use VID + PID + serial to distinguish multiple identical devices.

The vendor commands below provide the same serial plus additional metadata (platform, firmware version, output count) to control software that communicates over the vendor interface.

---

## `SYS.SERIAL`

**Direction:** Device to Host (GET), command 0x21
**wValue:** 0
**wIndex:** `0x0102`
**wLength:** 16

### Response (16 bytes)

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 16 | `serial` | ASCII hex unique board ID (null-padded if shorter) |

The serial string is the same value as the USB `iSerialNumber` descriptor — the Pico's flash unique ID formatted as lowercase hex (e.g., `"E46058388B1A2E2C"`).

### Example

```
Request:  bmRequestType=0xC1, bRequest=0x21, wValue=0, wIndex=0x0102, wLength=16
Response: 45 34 36 30 35 38 33 38 38 42 31 41 32 45 32 43
          "E46058388B1A2E2C"
```

---

## `SYS.PLATFORM`

**Direction:** Device to Host (GET), command 0x20
**wValue:** 0
**wIndex:** `0x0102`
**wLength:** 7 (the transfer clamps to wLength, so a shorter request gets the leading bytes)

### Response (7 bytes)

| Offset | Size | Field | Values |
|--------|------|-------|--------|
| 0 | 1 | `platform` | `0` = RP2040, `1` = RP2350 |
| 1 | 1 | `fw_major` | Major version (e.g., `0x01` for v1.x.x) |
| 2 | 1 | `fw_minor_patch_legacy` | `(minor << 4) \| patch`, each nibble caps at 15 |
| 3 | 1 | `num_outputs` | Number of output channels (compile-time `NUM_OUTPUT_CHANNELS`) |
| 4 | 1 | `fw_minor` | Minor version, full-width |
| 5 | 1 | `fw_patch` | Patch version, full-width |
| 6 | 1 | `fw_beta` | `0` = final release, `1`..`255` = beta N of this patch |

### Firmware Version Decoding

Request 7 bytes. Paged firmware always answers all 7. The fallback below matters only for the legacy opcode 0x7F on firmware 1.1.6 and older. If at least 6 arrive, use the full-width bytes; otherwise fall back to the legacy nibbles (correct only while minor and patch are both <= 15). A reply shorter than 7 bytes comes from firmware that predates the beta ordinal, which is by definition a final release, so the missing byte decodes as 0.

```
major = byte[1]
minor = (len >= 6) ? byte[4] : byte[2] >> 4
patch = (len >= 6) ? byte[5] : byte[2] & 0x0F
beta  = (len >= 7) ? byte[6] : 0
```

A beta sorts below the final release of the same patch, so the comparison key is `(major, minor, patch, beta == 0 ? 256 : beta)`.

See `firmware_versioning_spec.md` for the versioning policy, the nibble cap, and the full compatibility matrix.

### Platform Capability Matrix

| Feature | RP2040 (`platform=0`) | RP2350 (`platform=1`) |
|---------|----------------------|----------------------|
| Output channels | 9 | 9 |
| EQ precision | Fixed-point (32-bit) | Float + DCP double |
| Core 1 EQ worker | No | Yes |
| Max delay | 85ms (4096 samples) | 170ms (8192 samples) |

### Example

```
Request:  bmRequestType=0xC1, bRequest=0x20, wValue=0, wIndex=0x0102, wLength=7
Response: 01 01 17 09 01 07 01
          platform=RP2350, fw=v1.1.7 beta 1, outputs=9
```

---

## `SYS.BUILD_INFO`

**Direction:** Device to Host (GET), command 0x22
**wValue:** 0
**wIndex:** `0x0102`
**wLength:** 64

### Response (64 bytes)

| Offset | Size | Field | Content |
|--------|------|-------|---------|
| 0 | 48 | `git_describe` | Output of `git describe --always --dirty` at build time, e.g. `v1.1.7-3-g1a2b3c4-dirty`. ASCII, NUL-padded. Clamped from the front if longer than 47 chars (the hash and dirty flag at the end always survive) |
| 48 | 12 | `build_date` | `YYYY-MM-DD`, ASCII, NUL-padded |
| 60 | 4 | reserved | Zero |

The stamp is regenerated on every build by `scripts/gen_build_info.cmake`, so it always reflects the checkout the binary was actually compiled from. A `-dirty` suffix means the working tree had uncommitted changes; treat the hash as approximate in that case. Every paged firmware has this command. Older firmware without opcode 0x80 STALLs it, which hosts must treat as "provenance unavailable".

The version from `SYS.PLATFORM` is the compatibility contract; this string is provenance only. **No software may gate, compare, or parse it.** See `firmware_versioning_spec.md`, "Build provenance".

---

## Application Patterns

### Multi-Device Discovery

```c
// Pseudocode: enumerate all DSPi devices and identify each
for each usb_device with VID/PID matching DSPi:
    // Serial is available at enumeration via iSerialNumber
    serial = device.serial_number  // from USB descriptor

    // After opening the vendor interface, probe for the paged protocol
    if vendor_get(SYS.PROTOCOL_INFO, length=16) starts with "DSPi":
        platform_info = vendor_get(SYS.PLATFORM, length=7)
    else:                                  # 1.1.6 or older: legacy opcode 0x7F, wIndex 2
        platform_info = vendor_get_legacy(0x7F, length=7)
    platform  = platform_info[0]
    fw_major  = platform_info[1]
    n_outputs = platform_info[3]
    if len(platform_info) >= 6:            # full-width fields
        fw_minor = platform_info[4]
        fw_patch = platform_info[5]
    else:                                  # very old firmware: legacy nibbles
        fw_minor = platform_info[2] >> 4
        fw_patch = platform_info[2] & 0x0F

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

#define SYS_PAGE         0x01
#define SYS_PLATFORM     0x20
#define SYS_SERIAL       0x21
#define VENDOR_INTF      2
#define SYS_WINDEX       ((SYS_PAGE << 8) | VENDOR_INTF)

// GET SYS.SERIAL
uint8_t serial[16];
int ret = libusb_control_transfer(handle,
    LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
    SYS_SERIAL,
    0,              // wValue: no address
    SYS_WINDEX,     // wIndex: page in the high byte
    serial, 16,
    1000);          // timeout ms
printf("Serial: %.16s\n", serial);

// GET SYS.PLATFORM (paged firmware always returns 7 bytes)
uint8_t info[7];
ret = libusb_control_transfer(handle,
    LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
    SYS_PLATFORM,
    0, SYS_WINDEX,
    info, 7, 1000);
printf("Platform: %s, FW: v%d.%d.%d beta %d, Outputs: %d\n",
       info[0] ? "RP2350" : "RP2040",
       info[1], info[4], info[5], info[6],
       info[3]);
```

---

## Backward Compatibility

- Firmware 1.1.7 answers identification only through the `SYS` page. It refuses the 1.1.6 opcodes 0x7E, 0x7F and 0x80 on page 0, so a host written for 1.1.6 cannot identify a 1.1.7 device.
- A current host probes `SYS.PROTOCOL_INFO` first (`vendor_command_overhaul_plan.md` section 5.14). A STALL means firmware 1.1.6 or older, which still answers its own opcodes 0x7E, 0x7F and 0x80 with `wIndex` 2. Firmware older than those opcodes STALLs them too. Control software should handle the STALL or timeout gracefully and treat it as "identification unavailable."
- The existing `iSerialNumber` USB descriptor is unchanged and remains the primary device selection mechanism during enumeration.

## Old to new map (for reference)

| 1.1.6 opcode | Command in 1.1.7 |
|------|---------|
| 0x7A | `SYS.CORE1_MODE` |
| 0x7B | `ROUTE.OUTPUT_ENABLE_CONFLICT` |
| 0x7C, 0x7D | `OUTPUT.PIN` |
| 0x7E | `SYS.SERIAL` |
| 0x7F | `SYS.PLATFORM` |
| 0x80 | `SYS.BUILD_INFO` |
