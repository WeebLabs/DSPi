# Firmware Update via USB

*Last updated: 2026-11-16 (host interface on the paged protocol of firmware 1.1.7)*

## Overview

The DSPi firmware exposes a vendor command that reboots the device into the RP2040/RP2350 USB mass storage bootloader (BOOTSEL mode). Once in bootloader mode, the device appears as a USB drive. Dragging a `.uf2` firmware file onto this drive writes it to flash and reboots into the new firmware.

This allows firmware updates without physical access to the BOOTSEL button on the board.

## Vendor Command

### `SYS.ENTER_BOOTLOADER`

Reboots the device into UF2 bootloader mode. It is a zero-length SET in the paged protocol (see `../commands.md`).

| Command | Page | Cmd | Dir | Address (`wValue`) | Size |
|---|---|---|---|---|---|
| `SYS.ENTER_BOOTLOADER` | 0x01 | 0x30 | SET (zero-length) | none (0) | SET 0 |

| Field | Value |
|-------|-------|
| bmRequestType | `0x41` (host-to-device, vendor, interface) |
| bRequest | `0x30` |
| wValue | `0x0000` |
| wIndex | `0x0102` (page 0x01 in the high byte, vendor interface 2 in the low byte) |
| wLength | `0` |
| Response | None. The status stage is ACKed, then the device disconnects |

**Behavior:**
1. Device ACKs the status stage of the zero-length SET
2. Device reboots as soon as the status stage completes. Over UART and I2C it reboots after the response frame has gone out
3. Device enters the ROM USB bootloader
4. The original USB device disappears from the bus
5. A new USB mass storage device appears (the RP2040/RP2350 bootloader)
6. The host can now write a `.uf2` file to the drive

**This command does not return.** The USB device disconnects. The host application must expect the device to vanish after sending this command.

## Application Patterns

### Python (pyusb)

```python
import usb.core
import time

VENDOR_ID = 0x1209     # Replace with actual DSPi VID
PRODUCT_ID = 0x0001    # Replace with actual DSPi PID
VENDOR_INTERFACE = 2

def enter_bootloader(dev):
    """Reboot DSPi into UF2 bootloader mode."""
    try:
        dev.ctrl_transfer(0x41, 0x30, 0, (0x01 << 8) | VENDOR_INTERFACE, b"")
    except usb.core.USBError:
        # Expected if the device disconnects before the call returns
        pass
    print("Device is rebooting into bootloader mode...")

# Usage
dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)
if dev is None:
    print("DSPi not found")
else:
    enter_bootloader(dev)
    # Wait for bootloader to appear
    time.sleep(2)
    print("Look for a new USB drive named 'RPI-RP2'")
```

### Swift (macOS, IOKit)

```swift
import IOKit.usb

func enterBootloader(device: USBDevice) {
    let request = USBDeviceRequest(
        bmRequestType: 0x41,
        bRequest: 0x30,             // SYS.ENTER_BOOTLOADER
        wValue: 0,
        wIndex: (0x01 << 8) | 2,    // page SYS, vendor interface 2
        wLength: 0
    )

    // Send the command. Expect the device to disconnect.
    do {
        try device.sendControlRequest(request)
        // No reply bytes. Success is the ACK of the status stage.
    } catch {
        // Expected: device disconnected during or after response
    }

    // The device is gone. A USB drive named "RPI-RP2" will appear shortly.
}
```

### Node.js (node-usb)

```javascript
const usb = require('usb');

const VENDOR_INTERFACE = 2;

function enterBootloader(device) {
    device.open();

    // bmRequestType 0x41 = host-to-device, vendor, interface. wIndex carries page 0x01 (SYS).
    device.controlTransfer(0x41, 0x30, 0, (0x01 << 8) | VENDOR_INTERFACE, Buffer.alloc(0), (err) => {
        if (err) {
            // Expected if the device disconnects first
            console.log('Device rebooting into bootloader...');
        } else {
            console.log('Bootloader command accepted, device will disconnect');
        }
    });
}
```

### C (libusb)

```c
#include <libusb-1.0/libusb.h>

#define VENDOR_INTERFACE 2

int enter_bootloader(libusb_device_handle *handle) {
    int ret = libusb_control_transfer(
        handle,
        0x41,           // bmRequestType: host-to-device, vendor, interface
        0x30,           // bRequest: SYS.ENTER_BOOTLOADER
        0x0000,         // wValue: no address
        (0x01 << 8) | VENDOR_INTERFACE,   // wIndex: page SYS, interface 2
        NULL,
        0,              // wLength: zero-length SET
        1000            // timeout_ms
    );

    // ret may be negative if the device disconnected first. That is expected.
    if (ret == 0) {
        printf("Bootloader command accepted\n");
    }
    // Device will disconnect regardless
    return 0;
}
```

### Complete Update Workflow

A full firmware update cycle from the application's perspective:

```python
import usb.core
import time
import shutil
import os

VENDOR_ID = 0x1209
PRODUCT_ID = 0x0001
VENDOR_INTERFACE = 2

def update_firmware(uf2_path):
    # Step 1: Find the DSPi device
    dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)
    if dev is None:
        print("ERROR: DSPi not found")
        return False

    print("DSPi found, sending bootloader command...")

    # Step 2: Send the bootloader command
    try:
        dev.ctrl_transfer(0x41, 0x30, 0, (0x01 << 8) | VENDOR_INTERFACE, b"")
    except usb.core.USBError:
        pass  # Expected disconnect

    # Step 3: Wait for the bootloader drive to appear
    # On macOS: /Volumes/RPI-RP2
    # On Linux: auto-mounted or /media/<user>/RPI-RP2
    # On Windows: a new drive letter
    print("Waiting for bootloader drive...")
    boot_drive = None
    for attempt in range(30):  # Wait up to 15 seconds
        time.sleep(0.5)
        # macOS path — adjust for your platform
        if os.path.exists("/Volumes/RPI-RP2"):
            boot_drive = "/Volumes/RPI-RP2"
            break

    if boot_drive is None:
        print("ERROR: Bootloader drive did not appear")
        return False

    print(f"Bootloader drive found at {boot_drive}")

    # Step 4: Copy the UF2 file
    dest = os.path.join(boot_drive, os.path.basename(uf2_path))
    print(f"Writing {uf2_path} to {dest}...")
    shutil.copy2(uf2_path, dest)

    # Step 5: Wait for the device to reboot with new firmware
    print("Firmware written. Device will reboot automatically.")
    time.sleep(3)

    # Step 6: Verify the device re-enumerates
    dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)
    if dev is not None:
        print("SUCCESS: DSPi re-enumerated with new firmware")
        return True
    else:
        print("WARNING: DSPi not found after update — may need manual power cycle")
        return False

# Usage
update_firmware("/path/to/DSPi.uf2")
```

## Important Notes

### Audio interruption

Sending this command immediately stops all audio output. There is no graceful shutdown — the device reboots. If your application is playing audio, stop playback before sending the command to avoid a pop/click on the output.

### Device identity change

After entering bootloader mode, the device's USB VID/PID changes to the RP2040/RP2350 bootloader's identity (`2E8A:0003` for RP2040, `2E8A:000F` for RP2350). Your application should look for the bootloader by its VID/PID or by the drive label "RPI-RP2".

### Error handling

The `ctrl_transfer` call may:
- **Return 0 (ACK)**: Command accepted. The device reboots as soon as the status stage completes.
- **Throw a USB error / return negative**: The device disconnected before the call returned. This is normal and expected. Treat it as success.
- **STALL**: The request was refused, for example with a non-zero `wValue` or a non-empty payload. `SYS.LAST_RESULT` gives the reason.
- **Timeout**: The device didn't respond at all. Something is wrong. The device may be in a bad state. A physical power cycle or BOOTSEL button press may be needed.

### No confirmation prompt

The firmware reboots immediately upon receiving this command. There is no "are you sure?" mechanism. The application should implement its own confirmation dialog before sending the command.

### Safety

This command cannot brick the device. The RP2040/RP2350 ROM bootloader is in permanent ROM — it cannot be overwritten. Even if a bad firmware is flashed, holding the physical BOOTSEL button during power-on will always enter bootloader mode, allowing recovery.

## Firmware Implementation

**Command:** `SYS.ENTER_BOOTLOADER` (page 0x01, command 0x30) in `firmware/DSPi/vendor/pages/sys.def`. Firmware 1.1.6 and older used opcode 0xF0 as an IN request instead.

**Handler:** `sys_enter_bootloader_set()` in `firmware/DSPi/vendor/vc_page_sys.c`. On USB it arms the reboot and `firmware/DSPi/vendor/vc_usb.c` performs it after the status stage.

**Mechanism:** Calls `reset_usb_boot(0, 0)` from `pico/bootrom.h`. The first argument is a GPIO pin mask for activity LED (0 = none). The second argument is a disable mask for bootloader interfaces (0 = enable all).
