# Firmware Update via USB

## Overview

The DSPi firmware exposes a vendor command that reboots the device into the RP2040/RP2350 USB mass storage bootloader (BOOTSEL mode). Once in bootloader mode, the device appears as a USB drive. Dragging a `.uf2` firmware file onto this drive writes it to flash and reboots into the new firmware.

This allows firmware updates without physical access to the BOOTSEL button on the board.

## Vendor Command

### REQ_ENTER_BOOTLOADER (0xF0)

Reboots the device into UF2 bootloader mode.

| Field | Value |
|-------|-------|
| bmRequestType | `0xC1` (device-to-host, vendor, interface) |
| bRequest | `0xF0` |
| wValue | `0x0000` (unused) |
| wIndex | `0x0002` (vendor interface number) |
| wLength | `1` |
| Response | 1-byte success (`0x01`), then device disconnects |

**Behavior:**
1. Device sends the 1-byte ACK response
2. Device waits ~100ms for the USB response to reach the host
3. Device reboots into the ROM USB bootloader
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
        dev.ctrl_transfer(0xC1, 0xF0, 0, VENDOR_INTERFACE, 1)
    except usb.core.USBError:
        # Expected — device disconnects after responding
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
        bmRequestType: 0xC1,
        bRequest: 0xF0,
        wValue: 0,
        wIndex: 2,       // vendor interface
        wLength: 1
    )

    // Send the command — expect the device to disconnect
    do {
        let response = try device.sendControlRequest(request)
        // response[0] == 0x01 means success
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

    // bmRequestType 0xC1 = device-to-host, vendor, interface
    device.controlTransfer(0xC1, 0xF0, 0, VENDOR_INTERFACE, 1, (err, data) => {
        if (err) {
            // Expected — device disconnects
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
    uint8_t buf[1];
    int ret = libusb_control_transfer(
        handle,
        0xC1,           // bmRequestType: device-to-host, vendor, interface
        0xF0,           // bRequest: REQ_ENTER_BOOTLOADER
        0x0000,         // wValue: unused
        VENDOR_INTERFACE,
        buf,
        1,              // wLength
        1000            // timeout_ms
    );

    // ret may be negative (device disconnected) — that's expected
    if (ret == 1 && buf[0] == 0x01) {
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
        dev.ctrl_transfer(0xC1, 0xF0, 0, VENDOR_INTERFACE, 1)
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
- **Return 1 byte (`0x01`)**: Command accepted, device will reboot in ~100ms
- **Throw a USB error / return negative**: The device disconnected before the response completed — this is normal and expected. Treat it as success.
- **Timeout**: The device didn't respond at all — something is wrong. The device may be in a bad state. A physical power cycle or BOOTSEL button press may be needed.

### No confirmation prompt

The firmware reboots immediately upon receiving this command. There is no "are you sure?" mechanism. The application should implement its own confirmation dialog before sending the command.

### Safety

This command cannot brick the device. The RP2040/RP2350 ROM bootloader is in permanent ROM — it cannot be overwritten. Even if a bad firmware is flashed, holding the physical BOOTSEL button during power-on will always enter bootloader mode, allowing recovery.

## Firmware Implementation

**Request ID:** `REQ_ENTER_BOOTLOADER` (`0xF0`) in `firmware/DSPi/config.h`

**Handler:** `firmware/DSPi/usb_audio.c` vendor request handler

**Mechanism:** Calls `reset_usb_boot(0, 0)` from `pico/bootrom.h`. The first argument is a GPIO pin mask for activity LED (0 = none). The second argument is a disable mask for bootloader interfaces (0 = enable all).
