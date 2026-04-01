# USB Error Diagnostics

## Overview

The DSPi firmware tracks USB PHY-level errors detected by the RP2040/RP2350 SIE (Serial Interface Engine) hardware. These are transient electrical events — not application-layer failures — and are handled by clearing the hardware status bits and incrementing per-type diagnostic counters. No bus reset or re-enumeration occurs.

The host application can read these counters to assess USB link quality, diagnose intermittent audio glitches caused by electrical noise, and verify that a cable, hub, or port is reliable.

## Background

USB Full-Speed (12 Mbit/s) uses NRZI encoding with bit stuffing. The SIE hardware detects five classes of low-level errors:

| Error | Cause | Severity |
|-------|-------|----------|
| **CRC Error** | Corrupted packet (EMI, cable issue) | Common, benign — host retransmits |
| **Bit Stuff Error** | Invalid NRZI encoding (signal integrity) | Common, benign — host retransmits |
| **RX Timeout** | Device didn't respond in time (bus contention) | Occasional, benign |
| **RX Overflow** | Data arrived faster than SIE could process | Rare, indicates timing stress |
| **Data Sequence Error** | PID toggle out of sync (missed ACK) | Occasional, self-correcting |

All of these are recoverable. The USB protocol guarantees delivery via host-side retransmission. A non-zero error count does NOT mean audio data was lost — it means the USB hardware detected and recovered from a link-layer event.

High sustained error rates (>10/second) may indicate a marginal cable, noisy USB port, or hub compatibility issue.

## Wire Format

### UsbErrorStatsPacket (24 bytes)

Returned by `REQ_GET_USB_ERROR_STATS`. All fields are little-endian unsigned 32-bit integers. Fits in a single 64-byte USB control transfer.

```
Offset  Size  Type      Field        Description
0       4     uint32_t  total        Total error interrupts since last reset
4       4     uint32_t  crc          CRC errors (corrupted packet)
8       4     uint32_t  bitstuff     Bit stuffing violations (encoding error)
12      4     uint32_t  rx_overflow  RX FIFO overflows (SIE timing stress)
16      4     uint32_t  rx_timeout   RX timeouts (no response in time)
20      4     uint32_t  data_seq     Data sequence (PID toggle) errors
```

**Invariant:** `total >= crc + bitstuff + rx_overflow + rx_timeout + data_seq`. The total counts every error interrupt; the per-type fields count based on SIE status bits which may not always be set when the interrupt fires.

**Byte order:** Little-endian (ARM native). On little-endian hosts (x86, ARM), the raw bytes can be cast directly to a uint32_t array.

### C struct definition

```c
typedef struct __attribute__((packed)) {
    uint32_t total;        // byte 0
    uint32_t crc;          // byte 4
    uint32_t bitstuff;     // byte 8
    uint32_t rx_overflow;  // byte 12
    uint32_t rx_timeout;   // byte 16
    uint32_t data_seq;     // byte 20
} UsbErrorStatsPacket;     // 24 bytes
```

### Python struct format

```python
import struct
USB_ERROR_STATS_FORMAT = '<6I'  # 6 x uint32 little-endian, 24 bytes
fields = ('total', 'crc', 'bitstuff', 'rx_overflow', 'rx_timeout', 'data_seq')
```

### Swift struct

```swift
struct UsbErrorStats {
    let total: UInt32
    let crc: UInt32
    let bitstuff: UInt32
    let rxOverflow: UInt32
    let rxTimeout: UInt32
    let dataSeq: UInt32
}
// Parse from 24-byte Data with: data.withUnsafeBytes { $0.load(as: UsbErrorStats.self) }
```

## Vendor Commands

All commands use USB control transfers on the vendor interface (interface 2, class 0xFF).

### REQ_GET_USB_ERROR_STATS (0xB2)

Read the current error counters.

| Field | Value |
|-------|-------|
| bmRequestType | `0xC1` (device-to-host, vendor, interface) |
| bRequest | `0xB2` |
| wValue | `0x0000` (unused) |
| wIndex | `0x0002` (vendor interface number) |
| wLength | `24` |
| Response | 24-byte `UsbErrorStatsPacket` |

### REQ_RESET_USB_ERROR_STATS (0xB3)

Reset all error counters to zero.

| Field | Value |
|-------|-------|
| bmRequestType | `0xC1` (device-to-host, vendor, interface) |
| bRequest | `0xB3` |
| wValue | `0x0000` (unused) |
| wIndex | `0x0002` (vendor interface number) |
| wLength | `1` |
| Response | 1-byte success (`0x01`) |

## Application Patterns

### One-Shot Link Quality Check

Read the counters once to see if any errors have accumulated since the device was plugged in (or since the last reset).

```python
import usb.core

VENDOR_INTERFACE = 2

def get_usb_errors(dev):
    data = dev.ctrl_transfer(0xC1, 0xB2, 0, VENDOR_INTERFACE, 24)
    values = struct.unpack('<6I', data)
    return dict(zip(fields, values))

stats = get_usb_errors(dev)
if stats['total'] == 0:
    print("USB link clean — no errors detected")
else:
    print(f"USB errors: {stats['total']} total")
    for name in fields[1:]:
        if stats[name] > 0:
            print(f"  {name}: {stats[name]}")
```

### Periodic Monitoring (Rate Calculation)

Poll periodically and compute error rate to detect degrading link quality.

```python
import time

def monitor_usb_errors(dev, interval_sec=1.0, duration_sec=60):
    # Reset counters to start fresh
    dev.ctrl_transfer(0xC1, 0xB3, 0, VENDOR_INTERFACE, 1)

    prev = get_usb_errors(dev)
    prev_time = time.monotonic()

    for _ in range(int(duration_sec / interval_sec)):
        time.sleep(interval_sec)
        curr = get_usb_errors(dev)
        curr_time = time.monotonic()

        dt = curr_time - prev_time
        rate = (curr['total'] - prev['total']) / dt

        if rate > 0:
            print(f"USB error rate: {rate:.1f}/sec  "
                  f"(CRC={curr['crc'] - prev['crc']}, "
                  f"bitstuff={curr['bitstuff'] - prev['bitstuff']}, "
                  f"timeout={curr['rx_timeout'] - prev['rx_timeout']})")

        prev = curr
        prev_time = curr_time
```

### Cable/Hub Quality Assessment

Compare error rates across different cables or USB ports.

```python
def assess_link_quality(dev, test_duration_sec=30):
    """Run for test_duration_sec and return a quality rating."""
    dev.ctrl_transfer(0xC1, 0xB3, 0, VENDOR_INTERFACE, 1)  # reset
    time.sleep(test_duration_sec)
    stats = get_usb_errors(dev)

    rate = stats['total'] / test_duration_sec
    if rate == 0:
        return "excellent"
    elif rate < 1:
        return "good"
    elif rate < 10:
        return "marginal"
    else:
        return "poor — consider a different cable or port"
```

### Health Dashboard Thresholds

| Metric | Good | Warning | Poor |
|--------|------|---------|------|
| Error rate | 0/sec | <1/sec | >10/sec |
| CRC errors | 0 | <0.5/sec | >5/sec |
| RX overflow | 0 | Any | Sustained |
| Bit stuff | 0 | <0.5/sec | >5/sec |

- **Good:** No action needed.
- **Warning:** Monitor. May cause occasional audio glitches under heavy system load.
- **Poor:** Try a shorter/better cable, avoid USB hubs, or use a different port. Sustained high error rates correlate with intermittent audio dropouts.

### Integration with Buffer Stats

USB errors and buffer underruns are related but distinct. A burst of USB errors can cause the host to miss a retransmission window, leading to a late audio packet, which shows up as a buffer underrun. To correlate:

1. Reset both counters: `REQ_RESET_USB_ERROR_STATS` (0xB3) + `REQ_RESET_BUFFER_STATS` (0xB1)
2. Play audio for the test period
3. Read both: `REQ_GET_USB_ERROR_STATS` (0xB2) + `REQ_GET_BUFFER_STATS` (0xB0)
4. If buffer `consumer_min_fill_pct` hit 0% AND USB error count is elevated, the underrun was likely caused by USB link issues, not firmware timing

## Counter Behavior

- Counters are 32-bit unsigned integers. At 10 errors/second, they wrap after ~13.6 years. Overflow is not a practical concern.
- Counters survive across audio stream start/stop cycles — they are only reset by `REQ_RESET_USB_ERROR_STATS` or device power cycle.
- Counters are incremented inside the USB IRQ handler. Reads via vendor commands are not atomic across all 6 fields (each is read independently). For rate calculations, small inconsistencies between fields are negligible.
- The `total` field is incremented on every error interrupt. Per-type fields are incremented based on the SIE status register bits active at that instant. A single interrupt may set multiple SIE bits (e.g., both CRC and bit stuff), so per-type sums can occasionally exceed `total` for that window.

## Firmware Implementation

**Error handler:** `firmware/pico-extras/src/rp2_common/usb_device/usb_device.c`
- On any `USB_INTS_ERROR_BITS` interrupt: reads `usb_hw->sie_status`, clears all error bits via `usb_hw_clear->sie_status`, increments counters.
- No bus reset. No re-enumeration. The host retransmits automatically.

**Counters:** 6 `volatile uint32_t` globals in `usb_device.c`, accessed via `extern` from `usb_audio.c`.

**Vendor commands:** Handled in `firmware/DSPi/usb_audio.c` vendor request handler.
- `REQ_GET_USB_ERROR_STATS` (0xB2): reads all 6 counters into `UsbErrorStatsPacket`, sends 24 bytes.
- `REQ_RESET_USB_ERROR_STATS` (0xB3): zeroes all 6 counters, sends 1-byte ACK.

**Request IDs:** Defined in `firmware/DSPi/config.h`.
