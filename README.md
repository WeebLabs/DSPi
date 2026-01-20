# DSPi Firmware

**DSPi** transforms a standard Raspberry Pi Pico (RP2040) into a high-fidelity digital audio processor. It acts as a USB Sound Card with a built-in DSP engine, allowing you to tune your audio system with professional tools like active crossovers, parametric EQ, and time alignment.

Whether you are building a custom speaker system, integrating a subwoofer, or correcting room acoustics, DSPi provides the processing power you need for less than the cost of a cup of coffee.

---

## Key Capabilities

*   **USB Audio Interface:** Plug-and-play operation with macOS, Windows, Linux, and iOS (48kHz / 16-bit).
*   **10-Band Master EQ:** Apply broad tonal corrections or surgical fixes to the entire audio signal.
*   **Active Crossover:** Split frequencies between your main speakers and a subwoofer with dedicated output filters.
*   **Time Alignment:** Delay specific channels (up to 170ms) to ensure sound from all speakers reaches your ears at the exact same moment.
*   **Subwoofer Output:** Dedicated mono output channel with independent processing.
*   **Digital Preamp:** High-precision digital volume control with a hardware bypass mode for pure signal testing.

---

## Audio Signal Flow

DSPi processes audio in a linear pipeline, ensuring low latency and high precision.

1.  **Input (USB):** Audio enters from your computer.
2.  **Preamp & Master EQ:** The signal is volume-adjusted and passed through 10 bands of Parametric EQ (Left/Right).
3.  **Crossover Split:**
    *   **Main Channels:** Passed through to the digital output.
    *   **Subwoofer Channel:** Created by summing Left and Right audio.
4.  **Output Tuning:**
    *   **Main Outs:** 2 bands of PEQ per channel (great for baffle step compensation or room correction).
    *   **Sub Out:** 2 bands of PEQ (typically used for a Low Pass filter).
5.  **Time Alignment:** Each channel is individually delayed to align speaker drivers.
6.  **Hardware Output:**
    *   **S/PDIF (Digital):** Connects to your DAC or Receiver.
    *   **PDM (DAC):** Connects to a simple RC filter or subwoofer amplifier input.

---

## Hardware Setup

Connecting DSPi to your audio gear is straightforward. You will need a Raspberry Pi Pico.

### Wiring Guide

| Function | Pin | Connection |
| :--- | :--- | :--- |
| **Digital Audio Out** (S/PDIF) | `GPIO 20` | Connect to the input of a DAC or Receiver. (Requires a simple optical/coaxial circuit). |
| **Subwoofer Out** (PDM) | `GPIO 10` | Connect to an active subwoofer amp. (Requires a basic RC Low-Pass filter). |
| **USB** | `Micro-USB` | Connect to your Host device (PC/Mac). |

> **Note:** S/PDIF output requires a Toshiba TX179 (Optical) or a simple resistor divider (Coaxial). PDM output is a 1-bit logic signal that requires a resistor and capacitor to turn into analog audio.

### Subwoofer PDM Specifications
The subwoofer output uses a high-performance software-defined Sigma-Delta modulator running on Core 1.

*   **Modulation:** 2nd-Order Sigma-Delta
*   **Oversampling Ratio:** 256x (12.288 MHz bit clock)
*   **Dither:** TPDF (Triangular Probability Density Function)
*   **DC Protection:** Leaky integrator design preventing DC offset accumulation.

---

## Developer Reference

The following section details the internal architecture for developers wishing to modify the firmware or write custom control software.

### System Architecture
*   **Core 0:** Handles USB communication (TinyUSB), audio streaming, and control logic.
*   **Core 1:** Dedicated to the Sigma-Delta modulator (PDM generation) and buffer management.
*   **PIO & DMA:** Hardware offloading for S/PDIF encoding and bitstream generation ensures zero CPU overhead for I/O.
*   **Math Engine:** 32-bit fixed-point (`int32_t`) processing pipeline running natively at 48kHz.

### USB Control Protocol
Configuration is performed via **Interface 2** (Vendor Interface) using Control Transfers.

**Request Table (Hex Codes)**

| ID | Name | Payload | Description |
| :--- | :--- | :--- | :--- |
| `0x42` | `REQ_SET_EQ_PARAM` | 16 bytes | Upload filter parameters. |
| `0x43` | `REQ_GET_EQ_PARAM` | 4 bytes | Read back filter parameters. |
| `0x44` | `REQ_SET_PREAMP` | 4 bytes | Set global gain (float dB). |
| `0x46` | `REQ_SET_BYPASS` | 1 byte | Bypass Master EQ (1=On, 0=Off). |
| `0x48` | `REQ_SET_DELAY` | 4 bytes | Set channel delay (float ms). |
| `0x50` | `REQ_GET_STATUS` | 12 bytes | Get live meter levels and CPU load. |
| `0x51` | `REQ_SAVE_PARAMS` | 1 byte | Save settings to Flash. |
| `0x53` | `REQ_FACTORY_RESET` | 1 byte | Reset RAM to defaults. |

*(Full list of requests available in `config.h`)*

### Data Structures

**Filter Packet (16 bytes):**
```c
struct __attribute__((packed)) {
    uint8_t channel;  // 0-4
    uint8_t band;     // 0-9
    uint8_t type;     // 0=Flat, 1=Peak, 2=LS, 3=HS, 4=LP, 5=HP
    uint8_t reserved;
    float freq;       // Hz
    float Q;
    float gain_db;
}
```

---

## Installation

### Flashing the Firmware
1.  Download the latest `foxdac.uf2` release.
2.  Hold the **BOOTSEL** button on your Pico while plugging it into your computer.
3.  A drive named `RPI-RP2` will appear.
4.  Drag and drop the `foxdac.uf2` file onto this drive.
5.  The Pico will reboot as a DSPi Audio Device.

### Building from Source
If you wish to modify the code:

```bash
mkdir build
cd build
cmake ..
make
```
