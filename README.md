# DSPi Firmware

**DSPi** transforms a Raspberry Pi Pico or other RP2040-based board into a very competent and inexpensive little digital audio processor. It acts as a USB sound card with an onboard DSP engine, allowing you to make use of essential tools like room correction, active crossovers, parametric EQ, and time alignment.

It is my hope that the RP2040 and RP2350 will garner a reputation as the "swiss army knife of audio for less than a cup of coffee".

---

## Key Capabilities

*   **USB Audio Interface:** Plug-and-play under macOS, Windows, Linux, and iOS.
*   **Parametric Equalization:** Ten PEQ filters per master audio channel (Left/Right) and two filters per output channel (Left/Right/Sub).
*   **Per-Channel Gain & Mute:** Independent gain (-60dB to +10dB) and mute controls for each output channel.
*   **Time Alignment:** Delay individual channels (up to 170ms) to ensure alignment between subwoofer and main speakers.
*   **SPDIF Output:** Digital SPDIF output enables the use of any standard DAC.
*   **Subwoofer Output:** Dedicated mono output channel that enables direct subwoofer output without the need for a second DAC.

---

## Platform Support

| Feature | RP2040 (Pico) | RP2350 (Pico 2) |
|---------|---------------|-----------------|
| **Clock Speed** | 288 MHz (OC) | 288 MHz |
| **Audio Processing** | Q28 Fixed-Point | Mixed-Precision Float/Double |
| **Filter Bands** | 26 total | 50 total |
| **Output EQ** | 2 bands/channel | 10 bands/channel |
| **Math Engine** | Hand-optimized ARM Assembly | Hardware FPU + DCP Coprocessor |
| **Status** | ✅ Production | ✅ Production |

Both platforms are fully tested and production-ready. The RP2350 offers significantly more processing headroom thanks to its hardware floating-point unit and Double-precision Coprocessor (DCP), enabling more filter bands and higher-precision audio processing.

---

## Audio Signal Chain

DSPi processes audio in a linear, low latency pipeline.

1.  **Input (USB):** Audio from your PC or mobile device.
2.  **Preamp & Master EQ:** The signal is volume-adjusted and passed through 20 bands of PEQ (10 per channel).
3.  **Duplication & Summing:**
    *   **Main Channels:** Routed to the digital output after PEQ, becoming Main Outs.
    *   **Subwoofer Channel:** Created by summing left and right main channels, after PEQ.
4.  **Output Channels:**
    *   **Main Outs:** 2 bands of PEQ per channel (ideal for 12dB/oct or 24dB/oct high pass filter).
    *   **Subwoofer Out:** 2 bands of PEQ (ideal for 12dB/oct or 24dB/oct low pass filter)
5.  **Per-Channel Gain & Mute:** Each output channel has independent gain and mute controls applied after EQ.
6.  **Time Alignment:** Delays are applied to each channel, if configured.
7.  **Hardware Outputs:**
    *   **S/PDIF (Digital):** Connects to your DAC or Receiver.
    *   **PDM (Analog):** Connects to an active subwoofer's analog input.

---

## Hardware Setup

### Flashing the Firmware
1.  Download the latest `dspi.uf2` release.
2.  Hold the **BOOTSEL** button on your Pico while plugging it into your computer.
3.  A drive named `RPI-RP2` will appear.
4.  Drag and drop the `dspi.uf2` file onto this drive.
5.  The Pico will reboot and appear as a "Weeb Labs DSPi' audio device.
6.  Download and launch the DSPi Console application to control the DSPi.

### Wiring Guide

Connecting DSPi to your audio hardware is straightforward.

| Function | Pin | Connection |
| :--- | :--- | :--- |
| **Digital Audio Out** (S/PDIF) | `GPIO 20` | Connects to the input of a DAC or Receiver. |
| **Subwoofer Out** (PDM) | `GPIO 10` | Connects to an active subwoofer. |
| **USB** | `Micro-USB` | Connects to your Host device (PC/Mac/Mobile Device). |

> **Note:** S/PDIF output requires either a Toshiba TX179 optical transmitter or a simple resistor divider . PDM output is a 1-bit logic signal that requires a resistor and capacitor to turn into analog audio.

<img src="Images/toslink.jpg" alt="Alt text" width="49%">  <img src="Images/spdif_converter.jpg" alt="Alt text" width="49%">

---

## Developer Reference

The following section details the internal architecture for developers wishing to modify the firmware or write custom control software.

### System Architecture
*   **Core 0:** Handles USB communication (TinyUSB), audio streaming, and control logic.
*   **Core 1:** Dedicated to the Delta-Sigma modulator (PDM generation) and buffer management.
*   **PIO & DMA:** Hardware offloading for S/PDIF encoding and bitstream generation ensures zero CPU overhead for I/O.
*   **Math Engine:**
    *   **RP2040:** 32-bit fixed-point (Q28) processing with hand-optimized ARM assembly for the inner DSP loop.
    *   **RP2350:** Mixed-precision pipeline using single-precision floats for coefficients and double-precision accumulators via the hardware DCP coprocessor.

### Subwoofer PDM Specifications
The subwoofer output uses a high-performance software-defined Delta-Sigma modulator running on Core 1.

*   **Modulation:** 2nd-Order Delta-Sigma
*   **Oversampling Ratio:** 256x (12.288 MHz bit clock)
*   **Dither:** TPDF (Triangular Probability Density Function)
*   **DC Protection:** Leaky integrator design preventing DC offset accumulation.

The objective here was to use as much of Core 1 as necessary to produce an output that could be used full-range while sounding perfectly fine, even if will only be used to feed a subwoofer.  This implementation is very stable and without pops, clicks or idle tones.

### Performance Tuning

The firmware dynamically adjusts clock speed based on sample rate to maintain optimal PIO divider ratios for S/PDIF timing accuracy:

| Platform | 44.1 kHz Mode | 48 kHz Mode | Core Voltage |
|----------|---------------|-------------|--------------|
| **RP2040** | 264.6 MHz | 288 MHz | 1.20V (overclock) |
| **RP2350** | 264.6 MHz | 288 MHz | 1.10V (nominal) |

The RP2040 requires a slight voltage bump to reliably reach 288 MHz, while the RP2350 achieves this at its default voltage. Clock switching occurs automatically during sample rate changes with proper sequencing (voltage adjustment before frequency increase).

### USB Control Protocol
Configuration is performed via **Interface 2** (Vendor Interface) using Control Transfers under Windows and via **Interface 0** under macOS.

**Request Table (Hex Codes)**

| ID | Name | Payload | Description |
| :--- | :--- | :--- | :--- |
| `0x42` | `REQ_SET_EQ_PARAM` | 16 bytes | Upload filter parameters. |
| `0x43` | `REQ_GET_EQ_PARAM` | 4 bytes | Read back filter parameters. |
| `0x44` | `REQ_SET_PREAMP` | 4 bytes | Set global gain (float dB). |
| `0x46` | `REQ_SET_BYPASS` | 1 byte | Bypass Master EQ (1=On, 0=Off). |
| `0x48` | `REQ_SET_DELAY` | 4 bytes | Set channel delay (float ms). |
| `0x50` | `REQ_GET_STATUS` | 4 bytes | Get system statistics (see wValue table below). |
| `0x54` | `REQ_SET_CHANNEL_GAIN` | 4 bytes | Set output channel gain (float dB). |
| `0x56` | `REQ_SET_CHANNEL_MUTE` | 1 byte | Mute output channel (1=Muted, 0=Unmuted). |
| `0x51` | `REQ_SAVE_PARAMS` | 1 byte | Save settings to Flash. |
| `0x53` | `REQ_FACTORY_RESET` | 1 byte | Reset RAM to defaults. |

*(Full list of requests available in `config.h`)*

### REQ_GET_STATUS (0x50) - System Statistics

The `REQ_GET_STATUS` request returns a 4-byte value based on the `wValue` field in the control transfer:

| wValue | Returns | Description |
| :--- | :--- | :--- |
| `0` | uint32 | Peaks for channels 0-1 (packed 16-bit values) |
| `1` | uint32 | Peaks for channels 2-3 (packed 16-bit values) |
| `2` | uint32 | Peak for channel 4 + CPU0/CPU1 load (packed) |
| `3` | uint32 | PDM ring buffer overruns |
| `4` | uint32 | PDM ring buffer underruns |
| `5` | uint32 | PDM DMA overruns |
| `6` | uint32 | PDM DMA underruns |
| `7` | uint32 | SPDIF overruns |
| `8` | uint32 | SPDIF underruns |
| `9` | 12 bytes | Combined: all 5 peaks + CPU loads (12-byte transfer) |
| `10` | uint32 | USB audio packet count |
| `11` | uint32 | USB alt setting |
| `12` | uint32 | USB audio mounted state |
| `13` | uint32 | **System clock frequency (Hz)** |
| `14` | uint32 | **Core voltage (millivolts)** |
| `15` | uint32 | **Sample rate (Hz)** |
| `16` | int32 | **System temperature (centi-degrees C)** |

**Example:** To get the current clock frequency, send `REQ_GET_STATUS` with `wValue=13`. The device will return the system clock frequency in Hz as a 32-bit integer.

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



### Building from Source

To build the firmware yourself, you'll need a standard Raspberry Pi Pico C/C++ development environment.

#### 1. Install Prerequisites
Ensure you have the following tools installed:
*   **CMake** (3.13 or newer)
*   **Arm GNU Toolchain** (`arm-none-eabi-gcc`, etc.)
*   **Python 3** (for Pico SDK scripts)
*   **Git**

#### 2. Clone the Repository
Clone the project recursively to include the Pico SDK and other submodules:
```bash
git clone --recursive https://github.com/weeblabs/DSPi.git
cd DSPi
```

*If you already cloned without `--recursive`, run:*
```bash
git submodule update --init --recursive
```

#### 3. Build the Firmware
You can build for either the standard **RP2040** (Raspberry Pi Pico) or the newer **RP2350** (Raspberry Pi Pico 2). The build system uses separate directories to avoid conflicts. Both targets are fully tested and production-ready.

**Option A: Build for RP2040 (Standard Pico)**
```bash
mkdir build-rp2040
cd build-rp2040
cmake -DPICO_BOARD=pico ..
make
```
*Output:* `foxdac/foxdac.uf2`

**Option B: Build for RP2350 (Pico 2)**
```bash
mkdir build-rp2350
cd build-rp2350
cmake -DPICO_BOARD=pico2 ..
make
```
*Output:* `foxdac/foxdac.uf2`

#### 4. Flash the Device
1.  Hold the **BOOTSEL** button on your board while plugging it in.
2.  Drag and drop the generated `.uf2` file onto the `RPI-RP2` (or `RP2350`) drive.

