#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include <limits.h>

// ----------------------------------------------------------------------------
// GLOBAL VARIABLES
// ----------------------------------------------------------------------------
extern volatile int overruns;
extern volatile uint32_t pio_samples_dma;

// Buffer monitoring counters
extern volatile uint32_t pdm_ring_overruns;   // Core 0 couldn't push (ring full)
extern volatile uint32_t pdm_ring_underruns;  // Core 1 needed sample but ring empty
extern volatile uint32_t pdm_dma_overruns;    // Core 1 write caught up to DMA read
extern volatile uint32_t pdm_dma_underruns;   // Core 1 write fell behind DMA read
extern volatile uint32_t spdif_overruns;      // USB callback couldn't get buffer (pool full)
extern volatile uint32_t spdif_underruns;     // USB packet gap > 2ms (consumer likely starved)
extern volatile uint32_t usb_audio_packets;   // Debug: count of USB audio packets received
extern volatile uint32_t usb_audio_alt_set;   // Debug: last alt setting selected
extern volatile uint32_t usb_audio_mounted;   // Debug: audio mounted state

// USB audio feedback (SOF-measured, 10.14 fixed-point)
extern volatile uint32_t feedback_10_14;
extern volatile uint32_t nominal_feedback_10_14;

// ----------------------------------------------------------------------------
// CONFIGURATION
// ----------------------------------------------------------------------------

#define ENABLE_SUB 1

// S/PDIF Output Pins
#undef PICO_AUDIO_SPDIF_PIN
#define PICO_AUDIO_SPDIF_PIN   6    // S/PDIF 1 (Out 1-2)
#define PICO_SPDIF_PIN_2       7    // S/PDIF 2 (Out 3-4)
#if PICO_RP2350
#define PICO_SPDIF_PIN_3       8    // S/PDIF 3 (Out 5-6) — RP2350 only
#define PICO_SPDIF_PIN_4       9    // S/PDIF 4 (Out 7-8) — RP2350 only
#endif

// PDM Subwoofer Output Pin (PIO1)
#define PICO_PDM_PIN           10   // PDM sub (Out 9)

// ADAT bulk output default pin (RP2350 only).  GPIO 12 is the only default-free
// pin adjacent to the output cluster (5 = SPDIF RX, 6-9 = SPDIF TX, 10 = PDM,
// 11 = DAC mute, 13 = MCK, 14-19 = I2S/UART/I2C).
#if PICO_RP2350
#define PICO_ADAT_PIN          12
#endif

// Legacy aliases
#define PICO_AUDIO_SPDIF_SUB_PIN PICO_PDM_PIN

// Clip detection threshold — slightly above 1.0 to avoid false positives
// from float precision noise when 0 dBFS signals pass through biquad filters.
// +0.01 dB headroom: catches audible clipping, ignores float rounding artifacts.
#define CLIP_THRESH_F   1.001f              // RP2350 float path
#define CLIP_THRESH_Q28 ((1 << 28) + 268)   // RP2040 Q28 path (~0.001 in Q28)

#define FILTER_SHIFT 28

// PDM Configuration
#define PDM_OVERSAMPLE 256
#define PDM_DMA_BUFFER_SIZE 2048  // Doubled for more margin (was 1024)
#define PDM_DMA_RING_BITS 13      // log2(2048 * 4 bytes) = 13
#define PDM_PIO pio1
#define PDM_SM 0
#define PDM_CLIP_THRESH 29500  // ~90% modulation (was 26214 / 80%)

// PDM Sigma-Delta Tuning
// Dither mask: controls TPDF amplitude. Start small (0x1FF), increase if idle tones persist
#define PDM_DITHER_MASK 0x1FF
// Leakage shift: higher = less leakage. Applied once per audio sample.
// 16 gives ~1.4s time constant at 48kHz, safe for bass
#define PDM_LEAKAGE_SHIFT 16

// PDM Soft Start/Stop
#define PDM_FADE_IN_SHIFT   10                        // 2^10 = 1024 samples
#define PDM_FADE_IN_SAMPLES (1u << PDM_FADE_IN_SHIFT) // ~21ms at 48kHz

// SPDIF Buffer Configuration
#define AUDIO_BUFFER_COUNT    8   // Producer buffers per SPDIF instance
#define SPDIF_CONSUMER_BUFFER_COUNT 16  // Consumer buffers per SPDIF instance (DMA side)
#define AUDIO_BUFFER_SAMPLES  192

// DELAY CONFIGURATION
// Per-platform maximum delay line length.  RP2040 has 264 KB SRAM total; under
// the XIP-with-hot-RAM layout the delay arrays (NUM_DELAY_CHANNELS ×
// MAX_DELAY_SAMPLES × 4 bytes) still compete with the RAM-resident hot text and
// heap.  Keep RP2040 at 21 ms; RP2350 has ample SRAM for the longer window.
#if PICO_RP2350
#define MAX_DELAY_SAMPLES 2048   // 42 ms at 48 kHz
#else
#define MAX_DELAY_SAMPLES 1024   // 21 ms at 48 kHz (RP2040 SRAM budget)
#endif
#define MAX_DELAY_MASK    (MAX_DELAY_SAMPLES - 1)

// Latency alignment (in samples - automatically adapts to sample rate)
// SPDIF path: watermark = AUDIO_BUFFER_COUNT/2 buffers
// PDM path: DMA buffer = PDM_DMA_BUFFER_SIZE/8 PCM samples
#define SPDIF_BUFFER_SAMPLES  384  // Consumer path depth at 50% fill (16 buffers * 48 samples * 50%)
#define PDM_BUFFER_SAMPLES    (PDM_DMA_BUFFER_SIZE / 8)                           // 256
#define SUB_ALIGN_SAMPLES     (SPDIF_BUFFER_SAMPLES - PDM_BUFFER_SAMPLES)         // 128

// ----------------------------------------------------------------------------
// VENDOR INTERFACE CONFIGURATION (WinUSB / WCID)
// ----------------------------------------------------------------------------

#define VENDOR_INTERFACE_NUMBER  2

// RESTORED: Dummy Endpoint for macOS compatibility
#define VENDOR_EP_IN        0x83
#define VENDOR_EP_SIZE      64
#define VENDOR_EP_INTERVAL  10

// Microsoft OS 2.0 vendor request code — embedded in the BOS Platform
// Capability descriptor. Windows 8.1+ uses it as the bRequest in
// bmRequestType=0xC0 vendor-type GETs to fetch the MS OS 2.0 descriptor
// set. Reserved for Microsoft; never reuse for application opcodes.
//
// Note: 0x01 also happens to be the value of UAC1_REQ_SET_CUR
// (usb_descriptors.h). They cannot collide because UAC1 SET_CUR is
// TUSB_REQ_TYPE_CLASS and reaches the audio class handler in usb_audio.c,
// while MS OS 2.0 requests are TUSB_REQ_TYPE_VENDOR and reach
// tud_vendor_control_xfer_cb in vendor_commands.c.
#define MS_VENDOR_CODE      0x01

// Psychoacoustic bass enhancement (missing-fundamental harmonics; psybass.h)
#define REQ_SET_PSYBASS             0x30
#define REQ_GET_PSYBASS             0x31
#define REQ_SET_PSYBASS_CUTOFF      0x32
#define REQ_GET_PSYBASS_CUTOFF      0x33
#define REQ_SET_PSYBASS_HARMONICS   0x34
#define REQ_GET_PSYBASS_HARMONICS   0x35
#define REQ_SET_PSYBASS_DRIVE       0x36
#define REQ_GET_PSYBASS_DRIVE       0x37
#define REQ_SET_PSYBASS_CHARACTER   0x38
#define REQ_GET_PSYBASS_CHARACTER   0x39
#define REQ_SET_PSYBASS_ORIGINAL    0x3A
#define REQ_GET_PSYBASS_ORIGINAL    0x3B
#define REQ_SET_PSYBASS_MASK        0x3C
#define REQ_GET_PSYBASS_MASK        0x3D

// Vendor Request Commands (EP0 control transfers)
#define REQ_SET_EQ_PARAM    0x42
#define REQ_GET_EQ_PARAM    0x43
#define REQ_SET_PREAMP      0x44
#define REQ_GET_PREAMP      0x45
#define REQ_SET_BYPASS      0x46
#define REQ_GET_BYPASS      0x47
#define REQ_SET_DELAY       0x48
#define REQ_GET_DELAY       0x49

// Stereo upmixer (RP2350 only; see upmix.h and
// Documentation/Features/upmixer_spec.md).  RP2040 SETs STALL (unsupported),
// GETs return zeros.
#define REQ_UPMIX_SET_CONFIG        0x4A  // payload = UpmixConfigPacket (44 B)
#define REQ_UPMIX_GET_CONFIG        0x4B  // returns UpmixConfigPacket
#define REQ_UPMIX_SET_PARAM         0x4C  // wValue = UPMIX_PARAM_*, payload = float (4 B)
#define REQ_UPMIX_GET_PARAM         0x4D  // wValue = UPMIX_PARAM_*, returns float
#define REQ_UPMIX_GET_STATUS        0x4E  // returns UpmixStatus (16 B)
// 0x4F reserved for future upmixer extensions
#define REQ_GET_STATUS      0x50
#define REQ_SAVE_PARAMS     0x51
// REQ_SAVE_OUTPUT_CONFIG (0x52): persist the live physical IO/output configuration
// (output pins, output types, I2S MCK/BCK, SPDIF RX pin) into the directory's
// device-global block.  Used in OUTPUT_CONFIG_MODE_INDEPENDENT (the "stored
// independently, like master volume" mode); accepted but dormant in WITH_PRESET
// mode.  No payload.
//
// Reassigned from the former REQ_LOAD_PARAMS — a deprecated synchronous "revert
// to saved" that ran flash_load_params()->preset_load() in the USB control/IRQ
// handler without stopping SPDIF RX or fencing Core 1, crashing the device on
// SPDIF input.  Hosts use the deferred, SPDIF-safe REQ_PRESET_LOAD (0x91) instead.
#define REQ_SAVE_OUTPUT_CONFIG  0x52
#define REQ_FACTORY_RESET   0x53
#define REQ_SET_CHANNEL_GAIN 0x54
#define REQ_GET_CHANNEL_GAIN 0x55
#define REQ_SET_CHANNEL_MUTE 0x56
#define REQ_GET_CHANNEL_MUTE 0x57
#define REQ_SET_LOUDNESS            0x58
#define REQ_GET_LOUDNESS            0x59
#define REQ_SET_LOUDNESS_REF        0x5A
#define REQ_GET_LOUDNESS_REF        0x5B
#define REQ_SET_LOUDNESS_INTENSITY  0x5C
#define REQ_GET_LOUDNESS_INTENSITY  0x5D
#define REQ_SET_CROSSFEED           0x5E
#define REQ_GET_CROSSFEED           0x5F
#define REQ_SET_CROSSFEED_PRESET    0x60
#define REQ_GET_CROSSFEED_PRESET    0x61
#define REQ_SET_CROSSFEED_FREQ      0x62
#define REQ_GET_CROSSFEED_FREQ      0x63
#define REQ_SET_CROSSFEED_FEED      0x64
#define REQ_GET_CROSSFEED_FEED      0x65
#define REQ_SET_CROSSFEED_ITD       0x66
#define REQ_GET_CROSSFEED_ITD       0x67

// ADAT Input Commands (RP2350 only; state round-trips on RP2040). 0x6F reserved.
#define REQ_SET_ADAT_INPUT_ENABLE      0x68
#define REQ_GET_ADAT_INPUT_ENABLE      0x69
#define REQ_SET_ADAT_INPUT_PIN         0x6A  // wValue = GPIO; PIN_RESET_TO_DEFAULT
                                             // clears to unset (only while disabled)
#define REQ_GET_ADAT_INPUT_PIN         0x6B  // returns uint8_t (0xFF = unset)
#define REQ_SET_ADAT_INPUT_CLOCK_MODE  0x6C
#define REQ_GET_ADAT_INPUT_CLOCK_MODE  0x6D
#define REQ_GET_ADAT_INPUT_STATUS      0x6E

// Matrix Mixer Commands
#define REQ_SET_MATRIX_ROUTE        0x70
#define REQ_GET_MATRIX_ROUTE        0x71
#define REQ_SET_OUTPUT_ENABLE       0x72
#define REQ_GET_OUTPUT_ENABLE       0x73
#define REQ_SET_OUTPUT_GAIN         0x74
#define REQ_GET_OUTPUT_GAIN         0x75
#define REQ_SET_OUTPUT_MUTE         0x76
#define REQ_GET_OUTPUT_MUTE         0x77
#define REQ_SET_OUTPUT_DELAY        0x78
#define REQ_GET_OUTPUT_DELAY        0x79

// Core 1 Mode Query Commands
#define REQ_GET_CORE1_MODE          0x7A
#define REQ_GET_CORE1_CONFLICT      0x7B

// Pin Configuration Commands
#define REQ_SET_OUTPUT_PIN          0x7C  // wValue = (GPIO<<8)|output_index;
                                          // GPIO = PIN_RESET_TO_DEFAULT restores the default
#define REQ_GET_OUTPUT_PIN          0x7D

// Device Identification Commands
#define REQ_GET_SERIAL              0x7E
#define REQ_GET_PLATFORM            0x7F

// Clip Detection Commands
#define REQ_CLEAR_CLIPS             0x83

// Control Surfaces Commands (physical controls/indicators on user GPIOs).
// See control_surfaces.h and Documentation/Features/control_surfaces_spec.md.
#define REQ_SET_CS_BINDING          0x84  // wValue = slot (0-15), payload = 24-byte CsBinding
#define REQ_GET_CS_BINDING          0x85  // wValue = slot, returns 24-byte CsBinding
#define REQ_GET_CS_CAPS             0x86  // wValue = 0xFFFF: header + type table; wValue = noun: 12-byte noun descriptor
#define REQ_GET_CS_STATUS           0x87  // returns 22-byte CsStatusPacket
// 0x88-0x8A reserved (claimed by the I2S slave-mode branch)
#define REQ_SET_CS_NAME             0x8B  // wValue = slot (0-15), payload = 1-32 byte name (a single
                                          // NUL byte clears it); apply-live-only preview, deferred,
                                          // result via REQ_GET_CS_STATUS
#define REQ_GET_CS_NAME             0x8C  // wValue = slot, returns 32-byte NUL-terminated live name
#define REQ_SET_CS_IR_CMD           0x8D  // wValue = sub-slot (0-7), payload = 16-byte IrCommand;
                                          // apply-live-only preview, deferred to the main loop
#define REQ_GET_CS_IR_CMD           0x8E  // wValue = sub-slot, returns 16-byte IrCommand
#define REQ_CS_IR_LEARN             0x8F  // GET; wValue 1 = arm, 0 = cancel (each returns 1 status
                                          // byte), 2 = read result (returns 8-byte
                                          // {state, protocol, 0, 0, code_le32})

// Preset System Commands
#define REQ_PRESET_SAVE             0x90
#define REQ_PRESET_LOAD             0x91
#define REQ_PRESET_DELETE           0x92
#define REQ_PRESET_GET_NAME         0x93
#define REQ_PRESET_SET_NAME         0x94
#define REQ_PRESET_GET_DIR          0x95
#define REQ_PRESET_SET_STARTUP      0x96
#define REQ_PRESET_GET_STARTUP      0x97
// Output-config persistence mode (was REQ_PRESET_SET/GET_INCLUDE_PINS).  Selects
// whether the physical IO/output config travels with presets or is device-global.
#define REQ_SET_OUTPUT_CONFIG_MODE  0x98  // payload = uint8_t mode (see OUTPUT_CONFIG_MODE_*)
#define REQ_GET_OUTPUT_CONFIG_MODE  0x99  // returns uint8_t mode
#define REQ_PRESET_GET_ACTIVE       0x9A
#define REQ_SET_CHANNEL_NAME        0x9B
#define REQ_GET_CHANNEL_NAME        0x9C
// Control Surfaces persistence (whole live config: bindings + IR commands +
// slot names).
#define REQ_CS_SAVE                 0x9D  // GET; persists the whole live CS config in one flash
                                          // write; deferred, result via REQ_GET_CS_STATUS
#define REQ_CS_REVERT               0x9E  // GET; discards the live preview and re-applies the
                                          // stored config; deferred, result via REQ_GET_CS_STATUS

// Bulk parameter transfer
#define REQ_GET_ALL_PARAMS          0xA0
#define REQ_SET_ALL_PARAMS          0xA1

// Chunked bulk-params access (USB-only).  Works around the Windows/WinUSB
// 4 KB control-transfer cap that the full WireBulkParams (5864 B at V16)
// cannot cross (GitHub issue #62).  wValue = byte offset, wLength = chunk
// size.  GET: offset 0 snapshots the struct under the bulk lock, later
// offsets read the snapshot out; the lock frees after the final chunk.
// SET: chunks must be sequential from offset 0; the normal deferred apply
// fires when the last byte lands.  UART and I2C have no size cap and use
// plain 0xA0/0xA1; these two commands are refused on those transports.
#define REQ_GET_ALL_PARAMS_CHUNK    0xA2
#define REQ_SET_ALL_PARAMS_CHUNK    0xA3

// Test signal generator (see siggen.h and
// Documentation/Features/test_signals_spec.md).  0xA9-0xAF reserved for
// future generator extensions.
#define REQ_SIGGEN_SET_CONFIG       0xA4  // payload = SiggenConfig (36 B)
#define REQ_SIGGEN_GET_CONFIG       0xA5  // returns SiggenConfig
#define REQ_SIGGEN_CONTROL          0xA6  // wValue = SIGGEN_CTL_*; returns status byte
#define REQ_SIGGEN_GET_STATUS       0xA7  // returns SiggenStatus (16 B)
#define REQ_SIGGEN_GET_CAPS         0xA8  // wValue = type index / 0xFFFF = header

// I2S Output Configuration Commands
#define REQ_SET_OUTPUT_TYPE         0xC0
#define REQ_GET_OUTPUT_TYPE         0xC1
#define REQ_SET_I2S_BCK_PIN         0xC2  // wValue = (role<<8)|GPIO; GPIO =
                                          // PIN_RESET_TO_DEFAULT restores the role's default
#define REQ_GET_I2S_BCK_PIN         0xC3
#define REQ_SET_MCK_ENABLE          0xC4
#define REQ_GET_MCK_ENABLE          0xC5
#define REQ_SET_MCK_PIN             0xC6  // wValue = GPIO; PIN_RESET_TO_DEFAULT
                                          // restores PICO_I2S_MCK_PIN
#define REQ_GET_MCK_PIN             0xC7
#define REQ_SET_MCK_MULTIPLIER      0xC8
#define REQ_GET_MCK_MULTIPLIER      0xC9

// ADAT Bulk Output Commands (RP2350 only; RP2040 SETs return
// PIN_CONFIG_INVALID_OUTPUT, GETs return zeros).  The ADAT stream carries all
// 8 post-gain output channels; it only runs at 44.1/48 kHz and auto-suspends
// at higher rates (see adat_output.h).
#define REQ_SET_ADAT_ENABLE         0xCA  // wValue = 0/1, returns status byte
#define REQ_GET_ADAT_ENABLE         0xCB  // returns uint8_t configured enable
#define REQ_SET_ADAT_PIN            0xCC  // wValue = GPIO, returns status byte; GPIO =
                                          // PIN_RESET_TO_DEFAULT restores PICO_ADAT_PIN
#define REQ_GET_ADAT_PIN            0xCD  // returns uint8_t
#define REQ_GET_ADAT_STATUS         0xCE  // returns AdatStatus (8 B)

// Buffer statistics
#define REQ_GET_BUFFER_STATS        0xB0
#define REQ_RESET_BUFFER_STATS      0xB1
#define REQ_GET_USB_ERROR_STATS     0xB2
#define REQ_RESET_USB_ERROR_STATS   0xB3

// Volume Leveller Commands
#define REQ_SET_LEVELLER_ENABLE     0xB4
#define REQ_GET_LEVELLER_ENABLE     0xB5
#define REQ_SET_LEVELLER_AMOUNT     0xB6
#define REQ_GET_LEVELLER_AMOUNT     0xB7
#define REQ_SET_LEVELLER_SPEED      0xB8
#define REQ_GET_LEVELLER_SPEED      0xB9
#define REQ_SET_LEVELLER_MAX_GAIN   0xBA
#define REQ_GET_LEVELLER_MAX_GAIN   0xBB
#define REQ_SET_LEVELLER_LOOKAHEAD  0xBC
#define REQ_GET_LEVELLER_LOOKAHEAD  0xBD
#define REQ_SET_LEVELLER_GATE       0xBE
#define REQ_GET_LEVELLER_GATE       0xBF
#define REQ_SET_LEVELLER_MASKS      0xDE  // 2 bytes: [detector_mask, apply_mask]
#define REQ_GET_LEVELLER_MASKS      0xDF

// Per-Channel Preamp Commands
#define REQ_SET_PREAMP_CH           0xD0  // wValue = channel index (0=L, 1=R), payload = float dB
#define REQ_GET_PREAMP_CH           0xD1  // wValue = channel index, returns float dB

// Master Volume Commands
#define REQ_SET_MASTER_VOLUME       0xD2  // payload = float dB (-128 mute sentinel, -127..0 range)
#define REQ_GET_MASTER_VOLUME       0xD3  // returns float dB
#define REQ_SET_MASTER_VOLUME_MODE  0xD4  // payload = uint8_t mode (see MASTER_VOLUME_MODE_*)
#define REQ_GET_MASTER_VOLUME_MODE  0xD5  // returns uint8_t mode
#define REQ_SAVE_MASTER_VOLUME      0xD6  // no payload, stores live master vol to directory
#define REQ_GET_SAVED_MASTER_VOLUME 0xD7  // returns float dB from directory's independent field

// Master Volume Constants
#define MASTER_VOL_MUTE_DB          (-128.0f)  // Sentinel value: true -inf (mute)
#define MASTER_VOL_MIN_DB           (-127.0f)  // Minimum non-mute attenuation
#define MASTER_VOL_MAX_DB           (0.0f)     // Unity gain (no attenuation)
#define MASTER_VOL_DEFAULT_DB       (-20.0f)   // Power-on / fresh-device default

// User Volume Commands.  Mirror UAC1 host volume/mute, but as a vendor channel
// so non-USB controllers (e.g. Console while SPDIF input is active, an external
// hardware control surface) can drive the same value the OS slider would.
// Same `audio_state.volume` / `audio_state.mute` fields are touched, so a vendor
// SET is observable via UAC1 GET_CUR and vice versa — single source of truth for
// "user-perceived volume", which is what loudness compensation keys against.
//
// SET always applies (no input-source guard): writes audio_state.volume and
// drives apply_vol_index_to_audio() so vol_mul + the loudness coefficient table
// move together.  Caller-side convention (e.g. "Console only writes when USB
// input is inactive") is enforced by the caller, not the firmware.
#define REQ_SET_USER_VOLUME         0xDA  // payload = float dB (clamped to [-CENTER_VOLUME_INDEX, 0])
#define REQ_GET_USER_VOLUME         0xDB  // returns float dB (audio_state.volume / 256)
#define REQ_SET_USER_MUTE           0xDC  // payload = uint8_t (0/1)
#define REQ_GET_USER_MUTE           0xDD  // returns uint8_t (0/1)

// Master Volume Persistence Modes
// Mode 0 (default): master volume is independent of presets. REQ_SAVE_MASTER_VOLUME
//   stores it in the directory sector; it's applied at boot and on factory reset.
//   Preset save/load does not touch it.
// Mode 1: master volume is part of the preset. Saved with the preset, restored on
//   preset load — same as the old include_master_volume=1 behavior.
#define MASTER_VOLUME_MODE_INDEPENDENT   0
#define MASTER_VOLUME_MODE_WITH_PRESET   1

// Output (Physical IO) Configuration Persistence Modes
// Governs the device's physical IO/routing: output pins, output types (SPDIF/I2S),
// I2S MCK/BCK, and the SPDIF RX pin.  Same 0/1 convention as the master-volume
// modes, so the legacy include_pins directory byte (default 1) maps straight onto
// it with no value migration.
// Mode 1 (WITH_PRESET, default): IO config travels with presets — stored in each
//   slot, applied on load.  (Today's behavior.)
// Mode 0 (INDEPENDENT): IO config is device-global — REQ_SAVE_OUTPUT_CONFIG stores
//   it in the directory; it is applied at boot and preset load/factory reset never
//   touch it.  Mirrors the master-volume independent mode.
#define OUTPUT_CONFIG_MODE_INDEPENDENT   0
#define OUTPUT_CONFIG_MODE_WITH_PRESET   1

// Per-Band Bypass Commands
// wValue = (channel << 8) | band; payload = uint8_t (1 = bypass, anything else = active)
#define REQ_SET_BAND_BYPASS         0xD8
#define REQ_GET_BAND_BYPASS         0xD9

// DAC Hardware Mute Commands (board-level — see dac_hw_mute.h)
#define REQ_SET_DAC_HW_MUTE_CONFIG  0xEA  // payload = 16-byte DacHwMuteConfig
#define REQ_GET_DAC_HW_MUTE_CONFIG  0xEB  // returns 16-byte DacHwMuteConfig
#define REQ_TEST_DAC_HW_MUTE        0xEC  // no payload; pulses mute ~1s

// Audio Input Source Commands
#define REQ_SET_INPUT_SOURCE        0xE0  // payload = uint8_t (InputSource enum)
#define REQ_GET_INPUT_SOURCE        0xE1  // returns uint8_t
#define REQ_GET_SPDIF_RX_STATUS     0xE2  // returns 16-byte status struct (Phase 2)
#define REQ_GET_SPDIF_RX_CH_STATUS  0xE3  // returns 24-byte IEC 60958 channel status (Phase 2)
#define REQ_SET_SPDIF_RX_PIN        0xE4  // wValue = (index<<8)|GPIO; index 0..2 selects the
                                          // SPDIF input (old hosts send wValue=pin => index 0).
                                          // GPIO = PIN_RESET_TO_DEFAULT restores that input's
                                          // default. Returns status byte.
#define REQ_GET_SPDIF_RX_PIN        0xE5  // wValue = index (0..2); returns that input's GPIO
#define REQ_SET_SPDIF_INPUT_ENABLE  0xE9  // wValue = (index<<8)|enable; index 1..2, enable 0/1.
                                          // Enables/disables an optional SPDIF input. Returns status byte.
#define REQ_GET_SPDIF_INPUT_CONFIG  0xEF  // returns 5 bytes: count, enable mask (bit0=input1),
                                          // then the GPIO for inputs 0..2

// I2S Input Commands. The device is the rate authority while I2S input is
// active (the external source slaves to our BCK/LRCLK), so the sample rate
// is selected by command instead of detected.
#define REQ_SET_INPUT_RATE          0xED  // payload = uint32_t Hz (44100/48000/96000)
#define REQ_GET_INPUT_RATE          0xEE  // returns 2x uint32_t {current Hz, selected I2S Hz}
#define REQ_SET_I2S_RX_PIN          0xF1  // wValue = (pair<<8)|GPIO; pair 0..3 selects the
                                          // stereo pair (old hosts send wValue=pin => pair 0).
                                          // GPIO = PIN_RESET_TO_DEFAULT restores that pair's
                                          // default. Returns status byte.
#define REQ_GET_I2S_RX_PIN          0xF2  // wValue = pair (0..3); returns that pair's GPIO
// I2S input channel count (RP2350 multichannel input: 2/4/6/8 -> 1..4 stereo pairs).
#define REQ_SET_I2S_INPUT_CHANNELS  0xF3  // wValue = channel count (2/4/6/8); returns status byte
#define REQ_GET_I2S_INPUT_CHANNELS  0xF4  // returns uint8_t (active count)

// I2S Clock Mode Commands. Slave mode makes BCK/LRCLK inputs driven by an
// external master; the incoming rate is auto-detected (44.1/48/96 kHz) rather
// than commanded. Only meaningful while the input source is I2S; in master
// mode (default) the device drives the clocks and REQ_SET_INPUT_RATE selects
// the rate. SET is deferred: the change applies in the main loop, not live.
#define REQ_SET_I2S_CLOCK_MODE      0x88  // payload = uint8_t (0=master, 1=slave); deferred apply
#define REQ_GET_I2S_CLOCK_MODE      0x89  // returns uint8_t live mode (pending change not reflected)
#define REQ_GET_I2S_SLAVE_STATUS    0x8A  // returns 16-byte I2sSlaveStatusPacket

// I2S clock-pin mode: whether master and slave clock modes share one BCK/LRCLK
// pair (UNIFIED, legacy behavior) or each have their own (SPLIT: master keeps
// i2s_bck_pin, slave listens on i2s_bck_pin_slave; LRCLK = BCK + 1 in both).
// The slave-role pin itself is set via REQ_SET_I2S_BCK_PIN with role = 1 in
// wValue's high byte.  See Documentation/Features/clock_pins_spec.md.
#define REQ_SET_I2S_CLOCK_PIN_MODE  0xFE  // wValue = 0 unified / 1 split; returns PIN_CONFIG_* status
#define REQ_GET_I2S_CLOCK_PIN_MODE  0xFF  // returns uint8_t live mode

// LG Sound Sync (optical) Commands.  Per-preset feature: gate is stored in
// PresetSlot, vendor SET only updates live state — flash persistence happens
// on REQ_SAVE_PRESET (matching loudness/crossfeed/leveller toggle semantics).
// See Documentation/Features/lg_sound_sync_spec.md.
#define REQ_SET_LG_SOUND_SYNC_ENABLE  0xE6  // payload = uint8_t (0 = off, 1 = on)
#define REQ_GET_LG_SOUND_SYNC_ENABLE  0xE7  // returns uint8_t
#define REQ_GET_LG_SOUND_SYNC_STATUS  0xE8  // returns 16-byte LgSoundSyncStatus

// System
#define REQ_ENTER_BOOTLOADER        0xF0

// External control interfaces (UART / I2C target).  SETs are USB-only:
// the dispatcher rejects them when they arrive over UART or I2C so an
// external controller can never reconfigure (and lock itself out of) its
// own transport.  See Documentation/Features/control_interfaces_spec.md.
#define REQ_SET_UART_CONFIG         0xF5  // payload = UartCtrlConfig (8 bytes); USB-only
#define REQ_GET_UART_CONFIG         0xF6  // returns UartCtrlConfig (live)
#define REQ_SET_I2C_CONFIG          0xF7  // payload = I2cCtrlConfig (8 bytes); USB-only
#define REQ_GET_I2C_CONFIG          0xF8  // returns I2cCtrlConfig (live)
#define REQ_GET_CTRL_IFACE_STATUS   0xF9  // returns CtrlIfaceStatus (8 bytes)

// Loudness output-channel mask (bit k = compensate output k; see loudness.h)
#define REQ_SET_LOUDNESS_MASK       0xFA  // payload = uint16 LE mask
#define REQ_GET_LOUDNESS_MASK       0xFB  // returns uint16 LE mask

// Crossfeed output-pair mask (bit p = crossfeed on output pair p; see crossfeed.h)
#define REQ_SET_CROSSFEED_OUTPUTS   0xFC  // payload = uint8 pair mask
#define REQ_GET_CROSSFEED_OUTPUTS   0xFD  // returns uint8 pair mask

// UART control interface configuration.  Fixed 8N1 framing (the wire CRC16
// covers integrity, so parity adds nothing); only the baud rate is
// configurable.  Persisted device-level in the preset directory (V6+).
// notify_enable claims the former reserved byte: every stored/wired config
// predating it carries 0 there, so notifications default off with no
// directory version bump or wire change.
typedef struct __attribute__((packed)) {
    uint8_t  enabled;       // 0/1
    uint8_t  tx_pin;        // GPIO with UARTx TX mux (pin%4 == 0)
    uint8_t  rx_pin;        // GPIO with UARTx RX mux (pin%4 == 1), same instance
    uint8_t  notify_enable; // 0/1: push async notification frames (type 0x40)
    uint32_t baud;          // UART_CTRL_BAUD_MIN..MAX
} UartCtrlConfig;

// I2C target (slave) control interface configuration.  Bus speed and clock
// stretching are controller-side properties for a target and are not stored.
typedef struct __attribute__((packed)) {
    uint8_t enabled;       // 0/1
    uint8_t sda_pin;       // GPIO with I2Cx SDA mux (pin%2 == 0)
    uint8_t scl_pin;       // GPIO with I2Cx SCL mux (pin%2 == 1), same instance
    uint8_t address;       // 7-bit target address, 0x08..0x77
    uint8_t reserved[4];
} I2cCtrlConfig;

// REQ_GET_CTRL_IFACE_STATUS response.  last_status is the result of the most
// recent REQ_SET_*_CONFIG (PIN_CONFIG_* code); live reflects whether the
// interface is actually up (differs from config.enabled after a boot-time
// pin collision kept it down).
typedef struct __attribute__((packed)) {
    uint8_t uart_last_status;
    uint8_t uart_live;
    uint8_t i2c_last_status;
    uint8_t i2c_live;
    uint8_t proto_version;   // external wire protocol version (1)
    uint8_t reserved[3];
} CtrlIfaceStatus;

#define CTRL_IFACE_PROTO_VERSION    1

// Defaults (both interfaces ship disabled; one-time USB setup enables them)
#define UART_CTRL_DEFAULT_TX_PIN    16
#define UART_CTRL_DEFAULT_RX_PIN    17
#define UART_CTRL_DEFAULT_BAUD      115200u
#define UART_CTRL_BAUD_MIN          9600u
#define UART_CTRL_BAUD_MAX          1000000u
#define I2C_CTRL_DEFAULT_SDA_PIN    18
#define I2C_CTRL_DEFAULT_SCL_PIN    19
#define I2C_CTRL_DEFAULT_ADDRESS    0x42
#define I2C_CTRL_ADDRESS_MIN        0x08
#define I2C_CTRL_ADDRESS_MAX        0x77

// Preset configuration
#define PRESET_SLOTS                10
#define PRESET_NAME_LEN             32

// Preset startup modes
#define PRESET_STARTUP_SPECIFIED    0   // Load a specific default slot
#define PRESET_STARTUP_LAST_ACTIVE  1   // Load whichever slot was last active

// Preset status codes
#define PRESET_OK                   0x00
#define PRESET_ERR_INVALID_SLOT     0x01
#define PRESET_ERR_SLOT_EMPTY       0x02
#define PRESET_ERR_CRC              0x03
#define PRESET_ERR_FLASH_WRITE      0x04

// Platform IDs
#define PLATFORM_RP2040             0
#define PLATFORM_RP2350             1

// Firmware version (BCD encoded: major in high byte, minor.patch in low byte)
#define FW_VERSION_MAJOR            1
#define FW_VERSION_MINOR            1
#define FW_VERSION_PATCH            5
#define FW_VERSION_BCD              ((FW_VERSION_MAJOR << 8) | (FW_VERSION_MINOR << 4) | FW_VERSION_PATCH)

// Universal "reset to default" escape hatch for every single-pin SET command
// (REQ_SET_OUTPUT_PIN, REQ_SET_I2S_BCK_PIN, REQ_SET_MCK_PIN, REQ_SET_ADAT_PIN,
// REQ_SET_SPDIF_RX_PIN, REQ_SET_ADAT_INPUT_PIN, REQ_SET_I2S_RX_PIN).  Sending
// this value as the pin byte restores the platform default for the addressed
// target; the mapped default then passes through the command's normal
// validation, so a reset can still fail with PIN_IN_USE etc.  0xFF is not a
// valid GPIO on either platform, so GPIO 0 stays addressable as a real pin
// (pin byte 0 always means GPIO 0, never "default").  For ADAT input the
// default is "unset" (no free default GPIO exists), so reset clears the pin.
#define PIN_RESET_TO_DEFAULT        0xFF

// Pin config status codes (shared by S/PDIF, I2S, and MCK pin commands)
#define PIN_CONFIG_SUCCESS          0x00
#define PIN_CONFIG_INVALID_PIN      0x01
#define PIN_CONFIG_PIN_IN_USE       0x02
#define PIN_CONFIG_INVALID_OUTPUT   0x03
#define PIN_CONFIG_OUTPUT_ACTIVE    0x04
#define PIN_CONFIG_INVALID_PARAM    0x05  // non-pin field out of range (baud, I2C address)

// Output type identifiers
#define OUTPUT_TYPE_SPDIF           0
#define OUTPUT_TYPE_I2S             1

// I2S default pins
#define PICO_I2S_BCK_PIN            14   // BCK; LRCLK = BCK + 1 = GPIO 15

// Default slave-mode clock pair for SPLIT clock-pin mode (dormant in UNIFIED
// mode).  Nearest sequential pair to BCK/LRCLK that no other feature defaults
// to: RP2040 has 12/13 free (ADAT is RP2350-only, MCK defaults to 21); on
// RP2350 everything adjacent is a feature default, so 26/27 is the closest
// clean pair.
#if PICO_RP2350
#define PICO_I2S_BCK_PIN_SLAVE      26   // slave BCK; LRCLK = 27
#else
#define PICO_I2S_BCK_PIN_SLAVE      12   // slave BCK; LRCLK = 13
#endif

// Master clock (optional) is generated by hardware CLK_GPOUTn — not PIO —
// so the GPIO must map to one of the four clock-output pins on the chip:
//
//   RP2040: GPIO 21 → clk_gpout0 is the only DSPi-friendly choice.
//           GPIOs 23–25 are also CLK_GPOUTn-capable but reserved on the
//           DSPi board (control / SMPS / LED) and rejected by
//           is_valid_gpio_pin() in vendor_commands.c.
//   RP2350: GPIO 13 → clk_gpout0 is the legacy DSPi default and remains
//           the cleanest pick.  GPIO 15 → clk_gpout1 also exists but
//           collides with I2S LRCLK whenever any output is set to I2S.
//           GPIO 21 still works as on RP2040.
//
// We keep GPIO 13 as the default on RP2350 (matches existing hardware
// designs) and fall back to GPIO 21 on RP2040 (the only board-friendly
// option).  When an RP2040 board loads a preset that was saved with
// mck_pin = 13, the preset-apply path in flash_storage.c / bulk_params.c
// resets the pin to this default and disables MCK.
#if PICO_RP2350
#define PICO_I2S_MCK_PIN            13
#else
#define PICO_I2S_MCK_PIN            21
#endif

// S/PDIF RX DMA channel assignments (explicit, not auto-claimed)
#if PICO_RP2350
#define PICO_SPDIF_RX_DMA_CH0      5
#define PICO_SPDIF_RX_DMA_CH1      6
#else
#define PICO_SPDIF_RX_DMA_CH0      4
#define PICO_SPDIF_RX_DMA_CH1      5
#endif

// Soft VCXO (system PLL fractional trim, soft_vcxo.c): two explicit DMA
// channels, DMA timer 0, and one PWM slice used purely as a DREQ pacer
// (no GPIO).  RP2350 slice 8 has no bondable GPIO on RP2350A, so it can
// never conflict; RP2040 slice 7 (GPIOs 14/15) is guarded against in the
// control-surface PWM LED bind path.
#if PICO_RP2350
#define SOFT_VCXO_DMA_PULSE        11
#define SOFT_VCXO_DMA_PACE         12
#define SOFT_VCXO_PWM_SLICE         8
#else
#define SOFT_VCXO_DMA_PULSE        10
#define SOFT_VCXO_DMA_PACE         11
#define SOFT_VCXO_PWM_SLICE         7
#endif
#define SOFT_VCXO_DMA_TIMER         0

// Number of configurable outputs (SPDIF + PDM)
#if PICO_RP2350
#define NUM_SPDIF_INSTANCES         4
#define NUM_PIN_OUTPUTS             5   // 4 SPDIF + 1 PDM
#else
#define NUM_SPDIF_INSTANCES         2
#define NUM_PIN_OUTPUTS             3   // 2 SPDIF + 1 PDM
#endif

// USB Audio Feature Unit IDs
#define FEATURE_MUTE_CONTROL 1u
#define FEATURE_VOLUME_CONTROL 2u
#define ENDPOINT_FREQ_CONTROL 1u

// ----------------------------------------------------------------------------
// Channel Model
//
// The device has NUM_INPUT_CHANNELS input channels and NUM_OUTPUT_CHANNELS
// output channels.  There is NO separate "master" channel: the input channels
// ARE the pre-matrix per-channel processing — what used to be "master EQ" on the
// 2 stereo inputs, generalized to N inputs.  All DSP channels (inputs then
// outputs) share one NUM_CHANNELS-wide index space used by filters[], recipes,
// names, delays, metering, etc.:
//
//   index: [ 0 .. NUM_INPUT_CHANNELS-1 ] [ NUM_INPUT_CHANNELS .. NUM_CHANNELS-1 ]
//   role:  [          inputs           ] [              outputs                ]
//
//   Inputs : per-channel PEQ + metering, NO crossover.
//   Outputs: per-channel PEQ + crossover + gain/delay/mute.
//
// NUM_INPUT_CHANNELS is the platform MAX; the *active* input count follows the
// USB alt setting (2/4/6/8 on RP2350) — see usb_input_channels.  RP2040 is
// stereo-only.
// ----------------------------------------------------------------------------
#if PICO_RP2350
#define NUM_INPUT_CHANNELS   8   // max; active = 2/4/6/8 by USB alt
#define NUM_OUTPUT_CHANNELS  9   // 8 S/PDIF + 1 PDM
#else
#define NUM_INPUT_CHANNELS   2   // USB L/R
#define NUM_OUTPUT_CHANNELS  5   // 4 S/PDIF + 1 PDM
#endif

// Active input count for non-multichannel sources (stereo USB / S/PDIF / I2S).
// Inputs at index >= NUM_STEREO_INPUTS carry audio only in a multichannel USB
// alt (4/6/8).  Used as the stereo/multichannel threshold and the size of the
// extra-input buffers.
#define NUM_STEREO_INPUTS    2

#define NUM_CHANNELS  (NUM_INPUT_CHANNELS + NUM_OUTPUT_CHANNELS)

// Output channel indices begin right after the inputs.  CH_OUT_1 = first output
// (S/PDIF 1 L); CH_OUT_SUB = last output (PDM sub).
#define CH_OUT_1     (NUM_INPUT_CHANNELS + 0)   // S/PDIF 1 L
#define CH_OUT_2     (NUM_INPUT_CHANNELS + 1)   // S/PDIF 1 R
#define CH_OUT_3     (NUM_INPUT_CHANNELS + 2)   // S/PDIF 2 L
#define CH_OUT_4     (NUM_INPUT_CHANNELS + 3)   // S/PDIF 2 R
#if PICO_RP2350
#define CH_OUT_5     (NUM_INPUT_CHANNELS + 4)   // S/PDIF 3 L
#define CH_OUT_6     (NUM_INPUT_CHANNELS + 5)   // S/PDIF 3 R
#define CH_OUT_7     (NUM_INPUT_CHANNELS + 6)   // S/PDIF 4 L
#define CH_OUT_8     (NUM_INPUT_CHANNELS + 7)   // S/PDIF 4 R
#define CH_OUT_9_PDM (NUM_INPUT_CHANNELS + 8)   // PDM sub
#else
#define CH_OUT_5_PDM (NUM_INPUT_CHANNELS + 4)   // PDM sub
#endif
#define CH_OUT_SUB   (NUM_CHANNELS - 1)         // PDM sub (last output)

// Legacy aliases
#define CH_OUT_LEFT      CH_OUT_1
#define CH_OUT_RIGHT     CH_OUT_2

#define MAX_BANDS        12

// Crossover filter bands per OUTPUT channel. Crossover bands live at wire band
// indices [XOVER_BAND_BASE .. XOVER_BAND_BASE + MAX_XOVER_BANDS - 1]
// (i.e. 20..23) for vendor command addressing; see
// Documentation/Features/crossover_filters_spec.md.  Input channels have no
// crossover (their xover rows are unused).
//
// MAX_BANDS (12) is the PEQ storage depth; only bands 0..9 are active today.
// XOVER_BAND_BASE is deliberately set well above MAX_BANDS so the indices
// between active PEQ and crossover form a wide reserved gap: PEQ can grow to
// 20 bands (storage permitting) before it would ever collide with crossover.
// Band indices in [channel_band_counts[ch] .. XOVER_BAND_BASE) are rejected
// by the vendor handlers today.
//
// NOTE: REQ_GET_EQ_PARAM packs the band index into a 5-bit wValue field
// (bits[7:3]); XOVER_BAND_BASE + MAX_XOVER_BANDS - 1 (= 23) must stay <= 31.
#define XOVER_BAND_BASE   20
#define MAX_XOVER_BANDS   4
#define TOTAL_BAND_INDICES (XOVER_BAND_BASE + MAX_XOVER_BANDS)  // 24

// Core 1 Operating Mode
typedef enum {
    CORE1_MODE_IDLE      = 0,
    CORE1_MODE_PDM       = 1,
    CORE1_MODE_EQ_WORKER = 2,
} Core1Mode;

// Outputs assigned to Core 1 EQ worker
#if PICO_RP2350
#define CORE1_EQ_FIRST_OUTPUT  2
#define CORE1_EQ_LAST_OUTPUT   7   // S/PDIF pairs 2-4 = outputs 2-7
#else
#define CORE1_EQ_FIRST_OUTPUT  2
#define CORE1_EQ_LAST_OUTPUT   3   // S/PDIF pair 2 = outputs 2-3
#endif

// Core 1 EQ Worker Handshake
//
// vol_mul_start / vol_mul_step encode a per-sample linear ramp of the
// master-scaled output volume across the packet.  Sample i is multiplied by
// (vol_mul_start + i * vol_mul_step), giving click-free transitions when the
// host adjusts USB volume, master volume, or mute. Both cores apply the same
// ramp parameters so output slots stay phase-aligned (CLAUDE.md hard rule).
typedef struct {
    volatile bool     work_ready;
    volatile bool     work_done;
#if PICO_RP2350
    float           (*buf_out)[192];   // Pointer to buf_out array, set once at init
    uint32_t          sample_count;
    float             vol_mul_start;   // Master-scaled vol at sample 0
    float             vol_mul_step;    // Per-sample increment
    uint32_t          delay_write_idx;  // Snapshot for Core 1 delay processing
    int32_t          *spdif_out[3];     // Pairs 1-3 output buffers (NULL = skip)
    // ADAT-active snapshot for THIS packet: both cores must pick the same
    // slot finalization mode (in-place S24 staging vs fused convert+
    // interleave) so ADAT never reads unconverted rows.  See output_s24.h.
    uint8_t           finalize_s24;
#else
    int32_t         (*buf_out)[192];   // Pointer to buf_out array (Q28), set once at init
    uint32_t          sample_count;
    int32_t           vol_mul_start;   // Q15 master-scaled vol at sample 0
    int32_t           vol_mul_step;    // Q15 per-sample increment
    uint32_t          delay_write_idx;
    int32_t          *spdif_out[1];    // SPDIF pair 2 output buffer (NULL = skip)
#endif
    // Loudness snapshot for THIS packet; Core 0 samples the coefficient
    // pointer and output mask once and both cores use the same view, so a
    // mid-packet vendor write can't split the outputs across two settings.
    const void       *loud_coeffs;     // LoudnessCoeffs[2] or NULL = off
    uint16_t          loud_mask;       // Bit k = compensate output k
    // Crossfeed snapshot for THIS packet; same single-view rationale.
    const void       *xfeed_coeffs;    // CrossfeedCoeffs or NULL = off
    uint8_t           xfeed_mask;      // Bit p = crossfeed output pair p
    // Psychoacoustic bass snapshot for THIS packet; same single-view rationale.
    const void       *psybass_coeffs;  // PsybassCoeffs or NULL = off
    uint16_t          psybass_mask;    // Bit k = process output k
} Core1EqWork;

// ----------------------------------------------------------------------------
// DATA STRUCTURES
// ----------------------------------------------------------------------------

// Matrix Mixer Crosspoint
typedef struct __attribute__((packed)) {
    uint8_t enabled;        // Route active
    uint8_t phase_invert;   // Polarity flip
    uint8_t reserved[2];
    float gain_db;          // -inf to +12dB
    float gain_linear;      // Pre-computed multiplier
} MatrixCrosspoint;

// Output Channel State
typedef struct __attribute__((packed)) {
    uint8_t enabled;        // Output active (saves CPU when disabled)
    uint8_t mute;           // Soft mute
    uint8_t reserved[2];
    float gain_db;          // Per-output gain (-inf to +12dB)
    float gain_linear;      // Pre-computed
    float delay_ms;         // Per-output delay
    int32_t delay_samples;  // Pre-computed from sample rate
} OutputChannel;

// Matrix Mixer
typedef struct {
    MatrixCrosspoint crosspoints[NUM_INPUT_CHANNELS][NUM_OUTPUT_CHANNELS];
    OutputChannel outputs[NUM_OUTPUT_CHANNELS];
} MatrixMixer;

// Matrix Route Packet (for vendor commands)
typedef struct __attribute__((packed)) {
    uint8_t input;          // 0-1 (USB L/R); 0-7 on RP2350 8-channel USB mode
    uint8_t output;         // 0-8
    uint8_t enabled;        // 0 or 1
    uint8_t phase_invert;   // 0 or 1
    float gain_db;          // -inf to +12dB
} MatrixRoutePacket;

#if PICO_RP2350
typedef struct {
    // Biquad coefficients
    float b0, b1, b2, a1, a2;
    float s1, s2;                              // single-precision state (was double)

    // SVF coefficients and state
    float sva1, sva2, sva3;                    // integrator coefficients
    float svm0, svm1, svm2;                    // output mix coefficients
    float svic1eq, svic2eq;                    // integrator state
    uint32_t svf_type;                         // FilterType enum for inner loop specialization

    bool use_svf;                              // true = SVF path, false = biquad path
    bool svf_first_order;                      // true = one-pole SVF inner loop (1st-order types)
    bool bypass;
} Biquad;
#else
typedef struct {
    int32_t b0, b1, b2, a1, a2;
    int32_t s1, s2;
    bool bypass;
} Biquad;
#endif

// FilterType value-space contract.  These values are persisted in flash
// (PresetSlot.filter_recipes/.xover_recipes) and sent over USB (WireBandParams,
// REQ_SET/GET_EQ_PARAM), so NEVER renumber an existing value without bumping
// SLOT_DATA_VERSION + WIRE_FORMAT_VERSION and providing a migration (see
// remap_filter_type_pre_v18() in flash_storage.c).  The space is partitioned:
//
//     0..7    PEQ types        FILTER_FLAT .. FILTER_ALLPASS
//     8       FILTER_ALLPASS1  first-order all-pass
//     9..10   FILTER_LOWSHELF1, FILTER_HIGHSHELF1  first-order shelves
//     11      FILTER_LINKWITZ_TRANSFORM  pole/zero bass-extension biquad
//     12..31  reserved for future PEQ types (the PEQ block is everything
//             below FILTER_XOVER_FIRST; see filter_is_peq_type())
//     32..63  crossover types  FILTER_XOVER_FIRST .. FILTER_XOVER_LAST
//     64..    reserved for future crossover types (they must stay contiguous
//             from FILTER_XOVER_FIRST; xover_type_table[] is indexed by
//             (type - FILTER_XOVER_FIRST))
enum FilterType {
    FILTER_FLAT = 0, FILTER_PEAKING = 1, FILTER_LOWSHELF = 2,
    FILTER_HIGHSHELF = 3, FILTER_LOWPASS = 4, FILTER_HIGHPASS = 5,
    FILTER_NOTCH = 6, FILTER_ALLPASS = 7,

    // First-order all-pass: flat magnitude, phase 0 → -180° (-90° at the
    // corner freq), single parameter (freq); Q and gain are unused.
    // FILTER_ALLPASS (7) is the second-order RBJ all-pass; this is the 1st-order.
    FILTER_ALLPASS1 = 8,

    // First-order shelving filters: gentle 6 dB/oct shelf, monotonic (no Q),
    // single corner freq + gain.  Like ALLPASS1 these are genuine 1st-order
    // sections; on RP2350 they follow the hybrid SVF/biquad path (one-pole SVF
    // below Fs/7.5, degenerate biquad above), on RP2040 a Q28 degenerate biquad.
    // Q is unused.  First-order shelves prewarp g by A (not sqrt(A) like the
    // 2nd-order shelves).
    FILTER_LOWSHELF1 = 9, FILTER_HIGHSHELF1 = 10,

    // Linkwitz Transform: pole/zero biquad that replaces a driver's measured
    // 2nd-order rolloff (f0, Q0) with a target alignment (fp, Qp).  Wire
    // mapping: freq = f0, Q = Q0, gain_db carries fp in Hz (gain is unused by
    // this type), Qp travels separately as uint16 Q*512 (0 = 0.707 default);
    // see peq_qp_x512[] and Documentation/Features/peq_filters.md.
    FILTER_LINKWITZ_TRANSFORM = 11,

    // 12..31 reserved for future PEQ types.

    // Crossover filter types — indices 32..63. See crossover.h /
    // Documentation/Features/crossover_filters_spec.md for semantics.
    // Each value encodes (family, order, LP/HP); section count per filter is
    // ceil(order/2) for BW/Bes and order/2 for LR.  MUST stay contiguous from
    // FILTER_XOVER_FIRST (xover_type_table[] is indexed relative to it).
    FILTER_LR2_LP   = 32, FILTER_LR2_HP   = 33,
    FILTER_LR4_LP   = 34, FILTER_LR4_HP   = 35,
    FILTER_LR6_LP   = 36, FILTER_LR6_HP   = 37,
    FILTER_LR8_LP   = 38, FILTER_LR8_HP   = 39,

    FILTER_BW1_LP   = 40, FILTER_BW1_HP   = 41,
    FILTER_BW2_LP   = 42, FILTER_BW2_HP   = 43,
    FILTER_BW3_LP   = 44, FILTER_BW3_HP   = 45,
    FILTER_BW4_LP   = 46, FILTER_BW4_HP   = 47,
    FILTER_BW5_LP   = 48, FILTER_BW5_HP   = 49,
    FILTER_BW6_LP   = 50, FILTER_BW6_HP   = 51,
    FILTER_BW7_LP   = 52, FILTER_BW7_HP   = 53,
    FILTER_BW8_LP   = 54, FILTER_BW8_HP   = 55,

    FILTER_BES2_LP  = 56, FILTER_BES2_HP  = 57,
    FILTER_BES4_LP  = 58, FILTER_BES4_HP  = 59,
    FILTER_BES6_LP  = 60, FILTER_BES6_HP  = 61,
    FILTER_BES8_LP  = 62, FILTER_BES8_HP  = 63,

    FILTER_XOVER_FIRST = FILTER_LR2_LP,
    FILTER_XOVER_LAST  = FILTER_BES8_HP,
};

// Single source of truth for PEQ-vs-crossover classification: a PEQ type is
// any value in the PEQ block (everything below the crossover range).  Used by
// is_filter_flat() (dsp_pipeline.c) to flatten a crossover type that leaks
// into a PEQ band slot.  Routing PEQ vs crossover bands themselves is by BAND
// index (XOVER_BAND_BASE), independent of this.
static inline bool filter_is_peq_type(uint8_t t) { return t < FILTER_XOVER_FIRST; }

_Static_assert(FILTER_ALLPASS  < FILTER_XOVER_FIRST, "core PEQ block must precede the crossover range");
_Static_assert(FILTER_ALLPASS1 < FILTER_XOVER_FIRST, "first-order all-pass must sit in the PEQ block");
_Static_assert(FILTER_HIGHSHELF1 < FILTER_XOVER_FIRST, "first-order shelves must sit in the PEQ block");
_Static_assert(FILTER_LINKWITZ_TRANSFORM < FILTER_XOVER_FIRST, "Linkwitz Transform must sit in the PEQ block");

typedef struct __attribute__((packed)) {
    uint8_t channel;
    uint8_t band;
    uint8_t type;
    uint8_t bypass;     // User bypass: exactly 1 = bypassed; any other value (0, 0xFF, …) = active
    float freq;
    float Q;
    float gain_db;
} EqParamPacket;

typedef struct {
    uint16_t peaks[NUM_CHANNELS];
    uint8_t cpu0_load;
    uint8_t cpu1_load;
    uint32_t clip_flags;         // Per-channel clip latch bitmask (sticky, cleared by REQ_CLEAR_CLIPS).
                                 // 32-bit: NUM_CHANNELS can exceed 16 (17 on RP2350: 8 in + 9 out).
} SystemStatusPacket;

// ----------------------------------------------------------------------------
// VENDOR COMMAND PACKET STRUCTURES
// ----------------------------------------------------------------------------

typedef struct __attribute__((packed)) {
    uint8_t cmd;
    uint8_t channel;
    uint8_t band;
    uint8_t reserved;
    uint8_t data[60];
} VendorCmdPacket;

typedef struct __attribute__((packed)) {
    uint8_t cmd;
    uint8_t result;
    uint8_t reserved[2];
    union {
        struct __attribute__((packed)) {
            uint16_t peaks[NUM_CHANNELS];
            uint8_t cpu0_load;
            uint8_t cpu1_load;
        } status;
        EqParamPacket eq_param;
        float preamp_db;
        float delay_ms;
        uint8_t bypass;
        uint8_t raw[60];
    };
} VendorRespPacket;

// Buffer Statistics Wire Format — fits in a single 64-byte control transfer
typedef struct __attribute__((packed)) {
    uint8_t consumer_free;       // [0-4] SPDIF buffers available for DMA
    uint8_t consumer_prepared;   // [0-4] SPDIF buffers queued for DMA playback
    uint8_t consumer_playing;    // [0-1] DMA in-flight
    uint8_t consumer_fill_pct;   // Consumer pipeline fill %
    uint8_t consumer_min_fill_pct; // Lowest consumer fill since last reset
    uint8_t consumer_max_fill_pct; // Highest consumer fill since last reset
    uint8_t pad[2];
} SpdifBufferStats;              // 8 bytes

typedef struct __attribute__((packed)) {
    uint8_t dma_fill_pct;        // DMA circular buffer fill %
    uint8_t dma_min_fill_pct;
    uint8_t dma_max_fill_pct;
    uint8_t ring_fill_pct;       // Software ring buffer fill %
    uint8_t ring_min_fill_pct;
    uint8_t ring_max_fill_pct;
    uint8_t pad[2];
} PdmBufferStats;                // 8 bytes

typedef struct __attribute__((packed)) {
    uint8_t num_spdif;           // NUM_SPDIF_INSTANCES (2 or 4)
    uint8_t flags;               // Bit 0: PDM active, Bit 1: audio streaming
    uint16_t sequence;           // Monotonic counter, wraps at 65535
    SpdifBufferStats spdif[4];   // Per-instance (unused zeroed)
    PdmBufferStats pdm;
} BufferStatsPacket;             // 4 + 32 + 8 = 44 bytes

extern uint8_t channel_band_counts[NUM_CHANNELS];
extern volatile SystemStatusPacket global_status;

// ----------------------------------------------------------------------------
// Force time-critical functions into RAM. Under the XIP build, .time_critical
// placement is the load-bearing RAM-residency mechanism on both platforms;
// without it these functions would run from flash and stall the audio path.
// ----------------------------------------------------------------------------
#define DSP_TIME_CRITICAL __attribute__((section(".time_critical")))

// ----------------------------------------------------------------------------
// UTILS
// ----------------------------------------------------------------------------

#if !PICO_RP2350
static inline int32_t clip_s32(int32_t x) {
    if (x > INT32_MAX) return INT32_MAX;
    if (x < INT32_MIN) return INT32_MIN;
    return x;
}

static inline int32_t clip_s64_to_s32(int64_t x) {
    if (x > INT32_MAX) return INT32_MAX;
    if (x < INT32_MIN) return INT32_MIN;
    return (int32_t)x;
}

static inline int32_t clip_s24(int32_t x) {
    if (x > 0x7FFFFF) return 0x7FFFFF;
    if (x < -0x800000) return -0x800000;
    return x;
}

// Q15 fixed-point multiply using 16-bit partial products.
// Computes (sample * gain) >> 15 without 64-bit library call.
// Exact when the final result fits in int32 (always true for valid audio).
static inline int32_t fast_mul_q15(int32_t sample, int32_t gain) {
    int32_t sh = sample >> 16;
    uint32_t sl = (uint16_t)sample;
    int32_t gh = gain >> 16;
    uint32_t gl = (uint16_t)gain;

    int32_t hh = sh * gh;
    int32_t mid = sh * (int32_t)gl + (int32_t)sl * gh;
    uint32_t ll = sl * gl;

    return (int32_t)(((uint32_t)hh << 17) + ((uint32_t)mid << 1) + (ll >> 15));
}
#endif

#endif // CONFIG_H
