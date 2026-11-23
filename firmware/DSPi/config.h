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

#ifdef DSPI_PLATFORM_PINS
#include "platform_pins.h"      // the platform's default pins in place of the blocks marked below
#else
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
#endif  // !DSPI_PLATFORM_PINS

// Legacy aliases
#define PICO_AUDIO_SPDIF_SUB_PIN PICO_PDM_PIN

// Clip detection threshold — slightly above 1.0 to avoid false positives
// from float precision noise when 0 dBFS signals pass through biquad filters.
// +0.01 dB headroom: catches audible clipping, ignores float rounding artifacts.
#define CLIP_THRESH_F   1.001f              // RP2350 float path
// RP2040 samples reach full scale at 2^29 (Q28 value 2.0), not 1 << 28.
// See "RP2040 Sample Scale" in current_architecture.md.
#define SAMPLE_FULL_SCALE_Q28 (1 << 29)
#define CLIP_THRESH_Q28 (SAMPLE_FULL_SCALE_Q28 + SAMPLE_FULL_SCALE_Q28 / 1000)

#define FILTER_SHIFT 28

// PDM Configuration
#define PDM_OVERSAMPLE 256
#define PDM_DMA_BUFFER_SIZE 2048  // Doubled for more margin (was 1024)
#define PDM_DMA_RING_BITS 13      // log2(2048 * 4 bytes) = 13
#define PDM_PIO pio1
#define PDM_SM 0
#define PDM_FULL_SCALE 29500   // 0 dBFS lands here: ~90% modulation, the stable ceiling

// PDM Sigma-Delta Tuning
// Dither mask: controls TPDF amplitude. Start small (0x1FF), increase if idle tones persist
#define PDM_DITHER_MASK 0x1FF
// Leakage shift: higher = less leakage. Applied once per audio sample.
// 16 gives ~1.4s time constant at 48kHz, safe for bass
#define PDM_LEAKAGE_SHIFT 16

// PDM Soft Start/Stop
#define PDM_FADE_IN_SHIFT   10                        // 2^10 = 1024 samples
#define PDM_FADE_IN_SAMPLES (1u << PDM_FADE_IN_SHIFT) // ~21ms at 48kHz
#define PDM_HOLD_TIMEOUT_US 100000u   // pdm_hold_hw: covers the fade-out at 44.1 kHz with margin

// SPDIF Buffer Configuration
// Slot ring fill scale in 48-frame buckets (16 = 768 frames): the unit of
// SYS.BUFFER_STATS and of the clock servos' fill error and gains. The rings
// themselves track fill to the frame; buckets survive for wire compatibility.
#define SPDIF_CONSUMER_BUFFER_COUNT 16
#define SLOT_FILL_FRAMES_PER_BUCKET 48u

// Pipeline block buffers. The steady-state ring fill and the block size producers
// deliver come from the selected output latency option (output_latency.h).
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

// Sub alignment: the PDM sub is delayed by the ring fill target minus this.
// Known defect: PDM's real latency is not its DMA ring size, so the offset is
// miscalibrated; see current_architecture.md PDM "Known alignment defect".
#define PDM_BUFFER_SAMPLES    (PDM_DMA_BUFFER_SIZE / 8)                           // 256

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
// set. Reserved for Microsoft; 0x00-0x0F are never commands on any page.
//
// Note: 0x01 also happens to be the value of UAC1_REQ_SET_CUR
// (usb_descriptors.h). They cannot collide because UAC1 SET_CUR is
// TUSB_REQ_TYPE_CLASS and reaches the audio class handler in usb_audio.c,
// while MS OS 2.0 requests are TUSB_REQ_TYPE_VENDOR and reach
// tud_vendor_control_xfer_cb in vendor/vc_usb.c.
#define MS_VENDOR_CODE      0x01

// Master Volume Constants
#define MASTER_VOL_MUTE_DB          (-128.0f)  // Sentinel value: true -inf (mute)
#define MASTER_VOL_MIN_DB           (-127.0f)  // Minimum non-mute attenuation
#define MASTER_VOL_MAX_DB           (0.0f)     // Unity gain (no attenuation)
#define MASTER_VOL_DEFAULT_DB       (-20.0f)   // Power-on / fresh-device default

// Master Volume Persistence Modes
// Mode 0 (default): master volume is independent of presets. ROUTE.MASTER_VOLUME_SAVE
//   stores it in the directory sector; it's applied at boot and on factory reset.
//   Preset save/load does not touch it.
// Mode 1: master volume is part of the preset. Saved with the preset, restored on
//   preset load.
#define MASTER_VOLUME_MODE_INDEPENDENT   0
#define MASTER_VOLUME_MODE_WITH_PRESET   1

// Output (Physical IO) Configuration Persistence Modes
// Governs the device's physical IO/routing: output pins, output types (SPDIF/I2S),
// I2S MCK/BCK, and the SPDIF RX pin.  Same 0/1 convention as the master-volume
// modes, so the include_pins directory byte (default 1) maps straight onto it.
// Mode 1 (WITH_PRESET, default): IO config travels with presets, stored in each
//   slot and applied on load.
// Mode 0 (INDEPENDENT): IO config is device-global.  STORE.OUTPUT_CONFIG_SAVE stores
//   it in the directory; it is applied at boot and preset load/factory reset never
//   touch it.  Mirrors the master-volume independent mode.
#define OUTPUT_CONFIG_MODE_INDEPENDENT   0
#define OUTPUT_CONFIG_MODE_WITH_PRESET   1


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

// XPORT.STATUS reply.  last_status is the result of the most recent
// XPORT.UART_CONFIG or XPORT.I2C_CONFIG SET (PIN_CONFIG_* code); live reflects whether the
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
#ifndef DSPI_PLATFORM_PINS
#define UART_CTRL_DEFAULT_TX_PIN    16
#define UART_CTRL_DEFAULT_RX_PIN    17
#endif
#define UART_CTRL_DEFAULT_BAUD      115200u
#define UART_CTRL_BAUD_MIN          9600u
#define UART_CTRL_BAUD_MAX          1000000u
#ifndef DSPI_PLATFORM_PINS
#define I2C_CTRL_DEFAULT_SDA_PIN    18
#define I2C_CTRL_DEFAULT_SCL_PIN    19
#endif
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

// Firmware version.  The packed form squeezes minor and patch into one nibble
// each (plain nibble packing, not BCD; no carry), so each caps at 15.  It only
// feeds byte 2 of SYS.PLATFORM; see
// Documentation/Features/firmware_versioning_spec.md.
#define FW_VERSION_MAJOR            1
#define FW_VERSION_MINOR            1
#define FW_VERSION_PATCH            7
#define FW_VERSION_PACKED           ((FW_VERSION_MAJOR << 8) | (FW_VERSION_MINOR << 4) | FW_VERSION_PATCH)

// Pre-release ordinal: 0 = final release, 1..255 = beta N of this patch.  Betas
// share their patch number, so this byte is the only thing telling two of them
// apart; it must be zeroed in the same commit that tags the final release.
#define FW_VERSION_BETA             1

// Universal "reset to default" escape hatch for every single-pin SET command
// (OUTPUT.PIN, CLOCK.I2S_BCK_PIN, CLOCK.MCK_PIN, OUTPUT.ADAT_TX_PIN,
// INPUT.SPDIF_RX_PIN, INPUT.ADAT_RX_PIN, INPUT.I2S_RX_PIN).  Sending
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

#ifndef DSPI_PLATFORM_PINS
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
//           is_valid_gpio_pin() in vendor/vc_pins.c.
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
#endif  // !DSPI_PLATFORM_PINS

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

// Output slot rings: RP2040 lacks ENDLESS-mode DMA, so each slot's
// free-running ring needs a reload partner channel (slot i uses
// OUTPUT_SLOT_RELOAD_DMA_BASE + i). RP2350 uses ENDLESS mode, no partner.
#if !PICO_RP2350
#define OUTPUT_SLOT_RELOAD_DMA_BASE 6
#endif

// Clock / audio-path diagnostics (clock_diag.c and the statics it reads in
// soft_vcxo.c, input_servo.c, spdif_input.c): set by the DSPI_CLOCK_DIAG CMake
// option, compiled out otherwise.
#ifndef DSPI_CLOCK_DIAG
#define DSPI_CLOCK_DIAG            0
#endif

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
// NOTE: a band index travels as one byte (the low byte of FILTER.BAND's
// address, the CS binding index), so TOTAL_BAND_INDICES must stay <= 256.
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
    int32_t          *spdif_out[3];     // Pairs 1-3 slot staging (interleaved S24)
    // ADAT-active snapshot for THIS packet: both cores must pick the same
    // slot finalization mode (in-place S24 staging vs fused convert+
    // interleave) so ADAT never reads unconverted rows.  See output_s24.h.
    uint8_t           finalize_s24;
    uint8_t           silent_pairs;     // Core 1 -> Core 0: pairs whose staging it zeroed
#else
    int32_t         (*buf_out)[192];   // Pointer to buf_out array (Q28), set once at init
    uint32_t          sample_count;
    int32_t           vol_mul_start;   // Q15 master-scaled vol at sample 0
    int32_t           vol_mul_step;    // Q15 per-sample increment
    uint32_t          delay_write_idx;
    int32_t          *spdif_out[1];    // Pair 2 slot staging (interleaved S24)
#endif
    // Loudness snapshot for THIS packet; Core 0 samples the coefficient
    // pointer and output mask once and both cores use the same view, so a
    // mid-packet vendor write can't split the outputs across two settings.
    const void       *loud_coeffs;     // LoudnessCoeffs[2] or NULL = off
    uint16_t          loud_mask;       // Bit k = compensate output k
    // Crossfeed snapshot for THIS packet; same single-view rationale.
    const void       *xfeed_coeffs;    // CrossfeedCoeffs or NULL = off
    uint8_t           xfeed_mask;      // Bit p = crossfeed output pair p
    // Spatializer snapshot for THIS packet; same single-view rationale.
    const void       *spatial_coeffs;  // SpatialCoeffs (never NULL once published)
    uint8_t           spatial_mask;    // Bit p = spatializer output pair p
    // Psychoacoustic bass snapshot for THIS packet; same single-view rationale.
    const void       *psybass_coeffs;  // PsybassCoeffs or NULL = off
    uint16_t          psybass_mask;    // Bit k = process output k
    // Subharmonic synthesizer snapshot for THIS packet; same single-view rationale.
    const void       *subharm_coeffs;  // SubharmCoeffs or NULL = off
    uint16_t          subharm_mask;    // Bit k = process output k
    uint8_t           subharm_flags;   // SUBHARM_FLAG_* (link pairs, solo)
    uint8_t           subharm_phase;   // decimation phase at packet start
    // Tube preamp snapshot for THIS packet; same single-view rationale.
    const void       *tube_coeffs;     // TubeCoeffs or NULL = off
    uint16_t          tube_mask;       // Bit k = process output k
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

#if PICO_RP2350
typedef struct {
    // Biquad coefficients
    float b0, b1, b2, a1, a2;
    float s1, s2;                              // single-precision state (was double)

    // SVF coefficients and state
    float sva1, sva2;                          // integrator coefficients
    float svm0, svm1, svm2;                    // output mix coefficients
    float svic1eq, svic2eq;                    // integrator state
    float g;                                   // frequency warped cutoff frequency

    uint32_t filter_type;                      // FilterType enum for inner loop specialization
    bool use_svf;                              // true = SVF path, false = biquad path
    bool first_order;                          // true = filter type is first order
    bool bypass;
} Filter;
#else
typedef struct {
    int32_t b0, b1, b2, a1, a2;
    int32_t s1, s2;
    bool bypass;
} Filter;
#endif

// FilterType value-space contract.  These values are persisted in flash
// (PresetSlot.filter_recipes/.xover_recipes) and sent over USB (WireBandParams,
// FILTER.BAND), so NEVER renumber an existing value without bumping
// SLOT_DATA_VERSION + WIRE_FORMAT_VERSION and providing a migration (see
// remap_filter_type_pre_v18() in flash_storage.c).  The space is partitioned:
//
//     0..7    PEQ types        FILTER_FLAT .. FILTER_ALLPASS
//     8       FILTER_ALLPASS1  first-order all-pass
//     9..10   FILTER_LOWSHELF1, FILTER_HIGHSHELF1  first-order shelves
//     11      FILTER_LINKWITZ_TRANSFORM  pole/zero bass-extension biquad
//     12..13  FILTER_LOWPASS1, FILTER_HIGHPASS1  first-order low/high pass
//     14..31  reserved for future PEQ types (the PEQ block is everything
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

    // First-order low/high pass: single-pole 6 dB/oct rolloff, -3 dB at the
    // corner, no resonance.  Single parameter (freq); Q and gain are unused.
    // Same hybrid rule as the other 1st-order types: one-pole SVF below
    // Fs/7.5, degenerate biquad above (and always on RP2040).
    FILTER_LOWPASS1 = 12,
    FILTER_HIGHPASS1 = 13,

    // 14..31 reserved for future PEQ types.

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
    uint32_t clip_flags;         // Per-channel clip latch bitmask (sticky, cleared by MEAS.CLIP_CLEAR).
                                 // 32-bit: NUM_CHANNELS can exceed 16 (17 on RP2350: 8 in + 9 out).
} SystemStatusPacket;

// Buffer Statistics Wire Format — fits in a single 64-byte control transfer
typedef struct __attribute__((packed)) {
    uint8_t consumer_free;       // [0-16] free ring space, 48-frame buckets
    uint8_t consumer_prepared;   // [0-16] ring fill, 48-frame buckets
    uint8_t consumer_playing;    // [0-1] DMA in-flight
    uint8_t consumer_fill_pct;   // Consumer pipeline fill %
    uint8_t consumer_min_fill_pct; // Lowest consumer fill since last reset
    uint8_t consumer_max_fill_pct; // Highest consumer fill since last reset
    uint16_t fill_centi_pct;     // Sample-granular fill, 0.01% units on the
                                 // same 16-bucket scale (5000 = 50.00%).
                                 // Was pad[2]; zero on older firmware.
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

// OUTPUT.LATENCY_STATUS reply.  The effective option and its target depend on
// the active source and rate; the requested ID is what both re-derive from.
typedef struct __attribute__((packed)) {
    uint8_t  requested_id;       // stored OutputLatencyId
    uint8_t  effective_id;       // option in force (a stream-only request on USB differs)
    uint16_t effective_us;       // ring fill target at the current rate, ceiling applied
    uint16_t target_frames;      // live slot ring fill target
    uint16_t batch_frames;       // stream-input batch at the current rate
    uint8_t  pending;            // 1 until a SET has fully applied
    uint8_t  option_count;       // entries in the OUTPUT.LATENCY_OPTIONS reply
    uint16_t reserved;
} OutputLatencyStatus;           // 12 bytes

typedef struct __attribute__((packed)) {
    uint8_t  id;                 // OutputLatencyId
    uint8_t  flags;              // OUTPUT_LATENCY_FLAG_* | OUTPUT_LATENCY_WIRE_AVAILABLE
    uint16_t latency_us;         // nominal ring fill target
    uint16_t batch_us;           // nominal stream-input batch
    uint16_t effective_us;       // what selecting it gives now (source and rate applied)
} OutputLatencyOptionWire;       // 8 bytes
#define OUTPUT_LATENCY_WIRE_AVAILABLE  0x80u   // runs as itself on the active source

extern uint8_t channel_band_counts[NUM_CHANNELS];
extern volatile SystemStatusPacket global_status;

// ----------------------------------------------------------------------------
// Force time-critical functions into RAM. Under the XIP build, .time_critical
// placement is the load-bearing RAM-residency mechanism on both platforms;
// without it these functions would run from flash and stall the audio path.
// ----------------------------------------------------------------------------
#ifndef DSP_TIME_CRITICAL   // host test builds define it empty
#define DSP_TIME_CRITICAL __attribute__((section(".time_critical")))
#endif

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

// Same rounding as (x + 32) >> 6, without wrapping when x is near INT32_MAX.
static inline int32_t q28_to_s24(int32_t x) {
    return clip_s24((x >> 6) + ((x >> 5) & 1));
}

// Block peak to the 0..32767 meter scale RP2350 reports.
static inline uint16_t peak_to_meter_q28(int32_t pk) {
    return ((uint32_t)pk >= (uint32_t)SAMPLE_FULL_SCALE_Q28) ? 32767 : (uint16_t)(pk >> 14);
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
