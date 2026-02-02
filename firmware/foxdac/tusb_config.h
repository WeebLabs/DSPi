/*
 * TinyUSB Configuration for DSPi USB Audio Device
 * Migrated from pico-extras usb_device for RP2350 compatibility
 */

#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------

// Defined by compiler flags for RP2040/RP2350
#ifndef CFG_TUSB_MCU
#error CFG_TUSB_MCU must be defined (set by pico-sdk)
#endif

#define CFG_TUSB_OS             OPT_OS_PICO

// Enable TinyUSB debug output (0=off, 1=errors, 2=warnings, 3=info)
#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG          2
#endif

// Memory alignment
#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN      __attribute__ ((aligned(4)))

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------

#define CFG_TUD_ENABLED         1

// High-speed not needed for USB Audio 1.0
#define CFG_TUD_MAX_SPEED       OPT_MODE_FULL_SPEED

#define CFG_TUD_EP0_SIZE        64

// Use board-provided USB port
#ifndef BOARD_TUD_RHPORT
#define BOARD_TUD_RHPORT        0
#endif

//--------------------------------------------------------------------
// CLASS DRIVER CONFIGURATION
//--------------------------------------------------------------------

// Enable Audio and Vendor classes
#define CFG_TUD_AUDIO           1
#define CFG_TUD_VENDOR          1

// Disable unused classes
#define CFG_TUD_CDC             0
#define CFG_TUD_MSC             0
#define CFG_TUD_HID             0
#define CFG_TUD_MIDI            0
#define CFG_TUD_BTH             0
#define CFG_TUD_ECM_RNDIS       0
#define CFG_TUD_NCM             0
#define CFG_TUD_DFU             0
#define CFG_TUD_DFU_RUNTIME     0
#define CFG_TUD_NET             0

//--------------------------------------------------------------------
// AUDIO CLASS CONFIGURATION (UAC1)
//--------------------------------------------------------------------

// Audio function descriptor length (including IAD)
// Must match TUD_AUDIO_SPEAKER_STEREO_FB_DESC_LEN in usb_descriptors.c
// IAD(8) + AC_STD(9) + AC_CS(9) + CLK(8) + INPUT(17) + OUTPUT(12) + FEATURE_2CH(18)
// + AS_STD_ALT0(9) + AS_STD_ALT1(9) + AS_CS(16) + FORMAT(6) + EP_STD(7) + EP_CS(8) + FB_EP(7)
// = 8+9+9+8+17+12+18 + 9+9+16+6+7+8+7 = 143
#define CFG_TUD_AUDIO_FUNC_1_DESC_LEN                   143

// Number of audio control interfaces (we have 1)
#define CFG_TUD_AUDIO_FUNC_1_N_AC_INT                   1

// Number of Standard AS Interface Descriptors (4.9.1) defined per audio function
// We have 1 streaming interface with 2 alternate settings (alt 0 = idle, alt 1 = active)
#define CFG_TUD_AUDIO_FUNC_1_N_AS_INT                   1

// Number of audio streaming endpoints per audio function
#define CFG_TUD_AUDIO_FUNC_1_N_EP_OUT                   1
#define CFG_TUD_AUDIO_FUNC_1_N_EP_IN                    0

// Enable feedback endpoint for asynchronous mode
#define CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP                1

// Enable endpoint OUT flow control (required for speaker)
#define CFG_TUD_AUDIO_ENABLE_EP_OUT                     1

// Audio format configuration
#define CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX      2   // Not used (no TX)
#define CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_RX      2   // 16-bit samples
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX              0   // No TX
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX              2   // Stereo

// FIFO sizes - sized for maximum packet size at 48kHz
// Max packet: 48 samples/ms * 2 channels * 2 bytes = 192 bytes
// Add headroom for jitter: 192 * 2 = 384 bytes per FIFO entry
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ           (384 * 4)  // 4 packets worth of buffer
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX              384        // Max packet size

// Feedback endpoint format correction
#define CFG_TUD_AUDIO_ENABLE_FEEDBACK_FORMAT_CORRECTION 1

// Control request buffer
#define CFG_TUD_AUDIO_FUNC_1_CTRL_BUF_SZ                64

//--------------------------------------------------------------------
// VENDOR CLASS CONFIGURATION
//--------------------------------------------------------------------

// Vendor class uses control transfers only (no bulk endpoints needed)
// These defines are for bulk endpoints which we don't use, but TinyUSB may require them
#define CFG_TUD_VENDOR_EPSIZE                           64
#define CFG_TUD_VENDOR_EP_BUFSIZE                       64
#define CFG_TUD_VENDOR_RX_BUFSIZE                       64
#define CFG_TUD_VENDOR_TX_BUFSIZE                       64

#ifdef __cplusplus
}
#endif

#endif /* TUSB_CONFIG_H */
