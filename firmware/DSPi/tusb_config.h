/*
 * TinyUSB configuration for DSPi (Phase 1: audio only, until verified working)
 *
 * We're not enabling TinyUSB's integrated UAC driver (CFG_TUD_AUDIO = 0)
 * as it rejects any AC interface whose bInterfaceProtocol != UAC2.
 * Instead, DSPi registers a minimal UAC1 class driver
 * via usbd_app_driver_get_cb() — check usb_audio.c.
 */

#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

// CFG_TUSB_MCU is defined by the SDK's tinyusb cmake per-platform.
#ifndef CFG_TUSB_MCU
#error "CFG_TUSB_MCU must be defined by the build system"
#endif

#define CFG_TUSB_OS             OPT_OS_PICO
#define CFG_TUSB_DEBUG          0

#define CFG_TUD_ENABLED         1
#define CFG_TUH_ENABLED         0

#define CFG_TUD_ENDPOINT0_SIZE  64

// No built-in classes are used in Phase 1.
// The UAC1 audio function is handled by a custom class driver registered via
// usbd_app_driver_get_cb() (see usb_audio.c).
#define CFG_TUD_AUDIO           0
#define CFG_TUD_CDC             0
#define CFG_TUD_MSC             0
#define CFG_TUD_HID             0
#define CFG_TUD_MIDI            0
#define CFG_TUD_VENDOR          0
#define CFG_TUD_DFU_RUNTIME     0
#define CFG_TUD_ECM_RNDIS       0
#define CFG_TUD_NCM             0
#define CFG_TUD_BTH             0

#ifdef __cplusplus
}
#endif

#endif /* _TUSB_CONFIG_H_ */
