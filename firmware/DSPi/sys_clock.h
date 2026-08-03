/*
 * sys_clock.h; selectable system clock (307.2 / 384 / 480 MHz).
 *
 * All three modes exist on both platforms.  The setting is device-global
 * (flash directory, not per-preset) and every mode keeps VCO/postdiv values
 * whose 48 kHz family SPDIF/I2S dividers are exact in 8.8 fixed point.
 *
 * Overclocked boots are protected by a watchdog-scratch breadcrumb: if a boot
 * under a non-default mode or custom voltage crashes inside the confirm
 * window, the next boot forces mode 0, reports it via
 * sys_clock_fallback_active(), and latches the fallback across warm reboots
 * (a power cycle or a new REQ_SET_SYS_CLOCK retries).  The persisted setting
 * is never rewritten by the fallback.
 */

#ifndef SYS_CLOCK_H
#define SYS_CLOCK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SYS_CLOCK_MODE_307P2 = 0,
    SYS_CLOCK_MODE_384   = 1,
    SYS_CLOCK_MODE_480   = 2,
    SYS_CLOCK_MODE_COUNT = 3,
} SysClockMode;

// Full boot-time path: reads the persisted mode, honors the crash breadcrumb,
// then sequences vreg + PLL.  Must run before anything claims a PIO/DMA
// divider, i.e. first thing in core0_init().
void sys_clock_boot_init(void);

// Clears the breadcrumb once the firmware has proven itself: call every main
// loop iteration; it no-ops until several seconds of fed-watchdog uptime have
// elapsed since the breadcrumb was armed.
void sys_clock_confirm_tick(void);

// Runtime switch.  Sequences vreg + PLL and re-arms the breadcrumb; the caller
// owns everything else (audio teardown/restart, divider rebuild, persistence).
// The confirm tick disarms the breadcrumb after the proving window.
void sys_clock_apply(SysClockMode m, uint8_t vreg_sel);

// Voltage-only change at the current mode (no PLL relock, no audio impact).
// Re-arms the breadcrumb; the confirm tick disarms it.
void sys_clock_apply_vreg_only(uint8_t vreg_sel);

SysClockMode sys_clock_active_mode(void);
bool sys_clock_fallback_active(void);
bool sys_clock_mode_valid(uint8_t m);

// A voltage selection is valid only at or above the mode's default (the knob
// trades voltage up for stability, never down) and at most 1.30 V; 0xFF
// always means "the mode's default".
bool sys_clock_vreg_valid(uint8_t mode, uint8_t vreg_sel);

// 0xFF or any invalid selection resolves to the mode's default voltage.
uint8_t sys_clock_resolve_vreg(SysClockMode m, uint8_t vreg_sel);

// Vendor-command staging for the deferred main-loop apply (REQ_SET_SYS_CLOCK).
// Written from the vendor dispatch (IRQ/transport context), consumed once by
// the main loop; sys_clock_set_pending is the release flag.
extern volatile uint8_t sys_clock_req_mode;
extern volatile uint8_t sys_clock_req_vreg;
extern volatile bool    sys_clock_set_pending;

#ifdef __cplusplus
}
#endif

#endif
