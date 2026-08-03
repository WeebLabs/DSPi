#include "sys_clock.h"

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/watchdog.h"
#include "hardware/structs/watchdog.h"

#include "flash_clkdiv.h"
#include "flash_storage.h"

// Breadcrumb lives in watchdog scratch[1]: scratch[4..7] are the SDK's reboot
// vectors and scratch[2..3] carry bootrom reboot/usb-boot parameters, so
// neither is safe.  Scratch clears on power-on reset, survives warm resets.
#define SYS_CLOCK_BREADCRUMB_BASE  0xC1C0DE00u
#define SYS_CLOCK_BREADCRUMB(m)    (SYS_CLOCK_BREADCRUMB_BASE | (uint32_t)(m))
#define SYS_CLOCK_BREADCRUMB_MASK  0xFFFFFF00u
// Written instead of the breadcrumb on a fallback boot: keeps every warm
// reboot on the safe default until a power cycle or a new REQ_SET_SYS_CLOCK,
// preventing a crash-fallback-crash alternation loop.
#define SYS_CLOCK_FALLBACK_LATCH   0xC1C0FA11u
#define SYS_CLOCK_SCRATCH          (watchdog_hw->scratch[1])

// A boot or runtime switch is "proven" only after this much wall-clock main
// loop uptime with the watchdog fed; confirming on the first iteration would
// clear the breadcrumb long before USB enumeration or DSP load exist.
#define SYS_CLOCK_CONFIRM_UPTIME_US  (5u * 1000u * 1000u)

typedef struct {
    uint32_t sys_hz;
    uint32_t vco_hz;
    uint8_t  postdiv1;
    uint8_t  postdiv2;
    uint8_t  default_vreg;
} SysClockEntry;

// Every entry gives the 48 kHz family exactly representable 8.8 PIO
// dividers; changing a VCO/postdiv pair here breaks that invariant.
static const SysClockEntry sys_clock_table[SYS_CLOCK_MODE_COUNT] = {
    { 307200000u, 1536000000u, 5, 1, VREG_VOLTAGE_1_15 },
    { 384000000u, 1536000000u, 4, 1, VREG_VOLTAGE_1_20 },
    { 480000000u, 1440000000u, 3, 1, VREG_VOLTAGE_1_30 },
};

// Voltage ceiling: RP2350 permits a limited overvolt (up to 1.50 V, bench
// use) after the POWMAN voltage-limit unlock; the RP2040 regulator field
// ends at 1.30 V.
#if PICO_RP2350
#define SYS_CLOCK_VREG_CEIL ((uint8_t)VREG_VOLTAGE_1_50)
#else
#define SYS_CLOCK_VREG_CEIL ((uint8_t)VREG_VOLTAGE_1_30)
#endif

static SysClockMode sys_clock_mode_active = SYS_CLOCK_MODE_307P2;
static bool sys_clock_fell_back = false;
static bool sys_clock_confirmed = false;
static uint64_t sys_clock_armed_at_us = 0;

volatile uint8_t sys_clock_req_mode = 0;
volatile uint8_t sys_clock_req_vreg = 0xFF;
volatile bool    sys_clock_set_pending = false;

bool sys_clock_mode_valid(uint8_t m) {
    return m < (uint8_t)SYS_CLOCK_MODE_COUNT;
}

bool sys_clock_vreg_valid(uint8_t mode, uint8_t vreg_sel) {
    if (!sys_clock_mode_valid(mode)) return false;
    if (vreg_sel == 0xFF) return true;
    // The voltage knob exists to trade voltage UP for stability; anything
    // below the mode's default is a guaranteed-unstable (bricking) setting
    // and is rejected outright rather than clamped.
    return vreg_sel >= sys_clock_table[mode].default_vreg &&
           vreg_sel <= SYS_CLOCK_VREG_CEIL;
}

// Above VREG_VOLTAGE_MAX (1.30 V) the SDK silently clamps unless the POWMAN
// voltage limit is disabled first (RP2350 only; RP2040 cannot exceed it).
static void sys_clock_set_vreg(uint8_t vreg) {
#if PICO_RP2350
    if (vreg > (uint8_t)VREG_VOLTAGE_MAX) vreg_disable_voltage_limit();
#endif
    vreg_set_voltage((enum vreg_voltage)vreg);
}

uint8_t sys_clock_resolve_vreg(SysClockMode m, uint8_t vreg_sel) {
    if (!sys_clock_mode_valid((uint8_t)m)) m = SYS_CLOCK_MODE_307P2;
    if (!sys_clock_vreg_valid((uint8_t)m, vreg_sel) || vreg_sel == 0xFF)
        return sys_clock_table[m].default_vreg;
    return vreg_sel;
}

SysClockMode sys_clock_active_mode(void) {
    return sys_clock_mode_active;
}

bool sys_clock_fallback_active(void) {
    return sys_clock_fell_back;
}

// Raising voltage must lead the frequency step and dropping it must trail,
// otherwise the core runs briefly under-volted for its clock.
static void sys_clock_switch(SysClockMode m, uint8_t vreg) {
    const SysClockEntry *e = &sys_clock_table[m];
    uint8_t cur = (uint8_t)vreg_get_voltage();

#if PICO_RP2350
    // QSPI divider for the FASTER of old/new before the PLL moves, so the
    // flash link never overshoots mid-switch; trimmed to the new clock after.
    uint32_t cur_hz = clock_get_hz(clk_sys);
    dspi_flash_apply_clkdiv_for_hz(cur_hz > e->sys_hz ? cur_hz : e->sys_hz);
#endif

    if (vreg > cur) {
        sys_clock_set_vreg(vreg);
        busy_wait_ms(10);
        set_sys_clock_pll(e->vco_hz, e->postdiv1, e->postdiv2);
    } else {
        set_sys_clock_pll(e->vco_hz, e->postdiv1, e->postdiv2);
        if (vreg < cur) sys_clock_set_vreg(vreg);
    }

#if PICO_RP2350
    dspi_flash_apply_clkdiv_for_hz(e->sys_hz);
#endif
    sys_clock_mode_active = m;
}

void sys_clock_boot_init(void) {
#if PICO_RP2350
    // Tame the ROM's XIP divider before the directory read below and, more
    // importantly, before the PLL can climb to 384/480 MHz.
    dspi_flash_apply_clkdiv();
#endif

    uint8_t mode = 0, vreg_sel = 0xFF;
    preset_get_sys_clock(&mode, &vreg_sel);
    if (!sys_clock_mode_valid(mode)) mode = 0;

    // A watchdog reset with the breadcrumb still armed means the previous boot
    // never survived the confirm window; fall back and LATCH so every further
    // warm reboot stays safe.  Power cycling clears the latch and retries the
    // stored setting; the setting itself is never rewritten.
    bool latched = (SYS_CLOCK_SCRATCH == SYS_CLOCK_FALLBACK_LATCH);
    bool crumbed = watchdog_caused_reboot() &&
                   (SYS_CLOCK_SCRATCH & SYS_CLOCK_BREADCRUMB_MASK) == SYS_CLOCK_BREADCRUMB_BASE;
    if (latched || crumbed) {
        mode = (uint8_t)SYS_CLOCK_MODE_307P2;
        vreg_sel = 0xFF;
        sys_clock_fell_back = true;
        SYS_CLOCK_SCRATCH = SYS_CLOCK_FALLBACK_LATCH;
        sys_clock_confirmed = true;   // keep the latch; confirm must not clear it
    } else if (mode != (uint8_t)SYS_CLOCK_MODE_307P2 || vreg_sel != 0xFF) {
        // Custom voltage counts too: an undervolted mode 0 can crash-loop
        // just as hard as an overclock, and needs the same escape hatch.
        SYS_CLOCK_SCRATCH = SYS_CLOCK_BREADCRUMB(mode);
    }
    sys_clock_armed_at_us = time_us_64();

    sys_clock_switch((SysClockMode)mode, sys_clock_resolve_vreg((SysClockMode)mode, vreg_sel));

#if PICO_RP2350
    // Idempotent; the QMI timing must be re-asserted against the new sys_clk.
    dspi_flash_apply_clkdiv();
#endif
}

void sys_clock_confirm_tick(void) {
    if (sys_clock_confirmed) return;
    if (time_us_64() - sys_clock_armed_at_us < SYS_CLOCK_CONFIRM_UPTIME_US) return;
    sys_clock_confirmed = true;
    SYS_CLOCK_SCRATCH = 0;
}

void sys_clock_apply(SysClockMode m, uint8_t vreg_sel) {
    if (!sys_clock_mode_valid((uint8_t)m)) return;

    // Arm for every mode: a crash mid-switch must land on the safe default
    // even when the target is mode 0.  Also clears any fallback latch: a new
    // host command is the sanctioned way out of the latched state.
    SYS_CLOCK_SCRATCH = SYS_CLOCK_BREADCRUMB(m);
    sys_clock_confirmed = false;
    sys_clock_fell_back = false;
    sys_clock_armed_at_us = time_us_64();

    sys_clock_switch(m, sys_clock_resolve_vreg(m, vreg_sel));
}

void sys_clock_apply_vreg_only(uint8_t vreg_sel) {
    // Voltage step at an unchanged PLL: glitch-free for all clocks, so no
    // audio teardown is needed.  Breadcrumb still arms in case the new
    // voltage browns the core out; the confirm tick clears it after the
    // proving window.
    SYS_CLOCK_SCRATCH = SYS_CLOCK_BREADCRUMB(sys_clock_mode_active);
    sys_clock_confirmed = false;
    sys_clock_fell_back = false;
    sys_clock_armed_at_us = time_us_64();
    sys_clock_set_vreg(sys_clock_resolve_vreg(sys_clock_mode_active, vreg_sel));
    busy_wait_ms(10);
}
