/*
 * ui_hw.c; see ui_hw.h
 *
 * BOOT button: it sits on the QSPI_SS (flash chip-select) line, so reading it
 * means briefly tri-stating that line, during which flash (XIP) is not
 * reachable. The read runs from RAM with interrupts off on this core. Core 1
 * is RAM-resident in DSPi but may read flash-resident tables, so the read only
 * happens directly while core 1 is idle; otherwise it goes through the same
 * multicore lockout the flash writer uses (UI_BOOTSEL_LOCKOUT=0 disables the
 * read instead). An external switch on UI_PIN_BTN_GPIO avoids all of this
 * (build with UI_BTN_SOURCE=UI_BTN_SRC_GPIO).
 */
#include "ui_hw.h"
#include "ui_board.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/multicore.h"
#include "pico/platform.h"
#if UI_BTN_SOURCE == UI_BTN_SRC_BOOTSEL
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#include "hardware/regs/io_qspi.h"
#include "config.h"
#include "pdm_generator.h"     // core1_mode
#endif

#ifndef UI_BOOTSEL_LOCKOUT
#define UI_BOOTSEL_LOCKOUT 1
#endif

#if UI_BTN_SOURCE == UI_BTN_SRC_BOOTSEL
#define UI_QSPI_SS_IDX 1

// Whole body in RAM: flash is unreachable while the chip select is released.
static bool __no_inline_not_in_flash_func(bootsel_read_ram)(void) {
    uint32_t flags = save_and_disable_interrupts();
    hw_write_masked(&ioqspi_hw->io[UI_QSPI_SS_IDX].ctrl,
                    GPIO_OVERRIDE_LOW << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);
    for (volatile int i = 0; i < 3000; ++i) { }          // settle (scaled for a 307 MHz core clock)
    bool pressed = !(sio_hw->gpio_hi_in & SIO_GPIO_HI_IN_QSPI_CSN_BITS);   // button pulls the line low
    hw_write_masked(&ioqspi_hw->io[UI_QSPI_SS_IDX].ctrl,
                    GPIO_OVERRIDE_NORMAL << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);
    restore_interrupts(flags);
    return pressed;
}
#endif

void ui_hw_init(void) {
    gpio_init(UI_PIN_USER_LED);
    gpio_set_dir(UI_PIN_USER_LED, GPIO_OUT);
    gpio_put(UI_PIN_USER_LED, 1);                         // off (active low)
#if UI_BTN_SOURCE == UI_BTN_SRC_GPIO
    gpio_init(UI_PIN_BTN_GPIO);
    gpio_set_dir(UI_PIN_BTN_GPIO, GPIO_IN);
    gpio_pull_up(UI_PIN_BTN_GPIO);
#endif
}

bool ui_hw_button_pressed(uint32_t now_ms) {
    static bool     last;
    static uint32_t t_last;
    static bool     have;
    if (have && (int32_t)(now_ms - t_last) < 5) return last;
    t_last = now_ms; have = true;
#if UI_BTN_SOURCE == UI_BTN_SRC_GPIO
    last = !gpio_get(UI_PIN_BTN_GPIO);
#else
    if (core1_mode == CORE1_MODE_IDLE) {
        last = bootsel_read_ram();
    } else {
#if UI_BOOTSEL_LOCKOUT
        if (multicore_lockout_victim_is_initialized(1)) {
            multicore_lockout_start_blocking();
            last = bootsel_read_ram();
            multicore_lockout_end_blocking();
        }
#endif
        // else keep the last value
    }
#endif
    return last;
}

void ui_hw_user_led(bool on) { gpio_put(UI_PIN_USER_LED, !on); }
