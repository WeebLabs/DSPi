#include "flash_clkdiv.h"

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/bootrom.h"

#if PICO_RP2350

#include "hardware/structs/qmi.h"
#include "hardware/regs/qmi.h"
#include "hardware/clocks.h"

#define FLASH_BLOCK_ERASE_CMD   0xd8

// Per-sys-clock QSPI divider: 6 keeps flash at 51.2/64 MHz for the 307.2/384
// modes; 480 MHz uses 8 (60 MHz) because 80 MHz outruns the RXDELAY the ROM
// calibrated at its boot clock.  RAM-cached: the flash-op wrappers run with
// XIP down and must not call clock_get_hz (it lives in flash).
static uint8_t dspi_flash_div = 6;

static uint8_t clkdiv_for_hz(uint32_t sys_hz) {
    return (sys_hz > 400000000u) ? 8 : 6;
}

// Force the cached CLKDIV into the two registers that matter:
//   - DIRECT_CSR.CLKDIV: the ROM's direct-mode serial commands (erase/program)
//   - M0_TIMING.CLKDIV:  XIP reads once we leave direct mode
// Other fields (RXDELAY, COOLDOWN, RCMD/RFMT) are preserved via hw_write_masked.
static void __no_inline_not_in_flash_func(dspi_set_clkdiv)(void) {
    hw_write_masked(&qmi_hw->direct_csr,
                    (uint32_t)dspi_flash_div << QMI_DIRECT_CSR_CLKDIV_LSB,
                    QMI_DIRECT_CSR_CLKDIV_BITS);
    hw_write_masked(&qmi_hw->m[0].timing,
                    (uint32_t)dspi_flash_div << QMI_M0_TIMING_CLKDIV_LSB,
                    QMI_M0_TIMING_CLKDIV_BITS);
    __compiler_memory_barrier();
}

void dspi_flash_apply_clkdiv_for_hz(uint32_t sys_hz) {
    dspi_flash_div = clkdiv_for_hz(sys_hz);
    dspi_set_clkdiv();
}

void dspi_flash_apply_clkdiv(void) {
    dspi_flash_apply_clkdiv_for_hz(clock_get_hz(clk_sys));
}

// Re-implements SDK flash_range_erase/program but:
//   1) snapshots QMI m[0] before the ROM calls (the ROM clobbers RCMD/RFMT/
//      TIMING — same pattern as pico-sdk #1983's m[1]/PSRAM workaround),
//   2) forces CLKDIV=6 *inside* the flash op (between flash_exit_xip and the
//      ROM erase/program call, so writes also run at 51.2 MHz not the ROM
//      default ~102 MHz),
//   3) restores the saved RCMD/RFMT so XIP reads stay in quad continuous
//      mode, with our CLKDIV=6 overlaid on the preserved TIMING.
// Caller is responsible for the interrupt blackout + Core 1 parked, same
// contract as the SDK's flash_range_erase/program.  Note the blackout is
// deliberately NOT total: flash_irq_blackout_begin() (flash_storage.c) masks
// at the NVIC and leaves the two output DMA IRQ lines enabled so the audio
// slots keep clocking through the window.  Those handlers and everything they
// call are RAM-resident and touch no flash (enforced by
// scripts/check_ram_placement.py), which is the only thing that matters here:
// nothing below assumes exclusivity, it just requires that no interrupted
// code path performs an XIP access.
void __no_inline_not_in_flash_func(dspi_flash_range_erase)(uint32_t flash_offs, size_t count) {
    rom_connect_internal_flash_fn connect     = (rom_connect_internal_flash_fn) rom_func_lookup_inline(ROM_FUNC_CONNECT_INTERNAL_FLASH);
    rom_flash_exit_xip_fn         exit_xip    = (rom_flash_exit_xip_fn)         rom_func_lookup_inline(ROM_FUNC_FLASH_EXIT_XIP);
    rom_flash_range_erase_fn      range_erase = (rom_flash_range_erase_fn)      rom_func_lookup_inline(ROM_FUNC_FLASH_RANGE_ERASE);
    rom_flash_flush_cache_fn      flush_cache = (rom_flash_flush_cache_fn)      rom_func_lookup_inline(ROM_FUNC_FLASH_FLUSH_CACHE);

    uint32_t saved_timing = qmi_hw->m[0].timing;
    uint32_t saved_rcmd   = qmi_hw->m[0].rcmd;
    uint32_t saved_rfmt   = qmi_hw->m[0].rfmt;

    __compiler_memory_barrier();
    connect();
    exit_xip();
    dspi_set_clkdiv();
    range_erase(flash_offs, count, FLASH_BLOCK_SIZE, FLASH_BLOCK_ERASE_CMD);
    flush_cache();

    qmi_hw->m[0].rcmd   = saved_rcmd;
    qmi_hw->m[0].rfmt   = saved_rfmt;
    qmi_hw->m[0].timing = (saved_timing & ~QMI_M0_TIMING_CLKDIV_BITS)
                        | ((uint32_t)dspi_flash_div << QMI_M0_TIMING_CLKDIV_LSB);
    __compiler_memory_barrier();
}

void __no_inline_not_in_flash_func(dspi_flash_range_program)(uint32_t flash_offs, const uint8_t *data, size_t count) {
    rom_connect_internal_flash_fn connect       = (rom_connect_internal_flash_fn) rom_func_lookup_inline(ROM_FUNC_CONNECT_INTERNAL_FLASH);
    rom_flash_exit_xip_fn         exit_xip      = (rom_flash_exit_xip_fn)         rom_func_lookup_inline(ROM_FUNC_FLASH_EXIT_XIP);
    rom_flash_range_program_fn    range_program = (rom_flash_range_program_fn)    rom_func_lookup_inline(ROM_FUNC_FLASH_RANGE_PROGRAM);
    rom_flash_flush_cache_fn      flush_cache   = (rom_flash_flush_cache_fn)      rom_func_lookup_inline(ROM_FUNC_FLASH_FLUSH_CACHE);

    uint32_t saved_timing = qmi_hw->m[0].timing;
    uint32_t saved_rcmd   = qmi_hw->m[0].rcmd;
    uint32_t saved_rfmt   = qmi_hw->m[0].rfmt;

    __compiler_memory_barrier();
    connect();
    exit_xip();
    dspi_set_clkdiv();
    range_program(flash_offs, data, count);
    flush_cache();

    qmi_hw->m[0].rcmd   = saved_rcmd;
    qmi_hw->m[0].rfmt   = saved_rfmt;
    qmi_hw->m[0].timing = (saved_timing & ~QMI_M0_TIMING_CLKDIV_BITS)
                        | ((uint32_t)dspi_flash_div << QMI_M0_TIMING_CLKDIV_LSB);
    __compiler_memory_barrier();
}

#else  // PICO_RP2040

// On RP2040, boot2's PICO_FLASH_SPI_CLKDIV already governs both XIP reads and
// the SSI clock the bootrom uses for erase/program (via flash_enable_xip_via_boot2
// trampolining through boot2 around each op).  Nothing extra to do.

void dspi_flash_apply_clkdiv(void) {}
void dspi_flash_apply_clkdiv_for_hz(uint32_t sys_hz) { (void)sys_hz; }

// RAM-resident like the SDK implementations they wrap, so the flash-op entry
// points are uniformly XIP-safe on both platforms.
void __no_inline_not_in_flash_func(dspi_flash_range_erase)(uint32_t flash_offs, size_t count) {
    flash_range_erase(flash_offs, count);
}

void __no_inline_not_in_flash_func(dspi_flash_range_program)(uint32_t flash_offs, const uint8_t *data, size_t count) {
    flash_range_program(flash_offs, data, count);
}

#endif
