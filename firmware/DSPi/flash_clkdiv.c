#include "flash_clkdiv.h"

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/bootrom.h"

#if PICO_RP2350

#include "hardware/structs/qmi.h"
#include "hardware/regs/qmi.h"

#define DSPI_FLASH_SPI_CLKDIV   6
#define FLASH_BLOCK_ERASE_CMD   0xd8

// Force CLKDIV=6 into the two registers that matter:
//   - DIRECT_CSR.CLKDIV: the ROM's direct-mode serial commands (erase/program)
//   - M0_TIMING.CLKDIV:  XIP reads once we leave direct mode
// Other fields (RXDELAY, COOLDOWN, RCMD/RFMT) are preserved via hw_write_masked.
static void __no_inline_not_in_flash_func(dspi_set_clkdiv)(void) {
    hw_write_masked(&qmi_hw->direct_csr,
                    DSPI_FLASH_SPI_CLKDIV << QMI_DIRECT_CSR_CLKDIV_LSB,
                    QMI_DIRECT_CSR_CLKDIV_BITS);
    hw_write_masked(&qmi_hw->m[0].timing,
                    DSPI_FLASH_SPI_CLKDIV << QMI_M0_TIMING_CLKDIV_LSB,
                    QMI_M0_TIMING_CLKDIV_BITS);
    __compiler_memory_barrier();
}

void dspi_flash_apply_clkdiv(void) { dspi_set_clkdiv(); }

// Re-implements SDK flash_range_erase/program but:
//   1) snapshots QMI m[0] before the ROM calls (the ROM clobbers RCMD/RFMT/
//      TIMING — same pattern as pico-sdk #1983's m[1]/PSRAM workaround),
//   2) forces CLKDIV=6 *inside* the flash op (between flash_exit_xip and the
//      ROM erase/program call, so writes also run at 51.2 MHz not the ROM
//      default ~102 MHz),
//   3) restores the saved RCMD/RFMT so XIP reads stay in quad continuous
//      mode, with our CLKDIV=6 overlaid on the preserved TIMING.
// Caller is responsible for interrupts-off + Core 1 parked, same contract as
// the SDK's flash_range_erase/program.
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
                        | (DSPI_FLASH_SPI_CLKDIV << QMI_M0_TIMING_CLKDIV_LSB);
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
                        | (DSPI_FLASH_SPI_CLKDIV << QMI_M0_TIMING_CLKDIV_LSB);
    __compiler_memory_barrier();
}

#else  // PICO_RP2040

// On RP2040, boot2's PICO_FLASH_SPI_CLKDIV already governs both XIP reads and
// the SSI clock the bootrom uses for erase/program (via flash_enable_xip_via_boot2
// trampolining through boot2 around each op).  Nothing extra to do.

void dspi_flash_apply_clkdiv(void) {}

void dspi_flash_range_erase(uint32_t flash_offs, size_t count) {
    flash_range_erase(flash_offs, count);
}

void dspi_flash_range_program(uint32_t flash_offs, const uint8_t *data, size_t count) {
    flash_range_program(flash_offs, data, count);
}

#endif
