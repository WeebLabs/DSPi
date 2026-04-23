#ifndef FLASH_CLKDIV_H
#define FLASH_CLKDIV_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Wrappers around hardware_flash that force QMI CLKDIV=6 for both XIP reads
// and the ROM's direct-mode erase/program on RP2350.  On RP2040 they forward
// to the SDK — boot2's PICO_FLASH_SPI_CLKDIV already governs both there.
//
// Rationale: PICO_FLASH_SPI_CLKDIV only reaches hardware on RP2040 (via
// boot2).  On RP2350 boot2 isn't run by default, so we program QMI
// directly: apply CLKDIV=6 once at startup for XIP reads, and save/restore
// m[0] around each flash op (the ROM clobbers it) with CLKDIV=6 overlaid.
void dspi_flash_apply_clkdiv(void);
void dspi_flash_range_erase(uint32_t flash_offs, size_t count);
void dspi_flash_range_program(uint32_t flash_offs, const uint8_t *data, size_t count);

#ifdef __cplusplus
}
#endif

#endif
