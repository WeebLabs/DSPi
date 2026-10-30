# TDM8 output (RP2350)

The ADAT bulk output engine (`adat_output.c`) can alternatively emit all 8
post-gain outputs as one TDM8 stream, matching the TI TDM bus mode of the
TAD5242 (MD0 = TDM, target mode, 32-bit word length).

- Frame: 256 BCLK = 8 slots x 32 bits, 24-bit sample MSB-first + 8 zero bits.
- FSYNC: one-BCLK pulse, coincident with the slot 0 MSB (no 1-bit delay).
- Data and FSYNC change on the BCLK rising edge (DAC samples on falling).
- Pins: DATA = `PICO_TDM_DATA_PIN` (20), FSYNC = DATA + 1 (21), BCLK =
  `PICO_TDM_BCLK_PIN` (22). Runtime-assignable.
- Rate: 44.1 / 48 kHz only (suspends above, like ADAT).
- Daisy chain: each TAD5242 plays the last two slots and forwards the rest on
  MD6; chain four devices on one DATA line for 8 channels.
- Controller: RP2350 is the TDM controller (BCLK/FSYNC outputs).

Vendor commands: `0xCF` set format (0 ADAT, 1 TDM8), `0x80` get format,
`0x81` set pins (wValue = BCLK<<8 | DATA, 0xFF = default), `0x82` get pins.
`0xCA` (ADAT enable) enables/disables whichever format is selected.

Not yet persisted to flash/presets, not in bulk params; format and pins reset
to ADAT / defaults at boot. Not built or tested on hardware.
