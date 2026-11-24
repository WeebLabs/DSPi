# DSPi front-panel UI: XIAO RP2350 + SSD1331 OLED, OP-1 style

Not built or run on hardware yet. What has been checked: the pure logic and renderer (host tests, ASan/UBSan),
`ui.c` against a fake DSPi and fake hardware (`make -C ui/test run_ui`), and a clang syntax/semantic pass of
`ssd1331.c`, `rgb_led.c`, `ui_hw.c`, `ui.c` against the real RP2350 SDK headers and DSPi headers.

## Wiring

| Signal | XIAO pin | GPIO | Goes to |
|---|---|---|---|
| I2S BCK | D0 | 26 | PCM5102 BCK |
| I2S LRCK | D1 | 27 | PCM5102 LCK (always BCK + 1) |
| I2S DATA | D2 | 28 | PCM5102 DIN |
| Display RES | D3 | 5 | OLED RES |
| Display CS | D7 | 1 | OLED CS |
| Display SCK | D8 | 2 | OLED SCL/SCK |
| Display DC | D9 | 4 | OLED DC |
| Display MOSI | D10 | 3 | OLED SDA/MOSI |
| spare | D6 | 0 | optional external button to GND (build with `-DUI_BTN_SOURCE=UI_BTN_SRC_GPIO`) |
| do not use | D4, D5 | 6, 7 | idle S/PDIF slot pins DSPi drives at boot |

PCM5102: SCK pin tied to GND (internal PLL, no MCLK needed). Display and DAC on 3V3.

## Controls (one button: BOOT by default)

* Top level: short press = next page (NOW, EQ, FX, PRESET, SYS); long press (0.6 s) = open that page's menu. On NOW a long press toggles mute.
* In a menu: short press = next item; long press = activate it. The last item is always "back".
* Display dims after 30 s, turns off after 2 min without USB streaming; any press wakes it (that press does nothing else).
* RGB LED: green = streaming, slow blue blink = idle, orange = muted, red = clip, page colour while a menu is open, white while holding toward a long press. The LED and display levels are changed on the SYS page and are not saved.

## Build

`cmake -S firmware -B build -DPICO_BOARD=seeed_xiao_rp2350` turns the UI on automatically (`-DDSPI_UI=ON/OFF` overrides).
It adds ~12 KB of RAM (framebuffer): run `scripts/check_ram_placement.py` and update `ram_baseline.json` if it flags it.

## Things to verify on the bench

1. Display model: this driver is for the SSD1331 (96x64). If the module is something else, `ssd1331.c` is the only file to change.
2. Orientation / colour order: adjust `UI_DISP_REMAP` in `ui_board.h` (0x72 is the common value).
3. BOOT-button read: it briefly tri-states the flash chip-select, so it is the riskiest piece. If audio glitches when pressing, or the button never reads, use an external button on D6.
4. WS2812 on GPIO22 and its power enable on GPIO23: confirm against the board schematic.
5. Flash settings: a previously saved output config overrides the new factory defaults (I2S on 26/27/28). Do a factory reset from the host app once.
6. DSPi's default S/PDIF RX pin (GPIO 5) and I2S RX pin (GPIO 1) collide with display RES/CS. Do not select those inputs on this build.
7. The "saved pN" toast appears when the save is requested; it does not wait for the flash write result.

Licence: GPL-3.0, inherited from DSPi.
