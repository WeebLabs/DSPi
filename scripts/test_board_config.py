#!/usr/bin/env python3
"""Compile board-specific GPIO configuration against the pinned Pico SDK headers."""

import pathlib
import subprocess
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
DSP_DIR = ROOT / "firmware" / "DSPi"
BOARD_INCLUDE_DIR = (
    ROOT / "firmware" / "pico-sdk" / "src" / "boards" / "include"
)


class BoardConfigurationTests(unittest.TestCase):
    def compile_and_run(self, board_header, rp2350, platform, source):
        with tempfile.TemporaryDirectory(prefix="dspi-board-config-") as temp:
            temp_dir = pathlib.Path(temp)
            shim = temp_dir / "pico.h"
            shim.write_text(
                "#ifndef TEST_PICO_H\n"
                "#define TEST_PICO_H\n"
                "#define pico_board_cmake_set(name, value)\n"
                "#define pico_board_cmake_set_default(name, value)\n"
                "#include DSPI_TEST_BOARD_HEADER\n"
                "#endif\n"
            )
            c_file = temp_dir / "board_config_test.c"
            executable = temp_dir / "board_config_test"
            c_file.write_text(source)

            command = [
                "cc",
                "-std=c11",
                "-Wall",
                "-Wextra",
                "-Werror",
                f"-DPICO_RP2350={rp2350}",
                f'-DDSPI_TEST_BOARD_HEADER="boards/{board_header}"',
                "-I",
                str(temp_dir),
                "-I",
                str(BOARD_INCLUDE_DIR),
                "-I",
                str(
                    ROOT
                    / "firmware"
                    / "pico-sdk"
                    / "src"
                    / platform
                    / "hardware_regs"
                    / "include"
                ),
                "-I",
                str(DSP_DIR),
                str(c_file),
                "-o",
                str(executable),
            ]
            compiled = subprocess.run(
                command, text=True, capture_output=True, check=False
            )
            self.assertEqual(
                compiled.returncode,
                0,
                f"Host compile failed:\n{compiled.stdout}{compiled.stderr}",
            )

            ran = subprocess.run(
                [str(executable)], text=True, capture_output=True, check=False
            )
            self.assertEqual(
                ran.returncode,
                0,
                f"Host test failed:\n{ran.stdout}{ran.stderr}",
            )

    def test_pico_keeps_rp2040_gpio_and_slave_clock_defaults(self):
        self.compile_and_run(
            "pico.h",
            0,
            "rp2040",
            r"""
#include <assert.h>
#include "config.h"

#ifdef PICO_SPDIF_PIN_3
#error "RP2040 must not define the third S/PDIF output"
#endif

_Static_assert(PICO_I2S_BCK_PIN_SLAVE == 12, "RP2040 slave BCK");

int main(void) {
    assert(!dspi_board_gpio_is_reserved(8));
    assert(dspi_gpio_is_valid(8));
    assert(dspi_gpio_is_valid(0));
    assert(dspi_gpio_is_valid(22));
    assert(dspi_gpio_is_valid(26));
    assert(dspi_gpio_is_valid(28));
    assert(!dspi_gpio_is_valid(23));
    assert(!dspi_gpio_is_valid(24));
    assert(!dspi_gpio_is_valid(25));
    assert(!dspi_gpio_is_valid(29));
    assert(!dspi_gpio_is_valid(30));
    assert(!dspi_gpio_is_valid(255));
    return 0;
}
""",
        )

    def test_pico2_keeps_standard_rp2350_gpio_and_pin_defaults(self):
        self.compile_and_run(
            "pico2.h",
            1,
            "rp2350",
            r"""
#include <assert.h>
#include "config.h"

_Static_assert(PICO_SPDIF_PIN_3 == 8, "Pico 2 S/PDIF 3");
_Static_assert(PICO_I2S_BCK_PIN_SLAVE == 26, "Pico 2 slave BCK");
_Static_assert(PICO_RP2350A == 1, "Pico 2 silicon variant");
_Static_assert(PICO_FLASH_SIZE_BYTES == 4194304, "Pico 2 flash size");

int main(void) {
    assert(!dspi_board_gpio_is_reserved(8));
    assert(dspi_gpio_is_valid(8));
    assert(dspi_gpio_is_valid(0));
    assert(dspi_gpio_is_valid(22));
    assert(dspi_gpio_is_valid(26));
    assert(dspi_gpio_is_valid(29));
    assert(!dspi_gpio_is_valid(23));
    assert(!dspi_gpio_is_valid(24));
    assert(!dspi_gpio_is_valid(25));
    assert(!dspi_gpio_is_valid(30));
    assert(!dspi_gpio_is_valid(47));
    assert(!dspi_gpio_is_valid(255));
    return 0;
}
""",
        )

    def test_olimex_uses_reserved_psram_gpio_and_noncolliding_defaults(self):
        self.compile_and_run(
            "olimex_rp2350_xxl.h",
            1,
            "rp2350",
            r"""
#include <assert.h>
#include "config.h"

_Static_assert(PICO_SPDIF_PIN_3 == 26, "Olimex S/PDIF 3");
_Static_assert(PICO_I2S_BCK_PIN_SLAVE == 28, "Olimex slave BCK");
_Static_assert(PICO_RP2350A == 0, "Olimex silicon variant");
_Static_assert(PICO_FLASH_SIZE_BYTES == 16777216, "Olimex flash size");
_Static_assert(PICO_I2S_BCK_PIN_SLAVE != PICO_SPDIF_PIN_3,
               "Olimex secondary clock pair must not start on S/PDIF 3");
_Static_assert(PICO_I2S_BCK_PIN_SLAVE + 1 != PICO_SPDIF_PIN_3,
               "Olimex secondary clock pair must not end on S/PDIF 3");

int main(void) {
    assert(dspi_board_gpio_is_reserved(8));
    assert(!dspi_gpio_is_valid(8));
    assert(dspi_gpio_is_valid(0));
    assert(dspi_gpio_is_valid(7));
    assert(dspi_gpio_is_valid(9));
    assert(dspi_gpio_is_valid(22));
    assert(dspi_gpio_is_valid(26));
    assert(dspi_gpio_is_valid(28));
    assert(dspi_gpio_is_valid(29));
    assert(!dspi_gpio_is_valid(23));
    assert(!dspi_gpio_is_valid(24));
    assert(!dspi_gpio_is_valid(25));
    assert(!dspi_gpio_is_valid(30));
    assert(!dspi_gpio_is_valid(47));
    assert(!dspi_gpio_is_valid(255));
    return 0;
}
""",
        )

    def test_pico_uses_native_adc_pins_for_three_potentiometers(self):
        self.compile_and_run(
            "pico.h",
            0,
            "rp2040",
            r"""
#include <assert.h>
#include "config.h"
#include <hardware/platform_defs.h>

_Static_assert(DSPI_CONTROL_ADC_FIRST_PIN == 26, "Pico ADC first GPIO");
_Static_assert(DSPI_CONTROL_ADC_LAST_PIN == 28, "Pico ADC last GPIO");
_Static_assert(DSPI_CONTROL_ADC_FIRST_PIN - ADC_BASE_PIN == 0, "Pico ADC channel 0");
_Static_assert(DSPI_CONTROL_ADC_FIRST_PIN + 1 - ADC_BASE_PIN == 1, "Pico ADC channel 1");
_Static_assert(DSPI_CONTROL_ADC_LAST_PIN - ADC_BASE_PIN == 2, "Pico ADC channel 2");
_Static_assert(PICO_SMPS_MODE_PIN == 23, "Pico SMPS mode pin");

int main(void) {
    assert(dspi_control_adc_pin_is_valid(26));
    assert(dspi_control_adc_pin_is_valid(27));
    assert(dspi_control_adc_pin_is_valid(28));
    assert(!dspi_control_adc_pin_is_valid(25));
    assert(!dspi_control_adc_pin_is_valid(29));
    assert(!dspi_control_adc_pin_is_valid(40));
    assert(!dspi_control_adc_pin_is_valid(41));
    assert(!dspi_control_adc_pin_is_valid(42));
    assert(!dspi_control_adc_pin_is_valid(255));
    return 0;
}
""",
        )

    def test_pico2_uses_native_adc_pins_for_three_potentiometers(self):
        self.compile_and_run(
            "pico2.h",
            1,
            "rp2350",
            r"""
#include <assert.h>
#include "config.h"
#include <hardware/platform_defs.h>

_Static_assert(DSPI_CONTROL_ADC_FIRST_PIN == 26, "Pico 2 ADC first GPIO");
_Static_assert(DSPI_CONTROL_ADC_LAST_PIN == 28, "Pico 2 ADC last GPIO");
_Static_assert(DSPI_CONTROL_ADC_FIRST_PIN - ADC_BASE_PIN == 0, "Pico 2 ADC channel 0");
_Static_assert(DSPI_CONTROL_ADC_FIRST_PIN + 1 - ADC_BASE_PIN == 1, "Pico 2 ADC channel 1");
_Static_assert(DSPI_CONTROL_ADC_LAST_PIN - ADC_BASE_PIN == 2, "Pico 2 ADC channel 2");
_Static_assert(PICO_SMPS_MODE_PIN == 23, "Pico 2 SMPS mode pin");

int main(void) {
    assert(dspi_control_adc_pin_is_valid(26));
    assert(dspi_control_adc_pin_is_valid(27));
    assert(dspi_control_adc_pin_is_valid(28));
    assert(!dspi_control_adc_pin_is_valid(25));
    assert(!dspi_control_adc_pin_is_valid(29));
    assert(!dspi_control_adc_pin_is_valid(40));
    assert(!dspi_control_adc_pin_is_valid(41));
    assert(!dspi_control_adc_pin_is_valid(42));
    assert(!dspi_control_adc_pin_is_valid(255));
    return 0;
}
""",
        )

    def test_olimex_uses_native_adc_pins_without_general_high_gpio_access(self):
        self.compile_and_run(
            "olimex_rp2350_xxl.h",
            1,
            "rp2350",
            r"""
#include <assert.h>
#include "config.h"
#include <hardware/platform_defs.h>

#ifdef PICO_SMPS_MODE_PIN
#error "Olimex must not claim GPIO 23 as an SMPS mode pin"
#endif

_Static_assert(DSPI_CONTROL_ADC_FIRST_PIN == 40, "Olimex ADC first GPIO");
_Static_assert(DSPI_CONTROL_ADC_LAST_PIN == 42, "Olimex ADC last GPIO");
_Static_assert(DSPI_CONTROL_ADC_FIRST_PIN - ADC_BASE_PIN == 0, "Olimex ADC channel 0");
_Static_assert(DSPI_CONTROL_ADC_FIRST_PIN + 1 - ADC_BASE_PIN == 1, "Olimex ADC channel 1");
_Static_assert(DSPI_CONTROL_ADC_LAST_PIN - ADC_BASE_PIN == 2, "Olimex ADC channel 2");

int main(void) {
    assert(dspi_control_adc_pin_is_valid(40));
    assert(dspi_control_adc_pin_is_valid(41));
    assert(dspi_control_adc_pin_is_valid(42));
    assert(!dspi_control_adc_pin_is_valid(8));
    assert(!dspi_control_adc_pin_is_valid(26));
    assert(!dspi_control_adc_pin_is_valid(27));
    assert(!dspi_control_adc_pin_is_valid(28));
    assert(!dspi_control_adc_pin_is_valid(39));
    assert(!dspi_control_adc_pin_is_valid(43));
    assert(!dspi_control_adc_pin_is_valid(47));
    assert(!dspi_control_adc_pin_is_valid(255));
    assert(!dspi_gpio_is_valid(40));
    assert(!dspi_gpio_is_valid(41));
    assert(!dspi_gpio_is_valid(42));
    return 0;
}
""",
        )


if __name__ == "__main__":
    unittest.main()
