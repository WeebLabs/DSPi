/* Host stand-in for firmware/DSPi/config.h: only what spatial.c needs. */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#ifndef PICO_RP2350
#define PICO_RP2350 1
#endif
#if PICO_RP2350
#define NUM_SPDIF_INSTANCES 4
#define NUM_OUTPUT_CHANNELS 9
#else
#define NUM_SPDIF_INSTANCES 2
#define NUM_OUTPUT_CHANNELS 5
#endif
#define AUDIO_BUFFER_SAMPLES 192
#define DSP_TIME_CRITICAL
