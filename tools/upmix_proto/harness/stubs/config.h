/* Host build stand-in for firmware/DSPi/config.h: only what upmix.c needs. */
#pragma once
#define PICO_RP2350 1
#define NUM_INPUT_CHANNELS 8
#define NUM_STEREO_INPUTS 2
#define DSP_TIME_CRITICAL
#define NUM_OUTPUT_CHANNELS 9
#define AUDIO_BUFFER_SAMPLES 192
#define CLIP_THRESH_F 1.001f
