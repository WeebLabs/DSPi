#pragma once
#include <stdint.h>
#include <stdbool.h>
typedef enum { CS_NOUN_USER_VOLUME=0, CS_NOUN_USER_MUTE=2, CS_NOUN_LOUDNESS=3, CS_NOUN_CROSSFEED=4,
  CS_NOUN_LEVELLER=5, CS_NOUN_PRESET=6, CS_NOUN_CLIP=8, CS_NOUN_EQ_BYPASS=9, CS_NOUN_FILTER_FREQ=20,
  CS_NOUN_FILTER_GAIN=21, CS_NOUN_FILTER_Q=22, CS_NOUN_FILTER_TYPE=23, CS_NOUN_FILTER_BYPASS=24,
  CS_NOUN_LEVEL=28, CS_NOUN_SAMPLE_RATE=30, CS_NOUN_USB_STREAMING=31, CS_NOUN_PSYBASS=41,
  CS_NOUN_CPU_LOAD=53, CS_NOUN_TUBE=70, CS_NOUN_PRESET_SAVE=79 } CsNoun;
float cs_noun_get(uint8_t noun, uint8_t target, uint8_t index);
bool  cs_noun_dispatch(uint8_t noun, uint8_t target, uint8_t index, float value);
