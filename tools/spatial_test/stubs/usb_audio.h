/* Host stand-in: the matrix output enables the dispatcher reads. */
#pragma once
#include <stdbool.h>
typedef struct { bool enabled; } HOut;
typedef struct { HOut outputs[NUM_OUTPUT_CHANNELS]; } HMatrix;
extern HMatrix matrix_mixer;
