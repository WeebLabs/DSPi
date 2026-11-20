/* Host stand-in: upmix.c only takes offsetof(WireBulkParams, upmix.f). */
#pragma once
#include <stddef.h>
typedef struct { UpmixConfigPacket upmix; } WireBulkParams;
