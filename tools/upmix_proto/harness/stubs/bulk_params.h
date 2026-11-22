/* Host stand-in: upmix.c only takes offsets into WireBulkParams for notify. */
#pragma once
#include <stddef.h>
#include <stdint.h>
#ifdef UPMIX_PACKET_V1_SIZE   /* v2 engine: profile 0 head at upmix, the rest in upmix2 */
typedef struct { uint8_t b[36]; } HStubUpmixExt;
typedef struct {
    uint8_t upmix[44];
    struct { HStubUpmixExt p0_ext; uint8_t p1[44]; HStubUpmixExt p1_ext; } upmix2;
} WireBulkParams;
#else
typedef struct { UpmixConfigPacket upmix; } WireBulkParams;
#endif
