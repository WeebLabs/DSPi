/*
 * ac3_config.h; user settings for AC-3 decoding on the S/PDIF input.
 *
 * Both platforms hold the config so it round-trips through presets and the
 * wire; only RP2350 decodes.  Sourced by output_config_mode like the output
 * limiter (slot in WITH_PRESET, directory at boot in INDEPENDENT).  See
 * Documentation/Features/ac3_decoding_spec.md.
 */

#ifndef AC3_CONFIG_H
#define AC3_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#define AC3_DRC_OFF          0   // ignore the stream's dynamic range words
#define AC3_DRC_FULL         1   // apply them as sent (decoder default)
#define AC3_DRC_HALF         2   // apply half the gain change in dB
#define AC3_DRC_MODE_MAX     2

#define AC3_DIALNORM_IGNORE  0   // play at the coded level
#define AC3_DIALNORM_APPLY   1   // normalise dialogue to -31 dBFS (the AC-3 reference)
#define AC3_DIALNORM_MODE_MAX 1

typedef struct {
    uint8_t enabled;        // 0: bitstreams stay muted (Phase 0 behaviour)
    uint8_t drc_mode;       // AC3_DRC_*
    uint8_t dialnorm_mode;  // AC3_DIALNORM_*
} Ac3Config;

extern Ac3Config ac3_config;

void ac3_config_defaults(Ac3Config *c);
// Replace out-of-range fields with their defaults; true when anything changed.
bool ac3_config_sanitize(Ac3Config *c);

#endif // AC3_CONFIG_H
