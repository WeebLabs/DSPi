#ifndef BULK_PARAMS_H
#define BULK_PARAMS_H

/*
 * bulk_params.h — Wire format for bulk parameter transfer
 *
 * Defines a platform-independent binary format for transferring the complete
 * DSP state in a single USB control transfer.  Used for:
 *   - App startup (GET: read all params in one shot)
 *   - Post-preset-load UI sync (GET: refresh after preset switch)
 *   - Configuration restore (SET: apply a saved state)
 *
 * All multi-byte fields are little-endian (native ARM).  All float fields
 * are IEEE 754 single-precision.  Floats are at 4-byte-aligned offsets.
 *
 * Variable-dimension arrays (channels, outputs) are sized at platform
 * maximums.  The header's num_channels / num_output_channels fields tell
 * the host how many entries are valid; remaining entries are zero-padded.
 */

#include <stdint.h>
#include <stdbool.h>

// Fixed maximums for the wire format (sized for the largest platform).
// Channel index space: [ inputs 0..WIRE_MAX_INPUT_CHANNELS-1 ][ outputs ... ],
// matching the firmware NUM_CHANNELS layout.
#define WIRE_MAX_INPUT_CHANNELS   8   // RP2350 max input channels
#define WIRE_MAX_OUTPUT_CHANNELS  9   // RP2350 max output channels
#define WIRE_MAX_CHANNELS        17   // inputs + outputs (RP2350 max)
#define WIRE_MAX_BANDS           12   // Same on both
#define WIRE_MAX_PIN_OUTPUTS      5   // RP2350 max (4 SPDIF + 1 PDM)
#define WIRE_NAME_LEN            32   // Must match PRESET_NAME_LEN

#define WIRE_FORMAT_VERSION      29   // V29: append subharmonic synthesizer section (16 bytes; dbx-style octave divider, both platforms). V28: fourth selectable SPDIF input; input-config spdif_rx_pin_ext grows 2 to 3 entries, shifting the fields below it down one byte and consuming that section's last reserved byte (section size unchanged). V27: upmixer centre mode gains OFF (2), a surrounds-only setting that leaves L/R bit-exact; enum widening only, no struct or offset changes. V26: upmixer presence bell claims the upmix section reserved byte (int8, dB*2; struct sizes unchanged). V25: append upmixer section (44 bytes; RP2350 stereo upmixer, zeroed/ignored on RP2040). V24: ADAT input config (pin/enable/clock mode) claimed from the input-config reserved bytes (struct size unchanged). V23: append psybass section (24 bytes; psychoacoustic bass enhancement). V22: Linkwitz Transform target Q carried in the EQ WireBandParams reserved[2] bytes (uint16 LE, Q*512; zero for non-LT types; struct size unchanged). V21: I2S clock master/slave mode in the input-config section (claims one reserved byte; size unchanged). V20: crossfeed output_pair_mask replaces WireCrossfeedParams reserved byte; struct sizes unchanged. V19: loudness_output_mask replaces global reserved[2]; struct sizes unchanged. V18: leveller detector/apply channel masks (WireLevellerConfig grows 16 to 20 bytes). V17: append ADAT output config section (RP2350; zeroed/ignored on RP2040). V16: unified channel model (inputs are first-class channels with PEQ + metering; no "master"); matrix/preamp direct (8 inputs); compat-breaking, no migration.
#define WIRE_MAX_SPDIF_INSTANCES  4   // RP2350 max

// Platform IDs
#define WIRE_PLATFORM_RP2040      0
#define WIRE_PLATFORM_RP2350      1

// ============================================================================
// Section 1: Packet Header (16 bytes)
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t  format_version;         // WIRE_FORMAT_VERSION
    uint8_t  platform_id;            // WIRE_PLATFORM_RP2040 or _RP2350
    uint8_t  num_channels;           // Actual channel count (7 or 17)
    uint8_t  num_output_channels;    // Actual output count (5 or 9)
    uint8_t  num_input_channels;     // Actual input count (2 or 8)
    uint8_t  max_bands;              // Bands per channel in this payload (12)
    uint16_t payload_length;         // Total packet size including header
    uint16_t fw_version_major;       // Firmware version
    uint16_t fw_version_minor;
    uint32_t reserved;               // Zero, future flags
} WireHeader;                        // 16 bytes

// ============================================================================
// Section 2: Global Parameters (16 bytes)
// ============================================================================
typedef struct __attribute__((packed)) {
    float    preamp_gain_db;         // Preamp gain
    uint8_t  bypass;                 // Master EQ bypass (0/1)
    uint8_t  loudness_enabled;       // Loudness compensation (0/1)
    uint16_t loudness_output_mask;   // Bit k: loudness processes output channel k (V19+)
    float    loudness_ref_spl;       // Reference SPL
    float    loudness_intensity_pct; // Intensity percentage
} WireGlobalParams;                  // 16 bytes

// ============================================================================
// Section 3: Crossfeed Parameters (16 bytes)
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t  enabled;                // Crossfeed on/off
    uint8_t  preset;                 // Preset index
    uint8_t  itd_enabled;            // ITD simulation on/off
    uint8_t  output_pair_mask;       // Bit p: crossfeed runs on output pair p (V20+)
    float    custom_fc;              // Custom crossover frequency (Hz)
    float    custom_feed_db;         // Custom feed level (dB)
    uint32_t reserved2;              // Future expansion
} WireCrossfeedParams;               // 16 bytes

// ============================================================================
// Section 4: Legacy Channel Gain/Mute (16 bytes)
// ============================================================================
typedef struct __attribute__((packed)) {
    float    gain_db[3];             // Per-channel gain
    uint8_t  mute[3];               // Per-channel mute (0/1)
    uint8_t  reserved;
} WireLegacyChannels;                // 16 bytes

// ============================================================================
// Section 5: Per-Channel Delays (fixed at WIRE_MAX_CHANNELS)
// ============================================================================
typedef struct __attribute__((packed)) {
    float    delay_ms[WIRE_MAX_CHANNELS];  // ms, zero-padded beyond num_channels
} WireChannelDelays;                 // 68 bytes (17 channels)

// ============================================================================
// Section 6: Matrix Crosspoint (8 bytes each)
// Layout: input 0 outputs 0..8, then input 1 outputs 0..8 (row-major)
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t  enabled;
    uint8_t  phase_invert;
    uint8_t  reserved[2];
    float    gain_db;
} WireCrosspoint;                    // 8 bytes

// ============================================================================
// Section 7: Matrix Output Channel (12 bytes each)
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t  enabled;
    uint8_t  mute;
    uint8_t  reserved[2];
    float    gain_db;
    float    delay_ms;
} WireOutputChannel;                 // 12 bytes

// ============================================================================
// Section 8: Pin Configuration (8 bytes)
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t  num_pin_outputs;            // 3 on RP2040, 5 on RP2350
    uint8_t  pins[WIRE_MAX_PIN_OUTPUTS]; // GPIO pin numbers, zero-padded
    uint8_t  reserved[2];
} WirePinConfig;                         // 8 bytes

// ============================================================================
// Section 9: EQ Band Parameters (16 bytes each)
// Layout: channel 0 bands 0..11, channel 1 bands 0..11, ... (row-major)
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t  type;                   // Filter type enum
    uint8_t  bypass;                 // 1 = user-bypassed, anything else = active. See band_bypass_spec.md.
    uint8_t  reserved[2];            // Since V22: Linkwitz Transform target Q (uint16 LE, Q*512,
                                     // 0 = 0.707 default) when type == FILTER_LINKWITZ_TRANSFORM;
                                     // zero for all other types.
    float    freq;                   // Hz
    float    q;                      // Q factor
    float    gain_db;                // dB
} WireBandParams;                    // 16 bytes

// ============================================================================
// Section 10: Channel Names (544 bytes)
// ============================================================================
typedef struct __attribute__((packed)) {
    char names[WIRE_MAX_CHANNELS][WIRE_NAME_LEN];
} WireChannelNames;                  // 544 bytes (17 × 32)

// ============================================================================
// Section 11: I2S Configuration (16 bytes) — V3+
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t  output_types[WIRE_MAX_SPDIF_INSTANCES]; // Per-slot: 0=S/PDIF, 1=I2S
    uint8_t  bck_pin;                // BCK GPIO (LRCLK = BCK + 1)
    uint8_t  mck_pin;                // MCK GPIO
    uint8_t  mck_enabled;            // 0 = off, 1 = on
    uint8_t  mck_multiplier;         // 128 or 256
    // I2S clock-pin mode + slave pair.  Claimed from the reserved bytes with
    // "0 = absent, keep live" conventions so the wire layout/size and format
    // version are unchanged (old hosts send zeros here).  The mode needs a +1
    // sentinel because 0 (unified) is a meaningful value.
    uint8_t  clock_pin_mode_p1;      // 0 = absent; 1 = unified, 2 = split
    uint8_t  bck_pin_slave;          // Slave-mode BCK GPIO, LRCLK = +1 (0 = absent)
    uint8_t  reserved[6];            // Future expansion (must be 0)
} WireI2SConfig;                     // 16 bytes

// ============================================================================
// Section 12: Volume Leveller Configuration (20 bytes) — V4+
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t  enabled;                // 0/1
    uint8_t  speed;                  // 0=Slow, 1=Medium, 2=Fast
    uint8_t  lookahead;              // 0/1 (10ms lookahead delay)
    uint8_t  reserved;
    float    amount;                 // 0.0-100.0 (compression strength %)
    float    max_gain_db;            // 0.0-35.0 (max boost for quiet content)
    float    gate_threshold_db;      // -96.0-0.0 (silence gate level dBFS)
    uint8_t  detector_mask;          // Bit k: input channel k feeds the detector (V18+)
    uint8_t  apply_mask;             // Bit k: gain applied to input channel k (V18+)
    uint8_t  reserved2[2];           // Pad to 4-byte multiple
} WireLevellerConfig;                // 20 bytes

// ============================================================================
// Section 13: Per-Channel Preamp Configuration (32 bytes)
// ============================================================================
typedef struct __attribute__((packed)) {
    float    preamp_db[WIRE_MAX_INPUT_CHANNELS]; // Per-input-channel preamp (dB), 0..7
} WirePreampConfig;                              // 32 bytes (8 inputs)

// ============================================================================
// Section 14: Master Volume (16 bytes) — V6+
// ============================================================================
typedef struct __attribute__((packed)) {
    float    master_volume_db;   // Device master volume: -128 (mute sentinel), -127..0 dB range
    uint8_t  reserved[12];       // Future expansion (pad to 16 bytes)
} WireMasterVolume;              // 16 bytes

// ============================================================================
// Section 15: Input Source Configuration (16 bytes) — V7+
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t  input_source;           // InputSource enum (0=USB, 1=SPDIF, 2=I2S)
    uint8_t  spdif_rx_pin;          // SPDIF RX GPIO pin (applied on SET when apply_pins=true)
    uint8_t  i2s_rx_pin;             // I2S RX data GPIO, stereo pair 0 (V12+)
    uint8_t  i2s_input_rate;         // I2S input rate enum: 0=44100, 1=48000, 2=96000 (V12+)
    // I2S multichannel input (RP2350).  Claimed from the reserved bytes with a
    // 0 = "absent, keep live value" convention (matching i2s_rx_pin), so the
    // wire layout/size is unchanged and the format version need not bump.
    uint8_t  i2s_input_channels;     // Active I2S input channels: 2/4/6/8 (0 = absent)
    uint8_t  i2s_rx_pin_ext[3];      // I2S RX data GPIOs for stereo pairs 1..3 (0 = unset)
    // Optional SPDIF inputs 2..4, claimed from the reserved bytes with the same
    // 0 = "absent, keep live value" convention as the I2S fields above.  The
    // enable mask is stored PLUS ONE for that reason: a host that pushes zeros
    // here means "absent", and plain encoding 0 would read as "disable all".
    // V28 widened the pin array from 2 to 3 entries (SPDIF 4), shifting every
    // field below it down one byte and consuming the section's last reserved
    // byte; the section is now full.
    uint8_t  spdif_rx_pin_ext[3];    // SPDIF RX 2/3/4 GPIOs (0 = absent, keep live)
    uint8_t  spdif_rx_enabled_ext_p1;// SPDIF 2/3/4 enable mask + 1 (0 = absent;
                                     // 1 = all disabled, 2 = SPDIF2, 3 = 2+3, ...)
    uint8_t  i2s_clock_mode;         // I2S clock: 0=master, 1=slave.  Valid from wire V21;
                                     // pre-V21 readers see this as a reserved (zero) byte,
                                     // which decodes as master (the correct legacy default).
    // ADAT input (V24+, RP2350), claimed from the reserved bytes with the
    // same 0 = "absent, keep live value" convention as the SPDIF ext fields
    // (enable and clock mode stored PLUS ONE so an old host's zeros are
    // absent, not "disable/master").
    uint8_t  adat_input_pin;         // ADAT RX GPIO (0 = absent, keep live)
    uint8_t  adat_input_enabled_p1;  // enable + 1 (0 absent, 1 disabled, 2 enabled)
    uint8_t  adat_clock_mode_p1;     // clock mode + 1 (0 absent, 1 master, 2 slave)
} WireInputConfig;                   // 16 bytes (full; no reserved bytes left)

// ============================================================================
// Section 16: LG Sound Sync (16 bytes) — V8+
// ============================================================================
//
// Per-preset toggle plus runtime observation of LG TV's volume/mute output.
// Only `enabled` is honored on bulk SET; `present`, `volume`, `muted` are
// runtime-only fields produced by the detection state machine and ignored
// on SET (a host pushing them would be claiming knowledge it cannot have).
//
// See Documentation/Features/lg_sound_sync_spec.md for protocol decoding,
// the detection hysteresis, and the host volume integration.  The struct
// layout intentionally matches LgSoundSyncStatus (lg_sound_sync.h) so the
// vendor REQ_GET_LG_SOUND_SYNC_STATUS response and the WireBulkParams
// section share one source of truth — no parallel field-list to drift.
typedef struct __attribute__((packed)) {
    uint8_t  enabled;                // 0/1 — user gate, honored on bulk SET
    uint8_t  present;                // 0/1 — detection state, read-only
    uint8_t  volume;                 // 0..100 (or 0xFF if never decoded), read-only
    uint8_t  muted;                  // 0/1 — last decoded mute, read-only
    uint8_t  reserved[12];           // Pad to 16 bytes (future fields here)
} WireLgSoundSync;                   // 16 bytes

// ============================================================================
// Section 17: User Volume / Mute (16 bytes) — V9+
// ============================================================================
//
// Vendor-channel user-perceived volume + mute.  `user_volume_db` mirrors the
// same quantity the UAC1 host slider drives (`audio_state.volume`), expressed
// here as float dB to match every other dB field in this packet.  `user_mute`
// is the standalone vendor mute (NOT audio_state.mute — they have different
// gating semantics in the audio pipeline; see Documentation/current_architecture.md
// "Volume & Mute").  Both fields are honored on bulk SET and roundtrip
// independently of UAC1 mute state.
//
// Layout fixed at 16 bytes for forward compatibility — extending in the
// future just shrinks `reserved`, leaving offsets stable.
typedef struct __attribute__((packed)) {
    float    user_volume_db;         // [-CENTER_VOLUME_INDEX, 0] dB; clamped on apply
    uint8_t  user_mute;              // 0/1 — vendor mute, always honored regardless of input source
    uint8_t  reserved[11];           // Pad to 16 bytes (future fields here)
} WireUserVolume;                    // 16 bytes

// ============================================================================
// Section 18: DAC Hardware Mute (16 bytes) — V10+
// ============================================================================
//
// Board-level configuration for an external DAC's MUTE pin.  Wire-stable
// layout that matches `DacHwMuteConfig` in dac_hw_mute.h exactly so the
// dispatcher can memcpy between them.  See dac_hw_mute.h and
// Documentation/Features/dac_hardware_mute_spec.md for field semantics.
typedef struct __attribute__((packed)) {
    uint8_t  enabled;                // 0 = feature off, 1 = on
    uint8_t  active_low;             // 1 = assert LOW to mute, 0 = assert HIGH
    uint8_t  pin;                    // GPIO; 0xFF = no pin
    uint8_t  reserved0;              // alignment for hold_ms
    uint16_t hold_ms;                // mute-attack hold before clock-stop
    uint16_t release_ms;             // post-clock-restart hold before unmute
    uint8_t  reserved[8];            // zero-fill
} WireDacHwMute;                     // 16 bytes

// ============================================================================
// Section 19: Crossover Bands (704 bytes) — V11+
// ============================================================================
//
// Per-channel crossover bands, layout exactly mirrors `eq[][]` but with
// MAX_XOVER_BANDS columns (4 vs 12 for EQ).  Wire band indices for crossover
// are XOVER_BAND_BASE..XOVER_BAND_BASE+MAX_XOVER_BANDS-1 (20..23) when
// addressed via vendor commands; see
// Documentation/Features/crossover_filters_spec.md.
// `band` field in WireBandParams (which the legacy EQ section also doesn't
// carry) is implicit in the section's array position; row index = channel,
// column index = local crossover-band index (0..3).
//
// `q` and `gain_db` are unused for crossover filter types (any value in the
// FILTER_XOVER_FIRST..FILTER_XOVER_LAST range); the design code ignores
// them, but they exist in the struct for wire-format parity with EQ.
// Master rows (channel 0..1, the CH_MASTER_* slots) are zeroed on collect
// and skipped on apply because crossovers are an output-channel-only
// feature; storage symmetry with EQ is a UI convenience, not a usable slot.
#define WIRE_MAX_XOVER_BANDS  4   // == MAX_XOVER_BANDS

typedef struct __attribute__((packed)) {
    WireBandParams bands[WIRE_MAX_CHANNELS][WIRE_MAX_XOVER_BANDS];  // 17 × 4 × 16 = 1088
} WireCrossoverConfig;                                              // 1088 bytes (input rows unused)

// ============================================================================
// Section 20: ADAT Output Configuration (8 bytes); V17+
// ============================================================================
//
// ADAT lightpipe output (RP2350 only); streams the 8 post-gain output channels
// as one ADAT frame per sample (see adat_output.h).  Zeroed on collect and
// ignored on apply on RP2040.  `pin == 0` on apply means the platform default
// (PICO_ADAT_PIN).
typedef struct __attribute__((packed)) {
    uint8_t  enabled;                // 0/1 configured enable (persisted intent)
    uint8_t  pin;                    // data GPIO (0 = platform default on apply)
    uint8_t  reserved[6];            // Pad to 8 bytes (future fields here)
} WireAdatConfig;                    // 8 bytes

// ============================================================================
// Section 21: Psychoacoustic Bass (24 bytes); V23+
// ============================================================================
//
// Missing-fundamental bass enhancement (see psybass.h and
// Documentation/Features/psychoacoustic_bass_spec.md).  One global parameter
// set applied to the output channels selected by output_mask.
typedef struct __attribute__((packed)) {
    uint8_t  enabled;                // 0/1
    uint8_t  reserved0;              // Zero
    uint16_t output_mask;            // Bit k: psybass processes output channel k
    float    cutoff_hz;              // Speaker LF limit, 30-300 Hz
    float    harmonics_db;           // Harmonic mix level, -24..+12 dB
    float    drive_db;               // Odd-path clipper drive, 0..18 dB
    float    character_pct;          // Even<->odd harmonic blend, 0..100
    float    original_db;            // Original low-band level, -60..0 dB
} WirePsybassParams;                 // 24 bytes

// ============================================================================
// Section 22: Stereo Upmixer (44 bytes); V25+ (presence byte V26+)
// ============================================================================
//
// RP2350-only stereo upmixer (see upmix.h).  One global config; layout mirrors
// UpmixConfigPacket in upmix.h.  Zeroed on collect and ignored on apply on
// RP2040.  Float ranges are clamped downstream in upmix_compute_coefficients,
// so only enabled/mode fields need validation on apply.
typedef struct __attribute__((packed)) {
    uint8_t  enabled;                // 0/1
    uint8_t  center_mode;            // UPMIX_CENTER_* (0-1)
    uint8_t  surround_mode;          // UPMIX_SURROUND_* (0-2)
    int8_t   presence_q1;            // V26+: presence bell dB * 2 (was reserved)
    float    strength_pct;
    float    center_width_pct;
    float    corr_threshold_pct;
    float    attack_ms;
    float    release_ms;
    float    detector_hpf_hz;
    float    surround_delay_ms;
    float    surround_hpf_hz;
    float    surround_lpf_hz;
    float    decorr_pct;
} WireUpmixParams;                   // 44 bytes

// ============================================================================
// Section 23: Subharmonic Synthesizer (16 bytes); V29+
// ============================================================================
//
// dbx-style octave divider (see subharm.h and
// Documentation/Features/subharmonic_synth_spec.md).  One global parameter
// set applied to the output channels selected by output_mask.
typedef struct __attribute__((packed)) {
    uint8_t  enabled;                // 0/1
    uint8_t  reserved0;              // Zero
    uint16_t output_mask;            // Bit k: subharm processes output channel k
    float    low_db;                 // 24-36 Hz band level, -30..+6 dB (-30 = off)
    float    high_db;                // 36-56 Hz band level, -30..+6 dB (-30 = off)
    float    boost_db;               // LF boost bell, 0..+6 dB
} WireSubharmParams;                 // 16 bytes

// ============================================================================
// Complete Packet
// ============================================================================
typedef struct __attribute__((packed)) {
    WireHeader          header;          //   16
    WireGlobalParams    global;          //   16
    WireCrossfeedParams crossfeed;       //   16
    WireLegacyChannels  legacy;          //   16
    WireChannelDelays   delays;          //   68  (17 channels)
    WireCrosspoint      crosspoints[WIRE_MAX_INPUT_CHANNELS][WIRE_MAX_OUTPUT_CHANNELS];  // 576 (8×9)
    WireOutputChannel   outputs[WIRE_MAX_OUTPUT_CHANNELS];               //  108
    WirePinConfig       pins;            //    8
    WireBandParams      eq[WIRE_MAX_CHANNELS][WIRE_MAX_BANDS];           // 3264 (17×12)
    WireChannelNames    channel_names;   //  544  (17 channels)
    WireI2SConfig       i2s_config;      //   16
    WireLevellerConfig  leveller;        //   20
    WirePreampConfig    preamp;          //   32  (8 inputs)
    WireMasterVolume    master_volume;   //   16
    WireInputConfig     input_config;    //   16
    WireLgSoundSync     lg_sound_sync;   //   16
    WireUserVolume      user_volume;     //   16
    WireDacHwMute       dac_hw_mute;     //   16
    WireCrossoverConfig crossovers;      // 1088 (17×4; input rows unused)
    WireAdatConfig      adat_config;     //    8
    WirePsybassParams   psybass;         //   24  (V23+)
    WireUpmixParams     upmix;           //   44  (V25+)
    WireSubharmParams   subharm;         //   16  (V29+)
} WireBulkParams;                        // Total: 5960 bytes (V29 appends the 16-byte subharm section)

#define WIRE_BULK_PARAMS_SIZE  sizeof(WireBulkParams)

// Backward compatibility is intentionally broken at V16 (unified channel model).
// Only the current full-size layout is accepted; there are no legacy size
// anchors or per-section version gates; every section is always present.
// bulk_params_apply() rejects any payload whose format_version != current or
// whose length != sizeof(WireBulkParams).
#define WIRE_BULK_PARAMS_MIN_SIZE   WIRE_BULK_PARAMS_SIZE

// Buffer size for USB stream transfer (must be power of 2, >= WIRE_BULK_PARAMS_SIZE).
// V29 is 5960 bytes (17-channel EQ/names/crossover + ADAT + leveller masks + psybass + upmixer + subharm); 8192 is the next power of 2.
// Shared by both platforms (the wire format is platform-independent).
#define WIRE_BULK_BUF_SIZE     8192

// Collect current live DSP state into wire format
void bulk_params_collect(WireBulkParams *out);

// Apply wire format to live DSP state.  Returns 0 on success, nonzero on error.
// Caller must recalculate filters and delays after this returns.
// If apply_pins is true, output pin assignments from the payload are applied.
int bulk_params_apply(const WireBulkParams *in, bool apply_pins);

#endif // BULK_PARAMS_H
