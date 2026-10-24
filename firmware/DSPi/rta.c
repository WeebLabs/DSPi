/*
 * rta.c -- Onboard spectrum analyser engine.
 *
 * One capture buffer rotates over the selected channels.  It fills from the
 * audio path, is transformed in place from the main loop in bounded steps,
 * and publishes into per-channel band state and the shared bin frame.  All
 * state transitions happen on Core 0; Core 1 only ever copies samples
 * through the per-packet view and updates the bass state of its own rows.
 *
 * Design and protocol: Documentation/Features/spectrum_analyser_spec.md.
 */

#include "rta.h"
#include "audio_pipeline.h"
#include "usb_audio.h"
#include "bg_job.h"
#if PICO_RP2350
#include "upmix.h"
#endif
#include "pico/time.h"
#include "hardware/sync.h"
#include <string.h>

// ---------------------------------------------------------------------------
// Shared with the tap (see rta.h)
// ---------------------------------------------------------------------------

RtaPacketView rta_view;
rta_sample_t  rta_buf[RTA_MAX_POINTS];

// ---------------------------------------------------------------------------
// Engine state (Core 0 only)
// ---------------------------------------------------------------------------

enum { PH_FILL = 0, PH_FFT = 1, PH_FINISH = 2, PH_PUBLISH = 3 };

// Work between deadline checks in the transform job: FFT units (one butterfly
// is about 1 us on RP2350) and finish bins (about 1 us each). A step overruns
// its deadline by at most one chunk (current_architecture.md "Step budget").
#define RTA_FFT_UNITS_PER_CHUNK     16u
#define RTA_FINISH_BINS_PER_CHUNK   16u
// Chunk cost estimates: one per FFT stage (at most 8 at order 10), then the finish.
#define RTA_CHUNK_EST_FINISH        8u
_Static_assert(((RTA_ORDER_MAX - 1) & 1) + ((RTA_ORDER_MAX - 1) >> 1) + 3 <= RTA_CHUNK_EST_FINISH,
               "an FFT stage has no chunk estimate slot");

typedef struct {
    float    ema[RTA_MAX_BANDS - RTA_BASS_BANDS]; // FFT-band power EMA; bass owns its EMA
    uint8_t  avg[RTA_MAX_BANDS];
    uint8_t  peak[RTA_MAX_BANDS];
    uint8_t  seq;
    bool     seen;
    uint32_t t_us;
    // Peak-decay time carried between publishes so short intervals still
    // decay (20 dB/s over one 21 ms frame is under one 0.5 dB step).
    uint32_t peak_acc_us;
} RtaChannel;

static struct {
    RtaConfig cfg;
    RtaConfig cfg_next;
    uint8_t   cfg_next_src;        // ParamSource of the staging SET
    bool      cfg_pending;
    uint8_t   state;               // RTA_STATE_*
    uint8_t   phase;               // PH_*
    uint8_t   ch;                  // channel being captured / transformed
    uint8_t   order;
    RtaFftCursor fft;              // rta_fft_step cursor
    uint16_t  chunk_est_us[RTA_CHUNK_EST_FINISH + 1];   // last chunk time per stage
    RtaFinishCursor finish;        // rta_fft_finish_step cursor
    float     power[RTA_MAX_BANDS]; // band powers of the frame in flight
    uint16_t  n;                   // 1 << order
    uint16_t  wr;                  // capture write index
    uint32_t  frame_busy_us;       // transform time of the frame in flight
    const RtaBandTable *table;
    uint32_t  rate_hz;
    uint16_t  live_mask;
    uint32_t  last_read_us;
    bool      ever_read;
    RtaChannel ch_state[RTA_MAX_TRACKED];
    RtaBassCoeffs bass_coeffs;
    RtaBassChannel bass[RTA_MAX_TRACKED];
    uint16_t bass_mask;
    uint32_t bass_busy_acc_us;
    uint16_t bass_busy_us_per_s;
    // Bin frame: header | bins[n/2] | seq tail.  The finish steps fill
    // bin_stage and the publish copies it in, so readers never see it mid-write.
    uint8_t   bin_frame[RTA_BIN_FRAME_MAX];
    uint8_t   bin_stage[RTA_MAX_POINTS / 2];
    uint16_t  bin_len;             // fixed per config
    uint8_t   bin_seq;
    // Statistics, one-second windows.
    uint32_t  stat_window_us;
    uint32_t  busy_acc_us;
    uint16_t  frames_acc;
    uint16_t  busy_us_per_s;
    uint16_t  frames_per_s;
    uint16_t  last_frame_us;
} rta;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static inline uint8_t tap_width(uint8_t tap) {
    return (tap == RTA_TAP_INPUT) ? NUM_INPUT_CHANNELS : NUM_OUTPUT_CHANNELS;
}

static inline uint16_t tap_valid_mask(uint8_t tap) {
    return (uint16_t)((1u << tap_width(tap)) - 1u);
}

// Selected AND live right now.  Input side follows the active USB alt and
// the upmixer's derived rows; output side follows the matrix enables.
static uint16_t DSP_TIME_CRITICAL compute_live_mask(void) {
    uint16_t live = 0;
    if (rta.cfg.tap == RTA_TAP_INPUT) {
        uint8_t n = active_input_channel_count();
        if (n > NUM_INPUT_CHANNELS) n = NUM_INPUT_CHANNELS;
        live = (uint16_t)((1u << n) - 1u);
#if PICO_RP2350
        const UpmixCoeffs *um = (const UpmixCoeffs *)current_upmix_coeffs;
        if (um && n == NUM_STEREO_INPUTS) {
            live |= (uint16_t)(((1u << um->n_derived) - 1u) << NUM_STEREO_INPUTS);
        }
#endif
    } else {
        for (int o = 0; o < NUM_OUTPUT_CHANNELS; o++) {
            if (matrix_mixer.outputs[o].enabled) live |= (uint16_t)(1u << o);
        }
    }
    return live & rta.cfg.channel_mask & tap_valid_mask(rta.cfg.tap);
}

// Next set bit above cur, wrapping.  RTA_CH_NONE when mask is empty.
static uint8_t next_live(uint8_t cur, uint16_t mask) {
    if (!mask) return RTA_CH_NONE;
    for (int i = 1; i <= 16; i++) {
        uint8_t c = (uint8_t)((cur + i) & 15);
        if (mask & (1u << c)) return c;
    }
    return RTA_CH_NONE;
}

// Sequence numbers skip 0xFF, which marks a tail as invalid mid-write.
static uint8_t next_seq(uint8_t s) {
    s++;
    return (s == 0xFF) ? 0 : s;
}

static void clear_channel_state(void) {
    memset(rta.ch_state, 0, sizeof(rta.ch_state));
    memset(rta.bass, 0, sizeof(rta.bass));
    rta.bass_mask = 0;
}

static BgJob transform_job;

static void start_fill(uint8_t ch) {
    rta.ch = ch;
    rta.wr = 0;
    rta.fft.stage = 0;   // the permutation clears its own bitmap
    rta.fft.pos = 0;
    rta.frame_busy_us = 0;
    __dmb();
    rta.phase = PH_FILL;
}

// Everything the applied config and the current rate determine, so status
// and reads describe the applied config even while the engine is idle.
static void derive_layout(void) {
    rta.rate_hz = (uint32_t)audio_state.freq;
    rta_bass_configure(&rta.bass_coeffs, rta.rate_hz, rta.cfg.avg_ms);
    rta.order = rta.cfg.fft_order;
    rta.n = (uint16_t)(1u << rta.order);
    rta.table = rta_band_table(rta.rate_hz, rta.order);
    // Layout is fixed for the life of a config so the seq tail never moves
    // while a host is reading chunks.
    rta.bin_len = (uint16_t)(sizeof(RtaBinFrameHeader) + rta.n / 2 + 1);
    memset(rta.bin_frame, 0, sizeof(rta.bin_frame));
    rta.bin_frame[rta.bin_len - 1] = 0xFF;
}

// Begin a fresh capture from the lowest live channel.
static void arm(void) {
    derive_layout();
    rta.live_mask = compute_live_mask();
    uint8_t first = next_live(15, rta.live_mask);
    start_fill(first);
    rta.state = (first != RTA_CH_NONE && rta.table) ? RTA_STATE_CAPTURING : RTA_STATE_IDLE;
    // A start counts as a read, or auto-off would undo it on the next pass.
    rta.last_read_us = time_us_32();
    rta.ever_read = true;
}

static void stop_engine(void) {
    bg_job_cancel(&transform_job);
    rta.state = RTA_STATE_IDLE;
    rta.live_mask = 0;
    rta_view.tap = RTA_TAP_NONE;
    rta_view.take = 0;
    rta_view.bass_mask = 0;
    __dmb();
}

// ---------------------------------------------------------------------------
// Publish
// ---------------------------------------------------------------------------

// Whole 0.5 dB steps of peak decay earned since the last publish, carrying
// the unspent time in *acc.
static uint32_t peak_drop_steps(uint32_t *acc, uint32_t dt_us) {
    uint32_t rate = (uint32_t)rta.cfg.peak_decay_db_s * 2u;   // steps per second
    if (rate == 0) { *acc = 0; return 0; }
    uint32_t total = *acc + dt_us;
    uint32_t steps = (uint32_t)(((uint64_t)rate * total) / 1000000u);
    *acc = total - (uint32_t)(((uint64_t)steps * 1000000u) / rate);
    return steps;
}

// Fold one band power into the channel's average and peak.  dt drives the
// EMA coefficient so the time constant stays in real time however many
// channels share the rotation.
static void update_band(RtaChannel *c, uint8_t b, float power, uint32_t dt_us,
                        uint32_t drop, bool first) {
    if (b < RTA_BASS_BANDS) {
        c->avg[b] = rta_level_from_power(power);
    } else {
        uint8_t index = b - RTA_BASS_BANDS;
        float tau_us = (float)rta.cfg.avg_ms * 1000.0f;
        if (first || tau_us <= 0.0f) {
            c->ema[index] = power;
        } else {
            float a = (float)dt_us / (tau_us + (float)dt_us);
            c->ema[index] += a * (power - c->ema[index]);
        }
        c->avg[b] = rta_level_from_power(c->ema[index]);
    }

    uint8_t inst = rta_level_from_power(power);
    if (rta.cfg.peak_decay_db_s == 0 || first) {
        c->peak[b] = inst;
    } else {
        uint8_t decayed = (drop >= c->peak[b]) ? 0 : (uint8_t)(c->peak[b] - drop);
        c->peak[b] = (inst > decayed) ? inst : decayed;
    }
}

static void publish(const float *power) {
    RtaChannel *c = &rta.ch_state[rta.ch];
    const RtaBandTable *t = rta.table;
    uint32_t now = time_us_32();
    bool first = !c->seen;
    uint32_t dt = first ? 0 : (now - c->t_us);
    uint32_t drop = peak_drop_steps(&c->peak_acc_us, dt);
    for (uint8_t b = 0; b < t->n_bands; b++) {
        float p = b < RTA_BASS_BANDS
            ? rta_bass_power(&rta.bass_coeffs, &rta.bass[rta.ch], b) : power[b];
        update_band(c, b, p, dt, drop, first);
    }
    c->t_us = now;
    c->seen = true;
    c->seq++;

    // Bins, header and tail change together in this call; a chunked reader
    // spanning two publishes sees head and tail disagree and re-reads.
    memcpy(rta.bin_frame + sizeof(RtaBinFrameHeader), rta.bin_stage, rta.n / 2);
    RtaBinFrameHeader *h = (RtaBinFrameHeader *)rta.bin_frame;
    h->version = RTA_CFG_VERSION;
    h->channel = rta.ch;
    h->fft_order = rta.order;
    h->sample_rate_hz = rta.rate_hz;
    h->n_bins = (uint16_t)(rta.n / 2);
    h->reserved[0] = h->reserved[1] = h->reserved[2] = 0;
    rta.bin_seq = next_seq(rta.bin_seq);
    h->seq = rta.bin_seq;
    rta.bin_frame[rta.bin_len - 1] = rta.bin_seq;

    rta.frames_acc++;
    rta.last_frame_us = (uint16_t)(rta.frame_busy_us > 0xFFFF ? 0xFFFF : rta.frame_busy_us);
}

// ---------------------------------------------------------------------------
// Transform steps (main loop)
// ---------------------------------------------------------------------------

// Move to the next live channel, or stop if none.
static void advance(void) {
    rta.live_mask = compute_live_mask();
    uint8_t nx = next_live(rta.ch, rta.live_mask);
    if (nx == RTA_CH_NONE) {
        stop_engine();
        return;
    }
    start_fill(nx);
}

// Transform job (bg_job.h), started by rta_packet_end() when a frame fills and
// finished once that frame is published. The publish is indivisible, so it
// only ever starts a step and never lands on top of one nearly out of time.
static bool transform_step(void *ctx, uint32_t deadline_us) {
    (void)ctx;
    uint32_t t0 = time_us_32(), t_frame = t0;
    bool worked = false, published = false;
    while (rta.state != RTA_STATE_IDLE && rta.phase != PH_FILL) {
        if (rta.phase == PH_PUBLISH) {
            if (worked) break;
            // Charge the transform to the frame before publish reads the total.
            uint32_t now = time_us_32();
            rta.frame_busy_us += now - t_frame;
            publish(rta.power);
            advance();
            published = true;
            break;
        }
        // Chunk costs differ by stage, so each stage's last chunk predicts its next.
        uint8_t slot = rta.phase == PH_FFT ? rta.fft.stage : RTA_CHUNK_EST_FINISH;
        if (slot > RTA_CHUNK_EST_FINISH) slot = RTA_CHUNK_EST_FINISH;
        if (worked && bg_out_of_time(deadline_us, rta.chunk_est_us[slot])) break;
        uint32_t c0 = time_us_32();
        if (rta.phase == PH_FFT) {
            if (rta_fft_step(rta_buf, rta.order, &rta.fft, RTA_FFT_UNITS_PER_CHUNK)) {
                rta_fft_finish_begin(&rta.finish);
                rta.phase = PH_FINISH;
            }
        } else if (rta_fft_finish_step(&rta.finish, rta_buf, rta.order, rta.table, rta.power,
                                       rta.bin_stage, RTA_FINISH_BINS_PER_CHUNK)) {
            rta.phase = PH_PUBLISH;
        }
        uint32_t dt = time_us_32() - c0;
        rta.chunk_est_us[slot] = (uint16_t)(dt > 0xFFFFu ? 0xFFFFu : dt);
        worked = true;
    }
    uint32_t now = time_us_32();
    if (!published) rta.frame_busy_us += now - t_frame;
    rta.busy_acc_us += now - t0;
    return published || rta.state == RTA_STATE_IDLE || rta.phase == PH_FILL;
}

static BgJob transform_job = { BG_JOB_RTA_STEP, transform_step, NULL, false };

static void tick_stats(uint32_t now) {
    uint32_t elapsed = now - rta.stat_window_us;
    if (elapsed < 1000000u) return;
    rta.stat_window_us = now;
    uint32_t b = rta.busy_acc_us > 0xFFFF ? 0xFFFF : rta.busy_acc_us;
    uint32_t x = rta.busy_us_per_s;
    rta.busy_us_per_s = (uint16_t)(x - ((x + 3u) >> 2) + (b >> 2));
    uint64_t measured = (uint64_t)rta.bass_busy_acc_us * 1000000u / elapsed;
    uint32_t bass = measured > 0xFFFF ? 0xFFFF : (uint32_t)measured;
    rta.bass_busy_us_per_s = (uint16_t)bass;
    rta.bass_busy_acc_us = 0;
    rta.frames_per_s = rta.frames_acc;
    rta.busy_acc_us = 0;
    rta.frames_acc = 0;
}

// A channel that stops being tapped mid-fill (output disabled, USB alt
// change, upmixer parked) would never fill; move on instead of stalling.
static void check_fill_liveness(void) {
    if (rta.phase != PH_FILL) return;
    uint16_t live = compute_live_mask();
    if (!(live & (1u << rta.ch))) advance();
}

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------

static bool config_valid(const RtaConfig *c) {
    if (c->version != RTA_CFG_VERSION) return false;
    if (c->tap != RTA_TAP_INPUT && c->tap != RTA_TAP_OUTPUT) return false;
    if (c->fft_order < RTA_ORDER_MIN || c->fft_order > RTA_ORDER_MAX) return false;
    if ((c->channel_mask & tap_valid_mask(c->tap)) == 0) return false;
    return true;
}

static void apply_config(void) {
    RtaConfig n = rta.cfg_next;
    if (n.avg_ms > 10000) n.avg_ms = 10000;
    if (n.peak_decay_db_s > 100) n.peak_decay_db_s = 100;
    n.channel_mask &= tap_valid_mask(n.tap);
    n.reserved0 = 0;
    n.reserved = 0;
    bool restart = n.tap != rta.cfg.tap || n.channel_mask != rta.cfg.channel_mask ||
                   n.fft_order != rta.cfg.fft_order;
    rta.cfg = n;
    rta_bass_configure(&rta.bass_coeffs, rta.rate_hz, n.avg_ms);
    if (restart) {
        bool was_running = (rta.state != RTA_STATE_IDLE);
        stop_engine();
        clear_channel_state();
        if (was_running) arm(); else derive_layout();
    }
    // Every applied SET is announced, even an identical one: a host uses the
    // echo to confirm its push landed (spectrum_analyser_spec.md 7.1).
    _Static_assert(sizeof(RtaConfig) == NOTIFY_RTA_CONFIG_LEN, "RTA_CONFIG payload");
    notify_push_rta_config(&rta.cfg, (ParamSource)rta.cfg_next_src);
}

bool rta_set_config(const void *payload, uint16_t len, ParamSource src) {
    if (len != sizeof(RtaConfig)) return false;
    RtaConfig c;
    memcpy(&c, payload, sizeof(c));
    if (!config_valid(&c)) return false;
    rta.cfg_next = c;
    rta.cfg_next_src = (uint8_t)src;
    rta.cfg_pending = true;
    return true;
}

void rta_get_config(RtaConfig *out) { *out = rta.cfg; }

void rta_get_caps(RtaCaps *out) {
    memset(out, 0, sizeof(*out));
    out->version = RTA_CFG_VERSION;
    out->input_channels = NUM_INPUT_CHANNELS;
    out->output_channels = NUM_OUTPUT_CHANNELS;
    out->fft_order_min = RTA_ORDER_MIN;
    out->fft_order_max = RTA_ORDER_MAX;
#if PICO_RP2350
    out->fft_order_default = 10;
    out->dynamic_range_db = 120;
#else
    out->fft_order_default = 9;
    out->dynamic_range_db = 78;   // measured Q15 floor -78.5 dBFS (tools/rta_test)
#endif
    out->bass_bands = RTA_BASS_BANDS;
    out->bass_dynamic_range_db = 70;
    out->max_bands = RTA_MAX_BANDS;
    out->level_zero = RTA_LEVEL_ZERO_DBFS;
    out->idle_timeout_ms = RTA_IDLE_TIMEOUT_MS;
    out->max_bin_frame = RTA_BIN_FRAME_MAX;
}

uint16_t rta_get_caps_centres(uint8_t chunk, uint8_t *out) {
    if (chunk == 0) return 0;
    uint8_t first = (uint8_t)((chunk - 1) * 32);
    if (first >= RTA_MAX_BANDS) return 0;
    uint8_t n = (uint8_t)(RTA_MAX_BANDS - first);
    if (n > 32) n = 32;
    for (uint8_t i = 0; i < n; i++) {
        uint16_t hz = rta_band_centre_hz((uint8_t)(first + i));
        out[2 * i] = (uint8_t)(hz & 0xFF);
        out[2 * i + 1] = (uint8_t)(hz >> 8);
    }
    return (uint16_t)(n * 2);
}

// ---------------------------------------------------------------------------
// Reads
// ---------------------------------------------------------------------------

static uint16_t age_ms(uint32_t now, uint32_t then, bool valid) {
    if (!valid) return 0xFFFF;
    uint32_t ms = (now - then) / 1000u;
    return (ms > 0xFFFE) ? 0xFFFE : (uint16_t)ms;
}

bool rta_get_band_frame(uint8_t channel, RtaBandFrame *out) {
    if (channel >= tap_width(rta.cfg.tap)) return false;
    const RtaChannel *c = &rta.ch_state[channel];
    uint32_t now = time_us_32();
    memset(out, 0, sizeof(*out));
    out->version = RTA_CFG_VERSION;
    out->channel = channel;
    out->seq = c->seq;
    out->n_bands = rta.table ? rta.table->n_bands : 0;
    out->age_ms = age_ms(now, c->t_us, c->seen);
    memcpy(out->avg, c->avg, RTA_MAX_BANDS);
    memcpy(out->peak, c->peak, RTA_MAX_BANDS);
    return true;
}

const uint8_t *rta_bin_frame(uint16_t *len) {
    *len = rta.bin_len;
    return rta.bin_frame;
}

void rta_get_status(RtaStatus *out) {
    uint32_t now = time_us_32();
    memset(out, 0, sizeof(*out));
    out->version = RTA_CFG_VERSION;
    out->state = rta.state;
    if (rta.state != RTA_STATE_IDLE && rta.phase != PH_FILL) {
        out->state = RTA_STATE_TRANSFORMING;
    }
    out->tap = rta.cfg.tap;
    out->channel = (rta.state == RTA_STATE_IDLE) ? RTA_CH_NONE : rta.ch;
    out->live_mask = rta.live_mask;
    out->live_count = (uint8_t)__builtin_popcount(rta.live_mask);
    out->frames_per_s = rta.frames_per_s;
    out->busy_us_per_s = rta.busy_us_per_s;
    out->last_frame_us = rta.last_frame_us;
    out->idle_ms = age_ms(now, rta.last_read_us, rta.ever_read);
    out->sample_rate_hz = rta.rate_hz;
    out->first_band = rta.table ? 0 : 0xFF;
    out->bass_busy_us_per_s = rta.bass_busy_us_per_s;
}

bool rta_control(uint8_t action) {
    switch (action) {
    case RTA_CTL_STOP:
        stop_engine();
        return true;
    case RTA_CTL_START:
        if (rta.state == RTA_STATE_IDLE) { clear_channel_state(); arm(); }
        return true;
    case RTA_CTL_RESET_AVG:
        // Clear the published values too, so a read before the next publish
        // shows an empty frame rather than the pre-reset picture.
        memset(rta.bass, 0, sizeof(rta.bass));
        for (int i = 0; i < RTA_MAX_TRACKED; i++) {
            rta.ch_state[i].seen = false;
            memset(rta.ch_state[i].avg, 0, RTA_MAX_BANDS);
            memset(rta.ch_state[i].peak, 0, RTA_MAX_BANDS);
        }
        return true;
    default:
        return false;
    }
}

void rta_note_read(void) {
    rta.last_read_us = time_us_32();
    rta.ever_read = true;
    if (rta.state == RTA_STATE_IDLE && !(rta.cfg.flags & RTA_FLAG_MANUAL)) {
        clear_channel_state();
        arm();
    }
}

uint16_t rta_live_mask(void) { return rta.live_mask; }

// ---------------------------------------------------------------------------
// Pipeline integration (Core 0)
// ---------------------------------------------------------------------------

void __not_in_flash_func(rta_packet_begin)(uint32_t sample_count) {
    RtaPacketView *v = &rta_view;
    v->tap = RTA_TAP_NONE;
    v->take = 0;
    v->produced = 0;
    v->bass_tap = rta.cfg.tap;
    v->bass_mask = rta.state != RTA_STATE_IDLE ? compute_live_mask() : 0;
    // Core 0, before dispatch: reset channels that disappear or return.
    uint16_t changed = rta.state != RTA_STATE_IDLE ? v->bass_mask ^ rta.bass_mask : 0;
    for (int ch = 0; ch < RTA_MAX_TRACKED; ch++) {
        v->bass_busy_us[ch] = 0;
        if (changed & (1u << ch)) {
            memset(&rta.bass[ch], 0, sizeof(rta.bass[ch]));
            memset(&rta.ch_state[ch], 0, sizeof(rta.ch_state[ch]));
        }
    }
    rta.bass_mask = v->bass_mask;
    if (rta.state == RTA_STATE_CAPTURING && rta.phase == PH_FILL) {
        uint32_t room = rta.n - rta.wr;
        v->take = (uint16_t)(sample_count < room ? sample_count : room);
        v->wr = rta.wr;
        v->row = rta.ch;
        v->tap = rta.cfg.tap;
    }
    __dmb();
}

void __not_in_flash_func(rta_packet_end)(uint32_t sample_count) {
    (void)sample_count;
    __dmb();
    RtaPacketView *v = &rta_view;
    for (int ch = 0; ch < RTA_MAX_TRACKED; ch++)
        rta.bass_busy_acc_us += v->bass_busy_us[ch];
    v->bass_mask = 0;
    // Advance by what the tap actually copied, never by what was offered: a
    // row that was not tapped this packet leaves the count at 0.
    if (v->tap != RTA_TAP_NONE && v->produced) {
        rta.wr = (uint16_t)(rta.wr + v->produced);
        if (rta.wr >= rta.n) {
            rta.fft.stage = 0;
            rta.fft.pos = 0;
            rta.phase = PH_FFT;
            bg_job_start(&transform_job);
        }
    }
    v->take = 0;
    v->tap = RTA_TAP_NONE;
}

void DSP_TIME_CRITICAL rta_tap(uint8_t tap, uint8_t row, const rta_pipe_t *buf, uint32_t n) {
    RtaPacketView *v = &rta_view;
    if (v->bass_tap == tap && row < RTA_MAX_TRACKED && (v->bass_mask & (1u << row))) {
        uint32_t start = time_us_32();
        rta_bass_push(&rta.bass_coeffs, &rta.bass[row], buf, n);
        v->bass_busy_us[row] = time_us_32() - start;
    }
    if (v->tap != tap || v->row != row || !v->take) return;
    uint32_t take = v->take;
    rta_sample_t *dst = &rta_buf[v->wr];
#if PICO_RP2350
    for (uint32_t i = 0; i < take; i++) dst[i] = buf[i];
#else
    for (uint32_t i = 0; i < take; i++) {
        int32_t s = buf[i] >> 14;               // full scale 2^29 -> Q15
        if (s > 32767) s = 32767; else if (s < -32768) s = -32768;
        dst[i] = (int16_t)s;
    }
#endif
    v->produced = (uint16_t)take;
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------

void rta_service(void) {
    if (rta.cfg_pending) {
        rta.cfg_pending = false;
        apply_config();
    }
    uint32_t now = time_us_32();
    tick_stats(now);
    if (rta.state == RTA_STATE_IDLE) return;

    // MANUAL hands the run state to CONTROL entirely: no auto-start, no
    // auto-off.
    if (!(rta.cfg.flags & RTA_FLAG_MANUAL) &&
        (uint32_t)(now - rta.last_read_us) > (uint32_t)RTA_IDLE_TIMEOUT_MS * 1000u) {
        stop_engine();
        return;
    }
    // A rate change without a restart call still gets caught here.
    if ((uint32_t)audio_state.freq != rta.rate_hz) {
        rta_restart();
        return;
    }
    check_fill_liveness();
}

void rta_restart(void) {
    if (rta.state == RTA_STATE_IDLE) return;
    stop_engine();
    clear_channel_state();
    arm();
}

void rta_init(void) {
    memset(&rta, 0, sizeof(rta));
    rta.cfg.version = RTA_CFG_VERSION;
    rta.cfg.tap = RTA_TAP_OUTPUT;
    rta.cfg.channel_mask = tap_valid_mask(RTA_TAP_OUTPUT);
#if PICO_RP2350
    rta.cfg.fft_order = 10;
#else
    rta.cfg.fft_order = 9;
#endif
    rta.cfg.avg_ms = 250;
    rta.cfg.peak_decay_db_s = 20;
    rta.stat_window_us = time_us_32();
    derive_layout();
    stop_engine();
    bg_job_register(&transform_job);
}
