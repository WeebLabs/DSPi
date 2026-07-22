/*
 * adat_input.c; ADAT lightpipe 8-channel input (RP2350 only)
 *
 * Receiver architecture:
 *   PIO1 SM2 runs the adat_rx NRZI decoder (adat_input.pio) at clock
 *   divider 1.0, counting each wire bit cell with a 2-cycle poll loop
 *   whose length is set per sample rate (see adat_rx_set_cell). It emits
 *   the DECODED bitstream (1 = line transition), MSB first, autopushed
 *   every 32 bits. DMA channel 15 streams the words into an 8 KB ring
 *   (ENDLESS transfer count + hardware write-address wrap: a free-running
 *   ring with no IRQ and no reload channel). The main-loop poll consumes
 *   the ring as a bit stream.
 *
 * Frame handling is CPU-side. The sync header's 10-zero run cannot occur in
 * channel data (a forced 1 every 5th bit bounds data runs to 4), so a scan
 * for the 12-bit structural pattern [1][10x0][1] finds the frame boundary;
 * after that, frames sit at a FIXED bit offset (edge resync in the PIO
 * absorbs clock offset, so exactly 256 bits arrive per frame) and each
 * frame's header is verified before its samples are trusted. Header
 * verification doubles as the loss detector: a dark or unplugged line
 * decodes as zeros, which never match the header.
 *
 * Clock modes: see adat_input.h. Slave mode acquires by trying the two
 * supported cell timings (48 kHz, then 44.1 kHz) and accepting only a run of
 * valid frame headers. Once locked it measures the wire rate from the DMA
 * write pointer (32 ms fast windows plus a dual-anchor long window for the
 * ~0.1 ppm servo reference) and servos all outputs via input_servo_apply().
 * Master mode skips all of that; the stream is already in our clock domain.
 */

#include "adat_input.h"

#if PICO_RP2350

#include "audio_input.h"
#include "audio_pipeline.h"
#include "config.h"
#include "usb_audio.h"
#include "input_servo.h"
#include "notify.h"

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#include "adat_input.pio.h"

#include <stdio.h>
#include <string.h>

// ============================================================================
// RESOURCES
// ============================================================================

#define ADAT_RX_PIO         pio1    // PDM owns SM0, ADAT TX owns SM1
#define ADAT_RX_SM          2
#define ADAT_RX_DMA_CH      15      // the one permanently free DMA channel
#define ADAT_RX_RING_WORDS  2048u   // 8 KB = 256 frames = 5.3 ms at 48 kHz
#define ADAT_RX_RING_BITS   13u     // log2(ring bytes), for the DMA address wrap
#define ADAT_RX_RING_MASK   (ADAT_RX_RING_WORDS - 1u)
#define ADAT_RX_FRAME_WORDS 8u      // 256 bits per frame

_Static_assert((ADAT_RX_RING_WORDS & ADAT_RX_RING_MASK) == 0,
               "ADAT RX ring must be a power of two (DMA address wrap)");
_Static_assert(ADAT_RX_RING_WORDS % ADAT_RX_FRAME_WORDS == 0,
               "ring must hold whole frames so laps preserve frame phase");

// Batching: same rationale as I2S_INPUT_MIN_BLOCK (i2s_input.c); ~1 ms at 48k.
#define ADAT_INPUT_MIN_BLOCK    48u

// Sync/verify tuning
#define ADAT_SYNC_VERIFY_FRAMES 8u     // consecutive good headers before LOCKED
#define ADAT_HDR_FAIL_LIMIT     2u     // consecutive bad headers = lock loss
#define ADAT_SCAN_WORDS_PER_POLL 256u  // bounds sync-search CPU per poll
#define ADAT_RX_PROBE_DWELL_US  10000u // per-rate header-search dwell

// Slave-mode rate measurement (i2s_input.c slave machine pattern)
#define ADAT_RX_WINDOW_US       32000u     // fast measurement window
#define ADAT_RX_CLOCK_TIMEOUT_US 5000u     // safety net only; see rate machine
#define ADAT_RX_LONG_HALF_US    8000000ull // long-window anchor rotation
#define ADAT_RX_POLL_GAP_US     4000u      // stall guard (under ring fill time)
#define ADAT_RX_SERVO_INTERVAL  1000       // main-loop iterations (~20 ms)

// ============================================================================
// STATE
// ============================================================================

static uint32_t __attribute__((aligned(ADAT_RX_RING_WORDS * 4u)))
    adat_rx_ring[ADAT_RX_RING_WORDS];

static volatile AdatInputState adat_rx_state = ADAT_INPUT_INACTIVE;
static bool     adat_rx_running = false;
static uint8_t  adat_rx_active_pin = 0xFF;
static uint32_t adat_rx_prog_offset;
static bool     adat_rx_rate_ok = true;      // false = parked (rate > 48 kHz)
static uint32_t adat_rx_detected_rate = 0;   // Hz; master mode = device rate

// Frame cursor: frame bit 0 lives at bit `frame_bit` (0..31, MSB numbering)
// of ring word `rd_word`. Frames consume exactly 8 words, so frame_bit is
// constant while synced.
static uint32_t rd_word;
static uint32_t frame_bit;

// Sync search
static uint32_t scan_word;
static bool     sync_found;
static uint32_t verify_left;
static uint8_t  hdr_fail_run;

// Counters (status packet)
static uint8_t  adat_rx_lock_count;
static uint8_t  adat_rx_loss_count;
static uint8_t  adat_rx_slip_count;
static uint16_t adat_rx_header_err;

// Slave-mode rate measurement (see i2s_input.c for the anchor scheme)
static uint32_t meas_last_word;
static uint64_t meas_total_words;
static uint64_t meas_last_advance_us;
static uint64_t meas_last_poll_us;
static uint64_t meas_win_us;
static uint64_t meas_win_words;
static float    meas_hz_smooth;
static uint32_t meas_measured_hz;
static uint64_t long_us[2];
static uint64_t long_words[2];
static bool     long_valid;
static float    meas_hz_long;
static uint32_t adat_rx_servo_skip;

// Slave acquisition probes one exact decoder timing at a time. The DMA word
// rate is not trustworthy while the decoder cell is wrong: the emitted stream
// is corrupt and its rate depends partly on the decoder timeout.
static uint32_t probe_rate;
static uint64_t probe_started_us;

// ============================================================================
// HELPERS
// ============================================================================

static inline uint32_t adat_rx_write_index(void) {
    return (uint32_t)((dma_hw->ch[ADAT_RX_DMA_CH].write_addr -
                       (uintptr_t)adat_rx_ring) >> 2) & ADAT_RX_RING_MASK;
}

static void adat_rx_probe_arm(uint64_t now, uint32_t widx,
                              uint32_t preferred_rate);
static void adat_rx_meas_arm(uint64_t now, uint32_t widx);

static void adat_rx_set_state(AdatInputState st) {
    if (adat_rx_state == st) return;
    adat_rx_state = st;
    notify_push_adat_input_state((uint8_t)st,
                                 (st == ADAT_INPUT_LOCKED) ? adat_rx_detected_rate : 0,
                                 adat_clock_mode);
}

// Per-rate cell period. The SM runs at divider 1.0; the cell length is
// 2*Y+5 sys cycles via the Y reload (adat_input.pio). Cells must be odd
// (the poll loop counts in 2-cycle steps): at 307.2 MHz both rates are
// (27 at 44.1 kHz, 25 at 48 kHz). The 150 MHz fallback sys clock yields
// an even cell with degraded margins; ADAT input simply fails to lock
// there, which is acceptable for a safety-net clock. Never servoed: the
// per-edge re-anchoring absorbs percent-level offsets in both directions.
static void adat_rx_set_cell(uint32_t fs) {
    uint32_t sys = clock_get_hz(clk_sys);
    uint32_t denom = 256u * fs;
    uint32_t cell = (sys + denom / 2u) / denom;
    if ((cell & 1u) == 0) cell -= 1u;
    uint32_t yv = (cell - 5u) / 2u;
    if (yv < 2u) yv = 2u;
    if (yv > 31u) yv = 31u;   // set-immediate range
    pio_sm_exec(ADAT_RX_PIO, ADAT_RX_SM, pio_encode_set(pio_y, yv));
}

// Restart the sync search from ring position `from` (frames decoded so far
// stay delivered; audio resumes once a verified header run is found).
static void adat_rx_resync_scan(uint32_t from) {
    scan_word = from & ADAT_RX_RING_MASK;
    sync_found = false;
    verify_left = 0;
}

// Common lock-drop bookkeeping. The main loop reacts to RELOCKING exactly
// like a SPDIF lock loss (mute, drain, wait for relock + prefill).
static void adat_rx_drop_lock(void) {
    if (adat_rx_loss_count < 255) adat_rx_loss_count++;
    long_valid = false;
    uint32_t widx = adat_rx_write_index();
    if (adat_clock_mode == ADAT_CLOCK_MODE_SLAVE) {
        // Retry the last valid family first. A brief optical glitch can
        // recover immediately; a genuine family switch reaches the alternate
        // after one short probe dwell.
        uint32_t preferred = (adat_rx_detected_rate == 44100u) ? 44100u : 48000u;
        adat_rx_probe_arm(time_us_64(), widx, preferred);
    } else {
        adat_rx_resync_scan(widx);
    }
    adat_rx_set_state(ADAT_INPUT_RELOCKING);
}

// Structural header check ([1][10x0][1], user bits ignored) for the frame
// starting at bit k of ring word rd.
DSP_TIME_CRITICAL
static inline bool adat_rx_header_ok(uint32_t rd, uint32_t k) {
    uint32_t a = adat_rx_ring[rd & ADAT_RX_RING_MASK];
    uint32_t b = adat_rx_ring[(rd + 1u) & ADAT_RX_RING_MASK];
    uint64_t v = ((uint64_t)a << 32) | b;
    return ((uint32_t)(v >> (52u - k)) & 0xFFFu) == 0x801u;
}

// Decode the frame at (rd, k): verify the header, unstuff the 8 channel
// fields (exact inverse of adat_encode_frame in adat_output.c) and return
// the samples as int32 full-scale. False on header mismatch (samples
// untouched).
DSP_TIME_CRITICAL
static inline bool adat_rx_decode_frame(uint32_t rd, uint32_t k, int32_t *smp) {
    uint32_t w[8];
    if (k == 0) {
        for (uint32_t i = 0; i < 8u; i++)
            w[i] = adat_rx_ring[(rd + i) & ADAT_RX_RING_MASK];
    } else {
        uint32_t prev = adat_rx_ring[rd & ADAT_RX_RING_MASK];
        for (uint32_t i = 0; i < 8u; i++) {
            uint32_t next = adat_rx_ring[(rd + i + 1u) & ADAT_RX_RING_MASK];
            w[i] = (prev << k) | (next >> (32u - k));
            prev = next;
        }
    }

    if ((w[0] & 0xFFF00000u) != 0x80100000u) return false;

    for (uint32_t ch = 0; ch < 8u; ch++) {
        // 30-bit channel field at frame bit 16 + 30*ch, MSB first:
        // [1][n5][1][n4][1][n3][1][n2][1][n1][1][n0], nibbles MSB-first.
        uint32_t p = 16u + 30u * ch;
        uint32_t a = w[p >> 5];
        uint32_t b = ((p >> 5) < 7u) ? w[(p >> 5) + 1u] : 0u;
        uint32_t sh = p & 31u;
        uint32_t v = (uint32_t)(((((uint64_t)a << 32) | b) >> (34u - sh))) & 0x3FFFFFFFu;
        uint32_t s24 = (((v >> 25) & 0xFu) << 20) | (((v >> 20) & 0xFu) << 16) |
                       (((v >> 15) & 0xFu) << 12) | (((v >> 10) & 0xFu) << 8)  |
                       (((v >> 5)  & 0xFu) << 4)  |   (v         & 0xFu);
        smp[ch] = (int32_t)(s24 << 8);   // sign into bit 31: int32 full-scale
    }
    return true;
}

// Bounded search for the sync pattern in fresh decoded bits. A dark line
// decodes as all-zero words and is skipped without bit work. Runs only
// while unlocked (pipeline muted), so it stays in flash; noinline keeps it
// out of the RAM-resident poll body.
__attribute__((noinline))
static void adat_rx_scan(uint32_t widx) {
    uint32_t avail = (widx - scan_word) & ADAT_RX_RING_MASK;
    uint32_t budget = ADAT_SCAN_WORDS_PER_POLL;
    while (avail >= 2u && budget--) {
        uint32_t prev = adat_rx_ring[scan_word & ADAT_RX_RING_MASK];
        uint32_t curr = adat_rx_ring[(scan_word + 1u) & ADAT_RX_RING_MASK];
        if (prev | curr) {
            uint64_t b64 = ((uint64_t)prev << 32) | curr;
            for (uint32_t b = 0; b < 32u; b++) {
                if (((uint32_t)(b64 >> (52u - b)) & 0xFFFu) == 0x801u) {
                    rd_word = scan_word & ADAT_RX_RING_MASK;
                    frame_bit = b;
                    sync_found = true;
                    verify_left = ADAT_SYNC_VERIFY_FRAMES;
                    return;
                }
            }
        }
        scan_word = (scan_word + 1u) & ADAT_RX_RING_MASK;
        avail--;
    }
    // Never let the scan fall a full ring behind the writer.
    if (((widx - scan_word) & ADAT_RX_RING_MASK) > ADAT_RX_RING_WORDS - 64u)
        scan_word = (widx - 64u) & ADAT_RX_RING_MASK;
}

// Start (or switch) a slave-mode candidate. Bits already in the ring were
// decoded with the previous timing, so search only from the current writer.
// One partly assembled PIO word may straddle the retune; the scanner skips it
// naturally before reaching clean candidate-rate words.
static void adat_rx_probe_arm(uint64_t now, uint32_t widx,
                              uint32_t preferred_rate) {
    probe_rate = (preferred_rate == 44100u) ? 44100u : 48000u;
    probe_started_us = now;
    adat_rx_set_cell(probe_rate);
    adat_rx_resync_scan(widx);
}

// Prove the current exact cell timing with a consecutive structural-header
// run. A wrong candidate can contain an isolated accidental 0x801 pattern,
// but cannot preserve it at the fixed eight-word frame stride for the full
// verification run.
static bool adat_rx_probe_poll(uint64_t now, uint32_t widx) {
    if (!sync_found) adat_rx_scan(widx);

    while (sync_found && verify_left) {
        if (((widx - rd_word) & ADAT_RX_RING_MASK) < ADAT_RX_FRAME_WORDS + 1u)
            break;
        if (!adat_rx_header_ok(rd_word, frame_bit)) {
            adat_rx_header_err++;
            adat_rx_resync_scan(rd_word);
            break;
        }
        rd_word = (rd_word + ADAT_RX_FRAME_WORDS) & ADAT_RX_RING_MASK;
        verify_left--;
    }

    if (sync_found && verify_left == 0u) {
        adat_rx_detected_rate = probe_rate;
        // Begin fine-rate/servo measurement only after the decoder timing is
        // known-correct. The previous wrong-cell DMA-rate bootstrap was the
        // source of the real-hardware 44.1 kHz acquisition failure.
        adat_rx_meas_arm(now, widx);
        meas_hz_smooth = (float)probe_rate;
        long_us[0] = long_us[1] = now;
        long_words[0] = long_words[1] = 0;
        long_valid = false;
        adat_rx_set_state(ADAT_INPUT_SYNCING);
        return true;
    }

    if (now - probe_started_us >= ADAT_RX_PROBE_DWELL_US) {
        uint32_t alternate = (probe_rate == 48000u) ? 44100u : 48000u;
        adat_rx_probe_arm(now, widx, alternate);
    }
    return false;
}

// ============================================================================
// SLAVE-MODE LOCKED-RATE MEASUREMENT
// ============================================================================

// Re-anchor the measurement accumulators (start, stall, divider change).
static void adat_rx_meas_arm(uint64_t now, uint32_t widx) {
    meas_last_word = widx;
    meas_total_words = 0;
    meas_last_advance_us = now;
    meas_win_us = now;
    meas_win_words = 0;
    meas_hz_smooth = 0.0f;
    meas_measured_hz = 0;
    long_valid = false;
}

// Snap a measured wire rate to a supported ADAT rate (2% tolerance; 44.1 and
// 48 are 8.8% apart so the windows cannot overlap). 96k/SMUX streams never
// snap, which is the "no SMUX" enforcement. 0 = no match.
static uint32_t adat_rx_snap_rate(float hz) {
    static const uint32_t rates[2] = {44100, 48000};
    for (int i = 0; i < 2; i++) {
        float r = (float)rates[i];
        float d = hz - r;
        if (d < 0) d = -d;
        if (d <= r * 0.02f) return rates[i];
    }
    return 0;
}

DSP_TIME_CRITICAL __attribute__((noinline))
static void adat_rx_rate_machine(uint64_t now, uint32_t widx, bool stall) {
    if (stall) {
        // Word delta is ambiguous after a main-loop stall; re-anchor rather
        // than measure through it (i2s_input.c poll-gap watchdog pattern).
        meas_last_word = widx;
        meas_last_advance_us = now;
        meas_win_us = now;
        meas_win_words = meas_total_words;
        long_us[0] = long_us[1] = now;
        long_words[0] = long_words[1] = meas_total_words;
        long_valid = false;
        return;
    }

    uint32_t delta = (widx - meas_last_word) & ADAT_RX_RING_MASK;
    meas_last_word = widx;
    meas_total_words += delta;
    if (delta) meas_last_advance_us = now;

    // Safety net only: a dark line still decodes zero words at the nominal
    // rate, so real signal loss is caught by header verification, not here.
    if (adat_rx_state == ADAT_INPUT_LOCKED &&
        (now - meas_last_advance_us) > ADAT_RX_CLOCK_TIMEOUT_US) {
        adat_rx_drop_lock();
        meas_measured_hz = 0;
        meas_win_us = now;
        meas_win_words = meas_total_words;
        return;
    }

    uint64_t span = now - meas_win_us;
    if (span < ADAT_RX_WINDOW_US) return;

    // Fast-window rate: 8 ring words per frame
    float hz = (float)(uint32_t)(meas_total_words - meas_win_words) *
               (1e6f / 8.0f) / (float)span;
    meas_win_us = now;
    meas_win_words = meas_total_words;
    meas_measured_hz = (uint32_t)(hz + 0.5f);

    uint32_t snapped = adat_rx_snap_rate(hz);

    if (snapped != adat_rx_detected_rate) {
        // Wire rate changed (or a glitch burst disturbed the window). Header
        // verification chooses the new exact family during re-lock.
        adat_rx_drop_lock();
        return;
    }
    if (adat_rx_state == ADAT_INPUT_LOCKED) {
        meas_hz_smooth += 0.25f * (hz - meas_hz_smooth);
        // Long dual-anchor window: rotate every 8 s, measure 8-16 s span
        if (now - long_us[1] >= ADAT_RX_LONG_HALF_US) {
            long_us[0] = long_us[1];     long_words[0] = long_words[1];
            long_us[1] = now;            long_words[1] = meas_total_words;
        }
        uint64_t long_span = now - long_us[0];
        if (long_span >= ADAT_RX_LONG_HALF_US) {
            meas_hz_long = (float)(meas_total_words - long_words[0]) *
                           (1e6f / 8.0f) / (float)long_span;
            long_valid = true;
        }
    }
}

// ============================================================================
// PUBLIC API
// ============================================================================

void adat_input_init(void) {
    adat_rx_state = ADAT_INPUT_INACTIVE;
    adat_rx_running = false;
}

void adat_input_start(void) {
    if (adat_rx_running) return;
    uint8_t pin = adat_input_pin;
    if (pin == 0xFF || pin >= NUM_BANK0_GPIOS) {
        printf("ADAT RX: no pin configured, not starting\n");
        return;
    }

    pio_sm_claim(ADAT_RX_PIO, ADAT_RX_SM);
    adat_rx_prog_offset = pio_add_program(ADAT_RX_PIO, &adat_rx_program);
    dma_channel_claim(ADAT_RX_DMA_CH);

    // Input enable only; funcsel is never touched, so the RX pin may be the
    // ADAT TX pin itself (PIO reads the pad regardless of function): that is
    // the zero-hardware loopback self-test.
    gpio_set_input_enabled(pin, true);
#if HAS_PADS_BANK0_ISOLATION
    // RP2350 pads reset isolated (ISO=1) and gpio_set_input_enabled does not
    // clear it, so an external signal never reaches the PIO input mux; only
    // gpio_set_function clears ISO, which is why same-pin loopback worked (the
    // TX pio_gpio_init unisolated the shared pad).  Clear ISO directly rather
    // than via pio_gpio_init to keep funcsel untouched for the loopback case.
    hw_clear_bits(&pads_bank0_hw->io[pin], PADS_BANK0_GPIO0_ISO_BITS);
#endif
    adat_rx_active_pin = pin;

    pio_sm_config c = adat_rx_program_get_default_config(adat_rx_prog_offset);
    sm_config_set_jmp_pin(&c, pin);
    sm_config_set_in_shift(&c, false, true, 32);   // shift left (MSB first), autopush
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv_int_frac(&c, 1, 0);       // full sys_clk, no jitter
    pio_sm_init(ADAT_RX_PIO, ADAT_RX_SM,
                adat_rx_prog_offset + adat_rx_wrap_target, &c);
    // OSR = all-ones so `in osr, 1` emits a 1 (X and Y are both counters)
    pio_sm_exec(ADAT_RX_PIO, ADAT_RX_SM, pio_encode_mov_not(pio_osr, pio_null));

    // rate_ok is a master-mode park flag only: false means the device rate
    // (the rate authority in master mode) is above ADAT's 48 kHz ceiling.
    // Slave mode always runs regardless of the current pipeline rate; the
    // wire rate is detected and a >48k device rate at switch-in resolves
    // itself through the deferred rate change after lock (the switch-in
    // mute holds until then).
    adat_rx_rate_ok = (adat_clock_mode == ADAT_CLOCK_MODE_SLAVE) ||
                      (audio_state.freq <= 48000u);
    adat_rx_set_cell((adat_clock_mode == ADAT_CLOCK_MODE_SLAVE)
                         ? 48000u   // probe_arm below owns the candidate cell
                         : (adat_rx_rate_ok ? audio_state.freq : 48000u));

    // Free-running ring: ENDLESS transfer count (never decrements) plus the
    // hardware write-address wrap. One channel, no IRQ, no reload channel.
    dma_channel_config dc = dma_channel_get_default_config(ADAT_RX_DMA_CH);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
    channel_config_set_read_increment(&dc, false);
    channel_config_set_write_increment(&dc, true);
    channel_config_set_ring(&dc, true, ADAT_RX_RING_BITS);
    channel_config_set_dreq(&dc, pio_get_dreq(ADAT_RX_PIO, ADAT_RX_SM, false));
    dma_channel_configure(ADAT_RX_DMA_CH, &dc,
                          adat_rx_ring, &ADAT_RX_PIO->rxf[ADAT_RX_SM], 0, false);
    dma_channel_hw_addr(ADAT_RX_DMA_CH)->al1_transfer_count_trig =
        ((uint32_t)DMA_CH0_TRANS_COUNT_MODE_VALUE_ENDLESS
             << DMA_CH0_TRANS_COUNT_MODE_LSB) | 1u;

    pio_sm_set_enabled(ADAT_RX_PIO, ADAT_RX_SM, true);
    adat_rx_running = true;

    adat_rx_lock_count = adat_rx_loss_count = adat_rx_slip_count = 0;
    adat_rx_header_err = 0;
    hdr_fail_run = 0;
    adat_rx_servo_skip = 0;
    uint64_t now = time_us_64();
    meas_last_poll_us = now;
    uint32_t widx = adat_rx_write_index();
    adat_rx_meas_arm(now, widx);
    adat_rx_resync_scan(widx);

    if (adat_clock_mode == ADAT_CLOCK_MODE_SLAVE) {
        adat_rx_detected_rate = 0;
        adat_rx_probe_arm(now, widx, 48000u);
        adat_rx_set_state(ADAT_INPUT_ACQUIRING);
    } else if (!adat_rx_rate_ok) {
        adat_rx_detected_rate = 0;
        adat_rx_set_state(ADAT_INPUT_ACQUIRING);   // parked until rate <= 48k
    } else {
        adat_rx_detected_rate = audio_state.freq;
        adat_rx_set_state(ADAT_INPUT_SYNCING);
    }

    printf("ADAT RX: started on GPIO %u (%s clock mode)\n", pin,
           (adat_clock_mode == ADAT_CLOCK_MODE_SLAVE) ? "slave" : "master");
}

void adat_input_stop(void) {
    if (!adat_rx_running) return;

    pio_sm_set_enabled(ADAT_RX_PIO, ADAT_RX_SM, false);
    dma_channel_abort(ADAT_RX_DMA_CH);
    dma_channel_hw_addr(ADAT_RX_DMA_CH)->transfer_count = 0;   // leave ENDLESS mode
    dma_channel_unclaim(ADAT_RX_DMA_CH);
    pio_sm_clear_fifos(ADAT_RX_PIO, ADAT_RX_SM);
    pio_remove_program(ADAT_RX_PIO, &adat_rx_program, adat_rx_prog_offset);
    pio_sm_unclaim(ADAT_RX_PIO, ADAT_RX_SM);
    // IE only affects reads, so this is safe when the pin is shared with the
    // ADAT TX output (loopback).
    gpio_set_input_enabled(adat_rx_active_pin, false);
    adat_rx_active_pin = 0xFF;

    adat_rx_running = false;
    adat_rx_detected_rate = 0;
    meas_measured_hz = 0;
    input_servo_reset();   // park the VCXO; no longer tracking
    adat_rx_set_state(ADAT_INPUT_INACTIVE);
    printf("ADAT RX: stopped\n");
}

DSP_TIME_CRITICAL
uint32_t adat_input_poll(void) {
    if (!adat_rx_running) return 0;

    uint64_t now = time_us_64();
    uint32_t widx = adat_rx_write_index();

    uint64_t gap = now - meas_last_poll_us;
    meas_last_poll_us = now;
    bool stall = (gap > ADAT_RX_POLL_GAP_US);

    if (adat_clock_mode == ADAT_CLOCK_MODE_SLAVE) {
        AdatInputState st = adat_rx_state;
        if (st == ADAT_INPUT_ACQUIRING || st == ADAT_INPUT_RELOCKING)
            adat_rx_probe_poll(now, widx);
        else if (st == ADAT_INPUT_SYNCING || st == ADAT_INPUT_LOCKED)
            adat_rx_rate_machine(now, widx, stall);
    }
    // Master-only park (rate_ok can only be false in master mode; the mode
    // qualifier keeps that invariant locally visible). Slave mode proceeds
    // to LOCKED even if the pipeline is still above 48k: the main loop's
    // check_rate_change() then retunes the pipeline under the switch-in
    // mute before any output is enabled.
    if (adat_clock_mode == ADAT_CLOCK_MODE_MASTER && !adat_rx_rate_ok)
        return 0;

    // Master mode re-acquires by rescanning; there is no rate to re-detect.
    if (adat_rx_state == ADAT_INPUT_RELOCKING &&
        adat_clock_mode == ADAT_CLOCK_MODE_MASTER) {
        adat_rx_resync_scan(widx);
        adat_rx_set_state(ADAT_INPUT_SYNCING);
    }

    AdatInputState st = adat_rx_state;
    if (st != ADAT_INPUT_SYNCING && st != ADAT_INPUT_LOCKED) return 0;

    if (st == ADAT_INPUT_SYNCING) {
        if (!sync_found) {
            adat_rx_scan(widx);
            if (!sync_found) return 0;
        }
        // Verify a run of headers at frame stride before trusting audio.
        while (verify_left) {
            if (((widx - rd_word) & ADAT_RX_RING_MASK) < ADAT_RX_FRAME_WORDS + 1u)
                return 0;   // wait for more bits
            if (!adat_rx_header_ok(rd_word, frame_bit)) {
                adat_rx_header_err++;
                adat_rx_resync_scan(rd_word);   // false sync; resume searching
                return 0;
            }
            rd_word = (rd_word + ADAT_RX_FRAME_WORDS) & ADAT_RX_RING_MASK;
            verify_left--;
        }
        hdr_fail_run = 0;
        if (adat_rx_lock_count < 255) adat_rx_lock_count++;
        input_servo_reset();   // fresh integrator for the new lock
        adat_rx_set_state(ADAT_INPUT_LOCKED);
        return 0;
    }

    // LOCKED: lap guard first. Skipping whole frames preserves frame phase
    // (the ring holds an exact number of frames); the header check catches
    // anything it does not.
    uint32_t avail = (widx - rd_word) & ADAT_RX_RING_MASK;
    if (avail > ADAT_RX_RING_WORDS - 64u) {
        uint32_t skip = ((avail - ADAT_RX_RING_WORDS / 2u) / ADAT_RX_FRAME_WORDS)
                        * ADAT_RX_FRAME_WORDS;
        rd_word = (rd_word + skip) & ADAT_RX_RING_MASK;
        avail -= skip;
    }

    // Whole frames fully in the ring; -1 covers the k-shift spill word.
    uint32_t frames = avail ? (avail - 1u) / ADAT_RX_FRAME_WORDS : 0;
    if (frames < ADAT_INPUT_MIN_BLOCK) return 0;   // batch (see MIN_BLOCK)
    if (frames > 192u) frames = 192u;              // pipeline block capacity

    float *dst[8] = { buf_l, buf_r,
                      buf_in_ext[0], buf_in_ext[1], buf_in_ext[2],
                      buf_in_ext[3], buf_in_ext[4], buf_in_ext[5] };
    float pre[8];
    for (int ch = 0; ch < 8; ch++) pre[ch] = global_preamp_linear[ch];
    const float inv_2147483648 = 1.0f / 2147483648.0f;

    uint32_t done = 0;
    bool fail = false;
    while (done < frames) {
        int32_t s[8];
        if (!adat_rx_decode_frame(rd_word, frame_bit, s)) { fail = true; break; }
        for (int ch = 0; ch < 8; ch++)
            dst[ch][done] = (float)s[ch] * inv_2147483648 * pre[ch];
        rd_word = (rd_word + ADAT_RX_FRAME_WORDS) & ADAT_RX_RING_MASK;
        done++;
    }

    if (fail) {
        adat_rx_header_err++;
        if (++hdr_fail_run >= ADAT_HDR_FAIL_LIMIT) {
            if (adat_rx_slip_count < 255) adat_rx_slip_count++;
            adat_rx_drop_lock();
        } else {
            // Isolated bad frame: skip it, keep the lock; the next frame's
            // header decides whether this was a one-off bit error.
            rd_word = (rd_word + ADAT_RX_FRAME_WORDS) & ADAT_RX_RING_MASK;
        }
    } else {
        hdr_fail_run = 0;
    }

    if (done) process_input_block(done);
    return done;
}

DSP_TIME_CRITICAL
void adat_input_update_clock_servo(void) {
    if (adat_clock_mode != ADAT_CLOCK_MODE_SLAVE ||
        adat_rx_state != ADAT_INPUT_LOCKED)
        return;
    // Hold off while the pipeline has not yet followed the detected rate
    // (the muted window between slave lock at a >48k device rate and the
    // deferred rate change): slewing every output clock to the wire rate
    // early would be muted and slot-aligned, but pointless churn.
    if (adat_rx_detected_rate != audio_state.freq) return;
    if (++adat_rx_servo_skip < ADAT_RX_SERVO_INTERVAL) return;
    adat_rx_servo_skip = 0;

    float actual = long_valid ? meas_hz_long : meas_hz_smooth;
    input_servo_apply(actual, 0);
}

bool adat_input_check_rate_change(void) {
    if (adat_clock_mode != ADAT_CLOCK_MODE_SLAVE) return false;
    if (adat_rx_state != ADAT_INPUT_LOCKED) return false;
    uint32_t rate = adat_rx_detected_rate;
    if (rate == 0 || rate == audio_state.freq) return false;
    pending_rate = rate;
    __dmb();
    rate_change_pending = true;
    return true;
}

void adat_input_on_rate_change(uint32_t freq) {
    // Master-only park flag; see adat_input_start. Slave mode is never
    // parked (a clock-mode flip recomputes this through stop/start).
    adat_rx_rate_ok = (adat_clock_mode == ADAT_CLOCK_MODE_SLAVE) ||
                      (freq <= 48000u);
    if (!adat_rx_running) return;
    if (adat_clock_mode == ADAT_CLOCK_MODE_MASTER) {
        if (!adat_rx_rate_ok) {
            // Park: outputs stay muted through the never-completing prefill,
            // mirroring adat_output_on_rate_change's suspension above 48k.
            adat_rx_detected_rate = 0;
            adat_rx_set_state(ADAT_INPUT_ACQUIRING);
            return;
        }
        adat_rx_set_cell(freq);
        adat_rx_detected_rate = freq;
        adat_rx_resync_scan(adat_rx_write_index());
        adat_rx_set_state(ADAT_INPUT_SYNCING);
    }
    // Slave mode: the device rate follows the detected wire rate, so the RX
    // divider is already nominal for it; nothing to retune.
}

AdatInputState adat_input_get_state(void) {
    return adat_rx_state;
}

uint32_t adat_input_get_detected_rate(void) {
    return adat_rx_detected_rate;
}

void adat_input_get_status(AdatInputStatusPacket *out) {
    memset(out, 0, sizeof(*out));
    out->state = (uint8_t)adat_rx_state;
    out->clock_mode = adat_clock_mode;
    out->enabled = adat_input_enabled;
    out->pin = adat_input_pin;
    out->rate_ok = adat_rx_rate_ok ? 1 : 0;
    out->lock_count = adat_rx_lock_count;
    out->loss_count = adat_rx_loss_count;
    out->slip_count = adat_rx_slip_count;
    out->header_err = adat_rx_header_err;
    out->detected_rate = adat_rx_detected_rate;
    out->measured_hz = (adat_clock_mode == ADAT_CLOCK_MODE_SLAVE)
                           ? meas_measured_hz : 0;
}

#endif // PICO_RP2350
