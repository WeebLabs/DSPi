/*
 * net_flac.c - FLAC decode stage of the Sendspin NET input.  See net_flac.h
 * for the data flow, the memory-loan invariants and the accepted formats.
 *
 * Work bound (CPU): fx_flac_process() is driven in small slices so that no
 * single call can occupy the Core 0 main loop for long.  One slice consumes at
 * most NFL_IN_SLICE compressed bytes and produces at most NFL_OUT_SAMPLES
 * decoded samples (256 stereo frames); nfl_pump() runs at most NFL_MAX_SLICES
 * slices and stops early after NFL_BUDGET_US.  foxen decodes sample by sample
 * (rice decode + LPC per sample inside the state machine) so the input slice
 * bounds the decode work of a call; the only block-sized steps are the
 * per-frame stereo decorrelation/shift (a few cycles per sample) and the CRC.
 */
#include "net_flac.h"
#include "net_input.h"
#include "adat_output.h"
#include "foxen-flac.h"

#include <string.h>

#include "pico/time.h"

#ifdef SENDSPIN_DEBUG
#include <stdio.h>
#define NFL_LOG(...) printf("[flac] " __VA_ARGS__)
#else
#define NFL_LOG(...) do {} while (0)
#endif

#define NFL_OUT_SAMPLES   512u    /* int32 samples per decode slice (256 stereo frames) */
#define NFL_IN_SLICE      256u    /* compressed bytes fed per fx_flac_process() call */
#define NFL_MAX_SLICES    8u      /* slices per nfl_pump() call */
#define NFL_BUDGET_US     300u    /* ... or this much wall time, whichever first */
#define NFL_SI_LEN        42u     /* 'fLaC' + 4-byte block header + 34-byte STREAMINFO */
#define NFL_MIN_CHUNK     4096u   /* the arena must leave at least this for the chunk buffer */

static struct {
    bool       held;              /* arena on loan from adat_output.c */
    bool       active;            /* decoder initialised for the current stream */
    bool       evicted;           /* arena reclaimed by the ADAT/TDM output */
    uint8_t   *base;
    fx_flac_t *dec;
    int32_t   *out;               /* NFL_OUT_SAMPLES int32; also the byte staging area */
    uint8_t   *chunk;
    uint32_t   chunk_cap;
    uint32_t   rate;
    uint8_t    bits;
    uint16_t   max_block;         /* STREAMINFO max_blocksize (0 = unknown) */
    uint8_t    si[NFL_SI_LEN];    /* STREAMINFO header copy, last-block flag set; replayed on reset */
    bool       pending;
    uint32_t   chunk_len, chunk_off;
    fx_flac_state_t last_state;
    uint32_t   resyncs;
    /* Slimproto native-FLAC byte-stream mode (nfl_begin_stream) */
    bool       stream;            /* byte-stream mode: no per-chunk reset, header parsed by foxen */
    bool       info_ok;           /* STREAMINFO parsed and acceptable (rate/bits valid) */
    bool       acked;             /* caller applied the format: decoded audio may be emitted */
    bool       failed;            /* stream header unsupported or decoder error */
} f;

/* ------------------------------------------------------------------ */
/* arena                                                               */
/* ------------------------------------------------------------------ */
static void nfl_evict_cb(void)
{
    /* Called by adat_output_resync() on Core 0 main-loop context, never from
     * inside nfl_pump(): stop using the arena at once and give it back. */
    f.active = false;
    f.pending = false;
    f.held = false;
    f.evicted = true;
    adat_output_ring_return();
}

bool nfl_supported_now(void)
{
    return f.held || adat_output_ring_loanable();
}

static bool arena_take(void)
{
    if (f.held) return true;
    uint32_t bytes = 0;
    void *p = adat_output_ring_loan(&bytes, nfl_evict_cb);
    if (!p) return false;
    uint8_t *base = (uint8_t *)(((uintptr_t)p + 15u) & ~(uintptr_t)15u);
    uint32_t lost = (uint32_t)(base - (uint8_t *)p);
    uint32_t dec_sz = fx_flac_size(NFL_MAX_BLOCK, 2);
    uint32_t dec_al = (dec_sz + 15u) & ~15u;
    uint32_t out_sz = NFL_OUT_SAMPLES * 4u;
    if (!dec_sz || bytes < lost + dec_al + out_sz + NFL_MIN_CHUNK) {
        adat_output_ring_return();
        return false;
    }
    f.base = base;
    f.dec = (fx_flac_t *)base;
    f.out = (int32_t *)(base + dec_al);
    f.chunk = base + dec_al + out_sz;
    f.chunk_cap = bytes - lost - dec_al - out_sz;
    f.held = true;
    return true;
}

/* ------------------------------------------------------------------ */
/* helpers                                                             */
/* ------------------------------------------------------------------ */
static int b64_val(char ch)
{
    if (ch >= 'A' && ch <= 'Z') return ch - 'A';
    if (ch >= 'a' && ch <= 'z') return ch - 'a' + 26;
    if (ch >= '0' && ch <= '9') return ch - '0' + 52;
    if (ch == '+' || ch == '-') return 62;
    if (ch == '/' || ch == '_') return 63;
    return -1;
}

/* Standard Base64 (RFC 4648 s.4, padding included; the URL alphabet and a
 * missing padding are tolerated).  Returns bytes or -1. */
static int b64_decode(uint8_t *out, size_t cap, const char *in, size_t len)
{
    while (len && in[len - 1] == '=') len--;
    size_t o = 0;
    uint32_t acc = 0;
    int bits = 0;
    for (size_t i = 0; i < len; i++) {
        int v = b64_val(in[i]);
        if (v < 0) return -1;
        acc = (acc << 6) | (uint32_t)v;
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            if (o >= cap) return -1;
            out[o++] = (uint8_t)(acc >> bits);
            acc &= (1u << bits) - 1u;
        }
    }
    if (len % 4 == 1) return -1;
    return (int)o;
}

/* Replay the STREAMINFO block into a freshly reset decoder. */
static bool dec_prime(void)
{
    fx_flac_reset(f.dec);
    uint32_t in_len = NFL_SI_LEN;
    fx_flac_state_t st = fx_flac_process(f.dec, f.si, &in_len, NULL, NULL);
    f.last_state = st;
    return st == FLAC_END_OF_METADATA && in_len == NFL_SI_LEN;
}

/* ------------------------------------------------------------------ */
/* lifecycle                                                           */
/* ------------------------------------------------------------------ */
void nfl_end(void)
{
    f.active = false;
    f.pending = false;
    f.stream = f.info_ok = f.acked = f.failed = false;
    if (f.held) {
        f.held = false;
        adat_output_ring_return();
    }
}

int nfl_begin(const char *b64, size_t b64_len, uint32_t rate, uint8_t bits)
{
    f.active = false;
    f.pending = false;
    f.stream = f.info_ok = f.acked = f.failed = false;
    if (!b64 || !b64_len) return NFL_ERR_HEADER;
    if (!arena_take()) return NFL_ERR_NO_ARENA;

    int n = b64_decode(f.chunk, f.chunk_cap, b64, b64_len);
    if (n < (int)NFL_SI_LEN) { nfl_end(); return NFL_ERR_HEADER; }
    const uint8_t *h = f.chunk;
    if (memcmp(h, "fLaC", 4) != 0 || (h[4] & 0x7F) != 0 ||
        h[5] != 0 || h[6] != 0 || h[7] != 34) {
        nfl_end();
        return NFL_ERR_HEADER;
    }
    /* STREAMINFO: min_bs(16) max_bs(16) min_fs(24) max_fs(24) rate(20) ch-1(3) bps-1(5) ... */
    uint16_t max_bs = (uint16_t)((h[10] << 8) | h[11]);
    uint32_t sr  = ((uint32_t)h[18] << 12) | ((uint32_t)h[19] << 4) | (h[20] >> 4);
    uint8_t  ch  = (uint8_t)(((h[20] >> 1) & 7u) + 1u);
    uint8_t  bps = (uint8_t)((((h[20] & 1u) << 4) | (h[21] >> 4)) + 1u);
    if (ch != 2 || sr != rate || bps != bits || (bits != 16 && bits != 24) ||
        max_bs > NFL_MAX_BLOCK) {
        NFL_LOG("unsupported STREAMINFO: ch %u rate %u bps %u maxbs %u (stream/start %u/%u)\n",
                ch, (unsigned)sr, bps, max_bs, (unsigned)rate, bits);
        nfl_end();
        return NFL_ERR_FORMAT;
    }
    memcpy(f.si, h, NFL_SI_LEN);
    f.si[4] |= 0x80;            /* servers may leave "last metadata block" clear; we feed only this block */
    f.rate = rate;
    f.bits = bits;
    f.max_block = max_bs;

    if (!fx_flac_init(f.base, NFL_MAX_BLOCK, 2) || !dec_prime()) {
        nfl_end();
        return NFL_ERR_DECODER;
    }
    f.active = true;
    f.evicted = false;
    f.chunk_len = f.chunk_off = 0;
    f.resyncs = 0;
    return NFL_OK;
}

/* Slimproto: the HTTP body is a plain FLAC file stream ('fLaC', metadata blocks,
 * frames) whose chunking is arbitrary.  The decoder is initialised without the
 * STREAMINFO replay; foxen parses the metadata itself, and nfl_pump() stops right
 * after it so the caller can read the format and apply it (nfl_stream_info /
 * nfl_stream_ack) before any audio is emitted.  Chunks are plain input FIFO
 * segments: the decoder state is NOT reset between them. */
int nfl_begin_stream(void)
{
    f.active = false;
    f.pending = false;
    f.stream = f.info_ok = f.acked = f.failed = false;
    if (!arena_take()) return NFL_ERR_NO_ARENA;
    if (!fx_flac_init(f.base, NFL_MAX_BLOCK, 2)) { nfl_end(); return NFL_ERR_DECODER; }
    f.last_state = FLAC_INIT;
    f.rate = 0;
    f.bits = 16;
    f.max_block = 0;
    f.stream = true;
    f.active = true;
    f.evicted = false;
    f.chunk_len = f.chunk_off = 0;
    f.resyncs = 0;
    return NFL_OK;
}

bool nfl_stream_info(uint32_t *rate, uint8_t *bits)
{
    if (!f.stream || !f.info_ok) return false;
    if (rate) *rate = f.rate;
    if (bits) *bits = f.bits;
    return true;
}
void nfl_stream_ack(void) { f.acked = true; }
bool nfl_stream_failed(void) { return f.failed; }

void nfl_reset(void)
{
    f.pending = false;
    f.chunk_len = f.chunk_off = 0;
    if (f.stream) return;                  /* byte-stream mode: callers use nfl_end() */
    if (f.active && !dec_prime()) f.active = false;
}

bool nfl_active(void) { return f.active; }
uint8_t *nfl_chunk_buf(void) { return f.chunk; }
uint32_t nfl_chunk_cap(void) { return f.active ? f.chunk_cap : 0; }
bool nfl_take_evicted(void) { bool e = f.evicted; f.evicted = false; return e; }
uint32_t nfl_stat_frames_dropped(void) { return f.resyncs; }

uint32_t nfl_chunk_dur_us(void)
{
    uint32_t blk = f.max_block ? f.max_block : 4096u;
    return f.rate ? (uint32_t)((uint64_t)blk * 1000000ull / f.rate) : 0;
}

void nfl_chunk_submit(uint32_t len)
{
    if (!f.active || f.pending || len == 0 || len > f.chunk_cap) return;
    f.chunk_len = len;
    f.chunk_off = 0;
    f.pending = true;
}

bool nfl_pending(void) { return f.pending; }
uint32_t nfl_chunk_remaining(void) { return f.pending ? f.chunk_len - f.chunk_off : 0; }

/* ------------------------------------------------------------------ */
/* decode pump                                                         */
/* ------------------------------------------------------------------ */
/* Convert n decoded int32 samples (full-scale, as foxen delivers them) to
 * little-endian packed PCM in place and publish them. */
static void emit(uint32_t n)
{
    uint8_t *p = (uint8_t *)f.out;
    uint32_t bytes;
    if (f.bits == 16) {
        for (uint32_t i = 0; i < n; i++) {
            int32_t v = f.out[i] >> 16;
            p[2 * i]     = (uint8_t)v;
            p[2 * i + 1] = (uint8_t)(v >> 8);
        }
        bytes = n * 2u;
    } else {
        for (uint32_t i = 0; i < n; i++) {
            int32_t v = f.out[i] >> 8;            /* read before the (overlapping) writes */
            p[3 * i]     = (uint8_t)v;
            p[3 * i + 1] = (uint8_t)(v >> 8);
            p[3 * i + 2] = (uint8_t)(v >> 16);
        }
        bytes = n * 3u;
    }
    /* Room was verified before fx_flac_process() was called. */
    if (net_input_stage_pcm(0, p, bytes)) net_input_commit_pcm(bytes);
}

static void chunk_done(void)
{
    f.pending = false;
    if (f.stream) return;                  /* arbitrary segmentation: keep decoder state */
    /* Chunks hold whole frames ("a unit never spans chunks"): whatever state
     * the decoder is in now (clean, or stuck inside a truncated/garbled frame
     * or a half-read header) must not leak into the next chunk. */
    if (!dec_prime()) f.active = false;
}

bool nfl_pump(void)
{
    if (!f.pending) return false;
    uint64_t t0 = time_us_64();
    const uint32_t slice_bytes = NFL_OUT_SAMPLES * (f.bits / 8u);

    for (uint32_t s = 0; s < NFL_MAX_SLICES; s++) {
        /* Never call the decoder unless a whole output slice fits in the ring:
         * nothing it produces can be lost, and a full ring just stalls us. */
        if (net_input_ring_free_bytes() < slice_bytes) break;
        if (f.stream && f.info_ok && !f.acked) break;       /* waiting for the caller to apply the format */

        uint32_t left = f.chunk_len - f.chunk_off;
        uint32_t in_len = left < NFL_IN_SLICE ? left : NFL_IN_SLICE;
        uint32_t out_len = NFL_OUT_SAMPLES;
        fx_flac_state_t st = fx_flac_process(f.dec, f.chunk + f.chunk_off, &in_len, f.out, &out_len);
        f.chunk_off += in_len;

        /* A drop from IN_FRAME straight back to SEARCH_FRAME means foxen
         * rejected the frame (CRC / header / subframe error) and is
         * resynchronising; it emitted nothing for it. */
        if (f.last_state == FLAC_IN_FRAME && st == FLAC_SEARCH_FRAME) {
            f.resyncs++;
            NFL_LOG("frame dropped (decoder resync)\n");
        }
        f.last_state = st;

        if (f.stream && !f.info_ok && st == FLAC_END_OF_METADATA) {
            int64_t sr = fx_flac_get_streaminfo(f.dec, FLAC_KEY_SAMPLE_RATE);
            int64_t ch = fx_flac_get_streaminfo(f.dec, FLAC_KEY_N_CHANNELS);
            int64_t bp = fx_flac_get_streaminfo(f.dec, FLAC_KEY_SAMPLE_SIZE);
            int64_t mb = fx_flac_get_streaminfo(f.dec, FLAC_KEY_MAX_BLOCK_SIZE);
            if ((sr != 44100 && sr != 48000) || ch != 2 || (bp != 16 && bp != 24) ||
                mb > (int64_t)NFL_MAX_BLOCK) {
                NFL_LOG("unsupported stream STREAMINFO\n");
                f.failed = true;
                f.pending = false;
                return false;
            }
            f.rate = (uint32_t)sr;
            f.bits = (uint8_t)bp;
            f.max_block = (uint16_t)mb;
            f.info_ok = true;
            break;                                          /* caller applies the format first */
        }
        if (f.stream && st == FLAC_ERR) {
            f.failed = true;
            f.pending = false;
            return false;
        }
        if (out_len) emit(out_len & ~1u);
        if (st == FLAC_ERR) {
            NFL_LOG("decoder error, resetting\n");
            f.resyncs++;
            chunk_done();
            return false;
        }
        if (f.chunk_off >= f.chunk_len && out_len == 0) {
            chunk_done();
            return false;
        }
        if (in_len == 0 && out_len == 0) {      /* no progress at all: should not happen */
            NFL_LOG("decoder stalled, dropping rest of chunk\n");
            f.resyncs++;
            chunk_done();
            return false;
        }
        if (time_us_64() - t0 > NFL_BUDGET_US) break;
    }
    return f.pending;
}
