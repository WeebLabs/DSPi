/*
 * slimproto_client.c - Squeezelite-compatible Slimproto player (see the header).
 *
 * PROTOCOL FACTS USED (sources: squeezelite slimproto.c/pcm.c, aioslimproto
 * client.py/util.py/const.py, Music Assistant squeezelite provider; items I could
 * not confirm byte for byte are marked UNVERIFIED:):
 *
 *  framing   server->player: u16 BE length (of what follows) + 4 byte opcode +
 *            data.  player->server: 4 byte opcode + u32 BE length + data.
 *  HELO      opcode, len = 36 + caps, devid(1)=12, rev(1)=0, mac(6), uuid(16)=0,
 *            wlan_channellist(2) (0x4000 = reconnect), bytes_received(8), lang(2)
 *            then the ASCII capability list.  aioslimproto reads the list as a
 *            query string (',' -> '&'), takes `ModelName`/`Model`/`MaxSampleRate`
 *            and detects codecs by SUBSTRING (alc aac ogg ogf flc aif pcm mp3):
 *            the caps below contain exactly "flc" and "pcm" and none of the others.
 *  STAT      53 bytes: event(4) crlf(1) mas_init(1) mas_mode(1) stream_buf_full(4)
 *            stream_buf_size(4) bytes_rx(8) signal(2)=0xffff jiffies(4)
 *            out_buf_size(4) out_buf_full(4) elapsed_s(4) voltage(2)
 *            elapsed_ms(4) server_ts(4) error(2).  UNVERIFIED: the order of the
 *            two stream-buffer words (taken from squeezelite's sendSTAT assignment
 *            order); MA ignores both.
 *  strm      payload after the opcode (24 bytes + HTTP request): cmd, autostart,
 *            format, pcm size/rate/channels/endianness, threshold(KB), spdif,
 *            transition period/type, flags (0x20 = https), output_threshold
 *            (1/10 s), slaves, replay_gain(4) [also the interval/jiffies/server
 *            timestamp of p/u/a/t], server_port(2), server_ip(4).  Commands:
 *            s start (the HTTP request follows), p pause, u unpause, q stop,
 *            f flush, t status request (answered with STMt echoing replay_gain),
 *            a skip-ahead (ignored).
 *            pcm_size '0'..'3' = 1..4 bytes, pcm_rate index into
 *            {11025,22050,32000,44100,48000,8000,12000,16000,24000,96000,...},
 *            channels digit, endianness '0' = big, '1' = little, '?' = unknown
 *            (then the stream is a WAV file: RIFF header parsed here).
 *  autostart '0' wait for `u` (player reports STMl when buffered), '1' start by
 *            itself, '2'/'3' = same plus "wait for cont first" (direct HTTP
 *            streams: the player sends RESP with the response header, the server
 *            answers `cont`).  MA uses '2' (single play: wait for STMl then `u`)
 *            and '3' (enqueue/gapless).
 *  STAT events used: STMt timer/status, STMf flushed, STMc connected, STMs track
 *            started, STMd decode (stream) complete, STMu underrun/end of track
 *            (output ran dry after decode finished), STMo output underrun
 *            (output ran dry while the stream is still open), STMl buffer
 *            loaded, STMp paused, STMr resumed, STMn decode error.  DSCO(code):
 *            0 ok, 2 remote close, 3 unreachable, 4 timeout.
 *  audg      old_gainL(4) old_gainR(4) adjust(1) preamp(1) gainL(4) gainR(4);
 *            gains are 16.16 linear.  UNVERIFIED: older servers send only the
 *            first 10 bytes (the old gains are used then).
 *  aude      spdif_enable(1) dac_enable(1).  aioslimproto sends (1,1)/(0,1) for
 *            power and (1,0)/(0,0) for unmute/mute; here spdif_enable == 0 means
 *            "mute".  UNVERIFIED for the power-off case (also just mutes).
 *  HTTP      the request is sent verbatim from the strm command (MA: HTTP/1.0
 *            GET with Range: bytes=0-).  Status 200 and 206 accepted.  Chunked
 *            transfer coding is decoded anyway (UNVERIFIED whether MA's aiohttp
 *            ever uses it for an HTTP/1.0 request; cheap insurance), Content-Length
 *            is honoured, otherwise the body ends when the server closes.
 *            https (flags & 0x20) is refused (no TLS).
 *
 * MODEL: the ring (net_input) is both "stream buffer" and "output buffer".
 *   out_state STOPPED/PAUSED hold the consumer (net_input_set_hold), RUNNING
 *   releases it.  Track starts are marked in the ring by their free-running write
 *   position; STMs fires when the consumer passes it, and elapsed time is the
 *   consumed byte count since then.  Gapless: the next `strm s` arrives after
 *   STMd, its data is appended behind the current track (a format change waits
 *   until the ring has drained, no data is lost).
 *
 * Threshold caps: the server's stream threshold (MA: 200 KB) cannot fit the 32 KB
 * ring; STMl / autostart use min(threshold, 24 KB).
 */
#include "slimproto_client.h"
#include "sendspin_client.h"
#include "net_input.h"
#include "net_flac.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <math.h>

#include "lwip/tcp.h"
#include "lwip/dns.h"
#include "lwip/pbuf.h"
#include "lwip/ip_addr.h"
#include "pico/time.h"

#ifdef SLIMPROTO_DEBUG
#define SL_LOG(...) printf("[slimproto] " __VA_ARGS__)
#else
#define SL_LOG(...) do {} while (0)
#endif

/* ------------------------------------------------------------------ */
/* Tunables                                                            */
/* ------------------------------------------------------------------ */
#define SL_RX_BUF            1024     /* one inbound control frame (opcode + payload); larger ones are skipped */
#define SL_TX_BUF            320      /* one outbound message (HELO is the largest) */
#define SL_HDR_BUF           1024     /* HTTP request (from strm s), then the HTTP response header (-> RESP) */
#define SL_MAX_FRAME         4096     /* squeezelite MAXBUF: longer frames drop the connection */
#define SL_QLEN              12
#define SL_BACKOFF_MIN_MS    1000u
#define SL_BACKOFF_MAX_MS    10000u
#define SL_PHASE_TIMEOUT_US  15000000ull
#define SL_SERVER_IDLE_US    45000000ull   /* MA sends strm t every 5 s */
#define SL_STREAM_CONN_US    8000000ull
#define SL_STREAM_IDLE_US    10000000ull
#define SL_CONT_WAIT_US      10000000ull
#define SL_STMT_RUN_US       1000000ull
#define SL_STMT_IDLE_US      5000000ull
#define SL_STALL_US          15000000ull   /* running, ring full and not drained */
#define SL_THRESH_CAP        24576u        /* max bytes of ring fill we wait for */
#define SL_PCM_TMP           192u          /* byte-swap staging (stack) */

#define SL_CAPS "Model=squeezelite,ModelName=" SLIMPROTO_PLAYER_NAME ",AccuratePlayPoints=1," \
                "HasDigitalOut=1,MaxSampleRate=48000,flc,pcm"

enum { PH_NONE = 0, PH_DNS, PH_TCP, PH_RUN };
enum { SP_NONE = 0, SP_CONNECT, SP_HDR, SP_WAIT_CONT, SP_BODY, SP_DONE };
enum { OUT_STOPPED = 0, OUT_PAUSED, OUT_RUNNING };
enum { Q_STAT = 1, Q_RESP, Q_DSCO, Q_SETD };
enum { CS_SIZE = 0, CS_EXT, CS_SIZE_LF, CS_DATA, CS_DATA_CR, CS_DATA_LF, CS_END };
enum { PS_DETECT = 0, PS_WAV_RIFF, PS_WAV_CH, PS_WAV_FMT, PS_WAV_SKIP, PS_APPLY, PS_PCM };
enum { DSCO_OK = 0, DSCO_REMOTE = 2, DSCO_UNREACH = 3, DSCO_TIMEOUT = 4 };

typedef struct { uint32_t pos, rate; uint8_t bits; } track_mark_t;

typedef struct {
    bool     enabled;
    uint8_t  state, phase;
    bool     fatal, abort_flag, dns_pending, reconnect;
    struct tcp_pcb *pcb;
    ip_addr_t srv_ip;
    uint32_t backoff_ms;
    uint64_t retry_at_us, phase_start_us, last_rx_us, last_stmt_us;

    /* control rx framing */
    uint8_t  rx_hdr[2];
    uint8_t  rx_hdr_have;
    uint16_t rx_len, rx_have;
    bool     rx_skip;

    /* control tx */
    uint16_t tx_off, tx_end;
    bool     need_helo;
    uint8_t  q_n, q_head;
    struct { uint8_t kind; char code[4]; uint32_t aux; } q[SL_QLEN];
    uint16_t resp_len;           /* bytes of header in rsp+8 queued by Q_RESP */
    char     name[33];

    /* player */
    uint8_t  out_state;
    uint64_t pause_end_us;       /* timed pause (strm p with interval) */
    bool     aude_seen, aude_on;
    uint8_t  gain_pct;
    track_mark_t marks[3];
    uint8_t  mark_n;
    bool     cur_valid;
    uint32_t cur_start, cur_rate;
    uint8_t  cur_bits;
    bool     fmt_set;            /* ring format below was applied by us */
    uint32_t ring_rate;
    uint8_t  ring_bits;
    bool     sent_stml, sent_stmu, sent_stmo, sent_stmd;
    uint64_t bytes_rx;

    /* HTTP stream */
    struct tcp_pcb *spcb;
    bool     s_abort;
    uint8_t  sp;                 /* SP_* */
    uint64_t sp_start_us, s_rx_us, s_block_us;
    bool     s_blocked, s_fin, body_complete;
    uint8_t  autostart;          /* 0..3, decremented by cont */
    uint8_t  fmt;                /* 'f' or 'p' */
    uint32_t threshold, out_thr;
    ip_addr_t s_ip;
    uint16_t s_port, hdr_have, req_len;
    bool     req_sent;
    int64_t  clen;               /* Content-Length or -1 */
    bool     chunked;
    uint8_t  cs;                 /* CS_* */
    uint32_t cs_left, cs_acc;
    uint8_t  cs_digits;
    struct pbuf *hold;
    struct pbuf *hold_seg;
    uint16_t hold_off;

    /* body decode */
    bool     fmt_applied;
    uint8_t  ps;                 /* PS_* */
    uint8_t  det[4], det_n, det_off;
    bool     pcm_known;
    uint32_t pcm_rate;
    uint8_t  pcm_bps;            /* bytes per sample 2 or 3 */
    bool     pcm_be;
    uint8_t  carry[6], carry_n;
    uint8_t  wbuf[48];
    uint8_t  w_have, w_need;
    uint32_t w_skip, w_dataleft;
    bool     w_limited;

    uint8_t  rx[SL_RX_BUF] __attribute__((aligned(4)));
    uint8_t  tx[SL_TX_BUF] __attribute__((aligned(4)));
    uint8_t  rsp[8 + SL_HDR_BUF] __attribute__((aligned(4)));
} sl_t;

static sl_t *g_c;
#define c (*g_c)

/* ------------------------------------------------------------------ */
/* helpers                                                             */
/* ------------------------------------------------------------------ */
static inline uint32_t be32(const uint8_t *p) { return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]; }
static inline uint16_t be16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static inline void put32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v; }
static inline void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }

static void set_phase(uint8_t ph) { c.phase = ph; c.phase_start_us = time_us_64(); }

static void sl_set_state(uint8_t st) { c.state = st; }

/* ------------------------------------------------------------------ */
/* outgoing queue                                                      */
/* ------------------------------------------------------------------ */
static bool q_push(uint8_t kind, const char *code, uint32_t aux)
{
    if (c.q_n >= SL_QLEN) return false;       /* dropped: server resynchronises through strm t / STMt */
    uint8_t i = (uint8_t)((c.q_head + c.q_n) % SL_QLEN);
    c.q[i].kind = kind;
    if (code) memcpy(c.q[i].code, code, 4); else memset(c.q[i].code, 0, 4);
    c.q[i].aux = aux;
    c.q_n++;
    return true;
}
static void stat_evt(const char *code, uint32_t ts) { q_push(Q_STAT, code, ts); }

static void q_clear(void) { c.q_n = 0; c.q_head = 0; c.need_helo = false; }

static bool tx_busy(void) { return c.tx_off < c.tx_end; }

static void tx_flush(void)
{
    if (!c.pcb) return;
    while (c.tx_off < c.tx_end) {
        uint16_t avail = tcp_sndbuf(c.pcb);
        if (!avail) break;
        uint16_t n = (uint16_t)(c.tx_end - c.tx_off);
        if (n > avail) n = avail;
        err_t e = tcp_write(c.pcb, c.tx + c.tx_off, n, TCP_WRITE_FLAG_COPY);
        if (e == ERR_MEM) break;
        if (e != ERR_OK) { c.fatal = true; return; }
        c.tx_off = (uint16_t)(c.tx_off + n);
    }
    tcp_output(c.pcb);
}

static uint32_t elapsed_ms(void)
{
    if (!c.cur_valid || !c.cur_rate) return 0;
    uint32_t bpf = (uint32_t)(c.cur_bits / 8u) * 2u;
    uint32_t bytes = net_input_ring_rd_pos() - c.cur_start;
    if (bytes > 0x7fffffffu) return 0;
    return (uint32_t)(((uint64_t)(bytes / bpf) * 1000ull) / c.cur_rate);
}

static void build_stat(const char *ev, uint32_t server_ts)
{
    uint8_t *t = c.tx;
    uint32_t ms = elapsed_ms();
    memset(t, 0, 8 + 53);
    memcpy(t, "STAT", 4);
    put32(t + 4, 53);
    memcpy(t + 8, ev, 4);
    /* [12..14] num_crlf, mas_initialized, mas_mode = 0 */
    uint32_t cap = net_input_ring_capacity(), fill = net_input_ring_fill_bytes();
    put32(t + 15, fill);                 /* stream buffer fullness (the ring is both buffers) */
    put32(t + 19, cap);                  /* stream buffer size */
    put32(t + 23, (uint32_t)(c.bytes_rx >> 32));
    put32(t + 27, (uint32_t)c.bytes_rx);
    put16(t + 31, 0xffff);               /* signal strength: wired/unknown */
    put32(t + 33, (uint32_t)(time_us_64() / 1000ull));   /* jiffies */
    put32(t + 37, cap);                  /* output buffer size */
    put32(t + 41, fill);                 /* output buffer fullness */
    put32(t + 45, ms / 1000u);           /* elapsed seconds */
    /* [49..50] voltage = 0 */
    put32(t + 51, ms);                   /* elapsed milliseconds */
    put32(t + 55, server_ts);            /* server timestamp (echo of strm t replay_gain field) */
    /* [59..60] error code = 0 */
    c.tx_off = 0; c.tx_end = 8 + 53;
}

static void build_helo(void)
{
    uint8_t *t = c.tx;
    size_t cl = strlen(SL_CAPS);
    memset(t, 0, 44);
    memcpy(t, "HELO", 4);
    put32(t + 4, (uint32_t)(36 + cl));
    t[8] = 12;                           /* device id: squeezeplay/squeezelite */
    t[9] = 0;                            /* revision */
    net_input_get_mac(t + 10);
    put16(t + 32, c.reconnect ? 0x4000 : 0);
    put32(t + 34, (uint32_t)(c.bytes_rx >> 32));
    put32(t + 38, (uint32_t)c.bytes_rx);
    memcpy(t + 44, SL_CAPS, cl);
    c.tx_off = 0; c.tx_end = (uint16_t)(44 + cl);
}

static void service_tx(void)
{
    if (!c.pcb) return;
    for (int guard = 0; guard < 4; guard++) {
        if (tx_busy()) { tx_flush(); if (tx_busy() || c.fatal) return; }
        if (c.phase < PH_RUN) return;
        if (c.need_helo) { build_helo(); c.need_helo = false; tx_flush(); continue; }
        if (!c.q_n) return;
        uint8_t i = c.q_head;
        uint8_t kind = c.q[i].kind;
        if (kind == Q_RESP) {
            uint16_t total = (uint16_t)(8 + c.resp_len);
            if (tcp_sndbuf(c.pcb) < total) return;          /* wait for window; ordering is kept */
            memcpy(c.rsp, "RESP", 4);
            put32(c.rsp + 4, c.resp_len);
            if (tcp_write(c.pcb, c.rsp, total, TCP_WRITE_FLAG_COPY) != ERR_OK) return;
            tcp_output(c.pcb);
        } else if (kind == 0) {
            /* dropped entry */
        } else if (kind == Q_STAT) {
            build_stat(c.q[i].code, c.q[i].aux);
        } else if (kind == Q_DSCO) {
            memset(c.tx, 0, 9);
            memcpy(c.tx, "DSCO", 4);
            put32(c.tx + 4, 1);
            c.tx[8] = (uint8_t)c.q[i].aux;
            c.tx_off = 0; c.tx_end = 9;
        } else {   /* Q_SETD: id 0 + name */
            size_t nl = strlen(c.name);
            memcpy(c.tx, "SETD", 4);
            put32(c.tx + 4, (uint32_t)(nl + 2));
            c.tx[8] = 0;
            memcpy(c.tx + 9, c.name, nl + 1);
            c.tx_off = 0; c.tx_end = (uint16_t)(9 + nl + 1);
        }
        c.q_head = (uint8_t)((c.q_head + 1) % SL_QLEN);
        c.q_n--;
        if (kind != Q_RESP && kind != 0) tx_flush();
    }
}

/* ------------------------------------------------------------------ */
/* playback model                                                      */
/* ------------------------------------------------------------------ */
static void marks_clear(void) { c.mark_n = 0; c.cur_valid = false; }

static void mark_push(uint32_t pos, uint32_t rate, uint8_t bits)
{
    if (c.mark_n == 3) { c.marks[0] = c.marks[1]; c.marks[1] = c.marks[2]; c.mark_n = 2; }
    c.marks[c.mark_n].pos = pos; c.marks[c.mark_n].rate = rate; c.marks[c.mark_n].bits = bits;
    c.mark_n++;
    c.sent_stmu = c.sent_stmo = false;
}

static void out_set(uint8_t st)
{
    c.out_state = st;
    net_input_set_hold(st != OUT_RUNNING);
}

static void ring_reset(void)
{
    net_input_flush();
    marks_clear();
    c.sent_stmu = c.sent_stmo = false;
}

/* Ring format for a new track.  A different format must wait until the old
 * track's tail has been consumed (set_format flushes). */
static bool apply_format(uint32_t rate, uint8_t bits)
{
    if (!(c.fmt_set && c.ring_rate == rate && c.ring_bits == bits)) {
        if (net_input_ring_fill_bytes() != 0) return false;
        net_input_set_format(rate, 2, bits);
        c.fmt_set = true; c.ring_rate = rate; c.ring_bits = bits;
    }
    mark_push(net_input_ring_wr_pos(), rate, bits);
    c.fmt_applied = true;
    return true;
}

static uint32_t start_bytes(void)       /* ring fill that counts as "buffered" */
{
    uint32_t cap = net_input_ring_capacity();
    uint32_t lim = cap / 4u * 3u;
    if (lim > SL_THRESH_CAP) lim = SL_THRESH_CAP;
    return lim;
}

static uint8_t gain_to_pct(uint32_t g16)
{
    if (g16 >= 65536u) return 100;
    if (g16 == 0) return 0;
    float p = 100.0f * powf((float)g16 / 65536.0f, 2.0f / 3.0f);   /* inverse of gain = (pct/100)^1.5 */
    int v = (int)(p + 0.5f);
    return (uint8_t)(v < 0 ? 0 : (v > 100 ? 100 : v));
}

static void play_service(uint64_t now)
{
    uint32_t fill = net_input_ring_fill_bytes();

    if (c.out_state == OUT_PAUSED && c.pause_end_us && now >= c.pause_end_us) {
        c.pause_end_us = 0;
        out_set(OUT_RUNNING);
    }

    /* track started: the consumer passed the marked write position */
    if (c.mark_n && (int32_t)(net_input_ring_rd_pos() - c.marks[0].pos) > 0) {
        c.cur_start = c.marks[0].pos; c.cur_rate = c.marks[0].rate; c.cur_bits = c.marks[0].bits;
        c.cur_valid = true;
        for (uint8_t i = 1; i < c.mark_n; i++) c.marks[i - 1] = c.marks[i];
        c.mark_n--;
        stat_evt("STMs", 0);
    }

    bool stream_open = c.sp >= SP_CONNECT && c.sp < SP_DONE;
    bool decode_done = c.sp == SP_DONE;

    /* start / buffered notifications */
    if (c.sp >= SP_BODY && c.fmt_applied) {
        bool buffered = fill >= start_bytes() || decode_done || (c.threshold && fill >= c.threshold);
        if (c.autostart == 1 && c.out_state == OUT_STOPPED) {
            uint32_t need = 0;
            if (c.out_thr && c.ring_rate) need = (uint32_t)((uint64_t)c.out_thr * c.ring_rate / 10u) * (c.ring_bits / 8u) * 2u;
            if (need > start_bytes()) need = start_bytes();
            if (fill > 0 && (fill >= need || decode_done)) out_set(OUT_RUNNING);
        } else if (c.autostart == 0 && !c.sent_stml && (buffered || (c.sp == SP_DONE))) {
            c.sent_stml = true;
            stat_evt("STMl", 0);
        }
    }

    /* output ran dry */
    if (c.out_state == OUT_RUNNING && c.cur_valid && fill == 0 && !c.mark_n) {
        if (stream_open) {
            if (!c.sent_stmo && c.sp >= SP_BODY) { c.sent_stmo = true; stat_evt("STMo", 0); }
        } else if (!c.sent_stmu) {
            c.sent_stmu = true;
            c.cur_valid = false;
            stat_evt("STMu", 0);
        }
    }

    /* periodic status */
    uint64_t period = c.out_state == OUT_RUNNING ? SL_STMT_RUN_US : SL_STMT_IDLE_US;
    if (now - c.last_stmt_us >= period) {
        c.last_stmt_us = now;
        stat_evt("STMt", 0);
    }
}

/* ------------------------------------------------------------------ */
/* HTTP stream: teardown                                               */
/* ------------------------------------------------------------------ */
static void q_drop_resp(void)
{
    for (uint8_t i = 0; i < c.q_n; i++) {
        uint8_t k = (uint8_t)((c.q_head + i) % SL_QLEN);
        if (c.q[k].kind == Q_RESP) c.q[k].kind = 0;     /* rsp buffer is about to be reused */
    }
}

static void hold_free(void)
{
    if (c.hold) { pbuf_free(c.hold); c.hold = NULL; c.hold_seg = NULL; c.hold_off = 0; }
}

static void spcb_release(bool graceful)
{
    if (!c.spcb) return;
    struct tcp_pcb *p = c.spcb;
    c.spcb = NULL;
    tcp_arg(p, NULL); tcp_recv(p, NULL); tcp_sent(p, NULL); tcp_err(p, NULL);
    if (graceful) {
        if (tcp_close(p) != ERR_OK) { tcp_abort(p); c.s_abort = true; }
    } else {
        tcp_abort(p);
        c.s_abort = true;
    }
}

/* Forget the current stream completely (new strm s / q / f / connection loss).
 * The ring content is NOT touched here. */
static void stream_reset(void)
{
    spcb_release(false);
    hold_free();
    if (c.fmt == 'f') nfl_end();
    c.sp = SP_NONE;
    c.s_blocked = c.s_fin = c.body_complete = false;
    c.fmt_applied = false;
    c.hdr_have = 0; c.req_sent = false;
    c.carry_n = 0;
    c.det_n = c.det_off = 0;
    c.fmt = 0;
    if (c.state == SLIMPROTO_STATE_STREAMING) sl_set_state(SLIMPROTO_STATE_READY);
}

/* ------------------------------------------------------------------ */
/* HTTP stream: body decode                                            */
/* ------------------------------------------------------------------ */
static void stream_error(void)
{
    SL_LOG("stream error\n");
    stream_reset();
    c.sp = SP_DONE;
    stat_evt("STMn", 0);
}

/* S16/S24 interleaved stereo -> ring (LE).  Returns bytes consumed. */
static size_t pcm_push(const uint8_t *d, size_t n)
{
    const uint32_t bps = c.pcm_bps, bpf = 2u * bps;
    size_t used = 0;

    if (c.carry_n) {                                   /* finish a frame split across segments */
        uint32_t need = bpf - c.carry_n;
        uint32_t take = (uint32_t)(n < need ? n : need);
        if (c.carry_n + take == bpf && net_input_ring_free_bytes() < bpf) return 0;
        memcpy(c.carry + c.carry_n, d, take);
        c.carry_n = (uint8_t)(c.carry_n + take);
        used = take;
        if (c.carry_n < bpf) return used;
        uint8_t f[6];
        for (uint32_t i = 0; i < bpf; i++) f[i] = c.pcm_be ? c.carry[(i / bps) * bps + (bps - 1 - i % bps)] : c.carry[i];
        if (net_input_stage_pcm(0, f, bpf)) net_input_commit_pcm(bpf);
        c.carry_n = 0;
    }

    size_t frames = (n - used) / bpf;
    size_t maxf = net_input_ring_free_bytes() / bpf;
    bool all = frames <= maxf;
    if (!all) frames = maxf;
    size_t bytes = frames * bpf;
    if (bytes) {
        if (!c.pcm_be) {
            net_input_stage_pcm(0, d + used, (uint32_t)bytes);
        } else {
            uint8_t tmp[SL_PCM_TMP];                   /* multiple of 4 and 6 */
            size_t off = 0;
            while (off < bytes) {
                size_t k = bytes - off < SL_PCM_TMP ? bytes - off : SL_PCM_TMP;
                for (size_t i = 0; i < k; i += bps)
                    for (uint32_t j = 0; j < bps; j++) tmp[i + j] = d[used + off + i + bps - 1 - j];
                net_input_stage_pcm((uint32_t)off, tmp, (uint32_t)k);
                off += k;
            }
        }
        net_input_commit_pcm((uint32_t)bytes);
        used += bytes;
    }
    if (all) {                                         /* a trailing partial frame is carried */
        size_t rem = n - used;
        if (rem) { memcpy(c.carry, d + used, rem); c.carry_n = (uint8_t)rem; used += rem; }
    }
    return used;
}

static void pcm_params_from_strm(uint8_t size, uint8_t rate, uint8_t chans, uint8_t endian)
{
    static const uint32_t rates[] = { 11025, 22050, 32000, 44100, 48000, 8000, 12000, 16000, 24000, 96000 };
    c.pcm_known = false;
    if (size < '0' || size > '3' || rate < '0' || rate > '9' || chans != '2' ||
        (endian != '0' && endian != '1')) return;
    uint32_t bytes = (uint32_t)(size - '0') + 1u;
    uint32_t r = rates[rate - '0'];
    if ((bytes != 2 && bytes != 3) || (r != 44100 && r != 48000)) {
        c.pcm_known = false;
        c.pcm_rate = 1;                                /* known but unsupported -> rejected at the start */
        return;
    }
    c.pcm_known = true; c.pcm_rate = r; c.pcm_bps = (uint8_t)bytes; c.pcm_be = endian == '0';
}

/* WAV header: RIFF(12) then chunks until 'data'.  Returns bytes consumed;
 * sets c.ps / flags.  Format errors call stream_error() (stream is reset). */
static size_t wav_feed(const uint8_t *d, size_t n)
{
    size_t used = 0;
    while (used < n && c.ps != PS_APPLY && c.ps != PS_PCM) {
        if (c.ps == PS_WAV_SKIP) {
            uint32_t k = (uint32_t)(n - used < c.w_skip ? n - used : c.w_skip);
            c.w_skip -= k; used += k;
            if (!c.w_skip) { c.ps = PS_WAV_CH; c.w_have = 0; c.w_need = 8; }
            continue;
        }
        uint32_t k = (uint32_t)(n - used < (size_t)(c.w_need - c.w_have) ? n - used : (size_t)(c.w_need - c.w_have));
        memcpy(c.wbuf + c.w_have, d + used, k);
        c.w_have = (uint8_t)(c.w_have + k); used += k;
        if (c.w_have < c.w_need) break;
        if (c.ps == PS_WAV_RIFF) {
            if (memcmp(c.wbuf + 8, "WAVE", 4)) { stream_error(); return n; }
            c.ps = PS_WAV_CH; c.w_have = 0; c.w_need = 8;
        } else if (c.ps == PS_WAV_CH) {
            uint32_t sz = (uint32_t)c.wbuf[4] | ((uint32_t)c.wbuf[5] << 8) | ((uint32_t)c.wbuf[6] << 16) | ((uint32_t)c.wbuf[7] << 24);
            if (!memcmp(c.wbuf, "fmt ", 4)) {
                if (sz < 16 || sz > 40) { stream_error(); return n; }
                c.ps = PS_WAV_FMT; c.w_have = 0; c.w_need = (uint8_t)sz;
                c.w_skip = (sz & 1u);                  /* pad byte after an odd-sized chunk (rare) */
            } else if (!memcmp(c.wbuf, "data", 4)) {
                c.w_limited = sz != 0 && sz != 0xffffffffu;
                c.w_dataleft = sz;
                c.ps = PS_APPLY;
            } else {
                c.w_skip = sz + (sz & 1u);
                c.ps = PS_WAV_SKIP;
            }
        } else if (c.ps == PS_WAV_FMT) {
            uint16_t tag = (uint16_t)(c.wbuf[0] | (c.wbuf[1] << 8));
            uint16_t ch = (uint16_t)(c.wbuf[2] | (c.wbuf[3] << 8));
            uint32_t rate = (uint32_t)c.wbuf[4] | ((uint32_t)c.wbuf[5] << 8) | ((uint32_t)c.wbuf[6] << 16) | ((uint32_t)c.wbuf[7] << 24);
            uint16_t bits = (uint16_t)(c.wbuf[14] | (c.wbuf[15] << 8));
            if ((tag != 1 && tag != 0xfffe) || ch != 2 || (rate != 44100 && rate != 48000) ||
                (bits != 16 && bits != 24)) { stream_error(); return n; }
            c.pcm_rate = rate; c.pcm_bps = (uint8_t)(bits / 8u); c.pcm_be = false; c.pcm_known = true;
            c.ps = c.w_skip ? PS_WAV_SKIP : PS_WAV_CH;
            c.w_have = 0; c.w_need = 8;
        }
    }
    return used;
}

static size_t pcm_feed(const uint8_t *d, size_t n)
{
    size_t used = 0;
    while (used < n) {
        switch (c.ps) {
        case PS_DETECT: {
            uint32_t take = (uint32_t)(n - used < (size_t)(4 - c.det_n) ? n - used : (size_t)(4 - c.det_n));
            memcpy(c.det + c.det_n, d + used, take);
            c.det_n = (uint8_t)(c.det_n + take); used += take;
            if (c.det_n < 4) return used;
            if (!memcmp(c.det, "RIFF", 4)) {
                memcpy(c.wbuf, c.det, 4);
                c.w_have = 4; c.w_need = 12; c.ps = PS_WAV_RIFF; c.det_n = 0;
            } else if (c.pcm_known) {
                c.det_off = 0; c.ps = PS_APPLY;
            } else {                                   /* '?' parameters and no WAV header */
                stream_error();
                return n;
            }
            break;
        }
        case PS_WAV_RIFF: case PS_WAV_CH: case PS_WAV_FMT: case PS_WAV_SKIP: {
            size_t k = wav_feed(d + used, n - used);
            if (c.sp != SP_BODY) return n;             /* error: stream was reset */
            used += k;
            break;
        }
        case PS_APPLY:
            if (!c.fmt_applied && !apply_format(c.pcm_rate, (uint8_t)(c.pcm_bps * 8u))) return used;   /* wait for the old track to drain */
            c.ps = PS_PCM;
            break;
        case PS_PCM: {
            if (c.det_n > c.det_off) {                 /* the 4 detection bytes were raw audio */
                size_t k = pcm_push(c.det + c.det_off, (size_t)(c.det_n - c.det_off));
                c.det_off = (uint8_t)(c.det_off + k);
                if (c.det_off < c.det_n) return used;
                c.det_n = c.det_off = 0;
            }
            size_t avail = n - used;
            if (c.w_limited) {
                if (c.w_dataleft == 0) return n;        /* data chunk finished: ignore trailing chunks */
                if (avail > c.w_dataleft) avail = c.w_dataleft;
            }
            size_t k = pcm_push(d + used, avail);
            used += k;
            if (c.w_limited) c.w_dataleft -= (uint32_t)k;
            if (k < avail) return used;                 /* ring full */
            break;
        }
        default:
            return n;
        }
    }
    return used;
}

static size_t flac_feed(const uint8_t *d, size_t n)
{
    if (!nfl_active()) return n;
    if (c.ps == PS_DETECT) {                           /* foxen would search forever for the marker: check it here */
        size_t used = 0;
        while (c.det_n < 4 && used < n) c.det[c.det_n++] = d[used++];
        if (c.det_n < 4) return used;
        if (memcmp(c.det, "fLaC", 4)) { stream_error(); return n; }
        memcpy(nfl_chunk_buf(), c.det, 4);
        nfl_chunk_submit(4);
        nfl_pump();
        c.ps = PS_PCM;                                  /* marker seen */
        c.det_n = 0;
        return used;
    }
    if (nfl_pending()) {
        nfl_pump();
        if (nfl_pending()) return 0;
    }
    uint32_t cap = nfl_chunk_cap();
    size_t take = n < cap ? n : cap;
    memcpy(nfl_chunk_buf(), d, take);
    nfl_chunk_submit((uint32_t)take);
    nfl_pump();
    return take;
}

static size_t codec_feed(const uint8_t *d, size_t n)
{
    return c.fmt == 'f' ? flac_feed(d, n) : pcm_feed(d, n);
}

/* ------------------------------------------------------------------ */
/* HTTP stream: header / chunked / body                                */
/* ------------------------------------------------------------------ */
static void body_end(uint8_t dsco)
{
    if (c.body_complete) return;
    c.body_complete = true;
    spcb_release(true);
    q_push(Q_DSCO, NULL, dsco);
}

static bool hdr_value(const char *name, size_t nl, const char **val)
{
    char *s = (char *)c.rsp + 8;
    for (char *p = strstr(s, "\r\n"); p && *p; p = strstr(p + 2, "\r\n")) {
        char *l = p + 2;
        if (strncasecmp(l, name, nl) == 0) {
            l += nl;
            while (*l == ' ' || *l == '\t') l++;
            *val = l;
            return true;
        }
    }
    return false;
}

/* The response header is complete in c.rsp+8 (NUL terminated at hdr_have). */
static bool http_headers_done(void)
{
    char *s = (char *)c.rsp + 8;
    int status = 0;
    if (!strncmp(s, "HTTP/1.", 7) && s[8] == ' ') status = atoi(s + 9);
    c.resp_len = c.hdr_have;
    q_push(Q_RESP, NULL, 0);                       /* the server sees whatever we got (also errors/redirects) */
    if (status != 200 && status != 206) {
        SL_LOG("http status %d\n", status);
        return false;
    }
    c.clen = -1; c.chunked = false;
    const char *v;
    if (hdr_value("content-length:", 15, &v)) c.clen = atoll(v);
    if (hdr_value("transfer-encoding:", 18, &v) && strncasecmp(v, "chunked", 7) == 0) c.chunked = true;
    c.cs = CS_SIZE; c.cs_acc = 0; c.cs_digits = 0;
    return true;
}

/* Body bytes -> codec, removing chunked framing / honouring Content-Length.
 * Returns bytes consumed (< n only when the codec could not take more). */
static size_t body_feed(const uint8_t *d, size_t n)
{
    size_t used = 0;
    if (!c.chunked) {
        size_t orig = n;
        if (c.clen >= 0 && (uint64_t)n > (uint64_t)c.clen) n = (size_t)c.clen;   /* ignore bytes beyond the length */
        used = n ? codec_feed(d, n) : 0;
        if (c.clen >= 0) { c.clen -= (int64_t)used; if (c.clen == 0) body_end(DSCO_OK); }
        return c.body_complete ? orig : used;
    }
    while (used < n && c.cs != CS_END) {
        uint8_t b = d[used];
        switch (c.cs) {
        case CS_SIZE:
            if (b >= '0' && b <= '9') c.cs_acc = (c.cs_acc << 4) | (uint32_t)(b - '0');
            else if (b >= 'a' && b <= 'f') c.cs_acc = (c.cs_acc << 4) | (uint32_t)(b - 'a' + 10);
            else if (b >= 'A' && b <= 'F') c.cs_acc = (c.cs_acc << 4) | (uint32_t)(b - 'A' + 10);
            else if (b == ';') { c.cs = CS_EXT; used++; continue; }
            else if (b == '\r') { c.cs = CS_SIZE_LF; used++; continue; }
            else { stream_error(); return n; }
            if (++c.cs_digits > 7) { stream_error(); return n; }
            used++;
            break;
        case CS_EXT:
            if (b == '\r') c.cs = CS_SIZE_LF;
            used++;
            break;
        case CS_SIZE_LF:
            if (b != '\n') { stream_error(); return n; }
            used++;
            if (c.cs_acc == 0) { c.cs = CS_END; body_end(DSCO_OK); }      /* trailers are ignored */
            else { c.cs_left = c.cs_acc; c.cs = CS_DATA; }
            break;
        case CS_DATA: {
            size_t k = n - used < c.cs_left ? n - used : c.cs_left;
            size_t got = codec_feed(d + used, k);
            used += got; c.cs_left -= (uint32_t)got;
            if (got < k) return used;
            if (c.cs_left == 0) c.cs = CS_DATA_CR;
            break;
        }
        case CS_DATA_CR:
            if (b != '\r') { stream_error(); return n; }
            c.cs = CS_DATA_LF; used++;
            break;
        case CS_DATA_LF:
            if (b != '\n') { stream_error(); return n; }
            c.cs = CS_SIZE; c.cs_acc = 0; c.cs_digits = 0; used++;
            break;
        default:
            return n;
        }
        if (c.sp != SP_BODY) return n;                  /* reset by an error */
    }
    return c.cs == CS_END ? n : used;                   /* bytes after the last chunk are ignored */
}

/* Feed one TCP segment to the stream state machine.  Returns bytes consumed. */
static size_t stream_feed(const uint8_t *d, size_t n)
{
    size_t i = 0;
    if (c.sp == SP_HDR) {
        while (i < n) {
            if (c.hdr_have + 1u >= SL_HDR_BUF) { stream_error(); return n; }
            c.rsp[8 + c.hdr_have++] = d[i++];
            if (c.hdr_have >= 4 && !memcmp(c.rsp + 8 + c.hdr_have - 4, "\r\n\r\n", 4)) {
                c.rsp[8 + c.hdr_have] = 0;
                if (!http_headers_done()) {
                    stream_reset();
                    c.sp = SP_DONE;
                    stat_evt("STMn", 0);
                    return n;
                }
                c.sp = c.autostart >= 2 ? SP_WAIT_CONT : SP_BODY;
                c.sp_start_us = time_us_64();
                if (!c.chunked && c.clen == 0) body_end(DSCO_OK);
                break;
            }
        }
        if (c.sp == SP_HDR) return i;
    }
    if (c.sp == SP_WAIT_CONT) return i;                 /* hold the rest until cont */
    if (c.sp == SP_BODY) {
        if (c.body_complete) return n;                  /* ignore trailing junk */
        i += body_feed(d + i, n - i);
    }
    return i;
}

/* Drain the held pbuf chain into the stream state machine. */
static void stream_drain(void)
{
    while (c.hold && c.sp >= SP_HDR && c.sp < SP_DONE) {
        struct pbuf *seg = c.hold_seg;
        if (!seg) break;
        size_t avail = seg->len - c.hold_off;
        size_t used = avail ? stream_feed((const uint8_t *)seg->payload + c.hold_off, avail) : 0;
        if (!c.hold) return;                            /* stream was reset inside (error): chain already freed */
        if (used) {
            if (c.spcb) tcp_recved(c.spcb, (u16_t)used);
            c.bytes_rx += used;
            c.hold_off = (uint16_t)(c.hold_off + used);
        }
        if (c.hold_off >= seg->len) { c.hold_seg = seg->next; c.hold_off = 0; continue; }
        if (c.sp != SP_WAIT_CONT && !c.s_blocked) { c.s_blocked = true; c.s_block_us = time_us_64(); }
        return;                                         /* backpressure: keep the rest un-acked */
    }
    c.s_blocked = false;
    if (c.hold && !c.hold_seg) { pbuf_free(c.hold); c.hold = NULL; c.hold_off = 0; }
}

/* ------------------------------------------------------------------ */
/* HTTP stream: lwIP callbacks                                         */
/* ------------------------------------------------------------------ */
static err_t on_srecv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
    (void)arg;
    c.s_abort = false;
    if (!g_c || pcb != c.spcb) { if (p) pbuf_free(p); return ERR_OK; }
    if (!p) {                                           /* FIN */
        c.s_fin = true;
        c.s_rx_us = time_us_64();
        if (!c.hold) {
            if (c.sp == SP_HDR) { stream_error(); }
            else if (c.sp == SP_BODY || c.sp == SP_WAIT_CONT) body_end(DSCO_OK);
        }
        return c.s_abort ? ERR_ABRT : ERR_OK;
    }
    if (err != ERR_OK) { pbuf_free(p); return ERR_OK; }
    c.s_rx_us = time_us_64();
    if (c.hold) pbuf_cat(c.hold, p);
    else { c.hold = p; c.hold_seg = p; c.hold_off = 0; }
    stream_drain();
    return c.s_abort ? ERR_ABRT : ERR_OK;
}

static void on_serr(void *arg, err_t err)
{
    (void)arg; (void)err;
    if (!g_c) return;
    c.spcb = NULL;                                      /* already freed by lwIP */
    if (c.sp == SP_CONNECT || c.sp == SP_HDR) {
        q_push(Q_DSCO, NULL, DSCO_UNREACH);
        stream_error();
    } else if (c.sp == SP_BODY || c.sp == SP_WAIT_CONT) {
        c.s_fin = true;                                 /* decode what was received, then finish */
        if (!c.hold) body_end(DSCO_REMOTE);
    }
}

static err_t on_sconnected(void *arg, struct tcp_pcb *pcb, err_t err)
{
    (void)arg;
    if (!g_c || pcb != c.spcb || err != ERR_OK) return ERR_OK;
    if (tcp_write(pcb, c.rsp + 8, c.req_len, TCP_WRITE_FLAG_COPY) != ERR_OK) {
        stream_error();
        return c.s_abort ? ERR_ABRT : ERR_OK;
    }
    tcp_output(pcb);
    c.req_sent = true;
    c.hdr_have = 0;                                     /* the request buffer becomes the header buffer */
    c.sp = SP_HDR;
    c.sp_start_us = time_us_64();
    return ERR_OK;
}

static void stream_begin(const uint8_t *f, uint32_t len)
{
    /* f = strm frame incl. opcode, len bytes, len >= 28 */
    stream_reset();
    q_drop_resp();
    stat_evt("STMf", 0);

    uint32_t http_len = len - 28u;
    uint8_t fmt = f[6];
    c.autostart = (uint8_t)(f[5] - '0');
    c.threshold = (uint32_t)f[11] * 1024u;
    c.out_thr = f[16];
    uint8_t flags = f[15];
    c.sent_stml = c.sent_stmd = false;

    if ((fmt != 'f' && fmt != 'p') || (flags & 0x20) || http_len == 0 || http_len > SL_HDR_BUF - 1u ||
        c.autostart > 3) {
        SL_LOG("strm s refused: fmt %c flags %02x http %u\n", fmt, flags, (unsigned)http_len);
        c.sp = SP_DONE;
        stat_evt("STMn", 0);
        return;
    }
    c.fmt = fmt;
    c.ps = PS_DETECT; c.pcm_known = false; c.w_limited = false;
    if (fmt == 'p') {
        c.pcm_rate = 0;
        pcm_params_from_strm(f[7], f[8], f[9], f[10]);
        if (c.pcm_rate == 1 && !c.pcm_known) { c.fmt = 0; c.sp = SP_DONE; stat_evt("STMn", 0); return; }
    } else if (nfl_begin_stream() != NFL_OK) {
        c.fmt = 0; c.sp = SP_DONE;
        stat_evt("STMn", 0);
        return;
    }
    memcpy(c.rsp + 8, f + 28, http_len);
    c.req_len = (uint16_t)http_len;
    c.s_port = be16(f + 22);
    uint32_t ipraw;
    memcpy(&ipraw, f + 24, 4);                          /* network byte order == lwIP's representation */
    if (ipraw == 0) c.s_ip = c.srv_ip; else c.s_ip.addr = ipraw;

    struct tcp_pcb *pcb = tcp_new();
    if (!pcb) { stream_reset(); c.sp = SP_DONE; q_push(Q_DSCO, NULL, DSCO_UNREACH); stat_evt("STMn", 0); return; }
    c.spcb = pcb;
    c.sp = SP_CONNECT;
    c.sp_start_us = time_us_64();
    c.s_rx_us = c.sp_start_us;
    c.fmt_applied = false;
    c.body_complete = c.s_fin = c.s_blocked = false;
    c.carry_n = 0; c.det_n = c.det_off = 0;
    tcp_arg(pcb, NULL);
    tcp_recv(pcb, on_srecv);
    tcp_err(pcb, on_serr);
    tcp_nagle_disable(pcb);
    sl_set_state(SLIMPROTO_STATE_STREAMING);
    if (tcp_connect(pcb, &c.s_ip, c.s_port, on_sconnected) != ERR_OK) {
        spcb_release(false);
        q_push(Q_DSCO, NULL, DSCO_UNREACH);
        stream_error();
        return;
    }
    stat_evt("STMc", 0);
}

/* Per-poll stream work: timeouts, FLAC pump / format hand-over, completion. */
static void stream_service(uint64_t now)
{
    if (c.sp == SP_NONE || c.sp == SP_DONE) return;

    if (nfl_take_evicted()) {                           /* ADAT/TDM output reclaimed the decoder arena */
        q_push(Q_DSCO, NULL, DSCO_REMOTE);
        stream_error();
        return;
    }
    if (c.sp == SP_CONNECT && now - c.sp_start_us > SL_STREAM_CONN_US) {
        q_push(Q_DSCO, NULL, DSCO_TIMEOUT); stream_error(); return;
    }
    if (c.sp == SP_HDR && now - c.sp_start_us > SL_STREAM_CONN_US) {
        q_push(Q_DSCO, NULL, DSCO_TIMEOUT); stream_error(); return;
    }
    if (c.sp == SP_WAIT_CONT && now - c.sp_start_us > SL_CONT_WAIT_US) {
        c.autostart = (uint8_t)(c.autostart - 2);   /* server never answered: go on */
        c.sp = SP_BODY;
    }

    if (c.sp == SP_BODY) {
        if (c.fmt == 'f' && nfl_active()) {
            if (nfl_pending()) nfl_pump();
            uint32_t r; uint8_t b;
            if (!c.fmt_applied && nfl_stream_info(&r, &b) && apply_format(r, b)) {
                nfl_stream_ack();
                if (nfl_pending()) nfl_pump();
            }
            if (nfl_stream_failed()) { stream_error(); return; }
        }
        if (c.hold) stream_drain();
        if (c.sp != SP_BODY) return;

        /* FIN seen and everything held was consumed */
        if (c.s_fin && !c.hold && !c.body_complete) body_end(DSCO_OK);

        /* stalls */
        if (c.s_blocked && c.out_state == OUT_RUNNING && now - c.s_block_us > SL_STALL_US) {
            q_push(Q_DSCO, NULL, DSCO_TIMEOUT); stream_error(); return;
        }
        if (!c.s_blocked && !c.body_complete && now - c.s_rx_us > SL_STREAM_IDLE_US && !(c.fmt == 'f' && nfl_pending())) {
            body_end(DSCO_TIMEOUT);
        }
        if (c.body_complete && !c.hold && (c.fmt != 'f' || !nfl_active() || !nfl_pending())) {
            /* decode complete */
            c.sp = SP_DONE;
            if (c.fmt == 'f') nfl_end();
            c.carry_n = 0;
            if (!c.sent_stmd) { c.sent_stmd = true; stat_evt("STMd", 0); }
            if (c.state == SLIMPROTO_STATE_STREAMING) sl_set_state(SLIMPROTO_STATE_READY);
        }
    }
}

/* ------------------------------------------------------------------ */
/* control: incoming commands                                          */
/* ------------------------------------------------------------------ */
static void do_flush(bool stop)
{
    stream_reset();
    ring_reset();
    out_set(OUT_STOPPED);
    c.pause_end_us = 0;
    (void)stop;
    stat_evt("STMf", 0);
}

static void on_strm(const uint8_t *f, uint32_t len)
{
    if (len < 28) return;
    switch (f[4]) {
    case 's':
        stream_begin(f, len);
        break;
    case 'p': {
        uint32_t ms = be32(f + 18);
        out_set(OUT_PAUSED);
        c.pause_end_us = ms ? time_us_64() + (uint64_t)ms * 1000ull : 0;
        if (!ms) stat_evt("STMp", 0);
        break;
    }
    case 'u':
        c.pause_end_us = 0;               /* UNVERIFIED: a non-zero "start at" jiffies (sync) is ignored: start now */
        out_set(OUT_RUNNING);
        stat_evt("STMr", 0);
        break;
    case 'q':
        do_flush(true);
        break;
    case 'f':
        do_flush(false);
        break;
    case 't':
        stat_evt("STMt", be32(f + 18));    /* echo the timestamp field */
        break;
    case 'a':
        break;                            /* skip-ahead (sync): not supported */
    default:
        SL_LOG("strm %c ignored\n", f[4]);
        break;
    }
}

static void on_audg(const uint8_t *f, uint32_t len)
{
    uint32_t gl, gr;
    if (len >= 22) { gl = be32(f + 14); gr = be32(f + 18); }
    else if (len >= 14) { gl = be32(f + 4); gr = be32(f + 8); }
    else return;
    bool adjust = len >= 13 ? f[12] != 0 : true;
    if (!adjust) gl = gr = 65536u;
    uint32_t g = (uint32_t)(((uint64_t)gl + gr) / 2u);                 /* one volume control: average the channels */
    uint8_t pct = gain_to_pct(g);
    net_volume_set_percent(pct);
    c.gain_pct = pct;
}

static void on_aude(const uint8_t *f, uint32_t len)
{
    if (len < 6) return;
    bool on = f[4] != 0;                                              /* spdif_enable: 0 = muted / off */
    if (!on) net_mute_set(true);
    else if (c.aude_seen && !c.aude_on) net_mute_set(false);          /* never unmute on the very first message */
    c.aude_seen = true; c.aude_on = on;
}

static void on_setd(const uint8_t *f, uint32_t len)
{
    if (len < 5 || f[4] != 0) return;
    if (len > 5) {
        uint32_t n = len - 5;
        if (n > sizeof(c.name) - 1) n = sizeof(c.name) - 1;
        memcpy(c.name, f + 5, n);
        c.name[n] = 0;
    }
    q_push(Q_SETD, NULL, 0);
}

static void on_codc(const uint8_t *f, uint32_t len)
{
    if (len < 9 || c.sp < SP_HDR || c.sp >= SP_DONE) return;
    uint8_t fmt = f[4];
    if (fmt == 'p' && c.fmt == 'p') {
        bool k = c.pcm_known; uint32_t r = c.pcm_rate; uint8_t b = c.pcm_bps; bool be = c.pcm_be;
        pcm_params_from_strm(f[5], f[6], f[7], f[8]);
        if (!c.pcm_known && k) { c.pcm_known = k; c.pcm_rate = r; c.pcm_bps = b; c.pcm_be = be; }   /* keep the strm values */
    }
    /* UNVERIFIED: a codc that changes the container (e.g. 'p' -> 'f') after the stream started is ignored */
}

static void on_frame(const uint8_t *f, uint32_t len)
{
    c.last_rx_us = time_us_64();
    if (c.state == SLIMPROTO_STATE_HANDSHAKE) {
        sl_set_state(c.sp > SP_NONE && c.sp < SP_DONE ? SLIMPROTO_STATE_STREAMING : SLIMPROTO_STATE_READY);
        c.backoff_ms = SL_BACKOFF_MIN_MS;
    }
    if (!memcmp(f, "strm", 4)) on_strm(f, len);
    else if (!memcmp(f, "cont", 4)) {
        if (c.sp == SP_WAIT_CONT) {
            if (c.autostart >= 2) c.autostart = (uint8_t)(c.autostart - 2);
            c.sp = SP_BODY;
        }
    }
    else if (!memcmp(f, "audg", 4)) on_audg(f, len);
    else if (!memcmp(f, "aude", 4)) on_aude(f, len);
    else if (!memcmp(f, "setd", 4)) on_setd(f, len);
    else if (!memcmp(f, "codc", 4)) on_codc(f, len);
    else if (!memcmp(f, "serv", 4)) { /* UNVERIFIED: server switch not supported; stay connected */ }
    else if (!memcmp(f, "vers", 4)) { /* server version string: ignored */ }
    else SL_LOG("unhandled opcode %.4s\n", f);
}

static void ctl_feed(const uint8_t *d, size_t n)
{
    while (n && !c.fatal) {
        if (c.rx_hdr_have < 2) {
            c.rx_hdr[c.rx_hdr_have++] = *d++; n--;
            if (c.rx_hdr_have == 2) {
                c.rx_len = be16(c.rx_hdr);
                c.rx_have = 0;
                c.rx_skip = c.rx_len > SL_RX_BUF;
                if (c.rx_len > SL_MAX_FRAME) { c.fatal = true; return; }
                if (c.rx_len == 0) c.rx_hdr_have = 0;
            }
            continue;
        }
        size_t take = c.rx_len - c.rx_have;
        if (take > n) take = n;
        if (c.rx_have < SL_RX_BUF) {
            size_t room = SL_RX_BUF - c.rx_have;
            memcpy(c.rx + c.rx_have, d, take < room ? take : room);
        }
        c.rx_have = (uint16_t)(c.rx_have + take);
        d += take; n -= take;
        if (c.rx_have == c.rx_len) {
            c.rx_hdr_have = 0;
            if (!c.rx_skip) {
                if (c.rx_len >= 4) on_frame(c.rx, c.rx_len);
            } else {
                if (!memcmp(c.rx, "strm", 4) && c.rx[4] == 's') { stream_reset(); stat_evt("STMf", 0); c.sp = SP_DONE; stat_evt("STMn", 0); }
                SL_LOG("oversize frame skipped (%u)\n", (unsigned)c.rx_len);
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/* control connection                                                  */
/* ------------------------------------------------------------------ */
static void cleanup(bool graceful)
{
    stream_reset();
    if (c.pcb) {
        struct tcp_pcb *p = c.pcb;
        c.pcb = NULL;
        tcp_arg(p, NULL); tcp_recv(p, NULL); tcp_sent(p, NULL); tcp_err(p, NULL);
        if (graceful) {
            if (tcp_close(p) != ERR_OK) { tcp_abort(p); c.abort_flag = true; }
        } else {
            tcp_abort(p);
            c.abort_flag = true;
        }
    }
    c.dns_pending = false;
    c.tx_off = c.tx_end = 0;
    q_clear();
    c.fatal = false;
    c.rx_hdr_have = 0; c.rx_have = 0; c.rx_len = 0; c.rx_skip = false;
    set_phase(PH_NONE);
}

static void conn_fail(const char *why, bool graceful)
{
    (void)why;
    SL_LOG("fail: %s\n", why);
    cleanup(graceful);
    /* audio of the dead session is meaningless; the server restarts playback */
    ring_reset();
    out_set(OUT_STOPPED);
    c.pause_end_us = 0;
    if (!c.enabled) { sl_set_state(SLIMPROTO_STATE_IDLE); return; }
    sl_set_state(SLIMPROTO_STATE_ERROR);
    c.retry_at_us = time_us_64() + (uint64_t)c.backoff_ms * 1000ull;
    c.backoff_ms *= 2;
    if (c.backoff_ms > SL_BACKOFF_MAX_MS) c.backoff_ms = SL_BACKOFF_MAX_MS;
}

static err_t on_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
    (void)arg;
    if (!g_c) { if (p) pbuf_free(p); return ERR_OK; }
    c.abort_flag = false;
    if (!p) { conn_fail("peer closed", true); return c.abort_flag ? ERR_ABRT : ERR_OK; }
    if (err != ERR_OK || pcb != c.pcb) { pbuf_free(p); return ERR_OK; }
    for (struct pbuf *q = p; q; q = q->next) ctl_feed((const uint8_t *)q->payload, q->len);
    tcp_recved(pcb, p->tot_len);
    pbuf_free(p);
    if (c.fatal) { conn_fail("protocol", false); return c.abort_flag ? ERR_ABRT : ERR_OK; }
    service_tx();
    return c.abort_flag ? ERR_ABRT : ERR_OK;
}

static err_t on_sent(void *arg, struct tcp_pcb *pcb, u16_t len)
{
    (void)arg; (void)pcb; (void)len;
    if (g_c) service_tx();
    return ERR_OK;
}

static void on_err(void *arg, err_t err)
{
    (void)arg; (void)err;
    if (!g_c) return;
    c.pcb = NULL;                            /* already freed by lwIP */
    conn_fail("tcp error", false);
}

static err_t on_connected(void *arg, struct tcp_pcb *pcb, err_t err)
{
    (void)arg;
    if (!g_c || err != ERR_OK || pcb != c.pcb) return ERR_OK;
    set_phase(PH_RUN);
    sl_set_state(SLIMPROTO_STATE_HANDSHAKE);
    c.last_rx_us = time_us_64();
    c.need_helo = true;
    service_tx();
    c.reconnect = true;
    return ERR_OK;
}

static void do_connect(const ip_addr_t *ip)
{
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb) { conn_fail("tcp_new", false); return; }
    c.srv_ip = *ip;
    c.pcb = pcb;
    tcp_arg(pcb, NULL);
    tcp_recv(pcb, on_recv);
    tcp_sent(pcb, on_sent);
    tcp_err(pcb, on_err);
    tcp_nagle_disable(pcb);
    set_phase(PH_TCP);
    if (tcp_connect(pcb, ip, net_slimproto_port(), on_connected) != ERR_OK) conn_fail("tcp_connect", false);
}

static void dns_found(const char *name, const ip_addr_t *ip, void *arg)
{
    (void)name; (void)arg;
    if (!g_c || !c.dns_pending) return;
    c.dns_pending = false;
    if (!ip) { conn_fail("dns", false); return; }
    do_connect(ip);
}

static void begin_connect(void)
{
    const char *host = net_sendspin_host();
    if (!host || !*host || !net_slimproto_port()) { conn_fail("no host configured", false); return; }
    sl_set_state(SLIMPROTO_STATE_CONNECTING);
    set_phase(PH_DNS);
    ip_addr_t ip;
    err_t e = dns_gethostbyname(host, &ip, dns_found, NULL);
    if (e == ERR_OK) do_connect(&ip);
    else if (e == ERR_INPROGRESS) c.dns_pending = true;
    else conn_fail("dns request", false);
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */
void slimproto_client_start(void)
{
    if (g_c) return;
    uint32_t bytes = 0;
    void *mem = sendspin_client_mem_loan(&bytes);
    if (!mem || bytes < sizeof(sl_t)) {
        if (mem) sendspin_client_mem_return();
        SL_LOG("no RAM loan\n");
        return;                               /* state() reports IDLE */
    }
    memset(mem, 0, sizeof(sl_t));
    g_c = (sl_t *)mem;
    c.enabled = true;
    c.backoff_ms = SL_BACKOFF_MIN_MS;
    strncpy(c.name, SLIMPROTO_PLAYER_NAME, sizeof(c.name) - 1);
    c.state = SLIMPROTO_STATE_IDLE;
    c.fmt_set = false;
    c.gain_pct = 0xFF;
    out_set(OUT_STOPPED);
}

void slimproto_client_stop(void)
{
    if (!g_c) return;
    c.enabled = false;
    cleanup(true);
    ring_reset();
    net_input_set_hold(false);
    g_c = NULL;
    sendspin_client_mem_return();
}

bool slimproto_client_streaming(void) { return g_c && c.sp >= SP_CONNECT && c.sp < SP_DONE; }
uint8_t slimproto_client_state(void) { return g_c ? c.state : SLIMPROTO_STATE_IDLE; }

void slimproto_client_poll(void)
{
    if (!g_c || !c.enabled) return;
    uint64_t now = time_us_64();

    if (!c.pcb && !c.dns_pending && c.phase == PH_NONE &&
        (c.state == SLIMPROTO_STATE_IDLE || c.state == SLIMPROTO_STATE_ERROR) &&
        now >= c.retry_at_us) {
        begin_connect();
        return;
    }
    if (c.phase == PH_NONE) return;

    if (c.fatal) { conn_fail("fatal", false); return; }
    if (c.state != SLIMPROTO_STATE_READY && c.state != SLIMPROTO_STATE_STREAMING &&
        now - c.phase_start_us > SL_PHASE_TIMEOUT_US) {
        conn_fail("timeout", false);
        return;
    }
    if (c.phase == PH_RUN && now - c.last_rx_us > SL_SERVER_IDLE_US) {
        conn_fail("server silent", false);
        return;
    }
    if (!c.pcb) return;

    if (c.phase == PH_RUN) {
        stream_service(now);
        if (!g_c) return;
        play_service(now);
    }
    service_tx();
    if (c.fatal) conn_fail("fatal", false);
}

#ifdef SLIMPROTO_TEST_HOOKS
/* Host test only: size of the borrowed state (must fit sendspin_client_mem_loan()). */
size_t slimproto_state_bytes(void) { return sizeof(sl_t); }
#endif
