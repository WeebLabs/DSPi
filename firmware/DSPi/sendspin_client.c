/*
 * sendspin_client.c - minimal Sendspin player client (protocol layer).
 *
 * Flow (client-initiated, plain ws:// transport, Noise inside):
 *   TCP connect -> HTTP/1.1 Upgrade (RFC 6455) -> WS text: client/init ->
 *   WS text: server/init -> WS text: noise/handshake (msg1, from server) ->
 *   WS text: noise/handshake (msg2, ours, payload "{}") ->
 *   [encrypted binary frames from here] server/hello -> client/hello ->
 *   client/time x8 burst -> client/state(available) -> server/activate,
 *   stream/start, audio chunks (type 4), server/command ...
 *
 * Everything is driven from lwIP raw callbacks + sendspin_client_poll() on
 * the main-loop thread.
 *
 * Receive path / memory: encrypted records are NOT reassembled.  Each record is
 * one WebSocket binary frame (ciphertext || 16-byte tag, length known from the
 * frame header) and is decrypted with the streaming ChaCha20-Poly1305 API while
 * its bytes arrive from TCP:
 *   - audio chunks (id 4): the PCM is decrypted straight into the FREE part of
 *     the NET PCM ring (net_input_stage_pcm(), beyond the write index, invisible
 *     to the consumer) and published with net_input_commit_pcm() only after the
 *     Poly1305 tag verified; a failed tag publishes nothing and drops the link.
 *     The largest chunk is therefore the ring size (32 KB = 170 ms at 48 kHz /
 *     16-bit stereo; a 150 ms chunk is 28.8 KB), with no extra reassembly RAM.
 *     If the ring is momentarily too full the stream simply stalls (TCP window
 *     closes) until the consumer has drained enough; a stall > 5 s drops the link.
 *   - JSON and other small records: decrypted into the small g_rx buffer
 *     (SS_RX_BUF); larger ones are still authenticated but ignored.
 * Static RAM of this module is ~5 KB (g_rx 2 KB + state).
 *
 * Items I could not verify byte-for-byte against the real server are marked
 * "UNVERIFIED:" and are collected in the constants/functions noted there.
 */
#include "sendspin_client.h"
#include "sendspin_noise.h"
#include "net_input.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "lwip/tcp.h"
#include "lwip/dns.h"
#include "lwip/pbuf.h"
#include "lwip/ip_addr.h"
#include "pico/time.h"
#include "pico/rand.h"

#include "mbedtls/sha1.h"

#ifdef SENDSPIN_DEBUG
#define SS_LOG(...) printf("[sendspin] " __VA_ARGS__)
#else
#define SS_LOG(...) do {} while (0)
#endif

/* ------------------------------------------------------------------ */
/* Tunables                                                            */
/* ------------------------------------------------------------------ */
#ifndef SS_RX_BUF
#define SS_RX_BUF            2048    /* HTTP header / text frames / decrypted JSON (+NUL); bigger JSON records are skipped */
#endif
#define SS_BLK               256     /* decrypt block size (scratch ds_out) */
#define SS_STALL_TIMEOUT_US  5000000ull  /* ring full and not draining for this long -> reconnect */
#define SS_AUDIO_HDR         13      /* id(1) + timestamp(8) + send_ahead(4) */
_Static_assert(SS_RX_BUF >= 1100, "g_rx must hold the HTTP upgrade response (SS_HTTP_MAX) and handshake text");
#define SS_TX_BUF            1024    /* one outgoing frame at a time */
#define SS_PROLOGUE_MAX      640     /* client/init + server/init raw bytes */
#define SS_INIT_MAX          256     /* max client/init length */
#define SS_HTTP_MAX          1024    /* max HTTP upgrade response header */

#define SS_SYNC_BURST        8
#define SS_SYNC_INTERVAL_US  5000000ull   /* UNVERIFIED: spec says <= 10 s */
#define SS_SYNC_TIMEOUT_US   2000000ull
#define SS_PHASE_TIMEOUT_US  20000000ull
#define SS_BACKOFF_MIN_MS    1000u
#define SS_BACKOFF_MAX_MS    10000u
#define SS_LATE_SLACK_US     20000       /* drop chunks later than this after clear */

/* UNVERIFIED: values advertised in client/state (player@v1) */
#define SS_REQUIRED_LEAD_MS  100
#define SS_MIN_BUFFER_MS     100

/* UNVERIFIED: Noise suite string and PSK category we accept (Sentinel) */
#define SS_SUITE             "25519_ChaChaPoly_SHA256"
#define SS_SENTINEL_CAT      "sn"

/* UNVERIFIED: path; connection.md says /sendspin for both modes */
#define SS_WS_PATH           "/sendspin"

/* message ids (messaging.md) */
#define SS_ID_JSON           0
#define SS_ID_FRAGMENT       1
#define SS_ID_PLAYER_AUDIO   4

typedef enum {
    PH_NONE = 0, PH_DNS, PH_TCP, PH_HTTP,
    PH_WAIT_SERVER_INIT, PH_WAIT_NOISE1,
    PH_WAIT_HELLO,          /* encrypted from here on */
    PH_RUN
} phase_t;

enum { RX_HDR = 0, RX_PAYLOAD };
enum { DS_SINK = 0, DS_BUF, DS_RING };   /* where the plaintext of an encrypted record goes */

/* ------------------------------------------------------------------ */
/* State                                                               */
/* ------------------------------------------------------------------ */
static uint8_t g_rx[SS_RX_BUF] __attribute__((aligned(4)));

static struct {
    bool     enabled;
    uint8_t  state;
    uint8_t  phase;
    bool     fatal;
    bool     close_graceful;     /* peer sent WS close: close TCP gracefully */
    bool     abort_flag;          /* tcp_abort called in this callback */
    struct tcp_pcb *pcb;
    bool     dns_pending;
    uint32_t backoff_ms;
    uint64_t retry_at_us;
    uint64_t phase_start_us;
    uint64_t rx_us;               /* local time of the current tcp_recv */

    /* identity */
    bool     key_ok;
    uint8_t  s_priv[32], s_pub[32];
    char     client_id[48];

    /* websocket */
    char     ws_key[28];
    char     ws_accept[32];
    uint16_t http_have;

    /* ws rx parser */
    uint8_t  rx_st;
    uint8_t  hdr[14];
    uint8_t  hdr_have, hdr_need;
    uint8_t  opcode;
    bool     fin, masked;
    uint8_t  mask[4];
    uint64_t plen, pgot;
    uint8_t  msg_opcode;          /* 1 text / 2 binary while assembling, 0 none */
    uint32_t msg_len;
    uint64_t msg_total;
    bool     msg_discard;
    uint8_t  ctl[125];

    /* held pbuf chain (backpressure) */
    struct pbuf *hold;
    struct pbuf *hold_seg;
    uint16_t hold_off;

    /* streaming decrypt of the current encrypted binary record */
    bool     blocked;             /* no room in the PCM ring: waiting for the consumer */
    uint64_t blocked_since_us;
    bool     ds_on;
    bool     ds_bad;              /* protocol error to raise once the record authenticated */
    uint8_t  ds_route;            /* DS_*: valid once ds_got > 0 */
    uint8_t  ds_tag_have;
    uint32_t ds_len;              /* ciphertext bytes in the record (frame payload - tag) */
    uint32_t ds_got;              /* ciphertext bytes decrypted so far */
    uint8_t  ds_tag[SSN_TAG_LEN];
    uint8_t  ds_hdr[SS_AUDIO_HDR];    /* first plaintext bytes: id + audio header */
    uint8_t  ds_out[SS_BLK];

    /* tx: one framed message at a time */
    uint8_t  tx[SS_TX_BUF] __attribute__((aligned(4)));
    uint16_t tx_off, tx_end;
    bool     pong_pending;
    uint8_t  pong_len;
    uint8_t  pong_buf[125];

    /* handshake */
    uint8_t  prologue[SS_PROLOGUE_MAX];
    uint16_t init_len, sinit_len;
    uint8_t  server_pub[32];
    ssn_t    noise;

    /* session */
    bool     need_hello, need_state, need_time, need_goodbye;
    bool     state_sent;
    bool     streaming;
    bool     after_clear;
    uint32_t rate;
    uint8_t  channels, bits;
    uint8_t  rep_vol;
    bool     rep_mute;

    /* time sync */
    bool     burst_active, ct_out, synced;
    uint8_t  burst_idx;
    uint64_t ct_val, last_req_us, next_burst_us;
    int64_t  best_rtt, best_off, offset;
} c;

static void set_phase(uint8_t ph)
{
    c.phase = ph;
    c.phase_start_us = time_us_64();
}

/* ------------------------------------------------------------------ */
/* Weak persistence hooks                                              */
/* ------------------------------------------------------------------ */
__attribute__((weak)) bool sendspin_key_load(uint8_t priv[32]) { (void)priv; return false; }
__attribute__((weak)) void sendspin_key_store(const uint8_t priv[32]) { (void)priv; }

/* ------------------------------------------------------------------ */
/* RNG (pico_rand: TRNG + ROSC + timer on RP2350)                      */
/* ------------------------------------------------------------------ */
static int ss_rng(void *ctx, unsigned char *buf, size_t len)
{
    (void)ctx;
    while (len) {
        uint32_t r = get_rand_32();
        size_t n = len < 4 ? len : 4;
        memcpy(buf, &r, n);
        buf += n; len -= n;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* Tiny tolerant JSON field extractor                                  */
/* ------------------------------------------------------------------ */
/* Find `"key":` at nesting depth `depth` (top-level object keys = 1;
 * 0 = any depth). Returns pointer to the value (after ':' and blanks). */
static const char *json_find(const char *s, const char *key, int depth)
{
    size_t klen = strlen(key);
    int d = 0;
    for (const char *p = s; *p; p++) {
        char ch = *p;
        if (ch == '{' || ch == '[') { d++; continue; }
        if (ch == '}' || ch == ']') { d--; continue; }
        if (ch != '"') continue;
        const char *start = p + 1;
        const char *q = start;
        while (*q && *q != '"') { if (*q == '\\' && q[1]) q++; q++; }
        if (!*q) return NULL;
        const char *after = q + 1;
        while (*after == ' ' || *after == '\t' || *after == '\r' || *after == '\n') after++;
        if (*after == ':' && (depth == 0 || d == depth) &&
            (size_t)(q - start) == klen && memcmp(start, key, klen) == 0) {
            after++;
            while (*after == ' ' || *after == '\t' || *after == '\r' || *after == '\n') after++;
            return after;
        }
        p = q;   /* continue after closing quote */
    }
    return NULL;
}

static int json_str(const char *s, const char *key, int depth, char *out, size_t cap)
{
    const char *v = json_find(s, key, depth);
    if (!v || *v != '"') return -1;
    v++;
    size_t n = 0;
    while (*v && *v != '"') {
        if (*v == '\\' && v[1]) v++;       /* keep escaped char literally */
        if (n + 1 >= cap) return -1;
        out[n++] = *v++;
    }
    if (*v != '"') return -1;
    out[n] = 0;
    return (int)n;
}

static bool json_i64(const char *s, const char *key, int depth, int64_t *out)
{
    const char *v = json_find(s, key, depth);
    if (!v) return false;
    bool neg = false;
    if (*v == '-') { neg = true; v++; }
    if (*v < '0' || *v > '9') return false;
    int64_t r = 0;
    while (*v >= '0' && *v <= '9') { r = r * 10 + (*v - '0'); v++; }
    *out = neg ? -r : r;
    return true;
}

static bool json_bool(const char *s, const char *key, int depth, bool *out)
{
    const char *v = json_find(s, key, depth);
    if (!v) return false;
    if (strncmp(v, "true", 4) == 0) { *out = true; return true; }
    if (strncmp(v, "false", 5) == 0) { *out = false; return true; }
    return false;
}

/* ------------------------------------------------------------------ */
/* WebSocket helpers                                                   */
/* ------------------------------------------------------------------ */
static void ws_accept_key(const char *key, char out[29])
{
    static const char guid[] = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    uint8_t buf[64 + sizeof(guid)], digest[20];
    size_t kl = strlen(key);
    if (kl > 40) kl = 40;
    memcpy(buf, key, kl);
    memcpy(buf + kl, guid, sizeof(guid) - 1);
    mbedtls_sha1(buf, kl + sizeof(guid) - 1, digest);
    ssn_b64_encode(out, digest, 20);
}

static void ws_mask(uint8_t *buf, size_t len, const uint8_t mask[4])
{
    for (size_t i = 0; i < len; i++) buf[i] ^= mask[i & 3];
}

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
        c.tx_off += n;
    }
    tcp_output(c.pcb);
}

/* Payload area for an outgoing WS frame (leaves 8 bytes for the header). */
#define TX_PAYLOAD() (c.tx + 8)

/* Build WS header + mask payload (already at TX_PAYLOAD()), then send.
 * Returns false (nothing sent) if len doesn't fit. */
static bool ws_commit(uint8_t opcode, size_t len)
{
    if (len + 8 > SS_TX_BUF) return false;
    uint8_t mask[4];
    ss_rng(NULL, mask, 4);
    uint8_t hl = (len > 125) ? 8 : 6;
    uint8_t *h = c.tx + (8 - hl);
    h[0] = 0x80 | (opcode & 0x0f);
    if (len > 125) {
        h[1] = 0x80 | 126; h[2] = (uint8_t)(len >> 8); h[3] = (uint8_t)len;
        memcpy(h + 4, mask, 4);
    } else {
        h[1] = 0x80 | (uint8_t)len;
        memcpy(h + 2, mask, 4);
    }
    ws_mask(c.tx + 8, len, mask);
    c.tx_off = (uint16_t)(8 - hl);
    c.tx_end = (uint16_t)(8 + len);
    tx_flush();
    return true;
}

/* Encrypted JSON (message id 0). Caller formats JSON into *out (cap bytes). */
static bool enc_begin(char **out, size_t *cap)
{
    if (tx_busy() || c.phase < PH_WAIT_HELLO) return false;
    *out = (char *)TX_PAYLOAD() + 1;
    *cap = SS_TX_BUF - 8 - 1 - SSN_TAG_LEN;
    return true;
}

static bool enc_end(int n, size_t cap)
{
    if (n <= 0 || (size_t)n >= cap) return false;
    uint8_t *pt = TX_PAYLOAD();
    pt[0] = SS_ID_JSON;
    if (ssn_encrypt(&c.noise, pt, 1 + (size_t)n, pt) != 0) return false;
    return ws_commit(2, 1 + (size_t)n + SSN_TAG_LEN);
}

/* ------------------------------------------------------------------ */
/* Connection teardown / retry                                         */
/* ------------------------------------------------------------------ */
static void rx_reset(void)
{
    c.rx_st = RX_HDR; c.hdr_have = 0; c.hdr_need = 2;
    c.msg_opcode = 0; c.msg_len = 0; c.msg_total = 0; c.msg_discard = false;
    c.blocked = false; c.ds_on = false; c.ds_bad = false;
    c.http_have = 0;
}

static void cleanup(bool graceful)
{
    if (c.hold) { pbuf_free(c.hold); c.hold = NULL; c.hold_seg = NULL; c.hold_off = 0; }
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
    c.pong_pending = false;
    c.need_hello = c.need_state = c.need_time = c.need_goodbye = false;
    c.state_sent = false;
    c.burst_active = c.ct_out = c.synced = false;
    c.streaming = false;
    c.after_clear = false;
    c.fatal = false;
    c.close_graceful = false;
    rx_reset();
    ssn_rx_abort(&c.noise);
    memset(&c.noise, 0, sizeof(c.noise));
    set_phase(PH_NONE);
}

static void conn_fail(const char *why, bool graceful)
{
    (void)why;
    SS_LOG("fail: %s\n", why);
    cleanup(graceful);
    if (!c.enabled) { c.state = SENDSPIN_STATE_IDLE; return; }
    c.state = SENDSPIN_STATE_ERROR;
    c.retry_at_us = time_us_64() + (uint64_t)c.backoff_ms * 1000ull;
    c.backoff_ms *= 2;
    if (c.backoff_ms > SS_BACKOFF_MAX_MS) c.backoff_ms = SS_BACKOFF_MAX_MS;
}

/* ------------------------------------------------------------------ */
/* Outgoing messages                                                   */
/* ------------------------------------------------------------------ */
static bool send_client_init(void)
{
    if (tx_busy()) return false;
    char *p = (char *)TX_PAYLOAD();
    int n = snprintf(p, SS_INIT_MAX,
        "{\"type\":\"client/init\",\"payload\":{\"client_id\":\"%s\",\"version\":1,"
        "\"suite\":\"" SS_SUITE "\"}}", c.client_id);
    if (n <= 0 || n >= SS_INIT_MAX) return false;
    memcpy(c.prologue, p, (size_t)n);          /* exact bytes sent */
    c.init_len = (uint16_t)n;
    return ws_commit(1, (size_t)n);
}

/* bytes of audio the server may have outstanding: exactly the PCM ring */
static uint32_t advertised_capacity(void)
{
    uint32_t cap = net_input_ring_capacity();
#ifdef SENDSPIN_BUFFER_CAPACITY
    if ((uint32_t)SENDSPIN_BUFFER_CAPACITY < cap) cap = SENDSPIN_BUFFER_CAPACITY;
#endif
    return cap;
}

static bool send_hello(void)
{
    char *o; size_t cap;
    if (!enc_begin(&o, &cap)) return false;
    int n = snprintf(o, cap,
        "{\"type\":\"client/hello\",\"payload\":{"
        "\"name\":\"" SENDSPIN_CLIENT_NAME "\","
        "\"device_info\":{\"product_name\":\"DSPi\",\"manufacturer\":\"DSPi\","
        "\"software_version\":\"phase1\"},"
        "\"supported_roles\":[\"player@v1\"],"
        "\"player@v1_support\":{\"supported_formats\":["
        "{\"codec\":\"pcm\",\"channels\":2,\"sample_rate\":48000,\"bit_depth\":16},"
        "{\"codec\":\"pcm\",\"channels\":2,\"sample_rate\":44100,\"bit_depth\":16}],"
        "\"buffer_capacity\":%u},"
        "\"supported_pair_methods\":{},"
        "\"unpaired_access\":{\"enabled\":true}}}",
        (unsigned)advertised_capacity());
    return enc_end(n, cap);
}

static bool send_time_req(void)
{
    char *o; size_t cap;
    if (!enc_begin(&o, &cap)) return false;
    uint64_t ct = time_us_64();
    int n = snprintf(o, cap,
        "{\"type\":\"client/time\",\"payload\":{\"client_transmitted\":%llu}}",
        (unsigned long long)ct);
    if (!enc_end(n, cap)) return false;
    c.ct_val = ct; c.ct_out = true; c.last_req_us = ct;
    return true;
}

static bool send_state(void)
{
    char *o; size_t cap;
    if (!enc_begin(&o, &cap)) return false;
    uint8_t vol = net_volume_get_percent();
    bool mute = net_mute_get();
    if (vol > 100) vol = 100;
    int n = snprintf(o, cap,
        "{\"type\":\"client/state\",\"payload\":{\"available\":true,\"player\":{"
        "\"volume\":%u,\"muted\":%s,\"output_delay_ms\":0,"
        "\"required_lead_time_ms\":%d,\"min_buffer_ms\":%d,"
        "\"supported_commands\":[\"volume\",\"mute\"]}}}",
        (unsigned)vol, mute ? "true" : "false",
        SS_REQUIRED_LEAD_MS, SS_MIN_BUFFER_MS);
    if (!enc_end(n, cap)) return false;
    c.rep_vol = vol; c.rep_mute = mute;
    return true;
}

static bool send_goodbye(const char *reason)
{
    char *o; size_t cap;
    if (!enc_begin(&o, &cap)) return false;
    int n = snprintf(o, cap,
        "{\"type\":\"client/goodbye\",\"payload\":{\"reason\":\"%s\"}}", reason);
    return enc_end(n, cap);
}

static bool send_pong(void)
{
    memcpy(TX_PAYLOAD(), c.pong_buf, c.pong_len);
    return ws_commit(0xA, c.pong_len);
}

/* Push at most one pending message per call, in priority order. */
static void service_tx(void)
{
    if (!c.pcb) return;
    if (tx_busy()) { tx_flush(); if (tx_busy()) return; }
    if (c.phase < PH_HTTP) return;

    if (c.pong_pending) {
        if (send_pong()) c.pong_pending = false;
        return;
    }
    if (c.phase < PH_WAIT_HELLO) return;
    if (c.need_goodbye) { if (send_goodbye("shutdown")) c.need_goodbye = false; return; }
    if (c.need_hello) { if (send_hello()) c.need_hello = false; return; }
    if (c.need_time)  { if (send_time_req()) c.need_time = false; return; }
    if (c.need_state) {
        if (send_state()) {
            c.need_state = false;
            c.state_sent = true;
            if (c.state == SENDSPIN_STATE_SYNCING) {
                c.state = SENDSPIN_STATE_READY;
                c.backoff_ms = SS_BACKOFF_MIN_MS;   /* connection is healthy */
            }
        }
        return;
    }
}

/* ------------------------------------------------------------------ */
/* Incoming JSON (encrypted channel)                                   */
/* ------------------------------------------------------------------ */
static bool format_supported(int64_t rate, int64_t ch, int64_t bits)
{
    /* UNVERIFIED: stream/start `bit_depth` is the PCM sample width */
    return (rate == 44100 || rate == 48000) && ch == 2 && bits == 16;
}

static void on_time_reply(const char *js)
{
    int64_t ct, sr, st;
    if (!json_i64(js, "client_transmitted", 0, &ct) ||
        !json_i64(js, "server_received", 0, &sr) ||
        !json_i64(js, "server_transmitted", 0, &st)) return;
    if (!c.ct_out || (uint64_t)ct != c.ct_val) return;
    c.ct_out = false;
    int64_t cr = (int64_t)c.rx_us;
    int64_t rtt = (cr - ct) - (st - sr);
    int64_t off = ((sr - ct) + (st - cr)) / 2;     /* server = client + off */
    if (rtt >= 0 && rtt < c.best_rtt) { c.best_rtt = rtt; c.best_off = off; }
    if (++c.burst_idx < SS_SYNC_BURST) {
        c.need_time = true;
    } else {
        c.burst_active = false;
        if (c.best_rtt != INT64_MAX) {
            c.offset = c.best_off;       /* phase 1: plain replace, no filter */
            c.synced = true;
            if (!c.state_sent) c.need_state = true;   /* converged -> available */
        }
        c.next_burst_us = time_us_64() + SS_SYNC_INTERVAL_US;
    }
}

static void on_json(char *js)
{
    char type[32];
    if (json_str(js, "type", 1, type, sizeof(type)) < 0) return;

    if (!strcmp(type, "server/hello")) {
        if (c.phase == PH_WAIT_HELLO) {
            set_phase(PH_RUN);
            c.state = SENDSPIN_STATE_SYNCING;
            c.need_hello = true;
        }
    } else if (!strcmp(type, "server/time")) {
        on_time_reply(js);
    } else if (!strcmp(type, "server/activate")) {
        SS_LOG("activate\n");
    } else if (!strcmp(type, "stream/start")) {
        int64_t rate = 0, ch = 0, bits = 16;
        char codec[12] = "";
        json_str(js, "codec", 0, codec, sizeof(codec));
        json_i64(js, "sample_rate", 0, &rate);
        json_i64(js, "channels", 0, &ch);
        json_i64(js, "bit_depth", 0, &bits);
        if (strcmp(codec, "pcm") || !format_supported(rate, ch, bits)) {
            SS_LOG("unsupported stream %s %d/%d/%d\n", codec, (int)rate, (int)ch, (int)bits);
            c.streaming = false;
            return;
        }
        net_input_flush();
        net_input_set_format((uint32_t)rate, (uint8_t)ch, (uint8_t)bits);
        c.rate = (uint32_t)rate; c.channels = (uint8_t)ch; c.bits = (uint8_t)bits;
        c.streaming = true;
        c.after_clear = false;
        if (c.state == SENDSPIN_STATE_READY) c.state = SENDSPIN_STATE_STREAMING;
    } else if (!strcmp(type, "stream/clear")) {
        net_input_flush();
        c.after_clear = true;
    } else if (!strcmp(type, "stream/end")) {
        /* UNVERIFIED: whether stream/end should also drop buffered audio */
        net_input_flush();
        c.streaming = false;
        c.after_clear = false;
        if (c.state == SENDSPIN_STATE_STREAMING) c.state = SENDSPIN_STATE_READY;
    } else if (!strcmp(type, "server/command")) {
        char cmd[20];
        if (json_str(js, "command", 0, cmd, sizeof(cmd)) < 0) return;
        if (!strcmp(cmd, "volume")) {
            int64_t v;
            if (json_i64(js, "volume", 0, &v)) {
                if (v < 0) v = 0;
                if (v > 100) v = 100;
                net_volume_set_percent((uint8_t)v);
                c.need_state = true;
            }
        } else if (!strcmp(cmd, "mute")) {
            bool m;
            if (json_bool(js, "mute", 0, &m)) {
                net_mute_set(m);
                c.need_state = true;
            }
        }
    } else {
        SS_LOG("ignored json type %s\n", type);
    }
}

/* ------------------------------------------------------------------ */
/* Encrypted records: streaming decrypt                                */
/* ------------------------------------------------------------------ */
/* Invariants (see the file header):
 *  - one record == one unfragmented, unmasked WS binary frame; ds_len bytes of
 *    ciphertext followed by the 16-byte tag; the AEAD stream is started in
 *    parse_header() and finished in frame_done();
 *  - plaintext of an audio record goes to ring bytes [wr, wr + ds_len - 13)
 *    through net_input_stage_pcm(); nothing is visible to the consumer until
 *    ds_finish() has verified the tag and calls net_input_commit_pcm();
 *  - plaintext of other records goes to g_rx[0 .. ds_len) (+ NUL);
 *  - the first SS_AUDIO_HDR plaintext bytes are always kept in ds_hdr so the
 *    route can be chosen from the id byte and the audio header parsed later;
 *  - blocks never straddle the header boundary, so a ring-bound block never
 *    contains header bytes once the route is known. */
static uint32_t ds_staged(void)      /* PCM bytes already staged in the ring */
{
    return c.ds_got > SS_AUDIO_HDR ? c.ds_got - SS_AUDIO_HDR : 0;
}

static uint32_t ds_ring_room(void)   /* free ring bytes beyond what is staged */
{
    uint32_t free_b = net_input_ring_free_bytes(), st = ds_staged();
    return free_b > st ? free_b - st : 0;
}

/* Is there enough ring room to decrypt the next block? */
static bool ds_can_step(void)
{
    if (c.ds_route != DS_RING || c.ds_got < SS_AUDIO_HDR || c.ds_got >= c.ds_len) return true;
    uint32_t want = c.ds_len - c.ds_got;
    if (want > SS_BLK) want = SS_BLK;
    return ds_ring_room() >= want;
}

static void ds_begin(void)
{
    c.ds_on = true; c.ds_bad = false; c.ds_route = DS_SINK;
    c.ds_len = (uint32_t)c.plen - SSN_TAG_LEN;
    c.ds_got = 0; c.ds_tag_have = 0;
    if (ssn_rx_begin(&c.noise) != 0) c.fatal = true;
}

/* Choose the destination from the (just decrypted) first plaintext byte. */
static void ds_route_select(uint8_t id)
{
    if (id == SS_ID_PLAYER_AUDIO) {
        if (c.ds_len < SS_AUDIO_HDR) { c.ds_bad = true; c.ds_route = DS_SINK; }       /* protocol error */
        else if (c.streaming && c.ds_len - SS_AUDIO_HDR <= net_input_ring_capacity()) c.ds_route = DS_RING;
        else c.ds_route = DS_SINK;      /* not streaming, or can never fit the ring: authenticate and drop */
    } else {
        c.ds_route = (c.ds_len + 1u <= SS_RX_BUF) ? DS_BUF : DS_SINK;
    }
}

/* Place n freshly decrypted plaintext bytes (they start at offset c.ds_got). */
static void ds_store(const uint8_t *pt, uint32_t n)
{
    uint32_t off = c.ds_got;
    for (uint32_t i = 0; i < n && off + i < SS_AUDIO_HDR; i++) c.ds_hdr[off + i] = pt[i];
    if (off == 0) ds_route_select(pt[0]);
    if (c.ds_route == DS_BUF) {
        memcpy(g_rx + off, pt, n);
    } else if (c.ds_route == DS_RING) {
        uint32_t skip = off < SS_AUDIO_HDR ? SS_AUDIO_HDR - off : 0;    /* header bytes in this block */
        if (skip < n && !net_input_stage_pcm(off + skip - SS_AUDIO_HDR, pt + skip, n - skip))
            c.fatal = true;             /* cannot happen: room was checked before decrypting */
    }
}

/* Feed ciphertext of the current record.  Returns the bytes consumed; fewer
 * than `take` only when the ring has no room (c.blocked is set then). */
static size_t ds_consume(const uint8_t *d, size_t take)
{
    size_t used = 0;
    while (take && !c.fatal) {
        if (c.ds_got < c.ds_len) {
            uint32_t n = c.ds_len - c.ds_got;
            if (n > take) n = (uint32_t)take;
            if (n > SS_BLK) n = SS_BLK;
            if (c.ds_got < SS_AUDIO_HDR && n > SS_AUDIO_HDR - c.ds_got) n = SS_AUDIO_HDR - c.ds_got;
            if (c.ds_route == DS_RING && c.ds_got >= SS_AUDIO_HDR) {
                if (!ds_can_step()) {
                    if (!c.blocked) { c.blocked = true; c.blocked_since_us = time_us_64(); }
                    break;
                }
                uint32_t room = ds_ring_room();
                if (n > room) n = room;
            }
            if (ssn_rx_update(&c.noise, d, n, c.ds_out) != 0) { c.fatal = true; break; }
            ds_store(c.ds_out, n);
            c.ds_got += n;
            d += n; used += n; take -= n;
        } else {
            uint32_t n = SSN_TAG_LEN - c.ds_tag_have;
            if (n > take) n = (uint32_t)take;
            memcpy(c.ds_tag + c.ds_tag_have, d, n);
            c.ds_tag_have = (uint8_t)(c.ds_tag_have + n);
            d += n; used += n; take -= n;
        }
    }
    return used;
}

/* Audio record authenticated: parse the header and publish the staged PCM. */
static void ds_audio_commit(void)
{
    int64_t ts = 0;
    for (int i = 0; i < 8; i++) ts = (int64_t)(((uint64_t)ts << 8) | c.ds_hdr[1 + i]);
    /* send_ahead (ds_hdr[9..12]) is informational only (phase 1) */
    uint32_t n = c.ds_len - SS_AUDIO_HDR;
    uint32_t fb = (uint32_t)c.channels * (c.bits / 8u);
    if (fb) n -= n % fb;                               /* whole frames only */
    if (!n) return;

    if (c.after_clear && c.synced) {
        /* drop obviously late chunks after a stream/clear (nothing was published) */
        int64_t dur = (int64_t)((uint64_t)n * 1000000ull / ((uint64_t)c.rate * fb));
        int64_t server_now = (int64_t)time_us_64() + c.offset;
        if (ts + dur < server_now - SS_LATE_SLACK_US) { SS_LOG("late chunk dropped\n"); return; }
    }
    c.after_clear = false;
    net_input_commit_pcm(n);
}

/* End of an encrypted record (called from frame_done). */
static void ds_finish(void)
{
    c.ds_on = false;
    if (c.ds_got != c.ds_len || c.ds_tag_have != SSN_TAG_LEN) { ssn_rx_abort(&c.noise); c.fatal = true; return; }
    if (ssn_rx_finish(&c.noise, c.ds_tag) != 0) { c.fatal = true; return; }   /* auth failure */
    if (c.ds_len == 0 || c.ds_bad) { c.fatal = true; return; }               /* empty / short audio record */
    switch (c.ds_hdr[0]) {
    case SS_ID_JSON:
        if (c.ds_route == DS_BUF) {
            g_rx[c.ds_len] = 0;                         /* room guaranteed by ds_route_select() */
            on_json((char *)g_rx + 1);
        } else {
            SS_LOG("oversize json record skipped (%u bytes)\n", (unsigned)c.ds_len);
        }
        break;
    case SS_ID_PLAYER_AUDIO:
        if (c.ds_route == DS_RING) ds_audio_commit();
        break;
    case SS_ID_FRAGMENT:
        /* UNVERIFIED: fragmentation (id 1) is only used for >65 KB messages;
         * not reassembled here, silently ignored. */
        break;
    default:
        break;      /* unknown/reserved ids ignored (also: re-handshake, UNVERIFIED) */
    }
}

/* ------------------------------------------------------------------ */
/* Cleartext handshake messages (WS text)                              */
/* ------------------------------------------------------------------ */
static void on_text(char *js)
{
    char type[32];
    if (json_str(js, "type", 1, type, sizeof(type)) < 0) { c.fatal = true; return; }

    if (c.phase == PH_WAIT_SERVER_INIT) {
        if (strcmp(type, "server/init")) { c.fatal = true; return; }
        size_t n = c.msg_len;
        if (c.init_len + n > SS_PROLOGUE_MAX) { c.fatal = true; return; }
        memcpy(c.prologue + c.init_len, js, n);         /* exact wire bytes */
        c.sinit_len = (uint16_t)n;
        char sid[64];
        int sl = json_str(js, "server_id", 0, sid, sizeof(sid));
        if (sl < 0 || ssn_b64url_decode(c.server_pub, 32, sid, (size_t)sl) != 32) {
            c.fatal = true; return;
        }
        if (ssn_resp_init(&c.noise, c.s_priv, c.s_pub, c.server_pub,
                          c.prologue, (size_t)c.init_len + n, ss_rng, NULL) != 0) {
            c.fatal = true; return;
        }
        set_phase(PH_WAIT_NOISE1);
    } else if (c.phase == PH_WAIT_NOISE1) {
        if (strcmp(type, "noise/handshake")) { c.fatal = true; return; }
        /* Scratch lives in c.prologue: its content (client/init || server/init) was
         * consumed by ssn_resp_init() when this phase was entered and is never
         * read again, so the 640 bytes are free until the next connection. */
        _Static_assert(SS_PROLOGUE_MAX >= 200 + 128 + 96, "handshake scratch");
        uint8_t *msg = c.prologue, *payload = c.prologue + 200, *out = c.prologue + 328;
        const char *dv = json_find(js, "data", 0);
        if (!dv || *dv != '"') { c.fatal = true; return; }
        dv++;
        size_t dl = 0;
        while (dv[dl] && dv[dl] != '"') dl++;
        if (dv[dl] != '"') { c.fatal = true; return; }
        int ml = ssn_b64url_decode(msg, 200, dv, dl);
        if (ml < 0) { c.fatal = true; return; }
        size_t plen = 0;
        if (ssn_resp_read_msg1(&c.noise, msg, (size_t)ml, payload, 127, &plen) != 0) {
            c.fatal = true; return;
        }
        payload[plen] = 0;
        char cat[8];
        if (json_str((char *)payload, "psk_category", 0, cat, sizeof(cat)) < 0 ||
            strcmp(cat, SS_SENTINEL_CAT) != 0) {
            /* paired modes (lt/pr) need pairing.md support; not implemented */
            SS_LOG("psk category '%s' unsupported\n", cat);
            c.fatal = true; return;
        }
        uint8_t psk[32];
        ssn_sentinel_psk(psk);
        size_t ol = 0;
        if (ssn_resp_write_msg2(&c.noise, psk, (const uint8_t *)"{}", 2, out, 96, &ol) != 0) {
            c.fatal = true; return;
        }
        if (tx_busy()) { c.fatal = true; return; }
        char *p = (char *)TX_PAYLOAD();
        int n = snprintf(p, SS_TX_BUF - 8,
            "{\"type\":\"noise/handshake\",\"payload\":{\"data\":\"");
        n += (int)ssn_b64url_encode(p + n, out, ol);            /* 96 bytes -> <= 128 chars */
        memcpy(p + n, "\"}}", 3); n += 3;
        if (!ws_commit(1, (size_t)n)) { c.fatal = true; return; }
        set_phase(PH_WAIT_HELLO);       /* transport keys are live now */
    } else {
        /* text frames after the channel is up are not expected */
        c.fatal = true;
    }
}

/* ------------------------------------------------------------------ */
/* WebSocket frame parser                                              */
/* ------------------------------------------------------------------ */
static void dispatch_message(void)
{
    /* text only: encrypted binary records finish in ds_finish() */
    if (c.msg_discard || c.phase >= PH_WAIT_HELLO) { c.fatal = true; return; }
    g_rx[c.msg_len] = 0;
    on_text((char *)g_rx);
}

static void frame_done(void)
{
    if (c.opcode >= 8) {
        if (c.opcode == 8) {
            SS_LOG("close frame\n");
            /* reply close (best effort) then close gracefully */
            if (c.pcb && !tx_busy()) {
                c.tx[8] = 0x03; c.tx[9] = 0xE8;           /* 1000 */
                ws_commit(8, 2);
            }
            c.close_graceful = true;
            c.fatal = true;
            return;
        } else if (c.opcode == 9) {
            c.pong_len = (uint8_t)c.plen;
            memcpy(c.pong_buf, c.ctl, c.pong_len);
            c.pong_pending = true;
        } else if (c.opcode != 10) {
            c.fatal = true;          /* reserved control opcode */
            return;
        }
    } else if (c.fin) {
        if (c.msg_opcode == 2) ds_finish(); else dispatch_message();
        c.msg_opcode = 0;
        c.msg_len = 0; c.msg_total = 0; c.msg_discard = false;
    }
    c.rx_st = RX_HDR; c.hdr_have = 0; c.hdr_need = 2;
}

static void parse_header(void)
{
    uint8_t b0 = c.hdr[0], b1 = c.hdr[1];
    c.fin = (b0 & 0x80) != 0;
    c.opcode = b0 & 0x0f;
    c.masked = (b1 & 0x80) != 0;
    uint8_t l7 = b1 & 0x7f;
    size_t o = 2;
    if (l7 == 126) { c.plen = ((uint64_t)c.hdr[2] << 8) | c.hdr[3]; o = 4; }
    else if (l7 == 127) {
        c.plen = 0;
        for (int i = 0; i < 8; i++) c.plen = (c.plen << 8) | c.hdr[2 + i];
        o = 10;
    } else c.plen = l7;
    if (c.masked) memcpy(c.mask, c.hdr + o, 4);
    c.pgot = 0;

    if (b0 & 0x70) { c.fatal = true; return; }            /* RSV bits */
    if (c.plen >> 31) { c.fatal = true; return; }
    if (c.opcode >= 8) {
        if (!c.fin || c.plen > 125) { c.fatal = true; return; }
    } else if (c.opcode == 0) {
        if (!c.msg_opcode) { c.fatal = true; return; }
    } else if (c.opcode == 1 || c.opcode == 2) {
        if (c.msg_opcode) { c.fatal = true; return; }
        c.msg_opcode = c.opcode;
        c.msg_len = 0; c.msg_total = 0; c.msg_discard = false;
        if (c.opcode == 2) {
            /* Encrypted record: must be a single unmasked frame (so its length,
             * and hence where the tag starts, is known up front) holding at
             * least the tag. */
            if (c.phase < PH_WAIT_HELLO || !c.fin || c.masked || c.plen < SSN_TAG_LEN) {
                c.fatal = true; return;
            }
            ds_begin();
            if (c.fatal) return;
        }
    } else {
        c.fatal = true; return;
    }
    c.rx_st = RX_PAYLOAD;
    if (c.plen == 0) frame_done();
}

/* Returns the bytes consumed (< take only for a ring-blocked audio record). */
static size_t consume_payload(const uint8_t *d, size_t take)
{
    if (c.opcode >= 8) {
        for (size_t i = 0; i < take; i++)
            c.ctl[c.pgot + i] = c.masked ? d[i] ^ c.mask[(c.pgot + i) & 3] : d[i];
        return take;
    }
    if (c.msg_opcode == 2) return ds_consume(d, take);
    c.msg_total += take;
    if (c.msg_discard) return take;
    /* text messages need 1 spare byte for NUL */
    if (c.msg_len + take + 1 > SS_RX_BUF) { c.msg_discard = true; return take; }
    uint8_t *dst = g_rx + c.msg_len;
    if (c.masked) {
        for (size_t i = 0; i < take; i++) dst[i] = d[i] ^ c.mask[(c.pgot + i) & 3];
    } else {
        memcpy(dst, d, take);
    }
    c.msg_len += (uint32_t)take;
    return take;
}

static bool resume_audio(void)
{
    if (!c.blocked) return true;
    if (!ds_can_step()) return false;
    c.blocked = false;
    return true;
}

static size_t ws_feed(const uint8_t *d, size_t n)
{
    size_t i = 0;
    if (!resume_audio()) return 0;
    while (i < n && !c.fatal && !c.blocked) {
        if (c.rx_st == RX_HDR) {
            c.hdr[c.hdr_have++] = d[i++];
            if (c.hdr_have == 2) {
                uint8_t l7 = c.hdr[1] & 0x7f;
                c.hdr_need = (uint8_t)(2 + (l7 == 126 ? 2 : l7 == 127 ? 8 : 0) +
                                       ((c.hdr[1] & 0x80) ? 4 : 0));
            }
            if (c.hdr_have >= 2 && c.hdr_have == c.hdr_need) parse_header();
        } else {
            size_t take = n - i;
            uint64_t left = c.plen - c.pgot;
            if (take > left) take = (size_t)left;
            size_t got = consume_payload(d + i, take);
            i += got; c.pgot += got;
            if (got < take) break;                       /* ring full: c.blocked is set */
            if (c.pgot == c.plen) frame_done();
        }
    }
    return i;
}

/* ---- HTTP upgrade response ---- */
static bool http_header_ok(void)
{
    /* g_rx holds the response header, NUL-terminated at http_have */
    char *s = (char *)g_rx;
    if (strncmp(s, "HTTP/1.1 101", 12) != 0) return false;
    for (char *p = strstr(s, "\r\n"); p && *p; p = strstr(p + 2, "\r\n")) {
        char *l = p + 2;
        if (strncasecmp(l, "sec-websocket-accept:", 21) == 0) {
            l += 21;
            while (*l == ' ') l++;
            size_t al = strlen(c.ws_accept);
            return strncmp(l, c.ws_accept, al) == 0 &&
                   (l[al] == '\r' || l[al] == '\n' || l[al] == ' ' || l[al] == 0);
        }
    }
    return false;
}

static size_t http_feed(const uint8_t *d, size_t n)
{
    size_t i = 0;
    while (i < n) {
        if (c.http_have + 1 >= SS_HTTP_MAX) { c.fatal = true; return n; }
        g_rx[c.http_have++] = d[i++];
        if (c.http_have >= 4 && !memcmp(g_rx + c.http_have - 4, "\r\n\r\n", 4)) {
            g_rx[c.http_have] = 0;
            if (!http_header_ok()) { c.fatal = true; return i; }
            c.http_have = 0;
            rx_reset();
            set_phase(PH_WAIT_SERVER_INIT);
            c.state = SENDSPIN_STATE_HANDSHAKE;
            if (!send_client_init()) c.fatal = true;
            return i;                       /* rest (if any) goes to ws_feed */
        }
    }
    return i;
}

static size_t feed(const uint8_t *d, size_t n)
{
    if (c.phase == PH_HTTP) return http_feed(d, n);
    if (c.phase < PH_HTTP) return n;        /* stray data: drop */
    return ws_feed(d, n);
}

/* ------------------------------------------------------------------ */
/* lwIP callbacks                                                      */
/* ------------------------------------------------------------------ */
static void rx_drain(void)
{
    while (c.hold && !c.fatal) {
        struct pbuf *seg = c.hold_seg;
        if (!seg) { pbuf_free(c.hold); c.hold = NULL; break; }
        size_t avail = seg->len - c.hold_off;
        size_t used = avail ? feed((const uint8_t *)seg->payload + c.hold_off, avail) : 0;
        c.hold_off = (uint16_t)(c.hold_off + used);
        if (used && c.pcb) tcp_recved(c.pcb, (u16_t)used);
        if (c.fatal) break;
        if (c.hold_off >= seg->len) { c.hold_seg = seg->next; c.hold_off = 0; }
        else if (c.blocked) break;
        else if (!used) break;               /* no progress: avoid spinning */
    }
    if (c.fatal) {
        conn_fail("protocol/close", c.close_graceful);
        return;
    }
    service_tx();
}

static err_t on_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
    (void)arg;
    c.abort_flag = false;
    if (!p) { conn_fail("peer closed", true); return c.abort_flag ? ERR_ABRT : ERR_OK; }
    if (err != ERR_OK || pcb != c.pcb) { pbuf_free(p); return ERR_OK; }
    c.rx_us = time_us_64();
    if (c.hold) {
        pbuf_cat(c.hold, p);                 /* queue behind blocked data */
    } else {
        c.hold = p; c.hold_seg = p; c.hold_off = 0;
    }
    rx_drain();
    return c.abort_flag ? ERR_ABRT : ERR_OK;
}

static err_t on_sent(void *arg, struct tcp_pcb *pcb, u16_t len)
{
    (void)arg; (void)pcb; (void)len;
    tx_flush();
    return ERR_OK;
}

static void on_err(void *arg, err_t err)
{
    (void)arg; (void)err;
    c.pcb = NULL;                            /* already freed by lwIP */
    conn_fail("tcp error", false);
}

static err_t on_connected(void *arg, struct tcp_pcb *pcb, err_t err)
{
    (void)arg;
    if (err != ERR_OK || pcb != c.pcb) return ERR_OK;
    uint8_t rnd[16];
    ss_rng(NULL, rnd, 16);
    ssn_b64_encode(c.ws_key, rnd, 16);
    ws_accept_key(c.ws_key, c.ws_accept);
    int n = snprintf((char *)c.tx, SS_TX_BUF,
        "GET " SS_WS_PATH " HTTP/1.1\r\n"
        "Host: %s:%u\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Key: %s\r\n"
        "Sec-WebSocket-Version: 13\r\n\r\n",
        net_sendspin_host(), (unsigned)net_sendspin_port(), c.ws_key);
    if (n <= 0 || n >= SS_TX_BUF) { conn_fail("request too long", false); return c.abort_flag ? ERR_ABRT : ERR_OK; }
    c.tx_off = 0; c.tx_end = (uint16_t)n;
    set_phase(PH_HTTP);
    rx_reset();
    tx_flush();
    return ERR_OK;
}

static void do_connect(const ip_addr_t *ip)
{
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb) { conn_fail("tcp_new", false); return; }
    c.pcb = pcb;
    tcp_arg(pcb, NULL);
    tcp_recv(pcb, on_recv);
    tcp_sent(pcb, on_sent);
    tcp_err(pcb, on_err);
    tcp_nagle_disable(pcb);
    set_phase(PH_TCP);
    if (tcp_connect(pcb, ip, net_sendspin_port(), on_connected) != ERR_OK) {
        conn_fail("tcp_connect", false);
    }
}

static void dns_found(const char *name, const ip_addr_t *ip, void *arg)
{
    (void)name; (void)arg;
    if (!c.dns_pending) return;
    c.dns_pending = false;
    if (!ip) { conn_fail("dns", false); return; }
    do_connect(ip);
}

static void begin_connect(void)
{
    const char *host = net_sendspin_host();
    if (!host || !*host || !net_sendspin_port()) {
        conn_fail("no host configured", false);
        return;
    }
    c.state = SENDSPIN_STATE_CONNECTING;
    rx_reset();
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
void sendspin_client_start(void)
{
    if (c.enabled) return;
    if (!c.key_ok) {
        if (!sendspin_key_load(c.s_priv)) {
            /* Phase 1: fresh X25519 identity each boot. */
            if (ssn_gen_static(c.s_priv, c.s_pub, ss_rng, NULL) != 0) return;
            sendspin_key_store(c.s_priv);
        } else if (ssn_x25519_base(c.s_pub, c.s_priv, ss_rng, NULL) != 0) {
            return;
        }
        ssn_b64url_encode(c.client_id, c.s_pub, 32);      /* 43 chars */
        c.key_ok = true;
    }
    c.enabled = true;
    c.backoff_ms = SS_BACKOFF_MIN_MS;
    c.retry_at_us = 0;
    c.state = SENDSPIN_STATE_IDLE;
}

void sendspin_client_stop(void)
{
    if (!c.enabled) return;
    if (c.pcb && c.phase >= PH_RUN) {
        c.need_goodbye = true;
        service_tx();
    }
    c.enabled = false;
    cleanup(true);
    c.state = SENDSPIN_STATE_IDLE;
}

bool sendspin_client_streaming(void) { return c.streaming; }
uint8_t sendspin_client_state(void) { return c.state; }

void sendspin_client_poll(void)
{
    if (!c.enabled) return;
    uint64_t now = time_us_64();

    if (!c.pcb && !c.dns_pending && c.phase == PH_NONE &&
        (c.state == SENDSPIN_STATE_IDLE || c.state == SENDSPIN_STATE_ERROR) &&
        now >= c.retry_at_us) {
        begin_connect();
        return;
    }
    if (c.phase == PH_NONE) return;

    /* handshake-phase watchdog */
    if (c.state != SENDSPIN_STATE_READY && c.state != SENDSPIN_STATE_STREAMING &&
        now - c.phase_start_us > SS_PHASE_TIMEOUT_US) {
        conn_fail("timeout", false);
        return;
    }

    if (c.fatal) { conn_fail("fatal", false); return; }

    if (c.blocked && now - c.blocked_since_us > SS_STALL_TIMEOUT_US) {
        conn_fail("ring stalled", false);    /* consumer not draining: resync via reconnect */
        return;
    }
    if (c.hold) rx_drain();                  /* retry blocked audio */
    if (!c.pcb) return;

    if (c.phase >= PH_RUN) {
        if (c.burst_active && c.ct_out && now - c.last_req_us > SS_SYNC_TIMEOUT_US) {
            c.ct_out = false; c.burst_idx = 0;
            c.best_rtt = INT64_MAX;
            c.need_time = true;
        }
        if (!c.burst_active && now >= c.next_burst_us) {
            c.burst_active = true; c.burst_idx = 0; c.ct_out = false;
            c.best_rtt = INT64_MAX;
            c.need_time = true;
        }
        if (c.state_sent && !c.need_state &&
            (net_volume_get_percent() != c.rep_vol || net_mute_get() != c.rep_mute)) {
            c.need_state = true;             /* local volume/mute change */
        }
    }
    service_tx();
    if (c.fatal) conn_fail("fatal", false);
}
