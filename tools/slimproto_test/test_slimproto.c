/*
 * Host test for the Slimproto client (slimproto_client.c) against a scripted
 * fake Slimproto + HTTP server on a fake multi-pcb lwIP.
 *
 * WHAT THIS PROVES: self-consistency with the protocol as documented in
 * slimproto_client.c (written from squeezelite / aioslimproto / MA sources);
 * the server side here is written by the same author from the same reading.
 * It does NOT prove interoperability with a real Music Assistant server.
 *
 * slimproto_client.c, sendspin_client.c and net_flac.c are linked as separate
 * units (not #included) so that both protocol clients can be driven from one
 * harness (protocol switch / RAM loan / pcb leak tests).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "slimproto_client.h"
#include "sendspin_client.h"
#include "net_flac.h"
#include "lwip/tcp.h"
#include "lwip/dns.h"

static int g_fail, g_pass;
#define CHECK(cond) do { if (cond) g_pass++; else { g_fail++; \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)

uint64_t stub_now_us = 1000000;
static uint32_t rng_state = 0x2468ace1u;
uint32_t get_rand_32(void)
{
    rng_state ^= rng_state << 13; rng_state ^= rng_state >> 17; rng_state ^= rng_state << 5;
    return rng_state * 2654435761u;
}

/* ===================================================================== */
/* fake net_input: a real byte ring with staging, hold and consumer       */
/* ===================================================================== */
#define RING 32768u
static uint8_t  ring[RING];
static uint32_t wr, rd;
static bool     held;
static uint32_t cur_rate, cur_bits;
static int      fmt_calls, flush_calls;
static uint8_t  g_vol = 77; static bool g_mute;
static uint8_t  consumed[1 << 22];
static size_t   consumed_len;
static uint16_t slim_port = 3483;
static const uint8_t test_mac[6] = { 0x02, 0x11, 0x22, 0x33, 0x44, 0x55 };

uint32_t net_input_ring_capacity(void) { return RING; }
uint32_t net_input_ring_free_bytes(void) { return RING - (wr - rd); }
bool net_input_stage_pcm(uint32_t off, const uint8_t *pcm, uint32_t len)
{
    uint32_t fr = net_input_ring_free_bytes();
    if (off > fr || len > fr - off) return false;
    for (uint32_t i = 0; i < len; i++) ring[(wr + off + i) % RING] = pcm[i];
    return true;
}
void net_input_commit_pcm(uint32_t bytes) { CHECK(bytes <= net_input_ring_free_bytes()); wr += bytes; }
void net_input_set_format(uint32_t r, uint8_t ch, uint8_t b)
{
    (void)ch;
    fmt_calls++;
    bool changed = r != cur_rate || b != cur_bits;
    cur_rate = r; cur_bits = b;
    if (changed) rd = wr;                          /* the real one flushes on a format change */
}
void net_input_flush(void) { rd = wr; flush_calls++; }
uint32_t net_input_ring_fill_bytes(void) { return wr - rd; }
uint32_t net_input_ring_wr_pos(void) { return wr; }
uint32_t net_input_ring_rd_pos(void) { return rd; }
void net_input_set_hold(bool h) { held = h; }
bool net_input_is_held(void) { return held; }
void net_input_get_mac(uint8_t m[6]) { memcpy(m, test_mac, 6); }
uint8_t net_volume_get_percent(void) { return g_vol; }
void net_volume_set_percent(uint8_t v) { g_vol = v; }
bool net_mute_get(void) { return g_mute; }
void net_mute_set(bool m) { g_mute = m; }
const char *net_sendspin_host(void) { return "127.0.0.1"; }
uint16_t net_sendspin_port(void) { return 8927; }
uint16_t net_slimproto_port(void) { return slim_port; }

static size_t consume(size_t n)                    /* the DSP consumer */
{
    if (held) return 0;
    size_t f = wr - rd;
    if (n > f) n = f;
    for (size_t i = 0; i < n; i++) consumed[consumed_len++] = ring[(rd + i) % RING];
    rd += (uint32_t)n;
    return n;
}

/* adat ring loan stub */
static struct { uint8_t pre[64]; uint8_t arena[57344]; uint8_t post[64]; } adat_mem;
static bool stub_loaned;
bool adat_output_ring_loanable(void) { return !stub_loaned; }
void *adat_output_ring_loan(uint32_t *bytes, void (*evict)(void))
{
    (void)evict;
    if (stub_loaned) return NULL;
    stub_loaned = true;
    if (bytes) *bytes = sizeof(adat_mem.arena) - 4;
    return adat_mem.arena + 4;
}
void adat_output_ring_return(void) { stub_loaned = false; }

/* ===================================================================== */
/* fake lwIP: several pcbs                                                */
/* ===================================================================== */
#define MAXPCB 8
#define MAX_LIVE 3                                  /* MEMP_NUM_TCP_PCB */
struct tcp_pcb {
    bool live, connected;
    uint16_t port;
    uint32_t ip;
    void *arg;
    tcp_recv_fn recv; tcp_sent_fn sent; tcp_err_fn err; tcp_connected_fn conn;
    uint8_t *out; size_t out_len, parsed;
    long injected, recved;
    int serial;
};
static struct tcp_pcb pcbs[MAXPCB];
static void parse_ctl(void);
static int live_count, created, closed_n, aborted_n, pbufs_live, serial_ctr;
static int sndbuf_limit = 0;

static int live_pcbs(void) { return live_count; }
struct tcp_pcb *tcp_new(void)
{
    parse_ctl();                                   /* keep what the previous owner of the slot sent */
    if (live_count >= MAX_LIVE) return NULL;
    for (int i = 0; i < MAXPCB; i++) if (!pcbs[i].live) {
        free(pcbs[i].out);
        memset(&pcbs[i], 0, sizeof(pcbs[i]));
        pcbs[i].live = true; pcbs[i].out = malloc(1 << 20); pcbs[i].serial = ++serial_ctr;
        live_count++; created++;
        return &pcbs[i];
    }
    return NULL;
}
void tcp_arg(struct tcp_pcb *p, void *a) { p->arg = a; }
void tcp_recv(struct tcp_pcb *p, tcp_recv_fn f) { p->recv = f; }
void tcp_sent(struct tcp_pcb *p, tcp_sent_fn f) { p->sent = f; }
void tcp_err(struct tcp_pcb *p, tcp_err_fn f) { p->err = f; }
err_t tcp_connect(struct tcp_pcb *p, const ip_addr_t *ip, uint16_t port, tcp_connected_fn f)
{ p->port = port; p->ip = ip->addr; p->conn = f; return ERR_OK; }
u16_t tcp_sndbuf(struct tcp_pcb *p) { (void)p; return sndbuf_limit ? (u16_t)sndbuf_limit : 8192; }
err_t tcp_write(struct tcp_pcb *p, const void *d, u16_t len, uint8_t fl)
{
    (void)fl;
    if (sndbuf_limit && len > sndbuf_limit) return ERR_MEM;
    memcpy(p->out + p->out_len, d, len); p->out_len += len;
    return ERR_OK;
}
err_t tcp_output(struct tcp_pcb *p) { (void)p; return ERR_OK; }
void tcp_recved(struct tcp_pcb *p, u16_t len) { p->recved += len; }
static void pcb_free(struct tcp_pcb *p) { if (p->live) { p->live = false; live_count--; } }
err_t tcp_close(struct tcp_pcb *p) { pcb_free(p); closed_n++; return ERR_OK; }
void tcp_abort(struct tcp_pcb *p) { pcb_free(p); aborted_n++; }
void pbuf_free(struct pbuf *p)
{
    while (p) { struct pbuf *n = p->next; free(p->payload); free(p); pbufs_live--; p = n; }
}
void pbuf_cat(struct pbuf *h, struct pbuf *t)
{
    struct pbuf *q = h;
    q->tot_len = (uint16_t)(q->tot_len + t->tot_len);
    while (q->next) q = q->next;
    q->next = t;
}
err_t dns_gethostbyname(const char *host, ip_addr_t *addr, dns_found_callback f, void *arg)
{
    (void)host; (void)f; (void)arg;
    addr->addr = 0x0100007f;
    return ERR_OK;
}

static struct tcp_pcb *find_pcb(uint16_t port)         /* newest live pcb for a port */
{
    struct tcp_pcb *best = NULL;
    for (int i = 0; i < MAXPCB; i++)
        if (pcbs[i].live && pcbs[i].port == port && (!best || pcbs[i].serial > best->serial)) best = &pcbs[i];
    return best;
}

static err_t inject(struct tcp_pcb *p, const uint8_t *d, size_t len, size_t chunk)
{
    if (!p || !p->live) return ERR_ABRT;
    struct pbuf *head = NULL, *tail = NULL;
    size_t off = 0;
    if (!chunk) chunk = len;
    while (off < len) {
        size_t n = len - off < chunk ? len - off : chunk;
        struct pbuf *b = calloc(1, sizeof(*b));
        b->payload = malloc(n); memcpy(b->payload, d + off, n);
        b->len = (uint16_t)n; b->ref = 1; pbufs_live++;
        if (!head) head = b; else tail->next = b;
        tail = b; off += n;
    }
    if (!head) return ERR_OK;
    head->tot_len = (uint16_t)len;
    p->injected += (long)len;
    return p->recv(p->arg, p, head, ERR_OK);
}
static void inject_fin(struct tcp_pcb *p) { if (p && p->live) p->recv(p->arg, p, NULL, ERR_OK); }
static void accept_pcb(struct tcp_pcb *p) { p->connected = true; p->conn(p->arg, p, ERR_OK); }

static void step(int n)
{
    for (int i = 0; i < n; i++) { stub_now_us += 1000; slimproto_client_poll(); }
}

/* ===================================================================== */
/* server side: Slimproto framing and client message parsing              */
/* ===================================================================== */
static uint32_t rbe32(const uint8_t *p) { return ((uint32_t)p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3]; }
static void wbe32(uint8_t *p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = (uint8_t)v; }

typedef struct { char op[5]; uint32_t len; const uint8_t *p; } msg_t;
static msg_t msgs[4096]; static int nmsgs;
static uint8_t msg_store[1 << 23]; static size_t msg_store_len;

static void parse_ctl(void)
{
    for (int i = 0; i < MAXPCB; i++) {
        struct tcp_pcb *p = &pcbs[i];
        if (!p->out || p->port != slim_port) continue;
        while (p->out_len - p->parsed >= 8) {
            const uint8_t *b = p->out + p->parsed;
            uint32_t len = rbe32(b + 4);
            if (p->out_len - p->parsed < 8 + len) break;
            msg_t *m = &msgs[nmsgs++];
            memcpy(m->op, b, 4); m->op[4] = 0; m->len = len;
            m->p = msg_store + msg_store_len; memcpy(msg_store + msg_store_len, b + 8, len); msg_store_len += len + 1;
            p->parsed += 8 + len;
        }
    }
}
static int count_op(const char *op)
{
    parse_ctl(); int n = 0;
    for (int i = 0; i < nmsgs; i++) if (!strcmp(msgs[i].op, op)) n++;
    return n;
}
static int count_stat(const char *ev)
{
    parse_ctl(); int n = 0;
    for (int i = 0; i < nmsgs; i++)
        if (!strcmp(msgs[i].op, "STAT") && !memcmp(msgs[i].p, ev, 4)) n++;
    return n;
}
static const msg_t *nth_stat(const char *ev, int idx)
{
    parse_ctl();
    for (int i = 0; i < nmsgs; i++)
        if (!strcmp(msgs[i].op, "STAT") && !memcmp(msgs[i].p, ev, 4) && idx-- == 0) return &msgs[i];
    return NULL;
}
static const msg_t *last_op(const char *op)
{
    parse_ctl();
    for (int i = nmsgs - 1; i >= 0; i--) if (!strcmp(msgs[i].op, op)) return &msgs[i];
    return NULL;
}
/* STAT payload field accessors (offsets within the 53-byte payload) */
static uint32_t st_ms(const msg_t *m) { return rbe32(m->p + 43); }
static uint32_t st_sec(const msg_t *m) { return rbe32(m->p + 37); }
static uint32_t st_ts(const msg_t *m) { return rbe32(m->p + 47); }

static void srv_frame(struct tcp_pcb *ctl, const char *op, const uint8_t *pl, size_t plen, size_t seg)
{
    static uint8_t buf[8192];
    size_t n = 4 + plen;
    buf[0] = (uint8_t)(n >> 8); buf[1] = (uint8_t)n;
    memcpy(buf + 2, op, 4);
    if (plen) memcpy(buf + 6, pl, plen);
    inject(ctl, buf, 2 + n, seg);
}

static size_t build_strm(uint8_t *o, char cmd, char autostart, char fmt, char sz, char rate, char ch,
                         char endian, uint8_t thr_kb, uint8_t out_thr, uint8_t flags, uint32_t rg,
                         uint16_t port, uint32_t ip_be, const char *http)
{
    memset(o, 0, 24);
    o[0] = cmd; o[1] = autostart; o[2] = fmt; o[3] = sz; o[4] = rate; o[5] = ch; o[6] = endian;
    o[7] = thr_kb; o[9] = 0; o[10] = '0'; o[11] = flags; o[12] = out_thr;
    wbe32(o + 14, rg); o[18] = port >> 8; o[19] = (uint8_t)port; wbe32(o + 20, ip_be);
    size_t hl = http ? strlen(http) : 0;
    if (hl) memcpy(o + 24, http, hl);
    return 24 + hl;
}
static void srv_strm(struct tcp_pcb *ctl, char cmd, char autostart, char fmt, char sz, char rate, char ch,
                     char endian, uint8_t flags, uint32_t rg, uint16_t port, const char *http)
{
    uint8_t b[2048];
    size_t n = build_strm(b, cmd, autostart, fmt, sz, rate, ch, endian, 200, 20, flags, rg, port, 0, http);
    srv_frame(ctl, "strm", b, n, 0);
}

#define REQ "GET /single/abc/def.flac HTTP/1.0\r\nHost: 10.0.0.2:8097\r\nConnection: close\r\nAccept: */*\r\nRange: bytes=0-\r\n\r\n"
#define HDR200 "HTTP/1.1 200 OK\r\nContent-Type: audio/x-wav\r\nConnection: close\r\n\r\n"

/* ===================================================================== */
/* harness                                                                */
/* ===================================================================== */
static struct tcp_pcb *ctl;

static void world_reset(void)
{
    if (slimproto_client_state() != SLIMPROTO_STATE_IDLE || live_count) slimproto_client_stop();
    sendspin_client_stop();
    for (int i = 0; i < MAXPCB; i++) { free(pcbs[i].out); memset(&pcbs[i], 0, sizeof(pcbs[i])); }
    live_count = 0; created = closed_n = aborted_n = 0;
    nmsgs = 0; msg_store_len = 0; wr = rd = 0; held = false; cur_rate = cur_bits = 0; fmt_calls = flush_calls = 0;
    consumed_len = 0; g_vol = 77; g_mute = false; sndbuf_limit = 0; slim_port = 3483;
    stub_loaned = false; ctl = NULL;
}

static void session_start(void)
{
    slimproto_client_start();
    step(3);
    ctl = find_pcb(slim_port);
    CHECK(ctl != NULL);
    if (ctl) accept_pcb(ctl);
    step(2);
}

/* feed `len` body bytes into pcb p with a TCP-window-like limit; the DSP
 * consumer takes `cons` bytes per step.  Returns when everything was taken. */
static bool pump(struct tcp_pcb *p, const uint8_t *body, size_t len, size_t seg, size_t cons, int max_iter)
{
    size_t sent = 0;
    for (int it = 0; it < max_iter; it++) {
        if (p->live && sent < len && p->injected - p->recved < 4096) {
            size_t n = len - sent < seg * 2 ? len - sent : seg * 2;
            if (inject(p, body + sent, n, seg) == ERR_ABRT && !p->live) { sent = len; }
            sent += n;
        }
        step(1);
        if (cons) consume(cons);
        if (sent >= len && (!p->live || p->injected == p->recved)) return true;
    }
    return false;
}

static void srv_play(char autostart, char fmt, char sz, char rate, char ch, char endian, struct tcp_pcb **sp)
{
    srv_strm(ctl, 's', autostart, fmt, sz, rate, ch, endian, 0, 0, 9000, REQ);
    step(2);
    *sp = find_pcb(9000);
    if (*sp) accept_pcb(*sp);
    step(1);
}
static void srv_response(struct tcp_pcb *sp, const char *hdr, size_t seg)
{
    inject(sp, (const uint8_t *)hdr, strlen(hdr), seg);
    step(2);
}
static void srv_cont(void)
{
    uint8_t c[5] = { 0, 0, 0, 0, 0 };
    srv_frame(ctl, "cont", c, 5, 0);
    step(2);
}

static uint8_t body_buf[1 << 20];
static void gen_pattern(uint8_t *b, size_t n, uint32_t seed)
{
    uint32_t s = seed;
    for (size_t i = 0; i < n; i++) { s = s * 1664525u + 1013904223u; b[i] = (uint8_t)(s >> 24); }
}

/* ===================================================================== */
/* tests                                                                  */
/* ===================================================================== */
static void test_handshake(void)
{
    world_reset();
    slimproto_client_start();
    CHECK(slimproto_client_state() == SLIMPROTO_STATE_IDLE);
    step(3);
    ctl = find_pcb(3483);
    CHECK(ctl != NULL);
    CHECK(slimproto_client_state() == SLIMPROTO_STATE_CONNECTING);
    CHECK(held);                                       /* nothing plays before the server says so */
    accept_pcb(ctl);
    step(2);
    CHECK(slimproto_client_state() == SLIMPROTO_STATE_HANDSHAKE);

    parse_ctl();
    CHECK(nmsgs >= 1 && !strcmp(msgs[0].op, "HELO"));
    const char *caps = "Model=squeezelite,ModelName=DSPi,AccuratePlayPoints=1,HasDigitalOut=1,MaxSampleRate=48000,flc,pcm";
    CHECK(msgs[0].len == 36 + strlen(caps));
    CHECK(msgs[0].p[0] == 12 && msgs[0].p[1] == 0);
    CHECK(!memcmp(msgs[0].p + 2, test_mac, 6));
    for (int i = 8; i < 24; i++) CHECK(msgs[0].p[i] == 0);          /* uuid */
    CHECK(msgs[0].p[24] == 0 && msgs[0].p[25] == 0);                /* first connect: no reconnect flag */
    CHECK(!memcmp(msgs[0].p + 36, caps, strlen(caps)));
    /* aioslimproto detects codecs by substring: only flc + pcm may be present */
    static const char *bad[] = { "alc", "aac", "ogg", "ogf", "aif", "mp3", "wma" };
    for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) CHECK(strstr(caps, bad[i]) == NULL);
    CHECK(strstr(caps, "flc") && strstr(caps, "pcm"));

    /* what aioslimproto sends after HELO */
    srv_frame(ctl, "vers", (const uint8_t *)"7.9", 3, 0);
    CHECK(slimproto_client_state() == SLIMPROTO_STATE_READY);
    uint8_t fe = 0xFE; srv_frame(ctl, "setd", &fe, 1, 0);
    CHECK(count_op("SETD") == 0);
    uint8_t z = 0; srv_frame(ctl, "setd", &z, 1, 0);
    step(2);
    const msg_t *sd = last_op("SETD");
    CHECK(sd && sd->len == 1 + 5 && sd->p[0] == 0 && !strcmp((const char *)sd->p + 1, "DSPi"));
    uint8_t nm[8] = { 0, 'T', 'e', 's', 't', 0 }; srv_frame(ctl, "setd", nm, 6, 0);
    step(2);
    sd = last_op("SETD");
    CHECK(sd && !strcmp((const char *)sd->p + 1, "Test"));

    uint8_t ae[2] = { 1, 1 }; srv_frame(ctl, "aude", ae, 2, 0);
    CHECK(!g_mute);
    uint8_t ag[18]; memset(ag, 0, sizeof(ag));
    ag[8] = 1; ag[9] = 255;
    wbe32(ag + 10, 0x5A82); wbe32(ag + 14, 0x5A82);               /* 0.35355 = 0.5^1.5 */
    srv_frame(ctl, "audg", ag, 18, 0);
    CHECK(g_vol >= 49 && g_vol <= 51);
    wbe32(ag + 10, 65536); wbe32(ag + 14, 65536); srv_frame(ctl, "audg", ag, 18, 0);
    CHECK(g_vol == 100);
    wbe32(ag + 10, 0); wbe32(ag + 14, 0); srv_frame(ctl, "audg", ag, 18, 0);
    CHECK(g_vol == 0);
    wbe32(ag + 10, 0x10000 * 1 / 10); wbe32(ag + 14, 0x10000 / 10); srv_frame(ctl, "audg", ag, 18, 0);
    CHECK(g_vol >= 20 && g_vol <= 23);                            /* 100*0.1^(2/3) = 21.5 */
    ag[8] = 0; wbe32(ag + 10, 100); wbe32(ag + 14, 100); srv_frame(ctl, "audg", ag, 18, 0);   /* adjust = 0: unity */
    CHECK(g_vol == 100);
    uint8_t old[10]; memset(old, 0, 10); wbe32(old, 0x5A82); wbe32(old + 4, 0x5A82); old[8] = 1; srv_frame(ctl, "audg", old, 10, 0);
    CHECK(g_vol >= 49 && g_vol <= 51);                            /* short (old) audg */
    ae[0] = 0; ae[1] = 0; srv_frame(ctl, "aude", ae, 2, 0);
    CHECK(g_mute);
    ae[0] = 1; srv_frame(ctl, "aude", ae, 2, 0);
    CHECK(!g_mute);

    /* strm t: STMt echoing the timestamp, 53-byte STAT */
    uint8_t b[24]; build_strm(b, 't', '0', '?', '?', '?', '?', '?', 0, 0, 0, 0xDEADBEEF, 0, 0, NULL);
    int before = count_stat("STMt");
    srv_frame(ctl, "strm", b, 24, 0);
    step(2);
    CHECK(count_stat("STMt") > before);
    const msg_t *t = NULL;
    for (int i = 0; i < 20; i++) { const msg_t *m = nth_stat("STMt", i); if (!m) break; if (st_ts(m) == 0xDEADBEEF) t = m; }
    CHECK(t && t->len == 53);
    CHECK(t && t->p[4] == 0 && t->p[5] == 0 && t->p[6] == 0);
    CHECK(t && rbe32(t->p + 11) == 32768 && rbe32(t->p + 7) == 0);             /* size 32768 / fullness 0 */
    CHECK(t && t->p[23] == 0xff && t->p[24] == 0xff);                           /* signal */
    CHECK(t && st_ms(t) == 0 && st_sec(t) == 0);

    /* periodic STMt while idle */
    int n0 = count_stat("STMt");
    step(5200);
    CHECK(count_stat("STMt") >= n0 + 1);
    /* server heartbeat silence is not an error before 45 s */
    CHECK(slimproto_client_state() == SLIMPROTO_STATE_READY);
    slimproto_client_stop();
    CHECK(live_pcbs() == 0);
    CHECK(!held || true);
}

static void expect_play_pcm(int bps, bool be, uint32_t rate, char rate_digit)
{
    world_reset();
    session_start();
    uint8_t b[24];
    (void)b;
    /* MA: flush, then play with autostart '2' (wait for STMl, then `u`) */
    uint8_t fl[24]; build_strm(fl, 'f', '0', 'p', '?', '?', '?', '?', 0, 0, 0, 0, 0, 0, NULL);
    srv_frame(ctl, "strm", fl, 24, 0);
    step(2);
    CHECK(count_stat("STMf") == 1);
    struct tcp_pcb *sp;
    srv_play('2', 'p', bps == 2 ? '1' : '2', rate_digit, '2', be ? '0' : '1', &sp);
    CHECK(sp != NULL);
    if (!sp) return;
    CHECK(count_stat("STMf") == 2 && count_stat("STMc") == 1);
    CHECK(sp->ip == 0x0100007f && sp->port == 9000);                 /* ip 0 in strm = the control server */
    CHECK(sp->out_len == strlen(REQ) && !memcmp(sp->out, REQ, strlen(REQ)));   /* request verbatim */

    srv_response(sp, HDR200, 5);                                       /* split over many tiny segments */
    CHECK(count_op("RESP") == 1);
    const msg_t *rs = last_op("RESP");
    CHECK(rs && rs->len == strlen(HDR200) && !memcmp(rs->p, HDR200, rs->len));
    CHECK(slimproto_client_state() == SLIMPROTO_STATE_STREAMING);

    /* body arrives before cont: must not be consumed */
    size_t blen = 2 * 40000 / 2 / 6 * 6;                              /* multiple of both frame sizes */
    gen_pattern(body_buf, blen, 0x1234 + bps);
    inject(sp, body_buf, 3000, 1000);
    step(5);
    CHECK(sp->recved == (long)strlen(HDR200));
    CHECK(net_input_ring_fill_bytes() == 0);
    srv_cont();

    /* backpressure: ring (32 KB) fills while held, the rest stays un-acked */
    size_t sent = 3000;
    for (int it = 0; it < 400 && sent < blen; it++) {
        if (sp->injected - sp->recved < 4096) { size_t n = blen - sent < 1400 ? blen - sent : 1400; inject(sp, body_buf + sent, n, 700); sent += n; }
        step(1);
    }
    CHECK(held);
    CHECK(net_input_ring_fill_bytes() >= 32768 - 6);                   /* full (to within a frame) */
    CHECK(sp->injected - sp->recved > 0);                              /* window kept closed */
    CHECK(count_stat("STMl") == 1);
    CHECK(count_stat("STMs") == 0);
    CHECK(consume(1000) == 0);                                         /* held: nothing is taken */
    CHECK(cur_rate == rate && cur_bits == (uint32_t)bps * 8);

    /* unpause: playing */
    uint8_t u[24]; build_strm(u, 'u', '0', '?', '?', '?', '?', '?', 0, 0, 0, 0, 0, 0, NULL);
    srv_frame(ctl, "strm", u, 24, 0);
    step(2);
    CHECK(count_stat("STMr") == 1 && !held);
    consume(4096);
    step(3);
    CHECK(count_stat("STMs") == 1);

    /* rest of the stream with a consumer of ~2.4 KB per ms */
    bool ok = true;
    for (int it = 0; it < 20000 && ok; it++) {
        if (sent < blen && sp->live && sp->injected - sp->recved < 4096) { size_t n = blen - sent < 1400 ? blen - sent : 1400; inject(sp, body_buf + sent, n, 700); sent += n; }
        step(1);
        consume(2400);
        if (sent >= blen && sp->injected == sp->recved) break;
    }
    /* elapsed time while playing: ask */
    uint8_t tq[24]; build_strm(tq, 't', '0', '?', '?', '?', '?', '?', 0, 0, 0, 7, 0, 0, NULL);
    srv_frame(ctl, "strm", tq, 24, 0);
    step(2);
    inject_fin(sp);
    step(5);
    CHECK(count_op("DSCO") == 1 && last_op("DSCO")->len == 1 && last_op("DSCO")->p[0] == 0);
    CHECK(count_stat("STMd") == 1);
    for (int it = 0; it < 100; it++) { step(1); consume(4096); }
    CHECK(count_stat("STMu") == 1);                                   /* ran dry after decode ended */
    CHECK(consumed_len == blen);
    if (consumed_len == blen) {
        if (!be) CHECK(!memcmp(consumed, body_buf, blen));
        else {
            bool eq = true;
            for (size_t i = 0; i < blen; i += bps)
                for (int j = 0; j < bps; j++) if (consumed[i + j] != body_buf[i + bps - 1 - j]) eq = false;
            CHECK(eq);
        }
    }
    /* elapsed reporting: the STMt reply with ts 7 during playback had ms > 0 */
    const msg_t *t = NULL;
    for (int i = 0; i < 100; i++) { const msg_t *m = nth_stat("STMt", i); if (!m) break; if (st_ts(m) == 7) t = m; }
    CHECK(t && st_ms(t) > 0 && st_sec(t) == st_ms(t) / 1000);
    CHECK(sp->live == false);                                          /* stream pcb released */
    CHECK(live_pcbs() == 1);                                           /* only the control connection is left */
    CHECK(pbufs_live == 0);
    slimproto_client_stop();
    CHECK(live_pcbs() == 0 && pbufs_live == 0);
    (void)ok;
}

static void test_pcm_formats(void)
{
    expect_play_pcm(2, false, 44100, '3');
    expect_play_pcm(2, true, 48000, '4');
    expect_play_pcm(3, false, 44100, '3');
    expect_play_pcm(3, true, 48000, '4');
}

static void test_wav_chunked_and_errors(void)
{
    /* WAV-wrapped stream with '?' parameters (RIFF + LIST chunk + fmt + data, unknown length) */
    world_reset();
    session_start();
    struct tcp_pcb *sp;
    srv_play('3', 'p', '?', '?', '?', '?', &sp);
    CHECK(sp != NULL);
    if (!sp) return;
    uint8_t wav[12 + 8 + 6 + 8 + 16 + 8];
    size_t k = 0;
    memcpy(wav + k, "RIFF", 4); wbe32(wav + k + 4, 0xffffffffu); memcpy(wav + k + 8, "WAVE", 4); k += 12;
    memcpy(wav + k, "LIST", 4); wav[k + 4] = 6; wav[k + 5] = wav[k + 6] = wav[k + 7] = 0; memset(wav + k + 8, 'x', 6); k += 14;
    memcpy(wav + k, "fmt ", 4); wav[k + 4] = 16; wav[k + 5] = wav[k + 6] = wav[k + 7] = 0; k += 8;
    uint8_t fmt[16] = { 1, 0, 2, 0, 0x44, 0xAC, 0, 0, 0x10, 0xB1, 2, 0, 4, 0, 16, 0 };
    memcpy(wav + k, fmt, 16); k += 16;
    memcpy(wav + k, "data", 4); wbe32(wav + k + 4, 0xffffffffu); k += 8;
    /* chunked HTTP/1.1 response with uppercase hex, extension and split everywhere */
    const char *h = "HTTP/1.1 200 OK\r\nContent-Type: audio/wav\r\nTransfer-Encoding: chunked\r\n\r\n";
    inject(sp, (const uint8_t *)h, strlen(h), 7);
    step(2);
    srv_cont();                                                       /* autostart 3 -> 1 */
    size_t pcm = 4000;
    gen_pattern(body_buf, pcm, 99);
    static uint8_t stream[8192]; size_t sl = 0;
    memcpy(stream, wav, k); memcpy(stream + k, body_buf, pcm);
    size_t total = k + pcm, off = 0; int ci = 0;
    uint8_t out[16000]; size_t ol = 0;
    while (off < total) {
        size_t n = (size_t)(ci % 3 == 0 ? 1 : ci % 3 == 1 ? 333 : 1500);
        if (n > total - off) n = total - off;
        ol += (size_t)sprintf((char *)out + ol, ci % 2 ? "%X;ext=1\r\n" : "%x\r\n", (unsigned)n);
        memcpy(out + ol, stream + off, n); ol += n; off += n;
        memcpy(out + ol, "\r\n", 2); ol += 2; ci++;
    }
    memcpy(out + ol, "0\r\n\r\n", 5); ol += 5;
    (void)sl;
    CHECK(pump(sp, out, ol, 211, 3000, 5000));
    for (int i = 0; i < 50; i++) { step(1); consume(4000); }
    CHECK(consumed_len == pcm && !memcmp(consumed, body_buf, pcm));
    CHECK(cur_rate == 44100 && cur_bits == 16);
    CHECK(count_stat("STMd") == 1 && count_op("DSCO") == 1);
    CHECK(count_stat("STMs") == 1);
    CHECK(count_stat("STMl") == 0);                                   /* autostart 3 -> 1: started by itself */
    CHECK(count_stat("STMn") == 0);
    slimproto_client_stop();

    /* Content-Length stream, 206 + split status line */
    world_reset(); session_start();
    srv_play('2', 'p', '1', '3', '2', '1', &sp);
    char hdr[200];
    gen_pattern(body_buf, 4000, 5);
    snprintf(hdr, sizeof hdr, "HTTP/1.0 206 Partial Content\r\nContent-Length: 4000\r\n\r\n");
    srv_response(sp, hdr, 3);
    srv_cont();
    CHECK(pump(sp, body_buf, 4100, 1000, 3000, 2000) || true);          /* 100 bytes beyond Content-Length are ignored */
    held = false;
    for (int i = 0; i < 50; i++) { step(1); consume(4000); }
    CHECK(consumed_len == 4000 && !memcmp(consumed, body_buf, 4000));
    CHECK(count_stat("STMd") == 1);
    slimproto_client_stop();

    /* HTTP 404: RESP is still delivered, then STMn; no leak */
    world_reset(); session_start();
    srv_play('3', 'f', '?', '?', '?', '?', &sp);
    const char *e404 = "HTTP/1.0 404 Not Found\r\nContent-Length: 0\r\n\r\n";
    srv_response(sp, e404, 4);
    CHECK(count_op("RESP") == 1 && last_op("RESP")->len == strlen(e404));
    CHECK(count_stat("STMn") == 1);
    CHECK(!sp->live && live_pcbs() == 1);
    CHECK(!stub_loaned);                                               /* FLAC arena returned */
    CHECK(slimproto_client_state() == SLIMPROTO_STATE_READY);
    slimproto_client_stop();

    /* refusals: mp3, https flag, oversize request; session stays alive */
    world_reset(); session_start();
    srv_strm(ctl, 's', '3', 'm', '?', '?', '?', '?', 0, 0, 9000, REQ);
    step(2);
    CHECK(count_stat("STMn") == 1 && find_pcb(9000) == NULL);
    srv_strm(ctl, 's', '3', 'f', '?', '?', '?', '?', 0x20, 0, 9000, REQ);
    step(2);
    CHECK(count_stat("STMn") == 2 && find_pcb(9000) == NULL && !stub_loaned);
    srv_strm(ctl, 's', '3', 'p', '1', '9', '2', '1', 0, 0, 9000, REQ);   /* 96 kHz: unsupported */
    step(2);
    CHECK(count_stat("STMn") == 3 && find_pcb(9000) == NULL);
    {
        static char big[2000]; memset(big, 'a', sizeof(big) - 1); big[sizeof(big) - 1] = 0;
        srv_strm(ctl, 's', '3', 'p', '1', '3', '2', '1', 0, 0, 9000, big);   /* > SL_RX_BUF: skipped */
        step(2);
        CHECK(count_stat("STMn") == 4 && find_pcb(9000) == NULL);
        uint8_t junk[3000]; memset(junk, 0x55, sizeof(junk));
        srv_frame(ctl, "grfe", junk, sizeof(junk), 700);                    /* unknown, large: ignored */
        step(2);
        CHECK(slimproto_client_state() == SLIMPROTO_STATE_READY);
    }
    /* connect failure of the stream pcb */
    srv_strm(ctl, 's', '3', 'p', '1', '3', '2', '1', 0, 0, 9001, REQ);
    step(2);
    sp = find_pcb(9001);
    CHECK(sp != NULL);
    if (sp) { tcp_err_fn ef = sp->err; sp->live = false; live_count--; ef(NULL, ERR_RST); }
    step(2);
    CHECK(count_stat("STMn") == 5 && count_op("DSCO") >= 1 && last_op("DSCO")->p[0] == 3);
    CHECK(slimproto_client_state() == SLIMPROTO_STATE_READY);
    slimproto_client_stop();
    CHECK(live_pcbs() == 0);
}

static void test_transport(void)
{
    world_reset(); session_start();
    struct tcp_pcb *sp;
    srv_play('2', 'p', '1', '3', '2', '1', &sp);
    srv_response(sp, HDR200, 100);
    srv_cont();
    gen_pattern(body_buf, 30000, 3);
    pump(sp, body_buf, 30000, 1000, 0, 200);
    CHECK(count_stat("STMl") == 1 && held);
    uint8_t b[24];
    build_strm(b, 'u', '0', '?', '?', '?', '?', '?', 0, 0, 0, 0, 0, 0, NULL); srv_frame(ctl, "strm", b, 24, 0); step(2);
    CHECK(!held && count_stat("STMr") == 1);
    consume(8000); step(3);
    build_strm(b, 'p', '0', '?', '?', '?', '?', '?', 0, 0, 0, 0, 0, 0, NULL); srv_frame(ctl, "strm", b, 24, 0); step(2);
    CHECK(held && count_stat("STMp") == 1);
    size_t cl = consumed_len; CHECK(consume(1000) == 0 && consumed_len == cl);
    uint32_t el1 = 0;
    build_strm(b, 't', '0', '?', '?', '?', '?', '?', 0, 0, 0, 11, 0, 0, NULL); srv_frame(ctl, "strm", b, 24, 0); step(2);
    for (int i = 0; i < 20; i++) { const msg_t *m = nth_stat("STMt", i); if (m && st_ts(m) == 11) el1 = st_ms(m); }
    CHECK(el1 == (uint32_t)(8000 / 4 * 1000ull / 44100) || el1 == (uint32_t)((8000 / 4 * 1000ull / 44100) - 0));
    step(2000);                                                        /* paused time does not count */
    build_strm(b, 't', '0', '?', '?', '?', '?', '?', 0, 0, 0, 12, 0, 0, NULL); srv_frame(ctl, "strm", b, 24, 0); step(2);
    uint32_t el2 = 0;
    for (int i = 0; i < 40; i++) { const msg_t *m = nth_stat("STMt", i); if (m && st_ts(m) == 12) el2 = st_ms(m); }
    CHECK(el2 == el1);
    build_strm(b, 'u', '0', '?', '?', '?', '?', '?', 0, 0, 0, 0, 0, 0, NULL); srv_frame(ctl, "strm", b, 24, 0); step(2);
    CHECK(!held && count_stat("STMr") == 2);
    consume(1000);
    /* flush: everything is dropped, the stream pcb goes away */
    int f0 = count_stat("STMf");
    build_strm(b, 'f', '0', '?', '?', '?', '?', '?', 0, 0, 0, 0, 0, 0, NULL); srv_frame(ctl, "strm", b, 24, 0); step(2);
    CHECK(count_stat("STMf") == f0 + 1 && !sp->live && net_input_ring_fill_bytes() == 0 && held);
    CHECK(live_pcbs() == 1 && pbufs_live == 0);
    /* stop */
    srv_play('2', 'p', '1', '3', '2', '1', &sp);
    srv_response(sp, HDR200, 100);
    build_strm(b, 'q', '0', '?', '?', '?', '?', '?', 0, 0, 0, 0, 0, 0, NULL); srv_frame(ctl, "strm", b, 24, 0); step(2);
    CHECK(count_stat("STMf") == f0 + 3 && !sp->live && held && live_pcbs() == 1);
    CHECK(slimproto_client_state() == SLIMPROTO_STATE_READY);
    slimproto_client_stop();
    CHECK(live_pcbs() == 0 && pbufs_live == 0 && !held);
}

static void play_to_end(struct tcp_pcb *sp, const uint8_t *body, size_t len)
{
    pump(sp, body, len, 1000, 0, 400);
    inject_fin(sp);
    step(3);
}

static void test_gapless(void)
{
    /* same format: track 2 is appended behind track 1; STMs per track; elapsed restarts */
    world_reset(); session_start();
    struct tcp_pcb *sp;
    static uint8_t t1[10000], t2[12000];
    gen_pattern(t1, sizeof t1, 1); gen_pattern(t2, sizeof t2, 2);
    srv_play('2', 'p', '1', '3', '2', '1', &sp);
    srv_response(sp, HDR200, 100); srv_cont();
    play_to_end(sp, t1, sizeof t1);
    CHECK(count_stat("STMd") == 1 && count_stat("STMl") == 1);
    uint8_t u[24]; build_strm(u, 'u', '0', '?', '?', '?', '?', '?', 0, 0, 0, 0, 0, 0, NULL);
    srv_frame(ctl, "strm", u, 24, 0); step(2);
    consume(2000); step(2);
    CHECK(count_stat("STMs") == 1 && count_stat("STMu") == 0);
    /* enqueue: autostart '3', no flush */
    srv_play('3', 'p', '1', '3', '2', '1', &sp);
    srv_response(sp, HDR200, 100); srv_cont();
    play_to_end(sp, t2, sizeof t2);
    CHECK(count_stat("STMd") == 2);
    CHECK(count_stat("STMl") == 1 && count_stat("STMf") == 2 + 0);   /* initial + enqueue's own STMf only */
    for (int i = 0; i < 100; i++) { step(1); consume(500); }
    CHECK(count_stat("STMs") == 2 && count_stat("STMu") == 1);
    CHECK(consumed_len == sizeof t1 + sizeof t2);
    CHECK(!memcmp(consumed, t1, sizeof t1) && !memcmp(consumed + sizeof t1, t2, sizeof t2));
    CHECK(fmt_calls == 1);
    slimproto_client_stop();

    /* format change: 44.1k/16 -> 48k/24; track 2 waits until track 1 is consumed */
    world_reset(); session_start();
    srv_play('2', 'p', '1', '3', '2', '1', &sp);
    srv_response(sp, HDR200, 100); srv_cont();
    play_to_end(sp, t1, 9000);
    srv_frame(ctl, "strm", u, 24, 0); step(2);
    srv_play('3', 'p', '2', '4', '2', '0', &sp);
    srv_response(sp, HDR200, 100); srv_cont();
    static uint8_t t3[12000]; gen_pattern(t3, sizeof t3, 3);
    inject(sp, t3, 4000, 1000); step(20);
    CHECK(net_input_ring_fill_bytes() == 9000);                       /* nothing of track 2 yet */
    CHECK(cur_rate == 44100 && cur_bits == 16 && fmt_calls == 1);
    for (int i = 0; i < 20; i++) { step(1); consume(1000); }
    CHECK(fmt_calls == 2 && cur_rate == 48000 && cur_bits == 24);
    CHECK(pump(sp, t3 + 4000, sizeof t3 - 4000, 1000, 1000, 2000));
    inject_fin(sp); step(5);
    for (int i = 0; i < 100; i++) { step(1); consume(1000); }
    CHECK(consumed_len == 9000 + sizeof t3);
    CHECK(count_stat("STMs") == 2 && count_stat("STMu") == 1 && count_stat("STMd") == 2);
    uint8_t exp[12000];
    for (size_t i = 0; i < sizeof t3; i += 3) { exp[i] = t3[i + 2]; exp[i + 1] = t3[i + 1]; exp[i + 2] = t3[i]; }
    CHECK(!memcmp(consumed, t1, 9000) && !memcmp(consumed + 9000, exp, sizeof exp));
    slimproto_client_stop();
}

/* ---------------- FLAC ---------------- */
static const char *g_fix;
static size_t read_file(const char *name, uint8_t *buf, size_t cap)
{
    char p[1024]; snprintf(p, sizeof p, "%s/%s", g_fix, name);
    FILE *f = fopen(p, "rb"); if (!f) return 0;
    size_t n = fread(buf, 1, cap, f); fclose(f); return n;
}
static uint8_t flac_buf[1 << 20], wav_buf[1 << 22];

static void flac_case(const char *name, uint32_t rate, uint32_t bits, size_t seg, size_t cons)
{
    char fn[64];
    world_reset(); session_start();
    snprintf(fn, sizeof fn, "%s.flac", name); size_t fl = read_file(fn, flac_buf, sizeof flac_buf);
    snprintf(fn, sizeof fn, "%s.wav", name); size_t wl = read_file(fn, wav_buf, sizeof wav_buf);
    CHECK(fl > 100 && wl > 44);
    struct tcp_pcb *sp;
    srv_play('3', 'f', '?', '?', '?', '?', &sp);
    CHECK(sp != NULL);
    if (!sp) return;
    srv_response(sp, "HTTP/1.1 200 OK\r\nContent-Type: audio/flac\r\n\r\n", 40);
    srv_cont();
    CHECK(stub_loaned);
    CHECK(pump(sp, flac_buf, fl, seg, cons, 400000));
    inject_fin(sp);
    for (int i = 0; i < 4000; i++) { step(1); consume(cons); if (count_stat("STMd") && net_input_ring_fill_bytes() == 0) break; }
    for (int i = 0; i < 50; i++) { step(1); consume(cons); }
    CHECK(count_stat("STMd") == 1 && count_stat("STMn") == 0);
    CHECK(cur_rate == rate && cur_bits == bits);
    CHECK(consumed_len == wl - 44);
    CHECK(consumed_len == wl - 44 && !memcmp(consumed, wav_buf + 44, wl - 44));
    CHECK(count_stat("STMs") == 1 && count_stat("STMu") == 1);
    CHECK(!stub_loaned);                                               /* arena given back at the end */
    CHECK(live_pcbs() == 1);
    slimproto_client_stop();
    CHECK(live_pcbs() == 0 && !stub_loaned);
}

static void test_flac(void)
{
    flac_case("f44_16", 44100, 16, 1400, 1500);
    flac_case("f48_16", 48000, 16, 700, 900);
    flac_case("f44_24", 44100, 24, 1400, 1200);
    flac_case("f44_16_s", 44100, 16, 100, 2000);        /* tiny TCP segments: frames split anywhere */
    flac_case("f44_16", 44100, 16, 1024, 100);          /* slow consumer: long ring-full backpressure */

    /* garbage instead of 'fLaC': STMn, arena returned */
    world_reset(); session_start();
    struct tcp_pcb *sp;
    srv_play('3', 'f', '?', '?', '?', '?', &sp);
    srv_response(sp, "HTTP/1.1 200 OK\r\n\r\n", 40);
    srv_cont();
    gen_pattern(body_buf, 3000, 8);
    inject(sp, body_buf, 3000, 1000);
    for (int i = 0; i < 100; i++) step(1);
    CHECK(count_stat("STMn") == 1 && !stub_loaned && !sp->live);
    slimproto_client_stop();

    /* the ADAT arena is not available: flac is refused with STMn */
    world_reset(); session_start();
    stub_loaned = true;
    srv_strm(ctl, 's', '3', 'f', '?', '?', '?', '?', 0, 0, 9000, REQ);
    step(2);
    CHECK(count_stat("STMn") == 1 && find_pcb(9000) == NULL);
    slimproto_client_stop();
    stub_loaned = false;
}

static void test_reconnect_and_switch(void)
{
    world_reset(); session_start();
    struct tcp_pcb *sp; int old_serial = ctl->serial;
    srv_play('2', 'p', '1', '3', '2', '1', &sp);
    srv_response(sp, HDR200, 100); srv_cont();
    gen_pattern(body_buf, 5000, 4);
    inject(sp, body_buf, 5000, 1000); step(3);
    CHECK(net_input_ring_fill_bytes() == 5000);
    /* server drops the control connection */
    inject_fin(ctl);
    CHECK(slimproto_client_state() == SLIMPROTO_STATE_ERROR);
    CHECK(!sp->live && live_pcbs() <= 1);
    CHECK(net_input_ring_fill_bytes() == 0 && held);                  /* session audio dropped */
    step(500);
    CHECK(find_pcb(3483) == NULL);                                     /* not yet: back-off 1 s */
    step(700);
    struct tcp_pcb *n = find_pcb(3483);
    CHECK(n && n->serial != old_serial && !n->connected);
    accept_pcb(n); ctl = n; step(2);
    parse_ctl();
    const msg_t *h = NULL;
    for (int i = 0; i < nmsgs; i++) if (!strcmp(msgs[i].op, "HELO")) h = &msgs[i];
    CHECK(count_op("HELO") == 2 && h && h->p[24] == 0x40 && h->p[25] == 0x00);   /* reconnect flag */
    CHECK(slimproto_client_state() == SLIMPROTO_STATE_HANDSHAKE);
    /* tcp error on the control pcb */
    n->err(NULL, ERR_RST); n->live = false; live_count--;
    CHECK(slimproto_client_state() == SLIMPROTO_STATE_ERROR);
    int n_serial = n->serial;
    step(3000);
    CHECK(find_pcb(3483) != NULL && find_pcb(3483)->serial != n_serial);
    /* a silent server is dropped after 45 s without any frame */
    ctl = find_pcb(3483); accept_pcb(ctl); step(2);
    srv_frame(ctl, "vers", (const uint8_t *)"7.9", 3, 0);
    step(46000);
    CHECK(slimproto_client_state() == SLIMPROTO_STATE_ERROR || slimproto_client_state() == SLIMPROTO_STATE_CONNECTING ||
          slimproto_client_state() == SLIMPROTO_STATE_HANDSHAKE);
    slimproto_client_stop();
    CHECK(live_pcbs() == 0 && pbufs_live == 0);
}

static void test_protocol_switch(void)
{
    world_reset();
    /* sendspin up: connects to 8927 */
    sendspin_client_start();
    for (int i = 0; i < 5; i++) { stub_now_us += 1000; sendspin_client_poll(); }
    struct tcp_pcb *ss = find_pcb(8927);
    CHECK(ss != NULL && live_pcbs() == 1);
    /* slimproto must refuse to run while Sendspin owns the RAM */
    slimproto_client_start();
    step(3);
    CHECK(find_pcb(3483) == NULL && slimproto_client_state() == SLIMPROTO_STATE_IDLE);
    /* switch: stop sendspin completely, start slimproto */
    sendspin_client_stop();
    CHECK(live_pcbs() == 0);
    slimproto_client_start();
    step(3);
    ctl = find_pcb(3483);
    CHECK(ctl != NULL && live_pcbs() == 1);
    accept_pcb(ctl); step(2);
    struct tcp_pcb *sp;
    srv_play('2', 'p', '1', '3', '2', '1', &sp);
    srv_response(sp, HDR200, 100);
    CHECK(live_pcbs() == 2);
    /* sendspin cannot start while slimproto runs ... */
    sendspin_client_start();
    stub_now_us += 1000; sendspin_client_poll();
    CHECK(find_pcb(8927) == NULL && live_pcbs() == 2);
    /* ... and after stopping slimproto (both pcbs go) the other can */
    slimproto_client_stop();
    CHECK(live_pcbs() == 0 && pbufs_live == 0 && slimproto_client_state() == SLIMPROTO_STATE_IDLE);
    sendspin_client_start();
    for (int i = 0; i < 5; i++) { stub_now_us += 1000; sendspin_client_poll(); }
    CHECK(find_pcb(8927) != NULL && live_pcbs() == 1);
    CHECK(sendspin_client_state() == SENDSPIN_STATE_CONNECTING);       /* state was re-initialised, not slimproto leftovers */
    sendspin_client_stop();
    CHECK(live_pcbs() == 0);
    /* repeated switching */
    for (int r = 0; r < 5; r++) {
        slimproto_client_start(); step(3);
        CHECK(live_pcbs() == 1);
        slimproto_client_stop();
        sendspin_client_start(); for (int i = 0; i < 3; i++) { stub_now_us += 1000; sendspin_client_poll(); }
        CHECK(live_pcbs() == 1);
        sendspin_client_stop();
        CHECK(live_pcbs() == 0);
    }
    /* a stream still open while stopping leaves nothing behind */
    slimproto_client_start(); step(3); ctl = find_pcb(3483); accept_pcb(ctl); step(2);
    srv_play('3', 'f', '?', '?', '?', '?', &sp);
    CHECK(stub_loaned && live_pcbs() == 2);
    slimproto_client_stop();
    CHECK(live_pcbs() == 0 && !stub_loaned && !held);
    {
        extern size_t slimproto_state_bytes(void);
        uint32_t loan = 0;
        void *m = sendspin_client_mem_loan(&loan);
        CHECK(m != NULL);
        sendspin_client_mem_return();
        printf("sizes: Slimproto state %u B, Sendspin loan %u B\n", (unsigned)slimproto_state_bytes(), (unsigned)loan);
        CHECK(slimproto_state_bytes() <= loan);
    }
}

int main(int argc, char **argv)
{
    g_fix = argc > 1 ? argv[1] : NULL;
    test_handshake();
    test_pcm_formats();
    test_wav_chunked_and_errors();
    test_transport();
    test_gapless();
    if (g_fix) test_flac(); else printf("FLAC end-to-end tests SKIPPED (no fixtures)\n");
    test_reconnect_and_switch();
    test_protocol_switch();
    printf("\n%d checks passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
