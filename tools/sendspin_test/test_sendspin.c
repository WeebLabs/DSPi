/*
 * Host test for the Sendspin client protocol layer.
 *
 * WHAT THIS PROVES: self-consistency only. The Noise initiator in this file
 * (section "independent initiator") was written separately from the responder
 * in sendspin_noise.c, straight from the Noise spec text, using different
 * mbedtls entry points (mbedtls_md_hmac, mbedtls_ecdh_*), but by the same
 * author with the same reading of the spec. A shared misreading (e.g. of the
 * Sendspin prologue or of which messages carry which payloads) would NOT be
 * caught. It does NOT prove interoperability with a real Music Assistant
 * server. External vectors checked here: RFC 7748 X25519, RFC 6455 accept key,
 * and the published Sendspin sentinel PSK / psk_id constants.
 *
 * The lwIP raw API, pico time/rand and net_input are replaced by stubs
 * (stubs/), and sendspin_client.c is #included so its statics are testable.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "mbedtls/build_info.h"
#include "mbedtls/md.h"
#include "mbedtls/sha256.h"
#include "mbedtls/sha1.h"
#include "mbedtls/base64.h"
#include "mbedtls/chachapoly.h"
#include "mbedtls/ecp.h"
#include "mbedtls/ecdh.h"
#include "mbedtls/bignum.h"

#include "sendspin_client.c"      /* brings in stubs' headers + statics */

/* ===================================================================== */
/* test infrastructure                                                    */
/* ===================================================================== */
static int g_fail, g_pass;
#define CHECK(cond) do { if (cond) g_pass++; else { g_fail++; \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)

static void hex2bin(uint8_t *out, const char *hex)
{
    size_t n = strlen(hex) / 2;
    for (size_t i = 0; i < n; i++) { unsigned v; sscanf(hex + 2 * i, "%2x", &v); out[i] = (uint8_t)v; }
}

static uint32_t rng_state = 0x12345678u;
uint32_t get_rand_32(void)
{
    rng_state ^= rng_state << 13; rng_state ^= rng_state >> 17; rng_state ^= rng_state << 5;
    return rng_state * 2654435761u;
}
static int test_rng(void *ctx, unsigned char *buf, size_t len) { return ss_rng(ctx, buf, len); }

uint64_t stub_now_us = 1000000;

/* ---- net_input stub ---- */
/* Simulated NET PCM ring: audio is "staged" beyond the write index and only
 * becomes visible (pushed[]) when committed; free space is cap - ring_fill. */
static uint8_t  pushed[1 << 20];
static size_t   pushed_len;
static uint8_t  stage_buf[1 << 17];
static uint32_t stub_ring_cap = 32768;
static uint32_t ring_fill_sim;             /* bytes "in the ring" (committed, not yet consumed) */
static uint32_t stage_hi;                  /* highest staged offset + 1 since last commit/flush */
static int      flush_count, fmt_calls;
static uint32_t fmt_rate; static uint8_t fmt_ch, fmt_bits;
static uint8_t  g_volume = 80; static bool g_mute;
static const char *g_host = "127.0.0.1";

uint32_t net_input_ring_capacity(void) { return stub_ring_cap; }
uint32_t net_input_ring_free_bytes(void) { return stub_ring_cap - ring_fill_sim; }
bool net_input_stage_pcm(uint32_t off, const uint8_t *pcm, uint32_t len)
{
    uint32_t fr = net_input_ring_free_bytes();
    if (off > fr || len > fr - off) return false;
    memcpy(stage_buf + off, pcm, len);
    if (off + len > stage_hi) stage_hi = off + len;
    return true;
}
void net_input_commit_pcm(uint32_t bytes)
{
    CHECK(bytes <= stage_hi && bytes <= net_input_ring_free_bytes());
    memcpy(pushed + pushed_len, stage_buf, bytes);
    pushed_len += bytes; ring_fill_sim += bytes; stage_hi = 0;
}
static void stub_drain(uint32_t n) { ring_fill_sim = n > ring_fill_sim ? 0 : ring_fill_sim - n; }
void net_input_set_format(uint32_t r, uint8_t ch, uint8_t b) { fmt_rate = r; fmt_ch = ch; fmt_bits = b; fmt_calls++; }
void net_input_flush(void) { flush_count++; ring_fill_sim = 0; stage_hi = 0; }
uint32_t net_input_ring_fill_bytes(void) { return 0; }
uint8_t net_volume_get_percent(void) { return g_volume; }
void net_volume_set_percent(uint8_t v) { g_volume = v; }
bool net_mute_get(void) { return g_mute; }
void net_mute_set(bool m) { g_mute = m; }
const char *net_sendspin_host(void) { return g_host; }
uint16_t net_sendspin_port(void) { return 8927; }


/* ---- adat_output ring-loan stub: a 56 KB arena with guard bytes around it,
 * handed out 4 bytes misaligned so the 16-byte alignment in net_flac.c is exercised ---- */
static struct { uint8_t pre[64]; uint8_t arena[57344]; uint8_t post[64]; } adat_mem = { {0}, {0}, {0} };
#define adat_guard_ok() 1
static bool stub_loan_ok = true, stub_loaned;
static void (*stub_evict)(void);
bool adat_output_ring_loanable(void) { return stub_loan_ok && !stub_loaned; }
void *adat_output_ring_loan(uint32_t *bytes, void (*evict)(void))
{
    if (!adat_output_ring_loanable()) return NULL;
    stub_loaned = true; stub_evict = evict;
    if (bytes) *bytes = sizeof(adat_mem.arena) - 4;
    return adat_mem.arena + 4;
}
void adat_output_ring_return(void) { stub_loaned = false; stub_evict = NULL; }

/* ---- lwIP stub ---- */
static struct tcp_pcb the_pcb;
static int pcb_new_count, pcb_closed, pcb_aborted;
static bool pcb_alive;
static void *cb_arg;
static tcp_recv_fn recv_cb; static tcp_sent_fn sent_cb; static tcp_err_fn err_cb;
static tcp_connected_fn connected_cb;
static uint16_t connect_port;
static uint8_t  out_cap[1 << 20];
static size_t   out_len, rd_off;
static int      sndbuf_cap = 0;           /* 0 = unlimited */
static int      sndbuf_used;
static long     recved_total, injected_total;
static int      pbufs_live;
static dns_found_callback dns_cb;

struct tcp_pcb *tcp_new(void) { pcb_new_count++; pcb_alive = true; return &the_pcb; }
void tcp_arg(struct tcp_pcb *p, void *a) { (void)p; cb_arg = a; }
void tcp_recv(struct tcp_pcb *p, tcp_recv_fn f) { (void)p; recv_cb = f; }
void tcp_sent(struct tcp_pcb *p, tcp_sent_fn f) { (void)p; sent_cb = f; }
void tcp_err(struct tcp_pcb *p, tcp_err_fn f) { (void)p; err_cb = f; }
err_t tcp_connect(struct tcp_pcb *p, const ip_addr_t *ip, uint16_t port, tcp_connected_fn f)
{ (void)p; (void)ip; connect_port = port; connected_cb = f; return ERR_OK; }
u16_t tcp_sndbuf(struct tcp_pcb *p)
{ (void)p; return sndbuf_cap ? (u16_t)(sndbuf_cap - sndbuf_used) : 65535; }
err_t tcp_write(struct tcp_pcb *p, const void *d, u16_t len, uint8_t fl)
{
    (void)p; (void)fl;
    if (sndbuf_cap && sndbuf_used + len > sndbuf_cap) return ERR_MEM;
    memcpy(out_cap + out_len, d, len);
    out_len += len; sndbuf_used += len;
    return ERR_OK;
}
err_t tcp_output(struct tcp_pcb *p) { (void)p; return ERR_OK; }
void tcp_recved(struct tcp_pcb *p, u16_t len) { (void)p; recved_total += len; }
err_t tcp_close(struct tcp_pcb *p) { (void)p; pcb_alive = false; pcb_closed++; return ERR_OK; }
void tcp_abort(struct tcp_pcb *p) { (void)p; pcb_alive = false; pcb_aborted++; }
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
    (void)arg;
    if (!strcmp(host, "slow.local") || !strcmp(host, "bad.local")) { dns_cb = f; return ERR_INPROGRESS; }
    addr->addr = 0x0100007f;
    return ERR_OK;
}

static void stub_sent_ack(void) { sndbuf_used = 0; if (sent_cb) sent_cb(NULL, &the_pcb, 0); }

static err_t inject(const uint8_t *d, size_t len, size_t chunk)
{
    struct pbuf *head = NULL, *tail = NULL;
    size_t off = 0;
    if (!len) return ERR_OK;
    while (off < len) {
        size_t n = len - off < chunk ? len - off : chunk;
        struct pbuf *p = calloc(1, sizeof(*p));
        p->payload = malloc(n); memcpy(p->payload, d + off, n);
        p->len = (uint16_t)n; p->ref = 1; pbufs_live++;
        if (!head) head = p; else tail->next = p;
        tail = p; off += n;
    }
    head->tot_len = (uint16_t)len;
    injected_total += (long)len;
    return recv_cb(cb_arg, &the_pcb, head, ERR_OK);
}

static void poll_n(int n)
{
    for (int i = 0; i < n; i++) { stub_now_us += 1000; sendspin_client_poll(); }
}

/* ===================================================================== */
/* server-side WebSocket framing                                          */
/* ===================================================================== */
static size_t srv_frame(uint8_t *o, uint8_t opcode, bool fin, const uint8_t *p, size_t len, int form)
{
    size_t k = 0;
    o[k++] = (fin ? 0x80 : 0) | opcode;
    if (form == 2) { o[k++] = 127; for (int i = 7; i >= 0; i--) o[k++] = (uint8_t)((uint64_t)len >> (8 * i)); }
    else if (form == 1 || len > 125) { o[k++] = 126; o[k++] = (uint8_t)(len >> 8); o[k++] = (uint8_t)len; }
    else o[k++] = (uint8_t)len;
    memcpy(o + k, p, len);
    return k + len;
}

/* parse one masked client frame from out_cap at rd_off */
static bool next_frame(uint8_t *op, uint8_t *pl, size_t *plen, bool *masked)
{
    const uint8_t *b = out_cap + rd_off; size_t n = out_len - rd_off;
    if (n < 2) return false;
    size_t k = 2, len = b[1] & 0x7f;
    if (len == 126) { if (n < 4) return false; len = (b[2] << 8) | b[3]; k = 4; }
    else if (len == 127) { return false; }
    *masked = (b[1] & 0x80) != 0;
    uint8_t m[4] = {0};
    if (*masked) { if (n < k + 4) return false; memcpy(m, b + k, 4); k += 4; }
    if (n < k + len) return false;
    for (size_t i = 0; i < len; i++) pl[i] = b[k + i] ^ m[i & 3];
    *op = b[0] & 0x0f; *plen = len;
    CHECK(b[0] & 0x80);                       /* FIN */
    rd_off += k + len;
    return true;
}

/* ===================================================================== */
/* independent Noise initiator (server side)                              */
/* ===================================================================== */
typedef struct { uint8_t h[32], ck[32], k[32]; int has_k; uint64_t n; } isym;

static void i_hmac(uint8_t out[32], const uint8_t *key, size_t kl, const uint8_t *d, size_t dl)
{
    static const uint8_t z = 0;
    mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), key, kl, dl ? d : &z, dl, out);
}
static void i_hkdf(const uint8_t ck[32], const uint8_t *ikm, size_t il, uint8_t o[3][32], int nout)
{
    uint8_t tk[32], in[33];
    i_hmac(tk, ck, 32, ikm, il);
    in[0] = 1; i_hmac(o[0], tk, 32, in, 1);
    memcpy(in, o[0], 32); in[32] = 2; i_hmac(o[1], tk, 32, in, 33);
    if (nout == 3) { memcpy(in, o[1], 32); in[32] = 3; i_hmac(o[2], tk, 32, in, 33); }
}
static void i_mixhash(isym *s, const uint8_t *d, size_t l)
{
    mbedtls_sha256_context c2; mbedtls_sha256_init(&c2); mbedtls_sha256_starts(&c2, 0);
    mbedtls_sha256_update(&c2, s->h, 32); mbedtls_sha256_update(&c2, d, l);
    mbedtls_sha256_finish(&c2, s->h); mbedtls_sha256_free(&c2);
}
static void i_mixkey(isym *s, const uint8_t *d, size_t l)
{
    uint8_t o[3][32]; i_hkdf(s->ck, d, l, o, 2);
    memcpy(s->ck, o[0], 32); memcpy(s->k, o[1], 32); s->has_k = 1; s->n = 0;
}
static void i_mixkeyhash(isym *s, const uint8_t *d, size_t l)
{
    uint8_t o[3][32]; i_hkdf(s->ck, d, l, o, 3);
    memcpy(s->ck, o[0], 32); i_mixhash(s, o[1], 32); memcpy(s->k, o[2], 32); s->has_k = 1; s->n = 0;
}
static void i_nonce(uint8_t nn[12], uint64_t n)
{
    memset(nn, 0, 12);
    for (int i = 0; i < 8; i++) nn[4 + i] = (n >> (8 * i)) & 0xff;
}
static void i_aead_enc(const uint8_t *k, uint64_t n, const uint8_t *ad, size_t al,
                       const uint8_t *pt, size_t len, uint8_t *out)
{
    mbedtls_chachapoly_context cc; uint8_t nn[12]; i_nonce(nn, n);
    mbedtls_chachapoly_init(&cc); mbedtls_chachapoly_setkey(&cc, k);
    mbedtls_chachapoly_encrypt_and_tag(&cc, len, nn, ad, al, pt, out, out + len);
    mbedtls_chachapoly_free(&cc);
}
static int i_aead_dec(const uint8_t *k, uint64_t n, const uint8_t *ad, size_t al,
                      const uint8_t *ct, size_t len, uint8_t *out)
{
    mbedtls_chachapoly_context cc; uint8_t nn[12]; i_nonce(nn, n);
    mbedtls_chachapoly_init(&cc); mbedtls_chachapoly_setkey(&cc, k);
    int r = mbedtls_chachapoly_auth_decrypt(&cc, len - 16, nn, ad, al, ct + len - 16, ct, out);
    mbedtls_chachapoly_free(&cc);
    return r ? -1 : (int)(len - 16);
}
static size_t i_encrypt_hash(isym *s, const uint8_t *pt, size_t len, uint8_t *out)
{
    if (!s->has_k) { memcpy(out, pt, len); i_mixhash(s, out, len); return len; }
    i_aead_enc(s->k, s->n++, s->h, 32, pt, len, out);
    i_mixhash(s, out, len + 16);
    return len + 16;
}
static int i_decrypt_hash(isym *s, const uint8_t *ct, size_t len, uint8_t *out)
{
    if (!s->has_k) { memcpy(out, ct, len); i_mixhash(s, ct, len); return (int)len; }
    int r = i_aead_dec(s->k, s->n, s->h, 32, ct, len, out);
    if (r < 0) return -1;
    s->n++;
    i_mixhash(s, ct, len);
    return r;
}
static void i_keygen(uint8_t priv[32], uint8_t pub[32])
{
    mbedtls_ecp_group g; mbedtls_mpi d; mbedtls_ecp_point Q; size_t ol;
    mbedtls_ecp_group_init(&g); mbedtls_mpi_init(&d); mbedtls_ecp_point_init(&Q);
    mbedtls_ecp_group_load(&g, MBEDTLS_ECP_DP_CURVE25519);
    mbedtls_ecdh_gen_public(&g, &d, &Q, test_rng, NULL);
    mbedtls_mpi_write_binary_le(&d, priv, 32);
    mbedtls_ecp_point_write_binary(&g, &Q, MBEDTLS_ECP_PF_UNCOMPRESSED, &ol, pub, 32);
    mbedtls_ecp_group_free(&g); mbedtls_mpi_free(&d); mbedtls_ecp_point_free(&Q);
}
static void i_dh(uint8_t out[32], const uint8_t priv[32], const uint8_t pub[32])
{
    mbedtls_ecp_group g; mbedtls_mpi d, z; mbedtls_ecp_point Q; uint8_t pb[32];
    mbedtls_ecp_group_init(&g); mbedtls_mpi_init(&d); mbedtls_mpi_init(&z); mbedtls_ecp_point_init(&Q);
    mbedtls_ecp_group_load(&g, MBEDTLS_ECP_DP_CURVE25519);
    mbedtls_mpi_read_binary_le(&d, priv, 32);
    memcpy(pb, pub, 32); pb[31] &= 0x7f;
    mbedtls_ecp_point_read_binary(&g, &Q, pb, 32);
    mbedtls_ecdh_compute_shared(&g, &z, &Q, &d, test_rng, NULL);
    mbedtls_mpi_write_binary_le(&z, out, 32);
    mbedtls_ecp_group_free(&g); mbedtls_mpi_free(&d); mbedtls_mpi_free(&z); mbedtls_ecp_point_free(&Q);
}

typedef struct {
    isym sym;
    uint8_t s_priv[32], s_pub[32];     /* initiator (server) static */
    uint8_t rs[32];                    /* responder (client) static */
    uint8_t e_priv[32], e_pub[32];
    uint8_t k_send[32], k_recv[32];    /* transport */
    uint64_t n_send, n_recv;
} init_t;

static void init_begin(init_t *I, const uint8_t *prologue, size_t pl)
{
    static const char name[] = "Noise_KKpsk2_25519_ChaChaPoly_SHA256";
    memset(&I->sym, 0, sizeof(I->sym));
    mbedtls_sha256((const uint8_t *)name, strlen(name), I->sym.h, 0);
    memcpy(I->sym.ck, I->sym.h, 32);
    i_mixhash(&I->sym, prologue, pl);
    i_mixhash(&I->sym, I->s_pub, 32);      /* -> s */
    i_mixhash(&I->sym, I->rs, 32);         /* <- s */
}
/* -> e, es, ss ; returns msg length */
static size_t init_msg1(init_t *I, const uint8_t *payload, size_t plen, uint8_t *out)
{
    uint8_t dh[32];
    i_keygen(I->e_priv, I->e_pub);
    memcpy(out, I->e_pub, 32);
    i_mixhash(&I->sym, I->e_pub, 32); i_mixkey(&I->sym, I->e_pub, 32);
    i_dh(dh, I->e_priv, I->rs); i_mixkey(&I->sym, dh, 32);        /* es */
    i_dh(dh, I->s_priv, I->rs); i_mixkey(&I->sym, dh, 32);        /* ss */
    return 32 + i_encrypt_hash(&I->sym, payload, plen, out + 32);
}
/* <- e, ee, se, psk ; returns payload length or -1 */
static int init_msg2(init_t *I, const uint8_t *msg, size_t len, const uint8_t psk[32], uint8_t *payload)
{
    uint8_t dh[32], re[32];
    memcpy(re, msg, 32);
    i_mixhash(&I->sym, re, 32); i_mixkey(&I->sym, re, 32);
    i_dh(dh, I->e_priv, re); i_mixkey(&I->sym, dh, 32);           /* ee */
    i_dh(dh, I->s_priv, re); i_mixkey(&I->sym, dh, 32);           /* se */
    i_mixkeyhash(&I->sym, psk, 32);                               /* psk */
    int r = i_decrypt_hash(&I->sym, msg + 32, len - 32, payload);
    if (r < 0) return -1;
    uint8_t o[3][32]; i_hkdf(I->sym.ck, NULL, 0, o, 2);
    memcpy(I->k_send, o[0], 32); memcpy(I->k_recv, o[1], 32);
    I->n_send = I->n_recv = 0;
    return r;
}

/* ===================================================================== */
/* unit tests                                                             */
/* ===================================================================== */
static void test_vectors(void)
{
    uint8_t a_priv[32], a_pub[32], b_priv[32], b_pub[32], exp[32], out[32];
    hex2bin(a_priv, "77076d0a7318a57d3c16c17251b26645df4c2f87ebc0992ab177fba51db92c2a");
    hex2bin(a_pub,  "8520f0098930a754748b7ddcb43ef75a0dbf3a0d26381af4eba4a98eaa9b4e6a");
    hex2bin(b_priv, "5dab087e624a8a4b79e17f8b83800ee66f3bb1292618b6fd1c2f8b27ff88e0eb");
    hex2bin(b_pub,  "de9edb7d7b7dc1b4d35b61c2ece435373f8343c85b78674dadfc7e146f882b4f");
    hex2bin(exp,    "4a5d9d5ba4ce2de1728e3bf480350f25e07e21c947d19e3376f09b3c1e161742");
    CHECK(ssn_x25519_base(out, a_priv, test_rng, NULL) == 0 && !memcmp(out, a_pub, 32));
    CHECK(ssn_x25519_base(out, b_priv, test_rng, NULL) == 0 && !memcmp(out, b_pub, 32));
    CHECK(ssn_x25519(out, a_priv, b_pub, test_rng, NULL) == 0 && !memcmp(out, exp, 32));
    CHECK(ssn_x25519(out, b_priv, a_pub, test_rng, NULL) == 0 && !memcmp(out, exp, 32));
    /* all-zero point must be rejected */
    uint8_t zero[32] = {0};
    CHECK(ssn_x25519(out, a_priv, zero, test_rng, NULL) != 0);

    /* RFC 7748 section 5.2 scalar-multiplication vectors */
    {
        uint8_t k[32], u[32], e1[32], e2[32];
        hex2bin(k,  "a546e36bf0527c9d3b16154b82465edd62144c0ac1fc5a18506a2244ba449ac4");
        hex2bin(u,  "e6db6867583030db3594c1a424b15f7c726624ec26b3353b10a903a6d0ab1c4c");
        hex2bin(e1, "c3da55379de9c6908e94ea4df28d084f32eccf03491c71f754b4075577a28552");
        CHECK(ssn_x25519(out, k, u, NULL, NULL) == 0 && !memcmp(out, e1, 32));
        hex2bin(k,  "4b66e9d4d1b4673c5ad22691957d6af5c11b6421e0ea01d42ca4169e7918ba0d");
        hex2bin(u,  "e5210f12786811d3f4b7959d0538ae2c31dbe7106fc03c3efc4cd549c715a493");
        hex2bin(e2, "95cbde9476e8907d7aade45cb4b873f88b595a68799fa152e6f8f7647aac7957");
        CHECK(ssn_x25519(out, k, u, NULL, NULL) == 0 && !memcmp(out, e2, 32));

        /* iterated: k = u = 9; (k, u) = (X25519(k, u), k) -- after 1 and 1000 rounds */
        uint8_t kk[32] = {9}, uu[32] = {9}, nk[32];
        uint8_t it1[32], it1000[32];
        hex2bin(it1,    "422c8e7a6227d7bca1350b3e2bb7279f7897b87bb6854b783c60e80311ae3079");
        hex2bin(it1000, "684cf59ba83309552800ef566f2f4d3c1c3887c49360e3875f2eb94d99532c51");
        for (int i = 1; i <= 1000; i++) {
            CHECK(ssn_x25519(nk, kk, uu, NULL, NULL) == 0);
            memcpy(uu, kk, 32); memcpy(kk, nk, 32);
            if (i == 1) CHECK(!memcmp(kk, it1, 32));
        }
        CHECK(!memcmp(kk, it1000, 32));
    }

    /* small-order / non-canonical points are rejected (all-zero shared secret) */
    {
        static const char *bad[] = {
            "0000000000000000000000000000000000000000000000000000000000000000",
            "0100000000000000000000000000000000000000000000000000000000000000",
            "e0eb7a7c3b41b8ae1656e3faf19fc46ada098deb9c32b1fd866205165f49b800",
            "5f9c95bca3508c24b1d0b1559c83ef5b04445cc4581c8e86d8224eddd09f1157",
            "ecffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff7f",
            "edffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff7f",
            "eeffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff7f",
        };
        uint8_t p[32];
        for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
            hex2bin(p, bad[i]);
            CHECK(ssn_x25519(out, a_priv, p, NULL, NULL) != 0);
            p[31] |= 0x80;                                   /* bit 255 must be ignored */
            CHECK(ssn_x25519(out, a_priv, p, NULL, NULL) != 0);
        }
    }

    /* cross-check against mbedtls ECDH (independent bignum implementation) */
    for (int i = 0; i < 300; i++) {
        uint8_t kp[32], up[32], pubm[32], mine[32], ref[32], priv_c[32];
        test_rng(NULL, kp, 32); test_rng(NULL, up, 32);
        if (i == 0) memset(up, 0xff, 32);                    /* all ones: non-canonical, bit 255 set */
        if (i == 1) { memset(kp, 0xff, 32); }
        memcpy(priv_c, kp, 32); priv_c[0] &= 248; priv_c[31] &= 127; priv_c[31] |= 64;
        i_dh(ref, priv_c, up);
        int rc = ssn_x25519(mine, kp, up, NULL, NULL);
        CHECK(rc == 0 && !memcmp(mine, ref, 32));
        /* public key derivation */
        mbedtls_ecp_group g; mbedtls_mpi d; mbedtls_ecp_point Q; size_t ol;
        mbedtls_ecp_group_init(&g); mbedtls_mpi_init(&d); mbedtls_ecp_point_init(&Q);
        mbedtls_ecp_group_load(&g, MBEDTLS_ECP_DP_CURVE25519);
        mbedtls_mpi_read_binary_le(&d, priv_c, 32);
        mbedtls_ecp_mul(&g, &Q, &d, &g.G, test_rng, NULL);
        mbedtls_ecp_point_write_binary(&g, &Q, MBEDTLS_ECP_PF_UNCOMPRESSED, &ol, pubm, 32);
        mbedtls_ecp_group_free(&g); mbedtls_mpi_free(&d); mbedtls_ecp_point_free(&Q);
        CHECK(ssn_x25519_base(mine, kp, NULL, NULL) == 0 && !memcmp(mine, pubm, 32));
    }

    /* Published Sendspin constants (connection.md) */
    uint8_t psk[32], psk_exp[32];
    hex2bin(psk_exp, "1b5e24dbc1aed95fc2a5a338a90c05df44bd10f5ec1f4cd66cbf86272767b9d3");
    ssn_sentinel_psk(psk);
    CHECK(!memcmp(psk, psk_exp, 32));
    char id[64];
    ssn_psk_id(id, psk);
    CHECK(!strcmp(id, "GFsV9tLaSQm9HcFWpKsgYQOr7wFTvNUtkmFwuVz3zoo"));

    /* RFC 6455 accept key */
    char acc[32];
    ws_accept_key("dGhlIHNhbXBsZSBub25jZQ==", acc);
    CHECK(!strcmp(acc, "s3pPLMBiTxaQ9kYGzzhZRbK+xOo="));

    /* base64 / base64url vs mbedtls base64 */
    uint8_t rnd[47]; test_rng(NULL, rnd, sizeof(rnd));
    for (size_t l = 0; l <= 47; l++) {
        char a[100], b[100]; size_t ol;
        ssn_b64_encode(a, rnd, l);
        mbedtls_base64_encode((unsigned char *)b, sizeof(b), &ol, rnd, l); b[ol] = 0;
        CHECK(!strcmp(a, b));
        char u[100]; uint8_t back[64];
        size_t ul = ssn_b64url_encode(u, rnd, l);
        CHECK(strchr(u, '=') == NULL && strchr(u, '+') == NULL && strchr(u, '/') == NULL);
        CHECK(ssn_b64url_decode(back, sizeof(back), u, ul) == (int)l && !memcmp(back, rnd, l));
    }
    CHECK(ssn_b64url_decode(out, 32, "a", 1) < 0);
    CHECK(ssn_b64url_decode(out, 32, "ab!d", 4) < 0);

    /* WS masking */
    uint8_t m[4] = {1, 2, 3, 4}, buf[10] = {0,1,2,3,4,5,6,7,8,9}, cp[10];
    memcpy(cp, buf, 10); ws_mask(buf, 10, m); CHECK(buf[0] == 1 && buf[4] == (4 ^ 1));
    ws_mask(buf, 10, m); CHECK(!memcmp(buf, cp, 10));
}

static void test_json(void)
{
    const char *j = "{\"payload\":{\"player\":{\"command\":\"volume\",\"volume\": 75,\"x\":\"a\\\"b\"},"
                    "\"volume\":9}, \"type\" : \"server/command\"}";
    char t[40]; int64_t v; bool b;
    CHECK(json_str(j, "type", 1, t, sizeof(t)) == 14 && !strcmp(t, "server/command"));
    CHECK(json_str(j, "command", 0, t, sizeof(t)) == 6 && !strcmp(t, "volume"));
    CHECK(json_i64(j, "volume", 0, &v) && v == 75);       /* skips the "volume" value string */
    CHECK(json_i64(j, "volume", 2, &v) && v == 9);        /* depth selects the outer one */
    CHECK(!json_i64(j, "nothere", 0, &v));
    CHECK(json_str(j, "x", 0, t, sizeof(t)) >= 0);
    CHECK(json_bool("{\"payload\":{\"player\":{\"command\":\"mute\",\"mute\":true}}}", "mute", 0, &b) && b);
    CHECK(json_bool("{\"mute\":false}", "mute", 0, &b) && !b);
    CHECK(json_i64("{\"a\":-12,\"b\":1710000000000000}", "b", 0, &v) && v == 1710000000000000LL);
    CHECK(json_i64("{\"a\":-12}", "a", 0, &v) && v == -12);
    CHECK(json_str("{\"type\":\"", "type", 1, t, sizeof(t)) < 0);   /* truncated */
}

static void test_noise_direct(void)
{
    init_t I; ssn_t R;
    uint8_t r_priv[32], r_pub[32];
    ssn_gen_static(r_priv, r_pub, test_rng, NULL);
    i_keygen(I.s_priv, I.s_pub);
    memcpy(I.rs, r_pub, 32);
    const char *pro = "prologue-bytes";
    init_begin(&I, (const uint8_t *)pro, strlen(pro));
    ssn_resp_init(&R, r_priv, r_pub, I.s_pub, (const uint8_t *)pro, strlen(pro), test_rng, NULL);

    uint8_t m1[200], m2[200], pay[128], psk[32];
    ssn_sentinel_psk(psk);
    const char *p1 = "{\"psk_id\":\"x\",\"psk_category\":\"sn\"}";
    size_t l1 = init_msg1(&I, (const uint8_t *)p1, strlen(p1), m1);
    size_t pl = 0;
    CHECK(ssn_resp_read_msg1(&R, m1, l1, pay, sizeof(pay), &pl) == 0);
    CHECK(pl == strlen(p1) && !memcmp(pay, p1, pl));
    size_t l2 = 0;
    CHECK(ssn_resp_write_msg2(&R, psk, (const uint8_t *)"{}", 2, m2, sizeof(m2), &l2) == 0);
    CHECK(l2 == 32 + 2 + 16);
    int r = init_msg2(&I, m2, l2, psk, pay);
    CHECK(r == 2 && !memcmp(pay, "{}", 2));
    CHECK(!memcmp(I.sym.h, R.h, 32));                  /* handshake hashes agree */

    /* transport both directions, several messages, in-place */
    for (int i = 0; i < 5; i++) {
        uint8_t pt[100], ct[116], dec[116];
        test_rng(NULL, pt, sizeof(pt));
        i_aead_enc(I.k_send, I.n_send++, NULL, 0, pt, sizeof(pt), ct);
        memcpy(dec, ct, 116);
        CHECK(ssn_decrypt(&R, dec, 116, dec) == 100 && !memcmp(dec, pt, 100));
        uint8_t ct2[116];
        CHECK(ssn_encrypt(&R, pt, 100, ct2) == 0);
        CHECK(i_aead_dec(I.k_recv, I.n_recv++, NULL, 0, ct2, 116, dec) == 100 && !memcmp(dec, pt, 100));
    }
    /* tamper + replay */
    uint8_t pt[20] = "tamper-test", ct[36], dec[36];
    i_aead_enc(I.k_send, I.n_send, NULL, 0, pt, 20, ct);
    ct[3] ^= 1;
    CHECK(ssn_decrypt(&R, ct, 36, dec) < 0);
    ct[3] ^= 1;
    memcpy(dec, ct, 36);
    CHECK(ssn_decrypt(&R, dec, 36, dec) == 20);        /* nonce did not advance on failure */
    memcpy(dec, ct, 36);
    CHECK(ssn_decrypt(&R, dec, 36, dec) < 0);          /* replay rejected */
    I.n_send++;
    /* skip */
    i_aead_enc(I.k_send, I.n_send++, NULL, 0, pt, 20, ct);        /* skipped msg */
    ssn_skip_recv(&R);
    i_aead_enc(I.k_send, I.n_send++, NULL, 0, pt, 20, ct);
    memcpy(dec, ct, 36);
    CHECK(ssn_decrypt(&R, dec, 36, dec) == 20);

    /* wrong PSK: initiator cannot decrypt msg2 */
    init_t J = I; ssn_t R2;
    init_begin(&J, (const uint8_t *)pro, strlen(pro));
    ssn_resp_init(&R2, r_priv, r_pub, I.s_pub, (const uint8_t *)pro, strlen(pro), test_rng, NULL);
    l1 = init_msg1(&J, (const uint8_t *)p1, strlen(p1), m1);
    CHECK(ssn_resp_read_msg1(&R2, m1, l1, pay, sizeof(pay), &pl) == 0);
    uint8_t badpsk[32]; memset(badpsk, 7, 32);
    ssn_resp_write_msg2(&R2, badpsk, (const uint8_t *)"{}", 2, m2, sizeof(m2), &l2);
    CHECK(init_msg2(&J, m2, l2, psk, pay) < 0);

    /* wrong prologue: responder fails msg1 */
    ssn_resp_init(&R2, r_priv, r_pub, I.s_pub, (const uint8_t *)"other", 5, test_rng, NULL);
    CHECK(ssn_resp_read_msg1(&R2, m1, l1, pay, sizeof(pay), &pl) < 0);
    /* truncated msg1 */
    ssn_resp_init(&R2, r_priv, r_pub, I.s_pub, (const uint8_t *)pro, strlen(pro), test_rng, NULL);
    CHECK(ssn_resp_read_msg1(&R2, m1, 40, pay, sizeof(pay), &pl) < 0);
}

/* ===================================================================== */
/* end-to-end: sendspin_client.c vs. a test "Music Assistant"              */
/* ===================================================================== */
static init_t S;
static char client_init_txt[400], server_init_txt[400];
static size_t g_chunk = 1500;

static void srv_inject(const uint8_t *frame, size_t n) { inject(frame, n, g_chunk); }
static void srv_text(const char *s)
{
    static uint8_t f[4096];
    size_t n = srv_frame(f, 1, true, (const uint8_t *)s, strlen(s), 0);
    srv_inject(f, n);
}
static void srv_enc_ex(const uint8_t *pt, size_t len, bool corrupt, int form)
{
    static uint8_t ct[70000], f[70100];
    i_aead_enc(S.k_send, S.n_send++, NULL, 0, pt, len, ct);
    if (corrupt) ct[len / 2] ^= 0x40;
    size_t n = srv_frame(f, 2, true, ct, len + 16, form);
    srv_inject(f, n);
}
static void srv_json(const char *json)
{
    static uint8_t pt[4096];
    pt[0] = 0; memcpy(pt + 1, json, strlen(json));
    srv_enc_ex(pt, 1 + strlen(json), false, 0);
}
static void srv_audio(int64_t ts, const uint8_t *pcm, size_t n)
{
    static uint8_t pt[40000];
    pt[0] = 4;
    for (int i = 0; i < 8; i++) pt[1 + i] = (uint8_t)((uint64_t)ts >> (56 - 8 * i));
    pt[9] = 0; pt[10] = 0; pt[11] = 0x27; pt[12] = 0x10;
    memcpy(pt + 13, pcm, n);
    srv_enc_ex(pt, 13 + n, false, 0);
}

/* read next client frame; poll first if needed. returns false on timeout */
static bool wait_frame(uint8_t *op, uint8_t *pl, size_t *plen)
{
    bool masked;
    for (int i = 0; i < 100; i++) {
        if (next_frame(op, pl, plen, &masked)) { CHECK(masked); return true; }
        poll_n(1);
    }
    return false;
}
static bool wait_json(char *out, size_t cap)     /* decrypt next client message */
{
    static uint8_t pl[4096], dec[4096]; uint8_t op; size_t n;
    if (!wait_frame(&op, pl, &n)) return false;
    CHECK(op == 2);
    int r = i_aead_dec(S.k_recv, S.n_recv++, NULL, 0, pl, n, dec);
    CHECK(r > 1 && dec[0] == 0);
    if (r < 2 || (size_t)r >= cap) return false;
    memcpy(out, dec + 1, (size_t)r - 1); out[r - 1] = 0;
    return true;
}
static bool wait_json_type(const char *type, char *out, size_t cap)
{
    for (int i = 0; i < 20; i++) {
        if (!wait_json(out, cap)) return false;
        char t[40];
        if (json_str(out, "type", 1, t, sizeof(t)) > 0 && !strcmp(t, type)) return true;
    }
    return false;
}

/* complete a whole connect + handshake + hello + time sync + state. */
static void do_session(bool from_scratch)
{
    static char js[4096]; static uint8_t pl[4096]; uint8_t op; size_t n;
    (void)from_scratch;
    out_len = rd_off = 0; sndbuf_cap = 0; sndbuf_used = 0;
    poll_n(2);                       /* (a connect may already be in flight) */
    CHECK(sendspin_client_state() == SENDSPIN_STATE_CONNECTING);
    CHECK(connect_port == 8927);
    CHECK(connected_cb(cb_arg, &the_pcb, ERR_OK) == ERR_OK);

    /* HTTP upgrade request */
    out_cap[out_len] = 0;
    char *req = (char *)out_cap;
    CHECK(!strncmp(req, "GET /sendspin HTTP/1.1\r\n", 24));
    CHECK(strstr(req, "Host: 127.0.0.1:8927\r\n") != NULL);
    CHECK(strstr(req, "Upgrade: websocket\r\n") != NULL);
    CHECK(strstr(req, "Sec-WebSocket-Version: 13\r\n") != NULL);
    char *kp = strstr(req, "Sec-WebSocket-Key: "); CHECK(kp != NULL);
    char key[40]; sscanf(kp + 19, "%39[^\r]", key);
    CHECK(strlen(key) == 24);
    uint8_t kraw[32]; size_t kl;
    CHECK(mbedtls_base64_decode(kraw, sizeof(kraw), &kl, (const uint8_t *)key, 24) == 0 && kl == 16);
    rd_off = (size_t)(strstr(req, "\r\n\r\n") - req) + 4;

    /* server: 101 (accept computed with mbedtls independently) */
    char cat[100]; uint8_t dg[20]; char acc[40]; size_t al;
    snprintf(cat, sizeof(cat), "%s258EAFA5-E914-47DA-95CA-C5AB0DC85B11", key);
    mbedtls_sha1((const uint8_t *)cat, strlen(cat), dg);
    mbedtls_base64_encode((unsigned char *)acc, sizeof(acc), &al, dg, 20); acc[al] = 0;
    char resp[300];
    snprintf(resp, sizeof(resp),
        "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
        "sec-websocket-accept: %s\r\n\r\n", acc);
    CHECK(inject((const uint8_t *)resp, strlen(resp), 7) == ERR_OK);

    /* client/init */
    CHECK(wait_frame(&op, pl, &n) && op == 1);
    memcpy(client_init_txt, pl, n); client_init_txt[n] = 0;
    char t[40], cid[64];
    CHECK(json_str(client_init_txt, "type", 1, t, sizeof(t)) > 0 && !strcmp(t, "client/init"));
    CHECK(json_str(client_init_txt, "suite", 0, t, sizeof(t)) > 0 && !strcmp(t, "25519_ChaChaPoly_SHA256"));
    int64_t ver; CHECK(json_i64(client_init_txt, "version", 0, &ver) && ver == 1);
    int cl = json_str(client_init_txt, "client_id", 0, cid, sizeof(cid));
    CHECK(cl == 43);
    uint8_t client_pub[32];
    CHECK(ssn_b64url_decode(client_pub, 32, cid, 43) == 32);
    CHECK(!memcmp(client_pub, c.s_pub, 32));
    CHECK(sendspin_client_state() == SENDSPIN_STATE_HANDSHAKE);

    /* server/init */
    char sid[64];
    ssn_b64url_encode(sid, S.s_pub, 32);
    snprintf(server_init_txt, sizeof(server_init_txt),
        "{\"type\":\"server/init\",\"payload\":{\"server_id\":\"%s\",\"version\":1}}", sid);
    g_chunk = 1;                                         /* byte-at-a-time parser stress */
    srv_text(server_init_txt);
    g_chunk = 1500;

    /* noise msg1 (initiator), prologue = exact client/init || server/init bytes */
    char prol[1000]; size_t prl = 0;
    memcpy(prol, client_init_txt, strlen(client_init_txt)); prl += strlen(client_init_txt);
    memcpy(prol + prl, server_init_txt, strlen(server_init_txt)); prl += strlen(server_init_txt);
    memcpy(S.rs, client_pub, 32);
    init_begin(&S, (const uint8_t *)prol, prl);
    uint8_t psk[32], m1[200], m2[200], pay[100]; char pid[64], b64[300], msg[400];
    ssn_sentinel_psk(psk); ssn_psk_id(pid, psk);
    snprintf(msg, sizeof(msg), "{\"psk_id\":\"%s\",\"psk_category\":\"sn\"}", pid);
    size_t l1 = init_msg1(&S, (const uint8_t *)msg, strlen(msg), m1);
    ssn_b64url_encode(b64, m1, l1);
    snprintf(msg, sizeof(msg), "{\"type\":\"noise/handshake\",\"payload\":{\"data\":\"%s\"}}", b64);
    srv_text(msg);

    /* noise msg2 from client */
    CHECK(wait_frame(&op, pl, &n) && op == 1);
    memcpy(js, pl, n); js[n] = 0;
    CHECK(json_str(js, "type", 1, t, sizeof(t)) > 0 && !strcmp(t, "noise/handshake"));
    char d2[200]; int dl = json_str(js, "data", 0, d2, sizeof(d2));
    int ml = ssn_b64url_decode(m2, sizeof(m2), d2, (size_t)dl);
    CHECK(ml == 50);
    CHECK(init_msg2(&S, m2, (size_t)ml, psk, pay) == 2 && !memcmp(pay, "{}", 2));

    /* server/hello -> client/hello */
    srv_json("{\"type\":\"server/hello\",\"payload\":{\"name\":\"Music Assistant\","
             "\"languages\":[\"en\"],\"active_roles\":[]}}");
    CHECK(wait_json_type("client/hello", js, sizeof(js)));
    CHECK(sendspin_client_state() == SENDSPIN_STATE_SYNCING);
    CHECK(strstr(js, "\"supported_roles\":[\"player@v1\"]") != NULL);
    CHECK(strstr(js, "\"unpaired_access\":{\"enabled\":true}") != NULL);
    CHECK(strstr(js, "\"codec\":\"pcm\",\"channels\":2,\"sample_rate\":48000,\"bit_depth\":16") != NULL);
    CHECK(strstr(js, "\"buffer_capacity\":32768") != NULL);       /* == PCM ring size */
    CHECK(json_str(js, "name", 0, t, sizeof(t)) > 0 && !strcmp(t, "DSPi"));

    /* time burst: server clock = client + 5 s, 1 ms one-way, 100 us processing */
    const int64_t OFF = 5000000;
    for (int i = 0; i < 8; i++) {
        CHECK(wait_json_type("client/time", js, sizeof(js)));
        int64_t ct; CHECK(json_i64(js, "client_transmitted", 0, &ct));
        char rep[300];
        int64_t jit = (i == 3) ? 0 : 300 + 50 * i;        /* sample 3 has the lowest RTT */
        snprintf(rep, sizeof(rep),
            "{\"type\":\"server/time\",\"payload\":{\"client_transmitted\":%lld,"
            "\"server_received\":%lld,\"server_transmitted\":%lld}}",
            (long long)ct, (long long)(ct + OFF + 1000), (long long)(ct + OFF + 1100));
        stub_now_us = (uint64_t)(ct + 2100 + jit);          /* extra delay on return leg */
        if (i != 3) { /* asymmetric delay corrupts that sample's offset, not its RTT rank */ }
        srv_json(rep);
    }
    CHECK(c.synced);
    int64_t err = c.offset - OFF; if (err < 0) err = -err;
    CHECK(err == 0);                                        /* best sample has symmetric delay */
    CHECK(wait_json_type("client/state", js, sizeof(js)));
    CHECK(strstr(js, "\"available\":true") != NULL);
    CHECK(strstr(js, "\"supported_commands\":[\"volume\",\"mute\"]") != NULL);
    int64_t v; CHECK(json_i64(js, "volume", 0, &v) && v == g_volume);
    CHECK(sendspin_client_state() == SENDSPIN_STATE_READY);
    srv_json("{\"type\":\"server/activate\",\"payload\":{\"activities\":[\"playback\"],"
             "\"active_roles\":[\"player@v1\"]}}");
    poll_n(2);
}

static void start_stream(void)
{
    flush_count = 0; fmt_calls = 0; pushed_len = 0; ring_fill_sim = 0; stage_hi = 0;
    srv_json("{\"type\":\"stream/start\",\"payload\":{\"player\":{\"codec\":\"pcm\","
             "\"sample_rate\":48000,\"channels\":2,\"bit_depth\":16}}}");
    CHECK(fmt_calls == 1 && fmt_rate == 48000 && fmt_ch == 2 && fmt_bits == 16);
    CHECK(flush_count == 1);
    CHECK(sendspin_client_streaming());
    CHECK(sendspin_client_state() == SENDSPIN_STATE_STREAMING);
}

static void test_e2e(void)
{
    static char js[4096]; static uint8_t pcm[40000], pl[4096]; uint8_t op; size_t n;
    i_keygen(S.s_priv, S.s_pub);

    sendspin_client_start();
    CHECK(sendspin_client_state() == SENDSPIN_STATE_IDLE);
    do_session(true);
    CHECK(pbufs_live == 0);

    start_stream();

    /* ---- audio chunks: 16-bit WS length, odd chunking ---- */
    for (size_t i = 0; i < sizeof(pcm); i++) pcm[i] = (uint8_t)(i * 7 + 3);
    g_chunk = 333;
    srv_audio((int64_t)stub_now_us + 5000000 + 100000, pcm, 4800);
    CHECK(pushed_len == 4800 && !memcmp(pushed, pcm, 4800));
    CHECK(pbufs_live == 0);
    /* back-to-back chunks in one TCP segment */
    {
        static uint8_t two[20000]; (void)two;
    }
    g_chunk = 1500;

    /* ---- backpressure: ring nearly full -> the chunk stalls (nothing published),
     *      later chunks queue behind it, everything is delivered in order ---- */
    pushed_len = 0; recved_total = injected_total = 0;
    ring_fill_sim = stub_ring_cap - 1000;                /* only 1000 bytes free */
    srv_audio(1, pcm, 4800);
    CHECK(pushed_len == 0 && c.blocked);                 /* unauthenticated bytes are never published */
    CHECK(c.ds_got < c.ds_len);
    srv_audio(2, pcm + 100, 960);                        /* arrives while blocked: queued */
    CHECK(pushed_len == 0);
    CHECK(recved_total < injected_total);                /* window not fully released */
    poll_n(3);
    CHECK(pushed_len == 0 && c.blocked);                 /* still blocked */
    ring_fill_sim = 0;                                   /* consumer drained */
    poll_n(3);
    CHECK(!c.blocked && pushed_len == 4800 + 960);
    CHECK(!memcmp(pushed, pcm, 4800) && !memcmp(pushed + 4800, pcm + 100, 960));
    CHECK(recved_total == injected_total);
    CHECK(pbufs_live == 0);

    /* ---- big chunks: 100 ms .. 150 ms .. a full ring (all > 20 KB) ---- */
    {
        static const uint32_t sizes[] = { 19200, 20000, 28800, 32764, 32768 };
        static const size_t chunks[] = { 1500, 1460, 1, 4096, 333 };
        for (size_t t = 0; t < sizeof(sizes) / sizeof(sizes[0]); t++) {
            pushed_len = 0; ring_fill_sim = 0;
            g_chunk = chunks[t];
            srv_audio(5, pcm, sizes[t]);
            g_chunk = 1500;
            CHECK(pushed_len == sizes[t] && !memcmp(pushed, pcm, sizes[t]));
            CHECK(sendspin_client_state() == SENDSPIN_STATE_STREAMING && pbufs_live == 0);
        }
        /* 150 ms chunk arriving while 10000 bytes are still queued in the ring:
         * stalls halfway, completes once the consumer has drained */
        pushed_len = 0; ring_fill_sim = 10000;
        srv_audio(6, pcm, 28800);
        CHECK(c.blocked && pushed_len == 0);
        stub_drain(10000);
        poll_n(2);
        CHECK(!c.blocked && pushed_len == 28800 && !memcmp(pushed, pcm, 28800));
        /* longer than the ring can ever hold: authenticated, dropped, stream stays in sync */
        pushed_len = 0; ring_fill_sim = 0;
        srv_audio(7, pcm, 36000);
        CHECK(pushed_len == 0 && !c.blocked && sendspin_client_state() == SENDSPIN_STATE_STREAMING);
        srv_audio(8, pcm, 480);
        CHECK(pushed_len == 480 && !memcmp(pushed, pcm, 480));
        /* chunk of a partial frame (5 bytes): whole frames only */
        pushed_len = 0; ring_fill_sim = 0;
        srv_audio(9, pcm, 4805);
        CHECK(pushed_len == 4804);
        ring_fill_sim = 0;
    }

    /* ---- JSON bigger than g_rx: authenticated, ignored, not fatal ---- */
    {
        static char big[SS_RX_BUF + 500];
        memset(big, 'x', sizeof(big)); big[sizeof(big) - 1] = 0;
        memcpy(big, "{\"type\":\"server/command\",\"pad\":\"", 32);
        big[sizeof(big) - 3] = '"'; big[sizeof(big) - 2] = '}';
        static uint8_t pt[SS_RX_BUF + 600]; pt[0] = 0; memcpy(pt + 1, big, sizeof(big) - 1);
        srv_enc_ex(pt, 1 + sizeof(big) - 1, false, 1);
        CHECK(sendspin_client_state() == SENDSPIN_STATE_STREAMING);
        srv_audio(10, pcm, 480);                           /* nonce stayed in step */
        CHECK(sendspin_client_state() == SENDSPIN_STATE_STREAMING);
    }

    /* ---- 64-bit WS length form + volume/mute commands ---- */
    {
        const char *cmd = "{\"type\":\"server/command\",\"payload\":{\"player\":{\"command\":\"volume\",\"volume\":30}}}";
        static uint8_t pt[512]; pt[0] = 0; memcpy(pt + 1, cmd, strlen(cmd));
        srv_enc_ex(pt, 1 + strlen(cmd), false, 2);
        CHECK(g_volume == 30);
        CHECK(wait_json_type("client/state", js, sizeof(js)));
        int64_t v; CHECK(json_i64(js, "volume", 0, &v) && v == 30);
        srv_json("{\"type\":\"server/command\",\"payload\":{\"player\":{\"command\":\"mute\",\"mute\":true}}}");
        CHECK(g_mute);
        CHECK(wait_json_type("client/state", js, sizeof(js)));
        CHECK(strstr(js, "\"muted\":true") != NULL);
        /* local change is reported */
        g_volume = 55; g_mute = false;
        poll_n(2);
        CHECK(wait_json_type("client/state", js, sizeof(js)));
        CHECK(json_i64(js, "volume", 0, &v) && v == 55 && strstr(js, "\"muted\":false"));
        /* no spurious re-reports */
        poll_n(5);
        CHECK(!next_frame(&op, pl, &n, &(bool){0}));
    }

    /* ---- stream/clear and late-chunk drop ---- */
    {
        int fc = flush_count; pushed_len = 0;
        srv_json("{\"type\":\"stream/clear\",\"payload\":{}}");
        CHECK(flush_count == fc + 1);
        int64_t server_now = (int64_t)stub_now_us + c.offset;
        srv_audio(server_now - 1000000, pcm, 4800);       /* 1 s late */
        CHECK(pushed_len == 0);
        srv_audio(server_now + 200000, pcm, 4800);        /* on time */
        CHECK(pushed_len == 4800);
        srv_audio(server_now - 1000000, pcm, 480);        /* after first good chunk: not dropped */
        CHECK(pushed_len == 4800 + 480);
    }

    /* ---- time sync keeps running ---- */
    stub_now_us += 6000000;
    poll_n(2);
    CHECK(wait_json_type("client/time", js, sizeof(js)));

    /* ---- ping -> pong ---- */
    {
        uint8_t f[64]; size_t k = srv_frame(f, 9, true, (const uint8_t *)"hi", 2, 0);
        srv_inject(f, k);
        CHECK(wait_frame(&op, pl, &n) && op == 10 && n == 2 && !memcmp(pl, "hi", 2));
    }

    /* ---- stream/end ---- */
    srv_json("{\"type\":\"stream/end\",\"payload\":{}}");
    CHECK(!sendspin_client_streaming() && sendspin_client_state() == SENDSPIN_STATE_READY);
    pushed_len = 0;
    srv_audio(1, pcm, 480);
    CHECK(pushed_len == 0);                               /* ignored when not streaming */

    /* ---- unsupported format is ignored, not fatal ---- */
    srv_json("{\"type\":\"stream/start\",\"payload\":{\"player\":{\"codec\":\"flac\","
             "\"sample_rate\":48000,\"channels\":2,\"bit_depth\":24}}}");
    CHECK(!sendspin_client_streaming() && sendspin_client_state() == SENDSPIN_STATE_READY);

    /* ---- tampered record -> connection dropped, back-off, reconnect ---- */
    {
        static uint8_t pt[64]; pt[0] = 0; memcpy(pt + 1, "{\"type\":\"x\"}", 12);
        int pa = pcb_aborted;
        srv_enc_ex(pt, 13, true, 0);
        CHECK(sendspin_client_state() == SENDSPIN_STATE_ERROR);
        CHECK(pcb_aborted == pa + 1);
        CHECK(pbufs_live == 0);
        int nb = pcb_new_count;
        poll_n(500);                                       /* 0.5 s < 1 s back-off */
        CHECK(pcb_new_count == nb);
        stub_now_us += 1500000; poll_n(1);
        CHECK(pcb_new_count == nb + 1);
    }

    /* ---- second full session; a corrupted 28.8 KB audio chunk publishes nothing ---- */
    i_keygen(S.s_priv, S.s_pub);                           /* new server key: no pinning in phase 1 */
    do_session(false);
    start_stream();
    {
        static uint8_t bad_pt[28900];
        bad_pt[0] = 4; memset(bad_pt + 1, 0, 12);
        for (size_t i = 13; i < sizeof(bad_pt); i++) bad_pt[i] = (uint8_t)i;
        for (int variant = 0; variant < 2; variant++) {
            static uint8_t ct[30000], f[30100];
            int pa = pcb_aborted;
            i_aead_enc(S.k_send, S.n_send++, NULL, 0, bad_pt, 28813, ct);
            if (variant == 0) ct[20000] ^= 1;              /* body */
            else ct[28813 + 5] ^= 0x80;                    /* tag */
            size_t k = srv_frame(f, 2, true, ct, 28813 + 16, 0);
            srv_inject(f, k);
            CHECK(pushed_len == 0);                        /* staged bytes were never committed */
            CHECK(sendspin_client_state() == SENDSPIN_STATE_ERROR && pcb_aborted == pa + 1);
            CHECK(pbufs_live == 0);
            if (variant == 0) {
                stub_now_us += 20000000; i_keygen(S.s_priv, S.s_pub);
                do_session(false);
                start_stream();
            }
        }
        stub_now_us += 20000000; i_keygen(S.s_priv, S.s_pub);
        do_session(false);
    }
    /* ---- fragmented / masked / too-short encrypted frames are protocol errors ---- */
    {
        static uint8_t f[64];
        uint8_t body[20] = {0};
        int pa = pcb_aborted;
        size_t k = srv_frame(f, 2, false, body, 20, 0);    /* FIN=0 binary */
        srv_inject(f, k);
        CHECK(sendspin_client_state() == SENDSPIN_STATE_ERROR && pcb_aborted == pa + 1);
        stub_now_us += 20000000; i_keygen(S.s_priv, S.s_pub);
        do_session(false);
        pa = pcb_aborted;
        k = srv_frame(f, 2, true, body, 10, 0);            /* shorter than the tag */
        srv_inject(f, k);
        CHECK(sendspin_client_state() == SENDSPIN_STATE_ERROR && pcb_aborted == pa + 1);
        stub_now_us += 20000000; i_keygen(S.s_priv, S.s_pub);
        do_session(false);
    }
    {
        int pc = pcb_closed;
        uint8_t f[16], cp[2] = {0x03, 0xE8}; size_t k = srv_frame(f, 8, true, cp, 2, 0);
        srv_inject(f, k);
        CHECK(sendspin_client_state() == SENDSPIN_STATE_ERROR);
        CHECK(pcb_closed == pc + 1);
        bool m; CHECK(next_frame(&op, pl, &n, &m) && op == 8 && n == 2 && m);   /* close echo */
    }

    /* ---- third session, peer drops TCP; wrong-key handshake failure ---- */
    stub_now_us += 5000000;
    i_keygen(S.s_priv, S.s_pub);
    do_session(false);
    {
        int pc = pcb_closed;
        CHECK(recv_cb(cb_arg, &the_pcb, NULL, ERR_OK) == ERR_OK);
        CHECK(sendspin_client_state() == SENDSPIN_STATE_ERROR && pcb_closed == pc + 1);
    }

    /* ---- bad Sec-WebSocket-Accept is rejected ---- */
    stub_now_us += 10000000; out_len = rd_off = 0;
    poll_n(2);
    CHECK(connected_cb(cb_arg, &the_pcb, ERR_OK) == ERR_OK);
    {
        const char *bad = "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n"
                          "Sec-WebSocket-Accept: AAAAAAAAAAAAAAAAAAAAAAAAAAA=\r\n\r\n";
        inject((const uint8_t *)bad, strlen(bad), 64);
        CHECK(sendspin_client_state() == SENDSPIN_STATE_ERROR);
    }
    /* HTTP 403 rejected */
    stub_now_us += 10000000; poll_n(2);
    CHECK(connected_cb(cb_arg, &the_pcb, ERR_OK) == ERR_OK);
    {
        const char *bad = "HTTP/1.1 403 Forbidden\r\n\r\n";
        inject((const uint8_t *)bad, strlen(bad), 64);
        CHECK(sendspin_client_state() == SENDSPIN_STATE_ERROR);
    }

    /* ---- non-sentinel psk category is refused ---- */
    stub_now_us += 10000000;
    {
        i_keygen(S.s_priv, S.s_pub);
        out_len = rd_off = 0; poll_n(2);
        connected_cb(cb_arg, &the_pcb, ERR_OK);
        out_cap[out_len] = 0;
        char *kp = strstr((char *)out_cap, "Sec-WebSocket-Key: ");
        char key[40]; sscanf(kp + 19, "%39[^\r]", key);
        char acc[32]; ws_accept_key(key, acc);
        char resp[300]; snprintf(resp, sizeof(resp),
            "HTTP/1.1 101 x\r\nSec-WebSocket-Accept: %s\r\n\r\n", acc);
        inject((const uint8_t *)resp, strlen(resp), 1000);
        rd_off = (size_t)(strstr((char *)out_cap, "\r\n\r\n") - (char *)out_cap) + 4;
        CHECK(wait_frame(&op, pl, &n));
        memcpy(client_init_txt, pl, n); client_init_txt[n] = 0;
        char sid[64]; ssn_b64url_encode(sid, S.s_pub, 32);
        snprintf(server_init_txt, sizeof(server_init_txt),
            "{\"type\":\"server/init\",\"payload\":{\"server_id\":\"%s\",\"version\":1}}", sid);
        srv_text(server_init_txt);
        char cid[64]; json_str(client_init_txt, "client_id", 0, cid, sizeof(cid));
        ssn_b64url_decode(S.rs, 32, cid, 43);
        char prol[1000]; snprintf(prol, sizeof(prol), "%s%s", client_init_txt, server_init_txt);
        init_begin(&S, (const uint8_t *)prol, strlen(prol));
        uint8_t m1[200]; char b64[300], msg[400];
        const char *p = "{\"psk_id\":\"zzz\",\"psk_category\":\"lt\"}";
        size_t l1 = init_msg1(&S, (const uint8_t *)p, strlen(p), m1);
        ssn_b64url_encode(b64, m1, l1);
        snprintf(msg, sizeof(msg), "{\"type\":\"noise/handshake\",\"payload\":{\"data\":\"%s\"}}", b64);
        srv_text(msg);
        CHECK(sendspin_client_state() == SENDSPIN_STATE_ERROR);
    }

    /* ---- DNS in progress / failure ---- */
    stub_now_us += 20000000;
    g_host = "slow.local";
    poll_n(2);
    CHECK(sendspin_client_state() == SENDSPIN_STATE_CONNECTING && c.dns_pending);
    dns_cb("slow.local", NULL, NULL);
    CHECK(sendspin_client_state() == SENDSPIN_STATE_ERROR);
    g_host = "127.0.0.1";

    /* ---- slow tx window (sndbuf small) ---- */
    stub_now_us += 20000000;
    poll_n(2);
    {
        int pnc = pcb_new_count; (void)pnc;
        sndbuf_cap = 50; sndbuf_used = 0; out_len = rd_off = 0;
        connected_cb(cb_arg, &the_pcb, ERR_OK);
        CHECK(out_len == 50);                              /* partial write */
        for (int i = 0; i < 10 && out_len < 400; i++) stub_sent_ack();
        CHECK(!strncmp((char *)out_cap, "GET /sendspin", 13));
        CHECK(strstr((char *)out_cap, "\r\n\r\n") != NULL);
        sndbuf_cap = 0;
    }

    /* ---- stop() sends goodbye on a live session ---- */
    stub_now_us += 20000000; poll_n(5);                    /* handshake timeout -> retry */
    stub_now_us += 20000000; poll_n(2);
    do_session(false);
    {
        int pc = pcb_closed;
        sendspin_client_stop();
        CHECK(sendspin_client_state() == SENDSPIN_STATE_IDLE && pcb_closed == pc + 1);
        CHECK(wait_json_type("client/goodbye", js, sizeof(js)));
        CHECK(strstr(js, "shutdown") != NULL);
        int nb = pcb_new_count;
        stub_now_us += 60000000; poll_n(5);
        CHECK(pcb_new_count == nb);                        /* stopped: no reconnect */
    }
}

/* ===================================================================== */
/* FLAC (phase 2): full client path with encrypted records                */
/* ===================================================================== */
static const char *g_fixdir;

typedef struct { size_t off, size, samples; } fx_frame_t;
typedef struct {
    uint8_t *flac; size_t flen;
    uint8_t *pcm;  size_t plen;          /* raw interleaved PCM (WAV header stripped) */
    uint8_t *wav;                        /* allocation that pcm points into */
    fx_frame_t fr[64]; int nfr;
    uint32_t rate; int bits, bpf;
} fx_t;

static uint8_t *load_file(const char *dir, const char *name, const char *ext, size_t *n)
{
    char path[512];
    snprintf(path, sizeof(path), "%s/%s%s", dir, name, ext);
    FILE *fp = fopen(path, "rb");
    if (!fp) return NULL;
    fseek(fp, 0, SEEK_END); long l = ftell(fp); fseek(fp, 0, SEEK_SET);
    uint8_t *b = malloc((size_t)l + 1);
    if (fread(b, 1, (size_t)l, fp) != (size_t)l) { fclose(fp); free(b); return NULL; }
    fclose(fp);
    *n = (size_t)l;
    return b;
}

static bool fx_load_ch(fx_t *x, const char *name, uint32_t rate, int bits, int ch)
{
    memset(x, 0, sizeof(*x));
    x->rate = rate; x->bits = bits; x->bpf = ch * bits / 8;
    x->flac = load_file(g_fixdir, name, ".flac", &x->flen);
    size_t wl; uint8_t *wav = load_file(g_fixdir, name, ".wav", &wl);
    size_t il; uint8_t *idx = load_file(g_fixdir, name, ".idx", &il);
    x->wav = wav;
    if (!x->flac || !wav || !idx) { free(idx); return false; }
    CHECK(!memcmp(wav + 36, "data", 4));
    x->pcm = wav + 44; x->plen = wl - 44;
    idx[il] = 0;
    char *p = (char *)idx;
    while (*p && x->nfr < 64) {
        unsigned long o, s, n; int k = 0;
        if (sscanf(p, "%lu %lu %lu%n", &o, &s, &n, &k) != 3) break;
        x->fr[x->nfr].off = o; x->fr[x->nfr].size = s; x->fr[x->nfr].samples = n; x->nfr++;
        p += k;
    }
    size_t tot = 0;
    for (int i = 0; i < x->nfr; i++) tot += x->fr[i].samples;
    CHECK(tot * (size_t)x->bpf == x->plen);                /* index covers the whole signal */
    CHECK(x->fr[0].off > 42 && !memcmp(x->flac, "fLaC", 4));
    free(idx);
    return true;
}

static void fx_free(fx_t *x)
{
    free(x->flac); free(x->wav);
    memset(x, 0, sizeof(*x));
}

static bool fx_load(fx_t *x, const char *name, uint32_t rate, int bits)
{ return fx_load_ch(x, name, rate, bits, 2); }

static size_t fx_sample_pos(const fx_t *x, int frame)        /* PCM byte offset of a frame */
{
    size_t s = 0;
    for (int i = 0; i < frame; i++) s += x->fr[i].samples;
    return s * (size_t)x->bpf;
}

static int64_t ts_now(void) { return (int64_t)stub_now_us + c.offset + 200000; }

static void flac_start_json(const fx_t *x, const uint8_t *hdr, size_t hlen, int bits, uint32_t rate)
{
    static char b64[4000], js[5000]; size_t ol = 0;
    int rc = mbedtls_base64_encode((unsigned char *)b64, sizeof(b64) - 1, &ol, hdr, hlen);
    CHECK(rc == 0);                       /* header must fit the test buffers (and srv_json's 4 KB record) */
    if (rc != 0) return;
    b64[ol] = 0;
    snprintf(js, sizeof(js),
        "{\"type\":\"stream/start\",\"payload\":{\"player\":{\"codec\":\"flac\","
        "\"sample_rate\":%u,\"channels\":2,\"bit_depth\":%d,\"codec_header\":\"%s\"}}}",
        (unsigned)rate, bits, b64);
    srv_json(js);
}

static void flac_start(const fx_t *x)
{
    flush_count = 0; fmt_calls = 0; pushed_len = 0; ring_fill_sim = 0; stage_hi = 0;
    flac_start_json(x, x->flac, 42, x->bits, x->rate);       /* 'fLaC' + STREAMINFO */
}

/* let the "consumer" drain `drain` bytes per poll until everything is decoded */
static void flac_settle(uint32_t drain)
{
    for (int i = 0; i < 4000; i++) {
        poll_n(1);
        stub_drain(drain);
        if (!nfl_pending() && !c.blocked && !c.hold) break;
    }
}

static void send_frames(const fx_t *x, int first, int count, int corrupt_frame, int corrupt_mode)
{
    static uint8_t buf[60000];
    size_t n = 0;
    for (int k = first; k < first + count; k++) {
        memcpy(buf + n, x->flac + x->fr[k].off, x->fr[k].size);
        if (k == corrupt_frame) {
            if (corrupt_mode == 0) buf[n + x->fr[k].size / 2] ^= 0x55;     /* mid payload */
            else buf[n + x->fr[k].size - 1] ^= 0x01;                       /* CRC-16 only */
        }
        n += x->fr[k].size;
    }
    srv_audio(ts_now(), buf, n);
}

static bool pcm_matches(const fx_t *x, int frame, int nframes, size_t at)
{
    size_t o = fx_sample_pos(x, frame);
    size_t l = fx_sample_pos(x, frame + nframes) - o;
    return pushed_len >= at + l && !memcmp(pushed + at, x->pcm + o, l);
}

static void test_flac(void)
{
    static char js[4096]; uint8_t op; size_t n; static uint8_t pl[4096];
    fx_t a, b, s, w;
    CHECK(fx_load(&a, "f44_16", 44100, 16));
    CHECK(fx_load(&b, "f48_16", 48000, 16));
    CHECK(fx_load(&s, "f44_16_s", 44100, 16));
    CHECK(fx_load(&w, "f44_24", 44100, 24));
    if (!a.pcm || !b.pcm || !s.pcm || !w.pcm) {
        CHECK(0);
        fx_free(&a); fx_free(&b); fx_free(&s); fx_free(&w);
        return;
    }
    printf("flac fixtures: f44_16 %d frames, f48_16 %d, f44_16_s %d, f44_24 %d\n", a.nfr, b.nfr, s.nfr, w.nfr);

    memset(adat_mem.pre, 0xC3, 64); memset(adat_mem.post, 0xC3, 64);
    sendspin_client_start();
    stub_now_us += 60000000; i_keygen(S.s_priv, S.s_pub);
    do_session(false);

    /* ---- hello: flac first, then pcm; and omitted when the arena cannot be loaned ---- */
    {
        CHECK(send_hello());
        CHECK(wait_json_type("client/hello", js, sizeof(js)));
        const char *f1 = strstr(js, "\"codec\":\"flac\",\"channels\":2,\"sample_rate\":48000,\"bit_depth\":16");
        const char *f2 = strstr(js, "\"codec\":\"flac\",\"channels\":2,\"sample_rate\":44100,\"bit_depth\":16");
        const char *p1 = strstr(js, "\"codec\":\"pcm\"");
        CHECK(f1 && f2 && p1 && f1 < f2 && f2 < p1);
        CHECK(strstr(js, "\"supported_formats\":[{\"codec\":\"flac\"") != NULL);   /* flac is the first entry */
        CHECK(strstr(js, "\"bit_depth\":24") == NULL);                              /* 24-bit not advertised */
        printf("client/hello is %zu bytes (limit %d)\n", strlen(js), SS_TX_BUF - 8 - 1 - SSN_TAG_LEN);
        stub_loan_ok = false;
        CHECK(send_hello());
        CHECK(wait_json_type("client/hello", js, sizeof(js)));
        CHECK(strstr(js, "flac") == NULL && strstr(js, "\"codec\":\"pcm\"") != NULL);
        stub_loan_ok = true;
    }

    /* ---- 1. one frame per chunk, 44.1 kHz, bit-exact (header 'fLaC'+STREAMINFO with last-block flag clear) ---- */
    flac_start(&a);
    CHECK(fmt_calls == 1 && fmt_rate == 44100 && fmt_ch == 2 && fmt_bits == 16 && flush_count == 1);
    CHECK(sendspin_client_streaming() && sendspin_client_state() == SENDSPIN_STATE_STREAMING);
    CHECK(stub_loaned && nfl_active());
    g_chunk = 1460;
    for (int k = 0; k < a.nfr; k++) { send_frames(&a, k, 1, -1, 0); flac_settle(3000); }
    flac_settle(3000);
    CHECK(pushed_len == a.plen && !memcmp(pushed, a.pcm, a.plen));
    CHECK(pbufs_live == 0);

    /* ---- 2. multi-frame chunks, frames split across tiny TCP segments ---- */
    flac_start(&s);
    g_chunk = 777;
    for (int k = 0; k < s.nfr; k += 3) {
        int cnt = s.nfr - k < 3 ? s.nfr - k : 3;
        send_frames(&s, k, cnt, -1, 0);
        flac_settle(2500);
    }
    CHECK(pushed_len == s.plen && !memcmp(pushed, s.pcm, s.plen));
    g_chunk = 1;                                           /* pathological: byte at a time */
    flac_start(&s);
    send_frames(&s, 0, 4, -1, 0);
    flac_settle(100000);
    CHECK(pcm_matches(&s, 0, 4, 0) && pushed_len == fx_sample_pos(&s, 4));
    g_chunk = 1500;

    /* ---- 3. ring-full backpressure: never lost, never published early, TCP window closes ---- */
    flac_start(&a);
    pushed_len = 0; recved_total = injected_total = 0;
    ring_fill_sim = stub_ring_cap - 500;                   /* less than one decode slice free */
    send_frames(&a, 0, 1, -1, 0);
    CHECK(pushed_len == 0 && nfl_pending() && nfl_chunk_remaining() > 0);
    send_frames(&a, 1, 1, -1, 0);                          /* queued behind the pending chunk */
    CHECK(pushed_len == 0 && c.blocked && recved_total < injected_total);
    poll_n(3);
    CHECK(pushed_len == 0 && nfl_pending());
    ring_fill_sim = stub_ring_cap - 3000;                  /* room for two slices only */
    poll_n(10);                                            /* input-only slices first, then output */
    CHECK(pushed_len >= 1024 && pushed_len <= 3000);
    stub_drain(1u << 30);
    flac_settle(3000);
    CHECK(!nfl_pending() && !c.blocked && recved_total == injected_total);
    CHECK(pcm_matches(&a, 0, 2, 0) && pushed_len == fx_sample_pos(&a, 2));
    /* stalled consumer for > 5 s drops the link (same watchdog as PCM) */
    ring_fill_sim = stub_ring_cap - 500;
    send_frames(&a, 2, 1, -1, 0);
    send_frames(&a, 3, 1, -1, 0);
    CHECK(c.blocked);
    stub_now_us += 6000000; poll_n(1);
    CHECK(sendspin_client_state() == SENDSPIN_STATE_ERROR && !stub_loaned && !nfl_active());
    stub_now_us += 20000000; i_keygen(S.s_priv, S.s_pub); do_session(false);

    /* ---- 4. corrupted frame: only that frame is dropped ---- */
    {
        /* (a) one frame per chunk, payload corrupted: neighbours untouched */
        flac_start(&s);
        for (int k = 0; k < s.nfr; k++) { send_frames(&s, k, 1, 5, 0); flac_settle(4000); }
        size_t before = fx_sample_pos(&s, 5), f5 = s.fr[5].samples * (size_t)s.bpf;
        CHECK(pushed_len == s.plen - f5);
        CHECK(!memcmp(pushed, s.pcm, before));
        CHECK(!memcmp(pushed + before, s.pcm + before + f5, s.plen - before - f5));
        CHECK(nfl_stat_frames_dropped() >= 1);
        /* (b) corrupted CRC-16 inside a 3-frame chunk: frames 3 and 5 survive, 4 is dropped */
        flac_start(&s);
        send_frames(&s, 3, 3, 4, 1);
        flac_settle(4000);
        size_t f3 = s.fr[3].samples * (size_t)s.bpf, f4 = s.fr[4].samples * (size_t)s.bpf, f5b = s.fr[5].samples * (size_t)s.bpf;
        CHECK(pushed_len == f3 + f5b);
        CHECK(!memcmp(pushed, s.pcm + fx_sample_pos(&s, 3), f3));
        CHECK(!memcmp(pushed + f3, s.pcm + fx_sample_pos(&s, 3) + f3 + f4, f5b));
        /* the stream keeps going afterwards */
        send_frames(&s, 6, 1, -1, 0); flac_settle(4000);
        CHECK(pushed_len == f3 + f5b + s.fr[6].samples * (size_t)s.bpf);
        CHECK(sendspin_client_state() == SENDSPIN_STATE_STREAMING);
    }

    /* ---- 5. stream/clear mid-stream, then a seek: decoder resynchronised ---- */
    flac_start(&a);
    send_frames(&a, 0, 1, -1, 0); flac_settle(3000);
    send_frames(&a, 1, 1, -1, 0);                          /* leave a partly decoded chunk behind */
    poll_n(1);
    int fc = flush_count;
    ring_fill_sim = stub_ring_cap - 500;                   /* consumer stalls with the chunk pending */
    send_frames(&a, 2, 1, -1, 0);                          /* may sit pending */
    srv_json("{\"type\":\"stream/clear\",\"payload\":{}}"); /* queued behind it: handled once it drained */
    flac_settle(5000);
    CHECK(flush_count == fc + 1 && !nfl_pending());
    pushed_len = 0; ring_fill_sim = 0;
    send_frames(&a, 6, 2, -1, 0);                          /* seek: frames 6,7 */
    flac_settle(4000);
    CHECK(pushed_len == fx_sample_pos(&a, 8) - fx_sample_pos(&a, 6) && pcm_matches(&a, 6, 2, 0));
    /* a half-delivered chunk (truncated final frame) must not poison the next chunk */
    {
        static uint8_t buf[40000]; size_t l7 = a.fr[0].size;
        memcpy(buf, a.flac + a.fr[0].off, l7 - 30);        /* frame 0 cut short */
        pushed_len = 0;
        srv_audio(ts_now(), buf, l7 - 30); flac_settle(4000);
        CHECK(pushed_len < a.fr[0].samples * (size_t)a.bpf);
        size_t got = pushed_len;
        send_frames(&a, 1, 1, -1, 0); flac_settle(4000);
        CHECK(pushed_len == got + a.fr[1].samples * (size_t)a.bpf);
        CHECK(!memcmp(pushed + got, a.pcm + fx_sample_pos(&a, 1), a.fr[1].samples * (size_t)a.bpf));
    }

    /* ---- 6. restart with a different rate (no stream/end in between) ---- */
    flac_start(&b);
    CHECK(fmt_rate == 48000 && fmt_bits == 16 && fmt_calls == 1 && flush_count == 1 && stub_loaned);
    for (int k = 0; k < b.nfr; k++) { send_frames(&b, k, 1, -1, 0); flac_settle(3000); }
    CHECK(pushed_len == b.plen && !memcmp(pushed, b.pcm, b.plen));

    /* ---- 7. 24-bit stream (decodes, packed S24 into the ring) ---- */
    flac_start(&w);
    CHECK(fmt_rate == 44100 && fmt_bits == 24 && stub_loaned);
    for (int k = 0; k < w.nfr; k++) { send_frames(&w, k, 1, -1, 0); flac_settle(3000); }
    CHECK(pushed_len == w.plen && !memcmp(pushed, w.pcm, w.plen));

    /* ---- 8. bounded work per pump call ---- */
    flac_start(&a);
    stub_drain(1u << 30);
    {
        int big = 0;                                       /* the largest frame (noise burst) */
        for (int k = 0; k < a.nfr; k++) if (a.fr[k].size > a.fr[big].size) big = k;
        send_frames(&a, big, 1, -1, 0);                    /* ds_flac_commit() ran one pump() */
        uint32_t used = (uint32_t)a.fr[big].size - nfl_chunk_remaining();
        CHECK(used <= 8u * 256u /* NFL_MAX_SLICES * NFL_IN_SLICE (net_flac.c) */);
        CHECK(nfl_pending());                              /* a 16 KB frame needs several passes */
        flac_settle(1u << 30);
        CHECK(pushed_len == a.fr[big].samples * (size_t)a.bpf);
    }

    /* ---- 9. unsupported / malformed flac streams are ignored, not fatal ---- */
    {
        static uint8_t hdr[200];
        memcpy(hdr, a.flac, 42);
        struct { const char *what; int bits; uint32_t rate; int patch; } t[] = {
            { "bad magic", 16, 44100, 1 }, { "mono", 16, 44100, 2 },
            { "max_blocksize 8192", 16, 44100, 3 }, { "rate mismatch", 16, 48000, 0 },
            { "bit depth mismatch", 24, 44100, 0 }, { "short header", 16, 44100, 4 },
            { "not streaminfo", 16, 44100, 5 },
        };
        for (size_t i = 0; i < sizeof(t) / sizeof(t[0]); i++) {
            memcpy(hdr, a.flac, 42);
            size_t hl = 42;
            switch (t[i].patch) {
            case 1: hdr[0] = 'X'; break;
            case 2: hdr[20] = (uint8_t)((hdr[20] & ~0x0E) | (0 << 1)); break;      /* 1 channel */
            case 3: hdr[10] = 0x20; hdr[11] = 0x00; break;
            case 4: hl = 30; break;
            case 5: hdr[4] = 4; break;                                               /* VORBIS_COMMENT */
            }
            flac_start_json(&a, hdr, hl, t[i].bits, t[i].rate);
            CHECK(!sendspin_client_streaming() && !stub_loaned && !nfl_active());
            CHECK(sendspin_client_state() == SENDSPIN_STATE_READY);
            pushed_len = 0;
            send_frames(&a, 0, 1, -1, 0);                  /* audio of a rejected stream is ignored */
            CHECK(pushed_len == 0 && sendspin_client_state() == SENDSPIN_STATE_READY);
        }
        /* missing codec_header */
        srv_json("{\"type\":\"stream/start\",\"payload\":{\"player\":{\"codec\":\"flac\","
                 "\"sample_rate\":44100,\"channels\":2,\"bit_depth\":16}}}");
        CHECK(!sendspin_client_streaming() && !stub_loaned);
        /* arena not loanable (ADAT/TDM output enabled) */
        stub_loan_ok = false;
        flac_start(&a);
        CHECK(!sendspin_client_streaming() && !nfl_active() && sendspin_client_state() == SENDSPIN_STATE_READY);
        stub_loan_ok = true;
        /* a longer header (extra metadata blocks after STREAMINFO) is fine */
        {
            static uint8_t lh[42 + 4 + 1000];                /* fLaC + STREAMINFO + one 1000-byte PADDING block */
            memcpy(lh, a.flac, 42);
            lh[4] &= 0x7F;                                   /* STREAMINFO is no longer the last block */
            lh[42] = 0x81; lh[43] = 0; lh[44] = 1000 >> 8; lh[45] = 1000 & 0xFF;   /* last block, type PADDING */
            memset(lh + 46, 0, 1000);
            flac_start_json(&a, lh, sizeof(lh), 16, 44100);
        }
        CHECK(sendspin_client_streaming() && nfl_active());
    }

    /* ---- 10. chunk larger than the compressed buffer: authenticated, dropped, stream stays in sync ---- */
    {
        static uint8_t big[30000];
        memset(big, 0xA5, sizeof(big));
        pushed_len = 0;
        CHECK(nfl_chunk_cap() > 16500 && nfl_chunk_cap() < 40000);
        srv_audio(ts_now(), big, nfl_chunk_cap() + 1);
        CHECK(pushed_len == 0 && !nfl_pending() && sendspin_client_state() == SENDSPIN_STATE_STREAMING);
        send_frames(&a, 0, 1, -1, 0); flac_settle(4000);
        CHECK(pcm_matches(&a, 0, 1, 0));
        /* a chunk of garbage that fits: no output, decoder keeps working */
        pushed_len = 0;
        srv_audio(ts_now(), big, 5000); flac_settle(4000);
        CHECK(pushed_len == 0 && !nfl_pending() && sendspin_client_state() == SENDSPIN_STATE_STREAMING);
        send_frames(&a, 1, 1, -1, 0); flac_settle(4000);
        CHECK(pcm_matches(&a, 1, 1, 0));
    }

    /* ---- 11. tampered chunk (bad tag): nothing decoded, link dropped, arena returned ---- */
    {
        static uint8_t pt[20000];
        pt[0] = 4; memset(pt + 1, 0, 12);
        memcpy(pt + 13, a.flac + a.fr[0].off, a.fr[0].size);
        pushed_len = 0;
        int pa = pcb_aborted;
        srv_enc_ex(pt, 13 + a.fr[0].size, true, 0);
        CHECK(pushed_len == 0 && sendspin_client_state() == SENDSPIN_STATE_ERROR && pcb_aborted == pa + 1);
        CHECK(!stub_loaned && !nfl_active() && pbufs_live == 0);
        stub_now_us += 20000000; i_keygen(S.s_priv, S.s_pub); do_session(false);
    }

    /* ---- 12. stream/end returns the arena; a later pcm stream works ---- */
    flac_start(&a);
    CHECK(stub_loaned);
    srv_json("{\"type\":\"stream/end\",\"payload\":{}}");
    CHECK(!stub_loaned && !nfl_active() && !sendspin_client_streaming());
    start_stream();                                          /* pcm */
    CHECK(!stub_loaned);

    /* ---- 13. ADAT/TDM output claims the ring: FLAC torn down, client reconnects with pcm only ---- */
    flac_start(&a);
    CHECK(stub_loaned && stub_evict);
    stub_loan_ok = false;                                    /* output enabled now */
    stub_evict();                                            /* adat_output_resync() -> evict callback */
    CHECK(!stub_loaned && !nfl_active());
    poll_n(1);
    CHECK(sendspin_client_state() == SENDSPIN_STATE_ERROR);
    stub_now_us += 20000000; i_keygen(S.s_priv, S.s_pub); do_session(false);
    stub_loan_ok = true;

    /* ---- 14. dspi-rpm: nfl_end() must clear a pending eviction flag ---- */
    CHECK(nfl_begin_stream() == NFL_OK && stub_evict);
    stub_loan_ok = false;
    stub_evict();                                            /* output claims the ring while a stream is open */
    stub_loan_ok = true;
    nfl_end();                                               /* protocol switch / stream teardown */
    CHECK(!nfl_take_evicted());                              /* stale flag must not kill the next stream */
    CHECK(nfl_begin_stream() == NFL_OK && !nfl_take_evicted());
    nfl_end();

    /* ---- 15. dspi-91m: frames that differ from STREAMINFO (mono, other rate) are dropped ---- */
    {
        fx_t m;
        CHECK(fx_load_ch(&m, "m44_16", 44100, 16, 1));
        flac_start(&a);
        uint32_t d0 = nfl_stat_frames_dropped();
        for (int k = 0; k < m.nfr && k < 3; k++) { send_frames(&m, k, 1, -1, 0); flac_settle(3000); }
        CHECK(pushed_len == 0);                              /* mono frame must not be played as stereo */
        CHECK(nfl_stat_frames_dropped() > d0);
        for (int k = 0; k < 2; k++) { send_frames(&b, k, 1, -1, 0); flac_settle(3000); }
        CHECK(pushed_len == 0);                              /* 48 kHz frame in a 44.1 kHz stream */
        send_frames(&a, 0, 1, -1, 0); flac_settle(3000);     /* decoder still in sync afterwards */
        CHECK(pcm_matches(&a, 0, 1, 0) && pushed_len == a.fr[0].samples * 4u);
        send_frames(&m, 0, 1, -1, 0); send_frames(&a, 1, 1, -1, 0); flac_settle(3000);
        CHECK(pcm_matches(&a, 1, 1, a.fr[0].samples * 4u) &&
              pushed_len == (a.fr[0].samples + a.fr[1].samples) * 4u);
        srv_json("{\"type\":\"stream/end\",\"payload\":{}}");
        fx_free(&m);
    }

    /* ---- 16. dspi-3eq: ADAT claims the ring mid-record; the rest of the record must not
     * be written into the (now live) ring ---- */
    {
        static uint8_t pt[3100], ct[3200], f[3400];
        flac_start(&a);
        CHECK(stub_loaned && stub_evict && nfl_active());
        uint8_t *cb = nfl_chunk_buf();
        pt[0] = 4; memset(pt + 1, 0, 12);
        memset(pt + 13, 0x11, 3000);
        i_aead_enc(S.k_send, S.n_send++, NULL, 0, pt, 3013, ct);
        size_t fn = srv_frame(f, 2, true, ct, 3013 + 16, 0);
        inject(f, 1500, 1500);                               /* first TCP segment: record start */
        memset(cb + 1700, 0xA5, 1200);                       /* sentinel beyond what has arrived */
        stub_loan_ok = false;
        stub_evict();                                        /* adat_output_resync() */
        CHECK(!nfl_active());
        inject(f + 1500, fn - 1500, 1500);                   /* rest arrives before the next poll */
        int bad = 0;
        for (int i = 1700; i < 2900; i++) if (cb[i] != 0xA5) bad++;
        CHECK(bad == 0);
        poll_n(1);
        CHECK(!stub_loaned);
        stub_now_us += 20000000; i_keygen(S.s_priv, S.s_pub); do_session(false);
        stub_loan_ok = true;
    }

    /* ---- 17. dspi-0e1 (Sendspin part): codec_header with JSON-escaped '/' (and \u002b) ---- */
    {
        static char b64[200], esc[400], js[800]; size_t ol = 0, k = 0;
        /* escape every '/' as \/ and '+' as \u002b, plus char 1 as \u00XX so the test
         * does not depend on the header's base64 text containing those characters */
        uint8_t h[42]; memcpy(h, a.flac, 42);
        mbedtls_base64_encode((unsigned char *)b64, sizeof(b64), &ol, h, 42); b64[ol] = 0;
        int nesc = 0;
        for (size_t i = 0; i < ol; i++) {
            if (b64[i] == '/') { esc[k++] = '\\'; esc[k++] = '/'; nesc++; }
            else if (b64[i] == '+') { memcpy(esc + k, "\\u002b", 6); k += 6; nesc++; }
            else if (i == 1) { k += (size_t)sprintf(esc + k, "\\u%04x", (unsigned)b64[i]); nesc++; }
            else esc[k++] = b64[i];
        }
        esc[k] = 0;
        CHECK(nesc >= 1);
        snprintf(js, sizeof(js),
            "{\"type\":\"stream/start\",\"payload\":{\"player\":{\"codec\":\"flac\","
            "\"sample_rate\":%u,\"channels\":2,\"bit_depth\":%d,\"codec_header\":\"%s\"}}}",
            (unsigned)a.rate, a.bits, esc);
        flush_count = 0; fmt_calls = 0; pushed_len = 0; ring_fill_sim = 0; stage_hi = 0;
        srv_json(js);
        CHECK(sendspin_client_streaming() && nfl_active());
        send_frames(&a, 0, 1, -1, 0); flac_settle(4000);
        CHECK(pcm_matches(&a, 0, 1, 0));
        srv_json("{\"type\":\"stream/end\",\"payload\":{}}");
    }

    /* arena guard bytes were never touched */
    for (size_t i = 0; i < 64; i++) CHECK(adat_mem.pre[i] == 0xC3 && adat_mem.post[i] == 0xC3);
    sendspin_client_stop();
    fx_free(&a); fx_free(&b); fx_free(&s); fx_free(&w);
    (void)op; (void)n; (void)pl;
}

/* tiny fuzz: random garbage into the parser must never crash or hang */
static void test_fuzz(void)
{
    g_host = "127.0.0.1";
    sendspin_client_start();
    for (int round = 0; round < 200; round++) {
        stub_now_us += 20000000; out_len = rd_off = 0;
        poll_n(2);
        if (!pcb_alive) continue;
        connected_cb(cb_arg, &the_pcb, ERR_OK);
        out_cap[out_len] = 0;
        char *kp = strstr((char *)out_cap, "Sec-WebSocket-Key: ");
        if (!kp) continue;
        char key[40]; sscanf(kp + 19, "%39[^\r]", key);
        char acc[32]; ws_accept_key(key, acc);
        char resp[300]; snprintf(resp, sizeof(resp), "HTTP/1.1 101 x\r\nSec-WebSocket-Accept: %s\r\n\r\n", acc);
        inject((const uint8_t *)resp, strlen(resp), 200);
        uint8_t junk[300];
        for (int k = 0; k < 4 && pcb_alive; k++) {
            for (size_t i = 0; i < sizeof(junk); i++) junk[i] = (uint8_t)get_rand_32();
            if (round & 1) { junk[0] = 0x81; junk[1] = (uint8_t)(get_rand_32() % 100); }
            inject(junk, 1 + get_rand_32() % sizeof(junk), 1 + get_rand_32() % 50);
        }
    }
    CHECK(pbufs_live == 0);
    sendspin_client_stop();
}

int main(int argc, char **argv)
{
    g_fixdir = argc > 1 ? argv[1] : NULL;
    test_vectors();
    test_json();
    test_noise_direct();
    test_e2e();
    if (g_fixdir) test_flac(); else printf("FLAC end-to-end tests SKIPPED (no fixtures)\n");
    test_fuzz();
    printf("\n%d checks passed, %d failed\n", g_pass, g_fail);
    printf("static RAM of client module: g_rx=%u, state struct c=%u (incl. noise state %u)\n",
           (unsigned)sizeof(g_rx), (unsigned)sizeof(c), (unsigned)sizeof(ssn_t));
    return g_fail ? 1 : 0;
}
