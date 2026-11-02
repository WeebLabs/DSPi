/*
 * sendspin_noise.c - portable Noise_KKpsk2_25519_ChaChaPoly_SHA256 responder.
 * See sendspin_noise.h. Written from the Noise Protocol Framework spec
 * (rev 34): SymmetricState/CipherState/HandshakeState, HKDF = HMAC-SHA256.
 *
 * Conventions that matter for interop (all per the Noise spec):
 *  - protocol name "Noise_KKpsk2_25519_ChaChaPoly_SHA256" is 36 bytes (> 32),
 *    so h = SHA-256(name); ck = h.
 *  - MixHash(prologue), then pre-messages: initiator static, then responder
 *    static (order in which they are listed in the pattern).
 *  - In a pattern containing a psk token, every 'e' token additionally does
 *    MixKey(e.public_key).
 *  - ChaChaPoly nonce: 4 zero bytes || 64-bit little-endian counter.
 *  - Split(): k1 = initiator->responder, k2 = responder->initiator.
 */
#include "sendspin_noise.h"

#include <string.h>

#include "mbedtls/build_info.h"
#include "mbedtls/sha256.h"
#include "mbedtls/chachapoly.h"
#include "mbedtls/ecp.h"
#include "mbedtls/bignum.h"
#include "mbedtls/platform_util.h"

#define SSN_PROTOCOL_NAME "Noise_KKpsk2_25519_ChaChaPoly_SHA256"

/* ------------------------------------------------------------------ */
/* hashing                                                             */
/* ------------------------------------------------------------------ */

void ssn_sha256(uint8_t out[32], const uint8_t *in, size_t len)
{
    mbedtls_sha256_context c;
    mbedtls_sha256_init(&c);
    mbedtls_sha256_starts(&c, 0);
    if (len) mbedtls_sha256_update(&c, in, len);
    mbedtls_sha256_finish(&c, out);
    mbedtls_sha256_free(&c);
}

void ssn_hmac_sha256(uint8_t out[32], const uint8_t *key, size_t keylen,
                     const uint8_t *data, size_t len)
{
    uint8_t k[64], pad[64], inner[32];
    mbedtls_sha256_context c;
    memset(k, 0, sizeof(k));
    if (keylen > 64) ssn_sha256(k, key, keylen);
    else if (keylen) memcpy(k, key, keylen);

    for (int i = 0; i < 64; i++) pad[i] = k[i] ^ 0x36;
    mbedtls_sha256_init(&c);
    mbedtls_sha256_starts(&c, 0);
    mbedtls_sha256_update(&c, pad, 64);
    if (len) mbedtls_sha256_update(&c, data, len);
    mbedtls_sha256_finish(&c, inner);

    for (int i = 0; i < 64; i++) pad[i] = k[i] ^ 0x5c;
    mbedtls_sha256_starts(&c, 0);
    mbedtls_sha256_update(&c, pad, 64);
    mbedtls_sha256_update(&c, inner, 32);
    mbedtls_sha256_finish(&c, out);
    mbedtls_sha256_free(&c);
    mbedtls_platform_zeroize(k, sizeof(k));
    mbedtls_platform_zeroize(pad, sizeof(pad));
}

/* Noise HKDF: outputs 2 or 3 32-byte blocks (o3 may be NULL). */
static void ssn_hkdf(const uint8_t ck[32], const uint8_t *ikm, size_t ikm_len,
                     uint8_t o1[32], uint8_t o2[32], uint8_t o3[32])
{
    uint8_t temp[32], buf[33];
    ssn_hmac_sha256(temp, ck, 32, ikm, ikm_len);
    buf[0] = 0x01;
    ssn_hmac_sha256(o1, temp, 32, buf, 1);
    memcpy(buf, o1, 32); buf[32] = 0x02;
    ssn_hmac_sha256(o2, temp, 32, buf, 33);
    if (o3) {
        memcpy(buf, o2, 32); buf[32] = 0x03;
        ssn_hmac_sha256(o3, temp, 32, buf, 33);
    }
    mbedtls_platform_zeroize(temp, sizeof(temp));
}

/* ------------------------------------------------------------------ */
/* X25519                                                              */
/* ------------------------------------------------------------------ */

static int x25519_mul(uint8_t out[32], const uint8_t k[32], const uint8_t *u,
                      ssn_rng_fn rng, void *rng_ctx)
{
    mbedtls_ecp_group grp;
    mbedtls_ecp_point P, R;
    mbedtls_mpi d;
    uint8_t kc[32], uc[32];
    int ret;

    mbedtls_ecp_group_init(&grp);
    mbedtls_ecp_point_init(&P);
    mbedtls_ecp_point_init(&R);
    mbedtls_mpi_init(&d);

    memcpy(kc, k, 32);
    kc[0] &= 248; kc[31] &= 127; kc[31] |= 64;   /* RFC 7748 clamping */

    ret = mbedtls_ecp_group_load(&grp, MBEDTLS_ECP_DP_CURVE25519);
    if (ret) goto done;
    ret = mbedtls_mpi_read_binary_le(&d, kc, 32);
    if (ret) goto done;
    if (u) {
        memcpy(uc, u, 32);
        uc[31] &= 0x7f;                          /* RFC 7748: mask bit 255 */
    } else {
        memset(uc, 0, 32);
        uc[0] = 9;                               /* base point u = 9 */
    }
    ret = mbedtls_ecp_point_read_binary(&grp, &P, uc, 32);
    if (ret) goto done;
    ret = mbedtls_ecp_mul(&grp, &R, &d, &P, rng, rng_ctx);
    if (ret) goto done;
    {
        size_t olen = 0;   /* Montgomery export = little-endian u-coordinate */
        ret = mbedtls_ecp_point_write_binary(&grp, &R, MBEDTLS_ECP_PF_UNCOMPRESSED,
                                             &olen, out, 32);
        if (!ret && olen != 32) ret = -1;
    }
    if (ret) goto done;

    {   /* reject low-order points (all-zero output) */
        uint8_t acc = 0;
        for (int i = 0; i < 32; i++) acc |= out[i];
        if (acc == 0) ret = -1;
    }
done:
    mbedtls_ecp_group_free(&grp);
    mbedtls_ecp_point_free(&P);
    mbedtls_ecp_point_free(&R);
    mbedtls_mpi_free(&d);
    mbedtls_platform_zeroize(kc, sizeof(kc));
    return ret ? -1 : 0;
}

int ssn_x25519(uint8_t out[32], const uint8_t priv[32], const uint8_t pub[32],
               ssn_rng_fn rng, void *rng_ctx)
{
    return x25519_mul(out, priv, pub, rng, rng_ctx);
}

int ssn_x25519_base(uint8_t pub[32], const uint8_t priv[32],
                    ssn_rng_fn rng, void *rng_ctx)
{
    return x25519_mul(pub, priv, NULL, rng, rng_ctx);
}

int ssn_gen_static(uint8_t priv[32], uint8_t pub[32], ssn_rng_fn rng, void *rng_ctx)
{
    if (rng(rng_ctx, priv, 32) != 0) return -1;
    priv[0] &= 248; priv[31] &= 127; priv[31] |= 64;
    return ssn_x25519_base(pub, priv, rng, rng_ctx);
}

/* ------------------------------------------------------------------ */
/* base64                                                              */
/* ------------------------------------------------------------------ */

static size_t b64_enc(char *out, const uint8_t *in, size_t len, const char *tbl, bool pad)
{
    size_t o = 0, i = 0;
    while (i + 2 < len) {
        uint32_t v = ((uint32_t)in[i] << 16) | ((uint32_t)in[i + 1] << 8) | in[i + 2];
        out[o++] = tbl[(v >> 18) & 63]; out[o++] = tbl[(v >> 12) & 63];
        out[o++] = tbl[(v >> 6) & 63];  out[o++] = tbl[v & 63];
        i += 3;
    }
    if (len - i == 1) {
        uint32_t v = (uint32_t)in[i] << 16;
        out[o++] = tbl[(v >> 18) & 63]; out[o++] = tbl[(v >> 12) & 63];
        if (pad) { out[o++] = '='; out[o++] = '='; }
    } else if (len - i == 2) {
        uint32_t v = ((uint32_t)in[i] << 16) | ((uint32_t)in[i + 1] << 8);
        out[o++] = tbl[(v >> 18) & 63]; out[o++] = tbl[(v >> 12) & 63];
        out[o++] = tbl[(v >> 6) & 63];
        if (pad) out[o++] = '=';
    }
    out[o] = 0;
    return o;
}

size_t ssn_b64url_encode(char *out, const uint8_t *in, size_t len)
{
    return b64_enc(out, in, len,
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_", false);
}

size_t ssn_b64_encode(char *out, const uint8_t *in, size_t len)
{
    return b64_enc(out, in, len,
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/", true);
}

static int b64v(char c)
{
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '-') return 62;
    if (c == '_') return 63;
    return -1;
}

int ssn_b64url_decode(uint8_t *out, size_t max, const char *in, size_t len)
{
    while (len && in[len - 1] == '=') len--;      /* tolerate padding */
    if (len % 4 == 1) return -1;
    size_t o = 0; uint32_t acc = 0; int bits = 0;
    for (size_t i = 0; i < len; i++) {
        int v = b64v(in[i]);
        if (v < 0) return -1;
        acc = (acc << 6) | (uint32_t)v; bits += 6;
        if (bits >= 8) {
            bits -= 8;
            if (o >= max) return -1;
            out[o++] = (uint8_t)(acc >> bits);
            acc &= (1u << bits) - 1;
        }
    }
    return (int)o;
}

/* ------------------------------------------------------------------ */
/* sentinel PSK                                                        */
/* ------------------------------------------------------------------ */

void ssn_sentinel_psk(uint8_t psk[32])
{
    static const char label[] = "sendspin-sentinel-psk-v1";
    ssn_sha256(psk, (const uint8_t *)label, sizeof(label) - 1);
}

void ssn_psk_id(char *out, const uint8_t psk[32])
{
    static const char label[] = "sendspin-psk-id-v1";
    uint8_t buf[sizeof(label) - 1 + 32], h[32];
    memcpy(buf, label, sizeof(label) - 1);
    memcpy(buf + sizeof(label) - 1, psk, 32);
    ssn_sha256(h, buf, sizeof(buf));
    ssn_b64url_encode(out, h, 32);
}

/* ------------------------------------------------------------------ */
/* AEAD                                                                */
/* ------------------------------------------------------------------ */

static void make_nonce(uint8_t nonce[12], uint64_t n)
{
    nonce[0] = nonce[1] = nonce[2] = nonce[3] = 0;
    for (int i = 0; i < 8; i++) nonce[4 + i] = (uint8_t)(n >> (8 * i));
}

/* out may alias in; writes len + 16 bytes. */
static int aead_enc(const uint8_t key[32], uint64_t n, const uint8_t *ad, size_t adlen,
                    const uint8_t *in, size_t len, uint8_t *out)
{
    mbedtls_chachapoly_context c;
    uint8_t nonce[12];
    int ret;
    make_nonce(nonce, n);
    mbedtls_chachapoly_init(&c);
    ret = mbedtls_chachapoly_setkey(&c, key);
    if (!ret)
        ret = mbedtls_chachapoly_encrypt_and_tag(&c, len, nonce, ad, adlen, in, out, out + len);
    mbedtls_chachapoly_free(&c);
    return ret ? -1 : 0;
}

/* in: len bytes incl. 16-byte tag. out may alias in. Returns plaintext len or -1 */
static int aead_dec(const uint8_t key[32], uint64_t n, const uint8_t *ad, size_t adlen,
                    const uint8_t *in, size_t len, uint8_t *out)
{
    mbedtls_chachapoly_context c;
    uint8_t nonce[12];
    int ret;
    if (len < SSN_TAG_LEN) return -1;
    make_nonce(nonce, n);
    mbedtls_chachapoly_init(&c);
    ret = mbedtls_chachapoly_setkey(&c, key);
    if (!ret)
        ret = mbedtls_chachapoly_auth_decrypt(&c, len - SSN_TAG_LEN, nonce, ad, adlen,
                                              in + len - SSN_TAG_LEN, in, out);
    mbedtls_chachapoly_free(&c);
    return ret ? -1 : (int)(len - SSN_TAG_LEN);
}

/* ------------------------------------------------------------------ */
/* SymmetricState                                                      */
/* ------------------------------------------------------------------ */

static void mix_hash(ssn_t *st, const uint8_t *data, size_t len)
{
    mbedtls_sha256_context c;
    mbedtls_sha256_init(&c);
    mbedtls_sha256_starts(&c, 0);
    mbedtls_sha256_update(&c, st->h, 32);
    if (len) mbedtls_sha256_update(&c, data, len);
    mbedtls_sha256_finish(&c, st->h);
    mbedtls_sha256_free(&c);
}

static void mix_key(ssn_t *st, const uint8_t *ikm, size_t len)
{
    uint8_t o1[32], o2[32];
    ssn_hkdf(st->ck, ikm, len, o1, o2, NULL);
    memcpy(st->ck, o1, 32);
    memcpy(st->k, o2, 32);
    st->has_key = true;
    st->n = 0;
    mbedtls_platform_zeroize(o1, 32); mbedtls_platform_zeroize(o2, 32);
}

static void mix_key_and_hash(ssn_t *st, const uint8_t *ikm, size_t len)
{
    uint8_t o1[32], o2[32], o3[32];
    ssn_hkdf(st->ck, ikm, len, o1, o2, o3);
    memcpy(st->ck, o1, 32);
    mix_hash(st, o2, 32);
    memcpy(st->k, o3, 32);
    st->has_key = true;
    st->n = 0;
    mbedtls_platform_zeroize(o1, 32); mbedtls_platform_zeroize(o2, 32);
    mbedtls_platform_zeroize(o3, 32);
}

/* out may alias in; writes len (+16 if keyed) bytes; returns bytes written or -1 */
static int encrypt_and_hash(ssn_t *st, const uint8_t *in, size_t len, uint8_t *out)
{
    if (!st->has_key) {
        if (out != in && len) memcpy(out, in, len);
        mix_hash(st, out, len);
        return (int)len;
    }
    if (aead_enc(st->k, st->n, st->h, 32, in, len, out) != 0) return -1;
    st->n++;
    mix_hash(st, out, len + SSN_TAG_LEN);
    return (int)(len + SSN_TAG_LEN);
}

/* returns plaintext length or -1 */
static int decrypt_and_hash(ssn_t *st, const uint8_t *in, size_t len, uint8_t *out)
{
    if (!st->has_key) {
        if (out != in && len) memcpy(out, in, len);
        mix_hash(st, in, len);
        return (int)len;
    }
    /* mix_hash needs the ciphertext; hash it first in case out aliases in */
    uint8_t save_h[32];
    memcpy(save_h, st->h, 32);
    mix_hash(st, in, len);
    int r = aead_dec(st->k, st->n, save_h, 32, in, len, out);
    if (r < 0) return -1;
    st->n++;
    return r;
}

/* ------------------------------------------------------------------ */
/* Responder handshake                                                 */
/* ------------------------------------------------------------------ */

int ssn_resp_init(ssn_t *st, const uint8_t s_priv[32], const uint8_t s_pub[32],
                  const uint8_t rs_pub[32],
                  const uint8_t *prologue, size_t prologue_len,
                  ssn_rng_fn rng, void *rng_ctx)
{
    memset(st, 0, sizeof(*st));
    memcpy(st->s_priv, s_priv, 32);
    memcpy(st->s_pub, s_pub, 32);
    memcpy(st->rs, rs_pub, 32);
    st->rng = rng;
    st->rng_ctx = rng_ctx;

    /* InitializeSymmetric: name is 36 bytes > HASHLEN -> hash it */
    ssn_sha256(st->h, (const uint8_t *)SSN_PROTOCOL_NAME, sizeof(SSN_PROTOCOL_NAME) - 1);
    memcpy(st->ck, st->h, 32);
    st->has_key = false;
    st->n = 0;

    mix_hash(st, prologue, prologue_len);
    /* pre-messages: -> s (initiator static), <- s (responder static = ours) */
    mix_hash(st, st->rs, 32);
    mix_hash(st, st->s_pub, 32);
    st->stage = 0;
    return 0;
}

int ssn_resp_read_msg1(ssn_t *st, const uint8_t *msg, size_t len,
                       uint8_t *payload, size_t payload_cap, size_t *payload_len)
{
    uint8_t dh[32];
    int r;
    if (st->stage != 0) return -1;
    if (len < 32 + SSN_TAG_LEN) return -1;
    if (len - 32 - SSN_TAG_LEN > payload_cap) return -2;

    /* e */
    memcpy(st->re, msg, 32);
    mix_hash(st, st->re, 32);
    mix_key(st, st->re, 32);                 /* psk pattern: e also MixKey */
    /* es: responder computes DH(s, re) */
    if (ssn_x25519(dh, st->s_priv, st->re, st->rng, st->rng_ctx) != 0) return -3;
    mix_key(st, dh, 32);
    /* ss: DH(s, rs) */
    if (ssn_x25519(dh, st->s_priv, st->rs, st->rng, st->rng_ctx) != 0) return -3;
    mix_key(st, dh, 32);
    mbedtls_platform_zeroize(dh, sizeof(dh));

    r = decrypt_and_hash(st, msg + 32, len - 32, payload);
    if (r < 0) return -4;
    *payload_len = (size_t)r;
    st->stage = 1;
    return 0;
}

int ssn_resp_write_msg2(ssn_t *st, const uint8_t psk[32],
                        const uint8_t *payload, size_t payload_len,
                        uint8_t *out, size_t out_cap, size_t *out_len)
{
    uint8_t dh[32];
    int r;
    if (st->stage != 1) return -1;
    if (out_cap < 32 + payload_len + SSN_TAG_LEN) return -2;

    /* e */
    if (ssn_gen_static(st->e_priv, st->e_pub, st->rng, st->rng_ctx) != 0) return -3;
    memcpy(out, st->e_pub, 32);
    mix_hash(st, st->e_pub, 32);
    mix_key(st, st->e_pub, 32);
    /* ee: DH(e, re) */
    if (ssn_x25519(dh, st->e_priv, st->re, st->rng, st->rng_ctx) != 0) return -3;
    mix_key(st, dh, 32);
    /* se: initiator's s with responder's e => responder computes DH(e, rs) */
    if (ssn_x25519(dh, st->e_priv, st->rs, st->rng, st->rng_ctx) != 0) return -3;
    mix_key(st, dh, 32);
    mbedtls_platform_zeroize(dh, sizeof(dh));
    /* psk */
    mix_key_and_hash(st, psk, 32);

    if (payload_len) memcpy(out + 32, payload, payload_len);
    r = encrypt_and_hash(st, out + 32, payload_len, out + 32);
    if (r < 0) return -4;
    *out_len = 32 + (size_t)r;

    /* Split: k1 = initiator->responder, k2 = responder->initiator */
    uint8_t k1[32], k2[32];
    ssn_hkdf(st->ck, NULL, 0, k1, k2, NULL);
    memcpy(st->k_recv, k1, 32);
    memcpy(st->k_send, k2, 32);
    st->n_send = st->n_recv = 0;
    mbedtls_platform_zeroize(k1, 32); mbedtls_platform_zeroize(k2, 32);
    mbedtls_platform_zeroize(st->e_priv, 32);
    mbedtls_platform_zeroize(st->k, 32);
    st->has_key = false;
    st->stage = 2;
    return 0;
}

/* ------------------------------------------------------------------ */
/* Transport                                                           */
/* ------------------------------------------------------------------ */

int ssn_encrypt(ssn_t *st, const uint8_t *in, size_t len, uint8_t *out)
{
    if (st->stage != 2) return -1;
    if (aead_enc(st->k_send, st->n_send, NULL, 0, in, len, out) != 0) return -1;
    st->n_send++;
    return 0;
}

int ssn_decrypt(ssn_t *st, const uint8_t *in, size_t len, uint8_t *out)
{
    if (st->stage != 2) return -1;
    int r = aead_dec(st->k_recv, st->n_recv, NULL, 0, in, len, out);
    if (r >= 0) st->n_recv++;
    return r;
}

void ssn_skip_recv(ssn_t *st)
{
    st->n_recv++;
}
