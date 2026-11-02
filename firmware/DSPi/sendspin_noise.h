/*
 * sendspin_noise.h - portable Noise_KKpsk2_25519_ChaChaPoly_SHA256 RESPONDER
 * for the Sendspin player client, plus the small helpers it needs
 * (X25519, base64/base64url, SHA-256 helpers).
 *
 * No lwIP / Pico SDK dependency: only mbedtls (>= 3.x):
 *   MBEDTLS_CHACHAPOLY_C, MBEDTLS_CHACHA20_C, MBEDTLS_POLY1305_C,
 *   MBEDTLS_SHA256_C, MBEDTLS_ECP_C + MBEDTLS_ECP_DP_CURVE25519_ENABLED,
 *   MBEDTLS_BIGNUM_C   (HMAC/HKDF and base64 are implemented locally).
 *
 * Noise pattern (Noise spec rev 34, section 7 + 9.x):
 *   KKpsk2:
 *     -> s            (pre-message, initiator static known to responder)
 *     <- s            (pre-message, responder static known to initiator)
 *     ...
 *     -> e, es, ss
 *     <- e, ee, se, psk
 * The server is the initiator, this client is the responder.
 */
#ifndef SENDSPIN_NOISE_H
#define SENDSPIN_NOISE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SSN_KEY_LEN   32
#define SSN_HASH_LEN  32
#define SSN_TAG_LEN   16

/* Max transport message (ciphertext incl. tag), per Noise. */
#define SSN_MAX_MSG   65535

/* rng callback, same shape as mbedtls f_rng. Must return 0 on success. */
typedef int (*ssn_rng_fn)(void *ctx, unsigned char *buf, size_t len);

typedef struct {
    uint8_t  h[SSN_HASH_LEN];    /* handshake hash */
    uint8_t  ck[SSN_HASH_LEN];   /* chaining key */
    uint8_t  k[SSN_KEY_LEN];     /* current handshake cipher key */
    bool     has_key;
    uint64_t n;                  /* handshake cipher nonce */

    uint8_t  s_priv[32], s_pub[32];   /* our static */
    uint8_t  rs[32];                  /* initiator (server) static */
    uint8_t  e_priv[32], e_pub[32];   /* our ephemeral */
    uint8_t  re[32];                  /* initiator ephemeral */

    uint8_t  k_send[32], k_recv[32];  /* transport keys (after msg2) */
    uint64_t n_send, n_recv;
    int      stage;                   /* 0 init, 1 got msg1, 2 transport */
    ssn_rng_fn rng;
    void    *rng_ctx;
} ssn_t;

/* ---- primitives ---- */
void ssn_sha256(uint8_t out[32], const uint8_t *in, size_t len);
void ssn_hmac_sha256(uint8_t out[32], const uint8_t *key, size_t keylen,
                     const uint8_t *data, size_t len);

/* X25519 (RFC 7748). Returns 0 ok, <0 error (incl. all-zero shared secret). */
int ssn_x25519(uint8_t out[32], const uint8_t priv[32], const uint8_t pub[32],
               ssn_rng_fn rng, void *rng_ctx);
int ssn_x25519_base(uint8_t pub[32], const uint8_t priv[32],
                    ssn_rng_fn rng, void *rng_ctx);
/* Fill priv with 32 random bytes (clamped) and derive pub. */
int ssn_gen_static(uint8_t priv[32], uint8_t pub[32], ssn_rng_fn rng, void *rng_ctx);

/* base64url without padding. encode returns chars written (NUL-terminated),
 * out must hold 4*ceil(len/3)+1. decode returns bytes or -1. */
size_t ssn_b64url_encode(char *out, const uint8_t *in, size_t len);
int    ssn_b64url_decode(uint8_t *out, size_t max, const char *in, size_t len);
/* standard base64 with padding (for WebSocket) */
size_t ssn_b64_encode(char *out, const uint8_t *in, size_t len);

/* Sentinel PSK = SHA-256("sendspin-sentinel-psk-v1") */
void ssn_sentinel_psk(uint8_t psk[32]);
/* psk_id = base64url(SHA-256("sendspin-psk-id-v1" || psk)); out >= 44 bytes */
void ssn_psk_id(char *out, const uint8_t psk[32]);

/* ---- responder handshake ---- */
/* prologue = client/init text bytes || server/init text bytes (exact). */
int ssn_resp_init(ssn_t *st, const uint8_t s_priv[32], const uint8_t s_pub[32],
                  const uint8_t rs_pub[32],
                  const uint8_t *prologue, size_t prologue_len,
                  ssn_rng_fn rng, void *rng_ctx);

/* Process handshake message 1 (initiator -> us). Decrypted payload copied to
 * payload (cap payload_cap) and its length returned in *payload_len.
 * Returns 0 ok, <0 on failure (bad length / auth fail). */
int ssn_resp_read_msg1(ssn_t *st, const uint8_t *msg, size_t len,
                       uint8_t *payload, size_t payload_cap, size_t *payload_len);

/* Build handshake message 2 (us -> initiator), mixing in psk, then Split().
 * out must have room for 32 + payload_len + 16. Transport becomes active. */
int ssn_resp_write_msg2(ssn_t *st, const uint8_t psk[32],
                        const uint8_t *payload, size_t payload_len,
                        uint8_t *out, size_t out_cap, size_t *out_len);

/* ---- transport (valid after write_msg2) ---- */
/* out may equal in. out gets len + 16 bytes. 0 ok. */
int ssn_encrypt(ssn_t *st, const uint8_t *in, size_t len, uint8_t *out);
/* in-place allowed. Returns plaintext length (>=0) or <0 on auth failure.
 * The receive nonce only advances on success. */
int ssn_decrypt(ssn_t *st, const uint8_t *in, size_t len, uint8_t *out);
/* Skip one received message without decrypting (bumps receive nonce). */
void ssn_skip_recv(ssn_t *st);

#ifdef __cplusplus
}
#endif
#endif
