/*
 * Minimal mbedtls 3.6 config for the Sendspin host test.  The firmware itself
 * (dspi_mbedtls_config.h) needs only CHACHA20, POLY1305, CHACHAPOLY, SHA256 and
 * SHA1 for sendspin_noise.c / sendspin_client.c: X25519 is local, heap-free code.
 * MD_C/HMAC, BIGNUM, ECP, ECDH and BASE64 are enabled here only for the test's
 * independent initiator and as a reference implementation for the X25519
 * cross-check.
 */
#ifndef MBEDTLS_CONFIG_TEST_H
#define MBEDTLS_CONFIG_TEST_H

#define MBEDTLS_CHACHA20_C
#define MBEDTLS_POLY1305_C
#define MBEDTLS_CHACHAPOLY_C
#define MBEDTLS_SHA256_C
#define MBEDTLS_SHA1_C
#define MBEDTLS_MD_C
#define MBEDTLS_BIGNUM_C
#define MBEDTLS_ECP_C
#define MBEDTLS_ECP_DP_CURVE25519_ENABLED
#define MBEDTLS_ECDH_C
#define MBEDTLS_PLATFORM_C
#define MBEDTLS_BASE64_C
#define MBEDTLS_NO_PLATFORM_ENTROPY
#define MBEDTLS_ECP_NIST_OPTIM

#endif
