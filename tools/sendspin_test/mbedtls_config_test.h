/*
 * Minimal mbedtls 3.6 config for the Sendspin host test. This is also the
 * list of features the firmware's mbedtls config must provide for
 * sendspin_noise.c / sendspin_client.c:
 *   CHACHA20, POLY1305, CHACHAPOLY, SHA256, SHA1, ECP + CURVE25519, BIGNUM.
 * (MD_C and HMAC are only used by the test's independent initiator.)
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
