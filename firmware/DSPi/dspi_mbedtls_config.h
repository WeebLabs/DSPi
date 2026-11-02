/*
 * dspi_mbedtls_config.h - minimal Mbed TLS configuration for the DSPi
 * WiFi/Sendspin input (-DDSPI_ENABLE_WIFI=ON).
 *
 * Selected through PICO_MBEDTLS_CONFIG_FILE in the top-level CMakeLists.txt.
 * Only the primitives the Sendspin secure channel needs: X25519 ECDH,
 * ChaCha20-Poly1305, SHA-256, HMAC/HKDF, plus a hardware-seeded CTR_DRBG
 * (entropy comes from pico_mbedtls.c's mbedtls_hardware_poll()).  No TLS, no
 * X.509, no NIST curves, no PSA: the SDK still lists those source files but
 * they compile to nothing under this config.
 */

#ifndef DSPI_MBEDTLS_CONFIG_H
#define DSPI_MBEDTLS_CONFIG_H

// mbedtls 3.x hides struct fields; sendspin_noise.c reads mbedtls_ecp_point.X
// directly for the raw X25519 scalar multiplication.
#define MBEDTLS_ALLOW_PRIVATE_ACCESS

#define MBEDTLS_NO_PLATFORM_ENTROPY
#define MBEDTLS_ENTROPY_HARDWARE_ALT
#define MBEDTLS_ENTROPY_FORCE_SHA256

// Big-number + elliptic curve (Curve25519 only)
#define MBEDTLS_BIGNUM_C
#define MBEDTLS_ECP_C
#define MBEDTLS_ECDH_C
#define MBEDTLS_ECP_DP_CURVE25519_ENABLED
#define MBEDTLS_ECP_WINDOW_SIZE        2
#define MBEDTLS_ECP_FIXED_POINT_OPTIM  0

// AEAD
#define MBEDTLS_CHACHA20_C
#define MBEDTLS_POLY1305_C
#define MBEDTLS_CHACHAPOLY_C

// Hash / MAC / KDF
#define MBEDTLS_SHA256_C
#define MBEDTLS_SHA1_C            // WebSocket Sec-WebSocket-Accept check
#define MBEDTLS_MD_C
#define MBEDTLS_HKDF_C

// RNG: CTR_DRBG over the hardware entropy source
#define MBEDTLS_ENTROPY_C
#define MBEDTLS_CTR_DRBG_C
#define MBEDTLS_AES_C
#define MBEDTLS_AES_ROM_TABLES

// Size over speed where mbedtls offers the choice
#define MBEDTLS_AES_FEWER_TABLES
#define MBEDTLS_SHA256_SMALLER

#endif // DSPI_MBEDTLS_CONFIG_H
