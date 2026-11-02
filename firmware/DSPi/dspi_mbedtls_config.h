/*
 * dspi_mbedtls_config.h - minimal Mbed TLS configuration for the DSPi
 * WiFi/Sendspin input (-DDSPI_ENABLE_WIFI=ON).
 *
 * Selected through PICO_MBEDTLS_CONFIG_FILE in the top-level CMakeLists.txt.
 * Only the primitives the Sendspin secure channel (sendspin_noise.c) needs:
 * ChaCha20-Poly1305 (streaming API), SHA-256 (HMAC/HKDF are local code) and
 * SHA-1 (WebSocket Sec-WebSocket-Accept).
 *
 * Deliberately NOT enabled any more:
 *   - BIGNUM / ECP / ECDH: X25519 is a heap-free ref10-style routine in
 *     sendspin_noise.c.  mbedtls' bignum allocated ~1.2 KB of heap in ~3100
 *     calloc/free pairs per scalar multiplication (measured on the host test
 *     rig with a counting allocator) and its call chain needed ~1 KB of stack.
 *     The firmware heap (__end__ .. __HeapLimit) is only a few KB in this build
 *     and an allocation failure panics (pico_malloc), so the crypto must not
 *     malloc at all.  With these options off, no mbedtls code can reach
 *     malloc/calloc/free: ChaChaPoly, Poly1305 and SHA contexts live on the
 *     caller's stack / in static state.
 *   - CTR_DRBG / ENTROPY / AES: nothing uses them (random numbers come from
 *     pico_rand's get_rand_32() in sendspin_client.c).
 *   - TLS, X.509, NIST curves, PSA: the SDK still lists those source files but
 *     they compile to nothing under this config.
 */

#ifndef DSPI_MBEDTLS_CONFIG_H
#define DSPI_MBEDTLS_CONFIG_H

#define MBEDTLS_NO_PLATFORM_ENTROPY

// AEAD (used through mbedtls_chachapoly_*; streaming update/finish for big records)
#define MBEDTLS_CHACHA20_C
#define MBEDTLS_POLY1305_C
#define MBEDTLS_CHACHAPOLY_C

// Hashes
#define MBEDTLS_SHA256_C
#define MBEDTLS_SHA1_C            // WebSocket Sec-WebSocket-Accept check

// Size over speed where mbedtls offers the choice
#define MBEDTLS_SHA256_SMALLER

#endif // DSPI_MBEDTLS_CONFIG_H
