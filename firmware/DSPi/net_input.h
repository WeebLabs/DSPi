/*
 * net_input.h - WiFi / network audio input (INPUT_SOURCE_NET) for DSPi
 *
 * RP2350 + Raspberry Pi Pico 2 W only; compiled only when the build defines
 * DSPI_ENABLE_WIFI (CMake: -DDSPI_ENABLE_WIFI=ON -DPICO_BOARD=pico2_w).
 *
 * This module is the glue between the CYW43 radio / lwIP stack, the Sendspin
 * client (sendspin_client.c, which owns the TCP connection, the secure channel
 * and the audio framing) and DSPi's DSP pipeline:
 *
 *   sendspin_client --net_input_push_pcm()--> SPSC byte ring --net_input_poll()-->
 *   float conversion + preamp --> process_input_block()
 *
 * Everything here runs on Core 0 in main-loop context.  lwIP runs in "poll"
 * mode (pico_cyw43_arch_lwip_poll): the only place the network stack makes
 * progress is cyw43_arch_poll(), called from net_input_poll().
 *
 * Phase 1 limitations: WiFi credentials and the Sendspin server address are
 * RAM only (lost on reset, NOT saved in presets / flash).  The host re-sends
 * them with the vendor commands after every boot.
 */

#ifndef NET_INPUT_H
#define NET_INPUT_H

#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------------------------
// Lifecycle (called from main.c's source-switch / boot / flash-bracket paths)
// ---------------------------------------------------------------------------

// Bring up cyw43 + lwIP and start joining the configured WiFi network
// (non-blocking after the initial radio firmware load).  The selected network
// protocol client (net_proto_get(): Sendspin or Slimproto, never both) is
// started as soon as the link is up with an IP address, and stopped again if
// the link drops or the protocol selection changes.
void net_input_start(void);

// Stop the running protocol client, leave the network, cyw43_arch_deinit() (releases the
// PIO state machine and DMA channel), empty the ring, input_servo_reset().
// main.c restores the nominal output dividers (static there), as for I2S/ADAT.
void net_input_stop(void);

// Main-loop poll: cyw43_arch_poll(), WiFi state machine, <selected client>_poll(),
// then drain the PCM ring into process_input_block() in blocks of 48..192
// frames, paced against the output consumer pool.  Returns frames processed.
uint32_t net_input_poll(void);

// True while NET is the "locked" analogue: WiFi joined, format set, and PCM
// data flowing (data seen within the last ~400 ms or still buffered).
bool net_input_is_ready(void);

// True if cyw43/lwIP is initialised at all (between start and stop).
bool net_input_is_running(void);

// ---------------------------------------------------------------------------
// PCM path (called by the Sendspin client)
// ---------------------------------------------------------------------------

// Append interleaved little-endian PCM bytes (frame = channels * bits/8) to
// the ring.  Returns the number of bytes accepted (0 when full; the caller
// keeps the rest and retries, which backpressures the TCP window).
uint32_t net_input_push_pcm(const uint8_t *pcm, uint32_t bytes);

// Zero-copy-friendly variant used by the Sendspin client for audio chunks that
// are decrypted while they are still arriving (see sendspin_client.c): the
// plaintext is written into ring space that is still FREE (beyond the write
// index, invisible to the consumer) and published in one step once the record
// has been authenticated, or simply never published if it fails.
//   net_input_ring_capacity()    total ring size in bytes (what the client
//                                advertises as buffer_capacity; also the
//                                largest audio chunk that can ever be taken)
//   net_input_ring_free_bytes()  free space right now (only grows while the
//                                consumer drains)
//   net_input_stage_pcm()        copy len bytes to write_index + off without
//                                publishing; false if off + len > free space
//   net_input_commit_pcm()       publish the first `bytes` staged bytes
uint32_t net_input_ring_capacity(void);
uint32_t net_input_ring_free_bytes(void);
bool     net_input_stage_pcm(uint32_t off, const uint8_t *pcm, uint32_t len);
void     net_input_commit_pcm(uint32_t bytes);

// Declare the stream format.  Only 44100/48000 Hz, 2 channels, 16 or 24 bit
// are accepted (anything else is ignored).  A change of rate arms the same
// deferred perform_rate_change() path I2S/ADAT use; a format change empties
// the ring.
void net_input_set_format(uint32_t rate_hz, uint8_t channels, uint8_t bits);

// Drop all buffered audio (seek / stream end).  The input reports not-ready
// until new data arrives, which makes main.c mute and re-prefill.
void net_input_flush(void);

// Bytes currently buffered in the ring.
uint32_t net_input_ring_fill_bytes(void);

// Free-running (mod 2^32) byte positions of the ring write / read index.  The
// Slimproto client uses them to mark track boundaries and to derive elapsed
// play time from what the consumer has taken.
uint32_t net_input_ring_wr_pos(void);
uint32_t net_input_ring_rd_pos(void);

// Hold the consumer: while held net_input_poll() does not take data from the
// ring and net_input_is_ready() reports false (the pipeline mutes exactly as it
// does for a stalled stream) but the buffered audio is kept, so un-holding
// resumes where it stopped.  Used for Slimproto pause / "waiting for unpause".
// Cleared by net_input_start()/net_input_stop().
void net_input_set_hold(bool hold);
bool net_input_is_held(void);

// MAC address of the STA interface (CYW43), the Slimproto player id.  All zero
// until the radio has been initialised once.
void net_input_get_mac(uint8_t mac[6]);

// ---------------------------------------------------------------------------
// Volume / mute bridge (the client polls these to report/apply remote state)
// ---------------------------------------------------------------------------

// Percent 0..100 on the Sendspin gain curve gain = (pct/100)^1.5.
// Maps onto the user volume (update_user_volume(), dB, clamped to
// [-CENTER_VOLUME_INDEX, 0]); 0 maps to -CENTER_VOLUME_INDEX dB.
uint8_t net_volume_get_percent(void);
void    net_volume_set_percent(uint8_t pct);

// The vendor user_mute flag (same one REQ_SET_USER_MUTE drives).
bool net_mute_get(void);
void net_mute_set(bool mute);

// ---------------------------------------------------------------------------
// Configuration (RAM only in phase 1)
// ---------------------------------------------------------------------------

// Strings are copied (SSID <= 32 bytes, passphrase <= 63 bytes; an empty
// passphrase means an open network).  If the radio is running, the new
// credentials trigger a re-join.
void net_wifi_set_credentials(const char *ssid, const char *pass);
const char *net_wifi_get_ssid(void);
const char *net_wifi_get_pass(void);

// Network protocol selector (RAM + persisted via net_persist).  Exactly one
// client runs at a time (they share their RAM).  Changing the selection while
// the radio is up stops the running client and starts the new one.
typedef enum {
    NET_PROTO_SENDSPIN  = 0,   // default
    NET_PROTO_SLIMPROTO = 1,   // Squeezelite-compatible player (Music Assistant 'Squeezelite' provider)
} NetProto;
void    net_proto_set(uint8_t proto);       // values > NET_PROTO_SLIMPROTO are ignored
uint8_t net_proto_get(void);

// Server host: ONE host string for both protocols (net_sendspin_set_server()
// stores it); the port is per protocol.  Sendspin: port given to
// net_sendspin_set_server() (default 8927, WebSocket path "/sendspin").
// Slimproto: control port (default 3483) set with net_slimproto_set_port();
// 0 = default.  The Slimproto HTTP stream port/ip come from the server's strm
// command.
void net_slimproto_set_port(uint16_t port);
uint16_t net_slimproto_get_port(void);
uint16_t net_slimproto_port(void);          // alias used by slimproto_client.c

// Sendspin server (default port 8927, WebSocket path "/sendspin").
void net_sendspin_set_server(const char *host, uint16_t port);
const char *net_sendspin_get_host(void);
uint16_t net_sendspin_get_port(void);
const char *net_sendspin_get_path(void);
// Aliases used by sendspin_client.c
const char *net_sendspin_host(void);
uint16_t net_sendspin_port(void);

// ---------------------------------------------------------------------------
// Status
// ---------------------------------------------------------------------------

typedef enum {
    NET_WIFI_OFF            = 0,   // net_input not running
    NET_WIFI_INIT_FAILED    = 1,   // cyw43_arch_init() failed (retrying)
    NET_WIFI_NO_CREDENTIALS = 2,   // radio up, no SSID configured
    NET_WIFI_CONNECTING     = 3,   // join / DHCP in progress
    NET_WIFI_CONNECTED      = 4,   // link up with IP address
    NET_WIFI_FAILED         = 5,   // join failed (bad auth / no AP); retrying
} NetWifiState;

NetWifiState net_wifi_get_state(void);

// REQ_GET_NET_STATUS payload (little-endian, packed).
typedef struct __attribute__((packed)) {
    uint8_t  wifi_state;        // NetWifiState
    uint8_t  sendspin_state;    // state of the active protocol client: SENDSPIN_STATE_* or SLIMPROTO_STATE_*
                                // (same numeric meaning, selected by `protocol`; 0 if client not started)
    uint8_t  ready;             // net_input_is_ready()
    uint8_t  active;            // active_input_source == INPUT_SOURCE_NET
    uint32_t ip_addr;           // IPv4, network byte order as stored by lwIP (0 if none)
    uint32_t sample_rate;       // current stream rate (0 if format not set)
    uint32_t ring_fill_bytes;
    uint32_t ring_size_bytes;
    uint32_t ring_overflows;    // push attempts that were truncated/refused
    uint32_t ring_underruns;    // times the pipeline wanted data and the ring was empty
    int8_t   rssi_dbm;          // 0 if unknown / not connected
    uint8_t  bits;              // stream bit depth (0 if format not set)
    uint8_t  protocol;          // NetProto selected (v2 firmware; older firmware reports 0 = Sendspin here)
    uint8_t  reserved;          // 0
} NetStatusPacket;                // still 32 bytes: the new byte was formerly reserved[0] (always 0)
_Static_assert(sizeof(NetStatusPacket) == 32, "NetStatusPacket wire size");

void net_input_get_status(NetStatusPacket *out);

#endif // NET_INPUT_H
