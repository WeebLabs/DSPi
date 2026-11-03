/*
 * net_input.c - WiFi / network audio input (INPUT_SOURCE_NET)
 *
 * See net_input.h for the architecture overview.  Built only with
 * -DDSPI_ENABLE_WIFI=ON (RP2350 + Pico 2 W).
 *
 * Threading: everything runs on Core 0 in main-loop context (the vendor
 * command handlers also run there for UART/I2C, and from the USB task for
 * USB control transfers, which tud_task() services from the same loop).  The
 * PCM ring is therefore effectively single-threaded, but the indices are
 * still published with barriers so a future move of the producer to another
 * context stays safe (true SPSC).
 */

#ifdef DSPI_ENABLE_WIFI

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/sync.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"

#include "net_input.h"
#include "sendspin_client.h"
#include "slimproto_client.h"
#include "config.h"
#include "audio_input.h"
#include "i2s_input.h"
#include "audio_pipeline.h"
#include "input_servo.h"
#include "usb_audio.h"
#include "bulk_params.h"
#include "notify.h"

// ---------------------------------------------------------------------------
// Configuration (RAM only, phase 1)
// ---------------------------------------------------------------------------

#define NET_SSID_MAX      32
#define NET_PASS_MAX      63
#define NET_HOST_MAX      63
#define NET_DEFAULT_PORT  8927
#define NET_DEFAULT_PATH  "/sendspin"

static char     net_ssid[NET_SSID_MAX + 1];
static char     net_pass[NET_PASS_MAX + 1];
static char     net_host[NET_HOST_MAX + 1];
static uint16_t net_port = NET_DEFAULT_PORT;
static const char net_path[] = NET_DEFAULT_PATH;
#define NET_SLIM_DEFAULT_PORT 3483
static uint16_t net_slim_port = NET_SLIM_DEFAULT_PORT;
static uint8_t  net_proto = NET_PROTO_SENDSPIN;   // selected protocol
static uint8_t  client_proto;                     // protocol of the started client (valid while client_started)
static volatile bool ring_hold;                   // see net_input_set_hold()

// Retry cadence for failed init / join
#define NET_RETRY_MS        5000u
// Data gap after which an empty ring is declared "stream not active"
#define NET_STREAM_IDLE_US  400000u

// ---------------------------------------------------------------------------
// PCM ring (byte-oriented SPSC, power-of-two size)
// ---------------------------------------------------------------------------

#ifndef NET_RING_BYTES
#define NET_RING_BYTES      32768u
#endif
#define NET_RING_MASK       (NET_RING_BYTES - 1u)
_Static_assert((NET_RING_BYTES & (NET_RING_BYTES - 1u)) == 0, "ring must be a power of two");

// Backing store is borrowed from the I2S RX DMA rings (idle while NET is the
// source; see i2s_input_ring_storage()) to save 32 KB of .bss.  NULL while the
// input is stopped, which also makes push_pcm refuse data.
//
// Aliasing invariants (RP2350: 4 pairs x 8 KB == exactly NET_RING_BYTES, so the
// ring takes ALL of i2s_rx_ring and nothing else of it is left over):
//  - the ring is only touched between net_input_start() and net_input_stop();
//    main.c stops one source completely before starting another and starts no
//    I2S RX while NET is the active source, so the DMA never writes it then;
//  - bytes [rd, wr) are owned by the consumer (net_input_poll), bytes
//    [wr, rd + NET_RING_BYTES) by the producer: net_input_stage_pcm() writes
//    there without publishing, net_input_commit_pcm()/push_pcm() publish by
//    advancing wr.  A 150 ms Sendspin chunk (28.8 KB) is decrypted straight
//    into that producer area, so no separate receive buffer is needed.
static uint8_t *net_ring;
static volatile uint32_t ring_wr;   // free-running byte counters (wrap mod 2^32)
static volatile uint32_t ring_rd;

static inline uint32_t ring_fill(void) { return ring_wr - ring_rd; }

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static bool          net_running;          // between start and stop
static bool          cyw43_up;             // cyw43_arch_init() succeeded
static bool          client_started;
static NetWifiState  wifi_state = NET_WIFI_OFF;
static bool          join_in_progress;
static bool          rejoin_requested;     // credentials changed while running
static absolute_time_t next_retry;

static uint32_t      fmt_rate;             // 0 = format not set
static uint8_t       fmt_channels;
static uint8_t       fmt_bits;
static uint32_t      fmt_bytes_per_frame = 4;

static volatile bool stream_active;
static uint64_t      last_push_us;
static uint32_t      ring_overflows;
static uint32_t      ring_underruns;

// Last volume we applied, so a get right after a set round-trips exactly even
// where the dB mapping is lossy (0 and 1 percent both clamp to the floor).
static uint8_t       last_set_pct = 0xFF;
static int16_t       last_set_vol_8_8;

// ---------------------------------------------------------------------------
// WiFi
// ---------------------------------------------------------------------------

static void net_begin_join(void) {
    join_in_progress = false;
    if (!cyw43_up) return;
    if (net_ssid[0] == '\0') {
        wifi_state = NET_WIFI_NO_CREDENTIALS;
        return;
    }
    uint32_t auth = net_pass[0] ? CYW43_AUTH_WPA2_MIXED_PSK : CYW43_AUTH_OPEN;
    int rc = cyw43_arch_wifi_connect_async(net_ssid, net_pass[0] ? net_pass : NULL, auth);
    if (rc != 0) {
        wifi_state = NET_WIFI_FAILED;
        next_retry = make_timeout_time_ms(NET_RETRY_MS);
        printf("net: connect_async failed (%d)\n", rc);
        return;
    }
    join_in_progress = true;
    wifi_state = NET_WIFI_CONNECTING;
}

static void net_client_stop_if_started(void) {
    if (client_started) {
        if (client_proto == NET_PROTO_SLIMPROTO) slimproto_client_stop();
        else sendspin_client_stop();
        client_started = false;
    }
}

static void net_client_start(void) {
    client_proto = net_proto;
    client_started = true;
    if (client_proto == NET_PROTO_SLIMPROTO) slimproto_client_start();
    else sendspin_client_start();
}

static bool net_radio_init(void) {
    if (cyw43_arch_init() != 0) {
        wifi_state = NET_WIFI_INIT_FAILED;
        next_retry = make_timeout_time_ms(NET_RETRY_MS);
        printf("net: cyw43_arch_init failed\n");
        return false;
    }
    cyw43_up = true;
    cyw43_arch_enable_sta_mode();
    // Streaming audio: favour latency over the few mA power-save saves.
    cyw43_wifi_pm(&cyw43_state, cyw43_pm_value(CYW43_NO_POWERSAVE_MODE, 20, 1, 1, 1));
    netif_set_hostname(&cyw43_state.netif[CYW43_ITF_STA], "dspi");
    net_begin_join();
    return true;
}

static void net_wifi_service(void) {
    if (!cyw43_up) {
        if (absolute_time_diff_us(get_absolute_time(), next_retry) <= 0) net_radio_init();
        return;
    }

    if (rejoin_requested) {
        rejoin_requested = false;
        net_client_stop_if_started();
        cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
        net_begin_join();
        return;
    }

    int link = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
    switch (link) {
    case CYW43_LINK_UP:
        if (wifi_state != NET_WIFI_CONNECTED) {
            wifi_state = NET_WIFI_CONNECTED;
            join_in_progress = false;
            printf("net: WiFi up, ip %s\n",
                   ip4addr_ntoa(netif_ip4_addr(&cyw43_state.netif[CYW43_ITF_STA])));
        }
        // Protocol switched while running: stop the old client completely (it
        // releases its pcbs and RAM) before the new one is started next pass.
        if (client_started && client_proto != net_proto) net_client_stop_if_started();
        if (!client_started) net_client_start();
        break;
    case CYW43_LINK_FAIL:
    case CYW43_LINK_NONET:
    case CYW43_LINK_BADAUTH:
        // Join failed or the link dropped: tear the client down, back off,
        // then retry the whole join.
        net_client_stop_if_started();
        if (wifi_state == NET_WIFI_CONNECTED || join_in_progress) {
            wifi_state = NET_WIFI_FAILED;
            join_in_progress = false;
            next_retry = make_timeout_time_ms(NET_RETRY_MS);
            printf("net: link status %d\n", link);
        } else if (wifi_state == NET_WIFI_FAILED &&
                   absolute_time_diff_us(get_absolute_time(), next_retry) <= 0) {
            net_begin_join();
        } else if (wifi_state == NET_WIFI_NO_CREDENTIALS) {
            // wait for credentials
        }
        break;
    default:    // DOWN / JOIN / NOIP: in progress
        if (wifi_state == NET_WIFI_CONNECTED) {
            // Lost the link mid-session and the driver is re-associating.
            net_client_stop_if_started();
            wifi_state = NET_WIFI_CONNECTING;
        }
        break;
    }
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void net_input_start(void) {
    if (net_running) return;
    uint32_t ring_bytes = 0;
    net_ring = (uint8_t *)i2s_input_ring_storage(&ring_bytes);
    if (!net_ring || ring_bytes < NET_RING_BYTES) { net_ring = NULL; return; }
    net_running = true;
    ring_wr = ring_rd = 0;
    stream_active = false;
    last_push_us = 0;
    client_started = false;
    rejoin_requested = false;
    join_in_progress = false;
    ring_hold = false;
    wifi_state = NET_WIFI_OFF;
    // Firmware download to the radio blocks for a few hundred ms.
    net_radio_init();
}

void net_input_stop(void) {
    if (!net_running) return;
    net_client_stop_if_started();
    if (cyw43_up) {
        cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
        cyw43_arch_deinit();   // releases the PIO SM, program and DMA channel
        cyw43_up = false;
    }
    net_running = false;
    net_ring = NULL;
    ring_hold = false;
    join_in_progress = false;
    wifi_state = NET_WIFI_OFF;
    stream_active = false;
    ring_wr = ring_rd = 0;
    input_servo_reset();
}

bool net_input_is_running(void) { return net_running; }

bool net_input_is_ready(void) {
    return net_running && wifi_state == NET_WIFI_CONNECTED &&
           fmt_rate != 0 && stream_active && !ring_hold;
}

// ---------------------------------------------------------------------------
// PCM push / format / flush
// ---------------------------------------------------------------------------

uint32_t net_input_push_pcm(const uint8_t *pcm, uint32_t bytes) {
    if (!pcm || bytes == 0 || !net_ring) return 0;
    uint32_t wr = ring_wr;
    uint32_t free_b = NET_RING_BYTES - (wr - ring_rd);
    uint32_t n = bytes < free_b ? bytes : free_b;
    if (n < bytes) ring_overflows++;
    if (n == 0) return 0;

    uint32_t off = wr & NET_RING_MASK;
    uint32_t first = NET_RING_BYTES - off;
    if (first > n) first = n;
    memcpy(&net_ring[off], pcm, first);
    if (n > first) memcpy(&net_ring[0], pcm + first, n - first);

    __dmb();
    ring_wr = wr + n;
    last_push_us = time_us_64();
    stream_active = true;
    return n;
}

uint32_t net_input_ring_capacity(void) { return NET_RING_BYTES; }

uint32_t net_input_ring_free_bytes(void) {
    if (!net_ring) return 0;
    return NET_RING_BYTES - (ring_wr - ring_rd);
}

// Staging area = ring bytes [wr, rd + NET_RING_BYTES): the consumer only reads
// [rd, wr), and only the (single) producer context moves wr, so writing there
// is race-free; the area can only grow while the consumer advances rd.
bool net_input_stage_pcm(uint32_t off, const uint8_t *pcm, uint32_t len) {
    if (!net_ring || !pcm) return false;
    uint32_t wr = ring_wr;
    uint32_t free_b = NET_RING_BYTES - (wr - ring_rd);
    if (off > free_b || len > free_b - off) return false;
    uint32_t pos = (wr + off) & NET_RING_MASK;
    uint32_t first = NET_RING_BYTES - pos;
    if (first > len) first = len;
    memcpy(&net_ring[pos], pcm, first);
    if (len > first) memcpy(&net_ring[0], pcm + first, len - first);
    return true;
}

void net_input_commit_pcm(uint32_t bytes) {
    if (!net_ring || bytes == 0) return;
    uint32_t wr = ring_wr;
    uint32_t free_b = NET_RING_BYTES - (wr - ring_rd);
    if (bytes > free_b) { bytes = free_b; ring_overflows++; }   // not reachable under the staging contract
    __dmb();
    ring_wr = wr + bytes;
    last_push_us = time_us_64();
    stream_active = true;
}

void net_input_flush(void) {
    // Consumer-side reset: move the read index up to the write index.
    ring_rd = ring_wr;
    __dmb();
    stream_active = false;
    last_push_us = 0;
}

uint32_t net_input_ring_fill_bytes(void) { return ring_fill(); }
uint32_t net_input_ring_wr_pos(void) { return ring_wr; }
uint32_t net_input_ring_rd_pos(void) { return ring_rd; }
void net_input_set_hold(bool hold) { ring_hold = hold; __dmb(); }
bool net_input_is_held(void) { return ring_hold; }

void net_input_get_mac(uint8_t mac[6]) {
    memcpy(mac, cyw43_state.netif[CYW43_ITF_STA].hwaddr, 6);
}

void net_input_set_format(uint32_t rate_hz, uint8_t channels, uint8_t bits) {
    if (rate_hz != 44100 && rate_hz != 48000) return;
    if (channels != 2) return;
    if (bits != 16 && bits != 24) return;

    bool changed = (rate_hz != fmt_rate) || (channels != fmt_channels) || (bits != fmt_bits);
    fmt_channels = channels;
    fmt_bits = bits;
    fmt_bytes_per_frame = (uint32_t)channels * (bits / 8u);
    if (changed) net_input_flush();     // old-format bytes are meaningless now
    fmt_rate = rate_hz;

    // Same pattern as I2S/ADAT master: arm a deferred rate change; the main
    // loop runs perform_rate_change(r, true) and the NET prefill block owns
    // the output restart.  Only when NET really owns the pipeline.
    if (active_input_source == INPUT_SOURCE_NET && rate_hz != audio_state.freq) {
        pending_rate = rate_hz;
        __dmb();
        rate_change_pending = true;
    }
}

// ---------------------------------------------------------------------------
// Poll / drain into the pipeline
// ---------------------------------------------------------------------------

#define NET_MIN_BLOCK_FRAMES   48u
#define NET_MAX_BLOCK_FRAMES   192u
#define NET_MAX_BLOCKS_PER_POLL 2u

static inline int32_t ring_s16(uint32_t pos) {
    return (int16_t)(net_ring[pos & NET_RING_MASK] |
                     ((uint16_t)net_ring[(pos + 1u) & NET_RING_MASK] << 8));
}

static inline int32_t ring_s24(uint32_t pos) {
    uint32_t v = (uint32_t)net_ring[pos & NET_RING_MASK] |
                 ((uint32_t)net_ring[(pos + 1u) & NET_RING_MASK] << 8) |
                 ((uint32_t)net_ring[(pos + 2u) & NET_RING_MASK] << 16);
    return ((int32_t)(v << 8)) >> 8;
}

// Convert `frames` frames from the ring into buf_l/buf_r (float, with preamp)
// and advance the read index.
static void net_convert_block(uint32_t frames) {
    float preamp_l = global_preamp_linear[0];
    float preamp_r = global_preamp_linear[1];
    uint32_t pos = ring_rd;

    if (fmt_bits == 16) {
        const float k = 1.0f / 32768.0f;
        float gl = k * preamp_l, gr = k * preamp_r;
        for (uint32_t i = 0; i < frames; i++) {
            buf_l[i] = (float)ring_s16(pos) * gl;
            buf_r[i] = (float)ring_s16(pos + 2u) * gr;
            pos += 4u;
        }
    } else {
        const float k = 1.0f / 8388608.0f;
        float gl = k * preamp_l, gr = k * preamp_r;
        for (uint32_t i = 0; i < frames; i++) {
            buf_l[i] = (float)ring_s24(pos) * gl;
            buf_r[i] = (float)ring_s24(pos + 3u) * gr;
            pos += 6u;
        }
    }
    __dmb();
    ring_rd = pos;
}

uint32_t net_input_poll(void) {
    if (!net_running) return 0;

    cyw43_arch_poll();
    net_wifi_service();
    if (client_started) {
        if (client_proto == NET_PROTO_SLIMPROTO) slimproto_client_poll();
        else sendspin_client_poll();
    }

    if (!net_input_is_ready()) return 0;

    // Declare the stream idle when the sender went quiet and we ran dry.
    uint32_t bpf = fmt_bytes_per_frame;
    uint32_t frames_avail = ring_fill() / bpf;
    if (frames_avail < NET_MIN_BLOCK_FRAMES) {
        if (ring_fill() == 0 && (time_us_64() - last_push_us) > NET_STREAM_IDLE_US)
            stream_active = false;
        return 0;
    }

    // Pace against the output side: the outputs (not the network) set the
    // rate, so only top the consumer pool up to 50% like the siggen pump.
    // Producing faster would just overrun the producer pools.  This is also
    // what lets main.c's prefill (wait for 50% fill) complete.
    uint32_t total = 0;
    for (uint32_t b = 0; b < NET_MAX_BLOCKS_PER_POLL; b++) {
        if (get_slot_consumer_fill(0) >= SPDIF_CONSUMER_BUFFER_COUNT / 2) break;
        frames_avail = ring_fill() / bpf;
        if (frames_avail < NET_MIN_BLOCK_FRAMES) {
            ring_underruns++;
            break;
        }
        uint32_t frames = frames_avail > NET_MAX_BLOCK_FRAMES ? NET_MAX_BLOCK_FRAMES : frames_avail;
        net_convert_block(frames);
        process_input_block(frames);
        total += frames;
    }
    return total;
}

// ---------------------------------------------------------------------------
// Volume / mute bridge
// ---------------------------------------------------------------------------

// Sendspin gain curve: gain = (pct/100)^1.5, dB = 20*log10(gain) = 30*log10(pct/100).
static float net_pct_to_db(uint8_t pct) {
    if (pct == 0) return -(float)CENTER_VOLUME_INDEX;
    if (pct > 100) pct = 100;
    float db = 30.0f * log10f((float)pct / 100.0f);
    if (db < -(float)CENTER_VOLUME_INDEX) db = -(float)CENTER_VOLUME_INDEX;
    if (db > 0.0f) db = 0.0f;
    return db;
}

void net_volume_set_percent(uint8_t pct) {
    if (pct > 100) pct = 100;
    float db = net_pct_to_db(pct);
    update_user_volume(db);
    last_set_pct = pct;
    last_set_vol_8_8 = audio_state.volume;   // what update_user_volume stored
}

uint8_t net_volume_get_percent(void) {
    int16_t v = audio_state.volume;
    if (last_set_pct != 0xFF && v == last_set_vol_8_8) return last_set_pct;
    float db = (float)v / 256.0f;
    if (db >= 0.0f) return 100;
    if (db <= -(float)CENTER_VOLUME_INDEX) return 0;
    float pct = 100.0f * powf(10.0f, db / 30.0f);
    int p = (int)lrintf(pct);
    if (p < 0) p = 0;
    if (p > 100) p = 100;
    return (uint8_t)p;
}

bool net_mute_get(void) { return user_mute; }

void net_mute_set(bool mute) {
    user_mute = mute;
    uint8_t v = mute ? 1 : 0;
    notify_param_write(offsetof(WireBulkParams, user_volume.user_mute),
                       sizeof(uint8_t), &v);
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

void net_wifi_set_credentials(const char *ssid, const char *pass) {
    strncpy(net_ssid, ssid ? ssid : "", NET_SSID_MAX);
    net_ssid[NET_SSID_MAX] = '\0';
    strncpy(net_pass, pass ? pass : "", NET_PASS_MAX);
    net_pass[NET_PASS_MAX] = '\0';
    if (net_running) rejoin_requested = true;
}
const char *net_wifi_get_ssid(void) { return net_ssid; }
const char *net_wifi_get_pass(void) { return net_pass; }

void net_sendspin_set_server(const char *host, uint16_t port) {
    strncpy(net_host, host ? host : "", NET_HOST_MAX);
    net_host[NET_HOST_MAX] = '\0';
    net_port = port ? port : NET_DEFAULT_PORT;
}
void net_proto_set(uint8_t proto) {
    if (proto > NET_PROTO_SLIMPROTO) return;
    net_proto = proto;          // net_wifi_service() restarts the client on its next pass
}
uint8_t net_proto_get(void) { return net_proto; }
void net_slimproto_set_port(uint16_t port) { net_slim_port = port ? port : NET_SLIM_DEFAULT_PORT; }
uint16_t net_slimproto_get_port(void) { return net_slim_port; }
uint16_t net_slimproto_port(void) { return net_slim_port; }
const char *net_sendspin_get_host(void) { return net_host; }
uint16_t net_sendspin_get_port(void) { return net_port; }
const char *net_sendspin_get_path(void) { return net_path; }
// Names used by sendspin_client.c
const char *net_sendspin_host(void) { return net_host; }
uint16_t net_sendspin_port(void) { return net_port; }

// ---------------------------------------------------------------------------
// Status
// ---------------------------------------------------------------------------

NetWifiState net_wifi_get_state(void) { return wifi_state; }

void net_input_get_status(NetStatusPacket *out) {
    memset(out, 0, sizeof(*out));
    out->wifi_state     = (uint8_t)wifi_state;
    out->protocol       = net_proto;
    out->sendspin_state = !client_started ? 0
                        : (client_proto == NET_PROTO_SLIMPROTO ? slimproto_client_state()
                                                               : sendspin_client_state());
    out->ready          = net_input_is_ready() ? 1 : 0;
    out->active         = (active_input_source == INPUT_SOURCE_NET) ? 1 : 0;
    out->sample_rate    = fmt_rate;
    out->bits           = fmt_bits;
    out->ring_fill_bytes = ring_fill();
    out->ring_size_bytes = NET_RING_BYTES;
    out->ring_overflows  = ring_overflows;
    out->ring_underruns  = ring_underruns;
    if (cyw43_up && wifi_state == NET_WIFI_CONNECTED) {
        out->ip_addr = netif_ip4_addr(&cyw43_state.netif[CYW43_ITF_STA])->addr;
        int32_t rssi = 0;
        if (cyw43_wifi_get_rssi(&cyw43_state, &rssi) == 0)
            out->rssi_dbm = (int8_t)(rssi < -127 ? -127 : (rssi > 0 ? 0 : rssi));
    }
}

#endif // DSPI_ENABLE_WIFI
