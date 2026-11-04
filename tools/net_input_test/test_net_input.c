/* Host test for firmware/DSPi/net_input.c.  The unit is #included so the tests
 * can reach its static state; all SDK/lwIP/pipeline deps are faked below. */
#include "dspi_stubs.h"
#include "net_input.c"
#include <math.h>
#include <stdlib.h>

static int fails, checks;
#define CHECK(c) do { checks++; if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

/* ---- fakes ---- */
uint64_t fake_time_us = 1000000;
cyw43_t cyw43_state;
int active_input_source;
AudioState audio_state;
bool user_mute;
uint32_t pending_rate;
volatile bool rate_change_pending;
float global_preamp_linear[2] = {1.0f, 1.0f};
float buf_l[512], buf_r[512];
static uint8_t ring_mem[NET_RING_BYTES];
static int fake_link = CYW43_LINK_DOWN, fake_init_rc, n_init, n_connect, n_leave, n_deinit;
static int n_ss_start, n_ss_stop, n_sl_start, n_sl_stop, n_ss_poll, n_sl_poll;
static int consumer_fill, n_blocks, servo_resets;
static uint32_t block_frames[16];
static float block_l0[16];
static uint32_t last_auth; static char last_ssid[64]; static bool last_pw_null;
static uint8_t notify_val; static uint32_t notify_off;

int cyw43_arch_init(void) { n_init++; return fake_init_rc; }
void cyw43_arch_deinit(void) { n_deinit++; }
void cyw43_arch_enable_sta_mode(void) {}
void cyw43_arch_poll(void) {}
int cyw43_arch_wifi_connect_async(const char *s, const char *p, uint32_t a) {
    n_connect++; last_auth = a; last_pw_null = !p; snprintf(last_ssid, sizeof last_ssid, "%s", s); return 0; }
int cyw43_wifi_leave(cyw43_t *s, int i) { n_leave++; return 0; }
int cyw43_tcpip_link_status(cyw43_t *s, int i) { return fake_link; }
int cyw43_wifi_get_rssi(cyw43_t *s, int32_t *r) { *r = -200; return 0; }
int cyw43_wifi_pm(cyw43_t *s, uint32_t pm) { return 0; }
void update_user_volume(float db) { audio_state.volume = (int16_t)lrintf(db * 256.0f); }
void notify_param_write(uint32_t off, uint32_t len, const void *d) { notify_off = off; notify_val = *(const uint8_t *)d; }
void *i2s_input_ring_storage(uint32_t *b) { *b = sizeof ring_mem; return ring_mem; }
void input_servo_reset(void) { servo_resets++; }
int get_slot_consumer_fill(int s) { return consumer_fill; }
void process_input_block(uint32_t frames) {
    if (n_blocks < 16) { block_frames[n_blocks] = frames; block_l0[n_blocks] = buf_l[0]; }
    n_blocks++;
}
void sendspin_client_start(void) { n_ss_start++; } void sendspin_client_stop(void) { n_ss_stop++; }
void sendspin_client_poll(void) { n_ss_poll++; } uint8_t sendspin_client_state(void) { return 7; }
void slimproto_client_start(void) { n_sl_start++; } void slimproto_client_stop(void) { n_sl_stop++; }
void slimproto_client_poll(void) { n_sl_poll++; } uint8_t slimproto_client_state(void) { return 9; }

static void reset_all(void) {
    if (net_running) net_input_stop();
    fake_link = CYW43_LINK_DOWN; fake_init_rc = 0;
    n_init = n_connect = n_leave = n_deinit = 0;
    n_ss_start = n_ss_stop = n_sl_start = n_sl_stop = n_ss_poll = n_sl_poll = 0;
    consumer_fill = 0; n_blocks = 0; servo_resets = 0;
    active_input_source = 0; audio_state.volume = 0; audio_state.freq = 48000;
    rate_change_pending = false; pending_rate = 0;
    global_preamp_linear[0] = global_preamp_linear[1] = 1.0f;
    ring_overflows = ring_underruns = 0;
    fmt_rate = 0; fmt_channels = 0; fmt_bits = 0; fmt_bytes_per_frame = 4;
    net_ssid[0] = net_pass[0] = net_host[0] = 0; net_port = NET_DEFAULT_PORT;
    net_slim_port = NET_SLIM_DEFAULT_PORT; net_proto = NET_PROTO_SENDSPIN;
    last_set_pct = 0xFF; wifi_state = NET_WIFI_OFF; cyw43_up = false;
}
/* Bring the unit to "connected, stream-ready" with a given format. */
static void make_ready(uint32_t rate, uint8_t bits) {
    reset_all();
    net_wifi_set_credentials("ap", "pw");
    net_input_start();
    fake_link = CYW43_LINK_UP;
    net_input_poll();
    net_input_set_format(rate, 2, bits);
    net_input_set_hold(false);
}

static void test_ring(void) {
    reset_all();
    uint8_t in[100], out[100];
    for (int i = 0; i < 100; i++) in[i] = (uint8_t)i;
    CHECK(net_input_push_pcm(in, 10) == 0);              /* stopped: refused */
    CHECK(net_input_ring_free_bytes() == 0);
    net_input_start();
    CHECK(net_input_ring_capacity() == NET_RING_BYTES);
    CHECK(net_input_ring_free_bytes() == NET_RING_BYTES);
    CHECK(net_input_push_pcm(NULL, 5) == 0 && net_input_push_pcm(in, 0) == 0);
    CHECK(net_input_push_pcm(in, 100) == 100);
    CHECK(net_input_ring_fill_bytes() == 100 && stream_active);
    CHECK(memcmp(net_ring, in, 100) == 0);

    /* fill to the brim, overflow is truncated + counted */
    static uint8_t big[NET_RING_BYTES];
    memset(big, 0xAB, sizeof big);
    CHECK(net_input_push_pcm(big, NET_RING_BYTES) == NET_RING_BYTES - 100);
    CHECK(ring_overflows == 1 && net_input_ring_free_bytes() == 0);
    CHECK(net_input_push_pcm(big, 1) == 0 && ring_overflows == 2);

    /* wrap-around write */
    ring_rd = ring_wr - 0;           /* drain */
    ring_wr = ring_rd = NET_RING_BYTES - 40;
    CHECK(net_input_push_pcm(in, 100) == 100);
    for (int i = 0; i < 100; i++) out[i] = net_ring[(ring_rd + i) & NET_RING_MASK];
    CHECK(memcmp(in, out, 100) == 0);
    CHECK(net_ring[0] == in[40]);

    /* free-running 32-bit counter wrap */
    ring_wr = ring_rd = 0xFFFFFFF0u;
    CHECK(net_input_push_pcm(in, 64) == 64);
    CHECK(ring_wr == 0xFFFFFFF0u + 64 && net_input_ring_fill_bytes() == 64);
    CHECK(net_input_ring_free_bytes() == NET_RING_BYTES - 64);
    CHECK(net_input_ring_wr_pos() == ring_wr && net_input_ring_rd_pos() == ring_rd);

    /* flush */
    net_input_flush();
    CHECK(net_input_ring_fill_bytes() == 0 && !stream_active && last_push_us == 0);
}

static void test_stage_commit(void) {
    reset_all();
    uint8_t in[64];
    for (int i = 0; i < 64; i++) in[i] = (uint8_t)(i + 1);
    CHECK(!net_input_stage_pcm(0, in, 4));               /* not started */
    net_input_start();
    ring_wr = ring_rd = NET_RING_BYTES - 16;             /* stage across the wrap */
    CHECK(net_input_stage_pcm(0, in, 32));
    CHECK(net_input_stage_pcm(32, in + 32, 32));
    CHECK(net_input_ring_fill_bytes() == 0);             /* not published yet */
    CHECK(net_ring[NET_RING_BYTES - 1] == in[15] && net_ring[0] == in[16]);
    CHECK(!net_input_stage_pcm(0, NULL, 4));
    CHECK(!net_input_stage_pcm(NET_RING_BYTES + 1, in, 1));
    CHECK(!net_input_stage_pcm(NET_RING_BYTES - 8, in, 16));   /* off+len beyond free */
    CHECK(!net_input_stage_pcm(10, in, 0xFFFFFFFFu));          /* overflow-safe */
    CHECK(net_input_stage_pcm(NET_RING_BYTES, in, 0));
    net_input_commit_pcm(64);
    CHECK(net_input_ring_fill_bytes() == 64 && stream_active);
    net_input_commit_pcm(0);
    CHECK(net_input_ring_fill_bytes() == 64);
    net_input_commit_pcm(NET_RING_BYTES);                /* clamped */
    CHECK(net_input_ring_fill_bytes() == NET_RING_BYTES && ring_overflows == 1);
}

static void test_format(void) {
    reset_all();
    net_input_start();
    net_input_set_format(44100, 2, 16);
    CHECK(fmt_rate == 44100 && fmt_bytes_per_frame == 4);
    net_input_set_format(96000, 2, 16);  CHECK(fmt_rate == 44100);
    net_input_set_format(48000, 1, 16);  CHECK(fmt_rate == 44100);
    net_input_set_format(48000, 2, 32);  CHECK(fmt_rate == 44100);
    net_input_set_format(48000, 2, 24);
    CHECK(fmt_rate == 48000 && fmt_bits == 24 && fmt_bytes_per_frame == 6);
    uint8_t d[8] = {0};
    net_input_push_pcm(d, 8);
    net_input_set_format(48000, 2, 24);                  /* unchanged: no flush */
    CHECK(net_input_ring_fill_bytes() == 8);
    net_input_set_format(48000, 2, 16);                  /* changed: flush */
    CHECK(net_input_ring_fill_bytes() == 0);
    /* deferred rate change only when NET owns the pipeline */
    audio_state.freq = 48000;
    net_input_set_format(44100, 2, 16);
    CHECK(!rate_change_pending);
    active_input_source = INPUT_SOURCE_NET;
    net_input_set_format(48000, 2, 16);
    CHECK(!rate_change_pending);                         /* same as audio_state.freq */
    net_input_set_format(44100, 2, 16);
    CHECK(rate_change_pending && pending_rate == 44100);
}

static void put16(uint32_t pos, int16_t l, int16_t r) {
    net_ring[pos & NET_RING_MASK] = (uint8_t)l; net_ring[(pos + 1) & NET_RING_MASK] = (uint8_t)((uint16_t)l >> 8);
    net_ring[(pos + 2) & NET_RING_MASK] = (uint8_t)r; net_ring[(pos + 3) & NET_RING_MASK] = (uint8_t)((uint16_t)r >> 8);
}
static void put24(uint32_t pos, int32_t l, int32_t r) {
    for (int i = 0; i < 3; i++) {
        net_ring[(pos + i) & NET_RING_MASK] = (uint8_t)(l >> (8 * i));
        net_ring[(pos + 3 + i) & NET_RING_MASK] = (uint8_t)(r >> (8 * i));
    }
}

static void test_convert(void) {
    make_ready(48000, 16);
    stream_active = true;
    ring_wr = ring_rd = NET_RING_BYTES - 8;              /* block straddles the wrap */
    global_preamp_linear[0] = 0.5f; global_preamp_linear[1] = 2.0f;
    uint32_t frames = 60;
    for (uint32_t i = 0; i < frames; i++)
        put16(ring_wr + i * 4, (int16_t)(i * 100 - 3000), (int16_t)(-32768 + i));
    net_input_commit_pcm(frames * 4);
    CHECK(net_input_ring_fill_bytes() == frames * 4);
    uint32_t got = net_input_poll();
    CHECK(got == 60 && n_blocks == 1 && block_frames[0] == 60);
    int ok = 1;
    for (uint32_t i = 0; i < frames; i++) {
        float el = (float)(int16_t)(i * 100 - 3000) / 32768.0f * 0.5f;
        float er = (float)(int16_t)(-32768 + i) / 32768.0f * 2.0f;
        if (fabsf(buf_l[i] - el) > 1e-6f || fabsf(buf_r[i] - er) > 1e-6f) ok = 0;
    }
    CHECK(ok);
    CHECK(net_input_ring_fill_bytes() == 0);

    /* 24-bit signed, sign extension at both extremes */
    make_ready(44100, 24);
    stream_active = true;
    global_preamp_linear[0] = global_preamp_linear[1] = 1.0f;
    ring_wr = ring_rd = NET_RING_BYTES - 9;
    for (uint32_t i = 0; i < 50; i++) put24(ring_wr + i * 6, i == 0 ? -8388608 : (i == 1 ? 8388607 : -1), 1000);
    net_input_commit_pcm(50 * 6);
    n_blocks = 0;
    CHECK(net_input_poll() == 50);
    CHECK(buf_l[0] == -1.0f);
    CHECK(fabsf(buf_l[1] - 8388607.0f / 8388608.0f) < 1e-9f);
    CHECK(fabsf(buf_l[2] - (-1.0f / 8388608.0f)) < 1e-12f);
    CHECK(fabsf(buf_r[0] - 1000.0f / 8388608.0f) < 1e-12f);
    CHECK(net_input_ring_fill_bytes() == 0);
}

static void test_poll_pacing(void) {
    make_ready(48000, 16);
    stream_active = true;
    uint8_t *z = calloc(1, 4 * 1000);
    /* below min block: nothing, not an underrun-count (silent wait) */
    net_input_push_pcm(z, 4 * 47);
    CHECK(net_input_poll() == 0 && n_blocks == 0);
    /* 500 frames -> two 192-frame blocks per poll max */
    net_input_push_pcm(z, 4 * (500 - 47));
    CHECK(net_input_poll() == 384 && n_blocks == 2);
    CHECK(block_frames[0] == 192 && block_frames[1] == 192);
    /* 116 left: single block, then below min -> counted underrun on next pass inside same poll */
    n_blocks = 0;
    CHECK(net_input_poll() == 116 && n_blocks == 1);
    CHECK(ring_underruns == 1);
    /* consumer pool >= 50% full: no work */
    net_input_push_pcm(z, 4 * 100);
    consumer_fill = SPDIF_CONSUMER_BUFFER_COUNT / 2;
    n_blocks = 0;
    CHECK(net_input_poll() == 0 && n_blocks == 0);
    consumer_fill = 0;
    /* held: not ready, no consume */
    net_input_set_hold(true);
    CHECK(!net_input_is_ready() && net_input_poll() == 0 && n_blocks == 0);
    net_input_set_hold(false);
    CHECK(net_input_is_ready());
    /* idle detection: drain, then silence for > 400 ms */
    CHECK(net_input_poll() == 100);
    fake_time_us = last_push_us + 399000;
    CHECK(net_input_poll() == 0 && stream_active);
    fake_time_us = last_push_us + 401000;
    CHECK(net_input_poll() == 0 && !stream_active && !net_input_is_ready());
    free(z);
    /* not running */
    net_input_stop();
    CHECK(net_input_poll() == 0);
}

static void test_volume(void) {
    reset_all();
    CHECK(fabsf(net_pct_to_db(100)) < 1e-6f);
    CHECK(net_pct_to_db(0) == -(float)CENTER_VOLUME_INDEX);
    CHECK(net_pct_to_db(200) == 0.0f);                    /* clamped */
    CHECK(fabsf(net_pct_to_db(50) - 30.0f * log10f(0.5f)) < 1e-4f);
    CHECK(net_pct_to_db(1) >= -(float)CENTER_VOLUME_INDEX);
    for (int p = 0; p <= 100; p++) {                      /* set/get round trip */
        net_volume_set_percent((uint8_t)p);
        CHECK(net_volume_get_percent() == p);
    }
    net_volume_set_percent(250);
    CHECK(net_volume_get_percent() == 100);
    /* external change: derived from dB, monotonic, within +-1 of inverse curve */
    last_set_pct = 0xFF;
    int prev = -1, mono = 1, close = 1;
    for (int p = 2; p <= 100; p++) {
        audio_state.volume = (int16_t)lrintf(net_pct_to_db((uint8_t)p) * 256.0f);
        int g = net_volume_get_percent();
        if (g < prev) mono = 0;
        if (abs(g - p) > 1) close = 0;
        prev = g;
    }
    CHECK(mono && close);
    audio_state.volume = 256; CHECK(net_volume_get_percent() == 100);
    audio_state.volume = (int16_t)(-CENTER_VOLUME_INDEX * 256); CHECK(net_volume_get_percent() == 0);
    net_mute_set(true);  CHECK(user_mute && net_mute_get() && notify_val == 1);
    CHECK(notify_off == offsetof(WireBulkParams, user_volume.user_mute));
    net_mute_set(false); CHECK(!net_mute_get() && notify_val == 0);
}

static void test_config(void) {
    reset_all();
    net_wifi_set_credentials("a-very-long-ssid-that-exceeds-thirty-two-chars", "x");
    CHECK(strlen(net_wifi_get_ssid()) == 32);
    char longpw[100]; memset(longpw, 'p', 99); longpw[99] = 0;
    net_wifi_set_credentials("s", longpw);
    CHECK(strlen(net_wifi_get_pass()) == 63 && strcmp(net_wifi_get_ssid(), "s") == 0);
    net_wifi_set_credentials(NULL, NULL);
    CHECK(net_wifi_get_ssid()[0] == 0 && net_wifi_get_pass()[0] == 0);
    CHECK(!rejoin_requested);                             /* not running */
    net_sendspin_set_server("music.local", 0);
    CHECK(strcmp(net_sendspin_get_host(), "music.local") == 0 && net_sendspin_get_port() == 8927);
    CHECK(strcmp(net_sendspin_host(), "music.local") == 0 && net_sendspin_port() == 8927);
    net_sendspin_set_server("h", 1234); CHECK(net_sendspin_get_port() == 1234);
    memset(longpw, 'h', 99);
    net_sendspin_set_server(longpw, 1); CHECK(strlen(net_sendspin_get_host()) == 63);
    net_sendspin_set_server(NULL, 1);   CHECK(net_sendspin_get_host()[0] == 0);
    CHECK(strcmp(net_sendspin_get_path(), "/sendspin") == 0);
    CHECK(net_slimproto_get_port() == 3483 && net_slimproto_port() == 3483);
    net_slimproto_set_port(9000); CHECK(net_slimproto_get_port() == 9000);
    net_slimproto_set_port(0);    CHECK(net_slimproto_get_port() == 3483);
    CHECK(net_proto_get() == NET_PROTO_SENDSPIN);
    net_proto_set(NET_PROTO_SLIMPROTO); CHECK(net_proto_get() == NET_PROTO_SLIMPROTO);
    net_proto_set(2);   CHECK(net_proto_get() == NET_PROTO_SLIMPROTO);   /* ignored */
    net_proto_set(255); CHECK(net_proto_get() == NET_PROTO_SLIMPROTO);
    net_proto_set(NET_PROTO_SENDSPIN); CHECK(net_proto_get() == NET_PROTO_SENDSPIN);
}

static void test_wifi_sm(void) {
    /* no credentials */
    reset_all();
    net_input_start();
    CHECK(net_input_is_running() && wifi_state == NET_WIFI_NO_CREDENTIALS && n_connect == 0);
    net_input_start(); CHECK(n_init == 1);                /* idempotent */
    net_input_stop();
    CHECK(!net_input_is_running() && wifi_state == NET_WIFI_OFF && n_deinit == 1 && servo_resets == 1);
    net_input_stop(); CHECK(n_deinit == 1);

    /* open vs WPA2 */
    reset_all();
    net_wifi_set_credentials("open", "");
    net_input_start();
    CHECK(n_connect == 1 && last_auth == CYW43_AUTH_OPEN && last_pw_null && wifi_state == NET_WIFI_CONNECTING);
    net_input_stop();
    reset_all();
    net_wifi_set_credentials("sec", "pw");
    net_input_start();
    CHECK(last_auth == CYW43_AUTH_WPA2_MIXED_PSK && !last_pw_null && strcmp(last_ssid, "sec") == 0);

    /* link up -> connected -> sendspin client started once */
    fake_link = CYW43_LINK_JOIN;
    net_input_poll(); CHECK(wifi_state == NET_WIFI_CONNECTING && n_ss_start == 0);
    fake_link = CYW43_LINK_UP;
    net_input_poll(); net_input_poll();
    CHECK(wifi_state == NET_WIFI_CONNECTED && n_ss_start == 1 && n_ss_poll >= 1 && n_sl_poll == 0);
    NetStatusPacket st;
    net_input_get_status(&st);
    CHECK(st.wifi_state == NET_WIFI_CONNECTED && st.sendspin_state == 7 && st.protocol == 0);
    CHECK(st.ring_size_bytes == NET_RING_BYTES && st.rssi_dbm == -127 && st.active == 0 && st.ready == 0);

    /* protocol switch while up: old stopped, new started */
    net_proto_set(NET_PROTO_SLIMPROTO);
    net_input_poll();                  /* stops old and (same pass) starts new */
    CHECK(n_ss_stop == 1 && n_sl_start == 1);
    net_input_poll();
    CHECK(n_sl_start == 1 && n_sl_poll >= 1);
    net_input_get_status(&st);
    CHECK(st.sendspin_state == 9 && st.protocol == 1);

    /* credentials change -> client stopped, leave + rejoin */
    int c = n_connect, l = n_leave;
    net_wifi_set_credentials("other", "pw2");
    net_input_poll();
    CHECK(n_sl_stop == 1 && n_leave == l + 1 && n_connect == c + 1 && strcmp(last_ssid, "other") == 0);

    /* link failure: client torn down, FAILED, retry only after 5 s */
    fake_link = CYW43_LINK_UP; net_input_poll(); net_input_poll();
    CHECK(wifi_state == NET_WIFI_CONNECTED && n_sl_start == 2);
    fake_link = CYW43_LINK_BADAUTH;
    net_input_poll();
    CHECK(wifi_state == NET_WIFI_FAILED && n_sl_stop == 2);
    c = n_connect;
    fake_time_us += 4000000; net_input_poll(); CHECK(n_connect == c);
    fake_time_us += 1500000; net_input_poll(); CHECK(n_connect == c + 1 && wifi_state == NET_WIFI_CONNECTING);

    /* link drops to re-associating: client stopped, state CONNECTING */
    fake_link = CYW43_LINK_UP; net_input_poll(); net_input_poll();
    CHECK(wifi_state == NET_WIFI_CONNECTED);
    fake_link = CYW43_LINK_DOWN; net_input_poll();
    CHECK(wifi_state == NET_WIFI_CONNECTING);
    net_input_stop();

    /* radio init failure retried after NET_RETRY_MS */
    reset_all();
    fake_init_rc = -1;
    net_input_start();
    CHECK(wifi_state == NET_WIFI_INIT_FAILED && n_init == 1);
    net_input_poll(); CHECK(n_init == 1);
    fake_time_us += 5100000;
    fake_init_rc = 0; net_input_poll();
    CHECK(n_init == 2 && cyw43_up);
    net_input_stop();
}

int main(void) {
    test_ring(); test_stage_commit(); test_format(); test_convert();
    test_poll_pacing(); test_volume(); test_config(); test_wifi_sm();
    printf("%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
