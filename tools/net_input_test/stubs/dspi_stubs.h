/* Single fake of every Pico SDK / lwIP / DSPi header that net_input.c pulls in. */
#ifndef DSPI_STUBS_H
#define DSPI_STUBS_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define __dmb() __sync_synchronize()
typedef int64_t absolute_time_t;
extern uint64_t fake_time_us;
static inline uint64_t time_us_64(void) { return fake_time_us; }
static inline absolute_time_t get_absolute_time(void) { return (absolute_time_t)fake_time_us; }
static inline absolute_time_t make_timeout_time_ms(uint32_t ms) { return (absolute_time_t)fake_time_us + (int64_t)ms * 1000; }
static inline int64_t absolute_time_diff_us(absolute_time_t a, absolute_time_t b) { return b - a; }

/* cyw43 */
#define CYW43_ITF_STA 0
#define CYW43_AUTH_OPEN 0u
#define CYW43_AUTH_WPA2_MIXED_PSK 0x00400006u
#define CYW43_NO_POWERSAVE_MODE 0
#define CYW43_LINK_DOWN 0
#define CYW43_LINK_JOIN 1
#define CYW43_LINK_NOIP 2
#define CYW43_LINK_UP 3
#define CYW43_LINK_FAIL (-1)
#define CYW43_LINK_NONET (-2)
#define CYW43_LINK_BADAUTH (-3)
typedef struct { uint32_t addr; } ip4_addr_t;
struct netif { uint8_t hwaddr[6]; ip4_addr_t ip_addr; };
typedef struct { struct netif netif[1]; } cyw43_t;
extern cyw43_t cyw43_state;
int cyw43_arch_init(void);
void cyw43_arch_deinit(void);
void cyw43_arch_enable_sta_mode(void);
void cyw43_arch_poll(void);
int cyw43_arch_wifi_connect_async(const char *ssid, const char *pw, uint32_t auth);
int cyw43_wifi_leave(cyw43_t *s, int itf);
int cyw43_tcpip_link_status(cyw43_t *s, int itf);
int cyw43_wifi_get_rssi(cyw43_t *s, int32_t *rssi);
int cyw43_wifi_pm(cyw43_t *s, uint32_t pm);
static inline uint32_t cyw43_pm_value(int a, int b, int c, int d, int e) { return 0; }
static inline void netif_set_hostname(struct netif *n, const char *h) { (void)n; (void)h; }
static inline const ip4_addr_t *netif_ip4_addr(const struct netif *n) { return &n->ip_addr; }
static inline const char *ip4addr_ntoa(const ip4_addr_t *a) { (void)a; return "ip"; }

/* DSPi */
#define CENTER_VOLUME_INDEX 91
#define SPDIF_CONSUMER_BUFFER_COUNT 8
#define INPUT_SOURCE_NET 3
extern int active_input_source;
typedef struct { int16_t volume; uint32_t freq; } AudioState;
extern AudioState audio_state;
extern bool user_mute;
extern uint32_t pending_rate;
extern volatile bool rate_change_pending;
extern float global_preamp_linear[2];
extern float buf_l[512], buf_r[512];
typedef struct { struct { uint8_t user_mute; } user_volume; } WireBulkParams;
void update_user_volume(float db);
void notify_param_write(uint32_t off, uint32_t len, const void *data);
void *i2s_input_ring_storage(uint32_t *bytes);
void input_servo_reset(void);
int get_slot_consumer_fill(int slot);
void process_input_block(uint32_t frames);
void sendspin_client_start(void); void sendspin_client_stop(void); void sendspin_client_poll(void);
uint8_t sendspin_client_state(void);
void slimproto_client_start(void); void slimproto_client_stop(void); void slimproto_client_poll(void);
uint8_t slimproto_client_state(void);
#endif
