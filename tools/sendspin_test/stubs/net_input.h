/* Host-test stand-in for firmware/DSPi/net_input.h (API as specified). */
#ifndef STUB_NET_INPUT_H
#define STUB_NET_INPUT_H
#include <stdint.h>
#include <stdbool.h>
uint32_t net_input_ring_capacity(void);
uint32_t net_input_ring_free_bytes(void);
bool net_input_stage_pcm(uint32_t off, const uint8_t *pcm, uint32_t len);
void net_input_commit_pcm(uint32_t bytes);
void net_input_set_format(uint32_t rate_hz, uint8_t channels, uint8_t bits);
void net_input_flush(void);
uint32_t net_input_ring_fill_bytes(void);
uint8_t net_volume_get_percent(void);
void net_volume_set_percent(uint8_t);
bool net_mute_get(void);
void net_mute_set(bool);
const char *net_sendspin_host(void);
uint16_t net_sendspin_port(void);
#endif
