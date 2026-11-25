#ifndef NUOS_UART_LINK_H
#define NUOS_UART_LINK_H
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "nuos_link_proto.h"
esp_err_t nuos_link_init(void);
esp_err_t nuos_link_send_generic(uint8_t endpoint,const uint8_t *payload,uint16_t len);
esp_err_t nuos_link_factory_reset_h2(void);
esp_err_t nuos_link_ping(void);
bool nuos_link_peer_joined(void);
uint32_t nuos_link_peer_silent_ms(void);
void nuos_link_log_stats(void);
/* Request immediate application of the latest saved Zigbee configuration. */
void nuos_link_request_zigbee_config_sync(void);
#endif
