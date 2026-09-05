#ifndef _APP_ZIGBEE_CLUSTER_H_
#define _APP_ZIGBEE_CLUSTER_H_

#include <stdint.h>
#include <stdbool.h>

#include "esp_zigbee_core.h"
#include "ha/esp_zigbee_ha_standard.h"

#define NUOS_FIRST_ENDPOINT      1
#define NUOS_TOTAL_ENDPOINTS     30
#define NUOS_CUSTOM_CLUSTER_ID   0xFCE0
#define NUOS_ATTR_COUNT          10
#define NUOS_FIRST_ATTR_ID       0x0000

esp_zb_ep_list_t *nuos_init_zb_clusters(void);

esp_err_t nuos_update_endpoint_from_modbus(
    uint8_t endpoint,
    const uint32_t values[NUOS_ATTR_COUNT]);

esp_err_t nuos_set_modbus_value(
    uint8_t endpoint,
    uint8_t attr_index,
    uint32_t value);

esp_err_t send_report(
    uint8_t endpoint,
    uint16_t cluster_id,
    uint16_t attr_id);

bool nuos_init_sequence(void);

#endif
