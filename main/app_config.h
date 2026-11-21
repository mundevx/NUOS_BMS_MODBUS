#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#define APP_MAX_ENDPOINTS 50
#define APP_MAX_ATTRIBUTES 64
/* This is the storage/wire ceiling, not a per-endpoint Zigbee allocation.
 * Each H2 endpoint allocates exactly its configured attribute count. */
#define APP_NAME_LEN 32
#define APP_TOPIC_LEN 96
#define APP_MQTT_FIELD_LEN 96

/* Endpoint number is also the Modbus slave ID. 0 is reserved for broadcast. */
typedef enum {
    DEVICE_LT_PANEL = 0,
    DEVICE_DG,
    DEVICE_UPS,
    DEVICE_LIFT,
    DEVICE_ESCALATOR,
    DEVICE_FIRE_HYDRANT,
    DEVICE_ANEMOMETER,
    DEVICE_AC,
    DEVICE_VENTILATION_FAN,
    DEVICE_CUSTOM,
    DEVICE_SLD,
    DEVICE_LIGHTING,
    DEVICE_COUNT
} device_type_t;

typedef enum {
    MB_FC01 = 1,
    MB_FC02 = 2,
    MB_FC03 = 3,
    MB_FC04 = 4,
    MB_FC05 = 5,
    MB_FC06 = 6,
    MB_FC15 = 15,
    MB_FC16 = 16
} modbus_function_t;

typedef enum {
    MB_TYPE_BOOL = 0,
    MB_TYPE_UINT16,
    MB_TYPE_INT16,
    MB_TYPE_UINT32,
    MB_TYPE_INT32,
    MB_TYPE_FLOAT32,
    MB_TYPE_UINT64,
    MB_TYPE_INT64,
    MB_TYPE_STRING,
    MB_TYPE_UINT8,
    MB_TYPE_INT8,
    MB_TYPE_UINT24,
    MB_TYPE_UINT48
} modbus_data_type_t;

typedef enum {
    MB_WORD_BIG_FIRST = 0,
    MB_WORD_LOW_FIRST = 1
} modbus_word_order_t;

typedef struct {
    bool enabled;
    uint8_t attribute_id;       /* Zigbee attribute index, consumed by H2 */
    uint8_t function;           /* 1,2,3,4,5,6,15,16 */
    uint16_t address;           /* zero-based Modbus PDU address */
    uint8_t reg_count;          /* 1..32; coils use bit count where appropriate */
    uint8_t data_type;
    uint8_t word_order;
    /* Packed attribute support: each component is one 16-bit Modbus register. */
    uint8_t packed_count;
    uint16_t address2;
    uint16_t address3;
    char name[APP_NAME_LEN];
} modbus_attribute_t;

typedef struct {
    bool enabled;
    uint8_t slave_id;
    uint8_t device_type;
    uint8_t attribute_count;
    uint16_t cluster_id;
    modbus_attribute_t *attributes;
} endpoint_config_t;

typedef struct {
    uint32_t baud_rate;
    uint8_t data_bits;
    uint8_t stop_bits;
    uint8_t parity;             /* 0 none, 1 odd, 2 even */
} serial_config_t;

typedef struct {
    char broker[APP_MQTT_FIELD_LEN];
    uint16_t port;
    char username[APP_MQTT_FIELD_LEN];
    char password[APP_MQTT_FIELD_LEN];
    char client_id[APP_MQTT_FIELD_LEN];
    char publish_topic[APP_TOPIC_LEN];
} mqtt_display_config_t;

esp_err_t app_config_init(void);
esp_err_t app_config_load_serial(serial_config_t *cfg);
esp_err_t app_config_save_serial(const serial_config_t *cfg);
esp_err_t app_config_load_endpoint(uint8_t endpoint, endpoint_config_t *cfg);
esp_err_t app_config_save_endpoint(uint8_t endpoint, const endpoint_config_t *cfg);
esp_err_t app_config_delete_endpoint(uint8_t endpoint);
esp_err_t app_config_load_endpoint_count(uint8_t *count);
esp_err_t app_config_save_endpoint_count(uint8_t count);
esp_err_t app_config_load_mqtt(mqtt_display_config_t *cfg);
esp_err_t app_config_save_mqtt(const mqtt_display_config_t *cfg);
esp_err_t app_config_reset_defaults(void);
esp_err_t app_config_calculate_zigbee_hash(uint32_t *hash_out, uint8_t *endpoint_count_out);

esp_err_t app_config_load_template(uint8_t device_type, endpoint_config_t *cfg);
esp_err_t app_config_save_template(uint8_t device_type, const endpoint_config_t *cfg);
esp_err_t app_config_seed_device_templates(void);
void app_config_free_endpoint(endpoint_config_t *cfg);

const char *device_type_name(uint8_t type);
const char *modbus_data_type_name(uint8_t type);

#endif
