#ifndef MODBUS_MASTER_H
#define MODBUS_MASTER_H
#include "esp_err.h"
#include "cJSON.h"

/*
 * Modbus -> H2 test modes. Select one before building:
 *   1 = read all configured endpoints first, then send changed endpoint
 *       frames consecutively over NUOS Link.
 *   2 = read one endpoint completely, send its changed attributes, then
 *       continue to the next endpoint.
 */
#define MODBUS_SEND_MODE_ALL_ENDPOINTS_BATCH  1
#define MODBUS_SEND_MODE_PER_ENDPOINT        2
#ifndef MODBUS_DATA_SEND_MODE
#define MODBUS_DATA_SEND_MODE MODBUS_SEND_MODE_ALL_ENDPOINTS_BATCH
#endif

#if (MODBUS_DATA_SEND_MODE != MODBUS_SEND_MODE_ALL_ENDPOINTS_BATCH) && \
    (MODBUS_DATA_SEND_MODE != MODBUS_SEND_MODE_PER_ENDPOINT)
#error "Invalid MODBUS_DATA_SEND_MODE"
#endif
esp_err_t modbus_master_init(void);
esp_err_t modbus_apply_serial_config(void);
esp_err_t modbus_write_attribute(uint8_t endpoint, uint8_t attribute_id, const char *value);
esp_err_t modbus_status_to_json(cJSON *array);
#endif
