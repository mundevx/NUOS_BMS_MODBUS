/*
 * NUOS BMS - Modbus master for 30 Zigbee endpoints
 *
 * One Modbus cycle:
 *   - Read holding registers 0..19 using FC03.
 *   - Store REG0 as uint32_t ATTR0; ATTR1..ATTR9 remain zero.
 *   - Update the 10 uint32 attributes of one Zigbee endpoint.
 *
 * Endpoint sequence:
 *   EP1  <- fresh Modbus read
 *   EP2  <- fresh Modbus read
 *   ...
 *   EP30 <- fresh Modbus read
 *   repeat
 *
 * First Modbus transaction starts 2 seconds after Zigbee is joined.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_zigbee_core.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "app_zigbee_clusters.h"

static const char *TAG = "MODBUS_MASTER";

#define RS485_UART              UART_NUM_1
#define RS485_TX_PIN            3
#define RS485_RX_PIN            2
#define RS485_DE_PIN            5
#define RS485_BAUDRATE          9600

#define SLAVE_ID                1
#define REG_START_ADDRESS       0

#define MODBUS_REG_COUNT        1
#define ZIGBEE_VALUE_COUNT      10

#define MODBUS_RESPONSE_SIZE    7

#define FIRST_ENDPOINT           NUOS_FIRST_ENDPOINT
#define TOTAL_ENDPOINTS          NUOS_TOTAL_ENDPOINTS

static uint16_t modbus_crc16(
    const uint8_t *buffer,
    uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t pos = 0; pos < length; pos++) {

        crc ^= buffer[pos];

        for (uint8_t i = 0; i < 8; i++) {

            if (crc & 0x0001U) {
                crc >>= 1;
                crc ^= 0xA001U;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

static esp_err_t rs485_init(void)
{
    gpio_config_t de_config = {
        .pin_bit_mask = (1ULL << RS485_DE_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_RETURN_ON_ERROR(
        gpio_config(&de_config),
        TAG,
        "DE GPIO config failed");

    gpio_set_level(RS485_DE_PIN, 0);

    uart_config_t uart_config = {
        .baud_rate = RS485_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    ESP_RETURN_ON_ERROR(
        uart_driver_install(
            RS485_UART,
            1024,
            1024,
            0,
            NULL,
            0),
        TAG,
        "UART driver install failed");

    ESP_RETURN_ON_ERROR(
        uart_param_config(
            RS485_UART,
            &uart_config),
        TAG,
        "UART param config failed");

    ESP_RETURN_ON_ERROR(
        uart_set_pin(
            RS485_UART,
            RS485_TX_PIN,
            RS485_RX_PIN,
            UART_PIN_NO_CHANGE,
            UART_PIN_NO_CHANGE),
        TAG,
        "UART pin config failed");

    ESP_LOGI(TAG,
             "RS485 UART%d TX=%d RX=%d DE=%d baud=%d",
             RS485_UART,
             RS485_TX_PIN,
             RS485_RX_PIN,
             RS485_DE_PIN,
             RS485_BAUDRATE);

    return ESP_OK;
}

/*
 * Read 1 Modbus holding register (REG0) starting at register 0.
 *
 * FC03 request:
 *   slave, function, start_hi, start_lo, qty_hi, qty_lo, crc_lo, crc_hi
 *
 * Response:
 *   slave, function, byte_count,
 *   2 data bytes,
 *   crc_lo, crc_hi
 */
static esp_err_t modbus_read_reg0(uint32_t values[ZIGBEE_VALUE_COUNT])
{
    if (values == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Only REG0 is used for now. ATTR1..ATTR9 remain zero. */
    for (uint8_t i = 0; i < ZIGBEE_VALUE_COUNT; i++) {
        values[i] = 0;
    }

    /*
     * FC03:
     *   Slave       = 1
     *   Start Reg   = 0
     *   Quantity    = 1
     */
    uint8_t request[8] = {
        SLAVE_ID,
        0x03,
        0x00,
        0x00,
        0x00,
        0x01,
        0x00,
        0x00
    };

    uint16_t request_crc = modbus_crc16(request, 6);
    request[6] = (uint8_t)(request_crc & 0xFF);
    request[7] = (uint8_t)(request_crc >> 8);

    uart_flush_input(RS485_UART);

    gpio_set_level(RS485_DE_PIN, 1);
    esp_rom_delay_us(100);

    int written = uart_write_bytes(
        RS485_UART,
        request,
        sizeof(request));

    if (written != (int)sizeof(request)) {
        gpio_set_level(RS485_DE_PIN, 0);
        return ESP_FAIL;
    }

    esp_err_t err = uart_wait_tx_done(
        RS485_UART,
        pdMS_TO_TICKS(1000));

    gpio_set_level(RS485_DE_PIN, 0);

    if (err != ESP_OK) {
        return err;
    }

    /*
     * Normal response for quantity=1:
     *   01 03 02 DATA_H DATA_L CRC_L CRC_H
     *
     * Exception response:
     *   01 83 ERROR CRC_L CRC_H
     */
    uint8_t response[MODBUS_RESPONSE_SIZE] = {0};

    int length = uart_read_bytes(
        RS485_UART,
        response,
        sizeof(response),
        pdMS_TO_TICKS(1000));

    if (length == 5 &&
        response[0] == SLAVE_ID &&
        response[1] == 0x83) {

        ESP_LOGW(TAG,
                 "Modbus exception response: code=0x%02X",
                 response[2]);

        return ESP_FAIL;
    }

    if (length != MODBUS_RESPONSE_SIZE) {
        ESP_LOGW(TAG,
                 "Modbus response length=%d expected=%d",
                 length,
                 MODBUS_RESPONSE_SIZE);

        return ESP_ERR_TIMEOUT;
    }

    if (response[0] != SLAVE_ID) {
        ESP_LOGW(TAG,
                 "Wrong slave ID: 0x%02X",
                 response[0]);
        return ESP_FAIL;
    }

    if (response[1] & 0x80U) {
        ESP_LOGW(TAG,
                 "Modbus exception function=0x%02X code=0x%02X",
                 response[1],
                 response[2]);
        return ESP_FAIL;
    }

    if (response[1] != 0x03) {
        ESP_LOGW(TAG,
                 "Wrong Modbus function: 0x%02X",
                 response[1]);
        return ESP_FAIL;
    }

    if (response[2] != 0x02) {
        ESP_LOGW(TAG,
                 "Unexpected byte count=%u expected=2",
                 response[2]);
        return ESP_FAIL;
    }

    uint16_t received_crc =
        (uint16_t)response[5] |
        ((uint16_t)response[6] << 8);

    uint16_t calculated_crc =
        modbus_crc16(response, 5);

    if (received_crc != calculated_crc) {
        ESP_LOGW(TAG,
                 "CRC error rx=0x%04X calc=0x%04X",
                 received_crc,
                 calculated_crc);
        return ESP_FAIL;
    }

    /*
     * REG0 is 16-bit.
     * Store it as a uint32_t in ATTR0.
     *
     * Example:
     *   REG0 = 0x1234
     *   ATTR0 = 0x00001234
     */
    uint16_t reg0 =
        ((uint16_t)response[3] << 8) |
        (uint16_t)response[4];

    values[0] = (uint32_t)reg0;

    ESP_LOGI(TAG,
             "REG0 = %u (0x%04X) -> ATTR0 UINT32 = 0x%08lX",
             reg0,
             reg0,
             (unsigned long)values[0]);

    return ESP_OK;
}

static void print_values(
    uint8_t endpoint,
    const uint32_t values[ZIGBEE_VALUE_COUNT])
{
    ESP_LOGI(TAG,
             "EP%d Modbus data: "
             "[%lu, %lu, %lu, %lu, %lu, %lu, %lu, %lu, %lu, %lu]",
             endpoint,
             (unsigned long)values[0],
             (unsigned long)values[1],
             (unsigned long)values[2],
             (unsigned long)values[3],
             (unsigned long)values[4],
             (unsigned long)values[5],
             (unsigned long)values[6],
             (unsigned long)values[7],
             (unsigned long)values[8],
             (unsigned long)values[9]);
}

static void modbus_task(void *pvParameters)
{
    (void)pvParameters;

    bool joined_message_printed = false;

    /*
     * Do not begin at EP1 until Zigbee is joined.
     */
    while (!esp_zb_bdb_dev_joined()) {

        if (!joined_message_printed) {
            ESP_LOGI(TAG,
                     "Waiting for Zigbee commissioning...");
            joined_message_printed = true;
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    /*
     * Required initial 2-second delay after Zigbee join.
     */
    ESP_LOGI(TAG,
             "Zigbee joined. Waiting 2 seconds before first Modbus read...");

    vTaskDelay(pdMS_TO_TICKS(2000));

    joined_message_printed = false;

    uint32_t values[ZIGBEE_VALUE_COUNT];
    static int count = 0;
    static bool flag = true;
    while (1) {

        /*
         * If the device leaves the Zigbee network, stop Modbus polling.
         */
        if (!esp_zb_bdb_dev_joined()) {

            ESP_LOGW(TAG,
                     "Zigbee not joined. Pausing Modbus polling.");

            while (!esp_zb_bdb_dev_joined()) {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }

            ESP_LOGI(TAG,
                     "Zigbee joined again. Waiting 2 seconds...");

            vTaskDelay(pdMS_TO_TICKS(2000));
            flag = true;
            continue;
        }else{
            if(flag){

                vTaskDelay(pdMS_TO_TICKS(10));
                
                if(count++ >3000){
                    flag = false;
                    ESP_LOGI(TAG,
                        "Zigbee joined again. Waiting 30 seconds...");
                    count = 0;

                }else continue;
            }
        }

        /*
         * One fresh Modbus read PER endpoint.
         *
         * EP1 gets one fresh read,
         * EP2 gets another fresh read,
         * ...
         * EP30 gets another fresh read.
         */
        for (uint8_t endpoint = FIRST_ENDPOINT;
             endpoint < FIRST_ENDPOINT + TOTAL_ENDPOINTS;
             endpoint++) {

            esp_err_t err =
                modbus_read_reg0(values);

            if (err != ESP_OK) {

                ESP_LOGW(TAG,
                         "EP%d Modbus read failed: %s",
                         endpoint,
                         esp_err_to_name(err));

                /*
                 * Do not overwrite the endpoint when Modbus fails.
                 * Continue with the next endpoint.
                 */
                continue;
            }

            print_values(endpoint, values);

            /*
             * Update all 10 attributes and explicitly report all 10.
             */
            esp_err_t zb_ret =
                nuos_update_endpoint_from_modbus(
                    endpoint,
                    values);

            if (zb_ret != ESP_OK) {
                ESP_LOGW(TAG,
                         "EP%d Zigbee update/report failed: %s",
                         endpoint,
                         esp_err_to_name(zb_ret));
            } else {
                ESP_LOGI(TAG,
                         "EP%d: all 10 attributes sent",
                         endpoint);
            }

            /*
             * Yield to the RTOS/Zigbee stack between endpoints.
             */
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        /*
         * Immediately start the next EP1 -> EP30 sequence.
         * Every endpoint gets a fresh Modbus read.
         */
        ESP_LOGI(TAG,
                 "Completed EP1..EP30 Modbus/Zigbee update cycle");

        ESP_LOGI(TAG,
                 "Waiting 2000 ms before next Modbus cycle...");

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

esp_err_t modbus_master_init(void)
{
    ESP_RETURN_ON_ERROR(
        rs485_init(),
        TAG,
        "RS485 initialization failed");

    BaseType_t task_ret =
        xTaskCreate(
            modbus_task,
            "modbus_task",
            8192,
            NULL,
            10,
            NULL);

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG,
                 "Failed to create Modbus task");
        return ESP_FAIL;
    }

    return ESP_OK;
}
