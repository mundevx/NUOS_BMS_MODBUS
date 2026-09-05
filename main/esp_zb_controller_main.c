/*
 * NUOS BMS - 30 Endpoint / Modbus
 */

#define DECLARE_MAIN

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "ha/esp_zigbee_ha_standard.h"
#include "esp_zb_controller_main.h"
#include "app_zigbee_clusters.h"

static const char *TAG = "ZB_MAIN_FILE";

extern esp_err_t modbus_master_init(void);

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    esp_err_t ret =
        esp_zb_bdb_start_top_level_commissioning(mode_mask);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG,
                 "Commissioning start failed: 0x%x",
                 ret);
    }
}

void esp_zb_app_signal_handler(
    esp_zb_app_signal_t *signal_struct)
{
    if (signal_struct == NULL) {
        return;
    }

    uint32_t *p_sg_p =
        signal_struct->p_app_signal;

    esp_err_t err_status =
        signal_struct->esp_err_status;

    esp_zb_app_signal_type_t sig_type =
        *p_sg_p;

    switch (sig_type) {

    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:

        ESP_LOGI(TAG, "Initialize Zigbee stack");

        esp_zb_bdb_start_top_level_commissioning(
            ESP_ZB_BDB_MODE_INITIALIZATION);

        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:

        if (err_status == ESP_OK) {

            ESP_LOGI(
                TAG,
                "Zigbee started, factory_new=%s",
                esp_zb_bdb_is_factory_new() ? "YES" : "NO");

            /*
             * Modbus task waits for esp_zb_bdb_dev_joined()
             * and then waits another 2 seconds before the first FC03.
             */
            ESP_ERROR_CHECK(modbus_master_init());

            if (esp_zb_bdb_is_factory_new()) {

                esp_zb_scheduler_alarm(
                    (esp_zb_callback_t)
                        bdb_start_top_level_commissioning_cb,
                    ESP_ZB_BDB_MODE_NETWORK_STEERING,
                    5000);
            }

        } else {

            ESP_LOGW(TAG,
                     "Zigbee startup failed: %s",
                     esp_err_to_name(err_status));

            esp_zb_scheduler_alarm(
                (esp_zb_callback_t)
                    bdb_start_top_level_commissioning_cb,
                ESP_ZB_BDB_MODE_INITIALIZATION,
                1000);
        }

        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:

        if (err_status == ESP_OK) {

            ESP_LOGI(TAG,
                     "Network steering successful");

        } else {

            ESP_LOGW(TAG,
                     "Network steering failed: %s",
                     esp_err_to_name(err_status));

            esp_zb_scheduler_alarm(
                (esp_zb_callback_t)
                    bdb_start_top_level_commissioning_cb,
                ESP_ZB_BDB_MODE_NETWORK_STEERING,
                5000);
        }

        break;

    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:

        ESP_LOGI(TAG,
                 "Device announcement received");

        break;

    case ESP_ZB_ZDO_DEVICE_UNAVAILABLE:

        ESP_LOGW(TAG,
                 "Coordinator/parent unavailable");

        break;

    case ESP_ZB_ZDO_SIGNAL_LEAVE:

        ESP_LOGW(TAG,
                 "Device left Zigbee network");

        break;

    default:
        break;
    }
}

static void esp_zb_task(void *pvParameters)
{
    (void)pvParameters;

    esp_zb_cfg_t zb_nwk_cfg =
        ESP_ZB_ZR_CONFIG();

    esp_zb_init(&zb_nwk_cfg);

    esp_zb_ep_list_t *ep_list =
        nuos_init_zb_clusters();

    ESP_ERROR_CHECK(
        ep_list != NULL
            ? ESP_OK
            : ESP_ERR_NO_MEM);

    esp_zb_device_register(ep_list);

    esp_zb_set_primary_network_channel_set(
        ESP_ZB_PRIMARY_CHANNEL_MASK);

        
    // esp_zb_factory_reset();
    ESP_ERROR_CHECK(
        esp_zb_start(false));

    esp_zb_stack_main_loop();
}

void app_main(void)
{
    esp_zb_uart_config_t uart_config = {
        .port = UART_NUM_0,
        .rx_pin = 17,
        .tx_pin = 16,
        .uart_config ={
            .baud_rate = 460800,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .flags.allow_pd = 0,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 0,
            .stop_bits = UART_STOP_BITS_1,
            .source_clk = UART_SCLK_DEFAULT,
        }
    };
    esp_zb_platform_config_t config = {
        .radio_config = {
            .radio_mode = ZB_RADIO_MODE_UART_RCP,
            .radio_uart_config = uart_config,
        },
        .host_config = {
            .host_connection_mode =
                ZB_HOST_CONNECTION_MODE_NONE,
        },
    };

    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NOT_FOUND) {

        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());

    } else if (ret != ESP_OK) {

        ESP_ERROR_CHECK(ret);
    }

    ESP_ERROR_CHECK(
        esp_zb_platform_config(&config));

    if (!nuos_init_sequence()) {

        ESP_LOGE(TAG,
                 "Application initialization failed");
        return;
    }

    if (xTaskCreate(
            esp_zb_task,
            "Zigbee_main",
            16384,
            NULL,
            5,
            NULL) != pdPASS) {

        ESP_LOGE(TAG,
                 "Failed to create Zigbee task");
    }
}
