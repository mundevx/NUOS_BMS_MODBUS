/*
 * NUOS BMS - 30 Zigbee endpoints
 *
 * Each endpoint has:
 *   Custom Cluster : 0xFC00
 *   Attributes     : 0x0000 ... 0x0009
 *   Type           : UINT32
 *
 * Modbus:
 *   Start register : 0
 *   Quantity       : 1 holding register (FC03)
 *   REG0 is placed into ATTR0 as uint32_t.
 *   ATTR1..ATTR9 remain 0 for now.
 *
 * Zigbee reporting:
 *   - The device does NOT send unsolicited/manual reports.
 *   - Attribute values are updated locally only.
 *   - The Zigbee bridge/client must discover/read/configure the
 *     attributes during its interview.
 *   - Once reporting is configured by the bridge, the Zigbee stack
 *     can generate reports when an attribute changes.
 *
 * This prevents 30 x 10 attributes from flooding the bridge while
 * the device is still being interviewed.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_zigbee_core.h"
#include "esp_zigbee_type.h"
#include "ha/esp_zigbee_ha_standard.h"

#include "app_zigbee_clusters.h"

static const char *TAG = "ZB_CLUSTERS";

/*
 * IMPORTANT:
 * These must have static lifetime because the Zigbee attribute database
 * keeps pointers to the attribute values.
 */
static uint32_t g_zb_values[NUOS_TOTAL_ENDPOINTS][NUOS_ATTR_COUNT] = {0};

/* -------------------------------------------------------------------------- */
/* Reporting                                                                  */
/* -------------------------------------------------------------------------- */

/*
 * Do NOT manually send unsolicited reports.
 *
 * Reporting is controlled by the Zigbee bridge/client. The custom
 * attributes are created with ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, so the
 * bridge can configure reporting during interview.
 *
 * After the bridge successfully configures an attribute for reporting,
 * the Zigbee stack handles reporting when the attribute changes.
 */
esp_err_t send_report(uint8_t endpoint, uint16_t cluster_id, uint16_t attr_id)
{
    // (void)endpoint;
    // (void)cluster_id;
    // (void)attr_id;
 esp_zb_zcl_report_attr_cmd_t report_cmd_2 = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = 0x0000,
            .dst_endpoint = 1,
            .src_endpoint = endpoint,
        },

        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,

        .manuf_specific = 0,
        .manuf_code = 0,
        .dis_default_resp = 1,
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = cluster_id,
        .attributeID = attr_id,
    };

    return esp_zb_zcl_report_attr_cmd_req(&report_cmd_2);
    /*
     * Intentionally disabled.
     *
     * This function is retained for source compatibility with existing
     * code, but it must never generate an unsolicited report.
     */
   // return ESP_ERR_NOT_SUPPORTED;
}

/* -------------------------------------------------------------------------- */
/* Basic cluster                                                              */
/* -------------------------------------------------------------------------- */

static esp_zb_attribute_list_t *create_basic_cluster(void)
{
    esp_zb_basic_cluster_cfg_t cfg = {
        .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,
        .zcl_version  = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
    };

    esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create(&cfg);
    if (basic == NULL) {
        ESP_LOGE(TAG, "Basic cluster create failed");
        return NULL;
    }

    static char model_id[] = {
        16,
        'N','U','O','S','-','M','B','-','3','0','E','P','-','U','3','2'
    };

    static char manufacturer[] = {
        4, 'N','U','O','S'
    };

    esp_zb_basic_cluster_add_attr(
        basic,
        ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
        model_id);

    esp_zb_basic_cluster_add_attr(
        basic,
        ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
        manufacturer);

    return basic;
}

/* -------------------------------------------------------------------------- */
/* Custom cluster: 10 x uint32_t attributes                                  */
/* -------------------------------------------------------------------------- */

static esp_zb_attribute_list_t *create_custom_cluster(uint8_t endpoint)
{
    esp_zb_attribute_list_t *cluster =
        esp_zb_zcl_attr_list_create(NUOS_CUSTOM_CLUSTER_ID);

    if (cluster == NULL) {
        ESP_LOGE(TAG, "EP%d custom cluster create failed", endpoint);
        return NULL;
    }

    for (uint8_t i = 0; i < NUOS_ATTR_COUNT; i++) {

        uint16_t attr_id = NUOS_FIRST_ATTR_ID + i;

        esp_err_t ret =
            esp_zb_custom_cluster_add_custom_attr(
                cluster,
                attr_id,
                ESP_ZB_ZCL_ATTR_TYPE_U32,
                ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE |
                    ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
                &g_zb_values[endpoint - NUOS_FIRST_ENDPOINT][i]);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG,
                     "EP%d ATTR 0x%04X create failed: 0x%x",
                     endpoint,
                     attr_id,
                     ret);
            return NULL;
        }
    }

    return cluster;
}

/* -------------------------------------------------------------------------- */
/* Zigbee endpoint creation                                                   */
/* -------------------------------------------------------------------------- */

esp_zb_ep_list_t *nuos_init_zb_clusters(void)
{
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();

    if (ep_list == NULL) {
        ESP_LOGE(TAG, "Endpoint list create failed");
        return NULL;
    }

    for (uint8_t ep = NUOS_FIRST_ENDPOINT;
         ep < NUOS_FIRST_ENDPOINT + NUOS_TOTAL_ENDPOINTS;
         ep++) {

        esp_zb_cluster_list_t *cluster_list =
            esp_zb_zcl_cluster_list_create();

        if (cluster_list == NULL) {
            ESP_LOGE(TAG, "EP%d cluster list create failed", ep);
            return NULL;
        }

        /*
         * Put Basic only on EP1.
         */
        if (ep == NUOS_FIRST_ENDPOINT) {

            esp_zb_attribute_list_t *basic =
                create_basic_cluster();

            if (basic == NULL) {
                return NULL;
            }

            esp_zb_cluster_list_add_basic_cluster(
                cluster_list,
                basic,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        }

        /*
         * Identify cluster on every endpoint.
         */
        esp_zb_identify_cluster_cfg_t identify_cfg = {
            .identify_time =
                ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE
        };

        esp_zb_attribute_list_t *identify =
            esp_zb_identify_cluster_create(&identify_cfg);

        if (identify == NULL) {
            ESP_LOGE(TAG, "EP%d Identify cluster create failed", ep);
            return NULL;
        }

        esp_zb_cluster_list_add_identify_cluster(
            cluster_list,
            identify,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        /*
         * One custom cluster per endpoint.
         * Each has 10 x uint32 attributes:
         * 0x0000 ... 0x0009
         */
        esp_zb_attribute_list_t *custom =
            create_custom_cluster(ep);

        if (custom == NULL) {
            return NULL;
        }

        esp_zb_cluster_list_add_custom_cluster(
            cluster_list,
            custom,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_endpoint_config_t endpoint_config = {
            .endpoint = ep,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
            .app_device_version = 0,
        };

        esp_err_t ret = esp_zb_ep_list_add_ep(
            ep_list,
            cluster_list,
            endpoint_config);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "EP%d add failed: 0x%x", ep, ret);
            return NULL;
        }

        ESP_LOGI(TAG,
                 "EP%d -> Cluster 0x%04X -> 10 x UINT32",
                 ep,
                 NUOS_CUSTOM_CLUSTER_ID);
    }

    ESP_LOGI(TAG,
             "Created %d endpoints, %d UINT32 attributes/endpoint",
             NUOS_TOTAL_ENDPOINTS,
             NUOS_ATTR_COUNT);

    return ep_list;
}

/* -------------------------------------------------------------------------- */
/* Update one endpoint with 10 Modbus values                                  */
/* -------------------------------------------------------------------------- */

esp_err_t nuos_update_endpoint_from_modbus(
    uint8_t endpoint,
    const uint32_t values[NUOS_ATTR_COUNT])
{
    if (!esp_zb_bdb_dev_joined()) {
        return ESP_ERR_INVALID_STATE;
    }

    if (endpoint < NUOS_FIRST_ENDPOINT ||
        endpoint >= NUOS_FIRST_ENDPOINT + NUOS_TOTAL_ENDPOINTS ||
        values == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t index = endpoint - NUOS_FIRST_ENDPOINT;

    /*
     * Re-check Zigbee state immediately before touching the Zigbee
     * attribute database.
     */
    if (!esp_zb_bdb_dev_joined()) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_zb_lock_acquire(portMAX_DELAY);

    esp_err_t final_ret = ESP_OK;

    for (uint8_t i = 0; i < NUOS_ATTR_COUNT; i++) {

        /*
         * Update the persistent attribute storage.
         *
         * IMPORTANT:
         * Do not call esp_zb_zcl_report_attr_cmd_req() here.
         *
         * The bridge/client is responsible for configuring reporting
         * after it discovers/interviews the attribute.
         */
        g_zb_values[index][i] = values[i];

        esp_zb_zcl_status_t status =
            esp_zb_zcl_set_attribute_val(
                endpoint,
                NUOS_CUSTOM_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                NUOS_FIRST_ATTR_ID + i,
                &g_zb_values[index][i],
                false);
        printf("EP%d ATTR%u update: 0x%08lX\n", endpoint, i, (unsigned long)values[i]); 
        if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
            ESP_LOGE(TAG,
                     "EP%d ATTR%u update failed: 0x%x",
                     endpoint,
                     i,
                     status);
            final_ret = ESP_FAIL;
        }else{
            if(i==0) send_report(endpoint, NUOS_CUSTOM_CLUSTER_ID, NUOS_FIRST_ATTR_ID + i);
        }
    }

    esp_zb_lock_release();

    if (final_ret == ESP_OK) {
        ESP_LOGI(TAG,
                 "EP%d: 10 attributes updated locally; no unsolicited reports",
                 endpoint);
    }

    return final_ret;
}

/* -------------------------------------------------------------------------- */
/* Optional single attribute update                                           */
/* -------------------------------------------------------------------------- */

esp_err_t nuos_set_modbus_value(
    uint8_t endpoint,
    uint8_t attr_index,
    uint32_t value)
{
    if (attr_index >= NUOS_ATTR_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t values[NUOS_ATTR_COUNT] = {0};

    if (endpoint < NUOS_FIRST_ENDPOINT ||
        endpoint >= NUOS_FIRST_ENDPOINT + NUOS_TOTAL_ENDPOINTS) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t index = endpoint - NUOS_FIRST_ENDPOINT;

    memcpy(values, g_zb_values[index], sizeof(values));
    values[attr_index] = value;

    return nuos_update_endpoint_from_modbus(endpoint, values);
}

bool nuos_init_sequence(void)
{
    ESP_LOGI(TAG,
             "30-endpoint Zigbee initialized: Cluster=0x%04X, attrs=10 x UINT32",
             NUOS_CUSTOM_CLUSTER_ID);

    return true;
}
