#include "nuos_uart_link.h"
#include "app_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "NUOS_LINK";
#define UART_LINK UART_NUM_0
#define TX_PIN 16
#define RX_PIN 17
#define RX_BUF 4096
#define TX_BUF 4096
#define ACK_TIMEOUT 2500
#define CONFIG_SYNC_INITIAL_DELAY_MS 2500
#define CONFIG_SYNC_PERIOD_MS 2000
#define CONFIG_RETRY_GUARD_MS 8000

static SemaphoreHandle_t s_tx;
static SemaphoreHandle_t s_ack;
static SemaphoreHandle_t s_cfg_request;
static SemaphoreHandle_t s_ack_tx;
static volatile uint8_t s_ack_seq;
static volatile uint8_t s_ack_status;
static volatile bool s_joined;
static volatile bool s_h2_status_seen;
static volatile uint32_t s_h2_config_hash;
static volatile int64_t s_last = -1;
static uint8_t s_seq;
static nuos_link_parser_t s_parser;
static uint32_t s_tx_count, s_ack_ok, s_ack_to;
static int64_t s_last_config_send_us;
static bool s_config_sync_busy;

static uint32_t get_u32_le(const uint8_t *p)
{
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static void put_u32_le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static esp_err_t write_frame(uint8_t type, uint8_t ep, const uint8_t *p,
                             uint16_t len, uint8_t seq)
{
    uint8_t *frame = malloc(NUOS_LINK_MAX_FRAME);
    if (!frame) return ESP_ERR_NO_MEM;

    uint16_t n = nuos_link_build(frame, type, seq, ep, p, len);
    if (!n) {
        free(frame);
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_tx, pdMS_TO_TICKS(1000)) != pdTRUE) {
        free(frame);
        return ESP_ERR_TIMEOUT;
    }

    int w = uart_write_bytes(UART_LINK, frame, n);
    esp_err_t wait_ret = uart_wait_tx_done(UART_LINK, pdMS_TO_TICKS(1000));
    xSemaphoreGive(s_tx);
    free(frame);

    if (w != (int)n || wait_ret != ESP_OK) return ESP_FAIL;
    s_tx_count++;
    return ESP_OK;
}

static esp_err_t wait_ack(uint8_t seq)
{
    TickType_t end = xTaskGetTickCount() + pdMS_TO_TICKS(ACK_TIMEOUT);
    while (xTaskGetTickCount() < end) {
        TickType_t left = end - xTaskGetTickCount();
        if (xSemaphoreTake(s_ack, left) != pdTRUE) return ESP_ERR_TIMEOUT;
        if (s_ack_seq != seq) {
            ESP_LOGW(TAG,
                     "ACK received for different seq=%u while waiting seq=%u",
                     s_ack_seq, seq);
            continue;
        }
        ESP_LOGI(TAG, "ACK RX seq=%u status=0x%02X", s_ack_seq, s_ack_status);
        if (s_ack_status == NUOS_LINK_ST_OK) return ESP_OK;
        if (s_ack_status == NUOS_LINK_ST_NOT_JOINED) return ESP_ERR_INVALID_STATE;
        return ESP_FAIL;
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t send_ack_required_locked(uint8_t type, uint8_t ep,
                                          const uint8_t *payload, uint16_t len)
{
    for (int attempt = 0; attempt < 2; attempt++) {
        uint8_t seq = s_seq++;
        xSemaphoreTake(s_ack, 0);
        esp_err_t e = write_frame(type, ep, payload, len, seq);
        if (e != ESP_OK) return e;
        ESP_LOGI(TAG,
                 "ZB_CONFIG: waiting for ACK type=0x%02X ep=%u seq=%u attempt=%d timeout=%dms",
                 type, ep, seq, attempt + 1, ACK_TIMEOUT);
        e = wait_ack(seq);
        if (e == ESP_OK) {
            ESP_LOGI(TAG,
                     "ZB_CONFIG: ACK OK type=0x%02X ep=%u seq=%u attempt=%d",
                     type, ep, seq, attempt + 1);
            return ESP_OK;
        }
        if (e == ESP_ERR_INVALID_STATE) return e;
        ESP_LOGW(TAG,
                 "ZB_CONFIG: ACK failed type=0x%02X ep=%u attempt=%d result=%s",
                 type, ep, attempt + 1, esp_err_to_name(e));
        s_ack_to++;
    }
    return ESP_ERR_TIMEOUT;
}

static bool endpoint_has_enabled_attributes(const endpoint_config_t *cfg, uint16_t *count_out)
{
    uint16_t count = 0;
    if (cfg && cfg->enabled) {
        for (uint16_t i = 0; i < cfg->attribute_count; i++) if (cfg->attributes[i].enabled) count++;
    }
    if (count_out) *count_out = count;
    return count != 0 && cfg->cluster_id != 0;
}

/* ZB_CONFIG_EP payload:
 * [version][device_type][cluster_lo][cluster_hi][start_lo][start_hi][count_lo][count_hi]
 * followed by count x [attribute_id,data_type].
 * A configuration may therefore be sent in multiple chunks; H2 allocates the
 * endpoint exactly to the received configuration instead of using a fixed
 * attribute ceiling.
 */
static esp_err_t send_zigbee_configuration(uint32_t hash, uint8_t endpoint_count)
{
    if (s_config_sync_busy) {
        return ESP_ERR_INVALID_STATE;
    }

    s_config_sync_busy = true;
    esp_err_t e = ESP_OK;

    if (xSemaphoreTake(s_ack_tx, pdMS_TO_TICKS(2000)) != pdTRUE) {
        s_config_sync_busy = false;
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(TAG,
             "ZB_CONFIG: BEGIN hash=0x%08lX expected_endpoints=%u",
             (unsigned long)hash, endpoint_count);

    uint8_t p[6];
    p[0] = NUOS_ZB_CONFIG_VERSION;
    put_u32_le(&p[1], hash);
    p[5] = endpoint_count;

    e = send_ack_required_locked(NUOS_LINK_TYPE_ZB_CONFIG_BEGIN, 0, p, sizeof(p));
    if (e != ESP_OK) {
        goto done;
    }

    const uint16_t records_per_frame = (NUOS_LINK_MAX_PAYLOAD - 8u) / 2u;
    uint8_t sent_endpoints = 0;

    for (uint8_t ep = 1; ep <= APP_MAX_ENDPOINTS; ep++) {
        endpoint_config_t cfg = {0};
        cfg.slave_id = ep;

        e = app_config_load_endpoint(ep, &cfg);
        if (e != ESP_OK) {
            ESP_LOGE(TAG,
                     "ZB_CONFIG EP%u: failed to load endpoint config: %s",
                     ep, esp_err_to_name(e));
            app_config_free_endpoint(&cfg);
            goto done;
        }

        uint16_t enabled_count = 0;
        if (cfg.enabled && cfg.cluster_id != 0) {
            for (uint16_t i = 0; i < cfg.attribute_count; i++) {
                if (cfg.attributes[i].enabled) {
                    enabled_count++;
                }
            }
        }

        if (enabled_count == 0) {
            app_config_free_endpoint(&cfg);
            continue;
        }

        sent_endpoints++;

        ESP_LOGI(TAG,
                 "ZB_CONFIG EP%u: device_type=%u cluster=0x%04X attrs=%u",
                 ep, cfg.device_type, cfg.cluster_id, enabled_count);

        uint16_t sent = 0;
        while (sent < enabled_count) {
            uint16_t chunk = enabled_count - sent;
            if (chunk > records_per_frame) {
                chunk = records_per_frame;
            }

            uint8_t *ep_payload = malloc(8u + (size_t)chunk * 2u);
            if (!ep_payload) {
                app_config_free_endpoint(&cfg);
                e = ESP_ERR_NO_MEM;
                goto done;
            }

            size_t n = 0;
            ep_payload[n++] = NUOS_ZB_CONFIG_VERSION;
            ep_payload[n++] = cfg.device_type;
            ep_payload[n++] = (uint8_t)cfg.cluster_id;
            ep_payload[n++] = (uint8_t)(cfg.cluster_id >> 8);
            ep_payload[n++] = (uint8_t)sent;
            ep_payload[n++] = (uint8_t)(sent >> 8);
            ep_payload[n++] = (uint8_t)chunk;
            ep_payload[n++] = (uint8_t)(chunk >> 8);

            uint16_t skipped = 0;
            uint16_t added = 0;

            for (uint16_t i = 0;
                 i < cfg.attribute_count && added < chunk;
                 i++) {
                const modbus_attribute_t *a = &cfg.attributes[i];
                if (!a->enabled) {
                    continue;
                }

                if (skipped < sent) {
                    skipped++;
                    continue;
                }

                ep_payload[n++] = a->attribute_id;
                ep_payload[n++] = a->data_type;
                added++;
            }

            if (added != chunk) {
                ESP_LOGE(TAG,
                         "ZB_CONFIG EP%u: internal packing error requested=%u added=%u",
                         ep, chunk, added);
                free(ep_payload);
                app_config_free_endpoint(&cfg);
                e = ESP_ERR_INVALID_SIZE;
                goto done;
            }

            ESP_LOGI(TAG,
                     "ZB_CONFIG EP%u: sending chunk start=%u count=%u frame_len=%u",
                     ep, sent, chunk, (unsigned)n);

            e = send_ack_required_locked(NUOS_LINK_TYPE_ZB_CONFIG_EP,
                                          ep, ep_payload, (uint16_t)n);
            free(ep_payload);

            if (e != ESP_OK) {
                ESP_LOGW(TAG,
                         "ZB_CONFIG EP%u: chunk start=%u failed: %s",
                         ep, sent, esp_err_to_name(e));
                app_config_free_endpoint(&cfg);
                goto done;
            }

            sent = (uint16_t)(sent + chunk);
        }

        app_config_free_endpoint(&cfg);
    }

    if (sent_endpoints != endpoint_count) {
        ESP_LOGE(TAG,
                 "ZB_CONFIG: endpoint count mismatch expected=%u actually_sent=%u; COMMIT NOT SENT",
                 endpoint_count, sent_endpoints);
        e = ESP_ERR_INVALID_STATE;
        goto done;
    }

    ESP_LOGI(TAG,
             "ZB_CONFIG: all %u endpoints transmitted; sending COMMIT hash=0x%08lX",
             sent_endpoints, (unsigned long)hash);

    p[0] = NUOS_ZB_CONFIG_VERSION;
    put_u32_le(&p[1], hash);
    p[5] = endpoint_count;

    e = send_ack_required_locked(NUOS_LINK_TYPE_ZB_CONFIG_COMMIT,
                                 0, p, sizeof(p));

    if (e == ESP_OK) {
        s_last_config_send_us = esp_timer_get_time();
        ESP_LOGI(TAG,
                 "ZB_CONFIG: COMMIT ACKED by H2; waiting for status hash=0x%08lX",
                 (unsigned long)hash);
    }

done:
    xSemaphoreGive(s_ack_tx);
    s_config_sync_busy = false;

    if (e != ESP_OK) {
        ESP_LOGW(TAG,
                 "ZB_CONFIG: synchronization failed: %s (0x%x)",
                 esp_err_to_name(e), (unsigned)e);
    }

    return e;
}

static void handle(const nuos_link_frame_t *f)
{
    s_last = esp_timer_get_time();

    if (f->type == NUOS_LINK_TYPE_ACK || f->type == NUOS_LINK_TYPE_NAK) {
        s_ack_seq = f->seq;
        s_ack_status = f->len ? f->payload[0] : NUOS_LINK_ST_ZB_ERROR;
        xSemaphoreGive(s_ack);
    } else if (f->type == NUOS_LINK_TYPE_STATUS) {
        s_h2_status_seen = true;
        s_joined = f->len && (f->payload[0] & NUOS_LINK_FLAG_ZB_JOINED);
        if (f->len >= 5) {
            s_h2_config_hash = get_u32_le(&f->payload[1]);
        } else {
            s_h2_config_hash = 0;
        }
        ESP_LOGI(TAG, "H2 STATUS flags=0x%02X config_hash=0x%08lX",
                 f->len ? f->payload[0] : 0,
                 (unsigned long)s_h2_config_hash);
    }
}

static void rx_task(void *arg)
{
    (void)arg;
    uint8_t b[512];
    nuos_link_parser_init(&s_parser);

    for (;;) {
        int n = uart_read_bytes(UART_LINK, b, sizeof(b), pdMS_TO_TICKS(50));
        for (int i = 0; i < n; i++) {
            nuos_link_frame_t f;
            if (nuos_link_parser_push(&s_parser, b[i], &f)) handle(&f);
        }
    }
}

static void config_sync_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(CONFIG_SYNC_INITIAL_DELAY_MS));

    for (;;) {
        uint32_t local_hash = 0;
        uint8_t endpoint_count = 0;
        if (app_config_calculate_zigbee_hash(&local_hash, &endpoint_count) == ESP_OK) {
            int64_t now = esp_timer_get_time();
            bool guard_active = (s_last_config_send_us > 0) &&
                                ((now - s_last_config_send_us) <
                                 ((int64_t)CONFIG_RETRY_GUARD_MS * 1000LL));
            bool mismatch = !s_h2_status_seen || (s_h2_config_hash != local_hash);

            if (mismatch && !guard_active && !s_config_sync_busy) {
                (void)send_zigbee_configuration(local_hash, endpoint_count);
            }
        }

        if (xSemaphoreTake(s_cfg_request, pdMS_TO_TICKS(CONFIG_SYNC_PERIOD_MS)) == pdTRUE) {
            /* A web configuration change requested an immediate re-check. */
            continue;
        }
    }
}

esp_err_t nuos_link_init(void)
{
    s_tx = xSemaphoreCreateMutex();
    s_ack = xSemaphoreCreateBinary();
    s_cfg_request = xSemaphoreCreateBinary();
    s_ack_tx = xSemaphoreCreateMutex();
    if (!s_tx || !s_ack || !s_cfg_request || !s_ack_tx) return ESP_ERR_NO_MEM;

    uart_config_t c = {
        .baud_rate = NUOS_LINK_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    ESP_RETURN_ON_ERROR(uart_driver_install(UART_LINK, RX_BUF, TX_BUF, 0, NULL, 0), TAG, "link uart");
    ESP_RETURN_ON_ERROR(uart_param_config(UART_LINK, &c), TAG, "link params");
    ESP_RETURN_ON_ERROR(uart_set_pin(UART_LINK, TX_PIN, RX_PIN,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), TAG, "link pins");

    if (xTaskCreate(rx_task, "nuos_link_rx", 4096, NULL, 6, NULL) != pdPASS) return ESP_FAIL;
    if (xTaskCreate(config_sync_task, "nuos_zb_cfg", 8192, NULL, 5, NULL) != pdPASS) return ESP_FAIL;
    return ESP_OK;
}

void nuos_link_request_zigbee_config_sync(void)
{
    if (s_cfg_request) xSemaphoreGive(s_cfg_request);
}

esp_err_t nuos_link_send_generic(uint8_t endpoint, const uint8_t *payload, uint16_t len)
{
    if (endpoint == 0 || endpoint > NUOS_LINK_TOTAL_ENDPOINTS || !payload || len > NUOS_LINK_MAX_PAYLOAD) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_ack_tx, pdMS_TO_TICKS(2000)) != pdTRUE) return ESP_ERR_TIMEOUT;
    esp_err_t result = ESP_ERR_TIMEOUT;
    for (int attempt = 0; attempt < 2; attempt++) {
        uint8_t seq = s_seq++;
        xSemaphoreTake(s_ack, 0);
        ESP_LOGI(TAG,
                 "EP_DATA TX EP%u seq=%u payload_len=%u attempt=%d",
                 endpoint, seq, len, attempt + 1);
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, payload, len, ESP_LOG_DEBUG);

        esp_err_t e = write_frame(NUOS_LINK_TYPE_EP_DATA, endpoint, payload, len, seq);
        if (e != ESP_OK) {
            ESP_LOGW(TAG,
                     "EP_DATA TX WRITE FAILED EP%u seq=%u status=%s",
                     endpoint, seq, esp_err_to_name(e));
            result = e;
            break;
        }

        e = wait_ack(seq);
        if (e == ESP_OK) {
            s_ack_ok++;
            ESP_LOGI(TAG,
                     "EP_DATA ACK OK EP%u seq=%u attempt=%d",
                     endpoint, seq, attempt + 1);
            result = ESP_OK;
            break;
        }

        ESP_LOGW(TAG,
                 "EP_DATA ACK FAILED EP%u seq=%u attempt=%d status=%s",
                 endpoint, seq, attempt + 1, esp_err_to_name(e));

        if (e == ESP_ERR_INVALID_STATE) {
            result = e;
            break;
        }

        s_ack_to++;
    }
    xSemaphoreGive(s_ack_tx);
    return result;
}

esp_err_t nuos_link_ping(void)
{
    return write_frame(NUOS_LINK_TYPE_PING, 0, NULL, 0, s_seq++);
}

bool nuos_link_peer_joined(void)
{
    return s_joined;
}

uint32_t nuos_link_peer_silent_ms(void)
{
    if (s_last < 0) return UINT32_MAX;
    return (uint32_t)((esp_timer_get_time() - s_last) / 1000);
}

void nuos_link_log_stats(void)
{
    ESP_LOGI(TAG, "tx=%lu ack=%lu timeout=%lu crc=%lu resync=%lu joined=%d h2_cfg=0x%08lX",
             (unsigned long)s_tx_count, (unsigned long)s_ack_ok,
             (unsigned long)s_ack_to, (unsigned long)s_parser.crc_errors,
             (unsigned long)s_parser.resyncs, s_joined,
             (unsigned long)s_h2_config_hash);
}
