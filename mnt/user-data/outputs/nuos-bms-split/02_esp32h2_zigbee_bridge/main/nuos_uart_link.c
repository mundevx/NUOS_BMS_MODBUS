/*
 * NUOS BMS - ESP32-H2 side of the UART link (receiver)
 *
 * Three tasks:
 *
 *   nuos_link_rx   - drains the UART, runs the byte parser, and either
 *                    answers immediately (PING) or queues work.
 *   nuos_link_work - takes one endpoint off the queue, writes the 10
 *                    attributes into the ZCL database, emits the chunked
 *                    bulk report, then ACKs the C6.
 *   nuos_link_hb   - STATUS heartbeat so the C6 knows whether the Zigbee
 *                    network is joined.
 *
 * The parser is never allowed to block on Zigbee, which is why the work
 * queue exists: an APSDE-DATA request with ACK_TX can take tens of
 * milliseconds and the UART would otherwise overrun.
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_zigbee_core.h"

#include "nuos_uart_link.h"
#include "app_zigbee_clusters.h"

static const char *TAG = "NUOS_LINK_RX";

#define LINK_RX_BUF_SIZE        1024
#define LINK_TX_BUF_SIZE        256

typedef struct {
    uint8_t type;
    uint8_t seq;
    uint8_t endpoint;
    uint8_t len;
    uint8_t payload[NUOS_LINK_MAX_PAYLOAD];
} link_work_t;

static QueueHandle_t     s_work_q;
static SemaphoreHandle_t s_tx_mutex;

static nuos_link_parser_t s_parser;

static volatile bool     s_zb_started;

static volatile uint32_t s_rx_frames;
static volatile uint32_t s_applied;
static volatile uint32_t s_dropped_busy;
static volatile uint32_t s_dropped_not_joined;

/* -------------------------------------------------------------------------- */

void nuos_link_notify_zb_started(void)
{
    s_zb_started = true;
}

static bool link_zb_joined(void)
{
    /* esp_zb_lock_acquire() is only valid once the stack is up. */
    if (!s_zb_started) {
        return false;
    }

    /*
     * esp_zb_bdb_dev_joined() reads stack state, so it is taken under the
     * Zigbee lock like every other stack call from a non-Zigbee task.
     */
    esp_zb_lock_acquire(portMAX_DELAY);
    bool joined = esp_zb_bdb_dev_joined();
    esp_zb_lock_release();

    return joined;
}

static esp_err_t link_write_frame(uint8_t type,
                                  uint8_t seq,
                                  uint8_t endpoint,
                                  const uint8_t *payload,
                                  uint8_t len)
{
    uint8_t frame[NUOS_LINK_MAX_FRAME];

    uint16_t total = nuos_link_build(frame, type, seq, endpoint, payload, len);

    if (total == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    int written = uart_write_bytes(NUOS_LINK_UART, frame, total);

    xSemaphoreGive(s_tx_mutex);

    return (written == (int)total) ? ESP_OK : ESP_FAIL;
}

static void link_send_ack(uint8_t seq, uint8_t endpoint, uint8_t status)
{
    uint8_t type = (status == NUOS_LINK_ST_OK)
                       ? NUOS_LINK_TYPE_ACK
                       : NUOS_LINK_TYPE_NAK;

    link_write_frame(type, seq, endpoint, &status, 1);
}

static void link_send_status(void)
{
    uint8_t flags = link_zb_joined() ? NUOS_LINK_FLAG_ZB_JOINED : 0;

    link_write_frame(NUOS_LINK_TYPE_STATUS, 0, 0, &flags, 1);
}

/* -------------------------------------------------------------------------- */
/* Worker task - the only place that touches the Zigbee layer                 */
/* -------------------------------------------------------------------------- */

static uint8_t link_apply_ep_data(const link_work_t *w)
{
    if (w->len != NUOS_LINK_EP_DATA_LEN) {
        ESP_LOGW(TAG,
                 "EP%u bad payload length %u (expected %u)",
                 w->endpoint,
                 w->len,
                 (unsigned)NUOS_LINK_EP_DATA_LEN);
        return NUOS_LINK_ST_BAD_LENGTH;
    }

    if (w->endpoint < NUOS_FIRST_ENDPOINT ||
        w->endpoint >= NUOS_FIRST_ENDPOINT + NUOS_TOTAL_ENDPOINTS) {
        ESP_LOGW(TAG, "Bad endpoint %u", w->endpoint);
        return NUOS_LINK_ST_BAD_ENDPOINT;
    }

    uint32_t values[NUOS_ATTR_COUNT];

    nuos_link_unpack_values(w->payload, values, NUOS_ATTR_COUNT);

    ESP_LOGI(TAG,
             "EP%u <- link: [%lu, %lu, %lu, %lu, %lu, %lu, %lu, %lu, %lu, %lu]",
             w->endpoint,
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

    /*
     * Same call the single-chip build made straight from the Modbus task:
     * write all 10 attributes, then emit the chunked bulk report.
     */
    esp_err_t ret = nuos_update_endpoint_values(w->endpoint, values);

    if (ret == ESP_ERR_INVALID_STATE) {
        s_dropped_not_joined++;
        return NUOS_LINK_ST_NOT_JOINED;
    }

    if (ret != ESP_OK) {
        ESP_LOGW(TAG,
                 "EP%u Zigbee update/report failed: %s",
                 w->endpoint,
                 esp_err_to_name(ret));
        return NUOS_LINK_ST_ZB_ERROR;
    }

    s_applied++;

    return NUOS_LINK_ST_OK;
}

/*
 * Passthrough mode: the C6 built the ZCL frame itself and we hand the ASDU
 * to APSDE-DATA untouched.
 *
 *   payload[0..1] cluster id
 *   payload[2..3] destination short address
 *   payload[4]    destination endpoint
 *   payload[5..]  ASDU
 */
static uint8_t link_apply_raw_asdu(const link_work_t *w)
{
    if (w->len <= 5) {
        return NUOS_LINK_ST_BAD_LENGTH;
    }

    uint16_t cluster_id = nuos_link_get_u16(&w->payload[0]);
    uint16_t dst_short  = nuos_link_get_u16(&w->payload[2]);
    uint8_t  dst_ep     = w->payload[4];
    uint8_t  asdu_len   = (uint8_t)(w->len - 5u);

    esp_err_t ret = nuos_send_raw_asdu(w->endpoint,
                                       cluster_id,
                                       dst_short,
                                       dst_ep,
                                       &w->payload[5],
                                       asdu_len);

    if (ret == ESP_ERR_INVALID_STATE) {
        s_dropped_not_joined++;
        return NUOS_LINK_ST_NOT_JOINED;
    }

    if (ret != ESP_OK) {
        return NUOS_LINK_ST_ZB_ERROR;
    }

    s_applied++;

    return NUOS_LINK_ST_OK;
}

static void link_work_task(void *arg)
{
    (void)arg;

    link_work_t work;

    while (1) {

        if (xQueueReceive(s_work_q, &work, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        uint8_t status;

        if (!link_zb_joined()) {

            s_dropped_not_joined++;
            status = NUOS_LINK_ST_NOT_JOINED;

        } else if (work.type == NUOS_LINK_TYPE_EP_DATA) {

            status = link_apply_ep_data(&work);

        } else {

            status = link_apply_raw_asdu(&work);
        }

        link_send_ack(work.seq, work.endpoint, status);
    }
}

/* -------------------------------------------------------------------------- */
/* RX task                                                                    */
/* -------------------------------------------------------------------------- */

static void link_handle_frame(const nuos_link_frame_t *f)
{
    s_rx_frames++;

    switch (f->type) {

    case NUOS_LINK_TYPE_PING:
        link_send_status();
        break;

    case NUOS_LINK_TYPE_EP_DATA:
    case NUOS_LINK_TYPE_RAW_ASDU: {

        link_work_t work = {
            .type     = f->type,
            .seq      = f->seq,
            .endpoint = f->endpoint,
            .len      = f->len,
        };

        memcpy(work.payload, f->payload, f->len);

        if (xQueueSend(s_work_q, &work, 0) != pdTRUE) {

            s_dropped_busy++;

            ESP_LOGW(TAG,
                     "Work queue full, dropping EP%u seq=%u",
                     f->endpoint,
                     f->seq);

            link_send_ack(f->seq, f->endpoint, NUOS_LINK_ST_BUSY);
        }

        break;
    }

    default:
        ESP_LOGD(TAG, "Unhandled frame type 0x%02X", f->type);
        break;
    }
}

static void link_rx_task(void *arg)
{
    (void)arg;

    uint8_t chunk[128];
    nuos_link_frame_t frame;

    nuos_link_parser_init(&s_parser);

    while (1) {

        int len = uart_read_bytes(NUOS_LINK_UART,
                                  chunk,
                                  sizeof(chunk),
                                  pdMS_TO_TICKS(50));

        for (int i = 0; i < len; i++) {

            if (nuos_link_parser_push(&s_parser, chunk[i], &frame)) {
                link_handle_frame(&frame);
            }
        }
    }
}

static void link_heartbeat_task(void *arg)
{
    (void)arg;

    while (1) {

        link_send_status();

        vTaskDelay(pdMS_TO_TICKS(NUOS_LINK_HEARTBEAT_MS));
    }
}

/* -------------------------------------------------------------------------- */
/* Init                                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t nuos_link_init(void)
{
    s_work_q   = xQueueCreate(NUOS_LINK_QUEUE_DEPTH, sizeof(link_work_t));
    s_tx_mutex = xSemaphoreCreateMutex();

    if (s_work_q == NULL || s_tx_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    uart_config_t uart_config = {
        .baud_rate  = NUOS_LINK_BAUDRATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(
        uart_driver_install(NUOS_LINK_UART,
                            LINK_RX_BUF_SIZE,
                            LINK_TX_BUF_SIZE,
                            0,
                            NULL,
                            0),
        TAG,
        "link UART driver install failed");

    ESP_RETURN_ON_ERROR(
        uart_param_config(NUOS_LINK_UART, &uart_config),
        TAG,
        "link UART param config failed");

    ESP_RETURN_ON_ERROR(
        uart_set_pin(NUOS_LINK_UART,
                     NUOS_LINK_TX_PIN,
                     NUOS_LINK_RX_PIN,
                     UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE),
        TAG,
        "link UART pin config failed");

    if (xTaskCreate(link_rx_task, "nuos_link_rx", 3072, NULL, 7, NULL)
            != pdPASS) {
        ESP_LOGE(TAG, "Failed to create link RX task");
        return ESP_FAIL;
    }

    if (xTaskCreate(link_work_task, "nuos_link_work", 4096, NULL, 5, NULL)
            != pdPASS) {
        ESP_LOGE(TAG, "Failed to create link worker task");
        return ESP_FAIL;
    }

    if (xTaskCreate(link_heartbeat_task, "nuos_link_hb", 2560, NULL, 4, NULL)
            != pdPASS) {
        ESP_LOGE(TAG, "Failed to create link heartbeat task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG,
             "Link UART%d TX=%d RX=%d baud=%d queue=%d",
             NUOS_LINK_UART,
             NUOS_LINK_TX_PIN,
             NUOS_LINK_RX_PIN,
             NUOS_LINK_BAUDRATE,
             NUOS_LINK_QUEUE_DEPTH);

    return ESP_OK;
}

void nuos_link_log_stats(void)
{
    ESP_LOGI(TAG,
             "link stats: rx=%lu applied=%lu busy_drop=%lu not_joined=%lu "
             "crc_err=%lu resync=%lu",
             (unsigned long)s_rx_frames,
             (unsigned long)s_applied,
             (unsigned long)s_dropped_busy,
             (unsigned long)s_dropped_not_joined,
             (unsigned long)s_parser.crc_errors,
             (unsigned long)s_parser.resyncs);
}
