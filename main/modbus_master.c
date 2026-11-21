#include "modbus_master.h"
#include "app_config.h"
#include "device_templates.h"
#include "nuos_uart_link.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "cJSON.h"

static const char *TAG="MODBUS";
#define RS485_UART UART_NUM_1
#define RS485_TX 3
#define RS485_RX 2
#define RS485_DE 5
#define MB_TIMEOUT_MS 800
#define MB_BUS_LOCK_TIMEOUT_MS 2000
#define MB_MAX_REGS 125
#define MB_MAX_BITS 2000
#define POLL_PERIOD_MS 1000

static SemaphoreHandle_t s_bus_mutex;
static serial_config_t s_serial;

static SemaphoreHandle_t s_status_mutex;

/* Never let the polling task block forever behind a stuck bus owner. */
static esp_err_t modbus_bus_lock(uint8_t slave, uint8_t fc, uint16_t start)
{
    if (!s_bus_mutex ||
        xSemaphoreTake(s_bus_mutex, pdMS_TO_TICKS(MB_BUS_LOCK_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG,
                 "RS485 bus mutex timeout slave=%u FC%u PDU=%u; poll will retry",
                 slave, fc, start);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

#define MODBUS_STATUS_MAX_VALUE 16
#define MODBUS_STATUS_ERROR_LEN 80

typedef struct {
    bool valid;
    bool ok;
    uint8_t attribute_id;
    uint8_t function;
    uint8_t data_type;
    uint16_t address;
    uint16_t address2;
    uint16_t address3;
    uint8_t packed_count;
    uint32_t reference;
    uint8_t value_len;
    uint8_t value[MODBUS_STATUS_MAX_VALUE];
    char error[MODBUS_STATUS_ERROR_LEN];
} modbus_runtime_attr_status_t;

typedef struct {
    bool configured;
    bool cycle_valid;
    bool cycle_ok;
    uint8_t endpoint;
    uint8_t slave_id;
    uint8_t attribute_count;
    uint64_t last_update_ms;
    char message[MODBUS_STATUS_ERROR_LEN];
    modbus_runtime_attr_status_t *attr;
} modbus_runtime_endpoint_status_t;

static modbus_runtime_endpoint_status_t s_modbus_status[APP_MAX_ENDPOINTS];

static uint32_t status_reference(uint8_t function,uint16_t address)
{
    switch(function){
        case MB_FC03: case MB_FC06: case MB_FC16: return 40001u+address;
        case MB_FC04: return 30001u+address;
        case MB_FC02: return 10001u+address;
        default: return 1u+address;
    }
}

static void status_begin_endpoint(const endpoint_config_t *ep)
{
    if(!ep || ep->slave_id<1 || ep->slave_id>APP_MAX_ENDPOINTS || !s_status_mutex) return;
    xSemaphoreTake(s_status_mutex,portMAX_DELAY);
    modbus_runtime_endpoint_status_t *st=&s_modbus_status[ep->slave_id-1];
    free(st->attr);
    memset(st,0,sizeof(*st));
    st->configured=true; st->endpoint=ep->slave_id; st->slave_id=ep->slave_id;
    st->attribute_count=ep->attribute_count>APP_MAX_ATTRIBUTES?APP_MAX_ATTRIBUTES:ep->attribute_count;
    if(st->attribute_count){
        st->attr=calloc(st->attribute_count,sizeof(*st->attr));
        if(!st->attr){
            st->configured=false;
            xSemaphoreGive(s_status_mutex);
            return;
        }
    }
    st->last_update_ms=(uint64_t)(esp_timer_get_time()/1000);
    snprintf(st->message,sizeof(st->message),"Modbus RTU communication in progress");
    for(uint8_t i=0;i<st->attribute_count;i++){
        const modbus_attribute_t *a=&ep->attributes[i];
        modbus_runtime_attr_status_t *as=&st->attr[i];
        as->attribute_id=a->attribute_id; as->function=a->function; as->data_type=a->data_type;
        as->address=a->address; as->address2=a->address2; as->address3=a->address3;
        as->packed_count=a->packed_count; as->reference=status_reference(a->function,a->address);
        as->valid=a->enabled; as->ok=false;
        if(a->enabled) snprintf(as->error,sizeof(as->error),"Waiting for Modbus response");
    }
    xSemaphoreGive(s_status_mutex);
}

static void status_remove_endpoint(uint8_t ep)
{
    if(ep<1 || ep>APP_MAX_ENDPOINTS || !s_status_mutex) return;
    xSemaphoreTake(s_status_mutex,portMAX_DELAY);
    free(s_modbus_status[ep-1].attr);
    memset(&s_modbus_status[ep-1],0,sizeof(s_modbus_status[ep-1]));
    xSemaphoreGive(s_status_mutex);
}


static uint16_t crc16(const uint8_t *b,size_t n){uint16_t c=0xFFFF;for(size_t i=0;i<n;i++){c^=b[i];for(int j=0;j<8;j++)c=(c&1)?(c>>1)^0xA001:(c>>1);}return c;}

static uart_parity_t parity_from_cfg(uint8_t p){return p==1?UART_PARITY_ODD:p==2?UART_PARITY_EVEN:UART_PARITY_DISABLE;}

esp_err_t modbus_apply_serial_config(void)
{
    if(app_config_load_serial(&s_serial)!=ESP_OK) return ESP_FAIL;
    uart_config_t c={.baud_rate=(int)s_serial.baud_rate,.data_bits=s_serial.data_bits==7?UART_DATA_7_BITS:UART_DATA_8_BITS,.parity=parity_from_cfg(s_serial.parity),.stop_bits=s_serial.stop_bits==2?UART_STOP_BITS_2:UART_STOP_BITS_1,.flow_ctrl=UART_HW_FLOWCTRL_DISABLE,.source_clk=UART_SCLK_DEFAULT};
    esp_err_t e=uart_param_config(RS485_UART,&c); if(e!=ESP_OK)return e;
    ESP_LOGI(TAG,"RS485 UART1 %lu %u data %u stop parity=%u",(unsigned long)s_serial.baud_rate,s_serial.data_bits,s_serial.stop_bits,s_serial.parity);
    return ESP_OK;
}

static esp_err_t rs485_init(void)
{
    gpio_config_t g={.pin_bit_mask=1ULL<<RS485_DE,.mode=GPIO_MODE_OUTPUT,.pull_up_en=GPIO_PULLUP_DISABLE,.pull_down_en=GPIO_PULLDOWN_DISABLE,.intr_type=GPIO_INTR_DISABLE};
    ESP_RETURN_ON_ERROR(gpio_config(&g),TAG,"DE config"); gpio_set_level(RS485_DE,0);
    ESP_RETURN_ON_ERROR(uart_driver_install(RS485_UART,4096,4096,0,NULL,0),TAG,"uart install");
    ESP_RETURN_ON_ERROR(uart_set_pin(RS485_UART,RS485_TX,RS485_RX,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE),TAG,"uart pins");
    return modbus_apply_serial_config();
}

/* Read FC01/02 bits. */
static esp_err_t read_bits(uint8_t slave,uint8_t fc,uint16_t start,uint16_t qty,uint8_t *out,size_t outlen)
{
    if(qty==0||qty>MB_MAX_BITS||outlen<(qty+7)/8)return ESP_ERR_INVALID_ARG;
    uint8_t req[8] = {slave, fc, start >> 8, start & 255, qty >> 8, qty & 255, 0, 0};
    uint16_t c = crc16(req, 6);
    req[6] = c & 255;
    req[7] = c >> 8;
    ESP_LOGI(TAG, "TX slave=%u FC%u PDU=%u qty=%u [%02X %02X %02X %02X %02X %02X %02X %02X]",
             slave, fc, start, qty, req[0], req[1], req[2], req[3],
             req[4], req[5], req[6], req[7]);
    if (modbus_bus_lock(slave, fc, start) != ESP_OK) return ESP_ERR_TIMEOUT;
    uart_flush_input(RS485_UART);
    gpio_set_level(RS485_DE, 1);
    esp_rom_delay_us(100);
    int tx_len = uart_write_bytes(RS485_UART, req, sizeof(req));
    if (tx_len != sizeof(req)) {
        gpio_set_level(RS485_DE, 0);
        xSemaphoreGive(s_bus_mutex);
        ESP_LOGE(TAG, "RS485 TX short write slave=%u FC%u PDU=%u wrote=%d/%u",
                 slave, fc, start, tx_len, (unsigned)sizeof(req));
        return ESP_FAIL;
    }
    esp_err_t tx_done = uart_wait_tx_done(RS485_UART, pdMS_TO_TICKS(MB_TIMEOUT_MS));
    gpio_set_level(RS485_DE, 0);
    if (tx_done != ESP_OK) {
        xSemaphoreGive(s_bus_mutex);
        ESP_LOGE(TAG, "RS485 TX completion timeout slave=%u FC%u PDU=%u: %s",
                 slave, fc, start, esp_err_to_name(tx_done));
        return tx_done;
    }
    uint8_t h[3];
    int n = uart_read_bytes(RS485_UART, h, 3, pdMS_TO_TICKS(MB_TIMEOUT_MS));
    if (n != 3) {
        ESP_LOGW(TAG, "RX timeout waiting for slave=%u FC%u header (%d/3)", slave, fc, n);
        xSemaphoreGive(s_bus_mutex);
        return ESP_ERR_TIMEOUT;
    }
    ESP_LOGI(TAG, "RX header slave=%u FC=%u byte_count_or_exc=%u", h[0], h[1], h[2]);
    if(h[0]!=slave){ESP_LOGW(TAG,"FC%u wrong slave 0x%02X (wanted %u)",fc,h[0],slave);xSemaphoreGive(s_bus_mutex);return ESP_FAIL;}
    if(h[1]&0x80){uint8_t t[2]={0};int tr=uart_read_bytes(RS485_UART,t,2,pdMS_TO_TICKS(100));if(tr==2){uint8_t all[3];
            memcpy(all, h, 3);
            uint16_t rx = t[0] | ((uint16_t)t[1] << 8);
            if (crc16(all, 3) != rx)ESP_LOGW(TAG,"FC%u exception response CRC error",fc);}ESP_LOGW(TAG,"slave %u FC%u exception 0x%02X",slave,fc,h[2]);xSemaphoreGive(s_bus_mutex);return ESP_FAIL;}
    if(h[1]!=(uint8_t)fc){ESP_LOGW(TAG,"FC%u wrong response function 0x%02X",fc,h[1]);xSemaphoreGive(s_bus_mutex);return ESP_FAIL;}
    size_t bytes=h[2]; if(bytes>outlen+2){xSemaphoreGive(s_bus_mutex);return ESP_FAIL;}
    uint8_t buf[MB_MAX_BITS/8+5]; if(bytes+2>sizeof(buf)){xSemaphoreGive(s_bus_mutex);return ESP_ERR_INVALID_SIZE;}
    int r=uart_read_bytes(RS485_UART,buf,bytes+2,pdMS_TO_TICKS(MB_TIMEOUT_MS)); if(r!=(int)bytes+2){xSemaphoreGive(s_bus_mutex);return ESP_ERR_TIMEOUT;}
    uint8_t all[3 + MB_MAX_BITS / 8];
    memcpy(all, h, 3);
    memcpy(all + 3, buf, bytes);
    uint16_t rx = (uint16_t)buf[bytes] | ((uint16_t)buf[bytes + 1] << 8);
    if (crc16(all, 3 + bytes) != rx) {
        ESP_LOGW(TAG, "FC%u CRC error", fc);
        xSemaphoreGive(s_bus_mutex);
        return ESP_FAIL;
    }
    memcpy(out,buf,bytes);xSemaphoreGive(s_bus_mutex);return ESP_OK;
}

/* Read FC03/04. start is zero-based PDU address. */
static esp_err_t read_regs(uint8_t slave, uint8_t fc, uint16_t start,
                           uint16_t qty, uint16_t *out)
{
    if (qty == 0 || qty > MB_MAX_REGS || !out) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t req[8] = { slave, fc, start >> 8, start & 255,
                       qty >> 8, qty & 255, 0, 0 };
    uint16_t c = crc16(req, 6);
    req[6] = c & 255;
    req[7] = c >> 8;

    ESP_LOGI(TAG, "TX slave=%u FC%u PDU=%u qty=%u [%02X %02X %02X %02X %02X %02X %02X %02X]",
             slave, fc, start, qty, req[0], req[1], req[2], req[3],
             req[4], req[5], req[6], req[7]);

    if (modbus_bus_lock(slave, fc, start) != ESP_OK) return ESP_ERR_TIMEOUT;
    uart_flush_input(RS485_UART);
    gpio_set_level(RS485_DE, 1);
    esp_rom_delay_us(100);
    int tx_len = uart_write_bytes(RS485_UART, req, sizeof(req));
    if (tx_len != sizeof(req)) {
        gpio_set_level(RS485_DE, 0);
        xSemaphoreGive(s_bus_mutex);
        ESP_LOGE(TAG, "RS485 TX short write slave=%u FC%u PDU=%u wrote=%d/%u",
                 slave, fc, start, tx_len, (unsigned)sizeof(req));
        return ESP_FAIL;
    }
    esp_err_t tx_done = uart_wait_tx_done(RS485_UART, pdMS_TO_TICKS(MB_TIMEOUT_MS));
    gpio_set_level(RS485_DE, 0);
    if (tx_done != ESP_OK) {
        xSemaphoreGive(s_bus_mutex);
        ESP_LOGE(TAG, "RS485 TX completion timeout slave=%u FC%u PDU=%u: %s",
                 slave, fc, start, esp_err_to_name(tx_done));
        return tx_done;
    }

    uint8_t h[3];
    int n = uart_read_bytes(RS485_UART, h, 3, pdMS_TO_TICKS(MB_TIMEOUT_MS));
    if (n != 3) {
        ESP_LOGW(TAG, "RX timeout waiting for slave=%u FC%u header (%d/3)", slave, fc, n);
        xSemaphoreGive(s_bus_mutex);
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(TAG, "RX header slave=%u FC=%u byte_count_or_exc=%u", h[0], h[1], h[2]);

    if (h[0] != slave) {
        ESP_LOGW(TAG, "FC%u wrong slave 0x%02X (wanted %u)", fc, h[0], slave);
        xSemaphoreGive(s_bus_mutex);
        return ESP_FAIL;
    }

    if (h[1] & 0x80) {
        uint8_t t[2] = {0};
        int tr = uart_read_bytes(RS485_UART, t, 2, pdMS_TO_TICKS(100));
        if (tr == 2) {
            uint8_t all[5];
            memcpy(all, h, 3);
            memcpy(all + 3, t, 2);
            uint16_t rx = (uint16_t)t[0] | ((uint16_t)t[1] << 8);
            if (crc16(all, 3) != rx) {
                ESP_LOGW(TAG, "FC%u exception response CRC error", fc);
            }
        }
        ESP_LOGW(TAG, "slave %u FC%u exception 0x%02X", slave, fc, h[2]);
        xSemaphoreGive(s_bus_mutex);
        return ESP_FAIL;
    }

    if (h[1] != fc) {
        ESP_LOGW(TAG, "FC%u wrong response function 0x%02X", fc, h[1]);
        xSemaphoreGive(s_bus_mutex);
        return ESP_FAIL;
    }

    uint16_t bytes = h[2];
    if (bytes != qty * 2 || bytes + 2 > 2 * MB_MAX_REGS + 2) {
        xSemaphoreGive(s_bus_mutex);
        return ESP_FAIL;
    }

    uint8_t body[2 * MB_MAX_REGS + 2];
    int r = uart_read_bytes(RS485_UART, body, bytes + 2,
                            pdMS_TO_TICKS(MB_TIMEOUT_MS));
    if (r != (int)bytes + 2) {
        xSemaphoreGive(s_bus_mutex);
        return ESP_ERR_TIMEOUT;
    }

    uint8_t all[3 + 2 * MB_MAX_REGS];
    memcpy(all, h, 3);
    memcpy(all + 3, body, bytes);
    uint16_t rx = (uint16_t)body[bytes] | ((uint16_t)body[bytes + 1] << 8);
    if (crc16(all, 3 + bytes) != rx) {
        xSemaphoreGive(s_bus_mutex);
        return ESP_FAIL;
    }

    for (uint16_t i = 0; i < qty; i++) {
        out[i] = ((uint16_t)body[2 * i] << 8) | body[2 * i + 1];
    }

    /*
     * Log every successfully received Modbus register value.
     * start is the zero-based PDU address; convert it back to the
     * displayed 3xxxx/4xxxx reference for easier debugging.
     */
    uint32_t ref_base = (fc == MB_FC04) ? 30001u : 40001u;
    for (uint16_t i = 0; i < qty; i++) {
        uint32_t ref = ref_base + start + i;
        ESP_LOGI(TAG,
                 "RX REGISTER slave=%u FC%u REG=%lu PDU=%u VALUE=%u (0x%04X)",
                 slave,
                 fc,
                 (unsigned long)ref,
                 start + i,
                 out[i],
                 out[i]);
    }

    xSemaphoreGive(s_bus_mutex);
    return ESP_OK;
}

static esp_err_t write_single(uint8_t slave,uint8_t fc,uint16_t addr,uint16_t value)
{
    if (fc != 5 && fc != 6) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t req[8] = {slave, fc, addr >> 8, addr & 255,
                      value >> 8, value & 255, 0, 0};
    uint16_t c = crc16(req, 6);
    req[6] = c & 255;
    req[7] = c >> 8;
    if (modbus_bus_lock(slave, fc, addr) != ESP_OK) return ESP_ERR_TIMEOUT;
    uart_flush_input(RS485_UART);
    gpio_set_level(RS485_DE, 1);
    esp_rom_delay_us(100);
    int tx_len = uart_write_bytes(RS485_UART, req, sizeof(req));
    if (tx_len != sizeof(req)) {
        gpio_set_level(RS485_DE, 0);
        xSemaphoreGive(s_bus_mutex);
        return ESP_FAIL;
    }
    esp_err_t tx_done = uart_wait_tx_done(RS485_UART, pdMS_TO_TICKS(MB_TIMEOUT_MS));
    gpio_set_level(RS485_DE, 0);
    if (tx_done != ESP_OK) {
        xSemaphoreGive(s_bus_mutex);
        return tx_done;
    }
    uint8_t rsp[8];
    int n = uart_read_bytes(RS485_UART, rsp, sizeof(rsp), pdMS_TO_TICKS(MB_TIMEOUT_MS));
    xSemaphoreGive(s_bus_mutex);
    if (n != sizeof(rsp)) return ESP_ERR_TIMEOUT;
    if (memcmp(req, rsp, 6) != 0) return ESP_FAIL;
    uint16_t rx = rsp[6] | ((uint16_t)rsp[7] << 8);
    return crc16(rsp, 6) == rx ? ESP_OK : ESP_FAIL;
}

static esp_err_t write_multiple(uint8_t slave,uint8_t fc,uint16_t addr,uint16_t qty,const uint16_t *values)
{
    if((fc!=15&&fc!=16)||qty==0||qty>123||!values)return ESP_ERR_INVALID_ARG;
    uint16_t data_bytes=(fc==15)?(uint16_t)((qty+7)/8):(uint16_t)(qty*2);
    uint16_t len=(uint16_t)(7+data_bytes+2); uint8_t *req=calloc(1,len); if(!req)return ESP_ERR_NO_MEM;
    req[0]=slave;req[1]=fc;req[2]=addr>>8;req[3]=addr&255;req[4]=qty>>8;req[5]=qty&255;req[6]=(uint8_t)data_bytes;
    if(fc==15){for(uint16_t i=0;i<qty;i++)if(values[i])req[7+i/8]|=(uint8_t)(1u<<(i%8));}
    else for(uint16_t i=0;i<qty;i++){req[7+2*i]=values[i]>>8;req[8+2*i]=values[i]&255;}
    uint16_t c=crc16(req,len-2);req[len-2]=c&255;req[len-1]=c>>8;
    if (modbus_bus_lock(slave, fc, addr) != ESP_OK) {
        free(req);
        return ESP_ERR_TIMEOUT;
    }
    uart_flush_input(RS485_UART);
    gpio_set_level(RS485_DE, 1);
    esp_rom_delay_us(100);
    int tx_len = uart_write_bytes(RS485_UART, req, len);
    if (tx_len != len) {
        gpio_set_level(RS485_DE, 0);
        xSemaphoreGive(s_bus_mutex);
        free(req);
        return ESP_FAIL;
    }
    esp_err_t tx_done = uart_wait_tx_done(RS485_UART, pdMS_TO_TICKS(MB_TIMEOUT_MS));
    gpio_set_level(RS485_DE, 0);
    if (tx_done != ESP_OK) {
        xSemaphoreGive(s_bus_mutex);
        free(req);
        return tx_done;
    }
    uint8_t rsp[8];
    int n = uart_read_bytes(RS485_UART, rsp, sizeof(rsp), pdMS_TO_TICKS(MB_TIMEOUT_MS));
    xSemaphoreGive(s_bus_mutex);
    free(req);
    if (n != sizeof(rsp)) return ESP_ERR_TIMEOUT;
    uint16_t rx = rsp[6] | ((uint16_t)rsp[7] << 8);
    return (rsp[0] == slave && rsp[1] == fc && crc16(rsp, 6) == rx) ? ESP_OK : ESP_FAIL;
}

static uint64_t parse_u64(const char *s){return strtoull(s,NULL,0);}
static void encode_value(const modbus_attribute_t *a,const uint16_t *r,uint8_t *dst,size_t *len)
{
    /* Most Modbus attributes use standard 16-bit register semantics.
     * The LT Panel template is different: each Modbus PDU register carries
     * one 8-bit data byte in its low byte. For that mapping, reg_count equals
     * the attribute byte width (UINT8=1, UINT16=2, ...).
     *
     * UINT8:  attribute_value = low_byte(r[0])
     * UINT16: attribute_value = low_byte(r[0]) |
     *                              (low_byte(r[1]) << 8)
     *
     * Keep a one-register UINT16 as a normal 16-bit Modbus value so existing
     * DG/UPS/etc. templates retain their standard register behavior. */
    if (a->data_type == MB_TYPE_UINT8 ||
        a->data_type == MB_TYPE_INT8) {
        dst[0] = (uint8_t)r[0];
        *len = 1;
        return;
    }

    if (a->data_type == MB_TYPE_UINT16 ||
        a->data_type == MB_TYPE_INT16) {
        uint16_t attribute_value;
        if (a->reg_count >= 2) {
            const uint8_t reg0 = (uint8_t)r[0];
            const uint8_t reg1 = (uint8_t)r[1];
            attribute_value = (uint16_t)reg0 |
                              ((uint16_t)reg1 << 8);
        } else {
            /* Standard Modbus UINT16: one 16-bit register. */
            attribute_value = r[0];
        }

        /* UART numeric payloads are serialized most-significant byte first. */
        dst[0] = (uint8_t)(attribute_value >> 8);
        dst[1] = (uint8_t)attribute_value;
        *len = 2;
        return;
    }

    uint64_t v = 0;
    if (a->reg_count == 1) {
        v = r[0];
    } else if (a->reg_count == 2) {
        v = a->word_order ? (((uint32_t)r[1] << 16) | r[0])
                          : (((uint32_t)r[0] << 16) | r[1]);
    } else {
        for (uint8_t i = 0; i < a->reg_count && i < 4; i++) {
            uint8_t index = a->word_order ? (uint8_t)(a->reg_count - 1u - i) : i;
            v = (v << 16) | r[index];
        }
    }

    /* The UART ABI defines a fixed byte width for the remaining numeric types.
     * UINT8/INT8/UINT16/INT16 were handled above from their response bytes. */
    switch (a->data_type) {
    case MB_TYPE_BOOL:
        dst[0] = (uint8_t)(v ? 1u : 0u);
        *len = 1;
        break;
    case NUOS_TYPE_UINT24:
        dst[0] = (uint8_t)(v >> 16);
        dst[1] = (uint8_t)(v >> 8);
        dst[2] = (uint8_t)v;
        *len = 3;
        break;
    case MB_TYPE_UINT32:
    case MB_TYPE_INT32:
    case MB_TYPE_FLOAT32:
        dst[0] = (uint8_t)(v >> 24);
        dst[1] = (uint8_t)(v >> 16);
        dst[2] = (uint8_t)(v >> 8);
        dst[3] = (uint8_t)v;
        *len = 4;
        break;
    case MB_TYPE_UINT48:
        for (uint8_t i = 0; i < 6; i++) {
            dst[i] = (uint8_t)(v >> (8u * (5u - i)));
        }
        *len = 6;
        break;
    case NUOS_TYPE_UINT64:
    case NUOS_TYPE_INT64:
        for (uint8_t i = 0; i < 8; i++) {
            dst[i] = (uint8_t)(v >> (8u * (7u - i)));
        }
        *len = 8;
        break;
    default:
        /* Preserve the existing raw-register path for STRING/other legacy types. */
        *len = (size_t)a->reg_count * 2u;
        for (size_t i = 0; i < *len; i++) {
            dst[i] = (uint8_t)(v >> (8u * (*len - 1u - i)));
        }
        break;
    }
}

/*
 * Fast Modbus polling
 * -------------------
 * For FC03/FC04, attributes belonging to the same slave/function are grouped
 * into one contiguous read whenever the gap between configured ranges is <=
 * MODBUS_GROUP_GAP_MAX_REGS. Unconfigured registers inside that range are
 * intentionally read too; only configured attributes are forwarded to H2.
 *
 * Example (single-register attributes):
 *   0,1,2,3,4,5,6,16  -> one read: start=0, qty=17
 *   50,51,52,53,54     -> one read: start=50, qty=5
 *
 * A group is split if its total span would exceed the Modbus FC03/FC04 limit
 * of 125 registers. Packed/multi-address attributes are kept on their legacy
 * individual-read path because address2/address3 are separate Modbus ranges.
 */
#define MODBUS_GROUP_GAP_MAX_REGS 10u

typedef struct {
    uint16_t start;
    uint16_t end;       /* exclusive */
    uint8_t function;
    uint16_t attr_count;
    uint16_t *attr_indices;
} modbus_read_group_t;

static bool attr_can_be_grouped(const modbus_attribute_t *a)
{
    return a && a->enabled &&
           (a->function == MB_FC03 || a->function == MB_FC04) &&
           a->reg_count > 0 && a->packed_count == 0;
}

static bool group_can_include(const modbus_read_group_t *g,
                              const modbus_attribute_t *a)
{
    if (!g || !a || a->function != g->function) return false;
    if (a->address < g->start) return false;

    uint32_t gap = (uint32_t)a->address - (uint32_t)g->end;
    if (a->address < g->end) gap = 0; /* overlapping/adjacent range */
    if (gap > MODBUS_GROUP_GAP_MAX_REGS) return false;

    uint32_t new_end = (uint32_t)a->address + a->reg_count;
    return new_end - g->start <= MB_MAX_REGS;
}

static void log_group(const endpoint_config_t *ep, const modbus_read_group_t *g)
{
    uint16_t qty = (uint16_t)(g->end - g->start);
    ESP_LOGI(TAG,
             "FAST GROUP EP%u FC%u START=%u COUNT=%u ATTRS=%u",
             ep->slave_id, g->function, g->start, qty, g->attr_count);
}

static void payload_begin(uint8_t *payload, size_t *plen, uint8_t *count)
{
    *plen = 0;
    *count = 0;
    payload[(*plen)++] = 1; /* payload version */
    payload[(*plen)++] = 0; /* attribute count, filled later */
    payload[(*plen)++] = 0; // Attribute count HIGH byte
}

/*
 * C6 -> H2 collection model
 * -------------------------
 * Read every configured register of one Modbus slave first.  FC03/FC04
 * registers are grouped into the minimum number of contiguous reads using
 * MODBUS_GROUP_GAP_MAX_REGS.  Values are kept in RAM until ALL configured
 * reads for this slave are complete.  Only then is ONE EP_DATA frame built
 * and sent to H2.
 *
 * This prevents the previous behavior where every Modbus group generated a
 * separate H2 UART frame such as 117 bytes, 7 bytes, 12 bytes, etc.
 */
typedef struct {
    bool valid;
    bool changed;               /* differs from the last H2-ACKed value */
    uint8_t value[16];
    uint8_t len;
} collected_attribute_t;

/*
 * Last values successfully delivered to H2.
 * Cache is RAM-only: after C6 reboot every configured value is sent once.
 */
typedef struct {
    bool valid;
    uint8_t len;
    uint8_t value[16];
} modbus_last_sent_attribute_t;

typedef struct {
    bool signature_valid;
    bool peer_unavailable;
    uint32_t config_signature;
    modbus_last_sent_attribute_t attr[APP_MAX_ATTRIBUTES];
} modbus_last_sent_endpoint_t;

static modbus_last_sent_endpoint_t s_last_sent[APP_MAX_ENDPOINTS];

/* In all-endpoints mode, retain only already-encoded changed EP_DATA payloads
 * until every configured Modbus endpoint has completed its read cycle.
 * The endpoint's attributes and collected values are not kept in RAM.
 */
typedef struct {
    uint8_t *payload;
    uint16_t len;
} modbus_pending_endpoint_frame_t;

/* Include the Modbus mapping as well as the Zigbee attribute identity.
 * Changing a register address/type/word order invalidates the value cache.
 */
static uint32_t modbus_endpoint_cache_signature(const endpoint_config_t *ep)
{
    uint32_t crc = 0xFFFFFFFFu;
    uint8_t head[7] = {
        ep->slave_id,
        ep->enabled ? 1u : 0u,
        ep->device_type,
        ep->attribute_count,
        (uint8_t)ep->cluster_id,
        (uint8_t)(ep->cluster_id >> 8),
        0u
    };
    crc = nuos_link_crc32_update(crc, head, sizeof(head));

    for (uint16_t i = 0; i < ep->attribute_count; i++) {
        const modbus_attribute_t *a = &ep->attributes[i];
        uint8_t rec[13] = {
            a->enabled ? 1u : 0u,
            a->attribute_id,
            a->function,
            (uint8_t)a->address,
            (uint8_t)(a->address >> 8),
            a->reg_count,
            a->data_type,
            a->word_order,
            a->packed_count,
            (uint8_t)a->address2,
            (uint8_t)(a->address2 >> 8),
            (uint8_t)a->address3,
            (uint8_t)(a->address3 >> 8)
        };
        crc = nuos_link_crc32_update(crc, rec, sizeof(rec));
    }

    return nuos_link_crc32_finish(crc);
}

static void modbus_cache_prepare_endpoint(const endpoint_config_t *ep,
                                          modbus_last_sent_endpoint_t *cache)
{
    uint32_t signature = modbus_endpoint_cache_signature(ep);
    if (!cache->signature_valid || cache->config_signature != signature) {
        memset(cache->attr, 0, sizeof(cache->attr));
        cache->config_signature = signature;
        cache->signature_valid = true;
        ESP_LOGI(TAG,
                 "EP%u Modbus value cache invalidated (configuration changed)",
                 ep->slave_id);
    }
}

static bool modbus_attribute_value_changed(
    const modbus_last_sent_attribute_t *last,
    const collected_attribute_t *current)
{
    if (!last->valid || last->len != current->len) {
        return true;
    }
    return memcmp(last->value, current->value, current->len) != 0;
}

static void modbus_cache_commit_sent_values(
    const endpoint_config_t *ep,
    const collected_attribute_t *values,
    modbus_last_sent_endpoint_t *cache)
{
    for (uint16_t i = 0; i < ep->attribute_count && i < APP_MAX_ATTRIBUTES; i++) {
        if (!values[i].valid || !values[i].changed) {
            continue;
        }

        modbus_last_sent_attribute_t *last = &cache->attr[i];
        last->len = values[i].len;
        memcpy(last->value, values[i].value, values[i].len);
        last->valid = true;
    }
}

static void status_finalize_endpoint(const endpoint_config_t *ep,
                                     const collected_attribute_t *values,
                                     bool all_valid,
                                     esp_err_t overall)
{
    if(!ep || ep->slave_id<1 || ep->slave_id>APP_MAX_ENDPOINTS || !s_status_mutex) return;
    xSemaphoreTake(s_status_mutex,portMAX_DELAY);
    modbus_runtime_endpoint_status_t *st=&s_modbus_status[ep->slave_id-1];
    st->cycle_valid=true; st->cycle_ok=all_valid;
    st->last_update_ms=(uint64_t)(esp_timer_get_time()/1000);
    snprintf(st->message,sizeof(st->message),all_valid?
             "Last Modbus RTU read successful":"Modbus RTU read error: %s",
             esp_err_to_name(overall));
    for(uint16_t i=0;i<ep->attribute_count && i<APP_MAX_ATTRIBUTES;i++){
        const modbus_attribute_t *a=&ep->attributes[i];
        if(!a->enabled) continue;
        modbus_runtime_attr_status_t *as=&st->attr[i];
        if(values && values[i].valid){
            as->ok=true;
            as->value_len=values[i].len>MODBUS_STATUS_MAX_VALUE?MODBUS_STATUS_MAX_VALUE:values[i].len;
            memcpy(as->value,values[i].value,as->value_len);
            as->error[0]='\0';
        }else{
            as->ok=false; as->value_len=0;
            snprintf(as->error,sizeof(as->error),"Modbus register read failed: %s",esp_err_to_name(overall));
        }
    }
    xSemaphoreGive(s_status_mutex);
}


static void encode_collected_value(const modbus_attribute_t *a,
                                   const uint16_t *regs,
                                   uint8_t reg_count,
                                   uint8_t *dst,
                                   size_t *len)
{
    if (a->packed_count > 1) {
        uint8_t n = a->packed_count;
        if (n > 3) n = 3;

        uint64_t v = 0;
        for (uint8_t i = 0; i < n; i++) {
            uint8_t idx = a->word_order ? (uint8_t)(n - 1u - i) : i;
            v = (v << 16) | regs[idx];
        }

        switch (a->data_type) {
        case MB_TYPE_UINT48:
            for (uint8_t i = 0; i < 6; i++) {
                dst[i] = (uint8_t)(v >> (8u * (5u - i)));
            }
            *len = 6;
            break;
        case MB_TYPE_UINT32:
        case MB_TYPE_INT32:
        case MB_TYPE_FLOAT32:
            dst[0] = (uint8_t)(v >> 24);
            dst[1] = (uint8_t)(v >> 16);
            dst[2] = (uint8_t)(v >> 8);
            dst[3] = (uint8_t)v;
            *len = 4;
            break;
        default:
            /* Packed groups in the current templates are UINT48. */
            *len = 0;
            for (uint8_t i = 0; i < n; i++) {
                dst[(*len)++] = (uint8_t)(regs[i] >> 8);
                dst[(*len)++] = (uint8_t)regs[i];
            }
            break;
        }
        return;
    }

    (void)reg_count;
    encode_value(a, regs, dst, len);
}

static esp_err_t collect_group_values(const endpoint_config_t *ep,
                                      const modbus_read_group_t *g,
                                      collected_attribute_t *values)
{
    uint16_t qty = (uint16_t)(g->end - g->start);
    uint16_t *regs = calloc(qty, sizeof(uint16_t));
    if (!regs) return ESP_ERR_NO_MEM;

    log_group(ep, g);
    esp_err_t e = read_regs(ep->slave_id, g->function, g->start, qty, regs);
    if (e != ESP_OK) {
        ESP_LOGW(TAG,
                 "GROUP READ FAILED EP%u FC%u START=%u COUNT=%u",
                 ep->slave_id, g->function, g->start, qty);
        free(regs);
        return e;
    }

    for (uint16_t i = 0; i < g->attr_count; i++) {
        uint16_t ai = g->attr_indices[i];
        const modbus_attribute_t *a = &ep->attributes[ai];

        if (!a->enabled) continue;
        if (a->address < g->start ||
            (uint32_t)(a->address - g->start) + a->reg_count > qty) {
            ESP_LOGW(TAG, "EP%u ATTR%u outside group", ep->slave_id,
                     a->attribute_id);
            free(regs);
            return ESP_ERR_INVALID_SIZE;
        }

        uint16_t offset = (uint16_t)(a->address - g->start);
        size_t encoded_len = 0;
        encode_collected_value(a, &regs[offset], a->reg_count,
                               values[ai].value, &encoded_len);
        if (encoded_len > sizeof(values[ai].value)) {
            ESP_LOGW(TAG, "EP%u ATTR%u encoded value too large: %u",
                     ep->slave_id, a->attribute_id, (unsigned)encoded_len);
            free(regs);
            return ESP_ERR_INVALID_SIZE;
        }
        values[ai].len = (uint8_t)encoded_len;
        values[ai].valid = true;

        uint32_t ref_base;
        if (ep->device_type == DEVICE_LT_PANEL && a->function == MB_FC04) {
            ref_base = 43001u;
        } else {
            ref_base = (a->function == MB_FC04) ? 30001u : 40001u;
        }
        ESP_LOGI(TAG,
                 "COLLECT EP%u ATTR%u %s REG=%lu PDU=%u RAW_REG0=%u (0x%04X) len=%u",
                 ep->slave_id,
                 a->attribute_id,
                 a->name,
                 (unsigned long)(ref_base + a->address),
                 a->address,
                 regs[offset],
                 regs[offset],
                 values[ai].len);
        ESP_LOG_BUFFER_HEX(TAG, values[ai].value, values[ai].len);
    }

    free(regs);
    return ESP_OK;
}

static esp_err_t collect_single_value(const endpoint_config_t *ep,
                                      uint16_t ai,
                                      collected_attribute_t *values)
{
    const modbus_attribute_t *a = &ep->attributes[ai];
    if (!a->enabled) return ESP_OK;

    uint16_t regs[MB_MAX_REGS] = {0};
    uint8_t bits[MB_MAX_BITS / 8 + 1] = {0};
    esp_err_t e = ESP_OK;

    if (a->function == MB_FC01 || a->function == MB_FC02) {
        e = read_bits(ep->slave_id, a->function, a->address,
                      a->reg_count, bits, sizeof(bits));
        if (e == ESP_OK) {
            values[ai].value[0] = bits[0] & 1u;
            values[ai].len = 1;
            values[ai].valid = true;
        }
    } else if ((a->function == MB_FC03 || a->function == MB_FC04) &&
               a->packed_count > 1) {
        /* Packed attributes use separate Modbus register addresses. */
        uint8_t n = a->packed_count;
        if (n > 3) n = 3;

        uint16_t addresses[3] = { a->address, a->address2, a->address3 };
        for (uint8_t i = 0; i < n; i++) {
            e = read_regs(ep->slave_id, a->function, addresses[i], 1, &regs[i]);
            if (e != ESP_OK) break;
        }
        if (e == ESP_OK) {
            size_t len = 0;
            encode_collected_value(a, regs, n, values[ai].value, &len);
            if (len > sizeof(values[ai].value)) {
                e = ESP_ERR_INVALID_SIZE;
            } else {
                values[ai].len = (uint8_t)len;
                values[ai].valid = true;
            }
        }
    } else if (a->function == MB_FC03 || a->function == MB_FC04) {
        /* Exact fallback for an ordinary (non-packed) register attribute.
         * The previous code returned ESP_ERR_INVALID_ARG here, so every
         * grouped-read failure could never recover. */
        if (a->reg_count == 0 || a->reg_count > MB_MAX_REGS) {
            return ESP_ERR_INVALID_ARG;
        }

        e = read_regs(ep->slave_id, a->function, a->address,
                      a->reg_count, regs);
        if (e == ESP_OK) {
            size_t len = 0;
            encode_collected_value(a, regs, a->reg_count,
                                   values[ai].value, &len);
            if (len > sizeof(values[ai].value)) {
                e = ESP_ERR_INVALID_SIZE;
            } else {
                values[ai].len = (uint8_t)len;
                values[ai].valid = true;
            }
        }
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    if (e != ESP_OK) {
        ESP_LOGW(TAG,
                 "EP%u ATTR%u %s read failed FC%u PDU=%u",
                 ep->slave_id, a->attribute_id, a->name, a->function,
                 a->address);
        return e;
    }

    ESP_LOGI(TAG, "COLLECT EP%u ATTR%u %s len=%u",
             ep->slave_id, a->attribute_id, a->name, values[ai].len);
    ESP_LOG_BUFFER_HEX(TAG, values[ai].value, values[ai].len);
    return ESP_OK;
}

static esp_err_t build_changed_endpoint_payload(
    const endpoint_config_t *ep,
    const collected_attribute_t *values,
    uint8_t *payload,
    uint16_t *payload_len,
    uint16_t *changed_attr_count)
{
    if (!ep || !values || !payload || !payload_len || !changed_attr_count) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t plen = 0;
    uint8_t count = 0;
    payload_begin(payload, &plen, &count);

    for (uint16_t i = 0; i < ep->attribute_count; i++) {
        const modbus_attribute_t *a = &ep->attributes[i];
        if (!a->enabled || !values[i].valid || !values[i].changed) {
            continue;
        }

        if (count == 255u ||
            plen + 3u + values[i].len > NUOS_LINK_MAX_PAYLOAD) {
            ESP_LOGE(TAG,
                     "EP%u changed-attribute frame too large: count=%u len=%u",
                     ep->slave_id, count, (unsigned)plen);
            return ESP_ERR_INVALID_SIZE;
        }

        payload[plen++] = a->attribute_id;
        payload[plen++] = a->data_type;
        payload[plen++] = values[i].len;
        memcpy(&payload[plen], values[i].value, values[i].len);
        plen += values[i].len;
        count++;
    }

    payload[1] = count & 0xFFu;
    payload[2] = (count >> 8) & 0xFFu;
    *payload_len = (uint16_t)plen;
    *changed_attr_count = count;

    if (count > 0) {
        ESP_LOGI(TAG,
                 "EP%u EP_DATA prepared: CHANGED_ATTRS=%u PAYLOAD_LEN=%u",
                 ep->slave_id, count, (unsigned)*payload_len);
    }
    return ESP_OK;
}

/* Commit cache entries from an ACKed EP_DATA payload. This is used by the
 * all-endpoints mode because the collected_attribute_t array is released
 * before the UART batch starts.
 */
static bool modbus_cache_commit_payload(
    const endpoint_config_t *ep,
    const uint8_t *payload,
    uint16_t payload_len,
    modbus_last_sent_endpoint_t *cache)
{
    if (!ep || !payload || !cache || payload_len < 3 || payload[0] != 1u) {
        return false;
    }

    uint16_t count = nuos_link_get_u16(&payload[1]);
    size_t offset = 3;

    for (uint16_t n = 0; n < count; n++) {
        if (offset + 3u > payload_len) return false;

        uint8_t attribute_id = payload[offset++];
        uint8_t data_type = payload[offset++];
        uint8_t value_len = payload[offset++];
        if (offset + value_len > payload_len ||
            value_len > sizeof(cache->attr[0].value)) {
            return false;
        }

        int found = -1;
        for (uint16_t i = 0; i < ep->attribute_count; i++) {
            const modbus_attribute_t *a = &ep->attributes[i];
            if (a->enabled && a->attribute_id == attribute_id &&
                a->data_type == data_type) {
                found = (int)i;
                break;
            }
        }
        if (found < 0 || found >= APP_MAX_ATTRIBUTES) return false;

        modbus_last_sent_attribute_t *last = &cache->attr[found];
        last->len = value_len;
        memcpy(last->value, &payload[offset], value_len);
        last->valid = true;
        offset += value_len;
    }

    return offset == payload_len;
}

static esp_err_t send_pending_endpoint_frame(
    uint8_t endpoint,
    modbus_pending_endpoint_frame_t *pending)
{
    if (!pending || !pending->payload || pending->len == 0) {
        return ESP_OK;
    }

    endpoint_config_t cfg = {0};
    esp_err_t e = app_config_load_endpoint(endpoint, &cfg);
    if (e != ESP_OK) {
        ESP_LOGW(TAG,
                 "BATCH TX EP%u skipped: configuration reload failed: %s",
                 endpoint, esp_err_to_name(e));
        app_config_free_endpoint(&cfg);
        return e;
    }

    if (!cfg.enabled || !cfg.attributes || cfg.attribute_count == 0 ||
        cfg.slave_id != endpoint) {
        ESP_LOGW(TAG, "BATCH TX EP%u skipped: endpoint configuration changed/disabled",
                 endpoint);
        app_config_free_endpoint(&cfg);
        return ESP_ERR_INVALID_STATE;
    }

    modbus_last_sent_endpoint_t *cache = &s_last_sent[endpoint - 1u];
    uint32_t signature = modbus_endpoint_cache_signature(&cfg);
    if (!cache->signature_valid || cache->config_signature != signature) {
        ESP_LOGW(TAG,
                 "BATCH TX EP%u skipped: mapping changed after its Modbus read; retry next cycle",
                 endpoint);
        app_config_free_endpoint(&cfg);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG,
             "BATCH UART SEND EP%u payload_len=%u (all endpoint polling already complete)",
             endpoint, pending->len);
    e = nuos_link_send_generic(endpoint, pending->payload, pending->len);
    if (e == ESP_OK) {
        if (modbus_cache_commit_payload(&cfg, pending->payload,
                                        pending->len, cache)) {
            ESP_LOGI(TAG,
                     "BATCH UART ACK EP%u; changed-attribute cache committed",
                     endpoint);
        } else {
            /* Do not claim delivery in the value cache. Next poll will resend. */
            ESP_LOGE(TAG,
                     "BATCH UART ACK EP%u but payload/cache mapping validation failed; next poll will resend",
                     endpoint);
            e = ESP_FAIL;
        }
    } else {
        ESP_LOGW(TAG,
                 "BATCH UART SEND FAILED EP%u status=%s; cache not updated",
                 endpoint, esp_err_to_name(e));
    }

    app_config_free_endpoint(&cfg);
    return e;
}

static esp_err_t poll_endpoint(const endpoint_config_t *ep,
                                modbus_pending_endpoint_frame_t *pending)
{
    if (!ep || ep->slave_id < 1 || ep->slave_id > APP_MAX_ENDPOINTS ||
        !ep->attributes || ep->attribute_count == 0 ||
        ep->attribute_count > APP_MAX_ATTRIBUTES) {
        if(ep) status_remove_endpoint(ep->slave_id);
        return ESP_ERR_NOT_FOUND;
    }

    status_begin_endpoint(ep);

    collected_attribute_t *values =
        calloc(ep->attribute_count, sizeof(collected_attribute_t));
    uint8_t *used = calloc(ep->attribute_count, sizeof(uint8_t));
    if (!values || !used) {
        free(values);
        free(used);
        return ESP_ERR_NO_MEM;
    }

    esp_err_t overall = ESP_OK;

    /*
     * Phase 1: group all ordinary FC03/FC04 attributes and read every group.
     * Nothing is sent to H2 yet.
     */
    uint16_t remaining = 0;
    for (uint16_t i = 0; i < ep->attribute_count; i++) {
        if (attr_can_be_grouped(&ep->attributes[i])) remaining++;
    }

    while (remaining) {
        int first = -1;
        for (uint16_t i = 0; i < ep->attribute_count; i++) {
            if (used[i] || !attr_can_be_grouped(&ep->attributes[i])) continue;
            if (first < 0 ||
                ep->attributes[i].address < ep->attributes[first].address ||
                (ep->attributes[i].address == ep->attributes[first].address &&
                 ep->attributes[i].attribute_id < ep->attributes[first].attribute_id)) {
                first = (int)i;
            }
        }
        if (first < 0) break;

        uint16_t group_indices[256];
        modbus_read_group_t g = {
            .start = ep->attributes[first].address,
            .end = (uint16_t)(ep->attributes[first].address +
                              ep->attributes[first].reg_count),
            .function = ep->attributes[first].function,
            .attr_count = 0,
            .attr_indices = group_indices
        };

        bool expanded;
        do {
            expanded = false;
            for (uint16_t i = 0; i < ep->attribute_count; i++) {
                if (used[i] || !attr_can_be_grouped(&ep->attributes[i])) continue;
                if (!group_can_include(&g, &ep->attributes[i])) continue;
                if (g.attr_count >= (sizeof(group_indices) / sizeof(group_indices[0]))) break;

                group_indices[g.attr_count++] = i;
                used[i] = 1;
                remaining--;

                uint32_t new_end = (uint32_t)ep->attributes[i].address +
                                   ep->attributes[i].reg_count;
                if (new_end > g.end) g.end = (uint16_t)new_end;
                expanded = true;
            }
        } while (expanded);

        if (g.attr_count) {
            esp_err_t e = collect_group_values(ep, &g, values);
            if (e == ESP_ERR_TIMEOUT) {
                /* A silent slave is not a range/quantity exception.  Do not
                 * issue one timeout per attribute; abort this poll cycle and
                 * let poll_task retry on its next scheduled iteration. */
                ESP_LOGW(TAG,
                         "EP%u slave not responding at FC%u PDU=%u; "
                         "skip exact-read fallback and retry next poll",
                         ep->slave_id, g.function, g.start);
                status_finalize_endpoint(ep, values, false, e);
                free(values);
                free(used);
                return e;
            }
            if (e != ESP_OK) {
                /* Adaptive fallback: use exact reads for a returned Modbus
                 * exception or malformed response, but not for silence.
                 * the slave rejects that range (e.g. exception 0x02), retry
                 * every configured attribute at its exact PDU address. */
                uint16_t recovered = 0;
                ESP_LOGW(TAG,
                         "GROUP READ FAILED EP%u FC%u START=%u COUNT=%u; "
                         "falling back to exact attribute reads",
                         ep->slave_id, g.function, g.start,
                         (unsigned)(g.end - g.start));

                for (uint16_t gi = 0; gi < g.attr_count; gi++) {
                    uint16_t ai = g.attr_indices[gi];
                    esp_err_t fe = collect_single_value(ep, ai, values);
                    if (fe == ESP_ERR_TIMEOUT) {
                        ESP_LOGW(TAG,
                                 "EP%u slave stopped responding during fallback "
                                 "at ATTR%u; aborting remaining reads",
                                 ep->slave_id, ep->attributes[ai].attribute_id);
                        status_finalize_endpoint(ep, values, false, fe);
                        free(values);
                        free(used);
                        return fe;
                    }
                    ESP_LOGI(TAG, "EXACT FALLBACK EP%u ATTR%u result=%s",
                             ep->slave_id, ep->attributes[ai].attribute_id,
                             esp_err_to_name(fe));
                    if (fe == ESP_OK && values[ai].valid) {
                        recovered++;
                    }
                }

                if (recovered != g.attr_count) {
                    ESP_LOGW(TAG,
                             "GROUP FALLBACK PARTIAL EP%u recovered=%u/%u",
                             ep->slave_id, recovered, g.attr_count);
                    overall = e;
                } else {
                    ESP_LOGI(TAG,
                             "GROUP FALLBACK RECOVERED EP%u recovered=%u/%u",
                             ep->slave_id, recovered, g.attr_count);
                }
            }
        }
    }

    /*
     * Phase 2: read the remaining configured attributes (FC01/02 and packed
     * FC03/04).  They are also collected and NOT sent individually.
     */
    for (uint16_t i = 0; i < ep->attribute_count; i++) {
        const modbus_attribute_t *a = &ep->attributes[i];
        if (!a->enabled) continue;
        if (attr_can_be_grouped(a)) continue; /* already read above */

        if (a->function == MB_FC01 || a->function == MB_FC02 ||
            ((a->function == MB_FC03 || a->function == MB_FC04) &&
             a->packed_count > 1)) {
            esp_err_t e = collect_single_value(ep, i, values);
            if (e == ESP_ERR_TIMEOUT) {
                ESP_LOGW(TAG,
                         "EP%u slave timeout at ATTR%u; aborting remaining "
                         "Modbus reads; retry next poll",
                         ep->slave_id, a->attribute_id);
                status_finalize_endpoint(ep, values, false, e);
                free(values);
                free(used);
                return e;
            }
            if (e != ESP_OK) overall = e;
        }
    }

    /*
     * Phase 3: only after ALL configured Modbus reads for this slave are
     * complete, compare each result with the last H2-ACKed value.  Build
     * one EP_DATA delta frame containing only changed attributes.  If any
     * configured read failed, do not send a partial/stale frame.
     */
    bool all_valid = true;
    uint16_t expected = 0;
    uint16_t valid = 0;
    for (uint16_t i = 0; i < ep->attribute_count; i++) {
        if (!ep->attributes[i].enabled) continue;
        if (ep->attributes[i].function == MB_FC03 ||
            ep->attributes[i].function == MB_FC04 ||
            ep->attributes[i].function == MB_FC01 ||
            ep->attributes[i].function == MB_FC02) {
            expected++;
            if (values[i].valid) valid++;
            else all_valid = false;
        }
    }

    status_finalize_endpoint(ep, values, all_valid,
                              overall != ESP_OK ? overall : ESP_ERR_INVALID_STATE);

    if (!all_valid) {
        ESP_LOGW(TAG,
                 "EP%u READ CYCLE INCOMPLETE: valid=%u/%u; "
                 "H2 frame NOT sent",
                 ep->slave_id, valid, expected);
        free(values);
        free(used);
        return overall != ESP_OK ? overall : ESP_ERR_INVALID_STATE;
    }

    /* Only forward values that differ from the last H2-ACKed values. */
    modbus_last_sent_endpoint_t *cache = &s_last_sent[ep->slave_id - 1u];

    /* H2 may reboot or leave/rejoin its Zigbee network while C6 stays up.
     * Invalidate the cache once while H2 is unavailable so the current
     * values are replayed after H2 becomes ready again.
     */
    if (!nuos_link_peer_joined()) {
        if (!cache->peer_unavailable) {
            memset(cache->attr, 0, sizeof(cache->attr));
            cache->signature_valid = false;
            cache->peer_unavailable = true;
            ESP_LOGW(TAG,
                     "EP%u: H2 unavailable; value cache invalidated",
                     ep->slave_id);
        }
    } else {
        cache->peer_unavailable = false;
    }

    modbus_cache_prepare_endpoint(ep, cache);

    uint16_t changed_count = 0;
    for (uint16_t i = 0; i < ep->attribute_count; i++) {
        const modbus_attribute_t *a = &ep->attributes[i];
        values[i].changed = false;
        if (!a->enabled || !values[i].valid) {
            continue;
        }

        values[i].changed = modbus_attribute_value_changed(
            &cache->attr[i], &values[i]);
        if (values[i].changed) {
            changed_count++;
            ESP_LOGI(TAG,
                     "EP%u ATTR%u changed; queueing UART update",
                     ep->slave_id, a->attribute_id);
        }
    }

    if (changed_count == 0) {
        ESP_LOGI(TAG,
                 "EP%u: Modbus poll successful; no value changes; UART EP_DATA not sent",
                 ep->slave_id);
        free(values);
        free(used);
        return overall;
    }

    uint8_t payload[NUOS_LINK_MAX_PAYLOAD];
    uint16_t payload_len = 0;
    uint16_t payload_attr_count = 0;
    esp_err_t build_e = build_changed_endpoint_payload(
        ep, values, payload, &payload_len, &payload_attr_count);
    if (build_e != ESP_OK) {
        free(values);
        free(used);
        return build_e;
    }

    if (payload_len == 0 || payload_attr_count == 0) {
        free(values);
        free(used);
        return overall;
    }

    if (pending) {
        /* Mode 1: save this endpoint's delta. UART transmission is deferred
         * until the complete Modbus sweep over all endpoints has finished. */
        pending->payload = malloc(payload_len);
        if (!pending->payload) {
            ESP_LOGE(TAG,
                     "EP%u: cannot allocate %u-byte batch payload; cache not committed",
                     ep->slave_id, (unsigned)payload_len);
            free(values);
            free(used);
            return ESP_ERR_NO_MEM;
        }
        memcpy(pending->payload, payload, payload_len);
        pending->len = payload_len;
        ESP_LOGI(TAG,
                 "EP%u: queued %u changed attributes for end-of-sweep UART batch",
                 ep->slave_id, payload_attr_count);
    } else {
        /* Mode 2: send immediately after this endpoint's complete read cycle. */
        ESP_LOGI(TAG,
                 "PER-ENDPOINT UART SEND EP%u changed_attrs=%u payload_len=%u",
                 ep->slave_id, payload_attr_count, (unsigned)payload_len);
        esp_err_t send_e = nuos_link_send_generic(
            ep->slave_id, payload, payload_len);
        if (send_e == ESP_OK) {
            modbus_cache_commit_sent_values(ep, values, cache);
            ESP_LOGI(TAG,
                     "EP%u UART ACKED; changed-attribute cache committed",
                     ep->slave_id);
        } else {
            ESP_LOGW(TAG,
                     "EP%u UART SEND FAILED status=%s; cache not updated",
                     ep->slave_id, esp_err_to_name(send_e));
            overall = send_e;
        }
    }

    free(values);
    free(used);
    return overall;
}

static void poll_task(void *arg)
{
    (void)arg;

#if MODBUS_DATA_SEND_MODE == MODBUS_SEND_MODE_ALL_ENDPOINTS_BATCH
    ESP_LOGI(TAG,
             "DATA SEND MODE 1: poll ALL configured endpoints first, then send changed endpoint frames as one UART burst");
#else
    ESP_LOGI(TAG,
             "DATA SEND MODE 2: poll one endpoint completely, send its changed attributes immediately, then continue");
#endif

    uint32_t completed_polls = 0;

    for (;;) {
#if MODBUS_DATA_SEND_MODE == MODBUS_SEND_MODE_ALL_ENDPOINTS_BATCH
        modbus_pending_endpoint_frame_t pending[APP_MAX_ENDPOINTS] = {0};
        uint16_t pending_count = 0;
        ESP_LOGI(TAG, "========== MODBUS FULL SWEEP START ==========");
#endif

        for (uint8_t ep = 1; ep <= APP_MAX_ENDPOINTS; ep++) {
            /* app_config_load_endpoint() allocates cfg.attributes. Every
             * successful load is paired with app_config_free_endpoint(). */
            endpoint_config_t cfg = {0};
            esp_err_t cfg_err = app_config_load_endpoint(ep, &cfg);

            if (cfg_err != ESP_OK) {
                /* Unconfigured endpoint slots are normal; report only actual
                 * errors other than a missing configuration. */
                if (cfg_err != ESP_ERR_NOT_FOUND) {
                    ESP_LOGE(TAG,
                             "EP%u configuration load failed: %s; free_heap=%u",
                             ep, esp_err_to_name(cfg_err),
                             (unsigned)esp_get_free_heap_size());
                }
                app_config_free_endpoint(&cfg);
                continue;
            }

            if (!cfg.enabled || cfg.attribute_count == 0) {
                app_config_free_endpoint(&cfg);
                continue;
            }

#if MODBUS_DATA_SEND_MODE == MODBUS_SEND_MODE_ALL_ENDPOINTS_BATCH
            esp_err_t poll_err = poll_endpoint(&cfg, &pending[ep - 1u]);
            if (pending[ep - 1u].payload) pending_count++;
#else
            esp_err_t poll_err = poll_endpoint(&cfg, NULL);
#endif
            if (poll_err != ESP_OK) {
                ESP_LOGW(TAG, "EP%u poll cycle returned: %s",
                         ep, esp_err_to_name(poll_err));
            }

            app_config_free_endpoint(&cfg);
            completed_polls++;

            if ((completed_polls % 30u) == 0u) {
                ESP_LOGI(TAG,
                         "POLL TASK HEALTH cycles=%lu free_heap=%u min_free_heap=%u stack_hwm=%u words",
                         (unsigned long)completed_polls,
                         (unsigned)esp_get_free_heap_size(),
                         (unsigned)esp_get_minimum_free_heap_size(),
                         (unsigned)uxTaskGetStackHighWaterMark(NULL));
            }

            vTaskDelay(pdMS_TO_TICKS(5));
        }

#if MODBUS_DATA_SEND_MODE == MODBUS_SEND_MODE_ALL_ENDPOINTS_BATCH
        /* NUOS Link's EP_DATA frame header contains exactly one endpoint ID.
         * Therefore the global batch is a back-to-back burst of one frame per
         * changed endpoint, not a single multi-endpoint EP_DATA frame. */
        ESP_LOGI(TAG,
                 "========== MODBUS FULL SWEEP COMPLETE: queued_endpoint_frames=%u; START UART BURST ==========",
                 pending_count);

        uint16_t sent_count = 0;
        for (uint8_t ep = 1; ep <= APP_MAX_ENDPOINTS; ep++) {
            modbus_pending_endpoint_frame_t *p = &pending[ep - 1u];
            if (!p->payload) continue;

            esp_err_t send_err = send_pending_endpoint_frame(ep, p);
            if (send_err == ESP_OK) {
                sent_count++;
            } else {
                ESP_LOGW(TAG, "BATCH SEND EP%u result=%s",
                         ep, esp_err_to_name(send_err));
            }

            free(p->payload);
            p->payload = NULL;
            p->len = 0;
        }

        ESP_LOGI(TAG,
                 "========== UART BURST COMPLETE: sent_frames=%u queued_frames=%u; NEXT MODBUS SWEEP IN %u ms ==========",
                 sent_count, pending_count, (unsigned)POLL_PERIOD_MS);
#else
        ESP_LOGI(TAG, "Per-endpoint sweep complete; next Modbus sweep in %u ms",
                 (unsigned)POLL_PERIOD_MS);
#endif

        vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD_MS));
    }
}

esp_err_t modbus_write_attribute(uint8_t endpoint,uint8_t attribute_id,const char *value)
{
    endpoint_config_t c;if(app_config_load_endpoint(endpoint,&c)!=ESP_OK)return ESP_FAIL;
    if(attribute_id>=c.attribute_count||!value)return ESP_ERR_NOT_FOUND;
    modbus_attribute_t *a=&c.attributes[attribute_id];if(!a->enabled)return ESP_ERR_INVALID_STATE;
    if(a->function==5||a->function==6){uint16_t v=(uint16_t)parse_u64(value);return write_single(endpoint,a->function,a->address,a->function==5?(v?0xFF00:0):v);}
    if(a->function==15||a->function==16){
        uint16_t vals[123]={0};char *tmp=strdup(value);if(!tmp)return ESP_ERR_NO_MEM;uint16_t n=0;char *save=NULL;for(char *tok=strtok_r(tmp,",",&save);tok&&n<123;tok=strtok_r(NULL,",",&save))vals[n++]=(uint16_t)parse_u64(tok);free(tmp);
        if (n == 0) {
            return ESP_ERR_INVALID_ARG;
        }
        if (a->reg_count && n != a->reg_count) {
            return ESP_ERR_INVALID_ARG;
        }
        return write_multiple(endpoint, a->function, a->address, n, vals);
    }
    return ESP_ERR_NOT_SUPPORTED;
}


esp_err_t modbus_status_to_json(cJSON *array)
{
    if(!array || !s_status_mutex) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_status_mutex,portMAX_DELAY);
    for(uint8_t ep=1;ep<=APP_MAX_ENDPOINTS;ep++){
        const modbus_runtime_endpoint_status_t *st=&s_modbus_status[ep-1];
        if(!st->configured) continue;
        cJSON *o=cJSON_CreateObject();
        if(!o){xSemaphoreGive(s_status_mutex);return ESP_ERR_NO_MEM;}
        cJSON_AddNumberToObject(o,"endpoint",st->endpoint);
        cJSON_AddNumberToObject(o,"slave_id",st->slave_id);
        cJSON_AddStringToObject(o,"status",!st->cycle_valid?"no_data":(st->cycle_ok?"ok":"error"));
        cJSON_AddStringToObject(o,"message",st->message);
        cJSON_AddNumberToObject(o,"last_update_ms",(double)st->last_update_ms);
        cJSON *regs=cJSON_AddArrayToObject(o,"registers");
        if(!regs){cJSON_Delete(o);xSemaphoreGive(s_status_mutex);return ESP_ERR_NO_MEM;}
        for(uint8_t i=0;i<st->attribute_count;i++){
            const modbus_runtime_attr_status_t *as=&st->attr[i];
            if(!as->valid) continue;
            cJSON *r=cJSON_CreateObject();
            if(!r){cJSON_Delete(o);xSemaphoreGive(s_status_mutex);return ESP_ERR_NO_MEM;}
            cJSON_AddNumberToObject(r,"attribute_id",as->attribute_id);
            cJSON_AddNumberToObject(r,"function",as->function);
            cJSON_AddNumberToObject(r,"data_type",as->data_type);
            cJSON_AddNumberToObject(r,"reference",as->reference);
            cJSON_AddNumberToObject(r,"address",as->address);
            cJSON_AddNumberToObject(r,"packed_count",as->packed_count);
            cJSON_AddBoolToObject(r,"ok",as->ok);
            if(as->packed_count>1){
                cJSON_AddNumberToObject(r,"address2",as->address2);
                cJSON_AddNumberToObject(r,"address3",as->address3);
            }
            if(as->ok){
                char hex[MODBUS_STATUS_MAX_VALUE*3]={0};
                size_t p=0;
                for(uint8_t k=0;k<as->value_len && p+3<sizeof(hex);k++){
                    p+=(size_t)snprintf(hex+p,sizeof(hex)-p,"%02X%s",as->value[k],(k+1<as->value_len)?" ":"");
                }
                cJSON_AddStringToObject(r,"data_hex",hex);
                uint64_t numeric=0;
                for(uint8_t k=0;k<as->value_len;k++) numeric=(numeric<<8)|as->value[k];
                cJSON_AddNumberToObject(r,"data",(double)numeric);
            }else{
                cJSON_AddStringToObject(r,"error",as->error);
            }
            cJSON_AddItemToArray(regs,r);
        }
        cJSON_AddItemToArray(array,o);
    }
    xSemaphoreGive(s_status_mutex);
    return ESP_OK;
}

esp_err_t modbus_master_init(void)
{
    s_bus_mutex = xSemaphoreCreateMutex();
    s_status_mutex = xSemaphoreCreateMutex();
    if (!s_status_mutex || !s_bus_mutex) {
        return ESP_ERR_NO_MEM;
    }

    ESP_RETURN_ON_ERROR(rs485_init(), TAG, "RS485 init");

    if (xTaskCreate(poll_task, "modbus_poll", 8192, NULL, 7, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Modbus polling task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Modbus polling task started");
    return ESP_OK;
}
