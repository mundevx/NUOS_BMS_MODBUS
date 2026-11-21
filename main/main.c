#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#include "app_config.h"
#include "nuos_uart_link.h"
#include "modbus_master.h"
#include "web_server.h"


// #if !CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
// #error "NUOS C6: CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG must be enabled; UART0 is reserved for the H2 binary link."
// #endif


static const char *TAG="NUOS_C6";
void app_main(void)
{
    ESP_ERROR_CHECK(app_config_init());
    ESP_ERROR_CHECK(nuos_link_init());
    ESP_ERROR_CHECK(modbus_master_init());
    ESP_ERROR_CHECK(web_server_start());
    ESP_LOGI(TAG,"ESP32-C6 gateway ready: Web=192.168.4.1, RS485=UART1 GPIO3/2/5");
    for(;;) vTaskDelay(pdMS_TO_TICKS(10000));
}
