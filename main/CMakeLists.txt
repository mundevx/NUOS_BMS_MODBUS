idf_component_register(
    SRCS
        "main.c"
        "app_config.c"
        "device_templates.c"
        "modbus_master.c"
        "nuos_uart_link.c"
        "web_server.c"
        "nuos_c6_led.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash esp_http_server esp_wifi esp_event esp_netif cjson esp_timer driver
    EMBED_FILES "index.html"
)
