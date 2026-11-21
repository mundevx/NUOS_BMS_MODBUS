idf_component_register(
    SRCS
        "esp_zb_controller_main.c"
        "app_zigbee_clusters.c"
        "nuos_uart_link.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        nvs_flash
        esp_timer
        espressif__esp-zboss-lib
        espressif__esp-zigbee-lib
)

# If you keep using the shared zcl_utility helpers from the original tree,
# add them back instead of the SRCS list above:
#
# idf_component_register(
#     SRC_DIRS "." "../../../common/zcl_utility/src"
#     INCLUDE_DIRS "." "../../../common/zcl_utility/include"
#     REQUIRES driver nvs_flash esp_timer espressif__esp-zboss-lib espressif__esp-zigbee-lib
# )
