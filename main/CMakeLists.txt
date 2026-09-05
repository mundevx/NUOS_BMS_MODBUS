idf_component_register(
    SRCS 
   
   
   
    SRC_DIRS "." "../../../common/zcl_utility/src"
    INCLUDE_DIRS "." "../../../common/zcl_utility/include" 
    REQUIRES light_driver espressif__esp-zboss-lib mbedtls nvs_flash esp_timer app_update esp_http_server cjson esp_wifi
)
