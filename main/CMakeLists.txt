idf_component_register(SRCS "main.cpp" "flash.c" "ble_lwm2m.c"
                       PRIV_REQUIRES spi_flash esp_partition mbedtls lwm2m-protobuf bt esp_timer
                       REQUIRES nvs_flash driver 
                       INCLUDE_DIRS "")
