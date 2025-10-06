idf_component_register(SRCS "main.c" "factory_partition.c" "ble_lwm2m.c"
                       PRIV_REQUIRES spi_flash esp_partition mbedtls lwm2m-protobuf nvs_flash bt esp_timer
                       INCLUDE_DIRS "")
