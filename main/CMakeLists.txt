idf_component_register(SRCS "main.c" "factory_partition.c" "ble_lwm2m.c"
                       PRIV_REQUIRES spi_flash esp_partition mbedtls lwm2m-protobuf bt esp_timer
                       REQUIRES nvs_flash
                       INCLUDE_DIRS "")
