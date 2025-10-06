idf_component_register(SRCS "main.c" "factory_partition.c"
                       PRIV_REQUIRES spi_flash esp_partition mbedtls lwm2m-protobuf
                       INCLUDE_DIRS "")
