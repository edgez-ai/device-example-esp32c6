#ifndef FACTORY_PARTITION_H
#define FACTORY_PARTITION_H

#include "esp_partition.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize and find the factory data partition
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t factory_partition_init(void);

/**
 * @brief Read data from factory partition
 * 
 * @param offset Offset within the factory partition
 * @param data Buffer to read data into
 * @param size Size of data to read
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t factory_partition_read(size_t offset, void* data, size_t size);

/**
 * @brief Write data to factory partition
 * 
 * @param offset Offset within the factory partition
 * @param data Data to write
 * @param size Size of data to write
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t factory_partition_write(size_t offset, const void* data, size_t size);

/**
 * @brief Get the factory partition handle
 * 
 * @return const esp_partition_t* Partition handle or NULL if not initialized
 */
const esp_partition_t* factory_partition_get_handle(void);

#ifdef __cplusplus
}
#endif

#endif // FACTORY_PARTITION_H