#ifndef BLE_LWM2M_H
#define BLE_LWM2M_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "lwm2m.pb.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize BLE subsystem for LwM2M communication
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ble_lwm2m_init(void);

/**
 * @brief Set the factory partition data for BLE advertising
 * 
 * @param partition Pointer to factory partition data
 * @param valid Whether the partition data is valid
 */
void ble_lwm2m_set_factory_partition(const lwm2m_FactoryPartition* partition, bool valid);

/**
 * @brief Set the AES key status
 * 
 * @param exists Whether AES key exists in NVS
 */
void ble_lwm2m_set_aes_key_status(bool exists);

/**
 * @brief Start periodic BLE broadcasting
 * 
 * @param interval_ms Broadcast interval in milliseconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ble_lwm2m_start_periodic_broadcast(uint32_t interval_ms);

/**
 * @brief Stop periodic BLE broadcasting
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ble_lwm2m_stop_periodic_broadcast(void);

/**
 * @brief Broadcast LwM2M appearance message
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ble_lwm2m_broadcast_appearance(void);

/**
 * @brief Broadcast temperature data
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ble_lwm2m_broadcast_temperature(void);

/**
 * @brief Deinitialize BLE subsystem
 */
void ble_lwm2m_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* BLE_LWM2M_H */