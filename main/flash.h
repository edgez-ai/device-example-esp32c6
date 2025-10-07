/* Flash / Factory data helper module (renamed from factory_partition.*)
 * Provides access to the custom factory data partition plus convenience
 * helpers to read + parse LwM2M factory protobuf and detect stored AES key.
 */

#ifndef FLASH_H
#define FLASH_H

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "esp_partition.h"
#include "lwm2m.pb.h"  /* for lwm2m_FactoryPartition struct */

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Factory partition low level access (same API names with new prefix) ---- */
esp_err_t flash_factory_partition_init(void);
esp_err_t flash_factory_partition_read(size_t offset, void* data, size_t size);
esp_err_t flash_factory_partition_write(size_t offset, const void* data, size_t size);
const esp_partition_t* flash_factory_partition_get_handle(void);

/* ---- Higher level helpers ---- */

/* Check if an AES key is present in NVS (namespace "ble_lwm2m"). */
bool flash_check_aes_key_exists(void);

/* Read raw base64 factory data, decode, parse protobuf into out_partition.
 * Sets *valid=true on success (returns ESP_OK) else *valid=false.
 */
esp_err_t flash_load_lwm2m_factory_partition(lwm2m_FactoryPartition* out_partition, bool* valid);

/* Optional debug printer (safe no-op if partition invalid). */
void flash_debug_print_factory_partition(const lwm2m_FactoryPartition* partition, bool valid);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_H */