#include "factory_partition.h"
#include "esp_log.h"
#include "esp_partition.h"

static const char *TAG = "factory_partition";
static const esp_partition_t *s_factory_partition = NULL;

esp_err_t factory_partition_init(void)
{
    // Find the factory data partition (subtype 0x40)
    s_factory_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, 
                                                   (esp_partition_subtype_t)0x40, 
                                                   "factory_data");
    
    if (s_factory_partition == NULL) {
        ESP_LOGE(TAG, "Factory data partition not found");
        return ESP_ERR_NOT_FOUND;
    }
    
    ESP_LOGI(TAG, "Factory data partition found: address=0x%08lx, size=0x%08lx", 
             s_factory_partition->address, s_factory_partition->size);
    
    return ESP_OK;
}

esp_err_t factory_partition_read(size_t offset, void* data, size_t size)
{
    if (s_factory_partition == NULL) {
        ESP_LOGE(TAG, "Factory partition not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (offset + size > s_factory_partition->size) {
        ESP_LOGE(TAG, "Read would exceed partition boundary");
        return ESP_ERR_INVALID_SIZE;
    }
    
    esp_err_t err = esp_partition_read(s_factory_partition, offset, data, size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read from factory partition: %s", esp_err_to_name(err));
    }
    
    return err;
}

esp_err_t factory_partition_write(size_t offset, const void* data, size_t size)
{
    if (s_factory_partition == NULL) {
        ESP_LOGE(TAG, "Factory partition not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (offset + size > s_factory_partition->size) {
        ESP_LOGE(TAG, "Write would exceed partition boundary");
        return ESP_ERR_INVALID_SIZE;
    }
    
    // Erase the sector(s) if writing to the beginning or if needed
    if (offset % 4096 == 0) {
        size_t erase_size = (size + 4095) & ~4095; // Round up to nearest 4KB
        if (offset + erase_size > s_factory_partition->size) {
            erase_size = s_factory_partition->size - offset;
        }
        
        esp_err_t err = esp_partition_erase_range(s_factory_partition, offset, erase_size);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to erase factory partition: %s", esp_err_to_name(err));
            return err;
        }
    }
    
    esp_err_t err = esp_partition_write(s_factory_partition, offset, data, size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write to factory partition: %s", esp_err_to_name(err));
    }
    
    return err;
}

const esp_partition_t* factory_partition_get_handle(void)
{
    return s_factory_partition;
}