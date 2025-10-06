/*
 * SPDX-FileCopyrightTex#define BOOT_BUTTON_GPIO    9  // Use GPIO 9 instead of 0 to avoid bootloader conflict
#define BUTTON_PRESS_TIME_MS 3000  // 3 seconds
#define NVS_NAMESPACE "test_storage"
#define NVS_BUTTON_KEY "button_pressed"010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "factory_partition.h"
#include "mbedtls/base64.h"
#include "lwm2m.pb.h"
#include "lwm2m_helpers.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "main";

#define NVS_NAMESPACE "test_storage"
#define NVS_TEST_KEY "boot_count"

static void print_hex_bytes(const char* label, const uint8_t* data, size_t len) {
    if (len == 0) {
        ESP_LOGI(TAG, "%s: (empty)", label);
        return;
    }
    
    char hex_str[len * 2 + 1];
    for (size_t i = 0; i < len; i++) {
        sprintf(&hex_str[i * 2], "%02x", data[i]);
    }
    hex_str[len * 2] = '\0';
    ESP_LOGI(TAG, "%s: %s", label, hex_str);
}

static void print_factory_partition(const lwm2m_FactoryPartition* partition) {
    ESP_LOGI(TAG, "=== Factory Partition Data ===");
    ESP_LOGI(TAG, "Model: %ld", (long)partition->model);
    ESP_LOGI(TAG, "Vendor: %ld", (long)partition->vendor);
    ESP_LOGI(TAG, "Serial: %ld", (long)partition->serial);
    
    print_hex_bytes("Public Key", partition->public_key.bytes, partition->public_key.size);
    print_hex_bytes("Private Key", partition->private_key.bytes, partition->private_key.size);
    
    if (partition->bootstrap_server.size > 0) {
        // Try to print as string (null-terminate safely)
        char server_str[partition->bootstrap_server.size + 1];
        memcpy(server_str, partition->bootstrap_server.bytes, partition->bootstrap_server.size);
        server_str[partition->bootstrap_server.size] = '\0';
        ESP_LOGI(TAG, "Bootstrap Server: %s", server_str);
    } else {
        ESP_LOGI(TAG, "Bootstrap Server: (empty)");
    }
    
    print_hex_bytes("Signature", partition->signature, 64);
    ESP_LOGI(TAG, "=============================");
}

static void test_nvs_functionality(void)
{
    ESP_LOGI(TAG, "=== Testing NVS Functionality ===");
    
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated and will be erased");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    
    nvs_handle_t nvs_handle;
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS handle: %s", esp_err_to_name(err));
        return;
    }
    
    // Try to read existing value
    uint32_t boot_count = 0;
    size_t required_size = sizeof(boot_count);
    err = nvs_get_u32(nvs_handle, NVS_TEST_KEY, &boot_count);
    
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Previous boot count found: %lu", (unsigned long)boot_count);
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "Boot count not found - this appears to be first boot or after factory reset");
        boot_count = 0;
    } else {
        ESP_LOGE(TAG, "Failed to read boot count: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return;
    }
    
    // Increment and save boot count
    boot_count++;
    err = nvs_set_u32(nvs_handle, NVS_TEST_KEY, boot_count);
    if (err == ESP_OK) {
        err = nvs_commit(nvs_handle);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "✓ Boot count updated to: %lu", (unsigned long)boot_count);
            ESP_LOGI(TAG, "✓ NVS write/read test PASSED");
        } else {
            ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(err));
        }
    } else {
        ESP_LOGE(TAG, "Failed to set NVS value: %s", esp_err_to_name(err));
    }
    
    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "=============================");
}

void app_main(void)
{
    printf("Hello world!\n");
    
    // Test NVS functionality to verify factory reset works
    test_nvs_functionality();

    /* Initialize and read factory partition */
    esp_err_t err = factory_partition_init();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Factory partition initialized successfully");
        
        // Read factory data (base64 encoded)
        char factory_data_b64[1024] = {0};  // 1KB buffer for base64 data
        err = factory_partition_read(0, factory_data_b64, sizeof(factory_data_b64) - 1);
        
        if (err == ESP_OK) {
            // Ensure null termination for base64 string
            factory_data_b64[sizeof(factory_data_b64) - 1] = '\0';
            
            // Find the actual length of base64 string (stop at first null or newline)
            size_t b64_len = 0;
            for (size_t i = 0; i < sizeof(factory_data_b64) - 1; i++) {
                if (factory_data_b64[i] == '\0' || factory_data_b64[i] == '\n' || factory_data_b64[i] == '\r') {
                    break;
                }
                b64_len++;
            }
            
            if (b64_len > 0) {
                ESP_LOGI(TAG, "Factory data read successfully (base64 length: %zu)", b64_len);
                ESP_LOGI(TAG, "Base64 data: %.100s%s", factory_data_b64, b64_len > 100 ? "..." : "");
                
                // Decode base64
                uint8_t decoded_data[512];  // Buffer for decoded binary data
                size_t decoded_len = 0;
                
                int ret = mbedtls_base64_decode(decoded_data, sizeof(decoded_data), &decoded_len, 
                                               (const unsigned char*)factory_data_b64, b64_len);
                
                if (ret == 0) {
                    ESP_LOGI(TAG, "Base64 decoded successfully (binary length: %zu)", decoded_len);
                    ESP_LOG_BUFFER_HEX(TAG, decoded_data, decoded_len < 64 ? decoded_len : 64);
                    
                    // Parse protobuf
                    lwm2m_FactoryPartition factory_partition = lwm2m_FactoryPartition_init_zero;
                    int parse_result = lwm2m_read_factory_partition(decoded_data, decoded_len, &factory_partition);
                    
                    if (parse_result == 0) {
                        ESP_LOGI(TAG, "Factory partition protobuf parsed successfully");
                        print_factory_partition(&factory_partition);
                    } else {
                        ESP_LOGE(TAG, "Failed to parse factory partition protobuf (error: %d)", parse_result);
                    }
                } else {
                    ESP_LOGE(TAG, "Failed to decode base64 data (mbedtls error: %d)", ret);
                }
            } else {
                ESP_LOGW(TAG, "Factory data appears to be empty");
            }
        } else {
            ESP_LOGE(TAG, "Failed to read factory data: %s", esp_err_to_name(err));
        }
    } else {
        ESP_LOGE(TAG, "Failed to initialize factory partition: %s", esp_err_to_name(err));
    }

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
