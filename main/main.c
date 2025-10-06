/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
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
#include "ble_lwm2m.h"

static const char *TAG = "main";

#define NVS_NAMESPACE "test_storage"
#define NVS_TEST_KEY "boot_count"
#define NVS_AES_KEY "aes_key"
#define NVS_AES_NAMESPACE "ble_lwm2m"  // Must match namespace used in ble_lwm2m.c for AES key persistence
#define BROADCAST_INTERVAL_MS 10000  // 10 seconds

// Global variables for device info
static lwm2m_FactoryPartition g_factory_partition = {0};
static bool g_factory_partition_valid = false;
static bool g_aes_key_exists = false;

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

static bool check_aes_key_exists(void) {
    // Mirror logic in ble_lwm2m_load_aes_key(): AES key is stored in namespace "ble_lwm2m"
    // with blob key "aes_key" and a length byte "aes_len" that must be 16/24/32.
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_AES_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        if (err != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "AES key namespace open failed: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "AES key namespace not found yet");
        }
        return false;
    }

    uint8_t stored_len = 0;
    err = nvs_get_u8(handle, "aes_len", &stored_len);
    if (err != ESP_OK || !(stored_len == 16 || stored_len == 24 || stored_len == 32)) {
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGI(TAG, "AES key length marker not present");
        } else {
            ESP_LOGW(TAG, "Failed to read AES key length: %s", esp_err_to_name(err));
        }
        nvs_close(handle);
        return false;
    }

    size_t len = stored_len; // only attempt to read expected bytes
    uint8_t tmp_key[32];
    err = nvs_get_blob(handle, NVS_AES_KEY, tmp_key, &len);
    nvs_close(handle);

    if (err == ESP_OK && len == stored_len) {
        ESP_LOGI(TAG, "AES key found in NVS (len=%u)", (unsigned)len);
        return true;
    }

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "AES key blob not found (len marker existed)");
    } else {
        ESP_LOGW(TAG, "Error reading AES key blob: %s", esp_err_to_name(err));
    }
    return false;
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
    printf("LwM2M BLE Device Starting...\n");
    
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated and will be erased");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    
    // Test NVS functionality to verify factory reset works
    test_nvs_functionality();
    
    // Check if AES key exists in NVS
    g_aes_key_exists = check_aes_key_exists();

    /* Initialize and read factory partition */
    err = factory_partition_init();
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
                    g_factory_partition = (lwm2m_FactoryPartition)lwm2m_FactoryPartition_init_zero;
                    int parse_result = lwm2m_read_factory_partition(decoded_data, decoded_len, &g_factory_partition);
                    
                    if (parse_result == 0) {
                        ESP_LOGI(TAG, "Factory partition protobuf parsed successfully");
                        print_factory_partition(&g_factory_partition);
                        g_factory_partition_valid = true;
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

    // Check if we have BLE capability
    if (!(chip_info.features & CHIP_FEATURE_BLE)) {
        ESP_LOGE(TAG, "This chip does not support BLE");
        return;
    }

    // Initialize BLE
    err = ble_lwm2m_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BLE LwM2M initialization failed");
        return;
    }

    // Configure BLE with factory partition and AES key status
    ble_lwm2m_set_factory_partition(&g_factory_partition, g_factory_partition_valid);
    ble_lwm2m_set_aes_key_status(g_aes_key_exists);

    // Log the operational mode
    if (g_aes_key_exists) {
        ESP_LOGI(TAG, "AES key found - will broadcast temperature data every %d seconds", BROADCAST_INTERVAL_MS / 1000);
    } else {
        ESP_LOGI(TAG, "No AES key found - will broadcast LwM2M appearance every %d seconds", BROADCAST_INTERVAL_MS / 1000);
    }

    // Start periodic broadcasting
    err = ble_lwm2m_start_periodic_broadcast(BROADCAST_INTERVAL_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start periodic broadcasting");
        return;
    }

    ESP_LOGI(TAG, "LwM2M BLE Device initialized successfully");
    ESP_LOGI(TAG, "Broadcasting will begin in %d seconds...", BROADCAST_INTERVAL_MS / 1000);

    // Main loop - just keep the system running
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "System running... Free heap: %" PRIu32 " bytes", esp_get_free_heap_size());
    }
}
