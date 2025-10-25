/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "flash.h"
#include "mbedtls/base64.h"
#include "lwm2m.pb.h"
#include "lwm2m_helpers.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "ble_lwm2m.h"
/* LoRa functionality */
#include "lora.h"

static const char *TAG = "main";

#define BROADCAST_INTERVAL_MS 10000  // 10 seconds

// Global variables for device info
static lwm2m_FactoryPartition g_factory_partition = {0};
static bool g_factory_partition_valid = false;
static bool g_aes_key_exists = false;

/* LoRa receive callback function */
void lora_message_received(const uint8_t* data, size_t length, float rssi, float snr) {
    ESP_LOGI(TAG, "🎯 LoRa message callback triggered!");
    ESP_LOGI(TAG, "   Length: %d bytes", length);
    ESP_LOGI(TAG, "   RSSI: %.2f dBm", rssi);
    ESP_LOGI(TAG, "   SNR: %.2f dB", snr);
    
    // Print as hex for binary data
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, data, length, ESP_LOG_INFO);
    
    // Try to print as string if it appears to be text
    bool is_printable = true;
    for (size_t i = 0; i < length; i++) {
        if (data[i] < 32 && data[i] != '\0' && data[i] != '\n' && data[i] != '\r') {
            is_printable = false;
            break;
        }
    }
    
    if (is_printable && length > 0) {
        char* str_copy = malloc(length + 1);
        if (str_copy) {
            memcpy(str_copy, data, length);
            str_copy[length] = '\0';
            ESP_LOGI(TAG, "   Text: %s", str_copy);
            free(str_copy);
        }
    }
    
    // Signal quality assessment
    if (rssi > -80) {
        ESP_LOGI(TAG, "   Signal quality: Excellent");
    } else if (rssi > -100) {
        ESP_LOGI(TAG, "   Signal quality: Good");
    } else if (rssi > -120) {
        ESP_LOGI(TAG, "   Signal quality: Fair");
    } else {
        ESP_LOGI(TAG, "   Signal quality: Poor");
    }
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
    
    // Check AES key presence via flash module
    g_aes_key_exists = flash_check_aes_key_exists();

    // Load LwM2M factory partition (decode + parse)
    err = flash_load_lwm2m_factory_partition(&g_factory_partition, &g_factory_partition_valid);
    if (err == ESP_OK) {
        flash_debug_print_factory_partition(&g_factory_partition, g_factory_partition_valid);
    } else {
        ESP_LOGW(TAG, "Factory partition load failed: %s", esp_err_to_name(err));
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

    /* Initialize and start LoRa module with listen-before-send pattern */
    ESP_LOGI(TAG, "Initializing LoRa module for ESP32-C6...");
    esp_err_t lora_ret = lora_init();
    if (lora_ret != ESP_OK) {
        ESP_LOGE(TAG, "LoRa init failed: %s", esp_err_to_name(lora_ret));
    } else {
        lora_ret = lora_start_task(lora_message_received);
        if (lora_ret != ESP_OK) {
            ESP_LOGE(TAG, "LoRa task start failed: %s", esp_err_to_name(lora_ret));
        } else {
            ESP_LOGI(TAG, "LoRa module initialized and task started successfully");
            ESP_LOGI(TAG, "🎯 LoRa is now listening with callback support");
            
            // Send a test message after a short delay to demonstrate the functionality
            vTaskDelay(pdMS_TO_TICKS(2000)); // 2 second delay
            lora_send_message("Hello from ESP32-C6 Device! 🚀");
        }
    }

    // Main loop - keep the system running and send periodic LoRa messages
    int message_counter = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "System running... Free heap: %" PRIu32 " bytes", esp_get_free_heap_size());
        
        // Send periodic LoRa status message every 30 seconds
        message_counter++;
        if (message_counter % 6 == 0) { // Every 30 seconds (6 * 5 seconds)
            char status_msg[128];
            snprintf(status_msg, sizeof(status_msg), "ESP32-C6 Status #%d - Heap: %lu bytes", 
                    message_counter / 6, esp_get_free_heap_size());
            lora_send_message(status_msg);
        }
    }
}
