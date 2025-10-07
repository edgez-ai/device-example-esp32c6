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
#include "flash.h"
#include "mbedtls/base64.h"
#include "lwm2m.pb.h"
#include "lwm2m_helpers.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "ble_lwm2m.h"

static const char *TAG = "main";

#define BROADCAST_INTERVAL_MS 10000  // 10 seconds

// Global variables for device info
static lwm2m_FactoryPartition g_factory_partition = {0};
static bool g_factory_partition_valid = false;
static bool g_aes_key_exists = false;


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

    // Main loop - just keep the system running
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "System running... Free heap: %" PRIu32 " bytes", esp_get_free_heap_size());
    }
}
