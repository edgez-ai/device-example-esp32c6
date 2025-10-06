#include "ble_lwm2m.h"
#include <string.h>
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "freertos/semphr.h"
#include "pb_encode.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

static const char *TAG = "ble_lwm2m";

#define EXT_ADV_HANDLE      0
#define NUM_EXT_ADV         1

// Separate variable for handle so we can take its address when stopping
static uint8_t ext_adv_handle = EXT_ADV_HANDLE;

// Static variables for BLE state
static lwm2m_FactoryPartition g_factory_partition = {0};
static bool g_factory_partition_valid = false;
static bool g_aes_key_exists = false;
static TimerHandle_t broadcast_timer = NULL;
static bool ble_initialized = false;
static TaskHandle_t broadcast_task_handle = NULL;
static bool broadcast_task_running = false;
static SemaphoreHandle_t ble_sem = NULL;

// Forward declaration for internal helper used by broadcast task
static bool ble_lwm2m_check_aes_key(uint32_t *boot_count_out);

// Extended advertising parameters for periodic broadcasting
static esp_ble_gap_ext_adv_params_t ext_adv_params = {
    .type = ESP_BLE_GAP_SET_EXT_ADV_PROP_NONCONN_NONSCANNABLE_UNDIRECTED,
    .interval_min = 0x20,  // ~20ms interval
    .interval_max = 0x20,
    .channel_map = ADV_CHNL_ALL,
    .filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    .primary_phy = ESP_BLE_GAP_PHY_1M,
    .max_skip = 0,
    .secondary_phy = ESP_BLE_GAP_PHY_1M,
    .sid = 0,
    .scan_req_notif = false,
    .own_addr_type = BLE_ADDR_TYPE_RANDOM,
    .tx_power = EXT_ADV_TX_PWR_NO_PREFERENCE,
};

static esp_ble_gap_ext_adv_t ext_adv[1] = {
    [0] = {EXT_ADV_HANDLE, 0, 0},
};

#define FUNC_SEND_WAIT_SEM(func, sem) do {\
        esp_err_t __err_rc = (func);\
        if (__err_rc != ESP_OK) { \
            ESP_LOGE(TAG, "%s, message send fail, error = %d", __func__, __err_rc); \
        } else { \
            xSemaphoreTake(sem, portMAX_DELAY); \
        } \
} while(0);

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_EXT_ADV_SET_RAND_ADDR_COMPLETE_EVT:
        xSemaphoreGive(ble_sem);
        ESP_LOGI(TAG, "Extended advertising random address set, status %d", param->ext_adv_set_rand_addr.status);
        break;
    case ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT:
        xSemaphoreGive(ble_sem);
        ESP_LOGI(TAG, "Extended advertising params set, status %d", param->ext_adv_set_params.status);
        break;
    case ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT:
        xSemaphoreGive(ble_sem);
        ESP_LOGI(TAG, "Extended advertising data set, status %d", param->ext_adv_data_set.status);
        break;
    case ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT:
        xSemaphoreGive(ble_sem);
        ESP_LOGI(TAG, "Extended advertising start, status %d", param->ext_adv_start.status);
        break;
    case ESP_GAP_BLE_EXT_ADV_STOP_COMPLETE_EVT:
        xSemaphoreGive(ble_sem);
        ESP_LOGI(TAG, "Extended advertising stop, status %d", param->ext_adv_stop.status);
        break;
    default:
        break;
    }
}

static esp_err_t start_extended_advertising(void) {
    esp_err_t ret = esp_ble_gap_ext_adv_start(1, ext_adv);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start extended advertising: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Wait for advertising start complete
    xSemaphoreTake(ble_sem, portMAX_DELAY);
    return ESP_OK;
}

static esp_err_t stop_extended_advertising(void) {
    esp_err_t ret = esp_ble_gap_ext_adv_stop(1, &ext_adv_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop extended advertising: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Wait for advertising stop complete
    xSemaphoreTake(ble_sem, portMAX_DELAY);
    return ESP_OK;
}

static void broadcast_task(void* param) {
    uint32_t boot_count;
    bool has_aes_key = false; // default until helper fills it
    // Helper will populate boot_count and return AES key presence
    has_aes_key = ble_lwm2m_check_aes_key(&boot_count);
    
    ESP_LOGI(TAG, "Broadcast task started, has_aes_key: %s, boot_count: %lu", 
             has_aes_key ? "true" : "false", boot_count);
    
    while (1) {
        // Wait for timer notification
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        ESP_LOGI(TAG, "Timer triggered, broadcasting...");
        
        // Stop any existing advertising
        stop_extended_advertising();
        
        // Set new advertising data
        if (has_aes_key) {
            ble_lwm2m_broadcast_temperature();
        } else {
            ble_lwm2m_broadcast_appearance();
        }
        
        // Start advertising with new data
        start_extended_advertising();
        
        ESP_LOGI(TAG, "Broadcast complete, waiting for next timer...");
    }
}

// ---------------------------------------------------------------------------
// Internal helpers
// Placeholder implementation – replace with real NVS / secure storage lookup
// Returns whether an AES key exists and optionally sets boot_count.
static bool ble_lwm2m_check_aes_key(uint32_t *boot_count_out) {
    if (boot_count_out) {
        // For now just return 0; integrate with your boot counter source if needed
        *boot_count_out = 0;
    }
    return g_aes_key_exists;
}

static void broadcast_timer_callback(TimerHandle_t xTimer) {
    // Just notify the broadcast task instead of doing work here
    if (broadcast_task_handle != NULL) {
        xTaskNotifyGive(broadcast_task_handle);
    }
}

esp_err_t ble_lwm2m_init(void) {
    if (ble_initialized) {
        ESP_LOGW(TAG, "BLE already initialized");
        return ESP_OK;
    }
    
    esp_err_t ret;
    
    // Release memory for classic BT (not needed)
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    
    // Initialize BT controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "Initialize controller failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "Enable controller failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize Bluedroid
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "Init bluetooth failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "Enable bluetooth failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "Gap register callback failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Create semaphore for synchronization
    ble_sem = xSemaphoreCreateBinary();
    if (ble_sem == NULL) {
        ESP_LOGE(TAG, "Failed to create BLE semaphore");
        return ESP_ERR_NO_MEM;
    }
    
    // Create random address
    esp_bd_addr_t rand_addr;
    esp_ble_gap_addr_create_static(rand_addr);
    ESP_LOG_BUFFER_HEX(TAG, rand_addr, ESP_BD_ADDR_LEN);
    
    // Set extended advertising parameters
    FUNC_SEND_WAIT_SEM(esp_ble_gap_ext_adv_set_params(EXT_ADV_HANDLE, &ext_adv_params), ble_sem);
    FUNC_SEND_WAIT_SEM(esp_ble_gap_ext_adv_set_rand_addr(EXT_ADV_HANDLE, rand_addr), ble_sem);
    
    ble_initialized = true;
    ESP_LOGI(TAG, "Bluedroid BLE initialized successfully");
    return ESP_OK;
}

void ble_lwm2m_set_factory_partition(const lwm2m_FactoryPartition* partition, bool valid) {
    if (partition && valid) {
        memcpy(&g_factory_partition, partition, sizeof(lwm2m_FactoryPartition));
        g_factory_partition_valid = true;
        ESP_LOGI(TAG, "Factory partition data set for BLE advertising");
    } else {
        g_factory_partition_valid = false;
        ESP_LOGW(TAG, "Invalid factory partition data provided");
    }
}

void ble_lwm2m_set_aes_key_status(bool exists) {
    g_aes_key_exists = exists;
    ESP_LOGI(TAG, "AES key status set to: %s", exists ? "exists" : "not found");
}

esp_err_t ble_lwm2m_start_periodic_broadcast(uint32_t interval_ms) {
    if (!ble_initialized) {
        ESP_LOGE(TAG, "BLE not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Stop any existing broadcast
    ble_lwm2m_stop_periodic_broadcast();
    
    // Create the broadcast task
    broadcast_task_running = true;
    BaseType_t result = xTaskCreate(broadcast_task, 
                                   "ble_broadcast", 
                                   4096,  // Stack size
                                   NULL,  // Parameters
                                   5,     // Priority
                                   &broadcast_task_handle);
    
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create broadcast task");
        broadcast_task_running = false;
        return ESP_ERR_NO_MEM;
    }
    
    // Create the broadcast timer
    broadcast_timer = xTimerCreate("BroadcastTimer", 
                                   pdMS_TO_TICKS(interval_ms),
                                   pdTRUE,  // Auto-reload
                                   NULL,    // Timer ID
                                   broadcast_timer_callback);
                                   
    if (broadcast_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create broadcast timer");
        // Clean up task
        broadcast_task_running = false;
        xTaskNotifyGive(broadcast_task_handle);
        return ESP_ERR_NO_MEM;
    }

    // Start the timer
    if (xTimerStart(broadcast_timer, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start broadcast timer");
        xTimerDelete(broadcast_timer, 0);
        broadcast_timer = NULL;
        // Clean up task
        broadcast_task_running = false;
        xTaskNotifyGive(broadcast_task_handle);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Periodic broadcast started with interval %lu ms", interval_ms);
    return ESP_OK;
}

esp_err_t ble_lwm2m_stop_periodic_broadcast(void) {
    bool stopped_something = false;
    
    // Stop timer
    if (broadcast_timer != NULL) {
        xTimerStop(broadcast_timer, 0);
        xTimerDelete(broadcast_timer, 0);
        broadcast_timer = NULL;
        stopped_something = true;
    }
    
    // Stop task
    if (broadcast_task_handle != NULL) {
        broadcast_task_running = false;
        xTaskNotifyGive(broadcast_task_handle);
        // Wait a bit for task to clean up
        vTaskDelay(pdMS_TO_TICKS(100));
        stopped_something = true;
    }
    
    // Stop advertising
    stop_extended_advertising();
    
    if (stopped_something) {
        ESP_LOGI(TAG, "Periodic broadcast stopped");
        return ESP_OK;
    }
    
    ESP_LOGW(TAG, "No broadcast components to stop");
    return ESP_ERR_INVALID_STATE;
}

esp_err_t ble_lwm2m_broadcast_appearance(void) {
    if (!ble_initialized) {
        ESP_LOGE(TAG, "BLE not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!g_factory_partition_valid) {
        ESP_LOGE(TAG, "Factory partition not valid, cannot broadcast appearance");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Create LwM2MMessage with appearance
    lwm2m_LwM2MMessage message = lwm2m_LwM2MMessage_init_zero;
    message.timestamp = esp_timer_get_time() / 1000; // Convert to milliseconds
    message.which_body = lwm2m_LwM2MMessage_appearance_tag;
    
    // Fill appearance data
    message.body.appearance.model = g_factory_partition.model;
    message.body.appearance.serial = g_factory_partition.serial;
    message.body.appearance.public_key.size = g_factory_partition.public_key.size;
    memcpy(message.body.appearance.public_key.bytes, 
           g_factory_partition.public_key.bytes, 
           g_factory_partition.public_key.size);
    
    // Serialize the message
    uint8_t buffer[lwm2m_LwM2MMessage_size];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    
    if (!pb_encode(&stream, lwm2m_LwM2MMessage_fields, &message)) {
        ESP_LOGE(TAG, "Failed to encode LwM2MMessage: %s", PB_GET_ERROR(&stream));
        return ESP_FAIL;
    }
    
    size_t message_length = stream.bytes_written;
    ESP_LOGI(TAG, "Encoded LwM2MMessage appearance, length: %zu", message_length);
    
    // Create advertising data in the format expected by extended advertising
    // Format: [Length][Type][Data]...
    uint8_t adv_data[31];
    uint8_t adv_len = 0;
    
    // Add flags
    adv_data[adv_len++] = 2;      // Length
    adv_data[adv_len++] = 0x01;   // Flags type
    adv_data[adv_len++] = 0x06;   // BR/EDR not supported, General discoverable
    
    // Add name
    const char* name = "LwM2M-App";
    uint8_t name_len = strlen(name);
    adv_data[adv_len++] = name_len + 1;  // Length
    adv_data[adv_len++] = 0x09;          // Complete local name type
    memcpy(&adv_data[adv_len], name, name_len);
    adv_len += name_len;
    
    // Add manufacturer data (truncate if needed)
    uint8_t max_mfg_data = 31 - adv_len - 2; // 2 bytes for length+type
    uint8_t actual_mfg_len = (message_length > max_mfg_data) ? max_mfg_data : message_length;
    
    if (actual_mfg_len > 0) {
        adv_data[adv_len++] = actual_mfg_len + 1;  // Length
        adv_data[adv_len++] = 0xFF;                // Manufacturer data type
        memcpy(&adv_data[adv_len], buffer, actual_mfg_len);
        adv_len += actual_mfg_len;
    }
    
    // Set advertising data using extended advertising
    esp_err_t ret = esp_ble_gap_config_ext_adv_data_raw(EXT_ADV_HANDLE, adv_len, adv_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set extended advertising data: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Wait for data set complete
    xSemaphoreTake(ble_sem, portMAX_DELAY);
    
    ESP_LOGI(TAG, "Broadcasting LwM2M appearance message (adv_len: %d, mfg_len: %d)", adv_len, actual_mfg_len);
    return ESP_OK;
}

esp_err_t ble_lwm2m_broadcast_temperature(void) {
    if (!ble_initialized) {
        ESP_LOGE(TAG, "BLE not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    // Simulated temperature (25.0 - 35.0 C)
    float temperature = 25.0f + (esp_random() % 100) / 10.0f;

    // Build raw manufacturer payload: [serial(4)][temp_milli_c(2)][timestamp_ms(4)]
    uint8_t payload[10];
    uint32_t serial = g_factory_partition_valid ? (uint32_t)g_factory_partition.serial : 0;
    uint32_t timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    uint16_t temp_milli_c = (uint16_t)(temperature * 100); // centi-degrees *10 for extra precision
    memcpy(&payload[0], &serial, 4);
    memcpy(&payload[4], &temp_milli_c, 2);
    memcpy(&payload[6], &timestamp_ms, 4);

    uint8_t adv_data[31];
    uint8_t adv_len = 0;
    // Flags
    adv_data[adv_len++] = 2;      // Length
    adv_data[adv_len++] = 0x01;   // Flags type
    adv_data[adv_len++] = 0x06;   // General discoverable, BR/EDR not supported
    // Name
    const char *name = "LwM2M-Temp";
    uint8_t name_len = strlen(name);
    if (name_len + adv_len + 2 < 31) {
        adv_data[adv_len++] = name_len + 1;
        adv_data[adv_len++] = 0x09; // Complete local name
        memcpy(&adv_data[adv_len], name, name_len);
        adv_len += name_len;
    }
    // Manufacturer data
    if (adv_len + 2 + sizeof(payload) <= 31) {
        adv_data[adv_len++] = (uint8_t)(sizeof(payload) + 1);
        adv_data[adv_len++] = 0xFF; // Manufacturer specific
        memcpy(&adv_data[adv_len], payload, sizeof(payload));
        adv_len += sizeof(payload);
    } else {
        ESP_LOGW(TAG, "Not enough space for full temperature payload, skipping");
    }

    esp_err_t ret = esp_ble_gap_config_ext_adv_data_raw(EXT_ADV_HANDLE, adv_len, adv_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set extended advertising data: %s", esp_err_to_name(ret));
        return ret;
    }
    xSemaphoreTake(ble_sem, portMAX_DELAY);
    ESP_LOGI(TAG, "Broadcasting temperature: %.2f C (adv_len: %u)", temperature, adv_len);
    return ESP_OK;    
}

void ble_lwm2m_deinit(void) {
    if (!ble_initialized) {
        return;
    }
    
    // Stop periodic broadcasting
    ble_lwm2m_stop_periodic_broadcast();
    
    // Stop advertising
    stop_extended_advertising();
    
    // (No NimBLE deinit needed; using Bluedroid stack)
    
    // Disable and deinit Bluetooth controller
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    
    ble_initialized = false;
    ESP_LOGI(TAG, "BLE deinitialized");
}