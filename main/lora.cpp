/*
   LoRa Module for ESP32 LwM2M Gateway
   
   This module handles LoRa communication using RadioLib.
   Hardware configuration for Heltec ESP32S3 LoRa V3.2
   
   Features:
   - Listen before send pattern for collision avoidance
   - Automatic message reception with callback support
   - Channel activity detection
   - Signal strength and quality reporting
*/

// include the library
#include <RadioLib.h>

// include the hardware abstraction layer
#include "EspHal.h"
#include "lora.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_task_wdt.h"
#include "string.h"

static const char *TAG = "lora";

// Default configuration - using very aggressive settings to prevent watchdog timeout
#define LORA_RECEIVE_TIMEOUT_MS     1      // Extremely short timeout for listening (1ms to prevent blocking)
#define LORA_DEFAULT_LISTEN_TIME_MS 500    // Default time to listen before sending
#define LORA_MAX_PACKET_SIZE        256    // Maximum packet size
#define LORA_SEND_QUEUE_SIZE        10     // Queue size for messages to send
#define LORA_WATCHDOG_YIELD_MS      1      // Very frequent yielding every 1ms

// Message structure for send queue
typedef struct {
    char message[LORA_MAX_PACKET_SIZE];
    uint32_t listen_timeout_ms;
} lora_send_msg_t;

// create a new instance of the HAL class for Heltec ESP32S3 LoRa V3.2
// Verified SPI pins for Heltec ESP32S3 LoRa V3.2: SCK=9, MISO=11, MOSI=10
static EspHal* hal = nullptr;

// now we can create the radio module for Heltec ESP32S3 LoRa V3.2 (SX1262)
// Verified pins for Heltec ESP32S3 LoRa V3.2:
// NSS pin:   8
// DIO1 pin:  14  
// NRST pin:  12
// BUSY pin:  13
static SX1262* radio = nullptr;

// Task handle for LoRa communication task
static TaskHandle_t lora_task_handle = nullptr;

// Queue for messages to send
static QueueHandle_t send_queue = nullptr;

// Callback function for received messages
static lora_receive_callback_t receive_callback = nullptr;

// Helper function to listen for incoming packets
static bool listen_for_packets(uint32_t timeout_ms) {
    uint8_t byteArr[LORA_MAX_PACKET_SIZE];
    uint32_t listen_start = hal->millis();
    bool packet_received = false;
    
    ESP_LOGI(TAG, "[SX1262] Listening for %lu ms before sending...", timeout_ms);
    
    while ((hal->millis() - listen_start) < timeout_ms) {
        // Listen for incoming packets with very short timeout to prevent watchdog issues
        int state = radio->receive(byteArr, LORA_MAX_PACKET_SIZE, LORA_RECEIVE_TIMEOUT_MS * 1000); // Convert to microseconds
        
        if (state == RADIOLIB_ERR_NONE) {
            // Packet received successfully
            packet_received = true;
            size_t length = radio->getPacketLength();
            float rssi = radio->getRSSI();
            float snr = radio->getSNR();
            
            ESP_LOGI(TAG, "📡 Received packet during listen phase!");
            ESP_LOGI(TAG, "   Data length: %d bytes", length);
            
            // Print received data as string if printable
            if (length < LORA_MAX_PACKET_SIZE) {
                byteArr[length] = '\0';
                ESP_LOGI(TAG, "   Message: %s", (char*)byteArr);
            }
            
            ESP_LOGI(TAG, "   RSSI: %.2f dBm", rssi);
            ESP_LOGI(TAG, "   SNR: %.2f dB", snr);
            
            // Call user callback if provided
            if (receive_callback != nullptr) {
                receive_callback(byteArr, length, rssi, snr);
            }
            
            // Channel is busy, wait longer before sending
            ESP_LOGI(TAG, "   Channel busy, extending listen time...");
            vTaskDelay(pdMS_TO_TICKS(200)); // Additional delay when channel is busy
            
        } else if (state != RADIOLIB_ERR_RX_TIMEOUT) {
            // Some other error occurred (not just timeout)
            ESP_LOGW(TAG, "   Listen error: %d", state);
        }
        
        // Yield to other tasks more frequently to prevent watchdog issues
        vTaskDelay(pdMS_TO_TICKS(LORA_WATCHDOG_YIELD_MS));
    }
    
    if (!packet_received) {
        ESP_LOGI(TAG, "   Channel appears clear, ready to send");
    }
    
    return packet_received; // Return true if channel was busy
}

// LoRa communication task with listen-before-send pattern
static void lora_task(void *pvParameters) {
    ESP_LOGI(TAG, "🚀 LoRa communication task started with listen-before-send pattern");
    ESP_LOGI(TAG, "Using frequent task yielding to prevent watchdog timeouts");
    
    lora_send_msg_t send_msg;
    
    // Main communication loop
    for(;;) {
        // Check if there's a message to send in the queue
        if (xQueueReceive(send_queue, &send_msg, pdMS_TO_TICKS(100)) == pdTRUE) {
            ESP_LOGI(TAG, "📤 Processing message from queue: %s", send_msg.message);
            
            // Listen before sending (collision avoidance)
            bool channel_busy = listen_for_packets(send_msg.listen_timeout_ms);
            
            // Send the message (even if channel was busy, after waiting)
            ESP_LOGI(TAG, "[SX1262] Transmitting message: %s", send_msg.message);
            int state = radio->transmit(send_msg.message);
            
            if (state == RADIOLIB_ERR_NONE) {
                ESP_LOGI(TAG, "✅ Message transmitted successfully!");
            } else {
                ESP_LOGE(TAG, "❌ Transmission failed, code %d", state);
            }
            
        } else {
            // No message to send, just listen for incoming packets
            uint8_t byteArr[LORA_MAX_PACKET_SIZE];
            int state = radio->receive(byteArr, LORA_MAX_PACKET_SIZE, LORA_RECEIVE_TIMEOUT_MS * 1000);
            
            if (state == RADIOLIB_ERR_NONE) {
                // Packet received successfully
                size_t length = radio->getPacketLength();
                float rssi = radio->getRSSI();
                float snr = radio->getSNR();
                
                ESP_LOGI(TAG, "📡 Received packet!");
                ESP_LOGI(TAG, "   Data length: %d bytes", length);
                
                // Print received data as string if printable
                if (length < LORA_MAX_PACKET_SIZE) {
                    byteArr[length] = '\0';
                    ESP_LOGI(TAG, "   Message: %s", (char*)byteArr);
                }
                
                ESP_LOGI(TAG, "   RSSI: %.2f dBm", rssi);
                ESP_LOGI(TAG, "   SNR: %.2f dB", snr);
                
                // Call user callback if provided
                if (receive_callback != nullptr) {
                    receive_callback(byteArr, length, rssi, snr);
                }
                
            } else if (state != RADIOLIB_ERR_RX_TIMEOUT) {
                // Some error other than timeout
                ESP_LOGD(TAG, "Receive error: %d", state); // Use debug level to reduce spam
            }
        }
        
        // Yield to other tasks to prevent any remaining watchdog issues
        // Use longer yield to ensure IDLE task can run and reset watchdog
        vTaskDelay(pdMS_TO_TICKS(10)); // 10ms yield to allow system tasks to run
    }
}

extern "C" esp_err_t lora_init(void) {
    ESP_LOGI(TAG, "Starting LoRa initialization...");
    
    // Create HAL instance
    hal = new EspHal(9, 11, 10);
    if (hal == nullptr) {
        ESP_LOGE(TAG, "Failed to create HAL instance");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize the HAL first
    ESP_LOGI(TAG, "Initializing HAL...");
    hal->init();
    
    // Add a delay to let the system stabilize
    hal->delay(100);
    
    // Create radio instance
    radio = new SX1262(new Module(hal, 8, 14, 12, 13));
    if (radio == nullptr) {
        ESP_LOGE(TAG, "Failed to create radio instance");
        delete hal;
        hal = nullptr;
        return ESP_ERR_NO_MEM;
    }
    
    // initialize just like with Arduino
    ESP_LOGI(TAG, "[SX1262] Initializing ... ");
    
    // Add some debug output
    ESP_LOGI(TAG, "SPI pins - SCK: 9, MISO: 11, MOSI: 10");
    ESP_LOGI(TAG, "LoRa pins - NSS: 8, DIO1: 14, RST: 12, BUSY: 13");
    
    int state = radio->begin();
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "SX1262 initialization failed, code %d", state);
        delete radio;
        delete hal;
        radio = nullptr;
        hal = nullptr;
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "SX1262 initialization success!");
    
    return ESP_OK;
}

extern "C" esp_err_t lora_start_task(lora_receive_callback_t callback) {
    if (radio == nullptr || hal == nullptr) {
        ESP_LOGE(TAG, "LoRa not initialized. Call lora_init() first.");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Store the receive callback
    receive_callback = callback;
    
    // Create message queue for send requests
    send_queue = xQueueCreate(LORA_SEND_QUEUE_SIZE, sizeof(lora_send_msg_t));
    if (send_queue == nullptr) {
        ESP_LOGE(TAG, "Failed to create send queue");
        return ESP_ERR_NO_MEM;
    }
    
    // Create the LoRa communication task with lower priority to prevent watchdog issues
    BaseType_t xReturned = xTaskCreate(
        lora_task,              // Function that implements the task
        "lora_task",            // Text name for the task
        8192,                   // Increased stack size for more complex operations
        NULL,                   // Parameter passed into the task
        1,                      // Lower priority (was 5) to allow IDLE and other tasks to run
        &lora_task_handle       // Used to pass back a handle by which the created task can be referenced
    );
    
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create LoRa task");
        vQueueDelete(send_queue);
        send_queue = nullptr;
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "LoRa task started successfully with listen-before-send pattern");
    return ESP_OK;
}

extern "C" esp_err_t lora_send_message(const char* message) {
    return lora_send_message_with_timeout(message, LORA_DEFAULT_LISTEN_TIME_MS);
}

extern "C" esp_err_t lora_send_message_with_timeout(const char* message, uint32_t listen_timeout_ms) {
    if (send_queue == nullptr) {
        ESP_LOGE(TAG, "LoRa task not started. Call lora_start_task() first.");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (message == nullptr) {
        ESP_LOGE(TAG, "Message is null");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (strlen(message) >= LORA_MAX_PACKET_SIZE) {
        ESP_LOGE(TAG, "Message too long (max %d chars)", LORA_MAX_PACKET_SIZE - 1);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Prepare message for queue
    lora_send_msg_t send_msg;
    strncpy(send_msg.message, message, sizeof(send_msg.message) - 1);
    send_msg.message[sizeof(send_msg.message) - 1] = '\0'; // Ensure null termination
    send_msg.listen_timeout_ms = listen_timeout_ms;
    
    // Add message to send queue
    if (xQueueSend(send_queue, &send_msg, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to queue message for sending (queue full?)");
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG, "📝 Message queued for transmission: %s", message);
    return ESP_OK;
}

extern "C" bool lora_is_channel_busy(void) {
    if (radio == nullptr || hal == nullptr) {
        ESP_LOGE(TAG, "LoRa not initialized");
        return false;
    }
    
    // Try to receive with very short timeout to check channel activity
    uint8_t dummy_buffer[32];
    int state = radio->receive(dummy_buffer, sizeof(dummy_buffer), 5000); // 5ms timeout (reduced)
    
    // If we received something or if there's an active signal, channel is busy
    return (state == RADIOLIB_ERR_NONE);
}

extern "C" void lora_stop_task(void) {
    if (lora_task_handle != nullptr) {
        vTaskDelete(lora_task_handle);
        lora_task_handle = nullptr;
        ESP_LOGI(TAG, "LoRa task stopped");
    }
    
    // Cleanup queue
    if (send_queue != nullptr) {
        vQueueDelete(send_queue);
        send_queue = nullptr;
        ESP_LOGI(TAG, "Send queue deleted");
    }
    
    // Clear callback
    receive_callback = nullptr;
    
    // Cleanup radio resources
    if (radio != nullptr) {
        delete radio;
        radio = nullptr;
    }
    
    if (hal != nullptr) {
        hal->term();
        delete hal;
        hal = nullptr;
    }
    
    ESP_LOGI(TAG, "LoRa module cleaned up");
}