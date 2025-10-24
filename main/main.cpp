/*
   RadioLib Non-Arduino ESP-IDF Example

   This example shows how to use RadioLib without Arduino.
   In this case, a Heltec ESP32S3 LoRa V3.2 (ESP32S3 and SX1262)
   is used.

   Can be used as a starting point to port RadioLib to any platform!
   See this API reference page for details on the RadioLib hardware abstraction
   https://jgromes.github.io/RadioLib/class_hal.html

   For full API reference, see the GitHub Pages
   https://jgromes.github.io/RadioLib/
*/

// include the library
#include <RadioLib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"

// include the hardware abstraction layer
#include "EspHal.h"

// create a new instance of the HAL class for Heltec ESP32S3 LoRa V3.2
// Verified SPI pins for Heltec ESP32S3 LoRa V3.2: SCK=9, MISO=11, MOSI=10
EspHal* hal = new EspHal(9, 11, 10);

// now we can create the radio module for Heltec ESP32S3 LoRa V3.2 (SX1262)
// Verified pins for Heltec ESP32S3 LoRa V3.2:
// NSS pin:   8
// DIO1 pin:  14  
// NRST pin:  12
// BUSY pin:  13
SX1262 radio = new Module(hal, 8, 14, 12, 13);

static const char *TAG = "main";

// the entry point for the program
// it must be declared as "extern C" because the compiler assumes this will be a C function
extern "C" void app_main(void) {
  ESP_LOGI(TAG, "Starting LoRa initialization...");
  
  // Delete the main task from watchdog monitoring to prevent crashes
  esp_task_wdt_deinit();
  
  // Initialize the HAL first
  ESP_LOGI(TAG, "Initializing HAL...");
  hal->init();
  
  // Add a delay to let the system stabilize
  hal->delay(100);
  
  // initialize just like with Arduino
  ESP_LOGI(TAG, "[SX1262] Initializing ... ");
  
  // Add some debug output
  ESP_LOGI(TAG, "SPI pins - SCK: 9, MISO: 11, MOSI: 10");
  ESP_LOGI(TAG, "LoRa pins - NSS: 8, DIO1: 14, RST: 12, BUSY: 13");
  
  int state = radio.begin();
  if (state != RADIOLIB_ERR_NONE) {
    ESP_LOGE(TAG, "SX1262 initialization failed, code %d", state);
    while(true) {
      hal->delay(1000);
    }
  }
  ESP_LOGI(TAG, "SX1262 initialization success!");

  // loop forever
  for(;;) {
    // receive a packet
    ESP_LOGI(TAG, "[SX1262] Waiting for incoming packet ... ");
    
    // receive data as byte array with very short timeout to prevent watchdog
    uint8_t byteArr[256];
    state = radio.receive(byteArr, 256, 100000); // 100ms = 100,000 microseconds
    
    if(state == RADIOLIB_ERR_NONE) {
      // packet was successfully received
      ESP_LOGI(TAG, "Received packet!");
      
      // get the length of received data
      size_t length = radio.getPacketLength();
      ESP_LOGI(TAG, "Data length: %d bytes", length);
      
      // null terminate the received data to print as string
      if(length < 256) {
        byteArr[length] = '\0';
        ESP_LOGI(TAG, "Data: %s", (char*)byteArr);
      }
      
      // print RSSI (Received Signal Strength Indicator)
      ESP_LOGI(TAG, "RSSI: %.2f dBm", radio.getRSSI());
      
      // print SNR (Signal-to-Noise Ratio)  
      ESP_LOGI(TAG, "SNR: %.2f dB", radio.getSNR());
      
    } else if(state == RADIOLIB_ERR_RX_TIMEOUT) {
      // timeout occurred while waiting for a packet - this is normal
      // Don't log timeout to reduce spam, just continue listening
      
    } else {
      // some other error occurred
      ESP_LOGI(TAG, "failed, code %d", state);
    }

    // yield to other tasks to prevent watchdog issues
    vTaskDelay(pdMS_TO_TICKS(10)); // 10ms delay using FreeRTOS

  }

}