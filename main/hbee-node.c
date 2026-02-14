#include "hbee-node.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led.h"
#include "logs.h"
#include "lora_handle.h"
#include "network_setup.h"
#include "nvs_flash.h"
#include "operate.h"
#include "sensing.h"
#include <i2cdev.h>

static const char *TAG = "HBEE MAIN";

uint8_t ENLayer = 0xff;
uint16_t ENId;

void app_main(void) {
    config_led();
    // Set color to Red
    set_led_color(255, 0, 0);

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // save_id(); // Save the device ID

    ENId = read_id();

    ESP_LOGI(TAG, "Device ID: %d", ENId);
    set_led_color(0, 0, 255);

    // Setup functions for MQTT logging (dependent of REMOTE_LOGS_ENABLED constant)
    setup_logs();
    set_led_color(255, 255, 255);

    setup_lora_device();

    set_led_color(0, 255, 0);
    // STAGES
    // Wait for message from parent so we can know in which layer we are
    allocate_layer();

    // Forward message with our layer so the other nodes can do the same
    broadcast_initializing_message();

    // Initialize message listening thread on public channel
    // Listen for retransmission and to store its parents
    setup_message_listening();

    // Initialize sensor reading/transmitting thread
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(sensing_task, "sensing_task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}
