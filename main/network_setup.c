
#include "network_setup.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "hbee-node.h"
#include "lora_handle.h"
#include <driver/gpio.h>
#include <esp_log.h>

#include <stdint.h>
#include <sx127x.h>
static const char *TAG = "NETWORK_SETUP";

QueueHandle_t rxQueue;
typedef struct {
    uint8_t *data;
    uint16_t data_length;
} rxMessage;

void rx_callback(void *ctx, uint8_t *data, uint16_t data_length) {
    // sx127x *device = (sx127x *)ctx;

    rxMessage message;
    message.data = malloc(data_length);
    if (message.data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for received message");
        return;
    }
    memcpy(message.data, data, data_length);
    message.data_length = data_length;

    if (xQueueSend(rxQueue, &message, pdMS_TO_TICKS(100)) != pdPASS) {
        ESP_LOGE(TAG, "Failed to send message to queue");
        free(message.data);
    }
}

void allocate_layer() {
#if BYPASS_GATEWAY
    ENLayer = ENId;
    ESP_LOGI(TAG, "Layer set as: %d", ENLayer);
    return;
#endif
    ESP_LOGI(TAG, "Starting network setup");
    rxQueue = xQueueCreate(1, sizeof(rxMessage));

    if (rxQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create queue");
        abort();
    }

    ESP_ERROR_CHECK(sx127x_set_frequency(NET_SETUP_FREQ, &lora_device));
    ESP_ERROR_CHECK(sx127x_lora_set_spreading_factor(NET_SETUP_SF, &lora_device));
    sx127x_rx_set_callback(rx_callback, &lora_device, &lora_device);
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_RX_CONT, SX127X_MODULATION_LORA, &lora_device));

    rxMessage received;
    while (true) {
        if (xQueueReceive(rxQueue, &received, portMAX_DELAY) == pdPASS) {
            if (received.data_length != 2) {
                ESP_LOGW(TAG, "Received message with unexpected length: %d", received.data_length);
                free(received.data);
                continue;
            }
            uint8_t cmdId = received.data[0];
            if (cmdId != 0x01) {
                ESP_LOGW(TAG, "Received message with unexpected cmdId: %d", cmdId);
                free(received.data);
                continue;
            }
            uint8_t layer = received.data[1];

            ENLayer = layer + 1; // Set the layer to one more than the received layer, as we are one step further from the gateway
            ESP_LOGI(TAG, "Layer set as: %d", ENLayer);

            free(received.data);
            break;
        }
    }
}

void broadcast_initializing_message() {
    uint8_t message[2];
    message[0] = 0x01;    // Command ID
    message[1] = ENLayer; // Layer
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_STANDBY, SX127X_MODULATION_LORA, &lora_device));
    ESP_ERROR_CHECK(sx127x_set_frequency(BROADCAST_LORA_FREQ, &lora_device));
    ESP_ERROR_CHECK(sx127x_lora_reset_fifo(&lora_device));
    ESP_ERROR_CHECK(sx127x_lora_set_spreading_factor(BROADCAST_LORA_SF, &lora_device));

    ESP_ERROR_CHECK(sx127x_tx_set_pa_config(SX127X_PA_PIN_BOOST, 20, &lora_device));

    ESP_ERROR_CHECK(sx127x_lora_tx_set_for_transmission(message, sizeof(message), &lora_device));
    ESP_LOGI(TAG, "Broadcasting initializing message");
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_TX, SX127X_MODULATION_LORA, &lora_device));
}