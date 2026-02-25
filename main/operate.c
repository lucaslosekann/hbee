#include "operate.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "hbee-node.h"
#include "lora_handle.h"
#include <esp_log.h>
#include <esp_timer.h>
#include <stdlib.h>
#include <sx127x.h>
static const char *TAG = "OPERATE";

parentNode *parents[100] = {NULL};

int packets_sent = 0;
float lastVoltage = 0.0;

static SemaphoreHandle_t send_data_mutex = NULL;

typedef struct {
    float voltage;
    uint8_t *data;
    uint16_t data_length;
} tx_queue_item_t;

static QueueHandle_t tx_queue = NULL;

// listen on PuC

void update_parent(uint16_t sender_id, uint8_t layer, float voltage) {
    int parent_idx = -1;
    int empty_idx = -1;
    for (int i = 0; i < 100; i++) {
        if (parents[i] != NULL && parents[i]->id == sender_id) {
            parent_idx = i;
            break;
        }
        if (parents[i] == NULL && empty_idx == -1) {
            empty_idx = i; // First empty slot
        }
    }

    if (parent_idx != -1) {
        parents[parent_idx]->lastVoltage = voltage;
        parents[parent_idx]->status = ALIVE;
        parents[parent_idx]->lastHeard = esp_timer_get_time();
    } else if (empty_idx != -1) {
        parents[empty_idx] = malloc(sizeof(parentNode));
        if (parents[empty_idx] != NULL) {
            parents[empty_idx]->id = sender_id;
            parents[empty_idx]->layer = layer;
            parents[empty_idx]->lastVoltage = voltage;
            parents[empty_idx]->status = ALIVE;
            parents[empty_idx]->lastHeard = esp_timer_get_time();
        }
    }
}

static void rx_callback(void *ctx, uint8_t *data, uint16_t data_length) {
    // sx127x *device = (sx127x *)ctx;

    uint8_t cmdId = data[0];
    if (cmdId != CMD_ID_DATA && cmdId != CMD_ID_CHANNEL) {
        ESP_LOGW(TAG, "Received message with unexpected command ID: %d", cmdId);
        return;
    }

    if (cmdId == CMD_ID_CHANNEL) {
        uint8_t layer = data[1];
        uint16_t sender_id = (data[2] << 8) | data[3];
        uint16_t receiver_id = (data[4] << 8) | data[5];
        uint8_t channel_idx = data[6];
        float voltage;
        memcpy(&voltage, data + 7, sizeof(float));
        if (layer < ENLayer) {
            update_parent(sender_id, layer, voltage);
        }

        if (receiver_id != ENId) {
            ESP_LOGW(TAG, "Received channel switch command intended for node 0x%04x, but this node's ID is 0x%04x. Ignoring.", receiver_id, ENId);
            return;
        }
        uint64_t new_channel = get_channel_by_id(channel_idx);
        ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_STANDBY, SX127X_MODULATION_LORA, &lora_device));
        ESP_ERROR_CHECK(sx127x_set_frequency(new_channel, &lora_device));
        ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_RX_CONT, SX127X_MODULATION_LORA, &lora_device));
        ESP_LOGI(TAG, "Switched to new channel %lld based on command from child", new_channel);
        return;
    }

    uint8_t layer = data[1];
    uint16_t sender_id = (data[2] << 8) | data[3];
    // uint16_t receiver_id = (data[4] << 8) | data[5];
    float voltage;
    memcpy(&voltage, data + 8, sizeof(float));

    ESP_LOGI(TAG, "Received data from node 0x%04x (layer %d) with voltage %.2f V", sender_id, layer, voltage);

    // Update parent info
    if (layer < ENLayer) {
        update_parent(sender_id, layer, voltage);
    } else if (layer > ENLayer) {
        // Redirect to gateway
        // Only forward the actual payload, not metadata
        uint16_t payload_offset = 8 + sizeof(float);
        if (data_length > payload_offset) {
            ESP_LOGI(TAG, "Forwarding data from node 0x%04x to parent with voltage %.2f V", sender_id, voltage);
            send_data(voltage, data + payload_offset, data_length - payload_offset);
            return;
        } else {
            ESP_LOGW(TAG, "No payload to forward after voltage extraction");
        }
    }

    // Received message on chosen parent channel, now back to listen on broadcast channel
    ESP_ERROR_CHECK(sx127x_set_frequency(BROADCAST_LORA_FREQ, &lora_device));
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_RX_CONT, SX127X_MODULATION_LORA, &lora_device));
}

void setup_message_listening() {
    ESP_ERROR_CHECK(sx127x_set_frequency(BROADCAST_LORA_FREQ, &lora_device));
    ESP_ERROR_CHECK(sx127x_lora_set_spreading_factor(BROADCAST_LORA_SF, &lora_device));
    sx127x_rx_set_callback(rx_callback, &lora_device, &lora_device);
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_RX_CONT, SX127X_MODULATION_LORA, &lora_device));
}

void init_send_data_mutex() { send_data_mutex = xSemaphoreCreateMutex(); }

// Implement function to get the parent with most voltage
parentNode *get_parent() { return parents[0]; }

uint64_t get_channel_by_id(uint16_t id) {
    if (id == ENId && packets_sent <= 5) {
        return BROADCAST_LORA_FREQ;
    }
    int idx = (id % 7);
    switch (idx) {
    case 0:
        return CH1_LORA_FREQ;
    case 1:
        return CH2_LORA_FREQ;
    case 2:
        return CH3_LORA_FREQ;
    case 3:
        return CH4_LORA_FREQ;
    case 4:
        return CH5_LORA_FREQ;
    case 5:
        return CH6_LORA_FREQ;
    case 6:
        return CH7_LORA_FREQ;
    default:
        return BROADCAST_LORA_FREQ;
    }
}

void tx_callback(void *ctx) {
    sx127x *device = (sx127x *)ctx;
    packets_sent++;
    // ESP_LOGI(TAG, "Transmission complete, total packets sent: %d", packets_sent);

    ESP_ERROR_CHECK(sx127x_set_frequency(BROADCAST_LORA_FREQ, device));
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_RX_CONT, SX127X_MODULATION_LORA, device));
}

uint64_t setup_dynamic_channel(int parent_id) {
    if (parent_id == 0xffff && ENLayer != 1) {
        return BROADCAST_LORA_FREQ; // If no parent, just use broadcast frequency
    }
    uint64_t random_channel;
    uint8_t channel_idx;
    while (true) {
        channel_idx = rand() % 7;
        random_channel = get_channel_by_id(channel_idx);
        ESP_LOGI(TAG, "Checking channel %lld for CAD", random_channel);
        bool cad_result = detect_cad_blocking(random_channel);
        ESP_LOGI(TAG, "CAD check on channel %lld returned %s", random_channel, cad_result ? "busy" : "clear");
        if (!cad_result) break;
        vTaskDelay(pdMS_TO_TICKS(50)); // Short delay before trying next channel
        // Verify CAD
        // if cad detects no traffic, break and use this channel
        // if cad detects traffic, continue to next random channel
    }

    ESP_ERROR_CHECK(sx127x_set_frequency(BROADCAST_LORA_FREQ, &lora_device));
    uint8_t *data_to_send = malloc(10); // 6 bytes for command ID, parent ID, Node ID, channel index
    if (data_to_send == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for channel switch message");
        return BROADCAST_LORA_FREQ; // Fallback to broadcast frequency if memory allocation fails
    }
    data_to_send[0] = CMD_ID_CHANNEL;          // Command ID for channel
    data_to_send[1] = ENLayer;                 // Current layer
    data_to_send[2] = (ENId >> 8) & 0xFF;      // High byte of ID
    data_to_send[3] = ENId & 0xFF;             // Low
    data_to_send[4] = (parent_id >> 8) & 0xFF; // High byte of receiver ID
    data_to_send[5] = parent_id & 0xFF;        // Low byte of receiver ID
    data_to_send[6] = channel_idx;             // Channel index to switch to

    // Add last voltage
    memcpy(data_to_send + 7, &lastVoltage, sizeof(float));
    ESP_ERROR_CHECK(sx127x_lora_tx_set_for_transmission(data_to_send, 10, &lora_device));
    free(data_to_send);

    ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_TX, SX127X_MODULATION_LORA, &lora_device));
    vTaskDelay(pdMS_TO_TICKS(200));
    return random_channel;
    // Send channel to parent using broadcast frequency
    // Wait for ACK from parent confirming channel switch or wait 2 seconds
    // return the channel
}

void send_data(float voltage, uint8_t *data, uint16_t data_length) {
    if (tx_queue == NULL) {
        tx_queue = xQueueCreate(8, sizeof(tx_queue_item_t));
    }
    tx_queue_item_t item;
    item.voltage = voltage;
    item.data_length = data_length;
    item.data = malloc(data_length);
    if (item.data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for tx_queue_item data");
        return;
    }
    memcpy(item.data, data, data_length);
    if (xQueueSend(tx_queue, &item, pdMS_TO_TICKS(10)) != pdPASS) {
        ESP_LOGW(TAG, "TX queue full, dropping packet");
        free(item.data);
    }
}
void tx_queue_consumer_task(void *pvParameters) {
    tx_queue_item_t item;
    for (;;) {
        if (xQueueReceive(tx_queue, &item, portMAX_DELAY) == pdPASS) {
            if (xSemaphoreTake(send_data_mutex, portMAX_DELAY)) {
                lastVoltage = item.voltage;
                parentNode *parent = get_parent();
                int parent_id = 0xffff;
                if (parent != NULL) {
                    parent_id = parent->id;
                }
                uint64_t channel = setup_dynamic_channel(parent_id);

                int payload_length = sizeof(float) + item.data_length + 8;
                uint8_t *data_to_send = malloc(payload_length);
                if (data_to_send == NULL) {
                    ESP_LOGE(TAG, "Failed to allocate memory for transmission data");
                    xSemaphoreGive(send_data_mutex);
                    free(item.data);
                    continue;
                }

                data_to_send[0] = CMD_ID_DATA;
                data_to_send[1] = ENLayer;
                data_to_send[2] = (ENId >> 8) & 0xFF;
                data_to_send[3] = ENId & 0xFF;
                data_to_send[4] = (parent_id >> 8) & 0xFF;
                data_to_send[5] = parent_id & 0xFF;
                data_to_send[6] = (item.data_length >> 8) & 0xFF;
                data_to_send[7] = item.data_length & 0xFF;
                memcpy(data_to_send + 8, &item.voltage, sizeof(float));
                memcpy(data_to_send + 8 + sizeof(float), item.data, item.data_length);

                ESP_ERROR_CHECK(sx127x_set_frequency(channel, &lora_device));
                sx127x_tx_set_callback(tx_callback, &lora_device, &lora_device);
                ESP_ERROR_CHECK(sx127x_lora_tx_set_for_transmission(data_to_send, payload_length, &lora_device));
                free(data_to_send);

                ESP_LOGI(TAG, "Transmitting data to parent 0x%04x on channel %ld", parent_id, channel);
                ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_TX, SX127X_MODULATION_LORA, &lora_device));
                xSemaphoreGive(send_data_mutex);
                free(item.data);
            }
        }
    }
}

// Call this during initialization to start the consumer task
void start_tx_queue_consumer() {
    if (tx_queue == NULL) {
        tx_queue = xQueueCreate(8, sizeof(tx_queue_item_t));
    }
    xTaskCreate(tx_queue_consumer_task, "tx_queue_consumer", 4096, NULL, 10, NULL);
}