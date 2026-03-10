#include "operate.h"
#include "esp_err.h"
#include "esp_random.h"
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
static QueueHandle_t parent_confirmation_queue = NULL;

static esp_timer_handle_t channel_switch_timeout_timer;

static bool hasReceivedData = false;

parentNode *get_parent_by_id(uint16_t id) {
    for (int i = 0; i < 100; i++) {
        if (parents[i] != NULL && parents[i]->id == id) {
            return parents[i];
        }
    }
    return NULL;
}

void channel_switch_timeout_callback(void *arg) {
    ESP_LOGW(TAG, "Channel switch timeout occurred, switching back to broadcast channel");
    ESP_ERROR_CHECK(sx127x_set_frequency(BROADCAST_LORA_FREQ, &lora_device));
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_RX_CONT, SX127X_MODULATION_LORA, &lora_device));
}

uint64_t get_channel_by_id(uint16_t id) {
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
uint64_t setup_dynamic_channel(int parent_id) {
    uint64_t random_channel;
    uint8_t channel_idx;
    if (parent_id == 0xffff && ENLayer != 1) {
        return BROADCAST_LORA_FREQ; // If no parent, just use broadcast frequency
    }

    while (true) {
        channel_idx = esp_random() % 7;
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
    uint8_t *data_to_send = malloc(11); // 6 bytes for command ID, parent ID, Node ID, channel index
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
    data_to_send[6] = channel_idx;             // Channel to switch to

    // Add last voltage
    memcpy(data_to_send + 7, &lastVoltage, sizeof(float));
    ESP_ERROR_CHECK(sx127x_lora_tx_set_for_transmission(data_to_send, 11, &lora_device));
    free(data_to_send);

    ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_TX, SX127X_MODULATION_LORA, &lora_device));
    vTaskDelay(pdMS_TO_TICKS(200));
    return random_channel;
    // Send channel to parent using broadcast frequency
    // Wait for ACK from parent confirming channel switch or wait 2 seconds
    // return the channel
}

static void parent_confirm_callback(void *arg) {
    parentNode *parent = (parentNode *)arg;
    // send confirmation request to parent
    // if parent responds, mark as alive and update EWMA
    // if parent does not respond within timeout, mark as dead

    if (xSemaphoreTake(send_data_mutex, portMAX_DELAY)) {
        uint8_t *data_to_send = malloc(6); // 2 bytes for command ID, 4 bytes for parent ID
        if (data_to_send == NULL) {
            ESP_LOGE(TAG, "Failed to allocate memory for confirmation request");
            xSemaphoreGive(send_data_mutex);
            return;
        }
        data_to_send[0] = CMD_ID_CONFIRM_REQUEST;   // Command ID for confirmation
        data_to_send[1] = ENLayer;                  // Current layer
        data_to_send[2] = (ENId >> 8) & 0xFF;       // High byte of ID
        data_to_send[3] = ENId & 0xFF;              // Low byte of ID
        data_to_send[4] = (parent->id >> 8) & 0xFF; // High byte of receiver ID
        data_to_send[5] = parent->id & 0xFF;        // Low byte of receiver ID

        ESP_ERROR_CHECK(sx127x_set_frequency(setup_dynamic_channel(parent->id), &lora_device));
        ESP_ERROR_CHECK(sx127x_lora_tx_set_for_transmission(data_to_send, 6, &lora_device));
        ESP_LOGI(TAG, "Sending confirmation request to parent 0x%04x", parent->id);
        ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_TX, SX127X_MODULATION_LORA, &lora_device));
        xSemaphoreGive(send_data_mutex);
        free(data_to_send);

        // If does not receive confirmation response within timeout, mark as dead
        vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for 2 seconds for response
        if (parent->status == LOCALLY_DEAD) {
            parent->status = DEAD;
            ESP_LOGW(TAG, "Parent 0x%04x marked as dead after confirmation timeout", parent->id);
        }
    }
}

static void parent_confirmation_queue_consumer_task() {
    // send confirmation request to parent
    parentNode *parent;
    while (true) {
        if (xQueueReceive(parent_confirmation_queue, &parent, pdMS_TO_TICKS(10))) {
            // get semaphore to send confirmation request
            if (xSemaphoreTake(send_data_mutex, portMAX_DELAY)) {
                uint8_t *data_to_send = malloc(6); // 2 bytes for command ID, 4 bytes for parent ID
                if (data_to_send == NULL) {
                    ESP_LOGE(TAG, "Failed to allocate memory for confirmation request");
                    xSemaphoreGive(send_data_mutex);
                    return;
                }
                data_to_send[0] = CMD_ID_CONFIRM_REQUEST;   // Command ID for confirmation
                data_to_send[1] = ENLayer;                  // Current layer
                data_to_send[2] = (ENId >> 8) & 0xFF;       // High byte of ID
                data_to_send[3] = ENId & 0xFF;              // Low byte of ID
                data_to_send[4] = (parent->id >> 8) & 0xFF; // High byte of receiver ID
                data_to_send[5] = parent->id & 0xFF;        // Low byte of receiver ID

                ESP_ERROR_CHECK(sx127x_set_frequency(setup_dynamic_channel(parent->id), &lora_device));
                ESP_ERROR_CHECK(sx127x_lora_tx_set_for_transmission(data_to_send, 6, &lora_device));
                ESP_LOGI(TAG, "Sending confirmation request to parent 0x%04x", parent->id);
                ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_TX, SX127X_MODULATION_LORA, &lora_device));
                xSemaphoreGive(send_data_mutex);
                free(data_to_send);

                // If does not receive confirmation response within timeout, mark as dead
                vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for 2 seconds for response
                if (parent->status == SUSPECTED) {
                    parent->status = LOCALLY_DEAD;
                    ESP_LOGW(TAG, "Parent 0x%04x marked as locally dead after confirmation timeout", parent->id);
#if USE_EWMA_FOR_FAILURE_DETECTION
                    esp_timer_start_once(parent->confirmTimer, EWMA_K * EWMA_CONFIRMATION_CONSTANT * parent->ewmaInterval);
#else
                    esp_timer_start_once(parent->confirmTimer, EWMA_CONFIRMATION_CONSTANT * FAILURE_TIMEOUT);
#endif
                }
            }
        }
    }
}

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
        esp_timer_stop(parents[parent_idx]->confirmTimer);
        parents[parent_idx]->lastVoltage = voltage;
        parents[parent_idx]->status = ALIVE;
        parents[parent_idx]->silence = esp_timer_get_time() - parents[parent_idx]->lastHeard;
        parents[parent_idx]->lastHeard = esp_timer_get_time();
        if (!parents[parent_idx]->ewmaInitialized) {
            parents[parent_idx]->ewmaInterval = parents[parent_idx]->silence;
            parents[parent_idx]->ewmaInitialized = true;
        } else {
            ESP_LOGI(TAG, "Updating EWMA for parent 0x%04x: silence=%lld, previous EWMA=%f", sender_id, parents[parent_idx]->silence,
                     parents[parent_idx]->ewmaInterval);
            parents[parent_idx]->ewmaInterval = EWMA_ALPHA * parents[parent_idx]->silence + (1 - EWMA_ALPHA) * parents[parent_idx]->ewmaInterval;
        }

    } else if (empty_idx != -1) {
        parents[empty_idx] = malloc(sizeof(parentNode));
        if (parents[empty_idx] != NULL) {
            parents[empty_idx]->id = sender_id;
            parents[empty_idx]->layer = layer;
            parents[empty_idx]->lastVoltage = voltage;
            parents[empty_idx]->status = ALIVE;
            parents[empty_idx]->lastHeard = esp_timer_get_time();
            parents[empty_idx]->silence = 0;
            parents[empty_idx]->ewmaInterval = 0;
            parents[empty_idx]->ewmaInitialized = false;
            esp_timer_create_args_t args = {
                .callback = parent_confirm_callback, .arg = parents[empty_idx], .dispatch_method = ESP_TIMER_TASK, .name = "confirm_timer"};

            esp_timer_create(&args, &parents[empty_idx]->confirmTimer);
        }
    }
}

static void rx_callback(void *ctx, uint8_t *data, uint16_t data_length) {
    sx127x *device = (sx127x *)ctx;

    // get rssi
    float snr = 0.0f;
    int16_t rssi = 0;
    sx127x_lora_rx_get_packet_snr(device, &snr);
    sx127x_rx_get_packet_rssi(device, &rssi);

    ESP_LOGI(TAG, "Received packet: RSSI = %d dBm, SNR = %.2f dB", rssi, snr);

    uint8_t cmdId = data[0];
    if (cmdId != CMD_ID_DATA && cmdId != CMD_ID_CHANNEL && cmdId != CMD_ID_CONFIRM_REQUEST && cmdId != CMD_ID_CONFIRM_RESPONSE) {
        ESP_LOGW(TAG, "Received message with unexpected command ID: %d", cmdId);
        return;
    }

    if (cmdId == CMD_ID_CONFIRM_REQUEST) {
        uint16_t sender_id = (data[2] << 8) | data[3];
        uint16_t receiver_id = (data[4] << 8) | data[5];
        if (receiver_id != ENId) {
            ESP_LOGW(TAG, "Received confirmation request intended for node 0x%04x, but this node's ID is 0x%04x. Ignoring.", receiver_id, ENId);
            return;
        }
        // Send confirmation response
        uint8_t *response_data = malloc(6); // 2 bytes for command ID, 4 bytes for IDs
        if (response_data == NULL) {
            ESP_LOGE(TAG, "Failed to allocate memory for confirmation response");
            return;
        }
        response_data[0] = CMD_ID_CONFIRM_RESPONSE; // Command ID for confirmation response
        response_data[1] = ENLayer;                 // Current layer
        response_data[2] = (ENId >> 8) & 0xFF;      // High byte of ID
        response_data[3] = ENId & 0xFF;             // Low byte of ID
        response_data[4] = (sender_id >> 8) & 0xFF; // High byte of receiver ID
        response_data[5] = sender_id & 0xFF;        // Low byte of receiver ID

        ESP_ERROR_CHECK(sx127x_set_frequency(setup_dynamic_channel(sender_id), &lora_device));
        ESP_ERROR_CHECK(sx127x_lora_tx_set_for_transmission(response_data, 6, &lora_device));
        ESP_LOGI(TAG, "Sending confirmation response to parent 0x%04x", sender_id);
        ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_TX, SX127X_MODULATION_LORA, &lora_device));
        free(response_data);
        return;
    }

    if (cmdId == CMD_ID_CONFIRM_RESPONSE) {
        uint8_t layer = data[1];
        uint16_t sender_id = (data[2] << 8) | data[3];
        uint16_t receiver_id = (data[4] << 8) | data[5];
        if (receiver_id != ENId) {
            ESP_LOGW(TAG, "Received confirmation response intended for node 0x%04x, but this node's ID is 0x%04x. Ignoring.", receiver_id, ENId);
            return;
        }
        // Process confirmation response
        parentNode *parent = get_parent_by_id(sender_id);
        if (parent != NULL) {
            parent->status = ALIVE;
            parent->lastHeard = esp_timer_get_time();
            parent->silence = 0;
            parent->ewmaInterval = 0;
            parent->ewmaInitialized = false;
            ESP_LOGI(TAG, "Received confirmation response from parent 0x%04x, marking as alive", sender_id);
        } else {
            ESP_LOGW(TAG, "Received confirmation response from unknown parent 0x%04x", sender_id);
        }

        return;
    }

    if (cmdId == CMD_ID_CHANNEL) {
        xSemaphoreTake(send_data_mutex, pdMS_TO_TICKS(100)); // Take mutex to avoid conflicts with channel switching
        uint8_t layer = data[1];
        uint16_t sender_id = (data[2] << 8) | data[3];
        uint16_t receiver_id = (data[4] << 8) | data[5];
        uint8_t channel_idx = data[6];
        float voltage;
        memcpy(&voltage, data + 7, sizeof(float));
        if (layer + 1 == ENLayer) {
            update_parent(sender_id, layer, voltage);
        }

        if (receiver_id != ENId) {
            ESP_LOGW(TAG, "Received channel switch command intended for node 0x%04x, but this node's ID is 0x%04x. Ignoring.", receiver_id, ENId);
            xSemaphoreGive(send_data_mutex);
            return;
        }
        uint64_t new_channel = get_channel_by_id(channel_idx);
        ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_STANDBY, SX127X_MODULATION_LORA, &lora_device));
        ESP_ERROR_CHECK(sx127x_set_frequency(new_channel, &lora_device));
        ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_RX_CONT, SX127X_MODULATION_LORA, &lora_device));

        ESP_LOGI(TAG, "Switched to new channel %lld based on command from child", new_channel);
        xSemaphoreGive(send_data_mutex);
        esp_timer_start_once(channel_switch_timeout_timer, 1 * 1000000);

        return;
    }

    uint8_t layer = data[1];
    uint16_t sender_id = (data[2] << 8) | data[3];
    uint16_t receiver_id = (data[4] << 8) | data[5];

    float voltage;
    memcpy(&voltage, data + 8, sizeof(float));

    ESP_LOGI(TAG, "Received data from node 0x%04x (layer %d) with voltage %.2f V", sender_id, layer, voltage);

    // Update parent info
    if (layer + 1 == ENLayer) {
        update_parent(sender_id, layer, voltage);
    } else if (layer > ENLayer && (receiver_id == ENId || receiver_id == 0xffff)) {
        // Reset timer
        esp_timer_stop(channel_switch_timeout_timer);

        // Redirect to gateway
        // Only forward the actual payload, not metadata
        uint16_t payload_offset = 8 + sizeof(float);
        if (data_length > payload_offset) {
            ESP_LOGI(TAG, "Forwarding data from node 0x%04x to parent with voltage %.2f V", sender_id, voltage);
            int base_payload_length = data_length - payload_offset;

            int extra_size = sizeof(uint16_t)  // enid
                             + sizeof(int16_t) // rssi
                             + sizeof(float);  // snr
            // int extra_size = 0;

            int total_payload_length = base_payload_length + extra_size;

            uint8_t *payload_data = malloc(total_payload_length);
            if (payload_data == NULL) {
                ESP_LOGE(TAG, "Failed to allocate memory for payload");
                return;
            }

            // copia payload original
            memcpy(payload_data, data + payload_offset, base_payload_length);

            int offset = base_payload_length;

            memcpy(payload_data + offset, &ENId, sizeof(uint16_t));
            offset += sizeof(uint16_t);

            memcpy(payload_data + offset, &rssi, sizeof(int16_t));
            offset += sizeof(int16_t);

            memcpy(payload_data + offset, &snr, sizeof(float));
            offset += sizeof(float);

            send_data(voltage, payload_data, total_payload_length);

            free(payload_data);
        } else {
            ESP_LOGW(TAG, "No payload to forward after voltage extraction");
        }
    }

    // Received message on chosen parent channel, now back to listen on broadcast channel
    ESP_ERROR_CHECK(sx127x_set_frequency(BROADCAST_LORA_FREQ, &lora_device));
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_RX_CONT, SX127X_MODULATION_LORA, &lora_device));
    hasReceivedData = true;
}

void setup_message_listening() {
    ESP_ERROR_CHECK(sx127x_set_frequency(BROADCAST_LORA_FREQ, &lora_device));
    ESP_ERROR_CHECK(sx127x_lora_set_spreading_factor(BROADCAST_LORA_SF, &lora_device));
    sx127x_rx_set_callback(rx_callback, &lora_device, &lora_device);
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_RX_CONT, SX127X_MODULATION_LORA, &lora_device));

    esp_timer_create_args_t channel_switch_timeout_config = {
        .callback = channel_switch_timeout_callback, .arg = NULL, .dispatch_method = ESP_TIMER_TASK, .name = "channel_switch_timeout"};
    esp_timer_create(&channel_switch_timeout_config, &channel_switch_timeout_timer);
}

void init_send_data_mutex() {
    send_data_mutex = xSemaphoreCreateBinary();
    xSemaphoreGive(send_data_mutex); // Start with mutex available
}

// Implement function to get the parent with most voltage
parentNode *get_parent() {
    parentNode *best_parent = NULL;
    parentNode *alive_parents[100];
    int alive_count = 0;
    for (int i = 0; i < 100; i++) {
        if (parents[i] != NULL && parents[i]->status == ALIVE) {
// Check EWMA
#if NODE_FAILURE_DETECTION_ENABLED
            int64_t silence = esp_timer_get_time() - parents[i]->lastHeard;
#if USE_EWMA_FOR_FAILURE_DETECTION
            if (parents[i]->ewmaInitialized && silence > EWMA_K * parents[i]->ewmaInterval) {
                parents[i]->status = SUSPECTED;

                // xQueueSend(parent_confirmation_queue, &parents[i], pdMS_TO_TICKS(10)); // Send to confirmation queue

                ESP_LOGW(TAG, "Parent 0x%04x is suspected (silence: %lld, EWMA: %f)", parents[i]->id, silence, parents[i]->ewmaInterval);
                continue;
            }
#else
            if (silence > FAILURE_TIMEOUT) {
                parents[i]->status = SUSPECTED;

                // xQueueSend(parent_confirmation_queue, &parents[i], pdMS_TO_TICKS(10)); // Send to confirmation queue

                ESP_LOGW(TAG, "Parent 0x%04x is suspected (silence: %lld)", parents[i]->id, silence);
                continue;
            }
#endif
#endif
            // if (best_parent == NULL || parents[i]->lastVoltage > best_parent->lastVoltage) {
            //     best_parent = parents[i];
            // }
            alive_parents[alive_count++] = parents[i];
        }
    }

    // get random alive parent
    if (alive_count > 0) {
        int random_index = esp_random() % alive_count;
        return alive_parents[random_index];
    } else {
        ESP_LOGW(TAG, "No alive parents available");
        return NULL;
    }

    return best_parent;
}

void tx_callback(void *ctx) {
    sx127x *device = (sx127x *)ctx;
    packets_sent++;
    ESP_LOGI(TAG, "Transmission complete, total packets sent: %d", packets_sent);

    ESP_ERROR_CHECK(sx127x_set_frequency(BROADCAST_LORA_FREQ, device));
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_RX_CONT, SX127X_MODULATION_LORA, device));
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

void start_parent_confirmation_queue_consumer() {
    if (parent_confirmation_queue == NULL) {
        parent_confirmation_queue = xQueueCreate(8, sizeof(parentNode *));
    }
    xTaskCreate(parent_confirmation_queue_consumer_task, "parent_confirmation_queue_consumer", 4096, NULL, 10, NULL);
}