#include "operate.h"
#include "esp_err.h"
#include "hbee-node.h"
#include "lora_handle.h"
#include <esp_log.h>
#include <stdlib.h>
#include <sx127x.h>
static const char *TAG = "OPERATE";

parentNode *parents[100] = {NULL};

int packets_sent = 0;

// listen on PuC

static void rx_callback(void *ctx, uint8_t *data, uint16_t data_length) {
    // sx127x *device = (sx127x *)ctx;

    uint8_t cmdId = data[0];
    if (cmdId != 0x02) {
        ESP_LOGW(TAG, "Received message with unexpected command ID: %d", cmdId);
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
    } else {
        // Redirect to gateway
        // Only forward the actual payload, not metadata
        uint16_t payload_offset = 8 + sizeof(float);
        if (data_length > payload_offset) {
            send_data(voltage, data + payload_offset, data_length - payload_offset);
        } else {
            ESP_LOGW(TAG, "No payload to forward after voltage extraction");
        }
    }
}

void setup_message_listening() {
    ESP_ERROR_CHECK(sx127x_set_frequency(get_channel_by_id(ENId), &lora_device));
    ESP_ERROR_CHECK(sx127x_lora_set_spreading_factor(BROADCAST_LORA_SF, &lora_device));
    sx127x_rx_set_callback(rx_callback, &lora_device, &lora_device);
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_RX_CONT, SX127X_MODULATION_LORA, &lora_device));
}

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

    ESP_LOGI(TAG, "Transmitted, now listening for response on channel %lld", get_channel_by_id(ENId));
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_RX_CONT, SX127X_MODULATION_LORA, device));
    ESP_ERROR_CHECK(sx127x_set_frequency(get_channel_by_id(ENId), device));
}

void send_data(float voltage, uint8_t *data, uint16_t data_length) {
    int payload_length = sizeof(float) + data_length + 8; // 8 bytes for command ID, layer, ID, parent ID and data length
    uint8_t *data_to_send = malloc(payload_length);
    if (data_to_send == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for transmission data");
        return;
    }

    parentNode *parent = get_parent();
    int parent_id = 0xffff;
    uint64_t channel = BROADCAST_LORA_FREQ;
    if (parent != NULL && ENLayer != 1) {
        channel = get_channel_by_id(parent->id);
        parent_id = parent->id;
    }
    if (ENLayer == 1) {
        channel = get_channel_by_id(ENId);
    }

    data_to_send[0] = 0x02;                      // Command ID for data transmission
    data_to_send[1] = ENLayer;                   // Current layer
    data_to_send[2] = (ENId >> 8) & 0xFF;        // High byte of ID
    data_to_send[3] = ENId & 0xFF;               // Low
    data_to_send[4] = (parent_id >> 8) & 0xFF;   // High byte of receiver ID
    data_to_send[5] = parent_id & 0xFF;          // Low byte of receiver ID
    data_to_send[6] = (data_length >> 8) & 0xFF; // High byte of data length
    data_to_send[7] = data_length & 0xFF;        // Low byte of data length
    memcpy(data_to_send + 8, &voltage, sizeof(float));
    memcpy(data_to_send + 8 + sizeof(float), data, data_length);

    ESP_ERROR_CHECK(sx127x_set_frequency(channel, &lora_device));

    sx127x_tx_set_callback(tx_callback, &lora_device, &lora_device);

    ESP_ERROR_CHECK(sx127x_lora_tx_set_for_transmission(data_to_send, payload_length, &lora_device));
    free(data_to_send);

    ESP_LOGI(TAG, "Transmitting data to parent 0x%04x on channel %ld", parent_id, channel);
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_TX, SX127X_MODULATION_LORA, &lora_device));
}

void dump_lora_config() {
    uint64_t freq;
    sx127x_get_frequency(&lora_device, &freq);
    ESP_LOGI(TAG, "Current LoRa frequency: %ld", freq);

    sx127x_sf_t sf;
    sx127x_lora_get_spreading_factor(&lora_device, &sf);
    ESP_LOGI(TAG, "Current LoRa spreading factor: SF%d", sf);

    sx127x_mode_t opmod;
    sx127x_modulation_t modulation;
    sx127x_get_opmod(&lora_device, &opmod, &modulation);
    const char *opmod_str = (opmod == SX127X_MODE_SLEEP)       ? "SLEEP"
                            : (opmod == SX127X_MODE_STANDBY)   ? "STDBY"
                            : (opmod == SX127X_MODE_FSTX)      ? "FSTX"
                            : (opmod == SX127X_MODE_TX)        ? "TX"
                            : (opmod == SX127X_MODE_FSRX)      ? "FSRX"
                            : (opmod == SX127X_MODE_RX_CONT)   ? "RX_CONT"
                            : (opmod == SX127X_MODE_RX_SINGLE) ? "RX_SINGLE"
                                                               : "UNKNOWN";
    const char *modulation_str = (modulation == SX127X_MODULATION_LORA) ? "LORA" : "UNKNOWN";
    ESP_LOGI(TAG, "Current LoRa operation mode: %s", opmod_str);
    ESP_LOGI(TAG, "Current LoRa modulation: %s", modulation_str);

    uint8_t syncword;
    sx127x_lora_get_syncword(&lora_device, &syncword);
    ESP_LOGI(TAG, "Current LoRa sync word: 0x%04x", syncword);
}