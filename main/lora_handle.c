#include "lora_handle.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include "operate.h"
#include <driver/spi_master.h>
#include <esp_intr_alloc.h>
#include <esp_log.h>
#include <esp_utils.h>
#include <inttypes.h>
#include <sx127x.h>
static const char *TAG = "LORA_HANDLE";

sx127x lora_device;

void setup_lora_device() {
    sx127x_util_reset();

    spi_device_handle_t spi_device;
    sx127x_init_spi(&spi_device);

    ESP_ERROR_CHECK(sx127x_create(spi_device, &lora_device));
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_STANDBY, SX127X_MODULATION_LORA, &lora_device));
    ESP_ERROR_CHECK(sx127x_set_frequency(BROADCAST_LORA_FREQ, &lora_device));
    ESP_ERROR_CHECK(sx127x_lora_reset_fifo(&lora_device));
    ESP_ERROR_CHECK(sx127x_rx_set_lna_boost_hf(true, &lora_device));
    ESP_ERROR_CHECK(sx127x_rx_set_lna_gain(SX127X_LNA_GAIN_G4, &lora_device));
    ESP_ERROR_CHECK(sx127x_lora_set_bandwidth(SX127X_BW_125000, &lora_device));
    ESP_ERROR_CHECK(sx127x_lora_set_implicit_header(NULL, &lora_device));
    ESP_ERROR_CHECK(sx127x_lora_set_spreading_factor(BROADCAST_LORA_SF, &lora_device));
    ESP_ERROR_CHECK(sx127x_lora_set_syncword(18, &lora_device));
    ESP_ERROR_CHECK(sx127x_set_preamble_length(8, &lora_device));
    sx127x_tx_header_t header = {.enable_crc = true, .coding_rate = SX127X_CR_4_5};
    ESP_ERROR_CHECK(sx127x_lora_tx_set_explicit_header(&header, &lora_device));
    ESP_ERROR_CHECK(setup_task(&lora_device));

    gpio_install_isr_service(0);
    setup_gpio_interrupts((gpio_num_t)DIO0, &lora_device, GPIO_INTR_POSEDGE);
    setup_gpio_interrupts((gpio_num_t)DIO1, &lora_device, GPIO_INTR_POSEDGE);
    setup_gpio_interrupts((gpio_num_t)DIO2, &lora_device, GPIO_INTR_POSEDGE);

    start_tx_queue_consumer();
}

void save_id(int32_t id) {
    nvs_handle_t handle;
    nvs_open("storage", NVS_READWRITE, &handle);

    nvs_set_i32(handle, "device_id", id);
    nvs_commit(handle);
    nvs_close(handle);
}

int32_t read_id() {
    nvs_handle_t handle;
    int32_t id = 0;

    nvs_open("storage", NVS_READONLY, &handle);
    nvs_get_i32(handle, "device_id", &id);
    nvs_close(handle);

    return id;
}

// CAD detection

QueueHandle_t cadQueue;

void cad_callback(void *ctx, int cad_detected) {
    ESP_LOGI("CAD", "Callback fired! detected=%d", cad_detected);
    int result = cad_detected ? 1 : 0;
    xQueueOverwrite(cadQueue, &result);
}

bool detect_cad_blocking(uint64_t channel) {
    // Implement CAD detection logic here
    if (cadQueue == NULL) {
        cadQueue = xQueueCreate(1, sizeof(int));
    }
    xQueueReset(cadQueue);
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_STANDBY, SX127X_MODULATION_LORA, &lora_device));
    vTaskDelay(pdMS_TO_TICKS(2));
    ESP_ERROR_CHECK(sx127x_lora_reset_fifo(&lora_device));

    sx127x_lora_cad_set_callback(cad_callback, &lora_device, &lora_device);
    ESP_ERROR_CHECK(sx127x_set_frequency(channel, &lora_device));
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_CAD, SX127X_MODULATION_LORA, &lora_device));

    int received;
    if (xQueueReceive(cadQueue, &received, pdMS_TO_TICKS(1000)) == pdPASS) {
        return received != 0;
    }
    sx127x_get_frequency(&lora_device, &channel);
    ESP_LOGW(TAG, "CAD detection timed out on channel %lld", channel);
    sx127x_set_opmod(SX127X_MODE_STANDBY, SX127X_MODULATION_LORA, &lora_device);

    return true; // Assume channel is busy if we don't get a response in time
}