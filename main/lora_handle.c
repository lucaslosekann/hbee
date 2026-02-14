#include "lora_handle.h"
#include "nvs_flash.h"
#include <driver/spi_master.h>
#include <esp_intr_alloc.h>
#include <esp_utils.h>
#include <inttypes.h>
#include <sx127x.h>

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

    ESP_ERROR_CHECK(setup_task(&lora_device));

    gpio_install_isr_service(0);
    setup_gpio_interrupts((gpio_num_t)DIO0, &lora_device, GPIO_INTR_POSEDGE);
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
