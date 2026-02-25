
#include "sensing.h"
#include "logs.h"
#include "lora_handle.h"
#include "operate.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <hbee-node.h>
#include <ina260.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sx127x.h>

static const char *TAG = "SENSING";

void wait_for_sensor(ina260_t *dev) {
    bool ready, alert, overflow;
    while (1) {
        ESP_ERROR_CHECK(ina260_get_status(dev, &ready, &alert, &overflow));
        if (alert) printf("ALERT! ");
        if (overflow) printf("OVERFLOW! ");
        if (alert || overflow) printf("\n");
        if (ready) break;
    }
}

void sensing_task(void *pvParameters) {
    ina260_t dev;
    memset(&dev, 0, sizeof(ina260_t));

    ESP_ERROR_CHECK(ina260_init_desc(&dev, I2C_SENSING_ADDR, I2C_SENSING_PORT, I2C_SENSING_SDA, I2C_SENSING_SCL));
    printf("Initializing ina260\n");
    ESP_ERROR_CHECK(ina260_init(&dev));

    printf("Configuring ina260\n");
    // Triggered mode, 128 samples, 1.1 ms conversion time (~150 ms)
    ESP_ERROR_CHECK(ina260_set_config(&dev, INA260_MODE_TRIG_SHUNT_BUS, INA260_AVG_128, INA260_CT_1100, INA260_CT_1100));

    wait_for_sensor(&dev);

    float voltage, current, power;
    while (1) {
        ESP_ERROR_CHECK(ina260_trigger(&dev));

        wait_for_sensor(&dev);

        ESP_ERROR_CHECK(ina260_get_bus_voltage(&dev, &voltage));
        ESP_ERROR_CHECK(ina260_get_current(&dev, &current));
        ESP_ERROR_CHECK(ina260_get_power(&dev, &power));

        sx127x_mode_t opmod;
        sx127x_modulation_t modulation;
        sx127x_get_opmod(&lora_device, &opmod, &modulation);

        if (opmod != SX127X_MODE_STANDBY && opmod != SX127X_MODE_RX_CONT) {
            ESP_LOGW(TAG, "LoRa device not in standby or RX mode, skipping transmission");
            continue;
        }

        uint8_t data[6];
        memcpy(data, &current, sizeof(float));
        data[4] = (ENId >> 8) & 0xFF;
        data[5] = ENId & 0xFF;

        // ESP_LOGI(TAG, "Voltage: %.2f V, Current: %.2f mA, Power: %.2f mW", voltage, current, power);
        send_data(voltage, data, sizeof(data));

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
