
#include "sensing.h"
#include "logs.h"
#include "operate.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ina260.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

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

        send_data(voltage, (uint8_t *)&current, sizeof(float));

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
