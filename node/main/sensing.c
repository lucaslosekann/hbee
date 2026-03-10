
#include "sensing.h"
#include "esp_random.h"
#include "esp_sleep.h"
#include "led.h"
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
int sent_pkts = 0;
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

    int seconds = (10 * 2 - ENId * 2);
    vTaskDelay(pdMS_TO_TICKS(seconds * 1000));
    while (1) {
        // if (sent_pkts == 30) {
        if ((sent_pkts == 30 && ENId != 4 && ENId != 7) || (sent_pkts == 15 && (ENId == 4 || ENId == 7))) {
            ESP_LOGE(TAG, "Aborting program");

            set_led_color(255, 0, 0);

            // turn off the mcu
            esp_deep_sleep_start();
        }
        ESP_ERROR_CHECK(ina260_trigger(&dev));

        wait_for_sensor(&dev);

        ESP_ERROR_CHECK(ina260_get_bus_voltage(&dev, &voltage));
        ESP_ERROR_CHECK(ina260_get_current(&dev, &current));
        ESP_ERROR_CHECK(ina260_get_power(&dev, &power));

        sx127x_mode_t opmod;
        sx127x_modulation_t modulation;
        sx127x_get_opmod(&lora_device, &opmod, &modulation);

        uint8_t data[8];
        memcpy(data, &current, sizeof(float));
        data[4] = (ENId >> 8) & 0xFF;
        data[5] = ENId & 0xFF;
        data[6] = ENLayer;
        data[7] = sent_pkts;

        // ESP_LOGI(TAG, "Voltage: %.2f V, Current: %.2f mA, Power: %.2f mW", voltage, current, power);
        send_data(voltage, data, sizeof(data));
        sent_pkts++;

        // 20s delay
        vTaskDelay(pdMS_TO_TICKS(20000));
    }
}
