// #include "driver/rmt_tx.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include <stdio.h>
// Define the LED GPIO
#define LED_GPIO_NUM 21

led_strip_handle_t led_strip = NULL;

void config_led() {
    // Configuration for the WS2812 LED
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO_NUM,
        .max_leds = 1, // Only 1 LED on board
    };

    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
}

void set_led_color(uint8_t r, uint8_t g, uint8_t b) {
    if (led_strip == NULL) {
        ESP_LOGE("LED", "led_strip is NULL");
        return;
    }
    led_strip_set_pixel(led_strip, 0, g, r, b); // Set the first (and only) LED
    led_strip_refresh(led_strip);               // Refresh to apply the changes
}