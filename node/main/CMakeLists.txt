idf_component_register(SRCS "led.c" "operate.c" "network_setup.c" "sensing.c" "esp_utils.c" "hbee-node.c" "logs.c" "lora_handle.c"
                    INCLUDE_DIRS "." "includes"
                    PRIV_REQUIRES esp_wifi nvs_flash mqtt esp_driver_gpio)
