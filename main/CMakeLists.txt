idf_component_register(SRCS "esp_utils.c" "hbee-node.c" "logs.c"
                    INCLUDE_DIRS "." "includes"
                    PRIV_REQUIRES esp_wifi nvs_flash mqtt esp_driver_gpio)
