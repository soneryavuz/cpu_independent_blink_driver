idf_component_register(SRCS "blink_example_main.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio
                                esp_driver_gptimer
                                blink_module
                                )