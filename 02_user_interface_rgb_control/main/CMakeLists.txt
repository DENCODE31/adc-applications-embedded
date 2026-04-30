idf_component_register(SRCS
    "ledc_basic_example_main.c"
    "library_led_c.c"
    INCLUDE_DIRS "."
    REQUIRES esp_adc esp_driver_gpio esp_driver_ledc
)
