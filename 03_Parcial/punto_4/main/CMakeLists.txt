idf_component_register(SRCS "library_led_c.c" "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_adc)
