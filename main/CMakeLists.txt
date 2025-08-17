idf_component_register(SRCS "presenter.c"
                    INCLUDE_DIRS "."
                    REQUIRES bt esp_driver_gpio esp_hid nvs_flash)
