idf_component_register(SRCS "can.c" INCLUDE_DIRS "include" PRIV_REQUIRES gx5_core gx5_proto esp_driver_gpio esp_driver_twai esp_timer nvs_flash)
