idf_component_register(SRCS "can.c" INCLUDE_DIRS "include" REQUIRES gx5_proto PRIV_REQUIRES gx5_core esp_driver_gpio esp_driver_twai esp_timer nvs_flash)
