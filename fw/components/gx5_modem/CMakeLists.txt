idf_component_register(
    SRCS "modem.c"
    INCLUDE_DIRS "include"
    REQUIRES gx5_proto
    PRIV_REQUIRES gx5_core esp_driver_gpio esp_driver_uart esp_pm esp_hw_support nvs_flash esp_app_format app_update)
