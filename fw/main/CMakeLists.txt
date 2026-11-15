idf_component_register(
    SRCS "main.c"
    PRIV_REQUIRES gx5_core gx5_proto gx5_modem gx5_io gx5_store gx5_can gx5_ext
                  esp_driver_gpio esp_adc esp_event esp_timer esp_pm esp_hw_support nvs_flash esp_app_format)
