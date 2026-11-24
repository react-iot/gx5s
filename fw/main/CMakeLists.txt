idf_component_register(
    SRCS "main.c"
    PRIV_REQUIRES gx5_core gx5_proto gx5_modem gx5_io gx5_store gx5_can gx5_ext
                  esp_driver_gpio esp_driver_uart esp_adc ulp soc esp_event esp_timer esp_pm esp_hw_support nvs_flash esp_app_format)

# Supply watch for deep sleep (ulp/vcc.S), run by the ULP coprocessor.
ulp_embed_binary(ulp_main "ulp/vcc.S" "main.c" TYPE fsm)
