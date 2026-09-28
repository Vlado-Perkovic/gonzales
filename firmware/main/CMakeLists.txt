idf_component_register(
    SRCS "main.c" "console.c" "measure.c" "cfg.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_uart esp_driver_gpio esp_adc nvs_flash esp_timer
)
