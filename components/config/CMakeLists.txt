idf_component_register(
    SRCS "config.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_driver_gpio esp_driver_uart
)
