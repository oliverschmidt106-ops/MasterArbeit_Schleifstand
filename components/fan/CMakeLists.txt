idf_component_register(
    SRCS "fan.cpp"
    INCLUDE_DIRS "include"
    REQUIRES axis config esp_driver_gpio
)
