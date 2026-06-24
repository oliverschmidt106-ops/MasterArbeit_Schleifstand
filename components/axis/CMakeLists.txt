idf_component_register(
    SRCS "axis.cpp"
    INCLUDE_DIRS "include"
    REQUIRES config driver esp_driver_gpio esp_timer FastAccelStepper as5600
)
