idf_component_register(
    SRCS "protocol.cpp"
    INCLUDE_DIRS "include"
    REQUIRES config axis fv driver esp_driver_uart
)
