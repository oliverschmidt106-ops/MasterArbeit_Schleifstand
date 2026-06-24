idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES config axis protocol as5600 FastAccelStepper
)
