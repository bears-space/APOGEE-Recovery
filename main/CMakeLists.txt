idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES vigilant_engine esp_driver_ledc esp_driver_mcpwm
)
