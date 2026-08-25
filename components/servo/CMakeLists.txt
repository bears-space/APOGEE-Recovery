idf_component_register(
    SRCS "servo.c" "servoRead.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_common esp_driver_rmt
    PRIV_REQUIRES esp_driver_gpio esp_driver_mcpwm
)
