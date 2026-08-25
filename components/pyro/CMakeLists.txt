idf_component_register(
    SRCS "pyro.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_common
    PRIV_REQUIRES "driver esp_driver_gpio log"
)