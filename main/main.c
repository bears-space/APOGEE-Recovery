#include <unistd.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "status_led.h"
#include "vigilant.h"
#include "pyro.h"

static const char *TAG = "app_main";

void app_main(void) {
    VigilantConfig VgConfig = {.unique_component_name = "Vigilant ESP Test",
                               .network_mode = NW_MODE_APSTA};
    ESP_ERROR_CHECK(vigilant_init(VgConfig));

    PyroChannel channel1 = {
        .channel_number = 1,
        .gpio_pin = GPIO_NUM_18,
        .has_fired = false
    };

    initPyro();
}
