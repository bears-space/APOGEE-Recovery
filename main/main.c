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
    VigilantConfig VgConfig = {.unique_component_name = "Recovery",
                               .network_mode = NW_MODE_APSTA};
    ESP_ERROR_CHECK(vigilant_init(VgConfig));

    PyroChannel pyro1 = {
        .channel_number = 1,
        .gpio_pin = GPIO_NUM_37,
        .has_fired = false
    };

    PyroChannel pyro2 = {
        .channel_number = 2,
        .gpio_pin = GPIO_NUM_48,
        .has_fired = false
    };

    PyroChannel pyro3 = {
        .channel_number = 3,
        .gpio_pin = GPIO_NUM_35,
        .has_fired = false
    };

    PyroChannel pyro4 = {
        .channel_number = 4,
        .gpio_pin = GPIO_NUM_36,
        .has_fired = false
    };

    initPyro();

    // wait 10 seconds
    ESP_LOGI(TAG, "Waiting 10 seconds before firing pyro channels...");
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    firePyroChannel(&pyro1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    firePyroChannel(&pyro2);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    firePyroChannel(&pyro3);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    firePyroChannel(&pyro4);
}
