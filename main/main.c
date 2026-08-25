#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "servo.h"
#include "servoRead.h"
#include "vigilant.h"

#define SERVO_GPIO 21

void app_main(void) {
    const VigilantConfig config = {
        .unique_component_name = "Vigilant ESP Test",
        .network_mode = NW_MODE_APSTA,
    };
    ESP_ERROR_CHECK(vigilant_init(config));

    servo_handle_t servo = NULL;
    servoConfig_t servo_config = SERVO_CONFIG_DEFAULT(SERVO_GPIO);
    servo_config.output_inverted = true;
    ESP_ERROR_CHECK(servoNew(&servo_config, &servo));

    while (true) {
        ESP_ERROR_CHECK(servoSetAngle(servo, 0.0f));
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_ERROR_CHECK(servoSetAngle(servo, 90.0f));
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_ERROR_CHECK(servoSetAngle(servo, 180.0f));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    
}
