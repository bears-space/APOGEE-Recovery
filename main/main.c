#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "servo.h"
#include "servoRead.h"
#include "vigilant.h"
#include "esp_log.h"

#define SERVO_GPIO 21

static void servoPulseReaderTask(void *arg)
{
    (void)arg;

    static DRAM_ATTR servo_rmt_rx_t rmtReceiverIO17;
    static DRAM_ATTR servo_rmt_rx_t rmtReceiverIO18;
    ESP_ERROR_CHECK(servoRmtInit(&rmtReceiverIO17, GPIO_NUM_17));
    ESP_ERROR_CHECK(servoRmtInit(&rmtReceiverIO18, GPIO_NUM_18));

    while (true) {
        uint32_t ticks;
        
        if (servoGetPulseWidthTicks(&rmtReceiverIO18, &ticks)) {
            ESP_LOGI("SERVO [COTS-1]:", "Servo pulse width: %" PRIu32 " ticks\n", ticks);
        } else {
            ESP_LOGW("SERVO [COTS-1]:", "No pulse width measurement available\n");
        }

        if (servoGetPulseWidthTicks(&rmtReceiverIO17, &ticks)) {
            ESP_LOGI("SERVO [COTS-2]:", "Servo pulse width: %" PRIu32 " ticks\n", ticks);
        } else {
            ESP_LOGW("SERVO [COTS-2]:", "No pulse width measurement available\n");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

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

    // start a new task to read the servo pulse width in ticks
    BaseType_t result = xTaskCreate(servoPulseReaderTask, "servo_pulse_reader", 2048, NULL, 1, NULL);

    if (result != pdPASS) {
        ESP_LOGE("SERVO", "Failed to create servo reader task");
        return;
    }

    while (true) {
        ESP_ERROR_CHECK(servoSetAngle(servo, 0.0f));
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_ERROR_CHECK(servoSetAngle(servo, 90.0f));
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_ERROR_CHECK(servoSetAngle(servo, 180.0f));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
