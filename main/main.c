#include <inttypes.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "servo.h"
#include "servoRead.h"
#include "vigilant.h"

#define SERVO_GPIO 21
#define COTS_1_RX_GPIO GPIO_NUM_18
#define COTS_2_RX_GPIO GPIO_NUM_17

static const char* TAG = "SERVO";

static servo_rmt_rx_t s_cots_1_receiver = SERVO_RMT_RX_INITIALIZER;
static servo_rmt_rx_t s_cots_2_receiver = SERVO_RMT_RX_INITIALIZER;

static void servoPulseReaderTask(void* arg) {
    (void)arg;

    while (true) {
        uint32_t ticks;

        if (servoGetPulseWidthTicks(&s_cots_1_receiver, &ticks)) {
            ESP_LOGI(TAG, "COTS-1 pulse width: %" PRIu32 " ticks", ticks);
        } else {
            ESP_LOGW(TAG, "No COTS-1 pulse width measurement available");
        }

        if (servoGetPulseWidthTicks(&s_cots_2_receiver, &ticks)) {
            ESP_LOGI(TAG, "COTS-2 pulse width: %" PRIu32 " ticks", ticks);
        } else {
            ESP_LOGW(TAG, "No COTS-2 pulse width measurement available");
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

    esp_err_t error = servoRmtInit(&s_cots_1_receiver, COTS_1_RX_GPIO);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize COTS-1 receiver: %s",
                 esp_err_to_name(error));
        ESP_ERROR_CHECK(error);
    }

    error = servoRmtInit(&s_cots_2_receiver, COTS_2_RX_GPIO);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize COTS-2 receiver: %s",
                 esp_err_to_name(error));
        (void)servoRmtDeinit(&s_cots_1_receiver);
        ESP_ERROR_CHECK(error);
    }

    // start a new task to read the servo pulse width in ticks
    const BaseType_t result = xTaskCreate(
        servoPulseReaderTask, "servo_pulse_reader", 2048, NULL, 1, NULL);

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create servo reader task");
        (void)servoRmtDeinit(&s_cots_2_receiver);
        (void)servoRmtDeinit(&s_cots_1_receiver);
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
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
