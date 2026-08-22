#include <unistd.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "status_led.h"
#include "vigilant.h"
#include "driver/mcpwm_prelude.h"
#include "esp_err.h"
#include "esp_check.h"

#define SERVO_GPIO             21

#define SERVO_FREQUENCY_HZ     50
#define SERVO_PERIOD_US        (1000000 / SERVO_FREQUENCY_HZ)

#define SERVO_MIN_PULSE_US     1000
#define SERVO_MAX_PULSE_US     2000

static const char *TAG = "app_main";

static mcpwm_cmpr_handle_t servo_comparator;

static uint32_t servo_angle_to_pulse(float angle)
{
    if (angle < 0.0f) {
        angle = 0.0f;
    } else if (angle > 180.0f) {
        angle = 180.0f;
    }

    return SERVO_MIN_PULSE_US +
           (uint32_t)((angle / 180.0f) *
           (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US));
}

esp_err_t servo_set_angle(float angle)
{
    uint32_t pulse_width_us = servo_angle_to_pulse(angle);

    return mcpwm_comparator_set_compare_value(
        servo_comparator,
        pulse_width_us
    );
}

esp_err_t servo_init(void)
{
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_oper_handle_t operator = NULL;
    mcpwm_gen_handle_t generator = NULL;

    /* 1 MHz means one timer tick equals one microsecond. */
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,
        .period_ticks = SERVO_PERIOD_US,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_RETURN_ON_ERROR(
        mcpwm_new_timer(&timer_config, &timer),
        "servo",
        "Could not create timer"
    );

    mcpwm_operator_config_t operator_config = {
        .group_id = 0,
    };
    ESP_RETURN_ON_ERROR(
        mcpwm_new_operator(&operator_config, &operator),
        "servo",
        "Could not create operator"
    );

    ESP_RETURN_ON_ERROR(
        mcpwm_operator_connect_timer(operator, timer),
        "servo",
        "Could not connect timer"
    );

    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_RETURN_ON_ERROR(
        mcpwm_new_comparator(
            operator,
            &comparator_config,
            &servo_comparator
        ),
        "servo",
        "Could not create comparator"
    );

    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = SERVO_GPIO,
    };
    ESP_RETURN_ON_ERROR(
        mcpwm_new_generator(
            operator,
            &generator_config,
            &generator
        ),
        "servo",
        "Could not create generator"
    );

    /*
     * Start each 20 ms period HIGH.
     * Set the output LOW when the comparator value is reached.
     */
    ESP_RETURN_ON_ERROR(
        mcpwm_generator_set_action_on_timer_event(
            generator,
            MCPWM_GEN_TIMER_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                MCPWM_TIMER_EVENT_EMPTY,
                MCPWM_GEN_ACTION_HIGH
            )
        ),
        "servo",
        "Could not configure timer action"
    );

    ESP_RETURN_ON_ERROR(
        mcpwm_generator_set_action_on_compare_event(
            generator,
            MCPWM_GEN_COMPARE_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                servo_comparator,
                MCPWM_GEN_ACTION_LOW
            )
        ),
        "servo",
        "Could not configure comparator action"
    );

    /* Start at the center position. */
    ESP_RETURN_ON_ERROR(
        servo_set_angle(90.0f),
        "servo",
        "Could not set initial position"
    );

    ESP_RETURN_ON_ERROR(
        mcpwm_timer_enable(timer),
        "servo",
        "Could not enable timer"
    );

    ESP_RETURN_ON_ERROR(
        mcpwm_timer_start_stop(
            timer,
            MCPWM_TIMER_START_NO_STOP
        ),
        "servo",
        "Could not start timer"
    );

    return ESP_OK;
}

void app_main(void) {
    VigilantConfig VgConfig = {.unique_component_name = "Vigilant ESP Test",
                               .network_mode = NW_MODE_APSTA};
    ESP_ERROR_CHECK(vigilant_init(VgConfig));

    ESP_ERROR_CHECK(servo_init());

    while (true) {
        ESP_ERROR_CHECK(servo_set_angle(0));
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_ERROR_CHECK(servo_set_angle(90));
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_ERROR_CHECK(servo_set_angle(180));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
