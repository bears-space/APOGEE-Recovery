#include "servo.h"

#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"

#define SERVO_TIMER_RESOLUTION_HZ 1000000UL

struct servoInstance {
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t operator;
    mcpwm_cmpr_handle_t comparator;
    mcpwm_gen_handle_t generator;
    uint32_t minimum_pulse_width_us;
    uint32_t maximum_pulse_width_us;
    float minimum_angle_degrees;
    float maximum_angle_degrees;
    bool timer_enabled;
    bool timer_started;
};

static void recordFirstError(esp_err_t* firstErr, esp_err_t error) {
    if (*firstErr == ESP_OK && error != ESP_OK) {
        *firstErr = error;
    }
}

static esp_err_t releaseHardware(servo_handle_t servo) {
    esp_err_t result = ESP_OK;

    if (servo->timer_started) {
        recordFirstError(&result, mcpwm_timer_start_stop(
                                        servo->timer, MCPWM_TIMER_STOP_EMPTY));
        servo->timer_started = false;
    }
    if (servo->timer_enabled) {
        recordFirstError(&result, mcpwm_timer_disable(servo->timer));
        servo->timer_enabled = false;
    }
    if (servo->generator != NULL) {
        recordFirstError(&result, mcpwm_del_generator(servo->generator));
        servo->generator = NULL;
    }
    if (servo->comparator != NULL) {
        recordFirstError(&result, mcpwm_del_comparator(servo->comparator));
        servo->comparator = NULL;
    }
    if (servo->operator != NULL) {
        recordFirstError(&result, mcpwm_del_operator(servo->operator));
        servo->operator = NULL;
    }
    if (servo->timer != NULL) {
        recordFirstError(&result, mcpwm_del_timer(servo->timer));
        servo->timer = NULL;
    }

    return result;
}

static uint32_t pulseWidthAngle(servo_handle_t servo,
                                      float angle_degrees) {
    if (angle_degrees < servo->minimum_angle_degrees) {
        angle_degrees = servo->minimum_angle_degrees;
    } else if (angle_degrees > servo->maximum_angle_degrees) {
        angle_degrees = servo->maximum_angle_degrees;
    }

    const float angle_fraction =
        (angle_degrees - servo->minimum_angle_degrees) /
        (servo->maximum_angle_degrees - servo->minimum_angle_degrees);
    return servo->minimum_pulse_width_us +
           (uint32_t)(angle_fraction * (servo->maximum_pulse_width_us -
                                        servo->minimum_pulse_width_us));
}

esp_err_t servoNew(const servoConfig_t* config, servo_handle_t* ret_servo) {
    if (config == NULL || ret_servo == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *ret_servo = NULL;

    if (!GPIO_IS_VALID_OUTPUT_GPIO(config->gpio_num) || config->group_id < 0 ||
        config->frequency_hz == 0 || config->minimum_pulse_width_us == 0 ||
        config->minimum_pulse_width_us >= config->maximum_pulse_width_us ||
        config->minimum_angle_degrees >= config->maximum_angle_degrees ||
        config->initial_angle_degrees < config->minimum_angle_degrees ||
        config->initial_angle_degrees > config->maximum_angle_degrees) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint32_t period_ticks =
        SERVO_TIMER_RESOLUTION_HZ / config->frequency_hz;
    if (period_ticks == 0 || config->maximum_pulse_width_us >= period_ticks) {
        return ESP_ERR_INVALID_ARG;
    }

    servo_handle_t servo = calloc(1, sizeof(*servo));
    if (servo == NULL) {
        return ESP_ERR_NO_MEM;
    }

    servo->minimum_pulse_width_us = config->minimum_pulse_width_us;
    servo->maximum_pulse_width_us = config->maximum_pulse_width_us;
    servo->minimum_angle_degrees = config->minimum_angle_degrees;
    servo->maximum_angle_degrees = config->maximum_angle_degrees;

    const mcpwm_timer_config_t timer_config = {
        .group_id = config->group_id,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMER_RESOLUTION_HZ,
        .period_ticks = period_ticks,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    esp_err_t result = mcpwm_new_timer(&timer_config, &servo->timer);
    if (result != ESP_OK) {
        goto fail;
    }

    const mcpwm_operator_config_t operator_config = {
        .group_id = config->group_id,
    };
    result = mcpwm_new_operator(&operator_config, &servo->operator);
    if (result != ESP_OK) {
        goto fail;
    }

    result = mcpwm_operator_connect_timer(servo->operator, servo->timer);
    if (result != ESP_OK) {
        goto fail;
    }

    const mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    result = mcpwm_new_comparator(servo->operator, &comparator_config,
                                  &servo->comparator);
    if (result != ESP_OK) {
        goto fail;
    }

    const mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = config->gpio_num,
        .flags.invert_pwm = config->output_inverted,
    };
    result = mcpwm_new_generator(servo->operator, &generator_config,
                                 &servo->generator);
    if (result != ESP_OK) {
        goto fail;
    }

    result = mcpwm_generator_set_action_on_timer_event(
        servo->generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                       MCPWM_TIMER_EVENT_EMPTY,
                                                       MCPWM_GEN_ACTION_HIGH));
    if (result != ESP_OK) {
        goto fail;
    }

    result = mcpwm_generator_set_action_on_compare_event(
        servo->generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP, servo->comparator, MCPWM_GEN_ACTION_LOW));
    if (result != ESP_OK) {
        goto fail;
    }

    result = servoSetAngle(servo, config->initial_angle_degrees);
    if (result != ESP_OK) {
        goto fail;
    }

    result = mcpwm_timer_enable(servo->timer);
    if (result != ESP_OK) {
        goto fail;
    }
    servo->timer_enabled = true;

    result = mcpwm_timer_start_stop(servo->timer, MCPWM_TIMER_START_NO_STOP);
    if (result != ESP_OK) {
        goto fail;
    }
    servo->timer_started = true;

    *ret_servo = servo;
    return ESP_OK;

fail:
    (void)releaseHardware(servo);
    free(servo);
    return result;
}

esp_err_t servoSetAngle(servo_handle_t servo, float angle_degrees) {
    if (servo == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return mcpwm_comparator_set_compare_value(
        servo->comparator, pulseWidthAngle(servo, angle_degrees));
}

esp_err_t servoSetPulseWidth_us(servo_handle_t servo,
                                   uint32_t pulse_width_us) {
    if (servo == NULL || pulse_width_us < servo->minimum_pulse_width_us ||
        pulse_width_us > servo->maximum_pulse_width_us) {
        return ESP_ERR_INVALID_ARG;
    }

    return mcpwm_comparator_set_compare_value(servo->comparator,
                                              pulse_width_us);
}

esp_err_t servo_del(servo_handle_t servo) {
    if (servo == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const esp_err_t result = releaseHardware(servo);
    free(servo);
    return result;
}
