#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Opaque handle for one independently controlled servo. */
typedef struct servo_instance* servo_handle_t;

typedef struct {
    int gpio_num;
    int group_id;
    uint32_t frequency_hz;
    uint32_t minimum_pulse_width_us;
    uint32_t maximum_pulse_width_us;
    float minimum_angle_degrees;
    float maximum_angle_degrees;
    float initial_angle_degrees;
    /** Invert the physical GPIO output in the GPIO matrix. */
    bool output_inverted;
} servo_config_t;

#define SERVO_CONFIG_DEFAULT(gpio_)      \
    {                                    \
        .gpio_num = (gpio_),             \
        .group_id = 0,                   \
        .frequency_hz = 50,              \
        .minimum_pulse_width_us = 1000,  \
        .maximum_pulse_width_us = 2000,  \
        .minimum_angle_degrees = 0.0f,   \
        .maximum_angle_degrees = 180.0f, \
        .initial_angle_degrees = 90.0f,  \
        .output_inverted = false,        \
    }

/**
 * Create an independent servo instance.
 *
 * MCPWM resources are allocated by ESP-IDF for every handle. Additional
 * instances can be created until the selected MCPWM group has no free hardware
 * timers/operators; choose another group_id to use that group's resources.
 */
esp_err_t servo_new(const servo_config_t* config, servo_handle_t* ret_servo);

/** Set an angle, clamped to the configured angle range. */
esp_err_t servo_set_angle(servo_handle_t servo, float angle_degrees);

/** Set a raw pulse width inside the configured servo pulse range. */
esp_err_t servo_set_pulse_width_us(servo_handle_t servo,
                                   uint32_t pulse_width_us);

/** Stop and release one servo and all MCPWM resources owned by it. */
esp_err_t servo_del(servo_handle_t servo);

#ifdef __cplusplus
}
#endif
