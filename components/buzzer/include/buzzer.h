#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Let the component choose an unused LEDC timer or channel. */
#define BUZZER_LEDC_RESOURCE_AUTO (-1)

/** Opaque handle for one independently controlled buzzer. */
typedef struct buzzer_instance* buzzer_handle_t;

typedef struct {
    int gpio_num;
    int timer_num;
    int channel_num;
    uint8_t duty_resolution_bits;
    uint32_t initial_frequency_hz;
    bool output_inverted;
} buzzer_config_t;

#define BUZZER_CONFIG_DEFAULT(gpio_)              \
    {                                             \
        .gpio_num = (gpio_),                      \
        .timer_num = BUZZER_LEDC_RESOURCE_AUTO,   \
        .channel_num = BUZZER_LEDC_RESOURCE_AUTO, \
        .duty_resolution_bits = 10,               \
        .initial_frequency_hz = 1000,             \
        .output_inverted = false,                 \
    }

/**
 * Create an independent buzzer instance.
 *
 * Each instance reserves its own LEDC timer as well as a channel, allowing
 * different buzzers to play different frequencies simultaneously. Automatic
 * allocation covers resources owned by this component; explicit timer/channel
 * values are available when sharing LEDC with other components.
 */
esp_err_t buzzer_new(const buzzer_config_t* config,
                     buzzer_handle_t* ret_buzzer);

/** Play a tone. Volume must be in the range 0..100 percent. */
esp_err_t buzzer_set_tone(buzzer_handle_t buzzer, uint32_t frequency_hz,
                          uint8_t volume_percent);

/** Silence one buzzer without affecting any other instance. */
esp_err_t buzzer_stop(buzzer_handle_t buzzer);

/** Stop and release one buzzer and its LEDC resources. */
esp_err_t buzzer_del(buzzer_handle_t buzzer);

#ifdef __cplusplus
}
#endif
