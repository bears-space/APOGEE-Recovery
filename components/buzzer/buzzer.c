#include "buzzer.h"

#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "soc/soc_caps.h"

#if SOC_LEDC_TIMER_NUM > 32 || SOC_LEDC_CHANNEL_NUM > 32
#error "The buzzer resource allocator supports at most 32 timers and channels"
#endif

struct buzzer_instance {
    ledc_timer_t timer;
    ledc_channel_t channel;
    uint32_t maximum_duty;
};

static portMUX_TYPE resource_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t reserved_timers;
static uint32_t reserved_channels;

static bool requested_resource_is_valid(int resource, int resource_count) {
    return resource == BUZZER_LEDC_RESOURCE_AUTO ||
           (resource >= 0 && resource < resource_count);
}

static int select_resource(int requested, uint32_t reserved,
                           int resource_count) {
    if (requested != BUZZER_LEDC_RESOURCE_AUTO) {
        return (reserved & (UINT32_C(1) << requested)) == 0 ? requested : -1;
    }

    for (int resource = 0; resource < resource_count; ++resource) {
        if ((reserved & (UINT32_C(1) << resource)) == 0) {
            return resource;
        }
    }

    return -1;
}

static esp_err_t reserve_resources(const buzzer_config_t* config,
                                   ledc_timer_t* timer,
                                   ledc_channel_t* channel) {
    esp_err_t result = ESP_OK;

    portENTER_CRITICAL(&resource_lock);

    const int selected_timer =
        select_resource(config->timer_num, reserved_timers, SOC_LEDC_TIMER_NUM);
    const int selected_channel = select_resource(
        config->channel_num, reserved_channels, SOC_LEDC_CHANNEL_NUM);

    if (selected_timer < 0 || selected_channel < 0) {
        const bool explicitly_reserved =
            (config->timer_num != BUZZER_LEDC_RESOURCE_AUTO &&
             selected_timer < 0) ||
            (config->channel_num != BUZZER_LEDC_RESOURCE_AUTO &&
             selected_channel < 0);
        result =
            explicitly_reserved ? ESP_ERR_INVALID_STATE : ESP_ERR_NOT_FOUND;
    } else {
        reserved_timers |= UINT32_C(1) << selected_timer;
        reserved_channels |= UINT32_C(1) << selected_channel;
        *timer = (ledc_timer_t)selected_timer;
        *channel = (ledc_channel_t)selected_channel;
    }

    portEXIT_CRITICAL(&resource_lock);
    return result;
}

static void release_resources(ledc_timer_t timer, ledc_channel_t channel) {
    portENTER_CRITICAL(&resource_lock);
    reserved_timers &= ~(UINT32_C(1) << timer);
    reserved_channels &= ~(UINT32_C(1) << channel);
    portEXIT_CRITICAL(&resource_lock);
}

static void record_first_error(esp_err_t* first_error, esp_err_t error) {
    if (*first_error == ESP_OK && error != ESP_OK) {
        *first_error = error;
    }
}

static esp_err_t deconfigure_timer(ledc_timer_t timer) {
    esp_err_t result = ledc_timer_pause(LEDC_LOW_SPEED_MODE, timer);
    if (result != ESP_OK) {
        return result;
    }

    const ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = timer,
        .deconfigure = true,
    };
    return ledc_timer_config(&timer_config);
}

esp_err_t buzzer_new(const buzzer_config_t* config,
                     buzzer_handle_t* ret_buzzer) {
    if (config == NULL || ret_buzzer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *ret_buzzer = NULL;

    if (!GPIO_IS_VALID_OUTPUT_GPIO(config->gpio_num) ||
        !requested_resource_is_valid(config->timer_num, SOC_LEDC_TIMER_NUM) ||
        !requested_resource_is_valid(config->channel_num,
                                     SOC_LEDC_CHANNEL_NUM) ||
        config->duty_resolution_bits == 0 ||
        config->duty_resolution_bits > SOC_LEDC_TIMER_BIT_WIDTH ||
        config->initial_frequency_hz == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    buzzer_handle_t buzzer = calloc(1, sizeof(*buzzer));
    if (buzzer == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t result =
        reserve_resources(config, &buzzer->timer, &buzzer->channel);
    if (result != ESP_OK) {
        free(buzzer);
        return result;
    }

    const ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)config->duty_resolution_bits,
        .timer_num = buzzer->timer,
        .freq_hz = config->initial_frequency_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    result = ledc_timer_config(&timer_config);
    if (result != ESP_OK) {
        release_resources(buzzer->timer, buzzer->channel);
        free(buzzer);
        return result;
    }

    const ledc_channel_config_t channel_config = {
        .gpio_num = config->gpio_num,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = buzzer->channel,
        .timer_sel = buzzer->timer,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = config->output_inverted,
    };
    result = ledc_channel_config(&channel_config);
    if (result != ESP_OK) {
        const esp_err_t cleanup_result = deconfigure_timer(buzzer->timer);
        (void)cleanup_result;
        release_resources(buzzer->timer, buzzer->channel);
        free(buzzer);
        return result;
    }

    buzzer->maximum_duty = UINT32_C(1) << (config->duty_resolution_bits - 1);
    *ret_buzzer = buzzer;
    return ESP_OK;
}

esp_err_t buzzer_set_tone(buzzer_handle_t buzzer, uint32_t frequency_hz,
                          uint8_t volume_percent) {
    if (buzzer == NULL || frequency_hz == 0 || volume_percent > 100) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t result =
        ledc_set_freq(LEDC_LOW_SPEED_MODE, buzzer->timer, frequency_hz);
    if (result != ESP_OK) {
        return result;
    }

    const uint32_t duty = buzzer->maximum_duty * (uint32_t)volume_percent / 100;
    return ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, buzzer->channel, duty,
                                    0);
}

esp_err_t buzzer_stop(buzzer_handle_t buzzer) {
    if (buzzer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, buzzer->channel, 0, 0);
}

esp_err_t buzzer_del(buzzer_handle_t buzzer) {
    if (buzzer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t result = ESP_OK;
    record_first_error(&result, buzzer_stop(buzzer));

    const ledc_channel_config_t channel_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = buzzer->channel,
        .deconfigure = true,
    };
    record_first_error(&result, ledc_channel_config(&channel_config));
    record_first_error(&result, deconfigure_timer(buzzer->timer));

    release_resources(buzzer->timer, buzzer->channel);
    free(buzzer);
    return result;
}
