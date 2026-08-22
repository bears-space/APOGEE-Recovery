#include <stddef.h>
#include <stdint.h>

#include "driver/ledc.h"
#include "driver/mcpwm_prelude.h"
#include "esp_check.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "vigilant.h"

#define BUZZER_GPIO 38
#define BUZZER_VOLUME_PERCENT 100
#define BUZZER_MAX_DUTY 512
#define BPM 90
#define NOTE_GATE_PERCENT 90

#if BUZZER_VOLUME_PERCENT < 0 || BUZZER_VOLUME_PERCENT > 100
#error "BUZZER_VOLUME_PERCENT must be between 0 and 100"
#endif

#define SERVO_GPIO 21
#define SERVO_FREQUENCY_HZ 50
#define SERVO_PERIOD_US (1000000 / SERVO_FREQUENCY_HZ)
#define SERVO_MIN_PULSE_US 1000
#define SERVO_MAX_PULSE_US 2000
#define SERVO_SIGNAL_INVERTED 1

#if SERVO_SIGNAL_INVERTED
#define SERVO_PERIOD_START_ACTION MCPWM_GEN_ACTION_LOW
#define SERVO_PULSE_END_ACTION MCPWM_GEN_ACTION_HIGH
#else
#define SERVO_PERIOD_START_ACTION MCPWM_GEN_ACTION_HIGH
#define SERVO_PULSE_END_ACTION MCPWM_GEN_ACTION_LOW
#endif

#define NOTE_D6 1175
#define NOTE_E6 1319
#define NOTE_G6 1568
#define NOTE_A6 1760
#define NOTE_B6 1976
#define NOTE_C7 2093
#define NOTE_D7 2349

#define REST 0

typedef enum {
    SIXTEENTH = 1,
    EIGHTH = 2,
    QUARTER = 4,
    HALF = 8,
    WHOLE = 16,
} note_length_t;

typedef struct {
    uint16_t frequency_hz;
    note_length_t length;
} melody_note_t;

static mcpwm_cmpr_handle_t servo_comparator;

static const melody_note_t melody[] = {
    // Takt 8: "Du bist gut ge-nug"
    {REST, HALF},
    {NOTE_C7, EIGHTH},
    {NOTE_B6, EIGHTH},
    {NOTE_C7, EIGHTH},
    {NOTE_B6, SIXTEENTH},
    {NOTE_C7, SIXTEENTH},

    // Takt 9
    {NOTE_C7, EIGHTH},
    {NOTE_B6, QUARTER},
    {NOTE_A6, QUARTER},
    {NOTE_G6, QUARTER},
    {NOTE_D6, QUARTER},

    // Takt 10
    {NOTE_D6, EIGHTH},
    {REST, QUARTER},
    {NOTE_C7, EIGHTH},
    {NOTE_B6, EIGHTH},
    {NOTE_C7, EIGHTH},
    {NOTE_B6, SIXTEENTH},
    {NOTE_C7, SIXTEENTH},

    // Takt 11
    {NOTE_C7, EIGHTH},
    {NOTE_B6, QUARTER},
    {NOTE_A6, QUARTER},
    {NOTE_D7, QUARTER},
    {NOTE_A6, QUARTER},

    // Takt 12
    {NOTE_A6, EIGHTH},
    {REST, QUARTER},
    {NOTE_C7, EIGHTH},
    {NOTE_B6, EIGHTH},
    {NOTE_C7, EIGHTH},
    {NOTE_B6, SIXTEENTH},
    {NOTE_C7, SIXTEENTH},

    // Takt 13
    {NOTE_C7, EIGHTH},
    {NOTE_B6, QUARTER},
    {NOTE_A6, QUARTER},
    {NOTE_G6, QUARTER},
    {NOTE_E6, EIGHTH},
};

static uint32_t servo_angle_to_pulse(float angle) {
    if (angle < 0.0f) {
        angle = 0.0f;
    } else if (angle > 180.0f) {
        angle = 180.0f;
    }

    return SERVO_MIN_PULSE_US +
           (uint32_t)((angle / 180.0f) *
                      (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US));
}

static esp_err_t servo_set_angle(float angle) {
    return mcpwm_comparator_set_compare_value(servo_comparator,
                                              servo_angle_to_pulse(angle));
}

static esp_err_t servo_init(void) {
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_oper_handle_t operator = NULL;
    mcpwm_gen_handle_t generator = NULL;

    const mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,
        .period_ticks = SERVO_PERIOD_US,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_RETURN_ON_ERROR(mcpwm_new_timer(&timer_config, &timer), "servo",
                        "Could not create timer");

    const mcpwm_operator_config_t operator_config = {
        .group_id = 0,
    };
    ESP_RETURN_ON_ERROR(mcpwm_new_operator(&operator_config, &operator),
                        "servo", "Could not create operator");
    ESP_RETURN_ON_ERROR(mcpwm_operator_connect_timer(operator, timer), "servo",
                        "Could not connect timer");

    const mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_RETURN_ON_ERROR(
        mcpwm_new_comparator(operator, &comparator_config, &servo_comparator),
        "servo", "Could not create comparator");

    const mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = SERVO_GPIO,
    };
    ESP_RETURN_ON_ERROR(
        mcpwm_new_generator(operator, &generator_config, &generator), "servo",
        "Could not create generator");

    ESP_RETURN_ON_ERROR(
        mcpwm_generator_set_action_on_timer_event(
            generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                    MCPWM_TIMER_EVENT_EMPTY,
                                                    SERVO_PERIOD_START_ACTION)),
        "servo", "Could not configure timer action");

    ESP_RETURN_ON_ERROR(
        mcpwm_generator_set_action_on_compare_event(
            generator, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                      servo_comparator,
                                                      SERVO_PULSE_END_ACTION)),
        "servo", "Could not configure comparator action");

    ESP_RETURN_ON_ERROR(servo_set_angle(90.0f), "servo",
                        "Could not set initial position");
    ESP_RETURN_ON_ERROR(mcpwm_timer_enable(timer), "servo",
                        "Could not enable timer");
    ESP_RETURN_ON_ERROR(
        mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP), "servo",
        "Could not start timer");

    return ESP_OK;
}

static float servo_angle_for_note(uint16_t frequency_hz) {
    switch (frequency_hz) {
        case NOTE_D6:
            return 40.0f;
        case NOTE_E6:
            return 56.0f;
        case NOTE_G6:
            return 80.0f;
        case NOTE_A6:
            return 96.0f;
        case NOTE_B6:
            return 112.0f;
        case NOTE_C7:
            return 120.0f;
        case NOTE_D7:
            return 136.0f;
        case REST:
        default:
            return 90.0f;
    }
}

static uint32_t note_duration_ms(note_length_t length) {
    return (60000UL * (uint32_t)length) / (BPM * 4UL);
}

static void buzzer_set_frequency(uint32_t frequency_hz) {
    const uint32_t duty =
        BUZZER_MAX_DUTY * (uint32_t)BUZZER_VOLUME_PERCENT / 100;

    ESP_ERROR_CHECK(
        ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, frequency_hz));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

static void buzzer_stop(void) {
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

static void buzzer_init(void) {
    const ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    const ledc_channel_config_t channel_config = {
        .gpio_num = BUZZER_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
}

static void play_note(const melody_note_t* note) {
    const uint32_t duration_ms = note_duration_ms(note->length);
    ESP_ERROR_CHECK(servo_set_angle(servo_angle_for_note(note->frequency_hz)));

    if (note->frequency_hz == REST) {
        buzzer_stop();
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
        return;
    }

    const uint32_t sounding_ms = duration_ms * NOTE_GATE_PERCENT / 100;
    buzzer_set_frequency(note->frequency_hz);
    vTaskDelay(pdMS_TO_TICKS(sounding_ms));
    buzzer_stop();
    vTaskDelay(pdMS_TO_TICKS(duration_ms - sounding_ms));
}

void app_main(void) {
    const VigilantConfig config = {
        .unique_component_name = "Vigilant ESP Test",
        .network_mode = NW_MODE_APSTA,
    };
    ESP_ERROR_CHECK(vigilant_init(config));

    buzzer_init();
    ESP_ERROR_CHECK(servo_init());

    while (true) {
        for (size_t i = 0; i < sizeof(melody) / sizeof(melody[0]); ++i) {
            play_note(&melody[i]);
        }
    }
}
