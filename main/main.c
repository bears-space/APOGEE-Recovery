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
#define BPM 90
#define NOTE_GATE_PERCENT 90

#define SERVO_GPIO 21
#define SERVO_FREQUENCY_HZ 50
#define SERVO_PERIOD_US (1000000 / SERVO_FREQUENCY_HZ)
#define SERVO_MIN_PULSE_US 1000
#define SERVO_MAX_PULSE_US 2000
#define SERVO_INVERTED 1

#define NOTE_C4 262
#define NOTE_D4 294
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_G4 392
#define NOTE_A4 440

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
    // "Alle meine Entchen"
    {NOTE_C4, EIGHTH},
    {NOTE_D4, EIGHTH},
    {NOTE_E4, EIGHTH},
    {NOTE_F4, EIGHTH},
    {NOTE_G4, QUARTER},
    {NOTE_G4, QUARTER},

    // "schwimmen auf dem See"
    {NOTE_A4, EIGHTH},
    {NOTE_A4, EIGHTH},
    {NOTE_A4, EIGHTH},
    {NOTE_A4, EIGHTH},
    {NOTE_G4, HALF},

    // "schwimmen auf dem See"
    {NOTE_A4, EIGHTH},
    {NOTE_A4, EIGHTH},
    {NOTE_A4, EIGHTH},
    {NOTE_A4, EIGHTH},
    {NOTE_G4, HALF},

    // "Köpfchen in das Wasser"
    {NOTE_F4, EIGHTH},
    {NOTE_F4, EIGHTH},
    {NOTE_F4, EIGHTH},
    {NOTE_F4, EIGHTH},
    {NOTE_E4, QUARTER},
    {NOTE_E4, QUARTER},

    // "Schwänzchen in die Höh'"
    {NOTE_D4, EIGHTH},
    {NOTE_D4, EIGHTH},
    {NOTE_D4, EIGHTH},
    {NOTE_D4, EIGHTH},
    {NOTE_C4, HALF},
};

static uint32_t servo_angle_to_pulse(float angle) {
    if (angle < 0.0f) {
        angle = 0.0f;
    } else if (angle > 180.0f) {
        angle = 180.0f;
    }

#if SERVO_INVERTED
    angle = 180.0f - angle;
#endif

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
                                                    MCPWM_GEN_ACTION_HIGH)),
        "servo", "Could not configure timer action");

    ESP_RETURN_ON_ERROR(
        mcpwm_generator_set_action_on_compare_event(
            generator, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                      servo_comparator,
                                                      MCPWM_GEN_ACTION_LOW)),
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

static void servo_sweep_task(void* argument) {
    (void)argument;

    while (true) {
        ESP_ERROR_CHECK(servo_set_angle(0.0f));
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_ERROR_CHECK(servo_set_angle(90.0f));
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_ERROR_CHECK(servo_set_angle(180.0f));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static uint32_t note_duration_ms(note_length_t length) {
    return (60000UL * (uint32_t)length) / (BPM * 4UL);
}

static void buzzer_set_frequency(uint32_t frequency_hz) {
    ESP_ERROR_CHECK(
        ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, frequency_hz));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 512));
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
    ESP_ERROR_CHECK(xTaskCreate(servo_sweep_task, "servo_sweep", 3072, NULL, 5,
                                NULL) == pdPASS
                        ? ESP_OK
                        : ESP_FAIL);

    while (true) {
        for (size_t i = 0; i < sizeof(melody) / sizeof(melody[0]); ++i) {
            play_note(&melody[i]);
        }
    }
}
