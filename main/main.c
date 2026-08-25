#include <stddef.h>
#include <stdint.h>

#include "buzzer.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "vigilant.h"

#define BUZZER_GPIO 38
#define BUZZER_VOLUME_PERCENT 100
#define BPM 90
#define NOTE_GATE_PERCENT 90

#if BUZZER_VOLUME_PERCENT < 0 || BUZZER_VOLUME_PERCENT > 100
#error "BUZZER_VOLUME_PERCENT must be between 0 and 100"
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

static const melody_note_t melody[] = {
    {REST, HALF},
    {NOTE_C7, EIGHTH},
    {NOTE_B6, EIGHTH},
    {NOTE_C7, EIGHTH},
    {NOTE_B6, SIXTEENTH},
    {NOTE_C7, SIXTEENTH},

    {NOTE_C7, EIGHTH},
    {NOTE_B6, QUARTER},
    {NOTE_A6, QUARTER},
    {NOTE_G6, QUARTER},
    {NOTE_D6, QUARTER},

    {NOTE_D6, EIGHTH},
    {REST, QUARTER},
    {NOTE_C7, EIGHTH},
    {NOTE_B6, EIGHTH},
    {NOTE_C7, EIGHTH},
    {NOTE_B6, SIXTEENTH},
    {NOTE_C7, SIXTEENTH},

    {NOTE_C7, EIGHTH},
    {NOTE_B6, QUARTER},
    {NOTE_A6, QUARTER},
    {NOTE_D7, QUARTER},
    {NOTE_A6, QUARTER},

    {NOTE_A6, EIGHTH},
    {REST, QUARTER},
    {NOTE_C7, EIGHTH},
    {NOTE_B6, EIGHTH},
    {NOTE_C7, EIGHTH},
    {NOTE_B6, SIXTEENTH},
    {NOTE_C7, SIXTEENTH},

    {NOTE_C7, EIGHTH},
    {NOTE_B6, QUARTER},
    {NOTE_A6, QUARTER},
    {NOTE_G6, QUARTER},
    {NOTE_E6, EIGHTH},
};

static uint32_t note_duration_ms(note_length_t length) {
    return (60000UL * (uint32_t)length) / (BPM * 4UL);
}

static void play_note(buzzer_handle_t buzzer, const melody_note_t* note) {
    const uint32_t duration_ms = note_duration_ms(note->length);

    if (note->frequency_hz == REST) {
        ESP_ERROR_CHECK(buzzer_stop(buzzer));
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
        return;
    }

    const uint32_t sounding_ms = duration_ms * NOTE_GATE_PERCENT / 100;
    ESP_ERROR_CHECK(
        buzzer_set_tone(buzzer, note->frequency_hz, BUZZER_VOLUME_PERCENT));
    vTaskDelay(pdMS_TO_TICKS(sounding_ms));
    ESP_ERROR_CHECK(buzzer_stop(buzzer));
    vTaskDelay(pdMS_TO_TICKS(duration_ms - sounding_ms));
}

void app_main(void) {
    const VigilantConfig config = {
        .unique_component_name = "Vigilant ESP Test",
        .network_mode = NW_MODE_APSTA,
    };
    ESP_ERROR_CHECK(vigilant_init(config));

    buzzer_handle_t buzzer = NULL;
    const buzzer_config_t buzzer_config = BUZZER_CONFIG_DEFAULT(BUZZER_GPIO);
    ESP_ERROR_CHECK(buzzer_new(&buzzer_config, &buzzer));

    while (true) {
        for (size_t i = 0; i < sizeof(melody) / sizeof(melody[0]); ++i) {
            play_note(buzzer, &melody[i]);
        }
    }
}
