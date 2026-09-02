#pragma once

#include <stdbool.h>

#include "esp_err.h"
typedef struct {
    int channel_number;
    int gpio_pin;
    bool has_fired;
} PyroChannel;

void initPyro(size_t num_channels, PyroChannel (*channels)[num_channels]);
bool isIgnitionActive(void);
esp_err_t firePyroChannel(PyroChannel* channel);
