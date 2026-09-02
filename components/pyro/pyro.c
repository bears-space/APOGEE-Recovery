#include <stdio.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_attr.h"

#include "pyro.h"

#define INTERRUPT_PIN GPIO_NUM_2 // make this a kconfig later

static volatile bool ignitionActive = false;

static void IRAM_ATTR ignitionInterruptHandler(void *arg) { // the actual interrupt handler for the ignition signal
    ignitionActive = true;

    // Dontput any other code in here, make a task instead.
}

void initPyro(size_t num_channels, PyroChannel (*channels)[num_channels]) {
    ESP_LOGI("Pyro", "Initializing Pyro module...");

    // initialize the interrupt stuff
    gpio_config_t config = {
        .pin_bit_mask = 1ULL << INTERRUPT_PIN,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // we dont need any pull-resistors because we have an external one
        .intr_type    = GPIO_INTR_ANYEDGE, // trigger on any edge
    };

    ESP_ERROR_CHECK(gpio_config(&config));

    ESP_ERROR_CHECK(gpio_install_isr_service(0)); // NOTE: THIS CAN ONLY BE CALLED ONCE, SO IF YOU HAVE OTHER INTERRUPTS, YOU NEED TO CALL THIS IN YOUR MAIN INIT FUNCTION

    ESP_ERROR_CHECK(gpio_isr_handler_add( // attach the interrupt handler to the pin
        INTERRUPT_PIN,
        ignitionInterruptHandler,
        (void *)(uintptr_t)INTERRUPT_PIN
    ));

    // read the initial state of the ignition pin
    ignitionActive = (gpio_get_level(INTERRUPT_PIN) == 1); // if the pin is high, ignition is active, otherwise it is not

    uint64_t pin_mask = 0;
    for (size_t i = 0; i < num_channels; i++)
    {
        pin_mask |= 1ULL << (*channels)[i].gpio_pin;
        ESP_LOGI("Pyro", "Added channel %d on GPIO pin %d, whole pin mask: 0x%016llx", (*channels)[i].channel_number, (*channels)[i].gpio_pin, (unsigned long long)pin_mask);
    }

    gpio_config_t channelConfig = {
        .pin_bit_mask = pin_mask,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE, // no interrupts for the pyro channels
    };
    esp_err_t err = gpio_config(&channelConfig); // only set the pin mask once, for some reason it overwrites the previous config if you call it multiple times, so we do it once with a pin mask of all the channels
    if (err != ESP_OK) {
        ESP_LOGE("Pyro", "Failed to configure GPIO for pyro channels: %s", esp_err_to_name(err));
    }
    else {
        ESP_LOGI("Pyro", "Successfully configured GPIO for pyro channels.");
    }
    
}

bool isIgnitionActive(void) {
    return ignitionActive;
}

esp_err_t firePyroChannel(PyroChannel *channel) {
    // Simulate firing the pyro channel
    ESP_LOGI("Pyro", "Firing channel %d on GPIO pin %d.", channel->channel_number, channel->gpio_pin);
    
    if (!ignitionActive)
    {
        ESP_LOGE("Pyro", "Cannot fire channel %d: ignition not active.", channel->channel_number);
        return ESP_ERR_INVALID_STATE; // ignition not active, cannot fire
    }
    

    // set the pin high 
    esp_err_t err = gpio_set_level(channel->gpio_pin, 1);
    if (err != ESP_OK) {
        ESP_LOGE("Pyro", "Failed to set GPIO level for channel %d.", channel->channel_number);
        return err;
    }
    channel->has_fired = true;

    return ESP_OK;
}