#include <stdio.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_attr.h"

#include "pyro.h"

#define INTERRUPT_PIN GPIO_NUM_2 // make this a kconfig later

static volatile bool ignitionActive = false;

static void IRAM_ATTR ignitionInterruptHandler(void *arg) { // the actual interrupt handler for the ignition signal
    ignitionActive = true;

    // implement all logic here, maybe also a debounce??

    ESP_LOGI("Pyro", "Ignition signal detected!");
}

void initPyro(void) {
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