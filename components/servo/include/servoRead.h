#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/rmt_rx.h"
#include "esp_err.h"

#define SERVO_RX_SYMBOLS 48
#define SERVO_RMT_RX_INITIALIZER {0}

typedef struct {
    rmt_channel_handle_t channel;

    // The RMT driver may require cache-line alignment for its receive buffer.
    rmt_symbol_word_t symbols[SERVO_RX_SYMBOLS] __attribute__((aligned(32)));

    volatile uint32_t latest_high_ticks;
    volatile uint32_t rearm_errors;
    volatile bool rearm_pending;
    volatile bool stopping;
    bool enabled;
} servo_rmt_rx_t;

/** Initialize one zero-initialized receiver instance on the selected GPIO. */
esp_err_t servoRmtInit(servo_rmt_rx_t* receiver, gpio_num_t gpio);

/** Stop one receiver and release its RMT channel. */
esp_err_t servoRmtDeinit(servo_rmt_rx_t* receiver);

/** Consume the latest unread valid pulse-width sample. */
bool servoGetPulseWidthTicks(servo_rmt_rx_t* receiver, uint32_t* ticks);

/** Consume the latest unread valid pulse-width sample in nanoseconds. */
bool servoGetPulseWidthNanoseconds(servo_rmt_rx_t* receiver,
                                   uint32_t* width_ns);
