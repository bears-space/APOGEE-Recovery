#include "servoRead.h"

#include "driver/gpio.h"
#include "esp_attr.h"

#define SERVO_RMT_RESOLUTION_HZ 10000000UL  // 10 MHz: one tick = 100 ns
#define SERVO_MIN_HIGH_TICKS 5000U          // 500 µs
#define SERVO_MAX_HIGH_TICKS 25000U         // 2500 µs

// The callback can run while flash is unavailable in cache-safe builds.
static DRAM_ATTR const rmt_receive_config_t s_receive_config = {
    .signal_range_min_ns = 1000,     // reject glitches shorter than 1 µs
    .signal_range_max_ns = 3000000,  // finish after 3 ms of inactivity
    .flags =
        {
            .en_partial_rx = false,
        },
};

static esp_err_t IRAM_ATTR servoStartReceive(servo_rmt_rx_t* receiver) {
    return rmt_receive(receiver->channel, receiver->symbols,
                       sizeof(receiver->symbols), &s_receive_config);
}

static void IRAM_ATTR servoRecordRearmFailure(servo_rmt_rx_t* receiver) {
    __atomic_store_n(&receiver->rearm_pending, true, __ATOMIC_RELEASE);
    __atomic_add_fetch(&receiver->rearm_errors, 1, __ATOMIC_RELAXED);
}

static void servoTryRearm(servo_rmt_rx_t* receiver) {
    if (!__atomic_load_n(&receiver->rearm_pending, __ATOMIC_ACQUIRE) ||
        receiver->channel == NULL || !receiver->enabled ||
        __atomic_load_n(&receiver->stopping, __ATOMIC_ACQUIRE)) {
        return;
    }

    if (servoStartReceive(receiver) == ESP_OK) {
        __atomic_store_n(&receiver->rearm_pending, false, __ATOMIC_RELEASE);
    } else {
        __atomic_add_fetch(&receiver->rearm_errors, 1, __ATOMIC_RELAXED);
    }
}

static bool IRAM_ATTR servoRxDoneCallback(rmt_channel_handle_t channel,
                                          const rmt_rx_done_event_data_t* edata,
                                          void* user_data) {
    servo_rmt_rx_t* ctx = user_data;
    uint32_t high_ticks = 0;

    // Each symbol contains two levels; retain the last valid HIGH duration.
    for (size_t i = 0; i < edata->num_symbols; i++) {
        const rmt_symbol_word_t symbol = edata->received_symbols[i];

        if (symbol.level0 == 1 && symbol.duration0 >= SERVO_MIN_HIGH_TICKS &&
            symbol.duration0 <= SERVO_MAX_HIGH_TICKS) {
            high_ticks = symbol.duration0;
        }

        if (symbol.level1 == 1 && symbol.duration1 >= SERVO_MIN_HIGH_TICKS &&
            symbol.duration1 <= SERVO_MAX_HIGH_TICKS) {
            high_ticks = symbol.duration1;
        }
    }

    if (high_ticks != 0) {
        __atomic_store_n(&ctx->latest_high_ticks, high_ticks, __ATOMIC_RELEASE);
    }

    if (!__atomic_load_n(&ctx->stopping, __ATOMIC_ACQUIRE) &&
        servoStartReceive(ctx) != ESP_OK) {
        servoRecordRearmFailure(ctx);
    }

    return false;
}

esp_err_t servoRmtInit(servo_rmt_rx_t* receiver, gpio_num_t gpio) {
    if (receiver == NULL || !GPIO_IS_VALID_GPIO(gpio)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (receiver->channel != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    __atomic_store_n(&receiver->latest_high_ticks, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&receiver->rearm_errors, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&receiver->rearm_pending, false, __ATOMIC_RELAXED);
    __atomic_store_n(&receiver->stopping, false, __ATOMIC_RELAXED);
    receiver->enabled = false;

    const rmt_rx_channel_config_t channel_config = {
        .gpio_num = gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_RMT_RESOLUTION_HZ,
        .mem_block_symbols = SERVO_RX_SYMBOLS,
        .intr_priority = 0,
        .flags =
            {
                .invert_in = true, // needed on our apogee-recovery
                // ESP32-S3 has only one DMA-capable RX channel per RMT group.
                // Normal RMT memory is ample for a servo pulse and permits
                // multiple receivers.
                .with_dma = false,
                .allow_pd = false,
            },
    };

    esp_err_t err = rmt_new_rx_channel(&channel_config, &receiver->channel);
    if (err != ESP_OK) {
        receiver->channel = NULL;
        return err;
    }

    const rmt_rx_event_callbacks_t callbacks = {
        .on_recv_done = servoRxDoneCallback,
    };

    err = rmt_rx_register_event_callbacks(receiver->channel, &callbacks,
                                          receiver);
    if (err != ESP_OK) {
        (void)rmt_del_channel(receiver->channel);
        receiver->channel = NULL;
        return err;
    }

    err = rmt_enable(receiver->channel);
    if (err != ESP_OK) {
        (void)rmt_del_channel(receiver->channel);
        receiver->channel = NULL;
        return err;
    }
    receiver->enabled = true;

    err = servoStartReceive(receiver);

    if (err != ESP_OK) {
        (void)servoRmtDeinit(receiver);
    }

    return err;
}

esp_err_t servoRmtDeinit(servo_rmt_rx_t* receiver) {
    if (receiver == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (receiver->channel == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    __atomic_store_n(&receiver->stopping, true, __ATOMIC_RELEASE);

    if (receiver->enabled) {
        const esp_err_t disable_error = rmt_disable(receiver->channel);
        if (disable_error != ESP_OK) {
            __atomic_store_n(&receiver->stopping, false, __ATOMIC_RELEASE);
            return disable_error;
        }
        receiver->enabled = false;
    }

    const rmt_rx_event_callbacks_t callbacks = {0};
    esp_err_t result =
        rmt_rx_register_event_callbacks(receiver->channel, &callbacks, NULL);
    const esp_err_t delete_error = rmt_del_channel(receiver->channel);
    if (result == ESP_OK) {
        result = delete_error;
    }

    if (delete_error == ESP_OK) {
        receiver->channel = NULL;
        __atomic_store_n(&receiver->latest_high_ticks, 0, __ATOMIC_RELAXED);
        __atomic_store_n(&receiver->rearm_pending, false, __ATOMIC_RELAXED);
        __atomic_store_n(&receiver->stopping, false, __ATOMIC_RELAXED);
    }

    return result;
}

bool servoGetPulseWidthTicks(servo_rmt_rx_t* receiver, uint32_t* ticks) {
    if (receiver == NULL || ticks == NULL || receiver->channel == NULL) {
        return false;
    }

    servoTryRearm(receiver);

    const uint32_t value =
        __atomic_exchange_n(&receiver->latest_high_ticks, 0, __ATOMIC_ACQ_REL);

    if (value == 0) {
        return false;
    }

    *ticks = value;
    return true;
}

bool servoGetPulseWidthNanoseconds(servo_rmt_rx_t* receiver,
                                   uint32_t* width_ns) {
    if (width_ns == NULL) {
        return false;
    }

    uint32_t ticks;

    if (!servoGetPulseWidthTicks(receiver, &ticks)) {
        return false;
    }

    // At 10 MHz this is simply ticks × 100 ns.
    *width_ns =
        (uint32_t)(((uint64_t)ticks * 1000000000ULL) / SERVO_RMT_RESOLUTION_HZ);

    return true;
}
