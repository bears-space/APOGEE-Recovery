#include "servoRead.h"

#if !SOC_RMT_SUPPORT_DMA
#error "This ESP target does not support RMT RX with DMA"
#endif

static bool IRAM_ATTR servoRxDoneCallback(
    rmt_channel_handle_t channel,
    const rmt_rx_done_event_data_t *edata,
    void *user_data)
{
    servo_rmt_rx_t *ctx = user_data;
    uint32_t high_ticks = 0;

    // each symbol contains a HIGH and a LOW duration, but we only care about the HIGH duration for servo pulse width measurement
    for (size_t i = 0; i < edata->num_symbols; i++) {
        const rmt_symbol_word_t symbol = edata->received_symbols[i];

        if (symbol.level0 == 1 &&
            symbol.duration0 >= SERVO_MIN_HIGH_TICKS &&
            symbol.duration0 <= SERVO_MAX_HIGH_TICKS) {
            high_ticks = symbol.duration0;
        }

        if (symbol.level1 == 1 &&
            symbol.duration1 >= SERVO_MIN_HIGH_TICKS &&
            symbol.duration1 <= SERVO_MAX_HIGH_TICKS) {
            high_ticks = symbol.duration1;
        }
    }

    if (high_ticks != 0) {
        __atomic_store_n( // atomic store to avoid race conditions with the main task reading the latest_high_ticks value
            &ctx->latest_high_ticks,
            high_ticks,
            __ATOMIC_RELEASE
        );
    }

    /*
     * the driver has already returned the channel to its enabled state before
     * invoking this callback. rmt_receive() is explicitly allowed from ISR
     * context, so rearm immediately without involving the scheduler. the scheduler could fuck things up otherwise
     */
    esp_err_t err = rmt_receive(
        channel,
        ctx->symbols,
        sizeof(ctx->symbols),       // bytes, not number of symbols
        &s_receive_config
    );

    if (err != ESP_OK) {
        __atomic_add_fetch(
            &ctx->rearm_errors,
            1,
            __ATOMIC_RELAXED
        );
    }

    return false; // no task was awakened
}

esp_err_t servoRmtInit(gpio_num_t gpio)
{
    s_servo_rx.latest_high_ticks = 0;
    s_servo_rx.rearm_errors = 0;

    const rmt_rx_channel_config_t channel_config = {
        .gpio_num = gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_RMT_RESOLUTION_HZ,
        .mem_block_symbols = SERVO_RX_SYMBOLS,
        .intr_priority = 0,
        .flags = {
            .invert_in = false,
            .with_dma = true,
            .allow_pd = false,
        },
    };

    esp_err_t err = rmt_new_rx_channel(
        &channel_config,
        &s_servo_rx.channel
    );
    if (err != ESP_OK) {
        return err; // ESP_ERR_NOT_SUPPORTED if target lacks RMT DMA
    }

    const rmt_rx_event_callbacks_t callbacks = {
        .on_recv_done = servoRxDoneCallback,
    };

    err = rmt_rx_register_event_callbacks(
        s_servo_rx.channel,
        &callbacks,
        &s_servo_rx
    );
    if (err != ESP_OK) { //
        rmt_del_channel(s_servo_rx.channel);
        return err;
    }

    err = rmt_enable(s_servo_rx.channel);
    if (err != ESP_OK) {
        rmt_del_channel(s_servo_rx.channel);
        return err;
    }

    err = rmt_receive(
        s_servo_rx.channel,
        s_servo_rx.symbols,
        sizeof(s_servo_rx.symbols),
        &s_receive_config
    );

    if (err != ESP_OK) { // disable and delete the channel if we fail to start receiving
        rmt_disable(s_servo_rx.channel);
        rmt_del_channel(s_servo_rx.channel);
    }

    return err;
}

bool servoGetPulseWidthTicks(uint32_t *ticks)
{
    /*
    This function retrieves the latest pulse width measurement in ticks from the servo RMT receiver. It uses an atomic load operation to safely read the value of `latest_high_ticks` from the `s_servo_rx` structure, ensuring that there are no race conditions with the main task that may be reading this value concurrently
    */

    uint32_t value = __atomic_load_n(
        &s_servo_rx.latest_high_ticks,
        __ATOMIC_ACQUIRE
    );

    if (value == 0) {
        return false;
    }

    *ticks = value;
    return true;
}

bool servoGetPulseWidthNanoseconds(uint32_t *width_ns)
{
    uint32_t ticks;

    if (!servoGetPulseWidthTicks(&ticks)) {
        return false;
    }

    // At 10 MHz this is simply ticks × 100 ns.
    *width_ns = (uint32_t)(
        ((uint64_t)ticks * 1000000000ULL) /
        SERVO_RMT_RESOLUTION_HZ
    );

    return true;
}