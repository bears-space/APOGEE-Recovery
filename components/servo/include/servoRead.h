#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/rmt_rx.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "soc/soc_caps.h"

#define SERVO_RX_SYMBOLS 64

#define SERVO_RMT_RESOLUTION_HZ  10000000UL  // 10 MHz: one tick = 100 ns
#define SERVO_MIN_HIGH_TICKS     5000U       // 500 µs
#define SERVO_MAX_HIGH_TICKS     25000U      // 2500 µs

typedef struct {
    rmt_channel_handle_t channel;

    // Keep the DMA buffer in internal RAM and suitably aligned.
    rmt_symbol_word_t symbols[SERVO_RX_SYMBOLS]
        __attribute__((aligned(32)));

    volatile uint32_t latest_high_ticks;
    volatile uint32_t rearm_errors;
} servo_rmt_rx_t;

static DRAM_ATTR servo_rmt_rx_t s_servo_rx; // DRAM_ATTR forces the struct to be placed in DRAM, which is required for DMA access.

// this must also be in DRAM, because it is used by the RMT driver in DMA mode
static DRAM_ATTR rmt_receive_config_t s_receive_config = {
    .signal_range_min_ns = 1000,       // reject glitches shorter than 1 µs
    .signal_range_max_ns = 3000000,    // stop after level stays for 3 ms
    .flags = {
        .en_partial_rx = false,
    },
};