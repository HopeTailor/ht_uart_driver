#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/uart.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HT_UART_MAX_PAYLOAD_LEN 256

typedef void (*ht_uart_rx_callback_t)(const void *payload, size_t payload_len);

typedef struct {
    uart_port_t uart_num;
    int tx_pin;
    int rx_pin;
    uint32_t baud_rate;
    size_t rx_buffer_size;
    size_t tx_buffer_size;
    uint8_t task_priority;
} ht_uart_config_t;

esp_err_t ht_uart_init(const ht_uart_config_t *config);

esp_err_t ht_uart_set_rx_callback(uart_port_t uart_num, ht_uart_rx_callback_t cb);

esp_err_t ht_uart_send(uart_port_t uart_num, const void *payload, size_t payload_len);

#ifdef __cplusplus
}
#endif
