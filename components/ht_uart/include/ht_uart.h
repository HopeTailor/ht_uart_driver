/**
 * @file ht_uart.h
 * @brief Robust UART communication component with built-in COBS and CRC16 for ESP32.
 * 
 * This library provides a generic, decoupled UART communication interface utilizing 
 * COBS (Consistent Overhead Byte Stuffing) for packet framing and CRC16-CCITT-False 
 * for robust data integrity verification.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Maximum allowed payload length in bytes.
 * Prevents dynamic memory allocation (malloc fragmentation) in FreeRTOS environments.
 */
#define HT_UART_MAX_PAYLOAD_LEN 256

/**
 * @brief Callback function type for successfully received and validated payloads.
 * 
 * @param payload Pointer to the decoded data payload (cast to user struct).
 * @param payload_len Length of the received payload in bytes.
 */
typedef void (*ht_uart_rx_callback_t)(const void *payload, size_t payload_len);

/**
 * @brief Hardware and software configuration structure for UART.
 */
typedef struct {
    uart_port_t uart_num;      /*!< UART port number (e.g., UART_NUM_1) */
    int tx_pin;                /*!< GPIO pin for TX */
    int rx_pin;                /*!< GPIO pin for RX */
    uint32_t baud_rate;        /*!< Communication baud rate (e.g., 115200) */
    size_t rx_buffer_size;     /*!< Hardware RX ring buffer size */
    size_t tx_buffer_size;     /*!< Hardware TX ring buffer size */
    size_t queue_size;         /*!< UART event queue size */
} ht_uart_config_t;

/**
 * @brief Initialize the UART driver and create the event queue.
 * 
 * @param config Pointer to the configuration structure.
 * @param[out] out_queue Pointer to receive the created FreeRTOS QueueHandle_t.
 * @return 
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if parameters are invalid
 */
esp_err_t ht_uart_init(const ht_uart_config_t *config, QueueHandle_t *out_queue);

/**
 * @brief Register a receive callback function for a specific UART port.
 * 
 * @param uart_num UART port number.
 * @param cb Callback function to be executed when a valid frame arrives.
 * @return ESP_OK on success.
 */
esp_err_t ht_uart_set_rx_callback(uart_port_t uart_num, ht_uart_rx_callback_t cb);

/**
 * @brief Pack, encode (COBS), append CRC16, and transmit a raw data payload via UART.
 * 
 * @param uart_num UART port number.
 * @param payload Pointer to the structure or data buffer.
 * @param payload_len Size of the payload in bytes.
 * @return 
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if arguments are out of bounds or payload exceeds limit.
 */
esp_err_t ht_uart_send(uart_port_t uart_num, const void *payload, size_t payload_len);

/**
 * @brief Process incoming UART events, perform COBS decoding, and validate CRC16.
 * 
 * This function should be called by the user inside their dedicated task 
 * upon receiving a UART_DATA event from the queue.
 * 
 * @param uart_num UART port number.
 * @param event Pointer to the received uart_event_t structure.
 */
void ht_uart_process_event(uart_port_t uart_num, const uart_event_t *event);

#ifdef __cplusplus
}
#endif