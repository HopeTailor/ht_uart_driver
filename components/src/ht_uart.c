#include "ht_uart.h"
#include <string.h>

static ht_uart_rx_callback_t g_rx_callback[UART_NUM_MAX] = {NULL};

static uint8_t g_rx_buffer[UART_NUM_MAX][HT_UART_MAX_PAYLOAD_LEN * 2];
static size_t g_rx_index[UART_NUM_MAX] = {0};

static uint16_t crc16_ccitt_false(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)(data[i] << 8);
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static size_t cobs_encode(const uint8_t *src, size_t length, uint8_t *dst) {
    size_t read_index = 0, write_index = 1, code_index = 0;
    uint8_t code = 1;
    while (read_index < length) {
        if (src[read_index] == 0) {
            dst[code_index] = code;
            code = 1;
            code_index = write_index++;
            read_index++;
        } else {
            dst[write_index++] = src[read_index++];
            code++;
            if (code == 0xFF) {
                dst[code_index] = code;
                code = 1;
                code_index = write_index++;
            }
        }
    }
    dst[code_index] = code;
    return write_index;
}

static size_t cobs_decode(const uint8_t *src, size_t length, uint8_t *dst) {
    size_t read_index = 0, write_index = 0;
    uint8_t code = 0;
    while (read_index < length) {
        code = src[read_index];
        if (read_index + code > length && code != 1) return 0;
        read_index++;
        for (uint8_t i = 1; i < code; i++) dst[write_index++] = src[read_index++];
        if (code != 0xFF && read_index != length) dst[write_index++] = 0;
    }
    return write_index;
}

esp_err_t ht_uart_init(const ht_uart_config_t *config, QueueHandle_t *out_queue) {
    if(!config || !out_queue || config->uart_num >= UART_NUM_MAX) return ESP_ERR_INVALID_ARG;

    uart_config_t uart_cfg = {
        .baud_rate = config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_param_config(config->uart_num, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(config->uart_num, config->tx_pin, config->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(config->uart_num, config->rx_buffer_size, config->tx_buffer_size, config->queue_size, out_queue, 0));
    
    g_rx_index[config->uart_num] = 0;
    return ESP_OK;
}

esp_err_t ht_uart_set_rx_callback(uart_port_t uart_num, ht_uart_rx_callback_t cb) {
    if(uart_num >= UART_NUM_MAX) return ESP_ERR_INVALID_ARG;
    g_rx_callback[uart_num] = cb;
    return ESP_OK;
}

esp_err_t ht_uart_send(uart_port_t uart_num, const void *payload, size_t payload_len) {
    if(uart_num >= UART_NUM_MAX || !payload || payload_len == 0 || payload_len > HT_UART_MAX_PAYLOAD_LEN) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t tx_buffer[HT_UART_MAX_PAYLOAD_LEN + 2];
    memcpy(tx_buffer, payload, payload_len);
    
    uint16_t crc = crc16_ccitt_false((const uint8_t*)payload, payload_len);
    tx_buffer[payload_len] = (uint8_t)(crc >> 8);
    tx_buffer[payload_len + 1] = (uint8_t)(crc & 0xFF);

    uint8_t encoded_buffer[(HT_UART_MAX_PAYLOAD_LEN + 2) + 2 + (HT_UART_MAX_PAYLOAD_LEN / 254)];
    encoded_buffer[0] = 0x00;
    size_t encoded_len = cobs_encode(tx_buffer, payload_len + 2, &encoded_buffer[1]);
    encoded_buffer[encoded_len + 1] = 0x00;

    uart_write_bytes(uart_num, encoded_buffer, encoded_len + 2);
    return ESP_OK;
}

void ht_uart_process_event(uart_port_t uart_num, const uart_event_t *event) {
    if (uart_num >= UART_NUM_MAX || !event || event->type != UART_DATA) return;

    uint8_t dtmp[HT_UART_MAX_PAYLOAD_LEN * 2];
    uart_read_bytes(uart_num, dtmp, event->size, portMAX_DELAY);
    
    for(size_t i = 0; i < event->size; i++) {
        if(dtmp[i] == 0x00) {
            if(g_rx_index[uart_num] > 0) {
                uint8_t decoded[HT_UART_MAX_PAYLOAD_LEN];
                size_t decoded_len = cobs_decode(g_rx_buffer[uart_num], g_rx_index[uart_num], decoded);
                
                if(decoded_len >= 2) {
                    size_t payload_len = decoded_len - 2;
                    uint16_t received_crc = (decoded[payload_len] << 8) | decoded[payload_len + 1];
                    uint16_t calculated_crc = crc16_ccitt_false(decoded, payload_len);
                    
                    if(received_crc == calculated_crc && g_rx_callback[uart_num] != NULL) {
                        g_rx_callback[uart_num](decoded, payload_len);
                    }
                }
            }
            g_rx_index[uart_num] = 0;
        } 
        else {
            if(g_rx_index[uart_num] < sizeof(g_rx_buffer[uart_num])) {
                g_rx_buffer[uart_num][g_rx_index[uart_num]++] = dtmp[i];
            } 
            else {
                g_rx_index[uart_num] = 0;
            }
        }
    }
}