#include "ht_uart.h"

static ht_uart_rx_callback_t g_rx_callback[UART_NUM_MAX] = {NULL};
static QueueHandle_t g_uart_queue[UART_NUM_MAX] = {NULL};

static uint16_t crc16_ccitt_false(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for(size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)(data[i] << 8);
        for(int j = 0; j < 8; j++) {
            if(crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            }
            else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static size_t cobs_encode(const uint8_t *src, size_t length, uint8_t *dst) {
    size_t read_index = 0;
    size_t write_index = 1;
    size_t code_index = 0;
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
    size_t read_index = 0;
    size_t write_index = 0;
    uint8_t code = 0;
    uint8_t i = 0;

    while (read_index < length) {
        code = src[read_index];
        if (read_index + code > length && code != 1) {
            return 0;
        }
        read_index++;
        for (i = 1; i < code; i++) {
            dst[write_index++] = src[read_index++];
        }
        if (code != 0xFF && read_index != length) {
            dst[write_index++] = 0;
        }
    }
    return write_index;
}

