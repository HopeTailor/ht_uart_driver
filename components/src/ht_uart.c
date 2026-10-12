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

static void uart_event_task(void *pvParameters) {
    uart_port_t uart_num = (uart_port_t)(intptr_t)pvParameters;
    uart_event_t event;
    uint8_t *dtmp = (uint8_t *)malloc(HT_UART_MAX_PAYLOAD_LEN * 2);
    
    uint8_t rx_buffer[HT_UART_MAX_PAYLOAD_LEN * 2];
    size_t rx_index = 0;

    while (1) {
        if (xQueueReceive(g_uart_queue[uart_num], (void *)&event, (TickType_t)portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    uart_read_bytes(uart_num, dtmp, event.size, portMAX_DELAY);
                    for(size_t i = 0; i < event.size; i++) {
                        if(dtmp[i] == 0x00) {
                            if(rx_index > 0) {
                                uint8_t decoded[HT_UART_MAX_PAYLOAD_LEN];
                                size_t decoded_len = cobs_decode(rx_buffer, rx_index, decoded);
                                
                                if(decoded_len >= 2) {
                                    size_t payload_len = decoded_len - 2;
                                    uint16_t received_crc = (decoded[payload_len] << 8) | decoded[payload_len + 1];
                                    uint16_t calculated_crc = crc16_ccitt_false(decoded, payload_len);
                                    
                                    if(received_crc == calculated_crc && g_rx_callback[uart_num] != NULL) {
                                        g_rx_callback[uart_num](decoded, payload_len);
                                    }
                                }
                            }
                            rx_index = 0;
                        } else {
                            if(rx_index < sizeof(rx_buffer)) {
                                rx_buffer[rx_index++] = dtmp[i];
                            } else {
                                rx_index = 0;
                            }
                        }
                    }
                    break;
                case UART_FIFO_OVF:
                case UART_BUFFER_FULL:
                    uart_flush_input(uart_num);
                    xQueueReset(g_uart_queue[uart_num]);
                    rx_index = 0;
                    break;
                default:
                    break;
            }
        }
    }
    free(dtmp);
    vTaskDelete(NULL);
}