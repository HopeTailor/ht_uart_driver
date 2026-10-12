#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "ht_uart.h"

#define VL_SENSOR_COUNT 4

typedef struct __attribute__((packed)) {
    uint16_t vl_data[VL_SENSOR_COUNT];
    int16_t  deltaYaw;
    int64_t  encoderTicks;
    int32_t  encoderTicksPerSecond;
    uint8_t  button;
} robot_tx_data_t;

typedef struct __attribute__((packed)) {
    int16_t motor;
    int16_t servo;
    uint8_t neopixel;
} robot_rx_data_t;

static QueueHandle_t my_uart_queue;

void on_robot_data_received(const void *payload, size_t payload_len) {
    if (payload_len == sizeof(robot_rx_data_t)) {
        const robot_rx_data_t *rx_data = (const robot_rx_data_t *)payload;
        printf("Received Command -> Motor: %d, Servo: %d, NeoPixel: %d\n", 
               rx_data->motor, rx_data->servo, rx_data->neopixel);
    } else {
        printf("Warning: Received payload of unexpected size: %d\n", payload_len);
    }
}

void uart_event_task(void *pvParameters) {
    uart_event_t event;
    while(1) {
        if(xQueueReceive(my_uart_queue, &event, portMAX_DELAY)) {
            ht_uart_process_event(UART_NUM_1, &event);
        }
    }
}

void app_main(void) {
    ht_uart_config_t uart_cfg = {
        .uart_num = UART_NUM_1,
        .tx_pin = 17,
        .rx_pin = 16,
        .baud_rate = 115200,
        .rx_buffer_size = 1024,
        .tx_buffer_size = 1024,
        .queue_size = 20
    };

    ESP_ERROR_CHECK(ht_uart_init(&uart_cfg, &my_uart_queue));
    ESP_ERROR_CHECK(ht_uart_set_rx_callback(UART_NUM_1, on_robot_data_received));
    xTaskCreate(uart_event_task, "uart_events", 4096, NULL, 12, NULL);

    robot_tx_data_t sensor_data = {0};

    while(1) {
        sensor_data.vl_data[0] = 150; 
        sensor_data.deltaYaw = -45;
        sensor_data.encoderTicks = 10240;
        sensor_data.encoderTicksPerSecond = 500;
        sensor_data.button = 1;

        ht_uart_send(UART_NUM_1, &sensor_data, sizeof(sensor_data));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}