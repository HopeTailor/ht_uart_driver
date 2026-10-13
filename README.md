# HT-UART Component for ESP32 (ESP-IDF v5)

A robust, high-performance, and decoupled UART communication library designed for embedded robotics systems using ESP-IDF v5. It implements reliable packet framing and error-checking without dictating system architecture or forcing internal FreeRTOS tasks.

---

## Features

- **Robust Framing (COBS):** Uses Consistent Overhead Byte Stuffing (COBS) to safely transmit binary payloads containing arbitrary byte values (including `0x00`) using zero-byte packet delimiters.
- **Data Integrity (CRC16):** Implements CRC16-CCITT-False validation to guarantee that corrupted or truncated packets are automatically dropped.
- **Zero Heap Fragmentation:** Utilizes fixed-size static buffers and `__attribute__((packed))` data structures, avoiding dynamic memory allocation during runtime.
- **Decoupled Architecture:** Does not hardcode internal FreeRTOS tasks. Instead, it exposes the native event queue so developers have full control over task management, core pinning, and priorities.
- **Multi-Port Support:** Thread-safe array mapping supporting concurrent multi-port UART operations (`UART_NUM_0`, `UART_NUM_1`, etc.).

---

## Directory Structure

Place this component inside your project's `components` directory:

```text
your_project_root/
├── components/
│   └── ht_uart/
│       ├── CMakeLists.txt
│       ├── include/
│       │   └── ht_uart.h
│       └── src/
│           └── ht_uart.c
├── main/
│   └── main.c
└── CMakeLists.txt
```

---

## Quick Start Guide

### Initialization and Setup (`main.c`)

```c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "ht_uart.h"

static QueueHandle_t my_uart_queue;

// Callback triggered upon receiving a valid, CRC-verified packet
void on_data_received(const void *payload, size_t payload_len) {
    // Cast payload to your custom receiver struct
    // e.g., const my_rx_struct_t *data = payload;
}

// Dedicated event task to process incoming queue items
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

    // Initialize driver and bind queue
    ESP_ERROR_CHECK(ht_uart_init(&uart_cfg, &my_uart_queue));
    
    // Register callback
    ESP_ERROR_CHECK(ht_uart_set_rx_callback(UART_NUM_1, on_data_received));
    
    // Create background management task
    xTaskCreate(uart_event_task, "uart_events", 4096, NULL, 12, NULL);

    while(1) {
        // Transmit your custom packed payload safely
        // ht_uart_send(UART_NUM_1, &my_tx_struct, sizeof(my_tx_struct));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```