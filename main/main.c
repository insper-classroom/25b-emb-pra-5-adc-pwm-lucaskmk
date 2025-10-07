#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "pins.h"

#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define EOP_BYTE 0xFF

#define FILTER_DEPTH 12
#define ADC_CENTER 2048
#define ADC_TOLERANCE 150
#define MAX_SPEED 10
#define READ_DELAY_MS 25

typedef struct {
    uint8_t axis;
    int16_t value;
} axis_msg_t;

static QueueHandle_t qAxisData;

static int16_t process_adc(uint16_t raw, uint16_t history[], uint8_t *idx, uint32_t *acc) {
    *acc -= history[*idx];
    history[*idx] = raw;
    *acc += raw;
    *idx = (*idx + 1) % FILTER_DEPTH;
    uint16_t mean = *acc / FILTER_DEPTH;

    int16_t result = 0;
    if (mean > ADC_CENTER + ADC_TOLERANCE) {
        result = (int16_t)((mean - (ADC_CENTER + ADC_TOLERANCE)) * MAX_SPEED / (4095 - (ADC_CENTER + ADC_TOLERANCE)));
    } else if (mean < ADC_CENTER - ADC_TOLERANCE) {
        result = (int16_t)((mean - (ADC_CENTER - ADC_TOLERANCE)) * MAX_SPEED / (ADC_CENTER - ADC_TOLERANCE));
    }
    if (result > MAX_SPEED) result = MAX_SPEED;
    if (result < -MAX_SPEED) result = -MAX_SPEED;
    return result;
}

static void joystick_task(void *p) {
    uint8_t axis_id = (uint8_t)(uintptr_t)p;
    adc_select_input(axis_id);
    uint16_t buffer[FILTER_DEPTH] = {0};
    uint8_t idx = 0;
    uint32_t acc = 0;

    for (int i = 0; i < FILTER_DEPTH; i++) {
        adc_select_input(axis_id);
        uint16_t val = adc_read();
        buffer[i] = val;
        acc += val;
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    while (1) {
        adc_select_input(axis_id);
        uint16_t raw = adc_read();
        int16_t speed = process_adc(raw, buffer, &idx, &acc);
        if (speed != 0) {
            axis_msg_t msg = { .axis = axis_id, .value = speed };
            xQueueSend(qAxisData, &msg, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(READ_DELAY_MS));
    }
}

static void uart_sender_task(void *p) {
    axis_msg_t msg;
    while (1) {
        if (xQueueReceive(qAxisData, &msg, portMAX_DELAY) == pdPASS) {
            uint8_t lsb = msg.value & 0xFF;
            uint8_t msb = (msg.value >> 8) & 0xFF;
            uart_putc_raw(UART_ID, 0xFF);
            uart_putc_raw(UART_ID, msg.axis);
            uart_putc_raw(UART_ID, lsb);
            uart_putc_raw(UART_ID, msb);
        }
    }
}

int main(void) {
    stdio_init_all();
    adc_init();
    adc_gpio_init(ADC_X_PIN);
    adc_gpio_init(ADC_Y_PIN);

    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    qAxisData = xQueueCreate(16, sizeof(axis_msg_t));
    if (!qAxisData) while (1);

    xTaskCreate(joystick_task, "X_Axis", 256, (void *)0, 1, NULL);
    xTaskCreate(joystick_task, "Y_Axis", 256, (void *)1, 1, NULL);
    xTaskCreate(uart_sender_task, "UART", 256, NULL, 1, NULL);

    vTaskStartScheduler();
    for (;;);
    return 0;
}
