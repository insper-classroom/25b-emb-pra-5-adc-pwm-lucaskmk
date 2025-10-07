#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "pins.h"

typedef struct adc {
    int axis;
    int val;
} adc_t;

QueueHandle_t xQueueADC;

int map(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void x_task(void *pvParameters) {
    const int DEAD_ZONE = 200;
    const int NUM_SAMPLES = 10;
    int adc_x_readings[NUM_SAMPLES];
    int reading_index = 0;

    for (;;) {
        adc_select_input(ADC_X_CHANNEL);
        adc_x_readings[reading_index] = adc_read();
        reading_index = (reading_index + 1) % NUM_SAMPLES;

        int sum = 0;
        for(int i = 0; i < NUM_SAMPLES; i++) {
            sum += adc_x_readings[i];
        }
        int filtered_value = sum / NUM_SAMPLES;

        int mapped_value = map(filtered_value, 0, 4095, -255, 255);

        if (mapped_value > -DEAD_ZONE && mapped_value < DEAD_ZONE) {
            mapped_value = 0;
        }

        adc_t adc_data = {.axis = 0, .val = mapped_value};
        xQueueSend(xQueueADC, &adc_data, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void y_task(void *pvParameters) {
    const int DEAD_ZONE = 200;
    const int NUM_SAMPLES = 10;
    int adc_y_readings[NUM_SAMPLES];
    int reading_index = 0;

    for (;;) {
        adc_select_input(ADC_Y_CHANNEL);
        adc_y_readings[reading_index] = adc_read();
        reading_index = (reading_index + 1) % NUM_SAMPLES;
        
        int sum = 0;
        for(int i = 0; i < NUM_SAMPLES; i++) {
            sum += adc_y_readings[i];
        }
        int filtered_value = sum / NUM_SAMPLES;

        int mapped_value = map(filtered_value, 0, 4095, -255, 255);

        if (mapped_value > -DEAD_ZONE && mapped_value < DEAD_ZONE) {
            mapped_value = 0;
        }

        adc_t adc_data = {.axis = 1, .val = mapped_value};
        xQueueSend(xQueueADC, &adc_data, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void uart_task(void *pvParameters) {
    adc_t received_data;

    for (;;) {
        if (xQueueReceive(xQueueADC, &received_data, portMAX_DELAY) == pdPASS) {
            uint8_t axis_byte = received_data.axis;
            uint8_t val_lsb = (uint8_t)(received_data.val & 0xFF);
            uint8_t val_msb = (uint8_t)((received_data.val >> 8) & 0xFF);

            uart_putc_raw(uart0, EOP_BYTE);
            uart_putc_raw(uart0, axis_byte);
            uart_putc_raw(uart0, val_lsb);
            uart_putc_raw(uart0, val_msb);
        }
    }
}

int main(void) {
    stdio_init_all();

    adc_init();
    adc_gpio_init(ADC_X_PIN);
    adc_gpio_init(ADC_Y_PIN);

    xQueueADC = xQueueCreate(10, sizeof(adc_t));

    xTaskCreate(x_task, "x_task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(y_task, "y_task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(uart_task, "uart_task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    vTaskStartScheduler();

    for (;;);
    return 0;
}