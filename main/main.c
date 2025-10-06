#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>
#include <math.h> 

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "pins.h"

#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define ADC_MAX_VAL 4095
#define ADC_CENTER_VAL (ADC_MAX_VAL / 2)
#define SCALE_FACTOR (255.0f / 2047.0f)
#define DEAD_ZONE_THRESHOLD 30

#define FILTER_SIZE 16 
uint16_t x_history[FILTER_SIZE] = {0};
int x_idx = 0;
uint16_t y_history[FILTER_SIZE] = {0};
int y_idx = 0;

typedef struct adc {
    int axis; 
    int val;  
} adc_t;

QueueHandle_t xQueueADC;

void x_task(void *pvParameters);
void y_task(void *pvParameters);
void uart_task(void *pvParameters);

void adc_init_all(void) {
    adc_init();
    adc_gpio_init(ADC_X_PIN); 
    adc_gpio_init(ADC_Y_PIN); 
}

void uart_init_all(void) {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}

int scale_and_apply_dead_zone(uint16_t raw_adc) {
    int centered_val = raw_adc - ADC_CENTER_VAL;
    int final_val = (int)roundf((float)centered_val * SCALE_FACTOR);

    if (final_val >= -DEAD_ZONE_THRESHOLD && final_val <= DEAD_ZONE_THRESHOLD) {
        return 0;
    }

    return final_val;
}

void x_task(void *pvParameters) {
    adc_t adc_data;
    adc_data.axis = 0; 
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); 

    for (;;) {
        adc_select_input(ADC_X_CHANNEL);
        uint16_t raw_adc = adc_read();
        
        x_history[x_idx] = raw_adc;
        x_idx = (x_idx + 1) % FILTER_SIZE;

        uint32_t sum = 0;
        for (int i = 0; i < FILTER_SIZE; i++) {
            sum += x_history[i];
        }
        uint16_t filtered_adc = (uint16_t)(sum / FILTER_SIZE);
        
        adc_data.val = scale_and_apply_dead_zone(filtered_adc);
        xQueueSend(xQueueADC, &adc_data, 0);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void y_task(void *pvParameters) {
    adc_t adc_data;
    adc_data.axis = 1; 

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); 

    for (;;) {
        adc_select_input(ADC_Y_CHANNEL);
        uint16_t raw_adc = adc_read();

        y_history[y_idx] = raw_adc;
        y_idx = (y_idx + 1) % FILTER_SIZE;

        uint32_t sum = 0;
        for (int i = 0; i < FILTER_SIZE; i++) {
            sum += y_history[i];
        }
        uint16_t filtered_adc = (uint16_t)(sum / FILTER_SIZE);

        adc_data.val = scale_and_apply_dead_zone(filtered_adc);
        xQueueSend(xQueueADC, &adc_data, 0);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void uart_task(void *pvParameters) {
    adc_t received_data;
    
    for (;;) {
        if (xQueueReceive(xQueueADC, &received_data, portMAX_DELAY) == pdPASS) {
            
            if (received_data.val != 0) {
                
                int16_t value = (int16_t)received_data.val;
                
                uint8_t val_0_lsb = (uint8_t)(value & 0xFF);
                uint8_t val_1_msb = (uint8_t)((value >> 8) & 0xFF);

                uart_putc_raw(UART_ID, (uint8_t)received_data.axis);
                uart_putc_raw(UART_ID, val_1_msb);
                uart_putc_raw(UART_ID, val_0_lsb);
                uart_putc_raw(UART_ID, EOP_BYTE);
                
                vTaskDelay(pdMS_TO_TICKS(5));
            }
        }
    }
}

int main(void) {
    stdio_init_all();
    
    adc_init_all();
    uart_init_all();
    
    xQueueADC = xQueueCreate(40, sizeof(adc_t));

    if (xQueueADC == NULL) {
        for(;;);
    }

    xTaskCreate(x_task, "X_Task", configMINIMAL_STACK_SIZE, NULL, 2, NULL); 
    xTaskCreate(y_task, "Y_Task", configMINIMAL_STACK_SIZE, NULL, 2, NULL); 
    xTaskCreate(uart_task, "UART_Task", configMINIMAL_STACK_SIZE, NULL, 1, NULL); 

    vTaskStartScheduler();

    for (;;);
}