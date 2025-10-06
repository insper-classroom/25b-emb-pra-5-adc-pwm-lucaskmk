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

#define FILTER_SIZE 32 

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
    int center_diff = (int)raw_adc - ADC_CENTER_VAL;

    if (abs(center_diff) < DEAD_ZONE_THRESHOLD) {
        return 0;
    }

    int sign = (center_diff >= 0) ? 1 : -1;
    int scaled_val = (int)roundf((float)(abs(center_diff) - DEAD_ZONE_THRESHOLD) * SCALE_FACTOR) * sign;

    if (scaled_val > 255) scaled_val = 255;
    if (scaled_val < -255) scaled_val = -255;

    return scaled_val;
}

void x_task(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(10); 
    xLastWakeTime = xTaskGetTickCount();
    
    uint16_t adc_buffer[FILTER_SIZE];
    uint32_t buffer_index = 0;
    
    for (;;) {
        adc_select_input(ADC_X_CHANNEL);
        uint16_t raw_adc = adc_read();

        adc_buffer[buffer_index] = raw_adc;
        buffer_index = (buffer_index + 1) % FILTER_SIZE;

        uint32_t sum = 0;
        for (int i = 0; i < FILTER_SIZE; i++) {
            sum += adc_buffer[i];
        }
        uint16_t filtered_adc = (uint16_t)(sum / FILTER_SIZE);

        adc_t adc_data = {.axis = 0}; 
        adc_data.val = scale_and_apply_dead_zone(filtered_adc);
        
        if (adc_data.val != 0) {
             xQueueSend(xQueueADC, &adc_data, 0);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void y_task(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    xLastWakeTime = xTaskGetTickCount();
    
    uint16_t adc_buffer[FILTER_SIZE];
    uint32_t buffer_index = 0;

    for (;;) {
        adc_select_input(ADC_Y_CHANNEL);
        uint16_t raw_adc = adc_read();
        
        adc_buffer[buffer_index] = raw_adc;
        buffer_index = (buffer_index + 1) % FILTER_SIZE;

        uint32_t sum = 0;
        for (int i = 0; i < FILTER_SIZE; i++) {
            sum += adc_buffer[i];
        }
        uint16_t filtered_adc = (uint16_t)(sum / FILTER_SIZE);

        adc_t adc_data = {.axis = 1}; 
        adc_data.val = scale_and_apply_dead_zone(filtered_adc);
        
        if (adc_data.val != 0) {
            xQueueSend(xQueueADC, &adc_data, 0);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void uart_task(void *pvParameters) {
    adc_t received_data;
    
    for (;;) {
        // A task espera aqui até receber um dado na fila.
        if (xQueueReceive(xQueueADC, &received_data, portMAX_DELAY) == pdPASS) {
            
            int16_t value = (int16_t)received_data.val;
            
            // Extrai os bytes
            uint8_t val_0_lsb = (uint8_t)(value & 0xFF);
            uint8_t val_1_msb = (uint8_t)((value >> 8) & 0xFF);

            // SEQUÊNCIA PARA LITTLE-ENDIAN (LSB primeiro): AXIS VAL_0 VAL_1 EOP
            uart_putc_raw(UART_ID, (uint8_t)received_data.axis);
            
            // ENVIANDO LSB PRIMEIRO para Python interpretar como LITTLE
            uart_putc_raw(UART_ID, val_0_lsb); 
            
            // ENVIANDO MSB SEGUNDO
            uart_putc_raw(UART_ID, val_1_msb);
            
            uart_putc_raw(UART_ID, EOP_BYTE);
            
            // Um pequeno delay para evitar sobrecarga da UART/Python.
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}

int main(void) {
    stdio_init_all();
    
    adc_init_all();
    uart_init_all();
    
    printf("Sistema de Joystick Mouse iniciado.\n");

    xQueueADC = xQueueCreate(40, sizeof(adc_t));

    if (xQueueADC == NULL) {
        printf("Falha ao criar a fila xQueueADC.\n");
        for (;;) { tight_loop_contents(); }
    }
    
    xTaskCreate(x_task, "X_Task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(y_task, "Y_Task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(uart_task, "UART_Task", configMINIMAL_STACK_SIZE, NULL, 2, NULL);

    vTaskStartScheduler();

    for (;;) {
        tight_loop_contents();
    }
}
