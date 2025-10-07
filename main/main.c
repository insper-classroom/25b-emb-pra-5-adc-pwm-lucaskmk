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
#define EOP_BYTE 0xFF

#define NUM_SAMPLES        128   // filtro mais pesado
#define READ_MS            600   // lê o ADC a cada 600 ms
#define DEAD_ZONE          400   // zona morta maior
#define TARGET_DELTA       20    // ignora pequenas variações
#define ZERO_HOLD_CNT      6     // só zera após 6 leituras seguidas estáveis
#define RAMP_STEP          1     // movimento extremamente suave
#define SEND_INTERVAL_MS   300   // envia a cada 300 ms

typedef struct {
    uint8_t axis;
    int16_t target;
} target_msg_t;

static QueueHandle_t xQueueTargets = NULL;

static inline int map_int(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

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

void axis_reader_loop(int adc_channel, int axis_id) {
    uint16_t samples[NUM_SAMPLES];
    int idx = 0;
    int zero_hold = 0;
    int16_t last_local_target = 0;
    const TickType_t period = pdMS_TO_TICKS(READ_MS);
    for (int i = 0; i < NUM_SAMPLES; ++i) {
        adc_select_input(adc_channel);
        samples[i] = adc_read();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    for (;;) {
        adc_select_input(adc_channel);
        uint16_t raw = adc_read();
        samples[idx] = raw;
        idx = (idx + 1) % NUM_SAMPLES;
        uint32_t sum = 0;
        for (int i = 0; i < NUM_SAMPLES; ++i) sum += samples[i];
        int filtered = (int)(sum / NUM_SAMPLES);
        int mapped = map_int(filtered, 0, 4095, -255, 255);
        if (mapped > -DEAD_ZONE && mapped < DEAD_ZONE) {
            zero_hold++;
            if (zero_hold >= ZERO_HOLD_CNT) mapped = 0;
            else mapped = last_local_target;
        } else {
            zero_hold = 0;
        }
        if (abs(mapped - last_local_target) >= TARGET_DELTA) {
            target_msg_t m;
            m.axis = (uint8_t)axis_id;
            m.target = (int16_t)mapped;
            xQueueSend(xQueueTargets, &m, 0);
            last_local_target = m.target;
        }
        vTaskDelay(period);
    }
}

void x_task(void *pv) { (void)pv; axis_reader_loop(ADC_X_CHANNEL, 0); }
void y_task(void *pv) { (void)pv; axis_reader_loop(ADC_Y_CHANNEL, 1); }

static void send_packet_msb_first(uint8_t axis, int16_t value) {
    uint8_t msb = (uint8_t)((value >> 8) & 0xFF);
    uint8_t lsb = (uint8_t)(value & 0xFF);
    uart_putc_raw(UART_ID, axis);
    uart_putc_raw(UART_ID, msb);
    uart_putc_raw(UART_ID, lsb);
    uart_putc_raw(UART_ID, EOP_BYTE);
}

void uart_ramp_task(void *pvParameters) {
    (void)pvParameters;
    int16_t target_vals[2] = {0,0};
    int16_t current_vals[2] = {0,0};
    int16_t last_sent[2] = {32767,32767};
    const TickType_t period = pdMS_TO_TICKS(SEND_INTERVAL_MS);
    target_msg_t m;
    for (;;) {
        while (xQueueReceive(xQueueTargets, &m, 0) == pdPASS) {
            if (m.axis < 2) target_vals[m.axis] = m.target;
        }
        for (int axis = 0; axis < 2; ++axis) {
            int16_t tgt = target_vals[axis];
            int16_t cur = current_vals[axis];
            if (tgt != cur) {
                int delta = tgt - cur;
                int step = delta;
                if (abs(delta) > RAMP_STEP) step = (delta > 0) ? RAMP_STEP : -RAMP_STEP;
                cur = cur + step;
                current_vals[axis] = cur;
                if (cur != last_sent[axis]) {
                    send_packet_msb_first((uint8_t)axis, cur);
                    last_sent[axis] = cur;
                }
            } else {
                if (cur != last_sent[axis]) {
                    send_packet_msb_first((uint8_t)axis, cur);
                    last_sent[axis] = cur;
                }
            }
        }
        vTaskDelay(period);
    }
}

int main(void) {
    stdio_init_all();
    adc_init_all();
    uart_init_all();
    xQueueTargets = xQueueCreate(8, sizeof(target_msg_t));
    if (xQueueTargets == NULL) for (;;);
    xTaskCreate(x_task, "x_task", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    xTaskCreate(y_task, "y_task", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    xTaskCreate(uart_ramp_task, "uart_ramp", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    vTaskStartScheduler();
    for (;;);
    return 0;
}
