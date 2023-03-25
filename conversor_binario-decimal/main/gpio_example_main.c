/* GPIO Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

/* Pines de entrada */
#define BIT_IN_0    2
#define BIT_IN_1    4
#define BIT_IN_2    5
#define BIT_IN_3    12
#define BIT_IN_4    13
#define BIT_IN_5    14
#define BIT_IN_6    21
#define BIT_IN_7    22

/* Bit mask de los pines de entrada*/
#define BIT_IN_PIN_SEL_1    ((1ULL<<BIT_IN_0) | (1ULL<<BIT_IN_1) | (1ULL<<BIT_IN_2) | (1ULL<<BIT_IN_3))
#define BIT_IN_PIN_SEL_2    ((1ULL<<BIT_IN_4) | (1ULL<<BIT_IN_5) | (1ULL<<BIT_IN_6) | (1ULL<<BIT_IN_7))
#define BIT_IN_PIN_SEL      (BIT_IN_PIN_SEL_1 | BIT_IN_PIN_SEL_2)

#define ESP_INTR_FLAG_DEFAULT 0

static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

/**
 * Tarea GPIO Convertir:
 * Cada vez que se genera una interrupcion, lee los pines de entrada
 * y hace la conversion a decimal.
 */
static void gpio_task_convertir(void* arg)
{
    uint32_t io_num;
    uint32_t valor_convertido;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            valor_convertido = 0;
            if (gpio_get_level(BIT_IN_0) == 1)
                valor_convertido += 1;
            if (gpio_get_level(BIT_IN_1) == 1)
                valor_convertido += 2;
            if (gpio_get_level(BIT_IN_2) == 1)
                valor_convertido += 4;
            if (gpio_get_level(BIT_IN_3) == 1)
                valor_convertido += 8;
            if (gpio_get_level(BIT_IN_4) == 1)
                valor_convertido += 16;
            if (gpio_get_level(BIT_IN_5) == 1)
                valor_convertido += 32;
            if (gpio_get_level(BIT_IN_6) == 1)
                valor_convertido += 64;
            if (gpio_get_level(BIT_IN_7) == 1)
                valor_convertido += 128;
            printf("Valor en decimal: %"PRIu32"\n", valor_convertido);
        }
    }
}

void app_main(void)
{
    // Inicializar a cero la estructura de configuración.
    gpio_config_t io_conf = {};

    // Tipo de interrupcion GPIO
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    // Bit mask de los pines
    io_conf.pin_bit_mask = BIT_IN_PIN_SEL;
    // Modo GPIO
    io_conf.mode = GPIO_MODE_INPUT;
    // Activar modo pull-up
    io_conf.pull_up_en = 1;

    // Configurar GPIO con las configuraciones dadas
    gpio_config(&io_conf);

    // Crear una cola para manejar el evento GPIO de ISR
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    // Iniciar la tarea GPIO
    xTaskCreate(gpio_task_convertir, "gpio_task_convertir", 2048, NULL, 10, NULL);

    // Instalar el servicio GPIO ISR
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    /* Asignar manejador ISR para un pin GPIO */
    gpio_isr_handler_add(BIT_IN_0, gpio_isr_handler, (void*) BIT_IN_0);
    gpio_isr_handler_add(BIT_IN_1, gpio_isr_handler, (void*) BIT_IN_1);
    gpio_isr_handler_add(BIT_IN_2, gpio_isr_handler, (void*) BIT_IN_2);
    gpio_isr_handler_add(BIT_IN_3, gpio_isr_handler, (void*) BIT_IN_3);
    gpio_isr_handler_add(BIT_IN_4, gpio_isr_handler, (void*) BIT_IN_4);
    gpio_isr_handler_add(BIT_IN_5, gpio_isr_handler, (void*) BIT_IN_5);
    gpio_isr_handler_add(BIT_IN_6, gpio_isr_handler, (void*) BIT_IN_6);
    gpio_isr_handler_add(BIT_IN_7, gpio_isr_handler, (void*) BIT_IN_7);

    printf("Minimum free heap size: %"PRIu32" bytes\n", esp_get_minimum_free_heap_size());

    int cnt = 0;
    while(1) {
        printf("cnt: %d\n", cnt++);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
