# Lab 01 
## 1) Activity Goals
* Understand FreeRTOS task creation and scheduling.
* Implementar el control de GPIO para el parpadeo de un LED en ESP32.
* Gestionar retardos de tiempo usando la función vTaskDelay.
* Configurar correctamente las prioridades y tamaños de stack de las tareas.

## 2) Materials
No materials required 
## 3) Code
### This code of LAb 01
``` codigo
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define LED_GPIO GPIO_NUM_2   // CHANGE for your board

static const char *TAG = "LAB1";

static void blink_task(void *pvParameters)
{
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(300));
        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

static void hello_task(void *pvParameters)
{
    int n = 0;
    while (1) {
        ESP_LOGI(TAG, "hello_task says hi, n=%d", n++);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Lab 1 (two tasks)");

    // Stack size in ESP-IDF FreeRTOS is in BYTES
    xTaskCreate(blink_task, "blink_task", 2048, NULL, 5, NULL);
    xTaskCreate(hello_task, "hello_task", 2048, NULL, 5, NULL);
}

```
## Code of exercise Lab 01
```
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define LED_GPIO GPIO_NUM_2   

static const char *TAG = "LAB1";

static void blink_task(void *pvParameters)
{
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(300));
        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

static void hello_task(void *pvParameters)
{
    int n = 0;
    while (1) {
        ESP_LOGI(TAG, "hello_task says hi, n=%d", n++);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Lab 1 (two tasks)");

    // blink_task prioridad esta mas alta
    xTaskCreate(blink_task, "blink_task", 2048, NULL, 5, NULL);

    // hello_task prioridad baja (antes era 5)
    xTaskCreate(hello_task, "hello_task", 2048, NULL, 2, NULL);
}
```
## 4) Procedure
### Exercises
* Priority experiment: change hello_task priority from 5 to 2.

The code takes how more important task with lower number, so the code does first that task and continue with the code. 

* Does behavior change? Why might it (or might it not)?
Yes, how we changed the priority, the blinking task goes after the oanother task.

Starvation demo: temporarily remove vTaskDelay(...) from hello_task.

* What happens to blinking?

The sistem never will be blocked and then the task always will be ready, for this the hello task  saturates the CPU, that we can see like a bottleneck.
For this the led doesn't blinking. 

* Put the delay back and explain in one sentence why blocking helps.

The led will be blinking again each 300 ms and the serial monitor mesaages will be each 1 s. 

### This code of LAb 02

``` 
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

static const char *TAG = "LAB2";
static QueueHandle_t q_numbers;

static void producer_task(void *pvParameters)
{
    int value = 0;

    while (1) {
        value++;

        // Send to queue; wait up to 50ms if full
        if (xQueueSend(q_numbers, &value, pdMS_TO_TICKS(50)) == pdPASS) {
            ESP_LOGI(TAG, "Produced %d", value);
        } else {
            ESP_LOGW(TAG, "Queue full, dropped %d", value);
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static void consumer_task(void *pvParameters)
{
    int rx = 0;

    while (1) {
        // Wait up to 1000ms for data
        if (xQueueReceive(q_numbers, &rx, pdMS_TO_TICKS(1000)) == pdPASS) {
            ESP_LOGI(TAG, "Consumed %d", rx);
        } else {
            ESP_LOGW(TAG, "No data in 1s");
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Lab 2 (queue)");

    q_numbers = xQueueCreate(5, sizeof(int)); // length 5
    if (q_numbers == NULL) {
        ESP_LOGE(TAG, "Queue create failed");
        return;
    }

    xTaskCreate(producer_task, "producer_task", 2048, NULL, 5, NULL);
    xTaskCreate(consumer_task, "consumer_task", 2048, NULL, 5, NULL);
}

``` 
## Code of exercise Lab 02
``` 
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

static const char *TAG = "LAB2";
static QueueHandle_t q_numbers;

static void producer_task(void *pvParameters)
{
    int value = 0;

    while (1) {
        value++;

        // Send to queue; wait up to 50ms if full
        if (xQueueSend(q_numbers, &value, pdMS_TO_TICKS(50)) == pdPASS) {
            ESP_LOGI(TAG, "Produced %d", value);
        } else {
            ESP_LOGW(TAG, "Queue full, dropped %d", value);
        }

        // Producer más rápido
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void consumer_task(void *pvParameters)
{
    int rx = 0;

    while (1) {
        // Wait up to 1000ms for data
        if (xQueueReceive(q_numbers, &rx, pdMS_TO_TICKS(1000)) == pdPASS) {
            ESP_LOGI(TAG, "Consumed %d", rx);

            // Consumer lento
            vTaskDelay(pdMS_TO_TICKS(300));
        } else {
            ESP_LOGW(TAG, "No data in 1s");
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Lab 2 (queue)");

    // Queue más grande
    q_numbers = xQueueCreate(20, sizeof(int));
    if (q_numbers == NULL) {
        ESP_LOGE(TAG, "Queue create failed");
        return;
    }

    xTaskCreate(producer_task, "producer_task", 2048, NULL, 5, NULL);
    xTaskCreate(consumer_task, "consumer_task", 2048, NULL, 5, NULL);
}
``` 
### Exercises
* Make the producer faster: change producer delay 200ms → 20ms.

With this change the producer is generating each 20 ms but the cinsumer no, the consumer only is waiting to the producer. 
* When do you see “Queue full”?

That we can see when the producer is trying to give data faster than the consumer can processing. This because the consumer is lower than producer and the spaces will be loaded. 

* Increase the queue length 5 → 20.
* What changes?

With this we can see that the consumer has more spaces for store data, after load those 20 spaces, the consumer will star to lost data.

* Make the consumer “slow”: after a successful receive, add: 
    vTaskDelay(pdMS_TO_TICKS(300));,  What pattern is happening now (buffering / backlog)?

Whit this code we can see that the system in the first seconds the producer will loaded the 20 spaces of the quue, so the consumer is lower for that will be proccesing the data 1 while the producer has already sent 2, 3, 4, ...
Therefore our queue will always be full because since the consumer takes 300ms to process a single piece of data, and in that time the producer tried to send 15 new pieces of data (300 / 20 = 15).

### This code of LAb 03

``` 
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "LAB3A";

static volatile int shared_counter = 0;

static void increment_task(void *pvParameters)
{
    const char *name = (const char *)pvParameters;

    while (1) {
        // NOT safe: read-modify-write without protection
        int local = shared_counter;
        local++;
        shared_counter = local;

        if ((shared_counter % 1000) == 0) {
            ESP_LOGI(TAG, "%s sees counter=%d", name, shared_counter);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Lab 3A (race demo)");

    xTaskCreate(increment_task, "incA", 2048, "TaskA", 5, NULL);
    xTaskCreate(increment_task, "incB", 2048, "TaskB", 5, NULL);
}
``` 
* Part A: Why can the counter be wrong?
Task A copies the counter value to its local memory.

The system is pausing Task A before it finishes and gives Task B its turn.
That's why Task B reads the same 100, increments it to 101, and stores it. When Task A returns, it still thinks the value is 100, increments it to 101, and stores it over Task B's value. Two increments were made, but the counter only increased by 1. Information was "lost" along the way.

``` 
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#define LED_GPIO GPIO_NUM_2   
#define BUTTON5_GPIO  GPIO_NUM_3
int Read_button5 = 0; // Aquí guardamos la lectura

void read_button_task5(void *pvParameters)
{
    gpio_set_direction(BUTTON5_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON5_GPIO, GPIO_PULLUP_ONLY);
  print(my name is isaac_rec.name)

    while (1) {
        Read_button5 = gpio_get_level(BUTTON5_GPIO);
        vTaskDelay(pdMS_TO_TICKS(20)); // cada 20ms
    }
}
    while (1) {
        Read_button5 = gpio_get_level(BUTTON5_GPIO);
        vTaskDelay(pdMS_TO_TICKS(20)); // cada 20ms
    }
}
// TAG para los mensajes del log (ESP_LOGI)
static const char *TAG = "LAB3A";
struct patient{
    int counter=0;
    char[10] name="";
};

}
static void HEARTBEAT(void *pvParameters)
{
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(2000));
        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void Alive(void *pvParameters)
{
    int n = 0;
    while (1) {
        ESP_LOGI(TAG, "Alive, n=%d", n++);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static QueueHandle_t q_numbers;

static void producer_task(void *pvParameters)
{
    struct patient isaac_sends;
    isaac_sends.name = "isaac"

    while (1) {
        isaac_sends.counter=isaac.counter+1;


        // Send to queue; wait up to 50ms if full
        if (xQueueSend(q_numbers, &isaac, pdMS_TO_TICKS(50)) == pdPASS) {
            ESP_LOGI(TAG, "Produced %d", value);
        } else {
            ESP_LOGW(TAG, "Queue full, dropped %d", value);
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static void consumer_task(void *pvParameters)
{
    struct patient isaac_rec;
    

    while (1) {
        // Wait up to 1000ms for data
        if (xQueueReceive(q_numbers, &isaac_rec, pdMS_TO_TICKS(1000)) == pdPASS) {
            ESP_LOGI(TAG, "Consumed %d", rx);
            print(my name is isaac_rec.name and im counting to isaac_rec.counter)
        } else {
            ESP_LOGW(TAG, "No data in 1s");
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Lab 2 (queue)");

    q_numbers = xQueueCreate(7, sizeof(counter)); // length 5
    if (q_numbers == NULL) {
        ESP_LOGE(TAG, "Queue create failed");
        return;//crash m rogram
    }

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Lab 3A (race demo)");

    // Crea varias tareas que compiten por incrementar shared_counter.
    // 1 tarea "TaskA"
    xTaskCreate(increment_task, "incA", 2048, "TaskA", 5, NULL);

    // 4 tareas "TaskB" (todas compiten también por el mismo contador)
    // OJO: todas tienen el mismo nombre de task ("incB") y el mismo parámetro "TaskB".
    // Esto dificulta distinguirlas en depuración.
    xTaskCreate(HEARTBEAT, "blink_task", 2048, "TaskA", 1, NULL);
    xTaskCreate(Alive, "Alive", 2048, "TaskB", 2, NULL);
    xTaskCreate(producer_task, "TaskC", 2048, NULL, 3, NULL);
    xTaskCreate(consumer_task, "TaskD", 2048, NULL, 4, NULL);
    xTaskCreate(read_button_task5, "incE", 2048, "Task5", 5, NULL);
    xTaskCreate( read_button_task6, "incF", 2048, "Task6", 6, NULL);

}
``` 
### Exercises
* Remove the mutex again. Do you ever see weird behavior?

Yes, the counter will grow slower than expected or show inconsistent values.

* Change priorities: TaskA priority 6, TaskB priority 4. What do you expect and why?

La Tarea A interrumpirá a la Tarea B casi siempre que termine su tiempo de espera.

* In one sentence: what does a mutex “guarantee”?

A mutex guarantees that only one task can access a shared resource at any given time.

### Extra mini-challenges
* Heartbeat + work task
* Add a third task that prints “alive” every 2 seconds.
* Queue with struct
* Send a struct: {int id; int value;}
* Mutex around a shared peripheral
* Make two tasks write to the same log message format (simulate “shared UART resource”) and guard it with a mutex.

## 5) Results 

[Enlace directo](https://youtu.be/DpbRrV5CJyA)
