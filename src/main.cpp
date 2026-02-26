#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_log.h"

static const char* TAG = "gptimer_blink";

static TaskHandle_t s_blink_task = nullptr;
static constexpr gpio_num_t LED_GPIO = GPIO_NUM_2;

static void blink_task(void* arg){
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);

    while(true){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int level = gpio_get_level(LED_GPIO);
        gpio_set_level(LED_GPIO, !level);
    }
}

extern "C" void app_main(void) {
    xTaskCreatePinnedToCore(blink_task, "blink", 2048, nullptr, 10, &s_blink_task, 1);

    
}