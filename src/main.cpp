#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_log.h"

static const char* TAG = "gptimer_blink";

static TaskHandle_t s_blink_task = nullptr;
static constexpr gpio_num_t LED_GPIO = GPIO_NUM_2;

static bool IRAM_ATTR on_timer_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx){
    BaseType_t higher_woken = pdFALSE;
    if(s_blink_task){
        vTaskNotifyGiveFromISR(s_blink_task, &higher_woken);
    }
    return higher_woken == pdTRUE;
}

static void blink_task(void* arg){
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    int level = 0;
    gpio_set_level(LED_GPIO, level);

    while(true){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        level = !level;
        gpio_set_level(LED_GPIO, level);
    }
}

extern "C" void app_main(void) {
    xTaskCreatePinnedToCore(blink_task, "blink", 2048, nullptr, 10, &s_blink_task, 1);

    gptimer_handle_t timer = nullptr;
    gptimer_config_t tcfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000 // 1MHz -> 1 tick = us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&tcfg, &timer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = on_timer_alarm,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &cbs, nullptr));

    //alarm every 500ms
    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = 1000,
        .reload_count = 0,
        .flags = {
            .auto_reload_on_alarm = true,
        },
    };

    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm_cfg));

    ESP_ERROR_CHECK(gptimer_enable(timer));
    ESP_ERROR_CHECK(gptimer_start(timer));

    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Started gptimer blink on GPIO %d", (int)LED_GPIO);
}