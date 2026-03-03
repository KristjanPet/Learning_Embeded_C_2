#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_private/esp_clk.h"

static const char* TAG = "clock_demo";

static TaskHandle_t s_timer_task = nullptr;
static esp_pm_lock_handle_t s_cpu_lock = nullptr;

static constexpr gpio_num_t GPIO_TIMER = GPIO_NUM_2; // stable
static constexpr gpio_num_t GPIO_BUSY  = GPIO_NUM_4; // changes with CPU freq

static bool IRAM_ATTR on_alarm(gptimer_handle_t,
                              const gptimer_alarm_event_data_t*,
                              void*){
    BaseType_t hp = pdFALSE;
    vTaskNotifyGiveFromISR(s_timer_task, &hp);
    return hp == pdTRUE;
}

static void pm_lock_task(void*){
    while (true) {
        ESP_LOGI(TAG, "LOCK max freq");
        ESP_ERROR_CHECK(esp_pm_lock_acquire(s_cpu_lock));
        ESP_LOGI(TAG, "CPU freq now: %u Hz", (unsigned)esp_clk_cpu_freq());
        vTaskDelay(pdMS_TO_TICKS(3000));

        ESP_LOGI(TAG, "UNLOCK (allow scaling)");
        ESP_ERROR_CHECK(esp_pm_lock_release(s_cpu_lock));
        ESP_LOGI(TAG, "CPU freq now: %u Hz", (unsigned)esp_clk_cpu_freq());
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

static void timer_toggle_task(void*){
    gpio_set_direction(GPIO_TIMER, GPIO_MODE_OUTPUT);
    int level = 0;
    gpio_set_level(GPIO_TIMER, level);

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        level = !level;
        gpio_set_level(GPIO_TIMER, level);
    }
}

// Intentionally bad: busy-wait based timing
static void busy_toggle_task(void*)
{
    gpio_set_direction(GPIO_BUSY, GPIO_MODE_OUTPUT);
    int level = 0;
    gpio_set_level(GPIO_BUSY, level);

    while (true) {
        // crude busy loop: period depends on CPU frequency and compiler optimizations
        for (volatile int i = 0; i < 200000; i++) { }
        vTaskDelay(pdMS_TO_TICKS(10));
        level = !level;
        gpio_set_level(GPIO_BUSY, level);
    }
}

extern "C" void app_main(void)
{
    // Allow dynamic frequency scaling (we'll change it via power management config)
    esp_pm_config_t pm = {
        .max_freq_mhz = 240,
        .min_freq_mhz = 80,
        .light_sleep_enable = false,
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm));
    ESP_LOGI(TAG, "PM configured: min 80 MHz, max 240 MHz");

    ESP_ERROR_CHECK(esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "maxfreq", &s_cpu_lock));
    xTaskCreatePinnedToCore(pm_lock_task, "pm_lock", 2048, nullptr, 8, nullptr, 0);
    xTaskCreatePinnedToCore(timer_toggle_task, "timer_tog", 2048, nullptr, 10, &s_timer_task, 1);
    xTaskCreatePinnedToCore(busy_toggle_task,  "busy_tog",  2048, nullptr, 5,  nullptr,        1);

    gptimer_handle_t timer = nullptr;
    gptimer_config_t tcfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1'000'000,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&tcfg, &timer));

    gptimer_event_callbacks_t cbs = {.on_alarm = on_alarm};
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &cbs, nullptr));

    // Toggle notify every 1ms -> GPIO2 becomes 500 Hz square wave
    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = 1000,
        .reload_count = 0,
        .flags = {.auto_reload_on_alarm = true},
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm_cfg));

    ESP_ERROR_CHECK(gptimer_enable(timer));
    ESP_ERROR_CHECK(gptimer_start(timer));

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        ESP_LOGI(TAG, "Running... (CPU may scale between 80 and 240 depending on load)");
    }
}