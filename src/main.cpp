#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_sleep.h"

static const char* TAG = "sleep_demo";

static constexpr gpio_num_t WAKE_GPIO = GPIO_NUM_33;

extern "C" void app_main(void)
{
    // Configure wake pin as input with pull-up
    gpio_config_t io{};
    io.pin_bit_mask = 1ULL << WAKE_GPIO;
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io));

    // Report wake reason
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause == ESP_SLEEP_WAKEUP_EXT0) {
        ESP_LOGI(TAG, "Woke up from EXT0 (button on GPIO33)");
    } else if (cause == ESP_SLEEP_WAKEUP_UNDEFINED) {
        ESP_LOGI(TAG, "Power-on reset / normal boot");
    } else {
        ESP_LOGI(TAG, "Wakeup cause: %d", (int)cause);
    }

    ESP_LOGI(TAG, "Going to deep sleep in 3 seconds. Press button to wake.");
    vTaskDelay(pdMS_TO_TICKS(3000));

    // EXT0 uses a single RTC IO pin, level-based wake.
    // Wake when pin is LOW (button pressed).
    ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(WAKE_GPIO, 0));

    ESP_LOGI(TAG, "Entering deep sleep now.");
    esp_deep_sleep_start();
}