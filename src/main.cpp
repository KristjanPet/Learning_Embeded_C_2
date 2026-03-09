#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "blink_lib.hpp"

extern "C" void app_main(void)
{
    blink_lib::init(2);

    while (true) {
        blink_lib::toggle();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}