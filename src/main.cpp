#include <stdio.h>
#include <vector>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <cstring>

static const char* TAG = "mem_demo";

static int bad_return(int x) {
    if (x > 0) return 1;
    // missing return on purpose
}

extern "C" void app_main(void)
{

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}