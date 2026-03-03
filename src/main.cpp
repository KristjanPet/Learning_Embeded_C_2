#include <stdio.h>
#include <vector>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <cstring>

static const char* TAG = "mem_demo";

static void report_mem(const char* where){
    size_t free8 = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    size_t min8 = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
    size_t free32 = heap_caps_get_free_size(MALLOC_CAP_32BIT);

    ESP_LOGI(TAG, "[%s] heap free(8bit)=%u, min_free(8bit)=%u, free(32bit)=%u",
             where, (unsigned)free8, (unsigned)min8, (unsigned)free32);
}

static void worker_task(void*){
    report_mem("Worker start");

    //tracking stack usage
    UBaseType_t hw0 = uxTaskGetStackHighWaterMark(nullptr);
    ESP_LOGI(TAG, "worker stack high-water: %u words (~%u bytes)",
         (unsigned)hw0, (unsigned)hw0 * (unsigned)sizeof(StackType_t));


    uint8_t scratch[1200];
    memset(scratch, 0xA5, sizeof(scratch));
    ESP_LOGI(TAG, "scratch=%p", scratch);
    // alocate and free varying sizes to mimic a fregmentation-risk patteren
    std::vector<void*> blocks;
    blocks.reserve(200);

    for(int i=0; i<5; i++){
        //alcate mix sizes
        for(int j = 0; j < 200; j++){
            size_t sz = (j % 3 == 0) ? 64 : (j % 3 == 1) ? 256 : 1024;
            void* p = malloc(sz);
            if(!p){
                ESP_LOGE(TAG, "malloc failed at round=%d i=%d", i, j);
                break;
            }
            blocks.push_back(p);
        }

        //free every other block to create fragmentation risk
        for(size_t j = 0; j < blocks.size(); j += 2){
            free(blocks[j]);
            blocks[j] = nullptr;
        }

        report_mem("after alloc/free holes");

        // trying bigger allocations to see if fragmentation causes failures
        void* big = malloc(20 * 1024);
        ESP_LOGI(TAG, "big alloc (20KB) => %s", big ? "OK" : "FAILED");
        free(big);

        // cleanup remaining blocks
        for (void*& p : blocks) {
            if (p) free(p);
            p = nullptr;
        }
        blocks.clear();

        report_mem("after cleanup");
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    UBaseType_t hw1 = uxTaskGetStackHighWaterMark(nullptr);
    ESP_LOGI(TAG, "worker stack high-water at end: %u words (~%u bytes)",
             (unsigned)hw1, (unsigned)hw1 * (unsigned)sizeof(StackType_t));

    vTaskDelete(nullptr);
}

extern "C" void app_main(void)
{
    report_mem("boot");

    xTaskCreatePinnedToCore(worker_task, "worker", 2048, nullptr, 10, nullptr, 1);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        report_mem("main loop");
    }
}