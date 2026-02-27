#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <esp_timer.h>

static const char* TAG = "spi_dma_sd";

static constexpr int PIN_MOSI = 23;
static constexpr int PIN_MISO = 19;
static constexpr int PIN_SCK  = 18;
static constexpr int PIN_CS   = 5;

static spi_device_handle_t sd_dev = nullptr;

static esp_err_t spi_txrx(const uint8_t* tx, uint8_t* rx, size_t len){
    spi_transaction_t t{};
    t.length = len * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    return spi_device_transmit(sd_dev, &t);
}

extern "C" void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "Init SPI bus...");

    spi_bus_config_t buscfg{};
    buscfg.mosi_io_num = PIN_MOSI;
    buscfg.miso_io_num = PIN_MISO;
    buscfg.sclk_io_num = PIN_SCK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    // allow DMA
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg{};
    devcfg.clock_speed_hz = 10 * 1000 * 1000;       // 10MHz for SD init
    devcfg.mode = 0;                      // SPI mode 0
    devcfg.spics_io_num = PIN_CS;
    devcfg.queue_size = 1;

    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &sd_dev));

    // DMA-capable buffers
    const size_t N = 16 * 1024;
    auto* tx = (uint8_t*)heap_caps_malloc(N, MALLOC_CAP_DMA);
    auto* rx = (uint8_t*)heap_caps_malloc(N, MALLOC_CAP_DMA);
    assert(tx && rx);
    static constexpr int ITER = 200;
    const size_t chunk = 2048;

    // 80 dummy clocks: send at least 10 bytes of 0xFF with CS high.
    // Easiest: temporarily set CS as GPIO output and drive high, then send 0xFFs.
    gpio_set_direction((gpio_num_t)PIN_CS, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)PIN_CS, 1);

    memset(tx, 0xFF, N);
    memset(rx, 0x00, N);

    int64_t t0 = esp_timer_get_time();
    // CS high dummy clocks
    for (int i = 0; i < ITER; i++) {
        for(int j = 0; j < N; j += chunk){
            ESP_ERROR_CHECK(spi_txrx(tx + j, rx + j, chunk));
        }
    }

    int64_t t1 = esp_timer_get_time();
    ESP_LOGI(TAG, "Transferred %d x %u bytes in %lld us (%.2f MB/s)", ITER, (unsigned)N, (long long)(t1 - t0), (double)(ITER * N) / (double)(t1 - t0)); // bytes/us == MB/s approx

    // Now give CS back to SPI peripheral by re-adding device with spics_io_num,
    // simplest: just set CS low manually for CMD0 phase and do manual-CS transfers later.
    // For this lesson, we keep manual CS to inspect raw behavior.
    gpio_set_level((gpio_num_t)PIN_CS, 0);

    // CMD0 frame: 0x40 00 00 00 00 95 + then read response bytes
    uint8_t cmd0[6] = {0x40,0x00,0x00,0x00,0x00,0x95};

    // Send CMD0
    ESP_ERROR_CHECK(spi_txrx(cmd0, rx, sizeof(cmd0)));

    // Read 10 bytes response by sending 0xFF
    memset(tx, 0xFF, N);
    ESP_ERROR_CHECK(spi_txrx(tx, rx, 10));

    ESP_LOGI(TAG, "CMD0 response bytes:");
    for (int i = 0; i < 10; i++) {
        ESP_LOGI(TAG, "  [%d] 0x%02X", i, rx[i]);
    }

    // Release CS
    gpio_set_level((gpio_num_t)PIN_CS, 1);

    while (true) vTaskDelay(pdMS_TO_TICKS(1000));
}