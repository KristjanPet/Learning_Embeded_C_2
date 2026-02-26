#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

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
    ESP_LOGI(TAG, "Init SPI bus...");

    spi_bus_config_t buscfg{};
    buscfg.mosi_io_num = PIN_MOSI;
    buscfg.miso_io_num = PIN_MISO;
    buscfg.sclk_io_num = PIN_SCK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    // allow DMA
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    
}