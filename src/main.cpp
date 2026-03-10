#include <stdint.h>
#include <string.h>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

struct RxFrame {
    uint8_t type;
    std::vector<uint8_t> payload;
};

static const char* TAG = "rs485_lab";

static QueueHandle_t g_ack_q = nullptr;

// UART1 (Module A)
static constexpr uart_port_t U_TX = UART_NUM_1;
static constexpr int U_TX_PIN = 17;
static constexpr int U_RX_PIN = 16;
static constexpr int U_DE_PIN  = 4;

// UART2 (Module B)
static constexpr uart_port_t U2_TX = UART_NUM_2;
static constexpr int U2_TX_PIN = 25;
static constexpr int U2_RX_PIN = 26;
static constexpr int U2_DE_PIN  = 5;

static constexpr int BAUD = 115200;

static uint32_t read_u32_le(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static void write_u32_le(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void rs485_set_tx(int de_gpio) {
    gpio_set_level((gpio_num_t)de_gpio, 1); // DE=1
}

static void rs485_set_rx(int de_gpio) {
    gpio_set_level((gpio_num_t)de_gpio, 0); // DE=0
}

// ---------- UART init ----------
static void uart_init_port(uart_port_t port, int tx, int rx) {
    uart_config_t cfg{};
    cfg.baud_rate = BAUD;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_param_config(port, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(port, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(port, 2048, 2048, 0, nullptr, 0));
}

static bool parse_stream_byte(uint8_t b, RxFrame& out, std::vector<uint8_t>& buf, int64_t& last_byte_us) {
    int64_t now = esp_timer_get_time();

    // Inter-byte timeout: reset parser if too long between bytes
    if (last_byte_us != 0 && (now - last_byte_us) > 20'000) {
        buf.clear();
    }
    last_byte_us = now;

    buf.push_back(b);

    // Keep buffer from growing forever
    if (buf.size() > 512) buf.clear();

    // Find SOF AA55 at start; if not aligned, slide
    while (buf.size() >= 2) {
        if (buf[0] == 0xAA && buf[1] == 0x55) break;
        buf.erase(buf.begin());
    }
    if (buf.size() < 4) return false; // need SOF + LEN + TYPE at least

    uint8_t len = buf[2]; // TYPE+PAYLOAD length
    size_t total = 2 + 1 + len + 2; // SOF(2) + LEN(1) + (TYPE+PAYLOAD=len) + CRC(2)
    if (len < 1 || total > 512) { buf.clear(); return false; }

    if (buf.size() < total) return false; // wait more

    // We have a full candidate frame
    uint16_t rx_crc = (uint16_t)buf[total - 2] | ((uint16_t)buf[total - 1] << 8);
    uint16_t calc = crc16_ccitt(&buf[2], 1 + len);

    if (rx_crc != calc) {
        // Bad frame -> drop SOF and resync
        buf.erase(buf.begin());
        return false;
    }

    out.type = buf[3];
    out.payload.assign(buf.begin() + 4, buf.begin() + (2 + 1 + len)); // payload
    buf.erase(buf.begin(), buf.begin() + total);
    return true;
}

static esp_err_t rs485_send(uart_port_t port, int de_gpio,
                            const uint8_t* data, size_t len) {
    rs485_set_tx(de_gpio);
    vTaskDelay(pdMS_TO_TICKS(1));

    int written = uart_write_bytes(port, (const char*)data, (int)len);
    if (written != (int)len) return ESP_FAIL;

    ESP_ERROR_CHECK(uart_wait_tx_done(port, pdMS_TO_TICKS(50)));

    rs485_set_rx(de_gpio);
    return ESP_OK;
}

static std::vector<uint8_t> make_frame(uint8_t type, const uint8_t* payload, uint8_t payload_len) {
    std::vector<uint8_t> f;
    f.reserve(2 + 1 + 1 + payload_len + 2);

    f.push_back(0xAA);
    f.push_back(0x55);
    uint8_t len = (uint8_t)(1 + payload_len); // TYPE + payload
    f.push_back(len);
    f.push_back(type);
    for (uint8_t i = 0; i < payload_len; i++) f.push_back(payload[i]);

    // CRC over LEN..end of payload
    uint16_t crc = crc16_ccitt(&f[2], 1 + len); // LEN + TYPE+PAYLOAD
    f.push_back((uint8_t)(crc & 0xFF));
    f.push_back((uint8_t)(crc >> 8));
    return f;
}

static void rx_task(void*) {
    // Receive on UART2 (Module B)
    rs485_set_rx(U2_DE_PIN);

    std::vector<uint8_t> buf;
    buf.reserve(256);
    int64_t last_byte_us = 0;

    while (true) {
        uint8_t b;
        int n = uart_read_bytes(U2_TX, &b, 1, pdMS_TO_TICKS(200));
        if (n == 0) {
            continue;
        }

        RxFrame f;
        if (parse_stream_byte(b, f, buf, last_byte_us)) {
            ESP_LOGI(TAG, "RX frame: type=0x%02X payload_len=%u", f.type, (unsigned)f.payload.size());

            // If this is our data frame (TYPE=0x01), reply with ACK(TYPE=0x02) containing counter (first 4 bytes)
            if (f.type == 0x01 && f.payload.size() >= 4) {
                uint32_t counter = read_u32_le(f.payload.data());

                uint8_t ack_payload[4];
                write_u32_le(ack_payload, counter);

                auto ack_frame = make_frame(0x02, ack_payload, sizeof(ack_payload));

                // Small turnaround delay helps on some setups
                vTaskDelay(pdMS_TO_TICKS(1));

                // Send ACK from Module B back onto the same bus
                (void)rs485_send(U2_TX, U2_DE_PIN, ack_frame.data(), ack_frame.size());
            }
        }
    }
}

static void rx_ack_task(void*) {
    // Module A in RX so we can receive ACKs
    rs485_set_rx(U_DE_PIN);

    std::vector<uint8_t> buf;
    buf.reserve(256);
    int64_t last_byte_us = 0;

    while (true) {
        uint8_t b;
        int n = uart_read_bytes(U_TX, &b, 1, pdMS_TO_TICKS(200));
        if (n == 0) {
            buf.clear();
            last_byte_us = 0;
            continue;
        }

        RxFrame f;
        if (parse_stream_byte(b, f, buf, last_byte_us)) {
            if (f.type == 0x02 && f.payload.size() >= 4) {
                uint32_t ack = read_u32_le(f.payload.data());
                xQueueSend(g_ack_q, &ack, 0);
            }
        }
    }
}

extern "C" void app_main(void)
{
    gpio_set_direction((gpio_num_t)U_DE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)U2_DE_PIN, GPIO_MODE_OUTPUT);

    rs485_set_rx(U_DE_PIN);
    rs485_set_rx(U2_DE_PIN);

    uart_init_port(U_TX, U_TX_PIN, U_RX_PIN);
    uart_init_port(U2_TX, U2_TX_PIN, U2_RX_PIN);

    g_ack_q = xQueueCreate(8, sizeof(int32_t));
    assert(g_ack_q);

    xTaskCreatePinnedToCore(rx_task, "rx", 4096, nullptr, 10, nullptr, 1);
    xTaskCreatePinnedToCore(rx_ack_task, "rx_ack", 4096, nullptr, 10, nullptr, 1);

    uint32_t counter = 0;
    while (true) {
        uint8_t payload[8];
        memcpy(payload, &counter, sizeof(counter));
        payload[4] = 0xDE; payload[5] = 0xAD; payload[6] = 0xBE; payload[7] = 0xEF;

        auto frame = make_frame(0x01, payload, sizeof(payload));

        bool ok = false;
        for (int attempt = 1; attempt <= 3; attempt++) {
            // Clear any stale ACKs
            uint32_t dummy;
            while (xQueueReceive(g_ack_q, &dummy, 0) == pdTRUE) {}

            esp_err_t err = rs485_send(U_TX, U_DE_PIN, frame.data(), frame.size());
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "TX failed attempt %d", attempt);
                continue;
            }

            uint32_t ack = 0;
            if (xQueueReceive(g_ack_q, &ack, pdMS_TO_TICKS(50)) == pdTRUE && ack == counter) {
                ESP_LOGI(TAG, "TX counter=%u ACK OK (attempt %d)", (unsigned)counter, attempt);
                ok = true;
                break;
            } else {
                ESP_LOGW(TAG, "TX counter=%u ACK TIMEOUT (attempt %d)", (unsigned)counter, attempt);
            }
        }

        if (!ok) {
            ESP_LOGE(TAG, "TX counter=%u failed after retries", (unsigned)counter);
        }

        counter++;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}