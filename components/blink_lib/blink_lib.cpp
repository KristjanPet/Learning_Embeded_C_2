#include "driver/gpio.h"
#include "blink_lib.hpp"

namespace blink_lib {
    static gpio_num_t s_pin = GPIO_NUM_NC;
    static bool s_level = false;

    void init(int pin) {
        s_pin = static_cast<gpio_num_t>(pin);
        gpio_set_direction(s_pin, GPIO_MODE_OUTPUT);
        s_level = false;
        gpio_set_level(s_pin, s_level);
    }

    void set(bool on) {
        s_level = on;
        gpio_set_level(s_pin, s_level);
    }

    void toggle() {
        s_level = !s_level;
        gpio_set_level(s_pin, s_level);
    }
}