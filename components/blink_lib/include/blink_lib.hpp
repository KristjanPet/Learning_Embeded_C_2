#pragma once
#include "driver/gpio.h"

namespace blink_lib {
    void init(gpio_num_t pin);
    void set(bool on);
    void toggle();
}