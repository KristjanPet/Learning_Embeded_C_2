#pragma once
#include <cstdint>
// #include "driver/gpio.h"

namespace blink_lib {
    void init(int pin);
    void set(bool on);
    void toggle();
}