#pragma once

#include <chrono>

namespace LEDConstants
{
    // how many LEDs are in the strip
    const int LED_STRIP_LENGTH = 30;
    const int LED_PWM_PORT = 0;
    // WS2811: https://cdn-shop.adafruit.com/datasheets/WS2811.pdf
    // times for 0 bit
    const std::chrono::nanoseconds HIGH_TIME_0 = 500ns; // 0.5 us
    const std::chrono::nanoseconds LOW_TIME_0 = 2000ns; // 2.0 us
    // times for 1 bit
    const std::chrono::nanoseconds HIGH_TIME_1 = 1200ns; // 1.2 us
    const std::chrono::nanoseconds LOW_TIME_1 = 1300ns;  // 1.3 us
    const std::chrono::microseconds SYNC_TIME = 50us;    // above 50 us
}