#pragma once
#include <map>
#include <vector>
#include <string>

namespace LEDConstants
{
    enum LEDSegment
    {
        HORIZONTAL,
        VERTICAL
    };
    // how many LEDs are in the strip
    const int LED_STRIP_LENGTH = 22;
    const int LED_PWM_PORT = 5;
    typedef std::vector<std::pair<int, int>> LEDSegmentRange;
    const std::map<LEDSegment, LEDSegmentRange> LED_SEGMENTS = {
        {LEDSegment::VERTICAL, {{0, 9}, {18, 23}}},
        {LEDSegment::HORIZONTAL, {{10, 17}}}};
}