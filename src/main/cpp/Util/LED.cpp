#include "Util/LED.h"
#include <iostream>

LED::LED() {}

void LED::Init()
{
    m_led.SetLength(LEDConstants::LED_STRIP_LENGTH);
    m_led.SetData(m_emptyLedBuffer);
    // m_led.Start();
}

void LED::SetLEDSegment(LEDConstants::LEDSegment segment, int r, int g, int b, int blink_freq)
{
    if (LEDConstants::LED_SEGMENTS.find(segment) == LEDConstants::LED_SEGMENTS.end()) {
        return;
    }

    LEDSegmentState state = m_ledState[segment];
    if (state.r == r && state.g == g && state.b == b && state.blink_freq == blink_freq)
    {
        return;
    }
    state.r = r;
    state.g = g;
    state.b = b;
    state.blink_freq = blink_freq;
    m_ledState[segment] = state;
    m_led.Start();
    for (auto &seg : LEDConstants::LED_SEGMENTS.at(segment))
    {
        for (int i = seg.first; i <= seg.second; i++)
        {
            // our led strip is GRB
            m_ledBuffer[i].SetRGB(g, r, b);
        }
    }
    m_led.SetData(m_ledBuffer);
    m_led.Stop();
}

void LED::Periodic()
{
    for (auto &seg : m_ledState)
    {
        if (seg.second.blink_freq > 0)
        {
            seg.second.time_on += 20;
            if (seg.second.time_on >= seg.second.blink_freq)
            {
                seg.second.time_on = 0;
                seg.second.is_on = !seg.second.is_on;
                if (seg.second.is_on)
                {
                    SetLEDSegment(seg.first, seg.second.r, seg.second.g, seg.second.b, seg.second.blink_freq);
                }
                else
                {
                    SetLEDSegment(seg.first, 0, 0, 0, 0);
                }
            }
        }
    }
}

void LED::TurnAllOff()
{
    m_led.Start();
    m_led.SetData(m_emptyLedBuffer);
    m_led.Stop();
}

void LED::StartOutput()
{
    m_led.Start();
}

void LED::StopOutput()
{
    m_led.Stop();
}