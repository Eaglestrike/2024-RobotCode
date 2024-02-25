#include "Util/LED.h"
#include <iostream>

LED::LED() {}

void LED::Init()
{
    m_led.SetLength(LEDConstants::LED_STRIP_LENGTH);
    m_led.SetData(m_emptyLedBuffer);
    m_led.Start();
}

void LED::SetStripColor(int r, int g, int b)
{
    if (curr_r == r && curr_g == g && curr_b == b)
    {
        return;
    }
    curr_r = r;
    curr_g = g;
    curr_b = b;
    m_led.Start();
    for (int i = 0; i < LEDConstants::LED_STRIP_LENGTH; i++)
    {
        m_ledBuffer[i].SetRGB(r, g, b);
    }
    m_led.SetData(m_ledBuffer);
    m_led.Stop();
}

void LED::SetStripEnabled(bool enabled)
{
    if (enabled && !m_ledEnabled)
    {
        m_led.SetData(m_emptyLedBuffer);
        m_ledEnabled = true;
    }
    else
    {
        m_led.SetData(m_ledBuffer);
        m_ledEnabled = false;
    }
}

void LED::ToggleStripEnabled()
{
    SetStripEnabled(!m_ledEnabled);
}

void LED::SetBlinkMode(bool blinkMode)
{
    m_blinkMode = blinkMode;
}

void LED::LEDPeriodic()
{
    if (m_blinkMode)
    {
        ToggleStripEnabled();
    }
}

void LED::Start()
{
    m_led.Start();
}

void LED::Stop()
{
    m_led.SetData(m_emptyLedBuffer);
    m_led.Stop();
}