#include "Util/LED.h"
#include <iostream>

LED::LED() {}

void LED::Init()
{
    // std::cout << "init led\n";
    m_led.SetLength(LEDConstants::LED_STRIP_LENGTH);
    m_led.SetData(m_emptyLedBuffer);
    m_led.Start();
}

void LED::SetStripColor(int r, int g, int b)
{
    // std::cout << "setting strip color\n";
    for (int i = 0; i < LEDConstants::LED_STRIP_LENGTH; i++)
    {
        m_ledBuffer[i].SetRGB(r, g, b);
    }
    m_led.SetData(m_ledBuffer);
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

LED::~LED()
{
    m_led.Stop();
}