#include "Util/LED.h"

LED::LED()
{
    m_led.SetLength(LEDConstants::LED_STRIP_LENGTH);
    m_led.SetBitTiming(LEDConstants::HIGH_TIME_0, LEDConstants::LOW_TIME_0, LEDConstants::HIGH_TIME_1, LEDConstants::LOW_TIME_1);
    m_led.SetSyncTime(LEDConstants::SYNC_TIME);
    m_led.SetData(m_ledBuffer);
    m_led.Start();
}

void LED::SetStripColor(int r, int g, int b)
{
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
        m_led.Start();
        m_ledEnabled = true;
    }
    else
    {
        m_led.Stop();
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