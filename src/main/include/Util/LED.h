#pragma once

#include <frc/AddressableLED.h>
#include <array>
#include "Constants/LEDConstants.h"

class LED
{
public:
    LED();
    void Init();
    void SetStripColor(int r, int g, int b);
    void SetBlinkMode(bool blinkMode);
    void SetStripEnabled(bool enabled);
    void ToggleStripEnabled();
    void LEDPeriodic();
    void Start();
    void Stop();

private:
    frc::AddressableLED m_led{LEDConstants::LED_PWM_PORT};
    std::array<frc::AddressableLED::LEDData, LEDConstants::LED_STRIP_LENGTH> m_ledBuffer;
    std::array<frc::AddressableLED::LEDData, LEDConstants::LED_STRIP_LENGTH> m_emptyLedBuffer;
    int curr_r = 0;
    int curr_g = 0;
    int curr_b = 0;
    bool m_blinkMode = false;
    bool m_ledEnabled = false;
};