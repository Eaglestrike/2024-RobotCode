#pragma once

#include <frc/AddressableLED.h>
#include <array>
#include <map>
#include "Constants/LEDConstants.h"

class LED
{
public:
    struct LEDSegmentState
    {
        int r, g, b, blink_freq;
        bool is_on = true;
        int time_on = 0;
        LEDSegmentState() : r(0), g(0), b(0), blink_freq(0){};
    };
    LED();
    void Init();
    /**
     * Set the LED strip to a specific color
     * @param r Red value
     * @param g Green value
     * @param b Blue value
     * @param blink_freq Frequency of the blink in ms (0 for no blink, must be multiple of 20)
     */
    void SetLEDSegment(LEDConstants::LEDSegment segment, int r, int g, int b, int blink_freq);
    void Periodic();
    /**
     * Turn all LEDs off
     */
    void TurnAllOff();
    /**
     * Start the LED output
     */
    void StartOutput();
    /**
     * Stop the LED output (does not turn off the LEDs)
     */
    void StopOutput();

private:
    frc::AddressableLED m_led{LEDConstants::LED_PWM_PORT};
    std::array<frc::AddressableLED::LEDData, LEDConstants::LED_STRIP_LENGTH> m_ledBuffer;
    std::array<frc::AddressableLED::LEDData, LEDConstants::LED_STRIP_LENGTH> m_emptyLedBuffer;
    std::map<LEDConstants::LEDSegment, LEDSegmentState> m_ledState;
};