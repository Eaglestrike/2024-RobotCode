#pragma once

#include <vector>

#include <iostream>

#include <frc/shuffleboard/Shuffleboard.h>

#include <frc2/command/PIDCommand.h>
#include <units/voltage.h>

#include "ShuffleboardItemInterface.h"
#include "ShuffleboardHelper.h"

/**
 * Button on shuffleboard that executes a function for toggle true and a different one for toggle false
*/
class ShuffleboardToggleButton : public ShuffleboardItemInterface{
    public:
        ShuffleboardToggleButton(ItemData data, std::function<void()> callbackTrue, std::function<void()> callbackFalse, bool initVal);
        void update(bool edit) override;
        void enable(frc::ShuffleboardTab* tab) override;
        void disable() override;

    private:
        std::function<void()> callbackTrue_;
        std::function<void()> callbackFalse_;
        bool prevVal_;
        nt::GenericEntry* entry_;
};