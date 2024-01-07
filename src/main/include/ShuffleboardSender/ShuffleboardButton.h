#pragma once

#include <vector>

#include <iostream>

#include <frc/shuffleboard/Shuffleboard.h>

#include <frc2/command/PIDCommand.h>
#include <units/voltage.h>

#include "ShuffleboardItemInterface.h"
#include "ShuffleboardHelper.h"

/**
 * Button on shuffleboard that executes a function
*/
class ShuffleboardButton : public ShuffleboardItemInterface{
    public:
        ShuffleboardButton(ItemData data, std::function<void()> callback);
        void update(bool edit) override;
        void enable(frc::ShuffleboardTab* tab) override;
        void disable() override;

    private:
        std::function<void()> callback_;
        nt::GenericEntry* entry_;
};