#pragma once

#include <map>

#include "Util/Mechanism.h"
#include "Util/simplevectors.hpp"

#include "ShuffleboardSender/ShuffleboardSender.h"

#include "Submechanisms/Flywheel.h"
#include "Submechanisms/Pivot.h"

#include "Constants/ShooterConstants.h"

namespace vec = svector;

class Shooter : public Mechanism{
    public:
        enum State{
            IDLE,
            PREPARING,
            PREPARED,
            STROLL //Set to low speed
        };

        Shooter(std::string name, bool enabled, bool shuffleboard);

        void Idle();
        void Stroll();

        void Prepare(vec::Vector2D toSpeaker);

        bool CanShoot();

    private:
        void CoreInit() override;
        void CorePeriodic() override;
        void CoreTeleopPeriodic() override;
        
        void CoreShuffleboardInit() override;
        void CoreShuffleboardPeriodic() override;

        State state_;

        Flywheel flywheel_;
        Pivot pivot_;

        std::map<double, ShooterConstants::ShootConfig> shootData_;
        std::map<double, double> shootSpin_;

        ShuffleboardSender shuff_;
};