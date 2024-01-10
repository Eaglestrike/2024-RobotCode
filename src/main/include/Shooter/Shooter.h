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

        void SetUp(double vel, double spin, double pivot);
        Flywheel lflywheel_;
        Flywheel rflywheel_;
        Pivot pivot_;

        double strollSpeed_;

        std::map<double, ShooterConstants::ShootConfig> shootData_;
        double kSpin_;

        std::string StateToString(State state);
        ShuffleboardSender shuff_;
};