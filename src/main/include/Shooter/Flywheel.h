#pragma once

#include <ctre/Phoenix.h>

#include "Shooter/ShooterConstants.h"

#include "Util/Poses.h"
#include "Util/Mechanism.h"
#include "Util/VelocityProfile.h"

#include "ShuffleboardSender/ShuffleboardSender.h"

class Flywheel : public Mechanism{
    public:
        enum class State{
            IDLE,
            RAMPING,
            AT_TARGET,
            JUST_VOLTAGE
        };

        //Constructor
        Flywheel(std::string name, bool enabled = true, bool shuffleboard = false);

        //Controls
        void SetIdle();
        void SetTarget(double vel);
        void SetVoltage(double volts);

        //Info
        State GetState();
        bool AtTarget();

        //Getters and Setters
        Poses::Pose1D GetPose();
        void SetFeedforward(double ks, double kv, double ka);

    private:
        //Core Functions
        void CorePeriodic() override;
        void CoreTeleopPeriodic() override;

        void CoreShuffleboardInit() override;
        void CoreShuffleboardPeriodic() override;
        void CoreShuffleboardUpdate() override;

        //Motor
        WPI_TalonFX motor_;
        double volts_;
        double maxVolts_;
        Poses::Pose1D currPose_;

        //Logic
        State state_;
        VelocityProfile profile_;
        ShooterConstants::Feedforward feedforward_;

        //Shuffleboard
        std::string StateToString(State state);
};