#pragma once

#include <ctre/phoenix6/TalonFX.hpp>

#include "Util/Poses.h"
#include "Util/Mechanism.h"
#include "Util/VelocityProfile.h"

#include "ShuffleboardSender/ShuffleboardSender.h"

#include "Constants/ShooterConstants.h"

using ctre::phoenix6::hardware::TalonFX;

class Flywheel : public Mechanism{
    public:
        enum class State{
            STOP,
            RAMPING,
            AT_TARGET,
            JUST_VOLTAGE
        };

        //Constructor
        Flywheel(ShooterConstants::FlywheelConfig config, bool enabled = true, bool shuffleboard = false);

        //Controls
        void Stop();
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
        TalonFX motor_;
        double volts_;
        double maxVolts_;
        Poses::Pose1D currPose_;

        //Logic
        State state_;
        VelocityProfile profile_;
        ShooterConstants::Feedforward feedforward_;

        //Shuffleboard
        std::string StateToString(State state);

        ShuffleboardSender shuff_;
};