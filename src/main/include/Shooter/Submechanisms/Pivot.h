#pragma once

#include <frc/DutyCycleEncoder.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include "FFAutotuner/FFAutotuner.h"

#include "ShuffleboardSender/ShuffleboardSender.h"

#include "Util/Poses.h"
#include "Util/Mechanism.h"
#include "Util/TrapezoidalProfile.h"

#include "Constants/ShooterConstants.h"

using ctre::phoenix6::hardware::TalonFX;

class Pivot : public Mechanism{
    public:
        enum State{
            STOP,
            AIMING,
            AT_TARGET,
            JUST_VOLTAGE
        };

        Pivot(std::string name, bool enabled, bool shuffleboard);

        void Stop();

        void SetAngle(double angle);
        void SetVoltage(double volts);

        void Zero();

        bool AtTarget();

        Poses::Pose1D GetPose();

    private:
        void CorePeriodic() override;
        void CoreTeleopPeriodic() override;

        void CoreShuffleboardInit() override;
        void CoreShuffleboardPeriodic() override;
        State state_;

        TalonFX motor_;
        TalonFX motorChild_;
        double volts_;
        double maxVolts_;

        frc::DutyCycleEncoder encoder_;
        double offset_;

        struct Bounds{
            double min;
            double max;
        } bounds_;

        ShooterConstants::PID pid_;
        double accum_;
        ShooterConstants::Feedforward ff_;

        double maxV_;
        double maxA_;
        TrapezoidalProfile profile_;

        Poses::Pose1D currPose_; //Angle (rad)

        std::string StateToString(State state);
        ShuffleboardSender shuff_;
};