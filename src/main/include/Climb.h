#pragma once

#include <iostream>
// #include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include "ShuffleboardSender/ShuffleboardSender.h"
#include "Util/Mechanism.h"
#include "Constants/ClimbConstants.h"
// #include "Controls.h"
// #include "Constants.h"
// #include "TrajectoryCalc.h"

#include <frc/Solenoid.h>
#include <numeric>

class Climb :  public Mechanism{
    public:
        enum Target {
            EXTENDED,
            STOWED,
            CLIMB
        };

        enum State{
            MANUAL,
            MOVING,
            AT_TARGET,
        };

    Climb();
    void Periodic() override;
    void CoreShuffleboardPeriodic() override;
    void SetTarget(Target t);

    private:
        ShuffleboardSender m_shuff{"climb", true};
        void UpdatePos();
        rev::CANSparkMax m_motor{ClimbConstants::MOTOR_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
        rev::SparkMaxRelativeEncoder m_relEncoder = m_motor.GetEncoder();

        // frc::Solenoid pneumatic1_;
        // frc::Solenoid pneumatic2_;
        // frc::Solenoid brake_

        double MAX_VOLTS; //= ClimbConstants::MAX_VOLTS;
        double MIN_POS; //= ClimbConstants::MIN_POS;
        double MAX_POS; //= ClimbConstants::MAX_POS;
        double CLIMB_VOLTS; //= ClimbConstants::CLIMB_VOLTS;
        double HOLD_VOLTS; //= ClimbConstants::HOLD_VOLTS;


        double m_pos;

        Target m_targ = STOWED;
        State m_state = AT_TARGET;
};