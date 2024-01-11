#pragma once

#include <iostream>
#include "ShuffleboardSender/ShuffleboardSender.h"
#include "Util/Mechanism.h"
#include "Constants/MechanismConstants.h"
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DutyCycleEncoder.h>
// #include <frc/Solenoid.h>
#include <numeric>

using ctre::phoenix6::hardware::TalonFX;

class Climb : public Mechanism{
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
    void CorePeriodic() override;
    void CoreShuffleboardPeriodic() override;
    void PullUp();
    void Stow ();
    void Extend();
    State GetState();
    void SetTarget(Target t);

    private:
        ShuffleboardSender m_shuff {"Climb", true};
        void UpdatePos();
        bool AtTarget(double target);
        TalonFX m_master {Ids::MASTER_CLIMB_MOTOR}, m_slave {Ids::SLAVE_CLIMB_MOTOR};
        frc::DutyCycleEncoder m_absEncoder{Ids::CLIMB_ABS_ENCODER};
        //electronic solenoid brake

        double m_pos;
        Target m_targ = STOWED;
        State m_state = AT_TARGET;

        //CONSTANTS:
        double  MAX_VOLTS = 0.0, 
                MIN_POS= 0.0, 
                MAX_POS= 0.0, 
                POS_TOLERANCE= 0.0, 
                CLIMB_VOLTS= 0.0, 
                HOLD_VOLTS= 0.0;
};