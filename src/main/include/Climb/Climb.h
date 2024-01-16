#pragma once

#include <iostream>
#include "ShuffleboardSender/ShuffleboardSender.h"
#include "Util/Mechanism.h"
#include "Constants/MechanismConstants.h"
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DutyCycleEncoder.h>
// #include <frc/Solenoid.h>
#include <ctre/phoenix6/controls/Follower.hpp>
#include <numeric>

using ctre::phoenix6::hardware::TalonFX;
using ctre::phoenix6::controls::Follower;

class Climb : public Mechanism{
    public:
        enum Target {
            EXTENDED,
            STOWED,
            CLIMB
        };

        enum State{
            MOVING,
            AT_TARGET,
            WINDING
        };

    Climb();
    void CorePeriodic() override;
    void CoreTeleopPeriodic() override;
    void CoreShuffleboardPeriodic() override;
    void ManualPeriodic(double voltage);
    void PullUp();
    void Stow ();
    void Extend();
    State GetState();
    void SetTarget(Target t);

    private:
        ShuffleboardSender m_shuff {"Climb", true};
        void UpdatePos();
        bool AtTarget(double target);
        TalonFX m_master {Ids::MASTER_CLIMB_MOTOR};//, m_slave {Ids::SLAVE_CLIMB_MOTOR};
        frc::DutyCycleEncoder m_absEncoder{Ids::CLIMB_ABS_ENCODER};

        double m_pos;
        Target m_targ = STOWED;
        State m_state = AT_TARGET;

        struct StateInfo{
            double TARG_POS;
            double MOVE_VOLTS;
            double RETAIN_VOLTS;
        };

        //CONSTANTS:
        double  MAX_VOLTS = 0.0, 
                MIN_POS= 0.0, 
                MAX_POS= 0.0, 
                POS_TOLERANCE= 0.0;
        StateInfo CLIMB_INFO =  {MIN_POS, 
                            0.0, //climb volts
                            0.0}; // hang volts (might be zero or stowed retain volts if using static hooks)
        StateInfo STOW_INFO =    {MIN_POS,
                            0.0, // stow/ravel volts 
                            0.0}; // retain volts
        StateInfo EXTENDED_INFO = {MAX_POS,
                             0.0, // extend/unravel volts 
                             0.0}; // prob 0
};