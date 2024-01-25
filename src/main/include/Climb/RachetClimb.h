#pragma once

#include <iostream>
#include "ShuffleboardSender/ShuffleboardSender.h"
#include "Util/Mechanism.h"
#include "Constants/MechanismConstants.h"
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <frc/DigitalOutput.h>
#include <ctre/phoenix6/controls/Follower.hpp>
#include <numeric>

using ctre::phoenix6::hardware::CANcoder;
using ctre::phoenix6::hardware::TalonFX;
using ctre::phoenix6::controls::Follower;

class RachetClimb : public Mechanism{
    public:
        enum Target {
            EXTENDED,
            STOWED,
            CLIMB
        };

        enum State{
            MOVING,
            AT_TARGET
        };

    RachetClimb();
    void Zero();
    void CorePeriodic() override;
    void CoreTeleopPeriodic() override;
    void CoreShuffleboardPeriodic() override;
    void ManualPeriodic(double voltage); // use for winding
    void PullUp();
    void Stow ();
    void Extend();
    State GetState();
    void SetTarget(Target t);

    private:
        void UpdatePos();
        bool AtTarget(double target);

        ShuffleboardSender m_shuff {"rachet climb", true};

        TalonFX m_master {Ids::MASTER_CLIMB_MOTOR};//, m_slave {Ids::SLAVE_CLIMB_MOTOR};
        CANcoder m_absEncoder{Ids::CLIMB_ABS_ENCODER};
        frc::DigitalOutput m_actuator{Ids::CLIMB_ABS_ENCODER};

        double m_pos;
        Target m_targ = STOWED;
        State m_state = AT_TARGET;

        struct StateInfo{
            double TARG_POS;
            double MOVE_VOLTS;
        };

        //CONSTANTS:
        double  MAX_VOLTS = 0.0, 
                MIN_POS= 0.0, 
                MAX_POS= 0.0, 
                POS_TOLERANCE= 0.0; 

        bool ON = true;
        bool OFF = !ON;

        double STILL_VOLTS = 0.0 ;
        StateInfo CLIMB_INFO =  {MIN_POS, 
                                0.0}; 
        StateInfo STOW_INFO =    {MIN_POS, 
                                0.0}; // retain volts
        StateInfo EXTENDED_INFO = {MAX_POS,
                                    0.0}; // prob 0
};