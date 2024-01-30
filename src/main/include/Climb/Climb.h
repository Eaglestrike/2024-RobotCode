#pragma once

#include <iostream>
#include "ShuffleboardSender/ShuffleboardSender.h"
#include "Util/Mechanism.h"
#include "Constants/MechanismConstants.h"
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <frc/DigitalOutput.h>
#include <ctre/phoenix6/controls/Follower.hpp>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <numeric>

using ctre::phoenix6::hardware::CANcoder;
using ctre::phoenix6::hardware::TalonFX;
using ctre::phoenix6::controls::Follower;
using ctre::phoenix::motorcontrol::can::WPI_TalonSRX;

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
            MANUAL
        };

        Climb(bool enabled, bool dbg);
        void Zero();
        void SetManualInput(double ctrlr); // use for winding
        void PullUp();
        void Stow ();
        void Extend();
        void ToggleBrake();
        State GetState();
        void SetTarget(Target t);

    private:
        void Brake();
        void ReleaseBrake();
        void CorePeriodic() override;
        void CoreTeleopPeriodic() override;
        void CoreShuffleboardInit() override;
        void CoreShuffleboardPeriodic() override;

        void UpdatePos();
        bool AtTarget(double target);

        ShuffleboardSender m_shuff {"Climb", true};

        TalonFX m_master {Ids::MASTER_CLIMB_MOTOR};//, m_slave {Ids::SLAVE_CLIMB_MOTOR};
        
        WPI_TalonSRX m_brake{Ids::BRAKE};

        double m_pos;
        Target m_targ = STOWED;
        State m_state = AT_TARGET;

        double m_manualVolts=0;
        bool m_braking = true;

        struct StateInfo {
            double TARG_POS;
            double MOVE_VOLTS;
        };

        //CONSTANTS:
        double  MAX_VOLTS = 2.0, 
                MIN_POS= 0.0, 
                MAX_POS= 0.0, 
                POS_TOLERANCE= 0.0; 

        bool BREAK = true;

        // double STILL_VOLTS = 0.0 ;
        StateInfo CLIMB_INFO =  {MIN_POS, 
                                0.0}; 
        StateInfo STOW_INFO =    {MIN_POS, 
                                0.0}; // retain volts
        StateInfo EXTENDED_INFO = {MAX_POS,
                                    0.0}; // prob 0
};