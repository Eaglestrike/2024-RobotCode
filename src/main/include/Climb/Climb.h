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
            CLIMB,
            TMANUAL
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
        void Stop ();
        void Extend();
        // void ToggleBrake();
        State GetState();
        void SetTarget(Target t);
        void ChangeBrake(bool brake);

    private:
        void Brake();
        void ReleaseBrake();
        void CorePeriodic() override;
        void CoreTeleopInit() override;
        void CoreTeleopPeriodic() override;
        void CoreShuffleboardInit() override;
        void CoreShuffleboardPeriodic() override;

        void UpdatePos();
        bool AtTarget(double target, bool up);

        ShuffleboardSender m_shuff;

        TalonFX m_master {Ids::MASTER_CLIMB_MOTOR};//, m_slave {Ids::SLAVE_CLIMB_MOTOR};
        
        WPI_TalonSRX m_brake{Ids::BRAKE};

        double m_pos;
        Target m_targ = STOWED;
        State m_state = AT_TARGET;

        double m_manualVolts=0;
        bool m_brakeOverride = false;
        bool m_braking = true;

        double m_timer = -1;
        double WAIT_TIME_S = 0.1;
        double UNRACHET_VOLTS = -3.0; 

        double m_startTimeDown = -1;
        double UNRATCHET_WAIT = 0.3;

        bool m_zeroed = false;

        struct StateInfo {
            double TARG_POS;
            double MOVE_VOLTS;
        };

        //CONSTANTS:
        double  MANUAL_VOLTS = 7.0,
                MAX_VOLTS = 10.0, 
                MIN_POS= 0.0, 
                MAX_POS= 130.0, 
                POS_TOLERANCE= 0.0;

        bool BREAK = true;

        // double STILL_VOLTS = 0.0 ;
        StateInfo CLIMB_INFO =  {MIN_POS, 
                                -8.0}; 
        StateInfo STOW_INFO =    {MIN_POS, 
                                -8.0}; // retain volts
        StateInfo EXTENDED_INFO = {MAX_POS-5.0,
                                    8.0}; // prob 0
        

        double TELE_PERIOD = 135.0;

        double m_beginTime;
};