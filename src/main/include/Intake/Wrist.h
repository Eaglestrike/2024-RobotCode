#pragma once

#define WRIST_AUTOTUNING true

#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DutyCycleEncoder.h>
#include "Util/Mechanism.h"
#include "ShuffleboardSender/ShuffleboardSender.h"
#include "Constants/IntakeConstants.h"

#if WRIST_AUTOTUNING
#include "FFAutoTuner/FFAutotuner.h"
#endif

using ctre::phoenix6::hardware::TalonFX;

class Wrist: public Mechanism{
    public:
        Wrist(bool enabled, bool dbg);

        enum MechState{
            MOVING,
            AT_TARGET,
            STOPPED,
            COAST,
            CONST_VOLTAGE //Mainly testing state (but could be used for )
            #if WRIST_AUTOTUNING
            ,AUTOTUNING
            #endif
        };
        
        void Zero();
        void ManualPeriodic(double wristVolts);
        void CoreTeleopPeriodic() override;
        void CorePeriodic() override; 
        void CoreShuffleboardPeriodic() override;
        void CoreShuffleboardInit() override;
        void MoveTo(double newPos);
        void Coast();
        void Kill();
        MechState GetState();
    private:
        void MoveToSetPt();
        void ChangeSetPt(double newPos); //pos should be in radians, w 0 as extended and parallel to ground
        void UpdatePose();
        void UpdateTargetPose();
        double FFPIDCalculate();
        void CalcSpeedDecreasePos();
        bool AtSetpoint();
        void ResetPID();
        std::string DBGToString();
        void SetVoltage();
        double GetRelPos();
        double GetAbsEncoderPos();
        
        TalonFX m_wristMotor {IntakeConstants::WRIST_MOTOR};
        frc::DutyCycleEncoder m_wristEncoder{IntakeConstants::WRIST_ENCODER_CAN_ID};

        //MEMBER VARS
        //state vars
        MechState m_state = MechState::AT_TARGET;

        //profile vars
        double m_setPt; 
        double m_newSetPt = -1;
        double m_curPos, m_curVel, m_curAcc; // cur pose
        double m_targetPos = m_setPt, m_targetVel =0 , m_targetAcc = 0; // motion profile 
        double m_speedDecreasePos, // pos in motion profile where start decelerating
                m_totalErr = 0; // integral of position error for PID

        double m_absEncoderInit;

        //Shuffleboard
        ShuffleboardSender m_shuff;
        
        #if WRIST_AUTOTUNING
        FFAutotuner m_autoTuner {"Wrist Autotuner", FFAutotuner::ARM};
        #endif

        //Constants
        double m_kp = 0.0, m_ki = 0.0, m_kd = 0.0;
        double m_s = 0.0, m_g = 0.0, m_v = 0.0, m_a = 0.0;

        double MAX_POS = 0.0, MIN_POS = 0.0;

        double MAX_VEL = 0.0, MAX_ACC = 0.0;
        double POS_TOLERANCE = 0.0;

        double WRIST_ABS_ENCODER_OFFSET = 0.0;
        
        double MAX_VOLTS = 5.0;
        
        // will prob change
        double REL_CONV_FACTOR = 1 * (1.0 / 20.0) * (16.0 / 36.0) * (2 * M_PI) * (1.885 / 1.588);
        //for dbg
        double m_voltReq = 0.0;
};