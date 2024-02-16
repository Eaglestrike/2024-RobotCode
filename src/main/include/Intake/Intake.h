#pragma once

#include <frc/DigitalInput.h>

#include "ShuffleboardSender/ShuffleboardSender.h"
#include "Util/Logger.h"
#include "Constants/IntakeConstants.h"

#include "Intake/Wrist.h"
#include "Intake/Rollers.h"
#include "Intake/Channel.h"

class Intake: public Mechanism{
    public:
        Intake(bool enable, bool dbg);
        enum ActionState{
            STOW, 
            CLIMB,
            HALF_STOW,
            AMP_INTAKE, 
            PASSTHROUGH, 
            AMP_OUTTAKE,
            FEED_TO_SHOOTER,
            MANUAL_WRIST,
            MANUAL_CHANNEL,
            PASS_TO_AMP,
            NONE
        };
        
        ActionState GetState();
        void Stow();
        void Climb();
        void HalfStow();
        void Passthrough();
        void AmpOuttake();
        void AmpIntake();
        void FeedIntoShooter();
        // void CoreAutonomousPeriodic() override;
        void KeepIntakeDown(bool intakeDown);
        void Zero();
        void SetManual(bool manual);
        void SetManualInput(double manualInput);

        void EjectForward();
        void EjectBack();
        void EjectSplit();
        void EjectStop();

        bool HasGamePiece();
        bool InChannel();
        bool InIntake();

        void Log(FRCLogger& logger);

    private:
        void SetState(ActionState newAction);
        void CoreInit() override;
        void CorePeriodic() override;
        void CoreTeleopPeriodic() override;

        void CoreShuffleboardInit() override;
        void CoreShuffleboardPeriodic() override;

        bool GetBeamBreak1();
        bool GetBeamBreak2();
        bool DebounceBeamBreak1();

        Rollers m_rollers;
        Wrist m_wrist;
        Channel m_channel;
        ActionState m_actionState = NONE;

        //used for AmpIntake
        bool m_wentToPassthrough;

        bool m_keepIntakeDown = false;
        
        bool m_beam1broke= false;

        double m_outTimer = -1;
        double m_dbTimer = -1;

        double STOWED_POS = M_PI / 2,
        HALF_STOWED_POS = 1.0,
        INTAKE_POS = -0.69, 
        PASSTHROUGH_POS = -0.64,
        AMP_OUT_POS = 1.464; // 1.26 

        double INTAKE_WAIT_s = 0.0;
        double OUTTAKE_WAIT_s = 0.5;
        double BACK_PROPAGATE_WAIT_s = 0.2; //Wait time from passthrough to amp
        double DEBOUNCE_WAIT_s = 1.0;


        double m_manualVolts = 0;

        frc::DigitalInput m_beamBreak1{IntakeConstants::BEAM_BREAK1_ID};
        frc::DigitalInput m_beamBreak2{IntakeConstants::BEAM_BREAK2_ID};

        ShuffleboardSender m_shuff;
};