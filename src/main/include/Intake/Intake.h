#pragma once

#include <frc/DigitalInput.h>

#include "ShuffleboardSender/ShuffleboardSender.h"

#include "Constants/IntakeConstants.h"

#include "Intake/Wrist.h"
#include "Intake/Rollers.h"
#include "Intake/Channel.h"

class Intake: public Mechanism{
    public:
        Intake(bool enable, bool dbg);
        enum ActionState{
            STOW, 
            AMP_INTAKE, 
            PASSTHROUGH, 
            AMP_OUTTAKE,
            FEED_TO_SHOOTER,
            NONE
        };
        
        void SetState(ActionState newAction);
        ActionState GetState();
        void Stow();
        void Passthrough();
        void AmpOuttake();
        void AmpIntake();
        void FeedIntoShooter();
        // void CoreAutonomousPeriodic() override;
        void KeepIntakeDown(bool intakeDown);

        bool HasGamePiece();
        bool InChannel();
        bool InIntake();

    private:
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

        bool m_keepIntakeDown = false;
        
        bool m_beam1broke= false;

        double m_timer = -1;
        double m_dbTimer = -1;

        double STOWED_POS = M_PI / 2,
        INTAKE_POS = -0.7, 
        PASSTHROUGH_POS = INTAKE_POS,
        AMP_OUT_POS = 1.364; // 1.26 

        double INTAKE_WAIT_s = 0.1;
        double OUTTAKE_WAIT_s = 0.5;
        double DEBOUNCE_WAIT_s = 2.0;

        frc::DigitalInput m_beamBreak1{IntakeConstants::BEAM_BREAK1_ID};

        ShuffleboardSender m_shuff;
};