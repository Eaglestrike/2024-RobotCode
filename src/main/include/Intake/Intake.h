#pragma once

#include <frc/DigitalInput.h>

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
        void Stow();
        void Passthrough();
        void AmpOuttake();
        void AmpIntake();
        void FeedIntoShooter();
        void CorePeriodic() override;
        void CoreTeleopPeriodic() override;
        // void CoreAutonomousPeriodic() override;
        void KeepIntakeDown(bool intakeDown);
        bool HasGamePiece();
        bool InChannel();
        bool InIntake();

    private:
        enum DBGstate {
            POS_EDITOR,
            STATE_TESTER, // lowk thats j teleop
        };

        bool GetBeamBreak1();
        bool GetBeamBreak2();

        Rollers m_rollers;
        Wrist m_wrist;
        Channel m_channel;
        ActionState m_actionState = NONE;

        bool m_keepIntakeDown = false;

        double STOWED_POS = 0.0,
        INTAKE_POS = 0.0, 
        PASSTHROUGH_POS = 0.0,
        AMP_OUT_POS = 0.0;

        frc::DigitalInput m_beamBreak1{IntakeConstants::BEAM_BREAK1_ID};
};