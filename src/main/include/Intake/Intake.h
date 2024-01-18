#pragma once
#include "Wrist.h"
#include "Rollers.h"
#include "Channel.h"

class Intake: public Mechanism{
    public:
        enum ActionState{
            STOW, 
            AMP_INTAKE, 
            PASSTHROUGH, 
            AMP_OUTTAKE,
            FEED_TO_SHOOTER
        };

        enum GamePieceState{
            NONE,
            INTAKE,
            CHANNEL
        };
        
        void SetState(ActionState newAction);
        void Stow();
        void Passthrough();
        void AmpOuttake();
        void AmpIntake();
        void FeedIntoShooter();
        void CorePeriodic() override;
        void CoreTeleopPeriodic() override;
        bool HasGamePiece();
        bool InChannel();
        bool InIntake();

    private:

        enum DBGstate {
            POS_EDITOR,
            STATE_TESTER, // lowk thats j teleop
        };

        Rollers m_rollers{true, false};
        Wrist m_wrist{};
        Channel m_channel{};
        ActionState m_actionState = STOW;
        // double m_targWristPos = STOWED_POS;

        //could also put in vector w enum as key
        double STOWED_POS = 0.0,
        INTAKE_POS = 0.0, 
        PASSTHROUGH_POS = 0.0,
        AMP_OUT_POS = 0.0;
};