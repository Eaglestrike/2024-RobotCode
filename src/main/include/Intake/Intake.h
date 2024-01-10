#include "Wrist.h"
#include "Rollers.h"

class Intake: public Mechanism{
    public:
        enum TargetState{
            STOW, 
            AMP_INTAKE, 
            PASSTHROUGH, 
            AMP_OUTTAKE,
        };
        
        void SetState(TargetState t);
        void CorePeriodic() override;
        void CoreTeleopPeriodic() override;

    private:

        enum DBGstate {
            POS_EDITOR,
            STATE_TESTER, // lowk thats j teleop
        };

        Rollers m_rollers{};
        Wrist m_wrist{};
        TargetState m_targState = STOW;
        double m_targWristPos = STOWED_POS;

        //could also put in vector w enum as key
        double STOWED_POS = 0.0,
        INTAKE_POS = 0.0, 
        PASSTHROUGH_POS = 0.0,
        AMP_OUT_POS = 0.0;
};