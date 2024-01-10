#include "Intake/Intake.h"

void Intake::CorePeriodic(){
    m_rollers.Periodic();
    m_wrist.Periodic();
}

void Intake::CoreTeleopPeriodic(){
    m_rollers.TeleopPeriodic();
    m_wrist.TeleopPeriodic();
    
    switch(m_targState){
        // case STOW:
        //     break; 
        case AMP_INTAKE:
            /* if (beambreak1){
                m_rollers.SetState(Rollers::RETAIN);
            }*/
            break; 
        case PASSTHROUGH:
            /* if (beambreak2){
                m_wrist.MoveTo(STOWED_POS);
                m_rollers.SetState(Rollers::STOP);
                //chanell retain
            }*/ 
            break; 
        case AMP_OUTTAKE:
            if (m_wrist.GetState() == Wrist::AT_TARGET){
                //chanel on
                m_rollers.SetState(Rollers::OUTTAKE);
            }
            break;
    }
}


void Intake::SetState(TargetState t){
    if (t == m_targState) return;
    m_targState = t;
    
    double newWristPos = m_targWristPos;

    switch(t){
        case STOW:
            newWristPos = STOWED_POS;
            m_rollers.SetState(Rollers::STOP);
            break; 
        case AMP_INTAKE:
            newWristPos = INTAKE_POS;
            m_rollers.SetState(Rollers::INTAKE);
            //channel off
            break; 
        case PASSTHROUGH:
            newWristPos = INTAKE_POS;
            m_rollers.SetState(Rollers::INTAKE);
            //chanell on
            break; 
        case AMP_OUTTAKE:
            newWristPos = AMP_OUT_POS;
            break;
    }

    if (newWristPos != m_targWristPos){
        m_wrist.MoveTo(newWristPos);
        m_targWristPos = newWristPos;
    }
}