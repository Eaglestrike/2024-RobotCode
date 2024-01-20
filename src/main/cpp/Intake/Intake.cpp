#include "Intake/Intake.h"

Intake::Intake(bool enabled, bool dbg):
    m_rollers{enabled, false},
    m_channel{enabled, false},
    m_wrist{enabled, false}{
    Mechanism("intake",enabled, dbg);
}
void Intake::CorePeriodic(){
    m_rollers.Periodic();
    m_wrist.Periodic();
    m_channel.Periodic();
}

void Intake::CoreTeleopPeriodic(){
    m_rollers.TeleopPeriodic();
    m_wrist.TeleopPeriodic();
    m_channel.TeleopPeriodic();
    
    switch(m_actionState){
        case AMP_INTAKE:
            if (m_wrist.GetState() == Wrist::AT_TARGET)
                m_wrist.Coast();
            else if (m_wrist.GetState() == Wrist::COAST){
                /* if (beambreak1){
                    m_rollers.SetState(Rollers::RETAIN);
                    m_wrist.MoveTo(STOWED_POS);
                    m_actionState = NONE;
                }*/
            }
            break; 
        case PASSTHROUGH:
            if (m_wrist.GetState() == Wrist::AT_TARGET)
                m_wrist.Coast();
            else if (m_wrist.GetState() == Wrist::COAST){    
                /* if (beambreak2){
                    m_wrist.MoveTo(STOWED_POS);
                    m_rollers.SetState(Rollers::STOP);
                    m_channel.SetState(Channel::RETAIN)
                    m_actionState = NONE
                }*/ 
            }
            break; 
        case AMP_OUTTAKE:
            if (m_wrist.GetState() == Wrist::AT_TARGET){
                m_channel.SetState(Channel::ON);
                m_rollers.SetState(Rollers::OUTTAKE);
                m_actionState = NONE;
            }
            break;
    }
}


void Intake::SetState(ActionState newAction){
    if (newAction == m_actionState) return;
    m_actionState = newAction;
    
    double newWristPos;

    switch(newAction){
        case STOW:
            newWristPos = STOWED_POS;
            m_rollers.SetState(Rollers::STOP);
            m_actionState = NONE;
            break; 
        case AMP_INTAKE:
            newWristPos = INTAKE_POS;
            m_rollers.SetState(Rollers::INTAKE);
            m_channel.SetState(Channel::STOP);
            break; 
        case PASSTHROUGH:
            newWristPos = INTAKE_POS;
            m_rollers.SetState(Rollers::INTAKE);
            m_channel.SetState(Channel::ON);
            break; 
        case AMP_OUTTAKE:
            newWristPos = AMP_OUT_POS;
            break;
        case FEED_TO_SHOOTER:
            m_channel.SetState(Channel::ON);
            m_actionState = NONE;
            break; 
    }

    m_wrist.MoveTo(newWristPos);
}

void Intake::Stow(){
    SetState(STOW);
}

void Intake::Passthrough(){
    SetState(PASSTHROUGH);
}

void Intake::AmpOuttake(){
    SetState(AMP_OUTTAKE);
}

void Intake::AmpIntake(){
    SetState(AMP_INTAKE);
}

void Intake::FeedIntoShooter(){
    SetState(FEED_TO_SHOOTER);
}

bool Intake::HasGamePiece(){
    return (InChannel()||InIntake());
}

bool Intake::InChannel(){
    // if(beam break 2 broken)
    return true;
    // else return false;
}

bool Intake::InIntake(){
    // if(beam break 1 broken)
    return true;
    // else return false;
}