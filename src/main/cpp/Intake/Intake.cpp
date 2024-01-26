#include "Intake/Intake.h"

Intake::Intake(bool enabled, bool dbg):
    Mechanism("intake", enabled, dbg),
    m_rollers{true, dbg},
    m_wrist{enabled, dbg},
    m_channel{false, dbg},
    m_shuff{"intake", dbg}
{
}

void Intake::CoreInit(){
    m_rollers.Init();
    m_wrist.Init();
    m_channel.Init();
}

void Intake::CorePeriodic(){
    m_rollers.Periodic();
    m_wrist.Periodic();
    m_channel.Periodic();
}

Intake::ActionState Intake::GetState(){
    return m_actionState;
}

void Intake::CoreTeleopPeriodic(){
    m_rollers.TeleopPeriodic();
    m_wrist.TeleopPeriodic();
    m_channel.TeleopPeriodic();
    
    switch(m_actionState){
        case STOW:
            if (m_wrist.GetState() == Wrist::AT_TARGET)
                m_actionState = NONE;
            break;
        case AMP_INTAKE:
            if (m_wrist.ProfileDone())
                m_wrist.Coast();
            if (m_wrist.GetState() == Wrist::COAST && DebounceBeamBreak1()){
                m_rollers.SetStateBuffer(Rollers::RETAIN, INTAKE_WAIT_s);
                m_wrist.MoveTo(STOWED_POS);
                m_actionState = NONE;
            }
            break; 
        case PASSTHROUGH:
            if (m_wrist.GetState() == Wrist::AT_TARGET)
                m_wrist.Coast();
            else if (m_wrist.GetState() == Wrist::COAST){    
                /* if (beambreak2){
                    if (!keepIntakeDown) {
                        m_wrist.MoveTo(STOWED_POS);   
                    }
                    m_rollers.SetState(Rollers::STOP); // idk if this goes here or in the if
                    m_channel.SetState(Channel::RETAIN)
                    m_actionState = NONE
                }*/ 
            }
            break; 
        case AMP_OUTTAKE:
            if (m_wrist.ProfileDone()){
                m_channel.SetState(Channel::ON);
                m_rollers.SetState(Rollers::OUTTAKE);
            }
            if (m_timer >= OUTTAKE_WAIT_s){
                m_timer = -1;
                SetState(STOW);
            } else if (m_timer != -1){
                m_timer += 0.02;
            } else if (!GetBeamBreak1()){
                m_timer = 0;
            }
            break;
        default:
            break;
    }
}

void Intake::SetState(ActionState newAction){
    if (newAction == m_actionState) return;

    if (newAction == AMP_INTAKE && (m_wrist.GetState() == Wrist::COAST || m_timer !=-1)) return;
    m_actionState = newAction;
    
    double newWristPos;
    switch(newAction){
        case STOW:
            newWristPos = STOWED_POS;
            m_rollers.SetState(Rollers::STOP);
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
            m_rollers.SetState(Rollers::OUTTAKE);
            newWristPos = AMP_OUT_POS;
            break;
        case FEED_TO_SHOOTER:
            m_channel.SetState(Channel::ON);
            m_actionState = NONE;
            return;
        default:
            return;
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

void Intake::KeepIntakeDown(bool keepIntakeDown){
    m_keepIntakeDown = keepIntakeDown;
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
    return false;
    // else return false;
}

bool Intake::InIntake(){
    return m_beam1broke;
}

bool Intake::GetBeamBreak1() {
    return !m_beamBreak1.Get();
}

bool Intake::DebounceBeamBreak1(){
     if (m_dbTimer == -1){
        m_beam1broke = !m_beamBreak1.Get();
        if (m_beam1broke){
            m_dbTimer= 0;
        }
    } else {
        m_dbTimer+= 0.02;
        if (m_dbTimer >= DEBOUNCE_WAIT_s){
            m_dbTimer = -1;
        }
        m_beam1broke = true;
    }   
    return m_beam1broke;
}

void Intake::CoreShuffleboardInit(){
    //State (row 0)
    m_shuff.PutString("State", "", {2,1,0,0});

    //Target position (row 1)
    m_shuff.add("Stow", &STOWED_POS, {1,1,0,1}, true);
    m_shuff.add("Intake", &INTAKE_POS, {1,1,1,1}, true);
    m_shuff.add("Passthrough", &PASSTHROUGH_POS, {1,1,2,1}, true);
    m_shuff.add("Amp", &AMP_OUT_POS, {1,1,3,1}, true);
    m_shuff.add("wait time", &INTAKE_WAIT_s, {1,1,6,1}, true);
    m_shuff.add("out wait time", &OUTTAKE_WAIT_s, {1,1,7,1}, true);
    m_shuff.add("debounce wait time", &DEBOUNCE_WAIT_s, {1,1,7,1}, true);

    //Go to NONE state (row 2)
    m_shuff.addButton("NONE STATE", [&]{m_actionState = NONE;}, {4,1,0,2});
}

void Intake::CoreShuffleboardPeriodic(){
    switch(m_actionState){
        case STOW:
            m_shuff.PutString("State", "Stow");
            break;
        case AMP_INTAKE:
            m_shuff.PutString("State", "Amp Intake");
            break;
        case PASSTHROUGH:
            m_shuff.PutString("State", "Passthrough");
            break;
        case AMP_OUTTAKE:
            m_shuff.PutString("State", "Amp Outtake");
            break;
        case FEED_TO_SHOOTER:
            m_shuff.PutString("State", "Feed to Shooter");
            break;
        case NONE:
            m_shuff.PutString("State", "None");
            break;
    }

    switch(m_wrist.GetState()){
        case Wrist:: MOVING:
            m_shuff.PutString("Wrist", "Moving");
            break;
        case Wrist::AT_TARGET:
            m_shuff.PutString("Wrist", "At target");
            break;
        case Wrist::STOPPED:
            m_shuff.PutString("Wrist", "Stppped");
            break;
        case Wrist::COAST:
            m_shuff.PutString("Wrist", "Coast");
            break;
    }
            
    m_shuff.PutBoolean("BeamBreak 1", m_beam1broke);
    m_shuff.update(true);
}