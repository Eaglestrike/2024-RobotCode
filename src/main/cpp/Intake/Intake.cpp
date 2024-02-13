#include "Intake/Intake.h"

#include <algorithm>

Intake::Intake(bool enabled, bool dbg):
    Mechanism("intake", enabled, dbg),
    m_rollers{enabled, dbg},
    m_wrist{enabled, dbg},
    m_channel{enabled, dbg},
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
    DebounceBeamBreak1();

}


/**
 * Sets manual mode for wrist
 * 
 * @param manual If should be in manual mode
*/
void Intake::SetManual(bool manual) {
    if (manual) {
        m_actionState = MANUAL_WRIST;
    } else {
        m_wrist.MoveTo(0);
        Stow();
    }
}

/**
 * Sets manual input for wrist
 * 
 * @param manualInput manual JOYSTICK INPUT to set to wrist
*/
void Intake::SetManualInput(double manualInput) {
    manualInput = std::clamp(manualInput, -1.0, 1.0);
    manualInput *= m_wrist.GetMaxVolts() / 2;
    m_manualVolts = manualInput;
}


void Intake::CoreTeleopPeriodic(){
    m_rollers.TeleopPeriodic();
    m_wrist.TeleopPeriodic();
    m_channel.TeleopPeriodic();

    
    switch(m_actionState){
         case FEED_TO_SHOOTER:
            if(!InChannel()){
                m_channel.SetState(Channel::STOP);
                m_actionState = NONE;
            }
            return;
        case MANUAL_WRIST:
            m_wrist.ManualPeriodic(m_manualVolts);
            break;
        case STOW:
            if (m_wrist.GetState() == Wrist::AT_TARGET)
                m_actionState = NONE;
            break;
        case HALF_STOW:
            if (m_wrist.GetState() == Wrist::AT_TARGET)
                m_actionState = NONE;
            break;
        case PASS_TO_AMP:
            if (InIntake()){
                m_rollers.SetStateBuffer(Rollers::STOP, BACK_PROPAGATE_WAIT_s);
                m_channel.SetState(Channel::STOP);
                m_actionState = AMP_INTAKE;
            }
            break;
        case AMP_INTAKE:
            if (m_wrist.ProfileDone())
                m_wrist.Coast();
            if (/*m_wrist.GetState() == Wrist::COAST && */m_beam1broke){
                m_rollers.SetStateBuffer(Rollers::RETAIN, INTAKE_WAIT_s);
                SetState(STOW);
            }
            break; 
        case PASSTHROUGH:
            if (m_rollers.GetState() == Rollers::STOP){
                m_rollers.SetState(Rollers::PASS);
            }
            if (InIntake()){
                m_wrist.MoveTo(PASSTHROUGH_POS);
            }
            if (InChannel()){    
                m_rollers.SetState(Rollers::STOP);
                m_channel.SetState(Channel::RETAIN);
                if (!m_keepIntakeDown) {
                   SetState(STOW);
                } else 
                    m_actionState = NONE;
            }
            break; 
        case AMP_OUTTAKE:
            if (m_wrist.ProfileDone()){
                m_channel.SetState(Channel::IN);
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

    if (m_timer !=-1) return;
    if (newAction == AMP_INTAKE && (m_wrist.GetState() == Wrist::COAST || InIntake()) && m_channel.GetState() != Channel::OUT) return;
    if (newAction == PASSTHROUGH && InChannel()) return;

    std::cout << "made it out " << std::endl;

    if (newAction == AMP_INTAKE && (InChannel() || m_channel.GetState() == Channel::OUT)) newAction = PASS_TO_AMP;
    if (m_actionState == AMP_INTAKE) m_timer = -1;
    m_actionState = newAction;
    
    double newWristPos;
    switch(newAction){
        case STOW:
            newWristPos = STOWED_POS;
            m_rollers.SetState(Rollers::STOP);
            if (m_channel.GetState() != Channel::RETAIN){
                m_channel.SetState(Channel::STOP);
            }
            break; 
        case CLIMB:
            newWristPos = INTAKE_POS;
            m_rollers.SetState(Rollers::STOP);
            m_channel.SetState(Channel::STOP);
            break;
        case HALF_STOW:
            newWristPos = HALF_STOW;
            break; 
        case AMP_INTAKE:
            newWristPos = INTAKE_POS;
            m_rollers.SetState(Rollers::INTAKE);
            m_channel.SetState(Channel::STOP);
            break; 
        case PASS_TO_AMP:
            newWristPos = INTAKE_POS;
            m_rollers.SetState(Rollers::OUTTAKE);
            m_channel.SetState(Channel::OUT);
            break; 
        case PASSTHROUGH:
            newWristPos = INTAKE_POS;
            if (InIntake()){
                m_rollers.SetState(Rollers::STOP);
            } else{
                m_rollers.SetState(Rollers::PASS);
            }
            m_channel.SetState(Channel::IN);
            break; 
        case AMP_OUTTAKE:
            m_rollers.SetState(Rollers::OUTTAKE);
            newWristPos = AMP_OUT_POS;
            break;
        case FEED_TO_SHOOTER:
            m_channel.SetState(Channel::TO_SHOOT);
            return;
        default:
            return;
    }
    m_wrist.MoveTo(newWristPos);
}

void Intake::Log(FRCLogger& logger) {
    m_wrist.Log(logger);
    logger.LogBool("beambreak1", m_beam1broke);
    logger.LogNum("intake state", m_actionState);
    logger.LogBool("beambreak2", GetBeamBreak2());
}


// Beambreak stuff


bool Intake::HasGamePiece(){
    return (InChannel()||InIntake());
}

bool Intake::InChannel(){
    return GetBeamBreak2();
}

bool Intake::InIntake(){
    return m_beam1broke;
}


bool Intake::GetBeamBreak1() {
    return !m_beamBreak1.Get();
}

bool Intake::GetBeamBreak2() {
    // return false;
    return !m_beamBreak2.Get();
}


bool Intake::DebounceBeamBreak1(){
     if (m_dbTimer == -1){
        m_beam1broke = GetBeamBreak1();
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

/**
 * Zeros wrist
*/
void Intake::Zero(){
    m_wrist.Zero();
}


//DEBUG


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
    m_shuff.add("back wait time", &BACK_PROPAGATE_WAIT_s, {1,1,7,2}, true);
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
        case MANUAL_WRIST:
            m_shuff.PutString("State", "Manual Wrist");
            break;
        case CLIMB:
            m_shuff.PutString("State", "Climb");
            break;
        case PASS_TO_AMP:
            m_shuff.PutString("State", "PTA");
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
    m_shuff.PutBoolean("BeamBreak 2", GetBeamBreak2());
    m_shuff.PutNumber("debounce timer", m_dbTimer);
    m_shuff.PutNumber("REAL timer", m_timer);
    m_shuff.update(true);
}



//State machine getters & setters


Intake::ActionState Intake::GetState(){
    return m_actionState;
}

void Intake::Stow(){
    SetState(STOW);
}

void Intake::Climb(){
    SetState(CLIMB);
}

void Intake::HalfStow(){
    SetState(HALF_STOW);
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

void Intake::KeepIntakeDown(bool keepIntakeDown){
    m_keepIntakeDown = keepIntakeDown;
}