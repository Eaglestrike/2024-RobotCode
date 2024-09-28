#include "Climb/Climb.h"

#include "Util/Utils.h"

Climb::Climb(bool enabled, bool dbg): 
    m_shuff{"Climb", dbg},
    Mechanism{"climb", enabled, dbg}
{
    m_master.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
    m_master.SetInverted(true);
    Zero();
    // m_slave.SetControl(Follower(Ids::MASTER_CLIMB_MOTOR, false));
}


void Climb::CorePeriodic(){
    UpdatePos();
}

void Climb::CoreTeleopInit() {
    m_beginTime = Utils::GetCurTimeS();
}

void Climb::CoreTeleopPeriodic(){
    StateInfo info;
    double curTime = Utils::GetCurTimeS();
    switch (m_targ){
        case STOWED:
            if (m_state != MANUAL && m_state != AT_TARGET) {
                if (curTime - m_beginTime > TELE_PERIOD - 1.0) {
                    Brake();
                } else {
                    ReleaseBrake();
                }
            }
            info = STOW_INFO;
            break;
        case EXTENDED:
            if (m_state != MANUAL && m_state != AT_TARGET) {
                ReleaseBrake();
            }
            info = EXTENDED_INFO;
            break;
        case CLIMB:
            if (m_state != MANUAL && m_state != AT_TARGET) {
                if (curTime - m_beginTime > TELE_PERIOD - 1.0) {
                    Brake();
                } else {
                    ReleaseBrake();
                }
            }
            info = CLIMB_INFO;
            break;
    }

    double voltage = 0.0;

    switch(m_state){
        case MOVING:
            if (m_timer == -1)
                voltage = info.MOVE_VOLTS;
            else {
                voltage = UNRACHET_VOLTS;
                m_timer += 0.02;
                if (m_timer >= WAIT_TIME_S){
                    m_timer =-1; //-1 means finished
                }
            }
            if (AtTarget(info.TARG_POS, (info.MOVE_VOLTS > 0))){
                m_state = AT_TARGET;
            }
            break;
        case AT_TARGET:
            m_brakeOverride = false; //Default brake override for manual false

            if (m_targ != TMANUAL && !AtTarget(info.TARG_POS, (info.MOVE_VOLTS > 0))){
                m_state = MOVING;
            }

            if (m_targ == STOWED || m_targ == CLIMB || m_targ == TMANUAL) {
                Brake();
            }
            break;
        case MANUAL:
            if (m_pos <= MIN_POS-1 && m_manualVolts < 0) {
                voltage = 0;
            } else if (m_pos >= MAX_POS+1 && m_manualVolts > 0) {
                voltage = 0;
            } else {
                voltage = m_manualVolts;
            }

            if(!m_brakeOverride){ //Automatic braking
                m_braking = (voltage == 0); //Brake when not applying voltage
                if((m_timer < WAIT_TIME_S) && (voltage > 0)){//Move down to unratchet
                    voltage = UNRACHET_VOLTS;
                }
                m_timer += 0.02;
            }
            if (m_braking){
                if (voltage > 0) {
                    voltage = 0;
                }
                Brake();
            } else{
                ReleaseBrake();
            }
            m_state = AT_TARGET;
            break;
    }

    voltage = std::clamp(voltage, -MAX_VOLTS, MAX_VOLTS);
 
    m_master.SetVoltage(units::volt_t(-voltage));
}

void Climb::Brake(){
    m_brake.SetVoltage(units::volt_t(12));
    m_braking = true;
}

void Climb::ReleaseBrake(){
    m_brake.SetVoltage(units::volt_t(-12));
    m_braking = false;
}

void Climb::ChangeBrake(bool brake){
    m_brakeOverride = true;
    if (!brake){
        // m_timer = 0;
        ReleaseBrake();
    } else {
        Brake();
    }

}

void Climb::UpdatePos(){
    m_pos = -m_master.GetPosition().GetValueAsDouble();
}

bool Climb::AtTarget(double target, bool up){
    // if (std::abs(m_pos-target) <= POS_TOLERANCE)
    if (up && (m_pos >= target-POS_TOLERANCE))
        return true;
    else if (!up && (m_pos <= target+POS_TOLERANCE))
        return true;
    return false;
}

void Climb::Extend(){
    SetTarget(EXTENDED);
}

void Climb::Stow(){
    SetTarget(STOWED);
}

void Climb::PullUp(){
    m_startTimeDown = Utils::GetCurTimeS();
    SetTarget(CLIMB);
}

void Climb::Stop(){
    m_state = AT_TARGET;
}

Climb::State Climb::GetState(){
    return m_state;
}

void Climb::SetTarget(Target t){
    if (t == m_targ) return;
    m_state = MOVING;
    m_timer = 0;
    m_targ = t;
}

// should put break on here also 
void Climb::SetManualInput(double ctrlr){
    ctrlr = std::clamp(-ctrlr, -1.0, 1.0);
    ctrlr *= MANUAL_VOLTS;
    if (ctrlr <= 0){ //Reset timer when not moving up
        m_timer =0;
    }
    m_manualVolts = ctrlr;
    m_targ = TMANUAL;
    m_state = MANUAL;
}

void Climb::Zero(){
    m_master.SetPosition(units::angle::turn_t(0));
}

void Climb::CoreShuffleboardInit(){
    //settings (row 0)
    m_shuff.add("max pos", &MAX_POS, {1,1,0,0}, true);
    m_shuff.add("min pos", &MIN_POS, {1,1,1,0}, true);
    m_shuff.add("max volts", &MAX_VOLTS, {1,1,2,0}, true);
    m_shuff.add("tolerance", &POS_TOLERANCE, {1,1,3,0}, true);
     m_shuff.add("manual volt", &m_manualVolts, {1,1,4,0}, false);
     m_shuff.add("manual multiplier volts", &MANUAL_VOLTS, {1,1,5,0}, true);
     m_shuff.add("timer", &m_timer, {1,1,5,0}, false);
     m_shuff.add("wait time", &WAIT_TIME_S, {1,1,6,0}, true);

    //Climb (row 1)
    m_shuff.add("climb pos", &CLIMB_INFO.TARG_POS, {1,1,0,1}, true);
    m_shuff.add("climb volts", &CLIMB_INFO.MOVE_VOLTS, {1,1,1,1}, true);

    //Stow (row 2)
    m_shuff.add("stow pos", &STOW_INFO.TARG_POS, {1,1,0,2}, true);
    m_shuff.add("stow volts", &STOW_INFO.MOVE_VOLTS, {1,1,1,2}, true);

    //Extend (row 3)
    m_shuff.add("extend pos", &EXTENDED_INFO.TARG_POS, {1,1,0,3}, true);
    m_shuff.add("extend volts", &EXTENDED_INFO.MOVE_VOLTS, {1,1,1,3}, true);

    // // lock/unlock
    // m_shuff.add("locking", &m_braking, {1, 1, });
}

void Climb::CoreShuffleboardPeriodic(){
    //State (row 1 right)
    m_shuff.PutNumber("cur pos", m_pos, {1,1,4,0});
    switch(m_targ){
        case EXTENDED:
            m_shuff.PutString("targ state", "Extended", {2,1,5,0});
            break;
        case STOWED:
            m_shuff.PutString("targ state", "Stowed", {2,1,5,0});
            break;
        case CLIMB:
            m_shuff.PutString("targ state", "Climb", {2,1,5,0});
            break;
    }
    switch(m_state){
        case MOVING:
            m_shuff.PutString("state", "Moving", {2,1,6,0});
            break;
        case AT_TARGET:
            m_shuff.PutString("state", "At target", {2,1,6,0});
            break;
        case MANUAL:
            m_shuff.PutString("state", "Manual", {2,1,6,0});
            break;
    }

    m_shuff.addButton("Zero", [&]{Zero();}, {1,1,7,3});
    m_shuff.update(true);
}

