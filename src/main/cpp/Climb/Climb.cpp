#include "Climb/Climb.h"

void Climb::CorePeriodic(){
    UpdatePos();
}

void Climb::CoreTeleopPeriodic(){
    StateInfo info;
    switch (m_targ){
        case STOWED:
            info = STOW_INFO;
            break;
        case EXTENDED:
            info = EXTENDED_INFO;
            break;
        case CLIMB:
            info = CLIMB_INFO;
            break;
    }

    double voltage = 0.0;

    switch(m_state){
        case MOVING:
            voltage = info.MOVE_VOLTS;
            if (AtTarget(info.TARG_POS)){
                m_state = AT_TARGET;
            }
            m_actuator.Set(OFF); //brake off
            break;
        case AT_TARGET:
            m_actuator.Set(BREAK); //break on
            break;
    }

    voltage = std::clamp(voltage, -MAX_VOLTS, MAX_VOLTS);
 
    m_master.SetVoltage(units::volt_t(voltage));
}

void Climb::UpdatePos(){
    m_pos = m_absEncoder.GetPosition().GetValueAsDouble();
}

bool Climb::AtTarget(double target){
    // if (std::abs(m_pos-target) <= POS_TOLERANCE)
    if (m_pos >= target-POS_TOLERANCE)
        return true;
    return false;
}

Climb::Climb(){
    m_master.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
    // m_slave.SetControl(Follower(Ids::MASTER_CLIMB_MOTOR, false));
}

void Climb::Extend(){
    SetTarget(EXTENDED);
}

void Climb::Stow(){
    SetTarget(STOWED);
}

void Climb::PullUp(){
    SetTarget(CLIMB);
}

Climb::State Climb::GetState(){
    return m_state;
}

void Climb::SetTarget(Target t){
    if (t = m_targ) return;
    m_state = MOVING;
    m_targ = t;
}

// should put break on here also 
void Climb::ManualPeriodic(double voltage){
    voltage = std::clamp(voltage, -MAX_VOLTS, MAX_VOLTS);
    m_master.SetVoltage(units::volt_t(voltage));
}

void Climb::Zero(){
    m_absEncoder.SetPosition(units::angle::turn_t(0));
}

void Climb::CoreShuffleboardInit(){
    //settings (row 0)
    m_shuff.add("max pos", &MAX_POS, {1,1,0,0}, true);
    m_shuff.add("min pos", &MIN_POS, {1,1,1,0}, true);
    m_shuff.add("max volts", &MAX_VOLTS, {1,1,2,0}, true);
    m_shuff.add("tolerance", &POS_TOLERANCE, {1,1,3,0}, true);

    //Climb (row 1)
    m_shuff.add("climb pos", &CLIMB_INFO.TARG_POS, {1,1,0,1}, true);
    m_shuff.add("climb volts", &CLIMB_INFO.MOVE_VOLTS, {1,1,1,1}, true);

    //Stow (row 2)
    m_shuff.add("stow pos", &STOW_INFO.TARG_POS, {1,1,0,2}, true);
    m_shuff.add("stow volts", &STOW_INFO.MOVE_VOLTS, {1,1,1,2}, true);

    //Extend (row 3)
    m_shuff.add("extend pos", &EXTENDED_INFO.TARG_POS, {1,1,0,3}, true);
    m_shuff.add("extend volts", &EXTENDED_INFO.MOVE_VOLTS, {1,1,1,3}, true);
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
    }

    m_shuff.addButton("Zero", [&]{Zero();}, {1,1,7,3});
    m_shuff.update(true);
}