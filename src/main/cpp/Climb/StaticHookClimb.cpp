#include "Climb/StaticHookClimb.h"

void StaticHookClimb::CorePeriodic(){
    UpdatePos();
}

void StaticHookClimb::CoreTeleopPeriodic(){
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
                m_state = WAITING;
            }
            break;
        case WAITING:
            voltage = info.RETAIN_VOLTS;
            break;
        case DONE:
            break;
    }

    voltage = std::clamp(voltage, -MAX_VOLTS, MAX_VOLTS);
 
    m_master.SetVoltage(units::volt_t(voltage));
}

void StaticHookClimb::UpdatePos(){
    m_pos = m_absEncoder.GetAbsolutePosition();
}

bool StaticHookClimb::AtTarget(double target){
    if (std::abs(m_pos-target) <= POS_TOLERANCE)
        return true;
    return false;
}

StaticHookClimb::StaticHookClimb(){
    m_master.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
    // m_slave.SetControl(Follower(Ids::MASTER_CLIMB_MOTOR, false));
}

void StaticHookClimb::Extend(){
    SetTarget(EXTENDED);
}

void StaticHookClimb::Stow(){
    SetTarget(STOWED);
}

void StaticHookClimb::PullUp(){
    SetTarget(CLIMB);
}

StaticHookClimb::State StaticHookClimb::GetState(){
    return m_state;
}

void StaticHookClimb::SetTarget(Target t){
    if (t = m_targ) return;
    m_state = MOVING;
    m_targ = t;
}

void StaticHookClimb::ManualPeriodic(double voltage){
    voltage = std::clamp(voltage, -MAX_VOLTS, MAX_VOLTS);
    m_master.SetVoltage(units::volt_t(voltage));
}

void StaticHookClimb::CoreShuffleboardPeriodic(){
    m_shuff.PutNumber("cur pos", m_pos);
    m_shuff.PutNumber("targ state", m_targ);
    m_shuff.PutNumber("state", m_state);

    m_shuff.add("max pos", &MAX_POS, true);
    m_shuff.add("min pos", &MIN_POS, true);
    m_shuff.add("max volts", &MAX_VOLTS, true);
    m_shuff.add("tolerance", &POS_TOLERANCE, true);

    m_shuff.add("climb pos", &CLIMB_INFO.TARG_POS, true);
    m_shuff.add("climb volts", &CLIMB_INFO.MOVE_VOLTS, true);
    m_shuff.add("hang volts", &CLIMB_INFO.RETAIN_VOLTS, true);

    m_shuff.add("stow pos", &STOW_INFO.TARG_POS, true);
    m_shuff.add("stow volts", &STOW_INFO.MOVE_VOLTS, true);
    m_shuff.add("retain volts", &STOW_INFO.RETAIN_VOLTS, true);

    m_shuff.add("extend pos", &EXTENDED_INFO.TARG_POS, true);
    m_shuff.add("extend volts", &EXTENDED_INFO.MOVE_VOLTS, true);
    m_shuff.add("keep extended volts", &EXTENDED_INFO.RETAIN_VOLTS, true);
}