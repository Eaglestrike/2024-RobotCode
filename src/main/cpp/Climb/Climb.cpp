#include "Climb/Climb.h"

void Climb::CorePeriodic(){
    UpdatePos();
}

void Climb::CoreTeleopPeriodic(){
    StateInfo info;
    double targPos;
    switch (m_targ){
        case STOWED:
            info = STOW_INFO;
            targPos = MIN_POS;
            break;
        case EXTENDED:
            info = EXTENDED_INFO;
            targPos = MAX_POS;
            break;
        case CLIMB:
            info = CLIMB_INFO;
            targPos = MIN_POS;
            break;
    }

    double voltage = 0.0;
    if (m_state == AT_TARGET){
        voltage = info.RETAIN_VOLTS;
    } else if (m_state == MOVING){
        voltage = info.MOVE_VOLTS;
    }
    voltage = std::clamp(voltage, -MAX_VOLTS, MAX_VOLTS);

    if (AtTarget(targPos))
        m_state = AT_TARGET;
   
    m_master.SetVoltage(units::volt_t(voltage));
}

void Climb::UpdatePos(){
    m_pos = m_absEncoder.GetAbsolutePosition();
}

bool Climb::AtTarget(double target){
    if (std::abs(m_pos-target) <= POS_TOLERANCE)
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
    SetTarget(EXTENDED);
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

void Climb::ManualPeriodic(double voltage){
    voltage = std::clamp(voltage, -MAX_VOLTS, MAX_VOLTS);
    m_master.SetVoltage(units::volt_t(voltage));
}

void Climb::CoreShuffleboardPeriodic(){
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