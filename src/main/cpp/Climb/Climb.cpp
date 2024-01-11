#include "Climb/Climb.h"

//maybe PID ff or some other fancier motor control

void Climb::CorePeriodic(){
    UpdatePos();
    double voltage = 0;
    if (m_state == MOVING){
        double targPos;
        switch (m_targ){
            case STOWED:
                voltage = -MAX_VOLTS;
                targPos = MIN_POS;
                break;
            case EXTENDED:
                voltage = MAX_VOLTS;
                targPos = MAX_POS;
                break;
            case CLIMB:
                voltage = CLIMB_VOLTS;
                targPos = MIN_POS;
                break;
        }
        if (AtTarget(targPos))
            m_state = AT_TARGET;
    }
    else if (m_state == AT_TARGET && m_targ == CLIMB)
        voltage = HOLD_VOLTS;
    m_master.SetVoltage(units::volt_t(voltage));
}

void Climb::UpdatePos(){
    // m_pos = m_master.GetPosition().GetValueAsDouble();
    m_pos = m_absEncoder.GetAbsolutePosition();
}

bool Climb::AtTarget(double target){
    if (std::abs(m_pos-target) <= POS_TOLERANCE)
        return true;
    else return false;
}

Climb::Climb(){
    m_slave.SetControl(Follower(Ids::MASTER_CLIMB_MOTOR, false));
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

void Climb::CoreShuffleboardPeriodic(){
    m_shuff.PutNumber("Cur pos", m_pos);
    m_shuff.PutNumber("targ", m_targ);
    m_shuff.PutNumber("state", m_state);
    m_shuff.add("Extended pos", &MAX_POS, true);
    m_shuff.add("Stowed pos", &MIN_POS, true);
    m_shuff.add("Extend stow volts", &MAX_VOLTS, true);
    m_shuff.add("Climb volts", &CLIMB_VOLTS, true);
    m_shuff.add("Hold volts", &HOLD_VOLTS, true);
}