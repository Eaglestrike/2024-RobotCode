#include "Intake/Rollers.h"

#include <algorithm>

#include "Util/Utils.h"

#include <frc/smartdashboard/SmartDashboard.h>

Rollers::Rollers(bool enabled, bool shuffleboard)
    : Mechanism{"Rollers", enabled, shuffleboard},
    m_shuff{"Rollers", shuffleboard}{
    
    m_rollerMotorBack.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
    m_rollerMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
}

void Rollers::SetState(RollerState r) {
    m_state = r;
}

void Rollers::SetStateBuffer(RollerState r, double offset_s) {
    m_wait = offset_s;
    m_timer = 0;
    m_nxtState = r;
}

Rollers::RollerState Rollers::GetState() {
    return m_state;
}

void Rollers::CoreTeleopPeriodic() {
    double setVolts = 0;

    if (m_wait != -1){
        m_timer += 0.02;
        if (m_timer >= m_wait){
            m_wait = -1;
            m_state = m_nxtState;
        }
    }

    switch (m_state) {
        case INTAKE:
            setVolts = IN_VOLTS;
            break;
        case INTAKE_STRONG:
            setVolts = MAX_VOLTS;
            break;
        case RETAIN:
            setVolts = KEEP_VOLTS;
            break;
        case OUTTAKE:
            setVolts = OUT_VOLTS;
            break;
        case STOP:
            setVolts = 0;
            break;
        case SET_VOLTAGE:
            setVolts = m_voltReq;
            break;
    }

    SetRollerVolts(setVolts);
}


void Rollers::SetVoltage(){
    m_voltReq = std::clamp(m_voltReq, -MAX_VOLTS, MAX_VOLTS);
    m_state = SET_VOLTAGE;
}

void Rollers::StopRollers() {
    SetRollerVolts(0);

}

void Rollers::CoreShuffleboardInit(){
    m_shuff.add("Voltage", &m_voltReq, true);
    m_shuff.addButton("Set Voltage", [&]{SetVoltage();}, {2,1});
}

void Rollers::CoreShuffleboardPeriodic(){
switch(m_state){
        case RollerState::INTAKE:
            m_shuff.PutString("state", "Intake");
            break;
        case RollerState::RETAIN:
            m_shuff.PutString("state", "Retain");
            break;
        case RollerState::STOP:
            m_shuff.PutString("state", "Stppped");
            break;
        case RollerState::OUTTAKE:
            m_shuff.PutString("state", "Outtake");
            break;
    }

     switch(m_nxtState){
        case RollerState::INTAKE:
            m_shuff.PutString("nxt state", "Intake");
            break;
        case RollerState::RETAIN:
            m_shuff.PutString("nxt state", "Retain");
            break;
        case RollerState::STOP:
            m_shuff.PutString("nxt state", "Stppped");
            break;
        case RollerState::OUTTAKE:
            m_shuff.PutString("nxt state", "Outtake");
            break;
    }

    m_shuff.PutNumber("timer", m_timer);
    m_shuff.PutNumber("wait", m_wait);
    m_shuff.update(true);
}

/**
 * Sets roller volts for both motors because FX and sRX differnt versions cannot do leader/follower
*/
void Rollers::SetRollerVolts(double volts) {
    volts = std::clamp(volts, -12.0, 12.0);
    m_rollerMotor.SetVoltage(units::volt_t{volts});
    m_rollerMotorBack.SetVoltage(units::volt_t{volts});
}