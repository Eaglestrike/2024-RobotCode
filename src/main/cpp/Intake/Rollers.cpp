#include "Intake/Rollers.h"

#include <algorithm>

#include "Util/Utils.h"

#include <frc/smartdashboard/SmartDashboard.h>

Rollers::Rollers(bool enabled, bool shuffleboard)
    : Mechanism{"Rollers", enabled, shuffleboard},
    m_shuff{"Rollers", shuffleboard}{
    
    m_rollerMotorBack.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
    m_rollerMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
}

void Rollers::SetState(RollerState r) {
    m_state = r;
}

void Rollers::SetStateBuffer(RollerState r, double offset_s) {
    if (offset_s == 0) return;
    m_wait = offset_s;
    m_timer = 0;
    m_nxtState = r;
}

Rollers::RollerState Rollers::GetState() {
    return m_state;
}

void Rollers::CoreTeleopPeriodic() {
    double setVolts = 0, v2 = 0;

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
            // v2 = BACK_ROLLER_IN_VOLTS;
            break;
        case PASS:
            setVolts = PASS_VOLTS;
            v2 = BACK_ROLLER_IN_VOLTS;
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
    if (v2 == 0) v2 = setVolts;

    SetRollerVolts(setVolts, v2);
}


void Rollers::SetVoltage(){
    m_voltReq = std::clamp(m_voltReq, -MAX_VOLTS, MAX_VOLTS);
    m_state = SET_VOLTAGE;
}

void Rollers::StopRollers() {
    SetRollerVolts(0, 0);
}

void Rollers::CoreShuffleboardInit(){
    m_shuff.add("Voltage", &m_voltReq, true);
    m_shuff.addButton("Set Voltage", [&]{SetVoltage();}, {2,1});
    m_shuff.add("In volts ", &IN_VOLTS, {1,1,0,4}, true);
    m_shuff.add("pass volts ", &PASS_VOLTS, {1,1,0,5}, true);
    m_shuff.add("back in volts ", &BACK_ROLLER_IN_VOLTS, {1,1,0,5}, true);
    m_shuff.add("out volts", &OUT_VOLTS, {1,1,1,4}, true);
    m_shuff.add("keep volts", &KEEP_VOLTS, {1,1,2,4}, true);
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
void Rollers::SetRollerVolts(double volts, double v2) {
    volts = std::clamp(volts, -MAX_VOLTS, MAX_VOLTS);
    m_rollerMotor.SetVoltage(units::volt_t{volts});
    m_rollerMotorBack.SetVoltage(units::volt_t{-v2});
}