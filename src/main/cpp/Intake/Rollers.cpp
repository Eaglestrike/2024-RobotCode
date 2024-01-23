#include "Intake/Rollers.h"

#include "Util/Utils.h"

#include <frc/smartdashboard/SmartDashboard.h>

Rollers::Rollers(bool enabled, bool shuffleboard)
    : Mechanism{"Rollers", enabled, shuffleboard},
    m_shuff{"Rollers", shuffleboard}{
    m_rollerMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
}

void Rollers::SetState(RollerState r) {
    m_state = r;
}

Rollers::RollerState Rollers::GetState() {
    return m_state;
}

void Rollers::CoreTeleopPeriodic() {
    double setVolts = 0;

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

    m_rollerMotor.SetVoltage(units::volt_t{setVolts});
}


void Rollers::SetVoltage(){
    m_voltReq = std::clamp(m_voltReq, -MAX_VOLTS, MAX_VOLTS);
    m_state = SET_VOLTAGE;
}

void Rollers::StopRollers() {
    m_rollerMotor.SetVoltage(units::volt_t(0));

}

void Rollers::CoreShuffleboardInit(){
    m_shuff.add("Voltage", &m_voltReq, true);
    m_shuff.addButton("Set Voltage", [&]{SetVoltage();}, {2,1});
}

void Rollers::CoreShuffleboardPeriodic(){
    m_shuff.update(true);
}