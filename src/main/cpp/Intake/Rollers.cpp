#include "Intake/Rollers.h"

#include "Util/Utils.h"

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
    }

    m_rollerMotor.SetVoltage(units::volt_t{setVolts});
}


void Rollers::SetVoltage(){
    m_voltReq = std::clamp(m_voltReq, -MAX_VOLTS, MAX_VOLTS);
    m_rollerMotor.SetVoltage(units::volt_t(m_voltReq));
}

void Rollers::CoreShuffleboardInit(){
    m_shuff.add("Voltage", &m_voltReq, true);
}

void Rollers::CoreShuffleboardPeriodic(){
    m_shuff.update(true);
}