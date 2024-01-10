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
    /* if (beam is broken){
        m_state = RETAIN;
    }
    */

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