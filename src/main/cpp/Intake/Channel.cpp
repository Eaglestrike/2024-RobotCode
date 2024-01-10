#include "Intake/Channel.h"

#include "Util/Utils.h"

void Channel::SetState(ChannelState c) {
    m_state = c;
}

Channel::ChannelState Channel::GetState() {
    return m_state;
}

void Channel::CoreTeleopPeriodic() {
    double setVolts = 0;

    switch (m_state) {
        case ON:
            setVolts = IN_VOLTS;
            break;
        case RETAIN:
            setVolts = KEEP_VOLTS;
            break;
        case STOP:
            setVolts = 0;
            break;
    }

    m_channelMotor.SetVoltage(units::volt_t{setVolts});
}