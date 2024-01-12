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

void Channel::SetVoltage(){
    m_voltReq = std::clamp(m_voltReq, -MAX_VOLTS, MAX_VOLTS);
    m_channelMotor.SetVoltage(units::volt_t(m_voltReq));
}

void Channel::CoreShuffleboardInit(){
    m_shuff.add("Voltage", &m_voltReq, true);
}

void Channel::CoreShuffleboardPeriodic(){
    m_shuff.update(true);
}