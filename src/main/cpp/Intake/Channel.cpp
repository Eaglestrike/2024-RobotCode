#include "Intake/Channel.h"

#include "Util/Utils.h"

void Channel::SetState(ChannelState c) {
    m_state = c;
}

Channel::ChannelState Channel::GetState() {
    return m_state;
}

Channel::Channel(bool enabled, bool dbg){
    Mechanism("Channel", enabled, dbg);
    // m_channelMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
}

void Channel::CoreTeleopPeriodic() {
    double kickerV = 0;
    double channelV = 0;

    switch (m_state) {
        case ON:
            kickerV = m_kickerInfo.IN_VOLTS;
            channelV = m_channelInfo.IN_VOLTS;
            break;
        case RETAIN:
            kickerV = m_kickerInfo.KEEP_VOLTS;
            channelV = m_channelInfo.KEEP_VOLTS;
            break;
    }

    m_kickerMotor.SetVoltage(units::volt_t{std::clamp(kickerV, -m_kickerInfo.MAX_VOLTS, m_kickerInfo.MAX_VOLTS)});
    m_channelMotor.SetVoltage(units::volt_t{std::clamp(channelV, -m_channelInfo.MAX_VOLTS, m_channelInfo.MAX_VOLTS)});
}

void Channel::SetVoltage(){
    m_cVoltReq = std::clamp(m_cVoltReq, -m_channelInfo.MAX_VOLTS, m_channelInfo.MAX_VOLTS);
    m_channelMotor.SetVoltage(units::volt_t(m_cVoltReq));

    m_kVoltReq = std::clamp(m_kVoltReq, -m_kickerInfo.MAX_VOLTS, m_kickerInfo.MAX_VOLTS);
    m_kickerMotor.SetVoltage(units::volt_t(m_kVoltReq));
}

void Channel::CoreShuffleboardInit(){
    m_shuff.add("kicker voltage", &m_kVoltReq, true);
    m_shuff.add("channel voltage", &m_cVoltReq, true);
}

void Channel::CoreShuffleboardPeriodic(){
    m_shuff.update(true);
}