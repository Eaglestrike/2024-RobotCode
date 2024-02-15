#include "Intake/Channel.h"

#include "Util/Utils.h"

void Channel::SetState(ChannelState c) {
    m_state = c;
}

Channel::ChannelState Channel::GetState() {
    return m_state;
}

Channel::Channel(bool enabled, bool dbg): 
Mechanism("Channel", enabled, dbg), 
m_shuff{"Channel", dbg},
m_channelMotor{IntakeConstants::CHANNEL_MOTOR, rev::CANSparkLowLevel::MotorType::kBrushless}, 
m_kickerMotor{IntakeConstants::KICKER_MOTOR, rev::CANSparkLowLevel::MotorType::kBrushless}{
    m_kickerMotor.RestoreFactoryDefaults();
    m_kickerMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
    m_kickerMotor.SetInverted(true);
    m_channelMotor.RestoreFactoryDefaults();
    m_channelMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    m_channelMotor.SetInverted(true);
    // m_channelMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
}

void Channel::CoreTeleopPeriodic() {
    double kickerV = 0;
    double channelV = 0;

    switch (m_state) {
        case IN:
            kickerV = m_kickerInfo.IN_VOLTS;
            channelV = m_channelInfo.IN_VOLTS;
            break;
        case TO_SHOOT:
            kickerV = m_kickerInfo.PASS_VOLTS;
            channelV = m_channelInfo.PASS_VOLTS;
            break;
        case SPLIT:
            kickerV = m_kickerInfo.OUT_VOLTS;
            channelV = m_channelInfo.IN_VOLTS;
            break;
        case RETAIN:
            kickerV = m_kickerInfo.KEEP_VOLTS;
            channelV = m_channelInfo.KEEP_VOLTS;
            break;
         case OUT:
            kickerV = -m_kickerInfo.IN_VOLTS;
            channelV = -m_channelInfo.IN_VOLTS;
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
    m_shuff.add("kicker voltage", &m_kVoltReq, {1,1,0,1},true );
    m_shuff.add("channel voltage", &m_cVoltReq,  {1,1,0,1}, true);
    m_shuff.addButton("Set Voltage", [&]{SetVoltage();}, {2,1,0,0});

     m_shuff.add("kicker max", &m_kickerInfo.MAX_VOLTS, {1,1,0,2}, true);
     m_shuff.add("kicker keep", &m_kickerInfo.KEEP_VOLTS,{1,1,1,2}, true);
     m_shuff.add("kicker in", &m_kickerInfo.IN_VOLTS,{1,1,2,2}, true);
     m_shuff.add("kicker out", &m_kickerInfo.OUT_VOLTS,{1,1,3,2}, true);
       m_shuff.add("kicker SHOOT", &m_kickerInfo.PASS_VOLTS,{1,1,3,3}, true);

     m_shuff.add("channel max", &m_channelInfo.MAX_VOLTS, {1,1,0,3}, true);
     m_shuff.add("channel keep", &m_channelInfo.KEEP_VOLTS, {1,1,1,3},true);
     m_shuff.add("channel in", &m_channelInfo.IN_VOLTS, {1,1,2,3},true);
     m_shuff.add("channel SHOOT", &m_channelInfo.PASS_VOLTS, {1,1,3,4},true);

}

void Channel::CoreShuffleboardPeriodic(){

    switch(m_state){
        case STOP:
            m_shuff.PutString("State", "stop");
            break;
        case IN:
            m_shuff.PutString("State", "In");
            break;
        case RETAIN:
            m_shuff.PutString("State", "retain");
            break;
        case TO_SHOOT:
            m_shuff.PutString("State", "to shoot");
            break;
        case SPLIT:
            m_shuff.PutString("State", "split");
            break;
         case OUT:
            m_shuff.PutString("State", "out");
            break;
    }
    m_shuff.update(true);
}