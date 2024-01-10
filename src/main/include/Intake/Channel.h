#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include "Util/Mechanism.h"
#include "Constants/MechanismConstants.h"

using ctre::phoenix6::hardware::TalonFX;

class Channel : public Mechanism{
    public:
        enum ChannelState {
            ON,
            STOP,
            RETAIN,
        };

        Channel() = default;
        void SetState(ChannelState c);
        ChannelState Channel::GetState();
        void CoreTeleopPeriodic();

private:
  ChannelState m_state{STOP};
  TalonFX m_channelMotor{Ids::CHANNEL_MOTOR};

  //Constants
//   double MAX_VOLTS = 0.0;
  double KEEP_VOLTS = 0.0;
  double IN_VOLTS = 0.0;
};