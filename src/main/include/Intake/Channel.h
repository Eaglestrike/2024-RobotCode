#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include "Util/Mechanism.h"
#include "ShuffleboardSender/ShuffleboardSender.h"
#include "Constants/IntakeConstants.h"

using ctre::phoenix6::hardware::TalonFX;

class Channel : public Mechanism{
    public:
        enum ChannelState {
            ON,
            STOP,
            RETAIN,
        };

        Channel(bool enabled, bool dbg);
        void SetState(ChannelState c);
        ChannelState GetState();
        void CoreTeleopPeriodic() override;
        void CoreShuffleboardInit() override;
        void CoreShuffleboardPeriodic() override;

private:
  void SetVoltage();
  ChannelState m_state{STOP};
  TalonFX m_channelMotor{IntakeConstants::CHANNEL_MOTOR};
  ShuffleboardSender m_shuff {"Channel", false};

  //Constants
  double MAX_VOLTS = 0.0;
  double KEEP_VOLTS = 0.0;
  double IN_VOLTS = 0.0;

  //for dbg
   double m_voltReq = 0.0;
};