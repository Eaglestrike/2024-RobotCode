#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include "Util/Mechanism.h"
#include "ShuffleboardSender/ShuffleboardSender.h"
#include "Constants/IntakeConstants.h"
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <rev/CANSparkMax.h>

using ctre::phoenix::motorcontrol::can::WPI_TalonSRX;

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
  WPI_TalonSRX m_channelMotor;
  rev::CANSparkMax m_kickerMotor;
  ShuffleboardSender m_shuff;

  //Constants
  struct motorV { 
    double MAX_VOLTS;
    double KEEP_VOLTS;
    double IN_VOLTS;
  };
    motorV m_channelInfo = {8.0,
                            0.0,
                            3.0};

    motorV m_kickerInfo =  {8.0,
                            0.0,
                            3.0};
    
  //for dbg
   double m_kVoltReq = 0.0;
   double m_cVoltReq = 0.0;
};