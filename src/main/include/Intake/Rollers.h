#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include "Util/Mechanism.h"
#include "Constants/MechanismConstants.h"
#include "ShuffleboardSender/ShuffleboardSender.h"

using ctre::phoenix6::hardware::TalonFX;

class Rollers : public Mechanism{
    public:
        enum RollerState {
            INTAKE,
            INTAKE_STRONG, //Hold and intake strong
            RETAIN,
            STOP,
            OUTTAKE
        };

        Rollers(bool enabled, bool dbg);
        void SetState(RollerState r);
        RollerState GetState();
        void CoreTeleopPeriodic() override;
        void CoreShuffleboardInit() override;
        void CoreShuffleboardPeriodic() override;

private:
  void SetVoltage();
  RollerState m_state{STOP};
  TalonFX m_rollerMotor{Ids::ROLLER_MOTOR};
  ShuffleboardSender m_shuff {"Rollers", false};

  //Constants
  double MAX_VOLTS = 0.0;
  double KEEP_VOLTS = 0.0;
  double IN_VOLTS = 0.0;
  double OUT_VOLTS =0.0;

  //for dbg
  double m_voltReq = 0.0;
};