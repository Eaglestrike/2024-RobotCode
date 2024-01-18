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

        Rollers(bool enabled, bool shuffleboard);
        void SetState(RollerState r);
        RollerState GetState();
        void CoreTeleopPeriodic() override;
        void CoreShuffleboardInit() override;
        void CoreShuffleboardPeriodic() override;

        void SetVoltage();
        void StopRollers();

private:
  RollerState m_state{STOP};
  TalonFX m_rollerMotor{Ids::ROLLER_MOTOR, "rio"};
  ShuffleboardSender m_shuff{"Rollers", false};

  //Constants
  double MAX_VOLTS = 12.0;
  double KEEP_VOLTS = 0.0;
  double IN_VOLTS = 3.0;
  double OUT_VOLTS =0.0;

  //for dbg
  double m_voltReq = 0.0;
};