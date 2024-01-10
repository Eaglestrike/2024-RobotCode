#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include "Util/Mechanism.h"
#include "Constants/MechanismConstants.h"

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

        Rollers() = default;
        void SetState(RollerState r);
        RollerState Rollers::GetState();
        void CoreTeleopPeriodic();

private:
  RollerState m_state{STOP};
  TalonFX m_rollerMotor{Ids::ROLLER_MOTOR};

  //Constants
  double MAX_VOLTS = 0.0;
  double KEEP_VOLTS = 0.0;
  double IN_VOLTS = 0.0;
  double OUT_VOLTS =0.0;
};