#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/controller/PIDController.h>

#include "Constants/SwerveConstants.h"
#include "Util/Mechanism.h"
#include "Util/simplevectors.hpp"

namespace vec = svector; //!< Alias to vector namespace
using ctre::phoenix6::hardware::CANcoder;
using ctre::phoenix6::hardware::TalonFX;

/**
 * Interface with an individual swerve module
*/
class SwerveModule : public Mechanism{
public:
  SwerveModule(SwerveConstants::SwerveConfig config, bool enabled = true, bool shuffleboard = false);

  double GetCorrectedEncoderReading();
  double GetRawEncoderReading();
  vec::Vector2D GetVelocity();
  vec::Vector2D getPosition();

  void SetVector(vec::Vector2D vec);
  void SetSpeed(double speed);
  void SetAngle(double angle);
  void SetPID(double kP, double kI, double kD);
  void SetLock(bool lock);

private:
  void CorePeriodic() override;

  void CoreShuffleboardInit() override;
  void CoreShuffleboardPeriodic() override;
  void CoreShuffleboardUpdate() override;

  bool ShouldFlip(vec::Vector2D curAng, vec::Vector2D targetAng);

  
  TalonFX m_driveMotor;
  TalonFX m_angleMotor;
  CANcoder m_encoder;
  frc::PIDController m_controller;

  bool m_flipped;
  bool m_driveInverted; // if the drive motor is inverted (at angle = 0, positive voltage = negative movement)
  bool m_encoderInverted; // if positive encoder values = clockwise when robot is upright
  bool m_angMotorInverted; // if positive angle motor voltage = clockwise when robot is upright
  vec::Vector2D m_targetAngle;
  double m_targetSpeed;
  double m_offset;

  bool m_lock;

  vec::Vector2D m_position;
};
