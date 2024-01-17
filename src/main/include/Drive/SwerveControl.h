#pragma once

#include <array>
#include <frc/controller/PIDController.h>
#include <functional>

#include "Drive/SwerveModule.h"
#include "Util/simplevectors.hpp"

#include "Util/Mechanism.h"
#include "Util/Utils.h"

namespace vec = svector; //!< Alias to vector namespace

/**
 * Swerve controller for 4-wheel swerve
*/
class SwerveControl : public Mechanism{
public:
  SwerveControl(bool enabled = true, bool shuffleboard = false);

  vec::Vector2D GetRobotVelocity(double ang);

  void SetAutoMode(bool enabled);
  void SetFFConstants(double kS, double kV, double kA);
  void ResetFF();
  void ResetAngleCorrection(double startAng = 0);
  void SetAngleCorrectionPID(double kP, double kI, double kD);
  void SetRobotVelocity(vec::Vector2D vel, double angVel, double ang);
  void SetRobotVelocityTele(vec::Vector2D vel, double angVel, double ang, double angOfJoystick);
  void SetAngCorrection(bool angCorrection);
  void Lock();

private:
  void CoreInit() override;
  void CorePeriodic() override;

  void CoreShuffleboardInit() override;
  void CoreShuffleboardUpdate() override;

  void SetModuleVelocity(SwerveModule &module, double &prevSpeed, vec::Vector2D vel, double angVel, double ang);

  double m_kS, m_kV, m_kA;
  bool m_autoEnabled;
  bool m_angCorrectorInverted;

  // Swerve Modules
  SwerveModule m_fr, m_br, m_fl, m_bl;
  double m_pfr, m_pbr, m_pfl, m_pbl;

  double m_curAngle;
  bool m_angCorrection;
  frc::PIDController m_angleCorrector;
  double m_prevTime;
};
