#pragma once

#include <array>
#include <frc/controller/PIDController.h>
#include <functional>

#include "Drive/SwerveModule.h"
#include "Util/simplevectors.hpp"

#include "Util/Mechanism.h"

namespace vec = svector; //!< Alias to vector namespace

/**
 * Swerve controller for 4-wheel swerve
*/
class SwerveControl : public Mechanism{
public:
  template <typename T>
  using RefArray = std::array<std::reference_wrapper<T>, 4>; //!< Alias to an array of references

  SwerveControl(RefArray<SwerveModule> modules, bool enabled = true, bool shuffleboard = false);

  vec::Vector2D GetRobotVelocity(double ang);

  void ResetAngleCorrection(double startAng = 0);
  void ResetFeedForward();
  void SetFeedForward(double kS, double kV, double kA);
  void SetAngleCorrectionPID(double kP, double kI, double kD);
  void SetRobotVelocity(vec::Vector2D vel, double angVel, double ang, double time);
  void SetRobotVelocityTele(vec::Vector2D vel, double angVel, double ang, double time, double angOfJoystick);
  void SetAngCorrection(bool angCorrection);
  void Lock();


private:
  void CoreInit() override;
  void CorePeriodic() override;

  void CoreShuffleboardInit() override;
  void CoreShuffleboardUpdate() override;

  RefArray<SwerveModule> m_modules;
  std::array<double, 4> m_prevSpeeds;

  double m_kS;
  double m_kV;
  double m_kA;

  double m_curAngle;
  bool m_angCorrection;
  frc2::PIDController m_angleCorrector;
};
