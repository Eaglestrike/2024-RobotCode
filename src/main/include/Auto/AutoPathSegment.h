#pragma once

#include <frc/controller/PIDController.h>

#include "Drive/SwerveControl.h"
#include "Util/hermite.hpp"
#include "Util/Odometry.h"

namespace hm = hermite;

class AutoPathSegment {
public: 
  AutoPathSegment(SwerveControl &swerve, Odometry &odom);

  void Start();
  void Periodic();
  void Stop();

  void SetAutoPath(const std::string path);
  void SetDrivePID(double kP, double kI, double kD);
  void SetAngPID(double kP, double kI, double kD);

  double GetProgress() const;
  bool IsDone() const;
private:
  double GetAbsProgress() const;

  hm::Hermite<2> m_posSpline;
  hm::Hermite<1> m_angSpline;

  SwerveControl &m_swerve;
  Odometry &m_odom;
  double m_startTime;
  bool m_hasStarted;

  frc::PIDController m_posCorrect;
  frc::PIDController m_angCorrect;
};