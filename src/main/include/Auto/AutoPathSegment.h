#pragma once

#include <frc/controller/PIDController.h>

#include "Drive/SwerveControl.h"
#include "Util/hermite.hpp"
#include "Util/Odometry.h"

namespace hm = hermite;

/**
 * Auto Path segment for drivebase
*/
class AutoPathSegment {
public: 
  AutoPathSegment(bool shuffleboard, SwerveControl &swerve, Odometry &odom);

  void ShuffleboardInit();
  void ShuffleboardPeriodic();

  void Start();
  void Periodic();
  void Stop();

  void LoadAutoPath(const std::string path);
  void SetAutoPath(const std::string path);
  void SetDrivePID(double kP, double kI, double kD);
  void SetAngPID(double kP, double kI, double kD);
  void SetDriveTol(double tol);
  void SetAngTol(double tol);

  double GetProgress() const;
  bool IsDoneHermite() const;
  bool AtPosTarget() const;
  bool AtAngTarget() const;
  bool AtTarget() const;
private:
  double GetAbsProgress() const;

  bool m_shuffleboard;

  struct SwerveSpline{
    hm::Hermite<2> pos;
    hm::Hermite<1> ang;
  } m_spline;

  std::map<std::string, SwerveSpline> m_loadedSplines;

  SwerveControl &m_swerve;
  Odometry &m_odom;
  double m_startTime;
  bool m_hasStarted;

  frc::PIDController m_posCorrect;
  frc::PIDController m_angCorrect;
};