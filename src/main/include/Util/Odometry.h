#pragma once

#include <map>
#include <utility>

#include <frc/smartdashboard/Field2d.h>

#include "Util/PoseEstimator.h"
#include "Util/simplevectors.hpp"

namespace vec = svector;

/**
 * Odometry class for localization
*/
class Odometry {
public:
  struct XYAng {
    vec::Vector2D pos;
    double ang; // absolute, can wrap around (less than -pi or greater than pi)
  };

  Odometry(const bool &shuffleboard);

  void SetStartingConfig(const vec::Vector2D &pos, const double &ang, const double &joystickAng);
  void SetAuto(const bool &autoMode);
  void Reset();

  vec::Vector2D GetPos() const;
  vec::Vector2D GetCamPos() const;
  double GetAng() const;
  double GetYaw() const;
  vec::Vector2D GetVel() const;
  vec::Vector2D GetDBVel() const;
  double GetAngVel() const;
  double GetAngNorm() const;
  double GetJoystickAng() const;
  vec::Vector2D GetStartPos() const;
  double GetStartAng() const;
  bool GetTagDetected() const;

  void UpdateEncoder(const vec::Vector2D &vel, const double &angNavXAbs, const double &navXYaw, const double &swerveAngVel);
  void UpdateCams(const vec::Vector2D &relPos, const int &tagId, const long long &uniqueId, const long long &age);

  void ShuffleboardInit();
  void ShuffleboardPeriodic();

private:
  void Update(const double &deltaT, const double &prevAng);
  std::pair<bool, double> GetInterpolAng(const double &camTime);

  bool m_shuffleboard;
  bool m_isAuto;

  vec::Vector2D m_curPos, m_startPos, m_vel;
  double m_curAng, m_startAng, m_angVel;
  double m_curYaw;
  double m_joystickAng;

  vec::Vector2D m_prevVel;
  double m_prevDriveTime;

  PoseEstimator m_estimator;
  long long m_uniqueId;
  double m_timeOffset;
  double m_prevCamTime;
  double m_camStdDevCoef;
  double m_turnStdDevCoef;

  std::map<double, double> m_angHistory;

  bool m_trustCamsMore;

  // debug stuff
  vec::Vector2D m_camPos; // only used for shuffleboard prints for now
  // frc::Field2d m_field;
};