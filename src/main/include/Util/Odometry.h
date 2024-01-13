#pragma once

#include "Util/PoseEstimator.h"
#include "Util/simplevectors.hpp"

namespace vec = svector;

class Odometry {
public:
  struct XYAng {
    vec::Vector2D pos;
    double ang; // absolute, can wrap around (less than -pi or greater than pi)
  };

  Odometry();

  void SetStartingConfig(const vec::Vector2D &pos, const double &ang, const double &joystickAng);
  void Reset();

  vec::Vector2D GetPos() const;
  double GetAng() const;
  double GetJoystickAng() const;

  vec::Vector2D GetStartPos() const;
  double GetStartAng() const;

  void UpdateEncoder(const vec::Vector2D &vel, const double &angNavXAbs);
  void UpdateCams(const vec::Vector2D &relPos, const int &tagId, const unsigned long long &uniqueId);

private:
  void Update(const double &deltaT, const double &prevAng);

  vec::Vector2D m_curPos, m_startPos, m_vel;
  double m_curAng, m_startAng, m_angVel;
  double m_joystickAng;

  double m_prevTime;

  PoseEstimator m_estimator;
};